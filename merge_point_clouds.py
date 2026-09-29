#!/usr/bin/env python3
"""Merge KITTI frames using OXTS poses into fixed World ENU, UTM and ECEF.

No ICP, deskew or dynamic-object removal is applied. The optional voxel filter
keeps one original point per voxel, preserving frame IDs and scan point indices.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from pyproj.exceptions import ProjError
from scipy.spatial import cKDTree

import lidar_to_geographic as geo


def voxel_indices(world_xyz, size):
    if not np.isfinite(size) or size < 0:
        raise ValueError("voxel-size must be finite and nonnegative")
    if size == 0:
        return np.arange(len(world_xyz))
    cells = np.floor(np.asarray(world_xyz) / size)
    _, indices = np.unique(cells, axis=0, return_index=True)
    return np.sort(indices)


def overlap_statistics(first, second, threshold=1.0, samples=10000):
    """Symmetric sampled nearest-neighbor distances, before voxel filtering.

    Distances mix sampling, occlusion, pose and scene motion effects; they are
    NOT absolute positioning error. No correspondence or registration is fitted.
    """
    distances = []
    for source, target in ((first, second), (second, first)):
        selected = np.linspace(0, len(source) - 1, min(samples, len(source)), dtype=int)
        distances.append(cKDTree(target).query(source[selected], k=1)[0])
    distances = np.concatenate(distances)
    overlap = distances <= threshold
    return {
        "sample_count": len(distances), "threshold_m": threshold,
        "fraction_within_threshold": float(overlap.mean()),
        "median_m": float(np.median(distances)), "p95_m": float(np.percentile(distances, 95)),
        "all_samples_rmse_m": float(np.sqrt(np.mean(distances ** 2))),
        "within_threshold_rmse_m": (float(np.sqrt(np.mean(distances[overlap] ** 2)))
                                     if overlap.any() else None),
    }


def merge_frames(sequence, frames, calibration_dir, reference_frame=0,
                 utm_epsg=None, height_offset_m=0.0, voxel_size=0.2):
    frames = list(frames)
    if len(frames) < 2 or len(set(frames)) != len(frames) or min(frames) < 0:
        raise ValueError("Select at least two distinct nonnegative frame IDs")
    if frames != sorted(frames):
        raise ValueError("Frame IDs must be in increasing order")
    if not np.isfinite(voxel_size) or voxel_size < 0:
        raise ValueError("voxel-size must be finite and nonnegative")
    _, anchor_pose = geo.load_frame(sequence, reference_frame)
    reference = geo.make_world_reference(anchor_pose, height_offset_m)
    reference["frame_id"] = reference_frame
    fixed_epsg = geo.automatic_utm_epsg(*anchor_pose[:2]) if utm_epsg is None else utm_epsg
    calibration = geo.load_imu_to_lidar(calibration_dir / "calib_imu_to_velo.txt")
    blocks = {key: [] for key in ("world_xyz", "ecef_xyz", "utm_xyz", "intensity", "frame_id", "point_index")}
    summaries, comparisons, trajectory = [], [], []
    previous_world = None
    for frame in frames:
        scan, pose = geo.load_frame(sequence, frame)
        coordinates, metadata = geo.georeference_points(
            scan[:, :3], pose, calibration, fixed_epsg, height_offset_m)
        world = geo.ecef_to_world(coordinates["ecef_m"], reference)
        blocks["world_xyz"].append(world)
        blocks["ecef_xyz"].append(coordinates["ecef_m"])
        blocks["utm_xyz"].append(np.column_stack((coordinates["utm_easting_m"],
            coordinates["utm_northing_m"], coordinates["height_assumed_ellipsoid_m"])))
        blocks["intensity"].append(scan[:, 3])
        blocks["frame_id"].append(np.full(len(scan), frame, dtype=np.int32))
        blocks["point_index"].append(np.arange(len(scan), dtype=np.int32))
        anchor = geo.make_world_reference(pose, height_offset_m)
        trajectory.append(geo.ecef_to_world(np.array([anchor["origin_ecef_m"]]), reference)[0].tolist())
        summaries.append({"frame_id": frame, "point_count": len(scan),
                          "timestamps": geo.read_timing(sequence, frame),
                          "oxts_reported_position_accuracy_m": float(pose[23]),
                          "T_ecef_from_lidar": metadata["T_ecef_from_lidar"]})
        if previous_world is not None:
            comparisons.append({"frame_a": frames[len(summaries) - 2], "frame_b": frame,
                                **overlap_statistics(previous_world, world)})
        previous_world = world
        print(f"Frame {frame:010d}: {len(scan):,} points", flush=True)
    arrays = {key: np.concatenate(value) for key, value in blocks.items()}
    count = len(arrays["world_xyz"])
    keep = voxel_indices(arrays["world_xyz"], voxel_size)
    arrays = {key: value[keep] for key, value in arrays.items()}
    recovered = (arrays["world_xyz"] @ np.asarray(reference["R_ecef_from_world"]).T
                 + np.asarray(reference["origin_ecef_m"]))
    metadata = {
        "sequence": str(Path(sequence).resolve()), "frames": frames,
        "calibration_dir": str(calibration_dir.resolve()),
        "input_point_count": count, "output_point_count": len(keep),
        "utm_epsg": fixed_epsg, "ecef_epsg": 4978, "world_reference": reference,
        "height_offset_m": height_offset_m, "height_datum_verified": False,
        "voxel_size_m": voxel_size, "voxel_method": "Keep first original point per fixed-World voxel",
        "trajectory_world_m": trajectory, "frame_summaries": summaries,
        "neighbor_frame_distances": comparisons,
        "world_ecef_roundtrip_3d": geo.error_statistics(recovered - arrays["ecef_xyz"]),
        "distance_interpretation": "Symmetric nearest-neighbor distances on up to 10000 points per direction, "
            "before voxel filtering; includes moving objects and nonoverlap, not ground-truth error",
        "limitations": "One OXTS pose per scan; no ICP, deskew or moving-object removal. "
            "OXTS altitude plus offset assumed ellipsoidal. All frames share one UTM EPSG and World origin.",
    }
    return arrays, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, default=geo.DEFAULT_SEQUENCE)
    parser.add_argument("--calib-dir", type=Path)
    parser.add_argument("--frames", type=int, nargs="+", help="Default: all scans in the sequence")
    parser.add_argument("--reference-frame", type=int, default=0)
    parser.add_argument("--utm-epsg", type=int)
    parser.add_argument("--height-offset-m", type=float, default=0.0)
    parser.add_argument("--voxel-size", type=float, default=0.2, help="Meters; 0 keeps every point")
    parser.add_argument("--output-dir", type=Path, default=geo.ROOT / "Results/MultiFrame")
    args = parser.parse_args()
    try:
        frames = args.frames if args.frames is not None else sorted(
            int(path.stem) for path in (args.sequence / "velodyne_points/data").glob("*.bin"))
        arrays, metadata = merge_frames(args.sequence, frames, args.calib_dir or args.sequence.parent,
            args.reference_frame, args.utm_epsg, args.height_offset_m, args.voxel_size)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.output_dir / "merged_points.npz", **arrays)
        descriptions = {"world": metadata["world_reference"]["axes"],
                        "utm": f"XY EPSG:{metadata['utm_epsg']}; Z assumed ellipsoidal height, meters",
                        "ecef": "ECEF XYZ EPSG:4978, meters; altitude datum assumed"}
        for system, description in descriptions.items():
            geo.write_xyz_pcd(args.output_dir / f"merged_{system}.pcd", arrays[f"{system}_xyz"],
                              arrays["intensity"], description)
        metadata["files"] = ["merged_points.npz", "merged_world.pcd", "merged_utm.pcd", "merged_ecef.pcd"]
        (args.output_dir / "merged.json").write_text(
            json.dumps(metadata, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (OSError, ValueError, KeyError, IndexError, np.linalg.LinAlgError, ProjError) as error:
        parser.error(str(error))
    print(f"Merged {len(frames)} frames: {metadata['input_point_count']:,} -> {metadata['output_point_count']:,} points")
    print(f"Saved: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
