#!/usr/bin/env python3
"""Rectified KITTI pixel + depth -> LiDAR -> fixed World ENU / UTM / ECEF.

A pixel alone defines a ray. Supply --depth-m (optical-axis depth, not range),
or use depth from the closest LiDAR projection within --max-pixel-distance.
With no --pixel, select five exact LiDAR projections as reproducible examples.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from pyproj.exceptions import ProjError
from scipy.spatial import cKDTree

import lidar_to_geographic as geo
import project_lidar_to_camera as camera


def backproject_pixels(pixels, depth, projection):
    """Invert the full P=[M|b], retaining camera translation and stereo baseline.

    depth is s in s*[u,v,1] = P*[x_lidar,y_lidar,z_lidar,1].
    """
    pixels = np.asarray(pixels, dtype=np.float64)
    depth = np.asarray(depth, dtype=np.float64).reshape(-1)
    projection = np.asarray(projection, dtype=np.float64)
    if (pixels.ndim != 2 or pixels.shape[1] != 2 or not len(pixels)
            or len(depth) != len(pixels) or not np.isfinite(pixels).all()
            or not np.isfinite(depth).all() or np.any(depth <= 0)):
        raise ValueError("Expected finite N x 2 pixels and N strictly positive optical depths")
    if projection.shape != (3, 4) or not np.isfinite(projection).all():
        raise ValueError("Projection must be a finite 3 x 4 matrix")
    homogeneous = np.column_stack((pixels, np.ones(len(pixels)))) * depth[:, None]
    return np.linalg.solve(projection[:, :3], (homogeneous - projection[:, 3]).T).T


def visible_lidar(scan, projection, image_size):
    """Keep closest positive-depth point per image pixel (sparse z-buffer).

    This resolves collisions among measured LiDAR samples only, not full camera
    occlusion; different sensor viewpoints can still expose different surfaces.
    """
    homogeneous = np.column_stack((scan[:, :3], np.ones(len(scan)))) @ projection.T
    indices = np.flatnonzero(np.isfinite(homogeneous).all(axis=1) & (homogeneous[:, 2] > 0))
    pixels = homogeneous[indices, :2] / homogeneous[indices, 2:]
    width, height = image_size
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < width)
              & (pixels[:, 1] >= 0) & (pixels[:, 1] < height))
    pixels, indices = pixels[inside], indices[inside]
    depth = homogeneous[indices, 2]
    order = np.argsort(depth, kind="stable")
    cell = np.floor(pixels).astype(np.int64)
    keys = cell[:, 1] * width + cell[:, 0]
    _, first = np.unique(keys[order], return_index=True)
    keep = order[first]
    return pixels[keep], depth[keep], indices[keep]


def lookup_lidar_depth(pixels, visible_pixels, visible_depth, visible_indices, max_distance):
    if not np.isfinite(max_distance) or max_distance < 0:
        raise ValueError("max-pixel-distance must be finite and nonnegative")
    if not len(visible_pixels):
        raise ValueError("No LiDAR depth samples inside the camera image")
    distance, nearest = cKDTree(visible_pixels).query(pixels)
    if np.any(distance > max_distance):
        failed = np.flatnonzero(distance > max_distance).tolist()
        raise ValueError(f"No LiDAR depth within {max_distance:g} px for query indices {failed}; "
                         "supply measured --depth-m or choose another pixel")
    return visible_depth[nearest], visible_indices[nearest], visible_pixels[nearest], distance


def convert_pixels(pixels, depths, projection, pose, calibration, reference, utm_epsg=None,
                   height_offset_m=0.0):
    xyz = backproject_pixels(pixels, depths, projection)
    coordinates, metadata = geo.georeference_points(xyz, pose, calibration, utm_epsg, height_offset_m)
    world = geo.ecef_to_world(coordinates["ecef_m"], reference)
    check = np.column_stack((xyz, np.ones(len(xyz)))) @ projection.T
    metadata["pixel_reprojection_max_error_px"] = float(np.linalg.norm(
        check[:, :2] / check[:, 2:] - pixels, axis=1).max())
    metadata["world_reference"] = reference
    metadata["numerical_checks"] = geo.numerical_checks(xyz, coordinates, metadata)
    return xyz, world, coordinates, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, default=geo.DEFAULT_SEQUENCE)
    parser.add_argument("--calib-dir", type=Path)
    parser.add_argument("--frame", type=int, default=6)
    parser.add_argument("--reference-frame", type=int, default=0)
    parser.add_argument("--camera", type=int, choices=range(4), default=2)
    parser.add_argument("--pixel", nargs=2, type=float, action="append", metavar=("U", "V"))
    parser.add_argument("--depth-m", nargs="+", type=float,
                        help="One positive optical depth per --pixel, in the same order")
    parser.add_argument("--max-pixel-distance", type=float, default=2.0)
    parser.add_argument("--utm-epsg", type=int)
    parser.add_argument("--height-offset-m", type=float, default=0.0)
    parser.add_argument("--output-dir", type=Path, default=geo.ROOT / "Results/CameraToWorld")
    args = parser.parse_args()
    try:
        if args.depth_m is not None and args.pixel is None:
            raise ValueError("--depth-m requires --pixel")
        if args.frame < 0 or args.reference_frame < 0:
            raise ValueError("frame IDs must be nonnegative")
        calibration_dir = args.calib_dir or args.sequence.parent
        projection, size = camera.load_projection(calibration_dir, args.camera)
        with Image.open(args.sequence / f"image_{args.camera:02d}/data/{args.frame:010d}.png") as source:
            photo = source.convert("RGB")
        if photo.size != size:
            raise ValueError("Use the original rectified image dimensions from calibration")
        pose = np.loadtxt(args.sequence / f"oxts/data/{args.frame:010d}.txt").reshape(-1)
        anchor_pose = np.loadtxt(args.sequence / f"oxts/data/{args.reference_frame:010d}.txt").reshape(-1)
        if len(pose) != 30 or len(anchor_pose) != 30 or not np.isfinite([pose, anchor_pose]).all():
            raise ValueError("Expected 30 finite OXTS values in both frames")
        reference = geo.make_world_reference(anchor_pose, args.height_offset_m)
        reference["frame_id"] = args.reference_frame
        epsg = args.utm_epsg or geo.automatic_utm_epsg(*anchor_pose[:2])
        scan = None
        if args.depth_m is None:
            scan, pose = geo.load_frame(args.sequence, args.frame)
            visible_pixels, visible_depth, visible_indices = visible_lidar(scan, projection, size)
            if not len(visible_pixels):
                raise ValueError("No visible LiDAR samples")
        if args.pixel is None:
            # Choose spread-out measured samples, then use their exact projected coordinates.
            targets = np.array([[0.2, 0.7], [0.35, 0.55], [0.5, 0.65], [0.65, 0.55], [0.8, 0.7]]) * size
            selected = np.unique(cKDTree(visible_pixels).query(targets)[1])
            pixels = visible_pixels[selected]
            source_mode = "demo_exact_lidar_projection"
        else:
            pixels = np.asarray(args.pixel, dtype=np.float64)
            source_mode = "provided_optical_depth" if args.depth_m is not None else "nearest_lidar_depth"
        if (not np.isfinite(pixels).all() or np.any(pixels < 0)
                or np.any(pixels >= np.asarray(size))):
            raise ValueError(f"Pixels must lie inside the original {size[0]} x {size[1]} image")
        if args.depth_m is not None:
            depths = np.asarray(args.depth_m)
            indices, matches, distances = [None] * len(pixels), [None] * len(pixels), [None] * len(pixels)
        else:
            depths, indices, matches, distances = lookup_lidar_depth(
                pixels, visible_pixels, visible_depth, visible_indices, args.max_pixel_distance)
        xyz, world, coordinates, metadata = convert_pixels(
            pixels, depths, projection, pose, geo.load_imu_to_lidar(calibration_dir / "calib_imu_to_velo.txt"),
            reference, epsg, args.height_offset_m)
        rows = []
        for i, (pixel, depth) in enumerate(zip(pixels, depths)):
            row = {"query_id": i + 1, "u_px": float(pixel[0]), "v_px": float(pixel[1]),
                   "depth_m": float(depth), "depth_source": source_mode,
                   "lidar_point_index": int(indices[i]) if indices[i] is not None else None,
                   "matched_u_px": float(matches[i][0]) if matches[i] is not None else None,
                   "matched_v_px": float(matches[i][1]) if matches[i] is not None else None,
                   "match_distance_px": float(distances[i]) if distances[i] is not None else None}
            for prefix, values in (("lidar", xyz[i]), ("world", world[i]), ("ecef", coordinates["ecef_m"][i])):
                row.update({f"{prefix}_{axis}_m": float(value) for axis, value in zip("xyz", values)})
            for name in ("utm_easting_m", "utm_northing_m", "height_assumed_ellipsoid_m", "latitude_deg", "longitude_deg"):
                row[name] = float(coordinates[name][i])
            rows.append(row)
        metadata.update({"sequence": str(args.sequence.resolve()), "frame_id": args.frame,
                         "camera": args.camera, "image_size": list(size), "depth_source": source_mode,
                         "calibration_dir": str(calibration_dir.resolve()),
                         "projection_lidar_to_image": projection.tolist(),
                         "depth_definition": "s = (P_total @ [x_lidar,y_lidar,z_lidar,1])[2], optical-axis meters, not range",
                         "max_pixel_distance": args.max_pixel_distance,
                         "timestamps": geo.read_timing(args.sequence, args.frame),
                         "rows": rows,
                         "limitations": "Sparse nearest-sample depth assumes the same surface; pixel-only 3D is not unique. "
                             "Z-buffer does not fully resolve camera/LiDAR occlusion. No scan deskew. "
                             "Exact-projection demo and reprojection errors measure numerical consistency, not physical accuracy."})
        timestamps = args.sequence / f"image_{args.camera:02d}/timestamps.txt"
        if timestamps.exists():
            camera_time = timestamps.read_text().splitlines()[args.frame].strip()
            metadata["timestamps"]["camera"] = camera_time
            if "lidar_forward" in metadata["timestamps"]:
                metadata["timestamps"]["camera_minus_lidar_ms"] = float((np.datetime64(camera_time, "ns")
                    - np.datetime64(metadata["timestamps"]["lidar_forward"], "ns")) / np.timedelta64(1, "ms"))
        if source_mode == "demo_exact_lidar_projection":
            metadata["demo_lidar_roundtrip_3d"] = geo.error_statistics(xyz - scan[indices, :3])
        args.output_dir.mkdir(parents=True, exist_ok=True)
        with (args.output_dir / "pixel_world.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        (args.output_dir / "pixel_world.json").write_text(
            json.dumps(metadata, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        draw = ImageDraw.Draw(photo)
        for row in rows:
            u, v = row["u_px"], row["v_px"]
            draw.ellipse((u - 6, v - 6, u + 6, v + 6), outline=(255, 220, 0), width=2)
            label = f"{row['query_id']}: {row['depth_m']:.2f}m"
            x, y = min(u + 9, photo.width - 105), max(0, v - 22)
            draw.rectangle((x - 2, y - 2, x + 100, y + 14), fill=(15, 24, 39))
            draw.text((x, y), label, fill=(255, 240, 140))
        photo.save(args.output_dir / "pixel_world.png")
    except (OSError, ValueError, KeyError, IndexError, np.linalg.LinAlgError, ProjError) as error:
        parser.error(str(error))
    print(f"Converted {len(rows)} pixels ({source_mode}) into World, EPSG:{metadata['utm_epsg']}, ECEF")
    print(f"Saved: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
