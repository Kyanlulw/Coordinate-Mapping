#!/usr/bin/env python3
"""Check PCD/BIN identity and make camera / projected PCD / overlay comparisons.

Example:
  python check_lidar_alignment.py --frame 6 --camera 2 --pcd temp6.pcd \
      --roi 635 155 795 295

These checks validate the data and projection implementation. They do not measure
physical calibration accuracy without independently labeled correspondences.
"""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw, ImageFont

import project_lidar_to_camera as projector


def independent_projection(xyz, calibration_dir, camera):
    """Apply each coordinate transform separately, before perspective division."""
    velo = projector.read_calibration(calibration_dir / "calib_velo_to_cam.txt")
    cam = projector.read_calibration(calibration_dir / "calib_cam_to_cam.txt")
    camera0 = xyz @ velo["R"].reshape(3, 3).T + velo["T"]
    rectified0 = camera0 @ cam["R_rect_00"].reshape(3, 3).T
    p = cam[f"P_rect_{camera:02d}"].reshape(3, 4)
    homogeneous = rectified0 @ p[:, :3].T + p[:, 3]
    front = np.isfinite(homogeneous).all(axis=1) & (homogeneous[:, 2] > 0)
    pixels = homogeneous[front, :2] / homogeneous[front, 2:]
    depth = homogeneous[front, 2]
    width, height = cam[f"S_rect_{camera:02d}"]
    inside = (
        np.isfinite(pixels).all(axis=1)
        & (pixels[:, 0] >= 0) & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    )
    return pixels[inside], depth[inside]


def timestamp(sequence, relative_path, frame):
    value = (sequence / relative_path).read_text().splitlines()[frame].strip()
    return value, np.datetime64(value, "ns")


def font(size):
    try:
        return ImageFont.truetype("arial.ttf", size)
    except OSError:
        return ImageFont.load_default()


def comparison_row(image, pixels, depth, bounds, panel_width=600):
    x0, y0, x1, y1 = bounds
    scale = panel_width / (x1 - x0)
    panel_height = round((y1 - y0) * scale)
    photograph = image.crop(bounds).resize(
        (panel_width, panel_height), Image.Resampling.LANCZOS
    )
    keep = (
        (pixels[:, 0] >= x0) & (pixels[:, 0] < x1)
        & (pixels[:, 1] >= y0) & (pixels[:, 1] < y1)
    )
    locations = (pixels[keep] - [x0, y0]) * [scale, panel_height / (y1 - y0)]
    radius = 1 if scale > 1 else 0
    black = Image.new("RGB", photograph.size, (12, 16, 23))
    cloud = projector.draw_overlay(black, locations, depth[keep], radius, 1.0, 80.0)
    overlay = projector.draw_overlay(photograph, locations, depth[keep], radius, 0.9, 80.0)
    return photograph, cloud, overlay


def save_comparison(image, pixels, depth, rois, destination, frame, camera):
    bounds = [(0, 0, image.width, image.height)] + rois
    rows = [comparison_row(image, pixels, depth, box) for box in bounds]
    margin, gap, header, row_label, footer = 24, 18, 100, 36, 60
    width = 3 * 600 + 2 * margin + 2 * gap
    height = header + sum(row[0].height + row_label for row in rows) + footer
    sheet = Image.new("RGB", (width, height), (246, 248, 251))
    draw = ImageDraw.Draw(sheet)
    draw.text((margin, 16), f"Frame {frame:010d} | image_{camera:02d} | Same camera viewpoint",
              font=font(26), fill=(20, 30, 45))
    for column, label in enumerate(("Camera image", "PCD projected into camera", "Overlay (small points)")):
        draw.text((margin + column * (600 + gap), 62), label,
                  font=font(22), fill=(20, 30, 45))
    y = header
    for index, (row, box) in enumerate(zip(rows, bounds)):
        label = "Full scene" if index == 0 else f"Zoom: pixel bounds {box}"
        draw.text((margin, y + 6), label, font=font(18), fill=(60, 70, 85))
        y += row_label
        for column, panel in enumerate(row):
            sheet.paste(panel, (margin + column * (600 + gap), y))
        y += row[0].height
    draw.text((margin, y + 16),
              "Color = camera depth: red (near) -> yellow -> green -> cyan -> blue (80 m). "
              "Visual comparison is not a calibration error measurement.",
              font=font(18), fill=(40, 50, 65))
    sheet.save(destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, default=projector.DEFAULT_SEQUENCE)
    parser.add_argument("--frame", type=int, default=6)
    parser.add_argument("--camera", type=int, choices=range(4), default=2)
    parser.add_argument("--pcd", type=Path, default=projector.ROOT / "temp6.pcd")
    parser.add_argument("--roi", type=int, nargs=4, action="append", default=[],
                        metavar=("X0", "Y0", "X1", "Y1"), help="Crop bounds; can be repeated")
    parser.add_argument("--output-dir", type=Path, default=projector.ROOT / "outputs")
    args = parser.parse_args()
    if args.frame < 0:
        parser.error("frame must be nonnegative")

    try:
        if not args.pcd.is_file():
            raise ValueError(f"PCD not found: {args.pcd}")
        scan_path = args.sequence / "velodyne_points/data" / f"{args.frame:010d}.bin"
        photo_path = args.sequence / f"image_{args.camera:02d}/data" / f"{args.frame:010d}.png"
        scan = np.fromfile(scan_path, dtype="<f4").reshape(-1, 4)
        xyz = np.asarray(o3d.io.read_point_cloud(str(args.pcd)).points)
        if not len(xyz) or not np.isfinite(xyz).all():
            raise ValueError("PCD must contain finite XYZ points")
        identical = np.array_equal(xyz, scan[:, :3])
        with Image.open(photo_path) as source:
            image = source.convert("RGB")
        for x0, y0, x1, y1 in args.roi:
            if not (0 <= x0 < x1 <= image.width and 0 <= y0 < y1 <= image.height):
                raise ValueError("Each ROI must lie inside the original camera image")

        matrix, size = projector.load_projection(args.sequence.parent, args.camera)
        if image.size != size:
            raise ValueError(f"Image dimensions {image.size} differ from calibration {size}")
        pixels, depth = independent_projection(xyz, args.sequence.parent, args.camera)
        implementation_pixels, implementation_depth = projector.project_points(xyz, matrix, size)
        if not len(depth):
            raise ValueError("No PCD points project inside the camera image")
        np.testing.assert_allclose(implementation_pixels, pixels, atol=1e-8, rtol=0)
        np.testing.assert_allclose(implementation_depth, depth, atol=1e-8, rtol=0)
        max_difference = float(np.abs(pixels - implementation_pixels).max())

        lidar_text, lidar_time = timestamp(args.sequence, "velodyne_points/timestamps.txt", args.frame)
        cam_text, cam_time = timestamp(args.sequence, f"image_{args.camera:02d}/timestamps.txt", args.frame)
        _, start = timestamp(args.sequence, "velodyne_points/timestamps_start.txt", args.frame)
        _, end = timestamp(args.sequence, "velodyne_points/timestamps_end.txt", args.frame)
        camera_delay_ms = float((cam_time - lidar_time) / np.timedelta64(1, "ms"))
        spin_ms = float((end - start) / np.timedelta64(1, "ms"))

        report = [
            f"PCD: {args.pcd.resolve()}",
            f"BIN: {scan_path.resolve()}",
            f"Image: {photo_path.resolve()}",
            f"PCD XYZ equal to BIN XYZ, including point order: {identical}",
            f"PCD points: {len(xyz):,}; inside image: {len(depth):,}",
            f"Image / calibration dimensions: {size}",
            f"Max difference between separate and combined transforms: {max_difference:.3g} pixels",
            f"LiDAR forward timestamp: {lidar_text}",
            f"Camera timestamp: {cam_text}",
            f"Camera minus LiDAR forward timestamp: {camera_delay_ms:.6f} ms",
            f"LiDAR full spin duration: {spin_ms:.6f} ms",
            "",
            "The transform check confirms mathematical implementation consistency, not physical calibration accuracy.",
            "A timestamp difference alone does not prove the cause of a visible offset.",
            "The visualization uses the camera viewpoint; a free-view PCD viewer uses a different viewpoint.",
            "The current projector does not apply additional motion compensation or full camera-visibility filtering.",
            "To measure accuracy, label corresponding static 3D features and image pixels, then measure reprojection error.",
            "KITTI projection convention: https://www.cvlibs.net/publications/Geiger2013IJRR.pdf (equation 7)",
        ]
        args.output_dir.mkdir(parents=True, exist_ok=True)
        prefix = args.output_dir / f"alignment_check_camera_{args.camera:02d}_{args.frame:010d}"
        prefix.with_suffix(".txt").write_text("\n".join(report) + "\n", encoding="utf-8")
        save_comparison(image, pixels, depth, [tuple(roi) for roi in args.roi],
                        prefix.with_suffix(".png"), args.frame, args.camera)
        print("\n".join(report))
        print(f"Comparison: {prefix.with_suffix('.png')}")
    except (OSError, ValueError, KeyError, IndexError, AssertionError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
