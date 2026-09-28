#!/usr/bin/env python3
"""Project one KITTI raw Velodyne scan onto its rectified camera image.

Requires numpy and Pillow. Defaults to drive 0048, frame 6, left color camera.
Projection: [u*s, v*s, s] = P_rect_02 @ R_rect_00 @ T_velo_to_cam @ [x,y,z,1].
Reference: https://www.cvlibs.net/publications/Geiger2013IJRR.pdf
"""

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
DEFAULT_SEQUENCE = ROOT / "2011_09_26" / "2011_09_26_drive_0048_sync"


def read_calibration(path: Path) -> dict:
    """Read numeric calibration fields, skipping metadata such as calib_time."""
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(":")
        if not separator:
            continue
        try:
            fields[key.strip()] = np.array([float(item) for item in value.split()])
        except ValueError:
            continue
    return fields


def load_projection(calibration_dir: Path, camera: int) -> tuple:
    velo = read_calibration(calibration_dir / "calib_velo_to_cam.txt")
    cam = read_calibration(calibration_dir / "calib_cam_to_cam.txt")

    # Velodyne coordinates -> unrectified reference camera (camera 0).
    velo_to_cam = np.eye(4)
    velo_to_cam[:3, :3] = velo["R"].reshape(3, 3)
    velo_to_cam[:3, 3] = velo["T"].reshape(3)

    # All P_rect matrices use the rectified reference camera coordinate frame.
    # Use R_rect_00 even when projecting onto camera 2 or camera 3.
    rectification = np.eye(4)
    rectification[:3, :3] = cam["R_rect_00"].reshape(3, 3)
    camera_projection = cam[f"P_rect_{camera:02d}"].reshape(3, 4)
    projection = camera_projection @ rectification @ velo_to_cam
    if not np.isfinite(projection).all():
        raise ValueError("Calibration contains non-finite values")
    image_size = tuple(int(value) for value in cam[f"S_rect_{camera:02d}"])
    return projection, image_size


def project_points(points: np.ndarray, projection: np.ndarray, image_size: tuple) -> tuple:
    """Return in-image pixel coordinates and positive camera depths in meters."""
    xyz = points[:, :3]
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    homogeneous = np.column_stack((xyz, np.ones(len(xyz))))
    projected = homogeneous @ projection.T

    # Divide only points in front of the camera; s is camera optical-axis depth.
    projected = projected[np.isfinite(projected).all(axis=1) & (projected[:, 2] > 0)]
    depth = projected[:, 2]
    pixels = projected[:, :2] / depth[:, None]
    width, height = image_size
    visible = (
        np.isfinite(pixels).all(axis=1)
        & (pixels[:, 0] >= 0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < height)
    )
    return pixels[visible], depth[visible]


def depth_colors(depth: np.ndarray, color_max_depth: float) -> np.ndarray:
    """Map camera depth to RGB: red (near), yellow, green, cyan, blue (far)."""
    stops = np.array(
        [[255, 45, 35], [255, 210, 35], [60, 230, 90], [25, 200, 255], [70, 80, 255]]
    )
    positions = np.linspace(0, 1, len(stops))
    normalized = np.clip(depth / color_max_depth, 0, 1)
    return np.column_stack(
        [np.interp(normalized, positions, stops[:, channel]) for channel in range(3)]
    ).astype(np.uint8)


def draw_overlay(image: Image.Image, pixels: np.ndarray, depth: np.ndarray,
                 radius: int, opacity: float, color_max_depth: float) -> Image.Image:
    overlay = image.copy()
    draw = ImageDraw.Draw(overlay)
    colors = depth_colors(depth, color_max_depth)
    # Paint nearer points last when marker footprints overlap.
    for index in np.argsort(depth)[::-1]:
        u, v = pixels[index]
        u, v = int(u), int(v)
        color = tuple(int(channel) for channel in colors[index])
        if radius == 0:
            draw.point((u, v), fill=color)
        else:
            draw.ellipse((u - radius, v - radius, u + radius, v + radius), fill=color)
    return Image.blend(image, overlay, opacity)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE,
                        help="Path to a KITTI *_sync sequence")
    parser.add_argument("--calib-dir", type=Path,
                        help="Calibration directory (default: sequence's parent directory)")
    parser.add_argument("--frame", type=int, default=6, help="Frame number (default: 6)")
    parser.add_argument("--camera", type=int, choices=range(4), default=2,
                        help="Camera index: 0/1 grayscale, 2/3 color (default: 2)")
    parser.add_argument("--output", type=Path,
                        help="Output PNG path (default: outputs/lidar_on_camera_XX_FRAME.png)")
    parser.add_argument("--point-radius", type=int, default=1,
                        help="Point radius in pixels; 0 draws a single pixel (default: 1)")
    parser.add_argument("--opacity", type=float, default=0.85,
                        help="Point opacity from 0 to 1 (default: 0.85)")
    parser.add_argument("--color-max-depth", type=float, default=80.0,
                        help="Depth in meters mapped to blue; farther points remain blue (default: 80)")
    args = parser.parse_args()
    if args.frame < 0 or args.point_radius < 0:
        parser.error("frame and point-radius must be nonnegative")
    if not 0 <= args.opacity <= 1:
        parser.error("opacity must be between 0 and 1")
    if not np.isfinite(args.color_max_depth) or args.color_max_depth <= 0:
        parser.error("color-max-depth must be finite and positive")

    frame = f"{args.frame:010d}"
    image_path = args.sequence / f"image_{args.camera:02d}" / "data" / f"{frame}.png"
    lidar_path = args.sequence / "velodyne_points" / "data" / f"{frame}.bin"
    calibration_dir = args.calib_dir if args.calib_dir is not None else args.sequence.parent
    output = args.output if args.output is not None else (
        ROOT / "outputs" / f"lidar_on_camera_{args.camera:02d}_{frame}.png"
    )

    try:
        if output.suffix.lower() != ".png":
            raise ValueError("Output must have a .png extension")
        if output.resolve() in {image_path.resolve(), lidar_path.resolve()}:
            raise ValueError("Output must differ from the source image and scan")
        byte_count = lidar_path.stat().st_size
        if byte_count == 0 or byte_count % 16:
            raise ValueError("KITTI scan must contain four float32 values per point (16 bytes)")
        points = np.fromfile(lidar_path, dtype="<f4").reshape(-1, 4)
        with Image.open(image_path) as source_image:
            image = source_image.convert("RGB")
        projection, expected_size = load_projection(calibration_dir, args.camera)
        if image.size != expected_size:
            raise ValueError(
                f"Image size {image.size} differs from calibration {expected_size}; "
                "use the original rectified KITTI image"
            )
        pixels, depth = project_points(points, projection, image.size)
        if not len(depth):
            raise ValueError("No LiDAR points project inside this image")
        result = draw_overlay(image, pixels, depth, args.point_radius,
                              args.opacity, args.color_max_depth)
        output.parent.mkdir(parents=True, exist_ok=True)
        result.save(output)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))

    print(f"Camera image: {image_path}")
    print(f"LiDAR scan: {lidar_path}")
    print(f"Projected {len(depth):,} / {len(points):,} points inside {image.width}x{image.height} image")
    print(f"Camera depth: {depth.min():.2f} to {depth.max():.2f} m")
    print(f"Colors: red = near, blue = {args.color_max_depth:g} m or farther")
    print(f"Saved: {output.resolve()}")


if __name__ == "__main__":
    main()
