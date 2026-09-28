#!/usr/bin/env python3
"""Georeference a KITTI LiDAR scan or selected XYZ points using its OXTS pose.

LiDAR -> IMU -> local East/North/Up -> ECEF -> WGS84 -> UTM.
Requires numpy and pyproj. All transforms use float64 internally.

The OXTS text format does not identify its altitude datum. Geometry uses
OXTS altitude + --height-offset-m as an ASSUMED WGS84 ellipsoidal height.
This is one pose per scan, without per-point motion compensation.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from pyproj import CRS, Transformer
from pyproj.exceptions import ProjError


ROOT = Path(__file__).resolve().parent
DEFAULT_SEQUENCE = ROOT / "2011_09_26/2011_09_26_drive_0048_sync"


# Read calibration and build the 4x4 transform from IMU to LiDAR coordinates.
def load_imu_to_lidar(path: Path) -> np.ndarray:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(":")
        if separator and key in ("R", "T"):
            values[key] = np.array([float(item) for item in value.split()])
    transform = np.eye(4)
    transform[:3, :3] = values["R"].reshape(3, 3)
    transform[:3, 3] = values["T"].reshape(3)
    if not np.isfinite(transform).all():
        raise ValueError("IMU-to-LiDAR calibration contains non-finite values")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ValueError("Calibration R is not a rotation matrix")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise ValueError("Calibration R must have determinant +1")
    return transform


# Build the IMU-to-east/north/up rotation from roll, pitch, and yaw.
def imu_to_enu_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """KITTI angles: radians, yaw zero east, positive counterclockwise."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


# Rotate local east/north/up directions into Earth-centered, Earth-fixed axes.
def enu_to_ecef_rotation(latitude: float, longitude: float) -> np.ndarray:
    """Columns are the local east, north, up unit vectors in ECEF coordinates."""
    lat, lon = np.deg2rad([latitude, longitude])
    slat, clat = np.sin(lat), np.cos(lat)
    slon, clon = np.sin(lon), np.cos(lon)
    return np.array([
        [-slon, -slat * clon, clat * clon],
        [clon, -slat * slon, clat * slon],
        [0, clat, slat],
    ])


# Determine the WGS84 UTM EPSG code for the position's zone and hemisphere.
def automatic_utm_epsg(latitude: float, longitude: float) -> int:
    """Select the WGS84 UTM zone, including Norway/Svalbard exceptions."""
    if not (-80 <= latitude <= 84 and -180 <= longitude <= 180):
        raise ValueError("UTM requires latitude between -80 and 84 degrees")
    zone = min(60, int((longitude + 180) // 6) + 1)
    if 56 <= latitude < 64 and 3 <= longitude < 12:
        zone = 32
    elif 72 <= latitude <= 84 and 0 <= longitude < 42:
        zone = 31 if longitude < 9 else 33 if longitude < 21 else 35 if longitude < 33 else 37
    return (32600 if latitude >= 0 else 32700) + zone


# Convert LiDAR XYZ to geographic and UTM coordinates using calibration and the OXTS pose.
def georeference_points(xyz: np.ndarray, oxts: np.ndarray,
                       imu_to_lidar: np.ndarray, utm_epsg=None,
                       height_offset_m: float = 0.0) -> tuple:
    """Return per-point coordinates and metadata for a single OXTS pose."""
    xyz = np.asarray(xyz, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz) or not np.isfinite(xyz).all():
        raise ValueError("LiDAR XYZ must be a nonempty, finite N x 3 array")
    pose = np.asarray(oxts, dtype=np.float64).reshape(-1)
    if len(pose) < 6 or not np.isfinite(pose[:6]).all():
        raise ValueError("OXTS must provide finite latitude, longitude, altitude, roll, pitch, yaw")
    lat, lon, alt, roll, pitch, yaw = pose[:6]
    natural_epsg = automatic_utm_epsg(lat, lon)
    epsg = natural_epsg if utm_epsg is None else int(utm_epsg)
    if not (32601 <= epsg <= 32660 or 32701 <= epsg <= 32760):
        raise ValueError("UTM EPSG must be 32601..32660 (north) or 32701..32760 (south)")
    if (epsg < 32700) != (lat >= 0):
        raise ValueError("UTM hemisphere does not match the OXTS latitude")
    if not np.isfinite(height_offset_m):
        raise ValueError("Height offset must be finite")
    assumed_height = float(alt + height_offset_m)

    # Calibration is IMU -> LiDAR. Invert it to account for the sensor lever arm.
    lidar_to_imu = np.linalg.inv(imu_to_lidar)
    imu_xyz = xyz @ lidar_to_imu[:3, :3].T + lidar_to_imu[:3, 3]
    r_enu_imu = imu_to_enu_rotation(roll, pitch, yaw)
    enu_xyz = imu_xyz @ r_enu_imu.T

    # always_xy=True means longitude, latitude, height at the pyproj boundary.
    geodetic_to_ecef = Transformer.from_crs(4979, 4978, always_xy=True)
    ecef_to_geodetic = Transformer.from_crs(4978, 4979, always_xy=True)
    wgs84_to_utm = Transformer.from_crs(4326, epsg, always_xy=True)
    anchor_ecef = np.array(geodetic_to_ecef.transform(lon, lat, assumed_height, errcheck=True))
    r_ecef_enu = enu_to_ecef_rotation(lat, lon)
    ecef_xyz = enu_xyz @ r_ecef_enu.T + anchor_ecef
    longitude, latitude, height = ecef_to_geodetic.transform(*ecef_xyz.T, errcheck=True)
    easting, northing = wgs84_to_utm.transform(longitude, latitude, errcheck=True)
    coordinates = {
        "enu_m": enu_xyz,
        "ecef_m": ecef_xyz,
        "latitude_deg": np.asarray(latitude),
        "longitude_deg": np.asarray(longitude),
        "height_assumed_ellipsoid_m": np.asarray(height),
        "utm_easting_m": np.asarray(easting),
        "utm_northing_m": np.asarray(northing),
    }
    if not all(np.isfinite(value).all() for value in coordinates.values()):
        raise ValueError("Coordinate transformation produced non-finite results")

    t_enu_imu = np.eye(4)
    t_enu_imu[:3, :3] = r_enu_imu
    t_ecef_enu = np.eye(4)
    t_ecef_enu[:3, :3] = r_ecef_enu
    t_ecef_enu[:3, 3] = anchor_ecef
    metadata = {
        "point_count": len(xyz),
        "utm_epsg": epsg,
        "utm_crs": CRS.from_epsg(epsg).name,
        "wgs84_horizontal_epsg": 4326,
        "ecef_epsg": 4978,
        "origin_oxts": {
            "latitude_deg": float(lat), "longitude_deg": float(lon),
            "reported_altitude_m": float(alt), "roll_rad": float(roll),
            "pitch_rad": float(pitch), "yaw_rad": float(yaw),
        },
        "height_offset_m": float(height_offset_m),
        "origin_assumed_ellipsoidal_height_m": assumed_height,
        "height_datum_verified": False,
        "T_ecef_from_lidar": (t_ecef_enu @ t_enu_imu @ lidar_to_imu).tolist(),
        "T_enu_from_lidar": (t_enu_imu @ lidar_to_imu).tolist(),
        "enu_origin": "OXTS/IMU position of this frame; ENU is local to each frame",
        "rotation_convention": "R_enu_from_imu = Rz(yaw) @ Ry(pitch) @ Rx(roll)",
        "motion_compensation": "None; all points use the OXTS pose with the same frame ID",
        "height_assumption": (
            "OXTS text files do not specify the altitude datum. OXTS altitude plus height_offset_m "
            "is treated as WGS84 ellipsoidal height in the geometry. This assumption affects "
            "height and, to a smaller degree, horizontal coordinates. No geoid correction is automatic."
        ),
        "precision_note": "Float64 calculations and PCD XYZ; printed precision is not measurement accuracy",
        "references": [
            "https://www.cvlibs.net/publications/Geiger2013IJRR.pdf",
            "https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py",
            "https://proj.org/en/stable/operations/conversions/topocentric.html",
            "https://www.oxts.com/software/navsuite/documentation/manuals/NCOM_man.pdf",
        ],
    }
    return coordinates, metadata


# Save UTM coordinates, assumed height, and intensity as an ASCII PCD file.
def write_utm_pcd(path, coordinates, intensity, epsg):
    """Use ASCII float64 XYZ; legacy Open3D cannot read binary float64 PCD XYZ."""
    count = len(intensity)
    records = np.column_stack((coordinates["utm_easting_m"], coordinates["utm_northing_m"],
                               coordinates["height_assumed_ellipsoid_m"], intensity))
    header = (
        "# .PCD v0.7\n"
        f"# XY: EPSG:{epsg}; Z: assumed ellipsoidal height. See JSON metadata.\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 8 8 8 4\n"
        "TYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA ascii"
    )
    np.savetxt(path, records, fmt=["%.17g", "%.17g", "%.17g", "%.9g"],
               header=header, comments="", encoding="ascii")


# Read this frame's OXTS/LiDAR timestamps and compute their time difference.
def read_timing(sequence, frame):
    result = {}
    for key, relative in (("oxts", "oxts/timestamps.txt"),
                          ("lidar_forward", "velodyne_points/timestamps.txt"),
                          ("lidar_start", "velodyne_points/timestamps_start.txt"),
                          ("lidar_end", "velodyne_points/timestamps_end.txt")):
        path = sequence / relative
        if path.is_file():
            result[key] = path.read_text().splitlines()[frame].strip()
    if "oxts" in result and "lidar_forward" in result:
        delta = np.datetime64(result["oxts"], "ns") - np.datetime64(result["lidar_forward"], "ns")
        result["oxts_minus_lidar_ms"] = float(delta / np.timedelta64(1, "ms"))
    return result


# Parse command-line options, convert the selected points, and export the results.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE)
    parser.add_argument("--frame", type=int, default=6, help="Matching LiDAR/OXTS frame ID (default: 6)")
    parser.add_argument("--calib-dir", type=Path, help="Default: sequence parent directory")
    parser.add_argument("--point", nargs=3, type=float, action="append", metavar=("X", "Y", "Z"),
                        help="Convert this LiDAR point instead of the scan; repeat for more points")
    parser.add_argument("--utm-epsg", type=int, help="Override automatic WGS84 UTM zone selection")
    parser.add_argument("--height-offset-m", type=float, default=0.0,
                        help="Add to OXTS altitude to obtain assumed ellipsoidal height (default: 0)")
    parser.add_argument("--output", type=Path, help="CSV path; matching JSON and UTM PCD are also written")
    args = parser.parse_args()
    if args.frame < 0:
        parser.error("frame must be nonnegative")
    frame = f"{args.frame:010d}"
    lidar_path = args.sequence / "velodyne_points/data" / f"{frame}.bin"
    oxts_path = args.sequence / "oxts/data" / f"{frame}.txt"
    calib_dir = args.calib_dir if args.calib_dir is not None else args.sequence.parent
    calib_path = calib_dir / "calib_imu_to_velo.txt"
    mode = "points" if args.point else "scan"
    output = args.output if args.output is not None else (
        ROOT / "outputs/geographic" / f"{args.sequence.name}_{frame}_{mode}.csv"
    )
    try:
        if output.suffix.lower() != ".csv":
            raise ValueError("Output must have a .csv extension")
        if args.point:
            xyz = np.asarray(args.point, dtype=np.float64)
            intensity = np.full(len(xyz), np.nan)  # No measured intensity for manually supplied points.
        else:
            byte_count = lidar_path.stat().st_size
            if byte_count == 0 or byte_count % 16:
                raise ValueError("KITTI LiDAR BIN must contain 16 bytes per point")
            scan = np.fromfile(lidar_path, dtype="<f4").reshape(-1, 4)
            xyz, intensity = scan[:, :3], scan[:, 3]
        oxts = np.loadtxt(oxts_path).reshape(-1)
        if len(oxts) != 30 or not np.isfinite(oxts).all():
            raise ValueError("Expected 30 finite OXTS values")
        coordinates, metadata = georeference_points(
            xyz, oxts, load_imu_to_lidar(calib_path), args.utm_epsg, args.height_offset_m
        )
        metadata.update({
            "sequence": args.sequence.name, "frame_id": frame,
            "lidar_file": str(lidar_path.resolve()) if not args.point else None,
            "manual_lidar_points": args.point,
            "oxts_file": str(oxts_path.resolve()), "calibration_file": str(calib_path.resolve()),
            "timestamps": read_timing(args.sequence, args.frame),
            "oxts_reported_position_accuracy_m": float(oxts[23]),
            "oxts_status": dict(zip(("navstat", "numsats", "posmode", "velmode", "orimode"),
                                    (int(value) for value in oxts[25:30]))),
            "pcd_axes": "x=UTM easting, y=UTM northing, z=assumed ellipsoidal height; all meters",
            "pcd_format": "PCD 0.7 ASCII; float64 XYZ, float32 intensity",
        })
        table = np.column_stack((
            np.arange(len(xyz)), xyz, intensity, coordinates["utm_easting_m"],
            coordinates["utm_northing_m"], coordinates["latitude_deg"],
            coordinates["longitude_deg"], coordinates["height_assumed_ellipsoid_m"],
            coordinates["enu_m"],
        ))
        columns = ("point_index,lidar_x_m,lidar_y_m,lidar_z_m,intensity,utm_easting_m,"
                   "utm_northing_m,latitude_deg,longitude_deg,height_assumed_ellipsoid_m,"
                   "enu_east_m,enu_north_m,enu_up_m")
        output.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(output, table, delimiter=",", header=columns, comments="",
                   fmt=["%d"] + ["%.9g"] * 4 + ["%.6f"] * 2 + ["%.10f"] * 2 + ["%.6f"] * 4)
        pcd_path = output.with_name(output.stem + "_utm.ply")
        write_utm_pcd(pcd_path, coordinates, intensity, metadata["utm_epsg"])
        metadata["csv_file"] = str(output.resolve())
        metadata["utm_pcd_file"] = str(pcd_path.resolve())
        output.with_suffix(".json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (OSError, ValueError, KeyError, IndexError, np.linalg.LinAlgError, ProjError) as error:
        parser.error(str(error))

    print(f"Frame {frame}: converted {len(xyz):,} LiDAR points")
    print(f"UTM: {metadata['utm_crs']} (EPSG:{metadata['utm_epsg']})")
    print(f"CSV: {output.resolve()}")
    print(f"PCD: {pcd_path.resolve()}")
    print(f"Metadata: {output.with_suffix('.json').resolve()}")
    print("Height datum is unverified; see height_assumption in metadata. No scan motion compensation.")
    if len(xyz) <= 10:
        for index in range(len(xyz)):
            print(f"Point {index}: lat={coordinates['latitude_deg'][index]:.10f}, "
                  f"lon={coordinates['longitude_deg'][index]:.10f}, "
                  f"E={coordinates['utm_easting_m'][index]:.3f}, "
                  f"N={coordinates['utm_northing_m'][index]:.3f}")


if __name__ == "__main__":
    main()
