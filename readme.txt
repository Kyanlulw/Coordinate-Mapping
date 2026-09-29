KITTI raw data downloader

Requirements: Bash, unzip, and either wget or curl.
This Windows PC's Git Bash provides Bash, unzip, and curl; no extra
package installation is needed. The script uses curl when wget is missing.

Run from this project folder in PowerShell:

& 'C:\Program Files\Git\bin\bash.exe' ./raw_data_downloader.sh

Or, from this folder in Git Bash or a Linux Bash terminal:

bash ./raw_data_downloader.sh

Currently enabled downloads:
  2011_09_26_calib.zip
  2011_09_26_drive_0048_sync.zip

To change the selection, comment or uncomment entries in the script's
files array before running it.

Downloads and extracted data are written to the current working folder.
Each date folder contains its selected sequences and calibration files.
Existing extracted files are overwritten. Each ZIP is deleted only after
successful extraction. A failed download or extraction stops the script.

Project LiDAR onto one camera image

Requirements: Python 3.9+, numpy, Pillow.
Install packages if needed: python -m pip install numpy Pillow

Run from this project folder:
  python project_lidar_to_camera.py

The default is drive 0048, frame 6, camera 2 (left color image).
Output: outputs/lidar_on_camera_02_0000000006.png
Colors indicate camera depth: red = near, blue = 80 m or farther.
Points behind the camera or outside the image are excluded.

Choose a different frame or camera:
  python project_lidar_to_camera.py --frame 10 --camera 2
  python project_lidar_to_camera.py --sequence PATH_TO_SYNC_SEQUENCE --frame 0

If Python is not on PATH on this Windows PC, use PowerShell:
  & 'C:\Users\ACER\miniforge3\python.exe' .\project_lidar_to_camera.py

Projection uses the calibration files in the sequence's parent directory:
  image_homogeneous = P_rect_02 * R_rect_00 * T_velo_to_cam * [x, y, z, 1]
Then divide the first two coordinates by the third to obtain pixel (u, v).
It reads Velodyne .bin files directly; PCD conversion and OXTS are not needed.
This projects a synchronized scan without additional motion compensation.
Reference: https://www.cvlibs.net/publications/Geiger2013IJRR.pdf

Georeference LiDAR to UTM and WGS84 latitude/longitude

The script lidar_to_geographic.py uses matching LiDAR and OXTS frame IDs.
The default is frame 6 of drive 0048. The full scan is converted, including
points outside the camera field of view. No camera calibration is required.

Run in PowerShell using this project's environment:
  & '.\.venv\Scripts\python.exe' .\lidar_to_geographic.py --frame 6

Convert a specific LiDAR point (meters, x forward / y left / z up):
  & '.\.venv\Scripts\python.exe' .\lidar_to_geographic.py --frame 6 --point 10 0 0

On another machine, install requirements-geographic.txt in your Python environment:
  python -m pip install -r requirements-geographic.txt
  python lidar_to_geographic.py --frame 6

Outputs under outputs/geographic/:
  *_scan.csv: original point index, XYZ, intensity, UTM easting/northing,
              WGS84 latitude/longitude, assumed height, local ENU offsets, and ECEF XYZ (EPSG:4978).
  *_scan_utm.pcd: x = easting, y = northing, z = assumed ellipsoidal height.
  *_scan_ecef.pcd: Earth-centered Earth-fixed XYZ, EPSG:4978, meters.
  *_scan.json: EPSG code, exact input paths/frame ID, OXTS pose and timestamps,
              transformation matrices, height assumption, and navigation status.
Manual --point exports use *_points filenames and have no measured intensity (NaN).
PCD uses ASCII with float64 XYZ to preserve precision at large UTM coordinates.
ASCII also supports the legacy Open3D reader used by bin_to_pcd.py. A viewer
that converts to float32 may require a global coordinate shift for precise display.

This drive automatically selects WGS84 / UTM zone 32N, EPSG:32632.
UTM easting/northing are meters. Latitude/longitude are decimal degrees (EPSG:4326).
Use --utm-epsg to explicitly fix a zone when processing neighboring zones.

Transformation:
  p_imu = inverse(T_lidar_from_imu) * p_lidar
  p_enu = Rz(yaw) * Ry(pitch) * Rx(roll) * p_imu
  p_ecef = origin_ecef + R_ecef_from_enu * p_enu
  p_ecef -> WGS84 longitude/latitude -> UTM, using pyproj/PROJ
calib_imu_to_velo.txt stores IMU -> LiDAR, so it must be inverted.
KITTI yaw is zero east and increases counterclockwise, in radians.
Local ENU offsets are relative to the OXTS position of the selected frame.
Global UTM, rather than those per-frame ENU offsets, can combine different frames.
Projecting via WGS84 accounts for UTM grid convergence and scale. Adding ENU
offsets directly to UTM coordinates would omit these effects.

Accuracy and height:
  - One OXTS pose is applied to the entire scan. There is no interpolation,
    per-point deskew, or correction for independently moving objects.
  - The OXTS text format only says "altitude (m)"; it does not identify a vertical
    datum. OxTS can output geoidal altitude, so do not assume verified WGS84 height.
  - Computation treats OXTS altitude + --height-offset-m as ellipsoidal height.
    The default offset is 0. Height is explicitly marked as assumed in the CSV
    and JSON. Horizontal coordinates also depend slightly on this assumption.
    Supply a known ellipsoidal-minus-reported-height offset when available;
    no geoid model is downloaded or applied automatically.
  - Decimal precision does not establish absolute positioning accuracy.

References:
  https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
  https://proj.org/en/stable/operations/conversions/topocentric.html
  https://www.oxts.com/software/navsuite/documentation/manuals/NCOM_man.pdf


Complete the ECEF, multi-frame and Camera Pixel -> World / UTM results

Install all dependencies:
  python -m pip install -r requirements-mapping.txt

Generate the three deliverables and a Vietnamese report with figures:
  python generate_mapping_results.py

Default: drive 0048, frame 6 / camera 2 for single-frame examples, all 22 scans
for merging, one World ENU origin at IMU frame 0, one fixed UTM zone (32N).
The existing images in Results/L2C and Results/L2W are preserved.
Open Results/REPORT.md to see measured results, sample coordinates and figures.

New results:
  Results/ECEF/: CSV includes ECEF XYZ; absolute ECEF and UTM PCD; metadata;
                 numerical round-trip checks and an ECEF visualization.
  Results/MultiFrame/: merged_world.pcd, merged_utm.pcd, merged_ecef.pcd,
                      merged_points.npz, merged.json and comparison figures.
  Results/CameraToWorld/: pixel_world.csv, pixel_world.json, pixel_world.png.

World is one fixed local ENU system at the reference IMU position. It is NOT
an independent local ENU origin for every scan. UTM is a projected global CRS,
and its XY axes/scale differ from local ENU due to map projection effects.
All geometry remains float64. Height is still assumed ellipsoidal as above.

Choose frames and voxel filtering:
  python merge_point_clouds.py --frames 0 3 6 9 12 15 18 21 --voxel-size 0.2
  python merge_point_clouds.py --voxel-size 0 --output-dir outputs/merged_full
  python generate_mapping_results.py --frames 0 6 12 18 21 --output-dir outputs/report_subset

Voxel filtering retains the first original point in each fixed-World voxel.
NPZ stores world_xyz, ecef_xyz, utm_xyz, intensity, frame_id and point_index.
It preserves source identities; it does not average or fit the points.
Use --voxel-size 0 to retain every point. PCD stores XYZ and intensity; consult
NPZ for per-point provenance. No ICP, per-point deskew or moving-object removal
is applied. Nearest-neighbor overlap statistics are measured before voxel
filtering and are not absolute positioning error against surveyed ground truth.

Pixel + depth -> World / UTM / ECEF:
  python camera_pixel_to_world.py --frame 6 --camera 2
  python camera_pixel_to_world.py --pixel 620 200 --depth-m 20 --output-dir outputs/pixel_custom
  python camera_pixel_to_world.py --pixel 620 200 --max-pixel-distance 2 --output-dir outputs/pixel_lidar

The 20m depth above is an example input, not an observed depth at that pixel.
Repeat --pixel U V for several pixels, then supply one --depth-m D1 D2 ...
value for each pixel. Pixels are in the ORIGINAL RECTIFIED image resolution.
Depth means the optical-axis projection scale s in [u*s,v*s,s] = P_total*p,
not Euclidean range. The inverse retains the entire projection translation,
including the stereo baseline. Pixel-only 3D reconstruction is not unique.

Without --depth-m, depth comes from the nearest in-image LiDAR projection within
--max-pixel-distance. A sparse z-buffer retains the closest sample per image
pixel. No matching sample causes a clear error; no arbitrary depth is invented.
Depth transfer between nearby pixels assumes the same surface and does not fully
resolve occlusion. JSON/CSV record the matched LiDAR index and pixel distance.
With no --pixel, five exact projected LiDAR samples are used as numerical demos;
their round-trip errors do not measure independent physical accuracy.
The image in Results/CameraToWorld marks these sample positions.

Verification:
  python -m unittest test_lidar_to_geographic test_mapping_pipeline -v

Tests cover analytic ECEF coordinates, independent PROJ topocentric checks,
full camera baseline inversion, positive-depth validation, z-buffer selection,
a static scene observed from translated/rotated poses, point provenance and
float64 PCD export. The geographic script now uses the correct .pcd extension
for UTM exports (older code accidentally wrote PCD content under a .ply name).
