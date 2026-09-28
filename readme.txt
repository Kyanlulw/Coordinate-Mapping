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
              WGS84 latitude/longitude, assumed height, and local ENU offsets.
  *_scan_utm.pcd: x = easting, y = northing, z = assumed ellipsoidal height.
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
