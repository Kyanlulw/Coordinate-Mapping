from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parent
bin_file = ROOT / "2011_09_26/2011_09_26_drive_0048_sync/velodyne_points/data/0000000021.bin"
pcd_file = ROOT / "temp21.pcd"

# KITTI: mỗi point = [x, y, z, reflectance]
points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)

xyz = points[:, :3]

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)

if not o3d.io.write_point_cloud(str(pcd_file), pcd):
    raise RuntimeError(f"Could not write {pcd_file}")

print(f"Saved {len(xyz)} points to {pcd_file}")
