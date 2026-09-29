#!/usr/bin/env python3
"""Run the three mapping deliverables and build Results/REPORT.md with figures."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

import lidar_to_geographic as geo


def run_script(name, arguments):
    subprocess.run([sys.executable, str(geo.ROOT / name), *map(str, arguments)], check=True)


def figures(output, sequence, frame, merged, data, ecef_table):
    os.environ.setdefault('MPLCONFIGDIR', str(geo.ROOT / 'outputs/.matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    world = data['world_xyz']
    selected = np.linspace(0, len(world) - 1, min(120000, len(world)), dtype=int)
    sample, ids = world[selected], data['frame_id'][selected]
    # Comparison: applying only the reference pose to every raw frame (no ego-motion correction).
    reference_id = merged['world_reference']['frame_id']
    _, ref_pose = geo.load_frame(sequence, reference_id)
    calibration = geo.load_imu_to_lidar(Path(merged['calibration_dir']) / 'calib_imu_to_velo.txt')
    _, ref_meta = geo.georeference_points(np.zeros((1, 3)), ref_pose, calibration,
                                        merged['utm_epsg'], merged['height_offset_m'])
    t_ref = np.asarray(ref_meta['T_ecef_from_lidar'])
    naive = np.empty_like(sample)
    for frame_id in np.unique(ids):
        scan, _ = geo.load_frame(sequence, int(frame_id))
        mask = ids == frame_id
        raw = scan[data['point_index'][selected[mask]], :3].astype(np.float64)
        naive[mask] = geo.ecef_to_world(raw @ t_ref[:3, :3].T + t_ref[:3, 3], merged['world_reference'])
    limits = np.percentile(np.vstack((sample[:, :2], naive[:, :2])), [1, 99], axis=0)
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    titles = ['All scans placed at reference pose', 'Each scan placed using its own OXTS pose']
    for ax, cloud, title in zip(axes, [naive, sample], titles):
        plotted = ax.scatter(cloud[:, 0], cloud[:, 1], c=ids, s=.5, cmap='turbo',
                             vmin=min(merged['frames']), vmax=max(merged['frames']), rasterized=True)
        ax.set(xlabel='World East (m)', ylabel='World North (m)', title=title,
               xlim=limits[:, 0], ylim=limits[:, 1])
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.2)
    trajectory = np.asarray(merged['trajectory_world_m'])
    axes[1].plot(trajectory[:, 0], trajectory[:, 1], 'k.-', linewidth=1, markersize=3, label='IMU trajectory')
    axes[1].legend(loc='upper right')
    fig.colorbar(plotted, ax=axes, label='Frame ID', shrink=.7)
    fig.suptitle(f"{len(merged['frames'])} frames | {merged['input_point_count']:,} input points | "
                 f"{merged['output_point_count']:,} retained\n"
                 f"Display: {len(selected):,} sampled points; viewport uses 1st–99th percentiles")
    fig.savefig(output / 'MultiFrame/merged_comparison.png', dpi=180)
    plt.close(fig)

    ecef = np.column_stack([ecef_table[f'ecef_{axis}_m'] for axis in 'xyz'])
    anchor = ecef.mean(axis=0)
    indices = np.linspace(0, len(ecef) - 1, min(30000, len(ecef)), dtype=int)
    offsets = ecef[indices] - anchor
    fig = plt.figure(figsize=(10, 7), constrained_layout=True)
    ax = fig.add_subplot(111, projection='3d')
    points = ax.scatter(*offsets.T, c=ecef_table['height_assumed_ellipsoid_m'][indices],
                        cmap='viridis', s=.4, rasterized=True)
    ax.set(xlabel='ECEF X - center X (m)', ylabel='ECEF Y - center Y (m)',
           zlabel='ECEF Z - center Z (m)', title=f'Frame {frame}: {len(ecef):,} points in ECEF (EPSG:4978)')
    fig.colorbar(points, ax=ax, shrink=.6, label='Assumed ellipsoidal height (m)')
    fig.suptitle('Display center (m): ' + ', '.join(f'{v:.3f}' for v in anchor)
                 + '\nPCD and CSV retain absolute ECEF coordinates.', fontsize=10)
    fig.savefig(output / 'ECEF/ecef_cloud.png', dpi=170)
    plt.close(fig)

    pairs = merged['neighbor_frame_distances']
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
    x = np.arange(len(pairs))
    axes[0].plot(x, [p['median_m'] for p in pairs], 'o-', label='Median, all samples')
    axes[0].plot(x, [p['p95_m'] for p in pairs], 'o-', label='P95, all samples')
    axes[0].set(ylabel='3D nearest-neighbor distance (m)')
    axes[0].legend()
    axes[1].plot(x, [100 * p['fraction_within_threshold'] for p in pairs], 'o-', color='#0b826e')
    axes[1].set(ylabel='Samples within 1 m (%)', ylim=(0, 105), xlabel='Consecutive selected frame pairs')
    axes[1].set_xticks(x, [f"{p['frame_a']}–{p['frame_b']}" for p in pairs], rotation=55)
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle('Cloud overlap diagnostic before voxel filtering; not absolute positioning accuracy')
    fig.savefig(output / 'MultiFrame/overlap_metrics.png', dpi=170)
    plt.close(fig)


def write_report(output, sequence, frame, geo_meta, merged, pixel_meta, table):
    checks = geo_meta['numerical_checks']
    errors = [
        ('LiDAR → ECEF → LiDAR (3D)', checks['ecef_to_lidar_roundtrip_3d']),
        ('UTM + height → ECEF so với ECEF ban đầu (3D)', checks['utm_height_to_ecef_roundtrip_3d']),
        ('ENU so với phép topocentric của PROJ (3D)', checks['enu_vs_proj_topocentric_3d']),
        ('World → ECEF, các điểm đã ghép (3D)', merged['world_ecef_roundtrip_3d']),
    ]
    rows = '\n'.join(f"| {label} | {s['rmse_m']:.3e} | {s['max_m']:.3e} |" for label, s in errors)
    sample_indices = np.linspace(0, len(table) - 1, 5, dtype=int)
    examples = '\n'.join(f"| {i} | {table['ecef_x_m'][i]:.6f} | {table['ecef_y_m'][i]:.6f} | "
                         f"{table['ecef_z_m'][i]:.6f} |" for i in sample_indices)
    pixel_rows = '\n'.join(f"| {r['query_id']} | ({r['u_px']:.3f}, {r['v_px']:.3f}) | {r['depth_m']:.3f} | "
        f"({r['world_x_m']:.3f}, {r['world_y_m']:.3f}, {r['world_z_m']:.3f}) | "
        f"{r['utm_easting_m']:.3f} | {r['utm_northing_m']:.3f} |" for r in pixel_meta['rows'])
    pairs = merged['neighbor_frame_distances']
    trajectory = np.asarray(merged['trajectory_world_m'])
    path_length = np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum()
    oxts_accuracy = [s['oxts_reported_position_accuracy_m'] for s in merged['frame_summaries']]
    reference = merged['world_reference']
    frame_stem = f'frame_{frame:010d}'
    text = f'''# Kết quả chuyển đổi tọa độ và ghép point cloud

Dữ liệu: `{sequence.name}`, frame minh họa **{frame}**, camera **{pixel_meta['camera']:02d}**.
Báo cáo và hình được tạo từ dữ liệu KITTI có sẵn bằng `python3 generate_mapping_results.py`.

- [x] Xuất ECEF: CSV, PCD và hình minh họa.
- [x] Ghép {len(merged['frames'])} frame trong cùng hệ World / UTM / ECEF.
- [x] Camera Pixel + depth → World / UTM / ECEF; bảng số liệu và ảnh vị trí pixel.
- [x] Kiểm tra sai số số học của phép biến đổi và khoảng cách giữa các cloud.
- [ ] Sai số định vị tuyệt đối so với điểm khảo sát độc lập: chưa có ground truth.

## 1. Sai số tọa độ World / UTM

World được định nghĩa là ENU cố định tại IMU frame **{reference['frame_id']}**:
latitude = **{reference['latitude_deg']:.11f}°**, longitude = **{reference['longitude_deg']:.11f}°**,
height giả định ellipsoid = **{reference['height_assumed_ellipsoid_m']:.6f} m**.
Trục X hướng Đông, Y hướng Bắc, Z hướng lên; không đặt lại gốc theo từng frame.
UTM dùng chung **EPSG:{merged['utm_epsg']}**. World và UTM có khác biệt do phép chiếu bản đồ.

| Phép kiểm tra | RMSE (m) | Max (m) |
|---|---:|---:|
{rows}

Các sai số trên phản ánh độ nhất quán số học với dữ liệu float64 trước khi làm tròn CSV.
Chúng không phải độ chính xác GPS, hiệu chuẩn hay sai số định vị thực tế.
Các kiểm tra vòng kín có thể cùng kế thừa một sai lệch đầu vào; đối chiếu PROJ chỉ kiểm tra bước ENU/ECEF.
OXTS báo `pos_accuracy` tại frame {frame}: **{geo_meta['oxts_reported_position_accuracy_m']:.6f} m**;
khoảng giá trị trên các frame ghép: **{min(oxts_accuracy):.6f}–{max(oxts_accuracy):.6f} m**.
Đây là chỉ báo do bộ định vị cung cấp, chưa phải sai số đo được của từng điểm LiDAR.

Ảnh có sẵn trong `L2W/` là kết quả đối chiếu trực quan. Chưa xác nhận điểm chọn trên
CloudCompare và điểm trên công cụ bản đồ là cùng điểm khảo sát, nên không dùng hiệu
tọa độ giữa hai ảnh làm sai số tuyệt đối. Khi có cặp điểm cùng vị trí và hệ tọa độ,
đo sai số ngang bằng `sqrt((E - E_ref)^2 + (N - N_ref)^2)` và tổng hợp RMSE trên các cặp.

## 2. Kết quả chuyển sang ECEF

Chuyển **{geo_meta['point_count']:,} điểm** của frame {frame} theo chuỗi:
LiDAR → IMU → ENU tại OXTS → ECEF. ECEF sử dụng **EPSG:4978**, đơn vị mét.
Tọa độ ECEF là tọa độ tuyệt đối gốc tâm Trái Đất; hình bên dưới trừ một tâm hiển thị
để dễ đọc, còn CSV/PCD giữ nguyên tọa độ tuyệt đối.

| Point index trong BIN (từ 0) | ECEF X (m) | ECEF Y (m) | ECEF Z (m) |
|---:|---:|---:|---:|
{examples}

![Point cloud ECEF](ECEF/ecef_cloud.png)

Tệp: [CSV](ECEF/{frame_stem}.csv), [ECEF PCD](ECEF/{frame_stem}_ecef.pcd),
[UTM PCD](ECEF/{frame_stem}_utm.pcd), [metadata và sai số](ECEF/{frame_stem}.json).
PCD dùng ASCII float64 XYZ; khi mở tọa độ lớn trong CloudCompare nên chấp nhận Global Shift.

## 3. Kết quả ghép point cloud nhiều frame

Ghép các frame **{', '.join(map(str, merged['frames']))}** bằng tư thế OXTS tương ứng,
sau đó đưa vào cùng ECEF, cùng gốc World và cùng vùng UTM.

- Tổng số điểm đầu vào: **{merged['input_point_count']:,}**.
- Số điểm xuất: **{merged['output_point_count']:,}**; voxel **{merged['voxel_size_m']:g} m**.
- Bộ lọc giữ một điểm gốc trong mỗi voxel, bảo toàn `frame_id`, `point_index`, intensity trong NPZ.
- Chiều dài quỹ đạo IMU qua các frame đã chọn (3D): **{path_length:.3f} m**.
- Dùng `--voxel-size 0` nếu cần giữ toàn bộ điểm.

![So sánh ghép point cloud](MultiFrame/merged_comparison.png)

Trái: mọi scan dùng tư thế của frame gốc. Phải: mỗi scan dùng tư thế OXTS riêng.
Hai hình dùng cùng tập điểm, cùng hệ trục và cùng giới hạn hiển thị; màu biểu thị frame.
Hình lấy mẫu để dễ xem, không thay đổi dữ liệu xuất. Không áp dụng ICP hoặc tối ưu quỹ đạo.

Kiểm tra khoảng cách nearest-neighbor đối xứng trên tối đa 10.000 điểm mỗi chiều cho
mỗi cặp frame liên tiếp đã chọn, trước khi lọc voxel:

- Median khoảng cách theo từng cặp nằm trong **{min(p['median_m'] for p in pairs):.4f}–{max(p['median_m'] for p in pairs):.4f} m**.
- P95 theo từng cặp nằm trong **{min(p['p95_m'] for p in pairs):.4f}–{max(p['p95_m'] for p in pairs):.4f} m**.
- Tỷ lệ mẫu có điểm gần nhất trong 1 m: **{100 * min(p['fraction_within_threshold'] for p in pairs):.2f}–{100 * max(p['fraction_within_threshold'] for p in pairs):.2f}%**.

![Khoảng cách cloud giữa các frame](MultiFrame/overlap_metrics.png)

Khoảng cách này chịu ảnh hưởng của mật độ lấy mẫu, vùng không chồng lấn, che khuất,
vật thể chuyển động và sai số pose. Không diễn giải thành RMSE định vị tuyệt đối.

Tệp: [World PCD](MultiFrame/merged_world.pcd), [UTM PCD](MultiFrame/merged_utm.pcd),
[ECEF PCD](MultiFrame/merged_ecef.pcd), [NPZ có nguồn gốc từng điểm](MultiFrame/merged_points.npz),
[metadata và số liệu từng cặp frame](MultiFrame/merged.json).

## 4. Kết quả Camera Pixel → World / UTM

Ảnh đầu vào là ảnh KITTI đã rectified đúng kích thước hiệu chuẩn.
Một pixel riêng lẻ chỉ xác định một tia; để lấy tọa độ 3D cần thêm độ sâu.
Ví dụ dưới dùng **{len(pixel_meta['rows'])} pixel tại đúng vị trí chiếu của điểm LiDAR frame {frame}**,
với độ sâu theo trục quang học từ chính điểm đó, không phải khoảng cách Euclid tới camera.

Với `P_total = P_rect @ R_rect_00 @ T_velo_to_cam = [M | b]`:
`p_lidar = inverse(M) @ (depth * [u, v, 1] - b)`.
Sau đó dùng chuỗi chuyển LiDAR → ECEF → World / UTM. Phép tính giữ cả thành phần
tịnh tiến trong P, bao gồm baseline giữa các camera.

| ID | Pixel (u, v) | Depth (m) | World ENU (m) | UTM E (m) | UTM N (m) |
|---:|---|---:|---|---:|---:|
{pixel_rows}

![Vị trí pixel được chuyển đổi](CameraToWorld/pixel_world.png)

Sai số chiếu lại lớn nhất: **{pixel_meta['pixel_reprojection_max_error_px']:.3e} pixel**.
Sai số khôi phục điểm LiDAR của ví dụ: **{pixel_meta['demo_lidar_roundtrip_3d']['max_m']:.3e} m** (max 3D).
Đây là kiểm tra số học trên cặp pixel/depth sinh từ LiDAR, chưa phải kiểm định độc lập
độ chính xác định vị một vật thể được chọn bằng mắt.

Khi nhập pixel tùy chọn, chương trình có thể lấy depth từ mẫu LiDAR gần nhất trong
ngưỡng mặc định 2 pixel. Depth khi đó là ước lượng theo giả định cùng bề mặt; chương
trình báo lỗi nếu không có mẫu trong ngưỡng. Z-buffer giữ mẫu gần nhất trong mỗi
ô pixel, nhưng không giải quyết hoàn toàn che khuất giữa camera và LiDAR.

Tệp: [bảng đầy đủ gồm ECEF, latitude/longitude, height](CameraToWorld/pixel_world.csv),
[metadata, nguồn depth và point index](CameraToWorld/pixel_world.json).

## 5. Giới hạn và cách chạy lại

Một pose OXTS được áp dụng cho toàn bộ scan; chưa deskew từng điểm hoặc loại vật thể
động. Cao độ OXTS cộng `height-offset-m` được giả định là cao độ ellipsoid WGS84;
chưa có kiểm chứng datum cao độ hay mô hình geoid. Metadata lưu giả định này.

```bash
python3 -m pip install -r requirements-mapping.txt
python3 generate_mapping_results.py
python3 -m unittest test_lidar_to_geographic test_mapping_pipeline -v
```

Chạy riêng từng phần:

```bash
python3 lidar_to_geographic.py --frame 6 --output Results/ECEF/frame_0000000006.csv
python3 merge_point_clouds.py --frames 0 3 6 9 12 15 18 21 --voxel-size 0.2
python3 camera_pixel_to_world.py --frame 6 --camera 2
# Ví dụ cú pháp với depth tự cung cấp (20 m chỉ là số minh họa):
python3 camera_pixel_to_world.py --pixel 620 200 --depth-m 20 --output-dir outputs/pixel_custom
```

Nguồn quy ước: [KITTI raw data](https://www.cvlibs.net/datasets/kitti/raw_data.php),
[phép topocentric ECEF ↔ ENU của PROJ](https://proj.org/en/stable/operations/conversions/topocentric.html).
'''
    (output / 'REPORT.md').write_text(text, encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence', type=Path, default=geo.DEFAULT_SEQUENCE)
    parser.add_argument('--frame', type=int, default=6)
    parser.add_argument('--camera', type=int, choices=range(4), default=2)
    parser.add_argument('--reference-frame', type=int, default=0)
    parser.add_argument('--frames', type=int, nargs='+')
    parser.add_argument('--height-offset-m', type=float, default=0.0)
    parser.add_argument('--voxel-size', type=float, default=0.2)
    parser.add_argument('--output-dir', type=Path, default=geo.ROOT / 'Results')
    args = parser.parse_args()
    try:
        _, pose = geo.load_frame(args.sequence, args.reference_frame)
        epsg = geo.automatic_utm_epsg(*pose[:2])
        shared = ['--sequence', args.sequence.resolve(), '--height-offset-m', args.height_offset_m, '--utm-epsg', epsg]
        csv_path = args.output_dir / f'ECEF/frame_{args.frame:010d}.csv'
        run_script('lidar_to_geographic.py', [*shared, '--frame', args.frame, '--output', csv_path])
        run_script('merge_point_clouds.py', [*shared, '--reference-frame', args.reference_frame,
            '--voxel-size', args.voxel_size, '--output-dir', args.output_dir / 'MultiFrame',
            *(['--frames', *args.frames] if args.frames is not None else [])])
        run_script('camera_pixel_to_world.py', [*shared, '--frame', args.frame, '--camera', args.camera,
            '--reference-frame', args.reference_frame, '--output-dir', args.output_dir / 'CameraToWorld'])
        geo_meta = json.loads(csv_path.with_suffix('.json').read_text())
        merged = json.loads((args.output_dir / 'MultiFrame/merged.json').read_text())
        pixel_meta = json.loads((args.output_dir / 'CameraToWorld/pixel_world.json').read_text())
        table = np.atleast_1d(np.genfromtxt(csv_path, names=True, delimiter=','))
        with np.load(args.output_dir / 'MultiFrame/merged_points.npz') as data:
            figures(args.output_dir, args.sequence, args.frame, merged, data, table)
        write_report(args.output_dir, args.sequence, args.frame, geo_meta, merged, pixel_meta, table)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.error(str(error))
    print(f"Report: {(args.output_dir / 'REPORT.md').resolve()}")


if __name__ == '__main__':
    main()
