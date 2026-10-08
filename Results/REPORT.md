# Kết quả chuyển đổi tọa độ và ghép point cloud

Dữ liệu: `2011_09_26_drive_0048_sync`, frame minh họa **6**, camera **02**.
Báo cáo và hình được tạo từ dữ liệu KITTI có sẵn bằng `python3 generate_mapping_results.py`.

- [x] Xuất ECEF: CSV, PCD và hình minh họa.
- [x] Ghép 22 frame trong cùng hệ World / UTM / ECEF.
- [x] Camera Pixel + depth → World / UTM / ECEF; bảng số liệu và ảnh vị trí pixel.
- [x] Kiểm tra sai số số học của phép biến đổi và khoảng cách giữa các cloud.
- [ ] Sai số định vị tuyệt đối so với điểm khảo sát độc lập: chưa có ground truth.

## Báo cáo này dùng để kiểm tra điều gì?

Mục đích chung của báo cáo là kiểm tra toàn bộ chuỗi ánh xạ tọa độ có hoạt động
đúng và nhất quán hay không, từ cảm biến đến hệ tọa độ toàn cầu. Mỗi nhóm kết quả
trả lời một câu hỏi kỹ thuật khác nhau:

| Nhóm kết quả | Câu hỏi cần kiểm tra | Ứng dụng của kết quả |
|---|---|---|
| World / UTM và sai số số học | Công thức, chiều ma trận, thứ tự quay, hệ trục và phép chiếu có nhất quán không? | Phát hiện lỗi triển khai trước khi dùng tọa độ cho bản đồ hoặc đo đạc |
| ECEF | Một scan LiDAR có được đặt vào hệ tọa độ tuyệt đối gắn với Trái Đất không? | Trao đổi dữ liệu giữa các hệ địa lý và làm hệ trung gian để ghép nhiều frame |
| Ghép point cloud nhiều frame | Pose OXTS có bù được chuyển động của xe để các bề mặt tĩnh chồng lên nhau không? | Tạo bản đồ 3D rộng hơn một scan và đánh giá mức chồng lấn giữa các frame |
| Camera Pixel → World / UTM | Pixel cộng depth có đi ngược đúng qua camera, LiDAR và pose để ra tọa độ thế giới không? | Gán tọa độ địa lý cho vật thể hoặc điểm được phát hiện trên ảnh |
| Giới hạn | Các con số nào chỉ kiểm tra phần mềm, và kết luận nào cần ground truth? | Tránh hiểu sai sai số số học thành độ chính xác ngoài thực địa |

Báo cáo vì vậy là một bài kiểm tra tích hợp của pipeline. Nó chứng minh dữ liệu đi
qua các bước đúng về mặt toán học và cho các đầu ra hợp lý. Nó chưa phải chứng nhận
độ chính xác tuyệt đối ngoài thực địa vì chưa có điểm khảo sát độc lập làm ground truth.

## 1. Sai số tọa độ World / UTM

**Mục đích.** Phần này kiểm tra nền tảng toán học của pipeline trước khi đánh giá
point cloud bằng mắt. Nó được dùng để phát hiện các lỗi thường gặp như đảo sai chiều
ma trận IMU–LiDAR, sai dấu hoặc thứ tự roll/pitch/yaw, nhầm latitude với longitude,
chọn sai UTM zone, hoặc làm mất độ chính xác khi chuyển giữa các hệ tọa độ.

Mỗi dòng trong bảng có một nhiệm vụ riêng:

- **LiDAR → ECEF → LiDAR:** kiểm tra ma trận tổng và ma trận nghịch đảo có khôi phục
  đúng điểm đầu vào hay không.
- **UTM + height → ECEF:** kiểm tra phép chiếu WGS84/UTM và thứ tự longitude/latitude.
- **ENU so với PROJ topocentric:** so công thức ENU tự triển khai với thư viện PROJ
  độc lập để kiểm tra hướng trục East/North/Up.
- **World → ECEF:** kiểm tra hệ World cố định và gốc frame 0 có thể đổi ngược về ECEF.

Nếu một giá trị ở đây lớn bất thường, cần sửa công thức hoặc quy ước hệ trục trước
khi tin vào các kết quả ECEF, ghép frame hay Pixel → World ở các phần sau.

World được định nghĩa là ENU cố định tại IMU frame **0**:
latitude = **49.03086061586°**, longitude = **8.33974931234°**,
height giả định ellipsoid = **114.530075 m**.
Trục X hướng Đông, Y hướng Bắc, Z hướng lên; không đặt lại gốc theo từng frame.
UTM dùng chung **EPSG:32632**. World và UTM có khác biệt do phép chiếu bản đồ.

| Phép kiểm tra | RMSE (m) | Max (m) |
|---|---:|---:|
| LiDAR → ECEF → LiDAR (3D) | 4.062e-10 | 7.898e-10 |
| UTM + height → ECEF so với ECEF ban đầu (3D) | 1.103e-09 | 3.390e-09 |
| ENU so với phép topocentric của PROJ (3D) | 3.024e-10 | 5.209e-10 |
| World → ECEF, các điểm đã ghép (3D) | 0.000e+00 | 0.000e+00 |

Các sai số trên phản ánh độ nhất quán số học với dữ liệu float64 trước khi làm tròn CSV.
Chúng không phải độ chính xác GPS, hiệu chuẩn hay sai số định vị thực tế.
Các kiểm tra vòng kín có thể cùng kế thừa một sai lệch đầu vào; đối chiếu PROJ chỉ kiểm tra bước ENU/ECEF.
OXTS báo `pos_accuracy` tại frame 6: **0.116103 m**;
khoảng giá trị trên các frame ghép: **0.114691–0.117516 m**.
Đây là chỉ báo do bộ định vị cung cấp, chưa phải sai số đo được của từng điểm LiDAR.

Ảnh có sẵn trong `L2W/` là kết quả đối chiếu trực quan. Chưa xác nhận điểm chọn trên
CloudCompare và điểm trên công cụ bản đồ là cùng điểm khảo sát, nên không dùng hiệu
tọa độ giữa hai ảnh làm sai số tuyệt đối. Khi có cặp điểm cùng vị trí và hệ tọa độ,
đo sai số ngang bằng `sqrt((E - E_ref)^2 + (N - N_ref)^2)` và tổng hợp RMSE trên các cặp.

## 2. Kết quả chuyển sang ECEF

**Mục đích.** Phần này kiểm tra khả năng đặt toàn bộ scan vào một hệ Cartesian tuyệt
đối có gốc tại tâm Trái Đất. ECEF đóng vai trò hệ trung gian chung: hai điểm từ các
frame khác nhau có thể được so sánh trong cùng một hệ trước khi đổi sang World hoặc
UTM. Kết quả cũng kiểm tra rằng số điểm được bảo toàn, tọa độ hữu hạn và dữ liệu
float64 không làm mất phần chênh lệch nhỏ khi nền tọa độ có giá trị hàng triệu mét.

Các tọa độ trong bảng và hình được dùng để xác nhận cloud nằm gần vị trí OXTS dự
kiến và vẫn giữ đúng hình dạng cục bộ. Chúng không tự chứng minh vị trí ngoài đời là
chính xác; muốn kiểm tra điều đó phải so với tọa độ khảo sát độc lập của cùng một điểm.

Chuyển **114,395 điểm** của frame 6 theo chuỗi:
LiDAR → IMU → ENU tại OXTS → ECEF. ECEF sử dụng **EPSG:4978**, đơn vị mét.
Tọa độ ECEF là tọa độ tuyệt đối gốc tâm Trái Đất; hình bên dưới trừ một tâm hiển thị
để dễ đọc, còn CSV/PCD giữ nguyên tọa độ tuyệt đối.

| Point index trong BIN (từ 0) | ECEF X (m) | ECEF Y (m) | ECEF Z (m) |
|---:|---:|---:|---:|
| 0 | 4145653.916764 | 607777.811098 | 4792857.799786 |
| 28598 | 4145620.699200 | 607716.489008 | 4792889.901102 |
| 57197 | 4145616.648915 | 607712.073003 | 4792892.844502 |
| 85795 | 4145610.519631 | 607717.052553 | 4792896.692624 |
| 114394 | 4145617.602457 | 607721.865454 | 4792889.922161 |

![Point cloud ECEF](ECEF/ecef_cloud.png)

Tệp: [CSV](ECEF/frame_0000000006.csv), [ECEF PCD](ECEF/frame_0000000006_ecef.pcd),
[UTM PCD](ECEF/frame_0000000006_utm.pcd), [metadata và sai số](ECEF/frame_0000000006.json).
PCD dùng ASCII float64 XYZ; khi mở tọa độ lớn trong CloudCompare nên chấp nhận Global Shift.

## 3. Kết quả ghép point cloud nhiều frame

**Mục đích.** Phần này kiểm tra xem pose của từng frame có bù đúng chuyển động của
xe hay không. Nếu phép biến đổi đúng, tường, mặt đường và các vật thể tĩnh quan sát
ở nhiều thời điểm sẽ nằm gần nhau trong cùng hệ World. Nếu dùng sai pose, sai chiều
extrinsic hoặc sai gốc tọa độ, các cấu trúc này sẽ tách thành nhiều bản sao hoặc bị
kéo thành vệt.

Hình so sánh trái/phải là kiểm tra trực quan trực tiếp: hình trái cố tình đặt mọi
scan tại pose tham chiếu để cho thấy hậu quả khi bỏ qua ego-motion; hình phải áp dụng
pose OXTS riêng cho từng scan. Thống kê nearest-neighbor bổ sung một phép đo định
lượng về độ chồng lấn. Số điểm sau voxel cho biết mức giảm dữ liệu và khả năng tạo
một cloud đủ nhẹ để lưu trữ hoặc hiển thị, chứ không đo độ chính xác định vị.

Ghép các frame **0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21** bằng tư thế OXTS tương ứng,
sau đó đưa vào cùng ECEF, cùng gốc World và cùng vùng UTM.

- Tổng số điểm đầu vào: **2,603,914**.
- Số điểm xuất: **124,637**; voxel **0.2 m**.
- Bộ lọc giữ một điểm gốc trong mỗi voxel, bảo toàn `frame_id`, `point_index`, intensity trong NPZ.
- Chiều dài quỹ đạo IMU qua các frame đã chọn (3D): **17.001 m**.
- Dùng `--voxel-size 0` nếu cần giữ toàn bộ điểm.

![So sánh ghép point cloud](MultiFrame/merged_comparison.png)

Trái: mọi scan dùng tư thế của frame gốc. Phải: mỗi scan dùng tư thế OXTS riêng.
Hai hình dùng cùng tập điểm, cùng hệ trục và cùng giới hạn hiển thị; màu biểu thị frame.
Hình lấy mẫu để dễ xem, không thay đổi dữ liệu xuất. Không áp dụng ICP hoặc tối ưu quỹ đạo.

Kiểm tra khoảng cách nearest-neighbor đối xứng trên tối đa 10.000 điểm mỗi chiều cho
mỗi cặp frame liên tiếp đã chọn, trước khi lọc voxel:

- Median khoảng cách theo từng cặp nằm trong **0.0427–0.0603 m**.
- P95 theo từng cặp nằm trong **0.3223–0.6444 m**.
- Tỷ lệ mẫu có điểm gần nhất trong 1 m: **97.51–99.88%**.

![Khoảng cách cloud giữa các frame](MultiFrame/overlap_metrics.png)

Khoảng cách này chịu ảnh hưởng của mật độ lấy mẫu, vùng không chồng lấn, che khuất,
vật thể chuyển động và sai số pose. Không diễn giải thành RMSE định vị tuyệt đối.

Tệp: [World PCD](MultiFrame/merged_world.pcd), [UTM PCD](MultiFrame/merged_utm.pcd),
[ECEF PCD](MultiFrame/merged_ecef.pcd), [NPZ có nguồn gốc từng điểm](MultiFrame/merged_points.npz),
[metadata và số liệu từng cặp frame](MultiFrame/merged.json).

## 4. Kết quả Camera Pixel → World / UTM

**Mục đích.** Phần này kiểm tra chiều ngược của pipeline camera: từ một quan sát 2D
trên ảnh, kết hợp với depth, khôi phục điểm 3D LiDAR rồi đặt điểm đó vào World, UTM
và ECEF. Đây là bước cần thiết khi muốn gán vị trí địa lý cho tâm một bounding box,
điểm ảnh của biển báo, góc công trình hoặc một feature được phát hiện trên ảnh.

Năm ví dụ dùng chính điểm LiDAR đã biết để kiểm tra công thức nghịch đảo, thành phần
tịnh tiến và stereo baseline. Sai số chiếu lại trả lời câu hỏi “điểm khôi phục có
quay về đúng pixel ban đầu không?”, còn sai số khôi phục LiDAR trả lời “tọa độ 3D có
quay về đúng điểm ban đầu không?”. Vì đầu vào và đầu ra dùng cùng calibration, đây
là kiểm tra vòng kín của phần mềm, không phải đo sai số camera–LiDAR ngoài thực tế.

Ảnh đầu vào là ảnh KITTI đã rectified đúng kích thước hiệu chuẩn.
Một pixel riêng lẻ chỉ xác định một tia; để lấy tọa độ 3D cần thêm độ sâu.
Ví dụ dưới dùng **5 pixel tại đúng vị trí chiếu của điểm LiDAR frame 6**,
với độ sâu theo trục quang học từ chính điểm đó, không phải khoảng cách Euclid tới camera.

Với `P_total = P_rect @ R_rect_00 @ T_velo_to_cam = [M | b]`:
`p_lidar = inverse(M) @ (depth * [u, v, 1] - b)`.
Sau đó dùng chuỗi chuyển LiDAR → ECEF → World / UTM. Phép tính giữ cả thành phần
tịnh tiến trong P, bao gồm baseline giữa các camera.

| ID | Pixel (u, v) | Depth (m) | World ENU (m) | UTM E (m) | UTM N (m) |
|---:|---|---:|---|---:|---:|
| 1 | (806.791, 205.491) | 31.276 | (14.708, -35.035, -0.624) | 451751.986 | 5431061.241 |
| 2 | (433.781, 207.094) | 23.151 | (21.382, -20.120, 0.038) | 451758.788 | 5431076.091 |
| 3 | (619.995, 241.992) | 19.202 | (14.298, -20.402, -0.912) | 451751.704 | 5431075.871 |
| 4 | (248.057, 261.943) | 4.140 | (7.297, -6.854, 0.351) | 451744.824 | 5431089.474 |
| 5 | (993.430, 262.221) | 12.200 | (5.149, -18.401, -0.829) | 451742.576 | 5431077.951 |

![Vị trí pixel được chuyển đổi](CameraToWorld/pixel_world.png)

Sai số chiếu lại lớn nhất: **1.172e-13 pixel**.
Sai số khôi phục điểm LiDAR của ví dụ: **4.392e-15 m** (max 3D).
Đây là kiểm tra số học trên cặp pixel/depth sinh từ LiDAR, chưa phải kiểm định độc lập
độ chính xác định vị một vật thể được chọn bằng mắt.

Khi nhập pixel tùy chọn, chương trình có thể lấy depth từ mẫu LiDAR gần nhất trong
ngưỡng mặc định 2 pixel. Depth khi đó là ước lượng theo giả định cùng bề mặt; chương
trình báo lỗi nếu không có mẫu trong ngưỡng. Z-buffer giữ mẫu gần nhất trong mỗi
ô pixel, nhưng không giải quyết hoàn toàn che khuất giữa camera và LiDAR.

Tệp: [bảng đầy đủ gồm ECEF, latitude/longitude, height](CameraToWorld/pixel_world.csv),
[metadata, nguồn depth và point index](CameraToWorld/pixel_world.json).

## 5. Giới hạn và cách chạy lại

**Mục đích.** Phần này xác định phạm vi có thể tin cậy của các kết quả. Nó phân biệt
lỗi số học của phần mềm với sai số cảm biến và môi trường mà các phép kiểm tra vòng
kín không thể nhìn thấy. Khi cần báo cáo độ chính xác thực địa, phải bổ sung ground
truth, deskew theo thời gian, đánh giá calibration/synchronization và loại vật thể động.

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
