# Kiến Trúc Hệ Thống bku-vio (MSCKF VIO)

## 1. Chức năng (Functionalities)
Hệ thống MSCKF VIO (Multi-State Constraint Kalman Filter Visual Inertial Odometry) có chức năng:
- Lấy thông tin **Hình ảnh (Image)** từ Camera và thông tin **Gia tốc/Vận tốc góc (IMU)**.
- Kết hợp (sensor fusion) giữa hai nguồn tín hiệu này theo thời gian thực để tạo ra **Quỹ đạo (Trajectory) 3D** của robot/camera với độ chính xác cao.
- **Output:** `/vio/odometry` (6-DoF Pose hiện tại), `/vio/path` (Lịch sử đường đi).

## 2. Định hướng Implement (Implementation Roadmap)
Tự implement lõi (core) toán học và Computer Vision vào khung (skeleton) phân tầng đã thiết kế sẵn. Lộ trình thực hiện như sau:

- **Phase 1 (Infrastructure & Threads):** Refactor hệ thống về chuẩn Multi-threading, đảm bảo 0% tỉ lệ drop gói tin IMU hoặc trễ nhịp ảnh ROS. (Đã hoàn thiện framework tại `vio_node.py` và `utils/`).
- **Phase 2 (Visual Frontend - Thư mục `frontend/`):** 
  - `detectors.py`: Viết thuật toán (ví dụ: Harris corner).
  - `trackers.py`: Cài đặt theo dõi quang học (`cv2.calcOpticalFlowPyrLK`).
  - `feature_manager.py`: Lọc điểm nhiễu (RANSAC) và quản lý ID.
  - -> Output: Sinh ra tập điểm theo dõi bền bỉ để chuyển đi (`MatureFeatures`).
- **Phase 3 (IMU Propagation - File `backend/propagator.py`):** Cài đặt tích phân Kinematics RK4 để nhích (predict) Trạng thái Tương lai của Vị trí, Vận tốc, Sai số (Bias) và sinh ma trận hiệp phương sai Covariance.
- **Phase 4 (MSCKF Backend - File `backend/msckf_updater.py` & `state_server.py`):** Viết kỹ thuật Gauss-Newton Triangulation để cố định các điểm 3D ngoài đời thực mường tượng được từ Camera. Tối ưu hóa Error-state Jacobian thông qua Null-Space QR Decomposition, kết thúc bằng quá trình Update dữ liệu vào State Mạch. Phase này đã hoàn thành cho EuRoC `V1_01_easy`; hệ thống đạt mục tiêu ATE RMSE < 1.0 m khi đánh giá bằng `evo` với SE(3) alignment, không scale correction. Baseline full-run đã xác nhận trước đó trên EuRoC ghi nhận ATE RMSE `0.134976 m` với `2891` odometry poses, không có queue-drop warnings, và không có worker crashes, phù hợp với mốc full-run `0.132867 m` đã ghi nhận trước đó. Đây vẫn là mốc tham chiếu, nhưng không nên gọi là “kết quả runner hiện tại” vì path M6 automation đang được siết lại riêng.
- **Phase 5 (Pure VIO Hardening):** Khóa scope về pure VIO/MSCKF. Trọng tâm là giữ baseline EuRoC M4 reproducible, tăng độ ổn định runtime, và debug path HCMUT/D455 như smoke test mà không tạo estimator mode mới.

## 3. Cách Thiết Kế (Architecture & Design Pattern)

Hệ thống được chia nhánh ra **3 Lớp (Layers)** chuyên biệt tuân theo chuẩn **Clean Architecture** (SOLID principles), gói gọn trong **1 ROS 2 Node** vận hành chéo qua nhiều Luồng (Threading).

### Phân Rã Thư Mục & Vai Trò:
```text
vio_pkg/
├── frontend/ (Xử lý Ảnh & Computer Vision)
│   ├── feature_manager.py (Nhạc trưởng điều phối Detector/Tracker)
│   ├── interfaces.py      (Strategy Pattern - Abstract methods)
│   ├── detectors.py       (Thực thi tính toán ma trận ảnh gốc)
│   └── trackers.py        (Thực thi dòng quang học)
├── backend/  (Xử lý EKF Filter & Toán Học)
│   ├── msckf_updater.py   (Engine Kalman Update lõi)
│   ├── propagator.py      (Bộ dẫn đường quán tính IMU Toán Matrix)
│   └── state_server.py    (Bộ nhớ trượt Sliding Window 20 Frames)
├── utils/    (Cấu Trúc Tĩnh Data Definition)
│   └── common.py          (Các Dataclass State/Pose chung)
└── vio_node.py (Application Root)
```

### Luồng Dữ Liệu (Threading Producer-Consumer model)
- **Thread 1: ROS Callbacks (Producers):** Chiếm dụng Thread hệ thống cực thấp. Đón tín hiệu IMU và ảnh, đưa vào buffer/queue theo timestamp.
- **Thread 2: Ordered VIO Worker:** Lấy từng ảnh theo thứ tự thời gian, ghép với IMU tương ứng, propagate state, thêm camera clone, chạy visual frontend, chạy MSCKF update, rồi publish odometry/path/point cloud.
- **Backend MSCKF:** Hiện chạy trong ordered worker để update state theo đúng thứ tự timestamp. Không tạo batch update phá state nếu update quá lớn hoặc điều kiện số quá xấu.

## 4. Calibration, QoS, và dataset

### EuRoC mặc định

Launch mặc định vẫn dùng EuRoC `V1_01_easy`:

- `/home/ubuntu/VIO/dataset/V1_01_easy`
- Intrinsics/distortion EuRoC cam0 trong `backend/msckf_updater.py`
- IMU-camera extrinsics EuRoC trong `backend/state_server.py`
- Input QoS mặc định: `input_qos_reliability:=reliable`

Giữ `reliable` cho EuRoC. Khi từng chuyển global subscription sang
`BEST_EFFORT`, trajectory có thể chạy được lúc đầu nhưng diverge sau một thời
gian vì có khả năng mất IMU/image messages.

### Camera calibration override

`MSCKFUpdater` có API:

```python
set_camera_calibration(fx, fy, cx, cy, distortion_coefficients)
```

`VIOSystemNode` expose các ROS params tương ứng:

```text
camera_fx
camera_fy
camera_cx
camera_cy
camera_distortion
```

Các params này chỉ đổi intrinsics/distortion. Khi đổi camera/dataset, vẫn phải
đổi đúng IMU-camera extrinsics trong `StateServer`; nếu không, VIO metric sẽ
dễ diverge dù image tracking vẫn chạy.

### HCMUT / RealSense D455

Dataset HCMUT/D455 hiện dùng được như smoke test, calibration, và replay
debugging; chưa dùng làm metric benchmark chính thức:

- RGB-only bags: dùng cho frontend/demo, không đủ IMU để chạy VIO đầy đủ.
- `vio_hcmut_dataset`: có color image và IMU, chạy được full VIO smoke test với
  intrinsics D455 và extrinsics lấy từ `tf_static_extrinsics.txt`, nhưng chưa
  nên dùng làm metric-accuracy result trước khi khóa quy trình đánh giá M6.

D455 color intrinsics từ rosbag:

```text
fx=646.33728
fy=645.676147
cx=643.358276
cy=362.999176
D=[-0.05594548583030701, 0.06458555161952972, -0.0002526374883018434, 0.0008183500613085926, -0.021141313016414642]
```

Với D455/HCMUT, dùng `input_qos_reliability:=best_effort` nếu bag/device publish
sensor topics bằng best-effort QoS. Quan trọng: extrinsics phải khớp đúng
message frame trong bag. Bag này publish image ở `camera_color_optical_frame`
và IMU ở `camera_imu_optical_frame`, nên transform đúng là
`camera_imu_optical_frame -> camera_color_optical_frame`, không phải
`camera_imu_frame -> camera_color_optical_frame`. Nguồn sự thật hiện tại trong
workspace là file `/home/tyler/Desktop/bku-vio/tf_static_extrinsics.txt`.

Transform D455/HCMUT đang dùng cho smoke test, trích từ
`tf_static_extrinsics.txt`:

```text
R =
[[ 0.999996654005,  0.001598738436, -0.002033719319],
 [-0.001599047157,  0.999998710246, -0.000150184165],
 [ 0.002033476591,  0.000153435676,  0.999997920713]]
t = [0.028793809935, 0.007352355558, 0.015779949041]
```

### HCMUT D455 smoke status

- Runner: `tools/run_m6_eval.py --case hcmut_d455_smoke`
- Verified on `2026-05-30` với results root:
  - `/home/ubuntu/VIO/results/hcmut_smoke_stabilized_task5_20260530_001`
- Evidence path:
  - `/home/ubuntu/VIO/results/hcmut_smoke_stabilized_task5_20260530_001/hcmut_d455_smoke_summary.json`
- Verified smoke facts từ summary JSON:
  - `smoke_check.ok == true`
  - `worker_crash == false`
  - `frame_gap_resets == 0`
  - `backward_jump_resets == 0`
  - `forward_gap_resets == 0`
  - `accepted_updates == 5`
  - `peak_vel_norm == 158.136`
  - `large_image_timestamp_gaps == 0`
  - `imu_init_resets == 0`
  - `image_queue_full == 0`
- Runtime facts từ cùng smoke run:
  - `launch_returncode == 0`
  - `record_returncode == 0`
  - `node_finished == true`
  - `timed_out == false`

Deterministic replay harness cùng ngày vẫn báo `classification=intrinsic_propagation`
và `first_divergence=processed_frame:181, raw_image:201, vel_norm:0.177831`.
Điều này không làm hỏng smoke check hiện tại, nhưng cho thấy path HCMUT vẫn còn
late-run drift cần tiếp tục debug trước khi coi là accuracy-ready.

### Remediation update: HCMUT collapse around frame 183→184

- **Observed symptom:** Trên HCMUT/D455, trajectory từng sụp đổ đột ngột trong
  cửa sổ frame `183→184` sau giai đoạn chạy ổn định ban đầu.
- **Root cause:** Điều kiện số của bước MSCKF update trở nên xấu; khi nullspace
  projection thiếu ổn định theo hạng và covariance co quá mức, filter trở nên
  over-contractive và mất ổn định.
- **Implemented fix:** Bổ sung pipeline ổn định update gồm
  rank-aware nullspace, shrink guard cho covariance, stable linear solve kèm
  jitter nhỏ khi cần, và PSD guard sau update để giữ covariance hợp lệ.
- **Validation outcome:** Không còn collapse tại cửa sổ `183→184` trên path
  HCMUT smoke; regression tests pass; EuRoC harness tiếp tục ổn định.

### Milestone M6 status

- `tools/run_m6_eval.py` và replay harnesses đã tạo phần scaffolding đầu tiên
  cho M6.
- M6 hiện ở trạng thái **evaluation in progress** trong thực tế:
  systematic evaluation artifacts đang hình thành, nhưng chưa khóa thành bộ
  metric/baseline chính thức và chưa bắt đầu nhánh Raspberry Pi 5.
- EuRoC M6 automation profile bảo thủ hiện tại:
  - `bag_rate=0.15`
  - `image_processing_width=640`
  - `publish_debug_image=false`
  - `log_tracked_frames=false`
- Live verification hiện tại cho profile này:
  - direct-launch slice `300` frames chạy sạch, không queue-full, không
    forward-gap skip
  - full scripted `run_m6_eval.py --case euroc_v101_easy` vẫn có late-run
    `Image queue full` và `Skipping visual update after large forward image gap`
    nên chưa acceptance-ready

## 5. Verification

Chạy trong Docker container:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws

python3 src/vio_pkg/test/test_backend_consistency.py
python3 src/vio_pkg/test/test_runtime_safeguards.py
colcon build --packages-select vio_pkg
```

Full EuRoC run:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws
source install/setup.bash
ros2 launch vio_pkg vio_system.launch.py bag_rate:=0.2
```

Expected EuRoC runtime log includes:

```text
Input sensor QoS reliability: reliable
```

M6 automation verification đang dùng profile EuRoC bảo thủ riêng:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws
source install/setup.bash
python3 /home/ubuntu/VIO/tools/run_m6_eval.py --case euroc_v101_easy
```

Trạng thái verification EuRoC hiện tại nên hiểu như sau:

- **Historical full-run baseline preserved:** ATE RMSE `0.134976 m`,
  `2891` odometry poses, `0` queue-drop warnings, `0` worker crashes
- **Current M6 runner profile:** direct-launch slice ổn định ở `bag_rate=0.15`
  + `image_processing_width=640`
- **Current full scripted M6 case:** vẫn có late-run `Image queue full` và
  `Skipping visual update after large forward image gap: dt_img=0.1000s`

## 6. Deliverables
- Skeleton Data Flow Pipeline Python bảo mật tránh rò rỉ RAM rớt FPS.
- Cấu trúc Design phân tán rõ ràng (Modularity).
- Metric Output đánh giá được RMSE (Root Mean Square Error) Track.
- Liên kết Visualizer thời gian thực qua RViz2.
