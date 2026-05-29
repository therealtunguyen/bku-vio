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
- **Phase 4 (MSCKF Backend - File `backend/msckf_updater.py` & `state_server.py`):** Viết kỹ thuật Gauss-Newton Triangulation để cố định các điểm 3D ngoài đời thực mường tượng được từ Camera. Tối ưu hóa Error-state Jacobian thông qua Null-Space QR Decomposition, kết thúc bằng quá trình Update dữ liệu vào State Mạch. Phase này đã hoàn thành cho EuRoC `V1_01_easy`; hệ thống đạt mục tiêu ATE RMSE < 1.0 m khi đánh giá bằng `evo` với SE(3) alignment, không scale correction.
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

Dataset HCMUT/D455 hiện dùng được như smoke test, chưa dùng làm accuracy result:

- RGB-only bags: dùng cho frontend/demo, không đủ IMU để chạy VIO đầy đủ.
- `vio_hcmut_dataset`: có color image và IMU, chạy được full VIO smoke test với
  intrinsics D455 và extrinsics lấy từ `tf_static`, nhưng chưa nên dùng làm
  metric-accuracy result.

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
`camera_imu_frame -> camera_color_optical_frame`.

Transform D455/HCMUT đang dùng cho smoke test:

```text
R =
[[ 0.999996654005,  0.001598738420, -0.002033719299],
 [-0.001599047141,  0.999998710246, -0.000150184164],
 [ 0.002033476571,  0.000153435674,  0.999997920713]]
t = [0.028793809935, 0.007352355558, 0.015779949041]
```

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

## 6. Deliverables
- Skeleton Data Flow Pipeline Python bảo mật tránh rò rỉ RAM rớt FPS.
- Cấu trúc Design phân tán rõ ràng (Modularity).
- Metric Output đánh giá được RMSE (Root Mean Square Error) Track.
- Liên kết Visualizer thời gian thực qua RViz2.
