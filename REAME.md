# BKU-VIO — Visual-Inertial Odometry (CO3107)

> **Kết quả & số liệu** → bắt đầu ở [`docs/results/TASK.md`](docs/results/TASK.md)
> (index của tất cả task tuần 2). Các file con:
> [`table.md`](docs/results/table.md) (bảng số liệu — single source of truth),
> [`eval_commands.md`](docs/results/eval_commands.md) (lệnh chạy lại),
> [`euroc_baseline.md`](docs/results/euroc_baseline.md),
> [`hcmut_smoke.md`](docs/results/hcmut_smoke.md),
> [`trajectory_plot.md`](docs/results/trajectory_plot.md),
> [`week2_checklist.md`](docs/results/week2_checklist.md).
> Mọi accuracy number trong báo cáo phải có một dòng `confirmed` trong `table.md`.
> HCMUT hiện vẫn là `smoke-test only` — không dùng làm accuracy result.

## 0. Trạng thái hiện tại

- M4 MSCKF measurement update đã hoàn thành trên EuRoC `V1_01_easy`.
- Baseline EuRoC full-run đã xác nhận trước đó vẫn được giữ làm mốc đối chiếu:
  ATE RMSE `0.134976 m`, `2891` odometry poses, không có queue-drop warnings,
  và không có worker crashes.
- Tuy nhiên, runner M6 hiện tại chưa khép lại full-run EuRoC ở trạng thái
  acceptance-ready. Profile automation bảo thủ mới nhất đã được hạ còn
  `bag_rate=0.15` và `image_processing_width=640`; slice xác minh ngắn chạy
  ổn định, nhưng full scripted run vẫn có thể phát sinh `Image queue full` và
  `Skipping visual update after large forward image gap`.
- Project scope hiện là pure VIO/MSCKF; không còn nhánh ArUco trong kế hoạch.
- Launch mặc định vẫn chạy EuRoC với calibration EuRoC:
  - Intrinsics/distortion mặc định nằm trong `backend/msckf_updater.py`.
  - IMU-camera extrinsics mặc định nằm trong `backend/state_server.py`.
  - QoS mặc định là `input_qos_reliability:=reliable`.
- Không dùng thông số D455/HCMUT cho EuRoC trừ khi đang test riêng dataset đó.
- Dataset HCMUT/D455 hiện dùng cho smoke, calibration, và replay debugging.
  Extrinsics thật của thiết bị đã có trong `tf_static_extrinsics.txt`; phần còn
  thiếu để coi đây là evaluation path chính thức là khóa M6 metrics/baselines,
  không phải đi tìm extrinsics nữa.

## 1. Yêu cầu

| Tool | Ghi chú |
|------|---------|
| Docker | Đảm bảo Docker daemon đang chạy |
| EuRoC dataset | Tải và giải nén vào `dataset/V1_01_easy/` |

**Tải dataset:**

1. Tải file từ [Google Drive](https://drive.google.com/file/d/1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh/view?usp=drive_link)
2. Giải nén và đặt vào thư mục `dataset/V1_01_easy/`

## 2. Chạy ROS 2 Container

Mount `ros_ws` vào container để code được đồng bộ realtime:

```bash
docker run -p 6080:80 \
  --name vio-ros2 \
  --security-opt seccomp=unconfined \
  --shm-size=512m \
  -v <path/to/bku-vio>:/home/ubuntu/VIO \
  ghcr.io/tiryoh/ros2-desktop-vnc:humble
```

> ⚠️ Thay `<path/to/bku-vio>` bằng **absolute path** trên máy bạn.
> Ví dụ (Linux/WSL): `-v /home/yourname/bku-vio:/home/ubuntu/VIO`

Nếu container đã tồn tại:

```bash
docker start vio-ros2
```

Truy cập VNC: **http://localhost:6080**

## 3. Build package (trong container)

Mở terminal trong VNC rồi chạy:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws
colcon build --packages-select vio_pkg
source install/setup.bash
```

> Mỗi lần mở terminal mới đều phải `source install/setup.bash`.

## 4. Chạy ROS 2 Bag (EuRoC dataset)

```bash
# Terminal 1 — play rosbag thủ công nếu không dùng launch file
source /opt/ros/humble/setup.bash
ros2 bag play /home/ubuntu/VIO/dataset/V1_01_easy --clock --rate 0.2

# Kiểm tra các topic đang publish
ros2 topic list
```

Các topic chính của EuRoC:
| Topic | Message Type |
|-------|-------------|
| `/imu0` | `sensor_msgs/msg/Imu` |
| `/cam0/image_raw` | `sensor_msgs/msg/Image` |
| `/vicon/firefly_sbx/firefly_sbx` | `geometry_msgs/msg/TransformStamped` |

## 5. Test subscriber (py_sub)

```bash
# Terminal 2
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws && source install/setup.bash
ros2 run vio_pkg py_sub
```

## 6. Test bag_reader

```bash
# Terminal 2
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws && source install/setup.bash
ros2 run vio_pkg bag_reader
```

Node `bag_reader` sẽ subscribe cả 3 topic (IMU, Camera, Vicon) và in log mỗi N messages.

## 7. Kiểm tra nhanh (không cần rosbag)

Publish thủ công 1 message IMU để test:

```bash
ros2 topic pub /imu0 sensor_msgs/msg/Imu "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'imu'},
  linear_acceleration: {x: 0.0, y: 0.0, z: 9.81},
  angular_velocity: {x: 0.0, y: 0.0, z: 0.0}
}" --once
```
## 8. Chạy toàn bộ hệ thống (Quick Start)

Khởi động Docker container (thay `<path/to/ros_ws>` và `<path/to/dataset>` bằng đường dẫn thực trên máy bạn):

```bash
docker run -p 6080:80 \
  --name vio-ros2 \
  --security-opt seccomp=unconfined \
  --shm-size=512m \
  -v <path/to/bku-vio>:/home/ubuntu/VIO \
  ghcr.io/tiryoh/ros2-desktop-vnc:humble
```

Mở terminal trong VNC, build package và chạy launch file:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws
colcon build --packages-select vio_pkg
source install/setup.bash

# Dataset mặc định: /home/ubuntu/VIO/dataset/V1_01_easy
ros2 launch vio_pkg vio_system.launch.py bag_rate:=0.2

# hoặc chỉ định dataset_dir cụ thể:
ros2 launch vio_pkg vio_system.launch.py dataset_dir:=/home/ubuntu/VIO/dataset bag_rate:=0.2
```

Profile M6 automation bảo thủ hiện tại cho EuRoC:

```bash
ros2 launch vio_pkg vio_system.launch.py \
  dataset_dir:=/home/ubuntu/VIO/dataset \
  bag_rate:=0.15 \
  image_processing_width:=640 \
  enable_rviz:=false \
  publish_debug_image:=false \
  log_tracked_frames:=false
```

Trong log của node, với EuRoC bạn nên thấy:

```text
Camera calibration: fx=458.654000, fy=457.296000, ...
Input sensor QoS reliability: reliable
```

Nếu trajectory EuRoC bị diverge sau một lúc, kiểm tra trước:

- Đã rebuild và `source install/setup.bash` sau khi pull code mới chưa.
- Không còn process `ros2 launch`, `ros2 bag play`, hoặc `vio_system_node` cũ.
- Log vẫn là `Input sensor QoS reliability: reliable`.

## 9. Test và build check

Chạy trong container:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws

python3 src/vio_pkg/test/test_backend_consistency.py
python3 src/vio_pkg/test/test_runtime_safeguards.py
colcon build --packages-select vio_pkg
```

## 10. Chạy smoke test cho HCMUT/D455

HCMUT/D455 chưa phải accuracy run chính thức, nhưng đã có một smoke-test path
ổn định hơn với intrinsics D455 và extrinsics lấy đúng từ
`camera_imu_optical_frame -> camera_color_optical_frame` trong
`tf_static_extrinsics.txt`.

Color camera intrinsics từ `/camera/camera/color/camera_info`:

```text
fx=646.33728
fy=645.676147
cx=643.358276
cy=362.999176
D=[-0.05594548583030701, 0.06458555161952972, -0.0002526374883018434, 0.0008183500613085926, -0.021141313016414642]
```

Extrinsics hiện dùng, trích từ `tf_static_extrinsics.txt`:

```text
R =
[[ 0.999996654005,  0.001598738436, -0.002033719319],
 [-0.001599047157,  0.999998710246, -0.000150184165],
 [ 0.002033476591,  0.000153435676,  0.999997920713]]
t = [0.028793809935, 0.007352355558, 0.015779949041]
```

Milestone status hiện tại:
- M5 pure-VIO hardening: hoàn tất theo scope local docs
- M6: đã bắt đầu phần evaluation scaffolding (`tools/run_m6_eval.py`, replay
  harness HCMUT/EuRoC), nhưng chưa khóa full-run EuRoC automation path,
  chưa khóa systematic metrics/reporting, và chưa bắt đầu nhánh RPi 5

Có thể dùng launch preset:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws
source install/setup.bash

ros2 launch vio_pkg vio_hcmut.launch.py
```

Hoặc chạy tay để thấy đầy đủ params:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws
source install/setup.bash

ros2 run vio_pkg vio_system_node --ros-args \
  -p use_sim_time:=true \
  -p imu_init_sample_count:=120 \
  -p input_qos_reliability:=best_effort \
  -p image_processing_width:=752 \
  -p image_queue_size:=50 \
  -p runtime_diagnostics_enabled:=true \
  -p diagnostics_log_every_n_frames:=10 \
  -p publish_debug_image:=false \
  -p log_tracked_frames:=false \
  -p max_imu_dt:=0.05 \
  -p max_batch_dx_bias_norm:=0.06 \
  -p min_triangulation_parallax_deg:=2.0 \
  -p max_imu_init_gap:=0.2 \
  -p max_frame_timestamp_gap:=0.25 \
  -p camera_fx:=646.33728 \
  -p camera_fy:=645.676147 \
  -p camera_cx:=643.358276 \
  -p camera_cy:=362.999176 \
  -p camera_distortion:="[-0.05594548583030701, 0.06458555161952972, -0.0002526374883018434, 0.0008183500613085926, -0.021141313016414642]" \
  -p camera_extrinsics_convention:=camera_in_imu \
  -p camera_R_IC:="[0.999996654005, 0.001598738436, -0.002033719319, -0.001599047157, 0.999998710246, -0.000150184165, 0.002033476591, 0.000153435676, 0.999997920713]" \
  -p camera_t_IC:="[0.028793809935, 0.007352355558, 0.015779949041]"
```

Trong terminal khác:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/ubuntu/VIO/dataset/vio_hcmut_dataset --clock --rate 1.0 \
  --remap /camera/camera/color/image_raw:=/cam0/image_raw /camera/camera/imu:=/imu0
```

Lưu ý: bag này publish IMU ở `camera_imu_optical_frame`, không phải
`camera_imu_frame`. Dùng nhầm frame variant cho extrinsics sẽ làm triangulation
và MSCKF update xấu đi rõ rệt.

## Cấu trúc thư mục

```
bku-vio/
├── dataset/
│   └── V1_01_easy/          # EuRoC rosbag
├── ros_ws/
│   └── src/
│       └── vio_pkg/
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           └── vio_pkg/
│               ├── __init__.py
│               ├── py_sub.py        # Subscriber đơn giản
│               └── bag_reader.py    # Đọc IMU + Camera + Vicon
└── REAME.md
```
# MISC NOTE
pip install "numpy<2"
