# BKU-VIO — Visual-Inertial Odometry (CO3107)

> **Cần con số ATE / RPE hoặc trả lời câu hỏi "kết quả có tốt hơn không?"** →
> đọc [`docs/results/RESULTS.md`](docs/results/RESULTS.md) (single source of
> truth) và [`docs/results/WEEK2_CHECKLIST.md`](docs/results/WEEK2_CHECKLIST.md)
> trước khi chạy bất kỳ run mới nào. Mọi accuracy number trong báo cáo phải
> có một dòng tương ứng trong `RESULTS.md` ở trạng thái `confirmed`. HCMUT
> hiện vẫn là `smoke-test only` — không dùng làm accuracy result.

## 0. Trạng thái hiện tại

- M4 MSCKF measurement update đã hoàn thành trên EuRoC `V1_01_easy`.
- Launch mặc định vẫn chạy EuRoC với calibration EuRoC:
  - Intrinsics/distortion mặc định nằm trong `backend/msckf_updater.py`.
  - IMU-camera extrinsics mặc định nằm trong `backend/state_server.py`.
  - QoS mặc định là `input_qos_reliability:=reliable`.
- Không dùng thông số D455/HCMUT cho EuRoC trừ khi đang test riêng dataset đó.
- Dataset HCMUT/D455 hiện chỉ dùng smoke test. Muốn đánh giá accuracy cần có
  camera-IMU extrinsics thật của thiết bị.

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

HCMUT/D455 không phải accuracy run hiện tại vì thiếu camera-IMU extrinsics.
Chỉ dùng để kiểm tra node có nhận image/IMU và chạy được.

Color camera intrinsics từ `/camera/camera/color/camera_info`:

```text
fx=646.33728
fy=645.676147
cx=643.358276
cy=362.999176
D=[-0.05594548583030701, 0.06458555161952972, -0.0002526374883018434, 0.0008183500613085926, -0.021141313016414642]
```

Khi chạy HCMUT/D455, dùng `input_qos_reliability:=best_effort` và remap topic:

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws
source install/setup.bash

ros2 run vio_pkg vio_system_node --ros-args \
  -p use_sim_time:=true \
  -p input_qos_reliability:=best_effort \
  -p camera_fx:=646.33728 \
  -p camera_fy:=645.676147 \
  -p camera_cx:=643.358276 \
  -p camera_cy:=362.999176 \
  -p camera_distortion:="[-0.05594548583030701, 0.06458555161952972, -0.0002526374883018434, 0.0008183500613085926, -0.021141313016414642]"
```

Trong terminal khác:

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/ubuntu/VIO/dataset/vio_hcmut_dataset --clock --rate 1.0 \
  --remap /camera/camera/color/image_raw:=/cam0/image_raw /camera/camera/imu:=/imu0
```

Để dùng HCMUT/D455 làm kết quả metric, cần lấy transform thật giữa color camera
optical frame và IMU frame từ RealSense/librealsense hoặc ROS `/tf_static`.

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
