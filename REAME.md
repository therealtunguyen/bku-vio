# BKU-VIO — Visual-Inertial Odometry (CO3107)

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
  --security-opt seccomp=unconfined \
  --shm-size=512m \
  -v <đường_dẫn_tới_ros_ws>:/home/ubuntu/ros_ws \
  ghcr.io/tiryoh/ros2-desktop-vnc:humble
```

> ⚠️ Thay `<đường_dẫn_tới_ros_ws>` bằng **absolute path** tới thư mục `ros_ws` trên máy bạn.
> Ví dụ: `-v /Users/twang/HCMUT/bku-vio/ros_ws:/home/ubuntu/ros_ws`

Truy cập VNC: **http://localhost:6080**

> **Tip:** Nếu cần mount thêm dataset vào container:
> ```bash
> docker run -p 6080:80 \
>   --security-opt seccomp=unconfined \
>   --shm-size=512m \
>   -v <đường_dẫn_tới_ros_ws>:/home/ubuntu/ros_ws \
>   -v <đường_dẫn_tới_dataset>:/home/ubuntu/dataset \
>   ghcr.io/tiryoh/ros2-desktop-vnc:humble
> ```

## 3. Build package (trong container)

Mở terminal trong VNC rồi chạy:

```bash
cd ~/ros_ws
colcon build --packages-select vio_pkg
source install/setup.bash
```

> Mỗi lần mở terminal mới đều phải `source install/setup.bash`.

## 4. Chạy ROS 2 Bag (EuRoC dataset)

```bash
# Terminal 1 — play rosbag
ros2 bag play ~/dataset/V1_01_easy/

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
cd ~/ros_ws && source install/setup.bash
ros2 run vio_pkg py_sub
```

## 6. Test bag_reader

```bash
# Terminal 2
cd ~/ros_ws && source install/setup.bash
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
