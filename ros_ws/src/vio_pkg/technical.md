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
- **Phase 4 (MSCKF Backend - File `backend/msckf_updater.py` & `state_server.py`):** Viết kỹ thuật Gauss-Newton Triangulation để cố định các điểm 3D ngoài đời thực mường tượng được từ Camera. Tối ưu hóa Error-state Jacobian thông qua Null-Space QR Decomposition, kết thúc bằng quá trình Update dữ liệu vào State Mạch.

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
- **Thread 1: ROS Callbacks (Producers):** Chiếm dụng Thread hệ thống cực thấp. Đón tín hiệu chớp nhoáng thả vào RAM Queues.
- **Thread 2: Visual Frontend (Consumer/Producer):** Tách biệt tài nguyên. Đóng gói Ảnh & IMU sync lại, cày nát CPU qua OpenCV và đẩy các chuỗi ảnh trích xuất xuống Queue 2 cho Backend.
- **Thread 3: MSCKF Backend (Consumer):** Không đụng chạm gì Image Processing. Yên tâm thực thi đại số tuyến tính ma trận khổng lồ.

## 4. Deliverables
- Skeleton Data Flow Pipeline Python bảo mật tránh rò rỉ RAM rớt FPS.
- Cấu trúc Design phân tán rõ ràng (Modularity).
- Metric Output đánh giá được RMSE (Root Mean Square Error) Track.
- Liên kết Visualizer thời gian thực qua RViz2.
