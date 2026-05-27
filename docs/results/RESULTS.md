# BKU-VIO — Bảng tracking kết quả

**Mục đích.** Đây là **single source of truth** cho mọi con số ATE / RPE của
team. Mỗi dòng phải có lệnh đầy đủ và thư mục output đi kèm; nếu thiếu một
trong hai thì con số đó **không được dùng trong báo cáo / paper**.

Quy ước trạng thái:

- `confirmed` — đã chạy lại được, ATE / RPE đo trên ground-truth thật, có thể trích dẫn.
- `smoke-test` — chỉ kiểm tra pipeline chạy được, **không phải accuracy result**.
- `pending` — đã có lệnh + thư mục output dự kiến, chưa chạy hoặc đang chạy.
- `failed` — chạy nhưng diverge / crash; giữ lại để khỏi lặp lại sai lầm.

> Toàn bộ đường dẫn trong bảng là **đường dẫn bên trong container** ROS 2
> (`/home/ubuntu/VIO/...`). Trên host, prefix tương ứng là đường mount đã
> chọn khi `docker run -v <host_path>:/home/ubuntu/VIO`.

---

## 1. Bảng kết quả

| # | Dataset | Run command (rút gọn — xem chi tiết ở §3) | Bag output dir | Eval output dir | ATE [m] (RMSE) | RPE 1 m [m] (RMSE) | Ghi chú | Trạng thái |
|---|---------|--------------------------------------------|----------------|------------------|----------------|--------------------|---------|------------|
| 1 | EuRoC `V1_01_easy` | `ros2 run vio_pkg vio_system_node` + `ros2 bag record /vio/odometry /vio/gt_path` + `ros2 bag play V1_01_easy --rate 0.2` (chi tiết §3.A) | `results/euroc_V1_01_easy/bag` (2 892 `/vio/odometry`, 146 `/vio/gt_path`) | `results/euroc_V1_01_easy/eval` | **0.0905** (mean 0.0813, median 0.0752, max 0.2744, std 0.0397) | **0.0949** (mean 0.0898, median 0.0946, max 0.1481, std 0.0305) | **baseline đã chốt** — commit `e8425eb`, run 2026-05-27. EuRoC calibration mặc định, `imu_init_sample_count=200`, `bag_rate=0.2`. SE(3)-aligned (Umeyama). Plot: [`docs/paper/figures/euroc_V1_01_easy_baseline.png`](../paper/figures/euroc_V1_01_easy_baseline.png). Bất kỳ thay đổi backend nào sau ngày 2026-05-27 phải so sánh với dòng này. | `confirmed` |
| 2 | HCMUT `vio_hcmut_dataset` (D455) | xem §3.B — chạy `vio_system_node` với D455 intrinsics + remap topic + `input_qos_reliability:=best_effort` | `results/hcmut_smoke/bag` | _không tính ATE/RPE_ | **N/A** | **N/A** | **smoke test only**. Lý do: pipeline chạy được (nhận image + IMU + có trajectory), nhưng **calibration camera–IMU extrinsics chưa xác nhận** (đang dùng default EuRoC trong `state_server.py`); trajectory diverge sau vài giây; **không có ground-truth**. **KHÔNG báo accuracy từ dòng này.** Cần TF static thật giữa color optical frame và IMU frame (từ `librealsense` / `/tf_static`) trước khi nâng lên `confirmed`. | `smoke-test` |

> Khi điền số: lấy đúng `rmse` trong `stats.json` của file `ape_aligned.zip` /
> `rpe_1m_aligned.zip` (evo zip → unzip → `stats.json` → key `rmse`). Ghi số
> đến 4 chữ số sau dấu phẩy và đơn vị **mét**.

### Cách cập nhật một dòng

1. Chạy đúng lệnh ở §3 cho dataset tương ứng.
2. Sau khi `evaluate_m4.py` xong, mở 2 file `*.zip` bằng
   `evo_res results/<...>/eval/ape_aligned.zip --no_warnings` (hoặc unzip lấy
   `stats.json`) và copy `rmse` vào ô tương ứng.
3. Đổi `Trạng thái` thành `confirmed` (EuRoC) hoặc giữ `smoke-test` (HCMUT).
4. Commit lại file này.

---

## 2. Quy ước thư mục output

Tất cả run đi vào `results/` ở repo root (đã có trong `.gitignore`, không
commit binary). Cấu trúc bắt buộc cho mỗi run:

```
results/
└── <dataset_tag>/
    ├── bag/                       # ros2 bag record của /vio/odometry + /vio/gt_path
    │   ├── metadata.yaml
    │   └── <bag>_0.db3
    ├── eval/                      # output của tools/evaluate_m4.py
    │   ├── vio_odom.tum
    │   ├── vio_gt_path.tum
    │   ├── ape_aligned.zip
    │   └── rpe_1m_aligned.zip
    ├── plots/                     # output của tools/plot_trajectory.py
    │   └── trajectory_xy.png
    └── run_log.txt                # stdout/stderr của vio_system_node (tee từ launch)
```

`<dataset_tag>` ví dụ: `euroc_V1_01_easy`, `euroc_V1_01_easy_qos_best_effort`,
`hcmut_smoke`. Quy tắc: `<dataset>_<biến_thể>` — biến thể là tham số mà bạn
đổi so với baseline (qos, init flag, noise, …).

---

## 3. Lệnh tái chạy (copy-paste được)

> **Tiền điều kiện chung** (chạy trong container `vio-ros2`):
>
> ```bash
> source /opt/ros/humble/setup.bash
> cd /home/ubuntu/VIO/ros_ws
> colcon build --packages-select vio_pkg
> source install/setup.bash
> mkdir -p /home/ubuntu/VIO/results
> # evo cho đánh giá:
> pip install --user "evo[fastentrypoints]" "numpy<2"
> ```

### 3.A — EuRoC `V1_01_easy` (baseline đã chốt)

```bash
# === Terminal 1: launch VIO + bag player + RViz ===
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws && source install/setup.bash

RUN_TAG=euroc_V1_01_easy
RUN_DIR=/home/ubuntu/VIO/results/${RUN_TAG}
mkdir -p ${RUN_DIR}/bag ${RUN_DIR}/eval ${RUN_DIR}/plots

ros2 launch vio_pkg vio_system.launch.py \
    dataset_dir:=/home/ubuntu/VIO/dataset \
    bag_rate:=0.2 \
    2>&1 | tee ${RUN_DIR}/run_log.txt
```

```bash
# === Terminal 2: record VIO topics song song với launch ở terminal 1 ===
source /opt/ros/humble/setup.bash
RUN_DIR=/home/ubuntu/VIO/results/euroc_V1_01_easy

ros2 bag record \
    --output ${RUN_DIR}/bag \
    --storage sqlite3 \
    /vio/odometry /vio/gt_path
# Ctrl-C sau khi launch ở terminal 1 báo bag-play đã xong.
```

```bash
# === Terminal 3 (sau khi terminal 1 + 2 đã Ctrl-C): export TUM + chạy evo ===
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws && source install/setup.bash
RUN_DIR=/home/ubuntu/VIO/results/euroc_V1_01_easy

python3 /home/ubuntu/VIO/tools/evaluate_m4.py \
    ${RUN_DIR}/bag \
    ${RUN_DIR}/eval

# Đọc RMSE để copy vào bảng §1:
evo_res ${RUN_DIR}/eval/ape_aligned.zip --no_warnings | grep -E "rmse"
evo_res ${RUN_DIR}/eval/rpe_1m_aligned.zip --no_warnings | grep -E "rmse"
```

```bash
# === Terminal 3 (tiếp): vẽ trajectory cho paper ===
python3 /home/ubuntu/VIO/tools/plot_trajectory.py \
    --gt   ${RUN_DIR}/eval/vio_gt_path.tum \
    --vio  ${RUN_DIR}/eval/vio_odom.tum \
    --out  ${RUN_DIR}/plots/trajectory_xy.png \
    --title "EuRoC V1_01_easy — BKU-VIO baseline"
```

> Khi chạy lại để confirm: **lệnh phải giống y hệt** — đổi `RUN_TAG` thành
> `euroc_V1_01_easy_<biến_thể>` nếu đang test cải tiến, để không ghi đè baseline.

### 3.B — HCMUT `vio_hcmut_dataset` (SMOKE TEST — không tính ATE/RPE)

> ⚠️ **Không dùng output của block này làm accuracy result.** Mục đích duy
> nhất: xác nhận node nhận topic, ước lượng pose, không crash.

```bash
# === Terminal 1: VIO node với D455 intrinsics ===
source /opt/ros/humble/setup.bash
cd /home/ubuntu/VIO/ros_ws && source install/setup.bash

RUN_TAG=hcmut_smoke
RUN_DIR=/home/ubuntu/VIO/results/${RUN_TAG}
mkdir -p ${RUN_DIR}/bag ${RUN_DIR}/plots

ros2 run vio_pkg vio_system_node --ros-args \
    -p use_sim_time:=true \
    -p input_qos_reliability:=best_effort \
    -p camera_fx:=646.33728 \
    -p camera_fy:=645.676147 \
    -p camera_cx:=643.358276 \
    -p camera_cy:=362.999176 \
    -p camera_distortion:="[-0.05594548583030701, 0.06458555161952972, -0.0002526374883018434, 0.0008183500613085926, -0.021141313016414642]" \
    2>&1 | tee ${RUN_DIR}/run_log.txt
```

```bash
# === Terminal 2: record VIO odometry để xem trajectory shape ===
source /opt/ros/humble/setup.bash
RUN_DIR=/home/ubuntu/VIO/results/hcmut_smoke
ros2 bag record --output ${RUN_DIR}/bag --storage sqlite3 /vio/odometry
```

```bash
# === Terminal 3: play HCMUT bag (chú ý remap) ===
source /opt/ros/humble/setup.bash
ros2 bag play /home/ubuntu/VIO/dataset/vio_hcmut_dataset --clock --rate 1.0 \
    --remap /camera/camera/color/image_raw:=/cam0/image_raw \
            /camera/camera/imu:=/imu0
```

```bash
# === Terminal 4 (sau khi Ctrl-C terminal 1+2): xác nhận pipeline live ===
RUN_DIR=/home/ubuntu/VIO/results/hcmut_smoke
# Đếm số message odometry — đủ > 0 nghĩa là pipeline đã estimate được pose.
sqlite3 ${RUN_DIR}/bag/*.db3 \
    "select name, count(*) from messages m join topics t on m.topic_id=t.id group by name;"
```

**KHÔNG chạy `evaluate_m4.py` cho HCMUT.** Không có `/vio/gt_path` ⇒ evo
sẽ ra số vô nghĩa. Khi nào có camera–IMU extrinsics thật + ground-truth
(MoCap / SLAM tham chiếu), bổ sung block §3.C riêng và mở dòng mới trong §1.

### 3.C — Lệnh evo trần (nếu cần chạy thủ công, ngoài `evaluate_m4.py`)

```bash
# ATE (translation, sau khi align SE(3))
evo_ape tum  ${RUN_DIR}/eval/vio_gt_path.tum  ${RUN_DIR}/eval/vio_odom.tum \
    -a --pose_relation trans_part \
    --save_results ${RUN_DIR}/eval/ape_aligned.zip

# RPE 1 m (translation drift mỗi 1 m di chuyển)
evo_rpe tum  ${RUN_DIR}/eval/vio_gt_path.tum  ${RUN_DIR}/eval/vio_odom.tum \
    -a --delta 1 --delta_unit m --pose_relation trans_part \
    --save_results ${RUN_DIR}/eval/rpe_1m_aligned.zip
```

> Đây là **đúng cờ và đúng thứ tự** mà `tools/evaluate_m4.py` đang dùng.
> Đừng đổi `--pose_relation`, `--delta_unit` hay thứ tự `gt vio` nếu không
> muốn số khác nhau giữa các lần chạy.

---

## 4. Câu hỏi "kết quả có tốt hơn không?" — cách trả lời

1. Mở §1, tìm dòng `confirmed` baseline cho dataset tương ứng.
2. Chạy biến thể với `RUN_TAG=<dataset>_<biến_thể>` (giữ nguyên dataset).
3. Thêm một dòng mới vào §1 với cùng dataset, lệnh mới, output dir mới.
4. So sánh `ATE RMSE` và `RPE 1 m RMSE` với baseline:
   - **Cả hai giảm** ⇒ tốt hơn — ghi % giảm vào cột `Ghi chú`.
   - **Một giảm, một tăng** ⇒ trade-off — mô tả trong `Ghi chú`, không claim "better".
   - **Cả hai tăng** ⇒ tệ hơn — đổi trạng thái dòng đó thành `failed` (nhưng vẫn giữ để khỏi thử lại).
5. **Không so giữa các dataset khác nhau.** EuRoC vs HCMUT là so vô nghĩa.

---

## 5. Nhật ký thay đổi bảng

| Ngày | Người sửa | Nội dung |
|------|-----------|----------|
| 2026-05-18 | _scaffolding_ | Tạo bảng, thêm placeholder dòng baseline EuRoC + smoke-test HCMUT. |
| 2026-05-19 | _scaffolding_ | Ghi rõ giới hạn HCMUT: không có extrinsics thật, không có GT ⇒ không tính accuracy. |
| 2026-05-20 | _scaffolding_ | Đóng băng khối lệnh tái chạy ở §3 (evaluate_m4 + evo_ape + evo_rpe). |
| 2026-05-27 | Claude (bootstrap) | Chạy baseline EuRoC `V1_01_easy` trong container `vio-ros2` @ commit `e8425eb`. ATE RMSE = 0.0905 m, RPE 1 m RMSE = 0.0949 m, 2 892 odometry poses. Plot `trajectory_xy.png/pdf` + đầy đủ artifact lưu ở `results/euroc_V1_01_easy/` (ignored by git) và copy paper-figure vào `docs/paper/figures/euroc_V1_01_easy_baseline.{png,pdf}`. Trạng thái dòng #1 chuyển từ `pending` → `confirmed`. |
