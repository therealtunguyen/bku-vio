# Lệnh đánh giá lặp lại (Task 4)

Khối lệnh copy-paste để chạy lại bất cứ run nào trong [`table.md`](table.md).

**Tiền điều kiện (chạy trong container `vio-ros2` một lần / phiên):**

```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros_ws
colcon build --packages-select vio_pkg
source install/setup.bash
pip install --user "numpy<2" "evo[fastentrypoints]" matplotlib   # cần cho evo + plot
mkdir -p /home/ubuntu/results
```

> Hai loại đường dẫn xuất hiện dưới đây:
> - `/home/ubuntu/dataset/...` và `/home/ubuntu/ros_ws/...` — đúng cho container hiện tại (mount riêng từng thư mục).
> - `/home/ubuntu/VIO/...` — đúng nếu mount cả repo theo README (`-v <path>:/home/ubuntu/VIO`).
> Chọn 1 trong 2 tuỳ container, **đừng trộn**.

---

## A — EuRoC `V1_01_easy` (baseline)

**Output:** `results/euroc_V1_01_easy/`. **Wall clock:** ~13 phút (do `bag_rate=0.2`).

```bash
# === Terminal 1: VIO node ===
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros_ws && source install/setup.bash

RUN_TAG=euroc_V1_01_easy
RUN_DIR=/home/ubuntu/results/${RUN_TAG}
rm -rf "${RUN_DIR}"
mkdir -p "${RUN_DIR}/eval" "${RUN_DIR}/plots"   # KHÔNG mkdir bag/ — rosbag2 tự tạo, lỗi nếu đã tồn tại

ros2 run vio_pkg vio_system_node --ros-args \
    -p use_sim_time:=true \
    -p gt_csv_path:=/home/ubuntu/dataset/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv \
    -p imu_init_sample_count:=200 \
    2>&1 | tee ${RUN_DIR}/node.log
```

```bash
# === Terminal 2: bag record (chạy SAU khi node ở terminal 1 đã in dòng init đầu tiên) ===
source /opt/ros/humble/setup.bash
RUN_DIR=/home/ubuntu/results/euroc_V1_01_easy
ros2 bag record --output ${RUN_DIR}/bag --storage sqlite3 \
    /vio/odometry /vio/gt_path \
    2>&1 | tee ${RUN_DIR}/record.log
```

```bash
# === Terminal 3: bag play (chạy SAU khi record đã subscribe) ===
source /opt/ros/humble/setup.bash
ros2 bag play /home/ubuntu/dataset/V1_01_easy --clock --rate 0.2
# Khi xong: Ctrl-C terminal 2 trước (recorder), rồi terminal 1 (node).
```

```bash
# === Terminal 4: export TUM + run evo + plot ===
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros_ws && source install/setup.bash
RUN_DIR=/home/ubuntu/results/euroc_V1_01_easy

python3 /home/ubuntu/tools/evaluate_m4.py ${RUN_DIR}/bag ${RUN_DIR}/eval

evo_res ${RUN_DIR}/eval/ape_aligned.zip   --no_warnings
evo_res ${RUN_DIR}/eval/rpe_1m_aligned.zip --no_warnings

python3 /home/ubuntu/tools/plot_trajectory.py \
    --gt   ${RUN_DIR}/eval/vio_gt_path.tum \
    --vio  ${RUN_DIR}/eval/vio_odom.tum \
    --out  ${RUN_DIR}/plots/trajectory_xy.png \
    --title "EuRoC V1_01_easy - BKU-VIO baseline"
```

> Muốn full automate: copy [`tools/_run_baseline_in_container.sh`](../../tools/_run_baseline_in_container.sh) vào container và chạy. Đó là script tôi dùng để fill dòng baseline.

---

## B — HCMUT `vio_hcmut_dataset` (smoke test)

⚠️ **KHÔNG dùng output làm accuracy result.** Lý do: [`hcmut_smoke.md`](hcmut_smoke.md).
**Output:** `results/hcmut_smoke/`. **Wall clock:** ~70 s.

```bash
# === Terminal 1: VIO node với D455 intrinsics + QoS best_effort ===
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros_ws && source install/setup.bash

RUN_TAG=hcmut_smoke
RUN_DIR=/home/ubuntu/results/${RUN_TAG}
rm -rf "${RUN_DIR}" && mkdir -p "${RUN_DIR}"

ros2 run vio_pkg vio_system_node --ros-args \
    -p use_sim_time:=true \
    -p input_qos_reliability:=best_effort \
    -p camera_fx:=646.33728 \
    -p camera_fy:=645.676147 \
    -p camera_cx:=643.358276 \
    -p camera_cy:=362.999176 \
    -p camera_distortion:="[-0.05594548583030701, 0.06458555161952972, -0.0002526374883018434, 0.0008183500613085926, -0.021141313016414642]" \
    2>&1 | tee ${RUN_DIR}/node.log
```

```bash
# === Terminal 2: bag record (chỉ /vio/odometry — không có /vio/gt_path) ===
source /opt/ros/humble/setup.bash
RUN_DIR=/home/ubuntu/results/hcmut_smoke
ros2 bag record --output ${RUN_DIR}/bag --storage sqlite3 /vio/odometry
```

```bash
# === Terminal 3: bag play với remap topic D455 → topic EuRoC mà node subscribe ===
source /opt/ros/humble/setup.bash
ros2 bag play /home/ubuntu/dataset/vio_hcmut_dataset --clock --rate 1.0 \
    --remap /camera/camera/color/image_raw:=/cam0/image_raw \
            /camera/camera/imu:=/imu0
```

```bash
# === Terminal 4: xác nhận pipeline live (KHÔNG chạy evaluate_m4) ===
RUN_DIR=/home/ubuntu/results/hcmut_smoke
python3 - <<'EOF'
import sqlite3, glob
for db in sorted(glob.glob(f"{__import__('os').environ['RUN_DIR']}/bag/*.db3")):
    with sqlite3.connect(db) as c:
        for name, n in c.execute("select t.name, count(*) from messages m join topics t on m.topic_id=t.id group by t.name"):
            print(f"  {name}: {n}")
EOF
```

> Full automate: [`tools/_run_hcmut_smoke_in_container.sh`](../../tools/_run_hcmut_smoke_in_container.sh) — gồm cả phần thống kê pose để fill cột "Quan sát thực tế".

---

## C — Lệnh `evo` trần (chạy thủ công)

Hai lệnh này là **đúng cờ + đúng thứ tự `gt vio`** mà `evaluate_m4.py` đang dùng. **Đừng đổi** `--pose_relation`, `--delta_unit`, hay thứ tự — sẽ ra số khác.

```bash
# ATE (translation, sau khi align SE(3))
evo_ape tum  ${RUN_DIR}/eval/vio_gt_path.tum  ${RUN_DIR}/eval/vio_odom.tum \
    -a --pose_relation trans_part \
    --save_results ${RUN_DIR}/eval/ape_aligned.zip

# RPE 1 m (translation drift mỗi 1 m di chuyển, consecutive pairs)
evo_rpe tum  ${RUN_DIR}/eval/vio_gt_path.tum  ${RUN_DIR}/eval/vio_odom.tum \
    -a --delta 1 --delta_unit m --pose_relation trans_part \
    --save_results ${RUN_DIR}/eval/rpe_1m_aligned.zip
```

---

## D — Lưu commit hash kèm run

Mỗi run phải kèm hash repo lúc chạy, để khi đọc lại biết code version:

```bash
git -C /home/ubuntu/VIO rev-parse HEAD > ${RUN_DIR}/commit.txt   # hoặc đường dẫn mount repo của bạn
date -Iseconds > ${RUN_DIR}/run_time.txt
```
