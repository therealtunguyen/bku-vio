# EuRoC `V1_01_easy` baseline (Task 2)

**Trạng thái:** `confirmed` — dùng được trong báo cáo / paper.
**Commit:** `e8425eb` (branch `feature/backend-tyler`).
**Ngày chạy:** 2026-05-27 22:58 ICT.
**Container:** `vio-ros2` (image `ghcr.io/tiryoh/ros2-desktop-vnc:humble`).
**Lệnh tái chạy:** [`eval_commands.md` §A](eval_commands.md#a--euroc-v1_01_easy-baseline).

## Kết quả

### ATE (Absolute Pose Error, translation, SE(3)-aligned Umeyama)

| RMSE | mean | median | std | min | max |
|------|------|--------|-----|-----|-----|
| **0.0905 m** | 0.0813 | 0.0752 | 0.0397 | 0.0037 | 0.2744 |

### RPE 1 m (Relative Pose Error, translation, consecutive pairs, Δ=1m, SE(3)-aligned)

| RMSE | mean | median | std | min | max |
|------|------|--------|-----|-----|-----|
| **0.0949 m** | 0.0898 | 0.0946 | 0.0305 | 0.0242 | 0.1481 |

## Cấu hình run

| Tham số | Giá trị |
|---|---|
| Dataset | `dataset/V1_01_easy` (EuRoC, 145.6 s, 32 032 messages) |
| Bag rate | `0.2` (wall clock ~12 phút) |
| Camera intrinsics | EuRoC default (fx=458.654, fy=457.296, cx=367.215, cy=248.375) |
| Camera–IMU extrinsics | EuRoC default (trong `backend/state_server.py`) |
| `imu_init_sample_count` | 200 |
| `input_qos_reliability` | `reliable` (default) |
| `gt_csv_path` | `mav0/state_groundtruth_estimate0/data.csv` (Vicon) |
| Recorded topics | `/vio/odometry` (2 892 msg), `/vio/gt_path` (146 msg) |

## Artifact

```
results/euroc_V1_01_easy/
├── bag/                       # raw ros2 bag, 2892 odom + 146 gt_path
├── eval/
│   ├── vio_odom.tum           # 2 892 VIO poses
│   ├── vio_gt_path.tum        # 28 712 GT poses (Vicon, dày hơn)
│   ├── ape_aligned.zip        # full evo APE result
│   └── rpe_1m_aligned.zip     # full evo RPE result
├── plots/
│   ├── trajectory_xy.png      # 1200×1100 RGBA
│   └── trajectory_xy.pdf      # vector cho LaTeX
├── node.log play.log record.log
├── commit.txt                 # e8425eb30908a4cf3a036cb3d726c7c4c3a5069c
└── run_time.txt
```

> `results/` đã gitignore. Ảnh dùng cho paper đã được copy sang [`docs/paper/figures/euroc_V1_01_easy_baseline.{png,pdf}`](../paper/figures/) (track trong git).

## Đọc lại RMSE thủ công

```bash
# trong container, hoặc trên host nếu có evo:
evo_res results/euroc_V1_01_easy/eval/ape_aligned.zip --no_warnings
evo_res results/euroc_V1_01_easy/eval/rpe_1m_aligned.zip --no_warnings
```

## So sánh biến thể với baseline này

Khi nhóm có biến thể (tham số mới, code mới), **lặp lại** lệnh §A với `RUN_TAG=euroc_V1_01_easy_<biến_thể>`, thêm dòng mới vào [`table.md`](table.md), so sánh ATE + RPE theo quy trình §4 trong `table.md`. **Không sửa dòng baseline.**

Đề xuất biến thể trong [`week2_checklist.md` §C](week2_checklist.md): `no_gt_init`, `imu_init_100`, `meas_noise_1e-3`.

## Ghi chú thực thi

- Lần run đầu tiên có 2 bug: (1) `mkdir bag/` trước khiến `ros2 bag record` từ chối ghi, (2) `kill -INT` không kill được node con. Đã fix trong [`tools/_run_baseline_in_container.sh`](../../tools/_run_baseline_in_container.sh): không pre-create bag dir + dùng `pkill -INT` rồi fallback `pkill -9`.
- Node log dài 487 KB, ổn định suốt 12 phút, không có exception trừ shutdown bình thường.
