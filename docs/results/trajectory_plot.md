# Biểu đồ trajectory cho paper (Task 5)

**Mục đích:** ảnh minh họa nháp trong báo cáo / paper, lấy từ baseline EuRoC đã `confirmed`.

## File đầu ra

| File | Mục đích | Track trong git? |
|---|---|---|
| [`docs/paper/figures/euroc_V1_01_easy_baseline.png`](../paper/figures/euroc_V1_01_easy_baseline.png) | Slide, README, web | ✅ |
| [`docs/paper/figures/euroc_V1_01_easy_baseline.pdf`](../paper/figures/euroc_V1_01_easy_baseline.pdf) | LaTeX `\includegraphics` (vector) | ✅ |
| `results/euroc_V1_01_easy/plots/trajectory_xy.png` | Gốc (output của script) | ❌ (gitignore) |
| `results/euroc_V1_01_easy/plots/trajectory_xy.pdf` | Gốc (output của script) | ❌ (gitignore) |

## Nguồn dữ liệu

| File | Số pose | Vai trò |
|---|---|---|
| `results/euroc_V1_01_easy/eval/vio_gt_path.tum` | 28 712 | Ground truth (Vicon, ~200 Hz) |
| `results/euroc_V1_01_easy/eval/vio_odom.tum` | 2 892 | VIO output (~20 Hz, theo camera) |

Hai file này được export bởi `tools/evaluate_m4.py` từ bag `results/euroc_V1_01_easy/bag/`. Xem [`euroc_baseline.md`](euroc_baseline.md) cho chi tiết run.

## Cách tái tạo

```bash
python3 tools/plot_trajectory.py \
    --gt   results/euroc_V1_01_easy/eval/vio_gt_path.tum \
    --vio  results/euroc_V1_01_easy/eval/vio_odom.tum \
    --out  results/euroc_V1_01_easy/plots/trajectory_xy.png \
    --title "EuRoC V1_01_easy - BKU-VIO baseline"

# Promote sang figures cho paper nếu version mới tốt hơn:
cp results/euroc_V1_01_easy/plots/trajectory_xy.png docs/paper/figures/euroc_V1_01_easy_baseline.png
cp results/euroc_V1_01_easy/plots/trajectory_xy.pdf docs/paper/figures/euroc_V1_01_easy_baseline.pdf
```

Script: [`tools/plot_trajectory.py`](../../tools/plot_trajectory.py) — nhận 2 file TUM (gt, vio), xuất PNG + PDF, headless-safe (dùng được trong container không có DISPLAY).

## Đọc đồ thị

- Trục: `x [m]` / `y [m]` (top-down view).
- Đen: ground truth (Vicon).
- Xanh: VIO baseline.
- Chấm xanh lá: start. Chữ thập đỏ: end (theo GT).
- Aspect ratio 1:1 để không méo khoảng cách.

## Sinh thêm biến thể plot

Khi có biến thể chạy (xem [`week2_checklist.md` §C](week2_checklist.md)), tạo plot riêng:

```bash
python3 tools/plot_trajectory.py \
    --gt   results/euroc_V1_01_easy_no_gt_init/eval/vio_gt_path.tum \
    --vio  results/euroc_V1_01_easy_no_gt_init/eval/vio_odom.tum \
    --out  results/euroc_V1_01_easy_no_gt_init/plots/trajectory_xy.png \
    --title "EuRoC V1_01_easy - no GT init"
```

Plot so sánh nhiều biến thể trên cùng đồ thị: dùng `evo_traj` (chưa wrap vào script này):

```bash
evo_traj tum --ref results/euroc_V1_01_easy/eval/vio_gt_path.tum \
    results/euroc_V1_01_easy/eval/vio_odom.tum \
    results/euroc_V1_01_easy_no_gt_init/eval/vio_odom.tum \
    -a --plot_mode xy --save_plot results/compare_baseline_vs_no_gt_init.pdf
```
