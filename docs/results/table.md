# Bảng theo dõi kết quả (Task 1)

**Single source of truth** cho mọi con số ATE / RPE. Mỗi dòng có lệnh đầy đủ (link sang [`eval_commands.md`](eval_commands.md)) và thư mục output đi kèm — thiếu một trong hai thì **không được trích dẫn**.

Trạng thái: `confirmed` (dùng được trong báo cáo) · `smoke-test` (chỉ kiểm pipeline, KHÔNG accuracy) · `pending` (chưa chạy / đang chạy) · `failed` (đã chạy nhưng diverge/crash, giữ để khỏi lặp lại).

| # | Dataset | Lệnh | Bag dir | Eval dir | ATE [m] (RMSE) | RPE 1 m [m] (RMSE) | Ghi chú | Trạng thái |
|---|---------|------|---------|----------|----------------|--------------------|---------|------------|
| 1 | EuRoC `V1_01_easy` | [`eval_commands.md` §A](eval_commands.md#a--euroc-v1_01_easy-baseline) | `results/euroc_V1_01_easy/bag` (2 892 `/vio/odometry`, 146 `/vio/gt_path`) | `results/euroc_V1_01_easy/eval` | **0.0905** (mean 0.0813, median 0.0752, max 0.2744, std 0.0397) | **0.0949** (mean 0.0898, median 0.0946, max 0.1481, std 0.0305) | **Baseline đã chốt** — commit `e8425eb`, run 2026-05-27. EuRoC calibration mặc định, `imu_init_sample_count=200`, `bag_rate=0.2`, SE(3)-aligned. Chi tiết: [`euroc_baseline.md`](euroc_baseline.md). Plot: [`docs/paper/figures/euroc_V1_01_easy_baseline.png`](../paper/figures/euroc_V1_01_easy_baseline.png). | `confirmed` |
| 2 | HCMUT `vio_hcmut_dataset` (D455) | [`eval_commands.md` §B](eval_commands.md#b--hcmut-vio_hcmut_dataset-smoke-test) | `results/hcmut_smoke/bag` (221 `/vio/odometry`) | _không tính_ | **N/A** | **N/A** | **Smoke test only** — run 2026-05-27. Pipeline live (221 poses xuất ra) nhưng **diverge sau ~12.6 s sim** (max `‖pos‖` = 8 836 m). Lý do: camera–IMU extrinsics chưa xác nhận; không có ground-truth. **KHÔNG trích dẫn accuracy.** Chi tiết: [`hcmut_smoke.md`](hcmut_smoke.md). | `smoke-test` |

> Đơn vị: mét. Lấy `rmse` từ key cùng tên trong `stats.json` (unzip file `ape_aligned.zip` / `rpe_1m_aligned.zip` trong `eval/`), hoặc dùng `evo_res <file>.zip --no_warnings`.

## Quy trình cập nhật một dòng

1. Chạy đúng lệnh ở [`eval_commands.md`](eval_commands.md) cho dataset tương ứng (giữ `RUN_TAG` mới nếu là biến thể, **không ghi đè** dòng baseline).
2. Đổi `Trạng thái` thành `confirmed` (có GT) hoặc `smoke-test` (không GT).
3. Điền ATE / RPE RMSE. Ghi `commit hash` + ngày vào `Ghi chú`.
4. Commit lại file này.

## Cách trả lời "kết quả có tốt hơn không?"

1. Mở bảng này, tìm dòng `confirmed` baseline cho dataset tương ứng.
2. Chạy biến thể với `RUN_TAG=<dataset>_<biến_thể>` (cùng dataset, đổi tham số).
3. Thêm dòng mới vào bảng (không sửa dòng baseline).
4. So sánh ATE + RPE với baseline:
   - **Cả hai giảm** ⇒ tốt hơn — ghi % giảm vào `Ghi chú`.
   - **Một giảm một tăng** ⇒ trade-off — mô tả, **không** claim "better".
   - **Cả hai tăng** ⇒ tệ hơn — đổi trạng thái thành `failed`.
5. **Không so giữa các dataset khác nhau** — EuRoC vs HCMUT là so vô nghĩa.

## Quy ước thư mục output

Tất cả run đi vào `results/` ở repo root (gitignore, không commit binary). Cấu trúc bắt buộc:

```
results/
└── <dataset_tag>/
    ├── bag/                   # ros2 bag record /vio/odometry + /vio/gt_path
    ├── eval/                  # output evaluate_m4.py: *.tum + ape_*.zip + rpe_*.zip
    ├── plots/                 # output plot_trajectory.py: *.png + *.pdf
    ├── node.log play.log record.log
    └── commit.txt run_time.txt
```

`<dataset_tag>` = `<dataset>_<biến_thể>`. Ví dụ: `euroc_V1_01_easy`, `euroc_V1_01_easy_no_gt_init`, `hcmut_smoke`.

## Changelog

| Ngày | Người sửa | Nội dung |
|------|-----------|----------|
| 2026-05-18 | scaffolding | Tạo bảng, placeholder baseline + smoke HCMUT. |
| 2026-05-27 | Claude (bootstrap) | Chạy baseline EuRoC thật → dòng #1 `confirmed`. ATE 0.0905, RPE 0.0949. Chạy HCMUT smoke thật → dòng #2 cập nhật quan sát (diverge t=12.6s, 221 poses). Tách `RESULTS.md` cũ thành các file per-task. |
