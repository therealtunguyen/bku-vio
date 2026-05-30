# TASK tracker — Tuần 2 (CO3107 VIO)

> Index toàn bộ task sếp giao 18/05 → 24/05. Mỗi task có **file kết quả riêng** — click link để xem chi tiết.
> **Tổng quan:** 6/6 task chính ✅ DONE · 0 BLOCKED · 4 việc mở rộng đang chờ user quyết định.
> Definition of Done của sếp đã đạt: nhóm có **một nguồn duy nhất** ([`table.md`](table.md)) cho số liệu; mỗi con số có lệnh + thư mục output; HCMUT không bị nhầm là accuracy.

| Task | Hạn | File kết quả | Trạng thái | Tóm tắt |
|------|-----|---------------|-----------|---------|
| **1. Tạo bảng theo dõi** | 18/05 | [`table.md`](table.md) | ✅ DONE | Bảng 8 cột, single source of truth. |
| **2. Baseline EuRoC** | 18/05 | [`euroc_baseline.md`](euroc_baseline.md) | ✅ DONE — số thật | ATE 0.0905 m, RPE 1 m 0.0949 m, commit `e8425eb`. |
| **3. HCMUT smoke-test** | 19/05 | [`hcmut_smoke.md`](hcmut_smoke.md) | ✅ DONE — quan sát thật | 221 poses, diverge sau ~12.6 s sim, KHÔNG accuracy. |
| **4. Lệnh đánh giá lặp lại** | 20/05 | [`eval_commands.md`](eval_commands.md) | ✅ DONE | §A EuRoC, §B HCMUT, §C `evo` trần, §D commit logging. |
| **5. Trajectory plot cho paper** | 21/05 | [`trajectory_plot.md`](trajectory_plot.md) | ✅ DONE | `docs/paper/figures/euroc_V1_01_easy_baseline.{png,pdf}`. |
| **6. Checklist Tuần 2** | 23–24/05 | [`week2_checklist.md`](week2_checklist.md) | ✅ DONE | 7 section (A→G), data-freeze 2026-05-30. |

---

## Chi tiết từng task

### Task 1 — 18/05 — Tạo bảng theo dõi kết quả

- **Yêu cầu:** Cột dataset / lệnh / output dir / ATE / RPE / ghi chú / trạng thái. Output: bảng dùng chung hoặc markdown.
- **File kết quả:** [`table.md`](table.md)
- **Đã làm:** Bảng 8 cột với 2 dòng (EuRoC baseline `confirmed`, HCMUT `smoke-test`). Kèm quy ước thư mục output, quy trình cập nhật một dòng, và quy trình so sánh "tốt hơn không?".
- **Còn thiếu:** — không.

### Task 2 — 18/05 — Baseline EuRoC `V1_01_easy`

- **Yêu cầu:** Dùng thư mục và số liệu từ kết quả VIO hiện tại. Output: dòng "baseline đã chốt".
- **File kết quả:** [`euroc_baseline.md`](euroc_baseline.md) (chi tiết) + dòng #1 trong [`table.md`](table.md) (bảng).
- **Đã làm:** Chạy thật trong container `vio-ros2` @ commit `e8425eb` (2026-05-27, wall clock ~13 phút). 2 892 odometry poses + 146 GT poses.
- **Kết quả:**
  - ATE (trans, SE(3)-aligned) **RMSE 0.0905 m**, mean 0.0813, median 0.0752, std 0.0397, max 0.2744.
  - RPE 1 m (trans, consecutive) **RMSE 0.0949 m**, mean 0.0898, median 0.0946, std 0.0305, max 0.1481.
  - Artifact đầy đủ ở `results/euroc_V1_01_easy/`; ảnh paper ở `docs/paper/figures/euroc_V1_01_easy_baseline.{png,pdf}`.
- **Còn thiếu:** — không.

### Task 3 — 19/05 — HCMUT smoke-test only

- **Yêu cầu:** Đánh dấu "smoke test only", lý do calibration chưa xác nhận + trajectory diverge. Output: dòng HCMUT có giới hạn rõ ràng.
- **File kết quả:** [`hcmut_smoke.md`](hcmut_smoke.md) (chi tiết) + dòng #2 trong [`table.md`](table.md).
- **Đã làm:** Chạy smoke thật 2026-05-27 (wall clock 66 s, sim duration 48 s). Có quan sát số đo cụ thể.
- **Kết quả:**
  - 221 messages `/vio/odometry` xuất ra → pipeline live.
  - Diverge: `‖pos‖ > 10 m` lúc sim t = **+12.64 s**; max `‖pos‖` cuối run = **8 836 m**.
  - Lý do tài liệu hoá: extrinsics camera–IMU = default EuRoC (sai cho D455).
  - ATE / RPE = **N/A** — không có GT, không chạy `evaluate_m4.py`.
- **Còn thiếu:** Việc nâng dòng HCMUT lên `confirmed` cần TF static thật + ground-truth scene — ngoài scope Tuần 2 (xem [`week2_checklist.md` §G](week2_checklist.md)).

### Task 4 — 20/05 — Lệnh đánh giá lặp lại

- **Yêu cầu:** Khối lệnh chính xác cho `evaluate_m4.py`, `evo_ape`, `evo_rpe` — copy-paste chạy lại được.
- **File kết quả:** [`eval_commands.md`](eval_commands.md)
- **Đã làm:** 4 section:
  - §A — EuRoC baseline (4 terminal, full pipeline).
  - §B — HCMUT smoke (3 terminal + verify, KHÔNG chạy evo).
  - §C — `evo_ape` / `evo_rpe` trần (giữ đúng cờ + thứ tự `gt vio`).
  - §D — Lưu `commit.txt` kèm mỗi run.
- **Tự động hoá:** [`tools/_run_baseline_in_container.sh`](../../tools/_run_baseline_in_container.sh) (EuRoC) và [`tools/_run_hcmut_smoke_in_container.sh`](../../tools/_run_hcmut_smoke_in_container.sh) (HCMUT) là 2 orchestrator dùng cho bootstrap; team workflow chính vẫn theo `eval_commands.md`.
- **Còn thiếu:** — không.

### Task 5 — 21/05 — Biểu đồ trajectory cho paper

- **Yêu cầu:** Hình minh họa nháp từ EuRoC `confirmed`. Output: PNG/PDF + đường dẫn kết quả gốc.
- **File kết quả:** [`trajectory_plot.md`](trajectory_plot.md) (mô tả) + ảnh thật ở `docs/paper/figures/`.
- **Đã làm:**
  - Script: [`tools/plot_trajectory.py`](../../tools/plot_trajectory.py) (TUM → PNG/PDF, headless-safe).
  - Ảnh paper: `docs/paper/figures/euroc_V1_01_easy_baseline.png` (1200×1100) + `.pdf` (vector).
  - Ảnh gốc: `results/euroc_V1_01_easy/plots/trajectory_xy.{png,pdf}` (gitignored).
  - Dữ liệu nguồn: 28 712 GT poses + 2 892 VIO poses từ `results/euroc_V1_01_easy/eval/*.tum`.
- **Còn thiếu:** — không.

### Task 6 — 23–24/05 — Checklist thực nghiệm Tuần 2

- **Yêu cầu:** Liệt kê toàn bộ run cần làm trước data-freeze.
- **File kết quả:** [`week2_checklist.md`](week2_checklist.md)
- **Đã làm:** 7 section:
  - **A.** Trước khi chạy (commit hash, rebuild, sanity tests). Pending.
  - **B.** Baseline EuRoC — ✅ đã xong.
  - **C.** Biến thể EuRoC (`no_gt_init`, `imu_init_100`, `meas_noise_1e-3`). Pending.
  - **D.** Sequence khác (`V1_02_medium`, `V2_01_easy`). Optional.
  - **E.** HCMUT smoke — ✅ đã xong.
  - **F.** Kiểm tra trước data-freeze (đề xuất tag `data-freeze-week2`).
  - **G.** Việc còn thiếu để HCMUT thành accuracy — out-of-scope Tuần 2.
- **Còn thiếu:** thực thi check-box trong A/C/D/F là task tuần sau, không phải bản thân checklist.

---

## Definition of Done của sếp

| Tiêu chí | Trạng thái |
|---|---|
| Nhóm có một nguồn duy nhất cho số liệu | ✅ [`table.md`](table.md) |
| Mỗi con số có lệnh + thư mục output đi kèm | ✅ Dòng baseline: lệnh ở [`eval_commands.md §A`](eval_commands.md), output ở `results/euroc_V1_01_easy/` |
| HCMUT không bị nhầm là accuracy | ✅ Dòng #2 `smoke-test`, ATE/RPE = **N/A**, [`hcmut_smoke.md`](hcmut_smoke.md) giải thích chi tiết |

---

## Công việc mở rộng (chưa làm — chờ user quyết định)

| # | Việc | Lý do chưa làm | Thời gian ước tính |
|---|---|---|---|
| E1 | `git push origin feature/backend-tyler` | Push là action ra mạng, không tự làm. | < 1 phút |
| E2 | Biến thể EuRoC (1-3 cái: `no_gt_init`, `imu_init_100`, `meas_noise_1e-3`) | Nằm trong [`week2_checklist.md §C`](week2_checklist.md) nhưng là task tuần sau. | ~15 phút / biến thể |
| E3 | EuRoC sequence khác (`V1_02_medium`, `V2_01_easy`) | Cần download dataset; checklist §D đánh dấu optional. | ~20 phút / sequence |
| E4 | Gating HCMUT (TF static + GT) | Out-of-scope Tuần 2 — checklist §G. | Cần hardware/MoCap session, không tự làm được. |

Nói `push`, `biến thể X`, hoặc `sequence Y` thì tôi chạy tiếp.

---

## Cấu trúc thư mục

```
docs/results/
├── TASK.md              ← file này (index)
├── table.md             ← Task 1: bảng tracking
├── euroc_baseline.md    ← Task 2: baseline EuRoC chi tiết
├── hcmut_smoke.md       ← Task 3: HCMUT smoke chi tiết
├── eval_commands.md     ← Task 4: lệnh copy-paste
├── trajectory_plot.md   ← Task 5: ảnh + cách tái tạo
└── week2_checklist.md   ← Task 6: checklist Tuần 2

docs/paper/figures/
├── euroc_V1_01_easy_baseline.png    ← slide / web
└── euroc_V1_01_easy_baseline.pdf    ← LaTeX vector

results/                 ← gitignore; artifact thô per-run
├── euroc_V1_01_easy/    (bag, eval, plots, *.log, commit.txt)
└── hcmut_smoke/         (bag, *.log)

tools/
├── evaluate_m4.py                          (có sẵn — bag → TUM + evo)
├── plot_trajectory.py                      (mới — TUM → PNG/PDF)
├── _run_baseline_in_container.sh           (mới — orchestrator baseline)
└── _run_hcmut_smoke_in_container.sh        (mới — orchestrator smoke)
```

Commit gần nhất: `git log --oneline -3` để xem.
