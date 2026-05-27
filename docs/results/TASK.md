# TASK tracker — Tuần 2 (CO3107 VIO)

> Tracking các nhiệm vụ sếp giao ngày 18/05 → 24/05.
> Mỗi task: **Yêu cầu** · **Trạng thái** · **Đã làm gì** · **Kết quả / artifact** · **Còn thiếu**.
> Định nghĩa Done của cả gói: nhóm có **một nguồn duy nhất** cho số liệu; mỗi con số có lệnh + thư mục output; HCMUT không bị nhầm là accuracy result.

**Tổng quan:** 6/6 task chính ✅ DONE · 0 BLOCKED · 4 việc mở rộng đang chờ user quyết định.

---

## Task 1 — 18/05 — Tạo bảng theo dõi kết quả

- **Yêu cầu:** Cột dataset / lệnh chạy / thư mục output / ATE / RPE / ghi chú / trạng thái. Output: bảng dùng chung hoặc markdown.
- **Trạng thái:** ✅ **DONE**
- **Đã làm:**
  - Tạo [`docs/results/RESULTS.md`](RESULTS.md) làm single source of truth.
  - §1 = bảng (8 cột: #, dataset, lệnh rút gọn, bag dir, eval dir, ATE, RPE, ghi chú, trạng thái).
  - §2 = quy ước thư mục output (bag/, eval/, plots/, run_log.txt).
  - §4 = quy trình so sánh "tốt hơn không?".
  - §5 = changelog mỗi lần sửa bảng.
- **Kết quả:** `docs/results/RESULTS.md` đã commit `e912e41`.
- **Còn thiếu:** — không.

---

## Task 2 — 18/05 — Thêm kết quả EuRoC hiện tại (baseline đã chốt)

- **Yêu cầu:** Dùng thư mục và số liệu từ kết quả VIO hiện tại. Output: dòng "baseline đã chốt" trong bảng.
- **Trạng thái:** ✅ **DONE** (đã chạy thật, không phải placeholder)
- **Đã làm:**
  - Boot colima → start container `vio-ros2` → cài `numpy<2 evo matplotlib` → `colcon build vio_pkg`.
  - Chạy `vio_system_node` + `ros2 bag record /vio/odometry /vio/gt_path` + `ros2 bag play V1_01_easy --rate 0.2`.
  - Wall clock ~13 phút (144 s bag × rate 0.2 + setup + teardown).
  - Chạy `tools/evaluate_m4.py` → `ape_aligned.zip` + `rpe_1m_aligned.zip`.
  - Chạy `tools/plot_trajectory.py` → PNG + PDF.
  - Copy artifact ra host: `results/euroc_V1_01_easy/`.
  - Điền dòng #1 trong bảng, trạng thái `confirmed`, commit `e8425eb`.
- **Kết quả (số thật trên `V1_01_easy`, commit `e8425eb`, 2026-05-27):**

  | Metric (SE(3)-aligned) | RMSE [m] | mean | median | std | min | max |
  |---|---|---|---|---|---|---|
  | **ATE** (trans) | **0.0905** | 0.0813 | 0.0752 | 0.0397 | 0.0037 | 0.2744 |
  | **RPE 1 m** (trans, consecutive) | **0.0949** | 0.0898 | 0.0946 | 0.0305 | 0.0242 | 0.1481 |

  - Bag ghi: 2 892 `/vio/odometry` + 146 `/vio/gt_path` poses.
  - Artifact: `results/euroc_V1_01_easy/{bag,eval,plots,node.log,play.log,record.log,commit.txt,run_time.txt}`.
- **Còn thiếu:** — không (số đã `confirmed`).

---

## Task 3 — 19/05 — Ghi trạng thái dataset HCMUT (smoke test only)

- **Yêu cầu:** Đánh dấu HCMUT = "smoke test only". Lý do: pipeline chạy được nhưng trajectory diverge, calibration chưa xác nhận. Output: dòng HCMUT trong bảng với giới hạn rõ ràng.
- **Trạng thái:** ✅ **DONE**
- **Đã làm:**
  - Dòng #2 trong `RESULTS.md` §1: dataset = `vio_hcmut_dataset (D455)`, ATE/RPE = **N/A**, trạng thái = `smoke-test`.
  - Ghi chú đầy đủ: (a) calibration camera–IMU extrinsics chưa xác nhận (đang dùng default EuRoC); (b) trajectory diverge sau vài giây; (c) không có ground-truth; (d) **KHÔNG báo accuracy từ dòng này**; (e) điều kiện gating để nâng lên `confirmed` (cần TF static thật từ `librealsense` / `/tf_static`).
  - Block §3.B trong `RESULTS.md` viết lệnh smoke với cờ rõ: `input_qos_reliability:=best_effort` + remap topic D455.
  - Checklist §G trong `WEEK2_CHECKLIST.md` ghi rõ HCMUT KHÔNG nằm trong scope tuần 2.
- **Kết quả:** Dòng #2 bảng, §3.B, §G checklist — committed.
- **Còn thiếu (optional, không nằm trong yêu cầu):**
  - Smoke run thực tế (~2 phút) để fill ô `Ghi chú` với số đo quan sát được (số message `/vio/odometry`, thời gian trước khi diverge). Hiện đang là mô tả tổng quát.

---

## Task 4 — 20/05 — Chuẩn bị lệnh đánh giá lặp lại

- **Yêu cầu:** Lệnh chính xác cho `tools/evaluate_m4.py`, `evo_ape`, `evo_rpe`. Output: khối lệnh thành viên khác copy-paste chạy lại được.
- **Trạng thái:** ✅ **DONE**
- **Đã làm:**
  - `RESULTS.md` §3.A — block đầy đủ EuRoC: Terminal 1 (launch + tee log), Terminal 2 (`ros2 bag record`), Terminal 3 (`evaluate_m4.py` + `evo_res` + `plot_trajectory.py`).
  - §3.B — block smoke HCMUT (KHÔNG dùng `evaluate_m4.py`).
  - §3.C — lệnh `evo_ape` / `evo_rpe` trần, với note "đúng cờ và đúng thứ tự `gt vio` — đừng đổi".
  - Pre-điều kiện chung: `source ROS + colcon build + source install + pip install evo`.
  - Quy ước `RUN_TAG=<dataset>_<biến_thể>` để không ghi đè baseline khi test cải tiến.
- **Kết quả:** `RESULTS.md` §3 — committed.
- **Còn thiếu:** — không.

---

## Task 5 — 21/05 — Biểu đồ trajectory từ EuRoC đã xác nhận

- **Yêu cầu:** Dùng làm hình minh họa nháp trong paper. Output: PNG/PDF + đường dẫn kết quả gốc.
- **Trạng thái:** ✅ **DONE**
- **Đã làm:**
  - Viết `tools/plot_trajectory.py` (nhận 2 file TUM, xuất PNG + PDF, headless-safe).
  - Smoke-test bằng dữ liệu tổng hợp trước khi giao team.
  - Chạy với output thật của baseline (Task 2) → ảnh thật.
  - Copy ảnh từ `results/.../plots/` sang `docs/paper/figures/` để track trong git.
- **Kết quả:**
  - **Ảnh paper:** `docs/paper/figures/euroc_V1_01_easy_baseline.png` (1200×1100) + `.pdf` (vector).
  - **Nguồn gốc:** `results/euroc_V1_01_easy/plots/trajectory_xy.{png,pdf}` (gitignored).
  - **Dữ liệu nguồn:** `results/euroc_V1_01_easy/eval/vio_gt_path.tum` (28 712 GT poses) + `vio_odom.tum` (2 892 VIO poses).
  - **Lệnh tái tạo:** `python3 tools/plot_trajectory.py --gt <gt.tum> --vio <vio.tum> --out <out.png> --title "..."`.
- **Còn thiếu:** — không.

---

## Task 6 — 23–24/05 — Checklist thực nghiệm Tuần 2

- **Yêu cầu:** Liệt kê toàn bộ run cần làm trước data-freeze. Output: checklist đầy đủ.
- **Trạng thái:** ✅ **DONE**
- **Đã làm:** Tạo [`docs/results/WEEK2_CHECKLIST.md`](WEEK2_CHECKLIST.md) gồm 7 section:
  - **A.** Trước khi chạy bất kỳ run nào (commit hash, rebuild, sanity tests).
  - **B.** Baseline EuRoC `V1_01_easy` — BẮT BUỘC. ← *đã làm xong trong Task 2.*
  - **C.** EuRoC biến thể (no_gt_init / imu_init_100 / meas_noise_1e-3) để trả lời "tốt hơn không?".
  - **D.** EuRoC sequence khác (V1_02_medium, V2_01_easy) — optional.
  - **E.** HCMUT smoke — BẮT BUỘC để confirm pipeline live, KHÔNG tính accuracy.
  - **F.** Kiểm tra trước data-freeze (đề xuất tag `data-freeze-week2` ngày 2026-05-30).
  - **G.** Việc còn thiếu để HCMUT mới được tính accuracy (out-of-scope tuần 2).
- **Kết quả:** `docs/results/WEEK2_CHECKLIST.md` — committed.
- **Còn thiếu:** — không (đó là checklist; *thực thi* checklist là task tuần sau).

---

## Definition of Done (sếp ghi)

| Tiêu chí | Trạng thái |
|---|---|
| Nhóm có một nguồn duy nhất cho số liệu | ✅ `docs/results/RESULTS.md` |
| Mỗi con số có lệnh và thư mục output đi kèm | ✅ Dòng baseline có `RESULTS.md §3.A` + `results/euroc_V1_01_easy/` |
| HCMUT không bị nhầm là accuracy | ✅ Dòng #2 trạng thái `smoke-test`, ATE/RPE = **N/A**, ghi rõ lý do + điều kiện gating |

→ **Định nghĩa Done của gói tuần 2 đã đạt.**

---

## Công việc mở rộng (chưa làm — chờ user quyết định)

| # | Việc | Lý do chưa làm | Thời gian ước tính |
|---|---|---|---|
| E1 | `git push origin feature/backend-tyler` | Commit `e912e41` đang local; push là action ra mạng, không tự làm. | < 1 phút |
| E2 | HCMUT smoke run thực tế → fill số observation vào dòng #2 | Optional (yêu cầu sếp đã đạt với mô tả tổng quát). | ~3 phút |
| E3 | EuRoC biến thể (1-3 cái: `no_gt_init`, `imu_init_100`, `meas_noise_1e-3`) | Nằm trong checklist §C nhưng là task tuần sau, không phải tuần 2. | ~15 phút / biến thể |
| E4 | EuRoC sequence khác (`V1_02_medium`, `V2_01_easy`) | Cần download dataset; checklist §D đánh dấu optional. | ~20 phút / sequence |

Nói "push", "smoke", "biến thể X", hoặc "sequence Y" thì tôi chạy tiếp.

---

## File quan trọng (cheat-sheet)

```
docs/
├── results/
│   ├── TASK.md              ← file này
│   ├── RESULTS.md           ← bảng + lệnh + quy trình so sánh
│   └── WEEK2_CHECKLIST.md   ← checklist tuần 2
└── paper/
    └── figures/
        ├── euroc_V1_01_easy_baseline.png   ← cho slide
        └── euroc_V1_01_easy_baseline.pdf   ← cho paper LaTeX

results/                     ← gitignore; artifact thô
└── euroc_V1_01_easy/
    ├── bag/                 (raw recorded /vio/odometry + /vio/gt_path)
    ├── eval/                (vio_odom.tum, vio_gt_path.tum, ape_aligned.zip, rpe_1m_aligned.zip)
    ├── plots/               (trajectory_xy.png + .pdf)
    ├── node.log play.log record.log
    └── commit.txt run_time.txt

tools/
├── evaluate_m4.py                       (đã có sẵn — bag → TUM + evo)
├── plot_trajectory.py                   (mới — TUM → PNG/PDF)
└── _run_baseline_in_container.sh        (mới — orchestrator dùng cho run baseline; team theo §3 thay vì script này)
```

Commit cuối: `e912e41 docs(results): add single-source-of-truth tracking + EuRoC V1_01_easy baseline`.
