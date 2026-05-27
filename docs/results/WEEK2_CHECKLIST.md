# Checklist thực nghiệm Tuần 2 (2026-05-25 → 2026-05-31)

**Mục tiêu của tuần.** Đóng băng (data-freeze) toàn bộ con số dùng cho báo
cáo / paper. Sau ngày freeze, không chạy lại trừ khi tìm thấy bug ảnh hưởng
metric.

**Data-freeze deadline (đề xuất):** 2026-05-30 23:59 (giờ VN). Mọi run sau
deadline chỉ được dùng cho phân tích bổ sung, không thay con số chính trong
báo cáo.

**Single source of truth:** [`RESULTS.md`](./RESULTS.md). Mọi check ☑ dưới
đây phải đi kèm dòng tương ứng trong §1 của `RESULTS.md` với `Trạng thái`
đã chuyển sang `confirmed` (hoặc `smoke-test` cho HCMUT).

---

## A. Trước khi chạy bất kỳ run nào (1 lần / tuần)

- [ ] `git pull` về nhánh dùng để báo cáo và ghi lại **commit hash** vào
      `run_log.txt` của từng run. Lệnh: `git rev-parse HEAD > results/<tag>/commit.txt`
- [ ] Trong container: `colcon build --packages-select vio_pkg && source install/setup.bash`
- [ ] Chạy 2 test sanity, cả hai phải PASS:
      `python3 src/vio_pkg/test/test_backend_consistency.py`
      `python3 src/vio_pkg/test/test_runtime_safeguards.py`
- [ ] `pip show evo` ⇒ đảm bảo evo đã cài; `pip install "numpy<2"` nếu chưa.
- [ ] Đảm bảo không còn process cũ: `pkill -f vio_system_node; pkill -f "ros2 bag"`

## B. Baseline EuRoC `V1_01_easy` (BẮT BUỘC — đóng paper với dòng này)

- [ ] Chạy đúng block §3.A trong `RESULTS.md`, `RUN_TAG=euroc_V1_01_easy`.
- [ ] Kiểm log có dòng `Camera calibration: fx=458.654000, ...` và
      `Input sensor QoS reliability: reliable`.
- [ ] Trajectory không diverge (kiểm bằng RViz và bằng plot ở bước kế).
- [ ] `tools/evaluate_m4.py` ra 2 file `ape_aligned.zip`, `rpe_1m_aligned.zip` không lỗi.
- [ ] Chạy `tools/plot_trajectory.py` ⇒ có `trajectory_xy.png` + `.pdf`.
- [ ] Điền `ATE RMSE` và `RPE 1 m RMSE` vào dòng #1 trong `RESULTS.md`,
      đổi trạng thái `pending` → `confirmed`.
- [ ] Commit cả `RESULTS.md` + đường dẫn ảnh paper vào git
      (chỉ commit `RESULTS.md` và link mô tả; ảnh PNG/PDF không vào git vì `results/` đã gitignore — copy ảnh sang `docs/paper/figures/` nếu cần lưu trong git).

## C. EuRoC — biến thể để trả lời "kết quả có tốt hơn không?"

> Mỗi biến thể là một dòng mới trong §1 `RESULTS.md`, **không ghi đè dòng baseline**.

Chỉ chạy các biến thể mà nhóm thực sự dự định viết vào báo cáo. Đề xuất tối thiểu:

- [ ] `euroc_V1_01_easy_no_gt_init` — tắt `use_gt_initialization` (đặt false). Trả lời: ảnh hưởng init bias / gravity ra sao.
- [ ] `euroc_V1_01_easy_imu_init_100` — `imu_init_sample_count:=100` (giảm từ 200). Trả lời: trade-off init speed vs accuracy.
- [ ] (tuỳ chọn) `euroc_V1_01_easy_meas_noise_1e-3` — `measurement_noise: 0.001` (gấp 10× baseline). Trả lời: độ nhạy với measurement noise.

Mỗi biến thể: lặp lại đầy đủ block §3.A, đổi `RUN_TAG` và override param tương ứng qua `--ros-args -p ...` (hoặc edit `vio_euroc.yaml` rồi rebuild).

## D. EuRoC — sequence khác (nếu thời gian cho phép, optional cho báo cáo CO3107)

- [ ] Tải `V1_02_medium`, `V2_01_easy` về `dataset/`.
- [ ] Tạo `RUN_TAG=euroc_V1_02_medium`, lặp §3.A với `dataset_dir` mới.
- [ ] Bổ sung dòng tương ứng vào `RESULTS.md`.

> Nếu không có thời gian, **không bịa số cho các sequence chưa chạy**. Báo cáo chỉ ghi V1_01_easy.

## E. HCMUT smoke test (BẮT BUỘC — để xác nhận pipeline live)

- [ ] Chạy đúng block §3.B trong `RESULTS.md`, `RUN_TAG=hcmut_smoke`.
- [ ] Kiểm log thấy `Camera calibration: fx=646.337..., ...` và `Input sensor QoS reliability: best_effort`.
- [ ] Kiểm `/vio/odometry` có > 0 message (lệnh sqlite3 ở §3.B).
- [ ] **Không** chạy `evaluate_m4.py`. **Không** điền số vào ATE/RPE.
- [ ] Cập nhật ô `Ghi chú` của dòng #2 với: thời lượng trajectory trước khi diverge, số message odometry, lý do hard-stop (nếu có).
- [ ] Trạng thái phải vẫn là `smoke-test`. **Sếp / report reviewer phải đọc được rằng HCMUT KHÔNG phải accuracy result.**

## F. Kiểm tra trước khi data-freeze (chiều 2026-05-30)

- [ ] Mỗi dòng `confirmed` trong `RESULTS.md` có đủ: lệnh, output dir tồn tại, ATE, RPE, commit hash.
- [ ] Mỗi dòng `smoke-test` ghi rõ **lý do không tính accuracy**.
- [ ] `RESULTS.md` không còn ô `_PENDING_` cho bất kỳ dòng nào sẽ vào báo cáo.
- [ ] Ít nhất 1 ảnh trajectory (`trajectory_xy.png` của baseline) được copy vào `docs/paper/figures/` và commit vào git.
- [ ] Tag git commit `data-freeze-week2`: `git tag -a data-freeze-week2 -m "Week 2 data freeze for CO3107 report"`
- [ ] Báo trong nhóm chat: "Đã freeze. Số liệu lấy từ `RESULTS.md` @ tag `data-freeze-week2`."

## G. Việc còn thiếu (gating cho dùng HCMUT làm accuracy — KHÔNG nằm trong scope Tuần 2)

Chỉ ghi nhận để khỏi quên. Không cần làm trong tuần 2:

- [ ] Lấy `tf_static` thực giữa `camera_color_optical_frame` và IMU frame của D455 (qua `librealsense2_camera` hoặc `ros2 run tf2_ros tf2_echo`).
- [ ] Hard-code transform đó vào `state_server.py` hoặc expose qua param.
- [ ] Có ground-truth cho HCMUT scene (MoCap, hoặc VICON, hoặc loop-closure SLAM tham chiếu).
- [ ] Khi cả 2 đủ ⇒ mở `RUN_TAG=hcmut_v1` mới, mở dòng mới trong `RESULTS.md`, chạy `evaluate_m4.py` như EuRoC.

---

## Định nghĩa "Done" của Tuần 2

Nhóm có thể trả lời `kết quả có tốt hơn không?` chỉ bằng cách mở `RESULTS.md`:
mỗi con số đều có **lệnh chạy + thư mục output + commit hash**, HCMUT không
bị nhầm là accuracy, và có ít nhất một trajectory plot dùng được trong paper.
