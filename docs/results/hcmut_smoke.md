# HCMUT `vio_hcmut_dataset` smoke test (Task 3)

**Trạng thái:** `smoke-test` — pipeline live, **KHÔNG dùng làm accuracy result**.
**Commit:** `e8425eb`.
**Ngày chạy:** 2026-05-27 ~23:13 ICT.
**Lệnh tái chạy:** [`eval_commands.md` §B](eval_commands.md#b--hcmut-vio_hcmut_dataset-smoke-test).

## Lý do KHÔNG báo accuracy từ dataset này

1. **Camera–IMU extrinsics chưa xác nhận.** Node đang dùng default EuRoC trong `backend/state_server.py`. Cần lấy TF static thật giữa `camera_color_optical_frame` và IMU frame của D455 (qua `librealsense2_camera` hoặc `ros2 run tf2_ros tf2_echo`).
2. **Trajectory diverge nhanh.** Quan sát thực tế run này: `‖pos‖` vượt 10 m sau **~12.6 s** sim time; cuối run lên đến **8 836 m**.
3. **Không có ground-truth.** Bag không có topic GT, cũng không có MoCap / Vicon reference. `evaluate_m4.py` sẽ ra số vô nghĩa — **không chạy**.

⇒ Dùng dataset này chỉ để **kiểm tra pipeline còn chạy** (subscribe, estimate, publish), không để định lượng độ chính xác.

## Quan sát thực tế (run 2026-05-27)

| Mục | Giá trị |
|---|---|
| Wall clock tổng | 66.0 s |
| Sim duration cover được | 48.06 s (bag 53.55 s — node mất ~5 s đầu để init) |
| Messages `/vio/odometry` xuất ra | **221** |
| Pose đầu | `t=1776579935.879, pos=(0, 0, 0)` |
| Pose cuối | `t=1776579983.935, pos=(+5450, +4597, -5219)` |
| Max `‖pos‖` | **8 835.6 m** |
| Diverge (`‖pos‖ > 10 m`) | sim t ≈ **+12.64 s** sau pose đầu |
| Range x / y / z | 5 450 / 4 597 / 5 219 m |
| QoS | `best_effort` (D455 publish với reliability=1) |
| Camera intrinsics | D455 color (fx=646.34, fy=645.68, cx=643.36, cy=363.00) |
| Camera–IMU extrinsics | **DEFAULT EUROC** (lý do diverge) |
| Camera distortion | từ `/camera/camera/color/camera_info` (5 hệ số Brown-Conrady) |

> Tóm gọn: pipeline subscribe được cả 2 topic D455, frontend track feature, backend cập nhật state — nhưng vì extrinsics sai, state blow up rất nhanh.

## Điều kiện gating để nâng lên `confirmed`

Trước khi dòng này được tính accuracy (và viết vào báo cáo / paper), phải có **cả** 3:

- [ ] TF static thật `camera_color_optical_frame ↔ camera_imu_optical_frame` (từ `librealsense` hoặc `/tf_static`); hard-code vào `state_server.py` hoặc expose qua param.
- [ ] Ground-truth cho HCMUT scene (MoCap, VICON, hoặc loop-closure SLAM reference).
- [ ] Một run mới `RUN_TAG=hcmut_v1` chạy `evaluate_m4.py` ra `ape_aligned.zip` + `rpe_1m_aligned.zip` không lỗi.

⇒ Khi cả 3 đủ, mở **dòng mới** trong [`table.md`](table.md) (không sửa dòng smoke hiện tại).

Việc này **không nằm trong scope Tuần 2** — xem [`week2_checklist.md` §G](week2_checklist.md).

## Artifact

```
results/hcmut_smoke/
├── bag/              # 221 /vio/odometry messages
├── node.log          # contains the divergence trajectory + ExternalShutdownException at end (normal)
├── play.log
└── record.log
```

> Không có `eval/` — không chạy evo với dataset không có GT.

## Tóm tắt cho báo cáo (nếu cần một dòng)

> *Pipeline đã được smoke-test trên dataset RealSense D455 HCMUT
> (`vio_hcmut_dataset`, 53 s, 221 pose ước lượng); tuy nhiên do chưa có
> camera–IMU extrinsics thật cho D455 nên state nhanh chóng diverge
> (`‖pos‖` > 10 m sau ~12.6 s sim time). Không bao gồm trong đánh giá
> định lượng — chỉ xác nhận pipeline chạy end-to-end.*
