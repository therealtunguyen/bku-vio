"""
Runtime safeguard checks for VIO overload and MSCKF update sanity.

Run from the ROS workspace root:
    python3 src/vio_pkg/test/test_runtime_safeguards.py
"""

import os
import queue
import sys
import threading

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vio_pkg.backend.msckf_updater import MSCKFUpdater
from vio_pkg.backend.propagator import ImuPropagator
from vio_pkg.backend.state_server import StateServer
from vio_pkg.utils.common import ImuData
from vio_pkg.vio_node import (
    VIOSystemNode,
    compute_static_imu_initialization,
    compute_effective_camera_calibration,
    is_frame_timestamp_discontinuity,
    make_sensor_qos,
    prepare_image_for_processing,
    put_latest_image,
    sanitize_time_gap_threshold,
    update_initial_imu_buffer,
)
from test_hcmut_propagation_replay import (
    DEFAULT_CAMERA_EXTRINSICS_CONVENTION,
    DEFAULT_CAMERA_R_IC,
    DEFAULT_CAMERA_T_IC,
    DEFAULT_MAX_BATCH_DX_BIAS_NORM,
    DEFAULT_MIN_TRIANGULATION_PARALLAX_DEG,
)


def test_put_latest_image_drops_oldest_when_queue_is_full():
    image_queue = queue.Queue(maxsize=1)
    dropped = put_latest_image(image_queue, ("t0", "old"))

    assert dropped is False
    assert image_queue.qsize() == 1

    dropped = put_latest_image(image_queue, ("t1", "new"))

    assert dropped is True
    assert image_queue.qsize() == 1
    assert image_queue.get_nowait() == ("t1", "new")


def test_sensor_qos_defaults_to_reliable_for_euroc_bag_playback():
    qos = make_sensor_qos(depth=10)

    assert qos.reliability.name == "RELIABLE"


def test_sensor_qos_can_opt_into_best_effort_for_realsense_bags():
    qos = make_sensor_qos(depth=10, reliability_name="best_effort")

    assert qos.reliability.name == "BEST_EFFORT"


def test_prepare_image_for_processing_resizes_d455_to_euroc_like_width():
    image = np.zeros((720, 1280), dtype=np.uint8)

    processed, scale = prepare_image_for_processing(image, target_width=752)

    assert processed.shape == (423, 752)
    assert np.isclose(scale, 752.0 / 1280.0)


def test_prepare_image_for_processing_keeps_full_resolution_when_disabled():
    image = np.zeros((720, 1280), dtype=np.uint8)

    processed, scale = prepare_image_for_processing(image, target_width=0)

    assert processed.shape == image.shape
    assert scale == 1.0


def test_effective_camera_calibration_scales_intrinsics_not_distortion():
    distortion = [
        -0.05594548583030701,
        0.06458555161952972,
        -0.0002526374883018434,
        0.0008183500613085926,
        -0.021141313016414642,
    ]

    fx, fy, cx, cy, effective_distortion = compute_effective_camera_calibration(
        fx=646.33728,
        fy=645.676147,
        cx=643.358276,
        cy=362.999176,
        distortion_coefficients=distortion,
        scale=752.0 / 1280.0,
    )

    assert np.isclose(fx, 646.33728 * 752.0 / 1280.0)
    assert np.isclose(fy, 645.676147 * 752.0 / 1280.0)
    assert np.isclose(cx, 643.358276 * 752.0 / 1280.0)
    assert np.isclose(cy, 362.999176 * 752.0 / 1280.0)
    assert np.allclose(effective_distortion, distortion)


def test_hcmut_replay_defaults_match_corrected_runtime_preset():
    expected_r_ic = np.array([
        [0.999996654005, 0.00159873842, -0.002033719299],
        [-0.001599047141, 0.999998710246, -0.000150184164],
        [0.002033476571, 0.000153435674, 0.999997920713],
    ])
    expected_t_ic = np.array([0.028793809935, 0.007352355558, 0.015779949041])

    assert DEFAULT_CAMERA_EXTRINSICS_CONVENTION == "camera_in_imu"
    assert np.allclose(np.array(DEFAULT_CAMERA_R_IC).reshape(3, 3), expected_r_ic)
    assert np.allclose(np.array(DEFAULT_CAMERA_T_IC), expected_t_ic)
    assert DEFAULT_MAX_BATCH_DX_BIAS_NORM == 0.06
    assert DEFAULT_MIN_TRIANGULATION_PARALLAX_DEG == 2.0


def test_hcmut_launch_preset_uses_same_bias_rail_as_replay_default():
    launch_path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "launch",
        "vio_hcmut.launch.py",
    )
    with open(launch_path, "r", encoding="utf-8") as handle:
        launch_text = handle.read()

    assert f'"max_batch_dx_bias_norm": {DEFAULT_MAX_BATCH_DX_BIAS_NORM}' in launch_text
    assert (
        f'"min_triangulation_parallax_deg": '
        f"{DEFAULT_MIN_TRIANGULATION_PARALLAX_DEG}"
    ) in launch_text


def test_imu_propagator_skips_unreasonably_large_dt_discontinuity():
    server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
    propagator = ImuPropagator(server)
    propagator.max_imu_dt = 0.05
    server.state.timestamp = 1.0
    initial_position = server.state.position.copy()
    initial_velocity = server.state.velocity.copy()

    propagator.propagate([
        ImuData(
            timestamp=4.5,
            accel=np.array([0.0, 0.0, 9.81]),
            gyro=np.zeros(3),
        )
    ])

    assert np.allclose(server.state.position, initial_position)
    assert np.allclose(server.state.velocity, initial_velocity)
    assert server.state.timestamp == 4.5
    assert propagator.last_large_dt_count == 1


def test_msckf_rejects_unreasonably_large_batch_update_before_mutating_state():
    server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
    updater = MSCKFUpdater(server)
    updater.max_batch_dx_pos_norm = 0.05

    initial_position = server.state.position.copy()
    initial_covariance = server.covariance.copy()

    H = np.zeros((1, server.covariance.shape[0]))
    H[0, 0] = 1.0
    residual = np.array([1.0])

    dx = updater.measurement_update(H, residual)

    assert np.linalg.norm(dx[0:3]) > updater.max_batch_dx_pos_norm
    assert np.allclose(server.state.position, initial_position)
    assert np.allclose(server.covariance, initial_covariance)


def test_msckf_update_stats_report_ill_conditioned_rejection_reason():
    server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
    updater = MSCKFUpdater(server)
    updater.max_update_condition_number = 1.0

    H = np.eye(server.covariance.shape[0])
    residual = np.ones(server.covariance.shape[0])

    updater.measurement_update(H, residual)

    assert updater.last_batch_rejected is True
    assert updater.last_rejection_reason == "ill_conditioned"
    assert updater.last_innovation_condition_number > updater.max_update_condition_number


def test_msckf_update_stats_report_unreasonable_dx_rejection_reason():
    server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
    updater = MSCKFUpdater(server)
    updater.max_batch_dx_bias_norm = 1e-6

    H = np.zeros((2, server.covariance.shape[0]))
    H[:, 9:11] = np.eye(2)
    residual = np.array([1.0, 1.0])

    updater.measurement_update(H, residual)

    assert updater.last_batch_rejected is True
    assert updater.last_rejection_reason == "unreasonable_dx"
    assert updater.last_dx_bg_norm > updater.max_batch_dx_bias_norm


def test_vio_node_can_skip_msckf_updates_without_skipping_frame_processing():
    node = object.__new__(VIOSystemNode)
    node.enable_msckf_updates = False
    node.msckf_updater = type(
        "DummyUpdater",
        (),
        {
            "process_mature_features": lambda self, features: (_ for _ in ()).throw(
                AssertionError("MSCKF update should be skipped")
            ),
            "last_update_stats": {"accepted": 123},
        },
    )()

    update_stats = VIOSystemNode._run_msckf_update(node, ["feature"])

    assert update_stats["skipped"] == 1
    assert update_stats["mature"] == 1
    assert update_stats["accepted"] == 0


def test_vio_node_requests_stop_after_requested_processed_frame_count():
    node = object.__new__(VIOSystemNode)
    node.stop_after_processed_frames = 2
    node.images_processed = 1

    assert VIOSystemNode._should_request_stop(node) is False

    node.images_processed = 2

    assert VIOSystemNode._should_request_stop(node) is True


def test_update_initial_imu_buffer_resets_on_large_timestamp_gap():
    first = ImuData(
        timestamp=1.0,
        accel=np.zeros(3),
        gyro=np.zeros(3),
    )
    second = ImuData(
        timestamp=1.2,
        accel=np.ones(3),
        gyro=np.ones(3),
    )

    updated, was_reset, gap = update_initial_imu_buffer([first], second, 0.1)

    assert was_reset is True
    assert np.isclose(gap, 0.2)
    assert updated == [second]


def test_static_imu_initialization_matches_level_stationary_case():
    samples = [
        ImuData(
            timestamp=1.0 + 0.01 * idx,
            accel=np.array([0.0, 0.0, 9.81]),
            gyro=np.array([0.01, -0.02, 0.03]),
        )
        for idx in range(5)
    ]

    q_init, gyro_bias, accel_bias, init_end_time = compute_static_imu_initialization(
        samples
    )

    assert np.allclose(q_init, np.array([1.0, 0.0, 0.0, 0.0]))
    assert np.allclose(gyro_bias, np.array([0.01, -0.02, 0.03]))
    assert np.allclose(accel_bias, np.zeros(3), atol=1e-8)
    assert np.isclose(init_end_time, samples[-1].timestamp)


def test_frame_timestamp_discontinuity_detects_backward_and_large_forward_jumps():
    assert is_frame_timestamp_discontinuity(10.0, 9.5, 0.1) is True
    assert is_frame_timestamp_discontinuity(10.0, 10.25, 0.1) is True
    assert is_frame_timestamp_discontinuity(10.0, 10.03, 0.1) is False


def test_frame_timestamp_discontinuity_can_tolerate_brief_frame_drops():
    assert is_frame_timestamp_discontinuity(10.0, 10.1667, 0.2) is False
    assert is_frame_timestamp_discontinuity(10.0, 10.2501, 0.2) is True


def test_sanitize_time_gap_threshold_falls_back_for_non_positive_values():
    assert np.isclose(sanitize_time_gap_threshold(0.2, 0.1), 0.2)
    assert np.isclose(sanitize_time_gap_threshold(0.0, 0.1), 0.1)
    assert np.isclose(sanitize_time_gap_threshold(-1.0, 0.1), 0.1)


def test_state_server_rejects_unknown_camera_extrinsics_convention():
    server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))

    try:
        server.set_camera_extrinsics(
            np.eye(3),
            np.zeros(3),
            convention="not_a_real_convention",
        )
    except ValueError as exc:
        assert "convention" in str(exc)
    else:
        raise AssertionError("Expected ValueError for unknown convention")


def _make_dummy_frontend():
    return type(
        "DummyFrontend",
        (),
        {
            "active_tracks": [object()],
            "mature_tracks": [object()],
            "_prev_image": np.zeros((2, 2), dtype=np.uint8),
            "reset": lambda self: (
                self.active_tracks.clear(),
                self.mature_tracks.clear(),
                setattr(self, "_prev_image", None),
            ),
        },
    )()


def _make_dummy_logger():
    return type(
        "DummyLogger",
        (),
        {"warn": lambda self, msg: None},
    )()


def test_vio_node_does_not_reset_on_large_forward_gap_when_disabled():
    node = object.__new__(VIOSystemNode)
    node.frontend = _make_dummy_frontend()
    node.state_server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
    node.state_server.add_clone(1.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    node.state_server.state.timestamp = 12.0
    node.last_image_time = 12.0
    node._discontinuity_reset_count = 0
    node.max_frame_timestamp_gap = 0.1
    node.reset_on_large_frame_gap = False
    node.imu_lock = threading.Lock()
    node.imu_buffer = [
        ImuData(timestamp=12.05, accel=np.zeros(3), gyro=np.zeros(3)),
        ImuData(timestamp=12.40, accel=np.zeros(3), gyro=np.zeros(3)),
    ]
    node.get_logger = lambda: _make_dummy_logger()

    should_reset = VIOSystemNode._should_reset_for_frame_gap(node, 0.2)

    assert should_reset is False
    assert node.state_server.state.timestamp == 12.0
    assert len(node.state_server.state.clone_poses) == 1
    assert [imu.timestamp for imu in node.imu_buffer] == [12.05, 12.40]
    assert len(node.frontend.active_tracks) == 1
    assert len(node.frontend.mature_tracks) == 1
    assert node.frontend._prev_image is not None
    assert node._discontinuity_reset_count == 0


def test_vio_node_resets_on_large_forward_gap_when_enabled():
    node = object.__new__(VIOSystemNode)
    node.frontend = _make_dummy_frontend()
    node.state_server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
    node.state_server.add_clone(1.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    node.state_server.state.timestamp = 12.0
    node.last_image_time = 12.0
    node._discontinuity_reset_count = 0
    node.max_frame_timestamp_gap = 0.1
    node.reset_on_large_frame_gap = True
    node.imu_lock = threading.Lock()
    node.imu_buffer = [
        ImuData(timestamp=12.05, accel=np.zeros(3), gyro=np.zeros(3)),
        ImuData(timestamp=12.40, accel=np.zeros(3), gyro=np.zeros(3)),
    ]
    node.get_logger = lambda: _make_dummy_logger()

    should_reset = VIOSystemNode._should_reset_for_frame_gap(node, 0.2)

    assert should_reset is True

    VIOSystemNode._handle_frame_timestamp_discontinuity(node, 12.2, 0.2)

    assert node.last_image_time == 12.2
    assert node.state_server.state.timestamp == 12.2
    assert node.state_server.state.clone_poses == []
    assert node.state_server.covariance.shape == (15, 15)
    assert [imu.timestamp for imu in node.imu_buffer] == [12.40]
    assert node.frontend.active_tracks == []
    assert node.frontend.mature_tracks == []
    assert node.frontend._prev_image is None
    assert node._discontinuity_reset_count == 1


def test_vio_node_resets_temporal_state_on_backward_timestamp_discontinuity():
    node = object.__new__(VIOSystemNode)
    node.frontend = _make_dummy_frontend()
    node.state_server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
    node.state_server.add_clone(1.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    node.state_server.state.timestamp = 12.0
    node.last_image_time = 12.0
    node._discontinuity_reset_count = 0
    node.max_frame_timestamp_gap = 0.1
    node.reset_on_large_frame_gap = False
    node.imu_lock = threading.Lock()
    node.imu_buffer = [
        ImuData(timestamp=8.5, accel=np.zeros(3), gyro=np.zeros(3)),
        ImuData(timestamp=9.5, accel=np.zeros(3), gyro=np.zeros(3)),
    ]
    node.get_logger = lambda: _make_dummy_logger()

    VIOSystemNode._handle_frame_timestamp_discontinuity(node, 9.0, -3.0)

    assert node.last_image_time == 9.0
    assert node.state_server.state.timestamp == 9.0
    assert node.state_server.state.clone_poses == []
    assert node.state_server.covariance.shape == (15, 15)
    assert [imu.timestamp for imu in node.imu_buffer] == [9.5]
    assert node.frontend.active_tracks == []
    assert node.frontend.mature_tracks == []
    assert node.frontend._prev_image is None
    assert node._discontinuity_reset_count == 1


if __name__ == "__main__":
    test_put_latest_image_drops_oldest_when_queue_is_full()
    test_sensor_qos_defaults_to_reliable_for_euroc_bag_playback()
    test_sensor_qos_can_opt_into_best_effort_for_realsense_bags()
    test_prepare_image_for_processing_resizes_d455_to_euroc_like_width()
    test_prepare_image_for_processing_keeps_full_resolution_when_disabled()
    test_effective_camera_calibration_scales_intrinsics_not_distortion()
    test_imu_propagator_skips_unreasonably_large_dt_discontinuity()
    test_msckf_rejects_unreasonably_large_batch_update_before_mutating_state()
    test_msckf_update_stats_report_ill_conditioned_rejection_reason()
    test_msckf_update_stats_report_unreasonable_dx_rejection_reason()
    test_vio_node_can_skip_msckf_updates_without_skipping_frame_processing()
    test_vio_node_requests_stop_after_requested_processed_frame_count()
    test_update_initial_imu_buffer_resets_on_large_timestamp_gap()
    test_static_imu_initialization_matches_level_stationary_case()
    test_frame_timestamp_discontinuity_detects_backward_and_large_forward_jumps()
    test_frame_timestamp_discontinuity_can_tolerate_brief_frame_drops()
    test_sanitize_time_gap_threshold_falls_back_for_non_positive_values()
    test_state_server_rejects_unknown_camera_extrinsics_convention()
    test_vio_node_does_not_reset_on_large_forward_gap_when_disabled()
    test_vio_node_resets_on_large_forward_gap_when_enabled()
    test_vio_node_resets_temporal_state_on_backward_timestamp_discontinuity()
    print("Runtime safeguard tests passed.")
