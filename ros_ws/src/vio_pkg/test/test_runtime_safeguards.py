"""
Runtime safeguard checks for VIO overload and MSCKF update sanity.

Run from the ROS workspace root:
    python3 src/vio_pkg/test/test_runtime_safeguards.py
"""

import os
import queue
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vio_pkg.backend.msckf_updater import MSCKFUpdater
from vio_pkg.backend.propagator import ImuPropagator
from vio_pkg.backend.state_server import StateServer
from vio_pkg.utils.common import ImuData
from vio_pkg.vio_node import (
    compute_effective_camera_calibration,
    make_sensor_qos,
    prepare_image_for_processing,
    put_latest_image,
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


if __name__ == "__main__":
    test_put_latest_image_drops_oldest_when_queue_is_full()
    test_sensor_qos_defaults_to_reliable_for_euroc_bag_playback()
    test_sensor_qos_can_opt_into_best_effort_for_realsense_bags()
    test_prepare_image_for_processing_resizes_d455_to_euroc_like_width()
    test_prepare_image_for_processing_keeps_full_resolution_when_disabled()
    test_effective_camera_calibration_scales_intrinsics_not_distortion()
    test_imu_propagator_skips_unreasonably_large_dt_discontinuity()
    test_msckf_rejects_unreasonably_large_batch_update_before_mutating_state()
    print("Runtime safeguard tests passed.")
