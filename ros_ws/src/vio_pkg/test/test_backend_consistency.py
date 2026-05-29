"""
Backend consistency checks for camera clone handling.

Run from the ROS workspace root:
    python3 src/vio_pkg/test/test_backend_consistency.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vio_pkg.backend.math_utils import skew_symmetric
from vio_pkg.backend.msckf_updater import MSCKFUpdater
from vio_pkg.backend.state_server import (
    StateServer,
    canonicalize_camera_extrinsics,
)
from vio_pkg.utils.common import CameraPose, FeatureTrack


def _make_server() -> StateServer:
    server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
    server.max_window_size = 10
    return server


def _add_identity_clone(server: StateServer, timestamp: float, position: np.ndarray):
    server.state.position = position.copy()
    server.state.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    server.add_clone(timestamp, server.state.position, server.state.quaternion)


def _project(updater: MSCKFUpdater, point_w: np.ndarray, camera_position: np.ndarray):
    f_c = point_w - camera_position
    return np.array([
        updater.fx * f_c[0] / f_c[2] + updater.cx,
        updater.fy * f_c[1] / f_c[2] + updater.cy,
    ])


def _project_distorted(
    updater: MSCKFUpdater,
    point_w: np.ndarray,
    camera_position: np.ndarray,
):
    f_c = point_w - camera_position
    x = f_c[0] / f_c[2]
    y = f_c[1] / f_c[2]
    k1, k2, p1, p2 = updater.distortion_coefficients
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2
    x_d = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    y_d = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return np.array([
        updater.fx * x_d + updater.cx,
        updater.fy * y_d + updater.cy,
    ])


def test_clone_augmentation_keeps_lever_arm_orientation_coupling():
    t_ic = np.array([0.2, -0.1, 0.05])
    server = StateServer(R_IC=np.eye(3), t_IC=t_ic)

    position = np.array([1.0, 2.0, 3.0])
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    old_covariance = server.covariance.copy()
    server.add_clone(1.0, position, quaternion)

    expected_pos_theta = -skew_symmetric(t_ic) @ old_covariance[6:9, 6:9]
    actual_pos_theta = server.covariance[18:21, 6:9]

    assert np.allclose(actual_pos_theta, expected_pos_theta)
    assert not np.allclose(actual_pos_theta, 0.0)


def test_msckf_uses_authoritative_state_server_clones_not_frontend_snapshots():
    server = _make_server()
    updater = MSCKFUpdater(server)
    updater.distortion_coefficients = np.zeros(4)
    point_w = np.array([0.5, 0.1, 5.0])

    timestamps = [1.0, 2.0, 3.0]
    positions = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.2, 0.0, 0.0]),
        np.array([0.4, 0.0, 0.0]),
    ]
    for timestamp, position in zip(timestamps, positions):
        _add_identity_clone(server, timestamp, position)

    observations = [_project(updater, point_w, position) for position in positions]
    stale_camera_states = [
        CameraPose(
            timestamp=timestamp,
            position=np.array([100.0, 100.0, 100.0]),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        for timestamp in timestamps
    ]
    feature = FeatureTrack(
        feature_id=1,
        observations=observations,
        camera_states=stale_camera_states,
    )

    triangulated = updater.triangulate_feature(feature)
    assert triangulated is not None
    assert np.allclose(triangulated, point_w, atol=1e-6)

    H_x, H_f, residual, valid = updater.calc_residuals_and_jacobian(
        triangulated, feature
    )
    assert valid
    assert H_x.shape == (6, 33)
    assert H_f.shape == (6, 3)
    assert np.linalg.norm(residual) < 1e-9


def test_msckf_undistorts_euroc_observations_before_triangulation():
    server = _make_server()
    updater = MSCKFUpdater(server)
    point_w = np.array([1.3, 0.8, 4.0])

    timestamps = [1.0, 2.0, 3.0, 4.0]
    positions = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.3, 0.0, 0.0]),
        np.array([0.6, 0.0, 0.0]),
        np.array([0.9, 0.0, 0.0]),
    ]
    for timestamp, position in zip(timestamps, positions):
        _add_identity_clone(server, timestamp, position)

    observations = [
        _project_distorted(updater, point_w, position)
        for position in positions
    ]
    feature = FeatureTrack(
        feature_id=3,
        observations=observations,
        camera_states=[
            CameraPose(timestamp, position, np.array([1.0, 0.0, 0.0, 0.0]))
            for timestamp, position in zip(timestamps, positions)
        ],
    )

    triangulated = updater.triangulate_feature(feature)
    assert triangulated is not None
    assert np.allclose(triangulated, point_w, atol=1e-2)

    _, _, residual, valid = updater.calc_residuals_and_jacobian(
        triangulated, feature
    )
    assert valid
    assert np.linalg.norm(residual) < 1e-6


def test_msckf_camera_calibration_can_be_configured_for_hcmut_bag():
    server = _make_server()
    updater = MSCKFUpdater(server)

    updater.set_camera_calibration(
        fx=646.33728,
        fy=645.676147,
        cx=643.358276,
        cy=362.999176,
        distortion_coefficients=[
            -0.05594548583030701,
            0.06458555161952972,
            -0.0002526374883018434,
            0.0008183500613085926,
            -0.021141313016414642,
        ],
    )

    assert updater.fx == 646.33728
    assert updater.fy == 645.676147
    assert updater.cx == 643.358276
    assert updater.cy == 362.999176
    assert np.allclose(
        updater.camera_matrix,
        np.array([
            [646.33728, 0.0, 643.358276],
            [0.0, 645.676147, 362.999176],
            [0.0, 0.0, 1.0],
        ]),
    )
    assert updater.distortion_coefficients.shape == (5,)


def test_state_server_camera_extrinsics_can_be_configured_for_hcmut_bag():
    server = _make_server()
    r_ic = np.array([
        [2.033476571297e-03, 1.534356741860e-04, 9.999979207131e-01],
        [-9.999966540050e-01, -1.598738420073e-03, 2.033719299488e-03],
        [1.599047140929e-03, -9.999987102456e-01, 1.501841636702e-04],
    ])
    t_ic = np.array([0.015779949041, -0.028793809935, -0.007352355558])

    server.set_camera_extrinsics(r_ic, t_ic)

    assert np.allclose(server.R_IC, r_ic)
    assert np.allclose(server.t_IC, t_ic)


def test_state_server_can_canonicalize_imu_in_camera_extrinsics():
    r_ic = np.array([
        [2.033476571297e-03, 1.534356741860e-04, 9.999979207131e-01],
        [-9.999966540050e-01, -1.598738420073e-03, 2.033719299488e-03],
        [1.599047140929e-03, -9.999987102456e-01, 1.501841636702e-04],
    ])
    t_ic = np.array([0.015779949041, -0.028793809935, -0.007352355558])
    r_ci = r_ic.T
    t_ci = -(r_ci @ t_ic)

    canonical_r_ic, canonical_t_ic = canonicalize_camera_extrinsics(
        r_ci,
        t_ci,
        convention="imu_in_camera",
    )

    assert np.allclose(canonical_r_ic, r_ic)
    assert np.allclose(canonical_t_ic, t_ic)


def test_add_clone_matches_between_equivalent_extrinsics_conventions():
    r_ic = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    t_ic = np.array([0.2, -0.1, 0.05])
    r_ci = r_ic.T
    t_ci = -(r_ci @ t_ic)
    position = np.array([1.0, 2.0, 3.0])
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    server_ic = StateServer(R_IC=r_ic, t_IC=t_ic)
    server_ci = StateServer(
        R_IC=r_ci,
        t_IC=t_ci,
        camera_extrinsics_convention="imu_in_camera",
    )
    server_ic.add_clone(1.0, position, quaternion)
    server_ci.add_clone(1.0, position, quaternion)

    clone_ic = server_ic.state.clone_poses[-1]
    clone_ci = server_ci.state.clone_poses[-1]

    assert np.allclose(clone_ic.position, clone_ci.position)
    assert np.allclose(clone_ic.quaternion, clone_ci.quaternion)
    assert np.allclose(server_ic.covariance, server_ci.covariance)


def test_msckf_rejects_features_when_required_clone_was_marginalized():
    server = _make_server()
    updater = MSCKFUpdater(server)
    updater.distortion_coefficients = np.zeros(4)
    _add_identity_clone(server, 1.0, np.zeros(3))
    _add_identity_clone(server, 2.0, np.array([0.2, 0.0, 0.0]))

    feature = FeatureTrack(
        feature_id=2,
        observations=[
            np.array([updater.cx, updater.cy]),
            np.array([updater.cx, updater.cy]),
            np.array([updater.cx, updater.cy]),
        ],
        camera_states=[
            CameraPose(1.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])),
            CameraPose(2.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])),
            CameraPose(3.0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])),
        ],
    )

    assert updater.triangulate_feature(feature) is None
    H_x, H_f, residual, valid = updater.calc_residuals_and_jacobian(
        np.array([0.0, 0.0, 5.0]), feature
    )
    assert not valid
    assert H_x is None
    assert H_f is None
    assert residual is None


if __name__ == "__main__":
    test_clone_augmentation_keeps_lever_arm_orientation_coupling()
    test_msckf_uses_authoritative_state_server_clones_not_frontend_snapshots()
    test_msckf_undistorts_euroc_observations_before_triangulation()
    test_msckf_camera_calibration_can_be_configured_for_hcmut_bag()
    test_state_server_camera_extrinsics_can_be_configured_for_hcmut_bag()
    test_state_server_can_canonicalize_imu_in_camera_extrinsics()
    test_add_clone_matches_between_equivalent_extrinsics_conventions()
    test_msckf_rejects_features_when_required_clone_was_marginalized()
    print("Backend consistency tests passed.")
