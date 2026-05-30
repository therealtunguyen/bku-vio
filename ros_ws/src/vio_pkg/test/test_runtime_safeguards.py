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
from vio_pkg.backend.state_server import StateServer
from vio_pkg.vio_node import make_sensor_qos, put_latest_image


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
    test_msckf_rejects_unreasonably_large_batch_update_before_mutating_state()
    print("Runtime safeguard tests passed.")
