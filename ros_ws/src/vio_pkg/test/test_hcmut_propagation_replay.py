"""
Deterministic HCMUT/D455 propagation-only replay harness.

Reads IMU and image messages directly from the ROS 2 SQLite bag, sorts them by
header timestamp, and replays them through the same gravity-init and
propagation helpers used by VIOSystemNode.

Run inside the container:

    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/VIO/ros_ws
    python3 src/vio_pkg/test/test_hcmut_propagation_replay.py
"""

from __future__ import annotations

import argparse
import copy
import os
import sqlite3
import sys
from dataclasses import dataclass, field

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vio_pkg.backend.msckf_updater import MSCKFUpdater
from vio_pkg.backend.propagator import ImuPropagator
from vio_pkg.backend.state_server import StateServer
from vio_pkg.backend.math_utils import quaternion_to_matrix
from vio_pkg.frontend.detectors import ShiTomasiDetector
from vio_pkg.frontend.feature_manager import FeatureManager
from vio_pkg.frontend.interfaces import FrontendConfig
from vio_pkg.frontend.trackers import KLTTracker
from vio_pkg.utils.common import CameraPose
from vio_pkg.utils.common import ImuData
from vio_pkg.vio_node import (
    compute_effective_camera_calibration,
    compute_static_imu_initialization,
    is_frame_timestamp_discontinuity,
    prepare_image_for_processing,
    sanitize_time_gap_threshold,
    update_initial_imu_buffer,
)

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    HAS_ROS = True
except ImportError:
    HAS_ROS = False


DEFAULT_BAG_PATHS = (
    "/home/ubuntu/VIO/dataset/vio_hcmut_dataset",
    "/home/tyler/Desktop/bku-vio/dataset/vio_hcmut_dataset",
)
DEFAULT_IMU_TOPIC = "/camera/camera/imu"
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_IMU_INIT_SAMPLES = 120
DEFAULT_MAX_IMU_INIT_GAP = 0.2
DEFAULT_MAX_FRAME_GAP = 0.25
DEFAULT_MAX_IMU_DT = 0.05
DEFAULT_FULL_STOP_IMAGE = 260
DEFAULT_SNAPSHOT_IMAGE = 150
DEFAULT_SEGMENT_STOP_IMAGE = 240
DRIFT_THRESHOLD = 0.15
DEFAULT_REPLAY_MODE = "propagation"
VALID_REPLAY_MODES = ("propagation", "msckf")
DEFAULT_CAMERA_EXTRINSICS_CONVENTION = "camera_in_imu"
VALID_CAMERA_EXTRINSICS_CONVENTIONS = (
    "camera_in_imu",
    "imu_in_camera",
)
DEFAULT_EXTRINSICS_MODE = "configured"
VALID_EXTRINSICS_MODES = ("configured", "identity")
VALID_BIAS_MODES = (
    "baseline",
    "zero_accel_bias",
    "zero_gyro_bias",
    "zero_both_biases",
)
DEFAULT_IMAGE_PROCESSING_WIDTH = 752
DEFAULT_CAMERA_FX = 646.33728
DEFAULT_CAMERA_FY = 645.676147
DEFAULT_CAMERA_CX = 643.358276
DEFAULT_CAMERA_CY = 362.999176
DEFAULT_CAMERA_DISTORTION = (
    -0.05594548583030701,
    0.06458555161952972,
    -0.0002526374883018434,
    0.0008183500613085926,
    -0.021141313016414642,
)
DEFAULT_CAMERA_R_IC = (
    0.002033476571297,
    0.000153435674186,
    0.9999979207131,
    -0.999996654005,
    -0.001598738420073,
    0.002033719299488,
    0.001599047140929,
    -0.9999987102456,
    0.0001501841636702,
)
DEFAULT_CAMERA_T_IC = (
    0.015779949041,
    -0.028793809935,
    -0.007352355558,
)


@dataclass(frozen=True)
class ReplayEvent:
    header_time: float
    topic: str
    rowid: int
    priority: int
    message: object


@dataclass
class FrameMetric:
    processed_frame: int
    raw_image_index: int
    header_time: float
    vel_norm: float
    imu_count: int
    imu_large_dt_skips: int
    discontinuity_count: int
    gyro_mean_norm: float
    accel_mean_norm: float
    world_acc_mean_norm: float
    active_tracks: int = 0
    mature_features: int = 0
    msckf_accepted: int = 0
    msckf_rows: int = 0
    msckf_gated_out: int = 0
    msckf_triangulation_failed: int = 0
    msckf_invalid_jacobian: int = 0
    tri_fail_clone_missing: int = 0
    tri_fail_singular: int = 0
    tri_fail_negative_depth: int = 0
    tri_fail_reprojection: int = 0
    tri_fail_mean_track_length: float = 0.0
    tri_fail_mean_baseline: float = 0.0
    tri_fail_mean_parallax_deg: float = 0.0
    tri_fail_mean_reprojection_px: float = 0.0
    msckf_batch_rejected: int = 0
    msckf_dx_vel_norm: float = 0.0
    msckf_dx_bg_norm: float = 0.0
    msckf_dx_ba_norm: float = 0.0
    msckf_innovation_condition_number: float = 0.0


@dataclass
class TriangulationDiagnostic:
    success: bool
    reason: str
    track_length: int
    max_baseline: float
    max_parallax_deg: float
    mean_reprojection_px: float


@dataclass
class SweepResult:
    extrinsics_mode: str
    frame_180_vel_norm: float
    frame_200_vel_norm: float
    late_peak_vel_norm: float
    negative_depth_fails: int
    reprojection_fails: int
    accepted_updates: int


@dataclass
class ReplaySnapshot:
    state_server: StateServer
    imu_buffer: list[ImuData]
    initial_imu_buffer: list[ImuData]
    gravity_aligned: bool
    last_image_time: float
    raw_images_seen: int
    images_processed: int
    extra: dict[str, object] = field(default_factory=dict)


class PropagationReplay:
    def __init__(
        self,
        *,
        imu_init_samples: int,
        max_imu_init_gap: float,
        max_frame_gap: float,
        max_imu_dt: float,
    ):
        self.imu_init_samples = int(imu_init_samples)
        self.max_imu_init_gap = float(max_imu_init_gap)
        self.max_frame_gap = float(max_frame_gap)
        self.state_server = StateServer(R_IC=np.eye(3), t_IC=np.zeros(3))
        self.propagator = ImuPropagator(self.state_server)
        self.propagator.max_imu_dt = float(max_imu_dt)
        self.imu_buffer: list[ImuData] = []
        self.initial_imu_buffer: list[ImuData] = []
        self.gravity_aligned = False
        self.last_image_time = -1.0
        self.raw_images_seen = 0
        self.images_processed = 0
        self.discontinuity_count = 0
        self.bias_mode_applied = "baseline"

    def snapshot(self) -> ReplaySnapshot:
        copied_server = StateServer(
            R_IC=self.state_server.R_IC.copy(),
            t_IC=self.state_server.t_IC.copy(),
        )
        copied_server.state = copy.deepcopy(self.state_server.state)
        copied_server.covariance = self.state_server.covariance.copy()
        copied_server.max_window_size = self.state_server.max_window_size
        return ReplaySnapshot(
            state_server=copied_server,
            imu_buffer=copy.deepcopy(self.imu_buffer),
            initial_imu_buffer=copy.deepcopy(self.initial_imu_buffer),
            gravity_aligned=self.gravity_aligned,
            last_image_time=self.last_image_time,
            raw_images_seen=self.raw_images_seen,
            images_processed=self.images_processed,
            extra={},
        )

    @classmethod
    def from_snapshot(
        cls,
        snapshot: ReplaySnapshot,
        *,
        imu_init_samples: int,
        max_imu_init_gap: float,
        max_frame_gap: float,
        max_imu_dt: float,
    ) -> "PropagationReplay":
        replay = cls(
            imu_init_samples=imu_init_samples,
            max_imu_init_gap=max_imu_init_gap,
            max_frame_gap=max_frame_gap,
            max_imu_dt=max_imu_dt,
        )
        replay.state_server = snapshot.state_server
        replay.propagator = ImuPropagator(replay.state_server)
        replay.propagator.max_imu_dt = float(max_imu_dt)
        replay.imu_buffer = copy.deepcopy(snapshot.imu_buffer)
        replay.initial_imu_buffer = copy.deepcopy(snapshot.initial_imu_buffer)
        replay.gravity_aligned = snapshot.gravity_aligned
        replay.last_image_time = snapshot.last_image_time
        replay.raw_images_seen = snapshot.raw_images_seen
        replay.images_processed = snapshot.images_processed
        return replay

    def process_imu(self, event: ReplayEvent) -> None:
        stamp = _stamp_to_seconds(event.message.header.stamp)
        imu_data = ImuData(
            timestamp=stamp,
            accel=np.array(
                [
                    event.message.linear_acceleration.x,
                    event.message.linear_acceleration.y,
                    event.message.linear_acceleration.z,
                ],
                dtype=np.float64,
            ),
            gyro=np.array(
                [
                    event.message.angular_velocity.x,
                    event.message.angular_velocity.y,
                    event.message.angular_velocity.z,
                ],
                dtype=np.float64,
            ),
        )
        self.imu_buffer.append(imu_data)

        if self.gravity_aligned:
            return

        updated_init_buffer, _, _ = update_initial_imu_buffer(
            self.initial_imu_buffer,
            imu_data,
            self.max_imu_init_gap,
        )
        self.initial_imu_buffer = updated_init_buffer
        if len(self.initial_imu_buffer) < self.imu_init_samples:
            return

        q_init, w_avg, ba_init, init_end_time = compute_static_imu_initialization(
            self.initial_imu_buffer
        )
        with self.state_server.lock:
            self.state_server.state.quaternion = q_init
            self.state_server.state.gyro_bias = w_avg
            self.state_server.state.accel_bias = ba_init
            self.state_server.state.timestamp = init_end_time

        self.imu_buffer = [m for m in self.imu_buffer if m.timestamp > init_end_time]
        self.gravity_aligned = True

    def process_image(self, event: ReplayEvent) -> FrameMetric | None:
        self.raw_images_seen += 1
        image_time = _stamp_to_seconds(event.message.header.stamp)
        if not self.gravity_aligned:
            return None

        if is_frame_timestamp_discontinuity(
            self.last_image_time,
            image_time,
            self.max_frame_gap,
        ):
            self.discontinuity_count += 1
            self.imu_buffer = [m for m in self.imu_buffer if m.timestamp > image_time]
            with self.state_server.lock:
                self.state_server.clear_clones()
                self.state_server.state.timestamp = image_time
            self.last_image_time = image_time
            return None

        imu_measurements = [
            m for m in self.imu_buffer if self.last_image_time < m.timestamp <= image_time
        ]
        self.imu_buffer = [m for m in self.imu_buffer if m.timestamp > image_time]
        gyro_mean_norm = 0.0
        accel_mean_norm = 0.0
        world_acc_mean_norm = 0.0
        if imu_measurements:
            accel_mean = np.mean([m.accel for m in imu_measurements], axis=0)
            gyro_mean = np.mean([m.gyro for m in imu_measurements], axis=0)
            self.propagator.propagate(imu_measurements)
            with self.state_server.lock:
                state = self.state_server.state
                world_acc_mean = (
                    quaternion_to_matrix(state.quaternion)
                    @ (accel_mean - state.accel_bias)
                    + self.propagator.gravity
                )
            gyro_mean_norm = float(np.linalg.norm(gyro_mean))
            accel_mean_norm = float(np.linalg.norm(accel_mean))
            world_acc_mean_norm = float(np.linalg.norm(world_acc_mean))

        self.images_processed += 1
        self.last_image_time = image_time
        with self.state_server.lock:
            vel_norm = float(np.linalg.norm(self.state_server.state.velocity))

        return FrameMetric(
            processed_frame=self.images_processed,
            raw_image_index=self.raw_images_seen,
            header_time=image_time,
            vel_norm=vel_norm,
            imu_count=len(imu_measurements),
            imu_large_dt_skips=self.propagator.last_large_dt_count,
            discontinuity_count=self.discontinuity_count,
            gyro_mean_norm=gyro_mean_norm,
            accel_mean_norm=accel_mean_norm,
            world_acc_mean_norm=world_acc_mean_norm,
        )


class MsckfReplay(PropagationReplay):
    def __init__(
        self,
        *,
        imu_init_samples: int,
        max_imu_init_gap: float,
        max_frame_gap: float,
        max_imu_dt: float,
        image_processing_width: int,
        camera_fx: float,
        camera_fy: float,
        camera_cx: float,
        camera_cy: float,
        camera_distortion: list[float],
        camera_extrinsics_convention: str,
        camera_R_IC: np.ndarray,
        camera_t_IC: np.ndarray,
        max_batch_dx_pos_norm: float,
        max_batch_dx_vel_norm: float,
        max_batch_dx_bias_norm: float,
    ):
        super().__init__(
            imu_init_samples=imu_init_samples,
            max_imu_init_gap=max_imu_init_gap,
            max_frame_gap=max_frame_gap,
            max_imu_dt=max_imu_dt,
        )
        self.state_server.set_camera_extrinsics(
            camera_R_IC,
            camera_t_IC,
            convention=camera_extrinsics_convention,
        )
        self.frontend_config = FrontendConfig()
        self.detector = ShiTomasiDetector(self.frontend_config)
        self.tracker = KLTTracker(self.frontend_config)
        self.frontend = FeatureManager(
            self.detector,
            self.tracker,
            self.frontend_config,
        )
        self.msckf_updater = MSCKFUpdater(self.state_server)
        self.msckf_updater.max_batch_dx_pos_norm = float(max_batch_dx_pos_norm)
        self.msckf_updater.max_batch_dx_vel_norm = float(max_batch_dx_vel_norm)
        self.msckf_updater.max_batch_dx_bias_norm = float(max_batch_dx_bias_norm)
        self.image_processing_width = int(image_processing_width)
        self.base_camera_fx = float(camera_fx)
        self.base_camera_fy = float(camera_fy)
        self.base_camera_cx = float(camera_cx)
        self.base_camera_cy = float(camera_cy)
        self.base_camera_distortion = np.asarray(camera_distortion, dtype=np.float64)
        self._calibration_scale_applied = None
        self._apply_effective_camera_calibration(scale=1.0)

    def snapshot(self) -> ReplaySnapshot:
        snapshot = super().snapshot()
        snapshot.extra = {
            "frontend": copy.deepcopy(self.frontend),
            "calibration_scale_applied": self._calibration_scale_applied,
        }
        return snapshot

    @classmethod
    def from_snapshot(
        cls,
        snapshot: ReplaySnapshot,
        *,
        imu_init_samples: int,
        max_imu_init_gap: float,
        max_frame_gap: float,
        max_imu_dt: float,
        image_processing_width: int,
        camera_fx: float,
        camera_fy: float,
        camera_cx: float,
        camera_cy: float,
        camera_distortion: list[float],
        camera_extrinsics_convention: str,
        camera_R_IC: np.ndarray,
        camera_t_IC: np.ndarray,
        max_batch_dx_pos_norm: float,
        max_batch_dx_vel_norm: float,
        max_batch_dx_bias_norm: float,
    ) -> "MsckfReplay":
        replay = cls(
            imu_init_samples=imu_init_samples,
            max_imu_init_gap=max_imu_init_gap,
            max_frame_gap=max_frame_gap,
            max_imu_dt=max_imu_dt,
            image_processing_width=image_processing_width,
            camera_fx=camera_fx,
            camera_fy=camera_fy,
            camera_cx=camera_cx,
            camera_cy=camera_cy,
            camera_distortion=camera_distortion,
            camera_extrinsics_convention=camera_extrinsics_convention,
            camera_R_IC=camera_R_IC,
            camera_t_IC=camera_t_IC,
            max_batch_dx_pos_norm=max_batch_dx_pos_norm,
            max_batch_dx_vel_norm=max_batch_dx_vel_norm,
            max_batch_dx_bias_norm=max_batch_dx_bias_norm,
        )
        replay.state_server = snapshot.state_server
        replay.propagator = ImuPropagator(replay.state_server)
        replay.propagator.max_imu_dt = float(max_imu_dt)
        replay.msckf_updater = MSCKFUpdater(replay.state_server)
        replay.msckf_updater.max_batch_dx_pos_norm = float(max_batch_dx_pos_norm)
        replay.msckf_updater.max_batch_dx_vel_norm = float(max_batch_dx_vel_norm)
        replay.msckf_updater.max_batch_dx_bias_norm = float(max_batch_dx_bias_norm)
        replay.imu_buffer = copy.deepcopy(snapshot.imu_buffer)
        replay.initial_imu_buffer = copy.deepcopy(snapshot.initial_imu_buffer)
        replay.gravity_aligned = snapshot.gravity_aligned
        replay.last_image_time = snapshot.last_image_time
        replay.raw_images_seen = snapshot.raw_images_seen
        replay.images_processed = snapshot.images_processed
        replay.frontend = copy.deepcopy(snapshot.extra["frontend"])
        replay._calibration_scale_applied = None
        scale = snapshot.extra.get("calibration_scale_applied")
        replay._apply_effective_camera_calibration(
            scale=1.0 if scale is None else float(scale)
        )
        return replay

    def process_image(self, event: ReplayEvent) -> FrameMetric | None:
        self.raw_images_seen += 1
        image_time = _stamp_to_seconds(event.message.header.stamp)
        if not self.gravity_aligned:
            return None

        if is_frame_timestamp_discontinuity(
            self.last_image_time,
            image_time,
            self.max_frame_gap,
        ):
            self.discontinuity_count += 1
            self.imu_buffer = [m for m in self.imu_buffer if m.timestamp > image_time]
            self.frontend.reset()
            with self.state_server.lock:
                self.state_server.clear_clones()
                self.state_server.state.timestamp = image_time
            self.last_image_time = image_time
            return None

        full_res_image = _decode_grayscale_image(event.message)
        image, image_scale = prepare_image_for_processing(
            full_res_image,
            target_width=self.image_processing_width,
        )
        self._apply_effective_camera_calibration(scale=image_scale)

        imu_measurements = [
            m for m in self.imu_buffer if self.last_image_time < m.timestamp <= image_time
        ]
        self.imu_buffer = [m for m in self.imu_buffer if m.timestamp > image_time]
        gyro_mean_norm = 0.0
        accel_mean_norm = 0.0
        world_acc_mean_norm = 0.0
        if imu_measurements:
            accel_mean = np.mean([m.accel for m in imu_measurements], axis=0)
            gyro_mean = np.mean([m.gyro for m in imu_measurements], axis=0)
            self.propagator.propagate(imu_measurements)
            with self.state_server.lock:
                state = self.state_server.state
                world_acc_mean = (
                    quaternion_to_matrix(state.quaternion)
                    @ (accel_mean - state.accel_bias)
                    + self.propagator.gravity
                )
            gyro_mean_norm = float(np.linalg.norm(gyro_mean))
            accel_mean_norm = float(np.linalg.norm(accel_mean))
            world_acc_mean_norm = float(np.linalg.norm(world_acc_mean))

        with self.state_server.lock:
            state = self.state_server.state
            self.state_server.add_clone(
                image_time,
                state.position,
                state.quaternion,
            )
            clone = self.state_server.state.clone_poses[-1]
            current_camera_pose = CameraPose(
                timestamp=clone.timestamp,
                position=clone.position.copy(),
                quaternion=clone.quaternion.copy(),
            )

        mature_features = self.frontend.process_image(
            image_time,
            image,
            current_camera_pose,
        )
        with self.state_server.lock:
            triangulation_diagnostics = _analyze_mature_features(
                self.msckf_updater,
                mature_features,
            )
            self.msckf_updater.process_mature_features(mature_features)
            update_stats = dict(self.msckf_updater.last_update_stats)
            vel_norm = float(np.linalg.norm(self.state_server.state.velocity))
        triangulation_summary = _summarize_triangulation_diagnostics(
            triangulation_diagnostics
        )

        self.images_processed += 1
        self.last_image_time = image_time
        return FrameMetric(
            processed_frame=self.images_processed,
            raw_image_index=self.raw_images_seen,
            header_time=image_time,
            vel_norm=vel_norm,
            imu_count=len(imu_measurements),
            imu_large_dt_skips=self.propagator.last_large_dt_count,
            discontinuity_count=self.discontinuity_count,
            gyro_mean_norm=gyro_mean_norm,
            accel_mean_norm=accel_mean_norm,
            world_acc_mean_norm=world_acc_mean_norm,
            active_tracks=len(self.frontend.active_tracks),
            mature_features=len(mature_features),
            msckf_accepted=int(update_stats.get("accepted", 0)),
            msckf_rows=int(update_stats.get("rows", 0)),
            msckf_gated_out=int(update_stats.get("gated_out", 0)),
            msckf_triangulation_failed=int(
                update_stats.get("triangulation_failed", 0)
            ),
            msckf_invalid_jacobian=int(update_stats.get("invalid_jacobian", 0)),
            tri_fail_clone_missing=int(triangulation_summary["clone_missing"]),
            tri_fail_singular=int(triangulation_summary["singular"]),
            tri_fail_negative_depth=int(triangulation_summary["negative_depth"]),
            tri_fail_reprojection=int(triangulation_summary["reprojection"]),
            tri_fail_mean_track_length=float(
                triangulation_summary["mean_track_length"]
            ),
            tri_fail_mean_baseline=float(triangulation_summary["mean_baseline"]),
            tri_fail_mean_parallax_deg=float(
                triangulation_summary["mean_parallax_deg"]
            ),
            tri_fail_mean_reprojection_px=float(
                triangulation_summary["mean_reprojection_px"]
            ),
            msckf_batch_rejected=int(update_stats.get("batch_rejected", 0)),
            msckf_dx_vel_norm=float(update_stats.get("dx_vel_norm", 0.0)),
            msckf_dx_bg_norm=float(update_stats.get("dx_bg_norm", 0.0)),
            msckf_dx_ba_norm=float(update_stats.get("dx_accel_bias_norm", 0.0)),
            msckf_innovation_condition_number=float(
                update_stats.get("innovation_condition_number", 0.0)
            ),
        )

    def _apply_effective_camera_calibration(self, *, scale: float) -> None:
        if (
            self._calibration_scale_applied is not None
            and np.isclose(self._calibration_scale_applied, scale)
        ):
            return
        fx, fy, cx, cy, distortion = compute_effective_camera_calibration(
            fx=self.base_camera_fx,
            fy=self.base_camera_fy,
            cx=self.base_camera_cx,
            cy=self.base_camera_cy,
            distortion_coefficients=self.base_camera_distortion,
            scale=scale,
        )
        self.msckf_updater.set_camera_calibration(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            distortion_coefficients=distortion,
        )
        self._calibration_scale_applied = float(scale)


def _resolve_bag_path(configured_path: str | None) -> str:
    if configured_path:
        return configured_path
    for candidate in DEFAULT_BAG_PATHS:
        if os.path.exists(candidate):
            return candidate
    raise FileNotFoundError("Could not locate vio_hcmut_dataset bag directory")


def _resolve_camera_extrinsics_mode(
    *,
    mode: str,
    camera_R_IC: np.ndarray,
    camera_t_IC: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if mode == "configured":
        return camera_R_IC.copy(), camera_t_IC.copy()
    if mode == "identity":
        return np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)
    raise ValueError(f"Unsupported extrinsics mode: {mode}")


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _read_replay_events(
    bag_path: str,
    *,
    imu_topic: str,
    image_topic: str,
) -> list[ReplayEvent]:
    if not HAS_ROS:
        raise RuntimeError("ROS 2 Python message deserialization is unavailable")

    db_path = os.path.join(bag_path, os.path.basename(bag_path) + "_0.db3")
    conn = sqlite3.connect(db_path)
    try:
        topic_rows = conn.execute(
            "SELECT id, name, type FROM topics WHERE name IN (?, ?)",
            (imu_topic, image_topic),
        ).fetchall()
        topic_by_id = {topic_id: (name, type_name) for topic_id, name, type_name in topic_rows}
        if len(topic_by_id) != 2:
            raise RuntimeError("Could not resolve IMU and image topics in the bag")

        message_types = {
            topic_id: get_message(type_name)
            for topic_id, (_, type_name) in topic_by_id.items()
        }
        priorities = {imu_topic: 0, image_topic: 1}
        events: list[ReplayEvent] = []
        rows = conn.execute(
            "SELECT rowid, topic_id, data FROM messages "
            "WHERE topic_id IN (?, ?) ORDER BY rowid",
            tuple(sorted(topic_by_id.keys())),
        )
        for rowid, topic_id, data in rows:
            topic_name, _ = topic_by_id[topic_id]
            msg = deserialize_message(data, message_types[topic_id])
            events.append(
                ReplayEvent(
                    header_time=_stamp_to_seconds(msg.header.stamp),
                    topic=topic_name,
                    rowid=int(rowid),
                    priority=priorities[topic_name],
                    message=msg,
                )
            )
    finally:
        conn.close()

    events.sort(key=lambda event: (event.header_time, event.priority, event.rowid))
    return events


def _run_replay(
    events: list[ReplayEvent],
    *,
    start_index: int,
    stop_processed_image: int,
    snapshot_frame: int | None,
    runner: PropagationReplay,
    bias_mode: str = "baseline",
) -> tuple[list[FrameMetric], ReplaySnapshot | None, int | None]:
    metrics: list[FrameMetric] = []
    snapshot = None
    snapshot_event_index = None
    for event_index in range(start_index, len(events)):
        event = events[event_index]
        if event.priority == 0:
            was_aligned = runner.gravity_aligned
            runner.process_imu(event)
            if not was_aligned and runner.gravity_aligned:
                _apply_bias_mode(runner, bias_mode)
            continue

        metric = runner.process_image(event)
        if metric is None:
            continue
        metrics.append(metric)

        if snapshot_frame is not None and metric.processed_frame == snapshot_frame:
            snapshot = runner.snapshot()
            snapshot_event_index = event_index + 1

        if metric.processed_frame >= stop_processed_image:
            break

    return metrics, snapshot, snapshot_event_index


def _apply_bias_mode(replay: PropagationReplay, bias_mode: str) -> None:
    if bias_mode not in VALID_BIAS_MODES:
        raise ValueError(f"Unsupported bias mode: {bias_mode}")

    with replay.state_server.lock:
        if bias_mode in ("zero_accel_bias", "zero_both_biases"):
            replay.state_server.state.accel_bias[:] = 0.0
        if bias_mode in ("zero_gyro_bias", "zero_both_biases"):
            replay.state_server.state.gyro_bias[:] = 0.0
    replay.bias_mode_applied = bias_mode


def _frame_map(metrics: list[FrameMetric]) -> dict[int, FrameMetric]:
    return {metric.processed_frame: metric for metric in metrics}


def _summarize_triangulation_diagnostics(
    diagnostics: list[TriangulationDiagnostic],
) -> dict[str, float]:
    failed = [diag for diag in diagnostics if not diag.success]
    summary: dict[str, float] = {
        "clone_missing": 0,
        "singular": 0,
        "negative_depth": 0,
        "reprojection": 0,
        "mean_track_length": 0.0,
        "mean_baseline": 0.0,
        "mean_parallax_deg": 0.0,
        "mean_reprojection_px": 0.0,
    }
    if not failed:
        return summary

    for diag in failed:
        if diag.reason in summary:
            summary[diag.reason] += 1
    summary["mean_track_length"] = float(
        np.mean([diag.track_length for diag in failed])
    )
    summary["mean_baseline"] = float(
        np.mean([diag.max_baseline for diag in failed])
    )
    summary["mean_parallax_deg"] = float(
        np.mean([diag.max_parallax_deg for diag in failed])
    )
    reprojection_values = [
        diag.mean_reprojection_px
        for diag in failed
        if np.isfinite(diag.mean_reprojection_px)
    ]
    if reprojection_values:
        summary["mean_reprojection_px"] = float(np.mean(reprojection_values))
    return summary


def _analyze_mature_features(
    updater: MSCKFUpdater,
    mature_features,
) -> list[TriangulationDiagnostic]:
    diagnostics: list[TriangulationDiagnostic] = []
    for feature in mature_features:
        clone_sequence = updater._get_clone_sequence(feature)
        if clone_sequence is None:
            diagnostics.append(
                TriangulationDiagnostic(
                    success=False,
                    reason="clone_missing",
                    track_length=len(feature.observations),
                    max_baseline=0.0,
                    max_parallax_deg=0.0,
                    mean_reprojection_px=float("inf"),
                )
            )
            continue

        normalized_observations = updater._normalize_observations(feature.observations)
        camera_positions = [cam_pose.position for _, cam_pose in clone_sequence]
        ray_directions = []
        for obs_norm, (_, cam_pose) in zip(normalized_observations, clone_sequence):
            bearing_cam = np.array([obs_norm[0], obs_norm[1], 1.0], dtype=np.float64)
            bearing_cam /= np.linalg.norm(bearing_cam)
            R_wc = quaternion_to_matrix(cam_pose.quaternion)
            ray_directions.append(R_wc @ bearing_cam)

        max_baseline = 0.0
        for i in range(len(camera_positions)):
            for j in range(i + 1, len(camera_positions)):
                baseline = float(
                    np.linalg.norm(camera_positions[j] - camera_positions[i])
                )
                if baseline > max_baseline:
                    max_baseline = baseline

        max_parallax_deg = 0.0
        for i in range(len(ray_directions)):
            for j in range(i + 1, len(ray_directions)):
                dot = float(np.clip(np.dot(ray_directions[i], ray_directions[j]), -1.0, 1.0))
                parallax_deg = float(np.degrees(np.arccos(dot)))
                if parallax_deg > max_parallax_deg:
                    max_parallax_deg = parallax_deg

        A = []
        for obs_norm, (_, cam_pose) in zip(normalized_observations, clone_sequence):
            R_wc = quaternion_to_matrix(cam_pose.quaternion)
            R_cw = R_wc.T
            t = -R_cw @ cam_pose.position
            P_matrix = np.hstack([R_cw, t.reshape(3, 1)])
            P1 = P_matrix[0, :]
            P2 = P_matrix[1, :]
            P3 = P_matrix[2, :]
            x, y = obs_norm
            A.append(x * P3 - P1)
            A.append(y * P3 - P2)
        A = np.asarray(A, dtype=np.float64)

        try:
            _, _, V = np.linalg.svd(A)
        except np.linalg.LinAlgError:
            diagnostics.append(
                TriangulationDiagnostic(
                    success=False,
                    reason="singular",
                    track_length=len(feature.observations),
                    max_baseline=max_baseline,
                    max_parallax_deg=max_parallax_deg,
                    mean_reprojection_px=float("inf"),
                )
            )
            continue

        if abs(V[-1, 3]) < 1e-6:
            diagnostics.append(
                TriangulationDiagnostic(
                    success=False,
                    reason="singular",
                    track_length=len(feature.observations),
                    max_baseline=max_baseline,
                    max_parallax_deg=max_parallax_deg,
                    mean_reprojection_px=float("inf"),
                )
            )
            continue

        p_w = V[-1, :3] / V[-1, 3]
        reprojection_errors = []
        negative_depth = False
        for obs_norm, (_, cam_pose) in zip(normalized_observations, clone_sequence):
            R_cw = quaternion_to_matrix(cam_pose.quaternion).T
            p_c = R_cw @ (p_w - cam_pose.position)
            if p_c[2] < 1e-3:
                negative_depth = True
                break
            pred_norm = np.array([p_c[0] / p_c[2], p_c[1] / p_c[2]], dtype=np.float64)
            reprojection_errors.append(
                float(np.linalg.norm(pred_norm - obs_norm) * 0.5 * (updater.fx + updater.fy))
            )

        if negative_depth:
            diagnostics.append(
                TriangulationDiagnostic(
                    success=False,
                    reason="negative_depth",
                    track_length=len(feature.observations),
                    max_baseline=max_baseline,
                    max_parallax_deg=max_parallax_deg,
                    mean_reprojection_px=float("inf"),
                )
            )
            continue

        mean_reprojection_px = float(np.mean(reprojection_errors))
        if mean_reprojection_px > 5.0:
            diagnostics.append(
                TriangulationDiagnostic(
                    success=False,
                    reason="reprojection",
                    track_length=len(feature.observations),
                    max_baseline=max_baseline,
                    max_parallax_deg=max_parallax_deg,
                    mean_reprojection_px=mean_reprojection_px,
                )
            )
            continue

        diagnostics.append(
            TriangulationDiagnostic(
                success=True,
                reason="success",
                track_length=len(feature.observations),
                max_baseline=max_baseline,
                max_parallax_deg=max_parallax_deg,
                mean_reprojection_px=mean_reprojection_px,
            )
        )
    return diagnostics


def _decode_grayscale_image(msg) -> np.ndarray:
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    row_bytes = raw.reshape(msg.height, msg.step)

    if enc == "mono8":
        return row_bytes[:, : msg.width].copy()
    if enc == "mono16":
        img16 = row_bytes[:, : msg.width * 2].copy().view(np.uint16)
        return (img16.reshape(msg.height, msg.width) >> 8).astype(np.uint8)
    if enc in ("rgb8", "bgr8"):
        color = row_bytes[:, : msg.width * 3].copy().reshape(
            msg.height, msg.width, 3
        )
        code = cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(color, code)

    raise ValueError(f"Unsupported image encoding: {msg.encoding!r}")


def _print_summary(name: str, metrics: list[FrameMetric]) -> None:
    frame_by_index = _frame_map(metrics)
    max_vel = max((metric.vel_norm for metric in metrics), default=0.0)
    discontinuities = max((metric.discontinuity_count for metric in metrics), default=0)
    large_dt_skips = sum(metric.imu_large_dt_skips for metric in metrics)

    print(f"\n=== {name} ===")
    print(f"processed_frames={len(metrics)}")
    print(f"discontinuities={discontinuities}")
    print(f"imu_large_dt_skips={large_dt_skips}")
    print(f"max_vel_norm={max_vel:.6f}")
    for frame in (160, 170, 180, 190, 200):
        metric = frame_by_index.get(frame)
        if metric is None:
            print(f"frame_{frame}=missing")
        else:
            print(
                f"frame_{frame}=vel_norm:{metric.vel_norm:.6f}, "
                f"raw_image:{metric.raw_image_index}, ts:{metric.header_time:.9f}"
            )


def _print_window_report(
    metrics: list[FrameMetric],
    *,
    start_frame: int,
    stop_frame: int,
    every_n: int,
) -> None:
    window = [
        metric
        for metric in metrics
        if start_frame <= metric.processed_frame <= stop_frame
    ]
    if not window:
        print(
            f"\nwindow_report=missing start_frame={start_frame} stop_frame={stop_frame}"
        )
        return

    print(
        f"\n=== WINDOW REPORT frames {start_frame}-{stop_frame} step {every_n} ==="
    )
    for metric in window:
        if (metric.processed_frame - start_frame) % every_n != 0:
            continue
        print(
            "frame="
            f"{metric.processed_frame} "
            f"vel_norm={metric.vel_norm:.6f} "
            f"gyro_mean_norm={metric.gyro_mean_norm:.6f} "
            f"accel_mean_norm={metric.accel_mean_norm:.6f} "
            f"world_acc_mean_norm={metric.world_acc_mean_norm:.6f} "
            f"active_tracks={metric.active_tracks} "
            f"mature={metric.mature_features} "
            f"accepted={metric.msckf_accepted} "
            f"rows={metric.msckf_rows} "
            f"gated_out={metric.msckf_gated_out} "
            f"triang_fail={metric.msckf_triangulation_failed} "
            f"invalid_jacobian={metric.msckf_invalid_jacobian} "
            f"tri_clone_missing={metric.tri_fail_clone_missing} "
            f"tri_singular={metric.tri_fail_singular} "
            f"tri_negative_depth={metric.tri_fail_negative_depth} "
            f"tri_reprojection={metric.tri_fail_reprojection} "
            f"tri_fail_track_len={metric.tri_fail_mean_track_length:.2f} "
            f"tri_fail_baseline={metric.tri_fail_mean_baseline:.4f} "
            f"tri_fail_parallax_deg={metric.tri_fail_mean_parallax_deg:.3f} "
            f"tri_fail_reproj_px={metric.tri_fail_mean_reprojection_px:.3f} "
            f"batch_rejected={metric.msckf_batch_rejected} "
            f"dx_vel={metric.msckf_dx_vel_norm:.3e} "
            f"dx_bg={metric.msckf_dx_bg_norm:.3e} "
            f"dx_ba={metric.msckf_dx_ba_norm:.3e} "
            f"innovation_cond={metric.msckf_innovation_condition_number:.3e}"
        )


def _print_optical_flow_report(
    events: list[ReplayEvent],
    metrics: list[FrameMetric],
    *,
    start_frame: int,
    stop_frame: int,
    every_n: int,
) -> None:
    window = [
        metric
        for metric in metrics
        if start_frame <= metric.processed_frame <= stop_frame
        and (metric.processed_frame - start_frame) % every_n == 0
    ]
    if not window:
        print(
            f"\noptical_flow_report=missing start_frame={start_frame} stop_frame={stop_frame}"
        )
        return

    image_events = [event for event in events if event.priority == 1]
    print(
        f"\n=== OPTICAL FLOW REPORT frames {start_frame}-{stop_frame} step {every_n} ==="
    )
    for metric in window:
        raw_index = metric.raw_image_index
        if raw_index >= len(image_events):
            print(f"frame={metric.processed_frame} optical_flow=missing")
            continue

        prev_image = _decode_grayscale_image(image_events[raw_index - 1].message)
        next_image = _decode_grayscale_image(image_events[raw_index].message)
        points = cv2.goodFeaturesToTrack(
            prev_image,
            maxCorners=200,
            qualityLevel=0.01,
            minDistance=10,
        )
        if points is None or len(points) == 0:
            print(f"frame={metric.processed_frame} optical_flow=no_features")
            continue

        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_image,
            next_image,
            points,
            None,
        )
        if next_points is None or status is None:
            print(f"frame={metric.processed_frame} optical_flow=tracking_failed")
            continue

        good_prev = points[status.flatten() == 1].reshape(-1, 2)
        good_next = next_points[status.flatten() == 1].reshape(-1, 2)
        if len(good_prev) == 0:
            print(f"frame={metric.processed_frame} optical_flow=no_tracks")
            continue

        flow = np.linalg.norm(good_next - good_prev, axis=1)
        print(
            "frame="
            f"{metric.processed_frame} "
            f"raw_pair={raw_index}->{raw_index + 1} "
            f"tracked={len(flow)} "
            f"mean_flow={float(flow.mean()):.3f} "
            f"p95_flow={float(np.percentile(flow, 95)):.3f} "
            f"max_flow={float(flow.max()):.3f}"
        )


def _summarize_extrinsics_sweep(
    mode: str,
    metrics: list[FrameMetric],
) -> SweepResult:
    frame_by_index = _frame_map(metrics)
    frame_180 = frame_by_index.get(180)
    frame_200 = frame_by_index.get(200)
    late_metrics = [
        metric
        for metric in metrics
        if 160 <= metric.processed_frame <= 220
    ]
    return SweepResult(
        extrinsics_mode=mode,
        frame_180_vel_norm=0.0 if frame_180 is None else frame_180.vel_norm,
        frame_200_vel_norm=0.0 if frame_200 is None else frame_200.vel_norm,
        late_peak_vel_norm=max(
            (metric.vel_norm for metric in late_metrics),
            default=0.0,
        ),
        negative_depth_fails=sum(
            metric.tri_fail_negative_depth for metric in late_metrics
        ),
        reprojection_fails=sum(
            metric.tri_fail_reprojection for metric in late_metrics
        ),
        accepted_updates=sum(
            metric.msckf_accepted for metric in late_metrics
        ),
    )


def _print_extrinsics_sweep_report(results: list[SweepResult]) -> None:
    print("\n=== EXTRINSICS SWEEP ===")
    for result in results:
        print(
            "mode="
            f"{result.extrinsics_mode} "
            f"frame180_vel={result.frame_180_vel_norm:.6f} "
            f"frame200_vel={result.frame_200_vel_norm:.6f} "
            f"late_peak_vel={result.late_peak_vel_norm:.6f} "
            f"negative_depth_fails={result.negative_depth_fails} "
            f"reprojection_fails={result.reprojection_fails} "
            f"accepted_updates={result.accepted_updates}"
        )


def _classify(metrics: list[FrameMetric]) -> str:
    late_window = [
        metric.vel_norm
        for metric in metrics
        if 160 <= metric.processed_frame <= 200
    ]
    if not late_window:
        raise RuntimeError("Late drift window is missing from replay output")
    late_peak = max(late_window)
    return (
        "intrinsic_propagation"
        if late_peak >= DRIFT_THRESHOLD
        else "playback_or_ros_ordering"
    )


def _verify_segment_matches(
    full_metrics: list[FrameMetric],
    segment_metrics: list[FrameMetric],
    *,
    start_frame: int,
    stop_frame: int,
) -> None:
    full_by_frame = _frame_map(full_metrics)
    segment_by_frame = _frame_map(segment_metrics)
    for frame in range(start_frame + 1, stop_frame + 1):
        full_metric = full_by_frame.get(frame)
        segment_metric = segment_by_frame.get(frame)
        if full_metric is None or segment_metric is None:
            raise RuntimeError(f"Missing frame {frame} during segment verification")
        if not np.isclose(full_metric.vel_norm, segment_metric.vel_norm, atol=1e-12):
            raise RuntimeError(
                f"Segment replay diverged at frame {frame}: "
                f"{segment_metric.vel_norm:.12f} vs {full_metric.vel_norm:.12f}"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--replay-mode",
        choices=VALID_REPLAY_MODES,
        default=DEFAULT_REPLAY_MODE,
    )
    parser.add_argument("--bag-path", default=None)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--imu-init-samples", type=int, default=DEFAULT_IMU_INIT_SAMPLES)
    parser.add_argument("--full-stop-image", type=int, default=DEFAULT_FULL_STOP_IMAGE)
    parser.add_argument("--snapshot-image", type=int, default=DEFAULT_SNAPSHOT_IMAGE)
    parser.add_argument("--segment-stop-image", type=int, default=DEFAULT_SEGMENT_STOP_IMAGE)
    parser.add_argument("--bias-mode", choices=VALID_BIAS_MODES, default="baseline")
    parser.add_argument(
        "--extrinsics-mode",
        choices=VALID_EXTRINSICS_MODES,
        default=DEFAULT_EXTRINSICS_MODE,
    )
    parser.add_argument(
        "--camera-extrinsics-convention",
        choices=VALID_CAMERA_EXTRINSICS_CONVENTIONS,
        default=DEFAULT_CAMERA_EXTRINSICS_CONVENTION,
    )
    parser.add_argument("--report-extrinsics-sweep", action="store_true")
    parser.add_argument(
        "--image-processing-width",
        type=int,
        default=DEFAULT_IMAGE_PROCESSING_WIDTH,
    )
    parser.add_argument("--camera-fx", type=float, default=DEFAULT_CAMERA_FX)
    parser.add_argument("--camera-fy", type=float, default=DEFAULT_CAMERA_FY)
    parser.add_argument("--camera-cx", type=float, default=DEFAULT_CAMERA_CX)
    parser.add_argument("--camera-cy", type=float, default=DEFAULT_CAMERA_CY)
    parser.add_argument(
        "--camera-distortion",
        type=float,
        nargs="+",
        default=list(DEFAULT_CAMERA_DISTORTION),
    )
    parser.add_argument(
        "--camera-R-IC",
        type=float,
        nargs=9,
        default=list(DEFAULT_CAMERA_R_IC),
    )
    parser.add_argument(
        "--camera-t-IC",
        type=float,
        nargs=3,
        default=list(DEFAULT_CAMERA_T_IC),
    )
    parser.add_argument("--max-batch-dx-pos-norm", type=float, default=0.5)
    parser.add_argument("--max-batch-dx-vel-norm", type=float, default=1.0)
    parser.add_argument("--max-batch-dx-bias-norm", type=float, default=0.03)
    parser.add_argument("--report-window-start", type=int, default=150)
    parser.add_argument("--report-window-stop", type=int, default=220)
    parser.add_argument("--report-every", type=int, default=10)
    parser.add_argument("--report-optical-flow", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not HAS_ROS:
        print("ROS 2 Python dependencies are unavailable", file=sys.stderr)
        return 1

    bag_path = _resolve_bag_path(args.bag_path)
    max_imu_init_gap = sanitize_time_gap_threshold(DEFAULT_MAX_IMU_INIT_GAP, 0.1)
    max_frame_gap = sanitize_time_gap_threshold(DEFAULT_MAX_FRAME_GAP, 0.1)

    print(f"bag_path={bag_path}")
    print(f"imu_topic={args.imu_topic}")
    print(f"image_topic={args.image_topic}")
    print(f"replay_mode={args.replay_mode}")
    print(f"bias_mode={args.bias_mode}")
    print(f"extrinsics_mode={args.extrinsics_mode}")
    print(
        "camera_extrinsics_convention="
        f"{args.camera_extrinsics_convention}"
    )

    events = _read_replay_events(
        bag_path,
        imu_topic=args.imu_topic,
        image_topic=args.image_topic,
    )
    print(f"loaded_events={len(events)}")

    base_camera_R_IC = np.asarray(args.camera_R_IC, dtype=np.float64).reshape(3, 3)
    base_camera_t_IC = np.asarray(args.camera_t_IC, dtype=np.float64)
    camera_R_IC, camera_t_IC = _resolve_camera_extrinsics_mode(
        mode=args.extrinsics_mode,
        camera_R_IC=base_camera_R_IC,
        camera_t_IC=base_camera_t_IC,
    )
    runner_kwargs = dict(
        imu_init_samples=args.imu_init_samples,
        max_imu_init_gap=max_imu_init_gap,
        max_frame_gap=max_frame_gap,
        max_imu_dt=DEFAULT_MAX_IMU_DT,
    )
    if args.replay_mode == "propagation":
        full_runner = PropagationReplay(**runner_kwargs)
    else:
        full_runner = MsckfReplay(
            **runner_kwargs,
            image_processing_width=args.image_processing_width,
            camera_fx=args.camera_fx,
            camera_fy=args.camera_fy,
            camera_cx=args.camera_cx,
            camera_cy=args.camera_cy,
            camera_distortion=args.camera_distortion,
            camera_extrinsics_convention=args.camera_extrinsics_convention,
            camera_R_IC=camera_R_IC,
            camera_t_IC=camera_t_IC,
            max_batch_dx_pos_norm=args.max_batch_dx_pos_norm,
            max_batch_dx_vel_norm=args.max_batch_dx_vel_norm,
            max_batch_dx_bias_norm=args.max_batch_dx_bias_norm,
        )
    full_metrics, snapshot, snapshot_event_index = _run_replay(
        events,
        start_index=0,
        stop_processed_image=args.full_stop_image,
        snapshot_frame=args.snapshot_image,
        runner=full_runner,
        bias_mode=args.bias_mode,
    )
    if snapshot is None or snapshot_event_index is None:
        raise RuntimeError("Snapshot frame was not reached during full replay")
    if args.replay_mode == "propagation":
        segment_runner = PropagationReplay.from_snapshot(
            snapshot,
            **runner_kwargs,
        )
    else:
        segment_runner = MsckfReplay.from_snapshot(
            snapshot,
            **runner_kwargs,
            image_processing_width=args.image_processing_width,
            camera_fx=args.camera_fx,
            camera_fy=args.camera_fy,
            camera_cx=args.camera_cx,
            camera_cy=args.camera_cy,
            camera_distortion=args.camera_distortion,
            camera_extrinsics_convention=args.camera_extrinsics_convention,
            camera_R_IC=camera_R_IC,
            camera_t_IC=camera_t_IC,
            max_batch_dx_pos_norm=args.max_batch_dx_pos_norm,
            max_batch_dx_vel_norm=args.max_batch_dx_vel_norm,
            max_batch_dx_bias_norm=args.max_batch_dx_bias_norm,
        )
    if args.bias_mode != "baseline":
        _apply_bias_mode(segment_runner, args.bias_mode)
    segment_metrics, _, _ = _run_replay(
        events,
        start_index=snapshot_event_index,
        stop_processed_image=args.segment_stop_image,
        snapshot_frame=None,
        runner=segment_runner,
        bias_mode=args.bias_mode,
    )

    _print_summary("FULL DETERMINISTIC PASS", full_metrics)
    _print_summary("SNAPSHOT SEGMENT REPLAY", segment_metrics)
    _print_window_report(
        full_metrics,
        start_frame=args.report_window_start,
        stop_frame=args.report_window_stop,
        every_n=args.report_every,
    )
    if args.report_optical_flow:
        _print_optical_flow_report(
            events,
            full_metrics,
            start_frame=args.report_window_start,
            stop_frame=args.report_window_stop,
            every_n=args.report_every,
        )

    full_discontinuities = max(
        (metric.discontinuity_count for metric in full_metrics),
        default=0,
    )
    segment_discontinuities = max(
        (metric.discontinuity_count for metric in segment_metrics),
        default=0,
    )
    if full_discontinuities != 0 or segment_discontinuities != 0:
        raise RuntimeError(
            f"Unexpected discontinuities in deterministic replay: "
            f"full={full_discontinuities}, segment={segment_discontinuities}"
        )

    _verify_segment_matches(
        full_metrics,
        segment_metrics,
        start_frame=args.snapshot_image,
        stop_frame=args.segment_stop_image,
    )

    if args.replay_mode == "propagation":
        classification = _classify(full_metrics)
        print(f"\nclassification={classification}")
    else:
        late_peak = max(
            (
                metric.vel_norm
                for metric in full_metrics
                if 160 <= metric.processed_frame <= 220
            ),
            default=0.0,
        )
        print(f"\nmsckf_late_peak_vel_norm={late_peak:.6f}")
        if args.report_extrinsics_sweep:
            sweep_results = []
            sweep_cases = [
                ("configured:camera_in_imu", "configured", "camera_in_imu"),
                ("configured:imu_in_camera", "configured", "imu_in_camera"),
                ("identity", "identity", "camera_in_imu"),
            ]
            for label, mode, convention in sweep_cases:
                sweep_R_IC, sweep_t_IC = _resolve_camera_extrinsics_mode(
                    mode=mode,
                    camera_R_IC=base_camera_R_IC,
                    camera_t_IC=base_camera_t_IC,
                )
                sweep_runner = MsckfReplay(
                    **runner_kwargs,
                    image_processing_width=args.image_processing_width,
                    camera_fx=args.camera_fx,
                    camera_fy=args.camera_fy,
                    camera_cx=args.camera_cx,
                    camera_cy=args.camera_cy,
                    camera_distortion=args.camera_distortion,
                    camera_extrinsics_convention=convention,
                    camera_R_IC=sweep_R_IC,
                    camera_t_IC=sweep_t_IC,
                    max_batch_dx_pos_norm=args.max_batch_dx_pos_norm,
                    max_batch_dx_vel_norm=args.max_batch_dx_vel_norm,
                    max_batch_dx_bias_norm=args.max_batch_dx_bias_norm,
                )
                sweep_metrics, _, _ = _run_replay(
                    events,
                    start_index=0,
                    stop_processed_image=args.full_stop_image,
                    snapshot_frame=None,
                    runner=sweep_runner,
                    bias_mode=args.bias_mode,
                )
                sweep_results.append(
                    _summarize_extrinsics_sweep(label, sweep_metrics)
                )
            _print_extrinsics_sweep_report(sweep_results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
