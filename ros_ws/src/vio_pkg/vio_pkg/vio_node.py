import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, Image, PointCloud2, PointField, CameraInfo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
import csv
import struct
import time
import numpy as np
import threading
import queue

from .utils.common import CameraPose, ImuData
from .backend.math_utils import matrix_to_quaternion, quaternion_to_matrix
from .frontend.interfaces import FrontendConfig
from .frontend.feature_manager import FeatureManager
from .frontend.detectors import ShiTomasiDetector
from .frontend.trackers import KLTTracker
from .backend.state_server import StateServer
from .backend.propagator import ImuPropagator
from .backend.msckf_updater import MSCKFUpdater


def put_latest_image(image_queue: queue.Queue, item) -> bool:
    """
    Enqueue the newest image. If the worker is behind, discard one stale frame
    so the frontend does not process old images while newer data is dropped.
    """
    try:
        image_queue.put_nowait(item)
        return False
    except queue.Full:
        try:
            image_queue.get_nowait()
        except queue.Empty:
            pass
        image_queue.put_nowait(item)
        return True


def prepare_image_for_processing(
    image: np.ndarray,
    target_width: int = 0,
) -> tuple[np.ndarray, float]:
    """
    Optionally resize an image before frontend tracking.

    target_width <= 0 keeps the input resolution. Wider images are resized to
    target_width while preserving aspect ratio; smaller images are not upscaled.
    """
    image_width = int(image.shape[1])
    if target_width <= 0 or image_width <= target_width:
        return image, 1.0

    import cv2

    scale = float(target_width) / float(image_width)
    target_height = max(1, int(round(float(image.shape[0]) * scale)))
    resized = cv2.resize(
        image,
        (int(target_width), target_height),
        interpolation=cv2.INTER_AREA,
    )
    return resized, scale


def compute_effective_camera_calibration(
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    distortion_coefficients,
    scale: float,
) -> tuple[float, float, float, float, np.ndarray]:
    """Scale pinhole intrinsics for resized images; distortion is unchanged."""
    if scale <= 0.0:
        raise ValueError("image processing scale must be positive")

    return (
        float(fx) * scale,
        float(fy) * scale,
        float(cx) * scale,
        float(cy) * scale,
        np.asarray(distortion_coefficients, dtype=np.float64),
    )


def make_sensor_qos(
    depth: int,
    reliability_name: str = "reliable",
) -> QoSProfile:
    reliability_key = reliability_name.strip().lower()
    if reliability_key in ("reliable", "reliability_policy_reliable"):
        reliability = ReliabilityPolicy.RELIABLE
    elif reliability_key in ("best_effort", "besteffort", "best-effort"):
        reliability = ReliabilityPolicy.BEST_EFFORT
    else:
        raise ValueError(
            "input_qos_reliability must be 'reliable' or 'best_effort'"
        )

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
    )


def update_initial_imu_buffer(
    initial_imu_buffer: list[ImuData],
    imu_data: ImuData,
    max_gap_s: float,
) -> tuple[list[ImuData], bool, float]:
    """
    Append an IMU sample to the gravity-init buffer, resetting the buffer if the
    stream jumps backward or stalls long enough to invalidate static averaging.
    """
    if not initial_imu_buffer:
        return [imu_data], False, 0.0

    gap = float(imu_data.timestamp - initial_imu_buffer[-1].timestamp)
    if gap <= 0.0 or gap > max_gap_s:
        return [imu_data], True, gap

    updated = initial_imu_buffer.copy()
    updated.append(imu_data)
    return updated, False, gap


def compute_static_imu_initialization(
    initial_imu_buffer: list[ImuData],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Solve the static startup attitude and biases from buffered IMU samples.
    Returns (q_init, gyro_bias, accel_bias, init_end_time).
    """
    if not initial_imu_buffer:
        raise ValueError("initial_imu_buffer must not be empty")

    a_avg = np.mean([m.accel for m in initial_imu_buffer], axis=0)
    w_avg = np.mean([m.gyro for m in initial_imu_buffer], axis=0)

    z_axis = a_avg / np.linalg.norm(a_avg)
    target_z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z_axis, target_z)
    s = np.linalg.norm(v)
    c = np.dot(z_axis, target_z)

    if s < 1e-6:
        q_init = (
            np.array([1.0, 0.0, 0.0, 0.0])
            if c > 0
            else np.array([0.0, 1.0, 0.0, 0.0])
        )
    else:
        v_skew = np.array(
            [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]
        )
        R = np.eye(3) + v_skew + (v_skew @ v_skew) * ((1 - c) / (s**2))
        q_init = matrix_to_quaternion(R)

    R_init = quaternion_to_matrix(q_init)
    expected_static_accel = R_init.T @ np.array([0.0, 0.0, 9.81])
    ba_init = a_avg - expected_static_accel
    init_end_time = float(initial_imu_buffer[-1].timestamp)
    return q_init, w_avg, ba_init, init_end_time


def is_frame_timestamp_discontinuity(
    previous_image_time: float,
    image_time: float,
    max_gap_s: float,
) -> bool:
    return classify_frame_timestamp_gap(
        previous_image_time,
        image_time,
        max_gap_s,
    ) in ("backward_jump", "large_forward_gap")


def classify_frame_timestamp_gap(
    previous_image_time: float,
    image_time: float,
    max_gap_s: float,
) -> str:
    if previous_image_time < 0.0:
        return "first_frame"

    gap = float(image_time - previous_image_time)
    if gap <= 0.0:
        return "backward_jump"
    if gap > max_gap_s:
        return "large_forward_gap"
    return "ok"


def sanitize_time_gap_threshold(
    configured_value: float,
    fallback_value: float,
) -> float:
    """Return a positive time-gap threshold, falling back on invalid input."""
    if configured_value > 0.0:
        return float(configured_value)
    return float(fallback_value)


class VIOSystemNode(Node):
    def __init__(self):
        super().__init__('vio_system_node')
        
        # Thread-safe queues and locks
        self.imu_buffer = [] 
        self.imu_lock = threading.Lock()

        self.declare_parameter('image_queue_size', 50)
        image_queue_size = (
            self.get_parameter('image_queue_size')
            .get_parameter_value()
            .integer_value
        )
        if image_queue_size < 1:
            self.get_logger().warn(
                "image_queue_size must be >= 1; falling back to 50"
            )
            image_queue_size = 50
        self.image_queue = queue.Queue(maxsize=int(image_queue_size))
        
        # Initialization flags
        self.is_initialized = False
        self.last_image_time = -1.0
        self.initial_imu_buffer = []
        self.gravity_aligned = False
        
        # World map for persistence
        self.global_map_points = []
        self.backend_frame_count = 0
        
        # Modules
        self.frontend_config = FrontendConfig()
        self.declare_parameter('image_processing_width', 0)
        self.declare_parameter('runtime_diagnostics_enabled', False)
        self.declare_parameter('diagnostics_log_every_n_frames', 10)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('log_tracked_frames', True)
        self.declare_parameter('enable_msckf_updates', True)
        self.declare_parameter('stop_after_processed_frames', 0)
        self.declare_parameter('max_imu_init_gap', 0.1)
        self.declare_parameter('max_frame_timestamp_gap', 0.1)
        self.declare_parameter('reset_on_large_frame_gap', False)
        self.declare_parameter(
            'min_triangulation_parallax_deg',
            0.0,
        )
        self.image_processing_width = (
            self.get_parameter('image_processing_width')
            .get_parameter_value()
            .integer_value
        )
        if self.image_processing_width < 0:
            self.get_logger().warn(
                "image_processing_width must be >= 0; disabling resize"
            )
            self.image_processing_width = 0
        self.runtime_diagnostics_enabled = (
            self.get_parameter('runtime_diagnostics_enabled')
            .get_parameter_value()
            .bool_value
        )
        self.diagnostics_log_every_n_frames = (
            self.get_parameter('diagnostics_log_every_n_frames')
            .get_parameter_value()
            .integer_value
        )
        if self.diagnostics_log_every_n_frames < 1:
            self.get_logger().warn(
                "diagnostics_log_every_n_frames must be >= 1; falling back to 10"
            )
            self.diagnostics_log_every_n_frames = 10
        self.publish_debug_image = (
            self.get_parameter('publish_debug_image')
            .get_parameter_value()
            .bool_value
        )
        self.log_tracked_frames = (
            self.get_parameter('log_tracked_frames')
            .get_parameter_value()
            .bool_value
        )
        self.enable_msckf_updates = (
            self.get_parameter('enable_msckf_updates')
            .get_parameter_value()
            .bool_value
        )
        self.stop_after_processed_frames = (
            self.get_parameter('stop_after_processed_frames')
            .get_parameter_value()
            .integer_value
        )
        if self.stop_after_processed_frames < 0:
            self.get_logger().warn(
                "stop_after_processed_frames must be >= 0; disabling frame stop"
            )
            self.stop_after_processed_frames = 0
        configured_max_imu_init_gap = (
            self.get_parameter('max_imu_init_gap')
            .get_parameter_value()
            .double_value
        )
        configured_max_frame_timestamp_gap = (
            self.get_parameter('max_frame_timestamp_gap')
            .get_parameter_value()
            .double_value
        )
        self.reset_on_large_frame_gap = (
            self.get_parameter('reset_on_large_frame_gap')
            .get_parameter_value()
            .bool_value
        )
        self.max_imu_init_gap = sanitize_time_gap_threshold(
            configured_max_imu_init_gap,
            fallback_value=0.1,
        )
        self.max_frame_timestamp_gap = sanitize_time_gap_threshold(
            configured_max_frame_timestamp_gap,
            fallback_value=0.1,
        )
        if configured_max_imu_init_gap <= 0.0:
            self.get_logger().warn(
                "max_imu_init_gap must be > 0; falling back to 0.1"
            )
        if configured_max_frame_timestamp_gap <= 0.0:
            self.get_logger().warn(
                "max_frame_timestamp_gap must be > 0; falling back to 0.1"
            )
        self.detector = ShiTomasiDetector(self.frontend_config)
        self.tracker = KLTTracker(self.frontend_config)
        self.frontend = FeatureManager(self.detector, self.tracker, self.frontend_config)
        
        self.state_server = StateServer()
        self.imu_propagator = ImuPropagator(self.state_server)
        self.msckf_updater = MSCKFUpdater(self.state_server)
        self.declare_parameter('max_imu_dt', self.imu_propagator.max_imu_dt)
        self.declare_parameter(
            'max_batch_dx_pos_norm',
            self.msckf_updater.max_batch_dx_pos_norm,
        )
        self.declare_parameter(
            'max_batch_dx_vel_norm',
            self.msckf_updater.max_batch_dx_vel_norm,
        )
        self.declare_parameter(
            'max_batch_dx_bias_norm',
            self.msckf_updater.max_batch_dx_bias_norm,
        )
        self.imu_propagator.max_imu_dt = (
            self.get_parameter('max_imu_dt')
            .get_parameter_value()
            .double_value
        )
        if self.imu_propagator.max_imu_dt <= 0.0:
            self.get_logger().warn("max_imu_dt must be > 0; falling back to 0.05")
            self.imu_propagator.max_imu_dt = 0.05
        self.msckf_updater.max_batch_dx_pos_norm = (
            self.get_parameter('max_batch_dx_pos_norm')
            .get_parameter_value()
            .double_value
        )
        self.msckf_updater.max_batch_dx_vel_norm = (
            self.get_parameter('max_batch_dx_vel_norm')
            .get_parameter_value()
            .double_value
        )
        self.msckf_updater.max_batch_dx_bias_norm = (
            self.get_parameter('max_batch_dx_bias_norm')
            .get_parameter_value()
            .double_value
        )
        self.msckf_updater.min_triangulation_parallax_deg = (
            self.get_parameter('min_triangulation_parallax_deg')
            .get_parameter_value()
            .double_value
        )
        if self.msckf_updater.max_batch_dx_pos_norm <= 0.0:
            self.get_logger().warn(
                "max_batch_dx_pos_norm must be > 0; falling back to 0.5"
            )
            self.msckf_updater.max_batch_dx_pos_norm = 0.5
        if self.msckf_updater.max_batch_dx_vel_norm <= 0.0:
            self.get_logger().warn(
                "max_batch_dx_vel_norm must be > 0; falling back to 1.0"
            )
            self.msckf_updater.max_batch_dx_vel_norm = 1.0
        if self.msckf_updater.max_batch_dx_bias_norm <= 0.0:
            self.get_logger().warn(
                "max_batch_dx_bias_norm must be > 0; falling back to 0.25"
            )
            self.msckf_updater.max_batch_dx_bias_norm = 0.25
        if self.msckf_updater.min_triangulation_parallax_deg < 0.0:
            self.get_logger().warn(
                "min_triangulation_parallax_deg must be >= 0; falling back to 0.0"
            )
            self.msckf_updater.min_triangulation_parallax_deg = 0.0

        self.declare_parameter(
            'camera_R_IC',
            self.state_server.R_IC.reshape(-1).tolist(),
        )
        self.declare_parameter(
            'camera_t_IC',
            self.state_server.t_IC.tolist(),
        )
        self.declare_parameter(
            'camera_extrinsics_convention',
            'camera_in_imu',
        )
        camera_R_IC = np.array(
            list(
                self.get_parameter('camera_R_IC')
                .get_parameter_value()
                .double_array_value
            ),
            dtype=np.float64,
        ).reshape(3, 3)
        camera_t_IC = list(
            self.get_parameter('camera_t_IC')
            .get_parameter_value()
            .double_array_value
        )
        camera_extrinsics_convention = (
            self.get_parameter('camera_extrinsics_convention')
            .get_parameter_value()
            .string_value
        )
        self.state_server.set_camera_extrinsics(
            camera_R_IC,
            camera_t_IC,
            convention=camera_extrinsics_convention,
        )
        self.get_logger().info(
            "Camera-IMU extrinsics: "
            f"input_convention={camera_extrinsics_convention}, "
            f"R_IC={self.state_server.R_IC.tolist()}, "
            f"t_IC={self.state_server.t_IC.tolist()}"
        )

        self.declare_parameter('camera_fx', self.msckf_updater.fx)
        self.declare_parameter('camera_fy', self.msckf_updater.fy)
        self.declare_parameter('camera_cx', self.msckf_updater.cx)
        self.declare_parameter('camera_cy', self.msckf_updater.cy)
        self.declare_parameter(
            'camera_distortion',
            self.msckf_updater.distortion_coefficients.tolist(),
        )
        camera_distortion = list(
            self.get_parameter('camera_distortion')
            .get_parameter_value()
            .double_array_value
        )
        self.base_camera_fx = (
            self.get_parameter('camera_fx').get_parameter_value().double_value
        )
        self.base_camera_fy = (
            self.get_parameter('camera_fy').get_parameter_value().double_value
        )
        self.base_camera_cx = (
            self.get_parameter('camera_cx').get_parameter_value().double_value
        )
        self.base_camera_cy = (
            self.get_parameter('camera_cy').get_parameter_value().double_value
        )
        self.base_camera_distortion = np.asarray(camera_distortion, dtype=np.float64)
        self._calibration_scale_applied = None
        self._apply_effective_camera_calibration(scale=1.0)
        self.get_logger().info(
            "Base camera calibration: "
            f"fx={self.base_camera_fx:.6f}, "
            f"fy={self.base_camera_fy:.6f}, "
            f"cx={self.base_camera_cx:.6f}, "
            f"cy={self.base_camera_cy:.6f}, "
            f"distortion={self.base_camera_distortion.tolist()}"
        )
        self.get_logger().info(
            "Image processing: "
            f"target_width={self.image_processing_width}, "
            f"queue_size={self.image_queue.maxsize}, "
            f"diagnostics={self.runtime_diagnostics_enabled}, "
            f"publish_debug_image={self.publish_debug_image}, "
            f"log_tracked_frames={self.log_tracked_frames}, "
            f"enable_msckf_updates={self.enable_msckf_updates}, "
            f"stop_after_processed_frames={self.stop_after_processed_frames}, "
            f"max_imu_init_gap={self.max_imu_init_gap:.4f}, "
            f"max_frame_timestamp_gap={self.max_frame_timestamp_gap:.4f}, "
            f"reset_on_large_frame_gap={self.reset_on_large_frame_gap}, "
            f"max_imu_dt={self.imu_propagator.max_imu_dt:.4f}, "
            f"max_dx_pos={self.msckf_updater.max_batch_dx_pos_norm:.4f}, "
            f"max_dx_vel={self.msckf_updater.max_batch_dx_vel_norm:.4f}, "
            f"max_dx_bias={self.msckf_updater.max_batch_dx_bias_norm:.4f}, "
            "min_triangulation_parallax_deg="
            f"{self.msckf_updater.min_triangulation_parallax_deg:.4f}"
        )
        self.images_received = 0
        self.images_enqueued = 0
        self.images_dropped = 0
        self.images_processed = 0
        self._discontinuity_reset_count = 0
        self._shutdown_requested = False
        
        # ROS setup
        self.declare_parameter('input_qos_reliability', 'reliable')
        input_qos_reliability = (
            self.get_parameter('input_qos_reliability')
            .get_parameter_value()
            .string_value
        )
        imu_qos = make_sensor_qos(
            depth=100,
            reliability_name=input_qos_reliability,
        )
        image_qos = make_sensor_qos(
            depth=10,
            reliability_name=input_qos_reliability,
        )
        self.get_logger().info(
            f"Input sensor QoS reliability: {input_qos_reliability}"
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu0',
            self.imu_callback,
            imu_qos,
        )
        self.img_sub = self.create_subscription(
            Image,
            '/cam0/image_raw',
            self.image_callback,
            image_qos,
        )
        
        # Publishers for Output and Visualization
        self.odom_pub = self.create_publisher(Odometry, '/vio/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/vio/path', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/vio/point_cloud', 10)
        self.camera_marker_pub = self.create_publisher(Marker, '/vio/camera_pose', 10)
        self.debug_img_pub = self.create_publisher(Image, '/vio/camera/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/vio/camera/camera_info', 10)
        self.gt_path_pub = self.create_publisher(Path, '/vio/gt_path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.path_msg = Path()

        # Load ground truth from EuRoC CSV and publish once on a latched-style timer
        self.declare_parameter('gt_csv_path', '')
        self.declare_parameter('imu_init_sample_count', 200)
        gt_csv = self.get_parameter('gt_csv_path').get_parameter_value().string_value
        self.imu_init_sample_count = (
            self.get_parameter('imu_init_sample_count')
            .get_parameter_value()
            .integer_value
        )
        if self.imu_init_sample_count < 1:
            self.get_logger().warn(
                "imu_init_sample_count must be >= 1; falling back to 200"
            )
            self.imu_init_sample_count = 200
        self.gt_path_msg = self._load_gt_path(gt_csv)
        self._gt_timer = self.create_timer(1.0, self._publish_gt_path)
        
        # Start ordered VIO worker. Each image runs predict, clone, frontend,
        # MSCKF update, and publish in timestamp order.
        self.frontend_thread = threading.Thread(target=self.frontend_worker, daemon=True)
        self.frontend_thread.start()
        
        self.get_logger().info("VIO System Node initialized (ordered image worker).")

    def imu_callback(self, msg: Imu):
        if not hasattr(self, '_first_imu_rcv'):
            self.get_logger().info("Received FIRST IMU topic msg!")
            self._first_imu_rcv = True
            
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        imu_data = ImuData(
            timestamp=curr_time,
            accel=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]),
            gyro=np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        )
        with self.imu_lock:
            self.imu_buffer.append(imu_data)
            
            # Static IMU Initialization
            if not self.gravity_aligned:
                updated_init_buffer, was_reset, gap = update_initial_imu_buffer(
                    self.initial_imu_buffer,
                    imu_data,
                    self.max_imu_init_gap,
                )
                if was_reset:
                    self.get_logger().warn(
                        "Resetting IMU init buffer after timestamp discontinuity: "
                        f"dt={gap:.4f}s"
                    )
                self.initial_imu_buffer = updated_init_buffer
                if len(self.initial_imu_buffer) >= self.imu_init_sample_count:
                    q_init, w_avg, ba_init, init_end_time = (
                        compute_static_imu_initialization(self.initial_imu_buffer)
                    )
                    a_avg = np.mean(
                        [m.accel for m in self.initial_imu_buffer],
                        axis=0,
                    )
                    with self.state_server.lock:
                        self.state_server.state.quaternion = q_init
                        self.state_server.state.gyro_bias = w_avg
                        self.state_server.state.accel_bias = ba_init
                        self.state_server.state.timestamp = init_end_time
                        
                    init_start_time = self.initial_imu_buffer[0].timestamp
                    init_duration = init_end_time - init_start_time
                    self.imu_buffer = [
                        m for m in self.imu_buffer
                        if m.timestamp > init_end_time
                    ]
                    self.gravity_aligned = True
                    self.get_logger().info(
                        "Gravity Aligned! Initial Pitch/Roll solved. "
                        f"samples: {len(self.initial_imu_buffer)}, "
                        f"duration: {init_duration:.3f}s, "
                        f"q_init={q_init.tolist()}, "
                        f"a_avg={a_avg.tolist()}, |a_avg|={np.linalg.norm(a_avg):.6f}, "
                        f"w_avg={w_avg.tolist()}, "
                        f"ba_init={ba_init.tolist()}, |ba_init|={np.linalg.norm(ba_init):.6f}"
                    )
                    
    def image_callback(self, msg: Image):
        if not self.gravity_aligned:
            return  # Wait for IMU to align first
        if not hasattr(self, '_first_img_rcv'):
            self.get_logger().info("Received FIRST Image topic msg!")
            self._first_img_rcv = True
            
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        decode_start = time.perf_counter()
        full_res_img = self._ros_image_to_gray(msg)
        cv_img, image_scale = prepare_image_for_processing(
            full_res_img,
            target_width=int(self.image_processing_width),
        )
        decode_resize_ms = (time.perf_counter() - decode_start) * 1000.0
        self._apply_effective_camera_calibration(
            scale=image_scale,
            original_shape=full_res_img.shape,
            processed_shape=cv_img.shape,
        )

        self.images_received += 1
        if put_latest_image(
            self.image_queue,
            (curr_time, cv_img, image_scale, decode_resize_ms),
        ):
            self.images_dropped += 1
            self.get_logger().warn("Image queue full. Dropped stale frame.")
        self.images_enqueued += 1

    def _load_gt_path(self, csv_path: str) -> Path:
        path = Path()
        path.header.frame_id = 'world'
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # skip header
                for row in reader:
                    ts_ns = int(row[0])
                    sec = ts_ns // 1_000_000_000
                    nanosec = ts_ns % 1_000_000_000
                    ps = PoseStamped()
                    ps.header.frame_id = 'world'
                    ps.header.stamp.sec = sec
                    ps.header.stamp.nanosec = nanosec
                    ps.pose.position.x = float(row[1])
                    ps.pose.position.y = float(row[2])
                    ps.pose.position.z = float(row[3])
                    ps.pose.orientation.w = float(row[4])
                    ps.pose.orientation.x = float(row[5])
                    ps.pose.orientation.y = float(row[6])
                    ps.pose.orientation.z = float(row[7])
                    path.poses.append(ps)
            self.get_logger().info(f"Loaded {len(path.poses)} GT poses from {csv_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not load GT CSV: {e}")
        return path

    def _publish_gt_path(self):
        self.gt_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.gt_path_pub.publish(self.gt_path_msg)

    def _ros_image_to_gray(self, msg: Image) -> np.ndarray:
        """
        Decode ROS Image data without cv_bridge. The Humble cv_bridge build in
        this container is compiled against NumPy 1.x and crashes with NumPy 2.x.
        """
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        enc = msg.encoding.lower()

        if enc == "mono8":
            return raw.reshape(msg.height, msg.step)[:, :msg.width].copy()
        if enc == "mono16":
            row_bytes = raw.reshape(msg.height, msg.step)[:, :msg.width * 2]
            img16 = row_bytes.copy().view(np.uint16).reshape(msg.height, msg.width)
            return (img16 >> 8).astype(np.uint8)
        if enc in ("bgr8", "rgb8"):
            import cv2
            row_bytes = raw.reshape(msg.height, msg.step)[:, :msg.width * 3]
            color = row_bytes.copy().reshape(msg.height, msg.width, 3)
            code = cv2.COLOR_BGR2GRAY if enc == "bgr8" else cv2.COLOR_RGB2GRAY
            return cv2.cvtColor(color, code)

        raise ValueError(f"Unsupported image encoding: {msg.encoding!r}")

    def _apply_effective_camera_calibration(
        self,
        scale: float,
        original_shape: tuple | None = None,
        processed_shape: tuple | None = None,
    ) -> None:
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

        shape_note = ""
        if original_shape is not None and processed_shape is not None:
            shape_note = f", image_shape={original_shape}->{processed_shape}"
        self.get_logger().info(
            "Effective camera calibration: "
            f"scale={scale:.6f}, "
            f"fx={fx:.6f}, fy={fy:.6f}, "
            f"cx={cx:.6f}, cy={cy:.6f}"
            f"{shape_note}"
        )

    def _bgr_image_to_msg(self, image: np.ndarray) -> Image:
        msg = Image()
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.shape[1] * 3)
        msg.data = image.tobytes()
        return msg

    def frontend_worker(self):
        """
        Thread 2: Pops images, correlates IMU data, and runs Computer Vision tracking.
        """
        while rclpy.ok():
            try:
                # 1. Block until a new image arrives (1 s timeout to allow clean shutdown).
                try:
                    (
                        image_time,
                        image,
                        image_scale,
                        decode_resize_ms,
                    ) = self.image_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                frame_start = time.perf_counter()
                previous_image_time = self.last_image_time
                timestamp_gap = 0.0
                if previous_image_time >= 0.0:
                    timestamp_gap = image_time - previous_image_time
                gap_kind = classify_frame_timestamp_gap(
                    previous_image_time,
                    image_time,
                    self.max_frame_timestamp_gap,
                )
                if self._should_reset_for_frame_gap_kind(gap_kind):
                    self._handle_frame_timestamp_discontinuity(
                        image_time=image_time,
                        timestamp_gap=timestamp_gap,
                        gap_kind=gap_kind,
                    )
                    continue
                if gap_kind == "large_forward_gap":
                    self._skip_frontend_update_after_large_forward_gap(
                        image_time=image_time,
                        timestamp_gap=timestamp_gap,
                    )
                    continue

                # 2. Extract IMU measurements between the previous and current frame.
                with self.imu_lock:
                    imu_measurements = [
                        m for m in self.imu_buffer
                        if self.last_image_time < m.timestamp <= image_time
                    ]
                    self.imu_buffer = [
                        m for m in self.imu_buffer
                        if m.timestamp > image_time
                    ]

                # 3. IMU Prediction (propagate state forward — backend stub for now).
                imu_span = 0.0
                if imu_measurements:
                    imu_span = (
                        imu_measurements[-1].timestamp
                        - imu_measurements[0].timestamp
                    )
                propagate_start = time.perf_counter()
                if imu_measurements:
                    self.imu_propagator.propagate(imu_measurements)
                propagate_ms = (time.perf_counter() - propagate_start) * 1000.0

                # 4. Add the camera clone at this image timestamp, then use
                # that exact clone pose for frontend track bookkeeping.
                with self.state_server.lock:
                    state = self.state_server.state
                    self.state_server.add_clone(image_time, state.position, state.quaternion)
                    clone = self.state_server.state.clone_poses[-1]
                    current_cam_pose = CameraPose(
                        timestamp=clone.timestamp,
                        position=clone.position.copy(),
                        quaternion=clone.quaternion.copy(),
                    )

                # 5. Feature Tracking.
                frontend_start = time.perf_counter()
                mature_features = self.frontend.process_image(
                    image_time, image, current_cam_pose
                )
                frontend_ms = (time.perf_counter() - frontend_start) * 1000.0

                # 6. MSCKF measurement update and map point triangulation.
                backend_start = time.perf_counter()
                with self.state_server.lock:
                    update_stats = self._run_msckf_update(mature_features)
                    state = self.state_server.state
                    state_snapshot = (
                        state.position.copy(),
                        state.velocity.copy(),
                        state.accel_bias.copy(),
                        state.gyro_bias.copy(),
                    )

                    new_points_count = 0
                    for f in mature_features:
                        p = self.msckf_updater.triangulate_feature(f)
                        if p is not None:
                            self.global_map_points.append(p)
                            new_points_count += 1

                    if len(self.global_map_points) > 100000:
                        self.global_map_points = self.global_map_points[-100000:]
                backend_ms = (time.perf_counter() - backend_start) * 1000.0

                # 7. Publish visualizer image
                publish_start = time.perf_counter()
                sec = int(image_time)
                nanosec = int((image_time - sec) * 1e9)
                image_stamp = Time(sec=sec, nanosec=nanosec)
                if self.publish_debug_image:
                    import cv2

                    debug_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    for track in self.frontend.active_tracks:
                        pt = track.observations[-1]
                        cv2.circle(
                            debug_img,
                            (int(pt[0]), int(pt[1])),
                            3,
                            (0, 255, 0),
                            -1,
                        )
                    img_msg = self._bgr_image_to_msg(debug_img)
                    img_msg.header.stamp = image_stamp
                    img_msg.header.frame_id = "cam0"
                    self.debug_img_pub.publish(img_msg)

                # Publish CameraInfo cho AR Overlay rviz2
                cam_info = CameraInfo()
                cam_info.header.stamp = image_stamp
                cam_info.header.frame_id = "cam0"
                cam_info.width = image.shape[1]
                cam_info.height = image.shape[0]
                cam_info.distortion_model = "plumb_bob"
                fx, fy = self.msckf_updater.fx, self.msckf_updater.fy
                cx, cy = self.msckf_updater.cx, self.msckf_updater.cy
                cam_info.d = [
                    float(x)
                    for x in self.msckf_updater.distortion_coefficients
                ]
                cam_info.k = [float(fx), 0.0, float(cx), 0.0, float(fy), float(cy), 0.0, 0.0, 1.0]
                cam_info.p = [float(fx), 0.0, float(cx), 0.0, 0.0, float(fy), float(cy), 0.0, 0.0, 0.0, 1.0, 0.0]
                self.cam_info_pub.publish(cam_info)
                publish_ms = (time.perf_counter() - publish_start) * 1000.0

                # 8. Publish state and diagnostics after the visual update.
                self.publish_state(image_time)
                self.backend_frame_count += 1
                self.images_processed += 1
                if self.backend_frame_count % 10 == 0:
                    pos, vel, accel_bias, gyro_bias = state_snapshot
                    self.get_logger().info(
                        "MSCKF diagnostics: "
                        f"mature={update_stats.get('mature', 0)}, "
                        f"triangulated={update_stats.get('triangulated', 0)}, "
                        f"accepted={update_stats.get('accepted', 0)}, "
                        f"batch_rejected={update_stats.get('batch_rejected', 0)}, "
                        f"gated_out={update_stats.get('gated_out', 0)}, "
                        f"invalid={update_stats.get('invalid_jacobian', 0)}, "
                        f"triangulation_failed={update_stats.get('triangulation_failed', 0)}, "
                        f"rejected_ill_conditioned={update_stats.get('rejected_ill_conditioned', 0)}, "
                        f"rejected_unreasonable_dx={update_stats.get('rejected_unreasonable_dx', 0)}, "
                        f"skipped={update_stats.get('skipped', 0)}, "
                        f"rows={update_stats.get('rows', 0)}, "
                        f"dx_pos={update_stats.get('dx_pos_norm', 0.0):.3e}, "
                        f"dx_vel={update_stats.get('dx_vel_norm', 0.0):.3e}, "
                        f"dx_bg={update_stats.get('dx_bg_norm', 0.0):.3e}, "
                        f"dx_ba={update_stats.get('dx_accel_bias_norm', 0.0):.3e}, "
                        f"innovation_cond={update_stats.get('innovation_condition_number', 0.0):.3e}, "
                        f"pos={pos}, vel={vel}, ba={accel_bias}, bg={gyro_bias}"
                    )
                if (
                    self.runtime_diagnostics_enabled
                    and self.images_processed % self.diagnostics_log_every_n_frames == 0
                ):
                    pos, vel, accel_bias, gyro_bias = state_snapshot
                    total_ms = (time.perf_counter() - frame_start) * 1000.0
                    self.get_logger().info(
                        "Runtime diagnostics: "
                        f"frame={self.images_processed}, "
                        f"shape={image.shape}, "
                        f"scale={image_scale:.6f}, "
                        f"queue={self.image_queue.qsize()}/{self.image_queue.maxsize}, "
                        f"received={self.images_received}, "
                        f"enqueued={self.images_enqueued}, "
                        f"dropped={self.images_dropped}, "
                        f"dt_img={timestamp_gap:.4f}s, "
                        f"imu_count={len(imu_measurements)}, "
                        f"imu_span={imu_span:.4f}s, "
                        f"imu_large_dt_skips={self.imu_propagator.last_large_dt_count}, "
                        f"imu_max_dt={self.imu_propagator.last_max_dt:.4f}s, "
                        f"decode_resize={decode_resize_ms:.1f}ms, "
                        f"propagate={propagate_ms:.1f}ms, "
                        f"frontend={frontend_ms:.1f}ms, "
                        f"backend={backend_ms:.1f}ms, "
                        f"publish={publish_ms:.1f}ms, "
                        f"total={total_ms:.1f}ms, "
                        f"pos={pos}, "
                        f"vel_norm={np.linalg.norm(vel):.3f}, "
                        f"|ba|={np.linalg.norm(accel_bias):.3f}, "
                        f"|bg|={np.linalg.norm(gyro_bias):.3f}"
                    )
                    if timestamp_gap > self.max_frame_timestamp_gap:
                        self.get_logger().warn(
                            f"Large image timestamp gap: {timestamp_gap:.4f}s"
                        )
                if self.log_tracked_frames:
                    self.get_logger().info(
                        f"Successfully tracked and published frame at ts={image_time:.3f} "
                        f"(New Points: {new_points_count})"
                    )

                self.last_image_time = image_time
                if self._should_request_stop():
                    self._request_shutdown(
                        "Reached stop_after_processed_frames="
                        f"{self.stop_after_processed_frames}"
                    )
            except Exception as e:
                self.get_logger().error(f"Ordered VIO Worker Crashed: {e}")

    def _run_msckf_update(self, mature_features):
        if not self.enable_msckf_updates:
            return {
                "mature": len(mature_features),
                "too_short": 0,
                "triangulated": 0,
                "triangulation_failed": 0,
                "invalid_jacobian": 0,
                "gated_out": 0,
                "accepted": 0,
                "rows": 0,
                "dx_norm": 0.0,
                "dx_pos_norm": 0.0,
                "dx_vel_norm": 0.0,
                "dx_bg_norm": 0.0,
                "dx_accel_bias_norm": 0.0,
                "batch_rejected": 0,
                "rejected_ill_conditioned": 0,
                "rejected_unreasonable_dx": 0,
                "innovation_condition_number": 0.0,
                "skipped": 1,
            }

        self.msckf_updater.process_mature_features(mature_features)
        return dict(self.msckf_updater.last_update_stats)

    def _should_reset_for_frame_gap(self, timestamp_gap: float) -> bool:
        if self.last_image_time < 0.0:
            return False
        gap_kind = classify_frame_timestamp_gap(
            self.last_image_time,
            self.last_image_time + float(timestamp_gap),
            self.max_frame_timestamp_gap,
        )
        return self._should_reset_for_frame_gap_kind(gap_kind)

    def _should_reset_for_frame_gap_kind(self, gap_kind: str) -> bool:
        if gap_kind in ("first_frame", "ok"):
            return False
        if gap_kind == "backward_jump":
            return True
        if gap_kind == "large_forward_gap":
            return self.reset_on_large_frame_gap
        raise ValueError(f"Unknown frame timestamp gap classification: {gap_kind}")

    def _handle_frame_timestamp_discontinuity(
        self,
        image_time: float,
        timestamp_gap: float,
        gap_kind: str,
    ) -> None:
        self._discontinuity_reset_count += 1
        self.get_logger().warn(
            "Resetting temporal state after image timestamp discontinuity: "
            f"kind={gap_kind}, dt_img={timestamp_gap:.4f}s, "
            f"reset_count={self._discontinuity_reset_count}"
        )

        with self.imu_lock:
            self.imu_buffer = [
                m for m in self.imu_buffer
                if m.timestamp > image_time
            ]

        self.frontend.reset()
        with self.state_server.lock:
            self.state_server.clear_clones()
            self.state_server.state.timestamp = image_time
        self.last_image_time = image_time

    def _skip_frontend_update_after_large_forward_gap(
        self,
        image_time: float,
        timestamp_gap: float,
    ) -> None:
        with self.imu_lock:
            imu_measurements = [
                m for m in self.imu_buffer
                if self.last_image_time < m.timestamp <= image_time
            ]
            self.imu_buffer = [
                m for m in self.imu_buffer
                if m.timestamp > image_time
            ]
        if imu_measurements:
            self.imu_propagator.propagate(imu_measurements)
        self.get_logger().warn(
            "Skipping visual update after large forward image gap: "
            f"dt_img={timestamp_gap:.4f}s"
        )
        self.frontend.reset()
        self.last_image_time = image_time

    def _should_request_stop(self) -> bool:
        return (
            self.stop_after_processed_frames > 0
            and self.images_processed >= self.stop_after_processed_frames
        )

    def _request_shutdown(self, reason: str) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.get_logger().info(f"Stopping node: {reason}")
        if rclpy.ok():
            rclpy.shutdown()

    def publish_state(self, timestamp: float):
        """
        Publish the current state (Odometry), Historical Path, and Camera Frustum.
        """
        with self.state_server.lock:
            state = self.state_server.state
            q_ic = matrix_to_quaternion(self.state_server.R_IC)
            t_ic = self.state_server.t_IC.copy()
            
            # 1. Odometry 
            odom = Odometry()
            
            # Use the actual exact dataset timestamp instead of system time
            sec = int(timestamp)
            nanosec = int((timestamp - sec) * 1e9)
            current_ros_time = Time(sec=sec, nanosec=nanosec)
        
        odom.header.stamp = current_ros_time
        odom.header.frame_id = "world"
        odom.child_frame_id = "imu"
        
        odom.pose.pose.position = Point(x=state.position[0], y=state.position[1], z=state.position[2])
        # Note: Depending on your quaternion format [w, x, y, z] or [x, y, z, w], adapt this:
        odom.pose.pose.orientation = Quaternion(w=state.quaternion[0], x=state.quaternion[1], 
                                                y=state.quaternion[2], z=state.quaternion[3])
        self.odom_pub.publish(odom)

        # 2. Path
        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        
        self.path_msg.header.frame_id = "world"
        self.path_msg.poses.append(pose_stamped)
        self.path_pub.publish(self.path_msg)

        # 3. TF Broadcaster (world to imu)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "imu"
        t.transform.translation.x = state.position[0]
        t.transform.translation.y = state.position[1]
        t.transform.translation.z = state.position[2]
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        
        # 3b. TF Broadcaster (imu to cam0) to help RViz display images in 3D
        t_cam = TransformStamped()
        t_cam.header.stamp = odom.header.stamp
        t_cam.header.frame_id = "imu"
        t_cam.child_frame_id = "cam0"
        t_cam.transform.translation.x = float(t_ic[0])
        t_cam.transform.translation.y = float(t_ic[1])
        t_cam.transform.translation.z = float(t_ic[2])
        t_cam.transform.rotation.w = float(q_ic[0])
        t_cam.transform.rotation.x = float(q_ic[1])
        t_cam.transform.rotation.y = float(q_ic[2])
        t_cam.transform.rotation.z = float(q_ic[3])
        self.tf_broadcaster.sendTransform(t_cam)
        
        # 4. Point Cloud Marker for RViz (Awesome visuals)
        if len(self.global_map_points) > 0:
            pc_msg = self.create_point_cloud_msg(odom.header, self.global_map_points)
            self.pc_pub.publish(pc_msg)
            
        # 5. Camera Frustum Marker
        cam_marker = Marker()
        cam_marker.header = odom.header
        cam_marker.header.frame_id = "cam0"
        cam_marker.ns = "camera_frustum"
        cam_marker.id = 0
        cam_marker.type = Marker.LINE_LIST
        cam_marker.action = Marker.ADD
        cam_marker.scale.x = 0.05 # line thickness
        cam_marker.color.a = 1.0
        cam_marker.color.r = 0.0
        cam_marker.color.g = 1.0 # Green camera cage
        cam_marker.color.b = 0.0
        
        # Frustum shape
        w, h, z = 0.3, 0.2, 0.4
        p_cam = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=w, y=h, z=z), Point(x=w, y=-h, z=z),
            Point(x=-w, y=-h, z=z), Point(x=-w, y=h, z=z)
        ]
        
        # Connecting lines
        cam_marker.points.extend([
            p_cam[0], p_cam[1], p_cam[0], p_cam[2],
            p_cam[0], p_cam[3], p_cam[0], p_cam[4],
            p_cam[1], p_cam[2], p_cam[2], p_cam[3],
            p_cam[3], p_cam[4], p_cam[4], p_cam[1]
        ])
        
        self.camera_marker_pub.publish(cam_marker)

    def create_point_cloud_msg(self, header, points):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))
        
        msg.data = b''.join(buffer)
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = VIOSystemNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
