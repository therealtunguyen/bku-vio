import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from cv_bridge import CvBridge
import numpy as np
import threading
import queue

from .utils.common import CameraPose, ImuData
from .frontend.interfaces import FrontendConfig
from .frontend.feature_manager import FeatureManager
from .frontend.detectors import ShiTomasiDetector
from .frontend.trackers import KLTTracker
from .backend.state_server import StateServer
from .backend.propagator import ImuPropagator
from .backend.msckf_updater import MSCKFUpdater

class VIOSystemNode(Node):
    def __init__(self):
        super().__init__('vio_system_node')
        
        self.bridge = CvBridge()
        
        # Thread-safe queues and locks
        self.imu_buffer = [] 
        self.imu_lock = threading.Lock()
        
        self.image_queue = queue.Queue(maxsize=50)
        self.measurement_queue = queue.Queue(maxsize=50)
        
        # Initialization flags
        self.is_initialized = False
        self.last_image_time = -1.0
        
        # Modules
        self.frontend_config = FrontendConfig()
        self.detector = ShiTomasiDetector(self.frontend_config)
        self.tracker = KLTTracker(self.frontend_config)
        self.frontend = FeatureManager(self.detector, self.tracker, self.frontend_config)
        
        self.state_server = StateServer()
        self.imu_propagator = ImuPropagator(self.state_server)
        self.msckf_updater = MSCKFUpdater(self.state_server)
        
        # ROS setup
        # TODO: Change topics to match your dataset (e.g. EuRoC cam0/image_raw, imu0)
        self.imu_sub = self.create_subscription(Imu, '/imu0', self.imu_callback, 100)
        self.img_sub = self.create_subscription(Image, '/cam0/image_raw', self.image_callback, 10)
        
        # Publishers for Output and Visualization
        self.odom_pub = self.create_publisher(Odometry, '/vio/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/vio/path', 10)
        self.path_msg = Path()
        
        # Start Threads
        self.frontend_thread = threading.Thread(target=self.frontend_worker, daemon=True)
        self.backend_thread = threading.Thread(target=self.backend_worker, daemon=True)
        self.frontend_thread.start()
        self.backend_thread.start()
        
        self.get_logger().info("VIO System Node initialized (Multi-threaded).")

    def imu_callback(self, msg: Imu):
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        imu_data = ImuData(
            timestamp=curr_time,
            accel=np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]),
            gyro=np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        )
        with self.imu_lock:
            self.imu_buffer.append(imu_data)

    def image_callback(self, msg: Image):
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        try:
            self.image_queue.put_nowait((curr_time, cv_img))
        except queue.Full:
            self.get_logger().warn("Image queue full. Dropped frame.")

    def frontend_worker(self):
        """
        Thread 2: Pops images, correlates IMU data, and runs Computer Vision tracking.
        """
        while rclpy.ok():
            # 1. Block until a new image arrives (1 s timeout to allow clean shutdown).
            try:
                image_time, image = self.image_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # 2. Extract IMU measurements between the previous and current frame.
            with self.imu_lock:
                imu_measurements = [
                    m for m in self.imu_buffer
                    if self.last_image_time < m.timestamp <= image_time
                ]
                # Discard all consumed IMU data. Samples with timestamp >
                # image_time have not arrived yet or just arrived and will
                # be picked up by the next integration window.
                self.imu_buffer = [
                    m for m in self.imu_buffer
                    if m.timestamp > image_time
                ]

            # 3. IMU Prediction (propagate state forward — backend stub for now).
            if imu_measurements:
                self.imu_propagator.propagate(imu_measurements)

            # 4. Build current camera pose from propagated state.
            state = self.state_server.state
            current_cam_pose = CameraPose(
                timestamp=image_time,
                position=state.position.copy(),
                quaternion=state.quaternion.copy(),
            )

            # 5. Feature Tracking.
            mature_features = self.frontend.process_image(
                image_time, image, current_cam_pose
            )

            # 6. Forward mature features to the backend worker.
            if mature_features:
                try:
                    self.measurement_queue.put_nowait((image_time, mature_features))
                except queue.Full:
                    self.get_logger().warn("Measurement queue full. Dropped features.")

            self.last_image_time = image_time

    def backend_worker(self):
        """
        Thread 3: MSCKF core, running EKF state updates and publishing.
        """
        while rclpy.ok():
            # TODO: Pop mature_features from self.measurement_queue (with timeout)
            
            # --- Phase 4: MSCKF Update ---
            # 1. State Augmentation (clone state)
            # TODO: Call self.state_server.add_clone(...)
            
            # 2. Measurement Update
            # TODO: Call self.msckf_updater.process_mature_features(mature_features)

            # --- Phase 5: Publish Result ---
            # TODO: Call self.publish_state(image_time)
            pass

    def publish_state(self, timestamp: float):
        """
        Publish the current state (Odometry) and the historical Path.
        """
        state = self.state_server.state
        
        # 1. Odometry 
        odom = Odometry()
        odom.header.stamp.sec = int(timestamp)
        odom.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
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

def main(args=None):
    rclpy.init(args=args)
    node = VIOSystemNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
