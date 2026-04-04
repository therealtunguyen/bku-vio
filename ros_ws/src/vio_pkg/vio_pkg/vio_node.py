import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from cv_bridge import CvBridge
import numpy as np
import threading
import queue

from .utils.common import ImuData
from .frontend.feature_manager import FeatureManager
from .frontend.detectors import HarrisDetector
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
        self.detector = HarrisDetector()
        self.tracker = KLTTracker()
        self.frontend = FeatureManager(self.detector, self.tracker)
        
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
            # TODO: Pop image from self.image_queue (with timeout)
            # TODO: Grab synchronized IMU measurements between last_image_time and current from self.imu_buffer
            
            # --- Phase 3: IMU Prediction ---
            # TODO: Call self.imu_propagator.propagate(imu_measurements)
            
            # --- Phase 2: Feature Tracking ---
            # TODO: Call mature_features = self.frontend.process_image(image_time, image, current_cam_pose)
            
            # TODO: Push mature_features into self.measurement_queue
            pass

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
