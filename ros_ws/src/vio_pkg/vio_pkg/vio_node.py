import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, PointCloud2, PointField, CameraInfo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time
import csv
import struct
import numpy as np
import threading
import queue

from .utils.common import CameraPose, ImuData
from .backend.math_utils import matrix_to_quaternion
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
        self.initial_imu_buffer = []
        self.gravity_aligned = False
        
        # World map for persistence
        self.global_map_points = []
        
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
        self.pc_pub = self.create_publisher(PointCloud2, '/vio/point_cloud', 10)
        self.camera_marker_pub = self.create_publisher(Marker, '/vio/camera_pose', 10)
        self.debug_img_pub = self.create_publisher(Image, '/vio/camera/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/vio/camera/camera_info', 10)
        self.gt_path_pub = self.create_publisher(Path, '/vio/gt_path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.path_msg = Path()

        # Load ground truth from EuRoC CSV and publish once on a latched-style timer
        gt_csv = '/home/ubuntu/VIO/dataset/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv'
        self.gt_path_msg = self._load_gt_path(gt_csv)
        self._gt_timer = self.create_timer(1.0, self._publish_gt_path)
        
        # Start Threads
        self.frontend_thread = threading.Thread(target=self.frontend_worker, daemon=True)
        self.backend_thread = threading.Thread(target=self.backend_worker, daemon=True)
        self.frontend_thread.start()
        self.backend_thread.start()
        
        self.get_logger().info("VIO System Node initialized (Multi-threaded).")

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
                self.initial_imu_buffer.append(imu_data)
                if len(self.initial_imu_buffer) >= 20: # Use ~20 messages
                    a_avg = np.mean([m.accel for m in self.initial_imu_buffer], axis=0)
                    # Find R_WI that rotates a_avg/norm to [0, 0, 1]
                    z_axis = a_avg / np.linalg.norm(a_avg)
                    target_z = np.array([0.0, 0.0, 1.0])
                    v = np.cross(z_axis, target_z)
                    s = np.linalg.norm(v)
                    c = np.dot(z_axis, target_z)
                    
                    if s < 1e-6:
                        q_init = np.array([1.0, 0.0, 0.0, 0.0]) if c > 0 else np.array([0.0, 1.0, 0.0, 0.0])
                    else:
                        v_skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                        R = np.eye(3) + v_skew + (v_skew @ v_skew) * ((1 - c) / (s**2))
                        q_init = matrix_to_quaternion(R)
                    
                    with self.state_server.lock:
                        self.state_server.state.quaternion = q_init
                        self.state_server.state.timestamp = self.initial_imu_buffer[-1].timestamp
                        
                    self.gravity_aligned = True
                    self.get_logger().info(f"Gravity Aligned! Initial Pitch/Roll solved. a_avg: {a_avg}")
                    
    def image_callback(self, msg: Image):
        if not self.gravity_aligned:
            return  # Wait for IMU to align first
        if not hasattr(self, '_first_img_rcv'):
            self.get_logger().info("Received FIRST Image topic msg!")
            self._first_img_rcv = True
            
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        try:
            self.image_queue.put_nowait((curr_time, cv_img))
        except queue.Full:
            self.get_logger().warn("Image queue full. Dropped frame.")

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

    def frontend_worker(self):
        """
        Thread 2: Pops images, correlates IMU data, and runs Computer Vision tracking.
        """
        while rclpy.ok():
            try:
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
                    self.imu_buffer = [
                        m for m in self.imu_buffer
                        if m.timestamp > image_time
                    ]

                # 3. IMU Prediction (propagate state forward — backend stub for now).
                if imu_measurements:
                    self.imu_propagator.propagate(imu_measurements)

                # 4. Build current camera pose from propagated state.
                with self.state_server.lock:
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

                # 6. Forward to the backend worker
                try:
                    self.measurement_queue.put_nowait((image_time, mature_features))
                except queue.Full:
                    self.get_logger().warn("Measurement queue full. Dropped features.")

                # 7. Publish visualizer image
                import cv2
                debug_img = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                for track in self.frontend.active_tracks:
                    pt = track.observations[-1]
                    cv2.circle(debug_img, (int(pt[0]), int(pt[1])), 3, (0, 255, 0), -1)
                
                img_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                
                # Convert float timestamp back to ROS Time message
                sec = int(image_time)
                nanosec = int((image_time - sec) * 1e9)
                img_msg.header.stamp = Time(sec=sec, nanosec=nanosec)
                img_msg.header.frame_id = "cam0"
                self.debug_img_pub.publish(img_msg)

                # Publish CameraInfo cho AR Overlay rviz2
                cam_info = CameraInfo()
                cam_info.header = img_msg.header
                cam_info.width = image.shape[1]
                cam_info.height = image.shape[0]
                cam_info.distortion_model = "plumb_bob"
                fx, fy = self.msckf_updater.fx, self.msckf_updater.fy
                cx, cy = self.msckf_updater.cx, self.msckf_updater.cy
                cam_info.k = [float(fx), 0.0, float(cx), 0.0, float(fy), float(cy), 0.0, 0.0, 1.0]
                cam_info.p = [float(fx), 0.0, float(cx), 0.0, 0.0, float(fy), float(cy), 0.0, 0.0, 0.0, 1.0, 0.0]
                self.cam_info_pub.publish(cam_info)

                self.last_image_time = image_time
            except Exception as e:
                self.get_logger().error(f"Frontend Worker Crashed: {e}")

    def backend_worker(self):
        """
        Thread 3: MSCKF core, running EKF state updates and publishing.
        """
        while rclpy.ok():
            try:
                try:
                    # Pop mature_features from self.measurement_queue
                    image_time, mature_features = self.measurement_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # --- Phase 4: MSCKF Update ---
                with self.state_server.lock:
                    # 1. State Augmentation (clone state)
                    state = self.state_server.state
                    self.state_server.add_clone(image_time, state.position, state.quaternion)
                    
                    # 2. Measurement Update
                    self.msckf_updater.process_mature_features(mature_features)

                # Extract 3D points for visualization
                new_points_count = 0
                for f in mature_features:
                    p = self.msckf_updater.triangulate_feature(f)
                    if p is not None:
                        self.global_map_points.append(p)
                        new_points_count += 1
                        
                # Keep map bound
                if len(self.global_map_points) > 100000:
                    self.global_map_points = self.global_map_points[-100000:]

                # --- Phase 5: Publish Result ---
                self.publish_state(image_time)
                self.get_logger().info(f"Successfully tracked and published frame at ts={image_time:.3f} (New Points: {new_points_count})")
            except Exception as e:
                self.get_logger().error(f"Backend Worker Crashed: {e}")

    def publish_state(self, timestamp: float):
        """
        Publish the current state (Odometry), Historical Path, and Camera Frustum.
        """
        with self.state_server.lock:
            state = self.state_server.state
            
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
        t_cam.transform.translation.x = 0.0
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = 0.0
        t_cam.transform.rotation.w = 1.0 # identity
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
    rclpy.shutdown()

if __name__ == '__main__':
    main()
