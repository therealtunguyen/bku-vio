import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import TransformStamped

import numpy as np


class BagReaderNode(Node):
    def __init__(self):
        super().__init__('bag_reader_node')

        # ── Counters ──
        self._imu_count = 0
        self._img_count = 0
        self._vicon_count = 0

        # ── Subscribe IMU (/imu0) ──
        self.create_subscription(
            Imu,
            '/imu0',
            self._imu_callback,
            10,
        )
        self.get_logger().info('Subscribed to /imu0')

        # ── Subscribe Camera 0 (/cam0/image_raw) ──
        self.create_subscription(
            Image,
            '/cam0/image_raw',
            self._cam0_callback,
            10,
        )
        self.get_logger().info('📸 Subscribed to /cam0/image_raw')

        # ── Subscribe Vicon ground-truth ──
        self.create_subscription(
            TransformStamped,
            '/vicon/firefly_sbx/firefly_sbx',
            self._vicon_callback,
            10,
        )
        self.get_logger().info('🎯 Subscribed to /vicon/firefly_sbx/firefly_sbx')

        self.get_logger().info('BagReaderNode đã sẵn sàng! Hãy play rosbag.')

    def _imu_callback(self, msg: Imu):
        """Xử lý dữ liệu IMU."""
        self._imu_count += 1
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        if self._imu_count % 500 == 0:  # In mỗi 500 messages
            self.get_logger().info(
                f'[IMU #{self._imu_count}] '
                f'acc=({acc.x:.3f}, {acc.y:.3f}, {acc.z:.3f}) '
                f'gyro=({gyro.x:.4f}, {gyro.y:.4f}, {gyro.z:.4f})'
            )

    def _cam0_callback(self, msg: Image):
        """Xử lý dữ liệu ảnh từ Camera 0."""
        self._img_count += 1
        if self._img_count % 50 == 0:  # In mỗi 50 ảnh
            self.get_logger().info(
                f'[CAM0 #{self._img_count}] '
                f'size={msg.width}x{msg.height} '
                f'encoding={msg.encoding} '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
            )

    def _vicon_callback(self, msg: TransformStamped):
        """Xử lý dữ liệu ground-truth từ Vicon."""
        self._vicon_count += 1
        t = msg.transform.translation
        r = msg.transform.rotation
        if self._vicon_count % 500 == 0:  # In mỗi 500 messages
            self.get_logger().info(
                f'[VICON #{self._vicon_count}] '
                f'pos=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) '
                f'quat=({r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BagReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Kết thúc! '
            f'IMU: {node._imu_count}, '
            f'CAM0: {node._img_count}, '
            f'VICON: {node._vicon_count}'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
