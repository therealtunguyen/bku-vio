from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    dataset_path = "/home/ubuntu/VIO/dataset/V1_01_easy_ros2bag/"
    rviz_config = os.path.join(
        get_package_share_directory("vio_pkg"), "config", "vio.rviz"
    )

    return LaunchDescription(
        [
            # 1. Rosbag player
            ExecuteProcess(
                cmd=["ros2", "bag", "play", dataset_path, "--clock"], output="screen"
            ),
            # 2. VIO node
            Node(
                package="vio_pkg",
                executable="vio_system_node",
                name="vio_system_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            # 3. RViz with pre-configured displays
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=["-d", rviz_config],
            ),
        ]
    )
