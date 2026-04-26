from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_dataset_dir = "/home/ubuntu/VIO/dataset"

    dataset_dir_arg = DeclareLaunchArgument(
        "dataset_dir",
        default_value=default_dataset_dir,
        description="Absolute path to the dataset root (contains V1_01_easy/)",
    )

    dataset_dir = LaunchConfiguration("dataset_dir")

    bag_path = PathJoinSubstitution([dataset_dir, "V1_01_easy"])
    gt_csv_path = PathJoinSubstitution(
        [
            dataset_dir,
            "V1_01_easy",
            "mav0",
            "state_groundtruth_estimate0",
            "data.csv",
        ]
    )

    rviz_config = os.path.join(
        get_package_share_directory("vio_pkg"), "config", "vio.rviz"
    )

    return LaunchDescription(
        [
            dataset_dir_arg,
            # 1. Rosbag player
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock"], output="screen"
            ),
            # 2. VIO node
            Node(
                package="vio_pkg",
                executable="vio_system_node",
                name="vio_system_node",
                output="screen",
                parameters=[{"use_sim_time": True, "gt_csv_path": gt_csv_path}],
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
