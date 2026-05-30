from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    bag_rate_arg = DeclareLaunchArgument(
        "bag_rate",
        default_value="0.2",
        description="Playback rate for ros2 bag play during VIO debugging.",
    )
    imu_init_sample_count_arg = DeclareLaunchArgument(
        "imu_init_sample_count",
        default_value="200",
        description="Number of startup IMU samples to average for gravity/bias initialization.",
    )

    dataset_dir = LaunchConfiguration("dataset_dir")
    bag_rate = LaunchConfiguration("bag_rate")
    imu_init_sample_count = LaunchConfiguration("imu_init_sample_count")

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
            bag_rate_arg,
            imu_init_sample_count_arg,
            # 1. Rosbag player
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", bag_rate],
                output="screen",
            ),
            # 2. VIO node
            Node(
                package="vio_pkg",
                executable="vio_system_node",
                name="vio_system_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "gt_csv_path": gt_csv_path,
                        "imu_init_sample_count": ParameterValue(
                            imu_init_sample_count,
                            value_type=int,
                        ),
                    }
                ],
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
