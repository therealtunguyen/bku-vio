from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Whether to launch RViz.",
    )
    runtime_diagnostics_enabled_arg = DeclareLaunchArgument(
        "runtime_diagnostics_enabled",
        default_value="false",
        description="Whether to enable runtime diagnostics logging.",
    )
    diagnostics_log_every_n_frames_arg = DeclareLaunchArgument(
        "diagnostics_log_every_n_frames",
        default_value="10",
        description="Emit diagnostics every N processed frames.",
    )
    publish_debug_image_arg = DeclareLaunchArgument(
        "publish_debug_image",
        default_value="true",
        description="Whether to publish the debug image topic.",
    )
    image_processing_width_arg = DeclareLaunchArgument(
        "image_processing_width",
        default_value="0",
        description=(
            "Resize images to this width before tracking. "
            "0 keeps the original width."
        ),
    )
    log_tracked_frames_arg = DeclareLaunchArgument(
        "log_tracked_frames",
        default_value="true",
        description="Whether to log per-frame tracking summaries.",
    )
    stop_after_processed_frames_arg = DeclareLaunchArgument(
        "stop_after_processed_frames",
        default_value="0",
        description="Stop automatically after processing N frames; 0 disables the limit.",
    )

    dataset_dir = LaunchConfiguration("dataset_dir")
    bag_rate = LaunchConfiguration("bag_rate")
    imu_init_sample_count = LaunchConfiguration("imu_init_sample_count")
    enable_rviz = LaunchConfiguration("enable_rviz")
    runtime_diagnostics_enabled = LaunchConfiguration("runtime_diagnostics_enabled")
    diagnostics_log_every_n_frames = LaunchConfiguration(
        "diagnostics_log_every_n_frames"
    )
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    image_processing_width = LaunchConfiguration("image_processing_width")
    log_tracked_frames = LaunchConfiguration("log_tracked_frames")
    stop_after_processed_frames = LaunchConfiguration("stop_after_processed_frames")

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
            enable_rviz_arg,
            runtime_diagnostics_enabled_arg,
            diagnostics_log_every_n_frames_arg,
            publish_debug_image_arg,
            image_processing_width_arg,
            log_tracked_frames_arg,
            stop_after_processed_frames_arg,
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
                        "runtime_diagnostics_enabled": ParameterValue(
                            runtime_diagnostics_enabled,
                            value_type=bool,
                        ),
                        "diagnostics_log_every_n_frames": ParameterValue(
                            diagnostics_log_every_n_frames,
                            value_type=int,
                        ),
                        "publish_debug_image": ParameterValue(
                            publish_debug_image,
                            value_type=bool,
                        ),
                        "image_processing_width": ParameterValue(
                            image_processing_width,
                            value_type=int,
                        ),
                        "log_tracked_frames": ParameterValue(
                            log_tracked_frames,
                            value_type=bool,
                        ),
                        "stop_after_processed_frames": ParameterValue(
                            stop_after_processed_frames,
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
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
