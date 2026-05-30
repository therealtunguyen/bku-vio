from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="/home/ubuntu/VIO/dataset/vio_hcmut_dataset",
        description="Absolute path to the HCMUT D455 rosbag directory.",
    )
    bag_rate_arg = DeclareLaunchArgument(
        "bag_rate",
        default_value="0.5",
        description="Playback rate for the HCMUT D455 bag.",
    )
    imu_init_sample_count_arg = DeclareLaunchArgument(
        "imu_init_sample_count",
        default_value="120",
        description="Number of startup IMU samples for gravity/bias initialization.",
    )
    stop_after_processed_frames_arg = DeclareLaunchArgument(
        "stop_after_processed_frames",
        default_value="0",
        description="Stop automatically after processing N frames; 0 disables the limit.",
    )

    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    imu_init_sample_count = LaunchConfiguration("imu_init_sample_count")
    stop_after_processed_frames = LaunchConfiguration("stop_after_processed_frames")

    return LaunchDescription(
        [
            bag_path_arg,
            bag_rate_arg,
            imu_init_sample_count_arg,
            stop_after_processed_frames_arg,
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--clock",
                    "--rate",
                    bag_rate,
                    "--remap",
                    "/camera/camera/color/image_raw:=/cam0/image_raw",
                    "/camera/camera/imu:=/imu0",
                ],
                output="screen",
            ),
            Node(
                package="vio_pkg",
                executable="vio_system_node",
                name="vio_system_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "imu_init_sample_count": ParameterValue(
                            imu_init_sample_count,
                            value_type=int,
                        ),
                        "input_qos_reliability": "best_effort",
                        "image_processing_width": 752,
                        "image_queue_size": 50,
                        "runtime_diagnostics_enabled": True,
                        "diagnostics_log_every_n_frames": 10,
                        "publish_debug_image": False,
                        "log_tracked_frames": False,
                        "max_imu_dt": 0.05,
                        # Deterministic replay shows frame-200 needs a small
                        # bias-rail lift; 0.06 is the smallest value that
                        # consistently accepts that batch.
                        "max_batch_dx_bias_norm": 0.06,
                        # Late-window HCMUT drift is dominated by weak-geometry
                        # tracks; 2.0 deg is the smallest parallax floor that
                        # materially reduces the deterministic late peak.
                        "min_triangulation_parallax_deg": 2.0,
                        "max_imu_init_gap": 0.2,
                        "max_frame_timestamp_gap": 0.25,
                        # This bag contains several real multi-second image
                        # holes. Resetting temporal state on those forward
                        # gaps adds churn without improving replay stability.
                        "reset_on_large_frame_gap": False,
                        "stop_after_processed_frames": ParameterValue(
                            stop_after_processed_frames,
                            value_type=int,
                        ),
                        "camera_fx": 646.33728,
                        "camera_fy": 645.676147,
                        "camera_cx": 643.358276,
                        "camera_cy": 362.999176,
                        "camera_distortion": [
                            -0.05594548583030701,
                            0.06458555161952972,
                            -0.0002526374883018434,
                            0.0008183500613085926,
                            -0.021141313016414642,
                        ],
                        # camera_imu_optical_frame -> camera_color_optical_frame
                        "camera_extrinsics_convention": "camera_in_imu",
                        "camera_R_IC": [
                            0.999996654005,
                            0.00159873842,
                            -0.002033719299,
                            -0.001599047141,
                            0.999998710246,
                            -0.000150184164,
                            0.002033476571,
                            0.000153435674,
                            0.999997920713,
                        ],
                        "camera_t_IC": [
                            0.028793809935,
                            0.007352355558,
                            0.015779949041,
                        ],
                    }
                ],
            ),
        ]
    )
