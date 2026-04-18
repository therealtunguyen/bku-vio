from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Provide the path to the Euroc bag
    dataset_path = '/home/ubuntu/dataset/V1_01_easy/'

    return LaunchDescription([
        # 1. Start the rosbag player with the --clock flag to publish /clock
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', dataset_path, '--clock'],
            output='screen'
        ),
        
        # 2. Start the VIO node with use_sim_time=True
        Node(
            package='vio_pkg',
            executable='vio_system_node',
            name='vio_system_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 3. Start RViz with use_sim_time=True
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
