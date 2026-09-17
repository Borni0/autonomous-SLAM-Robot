"""Launch the YDLIDAR A3 driver plus a base_link -> lidar_link static TF.

The static TF is redundant with the URDF when robot_state_publisher is also
running; it's included here so this file works standalone (test_lidar).
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitute([
        FindPackageShare('rover_lidar'),
        'config', 'ydlidar.yaml',
    ])

    driver = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[params_file],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_link',
            '--x', '0', '--y', '0', '--z', '0.20',
        ],
        output='screen',
    )

    return LaunchDescription([driver, static_tf])
