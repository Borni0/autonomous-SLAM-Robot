"""Bring up description + LiDAR + RViz; no ESP32 / odometry."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rover_bringup')

    desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_description'), 'launch', 'description.launch.py'])
        ),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_lidar'), 'launch', 'a3.launch.py'])
        ),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitute([pkg, 'rviz', 'rover.rviz'])],
    )

    return LaunchDescription([desc, lidar, rviz])
