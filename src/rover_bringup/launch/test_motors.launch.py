"""Bring up robot description + ESP32 bridge ONLY (no LiDAR, no Nav2, no
odometry node). Use this when you want to verify the ESP32 receives the
serial commands and the wheels actually turn.

Drive the rover with:

    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.1}, angular: {z: 0.0}}"

(Always lift the wheels off the ground first.)
"""
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

    # ESP32 bridge only — /odom and odom->base_link are now published by
    # the bridge itself, so a separate diff_drive_odometry node is no
    # longer launched.
    bridge = Node(
        package='rover_hardware',
        executable='esp32_bridge',
        name='esp32_bridge',
        parameters=[PathJoinSubstitute([
            FindPackageShare('rover_hardware'), 'config', 'diff_drive_params.yaml',
        ])],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitute([pkg, 'rviz', 'rover.rviz'])],
    )

    return LaunchDescription([desc, bridge, rviz])