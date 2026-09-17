"""Bring up description + odometry + teleop (no ESP32, no LiDAR).

Use this to verify /odom and the odom->base_link TF are produced when
you publish /cmd_vel — without involving the actual motors or wheels.

    ros2 topic echo /odom
    ros2 run tf2_tools view_frames
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

    odom_only = Node(
        package='rover_hardware',
        executable='diff_drive_odometry',
        name='diff_drive_odometry',
        parameters=[PathJoinSubstitute([
            FindPackageShare('rover_hardware'), 'config', 'diff_drive_params.yaml',
        ])],
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix=['xterm -e'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitute([pkg, 'rviz', 'rover.rviz'])],
    )

    return LaunchDescription([desc, odom_only, teleop, rviz])