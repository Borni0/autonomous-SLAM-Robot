"""Bring up description + RViz + teleop for visual /odom inspection.

In the Phase 4 firmware the /odom topic is published by
``esp32_bridge`` (it parses the firmware's ``P`` lines), so a "test odom
without the ESP32" launch doesn't really exist any more. This launch
file is kept as a thin description + teleop + RViz check so the user
can confirm the URDF and TF tree without plugging the ESP32 in.

    ros2 topic list | grep odom      # empty until esp32_bridge launches
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

    return LaunchDescription([desc, teleop, rviz])
