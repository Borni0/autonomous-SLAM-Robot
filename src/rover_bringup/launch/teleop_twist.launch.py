"""Keyboard teleop helper.

Launches ``teleop_twist_keyboard`` (from ros-jazzy-teleop-twist-keyboard)
configured for the rover. Publishes on ``/cmd_vel``, the topic the ESP32
bridge subscribes.

Run standalone:

    ros2 launch rover_bringup teleop_twist.launch.py

The default keyboard speeds (i, j, k, l etc.) inside
``teleop_twist_keyboard`` are 0.5 m/s linear and 1.0 rad/s angular —
much too fast for an indoor rover. Set them on the running node:

    ros2 param set /teleop_twist_keyboard speed.linear 0.10
    ros2 param set /teleop_twist_keyboard speed.angular 0.30

(``speed`` is a double-array param; ``teleop_twist_keyboard`` reads
its first two elements at the time each key is pressed, so this works
at runtime.)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix=['xterm -e'],
        output='screen',
    )

    return LaunchDescription([teleop])