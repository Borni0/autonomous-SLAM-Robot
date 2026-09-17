"""SLAM Toolbox online-async launcher + teleop for mapping a new area."""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_params = PathJoinSubstitute([
        FindPackageShare('rover_navigation'),
        'config', 'slam_toolbox.yaml',
    ])

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix=['xterm -e'],
        output='screen',
    )

    return LaunchDescription([slam, teleop])
