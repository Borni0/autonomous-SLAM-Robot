"""Launch the compass node."""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitute([
        FindPackageShare('rover_compass'),
        'config', 'compass.yaml',
    ])

    node = Node(
        package='rover_compass',
        executable='compass_node',
        name='compass_node',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([node])
