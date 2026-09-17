"""Launch the ESP32 bridge + dead-reckoning odometry together."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg_file = PathJoinSubstitute([
        FindPackageShare('rover_hardware'),
        'config', 'diff_drive_params.yaml',
    ])

    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')

    bridge = Node(
        package='rover_hardware',
        executable='esp32_bridge',
        name='esp32_bridge',
        parameters=[cfg_file, {
            'serial_port': serial_port,
            'serial_baud': serial_baud,
        }],
        output='screen',
    )

    odom = Node(
        package='rover_hardware',
        executable='diff_drive_odometry',
        name='diff_drive_odometry',
        parameters=[cfg_file],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rover_esp32'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        bridge,
        odom,
    ])
