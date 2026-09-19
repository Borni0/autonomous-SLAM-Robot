"""Launch the ESP32 transparent-driver bridge.

The bridge subscribes /cmd_vel by default and sends `M L=… R=…` to the
ESP32. It publishes /odom + odom→base_link, /battery_state, /estop and
/tof/range by parsing the firmware's P and T telemetry records.

The optional `cmd_vel_remap` launch argument lets a parent launch
file redirect the bridge's subscription onto another topic (e.g.
/cmd_vel_out when soft_estop is in front of it). Default empty -> no
remapping.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _make_bridge(context, *args, **kwargs):
    cfg_file = PathJoinSubstitute([
        FindPackageShare('rover_hardware'),
        'config', 'diff_drive_params.yaml',
    ])
    serial_port = LaunchConfiguration('serial_port').perform(context)
    serial_baud = LaunchConfiguration('serial_baud').perform(context)
    cmd_vel_remap = LaunchConfiguration('cmd_vel_remap').perform(context).strip()

    remappings = []
    if cmd_vel_remap:
        remappings.append(('/cmd_vel', cmd_vel_remap))

    node = Node(
        package='rover_hardware',
        executable='esp32_bridge',
        name='esp32_bridge',
        parameters=[cfg_file, {
            'serial_port': serial_port,
            'serial_baud': serial_baud,
        }],
        remappings=remappings,
        output='screen',
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rover_esp32'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('cmd_vel_remap', default_value=''),
        OpaqueFunction(function=_make_bridge),
    ])
