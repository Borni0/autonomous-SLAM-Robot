"""Launch the robot_localization EKF node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg_file = PathJoinSubstitute([
        FindPackageShare('rover_localization'),
        'config', 'ekf.yaml',
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[cfg_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        ekf,
    ])
