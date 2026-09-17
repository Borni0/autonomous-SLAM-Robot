"""Standalone launch file for just the robot description."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = LaunchConfiguration(
        'urdf_path',
        default=PathJoinSubstitute([
            FindPackageShare('rover_description'),
            'urdf', 'rover.urdf.xacro',
        ]),
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_path,
    ])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # jsp is harmless even if no inputs; we use robot_state_publisher for fixed joints.
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Publish all zero joint angles so the wheels stay attached
            # to the chassis in RViz until real encoder feedback is wired.
            'publish_default_positions': True,
            'default_joint_states': [
                {'name': 'wheel_left_joint',  'position': 0.0},
                {'name': 'wheel_right_joint', 'position': 0.0},
            ],
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=PathJoinSubstitute([
                FindPackageShare('rover_description'),
                'urdf', 'rover.urdf.xacro',
            ]),
        ),
        rsp,
        jsp,
    ])
