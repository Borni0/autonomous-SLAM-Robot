"""Top-level rover bringup.

Usage:
    ros2 launch rover_bringup rover.launch.py mode:=just_description
    ros2 launch rover_bringup rover.launch.py mode:=mapping
    ros2 launch rover_bringup rover.launch.py mode:=navigation
    ros2 launch rover_bringup rover.launch.py mode:=navigation \
        world:=/home/<user>/rover_ws/maps/indoor_map.yaml
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EqualsSubstitution,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitute,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- arguments ----
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='just_description',
        choices=['just_description', 'mapping', 'navigation'],
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/rover_esp32',
    )
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud', default_value='115200',
    )
    use_ekf_arg = DeclareLaunchArgument('use_ekf', default_value='false')
    use_compass_arg = DeclareLaunchArgument('use_compass', default_value='false')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitute([
            FindPackageShare('rover_navigation'),
            'maps', 'empty.yaml',
        ]),
    )
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    mode = LaunchConfiguration('mode')
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    use_ekf = LaunchConfiguration('use_ekf')
    use_compass = LaunchConfiguration('use_compass')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    pkg_bringup = FindPackageShare('rover_bringup')
    pkg_description = FindPackageShare('rover_description')

    # ---- 1. robot_state_publisher ----
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitute([pkg_description, 'urdf', 'rover.urdf.xacro']),
    ])
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ---- 2. LiDAR ----
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_lidar'), 'launch', 'a3.launch.py'])
        ),
    )

    # ---- 3. ESP32 bridge + odometry ----
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_hardware'), 'launch', 'hardware.launch.py'])
        ),
        launch_arguments={
            'serial_port': serial_port,
            'serial_baud': serial_baud,
        }.items(),
    )

    # ---- 3b. Soft e-stop gate ----
    # Nav2's controller_server publishes /cmd_vel_nav. The velocity
    # smoother publishes on /cmd_vel by default; we point it at
    # /cmd_vel_in instead, the soft e-stop republishes on /cmd_vel_out,
    # and the ESP32 bridge subscribes /cmd_vel. Engage with:
    #   ros2 run rover_hardware estop_cli stop
    soft_estop = Node(
        package='rover_hardware',
        executable='soft_estop',
        name='soft_estop',
        parameters=[{'latched': True}],
        output='screen',
    )

    # ---- 4. EKF (optional) ----
    ekf_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitute([FindPackageShare('rover_localization'), 'launch', 'ekf.launch.py'])
            ),
        ),
    ], condition=IfCondition(use_ekf))

    # ---- 4b. Compass (optional) ----
    compass_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitute([FindPackageShare('rover_compass'), 'launch', 'compass.launch.py'])
            ),
        ),
    ], condition=IfCondition(use_compass))

    # ---- 5. Mapping (only) ----
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_navigation'), 'launch', 'mapping.launch.py'])
        ),
        condition=IfCondition(EqualsSubstitution(mode, 'mapping')),
    )

    # ---- 6. Navigation (only) ----
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_navigation'), 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={'map': world}.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'navigation')),
    )

    # ---- 7. RViz ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitute([pkg_bringup, 'rviz', 'rover.rviz'])],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        mode_arg,
        serial_port_arg,
        serial_baud_arg,
        use_ekf_arg,
        use_compass_arg,
        world_arg,
        rviz_arg,
        rsp,
        lidar_launch,
        hardware_launch,
        soft_estop,
        ekf_launch,
        compass_launch,
        mapping_launch,
        navigation_launch,
        rviz_node,
    ])
