"""Top-level rover bringup.

Usage:
    ros2 launch rover_bringup rover.launch.py mode:=just_description
    ros2 launch rover_bringup rover.launch.py mode:=mapping
    ros2 launch rover_bringup rover.launch.py mode:=navigation
    ros2 launch rover_bringup rover.launch.py mode:=navigation \
        world:=/home/<user>/rover_ws/maps/indoor_map.yaml
    ros2 launch rover_bringup rover.launch.py mode:=both
        # ESP32 bridge + RViz, no SLAM/Nav2. Useful for bench-testing
        # motors, encoders, and the soft e-stop against live hardware.

The /cmd_vel chain (Nav2 -> motors):

    Nav2 controller_server
      -> velocity_smoother (publishes /cmd_vel_nav)
      -> soft_estop       (subscribes /cmd_vel_in, publishes /cmd_vel_out)
      -> esp32_bridge     (subscribes /cmd_vel, sends M L=... R=...)

The remappings below connect the loose ends. See
``src/rover_hardware/rover_hardware/soft_estop.py`` for the canonical
wiring diagram.
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
        choices=['just_description', 'mapping', 'navigation', 'both'],
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
    # Gated off in `mapping` so SLAM Toolbox is the sole source of
    # odom->base_link; allowed in `navigation` (AMCL owns map->odom) and
    # `both` (bench-testing the ESP32 without Nav2). cmd_vel_remap wires
    # the bridge onto the soft e-stop's safe output below.
    hardware_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitute([FindPackageShare('rover_hardware'),
                                    'launch', 'hardware.launch.py'])
            ),
            launch_arguments={
                'serial_port': serial_port,
                'serial_baud': serial_baud,
                'cmd_vel_remap': '/cmd_vel_out',
            }.items(),
        ),
    ], condition=IfCondition([
        EqualsSubstitution(mode, 'navigation'),
        EqualsSubstitution(mode, 'both'),
    ]))

    # ---- 3b. Soft e-stop gate ----
    # Nav2's velocity_smoother publishes /cmd_vel_nav (see
    # rover_navigation/config/nav2_params.yaml). We remap the gate's
    # input onto /cmd_vel_nav so the smoother's output flows into the
    # gate, and remap the ESP32 bridge's input onto /cmd_vel_out so the
    # gate's safe output reaches the motors. Engage with:
    #   ros2 run rover_hardware estop_cli stop
    soft_estop = Node(
        package='rover_hardware',
        executable='soft_estop',
        name='soft_estop',
        parameters=[{'latched': True}],
        remappings=[('cmd_vel_in', '/cmd_vel_nav')],
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
