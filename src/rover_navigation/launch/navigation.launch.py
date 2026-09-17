"""Full Nav2 bringup: map_server + AMCL + planner/controller/BT + costmaps.

Note: ``nav2_bringup`` itself brings up ``lifecycle_manager_localization``
(amcl + map_server) and ``lifecycle_manager_navigation`` (planner,
controller, behavior, bt_navigator, smoother, costmaps). We DO NOT
add a duplicate lifecycle manager here — that would race the official
one and amcl would never configure.

The cmd_vel chain is:

    controller_server -> /cmd_vel_nav
      -> velocity_smoother -> /cmd_vel_in
        -> soft_estop -> /cmd_vel
          -> esp32_bridge

so ``velocity_smoother.smoothed_cmd_vel_topic`` must be ``cmd_vel_in``
(already set in nav2_params.yaml).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitute
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rover_navigation')
    nav2_bringup = FindPackageShare('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    nav_params = PathJoinSubstitute([pkg, 'config', 'nav2_params.yaml'])
    amcl_params = PathJoinSubstitute([pkg, 'config', 'amcl.yaml'])

    # ---- map_server ----
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ---- amcl ----
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[amcl_params],
        output='screen',
    )

    # ---- nav2 bringup (planner, controller, behaviors, bt, costmaps,
    # smoother, lifecycle managers, amcl/map_server too in newer versions).
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([nav2_bringup, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params,
            'map': map_yaml,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                os.path.expanduser('~'),
                'rover_ws', 'maps', 'indoor_map.yaml',
            ),
        ),
        map_server,
        amcl,
        nav2_bringup_launch,
    ])
