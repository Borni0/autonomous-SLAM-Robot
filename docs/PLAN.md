# ROS 2 Jazzy Workspace Plan — Autonomous Indoor Rover (Pi side)

> Version: 1.0 — 2026-09-17
> Target ROS 2 distro: Jazzy Jalisco (Ubuntu 24.04 Noble on the Pi)
> Hardware already built: ESP32-S3 + BTS7960 + encoders + MPU6050 + VL53L0X
> ESP32 firmware lives in: `~/AutonomousNavigationRobot/` (PlatformIO, Arduino)
> This workspace lives in: `~/rover_ws/src/`
> LiDAR assumed: A3 (YDLIDAR), driver `ydlidar_ros2_driver`

---

## Context

### Why
The ESP32 firmware (`~/AutonomousNavigationRobot/`) accepts motor commands over USB serial at 115200 baud but **does not stream encoder ticks back**. Therefore the ROS 2 side cannot subscribe to a wheel-odometry topic; it must derive odometry by integrating `/cmd_vel` through a differential-drive kinematic model using the same `WHEEL_BASE_M` and `WHEEL_RADIUS_M` constants the ESP32 firmware uses (`include/config.h`: `WHEEL_RADIUS_M = 0.032f`, `WHEEL_BASE_M = 0.150f`). Eventually a real encoder-based odometry source will replace the dead-reckoning node; the seam is the same `nav_msgs/Odometry` topic.

### What
A complete ROS 2 Jazzy workspace tree — six ament packages — ready for `colcon build --symlink-install` on a Raspberry Pi (or in a Docker container) where ROS 2 Jazzy has been installed. No ROS 2 commands will be executed on this Ubuntu 22.04 machine; **only files will be created.**

### Who-it's-for
A developer who wants to drive the rover in three modes:
1. `mode:=just_description` — see the URDF in RViz, nothing else.
2. `mode:=mapping` — teleop + SLAM Toolbox builds a map.
3. `mode:=navigation` — load saved map + AMCL + Nav2 + RViz with a goal tool.

---

## Architecture

### TF Tree

```
map
 └── odom                       (provided by AMCL when running)
      └── base_link             (provided by diff_drive_odometry node)
           ├── lidar_link       (static, from xacro)
           ├── compass_link     (static, from xacro; placeholder for MPU6050 mount)
           ├── wheel_left_link  (static, from xacro)
           └── wheel_right_link (static, from xacro)
```

| Edge | Producer | Source |
|---|---|---|
| `base_link → lidar_link` | `robot_state_publisher` | xacro `<joint name="lidar_joint" type="fixed">` |
| `base_link → compass_link` | `robot_state_publisher` | xacro |
| `base_link → wheel_left_link` | `robot_state_publisher` | xacro |
| `base_link → wheel_right_link` | `robot_state_publisher` | xacro |
| `odom → base_link` | `diff_drive_odometry` | dead-reckoning from `/cmd_vel` |
| `map → odom` | AMCL | (only `mode:=navigation`) |

### Node Graph

```
                            +----------------------+
                            |   ESP32 (ttyUSB0)    |
                            +----------+-----------+
                                       | 115200, "M L=40 R=40\n"
                                       v
+-------------------+        +---------+----------+        +---------------------------+
|  teleop_twist_joy |------> | esp32_bridge node  | -----> | diff_drive_odometry node |
|  (or keyboard)    | Twist  | (rover_hardware)   | Twist  |  (rover_hardware)         |
+-------------------+        +---------+----------+        +----+-----------+----------+
                                       ^                            | Odometry + TF
                                       | logs                       v
                              (separate read thread)        +-------+-------+
                                                              | robot_state_  |
                                                              |   publisher   |
                                                              +-------+-------+
                                                                      | URDF
                                                                      v
                                                              /tf_static + /tf
                                      +--------------+    +---------+-----------+
                                      | ydlidar node |--> | /scan (LaserScan)   |
                                      +--------------+    +---------------------+
                                                                      |
                                                                      v
                                                          +-----------+----------+
                                                          | EKF (robot_local)   |
                                                          |  /odom (relay)      |
                                                          +-----------+----------+
                                                                      |
                                              SLAM Toolbox / AMCL <----+
                                              Nav2 (planner + ctrl)

mode:=mapping:   slam_toolbox online_async  ← /scan + /tf + /odom
mode:=navigation: map_server -> AMCL -> Nav2 bringup
```

### ESP32 serial protocol (mirror of `~/AutonomousNavigationRobot/src/command.cpp`)

* **Baud:** 115200 (`SERIAL_BAUD` in `config.h`).
* **Line discipline:** ESP32 accepts LF-terminated lines (`\n`). CR (`\r`) is silently dropped. Lines are space-tokenised with `=` for key/value pairs.
* **Recognised commands (case-insensitive verb):**

  | Verb | Args | Effect | Use here |
  |---|---|---|---|
  | `AUTO` | – | robot enters AUTO mode (does its own obstacle avoidance using ToF) | optional, e.g. on startup |
  | `STOP` | – | immediate stop, MANUAL mode | watchdog |
  | `M` | `L=<int8> R=<int8>` | direct PWM percent in [-100, 100] | **only one we send from the Pi** |
  | `CALIB` | – | starts IMU gyro calibration | not used by Pi |
  | `RESET` | – | resets heading + queue | not used by Pi |
  | `GOTO` | `X=<float> Y=<float>` | queue a waypoint (ESP32 does its own nav) | not used by Pi |
  | `WPLS` / `WPCLR` | – | list/clear waypoints | not used by Pi |

* **Invalid lines** get `[ANR] ? Unknown command` echoed back.
* **No inbound telemetry.** Encoder ticks, IMU, ToF all stay on-device; we only get debug lines like `[ANR] MANUAL L=40 R=40` or `[ANR] FAULT: ...` for logging.
* **Throttle:** do not push `M L=… R=…` faster than ~20 Hz (matches `CONTROL_LOOP_MS = 20`).

### `Twist` → `M L=… R=…` conversion (inverse diff-drive kinematics)

```
v_left  = v − ω · WHEEL_BASE / 2
v_right = v + ω · WHEEL_BASE / 2
v_max   = MOTOR_MAX_LINEAR_MPS   (default 0.30 m/s — matches ESP32's
                                  percentToLinearMps(100) = 0.30 mapping)
pct_left  = clamp(round( 100 · v_left  / v_max ), −100, 100)
pct_right = clamp(round( 100 · v_right / v_max ), −100, 100)
line = f"M L={pct_left} R={pct_right}\n"
```

Constants mirrored in `rover_hardware/config/diff_drive_params.yaml`. **Critical:** `wheel_base` and `wheel_radius` here MUST equal the ESP32 `WHEEL_BASE_M = 0.150` and `WHEEL_RADIUS_M = 0.032` in `include/config.h`. The dead-reckoning node uses the same numbers for odometry.

---

## Directory Tree

Everything below is created relative to `~/rover_ws/`. Only paths beginning at `src/` go into the workspace; `docs/`, `maps/`, `README.md` and `BUILDING.md` are siblings and are not built by colcon.

```
rover_ws/
├── README.md                                    (workspace overview — pointer to this plan)
├── BUILDING.md                                  (install-on-Pi instructions, command list)
├── docs/
│   └── PLAN.md                                  (this file)
├── maps/                                        (saved maps, .pgm + .yaml)
│   └── .gitkeep
└── src/
    ├── rover_description/                       (ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── urdf/
    │   │   ├── rover.urdf.xacro                 (top-level — chassis + includes joints)
    │   │   ├── rover_base.xacro                 (base_link macro)
    │   │   ├── wheel.xacro                     (wheel_left_link / wheel_right_link)
    │   │   ├── lidar.xacro                      (lidar_link)
    │   │   ├── compass.xacro                    (compass_link)
    │   │   └── materials.xacro                  (RViz colors)
    │   ├── meshes/                              (empty placeholders for future CAD)
    │   │   └── .gitkeep
    │   └── launch/
    │       └── description.launch.py            (just robot_state_publisher)
    │
    ├── rover_bringup/                           (ament_python)
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── package.xml
    │   ├── resource/
    │   │   └── rover_bringup                    (ament marker)
    │   ├── rover_bringup/
    │   │   └── __init__.py
    │   ├── launch/
    │   │   ├── rover.launch.py                  (top-level; mode:= arg)
    │   │   ├── test_motors.launch.py
    │   │   ├── test_lidar.launch.py
    │   │   └── test_odom.launch.py
    │   ├── rviz/
    │   │   └── rover.rviz                       (minimal RViz config)
    │   └── config/
    │       └── bringup_params.yaml              (shared namespace defaults)
    │
    ├── rover_hardware/                          (ament_python)
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── package.xml
    │   ├── resource/
    │   │   └── rover_hardware                   (ament marker)
    │   ├── rover_hardware/
    │   │   ├── __init__.py
    │   │   ├── esp32_bridge.py                  (serial writer + read thread)
    │   │   └── diff_drive_odometry.py           (Twist -> Odometry + TF)
    │   ├── config/
    │   │   └── diff_drive_params.yaml
    │   └── test/
    │       └── test_kinematics.py               (sanity test for inverse kinematics)
    │
    ├── rover_lidar/                             (ament_python)
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── package.xml
    │   ├── resource/
    │   │   └── rover_lidar                      (ament marker)
    │   ├── rover_lidar/
    │   │   └── __init__.py
    │   ├── config/
    │   │   └── ydlidar.yaml
    │   └── launch/
    │       └── a3.launch.py                     (wraps ydlidar_ros2_driver_node)
    │
    ├── rover_navigation/                        (ament_python)
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── package.xml
    │   ├── resource/
    │   │   └── rover_navigation
    │   ├── rover_navigation/
    │   │   └── __init__.py
    │   ├── launch/
    │   │   ├── navigation.launch.py             (map_server + AMCL + Nav2)
    │   │   └── mapping.launch.py                (slam_toolbox online_async)
    │   ├── config/
    │   │   ├── nav2_params.yaml                 (controller, planner, BT, costmaps)
    │   │   ├── amcl.yaml
    │   │   └── slam_toolbox.yaml
    │   └── maps/
    │       └── .gitkeep                         (saved-map area; user drops maps here)
    │
    └── rover_localization/                      (ament_python)
        ├── setup.py
        ├── setup.cfg
        ├── package.xml
        ├── resource/
        │   └── rover_localization
        ├── rover_localization/
        │   └── __init__.py
        ├── launch/
        │   └── ekf.launch.py
        └── config/
            └── ekf.yaml
```

Total files (excluding `.gitkeep` and `__pycache__/`):

* 5 ament_python top-level packages × (~5 boilerplate) = ~25
* 1 ament_cmake package × ~9 = ~9
* Config YAML: 5 (`bringup_params`, `diff_drive_params`, `ydlidar`, `nav2_params`, `amcl`, `slam_toolbox`, `ekf`) = 7
* Launch files: 4 in `rover_bringup` + 2 in `rover_lidar`/`rover_navigation` (mapping, navigation) + 1 in `rover_description` + 1 in `rover_localization` = 8
* URDF/Xacro: 6 files
* RViz: 1
* Python modules: 3 (`esp32_bridge`, `diff_drive_odometry`, plus package `__init__.py`s)
* Tests: 1
* Docs: 3 (`README.md`, `BUILDING.md`, this `PLAN.md`)

---

## Per-file Responsibilities

### Top-level workspace files

#### `rover_ws/README.md`
Pointer to `docs/PLAN.md` and `BUILDING.md`. Three sentences: what the workspace is for, who maintains the ESP32 firmware, which packages exist.

#### `rover_ws/BUILDING.md`
A copy/paste-ready install procedure (no execution here) for the Pi:

1. Install Ubuntu 24.04 (Noble) or use the official OSRF Debian packages.
2. Install ROS 2 Jazzy: `sudo apt install ros-jazzy-desktop ros-jazzy-robot-localization ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-amcl ros-jazzy-ydlidar-ros2-driver`.
3. Install udev rule for the ESP32 so it appears as `/dev/rover` (symlink to `/dev/ttyUSB*`).
4. `sudo rosdep init && rosdep update`.
5. `cd ~/rover_ws && sudo rosdep install -y --from-paths src --ignore-src --rosdistro=jazzy`.
6. `source /opt/ros/jazzy/setup.bash && colcon build --symlink-install`.
7. `source install/setup.bash`.

#### `rover_ws/maps/.gitkeep`
Empty so the directory exists. Users save SLAM output here.

---

### Package: `rover_description` (ament_cmake)

#### `CMakeLists.txt`
Standard `ament_cmake` boilerplate; installs `urdf/`, `launch/`, `meshes/`; uses `xacro` to generate URDF; declares no libraries. Roughly:

```cmake
cmake_minimum_required(VERSION 3.16)
project(rover_description LANGUAGES)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf launch meshes DESTINATION share/${PROJECT_NAME})
ament_package()
```

#### `package.xml`
`format=3`, `<name>rover_description</name>`, `<buildtool_depend>ament_cmake</buildtool_depend>`, `<exec_depend>xacro</exec_depend>`, `<exec_depend>robot_state_publisher</exec_depend>`, `<exec_depend>joint_state_publisher</exec_depend>`, `<maintainer email=…>`, `<license>Apache-2.0</license>`.

#### `urdf/rover.urdf.xacro`
Top-level. Includes the macros and defines the chassis (a single `base_link` box with inertial block), then includes `<xacro:wheel_left>`, `<xacro:wheel_right>`, `<xacro:lidar_mount>`, `<xacro:compass_mount>`. Roots the robot at a fixed origin on the ground.

```
<robot name="rover" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find rover_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find rover_description)/urdf/rover_base.xacro"/>
  <xacro:include filename="$(find rover_description)/urdf/wheel.xacro"/>
  <xacro:include filename="$(find rover_description)/urdf/lidar.xacro"/>
  <xacro:include filename="$(find rover_description)/urdf/compass.xacro"/>

  <xacro:rover_base/>

  <!-- wheels -->
  <xacro:wheel side="left"  x="0" y="${wheel_base/2}" z="${wheel_radius}"/>
  <xacro:wheel side="right" x="0" y="${-wheel_base/2}" z="${wheel_radius}"/>

  <!-- lidar on top -->
  <xacro:lidar_mount x="0" y="0" z="0.20"/>

  <!-- compass on top deck -->
  <xacro:compass_mount x="0.05" y="0" z="0.05"/>
</robot>
```

Properties (declared via `<xacro:property>`): `wheel_base := 0.15` (metres), `wheel_radius := 0.032`, `chassis_length := 0.30`, `chassis_width := 0.22`, `chassis_height := 0.10`.

#### `urdf/rover_base.xacro`
Defines `<link name="base_link">` with a visual box (the deck), a collision box, and an inertial block (mass 5 kg; naive diagonal inertia `0.1 0.1 0.1`; origin at centre of mass). Exposes a `<xacro:rover_base/>` macro.

#### `urdf/wheel.xacro`
`<xacro:wheel side="${side}" x="${x}" y="${y}" z="${z}"/>` → creates either `wheel_left_link` or `wheel_right_link` (cylinder of radius `${wheel_radius}`, length 0.04, mass 0.2 kg) plus a fixed joint from `base_link`. Spinning joints (continuous) named `wheel_left_joint` / `wheel_right_joint` stay present so `joint_state_publisher` can publish `wheel_left_controller/...` style velocities even though they are no-ops for diff-drive TF. The joint axes matter: x-axis along the wheel's spin, NOT along the robot's forward direction — this is a common bug.

#### `urdf/lidar.xacro`
`<xacro:lidar_mount x="${x}" y="${y}" z="${z}"/>` → creates `lidar_link` (cylinder r=0.035, h=0.04) plus a fixed joint named `lidar_joint`. Designed so the ydlidar node's `frame_id: "lidar_link"` lines up.

#### `urdf/compass.xacro`
`<xacro:compass_mount x="${x}" y="${y}" z="${z}"/>` → creates `compass_link` (small box) + `compass_joint`. Placeholder for the MPU6050 mount point; the IMU is *not* actually streamed so this is visual only.

#### `urdf/materials.xacro`
Defines a few `<material name="…">` entries (`grey`, `dark_grey`, `red`, `blue`, `green`) with RViz colours.

#### `meshes/.gitkeep`
Empty.

#### `launch/description.launch.py`
Single argument `urdf_path` defaulting to `$(find rover_description)/urdf/rover.urdf.xacro`. Calls `xacro.process_file(urdf_path).toxml()` via the `xacro` Python module, then spawns `robot_state_publisher` with parameters `robot_description`, `use_sim_time`. Used by the top-level launch and standalone.

---

### Package: `rover_bringup` (ament_python)

#### `setup.py`
```python
from setuptools import setup
package_name = 'rover_bringup'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch',
                 ['launch/rover.launch.py',
                  'launch/test_motors.launch.py',
                  'launch/test_lidar.launch.py',
                  'launch/test_odom.launch.py']),
                ('share/' + package_name + '/rviz',
                 ['rviz/rover.rviz']),
                ('share/' + package_name + '/config',
                 ['config/bringup_params.yaml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='…', maintainer_email='…',
    description='Top-level launch orchestration for the rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_launch = rover_bringup.launch_helper:main',  # optional convenience CLI
        ],
    },
)
```
No console scripts strictly needed; launches are reached via `ros2 launch rover_bringup rover.launch.py`. The `rover_launch` helper is optional and can be omitted.

#### `setup.cfg`
Standard `[develop] install_dirs=.[develop] script_dir=$base/lib/rover_bringup` and `[install] install_scripts=$base/lib/rover_bringup`.

#### `package.xml`
`format=3`, depends on `rover_description`, `rover_lidar`, `rover_hardware`, `rover_localization`, `rover_navigation`, `robot_state_publisher`, `xacro`, `nav2_bringup`, `slam_toolbox`, `rviz2`. `exec_depend` for runtime nodes, `exec_depend` (or `depend`) for launches.

#### `resource/rover_bringup`
Empty file; ament index marker.

#### `rover_bringup/__init__.py`
Empty `__all__ = []` is fine; presence alone makes this a Python package.

#### `config/bringup_params.yaml`
Top-level defaults used by multiple launches:

```yaml
rover:
  namespace: ""            # ROS_DOMAIN_ID-like prefix, empty by default
  base_frame: base_link
  odom_frame: odom
  map_frame: map
  cmd_vel_topic: cmd_vel
  odom_topic: odom

esp32:
  serial_port: /dev/ttyUSB0    # overridden via launch arg
  baud: 115200
  cmd_timeout_ms: 500

lidar:
  frame_id: lidar_link
  topic: scan

ekf:
  use_ekf: false               # flip to true once IMU is wired

nav:
  map_yaml: /home/<user>/rover_ws/maps/indoor_map.yaml
  params_file: $(find rover_navigation)/config/nav2_params.yaml
  rviz_config: $(find rover_bringup)/rviz/rover.rviz
```

#### `rviz/rover.rviz`
Minimal hand-authored RViz config (NOT a hex blob — RViz2 reads JSON-style YAML). Includes displays:

* `TF` (enabled, show axes, show names)
* `RobotModel` (robot_description)
* `LaserScan` on topic `/scan` (`lidar_link`)
* `Map` (if `mode:=navigation`, on `/map`)
* `Path`
* `Odometry` on `/odom`
* `PoseArray` placeholder for AMCL particle cloud

Set Camera to follow `base_link`. Set Fixed Frame to `odom` (so things work whether `map` exists or not — switching to `map` is a one-click in RViz).

#### `launch/rover.launch.py`  *(the main file — most important)*

```python
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
    OpaqueFunction, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, Command, FindExecutable, PathJoinSubstitute
)
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # ---- args ----
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='just_description',
        choices=['just_description','mapping','navigation'])
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    serial_baud_arg = DeclareLaunchArgument('serial_baud', default_value='115200')
    use_ekf_arg    = DeclareLaunchArgument('use_ekf', default_value='false')
    world_arg      = DeclareLaunchArgument('world', default_value='$(find rover_navigation)/maps/empty.yaml')
    rviz_arg       = DeclareLaunchArgument('rviz', default_value='true')

    mode        = LaunchConfiguration('mode')
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    use_ekf     = LaunchConfiguration('use_ekf')
    world       = LaunchConfiguration('world')
    rviz        = LaunchConfiguration('rviz')

    pkg_bringup = FindPackageShare('rover_bringup').find('rover_bringup')

    # ---- 1. URDF + robot_state_publisher ----
    robot_description = Command([
        PathJoinSubstitute([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitute([FindPackageShare('rover_description'), 'urdf', 'rover.urdf.xacro']),
    ])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
        output='screen')

    # ---- 2. LiDAR ----
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_lidar'), 'launch', 'a3.launch.py'])))

    # ---- 3. ESP32 bridge + diff-drive odometry ----
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_hardware'), 'launch', 'hardware.launch.py'])),
        launch_arguments={
            'serial_port': serial_port,
            'serial_baud': serial_baud}.items())

    # ---- 4. EKF (optional) ----
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_localization'), 'launch', 'ekf.launch.py'])),
        condition=IfCondition(use_ekf))

    # ---- 5. mode-specific ----
    mapping_launch = IfCondition(mode == 'mapping'):   # pseudocode; use OpaqueFunction in real
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitute([FindPackageShare('rover_navigation'), 'launch', 'mapping.launch.py'])),
            condition=IfCondition(IfCondition(mode == 'mapping')))

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitute([FindPackageShare('rover_navigation'), 'launch', 'navigation.launch.py'])),
        launch_arguments={'map': world}.items(),
        condition=IfCondition(...))   # IfCondition is built from a Substitution comparison

    # ---- 6. RViz ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitute([pkg_bringup, 'rviz', 'rover.rviz'])],
        condition=IfCondition(rviz))

    return LaunchDescription([
        mode_arg, serial_port_arg, serial_baud_arg, use_ekf_arg, world_arg, rviz_arg,
        rsp, lidar_launch, hardware_launch, ekf_launch,
        mapping_launch, navigation_launch, rviz_node,
    ])
```

> **Implementation note:** `IfCondition` cannot take a Python equality check. The real file uses **two separate `OpaqueFunction`** strategies or, more idiomatically, a Python helper that checks `LaunchConfiguration('mode')` strings and conditionally appends actions. The canonical pattern is:

```python
from launch.substitutions import LaunchConfiguration
def _modes_from(context, mode): ...  # not idiomatic; use IfCondition with EqualsSubstitution
```

Actually the cleanest way in Jazzy is:

```python
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, NotEqualsSubstitution, LaunchConfiguration
# building condition objects statically is allowed:
WhenMapping = IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'mapping'))
```

For `mode:=just_description` neither mapping nor navigation launches are launched; only rsp + lidar + hardware + (optional) ekf + (optional) rviz.

#### `launch/test_motors.launch.py`
Loads just `rover_description` + `esp32_bridge` + `diff_drive_odometry` (no LiDAR, no EKF, no Nav2). Useful for testing wheels when the lidar is unplugged. Includes a `teleop_twist_keyboard` node with `prefix` set so the user gets a clear instruction printout.

#### `launch/test_lidar.launch.py`
Loads `rover_description` + `rover_lidar/a3.launch.py` + RViz with `/scan` visible. No hardware. Confirms the A3 driver comes up.

#### `launch/test_odom.launch.py`
Loads `rover_description` + `esp32_bridge` + `diff_drive_odometry` + a `teleop_twist_keyboard`. RViz shows `/odom` and `TF`. After driving the rover by hand (or by pushing the physical robot), the user verifies `base_link` moves as expected and the `M L=… R=…` lines appear in `ros2 topic echo /cmd_vel`.

---

### Package: `rover_hardware` (ament_python)

This package owns the **only two custom nodes** in the system.

#### `setup.py`
```python
from setuptools import setup
package_name='rover_hardware'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages',
                   ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch',
                   ['launch/hardware.launch.py']),
                  ('share/' + package_name + '/config',
                   ['config/diff_drive_params.yaml'])],
      install_requires=['setuptools', 'pyserial'],
      zip_safe=True,
      maintainer='…', maintainer_email='…',
      description='ESP32 serial bridge + dead-reckoning odometry',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'esp32_bridge        = rover_hardware.esp32_bridge:main',
          'diff_drive_odometry = rover_hardware.diff_drive_odometry:main',
      ]})
```

This is what makes `ros2 run rover_hardware esp32_bridge` work.

#### `setup.cfg`
Same pattern as above (`script_dir=$base/lib/rover_hardware`).

#### `package.xml`
`format=3`, `<depend>rclpy</depend>`, `<depend>geometry_msgs</depend>`, `<depend>nav_msgs</depend>`, `<depend>tf2_ros</depend>`, `<depend>serial</depend>` (PIP-installed via `setup.py install_requires`), `<exec_depend>xacro</exec_depend>` (no), `<exec_depend>tf2_geometry_msgs</exec_depend>`.

#### `resource/rover_hardware`
Empty marker.

#### `rover_hardware/__init__.py`
Empty / `__all__ = []`.

#### `rover_hardware/esp32_bridge.py`
Responsibilities:

1. Declare parameters (with `self.declare_parameter`):
   * `serial_port` (string, default `/dev/ttyUSB0`)
   * `serial_baud` (int,    default `115200`)
   * `wheel_base`   (double, default `0.150`) — MUST match ESP32 `WHEEL_BASE_M`
   * `wheel_radius` (double, default `0.032`) — MUST match ESP32 `WHEEL_RADIUS_M`
   * `v_max_mps`    (double, default `0.30`)  — matches `percentToLinearMps(100) = 0.30`
   * `cmd_timeout_ms` (int, default `500`)
   * `publish_rate_hz` (double, default `20.0`) — clamps how fast we push fresh `M` lines
   * `frame_id`     (string, default `base_link`) — for the `/diagnostics` header
2. Open the serial port via `serial.Serial(port, baud, timeout=0.05)` in `__init__`, log at INFO.
3. Subscribe `cmd_vel` with `self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)`.
4. `on_twist(msg)`:
   * convert → `(v_L_mps, v_R_mps)` → `(pct_L, pct_R)` and store in `self._last_cmd`,
   * record `self._last_cmd_time = self.get_clock().now()`,
   * if (now − last_send) > 1/publish_rate_hz, **immediately** push the line via `_send_motor(pct_L, pct_R)`.
5. Use a `ros2 timer` (period = `1/publish_rate_hz`) that:
   * re-sends the latest `M L=… R=…` if no fresh twist arrived (so the ESP32 doesn't time out its `REMOTE_TIMEOUT_MS=3000`),
   * if `(now − _last_cmd_time).nanoseconds / 1e6 > cmd_timeout_ms`, send `STOP\n` and log a WARNING ("cmd_vel watchdog tripped"),
   * reads any pending bytes from the serial port (passing them to `_log_serial_line`).
6. `_send_motor(left, right)` serialises `M L={} R={}\n`.utf-8 bytes via the `serial.Serial` write, with a `try/except serial.SerialException` that logs WARN and pauses for 1 s on disconnect (so the user can replug USB).
7. `_log_serial_line(line)` strips `\r`, calls `self.get_logger().info('[esp32] ' + line)`.
8. A small `_reader_thread()` runs in a Python `threading.Thread(daemon=True)` that loops `while rclpy.ok(): line = ser.readline().decode('utf-8', errors='replace'); if line: ...`. The timer alternates with that thread, which is safer than mixing blocking reads with the timer callback.
9. On `on_shutdown`: `self._send_motor(0, 0)` then `self._ser.close()`.
10. `main()` constructs `rclpy.init()`, instantiates the node, `rclpy.spin(node)`, finishes with `rclpy.shutdown()`.

Stops are explicit: cancellation of `rclpy.spin()` triggers `on_shutdown` via the registered shutdown callback (use `node.destroy_node()` first).

**Inverse kinematics snippet:**

```python
v = msg.linear.x     # m/s
w = msg.angular.z    # rad/s
half = params['wheel_base'] / 2.0
v_L = v - w * half
v_R = v + w * half
max_v = params['v_max_mps']
pct_L = max(-100, min(100, int(round(100.0 * v_L / max_v))))
pct_R = max(-100, min(100, int(round(100.0 * v_R / max_v))))
```

#### `rover_hardware/diff_drive_odometry.py`
Responsibilities:

1. Subscribe `/cmd_vel`. Maintain `_last_twist_time` and `_last_twist`.
2. Publish `/odom` (`nav_msgs/Odometry`) and `odom → base_link` TF (`tf2_ros.TransformBroadcaster`).
3. Parameters: `wheel_base`, `wheel_radius` (defaults 0.150 / 0.032), `publish_tf` (bool, true), `odom_frame` (`odom`), `base_frame` (`base_link`), `publish_rate_hz` (50), `cmd_timeout_ms` (500).
4. `update_pose(dt)` integration uses **exact arc** (matches the ESP32-side `Odometry::update`):
   ```python
   dL_m = vL_mps * dt
   dR_m = vR_mps * dt
   ds   = 0.5 * (dR_m + dL_m)
   dTh  = (dR_m - dL_m) / wheel_base
   if abs(dTh) < 1e-6:
       x += ds * cos(th); y += ds * sin(th)
   else:
       r_ = ds / dTh
       x += r_ * (sin(th + dTh) - sin(th))
       y += -r_ * (cos(th + dTh) - cos(th))
       th += dTh
   th = (th + math.pi) % (2*math.pi) - math.pi
   ```
   (The same exact-arithmetic integration is in `~/AutonomousNavigationRobot/src/odometry.cpp` lines 28-37.)
5. If no `/cmd_vel` has arrived for `cmd_timeout_ms`, set `_last_twist = (0, 0)` — integration continues to drive the robot's reported pose to a halt even though no command is being sent (no jump when the watchdog sends `STOP`).
6. `odom_msg.header.stamp = self.get_clock().now().to_msg()`; populate `pose.pose.position.{x,y,z}`, `pose.pose.orientation` (a `tf_transformations.quaternion_from_euler(0,0,th)`), and `twist.twist.{linear,angular}`.
7. Covariances: `pose.covariance` diagonal entries
   `[0.05, 0.05, 0.0, 0.0, 0.0, 0.05]` (rough indoor dead-reckoning numbers — tune later)
   and `twist.covariance` diagonal `[0.10, 0.10, 0.0, 0.0, 0.0, 0.25]`.
   `frame_id = "odom"`, `child_frame_id = "base_link"`.
8. TF message: `translation` = `(x, y, 0)`, `rotation` = quaternion from euler, `header.frame_id = "odom"`, `child_frame_id = "base_link"`.

#### `config/diff_drive_params.yaml`
```yaml
esp32_bridge:
  ros__parameters:
    serial_port: /dev/ttyUSB0
    serial_baud: 115200
    wheel_base: 0.150     # must equal ESP32 WHEEL_BASE_M in config.h
    wheel_radius: 0.032   # must equal ESP32 WHEEL_RADIUS_M in config.h
    v_max_mps: 0.30
    cmd_timeout_ms: 500
    publish_rate_hz: 20.0

diff_drive_odometry:
  ros__parameters:
    wheel_base: 0.150
    wheel_radius: 0.032
    publish_tf: true
    odom_frame: odom
    base_frame: base_link
    cmd_timeout_ms: 500
    publish_rate_hz: 50.0
```

Values for `wheel_base` and `wheel_radius` must be duplicated in three places: ESP32 `config.h`, the URDF (`rover.urdf.xacro` `<xacro:property>` defaults), and this YAML. Flagged as **critical** below.

#### `launch/hardware.launch.py`
Tiny launcher the top-level file can include:

```python
def generate_launch_description():
    cfg = PathJoinSubstitute([FindPackageShare('rover_hardware'), 'config', 'diff_drive_params.yaml'])
    args = [DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('serial_baud', default_value='115200')]
    bridge = Node(package='rover_hardware', executable='esp32_bridge',
                  parameters=[cfg,
                              {'serial_port': LaunchConfiguration('serial_port'),
                               'serial_baud': LaunchConfiguration('serial_baud')}],
                  output='screen')
    odom = Node(package='rover_hardware', executable='diff_drive_odometry',
                parameters=[cfg], output='screen')
    return LaunchDescription(args + [bridge, odom])
```

#### `test/test_kinematics.py`
`pytest` sanity test that:
* converts `(v=0.3, w=0.0)` → `(pct_L=100, pct_R=100)`,
* `(v=0.0, w=1.0)` with wheel_base=0.150 → `(pct_L=-25, pct_R=25)` (because `1 rad/s × 0.075 m = 0.075 m/s`, `0.075/0.30*100 = 25`),
* ensures `M` lines have the exact expected byte format `b"M L=40 R=-10\n"`.

Imports the kinematics function via direct import (refactored out of `esp32_bridge.py` into a `_kinematics.py` helper if needed for testability).

---

### Package: `rover_lidar` (ament_python)

#### `setup.py`
Ament Python boilerplate, `data_files` for `config/`, `launch/`. No console_scripts needed.

#### `package.xml`
`format=3`, `<exec_depend>ydlidar_ros2_driver</exec_depend>`, `<exec_depend>tf2_ros</exec_depend>`.

#### `config/ydlidar.yaml`
```yaml
/**:
  ros__parameters:
    # A3 default; many parameters are exposed via the driver
    port: /dev/ttyUSB1            # NOTE: ESP32 is ttyUSB0, LiDAR is ttyUSB1 normally
    frame_id: lidar_link
    angle_min: -3.14159265
    angle_max:  3.14159265
    range_min:  0.12               # A3 min
    range_max:  8.0                # A3 max in-door spec; bump to 16.0 if outdoors
    scan_frequency: 10
    baudrate: 115200
    lidar_baudrate: 115200
    ignore_array: ""
    resolution_type: 1             # 1 = 1 deg; some A3 firmwares use 0
    inverted: false
    auto_reconnect: true
    intensity: false
    sample_rate: 5
```

> **Important:** the ESP32 occupies `/dev/ttyUSB0` by default; the LiDAR must be set to `/dev/ttyUSB1` (or use a udev rule so it appears as `/dev/ydlidar`). See udev section in `BUILDING.md`.

#### `launch/a3.launch.py`
```python
return LaunchDescription([
    Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[PathJoinSubstitute([FindPackageShare('rover_lidar'),
                                        'config', 'ydlidar.yaml'])],
        output='screen'),
    # static TF — redundant with URDF but useful when running test_lidar.launch.py
    # without robot_state_publisher.
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_link',
                   '--child-frame-id', 'lidar_link',
                   '--x', '0', '--y', '0', '--z', '0.20'],
        output='screen'),
])
```

> When `rover_state_publisher` is also running, this static transform is redundant but harmless (the dynamic TF from joint_state_publisher is `static_publisher`-style under the hood).

---

### Package: `rover_navigation` (ament_python)

#### `setup.py`
Standard; data_files for `config/`, `launch/`, `maps/`. No console_scripts.

#### `package.xml`
`format=3`. Depends on `nav2_bringup`, `nav2_map_server`, `amcl`, `slam_toolbox`, `teleop_twist_keyboard` (optional), `rviz2`.

#### `config/nav2_params.yaml`
Single file for the whole Nav2 stack. Most parameters below are the Jazzy defaults, with values adjusted for an indoor wheeled rover whose footprint is ~30×22 cm.

```yaml
amcl:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    map_frame_id: "map"
    use_sim_time: false
    laser_min_range: 0.12         # A3 minimum
    laser_max_range: 8.0          # A3 maximum indoor
    min_particles: 500
    max_particles: 3000
    kld_err: 0.05
    kld_z: 0.99
    update_min_d: 0.10            # metres of travel before update
    update_min_a: 0.10            # radians of travel before update
    respawn_interval: 1.0
    initial_pose_x: 0.0
    initial_pose_y: 0.0
    initial_pose_a: 0.0
    sigma_x: 0.05
    sigma_y: 0.05
    sigma_a: 0.05
    laser_z_short: 0.05
    laser_model_type: "likelihood_field"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.5

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    default_bt_xml_filename: "nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - "nav2_compute_path_to_pose_action_bt_node"
      - "nav2_follow_path_action_bt_node"
      - "nav2_spin_action_bt_node"
      - "nav2_wait_action_bt_node"
      - "nav2_clear_costmap_service_bt_node"
      - "nav2_is_stuck_condition_bt_node"
      - "nav2_goal_reached_condition_bt_node"
      - "nav2_initial_pose_received_condition_bt_node"

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.05
    min_theta_velocity_threshold: 0.05
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::ProgressChecker"
      required_movement_radius: 0.10
      movement_time_allowance: 20.0
    goal_checker:
      plugin: "nav2_controller::GoalChecker"
      xy_goal_tolerance: 0.20
      yaw_goal_tolerance: 0.10
      stateful: true
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.30
      min_vel_x: -0.10
      max_vel_theta: 1.5
      min_vel_theta: -1.5
      vx_samples: 20
      vtheta_samples: 40
      sim_period: 0.05
      debug_trajectory_details: false
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalDist", "PathAlign", "PathDist", "Twirling"]
      Oscillation:
        plugin: "dwb_critics::Oscillation"
      ObstacleFootprint:
        plugin: "dwb_critics::ObstacleFootprint"
        scale: 0.5
      GoalDist:
        plugin: "dwb_critics::GoalDist"
        scale: 3.0
      PathAlign:
        plugin: "dwb_critics::PathAlign"
        scale: 0.5
      PathDist:
        plugin: "dwb_critics::PathDist"
        scale: 0.5
      Twirling:
        plugin: "dwb_critics::Twirling"
        scale: 2.0

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.20
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/footprint"
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.10
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_time: 5.0
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
      scale_x: 0.5
      scale_theta: 0.5

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: "map"
      robot_base_frame: "base_link"
      update_frequency: 1.0
      publish_frequency: 1.0
      resolution: 0.05
      track_unknown_space: true
      footprint: [[0.15, 0.11], [0.15, -0.11], [-0.15, -0.11], [-0.15, 0.11]]
      footprint_padding: 0.02
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "/map"
        enabled: true
        subscribe_to_updates: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: ["laser_scan_sensor"]
        laser_scan_sensor:
          topic: "/scan"
          sensor_frame: "lidar_link"
          data_type: "LaserScan"
          raytrace_range: 3.0
          obstacle_range: 3.0
          expected_update_rate: 0.3
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        cost_scaling_factor: 3.0
        inflation_radius: 0.30

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: "odom"
      robot_base_frame: "base_link"
      update_frequency: 5.0
      publish_frequency: 5.0
      resolution: 0.05
      width: 3
      height: 3
      rolling_window: true
      track_unknown_space: true
      footprint: [[0.15, 0.11], [0.15, -0.11], [-0.15, -0.11], [-0.15, 0.11]]
      footprint_padding: 0.02
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: ["laser_scan_sensor"]
        laser_scan_sensor:
          topic: "/scan"
          sensor_frame: "lidar_link"
          data_type: "LaserScan"
          raytrace_range: 3.0
          obstacle_range: 3.0
          expected_update_rate: 0.3
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        cost_scaling_factor: 3.0
        inflation_radius: 0.20

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    max_velocity: [0.30, 0.0, 1.5]
    min_velocity: [-0.10, 0.0, -1.5]
    max_acceleration: [0.50, 0.0, 3.0]
    max_deceleration: [1.0, 0.0, 3.0]
    odom_topic: "/odom"
    odom_duration: 0.1
    cmd_vel_topic: "cmd_vel_nav"
    smoothed_cmd_vel_topic: "cmd_vel"
```

The `velocity_smoother` republishes `/cmd_vel_nav` → `/cmd_vel` so our `esp32_bridge` only listens on smoothed velocities.

#### `config/amcl.yaml`
Already inlined above. We split it into a separate file for users who want to launch AMCL on its own:

```yaml
amcl:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    map_frame_id: "map"
    use_sim_time: false
    laser_min_range: 0.12
    laser_max_range: 8.0
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

#### `config/slam_toolbox.yaml`
For `mode:=mapping`:

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    resolution: 0.05
    map_start_pose: [0.0, 0.0, 0.0]
    map_update_interval: 2.0
    publish_tf: true          # in mapping mode it IS the odom→map producer
    transform_publishing_period: 0.05
    mode: "mapping"
    mapper_planner_frequency: 5.0
    scan_topic: /scan
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    # ... rest are the standard slam_toolbox online_async defaults
    do_loopclosing: true
    loop_search_maximum_distance: 3.0
    minimum_time_interval: 0.5
    minimum_trajectory_length: 0.3
    # ...
```

#### `launch/mapping.launch.py`
```python
return LaunchDescription([
    Node(package='slam_toolbox',
         executable='async_slam_toolbox_node',
         name='slam_toolbox',
         parameters=[PathJoinSubstitute([FindPackageShare('rover_navigation'),
                                         'config', 'slam_toolbox.yaml'])],
         output='screen'),
    Node(package='teleop_twist_keyboard',
         executable='teleop_twist_keyboard',
         name='teleop',
         prefix=['xterm -e'],
         output='screen'),
])
```

#### `launch/navigation.launch.py`
```python
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map')
    nav_params = PathJoinSubstitute([FindPackageShare('rover_navigation'),
                                     'config', 'nav2_params.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=...),
        # map_server
        Node(package='nav2_map_server',
             executable='map_server',
             name='map_server',
             parameters=[{'yaml_filename': map_yaml,
                          'use_sim_time': use_sim_time}]),
        # AMCL
        Node(package='nav2_amcl',
             executable='amcl',
             name='amcl',
             parameters=[PathJoinSubstitute([FindPackageShare('rover_navigation'),
                                             'config', 'amcl.yaml'])]),
        # Lifecycle manager for the three
        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'autostart': True,
                           'node_names': ['map_server', 'amcl']}]),
        # Nav2 bringup (includes controller_server, planner_server, bt_navigator,
        # behavior_server, global_costmap, local_costmap, velocity_smoother,
        # lifecycle_manager_navigation)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitute([FindPackageShare('nav2_bringup'),
                                    'launch', 'bringup_launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': nav_params,
                              'map': map_yaml}.items()),
    ])
```

#### `maps/.gitkeep`
Empty.

---

### Package: `rover_localization` (ament_python)

#### `setup.py`
Standard; data_files for `config/`, `launch/`. No console scripts.

#### `package.xml`
`format=3`, `<exec_depend>robot_localization</exec_depend>`.

#### `config/ekf.yaml`
Configured to take over `/odom` once enabled. Until the IMU is wired the only input is the dead-reckoning odometry — so we set this to "relay" mode (just republish), which means the EKF is largely a placeholder.

```yaml
ekf_filter_node:
  ros__parameters:
    use_sim_time: false
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    publish_acceleration: false
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   false, false, false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false

    # IMU disabled — there is no /imu_data published yet. Re-enable once
    # we get real telemetry.
    imu0: /imu_data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, false]
    imu0_differential: false
    imu0_relative: false

    process_noise_covariance: [
      0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0.04, 0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.02, 0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.015]
```

#### `launch/ekf.launch.py`
```python
return LaunchDescription([
    DeclareLaunchArgument('use_sim_time', default_value='false'),
    Node(package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         parameters=[PathJoinSubstitute([FindPackageShare('rover_localization'),
                                         'config', 'ekf.yaml']),
                     {'use_sim_time': LaunchConfiguration('use_sim_time')}],
         output='screen'),
])
```

`robot_localization` *does* take over `/odom` once launched — it consumes the dead-reckoning topic and republishes; downstream nodes should listen on `/odometry/filtered` instead. Switch by setting the `use_ekf` launch arg of `rover.launch.py` to `true`.

---

## Build & Verification

These are the commands the developer runs **on the Pi**, after installing ROS 2 Jazzy. They are not executed on this Ubuntu 22.04 box.

### 1. Install OS / ROS 2

```bash
# Ubuntu 24.04 Noble. Use OSRF deb packages.
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y ros-jazzy-desktop \
  ros-jazzy-robot-localization ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-amcl \
  ros-jazzy-ydlidar-ros2-driver ros-jazpy-xacro \
  python3-colcon-common-extensions python3-pip
pip3 install pyserial
sudo rosdep init && rosdep update
```

### 2. udev rules (so the ESP32 is /dev/rover, LiDAR is /dev/ydlidar)

```bash
# /etc/udev/rules.d/99-rover.rules
# ESP32-S3 USB CDC — by VID:PID 3036:4004 (typical)
SUBSYSTEM=="tty", ATTRS{idVendor}=="3036", ATTRS{idProduct}=="4004",
  SYMLINK+="rover", MODE="0666"
# YDLIDAR
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",
  SYMLINK+="ydlidar", MODE="0666"
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. Clone / copy this workspace

```bash
cd ~
# Replace with the actual git URL or scp path used in production.
git clone <repo> rover_ws       # or: scp -r borni@workstation:rover_ws .
cd ~/rover_ws
```

### 4. Install dependencies

```bash
cd ~/rover_ws
sudo apt install -y python3-rosdep
sudo rosdep install -y --from-paths src --ignore-src --rosdistro=jazzy
```

### 5. Build

```bash
cd ~/rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  rover_description rover_lidar rover_hardware \
  rover_localization rover_navigation rover_bringup
source install/setup.bash
```

### 6. Smoke tests (each one builds confidence before going further)

#### 6.1 Description only

```bash
ros2 launch rover_bringup rover.launch.py mode:=just_description rviz:=true
```

Expected: RViz opens, you see `base_link`, `lidar_link`, both wheels, `compass_link` and a few floating TF arrows. `/tf_static` is published. No errors in terminal.

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link  # should print a fixed transform
```

#### 6.2 Test LiDAR

```bash
# Plug in YDLIDAR on /dev/ttyUSB1 (or wherever udev mapped it)
ros2 launch rover_bringup test_lidar.launch.py
```

Expected: laser scan dots appear in RViz around `lidar_link`, rotating. Driver logs "YDLIDAR running at…" within a few seconds. If `ange_min/max` look wrong, edit `rover_lidar/config/ydlidar.yaml`.

#### 6.3 Test odometry + motors

```bash
# ESP32 must be plugged in, e.g. /dev/ttyUSB0 or /dev/rover
sudo chmod 666 /dev/ttyUSB0     # or rely on udev
ros2 launch rover_bringup test_odom.launch.py
```

In the second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Expected:
* Wheels respond (you can hear/feel the BTS7960 buzz).
* `ros2 topic echo /odom` shows the pose updating.
* `ros2 run tf2_ros tf2_echo odom base_link` shows the transform changing as you drive.
* `ros2 topic echo /esp32_bridge/_parameter_events` (sub to parameter events) or just watch `tail -f`: you see `M L=… R=…` lines in the terminal.

#### 6.4 Build a map

```bash
ros2 launch rover_bringup rover.launch.py mode:=mapping serial_port:=/dev/ttyUSB0 rviz:=true
```

Drive around (teleop keyboard from another shell, or use `teleop_twist_keyboard`). When satisfied:

```bash
# slam_toolbox publishes /map on the wire; capture it once:
ros2 run nav2_map_server map_saver_cli -f ~/rover_ws/maps/indoor_map
# outputs: indoor_map.pgm + indoor_map.yaml
```

#### 6.5 Navigate

```bash
ros2 launch rover_bringup rover.launch.py mode:=navigation serial_port:=/dev/ttyUSB0 \
  world:=/home/<user>/rover_ws/maps/indoor_map.yaml
```

In RViz use "2D Pose Estimate" to set initial pose, then "Nav2 Goal" to set a target. Nav2 should drive, plan, and announce "Goal reached".

### 7. Inspect a node's lifecycle

```bash
ros2 lifecycle get /amcl
ros2 lifecycle set /amcl configure   # or use lifecycle_manager auto via launch
ros2 run tf2_tools view_frames       # outputs a PDF of the TF tree
```

---

## Known Caveats

1. **Wheel-base and wheel-radius are duplicated in three files:**
   * ESP32 firmware `~/AutonomousNavigationRobot/include/config.h`
     (`WHEEL_BASE_M = 0.150f`, `WHEEL_RADIUS_M = 0.032f`)
   * URDF `~/rover_ws/src/rover_description/urdf/rover.urdf.xacro`
     (`<xacro:property name="wheel_base" value="0.15"/>` etc.)
   * `~/rover_ws/src/rover_hardware/config/diff_drive_params.yaml`
     (`wheel_base: 0.150`, `wheel_radius: 0.032`)
   A linter / doc test should grep these and fail on mismatch. **Critical** because differing values here will produce an odom drift proportional to `wheel_base²`.

2. **No encoder feedback.** Until the ESP32 firmware is extended to stream ticks (e.g. `T L=1234 R=1234\n`), `/odom` is dead-reckoned from `/cmd_vel`. Two consequences:
   * Wheels commanded ≠ wheels achieved → drift grows without bound (~5 % after a few metres of mixed carpet/tile).
   * When `mode:=mapping` is the only mode used, drift accumulates into the saved map. Improve by adding `cmd_vel` low-pass filtering on the rover side, or by extending the firmware with encoder streaming.

3. **AMCL requires `/scan` AND a working `/tf`.** Until `diff_drive_odometry` publishes its first `odom → base_link` transform, AMCL will emit `Waiting for transform from /odom to /base_link`. This is harmless — Nav2 will simply sit idle. Once `test_odom.launch.py` confirms TF flows, AMCL will start.

4. **The `map → odom` transform is produced by AMCL in navigation mode and by SLAM Toolbox in mapping mode** — not by us. We must not also broadcast it; the EKF is configured with `world_frame: odom` and `publish_tf: true` so it produces `odom → base_link` only (which is correct for the dead-reckoning node’s pose).

5. **Serial-port race.** ESP32 boots and asserts 115200; the Pi must wait ~2 s after opening the port before sending the first `M` line. The bridge sleeps 1 s after `serial.Serial.open()` and logs `[esp32] serial open: /dev/ttyUSB0@115200`.

6. **LiDAR on `/dev/ttyUSB1` collides with ESP32 on `/dev/ttyUSB0`.** Plug order on cold boot determines the enumeration. The udev rules in `BUILDING.md` make this reproducible; without them, hand-edit the `serial_port` launch arg.

7. **`use_sim_time: false` everywhere.** This is a real robot, not a sim. Setting it to `true` would freeze everything until a `/clock` topic appears.

8. **ESP32 reads `AUTO` mode by default** (see `command.cpp` line 18-19: `_mode(ControlMode::AUTO)` in the constructor). After our first `M L=… R=…` line it switches to MANUAL. To return it to AUTO sending `AUTO` is enough. Most users won't care.

9. **ESP32 `REMOTE_TIMEOUT_MS = 3000` in `config.h`.** If we stop sending `M` lines for >3 s the ESP32 times out and stops. Our 500 ms `cmd_timeout_ms` watchdog in the bridge is the *outer* watchdog (faster than the firmware's, and it sends `STOP`), and the 20 Hz republish handles refreshing the firmware's manual command.

10. **`teleop_twist_keyboard` is intentionally not auto-started in `rover.launch.py`.** Users run it in a separate terminal so Ctrl-C on the teleop keyboard doesn't kill the rest of the bringup. The mapping launch does include it for convenience.

11. **rosdep source.** Our `package.xml`s declare deps (`pyserial`, `xacro`) that exist on PyPI but not as apt packages. `pip install pyserial` is the cleanest path; documented in `BUILDING.md`.

12. **xacro generation at runtime.** The robot_state_publisher is given `robot_description: Command(['xacro', …])` so the URDF always reflects the latest xacro changes without rebuilding. Works because we used `--symlink-install`. If a developer edits xacro and `ros2 launch` reads cached URDF, `ros2 launch …  --show-args` and re-source.

13. **No Nav2 recovery behaviours are tuned.** The `Spin`, `BackUp`, `Wait` BT nodes are vanilla. Tune `behavior_server` once we have logs from real sessions.

14. **Roof-mounted IMU is not live.** The MPU6050 stays on the ESP32 I²C bus and feeds `headingFusion` inside the firmware; the ROS side does not see `/imu_data` because the ESP32 does not stream it. So `ekf.yaml` has all `imu0_*_config` entries `false`. When streaming is added, flip these to `true` and re-tune covariance.

---

## File-creation checklist (copy-paste into a task tracker)

- [ ] rover_ws/README.md
- [ ] rover_ws/BUILDING.md
- [ ] rover_ws/docs/PLAN.md  *(this file)*
- [ ] rover_ws/maps/.gitkeep
- [ ] rover_ws/src/rover_description/CMakeLists.txt
- [ ] rover_ws/src/rover_description/package.xml
- [ ] rover_ws/src/rover_description/urdf/materials.xacro
- [ ] rover_ws/src/rover_description/urdf/rover_base.xacro
- [ ] rover_ws/src/rover_description/urdf/wheel.xacro
- [ ] rover_ws/src/rover_description/urdf/lidar.xacro
- [ ] rover_ws/src/rover_description/urdf/compass.xacro
- [ ] rover_ws/src/rover_description/urdf/rover.urdf.xacro
- [ ] rover_ws/src/rover_description/meshes/.gitkeep
- [ ] rover_ws/src/rover_description/launch/description.launch.py
- [ ] rover_ws/src/rover_bringup/setup.py
- [ ] rover_ws/src/rover_bringup/setup.cfg
- [ ] rover_ws/src/rover_bringup/package.xml
- [ ] rover_ws/src/rover_bringup/resource/rover_bringup
- [ ] rover_ws/src/rover_bringup/rover_bringup/__init__.py
- [ ] rover_ws/src/rover_bringup/config/bringup_params.yaml
- [ ] rover_ws/src/rover_bringup/rviz/rover.rviz
- [ ] rover_ws/src/rover_bringup/launch/rover.launch.py
- [ ] rover_ws/src/rover_bringup/launch/test_motors.launch.py
- [ ] rover_ws/src/rover_bringup/launch/test_lidar.launch.py
- [ ] rover_ws/src/rover_bringup/launch/test_odom.launch.py
- [ ] rover_ws/src/rover_hardware/setup.py
- [ ] rover_ws/src/rover_hardware/setup.cfg
- [ ] rover_ws/src/rover_hardware/package.xml
- [ ] rover_ws/src/rover_hardware/resource/rover_hardware
- [ ] rover_ws/src/rover_hardware/rover_hardware/__init__.py
- [ ] rover_ws/src/rover_hardware/rover_hardware/esp32_bridge.py
- [ ] rover_ws/src/rover_hardware/rover_hardware/diff_drive_odometry.py
- [ ] rover_ws/src/rover_hardware/config/diff_drive_params.yaml
- [ ] rover_ws/src/rover_hardware/launch/hardware.launch.py
- [ ] rover_ws/src/rover_hardware/test/test_kinematics.py
- [ ] rover_ws/src/rover_lidar/setup.py
- [ ] rover_ws/src/rover_lidar/setup.cfg
- [ ] rover_ws/src/rover_lidar/package.xml
- [ ] rover_ws/src/rover_lidar/resource/rover_lidar
- [ ] rover_ws/src/rover_lidar/rover_lidar/__init__.py
- [ ] rover_ws/src/rover_lidar/config/ydlidar.yaml
- [ ] rover_ws/src/rover_lidar/launch/a3.launch.py
- [ ] rover_ws/src/rover_navigation/setup.py
- [ ] rover_ws/src/rover_navigation/setup.cfg
- [ ] rover_ws/src/rover_navigation/package.xml
- [ ] rover_ws/src/rover_navigation/resource/rover_navigation
- [ ] rover_ws/src/rover_navigation/rover_navigation/__init__.py
- [ ] rover_ws/src/rover_navigation/config/nav2_params.yaml
- [ ] rover_ws/src/rover_navigation/config/amcl.yaml
- [ ] rover_ws/src/rover_navigation/config/slam_toolbox.yaml
- [ ] rover_ws/src/rover_navigation/launch/mapping.launch.py
- [ ] rover_ws/src/rover_navigation/launch/navigation.launch.py
- [ ] rover_ws/src/rover_navigation/maps/.gitkeep
- [ ] rover_ws/src/rover_localization/setup.py
- [ ] rover_ws/src/rover_localization/setup.cfg
- [ ] rover_ws/src/rover_localization/package.xml
- [ ] rover_ws/src/rover_localization/resource/rover_localization
- [ ] rover_ws/src/rover_localization/rover_localization/__init__.py
- [ ] rover_ws/src/rover_localization/config/ekf.yaml
- [ ] rover_ws/src/rover_localization/launch/ekf.launch.py
