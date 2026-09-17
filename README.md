# rover_ws

ROS 2 Jazzy workspace for the autonomous indoor rover.

* **Design / plan:** [`docs/PLAN.md`](docs/PLAN.md) — read this first.
* **Install / build instructions for the Raspberry Pi 5:** [`BUILDING.md`](BUILDING.md).
* **ESP32 firmware (sibling repo):** `~/AutonomousNavigationRobot/` (PlatformIO).

## Target hardware

* **Raspberry Pi 5** (4 GB or 8 GB) running **Ubuntu 24.04 LTS (Noble)**.
* **ESP32-S3 DevKitC-1** running the existing firmware at `~/AutonomousNavigationRobot/`.
* **YDLIDAR A3** 2D LiDAR over USB.
* **3S LiPo (11.1 V) → 5 V / 6 A buck** feeding the Pi 5's USB-C PD input.

## Packages

| Package | Type | Purpose |
|---|---|---|
| `rover_description` | ament_cmake | URDF / xacro of the chassis + sensors |
| `rover_lidar` | ament_python | Thin wrapper around `ydlidar_ros2_driver` for the A3 |
| `rover_hardware` | ament_python | ESP32 serial bridge + dead-reckoning odometry + soft e-stop |
| `rover_compass` | ament_python | Compass / magnetometer node (real or synthetic) |
| `rover_localization` | ament_python | robot_localization EKF config + launch |
| `rover_navigation` | ament_python | Nav2 params, AMCL, SLAM Toolbox configs |
| `rover_bringup` | ament_python | Top-level launch orchestration + RViz config + drive helpers |

## udev device naming

After installing `docs/99-rover-esp32.rules` and
`docs/99-rover-a3-lidar.rules` (see `BUILDING.md` §3):

* `/dev/rover_esp32` — ESP32-S3 (USB CDC)
* `/dev/rover_a3` — YDLIDAR A3

These names are the defaults in `rover_lidar/config/ydlidar.yaml`,
`rover_hardware/config/diff_drive_params.yaml`, and the launch files.

## Three modes

```bash
ros2 launch rover_bringup rover.launch.py mode:=just_description
ros2 launch rover_bringup rover.launch.py mode:=mapping
ros2 launch rover_bringup rover.launch.py mode:=navigation \
    world:=/home/$USER/rover_ws/maps/indoor_map.yaml
```

Add `use_compass:=true` or `use_ekf:=true` to optionally bring up the
compass or the EKF.

## Quick tests before full bringup

```bash
ros2 launch rover_bringup test_lidar.launch.py        # LiDAR alone
ros2 launch rover_bringup test_odom.launch.py         # /odom + teleop
ros2 launch rover_bringup test_motors.launch.py       # ESP32 alone
```

Or use the top-level Makefile:

```bash
make build             # colcon build --symlink-install
make run-unit          # pure-Python unit tests (no ROS needed)
make launch-mapping    # SLAM Toolbox online_async + teleop
make launch-nav        # AMCL + Nav2 (needs ~/rover_ws/maps/indoor_map.yaml)
make launch-desc       # description-only smoke test
make install-udev      # copy docs/*.rules to /etc/udev/rules.d/
```

## Soft e-stop

```bash
ros2 run rover_hardware estop_cli stop       # freeze
ros2 run rover_hardware estop_cli release    # resume
```

## CI / pre-build sanity

```bash
./scripts/ci_check.sh
```

Runs `py_compile` over every Python file, every pure-Python unit test,
and parses every YAML config. Safe to run before `colcon build`.
