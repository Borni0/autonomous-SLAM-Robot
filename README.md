# rover_ws — Autonomous Indoor Navigation Rover

A single workspace containing everything you need to build and run the
GPS-free indoor rover:

* **Pi 5 / ROS 2 Jazzy software** under `src/` (SLAM, Nav2, AMCL, RViz,
  ESP32 bridge, soft e-stop, compass).
* **ESP32-S3 firmware** under `firmware/` (PlatformIO, BTS7960 drivers,
  wheel encoders, MPU6050 IMU, VL53L0X ToF, OLED, battery, e-stop).
* **Docs, udev rules, scripts, and Makefile** at the workspace root.

```
rover_ws/
├── src/                     # ROS 2 packages (colcon build)
├── firmware/                # ESP32-S3 PlatformIO project
├── docs/                    # design doc, Pi 5 hardware notes, udev rules
├── scripts/                 # CI / sanity helpers
├── maps/                    # saved SLAM maps land here
├── Makefile                 # make build / firmware-upload / launch-mapping / ...
├── BUILDING.md              # full Pi 5 install + colcon recipe
├── HARDWARE_PI5.md          # (in docs/) Pi 5 power / storage / cooling
└── .gitignore
```

## Target hardware

* **Raspberry Pi 5** (4 GB or 8 GB) running **Ubuntu 24.04 LTS (Noble)**.
* **ESP32-S3 DevKitC-1** running the firmware in `firmware/`.
* **YDLIDAR A3** 2D LiDAR over USB.
* **3S LiPo (11.1 V) → 5 V / 6 A buck** feeding the Pi 5's USB-C PD.

## Packages (`src/`)

| Package | Type | Purpose |
|---|---|---|
| `rover_description` | ament_cmake | URDF / xacro of the chassis + sensors |
| `rover_lidar` | ament_python | Thin wrapper around `ydlidar_ros2_driver` for the A3 |
| `rover_hardware` | ament_python | ESP32 serial bridge + dead-reckoning odometry + soft e-stop |
| `rover_compass` | ament_python | Compass / magnetometer node (real or synthetic) |
| `rover_localization` | ament_python | robot_localization EKF config + launch |
| `rover_navigation` | ament_python | Nav2 params, AMCL, SLAM Toolbox configs |
| `rover_bringup` | ament_python | Top-level launch orchestration + RViz config |

## Build & run

```bash
# 1. Install ROS 2 Jazzy and PlatformIO — see BUILDING.md
# 2. Build the ESP32 firmware and flash it:
make firmware-build
make firmware-upload             # writes to /dev/ttyUSB0 by default
# 3. Build the ROS 2 workspace:
make install-ros-deps
make build
source install/setup.bash
# 4. Sanity check:
make check
# 5. Launch:
make launch-desc                 # description + RViz
make launch-lidar                # LiDAR + /scan
make launch-odom                 # /odom + teleop (no ESP32)
make launch-motors               # ESP32 bridge + RViz (no odometry)
make launch-mapping              # SLAM Toolbox online_async
make launch-nav                  # AMCL + Nav2 (needs indoor_map.yaml)
```

`make firmware-upload` accepts a port override:

```bash
make firmware-upload FIRMWARE_PORT=/dev/ttyACM0
```

`make firmware-monitor` opens a 115200 baud serial monitor on the
ESP32.

## udev device naming

After `make install-udev`:

* `/dev/rover_esp32` — ESP32-S3 (USB CDC)
* `/dev/rover_a3` — YDLIDAR A3

These are the defaults in `src/rover_lidar/config/ydlidar.yaml` and
`src/rover_hardware/config/diff_drive_params.yaml`.

## Three launch modes

```bash
ros2 launch rover_bringup rover.launch.py mode:=just_description
ros2 launch rover_bringup rover.launch.py mode:=mapping
ros2 launch rover_bringup rover.launch.py mode:=navigation \
    world:=/home/$USER/rover_ws/maps/indoor_map.yaml
ros2 launch rover_bringup rover.launch.py mode:=both
    # ESP32 bridge + RViz only; no SLAM, no Nav2. Useful for bench-
    # testing motors and encoders against live hardware.
```

Add `use_compass:=true` or `use_ekf:=true` to optionally bring up
the compass or the EKF.

## /cmd_vel chain (Nav2 -> ESP32)

`ros2 launch rover_bringup rover.launch.py mode:=navigation` wires
Nav2's output all the way to the wheels:

```
Nav2 controller_server
    |
    v
velocity_smoother (publishes /cmd_vel_nav)
    |
    v
soft_estop       (subscribes /cmd_vel_in, publishes /cmd_vel_out)
    |
    v
esp32_bridge     (subscribes /cmd_vel, sends M L=... R=... to ESP32)
    |
    v
BTS7960 motors
```

The chain is connected in `src/rover_bringup/launch/rover.launch.py`
via two launch-time remappings: Nav2's smoother output is remapped
onto `/cmd_vel_in`, and the ESP32 bridge subscribes `/cmd_vel_out`.

To halt the rover from the keyboard:

```bash
ros2 run rover_hardware estop_cli stop       # freeze (zero /cmd_vel_out)
ros2 run rover_hardware estop_cli release    # resume
```

## Soft e-stop

```bash
ros2 run rover_hardware estop_cli stop       # freeze
ros2 run rover_hardware estop_cli release    # resume
```

## CI / pre-build sanity

```bash
./scripts/ci_check.sh          # py_compile + unit tests + YAML
./scripts/bringup_check.sh     # post-colcon-build verification
```

## Indoor mapping

See [`docs/MAPPING.md`](docs/MAPPING.md) for the step-by-step procedure
to generate a 2D occupancy grid (`indoor_map.pgm`/`indoor_map.yaml`)
that AMCL/Nav2 can localise against.
