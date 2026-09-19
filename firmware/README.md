# Autonomous Indoor Navigation Robot — Phase 4 (firmware)

> **This file lives inside the unified workspace at `~/rover_ws/`.**
> The Raspberry Pi side (SLAM, Nav2, RViz) is at `../src/`. The Pi
> talks to this firmware over USB serial at 115200 baud. See the
> top-level `../README.md` and `../BUILDING.md` for the whole picture.

GPS-free autonomous mobile robot prototype built on **ESP32-S3** with **differential drive**, **encoder odometry**, **VL53L0X obstacle sensing**, **closed-loop PID speed control**, **battery and emergency-stop monitoring**, an **OLED status display**, and a **waypoint navigation system**.

Phase 1 established indoor navigation without GPS using only wheel odometry and a forward ToF sensor. **Phase 2 added** emergency stop, battery monitor, PID control, and serial override. **Phase 3 added** waypoint navigation. **Phase 4 (current)** changes the firmware's role when the Raspberry Pi is connected:

- The **MPU6050 IMU and the firmware-side heading fusion were removed** (per project decision: ROS 2 owns navigation; AMCL corrects odometry drift from the A3 scan).
- When the Pi sends `ROS2 ON`, the firmware behaves as a **transparent motor/encoder/ToF/battery/e-stop driver**. The onboard avoidance FSM, waypoint queue, and reactive obstacle stops are bypassed while ROS 2 owns the brain.
- The firmware streams two telemetry records over USB serial so the Pi can build /odom, /battery_state, /estop, and /tof/range directly:

  ```text
  P x=<m> y=<m> theta=<rad> vL=<m/s> vR=<m/s> age=<ms>     ~50 Hz
  T bat=<volts> estop=<0|1> tof=<mm> mode=<0|1|2>         ~5 Hz
  ```

- The standalone behaviour (AUTO + waypoint + ToF avoidance) is still available when ROS2 mode is OFF, so the robot remains usable with just a USB serial cable and no Pi.

Future phases add camera, AI obstacle detection, and secure medicine delivery.

---

## Features

- Differential drive with two BTS7960 motor drivers
- Quadrature encoder counting on both wheels via interrupts
- Wheel odometry with arc-based pose integration
- Per-wheel ground-speed measurement for closed-loop control
- PID speed controller (KP, KI, KD with anti-windup)
- Time-of-Flight obstacle detection with hysteresis
- Reactive avoidance state machine (left/right pivot then recover) — used in standalone AUTO mode
- Hardware-level ToF safety override in ROS 2 mode (last-resort brake)
- Hardware emergency stop with debounce
- Battery monitor with low-battery safe-halt
- Serial command interface (`ROS2 ON/OFF`, `STOP`, `AUTO`, `RESET`, `M L=… R=…`, `GOTO`, `WPLS`, `WPCLR`)
- Streaming telemetry: pose (`P …`) and battery/e-stop/ToF/mode (`T …`)
- OLED real-time status display
- 50 Hz control loop, fail-safe on sensor dropout

---

## Hardware

| Component | Quantity |
|-----------|----------|
| ESP32-S3 DevKitC-1 | 1 |
| DC Encoder Motor (12 V) | 2 |
| Ball Caster Wheel | 2 |
| BTS7960 Motor Driver | 2 |
| VL53L0X ToF Sensor | 1 |
| 0.96" SSD1306 OLED (I2C) | 1 |
| LiPo Battery (3S recommended) | 1 |
| Voltage divider for battery monitor | 1 |
| Emergency-stop button | 1 |
| Robot Chassis | 1 |

### Pin Map

| Signal | ESP32-S3 GPIO |
|--------|---------------|
| I2C SDA | 8 |
| I2C SCL | 9 |
| Left motor RPWM | 4 |
| Left motor LPWM | 5 |
| Left motor R_EN | 6 |
| Left motor L_EN | 7 |
| Right motor RPWM | 10 |
| Right motor LPWM | 11 |
| Right motor R_EN | 12 |
| Right motor L_EN | 13 |
| Left encoder | 14 |
| Right encoder | 15 |
| ToF XSHUT | 16 |
| ESTOP button | 17 |
| Status LED | 18 |
| Battery ADC | 1 |

> No IMU: the MPU6050 was removed in Phase 4. Heading comes from wheel ticks; AMCL on the Pi corrects drift.

> ADC1_CH3 on the S3 is GPIO4. Adjust the divider and pin if your board differs.

All pins live in `include/config.h` — change them there if your wiring differs.

---

## Build & Upload

1. Install [PlatformIO](https://platformio.org/) (CLI or VS Code extension).
2. Connect the ESP32-S3 via USB.
3. From this directory:

```bash
pio run -t upload
pio device monitor
```

---

## Project Layout

```
AutonomousNavigationRobot/
├── platformio.ini
├── README.md
├── include/
│   ├── config.h
│   ├── motor.h
│   ├── encoder.h
│   ├── odometry.h
│   ├── tof.h
│   ├── obstacle.h
│   ├── navigation.h
│   ├── display.h
│   ├── estop.h
│   ├── battery.h
│   ├── pid.h
│   ├── command.h
│   ├── waypoint.h
│   ├── forward_controller.h
│   ├── telemetry.h
│   └── types.h
└── src/
    ├── main.cpp
    ├── motor.cpp
    ├── encoder.cpp
    ├── odometry.cpp
    ├── tof.cpp
    ├── obstacle.cpp
    ├── navigation.cpp
    ├── display.cpp
    ├── estop.cpp
    ├── battery.cpp
    ├── pid.cpp
    ├── command.cpp
    ├── waypoint.cpp
    ├── forward_controller.cpp
    └── telemetry.cpp
```

---

## Software Architecture

```text
   ┌──────────────┐    ┌──────────────┐
   │ Encoders ISR │    │   ToF sensor │
   └──────┬───────┘    └──────┬───────┘
          │ ticks            │ mm
          ▼                  ▼
   ┌──────────────┐    ┌──────────────┐
   │  Odometry    │◄──▶│  Obstacle FSM│
   │  + speed m/s │    └──────┬───────┘
   └──────┬───────┘           │
          │ pose              │ state
          ▼                  ▼
   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
   │   PID L/R    │◄───┤ Navigation/  │◄───┤   Command    │
   └──────┬───────┘    │ Manual cmd   │    └──────┬───────┘
          │            └──────┬───────┘           │
          │                   │          ROS2 ON / M L=… R=…
          ▼                   ▼                   │
   ┌─────────────────────────────┐                │
   │ Motors + OLED + ESTOP + BAT │                │
   └─────────────────────────────┘                │
                              ▲                   ▼
                              │       ┌──────────────────────┐
                              └──────▶│ Telemetry (P, T)     │──► USB-CDC ──► Pi
                                      └──────────────────────┘
```

The control loop runs at 50 Hz. The PID uses encoder-derived wheel speed; the navigation FSM chooses between `MOVING`, `SLOWING`, `OBSTACLE_STOP`, `AVOIDING_*`, `RECOVERING`, and `FAULT`. The pose and battery/e-stop/ToF telemetry are emitted to the USB-CDC port every control tick (pose) and at 5 Hz (telemetry) for the Raspberry Pi to consume. When `ROS2 ON` is active the firmware's nav FSM is bypassed.

### Navigation State Machine

```text
INIT ─► MOVING ──► SLOWING ──► OBSTACLE_STOP
                       ▲             │
                       │             ▼
                  AVOIDING_L / AVOIDING_R
                       │             │
                       ▼             ▼
                   RECOVERING ◄──────┘
                       │
                       ▼
                  MOVING / FAULT
```

### Obstacle Thresholds (mm)

| State | Enter ≤ | Exit ≥ |
|-------|---------|--------|
| CLEAR | — | > 500 |
| SLOW | 500 | 700 |
| BLOCKED | 250 | 500 |
| SENSOR_FAULT | ToF timeout or I2C error | recovered |

Hysteresis prevents oscillation at the boundary distances.

---

## Serial Commands

Open the serial monitor at **115200 baud**.

| Command | Effect |
|---------|--------|
| `ROS2 ON` | Enter transparent driver mode for the Raspberry Pi (bypass onboard nav FSM). |
| `ROS2 OFF` | Return to standalone behaviour (AUTO + ToF avoidance + waypoints). |
| `STOP` | Freeze both motors. Works in any mode. |
| `M L=40 R=40` | Manual drive (L/R speed in percent, –100…+100). |
| `AUTO` | Return to standalone autonomous navigation. |
| `RESET` | Reset odometry pose (zero x, y, θ). |
| `GOTO X=1.0 Y=0.5` | Queue a standalone waypoint at (x, y) in metres (only used in AUTO mode). |
| `WPLS` | Print waypoint queue length. |
| `WPCLR` | Clear all queued waypoints. |

In ROS 2 mode, `M L=… R=…` and `STOP` work without bouncing through the avoidance FSM. In MANUAL mode (`M L=… R=…` while not in ROS 2 mode), commands time out after `REMOTE_TIMEOUT_MS` (default 3 s) and revert to AUTO.

## ROS 2 Protocol

When the Raspberry Pi's `esp32_bridge` connects, it sends `ROS2 ON` once. From that point on the firmware streams two telemetry records at 115200 baud that the Pi parses directly:

| Record | Rate | Fields |
|--------|------|--------|
| `P` | ~50 Hz | `x`, `y` (m), `theta` (rad), `vL`, `vR` (m/s), `age` (ms since last encoder pulse) |
| `T` | ~5 Hz  | `bat` (V), `estop` (0/1), `tof` (mm), `mode` (0=ROS2, 1=AUTO, 2=FAULT) |

The Pi's `ros2_bridge` parses these and publishes `/odom`, `odom→base_link` TF, `/battery_state`, `/estop`, and `/tof/range`. On shutdown the bridge sends `ROS2 OFF` followed by `STOP`.

Hardware-level safety: even with `ROS2 ON`, the firmware's front ToF sensor can zero both motors for `EMERGENCY_HOLD_MS` (250 ms by default) if it sees an obstacle within `EMERGENCY_STOP_MM` (250 mm). This is independent of Nav2's local costmap and is a last-resort brake.

## Waypoint Navigation (standalone only)

When ROS 2 mode is **OFF**, the robot maintains a queue of up to **8 waypoints**, each expressed in the odometry frame in metres. While a waypoint is active:

1. The forward controller computes a desired linear speed and a steering correction from the heading error.
2. If the heading error exceeds `0.6 rad`, the robot pivots in place.
3. The PID wheel-speed controller closes the loop on per-wheel ground speed.
4. If the robot enters the waypoint's tolerance (`WAYPOINT_TOLERANCE_M`, default 0.10 m), the waypoint is popped and the next one begins.

Example session:

```text
RESET                 # zero the pose
GOTO X=1.0 Y=0.0      # 1 m forward
GOTO X=1.0 Y=1.0      # then turn 90 deg and 1 m
WPLS                  # -> 2 queued
```

> ⚠️ Wheel odometry drifts. Heading is derived from wheel ticks only (no IMU). Waypoint precision is therefore bounded; use this as a relative navigation tool, not an absolute positioning system. When ROS 2 mode is active the waypoint queue is **not** used — send goals through Nav2 instead.

---

## Kinematics

With wheel radius *r*, axle track *L*, and per-wheel arc lengths *dL, dR*:

```text
ds     = (dR + dL) / 2
dθ     = (dR - dL) / L
x'     = x + (ds/dθ) * (sin(θ+dθ) - sin(θ))
y'     = y - (ds/dθ) * (cos(θ+dθ) - cos(θ))
θ'     = θ + dθ
```

The code uses an exact arc update (no small-angle approximation). Theta is wrapped to `[-π, π]`. The IMU integration that used to replace θ with a gyro-integrated heading was removed in Phase 4.

---

## PID Tuning

`config.h` exposes `PID_KP`, `PID_KI`, `PID_KD`, and `PID_INTEGRAL_MAX`. Start with:

- **KP** ≈ 1.5 — proportional response to speed error
- **KI** ≈ 4.0 — eliminates steady-state error
- **KD** ≈ 0.05 — dampens oscillation
- **INTEGRAL_MAX** ≈ 60% — prevents integrator windup

If the robot oscillates while driving straight, reduce KI. If it cannot reach commanded speed, raise KI or KP. Measure wheel speed via the OLED “Trav” and “L/R” rows before tuning.

---

## Emergency Stop

- `ESTOP_PIN` is configured as INPUT_PULLUP (active LOW by default).
- Pressing the button halts both motors immediately and latches the system in FAULT.
- The OLED displays `** ESTOP ACTIVE **`.
- Power-cycle to recover.

If your hardware is active HIGH, set `ESTOP_ACTIVE_LOW = false` in `config.h`.

---

## Battery Monitor

| Field | Default | Meaning |
|-------|---------|---------|
| `BAT_DIVIDER_RATIO` | 4.0303 | 100 kΩ / 33 kΩ divider |
| `BAT_LOW_V` | 10.5 V | 3S LiPo low warning |
| `BAT_CRITICAL_V` | 9.6 V | 3S LiPo critical halt |
| `BAT_PERIOD_MS` | 1000 ms | Sample interval |

When the battery reaches CRITICAL, the robot halts and the OLED shows the fault. Recalibrate these values if you use a different battery or divider.

---

## Acceptance Tests

Before declaring Phase 4 complete:

1. Robot drives forward in a straight line within ±5 % drift over 1 m on flat ground.
2. Robot stops within 50 mm of a wall placed in its path.
3. Left and right wheel rotations are counted independently and direction is correct.
4. Total travel distance estimate matches a measured reference within 10 %.
5. Robot detects a wall and resumes after it is removed.
6. Disconnecting the ToF causes the robot to halt and the OLED to show FAULT.
7. All four BTS7960 enable pins stay HIGH during operation; no unintended reverse.
8. OLED shows mode, distance, speeds, total distance, battery, and heading without flicker.
9. Robot responds to obstacles in both left and right avoidance directions.
10. Pressing the ESTOP button halts the robot within 50 ms; power-cycle restores operation.
11. `M L=40 R=-40` over serial spins the robot in place.
12. **Phase 4 ROS 2 mode**:
    a. Connecting the Pi and starting `esp32_bridge` causes `ROS2 ON` to appear in the firmware's serial output.
    b. The Pi's `/odom` topic advances at ~50 Hz when wheels spin.
    c. The Pi's `/battery_state`, `/estop`, and `/tof/range` topics all populate within 1 s.
    d. With `ROS2 ON` active, `GOTO` commands on the firmware's serial are ignored (no firmware-side driving).
    e. Sending `ROS2 OFF` returns the firmware to AUTO behaviour.
13. **PID reaches commanded wheel speed within ~1 s** with no oscillation.
14. Disconnecting the battery monitor (short divider to GND) shows `LOW` on the OLED.
15. Pulling the battery voltage below `BAT_CRITICAL_V` halts the robot with a CRIT battery message.

---

## Known Limitations

- Single forward sensor cannot see lateral obstacles.
- Wheel slip is still a major error source for odometry.
- Heading is wheel-tick-only (no IMU). On long straight runs slip
  accumulates; AMCL on the Pi corrects drift from the A3 scan.
- When running standalone (ROS 2 OFF), no mapping, no localization,
  no global planning — only reactive avoidance + a small waypoint queue.
- Manual control is wired to serial only — Bluetooth/Wi-Fi are future work.

---

## Future Phases

| Phase | Highlights |
|-------|------------|
| 5 | Camera, AI obstacle detection, human detection, crowd awareness |
| 5 | Secure medicine storage, QR-code destinations, voice prompts, hospital integration |

---

## Safety Notes

This is a prototype. For any hospital deployment you must add:

- Independent emergency-stop hardware
- Redundant braking / wheel drop detection
- Manual override (button + remote)
- Low-battery handling (already in Phase 2)
- Tested behaviour around people and equipment
- Compliance review with hospital safety, electrical, and medical-device regulations

Never run this prototype near patients or in operational corridors without prior approval and supervision.
---

## Build & flash from the workspace root

The ESP32 firmware is built with **PlatformIO**. From anywhere in the
workspace:

```bash
# from ~/rover_ws/
make firmware-build
make firmware-upload    # writes to /dev/ttyUSB0 (or whatever the ESP32 enumerates as)
make firmware-monitor   # 115200 baud serial monitor (Ctrl+C to exit)
```

Or directly:

```bash
cd ~/rover_ws/firmware
pio run -t upload
pio device monitor
```

PlatformIO caches its toolchain under `firmware/.pio/` and build
artifacts under `firmware/build/` — both are git-ignored by the
root `.gitignore`.

## Cross-package constants

These constants are duplicated in three places and **must stay
identical**:

| Constant | Value | Firmware | ROS 2 |
|---|---|---|---|
| Wheel radius | 0.032 m | `include/config.h` `WHEEL_RADIUS_M` | `src/rover_description/urdf/rover_base.xacro` and `src/rover_hardware/config/diff_drive_params.yaml` |
| Wheel base | 0.150 m | `include/config.h` `WHEEL_BASE_M` | same files as above |
| PWM 100% → m/s | 0.30 m/s | `src/main.cpp` `percentToLinearMps` | `src/rover_hardware/config/diff_drive_params.yaml` `v_max_mps` |

If you change one, change all three. The firmware is the source of
truth; the ROS 2 side derives.
