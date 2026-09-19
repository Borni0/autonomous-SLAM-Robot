# Indoor Mapping Procedure

The first time the rover is deployed in a new environment, you have to
build a 2D occupancy grid for AMCL/Nav2 to localise against. This doc
captures the procedure that produced good maps on the test bench.

## Pre-flight

1. Verify ESP32 is flashed and `/dev/rover_esp32` exists (or fall back
   to `/dev/ttyUSB0`). `make firmware-monitor` should print
   `[ANR] Ready`.
2. Verify the LiDAR is spinning: `ros2 launch rover_lidar a3.launch.py`
   then `ros2 topic hz /scan` should report ~10 Hz.
3. Charge the battery above 11.5 V (`T bat=` line in serial monitor).
4. Place the rover within Wi-Fi range of the controlling laptop.

## Drive the mapping session

```bash
# On the laptop:
ssh pi@rover.local
cd ~/rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# In one terminal - the rover:
ros2 launch rover_bringup rover.launch.py mode:=mapping
```

In RViz:

* "2D Pose Estimate" sets the initial robot pose.
* For the first run, **drive the rover with a joystick or
  teleop_twist_keyboard**, **slowly** (linear.x < 0.10 m/s,
  angular.z < 0.5 rad/s), in smooth arcs. Avoid in-place rotations.
* Cover every room and corridor once, then **return** through
  previously scanned areas so SLAM Toolbox's loop closure can match
  the scan against itself.

## Do

* Drive slowly.
* Return through the same path so loop closure can fire.
* Keep the LiDAR's view of distinctive features (door frames, wall
  corners) unobstructed.
* Stop occasionally and let SLAM publish a few maps.

## Avoid

* High speed (wheel slip ruins odometry).
* Sudden in-place rotations (the encoder-derived pose jumps).
* Driving into a large open area with no features (the scan matcher
  gets lost).
* Losing LiDAR sight of any walls (occluding the sensor with a hand).
* Powering off mid-session.

## Save the map

When the map looks correct in RViz (walls are straight, rooms don't
shift, the robot returns to its starting location cleanly):

```bash
# In a second terminal on the rover:
ros2 run nav2_map_server map_saver_cli -f ~/rover_ws/maps/indoor_map
```

Two files land in `~/rover_ws/maps/`:

```
indoor_map.pgm    # the occupancy grid (white=free, black=wall)
indoor_map.yaml   # resolution, origin, thresholds
```

Move them into `src/rover_navigation/maps/` and commit them so the
default `world:=/home/$USER/rover_ws/maps/indoor_map.yaml` resolves.

## When the map is bad

Walk through this checklist, top to bottom:

1. Open `firmware/include/config.h` — confirm `WHEEL_RADIUS_M` and
   `WHEEL_BASE_M` match a physical measurement of the chassis.
2. With the rover on blocks, push it forward by hand. In
   `make firmware-monitor` both `Left Count` and `Right Count`
   should *increase* for forward motion and *decrease* for reverse.
3. `ros2 topic echo /odom` — pose drift while driving straight tells
   you the wheel radius is wrong (or one wheel is slipping).
4. Check `ros2 run tf2_ros tf2_echo odom base_link` — the transform
   should change smoothly, not jump.
5. Check `ros2 topic echo /scan | head` — `/scan` should be on
   `frame_id: lidar_link` (URDF-derived) or `laser_frame` (the
   driver's default). Mismatched frame_id produces a silently broken
   TF chain.

If the map is still distorted after all of the above, the most likely
cause is wheel-track or wheel-diameter calibration; measure the
chassis physically and update `config.h` + `diff_drive_params.yaml`.
