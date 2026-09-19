#!/usr/bin/env bash
# Tiny helpers around the spec's section 19-20 motor + odometry tests.
#
# Usage:
#   source src/rover_bringup/scripts/drive_once.sh
#   drive_once 0.1           # 0.1 m/s forward, single shot
#   drive_once 0.0 0.5       # 0.5 rad/s turn in place
#
# Requires:
#   * ros2 environment sourced
#   * esp32_bridge running (it publishes /odom + TF by itself)
#   * /cmd_vel unmuted by soft_estop (estop_cli release)
#
# Always verify the robot is lifted or otherwise safe before calling.

drive_once() {
  local v="${1:-0.0}"
  local w="${2:-0.0}"
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: ${v}}, angular: {z: ${w}}}"
}

drive_stop() {
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}"
}

estop_stop() {
  ros2 run rover_hardware estop_cli stop
}

estop_release() {
  ros2 run rover_hardware estop_cli release
}
