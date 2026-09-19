"""Pure-Python odometry accumulator for the ESP32 bridge.

Mirrors ``firmware/src/odometry.cpp`` so the Pi-side ``esp32_bridge``
can keep publishing /odom when the firmware pose stream
(``P x= y= theta= vL= vR= age=``) goes quiet.

Two state sources feed one estimator:

1. Firmware ``P`` records (50 Hz on the wire).  Authoritative.
2. Local dead-reckoning from /cmd_vel.  Fallback when the pose stream
   is stale or hasn't started.

The integration here MUST match ``firmware/src/odometry.cpp::update``
bit-for-bit so when given the same wheel velocities the Pi fallback
agrees with the firmware pose.  ``_kinematics.integrate_pose`` already
enforces that contract for the unit tests, so we delegate to it.

Both mutators are free of rclpy / threading / I/O so the file is
unit-testable without spinning up ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from rover_hardware._firmware_proto import PoseRecord
from rover_hardware._kinematics import integrate_pose, twist_to_wheel_mps


@dataclass
class _Snapshot:
    x: float
    y: float
    theta: float                # radians, CCW positive, normalised
    v_left_mps: float
    v_right_mps: float
    last_fresh_pose_ns: int     # 0 if no pose has been adopted yet
    last_step_ns: int           # wall-clock anchor for dt in fallback


class OdomAccumulator:
    """Owns the rover's pose estimate.

    Thread safety: a single instance is expected to be guarded by an
    external lock by the caller (the bridge uses ``self._lock``).
    """

    def __init__(self, wheel_base: float, now_ns: int = 0) -> None:
        self._wheel_base = wheel_base
        self._state = _Snapshot(0.0, 0.0, 0.0, 0.0, 0.0, 0, now_ns)

    @property
    def wheel_base(self) -> float:
        return self._wheel_base

    def snapshot(self) -> _Snapshot:
        return self._state

    def fill_odom_msg(
        self,
        odom: 'object',
        tf: 'object',
        odom_frame: str,
        base_frame: str,
        stamp_msg: 'object',
        twist_x: float,
        twist_z: float,
    ) -> None:
        """Stamp an Odometry + TransformStamped pair with the current pose.

        Caller is responsible for constructing empty messages; this only
        fills pose, twist (linear.x / angular.z), and covariance-free
        header / frame fields.
        """
        s = self._state
        qx, qy, qz, qw = quat_from_yaw(s.theta)

        odom.header.stamp = stamp_msg
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = s.x
        odom.pose.pose.position.y = s.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = twist_x
        odom.twist.twist.angular.z = twist_z

        tf.header.stamp = stamp_msg
        tf.header.frame_id = odom_frame
        tf.child_frame_id = base_frame
        tf.transform.translation.x = s.x
        tf.transform.translation.y = s.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

    def reset(self, now_ns: int) -> None:
        """Zero pose; preserve last_fresh_pose_ns so the fallback's
        freshness check still has an anchor."""
        self._state = _Snapshot(
            0.0, 0.0, 0.0, 0.0, 0.0,
            self._state.last_fresh_pose_ns, now_ns,
        )

    def update_from_pose(self, rec: PoseRecord, now_ns: int) -> None:
        """Adopt a firmware pose verbatim. Authoritative source."""
        self._state.x = rec.x
        self._state.y = rec.y
        self._state.theta = rec.theta
        self._state.v_left_mps = rec.v_left_mps
        self._state.v_right_mps = rec.v_right_mps
        self._state.last_fresh_pose_ns = now_ns
        self._state.last_step_ns = now_ns

    def integrate_dead_reckoning(
        self,
        v_cmd: float,
        w_cmd: float,
        now_ns: int,
        cmd_is_zero: bool,
        pose_stale_ns: int,
    ) -> bool:
        """Step the pose estimate forward by dead-reckoning the commanded
        twist. Returns True if the fallback actually integrated; False
        when a fresh firmware pose is in use and only the dt anchor was
        refreshed.

        Falls back when **either** the firmware pose is stale (or never
        arrived) **or** the cmd_vel watchdog has tripped (``cmd_is_zero``);
        in the latter case wheel speeds are zeroed so /odom.twist
        matches what the firmware will report.
        """
        last_pose_ns = self._state.last_fresh_pose_ns
        pose_age = (now_ns - last_pose_ns) if last_pose_ns else None
        fallback = (
            last_pose_ns == 0
            or (pose_age is not None and pose_age > pose_stale_ns)
            or cmd_is_zero
        )
        if not fallback:
            self._state.last_step_ns = now_ns
            return False

        dt = (now_ns - self._state.last_step_ns) / 1e9
        if dt < 0.0:
            dt = 0.0

        v_eff = 0.0 if cmd_is_zero else v_cmd
        w_eff = 0.0 if cmd_is_zero else w_cmd
        v_L, v_R = twist_to_wheel_mps(v_eff, w_eff, self._wheel_base)

        x, y, theta = integrate_pose(
            self._state.x, self._state.y, self._state.theta,
            v_L, v_R, self._wheel_base, dt,
        )

        self._state.x = x
        self._state.y = y
        self._state.theta = theta
        self._state.v_left_mps = v_L
        self._state.v_right_mps = v_R
        self._state.last_step_ns = now_ns
        return True


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Planar yaw (rad) -> (x, y, z, w) quaternion. Plain floats so the
    caller can construct a ``geometry_msgs.msg.Quaternion`` or compare
    values directly in a test."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))