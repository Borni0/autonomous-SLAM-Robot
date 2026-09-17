"""Dead-reckoning differential-drive odometry node.

The ESP32 firmware does not stream encoder ticks back over the serial
link, so we reconstruct odom->base_link TF from the same /cmd_vel that
the ESP32 receives. When real encoder feedback is added later, only
this node needs to be replaced.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import TransformBroadcaster

from rover_hardware._kinematics import integrate_pose


class DiffDriveOdometry(Node):
    def __init__(self) -> None:
        super().__init__('diff_drive_odometry')

        # ---- parameters ----
        self.declare_parameter('wheel_base', 0.150)
        self.declare_parameter('wheel_radius', 0.032)
        # Linear wheel speed (m/s) that the firmware maps to 100% PWM.
        # Must equal esp32_bridge.v_max_mps and firmware percentToLinearMps(100).
        self.declare_parameter('v_max_mps', 0.30)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_timeout_ms', 500)
        self.declare_parameter('publish_rate_hz', 50.0)

        self._wheel_base = float(self.get_parameter('wheel_base').value)
        self._v_max = float(self.get_parameter('v_max_mps').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._cmd_timeout_ns = int(self.get_parameter('cmd_timeout_ms').value) * 1_000_000
        self._period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)

        # ---- state ----
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._v_L_mps = 0.0
        self._v_R_mps = 0.0
        self._latest_v = 0.0
        self._latest_w = 0.0
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds
        self._last_tick_time_ns = self.get_clock().now().nanoseconds

        # ---- pub/sub ----
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_pub = TransformBroadcaster(self) if self._publish_tf else None
        self._sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_twist, qos_profile_system_default
        )

        self._timer = self.create_timer(self._period, self._on_tick)

    # ------------------------------------------------------------------
    def _on_twist(self, msg: Twist) -> None:
        self._latest_v = float(msg.linear.x)
        self._latest_w = float(msg.angular.z)
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds

    # ------------------------------------------------------------------
    def _on_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self._last_tick_time_ns) / 1e9
        self._last_tick_time_ns = now_ns
        if dt <= 0.0:
            return

        # If no new cmd_vel arrived, treat as STOP.
        if now_ns - self._last_cmd_time_ns > self._cmd_timeout_ns:
            v_cmd = 0.0
            w_cmd = 0.0
        else:
            v_cmd = self._latest_v
            w_cmd = self._latest_w

        # Direct differential-drive forward kinematics to wheel speeds
        # (avoids the integer-percent round-trip used by the bridge).
        half = self._wheel_base / 2.0
        self._v_L_mps = v_cmd - w_cmd * half
        self._v_R_mps = v_cmd + w_cmd * half

        self._x, self._y, self._theta = integrate_pose(
            self._x, self._y, self._theta,
            self._v_L_mps, self._v_R_mps, self._wheel_base, dt,
        )

        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        q = self._quat_from_yaw(self._theta)
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v_cmd
        odom.twist.twist.angular.z = w_cmd

        # Naive covariances — tune once we have ground-truth comparison.
        pose_cov = [0.05, 0.05, 0.0, 0.0, 0.0, 0.05]
        for i, val in enumerate(pose_cov):
            odom.pose.covariance[i * 6 + i] = val
        twist_cov = [0.10, 0.10, 0.0, 0.0, 0.0, 0.25]
        for i, val in enumerate(twist_cov):
            odom.twist.covariance[i * 6 + i] = val

        self._odom_pub.publish(odom)

        if self._tf_pub is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self._tf_pub.sendTransform(t)

    # ------------------------------------------------------------------
    @staticmethod
    def _quat_from_yaw(yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiffDriveOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
