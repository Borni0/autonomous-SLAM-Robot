"""ROS-side soft e-stop.

Listens to a Boolean on ``/soft_estop`` (Bool, default True = SAFE).
Whenever the value is False, this node forces every message on
``/cmd_vel`` to zero before it reaches the ESP32 bridge. When the
value returns to True, the latest /cmd_vel passes through unchanged.

Wiring:

    /cmd_vel_in -> [soft_estop] -> /cmd_vel_out -> esp32_bridge

The recommended topology is to remap the nav stack's published
``/cmd_vel_nav`` (or whatever Nav2 uses) into ``/cmd_vel_in``, and let
this node republish on ``/cmd_vel``. See rover_bringup/launch for the
exact ``remappings`` argument.

The hardware e-stop on the ESP32 (ESTOP_PIN) remains the authoritative
final stop; this node is a ROS-side convenience that prevents
autonomous commands from reaching the motors while a human has the
software estop latched.
"""
from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Bool


class SoftEstop(Node):
    def __init__(self) -> None:
        super().__init__('soft_estop')

        # ---- parameters ----
        self.declare_parameter('latched', True)
        self._latched = bool(self.get_parameter('latched').value)

        # ---- state ----
        self._estopped = False
        self._latest_cmd = Twist()
        self._lock = threading.Lock()

        # ---- pub/sub ----
        self._pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel_in', self._on_cmd_vel, qos_profile_system_default
        )
        self._estop_sub = self.create_subscription(
            Bool, 'soft_estop', self._on_estop, qos_profile_system_default
        )

        # 50 Hz gate so the bridge never goes silent when /cmd_vel_in is
        # slow or absent.
        self._timer = self.create_timer(1.0 / 50.0, self._on_tick)

        self.get_logger().info(
            f'soft_estop ready (latched={self._latched}); '
            f'send std_msgs/Bool False on /soft_estop to stop.'
        )

    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._latest_cmd = msg

    def _on_estop(self, msg: Bool) -> None:
        with self._lock:
            prev = self._estopped
            self._estopped = bool(msg.data)
            if self._estopped and not prev:
                self.get_logger().warn('SOFT E-STOP ENGAGED (cmd_vel forced to zero)')
            elif not self._estopped and prev:
                self.get_logger().info('soft e-stop released')

    def _on_tick(self) -> None:
        with self._lock:
            cmd = self._latest_cmd
            estopped = self._estopped

        out = Twist()
        if not estopped:
            out = cmd
        # else: leave out as zero Twist -> ESP32 stops.
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SoftEstop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
