"""rclpy node that talks to the ESP32-S3 over USB serial.

Protocol (see ~/AutonomousNavigationRobot/src/command.cpp):

* Baud: 115200.
* Line discipline: LF terminated, CR ignored.
* Motor command accepted: ``M L=<int8> R=<int8>\\n`` (e.g. ``M L=40 R=-10\\n``).
* The only command this node ever sends is ``M L=… R=…`` (plus ``STOP\\n``
  on watchdog).
* ESP32 logs back lines like ``[ANR] MANUAL L=40 R=10`` or
  ``[ANR] FAULT: …``; we just log them at INFO.
"""
from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rover_hardware._kinematics import motor_line, twist_to_wheel_pcts

try:
    import serial  # type: ignore
    _HAVE_SERIAL = True
except ImportError:  # pragma: no cover - exercised only on dev machines
    serial = None
    _HAVE_SERIAL = False


class Esp32Bridge(Node):
    def __init__(self) -> None:
        super().__init__('esp32_bridge')

        # ---- parameters ----
        # Default points at the udev symlink created by
        # docs/99-rover-esp32.rules so the device works without
        # permission games. Override at launch time if you must.
        self.declare_parameter('serial_port', '/dev/rover_esp32')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('wheel_base', 0.150)
        self.declare_parameter('wheel_radius', 0.032)
        self.declare_parameter('v_max_mps', 0.30)
        self.declare_parameter('cmd_timeout_ms', 500)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('serial_baud').value)
        self._wheel_base = float(self.get_parameter('wheel_base').value)
        self._v_max = float(self.get_parameter('v_max_mps').value)
        self._cmd_timeout_ns = int(self.get_parameter('cmd_timeout_ms').value) * 1_000_000
        self._period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1e-3)

        # ---- serial ----
        if not _HAVE_SERIAL:
            self.get_logger().fatal('pyserial not installed; run `pip3 install pyserial`')
            raise RuntimeError('pyserial missing')
        self._ser = serial.Serial(self._port, self._baud, timeout=0.05)
        time.sleep(1.0)  # let the ESP32 finish its boot banner
        self.get_logger().info(
            f'serial open: {self._port} @ {self._baud} '
            f'(wheel_base={self._wheel_base}, v_max={self._v_max})'
        )

        # ---- state ----
        self._latest_v = 0.0
        self._latest_w = 0.0
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds
        self._last_send_time_ns = 0
        self._stop_watchdog_tripped = False

        self._lock = threading.Lock()

        # ---- ROS ----
        self._sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_twist, qos_profile_system_default
        )

        self._send_timer = self.create_timer(self._period, self._on_tick)

        # Read the ESP32's debug output in its own thread so we never
        # block the ROS executor.
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------
    # Subscription
    # ------------------------------------------------------------------
    def _on_twist(self, msg: Twist) -> None:
        with self._lock:
            self._latest_v = float(msg.linear.x)
            self._latest_w = float(msg.angular.z)
            self._last_cmd_time_ns = self.get_clock().now().nanoseconds
            self._stop_watchdog_tripped = False

    # ------------------------------------------------------------------
    # Periodic send
    # ------------------------------------------------------------------
    def _on_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            age_ns = now_ns - self._last_cmd_time_ns
            if age_ns > self._cmd_timeout_ns:
                # No fresh twist in too long -> STOP.
                if not self._stop_watchdog_tripped:
                    self.get_logger().warn('cmd_vel watchdog tripped, sending STOP')
                    self._stop_watchdog_tripped = True
                self._send(b"STOP\n")
                self._last_send_time_ns = now_ns
                return

            # Re-send the latest command at ~publish_rate_hz so the ESP32's
            # own REMOTE_TIMEOUT_MS=3000 doesn't kick in.
            if now_ns - self._last_send_time_ns < int(self._period * 1e9):
                return

            pct_L, pct_R = twist_to_wheel_pcts(
                self._latest_v, self._latest_w, self._wheel_base, self._v_max
            )
            self._send(motor_line(pct_L, pct_R))
            self._last_send_time_ns = now_ns

    def _send(self, data: bytes) -> None:
        try:
            self._ser.write(data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'serial write failed: {exc}')

    # ------------------------------------------------------------------
    # Reader thread: passive log of ESP32 debug output.
    # ------------------------------------------------------------------
    def _reader_loop(self) -> None:
        while rclpy.ok():
            try:
                raw = self._ser.readline()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'serial read failed: {exc}')
                time.sleep(0.5)
                continue
            if not raw:
                continue
            try:
                line = raw.decode('utf-8', errors='replace').rstrip('\r\n')
            except Exception:  # noqa: BLE001
                continue
            if line:
                self.get_logger().info(f'[esp32] {line}')

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:  # type: ignore[override]
        try:
            self._ser.write(b"STOP\n")
            self._ser.flush()
            self._ser.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
