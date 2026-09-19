"""rclpy node that bridges the Raspberry Pi ↔ ESP32-S3 link.

Subscribes:
    /cmd_vel           (geometry_msgs/Twist) -> sent as ``M L=… R=…``

Parses from the ESP32:
    P x= y= theta= vL= vR= age=    -> /odom + odom→base_link TF
    T bat= estop= tof= mode=        -> /battery_state + /estop + /tof/range

On startup the bridge sends ``ROS2 ON`` so the firmware engages its
transparent driver mode (the firmware's own avoidance / waypoint FSM is
bypassed while ROS 2 owns navigation). On shutdown it sends ``ROS2 OFF``
followed by ``STOP`` so a bare ESP32 left powered up reverts to the
standalone AUTO behaviour.

If the pose stream stalls (``age`` growing or no lines for >200 ms), the
bridge falls back to its own dead-reckoning integration from
``/cmd_vel`` so /odom and the TF never go dark.

The serial protocol is documented in ``firmware/src/telemetry.cpp`` and
``firmware/src/command.cpp``.
"""
from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from rover_hardware._firmware_proto import (
    PoseRecord, TelemetryRecord, parse_pose, parse_telemetry,
)
from rover_hardware._kinematics import motor_line, twist_to_wheel_pcts
from rover_hardware._odom_accumulator import OdomAccumulator, quat_from_yaw

try:
    import serial  # type: ignore
    _HAVE_SERIAL = True
except ImportError:  # pragma: no cover
    serial = None
    _HAVE_SERIAL = False


# ---------------------------------------------------------------------------
# 3S LiPo -> percentage. Used to populate BatteryState.percentage.
# Voltages are nominal: full=12.6, nominal=11.1, low=10.5, critical=9.6.
# The exact open-circuit voltage at any given SoC depends on load;
# treat this as a UI hint, not a coulomb counter.
# ---------------------------------------------------------------------------
_BAT_FULL_V = 12.6
_BAT_NOMINAL_V = 11.1
_BAT_LOW_V = 10.5
_BAT_CRITICAL_V = 9.6


def _battery_percentage(volts: float) -> float:
    """Map a 3S LiPo voltage reading to a rough SoC percentage in [0, 100]."""
    if volts <= 0.0:
        return 0.0
    if volts >= _BAT_FULL_V:
        return 100.0
    if volts <= _BAT_CRITICAL_V:
        return 0.0
    # Linear between critical and full.
    return max(0.0, min(100.0, 100.0 * (volts - _BAT_CRITICAL_V) / (_BAT_FULL_V - _BAT_CRITICAL_V)))


class Esp32Bridge(Node):
    def __init__(self) -> None:
        super().__init__('esp32_bridge')

        # ---- parameters ----
        self.declare_parameter('serial_port', '/dev/rover_esp32')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('wheel_base', 0.150)
        self.declare_parameter('wheel_radius', 0.032)
        self.declare_parameter('v_max_mps', 0.30)
        self.declare_parameter('cmd_timeout_ms', 500)
        self.declare_parameter('publish_rate_hz', 20.0)
        # Dead-reckoning fallback when the pose stream stalls.
        self.declare_parameter('pose_stale_ms', 250)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tof_frame_id', 'tof_link')
        self.declare_parameter('battery_frame_id', 'base_link')

        self._port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('serial_baud').value)
        self._wheel_base = float(self.get_parameter('wheel_base').value)
        self._v_max = float(self.get_parameter('v_max_mps').value)
        self._cmd_timeout_ns = int(self.get_parameter('cmd_timeout_ms').value) * 1_000_000
        self._period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1e-3)
        self._pose_stale_ns = int(self.get_parameter('pose_stale_ms').value) * 1_000_000
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._tof_frame = self.get_parameter('tof_frame_id').value
        self._battery_frame = self.get_parameter('battery_frame_id').value

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

        # Encoder-derived state, owned by an accumulator so the pose
        # update + dead-reckoning path is unit-testable without rclpy.
        # Anchor the wall clock so the very first fallback step has a
        # sane dt instead of treating the wall clock as "forever ago".
        self._odom = OdomAccumulator(
            wheel_base=self._wheel_base,
            now_ns=self.get_clock().now().nanoseconds,
        )

        self._lock = threading.Lock()

        # ---- ROS pub/sub ----
        self._sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_twist, qos_profile_system_default
        )

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_pub = TransformBroadcaster(self)
        self._battery_pub = self.create_publisher(
            BatteryState, 'battery_state', qos_profile_sensor_data
        )
        self._estop_pub = self.create_publisher(
            Bool, 'estop', qos_profile_system_default
        )
        self._tof_pub = self.create_publisher(
            Range, 'tof/range', qos_profile_sensor_data
        )

        # Timer handles command resend + odom/TF publication.
        self._send_timer = self.create_timer(self._period, self._on_tick)

        # Read the ESP32's output in its own thread so we never block
        # the ROS executor.
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        # Engage the firmware's transparent driver mode.
        try:
            self._ser.write(b"ROS2 ON\n")
            self._ser.flush()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'failed to send ROS2 ON: {exc}')

    def _on_twist(self, msg: Twist) -> None:
        with self._lock:
            self._latest_v = float(msg.linear.x)
            self._latest_w = float(msg.angular.z)
            self._last_cmd_time_ns = self.get_clock().now().nanoseconds
            self._stop_watchdog_tripped = False

    def _on_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            age_ns = now_ns - self._last_cmd_time_ns
            if age_ns > self._cmd_timeout_ns:
                if not self._stop_watchdog_tripped:
                    self.get_logger().warn('cmd_vel watchdog tripped, sending STOP')
                    self._stop_watchdog_tripped = True
                self._send(b"STOP\n")
                self._last_send_time_ns = now_ns
            else:
                if now_ns - self._last_send_time_ns >= int(self._period * 1e9):
                    pct_L, pct_R = twist_to_wheel_pcts(
                        self._latest_v, self._latest_w, self._wheel_base, self._v_max
                    )
                    self._send(motor_line(pct_L, pct_R))
                    self._last_send_time_ns = now_ns

        # Publish /odom + odom->base_link. The pose fields come straight
        # from the firmware when fresh; otherwise we fall back to local
        # dead-reckoning so /odom is never silent.
        self._publish_odom(now_ns)

    def _publish_odom(self, now_ns: int) -> None:
        with self._lock:
            latest_v = self._latest_v
            latest_w = self._latest_w
            last_cmd_ns = self._last_cmd_time_ns
            cmd_zero = (now_ns - last_cmd_ns) > self._cmd_timeout_ns
            self._odom.integrate_dead_reckoning(
                v_cmd=latest_v,
                w_cmd=latest_w,
                now_ns=now_ns,
                cmd_is_zero=cmd_zero,
                pose_stale_ns=self._pose_stale_ns,
            )

        stamp_msg = self.get_clock().now().to_msg()
        twist_x = 0.0 if cmd_zero else latest_v
        twist_z = 0.0 if cmd_zero else latest_w

        odom = Odometry()
        # Nav2 expects commanded twist in /odom.twist (the firmware's
        # measured vL/vR lives in the encoder pipeline, not here).
        pose_cov = [0.02, 0.02, 0.0, 0.0, 0.0, 0.02]
        for i, val in enumerate(pose_cov):
            odom.pose.covariance[i * 6 + i] = val
        twist_cov = [0.05, 0.05, 0.0, 0.0, 0.0, 0.10]
        for i, val in enumerate(twist_cov):
            odom.twist.covariance[i * 6 + i] = val
        t = TransformStamped()
        self._odom.fill_odom_msg(
            odom, t,
            odom_frame=self._odom_frame,
            base_frame=self._base_frame,
            stamp_msg=stamp_msg,
            twist_x=twist_x,
            twist_z=twist_z,
        )
        self._odom_pub.publish(odom)
        self._tf_pub.sendTransform(t)

    def _send(self, data: bytes) -> None:
        try:
            self._ser.write(data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'serial write failed: {exc}')

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
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception:  # noqa: BLE001
                continue
            if not line:
                continue

            if line.startswith('P ') or line.startswith('P\t'):
                rec = parse_pose(line)
                if rec is not None:
                    self._on_pose(rec)
                    continue
            if line.startswith('T ') or line.startswith('T\t'):
                rec = parse_telemetry(line)
                if rec is not None:
                    self._on_telemetry(rec)
                    continue
            # Human-readable debug or unknown record -> log it.
            self.get_logger().info(f'[esp32] {line}')

    def _on_pose(self, rec: PoseRecord) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            self._odom.update_from_pose(rec, now_ns)

    def _on_telemetry(self, rec: TelemetryRecord) -> None:
        now = self.get_clock().now().to_msg()

        # ---- BatteryState ----
        bat = BatteryState()
        bat.header.stamp = now
        bat.header.frame_id = self._battery_frame
        bat.voltage = rec.battery_volts
        bat.current = float('nan')   # not measured
        bat.charge = float('nan')
        bat.capacity = float('nan')
        bat.design_capacity = float('nan')
        bat.percentage = _battery_percentage(rec.battery_volts) / 100.0
        bat.power_state = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        bat.present = True
        self._battery_pub.publish(bat)

        # ---- /estop (Bool) ----
        b = Bool()
        b.data = rec.estop
        self._estop_pub.publish(b)

        # ---- /tof/range (sensor_msgs/Range, INFRARED) ----
        rng = Range()
        rng.header.stamp = now
        rng.header.frame_id = self._tof_frame
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = 0.44  # ~25 deg, VL53L0X typical
        rng.min_range = 0.03
        rng.max_range = 2.0
        rng.range = rec.tof_mm / 1000.0 if rec.tof_mm > 0 else float('inf')
        self._tof_pub.publish(rng)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:  # type: ignore[override]
        try:
            self._ser.write(b"STOP\n")
            self._ser.write(b"ROS2 OFF\n")
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
