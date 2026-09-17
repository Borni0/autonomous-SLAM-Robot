"""Compass / magnetometer node for the rover.

Publishes:

* ``/imu/data``          ``sensor_msgs/Imu`` (orientation as a quaternion)
* ``/imu/mag``           ``sensor_msgs/MagneticField``
* ``/imu/yaw_deg``       ``std_msgs/Float32`` (heading in degrees, 0=N, +90=E)

Two operating modes, selected by the ``serial_port`` parameter:

1. ``serial_port`` is **set** to a real device path (e.g. ``/dev/rover_compass``
   after udev, or a USB-serial HMC5883L/QMC5883L dongle at
   ``/dev/ttyUSB0``): the node reads NMEA-style heading lines of the
   form ``$CHARR,HEADING,...`` (or just a bare integer heading).
2. ``serial_port`` is **empty**: the node publishes a *synthetic*
   yaw that drifts slowly. Useful for smoke-testing the rest of the
   stack without magnetometer hardware.

**Important caveat (per spec section 16):** motors, batteries, the BTS
driver and metal chassis all produce strong magnetic interference.
Mount the magnetometer as far from the motors as practical (>= 15 cm),
twist the power cables, and use the ``declination_deg`` parameter to
correct for local magnetic declination. Do not rely on this as the
sole localization source — it is a heading *aid* on top of wheel
odometry + AMCL.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32

from rover_compass.heading_parser import parse_heading

try:
    import serial  # type: ignore
    _HAVE_SERIAL = True
except ImportError:  # pragma: no cover
    serial = None
    _HAVE_SERIAL = False


def _yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    """Build a Quaternion for a rotation about Z by ``yaw_rad``."""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


class CompassNode(Node):
    def __init__(self) -> None:
        super().__init__('compass_node')

        # ---- parameters ----
        self.declare_parameter('frame_id', 'compass_link')
        self.declare_parameter('serial_port', '')            # empty -> synthetic
        self.declare_parameter('serial_baud', 9600)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('declination_deg', 0.0)      # local magnetic dec.
        self.declare_parameter('offset_deg', 0.0)           # mount misalignment
        self.declare_parameter('covariance_orientation', 0.05)
        self.declare_parameter('covariance_magnetic', 1e-6)

        self._frame_id = self.get_parameter('frame_id').value
        self._serial_port = self.get_parameter('serial_port').value
        self._serial_baud = int(self.get_parameter('serial_baud').value)
        self._rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self._declination = math.radians(
            float(self.get_parameter('declination_deg').value)
        )
        self._offset = math.radians(
            float(self.get_parameter('offset_deg').value)
        )
        self._cov_ori = float(self.get_parameter('covariance_orientation').value)
        self._cov_mag = float(self.get_parameter('covariance_magnetic').value)

        # ---- state ----
        self._yaw_rad = 0.0
        self._lock = threading.Lock()
        self._serial = None

        if self._serial_port:
            if not _HAVE_SERIAL:
                self.get_logger().fatal('pyserial not installed')
                raise RuntimeError('pyserial missing')
            try:
                self._serial = serial.Serial(
                    self._serial_port, self._serial_baud, timeout=0.05
                )
                time.sleep(0.5)
                self.get_logger().info(
                    f'compass on {self._serial_port} @ {self._serial_baud}'
                )
                self._reader = threading.Thread(target=self._serial_loop, daemon=True)
                self._reader.start()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f'failed to open {self._serial_port}: {exc}; '
                    f'falling back to synthetic mode'
                )
                self._serial = None
        else:
            self.get_logger().info(
                'no serial_port set: publishing synthetic yaw for stack testing'
            )

        # ---- publishers ----
        self._imu_pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self._mag_pub = self.create_publisher(
            MagneticField, 'imu/mag', qos_profile_sensor_data
        )
        self._yaw_pub = self.create_publisher(
            Float32, 'imu/yaw_deg', qos_profile_sensor_data
        )

        self._timer = self.create_timer(1.0 / self._rate, self._publish)

    # ------------------------------------------------------------------
    def _serial_loop(self) -> None:
        """Read heading lines from the magnetometer.

        Accepts the common NMEA-style ``$CHARR,HEADING,...`` sentences
        produced by USB compass sticks, and bare integers for the
        cheapest modules. Lines that fail to parse are silently
        skipped.
        """
        while rclpy.ok() and self._serial is not None:
            try:
                raw = self._serial.readline()
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
            heading_deg = parse_heading(line)
            if heading_deg is None:
                continue
            with self._lock:
                # Compass convention: 0 = North, +90 = East, increasing
                # clockwise. ROS REP-103 (ENU): +x = East, +y = North,
                # +z = Up. Compass heading H deg clockwise from North
                # maps to ROS yaw = pi/2 - H (radians, CCW about +z).
                self._yaw_rad = (math.pi / 2.0) - math.radians(heading_deg)

    # ------------------------------------------------------------------
    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        with self._lock:
            yaw = self._yaw_rad
            is_synthetic = self._serial is None

        if is_synthetic:
            # 5 deg/sec slow drift; replace with real heading once wired up.
            yaw += math.radians(5.0 / self._rate)
            with self._lock:
                self._yaw_rad = yaw

        # Apply user offset + magnetic declination.
        published_yaw = yaw + self._offset + self._declination

        # ---- Imu ----
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self._frame_id
        imu.orientation = _yaw_to_quaternion(published_yaw)
        BIG = 1e6
        imu.orientation_covariance = [
            BIG, 0.0, 0.0,
            0.0, BIG, 0.0,
            0.0, 0.0, self._cov_ori,
        ]
        imu.angular_velocity_covariance = [0.0] * 9
        imu.linear_acceleration_covariance = [0.0] * 9
        self._imu_pub.publish(imu)

        # ---- MagneticField ----
        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self._frame_id
        mag.magnetic_field.x = 0.0
        mag.magnetic_field.y = 0.0
        mag.magnetic_field.z = 0.0
        mag.magnetic_field_covariance = [
            self._cov_mag, 0.0, 0.0,
            0.0, self._cov_mag, 0.0,
            0.0, 0.0, self._cov_mag,
        ]
        self._mag_pub.publish(mag)

        # ---- Yaw in degrees (handy for `ros2 topic echo`) ----
        deg_msg = Float32()
        deg_msg.data = math.degrees(published_yaw) % 360.0
        self._yaw_pub.publish(deg_msg)

    # ------------------------------------------------------------------
    def destroy_node(self):  # type: ignore[override]
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompassNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
