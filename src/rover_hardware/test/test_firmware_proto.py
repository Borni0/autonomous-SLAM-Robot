"""Unit tests for the firmware-line parser.

Mirrors ``firmware/src/telemetry.cpp``. Run on the dev machine:

    python3 -m pytest src/rover_hardware/test/test_firmware_proto.py -v
"""
from __future__ import annotations

from rover_hardware._firmware_proto import parse_pose, parse_telemetry


class TestParsePose:
    def test_realistic_line(self) -> None:
        rec = parse_pose(
            'P x=0.1234 y=-0.0567 theta=0.0123 vL=0.300 vR=0.298 age=20'
        )
        assert rec is not None
        assert abs(rec.x - 0.1234) < 1e-9
        assert abs(rec.y + 0.0567) < 1e-9
        assert abs(rec.theta - 0.0123) < 1e-9
        assert abs(rec.v_left_mps - 0.300) < 1e-9
        assert abs(rec.v_right_mps - 0.298) < 1e-9
        assert rec.age_ms == 20

    def test_negative_theta_and_age(self) -> None:
        rec = parse_pose(
            'P x=0.0000 y=0.0000 theta=-1.5708 vL=-0.300 vR=0.300 age=250'
        )
        assert rec is not None
        assert abs(rec.theta + 1.5708) < 1e-9
        assert rec.age_ms == 250

    def test_tolerates_extra_whitespace(self) -> None:
        rec = parse_pose('   P    x=1.0  y=2.0   theta=0.0 vL=0  vR=0  age=0 ')
        assert rec is not None
        assert rec.x == 1.0
        assert rec.y == 2.0

    def test_rejects_malformed(self) -> None:
        assert parse_pose('') is None
        assert parse_pose('not a P line') is None
        assert parse_pose('P x=abc y=2.0') is None
        assert parse_pose('P x=1.0') is None  # missing fields
        assert parse_pose('T bat=11.0') is None  # wrong prefix

    def test_ignores_unknown_fields(self) -> None:
        rec = parse_pose(
            'P x=1.0 y=2.0 theta=0.0 vL=0 vR=0 age=0 extra=ok crc=12345'
        )
        assert rec is not None
        assert rec.x == 1.0


class TestParseTelemetry:
    def test_realistic_line(self) -> None:
        rec = parse_telemetry('T bat=11.85 estop=0 tof=523 mode=0')
        assert rec is not None
        assert abs(rec.battery_volts - 11.85) < 1e-9
        assert rec.estop is False
        assert rec.tof_mm == 523
        assert rec.mode == 0

    def test_estop_true_and_fault_mode(self) -> None:
        rec = parse_telemetry('T bat=9.50 estop=1 tof=200 mode=2')
        assert rec is not None
        assert rec.estop is True
        assert rec.mode == 2

    def test_tof_zero_distance_is_parsed(self) -> None:
        # Hardware may report 0 when out of range; parser still
        # succeeds; downstream uses inf().
        rec = parse_telemetry('T bat=11.10 estop=0 tof=0 mode=1')
        assert rec is not None
        assert rec.tof_mm == 0

    def test_rejects_malformed(self) -> None:
        assert parse_telemetry('') is None
        assert parse_telemetry('estop=0') is None  # missing T prefix
        assert parse_telemetry('T bat=xyz estop=0 tof=0 mode=0') is None
        assert parse_telemetry('P x=1.0') is None  # wrong prefix
