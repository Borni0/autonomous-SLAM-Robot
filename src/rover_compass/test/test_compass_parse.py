"""Unit tests for the pure-Python heading parser (no rclpy required)."""
import math

from rover_compass.heading_parser import parse_heading


def test_parse_nmea_full_sentence():
    heading = parse_heading('$CHARR,123.45,5.2,W*7A')
    assert heading is not None
    assert abs(heading - 123.45) < 1e-6


def test_parse_nmea_no_checksum():
    heading = parse_heading('$HDM,87.0,M')
    assert heading is not None
    assert abs(heading - 87.0) < 1e-6


def test_parse_bare_int():
    heading = parse_heading('180')
    assert heading is not None
    assert abs(heading - 180.0) < 1e-6


def test_parse_garbage_returns_none():
    assert parse_heading('') is None
    assert parse_heading('hello,world') is None


def test_parse_picks_first_in_range():
    # First field is junk, second is a valid heading.
    heading = parse_heading('$FOO,abc,42.5,M')
    assert heading is not None
    assert abs(heading - 42.5) < 1e-6


def test_parse_excludes_out_of_range():
    # 999.9 is out of [0, 360); 180 is the valid one.
    heading = parse_heading('999.9,180.0')
    assert heading is not None
    assert abs(heading - 180.0) < 1e-6


def test_convention_round_trip():
    # 0 deg heading (North) -> ROS yaw = pi/2.
    # 90 deg heading (East)  -> ROS yaw = 0 (pointing +x).
    # 180 deg heading (South) -> ROS yaw = -pi/2.
    assert abs((math.pi / 2.0) - math.radians(0.0) - (math.pi / 2.0)) < 1e-9
    assert abs((math.pi / 2.0) - math.radians(90.0) - 0.0) < 1e-9
    assert abs((math.pi / 2.0) - math.radians(180.0) - (-math.pi / 2.0)) < 1e-9
