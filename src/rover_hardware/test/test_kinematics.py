"""Unit tests for the pure-Python kinematics helpers."""
import math

from rover_hardware._kinematics import (
    integrate_pose,
    motor_line,
    twist_to_wheel_pcts,
)


def test_twist_forward_max():
    pct_L, pct_R = twist_to_wheel_pcts(0.30, 0.0, 0.150, 0.30)
    assert pct_L == 100
    assert pct_R == 100


def test_twist_in_place():
    # w = 1 rad/s -> v_L = -w*half, v_R = +w*half = 0.075 m/s
    pct_L, pct_R = twist_to_wheel_pcts(0.0, 1.0, 0.150, 0.30)
    assert pct_L == -25
    assert pct_R == 25


def test_twist_clamp():
    pct_L, pct_R = twist_to_wheel_pcts(1.0, 0.0, 0.150, 0.30)
    assert pct_L == 100
    assert pct_R == 100
    pct_L, pct_R = twist_to_wheel_pcts(-1.0, 0.0, 0.150, 0.30)
    assert pct_L == -100
    assert pct_R == -100


def test_motor_line_bytes():
    assert motor_line(40, -10) == b"M L=40 R=-10\n"


def test_integrate_pose_straight():
    # Both wheels equal -> straight line, no rotation.
    x, y, th = integrate_pose(0.0, 0.0, 0.0, 0.20, 0.20, 0.150, 1.0)
    assert abs(x - 0.20) < 1e-6
    assert abs(y) < 1e-6
    assert abs(th) < 1e-6


def test_integrate_pose_turn_in_place():
    # Differential speeds (equal magnitude, opposite signs) -> pure rotation,
    # no net translation.
    x, y, th = integrate_pose(0.0, 0.0, 0.0, -0.10, 0.10, 0.150, 1.0)
    assert math.hypot(x, y) < 1e-6
    # dTh = (0.20) / 0.150 = 4/3 rad.
    expected_th = 4.0 / 3.0
    assert abs(th - expected_th) < 1e-6


def test_integrate_pose_arc_left():
    # Right wheel faster than left -> robot curves to its left (+z angular).
    # From theta=0 this drives the robot in the +y direction.
    x, y, th = integrate_pose(0.0, 0.0, 0.0, 0.10, 0.20, 0.150, 1.0)
    # ds = 0.15, dTh = 0.10/0.150 = 2/3 rad.
    assert th > 0.0
    assert y > 0.0
    assert x > 0.0
    # Closed-form check.
    ds = 0.15
    d_th = 2.0 / 3.0
    radius = ds / d_th
    expected_x = radius * math.sin(d_th)
    expected_y = -radius * (math.cos(d_th) - 1.0)
    assert abs(x - expected_x) < 1e-9
    assert abs(y - expected_y) < 1e-9
    assert abs(th - d_th) < 1e-9
