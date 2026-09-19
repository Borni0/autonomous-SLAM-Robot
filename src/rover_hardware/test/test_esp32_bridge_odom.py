"""Unit tests for the ESP32-bridge odometry accumulator.

Locks down the kinematic contract between
``firmware/src/odometry.cpp`` and the Pi-side
``rover_hardware._odom_accumulator.OdomAccumulator``.

Run on the dev machine:

    python3 scripts/run_all_tests.py
"""
from __future__ import annotations

import math

from rover_hardware._firmware_proto import PoseRecord
from rover_hardware._kinematics import integrate_pose, twist_to_wheel_mps
from rover_hardware._odom_accumulator import OdomAccumulator, quat_from_yaw


WHEEL_BASE = 0.150
V_MAX = 0.30


def _record(x=0.0, y=0.0, theta=0.0, vL=0.0, vR=0.0, age=20) -> PoseRecord:
    return PoseRecord(x=x, y=y, theta=theta,
                      v_left_mps=vL, v_right_mps=vR, age_ms=age)


def test_adopt_pose_replaces_state() -> None:
    acc = OdomAccumulator(wheel_base=WHEEL_BASE, now_ns=100_000_000)
    acc.integrate_dead_reckoning(
        v_cmd=0.10, w_cmd=0.0,
        now_ns=120_000_000,
        cmd_is_zero=False,
        pose_stale_ns=250_000_000,
    )
    assert acc.snapshot().x > 0.0  # we moved forward

    rec = _record(x=1.0, y=-0.5, theta=0.1, vL=0.20, vR=0.21)
    acc.update_from_pose(rec, now_ns=200_000_000)
    snap = acc.snapshot()
    assert snap.x == 1.0
    assert snap.y == -0.5
    assert snap.theta == 0.1
    assert snap.v_left_mps == 0.20
    assert snap.v_right_mps == 0.21
    assert snap.last_fresh_pose_ns == 200_000_000


def test_reset_clears_pose_but_keeps_last_fresh_pose_marker() -> None:
    """reset() must zero x/y/theta but NOT the last_fresh_pose_ns
    marker, otherwise the fallback's freshness check has no anchor."""
    acc = OdomAccumulator(wheel_base=WHEEL_BASE)
    acc.update_from_pose(_record(x=2.0, y=3.0, theta=0.5), now_ns=10)
    acc.reset(now_ns=20)
    snap = acc.snapshot()
    assert snap.x == 0.0 and snap.y == 0.0 and snap.theta == 0.0
    assert snap.last_fresh_pose_ns == 10


def test_fallback_disabled_when_pose_is_fresh() -> None:
    acc = OdomAccumulator(wheel_base=WHEEL_BASE)
    acc.update_from_pose(_record(x=0.5, y=0.0, theta=0.0), now_ns=100)
    fired = acc.integrate_dead_reckoning(
        v_cmd=0.30, w_cmd=0.0,
        now_ns=200_000_000,  # well within pose_stale_ns
        cmd_is_zero=False,
        pose_stale_ns=250_000_000,
    )
    assert fired is False
    assert acc.snapshot().x == 0.5
    assert acc.snapshot().y == 0.0


def test_fallback_fires_when_cmd_is_zero_even_if_pose_fresh() -> None:
    """cmd_is_zero must always trigger fallback so /odom reflects the brake."""
    acc = OdomAccumulator(wheel_base=WHEEL_BASE)
    acc.update_from_pose(_record(x=0.7, y=0.0, theta=0.0), now_ns=100)
    fired = acc.integrate_dead_reckoning(
        v_cmd=0.30, w_cmd=0.0,
        now_ns=200_000_000,
        cmd_is_zero=True,
        pose_stale_ns=250_000_000,
    )
    assert fired is True
    assert acc.snapshot().v_left_mps == 0.0
    assert acc.snapshot().v_right_mps == 0.0


def test_forward_motion_matches_expected_distance() -> None:
    """Drive forward at 0.30 m/s for 1 s -> x ~ 0.30 m, y ~ 0.

    The 50 ticks of 20 ms each mirror the firmware's CONTROL_LOOP_MS."""
    acc = OdomAccumulator(wheel_base=WHEEL_BASE, now_ns=0)
    now = 0
    dt_ns = 20_000_000
    for _ in range(50):
        now += dt_ns
        acc.integrate_dead_reckoning(
            v_cmd=V_MAX, w_cmd=0.0,
            now_ns=now,
            cmd_is_zero=False,
            pose_stale_ns=10_000_000,  # force fallback each tick
        )
    snap = acc.snapshot()
    assert abs(snap.x - 0.30) < 1e-3
    assert abs(snap.y) < 1e-3
    assert abs(snap.theta) < 1e-6
    assert abs(snap.v_left_mps - V_MAX) < 1e-6
    assert abs(snap.v_right_mps - V_MAX) < 1e-6


def test_in_place_rotation_matches_expected_angle() -> None:
    """Pick w_cmd so |v_L| = |v_R| = V_MAX -> in-place rotation."""
    w_cmd = 2.0 * V_MAX / WHEEL_BASE  # = 4.0 rad/s
    acc = OdomAccumulator(wheel_base=WHEEL_BASE, now_ns=0)
    now = 0
    dt_ns = 20_000_000
    for _ in range(50):  # 1 s
        now += dt_ns
        acc.integrate_dead_reckoning(
            v_cmd=0.0, w_cmd=w_cmd,
            now_ns=now,
            cmd_is_zero=False,
            pose_stale_ns=10_000_000,
        )
    theta_norm = math.atan2(math.sin(acc.snapshot().theta),
                            math.cos(acc.snapshot().theta))
    expected = math.atan2(math.sin(4.0), math.cos(4.0))
    assert abs(theta_norm - expected) < 1e-4
    assert abs(acc.snapshot().x) < 1e-3
    assert abs(acc.snapshot().y) < 1e-3


def test_random_walk_matches_reference_integrate_pose() -> None:
    """Compare the accumulator against an independent integrate_pose()
    reference driven with the same twist sequence and dt."""
    acc = OdomAccumulator(wheel_base=WHEEL_BASE, now_ns=0)
    ref_x, ref_y, ref_theta = 0.0, 0.0, 0.0
    now = 0
    dt_ns = 20_000_000
    for k in range(25):
        now += dt_ns
        v_cmd = 0.10 + 0.005 * k
        w_cmd = 0.05 - 0.002 * k
        vL, vR = twist_to_wheel_mps(v_cmd, w_cmd, WHEEL_BASE)
        ref_x, ref_y, ref_theta = integrate_pose(
            ref_x, ref_y, ref_theta,
            vL, vR, WHEEL_BASE, 0.020,
        )
        acc.integrate_dead_reckoning(
            v_cmd=v_cmd, w_cmd=w_cmd,
            now_ns=now,
            cmd_is_zero=False,
            pose_stale_ns=10_000_000,
        )
    snap = acc.snapshot()
    assert abs(snap.x - ref_x) < 1e-6
    assert abs(snap.y - ref_y) < 1e-6
    # Theta may differ by 2*pi*N; normalize.
    dtheta = math.atan2(math.sin(snap.theta - ref_theta),
                        math.cos(snap.theta - ref_theta))
    assert abs(dtheta) < 1e-6


def test_zero_yaw_is_identity() -> None:
    assert quat_from_yaw(0.0) == (0.0, 0.0, 0.0, 1.0)


def test_quarter_turn() -> None:
    x, y, z, w = quat_from_yaw(math.pi / 2.0)
    assert x == 0.0 and y == 0.0
    s = math.sqrt(2.0) / 2.0
    assert abs(z - s) < 1e-12
    assert abs(w - s) < 1e-12


def test_negative_yaw_inverts_z() -> None:
    _, _, z1, w1 = quat_from_yaw(0.5)
    _, _, z2, w2 = quat_from_yaw(-0.5)
    assert abs(z1 + z2) < 1e-12
    assert abs(w1 - w2) < 1e-12
