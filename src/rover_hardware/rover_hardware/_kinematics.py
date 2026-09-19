"""Pure-Python kinematics helpers, kept importable for unit tests.

The protocol on the ESP32 side is documented in
``firmware/src/command.cpp`` and ``firmware/src/main.cpp``.

A single line `M L=<int8> R=<int8>\\n` accepts wheel speed percentages
in [-100, 100]. The firmware internally maps 100% to 0.30 m/s
(see percentToLinearMps in firmware/src/main.cpp).
"""
from __future__ import annotations


def twist_to_wheel_pcts(
    v_mps: float, w_radps: float, wheel_base: float, v_max_mps: float
) -> tuple[int, int]:
    """Differential-drive inverse kinematics.

    Parameters
    ----------
    v_mps : float
        Linear velocity (m/s), +x forward.
    w_radps : float
        Angular velocity (rad/s), +z counter-clockwise.
    wheel_base : float
        Distance between the two wheels (m).
    v_max_mps : float
        Wheel linear speed that corresponds to 100% PWM.

    Returns
    -------
    (pct_left, pct_right) : tuple[int, int]
        Integers in [-100, 100].
    """
    if v_max_mps <= 0.0:
        raise ValueError("v_max_mps must be > 0")
    v_L, v_R = twist_to_wheel_mps(v_mps, w_radps, wheel_base)
    pct_L = int(round(100.0 * v_L / v_max_mps))
    pct_R = int(round(100.0 * v_R / v_max_mps))
    pct_L = max(-100, min(100, pct_L))
    pct_R = max(-100, min(100, pct_R))
    return pct_L, pct_R


def twist_to_wheel_mps(
    v_mps: float, w_radps: float, wheel_base: float
) -> tuple[float, float]:
    """Forward differential-drive kinematics: twist -> wheel ground speeds.

    Used both by ``twist_to_wheel_pcts`` (production) and by
    ``OdomAccumulator.integrate_dead_reckoning`` + the random-walk
    test, so the formula lives in exactly one place.
    """
    half = wheel_base / 2.0
    return v_mps - w_radps * half, v_mps + w_radps * half


def motor_line(pct_left: int, pct_right: int) -> bytes:
    """Serialise the `M L=... R=...` command line for the ESP32."""
    return f"M L={pct_left} R={pct_right}\n".encode("utf-8")


def integrate_pose(
    x: float, y: float, theta: float,
    v_L_mps: float, v_R_mps: float, wheel_base: float, dt: float
) -> tuple[float, float, float]:
    """Exact-arc differential-drive pose integration.

    Mirrors ``Odometry::update`` in ``firmware/src/odometry.cpp`` so
    dead-reckoning here matches the firmware's pose estimates
    bit-for-bit (when given the same wheel velocities).
    """
    import math

    dL = v_L_mps * dt
    dR = v_R_mps * dt
    ds = 0.5 * (dR + dL)
    dTh = (dR - dL) / wheel_base

    if abs(dTh) < 1e-6:
        x += ds * math.cos(theta)
        y += ds * math.sin(theta)
    else:
        theta_new = theta + dTh
        radius = ds / dTh
        x += radius * (math.sin(theta_new) - math.sin(theta))
        y += -radius * (math.cos(theta_new) - math.cos(theta))
        theta = theta_new

    # Normalise theta to [-pi, pi]
    theta = math.atan2(math.sin(theta), math.cos(theta))
    return x, y, theta
