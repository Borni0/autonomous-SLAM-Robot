"""Pure-Python parsers for the ESP32 firmware's telemetry protocol.

Kept importable for unit tests (no rclpy dependency).

Firmware (src/telemetry.cpp) emits two line-oriented records:

    P x=<m> y=<m> theta=<rad> vL=<m/s> vR=<m/s> age=<ms>
    T bat=<volts> estop=<0|1> tof=<mm> mode=<0|1|2>

These helpers parse each line into a typed dataclass. Lines that fail
to parse return ``None`` so callers can ignore them silently.

The protocol is whitespace-tolerant: ``P x=1.0`` and ``P x=1.0   y=2.0``
both work. Unknown fields are silently ignored, so the firmware can add
new fields without breaking older Pi-side code.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PoseRecord:
    """One firmware-side ``P`` line."""
    x: float          # metres
    y: float          # metres
    theta: float      # radians, CCW positive
    v_left_mps: float
    v_right_mps: float
    age_ms: int


@dataclass(frozen=True)
class TelemetryRecord:
    """One firmware-side ``T`` line."""
    battery_volts: float
    estop: bool
    tof_mm: int
    mode: int         # 0 = ROS2, 1 = AUTO, 2 = FAULT


def _fields_after_prefix(line: str, prefix: str) -> Optional[dict]:
    """Common parser: read ``key=value`` tokens following a single-letter
    prefix (``P`` or ``T``). Leading whitespace is ignored.
    Returns ``None`` if the prefix is missing."""
    if not line:
        return None
    stripped = line.lstrip()
    if not stripped.startswith(prefix):
        return None
    fields = {}
    for token in stripped.split():
        if '=' not in token:
            continue
        k, v = token.split('=', 1)
        fields[k] = v
    return fields


def parse_pose(line: str) -> Optional[PoseRecord]:
    """Parse a ``P`` line. Returns ``None`` on malformed input."""
    fields = _fields_after_prefix(line, 'P')
    if fields is None:
        return None
    try:
        return PoseRecord(
            x=float(fields['x']),
            y=float(fields['y']),
            theta=float(fields['theta']),
            v_left_mps=float(fields['vL']),
            v_right_mps=float(fields['vR']),
            age_ms=int(fields['age']),
        )
    except (KeyError, ValueError):
        return None


def parse_telemetry(line: str) -> Optional[TelemetryRecord]:
    """Parse a ``T`` line. Returns ``None`` on malformed input."""
    fields = _fields_after_prefix(line, 'T')
    if fields is None:
        return None
    try:
        return TelemetryRecord(
            battery_volts=float(fields['bat']),
            estop=(int(fields['estop']) != 0),
            tof_mm=int(fields['tof']),
            mode=int(fields['mode']),
        )
    except (KeyError, ValueError):
        return None
