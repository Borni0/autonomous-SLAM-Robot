"""Pure-Python heading parser for NMEA-style compass sentences.

Kept ROS-free so it can be unit-tested without rclpy installed.
"""
from __future__ import annotations


def parse_heading(line: str):
    """Extract a compass heading (degrees, 0..360) from a text line.

    Accepts:

    * NMEA-style sentences: ``$CHARR,HEADING,...`` with optional
      trailing ``*XX`` checksum, e.g. ``$CHARR,123.45,5.2,W*7A``.
    * Bare numbers: ``"180"``.
    * Comma-separated fields where the first parseable float in the
      range [0, 360) is the heading.

    Returns ``None`` when no valid heading can be parsed.
    """
    if not line:
        return None
    if '*' in line:
        line = line.split('*', 1)[0]
    if line.startswith('$'):
        line = line[1:]
    parts = [p for p in line.split(',') if p]
    if not parts:
        return None
    for p in parts:
        try:
            val = float(p)
        except ValueError:
            continue
        if 0.0 <= val < 360.0:
            return val
    return None
