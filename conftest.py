"""Top-level pytest config.

Adds ``src/<pkg>`` directories to ``sys.path`` so the unit tests can
import their own packages without ``colcon build`` having been run.
The same trick is used by ament's ``add_to_path`` at runtime; here we
do it statically so a developer can run ``pytest`` straight from a
fresh checkout.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
for sub in ("rover_hardware", "rover_compass"):
    p = SRC / sub
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
