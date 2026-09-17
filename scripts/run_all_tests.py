#!/usr/bin/env python3
"""Discover and run all `test_*` functions in the rover_ws packages.

Runs WITHOUT ROS 2 installed — exercises only pure-Python helpers
(e.g. rover_hardware/_kinematics.py, rover_compass/heading_parser.py).
"""
from __future__ import annotations

import importlib.util
import os
import sys
import traceback

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
ROOT = os.path.abspath(ROOT)

total = 0
failed = 0

if not os.path.isdir(ROOT):
    print(f'ERROR: src/ not found at {ROOT}', file=sys.stderr)
    sys.exit(2)

for pkg in sorted(os.listdir(ROOT)):
    pkg_root = os.path.join(ROOT, pkg)
    test_dir = os.path.join(pkg_root, 'test')
    if not os.path.isdir(test_dir):
        continue
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)
    for fname in sorted(os.listdir(test_dir)):
        if not fname.startswith('test_') or not fname.endswith('.py'):
            continue
        path = os.path.join(test_dir, fname)
        mod_name = f'{pkg}__test__{fname[:-3]}'
        spec = importlib.util.spec_from_file_location(mod_name, path)
        mod = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(mod)
        except Exception as exc:
            print(f'SKIP  {pkg}/{fname} (load error: {type(exc).__name__}: {exc})')
            continue
        tests = [n for n in dir(mod) if n.startswith('test_')]
        if not tests:
            continue
        for name in tests:
            total += 1
            try:
                getattr(mod, name)()
                print(f'  PASS  {pkg}/{fname}::{name}')
            except AssertionError as exc:
                failed += 1
                print(f'  FAIL  {pkg}/{fname}::{name}: {exc}')
            except Exception as exc:
                failed += 1
                print(f'  ERROR {pkg}/{fname}::{name}: {type(exc).__name__}: {exc}')
                traceback.print_exc()

print()
print(f'{total - failed}/{total} tests passed')
sys.exit(0 if failed == 0 else 1)
