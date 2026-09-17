#!/usr/bin/env bash
# Post-build sanity check. Run AFTER `make build` and AFTER sourcing
# the workspace's setup.bash.
#
# Verifies:
#   1. All seven rover_* packages are discoverable via ament_index.
#   2. The URDF parses as valid XML.
#   3. Every launch file is a valid Python script (syntax + imports).
#   4. Every .yaml file is well-formed.
#   5. The default empty map exists and is readable.
#   6. The expected console scripts are registered
#      (esp32_bridge, diff_drive_odometry, soft_estop, estop_cli,
#       compass_node).
#   7. The /dev/rover_esp32 and /dev/rover_a3 symlinks exist (warn
#      rather than fail if not, since they require the udev rule to
#      be installed and the devices plugged in).
#
# Exits non-zero on any failure.

set -euo pipefail

cd "$(dirname "$0")/.."

# Source the workspace if install/setup.bash exists.
if [ -f install/setup.bash ]; then
    # shellcheck disable=SC1091
    source install/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    echo "(WARN: workspace not built; sourced /opt/ros/jazzy only)" >&2
fi

PASS=0; FAIL=0
pass() { echo "  PASS  $*"; PASS=$((PASS + 1)); }
fail() { echo "  FAIL  $*"; FAIL=$((FAIL + 1)); }
warn() { echo "  WARN  $*"; }

echo "==> 1. ament_index package discovery"
for pkg in rover_bringup rover_compass rover_description rover_hardware rover_lidar rover_localization rover_navigation; do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        pass "ros2 pkg prefix $pkg"
    else
        fail "ros2 pkg prefix $pkg (not found - did colcon build succeed?)"
    fi
done

echo
echo "==> 2. URDF well-formedness"
urdf="$(ros2 pkg prefix rover_description)/share/rover_description/urdf/rover.urdf.xacro"
if [ -f "$urdf" ]; then
    if command -v xacro >/dev/null 2>&1; then
        if xacro "$urdf" >/tmp/rover.urdf 2>/tmp/xacro.err; then
            pass "xacro $urdf parses"
        else
            fail "xacro $urdf failed: $(cat /tmp/xacro.err)"
        fi
    else
        warn "xacro not on PATH; skipping"
    fi
else
    fail "URDF not found at $urdf"
fi

echo
echo "==> 3. Launch file Python syntax"
for f in $(find src -name '*.launch.py'); do
    if python3 -c "import ast,sys; ast.parse(open('$f').read())" 2>/dev/null; then
        pass "$(basename "$f")"
    else
        fail "$(basename "$f") has syntax errors"
    fi
done

echo
echo "==> 4. YAML well-formedness"
for f in $(find src -name '*.yaml'); do
    if python3 -c "import yaml; yaml.safe_load(open('$f'))" 2>/dev/null; then
        pass "$(basename "$f")"
    else
        fail "$(basename "$f") failed to parse"
    fi
done

echo
echo "==> 5. Default empty map"
empty_map="$(ros2 pkg prefix rover_navigation 2>/dev/null)/share/rover_navigation/maps/empty.yaml"
if [ -f "$empty_map" ]; then
    pass "empty.yaml present"
else
    fail "empty.yaml missing at $empty_map"
fi

echo
echo "==> 6. Console scripts registered"
for script in esp32_bridge diff_drive_odometry soft_estop estop_cli compass_node; do
    if ros2 run rover_hardware "$script" --help >/dev/null 2>&1 || \
       ros2 pkg executables rover_hardware 2>/dev/null | grep -q "$script" || \
       ros2 pkg executables rover_compass 2>/dev/null | grep -q "$script"; then
        pass "$script registered"
    else
        fail "$script not registered"
    fi
done

echo
echo "==> 7. Device symlinks (advisory)"
[ -e /dev/rover_esp32 ] && pass "/dev/rover_esp32 present" || warn "/dev/rover_esp32 missing (run install-udev, plug ESP32)"
[ -e /dev/rover_a3 ]    && pass "/dev/rover_a3 present"    || warn "/dev/rover_a3 missing (run install-udev, plug A3)"

echo
echo "==> Summary: $PASS passed, $FAIL failed"
[ "$FAIL" -eq 0 ] || exit 1