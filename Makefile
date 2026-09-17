# Top-level Makefile for the rover_ws workspace.
#
# Assumes you have sourced /opt/ros/jazzy/setup.bash (or have ROS 2
# Jazzy installed and on your PATH).
#
# Most common targets:
#   make build               # colcon build --symlink-install
#   make test                # colcon test
#   make run-unit            # standalone unit tests, no ROS needed
#   make launch-mapping      # rover_bringup/launch/rover.launch.py mode:=mapping
#   make launch-nav          # rover_bringup/launch/rover.launch.py mode:=navigation
#   make launch-desc         # rover_bringup/launch/rover.launch.py mode:=just_description
#   make clean               # rm -rf build install log

SHELL := /usr/bin/env bash
ROS_DISTRO ?= jazzy
WORKSPACE := $(shell pwd)

.PHONY: help build test run-unit run-all-tests lint check \
    launch-mapping launch-nav launch-desc launch-lidar launch-odom launch-motors \
    install-udev install-ros-deps clean distclean

help:
	@echo "Common targets:"
	@echo "  make build              - colcon build --symlink-install"
	@echo "  make test               - colcon test (incl. launch tests)"
	@echo "  make run-unit           - run pure-Python unit tests (no ROS needed)"
	@echo "  make check              - post-build sanity (needs colcon-built install/)"
	@echo "  make launch-mapping     - launch in mapping mode"
	@echo "  make launch-nav         - launch in navigation mode (needs indoor_map.yaml)"
	@echo "  make launch-desc        - launch description only (no lidar/hw/nav)"
	@echo "  make launch-lidar       - launch LiDAR driver + RViz"
	@echo "  make launch-odom        - launch odometry + RViz + teleop (no ESP32)"
	@echo "  make launch-motors      - launch ESP32 bridge + RViz (no odometry)"
	@echo "  make install-udev       - copy docs/*.rules to /etc/udev/rules.d/"
	@echo "  make install-ros-deps   - rosdep install for the workspace"
	@echo "  make clean              - rm -rf build install log"

build:
	@if ! command -v colcon >/dev/null 2>&1; then \
	    echo "colcon not found. Did you source /opt/ros/$(ROS_DISTRO)/setup.bash?"; \
	    exit 1; \
	fi
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

test:
	colcon test --event-handlers console_direct+
	colcon test-result --verbose

run-unit:
	@echo "Running pure-Python unit tests (no ROS required)..."
	@python3 $(WORKSPACE)/scripts/run_all_tests.py

run-all-tests: run-unit

check:
	@if [ ! -f install/setup.bash ]; then \
	    echo "install/setup.bash missing - run 'make build' first"; \
	    exit 1; \
	fi
	./scripts/bringup_check.sh

install-ros-deps:
	@if ! command -v rosdep >/dev/null 2>&1; then \
	    echo "rosdep not found. Install with: sudo apt install -y python3-rosdep"; \
	    exit 1; \
	fi
	rosdep install -y --from-paths src --ignore-src --rosdistro=$(ROS_DISTRO)

install-udev:
	sudo cp $(WORKSPACE)/docs/99-rover-esp32.rules   /etc/udev/rules.d/
	sudo cp $(WORKSPACE)/docs/99-rover-a3-lidar.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	sudo udevadm trigger
	@echo "Replug ESP32 + A3. They should appear at /dev/rover_esp32 and /dev/rover_a3"

launch-mapping:
	ros2 launch rover_bringup rover.launch.py mode:=mapping

launch-nav:
	ros2 launch rover_bringup rover.launch.py mode:=navigation

launch-desc:
	ros2 launch rover_bringup rover.launch.py mode:=just_description

launch-lidar:
	ros2 launch rover_bringup test_lidar.launch.py

launch-odom:
	ros2 launch rover_bringup test_odom.launch.py

launch-motors:
	ros2 launch rover_bringup test_motors.launch.py

clean:
	rm -rf build install log

distclean: clean
	rm -rf src/*/build src/*/install src/*/log
