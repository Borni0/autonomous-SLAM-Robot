# Top-level Makefile for the rover_ws workspace.
#
# Two sub-builds live here:
#   * ROS 2 Jazzy colcon build (src/...)
#   * PlatformIO ESP32 firmware build (firmware/)
#
# Common targets:
#   make build              # colcon build --symlink-install
#   make firmware-build     # pio run (no upload)
#   make firmware-upload    # pio run -t upload
#   make firmware-monitor   # pio device monitor
#   make run-unit           # pure-Python unit tests, no ROS needed
#   make check              # post-colcon-build sanity
#   make launch-mapping     # launch SLAM Toolbox + teleop
#   make launch-nav         # launch AMCL + Nav2
#   make install-udev       # install /etc/udev/rules.d/ rules
#   make clean              # rm -rf build install log firmware/.pio firmware/build

SHELL := /usr/bin/env bash
ROS_DISTRO ?= jazzy
WORKSPACE := $(shell pwd)
FIRMWARE_PORT ?= /dev/ttyUSB0
FIRMWARE_BAUD ?= 115200

.PHONY: help build test run-unit run-all-tests check \
    firmware-build firmware-upload firmware-monitor firmware-clean \
    launch-mapping launch-nav launch-desc launch-lidar launch-odom launch-motors \
    install-udev install-ros-deps clean distclean

help:
	@echo "Common targets:"
	@echo "  make build              - colcon build --symlink-install"
	@echo "  make firmware-build     - pio run (compile ESP32 firmware only)"
	@echo "  make firmware-upload    - pio run -t upload (writes to ESP32)"
	@echo "  make firmware-monitor   - pio device monitor (Ctrl+C to exit)"
	@echo "  make test               - colcon test"
	@echo "  make run-unit           - pure-Python unit tests, no ROS needed"
	@echo "  make check              - post-build sanity (needs install/setup.bash)"
	@echo "  make launch-mapping     - launch SLAM Toolbox + teleop"
	@echo "  make launch-nav         - launch AMCL + Nav2"
	@echo "  make launch-desc        - launch description + RViz only"
	@echo "  make launch-lidar       - launch LiDAR driver + RViz"
	@echo "  make launch-odom        - launch odometry + RViz + teleop (no ESP32)"
	@echo "  make launch-motors      - launch ESP32 bridge + RViz (no odometry)"
	@echo "  make install-udev       - install /etc/udev/rules.d/ rules"
	@echo "  make install-ros-deps   - rosdep install for src/"
	@echo "  make clean              - rm -rf build install log firmware/.pio firmware/build"

# --------------------------------------------------------------------------
# ROS 2 colcon build
# --------------------------------------------------------------------------
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

# --------------------------------------------------------------------------
# ESP32 PlatformIO firmware build
# --------------------------------------------------------------------------
firmware-build:
	@if ! command -v pio >/dev/null 2>&1; then \
	    echo "PlatformIO not installed. Install with: pip3 install platformio"; \
	    exit 1; \
	fi
	cd $(WORKSPACE)/firmware && pio run

firmware-upload:
	@if ! command -v pio >/dev/null 2>&1; then \
	    echo "PlatformIO not installed. Install with: pip3 install platformio"; \
	    exit 1; \
	fi
	@if [ ! -e $(FIRMWARE_PORT) ]; then \
	    echo "Serial port $(FIRMWARE_PORT) not found."; \
	    echo "Set FIRMWARE_PORT= make firmware-upload   e.g.  make firmware-upload FIRMWARE_PORT=/dev/ttyACM0"; \
	    exit 1; \
	fi
	cd $(WORKSPACE)/firmware && pio run -t upload --upload-port $(FIRMWARE_PORT)

firmware-monitor:
	@if ! command -v pio >/dev/null 2>&1; then \
	    echo "PlatformIO not installed. Install with: pip3 install platformio"; \
	    exit 1; \
	fi
	cd $(WORKSPACE)/firmware && pio device monitor -b $(FIRMWARE_BAUD)

firmware-clean:
	rm -rf $(WORKSPACE)/firmware/.pio $(WORKSPACE)/firmware/build

# --------------------------------------------------------------------------
# Launches
# --------------------------------------------------------------------------
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

# --------------------------------------------------------------------------
# Cleanup
# --------------------------------------------------------------------------
clean:
	rm -rf build install log
	rm -rf firmware/.pio firmware/build

distclean: clean
	rm -rf src/*/build src/*/install src/*/log