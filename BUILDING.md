# Building the rover_ws workspace on a Raspberry Pi 5

End-to-end recipe for an **Ubuntu 24.04 LTS (Noble)** install on a
Raspberry Pi 5 (4 GB or 8 GB). Targets **ROS 2 Jazzy Jalisco**.

> If you are on a Pi 4B, this recipe is identical except the
> `sudo apt install linux-modules-extra-raspi` line is unnecessary
> and the `usb_max_current_enable=1` boot/config tweak is harmless.

---

## 0. Hardware checklist before you power up

- [ ] **Pi 5 PSU: 5 V / 5 A USB-C PD.** The official 27 W PSU is
      strongly recommended. A 3 A phone charger will brownout the
      moment the BTS7960s draw current spikes from the motors, and
      ROS 2 nodes will crash mid-mission.
- [ ] **3S LiPo (11.1 V nominal) → 5 V buck** rated for **>= 6 A**
      continuous (e.g. a D24V22F5 or Mean Well NSD05-5S5). Feed the
      Pi 5's USB-C PD input from this buck so the rover is genuinely
      untethered. Do NOT power the Pi 5 from your laptop USB.
- [ ] **ESP32-S3** connected to one of the Pi 5's USB-A (or USB-3.0
      via the official Pi 5 USB-C-PD-to-USB-A dongle) ports.
- [ ] **A3 LiDAR** on its own USB-A port — preferably the blue
      USB-3.0 port for stability.
- [ ] `sudo usermod -aG dialout $USER` (log out / log back in).

---

## 1. Install Ubuntu 24.04 LTS

Use the official Raspberry Pi Imager (https://www.raspberrypi.com/software/)
and write **Ubuntu 24.04 LTS desktop** (or server, if you want headless)
to a microSD card or USB SSD. On first boot:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget build-essential cmake python3-pip \
    python3-colcon-common-extensions python3-rosdep python3-vcstool
```

Enable the UART overlay if you ever plan to wire the A3 LiDAR to GPIO
instead of USB (not needed for the current build, but good hygiene):

```bash
sudo raspi-config nonint enable_uart     # only if using GPIO UART
```

---

## 2. Install ROS 2 Jazzy

ROS 2 Jazzy is in the official Ubuntu 24.04 repos and on the ROS 2
apt server for **arm64**:

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# `ros-jazzy-desktop` is large (~1.5 GB) and includes RViz; use
# `ros-jazzy-ros-base` if you want a slimmer headless install.
sudo apt install -y ros-jazzy-desktop \
    ros-jazzy-rviz2 ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
    ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox ros-jazzy-amcl \
    ros-jazzy-robot-localization \
    ros-jazzy-ydlidar-ros2-driver \
    ros-jazzy-teleop-twist-keyboard \
    python3-serial minicom

pip3 install pyserial

sudo rosdep init
rosdep update

# Source ROS 2 in every shell.
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 --help   # smoke check
```

If `sudo rosdep init` complains it already exists, ignore it — just run
`rosdep update`.

---

## 3. Install the udev rules from this workspace

```bash
cd ~/rover_ws
sudo cp docs/99-rover-esp32.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the ESP32 and the A3 LiDAR. They should now appear at:

```
/dev/rover_esp32        # ESP32-S3 (USB CDC)
/dev/rover_lidar        # (if your A3 enumerates as a recognized VID; else /dev/ttyUSB*)
```

Verify:

```bash
ls -l /dev/rover_esp32
lsusb
```

---

## 4. Drop the workspace in

The workspace is a single folder containing both the ROS 2 source
tree (`src/`) and the ESP32 firmware (`firmware/`).

From your workstation:

```bash
scp -r /home/borni/rover_ws ubuntu@<pi-ip>:~/
```

(Or `git clone` if you've pushed the tree to a remote.)

On the Pi:

```bash
cd ~
test -d rover_ws || (echo "rover_ws not found" && exit 1)
cd ~/rover_ws
```

---

## 5. Resolve dependencies and build

```bash
cd ~/rover_ws
sudo rosdep install -y --from-paths src --ignore-src --rosdistro=jazzy
source /opt/ros/jazzy/setup.bash
make build              # colcon build --symlink-install
source install/setup.bash
echo "source ~/rover_ws/install/setup.bash" >> ~/.bashrc
```

Or use the Makefile directly:

```bash
make install-ros-deps
make build
source install/setup.bash
```

Expected packages in `ros2 pkg list | grep rover`:

```
rover_bringup
rover_compass
rover_description
rover_hardware
rover_lidar
rover_localization
rover_navigation
```

---

## 6. Verify (in this order — mirrors spec section 29)

| Step | Command | What you should see |
|---|---|---|
| 6.0 | `make firmware-build && make firmware-upload` | PlatformIO compiles the ESP32 firmware and flashes it via USB |
| 6.1 | `make launch-desc` (or `ros2 launch rover_bringup rover.launch.py mode:=just_description`) | RViz with robot model + TF arrows for base_link, lidar_link, compass_link, wheel_*_link |
| 6.2 | `make launch-lidar` + `ros2 topic echo /scan --once` | LaserScan message with non-empty ranges |
| 6.3 | `make launch-odom` + drive (wheels lifted) | `/odom` updates; `tf2_tools view_frames` shows `odom → base_link` |
| 6.4 | `make launch-motors` then `source ~/rover_ws/src/rover_bringup/scripts/drive_once.sh && drive_once 0.1` | Wheels spin for one tick; `esp32_bridge` logs `[esp32] MANUAL L=… R=…` |
| 6.5 | `make launch-mapping` + teleop | `/map` builds |
| 6.6 | `ros2 run nav2_map_server map_saver_cli -f ~/rover_ws/maps/indoor_map` | `indoor_map.yaml` + `indoor_map.pgm` written |
| 6.7 | `make launch-nav` (or `ros2 launch rover_bringup rover.launch.py mode:=navigation world:=/home/$USER/rover_ws/maps/indoor_map.yaml`) | RViz "2D Pose Estimate" then "Nav2 Goal" makes the rover drive |

Soft e-stop works anywhere in this flow:

```bash
ros2 run rover_hardware estop_cli stop       # freeze
ros2 run rover_hardware estop_cli release    # resume
```

---

## 7. Pi 5 specific gotchas

- **Undervoltage = silent ROS 2 crashes.** If you see
  `rcu_sched self-detected stall on CPU` or
  `systemd-logind: Failed to start user slice`, check
  `dmesg | grep -i 'voltage\|current'`. The fix is a beefier PSU
  or buck converter, not a software workaround.
- **Swap to USB-SSD before mapping.** The default microSD card will
  throttle during `colcon build` and during long SLAM sessions.
  Use a USB-3 SSD with the Pi 5's official SSD HAT, or accept
  throttling.
- **`zram` swap.** On a 4 GB Pi 5 add `zram` to avoid OOM kills
  during colcon:

  ```bash
  sudo apt install -y systemd-zram-generator
  echo -e "[zram0]\ncompression-algorithm=zstd\nzram-size=ram*2\nfs-type=swap" \
      | sudo tee /etc/systemd/zram-generator.conf
  sudo systemctl daemon-reload
  sudo systemctl start systemd-zram-setup@zram0.service
  ```
- **CPU governor.** Force performance during a build to roughly
  halve the colcon time:

  ```bash
  echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
  ```
- **No /dev/ttyAMA0 unless you wired the LiDAR to GPIO.** The
  A3 in this workspace is assumed to be on USB. If you switch to
  GPIO UART, edit `rover_lidar/config/ydlidar.yaml`.

---

## 8. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros2: command not found` | New shell, no source | `source /opt/ros/jazzy/setup.bash` |
| `Package 'rover_xxx' not found` | Not built | `colcon build --symlink-install` then `source install/setup.bash` |
| `/dev/rover_esp32: Permission denied` | udev rule didn't fire | `sudo chmod 666 /dev/ttyUSB0` as fallback, then investigate `udevadm monitor` |
| `ydlidar: open failed` | Wrong port | `ls /dev/ttyUSB*` and edit `rover_lidar/config/ydlidar.yaml` |
| `/scan` empty | Driver not receiving motor | Check that the A3 is spinning (you'll hear it) and that `range_min`/`range_max` in `ydlidar.yaml` match the A3's spec |
| `/odom` not advancing | `diff_drive_odometry` not running, or no `/cmd_vel` | `ros2 node list`, `ros2 topic hz /cmd_vel` |
| RViz shows wheels detached from chassis | Normal — the wheels are continuous joints with no `joint_state_publisher` driving them yet | Add `joint_state_publisher_gui` or accept the default zero-angle pose |
| AMCL `no laser scan received` | `/scan` topic missing or wrong frame | `ros2 topic echo /scan --once`; check `ydlidar.yaml`'s `frame_id` |
| Nav2 controller says `transform map→odom not found` | AMCL hasn't localized yet | Click "2D Pose Estimate" in RViz before "Nav2 Goal" |
