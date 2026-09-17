# Raspberry Pi 5 hardware integration for the rover

This addendum is for the **Raspberry Pi 5** specifically. The Pi 5
brings faster I/O (PCIe Gen 2, USB 3.0) and a much more aggressive
power budget than the Pi 4, which means **undervoltage brownouts are
the #1 cause of "weird" failures** during a mission. Treat this
document as required reading before you power the rover.

---

## Power tree

```
         +------------------------------+
         |     3S LiPo (11.1 V nom)     |
         +-------------+----------------+
                       |
                       v
              +------------------+        +--------+
              |  5 V / >= 6 A    | -----> |  Pi 5  |  via USB-C PD
              |  buck regulator  |        +--------+
              |  (e.g. D24V22F5)|              |
              +------------------+              | USB-A
                                               v
                                       +---------------+
                                       |  ESP32-S3     |
                                       +---------------+
                                       |  YDLIDAR A3   | (separate USB-A)
                                       +---------------+
                                       |  BTS7960 x2   |  <-- fed DIRECTLY
                                       +---------------+      from LiPo
```

* **Do NOT** power the Pi 5 from your laptop USB during a run.
* **Do NOT** share the buck regulator's 5 V rail with the motors.
  The BTS7960s can draw 25 A peak and will collapse any 5 V rail
  they touch. Power them straight from the 3S LiPo through a
  separate high-current buck (or straight off the pack via a
  fuse/TVS).
* **Use the official Pi 5 27 W PSU** for bench testing with USB
  power. For battery runs, any USB-C PD source that advertises
  5 V / 5 A is acceptable (Pi 5 negotiates 5 V only, not 9/12/15/20).

---

## Undervoltage watchdog

Pi 5 logs to `dmesg` when its USB-C current budget is exceeded.
Useful commands:

```bash
# Kernel ring buffer, watch for "voltage" or "current":
sudo dmesg -w | grep -i 'voltage\|current\|over-current'

# Polled status:
vcgencmd pmic_read_adc EXT5V_V  # input voltage on the USB-C rail
vcgencmd get_throttled          # bitmask, see below
```

The `get_throttled` bitmask decodes as:

| Bit | Meaning |
|---|---|
| 0  | Undervoltage detected |
| 1  | ARM frequency capped due to undervoltage |
| 2  | Currently throttled |
| 3  | Soft temperature limit |
| 16 | Undervoltage has occurred since boot |
| 17 | Throttling has occurred since boot |

If any of bits 0/1/2 are set *now* or since boot, the Pi is not
getting enough power and ROS 2 nodes will eventually crash.

---

## Storage

`colcon build --symlink-install` on ROS 2 Jazzy desktop takes ~30
minutes on a Pi 5 from a microSD card; bring it down to ~6 minutes
with a USB-3 SSD. Recommended:

* **Raspberry Pi SSD HAT** with a name-brand NVMe (e.g. Samsung
  970 EVO Plus 250 GB, or a WD SN570). Power via the Pi 5's PCIe
  FFC, boot from the SSD.
* **OR** a USB-3 enclosure with any SATA/NVMe SSD. Plug into a
  blue USB-3 port.

If you must stick with microSD: use an **A2-rated** card
(Samsung EVO Select, SanDisk Extreme). The Pi 5 will throttle
during `colcon build` on cheap cards, doubling the build time.

---

## Network

For SSH during bringup, prefer **wired Ethernet** to the Pi 5's
onboard RJ45. Wi-Fi works but introduces latency on RViz topic
streams.

---

## USB topology

The Pi 5 has two USB-3 ports (blue, internal) and two USB-2 ports
(internal). Plug the LiDAR into a **USB-3 port** — the A3 spins at
10 Hz with 4 KB/s, well within USB-2 bandwidth, but USB-3 has
better noise immunity on the VBUS line when the BTS7960s are
active.

Plug the ESP32 into **any port** — its traffic is ~1 KB/s.

Avoid plugging both devices into the same USB-2 root hub if you
can; on Pi 5 the two USB-2 ports share a hub.

---

## GPIO UART (not used by default)

If you later decide to wire the A3 to GPIO UART instead of USB,
the Pi 5 uses **GPIO 14 (TXD) / GPIO 15 (RXD)** with the
`enable_uart=1` overlay in `/boot/firmware/config.txt`. The Pi 5
uses `/dev/ttyAMA0` for this by default.

This workspace assumes USB. Edit `rover_lidar/config/ydlidar.yaml`
and `rover_bringup/launch/rover.launch.py` if you switch.

---

## Cooling is mandatory for SLAM

SLAM Toolbox pegs all four Cortex-A76 cores. Without a heatsink
or fan the Pi 5 will thermal-throttle within ~90 seconds and your
map will be built from a stuttering CPU.

Recommended: **official Pi 5 active cooler** ($5) or any 30 mm
5 V fan blowing across a 15×15 mm aluminum heatsink on the SoC.

```bash
vcgencmd measure_temp      # should stay below 70 degC under load
```

---

## Boot order and what to test

1. Power on the Pi 5 from the official 27 W PSU first.
2. SSH in. Confirm voltage is okay (`vcgencmd get_throttled` == 0).
3. Switch to the battery buck. Re-check `get_throttled` and run
   `stress-ng --cpu 4 --timeout 60s` while watching temperature.
4. Only then plug in the ESP32 and the LiDAR.
5. Run `ros2 launch rover_bringup rover.launch.py mode:=just_description`
   and verify RViz shows the model.