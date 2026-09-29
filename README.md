# Demo_MaRS_open-day_SUPER
This is the demo which used for HKU open days
# UAV LiDAR System Setup

This repository contains the setup and launch instructions for the UAV onboard system, including:

- Livox MID-360 LiDAR
- ROS 2 Humble
- Livox ROS Driver 2
- QGroundControl
- PX4 Flight Controller

---

## 1. System Environment

Tested environment:

```text
Ubuntu 22.04
ROS 2 Humble
Livox ROS Driver 2
PX4
QGroundControl
```

ROS workspace:

```bash
~/ws_livox
```

---

# 2. Livox MID-360 Network Configuration

## Current Network Configuration

The current MID-360 connection uses:

```text
PC Ethernet Interface: enp4s0

PC IP:
192.168.50.125

MID-360 IP:
192.168.50.163
```

The LiDAR is connected directly through `enp4s0`.

Check available Ethernet interfaces:

```bash
ip -br link
```

Check current IP addresses:

```bash
ip addr
```

Check Ethernet link:

```bash
cat /sys/class/net/enp4s0/carrier
```

Expected output:

```text
1
```

Check connection speed:

```bash
sudo ethtool enp4s0 | grep -E "Speed|Duplex|Link detected"
```

Example:

```text
Speed: 100Mb/s
Duplex: Full
Link detected: yes
```

---

# 3. Test MID-360 Connection

Ping the LiDAR:

```bash
ping -I enp4s0 -c 4 192.168.50.163
```

Expected output:

```text
64 bytes from 192.168.50.163
```

Check ARP:

```bash
ip neigh show dev enp4s0
```

Expected:

```text
192.168.50.163 lladdr XX:XX:XX:XX:XX:XX REACHABLE
```

---

# 4. Capture LiDAR Network Data

If the LiDAR IP or Ethernet interface is unknown, use `tcpdump`.

Monitor all UDP traffic:

```bash
sudo tcpdump -ni any -nn udp
```

Monitor only the MID-360 interface:

```bash
sudo tcpdump -ni enp4s0 -nn
```

Monitor only the LiDAR:

```bash
sudo tcpdump -ni enp4s0 -nn host 192.168.50.163
```

This is useful when changing Ethernet ports or debugging network problems.

---

# 5. MID-360 Driver Configuration

Configuration file:

```text
~/ws_livox/src/livox_ros_driver2/config/MID360_config.json
```

Current configuration:

```json
{
  "host_net_info": {
    "cmd_data_ip": "192.168.50.125",
    "push_msg_ip": "192.168.50.125",
    "point_data_ip": "192.168.50.125",
    "imu_data_ip": "192.168.50.125"
  },

  "lidar_configs": [
    {
      "ip": "192.168.50.163"
    }
  ]
}
```

Check the configuration:

```bash
grep -nE '192\.168\.' \
~/ws_livox/src/livox_ros_driver2/config/MID360_config.json
```

Also check the installed configuration:

```bash
grep -nE '192\.168\.' \
~/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

---

# 6. Build Livox ROS Driver

If the configuration or source code has been changed:

```bash
cd ~/ws_livox

source /opt/ros/humble/setup.zsh

colcon build --packages-select livox_ros_driver2 --symlink-install

source install/setup.zsh
```

---

# 7. Launch MID-360 with RViz

```bash
cd ~/ws_livox

source /opt/ros/humble/setup.zsh

source install/setup.zsh

ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

This starts:

```text
Livox ROS Driver 2
+
MID-360
+
RViz
```

---

# 8. Launch MID-360 Without RViz

If only LiDAR data is required:

```bash
cd ~/ws_livox

source /opt/ros/humble/setup.zsh

source install/setup.zsh

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

---

# 9. Check ROS 2 Topics

Open another terminal:

```bash
source /opt/ros/humble/setup.zsh

cd ~/ws_livox

source install/setup.zsh
```

List Livox topics:

```bash
ros2 topic list | grep livox
```

Typical topics:

```text
/livox/lidar
/livox/imu
```

Check LiDAR frequency:

```bash
ros2 topic hz /livox/lidar
```

Check IMU frequency:

```bash
ros2 topic hz /livox/imu
```

---

# 10. RViz Configuration

If ROS receives LiDAR data but RViz does not display the point cloud:

Set:

```text
Global Options
└── Fixed Frame
    └── livox_frame
```

PointCloud2 topic:

```text
/livox/lidar
```

---

# 11. Start QGroundControl

QGroundControl AppImage location:

```text
~/Downloads/QGroundControl-x86_64.AppImage
```

Launch QGroundControl:

```bash
cd ~/Downloads

./QGroundControl-x86_64.AppImage
```

If permission is denied:

```bash
chmod +x ~/Downloads/QGroundControl-x86_64.AppImage
```

Then:

```bash
cd ~/Downloads

./QGroundControl-x86_64.AppImage
```

---

# 12. Quick Start

## Terminal 1 — MID-360

```bash
cd ~/ws_livox

source /opt/ros/humble/setup.zsh

source install/setup.zsh

ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

## Terminal 2 — QGroundControl

```bash
cd ~/Downloads

./QGroundControl-x86_64.AppImage
```

## Terminal 3 — LiDAR Status

```bash
source /opt/ros/humble/setup.zsh

cd ~/ws_livox

source install/setup.zsh

ros2 topic hz /livox/lidar
```

---

# 13. Troubleshooting

## No Point Cloud

First test the LiDAR network:

```bash
ping -I enp4s0 192.168.50.163
```

If the ping fails, check:

```bash
ip addr
```

```bash
ip -br link
```

```bash
sudo ethtool enp4s0
```

---

## Ethernet Port Changed

Find active interfaces:

```bash
ip -br link
```

An active interface should contain:

```text
UP
LOWER_UP
```

Capture traffic:

```bash
sudo tcpdump -ni any -nn udp
```

The interface receiving traffic from:

```text
192.168.50.163
```

is the LiDAR Ethernet interface.

---

## Check LiDAR Packets

```bash
sudo tcpdump -ni enp4s0 -nn host 192.168.50.163
```

If packets are continuously received, the physical network connection is working.

---

## ROS Topic Exists but Has No Data

Check:

```bash
ros2 topic hz /livox/lidar
```

If it shows:

```text
waiting for messages
```

verify that `MID360_config.json` contains:

```text
Host IP:  192.168.50.125
LiDAR IP: 192.168.50.163
```

Then rebuild:

```bash
cd ~/ws_livox

source /opt/ros/humble/setup.zsh

colcon build --packages-select livox_ros_driver2 --symlink-install

source install/setup.zsh
```

---

# Current Hardware Network

```text
                         Ethernet
┌────────────────┐      enp4s0       ┌─────────────────┐
│                │◄─────────────────►│                 │
│ Ubuntu PC      │                   │ Livox MID-360   │
│                │                   │                 │
│ 192.168.50.125 │                   │ 192.168.50.163  │
└────────────────┘                   └─────────────────┘

        │
        │ USB
        ▼

┌────────────────┐
│ PX4 Flight     │
│ Controller     │
└────────────────┘
        ▲
        │
        │ MAVLink
        ▼
┌────────────────┐
│ QGroundControl │
└────────────────┘
```

---

# Notes

- Remove propellers during flight controller configuration and motor testing.
- Confirm the LiDAR IP and PC IP before launching the driver.
- If the Ethernet port changes, verify the interface name again.
- Do not assume the LiDAR is always connected to the same Linux interface.
- Use `tcpdump` to identify the correct Ethernet interface if necessary.
