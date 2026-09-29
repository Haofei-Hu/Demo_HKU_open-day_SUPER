# Demo_HKU_open-day_SUPER

HKU Open Day UAV demonstration using Livox MID360, ROS 2 Humble, SUPER and PX4.

## Quick Start — LiDAR and QGroundControl

### Terminal 1 — Start MID360 with RViz

Use this mode to display the raw LiDAR point cloud.

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

### Terminal 2 — Start QGroundControl

```bash
~/Downloads/QGroundControl-x86_64.AppImage
```

If permission is denied, run this once, then launch QGC again:

```bash
chmod +x ~/Downloads/QGroundControl-x86_64.AppImage
```

### Alternative — MID360 for FAST-LIO

Use CustomMsg mode when working with FAST-LIO.

Stop the existing Livox driver with Ctrl+C before switching modes.
Run only one Livox driver instance.

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

- `rviz_MID360_launch.py`: PointCloud2 output for RViz.
- `msg_MID360_launch.py`: CustomMsg output for FAST-LIO.
- CustomMsg cannot be displayed directly using RViz's standard PointCloud2 display.

### Check LiDAR and IMU Frequency

Open another terminal:

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
```

Run each check separately. Press Ctrl+C before the next check.

```bash
ros2 topic hz /livox/lidar
```

```bash
ros2 topic hz /livox/imu
```

Latest observed rates in CustomMsg mode:

| Topic | Observed rate |
|---|---:|
| `/livox/lidar` | Approximately 10 Hz |
| `/livox/imu` | Approximately 200 Hz |

## Deployment Progress

Updated: **2026-09-29**

| Component | Status | Notes |
|---|---|---|
| Ubuntu 22.04 + ROS 2 Humble | Complete | zsh environment |
| SUPER compilation | Complete | 6 packages built |
| Official click_demo | Passed | Simulated drone moves and reaches the goal |
| Backup trajectory configuration | Fixed | Added `backup_traj.penna_thr` |
| Original angular-rate limit | Passed | Demo succeeds with `max_omg: 2.5` |
| MID360 network | Passed | LiDAR: `192.168.50.163`; PC: `192.168.50.125` |
| MID360 point cloud | Verified | Approximately 10 Hz |
| MID360 IMU | Verified | Approximately 200 Hz |
| QGroundControl | Launch command documented | FCU connection not verified in this session |
| FAST-LIO ROS2 | Pending | Download/build not yet confirmed |
| FAST-LIO → SUPER | Pending | Registered cloud and odometry integration |
| SUPER → PX4 | Pending | Controller/bridge and SITL validation |

## SUPER Simulation — Verified Setup

Workspace: `~/super_ws`

Source: `~/super_ws/src/SUPER`

Upstream commit:

```text
2ad3419c127a617c6d7df6925e81a14175a9c096
```

Configuration file: `super_planner/config/click_smooth_ros2.yaml`

Verified limits:

```yaml
max_vel: 5.0
max_acc: 5.0
max_omg: 2.5
```

Required setting under `traj_opt.backup_traj`:

```yaml
penna_thr: 1.0e+5
```

The ROS2 example used `penna_max_acc_thr` and `penna_min_acc_thr`,
but the loader expects `penna_thr`. Correcting this restored successful
planning. Restoring `max_omg` from 6.0 to 2.5 also passed.

See [upstream issue #20](https://github.com/hku-mars/SUPER/issues/20).

### Launch SUPER Simulation

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
ros2 launch mission_planner click_demo.launch.py
```

Use **2D Goal Pose** in the top-down RViz window to select a nearby,
unobstructed target.

This demo uses an ideal tracking simulator and does not validate PX4
flight control. Stop it before connecting real FAST-LIO data to SUPER,
because the simulator also publishes `/cloud_registered`.

## Next Steps

- [ ] Build and run FAST-LIO ROS2.
- [ ] Verify `/cloud_registered`, `/Odometry` and stable handheld mapping.
- [ ] Connect real FAST-LIO data to SUPER without the simulator.
- [ ] Verify coordinate consistency, mapping and planning.
- [ ] Integrate the PX4 control layer.
- [ ] Complete SITL and propeller-off bench testing.

---
