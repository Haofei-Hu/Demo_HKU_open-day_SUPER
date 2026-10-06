# Demo_HKU_open-day_SUPER

Deployment notes for an HKU Open Day UAV demonstration using Livox MID360, FAST_LIO, SUPER, MAVROS, and PX4.

**Last updated: 2026-10-06**

## Quick Start

Run these commands on the **existing Ubuntu 22.04 / ROS 2 Humble computer**, using separate zsh terminals. The original system disk and workspaces are now running on the replacement AMD computer. This is a restart guide, not a fresh installation guide.

**Next-session breakpoint:** the FAST_LIO pose-covariance repair is present in the local source. A successful build and restart with the repaired binary have **not** been confirmed. Complete [the build and restart checkpoint](#build-and-restart-checkpoint) before treating the repair as active.

Before starting, connect to the network and wait until the computer's date and time are correct. Keep QGroundControl closed while MAVROS uses the flight controller USB port.

Keep each process terminal open. External vision fusion remains disabled (`EKF2_EV_CTRL = 0`). This procedure starts localization, planning, and communication; it does not automatically arm or take off.

The terminal labels used today are: **1 MID360, 2 FAST_LIO/RViz, 3 checks, 4 visualization TF, 5 SUPER, 6 MAVROS, 7 body-pose diagnostic**. Terminal 8 is an optional transport test; Terminal 9 is for the pending build.

### Terminal 1 — MID360

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Terminal 2 — FAST_LIO

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Keep the sensor stationary during initialization. Wait for:

```text
IMU Initial Done
Initialize the map kdtree
```

### Terminal 4 — Visualization Transform

```bash
source /opt/ros/humble/setup.zsh
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id world --child-frame-id camera_init
```

This defines the current visualization convention. It does not align FAST_LIO with the PX4 navigation frame.

### Terminal 5 — SUPER

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
mkdir -p ~/super_ws/src/SUPER/super_planner/log/replan_logs ~/super_ws/src/SUPER/super_planner/log/cmd_logs
ros2 run super_planner fsm_node --ros-args -p config_name:=mid360_real_ros2.yaml
```

`Wait for goal` is normal before submitting a goal.

### Terminal 6 — MAVROS

Close any QGroundControl instance directly connected to the flight controller USB port.

```bash
source /opt/ros/humble/setup.zsh
ros2 launch mavros px4.launch fcu_url:="serial:///dev/ttyACM0:2000000"
```

### Terminal 7 — Body-Pose Diagnostic (Optional)

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
python3 ~/Downloads/fastlio_body_debug.py
```

This publishes `/super_debug/body_pose` for inspection. It does not send flight commands.

### Terminal 3 — Checks

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
source ~/super_ws/install/setup.zsh
```

Run each check separately:

```bash
timeout 10s ros2 topic hz /Odometry
```

```bash
timeout 10s ros2 topic echo /mavros/state --once
```

```bash
ros2 param get /mavros/param EKF2_EV_CTRL
```

```bash
timeout 10s ros2 topic echo /mavros/timesync_status --field round_trip_time_ms
```

Expected basic results:

- `/Odometry`: approximately 10 Hz.
- `/mavros/state`: `connected: true`.
- `/mavros/state`: `armed: false`; `EKF2_EV_CTRL` remains `0` for these diagnostics.
- Latest time-sync round-trip samples were approximately **0.72–4.20 ms**. Recheck each session; the older multi-second anomaly has no confirmed root cause.

A 10 Hz topic can still carry old measurements. After restarting FAST_LIO, run [the timestamp-age check](#odometry-timestamp-age-check) in Terminal 3. Today's recovered stream averaged **21–23 ms**, with a maximum of **50 ms** during a two-minute planning test.

### RViz — Map and Planning Goal

In the RViz window started by FAST_LIO:

1. Open `~/fastlio_ws/mid360_super.rviz` through **File → Open Config**.
2. Set **Fixed Frame** to `world`.
3. Enable the PointCloud2 display for `/rog_map/occ`.
4. Enable the MarkerArray display for `/visualization/committed_traj`.
5. Set both displays to **Best Effort / Volatile**.
6. Under **Tool Properties → 2D Goal Pose**, set the topic to `/goal_pose`.
7. Select **2D Goal Pose**, then click and drag in a nearby accessible area.

Use **2D Goal Pose** to submit a goal. **2D Pose Estimate** is the initial-pose tool.

Recheck the configured goal height after restarting FAST_LIO. A visible trajectory or `FOLLOW_TRAJ` state confirms planner activity; aircraft trajectory execution is still pending integration.

The last reported test setting was `click_height: -0.05`. It is a coordinate in the initialized planning frame, not height above the floor. The final 3 m × 6 m venue settings have not been applied; see [the demonstration requirements](#11-demonstration-venue-requirements).

## 1. Deployment Status

**Current stage:** sensing, localization, real-data planning, and MAVROS communication were rechecked on 2026-10-06. ROS pose-covariance transport was demonstrated with external vision fusion disabled. The source covariance repair awaits a confirmed machine build and runtime check. PX4 fusion, trajectory execution, and venue boundary protection remain unfinished.

| Component | Status | Evidence / remaining work |
| --- | --- | --- |
| Ubuntu 22.04, ROS 2 Humble, zsh | Installed | Working deployment environment |
| MID360 network and driver | Rechecked today | Driver initialized; isolated `/livox/imu` check approximately 200 Hz |
| FAST_LIO localization | Rechecked today | `/Odometry` approximately 10 Hz; after restart, timestamp age averaged 21–23 ms during a two-minute planning test |
| SUPER simulation | Previously verified | Click-goal demo succeeded after correcting the backup trajectory configuration |
| SUPER with real FAST_LIO data | Rechecked today | `/rog_map/occ` width 4266; map and trajectory displayed in RViz |
| SUPER trajectory generation | Rechecked today | `FOLLOW_TRAJ` and repeated `ReplanOnce succeed` after a clicked goal |
| SUPER command output | Rechecked today | `/planning/pos_cmd` approximately 99.5–100 Hz |
| Nominal body-pose diagnostic script | Rechecked today | `/super_debug/body_pose` approximately 10 Hz; mounting and world alignment remain preliminary |
| QGroundControl direct USB connection | Previously verified | Firmware and MAVLink information retrieved |
| MAVROS–PX4 USB connection | Recovered after reboot | Latest reported state: `connected: true`, `armed: false`, `mode: AUTO.LOITER` |
| Raw pose-covariance transport to MAVROS | Verified at ROS topic level today | One publisher and one MAVROS subscriber; approximately 10 Hz after timestamp freshness recovered |
| Host clock synchronization | Rechecked after reboot | `timedatectl`: synchronized, NTP active, Asia/Hong_Kong, RTC in UTC |
| MAVROS time synchronization | Rechecked today | Observed round-trip samples approximately 0.72–4.20 ms; older anomaly cause unresolved |
| FAST_LIO pose-covariance source repair | Present in local source | Build, installed binary, and first-message runtime verification pending |
| FAST_LIO localization fused by PX4 | Not completed | Last confirmed `EKF2_EV_CTRL` value was `0` |
| SUPER commands converted to PX4 setpoints | Not completed | Controller/bridge implementation and validation required |
| Autonomous takeoff and task management | Not implemented | Takeoff, hover, goal sequencing, and landing states required |
| Offboard flight and autonomous obstacle avoidance | Not validated | No completed real-flight execution test |
| Final venue configuration | Requirements recorded | 3 m wide × 6 m long × 3 m high, target 2 m/s, maximum height 2 m; boundary enforcement pending |

A successful planner state does not establish that the aircraft followed the trajectory.

Closing process terminals or rebooting the computer normally requires starting the software again. Previous test results are not a substitute for checking the current session.

## 2. Hardware and Software

| Item | Recorded configuration |
| --- | --- |
| Current computer | Replacement AMD computer using the original Ubuntu 22.04 system disk and existing workspaces |
| Available resources | 8 logical CPUs; approximately 15 GiB RAM, 11 GiB available at the last check; 2 GiB swap unused |
| ROS distribution | ROS 2 Humble |
| Shell | zsh |
| LiDAR | Livox MID360 |
| Livox ROS Driver 2 | Version 1.2.7 reported in startup logs |
| SUPER commit | `2ad3419c127a617c6d7df6925e81a14175a9c096` |
| FAST_LIO branch | `ROS2` |
| FAST_LIO commit | `a4743b095409588842a5b30ddfa27e29d2f99164` |
| ikd-Tree commit | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |
| MAVROS / MAVROS extras | 2.14.0 |
| Flight controller architecture | `HKUST_NXT_DUAL`, STM32H7 |
| PX4 firmware | Custom-board build of Release 1.17.0 |
| PX4 git hash | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| PX4 build date | 2026-05-13 18:40:15 |
| Flight controller connection | USB MAVLink |
| Recorded USB device | `/dev/ttyACM0` |
| USB identification | `usb-Matek_HKUST_UAV_NxtPX4_0-if00` |
| Communication route | MAVROS over MAVLink |
| uXRCE-DDS status during inspection | Client not running; `UXRCE_DDS_CFG = 0` |

### Workspace locations

```text
~/ws_livox/
~/fastlio_ws/
~/fastlio_ws/src/FAST_LIO/
~/super_ws/
~/super_ws/src/SUPER/
~/fastlio_ws/mid360_super.rviz
~/Downloads/fastlio_body_debug.py
~/Downloads/raw_pose_cov_transport.py
~/Downloads/QGroundControl-x86_64.AppImage
```

These are paths on the onboard Ubuntu computer. Check the repository contents separately to determine which configuration files and scripts have been committed.

Today's diagnostic scripts and covariance repair are now archived in this repository:

| File | Purpose |
| --- | --- |
| [fastlio_body_debug.py](fastlio_body_debug.py) | Existing nominal mounting diagnostic; no flight-control output |
| [raw_pose_cov_transport.py](raw_pose_cov_transport.py) | Raw ROS pose/covariance transport test; requires external vision fusion disabled |
| [odom_age_check.py](odom_age_check.py) | The isolated 15-second, depth-one timestamp-age check |
| [apply_fastlio_covariance_fix.py](apply_fastlio_covariance_fix.py) | Apply the reviewed source replacement with a backup and mismatch check |
| [fastlio_pose_covariance_fix.patch](patches/fastlio_pose_covariance_fix.patch) | Equivalent patch for the recorded FAST_LIO source |
| [test_raw_transport.py](tests/test_raw_transport.py) | ROS DDS test with a simulated MAVROS parameter service |
| [verify_fastlio_pose_covariance.py](tests/verify_fastlio_pose_covariance.py) | Isolated Eigen/C++ covariance verification against the actual patched function |

The complete FAST_LIO/Livox/SUPER workspaces and their local configuration files are not vendored here. Their paths above still refer to the Ubuntu machine.

From the root of a local clone, the following copies the diagnostic scripts to the locations used in this runbook:

```bash
cp fastlio_body_debug.py raw_pose_cov_transport.py odom_age_check.py ~/Downloads/
```

The covariance helper defaults to `~/fastlio_ws/src/FAST_LIO/src/laserMapping.cpp`. Run it only after checking that this is the intended source file:

```bash
python3 apply_fastlio_covariance_fix.py
```

It detects the exact repair already used today, makes no change if present, refuses an unexpected old block, and saves a source backup under `~/fastlio_cov_backup_*` before writing. Alternatively use the patch with `git apply --check` first; do not apply both approaches. Building and verifying the installed executable remain separate steps.

### MID360 network

| Item | Value |
| --- | --- |
| Ethernet interface | `enp4s0` |
| Onboard computer IP | `192.168.50.125/24` |
| MID360 IP | `192.168.50.163` |
| General network interface | `enp5s0`, last reported `192.168.1.102/24` |

Useful checks:

```bash
ip -br addr
ping -I enp4s0 -c 4 192.168.50.163
ip neigh show dev enp4s0
```

The driver startup log reports the configuration file actually loaded. Preserve the full JSON structure when changing addresses.

Today's launch loaded `~/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json` and successfully enabled normal mode, Livox CustomMsg output, and IMU output.

## 3. System Architecture

### Current deployment

```text
MID360
  |
  v
Livox ROS Driver 2
  | /livox/lidar + /livox/imu
  v
FAST_LIO
  | /Odometry + /cloud_registered
  v
SUPER / ROG-Map
  |                         |
  | /planning/pos_cmd       | Occupancy map and trajectory markers
  v                         v
Controller bridge          RViz
[not implemented]
  |
  v
MAVROS <---- USB MAVLink ----> PX4
```

A nominal mounting diagnostic is available:

```text
FAST_LIO /Odometry
  |
  v
fastlio_body_debug.py
  |
  v
/super_debug/body_pose
```

Today's separate covariance transport test used:

```text
FAST_LIO /Odometry
  |
  v
raw_pose_cov_transport.py [transport only; EKF2_EV_CTRL must remain 0]
  |
  v
/mavros/vision_pose/pose_cov → MAVROS vision_pose plugin
```

The raw transport test retains the FAST_LIO IMU origin, `camera_init` frame, timestamp, and covariance. It does not apply the nominal mounting diagnostic transform. Neither path establishes PX4 fusion or alignment with its navigation frame. The older PoseStamped relay test is a separate historical transport check.

### Intended operator workflow

The Ubuntu computer is the onboard computer. A ground computer will provide the operator interface.

```text
Ground computer
  | Network connection
  | Start software, inspect the map, submit goals
  v
Onboard computer
  | Localization, planning, task management, controller
  | MAVROS over USB
  v
PX4 flight controller
  | Low-level flight control
  v
Aircraft
```

The ground connection, task interface, and automated takeoff workflow are planned features.

For initial development, the intended approach is:

1. Connect to the onboard computer using SSH and/or remote desktop.
2. Start the software and inspect localization.
3. Use RC-controlled takeoff and hover during initial flight validation.
4. Switch to Offboard only after the execution chain is validated.
5. Submit a local goal through RViz.
6. Later add automated takeoff, waypoint sequencing, hover, and landing.

Clicking a goal in the current deployment generates a plan. It does not automatically arm or launch the aircraft.

## 4. Clock Synchronization

### Current startup practice

Connect to the network and wait for the date and time to be correct **before** starting ROS. A manual `chronyc` command is not required for the current restart procedure. The existing Chrony installation has not been removed or disabled.

Check:

```bash
timedatectl status
```

After today's reboot, the reported state was:

```text
Local time: Tue 2026-10-06 21:14:42 HKT
Universal time: Tue 2026-10-06 13:14:42 UTC
Time zone: Asia/Hong_Kong (HKT, +0800)
System clock synchronized: yes
NTP service: active
RTC in local TZ: no
```

This verifies the reported synchronization state. It does not identify the active NTP daemon, measure absolute clock accuracy, or confirm that runtime stepping was disabled for this boot.

A clock step while ROS is running can make timestamps discontinuous. Avoid manually changing the time during a run. The earlier timing problems were not conclusively attributed to clock steps.

### Historical Chrony troubleshooting

An earlier session installed Chrony, checked that `systemd-timesyncd` was inactive, and retained these configuration entries:

```text
rtcsync
makestep 1 3
```

That session also ran the following manual gate and received `200 OK`, with approximately 28 microseconds of remaining correction and 2.658 ppm skew:

```bash
chronyc waitsync 60 0.01 10 1 && sudo chronyc makestep 1 0
```

The second command disables automatic stepping for that Chrony process. Its effect must not be assumed to survive a reboot or Chrony restart. It is retained here as troubleshooting history, not as a mandatory command for today's startup.

If time synchronization fails, inspect the configured time service before starting the stack:

```bash
chronyc sources -v
chronyc tracking
systemctl is-active chrony systemd-timesyncd
```

An offline cold start needs a suitable time source or a separately validated offline startup procedure. No unattended ROS startup service has been configured.

### FAST_LIO time parameters

The following values were checked on 2026-10-06:

```yaml
common:
  time_sync_en: false
  time_offset_lidar_to_imu: 0.0

preprocess:
  timestamp_unit: 3
```

These are selected entries from the existing configuration, not a replacement YAML file.

`time_sync_en` concerns LiDAR–IMU timestamp alignment. It does not synchronize the host computer with NTP.

Use `use_sim_time: false` for this real-hardware deployment. The absence of that key from the checked YAML does not itself indicate an error.

Preserve original measurement timestamps in diagnostic and localization bridges. Replacing an old measurement timestamp with the current time does not repair timing problems.

## 5. Startup Procedure

Use a separate terminal for each long-running process.

The terminal numbers below match the current session: 1 MID360, 2 FAST_LIO/RViz, 3 checks, 4 visualization TF, 5 SUPER, 6 MAVROS, and 7 body-pose diagnostics. Keep the process terminals open while they are needed; Terminal 3 is for temporary checks. Terminal 8 is optional transport testing and Terminal 9 is the build terminal.

Do not start duplicate driver or MAVROS instances.

### Before starting any ROS process

Connect to the network, wait for correct system time, and inspect `timedatectl status` as described in Section 4. Complete the pending source build before using the covariance repair.

### Terminal 1 — MID360 driver

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Keep this terminal running.

Expected messages include:

```text
Init lidar success!
livox/lidar publish use livox custom format
livox/imu publish use imu format
```

This launch provides the Livox CustomMsg data used by FAST_LIO.

### Terminal 2 — FAST_LIO

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Keep the sensor stationary during initialization.

Expected messages include:

```text
IMU Initial Done
Initialize the map kdtree
```

This launch also starts RViz. Load the saved visualization configuration through:

```text
File → Open Config → ~/fastlio_ws/mid360_super.rviz
```

An initial `No point, skip this scan!` message should be assessed together with subsequent initialization and topic output.

### Terminal 4 — Visualization frame convention

```bash
source /opt/ros/humble/setup.zsh
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id world --child-frame-id camera_init
```

Keep this terminal running.

The current visualization setup treats `world` and `camera_init` as coincident.

This convention does not align FAST_LIO with the PX4 local frame, establish geographic heading, or calibrate the LiDAR-to-airframe mounting transform.

### Terminal 5 — SUPER with real sensor data

Create the log directories once:

```bash
mkdir -p ~/super_ws/src/SUPER/super_planner/log/replan_logs ~/super_ws/src/SUPER/super_planner/log/cmd_logs
```

Start the planner:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
ros2 run super_planner fsm_node --ros-args -p config_name:=mid360_real_ros2.yaml
```

Keep this terminal running.

Before a goal is submitted, the following is a normal waiting state:

```text
Current state: INIT
Wait for goal.
```

Use this standalone planner for real FAST_LIO input.

The simulation entry point `click_demo.launch.py` also launches simulated data sources. Run simulation separately to avoid conflicting point-cloud and odometry publishers.

### Terminal 6 — MAVROS

Close QGroundControl if it is directly using the flight controller USB port.

```bash
source /opt/ros/humble/setup.zsh
ros2 launch mavros px4.launch fcu_url:="serial:///dev/ttyACM0:2000000"
```

Keep this terminal running.

The baseline command omits `gcs_url`. In the recorded MAVROS 2.14 launch file, its default is empty.

Do not pass:

```text
gcs_url:=""
```

That form produces an empty launch argument and was rejected as malformed.

Verified prerequisites:

- `/usr/share/GeographicLib/geoids/egm96-5.pgm` exists.
- User `mars` belongs to `dialout`.
- The recorded flight controller port is `/dev/ttyACM0`.

Check the device after reconnecting USB:

```bash
ls -l /dev/serial/by-id/
```

### Terminal 7 — Nominal body-pose diagnostic

Use the previously deployed diagnostic script:

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
python3 ~/Downloads/fastlio_body_debug.py
```

Keep this terminal running while checking `/super_debug/body_pose`.

If the script is stored elsewhere, adjust the path to the actual file.

This script is for mounting-transform inspection. It is not the flight controller bridge.

### Terminal 3 — Checks

Initialize this terminal:

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
source ~/super_ws/install/setup.zsh
```

Run checks individually. Stop each continuous frequency check with Ctrl+C before running the next one.

```bash
timeout 10s ros2 topic hz /livox/lidar
timeout 10s ros2 topic hz /livox/imu
timeout 10s ros2 topic hz /Odometry
timeout 10s ros2 topic hz /cloud_registered
timeout 10s ros2 topic hz /super_debug/body_pose
```

Check the planner command stream after generating a trajectory:

```bash
timeout 10s ros2 topic hz /planning/pos_cmd
```

Check the flight controller:

```bash
timeout 10s ros2 topic echo /mavros/state --once
timeout 10s ros2 topic echo /mavros/local_position/pose --once
timeout 10s ros2 topic echo /mavros/estimator_status --once
ros2 param get /mavros/param EKF2_EV_CTRL
```

Recheck time synchronization:

```bash
timeout 10s ros2 topic echo /mavros/timesync_status --field round_trip_time_ms
```

The latest reported connected-state snapshot was:

```yaml
connected: true
armed: false
guided: true
manual_input: false
mode: AUTO.LOITER
system_status: 0
```

`connected: true` establishes MAVLink communication. It does not establish estimator readiness or Offboard operation.

## 6. Configuration Record

Values below are the last recorded deployment settings. Recheck coordinate-dependent values after restarting FAST_LIO.

### FAST_LIO

File:

```text
~/fastlio_ws/src/FAST_LIO/config/mid360.yaml
```

| Parameter | Recorded value |
| --- | --- |
| `common.lid_topic` | `/livox/lidar` |
| `common.imu_topic` | `/livox/imu` |
| `common.time_sync_en` | `false` |
| `common.time_offset_lidar_to_imu` | `0.0` |
| `preprocess.lidar_type` | `1` |
| `preprocess.timestamp_unit` | `3` |
| `preprocess.scan_rate` | `10` |
| `mapping.extrinsic_est_en` | `true` |
| `mapping.extrinsic_T` | `[-0.011, -0.02329, 0.04412]` |
| `mapping.extrinsic_R` | Identity matrix |
| `pcd_save.pcd_save_en` | `false` |

The configured IMU–LiDAR extrinsic and the airframe mounting transform describe different relationships.

Online extrinsic estimation is enabled. A diagnostic script using the initial configured extrinsic does not automatically track the live estimated extrinsic.

The active launch reads an installed configuration. If source edits do not appear in startup behavior, check whether the installed file is a symlink or a copied file that needs to be rebuilt.

### SUPER real-sensor configuration

File:

```text
~/super_ws/src/SUPER/super_planner/config/mid360_real_ros2.yaml
```

| Parameter | Recorded value |
| --- | --- |
| `fsm.click_goal_en` | `true` |
| `fsm.click_goal_topic` | `/goal_pose` |
| `fsm.click_height` | `-0.05` in the latest test; previously `-0.19` |
| `fsm.click_yaw_en` | `true` |
| `fsm.cmd_topic` | `/planning/pos_cmd` |
| `fsm.mpc_cmd_topic` | `/planning_cmd/poly_traj` |
| `rog_map.ros_callback.cloud_topic` | `/cloud_registered` |
| `rog_map.ros_callback.odom_topic` | `/Odometry` |
| `rog_map.resolution` | `0.1` |
| `rog_map.inflation_resolution` | `0.2` |
| `rog_map.inflation_step` | `2` |
| `rog_map.unk_inflation_step` | `1` |
| `rog_map.virtual_ground_height` | `-1.0` |
| `rog_map.raycasting.enable` | `true` |
| `traj_opt.backup_traj.penna_thr` | `1.0e+5` |

The recorded startup log confirmed that SUPER loaded the real configuration from the source directory above. Restart SUPER after changing its YAML values.

### Backup trajectory correction

The deployed configuration loader expects:

```yaml
penna_thr: 1.0e+5
```

Correcting the backup trajectory key restored successful simulation planning.

Increasing the angular-rate limit alone was not the configuration fix.

### Virtual ground

The original warning was:

```text
Odom below virtual ground, please check map parameter
```

At that time, FAST_LIO reported approximately `z = -0.217 m`.

ROG-Map adjusts the effective virtual-ground boundary using its grid and inflation settings. Therefore, the raw YAML height is not necessarily the boundary used by the odometry check.

The last recorded working setting was:

```yaml
virtual_ground_height: -1.0
```

The warning disappeared during subsequent testing.

This value is a map boundary used during deployment testing. It is not a measurement of the physical floor.

### Click-goal height

The original setting was:

```yaml
click_height: 1.5
```

SUPER overwrote the clicked goal's Z coordinate with this value. With odometry near `z = -0.186 m`, the resulting trajectory climbed toward `z = 1.5 m`.

The first test adjustment was:

```yaml
click_height: -0.19
```

The latest reported test used `click_height: -0.05`. After a FAST_LIO restart, raw odometry Z was approximately `-0.070 m`, so this target was only about 2 cm higher in that initialized frame.

These are fixed coordinates in the current planning frame. They are not height above the floor or an automatic “maintain current altitude” command. `click_height` does not constrain the entire trajectory to the venue's 2 m maximum height.

Recheck it after restarting FAST_LIO because its initialized frame can change. A three-dimensional planner can also change intermediate altitude to avoid obstacles.

## 7. RViz and Goal Submission

### Displays

| Display | Type | Topic / setting |
| --- | --- | --- |
| Fixed Frame | Global option | `world` |
| Occupancy map | PointCloud2 | `/rog_map/occ` |
| Committed trajectory | MarkerArray | `/visualization/committed_traj` |
| Registered LiDAR cloud | PointCloud2 | `/cloud_registered`, optional |

For the occupancy map and committed trajectory displays:

```text
Reliability Policy: Best Effort
Durability Policy: Volatile
```

Recorded occupancy-map display settings:

```text
Style: Boxes
Size (m): 0.1
Alpha: 1
Position Transformer: XYZ
Color Transformer: FlatColor
Color: 255; 255; 255
```

To add the trajectory display:

```text
Displays → Add → MarkerArray
Topic → /visualization/committed_traj
```

Enable the checkbox for the display.

### Submit a goal

1. Open `Panels → Tool Properties`.
2. Expand `2D Goal Pose`.
3. Set its topic to `/goal_pose`.
4. Select `2D Goal Pose` in the top toolbar.
5. Click and drag in a nearby accessible area to choose the goal and heading.
6. Observe the committed trajectory and planner logs.

`2D Pose Estimate` publishes an initial pose. It is not the planning-goal tool.

Previously observed successful logs:

```text
GenerateExpTrajectory SUCCESS
change state from [GENERATE_TRAJ] to [FOLLOW_TRAJ]
ReplanOnce succeed
```

Until a controller is integrated, the aircraft does not follow the generated trajectory. A planner trajectory finishing in time does not prove that the physical aircraft reached the goal.

## 8. Mounting Transform and External Localization

### Recorded mounting arrangement

- Flight controller arrow points toward the aircraft nose.
- Flight controller is installed upside down.
- The board orientation has already been configured in PX4.
- Last confirmed `SENS_BOARD_ROT` value: `8`.
- LiDAR mounting yaw is approximately 45 degrees counterclockwise relative to the nose.
- LiDAR center is approximately 0.10 m above the flight controller center.
- The two centers are nominally on the same vertical axis.

The diagnostic transform uses a nominal +45-degree mounting yaw under its stated frame convention.

Do not apply the flight controller's mounting correction a second time to a pose already expressed in the corrected airframe frame.

### Diagnostic script behavior

`fastlio_body_debug.py`:

- Subscribes to `/Odometry`.
- Accepts the recorded `camera_init` parent frame and `body` child frame.
- Applies a nominal mounting rotation and lever-arm correction.
- Publishes a `geometry_msgs/msg/PoseStamped` on `/super_debug/body_pose`.
- Retains the input measurement timestamp and parent frame.
- Uses the initial configured IMU–LiDAR offset.
- Does not publish flight commands, arm the aircraft, or set PX4 parameters.

It does not perform:

- Calibration against the live online extrinsic estimate.
- Alignment with the PX4 local-frame origin and heading.
- Velocity estimation or covariance construction.
- PX4 estimator fusion validation.
- Flight control.

Today's separately sampled motion checks showed approximately:

| Motion | Diagnostic angle change | PX4 angle change |
| --- | --- | --- |
| Yaw | +24.96 degrees | +24.77 degrees |
| Pitch | -11.18 degrees | -10.27 degrees |
| Roll | +10.01 degrees | +11.16 degrees |

This supports the nominal axis directions but is not synchronized calibration. A later stationary snapshot gave an approximately 145.4-degree yaw difference; it is not an accepted alignment parameter. Earlier yaw offsets must also be discarded after initialization changes.

The inspected FAST_LIO initialization sets its gravity estimate without guaranteeing that `camera_init` is level. Validate gravity and full world-frame rotation, rather than assuming that a yaw-only alignment is sufficient. PX4's invalid horizontal-position status also prevents using its current local translation as a validated alignment reference.

### MAVROS topic direction

| Topic | Direction |
| --- | --- |
| `/mavros/vision_pose/pose` | ROS pose input to MAVROS |
| `/mavros/vision_pose/pose_cov` | ROS pose-with-covariance input to MAVROS |
| `/mavros/odometry/out` | ROS odometry input to MAVROS |
| `/mavros/odometry/in` | MAVROS odometry output to ROS |
| `/mavros/local_position/pose` | PX4 local pose reported through MAVROS |

Check the actual endpoints when using these interfaces:

```bash
ros2 topic info /mavros/vision_pose/pose --verbose
ros2 topic info /mavros/odometry/out --verbose
ros2 topic info /mavros/odometry/in --verbose
```

A PoseStamped message cannot be directly relayed into an Odometry topic.

A snapshot of FAST_LIO `/Odometry` contained zero twist values. Do not treat those values as validated measured velocity for PX4 velocity fusion.

MAVROS performs its ENU/FLU to NED/FRD conversion, but the vision-pose input does not establish FAST_LIO-to-PX4 world alignment by reading `frame_id` or applying the visualization TF. Perform the required physical and world-frame alignment explicitly, without duplicating MAVROS axis conversions.

### Optional Terminal 8 — Raw pose-covariance transport test

This is a diagnostic operation, not part of the default startup sequence.

Use [raw_pose_cov_transport.py](raw_pose_cov_transport.py), copied to `~/Downloads/`, only while the aircraft is disarmed and `EKF2_EV_CTRL = 0`. It forwards raw `/Odometry` pose and covariance to `/mavros/vision_pose/pose_cov`, retaining the original frame and timestamp. It does not convert to the flight-controller center or align with the PX4 navigation frame.

Check the prerequisite in Terminal 3:

```bash
ros2 param get /mavros/param EKF2_EV_CTRL
```

Run in Terminal 8:

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/raw_pose_cov_transport.py
```

The script checks for a recent read-only confirmation of `EKF2_EV_CTRL = 0`. It rejects stale/future, repeated or backward timestamps, unexpected frames, invalid pose values, and invalid covariance diagonals. Its parameter polling is not an atomic interlock: **stop this script before changing fusion parameters**.

Inspect it from Terminal 3:

```bash
ros2 topic info /mavros/vision_pose/pose_cov --verbose
timeout 15s ros2 topic hz /mavros/vision_pose/pose_cov
```

Today's test showed:

```text
Publisher count: 1
Subscription count: 1
Rate: approximately 10 Hz
```

Initially, the endpoints existed but no sustained output passed the freshness check because input timestamps were approximately 1.3–1.8 seconds old. After restarting FAST_LIO, the output recovered to approximately 10 Hz, with a measured maximum interval of 0.131 seconds.

This confirms ROS transport only. PX4 receipt of the underlying visual-odometry measurement has not been verified, and fusion remains disabled. A valid covariance diagonal check also does not establish correct covariance semantics; the FAST_LIO source repair below still needs runtime verification.

An earlier `topic_tools` relay from `/super_debug/body_pose` to `/mavros/vision_pose/pose` was a separate PoseStamped transport test. Do not run multiple pose input paths together.

### FAST_LIO pose-covariance source repair

The actual machine source was inspected at commit `a4743b095409588842a5b30ddfa27e29d2f99164`, in `~/fastlio_ws/src/FAST_LIO/src/laserMapping.cpp`. `state_ikfom` stores **position first, then rotation**. The old `publish_odometry()` code published before filling covariance and incorrectly swapped those state blocks.

The repair computes the current covariance before publishing and converts the IMU-local rotation error to fixed world axes, including position/rotation cross terms:

```cpp
const auto P = kf.get_P();
Eigen::Matrix<double, 6, 6> covariance_axes =
    Eigen::Matrix<double, 6, 6>::Identity();
covariance_axes.block<3, 3>(3, 3) = state_point.rot.toRotationMatrix();
const Eigen::Matrix<double, 6, 6> pose_covariance =
    covariance_axes * P.block<6, 6>(0, 0) * covariance_axes.transpose();
for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
        odomAftMapped.pose.covariance[i * 6 + j] = pose_covariance(i, j);
pubOdomAftMapped->publish(odomAftMapped);
```

The last patch command reported `Fix already present; no changes made.` This confirms that the replacement block was detected in the local source; it does **not** confirm a successful package build or that the running executable contains it. Existing YAML and RViz edits must be preserved.

Isolated Eigen/C++ checks exercised current-frame covariance publication, cross terms, and the rotation Jacobian. They do not substitute for building the real ROS package or validating its output on the aircraft. A later bridge must also transform covariance for its mounting, lever-arm, and world-frame conversions.

## 9. MAVROS, PX4, and Timing Findings

### External vision fusion

The last confirmed value was:

```text
EKF2_EV_CTRL = 0
```

External vision fusion remained disabled during today's transport tests.

The reported estimator status had valid attitude, invalid horizontal velocity and horizontal-position flags, and `const_pos_mode_status_flag: true`. The presence of `/mavros/local_position/pose` therefore did not establish usable horizontal localization.

Other read-only parameter checks returned `EKF2_HGT_REF = 1` and `EKF2_EV_DELAY = 0.0`. The height-reference setting does not prove a valid height source. A zero configured delay correction does not mean that actual measurement latency is zero.

Before enabling fusion, validate:

- Body-frame mounting transformation.
- Local-world origin and heading convention.
- Sensor timestamps and MAVROS time synchronization.
- Pose update rate and delivery gaps.
- Covariance and delay handling.
- Which measurements are actually available for fusion.

The current diagnostic pose stream is approximately 10 Hz. PX4 documentation recommends 30–50 Hz external pose input; the inspected upstream 1.17 source does not automatically reject a fresh 10 Hz stream solely for that rate. Acceptance on this custom firmware and flight suitability still need measurement. Repeating old measurements at a higher publication rate does not improve sensor freshness.

### Timing findings on 2026-10-06

Historical MAVROS time-sync measurements included approximately 8300–9200 ms round-trip time. Today's visible samples were approximately **0.72–4.20 ms**. The previous anomaly was not reproduced in those samples; its cause has not been proven or permanently resolved.

The FAST_LIO latency investigation found:

| Check | Result | Interpretation |
| --- | --- | --- |
| Isolated `/livox/imu` frequency | Approximately 200 Hz | The FAST_LIO IMU input was not running at the suspected 50 Hz |
| Raw serialized LiDAR header, depth 1 | Age min/mean/max 0.104 / 0.109 / 0.118 s | Avoided expensive CustomMsg decoding; scan timestamp semantics include acquisition time |
| Isolated `/Odometry`, depth 1, before restart | Age min/mean/max 1.292 / 1.376 / 1.786 s | Approximately 10 Hz publication did not guarantee fresh estimates |
| Same odometry check after FAST_LIO restart | Age min/mean/max 0.016 / 0.023 / 0.035 s | Freshness recovered after restarting FAST_LIO |
| Two minutes with SUPER planning active | Window means 0.021–0.023 s; largest observed age 0.050 s | No progressive backlog observed during this test |

The combined Python LiDAR/IMU/odometry probe received too few IMU messages and reported much larger LiDAR ages. Its CustomMsg decoding and callback load made it unsuitable for attributing the delay. Separate lightweight checks were used instead. Stopping SUPER alone did not clear the earlier odometry delay; restarting FAST_LIO did.

The trigger of the old process's delay is still unknown. Process CPU percentages on an eight-core host do not establish whole-machine CPU saturation. If it recurs, inspect processing queues and active accumulated-map/path publication settings. The two-minute recovery test is not a long-duration flight validation. Preserve measurement timestamps rather than substituting the current time or loosening the transport freshness check.

### Odometry timestamp-age check

Run in Terminal 3 after sourcing ROS. This measures receipt time minus the original measurement timestamp using a depth-one subscriber:

The same check is available as [odom_age_check.py](odom_age_check.py). After copying it to Downloads, run `python3 ~/Downloads/odom_age_check.py`, or paste the inline version below.

```bash
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('odom_age_check')
ages = []

def callback(msg):
    stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
    ages.append((node.get_clock().now().nanoseconds - stamp) / 1e9)

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
sub = node.create_subscription(Odometry, '/Odometry', callback, qos)
try:
    end = time.monotonic() + 15
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if ages:
        print(f'/Odometry: n={len(ages)}, age min/avg/max='
              f'{min(ages):.3f}/{sum(ages)/len(ages):.3f}/'
              f'{max(ages):.3f} seconds')
    else:
        print('No Odometry received')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

### Communication interruption after reboot

During recovery, the following was observed:

```text
connected: false
mode: ''
Parameter not set.
```

MAVROS also reported:

```text
MAVConnSerial::send_message: TX queue overflow
DROPPED Message-Id ...
```

A reboot later restored `connected: true`.

TX queue overflow indicates a backed-up transmit queue. It does not by itself identify a baud-rate mismatch, USB failure, or clock jump.

A parameter query returning `Parameter not set.` during a communication problem is not evidence that the PX4 parameter became zero.

### USB ownership

Inspect the port owner when communication fails:

```bash
sudo fuser -v /dev/ttyACM0
```

MAVROS and QGroundControl should not independently open the same USB serial device at the same time.

For a direct QGroundControl USB check:

1. Stop MAVROS in Terminal 6 with Ctrl+C.
2. Close any other program using the port.
3. Open QGroundControl.
4. Close QGroundControl before restarting MAVROS.

QGroundControl launch:

```bash
~/Downloads/QGroundControl-x86_64.AppImage
```

If required, grant execution permission once:

```bash
chmod +x ~/Downloads/QGroundControl-x86_64.AppImage
```

MAVROS-to-QGroundControl UDP forwarding remains unverified. The baseline MAVROS startup command does not enable it.

## 10. Common Problems

### Message type is invalid

For Livox CustomMsg checks:

```bash
source ~/ws_livox/install/setup.zsh
```

For SUPER PositionCommand checks:

```bash
source ~/super_ws/install/setup.zsh
```

Each new terminal needs the relevant ROS environment.

### A topic check produces no output

First inspect publishers and subscribers:

```bash
ros2 topic info /super_debug/body_pose --verbose
```

If the diagnostic script was stopped to run a check in the same terminal, its output topic will stop as well.

Keep the body-pose diagnostic publisher in Terminal 7 and run checks in Terminal 3. Keep the optional raw covariance transport in Terminal 8 when testing that path.

A timeout returning without a message means no sample was received during that interval. Investigate the publisher and connection before changing settings.

### Startup discovery warning followed by rate output

A message such as:

```text
topic [...] does not appear to be published yet
```

can occur before discovery completes.

If continuous frequency output follows, assess the measured stream. The initial warning alone does not establish a continuing failure.

### RViz occupancy map is empty

Check the map output:

```bash
ros2 topic echo /rog_map/occ --once --field width
```

A previous test reported a width of `7637` while RViz showed no map. Changing the display QoS to Best Effort restored visualization.

Also check the fixed frame and position transformer. `Status: Ok` alone does not prove the display contains a nonempty map.

### SUPER remains in INIT

`Wait for goal` is normal before a goal is submitted.

Confirm that RViz uses `2D Goal Pose` with topic `/goal_pose`.

### SUPER reports No odom

Inspect the actual odometry stream:

```bash
timeout 10s ros2 topic hz /Odometry
ros2 topic info /Odometry --verbose
```

Possible issues include a stopped publisher, delayed delivery, incorrect topic configuration, or freshness checks.

The previously observed persistent warning was not conclusively attributed to a single cause.

### SUPER cannot write logs on exit

Ensure these directories exist:

```bash
mkdir -p ~/super_ws/src/SUPER/super_planner/log/replan_logs ~/super_ws/src/SUPER/super_planner/log/cmd_logs
```

### Terminal shows a continuation prompt or malformed command

Copy only the commands inside a code block.

Do not include terminal prompts, HTML line-break tags, or trailing backslashes. Press Ctrl+C to cancel accidental continuation input.

## 11. Demonstration Venue Requirements

The user specified these final demonstration requirements on 2026-10-06:

| Requirement | Value |
| --- | --- |
| Venue width | 3 m |
| Venue length | 6 m |
| Venue ceiling height | 3 m |
| Demonstration target speed | 2 m/s |
| Maximum flight height | 2 m |
| Boundary clearance | The whole aircraft, including propellers, must not collide with any venue boundary |

**These are requirements, not an applied or validated configuration.** The flight-height reference point and whether 2 m/s is a cruise target or strict speed ceiling still need clarification. Raw FAST_LIO Z is not a measured height above the venue floor.

Before selecting final values, measure the aircraft's complete horizontal and vertical envelope, define the demonstration route, and locate the floor and boundaries in a fixed venue frame. Account for localization and tracking error, total reaction latency, braking acceleration, and jerk when shrinking the usable volume. At 2 m/s, the observed 50 ms pose age alone corresponds to 0.10 m of travel; it excludes other reaction delays and braking distance.

Relevant SUPER configuration entries and their limits are:

| Setting | Purpose and limitation |
| --- | --- |
| `traj_opt.boundary.max_vel`, `max_acc`, `max_jerk` | Optimization bounds; setting `max_vel: 2` alone does not establish a strict executed speed ceiling |
| `super_planner.robot_r` | Collision envelope; derive it from the actual aircraft including propellers and accessories |
| `rog_map.inflation_resolution`, `inflation_step`, unknown-space inflation | Obstacle margins; account for their interaction with robot radius and grid quantization |
| `rog_map.virtual_ground_height`, `virtual_ceil_height` | Vertical map boundaries; effective heights are adjusted by grid and inflation settings |
| `rog_map.map_size`, map origin, `map_sliding.enable` | Map storage and sliding behavior; these do not establish a fixed venue geofence |
| `fsm.click_height` | Clicked goal Z only; does not bound the entire trajectory |

The inspected velocity constraints use optimization penalties, and backup trajectories can exceed the configured nominal velocity bound. A negative `penna_jerk` disables its jerk penalty, so changing `max_jerk` alone may have no effect. Final limits must be verified on both committed and backup trajectories and during execution.

The eventual configuration needs fixed horizontal and vertical boundaries, goal and complete-trajectory validation, execution monitoring, and tested stopping/hover behavior. A visible wall in the occupancy map or a chosen map size does not by itself guarantee boundary protection. No final parameter values or flight-clearance guarantees have been accepted yet.

## 12. Remaining Work

### Build and restart checkpoint

Today's last confirmed action was detecting the covariance repair already present in the FAST_LIO source. Available memory was approximately 11 GiB. **No successful build output or repaired-binary restart has been reported.** The final running/stopped state of the terminals has not been confirmed.

On the next session, inspect existing processes first. Stop FAST_LIO/RViz in Terminal 2, SUPER in Terminal 5, and the optional transport test in Terminal 8 before rebuilding. Keep external vision fusion disabled. Do not start duplicate driver or MAVROS instances.

In a separate **Terminal 9**, source the dependencies and back up the existing installed workspace before building:

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
cd ~/fastlio_ws
tar -czf ~/fastlio_install_before_covfix_$(date +%Y%m%d_%H%M%S).tar.gz install
```

After the backup succeeds, run:

```bash
MAKEFLAGS="-j2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select fast_lio --executor sequential
```

Wait for a successful build summary. If it fails, retain the error output and resolve it before restarting; the old installed executable may still exist. Re-source `~/fastlio_ws/install/setup.zsh` in Terminal 2 and restart FAST_LIO using Quick Start with the sensor stationary.

Then inspect `/Odometry` from initialization onward: confirm covariance is populated for the current pose, finite and consistent with the intended fixed-axis convention. Run the timestamp-age check in Terminal 3. Restart SUPER and repeat the planning-load freshness check only after localization is ready. Restart the optional transport test only if needed and only with `EKF2_EV_CTRL = 0`.

FAST_LIO initialization can change the planning frame. Recheck clicked goal height and perform a new mounting/world-alignment assessment; do not reuse an earlier yaw or translation offset.

### Confirmed today

- [x] Recover the deployment after reboot on the replacement computer.
- [x] Check system time, Ethernet interfaces, and flight-controller USB identity.
- [x] Confirm MID360 output and isolated IMU input at approximately 200 Hz.
- [x] Recover fresh approximately 10 Hz odometry after restarting FAST_LIO.
- [x] Confirm real-data SUPER mapping, clicked-goal planning, and approximately 100 Hz command publication.
- [x] Observe stable odometry timestamp age for two minutes under planning load.
- [x] Confirm MAVROS connection, disarmed state, and `EKF2_EV_CTRL = 0`.
- [x] Recheck MAVROS time-sync samples in the millisecond range.
- [x] Demonstrate raw pose-covariance ROS transport at approximately 10 Hz.
- [x] Confirm the source covariance repair is present; record build and runtime verification as pending.
- [x] Record final venue dimensions, speed, height, and boundary requirements.

### Localization integration

- [ ] Complete the FAST_LIO build and verify the repaired covariance from the installed executable.
- [ ] Validate mounting transforms with synchronized motion tests.
- [ ] Define the FAST_LIO-to-PX4 world rotation and origin, including gravity alignment.
- [ ] Validate measurement rate, timestamps, covariance, and delay.
- [ ] Confirm PX4 receives visual-odometry measurements, not merely that MAVROS subscribes to a ROS topic.
- [ ] Configure only the external measurements that are actually available.
- [ ] Verify PX4 fusion and estimator validity from logs and status.
- [ ] Run longer-duration freshness checks and investigate the earlier delay if it recurs.

### Control integration

- [ ] Implement the SUPER-to-MAVROS controller or setpoint bridge.
- [ ] Validate coordinate conventions, units, and setpoint semantics.
- [ ] Provide continuous setpoints before entering Offboard.
- [ ] Handle stale localization, stale trajectories, and communication loss.
- [ ] Implement trajectory completion behavior and hover.
- [ ] Validate mode transitions and control behavior in simulation and bench tests.
- [ ] Perform controlled flight validation.

`/planning/pos_cmd` uses `mars_quadrotor_msgs/msg/PositionCommand`. It is not a native PX4 setpoint message.

Do not copy its fields directly into an unrelated flight-control interface. In particular, its thrust-related fields must not be assumed to represent PX4 normalized throttle.

### Venue configuration and validation

- [ ] Measure the aircraft envelope and establish floor/height reference, fixed venue frame, and demonstration route.
- [ ] Select and verify velocity, acceleration, jerk, collision margins, and effective vertical boundaries.
- [ ] Implement fixed venue boundary checks for goals, complete trajectories, backup trajectories, and execution.
- [ ] Validate braking, tracking error, and clearance before attempting the target 2 m/s demonstration.

### Operator and task management

- [ ] Configure and verify the ground-to-onboard network connection.
- [ ] Provide an operator interface for start, status, goals, and task cancellation.
- [ ] Implement takeoff, hover, task execution, and landing states.
- [ ] Implement multiple-goal sequencing with validated arrival criteria.
- [ ] Verify RC mode switching and manual takeover.
- [ ] Define behavior when the ground connection is lost.
- [ ] Add an automatic startup gate if unattended service startup is required.

## 13. Development Checks

Run the transport test on a separate test host with ROS 2 Humble sourced, from this repository's root:

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 PYTHONPATH="$PWD" python3 tests/test_raw_transport.py
```

This test creates its own `/Odometry` publisher and simulated `/mavros/param` service. Use a dedicated ROS domain with no hardware nodes; it does not use a real flight controller. It checks field preservation, invalid-input rejection, the nonzero/stale parameter gate, and resumed forwarding.

To check the repaired C++ publisher without building the whole ROS package, install `g++` and Eigen headers on the test host and run from the repository root:

```bash
python3 tests/verify_fastlio_pose_covariance.py ~/fastlio_ws/src/FAST_LIO/src/laserMapping.cpp /usr/include/eigen3
```

This compiles the actual function against message/publisher stubs and checks covariance block order, fixed-axis rotation/cross terms, first/second-frame publication, and unchanged pose/time/TF. It does not replace the real FAST_LIO build or PX4 fusion validation.

## 14. References

- [Project repository](https://github.com/Haofei-Hu/Demo_HKU_open-day_SUPER)
- [SUPER](https://github.com/hku-mars/SUPER)
- [FAST_LIO ROS2 branch](https://github.com/hku-mars/FAST_LIO/tree/ROS2)
- [Recorded FAST_LIO MID360 configuration](https://github.com/hku-mars/FAST_LIO/blob/a4743b095409588842a5b30ddfa27e29d2f99164/config/mid360.yaml)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [MAVROS 2.14 PX4 launch file](https://github.com/mavlink/mavros/blob/2.14.0/mavros/launch/px4.launch)
- [Chrony configuration manual](https://chrony-project.org/doc/4.2/chrony.conf.html)
- [Chrony command manual](https://chrony-project.org/doc/4.2/chronyc.html)
- [PX4 external position estimation](https://docs.px4.io/main/en/ros/external_position_estimation)
- [PX4 Offboard mode](https://docs.px4.io/main/en/flight_modes/offboard)

This repository records deployment progress. Successful mapping, trajectory generation, and MAVLink communication are separate milestones from validated autonomous flight.
