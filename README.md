# Demo_HKU_open-day_SUPER

Deployment notes for an HKU Open Day UAV demonstration using Livox MID360, FAST_LIO, SUPER, MAVROS, and PX4.

**Last updated: 2026-10-06**

**Current stage:** LiDAR sensing, FAST_LIO localization, SUPER planning, and MAVROS–PX4 communication have been demonstrated. Host clock synchronization has now been configured and checked. External localization fusion, trajectory execution, and autonomous flight remain unfinished.

This README documents the existing Ubuntu deployment. It is not a complete fresh-install guide.
## Quick Start

Run these commands on the **onboard Ubuntu computer** using separate zsh terminals.

Before starting, connect to the network and wait until the computer's date and time are correct. Keep QGroundControl closed while MAVROS uses the flight controller USB port.

Keep each process terminal open. This procedure starts localization, planning, and communication; it does not automatically arm or take off.

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

### Terminal 3 — Visualization Transform

```bash
source /opt/ros/humble/setup.zsh
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id world --child-frame-id camera_init
```

This defines the current visualization convention. It does not align FAST_LIO with the PX4 navigation frame.

### Terminal 4 — SUPER

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
mkdir -p ~/super_ws/src/SUPER/super_planner/log/replan_logs ~/super_ws/src/SUPER/super_planner/log/cmd_logs
ros2 run super_planner fsm_node --ros-args -p config_name:=mid360_real_ros2.yaml
```

`Wait for goal` is normal before submitting a goal.

### Terminal 5 — MAVROS

Close any QGroundControl instance directly connected to the flight controller USB port.

```bash
source /opt/ros/humble/setup.zsh
ros2 launch mavros px4.launch fcu_url:="serial:///dev/ttyACM0:2000000"
```

### Terminal 6 — Body-Pose Diagnostic (Optional)

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
python3 ~/Downloads/fastlio_body_debug.py
```

This publishes `/super_debug/body_pose` for inspection. It does not send flight commands.

### Terminal 7 — Checks

```bash
source /opt/ros/humble/setup.zsh
```

Run each check separately:

```bash
timeout 10s ros2 topic hz /Odometry
```

```bash
timeout 10s ros2 topic echo /mavros/state --once
```

```bash
timeout 10s ros2 topic echo /mavros/timesync_status --field round_trip_time_ms
```

Expected basic results:

- `/Odometry`: approximately 10 Hz.
- `/mavros/state`: `connected: true`.
- Time-sync output: assess the current measurements; the previous multi-second delay remains a troubleshooting item.

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
## 1. Deployment Status

The table distinguishes previous successful tests from the latest startup session.

| Component | Status | Evidence / remaining work |
| --- | --- | --- |
| Ubuntu 22.04, ROS 2 Humble, zsh | Installed | Working deployment environment |
| MID360 network and driver | Previously verified | CustomMsg LiDAR approximately 10 Hz; IMU approximately 200 Hz |
| FAST_LIO localization | Previously verified | `/Odometry` and `/cloud_registered` approximately 10 Hz |
| SUPER simulation | Previously verified | Click-goal demo succeeded after correcting the backup trajectory configuration |
| SUPER with real FAST_LIO data | Previously verified | Occupancy map and committed trajectory displayed in RViz |
| SUPER trajectory generation | Previously verified | `GenerateExpTrajectory SUCCESS`, `FOLLOW_TRAJ`, and `ReplanOnce succeed` |
| SUPER command output | Previously verified | `/planning/pos_cmd` approximately 99.5–100 Hz |
| Nominal body-pose diagnostic script | Previously verified | `/super_debug/body_pose` approximately 10 Hz; timing gaps require further assessment |
| QGroundControl direct USB connection | Previously verified | Firmware and MAVLink information retrieved |
| MAVROS–PX4 USB connection | Recovered after reboot | Latest reported state: `connected: true`, `armed: false`, `mode: AUTO.LOITER` |
| Diagnostic pose relay to MAVROS | Previously verified at ROS topic level | One publisher and one MAVROS subscriber; approximately 10 Hz |
| Chrony clock synchronization | Verified on 2026-10-06 | Startup synchronization check passed; runtime automatic stepping disabled for the current Chrony process |
| MAVROS time synchronization | Pending recheck | Previous measurements showed approximately 8–9 seconds of round-trip time |
| FAST_LIO localization fused by PX4 | Not completed | Last confirmed `EKF2_EV_CTRL` value was `0` |
| SUPER commands converted to PX4 setpoints | Not completed | Controller/bridge implementation and validation required |
| Autonomous takeoff and task management | Not implemented | Takeoff, hover, goal sequencing, and landing states required |
| Offboard flight and autonomous obstacle avoidance | Not validated | No completed real-flight execution test |

A successful planner state does not establish that the aircraft followed the trajectory.

Closing process terminals or rebooting the computer normally requires starting the software again. Previous test results are not a substitute for checking the current session.

## 2. Hardware and Software

| Item | Recorded configuration |
| --- | --- |
| Onboard computer | Ubuntu 22.04 |
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
~/Downloads/QGroundControl-x86_64.AppImage
```

These are paths on the onboard Ubuntu computer. Check the repository contents separately to determine which configuration files and scripts have been committed.

### MID360 network

| Item | Value |
| --- | --- |
| Ethernet interface | `enp4s0` |
| Onboard computer IP | `192.168.50.125/24` |
| MID360 IP | `192.168.50.163` |

Useful checks:

```bash
ip -br addr
ping -I enp4s0 -c 4 192.168.50.163
ip neigh show dev enp4s0
```

The driver startup log reports the configuration file actually loaded. Preserve the full JSON structure when changing addresses.

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

A separate diagnostic path has been tested:

```text
FAST_LIO /Odometry
  |
  v
fastlio_body_debug.py
  |
  v
/super_debug/body_pose
  |
  v
Optional topic relay
  |
  v
/mavros/vision_pose/pose
```

The diagnostic relay establishes ROS-side transport. It does not establish PX4 estimator fusion or correct alignment with the PX4 local frame.

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

### Why this was added

The onboard computer can change its system time after obtaining a network connection during boot.

A clock step while ROS nodes are running can make timestamps discontinuous and affect time-dependent processing. This is a possible contributor to earlier timing problems, but it has not been established as the sole cause of the MAVROS communication failures.

The startup policy is:

1. Connect to a valid time source.
2. Complete initial synchronization.
3. Disable further automatic clock stepping for the current Chrony process.
4. Start the LiDAR and ROS nodes.
5. Keep Chrony running for gradual clock corrections.

### Verified configuration

Chrony is now installed and active. `systemd-timesyncd` is inactive.

Relevant entries in `/etc/chrony/chrony.conf`:

```text
rtcsync
makestep 1 3
```

`makestep 1 3` allows a clock step for an offset larger than one second during the first three clock updates after Chrony starts.

The limit counts clock updates, not seconds after boot. A late network connection can therefore still cause a startup step unless the synchronization gate has been completed.

The existing NTP pool configuration was retained.

### Required startup gate

Run this in a check terminal before starting Livox, FAST_LIO, MAVROS, or SUPER:

```bash
chronyc waitsync 60 0.01 10 1 && sudo chronyc makestep 1 0
```

The command:

- Checks synchronization up to 60 times, approximately once per second.
- Requires the reported remaining clock correction to be below 10 milliseconds.
- Requires the reported frequency-estimation skew to be below 10 ppm.
- Runs the second command only when the synchronization check succeeds.
- Disables further automatic stepping for the current Chrony process.

Expected final response:

```text
200 OK
```

Run the gate once per computer boot, before starting the ROS processes. Run it again if Chrony is restarted.

This is a manual startup gate. No automatic ROS startup service has been configured to enforce it.

If synchronization fails, inspect the time source before starting the stack:

```bash
chronyc sources -v
chronyc tracking
```

An offline cold start needs an appropriate local time source or a separately designed offline startup policy.

### Latest successful result

Recorded on 2026-10-06:

```text
Startup gate: passed
Chrony command response: 200 OK
Remaining correction: approximately 28 microseconds
Skew: 2.658 ppm
Leap status: Normal

chrony: active
systemd-timesyncd: inactive
```

The reported remaining correction is not a measurement of absolute clock accuracy.

Status checks:

```bash
chronyc tracking
systemctl is-active chrony systemd-timesyncd
```

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

The terminal numbers below are fixed labels for this runbook. Keep terminals 1–6 open while their processes are required. Terminal 7 is for temporary checks.

Do not start duplicate driver or MAVROS instances.

### Before starting any ROS process

Complete the clock synchronization gate from Section 4.

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

### Terminal 3 — Visualization frame convention

```bash
source /opt/ros/humble/setup.zsh
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id world --child-frame-id camera_init
```

Keep this terminal running.

The current visualization setup treats `world` and `camera_init` as coincident.

This convention does not align FAST_LIO with the PX4 local frame, establish geographic heading, or calibrate the LiDAR-to-airframe mounting transform.

### Terminal 4 — SUPER with real sensor data

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

### Terminal 5 — MAVROS

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

### Terminal 6 — Nominal body-pose diagnostic

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

### Terminal 7 — Checks

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
| `fsm.click_height` | `-0.19` |
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

The setting was changed to:

```yaml
click_height: -0.19
```

This is a fixed coordinate in the current planning frame. It is not height above the floor or an automatic “maintain current altitude” command.

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

Previous rotation tests showed broadly consistent changes in diagnostic and PX4 yaw. This is preliminary evidence, not a completed calibration.

### MAVROS topic direction

| Topic | Direction |
| --- | --- |
| `/mavros/vision_pose/pose` | ROS pose input to MAVROS |
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

### Optional Terminal 8 — Transport-only pose relay

This is a diagnostic operation, not part of the default startup sequence.

Use it only after confirming that external vision fusion remains disabled and the aircraft is disarmed.

If needed, install the relay package:

```bash
sudo apt install ros-humble-topic-tools
```

Run:

```bash
source /opt/ros/humble/setup.zsh
ros2 run topic_tools relay /super_debug/body_pose /mavros/vision_pose/pose
```

The relay can run without printing continuous output.

Inspect it from Terminal 7:

```bash
ros2 topic info /mavros/vision_pose/pose --verbose
timeout 10s ros2 topic hz /mavros/vision_pose/pose
```

A previous test showed:

```text
Publisher count: 1
Subscription count: 1
Rate: approximately 10 Hz
```

This confirms topic transport only. Matching topic connections do not establish correct coordinate alignment, timing, or fusion.

## 9. MAVROS, PX4, and Timing Findings

### External vision fusion

The last confirmed value was:

```text
EKF2_EV_CTRL = 0
```

At that point, PX4 external vision fusion was disabled.

Earlier estimator status also reported invalid horizontal-position flags and constant-position mode. The presence of `/mavros/local_position/pose` therefore did not establish usable horizontal localization.

Before enabling fusion, validate:

- Body-frame mounting transformation.
- Local-world origin and heading convention.
- Sensor timestamps and MAVROS time synchronization.
- Pose update rate and delivery gaps.
- Covariance and delay handling.
- Which measurements are actually available for fusion.

The current diagnostic pose stream is approximately 10 Hz. A higher average publication rate created by repeating old measurements would not improve sensor freshness.

### Previous timing anomalies

Before the Chrony setup, MAVROS time-sync measurements included:

```text
round_trip_time_ms: approximately 8300–9200
```

Diagnostic pose checks also showed nonuniform message intervals, including gaps of several hundred milliseconds.

The causes were not isolated. Startup discovery warnings alone do not explain all measured timing gaps.

After the clock setup, remeasure both the MAVROS time-sync stream and pose delivery intervals before proceeding with estimator fusion.

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

1. Stop MAVROS in Terminal 5 with Ctrl+C.
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

Keep the publisher in Terminal 6 and run the check in Terminal 7.

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

## 11. Remaining Work

### Immediate continuation

- [x] Install and enable Chrony.
- [x] Confirm that systemd-timesyncd is inactive.
- [x] Pass the startup synchronization gate.
- [x] Disable runtime automatic stepping for the current Chrony process.
- [ ] Restart Livox and FAST_LIO after the clock setup.
- [ ] Recheck odometry and diagnostic pose delivery intervals.
- [ ] Restart SUPER and confirm real-data planning.
- [ ] Restart MAVROS and remeasure time synchronization.
- [ ] Confirm current PX4 estimator status and external vision parameters.

### Localization integration

- [ ] Validate mounting transforms with synchronized motion tests.
- [ ] Define the relationship between the FAST_LIO planning frame and PX4 navigation frame.
- [ ] Validate measurement rate, timestamps, covariance, and delay.
- [ ] Configure only the external measurements that are actually available.
- [ ] Verify PX4 fusion and estimator validity from logs and status.

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

### Operator and task management

- [ ] Configure and verify the ground-to-onboard network connection.
- [ ] Provide an operator interface for start, status, goals, and task cancellation.
- [ ] Implement takeoff, hover, task execution, and landing states.
- [ ] Implement multiple-goal sequencing with validated arrival criteria.
- [ ] Verify RC mode switching and manual takeover.
- [ ] Define behavior when the ground connection is lost.
- [ ] Add an automatic startup gate if unattended service startup is required.

## 12. References

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
