# Demo_HKU_open-day_SUPER

An indoor autonomous flight project for the HKU Open Day demonstration, integrating Livox MID360, FAST_LIO, SUPER, MAVROS, and PX4.

**Status — October 12, 2026:** the complete fixed-position flight sequence has succeeded on the aircraft: **OFFBOARD entry, automatic arming, takeoff, a hover test configured for 1 m above the floor and a 6-second hold, followed by automatic landing**. This establishes the first flight-control milestone. Repeated hover tests are complete; approximately 10 cm lateral motion remains a tuning/estimation diagnostic item. Planner-driven trajectory flight and obstacle avoidance are the next integration stage.

## Quick Start: restore the aircraft after reboot

For fewer terminal windows, use the [compact tmux startup](docs/SESSION_STARTUP.md). It groups persistent nodes in the background, preserves per-node logs, and keeps flight initiation explicit. After the one-time setup, the command sequence is:

```bash
python3 ~/Downloads/super_session.py start
python3 ~/Downloads/super_session.py align
```

Then use the current measured quaternion with `super_session.py vision QX QY QZ QW`. Check fusion and preflight, and use `super_session.py hover --execute` for the existing hover controller. The separate `/first_hover/start` service still initiates the flight. For disarmed ground planning after fusion is valid:

```bash
python3 ~/Downloads/super_session.py planner
# After a nonempty occupancy map and coordinate-preview readiness:
python3 ~/Downloads/super_session.py goal
# Stop planning only; localization and vision remain running:
python3 ~/Downloads/super_session.py planner-stop
```

These commands manage SUPER and a debug-coordinate preview. They do not execute a trajectory on the aircraft. The [session runbook](docs/SESSION_STARTUP.md#ground-planning-in-the-same-session) describes readiness checks and prerequisites. For the next low-speed preview, use `planner --planner-profile low-speed` after `planner-stop`. The motion-profile helper also saves the final 2 m/s / 1 m/s² candidate; it is not a flight-ready configuration. After the low-speed goal is producing commands, `super_session.py planner-check` samples reference motion limits and freshness without flight output. An optional [live RViz2 view](docs/SESSION_STARTUP.md#live-rviz2-view) shows clouds, map, and planning trajectories. The numbered commands below remain available for manual recovery.

Run these commands in separate terminals on the **onboard Ubuntu computer** through NoMachine. This assumes the ROS workspaces and integration scripts are already installed. A normal reboot does not require recompilation or reinstallation.

Keep the aircraft on its actual takeoff floor, stationary and disarmed, with RC connected and the arm switch OFF. Start terminals **1–4**, perform the session pairing in **5**, start **6–8**, and then use **11** for hover. Terminals **9/10** are optional ground planning tools and stay stopped during the hover test.

### Before opening the ROS terminals

Check the current Wi-Fi address and remote-desktop service:

```bash
nmcli device status
ip -4 -br addr show dev wlx6c1ff7880bdf
sudo /usr/NX/bin/nxserver --status
```

Connect NoMachine to the current IP using NX, port 4000. The last recorded address is `192.168.6.146`; DHCP may change it. See [remote recovery](docs/REMOTE_RECOVERY_2026-10-09.md) if the interface or desktop is unavailable.

### Terminal 1: MID360

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Terminal 2: FAST_LIO

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" \
  -p use_sim_time:=false \
  -p publish.map_en:=false \
  -p publish.path_en:=false \
  -p publish.scan_publish_en:=true \
  -p publish.scan_bodyframe_pub_en:=false
```

Wait for map initialization. The recent healthy baseline was approximately 20–35 ms odometry age, `irx≈0`, and `lidar_q=0`.

### Terminal 3: MAVROS

```bash
source /opt/ros/humble/setup.zsh
ros2 launch mavros px4.launch \
  fcu_url:="serial:///dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00:2000000" \
  gcs_url:="udp://127.0.0.1:14555@127.0.0.1:14550"
```

Verify the serial path with `ls -l /dev/serial/by-id/` if the FC is not connected. MAVROS owns the onboard USB connection; QGC uses UDP forwarding.

### Terminal 4: FC-centre attitude diagnostics

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/fastlio_body_debug.py
```

### Terminal 5: parameter pull and fresh session alignment

Keep terminals 6/7/11 stopped while measuring a new alignment. Confirm `connected: true` and `armed: false` before changing fusion parameters:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
ros2 service call /mavros/param/pull \
  mavros_msgs/srv/ParamPull '{force_pull: true}'
timeout 10s ros2 topic echo /mavros/state --once
```

Then, while disarmed:

```bash
ros2 param set /mavros/param EKF2_EV_CTRL 0
ros2 param get /mavros/param EKF2_EV_CTRL
ros2 param get /mavros/param EKF2_HGT_REF
python3 ~/Downloads/odom_age_check.py
python3 ~/Downloads/frame_alignment_check.py
```

Keep the aircraft stationary for the 15-second pairing. Replace the four placeholders below with this run's `Candidate rotation quaternion xyzw`:

```text
python3 ~/Downloads/update_preview_alignment.py QX QY QZ QW
```

Remeasure after each FAST_LIO initialization. Do not reuse a historical quaternion or change the alignment during a locked flight session. Missing helper files can be obtained through the [installation instructions](docs/FIRST_HOVER_2026-10-09.md#download-and-install).

### Terminal 6: aligned pose preview

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/aligned_pose_preview.py
```

Wait for fresh preview output. Its zero is a session origin; terminal 11 captures the actual takeoff-floor pose separately.

### Terminal 7: continuous external-vision relay

With the aircraft still disarmed, restore the recorded fusion configuration in terminal 5:

```bash
ros2 param set /mavros/param EKF2_HGT_REF 3
ros2 param set /mavros/param EKF2_EV_POS_X 0.0
ros2 param set /mavros/param EKF2_EV_POS_Y 0.0
ros2 param set /mavros/param EKF2_EV_POS_Z 0.0
ros2 param set /mavros/param EKF2_EV_QMIN 0
ros2 param set /mavros/param EKF2_EV_CTRL 11
```

Start terminal 7:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/flight_pose_cov_transport.py \
  --ros-args -p publish_to_mavros:=true
```

Keep this relay running throughout flight. Wait for `Session locked; first pose forwarded with original stamp/covariance`, then check in terminal 5:

```bash
ros2 topic info /mavros/vision_pose/pose_cov
timeout 10s ros2 topic hz /mavros/vision_pose/pose_cov
timeout 10s ros2 topic echo /mavros/estimator_status --once
```

Expect one vision publisher, a MAVROS subscriber, approximately 10 Hz, and valid local position/velocity. Stop historical ground/raw vision relays before running this one.

### Terminal 8: QGroundControl and preflight

```bash
~/Downloads/QGroundControl-x86_64.AppImage
```

In **Application Settings → Comm Links**, disable Pixhawk auto-connect and enable UDP. Select POSCTL with RC, keep the arm switch OFF, and release Kill for the disarmed preflight check. In the MAVLink Console:

```text
listener vehicle_visual_odometry 1
listener actuator_armed 1
commander check
```

Confirm fresh visual odometry, `armed: False`, `kill: False`, `termination: False`, and `Preflight check: OK` in POSCTL.

### Terminals 9/10: optional ground planning

For hover, leave both stopped. For a separate disarmed SUPER check, prepare the diagnostic configuration in terminal 5:

```bash
if [ ! -f "$HOME/super_ws/src/SUPER/super_planner/config/mid360_ground_diagnostic_ros2.yaml" ]; then
  python3 ~/Downloads/create_super_ground_diagnostic.py
fi
mkdir -p ~/super_ws/src/SUPER/super_planner/log
```

Terminal 9:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
ros2 run super_planner fsm_node --ros-args \
  -p use_sim_time:=false \
  -p config_name:=mid360_ground_diagnostic_ros2.yaml
```

Terminal 10:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/control_bridge_preview.py
```

The preview waits for a planner command. A small disarmed planning goal can be generated from terminal 5 using `python3 ~/Downloads/send_control_preview_goal.py`. This is separate from flight execution. Stop both processes before continuing with terminal 11.

### Terminal 11: 1 m hover, 6-second hold, automatic landing

Keep terminals 1–4 and 6–8 running. Stop other setpoint senders, including `ground_setpoint_transport.py`. In terminal 5, check the retained speed and OFFBOARD-loss settings:

```bash
for ros_param_name in MPC_XY_VEL_ALL MPC_Z_VEL_ALL MPC_XY_VEL_MAX MPC_Z_VEL_MAX_UP MPC_Z_VEL_MAX_DN COM_OF_LOSS_T COM_OBL_RC_ACT; do
  printf '%s: ' "$ros_param_name"
  ros2 param get /mavros/param "$ros_param_name"
done
```

Expected values: `-1.0, -1.0, 0.5, 0.5, 0.5, 0.5, 4`. Keep the aircraft on the takeoff floor with no external display/power tether. Start terminal 11:

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/first_hover_terminal.py --execute --hover-height 1.0 --hold-seconds 6.0
```

Wait for `Phase: READY` and `GROUND LOCK`; confirm the startup profile shows `hover FC=1 m, hold=6 s`. Starting this process alone does not arm or take off.

**Start flight — terminal 5**, after the current preflight checks pass and the flight area is clear:

```bash
ros2 service call /first_hover/start std_srvs/srv/Trigger '{}'
```

This starts one automatic flight: approximately 2 seconds of ground setpoint streaming → confirmed OFFBOARD → arming → takeoff → fixed XY/yaw hover → AUTO.LAND. The 1 m floor-relative FC target is `z_ground + 0.915 m`, accounting for the 8.5 cm ground clearance.

**Early landing — terminal 5:**

```bash
ros2 service call /first_hover/land std_srvs/srv/Trigger '{}'
```

RC mode takeover has priority. Kill stops motors directly rather than commanding descent. After landing and disarming, restart only terminal 11 for another hover in the same healthy localization session; keep FAST_LIO and the vision chain running. A FAST_LIO restart requires a new alignment and relay session.

For detailed guards, phase timing, check-only mode, and troubleshooting, see the [complete flight workflow](docs/FIRST_HOVER_2026-10-09.md).

## Project progress

| Component | Status | Result |
| --- | --- | --- |
| MID360 → FAST_LIO | Operational | Approximately 10 Hz odometry and registered point clouds |
| Localization timing | Fixed and deployed | IMU Best Effort subscription removed the observed backlog; recent ground pose ages were approximately 20–35 ms |
| FAST_LIO → PX4 external vision | Operational | Position, height, and yaw fusion through MAVROS; fresh `vehicle_visual_odometry` and valid local position/velocity |
| Fixed-position flight | **Successful on aircraft** | Complete automatic takeoff, hover, and landing sequence |
| SUPER mapping and planning | Ground integration complete | Occupancy map, trajectory generation, replanning, and converted command preview |
| SUPER → flight trajectory execution | Pending | Ground preview is separate from the completed hover controller |
| Remote operation | Operational | Mac → NoMachine → onboard Ubuntu desktop, including QGC and terminals |
| Loop recording and XYZ analysis | Tools ready | Bag export, point-cloud inspection, timing gaps, and return-to-marker metrics |
| Venue limits and 2 m/s demonstration | Pending | Surveyed boundaries, clearance margins, and staged flight validation |

The successful flight is a functional milestone. Height error, horizontal drift, timing distributions, and repeatability will be quantified from flight logs and independent measurements in the next test campaign.

- [Current progress and flight milestone](docs/PROGRESS_2026-10-10.md)
- [Complete flight workflow: terminals 1–11 and reboot recovery](docs/FIRST_HOVER_2026-10-09.md)
- [Next development and validation steps](docs/NEXT_STEPS.md)
- [Three-terminal bag recording and XYZ analysis](docs/BAG_XYZ_2026-10-09.md)
- [Remote desktop and AX900 recovery](docs/REMOTE_RECOVERY_2026-10-09.md)

## System architecture

```mermaid
flowchart LR
    L[MID360] --> O[FAST_LIO]
    O --> A[Session alignment and FC-centre conversion]
    A --> V[Continuous external-vision relay]
    V --> M[MAVROS]
    M <--> F[PX4 estimation and position control]
    M <--> Q[QGroundControl]
    M --> H[Fixed-position hover controller]
    H -->|20 Hz position and yaw| M
    O --> S[SUPER mapping and planning]
    S --> P[Ground command-coordinate preview]
    O --> B[Bag recording and XYZ analysis]
```

The onboard computer runs the complete ROS stack and connects to the flight controller over USB. Remote desktop access uses Wi-Fi; the USB link stays onboard during flight. QGC receives the MAVROS UDP forwarding stream rather than opening the flight-controller serial port directly.

The hover controller currently uses 10 Hz localization and sends 20 Hz position/yaw setpoints. Increasing sensor frequency is a separate optimization; duplicate samples and rewritten timestamps are not used to simulate a higher measurement rate.

## Repository structure

```text
scripts/
  localization/   Pose conversion, alignment, and external-vision relays
  flight/         Hover control, references, and parameter helpers
  planning/       SUPER command preview and diagnostic goals
  analysis/       Bag export and message-timing diagnostics
  maintenance/    Source patches, generators, and deployment helper
  layout.json     Source-to-aircraft filename mapping
docs/             Runbooks, progress, and historical checkpoints
configs/          Recorded configuration snapshots
patches/          FAST_LIO source patches
system/           AX900 startup configuration
tests/            Offline and isolated ROS checks
data/             Data storage conventions
```

Three complete standalone files remain at the root for stable download URLs: `first_hover_terminal.py`, `odom_age_check.py`, and `update_preview_alignment.py`.

The [script index](scripts/README.md) explains the categories and deployment rules. The [documentation index](docs/README.md) links the operational runbooks and historical checkpoints.

## Aircraft deployment

The aircraft uses Ubuntu 22.04, ROS2 Humble, and zsh. Livox, FAST_LIO, SUPER, and MAVROS workspaces must already be installed; this repository contains integration scripts and records rather than a complete bundled ROS distribution.

From the repository root on the onboard computer:

```bash
python3 scripts/maintenance/install_scripts.py
```

This installs missing scripts into `~/Downloads` and preserves existing files, including the active session alignment. To update changed files, first land, disarm, and stop affected processes, then use:

```bash
python3 scripts/maintenance/install_scripts.py --overwrite
```

Changed files are backed up before replacement. Localization or alignment updates require a fresh stationary pairing before restoring the vision relay. The deployment helper does not start nodes or change PX4 parameters.

Aircraft commands retain the flat `~/Downloads/script_name.py` paths even though repository sources are categorized. The standalone hover controller can also be downloaded separately using the [flight runbook](docs/FIRST_HOVER_2026-10-09.md#download-and-install).

## Configuration and coordinate conventions

| Parameter | Recorded test configuration |
| --- | --- |
| `EKF2_EV_CTRL` | `11`: horizontal position, height, and yaw; no external velocity |
| `EKF2_HGT_REF` | `3`: vision height reference |
| `EKF2_BARO_CTRL` | `1` |
| `EKF2_EV_DELAY` | `0.0` |
| `EKF2_EV_POS_X/Y/Z` | `0.0`: incoming poses already refer to the FC centre |
| `EKF2_EV_QMIN` | `0` |
| `MPC_XY_VEL_ALL`, `MPC_Z_VEL_ALL` | `-1.0` |
| `MPC_XY_VEL_MAX` | `0.5 m/s` |
| `MPC_Z_VEL_MAX_UP`, `MPC_Z_VEL_MAX_DN` | `0.5 m/s` |
| `COM_OF_LOSS_T` | `0.5 s` |
| `COM_OBL_RC_ACT` | `4`: Land in the recorded firmware |

FAST_LIO publishes `camera_init → body` at the LIO IMU origin. Conversion applies the nominal mounting rotation, lever arm, session world rotation, and fixed translation origin to produce FC-centre ENU poses. MAVROS handles ENU/NED conversion.

The nominal LiDAR mounting has approximately +45° yaw relative to the aircraft and a 10 cm vertical offset above the FC. The recorded internal IMU–LiDAR configuration uses `extrinsic_T=[-0.011,-0.02329,0.04412]` and identity rotation. A paired world quaternion is a session alignment, not an independent mechanical extrinsic calibration.

SUPER's `world` coordinates currently follow the numerical LIO world. The ground command preview converts these into the aligned FC frame. Trajectory flags `1` and `2` represent regular and backup trajectories in the pinned publisher; ground preview success does not implement trajectory flight.

## Development checks

Pure repository checks run from the checkout. ROS graph tests require Humble, MAVROS message/service definitions, and SUPER `PositionCommand`; use an environment and ROS domain isolated from the aircraft.

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:$PWD/scripts/localization:$PWD/scripts/flight:$PWD/scripts/planning:$PWD/scripts/analysis:$PWD/scripts/maintenance${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=180
export ROS_LOCALHOST_ONLY=1
python3 tests/test_repository_layout.py
python3 tests/test_super_session.py
python3 tests/test_super_motion_profiles.py
python3 tests/test_control_chain_metrics.py
python3 tests/test_control_chain_metrics_ros.py
python3 tests/test_hover_reference.py
python3 tests/test_bag_to_xyz.py
python3 scripts/maintenance/build_first_hover_terminal.py --check
python3 tests/test_aligned_pose_preview.py
python3 tests/test_control_bridge_preview.py
python3 tests/test_flight_pose_cov_transport.py
python3 tests/test_first_hover_control.py
FIRST_HOVER_TEST_MODULE=first_hover_terminal python3 tests/test_first_hover_control.py
```

The deployment, reference, bag-export, motion-profile, session-lifecycle, and control-chain metric suites contain 34 unit tests. Motion-profile, session-lifecycle, deployment, and reference checks passed for this update; bag-export checks are recorded separately in the progress report. Real tmux dummy-node and isolated ROS parameter-service checks also passed for the compact manager. Isolated ROS checks cover pose/covariance conversion, planning preview, continuous vision forwarding, and one-shot hover control. The generated standalone controller matches the canonical implementation. Physical takeoff/hover/landing success is recorded separately from these software checks.

## Versions and references

| Component | Recorded version |
| --- | --- |
| Ubuntu / ROS | 22.04 / Humble |
| Livox driver | `livox_ros_driver2` 1.2.7 |
| FAST_LIO | ROS2, `a4743b095409588842a5b30ddfa27e29d2f99164` |
| SUPER | `2ad3419c127a617c6d7df6925e81a14175a9c096` |
| MAVROS | 2.14.0 |
| PX4 | HKUST_NXT_DUAL, 1.17.0, `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |

[SUPER](https://github.com/hku-mars/SUPER) · [FAST_LIO ROS2](https://github.com/hku-mars/FAST_LIO/tree/ROS2) · [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) · [MAVROS](https://github.com/mavlink/mavros/tree/2.14.0) · [PX4 external position estimation](https://docs.px4.io/main/en/ros/external_position_estimation) · [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard)
