# Demo_HKU_open-day_SUPER

An indoor autonomous flight project for the HKU Open Day demonstration, integrating Livox MID360, FAST_LIO, SUPER, MAVROS, and PX4.

**Status — October 16, 2026:** the complete demonstration supervisor and automatic test entrypoint are implemented: 2 m takeoff, 4.5 m outbound, return, six-second hover and confirmed landing. The full synthetic ROS lifecycle and an injected return-failure test have passed. The new ground workflow captures the actual scene and runs real SUPER with virtual vehicle feedback in an isolated domain; the first onboard attempt passed preparation but failed backup planning for the 4.5 m route in a location subsequently reported to have approximately 1.5 m forward room. A separate 1 m rehearsal is available; the subsequent 1 m real-SUPER/virtual-vehicle sequence has been reported complete, with detailed report archival pending. Physical full-mission flight remains unverified. See [the complete mission test](docs/DEMONSTRATION_MISSION.md).

**Status — October 15, 2026:** the session-relative map correction has passed an aircraft ground audit. The controller accepted 217 matched references and retained `AUDIT_END` without faults across 18 status reports. Concurrent 30-second sampling found fresh raw, aligned, native-vision and FC poses; native-vision maximum age/gap was 35/36 ms. All remote preparation checks passed. RC/manual POSCTL remain required for supervised flight; this result does not validate airborne SUPER tracking or the 2 m/s demonstration. See [the recorded audit](data/observed_2026-10-15/session_map_ground_audit.json).

The automatic ground runner subsequently completed the full aircraft workflow with `RESULT: SUCCESS`: fresh quaternion capture/application, session-map creation, one audit and saved report. It accepted 217 references with no controller fault. See [the automatic run](data/observed_2026-10-15/automatic_ground_success.json); long-term localization stability and supervised flight remain unverified.

**Status — October 14, 2026:** the aircraft has completed OFFBOARD entry, automatic arming, takeoff, a hover test configured for 1 m above the floor with a 6-second hold, and automatic landing. Repeated hover tests are complete; approximately 10 cm lateral motion remains a tuning/estimation diagnostic item. The 50 Hz localization/vision chain and low-speed advertised-motion checks have passed ground samples. A disarmed 1 m vertical reference test progressed through `TRACK` to `END_HOLD`, accepting 684 samples and holding forward/left/FC-floor-height `[-0.0226, -0.0447, 1.0395] m` while suppressing subsequent restart references. These locked-frame endpoint offsets are within 10 cm per axis. A later 120-second stationary observation with online IMU–LiDAR extrinsic estimation disabled measured maximum forward/left/up coordinate excursions below 2 cm in both external and FC poses. A follow-up vertical reference trial in that fixed-extrinsic session latched a continuity fault after 409 accepted samples; detailed transition diagnostics are prepared before flight integration. See the [vertical reference result](data/observed_2026-10-13/vertical_goal_ground.json), [stationary observation](data/observed_2026-10-13/fastlio_stationary_diagnosis.json) and [integration progress](docs/PROGRESS_2026-10-13.md). Native SUPER trajectory flight, airborne tracking accuracy and obstacle avoidance remain the next integration stage. The cause of earlier stationary attitude/position inconsistencies and stability over a complete demonstration remain under investigation.

## Quick Start: restore the aircraft after reboot

For the complete demonstration test, after installing the matching script release:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry
```

This automatically restarts the grounded stack, captures the quaternion and map, and rehearses the entire route using real SUPER and a virtual FC/vehicle in private ROS domain 197. The actual aircraft stays disarmed. The fixture geometry is explicitly synthetic while aircraft dimensions and takeoff wall distances remain unmeasured. The final `RESULT`, scope and saved report distinguish this rehearsal from flight. See [installation, stages and report interpretation](docs/DEMONSTRATION_MISSION.md).

For the current short location, append `--route-profile short-1m` to the complete-test command. It retains climb, return, final hover and landing, but requests 1 m outbound. Its report cannot count as a completed 4.5 m demonstration. The actual map and backup planner still have to admit that route.

During virtual integration, append `--motion-policy observe` to record speed/acceleration target exceedances as warnings while retaining position, polynomial, freshness and continuity checks. Reports retain actual peaks and counts; completion with this option does not establish 2/1 motion compliance. The demonstration preparation/controller CLIs now default to observation as well, with 2.4 m/s and 1.5 m/s² warning advisories. Strict 2/1 rejection remains an explicit diagnostic option; physical startup and position/freshness checks remain required.

The following short tests remain available as diagnostics.

For automatic alignment capture and the complete disarmed audit, use the [automatic ground test](docs/AUTOMATED_GROUND_TEST.md) after installing the matching script release:

```bash
python3 ~/Downloads/super_ground_test.py --restart --fixed-lio-extrinsics
```

This captures and applies a fresh quaternion, prepares the session map, runs one ground audit and saves a structured report. It preserves flight initiation as a separate operation; a passing ground result does not initiate takeoff. The manual steps below remain available for diagnosis.

For the separate 5 m forward reference audit, capped at 2 m/s and 1 m/s²:

```bash
python3 ~/Downloads/super_ground_test.py --restart --fixed-lio-extrinsics \
  --test-profile forward-5m-2mps
```

This also remains disarmed and produces debug output only. The report records the actual sampled planner peaks and `SUCCESS`/`FAIL`; it does not establish physical 2 m/s tracking. Map capture retains the 2.5 cm per-axis limit and waits up to 12 seconds for a complete stable three-second window. See [the test profile](docs/AUTOMATED_GROUND_TEST.md#forward-5-m-ground-audit) for installation and result interpretation.

An earlier fresh-session trial exposed a planner map-height mismatch while localization and parameter checks remained healthy. Managed `low-speed-3d` startup now generates frozen session-relative vertical map bounds without changing the source profile, obstacle clouds or flight margins. Aircraft capture, planning and a complete disarmed audit subsequently passed using this profile; intermittent startup freshness and capture failures still require repeatability evidence. See [October 15 progress](docs/PROGRESS_2026-10-15.md).

The managed disarmed SUPER-to-PX4 transport is verified: native position/yaw setpoints publish at approximately 20 Hz and PX4 reports fresh position-control intent. Use the [transport workflow](docs/SESSION_STARTUP.md#managed-disarmed-transport-test) and `wire-stop` to stop the sender; this is separate from trajectory flight. See [October 14 integration](docs/PROGRESS_2026-10-14.md).

The [one-goal handover audit](docs/SUPER_HANDOVER.md) has passed a sustained aircraft ground trial: 217 references accepted, `AUDIT_END` retained for more than six seconds, and concurrent 30-second input sampling fresh. A repeated audit held `[0.3462, -0.0154, 0.0757] m` in forward/left/FC-floor-height coordinates with fresh concurrent native-vision sampling; see [the latest ground result](data/observed_2026-10-14/super_handover_vision_audit.json). The auditor emits debug output only; planner-driven aircraft flight remains pending. `super_session.py handover-check` preserves localization/planning, and `/super_flight/audit` explicitly requests the one ground goal after `READY`. The aircraft read-only `super_session.py flight-check` snapshot passed all remote preparation checks, with 7.12 mm FC/vision position separation and 0.01145° yaw difference. RC/manual POSCTL remain the two supervised-flight input blockers; see [the snapshot](data/observed_2026-10-14/remote_flight_readiness.json).

For fewer terminal windows, use the [compact tmux startup](docs/SESSION_STARTUP.md). It groups persistent nodes in the background, preserves per-node logs, and keeps flight initiation explicit. After the one-time setup, the command sequence is:

```bash
python3 ~/Downloads/super_session.py start
python3 ~/Downloads/super_session.py align
```

For the current fixed-extrinsic ground comparison, use `start --fixed-lio-extrinsics` when creating a new session, then run the same fresh survey and vision sequence. Plain `start` uses the original YAML-based baseline; the override does not edit that file. See the [comparison workflow](docs/SESSION_STARTUP.md).

Then use the current measured quaternion with `super_session.py vision QX QY QZ QW`. Check fusion and preflight, and use `super_session.py hover --execute` for the existing hover controller. The separate `/first_hover/start` service still initiates the flight. For disarmed ground planning after fusion is valid:

```bash
python3 ~/Downloads/super_session.py planner --planner-profile low-speed-3d
# Before a goal, start bounded preview and wait for WAIT_TRAJECTORY:
python3 ~/Downloads/super_session.py reference-preview
# After a nonempty occupancy map and preview readiness:
python3 ~/Downloads/super_session.py goal
# Alternative: a 1 m vertical reference, on the actual floor and before any goal:
# python3 ~/Downloads/super_session.py goal --goal-height 1.0
# Stop planning only; localization and vision remain running:
python3 ~/Downloads/super_session.py planner-stop
```

These commands manage SUPER and a debug-coordinate preview. They do not execute a trajectory on the aircraft. The [session runbook](docs/SESSION_STARTUP.md#ground-planning-in-the-same-session) describes readiness checks and prerequisites. For the next low-speed preview, use `planner --planner-profile low-speed-3d` after `planner-stop`. The motion-profile helper also saves the final 2 m/s / 1 m/s² candidate; it is not a flight-ready configuration. After the low-speed goal is producing commands, `super_session.py planner-check` samples reference motion limits and freshness without flight output. The separate [bounded ground reference preview](docs/BOUNDED_REFERENCE_PREVIEW.md) adds fixed forward/floor coordinates, point-envelope checks and committed-trajectory endpoint holding. A stationary aircraft test reached `END_HOLD` and suppressed restart references; native flight execution remains pending. An optional [live RViz2 view](docs/SESSION_STARTUP.md#live-rviz2-view) shows clouds, map, and planning trajectories. The numbered commands below remain available for manual recovery.

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
| MID360 → FAST_LIO | Operational | Approximately 50 Hz odometry/vision in current samples; 17–23 ms scans in the recorded timing excerpt |
| Localization timing | Fixed and deployed | IMU Best Effort subscription removed the observed backlog; recent ground pose ages were approximately 20–35 ms |
| FAST_LIO → PX4 external vision | Operational | Position, height, and yaw fusion through MAVROS; fresh `vehicle_visual_odometry` and valid local position/velocity |
| Fixed-position flight | **Successful on aircraft** | Complete automatic takeoff, hover, and landing sequence |
| SUPER mapping and planning | Ground integration complete | Occupancy map, trajectory generation, replanning, and converted command preview |
| SUPER → PX4 ground transport | Verified on aircraft | Approximately 20 Hz native position/yaw publication and fresh PX4 position-control intent while disarmed |
| SUPER → flight trajectory execution | Prototype under validation | Aircraft ground audit held AUDIT_END beyond six seconds with fresh concurrent sampling; airborne handover pending |
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
python3 tests/test_super_3d_goal_profile.py
python3 tests/test_ground_height_goal.py
python3 tests/test_bounded_reference.py
python3 tests/test_super_exact_start.py
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

The deployment, reference, bag-export, motion-profile, height-goal, session-lifecycle, control-chain metric, bounded-reference and exact-start patch suites contain 58 unit tests. Separate 3D-goal generation, session-profile checks and bounded-reference checks passed for this update; the bounded-reference ROS fixture also passed; earlier motion-profile, control-chain metric, reference and bag-export results are recorded separately in the progress report. Real tmux dummy-node and isolated ROS parameter-service checks also passed for the compact manager. Isolated ROS checks cover pose/covariance conversion, planning preview, continuous vision forwarding, and one-shot hover control. The generated standalone controller matches the canonical implementation. Physical takeoff/hover/landing success is recorded separately from these software checks.

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
