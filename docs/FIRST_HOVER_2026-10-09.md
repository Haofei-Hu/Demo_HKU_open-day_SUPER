# Complete fixed-position flight workflow

**Milestone achieved:** the complete automatic flight sequence has succeeded on the aircraft, including takeoff, fixed-position hover, and automatic landing. The latest profile is configured for an FC-centre height of **1 m above the floor** and a **6-second hold**. Current project status is maintained in [October 10 progress](PROGRESS_2026-10-10.md).

This runbook uses the existing 10 Hz localization chain and a 20 Hz position/yaw controller. SUPER trajectory planning is not part of this hover flight. The default controller profile remains 0.5 m / 2 seconds; the tested 1 m / 6-second profile is selected explicitly at startup.

All ROS commands below run on the onboard Ubuntu 22.04 computer with ROS2 Humble and zsh. Mac NoMachine provides access to that desktop; the Mac's local terminal does not run the ROS stack.

## Terminal roles

| Terminal | Process | During hover flight |
| --- | --- | --- |
| 1 | MID360 driver | Running |
| 2 | FAST_LIO: odometry and registered cloud | Running |
| 3 | MAVROS: onboard USB and QGC UDP forwarding | Running |
| 4 | FC-centre attitude diagnostics | Running |
| 5 | Parameters, alignment, checks, start/land services | Command window |
| 6 | Aligned pose preview | Running |
| 7 | Continuous external-vision relay | Running throughout flight |
| 8 | QGroundControl | Running |
| 9 | SUPER ground diagnostic planner | Stopped |
| 10 | Ground planner-command preview | Stopped |
| 11 | Standalone hover controller | Running |

## Reboot preparation

Place the aircraft on its actual takeoff floor, disarmed and stationary, with the RC arm switch OFF. Restore Wi-Fi and the remote desktop, then start terminals 1–4, pair the session in terminal 5, restore 6–8, and finally start 11. Compiled workspace fixes persist across reboot; a new FAST_LIO world requires a new alignment measurement.

On onboard Ubuntu:

```bash
nmcli device status
ip -4 -br addr show dev wlx6c1ff7880bdf
sudo /usr/NX/bin/nxserver --status
ls -l /dev/serial/by-id/
pgrep -af '[f]astlio_mapping|[m]avros_node|[a]ligned_pose_preview.py|[f]light_pose_cov_transport.py'
```

NoMachine uses NX, the current onboard Wi-Fi IP, port 4000, and the local Ubuntu account. The last recorded Wi-Fi address is `192.168.6.146`; DHCP may change it. See [remote recovery](REMOTE_RECOVERY_2026-10-09.md).

## Download and install

The full checkout can install categorized sources into the flat aircraft Downloads directory using the [deployment helper](../scripts/README.md#deploy-into-downloads). For an existing deployment, terminal 11 can be updated independently. Stop the old controller and keep the aircraft disarmed before replacing its file:

```bash
hover_download_dir=$(mktemp -d "$HOME/Downloads/hover_download_XXXXXX")
if curl -q --noproxy '*' --connect-timeout 10 --max-time 30 -fL \
  'https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/main/first_hover_terminal.py' \
  -o "$hover_download_dir/first_hover_terminal.py" \
  && python3 -m py_compile "$hover_download_dir/first_hover_terminal.py"
then
  if [ -f "$HOME/Downloads/first_hover_terminal.py" ]; then
    cp "$HOME/Downloads/first_hover_terminal.py" "$hover_download_dir/previous.py"
  fi
  cp "$hover_download_dir/first_hover_terminal.py" "$HOME/Downloads/first_hover_terminal.py"
fi
```

The download bypasses a stale local proxy while retaining TLS and HTTP checks. Failed downloads do not replace the existing script. Updating a file does not change an already-running instance.

Missing terminal-5 helpers can be downloaded separately:

```bash
check_download_dir=$(mktemp -d "$HOME/Downloads/check_download_XXXXXX")
for script_name in odom_age_check.py update_preview_alignment.py; do
  if [ ! -f "$HOME/Downloads/$script_name" ]; then
    if curl -q --noproxy '*' --connect-timeout 10 --max-time 30 -fL \
      "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/main/$script_name" \
      -o "$check_download_dir/$script_name" \
      && python3 -m py_compile "$check_download_dir/$script_name"
    then
      cp "$check_download_dir/$script_name" "$HOME/Downloads/$script_name"
    else
      break
    fi
  fi
done
```

A script executed through stdin is not automatically saved. Persistent operation uses the file under `~/Downloads`; the standalone controller does not require adjacent integration modules.

## Restore localization: terminals 1–8

### Terminal 1: MID360

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

The recorded LiDAR publication setting is `publish_freq=10.0`. Keep the aircraft still during initialization.

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

Recent ground timing windows showed publication age approximately 0.02–0.03 seconds, `irx≈0`, and `lidar_q=0`. A startup scan warning before map initialization can occur; sustained stale input or queue buildup requires diagnosis.

### Terminal 3: MAVROS

```bash
source /opt/ros/humble/setup.zsh
ros2 launch mavros px4.launch \
  fcu_url:="serial:///dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00:2000000" \
  gcs_url:="udp://127.0.0.1:14555@127.0.0.1:14550"
```

Verify the actual serial-by-id path on the aircraft. MAVROS owns USB; QGC receives local UDP forwarding.

### Terminal 4: FC-centre attitude diagnostics

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/fastlio_body_debug.py
```

### Terminal 5: parameter cache and session pairing

Keep preview/relay terminals 6/7 stopped during alignment. Pull the parameter cache and confirm connected/disarmed state:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
ros2 service call /mavros/param/pull \
  mavros_msgs/srv/ParamPull '{force_pull: true}'
timeout 10s ros2 topic echo /mavros/state --once
```

With the aircraft disarmed:

```bash
ros2 param set /mavros/param EKF2_EV_CTRL 0
ros2 param get /mavros/param EKF2_EV_CTRL
ros2 param get /mavros/param EKF2_HGT_REF
python3 ~/Downloads/odom_age_check.py
python3 ~/Downloads/frame_alignment_check.py
```

The pairing survey runs for 15 seconds. Keep the aircraft stationary and use this session's `Candidate rotation quaternion xyzw`:

```text
python3 ~/Downloads/update_preview_alignment.py QX QY QZ QW
```

`QX QY QZ QW` are placeholders. Historical quaternions are not reusable calibration constants. This measurement aligns the current LIO/PX4 worlds; it does not independently calibrate mechanical mounting.

### Terminal 6: aligned pose preview

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/aligned_pose_preview.py
```

In terminal 5:

```bash
timeout 10s ros2 topic echo /super_debug/aligned_pose_cov \
  geometry_msgs/msg/PoseWithCovarianceStamped --once
```

The preview's first accepted FC-centre position is its fixed origin. Terminal 11 separately captures the actual takeoff-floor pose; a preview zero alone is not a floor-height measurement.

### Terminal 7: continuous external vision

Stop historical raw/aligned/ground vision relays. On the ground, restore the recorded fusion configuration in terminal 5:

```bash
ros2 param set /mavros/param EKF2_HGT_REF 3
ros2 param set /mavros/param EKF2_EV_POS_X 0.0
ros2 param set /mavros/param EKF2_EV_POS_Y 0.0
ros2 param set /mavros/param EKF2_EV_POS_Z 0.0
ros2 param set /mavros/param EKF2_EV_QMIN 0
ros2 param set /mavros/param EKF2_EV_CTRL 11
```

`EV_CTRL=11` enables horizontal position, height, and yaw, without external velocity. Zero EV_POS is appropriate because conversion already places the incoming pose at the FC centre.

Then start terminal 7:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/flight_pose_cov_transport.py \
  --ros-args -p publish_to_mavros:=true
```

Wait for `Session locked; first pose forwarded with original stamp/covariance`. This relay continues after arming while freezing geometry, origin, and source identities. Its default without `publish_to_mavros` is debug-only.

In terminal 5:

```bash
ros2 topic info /mavros/vision_pose/pose_cov
timeout 10s ros2 topic hz /mavros/vision_pose/pose_cov
timeout 10s ros2 topic echo /mavros/estimator_status --once
timeout 10s ros2 topic echo /mavros/local_position/pose --once
```

Expect one native vision publisher, a MAVROS subscriber, approximately 10 Hz, and valid local position/velocity. Without GPS, false absolute-horizontal-position validity does not by itself invalidate locally aided position control.

For relay warnings:

```bash
timeout 12s ros2 topic echo /rosout rcl_interfaces/msg/Log \
  --qos-durability volatile --qos-reliability best_effort \
  --filter "m.name == 'flight_pose_cov_transport'"
```

In Humble, do not combine this filter with `--field msg`: the filter would receive a string rather than the full Log message. An alignment-file change requires a new grounded session; parameter waits require current cache/readback confirmation.

### Terminal 8: QGroundControl

```bash
~/Downloads/QGroundControl-x86_64.AppImage
```

Under **Application Settings → Comm Links**, disable Pixhawk auto-connect and enable UDP. The USB cable remains onboard and connected to MAVROS; QGC listens on UDP 14550.

In the MAVLink Console:

```text
listener vehicle_visual_odometry 1
listener vehicle_local_position 1
listener manual_control_switches 1
listener actuator_armed 1
commander check
```

Confirm current POSCTL preflight, disarmed state, released Kill, and inactive termination. Historical checks do not replace a current-session check.

## Optional ground planning: terminals 9/10

Keep these processes stopped during the hover test. For separate disarmed SUPER integration checks, prepare the diagnostic copy in terminal 5:

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

A planner waiting for a goal produces no `/planning/pos_cmd`. To run a small ground preview target:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/send_control_preview_goal.py
timeout 10s ros2 topic echo /rog_map/occ \
  sensor_msgs/msg/PointCloud2 --once --field width
timeout 10s ros2 topic hz /planning/pos_cmd
```

The helper is not a takeoff command. With a stationary aircraft, trajectory time can finish without the aircraft reaching the goal, causing replanning. Occupancy processing can relocate the accepted goal. The ground setpoint relay latches off on arming/OFFBOARD and is not the flight controller.

## Terminal 11: checked startup and flight

Keep terminals 1–4 and 6–8 running. Stop SUPER, ground setpoint senders, and any other MAVROS control publisher. Do not modify alignment or restart localization once the flight session is locked.

The aircraft must rest on the actual takeoff floor with the onboard computer/FC secured and no external display or power tether. Use connected RC, arm switch OFF, and POSCTL. The test uses an FC-centre ground clearance of approximately 0.085 m.

Read back the flight limits and configured OFFBOARD-loss action:

```bash
for ros_param_name in MPC_XY_VEL_ALL MPC_Z_VEL_ALL MPC_XY_VEL_MAX MPC_Z_VEL_MAX_UP MPC_Z_VEL_MAX_DN COM_OF_LOSS_T COM_OBL_RC_ACT; do
  printf '%s: ' "$ros_param_name"
  ros2 param get /mavros/param "$ros_param_name"
done
```

Expected values are `-1.0, -1.0, 0.5, 0.5, 0.5, 0.5, 4`. The controller reads these parameters; it does not write them. `configure_first_hover.py` defaults to read/backup and requires `--apply` to update its seven settings.

Run a check-only instance with the intended profile:

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/first_hover_terminal.py --hover-height 1.0 --hold-seconds 6.0
```

Wait for `Phase: READY` and `GROUND LOCK`. Check-only mode publishes debug targets but creates no native setpoint publisher and makes no mode/arming requests. In QGC, with the arm switch still OFF:

```text
listener actuator_armed 1
commander check
```

Confirm `armed: False`, `kill: False`, `termination: False`, and POSCTL `Preflight check: OK`. Then stop the check-only instance with Ctrl+C and start the execution instance:

```bash
python3 ~/Downloads/first_hover_terminal.py --execute --hover-height 1.0 --hold-seconds 6.0
```

The startup log should show `hover FC=1 m, hold=6 s` and `OFFBOARD budget=17 s`. Wait for a new READY/GROUND LOCK. With the flight area clear and manual takeover available, trigger the flight from terminal 5:

```bash
ros2 service call /first_hover/start std_srvs/srv/Trigger '{}'
```

**This service starts automatic OFFBOARD entry, arming, and takeoff.** The expected sequence is:

```text
PRESTREAM → MODE_REQUEST → ARM_REQUEST → RISE → HOLD
          → LAND_REQUEST → LANDING → DONE
```

Starting `--execute` or reaching READY alone does not initiate flight. The start service accepts one task; its success response is not a substitute for subsequent FC mode/arming and flight observations.

Early landing:

```bash
ros2 service call /first_hover/land std_srvs/srv/Trigger '{}'
```

Manual mode takeover, such as switching to POSCTL, stops controller setpoints without overriding the pilot's mode. Kill stops motors directly and is not a controlled-descent command. A repeat test requires a new instance after landing and disarming.

If the service waits indefinitely, inspect the node:

```bash
ros2 node info /first_hover
```

## Controller behavior and limits

| Profile | FC height above floor | Hold | Ground-to-target rise | Ascent reference | OFFBOARD budget |
| --- | --- | --- | --- | --- | --- |
| Default | 0.5 m | 2 s | 0.415 m | Approximately 3.891 s | 8 s |
| Latest flight test | 1.0 m | 6 s | 0.915 m | Approximately 8.578 s | 17 s |

The controller supports startup heights greater than 0.085 m and at most 1 m, with holds from 0 to 6 seconds. Profile values are fixed at initialization. The reference peak vertical speed is 0.2 m/s; PX4's configured speed caps are separate.

Ground initialization requires fresh FC/RC/estimator/landing/local/vision state, parameter confirmation, unique sources, and approximately 3 seconds of stability. After explicit start, the controller streams ground targets for 2 seconds, requests OFFBOARD, confirms it, and then requests arming. **OFFBOARD precedes arming.** Requests are not forced or automatically retried.

XY/yaw remain fixed. The hold begins after local height enters target ±6 cm, then AUTO.LAND is requested. PX4 handles descent and landing detection. DONE requires on-ground and disarmed state; the controller does not force-disarm in flight or start a second task.

Stale inputs, invalid estimates, source/geometry changes, a control-timer interruption over 0.2 seconds, horizontal departure over 0.3 m, or yaw departure over 15° stop the task. If the aircraft is still armed in the controller's OFFBOARD mode, AUTO.LAND is requested. Manual mode takeover has priority. Configured OFFBOARD-loss fallback is `COM_OF_LOSS_T=0.5`, `COM_OBL_RC_ACT=4`; dedicated degraded-input and loss-response tests remain on the validation plan.

Normal automatic landing is part of the completed flight milestone. Emergency/loss response, surveyed venue limits, and planner-driven flight are separate next steps.

## Development validation

`tests/test_first_hover_control.py` uses isolated ROS Humble DDS and MAVROS 2.14 messages/services, with 10 Hz simulated localization and approximately 20 Hz setpoints. It checks both profiles, explicit one-shot start, OFFBOARD-before-arm ordering, rejected requests, native landing and disarm confirmation, stale vision/RC, invalid estimates, departure, duplicate sources, timer interruption, and pilot takeover.

`first_hover_terminal.py` is generated from the canonical hover, reference, preview parameter table, and ground pose-validation sources. Generation and test entry points:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:$PWD/scripts/localization:$PWD/scripts/flight:$PWD/scripts/planning:$PWD/scripts/analysis:$PWD/scripts/maintenance${PYTHONPATH:+:$PYTHONPATH}"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=194
python3 scripts/maintenance/build_first_hover_terminal.py --check
python3 tests/test_hover_reference.py
python3 tests/test_first_hover_control.py
FIRST_HOVER_TEST_MODULE=first_hover_terminal python3 tests/test_first_hover_control.py
```

Run graph tests sequentially, isolated from the aircraft. Concurrent test load previously triggered the timer protection; sequential reruns passed without relaxing the guard. Simulated mission-clock advancement is used in tests and does not represent measured flight duration.

The aircraft has completed the functional takeoff/hover/landing sequence. Further metrics and repeatability are tracked in [next steps](NEXT_STEPS.md).
