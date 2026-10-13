# Compact aircraft startup

`super_session.py` groups the localization, ground planning, and hover processes in a private tmux session. One onboard command terminal is enough; a second terminal can show logs. QGC remains a separate application. The original numbered runbook is still available for diagnosis.

This manager has software/isolated-fixture verification. Aircraft localization startup has been checked with current telemetry, fresh alignment, native vision, and valid local position/velocity. Ground planning and hover readiness still require their own checks before use. It does not implement the planned SUPER trajectory bridge.

## Install once

Install tmux on onboard Ubuntu:

```bash
sudo apt update
sudo apt install tmux
```

From a checkout, `python3 scripts/maintenance/install_scripts.py` installs the manager without replacing existing files. To download only the manager:

```bash
session_download_dir=$(mktemp -d "$HOME/Downloads/session_download_XXXXXX")
if curl -q --noproxy '*' --connect-timeout 10 --max-time 30 -fL \
  'https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/main/scripts/maintenance/super_session.py' \
  -o "$session_download_dir/super_session.py" \
  && python3 -m py_compile "$session_download_dir/super_session.py"
then
  if [ -f "$HOME/Downloads/super_session.py" ]; then
    cp "$HOME/Downloads/super_session.py" "$session_download_dir/previous.py"
  fi
  cp "$session_download_dir/super_session.py" "$HOME/Downloads/super_session.py"
fi
```

Existing integration scripts/workspaces remain prerequisites. The manager sources their Bash setup files itself, even from a zsh command terminal. A custom serial endpoint can be supplied with `start --fcu-url`; the recorded default is the serial-by-id path in the numbered runbook.

## Replace manual terminals

On the ground and disarmed, stop manually running stack nodes with Ctrl+C before the first managed start. The manager rejects detected existing stack processes rather than duplicating or terminating them. Do not migrate a running flight into tmux.

Keep the aircraft disarmed and stationary on its takeoff floor. Flight preparation also requires RC connected with the arm switch OFF; disarmed ground planning can be checked remotely. Start the core:

```bash
python3 ~/Downloads/super_session.py start
```

This creates tmux windows for MID360, FAST_LIO, MAVROS, and FC-centre diagnostics. No vision or control process is started yet. Calling `start` again shows status rather than restarting nodes.

Run fresh-session checks:

```bash
python3 ~/Downloads/super_session.py align
```

This requires fresh connected/disarmed/on-ground state, pulls MAVROS parameters, sets and reads back EV_CTRL=0, then runs odometry-age and 15-second attitude checks. A quaternion is printed, not automatically saved. Inspect the sample count, ages, pairing gap, and rotation spread; remain stationary. If the survey is stale, inconsistent, or has insufficient matches, diagnose it before enabling vision.

Use this run's four xyzw values, replacing the placeholders:

```text
python3 ~/Downloads/super_session.py vision QX QY QZ QW
```

This requires EV_CTRL=0, HGT_REF=3, zero EV_POS, and QMIN=0. It backs up/updates the preview quaternion, starts the preview, waits for a fresh sample, sets/readbacks EV_CTRL=11, and starts the native continuous vision relay. It refuses to modify an already initialized vision/control session.

Check actual fusion and open QGC:

```bash
source /opt/ros/humble/setup.zsh
timeout 10s ros2 topic hz /mavros/vision_pose/pose_cov
timeout 10s ros2 topic echo /mavros/estimator_status mavros_msgs/msg/EstimatorStatus \
  --qos-reliability best_effort --qos-durability volatile --once
~/Downloads/QGroundControl-x86_64.AppImage
```

Use POSCTL and the current QGC preflight check as described in the [flight runbook](FIRST_HOVER_2026-10-09.md). QGC still uses MAVROS UDP rather than opening USB directly.

## Ground planning in the same session

After localization is valid, start SUPER and its coordinate preview:

```bash
python3 ~/Downloads/super_session.py planner
python3 ~/Downloads/super_session.py status
```

This requires the managed core, preview, and vision windows, fresh connected/disarmed/on-ground FC state, and valid local position/velocity telemetry. A disarmed AUTO.LOITER state is sufficient for this ground test; RC input and POSCTL are requirements of the flight workflow. A hover window or an existing native setpoint sender blocks planning startup.

The manager starts two windows: `planner` and `cmd_preview`. It creates the existing ground diagnostic YAML only if absent, validates its input/goal/command topics, and preserves an existing profile. This profile changes optimizer convergence tolerance and logging only; its motion limits are not a flight configuration. Required helpers are `create_super_ground_diagnostic.py`, `send_control_preview_goal.py`, and `control_bridge_preview.py` in the active scripts directory. They can be installed from a checkout using the deployment helper.

Inspect the occupancy map before sending a goal:

```bash
source /opt/ros/humble/setup.zsh
timeout 10s ros2 topic echo /rog_map/occ sensor_msgs/msg/PointCloud2 \
  --qos-reliability best_effort --once --field width
python3 ~/Downloads/super_session.py logs
```

Select `planner` and `cmd_preview` with Ctrl+B, then W. Before a goal, the planner may remain in INIT, while the coordinate preview reports `READY coordinates; WAIT: fresh /planning/pos_cmd`. Confirm a nonempty occupancy map and fresh inputs. Detach with Ctrl+B, then D.

Send one small diagnostic goal and inspect its output:

```bash
python3 ~/Downloads/super_session.py goal
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
timeout 10s ros2 topic hz /planning/pos_cmd
timeout 10s ros2 topic echo /super_debug/control_target geometry_msgs/msg/PoseStamped \
  --qos-reliability best_effort --once
```

The command terminal must source the SUPER overlay to resolve `mars_quadrotor_msgs/msg/PositionCommand`; the manager sources its child processes independently. A message-type error in the CLI does not by itself establish planner failure.

The goal is +0.30 m along planner-world X, with heading retained. This axis is not necessarily aircraft forward. Goal height follows SUPER's `click_height`; the planner may relocate the goal to free space. Inspect `Get goal at` and the transformed preview before drawing conclusions about direction or displacement. The preview publishes only `/super_debug/control_target`. These actions send no MAVROS flight setpoints, mode requests, arming requests, or flight-start requests. Replanning is expected while the stationary aircraft does not follow the target.

Stop just these two windows before returning to hover preparation:

```bash
python3 ~/Downloads/super_session.py planner-stop
```

The existing localization and vision session remains running. Updating the manager or missing planning helpers does not require a new alignment; preserve the active `aligned_pose_preview.py` and do not repeat `start`, `align`, or `vision` in an initialized session.

### Low-speed preview and demonstration candidate

Generate separate motion candidates from the actual diagnostic configuration:

```bash
python3 ~/Downloads/create_super_motion_profiles.py
```

The helper preserves the diagnostic source and its comments, backs up the source, and writes separate configurations for baseline low-speed preview, reserved-acceleration preview, reserved velocity/acceleration preview, and the final demonstration candidate. An existing different candidate is refused before any candidate is written. Repeating generation with an unchanged source is idempotent.

| Candidate | Planner velocity limit | Planner acceleration limit | Use |
| --- | --- | --- | --- |
| `mid360_low_speed_preview_ros2.yaml` | 0.3 m/s | 0.5 m/s² | Original low-speed ground test |
| `mid360_low_speed_margin_ros2.yaml` | 0.3 m/s | 0.45 m/s² | Reserved acceleration target; strict sampled acceptance stays 0.3 / 0.5 |
| `mid360_low_speed_reserved_ros2.yaml` | 0.25 m/s | 0.45 m/s² | Reserved velocity and acceleration targets; strict sampled acceptance stays 0.3 / 0.5 |
| `mid360_demo_candidate_ros2.yaml` | 2.0 m/s | 1.0 m/s² | Final motion target, pending flight integration |

Only `traj_opt.boundary.max_vel` and `max_acc` change. Height, yaw, jerk, nominal collision radius, map settings, and topic names are preserved. Planner optimizer limits do not establish measured tracking limits or a hard flight boundary. In particular, `click_height=-0.05`, accepted planner Z, and the virtual ground/ceiling are not floor-referenced laboratory limits.

Switch only the ground planning windows:

```bash
python3 ~/Downloads/super_session.py planner-stop
python3 ~/Downloads/super_session.py planner --planner-profile low-speed
```

Localization and vision remain running. After map/preview readiness, use the same `goal` diagnostic and inspect the accepted target and optimizer output. The manager validates the low-speed limits before launch. It does not offer the 2 m/s candidate as a startup profile and does not change PX4 parameters. The original baseline and demonstration files are preserved when the margin candidate is added.

For ideal constant-acceleration straight-line motion, 2 m/s at 1 m/s² requires approximately 2 seconds and 2 metres to accelerate, with another 2 metres to brake. Real trajectories, clearance, latency, and tracking error require additional margin. Flight execution still needs the fixed takeoff-floor/forward reference, transformed goals, envelope enforcement, endpoint hold, and takeover/loss handling.

### Sample planner motion and freshness

With the grounded low-speed planner actively publishing after a diagnostic goal:

```bash
python3 ~/Downloads/super_session.py planner-check
```

This sources the session environment and runs the read-only `control_chain_age_check.py` for 10 seconds. It checks the managed planning/localization prerequisites, then reports source age, receive gaps, nonincreasing/invalid stamps, and unexpected frames for raw odometry, planner commands, transformed debug targets, and FC local pose. Planner vectors are checked using full three-dimensional norms against 0.3 m/s and 0.5 m/s². Frames and active regular/backup flags must match the pinned integration; nonfinite position/derivative values are rejected. The freshness check requires at least 20 samples per stream, ages/receive gaps/latest receipt within 0.25 seconds, and no stamps more than 0.05 seconds in the future. Publisher endpoint identities are polled from the ROS graph every 0.2 seconds; exactly one cumulative identity must be discovered per sampled topic. The step report compares cached graph identities, not per-message DDS metadata. It also requires fresh connected/disarmed FC state and no arming observed during sampling. Floating-point comparison tolerance is 1e-9.

Without the manager, the same check can be run directly:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/control_chain_age_check.py --check-motion \
  --max-speed 0.3 --max-acceleration 0.5
```

The advertised derivatives are planner-reference values, not measured aircraft speed/acceleration. `Acc` in the optimizer's printed cost report is a penalty value, not the reference acceleration maximum. The report also records the largest adjacent planner position step and its stamp interval. A stationary aircraft can trigger trajectory restarts/replanning; these steps need separate review before endpoint-hold/flight behavior is implemented.

Exit status 2 means the sampled freshness, requested motion limits, or disarmed-state checks did not pass. The report is still printed; the manager may then show the helper's nonzero exit status. Inspect the values before changing a limit. This command publishes no planner goal, native setpoint, mode request, or arming request, and it changes no PX4 parameter. Passing sampled checks does not establish tracking accuracy, mechanical calibration, endpoint hold, envelope enforcement, or flight readiness.

### Reserve optimizer acceleration margin

The first low-speed aircraft sample reported 0.267870 m/s advertised speed and 0.503837 m/s² advertised acceleration. Freshness and disarmed-state checks passed; strict motion acceptance did not. The pinned optimizer uses penalty costs and a penalty-based success check, so SUCCESS is not a hard derivative bound.

Generate the separate candidate and switch only ground planning:

```bash
python3 ~/Downloads/create_super_motion_profiles.py
python3 ~/Downloads/super_session.py planner-stop
python3 ~/Downloads/super_session.py planner --planner-profile low-speed-margin
```

After a nonempty map is received, send the diagnostic `goal` and rerun `planner-check`. The internal planner acceleration target becomes 0.45 m/s²; the checker remains at 0.5 m/s², with no extra tolerance. This headroom is a candidate correction requiring another physical sample, not a guarantee of a continuous bound. The final 2 m/s / 1 m/s² demonstration candidate remains unchanged.

The first sample also recorded a 0.223607 m reference step across 0.030030 seconds. The pinned FSM restarts planning from the current robot state after trajectory time ends while the stationary aircraft remains away from its goal. The report now includes before/after reference positions, nearby raw odometry, and cached single-publisher identity comparison to help correlate the step; it does not automatically classify it as a restart or repair the reference. A flight bridge must handle discontinuities and endpoint hold before native execution.

### Reserve velocity margin after the 50 Hz trial

The MID360 driver publication setting was changed at startup from 10 Hz to 50 Hz. Native MAVROS vision publication measured approximately 50 Hz, with valid relative horizontal position and horizontal/vertical velocity telemetry. A subsequent 10-second ground control-chain sample passed freshness checks: source age maxima were 24 ms for odometry, 8 ms for planner commands, 14 ms for converted targets, and 17 ms for FC local pose. The earlier approximately 0.34-second gap was not observed in this sample; this does not establish that it cannot recur or isolate its cause.

Advertised speed reached 0.317892 m/s despite the 0.3 m/s optimizer target, while acceleration reached 0.373977 m/s² and stayed below the 0.5 m/s² acceptance limit. Use the separate reserved candidate for the next sample:

```bash
python3 ~/Downloads/super_session.py planner-stop
python3 ~/Downloads/super_session.py planner --planner-profile low-speed-reserved
```

The manager generates the candidate if missing using the updated `create_super_motion_profiles.py`. Internal limits are 0.25 m/s and 0.45 m/s²; sampled acceptance remains strictly 0.3 m/s and 0.5 m/s². Previous candidates, localization, active alignment and PX4 parameters are preserved. After a nonempty occupancy map is available, run `goal` and `planner-check`. A subsequent 10-second physical ground sample passed strict advertised-derivative, freshness and disarmed-state acceptance, with speed/acceleration maxima of 0.289609 m/s and 0.310397 m/s². Continuous bounds and executed tracking remain unvalidated.

The same sample recorded a 0.424264 m reference step from `[0.35, -0.05, 0.15]` to `[0.05, -0.05, -0.15]` over approximately 20 ms, with a stable cached single-publisher identity. The new reference was near the stationary raw robot position, consistent with the pinned FSM restarting from the robot state after the previous trajectory ended. The advertised derivatives do not describe the finite difference across this restart. A passing derivative check therefore cannot approve direct flight execution; reference continuity and endpoint hold remain separate bridge requirements.

The successful reserved-profile sample still contained a 0.424264 m step over 20.310 ms, from `[0.25, -0.05, 0.15]` to `[-0.05, -0.05, -0.15]`. Nearby raw odometry was `[-0.007629, -0.0947, -0.14885]`. Source age maxima were 32/9/17/16 ms for odometry/planner/debug target/FC local pose, respectively, and all receive gap maxima were at most 46 ms. This closes the sampled freshness and derivative-limit check; reference continuity and endpoint hold remain the next integration work.

### Inspect bounded reference and endpoint hold

The separate [bounded ground reference preview](BOUNDED_REFERENCE_PREVIEW.md) locks stationary takeoff yaw/FC-centre position, applies forward/lateral/floor-height point bounds, matches position commands to committed polynomial trajectories, and holds the first verified endpoint while suppressing stationary restarts. It publishes only additional debug topics. Start `reference-preview` after a fresh reserved-profile planner and before sending one diagnostic goal; wait for `WAIT_TRAJECTORY` before the goal. The existing raw-coordinate preview is preserved for comparison. A fault reports its reason and stops output rather than rebasing, shifting or clipping the target. Same-height and 1 m vertical physical ground trials reached `END_HOLD` and suppressed stationary restart samples; native flight support is not implemented. For a vertical reference, use `goal --goal-height 1.0` with the low-speed-3d profile on the actual takeoff floor; see the [vertical workflow](BOUNDED_REFERENCE_PREVIEW.md#vertical-1-m-target-in-a-ground-preview).

## Live RViz2 view

The pinned SUPER planner's `rviz/real_svc.rviz` uses ROS 1 plugin identifiers. The simulator's `rviz2/benchmark.rviz` is a ROS 2 configuration, but its camera targets `perfect_drone` and includes simulation topics. The repository provides [super_ground_ros2.rviz](../configs/rviz/super_ground_ros2.rviz), derived from that ROS 2 configuration with the real planning topics, a `world` view, visible docks, and no goal-publishing tool.

Download it into a new directory in the onboard Ubuntu desktop terminal:

```bash
source /opt/ros/humble/setup.zsh
source ~/fastlio_ws/install/setup.zsh
source ~/super_ws/install/setup.zsh
rviz_view_dir=$(mktemp -d "$HOME/Downloads/rviz_view_XXXXXX")
curl -q --noproxy '*' --connect-timeout 10 --max-time 30 -fL \
  'https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/main/configs/rviz/super_ground_ros2.rviz' \
  -o "$rviz_view_dir/super_ground_ros2.rviz"
```

In this pinned integration, SUPER's `world` numerically aliases FAST_LIO's `camera_init`. If no existing TF already connects those frames, start the visualization-only identity alias once:

```bash
nohup ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id world --child-frame-id camera_init \
  --ros-args -r __node:=super_rviz_world_alias \
  > ~/Downloads/rviz_world_tf.log 2>&1 &
```

Open the viewer after a successful download:

```bash
nohup rviz2 -d "$rviz_view_dir/super_ground_ros2.rviz" \
  > ~/Downloads/rviz_super.log 2>&1 &
```

It shows registered clouds (`/cloud_registered`), occupied map (`/rog_map/occ`), regular/backup trajectory markers (`/visualization/exp_traj`, `/visualization/backup_traj`), planner path (`/fsm/path`), and goal markers (`/visualization/goal`) when published. All topic subscriptions use Best Effort/Volatile settings. Before a goal, an empty trajectory display is expected. A backup trajectory is only visible when one is published.

The alias connects the raw planning/world display frames only. It does not align MAVROS `map` or `px4_enu_preview`, and those frames must not be given the same identity transform. Neither the viewer nor the alias sends planner goals, FC setpoints, mode requests, or arming requests. Keep the current localization session running. The optional viewer/TF process is separate from managed core shutdown; close the RViz window when finished. Inspect `~/Downloads/rviz_super.log` if the window does not open.

The configuration and upstream topic/plugin definitions were checked offline. Actual onboard rendering remains a desktop validation step.

## Hover and explicit flight initiation

Optional check-only hover process:

```bash
python3 ~/Downloads/super_session.py hover
```

After current preflight checks, enable the existing 1 m / 6-second controller:

```bash
python3 ~/Downloads/super_session.py hover --execute
```

This can replace a managed check-only hover instance while grounded/disarmed. It creates no flight start request. Inspect the hover window for READY and GROUND LOCK before using the separate flight service:

```bash
source /opt/ros/humble/setup.zsh
ros2 service call /first_hover/start std_srvs/srv/Trigger '{}'
```

**The service above initiates automatic OFFBOARD entry, arming, and flight.** Early landing retains the existing interface:

```bash
ros2 service call /first_hover/land std_srvs/srv/Trigger '{}'
```

After landing, disarming, and returning to POSCTL, replace only the hover instance for another test:

```bash
python3 ~/Downloads/super_session.py hover --execute --restart
```

The localization session remains running. Fresh ground-state checks block hover replacement while armed or airborne. The new controller again waits for READY and explicit start.

## Logs, detach, and stop

```bash
python3 ~/Downloads/super_session.py status
python3 ~/Downloads/super_session.py logs
```

In tmux, use **Ctrl+B, then N/P** to change windows, **Ctrl+B, then W** to select one, and **Ctrl+B, then D** to detach. Ctrl+C interrupts the process in the selected pane; do not use it merely to leave the log view. Closing the attached terminal or losing NoMachine does not intentionally stop the nodes.

Logs are stored in `~/Downloads/super_session_logs/<run>/`, one per process. `status` reports process activity, not ROS readiness. Failed nodes remain visible and are never automatically restarted. Existing ROS timing/source/parameter gates remain in the flight and vision scripts.

To stop the managed stack after landing/disarming:

```bash
python3 ~/Downloads/super_session.py stop
```

Stopping requires fresh FC ground state. It interrupts hover/planning/vision first, then the remaining nodes, with MAVROS last; QGC is not closed. If telemetry is unavailable, restore the FC connection and inspect the logs rather than force-stopping an unverified flight session.

## Verification

```bash
python3 tests/test_super_session.py
python3 tests/test_super_motion_profiles.py
python3 tests/test_control_chain_metrics.py
python3 tests/test_control_chain_metrics_ros.py
python3 tests/test_super_session_tmux.py
python3 tests/test_super_session_ros.py
```

Use an isolated test environment. The tmux check requires tmux; the ROS check requires Humble and MAVROS messages/services including ParamPull.

Nineteen lifecycle unit tests cover invalid inputs, ground-state gates, one-time alignment/vision initialization, stale preview/configuration rejection, explicit hover startup/replacement, planning/hover exclusion, local planning shutdown, diagnostic/low-speed profile preservation and validation, bounded-preview launch/hover exclusion/local shutdown, shutdown ordering, and argument quoting. A real tmux test uses dummy nodes to check windows, logs, idempotent startup, local planning stop, and cooperative full stop without leaked processes. An isolated ROS test checks MAVROS 2.14 ParamPull, typed parameter readback, bounded EV_CTRL writes, fresh valid ground localization in AUTO.LOITER without RC mode flags, and armed/disconnected/stale-state rejection.

Three motion-profile tests cover preservation of all non-motion configuration, source/comments, idempotent generation, conflicting-candidate refusal before writes, and invalid/duplicate limit rejection. Low-speed planner convergence and aircraft behavior remain physical validation steps.

Four offline metric tests verify combined-axis limits, nonfinite/inactive/wrong-frame rejection, timestamp order and reference-step reporting, and missing/stale/future/gapped data. An isolated DDS fixture exercises live sampling with excessive speed/acceleration, armed state, stale data, inactive flags, and wrong pose frames and multiple discovered command publishers.

Nine bounded-reference unit tests and an isolated DDS fixture verify polynomial evaluation, point bounds, completion evidence, endpoint hold across restarts, moving jumps, malformed trajectories, stale feedback and alignment/source/arming latches. The live coordinate-conversion fixture and real tmux lifecycle test also pass with the optional reference-preview window. See the bounded-reference runbook for scope and physical validation requirements.

The first bounded-preview aircraft trial locked the frame but rejected a reference 5.52 cm behind it. The current preview includes a 10 cm backward allowance and keeps the original SUPER rest-start behavior. See the [rollback guide](EXACT_PLANNER_START.md) if the optional exact-start patch was applied. Rebuild only when restoring a previously patched executable. Restart ground planning/reference preview and resample while retaining the 50 Hz localization and vision session.

The current ground preview checks initial reference offsets against fresh measured position separately on forward, left and up axes, with an absolute 0.10 m limit per axis. Its debug floor envelope is −0.10–2 m. This allows small grid offsets during a disarmed test without changing coordinates. The status prints the accepted initial offsets. Actual flight-height selection, continuous tracking error and clearance remain separate checks for native flight integration.

### Same-height 3D ground goals

Use the current `low-speed-3d` profile for bounded-reference testing:

```bash
python3 ~/Downloads/super_session.py planner-stop
python3 ~/Downloads/super_session.py planner --planner-profile low-speed-3d
python3 ~/Downloads/super_session.py reference-preview
```

The manager creates `mid360_low_speed_3d_goal_ros2.yaml` separately from the reserved profile. Only `fsm.click_height` changes to −6, disabling the pinned planner's fixed-height override; this is a sentinel, not a six-metre goal. Motion limits, map settings and original profiles remain unchanged. The goal helper sends the current raw Z and prints requested XYZ. Wait for `WAIT_TRAJECTORY` before one goal, then compare the accepted `Get goal at` position and bounded status. Map goal/start relocation can still displace the result, so the existing per-axis offset and continuity gates remain active.

A recorded aircraft sample had raw odometry Z 1.494 m while the accepted goal Z was 0.150 m under `click_height=-0.05`. Those values are planner coordinates, not floor height. A subsequent same-height test accepted goal Z 1.450 m and reached `END_HOLD`; its initial offsets were within 10 cm per axis. Map snapping remains active, so each new start and goal still needs inspection. Fault status now includes the first fault's phase and accepted command count.

### Battery changes and fresh-session recovery

When a battery change powers off the onboard computer, treat the next boot as a new localization session:

```bash
python3 ~/Downloads/super_session.py start
python3 ~/Downloads/super_session.py align
# Use the freshly measured quaternion, not a prior battery/session value:
python3 ~/Downloads/super_session.py vision QX QY QZ QW
```

Keep the aircraft stationary on its takeoff floor through initialization and pairing. If only the FC/sensors reboot while the computer remains running, use grounded `stop`, `start`, `align` before fresh vision initialization. Stop/start clears managed planning/reference state; configuration files persist. Recheck input age, estimator validity and external/FC pose agreement. Each flight still requires explicit initiation and current RC/flight readiness. Cold-start recovery does not establish that localization will remain stable for the complete demonstration.

A clean-session sample on October 13 restored full pose rotation agreement from approximately 35° to 1.75° and odometry maximum age to 27 ms. The underlying cause and complete-run stability remain under investigation.
