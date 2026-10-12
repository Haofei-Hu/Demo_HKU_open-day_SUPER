# Compact aircraft startup

`super_session.py` groups the proven localization and hover processes in a private tmux session. One onboard command terminal is enough; a second terminal can show logs. QGC remains a separate application. The original numbered runbook is still available for diagnosis.

This manager has software/isolated-fixture verification. Its first aircraft startup must confirm current telemetry, alignment, native vision, and hover READY before any flight request. It does not implement the planned SUPER trajectory bridge.

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

Keep RC connected, the arm switch OFF, and the aircraft stationary on its takeoff floor. Start the core:

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
timeout 10s ros2 topic echo /mavros/estimator_status --once
~/Downloads/QGroundControl-x86_64.AppImage
```

Use POSCTL and the current QGC preflight check as described in the [flight runbook](FIRST_HOVER_2026-10-09.md). QGC still uses MAVROS UDP rather than opening USB directly.

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

Stopping requires fresh FC ground state. It interrupts hover/vision first, then the remaining nodes, with MAVROS last; QGC is not closed. If telemetry is unavailable, restore the FC connection and inspect the logs rather than force-stopping an unverified flight session.

## Verification

```bash
python3 tests/test_super_session.py
python3 tests/test_super_session_tmux.py
python3 tests/test_super_session_ros.py
```

Use an isolated test environment. The tmux check requires tmux; the ROS check requires Humble and MAVROS messages/services including ParamPull.

Eight lifecycle unit tests cover invalid inputs, ground-state gates, one-time alignment/vision initialization, stale preview/configuration rejection, explicit hover startup/replacement, shutdown ordering, and argument quoting. A real tmux test uses dummy nodes to check windows, logs, idempotent startup, and cooperative stop without leaked processes. An isolated ROS test checks MAVROS 2.14 ParamPull, typed parameter readback, bounded EV_CTRL writes, and armed/disconnected/stale-state rejection.
