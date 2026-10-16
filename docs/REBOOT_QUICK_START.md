# Startup after power-on or reboot

Use the onboard Ubuntu terminal, locally or through NoMachine. The existing ROS 2 Humble, Livox, FAST_LIO and SUPER workspaces must already be installed. A normal reboot requires no rebuild, script download or dependency installation. Keep the aircraft stationary and disarmed on its actual takeoff floor during initialization.

## Install or update the matching scripts

Run this once when deploying this release, or when deliberately updating the scripts. It is not a startup step after every reboot. Use a grounded, disarmed aircraft and finish any running automatic task before updating.

```bash
(
set -e
demo_revision='ab1b1639f8c92e45039d89d3573ae7b5b3ffe180'
demo_update_dir=$(mktemp -d)
curl -q --noproxy '*' -fL --connect-timeout 10 --max-time 30 \
  "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/$demo_revision/scripts/maintenance/install_demo_test.py" \
  -o "$demo_update_dir/install_demo_test.py"
python3 "$demo_update_dir/install_demo_test.py" --revision "$demo_revision"
)
```

The installer validates the matching release manifest and backs up replaced scripts. It preserves the measured alignment and any existing `~/Downloads/open_day_mission.json`, including its local measurements. It does not launch ROS or initiate flight. An existing mission file is retained rather than automatically migrated; inspect its settings before selecting it for a run. The integration workspaces and earlier localization helpers remain prerequisites.

## Run the complete ground rehearsal after each reboot

Connect NoMachine if needed, open one terminal on onboard Ubuntu, and run:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry \
  --motion-policy observe
```

No manual `source` commands, quaternion copying, numbered terminals or separate `goal` requests are needed. The entrypoint sources the installed ROS workspaces itself. `--restart` replaces an existing managed session only while grounded and disarmed; after a computer reboot it simply starts a new one. Closing a terminal alone does not stop the background tmux session.

The runner performs the following sequence:

1. Start MID360, FAST_LIO, MAVROS and body conversion, with online LIO extrinsic estimation disabled for this session.
2. Measure and apply a fresh alignment quaternion, start external vision, and capture the session-relative map configuration.
3. Sample the real vision chain for 30 seconds and capture a fresh occupied scene.
4. Run real SUPER with virtual FC/vehicle feedback in isolated localhost-only ROS domain 197: rise to 2 m, stabilize, fly forward 4.5 m, stop, return home, hover six seconds, land, and confirm landed/disarmed feedback.
5. Save the result, stage logs, alignment, scene and mission report; confirm the actual aircraft remains grounded and disarmed.

The actual aircraft receives localization initialization. All flight requests in the rehearsal address the virtual backend. `--fixture-geometry` supplies explicitly synthetic aircraft dimensions and placement; it does not measure laboratory walls or establish physical clearance. A larger laboratory still needs a clear route in the captured map and a valid backup trajectory.

### Automatic RViz2 preview

On the onboard desktop, the same command opens RViz2 after the isolated domain 197 check and virtual backend startup, before the controller starts its mission. No second terminal, configuration download or manual launch timing is needed. RViz shows the captured scene and occupied map as points, SUPER's exploration/backup trajectories and goals when published, and an orange virtual-vehicle position arrow. The scene is a frozen snapshot; planning and virtual vehicle feedback update during the rehearsal. The arrow uses raw planner/world coordinates, not the aligned ENU pose as an identity overlay.

The runner owns this viewer process and closes it on completion, failure or interruption. Closing the viewer manually does not stop the test. Other desktop windows are unaffected. Close a viewer launched manually in domain 197 before starting a new rehearsal; the isolation check still rejects pre-existing nodes.

Append `--no-rviz` to disable the viewer. Without a graphical desktop or an installed `rviz2` executable, the runner skips the viewer and records why; the mission still has to pass its normal checks. Each run saves `rehearsal_view.rviz`, `rviz.log` when launched, and `rviz_status.json`; the nested mission report includes `rehearsal_viewer`. A successful process launch is not proof that the remote desktop rendered the window. Inspect `rviz.log` if no window appears.

The planner targets 2 m/s and 1 m/s². Observation mode records exceedances and uses 2.4 m/s and 1.5 m/s² as warning advisories; exceeding either alone does not stop the mission. Position, polynomial, continuity, freshness and landing checks remain active. See [the complete test specification](DEMONSTRATION_MISSION.md).

For a separate one-metre rehearsal, append `--route-profile short-1m`. This changes the virtual outbound distance and cannot count as a completed 4.5 m demonstration or enable physical flight. Use the default full route in a suitable laboratory.

## Read the result and retain the report

Wait for the foreground runner to finish. It prints `RESULT: SUCCESS` or `RESULT: FAIL` and the exact `REPORT:` path. Success requires the entire rehearsal, including the return, six-second hold and confirmed landing. It does not establish physical flight completion. Do not replace a failed result with a successful preparation stage.

To inspect the latest saved rehearsal result from a new terminal:

```bash
python3 - <<'PY'
from pathlib import Path
import json

reports = sorted((Path.home() / 'Downloads/super_demo_tests').glob('*/report.json'))
if not reports:
    raise SystemExit('No demonstration rehearsal report found.')
path = reports[-1]
report = json.loads(path.read_text())
print('RESULT:', report.get('result', 'UNKNOWN'))
print('STAGE:', report.get('stage', 'UNKNOWN'))
print('ERROR:', report.get('error') or 'None')
print('REPORT:', path)
PY
```

`RUNNING` is unfinished, including when a process was interrupted before it could finalize its report. On failure, retain the printed report and adjacent logs. The runner does not automatically retry, rebase or turn a partial mission into success. The latest cloud stress trial stopped on an approximately 0.5-second setpoint timer gap; its cause and recurrence on the aircraft computer remain unverified. See [validation evidence](../data/software_checks_2026-10-16/flight_motion_policy.json).

## Inspect or stop the background stack

After the foreground test has finished:

```bash
python3 ~/Downloads/super_session.py status
```

Status prints the current session log directory. An active process alone does not establish fresh telemetry. To stop the managed stack after landing and disarming:

```bash
python3 ~/Downloads/super_session.py stop
```

Run the complete rehearsal command again for another fresh automatic test. Do not reuse a quaternion from a previous boot or LIO initialization. For individual node diagnosis, use [compact session startup](SESSION_STARTUP.md); the historical [numbered hover workflow](FIRST_HOVER_2026-10-09.md) is separate from this demonstration.

## Prepare physical execution separately

The physical route is also 2 m up, 4.5 m forward, return to the original hover point, six-second hold, then landing. SUPER plans a new return trajectory to the original point; the return is not a recorded replay of the outbound path. The initial heading is retained, so a straight return flies backward without a yaw turn.

Before physical preparation, fill a separate measured mission file using the schema in `~/Downloads/open_day_mission.json`. The canonical operating box remains 5.8 m along the heading, 2.8 m wide and 2.8 m high, even in a larger laboratory. Survey that box, takeoff distances from its rear and left boundaries, and all six propeller-inclusive FC-centre extents. Replace every null geometry field with an actual measurement; do not substitute synthetic fixture values. The current floor calibration assumes an FC centre 0.085 m above the floor; a different installation requires a separate calibration change.

The physical entrypoint also requires current RC input/manual POSCTL, one controller, compatible PX4 parameter readbacks and on-site takeover. In particular, it expects `MPC_XY_VEL_MAX=2.0`; the last recorded aircraft setting was 0.5. It checks this setting and does not silently change it. Actual receipt of the new raw position-plus-velocity setpoints and complete physical tracking remain to be verified.

With the measured configuration saved as `~/Downloads/open_day_mission_measured.json`, prepare without flight execution:

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_measured.json" \
  --motion-policy observe
```

This starts a new stack, captures a fresh quaternion and prepares the real planner/controller. `PREPARATION: READY; MISSION: NOT RUN (check-only)` means preparation passed; the check-only controller then exits. Reports are saved under `~/Downloads/super_demo_reports/<timestamp>/`. This entrypoint requires actual mission geometry even in check-only mode.

For a supervised physical trial after prerequisites and clearance have been confirmed, run the separate execution entry in terminal A:

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_measured.json" \
  --motion-policy observe --execute --on-site-supervised
```

Keep terminal A running. It prepares another fresh session and waits for `Phase: READY` and an explicit start. With the operator at the aircraft and RC takeover available, terminal B sends the start request once:

```bash
source /opt/ros/humble/setup.zsh
ros2 service call /super_demo/start std_srvs/srv/Trigger '{}'
```

The start service is flight initiation. Follow the controller's acceptance or rejection; a rejected start does not authorize another mode or arming command. Rebooting the computer, opening a terminal, installing scripts or running the rehearsal cannot initiate this physical mission.
