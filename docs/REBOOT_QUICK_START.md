# Startup after power-on or reboot

Use the onboard Ubuntu terminal, locally or through NoMachine. The existing ROS 2 Humble, Livox, FAST_LIO and SUPER workspaces must already be installed. A normal reboot requires no rebuild, script download or dependency installation. Keep the aircraft stationary and disarmed on its actual takeoff floor during initialization.

## Install or update the matching scripts

Run this once when deploying this release, or when deliberately updating the scripts. It is not a startup step after every reboot. Use a grounded, disarmed aircraft and finish any running automatic task before updating.

```bash
(
set -e
demo_revision='57f283d2622e923019b15d04a244de10b7717cb8'
demo_update_dir=$(mktemp -d)
curl -q --noproxy '*' -fL --connect-timeout 10 --max-time 30 \
  "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/$demo_revision/scripts/maintenance/install_demo_test.py" \
  -o "$demo_update_dir/install_demo_test.py"
python3 "$demo_update_dir/install_demo_test.py" --revision "$demo_revision"
)
```

The installer validates the matching release manifest and backs up replaced scripts. It preserves measured alignment and local mission geometry. For the canonical `~/Downloads/open_day_mission.json`, an existing 2 m mission is backed up and migrated to the requested 1.5 m height and matching takeoff stage; vehicle, venue and takeoff measurements are retained. Other mission settings are retained. It does not launch ROS or initiate flight. Separately named measured mission files are not modified: set their `flight.height_fc_above_floor_m` to `1.5` and first sequence entry to `TAKEOFF_TO_1_5M` before selecting them. The integration workspaces and earlier localization helpers remain prerequisites.

## Run the complete ground rehearsal after each reboot

Connect NoMachine if needed, open one terminal on onboard Ubuntu, and run:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry \
  --motion-policy observe
```

No manual `source` commands, quaternion copying, numbered terminals or separate `goal` requests are needed. The entrypoint sources the installed ROS workspaces itself. `--restart` replaces an existing managed session only while grounded and disarmed; after a computer reboot it simply starts a new one. Closing a terminal alone does not stop the background tmux session.

The runner performs the following sequence:

1. Open or reuse QGroundControl, then start MID360, FAST_LIO, MAVROS and body conversion, with online LIO extrinsic estimation disabled for this session.
2. Measure and apply a fresh alignment quaternion, start external vision, and capture the session-relative map configuration.
3. Sample the real vision chain for 30 seconds and capture a fresh occupied scene.
4. Run real SUPER with virtual FC/vehicle feedback in isolated localhost-only ROS domain 197: rise to 1.5 m, stabilize, fly forward 4.5 m, stop, return home, hover six seconds, land, and confirm landed/disarmed feedback.
5. Save the result, stage logs, alignment, scene and mission report; confirm the actual aircraft remains grounded and disarmed.

The actual aircraft receives localization initialization. All flight requests in the rehearsal address the virtual backend. `--fixture-geometry` supplies explicitly synthetic aircraft dimensions and placement; it does not measure laboratory walls or establish physical clearance. A larger laboratory still needs a clear route in the captured map and a valid backup trajectory.

### Automatic QGroundControl startup

Before the actual stack is prepared, the demonstration rehearsal and physical preparation entrypoints open QGC or reuse an existing same-user instance. The default is `~/Downloads/QGroundControl-x86_64.AppImage`, with `QGroundControl.AppImage` and installed executable names as fallbacks. Use `--qgc-path /path/to/QGroundControl.AppImage` for another installed location. The launcher starts the existing executable; it does not download QGC or change its settings.

Retain the existing QGC setup: disable flight-controller USB/serial auto-connect and receive MAVROS UDP forwarding instead. QGC may open before that stream is available; opening the application is not proof of FC connection or preflight readiness. The run report records `qgc` and the launch log is `qgc.log`. Existing instances are reused without creating a second window. QGC is left open after test completion and managed stack shutdown.

`--no-qgc` skips it for the disarmed rehearsal, and `--logic-only` does not open the aircraft ground station. Missing desktop or executable is recorded during remote ground testing. Physical execution requires QGC to have been launched or already running; a launch failure is reported before stack preparation or any flight request. No application-start deadline is added.

### Automatic RViz2 preview

On the onboard desktop, the same command opens RViz2 after the isolated domain 197 check and virtual backend startup, before the controller starts its mission. No second terminal, configuration download or manual launch timing is needed. RViz shows the captured scene and occupied map as points, SUPER's exploration/backup trajectories and goals when published, and an orange virtual-vehicle position arrow. The scene is a frozen snapshot; planning and virtual vehicle feedback update during the rehearsal. The arrow uses raw planner/world coordinates, not the aligned ENU pose as an identity overlay.

The runner owns this viewer process and closes it on completion, failure or interruption. Closing the viewer manually does not stop the test. Other desktop windows are unaffected. Close a viewer launched manually in domain 197 before starting a new rehearsal; the isolation check still rejects pre-existing nodes.

Append `--no-rviz` to disable the viewer. Without a graphical desktop or an installed `rviz2` executable, the runner skips the viewer and records why; the mission still has to pass its normal checks. Each run saves `rehearsal_view.rviz`, `rviz.log` when launched, and `rviz_status.json`; the nested mission report includes `rehearsal_viewer`. A successful process launch is not proof that the remote desktop rendered the window. Inspect `rviz.log` if no window appears.

The planner targets 2 m/s and 1 m/s². Observation mode records exceedances and uses 2.4 m/s and 1.5 m/s² as warning advisories; exceeding either alone does not stop the mission. Position, polynomial, continuity, freshness and landing checks remain active. See [the complete test specification](DEMONSTRATION_MISSION.md).

At each navigation goal, `WAIT_PLAN` holds the leg's starting hover point until the first verified trajectory arrives. This wait has no planning deadline in the complete rehearsal or physical demonstration controller. Console progress shows elapsed waiting time, and reports retain `planning_wait_policy` and `planner_wait_elapsed_s`. The complete controller and rehearsal have no total execution deadline or individual committed-trajectory duration cap. Stage transitions follow arrival, hold completion and landed/disarmed feedback. Reports record these execution limits as null. Planner exit, invalid input, stale telemetry, venue violations and operator stop remain active checks. Planning attempts that fail to produce a valid committed trajectory are not treated as mission success.

At a verified intermediate rest endpoint, the controller holds the committed position with zero velocity while SUPER prepares the next stage. This wait has no deadline, including when stationary endpoint commands continue after their polynomial ends. Callback scheduling gaps are recorded as advisories; the complete mission no longer aborts solely at a 0.2 s callback gap or 0.25 s reference stamp interval. Actual input validity, polynomial continuity and venue bounds remain checked. It does not complete the leg or initiate return early.

SUPER may relocate a goal to a nearby free map cell. The supervisor confirms that accepted goal through the existing planner goal markers, checks it against the venue, and uses it for arrival and leg sequencing. Requested and accepted points and deviations remain in the report. Return is requested to original home; waiting and final hovering hold the actually reached accepted points without an unplanned jump. `nominal_waypoints_reached` separately reports accuracy to the original requested points.

For a separate one-metre rehearsal, append `--route-profile short-1m`. This changes the virtual outbound distance and cannot count as a completed 4.5 m demonstration or enable physical flight. Use the default full route in a suitable laboratory.

## Curved routes inside the venue

After takeoff, the entire configured vehicle envelope must remain inside the outer operating box. SUPER can replan curved routes around obstacles within that box; it is not constrained to a 10 cm tube around the starting heading. Initial-reference, tracking and landing-position offsets are recorded without a separate 10 cm abort rule. The 10 cm arrival tolerance still determines when the requested hover point has been reached; it does not define the permissible path. The former extra 10 cm route inset is removed from geometry validation.

Both planned positions/stage endpoints and measured vehicle positions are checked against the same venue. Freshness, source consistency, finite values, polynomial matching, reference continuity, planner backup constraints, heading policy and operator takeover remain separate requirements. These are data/control checks rather than a fixed straight-line route. QGC and RViz startup do not initiate a physical flight.

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

The physical route is also 1.5 m up, 4.5 m forward, return to the original hover point, six-second hold, then landing. SUPER plans a new return trajectory to the original point; the return is not a recorded replay of the outbound path. The initial heading is retained, so a straight return flies backward without a yaw turn.

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
