# Startup after power-on or reboot

**[Quick start: complete installation, reboot startup, results, and recovery](QUICK_START.md)**

Use the onboard Ubuntu terminal, locally or through NoMachine. The existing ROS 2 Humble, Livox, FAST_LIO and SUPER workspaces must already be installed. A normal reboot requires no rebuild, script download or dependency installation. Keep the aircraft stationary and disarmed on its actual takeoff floor during initialization.

## Install or update the matching scripts

Run this once when deploying this release, or when deliberately updating the scripts. It is not a startup step after every reboot. Use a grounded, disarmed aircraft and finish any running automatic task before updating.

```bash
(
set -e
demo_revision='4624a0f50469fcd1351de9aaff4e9c41a7ff1638'
demo_update_dir=$(mktemp -d)
curl -q --noproxy '*' -fL --connect-timeout 10 --max-time 30 \
  "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/$demo_revision/scripts/maintenance/install_demo_test.py" \
  -o "$demo_update_dir/install_demo_test.py"
python3 -m py_compile "$demo_update_dir/install_demo_test.py"
python3 "$demo_update_dir/install_demo_test.py" --revision "$demo_revision"
)
```

Always install the complete matching 28-file release. Updating only a controller or reference helper can mix incompatible interfaces: an observed two-file update called a four-argument `inverse_goal()` against an older three-argument implementation. After installation, the normal `--restart` command below clears the previous grounded session before using the new release.

The installer validates the matching release manifest and backs up replaced scripts. It preserves measured alignment and local mission geometry. For the canonical `~/Downloads/open_day_mission.json`, an existing 2 m mission is backed up and migrated to the requested 1.5 m height and matching takeoff stage; vehicle, venue and takeoff measurements are retained. Known canonical single-round profiles are also backed up and expanded to five rounds, preserving vehicle, venue, takeoff and alignment measurements. Explicit custom profiles are retained. Other mission settings are retained. It does not launch ROS or initiate flight. Separately named measured mission files are not modified. Keep `flight.round_trips=5`; `--flight-height 1.5` validates the survey at the selected height and saves the matching takeoff stage in a per-run copy. The integration workspaces and earlier localization helpers remain prerequisites.

This release separates the unchanged **2 m/s, 1 m/s² optimization targets** from **4 m/s, 2 m/s² trajectory acceptance bounds**, and passes the existing rehearsal venue to SUPER's search and corridor generation. It requires two independent, one-time SUPER source patches and one rebuild after installation:

```bash
(
set -e
python3 ~/Downloads/apply_super_motion_acceptance.py
python3 ~/Downloads/apply_super_venue_bounds.py
bash --noprofile --norc -c '
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/super_ws/install/setup.bash"
cd "$HOME/super_ws"
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --packages-select rog_map super_planner --executor sequential
'
)
```

The current patch preserves the measured continuous initial position in venue-enabled `PlanFromRest` (`VENUE_REST_START_V1`), eliminating grid-centre relocation at a stationary restart. It rejects occupied, invalid, or out-of-map/venue starts and retains downstream obstacle and trajectory checks. It also intersects all planner checks with the initialized map-height interval (`VENUE_MAP_HEIGHT_V1`). Apply the patch and rebuild once even when the previous eight venue markers are already installed.

The patch also joins valid continuous endpoints to admissible neighbouring search cells (`VENUE_GRID_ANCHOR_V1`). This prevents a near-boundary rest point from being rejected solely because its nearest grid centre lies outside the rotated venue. The short connector must pass the existing map/unknown checks. A failed search initialization retains an unfinished in-venue goal instead of silently returning to `WAIT_GOAL`. No boundary tolerance, inset or mission deadline is added. The patch retains the duplicate virtual floor/ceiling inflation fixes (`VIRTUAL_HEIGHT_QUERY_V1` / `VIRTUAL_HEIGHT_CORRIDOR_V1`), candidate polynomial checks (`VENUE_CANDIDATE_CHECK_V1`) and from-rest commit fix. Rebuild both packages when upgrading an older release; map code is statically linked into the planner.

Each patcher saves a reversible backup. Motion targets and optimization penalties are preserved; the venue patch applies the existing box during planning. The manager/runner verify the installed binary. If motion acceptance has already been compiled, apply the additional venue patch and rebuild once; its rollback preserves motion acceptance. See [motion acceptance](MOTION_ACCEPTANCE.md) and [planner venue bounds](PLANNER_VENUE_BOUNDS.md). Subsequent power-on/reboot starts use the normal command below without rebuilding.

## Run the complete ground rehearsal after each reboot

An optional [independent left-facing 4.8 m rehearsal](LEFT_48M_REHEARSAL.md) reuses a saved successful map in a separate installation and ROS domain. It leaves this forward-demonstration startup workflow unchanged.

Connect NoMachine if needed, open one terminal on onboard Ubuntu, and run:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry --fast-start \
  --motion-policy observe
```

No manual `source` commands, quaternion copying, numbered terminals or separate `goal` requests are needed. The entrypoint sources the installed ROS workspaces itself. `--restart` replaces an existing managed session only while grounded and disarmed; after a computer reboot it simply starts a new one. Closing a terminal alone does not stop the background tmux session.

The runner performs the following sequence:

1. Open or reuse QGroundControl, then start MID360, FAST_LIO, MAVROS and body conversion, with online LIO extrinsic estimation disabled for this session.
2. Measure and apply a fresh alignment quaternion, start external vision, and capture the session-relative map configuration.
3. Wait for a continuous fresh 30-second real vision window, then capture a fresh occupied scene.
4. Run real SUPER with virtual FC/vehicle feedback in isolated localhost-only ROS domain 197: rise to 0.5 m, stabilize, complete five 4.5 m outbound/return rounds, hover six seconds after the fifth return, land, and confirm landed/disarmed feedback.
5. Save the result, stage logs, alignment, scene and mission report; confirm the actual aircraft remains grounded and disarmed.

`--fast-start` accelerates a fresh start after every computer reboot. The 15-second raw-odometry observation and 15-second quaternion survey run concurrently, after the existing grounded initialization. Once that new quaternion is applied and native vision is fresh, the unchanged 30-second continuous vision observation overlaps session-map capture and planner preparation. Both preparation and the qualified vision window must finish before the fresh scene is captured. If map preparation takes longer, vision monitoring continues through its completion, including source-identity and freshness checks. It never uses a previous boot's quaternion or an old scene. Omit the flag to retain the sequential workflow.

The component nodes continue to run in automatically managed tmux windows. Independent sampling runs in separate processes with logs and progress collected by the main terminal; manually opening desktop terminals is unnecessary for concurrency. A collector failure or cancellation reaps its parallel peer. Reports include `startup_policy`, per-stage start/end times and elapsed duration, and mark the vision stage as overlapping planner preparation. On a live ROS test with synthetic stationary publishers, identical 15-second collectors took 30.65 seconds sequentially and 15.40 seconds concurrently. This measures that sampling phase, not actual aircraft cold-boot latency; FC connection/parameter retrieval, sensor startup and data settling still determine the remaining wait. No sampling duration, quality threshold or mission waiting policy changes.

Alignment excludes body samples outside the existing -0.05..0.25 s receive-time header-age band before estimation, and reports excluded counts and raw age extrema. It still requires enough valid matched pairs and the existing pairing/rotation checks. Full-rehearsal preparation streams progress without the historical 120-second action deadline.

Vision preparation uses the test runner's existing 30-second observation duration as a continuous qualified window. This is a selected diagnostic observation length, not a SUPER or PX4 mandated delay; no thirty-second wait is inserted into individual mission legs. `VISION WAIT` records temporal interruptions and waits for a new complete window, with no total acquisition deadline and no stack restart. The original -0.05..0.25 s header-age band and 0.25 s receive-gap limit remain unchanged. Final logs retain lifetime statistics, including rejected intervals, followed by qualified-window statistics. Invalid/reordered stamps, wrong frames or changed/duplicate publishers still fail. This policy applies to preparation; it does not change flight-controller freshness handling.

The canonical route performs five complete round trips in one flight: ten navigation legs and 45 m nominal horizontal travel before curved-path adjustments. Endpoints stay tied to the original takeoff frame. There is one takeoff, one final six-second hover and one landing; intermediate returns continue to the next outbound leg. Logs print `Round trip n/5 complete`. Reports preserve all ten accepted goals/arrival records and require all twenty ordered stages plus `round_trips_completed=5` for success.

Ground rehearsals use a requested FC-centre floor height of **0.5 m**. The runner freezes this setting and `TAKEOFF_TO_0_5M` in its per-run `mission.json`; it does not rewrite the measured flight mission, which retains its stored height. Physical preparation can explicitly select **1.5 m** with `--flight-height 1.5` and a separate per-run mission copy. Both reports record the execution context and requested height. Physical preparation rejects rehearsal configurations. The same automatic test command selects 0.5 m without another flag.

During complete-demo map initialization, `Map capture WAIT` means the runner is still waiting for a fresh three-second raw-odometry window whose range is at most 2.5 cm per axis. There is no capture deadline: passing 12 seconds does not fail the rehearsal. Progress and observed ranges appear in the planner-stage log. Lost ground state, stale odometry or changed sources still stop initialization; a stable window is required before a map profile is written. Ctrl+C cancels the run.

The actual aircraft receives localization initialization. All flight requests in the rehearsal address the virtual backend. `--fixture-geometry` supplies explicitly synthetic aircraft dimensions and placement; it does not measure laboratory walls or establish physical clearance. A larger laboratory still needs a clear route in the captured map and a valid backup trajectory.

### Automatic QGroundControl startup

Before the actual stack is prepared, the demonstration rehearsal and physical preparation entrypoints open QGC or reuse an existing same-user instance. The default is `~/Downloads/QGroundControl-x86_64.AppImage`, with `QGroundControl.AppImage` and installed executable names as fallbacks. Use `--qgc-path /path/to/QGroundControl.AppImage` for another installed location. The launcher starts the existing executable; it does not download QGC or change its settings.

Retain the existing QGC setup: disable flight-controller USB/serial auto-connect and receive MAVROS UDP forwarding instead. QGC may open before that stream is available; opening the application is not proof of FC connection or preflight readiness. The run report records `qgc` and the launch log is `qgc.log`. Existing instances are reused without creating a second window. QGC is left open after test completion and managed stack shutdown.

`--no-qgc` skips it for the disarmed rehearsal, and `--logic-only` does not open the aircraft ground station. Missing desktop or executable is recorded during remote ground testing. Physical execution requires QGC to have been launched or already running; a launch failure is reported before stack preparation or any flight request. No application-start deadline is added.

### Automatic RViz2 preview

On the onboard desktop, the same command opens RViz2 after the isolated domain 197 check and virtual backend startup, before the controller starts its mission. No second terminal, configuration download or manual launch timing is needed. RViz shows the captured scene and occupied map as 5-pixel points, SUPER's exploration/backup trajectories and goals when published, and an orange virtual-vehicle body-heading arrow. The scene is a frozen snapshot; planning and virtual vehicle feedback update during the rehearsal. The arrow position uses the raw planner/world localization reference. Its direction is body +X transformed from the aligned body attitude into that same world frame, including the mounting rotation; it no longer displays the IMU +X axis as the nose. It represents heading, not velocity. Initial heading is held during return, so backward travel does not turn the arrow. The captured initial heading is fixed for the rehearsal; moving the physical aircraft afterward does not steer the virtual vehicle.

The runner owns this viewer process and closes it on completion, failure or interruption. Closing the viewer manually does not stop the test. Other desktop windows are unaffected. Close a viewer launched manually in domain 197 before starting a new rehearsal; the isolation check still rejects pre-existing nodes.

Append `--no-rviz` to disable the viewer. Without a graphical desktop or an installed `rviz2` executable, the runner skips the viewer and records why; the mission still has to pass its normal checks. Each run saves `rehearsal_view.rviz`, `rviz.log` when launched, and `rviz_status.json`; the nested mission report includes `rehearsal_viewer`. A successful process launch is not proof that the remote desktop rendered the window. Inspect `rviz.log` if no window appears.

The planner targets 2 m/s and 1 m/s² while accepting candidate trajectories up to 4 m/s and 2 m/s². Strict reference checks use those separate acceptance bounds. Observation mode records exceedances and retains 2.4 m/s and 1.5 m/s² as warning advisories; exceeding either alone does not stop the mission. Position, polynomial, continuity, freshness and landing checks remain active. See [motion acceptance](MOTION_ACCEPTANCE.md) and [the complete test specification](DEMONSTRATION_MISSION.md).

At each navigation goal, `WAIT_PLAN` holds the leg's starting hover point until the first verified trajectory arrives. This wait has no planning deadline in the complete rehearsal or physical demonstration controller. Console progress shows elapsed waiting time, and reports retain `planning_wait_policy` and `planner_wait_elapsed_s`. The complete controller and rehearsal have no total execution deadline or individual committed-trajectory duration cap. Stage transitions follow arrival, hold completion and landed/disarmed feedback. Reports record these execution limits as null. Planner exit, invalid input, stale telemetry, venue violations and operator stop remain active checks. Planning attempts that fail to produce a valid committed trajectory are not treated as mission success.

At a verified intermediate rest endpoint, the controller holds the committed position with zero velocity while SUPER prepares the next stage. This wait has no deadline, including when stationary endpoint commands continue after their polynomial ends. Callback scheduling gaps are recorded as advisories; the complete mission no longer aborts solely at a 0.2 s callback gap or 0.25 s reference stamp interval. Actual input validity, polynomial continuity and venue bounds remain checked. It does not complete the leg or initiate return early.

In observation mode, a replacement trajectory that starts from rest is checked against the preceding verified rest endpoint. Its anchor may differ within the two existing 2 cm command-to-polynomial position tolerances (4 cm combined). Continuity then includes the anchor difference and displacement of the new committed polynomial, rather than extrapolating only the expired stationary polynomial. Reports record these transitions in `stationary_replan_handoffs_by_leg`. A larger anchor difference, a moving restart, an unmatched command or a venue violation still fails its corresponding check.

SUPER may relocate a goal to a nearby free map cell. The supervisor confirms that accepted goal through the existing planner goal markers, checks it against the venue, and uses it for arrival and leg sequencing. Requested and accepted points and deviations remain in the report. Return is requested to original home; waiting and final hovering hold the actually reached accepted points without an unplanned jump. `nominal_waypoints_reached` separately reports accuracy to the original requested points.

For a separate one-metre rehearsal, append `--route-profile short-1m`. This changes the virtual outbound distance and cannot count as a completed 4.5 m demonstration or enable physical flight. Use the default full route in a suitable laboratory.

## Curved routes inside the venue

After takeoff, the entire configured vehicle envelope must remain inside the outer operating box. SUPER can replan curved routes around obstacles within that box; it is not constrained to a 10 cm tube around the starting heading. Initial-reference, tracking and landing-position offsets are recorded without a separate 10 cm abort rule. The 10 cm arrival tolerance still determines when the requested hover point has been reached; it does not define the permissible path. The former extra 10 cm route inset is removed from geometry validation.

Both planned positions/stage endpoints and measured vehicle positions are checked against the same venue. Freshness, source consistency, finite values, polynomial matching, reference continuity, planner backup constraints, heading policy and operator takeover remain separate requirements. These are data/control checks rather than a fixed straight-line route. QGC and RViz startup do not initiate a physical flight.

The forward ground rehearsal now also passes the same vehicle-centre box to A* search and every corridor-generation branch. This prevents the previous mismatch in which SUPER could plan outside the box and only then encounter the controller boundary check. It writes `venue_reference.json` and a separate per-run profile, preserving the captured cloud and existing geometry. It adds no boundary inset or mission deadline. [Planner-side venue injection](PLANNER_VENUE_BOUNDS.md) also supports the separate physical preparation entrypoint with measured mission geometry and a fresh live FC anchor. A frozen rehearsal does not establish physical clearance.

Finite-venue A* searches also run to goal/horizon completion or grid exhaustion, with no inherited 0.1/0.2-second elapsed-time cutoff. The upgraded planner logs `VENUE_SEARCH_COMPLETE_V1` at startup and detailed evidence when a search exhausts. Update the scripts, rerun the same venue patch command and rebuild once to upgrade an existing venue V1 installation; only two additional C++ files change. Profiles without venue halfspaces retain upstream search budgets. A full search can still return `NO_PATH` for the captured obstacles.

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

`RUNNING` is unfinished, including when a process was interrupted before it could finalize its report. On failure, retain the printed report and adjacent logs. The runner does not automatically retry, rebase or turn a partial mission into success. The October 16 timer-gap failure remains historical evidence; complete-demo callback gaps are now advisory. The current real-SUPER cloud replay completed five rounds, final hold and landing. See [the current scoped verification](../data/software_checks_2026-10-18/stationary_handoff_release.json). A complete onboard rerun after the matching full-release installation remains pending.

Ctrl+C is forwarded once by the outer startup wrapper to its owned runner. It waits for report finalization and owned-process cleanup instead of printing the outer `subprocess.wait` traceback. An operator-cancelled run remains incomplete: `RESULT: FAIL`, `cancelled_by_operator=true`, an explicit cancellation error and exit status 130. It is not reported as successful completion. Repeated Ctrl+C during cleanup does not send repeated interrupts into the runner's finalization.

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

## Recover an exited MAVROS in a core-only session

An observed startup reached the FC heartbeat and then aborted with
`std::future_error: Promise already satisfied` (exit code -6). The serial device
was still present. With MAVROS gone, `stop` cannot obtain fresh ground state and
the next `--restart` fails at the stop stage. The C++ crash's root cause remains
unresolved; increasing a planning or motion limit does not address it.

First run `super_session.py status`, inspect `mavros.log` in the printed session
directory and check `ls -l /dev/serial/by-id/`. For the observed case with only
`lidar`, `lio`, `mavros`, and `body` windows, with MAVROS exited and the other three
active, this explicitly restores the communication process. It refuses other
managed layouts and a second MAVROS process, and then requires fresh ground
telemetry. It does not automatically resume a controller or send a flight request.

```bash
(
set -e
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 - <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, str(Path.home() / 'Downloads'))
import super_session as m
s = m.Session(m.arguments(['start']))
s.load()
windows = s.windows()
if set(windows) != set(m.CORE):
    raise SystemExit('Not a core-only session; no recovery performed')
if windows.get('mavros') != '1':
    raise SystemExit('MAVROS is not exited; no duplicate launch')
if any(windows.get(n) != '0' for n in ('lidar', 'lio', 'body')):
    raise SystemExit('Another core process is down; inspect its log')
if any('mavros_node' in p for p in m.unmanaged_processes()):
    raise SystemExit('Another MAVROS process exists; no duplicate launch')
s.tmux('kill-window', '-t', s.name + ':mavros')
s.launch('mavros', s.core_commands()['mavros'])
gate = m.GroundGate()
gate.close()
print('MAVROS restored; connected/disarmed/on-ground state confirmed')
PY
)
```

After successful recovery, rerun the normal complete-test command. This recovery
worked on the reported aircraft session; it is not a permanent MAVROS crash fix
or a general-purpose force-stop command.

During vision startup, the current manager first reads `EKF2_EV_CTRL` and skips a
redundant write. If a write response times out, it performs a grounded readback:
the confirmed requested value permits continuation, while an unavailable or
different value blocks the action with a specific error. The write itself is
never replayed. Isolated DDS tests cover both applied and unapplied writes with
delayed replies. A successful readback does not conceal a dead MAVROS process.

## Prepare physical execution separately

The current physical command selects 1.5 m FC-centre height above the floor and five consecutive rounds of 4.5 m forward and return to the original hover point, followed by one six-second hold and landing. SUPER plans a new return trajectory to the original point; the return is not a recorded replay of the outbound path. The initial heading is retained, so a straight return flies backward without a yaw turn.

Before physical preparation, fill a separate measured mission file using the schema in `~/Downloads/open_day_mission.json`. The canonical operating box remains 5.8 m along the heading, 2.8 m wide and 2.8 m high, even in a larger laboratory. Survey that box, takeoff distances from its rear and left boundaries, and all six propeller-inclusive FC-centre extents. Replace every null geometry field with an actual measurement; do not substitute synthetic fixture values. The current floor calibration assumes an FC centre 0.085 m above the floor; a different installation requires a separate calibration change. Use the [measurement checklist and exact JSON fields](FLIGHT_SITE_MEASUREMENTS.md).

The physical entrypoint also requires current RC input/manual POSCTL, one controller, compatible PX4 parameter readbacks and on-site takeover. In particular, it expects `MPC_XY_VEL_MAX=2.0`; the last recorded aircraft setting was 0.5. It checks this setting and does not silently change it. Actual receipt of the new raw position-plus-velocity setpoints and complete physical tracking remain to be verified.

With the measured configuration saved as `~/Downloads/open_day_mission_measured.json`, prepare without flight execution:

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_measured.json" \
  --flight-height 1.5 \
  --motion-policy observe
```

This starts a new stack, captures a fresh quaternion and prepares the real planner/controller. It writes `venue_reference.json` in the managed session log directory and a separate live venue planner YAML. The six planes use the supplied aircraft extents and surveyed wall offsets, the captured FC ground position/yaw and the paired localization origin. The controller reads this same record, verifies the source identities, file hashes and running SUPER `config_name`, and retains the captured anchor at READY. It changes no source YAML, cloud, speed target, acceptance bound or mission deadline. Existing compiled venue support is required; this Python integration needs no additional C++ rebuild if all nine current markers, including `VENUE_MAP_HEIGHT_V1`, are already installed. `PREPARATION: READY; MISSION: NOT RUN (check-only)` means preparation passed; the check-only controller then exits. Reports are saved under `~/Downloads/super_demo_reports/<timestamp>/`. This entrypoint requires actual mission geometry even in check-only mode.

For a supervised physical trial after prerequisites and clearance have been confirmed, run the separate execution entry in terminal A:

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_measured.json" \
  --flight-height 1.5 \
  --motion-policy observe --execute --on-site-supervised
```

Keep terminal A running. It prepares another fresh session and waits for `Phase: READY` and an explicit start. With the operator at the aircraft and RC takeover available, terminal B sends the start request once:

```bash
source /opt/ros/humble/setup.zsh
ros2 service call /super_demo/start std_srvs/srv/Trigger '{}'
```

The start service is flight initiation. Follow the controller's acceptance or rejection; a rejected start does not authorize another mode or arming command. Rebooting the computer, opening a terminal, installing scripts or running the rehearsal cannot initiate this physical mission.
