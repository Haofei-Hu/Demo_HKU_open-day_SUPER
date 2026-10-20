# Quick start after power-on or reboot

Matching release: `3368af293148d2c0ddf03a26c9417407b9532ec8`.
These commands run a **virtual ground rehearsal on an actual LiDAR map**. The physical aircraft remains stationary and disarmed; a virtual vehicle executes the motion.

## Install or update once

Finish any running automatic test before updating. Use the complete installer to avoid mixing script versions. It downloads and verifies all 28 files before backing up and replacing changed files. Existing alignment and measured mission configurations are preserved according to the installer rules.

```bash
(
set -e
demo_revision='3368af293148d2c0ddf03a26c9417407b9532ec8'
demo_update_dir=$(mktemp -d "$HOME/Downloads/demo_full_update_XXXXXX")

curl -q --noproxy '*' -fL \
  --connect-timeout 10 --max-time 30 \
  "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/$demo_revision/scripts/maintenance/install_demo_test.py" \
  -o "$demo_update_dir/install_demo_test.py"

python3 -m py_compile "$demo_update_dir/install_demo_test.py"
python3 "$demo_update_dir/install_demo_test.py" --revision "$demo_revision"
)
```

This release includes C++ corrections for continuous stationary starts and consistent map-height bounds. **Apply the venue patch and rebuild once after updating**, even if an earlier venue patch is already installed:

```bash
(
set -e
if tmux -L super-aircraft has-session -t super-flight 2>/dev/null; then
  python3 ~/Downloads/super_session.py stop
fi
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

Finish the current ground rehearsal before this update, and stop the stack only when the physical aircraft is landed and disarmed. Older installations without the motion-acceptance patch should follow the [complete one-time patch procedure](REBOOT_QUICK_START.md#install-or-update-the-matching-scripts). Subsequent normal reboots require no rebuild.

## Run after every reboot

Power the flight controller and LiDAR, and keep the aircraft stationary at its takeoff location. Connect to the onboard Ubuntu desktop through NoMachine and open one terminal:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics \
  --fixture-geometry --motion-policy observe --fast-start
```

A normal reboot requires no download, rebuild, manual `source`, or quaternion copying. The runner automatically:

1. Opens or reuses QGC and starts the LiDAR, FAST_LIO, MAVROS, and body-frame conversion.
2. Observes fresh odometry and measures the session quaternion in parallel, applies alignment, and starts external vision.
3. Prepares the map/planner alongside a continuous 30-second fresh-localization observation, then captures the scene.
4. Opens RViz and runs the complete rehearsal with real SUPER and virtual vehicle feedback.
5. Prints `RESULT: SUCCESS` or `RESULT: FAIL` and saves reports and stage logs.

`--fast-start` runs independent preparation steps concurrently; each run still performs fresh measurements and mapping. The 30-second observation belongs to preparation, not to each route segment. Closing the terminal does not stop background tmux nodes. Running the command again checks grounded state before clearing the previous session.

## Mission settings

| Setting | Value |
|---|---|
| Virtual takeoff target | FC centre 0.5 m above the floor |
| Physical target | FC centre 1.5 m above the floor, explicitly selected with `--flight-height 1.5` |
| Route | 4.5 m outbound and return to the original hover point, five round trips |
| Return behavior | SUPER plans a new return path while retaining the initial heading |
| Final stages | Hover for 6 seconds, land, confirm landed and disarmed |
| Planning targets | 2 m/s, 1 m/s² |
| Trajectory acceptance bounds | 4 m/s, 2 m/s² |
| Controller motion policy | `observe` records speed/acceleration exceedances; those exceedances alone do not terminate the mission |
| RViz point clouds | `Size (Pixels): 5` |
| Planning and whole-mission deadlines | None; arrival, hover, and landing feedback advance the stages |

SUPER may follow curved obstacle-avoiding paths. After takeoff, position checks use the outer venue and aircraft envelope, not a narrow corridor along the takeoff axis. `--fixture-geometry` uses explicitly synthetic extents and placement; these must not be used as a physical survey. RViz closes when the rehearsal ends or fails; QGC remains open.

## Reports and results

Each run prints its own directory:

```text
~/Downloads/super_demo_tests/<timestamp>/
  report.json             Overall workflow result
  mission_report.json     Round trips, hover, and landing result
  alignment.json          Session quaternion measurement
  scene.json              Captured map and coordinate information
  planner_rehearsal.log   SUPER rehearsal log
  venue_reference.json    Session venue frame and boundaries
```

Files for later stages may not exist if preparation failed. Read the latest result:

```bash
python3 - <<'PY'
from pathlib import Path
import json
paths = sorted((Path.home() / 'Downloads/super_demo_tests').glob('*/report.json'))
if not paths:
    raise SystemExit('No rehearsal report found')
p = paths[-1]
for name in ('report.json', 'mission_report.json'):
    f = p.parent / name
    if not f.exists():
        continue
    r = json.loads(f.read_text())
    print('\nREPORT:', f)
    for key in ('result', 'stage', 'error', 'round_trips_completed'):
        print(key + ':', r.get(key))
PY
```

`RUNNING` means incomplete, including reports left by an interrupted process. Keep the entire run directory. Seeing one completed leg in RViz does not establish that all five rounds succeeded.

## Inspect or stop the background stack

```bash
python3 ~/Downloads/super_session.py status
```

After confirming that the physical aircraft is landed and disarmed:

```bash
python3 ~/Downloads/super_session.py stop
```

## Known issues and recovery

| Symptom | Meaning and action |
|---|---|
| `inverse_goal() takes 3 positional arguments but 4 were given` | Mixed script versions; install the complete release and rerun |
| `set_parameters response` timeout | The manager reads back `EKF2_EV_CTRL` and continues only if the requested value is confirmed; it does not replay the write |
| `mavros: EXITED` and grounded-state timeout during stop | Inspect the MAVROS log; an exited bridge cannot supply state. Use the [restricted core-only recovery procedure](REBOOT_QUICK_START.md#recover-an-exited-mavros-in-a-core-only-session) |
| `std::future_error: Promise already satisfied` | Observed MAVROS crash with no confirmed permanent root-cause fix |
| `Odom above virtual ceil` followed by an occupied rest start | Confirm the rebuilt planner logs `VENUE_MAP_HEIGHT_V1`. Search, all corridors and complete candidate trajectories must share the initialized map-height bounds; do not enlarge the ceiling to hide a violation |
| `position step` after a stationary replan | Confirm the rebuilt planner logs `VENUE_REST_START_V1`. It retains the measured start instead of relocating it to a grid centre; controller continuity checks remain active |
| Persistent `WAIT_PLAN` | No executable trajectory yet; inspect this run's `planner_rehearsal.log` for path-search, corridor, or optimization failures |

See [validation scope and known limitations](VALIDATION.md). The complete physical five-round mission remains unverified. Use the separate [1.5 m physical flight guide](PHYSICAL_FLIGHT.md) with [measured aircraft geometry and takeoff placement](FLIGHT_SITE_MEASUREMENTS.md).

For variable takeoff placement, [select the left/right venue boundaries in RViz](RVIZ_VENUE_CALIBRATION.md) before physical preparation. The ground rehearsal command above is unchanged.
