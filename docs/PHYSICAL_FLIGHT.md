# Complete physical demonstration: 1.5 m

Sequence: **rise to an FC-centre height of 1.5 m above the floor → fly 4.5 m outbound and return to the original hover point, five round trips → hover for 6 seconds → land and confirm disarmed**. Planning targets remain 2 m/s and 1 m/s²; acceptance bounds remain 4 m/s and 2 m/s².

For the initial **1.0 m hover-height trial**, explicitly use `--flight-height 1.0`
in the preparation command below. This sets takeoff, outbound target and return
hover heights to 1.0 m above the floor at the FC centre. Keep `--venue-ceiling 2`:
the outer ceiling remains 2 m, and avoidance can change altitude inside that box.
The measured source file is preserved; the effective height appears in the
per-run `flight_mission.json` and report. Omitting the height option still uses
the existing 1.5 m default.

`super_demo.py` is the physical-flight entrypoint. First use the [complete installer](QUICK_START.md#install-or-update-once) for matching release `560633172b32a12e95c96ee0d5c7b4613aac2e41`. The matching release includes the continuous-rest-start C++ correction. Complete the one-time venue patch and rebuild in the quick start before preparing physical flight.

## 1. Use the measured configuration for this placement

The mission file is `~/Downloads/open_day_mission_measured.json`. When reusing measurements, confirm that they describe the current aircraft and takeoff placement. The default outer venue is 5.8 × 2.8 × 2.8 m, with the nose along the 5.8 m dimension. RViz lateral calibration uses the selected left and right boundaries exactly, including widths greater than 2.8 m. Rear and left offsets are measured from the FC centre.

Check the file and aircraft-envelope clearance at 1.5 m. This command starts neither ROS nor flight:

```bash
python3 - <<'PY'
from pathlib import Path
import json, sys
base=Path.home()/'Downloads'
sys.path.insert(0,str(base))
from demo_mission import DemoMission
p=base/'open_day_mission_measured.json'
if not p.is_file():
    raise SystemExit('Missing measured configuration: '+str(p)+'; follow the measurement guide, not a virtual fixture')
c=DemoMission.physical_height(json.loads(p.read_text()),1.5)
m=DemoMission(c)
print('Takeoff placement:',c['takeoff'])
print('Aircraft envelope:',c['vehicle'])
print('Waypoints:',m.waypoints())
print('Round trips:',m.round_trips)
if m.round_trips!=5:
    raise SystemExit('The measured file does not specify five round trips; check the mission configuration')
print('CONFIG: PASS; file validation only, not confirmation of on-site flight readiness')
PY
```

If the file is missing, copy `open_day_mission.json` to the separate measured filename and fill the [measurement fields](FLIGHT_SITE_MEASUREMENTS.md). An unspecified left offset cannot be inferred from approximate placement; the template's `null` values are not automatically replaced. Estimates based on 5-inch props and a 205 mm wheelbase are documented in the measurement guide and do not establish the envelope of protruding sensors or batteries.

If takeoff placement changes, use [RViz lateral calibration](RVIZ_VENUE_CALIBRATION.md)
to select left/right sides before each preparation instead of measuring and
entering the lateral offset. The aircraft-envelope measurements are still needed.
For that workflow the left-offset field may remain unset in the source file;
the full-file validation above applies only after placement has been supplied.
Do not fill it with an assumed centred placement to make validation pass.

## 2. Prepare physical execution after startup

Have an on-site RC operator and confirm the required POSCTL/manual input, valid localization, and parameter readbacks. The controller checks these conditions. PX4 speed settings and planner targets are separate: physical preparation currently expects `MPC_XY_VEL_MAX=2.0`; an earlier aircraft readback was 0.5. A mismatch stops preparation with a report rather than silently changing the value.

Terminal A:

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_measured.json" \
  --flight-height 1.5 \
  --calibrate-venue --rear-offset 0.5 \
  --venue-width 2 --venue-ceiling 2 \
  --motion-policy observe --execute --on-site-supervised
```

The script opens or reuses QGC, starts localization, measures the alignment quaternion,
and prepares external vision. Select the actual left/right boundaries in the frozen
RViz view and inspect the cyan box before typing `confirm`. The explicit width
option intersects the selected sides with a strip extending 1 m either side of
the takeoff centre. The ceiling is 2 m above the floor, with the aircraft's top
extent subtracted for FC-centre checks. The same box goes to SUPER search,
corridors and the controller. It then waits for `Phase: READY`. Keep terminal A
running. These options narrow the selected venue; they do not alter or substitute
the measured aircraft envelope. Omitting them uses the selected sides and the
original 2.8 m ceiling.

The physical entrypoint defaults to 1.5 m, including when the stored survey contains a different height. `--flight-height 1.5` explicitly selects that default and saves the effective mission as `flight_mission.json` in the run's report directory without overwriting the measured source file. The height is **1.5 m from the takeoff floor to the FC centre**. With the existing grounded FC-height calibration of 0.085 m, the requested rise is approximately 1.415 m. Ground rehearsal defaults to 0.5 m and accepts an explicit `--rehearsal-height 1.5`.

For preparation checks only, omit `--execute --on-site-supervised`; that mode does not execute flight.

## 3. Start after READY

Terminal B:

```bash
source /opt/ros/humble/setup.zsh
ros2 service call /super_demo/start std_srvs/srv/Trigger '{}'
```

**This call initiates physical flight.** Once accepted, the controller requests OFFBOARD, arms, and executes the mission. If rejected, resolve the reported reason; do not bypass preparation with a separate arming request.

SUPER may alter the path and altitude to avoid obstacles. The 1.5 m setting is the takeoff, hover, and mission target height, not a fixed-altitude two-dimensional route. The return target is the original hover point with the initial heading retained. Venue and propeller-inclusive aircraft-envelope checks remain active.

## 4. Status, early landing, and results

Use terminal B to inspect mission status:

```bash
timeout 10s ros2 topic echo \
  /super_debug/super_flight_status std_msgs/msg/String \
  --no-daemon --qos-reliability best_effort \
  --qos-durability volatile --full-length --field data
```

To request early landing while the controller is still operating:

```bash
ros2 service call /super_demo/land std_srvs/srv/Trigger '{}'
```

RC takeover is an independent on-site control path. `Ctrl+C` terminates the process; it is not a hover command or confirmation of landing. Stop the background stack only after landing and disarming.

Reports are saved under `~/Downloads/super_demo_reports/<timestamp>/`:

- `flight_mission.json`: effective 1.5 m mission copy.
- `preparation.json`: preparation results and stage-log paths.
- `report.json`: flight stages, completed round trips, and final result.

The release has been checked for 1.5 m configuration handling, ROS controller initialization, and start conditions. **A complete 1.5 m physical demonstration on this aircraft has not yet been verified.** Retain the entire report directory after the field trial.
