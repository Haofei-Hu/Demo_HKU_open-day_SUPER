# Select lateral venue boundaries in RViz before preparation

Use `super_demo_test.py --calibrate-venue` for an automatic rehearsal, or
`super_demo.py --calibrate-venue` for physical preparation. Select the left and right boundaries in
RViz after fresh localization and alignment, before SUPER or the mission
controller starts. This avoids entering a new left-wall distance for each takeoff
position. It does not detect a net or relocalize against a saved map.

Left and right mean **looking along the aircraft nose**, regardless of which
side has a wall or a table. Point the aircraft along the 5.8 m venue length before
starting. Two side points define parallel boundaries along that confirmed heading;
they do not estimate the heading of a wall. Select the actual boundary surfaces,
not objects inside the venue. The two points can be at different heights and
forward positions. Environmental points are rotated into the aligned frame
without applying the vehicle's IMU-to-FC lever arm.

## Select points, then automatically run the complete rehearsal

After installation, use one command:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics \
  --fixture-geometry --motion-policy observe --fast-start \
  --calibrate-venue --rear-offset 0.5
```

The runner opens QGC, starts fresh localization and alignment, opens the frozen
RViz selection view, and waits for the two boundary picks and `confirm` in the
launching terminal. Confirmation automatically continues to planner preparation,
the existing fresh vision observation, map capture and the complete real-SUPER
virtual rehearsal. **Do not add `--calibration-only` for this workflow.**

The selection window closes after confirmation because the rehearsal viewer runs
in the separate simulation ROS domain. Rehearsal RViz opens automatically when
preparation finishes; the terminal reports progress between the two windows.
No second launch or start service is needed for the virtual mission.

To rehearse at **1.5 m height inside a maximum 2 m total lateral width**, use:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics \
  --fixture-geometry --motion-policy observe --fast-start \
  --calibrate-venue --rear-offset 0.5 \
  --rehearsal-width 2 --rehearsal-height 1.5
```

Select the actual side boundaries as usual. The width option intersects those
boundaries with a strip extending 1 m left and 1 m right of the takeoff centre.
A nearer selected side remains nearer; the box never expands beyond the survey.
The cyan preview shows the resulting box before confirmation, and SUPER search,
corridors and controller use that same box, including vehicle extents. The report
retains both the original picks and the requested width cap. These optional
rehearsal settings do not change the physical mission or the default rehearsal.
Narrowing the box can remove an obstacle-avoidance route; it does not guarantee
that a route exists inside it.

By default, the virtual route uses a 0.5 m takeoff target, five 4.5 m out-and-back rounds,
six seconds at home, then landing. `--fixture-geometry` supplies labelled virtual
aircraft dimensions; the final left/right placement comes from the picks, not the
fixture's default centre. The exact selected width and fixed heading/origin are
carried into the captured scene. Later pose samples cannot shift that box.
The original cloud and mission template are not edited.

The final result and combined report are saved under
`~/Downloads/super_demo_tests/<timestamp>/report.json`. That directory also holds
the calibration, frozen mission, scene and mission report. A successful
rehearsal does not establish physical flight readiness. Each `--restart` uses a
new localization session and requires new picks; do not feed a previous session's
calibration-only file into a new session.

## Try selection with the existing template

This mode accepts an incomplete aircraft-envelope template. It confirms only the
selected venue and **does not prepare or execute flight**:

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission.json" \
  --flight-height 1.5 \
  --calibrate-venue --calibration-only --rear-offset 0.5
```

`--rear-offset 0.5` uses the operator's previously supplied rear placement.
Left/right selection does not measure that distance. Keep the FC at that rear
offset, or supply the appropriate value. Length and height remain 5.8 m and
2.8 m; they are displayed rather than inferred from the clicks.

1. The runner prepares localization and vision, then automatically opens RViz
   centred near the aircraft. It accumulates one second of fresh display points,
   prints **FROZEN MAP READY**, and freezes the displayed cloud. Live localization
   continues separately; the map does not refresh while selecting.
2. **Publish Point** is selected automatically. Click a visible cloud point on the
   **left boundary**, then the **right boundary**. The green and red markers are
   labelled **1 LEFT** and **2 RIGHT**. Empty background does not produce a point.
   If another tool is active, press **U** with focus on the RViz view to return to
   Publish Point. Move Camera can be used to change the view, then press U again.
3. Check the complete cyan box against the actual venue. The terminal shows the
   computed left/right clearances, width and rear offset. **The two selected
   lateral boundaries are used exactly**, including widths greater than 2.8 m.
   No width cap or recentering is applied. Length remains 5.8 m and height 2.8 m.
4. Type `confirm` and Enter in the launching terminal to accept, or `reset` and
   Enter to clear both points while keeping the frozen cloud. Type `refresh` for
   a new display snapshot; this also clears both picks. Close RViz or press Ctrl+C
   to cancel. There is no selection deadline.

For a terminal without interactive stdin, the equivalent services are:

```bash
ros2 service call /super_venue_calibration/confirm std_srvs/srv/Trigger '{}'
# Or discard both points:
ros2 service call /super_venue_calibration/reset std_srvs/srv/Trigger '{}'
# Or replace the frozen cloud and clear both points:
ros2 service call /super_venue_calibration/refresh std_srvs/srv/Trigger '{}'
```

Confirmation queues the final fresh-session checks; it is not a flight command.
The display uses five-pixel squares to make sparse points easier to hit. Clicks
must belong to the current snapshot epoch; they do not use the 250 ms live-pose
age threshold. Sensor and controller freshness checks remain in force. A frozen
picture improves selection usability; it does not correct localization drift.
The aircraft must still lie between the selected sides, with LEFT/RIGHT defined
along its nose. If this check fails, both markers remain visible and the terminal
shows each point's signed lateral coordinate; type `reset` to correct them.
Successful selection prints:

```text
CALIBRATION: SUCCESS; FLIGHT READINESS: NOT ASSESSED; MISSION: NOT RUN
```

The run directory under `~/Downloads/super_demo_reports/` contains
`venue_calibration.json`, `flight_mission.json`, `venue_calibration.rviz`,
`venue_calibration_rviz.log` and `preparation.json`. Original mission measurements
are not overwritten. `flight_mission.json` may still have missing aircraft
measurements in calibration-only mode and is not flight-ready merely because
selection passed.

## Integrate selection into each physical preparation

Supply an aircraft mission with its propeller-inclusive envelope already filled.
Its left and rear placement fields may be empty when `--rear-offset` is supplied:

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_measured.json" \
  --flight-height 1.5 --motion-policy observe \
  --calibrate-venue --rear-offset 0.5
```

After confirmation the runner creates a per-run physical mission, validates the
aircraft envelope and route against the marked box, and continues to the existing
check-only planner/controller preparation. The calibrated anchor is shared by
planner and controller and is not shifted to a newer FC sample.

The supervised execution entrypoint accepts the same calibration options in
addition to its existing `--execute --on-site-supervised` flags. Selection is
still required before READY. The separate explicit start request is still
required; selecting boundaries never arms or starts the mission. See the
[physical flight guide](PHYSICAL_FLIGHT.md).

## Session validity and limitations

The calibration is tied to the localization publisher identities, alignment file,
origin, grounded pose and heading. Restarting localization requires new clicks.
Changing the placement during preparation invalidates the existing grounded
checks. Do not reuse a per-run calibrated mission as a placement survey for a
new session. The source mission can be reused with fresh `--calibrate-venue`.

A fresh source sample cannot move the already confirmed walls. A missing source,
changed origin or native control publisher blocks calibration. The actual
boundary must remain identifiable by the operator; marking a nearby obstacle
instead of the boundary does not establish the venue. RViz point-picking and
localization uncertainty remain measurement errors, not a guarantee of physical
containment. Measured vehicle extents and the existing planner/controller checks
remain necessary.
