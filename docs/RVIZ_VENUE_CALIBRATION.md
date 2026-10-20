# Select lateral venue boundaries in RViz before preparation

Use `super_demo.py --calibrate-venue` to select the left and right boundaries in
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
   computed left/right clearances, width and rear offset. The selected width is
   capped at 2.8 m, centred between the two marked sides. A narrower selection is
   retained as a narrower box; it is never expanded past a selected boundary.
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
