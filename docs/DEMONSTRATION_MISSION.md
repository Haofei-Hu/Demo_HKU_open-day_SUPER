# Complete demonstration test

The automatic acceptance target is the entire [canonical demonstration](../configs/demonstration/open_day_mission.json): take off to an FC-centre floor height of 2 m, stabilize, travel forward 4.5 m, stop, return to the original hover point, hold six seconds, request AUTO.LAND, and confirm landed and disarmed feedback. Initial heading is retained. Horizontal speed is capped at 2 m/s and acceleration at 1 m/s²; the route need not reach either cap. Initial, endpoint and tracking errors remain limited to 10 cm per axis.

## From a new terminal

The existing onboard ROS workspaces, tmux, localization helpers and MAVROS installation are prerequisites. Install one matching release using `scripts/maintenance/install_demo_test.py --revision <full-commit-SHA>`. The installer validates every downloaded file against the pinned release manifest before replacing code, saves backups, preserves measured alignment and retains an existing mission configuration. It does not start ROS.

Keep the physical aircraft stationary on its takeoff floor and disarmed. Run:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry
```

The entrypoint sources the installed workspaces itself. It:

1. Stops the managed grounded session, starts MID360/FAST_LIO/MAVROS/body conversion and measures a fresh alignment.
2. Applies this run's quaternion, initializes vision using the existing readback-confirmed procedure, and creates the session-relative demonstration map profile.
3. Checks the real vision chain for 30 seconds and captures a fresh occupied map plus paired raw/aligned geometry.
4. Starts real SUPER in localhost-only ROS domain 197, with the captured static obstacle model and virtual FC/vehicle feedback. All takeoff, OFFBOARD, arming and landing operations in this rehearsal address that virtual backend.
5. Runs every mission stage in order, checks the final return hold and landing confirmation, and confirms the actual aircraft remains grounded and disarmed.
6. Saves the report and prints `RESULT: SUCCESS` or `RESULT: FAIL`, `REPORT: <path>` and the execution scope. Any failure exits nonzero. It never substitutes a synthetic planner when real SUPER fails.

The actual aircraft receives localization initialization but no native flight setpoints, mode or arming requests from this test. The existing guarded EV initialization is retained; the test does not change PX4 speed caps, aircraft geometry or error limits. The real source YAML remains intact. The private rehearsal profile changes only the input cloud topic and disables raycasting so a frozen occupied model does not invent clearing rays from virtual viewpoints. Both adaptations are reported.

## Geometry and scope

The venue is an outer box: length 5.8 m, width 2.8 m, height 2.8 m. Forward follows the long edge. The canonical file leaves the FC-centre distances from the rear and left walls and the aircraft's six propeller-inclusive extents unset until measured. A session quaternion does not survey those walls. Different valid takeoff positions can be configured without rewriting the mission.

`--fixture-geometry` explicitly supplies a virtual test placement (0.5 m from the rear, 1.4 m from the left) and virtual aircraft extents. The frozen report marks these as synthetic. They are not measurements of the actual aircraft; `physical_venue_clearance_verified` remains false. Physical execution refuses synthetic geometry. Once actual geometry is measured, use a separate measured mission file with `--mission-file` and omit the fixture flag.

For development without the aircraft stack or actual SUPER:

```bash
python3 ~/Downloads/super_demo_test.py --logic-only --fixture-geometry
```

This is explicitly a synthetic logic test. Its report has `actual_localization_tested=false` and `actual_SUPER_tested=false`. It cannot establish real planning, physical tracking, or flight completion.

## Short-site rehearsal

For a current location with approximately 1.5 m from the aircraft centre to the forward obstacle and sufficient height, select the separate one-metre test:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry \
  --route-profile short-1m
```

This retains the 2 m climb, return to home, six-second hover and confirmed landing. It changes only the rehearsal's outbound distance to 1 m. Speed and acceleration remain capped at 2 m/s and 1 m/s²; a one-metre stop-to-stop route cannot establish a 2 m/s speed trial. Both the actual map's obstacles and the backup trajectory must still permit the route. No backup or obstacle checks are disabled.

The canonical mission file remains unchanged at 4.5 m. The short test freezes a separate `short_site_1m_rehearsal` configuration and uses `SUPER_OUTBOUND_1M` in its expected/completed stages. Its virtual box is 2 m long with the test takeoff centre 0.5 m from its rear, representing 1.5 m forward room. The box width, height and aircraft extents remain labelled test geometry; this is not a survey of the current room or physical flight clearance. Short profiles cannot enable physical execution. `SUCCESS` here has `demonstration_completed=false` and cannot count as a completed 4.5 m demonstration.

## Reports and failure interpretation

Each run creates `~/Downloads/super_demo_tests/<timestamp>/report.json`, stage logs, the captured alignment, a frozen mission file and a nested `mission_report.json`. Default rehearsal runs also retain the scene snapshot and private SUPER log. Upload the top-level report to retain the failed stage and the last 80 lines of a failed subprocess; the full files remain beside it. When the private SUPER log exists, the top-level report now also retains its last 100 lines and counts exploration successes, backup optimization/constraint failures, accepted goals and missing-odometry messages. These are evidence markers, not automatic root-cause assertions.

A complete result requires all eight ordered stages, two navigation goals, the return hold, fresh landing-position feedback within 10 cm per horizontal axis of home, and fresh landed/disarmed confirmation. Completing the outbound leg alone cannot pass. The report identifies actual localization and SUPER usage, the virtual backend, completed stages, reference peaks, time-aligned tracking extrema and any fault. Native publication in the private graph is not proof of MAVROS-to-PX4 transport on the aircraft.

SUPER may commit an exploration prefix followed by a backup braking segment. Such an intermediate endpoint may change during continuous replanning; it is not final navigation completion. Every accepted command still requires fresh increasing stamps, matching committed polynomials, corridor/body bounds, motion limits and continuity. Final holding requires the original requested navigation goal within 10 cm per axis. Feedback is compared to the sent reference at the feedback's original measurement timestamp, with no timestamp rewriting.

## Validation status

The complete synthetic ROS lifecycle has passed from the flat deployment directory, including return, six-second hold and confirmed landing. An injected return-command dropout produced `FAIL` rather than partial success. Source identity, snapshot capture, time-aligned tracking, rolling-stage handling, release checks and existing ground-manager tests have regression coverage. See [the software validation record](PROGRESS_2026-10-16.md).

The first onboard real-SUPER rehearsal passed real alignment, map capture and 30-second vision sampling, then failed at outbound planning: exploration optimization succeeded while backup optimization repeatedly failed. The current site was subsequently reported to have only approximately 1.5 m forward room, compared with the requested 4.5 m route. A complete onboard real-SUPER rehearsal is still unverified. Actual full-mission flight, dynamic obstacle response, physical tracking and surveyed clearance remain unverified. The existing separate 1 m / six-second aircraft hover and landing result does not validate this new mission. Future supervised execution uses a separate entrypoint and requires measured geometry, compatible readback-confirmed PX4 limits, fresh RC/manual POSCTL and an explicit start request; restarting a terminal cannot initiate flight.
