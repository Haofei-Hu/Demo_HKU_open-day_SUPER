# Complete demonstration test

The automatic acceptance target is the entire [canonical demonstration](../configs/demonstration/open_day_mission.json): take off to an FC-centre floor height of 1.5 m, stabilize, travel forward 4.5 m, stop, return to the original hover point, hold six seconds, request AUTO.LAND, and confirm landed and disarmed feedback. Initial heading is retained. Planner motion targets are 2 m/s and 1 m/s²; the selected downstream motion policy determines whether derivative exceedances stop tracking or are recorded as warnings. After takeoff, planned and measured vehicle positions are bounded by the configured outer venue, including the aircraft envelope. SUPER may use curved paths; 10 cm is retained for arrival detection rather than as a route or tracking-error abort threshold.

## From a new terminal

For the recommended commands after every power-on or reboot, including the pinned installation and separate physical entrypoint, use [the reboot quick start](REBOOT_QUICK_START.md).

The existing onboard ROS workspaces, tmux, localization helpers and MAVROS installation are prerequisites. Install one matching release using `scripts/maintenance/install_demo_test.py --revision <full-commit-SHA>`. The installer validates every downloaded file against the pinned release manifest before replacing code, saves backups, preserves measured alignment and retains an existing mission configuration. It does not start ROS.

Keep the physical aircraft stationary on its takeoff floor and disarmed. Run:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry
```

The entrypoint sources the installed workspaces itself. It:

1. Opens or reuses QGC, stops the managed grounded session, starts MID360/FAST_LIO/MAVROS/body conversion and measures a fresh alignment.
2. Applies this run's quaternion, initializes vision using the existing readback-confirmed procedure, and creates the session-relative demonstration map profile.
3. Checks the real vision chain for 30 seconds and captures a fresh occupied map plus paired raw/aligned geometry.
4. Starts real SUPER in localhost-only ROS domain 197, with the captured static obstacle model and virtual FC/vehicle feedback. All takeoff, OFFBOARD, arming and landing operations in this rehearsal address that virtual backend.
5. Runs every mission stage in order, checks the final return hold and landing confirmation, and confirms the actual aircraft remains grounded and disarmed.
6. Saves the report and prints `RESULT: SUCCESS` or `RESULT: FAIL`, `REPORT: <path>` and the execution scope. Any failure exits nonzero. It never substitutes a synthetic planner when real SUPER fails.

On a desktop, RViz2 is managed automatically in private domain 197: it starts after the isolation check and backend startup, and closes when the rehearsal finishes or is interrupted. It displays the frozen occupied scene, real SUPER planning markers, and a raw-world virtual-vehicle pose arrow. No identity transform is applied to aligned ENU or MAVROS map poses. `--no-rviz` disables it; missing desktop/RViz is recorded as skipped. Viewer logs/status are stored beside the report, and the nested mission report records its lifecycle independently of mission success. See [automatic preview behavior](REBOOT_QUICK_START.md#automatic-rviz2-preview).

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

This retains the 1.5 m climb, return to home, six-second hover and confirmed landing. It changes only the rehearsal's outbound distance to 1 m. Planner motion targets remain 2 m/s and 1 m/s²; a one-metre stop-to-stop route does not establish a 2 m/s speed trial. Both the actual map's obstacles and the backup trajectory must still permit the route. No backup or obstacle checks are disabled.

The canonical mission file remains unchanged at 4.5 m. The short test freezes a separate `short_site_1m_rehearsal` configuration and uses `SUPER_OUTBOUND_1M` in its expected/completed stages. Its virtual box is 2 m long with the test takeoff centre 0.5 m from its rear, representing 1.5 m forward room. The box width, height and aircraft extents remain labelled test geometry; this is not a survey of the current room or physical flight clearance. Short profiles cannot enable physical execution. `SUCCESS` here has `demonstration_completed=false` and cannot count as a completed 4.5 m demonstration.

## Motion targets and flight policy

The ground-test CLI retains its diagnostic default `--motion-policy strict`, rejecting advertised speed above 2 m/s or acceleration above 1 m/s². The demonstration preparation and controller CLIs default to `observe`, including explicitly enabled flight execution. To use observation during the ground workflow:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry \
  --route-profile short-1m --motion-policy observe
```

Observation records per-leg derivative peaks, 2/1 target-exceedance counts and the last violating sample. Advisory thresholds are 2.4 m/s and 1.5 m/s²; passing either produces a warning and separate per-leg advisory counts. Neither target nor advisory exceedance alone stops tracking in this policy. Completing with warnings does not establish 2/1 or 2.4/1.5 motion compliance. Explicit `--motion-policy strict` retains exact 2/1 rejection. The library constructor still defaults to strict for callers that do not select a policy.

Polynomial agreement, finite values, freshness, source identities, polynomial-based position continuity in observation mode, venue/obstacle bounds, arrival detection and confirmed landing remain required. Observation does not disable SUPER's backup planner or turn a failed stage into success. It does not change YAML motion values, PX4 parameters, optimizer penalties or the existing 15 mm continuity margin. The complete mission does not reject a reference solely for a long stamp interval; its position step is checked against the prior committed polynomial. Observation mode calculates the displacement allowance from the previous committed polynomial rather than assuming a hard 2 m/s speed bound. A newly received command cannot enlarge that allowance by advertising a higher velocity. Strict mode retains its configured-speed bound.

Pinned SUPER uses penalty-based derivative constraints: the [exploration optimizer](https://github.com/hku-mars/SUPER/blob/2ad3419c127a617c6d7df6925e81a14175a9c096/super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp#L715) checks acceleration penalty against a configured margin, and its final velocity-penalty rejection is commented out. Its backup optimizer also has nonzero admission margins. Thus a planner-accepted trajectory and an exact 2/1 downstream hard check can disagree. The new mode exposes that distinction without modifying upstream behavior.

The separate `super_demo.py` preparation entry and `super_demo_control.py` controller accept `--motion-policy observe|strict`, defaulting to observation. Without `--execute`, they retain preparation/check-only behavior. Physical execution still requires measured canonical mission geometry, on-site supervision, current RC/manual POSCTL, readback-confirmed PX4 parameters and a separate explicit start request. Short-site synthetic profiles remain ineligible for physical execution. Existing PX4 speed limits and fallback settings are read and checked, not raised by this policy change. Independent tracking or continuity faults can still stop a mission even when derivative exceedances themselves are advisory.

## Venue-based position policy and QGC

The complete demonstration now follows SUPER's curved trajectories inside the configured operating box. The vehicle-inclusive venue check replaces the earlier straight-line ±10 cm lateral/height corridor. Initial-reference, measured tracking and landing home-offset metrics remain in the report without a separate 10 cm abort threshold. A navigation leg still has to arrive at the requested point before the next leg or six-second return hold can complete. Position errors do not count as successful arrival merely because they remain inside the box. The former extra 10 cm preflight route inset is removed; the requested route and aircraft envelope must directly fit the outer venue.

`post_takeoff_position_policy` records the active behavior independently of any retained local mission template. The schema's historical `axis_error_limit_m=0.1` now describes arrival detection for this controller, not a path tube or tracking abort. Base short-goal diagnostics retain their own strict offset policy. Fresh data, polynomial agreement, reference continuity, map/backup planning and whole-mission completion still apply.

Both demonstration entrypoints open or reuse QGroundControl before preparing the actual stack. QGC stays open after the rehearsal; RViz remains owned and closed by the rehearsal. Launch logs/status do not establish GUI rendering, MAVLink connection or flight readiness. See [automatic QGC startup](REBOOT_QUICK_START.md#automatic-qgroundcontrol-startup) for paths and headless behavior.

## Reports and failure interpretation

Each run creates `~/Downloads/super_demo_tests/<timestamp>/report.json`, stage logs, the captured alignment, a frozen mission file and a nested `mission_report.json`. Default rehearsal runs also retain the scene snapshot and private SUPER log. Upload the top-level report to retain the failed stage and the last 80 lines of a failed subprocess; the full files remain beside it. When the private SUPER log exists, the top-level report now also retains its last 100 lines and counts exploration successes, backup optimization/constraint failures, accepted goals and missing-odometry messages. These are evidence markers, not automatic root-cause assertions.

A complete result requires all eight ordered stages, two navigation goals, the return hold, fresh landing-position feedback inside the configured venue, and fresh landed/disarmed confirmation. Completing the outbound leg alone cannot pass. The report identifies actual localization and SUPER usage, the virtual backend, completed stages, reference peaks, time-aligned tracking extrema and any fault. Native publication in the private graph is not proof of MAVROS-to-PX4 transport on the aircraft.

The complete demonstration supervisor no longer imposes the previous eight-second first-reference deadline. For either leg, `WAIT_PLAN` holds the starting hover point until a verified committed trajectory and matching command become available. This applies to both rehearsal and physical demonstration entrypoints. Waiting time is measured and reported. The complete controller and rehearsal also have no total execution deadline or committed-trajectory duration cap; neither is replaced by a longer deadline. Arrival and confirmed landing determine stage completion. Feedback validity, polynomial matching, venue bounds, process health and operator stop still apply. This does not change SUPER's optimization constraints or guarantee that an eventual trajectory will satisfy the route checks. The separate historical one-goal diagnostic controller retains its own workflow and is not the complete demonstration entrypoint.

Planner participation and mission completion are reported independently: a failed run whose backend ledger confirms real SUPER use now retains `actual_SUPER_tested=true`, while `result=FAIL` and incomplete stages remain unchanged.

SUPER may commit an exploration prefix followed by a backup braking segment. Such an intermediate endpoint may change during continuous replanning; it is not final navigation completion. Every accepted command still requires fresh increasing stamps, matching committed polynomials, venue/body bounds, motion limits and continuity. Final holding requires the original requested navigation goal within 10 cm per axis. Feedback is compared to the sent reference at the feedback's original measurement timestamp, with no timestamp rewriting.

## Validation status

The complete synthetic ROS lifecycle has passed from the flat deployment directory, including return, six-second hold and confirmed landing. An injected return-command dropout produced `FAIL` rather than partial success. Source identity, snapshot capture, time-aligned tracking, rolling-stage handling, release checks and existing ground-manager tests have regression coverage. See [the software validation record](PROGRESS_2026-10-16.md).

The first onboard real-SUPER rehearsal passed real alignment, map capture and 30-second vision sampling, then failed at outbound planning: exploration optimization succeeded while backup optimization repeatedly failed. The current site was subsequently reported to have only approximately 1.5 m forward room, compared with the requested 4.5 m route. A later one-metre real-SUPER/virtual-vehicle observation sequence has been reported complete, with report archival pending. This does not validate the full 4.5 m route. Actual full-mission flight, dynamic obstacle response, physical tracking and surveyed clearance remain unverified. The existing separate 1 m / six-second aircraft hover and landing result does not validate this new mission. Future supervised execution uses a separate entrypoint and requires measured geometry, compatible readback-confirmed PX4 limits, fresh RC/manual POSCTL and an explicit start request; restarting a terminal cannot initiate flight.

## Waiting at intermediate rest endpoints

A SUPER stage may stop before the requested navigation goal. The complete mission accepts fresh stationary commands at that committed rest endpoint after the polynomial end time; the former 30 ms matching window is not a deadline for a stopped stage. If subsequent planner commands pause after a verified rest endpoint, the controller continues sending that held position with zero velocity and waits without a planning timeout. It does not mark the leg complete or send the return goal until the requested endpoint has been reached.

A later committed stage can resume continuously from the held point after an arbitrarily long rest. Original received timestamps are preserved. Only certified rest endpoints qualify: terminal position and derivatives must agree with the committed polynomial, and vehicle-inclusive venue and continuous-position checks remain active. Actual input validity and localization remain separate checks. Complete-mission callback gaps are recorded as timing advisories, and an elapsed reference stamp interval alone does not abort; polynomial position continuity still applies. Reports include `intermediate_rest_wait`; a moving-stream expiry records pending-command evidence in `command_stream_failure`. The historical one-goal diagnostic keeps its original matching and gap policy.

## Causal command-to-trajectory association

A newly published replan can have a future start time. A command is matched to a committed polynomial only after that polynomial has started at the command's original timestamp. Publishing a future plan does not immediately replace active continuity evidence. The former 30 ms pre-start matching tolerance could associate an ordinary moving command with a nearby future start pose, then clamp that future polynomial's pre-start displacement to zero. This could falsely reject normal motion near 1.5 m/s without any actual position jump.

The correction keeps the 15 mm continuity margin, source stamps, geometry and planner/PX4 limits unchanged. Future-start replans are accepted for storage and selected when active. Any later continuity failure saves original timestamps, positions and both selected polynomial snapshots in `reference_continuity_failure` for offline inspection.
