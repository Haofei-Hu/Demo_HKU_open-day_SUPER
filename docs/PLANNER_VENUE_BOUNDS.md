# One venue for rehearsal planning and control

The forward ground rehearsal now passes the controller's existing vehicle-inclusive operating box to SUPER before planning. A* search and exploration/backup corridor generation use that same box. The outer dimensions remain 5.8 m forward, 2.8 m wide and 2.8 m high for the canonical mission. There is no additional inset or straight-line tube. Curved obstacle-avoiding routes inside the box remain permitted.

This corrects the integration gap exposed by run `20261017_152536_523398`: the controller received a committed intermediate endpoint at forward/left/floor-height approximately `[1.957, -2.230, 1.662] m`. The right-side hull clearance was approximately `-1.030 m`, so control rejected the trajectory before starting its first horizontal leg. The planner previously did not receive the controller's venue constraints. Raising motion acceptance alone cannot correct this position-boundary mismatch.

## Fixed coordinate conversion

The private rehearsal uses the captured initial FC pose and heading, which its ideal virtual backend holds exactly before takeoff. The generator converts six vehicle-centre halfspaces into raw planner `world` coordinates using the same alignment, fixed raw goal quaternion, mounting lever arm and 0.085 m floor convention as the controller. It includes arbitrary captured world-frame tilt and heading. Both rear/front and left/right offsets use the existing mission geometry; the hull is deducted once.

For the canonical fixture placement, the accepted FC-centre ranges in forward/left/floor-height coordinates are `[-0.3, 5.1]`, `[-1.2, 1.2]` and `[0.085, 2.7] m`. These are derived from the existing 5.8 × 2.8 × 2.8 m box, takeoff placement and fixture hull, rather than new tuning thresholds. Measured geometry follows the same calculation. Fixture values remain synthetic.

Each rehearsal writes a separate planner YAML with `super_planner.venue_halfspaces`, a flattened six-by-four matrix with inequality `normal dot raw_position + offset <= 0`. It also saves `venue_reference.json` containing the transform, bounds, planes and dependency hashes. The nested mission report retains this under `planner_venue_bounds`. The source YAML, captured occupied cloud, robot radius, 2/1 optimization targets and 4/2 motion acceptance bounds are preserved.

The source patch:

- Adds the optional `DemoVenue` helper. A profile without halfspaces keeps the original behavior.
- Applies the halfspaces to normal and escape A* neighbor admission and rejects out-of-box search endpoints.
- Intersects all five successful corridor-generation branches with the same halfspaces: empty-cloud point/line boxes, both CIRI success paths and the fallback empty polytope. Out-of-box seeds cannot create a corridor.
- Supplies one immutable plane set to search and corridor generation from `SuperPlanner` initialization.

Obstacle processing and backup optimization remain active. The controller continues to validate actual references and feedback; the planner constraint does not replace those checks. If the captured map admits no route or backup inside the box, planning can continue waiting for a valid result without a mission deadline. Such a wait is not a successful mission.

## Complete the finite search instead of restarting at 100 ms

Run `20261018_144849_141272` confirmed both the 4/2 motion marker and the venue marker, but stayed in `WAIT_PLAN`. Inflated-map search exhausted its reachable component after 1,071 iterations. The probability-map fallback repeatedly hit upstream's 0.1-second per-search cutoff, returned `TIME_OUT` and restarted. Waiting longer at mission level did not accumulate a longer individual search. This establishes repeated truncation; it does not establish whether the probability map contains a path.

The current upgrade disables the original elapsed-time cutoffs only when explicit finite venue halfspaces are enabled: the point-to-point 0.1-second budget, its post-loop elapsed-time rejection and the escape-search 0.2-second budget. A search then ends on goal/horizon completion or exhaustion of the finite admissible grid. Profiles without venue halfspaces keep upstream behavior. Venue planes, cloud/occupancy, inflation, unknown-cell policy, robot radius, motion limits and backup optimization are unchanged.

The upgrade also logs exhausted searches with map type, node count, elapsed time, start/goal coordinates and goal occupancy on both maps. A completed `NO_PATH` remains a distinct outcome from `TIME_OUT`; removing a deadline cannot create a route through obstacles. The mission holds while awaiting a verified trajectory, and Ctrl+C remains available. `venue_reference.json` records `search_policy.elapsed_time_limit_s=null`.

## Install once, then use normal startup

Use the pinned installer in [reboot quick start](REBOOT_QUICK_START.md#install-or-update-the-matching-scripts). After finishing or interrupting a rehearsal, apply the backed-up source change on the grounded, disarmed aircraft computer:

```bash
python3 ~/Downloads/apply_super_venue_bounds.py
```

Then rebuild **both `rog_map` and `super_planner`** as shown in the quick start. ROG-Map is statically linked into SUPER, so rebuilding only the map library leaves the installed planner with the old query. If the motion-acceptance patch has already been applied and compiled, it is retained. New installations apply both patches and compile once. Normal reboots need no patch or build step.

The patch validates six existing source files before writing and adds or upgrades one helper header. Upgrades preserve the plane geometry and back up only changed files. It prints a `venue_bounds_backup_*` directory with originals and hashes. Repeating the command is idempotent; `--dry-run` shows the source diff. `--restore /absolute/path/to/venue_bounds_backup_...` restores only the files recorded in that backup and removes an added header only if that application created it, refusing to overwrite later changes. Rebuild after restoring and use a compatible runner version. The separate motion-acceptance patch is preserved by venue rollback.

Before actual-stack preparation, the new forward runner checks both the patched source and the marker in the installed `fsm_node`. This avoids spending a full alignment/map preparation cycle with an uncompiled planner. A normal bounded rehearsal prints:

```text
DEMO VENUE: same vehicle-inclusive box for A* search and all corridors; no extra inset
```

Its `planner_rehearsal.log` includes:

```text
VENUE_BOUNDS_V1: search and corridors use six world-frame FC-centre halfspaces
VENUE_SEARCH_COMPLETE_V1: finite venue search; no elapsed-time cutoff
```

## Scope and evidence

The original forward and short-site frozen-scene rehearsals use private domain 197. The live physical preparation entrypoint now has a separate capture path described below. The independent left experiment keeps its own pinned workflow. Neither capture infers measured laboratory boundaries from the scene.

The initial venue release passed 135 focused checks; its [validation record](../data/software_checks_2026-10-18/planner_venue_bounds.json) distinguishes software tests from onboard execution. Compiled C++ checks exercise the actual patched A* admission predicate and all three complete upstream corridor-generator functions, covering all five success branches with map/decomposition test doubles. They reject the recorded out-of-box location and admit a curved route inside the box. Geometry tests compare the generated planes directly with the controller across rotated/tilted frames, a nonzero mounting lever, all six faces and random points. Full upstream-file apply, repeat, coexistence with the motion patch and byte-exact restore are verified separately. The subsequent search-completion and cancellation checks are recorded [separately](../data/software_checks_2026-10-18/bounded_search_completion.json).

The follow-up release passed 140 focused checks plus a real ROS virtual-backend cancellation trial. The trial confirms exit 130, a finalized operator-cancel report and no remaining private-domain nodes; it does not run real SUPER or PX4. Source upgrade, idempotence and exact rollback were also checked against the pinned full upstream files.

A new complete onboard real-SUPER rehearsal remains required after deployment. These checks do not establish physical tracking, obstacle clearance in an unsurveyed room, or aircraft flight completion.

## From-rest trajectory acceptance and epoch

Run `20261018_160943_724717` advanced beyond A*: its probability-map searches succeeded and primary optimization completed. `generateExpTraj` then rejected each candidate with `Replan over time(...)` at about 0.48–0.53 seconds. The prior search-completion release did not remove this separate `replan_forward_dt` rejection.

`VENUE_REST_COMMIT_V1` exempts only a from-rest candidate with an explicit venue from that elapsed-time rejection. After primary optimization, its preliminary epoch is refreshed before backup construction. Once the primary/backup pair is accepted, both position/yaw epochs are shifted to the commit time, preserving the backup's relative start offset. This avoids treating time spent waiting for optimization as already executed motion. Corridor, goal-connected and known-free flags, polynomial shape/duration, geometry and motion constraints remain intact. Failed backup generation still does not commit a trajectory.

Moving replans continue to use their original splice-time checks: they predict a future state on an already executing trajectory, and shifting that result as if the vehicle were stationary would be incorrect. This update does not impose a mission waiting deadline and does not change `replan_forward_dt` or any motion/geometric parameter. The initial from-rest release covered the frozen-map rehearsal; the live preparation extension below reuses the same compiled constraints.

That from-rest upgrade used the same `apply_super_venue_bounds.py` and changed `astar.h` and `super_planner.cpp`. The current combined release additionally requires the ROG-Map rebuild described above. The runner requires the additional marker in the installed binary. A successful from-rest commit logs:

```text
VENUE_REST_COMMIT_V1: from-rest plan starts at commit; backup offset preserved
```

[Validation](../data/software_checks_2026-10-18/rest_plan_commit.json) includes 77 checks, compiled timestamp tests using the upstream `ExpTraj`/`BackupTraj` containers, and full-source fresh/upgrade/restore checks. This does not establish full compilation on the aircraft or completion of the recorded fence-crossing route; those remain deployment checks.

## Fence replay: virtual-height queries and candidate publication

The frozen map from `20261018_162423_709951` exposed two independent issues. ROG-Map `Config` already moves its virtual floor/ceiling inward for inflation. `InfMap::getGridType(Vec3f)` moved them inward a second time by `(1 + inflation_step) * inflation_resolution`. In this capture, the configured effective raw bounds were -1.3/2.7 m, but A* saw -0.7/2.1 m. A real ROG-Map empty-cell probe at raw Z=2.35 m returned occupied from that interface and free from `isOccupiedInflate`. This extra band can remove an upper route while the ordinary-map fallback tries an inadequate low gap. The uploaded backup seed ended only 0.141421 m from a recorded obstacle, below the unchanged 0.2 m radius.

`VIRTUAL_HEIGHT_QUERY_V1` removes the second height subtraction/addition from that query. It retains the already-inflated bounds, their closed-boundary rejection, obstacle counters, unknown-cell classification and local-map checks. It changes no YAML parameter or captured point. This API correction also applies to profiles without venue halfspaces; live venue injection is handled separately by the physical preparation extension below.

The corresponding `VIRTUAL_HEIGHT_CORRIDOR_V1` change also removes the second radius inset when a corridor crop is clipped by those already-inflated virtual faces. Ordinary crop faces keep their existing radius inset. This fixes the observed disagreement where A* admitted a raw Z=2.55 m seed but corridor construction capped it at 2.5 m despite a configured effective ceiling of 2.7 m.

With the query correction alone, a cloud replay crossed outbound but a return candidate placed its rest endpoint approximately 0.000989 m beyond the rear plane. Corridor constraints are soft optimizer penalties, so supplying planes alone cannot guarantee every optimized trajectory satisfies them. `VENUE_CANDIDATE_CHECK_V1` checks primary and backup polynomials against the same six planes before replacing the committed command. It converts each plane projection to Bernstein form over each piece, using subdivision to certify the entire interval, rather than checking only command samples. A violating, nonfinite or numerically uncertified candidate returns to replanning while the previous committed trajectory remains intact. Both from-rest and moving replans pass this check. No venue inset, goal replay, mission deadline or motion-limit adjustment is added. Profiles without an explicit venue retain their candidate behavior.

The same patch command upgrades an existing deployment with exact source guards and rollback. The runner requires the height-query, height-corridor, candidate-check and grid-anchor markers in the installed planner, in addition to the earlier venue, complete-search and rest-commit markers.

The [frozen-map validation record](../data/software_checks_2026-10-18/fence_replay.json) preserves the baseline and intermediate failed trials. The final actual-SUPER replay completed all five round trips, ten ordered goals, the six-second final hold, landing and disarmed confirmation: [mission report](../data/software_checks_2026-10-18/fence_replay_mission_report.json). It rejected 96 out-of-venue candidate plans before publication while continuing the mission. The captured scene, 2/1 targets, 4/2 acceptance, venue and hull dimensions were unchanged. Sampled peak reference speed was 1.544 m/s and acceleration 1.009 m/s²; a 2 m/s target is not a promise to reach that speed on a curved short route. Recorded virtual floor height reached approximately 2.30 m during crossing; 0.5 m is the takeoff height, not a fixed-altitude constraint.

Validation includes actual ROS Humble builds of both upstream packages, a repeated incremental build, 110 regression checks before the final corridor correction and 22 focused checks afterward. This is one completed five-round software replay with ideal virtual feedback, not PX4 SITL or physical flight. It does not guarantee repeated success, physical tracking or clearance of unobserved nets. The aircraft still needs the two-package rebuild and its own grounded rerun.

## Continue from a valid rest point near a rotated boundary

Run `20261018_170834_397877` exposed a separate grid-initialization defect. Its first committed rest endpoint was raw `[5.297986669, 0.161258683, 2.448742986]` m, corresponding to a valid FC floor height of 2.695339 m inside the unchanged 2.7 m ceiling. The nearest 0.2 m inflated-map grid centre was `[5.3, 0.1, 2.5]` m, whose transformed floor height was 2.743087 m. A* applied the venue predicate to that centre and reported the valid start as outside its local map. An `INIT_ERROR` then invalidated the navigation goal; after the previous trajectory finished, SUPER switched to `WAIT_GOAL` while the supervisor held the intermediate endpoint. The supervisor did not time out. The top-level report was cancelled by the operator; its nested mission report remained incomplete.

`VENUE_GRID_ANCHOR_V1` keeps the continuous point and venue unchanged. If its nearest centre is outside the box, normal and escape search choose the nearest admissible centre among the immediately adjacent grid cells. Candidates must remain inside the same venue and map, pass the selected occupancy/unknown and neighbour policy, and have a collision-checked straight connector from the continuous point. Convex venue planes keep that connector inside the box. The search path retains the continuous start/goal connectors through the existing planner path assembly, and corridor and continuous polynomial checks still apply. If no connector is valid, search reports no path rather than allowing an out-of-box cell. Profiles without explicit venue planes retain their original grid initialization.

For a bounded search, an initialization failure no longer invalidates a goal that is itself inside the venue. SUPER retains that goal for subsequent planning; this does not resend the goal or declare arrival. Out-of-venue goals and profiles without venue constraints keep their original invalidation behavior. The upgrade changes the search header, search implementation and planner goal handling, with guarded backup, idempotence and exact rollback.

The [branch replay record](../data/software_checks_2026-10-18/branch_grid_replay.json) documents an actual-SUPER baseline that reproduced the same intermediate stop and `WAIT_GOAL`, followed by two completed five-round fixed replays. The captured planner YAML was reconstructed byte-for-byte and verified against its recorded SHA256; the 8,792-point occupied cloud, alignment, mission, hull, venue and 2/1 targets with 4/2 acceptance were preserved. The final [mission report](../data/software_checks_2026-10-18/branch_grid_replay_mission_report.json) contains all ten accepted goals, twenty ordered completed stages, final six-second hold, landing and disarmed confirmation. Validation also includes 64 regression checks, compiled checks of the observed point and all six rotated faces, blocked/unknown connector cases, legacy goal invalidation, source upgrades and exact rollback. These remain frozen-map software rehearsals with ideal feedback, without real PX4 or physical flight.

The final binary also completed five round trips, final hover, landing and disarm using the previous 8,395-point fence capture (`20261018_162423_709951`), preserving that earlier successful workflow. Both final-scene replays retained all ten goals without a goal-invalidation event.


## Live physical preparation

`super_demo.py` now passes the measured mission to the manager as
`planner --planner-profile demo --venue-mission <measured.json>`. After the
existing session-map observation, `create_live_venue_profile.py` reads ten fresh
identical-stamp raw/aligned pairs and current grounded FC feedback. It records
one FC ground position and heading, the fixed localization origin and publisher
identities in `venue_reference.json`. No map snapshot or control output is used
by this capture. The existing single-source, fresh grounded localization gates
remain in effect.

The shared `venue_geometry.py` function constructs the same six planes for the
live entry and frozen rehearsal. The live profile adds only those planes to a
new session YAML, retaining live cloud input, raycasting, motion targets,
acceptance bounds, radius and virtual map settings. Actual mission measurements
are required even for check-only physical preparation; null or synthetic
rehearsal geometry is not supplied automatically.

The physical controller requires the resulting reference, checks its mission,
alignment and planner hashes, recomputes its planes, and reads the actual
`/fsm_node` `config_name` before READY. It checks that localization/FC source
identities and origin still match. READY binds its hover point and venue to the
captured FC ground position and heading rather than independently rebasing the
box. Both outbound and return goals use the record's fixed raw heading/lever
conversion. Existing ground displacement and heading checks remain; no new
margin or mission time limit is introduced.

This is a software integration, not an airborne validation. The [validation
record](../data/software_checks_2026-10-18/live_venue_preparation.json) separates
geometry tests, a real ROS synthetic-telemetry check-only test and the original
actual-SUPER frozen-scene regression. Physical measurements, current hardware
readbacks, RC/manual takeover and supervised tracking trials remain required.
