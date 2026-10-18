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

## Install once, then use normal startup

Use the pinned installer in [reboot quick start](REBOOT_QUICK_START.md#install-or-update-the-matching-scripts). After finishing or interrupting a rehearsal, apply the backed-up source change on the grounded, disarmed aircraft computer:

```bash
python3 ~/Downloads/apply_super_venue_bounds.py
```

Then rebuild `super_planner` as shown in the quick start. Only that package needs rebuilding; ROG-Map is unchanged. If the motion-acceptance patch has already been applied and compiled, it is retained. New installations apply both patches and compile once. Normal reboots need no patch or build step.

The venue patch validates the five existing source files before writing and adds one helper header. It prints a `venue_bounds_backup_*` directory with originals and hashes. Repeating the command is idempotent; `--dry-run` shows the source diff. `--restore /absolute/path/to/venue_bounds_backup_...` restores only those edits and removes its added header, refusing to overwrite later changes. Rebuild after restoring and use a compatible runner version. The separate motion-acceptance patch is preserved by venue rollback.

Before actual-stack preparation, the new forward runner checks both the patched source and the marker in the installed `fsm_node`. This avoids spending a full alignment/map preparation cycle with an uncompiled planner. A normal bounded rehearsal prints:

```text
DEMO VENUE: same vehicle-inclusive box for A* search and all corridors; no extra inset
```

Its `planner_rehearsal.log` includes:

```text
VENUE_BOUNDS_V1: search and corridors use six world-frame FC-centre halfspaces
```

## Scope and evidence

This automatic boundary injection applies to the original forward and short-site **frozen-scene ground rehearsals in private domain 197**. The independent left experiment keeps its own pinned workflow. Physical preparation does not infer laboratory boundaries from a frozen scene; its existing surveyed-venue/controller checks remain. Automatic planner-side injection for a physical flight is not established by this release.

All 135 focused checks passed. The [validation record](../data/software_checks_2026-10-18/planner_venue_bounds.json) distinguishes software tests from onboard execution. Compiled C++ checks exercise the actual patched A* admission predicate and all three complete upstream corridor-generator functions, covering all five success branches with map/decomposition test doubles. They reject the recorded out-of-box location and admit a curved route inside the box. Geometry tests compare the generated planes directly with the controller across rotated/tilted frames, a nonzero mounting lever, all six faces and random points. Full upstream-file apply, repeat, coexistence with the motion patch and byte-exact restore are verified separately.

A new complete onboard real-SUPER rehearsal remains required after deployment. These checks do not establish physical tracking, obstacle clearance in an unsurveyed room, or aircraft flight completion.
