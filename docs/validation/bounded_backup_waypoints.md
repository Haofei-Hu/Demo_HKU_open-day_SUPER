# Bounded backup waypoint verification

## Reproduction

The uploaded `super_boundary_debug_20261020_134725.zip` contains the captured
scene for the rehearsal `20261020_133636_673543`. The mission uses a 2 m lateral
width, a 2 m outer ceiling, a 1.5 m hover target, and five 4.5 m round trips.
Its frozen occupied cloud has 7,323 points.

Replaying the scene with real SUPER reproduced `WAIT_PLAN`. Exploration and
backup optimization returned success, but the continuous venue check rejected
the backup polynomial. Diagnostic projection of a rejected final backup piece
onto the ceiling plane showed a **3.235 mm** violation; another candidate crossed
the lateral plane by **2.854 mm**. Their exploration trajectories passed the
same check. Evaluating polynomial extrema independently confirmed these were
actual violations, not a display issue or a Bernstein-certification false alarm.

The original backup optimizer used unconstrained Cartesian waypoint variables
with finite corridor penalties. Success under those penalties does not imply
membership in the closed venue. Repeating the same optimization was not a
reliable way to satisfy the stricter publication check.

## Correction and scope

For venue-enabled planning, the backup optimizer now uses SUPER's existing
convex-vertex waypoint representation (`pos_constraint_type=2`) on a local
configuration copy. The terminal stopping point and backup knots are
parameterized inside their existing corridor. The source YAML and its motion
parameters remain unchanged.

The complete polynomial checks remain necessary and active because an
interpolating curve can leave a convex set even when its knots are inside.
Obstacle processing, primary optimization, venue dimensions, hull deduction,
map heights, 2/1 optimization targets, 4/2 acceptance bounds and mission waiting
policies are preserved. Legacy profiles without venue planes retain their
configured representation.

## Validation

The clean release build, without diagnostic instrumentation and with the normal
source/executable compatibility check, completed both replay cases:

| Captured scene | Result | Completed motion |
|---|---|---|
| Uploaded 2 m width / 2 m ceiling, 1.5 m hover | SUCCESS | Five round trips, 20 stages, 6.05 s final hold, landed and disarmed |
| Earlier 2.8 m width / 2.8 m ceiling, 0.5 m hover | SUCCESS | Five round trips, 20 stages, final hold, landed and disarmed |

The uploaded scene payload and mission were preserved; only dependency paths
were remapped to the cloud workspace. Regenerated venue-plane coefficients
differed by at most `1.11e-16` from the uploaded record.
All 67 related tests passed. See the [machine-readable verification record](bounded_backup_waypoints.json).

Focused checks cover guarded upgrades from the preceding nine-marker release,
idempotence, byte-exact rollback, refusal of partial edits, detection of an old
installed executable, preservation of legacy optimizer configuration, unchanged
motion targets, and continuous rejection of between-sample boundary violations.

These are real-SUPER frozen-map rehearsals with ideal virtual feedback. They do
not use PX4 dynamics, operate an aircraft, validate physical tracking accuracy,
or guarantee a path exists in every subsequent map.
