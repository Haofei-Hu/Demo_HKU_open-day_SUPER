# Demonstration motion targets and acceptance bounds

The demonstration continues to optimize for **2 m/s speed and 1 m/s² acceleration**. Its separate accepted-trajectory bounds are now **4 m/s and 2 m/s²**, as requested. These bounds allow optimizer overshoot without raising the optimization targets.

| Layer | Speed | Acceleration | Behavior |
| --- | --- | --- | --- |
| SUPER optimization target | 2 m/s | 1 m/s² | Original magnitude bounds and penalty weights |
| SUPER candidate acceptance | 4 m/s | 2 m/s² | Reject the candidate and keep planning above either bound |
| Demonstration reference, `strict` | 4 m/s | 2 m/s² | Reject an advertised derivative above either bound |
| Demonstration reference, `observe` | Recorded | Recorded | Derivative exceedances alone remain record-only |
| Existing warning advisories | 2.4 m/s | 1.5 m/s² | Warning only, including with `strict` |

`observe` does not bypass the planner's own candidate checks. Position, collision/corridor, body-rate, thrust, source validity, freshness and polynomial continuity checks remain active. Backup planning remains enabled. This change adds no planning or mission execution deadline and writes no PX4 parameters. The requested route and ground/physical heights remain unchanged.

## Why SUPER needs a source change

At the recorded upstream revision `2ad3419c127a617c6d7df6925e81a14175a9c096`, the backup optimizer's `Acc` log is the largest sampled squared-acceleration violation, not the acceleration itself or its weighted cost. With a 1 m/s² target and `penna_margin=0.05`, the original check permits only about `sqrt(1.05) = 1.025 m/s²`. A logged violation near 0.149 corresponds to about 1.072 m/s² and can therefore reject a backup trajectory. The backup's separate full-polynomial check also uses a multiple of the same optimization target.

Changing only the Python failure threshold would leave these C++ rejection paths active. The guarded source patch adds optional YAML keys `traj_opt.boundary.acceptance_max_vel` and `acceptance_max_acc`, and separates sampled and full-polynomial acceptance in both exploration and backup optimization. Optimization still uses `max_vel=2` and `max_acc=1`. Profiles without the new keys retain the original checks.

The demo manager writes a separate `mid360_demo_acceptance_<hash>.yaml` with only the two additional keys, then derives the session map profile as before. It preserves the original YAML and checks that the installed `fsm_node` contains the compiled patch marker before selecting the new profile. Nominal-target exceedances and acceptance-bound exceedances have separate counters in reports.

## One-time installation and rebuild

First finish or interrupt the current ground rehearsal. Install the matching release using [the pinned installer](REBOOT_QUICK_START.md#install-or-update-the-matching-scripts), then run this once on the grounded, disarmed aircraft computer:

```bash
(
set -e
python3 ~/Downloads/apply_super_motion_acceptance.py
bash --noprofile --norc -c '
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/super_ws/install/setup.bash"
cd "$HOME/super_ws"
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --packages-select super_planner --executor sequential
'
)
```

The patcher validates all three source files before writing, saves originals and hashes in the printed `motion_acceptance_backup_*` directory, and is idempotent. `--dry-run` prints its diff without editing. Unsupported or partly modified contexts are reported rather than overwritten. It supports the recorded SUPER source layout; an incompatible source tree needs inspection before patching.

To restore these source edits later, use `python3 ~/Downloads/apply_super_motion_acceptance.py --restore /absolute/path/to/motion_acceptance_backup_...` and rebuild again. Restoration refuses to overwrite later source edits. Do not restore while running a rehearsal that uses the new acceptance profile.

A normal reboot after this one-time update needs neither the patch command nor a rebuild. Run the original forward rehearsal:

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics --fixture-geometry \
  --motion-policy observe
```

Expected planner-stage output includes `DEMO MOTION: target=2 m/s, 1 m/s^2; acceptance=4 m/s, 2 m/s^2`. The actual planner log includes `MOTION_ACCEPTANCE_V1` with `target=2/1 accept=4/2` for its optimization configurations. The mission report records the same values under `motion_acceptance`, separately from the warning advisories and nominal peaks. `RESULT: SUCCESS` still requires the full rehearsal, including all five returns, final hover and landing.

## Validation scope and planner cleanup

The [validation record](../data/software_checks_2026-10-17/motion_acceptance_4_2.json) covers 125 passing software checks. Compiled tests exercise the actual patched configuration header and extracted upstream acceptance guards, including the observed 1.072 m/s² case, exact-bound acceptance and above-bound rejection. Applying, repeating and restoring the patch against all three complete upstream source files preserves the original bytes on restoration. ROS tests check controller initialization, report values, references, mission sequencing, viewer startup and installer compatibility. This is not a complete onboard SUPER build or a newly completed aircraft rehearsal; those need the deployment commands above and a saved onboard result.

The same release fixes owned virtual-planner cleanup. Exiting the `ros2 run` wrapper no longer skips cleanup of its surviving `fsm_node` child, and planner cleanup occurs before ROS/viewer teardown. Real process-group tests confirm that an unrelated process is untouched. Shutdown grace periods apply only after exit/interruption; they are not mission deadlines. This fix does not terminate unknown nodes or retroactively remove old orphaned planners. If an existing private-domain node blocks startup, identify and stop that old rehearsal separately.
