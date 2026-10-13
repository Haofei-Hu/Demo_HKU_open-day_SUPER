# Original SUPER rest-start behavior and patch rollback

The current ground workflow retains SUPER's original rest-start selection and permits references up to 10 cm behind the captured takeoff point. The first bounded-preview trial rejected forward/left/FC-height `[-0.0552, -0.0466, 0.1089]` under a zero-backward allowance. That point is within the revised envelope. Endpoint-hold, polynomial matching, continuity and feedback checks remain active. A subsequent same-height stationary aircraft test reached `END_HOLD` with initial offsets within 10 cm per axis; native flight tracking remains pending.

An exact-start source patch was prepared for this result, but is no longer required by the deployment workflow. If it was never applied on the aircraft, no source change or rebuild is needed. Keep the current 50 Hz localization and vision session running while restarting only ground planning/reference preview.

## Restore if the optional patch was applied

Install the current `apply_super_exact_start.py` and `bounded_reference_preview.py` into `~/Downloads`. With the aircraft stationary and disarmed:

```bash
python3 ~/Downloads/super_session.py planner-stop &&
python3 ~/Downloads/apply_super_exact_start.py --restore-applied
```

`Original rest-start source present; no files written` means the original source is already present. Rebuild only if a patched executable was previously built.

`Restored:` means the helper found a matching backup in `~/Downloads/super_exact_start_backup_*` and restored the original source. It checks the source path, pinned revision and current patched checksum, then verifies the original backup checksum. A missing backup, unrelated source edits or a damaged backup prevent restoration. It does not reset the Git checkout or overwrite later edits.

After restoration, rebuild only the planning package:

```bash
(
set -e
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/super_ws/install/setup.zsh
cd ~/super_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --packages-select super_planner --executor sequential
)
```

If the backup is outside Downloads, specify its parent using `--backup-root`, or restore a particular backup directory:

```text
python3 ~/Downloads/apply_super_exact_start.py --restore BACKUP_DIRECTORY
```

Source restoration does not replace an already built executable. Confirm a successful rebuild before restarting a previously patched planner.

## Resume the bounded ground preview

```bash
python3 ~/Downloads/super_session.py planner --planner-profile low-speed-3d
python3 ~/Downloads/super_session.py reference-preview
```

Follow the [bounded preview workflow](BOUNDED_REFERENCE_PREVIEW.md): wait for `WAIT_TRAJECTORY`, confirm a nonempty occupancy map, send one goal and inspect `TRACK` → `END_HOLD`. The preview allows forward −0.10–5 m, lateral ±2 m and FC-centre floor height −0.10–2 m. The initial reference must also remain within 0.10 m of the current measured pose on each forward/left/up axis separately. The negative floor margin is a disarmed-preview allowance and does not define a flight target. References are neither translated nor clipped. No native flight setpoints are generated.

## Archived exact-start experiment

In pinned SUPER `2ad3419c127a617c6d7df6925e81a14175a9c096`, `PlanFromRest` calls `getNearestCellNot(OCCUPIED, robot_state_.p, ...)`. ROG-Map returns a cell-centre position even when the current cell is already nonoccupied. At the deployed 0.1 m resolution, this can introduce centimetres of displacement. This mechanism is consistent with the reported trial; start-cell classification and additional pose/lever-arm contributions have not been physically isolated.

The archived [source patch](../patches/super_exact_rest_start.patch) replaces that selection with the measured position for known-free/unknown cells and rejects invalid/occupied starts. Its helper's default invocation applies that experiment; it is not part of the current startup workflow. The helper and patch remain available for review and checksum-guarded rollback.

Four offline checks cover backup/idempotence/rollback, guarded automatic backup selection, revision/block/concurrent-edit rejection and compiled execution of the exact replacement block. The compilation fixture uses mocked map/logger interfaces and optionally real Eigen; it is not a full SUPER package build. The revised backward allowance and endpoint hold are separately exercised by offline and isolated ROS fixtures.

```bash
python3 tests/test_super_exact_start.py
python3 tests/test_bounded_reference.py
```
