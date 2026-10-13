# Exact SUPER rest-start position

The first bounded-reference aircraft trial rejected forward/left/FC-height `[-0.0552, -0.0466, 0.1089]`: the initial reference was approximately 5.5 cm behind the captured takeoff point. Lateral displacement and height were within the preview envelope. Output stayed blocked.

In pinned SUPER `2ad3419c127a617c6d7df6925e81a14175a9c096`, `PlanFromRest` calls `getNearestCellNot(OCCUPIED, robot_state_.p, ...)` and uses its result as the optimizer's initial position. ROG-Map converts the starting position to a grid index and returns a cell-centre position, even when the current cell already satisfies the nonoccupied condition. The deployed 0.1 m resolution can therefore introduce centimetres of displacement. This mechanism is consistent with the observed trial; actual start-cell classification and any additional pose/lever-arm error still require physical readback.

The [reviewed source patch](../patches/super_exact_rest_start.patch) retains the exact measured position for known-free or unknown start cells. It rejects nonfinite positions and other classifications, including occupied/out-of-map positions, rather than relocating the start. Path search, corridor generation, goal relocation, map inflation, collision radius, optimizer limits and the bounded-preview envelope are preserved. A start that cannot satisfy remaining corridor/clearance checks must be diagnosed rather than forced through the map.

## Apply while ground planning is stopped

Install the current `apply_super_exact_start.py` and `bounded_reference_preview.py` from the deployment catalog. The latter adds current aligned FC-centre coordinates to fault reports; it does not change acceptance limits. On onboard Ubuntu, keep the aircraft stationary and disarmed:

```bash
python3 ~/Downloads/super_session.py planner-stop &&
python3 ~/Downloads/apply_super_exact_start.py
```

The helper checks the pinned Git revision and exact source block before writing. It preserves unrelated source bytes, saves the original file and before/after SHA256 values in an external backup directory, refuses a source that changes during backup, and is idempotent. It makes no ROS calls and launches no processes.

Rebuild only the planning package using existing installed dependencies:

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

Localization, MAVROS and continuous vision stay running. The patched source has no effect until the package builds successfully and a new planner process uses the rebuilt executable. A full-package build on the aircraft remains required; the cloud's independent guard compilation is not a substitute.

## Resample

After a successful build:

```bash
python3 ~/Downloads/super_session.py planner --planner-profile low-speed-reserved
python3 ~/Downloads/super_session.py reference-preview
```

Follow the [bounded preview workflow](BOUNDED_REFERENCE_PREVIEW.md): wait for `WAIT_TRAJECTORY`, confirm a nonempty occupancy map, send one goal and inspect status. Look for `Exact odometry start retained` in the current session's `planner.log` to verify the rebuilt code was exercised. If `Exact start rejected` is logged, inspect the start-cell classification and mapping/clearance conditions. If the preview still faults, compare commanded coordinates with the newly reported measured coordinates; initial geometry or accumulated pose movement must be reviewed separately. No frame rebasing, coordinate clipping, backwards allowance or extra derivative tolerance is introduced by this fix.

The final demonstration candidate remains 2 m/s with 1 m/s² acceleration. This remains ground planning with debug output; native trajectory execution is pending.

## Restore safely

Keep the printed backup directory. With ground planning stopped, replace the placeholder with that directory:

```text
python3 ~/Downloads/apply_super_exact_start.py --restore BACKUP_DIRECTORY
```

Restore verifies the saved original hash and refuses to overwrite source edits made after patching. Rebuild `super_planner` before using the restored implementation. Restoring the old cell-centre behavior does not make its displaced reference acceptable to the bounded preview.

## Verification

Three unit checks cover backup/idempotence/rollback, revision/block/concurrent-edit rejection and compiled execution of the exact C++ replacement block. The compiled fixture preserves a non-cell-centre position for known-free/unknown classifications, rejects invalid map starts and rejects NaN before querying the map. It uses mocked map/logger interfaces and optionally real Eigen vectors; it does not build the complete planner. The patch artifact was also applied to the pinned full source and compared byte-for-byte with the helper's replacement. The updated fault-reporting node passed its isolated ROS fixture. Aircraft compilation and physical resampling remain pending.

```bash
python3 tests/test_super_exact_start.py
# With Eigen installed, exercise the real vector API:
EXACT_START_EIGEN_INCLUDE=/usr/include/eigen3 python3 tests/test_super_exact_start.py
```
