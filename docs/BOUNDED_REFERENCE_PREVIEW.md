# Bounded ground reference preview

The 50 Hz localization chain and low-speed advertised-derivative checks have passed a ground sample. A stationary trajectory restart still moved the reference by 0.424264 m. `bounded_reference_preview.py` adds a separate one-goal debug stage to inspect endpoint hold and reject discontinuous references before native flight integration.

It publishes only `/super_debug/bounded_control_target` (`geometry_msgs/PoseStamped`) and `/super_debug/bounded_reference_status` (`std_msgs/String`). It creates no MAVROS output, goal publisher, mode client, arming client, or flight-start service. The existing coordinate preview and continuous vision relay remain separate.

## Start in the managed session

Install the updated `super_session.py`, `create_super_3d_goal_profile.py`, `send_control_preview_goal.py`, `control_bridge_preview.py`, and `bounded_reference_preview.py` into the active Downloads directory. Keep the aircraft disarmed, stationary on the actual takeoff floor, with valid local position/velocity and the current vision session running. A fresh grounded AUTO.LOITER state is sufficient for this diagnostic.

Start a fresh ground planner without sending a goal:

```bash
python3 ~/Downloads/super_session.py planner-stop
python3 ~/Downloads/super_session.py planner --planner-profile low-speed-3d
python3 ~/Downloads/super_session.py reference-preview
```

The `low-speed-3d` profile keeps the reserved 0.25 m/s / 0.45 m/s² limits and disables SUPER's fixed goal-Z override. The diagnostic goal requests +0.30 m along planner X and retains current raw odometry Z. In pinned SUPER, `click_height=-6` is a configuration sentinel because override occurs only above −5; it is not a target height. Goal snapping to a nonoccupied inflated cell still occurs, so inspect requested and accepted coordinates. This prevents a stale absolute planner Z from being treated as the current floor/flight height. It does not change rest-start selection.

This adds one managed tmux window, `reference_preview`, without restarting localization or rewriting alignment. The manager validates fresh grounded localization and the generated `PolynomialTrajectory` message. The node also requires fresh state, landed-state and estimator telemetry, ten matched raw/aligned pose pairs, unchanged alignment contents, and one fixed discovered publisher per input topic.

Inspect readiness from the same command terminal:

```bash
source /opt/ros/humble/setup.zsh
timeout 8s ros2 topic echo /super_debug/bounded_reference_status std_msgs/msg/String --no-daemon \
  --qos-reliability best_effort --qos-durability volatile --full-length --field data
```

Wait for `WAIT_TRAJECTORY`, indicating the ground frame is locked. Confirm a nonempty occupancy map as in the session runbook, then send one diagnostic goal:

```bash
python3 ~/Downloads/super_session.py goal
timeout 8s ros2 topic echo /super_debug/bounded_reference_status std_msgs/msg/String --no-daemon \
  --qos-reliability best_effort --qos-durability volatile --full-length --field data
```

Expected progression is `WAIT_TRAJECTORY` → `TRACK` → `END_HOLD`. A fault prints its first reason, phase before fault, and accepted sample count, then suppresses output. `accepted=0` identifies a fault before any reference was accepted; a positive count identifies a failure after tracking began. Inspect the reason before changing configuration or sending another goal; boundary margins should reflect the intended test area. To start a different goal, stop/restart the ground planning and preview processes after reviewing current state. `planner-stop` stops reference preview first, then coordinate preview and planner, while preserving localization and vision. Hover startup excludes all three planning windows.

If a ROS CLI command fails with `xmlrpc.client.Fault` containing `!rclpy.ok()`, the traceback is from the CLI daemon's ROS context. The commands above bypass that daemon; they do not restart localization or establish that the preview process is healthy. Inspect `reference_preview.log` in the current session log directory alongside the status topic. A normal `timeout` expiry after the requested observation interval is separate from a node traceback.

The first aircraft trial reached `WAIT_TRAJECTORY` and rejected an initial reference at forward/left/FC-height `[-0.0552, -0.0466, 0.1089]`. A 10 cm backward allowance is now included in the ground-preview envelope, so this point is within the configured bounds. Original SUPER rest-start selection is retained; see the [rollback guide](EXACT_PLANNER_START.md) if the optional exact-start patch was applied. The preview does not translate or clip the trajectory. New fault reports include current measured coordinates in the locked frame to distinguish reference displacement from position drift.

## Fixed coordinates and envelope

The frame uses the matched aligned FC-centre position and yaw at startup. Yaw defines forward; positive lateral displacement is left. Floor height is the FC-centre Z displacement from that position plus the measured 0.085 m resting FC height. The ground diagnostic envelope is forward −0.10–5 m, lateral ±2 m, and FC-centre floor height −0.10–2 m. The frame remains fixed for the process.

These are preview-frame point bounds. They do not establish rotor clearance, obstacle-free execution or agreement with an independently surveyed physical axis. A flight bridge still needs explicit validation of the preview-to-EKF frame relationship and vehicle envelope. SUPER's grid/free-space relocation can also move the initial reference or accepted goal. An initial reference outside the envelope or more than 0.10 m from the current measured position on any captured forward/left/up axis is rejected. Each axis is checked separately, not by total three-dimensional distance. The current measured position comes from fresh aligned feedback; the captured yaw and floor origin remain fixed. The status reports the accepted initial axis offsets. References are not shifted or clipped. The negative floor margin is for this disarmed diagnostic only; a flight interface must use the actual flight height and validate clearance. This startup guard does not measure full-flight tracking error or promise 10 cm flight accuracy.

## Endpoint evidence and continuity

The implementation is pinned to SUPER `2ad3419c127a617c6d7df6925e81a14175a9c096`. Its ROS2 `PolynomialTrajectory` uses `float64 start_wt_pos`, degree-seven coefficients in descending power order, and per-piece durations in seconds. `POSITION_TRAJ` messages are committed trajectories; coefficient-free `HEART_BEAT` messages are not trajectories. Emergency trajectories latch a fault. This first diagnostic supports regular position commands only; backup trajectories require separate handling.

Each accepted command must match a cached committed trajectory in time, raw position, velocity and acceleration. The model checks advertised three-dimensional speed ≤0.3 m/s and acceleration ≤0.5 m/s², converted point bounds, increasing timestamps, and a position-step guard of `0.015 m + 0.3 m/s × stamp interval`. A gap over 0.25 seconds or a larger step faults while tracking. The step guard includes a diagnostic allowance and is not a hard converted-reference speed bound.

`END_HOLD` requires the planned end time to have elapsed, raw position within 0.015 m of the polynomial endpoint, and low command and polynomial terminal derivatives (velocity ≤0.03 m/s, acceleration ≤0.10 m/s²). Zero velocity or `trajectory_flag=1` alone cannot establish completion. The held pose is the last accepted converted pose in that verified endpoint neighborhood. Subsequent stationary restarts do not replace it; changed goals require an explicit new diagnostic session.

Tracking output preserves the source stamp. Held debug poses use the current stamp at 50 Hz, while status explicitly remains `END_HOLD`. They are generated references, not newly measured localization. Holding still requires fresh grounded estimator/state and matched localization, unchanged alignment, and unchanged input publisher identities. Publisher identities are polled every 0.2 seconds and are graph evidence rather than per-message provenance. Arming, geometry/source changes, invalid references and tracking faults latch output off.

This stage does not smooth or reconcile a moving discontinuity, implement autonomous takeoff/landing, or claim collision-safe tracking. Native trajectory execution, exact terminal transition dynamics, actual loss responses, RC takeover and staged speed increases remain separate work. The final demonstration candidate remains 2 m/s with 1 m/s² acceleration.

## Verification

Nine offline checks exercise fixed yaw/floor bounds, pinned polynomial evaluation, malformed trajectories, completion evidence, endpoint hold across restarts, moving jumps, timestamp order, advertised caps, and changed goals. An isolated ROS Humble fixture exercises committed-trajectory matching, held output, restart suppression, stale estimator/localization rejection and alignment/source/arming latches. Publisher inspection verifies that the preview owns no MAVROS or goal output. Lifecycle and real tmux fixtures cover guarded launch, hover exclusion and stopping the preview before planner shutdown. These software checks are separate from the observed stationary-ground trial. That trial accepted 227 references, reached `END_HOLD` with initial offsets `[0.0013, 0.0022, −0.0537] m`, and held one endpoint across the shown 34-second log interval while suppressing subsequent restart samples. See the [recorded result](../data/observed_2026-10-13/bounded_reference_ground.json). Native flight execution and airborne tracking remain unverified.

```bash
python3 tests/test_bounded_reference.py
python3 tests/test_super_session.py
# In an isolated Humble environment with the pinned MAVROS/SUPER messages:
python3 tests/test_bounded_reference_ros.py
```
