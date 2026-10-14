# One-goal SUPER handover

The disarmed SUPER → MAVROS → PX4 protocol path is verified on the aircraft. `super_flight_control.py` adds a separate one-goal controller prototype. Its default mode audits a ground trajectory and emits debug poses only. Aircraft handover auditing and planner-driven flight remain unverified; isolated ROS fixtures use simulated telemetry and MAVROS command services, not a PX4 dynamics simulation.

## Ground audit

Keep the existing fixed-geometry localization/vision session and the reserved `low-speed-3d` planner running. The manager's `handover-check` launches the default check-only controller and stops a managed wire sender if present. It refuses other native writers, another hover/reference/handover window, or an older standalone hover artifact without the extension hooks. Localization and planning stay running.

The flat Downloads installation needs the current `super_session.py`, `super_flight_control.py`, `first_hover_terminal.py`, `bounded_reference_preview.py` and the existing coordinate preview. No workspace rebuild is required. Updating the standalone hover artifact does not change its existing CLI defaults; the managed hover command still supplies the 1 m / 6 s arguments explicitly. Back up the prior scripts before replacement.

```bash
python3 ~/Downloads/super_session.py handover-check

source /opt/ros/humble/setup.zsh
timeout 12s ros2 topic echo \
  /super_debug/super_flight_status std_msgs/msg/String \
  --no-daemon --qos-reliability best_effort \
  --qos-durability volatile --full-length --field data
```

Wait for `READY`. The checker requires fresh, valid FC/external poses and estimator status, confirmed fusion/low-speed/LAND-fallback parameters and LOCAL_NED, ten identical-stamp raw/aligned geometry pairs, fixed publisher identities, unchanged geometry/configuration files and a three-second stationary capture. Check-only operation accepts disarmed POSCTL or AUTO.LOITER without RC. It creates no native setpoint publisher and does not call mode or arming services.

After capture, the handover controller permits estimated ground offsets up to 0.10 m on each captured forward/left/up axis. It does not use the original hover controller's 0.05 m total-distance ground check. Exceeding the margin latches `STOPPED` and reports the signed axis offsets; estimated movement alone does not prove physical aircraft movement. A stopped checker must be stopped through the manager before a fresh audit can launch. Repeating the download does not reset the running checker.

Request one forward goal at the captured ground height:

```bash
ros2 service call /super_flight/audit std_srvs/srv/Trigger '{}'

timeout 15s ros2 topic echo \
  /super_debug/super_flight_status std_msgs/msg/String \
  --no-daemon --qos-reliability best_effort \
  --qos-durability volatile --full-length --field data
```

The requested movement is 0.30 m along captured FC forward, retaining heading; it is not planner-world X. The goal uses the inverse session world/mount/terminal lever conversion. Expected phases are `AUDIT_WAIT` → `AUDIT_TRACK` → `AUDIT_END`. A matched polynomial and verified terminal sample are required before endpoint hold. The debug target is `/super_debug/super_flight_target`; no native commands are emitted. Do not issue an additional manager/RViz goal during this one-goal session. Another `/goal_pose` publisher blocks goal initiation; an endpoint/source/configuration change or discontinuity latches `STOPPED` with its reason.

During goal handover, SUPER may continue emitting freshly stamped references for its previous goal until the new trajectory starts. The auditor waits for the committed polynomial and discards queued commands stamped before its start, then requires polynomial position/velocity/acceleration matching before checking the command corridor. The status includes `preplan commands discarded`. Commands after the new start that cannot be matched remain subject to the freshness timeout; a matched reference outside the 0.10 m lateral margin still stops the audit. This does not reconcile a jump after tracking has started.

Stop the auditor without restarting localization:

```bash
python3 ~/Downloads/super_session.py handover-stop
```

`planner-stop` and full `stop` also stop the auditor. A fault or completed audit cannot silently start another goal; stop the auditor and create a fresh grounded check session for a new trial.

## Experimental flight lifecycle

The controller's explicit execution mode reuses the verified hover lifecycle through extension hooks. The original hover controller retains its own services, default RC gates and reference/envelope behavior. The manager currently exposes only the new ground-check lifecycle.

The prototype flight sequence is:

```mermaid
flowchart LR
    READY -->|explicit start and RC/POSCTL gates| PRESTREAM
    PRESTREAM --> OFFBOARD
    OFFBOARD --> ARM
    ARM --> RISE[1 m floor-referenced rise]
    RISE --> HOVER[2 s settled hover]
    HOVER --> GOAL[One 0.30 m forward SUPER goal]
    GOAL --> TRACK[Verified polynomial and command tracking]
    TRACK --> HOLD[6 s verified endpoint hold]
    HOLD --> LAND[AUTO.LAND]
    LAND --> DONE[Landed and disarmed confirmation]
```

Execution still requires fresh RC, connected/disarmed POSCTL with manual input and on-ground confirmation, an explicit start, one native position sender and a MAVROS subscriber. Starting the node alone never requests a mode change or arming. A goal is sent only after reaching the hover height and holding it for two seconds. No mode/arm/goal retry or second flight is provided.

This initial trial uses the 0.25 m/s, 0.45 m/s² planner profile and existing 0.5 m/s PX4 velocity caps, not the final 2 m/s demonstration profile. It supports regular trajectories only. The position/yaw interface runs at 20 Hz. Incoming command/polynomial matching and freshness are preserved; the endpoint hold creates new commands rather than retiming measurements. The existing discontinuity checks reject a source gap over 0.25 s or a converted step over `0.015 + 0.3*dt` metres. A missing terminal sample is not repaired by forwarding a planner restart.

Initial reference offset, map-relocated endpoint error and estimated tracking error are limited to 0.10 m per forward/left/up axis. Both commanded and estimated measured positions must stay inside the initial straight-line corridor: forward `-0.10..0.40 m`, lateral `±0.10 m`, and hover-height deviation `±0.10 m`. This prevents separate reference and tracking offsets from stacking into a larger transverse/height departure. It does not establish independent physical accuracy. The wider captured laboratory frame remains forward `-0.10..5 m`, left `±2 m` and FC floor height `-0.10..2 m`.

The planner must provide a verified first reference within eight seconds. Committed trajectories are limited to 20 seconds, and the total OFFBOARD flight budget is 45 seconds. A 0.2-second control-timer interruption, stale feedback, invalid estimate, changed geometry/source, unacceptable tracking/reference, emergency trajectory or command loss stops the stream and requests LAND only if the controller still owns an armed OFFBOARD vehicle. Verified PX4 OFFBOARD-loss LAND remains the fallback if the request cannot progress. Pilot mode takeover has priority and is not overwritten. No in-flight disarm is requested; completion requires FC landed/disarmed telemetry.

The status topic includes maximum control callback runtime and timer gap. A test run triggered the retained 0.2-second watchdog; subsequent representative mixed-rate runs passed. That event verifies fail-stop behavior rather than proving a permanent scheduling correction. Small matrix work defaults to one BLAS worker when the process has no existing setting. Onboard timing, actual tracking, endpoint behavior, rescue handling, surveyed map geometry and full-session localization stability need physical validation before increasing the trial scope.

## Software validation

- The original hover ROS regression passed its complete lifecycle, low-rate feedback, height/hold profiles, rejection, loss, takeover and no-restart cases.
- Nine new isolated ROS scenarios verified default no-flight auditing and endpoint hold, restart rejection, configuration/source latches, missing-RC flight blocking, successful one-goal flight lifecycle, pilot takeover, command-loss landing and tracking/position departure handling. The asynchronous LAND assertion was corrected to await service delivery; control thresholds were not relaxed.
- Numerical checks verified inverse nonidentity session geometry, per-axis tracking rejection, measured corridor protection against stacked offsets and captured ground offsets in rotated axes. A further ROS ground-margin case verified that 0.06 m offsets on each axis retain `READY`, while 0.101 m on captured forward latches `STOPPED` without control requests; the ground endpoint-hold fixture also passed after this change.
- Thirty manager lifecycle checks and three repository-layout checks passed. The standalone hover artifact matches its canonical source.

These are software results. The next aircraft result to record is `READY` → `AUDIT_TRACK` → `AUDIT_END` with default check-only output, followed by supervised staged flight validation when RC and an on-site pilot are available.
