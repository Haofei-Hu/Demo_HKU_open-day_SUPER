# One-goal SUPER handover

The disarmed SUPER → MAVROS → PX4 protocol path is verified on the aircraft. `super_flight_control.py` adds a separate one-goal controller prototype. Its default mode audits a ground trajectory and emits debug poses only. The latest aircraft ground audit retained `AUDIT_END` beyond six seconds with fresh concurrent sampling. Planner-driven aircraft flight and the cause of an earlier local-pose freshness fault remain unresolved; isolated ROS fixtures use simulated telemetry and MAVROS command services, not a PX4 dynamics simulation.

## Ground audit

Fresh LIO sessions can have a nonzero raw-world vertical origin even when aligned/FC coordinates are near zero. Managed `low-speed-3d` startup now creates a session-local map profile before launching SUPER; the auditor receives that exact profile path. Only virtual ground/ceiling and initial map-centre Z are translated, and the translation remains fixed for the core session. This prevents a negative raw-world origin from falling below a hardcoded virtual floor. It does not change physical height goals, actual cloud obstacles or 10 cm margins. See [the observed failure and validation](PROGRESS_2026-10-15.md).

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

While parameter confirmation is incomplete, `CHECKING` status and warnings identify the unconfirmed `/mavros/param/get_parameters` or `/mavros/setpoint_position/get_parameters` service, its availability, pending request age, last confirmation age and any named value/type mismatch or response error. Individual CLI reads can match the required configuration while the controller still lacks a recent accepted batch response. Diagnose that distinction before changing FC parameters. Strict value/type matching, the original two-second pending-call deadline and three-second confirmation freshness remain unchanged; the auditor refuses a goal until `READY`.

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

## Latest aircraft vision-chain audit

Session `20261014_222151_860099` completed a new disarmed audit after the freshness diagnostic update. The service accepted one goal, 217 matched references were accepted, and `AUDIT_END` retained `[0.3462, -0.0154, 0.0757] m` in captured forward/left/FC-floor-height coordinates across 18 consecutive status reports in a 20-second display window. Reference endpoint errors were `[0.0462, -0.0154, -0.0093] m` from the requested 0.30 m same-height forward target, within the existing 0.10 m per-axis margins. No fault appeared in the captured status window.

Concurrent 30-second `--vision-chain` sampling covered raw odometry, aligned pose, native vision and FC local pose. Native vision received 1,500 messages, with maximum header age 47 ms and receive gap 51 ms. FC local-pose maxima were 21/44 ms; all four topics had one discovered publisher, valid increasing stamps and expected frames. The observer loop gap peaked at 24.746 ms and the sampled chain was fresh. This confirms a sustained ground reference hold with concurrent native-vision observation; the prior intermittent freshness faults remain unexplained. Aircraft tracking, airborne handover and 2 m/s flight remain unverified. See [the structured result](../data/observed_2026-10-14/super_handover_vision_audit.json).

## Exact-stamp vision pairing under callback bursts

The visual-pose relay originally used independent depth-one DDS readers for raw and aligned poses while requiring identical timestamps to join them. During a short callback pause, each reader can retain a different latest timestamp. Fresh inputs alone then do not guarantee a matched pair or native vision output. An isolated real-DDS regression reproduces this condition with 50 Hz-equivalent batches and a one-sample aligned-stream delay: both inputs remain fresh but the original queue produces no paired output for 1.162 seconds.

The relay now uses bounded depth-16 readers, covering the 250 ms freshness window at 50 Hz with headroom. In the same regression it forwards 52 identical input poses with a maximum output gap of 60 ms. Every queued input still passes the original timestamp-age checks; measurements are never retimed or republished to bridge a gap. Original source, geometry, parameter, covariance and writer-conflict gates remain active. An output-gap warning additionally reports raw/aligned receive ages, header ages and unmatched cache sizes. This fixes the reproduced join-starvation defect; the original aircraft faults lack event-time evidence establishing that defect as their cause, so aircraft confirmation is still required.

To load an updated relay into an existing grounded session, stop dependent audit/control windows first and use the explicitly requested `vision-restart` action:

```bash
python3 ~/Downloads/super_session.py handover-stop
python3 ~/Downloads/super_session.py vision-restart
python3 ~/Downloads/super_session.py handover-check
python3 ~/Downloads/control_chain_age_check.py --vision-chain --seconds 30
```

The restart requires fresh connected/disarmed/on-ground telemetry, one raw/aligned source, fresh aligned preview, the existing fusion parameter contract and absence of control writers or dependent hover/wire/reference/audit windows. It replaces only the vision process, waits for the old publisher to disappear, preserves the alignment file and core/planner processes, and verifies new vision output and FC localization. It performs no parameter writes, mode or arming requests. It is not an automatic recovery path and must not be used in flight.

The source-discovery check waits up to the existing 12-second gate timeout when either count is zero. More than one publisher is rejected immediately. A discovery timeout or duplicate-source rejection reports the observed per-topic counts and leaves the old relay running. Thirty-five lifecycle tests include delayed discovery, persistent absence and duplicate publishers; the new relay is never launched before unique inputs are discovered.

An aircraft parameter-service timeout was followed by a successful independent read with all six fusion values correct. MAVROS logged `failed to send response ... client will not receive response`. The manager now retries only idempotent `GetParameters` requests, reusing the discovered client within the original 12-second response budget. Late replies to cancelled requests are ignored. Writes and force-pulls are not replayed, and unsafe ground state or exhausted reads still block a restart. A live-DDS fixture under 50 Hz telemetry verifies healthy reads, first-response delay recovery, exhausted reads, single write transmission and arming rejection during a wait. This handles transient response loss; it does not make a persistently unavailable service valid.

## Earlier aircraft ground result

The latest trial accepted 217 references, reached `AUDIT_END` and retained the same endpoint in 18 successive status reports within a 20-second observation window, exceeding the six-second ground hold requirement. The held forward/left/FC-floor-height was `[0.2967, 0.0055, 0.1052] m`, with forward/left/up endpoint offsets `[-0.0033, 0.0055, 0.0202] m` from the requested same-height forward target. Concurrent 30-second sampling found all four chain topics fresh, with single publishers and no invalid/reordered stamps or unexpected frames. Local-pose maximum receive gap/header age were 44/22 ms. No native output or mode/arming requests were made. This verifies a sustained ground reference hold; physical tracking and airborne handover remain separate.

### Earlier terminal/freshness trial

The October 14 disarmed trial progressed through `READY` → `AUDIT_WAIT` → `AUDIT_TRACK` → `AUDIT_END`, accepting 218 references. Its held forward/left/FC-floor-height was `[0.3172, 0.0106, 0.0941] m`. These are reference coordinates, not physical aircraft displacement. No native output or mode/arming request was made.

Approximately 0.68 s after reaching `AUDIT_END`, the auditor latched `STOPPED` on `Missing/stale local`. It therefore verifies terminal-reference arrival and brief endpoint hold, not a stable six-second hold. The original generic fault does not distinguish receiver starvation, transport loss or a header-time excursion. A separate later 30-second sample found local-pose receive gaps at most 45 ms and header ages at most 19 ms, with one publisher and no invalid/reordered stamps; this does not establish the cause of the earlier fault. The supplied MAVROS log tail predates that event. Keep the 0.25 s freshness guard and collect event-time evidence before airborne handover. See [the recorded observation](../data/observed_2026-10-14/super_handover_audit.json).

Local and native-vision freshness faults include monotonic `receive_age`, ROS `header_age`, the last source stamp and the mission phase. The limits remain 0.25 s for reception/header age and -0.05 s for future-dated headers. A historical `Phase: READY` log does not establish current readiness: a later input gap can latch `STOPPED` before the audit request, which then rejects the request without sending a goal.

For vision-loss diagnosis, the sampler's optional `--vision-chain` mode observes `/Odometry`, `/super_debug/aligned_pose_cov`, `/mavros/vision_pose/pose_cov` and `/mavros/local_position/pose`. It prints timestamped receive-gap/header-age transitions after an initial three-second discovery period, per-source statistics and the observer's maximum loop gap. It creates subscriptions only. Its report exit status is not a flight-readiness verdict. An independent subscriber can observe healthy input while the controller's own receiver has a gap; compare both event timestamps rather than treating a later healthy window as proof of uninterrupted reception.

```bash
python3 ~/Downloads/control_chain_age_check.py --vision-chain --seconds 30
```

The October 14 session `20261014_215344_305294` stopped after 86 accepted ground references on a generic native-vision freshness fault. A fresh session `20261014_222151_860099` reached `READY` approximately 49.5 seconds after auditor startup, then stopped on the same fault before goal acceptance (`accepted=0`). Its audit service rejected the request. Subsequent readiness snapshots do not establish either fault's cause. No aircraft motion or 2 m/s execution was validated by these attempts.

Diagnostic validation passed five metric unit tests, eleven isolated live-DDS sampling cases (including native-only receive loss and stale headers with fresh upstream inputs), and four controller fault-latch cases for local/vision receive loss and future headers. The controller cases retain the 250 ms receive/header limits, report the failed source and reject further audit/start requests after stopping. These fixtures simulate telemetry; they do not establish the aircraft fault's cause or validate PX4 flight dynamics.

To collect the original planning-chain view during a new disarmed audit, wait for current `READY`, then run the read-only sampler alongside the audit in the same command terminal:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
audit_sample_log=$(mktemp "$HOME/Downloads/handover_sample_XXXXXX")
python3 ~/Downloads/control_chain_age_check.py --seconds 30 > "$audit_sample_log" 2>&1 &
audit_sampler_pid=$!
timeout 8s ros2 service call /super_flight/audit std_srvs/srv/Trigger '{}'
timeout 20s ros2 topic echo \
  /super_debug/super_flight_status std_msgs/msg/String \
  --no-daemon --qos-reliability best_effort \
  --qos-durability volatile --full-length --field data
wait "$audit_sampler_pid"
cat "$audit_sample_log"
```

Only the explicit audit call requests a ground planning goal. Sampling and status inspection do not emit flight requests. Evaluate the independent sampler together with the controller's fault-time ages; a later healthy window alone cannot explain an earlier latch.

## Read-only preparation snapshot

`flight-check` consolidates grounded state, fresh local/vision/estimator telemetry, pose agreement, typed parameter readbacks, LOCAL_NED, RC input, native publisher conflicts, managed processes and the actual reserved 3D planner configuration. It creates only telemetry subscriptions and parameter-read clients; it starts no process, publishes no goal/setpoint and calls no mode, arming or parameter-write service. Existing localization, planning and the completed auditor remain running.

```bash
python3 ~/Downloads/super_session.py flight-check
```

The JSON includes `checks`, `blockers`, `remote_preparation_checks_passed` and `sampled_software_prerequisites_passed`. During remote preparation, missing fresh RC or disarmed POSCTL/manual input should remain visible as blockers for later supervised execution. The snapshot does not assess on-site supervision, physical clearance, complete-flight tracking or the PX4 `commander check`; no flight is initiated even if all sampled checks pass. Existing airborne or stale/non-grounded FC state prevents the grounded check from running. Its normal exit means a report was produced; use the JSON booleans to inspect prerequisite results.

### Aircraft remote preparation result

The aircraft snapshot returned `remote_preparation_checks_passed=true`, no errors, all required parameter matches, fresh valid localization, LOCAL_NED, single localization sources, active managed processes and the reserved 3D profile. FC/vision position separation was 0.00712434 m and yaw difference 0.0114514°. The aircraft was connected/disarmed in AUTO.LOITER with manual input false and no native control publishers. One RC publisher was advertised, but no fresh RC data/channels were received; publisher presence alone does not establish a live transmitter.

Only `posctl_with_manual_input` and `fresh_rc_with_seven_channels` blocked the full sampled prerequisites. These are retained on-site input requirements. Continue with supervised 1 m takeoff, one 0.30 m low-speed forward handover, six-second endpoint hold and landing after on-site RC/manual mode and PX4 preflight are verified. The current remote session remains grounded. See [the aircraft readiness snapshot](../data/observed_2026-10-14/remote_flight_readiness.json).

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
- Thirty-two manager lifecycle checks and three repository-layout checks passed. Three isolated read-only readiness cases passed without any mode, arming, parameter-write, goal or native requests. The standalone hover artifact matches its canonical source.

The sustained default ground audit and aircraft read-only readiness snapshot are recorded separately from these software results. The next physical result is supervised staged flight validation when RC and an on-site pilot are available.
