# Automatic ground test

`super_ground_test.py` prepares a fresh aircraft session, captures its alignment and runs one disarmed SUPER handover audit. It sources the existing ROS/workspace environments itself. Once installed, one onboard terminal command runs the workflow:

```bash
python3 ~/Downloads/super_ground_test.py --restart --fixed-lio-extrinsics
```

Keep the aircraft stationary, disarmed and on its takeoff floor. `--restart` explicitly requests replacement of an existing managed session; the existing manager requires fresh connected/disarmed/on-ground telemetry before stopping it. Without this flag, an existing session is retained and the runner refuses to proceed. Unmanaged nodes are never terminated. A process lock prevents two automatic runners, including runners with different session names, from operating together.

`--fixed-lio-extrinsics` uses the current comparison override; it does not edit the FAST_LIO YAML or mechanical mounting transform. Omitting it selects the original YAML baseline. No dependencies are installed and no scripts are downloaded by the test itself.

## Workflow

1. Validate installed prerequisites and start the managed core. An explicitly requested restart uses the existing guarded stop action.
2. Run the existing odometry check and a new 15-second stamp-paired attitude survey. Write `alignment.json` in a new run directory.
3. Validate and apply this run's full-precision quaternion through the existing vision initialization. No historical quaternion is reused and terminal text is not parsed.
4. Capture the current map reference and launch the `low-speed-3d` planner with frozen session-relative vertical bounds.
5. Start the check-only handover controller and wait up to 60 seconds for a fresh `READY`, valid localization and unique input/status publishers.
6. Call `/super_flight/audit` once. Observe the audit and all four localization streams concurrently for 30 seconds. Require a continuous, fresh `AUDIT_END` hold of at least six seconds with accepted references.
7. Save the read-only preparation snapshot and require `remote_preparation_checks_passed` before reporting `SUCCESS`.

The automatic quaternion acceptance policy requires at least 20 matched samples, unit/finite components, pairing gap at most 50 ms, body timestamp ages between -50 and 250 ms, and maximum rotation spread at most 2 degrees. `--max-alignment-spread-deg` makes that last automation criterion explicit and configurable; it changes neither flight tracking tolerances nor map-capture limits. The existing 2.5 cm map-capture criterion and 10 cm flight/reference margins remain unchanged.

Both receipt gaps and timestamp ages are checked during the audit. Missing/invalid estimator state, changed or duplicate input sources, a native control publisher, stale status, controller faults or a rejected audit fail the run. The audit request is never replayed after a lost acknowledgement. A single failed step stops the sequence; there are no automatic node restarts, survey retries, rebases or tolerance changes.

The manager retains its existing parameter handling: alignment initialization pulls parameters and changes `EKF2_EV_CTRL` through readback-confirmed operations; only bounded idempotent parameter reads may retry. This runner does not add parameter-write retries.

## Reports and recovery

Every run creates:

```text
~/Downloads/super_ground_tests/<timestamp>/
    report.json
    alignment.json
    flight_check.json       # reached only after the audit passes
    <stage-number>_<action>.log
```

`report.json` records the failed/current stage, measured quaternion, managed session/log directory, audit transitions, four-stream timing and the final readiness snapshot. Failure reports include partial audit evidence when available. Native node logs remain under the existing `super_session_logs` directory. A failed run exits nonzero; a completed ground run exits zero. Missing RC/manual POSCTL is retained in the readiness snapshot but does not turn a valid remote ground preparation into a flight approval.

After the final report is saved, the terminal ends with `RESULT: SUCCESS` or `RESULT: FAIL`, followed by `REPORT: <absolute path>`. The JSON uses the same `result` value, records start/finish times and includes `failed_stage` plus `error` on failure. In-progress reports use `RUNNING`. A `SUCCESS` result refers to the complete disarmed test, not flight readiness.

Processes and logs remain available after success or failure. Inspect the reported stage before restarting; another full test requires an explicit `--restart`. Genuine bad input still fails the run. One successful run cannot establish reliability over future restarts or physical tracking accuracy.

## Installation and compatibility

Update these three files together in the existing flat aircraft script directory, backing up their previous versions:

- `scripts/maintenance/super_ground_test.py`
- `scripts/maintenance/super_session.py`
- `scripts/localization/frame_alignment_check.py`

The existing integration prerequisites, including `create_session_map_profile.py`, `control_chain_age_check.py`, the handover controller and all four built ROS workspaces, must already be installed. The deployment catalog includes the new runner; the existing installer preserves installed files unless an explicit update is requested.

Manual alignment is unchanged. `super_session.py align --report-json NEW_FILE` additionally writes a machine-readable survey; `flight-check --report-json NEW_FILE` writes the readiness snapshot. Neither output may overwrite an existing file. Saving a manual survey does not apply it automatically.

## Future supervised flight entry

Preparation is isolated in `GroundTest.prepare()` and auditing in `AuditObserver`. A future flight runner can reuse preparation while replacing the audit stage with a separate, explicitly requested flight lifecycle. It must recheck current RC/manual POSCTL, landed state, valid localization, source/configuration identity, single control ownership, parameter limits and on-site supervision before enabling any flight output. A saved ground report is not sufficient authorization or current evidence for takeoff.

The current runner has no `--execute` option and creates no native setpoint publisher, mode client or arming client. Its sole controller trigger is the disarmed audit service. Validation uses isolated ROS telemetry/services; the first aircraft run of the automatic runner remains to be performed.
