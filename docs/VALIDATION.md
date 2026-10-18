# Verification scope and known issues

## Available evidence

| Item | Result and scope |
|---|---|
| Complete five-round mission | Real SUPER, captured aircraft point clouds, and virtual vehicle feedback completed 20 stages, final hover, and landing; [full report](../data/software_checks_2026-10-18/stationary_handoff_ground_mission_report.json) |
| Stationary trajectory handoffs | Targeted small-handoff reproduction and controller regressions; the full five-round replay did not exercise that branch, so the evidence is recorded separately; [verification summary](../data/software_checks_2026-10-18/stationary_handoff_release.json) |
| Physical fixed-position flight | Earlier on-site takeoff, hover, and landing records; these do not establish complete obstacle-avoidance mission success |
| 1.8 m physical entrypoint | Configuration and aircraft-envelope validation, report stages, ROS controller initialization, and explicit start conditions tested without operating an aircraft from the cloud |

Virtual feedback does not reproduce physical dynamics, wind, FC tracking error, or missed LiDAR detections. The complete physical five-round and 1.8 m missions still require on-site validation. `SUCCESS` must come from the complete report for the corresponding run, not a partial leg or an earlier report.

The 1.8 m release passed 47 related checks: 13 mission geometry/report checks, five complete-installer checks, three deployment-layout checks, and 26 ROS controller, handoff, and rehearsal-profile checks. ROS checks used an isolated test domain without an aircraft connection.

The continuous-rest-start release passed 51 related checks and a rebuilt real-SUPER replay with five round trips, all 20 stages, a 6.05-second final hover, and confirmed virtual landing/disarming. See the [release verification record](../data/software_checks_2026-10-18/continuous_rest_start_release.json) and [complete replay report](../data/software_checks_2026-10-18/continuous_rest_start_ground_mission_report.json). The recorded 5.44 cm start-selection regression is tested separately; the full replay uses an earlier captured map, not the latest failing onboard scene.

## Known issues

- MAVROS has exited during startup with `std::future_error: Promise already satisfied`; no permanent root-cause fix has been confirmed. See [restricted core-only recovery](REBOOT_QUICK_START.md#recover-an-exited-mavros-in-a-core-only-session).
- Parameter-service write responses can time out. The current manager confirms the value by readback and does not automatically replay writes.
- Updating individual scripts from mismatched releases has caused interface errors. Use the [complete installer](QUICK_START.md) to verify matching files.
- Persistent `WAIT_PLAN` requires inspection of the current planner log: the map may have no path, corridors may be infeasible, or optimization may fail. Waiting alone does not imply an executable trajectory exists.
- An onboard virtual rehearsal stopped during the third outbound leg on a 5.44 cm stationary-replan discontinuity. The continuous-rest-start source patch now removes grid-centre relocation in venue-enabled planning; it requires a planner rebuild. The recorded coordinate regression is covered separately from replay on an earlier captured scene. A new run on the failing onboard scene remains necessary; arbitrary localization jumps are not accepted.

See [development checks](DEVELOPMENT.md) for verification commands and the [history index](HISTORY.md) for earlier debugging details.
