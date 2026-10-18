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

## Known issues

- MAVROS has exited during startup with `std::future_error: Promise already satisfied`; no permanent root-cause fix has been confirmed. See [restricted core-only recovery](REBOOT_QUICK_START.md#recover-an-exited-mavros-in-a-core-only-session).
- Parameter-service write responses can time out. The current manager confirms the value by readback and does not automatically replay writes.
- Updating individual scripts from mismatched releases has caused interface errors. Use the [complete installer](QUICK_START.md) to verify matching files.
- Persistent `WAIT_PLAN` requires inspection of the current planner log: the map may have no path, corridors may be infeasible, or optimization may fail. Waiting alone does not imply an executable trajectory exists.
- A subsequent onboard virtual rehearsal completed two rounds and stopped during the third outbound leg on a 5.44 cm position discontinuity after a stationary replan. The existing small-handoff handling did not accept this transition; its cause remains under investigation. Earlier successful replays do not establish that every handoff is resolved.

See [development checks](DEVELOPMENT.md) for verification commands and the [history index](HISTORY.md) for earlier debugging details.
