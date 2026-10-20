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

The map-height correction was tested against the uploaded failing scene itself.
The preceding binary reproduced the third-return deadlock at raw Z
`-11.796818400889041 m`, above the initialized `-11.8 m` map ceiling. The corrected
binary completed all five round trips, 20 stages, a 6.05-second final hold and
virtual landing/disarming. All 52 related tests passed. No map-ceiling or invalid
rest-start warnings occurred in that replay. Candidate rejections remain normal:
the planner retains a verified trajectory and searches for a replacement.
See the [reproduction and verification record](../data/software_checks_2026-10-18/map_height_release.json)
and [full mission report](../data/software_checks_2026-10-18/map_height_ground_mission_report.json).
This validates the captured-scene regression, not physical flight or every future map.

The bounded-backup correction addresses a reproduced `WAIT_PLAN` deadlock:
finite soft penalties allowed a backup stopping point a few millimetres outside
the venue, so the continuous publication check rejected it. The clean corrected
build completed five round trips and virtual landing on both the failing
2 m-wide scene and an earlier 2.8 m-wide scene; 67 related tests passed.
See [bounded backup waypoint verification](validation/bounded_backup_waypoints.md).
This does not establish physical flight readiness.

## Known issues

- MAVROS has exited during startup with `std::future_error: Promise already satisfied`; no permanent root-cause fix has been confirmed. See [restricted core-only recovery](REBOOT_QUICK_START.md#recover-an-exited-mavros-in-a-core-only-session).
- Parameter-service write responses can time out. The current manager confirms the value by readback and does not automatically replay writes.
- Updating individual scripts from mismatched releases has caused interface errors. Use the [complete installer](QUICK_START.md) to verify matching files.
- Persistent `WAIT_PLAN` requires inspection of the current planner log: the map may have no path, corridors may be infeasible, or optimization may fail. Waiting alone does not imply an executable trajectory exists.
- The stationary-start fix exposed a separate map-height mismatch on run `20261018_222714_088769`: a committed candidate could satisfy the tilted venue but cross the initialized map ceiling. `VENUE_MAP_HEIGHT_V1` intersects search, all corridors and complete candidate checks with the effective map-height interval. Updating requires a C++ rebuild; Python updates alone are insufficient. Actual onboard reruns remain necessary.

See [development checks](DEVELOPMENT.md) for verification commands and the [history index](HISTORY.md) for earlier debugging details.
