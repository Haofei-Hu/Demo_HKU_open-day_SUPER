# Deployment checkpoint — 2026-10-07

This records work observed on the aircraft's Ubuntu machine. Cloud tests use synthetic ROS data and do not establish flight readiness. The aircraft remained disarmed; external vision fusion remained disabled (`EKF2_EV_CTRL=0`).

## Completed code and machine checks

- FAST_LIO upstream source: `a4743b095409588842a5b30ddfa27e29d2f99164`. Existing MID360 YAML and RViz edits were retained.
- The covariance repair was built successfully on the machine (initial build 1 min 22 s; incremental build succeeded). `/Odometry` from the installed workspace had symmetric, populated covariance and approximately 10 Hz output.
- Added the timing probe in [apply_fastlio_timing_probe.py](../scripts/maintenance/apply_fastlio_timing_probe.py) and [its patch](../patches/fastlio_timing_probe.patch).
- Changed **only IMU subscription reliability** from default Reliable to Best Effort with `KEEP_LAST(10)` in [apply_fastlio_imu_qos.py](../scripts/maintenance/apply_fastlio_imu_qos.py) and [its patch](../patches/fastlio_imu_best_effort.patch). Rebuilt and restarted the machine's FAST_LIO executable.
- Archived the attitude pairing, aligned ground preview, aligned ground transport and stationary-position checks. The standalone aligned relay avoids dependence on the previously failed text-conversion command.

## Timing investigation and result

The isolated sensor probe reported fresh input: LiDAR age min/mean/max **0.105 / 0.109 / 0.117 s**, IMU **0.000 / 0.001 / 0.006 s**, approximately 200 Hz IMU. Odometry was nevertheless **1.284 / 1.368 / 1.653 s** old. The LiDAR header describes scan acquisition; its roughly 0.11 s age is not equivalent to odometry age stamped at scan end. CPU sampling showed approximately 84% aggregate idle, not machine-wide exhaustion.

The probe measured ages inside the actual C++ callbacks, before configured timestamp correction. With the original IMU reliability, the runtime showed:

```text
pub_age=1.672..1.753 lrx≈0.110 irx≈1.274..1.275
latest_imu≈1.283..1.284 lidar_q=16..17 imu_q≈1
pre_ms=0.3..0.6 proc_ms=4.3..9.3 scan_dt≈0.100
```

Processing each scan took milliseconds while IMU callback delivery was over a second behind; synchronization consequently retained LiDAR scans. These observations point to the IMU delivery path in this environment. They do not identify a particular DDS retransmission/cache mechanism or prove the cause of every earlier position error.

After the IMU Best Effort change and rebuild, the reported window showed:

```text
pub_age=0.023..0.024 lrx≈0.110 irx=0.000 (rounded)
latest_lidar=0.124..0.125 latest_imu=0.009..0.010
lidar_q=0 imu_q=3 oldest_lidar_age=-1.000 (queue empty)
pre_ms=0.3..0.5 proc_ms=7.8..8.9 scan_dt=0.100
```

Original measurement timestamps and the relay's freshness limits were retained. No `EKF2_EV_DELAY`, NTP offset or IMU rate change was used to hide the delay. Longer operation, packet loss and full planning/point-cloud load remain to be checked.

## Configuration values retained

The active machine reads the installed `mid360.yaml`. Reported values were `point_filter_num=3`, `filter_size_surf=0.5`, `filter_size_map=0.5`, `max_iteration=3`, `common.time_sync_en=false` and `common.time_offset_lidar_to_imu=0.0`; these were not changed to solve the delay. ROS publication flags in Quick Start are startup overrides, not edits to the underlying YAML.

The real SUPER configuration was read at `~/super_ws/src/SUPER/super_planner/config/mid360_real_ros2.yaml`: `click_goal_en=true`, `click_goal_topic="/goal_pose"`, `click_height=-0.05`, `virtual_ground_height=-1.0`. The −0.05 value is a planning test Z coordinate, not the venue's above-floor flight height. The complete aircraft YAML/RViz files have not been copied into this repository; only values supplied in the session are recorded.

## Motion and alignment observations

An earlier approximately 30 cm physical movement gave an implausible approximately 1.85 m preview displacement. A subsequent measurement also found large raw odometry variation, with a constant inferred preview origin (145 matched stamps, inferred variation zero). This localized that discrepancy upstream of the preview's origin subtraction.

After a localization reset, a stationary 15-second raw check reported position ranges **[0.0077, 0.0092, 0.0163] m**, mean/max age **0.025 / 0.029 s**. A later manual translation was corrected by the operator to **approximately 20 cm**; raw odometry displacement magnitude was **0.215 m**. That is coarse agreement, not a precision scale calibration.

The final attitude survey after the final FAST_LIO rebuild/restart produced:

```text
Received body/imu: 150 704 Matched: 141
Candidate rotation quaternion xyzw: [0.000591, 0.018708, 0.58887, 0.808011]
Candidate rotation RPY degrees: [1.318, 1.693, 72.188]
Rotation spread mean/max: 0.056/0.115 deg
Stamp pairing gap max: 0.031s
Body age mean/max: 0.026/0.027s
```

The preview archives this **session-specific** quaternion; remeasure after any FAST_LIO initialization change. Earlier candidates at approximately 68°, 79° and other headings are historical observations, not reusable mounting calibrations. Nominal mounting is LiDAR +45° yaw, 0.10 m above the FC, with configured IMU–LiDAR offset `[-0.011,-0.02329,0.04412]`; online extrinsic estimation is enabled, but these scripts do not ingest its live estimated extrinsics.

The preview composes mounting and world rotation, shifts position to nominal FC centre, transforms position/angular covariance including lever-arm cross terms, preserves stamps and subtracts the first accepted FC position. It publishes only the debug ENU topic. Its origin has no floor or venue meaning yet. MAVROS performs its own ROS ENU/body-to-MAVLink conversions; do not add a second manual ENU-to-NED conversion here.

## Ground transport checkpoint

Only the aligned relay was running; do not run raw and aligned relays together. `/mavros/vision_pose/pose_cov` output was **9.997–10.003 Hz**, observed intervals **0.091–0.112 s**. The initial parameter-service warning cleared before forwarding began. The relay rejects stale/future/repeated stamps and invalid pose/covariance; it requires a recent integer `EKF2_EV_CTRL=0` confirmation. A nonzero confirmation makes this ground-only relay stop forwarding. Parameter polling is not an atomic interlock: stop the relay before changing fusion parameters.

This verifies ROS transport, **not PX4 receipt or fusion**. Parameter reads: `EKF2_HGT_REF=1`, `EKF2_EV_DELAY=0.0`, `EKF2_EV_POS_X/Y/Z=0.0`; these were not changed. `SENS_BOARD_ROT=8` is already handled by PX4. Last state observed was connected, disarmed, `AUTO.LOITER`; horizontal estimator validity was false before external fusion. A time-sync round-trip sample was **6.454 ms** on October 7; October 6's 0.72–4.20 ms samples are separate observations.

## Current terminal assignments

| Terminal | Process / role |
| --- | --- |
| 1 | MID360 driver |
| 2 | FAST_LIO headless diagnostic baseline; map, path and both scan outputs disabled |
| 3 | Read-only checks and 15-second attitude survey |
| 4 | Optional visualization-only identity `world` to `camera_init` TF |
| 5 | SUPER stopped while `/cloud_registered` is disabled |
| 6 | MAVROS serial connection, baseline GCS forwarding disabled |
| 7 | `fastlio_body_debug.py` |
| 8 | `aligned_pose_preview.py` |
| 9 | `aligned_pose_cov_transport.py`; use for builds only after stopping relay |
| 10 | QGroundControl when continuing the next check |

See the README's first Quick Start for restart commands. The currently tested localization profile does not supply SUPER's registered-cloud input. Before planning, restore world scan output, validate load and perform a new session alignment after any FAST_LIO restart.

## Next session: actual PX4 receipt

The following sequence was prepared but **not performed** by the end of the session:

1. Stop the ground relay (9) and MAVROS (6). Keep FAST_LIO and preview running if preserving this session alignment.
2. Start QGroundControl (10), disable automatic Pixhawk/USB connection and disconnect any existing USB connection; enable UDP listening on port 14550. MAVROS must own the serial port.
3. Restart MAVROS (6) with localhost GCS forwarding:

   ```bash
   source /opt/ros/humble/setup.zsh
   ros2 launch mavros px4.launch \
     fcu_url:="serial:///dev/ttyACM0:2000000" \
     gcs_url:="udp://127.0.0.1:14555@127.0.0.1:14550"
   ```

4. Confirm connected/disarmed state and `EKF2_EV_CTRL=0`; restart the aligned ground relay (9).
5. In QGroundControl's MAVLink Console, run the read-only commands and retain their outputs:

   ```text
   ver all
   listener vehicle_visual_odometry 5
   ```

Verify actual firmware and fresh visual-odometry reception before designing a fusion-capable bridge. No parameter changes, arming, Offboard entry or flight have been completed.

## Venue requirements clarified today

- Venue: 3 m wide × 6 m long × 3 m high; demonstration target speed 2 m/s.
- Maximum height: **FC centre 2 m above floor**.
- Airframe: 225 mm wheelbase and 5-inch (127 mm) propellers. A symmetric X-frame gives an approximate axis-aligned propeller envelope of 286 mm; this is not a measured collision envelope, especially under yaw or with accessories.
- Takeoff is near a short side, **not necessarily centred**. Exact boundary distances and venue heading are still needed.
- FC centre on the takeoff floor: approximately 8.5 cm high. If the FC-at-takeoff origin is actually established on that floor, floor Z is −0.085 m and the FC height ceiling is relative Z 1.915 m. Do not apply those numbers directly to an uncalibrated bench preview or raw `camera_init` frame.
- Final boundary margins, map planes, route, flight controller bridge and execution limits remain unapplied. Target versus strict ceiling interpretation of 2 m/s is still to be resolved. A ten-second flight would not remove the consequences of delayed localization.
