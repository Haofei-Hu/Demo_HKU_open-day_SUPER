# FAST-LIO with the PX4 IMU

The default remains the MID360 internal IMU. Select `--lio-imu-source px4`
explicitly for a new session. This is a nominal-mount comparison, not a calibrated
or flight-validated sensor replacement.

## What changes

- FAST-LIO reads `/mavros/imu/data_raw`, retaining its original timestamps.
  MAVROS already converts aircraft FRD gyro/acceleration into body FLU. The
  unavailable orientation (`orientation_covariance[0] = -1`) is expected.
  `/mavros/imu/data` is not substituted: its attitude and cached acceleration
  need not represent the same measurement instant.
- A separate per-session YAML uses the repository's nominal LiDAR-to-body
  rotation of +45 degrees yaw and translation `[0, 0, 0.10]` m. PX4 IMU origin
  is assumed to be the FC centre, and time offset is zero. Online extrinsic
  estimation is disabled. These assumptions have not been measured.
- Both body and aligned-pose converters use identity IMU-to-body rotation and
  zero IMU-to-FC lever arm in PX4 mode. This prevents applying the previous
  Livox IMU conversion a second time. World alignment is surveyed again for
  each new session; previous world-alignment assignments are not overwritten
  by the IMU selector.
- MAVROS starts before FAST-LIO. There is no automatic fallback to Livox if the
  PX4 stream is missing. Existing sessions cannot switch source in place.
- Before alignment, a 10-second read-only input check records original stamp
  intervals, receive intervals, header ages, acceleration norms, frames and
  publisher identities. It requires data, finite vectors, increasing nonzero
  stamps, one publisher and `base_link`. It does **not** certify delay, sample
  rate adequacy, physical accuracy or flight readiness. No rate/gap flight
  limit is introduced.

The source FAST-LIO YAML, LiDAR input, filter/noise settings, PX4 stream rates,
PID, flight height, speed and venue boundaries are unchanged. IMU selection does
not write FC parameters. The existing alignment/vision preparation still has
its normal EKF parameter handling.

## Install and inspect on the ground

Use the matching [installer](QUICK_START.md#install-or-update-once). Finish and
stop the previous run while landed and disarmed before updating converter files.
Then, with the aircraft stationary:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

python3 ~/Downloads/super_session.py start \
  --fixed-lio-extrinsics --lio-imu-source px4

python3 ~/Downloads/px4_imu_input_check.py \
  --seconds 30 --report "$HOME/Downloads/px4_imu_check_$(date +%Y%m%d_%H%M%S).json"

ros2 topic info /mavros/imu/data_raw --verbose
timeout 10s ros2 topic echo /Odometry nav_msgs/msg/Odometry \
  --no-daemon --qos-reliability best_effort --once --field header
```

This starts localization only; it does not start vision forwarding, control,
arming or a flight. Check that FAST-LIO subscribes to `data_raw` with compatible
QoS, that odometry advances, and inspect `lio.log`. The previously installed
FAST-LIO Best Effort IMU subscription remains necessary when the publisher uses
Best Effort. This Python update does not rebuild FAST-LIO.

Around 50 Hz was observed on the current aircraft, with receive gaps up to
79 ms. This is a transport observation, not proof of adequate integration or a
reason to invent a fixed time offset. Compare original stamp gaps against
receive gaps, and validate stationary and measured hand-motion localization
before physical flight. A frozen-map virtual rehearsal does not exercise
in-flight IMU vibration or prove the replacement works during flight.

## Complete ground rehearsal

Add `--lio-imu-source px4` to the existing `super_demo_test.py --restart ...`
command. All other route and boundary options remain the same. The restart uses
the existing fresh grounded/disarmed check, then captures a new IMU configuration,
world alignment and map. Do not reuse the previous session's venue calibration.

`super_ground_test.py` and `super_demo.py` also propagate this explicit selection,
but accepting the option is not evidence of physical flight validation.

## Reports and rollback

The session log folder contains `lio_imu_selection.json`, `fastlio_px4_imu.yaml`
and, after alignment preparation, `px4_imu_input.json`. The selection report saves
source/profile hashes, assumptions, converter hashes and backup paths. The main
run report retains the session paths. `~/Downloads/lio_imu_mount_state.json`
records the original Livox converter expressions; keep it with the backups.

To restore Livox, stop while grounded/disarmed and start a fresh session:

```bash
python3 ~/Downloads/super_session.py stop
python3 ~/Downloads/super_session.py start \
  --fixed-lio-extrinsics --lio-imu-source livox
```

This restores the saved original mounting expressions and original installed
FAST-LIO YAML. Run fresh alignment before vision/planning. A new full rehearsal
with `--restart --lio-imu-source livox` performs that preparation automatically.
