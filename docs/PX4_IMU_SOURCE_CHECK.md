# PX4 IMU candidate check

The captured FAST-LIO configuration uses `/livox/imu` (~200 Hz), from the MID360.
The previously recorded `/mavros/imu/data` stream was ~50 Hz. Availability and
rate of `/mavros/imu/data_raw` have not yet been established on the aircraft.

Run `scripts/analysis/imu_source_check.py --duration 20` with ROS Humble sourced,
ROS_DOMAIN_ID=0 and ROS_LOCALHOST_ONLY=0. Keep the aircraft physically stationary
with motors stopped and keep the existing Livox and MAVROS nodes running.
The tool only subscribes to three sensor_msgs/Imu topics; it does not start
nodes, switch sources, request MAVLink streams, publish vision or change any
parameters. It saves report.json and raw samples.jsonl in a timestamped directory
under ~/Downloads/imu_source_checks and creates imu_sources.zip there.

The report includes receipt and timestamp rates, gaps, duplicate/backward stamps,
frame IDs, covariance unavailable flags, acceleration/gyro norms and receipt ages.
NO_DATA means no messages were received during this capture, not that the hardware
lacks an IMU. Sensor QoS is best effort; observed gaps may include subscriber loss.
Receipt age does not establish the physical time offset between LiDAR and IMU.
Livox acceleration in the observed stream is g-like; MAVROS convention is m/s².
Do not rescale blindly: this FAST-LIO version normalizes against initialization
gravity. FAST-LIO consumes acceleration/angular velocity, not IMU orientation.

Before an external-IMU LIO trial, establish adequate message rate, timestamp
compatibility and actual sensor axes. The existing body conversion assumes the
MID360 IMU frame, nominal +45-degree yaw into body FLU, with nominal LiDAR origin
[0,0,0.10] m from the FC centre. Switching only common.imu_topic invalidates that
conversion and the current LiDAR-to-IMU extrinsics. The nominal mount is not a
calibration of the flight-controller IMU position. MAVROS already converts PX4
frames; do not apply the PX4 board rotation a second time. An isolated ground
comparison must keep candidate LIO output out of the external-vision/control
path until the transforms and timing are validated. Changing sources is not
proof that the MID360 IMU caused drift; flight data also showed FC vibration.
