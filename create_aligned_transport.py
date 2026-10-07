from pathlib import Path
import sys

directory = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / 'Downloads'
source = directory / 'raw_pose_cov_transport.py'
target = directory / 'aligned_pose_cov_transport.py'
text = source.read_text()
changes = [
    ('RawPoseCovTransport', 'AlignedPoseCovTransport', 2),
    ("'raw_pose_cov_transport'", "'aligned_pose_cov_transport'", 1),
    ("Odometry, '/Odometry', self.receive, qos_profile_sensor_data",
     "PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', self.receive, qos_profile_sensor_data", 1),
    ("if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':",
     "if msg.header.frame_id != 'px4_enu_preview':", 1),
    ('TRANSPORT ONLY: raw IMU-origin pose/covariance, camera_init retained; ',
     'GROUND TRANSPORT: aligned FC-centre pose/covariance, preview origin; ', 1),
    ('no FC mounting or PX4 world alignment. Waiting for EKF2_EV_CTRL=0.',
     'preliminary calibration. Waiting for EKF2_EV_CTRL=0.', 1),
    ('First raw pose/covariance forwarded with original frame and stamp. ',
     'First aligned pose/covariance forwarded with original frame and stamp. ', 1),
]
for old, new, count in changes:
    if text.count(old) != count:
        raise SystemExit(f'Source mismatch: {old!r}; no file written.')
    text = text.replace(old, new)
text = text.replace('/Odometry.pose', '/super_debug/aligned_pose_cov.pose')
text = text.replace('Unexpected FAST_LIO frame names', 'Unexpected preview frame name')
compile(text, str(target), 'exec')
if target.exists() and target.read_text() != text:
    raise SystemExit('Target already exists with different contents; no file written.')
target.write_text(text)
print('Created:', target)
print('EV_CTRL=0 gate, timestamp checks and original stamps retained.')
