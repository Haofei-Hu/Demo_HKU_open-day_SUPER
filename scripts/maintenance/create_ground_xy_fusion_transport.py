#!/usr/bin/env python3
"""Create a separate reviewed ground XY relay from the existing aligned relay."""
from pathlib import Path
import sys
root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / 'Downloads'
source=(root/'aligned_pose_cov_transport.py').read_text()
doc_start=source.index('"""');doc_end=source.index('"""',doc_start+3)+3
source=source[:doc_start]+'''"""Disarmed ground XY-fusion test only; never use for flight.

Preserve the preview pose/covariance/frame/stamp. Require recent FC state,
EV_CTRL in {0,1,9}, zero FC-centred EV lever arm and EV_QMIN=0. No parameter
writes, arming, mode changes or setpoints. Stop before source/preview resets.
"""'''+source[doc_end:]
changes=[
('AlignedPoseCovTransport','GroundXYFusionTransport',2),
("'aligned_pose_cov_transport'","'ground_xy_fusion_transport'",1),
('from nav_msgs.msg import Odometry','from mavros_msgs.msg import State',1),
('ev_zero','configuration_ok',7),
("self.declare_parameter('max_age_seconds', 0.5)","self.declare_parameter('max_age_seconds', 0.25)",1),
("request.names = ['EKF2_EV_CTRL']","request.names = ['EKF2_EV_CTRL', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y',\n                         'EKF2_EV_POS_Z', 'EKF2_EV_QMIN']",1),
('response is not None and len(response.values) == 1\n                and response.values[0].type == 2\n                and response.values[0].integer_value == 0',
 'response is not None and len(response.values) == 5\n                and response.values[0].type == 2\n                and response.values[0].integer_value in (0, 1, 9)\n                and all(v.type == 3 and math.isfinite(v.double_value)\n                        and abs(v.double_value) <= 1e-9\n                        for v in response.values[1:4])\n                and response.values[4].type == 2\n                and response.values[4].integer_value == 0',1),
("'GROUND TRANSPORT: aligned FC-centre pose/covariance, preview origin; '\n            'preliminary calibration. Waiting for EKF2_EV_CTRL=0.'",
 "'DISARMED GROUND XY/YAW TEST ONLY: EV_CTRL 0/1/9, zero EV_POS and QMIN=0. '\n            'Preserves original pose and stamp; no flight/setpoint output.'",1),
("'EKF2_EV_CTRL is not confirmed integer 0; publishing blocked'", "'Ground XY parameter configuration not confirmed; publishing blocked'",1),
("'Awaiting recent EKF2_EV_CTRL=0 confirmation; publishing blocked'", "'Awaiting recent ground XY parameter confirmation; publishing blocked'",1),
("'This confirms ROS transport only, not PX4 fusion.'", "'Check PX4 estimator aid-source status to verify fusion.'",1),
('        self.timer = self.create_timer(1.0, self.poll_parameter)',
 '        self.connected_disarmed = False\n        self.state_received_at = -math.inf\n        self.state_stamp = 0\n        self.arm_seen = False\n        self.state_subscription = self.create_subscription(\n            State, \'/mavros/state\', self.state_received, qos_profile_sensor_data)\n        self.timer = self.create_timer(1.0, self.poll_parameter)',1),
('    def warn(self, reason):', '''    def state_received(self, msg):
        self.state_received_at = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.state_stamp = stamp
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        self.arm_seen = self.arm_seen or msg.armed
        self.connected_disarmed = (
            msg.connected and not msg.armed and stamp > 0 and -0.05 <= age <= 2)

    def warn(self, reason):''',1),
('    def receive(self, msg):\n', '''    def receive(self, msg):
        if self.arm_seen:
            self.warn('Arming observed: ground relay latched off; no flight support')
            return
        state_age = (self.get_clock().now().nanoseconds - self.state_stamp) / 1e9
        if (not self.connected_disarmed or time.monotonic() - self.state_received_at > 2
                or not -0.05 <= state_age <= 2):
            self.warn('Awaiting fresh connected/disarmed FC state; publishing blocked')
            return
''',1),
]
for old,new,count in changes:
    actual=source.count(old)
    if actual!=count: raise SystemExit(f'{old[:100]!r}: expected {count}, found {actual}')
    source=source.replace(old,new)
compile(source,'ground_xy_fusion_transport.py','exec')
target = root / 'ground_xy_fusion_transport.py'
if target.exists() and target.read_text() != source:
    raise SystemExit('Target differs from this reviewed version; no file written.')
target.write_text(source)
print('Prepared ground_xy_fusion_transport.py')
