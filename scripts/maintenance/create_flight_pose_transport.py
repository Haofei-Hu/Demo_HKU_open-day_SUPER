#!/usr/bin/env python3
"""Install the reviewed session-vision relay from plain source."""
import hashlib
from pathlib import Path
import shutil
import tempfile

source = r'''#!/usr/bin/env python3
"""Session-bound visual pose relay. Debug output by default; no flight commands.

Initialize against three identical-stamp raw/aligned poses while disarmed.
Continue localization after subsequent arming, in any mode. Freeze the source
identities, geometry file and inferred origin; reject changes instead of rebasing.
Native output requires the explicit immutable publish_to_mavros parameter.
This relay alone does not implement flight control, venue limits or failsafes.
"""
import hashlib
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State

from control_bridge_preview import Mapping, load_converter


DEBUG_TOPIC = '/super_debug/flight_vision_pose_cov'
NATIVE_TOPIC = '/mavros/vision_pose/pose_cov'
PARAMETERS = ['EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_EV_POS_X',
              'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z', 'EKF2_EV_QMIN']


class FlightPoseCovTransport(Node):
    def __init__(self, **kwargs):
        super().__init__('flight_pose_cov_transport', **kwargs)
        immutable = ParameterDescriptor(read_only=True)
        self.declare_parameter('alignment_path', str(Path.home() / 'Downloads/aligned_pose_preview.py'), immutable)
        self.declare_parameter('publish_to_mavros', False, immutable)
        self.native = self.get_parameter('publish_to_mavros').value
        if not isinstance(self.native, bool):
            raise ValueError('publish_to_mavros must be boolean')
        self.path = Path(self.get_parameter('alignment_path').value)
        self.geometry_digest = hashlib.sha256(self.path.read_bytes()).digest()
        self.mapping = Mapping(load_converter(self.path))
        self.source_ids = None
        self.fault = None
        self.ever_sent = False
        self.sent = 0
        self.last_stamp = 0
        self.raw, self.aligned = {}, {}
        self.state = None
        self.state_at = -math.inf
        self.configuration_ok = False
        self.verified_at = -math.inf
        self.future = None
        self.request_at = -math.inf
        self.last_warning = -math.inf
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, NATIVE_TOPIC if self.native else DEBUG_TOPIC, 10)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.inputs = [
            self.create_subscription(Odometry, '/Odometry', self.raw_received, latest),
            self.create_subscription(PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov',
                                     self.aligned_received, latest),
            self.create_subscription(State, '/mavros/state', self.state_received, qos_profile_sensor_data),
        ]
        self.client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.parameter_timer = self.create_timer(1., self.poll_parameter)
        self.get_logger().info(
            f'SESSION VISION {"MAVROS" if self.native else "DEBUG"}: '
            'initialize disarmed; continue after arming; fixed geometry/origin/source IDs. '
            'No setpoints, mode changes or arming requests.')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def fresh(self, msg, limit):
        stamp = self.stamp(msg)
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp > 0 and -.05 <= age <= limit

    def warn(self, text):
        if time.monotonic() - self.last_warning > 3:
            self.get_logger().warning(text)
            self.last_warning = time.monotonic()

    def fail(self, text):
        if self.fault is None:
            self.fault = text
            self.get_logger().error(text + '; publishing latched off, restart after ground inspection')

    def state_received(self, msg):
        self.state, self.state_at = msg, time.monotonic()
        if self.fresh(msg, 2) and msg.armed and not self.ever_sent:
            self.fail('Arming before disarmed initialization completed')

    def state_ready(self):
        return (self.state is not None and self.state.connected
                and time.monotonic() - self.state_at <= 2 and self.fresh(self.state, 2))

    def poll_parameter(self):
        if self.future is not None and not self.future.done():
            if time.monotonic() - self.request_at > 2:
                self.future.cancel()
                self.future = None
                self.configuration_ok = False
            return
        if not self.client.service_is_ready():
            self.configuration_ok = False
            return
        request = GetParameters.Request(names=PARAMETERS)
        self.request_at = time.monotonic()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.parameter_result)

    def parameter_result(self, future):
        try:
            values = future.result().values
            self.configuration_ok = (
                len(values) == 6
                and values[0].type == 2 and values[0].integer_value == 11
                and values[1].type == 2 and values[1].integer_value == 3
                and all(v.type == 3 and math.isfinite(v.double_value) and abs(v.double_value) <= 1e-9
                        for v in values[2:5])
                and values[5].type == 2 and values[5].integer_value == 0)
            self.verified_at = time.monotonic()
        except Exception:
            self.configuration_ok = False

    def identities(self):
        ids = []
        for topic in ('/Odometry', '/super_debug/aligned_pose_cov'):
            publishers = self.get_publishers_info_by_topic(topic)
            if len(publishers) != 1:
                raise ValueError('Need one raw and one aligned localization source')
            ids.append(bytes(publishers[0].endpoint_gid))
        return tuple(ids)

    def raw_received(self, msg):
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
            self.warn('Unexpected raw odometry frames; no output')
            return
        self.store(self.raw, msg)

    def aligned_received(self, msg):
        if msg.header.frame_id != 'px4_enu_preview':
            self.warn('Unexpected aligned pose frame; no output')
            return
        self.store(self.aligned, msg)

    def store(self, bucket, msg):
        stamp = self.stamp(msg)
        if self.fault is not None:
            self.warn(self.fault)
            return
        if stamp <= self.last_stamp or not self.fresh(msg, .25):
            return
        bucket[stamp] = msg
        while len(bucket) > 24:
            del bucket[min(bucket)]
        if stamp in self.raw and stamp in self.aligned:
            raw, aligned = self.raw.pop(stamp), self.aligned.pop(stamp)
            self.process_pair(raw, aligned)

    def process_pair(self, raw, aligned):
        try:
            if not self.state_ready():
                raise ValueError('Waiting for fresh connected FC state')
            if not self.ever_sent and self.state.armed:
                self.fail('Disarmed initialization required')
                return
            if hashlib.sha256(self.path.read_bytes()).digest() != self.geometry_digest:
                self.fail('Active alignment file changed')
                return
            try:
                identities = self.identities()
            except ValueError:
                if self.source_ids is not None:
                    self.fail('Localization source count changed')
                    return
                raise
            if self.source_ids is None:
                self.source_ids = identities
            elif identities != self.source_ids:
                self.fail('Localization publisher identity changed')
                return
            if not self.fresh(raw, .25) or not self.fresh(aligned, .25):
                raise ValueError('Stale paired localization; no output')
            covariance = np.asarray(aligned.pose.covariance).reshape(6, 6)
            if (not np.all(np.isfinite(covariance))
                    or np.max(np.abs(covariance-covariance.T)) > 1e-8):
                raise ValueError('Invalid aligned covariance; no output')
            eigenvalues = np.linalg.eigvalsh(covariance)
            trace = np.trace(covariance)
            if (not np.all(np.isfinite(eigenvalues)) or eigenvalues.min() < -1e-10
                    or not math.isfinite(trace) or trace <= 0):
                raise ValueError('Invalid aligned covariance; no output')
            self.mapping.pair(raw.pose.pose, aligned.pose.pose)
            self.last_stamp = self.stamp(aligned)
            if self.mapping.pairs < 3:
                return
            if not self.configuration_ok or time.monotonic() - self.verified_at > 2:
                raise ValueError('Waiting for recent EV_CTRL=11, HGT_REF=3, zero EV_POS and QMIN=0')
            if self.native:
                if self.count_publishers(NATIVE_TOPIC) != 1:
                    raise ValueError('Need exactly one MAVROS vision writer; no output')
                if self.publisher.get_subscription_count() < 1:
                    raise ValueError('Waiting for MAVROS vision subscriber')
            out = PoseWithCovarianceStamped()
            out.header, out.pose = aligned.header, aligned.pose
            self.publisher.publish(out)
            self.ever_sent = True
            self.sent += 1
            if self.sent == 1:
                self.get_logger().info('Session locked; first pose forwarded with original stamp/covariance')
        except (ValueError, OSError, np.linalg.LinAlgError) as exc:
            if self.mapping.failed:
                self.fail(str(exc))
            else:
                self.warn(str(exc))


def main():
    rclpy.init()
    node = FlightPoseCovTransport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
data = source.encode('utf-8')
if hashlib.sha256(data).hexdigest() != '68cee4aa2aa4a00c1f66de3e3c4e08cfd82cc17ede07ed00fca67fce3cbc6e02':
    raise SystemExit('Source checksum mismatch; no file written.')
root = Path.home() / 'Downloads'
if not (root / 'control_bridge_preview.py').is_file() or not (root / 'aligned_pose_preview.py').is_file():
    raise SystemExit('Missing existing control/alignment helper; no file written.')
target = root / 'flight_pose_cov_transport.py'
compile(source, str(target), 'exec')
if target.exists() and target.read_bytes() == data:
    print('Already current:', target)
else:
    if target.exists():
        backup = Path(tempfile.mkdtemp(prefix='flight_pose_backup_', dir=str(root))) / target.name
        shutil.copy2(target, backup)
        print('Backup:', backup)
    target.write_bytes(data)
    print('Created:', target)
