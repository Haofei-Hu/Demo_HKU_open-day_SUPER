#!/usr/bin/env python3
"""Disarmed floor-referenced hover preview. Publishes only a debug target.

Place the aircraft on its actual takeoff floor before starting. Keep the current
localization session. No MAVROS setpoint, arming or mode-change output exists.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State

from hover_reference import HoverReference
from ground_setpoint_transport import pose_values


LIMITS = {
    'MPC_XY_VEL_ALL': -1.0, 'MPC_Z_VEL_ALL': -1.0,
    'MPC_XY_VEL_MAX': .5, 'MPC_Z_VEL_MAX_UP': .5, 'MPC_Z_VEL_MAX_DN': .5,
    'COM_OF_LOSS_T': .5, 'COM_OBL_RC_ACT': 4, 'EKF2_EV_CTRL': 11, 'EKF2_HGT_REF': 3,
}
OUTPUT = '/super_debug/first_hover_target'


class FirstHoverPreview(Node):
    def __init__(self):
        super().__init__('first_hover_preview')
        self.state = self.local = self.vision = None
        self.receipts = {}
        self.reference = None
        self.started_at = None
        self.source_ids = None
        self.fault = None
        self.configuration_ok = False
        self.verified_at = -math.inf
        self.future = None
        self.request_at = -math.inf
        self.last_warning = self.last_report = -math.inf
        self.sent = 0
        self.publisher = self.create_publisher(PoseStamped, OUTPUT, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.inputs = [
            self.create_subscription(State, '/mavros/state', self.state_received, qos_profile_sensor_data),
            self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_received, qos),
            self.create_subscription(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov',
                                     self.vision_received, qos),
        ]
        self.client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.parameter_timer = self.create_timer(1., self.poll_parameters)
        self.output_timer = self.create_timer(.05, self.publish_preview)
        self.get_logger().info(
            'GROUND HOVER PREVIEW ONLY: floor FC height 0.085 m, target 0.5 m above floor; '
            'fixed XY/yaw, smooth up/hold/down, then ground hold. No arming/mode/MAVROS output.')

    def store(self, field, msg):
        setattr(self, field, msg)
        self.receipts[field] = time.monotonic()

    def state_received(self, msg):
        self.store('state', msg)
        if msg.armed or msg.mode == 'OFFBOARD':
            self.fail('Arming/OFFBOARD observed: ground preview latched off')

    def local_received(self, msg):
        self.store('local', msg)

    def vision_received(self, msg):
        self.store('vision', msg)

    def fail(self, reason):
        if self.fault is None:
            self.fault = reason
            self.get_logger().error(reason)

    def fresh(self, field, limit):
        msg = getattr(self, field)
        if msg is None or time.monotonic() - self.receipts.get(field, -math.inf) > limit:
            return False
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp > 0 and -.05 <= age <= limit

    def poll_parameters(self):
        if self.future is not None and not self.future.done():
            if time.monotonic() - self.request_at > 2:
                self.future.cancel()
                self.configuration_ok = False
            return
        if not self.client.service_is_ready():
            self.configuration_ok = False
            return
        self.request_at = time.monotonic()
        self.future = self.client.call_async(GetParameters.Request(names=list(LIMITS)))
        self.future.add_done_callback(self.parameter_result)

    def parameter_result(self, future):
        try:
            values = future.result().values
            self.configuration_ok = len(values) == len(LIMITS)
            for value, expected in zip(values, LIMITS.values()):
                number = value.integer_value if isinstance(expected, int) else value.double_value
                expected_type = 2 if isinstance(expected, int) else 3
                self.configuration_ok &= (value.type == expected_type and math.isfinite(number)
                                          and abs(number - expected) <= 1e-6)
            self.verified_at = time.monotonic()
        except Exception:
            self.configuration_ok = False

    def identities(self):
        result = []
        for topic in ('/mavros/local_position/pose', '/mavros/vision_pose/pose_cov'):
            publishers = self.get_publishers_info_by_topic(topic)
            if len(publishers) != 1:
                raise ValueError('Need exactly one local pose and one session vision source')
            result.append(bytes(publishers[0].endpoint_gid))
        return tuple(result)

    def validate(self):
        if self.fault is not None:
            raise ValueError(self.fault)
        if not self.configuration_ok or time.monotonic() - self.verified_at > 2:
            raise ValueError('Waiting for recent low-speed/fusion/fallback parameter confirmation')
        if not self.fresh('state', 2) or not self.state.connected or self.state.mode != 'POSCTL':
            raise ValueError('Need fresh connected/disarmed POSCTL')
        if self.state.armed:
            raise ValueError('Ground preview requires disarmed aircraft')
        if not self.fresh('local', .25) or not self.fresh('vision', .25):
            raise ValueError('Waiting for fresh FC pose and session vision')
        if self.local.header.frame_id != 'map' or self.vision.header.frame_id != 'px4_enu_preview':
            raise ValueError('Unexpected local/vision frame')
        local, yaw = pose_values(self.local)
        vision = PoseStamped(pose=self.vision.pose.pose)
        external, external_yaw = pose_values(vision)
        if math.dist(local, external) > .1:
            raise ValueError('FC/vision positions differ by more than 0.1 m')
        if abs(math.atan2(math.sin(yaw - external_yaw), math.cos(yaw - external_yaw))) > math.radians(3):
            raise ValueError('FC/vision headings differ by more than 3 degrees')
        identities = self.identities()
        if self.source_ids is not None and identities != self.source_ids:
            raise ValueError('Localization publisher identities changed')
        if self.reference is not None and math.dist(local, self.reference.ground) > .1:
            raise ValueError('Aircraft moved from captured ground position during preview')
        if self.count_publishers(OUTPUT) != 1:
            raise ValueError('Duplicate hover preview writer')
        return local, yaw, identities

    def publish_preview(self):
        try:
            local, yaw, identities = self.validate()
            now = time.monotonic()
            if self.reference is None:
                self.reference = HoverReference(local, yaw)
                self.source_ids, self.started_at = identities, now
                self.get_logger().info(
                    f'Captured ground FC xyz={local}; hover target z={local[2] + self.reference.lift:.3f}; '
                    f'reference duration={self.reference.duration:.3f}s. DEBUG only.')
            point, yaw, phase = self.reference.at(now - self.started_at)
            out = PoseStamped()
            out.header.stamp = self.get_clock().now().to_msg()  # new reference command, not a retimed measurement
            out.header.frame_id = 'px4_enu_preview'
            out.pose.position.x, out.pose.position.y, out.pose.position.z = point
            out.pose.orientation.z, out.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.publisher.publish(out)
            self.sent += 1
            if now - self.last_report >= 1:
                self.last_report = now
                self.get_logger().info(f'{phase} xyz={[round(v, 3) for v in point]} DEBUG only')
        except ValueError as exc:
            if self.reference is not None:
                self.fail('Ground preview stopped: ' + str(exc))
            elif time.monotonic() - self.last_warning > 3:
                self.last_warning = time.monotonic()
                self.get_logger().warning(str(exc))


def main():
    rclpy.init()
    node = FirstHoverPreview()
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
