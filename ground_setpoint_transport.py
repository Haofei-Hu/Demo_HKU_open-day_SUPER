#!/usr/bin/env python3
"""Disarmed AUTO.LOITER/POSCTL wire test, ENU position/yaw only; never use for flight."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State


def pose_values(msg):
    p, q = msg.pose.position, msg.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite pose')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if abs(norm - 1) > .02:
        raise ValueError('Invalid pose quaternion')
    x, y, z, w = [v / norm for v in values[3:]]
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return values[:3], yaw


class GroundSetpointTransport(Node):
    def __init__(self):
        super().__init__('ground_setpoint_transport')
        self.state = self.local = self.target = None
        self.state_at = self.local_at = self.target_at = -math.inf
        self.latched = False
        self.last_target_stamp = 0
        self.configuration_ok = False
        self.verified_at = -math.inf
        self.future = None
        self.last_warning = -math.inf
        self.sent = 0
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.inputs = [
            self.create_subscription(State, '/mavros/state', self.state_received, qos_profile_sensor_data),
            self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_received, qos_profile_sensor_data),
            self.create_subscription(PoseStamped, '/super_debug/control_target', self.target_received, qos_profile_sensor_data)]
        self.client = self.create_client(GetParameters, '/mavros/setpoint_position/get_parameters')
        self.parameter_timer = self.create_timer(1., self.poll_parameter)
        self.output_timer = self.create_timer(.05, self.send)
        self.get_logger().info('GROUND WIRE TEST: disarmed AUTO.LOITER/POSCTL, 20 Hz position/yaw; no mode or arming requests. Arming/OFFBOARD latches output off.')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def fresh(self, msg, receipt, limit):
        if msg is None or time.monotonic() - receipt > limit:
            return False
        stamp = self.stamp(msg)
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp > 0 and -.05 <= age <= limit

    def state_received(self, msg):
        self.latched = self.latched or msg.armed or msg.mode == 'OFFBOARD'
        self.state, self.state_at = msg, time.monotonic()

    def local_received(self, msg):
        self.local, self.local_at = msg, time.monotonic()

    def target_received(self, msg):
        stamp = self.stamp(msg)
        if stamp <= self.last_target_stamp or not self.fresh(msg, time.monotonic(), .25):
            self.target = None
            return
        self.last_target_stamp = stamp
        self.target, self.target_at = msg, time.monotonic()

    def warn(self, reason):
        if time.monotonic() - self.last_warning > 3:
            self.get_logger().warning(reason)
            self.last_warning = time.monotonic()

    def poll_parameter(self):
        if self.future is not None and not self.future.done():
            if time.monotonic() - self.request_at > 2:
                self.future.cancel()
                self.configuration_ok = False
            return
        if not self.client.service_is_ready():
            self.configuration_ok = False
            return
        request = GetParameters.Request()
        request.names = ['mav_frame']
        self.request_at = time.monotonic()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.parameter_result)

    def parameter_result(self, future):
        try:
            response = future.result()
            self.configuration_ok = (response is not None and len(response.values) == 1
                                     and response.values[0].type == 4
                                     and response.values[0].string_value == 'LOCAL_NED')
            self.verified_at = time.monotonic()
        except Exception:
            self.configuration_ok = False

    def send(self):
        try:
            if self.latched:
                raise ValueError('BLOCKED: arming/OFFBOARD observed; restart required')
            if not self.configuration_ok or time.monotonic() - self.verified_at > 2:
                raise ValueError('Waiting for recent MAVROS mav_frame=LOCAL_NED confirmation')
            if not self.fresh(self.state, self.state_at, 2):
                raise ValueError('Waiting for fresh FC state')
            if not self.state.connected or self.state.armed or self.state.mode not in ('AUTO.LOITER', 'POSCTL'):
                raise ValueError('Ground test requires connected, disarmed AUTO.LOITER/POSCTL')
            if not self.fresh(self.local, self.local_at, .25) or not self.fresh(self.target, self.target_at, .25):
                raise ValueError('Waiting for fresh local pose and debug target')
            if self.local.header.frame_id != 'map' or self.target.header.frame_id != 'px4_enu_preview':
                raise ValueError('Unexpected local/preview frame')
            p, yaw = pose_values(self.target)
            local, local_yaw = pose_values(self.local)
            if math.dist(p, local) > 1.:
                raise ValueError('Target exceeds 1 m ground-test radius from FC pose')
            if abs(math.atan2(math.sin(yaw-local_yaw), math.cos(yaw-local_yaw))) > math.radians(30):
                raise ValueError('Target exceeds 30 degree ground-test heading difference')
            if (self.count_publishers('/mavros/setpoint_position/local') != 1 or self.publisher.get_subscription_count() < 1
                    or self.count_publishers('/super_debug/control_target') != 1):
                raise ValueError('Need one target source, one setpoint publisher and a MAVROS subscriber')
            out = PoseStamped()
            out.header.stamp = self.target.header.stamp
            out.header.frame_id = 'map'
            out.pose.position.x, out.pose.position.y, out.pose.position.z = p
            out.pose.orientation.z, out.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.publisher.publish(out)
            self.sent += 1
            if self.sent == 1 or self.sent % 20 == 0:
                self.get_logger().info(f'SENT ENU xyz={[round(v, 3) for v in p]} yaw={math.degrees(yaw):.1f} deg; ground only')
        except ValueError as exc:
            self.warn(str(exc))


def main():
    rclpy.init()
    node = GroundSetpointTransport()
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
