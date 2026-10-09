#!/usr/bin/env python3
"""Disarmed ground XYZ-fusion test only; never use for flight.

Preserve the preview pose/covariance/frame/stamp. Require recent FC state,
EV_CTRL in {0,1,9,11}, zero FC-centred EV lever arm and EV_QMIN=0. No parameter
writes, arming, mode changes or setpoints. Stop before source/preview resets.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State

class GroundHeightFusionTransport(Node):

    def __init__(self):
        super().__init__('ground_height_fusion_transport')
        self.declare_parameter('max_age_seconds', 0.25)
        self.max_age = float(self.get_parameter('max_age_seconds').value)
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_age_seconds must be finite and positive')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', self.receive, qos_profile_sensor_data)
        self.client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.future = None
        self.configuration_ok = False
        self.verified_at = -math.inf
        self.last_input_stamp = None
        self.sent = 0
        self.last_warning = -math.inf
        self.connected_disarmed = False
        self.state_received_at = -math.inf
        self.state_stamp = 0
        self.arm_seen = False
        self.state_subscription = self.create_subscription(State, '/mavros/state', self.state_received, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0, self.poll_parameter)
        self.get_logger().info('DISARMED GROUND XYZ/YAW TEST ONLY: EV_CTRL 0/1/9/11, zero EV_POS and QMIN=0. Original stamps retained; no flight/setpoint output.')

    def state_received(self, msg):
        self.state_received_at = time.monotonic()
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.state_stamp = stamp
        age = (self.get_clock().now().nanoseconds - stamp) / 1000000000.0
        self.arm_seen = self.arm_seen or msg.armed
        self.connected_disarmed = msg.connected and (not msg.armed) and (stamp > 0) and (-0.05 <= age <= 2)

    def warn(self, reason):
        now = time.monotonic()
        if now - self.last_warning >= 3:
            self.get_logger().warning(reason)
            self.last_warning = now

    def poll_parameter(self):
        if self.future is not None and (not self.future.done()):
            if time.monotonic() - self.request_at > 2:
                self.future.cancel()
                self.future = None
                self.configuration_ok = False
            return
        if not self.client.service_is_ready():
            self.configuration_ok = False
            self.warn('MAVROS parameter service unavailable; publishing blocked')
            return
        request = GetParameters.Request()
        request.names = ['EKF2_EV_CTRL', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z', 'EKF2_EV_QMIN']
        self.request_at = time.monotonic()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.parameter_result)

    def parameter_result(self, future):
        try:
            response = future.result()
            self.configuration_ok = response is not None and len(response.values) == 5 and (response.values[0].type == 2) and (response.values[0].integer_value in (0, 1, 9, 11)) and all((v.type == 3 and math.isfinite(v.double_value) and (abs(v.double_value) <= 1e-09) for v in response.values[1:4])) and (response.values[4].type == 2) and (response.values[4].integer_value == 0)
            self.verified_at = time.monotonic()
        except Exception:
            self.configuration_ok = False
        if not self.configuration_ok:
            self.warn('Ground XYZ parameter configuration not confirmed; publishing blocked')

    def receive(self, msg):
        if self.arm_seen:
            self.warn('Arming observed: ground relay latched off; no flight support')
            return
        state_age = (self.get_clock().now().nanoseconds - self.state_stamp) / 1000000000.0
        if not self.connected_disarmed or time.monotonic() - self.state_received_at > 2 or (not -0.05 <= state_age <= 2):
            self.warn('Awaiting fresh connected/disarmed FC state; publishing blocked')
            return
        if not self.configuration_ok or time.monotonic() - self.verified_at > 2:
            self.warn('Awaiting recent ground XYZ parameter confirmation; publishing blocked')
            return
        if msg.header.frame_id != 'px4_enu_preview':
            self.warn('Unexpected preview frame name; input rejected')
            return
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1000000000.0
        if stamp <= 0 or age < -0.05 or age > self.max_age:
            self.warn(f'Input stamp age {age:.3f}s outside allowed range; rejected')
            return
        if self.last_input_stamp is not None and stamp <= self.last_input_stamp:
            self.warn('Repeated/backwards stamp; restart this check after a source reset')
            return
        p, q = (msg.pose.pose.position, msg.pose.pose.orientation)
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *msg.pose.covariance]
        if not all((math.isfinite(v) for v in values)):
            self.warn('Nonfinite pose/covariance; input rejected')
            return
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 0.01:
            self.warn('Quaternion is not unit length; input rejected')
            return
        if any((msg.pose.covariance[i * 6 + i] < 0 for i in range(6))):
            self.warn('Negative covariance diagonal; input rejected')
            return
        if not any((msg.pose.covariance[i * 6 + i] > 0 for i in range(6))):
            self.warn('No positive covariance diagonal; input rejected')
            return
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose = msg.pose
        self.publisher.publish(out)
        self.last_input_stamp = stamp
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info('First aligned pose/covariance forwarded with original frame and stamp. Check PX4 estimator aid-source status to verify fusion.')

def main():
    rclpy.init()
    node = GroundHeightFusionTransport()
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
