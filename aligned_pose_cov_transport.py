#!/usr/bin/env python3
"""Ground transport check only. This is NOT a flight localization bridge.

Copy /super_debug/aligned_pose_cov.pose to /mavros/vision_pose/pose_cov without coordinate,
origin, mounting, covariance or timestamp changes. Require a recent read
of EKF2_EV_CTRL == 0. Never enable fusion while using this script.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class AlignedPoseCovTransport(Node):
    def __init__(self):
        super().__init__('aligned_pose_cov_transport')
        self.declare_parameter('max_age_seconds', 0.5)
        self.max_age = float(self.get_parameter('max_age_seconds').value)
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_age_seconds must be finite and positive')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', self.receive, qos_profile_sensor_data)
        self.client = self.create_client(
            GetParameters, '/mavros/param/get_parameters')
        self.future = None
        self.ev_zero = False
        self.verified_at = -math.inf
        self.last_input_stamp = None
        self.sent = 0
        self.last_warning = -math.inf
        self.timer = self.create_timer(1.0, self.poll_parameter)
        self.get_logger().info(
            'GROUND TRANSPORT: aligned FC-centre pose/covariance, preview origin; '
            'preliminary calibration. Waiting for EKF2_EV_CTRL=0.')

    def warn(self, reason):
        now = time.monotonic()
        if now - self.last_warning >= 3:
            self.get_logger().warning(reason)
            self.last_warning = now

    def poll_parameter(self):
        if self.future is not None and not self.future.done():
            if time.monotonic() - self.request_at > 2:
                self.future.cancel()
                self.future = None
                self.ev_zero = False
            return
        if not self.client.service_is_ready():
            self.ev_zero = False
            self.warn('MAVROS parameter service unavailable; publishing blocked')
            return
        request = GetParameters.Request()
        request.names = ['EKF2_EV_CTRL']
        self.request_at = time.monotonic()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.parameter_result)

    def parameter_result(self, future):
        try:
            response = future.result()
            self.ev_zero = (
                response is not None and len(response.values) == 1
                and response.values[0].type == 2
                and response.values[0].integer_value == 0)
            self.verified_at = time.monotonic()
        except Exception:
            self.ev_zero = False
        if not self.ev_zero:
            self.warn('EKF2_EV_CTRL is not confirmed integer 0; publishing blocked')

    def receive(self, msg):
        if not self.ev_zero or time.monotonic() - self.verified_at > 2:
            self.warn('Awaiting recent EKF2_EV_CTRL=0 confirmation; publishing blocked')
            return
        if msg.header.frame_id != 'px4_enu_preview':
            self.warn('Unexpected preview frame name; input rejected')
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or age < -0.05 or age > self.max_age:
            self.warn(f'Input stamp age {age:.3f}s outside allowed range; rejected')
            return
        if self.last_input_stamp is not None and stamp <= self.last_input_stamp:
            self.warn('Repeated/backwards stamp; restart this check after a source reset')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *msg.pose.covariance]
        if not all(math.isfinite(v) for v in values):
            self.warn('Nonfinite pose/covariance; input rejected')
            return
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
            self.warn('Quaternion is not unit length; input rejected')
            return
        if any(msg.pose.covariance[i*6+i] < 0 for i in range(6)):
            self.warn('Negative covariance diagonal; input rejected')
            return
        if not any(msg.pose.covariance[i*6+i] > 0 for i in range(6)):
            self.warn('No positive covariance diagonal; input rejected')
            return
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose = msg.pose
        self.publisher.publish(out)
        self.last_input_stamp = stamp
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info(
                'First aligned pose/covariance forwarded with original frame and stamp. '
                'This confirms ROS transport only, not PX4 fusion.')


def main():
    rclpy.init()
    node = AlignedPoseCovTransport()
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
