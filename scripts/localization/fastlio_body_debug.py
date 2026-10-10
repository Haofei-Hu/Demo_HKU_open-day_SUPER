#!/usr/bin/env python3
"""Nominal mounting transform for inspection only; no MAVROS or TF output."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])


def rotate(q, v):
    xyz = q[:3]
    return v + 2*np.cross(xyz, np.cross(xyz, v) + q[3]*v)


# B: nominal forward-left-up body frame at the FC centre.
# I: FAST_LIO IMU frame. L: nominal LiDAR frame/origin.
# R_IL is assumed identity from the configuration, NOT the online estimate.
angle = math.radians(45.0)
Q_BI = np.array([0., 0., math.sin(angle/2), math.cos(angle/2)])
Q_IB = Q_BI * np.array([-1., -1., -1., 1.])
T_BL = np.array([0., 0., 0.10])
T_IL = np.array([-0.011, -0.02329, 0.04412])
T_BI = T_BL - rotate(Q_BI, T_IL)


def transform(p_wi, q_wi):
    q_wb = multiply(q_wi, Q_IB)
    return p_wi - rotate(q_wb, T_BI), q_wb


class BodyDebug(Node):
    def __init__(self):
        super().__init__('fastlio_body_debug')
        self.pub = self.create_publisher(PoseStamped, '/super_debug/body_pose', 10)
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.callback, qos_profile_sensor_data)
        self.last_log = 0
        self.get_logger().info(
            'DEBUG ONLY: nominal +45deg yaw, LiDAR 0.10m above FC; '
            'factory IMU offset, no live extrinsic estimate. '
            'Output /super_debug/body_pose; original frame/time retained.')

    def callback(self, msg):
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v = np.array([p.x, p.y, p.z])
        quat = np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(quat)
        if not np.all(np.isfinite(v)) or not np.isfinite(norm) or norm < 1e-8:
            return
        v, quat = transform(v, quat/norm)
        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, v)
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = map(float, quat)
        self.pub.publish(out)
        now = self.get_clock().now().nanoseconds
        if now-self.last_log > 2_000_000_000:
            self.last_log = now
            x, y, z, w = quat
            roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
            pitch = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            self.get_logger().info(
                f'body xyz={v.round(3).tolist()} '
                f'rpy_deg={np.degrees([roll, pitch, yaw]).round(2).tolist()}')


def main():
    rclpy.init()
    node = BodyDebug()
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
