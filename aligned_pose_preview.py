#!/usr/bin/env python3
"""Ground-only coordinate preview with original stamps; no MAVROS output."""
import math
import numpy as np


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])


def rotation(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


# Archived final October 7 survey ONLY. Re-measure and update after LIO restart.
Q_WC = np.array([0.000591, 0.018708, 0.58887, 0.808011])
Q_WC /= np.linalg.norm(Q_WC)
R_WC = rotation(Q_WC)
a = math.radians(45.) / 2
Q_BI = np.array([0., 0., math.sin(a), math.cos(a)])
Q_IB = Q_BI * [-1, -1, -1, 1]
# Nominal mounting; configured IMU-LiDAR offset, not live online extrinsics.
T_BI = np.array([0., 0., .10]) - rotation(Q_BI) @ np.array([-.011, -.02329, .04412])
AXES = np.zeros((6, 6))
AXES[:3, :3] = AXES[3:, 3:] = R_WC


def convert(p, q, covariance):
    q_cb = multiply(q, Q_IB)
    lever = -rotation(q_cb) @ T_BI
    position = R_WC @ (p + lever)
    orientation = multiply(Q_WC, q_cb)
    jacobian = np.eye(6)
    jacobian[:3, 3:] = -skew(lever)
    jacobian = AXES @ jacobian
    cov = jacobian @ covariance @ jacobian.T
    return position, orientation, (cov + cov.T) / 2


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped

    class Preview(Node):
        def __init__(self):
            super().__init__('aligned_pose_preview')
            self.origin = None
            self.last_stamp = None
            self.last_log = 0
            self.pub = self.create_publisher(
                PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', 10)
            self.sub = self.create_subscription(
                Odometry, '/Odometry', self.callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.get_logger().info(
                'GROUND PREVIEW: nominal mounting and preliminary world rotation. '
                'First accepted FC position is zero; no floor or venue calibration.')

        def callback(self, msg):
            if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
                return
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            if self.last_stamp is not None and stamp <= self.last_stamp:
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            p = np.array([p.x, p.y, p.z])
            q = np.array([q.x, q.y, q.z, q.w])
            cov = np.array(msg.pose.covariance).reshape(6, 6)
            if not all(np.all(np.isfinite(v)) for v in (p, q, cov)):
                return
            if abs(np.linalg.norm(q)-1) > .02:
                return
            if np.max(np.abs(cov-cov.T)) > 1e-8:
                return
            if np.linalg.eigvalsh(cov).min() < -1e-10 or np.trace(cov) <= 0:
                return
            p, q, cov = convert(p, q/np.linalg.norm(q), cov)
            if self.origin is None:
                self.origin = p.copy()
            out = PoseWithCovarianceStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = 'px4_enu_preview'
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, p-self.origin)
            out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = map(float, q)
            out.pose.covariance = cov.ravel().tolist()
            self.pub.publish(out)
            self.last_stamp = stamp
            now = self.get_clock().now().nanoseconds
            if now-self.last_log > 1000000000:
                age = (now-stamp)/1e9
                self.get_logger().info(
                    f'xyz={(p-self.origin).round(3).tolist()} age={age:.3f}s '
                    f'fresh={-.05 <= age <= .5}; preview only')
                self.last_log = now

    rclpy.init()
    node = Preview()
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
