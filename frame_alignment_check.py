#!/usr/bin/env python3
"""Read-only, stamp-paired attitude survey; no calibrated transform is saved."""
import math
import time
from collections import deque
import numpy as np


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                     w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def quaternion(q):
    v = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(v)
    return v / norm if np.all(np.isfinite(v)) and abs(norm-1) < .02 else None


def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('frame_alignment_check')
    imu = deque(maxlen=4000)
    pending = deque(maxlen=100)
    rotations, gaps, ages = [], [], []
    counts = [0, 0]
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_imu(msg):
        counts[1] += 1
        q = quaternion(msg.orientation)
        if msg.orientation_covariance[0] != -1 and q is not None:
            imu.append((stamp(msg), q))

    def on_body(msg):
        counts[0] += 1
        q = quaternion(msg.pose.orientation)
        if msg.header.frame_id == 'camera_init' and q is not None:
            age = node.get_clock().now().nanoseconds / 1e9 - stamp(msg)
            pending.append((stamp(msg), q, age))

    subscriptions = [
        node.create_subscription(PoseStamped, '/super_debug/body_pose', on_body, qos),
        node.create_subscription(Imu, '/mavros/imu/data', on_imu, qos)]
    try:
        end = time.monotonic() + 15
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            while pending and imu:
                t, q, age = pending[0]
                if t > imu[-1][0]:
                    break
                pending.popleft()
                nearest, q_px4 = min(imu, key=lambda item: abs(item[0]-t))
                if abs(nearest-t) <= .05:
                    rotations.append(multiply(q_px4, q * [-1, -1, -1, 1]))
                    gaps.append(abs(nearest-t))
                    ages.append(age)
        print('Received body/imu:', *counts, 'Matched:', len(rotations))
        if len(rotations) < 20:
            print('Insufficient matched samples; no alignment estimate.')
            return
        matrix = np.array(rotations)
        _, vectors = np.linalg.eigh(matrix.T @ matrix)
        q = vectors[:, -1]
        if q[3] < 0:
            q = -q
        errors = np.degrees(2*np.arccos(np.clip(np.abs(matrix @ q), 0, 1)))
        x, y, z, w = q
        angles = [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                  math.asin(float(np.clip(2*(w*y-z*x), -1, 1))),
                  math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
        print('Candidate rotation quaternion xyzw:', q.round(6).tolist())
        print('Candidate rotation RPY degrees:', np.degrees(angles).round(3).tolist())
        print(f'Rotation spread mean/max: {errors.mean():.3f}/{errors.max():.3f} deg')
        print(f'Stamp pairing gap max: {max(gaps):.3f}s; body age mean/max: '
              f'{np.mean(ages):.3f}/{max(ages):.3f}s')
        print('Inspection only: nominal mounting, no position origin/floor calibration.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
