#!/usr/bin/env python3
"""Read-only, stamp-paired attitude survey; no calibrated transform is saved."""
import argparse
import json
import math
from pathlib import Path
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


class FreshBodySamples:
    """Exclude old/future body poses from the estimate, retaining age evidence."""
    def __init__(self):
        self.rejected_age=0;self.observed_min=self.observed_max=None

    def receive(self,msg,now_ns):
        q=quaternion(msg.pose.orientation)
        if msg.header.frame_id!='camera_init' or q is None:return None
        ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age=(now_ns-ns)/1e9
        self.observed_min=age if self.observed_min is None else min(self.observed_min,age)
        self.observed_max=age if self.observed_max is None else max(self.observed_max,age)
        if ns<=0 or not -.05<=age<=.25:
            self.rejected_age+=1
            return None
        return stamp(msg),q,age

    def evidence(self):
        return dict(body_age_rejected=self.rejected_age,
                    observed_body_age_min_s=self.observed_min,
                    observed_body_age_max_s=self.observed_max,
                    body_sample_policy='Only original body stamps within -0.05..0.25 s enter pairing/estimation')


def summarize(rotations, gaps, ages, counts):
    report = dict(schema_version=1, received_body=counts[0], received_imu=counts[1],
                  matched=len(rotations), estimate_available=False)
    if len(rotations) < 20:
        return report
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
    report.update(estimate_available=True, quaternion_xyzw=q.tolist(),
                  rotation_rpy_deg=np.degrees(angles).tolist(),
                  rotation_spread_mean_deg=float(errors.mean()),
                  rotation_spread_max_deg=float(errors.max()),
                  pairing_gap_max_s=max(gaps), body_age_min_s=min(ages),
                  body_age_mean_s=float(np.mean(ages)), body_age_max_s=max(ages))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-json', type=Path, help='write a new machine-readable survey; never overwrite')
    parser.add_argument('--seconds', type=float, default=15., help='survey duration, 5 to 60 seconds')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 60:
        parser.error('--seconds must be between 5 and 60')
    if args.output_json is not None and (args.output_json.exists() or args.output_json.is_symlink()):
        parser.error('--output-json must name a new file')
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
    body_samples=FreshBodySamples()
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_imu(msg):
        counts[1] += 1
        q = quaternion(msg.orientation)
        if msg.orientation_covariance[0] != -1 and q is not None:
            imu.append((stamp(msg), q))

    def on_body(msg):
        counts[0] += 1
        sample=body_samples.receive(msg,node.get_clock().now().nanoseconds)
        if sample is not None:pending.append(sample)

    subscriptions = [
        node.create_subscription(PoseStamped, '/super_debug/body_pose', on_body, qos),
        node.create_subscription(Imu, '/mavros/imu/data', on_imu, qos)]
    try:
        start_stamp = node.get_clock().now().nanoseconds
        end = time.monotonic() + args.seconds
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
        report = summarize(rotations, gaps, ages, counts)
        report.update(body_samples.evidence())
        report.update(ros_start_stamp_ns=start_stamp,
                      ros_end_stamp_ns=node.get_clock().now().nanoseconds,
                      completed_monotonic_s=time.monotonic())
        if args.output_json is not None:
            with args.output_json.open('x') as handle:
                json.dump(report, handle, indent=2, allow_nan=False)
                handle.write('\n')
        print('Received body/imu:', *counts, 'Matched:', len(rotations))
        print('Body samples excluded by original-stamp age:',body_samples.rejected_age,
              '; observed body age min/max:',body_samples.observed_min,body_samples.observed_max)
        if not report['estimate_available']:
            print('Insufficient matched samples; no alignment estimate.')
            if args.output_json is not None:
                raise SystemExit(2)
            return
        q = np.array(report['quaternion_xyzw'])
        print('Candidate rotation quaternion xyzw:', q.round(6).tolist())
        print('Candidate rotation RPY degrees:', np.array(report['rotation_rpy_deg']).round(3).tolist())
        print(f'Rotation spread mean/max: {report["rotation_spread_mean_deg"]:.3f}/{report["rotation_spread_max_deg"]:.3f} deg')
        print(f'Stamp pairing gap max: {max(gaps):.3f}s; body age mean/max: '
              f'{np.mean(ages):.3f}/{max(ages):.3f}s')
        print('Inspection only: nominal mounting, no position origin/floor calibration.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
