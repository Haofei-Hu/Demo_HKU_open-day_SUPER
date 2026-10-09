#!/usr/bin/env python3
"""Read-only 10-second freshness check of the physical control-preview chain."""
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mars_quadrotor_msgs.msg import PositionCommand


def main():
    rclpy.init()
    node = rclpy.create_node('control_chain_age_check')
    topics = [('/Odometry', Odometry), ('/planning/pos_cmd', PositionCommand),
              ('/super_debug/control_target', PoseStamped),
              ('/mavros/local_position/pose', PoseStamped)]
    samples = {topic: [] for topic, _ in topics}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def record(msg, topic):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds-stamp) / 1e9
        samples[topic].append((time.monotonic(), age))

    subscriptions = [node.create_subscription(
        kind, topic, lambda msg, name=topic: record(msg, name), qos)
        for topic, kind in topics]
    try:
        end = time.monotonic() + 10
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        for topic, _ in topics:
            values = samples[topic]
            print(topic)
            if not values:
                print('  No messages received')
                continue
            ages = [age for _, age in values]
            gaps = [values[i][0]-values[i-1][0] for i in range(1, len(values))]
            print(f'  n={len(values)}, age min/avg/max='
                  f'{min(ages):.3f}/{sum(ages)/len(ages):.3f}/{max(ages):.3f}s')
            print(f'  receive gap max={max(gaps, default=0):.3f}s, '
                  f'last receive age={time.monotonic()-values[-1][0]:.3f}s')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
