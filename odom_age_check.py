#!/usr/bin/env python3
"""Measure /Odometry timestamp age for 15 seconds using a depth-one subscriber."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


def main():
    rclpy.init()
    node = Node('odom_age_check')
    ages = []

    def callback(msg):
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        ages.append((node.get_clock().now().nanoseconds - stamp) / 1e9)

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscription = node.create_subscription(
        Odometry, '/Odometry', callback, qos)
    try:
        end = time.monotonic() + 15
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if ages:
            print(f'/Odometry: n={len(ages)}, age min/avg/max='
                  f'{min(ages):.3f}/{sum(ages)/len(ages):.3f}/'
                  f'{max(ages):.3f} seconds')
        else:
            print('No Odometry received')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
