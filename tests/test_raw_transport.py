#!/usr/bin/env python3
"""Manual ROS DDS check; run in a dedicated domain without hardware nodes."""
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from raw_pose_cov_transport import RawPoseCovTransport


def main():
    rclpy.init()
    transport = RawPoseCovTransport()
    fake_mavros = Node('param', namespace='/mavros')
    fake_mavros.declare_parameter('EKF2_EV_CTRL', 0)
    probe = Node('transport_test_probe')
    publisher = probe.create_publisher(Odometry, '/Odometry', qos_profile_sensor_data)
    received = []
    subscription = probe.create_subscription(
        PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', received.append, 10)
    executor = SingleThreadedExecutor()
    for node in [transport, fake_mavros, probe]:
        executor.add_node(node)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait_for(predicate):
        deadline = time.monotonic() + 10
        while not predicate():
            assert time.monotonic() < deadline, 'Timed out waiting for DDS/service discovery'
            spin(0.05)

    def message():
        msg = Odometry()
        msg.header.frame_id = 'camera_init'
        msg.child_frame_id = 'body'
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.pose.pose.position.x = 1.25
        msg.pose.pose.position.y = -2.5
        msg.pose.pose.position.z = 0.75
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [float(i+1)/100 if i == j else 0.0
                               for i in range(6) for j in range(6)]
        return msg

    def reject(msg):
        count = len(received)
        publisher.publish(msg)
        spin(0.2)
        assert len(received) == count, 'Rejected input incorrectly produced output'

    try:
        wait_for(lambda: publisher.get_subscription_count() == 1
                 and transport.publisher.get_subscription_count() == 1
                 and transport.ev_zero)
        original = message()
        publisher.publish(original)
        spin(0.2)
        assert len(received) == 1
        assert received[0].header == original.header
        assert received[0].pose == original.pose
        reject(original)  # repeated stamp
        for case in ['stale', 'future', 'nonfinite', 'wrong_frame', 'bad_quaternion',
                     'negative_covariance', 'zero_covariance']:
            msg = message()
            if case == 'stale':
                msg.header.stamp.sec -= 2
            elif case == 'future':
                msg.header.stamp.sec += 2
            elif case == 'nonfinite':
                msg.pose.pose.position.x = math.nan
            elif case == 'wrong_frame':
                msg.child_frame_id = 'unexpected'
            elif case == 'bad_quaternion':
                msg.pose.pose.orientation.w = 2.0
            elif case == 'negative_covariance':
                msg.pose.covariance[0] = -0.01
            elif case == 'zero_covariance':
                msg.pose.covariance = [0.0] * 36
            reject(msg)
        fake_mavros.set_parameters([Parameter('EKF2_EV_CTRL', value=1)])
        wait_for(lambda: not transport.ev_zero)
        reject(message())
        fake_mavros.set_parameters([Parameter('EKF2_EV_CTRL', value=0)])
        wait_for(lambda: transport.ev_zero)
        # A fresh read with an unavailable service cannot keep the gate open.
        transport.timer.cancel()
        transport.verified_at = time.monotonic() - 3
        reject(message())
        transport.timer.reset()
        wait_for(lambda: time.monotonic() - transport.verified_at < 1)
        publisher.publish(message())
        spin(0.2)
        assert len(received) == 2
        print('PASS: actual ROS DDS transport preserves pose, covariance, frame and stamp; '
              'eight invalid-input cases rejected; nonzero/stale parameter gate blocked; '
              'fresh zero parameter restored transport')
    finally:
        executor.shutdown()
        for node in [transport, fake_mavros, probe]:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
