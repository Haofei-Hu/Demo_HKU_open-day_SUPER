#!/usr/bin/env python3
"""Synthetic ROS DDS test; dedicated domain, no aircraft hardware."""
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State
from ground_xy_fusion_transport import GroundXYFusionTransport


def main(transport_type=GroundXYFusionTransport, height_enabled=False):
    rclpy.init()
    transport = transport_type()
    parameters = Node('param', namespace='/mavros')
    for name, value in [('EKF2_EV_CTRL', 0), ('EKF2_EV_POS_X', 0.0),
                        ('EKF2_EV_POS_Y', 0.0), ('EKF2_EV_POS_Z', 0.0),
                        ('EKF2_EV_QMIN', 0)]:
        parameters.declare_parameter(name, value)
    probe = Node('ground_xy_test')
    publisher = probe.create_publisher(
        PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', 10)
    state_publisher = probe.create_publisher(State, '/mavros/state', 10)
    received = []
    subscription = probe.create_subscription(
        PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', received.append, 10)
    state_settings = {'connected': True, 'armed': False, 'age': 0.0}

    def publish_state():
        msg = State()
        ns = probe.get_clock().now().nanoseconds - int(state_settings['age'] * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        msg.connected = state_settings['connected']
        msg.armed = state_settings['armed']
        msg.mode = 'AUTO.LOITER'
        state_publisher.publish(msg)

    state_timer = probe.create_timer(.1, publish_state)
    executor = SingleThreadedExecutor()
    for node in [transport, parameters, probe]:
        executor.add_node(node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)

    def wait_for(predicate):
        end = time.monotonic() + 10
        while not predicate():
            assert time.monotonic() < end, 'Timed out'
            spin(.05)

    def message():
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_enu_preview'
        msg.pose.pose.position.x = .2
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [.01 if i == j else 0.0
                               for i in range(6) for j in range(6)]
        return msg

    def reject(msg):
        count = len(received)
        publisher.publish(msg)
        spin(.15)
        assert len(received) == count, 'Rejected input produced output'

    def accept():
        original = message()
        count = len(received)
        publisher.publish(original)
        spin(.15)
        assert len(received) == count + 1, 'Valid input blocked'
        assert received[-1].header == original.header
        assert received[-1].pose == original.pose
        return original

    def set_parameter(name, value):
        result = parameters.set_parameters([Parameter(name, value=value)])
        assert result[0].successful

    try:
        wait_for(lambda: publisher.get_subscription_count() == 1
                 and transport.publisher.get_subscription_count() == 1
                 and transport.configuration_ok and transport.connected_disarmed)
        original = accept()
        reject(original)
        for case in ['stale', 'future', 'frame', 'position', 'quaternion', 'covariance']:
            msg = message()
            if case == 'stale':
                msg.header.stamp.sec -= 1
            elif case == 'future':
                msg.header.stamp.sec += 1
            elif case == 'frame':
                msg.header.frame_id = 'camera_init'
            elif case == 'position':
                msg.pose.pose.position.x = math.nan
            elif case == 'quaternion':
                msg.pose.pose.orientation.w = 2.0
            elif case == 'covariance':
                msg.pose.covariance[0] = -1.0
            reject(msg)
        set_parameter('EKF2_EV_CTRL', 1)
        spin(1.2)
        assert transport.configuration_ok
        accept()  # actual intended XY-only fusion setting remains supplied
        set_parameter('EKF2_EV_CTRL', 9)
        spin(1.2)
        assert transport.configuration_ok
        accept()  # XY + vision yaw can initialize heading without a magnetometer
        if height_enabled:
            set_parameter('EKF2_EV_CTRL', 11)
            spin(1.2)
            assert transport.configuration_ok
            original_height = message()
            original_height.pose.pose.position.z = .2
            count = len(received)
            publisher.publish(original_height)
            spin(.15)
            assert len(received) == count + 1
            assert received[-1].header == original_height.header
            assert received[-1].pose == original_height.pose
        for mask in [2, 3, 4, 8, 15, -1] + ([] if height_enabled else [11]):
            set_parameter('EKF2_EV_CTRL', mask)
            wait_for(lambda: not transport.configuration_ok)
            reject(message())
            set_parameter('EKF2_EV_CTRL', 1)
            wait_for(lambda: transport.configuration_ok)
        for name, bad, good in [('EKF2_EV_POS_X', .1, 0.0),
                                ('EKF2_EV_POS_Y', .1, 0.0),
                                ('EKF2_EV_POS_Z', .1, 0.0),
                                ('EKF2_EV_QMIN', 1, 0)]:
            set_parameter(name, bad)
            wait_for(lambda: not transport.configuration_ok)
            reject(message())
            set_parameter(name, good)
            wait_for(lambda: transport.configuration_ok)
        transport.timer.cancel()
        transport.verified_at = time.monotonic() - 3
        reject(message())
        transport.timer.reset()
        wait_for(lambda: time.monotonic() - transport.verified_at < 1)
        accept()
        state_settings['connected'] = False
        wait_for(lambda: not transport.connected_disarmed)
        reject(message())
        state_settings['connected'] = True
        state_settings['age'] = 3
        spin(.3)
        reject(message())  # stale header despite fresh ROS receipt
        state_settings['age'] = 0
        wait_for(lambda: transport.connected_disarmed)
        state_timer.cancel()
        transport.state_received_at = time.monotonic() - 3
        reject(message())
        state_timer.reset()
        wait_for(lambda: time.monotonic() - transport.state_received_at < .2)
        accept()
        # A delayed state can expire by header age before its ROS receipt expires.
        state_settings['age'] = 1.8
        wait_for(lambda: transport.connected_disarmed
                 and 1.7 <= (probe.get_clock().now().nanoseconds - transport.state_stamp) / 1e9 <= 1.95)
        state_timer.cancel()
        spin(.35)
        assert time.monotonic() - transport.state_received_at < 2
        assert (probe.get_clock().now().nanoseconds - transport.state_stamp) / 1e9 > 2
        reject(message())
        state_settings['age'] = 0
        state_timer.reset()
        wait_for(lambda: time.monotonic() - transport.state_received_at < .2
                 and (probe.get_clock().now().nanoseconds - transport.state_stamp) / 1e9 < .2)
        accept()
        state_settings['armed'] = True
        wait_for(lambda: transport.arm_seen)
        reject(message())
        state_settings['armed'] = False
        spin(.3)
        reject(message())  # latches off after arming, even if later disarmed
        masks = '0/1/9/11' if height_enabled else '0/1/9'
        print(f'PASS: preserved fields, input rejection, EV_CTRL {masks} only, '
              'EV offsets/quality gate, stale confirmation/state (including cumulative header age) and disconnect '
              'blocks, arming latches ground relay off; no parameter writes/setpoints.')
    finally:
        executor.shutdown()
        for node in [transport, parameters, probe]:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
