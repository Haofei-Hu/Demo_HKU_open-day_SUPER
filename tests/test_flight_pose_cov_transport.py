#!/usr/bin/env python3
"""Isolated ROS graph checks for session vision. No aircraft or real arming."""
import math
from pathlib import Path
import shutil
import tempfile
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State

from control_bridge_preview import load_converter
from flight_pose_cov_transport import FlightPoseCovTransport, DEBUG_TOPIC, NATIVE_TOPIC, PARAMETERS


def main():
    rclpy.init()
    directory = Path(tempfile.mkdtemp(prefix='session_vision_test_'))
    path = directory / 'alignment.py'
    source = Path(__file__).resolve().parents[1] / 'aligned_pose_preview.py'
    shutil.copy2(source, path)
    convert = load_converter(path)
    probe = Node('session_vision_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    bridge = None
    state_pub = probe.create_publisher(State, '/mavros/state', 10)
    raw_pub = probe.create_publisher(Odometry, '/Odometry', 10)
    aligned_pub = probe.create_publisher(PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', 10)
    debug, native, inputs = [], [], []
    subscriptions = [
        probe.create_subscription(PoseWithCovarianceStamped, DEBUG_TOPIC, debug.append, 10),
        probe.create_subscription(PoseWithCovarianceStamped, NATIVE_TOPIC, native.append, 10),
    ]
    settings = {'armed': False, 'mode': 'POSCTL', 'connected': True, 'state_age': 0.,
                'pose_age': 0., 'fixed_stamp': None, 'bad': None, 'sign': 1.,
                'origin_change': 0., 'ctrl': 11, 'hgt': 3, 'lever': 0., 'qmin': 0}

    def parameters(request, response):
        assert request.names == PARAMETERS
        response.values = [ParameterValue(type=2, integer_value=settings['ctrl']),
                           ParameterValue(type=2, integer_value=settings['hgt']),
                           *[ParameterValue(type=3, double_value=settings['lever']) for _ in range(3)],
                           ParameterValue(type=2, integer_value=settings['qmin'])]
        return response

    service = probe.create_service(GetParameters, '/mavros/param/get_parameters', parameters)

    def set_stamp(msg, ns):
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)

    def state():
        msg = State()
        set_stamp(msg, probe.get_clock().now().nanoseconds - int(settings['state_age']*1e9))
        msg.connected, msg.armed, msg.mode = settings['connected'], settings['armed'], settings['mode']
        state_pub.publish(msg)

    def poses():
        ns = settings['fixed_stamp']
        if ns is None:
            ns = probe.get_clock().now().nanoseconds - int(settings['pose_age']*1e9)
        raw, aligned = Odometry(), PoseWithCovarianceStamped()
        set_stamp(raw, ns)
        set_stamp(aligned, ns)
        raw.header.frame_id, raw.child_frame_id = 'camera_init', 'body'
        aligned.header.frame_id = 'px4_enu_preview'
        p, q = np.array([.02, .03, .04]), np.array([0., 0., 0., 1.])
        raw.pose.pose.position.x, raw.pose.pose.position.y, raw.pose.pose.position.z = p.tolist()
        raw.pose.pose.orientation.w = 1.
        covariance = np.diag([1e-5, 2e-5, 3e-5, 1e-6, 2e-6, 3e-6])
        position, orientation, covariance = convert(p, q, covariance)
        position -= np.array([.05 + settings['origin_change'], .04, .03])
        orientation *= settings['sign']
        target = aligned.pose.pose
        target.position.x, target.position.y, target.position.z = position.tolist()
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = orientation.tolist()
        if settings['bad'] == 'asymmetric':
            covariance[0, 1] += .001
        elif settings['bad'] == 'indefinite':
            covariance[0, 1] = covariance[1, 0] = .001
        elif settings['bad'] == 'zero':
            covariance[:] = 0.
        elif settings['bad'] == 'nonfinite':
            covariance[0, 0] = math.nan
        elif settings['bad'] == 'quaternion':
            target.orientation.w += 2.
        elif settings['bad'] == 'frame':
            raw.header.frame_id = 'world'
        aligned.pose.covariance = covariance.ravel().tolist()
        inputs.append(aligned)
        raw_pub.publish(raw)
        aligned_pub.publish(aligned)

    timers = [probe.create_timer(.05, state), probe.create_timer(.1, poses)]

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    def wait(predicate):
        end = time.monotonic() + 9
        while not predicate():
            assert time.monotonic() < end, 'Timed out'
            spin(.02)

    def new_bridge(output=False):
        nonlocal bridge
        if bridge is not None:
            executor.remove_node(bridge)
            bridge.destroy_node()
            spin(.15)
        bridge = FlightPoseCovTransport(parameter_overrides=[
            Parameter('alignment_path', value=str(path)), Parameter('publish_to_mavros', value=output)])
        executor.add_node(bridge)

    def blocked(messages):
        spin(.2)
        count = len(messages)
        spin(.3)
        assert len(messages) == count, 'Blocked condition emitted a pose'

    def resumes(messages):
        count = len(messages)
        wait(lambda: len(messages) > count)

    try:
        new_bridge()
        wait(lambda: len(debug) >= 4)
        assert not native, 'Default preview published to MAVROS'
        out = debug[-1]
        assert any(out == msg for msg in inputs), 'Pose/covariance/frame/stamp was changed'
        count = len(debug)
        spin(.6)
        assert 4 <= len(debug)-count <= 8, 'Output not tied to approximately 10 Hz input'
        original = bridge.mapping.origin.copy()
        settings['armed'] = True
        resumes(debug)
        settings['mode'] = 'OFFBOARD'
        resumes(debug)
        assert bridge.fault is None and np.allclose(bridge.mapping.origin, original), 'Normal arming rebased or stopped vision'
        settings['sign'] = -1.
        resumes(debug)
        settings['sign'] = 1.
        for key, value in [('connected', False), ('state_age', 3.), ('pose_age', 1.),
                           ('pose_age', -1.), ('bad', 'asymmetric'), ('bad', 'indefinite'),
                           ('bad', 'zero'), ('bad', 'nonfinite'), ('bad', 'quaternion'), ('bad', 'frame')]:
            previous = settings[key]
            settings[key] = value
            blocked(debug)
            settings[key] = previous
            resumes(debug)
        settings['fixed_stamp'] = probe.get_clock().now().nanoseconds
        blocked(debug)
        settings['fixed_stamp'] = None
        resumes(debug)
        for key, value in [('ctrl', 9), ('hgt', 1), ('lever', .1), ('qmin', 1)]:
            previous = settings[key]
            settings[key] = value
            wait(lambda: not bridge.configuration_ok)
            blocked(debug)
            settings[key] = previous
            resumes(debug)
        probe.destroy_service(service)
        wait(lambda: not bridge.configuration_ok)
        blocked(debug)
        service = probe.create_service(GetParameters, '/mavros/param/get_parameters', parameters)
        resumes(debug)
        settings['origin_change'] = .02
        wait(lambda: bridge.fault is not None)
        blocked(debug)
        settings['origin_change'] = 0.
        blocked(debug)
        # Startup while armed must not silently adopt a new origin.
        new_bridge()
        wait(lambda: bridge.fault is not None)
        blocked(debug)
        settings['armed'], settings['mode'] = False, 'POSCTL'
        blocked(debug)
        new_bridge()
        resumes(debug)
        path.write_text(path.read_text() + '\n# changed calibration file\n')
        wait(lambda: bridge.fault is not None)
        blocked(debug)
        shutil.copy2(source, path)
        new_bridge()
        resumes(debug)
        probe.destroy_publisher(aligned_pub)
        aligned_pub = probe.create_publisher(PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', 10)
        wait(lambda: bridge.fault is not None)
        blocked(debug)
        new_bridge()
        resumes(debug)
        extra_raw = probe.create_publisher(Odometry, '/Odometry', 10)
        wait(lambda: bridge.fault is not None)
        blocked(debug)
        probe.destroy_publisher(extra_raw)
        new_bridge(output=True)
        resumes(native)
        assert any(native[-1] == msg for msg in inputs)
        extra_native = probe.create_publisher(PoseWithCovarianceStamped, NATIVE_TOPIC, 10)
        wait(lambda: bridge.count_publishers(NATIVE_TOPIC) == 2)
        blocked(native)
        probe.destroy_publisher(extra_native)
        resumes(native)
        settings['armed'], settings['mode'] = True, 'OFFBOARD'
        resumes(native)
        assert bridge.fault is None
        result = bridge.set_parameters([Parameter('publish_to_mavros', value=False)])
        assert not result[0].successful, 'Output switch changed while running'
        print('PASS: default debug/native opt-in; identical pose/covariance/stamp, fixed session across arming/OFFBOARD; source/geometry/origin resets latch off; stale/state/parameter/covariance/duplicate-writer gates.')
    finally:
        executor.shutdown()
        if bridge is not None:
            bridge.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(directory)


if __name__ == '__main__':
    main()
