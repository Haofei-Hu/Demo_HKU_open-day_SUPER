#!/usr/bin/env python3
"""Isolated ROS graph test: native position/yaw ground relay and stop gates."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from ground_setpoint_transport import GroundSetpointTransport


def main():
    rclpy.init()
    bridge = GroundSetpointTransport()
    probe = Node('ground_setpoint_probe')
    state_pub = probe.create_publisher(State, '/mavros/state', 10)
    local_pub = probe.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
    target_pub = probe.create_publisher(PoseStamped, '/super_debug/control_target', 10)
    settings = {'connected': True, 'armed': False, 'mode': 'POSCTL',
                'frame': 'LOCAL_NED', 'target_age': 0., 'local_age': 0., 'state_age': 0.,
                'bad': None, 'fixed_stamp': None}
    received, targets = [], []
    sub = probe.create_subscription(PoseStamped, '/mavros/setpoint_position/local', received.append, 10)

    def parameter(request, response):
        assert request.names == ['mav_frame']
        value = ParameterValue(type=4, string_value=settings['frame'])
        response.values = [value]
        return response

    service = probe.create_service(GetParameters, '/mavros/setpoint_position/get_parameters', parameter)

    def stamp(msg, age):
        ns = probe.get_clock().now().nanoseconds - int(age * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)

    def send_state():
        msg = State()
        stamp(msg, settings['state_age'])
        msg.connected, msg.armed, msg.mode = settings['connected'], settings['armed'], settings['mode']
        state_pub.publish(msg)

    def send_local():
        msg = PoseStamped()
        stamp(msg, settings['local_age'])
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = .05, .02, .01
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(math.radians(111)/2), math.cos(math.radians(111)/2)
        local_pub.publish(msg)

    def send_target():
        msg = PoseStamped()
        stamp(msg, settings['target_age'])
        if settings['fixed_stamp'] is not None:
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(settings['fixed_stamp'], 1_000_000_000)
        msg.header.frame_id = 'px4_enu_preview'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = -.3, .17, .22
        # Nonzero roll/pitch must be removed; ENU position and yaw retained.
        roll, pitch, yaw = .2, -.1, math.radians(111)
        sr, sp, sy = math.sin(roll/2), math.sin(pitch/2), math.sin(yaw/2)
        cr, cp, cy = math.cos(roll/2), math.cos(pitch/2), math.cos(yaw/2)
        q = msg.pose.orientation
        q.x, q.y, q.z, q.w = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
        bad = settings['bad']
        if bad == 'nonfinite':
            msg.pose.position.x = math.nan
        elif bad == 'norm':
            q.w = 4.
        elif bad == 'frame':
            msg.header.frame_id = 'world'
        elif bad == 'distance':
            msg.pose.position.x = 1.5
        elif bad == 'yaw':
            q.x, q.y, q.z, q.w = 0., 0., 0., 1.
        targets.append(msg)
        target_pub.publish(msg)

    timers = [probe.create_timer(.05, send_state), probe.create_timer(.05, send_local),
              probe.create_timer(.01, send_target)]
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    def wait(predicate):
        end = time.monotonic() + 7
        while not predicate():
            assert time.monotonic() < end, 'Timed out'
            spin(.02)

    def blocked():
        spin(.15)
        count = len(received)
        spin(.2)
        assert len(received) == count, 'Blocked case emitted native setpoint'

    def resumes():
        count = len(received)
        wait(lambda: len(received) > count)

    try:
        wait(lambda: len(received) >= 4)
        output = received[-1]
        p, q = output.pose.position, output.pose.orientation
        assert [p.x, p.y, p.z] == [-.3, .17, .22], 'Unexpected ENU/NED conversion in relay'
        assert q.x == q.y == 0.
        assert abs(q.z-math.sin(math.radians(111)/2)) < 1e-12
        assert abs(q.w-math.cos(math.radians(111)/2)) < 1e-12
        assert output.header.frame_id == 'map'
        assert any(output.header.stamp == target.header.stamp for target in targets), 'Source stamp replaced'
        start_count = len(received)
        spin(.5)
        assert 7 <= len(received)-start_count <= 13, 'Native output rate not approximately 20 Hz'
        names = [name for name, _ in bridge.get_publisher_names_and_types_by_node(bridge.get_name(), bridge.get_namespace())]
        assert '/mavros/setpoint_position/local' in names
        assert not any(name in names for name in ['/mavros/setpoint_raw/local', '/mavros/setpoint_attitude/attitude'])
        settings['mode'] = 'AUTO.LOITER'
        resumes()
        settings['mode'] = 'POSCTL'
        resumes()
        for key, value in [('connected', False), ('mode', 'MANUAL'), ('mode', 'STABILIZED'), ('state_age', 3.),
                           ('local_age', 1.), ('target_age', 1.), ('target_age', -1.),
                           ('bad', 'nonfinite'), ('bad', 'norm'), ('bad', 'frame'),
                           ('bad', 'distance'), ('bad', 'yaw')]:
            previous = settings[key]
            settings[key] = value
            blocked()
            settings[key] = previous
            resumes()
        settings['fixed_stamp'] = probe.get_clock().now().nanoseconds
        blocked()
        settings['fixed_stamp'] = None
        resumes()
        extra = probe.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        wait(lambda: bridge.count_publishers('/mavros/setpoint_position/local') == 2)
        blocked()
        probe.destroy_publisher(extra)
        resumes()
        settings['frame'] = 'BODY_NED'
        wait(lambda: not bridge.configuration_ok)
        blocked()
        settings['frame'] = 'LOCAL_NED'
        resumes()
        probe.destroy_service(service)
        wait(lambda: not bridge.configuration_ok)
        blocked()
        service = probe.create_service(GetParameters, '/mavros/setpoint_position/get_parameters', parameter)
        resumes()
        settings['mode'] = 'OFFBOARD'
        wait(lambda: bridge.latched)
        blocked()
        settings['mode'] = 'AUTO.LOITER'
        blocked()
        # New process equivalent, to independently test arming latch.
        bridge.latched = False
        resumes()
        settings['armed'] = True
        wait(lambda: bridge.latched)
        blocked()
        settings['armed'] = False
        blocked()
        print('PASS: disarmed POSCTL/AUTO.LOITER, 20 Hz ENU position/yaw, preserved stamps; stale/invalid inputs, duplicate writer, frame changes and service loss blocked; OFFBOARD and arming latch off.')
    finally:
        executor.shutdown()
        bridge.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
