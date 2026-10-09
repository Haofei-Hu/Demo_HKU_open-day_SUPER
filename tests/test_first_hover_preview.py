#!/usr/bin/env python3
"""ROS DDS test of floor-offset hover preview and failure latches; no aircraft."""
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State
from first_hover_preview import FirstHoverPreview, LIMITS, OUTPUT


def main():
    rclpy.init()
    parameters = Node('param', namespace='/mavros')
    for name, value in LIMITS.items():
        parameters.declare_parameter(name, value)
    probe = Node('first_hover_probe')
    state_pub = probe.create_publisher(State, '/mavros/state', 10)
    local_pub = probe.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
    vision_pub = probe.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
    output = []
    subscription = probe.create_subscription(PoseStamped, OUTPUT, output.append, 10)
    settings = {'bad': None}
    ground = (.12, -.08, -.74)  # takeoff floor below the old table-initialized origin
    yaw = math.radians(135)

    def stamp(msg):
        msg.header.stamp = probe.get_clock().now().to_msg()
        if settings['bad'] == 'stale':
            msg.header.stamp.sec -= 1

    def sources():
        state = State(connected=True, armed=False, mode='POSCTL')
        stamp(state)
        if settings['bad'] == 'armed':
            state.armed = True
        if settings['bad'] == 'offboard':
            state.mode = 'OFFBOARD'
        if settings['bad'] == 'disconnected':
            state.connected = False
        state_pub.publish(state)
        local = PoseStamped()
        stamp(local)
        local.header.frame_id = 'map'
        local.pose.position.x, local.pose.position.y, local.pose.position.z = ground
        local.pose.orientation.z, local.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        vision = PoseWithCovarianceStamped()
        stamp(vision)
        vision.header.frame_id = 'px4_enu_preview'
        vision.pose.pose = local.pose
        # Shared pose: both sources move together; departure from captured ground must stop output.
        if settings['bad'] == 'moved':
            local.pose.position.x += .2
        if settings['bad'] == 'frame':
            local.header.frame_id = 'world'
        if settings['bad'] == 'nan':
            local.pose.position.z = math.nan
        if settings['bad'] == 'vision_missing':
            local_pub.publish(local)
            return
        local_pub.publish(local)
        vision_pub.publish(vision)

    timer = probe.create_timer(.05, sources)
    executor = SingleThreadedExecutor()
    executor.add_node(parameters)
    executor.add_node(probe)
    bridge = None

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    def wait(predicate):
        end = time.monotonic() + 7
        while not predicate():
            assert time.monotonic() < end, 'Timed out'
            spin(.02)

    def new_bridge():
        nonlocal bridge
        if bridge is not None:
            executor.remove_node(bridge)
            bridge.destroy_node()
        settings['bad'] = None
        bridge = FirstHoverPreview()
        executor.add_node(bridge)
        wait(lambda: bridge.sent >= 4)

    def blocked():
        spin(.4)
        count = len(output)
        spin(.2)
        assert len(output) == count, 'Fault emitted preview targets'
        assert bridge.fault is not None
        settings['bad'] = None
        spin(.3)
        assert len(output) == count, 'Fault silently resumed or rebased'

    try:
        new_bridge()
        assert bridge.reference.ground == ground
        assert abs(bridge.reference.lift - .415) < 1e-12
        first = output[-1]
        assert first.header.frame_id == 'px4_enu_preview'
        count = len(output)
        spin(.5)
        assert 7 <= len(output) - count <= 13, 'Output not approximately 20 Hz'
        for message in output:
            p = message.pose.position
            assert (p.x, p.y) == ground[:2] and ground[2] <= p.z <= ground[2] + .415
            q = message.pose.orientation
            assert q.x == q.y == 0 and abs(q.z - math.sin(yaw/2)) < 1e-12
        bridge.started_at -= 30.
        spin(.1)
        p = output[-1].pose.position
        assert (p.x, p.y, p.z) == ground, 'Completion reference did not hold original ground pose'
        names = bridge.get_publisher_names_and_types_by_node(bridge.get_name(), bridge.get_namespace())
        assert not any(name.startswith('/mavros/') for name, _ in names), 'Preview sent native flight output'

        for bad in ('stale', 'armed', 'offboard', 'disconnected', 'moved', 'frame', 'nan', 'vision_missing'):
            new_bridge()
            settings['bad'] = bad
            blocked()

        new_bridge()
        parameters.set_parameters([Parameter('COM_OBL_RC_ACT', value=0)])
        wait(lambda: bridge.fault is not None)
        blocked()
        parameters.set_parameters([Parameter('COM_OBL_RC_ACT', value=4)])

        new_bridge()
        duplicate = probe.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        wait(lambda: bridge.fault is not None)
        blocked()
        probe.destroy_publisher(duplicate)

        new_bridge()
        bridge.source_ids = (b'changed', b'changed')
        blocked()
        print('PASS: table-to-floor offset, fixed XY/yaw, smooth rise/hold/down reference, '
              '20 Hz debug-only output, completion ground hold, source/state/frame/configuration '
              'failure latches; no arming/mode/setpoint output')
    finally:
        if bridge is not None:
            executor.remove_node(bridge)
            bridge.destroy_node()
        executor.shutdown()
        probe.destroy_node()
        parameters.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
