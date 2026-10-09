#!/usr/bin/env python3
"""Isolated DDS/services exercise; simulated telemetry, no vehicle attached."""
import math
import importlib
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, RCIn
from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger
controller = importlib.import_module(os.environ.get('FIRST_HOVER_TEST_MODULE', 'first_hover_control'))
FirstHoverControl, PARAMETERS, NATIVE = controller.FirstHoverControl, controller.PARAMETERS, controller.NATIVE


def main():
    rclpy.init()
    params = Node('param', namespace='/mavros')
    for key, value in PARAMETERS.items():
        params.declare_parameter(key, value)
    frame = Node('setpoint_position', namespace='/mavros')
    frame.declare_parameter('mav_frame', 'LOCAL_NED')
    probe = Node('hover_flight_fixture')
    kinds = {'state': (State, '/mavros/state'), 'local': (PoseStamped, '/mavros/local_position/pose'),
             'vision': (PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov'),
             'estimator': (EstimatorStatus, '/mavros/estimator_status'),
             'extended': (ExtendedState, '/mavros/extended_state'), 'rc': (RCIn, '/mavros/rc/in')}
    publishers = {key: probe.create_publisher(kind, topic, 10) for key, (kind, topic) in kinds.items()}
    ground = [.12, -.08, -.74]  # verifies a floor below an earlier origin
    settings = {}
    commands, targets = [], []
    executor = SingleThreadedExecutor()
    for node in (probe, params, frame):
        executor.add_node(node)
    bridge = None

    def reset():
        settings.clear()
        settings.update(mode='POSCTL', armed=False, bad=None, rejected=None, target=None,
                        landed=ExtendedState.LANDED_STATE_ON_GROUND, auto_landed=True)
        commands.clear()
        targets.clear()

    def stamp(msg):
        msg.header.stamp = probe.get_clock().now().to_msg()

    def telemetry():
        xyz = list(ground)
        if settings['armed'] and settings['target'] is not None:
            p = settings['target'].pose.position
            xyz = [p.x, p.y, p.z]
        if settings['bad'] == 'departure':
            xyz[0] += .35
        messages = {
            'state': State(connected=True, armed=settings['armed'], mode=settings['mode'], manual_input=True),
            'local': PoseStamped(), 'vision': PoseWithCovarianceStamped(),
            'estimator': EstimatorStatus(attitude_status_flag=True, velocity_horiz_status_flag=True,
                                         velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True,
                                         pos_vert_abs_status_flag=True),
            'extended': ExtendedState(landed_state=settings['landed']),
            'rc': RCIn(channels=[1500]*16),
        }
        local = messages['local']
        local.header.frame_id = 'map'
        local.pose.position.x, local.pose.position.y, local.pose.position.z = xyz
        local.pose.orientation.z, local.pose.orientation.w = math.sin(.3), math.cos(.3)
        messages['vision'].pose.pose = local.pose
        messages['vision'].header.frame_id = 'px4_enu_preview'
        if settings['bad'] == 'estimate':
            messages['estimator'].pos_horiz_rel_status_flag = False
        for key, msg in messages.items():
            if key == settings['bad']:
                continue
            stamp(msg)
            publishers[key].publish(msg)

    def mode(request, response):
        commands.append(request.custom_mode)
        response.mode_sent = settings['rejected'] != request.custom_mode
        if response.mode_sent:
            settings['mode'] = request.custom_mode
            if request.custom_mode == 'AUTO.LAND' and settings['auto_landed']:
                settings['landed'] = ExtendedState.LANDED_STATE_ON_GROUND
                settings['armed'] = False
        return response

    def arm(request, response):
        commands.append('ARM' if request.value else 'DISARM')
        response.success = settings['rejected'] != 'ARM'
        if response.success:
            settings['armed'] = request.value
            settings['landed'] = ExtendedState.LANDED_STATE_IN_AIR
        return response

    def target(msg):
        targets.append((time.monotonic(), msg))
        settings['target'] = msg

    probe.create_service(SetMode, '/mavros/set_mode', mode)
    probe.create_service(CommandBool, '/mavros/cmd/arming', arm)
    probe.create_subscription(PoseStamped, NATIVE, target, 10)
    probe.create_timer(.1, telemetry)  # the localization under test stays at 10 Hz
    start = probe.create_client(Trigger, '/first_hover/start')
    stop = probe.create_client(Trigger, '/first_hover/land')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)

    def wait(predicate, seconds=8):
        end = time.monotonic() + seconds
        while not predicate():
            assert time.monotonic() < end, (bridge.phase, bridge.fault, commands)
            spin(.02)

    def call(client):
        wait(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        wait(future.done)
        return future.result()

    def new(execute=True):
        nonlocal bridge
        if bridge:
            executor.remove_node(bridge)
            bridge.destroy_node()
            spin(.15)
        reset()
        bridge = FirstHoverControl(execute=execute)
        executor.add_node(bridge)
        wait(lambda: bridge.phase == 'READY', seconds=9)

    def fly():
        assert call(start).success
        wait(lambda: bridge.phase == 'RISE')
        assert commands == ['OFFBOARD', 'ARM']
        assert call(start).success is False, 'Duplicate start accepted'

    def stopped():
        wait(lambda: bridge.phase == 'STOPPED')
        spin(.2)
        count = len(targets)
        settings['bad'] = None
        spin(.3)
        assert len(targets) == count, 'Fault silently resumed'
        assert not call(start).success, 'Fault allowed rearm'

    try:
        new(execute=False)
        assert not call(start).success
        spin(.3)
        assert not targets and not commands
        assert bridge.count_publishers(NATIVE) == 0, 'Default created a flight publisher'

        new()
        assert bridge.reference.ground == tuple(ground)
        assert abs(bridge.reference.lift - .415) < 1e-10
        assert not targets and not commands, 'Starting execute node initiated flight'
        fly()
        initial = len(targets)
        spin(.6)
        assert 9 <= len(targets)-initial <= 15, 'Native setpoints not ~20 Hz'
        # Advance the mission clock, not the measurements. All telemetry remains 10 Hz.
        bridge.armed_at -= 4
        wait(lambda: bridge.phase == 'HOLD')
        spin(.15)
        for _, msg in targets:
            p = msg.pose.position
            assert p.x == ground[0] and p.y == ground[1]
            assert ground[2] <= p.z <= ground[2]+.415+1e-10
            assert msg.header.frame_id == 'map'
        assert abs(targets[-1][1].pose.position.z - (ground[2]+.415)) < 1e-10
        bridge.hold_at -= 2
        wait(lambda: bridge.phase == 'DONE')
        spin(.2)
        count = len(targets)
        spin(.25)
        assert len(targets) == count
        assert commands == ['OFFBOARD', 'ARM', 'AUTO.LAND']
        assert not call(start).success

        for rejected in ('OFFBOARD', 'ARM'):
            new()
            settings['rejected'] = rejected
            assert call(start).success
            stopped()
            assert not settings['armed']
            assert commands == (['OFFBOARD'] if rejected == 'OFFBOARD' else ['OFFBOARD', 'ARM'])

        for bad in ('vision', 'rc', 'estimate', 'departure'):
            new()
            fly()
            settings['bad'] = bad
            stopped()
            assert commands == ['OFFBOARD', 'ARM', 'AUTO.LAND'], (bad, commands)

        new()
        fly()
        settings['mode'] = 'POSCTL'
        stopped()
        assert commands == ['OFFBOARD', 'ARM'], 'Pilot takeover was overridden by LAND'

        new()
        fly()
        duplicate = probe.create_publisher(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', 10)
        stopped()
        probe.destroy_publisher(duplicate)

        new()
        fly()
        bridge.last_tick -= .3
        stopped()

        new()
        fly()
        settings['auto_landed'] = False
        assert call(stop).success
        wait(lambda: bridge.phase == 'LANDING')
        spin(.5)
        assert bridge.phase == 'LANDING' and settings['armed'], 'Timed descent claimed landing'
        settings['landed'] = ExtendedState.LANDED_STATE_ON_GROUND
        spin(.2)
        assert bridge.phase == 'LANDING', 'Ground indication alone claimed disarm'
        settings['armed'] = False
        wait(lambda: bridge.phase == 'DONE')
        assert 'DISARM' not in commands, 'Controller force-disarmed in flight'
        print('PASS: default diagnostics only, explicit one-shot start, 10 Hz localization / '
              '20 Hz fixed XY/yaw setpoints, 0.415 m lift, mode/arm rejection, native landing '
              'and confirmed disarm, stale vision/RC, invalid estimate, deviation, duplicate '
              'source, timer interruption, pilot takeover, no automatic restart or forced disarm.')
    finally:
        if bridge:
            executor.remove_node(bridge)
            bridge.destroy_node()
        executor.shutdown()
        for node in (probe, params, frame):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
