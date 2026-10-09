#!/usr/bin/env python3
"""Isolated ROS parameter services: backups, exact writes and stop gates."""
import json
from pathlib import Path
import tempfile
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from mavros_msgs.msg import State
from configure_first_hover import HoverConfiguration, NAMES, TARGETS


def main():
    rclpy.init()
    server = Node('param', namespace='/mavros')
    original = {name: 3.0 for name in NAMES}
    original.update(MPC_XY_VEL_MAX=12.0, MPC_Z_VEL_MAX_UP=3.0, MPC_Z_VEL_MAX_DN=1.5,
                    MPC_XY_VEL_ALL=10.0, MPC_Z_VEL_ALL=3.0, MPC_ACC_HOR=3.0,
                    MPC_ACC_HOR_MAX=5.0, MPC_THR_HOVER=.5, COM_OF_LOSS_T=1.0,
                    COM_OBL_RC_ACT=0, EKF2_EV_CTRL=11, EKF2_HGT_REF=3)
    for name, value in original.items():
        server.declare_parameter(name, value)
    calls = []
    settings = {'reject': None, 'armed': False}

    def on_set(parameters):
        calls.extend(p.name for p in parameters)
        if any(p.name == settings['reject'] for p in parameters):
            return SetParametersResult(successful=False, reason='test rejection')
        return SetParametersResult(successful=True)

    server.add_on_set_parameters_callback(on_set)
    probe = Node('hover_config_state_fixture')
    publisher = probe.create_publisher(State, '/mavros/state', 10)

    def state():
        msg = State(connected=True, armed=settings['armed'], mode='POSCTL')
        msg.header.stamp = probe.get_clock().now().to_msg()
        return msg

    timer = probe.create_timer(.05, lambda: publisher.publish(state()))

    class DelayedSetter:
        """Delay discovery while retaining the real parameter service behind it."""
        def __init__(self, client, delay, on_wait):
            self.client, self.delay, self.on_wait = client, delay, on_wait
            self.available_at = None

        def service_is_ready(self):
            if self.available_at is None:
                self.available_at = time.monotonic() + self.delay
                self.on_wait()
            return time.monotonic() >= self.available_at and self.client.service_is_ready()

        def wait_for_service(self, timeout_sec):
            end = time.monotonic() + timeout_sec
            while not self.service_is_ready() and time.monotonic() < end:
                time.sleep(.01)
            return self.service_is_ready()

        def call_async(self, request):
            return self.client.call_async(request)

    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(probe)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    bridge = None
    try:
        with tempfile.TemporaryDirectory() as root:
            bridge = HoverConfiguration()
            path = bridge.run(root)
            journal = json.loads(path.read_text())
            assert journal['before'] == original
            assert journal['requested'] == TARGETS and journal['applied'] == []
            assert calls == [], 'Read-only run modified parameters'
            path = bridge.run(root, apply=True)
            journal = json.loads(path.read_text())
            assert journal['before'] == original, 'Backup omitted original coupled limits'
            assert journal['applied'] == list(TARGETS) and calls == list(TARGETS)
            for name in NAMES:
                assert server.get_parameter(name).value == TARGETS.get(name, original[name])
            assert journal['after']['MPC_THR_HOVER'] == .5
            assert journal['after']['MPC_ACC_HOR'] == 3.0
            assert journal['after']['MPC_ACC_HOR_MAX'] == 5.0
            publishers = bridge.get_publisher_names_and_types_by_node(bridge.get_name(), bridge.get_namespace())
            assert not any(topic.startswith('/mavros/') for topic, _ in publishers), 'Unexpected flight output'

            # Readiness must reject stale/future stamps, mode, disconnection and latch arming.
            for bad in ('old', 'future', 'mode', 'connected', 'receipt', 'armed'):
                bridge.arm_seen = False
                msg = state()
                if bad == 'old':
                    msg.header.stamp.sec -= 3
                elif bad == 'future':
                    msg.header.stamp.sec += 3
                elif bad == 'mode':
                    msg.mode = 'OFFBOARD'
                elif bad == 'connected':
                    msg.connected = False
                elif bad == 'armed':
                    msg.armed = True
                bridge.receive_state(msg)
                if bad == 'receipt':
                    bridge.received_at = time.monotonic() - 3
                assert not bridge.ready(), f'Invalid {bad} state allowed'
            bridge.receive_state(state())
            assert not bridge.ready(), 'Arming latch cleared on disarm'
            writes = len(calls)
            try:
                bridge.run(root, apply=True)
                raise AssertionError('Armed initialization not refused')
            except RuntimeError:
                pass
            assert len(calls) == writes
            bridge.destroy_node()
            bridge = HoverConfiguration()

            # A discovery wait must consume arming updates and recheck state age
            # immediately before the first write; ordinary reads remain available.
            for failure in ('stale', 'armed'):
                writes = len(calls)
                on_wait = timer.cancel if failure == 'stale' else lambda: settings.update(armed=True)
                bridge.setter = DelayedSetter(bridge.setter, 2.3 if failure == 'stale' else .3, on_wait)
                try:
                    bridge.run(root, apply=True)
                    raise AssertionError(f'Delayed setter accepted {failure} state')
                except RuntimeError:
                    pass
                assert len(calls) == writes, f'Wrote after {failure} state during discovery wait'
                journal = json.loads(bridge.backup.read_text())
                assert journal['applied'] == [] and 'error' in journal
                assert not bridge.ready()
                assert set(bridge.read()) == set(NAMES), 'State gate blocked read-only inspection'
                bridge.destroy_node()
                settings['armed'] = False
                timer.reset()
                bridge = HoverConfiguration()

            # Missing parameters must block before any mutation.
            server.undeclare_parameter('SYS_VEHICLE_RESP')
            writes = len(calls)
            try:
                bridge.run(root, apply=True)
                raise AssertionError('Incomplete parameter response accepted')
            except RuntimeError:
                pass
            assert len(calls) == writes
            server.declare_parameter('SYS_VEHICLE_RESP', original['SYS_VEHICLE_RESP'])

            # A rejected write stops later writes and retains a truthful partial journal.
            server.set_parameters([Parameter('MPC_XY_VEL_MAX', value=12.0)])
            calls.clear()
            settings['reject'] = 'MPC_XY_VEL_MAX'
            try:
                bridge.run(root, apply=True)
                raise AssertionError('Rejected parameter write did not stop')
            except RuntimeError:
                pass
            journal = json.loads(bridge.backup.read_text())
            assert journal['before']['MPC_XY_VEL_MAX'] == 12.0
            assert journal['applied'] == ['MPC_XY_VEL_ALL', 'MPC_Z_VEL_ALL']
            assert calls == list(TARGETS)[:3], 'Continued writes after a rejection'
            assert 'error' in journal and server.get_parameter('MPC_XY_VEL_MAX').value == 12.0
        print('PASS: read-only default, backup before writes, exact low-speed/failsafe settings, '
              'unchanged thrust/acceleration, disarmed-state gates, missing parameter, '
              'delayed-service stale/arming gates, read-only inspection after state loss, '
              'partial rejection journal, no flight output')
    finally:
        if bridge is not None:
            bridge.destroy_node()
        executor.shutdown(timeout_sec=2)
        worker.join(timeout=2)
        server.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
