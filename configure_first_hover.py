#!/usr/bin/env python3
"""Back up and optionally apply low-speed settings while connected/disarmed.

No mode changes, arming requests or setpoints. --apply performs parameter writes.
This is preparation for testing, not a validated flight controller or geofence.
"""
import argparse
import json
import math
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters, SetParameters
from mavros_msgs.msg import State


TARGETS = {
    'MPC_XY_VEL_ALL': -1.0,  # use individual limits instead of automatic overrides
    'MPC_Z_VEL_ALL': -1.0,
    'MPC_XY_VEL_MAX': 0.5,
    'MPC_Z_VEL_MAX_UP': 0.5,
    'MPC_Z_VEL_MAX_DN': 0.5,
    'COM_OF_LOSS_T': 0.5,
    'COM_OBL_RC_ACT': 4,  # Land in the pinned PX4 firmware
}
NAMES = list(TARGETS) + [
    'SYS_VEHICLE_RESP', 'MPC_ACC_HOR', 'MPC_ACC_HOR_MAX', 'MPC_THR_HOVER',
    'MPC_XY_CRUISE', 'MPC_VEL_MANUAL', 'MPC_Z_V_AUTO_UP', 'MPC_Z_V_AUTO_DN',
    'EKF2_EV_CTRL', 'EKF2_HGT_REF',
]


class HoverConfiguration(Node):
    def __init__(self):
        super().__init__('configure_first_hover')
        self.state = None
        self.received_at = -math.inf
        self.arm_seen = False
        self.subscription = self.create_subscription(
            State, '/mavros/state', self.receive_state, qos_profile_sensor_data)
        self.getter = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.setter = self.create_client(SetParameters, '/mavros/param/set_parameters')
        self.backup = None
        self.journal = None

    def receive_state(self, msg):
        self.state, self.received_at = msg, time.monotonic()
        self.arm_seen = self.arm_seen or msg.armed

    def ready(self):
        if self.state is None or self.arm_seen:
            return False
        stamp = self.state.header.stamp.sec * 1_000_000_000 + self.state.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return (stamp > 0 and -.05 <= age <= 2 and time.monotonic() - self.received_at <= 2
                and self.state.connected and not self.state.armed and self.state.mode == 'POSCTL')

    def call(self, client, request, require_ready=False):
        end = time.monotonic() + 5
        while not client.service_is_ready():
            if time.monotonic() >= end:
                raise RuntimeError('MAVROS parameter service unavailable; stop and inspect')
            rclpy.spin_once(self, timeout_sec=.05)
        if require_ready:
            rclpy.spin_once(self, timeout_sec=0)
            if not self.ready():
                raise RuntimeError('State changed/stale; stopping further parameter writes')
        future = client.call_async(request)
        end = time.monotonic() + 8
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
        if not future.done():
            future.cancel()
            raise RuntimeError('Parameter request timed out; read actual FC values before retrying')
        return future.result()

    def read(self):
        response = self.call(self.getter, GetParameters.Request(names=NAMES))
        if response is None or len(response.values) != len(NAMES):
            raise RuntimeError('Incomplete parameter response')
        values = {}
        for name, value in zip(NAMES, response.values):
            expected_type = 2 if name in ('COM_OBL_RC_ACT', 'EKF2_EV_CTRL', 'EKF2_HGT_REF') else 3
            if value.type != expected_type:
                raise RuntimeError(f'Missing/unexpected parameter type: {name}; no further writes')
            number = value.integer_value if value.type == 2 else value.double_value
            if not math.isfinite(number):
                raise RuntimeError(f'Nonfinite parameter: {name}')
            values[name] = number
        return values

    def save_journal(self):
        self.backup.write_text(json.dumps(self.journal, indent=2, allow_nan=False) + '\n')

    def run(self, directory, apply=False):
        end = time.monotonic() + 8
        while not self.ready() and not self.arm_seen and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
        if not self.ready():
            raise RuntimeError('Need fresh connected/disarmed POSCTL; no parameter writes')
        before = self.read()
        if before['EKF2_EV_CTRL'] != 11 or before['EKF2_HGT_REF'] != 3:
            raise RuntimeError('Expected current vision fusion CTRL=11/HGT_REF=3; no writes')
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(prefix='px4_before_first_hover_', suffix='.json',
                                         dir=directory, delete=False) as handle:
            self.backup = Path(handle.name)
        self.journal = {'before': before, 'requested': TARGETS, 'applied': [],
                        'note': 'PX4 can also clamp cruise/manual/auto vertical speeds to these caps.'}
        self.save_journal()
        print('Backup:', self.backup, flush=True)
        if not apply:
            print('READ ONLY: no parameters written. Requested:', TARGETS)
            return self.backup
        try:
            for name, number in TARGETS.items():
                rclpy.spin_once(self, timeout_sec=0)
                if not self.ready():
                    raise RuntimeError('State changed/stale; stopping further parameter writes')
                parameter = Parameter(name, value=number).to_parameter_msg()
                response = self.call(self.setter, SetParameters.Request(parameters=[parameter]),
                                     require_ready=True)
                if response is None or len(response.results) != 1 or not response.results[0].successful:
                    reason = response.results[0].reason if response and response.results else 'no result'
                    raise RuntimeError(f'Parameter refused: {name}: {reason}')
                self.journal['applied'].append(name)
                self.save_journal()
            after = self.read()
            self.journal['after'] = after
            self.save_journal()
            if not self.ready():
                raise RuntimeError('Fresh disarmed POSCTL lost during verification')
            for name, requested in TARGETS.items():
                if abs(after[name] - requested) > 1e-6:
                    raise RuntimeError(f'Readback mismatch: {name}={after[name]}; expected {requested}')
                print(f'{name}: {after[name]}', flush=True)
            print('Settings read back. No arming, mode changes or setpoints sent.')
            print('Hover thrust/acceleration parameters were not requested to change.')
            return self.backup
        except Exception as exc:
            self.journal['error'] = str(exc)
            self.save_journal()
            print('Stopped; previous writes can remain. Inspect backup and actual FC readback.', flush=True)
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--backup-dir', type=Path, default=Path.home() / 'Downloads')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = HoverConfiguration()
    try:
        node.run(args.backup_dir, args.apply)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
