#!/usr/bin/env python3
"""Isolated DDS test with analytic 90-degree world / 45-degree mount geometry."""
import math
from pathlib import Path
import tempfile
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mars_quadrotor_msgs.msg import PositionCommand
from control_bridge_preview import load_converter, node_type


def main():
    with tempfile.TemporaryDirectory() as directory:
        alignment = Path(directory) / 'alignment.py'
        # Reuse the real mounting/rotation/lever mathematics, with an analytic fixture.
        original = (Path(__file__).resolve().parents[1] / 'aligned_pose_preview.py').read_text()
        alignment.write_text(original +
            '\nQ_WC = np.array([0.,0.,2**-.5,2**-.5])\n'
            'R_WC = rotation(Q_WC)\n'
            'Q_IB = np.array([0.,0.,-np.sin(np.pi/8),np.cos(np.pi/8)])\n'
            'T_BI = np.array([.1,0.,.1])\n'
            'AXES[:3,:3] = AXES[3:,3:] = R_WC\n'
            'import rclpy\n'
            'rclpy.init()\n'
            'raise RuntimeError("unguarded ROS entry must never run")\n')
        # Inline-callback variant must also load without executing ROS startup.
        inline = Path(directory) / 'inline_alignment.py'
        inline.write_text(alignment.read_text().replace('def convert(', 'def unused_conversion('))
        fallback = load_converter(inline)
        p, q, _ = fallback(np.array([2.,1.,.5]),
                           np.array([0.,0.,math.sin(math.pi/8),math.cos(math.pi/8)]), np.eye(6))
        np.testing.assert_allclose(p, [-1.,1.9,.4], atol=1e-9)
        np.testing.assert_allclose(q, [0.,0.,2**-.5,2**-.5], atol=1e-9)
        rclpy.init(args=['--ros-args', '-p', f'alignment_path:={alignment}'])
        preview = node_type()()
        probe = Node('control_preview_test')
        odom_pub = probe.create_publisher(Odometry, '/Odometry', 10)
        aligned_pub = probe.create_publisher(PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', 10)
        state_pub = probe.create_publisher(State, '/mavros/state', 10)
        cmd_pub = probe.create_publisher(PositionCommand, '/planning/pos_cmd', 10)
        received = []
        sub = probe.create_subscription(PoseStamped, '/super_debug/control_target', received.append, 10)
        settings = {'connected': True, 'armed': False, 'age': 0., 'shift': 0.}

        def state():
            msg = State()
            ns = probe.get_clock().now().nanoseconds - int(settings['age'] * 1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            msg.connected, msg.armed = settings['connected'], settings['armed']
            state_pub.publish(msg)

        def poses():
            stamp = probe.get_clock().now().to_msg()
            raw = Odometry()
            raw.header.stamp, raw.header.frame_id, raw.child_frame_id = stamp, 'camera_init', 'body'
            raw.pose.pose.position.x, raw.pose.pose.position.y, raw.pose.pose.position.z = 2., 1., .5
            raw.pose.pose.orientation.z, raw.pose.pose.orientation.w = math.sin(math.pi/8), math.cos(math.pi/8)
            aligned = PoseWithCovarianceStamped()
            aligned.header.stamp, aligned.header.frame_id = stamp, 'px4_enu_preview'
            aligned.pose.pose.position.x = .2 + settings['shift']
            aligned.pose.pose.position.y, aligned.pose.pose.position.z = -.1, .05
            # q and -q represent identical rotations.
            aligned.pose.pose.orientation.z = aligned.pose.pose.orientation.w = -2**-.5
            odom_pub.publish(raw)
            aligned_pub.publish(aligned)

        state_timer = probe.create_timer(.05, state)
        pose_timer = probe.create_timer(.05, poses)
        executor = SingleThreadedExecutor()
        executor.add_node(preview)
        executor.add_node(probe)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)

        def wait_for(predicate):
            end = time.monotonic() + 8
            while not predicate():
                assert time.monotonic() < end, 'Timed out'
                spin(.02)

        def command(flag=1, yaw=math.pi/4):
            msg = PositionCommand()
            msg.header.stamp, msg.header.frame_id = probe.get_clock().now().to_msg(), 'world'
            msg.position.x, msg.position.y, msg.position.z = 2.3, 1.4, .7
            msg.attitude.z = msg.yaw = yaw
            msg.trajectory_flag = flag
            return msg

        def accept(msg, expected, expected_yaw):
            count = len(received)
            cmd_pub.publish(msg)
            spin(.1)
            assert len(received) == count + 1, 'Valid preview target blocked'
            out = received[-1]
            assert out.header.stamp == msg.header.stamp
            assert out.header.frame_id == 'px4_enu_preview'
            p, q = out.pose.position, out.pose.orientation
            np.testing.assert_allclose([p.x,p.y,p.z], expected, atol=1e-9)
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            assert abs(math.atan2(math.sin(yaw-expected_yaw), math.cos(yaw-expected_yaw))) < 1e-9
            assert msg.header.frame_id == 'world', 'Mutated incoming command'

        def reject(msg):
            count = len(received)
            cmd_pub.publish(msg)
            spin(.08)
            assert len(received) == count, 'Invalid preview target emitted'

        try:
            wait_for(lambda: preview.ready() and cmd_pub.get_subscription_count() == 1
                     and preview.publisher.get_subscription_count() == 1)
            names = [name for name, _ in preview.get_publisher_names_and_types_by_node(
                preview.get_name(), preview.get_namespace())]
            assert not any(name.startswith('/mavros/') for name in names)
            original = command()
            accept(original, [-.2,.2,.25], math.pi/2)
            reject(original)
            accept(command(2), [-.2,.2,.25], math.pi/2)
            # Rotating the desired body rotates the lever arm, not just the yaw.
            accept(command(1, 3*math.pi/4), [-.1,.3,.25], math.pi)
            for flag in [0,3,4,5,6,7,8,255]:
                reject(command(flag))
            for case in ['frame', 'position', 'attitude', 'yaw', 'stale', 'future', 'zero']:
                msg = command()
                if case == 'frame':
                    msg.header.frame_id = 'camera_init'
                elif case == 'position':
                    msg.position.x = math.nan
                elif case == 'attitude':
                    msg.attitude.x = math.inf
                elif case == 'yaw':
                    msg.yaw = math.nan
                elif case == 'stale':
                    msg.header.stamp.sec -= 1
                elif case == 'future':
                    msg.header.stamp.sec += 1
                else:
                    msg.header.stamp.sec = msg.header.stamp.nanosec = 0
                reject(msg)
            settings['connected'] = False
            spin(.15)
            reject(command())
            settings['connected'], settings['age'] = True, 3.
            spin(.15)
            reject(command())
            settings['age'] = 0.
            wait_for(preview.ready)
            pose_timer.cancel()
            spin(.35)
            reject(command())
            pose_timer.reset()
            wait_for(preview.ready)
            state_timer.cancel()
            preview.state_at = time.monotonic() - 3
            reject(command())
            state_timer.reset()
            wait_for(preview.ready)
            accept(command(), [-.2,.2,.25], math.pi/2)
            # Header freshness must keep expiring after a delayed state was received.
            settings['age'] = 1.8
            wait_for(lambda: preview.connected_disarmed
                     and 1.7 <= (probe.get_clock().now().nanoseconds - preview.state_stamp) / 1e9 <= 1.95)
            state_timer.cancel()
            spin(.35)
            assert time.monotonic() - preview.state_at < 2
            assert (probe.get_clock().now().nanoseconds - preview.state_stamp) / 1e9 > 2
            reject(command())
            settings['age'] = 0.
            state_timer.reset()
            wait_for(preview.ready)
            accept(command(), [-.2,.2,.25], math.pi/2)
            settings['armed'] = True
            wait_for(lambda: preview.arm_seen)
            reject(command())
            settings['armed'] = False
            spin(.1)
            reject(command())
            settings['shift'] = .02
            wait_for(lambda: preview.mapping.failed)
            settings['shift'] = 0.
            spin(.1)
            assert preview.mapping.failed
            reject(command())
            print('PASS: exact world/mount/lever geometry, flags 1/2, stamp preservation, '
                  'invalid inputs/stale state (including cumulative header age)/localization rejected, origin mismatch '
                  'and arming latch; unguarded/inline preview startup skipped; '
                  'publishers contain no MAVROS output.')
        finally:
            executor.shutdown()
            preview.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
