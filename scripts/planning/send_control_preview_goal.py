#!/usr/bin/env python3
"""Send one small SUPER goal for the disarmed debug-coordinate check."""
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State


def main():
    rclpy.init()
    node = rclpy.create_node('send_control_preview_goal')
    received = {}
    subscriptions = [
        node.create_subscription(Odometry, '/Odometry',
                                 lambda msg: received.update(odom=msg), qos_profile_sensor_data),
        node.create_subscription(State, '/mavros/state',
                                 lambda msg: received.update(state=msg), qos_profile_sensor_data)]
    publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)

    def fresh(msg, limit):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp > 0 and -.05 <= age <= limit

    try:
        end = time.monotonic() + 8
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if not all(key in received for key in ('odom', 'state')):
                continue
            odom, state = received['odom'], received['state']
            if not fresh(odom, .25) or not fresh(state, 2):
                continue
            if not state.connected or state.armed:
                raise SystemExit('Need connected, disarmed FC; no goal sent.')
            if odom.header.frame_id != 'camera_init' or odom.child_frame_id != 'body':
                raise SystemExit('Unexpected odometry frames; no goal sent.')
            if publisher.get_subscription_count() < 1:
                continue
            goal = PoseStamped()
            goal.header.frame_id = 'world'
            goal.header.stamp = node.get_clock().now().to_msg()
            goal.pose.position.x = odom.pose.pose.position.x + .30
            goal.pose.position.y = odom.pose.pose.position.y
            goal.pose.position.z = odom.pose.pose.position.z
            goal.pose.orientation = odom.pose.pose.orientation
            publisher.publish(goal)
            print('Sent one planner goal: +0.30 m along planner X, heading retained.')
            print('Requested planner xyz:', [round(getattr(goal.pose.position,axis),6) for axis in ('x','y','z')])
            print('Use low-speed-3d to retain message Z; inspect Get goal at for map relocation.')
            end = time.monotonic() + .5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            break
        else:
            raise SystemExit('No fresh pose/state or goal subscriber; no goal sent.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
