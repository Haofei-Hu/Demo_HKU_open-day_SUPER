#!/usr/bin/env python3
"""Read-only five-second XY/yaw averages for settled ground comparisons."""
import math
import time


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

    rclpy.init()
    node = rclpy.create_node('ground_pose_snapshot')
    topics = [('/super_debug/aligned_pose_cov', PoseWithCovarianceStamped, True),
              ('/mavros/local_position/pose', PoseStamped, False)]
    samples = {topic: [] for topic, _, _ in topics}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def record(msg, topic, nested):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or not -.05 <= age <= .5:
            return
        pose = msg.pose.pose if nested else msg.pose
        p, q = pose.position, pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if abs(norm - 1) > .02:
            return
        x, y, z, w = [value / norm for value in [q.x, q.y, q.z, q.w]]
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        samples[topic].append((p.x, p.y, yaw, age))

    subscriptions = [node.create_subscription(
        cls, topic, lambda msg, topic=topic, nested=nested: record(msg, topic, nested), qos)
        for topic, cls, nested in topics]
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        for topic, _, _ in topics:
            rows = samples[topic]
            print(topic, 'n=', len(rows))
            if not rows:
                print('  No fresh valid samples received')
                continue
            count = len(rows)
            yaw = math.atan2(sum(math.sin(row[2]) for row in rows),
                             sum(math.cos(row[2]) for row in rows))
            print(f'  mean XY m: {sum(row[0] for row in rows)/count:.4f}, '
                  f'{sum(row[1] for row in rows)/count:.4f}')
            print(f'  mean yaw deg: {math.degrees(yaw):.3f}')
            print(f'  age mean/max s: {sum(row[3] for row in rows)/count:.3f}/'
                  f'{max(row[3] for row in rows):.3f}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
