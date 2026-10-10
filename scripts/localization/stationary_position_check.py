import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy


from aligned_pose_preview import R_WC, Q_BI, T_BI, rotation

# Use the same calibration as the colocated preview, including its lever arm.
T_IC = -rotation(Q_BI).T @ T_BI

def main():
    rclpy.init()
    node = rclpy.create_node('stationary_position_check')
    names = ['/Odometry', '/super_debug/aligned_pose_cov']
    samples = {name: [] for name in names}
    cached = {name: {} for name in names}
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


    def record(msg, name):
        t = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        p = np.array([p.x, p.y, p.z])
        q = np.array([q.x, q.y, q.z, q.w])
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):
            return
        if np.linalg.norm(q) < 1e-8:
            return
        age = (node.get_clock().now().nanoseconds-t)/1e9
        samples[name].append((p, age))
        cached[name][t] = (p, q)


    subscriptions = [node.create_subscription(
        cls, name, lambda msg, name=name: record(msg, name), qos)
        for name, cls in zip(names, [Odometry, PoseWithCovarianceStamped])]
    try:
        end = time.monotonic()+15
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        for name in names:
            rows = samples[name]
            print(name, 'samples=', len(rows))
            if rows:
                positions = np.array([row[0] for row in rows])
                ages = [row[1] for row in rows]
                print('  first -> last delta xyz m:',
                      (positions[-1]-positions[0]).round(4).tolist())
                print('  min -> max range xyz m:',
                      np.ptp(positions, axis=0).round(4).tolist())
                print(f'  age mean/max: {np.mean(ages):.3f}/{max(ages):.3f}s')
        common = sorted(cached[names[0]].keys() & cached[names[1]].keys())
        origins = []
        for t in common:
            p, q = cached[names[0]][t]
            preview, _ = cached[names[1]][t]
            origins.append(R_WC @ (p+rotation(q) @ T_IC)-preview)
        print('Matched stamps:', len(common))
        if origins:
            print('Inferred origin variation xyz m:',
                  np.ptp(np.array(origins), axis=0).round(9).tolist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
