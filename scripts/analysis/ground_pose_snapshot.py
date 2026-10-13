#!/usr/bin/env python3
"""Read-only timed position/yaw observations for stationary ground comparisons."""
import argparse
import math
import time


def summarize(rows):
    """Rows: x, y, z, yaw, source age, monotonic receipt time."""
    if len(rows)<2:
        raise ValueError('Need at least two fresh valid samples')
    initial=[r for r in rows if r[5]<=rows[0][5]+2.]
    final=[r for r in rows if r[5]>=rows[-1][5]-2.]
    def mean_xyz(window):
        return [sum(r[i] for r in window)/len(window) for i in range(3)]
    first,last=mean_xyz(initial),mean_xyz(final)
    yaw=math.atan2(sum(math.sin(r[3]) for r in initial),sum(math.cos(r[3]) for r in initial))
    c,s=math.cos(yaw),math.sin(yaw)
    offsets=[]
    for row in rows:
        dx,dy,dz=[row[i]-first[i] for i in range(3)]
        offsets.append((c*dx+s*dy,-s*dx+c*dy,dz))
    return dict(count=len(rows),span=rows[-1][5]-rows[0][5],
                mean_xyz=mean_xyz(rows),initial_xyz=first,final_xyz=last,
                delta_xyz=[b-a for a,b in zip(first,last)],
                range_xyz=[max(r[i] for r in rows)-min(r[i] for r in rows) for i in range(3)],
                max_forward_left_up=[max(abs(r[i]) for r in offsets) for i in range(3)],
                axis_yaw=yaw,
                mean_yaw=math.atan2(sum(math.sin(r[3]) for r in rows),sum(math.cos(r[3]) for r in rows)),
                age_mean=sum(r[4] for r in rows)/len(rows),age_max=max(r[4] for r in rows),
                receive_gap_max=max(b[5]-a[5] for a,b in zip(rows,rows[1:])))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=5.,help='observation seconds, 5 to 300; default 5')
    args=parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 5<=args.duration<=300:
        parser.error('--duration must be finite and between 5 and 300 seconds')
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
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if abs(norm - 1) > .02:
            return
        x, y, z, w = [value / norm for value in [q.x, q.y, q.z, q.w]]
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        samples[topic].append((p.x, p.y, p.z, yaw, age, time.monotonic()))

    subscriptions = [node.create_subscription(
        cls, topic, lambda msg, topic=topic, nested=nested: record(msg, topic, nested), qos)
        for topic, cls, nested in topics]
    try:
        start=time.monotonic()
        print(f'Read-only stationary observation: {args.duration:g}s. No flight or parameter requests.',flush=True)
        print('Each source uses its initial two-second mean position and yaw. FC fusion is not independent physical accuracy evidence.',flush=True)
        print('ROS start stamp ns:',node.get_clock().now().nanoseconds,flush=True)
        end = start + args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        for topic, _, _ in topics:
            rows = samples[topic]
            print(topic, 'n=', len(rows))
            if len(rows)<2:
                print('  No fresh valid samples received')
                continue
            report=summarize(rows)
            print(f'  sampled span: {report["span"]:.3f}s; receive gap max: {report["receive_gap_max"]:.3f}s; last valid receive age: {time.monotonic()-rows[-1][5]:.3f}s')
            print('  mean XYZ m:',[round(v,4) for v in report['mean_xyz']])
            print('  initial/final two-second mean XYZ m:',[round(v,4) for v in report['initial_xyz']], [round(v,4) for v in report['final_xyz']])
            print('  mean-window change XYZ cm:',[round(v*100,3) for v in report['delta_xyz']])
            print('  min/max range XYZ cm:',[round(v*100,3) for v in report['range_xyz']])
            print('  max excursion forward/left/up cm:',[round(v*100,3) for v in report['max_forward_left_up']])
            print(f'  fixed axis yaw: {math.degrees(report["axis_yaw"]):.3f} deg; mean yaw: {math.degrees(report["mean_yaw"]):.3f} deg')
            print(f'  age mean/max s: {report["age_mean"]:.3f}/{report["age_max"]:.3f}')
        print('ROS end stamp ns:',node.get_clock().now().nanoseconds)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
