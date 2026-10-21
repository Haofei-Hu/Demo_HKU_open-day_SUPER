#!/usr/bin/env python3
"""Read-only IMU source capture; never starts nodes or changes FC/LIO parameters."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time
import zipfile
from datetime import datetime

TOPICS = ('/livox/imu', '/mavros/imu/data_raw', '/mavros/imu/data')


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    return values[round((len(values) - 1) * fraction)]


def summarize(rows):
    if not rows:
        return {'count': 0, 'status': 'NO_DATA'}
    dt = [(b['stamp_ns'] - a['stamp_ns']) / 1e9
          for a, b in zip(rows, rows[1:])]
    positive = [d for d in dt if d > 0]
    elapsed = rows[-1]['receive_mono_s'] - rows[0]['receive_mono_s']
    norms = {}
    for name in ('accel', 'gyro'):
        values = [math.sqrt(sum(v*v for v in r[name])) for r in rows
                  if all(math.isfinite(v) for v in r[name])]
        norms[name + '_norm_median'] = statistics.median(values) if values else None
        norms[name + '_norm_p99'] = percentile(values, .99)
    ages = [(r['receive_ros_ns'] - r['stamp_ns']) / 1e9 for r in rows]
    return dict(count=len(rows), status='RECEIVED',
                frames=sorted(set(r['frame'] for r in rows)),
                receipt_hz=(len(rows)-1)/elapsed if elapsed > 0 else None,
                stamp_hz_from_median_dt=1/statistics.median(positive) if positive else None,
                stamp_gap_max_s=max(positive) if positive else None,
                stamp_gap_p99_s=percentile(positive, .99),
                duplicate_stamp_count=sum(d == 0 for d in dt),
                backwards_stamp_count=sum(d < 0 for d in dt),
                zero_stamp_count=sum(r['stamp_ns'] == 0 for r in rows),
                age_median_s=statistics.median(ages), age_p99_s=percentile(ages, .99),
                age_min_s=min(ages),
                accel_unavailable_count=sum(r['accel_cov0'] == -1 for r in rows),
                gyro_unavailable_count=sum(r['gyro_cov0'] == -1 for r in rows),
                nonfinite_count=sum(not all(math.isfinite(v) for v in r['accel']+r['gyro']) for r in rows),
                **norms)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 5 <= args.duration <= 120:
        parser.error('--duration must be between 5 and 120 seconds')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    rclpy.init()
    node = rclpy.create_node('imu_source_check')
    out = Path.home() / 'Downloads' / 'imu_source_checks' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True)
    samples = {topic: [] for topic in TOPICS}
    subscriptions = []
    def receive(msg, topic):
        a, g = msg.linear_acceleration, msg.angular_velocity
        samples[topic].append(dict(
            stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
            receive_ros_ns=node.get_clock().now().nanoseconds,
            receive_mono_s=time.monotonic(), frame=msg.header.frame_id,
            accel=[a.x,a.y,a.z], gyro=[g.x,g.y,g.z],
            accel_cov0=float(msg.linear_acceleration_covariance[0]),
            gyro_cov0=float(msg.angular_velocity_covariance[0])))
    for topic in TOPICS:
        subscriptions.append(node.create_subscription(
            Imu, topic, lambda msg, t=topic: receive(msg,t), qos_profile_sensor_data))
    print('READ ONLY: keep aircraft stationary, motors stopped; existing Livox and MAVROS must be running.', flush=True)
    start = time.monotonic()
    interrupted = False
    try:
        while time.monotonic()-start < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        interrupted = True
    report = {'duration_s': time.monotonic()-start, 'interrupted': interrupted,
              'note': 'Receipt ages do not measure calibrated inter-sensor time offset. Acceleration units are source-dependent; Livox may use g. No source is approved for flight by this report.',
              'topics': {}}
    for topic in TOPICS:
        result = summarize(samples[topic])
        result['publishers'] = [dict(node=x.node_name, namespace=x.node_namespace,
                                    type=x.topic_type)
                                for x in node.get_publishers_info_by_topic(topic)]
        report['topics'][topic] = result
        print(topic + ': ' + json.dumps(result, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    (out / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    with (out / 'samples.jsonl').open('w') as stream:
        for topic, rows in samples.items():
            for row in rows:
                stream.write(json.dumps(dict(topic=topic, **row))+'\n')
    bundle = out / 'imu_sources.zip'
    with zipfile.ZipFile(bundle, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name in ('report.json', 'samples.jsonl'):
            archive.write(out/name, name)
    print('BUNDLE: ' + str(bundle), flush=True)


if __name__ == '__main__':
    main()
