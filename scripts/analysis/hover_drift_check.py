#!/usr/bin/env python3
"""Read-only, bounded, disarmed ROS 2 hover-drift evidence collection.

Observe an already running stack. Never publish, start nodes, pull/write PX4
parameters, arm, or change modes. Missing optional vision streams are allowed.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import time
import zipfile


RAW = '/Odometry'
POSES = (RAW, '/super_debug/aligned_pose_cov', '/mavros/vision_pose/pose_cov',
         '/mavros/local_position/pose')
IMUS = ('/livox/imu', '/mavros/imu/data')
CLOUD = '/cloud_registered'
STATE, EXTENDED = '/mavros/state', '/mavros/extended_state'
TOPICS = POSES + IMUS + (CLOUD, STATE, EXTENDED)
LIO_PARAMETERS = ['point_filter_num', 'filter_size_surf', 'filter_size_map',
                  'max_iteration', 'preprocess.blind', 'preprocess.scan_rate',
                  'common.time_sync_en', 'common.time_offset_lidar_to_imu',
                  'common.lid_topic', 'common.imu_topic', 'mapping.extrinsic_est_en',
                  'mapping.extrinsic_T', 'mapping.extrinsic_R', 'publish.dense_publish_en',
                  'publish.scan_publish_en', 'publish.scan_bodyframe_pub_en']
PX4_PARAMETERS = ['MPC_THR_HOVER', 'MPC_USE_HTE', 'MPC_XY_P', 'MPC_XY_VEL_P_ACC',
                  'MPC_XY_VEL_I_ACC', 'MPC_XY_VEL_D_ACC', 'EKF2_EV_CTRL',
                  'EKF2_HGT_REF', 'EKF2_EV_DELAY', 'EKF2_EV_NOISE_MD',
                  'EKF2_EVP_NOISE', 'EKF2_EVA_NOISE', 'EKF2_EV_POS_X',
                  'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z']


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    return value


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Quaternion is not finite XYZW')
    norm = math.sqrt(sum(v*v for v in q))
    if abs(norm-1.) > .02:
        raise ValueError('Quaternion is not normalized')
    x, y, z, w = [v/norm for v in q]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def summarize_stream(rows, started, ended):
    out = {'count': len(rows)}
    if not rows:
        return out
    receipts = [r['received_monotonic_s'] for r in rows]
    stamps = [r['stamp_ns'] for r in rows]
    ages = [(r['received_ros_ns']-r['stamp_ns'])/1e9 for r in rows]
    span = receipts[-1]-receipts[0]
    out.update(receipt_span_s=span, receipt_hz=(len(rows)-1)/span if span > 0 else None,
               first_receive_delay_s=receipts[0]-started,
               max_receive_gap_s=max((b-a for a, b in zip(receipts, receipts[1:])), default=None),
               end_receive_age_s=ended-receipts[-1],
               header_age_s={'min': min(ages), 'mean': sum(ages)/len(ages), 'max': max(ages)},
               invalid_stamp_count=sum(v <= 0 for v in stamps),
               nonincreasing_stamp_count=sum(b <= a for a, b in zip(stamps, stamps[1:])),
               frames=sorted({r.get('frame', '') for r in rows}))
    return out


def summarize_pose(rows, started, ended):
    out = summarize_stream(rows, started, ended)
    valid = []
    for row in rows:
        try:
            if len(row['position']) != 3 or not all(math.isfinite(v) for v in row['position']):
                raise ValueError('Position is invalid')
            valid.append((row, quaternion_yaw(row['quaternion'])))
        except (ValueError, TypeError, KeyError):
            continue
    out.update(valid_pose_count=len(valid), invalid_pose_count=len(rows)-len(valid))
    if not valid:
        return out
    out['child_frames'] = sorted({r.get('child_frame', '') for r, _ in valid})
    out['coordinate_frame_consistent'] = len(out['frames']) == 1 and len(out['child_frames']) == 1
    if not out['coordinate_frame_consistent']:
        return out  # Mixing coordinate systems cannot measure a physical displacement.
    first = [r for r, _ in valid if r['received_monotonic_s'] <= valid[0][0]['received_monotonic_s']+5.]
    last = [r for r, _ in valid if r['received_monotonic_s'] >= valid[-1][0]['received_monotonic_s']-5.]
    def xyz_mean(window):
        return [sum(r['position'][i] for r in window)/len(window) for i in range(3)]
    initial, final = xyz_mean(first), xyz_mean(last)
    delta = [b-a for a, b in zip(initial, final)]
    unwrapped = [valid[0][1]]
    for (_, previous), (_, current) in zip(valid, valid[1:]):
        unwrapped.append(unwrapped[-1]+wrap_angle(current-previous))
    first_yaw = [a for (r, _), a in zip(valid, unwrapped) if r['received_monotonic_s'] <= valid[0][0]['received_monotonic_s']+5.]
    last_yaw = [a for (r, _), a in zip(valid, unwrapped) if r['received_monotonic_s'] >= valid[-1][0]['received_monotonic_s']-5.]
    initial_yaw, final_yaw = sum(first_yaw)/len(first_yaw), sum(last_yaw)/len(last_yaw)
    out.update(xyz_range_m=[max(r['position'][i] for r, _ in valid)-min(r['position'][i] for r, _ in valid) for i in range(3)],
               initial_mean_xyz_m=initial, final_mean_xyz_m=final,
               mean_window_displacement_xyz_m=delta,
               mean_window_displacement_norm_m=math.sqrt(sum(v*v for v in delta)),
               max_displacement_from_initial_mean_m=max(math.sqrt(sum((p-b)**2 for p, b in zip(r['position'], initial))) for r, _ in valid),
               yaw_initial_mean_deg=math.degrees(wrap_angle(initial_yaw)),
               yaw_final_mean_deg=math.degrees(wrap_angle(final_yaw)),
               yaw_drift_deg=math.degrees(final_yaw-initial_yaw),
               yaw_range_deg=math.degrees(max(unwrapped)-min(unwrapped)),
               mean_window_seconds=5., mean_windows_may_overlap=(valid[-1][0]['received_monotonic_s']-valid[0][0]['received_monotonic_s'] < 10.))
    return out


def summarize_imu(rows, started, ended):
    out = summarize_stream(rows, started, ended)
    for field in ('linear_acceleration', 'angular_velocity'):
        vectors = [r[field] for r in rows if field in r and len(r[field]) == 3 and all(math.isfinite(v) for v in r[field])]
        stats = {'valid_count': len(vectors), 'invalid_count': len(rows)-len(vectors)}
        if vectors:
            means = [sum(v[i] for v in vectors)/len(vectors) for i in range(3)]
            stats.update(mean_xyz=means, std_xyz=[math.sqrt(sum((v[i]-means[i])**2 for v in vectors)/len(vectors)) for i in range(3)])
        out[field] = stats
    out['note'] = 'Population standard deviation of received samples; different sensor rates/filters are not directly comparable.'
    return out


def save_artifacts(folder, report, rows, started, ended):
    report['streams'] = {t: (summarize_pose if t in POSES else summarize_imu if t in IMUS else summarize_stream)(rows.get(t, []), started, ended) for t in TOPICS}
    report['missing_optional_topics'] = [t for t in POSES[1:]+IMUS+(CLOUD,) if not rows.get(t)]
    issues = report.setdefault('data_issues', [])
    warnings = report.setdefault('stream_warnings', [])
    for topic, summary in report['streams'].items():
        for key in ('invalid_stamp_count', 'nonincreasing_stamp_count', 'invalid_pose_count'):
            if summary.get(key, 0):
                issues.append(f'{topic}: {key}={summary[key]}')
        if len(summary.get('frames', [])) > 1 or len(summary.get('child_frames', [])) > 1:
            issues.append(f'{topic}: frame changed; displacement is not purely physical motion')
        if topic in POSES+IMUS+(CLOUD,) and summary['count']:
            for key in ('first_receive_delay_s', 'max_receive_gap_s', 'end_receive_age_s'):
                if (summary.get(key) or 0.) > 1.:
                    (issues if topic == RAW else warnings).append(f'{topic}: {key}={summary[key]:.3f}s exceeds 1s stream-continuity check')
    if len(rows.get(RAW, [])) < 2:
        issues.append('/Odometry: fewer than two captured samples')
    if report.get('result') == 'CAPTURE_COMPLETE' and (issues or len(rows.get(RAW, [])) < 2):
        report['result'] = 'INCOMPLETE_DATA'
    report['data_issues'] = list(dict.fromkeys(issues))
    report['stream_warnings'] = list(dict.fromkeys(warnings))
    (folder/'report.json').write_text(json.dumps(finite_json(report), indent=2, allow_nan=False)+'\n')
    lines = [f"RESULT: {report.get('result', 'ERROR')}", 'Flight readiness and physical accuracy: NOT ASSESSED',
             'Read-only observation; no automatic pass threshold. FC and vision can share the same source.']
    if report.get('error'):
        lines.append('ERROR: '+report['error'])
    for topic, summary in report['streams'].items():
        lines.append('\n'+topic+'\n'+json.dumps(finite_json(summary), indent=2, allow_nan=False))
    lines += ['\nData issues: '+json.dumps(report['data_issues']),
              'Stream warnings: '+json.dumps(report['stream_warnings']),
              'Missing optional topics: '+', '.join(report['missing_optional_topics']),
              'Parameter readbacks and errors: report.json; raw clouds were not saved.']
    (folder/'summary.txt').write_text('\n'.join(lines)+'\n')
    with zipfile.ZipFile(folder/'diagnostic.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(folder.iterdir()):
            if path.is_file() and path.name != 'diagnostic.zip':
                archive.write(path, arcname=path.name)


class Collector:
    def __init__(self, folder, report):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        from mavros_msgs.msg import State, ExtendedState
        from rcl_interfaces.srv import GetParameters
        self.ros, self.GetParameters = rclpy, GetParameters
        self.node = rclpy.create_node('hover_drift_check')
        self.report, self.rows = report, {t: [] for t in TOPICS}
        self.output = (folder/'telemetry.jsonl').open('w')
        self.phase, self.started, self.ended = 'waiting', time.monotonic(), time.monotonic()
        self.latest, self.graph, self.pending, self.attempted = {}, {}, {}, set()
        self.raw_gid, self.last_graph, self.ground_fault = None, -math.inf, None
        self.ON_GROUND = ExtendedState.LANDED_STATE_ON_GROUND
        self.IN_AIR = {ExtendedState.LANDED_STATE_IN_AIR, ExtendedState.LANDED_STATE_TAKEOFF, ExtendedState.LANDED_STATE_LANDING}
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        classes = dict(zip(POSES, (Odometry, PoseWithCovarianceStamped, PoseWithCovarianceStamped, PoseStamped)))
        classes.update({t: Imu for t in IMUS})
        classes.update({CLOUD: PointCloud2, STATE: State, EXTENDED: ExtendedState})
        self.subscriptions = [self.node.create_subscription(cls, topic, self.callback(topic), qos) for topic, cls in classes.items()]

    def issue(self, message):
        if message not in self.report['data_issues']:
            self.report['data_issues'].append(message)

    def callback(self, topic):
        def receive(msg):
            row = dict(topic=topic, phase=self.phase, received_monotonic_s=time.monotonic(),
                       received_ros_ns=self.node.get_clock().now().nanoseconds,
                       stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                       frame=msg.header.frame_id)
            if topic in POSES:
                pose = msg.pose if topic == POSES[-1] else msg.pose.pose
                row['position'] = [float(getattr(pose.position, k)) for k in ('x', 'y', 'z')]
                row['quaternion'] = [float(getattr(pose.orientation, k)) for k in ('x', 'y', 'z', 'w')]
                row['covariance'] = None if topic == POSES[-1] else list(msg.pose.covariance)
                row['child_frame'] = getattr(msg, 'child_frame_id', '')
                try:
                    row['yaw_rad'] = quaternion_yaw(row['quaternion'])
                except ValueError:
                    row['yaw_rad'] = None
            elif topic in IMUS:
                row['angular_velocity'] = [float(getattr(msg.angular_velocity, k)) for k in ('x', 'y', 'z')]
                row['linear_acceleration'] = [float(getattr(msg.linear_acceleration, k)) for k in ('x', 'y', 'z')]
            elif topic == CLOUD:
                row.update(width=msg.width, height=msg.height, point_step=msg.point_step,
                           row_step=msg.row_step, is_dense=msg.is_dense,
                           data_bytes=len(msg.data), fields=[dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count) for f in msg.fields])
            elif topic == STATE:
                row.update(connected=bool(msg.connected), armed=bool(msg.armed), mode=msg.mode)
                if msg.armed:
                    self.ground_fault = 'Armed FC state observed; capture stopped'
            else:
                row['landed_state'] = msg.landed_state
                if msg.landed_state in self.IN_AIR:
                    self.ground_fault = 'Airborne/takeoff/landing FC state observed; capture stopped'
            self.latest[topic] = row
            if self.phase == 'capture':
                self.rows[topic].append(row)
            self.output.write(json.dumps(finite_json(row), allow_nan=False)+'\n')
        return receive

    def ground_ready(self):
        now, ros_now = time.monotonic(), self.node.get_clock().now().nanoseconds
        for topic in (STATE, EXTENDED):
            row = self.latest.get(topic)
            if not row or now-row['received_monotonic_s'] > 2.5 or row['stamp_ns'] <= 0:
                return False
            if not -.5 <= (ros_now-row['stamp_ns'])/1e9 <= 2.5:
                return False
        return self.latest[STATE]['connected'] and not self.latest[STATE]['armed'] and self.latest[EXTENDED]['landed_state'] == self.ON_GROUND

    def update_graph(self):
        self.last_graph = time.monotonic()
        for topic in TOPICS:
            infos = self.node.get_publishers_info_by_topic(topic)
            self.graph[topic] = [dict(node=i.node_name, namespace=i.node_namespace,
                                     gid=bytes(i.endpoint_gid).hex(), reliability=i.qos_profile.reliability.name) for i in infos]
        raw = self.graph[RAW]
        if len(raw) > 1:
            self.issue('Multiple /Odometry publishers; source is ambiguous')
        if len(raw) == 1:
            gid = raw[0]['gid']
            if self.phase == 'capture' and self.raw_gid is not None and gid != self.raw_gid:
                self.issue('/Odometry publisher changed during capture')
            if self.phase != 'capture' or self.raw_gid is None:
                self.raw_gid = gid
        self.report['publishers_at_end'] = self.graph
        self.request_parameters()

    def request_parameters(self):
        raw = self.graph.get(RAW, [])
        candidates = [(r['node'], r['namespace']) for r in raw]
        if not candidates:
            candidates = [(name, ns) for name, ns in self.node.get_node_names_and_namespaces()
                          if 'fastlio' in name.lower() or 'laser_mapping' in name.lower()]
        if len(candidates) == 1:
            name, namespace = candidates[0]
            self.report['fastlio_parameter_node'] = namespace.rstrip('/')+'/'+name
        targets = [('fastlio', self.report.get('fastlio_parameter_node'), LIO_PARAMETERS),
                   ('px4', '/mavros/param', PX4_PARAMETERS)]
        for label, node, names in targets:
            if label in self.attempted or node is None:
                continue
            service = node+'/get_parameters'
            client = self.node.create_client(self.GetParameters, service)
            if not client.service_is_ready():
                self.node.destroy_client(client)
                continue
            self.attempted.add(label)
            result = dict(service=service, requested=names, status='PENDING', values={})
            self.report['parameter_readbacks'][label] = result
            futures = {}
            # One unknown name can make ROS return an empty result for a whole
            # batch. Separate bounded reads preserve the supported parameters.
            for name in names:
                try:
                    futures[name] = client.call_async(self.GetParameters.Request(names=[name]))
                except Exception as exc:
                    result['values'][name] = {'error': f'{type(exc).__name__}: {exc}'}
            self.pending[label] = (client, futures, time.monotonic())

    def parameter_progress(self, finish=False):
        attrs = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
                 5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
                 8: 'double_array_value', 9: 'string_array_value'}
        for label, (client, futures, sent) in list(self.pending.items()):
            result = self.report['parameter_readbacks'][label]
            for name, future in list(futures.items()):
                if not future.done() and not finish and time.monotonic()-sent <= 5.:
                    continue
                try:
                    if not future.done():
                        future.cancel()
                        raise TimeoutError('Capture ended before response' if finish else 'GetParameters response exceeded 5 seconds')
                    values = future.result().values
                    if len(values) != 1 or values[0].type not in attrs:
                        raise ValueError('Missing response or parameter not set')
                    value = values[0]
                    data = getattr(value, attrs[value.type])
                    result['values'][name] = {'type': value.type, 'value': list(data) if value.type >= 5 else data}
                except Exception as exc:
                    result['values'][name] = {'error': f'{type(exc).__name__}: {exc}'}
                del futures[name]
            if not futures:
                result['status'] = 'PARTIAL' if any('error' in v for v in result['values'].values()) else 'RECEIVED'
                self.node.destroy_client(client)
                del self.pending[label]

    def poll(self):
        self.ros.spin_once(self.node, timeout_sec=.02)
        if self.ground_fault:
            raise RuntimeError(self.ground_fault)
        if time.monotonic()-self.last_graph >= 1.:
            self.update_graph()
        self.parameter_progress()
        if self.phase == 'capture' and not self.ground_ready():
            raise RuntimeError('Fresh connected/disarmed/on-ground FC telemetry lost; partial capture saved')

    def run(self, duration, wait):
        deadline, next_print = time.monotonic()+wait, 0.
        while time.monotonic() < deadline:
            self.poll()
            raw = self.latest.get(RAW)
            if self.ground_ready() and raw and time.monotonic()-raw['received_monotonic_s'] <= 1. and len(self.graph.get(RAW, [])) == 1:
                break
            if time.monotonic() >= next_print:
                print('WAIT: need one live /Odometry source and fresh connected/disarmed/on-ground MAVROS telemetry. No nodes are started.', flush=True)
                next_print = time.monotonic()+5.
        else:
            raise TimeoutError(f'Required streams/ground state did not become ready within {wait:g}s')
        self.phase, self.started = 'capture', time.monotonic()
        self.raw_gid = self.graph[RAW][0]['gid']
        self.report['publishers_at_start'] = dict(self.graph)
        print(f'CAPTURE: keep aircraft physically stationary with motors stopped for {duration:g}s.', flush=True)
        end, heartbeat = self.started+duration, self.started+10.
        while time.monotonic() < end:
            self.poll()
            if time.monotonic() >= heartbeat:
                print(f'CAPTURE {time.monotonic()-self.started:.0f}/{duration:g}s; odometry={len(self.rows[RAW])}, cloud metadata={len(self.rows[CLOUD])}', flush=True)
                self.output.flush()
                heartbeat = time.monotonic()+10.
        self.report['result'] = 'CAPTURE_COMPLETE'

    def close(self):
        self.ended = time.monotonic()
        self.parameter_progress(finish=True)
        for label in ('fastlio', 'px4'):
            self.report['parameter_readbacks'].setdefault(label, {'status': 'UNAVAILABLE', 'requested': LIO_PARAMETERS if label == 'fastlio' else PX4_PARAMETERS, 'error': 'Parameter node/service not discovered; no write or parameter pull attempted'})
        self.output.close()
        self.node.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=120., help='capture seconds, 5 to 300')
    parser.add_argument('--wait', type=float, default=30., help='startup deadline seconds, 1 to 120')
    parser.add_argument('--output', type=Path, help='new report directory (must not already exist)')
    args = parser.parse_args(argv)
    for name, lower, upper in (('duration', 5, 300), ('wait', 1, 120)):
        value = getattr(args, name)
        if not math.isfinite(value) or not lower <= value <= upper:
            parser.error(f'--{name} must be finite and between {lower} and {upper}')
    folder = (args.output or Path.home()/'Downloads/hover_drift_checks'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')).expanduser()
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, result='ERROR', requested_duration_s=args.duration,
                  startup_wait_s=args.wait, parameter_readbacks={}, data_issues=[],
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  flight_readiness='NOT_ASSESSED', parameter_writes=False, raw_cloud_saved=False,
                  notes=['Receive times include Python scheduling; message age retains outliers.',
                         'PX4 values are read from the MAVROS parameter cache without a forced refresh; not an independent direct FC readback.',
                         'Stationary means independently physically fixed, not merely a hover command.',
                         'Position/yaw statistics are estimates, not independent physical truth.'])
    collector, ros = None, None
    started = ended = time.monotonic()
    print('REPORT DIRECTORY:', folder, flush=True)
    try:
        import rclpy
        ros = rclpy
        ros.init()
        collector = Collector(folder, report)
        collector.run(args.duration, args.wait)
    except KeyboardInterrupt:
        report.update(result='CANCELLED', error='Operator interrupted; partial data preserved')
    except Exception as exc:
        report.update(result='ERROR', error=f'{type(exc).__name__}: {exc}')
    finally:
        rows = {t: [] for t in TOPICS}
        if collector is not None:
            collector.ended = time.monotonic()
            try:
                collector.close()
            except Exception as exc:
                report['cleanup_error'] = f'{type(exc).__name__}: {exc}'
                report['result'] = 'ERROR'
                collector.output.close()
            rows, started, ended = collector.rows, collector.started, collector.ended
        if ros is not None and ros.ok():
            try:
                ros.shutdown()
            except Exception as exc:
                report['shutdown_error'] = f'{type(exc).__name__}: {exc}'
        (folder/'telemetry.jsonl').touch(exist_ok=True)
        report['capture_elapsed_s'] = max(0., ended-started) if collector is not None and collector.phase == 'capture' else 0.
        save_artifacts(folder, report, rows, started, ended)
    print('RESULT:', report['result'], flush=True)
    raw = report['streams'][RAW]
    print('RAW /Odometry:', json.dumps({k: raw[k] for k in ('count', 'receipt_hz', 'xyz_range_m', 'mean_window_displacement_xyz_m', 'yaw_drift_deg', 'end_receive_age_s') if k in raw}), flush=True)
    for label, names in (('fastlio', ('mapping.extrinsic_est_en', 'filter_size_surf', 'filter_size_map', 'publish.dense_publish_en')), ('px4', ('MPC_THR_HOVER', 'EKF2_EV_CTRL', 'EKF2_HGT_REF'))):
        values = report['parameter_readbacks'].get(label, {}).get('values', {})
        print(label.upper(), json.dumps({name: values.get(name, {'error': 'unavailable'}) for name in names}), flush=True)
    if report.get('error'):
        print('ERROR:', report['error'], flush=True)
    print('SUMMARY:', folder/'summary.txt', '\nBUNDLE:', folder/'diagnostic.zip', flush=True)
    return 0 if report['result'] == 'CAPTURE_COMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
