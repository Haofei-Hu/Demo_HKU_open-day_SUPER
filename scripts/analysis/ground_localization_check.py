#!/usr/bin/env python3
"""Interactive disarmed localization measurements; subscribers and local files only."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import select
import subprocess
import sys
import time

RAW = '/Odometry'
ALIGNED = '/super_debug/aligned_pose_cov'
VISION = '/mavros/vision_pose/pose_cov'
LOCAL = '/mavros/local_position/pose'
TOPICS = (RAW, ALIGNED, VISION, LOCAL)
FRAMES = {RAW: 'camera_init', ALIGNED: 'px4_enu_preview', VISION: 'px4_enu_preview', LOCAL: 'map'}
CONTROL_TOPICS = (
    '/mavros/setpoint_raw/local', '/mavros/setpoint_raw/attitude',
    '/mavros/setpoint_position/local', '/mavros/setpoint_velocity/cmd_vel',
    '/mavros/setpoint_velocity/cmd_vel_unstamped',
    '/mavros/setpoint_attitude/attitude', '/mavros/setpoint_attitude/thrust',
)


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value


def save(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(finite_json(report), indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def yaw(q):
    x, y, z, w = q
    norm = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(norm) or abs(norm-1) > .02:
        raise ValueError('Invalid quaternion')
    x, y, z, w = (v/norm for v in (x, y, z, w))
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def flu(delta, heading):
    x, y, z = delta
    c, s = math.cos(heading), math.sin(heading)
    return [c*x+s*y, -s*x+c*y, z]


def mean(values):
    return sum(values)/len(values)


def statistics(rows):
    """Retain age/gap outliers; do not turn sampled covariance into accuracy."""
    out = dict(count=len(rows))
    if not rows:
        return out
    ages = [(r['received_ros_ns']-r['stamp_ns'])/1e9 for r in rows]
    gaps = [(b['received_monotonic_ns']-a['received_monotonic_ns'])/1e9
            for a, b in zip(rows, rows[1:])]
    valid = []
    for r in rows:
        try:
            if not all(math.isfinite(v) for v in r['position']+r['quaternion']):
                continue
            angle = yaw(r['quaternion'])
            valid.append((r, angle))
        except (ValueError, ZeroDivisionError):
            pass
    stamps = [r['stamp_ns'] for r in rows]
    out.update(header_age_s=dict(min=min(ages), mean=mean(ages), max=max(ages)),
               receive_gap_max_s=max(gaps, default=None),
               invalid_stamp_count=sum(v <= 0 for v in stamps),
               nonincreasing_stamp_count=sum(b <= a for a, b in zip(stamps, stamps[1:])),
               invalid_pose_count=len(rows)-len(valid),
               frames=sorted(set(r['frame'] for r in rows)),
               publisher_gids=sorted(set(r['publisher_gid'] for r in rows)))
    if valid:
        positions = [r['position'] for r, _ in valid]
        out.update(mean_xyz_m=[mean([p[i] for p in positions]) for i in range(3)],
                   range_xyz_m=[max(p[i] for p in positions)-min(p[i] for p in positions)
                                for i in range(3)],
                   mean_yaw_rad=math.atan2(mean([math.sin(a) for _, a in valid]),
                                          mean([math.cos(a) for _, a in valid])))
    covariances = [r['covariance_diagonal'] for r in rows if r['covariance_diagonal'] is not None]
    if covariances:
        out['covariance_diagonal_min_max'] = [
            [min(c[i] for c in covariances), max(c[i] for c in covariances)] for i in range(6)]
    return out


def qos_comparison(best, reliable):
    """Compare only stamps inside both observers' overlapping source-time span."""
    b = {r['stamp_ns']: r for r in best if r['stamp_ns'] > 0}
    r = {r['stamp_ns']: r for r in reliable if r['stamp_ns'] > 0}
    if not b or not r:
        return dict(comparable=False)
    lo, hi = max(min(b), min(r)), min(max(b), max(r))
    if hi < lo:
        return dict(comparable=False)
    bs, rs = {s for s in b if lo <= s <= hi}, {s for s in r if lo <= s <= hi}
    common = bs & rs
    delays = [(r[s]['received_monotonic_ns']-b[s]['received_monotonic_ns'])/1e6 for s in common]
    return dict(comparable=True, common_stamps=len(common),
                best_effort_only=len(bs-rs), reliable_only=len(rs-bs),
                reliable_minus_best_receipt_ms=dict(
                    min=min(delays), mean=mean(delays), max=max(delays)) if delays else None)


def comparison(baseline, current, heading, expected_xyz=None, expected_yaw_deg=0.):
    results = {}
    for topic in TOPICS:
        old, new = baseline.get(topic, {}), current.get(topic, {})
        if 'mean_xyz_m' not in old or 'mean_xyz_m' not in new:
            results[topic] = dict(available=False)
            continue
        delta = [b-a for a, b in zip(old['mean_xyz_m'], new['mean_xyz_m'])]
        result = dict(available=True, delta_source_xyz_m=delta,
                      yaw_change_deg=math.degrees(wrap(new['mean_yaw_rad']-old['mean_yaw_rad'])))
        if topic != RAW:
            measured = flu(delta, heading)
            result['delta_forward_left_up_m'] = measured
            if expected_xyz is not None:
                result['measured_minus_manual_forward_left_up_m'] = [
                    a-b for a, b in zip(measured, expected_xyz)]
            result['yaw_change_minus_manual_deg'] = math.degrees(wrap(
                math.radians(result['yaw_change_deg']-expected_yaw_deg)))
        results[topic] = result
    return results


class Observer:
    def __init__(self, folder):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from mavros_msgs.msg import State
        self.rclpy = rclpy
        self.node = rclpy.create_node('ground_localization_check')
        self.rows = {(topic, policy): [] for topic in TOPICS for policy in ('best_effort', 'reliable')}
        self.graph = {}
        self.source_gids = {}
        self.reliable_created = set()
        self.state = None
        self.state_at = 0.
        self.armed_seen = False
        self.active = False
        self.last_graph = -math.inf
        self.phase = 'waiting'
        self.raw_file = (folder/'samples.jsonl').open('w')
        self.types = {RAW: Odometry, ALIGNED: PoseWithCovarianceStamped,
                      VISION: PoseWithCovarianceStamped, LOCAL: PoseStamped}
        self.qos = {
            name: QoSProfile(depth=50, reliability=policy, durability=DurabilityPolicy.VOLATILE)
            for name, policy in [('best_effort', ReliabilityPolicy.BEST_EFFORT),
                                 ('reliable', ReliabilityPolicy.RELIABLE)]}
        self.subscriptions = [
            self.node.create_subscription(cls, topic, self.callback(topic, 'best_effort'),
                                          self.qos['best_effort'])
            for topic, cls in self.types.items()]
        self.subscriptions.append(self.node.create_subscription(
            State, '/mavros/state', self.receive_state, self.qos['best_effort']))

    def receive_state(self, msg):
        self.state, self.state_at = msg, time.monotonic()
        self.armed_seen |= bool(msg.armed)

    def callback(self, topic, policy):
        def receive(msg):
            pose = msg.pose if topic == LOCAL else msg.pose.pose
            cov = None if topic == LOCAL else [float(msg.pose.covariance[i]) for i in (0, 7, 14, 21, 28, 35)]
            row = dict(topic=topic, policy=policy, phase=self.phase,
                       received_monotonic_ns=time.monotonic_ns(),
                       received_ros_ns=self.node.get_clock().now().nanoseconds,
                       stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                       frame=msg.header.frame_id, publisher_gid=self.source_gids.get(topic, ''),
                       publisher_gid_basis='single-publisher graph discovery',
                       position=[float(getattr(pose.position, k)) for k in ('x', 'y', 'z')],
                       quaternion=[float(getattr(pose.orientation, k)) for k in ('x', 'y', 'z', 'w')],
                       covariance_diagonal=cov,
                       covariance=None if topic == LOCAL else list(msg.pose.covariance))
            self.rows[(topic, policy)].append(row)
            self.raw_file.write(json.dumps(finite_json(row), allow_nan=False)+'\n')
        return receive

    def graph_check(self):
        from rclpy.qos import ReliabilityPolicy
        self.last_graph = time.monotonic()
        controls = {t: self.node.count_publishers(t) for t in CONTROL_TOPICS}
        if any(controls.values()):
            raise RuntimeError('Control publishers are present. End the flight controller while '
                               'landed/disarmed before handling the aircraft: '+str(controls))
        for topic in TOPICS:
            infos = self.node.get_publishers_info_by_topic(topic)
            self.graph[topic] = [
                dict(node=i.node_name, namespace=i.node_namespace,
                     gid=bytes(i.endpoint_gid).hex(), reliability=i.qos_profile.reliability.name)
                for i in infos]
            if len(infos) > 1:
                raise RuntimeError('Multiple localization publishers on '+topic)
            if len(infos) == 1:
                gid = bytes(infos[0].endpoint_gid).hex()
                if self.active and self.source_gids.get(topic, gid) != gid:
                    raise RuntimeError('Localization source changed during manual measurements: '+topic)
                self.source_gids[topic] = gid
            if (len(infos) == 1 and infos[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
                    and topic not in self.reliable_created):
                self.subscriptions.append(self.node.create_subscription(
                    self.types[topic], topic, self.callback(topic, 'reliable'), self.qos['reliable']))
                self.reliable_created.add(topic)

    def poll(self):
        self.rclpy.spin_once(self.node, timeout_sec=.02)
        if self.armed_seen:
            raise RuntimeError('Armed FC state observed. Stop manual handling; diagnostic stopped.')
        if time.monotonic()-self.last_graph >= 1.:
            self.graph_check()
        if self.active and (self.state is None or not self.state.connected
                            or time.monotonic()-self.state_at > 2.5):
            raise RuntimeError('Fresh connected/disarmed FC state was lost; diagnostic stopped.')

    def wait_ready(self):
        last_print = -math.inf
        while True:
            self.poll()
            now = time.monotonic()
            missing = [t for t in TOPICS if not self.rows[(t, 'best_effort')]
                       or now-self.rows[(t, 'best_effort')][-1]['received_monotonic_ns']/1e9 > 1.
                       or len(self.graph.get(t, [])) != 1]
            state_ok = self.state is not None and self.state.connected and now-self.state_at <= 2.5
            if not missing and state_ok:
                self.active = True
                return
            if now-last_print > 3:
                print('WAIT: '+(', '.join(missing) or 'localization received')+
                      ('; connected/disarmed state needed' if not state_ok else '')+
                      '. Ctrl+C cancels; this tool does not start localization.', flush=True)
                last_print = now

    def ask(self, prompt, numeric=False):
        print(prompt, flush=True)
        while True:
            self.poll()
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    raise EOFError('Interactive terminal closed')
                if not numeric:
                    return
                try:
                    value = float(line.strip())
                    if math.isfinite(value) and value > 0:
                        return value
                except ValueError:
                    pass
                print('Enter the actual positive measurement as a number.', flush=True)

    def capture(self, name, seconds):
        self.phase = name
        starts = {key: len(rows) for key, rows in self.rows.items()}
        print(f'CAPTURE {name}: keep still for {seconds:g} seconds.', flush=True)
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.poll()
        sampled = {key: rows[starts[key]:] for key, rows in self.rows.items()}
        self.raw_file.flush()
        result = dict(name=name, topics={t: statistics(sampled[(t, 'best_effort')]) for t in TOPICS},
                      reliable_topics={t: statistics(sampled[(t, 'reliable')]) for t in TOPICS},
                      qos_comparison={t: qos_comparison(sampled[(t, 'best_effort')],
                                                       sampled[(t, 'reliable')]) for t in TOPICS},
                      publishers=dict(self.graph))
        self.phase = name+'_moving'
        return result

    def close(self):
        self.raw_file.close()
        self.node.destroy_node()


def prepare(folder, scripts, restart):
    """Use existing guarded core/vision preparation; never launch a controller."""
    sys.path.insert(0, str(scripts))
    import super_session as manager
    from super_ground_test import alignment_quaternion
    session = manager.Session(manager.arguments(['start', '--scripts-dir', str(scripts)]))
    def action(name, *extra):
        command = [sys.executable, str(scripts/'super_session.py'),
                   '--scripts-dir', str(scripts), name]
        if name == 'vision':
            command.append('--')
        command.extend(map(str, extra))
        log = folder/('prepare_'+name+'.log')
        print('PREPARE:', name, 'LOG:', log, flush=True)
        with log.open('w') as handle:
            result = subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT)
        print(log.read_text(errors='replace'), end='', flush=True)
        if result.returncode:
            raise RuntimeError(f'{name} failed; inspect {log}')
    if session.exists():
        if not restart:
            raise RuntimeError('Managed session exists; use --prepare --restart while landed/disarmed, '
                               'or omit --prepare to observe existing localization.')
        action('stop')
    action('start', '--fixed-lio-extrinsics')
    survey = folder/'alignment.json'
    action('align', '--report-json', survey, '--parallel-observations')
    q = alignment_quaternion(json.loads(survey.read_text()))
    action('vision', *q)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=float, default=3., help='stationary capture duration in seconds')
    parser.add_argument('--stationary-only', action='store_true', help='record one stationary window')
    parser.add_argument('--output', type=Path, help='new report directory')
    parser.add_argument('--prepare', action='store_true',
                        help='prepare managed core, fresh alignment and vision only; uses existing manager parameter policy')
    parser.add_argument('--restart', action='store_true', help='with --prepare, replace a grounded/disarmed managed session')
    parser.add_argument('--scripts-dir', type=Path, default=Path.home()/'Downloads')
    args = parser.parse_args(argv)
    if not math.isfinite(args.window) or not 1 <= args.window <= 300:
        parser.error('--window must be between 1 and 300 seconds')
    if args.restart and not args.prepare:
        parser.error('--restart requires --prepare')
    folder = args.output or Path.home()/'Downloads'/'ground_localization_checks'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder = folder.expanduser()
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, result='RUNNING', scope='disarmed manual localization observations',
                  flight_readiness='NOT_ASSESSED', captures=[], diagnostic_parameter_writes=False,
                  managed_vision_preparation_requested=args.prepare,
                  notes=[
                      'Distances and angles are manual measurements, not motion-capture ground truth.',
                      'All aligned/vision/FC deltas use the baseline FC yaw; raw deltas remain in raw axes.',
                      'Returning to the same physical mark reveals closure error and manual placement error together.',
                      'Yaw test requires rotating about the FC centre; actual pivot translation is also measured.',
                      'QoS comparison concerns these observers, not delivery to existing controllers.',
                      'Receipt times are Python callback times and include executor scheduling.',
                      'Timing and covariance outliers are retained; no geometric or flight pass threshold is imposed.'])
    path = folder/'report.json'
    save(path, report)
    print('REPORT:', path, flush=True)
    observer = None
    ros = None
    lock = None
    try:
        if args.prepare:
            lock_dir = Path.home()/'.local/state/super-session'
            lock_dir.mkdir(parents=True, exist_ok=True)
            lock = (lock_dir/'automated_ground_test.lock').open('a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('Another automated run owns the aircraft stack; no preparation performed')
            prepare(folder, args.scripts_dir.expanduser(), args.restart)
        alignment = args.scripts_dir.expanduser()/'aligned_pose_preview.py'
        if alignment.is_file():
            content = alignment.read_bytes()
            (folder/'alignment_snapshot.py.txt').write_bytes(content)
            report['alignment_sha256'] = hashlib.sha256(content).hexdigest()
        import rclpy
        ros = rclpy
        ros.init()
        observer = Observer(folder)
        observer.wait_ready()
        observer.ask('DISARMED MANUAL CHECK. Keep motors stopped. Mark the FC centre position and nose direction. '
                     'Place the aircraft stationary at this mark; press Enter.')
        baseline = observer.capture('baseline', args.window)
        report['captures'].append(baseline)
        if any('mean_xyz_m' not in baseline['topics'][t] for t in TOPICS):
            raise RuntimeError('Baseline is missing valid poses; inspect the saved samples')
        heading = baseline['topics'][LOCAL]['mean_yaw_rad']
        report['baseline_fc_yaw_rad'] = heading
        report['baseline_vision_minus_fc_yaw_deg'] = math.degrees(wrap(
            baseline['topics'][VISION]['mean_yaw_rad']-heading))
        save(path, report)
        steps = [] if args.stationary_only else [
            ('forward', 0, 'Move FORWARD along the original nose direction without rotating '
             '(about 0.5 m). Hold still and enter the actual measured distance in METRES:'),
            ('left', 1, 'Move LEFT relative to the original nose, without rotating '
             '(about 0.5 m). Hold still and enter the actual measured distance in METRES:'),
            ('up', 2, 'Lift UP without horizontal movement or rotation '
             '(about 0.3 m). Hold still and enter the actual measured distance in METRES:'),
            ('yaw_left', None, 'Turn the nose LEFT about the FC centre, keeping its position fixed '
             '(about 90 degrees). Hold still and enter the actual turn in DEGREES:')]
        for name, axis, prompt in steps:
            magnitude = observer.ask(prompt, numeric=True)
            expected = [0., 0., 0.]
            if axis is not None:
                expected[axis] = magnitude
            current = observer.capture(name, args.window)
            current['manual_expected_forward_left_up_m'] = expected
            current['manual_expected_yaw_deg'] = magnitude if axis is None else 0.
            current['comparison'] = comparison(baseline['topics'], current['topics'], heading,
                                               expected, current['manual_expected_yaw_deg'])
            report['captures'].append(current)
            save(path, report)
            for t in (ALIGNED, VISION, LOCAL):
                r = current['comparison'][t]
                print(t, 'delta forward/left/up m:', r.get('delta_forward_left_up_m'),
                      'yaw change deg:', r.get('yaw_change_deg'), flush=True)
            observer.ask('Return the FC centre to the original mark AND restore the original nose direction. '
                         'Keep still, then press Enter.')
            home = observer.capture('return_after_'+name, args.window)
            home['comparison'] = comparison(baseline['topics'], home['topics'], heading, [0., 0., 0.])
            report['captures'].append(home)
            save(path, report)
        report['result'] = 'CAPTURE_COMPLETE'
        report['complete_pose_windows'] = all(
            all(c['topics'][t].get('invalid_pose_count', 0) == 0
                and c['topics'][t].get('count', 0) >= 2
                and c['topics'][t].get('invalid_stamp_count', 0) == 0
                and c['topics'][t].get('nonincreasing_stamp_count', 0) == 0
                and c['topics'][t].get('frames') == [FRAMES[t]] for t in TOPICS)
            for c in report['captures'])
        if not report['complete_pose_windows']:
            report['result'] = 'INCOMPLETE_DATA'
    except KeyboardInterrupt:
        report['result'] = 'CANCELLED'
        report['error'] = 'Operator interrupted the diagnostic'
    except Exception as exc:
        report['result'] = 'ERROR'
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if observer is not None:
            report['publishers_at_end'] = observer.graph
            report['whole_run_qos_comparison'] = {
                t: qos_comparison(observer.rows[(t, 'best_effort')], observer.rows[(t, 'reliable')])
                for t in TOPICS}
            report['whole_run_statistics'] = {
                t: {policy: statistics(observer.rows[(t, policy)]) for policy in ('best_effort', 'reliable')}
                for t in TOPICS}
            observer.close()
        if ros is not None and ros.ok():
            ros.shutdown()
        if lock is not None:
            lock.close()
        save(path, report)
        import zipfile
        with zipfile.ZipFile(folder/'diagnostic.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
            for entry in sorted(folder.iterdir()):
                if entry.is_file() and entry.name != 'diagnostic.zip':
                    archive.write(entry, arcname=entry.name)
    print('RESULT:', report['result'])
    if report.get('error'):
        print('ERROR:', report['error'])
    print('REPORT:', path)
    print('BUNDLE:', folder/'diagnostic.zip')
    print('FLIGHT READINESS: NOT ASSESSED. After handling, prepare a fresh session/alignment/venue before flight.')
    return 0 if report['result'] == 'CAPTURE_COMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
