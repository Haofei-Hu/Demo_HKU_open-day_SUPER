#!/usr/bin/env python3
"""One-command, one-goal disarmed audit with fresh automatic alignment.

Preparation and audit are separate stages. There is deliberately no flight,
mode, arming or native-setpoint entry point in this runner.
"""
import argparse
from datetime import datetime
import fcntl
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

import super_session as manager


def write_report(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def alignment_quaternion(report, max_spread=2.):
    """Validate a freshly produced survey before applying any alignment."""
    if report.get('schema_version') != 1 or not report.get('estimate_available'):
        raise RuntimeError('No usable alignment estimate in this run')
    q = report.get('quaternion_xyzw', [])
    if (len(q) != 4 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q)
            or abs(math.sqrt(sum(v*v for v in q))-1) > .02):
        raise RuntimeError('Invalid measured quaternion')
    checks = {
        'matched samples >= 20': report.get('matched', 0) >= 20,
        'pair gap <= 0.05 s': 0 <= report.get('pairing_gap_max_s', math.inf) <= .05,
        'body timestamps within -0.05..0.25 s':
            -.05 <= report.get('body_age_min_s', -math.inf)
            <= report.get('body_age_max_s', math.inf) <= .25,
        f'rotation spread <= {max_spread:g} deg':
            0 <= report.get('rotation_spread_max_deg', math.inf) <= max_spread,
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise RuntimeError('Alignment quality failed: '+', '.join(failed))
    return q


class AuditObserver:
    """Observe the existing checker and trigger only its disarmed audit."""
    def __init__(self):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from control_chain_age_check import Series
        self.Series = Series
        self.gate = manager.GroundGate()
        self.status = None
        self.status_at = -math.inf
        self.active = False
        self.last_graph = -math.inf
        self.end_since = None
        self.end_hold = 0.
        self.states = []
        self.freshness_fault = None
        self.audit_request_attempted = False
        self.audit_acknowledged = False
        self.source_identities = {}
        self.kinds = {
            '/Odometry': (Odometry, 'camera_init'),
            '/super_debug/aligned_pose_cov': (PoseWithCovarianceStamped, 'px4_enu_preview'),
            '/mavros/vision_pose/pose_cov': (PoseWithCovarianceStamped, 'px4_enu_preview'),
            '/mavros/local_position/pose': (PoseStamped, 'map'),
        }
        self.series = {name: Series() for name in self.kinds}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, (kind, _) in self.kinds.items():
            self.gate.node.create_subscription(kind, topic,
                lambda msg, name=topic: self.receive(name, msg), qos)
        self.gate.node.create_subscription(String, '/super_debug/super_flight_status', self.receive_status, qos)

    def receive(self, topic, msg):
        if not self.active:
            return
        ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.gate.node.get_clock().now().nanoseconds-ns)/1e9
        series = self.series[topic]
        series.add(time.monotonic(), ns, age)
        if msg.header.frame_id != self.kinds[topic][1] or (
                topic == '/Odometry' and msg.child_frame_id != 'body'):
            series.bad_frames += 1
        if (ns <= 0 or not -.05 <= age <= .25 or series.reordered
                or series.bad_frames or series.gap_max > .25):
            self.freshness_fault = f'{topic}: invalid/stale sample or gap; header age={age:.6f}s'

    def receive_status(self, msg):
        self.status, self.status_at = msg.data, time.monotonic()
        if self.active:
            phase = msg.data.split(':', 1)[0]
            if not self.states or self.states[-1]['phase'] != phase:
                self.states.append(dict(phase=phase, monotonic_s=self.status_at, status=msg.data))
                print(msg.data, flush=True)
            if phase == 'AUDIT_END':
                if self.end_since is None:
                    self.end_since = self.status_at
                self.end_hold = max(self.end_hold, self.status_at-self.end_since)
            else:
                self.end_since = None
                self.end_hold = 0.

    def graph_ready(self):
        node = self.gate.node
        topics = (*self.kinds, '/super_debug/super_flight_status')
        counts = {name: node.count_publishers(name) for name in topics}
        if any(count > 1 for count in counts.values()):
            raise RuntimeError('Multiple localization/status publishers: '+str(counts))
        if any(node.count_publishers(name) for name in manager.NATIVE_CONTROL_TOPICS):
            raise RuntimeError('Native control publisher present; ground test stopped')
        for name in topics:
            identities = {bytes(item.endpoint_gid) for item in node.get_publishers_info_by_topic(name)}
            seen = self.source_identities.setdefault(name, set())
            seen.update(identities)
            if len(seen) > 1:
                raise RuntimeError('Localization/status source identity changed: '+name)
        for name in self.series:
            self.series[name].sources.update(bytes(item.endpoint_gid)
                for item in node.get_publishers_info_by_topic(name))
            if len(self.series[name].sources) > 1:
                raise RuntimeError('Localization source identity changed: '+name)
        self.last_graph = time.monotonic()
        return all(count == 1 for count in counts.values())

    def guard(self):
        self.gate.check()
        now = time.monotonic()
        if self.status and self.status.startswith('STOPPED:'):
            raise RuntimeError(self.status)
        if self.active:
            if self.freshness_fault:
                raise RuntimeError(self.freshness_fault)
            if not self.gate.localization_ready():
                raise RuntimeError('Ground localization became stale or invalid during audit')
            if now-self.status_at > 2.:
                raise RuntimeError('Controller status became stale during audit')
            if self.status and ('DEBUG ONLY' not in self.status or 'fault=None' not in self.status):
                raise RuntimeError('Unexpected controller output: '+self.status)
            for name, series in self.series.items():
                if series.last_receive is not None and now-series.last_receive > .25:
                    raise RuntimeError('Input receive gap during audit: '+name)
        if now-self.last_graph >= .5 and not self.graph_ready() and self.active:
            raise RuntimeError('A required publisher disappeared during audit')

    def wait_ready(self, timeout=60.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.gate.ros.spin_once(self.gate.node, timeout_sec=.02)
            self.guard()
            if (self.status and self.status.startswith('READY:') and 'DEBUG ONLY' in self.status
                    and 'fault=None' in self.status and time.monotonic()-self.status_at < 2.
                    and self.gate.localization_ready() and self.graph_ready()):
                return
        raise RuntimeError('Timed out waiting for current check-only READY; last status='+str(self.status))

    def audit(self, seconds=30.):
        from std_srvs.srv import Trigger
        self.series = {name: self.Series() for name in self.kinds}
        self.graph_ready()
        self.active = True
        began = time.monotonic()
        # This request is deliberately sent once. A lost acknowledgement is a
        # failed run, never a reason to repeat a possibly accepted goal.
        self.audit_request_attempted = True
        response = self.gate.call(Trigger, '/super_flight/audit', Trigger.Request())
        self.audit_acknowledged = response.success
        if not response.success:
            raise RuntimeError('Audit rejected: '+response.message)
        loop_at = time.monotonic()
        loop_gap = 0.
        while time.monotonic()-began < seconds:
            now = time.monotonic()
            loop_gap = max(loop_gap, now-loop_at)
            loop_at = now
            self.gate.ros.spin_once(self.gate.node, timeout_sec=.02)
            self.guard()
        now = time.monotonic()
        self.graph_ready()
        if (not self.status or not self.status.startswith('AUDIT_END:') or self.end_hold < 6.
                or not re.search(r'accepted=[1-9]\d*;', self.status)):
            raise RuntimeError('No verified six-second AUDIT_END hold; last status='+str(self.status))
        if not all(series.fresh(now) for series in self.series.values()):
            raise RuntimeError('Concurrent vision-chain sampling failed freshness/source checks')
        topics = {}
        for name, s in self.series.items():
            topics[name] = dict(n=s.n, age_min_avg_max_s=[s.age_min,s.age_sum/s.n,s.age_max],
                receive_gap_max_s=s.gap_max, last_receive_age_s=now-s.last_receive,
                invalid_stamps=s.invalid, nonincreasing_stamps=s.reordered,
                unexpected_frames=s.bad_frames, publisher_identities_discovered=len(s.sources))
        return dict(trigger_success=True, duration_s=now-began, end_hold_s=self.end_hold,
                    final_status=self.status, phases=self.states, topics=topics,
                    sampled_vision_chain_fresh=True, observer_loop_gap_max_s=loop_gap)

    def snapshot(self):
        """Retain partial evidence even if the audit or its acknowledgement fails."""
        def finite(value):
            return value if math.isfinite(value) else None
        return dict(audit_request_attempted=self.audit_request_attempted,
            audit_acknowledged=self.audit_acknowledged, final_status=self.status,
            end_hold_s=self.end_hold, phases=self.states,
            freshness_fault=self.freshness_fault,
            topics={name:dict(n=s.n, age_min_s=finite(s.age_min),age_max_s=finite(s.age_max),
                receive_gap_max_s=s.gap_max,invalid_stamps=s.invalid,
                nonincreasing_stamps=s.reordered,unexpected_frames=s.bad_frames,
                publisher_identities_discovered=len(s.sources)) for name,s in self.series.items()})

    def close(self):
        self.gate.close()


class GroundTest:
    def __init__(self, args):
        self.args = args
        self.session = manager.Session(manager.arguments([
            'start', '--session', args.session, '--scripts-dir', str(args.scripts_dir)]))
        folder = args.scripts_dir/'super_ground_tests'
        folder.mkdir(parents=True, exist_ok=True)
        self.folder = folder/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.folder.mkdir()
        self.path = self.folder/'report.json'
        self.report = dict(schema_version=1, scope='Disarmed one-goal ground audit',
            passed=False, stage='preflight', stages=[], native_output=False,
            mode_or_arming_requests=False, flight_verified=False,
            alignment_max_spread_deg=args.max_alignment_spread_deg)
        self.observer = None

    def action(self, action, *extra):
        self.report['stage'] = action
        write_report(self.path, self.report)
        command = [sys.executable, str(self.args.scripts_dir/'super_session.py'), action,
                   '--session', self.args.session, '--scripts-dir', str(self.args.scripts_dir), *map(str, extra)]
        logfile = self.folder/(str(len(self.report['stages']))+'_'+action+'.log')
        print(f'[{action}] {logfile}', flush=True)
        stage = dict(action=action, log=str(logfile))
        self.report['stages'].append(stage)
        try:
            with logfile.open('x') as handle:
                result = subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, timeout=120)
            print(logfile.read_text(), end='', flush=True)
            stage['returncode'] = result.returncode
            if result.returncode:
                raise RuntimeError(f'{action} failed; inspect {logfile}')
        finally:
            write_report(self.path, self.report)

    def prepare(self):
        """Reusable preparation; any future flight entry must add fresh flight gates."""
        required = (*manager.SCRIPTS, 'super_session.py', 'control_chain_age_check.py',
                    'super_flight_control.py', 'ground_setpoint_transport.py')
        for name in required:
            if not (self.args.scripts_dir/name).is_file():
                raise RuntimeError('Missing prerequisite: '+name)
        if self.session.exists():
            if not self.args.restart:
                raise RuntimeError('Managed session already exists; use --restart only while grounded/disarmed')
            self.action('stop')
        extra = ['--fixed-lio-extrinsics'] if self.args.fixed_lio_extrinsics else []
        self.action('start', *extra)
        self.session.load()
        self.report['session'] = self.session.config
        survey = self.folder/'alignment.json'
        self.action('align', '--report-json', survey)
        measured = json.loads(survey.read_text())
        self.report['alignment'] = measured
        q = alignment_quaternion(measured, self.args.max_alignment_spread_deg)
        print('Applying this run\'s measured quaternion xyzw:', q, flush=True)
        self.action('vision', *q)
        self.action('planner', '--planner-profile', 'low-speed-3d')
        self.session.load()
        self.report['session'] = self.session.config

    def run(self):
        print('DISARMED TEST ONLY. Keep the aircraft stationary on its takeoff floor.', flush=True)
        print('Report:', self.path, flush=True)
        try:
            self.prepare()
            self.action('handover-check')
            self.report['stage'] = 'audit'
            write_report(self.path, self.report)
            self.observer = AuditObserver()
            self.observer.wait_ready()
            self.report['audit'] = self.observer.audit(self.args.sample_seconds)
            self.observer.close()
            self.observer = None
            snapshot = self.folder/'flight_check.json'
            self.action('flight-check', '--report-json', snapshot)
            self.report['readiness'] = json.loads(snapshot.read_text())
            if not self.report['readiness'].get('remote_preparation_checks_passed'):
                raise RuntimeError('Remote preparation snapshot failed; inspect flight_check.json')
            self.report.update(passed=True, stage='complete')
            print('PASS: fresh alignment, session map, six-second audit endpoint hold and remote checks.', flush=True)
            print('Stack retained. Flight and reboot repeatability are not established by this run.', flush=True)
            return 0
        except (Exception, KeyboardInterrupt) as exc:
            self.report['error'] = f'{type(exc).__name__}: {exc}'
            if self.observer is not None:
                self.report['last_controller_status'] = self.observer.status
                self.report['partial_audit'] = self.observer.snapshot()
            print('STOPPED:', self.report['error'], flush=True)
            print('Stack and logs retained; no automatic retry, rebase or flight request.', flush=True)
            return 130 if isinstance(exc, KeyboardInterrupt) else 1
        finally:
            if self.observer is not None:
                try:
                    self.observer.close()
                except Exception as exc:
                    self.report['cleanup_error'] = f'{type(exc).__name__}: {exc}'
            write_report(self.path, self.report)
            print('Report:', self.path, flush=True)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scripts-dir', type=Path, default=Path.home()/'Downloads')
    parser.add_argument('--session', default='super-flight')
    parser.add_argument('--restart', action='store_true', help='explicitly replace a managed grounded/disarmed session')
    parser.add_argument('--fixed-lio-extrinsics', action='store_true', help='use the existing fixed-extrinsic startup override')
    parser.add_argument('--max-alignment-spread-deg', type=float, default=2., help='automatic survey acceptance limit, default 2 degrees')
    parser.add_argument('--sample-seconds', type=float, default=30., help='concurrent audit observation, 20 to 300 seconds')
    parser.add_argument('--ros-env', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    args.scripts_dir = args.scripts_dir.expanduser().resolve()
    if (not math.isfinite(args.sample_seconds) or not 20 <= args.sample_seconds <= 300
            or not math.isfinite(args.max_alignment_spread_deg) or not 0 < args.max_alignment_spread_deg <= 5):
        parser.error('Use 20..300 seconds and a finite alignment spread in (0, 5] degrees')
    manager.arguments(['start', '--session', args.session])
    return args


def main():
    args = arguments()
    if not args.ros_env:
        command = manager.source_prefix()+'exec '+manager.shell_command([
            sys.executable, Path(__file__).resolve(), *sys.argv[1:], '--ros-env'])
        return subprocess.run(['bash', '--noprofile', '--norc', '-c', command]).returncode
    folder = Path.home()/'.local/state/super-session'
    folder.mkdir(parents=True, exist_ok=True)
    with (folder/'automated_ground_test.lock').open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('Another automated test owns the aircraft stack; no actions taken.', file=sys.stderr)
            return 1
        return GroundTest(args).run()


if __name__ == '__main__':
    raise SystemExit(main())
