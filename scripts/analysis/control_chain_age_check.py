#!/usr/bin/env python3
"""Read-only sampled freshness/motion report; no goals, setpoints or FC services.

Advertised derivatives belong to the planner IMU-origin world reference, not
measured aircraft motion or a validated flight-capable FC-centre reference.
"""
import argparse
import math
from pathlib import Path
import time


class Series:
    def __init__(self):
        self.n = self.invalid = self.reordered = self.bad_frames = 0
        self.age_sum = self.gap_max = 0.
        self.age_min, self.age_max = math.inf, -math.inf
        self.last_receive = self.last_stamp = None
        self.sources = set()

    def add(self, received, stamp, age):
        self.n += 1
        if self.last_receive is not None:
            self.gap_max = max(self.gap_max, received-self.last_receive)
        self.last_receive = received
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reordered += 1
        self.last_stamp = stamp
        if stamp <= 0 or not math.isfinite(age):
            self.invalid += 1
            return
        self.age_sum += age
        self.age_min, self.age_max = min(self.age_min, age), max(self.age_max, age)

    def fresh(self, now, minimum=20, limit=.25):
        return (self.n >= minimum and not self.invalid and not self.reordered and not self.bad_frames
                and len(self.sources) == 1
                and self.last_receive is not None and now-self.last_receive <= limit
                and -.05 <= self.age_min and self.age_max <= limit and self.gap_max <= limit)

    def report(self, now):
        if not self.n:
            print('  No messages received')
            return
        count = self.n-self.invalid
        if count:
            print(f'  n={self.n}, age min/avg/max={self.age_min:.3f}/'
                  f'{self.age_sum/count:.3f}/{self.age_max:.3f}s')
        print(f'  receive gap max={self.gap_max:.3f}s, '
              f'last receive age={now-self.last_receive:.3f}s; '
              f'invalid stamps={self.invalid}, nonincreasing stamps={self.reordered}, '
              f'unexpected frames={self.bad_frames}, publisher identities discovered={len(self.sources)}')


class Motion:
    def __init__(self):
        self.n = self.invalid = 0
        self.speed = self.acceleration = self.step = self.step_dt = 0.
        self.previous = None
        self.step_detail = None

    def add(self, stamp, frame, flag, position, velocity, acceleration, source=None, robot=None):
        values = (*position, *velocity, *acceleration)
        if (stamp <= 0 or frame != 'world' or flag not in (1, 2)
                or any(len(v) != 3 for v in (position, velocity, acceleration))
                or not all(math.isfinite(v) for v in values)):
            self.invalid += 1
            return
        self.n += 1
        self.speed = max(self.speed, math.hypot(*velocity))
        self.acceleration = max(self.acceleration, math.hypot(*acceleration))
        if self.previous is not None:
            old_stamp, old_position, old_velocity, old_flag, old_source = self.previous
            if stamp <= old_stamp:
                self.invalid += 1
                return
            step = math.dist(position, old_position)
            if step > self.step:
                self.step, self.step_dt = step, (stamp-old_stamp)/1e9
                self.step_detail = dict(before=(old_stamp,old_position,old_velocity,old_flag),
                                        after=(stamp,tuple(position),tuple(velocity),flag),
                                        same_source=(old_source == source if old_source is not None and source is not None else None),
                                        robot=robot)
        self.previous = stamp, tuple(position), tuple(velocity), flag, source

    def within(self, speed, acceleration):
        return (self.n >= 20 and not self.invalid
                and self.speed <= speed+1e-9 and self.acceleration <= acceleration+1e-9)


def freshness_reason(series, received_now, ros_now_ns):
    """Same receive/header limits as the controller; report each separately."""
    if series.last_receive is None:
        return 'no_messages', math.inf, math.inf
    receipt_age=received_now-series.last_receive
    header_age=(ros_now_ns-series.last_stamp)/1e9
    if series.last_stamp <= 0:
        return 'invalid_stamp',receipt_age,header_age
    if receipt_age > .25:
        return 'receive_gap',receipt_age,header_age
    if not -.05 <= header_age <= .25:
        return 'header_age',receipt_age,header_age
    return None,receipt_age,header_age


class StableVisionWindow:
    """Wait for a complete fresh observation interval; preserve lifetime evidence."""
    def __init__(self,seconds):
        self.seconds=seconds;self.started=None;self.samples={}
        self.restarts=0;self.fault=None;self.reason='waiting for current fresh sources'

    def reset(self,reason):
        if self.started is not None:
            self.restarts+=1
            print('VISION WAIT: observation window interrupted; '+reason+'; waiting for a new complete window',flush=True)
        self.started=None;self.samples={};self.reason=reason

    def record(self,topic,received,stamp,age):
        if self.started is None:return
        series=self.samples[topic]
        if not -.05<=age<=.25:
            self.reset(f'{topic} header_age={age:.6f}s')
        elif series.last_receive is not None and received-series.last_receive>.25:
            self.reset(f'{topic} receive_gap={received-series.last_receive:.6f}s')
        else:series.add(received,stamp,age)

    def update(self,history,now,ros_ns):
        for topic,series in history.items():
            if series.invalid or series.reordered or series.bad_frames or len(series.sources)>1:
                self.fault=f'{topic}: invalid/reordered stamps, unexpected frames or changed/duplicate sources'
                return False
        reasons=[]
        for topic,series in history.items():
            reason,_,_=freshness_reason(series,now,ros_ns)
            if reason or len(series.sources)!=1:reasons.append(topic+': '+(reason or 'source discovery'))
        if reasons:
            self.reset('; '.join(reasons))
            return False
        if self.started is None:
            self.started=now;self.samples={topic:Series() for topic in history}
            for topic,series in history.items():
                qualified=self.samples[topic]
                qualified.sources.update(series.sources)
                qualified.last_receive,qualified.last_stamp=series.last_receive,series.last_stamp
            self.reason='collecting continuous fresh samples'
            print(f'VISION WINDOW: all current sources fresh; collecting {self.seconds:g}s with unchanged age/gap limits',flush=True)
        return now-self.started>=self.seconds and all(s.fresh(now) for s in self.samples.values())


def monitor(seconds, check_motion=False, max_speed=.3, max_acceleration=.5, vision_chain=False,wait_stable=False,ready_file=None):
    if wait_stable and (not vision_chain or check_motion):
        raise ValueError('Stable observation requires --vision-chain without motion checking')
    if ready_file is not None and not wait_stable:
        raise ValueError('Preparation completion signal requires stable vision observation')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from mavros_msgs.msg import State
    from mars_quadrotor_msgs.msg import PositionCommand

    rclpy.init()
    node = rclpy.create_node('control_chain_age_check')
    topics = [('/Odometry', Odometry), ('/planning/pos_cmd', PositionCommand),
              ('/super_debug/control_target', PoseStamped),
              ('/mavros/local_position/pose', PoseStamped)]
    samples = {topic: Series() for topic, _ in topics}
    frames = {'/Odometry':'camera_init', '/planning/pos_cmd':'world',
              '/super_debug/control_target':'px4_enu_preview',
              '/mavros/local_position/pose':'map'}
    if vision_chain:
        topics=[('/Odometry',Odometry),
                ('/super_debug/aligned_pose_cov',PoseWithCovarianceStamped),
                ('/mavros/vision_pose/pose_cov',PoseWithCovarianceStamped),
                ('/mavros/local_position/pose',PoseStamped)]
        frames.update({'/super_debug/aligned_pose_cov':'px4_enu_preview',
                       '/mavros/vision_pose/pose_cov':'px4_enu_preview'})
        samples={topic:Series() for topic,_ in topics}
    motion = Motion()
    state = dict(msg=None, receipt=-math.inf, armed_seen=False)
    observation = dict(raw=None)
    current_sources = {topic:None for topic, _ in topics}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    stable=StableVisionWindow(seconds) if wait_stable else None

    def stamp(msg):
        return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

    def record(msg, topic):
        ns = stamp(msg)
        series = samples[topic]
        received=time.monotonic();age=(node.get_clock().now().nanoseconds-ns)/1e9
        series.add(received,ns,age)
        source = current_sources[topic]
        if msg.header.frame_id != frames[topic] or (topic == '/Odometry' and msg.child_frame_id != 'body'):
            series.bad_frames += 1
        if stable is not None:stable.record(topic,received,ns,age)
        if topic == '/Odometry':
            p = msg.pose.pose.position
            observation['raw'] = (ns,(p.x,p.y,p.z))
        elif topic == '/planning/pos_cmd':
            vectors = [tuple(getattr(getattr(msg, field), axis) for axis in ('x','y','z'))
                       for field in ('position','velocity','acceleration')]
            motion.add(ns, msg.header.frame_id, msg.trajectory_flag, *vectors,
                       source=source, robot=observation['raw'])

    def state_received(msg):
        state.update(msg=msg, receipt=time.monotonic(),
                     armed_seen=state['armed_seen'] or msg.armed)

    def callback(topic):
        def received(msg):
            record(msg, topic)
        return received

    subscriptions = [node.create_subscription(kind, topic, callback(topic), qos)
                     for topic, kind in topics]
    if check_motion:
        subscriptions.append(node.create_subscription(State, '/mavros/state', state_received, qos))
    try:
        print(f'Read-only control-chain sampling: {seconds:g}s. No goal or flight requests.', flush=True)
        if stable is not None:print('VISION PREPARATION: wait for a complete fresh window; no total waiting deadline; lifetime faults remain reported',flush=True)
        end = time.monotonic()+seconds
        started=time.monotonic()
        live_states={topic:None for topic,_ in topics}
        loop_at=started
        loop_gap_max=0.
        graph_at = -math.inf
        progress_at=started
        while stable is not None or time.monotonic() < end:
            now=time.monotonic()
            loop_gap_max=max(loop_gap_max,now-loop_at)
            loop_at=now
            if time.monotonic()-graph_at >= .2:
                for topic, _ in topics:
                    identities = {bytes(endpoint.endpoint_gid)
                                  for endpoint in node.get_publishers_info_by_topic(topic)}
                    samples[topic].sources.update(identities)
                    current_sources[topic] = next(iter(identities)) if len(identities) == 1 else None
                graph_at = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.02 if stable is not None else min(.02, max(0., end-time.monotonic())))
            if vision_chain:
                now=time.monotonic();ros_ns=node.get_clock().now().nanoseconds
                for topic,_ in topics:
                    reason,receipt_age,header_age=freshness_reason(samples[topic],now,ros_ns)
                    if reason!=live_states[topic]:
                        print(f'VISION EVENT ros_ns={ros_ns} topic={topic} '
                              f'state={reason or "recovered"} receive_age={receipt_age:.6f}s '
                              f'header_age={header_age:.6f}s last_stamp_ns={samples[topic].last_stamp}',flush=True)
                        live_states[topic]=reason
            if stable is not None:
                qualified=stable.update(samples,time.monotonic(),node.get_clock().now().nanoseconds)
                preparation_ready=ready_file is None or ready_file.is_file()
                if (qualified and preparation_ready) or stable.fault:
                    break
                if time.monotonic()-progress_at>=5.:
                    elapsed=0. if stable.started is None else time.monotonic()-stable.started
                    reason='awaiting parallel map/planner preparation' if qualified and not preparation_ready else stable.reason
                    print(f'VISION WAIT: qualified window={elapsed:.1f}/{seconds:g}s; resets={stable.restarts}; {reason}',flush=True)
                    progress_at=time.monotonic()
        now = time.monotonic()
        if stable is not None:
            print('Lifetime acquisition statistics, including unsettled/rejected intervals:')
            for topic,_ in topics:
                print(topic);samples[topic].report(now)
            print(f'Continuous vision window: required={seconds:g}s; completed={stable.fault is None}; resets={stable.restarts}; fault={stable.fault}',flush=True)
            if stable.fault is None:samples=stable.samples
        for topic, _ in topics:
            print(topic)
            samples[topic].report(now)
        if not check_motion:
            if vision_chain:
                fresh=all(s.fresh(now) for s in samples.values()) and (stable is None or stable.fault is None)
                print(f'Sampled vision chain fresh: {fresh}; '
                      f'observer loop gap max={loop_gap_max:.6f}s')
                print('Independent subscriber observation; this cannot prove controller reception or physical accuracy.')
                return 0 if fresh else 2
            return 0
        print(f'Planner advertised 3D speed max: {motion.speed:.6f} m/s '
              f'(requested <= {max_speed:g})')
        print(f'Planner advertised 3D acceleration max: {motion.acceleration:.6f} m/s^2 '
              f'(requested <= {max_acceleration:g}); valid={motion.n}, invalid={motion.invalid}')
        print(f'Largest planner position step: {motion.step:.6f} m '
              f'across stamp gap {motion.step_dt:.6f}s; inspect stationary replans separately.')
        if motion.step_detail:
            detail = motion.step_detail
            for label in ('before','after'):
                ns,p,v,flag = detail[label]
                print(f'  {label}: stamp_ns={ns}, xyz={[round(x,6) for x in p]}, '
                      f'speed={math.hypot(*v):.6f} m/s, flag={flag}')
            print(f'  same cached single publisher identity: {detail["same_source"]}')
            if detail['robot'] is not None:
                robot_stamp,robot_position = detail['robot']
                age = (detail['after'][0]-robot_stamp)/1e9
                print(f'  latest raw odometry at step: xyz={[round(x,6) for x in robot_position]}, '
                      f'stamp gap={age:.6f}s')
        state_msg = state['msg']
        state_age = ((node.get_clock().now().nanoseconds-stamp(state_msg))/1e9
                     if state_msg is not None else math.inf)
        state_ok = (state_msg is not None and stamp(state_msg) > 0 and -.05 <= state_age <= 2
                    and now-state['receipt'] <= 2 and state_msg.connected
                    and not state['armed_seen'] and not state_msg.armed)
        fresh = all(series.fresh(now) for series in samples.values())
        limits = motion.within(max_speed, max_acceleration)
        print(f'Fresh chain: {fresh}; sampled planner motion limits: {limits}; '
              f'fresh connected/disarmed state, no arming observed: {state_ok}')
        print('Position steps are diagnostic only. This report does not validate tracking, '
              'endpoint hold, bounds, or flight execution.')
        return 0 if fresh and limits and state_ok else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--ready-file',type=Path,help='with --wait-stable, keep checking until this preparation-completion file exists')
    parser.add_argument('--check-motion', action='store_true')
    parser.add_argument('--vision-chain',action='store_true',
                        help='sample raw/aligned/native vision/FC pose; print timed receive/header freshness events')
    parser.add_argument('--wait-stable',action='store_true',
                        help='vision preparation: wait for a complete fresh sampling window without a total deadline')
    parser.add_argument('--max-speed', type=float, default=.3)
    parser.add_argument('--max-acceleration', type=float, default=.5)
    args = parser.parse_args()
    if args.vision_chain and args.check_motion:
        parser.error('--vision-chain and --check-motion select different topic sets')
    if args.wait_stable and not args.vision_chain:parser.error('--wait-stable requires --vision-chain')
    if args.ready_file is not None and not args.wait_stable:parser.error('--ready-file requires --wait-stable')
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 30:
        parser.error('--seconds must be finite and in [1, 30]')
    if not all(math.isfinite(v) and v > 0 for v in (args.max_speed,args.max_acceleration)):
        parser.error('Motion limits must be finite and positive')
    return monitor(args.seconds,args.check_motion,args.max_speed,args.max_acceleration,args.vision_chain,args.wait_stable,args.ready_file)


if __name__ == '__main__':
    raise SystemExit(main())
