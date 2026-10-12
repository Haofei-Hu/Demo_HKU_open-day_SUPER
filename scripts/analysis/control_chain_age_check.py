#!/usr/bin/env python3
"""Read-only sampled freshness/motion report; no goals, setpoints or FC services.

Advertised derivatives belong to the planner IMU-origin world reference, not
measured aircraft motion or a validated flight-capable FC-centre reference.
"""
import argparse
import math
import time


class Series:
    def __init__(self):
        self.n = self.invalid = self.reordered = self.bad_frames = 0
        self.age_sum = self.gap_max = 0.
        self.age_min, self.age_max = math.inf, -math.inf
        self.last_receive = self.last_stamp = None

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
              f'unexpected frames={self.bad_frames}')


class Motion:
    def __init__(self):
        self.n = self.invalid = 0
        self.speed = self.acceleration = self.step = self.step_dt = 0.
        self.previous = None

    def add(self, stamp, frame, flag, position, velocity, acceleration):
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
            old_stamp, old_position = self.previous
            if stamp <= old_stamp:
                self.invalid += 1
                return
            step = math.dist(position, old_position)
            if step > self.step:
                self.step, self.step_dt = step, (stamp-old_stamp)/1e9
        self.previous = stamp, tuple(position)

    def within(self, speed, acceleration):
        return (self.n >= 20 and not self.invalid
                and self.speed <= speed+1e-9 and self.acceleration <= acceleration+1e-9)


def monitor(seconds, check_motion=False, max_speed=.3, max_acceleration=.5):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
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
    motion = Motion()
    state = dict(msg=None, receipt=-math.inf, armed_seen=False)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    def stamp(msg):
        return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

    def record(msg, topic):
        ns = stamp(msg)
        series = samples[topic]
        series.add(time.monotonic(), ns, (node.get_clock().now().nanoseconds-ns)/1e9)
        if msg.header.frame_id != frames[topic] or (topic == '/Odometry' and msg.child_frame_id != 'body'):
            series.bad_frames += 1
        if topic == '/planning/pos_cmd':
            vectors = [tuple(getattr(getattr(msg, field), axis) for axis in ('x','y','z'))
                       for field in ('position','velocity','acceleration')]
            motion.add(ns, msg.header.frame_id, msg.trajectory_flag, *vectors)

    def state_received(msg):
        state.update(msg=msg, receipt=time.monotonic(),
                     armed_seen=state['armed_seen'] or msg.armed)

    subscriptions = [node.create_subscription(kind, topic,
                     lambda msg, name=topic: record(msg, name), qos) for topic, kind in topics]
    if check_motion:
        subscriptions.append(node.create_subscription(State, '/mavros/state', state_received, qos))
    try:
        print(f'Read-only control-chain sampling: {seconds:g}s. No goal or flight requests.', flush=True)
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., end-time.monotonic())))
        now = time.monotonic()
        for topic, _ in topics:
            print(topic)
            samples[topic].report(now)
        if not check_motion:
            return 0
        print(f'Planner advertised 3D speed max: {motion.speed:.6f} m/s '
              f'(requested <= {max_speed:g})')
        print(f'Planner advertised 3D acceleration max: {motion.acceleration:.6f} m/s^2 '
              f'(requested <= {max_acceleration:g}); valid={motion.n}, invalid={motion.invalid}')
        print(f'Largest planner position step: {motion.step:.6f} m '
              f'across stamp gap {motion.step_dt:.6f}s; inspect stationary replans separately.')
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
    parser.add_argument('--check-motion', action='store_true')
    parser.add_argument('--max-speed', type=float, default=.3)
    parser.add_argument('--max-acceleration', type=float, default=.5)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 30:
        parser.error('--seconds must be finite and in [1, 30]')
    if not all(math.isfinite(v) and v > 0 for v in (args.max_speed,args.max_acceleration)):
        parser.error('Motion limits must be finite and positive')
    return monitor(args.seconds,args.check_motion,args.max_speed,args.max_acceleration)


if __name__ == '__main__':
    raise SystemExit(main())
