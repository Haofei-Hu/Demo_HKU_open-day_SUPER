#!/usr/bin/env python3
"""One-goal ground reference preview with endpoint and discontinuity checks.

Only /super_debug output. No native setpoints, goals, mode/arming services or
flight support. The envelope is relative to a stationary FC-centre pose in the
aligned preview frame; its yaw defines forward, not planner-world X.
"""
from collections import deque
import hashlib
import math
from pathlib import Path
import time

import numpy as np
from control_bridge_preview import arrays, node_type as coordinate_node_type


def norm(vector):
    return float(np.linalg.norm(vector))


class TakeoffFrame:
    def __init__(self, position, quaternion, max_forward=5.):
        if not math.isfinite(max_forward) or not 0 < max_forward <= 5.1:
            raise ValueError('Invalid forward reference bound')
        self.max_forward = float(max_forward)
        self.origin = np.array(position, dtype=float, copy=True)
        q = np.asarray(quaternion, dtype=float)
        if self.origin.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite([*self.origin, *q])) or abs(norm(q)-1) > .02:
            raise ValueError('Invalid stationary takeoff pose')
        x, y, z, w = q/norm(q)
        self.yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.axes = np.array([[c,s,0],[-s,c,0],[0,0,1]])

    def coordinates(self, position):
        p = self.axes @ (np.asarray(position)-self.origin)
        p[2] += .085  # Measured FC-centre height on the takeoff floor.
        return p

    def check(self, position):
        p = self.coordinates(position)
        if not np.all(np.isfinite(p)) or not (-.10 <= p[0] <= self.max_forward and abs(p[1]) <= 2 and -.10 <= p[2] <= 2):
            raise ValueError('Outside forward/left/floor-height envelope: '+str(p.round(4).tolist()))
        return p


class Plan:
    """Pinned ROS2 PolynomialTrajectory: descending powers, local piece seconds."""
    def __init__(self, msg):
        self.start = float(msg.start_wt_pos)
        durations = np.asarray(msg.time_pos, dtype=float)
        n = int(msg.piece_num_pos)
        values = [msg.coef_pos_x, msg.coef_pos_y, msg.coef_pos_z]
        if (msg.header.frame_id != 'world' or not msg.type & 2 or msg.type & 16
                or msg.order_pos != 7 or not 1 <= n <= 256 or len(durations) != n
                or any(len(v) != n*8 for v in values) or not math.isfinite(self.start) or self.start <= 0
                or not np.all(np.isfinite(durations)) or np.any(durations <= 0) or durations.sum() > 120):
            raise ValueError('Invalid/unsupported committed position trajectory')
        self.durations = durations.copy()
        self.coefficients = np.asarray(values, dtype=float).reshape(3,n,8).copy()
        if not np.all(np.isfinite(self.coefficients)):
            raise ValueError('Nonfinite polynomial coefficients')
        self.duration = float(durations.sum())
        self.end = self.start+self.duration
        self.endpoint, self.end_velocity, self.end_acceleration = self.evaluate(self.duration)
        if not np.all(np.isfinite([*self.endpoint,*self.end_velocity,*self.end_acceleration])):
            raise ValueError('Nonfinite polynomial endpoint')

    def evaluate(self, elapsed):
        t = min(max(float(elapsed),0.), self.duration)
        index = 0
        while index < len(self.durations)-1 and t > self.durations[index]:
            t -= self.durations[index]
            index += 1
        result = []
        for order in (0,1,2):
            result.append(np.array([np.polyval(np.polyder(axis[index],order),t)
                                    for axis in self.coefficients]))
        return tuple(result)

    def matches(self, stamp, position, velocity, acceleration):
        elapsed = stamp/1e9-self.start
        if not -.03 <= elapsed <= self.duration+.03:
            return False
        predicted = self.evaluate(elapsed)
        return all(norm(a-b) <= limit for a,b,limit in
                   zip(predicted,(position,velocity,acceleration),(.02,.03,.05)))

    def displacement_bound(self,begin,end):
        """Conservative displacement of this previously verified polynomial.

        Taylor coefficients bound each piece's displacement by the triangle
        inequality. The new command's advertised speed cannot enlarge this
        bound and conceal a jump when switching committed trajectories.
        """
        if not all(math.isfinite(t) for t in (begin,end)) or end<begin:
            raise ValueError('Invalid polynomial continuity interval')
        begin=max(0.,float(begin));end=max(begin,float(end));bound=0.;offset=0.
        for index,duration in enumerate(self.durations):
            left=max(begin,offset);right=min(end,offset+duration)
            if right>left:
                elapsed=left-offset;dt=right-left
                for order in range(1,8):
                    derivative=np.array([np.polyval(np.polyder(axis[index],order),elapsed)
                                         for axis in self.coefficients])
                    bound+=norm(derivative)*dt**order/math.factorial(order)
            offset+=duration
        if end>self.duration:
            left=max(begin,self.duration)-self.duration;right=end-self.duration
            bound+=norm(self.end_velocity)*(right-left)
            bound+=norm(self.end_acceleration)*(right*right-left*left)/2
        if not math.isfinite(bound):raise ValueError('Nonfinite polynomial displacement bound')
        return float(bound)


class Reference:
    """A terminal state is permanent for this process; no automatic new goal."""
    def __init__(self, frame, max_speed=.3, max_acceleration=.5, motion_policy='strict'):
        if motion_policy not in ('strict','observe'):raise ValueError('Unknown motion policy')
        if (not all(math.isfinite(v) for v in (max_speed,max_acceleration))
                or not 0 < max_speed <= 2. or not 0 < max_acceleration <= 1.):
            raise ValueError('Invalid reference motion limits')
        self.max_speed,self.max_acceleration = float(max_speed),float(max_acceleration)
        self.motion_policy=motion_policy
        self.last_motion_violation=None
        self.motion_summary=dict(checked_samples=0,speed_exceedances=0,acceleration_exceedances=0,
                                 peak_speed_mps=0.,peak_acceleration_mps2=0.)
        self.frame = frame
        self.plans = deque(maxlen=24)
        self.phase = 'WAIT_TRAJECTORY'
        self.reason = ''
        self.last = None
        self.last_evidence = None
        self.last_plan = None
        self.endpoint = None
        self.initial_offset = None
        self.accepted = self.restarts_blocked = 0

    def fault(self, reason):
        self.phase, self.reason = 'FAULT', str(reason)

    def check_plan_endpoint(self, plan, adding=False):
        if self.endpoint is not None and (not adding or self.phase == 'TRACK') and norm(plan.endpoint-self.endpoint) > .02:
            raise ValueError('Committed endpoint changed; explicit new session required' if adding else
                             'Command belongs to a different goal')

    def can_hold_endpoint(self, plan, position):
        return True

    def add_plan(self, msg):
        plan = Plan(msg)
        self.check_plan_endpoint(plan, adding=True)
        self.plans.append(plan)

    def command(self, stamp, raw, velocity, acceleration, position, quaternion, measured_position=None):
        if self.phase == 'FAULT':
            return False
        vectors = tuple(np.asarray(v,dtype=float) for v in (raw,velocity,acceleration,position))
        raw, velocity, acceleration, position = vectors
        quaternion = np.asarray(quaternion,dtype=float)
        if stamp <= 0 or any(v.shape != (3,) for v in vectors) or not np.all(np.isfinite(np.concatenate(vectors))):
            raise ValueError('Invalid command vectors/stamp')
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)) or abs(norm(quaternion)-1) > .02:
            raise ValueError('Invalid converted attitude')
        if self.last is not None and stamp <= self.last[0]:
            raise ValueError('Nonincreasing command stamp')
        if self.phase == 'END_HOLD':
            if norm(position-self.last[1]) > .05:
                self.restarts_blocked += 1
            return False  # Keep the verified endpoint; never accept another trajectory.
        speed,accel=norm(velocity),norm(acceleration)
        speed_exceeded=speed>self.max_speed+1e-9
        acceleration_exceeded=accel>self.max_acceleration+1e-9
        if speed_exceeded or acceleration_exceeded:
            self.last_motion_violation=dict(stamp_ns=int(stamp),speed_mps=speed,acceleration_mps2=accel,
                speed_target_mps=self.max_speed,acceleration_target_mps2=self.max_acceleration,
                velocity_xyz=velocity.tolist(),acceleration_xyz=acceleration.tolist())
            if self.motion_policy=='strict':
                raise ValueError(f'Advertised planner derivative exceeds {self.max_speed:g}/{self.max_acceleration:g}; '
                    f'speed={speed:.9f} m/s, acceleration={accel:.9f} m/s^2; stamp_ns={stamp}')
        plan = next((p for p in reversed(self.plans) if p.matches(stamp,raw,velocity,acceleration)),None)
        if plan is None:
            return None  # DDS topics may arrive in either order; bounded node queue retries.
        self.motion_summary['checked_samples']+=1
        self.motion_summary['speed_exceedances']+=int(speed_exceeded)
        self.motion_summary['acceleration_exceedances']+=int(acceleration_exceeded)
        self.motion_summary['peak_speed_mps']=max(self.motion_summary['peak_speed_mps'],speed)
        self.motion_summary['peak_acceleration_mps2']=max(self.motion_summary['peak_acceleration_mps2'],accel)
        self.check_plan_endpoint(plan)
        self.frame.check(position)
        if self.last is None:
            measured = self.frame.origin if measured_position is None else np.asarray(measured_position,dtype=float)
            if measured.shape != (3,) or not np.all(np.isfinite(measured)):
                raise ValueError('Invalid measured start position')
            offset = self.frame.axes @ (position-measured)
            if np.any(np.abs(offset) > .10):
                raise ValueError('Initial reference offset exceeds 0.10 m per axis: '+str(offset.round(4).tolist()))
            self.initial_offset = offset.copy()
        else:
            dt = (stamp-self.last[0])/1e9
            step = norm(position-self.last[1])
            allowed_step = .015+self.max_speed*dt
            continuity_basis='configured speed limit'
            if self.motion_policy=='observe' and self.last_plan is not None:
                allowed_step=.015+self.last_plan.displacement_bound(
                    self.last[0]/1e9-self.last_plan.start,stamp/1e9-self.last_plan.start)
                continuity_basis='previous committed polynomial'
            if dt > .25 or step > allowed_step:
                triggers = []
                if dt > .25:triggers.append('stamp gap')
                if step > allowed_step:triggers.append('position step')
                before = self.frame.coordinates(self.last[1]).round(4).tolist()
                after = self.frame.coordinates(position).round(4).tolist()
                previous = self.last_evidence
                details = (f'Reference gap or discontinuity ({", ".join(triggers)}): '
                           f'dt={dt:.6f}s (max 0.250000s); step={step:.6f}m (allowed {allowed_step:.6f}m; {continuity_basis}); '
                           f'previous/current forward-left-FC-height={before}/{after}; '
                           f'previous/current stamp_ns={self.last[0]}/{stamp}')
                if previous is not None:
                    details += (f'; previous/current raw xyz={previous["raw"].round(4).tolist()}/{raw.round(4).tolist()}; '
                                f'previous speed/acc={norm(previous["velocity"]):.6f}/{norm(previous["acceleration"]):.6f}; '
                                f'previous/current plan time remaining={previous["end"]-self.last[0]/1e9:.6f}/{plan.end-stamp/1e9:.6f}s; '
                                f'previous raw endpoint distance={norm(previous["raw"]-previous["endpoint"]):.6f}m')
                raise ValueError(details+'; no automatic reconciliation')
        self.endpoint = plan.endpoint.copy()
        self.last = stamp, position.copy(), np.array(quaternion,dtype=float,copy=True)
        self.last_evidence = dict(raw=raw.copy(),velocity=velocity.copy(),acceleration=acceleration.copy(),
                                  end=plan.end,endpoint=plan.endpoint.copy())
        self.last_plan = plan
        self.phase = 'TRACK'
        self.accepted += 1
        # Use planned end time AND raw endpoint/terminal derivatives. A zero-speed
        # sample or PositionCommand flag alone cannot establish completion.
        if (stamp/1e9 >= plan.end and norm(raw-plan.endpoint) <= .015
                and norm(velocity) <= .03 and norm(acceleration) <= .10
                and norm(plan.end_velocity) <= .03 and norm(plan.end_acceleration) <= .10
                and self.can_hold_endpoint(plan,position)):
            self.phase = 'END_HOLD'
        return True


class RollingGoalReference(Reference):
    """Allow changing committed stages, but hold only the fixed navigation goal.

    Callbacks check the stage endpoint/corridor and the final requested goal.
    Polynomial agreement, derivatives, initial offset and continuity remain the
    base Reference policy. The default short-goal Reference is unchanged.
    """
    def __init__(self,frame,endpoint_check,final_goal_check,motion_policy='strict'):
        super().__init__(frame,2.,1.,motion_policy)
        self.endpoint_check=endpoint_check;self.final_goal_check=final_goal_check

    def check_plan_endpoint(self,plan,adding=False):
        if adding:self.endpoint_check(plan)

    def can_hold_endpoint(self,plan,position):
        return self.final_goal_check(plan,position)


def node_type():
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import ExtendedState, EstimatorStatus
    from mars_quadrotor_msgs.msg import PolynomialTrajectory
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data

    Base = coordinate_node_type(node_name='bounded_reference_preview',
                                output_topic='/super_debug/bounded_control_target')

    class BoundedPreview(Base):
        def __init__(self):
            super().__init__()
            self.reference = None
            self.latest_aligned_pose = None
            self.pending = deque(maxlen=64)
            self.poly_pending = deque(maxlen=24)
            self.feedback = {}
            self.locked_sources = None
            self.fault_reason = ''
            self.fault_phase = None
            self.graph_at = -math.inf
            self.alignment_path = Path(self.get_parameter('alignment_path').value)
            self.alignment_hash = hashlib.sha256(self.alignment_path.read_bytes()).digest()
            self.status_pub = self.create_publisher(String, '/super_debug/bounded_reference_status',10)
            self.topics = ('/Odometry','/super_debug/aligned_pose_cov','/mavros/state',
                           '/mavros/extended_state','/mavros/estimator_status',
                           '/planning/pos_cmd','/planning_cmd/poly_traj')
            self.sources_ready = False
            for name,kind,topic in (
                ('ground',ExtendedState,'/mavros/extended_state'),
                ('estimator',EstimatorStatus,'/mavros/estimator_status')):
                self.inputs.append(self.create_subscription(kind,topic,
                    self.feedback_callback(name),qos_profile_sensor_data))
            self.inputs.append(self.create_subscription(PolynomialTrajectory,'/planning_cmd/poly_traj',
                                                       self.polynomial_received,qos_profile_sensor_data))
            self.output_timer = self.create_timer(.02,self.tick)
            self.get_logger().info('GROUND REFERENCE ONLY: forward -0.10..5 m, left +/-2 m, '
                                   'FC floor height -0.10..2 m; initial offset <=0.10 m per axis; '
                                   'first-goal endpoint hold. No native output.')

        def feedback_callback(self,name):
            def received(msg):
                self.feedback[name] = msg,time.monotonic()
            return received

        def fail(self,reason):
            if self.fault_reason:
                return  # Preserve the first fault and its tracking phase.
            self.fault_phase = self.reference.phase if self.reference is not None else 'UNLOCKED'
            self.fault_reason = str(reason)
            if self.reference is not None:
                self.reference.fault(reason)
            self.warn('FAULT latched: '+str(reason))

        def ready(self):
            if not super().ready() or self.fault_reason or not self.sources_ready:
                return False
            for name in ('ground','estimator'):
                if name not in self.feedback:
                    return False
                msg,at = self.feedback[name]
                if time.monotonic()-at > 2 or not self.fresh(self.stamp(msg),2):
                    return False
            ground = self.feedback['ground'][0]
            est = self.feedback['estimator'][0]
            return (ground.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
                    and est.attitude_status_flag and est.velocity_horiz_status_flag
                    and est.velocity_vert_status_flag and est.pos_horiz_rel_status_flag
                    and est.pos_vert_abs_status_flag and not est.accel_error_status_flag)

        def command_received(self,msg):
            self.pending.append(msg)

        def polynomial_received(self,msg):
            # HEART_BEAT has empty coefficient arrays; it is not a new trajectory.
            if msg.type & PolynomialTrajectory.EMER_STOP:
                self.fail('Planner emergency trajectory')
            elif msg.type & PolynomialTrajectory.POSITION_TRAJ:
                self.poly_pending.append(msg)

        def aligned_received(self,msg):
            if msg.header.frame_id == 'px4_enu_preview' and self.fresh(self.stamp(msg)):
                self.latest_aligned_pose = msg.pose.pose
            super().aligned_received(msg)
            if (self.reference is None and self.mapping.pairs >= 10 and self.ready()
                    and self.fresh(self.stamp(msg)) and msg.header.frame_id == 'px4_enu_preview'):
                try:
                    self.reference = Reference(TakeoffFrame(*arrays(msg.pose.pose)))
                except ValueError as exc:
                    self.fail(exc)
                    return
                self.get_logger().info(f'GROUND FRAME LOCKED: yaw={math.degrees(self.reference.frame.yaw):.2f} deg; '
                                       'rest FC height=0.085 m; await one diagnostic goal')

        def check_sources(self):
            sources = {}
            for topic in self.topics:
                gids = {bytes(info.endpoint_gid) for info in self.get_publishers_info_by_topic(topic)}
                if len(gids) != 1:
                    self.sources_ready = False
                    if self.locked_sources is not None:
                        self.fail('Missing/multiple publishers: '+topic)
                    return
                sources[topic] = next(iter(gids))
            if self.locked_sources is not None and sources != self.locked_sources:
                self.fail('Publisher identity changed')
                return
            self.locked_sources = sources
            self.sources_ready = True

        def publish_pose(self,stamp,p,q):
            msg = PoseStamped()
            msg.header.stamp, msg.header.frame_id = stamp,'px4_enu_preview'
            msg.pose.position.x,msg.pose.position.y,msg.pose.position.z = map(float,p)
            msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w = map(float,q)
            self.publisher.publish(msg)

        def tick(self):
            if self.arm_seen:
                self.fail('Arming observed; ground preview permanently disabled')
            if self.mapping.failed:
                self.fail('Session coordinate mapping changed')
            if time.monotonic()-self.graph_at >= .2:
                self.check_sources()
                self.graph_at = time.monotonic()
            try:
                if hashlib.sha256(self.alignment_path.read_bytes()).digest() != self.alignment_hash:
                    self.fail('Active alignment file changed')
            except OSError as exc:
                self.fail(exc)
            if not self.ready() or self.reference is None:
                self.pending.clear()
                return
            try:
                while self.poly_pending:
                    msg = self.poly_pending.popleft()
                    if not self.fresh(self.stamp(msg)):
                        raise ValueError('Stale committed trajectory message')
                    self.reference.add_plan(msg)
                while self.pending:
                    msg = self.pending[0]
                    ns = self.stamp(msg)
                    if not self.fresh(ns):
                        raise ValueError('Stale command while waiting for matching trajectory')
                    if msg.trajectory_flag != 1:
                        raise ValueError('Only regular trajectories supported in this ground preview')
                    p,q = self.mapping.command(msg)
                    vectors = [np.array([getattr(getattr(msg,f),axis) for axis in ('x','y','z')])
                               for f in ('position','velocity','acceleration')]
                    if self.latest_aligned_pose is None:
                        raise ValueError('Missing measured start pose')
                    measured,_ = arrays(self.latest_aligned_pose)
                    accepted = self.reference.command(ns,*vectors,p,q,measured_position=measured)
                    if accepted is None:
                        break
                    self.pending.popleft()
                    if accepted:
                        self.publish_pose(msg.header.stamp,p,q)
                if self.reference.phase == 'TRACK' and self.reference.last is not None and not self.fresh(self.reference.last[0]):
                    raise ValueError('Command stream expired before confirmed endpoint')
                if self.reference.phase == 'END_HOLD':
                    _,p,q = self.reference.last
                    self.publish_pose(self.get_clock().now().to_msg(),p,q)
            except (ValueError,OverflowError) as exc:
                context = ''
                if self.latest_aligned_pose is not None:
                    try:
                        measured,_ = arrays(self.latest_aligned_pose)
                        relative = self.reference.frame.coordinates(measured).round(4).tolist()
                        context = '; measured forward/left/FC-height='+str(relative)
                    except ValueError:
                        pass
                self.fail(str(exc)+context)

        def report(self):
            if self.fault_reason:
                text = ('FAULT: '+self.fault_reason+f'; phase before fault={self.fault_phase}; '
                        f'accepted={self.reference.accepted if self.reference is not None else 0}; DEBUG ONLY')
            elif not self.ready() or self.reference is None:
                text = 'WAIT: fresh grounded feedback, fixed sources and paired geometry'
            else:
                r = self.reference
                coordinates = r.frame.coordinates(r.last[1]).round(4).tolist() if r.last else None
                text = (f'{r.phase}: forward/left/FC-height={coordinates}; '
                        f'initial forward/left/up offset={r.initial_offset.round(4).tolist() if r.initial_offset is not None else None}; '
                        f'accepted={r.accepted}; subsequent references blocked={r.restarts_blocked}; DEBUG ONLY')
            msg = String();msg.data = text
            self.status_pub.publish(msg)
            self.get_logger().info(text)

    return BoundedPreview


def main():
    import rclpy
    rclpy.init()
    node = node_type()()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
