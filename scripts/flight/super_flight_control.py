#!/usr/bin/env python3
"""One-goal low-speed SUPER handover; default is disarmed debug auditing only.

The audit requests a 0.30 m forward goal at the captured ground height and
validates the committed polynomial, initial offset, continuity and endpoint.
It creates no native publisher and makes no mode or arming requests.
--execute still requires fresh RC/POSCTL and an explicit /super_flight/start.
"""
import argparse
from collections import deque
import math
import os
from pathlib import Path
import time

# This process only uses small matrices; avoid unnecessary BLAS worker pools.
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mars_quadrotor_msgs.msg import PositionCommand, PolynomialTrajectory
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from first_hover_terminal import FirstHoverControl, NATIVE, PARAMETERS
from control_bridge_preview import Mapping, arrays, load_converter, quaternion_rpy
from bounded_reference_preview import Reference, Plan, TakeoffFrame


def inverse_goal(mapping, raw_pose, target,goal_quaternion=None):
    """Retain planner yaw; invert the same terminal FC-centre lever conversion."""
    if mapping.failed or mapping.origin is None or mapping.pairs<10:
        raise ValueError('Need ten fixed-geometry localization pairs')
    _,q=arrays(raw_pose)
    x,y,z,w=q
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    goal_q=quaternion_rpy(0.,0.,yaw)
    if goal_quaternion is not None:
        goal_q=np.array(goal_quaternion,dtype=float,copy=True)
        if (goal_q.shape!=(4,) or not np.all(np.isfinite(goal_q))
                or abs(np.linalg.norm(goal_q)-1)>.02 or np.max(np.abs(goal_q[:2]))>1e-8):
            raise ValueError('Expected a finite fixed planner yaw quaternion')
        goal_q/=np.linalg.norm(goal_q)
    target=np.asarray(target,dtype=float)
    zero=mapping.converter(np.zeros(3),goal_q,np.eye(6))[0]
    basis=np.column_stack([mapping.converter(axis,goal_q,np.eye(6))[0]-zero for axis in np.eye(3)])
    if (target.shape!=(3,) or not np.all(np.isfinite(target)) or not np.all(np.isfinite(zero))
            or not np.allclose(basis.T@basis,np.eye(3),atol=1e-8,rtol=0)
            or abs(np.linalg.det(basis)-1)>1e-8):
        raise ValueError('Expected finite target and rigid session conversion')
    raw=np.linalg.solve(basis,target+mapping.origin-zero)
    check=mapping.converter(raw,goal_q,np.eye(6))[0]-mapping.origin
    if not np.all(np.isfinite(raw)) or not np.allclose(check,target,atol=1e-8,rtol=0):
        raise ValueError('Goal conversion round-trip failed')
    return raw,goal_q


class SuperFlightControl(FirstHoverControl):
    ALLOW_EXTENDED_EXECUTION=False
    PLANNER_LIMITS=(.25,.45)
    REFERENCE_LIMITS=(.3,.5)
    MAX_DISTANCE=.3
    FRAME_FORWARD_LIMIT=5.
    MAX_TRAJECTORY_SECONDS=20.
    NODE_NAME='super_flight'
    DEBUG_TOPIC='/super_debug/super_flight_target'
    START_SERVICE='/super_flight/start'
    LAND_SERVICE='/super_flight/land'
    RC_IN_CHECK=False  # Remote auditing only; execute always retains the RC gate.
    SOURCE_TOPICS=('/Odometry','/super_debug/aligned_pose_cov',
                   '/planning/pos_cmd','/planning_cmd/poly_traj')

    def startup_message(self):
        return ('EXECUTE ENABLED; awaiting explicit /super_flight/start; rise, hover, one SUPER goal, hold, LAND.'
                if self.execute else 'CHECK ONLY; await READY then /super_flight/audit; no native/mode/arming output.')

    def position_policy_message(self):
        return 'initial/endpoint/tracking tolerance 0.10 m per axis'

    def goal_policy_message(self):
        return 'one goal, no restart forwarding'

    def route_description(self):
        return f'forward {self.distance:g} m'

    def __init__(self,execute=False,hover_fc_height=1.,hold_seconds=6.,distance=.3,
                 alignment_path=None,planner_config=None):
        if execute and not self.ALLOW_EXTENDED_EXECUTION and (self.PLANNER_LIMITS!=(.25,.45) or self.REFERENCE_LIMITS!=(.3,.5)
                        or self.MAX_DISTANCE>.3 or self.FRAME_FORWARD_LIMIT>5.):
            raise ValueError('Extended speed/distance profiles are ground-audit only; flight blocked')
        if not math.isfinite(distance) or not 0<distance<=self.MAX_DISTANCE:
            raise ValueError(f'Initial SUPER handover distance must be in (0, {self.MAX_DISTANCE:g}] m')
        self.callback_runtime=self.max_callback_runtime=self.max_timer_gap=0.
        super().__init__(execute,hover_fc_height,hold_seconds)
        self.distance=float(distance)
        self.offboard_budget=45.
        self.declare_parameter('alignment_path',str(alignment_path or Path.home()/'Downloads/aligned_pose_preview.py'))
        self.declare_parameter('planner_config',str(planner_config or
            Path.home()/'super_ws/src/SUPER/super_planner/config/mid360_low_speed_3d_goal_ros2.yaml'))
        self.alignment_path=Path(self.get_parameter('alignment_path').value)
        self.config_path=Path(self.get_parameter('planner_config').value)
        self.alignment_bytes=self.alignment_path.read_bytes()
        self.config_bytes=self.config_path.read_bytes()
        cfg=yaml.safe_load(self.config_bytes)
        if (cfg['fsm']['click_height']!=-6. or cfg['fsm']['click_goal_topic']!='/goal_pose'
                or not cfg['fsm']['click_goal_en'] or cfg['fsm']['cmd_topic']!='/planning/pos_cmd'
                or cfg['traj_opt']['boundary']['max_vel']!=self.PLANNER_LIMITS[0]
                or cfg['traj_opt']['boundary']['max_acc']!=self.PLANNER_LIMITS[1]):
            raise ValueError(f'Need 3D planner limits {self.PLANNER_LIMITS}; profile mismatch')
        self.mapping=Mapping(load_converter(self.alignment_path))
        self.raw_cache={};self.aligned_cache={}
        self.pair_ns=0;self.raw_pose=None;self.aligned_pose=None
        self.planner_sources=None
        self.frame=self.follower=None
        self.requested_target=self.requested_raw=self.goal_q=None
        self.goal_issued_ns=0
        self.pending=deque(maxlen=64);self.poly_pending=deque(maxlen=24)
        self.endpoint_since=None
        self.audit_used=False
        self.preplan_commands_discarded=0
        self.parameter_errors={}
        self.goal_pub=self.create_publisher(PoseStamped,'/goal_pose',10)
        self.status_pub=self.create_publisher(String,'/super_debug/super_flight_status',10)
        self.create_subscription(Odometry,'/Odometry',self.raw_received,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov',
                                 self.aligned_received,qos_profile_sensor_data)
        self.create_subscription(PositionCommand,'/planning/pos_cmd',self.command_received,qos_profile_sensor_data)
        self.create_subscription(PolynomialTrajectory,'/planning_cmd/poly_traj',self.plan_received,qos_profile_sensor_data)
        self.create_service(Trigger,'/super_flight/audit',self.audit)
        self.create_timer(1.,self.report)
        self.get_logger().info('SUPER HANDOVER: '+('explicit flight start required; ' if execute else
            'DISARMED AUDIT ONLY; ')+f'{self.route_description()}, {self.PLANNER_LIMITS[0]:g}/{self.PLANNER_LIMITS[1]:g} planner, '
            +self.position_policy_message()+'; '+self.goal_policy_message()+'.')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec

    def fresh_ns(self,ns):
        return ns>0 and -.05<=(self.get_clock().now().nanoseconds-ns)/1e9<=.25

    def raw_received(self,msg):
        if msg.header.frame_id=='camera_init' and msg.child_frame_id=='body':
            self.pair(self.raw_cache,msg,msg.pose.pose)

    def aligned_received(self,msg):
        if msg.header.frame_id=='px4_enu_preview':
            self.pair(self.aligned_cache,msg,msg.pose.pose)

    def pair(self,bucket,msg,pose):
        ns=self.stamp(msg)
        if ns<=self.pair_ns or not self.fresh_ns(ns):return
        bucket[ns]=pose
        while len(bucket)>100:bucket.pop(next(iter(bucket)))
        if ns in self.raw_cache and ns in self.aligned_cache:
            try:
                raw,aligned=self.raw_cache.pop(ns),self.aligned_cache.pop(ns)
                self.mapping.pair(raw,aligned)
                self.raw_pose,self.aligned_pose,self.pair_ns=raw,aligned,ns
            except ValueError as exc:self.abort(str(exc))

    def plan_received(self,msg):
        active=self.phase in ('WAIT_PLAN','TRACK','END_HOLD','AUDIT_WAIT','AUDIT_TRACK','AUDIT_END')
        if active and msg.type & PolynomialTrajectory.EMER_STOP:
            self.abort('Planner emergency stop')
        elif active and msg.type & PolynomialTrajectory.POSITION_TRAJ:
            self.poly_pending.append(msg)

    def command_received(self,msg):
        if self.phase in ('WAIT_PLAN','TRACK','AUDIT_WAIT','AUDIT_TRACK'):
            self.pending.append(msg)

    def parameter_result(self,key,future):
        # Keep the original strict confirmation behavior; add its missing
        # rejection evidence instead of treating an unconfirmed reply as valid.
        super().parameter_result(key,future)
        if key in self.confirmed:
            self.parameter_errors.pop(key,None)
            return
        expected=self.EXPECTED_PARAMETERS if key=='fc' else {'mav_frame':'LOCAL_NED'}
        try:
            if future.cancelled():raise ValueError('Request cancelled after response timeout')
            response=future.result()
            if response is None:raise ValueError('Empty parameter service response')
            if len(response.values)!=len(expected):
                raise ValueError(f'Expected {len(expected)} values, received {len(response.values)}')
            details=[]
            for (name,target),value in zip(expected.items(),response.values):
                kind=4 if isinstance(target,str) else (2 if isinstance(target,int) else 3)
                actual={2:value.integer_value,3:value.double_value,4:value.string_value}.get(value.type)
                good=(value.type==kind and (actual==target if kind==4 else
                      math.isfinite(actual) and abs(actual-target)<=1e-6))
                if not good:details.append(f'{name}: actual={actual!r}, type={value.type}; expected={target!r}, type={kind}')
            self.parameter_errors[key]='; '.join(details) or 'Response failed strict confirmation'
        except Exception as exc:
            self.parameter_errors[key]=str(exc)

    def parameter_wait_detail(self):
        now=time.monotonic();details=[]
        for key,client in self.parameter_clients.items():
            age=now-self.confirmed.get(key,-math.inf)
            if age<=3:continue
            pending=self.parameter_futures.get(key)
            pending_age=now-pending[1] if pending is not None and not pending[0].done() else None
            pending_text=f'{pending_age:.3f}s' if pending_age is not None else 'none'
            service='/mavros/param/get_parameters' if key=='fc' else '/mavros/setpoint_position/get_parameters'
            details.append(f'{service}: ready={client.service_is_ready()}, confirmation_age={age:.3f}s, '
                f'pending_age={pending_text}, last_error={self.parameter_errors.get(key,"No valid response yet")}')
        return '; '.join(details)

    def validate(self,ground=False):
        if not self.execute and self.state is not None and (self.state.armed or self.state.mode=='OFFBOARD'):
            raise ValueError('Arming/OFFBOARD observed; audit permanently disabled')
        try:
            xyz,yaw,identities=super().validate(ground=ground)
        except ValueError as exc:
            if str(exc)=='Waiting for confirmed fusion, speed limits, LAND fallback and LOCAL_NED':
                raise ValueError(str(exc)+'; '+self.parameter_wait_detail()) from exc
            if str(exc) in ('Missing/stale local','Missing/stale vision'):
                field=str(exc).rsplit(' ',1)[1]
                receipt_age=time.monotonic()-self.receipts.get(field,-math.inf)
                msg=getattr(self,field)
                stamp_ns=self.stamp(msg) if msg is not None else 0
                header_age=(self.get_clock().now().nanoseconds-stamp_ns)/1e9 if stamp_ns>0 else math.inf
                raise ValueError(f'Missing/stale {field}; '
                    f'receive_age={receipt_age:.6f}s (max 0.250000s); '
                    f'header_age={header_age:.6f}s (range -0.050000..0.250000s); '
                    f'{field}_stamp_ns={stamp_ns}; phase={self.phase}') from exc
            raise
        if not self.execute and self.extended.landed_state!=self.extended.LANDED_STATE_ON_GROUND:
            raise ValueError('Audit requires grounded aircraft')
        try:
            unchanged=(self.alignment_path.read_bytes()==self.alignment_bytes and
                       self.config_path.read_bytes()==self.config_bytes)
        except OSError as exc:raise ValueError('Cannot verify session geometry/configuration: '+str(exc)) from exc
        if not unchanged:raise ValueError('Session geometry/planner configuration changed')
        if self.mapping.failed or self.mapping.pairs<10 or not self.fresh_ns(self.pair_ns):
            raise ValueError('Waiting for ten fresh, fixed-geometry localization pairs')
        aligned,_=arrays(self.aligned_pose)
        if np.any(np.abs(np.asarray(xyz)-aligned)>.10):
            raise ValueError('FC/aligned pose discrepancy exceeds 0.10 m per axis')
        sources={}
        for topic in self.SOURCE_TOPICS:
            infos=self.get_publishers_info_by_topic(topic)
            if len(infos)!=1:raise ValueError('Need one source for '+topic)
            sources[topic]=bytes(infos[0].endpoint_gid)
        if self.planner_sources is not None and sources!=self.planner_sources:
            raise ValueError('Planner/localization source identity changed')
        self.planner_sources=sources
        if not self.execute and self.count_publishers(NATIVE):
            raise ValueError('Stop native ground sender before handover audit')
        return xyz,yaw,identities

    def check_measured_position(self,xyz):
        return self.frame.check(xyz)

    def check_envelope(self,xyz,yaw,ground):
        if self.reference is None:return
        if ground:
            offset=self.frame.axes@(np.asarray(xyz)-np.asarray(self.reference.ground))
            if np.any(np.abs(offset)>.10):
                raise ValueError('Estimated ground offset exceeds 0.10 m per axis; '
                                 'forward/left/up='+str(offset.round(4).tolist()))
        if self.frame is not None:self.check_measured_position(xyz)
        if self.execute and self.phase in ('HOLD','WAIT_PLAN','TRACK','END_HOLD'):
            self.target_ok(xyz,label='Measured position')
        if abs(self.angle(yaw-self.reference.yaw))>math.radians(15):
            raise ValueError('Heading departure exceeds 15 degrees')

    def change(self,phase):
        super().change(phase)
        if phase=='READY':
            self.frame=TakeoffFrame(self.reference.ground,quaternion_rpy(0.,0.,self.reference.yaw),
                                    max_forward=self.FRAME_FORWARD_LIMIT)

    def allowed_modes(self):
        return dict(super().allowed_modes(),WAIT_PLAN=('OFFBOARD',),TRACK=('OFFBOARD',),END_HOLD=('OFFBOARD',))

    def target_ok(self,position,label='Reference'):
        p=self.frame.check(position)
        nominal_height=self.hover_fc_height if self.execute else .085
        if not (-.10<=p[0]<=self.distance+.10 and abs(p[1])<=.10 and abs(p[2]-nominal_height)<=.10):
            raise ValueError(label+' outside initial straight-line handover corridor: '+str(p.round(4).tolist()))

    def tracking_ok(self,xyz,target):
        error=self.frame.axes@(np.asarray(xyz)-np.asarray(target))
        if np.any(np.abs(error)>.10):
            raise ValueError('Tracking error exceeds 0.10 m per axis: '+str(error.round(4).tolist()))

    def make_follower(self):
        return Reference(self.frame,*self.REFERENCE_LIMITS)

    def endpoint_evidence(self,plan):
        endpoint=self.mapping.converter(plan.endpoint,self.goal_q,np.eye(6))[0]-self.mapping.origin
        error=self.frame.axes@(endpoint-self.requested_target)
        detail=(f'requested planner xyz={self.requested_raw.round(6).tolist()}; '
                f'committed planner endpoint={plan.endpoint.round(6).tolist()}; '
                f'requested aligned xyz={self.requested_target.round(6).tolist()}; '
                f'committed aligned endpoint={endpoint.round(6).tolist()}; '
                f'endpoint error forward/left/up={error.round(6).tolist()}; '
                f'plan start={plan.start:.6f}, duration={plan.duration:.6f}s')
        return endpoint,error,detail

    def accept_plan(self,plan,msg):
        endpoint,error,detail=self.endpoint_evidence(plan)
        if np.any(np.abs(error)>.10):
            raise ValueError('Committed endpoint differs from requested goal by over 0.10 m per axis; '
                             'this does not establish map relocation; '+detail)
        self.target_ok(endpoint)
        self.follower.add_plan(msg)

    def supported_command(self,flag):
        return flag==1

    def convert_command(self,msg):
        return self.mapping.command(msg)

    def reference_accepted(self,msg):
        pass

    def issue_goal(self):
        if self.goal_pub.get_subscription_count()<1 or self.count_publishers('/goal_pose')!=1:
            raise ValueError('Need planner goal subscriber and this controller as sole goal source')
        self.follower=self.make_follower()
        self.pending.clear();self.poly_pending.clear()
        target=np.array(self.reference.ground,dtype=float)
        target+=self.frame.axes.T@np.array([self.distance,0.,self.reference.lift if self.execute else 0.])
        self.target_ok(target)
        raw,q=inverse_goal(self.mapping,self.raw_pose,target)
        msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,raw)
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=map(float,q)
        self.requested_target,self.requested_raw,self.goal_q=target,raw,q
        self.goal_issued_ns=self.stamp(msg)
        self.goal_pub.publish(msg)
        self.change('WAIT_PLAN' if self.execute else 'AUDIT_WAIT')
        self.get_logger().info(f'One goal: aligned={target.round(4).tolist()}, planner={raw.round(4).tolist()}')

    def audit(self,request,response):
        response.success=False
        if self.execute or self.phase!='READY' or self.audit_used:
            response.message='Audit requires unused check-only READY session'
            return response
        try:
            self.validate(ground=True)
            self.audit_used=True
            self.issue_goal()
            response.success=True
            response.message='Disarmed one-goal audit started; debug output only'
        except (ValueError,OSError) as exc:
            self.abort(str(exc));response.message=str(exc)
        return response

    def start(self,request,response):
        response=super().start(request,response)
        if response.success:
            response.message='Accepted: OFFBOARD, arm, 1 m rise, stable hover, one SUPER goal, endpoint hold, LAND'
        return response

    def stop_requested(self,request,response):
        if self.execute and self.phase in ('RISE','HOLD','WAIT_PLAN','TRACK','END_HOLD'):
            self.land('Operator requested landing')
        else:self.abort('Operator stopped session')
        response.success=True;response.message='Stopped or landing requested; no restart'
        return response

    def process_reference(self,xyz):
        while self.poly_pending:
            msg=self.poly_pending.popleft()
            if self.follower.phase=='END_HOLD':continue
            if not self.fresh_ns(self.stamp(msg)):raise ValueError('Stale committed trajectory')
            plan=Plan(msg)
            if plan.start<self.goal_issued_ns/1e9-.05:continue
            if self.MAX_TRAJECTORY_SECONDS is not None and plan.duration>self.MAX_TRAJECTORY_SECONDS:
                raise ValueError(f'Initial handover trajectory exceeds {self.MAX_TRAJECTORY_SECONDS:g} s')
            self.accept_plan(plan,msg)
        while self.pending:
            msg=self.pending[0];ns=self.stamp(msg)
            if ns<self.goal_issued_ns:self.pending.popleft();continue
            # A previous goal may still emit freshly stamped commands while
            # SUPER plans the new goal. They cannot be judged as its reference.
            if not self.follower.plans:break
            if self.follower.last is None and ns<min(p.start for p in self.follower.plans)*1e9:
                self.pending.popleft();self.preplan_commands_discarded+=1;continue
            if not self.fresh_ns(ns):raise ValueError('Stale command awaiting committed polynomial')
            if not self.supported_command(msg.trajectory_flag):
                raise ValueError('Unsupported initial handover command flag: '+str(msg.trajectory_flag))
            vectors=[np.array([getattr(getattr(msg,name),a) for a in ('x','y','z')])
                     for name in ('position','velocity','acceleration')]
            if not any(plan.matches(ns,*vectors) for plan in self.follower.plans):break
            p,q=self.convert_command(msg)
            self.target_ok(p)
            accepted=self.follower.command(ns,*vectors,p,q,measured_position=xyz)
            if accepted is None:break
            if accepted:self.reference_accepted(msg)
            self.pending.popleft()
        self.check_command_stream()

    def check_command_stream(self):
        if self.follower.phase=='TRACK' and not self.fresh_ns(self.follower.last[0]):
            raise ValueError('Command stream expired before confirmed endpoint')

    def mission_tick(self,now,xyz,yaw):
        if self.phase=='RISE':
            u=min(1.,(now-self.armed_at)/self.reference.ramp_seconds)
            target=np.array(self.reference.ground)+np.array([0.,0.,
                self.reference.lift*(u**3*(10-15*u+6*u*u))])
            self.tracking_ok(xyz,target)
            return False  # Preserve the existing smooth-rise/arrival logic.
        if self.phase=='HOLD':
            target=np.array(self.reference.ground)+np.array([0.,0.,self.reference.lift])
            self.tracking_ok(xyz,target)
            self.publish(target,self.reference.yaw,native=True)
            if now-self.hold_at>=2.:self.issue_goal()
            return True
        active=('WAIT_PLAN','TRACK','END_HOLD','AUDIT_WAIT','AUDIT_TRACK','AUDIT_END')
        if self.phase not in active:return False
        if self.execute and not self.state.armed:raise ValueError('Disarming observed during handover')
        if self.execute and now-self.armed_at>self.offboard_budget:
            self.land('45-second flight budget reached');return True
        if self.follower.phase!='END_HOLD':self.process_reference(xyz)
        if self.follower.last is None:
            if now-self.phase_at>8.:raise ValueError('No verified planner reference within 8 s')
            target=np.array(self.reference.ground)+np.array([0.,0.,self.reference.lift if self.execute else 0.])
        else:
            target=self.follower.last[1]
            desired=('END_HOLD' if self.execute else 'AUDIT_END') if self.follower.phase=='END_HOLD' else (
                'TRACK' if self.execute else 'AUDIT_TRACK')
            if self.phase!=desired:self.change(desired)
        if self.execute:self.tracking_ok(xyz,target)
        self.publish(target,self.reference.yaw,native=self.execute)
        if self.follower.phase=='END_HOLD' and self.execute:
            if self.endpoint_since is None:self.endpoint_since=now
            if now-self.endpoint_since>=self.hold_seconds:self.land('SUPER endpoint hold completed')
        return True

    def tick(self):
        entered=time.monotonic()
        gap=entered-self.last_tick
        if math.isfinite(gap):self.max_timer_gap=max(self.max_timer_gap,gap)
        if self.started and gap>.2:
            self.get_logger().warning(f'Timer gap={gap:.6f}s; previous callback runtime={self.callback_runtime:.6f}s')
        try:super().tick()
        finally:
            self.callback_runtime=time.monotonic()-entered
            self.max_callback_runtime=max(self.max_callback_runtime,self.callback_runtime)

    def report(self):
        accepted=self.follower.accepted if self.follower is not None else 0
        coordinates=(self.frame.coordinates(self.follower.last[1]).round(4).tolist()
                     if self.frame is not None and self.follower is not None and self.follower.last is not None else None)
        mode='EXECUTE' if self.execute else 'DEBUG ONLY'
        parameter_wait=self.parameter_wait_detail() if self.phase=='CHECKING' else ''
        self.status_pub.publish(String(data=f'{self.phase}: forward/left/FC-height={coordinates}; '
            f'accepted={accepted}; preplan commands discarded={self.preplan_commands_discarded}; '
            f'{mode}; callback max={self.max_callback_runtime:.4f}s; '
            f'timer gap max={self.max_timer_gap:.4f}s; fault={self.fault}'+
            (f'; parameter_wait={parameter_wait}' if parameter_wait else '')))


def main():
    import rclpy
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--alignment-path',type=Path)
    parser.add_argument('--planner-config',type=Path)
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=None
    try:
        node=SuperFlightControl(execute=args.execute,alignment_path=args.alignment_path,planner_config=args.planner_config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.abort('Controller interrupted')
            end=time.monotonic()+.3
            while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
