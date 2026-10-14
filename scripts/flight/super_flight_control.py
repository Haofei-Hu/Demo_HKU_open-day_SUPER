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

from first_hover_terminal import FirstHoverControl, NATIVE
from control_bridge_preview import Mapping, arrays, load_converter, quaternion_rpy
from bounded_reference_preview import Reference, Plan, TakeoffFrame


def inverse_goal(mapping, raw_pose, target):
    """Retain planner yaw; invert the same terminal FC-centre lever conversion."""
    if mapping.failed or mapping.origin is None or mapping.pairs<10:
        raise ValueError('Need ten fixed-geometry localization pairs')
    _,q=arrays(raw_pose)
    x,y,z,w=q
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    goal_q=quaternion_rpy(0.,0.,yaw)
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

    def __init__(self,execute=False,hover_fc_height=1.,hold_seconds=6.,distance=.3,
                 alignment_path=None,planner_config=None):
        if not math.isfinite(distance) or not 0<distance<=.3:
            raise ValueError('Initial SUPER handover distance must be in (0, 0.30] m')
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
                or cfg['traj_opt']['boundary']['max_vel']!=.25
                or cfg['traj_opt']['boundary']['max_acc']!=.45):
            raise ValueError('Need the reserved low-speed-3d planner profile')
        self.mapping=Mapping(load_converter(self.alignment_path))
        self.raw_cache={};self.aligned_cache={}
        self.pair_ns=0;self.raw_pose=None;self.aligned_pose=None
        self.planner_sources=None
        self.frame=self.follower=None
        self.requested_target=self.goal_q=None
        self.goal_issued_ns=0
        self.pending=deque(maxlen=64);self.poly_pending=deque(maxlen=24)
        self.endpoint_since=None
        self.audit_used=False
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
            'DISARMED AUDIT ONLY; ')+f'forward {distance:g} m, 0.25/0.45 planner, '
            'initial/endpoint/tracking tolerance 0.10 m per axis; one goal, no restart forwarding.')

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

    def validate(self,ground=False):
        if not self.execute and self.state is not None and (self.state.armed or self.state.mode=='OFFBOARD'):
            raise ValueError('Arming/OFFBOARD observed; audit permanently disabled')
        xyz,yaw,identities=super().validate(ground=ground)
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

    def check_envelope(self,xyz,yaw,ground):
        if self.reference is None:return
        if ground:
            offset=self.frame.axes@(np.asarray(xyz)-np.asarray(self.reference.ground))
            if np.any(np.abs(offset)>.10):
                raise ValueError('Estimated ground offset exceeds 0.10 m per axis; '
                                 'forward/left/up='+str(offset.round(4).tolist()))
        if self.frame is not None:self.frame.check(xyz)
        if self.execute and self.phase in ('HOLD','WAIT_PLAN','TRACK','END_HOLD'):
            self.target_ok(xyz,label='Measured position')
        if abs(self.angle(yaw-self.reference.yaw))>math.radians(15):
            raise ValueError('Heading departure exceeds 15 degrees')

    def change(self,phase):
        super().change(phase)
        if phase=='READY':
            self.frame=TakeoffFrame(self.reference.ground,quaternion_rpy(0.,0.,self.reference.yaw))

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

    def issue_goal(self):
        if self.goal_pub.get_subscription_count()<1 or self.count_publishers('/goal_pose')!=1:
            raise ValueError('Need planner goal subscriber and this controller as sole goal source')
        self.follower=Reference(self.frame)
        self.pending.clear();self.poly_pending.clear()
        target=np.array(self.reference.ground,dtype=float)
        target+=self.frame.axes.T@np.array([self.distance,0.,self.reference.lift if self.execute else 0.])
        self.target_ok(target)
        raw,q=inverse_goal(self.mapping,self.raw_pose,target)
        msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,raw)
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=map(float,q)
        self.requested_target,self.goal_q=target,q
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
            if plan.duration>20.:raise ValueError('Initial handover trajectory exceeds 20 s')
            endpoint=self.mapping.converter(plan.endpoint,self.goal_q,np.eye(6))[0]-self.mapping.origin
            error=self.frame.axes@(endpoint-self.requested_target)
            if np.any(np.abs(error)>.10):raise ValueError('Map-relocated endpoint exceeds 0.10 m per axis')
            self.target_ok(endpoint)
            self.follower.add_plan(msg)
        while self.pending:
            msg=self.pending[0];ns=self.stamp(msg)
            if ns<self.goal_issued_ns:self.pending.popleft();continue
            if not self.fresh_ns(ns):raise ValueError('Stale command awaiting committed polynomial')
            if msg.trajectory_flag!=1:raise ValueError('Only regular initial handover trajectories supported')
            p,q=self.mapping.command(msg)
            self.target_ok(p)
            vectors=[np.array([getattr(getattr(msg,name),a) for a in ('x','y','z')])
                     for name in ('position','velocity','acceleration')]
            accepted=self.follower.command(ns,*vectors,p,q,measured_position=xyz)
            if accepted is None:break
            self.pending.popleft()
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
        self.status_pub.publish(String(data=f'{self.phase}: forward/left/FC-height={coordinates}; '
            f'accepted={accepted}; {mode}; callback max={self.max_callback_runtime:.4f}s; '
            f'timer gap max={self.max_timer_gap:.4f}s; fault={self.fault}'))


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
