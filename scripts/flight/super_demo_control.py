#!/usr/bin/env python3
"""One complete venue-bounded demonstration; check-only unless explicitly enabled.

    0.5 m virtual / 1.5 m physical takeoff -> SUPER outbound/return -> 6 s hold -> LAND

No parameter writes. Execution also requires an explicit start service, current
RC/POSCTL and the on-site-supervision flag. Reports distinguish simulation.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import PositionTarget
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import MarkerArray

from demo_mission import DemoMission,MissionReport,default_report_path
from first_hover_terminal import PARAMETERS,pose_values
from super_flight_control import SuperFlightControl,inverse_goal
from bounded_reference_preview import Plan,TakeoffFrame,RollingGoalReference
from super_session import NATIVE_CONTROL_TOPICS


def planner_goal_marker(msg,issued_ns):
    """Pinned SUPER vizGoalPath is [goal sphere, robot sphere, goal->robot line]."""
    spheres=[m for m in msg.markers if m.action==0 and m.type==2 and m.ns=='goal']
    lines=[m for m in msg.markers if m.action==0 and m.type==5 and m.ns=='goal_line']
    if len(spheres)!=2 or len(lines)!=1 or len(lines[0].points)!=2:return None
    if any(m.header.frame_id!='world' or
           m.header.stamp.sec*1000000000+m.header.stamp.nanosec<issued_ns for m in [*spheres,*lines]):return None
    def xyz(p):return np.array([p.x,p.y,p.z],dtype=float)
    goal,robot=map(lambda m:xyz(m.pose.position),spheres)
    path=np.array([xyz(p) for p in lines[0].points])
    if not np.all(np.isfinite([goal,robot,*path])):raise ValueError('Nonfinite planner goal feedback')
    if not np.allclose(path,[goal,robot],atol=1e-6,rtol=0):return None
    return goal


class VenuePlan(Plan):
    """Associate fresh stationary commands with a committed rest endpoint."""
    def matches(self,stamp,position,velocity,acceleration):
        if super().matches(stamp,position,velocity,acceleration):return True
        return bool(stamp/1e9>=self.end
            and np.linalg.norm(self.end_velocity)<=.03
            and np.linalg.norm(self.end_acceleration)<=.10
            and np.linalg.norm(np.asarray(position)-self.endpoint)<=.015
            and np.linalg.norm(velocity)<=.03 and np.linalg.norm(acceleration)<=.05)


class VenueFrame(TakeoffFrame):
    def __init__(self,position,quaternion,mission,on_floor):
        super().__init__(position,quaternion,max_forward=mission.distance+.1)
        self.mission,self.on_floor=mission,on_floor

    def check(self,position):
        point=self.coordinates(position)
        self.mission.check(point,on_floor=self.on_floor())
        return point


class VenueReference(RollingGoalReference):
    """Observe initial position offsets; frame checks enforce the entire vehicle's bounds."""
    def check_initial_offset(self,offset):
        pass

    def add_plan(self,msg):
        plan=VenuePlan(msg)
        self.check_plan_endpoint(plan,adding=True)
        self.plans.append(plan)

    def resting_at_endpoint(self):
        plan=self.last_plan;sample=self.last_evidence
        return bool(plan is not None and sample is not None and self.last[0]/1e9>=plan.end
            and np.linalg.norm(sample['raw']-plan.endpoint)<=.015
            and np.linalg.norm(sample['velocity'])<=.03
            and np.linalg.norm(sample['acceleration'])<=.10
            and np.linalg.norm(plan.end_velocity)<=.03
            and np.linalg.norm(plan.end_acceleration)<=.10)

    def max_command_gap(self):
        # Actual position continuity is checked against the committed polynomial.
        # An elapsed stamp interval alone is not a mission failure.
        return math.inf


class DemoMissionControl(SuperFlightControl):
    NODE_NAME='super_demo'
    DEBUG_TOPIC='/super_debug/demo_target'
    START_SERVICE='/super_demo/start'
    LAND_SERVICE='/super_demo/land'
    MAX_HOVER_HEIGHT=2.
    ALLOW_EXTENDED_EXECUTION=True
    PLANNER_LIMITS=REFERENCE_LIMITS=(2.,1.)
    MAX_DISTANCE=4.5
    FRAME_FORWARD_LIMIT=4.6
    MAX_TRAJECTORY_SECONDS=None
    SOURCE_TOPICS=SuperFlightControl.SOURCE_TOPICS+('/visualization/goal',)
    NATIVE_TOPIC='/mavros/setpoint_raw/local'
    EXPECTED_PARAMETERS=dict(PARAMETERS,MPC_XY_VEL_MAX=2.)
    ADVISORY_SPEED=2.4
    ADVISORY_ACCELERATION=1.5

    def __init__(self,mission_file,report_path,execute=False,on_site_supervised=False,
                 alignment_path=None,planner_config=None,simulation=False,motion_policy='strict'):
        if motion_policy not in ('strict','observe'):raise ValueError('Unknown motion policy')
        self.motion_policy=motion_policy;self.motion_warned_legs=set()
        if execute and not on_site_supervised:
            raise ValueError('Flight requires explicit on-site supervision; no flight publisher created')
        self.mission_file=Path(mission_file);self.mission_bytes=self.mission_file.read_bytes()
        config=json.loads(self.mission_bytes)
        if execute and not simulation and config.get('mission_id')!='open_day_out_and_back':
            raise ValueError('Short-site rehearsal profiles cannot enable physical flight')
        if execute and not simulation and str(config.get('status','')).startswith('SYNTHETIC'):
            raise ValueError('Synthetic test geometry cannot enable a physical flight')
        self.mission=DemoMission(config)
        if self.mission.rehearsal_only and not simulation:
            raise ValueError('Ground rehearsal height is for virtual execution only; physical tests require 1.5 m')
        self.MAX_DISTANCE=self.mission.distance
        self.FRAME_FORWARD_LIMIT=self.mission.distance+.1
        self.mission_report=MissionReport(report_path,self.mission,'simulation' if simulation else
                                         ('flight' if execute else 'check-only'))
        self.velocity=None;self.leg=None;self.home=self.far=None
        self.accepted_raw=self.accepted_target=self.previous_accepted_raw=None
        self.leg_start_target=self.return_hold_target=None
        self.arrived_since=self.final_hold_since=None
        self.native_velocity=np.zeros(3);self.goal_count=0;self.stages_started=False
        self.plan_wait_started_at=None;self.plan_wait_elapsed_s=0.
        self.max_tracking_error=np.zeros(3)
        self.sent_references=deque(maxlen=512)
        self.endpoint_kinds=set();self.plan_evidence=[]
        self.mission_report.data['committed_endpoints']=self.plan_evidence
        self.mission_report.data['tracking_policy']='FC pose timestamp matched to previously sent position/velocity reference'
        self.mission_report.data['post_takeoff_position_policy']=dict(
            abort_boundary='Configured outer venue including propeller-inclusive vehicle envelope',
            straight_line_corridor_m=None,tracking_error_abort_m=None,initial_offset_abort_m=None,
            arrival_tolerance_per_axis_m=.1)
        self.mission_report.data['peak_reference_speed_mps']=0.
        self.mission_report.data['peak_reference_acceleration_mps2']=0.
        self.mission_report.data['motion_policy']=motion_policy
        self.mission_report.data['planner_motion_by_leg']={}
        self.mission_report.data['motion_advisories']=dict(speed_mps=self.ADVISORY_SPEED,
            acceleration_mps2=self.ADVISORY_ACCELERATION,action='warning_only',
            derivative_exceedance_alone_stops_motion=motion_policy=='strict')
        self.mission_report.data['motion_advisory_by_leg']={}
        self.mission_report.data['planning_wait_policy']=dict(timeout_s=None,
            action='Hold the leg start until a verified trajectory arrives or the operator stops')
        self.mission_report.data['execution_limits']=dict(controller_timeout_s=None,
            committed_trajectory_duration_s=None,callback_gap_abort_s=None,
            reference_stamp_gap_abort_s=None)
        self.mission_report.data['navigation_goal_policy']=dict(
            source='/visualization/goal',policy='SUPER accepted free-space goal inside vehicle-inclusive venue',
            requested_targets_preserved=True,arrival_tolerance_per_axis_m=.1)
        self.mission_report.data['accepted_goals']={}
        super().__init__(execute=execute,hover_fc_height=self.mission.hover_height,hold_seconds=6.,distance=self.mission.distance,
                         alignment_path=alignment_path,planner_config=planner_config)
        # Also disable the inherited first-hover rise-time fallback.
        self.offboard_budget=math.inf
        self.create_subscription(TwistStamped,'/mavros/local_position/velocity_local',
                                 lambda msg:self.store('velocity',msg),qos_profile_sensor_data)
        self.create_subscription(MarkerArray,'/visualization/goal',self.planner_goal_received,qos_profile_sensor_data)
        self.get_logger().info(f'MISSION {config["mission_id"]}: {self.mission.hover_height:g} m rise, {self.distance:g} m outbound, '
                               'return, 6 s hold, LAND; no parameter writes.')

    def make_native_publisher(self):
        return self.create_publisher(PositionTarget,self.NATIVE_TOPIC,10)

    def completion_target(self):
        target=getattr(self,'accepted_target',None)
        return self.requested_target if target is None else target

    def planner_goal_received(self,msg):
        if self.phase not in ('WAIT_PLAN','TRACK','END_HOLD') or self.leg is None:return
        try:
            sources=self.get_publishers_info_by_topic('/visualization/goal')
            polynomials=self.get_publishers_info_by_topic('/planning_cmd/poly_traj')
            if len(sources)!=1 or len(polynomials)!=1:return
            goal_source,plan_source=sources[0],polynomials[0]
            locked=self.planner_sources
            if (locked is not None and locked.get('/visualization/goal')!=bytes(goal_source.endpoint_gid)):
                return
            if (goal_source.node_name!=plan_source.node_name
                    or goal_source.node_namespace!=plan_source.node_namespace
                    or bytes(goal_source.endpoint_gid)[:12]!=bytes(plan_source.endpoint_gid)[:12]):return
            raw=planner_goal_marker(msg,self.goal_issued_ns)
            if raw is None:return
            if self.previous_accepted_raw is not None and np.allclose(raw,self.previous_accepted_raw,atol=1e-6,rtol=0):
                return  # Old-leg visualization is not a new-goal acknowledgement.
            if self.accepted_raw is not None:
                if not np.allclose(raw,self.accepted_raw,atol=1e-6,rtol=0):
                    raise ValueError('SUPER accepted goal changed within the current navigation leg')
                return
            target=self.mapping.converter(raw,self.goal_q,np.eye(6))[0]-self.mapping.origin
            self.target_ok(target,'SUPER accepted goal')
            self.accepted_raw=raw.copy();self.accepted_target=np.array(target,copy=True)
            shift=self.frame.axes@(self.accepted_target-self.requested_target)
            self.mission_report.data['accepted_goals'][self.leg]=dict(
                goal_request_stamp_ns=int(self.goal_issued_ns),
                feedback_stamp_ns=min(self.stamp(m) for m in msg.markers if m.action==0 and m.type==2 and m.ns=='goal'),
                requested_planner_xyz=self.requested_raw.tolist(),accepted_planner_xyz=raw.tolist(),
                requested_aligned_xyz=self.requested_target.tolist(),accepted_aligned_xyz=self.accepted_target.tolist(),
                adjustment_forward_left_up_m=shift.tolist(),feedback='Pinned SUPER goal/path markers')
            self.get_logger().info(f'SUPER accepted {self.leg} goal: planner xyz={raw.round(6).tolist()}; '
                f'adjustment forward/left/up={shift.round(6).tolist()}; arrival uses this confirmed goal')
            self.mission_report.save()
        except ValueError as exc:self.abort(str(exc))

    def startup_message(self):
        return ('DEMONSTRATION EXECUTE ENABLED; explicit /super_demo/start and current RC/POSCTL required'
                if self.execute else 'DEMONSTRATION CHECK ONLY; READY does not execute or validate a flight')

    def position_policy_message(self):
        return 'post-takeoff position uses vehicle-inclusive venue bounds; 0.10 m arrival detection only'

    def validate(self,ground=False):
        result=super().validate(ground)
        if self.mission_file.read_bytes()!=self.mission_bytes:raise ValueError('Mission definition changed')
        if not self.fresh('velocity',.25):raise ValueError('Missing/stale local velocity')
        if self.count_publishers('/mavros/local_position/velocity_local')!=1:
            raise ValueError('Need one local velocity source')
        if not self.execute and any(self.count_publishers(t) for t in NATIVE_CONTROL_TOPICS):
            raise ValueError('Native controller present during mission check')
        return result

    def change(self,phase):
        if self.plan_wait_started_at is not None and phase!='WAIT_PLAN':
            self.plan_wait_elapsed_s=self.planning_wait_seconds()
            self.plan_wait_started_at=None
        super().change(phase)
        if phase=='WAIT_PLAN':self.plan_wait_started_at=self.phase_at
        self.mission_report.data['planner_wait_elapsed_s']=self.planning_wait_seconds()
        if phase=='READY':
            from control_bridge_preview import quaternion_rpy
            self.frame=VenueFrame(self.reference.ground,quaternion_rpy(0,0,self.reference.yaw),
                self.mission,lambda:self.extended is not None and self.extended.landed_state==1)
            self.home=np.array(self.reference.ground)+np.array([0.,0.,self.reference.lift])
            self.far=self.home+self.frame.axes.T@np.array([self.distance,0.,0.])
        self.mission_report.event(phase,leg=self.leg)
        if phase=='RISE':self.stages_started=True
        elif phase=='HOLD' and self.stages_started:
            self.mission_report.completed(self.mission.PHASES[0])
        elif phase=='LANDING' and self.stages_started:
            self.mission_report.completed('AUTO_LAND')
        elif phase=='DONE' and self.stages_started:
            self.mission_report.completed('CONFIRM_LANDED_AND_DISARMED')
            self.mission_report.data['max_tracking_error_forward_left_up_m']=self.max_tracking_error.tolist()
            self.mission_report.finish()
        elif phase=='STOPPED':self.mission_report.finish(self.fault or 'Mission stopped')

    def planning_wait_seconds(self,now=None):
        now=time.monotonic() if now is None else now
        active=max(0.,now-self.plan_wait_started_at) if self.plan_wait_started_at is not None else 0.
        return self.plan_wait_elapsed_s+active

    def report(self):
        super().report()
        self.mission_report.data['planner_wait_elapsed_s']=self.planning_wait_seconds()
        if self.phase=='WAIT_PLAN' and self.follower is not None and self.follower.last is None:
            self.get_logger().info('WAIT_PLAN: holding position; waiting for the first verified '
                f'{self.leg} reference; waited {time.monotonic()-self.plan_wait_started_at:.1f} s; '
                'no planning deadline')

    def allowed_modes(self):
        return dict(super().allowed_modes(),FINAL_HOLD=('OFFBOARD',))

    def target_ok(self,position,label='Reference'):
        self.frame.check(position)

    def check_envelope(self,xyz,yaw,ground):
        super().check_envelope(xyz,yaw,ground)
        if self.frame is not None and self.phase=='FINAL_HOLD':self.target_ok(xyz,'Measured home hover')

    def tracking_ok(self,xyz,target):
        # FC poses describe a past instant. Compare with the position/velocity
        # command active at that instant, never with a newer outgoing target.
        stamp=self.stamp(self.local)
        sent=next((item for item in reversed(self.sent_references) if item[0]<=stamp),None)
        if sent is None:raise ValueError('No sent-reference evidence at the FC pose timestamp')
        ns,position,velocity=sent;dt=(stamp-ns)/1e9
        if not 0<=dt<=.25:raise ValueError('Sent reference is stale at the FC pose timestamp')
        expected=position+velocity*dt
        self.frame.check(xyz)
        error=np.abs(self.frame.axes@(np.asarray(xyz)-expected))
        self.max_tracking_error=np.maximum(self.max_tracking_error,error)
        self.mission_report.data['max_tracking_error_forward_left_up_m']=self.max_tracking_error.tolist()

    def make_follower(self):
        return VenueReference(self.frame,self.check_stage_endpoint,self.final_goal_reached,self.motion_policy)

    def process_reference(self,xyz):
        try:
            # PositionCommand is a live reference stream, not a work queue.
            # Preserve the newest received stamp and validate it normally;
            # do not spend the control cycle replaying superseded samples.
            if len(self.pending)>1 and self.fresh_ns(self.stamp(self.pending[-1])):
                discarded=len(self.pending)-1;latest=self.pending[-1]
                self.pending.clear();self.pending.append(latest)
                self.mission_report.data['superseded_command_samples']=self.mission_report.data.get('superseded_command_samples',0)+discarded
            super().process_reference(xyz)
            if (self.follower.phase=='TRACK' and self.follower.resting_at_endpoint()
                    and self.final_goal_reached(self.follower.last_plan,self.follower.last[1])):
                # Goal feedback can arrive after the last terminal command.
                self.follower.phase='END_HOLD';self.native_velocity=np.zeros(3)
                wait=self.mission_report.data.get('intermediate_rest_wait')
                if wait is not None:wait['waiting']=False
        finally:
            if self.follower is not None:
                evidence=dict(self.follower.motion_summary,
                    last_violation=self.follower.last_motion_violation,policy=self.motion_policy)
                self.mission_report.data['planner_motion_by_leg'][self.leg]=evidence
                if self.follower.last_continuity_failure is not None:
                    self.mission_report.data['reference_continuity_failure']=self.follower.last_continuity_failure

    def check_command_stream(self):
        if self.follower.phase=='TRACK' and self.follower.resting_at_endpoint():
            self.native_velocity=np.zeros(3)
            goal_known=self.accepted_target is not None
            waiting=not self.fresh_ns(self.follower.last[0]) or not goal_known
            self.mission_report.data['intermediate_rest_wait']=dict(
                leg=self.leg,waiting=waiting,timeout_s=None,
                accepted_goal_confirmed=goal_known,
                last_accepted_stamp_ns=int(self.follower.last[0]))
            if waiting and not getattr(self,'rest_wait_warned',False):
                reason='subsequent SUPER commands' if goal_known else 'SUPER accepted-goal feedback'
                self.get_logger().warning(f'Holding verified rest endpoint; waiting for {reason} '
                    'with no deadline. The navigation goal remains incomplete.')
            self.rest_wait_warned=waiting
            return
        self.rest_wait_warned=False
        wait=self.mission_report.data.get('intermediate_rest_wait')
        if wait is not None:wait['waiting']=False
        try:super().check_command_stream()
        except ValueError as exc:
            oldest=self.stamp(self.pending[0]) if self.pending else None
            newest=self.stamp(self.pending[-1]) if self.pending else None
            detail=dict(leg=self.leg,pending_commands=len(self.pending),
                oldest_pending_stamp_ns=oldest,newest_pending_stamp_ns=newest,
                last_accepted_stamp_ns=int(self.follower.last[0]),
                latest_pending_fresh=bool(newest and self.fresh_ns(newest)))
            self.mission_report.data['command_stream_failure']=detail
            raise ValueError(str(exc)+'; '+json.dumps(detail)) from exc

    def check_timer_gap(self,gap):
        data=self.mission_report.data
        data['callback_gap_advisories']=data.get('callback_gap_advisories',0)+1
        data['largest_callback_gap_s']=max(data.get('largest_callback_gap_s',0.),float(gap))
        self.get_logger().warning(f'Callback scheduling gap {gap:.6f}s; timing advisory only. '
            'Continuing input validity, committed-trajectory and venue checks.')

    def check_stage_endpoint(self,plan):
        endpoint,error,detail=self.endpoint_evidence(plan)
        try:self.target_ok(endpoint,'Committed mission-stage endpoint')
        except ValueError as exc:raise ValueError(str(exc)+'; '+detail) from exc
        if self.accepted_target is None:return False
        error=self.frame.axes@(endpoint-self.completion_target())
        return bool(np.all(np.abs(error)<=.1))

    def final_goal_reached(self,plan,position):
        error=self.frame.axes@(np.asarray(position)-self.completion_target())
        return self.check_stage_endpoint(plan) and bool(np.all(np.abs(error)<=.1))

    def accept_plan(self,plan,msg):
        final=self.check_stage_endpoint(plan);self.follower.add_plan(msg)
        endpoint,error,detail=self.endpoint_evidence(plan)
        kind=('WAIT_ACCEPTED_GOAL' if self.accepted_target is None else
              'FINAL_GOAL_CANDIDATE' if final else 'INTERMEDIATE_STOP')
        key=(self.leg,kind)
        if key not in self.endpoint_kinds:
            self.endpoint_kinds.add(key)
            self.plan_evidence.append(dict(leg=self.leg,kind=kind,requested_planner_xyz=self.requested_raw.tolist(),
                committed_planner_endpoint=plan.endpoint.tolist(),error_forward_left_up=error.tolist(),
                completion_target_aligned_xyz=self.completion_target().tolist()))
            self.get_logger().info('Mission committed endpoint '+kind+'; '+detail)

    def supported_command(self,flag):
        return flag in (1,2)

    def convert_command(self,msg):
        raw=np.array([getattr(msg.position,axis) for axis in ('x','y','z')])
        # Fixed desired heading makes the body-to-FC lever transform constant;
        # position and velocity use the same rigid rotation.
        p,q,_=self.mapping.converter(raw,self.goal_q,np.eye(6))
        return p-self.mapping.origin,q

    def reference_accepted(self,msg):
        self.native_velocity=self.goal_rotation@np.array([getattr(msg.velocity,a) for a in ('x','y','z')])
        speed=float(np.linalg.norm(self.native_velocity))
        acceleration=math.hypot(*(getattr(msg.acceleration,a) for a in ('x','y','z')))
        self.mission_report.data['peak_reference_acceleration_mps2']=max(
            self.mission_report.data['peak_reference_acceleration_mps2'],acceleration)
        summary=self.mission_report.data['motion_advisory_by_leg'].setdefault(self.leg,
            dict(speed_exceedances=0,acceleration_exceedances=0))
        speed_exceeded=speed>self.ADVISORY_SPEED+1e-9
        acceleration_exceeded=acceleration>self.ADVISORY_ACCELERATION+1e-9
        summary['speed_exceedances']+=int(speed_exceeded)
        summary['acceleration_exceedances']+=int(acceleration_exceeded)
        if speed_exceeded or acceleration_exceeded:
            summary['last_exceedance']=dict(stamp_ns=self.stamp(msg),speed_mps=speed,
                                             acceleration_mps2=acceleration)
        if (self.motion_policy=='observe' and (speed_exceeded or acceleration_exceeded)
                and self.leg not in self.motion_warned_legs):
            self.motion_warned_legs.add(self.leg)
            self.get_logger().warning('Mission motion advisories exceeded; warning only: '
                f'speed={speed:.6f} m/s (advisory {self.ADVISORY_SPEED:g}), '
                f'acceleration={acceleration:.6f} m/s^2 (advisory {self.ADVISORY_ACCELERATION:g}); '
                'position, polynomial agreement, continuity and freshness checks remain active')

    def request_leg(self,leg,target):
        if self.goal_count>=2:raise ValueError('Demonstration permits exactly one outbound and one return goal')
        if self.goal_pub.get_subscription_count()<1 or self.count_publishers('/goal_pose')!=1:
            raise ValueError('Need this mission as sole goal writer and a SUPER subscriber')
        self.leg_start_target=(self.follower.last[1].copy() if self.follower is not None and self.follower.last is not None
                               else self.home.copy())
        self.previous_accepted_raw=self.accepted_raw
        self.accepted_raw=self.accepted_target=None
        self.leg=leg;self.requested_target=np.array(target,copy=True)
        self.requested_raw,self.goal_q=inverse_goal(self.mapping,self.raw_pose,self.requested_target)
        zero=self.mapping.converter(np.zeros(3),self.goal_q,np.eye(6))[0]
        self.goal_rotation=np.column_stack([self.mapping.converter(v,self.goal_q,np.eye(6))[0]-zero for v in np.eye(3)])
        self.follower=self.make_follower();self.pending.clear();self.poly_pending.clear()
        self.native_velocity=np.zeros(3);self.arrived_since=None
        msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
        for axis,value in zip(('x','y','z'),self.requested_raw):setattr(msg.pose.position,axis,float(value))
        for axis,value in zip(('x','y','z','w'),self.goal_q):setattr(msg.pose.orientation,axis,float(value))
        self.goal_issued_ns=self.stamp(msg);self.goal_count+=1;self.goal_pub.publish(msg)
        self.mission_report.data['planner_goals_sent']=self.goal_count
        self.mission_report.event('GOAL_'+leg.upper(),requested_aligned_xyz=self.requested_target.tolist(),
                                  requested_planner_xyz=self.requested_raw.tolist())
        self.change('WAIT_PLAN')

    def audit(self,request,response):
        response.success=False;response.message='Full demonstration requires simulation or supervised flight; no one-goal audit'
        return response

    def start(self,request,response):
        response=super().start(request,response)
        if response.success:response.message=f'Accepted one mission: {self.mission.hover_height:g} m rise, {self.distance:g} m outbound, return, 6 s hold, LAND'
        return response

    def arrived(self,xyz,target):
        velocity=self.velocity.twist.linear
        return (np.all(np.abs(self.frame.axes@(np.asarray(xyz)-target))<=.1)
                and math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)<=.1)

    def check_landing_feedback(self):
        if not self.fresh('local',.25):raise ValueError('Missing/stale local pose during demonstration landing')
        xyz,_=pose_values(self.local)
        point=self.frame.check(xyz)
        self.mission_report.data['landing_max_horizontal_error_m']=max(
            self.mission_report.data.get('landing_max_horizontal_error_m',0.),
            abs(float(point[0])),abs(float(point[1])))

    def tick(self):
        if self.phase=='LANDING':
            try:self.check_landing_feedback()
            except ValueError as exc:
                # PX4 continues landing; this observer cannot claim success or
                # resume native setpoints after landing feedback fails.
                self.abort(str(exc));return
        super().tick()

    def mission_tick(self,now,xyz,yaw):
        if self.phase=='RISE':
            u=min(1.,(now-self.armed_at)/self.reference.ramp_seconds)
            target=np.array(self.reference.ground)+np.array([0,0,self.reference.lift*(u**3*(10-15*u+6*u*u))])
            self.tracking_ok(xyz,target)
            self.native_velocity=np.array([0.,0.,self.reference.lift/self.reference.ramp_seconds*30*u*u*(1-u)**2])
            return False
        if self.phase=='HOLD':
            self.native_velocity=np.zeros(3);self.tracking_ok(xyz,self.home)
            self.publish(self.home,self.reference.yaw,native=self.execute)
            if self.arrived(xyz,self.home):
                if self.arrived_since is None:self.arrived_since=now
                if now-self.arrived_since>=2.:
                    self.mission_report.completed('STABILIZE_AT_HOME');self.request_leg('outbound',self.far)
            else:self.arrived_since=None
            return True
        if self.phase in ('WAIT_PLAN','TRACK','END_HOLD'):
            if not self.state.armed:raise ValueError('Disarmed during demonstration')
            if self.follower.phase!='END_HOLD':self.process_reference(xyz)
            if self.follower.last is None:
                target=getattr(self,'leg_start_target',None)
                if target is None:target=self.home if self.leg=='outbound' else self.far
            else:
                target=self.follower.last[1]
                desired='END_HOLD' if self.follower.phase=='END_HOLD' else 'TRACK'
                if self.phase!=desired:self.change(desired)
            if self.follower.phase=='END_HOLD':self.native_velocity=np.zeros(3)
            self.tracking_ok(xyz,target);self.publish(target,self.reference.yaw,native=self.execute)
            if self.follower.phase=='END_HOLD' and self.arrived(xyz,self.completion_target()):
                if self.arrived_since is None:self.arrived_since=now
                if now-self.arrived_since>=1.:
                    error=self.frame.axes@(np.asarray(xyz)-self.requested_target)
                    arrivals=self.mission_report.data.setdefault('goal_arrivals',{})
                    arrivals[self.leg]=dict(measured_aligned_xyz=np.asarray(xyz).tolist(),
                        requested_aligned_xyz=self.requested_target.tolist(),
                        accepted_aligned_xyz=self.completion_target().tolist(),
                        error_to_request_forward_left_up_m=error.tolist(),
                        nominal_position_reached=bool(np.all(np.abs(error)<=.1)))
                    self.mission_report.data['nominal_waypoints_reached']=all(
                        entry['nominal_position_reached'] for entry in arrivals.values())
                    if self.leg=='outbound':
                        self.mission_report.completed(self.mission.PHASES[2])
                        self.mission_report.completed('STABILIZE_AT_FAR_POINT')
                        self.request_leg('return',self.home)
                    else:
                        self.mission_report.completed('SUPER_RETURN_HOME')
                        self.return_hold_target=self.completion_target().copy()
                        self.mission_report.data['return_hold_target_aligned_xyz']=self.return_hold_target.tolist()
                        self.mission_report.data['return_home_offset_forward_left_up_m']=(self.frame.axes@(self.return_hold_target-self.home)).tolist()
                        self.final_hold_since=now;self.change('FINAL_HOLD')
            else:self.arrived_since=None
            return True
        if self.phase=='FINAL_HOLD':
            target=self.return_hold_target if self.return_hold_target is not None else self.home
            self.native_velocity=np.zeros(3);self.tracking_ok(xyz,target)
            self.publish(target,self.reference.yaw,native=self.execute)
            if not self.arrived(xyz,target):self.final_hold_since=now
            if now-self.final_hold_since>=6.:
                self.mission_report.data['return_hover_duration_s']=now-self.final_hold_since
                self.mission_report.data['planner_goals_sent']=self.goal_count
                self.mission_report.completed('HOLD_HOME_6S');self.land('Demonstration return hover complete')
            return True
        return False

    def publish(self,xyz,yaw,native=False):
        super().publish(xyz,yaw,native=False)
        if native:
            msg=PositionTarget();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='map'
            msg.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
            msg.type_mask=(PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|
                           PositionTarget.IGNORE_YAW_RATE)
            for axis,p,v in zip(('x','y','z'),xyz,self.native_velocity):
                setattr(msg.position,axis,float(p));setattr(msg.velocity,axis,float(v))
            msg.yaw=float(yaw);self.native_pub.publish(msg);self.last_target=list(xyz);self.sent+=1
            self.sent_references.append((self.stamp(msg),np.array(xyz,dtype=float,copy=True),
                                         np.array(self.native_velocity,dtype=float,copy=True)))
            self.mission_report.data['peak_reference_speed_mps']=max(
                self.mission_report.data['peak_reference_speed_mps'],float(np.linalg.norm(self.native_velocity)))


def arguments(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mission-file',type=Path,required=True)
    p.add_argument('--alignment-path',type=Path,required=True)
    p.add_argument('--planner-config',type=Path,required=True)
    p.add_argument('--report-json',type=Path,default=None)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--on-site-supervised',action='store_true')
    p.add_argument('--preparation-report',type=Path)
    p.add_argument('--motion-policy',choices=('strict','observe'),default='observe',
                   help='observe records derivative exceedances; strict retains exact 2/1 rejection')
    return p.parse_args(argv)


def main():
    args=arguments();report=args.report_json or default_report_path();node=None
    if report.exists():raise SystemExit('Refusing existing report: '+str(report))
    rclpy.init()
    try:
        node=DemoMissionControl(args.mission_file,report,args.execute,args.on_site_supervised,
                               args.alignment_path,args.planner_config,motion_policy=args.motion_policy)
        if args.preparation_report:
            node.mission_report.data['preparation']=json.loads(args.preparation_report.read_text())
            node.mission_report.save()
        began=time.monotonic()
        while rclpy.ok() and node.phase not in ('DONE','STOPPED'):
            rclpy.spin_once(node,timeout_sec=.05)
            if node.phase=='CHECKING' and time.monotonic()-began>60:
                node.abort('Demonstration readiness timed out; '+node.parameter_wait_detail())
            if not args.execute and node.phase=='READY':
                node.mission_report.data.update(result='NOT_RUN',preparation_result='SUCCESS',
                    error=None,physical_flight_verified=False)
                node.mission_report.save()
                print('PREPARATION: READY; MISSION: NOT RUN (check-only)',flush=True)
                print('REPORT:',report,flush=True)
                break
    except KeyboardInterrupt:
        if node is not None:node.abort('Mission interrupted')
    except Exception as exc:
        if node is not None:node.abort(type(exc).__name__+': '+str(exc))
        else:
            data=json.loads(report.read_text()) if report.exists() else dict(mode='check-only',physical_flight_verified=False)
            data.update(result='FAIL',error=type(exc).__name__+': '+str(exc))
            report.parent.mkdir(parents=True,exist_ok=True);report.write_text(json.dumps(data,indent=2)+'\n')
            print('RESULT: FAIL\nREPORT:',report,flush=True)
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if node is not None and (node.phase=='DONE' or not args.execute and node.phase=='READY') else 1


if __name__=='__main__':raise SystemExit(main())
