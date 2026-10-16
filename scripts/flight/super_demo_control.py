#!/usr/bin/env python3
"""One complete venue-bounded demonstration; check-only unless explicitly enabled.

    2 m takeoff -> SUPER 4.5 m outbound -> SUPER return -> 6 s hold -> LAND

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

from demo_mission import DemoMission,MissionReport,default_report_path
from first_hover_terminal import PARAMETERS,pose_values
from super_flight_control import SuperFlightControl,inverse_goal
from bounded_reference_preview import TakeoffFrame,RollingGoalReference
from super_session import NATIVE_CONTROL_TOPICS


class VenueFrame(TakeoffFrame):
    def __init__(self,position,quaternion,mission,on_floor):
        super().__init__(position,quaternion,max_forward=mission.distance+.1)
        self.mission,self.on_floor=mission,on_floor

    def check(self,position):
        point=self.coordinates(position)
        self.mission.check(point,on_floor=self.on_floor())
        return point


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
        self.MAX_DISTANCE=self.mission.distance
        self.FRAME_FORWARD_LIMIT=self.mission.distance+.1
        self.mission_report=MissionReport(report_path,self.mission,'simulation' if simulation else
                                         ('flight' if execute else 'check-only'))
        self.velocity=None;self.leg=None;self.home=self.far=None
        self.arrived_since=self.final_hold_since=None
        self.native_velocity=np.zeros(3);self.goal_count=0;self.stages_started=False
        self.max_tracking_error=np.zeros(3)
        self.sent_references=deque(maxlen=512)
        self.endpoint_kinds=set();self.plan_evidence=[]
        self.mission_report.data['committed_endpoints']=self.plan_evidence
        self.mission_report.data['tracking_policy']='FC pose timestamp matched to previously sent position/velocity reference'
        self.mission_report.data['peak_reference_speed_mps']=0.
        self.mission_report.data['peak_reference_acceleration_mps2']=0.
        self.mission_report.data['motion_policy']=motion_policy
        self.mission_report.data['planner_motion_by_leg']={}
        self.mission_report.data['motion_advisories']=dict(speed_mps=self.ADVISORY_SPEED,
            acceleration_mps2=self.ADVISORY_ACCELERATION,action='warning_only',
            derivative_exceedance_alone_stops_motion=motion_policy=='strict')
        self.mission_report.data['motion_advisory_by_leg']={}
        super().__init__(execute=execute,hover_fc_height=2.,hold_seconds=6.,distance=self.mission.distance,
                         alignment_path=alignment_path,planner_config=planner_config)
        self.offboard_budget=90.
        self.create_subscription(TwistStamped,'/mavros/local_position/velocity_local',
                                 lambda msg:self.store('velocity',msg),qos_profile_sensor_data)
        self.get_logger().info(f'MISSION {config["mission_id"]}: 2 m rise, {self.distance:g} m outbound, '
                               'return, 6 s hold, LAND; no parameter writes.')

    def make_native_publisher(self):
        return self.create_publisher(PositionTarget,self.NATIVE_TOPIC,10)

    def startup_message(self):
        return ('DEMONSTRATION EXECUTE ENABLED; explicit /super_demo/start and current RC/POSCTL required'
                if self.execute else 'DEMONSTRATION CHECK ONLY; READY does not execute or validate a flight')

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
        super().change(phase)
        if phase=='READY':
            from control_bridge_preview import quaternion_rpy
            self.frame=VenueFrame(self.reference.ground,quaternion_rpy(0,0,self.reference.yaw),
                self.mission,lambda:self.extended is not None and self.extended.landed_state==1)
            self.home=np.array(self.reference.ground)+np.array([0.,0.,1.915])
            self.far=self.home+self.frame.axes.T@np.array([self.distance,0.,0.])
        self.mission_report.event(phase,leg=self.leg)
        if phase=='RISE':self.stages_started=True
        elif phase=='HOLD' and self.stages_started:
            self.mission_report.completed('TAKEOFF_TO_2M')
        elif phase=='LANDING' and self.stages_started:
            self.mission_report.completed('AUTO_LAND')
        elif phase=='DONE' and self.stages_started:
            self.mission_report.completed('CONFIRM_LANDED_AND_DISARMED')
            self.mission_report.data['max_tracking_error_forward_left_up_m']=self.max_tracking_error.tolist()
            self.mission_report.finish()
        elif phase=='STOPPED':self.mission_report.finish(self.fault or 'Mission stopped')

    def allowed_modes(self):
        return dict(super().allowed_modes(),FINAL_HOLD=('OFFBOARD',))

    def target_ok(self,position,label='Reference'):
        p=self.frame.check(position)
        if not (-.1<=p[0]<=self.distance+.1 and abs(p[1])<=.1 and abs(p[2]-2.)<=.1):
            raise ValueError(label+' outside demonstration corridor: '+str(p.round(5).tolist()))

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
        error=np.abs(self.frame.axes@(np.asarray(xyz)-expected))
        self.max_tracking_error=np.maximum(self.max_tracking_error,error)
        self.mission_report.data['max_tracking_error_forward_left_up_m']=self.max_tracking_error.tolist()
        if np.any(error>.1):
            raise ValueError('Time-aligned tracking error exceeds 0.10 m per axis: '+
                             str(error.round(6).tolist())+f'; FC stamp={stamp}, reference stamp={ns}')

    def make_follower(self):
        return RollingGoalReference(self.frame,self.check_stage_endpoint,self.final_goal_reached,self.motion_policy)

    def process_reference(self,xyz):
        try:super().process_reference(xyz)
        finally:
            if self.follower is not None:
                evidence=dict(self.follower.motion_summary,
                    last_violation=self.follower.last_motion_violation,policy=self.motion_policy)
                self.mission_report.data['planner_motion_by_leg'][self.leg]=evidence

    def check_stage_endpoint(self,plan):
        endpoint,error,detail=self.endpoint_evidence(plan)
        try:self.target_ok(endpoint,'Committed mission-stage endpoint')
        except ValueError as exc:raise ValueError(str(exc)+'; '+detail) from exc
        return bool(np.all(np.abs(error)<=.1))

    def final_goal_reached(self,plan,position):
        error=self.frame.axes@(np.asarray(position)-self.requested_target)
        return self.check_stage_endpoint(plan) and bool(np.all(np.abs(error)<=.1))

    def accept_plan(self,plan,msg):
        self.check_stage_endpoint(plan);self.follower.add_plan(msg)
        endpoint,error,detail=self.endpoint_evidence(plan)
        kind='FINAL_GOAL_CANDIDATE' if np.all(np.abs(error)<=.1) else 'INTERMEDIATE_STOP'
        key=(self.leg,kind)
        if key not in self.endpoint_kinds:
            self.endpoint_kinds.add(key)
            self.plan_evidence.append(dict(leg=self.leg,kind=kind,requested_planner_xyz=self.requested_raw.tolist(),
                committed_planner_endpoint=plan.endpoint.tolist(),error_forward_left_up=error.tolist()))
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
        if response.success:response.message=f'Accepted one mission: 2 m rise, {self.distance:g} m outbound, return, 6 s hold, LAND'
        return response

    def arrived(self,xyz,target):
        velocity=self.velocity.twist.linear
        return (np.all(np.abs(self.frame.axes@(np.asarray(xyz)-target))<=.1)
                and math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)<=.1)

    def check_landing_feedback(self):
        if not self.fresh('local',.25):raise ValueError('Missing/stale local pose during demonstration landing')
        xyz,_=pose_values(self.local)
        point=self.frame.check(xyz)
        if abs(point[0])>.1 or abs(point[1])>.1:
            raise ValueError('Landing departed home by over 0.10 m per horizontal axis')
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
        if self.started and self.armed_at is not None and now-self.armed_at>self.offboard_budget:
            raise ValueError('Complete mission exceeded 90-second budget')
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
                if now-self.phase_at>8.:raise ValueError('No verified mission reference within 8 s')
                target=self.home if self.leg=='outbound' else self.far
            else:
                target=self.follower.last[1]
                desired='END_HOLD' if self.follower.phase=='END_HOLD' else 'TRACK'
                if self.phase!=desired:self.change(desired)
            if self.follower.phase=='END_HOLD':self.native_velocity=np.zeros(3)
            self.tracking_ok(xyz,target);self.publish(target,self.reference.yaw,native=self.execute)
            if self.follower.phase=='END_HOLD' and self.arrived(xyz,self.requested_target):
                if self.arrived_since is None:self.arrived_since=now
                if now-self.arrived_since>=1.:
                    if self.leg=='outbound':
                        self.mission_report.completed(self.mission.PHASES[2])
                        self.mission_report.completed('STABILIZE_AT_FAR_POINT')
                        self.request_leg('return',self.home)
                    else:
                        self.mission_report.completed('SUPER_RETURN_HOME')
                        self.final_hold_since=now;self.change('FINAL_HOLD')
            else:self.arrived_since=None
            return True
        if self.phase=='FINAL_HOLD':
            self.native_velocity=np.zeros(3);self.tracking_ok(xyz,self.home)
            self.publish(self.home,self.reference.yaw,native=self.execute)
            if not self.arrived(xyz,self.home):self.final_hold_since=now
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
