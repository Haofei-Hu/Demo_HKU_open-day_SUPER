"""Isolated SUPER handover fixture. All telemetry/command services are simulated."""
import math
from pathlib import Path
import sys
import tempfile
import time

import numpy as np
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus,RCIn
from mavros_msgs.srv import SetMode,CommandBool
from std_srvs.srv import Trigger
from mars_quadrotor_msgs.msg import PositionCommand,PolynomialTrajectory

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts/planning'),str(ROOT/'scripts/flight')]
from first_hover_terminal import PARAMETERS,NATIVE,HoverReference
from super_flight_control import SuperFlightControl,inverse_goal
from control_bridge_preview import Mapping,quaternion_rpy
from bounded_reference_preview import Plan


def run(case):
    with tempfile.TemporaryDirectory() as directory:
        alignment=Path(directory)/'alignment.py'
        alignment.write_text('def convert(p,q,covariance):\n    return p,q,covariance\n')
        config=Path(directory)/'planner.yaml'
        cfg=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
        cfg['fsm']['click_height']=-6.
        cfg['traj_opt']['boundary'].update(max_vel=.25,max_acc=.45)
        config.write_text(yaml.safe_dump(cfg))
        rclpy.init()
        params=Node('param',namespace='/mavros')
        for key,value in PARAMETERS.items():params.declare_parameter(key,value)
        frame=Node('setpoint_position',namespace='/mavros')
        frame.declare_parameter('mav_frame','LOCAL_NED')
        probe=Node('super_handover_fixture')
        execute=case.startswith('flight') or case=='missing_rc'
        ground=np.array([.12,-.08,-.74]);yaw=.6
        settings=dict(mode='POSCTL' if execute else 'AUTO.LOITER',armed=False,pose=ground.copy(),
                      landed=ExtendedState.LANDED_STATE_ON_GROUND,plan=None,drop_commands=False,
                      restart=False,rc=case!='missing_rc',departure=0.)
        requests=[];native=[];debug=[];goals=[]
        kinds={
            'state':(State,'/mavros/state'), 'local':(PoseStamped,'/mavros/local_position/pose'),
            'vision':(PoseWithCovarianceStamped,'/mavros/vision_pose/pose_cov'),
            'aligned':(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov'),
            'raw':(Odometry,'/Odometry'), 'extended':(ExtendedState,'/mavros/extended_state'),
            'estimator':(EstimatorStatus,'/mavros/estimator_status'), 'rc':(RCIn,'/mavros/rc/in'),
            'command':(PositionCommand,'/planning/pos_cmd'), 'plan':(PolynomialTrajectory,'/planning_cmd/poly_traj')}
        pubs={name:probe.create_publisher(kind,topic,10) for name,(kind,topic) in kinds.items()}
        executor=SingleThreadedExecutor()
        for node in (params,frame,probe):executor.add_node(node)
        bridge=SuperFlightControl(execute=execute,alignment_path=alignment,planner_config=config)
        executor.add_node(bridge)

        telemetry_ticks=0
        def stamp(msg):msg.header.stamp=probe.get_clock().now().to_msg()
        def telemetry():
            nonlocal telemetry_ticks
            telemetry_ticks+=1
            pose=PoseStamped();stamp(pose);pose.header.frame_id='map'
            measured=settings['pose']+np.array([settings['departure'],0.,0.])
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,measured)
            q=quaternion_rpy(0.,0.,yaw)
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,q)
            raw=Odometry();raw.header.stamp=pose.header.stamp;raw.header.frame_id='camera_init';raw.child_frame_id='body'
            raw.pose.pose=pose.pose
            vision=PoseWithCovarianceStamped();vision.header.stamp=pose.header.stamp
            vision.header.frame_id='px4_enu_preview';vision.pose.pose=pose.pose
            aligned=PoseWithCovarianceStamped();aligned.header=vision.header;aligned.pose=vision.pose
            state=State(connected=True,armed=settings['armed'],mode=settings['mode'],manual_input=execute)
            extended=ExtendedState(landed_state=settings['landed'])
            estimator=EstimatorStatus(attitude_status_flag=True,velocity_horiz_status_flag=True,
                velocity_vert_status_flag=True,pos_horiz_rel_status_flag=True,pos_vert_abs_status_flag=True)
            messages=dict(state=state,local=pose,vision=vision,aligned=aligned,raw=raw,
                          extended=extended,estimator=estimator,rc=RCIn(channels=[1500]*16))
            for name,msg in messages.items():
                if name=='rc' and not settings['rc']:continue
                # Representative mixed telemetry rates: paired/vision 50 Hz,
                # local pose 25 Hz, status/estimator/landing 10 Hz.
                if name=='local' and telemetry_ticks%2:continue
                if name in ('state','extended','estimator') and telemetry_ticks%5:continue
                if name not in ('local','vision','aligned','raw'):stamp(msg)
                pubs[name].publish(msg)

        def make_plan(start,end,when):
            duration=4.
            msg=PolynomialTrajectory();stamp(msg);msg.header.frame_id='world'
            msg.type=3;msg.order_pos=7;msg.piece_num_pos=1;msg.start_wt_pos=float(when);msg.time_pos=[duration]
            for axis,name in enumerate(('coef_pos_x','coef_pos_y','coef_pos_z')):
                delta=end[axis]-start[axis]
                setattr(msg,name,[0.,0.,float(6*delta/duration**5),float(-15*delta/duration**4),
                                 float(10*delta/duration**3),0.,0.,float(start[axis])])
            return msg

        def goal_received(msg):
            goals.append(msg)
            end=np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            plan=make_plan(settings['pose'].copy(),end,probe.get_clock().now().nanoseconds/1e9+.1)
            settings['plan']=plan;pubs['plan'].publish(plan)

        def planning():
            poly=settings['plan']
            if poly is None or settings['drop_commands']:return
            now=probe.get_clock().now().nanoseconds/1e9
            if now<poly.start_wt_pos:return
            plan=Plan(poly)
            if settings['restart'] or now>plan.end+.06:
                # Same endpoint, fresh polynomial, jump back to original start.
                start=ground.copy()
                if execute:start[2]+=.915
                poly=make_plan(start,plan.endpoint,now)
                settings['plan']=poly;settings['restart']=False
                pubs['plan'].publish(poly);plan=Plan(poly)
            p,v,a=plan.evaluate(now-poly.start_wt_pos)
            msg=PositionCommand();stamp(msg);msg.header.frame_id='world';msg.trajectory_flag=1;msg.yaw=yaw
            msg.attitude.z=yaw
            for field,values in (('position',p),('velocity',v),('acceleration',a)):
                for axis,value in zip(('x','y','z'),values):setattr(getattr(msg,field),axis,float(value))
            pubs['command'].publish(msg)

        def mode(request,response):
            requests.append(request.custom_mode);settings['mode']=request.custom_mode;response.mode_sent=True
            if request.custom_mode=='AUTO.LAND':
                settings['armed']=False;settings['landed']=ExtendedState.LANDED_STATE_ON_GROUND
            return response
        def arm(request,response):
            requests.append('ARM' if request.value else 'DISARM')
            settings['armed']=request.value;settings['landed']=ExtendedState.LANDED_STATE_IN_AIR
            response.success=True;return response
        def target(msg):
            native.append(msg)
            if execute and settings['armed']:
                settings['pose']=np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        probe.create_service(SetMode,'/mavros/set_mode',mode)
        probe.create_service(CommandBool,'/mavros/cmd/arming',arm)
        probe.create_subscription(PoseStamped,NATIVE,target,10)
        probe.create_subscription(PoseStamped,bridge.DEBUG_TOPIC,debug.append,10)
        probe.create_subscription(PoseStamped,'/goal_pose',goal_received,10)
        probe.create_timer(.02,telemetry);probe.create_timer(.01,planning)
        start_client=probe.create_client(Trigger,'/super_flight/start')
        audit_client=probe.create_client(Trigger,'/super_flight/audit')

        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        def wait(predicate,seconds=12):
            end=time.monotonic()+seconds
            while not predicate():
                assert time.monotonic()<end,(case,bridge.phase,bridge.fault,requests)
                spin(.02)
        def call(client):
            wait(client.service_is_ready)
            future=client.call_async(Trigger.Request());wait(future.done)
            return future.result()
        try:
            if case=='missing_rc':
                spin(4.5)
                assert bridge.phase=='CHECKING' and not call(start_client).success
                assert not requests and not native and not goals
                print('PASS:',case,bridge.phase)
                return
            wait(lambda:bridge.phase=='READY')
            assert not requests and not native and not goals,'Initialization caused motion'
            if not execute:
                assert bridge.count_publishers(NATIVE)==0
                assert not call(start_client).success
                assert call(audit_client).success
            else:
                assert not call(audit_client).success
                assert call(start_client).success
            wait(lambda:bridge.phase in ('TRACK','AUDIT_TRACK'),seconds=24)
            assert len(goals)==1
            if case=='audit_restart':
                spin(1.8);settings['restart']=True
                wait(lambda:bridge.phase=='STOPPED')
                assert 'discontinuity' in bridge.fault
            elif case=='audit_config':
                config.write_text(config.read_text()+'\n# changed\n')
                wait(lambda:bridge.phase=='STOPPED')
                assert 'configuration changed' in bridge.fault
            elif case=='audit_source':
                probe.destroy_publisher(pubs['raw'])
                pubs['raw']=probe.create_publisher(Odometry,'/Odometry',10)
                wait(lambda:bridge.phase=='STOPPED')
                assert 'source' in bridge.fault
            elif case=='flight_pilot':
                settings['mode']='POSCTL'
                wait(lambda:bridge.phase=='STOPPED')
                assert requests==['OFFBOARD','ARM'],'Pilot takeover was overridden'
            elif case=='flight_loss':
                settings['drop_commands']=True
                wait(lambda:bridge.phase=='STOPPED')
                wait(lambda:requests[-1]=='AUTO.LAND')
                assert requests==['OFFBOARD','ARM','AUTO.LAND']
                assert 'expired' in bridge.fault
            elif case=='flight_tracking':
                settings['departure']=.16
                wait(lambda:bridge.phase=='STOPPED')
                wait(lambda:requests[-1]=='AUTO.LAND')
                assert requests==['OFFBOARD','ARM','AUTO.LAND']
            else:
                wait(lambda:bridge.phase=='END_HOLD' if execute else bridge.phase=='AUDIT_END')
                held=bridge.follower.last[1].copy()
                spin(.4)
                assert np.array_equal(held,bridge.follower.last[1]),'Restart altered terminal reference'
                if execute:
                    wait(lambda:bridge.phase=='DONE')
                    assert requests==['OFFBOARD','ARM','AUTO.LAND']
                else:
                    assert not requests and not native and bridge.count_publishers(NATIVE)==0
                    assert not call(audit_client).success
                    settings['armed']=True
                    wait(lambda:bridge.phase=='STOPPED')
            if bridge.phase=='STOPPED':
                spin(.1);count=len(native);count_debug=len(debug)
                settings['drop_commands']=False;settings['armed']=False
                spin(.3)
                assert len(native)==count and len(debug)==count_debug,'Fault resumed output'
                assert not call(start_client).success and not call(audit_client).success
            assert len(goals)==1,'Goal silently retried'
            print('PASS:',case,bridge.phase)
        finally:
            executor.shutdown();bridge.destroy_node();probe.destroy_node();frame.destroy_node();params.destroy_node()
            rclpy.shutdown()


def numerical_checks():
    # Verify inverse mounting/world geometry without identity-frame assumptions.
    angle=.7
    rotation=np.array([[math.cos(angle),-math.sin(angle),0.],[math.sin(angle),math.cos(angle),0.],[0.,0.,1.]])
    lever=np.array([.04,-.02,.06])
    mapping=Mapping(lambda p,q,c:(rotation@p+lever,q,c))
    mapping.origin=np.array([.3,-.4,.8]);mapping.pairs=10
    pose=PoseStamped().pose;pose.orientation.w=1.
    wanted=np.array([1.,2.,.7]);raw,_=inverse_goal(mapping,pose,wanted)
    assert np.allclose(rotation@raw+lever-mapping.origin,wanted)
    control=object.__new__(SuperFlightControl)
    control.frame=__import__('bounded_reference_preview').TakeoffFrame([0.,0.,0.],[0.,0.,0.,1.])
    try:control.tracking_ok([.101,0.,0.],[0.,0.,0.])
    except ValueError:pass
    else:raise AssertionError('Per-axis tracking guard missing')
    control.execute=True;control.hover_fc_height=1.;control.distance=.3
    control.reference=HoverReference([0.,0.,0.],0.,hover_fc_height=1.,hold_seconds=6.)
    control.phase='TRACK'
    control.check_envelope([.3,0.,.915],0.,False)
    try:control.check_envelope([.3,0.,1.025],0.,False)
    except ValueError:pass
    else:raise AssertionError('Physical corridor allowed stacked reference/tracking offsets')


if __name__=='__main__':
    numerical_checks()
    for case in ('audit_hold','audit_restart','audit_config','audit_source','missing_rc',
                 'flight_hold','flight_pilot','flight_loss','flight_tracking'):
        run(case)
    print('PASS: default no-flight audit; one-goal handover; terminal hold; restart/loss/config/RC/pilot guards.')
