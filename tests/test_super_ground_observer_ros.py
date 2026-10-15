"""Live DDS audit observation and failure guards; simulated aircraft only."""
from pathlib import Path
import sys
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus
from mavros_msgs.srv import SetMode,CommandBool
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/analysis')]
from super_ground_test import AuditObserver


def run(case):
    ctx=Context();rclpy.init(context=ctx)
    probe=Node('automatic_ground_fixture',context=ctx)
    kinds={'/Odometry':Odometry,'/mavros/state':State,'/mavros/extended_state':ExtendedState,
           '/mavros/estimator_status':EstimatorStatus,'/mavros/local_position/pose':PoseStamped,
           '/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
           '/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped}
    pubs={name:probe.create_publisher(kind,name,10) for name,kind in kinds.items()}
    status=probe.create_publisher(String,'/super_debug/super_flight_status',10)
    requested=[];flight=[];extra=None;last_status=-1.
    def audit(req,res):
        requested.append(time.monotonic())
        if case=='lost_ack':time.sleep(.8)
        res.success=True;res.message='Disarmed audit fixture';return res
    probe.create_service(Trigger,'/super_flight/audit',audit)
    def arm(req,res):flight.append('arm');res.success=False;return res
    def mode(req,res):flight.append('mode');res.mode_sent=False;return res
    probe.create_service(CommandBool,'/mavros/cmd/arming',arm)
    probe.create_service(SetMode,'/mavros/set_mode',mode)
    def publish():
        nonlocal extra,last_status
        now=time.monotonic();elapsed=now-requested[0] if requested else 0.
        if requested and elapsed>.5 and extra is None:
            if case=='duplicate':extra=probe.create_publisher(Odometry,'/Odometry',10)
            if case=='native':extra=probe.create_publisher(PoseStamped,'/mavros/setpoint_position/local',10)
        for topic,kind in kinds.items():
            if requested and elapsed>.5 and case=='vision_gap' and topic=='/mavros/vision_pose/pose_cov':continue
            msg=kind();msg.header.stamp=probe.get_clock().now().to_msg()
            if kind is State:msg.connected=True;msg.mode='AUTO.LOITER'
            elif kind is ExtendedState:msg.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
            elif kind is EstimatorStatus:
                msg.attitude_status_flag=msg.velocity_horiz_status_flag=msg.velocity_vert_status_flag=True
                msg.pos_horiz_rel_status_flag=msg.pos_vert_abs_status_flag=True
            elif kind is Odometry:
                msg.header.frame_id='camera_init';msg.child_frame_id='body';msg.pose.pose.orientation.w=1.
            elif kind is PoseStamped:msg.header.frame_id='map';msg.pose.orientation.w=1.
            else:msg.header.frame_id='px4_enu_preview';msg.pose.pose.orientation.w=1.
            pubs[topic].publish(msg)
        if now-last_status>.2:
            phase='READY' if not requested else ('AUDIT_TRACK' if elapsed<1. else 'AUDIT_END')
            if requested and case=='stopped' and elapsed>1.5:phase='STOPPED'
            status.publish(String(data=f'{phase}: accepted={0 if phase=="READY" else 217}; DEBUG ONLY; fault='+
                                  ('Injected fault' if phase=='STOPPED' else 'None')))
            last_status=now
    probe.create_timer(.02,publish)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(probe)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start();observer=None
    try:
        observer=AuditObserver();observer.wait_ready(timeout=10)
        if case=='lost_ack':observer.gate.READ_RESPONSE_BUDGET=.3
        if case=='healthy':
            result=observer.audit(seconds=20.)
            assert result['end_hold_s']>=6. and result['sampled_vision_chain_fresh']
            assert all(value['n']>=500 for value in result['topics'].values())
        else:
            try:observer.audit(seconds=20.)
            except RuntimeError as exc:
                expected={'vision_gap':('stale','gap'),'duplicate':('Multiple',),
                          'native':('Native control',),'stopped':('STOPPED',),'lost_ack':('response',)}[case]
                assert any(part in str(exc) for part in expected),str(exc)
            else:raise AssertionError('Invalid audit passed: '+case)
        assert len(requested)==1,(case,requested)
        assert not flight,flight
        print('PASS: automatic ground observer',case,'one audit, no mode/arming requests',flush=True)
    finally:
        if observer:observer.close()
        executor.shutdown();thread.join(timeout=2);probe.destroy_node();rclpy.shutdown(context=ctx)


if __name__=='__main__':
    for case in ('healthy','vision_gap','duplicate','native','stopped','lost_ack'):run(case)
