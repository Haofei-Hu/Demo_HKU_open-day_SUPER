"""Speed observation with a separate telemetry process, as on the aircraft."""
import json
import multiprocessing
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/analysis')]


def producer(connection,overspeed):
    import rclpy
    from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State,ExtendedState,EstimatorStatus
    from mavros_msgs.srv import SetMode,CommandBool
    from mars_quadrotor_msgs.msg import PositionCommand
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init();node=rclpy.create_node('speed_observer_process_fixture')
    kinds={'/Odometry':Odometry,'/mavros/state':State,'/mavros/extended_state':ExtendedState,
           '/mavros/estimator_status':EstimatorStatus,'/mavros/local_position/pose':PoseStamped,
           '/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
           '/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped,'/planning/pos_cmd':PositionCommand}
    pubs={name:node.create_publisher(kind,name,10) for name,kind in kinds.items()}
    status=node.create_publisher(String,'/super_debug/super_flight_status',10)
    requests=[];flight=[];last_status=-1.
    def audit(req,res):
        requests.append(time.monotonic());res.success=True;res.message='Disarmed fixture';return res
    def arm(req,res):flight.append('ARM');res.success=False;return res
    def mode(req,res):flight.append('MODE');res.mode_sent=False;return res
    node.create_service(Trigger,'/super_flight/audit',audit)
    node.create_service(CommandBool,'/mavros/cmd/arming',arm)
    node.create_service(SetMode,'/mavros/set_mode',mode)
    def publish():
        nonlocal last_status
        now=time.monotonic()
        for topic,kind in kinds.items():
            msg=kind();msg.header.stamp=node.get_clock().now().to_msg()
            if kind is State:msg.connected=True;msg.mode='AUTO.LOITER'
            elif kind is ExtendedState:msg.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
            elif kind is EstimatorStatus:
                msg.attitude_status_flag=msg.velocity_horiz_status_flag=msg.velocity_vert_status_flag=True
                msg.pos_horiz_rel_status_flag=msg.pos_vert_abs_status_flag=True
            elif kind is Odometry:
                msg.header.frame_id='camera_init';msg.child_frame_id='body';msg.pose.pose.orientation.w=1.
            elif kind is PoseStamped:msg.header.frame_id='map';msg.pose.orientation.w=1.
            elif kind is PositionCommand:
                msg.header.frame_id='world';msg.trajectory_flag=1
                msg.velocity.x=2.01 if overspeed else 2.;msg.acceleration.x=1.
            else:msg.header.frame_id='px4_enu_preview';msg.pose.pose.orientation.w=1.
            pubs[topic].publish(msg)
        if now-last_status>.2:
            phase='READY' if not requests else ('AUDIT_TRACK' if now-requests[0]<1. else 'AUDIT_END')
            status.publish(String(data=f'{phase}: accepted=217; DEBUG ONLY; fault=None'))
            last_status=now
    node.create_timer(.02,publish);connection.send('ready')
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            if connection.poll():
                message=connection.recv()
                if message=='stop':break
                if message=='report':connection.send(dict(audit_requests=len(requests),flight_requests=flight))
    finally:node.destroy_node();rclpy.shutdown()


def run(overspeed):
    from super_ground_test import AuditObserver
    parent,child=multiprocessing.get_context('spawn').Pipe()
    process=multiprocessing.get_context('spawn').Process(target=producer,args=(child,overspeed))
    process.start();observer=None
    try:
        assert parent.poll(15) and parent.recv()=='ready'
        observer=AuditObserver(motion_limits=(2.,1.));observer.wait_ready(timeout=12.)
        if overspeed:
            try:observer.audit(seconds=20.)
            except RuntimeError as exc:assert 'Planner motion exceeds' in str(exc),str(exc)
            else:raise AssertionError('2.01 m/s reference passed the 2 m/s limit')
        else:
            result=observer.audit(seconds=20.)
            assert result['planner_motion']['peak_speed_mps']==2.
            assert result['planner_motion']['peak_acceleration_mps2']==1.
            assert result['sampled_vision_chain_fresh'] and result['end_hold_s']>=6.
        parent.send('report');assert parent.poll(5)
        calls=parent.recv();assert calls==dict(audit_requests=1,flight_requests=[]),calls
        print('PASS: separate-process planner motion observer', 'overspeed rejected' if overspeed else '2/1 peaks and hold',flush=True)
    except Exception:
        if observer is not None:print(json.dumps(observer.snapshot(),indent=2),flush=True)
        raise
    finally:
        if observer is not None:observer.close()
        if process.is_alive():parent.send('stop')
        process.join(timeout=5)
        if process.is_alive():process.terminate();process.join()
        parent.close();child.close()


if __name__=='__main__':
    run(False);run(True)
