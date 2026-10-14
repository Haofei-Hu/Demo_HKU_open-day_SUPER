"""Read-only readiness against isolated simulated ROS telemetry/services."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, RCIn
from mavros_msgs.srv import SetMode, CommandBool
from rcl_interfaces.msg import SetParametersResult

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts/flight')]
from first_hover_terminal import PARAMETERS, NATIVE


def run_case(case):
    rclpy.init()
    params=Node('param',namespace='/mavros')
    for name,value in PARAMETERS.items():params.declare_parameter(name,value)
    frame=Node('setpoint_position',namespace='/mavros')
    frame.declare_parameter('mav_frame','LOCAL_NED')
    probe=Node('readiness_fixture')
    requests=[];native_messages=[];goals=[];writes=[]
    params.add_on_set_parameters_callback(lambda values:(writes.extend(values) or SetParametersResult(successful=True)))
    frame.add_on_set_parameters_callback(lambda values:(writes.extend(values) or SetParametersResult(successful=True)))
    kinds={
        'state':(State,'/mavros/state'),
        'extended':(ExtendedState,'/mavros/extended_state'),
        'estimator':(EstimatorStatus,'/mavros/estimator_status'),
        'local':(PoseStamped,'/mavros/local_position/pose'),
        'vision':(PoseWithCovarianceStamped,'/mavros/vision_pose/pose_cov'),
        'aligned':(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov'),
        'raw':(Odometry,'/Odometry')}
    if case!='remote':kinds['rc']=(RCIn,'/mavros/rc/in')
    pubs={name:probe.create_publisher(kind,topic,10) for name,(kind,topic) in kinds.items()}
    writer=probe.create_publisher(PoseStamped,NATIVE,10) if case=='native_writer' else None
    probe.create_subscription(PoseStamped,NATIVE,native_messages.append,10)
    probe.create_subscription(PoseStamped,'/goal_pose',goals.append,10)
    def command(request,response):
        requests.append(request)
        if hasattr(response,'mode_sent'):response.mode_sent=True
        else:response.success=True
        return response
    probe.create_service(SetMode,'/mavros/set_mode',command)
    probe.create_service(CommandBool,'/mavros/cmd/arming',command)

    def publish():
        stamp=probe.get_clock().now().to_msg()
        messages={
            'state':State(connected=True,armed=False,mode='AUTO.LOITER' if case=='remote' else 'POSCTL',manual_input=case!='remote'),
            'extended':ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND),
            'estimator':EstimatorStatus(attitude_status_flag=True,velocity_horiz_status_flag=True,
                velocity_vert_status_flag=True,pos_horiz_rel_status_flag=True,pos_vert_abs_status_flag=True),
            'rc':RCIn(channels=[1500]*16)}
        local=PoseStamped();local.header.frame_id='map';local.pose.orientation.w=1.
        vision=PoseWithCovarianceStamped();vision.header.frame_id='px4_enu_preview';vision.pose.pose=local.pose
        raw=Odometry();raw.header.frame_id='camera_init';raw.child_frame_id='body';raw.pose.pose=local.pose
        messages.update(local=local,vision=vision,aligned=vision,raw=raw)
        for name,publisher in pubs.items():
            msg=messages[name];msg.header.stamp=stamp;publisher.publish(msg)
    probe.create_timer(.02,publish)
    executor=SingleThreadedExecutor()
    for node in (params,frame,probe):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        program=(f'import sys,json,importlib.util; sys.path[:0]={[str(ROOT),str(ROOT/"scripts/flight")]!r}; '
            f'spec=importlib.util.spec_from_file_location("session",{str(ROOT/"scripts/maintenance/super_session.py")!r}); '
            'module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); '
            'gate=module.GroundGate(); '
            'report=gate.flight_check(); gate.close(); print(json.dumps(report))')
        result=subprocess.run([sys.executable,'-c',program],capture_output=True,text=True,timeout=25)
        assert result.returncode==0,(result.stdout,result.stderr)
        report=json.loads(result.stdout.strip().splitlines()[-1]);checks=report['checks']
        assert checks['required_parameters'] and checks['local_ned'] and checks['fresh_valid_localization']
        assert checks['fc_vision_pose_agreement'] and checks['single_localization_sources']
        if case=='remote':
            assert not checks['fresh_rc_with_seven_channels'] and not checks['posctl_with_manual_input']
            assert not checks['single_rc_source']
        else:
            assert checks['fresh_rc_with_seven_channels'] and checks['posctl_with_manual_input']
        assert checks['no_native_control_publishers']==(writer is None)
        assert not requests and not writes and not native_messages and not goals
        assert len(probe.get_publishers_info_by_topic(NATIVE))==int(writer is not None)
        print('PASS:',case,'read-only; checks=',checks,flush=True)
    finally:
        executor.shutdown();thread.join(timeout=3)
        for node in (probe,frame,params):node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    for case in ('remote','rc_present','native_writer'):run_case(case)
