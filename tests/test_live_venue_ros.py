"""Private-domain grounded capture and physical check-only controller integration."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,TwistStamped,PointStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus
from mars_quadrotor_msgs.msg import PositionCommand,PolynomialTrajectory
from visualization_msgs.msg import MarkerArray
import yaml

from control_bridge_preview import quaternion_rpy
from first_hover_terminal import PARAMETERS
from test_demo_mission import fixture

ROOT=Path(__file__).resolve().parents[1]

CHILD=r'''
import json,sys,time
from pathlib import Path
import numpy as np
import rclpy
from super_session import GroundGate,NATIVE_CONTROL_TOPICS
from create_live_venue_profile import create
from super_demo_control import DemoMissionControl
folder=Path(sys.argv[1]);case=sys.argv[2]
if case=='rviz':
    from calibrate_venue_rviz import calibrate
    (folder/'calibrating').touch()
    calibrated=calibrate(json.loads((folder/'mission.json').read_text()),folder/'alignment.py',folder,.5)
    (folder/'mission.json').write_text(json.dumps(calibrated))
    assert calibrated['venue_calibration']['confirmed_by_operator']
    assert abs(calibrated['takeoff']['distance_from_left_m']-1.1)<1e-9
    assert calibrated['venue']['width_left_m']<=2.8
    assert (folder/'venue_calibration.json').exists()
gate=GroundGate()
try:
    profile=create(folder/'planner.yaml',folder/'mission.json',folder/'alignment.py',folder/'venue_reference.json',gate)
except ValueError as exc:
    assert case=='native' and 'Native writer present' in str(exc),str(exc)
    assert not (folder/'venue_reference.json').exists()
    print('PASS native writer blocked before profile creation',flush=True)
    sys.exit(0)
finally:gate.close()
assert case!='native'
(folder/'captured').touch()
rclpy.init();node=None
try:
    node=DemoMissionControl(folder/'mission.json',folder/'report.json',
        alignment_path=folder/'alignment.py',planner_config=profile,
        venue_reference=folder/'venue_reference.json',motion_policy='observe')
    began=time.monotonic()
    while time.monotonic()-began<12 and node.phase not in ('READY','STOPPED'):
        rclpy.spin_once(node,timeout_sec=.02)
        if case=='wrong_config' and node.live_config_error:break
    assert not node.started and node.sent==0 and node.goal_count==0
    assert node.native_pub is None
    assert not any(node.count_publishers(topic) for topic in NATIVE_CONTROL_TOPICS)
    if case=='wrong_config':
        assert node.phase=='CHECKING' and 'config_name differs' in node.live_config_error,(node.phase,node.live_config_error)
        print('PASS wrong live config_name blocks READY with no control output',flush=True)
    else:
        assert node.phase=='READY',(node.phase,node.fault)
        capture=json.loads((folder/'venue_reference.json').read_text())
        np.testing.assert_allclose(node.reference.ground,capture['ground_fc_xyz'],atol=0,rtol=0)
        assert node.reference.yaw==capture['ground_fc_yaw_rad']
        assert node.local.pose.position.x>node.reference.ground[0]+.015
        assert node.live_config_confirmed
        assert node.mission_report.data['mode']=='check-only'
        assert not node.mission_report.data['physical_flight_verified']
        print('PASS fresh live capture, planner config readback, immutable FC anchor and check-only READY',flush=True)
finally:
    if node is not None:node.destroy_node()
    rclpy.shutdown()
'''


def run(case):
    rclpy.init();node=Node('fsm_node');params=Node('param',namespace='/mavros')
    frame=Node('setpoint_position',namespace='/mavros');frame.declare_parameter('mav_frame','LOCAL_NED')
    for key,value in dict(PARAMETERS,MPC_XY_VEL_MAX=2.).items():params.declare_parameter(key,value)
    node.declare_parameter('config_name','wrong.yaml' if case=='wrong_config' else 'planner_live_venue.yaml')
    kinds={'/Odometry':Odometry,'/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
        '/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped,'/mavros/local_position/pose':PoseStamped,
        '/mavros/state':State,'/mavros/extended_state':ExtendedState,'/mavros/estimator_status':EstimatorStatus,
        '/mavros/local_position/velocity_local':TwistStamped,'/planning/pos_cmd':PositionCommand,
        '/planning_cmd/poly_traj':PolynomialTrajectory,'/visualization/goal':MarkerArray}
    publishers={topic:node.create_publisher(kind,topic,10) for topic,kind in kinds.items()}
    if case=='native':publishers['native']=node.create_publisher(PoseStamped,'/mavros/setpoint_position/local',10)
    with tempfile.TemporaryDirectory() as directory:
        folder=Path(directory)
        clicked={'sent':0,'ack':0,'preview':None,'future':None}
        if case=='rviz':
            from std_srvs.srv import Trigger
            import math
            click_pub=node.create_publisher(PointStamped,'/super_venue/clicked_point',10)
            confirm=node.create_client(Trigger,'/super_venue_calibration/confirm')
            def markers(msg):
                clicked['ack']=sum(m.type==2 and m.action==0 for m in msg.markers)
                if any(m.id==20 and m.action==0 for m in msg.markers) and clicked['preview'] is None:
                    clicked['preview']=time.monotonic()
            node.create_subscription(MarkerArray,'/super_debug/venue_calibration',markers,10)
            def choose():
                if not (folder/'calibrating').exists() or not node.count_subscribers('/super_venue/clicked_point'):return
                index=clicked['sent']
                if index<2 and clicked['ack']==index:
                    lateral=(1.1,-1.7)[index]
                    msg=PointStamped();msg.header.frame_id='camera_init';msg.header.stamp=node.get_clock().now().to_msg()
                    msg.point.x=.2-math.sin(.7)*lateral;msg.point.y=-.3+math.cos(.7)*lateral;msg.point.z=.8
                    click_pub.publish(msg);clicked['sent']+=1
                if (clicked['preview'] is not None and time.monotonic()-clicked['preview']>3
                        and clicked['future'] is None and confirm.service_is_ready()):
                    clicked['future']=confirm.call_async(Trigger.Request())
            node.create_timer(.1,choose)
        def telemetry():
            stamp=node.get_clock().now().to_msg();moved=(folder/'captured').exists()
            for topic,kind in kinds.items():
                if kind in (PositionCommand,PolynomialTrajectory,MarkerArray):continue
                msg=kind();msg.header.stamp=stamp
                if kind is State:msg.connected=True;msg.mode='AUTO.LOITER'
                elif kind is ExtendedState:msg.landed_state=1
                elif kind is EstimatorStatus:
                    msg.attitude_status_flag=msg.velocity_horiz_status_flag=msg.velocity_vert_status_flag=True
                    msg.pos_horiz_rel_status_flag=msg.pos_vert_abs_status_flag=True
                elif kind is TwistStamped:msg.header.frame_id='map'
                else:
                    msg.header.frame_id='camera_init' if kind is Odometry else ('map' if kind is PoseStamped else 'px4_enu_preview')
                    if kind is Odometry:msg.child_frame_id='body'
                    pose=msg.pose if kind is PoseStamped else msg.pose.pose
                    pose.position.x=.2+(.02 if moved else 0);pose.position.y=-.3;pose.position.z=.1
                    q=quaternion_rpy(0,0,.7+(.01 if moved else 0))
                    for axis,value in zip(('x','y','z','w'),q):setattr(pose.orientation,axis,float(value))
                publishers[topic].publish(msg)
        node.create_timer(.02,telemetry);executor=SingleThreadedExecutor()
        for item in (node,params,frame):executor.add_node(item)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        try:
            (folder/'mission.json').write_text(json.dumps(fixture()))
            (folder/'alignment.py').write_text('def convert(p,q,c):\n return p,q,c\n')
            config=dict(super_planner={},fsm=dict(click_height=-6.,click_goal_topic='/goal_pose',
                click_goal_en=True,cmd_topic='/planning/pos_cmd'),traj_opt=dict(boundary=dict(max_vel=2.,max_acc=1.)))
            (folder/'planner.yaml').write_text(yaml.safe_dump(config))
            result=subprocess.run([sys.executable,'-c',CHILD,str(folder),case],capture_output=True,text=True,timeout=30)
            assert result.returncode==0,result.stdout+'\n'+result.stderr
            print(result.stdout,flush=True)
        finally:
            executor.shutdown();thread.join(timeout=2)
            for item in (node,params,frame):item.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    for case in ('healthy','wrong_config','native','rviz'):run(case)
