"""Isolated DDS checks for one vertical goal and ground/readiness rejection."""
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from mavros_msgs.msg import State,ExtendedState
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/planning'))
from control_bridge_preview import load_converter,quaternion_rpy


def run(case):
    with tempfile.TemporaryDirectory() as folder:
        folder=Path(folder);alignment=folder/'aligned.py';config=folder/'config.yaml'
        alignment.write_text((ROOT/'scripts/localization/aligned_pose_preview.py').read_text()+
            '\nQ_WC=np.array([0.,0.,2**-.5,2**-.5])\nR_WC=rotation(Q_WC)\n'
            'AXES[:3,:3]=AXES[3:,3:]=R_WC\n')
        config.write_text('fsm:\n  click_height: -6.0\n  click_goal_topic: /goal_pose\n  click_goal_en: true\n'
                          'traj_opt:\n  boundary:\n    max_vel: 0.25\n    max_acc: 0.45\n')
        original=alignment.read_bytes(),config.read_bytes()
        converter=load_converter(alignment)
        raw_p=np.array([2.,.5,.8]);raw_q=quaternion_rpy(.14,-.09,.8)
        predicted_p,predicted_q,_=converter(raw_p,raw_q,np.eye(6))
        aligned_p=np.array([.2,-.1,.05]);origin=predicted_p-aligned_p
        rclpy.init();node=rclpy.create_node('height_goal_fixture')
        pubs={topic:node.create_publisher(kind,topic,10) for topic,kind in (
            ('/Odometry',Odometry),('/super_debug/aligned_pose_cov',PoseWithCovarianceStamped),
            ('/mavros/state',State),('/mavros/extended_state',ExtendedState),
            ('/super_debug/bounded_reference_status',String))}
        goals=[];sub=node.create_subscription(PoseStamped,'/goal_pose',goals.append,10)
        def telemetry():
            stamp=node.get_clock().now().to_msg()
            raw=Odometry();raw.header.stamp=stamp;raw.header.frame_id='camera_init';raw.child_frame_id='body'
            aligned=PoseWithCovarianceStamped();aligned.header.stamp=stamp;aligned.header.frame_id='px4_enu_preview'
            for pose,p,q in ((raw.pose.pose,raw_p,raw_q),(aligned.pose.pose,aligned_p,predicted_q)):
                pose.position.x,pose.position.y,pose.position.z=map(float,p)
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,q)
            state=State();state.header.stamp=stamp;state.connected=True;state.armed=case=='armed'
            ground=ExtendedState();ground.header.stamp=stamp;ground.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
            status=String();status.data=('END_HOLD:' if case=='held' else 'WAIT_TRAJECTORY:')+' DEBUG ONLY'
            for topic,msg in (('/Odometry',raw),('/super_debug/aligned_pose_cov',aligned),
                              ('/mavros/state',state),('/mavros/extended_state',ground),
                              ('/super_debug/bounded_reference_status',status)):pubs[topic].publish(msg)
        timer=node.create_timer(.02,telemetry)
        process=subprocess.Popen([sys.executable,str(ROOT/'scripts/planning/send_ground_height_goal.py'),
            '--height','1','--alignment-path',str(alignment),'--planner-config',str(config)],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        checked=False
        try:
            end=time.monotonic()+10
            while process.poll() is None:
                assert time.monotonic()<end,'Height-goal fixture timed out'
                rclpy.spin_once(node,timeout_sec=.01)
                publishers=(node.get_publisher_names_and_types_by_node('send_ground_height_goal','/')
                    if ('send_ground_height_goal','/') in node.get_node_names_and_namespaces() else [])
                if any(name=='/goal_pose' for name,_ in publishers):
                    checked=True
                    assert not any(name.startswith('/mavros/') for name,_ in publishers)
            for _ in range(20):rclpy.spin_once(node,timeout_sec=.01)
            output=process.communicate(timeout=1)[0]
            if case=='valid':
                assert process.returncode==0,output
                assert checked and len(goals)==1,(checked,len(goals),output)
                goal=goals[0];assert goal.header.frame_id=='world'
                p=np.array([getattr(goal.pose.position,a) for a in ('x','y','z')])
                q=np.array([getattr(goal.pose.orientation,a) for a in ('x','y','z','w')])
                point=converter(p,q,np.eye(6))[0]-origin
                np.testing.assert_allclose(point,aligned_p+[0,0,.915],atol=1e-8)
            else:
                assert process.returncode!=0 and not goals,output
                assert ('Arming observed' if case=='armed' else 'WAIT_TRAJECTORY') in output,output
            assert (alignment.read_bytes(),config.read_bytes())==original
        finally:
            if process.poll() is None:process.terminate();process.wait(timeout=3)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    for case in ('valid','armed','held'):run(case)
    print('PASS: one FC-centre vertical goal with geometry inversion, armed/held rejection, '
          'unchanged geometry/config and no native publishers.')
