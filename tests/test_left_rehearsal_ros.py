"""Fixed-heading left-facing goals use the existing five-round state machine."""
import math
import base64
import json
import multiprocessing
import os
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'scripts/simulation')]
from test_demo_accepted_goal_ros import Harness
from left_rehearsal import LeftMission,mission_fixture
from left_demo_control import LeftMissionControl
from super_demo_control import VenueFrame
from test_left_rehearsal import make_scene,yaw
from left_rehearsal import rotated_scene


class Tests(unittest.TestCase):
    def test_left_viewer_uses_domain_198_without_changing_parent_environment(self):
        from demo_mission_simulation import RehearsalViewer
        with tempfile.TemporaryDirectory() as directory,\
            patch.dict(os.environ,{'ROS_DOMAIN_ID':'198','ROS_LOCALHOST_ONLY':'1','DISPLAY':':1'}),\
            patch('demo_mission_simulation.shutil.which',return_value='rviz2'),\
            patch('demo_mission_simulation.subprocess.Popen') as spawn,\
            patch('demo_mission_simulation.os.killpg'):
            spawn.return_value.pid=987;spawn.return_value.poll.return_value=None
            spawn.return_value.wait.return_value=-15
            viewer=RehearsalViewer(Path(directory)/'report.json',domain_id=198)
            viewer.start(None)
            self.assertEqual(spawn.call_args.kwargs['env']['ROS_DOMAIN_ID'],'198')
            self.assertEqual(viewer.state['domain_id'],198)
            self.assertEqual(os.environ['ROS_DOMAIN_ID'],'198')
            viewer.close()

    def test_entry_uses_separate_domain_reports_and_never_creates_aircraft_session(self):
        import super_left_test
        import super_session
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source=make_scene(folder)
            before={p:p.read_bytes() for p in folder.iterdir()}
            calls=[]
            def rehearsal(mission,report,**kwargs):
                calls.append((mission,report,kwargs))
                report.write_text(json.dumps(dict(result='SUCCESS',physical_flight_verified=False)))
                return True
            with patch.object(sys,'argv',['super_left_test.py','--ros-env','--scene-file',str(source),
                    '--reports-dir',str(folder/'left_reports'),'--no-rviz']),\
                patch.object(super_left_test,'check_private_domain'),\
                patch('demo_mission_simulation.run',side_effect=rehearsal),\
                patch('qgc_startup.open_qgc',return_value={'status':'already_running'}) as qgc,\
                patch.object(super_session,'Session') as session:
                self.assertEqual(super_left_test.main(),0)
                session.assert_not_called();qgc.assert_called_once()
            self.assertEqual(len(calls),1);mission,report,options=calls[0]
            self.assertEqual(options['domain_id'],198);self.assertFalse(options['rviz'])
            self.assertEqual(options['motion_policy'],'observe')
            self.assertIs(options['controller_class'],LeftMissionControl)
            self.assertEqual(report.parent.parent,folder/'left_reports')
            self.assertEqual(LeftMission(json.loads(mission.read_text())).distance,4.8)
            scene=json.loads(options['scene_file'].read_text())
            self.assertEqual(scene['virtual_heading_rotation_deg'],90.)
            self.assertEqual(scene['left_motion_override']['after']['max_acc'],1.5)
            self.assertEqual(scene['left_motion_override']['after']['max_vel'],2.)
            for path,data in before.items():self.assertEqual(path.read_bytes(),data)

    def test_left_controller_and_reference_load_15_without_relaxing_default_reference(self):
        import rclpy
        from bounded_reference_preview import Reference
        from left_rehearsal import left_planner_profile
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source=make_scene(folder)
            scene=left_planner_profile(rotated_scene(source),folder/'report.json')
            mission=folder/'mission.json';mission.write_text(json.dumps(mission_fixture()))
            rclpy.init();node=None
            try:
                node=LeftMissionControl(mission,folder/'report.json',
                    alignment_path=scene['alignment_path'],planner_config=scene['planner_config'],simulation=True)
                self.assertEqual(node.PLANNER_LIMITS,(2.,1.5))
                self.assertEqual(node.mission.config['flight']['acceleration_limit_mps2'],1.5)
                node.frame=VenueFrame([0,0,0],[0,0,0,1],node.mission)
                follower=node.make_follower()
                self.assertEqual(follower.max_speed,2.)
                self.assertEqual(follower.max_acceleration,1.5)
                with self.assertRaisesRegex(ValueError,'Invalid reference motion limits'):
                    Reference(node.frame,2.,1.5)
            finally:
                if node is not None:node.destroy_node()
                if rclpy.ok():rclpy.shutdown()

    def test_ten_alternating_goals_are_left_of_original_heading_with_fixed_yaw(self):
        node=Harness();node.mission=LeftMission(mission_fixture())
        node.reference.yaw=math.pi/2
        node.goal_q=np.array([0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
        node.raw_pose.orientation.z=float(node.goal_q[2]);node.raw_pose.orientation.w=float(node.goal_q[3])
        node.frame=VenueFrame([0,0,0],node.goal_q,node.mission)
        node.home=np.array([0.,0.,.415]);node.far=node.home+node.frame.axes.T@np.array([4.8,0.,0.])
        node.distance=4.8;node.goal_count=0
        node.request_leg('outbound',node.far)
        np.testing.assert_allclose(node.far,[0.,4.8,.415],atol=1e-12)
        for number in range(1,6):
            for leg in ('outbound','return'):
                self.assertEqual(node.round_index,number);self.assertEqual(node.leg,leg)
                np.testing.assert_allclose(node.requested_target,node.far if leg=='outbound' else node.home)
                np.testing.assert_allclose(node.goal_q,[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
                node.now_ns+=2_000_000_000;node.feedback(node.requested_raw)
                node.follower.phase='END_HOLD';node.follower.last=(node.now_ns,node.accepted_target.copy(),node.goal_q)
                node.phase='END_HOLD';now=node.now_ns/1e9;node.arrived_since=now-1.1
                node.mission_tick(now,node.accepted_target.copy(),math.pi/2)
        self.assertEqual(node.goal_count,10);self.assertEqual(node.phase,'FINAL_HOLD')
        self.assertEqual(node.mission_report.data['round_trips_completed'],5)
        completed=[call.args[0] for call in node.mission_report.completed.call_args_list]
        self.assertIn('ROUND_05_SUPER_OUTBOUND_4_8M',completed)
        node.mission_tick(now+6.1,node.return_hold_target,math.pi/2)
        self.assertEqual(node.phase,'LAND_REQUEST')

    def test_controller_rejects_physical_context_before_loading_files_or_ros(self):
        with self.assertRaisesRegex(ValueError,'virtual rehearsal only'):
            LeftMissionControl('not-created.json','not-created-report.json',execute=True,on_site_supervised=True)

    def test_rotated_saved_scene_starts_live_backend_with_fixed_fc_centre(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import PointCloud2,PointField
        from demo_mission_simulation import backend
        from control_bridge_preview import Mapping,load_converter
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source=make_scene(folder);data=rotated_scene(source)
            cloud=PointCloud2();cloud.header.frame_id='world';cloud.width=cloud.height=1
            cloud.point_step=cloud.row_step=12
            cloud.fields=[PointField(name=a,offset=i*4,datatype=PointField.FLOAT32,count=1)
                          for i,a in enumerate(('x','y','z'))]
            cloud.data=list(struct.pack('<fff',1.,2.,3.))
            data['cloud_serialized_base64']=base64.b64encode(serialize_message(cloud)).decode()
            scene=folder/'left_scene.json';scene.write_text(json.dumps(data))
            parent,child=multiprocessing.get_context('spawn').Pipe()
            process=multiprocessing.get_context('spawn').Process(target=backend,args=(child,None,scene))
            process.start();node=None
            try:
                self.assertTrue(parent.poll(15));self.assertEqual(parent.recv(),'ready')
                rclpy.init();node=rclpy.create_node('left_scene_backend_test')
                poses={};raw={};aligned={}
                def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                subs=[node.create_subscription(PoseStamped,'/mavros/local_position/pose',
                        lambda msg:poses.update({stamp(msg):msg}),qos_profile_sensor_data),
                    node.create_subscription(Odometry,'/Odometry',
                        lambda msg:raw.update({stamp(msg):msg}),qos_profile_sensor_data),
                    node.create_subscription(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov',
                        lambda msg:aligned.update({stamp(msg):msg}),qos_profile_sensor_data)]
                began=time.monotonic()
                while not poses.keys()&raw.keys()&aligned.keys() and time.monotonic()-began<8:
                    rclpy.spin_once(node,timeout_sec=.02)
                common=poses.keys()&raw.keys()&aligned.keys();self.assertTrue(common)
                key=max(common);pose=poses[key].pose
                np.testing.assert_allclose([pose.position.x,pose.position.y,pose.position.z],data['aligned_position'])
                actual=[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
                self.assertAlmostEqual(yaw(actual),yaw(data['aligned_quaternion']))
                mapping=Mapping(load_converter(data['alignment_path']))
                mapping.pair(raw[key].pose.pose,aligned[key].pose.pose)
                np.testing.assert_allclose(mapping.origin,data['origin'],atol=1e-8)
                parent.send('report');self.assertTrue(parent.poll(3));ledger=parent.recv()
                self.assertFalse(ledger['armed']);self.assertEqual(ledger['goals'],[])
            finally:
                if node is not None:node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
                if process.is_alive():parent.send('stop')
                process.join(timeout=5)
                if process.is_alive():process.terminate();process.join(timeout=5)
                parent.close();child.close()
            self.assertEqual(process.exitcode,0)


if __name__=='__main__':unittest.main()
