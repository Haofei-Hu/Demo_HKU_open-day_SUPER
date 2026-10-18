"""A live planner and controller share one measured frame without moving it."""
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tests'),*(str(ROOT/'scripts'/p) for p in
              ('planning','flight','maintenance','localization','simulation'))]
from control_bridge_preview import load_converter,quaternion_rpy
from create_live_venue_profile import create
from demo_mission import DemoMission
from live_venue import LiveVenue,SOURCE_TOPICS
from super_demo_control import VenueFrame
from test_demo_mission import fixture


def write_reference(folder,ground=(.2,-.3,.1),heading=.7,ids=None,mission=None,alignment=None,source=None):
    folder=Path(folder)
    mission=mission or folder/'mission.json'
    if not mission.exists():mission.write_text(json.dumps(fixture()))
    alignment=alignment or folder/'alignment.py'
    if not alignment.exists():alignment.write_text('def convert(p,q,c):\n return p,q,c\n')
    source=source or folder/'planner.yaml'
    if not source.exists():source.write_text(yaml.safe_dump(dict(super_planner={},
        fsm=dict(click_height=-6.,click_goal_topic='/goal_pose',click_goal_en=True,cmd_topic='/planning/pos_cmd'),
        traj_opt=dict(boundary=dict(max_vel=2.,max_acc=1.,acceptance_max_vel=4.,acceptance_max_acc=2.)))))
    observation=dict(ground_fc_xyz=list(ground),ground_fc_yaw_rad=heading,origin=[0.,0.,0.],
        raw_position=list(ground),raw_quaternion=quaternion_rpy(.1,-.1,.4).tolist(),
        raw_stamp_ns=123,local_stamp_ns=125,source_ids=ids or {t:('01'*24) for t in SOURCE_TOPICS})
    record=folder/'venue_reference.json'
    with patch('create_live_venue_profile.capture',return_value=observation):
        profile=create(source,mission,alignment,record,None)
    return record,mission,alignment,profile


class Tests(unittest.TestCase):
    def test_live_goals_keep_the_captured_raw_heading_and_its_lever_transform(self):
        from geometry_msgs.msg import Pose
        from aligned_pose_preview import rotation
        from super_flight_control import inverse_goal
        pose=Pose();current=quaternion_rpy(0,0,1.2);fixed=quaternion_rpy(0,0,.4)
        for axis,value in zip(('x','y','z','w'),current):setattr(pose.orientation,axis,float(value))
        def converter(p,q,c):return np.asarray(p)+rotation(q)@np.array([.1,.02,.03]),q,c
        mapping=NS(failed=False,pairs=10,origin=np.zeros(3),converter=converter)
        target=np.array([1.,2.,1.5]);raw,q=inverse_goal(mapping,pose,target,fixed)
        np.testing.assert_allclose(q,fixed,atol=1e-12)
        np.testing.assert_allclose(converter(raw,q,np.eye(6))[0],target,atol=1e-12)
        current_raw,_=inverse_goal(mapping,pose,target)
        self.assertGreater(np.linalg.norm(raw-current_raw),.05)
        with self.assertRaisesRegex(ValueError,'fixed planner yaw'):
            inverse_goal(mapping,pose,target,quaternion_rpy(.1,0,.4))

    def test_physical_preparation_requests_live_capture_and_plain_rehearsal_does_not(self):
        from super_demo import DemoPreparation,arguments
        from super_ground_test import GroundTest
        from super_session import arguments as manager_args
        runner=DemoPreparation.__new__(DemoPreparation)
        runner.args=arguments(['--mission-file','measured.json'])
        with patch.object(GroundTest,'action') as action:
            runner.action('planner','--planner-profile','demo')
            action.assert_called_once_with('planner','--planner-profile','demo',
                '--venue-mission',Path('measured.json').resolve())
        parsed=manager_args(['planner','--planner-profile','demo','--venue-mission','measured.json'])
        self.assertEqual(parsed.venue_mission,Path('measured.json'))
        self.assertIsNone(manager_args(['planner','--planner-profile','demo']).venue_mission)
        with self.assertRaises(SystemExit):manager_args(['start','--venue-mission','measured.json'])

    def test_physical_controller_requires_live_reference_before_creating_ros_node(self):
        from super_demo_control import DemoMissionControl
        with tempfile.TemporaryDirectory() as folder:
            mission=Path(folder)/'mission.json';mission.write_text(json.dumps(fixture()))
            with self.assertRaisesRegex(ValueError,'shared live venue reference'):
                DemoMissionControl(mission,Path(folder)/'report.json')

    def test_only_six_planes_added_source_geometry_motion_and_cloud_inputs_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            paths=write_reference(folder);record,mission,alignment,profile=paths
            source=profile.with_name('planner.yaml');before=source.read_bytes()
            expected=yaml.safe_load(before);expected['super_planner']['venue_halfspaces']=np.array(
                json.loads(record.read_text())['halfspaces']).reshape(-1).tolist()
            self.assertEqual(yaml.safe_load(profile.read_bytes()),expected)
            reference=LiveVenue(*paths)
            self.assertFalse(reference.data['native_control_or_flight_requests'])
            self.assertEqual(reference.data['additional_boundary_inset_m'],0.)
            self.assertTrue(reference.data['vehicle_extents_applied_once'])
            self.assertEqual(source.read_bytes(),before)
            with self.assertRaisesRegex(ValueError,'existing live venue'):
                create(source,mission,alignment,record,None)

    def test_rotated_physical_frame_with_tilt_and_lever_matches_all_controller_faces(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);alignment=folder/'alignment.py'
            alignment.write_text('''import numpy as np
def convert(p,q,c):
    a=.31
    r=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
    x,y,z,w=q
    body=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return r@(p+body@np.array([.06,-.04,.09])),q,c
''')
            paths=write_reference(folder,alignment=alignment);reference=LiveVenue(*paths)
            mission=DemoMission.load(paths[1]);convert=load_converter(alignment)
            frame=VenueFrame(reference.ground,quaternion_rpy(0,0,reference.yaw),mission,lambda:False)
            geometry=reference.geometry;matrix=np.array(geometry['world_to_venue_matrix'])
            offset=np.array(geometry['world_to_venue_offset']);planes=np.array(reference.data['halfspaces'])
            lower=np.array(geometry['fc_centre_lower_forward_left_floor_m'])
            upper=np.array(geometry['fc_centre_upper_forward_left_floor_m']);centre=(lower+upper)/2
            points=list(np.random.default_rng(18).uniform(lower-.2,upper+.2,size=(100,3)))
            for axis in range(3):
                for face in (lower[axis],upper[axis]):
                    for delta in (-.00001,0.,.00001):
                        point=centre.copy();point[axis]=face+delta;points.append(point)
            for point in points:
                raw=np.linalg.solve(matrix,point-offset)
                aligned=convert(raw,reference.goal_q,np.eye(6))[0]-reference.origin
                np.testing.assert_allclose(frame.coordinates(aligned),point,atol=1e-12)
                admitted=bool(np.all(planes[:,:3]@raw+planes[:,3]<=1e-9))
                try:frame.check(aligned);control=True
                except ValueError:control=False
                self.assertEqual(admitted,control,point)

    def test_controller_binds_captured_fc_ground_and_yaw_instead_of_rebasing_at_ready(self):
        with tempfile.TemporaryDirectory() as folder:
            reference=LiveVenue(*write_reference(folder))
            hover=NS(ground=reference.ground+np.array([.01,-.02,.01]),yaw=reference.yaw+.01,lift=1.415)
            reference.bind(hover)
            np.testing.assert_array_equal(hover.ground,reference.ground)
            self.assertEqual(hover.yaw,reference.yaw);self.assertEqual(hover.lift,1.415)
            with self.assertRaisesRegex(ValueError,'ground offset'):
                reference.bind(NS(ground=reference.ground+[1.,0,0],yaw=reference.yaw))
            with self.assertRaisesRegex(ValueError,'Heading departure'):
                reference.bind(NS(ground=reference.ground,yaw=reference.yaw+math.radians(16)))

    def test_mismatched_files_modified_reference_and_replaced_source_are_rejected(self):
        for changed in ('mission','alignment','profile','record','source','origin'):
            with self.subTest(changed=changed),tempfile.TemporaryDirectory() as folder:
                paths=write_reference(folder);reference=LiveVenue(*paths)
                node=NS(get_publishers_info_by_topic=lambda topic:[NS(endpoint_gid=bytes.fromhex('01'*24))])
                mapping=NS(origin=np.zeros(3));reference.validate(node,mapping)
                if changed in ('mission','alignment','profile'):
                    path=paths[{'mission':1,'alignment':2,'profile':3}[changed]]
                    path.write_bytes(path.read_bytes()+b' ')
                    with self.assertRaisesRegex(ValueError,'file mismatch'):LiveVenue(*paths)
                elif changed=='record':
                    paths[0].write_bytes(paths[0].read_bytes()+b' ')
                    with self.assertRaisesRegex(ValueError,'reference changed'):reference.validate(node,mapping)
                elif changed=='source':
                    node.get_publishers_info_by_topic=lambda topic:[NS(endpoint_gid=bytes.fromhex('02'*24))]
                    with self.assertRaisesRegex(ValueError,'source changed'):reference.validate(node,mapping)
                else:
                    mapping.origin=np.ones(3)
                    with self.assertRaisesRegex(ValueError,'origin changed'):reference.validate(node,mapping)

    def test_missing_measured_geometry_stops_before_any_ros_capture(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);mission=folder/'mission.json'
            mission.write_bytes((ROOT/'configs/demonstration/open_day_mission.json').read_bytes())
            with patch('create_live_venue_profile.capture') as capture:
                with self.assertRaisesRegex(ValueError,'measured geometry'):
                    create(folder/'unused.yaml',mission,folder/'unused.py',folder/'reference.json',None)
                capture.assert_not_called()


if __name__=='__main__':unittest.main()
