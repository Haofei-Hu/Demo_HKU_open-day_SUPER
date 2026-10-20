"""Selected environmental boundaries, session binding and preparation sequencing."""
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch,Mock
import numpy as np
import yaml

from rviz_venue_calibration import (axes,world_rotation,selected_geometry,apply_geometry,
                                   validate_geometry,validate_session,wireframe,METHOD)
from demo_mission import DemoMission
from test_demo_mission import fixture
from live_venue import SOURCE_TOPICS,digest,LiveVenue
from create_live_venue_profile import create
from super_demo import arguments,DemoPreparation
from super_ground_test import GroundTest


class Tests(unittest.TestCase):
    def scene(self):
        from control_bridge_preview import quaternion_rpy
        from aligned_pose_preview import rotation
        r=rotation(quaternion_rpy(.21,-.17,.8))
        # Deliberately large body lever: wall points must not receive this shift.
        converter=lambda p,q,c:(r@(p+np.array([.3,-.2,.1])),q,c)
        obs=dict(ground_fc_xyz=[.1,-.2,.03],ground_fc_yaw_rad=.47,origin=[3.,-2.,-7.],
                 raw_quaternion=[0.,0.,0.,1.],raw_position=[1.,2.,3.],raw_stamp_ns=12,local_stamp_ns=13,
                 source_ids={t:'ab'*24 for t in SOURCE_TOPICS})
        def point(f,l,z):
            return r.T@(np.array(obs['ground_fc_xyz'])+obs['origin']+axes(.47).T@np.array([f,l,z-.085]))
        return converter,obs,r,point

    def test_tilted_world_different_pick_depth_and_height_exclude_body_lever(self):
        convert,obs,r,point=self.scene()
        np.testing.assert_allclose(world_rotation(convert,obs['raw_quaternion']),r,atol=1e-12)
        g=selected_geometry(point(2,1.,1.5),point(-.2,-1.8,.8),obs,r,.5)
        self.assertAlmostEqual(g['left_offset_m'],1.);self.assertAlmostEqual(g['width_m'],2.8)
        vertices,edges=wireframe(g,obs,r)
        local=(axes(.47)@(r@vertices.T-np.array(obs['origin'])[:,None]-np.array(obs['ground_fc_xyz'])[:,None])).T
        np.testing.assert_allclose(local.min(axis=0),[-.5,-1.8,-.085],atol=1e-12)
        np.testing.assert_allclose(local.max(axis=0),[5.3,1.,2.715],atol=1e-12);self.assertEqual(len(edges),12)

    def test_narrower_selection_is_not_widened_and_wider_selection_caps_inside_markers(self):
        _,obs,r,p=self.scene()
        for left,right,width in ((1.,-1.6,2.6),(1.6,-1.6,2.8)):
            g=selected_geometry(p(0,left,1),p(0,right,1),obs,r,.5)
            self.assertAlmostEqual(g['width_m'],width)
            original=fixture();c=apply_geometry(original,g,obs,'hash');m=DemoMission(c)
            self.assertEqual(m.width,g['width_m']);self.assertNotIn('venue_calibration',original)
            self.assertGreaterEqual(g['lateral_limits_m'][0],right-1e-12)
            self.assertLessEqual(g['lateral_limits_m'][1],left+1e-12)
            with self.assertRaises(ValueError):m.check([0.,left,1.5])
            with self.assertRaises(ValueError):m.check([0.,right,1.5])
            tampered=deepcopy(c);tampered['venue']['width_left_m']+=.01
            with self.assertRaisesRegex(ValueError,'changed'):DemoMission(tampered)

    def test_reversed_outside_nonfinite_and_uncalibrated_width_rejected(self):
        _,obs,r,p=self.scene()
        for l,rr in ((-1.,1.),(1.,.1),(float('nan'),-1.)):
            with self.assertRaises(ValueError):selected_geometry(p(0,l,0),p(0,rr,0),obs,r,.5)
        c=fixture();c['venue']['width_left_m']=2.6
        with self.assertRaisesRegex(ValueError,'Venue differs'):DemoMission(c)

    def test_session_change_refused_and_small_observation_changes_do_not_move_anchor(self):
        _,obs,r,p=self.scene();g=selected_geometry(p(0,1.4,0),p(0,-1.4,0),obs,r,.5)
        data=apply_geometry(fixture(),g,obs,'hash')['venue_calibration']
        current=deepcopy(obs);current['ground_fc_xyz'][0]+=.02
        self.assertEqual(validate_session(data,current,'hash'),obs)
        for field,value in [('source_ids',{}),('origin',[0,0,0]),('ground_fc_xyz',[2,2,2]),('ground_fc_yaw_rad',2.)]:
            current=deepcopy(obs);current[field]=value
            with self.assertRaises(ValueError):validate_session(data,current,'hash')
        with self.assertRaises(ValueError):validate_session(data,obs,'different alignment')

    def test_live_profile_keeps_confirmed_frame_and_rejects_another_session(self):
        _,obs,_,_=self.scene()
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d);alignment=folder/'alignment.py'
            alignment.write_text('import numpy as np\ndef convert(p,q,c):\n return p+np.array([.2,-.1,.05]),q,c\n')
            r=np.eye(3);ground=np.array(obs['ground_fc_xyz']);origin=np.array(obs['origin'])
            def p(l):return ground+origin+axes(obs['ground_fc_yaw_rad']).T@np.array([0,l,0])
            g=selected_geometry(p(1.4),p(-1.4),obs,r,.5)
            c=apply_geometry(fixture(),g,obs,digest(alignment));mission=folder/'mission.json';mission.write_text(json.dumps(c))
            source=folder/'planner.yaml';source.write_text('super_planner: {}\n')
            current=deepcopy(obs);current['ground_fc_xyz'][0]+=.02
            record=folder/'reference.json'
            with patch('create_live_venue_profile.capture',return_value=current):
                profile=create(source,mission,alignment,record,None)
            ref=LiveVenue(record,mission,alignment,profile)
            np.testing.assert_array_equal(ref.ground,obs['ground_fc_xyz'])
            current=deepcopy(obs);current['source_ids']={}
            with patch('create_live_venue_profile.capture',return_value=current):
                with self.assertRaisesRegex(ValueError,'another localization'):
                    create(source,mission,alignment,folder/'other.json',None)

    def test_cli_calibration_only_cannot_execute_and_hook_replaces_lateral_fields(self):
        opts=['--mission-file','mission.json','--calibrate-venue','--calibration-only','--rear-offset','.5']
        args=arguments(opts);self.assertFalse(args.execute);self.assertEqual(args.flight_height,1.5)
        with self.assertRaises(SystemExit):arguments(opts+['--execute'])
        with self.assertRaises(SystemExit):arguments(['--mission-file','mission.json','--calibration-only'])
        _,obs,r,p=self.scene();g=selected_geometry(p(0,1.,0),p(0,-1.8,0),obs,r,.5)
        with tempfile.TemporaryDirectory() as d:
            runner=object.__new__(DemoPreparation);runner.args=args;args.scripts_dir=Path(d)
            runner.folder=Path(d);runner.path=Path(d)/'prep.json';runner.report={}
            runner.calibration_source=fixture();original=deepcopy(runner.calibration_source)
            result=apply_geometry(original,g,obs,'hash')
            with patch('calibrate_venue_rviz.calibrate',return_value=result):runner.after_vision_ready()
            self.assertEqual(runner.calibration_source,original)
            saved=json.loads(args.mission_file.read_text());self.assertAlmostEqual(saved['takeoff']['distance_from_left_m'],1.)
            self.assertEqual(saved['flight']['height_fc_above_floor_m'],1.5)

    def test_selection_precedes_planner_and_calibration_only_never_launches_it(self):
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)
            for only in (True,False):
                runner=object.__new__(DemoPreparation)
                runner.args=arguments(['--mission-file','mission.json','--calibrate-venue'])
                runner.args.scripts_dir=folder
                runner.args.test_profile='forward-5m-2mps'
                runner.args.max_alignment_spread_deg=2.
                runner.session=Mock();runner.session.exists.return_value=False
                runner.session.config={}
                runner.folder=folder;runner.report={}
                (folder/'alignment.json').write_text('{}')
                calls=[]
                runner.action=lambda action,*extra:calls.append(action)
                runner.after_vision_ready=lambda:calls.append('select_and_confirm')
                with patch('pathlib.Path.is_file',return_value=True),patch(
                        'super_ground_test.alignment_quaternion',return_value=[0.,0.,0.,1.]):
                    GroundTest.prepare(runner,planner_profile='demo',prepare_planner=not only)
                self.assertEqual(calls,['start','align','vision','select_and_confirm']+
                                 ([] if only else ['planner']))

if __name__=='__main__':unittest.main()
