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
                                   validate_geometry,validate_session,wireframe,METHOD,freeze_scene,validate_frozen_scene)
from demo_mission import DemoMission
from test_demo_mission import fixture
from live_venue import SOURCE_TOPICS,digest,LiveVenue
from create_live_venue_profile import create
from super_demo import arguments,DemoPreparation
from super_ground_test import GroundTest
from calibrate_venue_rviz import FrozenCloud,rviz_config


class Tests(unittest.TestCase):
    def test_explicit_operation_box_retains_physical_clearances_and_readiness_gates(self):
        from rviz_venue_calibration import cap_selected_box
        _,obs,r,p=self.scene()
        geometry=cap_selected_box(selected_geometry(p(0,1.69,1),p(0,-1.317,1),obs,r,.5),2.,2.)
        config=apply_geometry(fixture(),geometry,obs,'hash')
        mission=DemoMission(config)
        self.assertFalse(mission.rehearsal_only)
        self.assertEqual((mission.width,mission.height,mission.hover_height),(2.,2.,1.5))
        mission.check([1.,.79,1.89])
        for point in ([1.,.81,1.5],[1.,-.81,1.5],[1.,0.,1.91]):
            with self.assertRaises(ValueError):mission.check(point)
        config['vehicle']['centre_to_top_m']=None
        with self.assertRaisesRegex(ValueError,'Missing/invalid'):DemoMission(config)
        args=arguments(['--mission-file','mission.json','--calibrate-venue','--venue-width','2','--venue-ceiling','2'])
        self.assertFalse(args.execute)
        self.assertEqual((args.venue_width,args.venue_ceiling),(2.,2.))
        with self.assertRaises(SystemExit):arguments(['--mission-file','mission.json','--venue-width','2'])

    def test_ceiling_cap_preserves_body_clearance_and_rejects_physical_reuse(self):
        from rviz_venue_calibration import cap_rehearsal_ceiling
        _,obs,r,p=self.scene()
        selected=selected_geometry(p(0,1.4,1),p(0,-1.4,1),obs,r,.5)
        capped=cap_rehearsal_ceiling(selected,2.)
        config=DemoMission.ground_rehearsal(apply_geometry(fixture(),capped,obs,'hash'),1.5)
        mission=DemoMission(config)
        self.assertEqual(mission.height,2.)
        self.assertEqual(selected['floor_height_limits_m'],[0.,2.8])
        top=mission.height-mission.hull['top']
        mission.check([1.,0.,top])
        with self.assertRaises(ValueError):mission.check([1.,0.,top+.01])
        vertices,_=wireframe(capped,obs,r)
        local=(axes(obs['ground_fc_yaw_rad'])@(r@vertices.T-np.array(obs['origin'])[:,None]-
                np.array(obs['ground_fc_xyz'])[:,None])).T
        self.assertAlmostEqual(local[:,2].max()+.085,2.)
        with self.assertRaisesRegex(ValueError,'not a physical'):
            validate_geometry(apply_geometry(fixture(),capped,obs,'hash'))
        config['venue']['height_m']=2.8
        with self.assertRaisesRegex(ValueError,'changed'):validate_geometry(config)
        for height in (0,-1,3.,float('nan'),True,None):
            with self.assertRaises(ValueError):cap_rehearsal_ceiling(selected,height)

    def test_rehearsal_cap_shrinks_preview_and_planner_geometry_without_expanding_survey(self):
        from rviz_venue_calibration import cap_rehearsal_width
        _,obs,r,p=self.scene()
        for left,right,expected in ((1.69,-1.317,[-1.,1.]),(.7,-1.6,[-1.,.7])):
            selected=selected_geometry(p(0,left,1),p(0,right,1),obs,r,.5)
            capped=cap_rehearsal_width(selected,2.)
            np.testing.assert_allclose(capped['lateral_limits_m'],expected)
            np.testing.assert_allclose(selected['lateral_limits_m'],[right,left])
            config=DemoMission.ground_rehearsal(apply_geometry(fixture(),capped,obs,'hash'))
            mission=DemoMission(config)
            self.assertAlmostEqual(mission.width,expected[1]-expected[0])
            vertices,_=wireframe(capped,obs,r)
            local=(axes(obs['ground_fc_yaw_rad'])@(r@vertices.T-np.array(obs['origin'])[:,None]-
                    np.array(obs['ground_fc_xyz'])[:,None])).T
            np.testing.assert_allclose([local[:,1].min(),local[:,1].max()],expected)
            with self.assertRaises(ValueError):mission.check([1.,expected[1],.5])
            physical=apply_geometry(fixture(),capped,obs,'hash')
            with self.assertRaisesRegex(ValueError,'not a physical'):validate_geometry(physical)
            changed=deepcopy(config);changed['venue_calibration']['geometry']['lateral_limits_m'][1]+=.1
            with self.assertRaisesRegex(ValueError,'Invalid calibrated'):validate_geometry(changed)
        for width in (0,-2,float('nan'),float('inf'),True,None):
            with self.assertRaises(ValueError):cap_rehearsal_width(selected,width)

    def test_frozen_scene_preserves_selected_frame_and_raw_aligned_rotation_pair(self):
        from aligned_pose_preview import rotation,multiply
        from control_bridge_preview import quaternion_rpy
        from venue_geometry import yaw
        from demo_venue import halfspaces
        from super_demo_control import VenueFrame
        world=quaternion_rpy(.2,-.15,.8);r=rotation(world)
        raw_q=quaternion_rpy(.1,.2,-.3);lever=np.array([.04,-.03,.11])
        def convert(p,q,c):return r@(p+rotation(q)@lever),multiply(world,q),c
        raw_p=np.array([1.,2.,-3.]);origin=np.array([.5,-.2,1.])
        ground,aligned_q,_=convert(raw_p,raw_q,np.eye(6));ground-=origin
        heading=yaw(aligned_q)+.03  # FC yaw differs slightly from external vision.
        obs=dict(ground_fc_xyz=ground.tolist(),ground_fc_yaw_rad=heading,origin=origin.tolist(),
                 raw_position=raw_p.tolist(),raw_quaternion=raw_q.tolist(),source_ids={'raw':'one'})
        def point(lateral):return r.T@(origin+ground+axes(heading).T@np.array([0,lateral,.5]))
        g=selected_geometry(point(1.69),point(-1.317),obs,r,.5)
        config=DemoMission.ground_rehearsal(apply_geometry(fixture(),g,obs,'hash'))
        scene=dict(raw_position=raw_p.tolist(),raw_quaternion=raw_q.tolist(),
                   aligned_position=(ground+[.02,0,0]).tolist(),aligned_quaternion=aligned_q.tolist(),
                   origin=origin.tolist(),alignment_sha256='hash')
        current=deepcopy(obs);current['ground_fc_xyz'][0]+=.02
        frozen=freeze_scene(config,scene,current,convert);validate_frozen_scene(config,frozen)
        np.testing.assert_allclose(frozen['aligned_position'],ground,atol=1e-12)
        p,q,_=convert(np.array(frozen['raw_position']),np.array(frozen['raw_quaternion']),np.eye(6))
        np.testing.assert_allclose(p-origin,ground,atol=1e-12)
        np.testing.assert_allclose(q,frozen['aligned_quaternion'],atol=1e-12)
        self.assertAlmostEqual(yaw(frozen['local_quaternion']),heading)
        mission=DemoMission(config);self.assertAlmostEqual(mission.width,3.007)
        planes,record=halfspaces(frozen,mission,convert)
        frame=VenueFrame(ground,frozen['local_quaternion'],mission)
        matrix=np.array(record['world_to_venue_matrix']);offset=np.array(record['world_to_venue_offset'])
        for lateral in (-1.317+mission.hull['right'],1.69-mission.hull['left']):
            raw=np.linalg.solve(matrix,np.array([1.,lateral,.5])-offset)
            fc=convert(raw,np.array(record['fixed_raw_goal_quaternion_xyzw']),np.eye(6))[0]-origin
            self.assertTrue(np.all(planes[:,:3]@raw+planes[:,3]<=1e-8))
            frame.check(fc)
        changed=deepcopy(frozen);changed['aligned_position'][0]+=.02
        with self.assertRaisesRegex(ValueError,'moved'):validate_frozen_scene(config,changed)
        with self.assertRaisesRegex(ValueError,'physical mission'):DemoMission.physical_height(config,1.5)
        current['source_ids']={}
        with self.assertRaisesRegex(ValueError,'another localization'):freeze_scene(config,scene,current,convert)

    def test_display_freezes_until_refresh_and_opens_with_point_tool(self):
        from std_msgs.msg import Header
        from sensor_msgs_py import point_cloud2
        cloud=FrozenCloud()
        def message(ns,x):
            header=Header();header.frame_id='camera_init'
            header.stamp.sec=ns//1_000_000_000;header.stamp.nanosec=ns%1_000_000_000
            return point_cloud2.create_cloud_xyz32(header,[[x,1.,2.]])
        cloud.feed(message(1_000_000_000,1.),1_010_000_000,0.)
        cloud.feed(message(2_100_000_000,2.),2_110_000_000,1.1)
        original=bytes(cloud.message.data);self.assertEqual(cloud.message.width,2)
        cloud.feed(message(3_200_000_000,100.),3_210_000_000,2.2)
        self.assertEqual(bytes(cloud.message.data),original)
        cloud.reset();self.assertIsNone(cloud.message)
        cloud.feed(message(4_000_000_000,4.),5_000_000_000,3.)
        self.assertEqual(cloud.parts,[])  # Old sensor samples cannot populate a new snapshot.
        cloud.feed(message(5_000_000_000,5.),5_010_000_000,4.)
        cloud.feed(message(6_100_000_000,6.),6_110_000_000,5.1)
        self.assertNotEqual(bytes(cloud.message.data),original)
        view=rviz_config([3.,-2.,-8.])['Visualization Manager']
        self.assertEqual(view['Tools'][0]['Class'],'rviz_default_plugins/PublishPoint')
        self.assertFalse(view['Tools'][0]['Single click'])
        self.assertEqual(view['Views']['Current']['Focal Point'],dict(X=3.,Y=-2.,Z=-8.))
        self.assertNotIn('/cloud_registered',str(view))

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

    def test_selected_sides_are_exact_including_wide_asymmetric_venues(self):
        _,obs,r,p=self.scene()
        for left,right,width in ((1.,-1.6,2.6),(1.6,-1.6,3.2),(.4,-5.,5.4)):
            g=selected_geometry(p(0,left,1),p(0,right,1),obs,r,.5)
            self.assertAlmostEqual(g['width_m'],width)
            original=fixture();c=apply_geometry(original,g,obs,'hash');m=DemoMission(c)
            self.assertEqual(m.width,g['width_m']);self.assertNotIn('venue_calibration',original)
            np.testing.assert_allclose(g['lateral_limits_m'],[right,left],atol=1e-12,rtol=0)
            self.assertAlmostEqual(m.left,left)
            self.assertAlmostEqual(m.width-m.left,-right)
            with self.assertRaises(ValueError):m.check([0.,left,1.5])
            with self.assertRaises(ValueError):m.check([0.,right,1.5])
            tampered=deepcopy(c);tampered['venue']['width_left_m']+=.01
            with self.assertRaisesRegex(ValueError,'changed'):DemoMission(tampered)
            tampered=deepcopy(c);tampered['venue_calibration']['geometry']['lateral_limits_m'][0]+=.01
            with self.assertRaisesRegex(ValueError,'Invalid calibrated'):DemoMission(tampered)

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

    def test_physical_hook_passes_box_options_before_confirmation_and_keeps_source(self):
        from rviz_venue_calibration import cap_selected_box
        args=arguments(['--mission-file','mission.json','--calibrate-venue','--rear-offset','.5',
                        '--venue-width','2','--venue-ceiling','2','--execute','--on-site-supervised'])
        _,obs,r,p=self.scene()
        with tempfile.TemporaryDirectory() as d:
            runner=object.__new__(DemoPreparation);runner.args=args;args.scripts_dir=Path(d)
            runner.folder=Path(d);runner.path=Path(d)/'prep.json';runner.report={}
            runner.calibration_source=fixture();original=deepcopy(runner.calibration_source)
            def calibrate(config,alignment,folder,rear,*,venue_width,venue_ceiling):
                self.assertEqual((venue_width,venue_ceiling),(2.,2.))
                geometry=selected_geometry(p(0,1.69,1),p(0,-1.317,1),obs,r,rear)
                return apply_geometry(config,cap_selected_box(geometry,venue_width,venue_ceiling),obs,'hash')
            with patch('calibrate_venue_rviz.calibrate',side_effect=calibrate):runner.after_vision_ready()
            self.assertEqual(runner.calibration_source,original)
            mission=DemoMission.load(args.mission_file)
            self.assertEqual((mission.width,mission.height,mission.hover_height),(2.,2.,1.5))
            self.assertFalse(mission.rehearsal_only)

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
