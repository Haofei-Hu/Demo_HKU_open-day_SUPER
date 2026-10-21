"""Separate actual-position limits without expanding admissible references."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),*(str(ROOT/'scripts'/p) for p in
              ('planning','flight','maintenance','localization','simulation'))]
from demo_mission import DemoMission,TakeoffFloorPolicy,MissionReport
from rviz_venue_calibration import (selected_geometry,cap_selected_box,apply_geometry,
    separate_execution_geometry,validate_geometry,wireframe)
from venue_geometry import planes_for_frame
from super_demo_control import VenueFrame,DemoMissionControl
from control_bridge_preview import quaternion_rpy
from test_demo_mission import fixture


def configuration(split=True,limits=(-1.7,1.6)):
    obs=dict(ground_fc_xyz=[0.,0.,0.],ground_fc_yaw_rad=0.,origin=[0.,0.,0.])
    geometry=cap_selected_box(selected_geometry([0.,limits[1],1.],[0.,limits[0],1.],
                                               obs,np.eye(3),.5),2.,2.)
    if split:geometry=separate_execution_geometry(geometry)
    config=fixture()
    for side in ('front','rear','left','right'):config['vehicle']['centre_to_'+side+'_m']=.136
    config['vehicle']['centre_to_top_m']=.165
    config['flight']['height_fc_above_floor_m']=1.
    return apply_geometry(config,geometry,obs,'test-only')


class Tests(unittest.TestCase):
    def test_measured_side_and_ceiling_allowance_does_not_admit_references(self):
        mission=DemoMission(configuration())
        for point in ([2.,1.1,1.],[2.,-1.1,1.],[2.,0.,2.2]):
            with self.subTest(point=point):
                with self.assertRaisesRegex(ValueError,'outside venue'):mission.check(point)
                mission.check_execution(point)
                TakeoffFloorPolicy().check(mission,point,'TRACK',2)
        for point in ([2.,1.265,1.],[2.,-1.265,1.],[2.,0.,2.636]):
            with self.assertRaisesRegex(ValueError,'outside venue'):mission.check_execution(point)

    def test_front_incident_and_floor_are_not_relaxed(self):
        mission=DemoMission(configuration())
        for point in ([5.17474517897,.5880515,.8273956],[-.365,0.,1.],[0.,0.,.084]):
            with self.assertRaisesRegex(ValueError,'outside venue'):mission.check_execution(point)
        mission.check_execution([5.164,0.,1.])
        # The existing bounded initial ground-contact rule still applies.
        TakeoffFloorPolicy().check(mission,[0.,0.,.064],'RISE',2)
        with self.assertRaises(ValueError):TakeoffFloorPolicy().check(mission,[0.,0.,.064],'TRACK',2)

    def test_all_measured_flight_phases_use_outer_box(self):
        mission=DemoMission(configuration())
        for phase in ('RISE','HOLD','WAIT_PLAN','TRACK','END_HOLD','FINAL_HOLD','LANDING'):
            frame=VenueFrame((.3,-.4,.1),quaternion_rpy(0.,0.,.7),mission)
            position=frame.origin+frame.axes.T@np.array([2.,1.1,1.-.085])
            frame.check_measured(position,phase,2)
            with self.assertRaises(ValueError):frame.check(position)
        control=DemoMissionControl.__new__(DemoMissionControl)
        control.frame=frame;control.execute=True
        control.reference=NS(ground=frame.origin,yaw=frame.yaw)
        control.extended=NS(landed_state=2);control.mission_report=NS(data={})
        control.target_ok=lambda *a,**kw: self.fail('Measured feedback must not use the inner reference box')
        for phase in ('HOLD','WAIT_PLAN','TRACK','END_HOLD','FINAL_HOLD'):
            control.phase=phase
            control.check_envelope(position,frame.yaw,False)

    def test_planner_planes_byte_identical_and_same_rotated_frame(self):
        mission=DemoMission(configuration());old=DemoMission(configuration(False))
        yaw=.9;ground=np.array([.4,-.2,.6]);origin=np.array([1.,2.,3.])
        rotation=np.array([[1.,0.,0.],[0.,np.cos(.3),-np.sin(.3)],[0.,np.sin(.3),np.cos(.3)]])
        def convert(p,q,c):return rotation@p+np.array([.1,.2,.3]),q,c
        q=quaternion_rpy(0.,0.,.2)
        planes,record=planes_for_frame(mission,convert,q,origin,ground,yaw)
        previous,_=planes_for_frame(old,convert,q,origin,ground,yaw)
        np.testing.assert_array_equal(planes,previous)
        frame=VenueFrame(ground,quaternion_rpy(0.,0.,yaw),mission)
        matrix=np.asarray(record['world_to_venue_matrix']);offset=np.asarray(record['world_to_venue_offset'])
        for relative in ([2.,.8,1.],[2.,1.1,1.],[2.,0.,2.2],[5.175,0.,1.]):
            raw=np.linalg.solve(matrix,np.asarray(relative)-offset)
            aligned=convert(raw,q,np.eye(6))[0]-origin
            np.testing.assert_allclose(frame.coordinates(aligned),relative,atol=1e-12)
            admitted=bool(np.all(planes[:,:3]@raw+planes[:,3]<=1e-9))
            self.assertEqual(admitted,relative==[2.,.8,1.])
        self.assertEqual(record['additional_boundary_inset_m'],0.)

    def test_expansion_preserves_lateral_midpoint_and_unchanged_front(self):
        config=configuration(limits=(-.7,1.5));g=config['venue_calibration']['geometry']
        e=g['execution_envelope']
        np.testing.assert_allclose(e['lateral_limits_m'],[-1.25,1.55])
        self.assertEqual(e['forward_limits_m'],g['forward_limits_m'])
        vertices,_=wireframe(e,dict(ground_fc_xyz=[0.,0.,0.],ground_fc_yaw_rad=0.,origin=[0.,0.,0.]),np.eye(3))
        np.testing.assert_allclose(np.ptp(vertices,axis=0),[5.8,2.8,2.8])
        for key in ('lateral_limits_m','forward_limits_m','floor_height_limits_m'):
            changed=deepcopy(config);changed['venue_calibration']['geometry']['execution_envelope'][key][1]+=.1
            with self.assertRaisesRegex(ValueError,'Execution venue changed'):validate_geometry(changed)
        shifted=deepcopy(g)
        shifted['forward_limits_m']=[x+.5 for x in shifted['forward_limits_m']]
        with self.assertRaisesRegex(ValueError,'Planning box must fit'):separate_execution_geometry(shifted)

    def test_old_missions_and_report_are_explicit(self):
        old=DemoMission(configuration(False))
        self.assertIsNone(old.execution_envelope)
        for point in ([2.,0.,1.],[2.,.9,1.]):
            for check in (old.check,old.check_execution):
                if point[1]>.864:
                    with self.assertRaises(ValueError):check(point)
                else:check(point)
        mission=DemoMission(configuration())
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'report.json'
            MissionReport(path,mission,'check-only')
            policy=json.loads(path.read_text())['venue_boundary_policy']
            self.assertEqual(policy['planning_outer_box']['floor_height_limits_m'],[0.,2.])
            self.assertEqual(policy['measured_execution_outer_box']['floor_height_limits_m'],[0.,2.8])

    def test_explicit_selection_required_and_preserved_in_rehearsal(self):
        from super_demo import arguments,DemoPreparation
        args=arguments(['--mission-file','mission.json','--calibrate-venue','--venue-width','2',
                        '--venue-ceiling','2','--full-size-execution-venue'])
        self.assertTrue(args.full_size_execution_venue)
        with self.assertRaises(SystemExit):arguments(['--mission-file','mission.json','--full-size-execution-venue'])
        with tempfile.TemporaryDirectory() as directory:
            runner=DemoPreparation.__new__(DemoPreparation)
            runner.args=args;runner.folder=Path(directory);runner.path=runner.folder/'preparation.json'
            runner.report={};runner.calibration_source=fixture()
            with patch('calibrate_venue_rviz.calibrate',return_value=configuration()) as capture:
                runner.after_vision_ready()
                self.assertIs(capture.call_args.kwargs['full_size_execution_venue'],True)
                self.assertIsNotNone(DemoMission.load(args.mission_file).execution_envelope)
        rehearsal=DemoMission(DemoMission.ground_rehearsal(configuration(),1.5))
        self.assertIsNotNone(rehearsal.execution_envelope)
        self.assertEqual(rehearsal.height,2.)

    def test_live_controller_reads_both_boxes_without_expanding_planner(self):
        import rclpy
        from test_live_venue import write_reference
        from live_venue import SOURCE_TOPICS
        config=configuration()
        config['venue_calibration']['anchor']=dict(ground_fc_xyz=[.2,-.3,.1],ground_fc_yaw_rad=.7,
            origin=[0.,0.,0.],raw_quaternion=quaternion_rpy(.1,-.1,.4).tolist(),
            source_ids={t:'01'*24 for t in SOURCE_TOPICS})
        rclpy.init();node=None
        try:
            with tempfile.TemporaryDirectory() as directory:
                folder=Path(directory);mission=folder/'mission.json'
                mission.write_text(json.dumps(config))
                alignment=folder/'alignment.py';alignment.write_text('def convert(p,q,c):\n return p,q,c\n')
                config['venue_calibration']['alignment_sha256']=hashlib.sha256(alignment.read_bytes()).hexdigest()
                mission.write_text(json.dumps(config))
                record,_,_,planner=write_reference(folder,mission=mission,alignment=alignment)
                node=DemoMissionControl(mission,folder/'report.json',alignment_path=alignment,
                    planner_config=planner,venue_reference=record)
                self.assertFalse(node.execute)
                self.assertEqual(node.mission.height,2.)
                self.assertEqual(node.live_venue.data['boundary_policy'],node.mission.boundary_policy())
                self.assertIn('references use planning box',node.position_policy_message())
                changed=json.loads(record.read_text())
                changed['boundary_policy']['measured_execution_outer_box']['floor_height_limits_m'][1]=3.
                record.write_text(json.dumps(changed))
                from live_venue import LiveVenue
                with self.assertRaisesRegex(ValueError,'boundary policy differs'):
                    LiveVenue(record,mission,alignment,planner)
        finally:
            if node is not None:node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':unittest.main()
