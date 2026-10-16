"""Independent left-facing scene, mission, installation and report selection."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/simulation'),str(ROOT/'scripts/flight'),str(ROOT/'scripts/planning')]
from demo_mission import DemoMission,MissionReport
from left_rehearsal import LeftMission,mission_fixture,multiply,rotated_scene,successful_scene
from control_bridge_preview import load_converter

spec=importlib.util.spec_from_file_location('left_install',ROOT/'scripts/maintenance/install_left_rehearsal.py')
installer=importlib.util.module_from_spec(spec);spec.loader.exec_module(installer)


def make_scene(folder):
    folder=Path(folder);alignment=folder/'alignment.py';planner=folder/'planner.yaml'
    source=(ROOT/'scripts/localization/aligned_pose_preview.py').read_text()
    source=source.replace('Q_WC = np.array([0.000591, 0.018708, 0.58887, 0.808011])',
                          'Q_WC = np.array([0.12, -0.17, 0.31, 0.91])')
    # Importing this measured preview is never allowed, even for arithmetic.
    alignment.write_text(source+'\nraise RuntimeError("Do not import measured preview")\n')
    planner.write_text('fixture: true\n')
    raw_q=np.array([.1,-.2,.3,.9]);raw_q/=np.linalg.norm(raw_q)
    raw_position=np.array([1.2,-.4,.3]);origin=np.array([.8,.1,-.2])
    p,q,_=load_converter(alignment)(raw_position,raw_q,np.eye(6))
    data=dict(schema_version=1,alignment_path=str(alignment),planner_config=str(planner),
        alignment_sha256=hashlib.sha256(alignment.read_bytes()).hexdigest(),
        planner_sha256=hashlib.sha256(planner.read_bytes()).hexdigest(),
        raw_quaternion=raw_q.tolist(),raw_position=raw_position.tolist(),origin=origin.tolist(),
        aligned_position=(p-origin).tolist(),aligned_quaternion=q.tolist(),
        cloud_serialized_base64='unchanged-map-snapshot',cloud_points=9734)
    scene=folder/'scene.json';scene.write_text(json.dumps(data))
    return scene


def yaw(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


class Tests(unittest.TestCase):
    def test_five_round_route_and_canonical_configuration_are_separate(self):
        canonical=ROOT/'configs/demonstration/open_day_mission.json';before=canonical.read_bytes()
        cfg=mission_fixture();mission=LeftMission(cfg)
        self.assertEqual(len(mission.PHASES),20);self.assertEqual(mission.round_trips,5)
        self.assertEqual(mission.waypoints()['far_hover'],[4.8,0.,.5])
        self.assertEqual(mission.outbound_stage,'SUPER_OUTBOUND_4_8M')
        self.assertEqual(mission.PHASES[2],'ROUND_01_SUPER_OUTBOUND_4_8M')
        self.assertEqual(mission.config['flight']['horizontal_speed_limit_mps'],2.)
        self.assertEqual(mission.config['flight']['acceleration_limit_mps2'],1.)
        self.assertFalse(mission.demonstration);self.assertEqual(canonical.read_bytes(),before)
        with self.assertRaisesRegex(ValueError,'Unknown mission'):DemoMission(cfg)
        cfg.pop('execution_context')
        with self.assertRaisesRegex(ValueError,'virtual rehearsal only'):LeftMission(cfg)

    def test_whole_report_and_virtual_only_scope(self):
        mission=LeftMission(mission_fixture())
        with tempfile.TemporaryDirectory() as directory:
            report=MissionReport(Path(directory)/'report.json',mission,'simulation')
            for stage in mission.PHASES:report.completed(stage)
            report.finish()
            self.assertEqual(report.data['result'],'SUCCESS')
            self.assertEqual(report.data['round_trips_completed'],5)
            self.assertFalse(report.data['physical_flight_verified'])
            self.assertFalse(report.data['demonstration_completed'])
            with self.assertRaisesRegex(ValueError,'virtual execution only'):
                MissionReport(Path(directory)/'physical.json',mission,'flight')

    def test_scene_turns_body_left_without_moving_fc_or_rotating_map(self):
        with tempfile.TemporaryDirectory() as directory:
            source=make_scene(directory);before={p:p.read_bytes() for p in Path(directory).iterdir()}
            original=json.loads(source.read_text());scene=rotated_scene(source)
            delta=(yaw(scene['aligned_quaternion'])-yaw(original['aligned_quaternion']))%(2*math.pi)
            self.assertAlmostEqual(delta,math.pi/2,places=10)
            # The commanded forward vector is now the original heading's left.
            old_yaw=yaw(original['aligned_quaternion']);new_yaw=yaw(scene['aligned_quaternion'])
            np.testing.assert_allclose(4.8*np.array([math.cos(new_yaw),math.sin(new_yaw)]),
                                      4.8*np.array([-math.sin(old_yaw),math.cos(old_yaw)]),atol=1e-10)
            for key in ('aligned_position','origin','cloud_serialized_base64','cloud_points',
                        'alignment_path','alignment_sha256','planner_config','planner_sha256'):
                self.assertEqual(scene[key],original[key])
            p,q,_=load_converter(scene['alignment_path'])(np.array(scene['raw_position']),
                np.array(scene['raw_quaternion']),np.eye(6))
            np.testing.assert_allclose(p-scene['origin'],original['aligned_position'],atol=1e-10)
            np.testing.assert_allclose(q,scene['aligned_quaternion'],atol=1e-10)
            for path,data in before.items():self.assertEqual(path.read_bytes(),data)

    def test_scene_rejects_modified_dependencies_and_double_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            source=make_scene(directory);data=rotated_scene(source)
            rotated=Path(directory)/'rotated.json';rotated.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'already rotated'):rotated_scene(rotated)
            Path(data['planner_config']).write_text('changed: true\n')
            with self.assertRaisesRegex(ValueError,'dependency changed'):rotated_scene(source)

    def test_selects_latest_completed_success_and_ignores_active_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory)
            for name,result,actual in (('001','SUCCESS',True),('002','SUCCESS',True),
                                       ('003','FAIL',True),('004','RUNNING',True),('005','SUCCESS',False)):
                folder=base/name;folder.mkdir();(folder/'scene.json').write_text('{}')
                (folder/'report.json').write_text(json.dumps(dict(result=result,actual_SUPER_tested=actual)))
            self.assertEqual(successful_scene(base),base/'002/scene.json')
            with self.assertRaisesRegex(ValueError,'No successful'):successful_scene(base/'missing')

    def fetch(self,path):return (ROOT/path).read_bytes()

    def test_pinned_bundle_installs_separately_and_reinstall_is_read_only(self):
        with tempfile.TemporaryDirectory() as directory:
            downloads=Path(directory);original=downloads/'super_demo_test.py';original.write_text('successful original\n')
            mission=downloads/'open_day_mission.json';mission.write_text('measured original\n')
            root=downloads/'super_left_48_rehearsal'
            destination=installer.install('a'*40,root,self.fetch)
            self.assertEqual(destination,root/('a'*40))
            self.assertTrue((destination/'super_left_test.py').is_file())
            before={p:p.stat().st_mtime_ns for p in destination.iterdir()}
            self.assertEqual(installer.install('a'*40,root,self.fetch),destination)
            for p,mtime in before.items():self.assertEqual(p.stat().st_mtime_ns,mtime)
            self.assertEqual(original.read_text(),'successful original\n')
            self.assertEqual(mission.read_text(),'measured original\n')

    def test_corrupt_bundle_never_publishes_a_partial_release(self):
        with tempfile.TemporaryDirectory() as directory:
            def corrupt(path):return b'broken' if path.endswith('left_rehearsal.py') else self.fetch(path)
            with self.assertRaisesRegex(ValueError,'Checksum mismatch'):
                installer.install('b'*40,Path(directory),corrupt)
            self.assertEqual(list(Path(directory).iterdir()),[])


if __name__=='__main__':unittest.main()
