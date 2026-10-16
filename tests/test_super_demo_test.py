"""Whole-flow orchestration; no false pass from partial or synthetic substitution."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/flight')]
import super_demo_test as m


class Tests(unittest.TestCase):
    def runner(self,folder,logic=False,fixture=True):
        base=Path(folder);mission=base/'mission.json'
        mission.write_bytes((ROOT/'configs/demonstration/open_day_mission.json').read_bytes())
        for name in ('demo_mission.py','demo_mission_simulation.py','super_demo_control.py','capture_demo_scene.py'):
            (base/name).touch()
        args=argparse.Namespace(scripts_dir=base,mission_file=mission,session='demo',logic_only=logic,
            fixture_geometry=fixture,restart=True,fixed_lio_extrinsics=True,test_profile='forward-5m-2mps',
            max_alignment_spread_deg=2.)
        runner=m.DemoTest(args);actions=[]
        runner.session=NS(config={'planner_config_path':'/test/planner.yaml'},load=lambda:None)
        runner.prepare=lambda planner_profile:actions.append(('prepare',planner_profile))
        def step(name,command,env=None):
            actions.append((name,list(map(str,command)),env))
            if name=='complete_mission':
                (runner.folder/'mission_report.json').write_text(json.dumps(dict(result='SUCCESS',
                    completed_stages=list(m.DemoMission.PHASES),simulation_backend=dict(actual_SUPER_used=not logic))))
        runner.step=step
        return runner,actions

    def gate(self):return patch.object(m.manager,'GroundGate',return_value=NS(check=lambda:None,close=lambda:None))

    def test_default_runs_real_preparation_scene_and_whole_mission(self):
        with tempfile.TemporaryDirectory() as folder,self.gate():
            runner,actions=self.runner(folder)
            self.assertEqual(runner.run_complete(),0)
            self.assertEqual([a[0] for a in actions],['prepare','vision_chain','capture_scene','complete_mission'])
            self.assertEqual(actions[0][1],'demo')
            self.assertIn('--scene-file',actions[-1][1]);self.assertEqual(actions[-1][2]['ROS_DOMAIN_ID'],'197')
            self.assertFalse(runner.report['physical_flight_verified'])
            self.assertFalse(runner.report['physical_venue_clearance_verified'])
            self.assertTrue(runner.report['actual_SUPER_tested'])

    def test_real_mode_rejects_synthetic_fallback(self):
        with tempfile.TemporaryDirectory() as folder,self.gate():
            runner,_=self.runner(folder);step=runner.step
            def substitute(name,command,env=None):
                step(name,command,env)
                if name=='complete_mission':
                    p=runner.folder/'mission_report.json';data=json.loads(p.read_text())
                    data['simulation_backend']['actual_SUPER_used']=False;p.write_text(json.dumps(data))
            runner.step=substitute
            self.assertEqual(runner.run_complete(),1)
            self.assertIn('no synthetic fallback',runner.report['error'])

    def test_explicit_logic_mode_never_starts_aircraft_or_real_planner(self):
        with tempfile.TemporaryDirectory() as folder:
            runner,actions=self.runner(folder,logic=True)
            self.assertEqual(runner.run_complete(),0)
            self.assertEqual([a[0] for a in actions],['complete_mission'])
            self.assertNotIn('--scene-file',actions[0][1])
            self.assertFalse(runner.report['actual_localization_tested'])

    def test_unknown_actual_geometry_stops_before_startup(self):
        with tempfile.TemporaryDirectory() as folder:
            runner,actions=self.runner(folder,fixture=False)
            self.assertEqual(runner.run_complete(),1);self.assertEqual(actions,[])
            self.assertIn('measured geometry',runner.report['error'])

    def test_outbound_only_is_not_complete_mission(self):
        with tempfile.TemporaryDirectory() as folder:
            runner,_=self.runner(folder,logic=True);step=runner.step
            def partial(name,command,env=None):
                step(name,command,env)
                p=runner.folder/'mission_report.json';data=json.loads(p.read_text())
                data['completed_stages']=data['completed_stages'][:4];p.write_text(json.dumps(data))
            runner.step=partial
            self.assertEqual(runner.run_complete(),1)
            self.assertIn('ordered mission',runner.report['error'])


if __name__=='__main__':unittest.main()
