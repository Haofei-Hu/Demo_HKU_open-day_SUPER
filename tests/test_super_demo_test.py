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
import super_demo as physical_entry


class Tests(unittest.TestCase):
    def runner(self,folder,logic=False,fixture=True,motion='strict'):
        base=Path(folder);mission=base/'mission.json'
        mission.write_bytes((ROOT/'configs/demonstration/open_day_mission.json').read_bytes())
        for name in ('demo_mission.py','demo_mission_simulation.py','super_demo_control.py','capture_demo_scene.py'):
            (base/name).touch()
        args=argparse.Namespace(scripts_dir=base,mission_file=mission,session='demo',logic_only=logic,
            fixture_geometry=fixture,restart=True,fixed_lio_extrinsics=True,test_profile='forward-5m-2mps',
            max_alignment_spread_deg=2.,motion_policy=motion)
        runner=m.DemoTest(args);actions=[]
        runner.session=NS(config={'planner_config_path':'/test/planner.yaml'},load=lambda:None)
        runner.prepare=lambda planner_profile:actions.append(('prepare',planner_profile))
        def step(name,command,env=None):
            actions.append((name,list(map(str,command)),env))
            if name=='complete_mission':
                frozen=Path(command[command.index('--mission-file')+1])
                phases=m.DemoMission.load(frozen).PHASES
                (runner.folder/'mission_report.json').write_text(json.dumps(dict(result='SUCCESS',
                    completed_stages=list(phases),simulation_backend=dict(actual_SUPER_used=not logic))))
        runner.step=step
        return runner,actions

    def gate(self):return patch.object(m.manager,'GroundGate',return_value=NS(check=lambda:None,close=lambda:None))

    def test_physical_preparation_defaults_to_observe_without_execution(self):
        args=physical_entry.arguments(['--mission-file','measured.json'])
        self.assertEqual(args.motion_policy,'observe')
        self.assertFalse(args.execute)
        self.assertEqual(physical_entry.arguments(['--mission-file','measured.json',
            '--motion-policy','strict']).motion_policy,'strict')

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

    def test_qgc_is_opened_before_actual_stack_preparation_and_not_in_logic_only(self):
        with tempfile.TemporaryDirectory() as folder,self.gate():
            runner,actions=self.runner(folder)
            runner.open_qgc=lambda:actions.append(('qgc',))
            self.assertEqual(runner.run_complete(),0)
            self.assertEqual([a[0] for a in actions][:2],['qgc','prepare'])
        with tempfile.TemporaryDirectory() as folder:
            runner,_=self.runner(folder,logic=True)
            with patch.object(runner,'open_qgc') as launch:
                self.assertEqual(runner.run_complete(),0);launch.assert_not_called()

    def test_physical_qgc_launch_failure_is_saved_before_preparation(self):
        from super_ground_test import GroundTest
        with tempfile.TemporaryDirectory() as folder:
            runner,_=self.runner(folder)
            with patch('qgc_startup.open_qgc',return_value={'status':'unavailable','reason':'missing app'}):
                with self.assertRaisesRegex(RuntimeError,'Open QGroundControl'):
                    GroundTest.open_qgc(runner,required=True)
            saved=json.loads(runner.path.read_text())
            self.assertEqual(saved['stage'],'qgc');self.assertEqual(saved['qgc']['status'],'unavailable')

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

    def test_failed_rehearsal_retains_actual_planner_rejection_in_report(self):
        with tempfile.TemporaryDirectory() as folder:
            runner,_=self.runner(folder,logic=True)
            def failed(name,command,env=None):
                (runner.folder/'planner_rehearsal.log').write_text(
                    'GenerateExpTrajectory SUCCESS\n'
                    'Opt failed, Omg or thr or Pos violation.\n'
                    'generateBackupTrajectory return [OPT_FAILED]\n')
                raise RuntimeError('No verified mission reference within 8 s')
            runner.step=failed
            self.assertEqual(runner.run_complete(),1)
            report=json.loads(runner.path.read_text())
            evidence=report['planner_diagnostics']
            self.assertEqual(evidence['marker_counts']['exploration_success'],1)
            self.assertEqual(evidence['marker_counts']['backup_optimization_failure'],1)
            self.assertEqual(evidence['marker_counts']['backup_constraint_rejection'],1)
            self.assertIn('generateBackupTrajectory',evidence['output_tail'])
            self.assertFalse(evidence['cause_inferred'])
            self.assertFalse(report['physical_flight_verified'])

    def test_failed_mission_still_records_observed_real_planner_participation(self):
        with tempfile.TemporaryDirectory() as folder,self.gate():
            runner,_=self.runner(folder)
            def failed(name,command,env=None):
                if name=='complete_mission':
                    (runner.folder/'mission_report.json').write_text(json.dumps(dict(result='FAIL',
                        completed_stages=['TAKEOFF_TO_1_5M'],simulation_backend=dict(actual_SUPER_used=True))))
                    raise RuntimeError('No verified mission reference within 8 s')
            runner.step=failed
            self.assertEqual(runner.run_complete(),1)
            report=json.loads(runner.path.read_text())
            self.assertTrue(report['actual_SUPER_tested'])
            self.assertEqual(report['result'],'FAIL');self.assertFalse(report['demonstration_completed'])
            self.assertFalse(report['physical_flight_verified'])

    def test_short_site_choice_keeps_source_and_reports_one_metre(self):
        with tempfile.TemporaryDirectory() as folder,self.gate():
            runner,_=self.runner(folder)
            runner.args.route_profile='short-1m';before=runner.args.mission_file.read_bytes()
            self.assertEqual(runner.run_complete(),0)
            self.assertEqual(runner.args.mission_file.read_bytes(),before)
            self.assertEqual(runner.report['mission']['mission_id'],'short_site_1m_rehearsal')
            self.assertEqual(runner.report['waypoints']['far_hover'],[1.,0.,1.5])
            self.assertEqual(runner.report['mission_report']['completed_stages'][2],'SUPER_OUTBOUND_1M')
            self.assertFalse(runner.report['demonstration_completed'])

    def test_observe_policy_is_explicit_and_forwarded_to_private_child(self):
        with tempfile.TemporaryDirectory() as folder,self.gate():
            runner,actions=self.runner(folder,motion='observe')
            self.assertEqual(runner.run_complete(),0)
            command=actions[-1][1]
            self.assertEqual(command[command.index('--motion-policy')+1],'observe')
            self.assertEqual(actions[-1][2]['ROS_DOMAIN_ID'],'197')
            self.assertEqual(runner.report['motion_policy'],'observe')

    def test_viewer_defaults_on_and_headless_choice_reaches_private_child(self):
        with tempfile.TemporaryDirectory() as folder,self.gate():
            runner,actions=self.runner(folder)
            self.assertEqual(runner.run_complete(),0)
            self.assertNotIn('--no-rviz',actions[-1][1])
        with tempfile.TemporaryDirectory() as folder,self.gate():
            runner,actions=self.runner(folder);runner.args.no_rviz=True
            self.assertEqual(runner.run_complete(),0)
            self.assertIn('--no-rviz',actions[-1][1])


if __name__=='__main__':unittest.main()
