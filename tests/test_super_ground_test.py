"""Automatic survey consumption, staged failures and non-flight orchestration."""
import fcntl
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import subprocess
import unittest
from unittest.mock import patch
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/maintenance'))
import super_ground_test as m


def survey(**changes):
    result = dict(schema_version=1, estimate_available=True, matched=705,
        quaternion_xyzw=[-.027270005024089834,-.02941700541964249,.965770177928685,.2562790472155746],
        pairing_gap_max_s=.033, body_age_min_s=.01, body_age_max_s=.019,
        rotation_spread_max_deg=.756)
    result.update(changes)
    return result


class Observer:
    error = None
    calls = 0
    status = 'READY: DEBUG ONLY; fault=None'
    def __init__(self,**kwargs):
        self.motion_limits=kwargs.get('motion_limits')
    def wait_ready(self):
        pass
    def audit(self, seconds):
        type(self).calls += 1
        if self.error:
            raise RuntimeError(self.error)
        report=dict(sampled_vision_chain_fresh=True, end_hold_s=7.)
        if self.motion_limits:report['planner_motion']=dict(peak_speed_mps=2.,peak_acceleration_mps2=1.)
        return report
    def close(self):
        pass
    def snapshot(self):
        return dict(final_status=self.status)


class Tests(unittest.TestCase):
    def test_measured_quaternion_is_preserved_without_rounding(self):
        value=survey()
        self.assertEqual(m.alignment_quaternion(value),value['quaternion_xyzw'])

    def test_bad_alignment_stops_before_application(self):
        for change in (dict(estimate_available=False), dict(matched=19),
                       dict(quaternion_xyzw=[0,0,0,2]), dict(quaternion_xyzw=[0,0,0,float('nan')]),
                       dict(pairing_gap_max_s=.051), dict(body_age_max_s=.251),
                       dict(body_age_min_s=-.051), dict(rotation_spread_max_deg=2.01)):
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                m.alignment_quaternion(survey(**change))

    def runner(self, folder, extra=(), existing=False, alignment=None, ready=True):
        scripts=Path(folder)/'Downloads';scripts.mkdir(exist_ok=True)
        for name in (*m.manager.SCRIPTS,'super_session.py','control_chain_age_check.py',
                     'super_flight_control.py','ground_setpoint_transport.py','ground_speed_audit.py',
                     'create_super_speed_ground_profile.py','create_super_motion_profiles.py',
                     'bounded_reference_preview.py'):
            (scripts/name).touch()
        runner=m.GroundTest(m.arguments(['--scripts-dir',str(scripts),*extra]))
        runner.session.exists=lambda:existing
        runner.session.config=dict(logs=str(scripts/'logs'),fixed_lio_extrinsics=True)
        runner.session.load=lambda:None
        calls=[]
        def action(name,*args):
            runner.report['stage']=name
            calls.append((name,args))
            if name=='align':
                Path(args[1]).write_text(json.dumps(alignment or survey()))
            if name=='flight-check':
                Path(args[1]).write_text(json.dumps(dict(remote_preparation_checks_passed=ready)))
        runner.action=action
        return runner,calls

    def setUp(self):
        Observer.error=None
        Observer.calls=0

    def test_nominal_workflow_uses_current_survey_once_and_saves_result(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(m,'AuditObserver',Observer):
            runner,calls=self.runner(folder,extra=['--fixed-lio-extrinsics'])
            output=io.StringIO()
            with redirect_stdout(output):self.assertEqual(runner.run(),0)
            self.assertEqual([c[0] for c in calls],['start','align','vision','planner','handover-check','flight-check'])
            self.assertEqual(calls[0][1],('--fixed-lio-extrinsics',))
            self.assertEqual(list(calls[2][1]),survey()['quaternion_xyzw'])
            self.assertEqual(Observer.calls,1)
            result=json.loads(runner.path.read_text())
            self.assertTrue(result['passed'])
            self.assertEqual(result['result'],'SUCCESS')
            self.assertIn('RESULT: SUCCESS',output.getvalue())
            self.assertIn('REPORT: '+str(runner.path),output.getvalue())
            self.assertIn('finished_at',result)
            self.assertFalse(result['native_output'])
            self.assertFalse(result['mode_or_arming_requests'])
            self.assertIn('readiness',result)

    def test_existing_session_is_retained_without_explicit_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            runner,calls=self.runner(folder,existing=True)
            self.assertEqual(runner.run(),1)
            self.assertEqual(calls,[])

    def test_fast_profile_is_separate_and_never_promotes_flight_readiness(self):
        required={'connected_disarmed_on_ground','fresh_valid_localization','fc_vision_pose_agreement',
                  'required_parameters','local_ned','no_native_control_publishers',
                  'single_localization_sources','managed_processes_active','ground_speed_profile'}
        ready=dict(checks={name:True for name in required},errors=[],remote_preparation_checks_passed=False)
        ready['checks']['reserved_3d_planner']=False
        self.assertTrue(m.ground_speed_prerequisites(ready))
        for name in required:
            changed=json.loads(json.dumps(ready));changed['checks'][name]=False
            self.assertFalse(m.ground_speed_prerequisites(changed),name)
        with tempfile.TemporaryDirectory() as folder,patch.object(m,'AuditObserver',Observer):
            runner,calls=self.runner(folder,extra=['--test-profile','forward-5m-2mps'])
            original=runner.action
            def action(name,*args):
                original(name,*args)
                if name=='flight-check':Path(args[1]).write_text(json.dumps(ready))
            runner.action=action
            self.assertEqual(runner.args.sample_seconds,40.)
            self.assertEqual(runner.run(),0)
            self.assertIn(('planner',('--planner-profile','forward-5m-2mps')),calls)
            self.assertIn('speed-check',[call[0] for call in calls])
            result=json.loads(runner.path.read_text())
            self.assertTrue(result['ground_prerequisites_passed'])
            self.assertFalse(result['readiness']['remote_preparation_checks_passed'])
            self.assertEqual(result['audit']['planner_motion']['peak_speed_mps'],2.)

    def test_explicit_restart_uses_guarded_manager_stop_before_start(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(m,'AuditObserver',Observer):
            runner,calls=self.runner(folder,existing=True,extra=['--restart'])
            self.assertEqual(runner.run(),0)
            self.assertEqual([c[0] for c in calls][:2],['stop','start'])

    def test_failed_survey_never_starts_vision_or_planner(self):
        with tempfile.TemporaryDirectory() as folder:
            runner,calls=self.runner(folder,alignment=survey(rotation_spread_max_deg=3.))
            output=io.StringIO()
            with redirect_stdout(output):self.assertEqual(runner.run(),1)
            self.assertEqual([c[0] for c in calls],['start','align'])
            self.assertFalse(json.loads(runner.path.read_text())['passed'])
            result=json.loads(runner.path.read_text())
            self.assertEqual(result['result'],'FAIL')
            self.assertEqual(result['failed_stage'],'align')
            self.assertIn('RESULT: FAIL',output.getvalue())
            self.assertNotIn('RESULT: SUCCESS',output.getvalue())
            self.assertIn('REPORT: '+str(runner.path),output.getvalue())

    def test_failed_audit_is_not_retried_or_promoted_to_pass(self):
        Observer.error='Missing/stale vision'
        with tempfile.TemporaryDirectory() as folder, patch.object(m,'AuditObserver',Observer):
            runner,calls=self.runner(folder)
            self.assertEqual(runner.run(),1)
            self.assertEqual(Observer.calls,1)
            self.assertNotIn('flight-check',[c[0] for c in calls])
            self.assertIn('Missing/stale vision',json.loads(runner.path.read_text())['error'])

    def test_failed_final_readiness_prevents_success(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(m,'AuditObserver',Observer):
            runner,_=self.runner(folder,ready=False)
            self.assertEqual(runner.run(),1)
            self.assertFalse(json.loads(runner.path.read_text())['passed'])

    def test_runner_has_no_execute_option(self):
        with self.assertRaises(SystemExit):m.arguments(['--execute'])

    def test_child_failure_is_reported_and_stops_before_alignment(self):
        with tempfile.TemporaryDirectory() as folder:
            runner,_=self.runner(folder)
            (runner.args.scripts_dir/'super_session.py').write_text(
                "print('fixture core failure')\nraise SystemExit(7)\n")
            runner.action=m.GroundTest.action.__get__(runner)
            self.assertEqual(runner.run(),1)
            result=json.loads(runner.path.read_text())
            self.assertEqual(result['stage'],'start')
            self.assertEqual(result['stages'][0]['returncode'],7)
            self.assertIn('fixture core failure',Path(result['stages'][0]['log']).read_text())
            self.assertIn('fixture core failure',result['failure_output'])
            self.assertFalse((runner.folder/'alignment.json').exists())

    def test_generated_vision_command_passes_real_manager_parser(self):
        quaternions=[
            [-.010378974288993956,-.01017938613075453,.9638858381472019,.2659186868386183],
            [-1e-20,1e-30,0.,1.]]
        for q in quaternions:
            with self.subTest(q=q),tempfile.TemporaryDirectory(prefix='auto cli ') as folder:
                runner,_=self.runner(folder)
                runner.action=m.GroundTest.action.__get__(runner)
                def parsed_child(command,**kwargs):
                    args=m.manager.arguments(command[2:])
                    self.assertEqual(args.action,'vision')
                    self.assertEqual(args.quaternion,q)
                    self.assertEqual(args.scripts_dir,runner.args.scripts_dir)
                    return subprocess.CompletedProcess(command,0)
                with patch.object(m.subprocess,'run',side_effect=parsed_child):
                    runner.action('vision',*q)

    def test_global_lock_blocks_a_second_runner_before_any_action(self):
        with tempfile.TemporaryDirectory() as folder:
            home=Path(folder);state=home/'.local/state/super-session';state.mkdir(parents=True)
            with (state/'automated_ground_test.lock').open('a') as handle:
                fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
                with patch.object(m.Path,'home',return_value=home), \
                     patch.object(sys,'argv',['super_ground_test.py','--ros-env','--session','other']), \
                     patch.object(m,'GroundTest') as runner:
                    self.assertEqual(m.main(),1)
                    runner.assert_not_called()


if __name__=='__main__':unittest.main()
