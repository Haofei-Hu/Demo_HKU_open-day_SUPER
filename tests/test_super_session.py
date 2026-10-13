"""Lifecycle checks without real aircraft or ROS publishers."""
import argparse
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('session', ROOT/'scripts/maintenance/super_session.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class FakeGate:
    blocked = False
    stale_preview = False
    bad_localization = False
    values = dict(EKF2_EV_CTRL=0, EKF2_HGT_REF=3, EKF2_EV_POS_X=0., EKF2_EV_POS_Y=0., EKF2_EV_POS_Z=0., EKF2_EV_QMIN=0)
    writes = []
    def __init__(self):
        if self.blocked:
            raise RuntimeError('not grounded')
        self.node = argparse.Namespace(count_publishers=lambda topic: 0)
        self.receipts = {'preview':float('inf'), 'vision':float('inf')}
    def check(self):
        if self.blocked:
            raise RuntimeError('not grounded')
    def pull(self): pass
    def parameters(self, names): return {name:self.values[name] for name in names}
    def set_ctrl(self, value): self.writes.append(value)
    def grounded(self): return not self.blocked
    def pose_ready(self, name): return not (self.stale_preview and name=='preview')
    def wait(self, predicate, description):
        if not predicate(): raise RuntimeError(description)
    def localization_ready(self): return not self.bad_localization
    def close(self): pass


class MemorySession(m.Session):
    def __init__(self, args):
        super().__init__(args)
        self.config={'hover_execute':False, 'logs':'/tmp/logs'}
        self.window_state={name:'0' for name in m.CORE}
        self.started=[];self.operations=[]
    def load(self): pass
    def save(self): pass
    def windows(self): return self.window_state.copy()
    def launch(self, name, command, first=False):
        self.started.append((name,command))
        self.window_state[name]='0'
    def interrupt(self, name):
        self.operations.append(('interrupt',name))
        self.window_state[name]='1'
    def tmux(self, *args, **kwargs):
        self.operations.append(args)
        if args[0]=='kill-window': self.window_state.pop(args[2].split(':')[-1])


class Tests(unittest.TestCase):
    def setUp(self):
        FakeGate.blocked=FakeGate.stale_preview=FakeGate.bad_localization=False
        FakeGate.values.update(EKF2_EV_CTRL=0,EKF2_HGT_REF=3)
        FakeGate.writes=[]

    def manager(self, argv): return MemorySession(m.arguments(argv))

    def test_invalid_quaternions_and_execution_action(self):
        for argv in (['vision'],['vision','0','0','0','nan'],['vision','0','0','0','2'],
                     ['start','--execute'],['start','--session','bad;touch'],['hover','--hover-height','2'],
                     ['hover','--planner-profile','low-speed']):
            with self.assertRaises(SystemExit): m.arguments(argv)
        self.assertEqual(m.arguments(['vision','0','0','0','1']).quaternion,[0.,0.,0.,1.])

    def test_existing_flight_blocks_mutations(self):
        FakeGate.blocked=True
        for action in ('align','vision','planner','goal','planner-check','planner-stop','hover','stop'):
            args=[action]+(['0','0','0','1'] if action=='vision' else [])
            s=self.manager(args)
            with patch.object(m,'GroundGate',FakeGate), self.assertRaises(RuntimeError): s.mutate(action)
            self.assertEqual(s.started,[])
            self.assertEqual(s.operations,[])
        self.assertEqual(FakeGate.writes,[])

    def test_align_requires_core_only_and_never_saves_quaternion(self):
        s=self.manager(['align']);s.window_state['vision']='0'
        with patch.object(m,'GroundGate',FakeGate),patch.object(m,'run') as run:
            with self.assertRaises(RuntimeError): s.mutate('align')
            run.assert_not_called()
        s=self.manager(['align'])
        with patch.object(m,'GroundGate',FakeGate),patch.object(m,'run') as run:
            s.mutate('align')
            self.assertEqual(len(run.call_args_list),2)
            self.assertFalse(any('update_preview' in str(call) for call in run.call_args_list))
        self.assertEqual(FakeGate.writes,[0])
        self.assertEqual(s.started,[])

    def test_vision_initializes_once_then_enables_fusion(self):
        s=self.manager(['vision','0','0','0','1'])
        with patch.object(m,'GroundGate',FakeGate),patch.object(m,'run') as run:
            s.mutate('vision')
            self.assertIn('update_preview_alignment.py',str(run.call_args))
            self.assertEqual([name for name,_ in s.started],['preview','vision'])
            self.assertNotIn('--execute',str(s.started))
            with self.assertRaises(RuntimeError): s.mutate('vision')
        self.assertEqual(FakeGate.writes,[11])

    def test_bad_configuration_or_stale_preview_blocks_fusion(self):
        FakeGate.values['EKF2_HGT_REF']=1
        s=self.manager(['vision','0','0','0','1'])
        with patch.object(m,'GroundGate',FakeGate),patch.object(m,'run') as run:
            with self.assertRaises(RuntimeError): s.mutate('vision')
            run.assert_not_called()
        FakeGate.values['EKF2_HGT_REF']=3
        FakeGate.stale_preview=True
        with patch.object(m,'GroundGate',FakeGate),patch.object(m,'run'):
            with self.assertRaises(RuntimeError): s.mutate('vision')
        self.assertEqual(FakeGate.writes,[])
        self.assertEqual([name for name,_ in s.started],['preview'])

    def test_hover_only_launches_and_check_only_upgrade(self):
        s=self.manager(['hover']);s.window_state.update(preview='0',vision='0')
        with patch.object(m,'GroundGate',FakeGate):
            s.mutate('hover')
            self.assertNotIn('--execute',s.started[-1][1])
            s.args.execute=True
            s.mutate('hover')
            self.assertIn('--execute',s.started[-1][1])
            with self.assertRaises(RuntimeError): s.mutate('hover')
            s.args.restart=True
            s.mutate('hover')
            self.assertIn('--execute',s.started[-1][1])
            self.assertEqual(s.window_state['vision'],'0')
        self.assertEqual(FakeGate.writes,[])
        self.assertFalse(any('first_hover/start' in command for _,command in s.started))

    def test_stop_orders_control_before_localization_and_mavros_last(self):
        s=self.manager(['stop']);s.window_state.update(preview='0',vision='0',hover='0')
        with patch.object(m,'GroundGate',FakeGate): s.mutate('stop')
        names=[op[1] for op in s.operations if op[0]=='interrupt']
        self.assertEqual(names,['hover','vision','preview','body','lio','lidar','mavros'])
        self.assertEqual(s.operations[-1],('kill-session','-t','super-flight'))

    def test_ground_planner_has_no_flight_actions_and_excludes_hover(self):
        import types
        package=types.ModuleType('mars_quadrotor_msgs')
        messages=types.ModuleType('mars_quadrotor_msgs.msg')
        messages.PositionCommand=lambda:object()
        s=self.manager(['planner']);s.window_state.update(preview='0',vision='0')
        with patch.object(m,'GroundGate',FakeGate), patch.dict('sys.modules',
                {'mars_quadrotor_msgs':package,'mars_quadrotor_msgs.msg':messages}), \
                patch.object(s,'planner_profile',return_value=Path('/tmp/mid360_ground_diagnostic_ros2.yaml')):
            s.mutate('planner')
            self.assertEqual([name for name,_ in s.started],['planner','cmd_preview'])
            self.assertNotIn('--execute',str(s.started))
            with self.assertRaises(RuntimeError):s.mutate('planner')
            s.args=m.arguments(['hover','--execute'])
            with self.assertRaises(RuntimeError):s.mutate('hover')
            s.mutate('planner-stop')
        self.assertEqual(FakeGate.writes,[])
        self.assertEqual(s.window_state['vision'],'0')
        self.assertEqual(s.window_state['preview'],'0')
        self.assertFalse(any('kill-session'==op[0] for op in s.operations))

    def test_planning_requires_fused_localization_and_goal_is_readonly(self):
        s=self.manager(['planner']);s.window_state.update(preview='0',vision='0')
        FakeGate.bad_localization=True
        with patch.object(m,'GroundGate',FakeGate),self.assertRaises(RuntimeError):s.mutate('planner')
        self.assertEqual(s.started,[])
        FakeGate.bad_localization=False
        with tempfile.TemporaryDirectory() as directory:
            s=self.manager(['goal','--scripts-dir',directory])
            s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0')
            helper=Path(directory)/'send_control_preview_goal.py';helper.write_text('# fixture')
            with patch.object(m,'GroundGate',FakeGate),patch.object(m,'run') as run:
                s.mutate('goal')
                self.assertEqual(run.call_args.args[0],['python3','-u',helper])
            self.assertEqual(s.started,[])
        self.assertEqual(FakeGate.writes,[])

    def test_reference_preview_is_ground_only_and_stops_before_planner(self):
        import types
        messages=types.ModuleType('mars_quadrotor_msgs.msg')
        messages.PolynomialTrajectory=lambda:object()
        with tempfile.TemporaryDirectory() as directory:
            helper=Path(directory)/'bounded_reference_preview.py';helper.write_text('# fixture')
            s=self.manager(['reference-preview','--scripts-dir',directory])
            s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0')
            with patch.object(m,'GroundGate',FakeGate),patch.dict('sys.modules',{'mars_quadrotor_msgs.msg':messages}):
                s.mutate('reference-preview')
                self.assertEqual([name for name,_ in s.started],['reference_preview'])
                self.assertNotIn('--execute',s.started[-1][1])
                with self.assertRaises(RuntimeError):s.mutate('reference-preview')
                s.args=m.arguments(['hover'])
                with self.assertRaises(RuntimeError):s.mutate('hover')
                s.mutate('planner-stop')
            names=[op[1] for op in s.operations if op[0]=='interrupt']
            self.assertEqual(names,['reference_preview','cmd_preview','planner'])
            self.assertEqual(s.window_state['vision'],'0')
            self.assertEqual(FakeGate.writes,[])

    def test_planner_profile_preserves_existing_configuration_and_validates_topics(self):
        import yaml
        data=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory);folder=home/'super_ws/src/SUPER/super_planner/config';folder.mkdir(parents=True)
            target=folder/'mid360_ground_diagnostic_ros2.yaml';target.write_text(yaml.safe_dump(data))
            before=target.read_bytes();s=self.manager(['planner'])
            with patch.object(m.Path,'home',return_value=home),patch.object(m,'run') as run:
                self.assertEqual(s.planner_profile(),target)
                run.assert_not_called()
                self.assertEqual(target.read_bytes(),before)
                data['rog_map']['ros_callback']['cloud_topic']='/wrong'
                target.write_text(yaml.safe_dump(data))
                with self.assertRaises(RuntimeError):s.planner_profile()

    def test_planner_check_is_readonly_and_requires_managed_planning(self):
        with tempfile.TemporaryDirectory() as directory:
            s=self.manager(['planner-check','--scripts-dir',directory])
            s.window_state.update(preview='0',vision='0')
            helper=Path(directory)/'control_chain_age_check.py';helper.write_text('# fixture')
            with patch.object(m,'GroundGate',FakeGate),patch.object(m,'run') as run:
                with self.assertRaises(RuntimeError):s.mutate('planner-check')
                run.assert_not_called()
                s.window_state.update(planner='0',cmd_preview='0')
                s.mutate('planner-check')
                self.assertEqual(run.call_args.args[0],['python3','-u',helper,'--check-motion',
                    '--max-speed','0.3','--max-acceleration','0.5'])
            self.assertEqual(s.started,[])
            self.assertEqual(s.operations,[])
            self.assertEqual(FakeGate.writes,[])

    def test_commands_keep_argument_boundaries(self):
        args=m.arguments(['start','--scripts-dir',"/tmp/path with spaces/$(touch BAD)"])
        commands=m.Session(args).core_commands()
        import shlex
        body=shlex.split(commands['body'])
        self.assertEqual(body[-1],'/tmp/path with spaces/$(touch BAD)/fastlio_body_debug.py')
        self.assertIn('publish.scan_publish_en:=true',commands['lio'])
        self.assertFalse(any('first_hover' in command for command in commands.values()))

    def test_margin_profile_requires_reserved_acceleration_target(self):
        import yaml
        data=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory);folder=home/'super_ws/src/SUPER/super_planner/config';folder.mkdir(parents=True)
            diagnostic=folder/'mid360_ground_diagnostic_ros2.yaml'
            diagnostic.write_text(yaml.safe_dump(data));before=diagnostic.read_bytes()
            data['traj_opt']['boundary'].update(max_vel=.3,max_acc=.45)
            target=folder/'mid360_low_speed_margin_ros2.yaml';target.write_text(yaml.safe_dump(data))
            s=self.manager(['planner','--planner-profile','low-speed-margin'])
            with patch.object(m.Path,'home',return_value=home),patch.object(m,'run') as run:
                self.assertEqual(s.planner_profile(),target)
                run.assert_not_called()
                self.assertEqual(diagnostic.read_bytes(),before)
                data['traj_opt']['boundary']['max_acc']=.5
                target.write_text(yaml.safe_dump(data))
                with self.assertRaisesRegex(RuntimeError,'max_acc=0.45'):s.planner_profile()

    def test_reserved_profile_preserves_old_candidates_and_checks_both_limits(self):
        import yaml
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory);folder=home/'super_ws/src/SUPER/super_planner/config';folder.mkdir(parents=True)
            diagnostic=folder/'mid360_ground_diagnostic_ros2.yaml'
            diagnostic.write_bytes((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_bytes())
            scripts=home/'Downloads';scripts.mkdir()
            (scripts/'create_super_motion_profiles.py').write_bytes(
                (ROOT/'scripts/maintenance/create_super_motion_profiles.py').read_bytes())
            import subprocess
            subprocess.run(['python3',str(scripts/'create_super_motion_profiles.py'),'--source',str(diagnostic)],check=True)
            target=folder/'mid360_low_speed_reserved_ros2.yaml'
            target.unlink()  # Simulate an installation with only the previous candidates.
            preserved={p:p.read_bytes() for p in folder.glob('*.yaml') if p != target}
            s=self.manager(['planner','--planner-profile','low-speed-reserved','--scripts-dir',str(scripts)])
            with patch.object(m.Path,'home',return_value=home):
                self.assertEqual(s.planner_profile(),target)
            with patch.object(m.Path,'home',return_value=home),patch.object(m,'run') as run:
                self.assertEqual(s.planner_profile(),target)
                run.assert_not_called()
                for name, bad_value in (('max_vel',.3),('max_acc',.5)):
                    data=yaml.safe_load(target.read_text())
                    data['traj_opt']['boundary'].update(max_vel=.25,max_acc=.45)
                    data['traj_opt']['boundary'][name]=bad_value
                    target.write_text(yaml.safe_dump(data))
                    with self.assertRaisesRegex(RuntimeError,'max_vel=0.25 and max_acc=0.45'):
                        s.planner_profile()
                self.assertEqual(preserved,{p:p.read_bytes() for p in preserved})

    def test_3d_profile_preserves_reserved_file_and_rejects_fixed_height(self):
        import yaml
        data=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
        data['traj_opt']['boundary'].update(max_vel=.25,max_acc=.45)
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory);folder=home/'super_ws/src/SUPER/super_planner/config';folder.mkdir(parents=True)
            (folder/'mid360_ground_diagnostic_ros2.yaml').write_text(yaml.safe_dump(data))
            reserved=folder/'mid360_low_speed_reserved_ros2.yaml';reserved.write_text(yaml.safe_dump(data))
            before=reserved.read_bytes()
            scripts=home/'Downloads';scripts.mkdir()
            (scripts/'create_super_3d_goal_profile.py').write_bytes((ROOT/'scripts/maintenance/create_super_3d_goal_profile.py').read_bytes())
            s=self.manager(['planner','--planner-profile','low-speed-3d','--scripts-dir',str(scripts)])
            with patch.object(m.Path,'home',return_value=home):
                target=s.planner_profile()
            self.assertEqual(target.name,'mid360_low_speed_3d_goal_ros2.yaml')
            expected=yaml.safe_load(before);expected['fsm']['click_height']=-6.
            self.assertEqual(yaml.safe_load(target.read_text()),expected)
            with patch.object(m.Path,'home',return_value=home),patch.object(m,'run') as run:
                s.planner_profile();run.assert_not_called()
                expected['fsm']['click_height']=-.05;target.write_text(yaml.safe_dump(expected))
                with self.assertRaisesRegex(RuntimeError,'click_height=-6'):s.planner_profile()
            self.assertEqual(reserved.read_bytes(),before)

    def test_low_speed_profile_is_separate_and_rejects_unexpected_limits(self):
        import yaml
        data=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
        with tempfile.TemporaryDirectory() as directory:
            home=Path(directory);folder=home/'super_ws/src/SUPER/super_planner/config';folder.mkdir(parents=True)
            diagnostic=folder/'mid360_ground_diagnostic_ros2.yaml'
            diagnostic.write_text(yaml.safe_dump(data));before=diagnostic.read_bytes()
            target=folder/'mid360_low_speed_preview_ros2.yaml'
            scripts=home/'Downloads';scripts.mkdir()
            (scripts/'create_super_motion_profiles.py').write_bytes(
                (ROOT/'scripts/maintenance/create_super_motion_profiles.py').read_bytes())
            s=self.manager(['planner','--planner-profile','low-speed','--scripts-dir',str(scripts)])
            with patch.object(m.Path,'home',return_value=home):
                self.assertEqual(s.planner_profile(),target)
                self.assertEqual(diagnostic.read_bytes(),before)
                demo=yaml.safe_load((folder/'mid360_demo_candidate_ros2.yaml').read_text())
                self.assertEqual(demo['traj_opt']['boundary']['max_vel'],2.)
                self.assertEqual(demo['traj_opt']['boundary']['max_acc'],1.)
            with patch.object(m.Path,'home',return_value=home),patch.object(m,'run') as run:
                self.assertEqual(s.planner_profile(),target)
                run.assert_not_called()
                self.assertEqual(diagnostic.read_bytes(),before)
                data=yaml.safe_load(target.read_text())
                data['traj_opt']['boundary']['max_vel']=2.
                target.write_text(yaml.safe_dump(data))
                with self.assertRaisesRegex(RuntimeError,'Low-speed preview'):
                    s.planner_profile()


if __name__=='__main__': unittest.main()
