"""Lifecycle checks without real aircraft or ROS publishers."""
import argparse
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import types
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
    publisher_counts = {}
    def __init__(self):
        if self.blocked:
            raise RuntimeError('not grounded')
        self.node = argparse.Namespace(count_publishers=lambda topic: self.publisher_counts.get(topic,0))
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
    def flight_check(self):
        return {'checks':{'fresh_rc_with_seven_channels':False,'posctl_with_manual_input':False},'errors':[]}
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
        FakeGate.publisher_counts={}

    def manager(self, argv): return MemorySession(m.arguments(argv))

    def test_invalid_quaternions_and_execution_action(self):
        for argv in (['vision'],['vision','0','0','0','nan'],['vision','0','0','0','2'],
                     ['start','--execute'],['flight-check','--execute'],['start','--session','bad;touch'],['hover','--hover-height','2'],
                     ['hover','--planner-profile','low-speed']):
            with self.assertRaises(SystemExit): m.arguments(argv)
        self.assertEqual(m.arguments(['vision','0','0','0','1']).quaternion,[0.,0.,0.,1.])

    def test_fixed_lio_extrinsics_changes_only_lio_startup(self):
        baseline=m.Session(m.arguments(['start'])).core_commands()
        fixed=m.Session(m.arguments(['start','--fixed-lio-extrinsics'])).core_commands()
        self.assertEqual(set(baseline),set(fixed))
        for name in ('lidar','mavros','body'):
            self.assertEqual(baseline[name],fixed[name])
        self.assertEqual(fixed['lio'],baseline['lio']+' -p mapping.extrinsic_est_en:=false')
        for action in ('align','vision','planner','stop','hover'):
            argv=[action,'--fixed-lio-extrinsics']
            if action=='vision':argv+=['0','0','0','1']
            with self.assertRaises(SystemExit):m.arguments(argv)

    def test_fixed_lio_start_records_override_and_preserves_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            scripts=Path(directory)/'Downloads';scripts.mkdir()
            for name in m.SCRIPTS:(scripts/name).write_text('# unchanged prerequisite\n')
            before={name:(scripts/name).read_bytes() for name in m.SCRIPTS}
            s=self.manager(['start','--fixed-lio-extrinsics','--scripts-dir',str(scripts)])
            with patch.object(s,'exists',return_value=False),patch.object(m,'SETUPS',()), \
                 patch.object(m,'unmanaged_processes',return_value=[]),patch.object(m,'ros_environment',return_value={}):
                s.start()
            self.assertTrue(s.config['fixed_lio_extrinsics'])
            self.assertFalse(s.config['hover_execute'])
            self.assertEqual([name for name,_ in s.started],list(m.CORE))
            self.assertEqual(before,{name:(scripts/name).read_bytes() for name in m.SCRIPTS})

    def test_fixed_lio_override_cannot_rebase_existing_session(self):
        s=self.manager(['start','--fixed-lio-extrinsics'])
        with patch.object(s,'exists',return_value=True):
            with self.assertRaisesRegex(RuntimeError,'Existing session has no fixed-LIO override'):
                s.start()
        self.assertEqual(s.started,[])

    def test_existing_flight_blocks_mutations(self):
        FakeGate.blocked=True
        for action in ('align','vision','planner','goal','planner-check','planner-stop','wire-test','wire-stop','handover-check','handover-stop','flight-check','hover','stop'):
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

    def test_wire_launch_and_stop_preserve_localization_without_flight_requests(self):
        with tempfile.TemporaryDirectory() as directory:
            helper=Path(directory)/'ground_setpoint_transport.py';helper.write_text('# fixture')
            s=self.manager(['wire-test','--scripts-dir',directory])
            s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0')
            FakeGate.publisher_counts={'/super_debug/control_target':1}
            with patch.object(m,'GroundGate',FakeGate):
                s.mutate('wire-test')
                self.assertEqual(s.started,[('wire_test',m.shell_command(['python3','-u',helper]))])
                with self.assertRaises(RuntimeError):s.mutate('wire-test')
                s.mutate('wire-stop')
            self.assertNotIn('wire_test',s.window_state)
            for name in ('lidar','lio','mavros','body','preview','vision','planner','cmd_preview'):
                self.assertEqual(s.window_state[name],'0')
            self.assertEqual(FakeGate.writes,[])

    def test_wire_rejects_missing_or_duplicate_sources_and_other_control_writers(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory)/'ground_setpoint_transport.py').write_text('# fixture')
            for counts in ({},{'/super_debug/control_target':2},
                           {'/super_debug/control_target':1,'/mavros/setpoint_position/local':1},
                           {'/super_debug/control_target':1,'/mavros/setpoint_raw/local':1}):
                s=self.manager(['wire-test','--scripts-dir',directory])
                s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0')
                FakeGate.publisher_counts=counts
                with patch.object(m,'GroundGate',FakeGate),self.assertRaises(RuntimeError):s.mutate('wire-test')
                self.assertEqual(s.started,[])
            s=self.manager(['wire-test','--scripts-dir',directory])
            with patch.object(m,'GroundGate',FakeGate),self.assertRaises(RuntimeError):s.mutate('wire-test')
            self.assertEqual(s.started,[])

    def test_wire_waits_for_delayed_discovery_and_rejects_duplicate_or_missing_source(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory)/'ground_setpoint_transport.py').write_text('# fixture')
            for counts, succeeds, error in (([0,0,1],True,None),
                                            ([0,2],False,'discovered 2'),
                                            ([0,0,0],False,'zero found')):
                class DiscoveringGate(FakeGate):
                    def __init__(self):
                        super().__init__()
                        self.counts=iter(counts)
                        self.last_count=0
                        self.node.count_publishers=self.count_publishers
                    def count_publishers(self,topic):
                        if topic=='/super_debug/control_target':
                            self.last_count=next(self.counts,self.last_count)
                            return self.last_count
                        return 0
                    def wait(self,predicate,description):
                        for _ in range(4):
                            if predicate():return
                        raise RuntimeError(description)
                s=self.manager(['wire-test','--scripts-dir',directory])
                s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0')
                with patch.object(m,'GroundGate',DiscoveringGate):
                    if succeeds:
                        s.mutate('wire-test')
                        self.assertEqual([name for name,_ in s.started],['wire_test'])
                    else:
                        with self.assertRaisesRegex(RuntimeError,error):s.mutate('wire-test')
                        self.assertEqual(s.started,[])
                self.assertEqual(FakeGate.writes,[])

    def test_wire_commands_source_workspace_setups_without_a_preinitialized_shell(self):
        for action in ('wire-test','wire-stop','handover-check','handover-stop','flight-check'):
            args=m.arguments([action])
            with patch.object(m,'arguments',return_value=args), \
                 patch.object(m.subprocess,'run',side_effect=[argparse.Namespace(returncode=0),
                                                            argparse.Namespace(returncode=17)]) as execute, \
                 patch.object(m,'Session') as session:
                with self.assertRaises(SystemExit) as result:m.main()
                self.assertEqual(result.exception.code,17)
                command=execute.call_args_list[1].args[0]
                self.assertEqual(command[:3],['bash','--noprofile','--norc'])
                self.assertIn('setup.bash',command[-1])
                self.assertIn('--ros-env',command[-1])
                session.assert_not_called()

    def test_flight_check_is_read_only_and_reports_remote_blockers(self):
        with tempfile.TemporaryDirectory() as directory:
            profile=Path(directory)/'planner.yaml'
            profile.write_text('fsm:\n  click_height: -6.0\n  click_goal_topic: /goal_pose\n'
                '  cmd_topic: /planning/pos_cmd\n  click_goal_en: true\n'
                'traj_opt:\n  boundary:\n    max_vel: 0.25\n    max_acc: 0.45\n')
            before=profile.read_bytes()
            s=self.manager(['flight-check'])
            s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0',handover_check='0')
            s.config.update(planner_profile='low-speed-3d',planner_config_path=str(profile))
            output=io.StringIO()
            with patch.object(m,'GroundGate',FakeGate),patch.object(FakeGate,'pull') as pull, \
                    patch.object(m,'run') as run,redirect_stdout(output):
                s.mutate('flight-check')
            report=json.loads(output.getvalue())
            self.assertFalse(report['sampled_software_prerequisites_passed'])
            self.assertTrue(report['remote_preparation_checks_passed'])
            self.assertEqual(set(report['blockers']),{'fresh_rc_with_seven_channels','posctl_with_manual_input'})
            self.assertTrue(report['checks']['reserved_3d_planner'])
            pull.assert_not_called();run.assert_not_called()
            self.assertEqual(FakeGate.writes,[])
            self.assertEqual(s.started,[]);self.assertEqual(s.operations,[])
            self.assertEqual(profile.read_bytes(),before)

    def test_flight_check_reads_actual_planner_limits_and_reports_missing_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            profile=Path(directory)/'planner.yaml'
            profile.write_text('fsm:\n  click_height: -6.0\n  click_goal_topic: /goal_pose\n'
                '  cmd_topic: /planning/pos_cmd\n  click_goal_en: true\n'
                'traj_opt:\n  boundary:\n    max_vel: 2.0\n    max_acc: 1.0\n')
            s=self.manager(['flight-check'])
            s.config.update(planner_profile='low-speed-3d',planner_config_path=str(profile))
            output=io.StringIO()
            with patch.object(m,'GroundGate',FakeGate),redirect_stdout(output):s.mutate('flight-check')
            report=json.loads(output.getvalue())
            self.assertIn('reserved_3d_planner',report['blockers'])
            self.assertIn('managed_processes_active',report['blockers'])
            self.assertEqual(s.started,[]);self.assertEqual(s.operations,[])

    def test_planner_stop_stops_native_wire_first_and_hover_excludes_wire(self):
        s=self.manager(['planner-stop'])
        s.window_state.update(wire_test='0',reference_preview='0',cmd_preview='0',planner='0',preview='0',vision='0')
        with patch.object(m,'GroundGate',FakeGate):s.mutate('planner-stop')
        interrupts=[op[1] for op in s.operations if op[0]=='interrupt']
        self.assertEqual(interrupts,['wire_test','reference_preview','cmd_preview','planner'])
        self.assertEqual(s.window_state['vision'],'0')
        s=self.manager(['hover','--execute']);s.window_state.update(wire_test='0',preview='0',vision='0')
        with patch.object(m,'GroundGate',FakeGate),self.assertRaises(RuntimeError):s.mutate('hover')
        self.assertEqual(s.started,[])

    def test_handover_check_stops_wire_and_launches_without_execute(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name in ('super_flight_control.py','bounded_reference_preview.py'):
                (root/name).write_text('# fixture')
            (root/'first_hover_terminal.py').write_text(
                'class FirstHoverControl:\n'
                '    def mission_tick(self): pass\n'
                '    def allowed_modes(self): pass\n'
                '    def check_envelope(self): pass\n')
            s=self.manager(['handover-check','--scripts-dir',directory])
            s.config.update(planner_profile='low-speed-3d',planner_config_path='/tmp/profile.yaml')
            s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0',wire_test='0')
            msg=types.ModuleType('mars_quadrotor_msgs.msg');msg.PolynomialTrajectory=lambda:None
            with patch.object(m,'GroundGate',FakeGate),patch.dict('sys.modules',{'mars_quadrotor_msgs.msg':msg}):
                s.mutate('handover-check')
                self.assertNotIn('wire_test',s.window_state)
                self.assertEqual([name for name,_ in s.started],['handover_check'])
                self.assertNotIn('--execute',s.started[0][1])
                self.assertIn('--planner-config',s.started[0][1])
                with self.assertRaises(RuntimeError):s.mutate('handover-check')
                for action in ('goal','wire-test','hover'):
                    with self.assertRaises(RuntimeError):s.mutate(action)
                s.mutate('handover-stop')
            self.assertEqual(FakeGate.writes,[])
            for name in ('lidar','lio','mavros','body','preview','vision','planner','cmd_preview'):
                self.assertEqual(s.window_state[name],'0')

    def test_handover_requires_current_helper_and_no_other_native_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name in ('super_flight_control.py','bounded_reference_preview.py'):(root/name).write_text('# fixture')
            legacy=root/'first_hover_terminal.py';legacy.write_text('class FirstHoverControl: pass\n')
            s=self.manager(['handover-check','--scripts-dir',directory])
            s.config.update(planner_profile='low-speed-3d',planner_config_path='/tmp/profile.yaml')
            s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0')
            with patch.object(m,'GroundGate',FakeGate),self.assertRaisesRegex(RuntimeError,'Update first_hover'):
                s.mutate('handover-check')
            legacy.write_text('class FirstHoverControl:\n'
                '    def mission_tick(self): pass\n    def allowed_modes(self): pass\n'
                '    def check_envelope(self): pass\n')
            FakeGate.publisher_counts={'/mavros/setpoint_raw/local':1}
            with patch.object(m,'GroundGate',FakeGate),self.assertRaisesRegex(RuntimeError,'other native control'):
                s.mutate('handover-check')
            self.assertEqual(s.started,[])

    def test_planner_shutdown_stops_handover_auditor_before_planning(self):
        s=self.manager(['planner-stop'])
        s.window_state.update(handover_check='0',planner='0',cmd_preview='0',preview='0',vision='0')
        with patch.object(m,'GroundGate',FakeGate):s.mutate('planner-stop')
        self.assertEqual([op[1] for op in s.operations if op[0]=='interrupt'],
                         ['handover_check','cmd_preview','planner'])
        self.assertEqual(s.window_state['vision'],'0')

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

    def test_height_goal_requires_3d_preview_and_forwards_ground_only_arguments(self):
        with tempfile.TemporaryDirectory() as directory:
            s=self.manager(['goal','--goal-height','1','--scripts-dir',directory])
            s.window_state.update(preview='0',vision='0',planner='0',cmd_preview='0')
            helper=Path(directory)/'send_ground_height_goal.py';helper.write_text('# fixture')
            with patch.object(m,'GroundGate',FakeGate),patch.object(m,'run') as run:
                with self.assertRaisesRegex(RuntimeError,'Height goal requires'):s.mutate('goal')
                s.config.update(planner_profile='low-speed-3d',planner_config_path='/tmp/3d.yaml')
                with self.assertRaisesRegex(RuntimeError,'Height goal requires'):s.mutate('goal')
                s.window_state['reference_preview']='0';s.mutate('goal')
                self.assertEqual(run.call_args.args[0],['python3','-u',helper,'--height','1.0',
                    '--alignment-path',Path(directory)/'aligned_pose_preview.py','--planner-config','/tmp/3d.yaml'])
            self.assertEqual(s.started,[]);self.assertEqual(FakeGate.writes,[])

    def test_height_goal_arguments_are_bounded_and_goal_only(self):
        for argv in (['goal','--goal-height','nan'],['goal','--goal-height','0.085'],
                     ['goal','--goal-height','1.01'],['hover','--goal-height','1']):
            with self.assertRaises(SystemExit):m.arguments(argv)
        self.assertEqual(m.arguments(['goal','--goal-height','1']).goal_height,1.)

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
