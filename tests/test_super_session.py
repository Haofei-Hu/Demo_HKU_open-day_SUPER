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
        if args[0]=='kill-window': self.window_state.pop('hover')


class Tests(unittest.TestCase):
    def setUp(self):
        FakeGate.blocked=FakeGate.stale_preview=False
        FakeGate.values.update(EKF2_EV_CTRL=0,EKF2_HGT_REF=3)
        FakeGate.writes=[]

    def manager(self, argv): return MemorySession(m.arguments(argv))

    def test_invalid_quaternions_and_execution_action(self):
        for argv in (['vision'],['vision','0','0','0','nan'],['vision','0','0','0','2'],
                     ['start','--execute'],['start','--session','bad;touch'],['hover','--hover-height','2']):
            with self.assertRaises(SystemExit): m.arguments(argv)
        self.assertEqual(m.arguments(['vision','0','0','0','1']).quaternion,[0.,0.,0.,1.])

    def test_existing_flight_blocks_mutations(self):
        FakeGate.blocked=True
        for action in ('align','vision','hover','stop'):
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

    def test_commands_keep_argument_boundaries(self):
        args=m.arguments(['start','--scripts-dir',"/tmp/path with spaces/$(touch BAD)"])
        commands=m.Session(args).core_commands()
        import shlex
        body=shlex.split(commands['body'])
        self.assertEqual(body[-1],'/tmp/path with spaces/$(touch BAD)/fastlio_body_debug.py')
        self.assertIn('publish.scan_publish_en:=true',commands['lio'])
        self.assertFalse(any('first_hover' in command for command in commands.values()))


if __name__=='__main__': unittest.main()
