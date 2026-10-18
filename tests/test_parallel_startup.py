"""Real subprocess barriers prove cold-start overlap, failure handling and cleanup."""
import _thread
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/flight'),str(ROOT/'tests')]
import super_session as manager
import super_demo_test as demo
import test_super_demo_test as fixtures
import test_super_ground_test as ground_fixtures


class Tests(unittest.TestCase):
    def command(self,code,*args):return [sys.executable,'-u','-c',code,*map(str,args)]

    def test_both_readonly_collectors_start_before_either_can_finish(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            code='''from pathlib import Path
import sys,time
root=Path(sys.argv[1]);me=sys.argv[2];other=sys.argv[3]
(root/me).touch();deadline=time.monotonic()+3
while not (root/other).exists():
 if time.monotonic()>deadline:raise SystemExit(9)
 time.sleep(.01)
(root/(me+'_done')).touch()
'''
            manager.parallel_observations([self.command(code,root,'a','b'),self.command(code,root,'b','a')])
            self.assertTrue((root/'a_done').exists());self.assertTrue((root/'b_done').exists())

    def test_failed_collector_interrupts_and_reaps_its_peer(self):
        with tempfile.TemporaryDirectory() as directory:
            pid=Path(directory)/'pid'
            slow='''import os,signal,sys,time
from pathlib import Path
signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
Path(sys.argv[1]).write_text(str(os.getpid()))
time.sleep(30)
'''
            failed='''import sys,time
from pathlib import Path
while not Path(sys.argv[1]).exists():time.sleep(.01)
raise SystemExit(7)
'''
            with self.assertRaises(subprocess.CalledProcessError) as error:
                manager.parallel_observations([self.command(slow,pid),self.command(failed,pid)])
            self.assertEqual(error.exception.returncode,7)
            with self.assertRaises(ProcessLookupError):os.kill(int(pid.read_text()),0)

    def test_parent_cancellation_reaps_both_collectors(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            code='''import os,signal,sys,time
from pathlib import Path
signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
Path(sys.argv[1]).write_text(str(os.getpid()))
time.sleep(30)
'''
            def interrupt():
                deadline=time.monotonic()+3
                while len(list(root.iterdir()))<2 and time.monotonic()<deadline:time.sleep(.01)
                _thread.interrupt_main()
            thread=threading.Thread(target=interrupt);thread.start()
            with self.assertRaises(KeyboardInterrupt):
                manager.parallel_observations([self.command(code,root/'a'),self.command(code,root/'b')])
            thread.join()
            for path in root.iterdir():
                with self.assertRaises(ProcessLookupError):os.kill(int(path.read_text()),0)

    def test_alignment_option_is_explicit_and_only_for_alignment(self):
        self.assertTrue(manager.arguments(['align','--parallel-observations']).parallel_observations)
        self.assertFalse(manager.arguments(['align']).parallel_observations)
        with self.assertRaises(SystemExit):manager.arguments(['vision-restart','--parallel-observations'])

    def test_preparation_still_applies_the_current_measured_quaternion(self):
        fixture=ground_fixtures.Tests()
        with tempfile.TemporaryDirectory() as directory:
            runner,calls=fixture.runner(directory);runner.args.fast_start=True
            runner.after_vision_ready=lambda:calls.append(('parallel_vision_started',()))
            runner.prepare(planner_profile='demo')
            self.assertIn('--parallel-observations',calls[1][1])
            self.assertEqual(list(calls[2][1]),ground_fixtures.survey()['quaternion_xyzw'])
            self.assertEqual([call[0] for call in calls],['start','align','vision','parallel_vision_started','planner'])

    def test_vision_observation_overlaps_planner_and_finishes_before_capture(self):
        fixture=fixtures.Tests()
        with tempfile.TemporaryDirectory() as directory,fixture.gate():
            runner,actions=fixture.runner(directory);runner.args.fast_start=True
            started=Path(directory)/'started';planned=Path(directory)/'planned'
            code='''from pathlib import Path
import sys,time
Path(sys.argv[1]).touch()
while not Path(sys.argv[2]).exists():time.sleep(.01)
print('qualified fresh vision window')
'''
            runner.vision_command=lambda:self.command(code,started,planned)
            def prepare(planner_profile):
                actions.append(('prepare',planner_profile));runner.after_vision_ready()
                deadline=time.monotonic()+3
                while not started.exists() and time.monotonic()<deadline:time.sleep(.01)
                self.assertTrue(started.exists());planned.touch()
            runner.prepare=prepare
            step=runner.step
            def check(name,command,env=None):
                if name=='capture_scene':
                    self.assertTrue(runner.startup_vision['finished'])
                    self.assertEqual(runner.startup_vision['stage']['returncode'],0)
                    self.assertTrue((runner.folder/'planner_preparation_ready').is_file())
                step(name,command,env)
            runner.step=check
            self.assertEqual(runner.run_complete(),0)
            self.assertTrue(runner.report['startup_policy']['fresh_alignment'])
            self.assertFalse(runner.report['startup_policy']['cached_scene_used'])
            self.assertEqual([item[0] for item in actions],['prepare','capture_scene','complete_mission'])
            self.assertEqual(runner.startup_vision['stage']['parallel_with'],'planner')

    def test_failed_vision_blocks_capture_and_reaps_active_planner(self):
        fixture=fixtures.Tests()
        with tempfile.TemporaryDirectory() as directory,fixture.gate():
            runner,actions=fixture.runner(directory);runner.args.fast_start=True
            pid=Path(directory)/'planner_pid'
            runner.vision_command=lambda:self.command('import time;time.sleep(.15);raise SystemExit(8)')
            code='''import os,signal,sys,time
from pathlib import Path
signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
Path(sys.argv[1]).write_text(str(os.getpid()))
while True:print('map preparation progress',flush=True);time.sleep(.03)
'''
            def prepare(planner_profile):
                runner.after_vision_ready();demo.DemoTest.step(runner,'planner',self.command(code,pid))
            runner.prepare=prepare
            self.assertEqual(runner.run_complete(),1)
            self.assertEqual(runner.report['failed_stage'],'vision_chain')
            self.assertEqual(runner.startup_vision['stage']['returncode'],8)
            self.assertEqual(actions,[])
            with self.assertRaises(ProcessLookupError):os.kill(int(pid.read_text()),0)

    def test_planner_failure_reaps_background_vision(self):
        fixture=fixtures.Tests()
        with tempfile.TemporaryDirectory() as directory,fixture.gate():
            runner,_=fixture.runner(directory);runner.args.fast_start=True
            runner.vision_command=lambda:self.command('import time;time.sleep(30)')
            def prepare(planner_profile):
                runner.after_vision_ready();raise RuntimeError('planner rejected its configuration')
            runner.prepare=prepare
            self.assertEqual(runner.run_complete(),1)
            self.assertIsNotNone(runner.startup_vision['process'].poll())

    def test_cancelled_preparation_reaps_both_parallel_groups(self):
        fixture=fixtures.Tests()
        with tempfile.TemporaryDirectory() as directory,fixture.gate():
            runner,_=fixture.runner(directory);runner.args.fast_start=True
            root=Path(directory);stop=threading.Event()
            code='''import os,signal,sys,time
from pathlib import Path
signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
Path(sys.argv[1]).write_text(str(os.getpid()))
while True:print('preparation progress',flush=True);time.sleep(.03)
'''
            runner.vision_command=lambda:self.command(code,root/'vision_pid')
            def prepare(planner_profile):
                runner.after_vision_ready();demo.DemoTest.step(runner,'planner',self.command(code,root/'planner_pid'))
            runner.prepare=prepare
            def interrupt():
                deadline=time.monotonic()+3
                while not stop.is_set() and time.monotonic()<deadline:
                    if (root/'vision_pid').exists() and (root/'planner_pid').exists():break
                    stop.wait(.01)
                if not stop.is_set():_thread.interrupt_main()
            thread=threading.Thread(target=interrupt);thread.start()
            try:self.assertEqual(runner.run_complete(),130)
            finally:stop.set();thread.join()
            self.assertTrue(runner.report['cancelled_by_operator'])
            for name in ('vision_pid','planner_pid'):
                with self.assertRaises(ProcessLookupError):os.kill(int((root/name).read_text()),0)


if __name__=='__main__':unittest.main()
