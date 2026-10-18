"""Real SIGINT delivery through the outer ROS bootstrap, with no aircraft/ROS."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/flight'),str(ROOT/'tests')]
from super_demo_test import run_ros_bootstrap
import test_super_demo_test as runner_fixture


class Tests(unittest.TestCase):
    def test_normal_exit_status_is_preserved(self):
        previous={sig:signal.getsignal(sig) for sig in (signal.SIGHUP,signal.SIGTERM)}
        self.assertEqual(run_ros_bootstrap([sys.executable,'-c','raise SystemExit(7)']),7)
        self.assertEqual({sig:signal.getsignal(sig) for sig in previous},previous)

    def test_terminal_signals_only_owned_child_once_and_waits_for_report(self):
        for signum in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            with self.subTest(signal=signum):self.signal_trial(signum)

    def signal_trial(self,signum):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);ready=folder/'ready';report=folder/'report.json';parent_ready=folder/'parent_ready'
            child='''import json,signal,sys,time
from pathlib import Path
count=0
def cancel(*args):
 global count
 count+=1
signal.signal(signal.SIGINT,cancel)
Path(sys.argv[1]).write_text(str(__import__('os').getpid()))
while not count:time.sleep(.01)
time.sleep(.4)
Path(sys.argv[2]).write_text(json.dumps({'interrupts':count,'cleanup_finished':True}))
raise SystemExit(130)
'''
            wrapper='''import signal,sys
from pathlib import Path
from super_demo_test import run_ros_bootstrap
Path(sys.argv[1]).write_text('ready')
raise SystemExit(run_ros_bootstrap([sys.executable,'-c',sys.argv[2],sys.argv[3],sys.argv[4]]))
'''
            env=dict(os.environ)
            extra=[str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/flight'),str(ROOT/'scripts/planning'),str(ROOT)]
            env['PYTHONPATH']=os.pathsep.join(extra+[env.get('PYTHONPATH','')])
            unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True)
            process=subprocess.Popen([sys.executable,'-c',wrapper,str(parent_ready),child,str(ready),str(report)],
                env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
            try:
                deadline=time.monotonic()+8
                while not ready.exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.01)
                self.assertTrue(ready.exists())
                os.killpg(process.pid,signum)
                time.sleep(.1);os.killpg(process.pid,signum)
                output,_=process.communicate(timeout=8)
                self.assertEqual(process.returncode,128+signum,output)
                self.assertNotIn('Traceback',output)
                self.assertEqual(output.count('CANCEL:'),1)
                self.assertEqual(json.loads(report.read_text()),{'interrupts':1,'cleanup_finished':True})
                self.assertIsNone(unrelated.poll())
            finally:
                if process.poll() is None:process.kill();process.wait(timeout=3)
                if ready.exists():
                    try:os.kill(int(ready.read_text()),signal.SIGKILL)
                    except ProcessLookupError:pass
                unrelated.terminate();unrelated.wait(timeout=3)

    def test_operator_cancelled_run_is_saved_as_incomplete_without_traceback(self):
        with tempfile.TemporaryDirectory() as folder:
            runner,_=runner_fixture.Tests().runner(folder,logic=True)
            def cancel(*args,**kwargs):raise KeyboardInterrupt()
            runner.step=cancel
            self.assertEqual(runner.run_complete(),130)
            report=json.loads(runner.path.read_text())
            self.assertEqual(report['result'],'FAIL')
            self.assertTrue(report['cancelled_by_operator'])
            self.assertFalse(report['demonstration_completed'])
            self.assertEqual(report['error'],'Operator cancelled the ground rehearsal (Ctrl+C)')


if __name__=='__main__':unittest.main()
