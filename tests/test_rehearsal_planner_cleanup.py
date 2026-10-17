"""Real process groups: wrapper exit must not leave a virtual planner child."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/simulation'))
from demo_mission_simulation import planner_group_active,stop_owned_planner


class Tests(unittest.TestCase):
    def test_exited_wrapper_child_is_stopped_without_signalling_unrelated_child(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'child_pid'
            code='''import subprocess,sys
from pathlib import Path
child=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"])
Path(sys.argv[1]).write_text(str(child.pid))
'''
            unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True)
            wrapper=subprocess.Popen([sys.executable,'-c',code,str(path)],start_new_session=True,
                                     stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            try:
                self.assertEqual(wrapper.wait(timeout=5),0)
                self.assertTrue(planner_group_active(wrapper.pid))
                stop_owned_planner(wrapper)
                self.assertFalse(planner_group_active(wrapper.pid))
                self.assertIsNone(unrelated.poll())
            finally:
                if planner_group_active(wrapper.pid):os.killpg(wrapper.pid,signal.SIGKILL)
                wrapper.wait(timeout=3)
                unrelated.terminate();unrelated.wait(timeout=3)

    def test_live_owned_planner_exits_and_original_sigint_handler_is_restored(self):
        process=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True,
                                 stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        previous=signal.getsignal(signal.SIGINT)
        try:
            stop_owned_planner(process)
            self.assertIsNotNone(process.poll())
            self.assertEqual(signal.getsignal(signal.SIGINT),previous)
        finally:
            if process.poll() is None:process.kill();process.wait(timeout=3)

    def test_caller_group_is_never_signalled(self):
        with self.assertRaisesRegex(ValueError,'caller process group'):
            stop_owned_planner(Mock(pid=os.getpgrp()))
        stop_owned_planner(None)


if __name__=='__main__':unittest.main()
