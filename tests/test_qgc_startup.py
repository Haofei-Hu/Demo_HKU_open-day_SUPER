"""QGC opens once before preparation without mode, arming or parameter commands."""
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/maintenance'))
import qgc_startup as m


class Tests(unittest.TestCase):
    def test_existing_instance_is_reused_even_from_a_headless_shell(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{},clear=True),\
                patch.object(m,'running_qgc',return_value=123),patch.object(m.subprocess,'Popen') as spawn:
            self.assertEqual(m.open_qgc(folder)['status'],'already_running');spawn.assert_not_called()

    def test_disabled_headless_and_missing_application_do_not_spawn(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{},clear=True),\
                patch.object(m,'running_qgc',return_value=None),patch.object(m.subprocess,'Popen') as spawn:
            self.assertEqual(m.open_qgc(folder,enabled=False)['status'],'disabled')
            self.assertEqual(m.open_qgc(folder)['status'],'skipped')
            with patch.dict(os.environ,{'DISPLAY':':1'}):
                self.assertEqual(m.open_qgc(folder,Path(folder)/'missing')['status'],'unavailable')
            spawn.assert_not_called()

    def test_real_detached_child_is_found_and_reused_without_second_launch(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'DISPLAY':':1'}):
            app=Path(folder)/'QGroundControl-test.AppImage'
            app.write_text('#!/usr/bin/env python3\nimport time\ntime.sleep(30)\n');app.chmod(0o755)
            with patch.object(m,'running_qgc',return_value=None):status=m.open_qgc(folder,app)
            process=m._launched.pop()
            try:
                self.assertEqual(status['status'],'launched')
                deadline=time.monotonic()+3
                while m.running_qgc()!=process.pid and time.monotonic()<deadline:time.sleep(.01)
                self.assertEqual(m.running_qgc(),process.pid)
                with patch.object(m.subprocess,'Popen') as spawn:
                    self.assertEqual(m.open_qgc(folder,app)['pid'],process.pid);spawn.assert_not_called()
                self.assertFalse(status['connection_verified']);self.assertFalse(status['rendering_verified'])
            finally:
                if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=3)

    def test_launch_error_is_reported_without_retry_or_vehicle_request(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'DISPLAY':':1'}),\
                patch.object(m,'running_qgc',return_value=None),patch.object(m.subprocess,'Popen',side_effect=OSError('failure')) as spawn:
            app=Path(folder)/'qgc';app.touch();app.chmod(0o755)
            status=m.open_qgc(folder,app)
            self.assertEqual(status['status'],'unavailable');self.assertIn('failure',status['reason'])
            self.assertEqual(spawn.call_count,1)


if __name__=='__main__':unittest.main()
