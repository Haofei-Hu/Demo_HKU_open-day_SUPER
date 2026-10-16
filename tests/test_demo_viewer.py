"""Viewer ownership and domain separation without an aircraft or a desktop."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch
import math
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/simulation'))
sys.path.insert(0,str(ROOT/'scripts/localization'))
import demo_mission_simulation as m


class Tests(unittest.TestCase):
    def test_arrow_uses_body_heading_with_mounting_and_world_rotations(self):
        from aligned_pose_preview import multiply,rotation
        world=np.array([.2,-.3,.4,.8]);world/=np.linalg.norm(world)
        mounting=np.array([0.,0.,math.sin(math.pi/8),math.cos(math.pi/8)])
        inverse_mount=mounting*np.array([-1,-1,-1,1])
        raw_attitudes=([0.,0.,0.,1.],[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.])
        for raw in raw_attitudes:
            expected=multiply(raw,inverse_mount)
            aligned=multiply(world,expected)
            actual=m.viewer_body_orientation(rotation(world),aligned)
            self.assertAlmostEqual(abs(np.dot(actual,expected)),1.,places=10)
            np.testing.assert_allclose(rotation(np.array(actual))[:,0],rotation(expected)[:,0],atol=1e-10)

    def environment(self,**extra):
        return patch.dict(os.environ,dict(ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',**extra),clear=True)

    def test_headless_and_disabled_runs_do_not_spawn_and_are_reported(self):
        with tempfile.TemporaryDirectory() as directory,self.environment(),patch.object(m.subprocess,'Popen') as spawn:
            viewer=m.RehearsalViewer(Path(directory)/'report.json');viewer.start(None);viewer.close()
            self.assertEqual(viewer.state['status'],'skipped');spawn.assert_not_called()
            viewer=m.RehearsalViewer(Path(directory)/'report.json',False);viewer.start(None);viewer.close()
            self.assertEqual(viewer.state['status'],'disabled');spawn.assert_not_called()
            self.assertEqual(json.loads((Path(directory)/'rviz_status.json').read_text()),viewer.state)

    def test_actual_aircraft_domain_cannot_launch_viewer(self):
        with tempfile.TemporaryDirectory() as directory,patch.dict(os.environ,{'ROS_DOMAIN_ID':'0'},clear=True),\
                patch.object(m.subprocess,'Popen') as spawn:
            with self.assertRaisesRegex(ValueError,'domain 197'):
                m.RehearsalViewer(Path(directory)/'report.json').start(None)
            spawn.assert_not_called()

    def test_launch_uses_own_group_private_domain_and_raw_world_topics(self):
        with tempfile.TemporaryDirectory() as directory,self.environment(DISPLAY=':1'),\
                patch.object(m.shutil,'which',return_value='/opt/ros/humble/bin/rviz2'),\
                patch.object(m.subprocess,'Popen') as spawn,patch.object(m.os,'killpg') as kill:
            process=spawn.return_value;process.pid=987;process.poll.return_value=None;process.wait.return_value=-15
            viewer=m.RehearsalViewer(Path(directory)/'report.json')
            viewer.start({'raw_position':[1.,2.,3.]})
            args,kwargs=spawn.call_args
            self.assertEqual(args[0][:2],['rviz2','-d']);self.assertTrue(kwargs['start_new_session'])
            self.assertEqual(kwargs['env']['ROS_DOMAIN_ID'],'197')
            self.assertEqual(kwargs['env']['ROS_LOCALHOST_ONLY'],'1')
            config=json.loads(Path(args[0][2]).read_text())['Visualization Manager']
            self.assertEqual(config['Global Options']['Fixed Frame'],'world')
            self.assertEqual(config['Views']['Current']['Focal Point'],dict(X=1.,Y=2.,Z=3.))
            displays=config['Displays'];topics={d['Topic']['Value'] for d in displays}
            self.assertIn('/demo_scene/vehicle_pose',topics);self.assertIn('/demo_scene/occupied_points',topics)
            self.assertNotIn('/mavros/local_position/pose',topics)
            for display in displays:
                self.assertEqual(display['Topic']['Reliability Policy'],'Best Effort')
                if display['Class'].endswith('PointCloud2'):self.assertEqual(display['Style'],'Points')
            viewer.close();kill.assert_called_once_with(987,signal.SIGTERM)
            self.assertEqual(viewer.state['status'],'closed');self.assertFalse(viewer.state['rendering_verified'])

    def test_missing_binary_and_spawn_failure_are_optional_viewer_errors(self):
        with tempfile.TemporaryDirectory() as directory,self.environment(DISPLAY=':1'),\
                patch.object(m.shutil,'which',return_value=None),patch.object(m.subprocess,'Popen') as spawn:
            viewer=m.RehearsalViewer(Path(directory)/'report.json');viewer.start(None);viewer.close()
            self.assertEqual(viewer.state['status'],'skipped');spawn.assert_not_called()
        with tempfile.TemporaryDirectory() as directory,self.environment(WAYLAND_DISPLAY='wayland-0'),\
                patch.object(m.shutil,'which',return_value='rviz2'),\
                patch.object(m.subprocess,'Popen',side_effect=OSError('cannot start')):
            viewer=m.RehearsalViewer(Path(directory)/'report.json');viewer.start(None);viewer.close()
            self.assertEqual(viewer.state['status'],'unavailable');self.assertIsNone(viewer.log)

    def test_exited_viewer_is_not_signalled_or_treated_as_mission_success(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(m.os,'killpg') as kill:
            viewer=m.RehearsalViewer(Path(directory)/'report.json')
            viewer.process=Mock();viewer.process.poll.return_value=1;viewer.close()
            self.assertEqual(viewer.state['status'],'exited');kill.assert_not_called()
            self.assertEqual(viewer.state['returncode'],1)

    def test_shutdown_timeout_kills_only_owned_group(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(m.os,'killpg') as kill:
            viewer=m.RehearsalViewer(Path(directory)/'report.json')
            viewer.process=Mock(pid=987);viewer.process.poll.return_value=None
            viewer.process.wait.side_effect=[subprocess.TimeoutExpired('rviz2',3),-9]
            viewer.close()
            self.assertEqual([call.args for call in kill.call_args_list],[(987,signal.SIGTERM),(987,signal.SIGKILL)])
            self.assertEqual(viewer.state['status'],'closed')

    def test_real_owned_child_is_reaped_without_signalling_other_processes(self):
        real_spawn=subprocess.Popen
        with tempfile.TemporaryDirectory() as directory,self.environment(DISPLAY=':1'),\
                patch.object(m.shutil,'which',return_value='rviz2'),\
                patch.object(m.subprocess,'Popen',side_effect=lambda *a,**kw:
                    real_spawn([sys.executable,'-c','import time; time.sleep(30)'],**kw)):
            viewer=m.RehearsalViewer(Path(directory)/'report.json');viewer.start(None)
            process=viewer.process
            try:viewer.close()
            finally:
                if process.poll() is None:process.kill();process.wait(timeout=3)
            self.assertIsNotNone(process.returncode)
            self.assertEqual(viewer.state['status'],'closed')


if __name__=='__main__':unittest.main()
