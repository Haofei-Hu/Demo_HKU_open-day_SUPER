"""Only the private static-snapshot inputs change; physical configuration stays."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/simulation'))
from demo_mission_simulation import rehearsal_profile


class Tests(unittest.TestCase):
    def test_motion_geometry_bounds_and_original_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'original.yaml'
            cfg=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
            cfg['traj_opt']['boundary'].update(max_vel=2.,max_acc=1.)
            source.write_text(yaml.safe_dump(cfg));original=source.read_bytes()
            target=rehearsal_profile(source,Path(folder)/'report.json')
            expected=deepcopy(cfg)
            expected['rog_map']['ros_callback']['cloud_topic']='/demo_scene/occupied_points'
            expected['rog_map']['raycasting']['enable']=False
            self.assertEqual(yaml.safe_load(target.read_text()),expected)
            self.assertEqual(source.read_bytes(),original)
            with self.assertRaises(ValueError):rehearsal_profile(source,Path(folder)/'report.json')


if __name__=='__main__':unittest.main()
