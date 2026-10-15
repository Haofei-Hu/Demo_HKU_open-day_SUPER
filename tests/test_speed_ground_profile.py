"""A separate 2/1 profile preserves low-speed settings and physical geometry."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/maintenance'))
from create_super_speed_ground_profile import create


class Tests(unittest.TestCase):
    def test_only_motion_limits_change_and_repeat_is_identical(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'low.yaml'
            cfg=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
            cfg['fsm']['click_height']=-6.
            cfg['traj_opt']['boundary'].update(max_vel=.25,max_acc=.45)
            source.write_text(yaml.safe_dump(cfg));before=source.read_bytes()
            target=create(source);first=target.read_bytes()
            expected=deepcopy(cfg);expected['traj_opt']['boundary'].update(max_vel=2.,max_acc=1.)
            self.assertEqual(yaml.safe_load(first),expected)
            self.assertEqual(source.read_bytes(),before)
            self.assertEqual(create(source),target)
            self.assertEqual(target.read_bytes(),first)
            target.write_text('unrelated config')
            with self.assertRaises(ValueError):create(source)
            self.assertEqual(target.read_text(),'unrelated config')

    def test_wrong_source_is_rejected_without_a_new_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'low.yaml'
            source.write_text('fsm: {click_height: -0.05}\ntraj_opt: {boundary: {max_vel: 0.25, max_acc: 0.45}}\n')
            with self.assertRaises(ValueError):create(source)
            self.assertEqual(list(Path(directory).glob('*')), [source])


if __name__=='__main__':unittest.main()
