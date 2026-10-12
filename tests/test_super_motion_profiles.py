"""Candidate isolation, repeatability and refusal of conflicting configuration."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('profiles', ROOT/'scripts/maintenance/create_super_motion_profiles.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
SOURCE = (ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text()


class Tests(unittest.TestCase):
    def test_candidates_change_only_motion_limits_preserving_source_and_comments(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'mid360_ground_diagnostic_ros2.yaml'
            source.write_text(SOURCE)
            paths = m.create(source)
            for path in paths:
                actual = yaml.safe_load(path.read_text())
                original = yaml.safe_load(SOURCE)
                velocity, acceleration = m.PROFILES[path.name]
                self.assertEqual(actual['traj_opt']['boundary']['max_vel'], velocity)
                self.assertEqual(actual['traj_opt']['boundary']['max_acc'], acceleration)
                actual['traj_opt']['boundary'].update(max_vel=5., max_acc=5.)
                self.assertEqual(actual, original)
                self.assertIn('# Virtual ceil and ground', path.read_text())
            self.assertEqual(source.read_text(), SOURCE)
            before = {p:p.stat().st_mtime_ns for p in paths}
            m.create(source)
            self.assertEqual(before, {p:p.stat().st_mtime_ns for p in paths})

    def test_conflicting_demo_candidate_blocks_all_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'mid360_ground_diagnostic_ros2.yaml'
            source.write_text(SOURCE)
            conflicting = source.with_name('mid360_demo_candidate_ros2.yaml')
            conflicting.write_text('# retain existing configuration')
            with self.assertRaisesRegex(ValueError, 'no files written'):
                m.create(source)
            self.assertFalse(source.with_name('mid360_low_speed_preview_ros2.yaml').exists())
            self.assertFalse(source.with_name('mid360_low_speed_margin_ros2.yaml').exists())
            self.assertFalse(source.with_name('mid360_low_speed_reserved_ros2.yaml').exists())
            self.assertEqual(conflicting.read_text(), '# retain existing configuration')
            self.assertEqual(source.read_text(), SOURCE)

    def test_invalid_or_duplicate_motion_limit_is_rejected(self):
        for text in (SOURCE.replace('max_vel: 5.0', 'max_vel: .nan'),
                     SOURCE.replace('max_acc: 5.0', 'max_acc: true'),
                     SOURCE.replace('max_vel: 5.0', 'max_vel: 5.0\n    max_vel: 4.0')):
            with self.assertRaises(ValueError):
                m.candidate(text, .3, .5)


if __name__ == '__main__':
    unittest.main()
