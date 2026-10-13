"""Preserve motion/map settings while disabling fixed goal Z in a separate file."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('three_d',ROOT/'scripts/maintenance/create_super_3d_goal_profile.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
TEXT=(ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text().replace('max_vel: 5.0','max_vel: 0.25').replace('max_acc: 5.0','max_acc: 0.45')


class Tests(unittest.TestCase):
    def test_only_goal_height_override_changes_and_generation_is_repeatable(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'mid360_low_speed_reserved_ros2.yaml';source.write_text(TEXT)
            target=m.create(source)
            expected=yaml.safe_load(TEXT);expected['fsm']['click_height']=-6.
            self.assertEqual(yaml.safe_load(target.read_text()),expected)
            self.assertEqual(source.read_text(),TEXT)
            self.assertIn('# Virtual ceil and ground',target.read_text())
            before=target.stat().st_mtime_ns;m.create(source)
            self.assertEqual(before,target.stat().st_mtime_ns)

    def test_existing_different_profile_blocks_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'mid360_low_speed_reserved_ros2.yaml';source.write_text(TEXT)
            target=source.with_name('mid360_low_speed_3d_goal_ros2.yaml');target.write_text('# local profile')
            with self.assertRaisesRegex(ValueError,'no files written'):m.create(source)
            self.assertEqual(target.read_text(),'# local profile')
            self.assertEqual(source.read_text(),TEXT)
            self.assertFalse(list(source.parent.glob('three_d_goal_source_*')))

    def test_wrong_limits_or_invalid_duplicate_height_block_writes(self):
        for text in (TEXT.replace('max_vel: 0.25','max_vel: 2.0'),
                     TEXT.replace('click_height: -0.05','click_height: .nan'),
                     TEXT.replace('click_height: -0.05','click_height: false'),
                     TEXT.replace('click_height: -0.05','click_height: -0.05\n  click_height: -0.05')):
            with tempfile.TemporaryDirectory() as directory:
                source=Path(directory)/'source.yaml';source.write_text(text)
                with self.assertRaises(ValueError):m.create(source)
                self.assertEqual(source.read_text(),text)
                self.assertEqual(list(source.parent.iterdir()),[source])


if __name__=='__main__':unittest.main()
