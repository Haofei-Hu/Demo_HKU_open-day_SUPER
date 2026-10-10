#!/usr/bin/env python3
"""Check deployable directory layout without ROS or aircraft access."""
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('install_scripts', ROOT / 'scripts/maintenance/install_scripts.py')
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class LayoutTests(unittest.TestCase):
    def test_catalog_covers_sources_and_stable_urls(self):
        layout = json.loads((ROOT / 'scripts/layout.json').read_text())
        expected = {p.relative_to(ROOT).as_posix() for p in ROOT.glob('*.py')}
        expected |= {p.relative_to(ROOT).as_posix() for p in (ROOT / 'scripts').glob('*/*.py')
                     if p.name != 'install_scripts.py'}
        self.assertEqual(expected, set(layout.values()))
        for name in ('first_hover_terminal.py', 'odom_age_check.py', 'update_preview_alignment.py'):
            self.assertEqual(layout[name], name)
        self.assertEqual(len(layout), len(set(layout)))

    def test_flat_install_preserves_session_by_default_and_backups_updates(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'Downloads'
            destination.mkdir()
            old = b'# measured active session\nQ_WC = [0, 0, 0, 1]\n'
            target = destination / 'aligned_pose_preview.py'
            target.write_bytes(old)
            written, retained, backup = installer.install(destination)
            self.assertIn('aligned_pose_preview.py', retained)
            self.assertEqual(target.read_bytes(), old)
            self.assertIsNone(backup)
            for name, data in installer.sources().items():
                if name != target.name:
                    self.assertEqual((destination / name).read_bytes(), data)
            written, retained, backup = installer.install(destination, overwrite=True)
            self.assertEqual((backup / target.name).read_bytes(), old)
            self.assertEqual(target.read_bytes(), installer.sources()[target.name])
            self.assertNotIn('build_first_hover_terminal.py', written)

    def test_invalid_catalog_source_stops_before_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'repo'
            (root / 'scripts').mkdir(parents=True)
            (root / 'scripts/layout.json').write_text('{"broken.py": "broken.py"}')
            (root / 'broken.py').write_text('def syntax error')
            destination = Path(directory) / 'Downloads'
            with self.assertRaises(SyntaxError):
                installer.install(destination, root=root)
            self.assertFalse(destination.exists())


if __name__ == '__main__':
    unittest.main()
