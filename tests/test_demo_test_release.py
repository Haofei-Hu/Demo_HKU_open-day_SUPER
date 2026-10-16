"""Atomic prevalidation and preservation of measured deployment files."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('demo_install',ROOT/'scripts/maintenance/install_demo_test.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class Tests(unittest.TestCase):
    def release(self):
        manifest=json.loads((ROOT/'configs/demonstration/demo_test_release.json').read_text())
        def fetch(path):return (ROOT/path).read_bytes()
        return manifest,fetch

    def test_manifest_matches_every_pinned_file(self):
        manifest,fetch=self.release()
        for name,entry in manifest['files'].items():
            self.assertEqual(hashlib.sha256(fetch(entry['path'])).hexdigest(),entry['sha256'],name)
        self.assertIn('super_demo_test.py',manifest['files'])
        self.assertTrue(manifest['files']['open_day_mission.json']['preserve_existing'])

    def test_update_retains_measurements_and_backs_up_changed_code(self):
        _,fetch=self.release()
        with tempfile.TemporaryDirectory() as folder:
            destination=Path(folder);script=destination/'super_demo_test.py';script.write_text('# previous\n')
            mission=destination/'open_day_mission.json';mission.write_text('{"measured":true}\n')
            alignment=destination/'aligned_pose_preview.py';alignment.write_text('# measured quaternion\n')
            backup=m.install('a'*40,destination,fetch)
            self.assertEqual((backup/'previous_super_demo_test.py').read_text(),'# previous\n')
            self.assertEqual(mission.read_text(),'{"measured":true}\n')
            self.assertEqual(alignment.read_text(),'# measured quaternion\n')

    def test_corrupt_download_replaces_nothing(self):
        _,fetch=self.release()
        with tempfile.TemporaryDirectory() as folder:
            destination=Path(folder);target=destination/'super_demo_test.py';target.write_text('# keep\n')
            def corrupt(path):return b'corrupt' if path.endswith('super_demo_control.py') else fetch(path)
            with self.assertRaisesRegex(ValueError,'Checksum mismatch'):m.install('b'*40,destination,corrupt)
            self.assertEqual(target.read_text(),'# keep\n')
            self.assertFalse((destination/'demo_mission.py').exists())


if __name__=='__main__':unittest.main()
