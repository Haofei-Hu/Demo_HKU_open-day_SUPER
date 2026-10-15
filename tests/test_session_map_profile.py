"""Session-coordinate translation, immutable anchors and unchanged constraints."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('session_map',ROOT/'scripts/maintenance/create_session_map_profile.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def configuration():
    cfg=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
    cfg['rog_map']['virtual_ground_height']=-1.5
    cfg['fsm']['click_height']=-6.
    cfg['traj_opt']['boundary'].update(max_vel=.25,max_acc=.45)
    return cfg


class Tests(unittest.TestCase):
    def test_capture_diagnostics_distinguish_excursion_from_net_change(self):
        span,ranges,change=m.capture_summary([
            (10.,0.,0.,-1.),(11.,.04,-.03,-.99),(13.,.01,-.01,-1.)])
        self.assertEqual(span,3.)
        for actual,expected in zip(ranges,(.04,.03,.01)):
            self.assertAlmostEqual(actual,expected)
        for actual,expected in zip(change,(.01,-.01,0.)):
            self.assertAlmostEqual(actual,expected)

    def test_recorded_below_ground_case_and_translated_effective_boundaries(self):
        cfg=configuration();original=deepcopy(cfg)
        self.assertAlmostEqual(m.effective_bounds(cfg)[0],-1.1)
        self.assertLess(-1.1827157092876552,m.effective_bounds(cfg)[0])
        updated,shift,bounds=m.translate(cfg,-1.1827157092876552)
        self.assertAlmostEqual(shift,-1.2)
        self.assertAlmostEqual(updated['rog_map']['virtual_ground_height'],-2.7)
        self.assertAlmostEqual(updated['rog_map']['virtual_ceil_height'],2.3)
        self.assertAlmostEqual(updated['rog_map']['fix_map_origin'][2],.3)
        self.assertAlmostEqual(bounds[0],-2.3);self.assertAlmostEqual(bounds[1],1.7)
        self.assertEqual(cfg,original)
        updated['rog_map']['virtual_ground_height']=original['rog_map']['virtual_ground_height']
        updated['rog_map']['virtual_ceil_height']=original['rog_map']['virtual_ceil_height']
        updated['rog_map']['fix_map_origin'][2]=original['rog_map']['fix_map_origin'][2]
        self.assertEqual(updated,original,'An unrelated configuration field changed')

    def test_vertical_span_and_corridor_clearance_across_session_origins(self):
        cfg=configuration();floor,ceiling,grid=m.effective_bounds(cfg)
        for z in (-5.7,-1.1827,-.1,0.,.1,1.18,4.01):
            updated,shift,bounds=m.translate(cfg,z)
            self.assertAlmostEqual(bounds[1]-bounds[0],ceiling-floor)
            self.assertGreater(z-bounds[0],cfg['super_planner']['corridor_bound_dis'])
            self.assertGreater(bounds[1]-z,cfg['super_planner']['corridor_bound_dis'])
            self.assertAlmostEqual(shift/grid,round(shift/grid))

    def test_creation_is_idempotent_and_source_remains_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'base.yaml';target=Path(folder)/'session.yaml';record=Path(folder)/'anchor.json'
            source.write_text(yaml.safe_dump(configuration()))
            original=source.read_bytes()
            obs=dict(raw_z=-1.1827,raw_source_gid='source-a')
            m.create(source,target,record,obs,'alignment-a')
            created=target.read_bytes();anchor=record.read_bytes()
            m.create(source,target,record,dict(obs,raw_z=-1.17),'alignment-a')
            self.assertEqual(target.read_bytes(),created)
            self.assertEqual(record.read_bytes(),anchor)
            self.assertEqual(source.read_bytes(),original)
            for change,digest in ((dict(obs,raw_z=-1.4),'alignment-a'),
                                  (dict(obs,raw_source_gid='source-b'),'alignment-a'),
                                  (obs,'alignment-b')):
                with self.assertRaises(ValueError):m.create(source,target,record,change,digest)
                self.assertEqual(target.read_bytes(),created)
                self.assertEqual(record.read_bytes(),anchor)

    def test_refuses_source_overwrite_and_conflicting_or_symlink_outputs(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'base.yaml';target=Path(folder)/'session.yaml';record=Path(folder)/'anchor.json'
            source.write_text(yaml.safe_dump(configuration()));original=source.read_bytes()
            obs=dict(raw_z=-1.18,raw_source_gid='a')
            with self.assertRaises(ValueError):m.create(source,source,record,obs,'hash')
            target.write_text('unrelated config')
            with self.assertRaises(ValueError):m.create(source,target,record,obs,'hash')
            self.assertFalse(record.exists());self.assertEqual(target.read_text(),'unrelated config')
            target.unlink();target.symlink_to(source)
            with self.assertRaises(ValueError):m.create(source,target,record,obs,'hash')
            self.assertEqual(source.read_bytes(),original)


if __name__=='__main__':unittest.main()
