"""Nominal external-IMU coordinates, rollback and timestamp diagnostics."""
import ast
from copy import deepcopy
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/maintenance'))
import configure_lio_imu as m
from px4_imu_input_check import Observation


class Tests(unittest.TestCase):
    def files(self,p):
        for name in m.FILES:shutil.copy2(ROOT/'scripts/localization'/name,p/name)
        source=p/'source.yaml'
        shutil.copy2(ROOT/'configs/observed_2026-10-13/fastlio_mid360_baseline.yaml',source)
        return source

    def test_profile_changes_only_sensor_selection_and_rigid_extrinsics(self):
        original=yaml.safe_load((ROOT/'configs/observed_2026-10-13/fastlio_mid360_baseline.yaml').read_text())
        saved=deepcopy(original);changed=m.px4_config(original);p=m.parameters(changed)
        self.assertEqual(original,saved)
        self.assertEqual(p['common']['imu_topic'],'/mavros/imu/data_raw')
        R=np.array(p['mapping']['extrinsic_R']).reshape(3,3)
        np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-15)
        np.testing.assert_allclose(R@[1,0,0],[2**-.5,2**-.5,0])
        np.testing.assert_allclose(p['mapping']['extrinsic_T'],[0,0,.1])
        self.assertFalse(p['mapping']['extrinsic_est_en'])
        for group,keys in [('common',['imu_topic','time_sync_en','time_offset_lidar_to_imu']),
                           ('mapping',['extrinsic_R','extrinsic_T','extrinsic_est_en'])]:
            for key in keys:p[group][key]=m.parameters(original)[group][key]
        self.assertEqual(changed,original)

    def test_switch_restores_mount_but_preserves_new_world_alignment(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);source=self.files(p);before={n:(p/n).read_text() for n in m.FILES};source_bytes=source.read_bytes()
            record=m.configure(p,'px4',p/'px4.json',source,p/'px4.yaml');m.verify_mount(p,'px4')
            self.assertFalse(record['calibrated']);self.assertFalse(record['fc_parameters_written'])
            for n in m.FILES:
                original=ast.parse(before[n]);changed=ast.parse((p/n).read_text())
                def strip(tree):
                    return ast.dump(ast.Module(body=[x for x in tree.body if not (isinstance(x,ast.Assign) and
                        any(isinstance(t,ast.Name) and t.id in m.FIELDS for t in x.targets))],type_ignores=[]))
                self.assertEqual(strip(original),strip(changed))
            alignment=p/'aligned_pose_preview.py'
            alignment.write_text(alignment.read_text()+'\n# a later session alignment was retained\n')
            m.configure(p,'livox',p/'livox.json');m.verify_mount(p,'livox')
            for n in m.FILES:self.assertTrue(m.equal_expressions(m.expressions((p/n).read_text()),m.expressions(before[n])))
            self.assertIn('later session alignment',alignment.read_text())
            self.assertEqual(source.read_bytes(),source_bytes)

    def test_px4_conversion_does_not_apply_livox_yaw_or_lever_twice(self):
        # Execute only numpy/math definitions and pure transform functions, no ROS.
        import math
        source=(ROOT/'scripts/localization/fastlio_body_debug.py').read_text()
        source=m.rewrite(source,m.PX4_MOUNT,'body.py')
        tree=ast.parse(source)
        env={'np':np,'math':math}
        definitions=[n for n in tree.body if isinstance(n,(ast.Assign,ast.FunctionDef))]
        exec(compile(ast.Module(body=definitions,type_ignores=[]),'body.py','exec'),env)
        q=np.array([0.,0.,np.sin(.4),np.cos(.4)]);pos=np.array([1.,2.,3.])
        actual_p,actual_q=env['transform'](pos,q)
        np.testing.assert_allclose(actual_p,pos);np.testing.assert_allclose(actual_q,q)

    def test_aligned_px4_covariance_has_world_rotation_without_old_lever(self):
        source=(ROOT/'scripts/localization/aligned_pose_preview.py').read_text()
        env={'__name__':'test_converter'}
        exec(compile(m.rewrite(source,m.PX4_MOUNT,'aligned.py'),'aligned.py','exec'),env)
        point=np.array([1.,2.,3.]);q=np.array([0.,0.,0.,1.]);cov=np.diag([1.,2.,3.,.1,.2,.3])
        pos,ori,actual=env['convert'](point,q,cov)
        np.testing.assert_allclose(pos,env['R_WC']@point)
        np.testing.assert_allclose(ori,env['Q_WC'])
        np.testing.assert_allclose(actual,env['AXES']@cov@env['AXES'].T)

    def test_installer_reset_is_reconciled_but_unknown_mount_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);source=self.files(p)
            m.configure(p,'px4',p/'first.json',source,p/'first.yaml')
            shutil.copy2(ROOT/'scripts/localization/fastlio_body_debug.py',p/'fastlio_body_debug.py')
            with self.assertRaises(ValueError):m.verify_mount(p,'px4')
            m.configure(p,'px4',p/'second.json',source,p/'second.yaml');m.verify_mount(p,'px4')
            f=p/'fastlio_body_debug.py';f.write_text(m.rewrite(f.read_text(),dict(m.PX4_MOUNT,T_BI='np.array([1.,0.,0.])'),f))
            with self.assertRaisesRegex(ValueError,'Custom mounting'):m.configure(p,'livox',p/'third.json')

    def test_failed_record_restores_scripts_state_and_removes_created_profile(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);source=self.files(p);before={n:(p/n).read_bytes() for n in m.FILES}
            with self.assertRaises(FileNotFoundError):m.configure(p,'px4',p/'absent'/'record.json',source,p/'new.yaml')
            self.assertFalse((p/'new.yaml').exists());self.assertFalse((p/m.STATE).exists())
            for n in m.FILES:self.assertEqual((p/n).read_bytes(),before[n])

    def test_timestamp_and_receive_gaps_are_separate(self):
        o=Observation()
        for i,receive in enumerate([0.,.02,.10,.101]):
            stamp=10**9+i*20_000_000
            o.add(stamp,receive,stamp+15_000_000,'base_link',(0.,0.,9.8),(0.,0.,0.))
        r=o.report({'a'});self.assertEqual(r['result'],'SUCCESS')
        self.assertAlmostEqual(r['original_stamp_rate_hz'],50)
        self.assertAlmostEqual(r['original_stamp_gap_s']['max'],.02)
        self.assertAlmostEqual(r['receive_gap_s']['max'],.08)
        self.assertFalse(r['flight_readiness_assessed'])
        o.add(stamp,.12,stamp,'base_link',(0.,0.,9.8),(0.,0.,0.))
        self.assertEqual(o.report({'a'})['result'],'FAIL')

    def test_invalid_vectors_missing_sources_wrong_frames_fail_input_check(self):
        o=Observation();o.add(1,0,1,'base_link',(float('nan'),0,0),(0,0,0))
        self.assertEqual(o.report(set())['result'],'FAIL')
        o=Observation()
        for i in (1,2):o.add(i,float(i),i,'map',(0,0,9.8),(0,0,0))
        self.assertEqual(o.report({'a'})['result'],'FAIL')
        self.assertEqual(o.report({'a','b'})['result'],'FAIL')

if __name__=='__main__':unittest.main()
