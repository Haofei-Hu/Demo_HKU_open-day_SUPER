import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('ground_check', ROOT/'scripts/analysis/ground_localization_check.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def row(stamp, receipt=None, xyz=(0., 0., 0.), angle=0.):
    return dict(stamp_ns=stamp, received_ros_ns=stamp+20_000_000,
                received_monotonic_ns=receipt if receipt is not None else stamp,
                position=list(xyz), quaternion=[0., 0., math.sin(angle/2), math.cos(angle/2)],
                frame='map', publisher_gid='abc', covariance_diagonal=[1e-6]*6)


class Measurements(unittest.TestCase):
    def test_forward_and_left_use_one_fixed_fc_heading(self):
        base = {t: dict(mean_xyz_m=[1., 2., 3.], mean_yaw_rad=math.pi/2) for t in m.TOPICS}
        forward = {t: dict(mean_xyz_m=[1., 2.5, 3.], mean_yaw_rad=math.pi/2) for t in m.TOPICS}
        result = m.comparison(base, forward, math.pi/2, [.5, 0, 0])
        for t in (m.ALIGNED, m.VISION, m.LOCAL):
            for v in result[t]['measured_minus_manual_forward_left_up_m']:
                self.assertAlmostEqual(v, 0.)
        self.assertNotIn('delta_forward_left_up_m', result[m.RAW])
        left = {t: dict(mean_xyz_m=[.5, 2., 3.], mean_yaw_rad=math.pi/2) for t in m.TOPICS}
        for v in m.comparison(base, left, math.pi/2, [0, .5, 0])[m.LOCAL]['measured_minus_manual_forward_left_up_m']:
            self.assertAlmostEqual(v, 0.)

    def test_source_heading_difference_is_not_rotated_away(self):
        base = {m.VISION: dict(mean_xyz_m=[0, 0, 0], mean_yaw_rad=math.pi/2)}
        moved = {m.VISION: dict(mean_xyz_m=[0, .5, 0], mean_yaw_rad=math.pi/2)}
        result = m.comparison(base, moved, 0., [.5, 0, 0])[m.VISION]
        self.assertEqual(result['measured_minus_manual_forward_left_up_m'], [-.5, .5, 0])

    def test_yaw_wrap_and_translation_during_rotation_retained(self):
        base = {m.LOCAL: dict(mean_xyz_m=[0, 0, 0], mean_yaw_rad=math.radians(170))}
        moved = {m.LOCAL: dict(mean_xyz_m=[.1, 0, 0], mean_yaw_rad=math.radians(-100))}
        result = m.comparison(base, moved, 0., [0, 0, 0], 90)[m.LOCAL]
        self.assertAlmostEqual(result['yaw_change_deg'], 90.)
        self.assertAlmostEqual(result['yaw_change_minus_manual_deg'], 0.)
        self.assertEqual(result['delta_forward_left_up_m'], [.1, 0, 0])

    def test_statistics_keep_stale_and_reordered_samples(self):
        rows = [row(1_000_000_000), row(900_000_000, 2_000_000_000)]
        rows[-1]['received_ros_ns'] = 2_000_000_000
        out = m.statistics(rows)
        self.assertEqual(out['count'], 2)
        self.assertEqual(out['nonincreasing_stamp_count'], 1)
        self.assertEqual(out['header_age_s']['max'], 1.1)
        self.assertEqual(out['receive_gap_max_s'], 1.)

    def test_qos_comparison_excludes_window_edges(self):
        best = [row(s*100, s*100+10) for s in (1, 2, 3, 5, 6)]
        reliable = [row(s*100, s*100+20) for s in (2, 3, 4, 5, 7)]
        result = m.qos_comparison(best, reliable)
        self.assertEqual(result['common_stamps'], 3)
        self.assertEqual(result['reliable_only'], 1)
        self.assertEqual(result['best_effort_only'], 1)
        self.assertAlmostEqual(result['reliable_minus_best_receipt_ms']['mean'], .00001)

    def test_missing_and_invalid_data_remain_explicit(self):
        self.assertEqual(m.statistics([]), dict(count=0))
        invalid = row(0)
        invalid['quaternion'] = [0., 0., 0., 0.]
        report = m.statistics([invalid])
        self.assertEqual(report['invalid_pose_count'], 1)
        self.assertNotIn('mean_xyz_m', report)
        self.assertFalse(m.qos_comparison([], [row(1)])['comparable'])
        self.assertFalse(m.comparison({}, {}, 0.)[m.LOCAL]['available'])
        self.assertIsNone(m.finite_json(dict(age=float('inf')))['age'])


if __name__ == '__main__':
    unittest.main()
