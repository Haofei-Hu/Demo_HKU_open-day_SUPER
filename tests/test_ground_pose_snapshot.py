"""Numerical checks for timed stationary diagnostics without ROS."""
import importlib.util
import math
from pathlib import Path
import unittest

path=Path(__file__).resolve().parents[1]/'scripts/analysis/ground_pose_snapshot.py'
spec=importlib.util.spec_from_file_location('ground_snapshot',path)
snapshot=importlib.util.module_from_spec(spec)
spec.loader.exec_module(snapshot)


class SnapshotTests(unittest.TestCase):
    def test_transient_excursion_is_reported_despite_equal_end_windows(self):
        rows=[(0.,0.,.085,0.,.01,float(t)) for t in range(11)]
        rows[5]=(.12,-.03,.125,0.,.02,5.)
        r=snapshot.summarize(rows)
        self.assertEqual(r['delta_xyz'],[0.,0.,0.])
        for actual,expected in zip(r['max_forward_left_up'],[.12,.03,.04]):
            self.assertAlmostEqual(actual,expected)
        self.assertEqual(r['span'],10.)
        self.assertEqual(r['receive_gap_max'],1.)

    def test_fixed_yaw_axes_and_initial_mean(self):
        rows=[(1.,2.+.01*t,3.,math.pi/2,.01,float(t)) for t in range(11)]
        r=snapshot.summarize(rows)
        self.assertAlmostEqual(r['initial_xyz'][1],2.01)
        self.assertAlmostEqual(r['final_xyz'][1],2.09)
        self.assertAlmostEqual(r['delta_xyz'][1],.08)
        self.assertAlmostEqual(r['max_forward_left_up'][0],.09)
        self.assertAlmostEqual(r['max_forward_left_up'][1],0.)

    def test_wraparound_yaw_and_missing_samples(self):
        rows=[(0.,0.,0.,math.radians(179 if t%2==0 else -179),.01,float(t)) for t in range(8)]
        r=snapshot.summarize(rows)
        self.assertAlmostEqual(abs(r['mean_yaw']),math.pi)
        self.assertGreater(abs(r['axis_yaw']),3.)
        for rows in ([],[(0.,0.,0.,0.,.01,0.)]):
            with self.assertRaises(ValueError):snapshot.summarize(rows)


if __name__=='__main__':unittest.main()
