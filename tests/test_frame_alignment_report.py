"""The survey's machine report describes the same measured quaternion."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('survey',ROOT/'scripts/localization/frame_alignment_check.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class Tests(unittest.TestCase):
    def test_old_and_future_body_samples_are_excluded_with_original_age_evidence(self):
        samples=m.FreshBodySamples();now=100_000_000_000
        def pose(ns):
            sec,nanosec=divmod(ns,1_000_000_000)
            return NS(header=NS(stamp=NS(sec=sec,nanosec=nanosec),frame_id='camera_init'),
                      pose=NS(orientation=NS(x=0.,y=0.,z=0.,w=1.)))
        for ns in (now-302_000_000,now+51_000_000,0):
            self.assertIsNone(samples.receive(pose(ns),now))
        for ns in (now-250_000_000,now+50_000_000,now-17_000_000):
            accepted=samples.receive(pose(ns),now)
            self.assertIsNotNone(accepted)
            self.assertEqual(accepted[0],m.stamp(pose(ns)))
            self.assertAlmostEqual(accepted[2],(now-ns)/1e9)
        self.assertEqual(samples.evidence()['body_age_rejected'],3)
        self.assertEqual(samples.evidence()['observed_body_age_max_s'],100.)

    def test_insufficient_pairs_do_not_expose_a_candidate(self):
        report=m.summarize([],[],[],[100,200])
        self.assertFalse(report['estimate_available'])
        self.assertNotIn('quaternion_xyzw',report)

    def test_antipodal_samples_recover_the_measured_rotation(self):
        q=np.array([.01,-.02,.7,.71]);q/=np.linalg.norm(q)
        rotations=[q if i%2 else -q for i in range(100)]
        report=m.summarize(rotations,[.01]*100,[.02]*100,[100,500])
        self.assertTrue(report['estimate_available'])
        self.assertAlmostEqual(abs(np.dot(q,report['quaternion_xyzw'])),1.)
        self.assertLess(report['rotation_spread_max_deg'],.00001)
        self.assertEqual(report['body_age_max_s'],.02)
        self.assertEqual(report['matched'],100)


if __name__=='__main__':unittest.main()
