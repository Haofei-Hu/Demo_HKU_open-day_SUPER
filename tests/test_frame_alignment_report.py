"""The survey's machine report describes the same measured quaternion."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('survey',ROOT/'scripts/localization/frame_alignment_check.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class Tests(unittest.TestCase):
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
