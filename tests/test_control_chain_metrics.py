"""Motion norms, invalid references and freshness failures without ROS."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('metrics',ROOT/'scripts/analysis/control_chain_age_check.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Tests(unittest.TestCase):
    def motion(self, velocity=(.18,.24,0.), acceleration=(.3,.4,0.)):
        result=m.Motion()
        for i in range(20):
            result.add(1_000_000_000+i*10_000_000,'world',1,(0.,0.,0.),velocity,acceleration,source=b'fixture')
        return result

    def test_norms_detect_combined_axis_limit_exceedance(self):
        nominal=self.motion()
        self.assertAlmostEqual(nominal.speed,.3)
        self.assertAlmostEqual(nominal.acceleration,.5)
        self.assertTrue(nominal.within(.3,.5))
        self.assertFalse(self.motion(velocity=(.25,.25,0.)).within(.3,.5))
        self.assertFalse(self.motion(acceleration=(.4,.4,0.)).within(.3,.5))
        self.assertFalse(self.motion(velocity=(.300001,0.,0.)).within(.3,.5))

    def test_bad_frame_flag_or_nonfinite_reference_invalidates_sample(self):
        cases=[('bad',1,(0.,0.,0.)),('world',0,(0.,0.,0.)),
               ('world',3,(0.,0.,0.)),('world',2,(float('nan'),0.,0.))]
        for frame,flag,position in cases:
            metric=self.motion()
            metric.add(2_000_000_000,frame,flag,position,(0.,0.,0.),(0.,0.,0.))
            self.assertFalse(metric.within(.3,.5))
        valid=self.motion()
        valid.add(2_000_000_000,'world',2,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.))
        self.assertTrue(valid.within(.3,.5))

    def test_reference_step_is_reported_and_reordered_commands_fail(self):
        metric=self.motion()
        metric.add(1_200_000_000,'world',1,(.3,0.,0.),(0.,0.,0.),(0.,0.,0.),source=b'fixture')
        self.assertAlmostEqual(metric.step,.3)
        self.assertAlmostEqual(metric.step_dt,.01)
        self.assertEqual(metric.step_detail['before'][1],(0.,0.,0.))
        self.assertEqual(metric.step_detail['after'][1],(.3,0.,0.))
        self.assertTrue(metric.step_detail['same_source'])
        self.assertTrue(metric.within(.3,.5))  # Steps are separate ground-replan diagnostics.
        metric.add(1_200_000_000,'world',1,(0.,0.,0.),(0.,0.,0.),(0.,0.,0.))
        self.assertFalse(metric.within(.3,.5))

    def test_series_rejects_stale_future_missing_and_nonmonotonic_input(self):
        def series():
            result=m.Series()
            result.sources.add(b'fixture')
            for i in range(20):result.add(i*.01,1_000_000_000+i*10_000_000,.02)
            return result
        self.assertTrue(series().fresh(.2))
        self.assertFalse(series().fresh(.5))
        self.assertFalse(m.Series().fresh(.2))
        for stamp,age in [(0,.02),(1,.02),(2_000_000_000,.3),
                          (2_000_000_000,-.1),(2_000_000_000,float('nan'))]:
            metric=series();metric.add(.2,stamp,age)
            self.assertFalse(metric.fresh(.21))
        metric=series();metric.add(.5,2_000_000_000,.02)
        self.assertFalse(metric.fresh(.51))
        metric=series();metric.bad_frames=1
        self.assertFalse(metric.fresh(.2))
        metric=series();metric.sources.add(b'second')
        self.assertFalse(metric.fresh(.2))


if __name__=='__main__':unittest.main()
