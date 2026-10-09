#!/usr/bin/env python3
"""Independent finite-difference checks of floor height, rates and hold endpoints."""
import math
import unittest

from hover_reference import HoverReference


class HoverReferenceTest(unittest.TestCase):
    def test_fixed_xy_floor_relative_height_and_no_rebase(self):
        reference = HoverReference((1.2, -.8, .034), math.radians(135))
        self.assertLess(reference.duration, 10.)
        first, yaw, phase = reference.at(0.)
        self.assertEqual(first, (1.2, -.8, .034))
        apex, _, phase = reference.at(reference.ramp_seconds + 1.)
        self.assertAlmostEqual(apex[2] - first[2] + .085, .5)
        self.assertEqual(phase, 'HOLD_PREVIEW')
        for elapsed in (reference.duration, reference.duration + 100):
            point, heading, phase = reference.at(elapsed)
            self.assertEqual(point, first)
            self.assertEqual(heading, yaw)
            self.assertEqual(phase, 'COMPLETE_GROUND_HOLD')

    def test_rates_and_stationary_joins(self):
        reference = HoverReference((0., 0., 0.), 0.)
        step = .001
        points = [reference.at(i * step)[0] for i in range(math.ceil(reference.duration / step) + 1)]
        velocities = [(points[i + 1][2] - points[i][2]) / step for i in range(len(points) - 1)]
        self.assertLessEqual(max(abs(v) for v in velocities), .200001)
        self.assertGreater(max(velocities), .1999)
        self.assertLess(min(velocities), -.1999)
        self.assertTrue(all(p[:2] == (0., 0.) and 0 <= p[2] <= .415000001 for p in points))
        for join in (0., reference.ramp_seconds, reference.ramp_seconds + reference.hold_seconds,
                     reference.duration):
            left = reference.at(max(0., join - step))[0][2]
            middle = reference.at(join)[0][2]
            right = reference.at(join + step)[0][2]
            self.assertLess(abs(right - left) / (2 * step), 1e-6)
            self.assertLess(abs(right - 2 * middle + left) / step ** 2, .001)

    def test_invalid_inputs(self):
        for args in [((0, 0), 0.), ((0, 0, math.nan), 0.), ((0, 0, 0), math.inf)]:
            with self.assertRaises(ValueError):
                HoverReference(*args)
        for kwargs in ({'hover_fc_height': 2.1}, {'hover_fc_height': .05},
                       {'ground_fc_height': 0.}, {'peak_vertical_speed': 0.},
                       {'peak_vertical_speed': .5}, {'hold_seconds': -1.},
                       {'hold_seconds': 6.1}, {'hold_seconds': math.nan}):
            with self.assertRaises(ValueError):
                HoverReference((0, 0, 0), 0., **kwargs)
        reference = HoverReference((0, 0, 0), 0.)
        for elapsed in (-1., math.nan, math.inf):
            with self.assertRaises(ValueError):
                reference.at(elapsed)

    def test_one_metre_six_second_profile(self):
        reference = HoverReference((1.2, -.8, -.74), .6,
                                   hover_fc_height=1., hold_seconds=6.)
        self.assertAlmostEqual(reference.lift, .915)
        self.assertAlmostEqual(reference.ramp_seconds, 8.578125)
        for elapsed in (reference.ramp_seconds, reference.ramp_seconds + 5.999):
            point, yaw, phase = reference.at(elapsed)
            self.assertEqual(phase, 'HOLD_PREVIEW')
            self.assertEqual(point[:2], (1.2, -.8))
            self.assertAlmostEqual(point[2] - reference.ground[2] + .085, 1.)
            self.assertEqual(yaw, .6)
        step = .001
        velocities = [(reference.at(t+step)[0][2]-reference.at(t)[0][2])/step
                      for t in (i*step for i in range(math.ceil(reference.duration/step))) ]
        self.assertLessEqual(max(abs(v) for v in velocities), .200001)


if __name__ == '__main__':
    unittest.main()
