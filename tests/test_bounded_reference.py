"""Pinned polynomial evaluation, fixed envelope, completion and restart guards."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/planning'))
from bounded_reference_preview import Plan, Reference, TakeoffFrame


def trajectory(start=100., distance=.1):
    # Quintic rest-to-rest motion over 2 seconds, padded to pinned degree 7.
    return NS(header=NS(frame_id='world'),type=3,order_pos=7,piece_num_pos=1,
              start_wt_pos=start,time_pos=[2.],
              coef_pos_x=[0.,0.,.1875*distance,-.9375*distance,1.25*distance,0.,0.,0.],
              coef_pos_y=[0.]*8,coef_pos_z=[0.]*8)


class Tests(unittest.TestCase):
    def reference(self):
        r=Reference(TakeoffFrame([0,0,0],[0,0,0,1]))
        r.add_plan(trajectory())
        return r

    def sample(self,r,t,offset=(0,0,0)):
        p,v,a=r.plans[0].evaluate(t)
        return r.command(round((100+t)*1e9),p,v,a,p+offset,[0,0,0,1])

    def test_forward_and_floor_frame_is_fixed_and_sign_invariant(self):
        q=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)]
        f=TakeoffFrame([2,-1,.3],q)
        np.testing.assert_allclose(f.coordinates([2,0,1.215]),[1,0,1],atol=1e-12)
        np.testing.assert_allclose(f.axes,TakeoffFrame([2,-1,.3],-np.array(q)).axes)
        base=TakeoffFrame([0,0,0],[0,0,0,1])
        np.testing.assert_allclose(base.check([-.0552,-.0466,.0239]),[-.0552,-.0466,.1089])
        base.check([-.10,0,0])
        # Backward allowance is measured along captured yaw, not world X.
        np.testing.assert_allclose(f.check([2,-1.099,.3]),[-.099,0,.085],atol=1e-12)
        base.check([0,0,-.185])
        for p in ([-.100001,0,0],[5.001,0,0],[0,2.001,0],[0,0,-.185001],[0,0,1.916]):
            with self.assertRaises(ValueError):TakeoffFrame([0,0,0],[0,0,0,1]).check(p)

    def test_initial_offset_is_per_axis_and_uses_current_measured_position(self):
        # Actual reported reference is >10 cm in norm, but <10 cm on every axis.
        r=self.reference()
        self.assertTrue(self.sample(r,0.,(-.0571,-.0181,-.0855)))
        np.testing.assert_allclose(r.initial_offset,[-.0571,-.0181,-.0855])
        for axis in range(3):
            for sign in (-1,1):
                r=self.reference();inside=np.zeros(3);inside[axis]=sign*.10
                self.assertTrue(self.sample(r,0.,inside))
                r=self.reference();offset=np.zeros(3);offset[axis]=sign*.100001
                with self.assertRaisesRegex(ValueError,'offset exceeds|Outside'):
                    self.sample(r,0.,offset)
        # Current measurement, rather than the earlier captured origin, gates handover.
        r=self.reference()
        with self.assertRaisesRegex(ValueError,'offset exceeds'):
            r.command(100000000000,[0,0,0],[0,0,0],[0,0,0],[.05,0,0],[0,0,0,1],
                      measured_position=[-.06,0,0])
        # Yaw defines forward/left; a world-axis diagonal can exceed that limit.
        q=[0,0,math.sin(math.pi/8),math.cos(math.pi/8)]
        r=Reference(TakeoffFrame([0,0,0],q));r.add_plan(trajectory())
        with self.assertRaisesRegex(ValueError,'offset exceeds'):
            self.sample(r,0.,(.08,.08,0))

    def test_piece_coefficients_use_descending_local_seconds(self):
        m=trajectory();m.time_pos=[1.,2.];m.piece_num_pos=2
        m.coef_pos_x=[0.,0.,0.,0.,0.,0.,.1,0.]+[0.,0.,0.,0.,0.,0.,.1,.1]
        m.coef_pos_y=m.coef_pos_z=[0.]*16
        p=Plan(m)
        np.testing.assert_allclose(p.evaluate(2.5),[[.25,0,0],[.1,0,0],[0,0,0]])
        np.testing.assert_allclose(p.endpoint,[.3,0,0])
        self.assertAlmostEqual(p.end,103.)

    def test_invalid_committed_trajectory_rejected(self):
        for field,value in (('type',1),('type',19),('order_pos',6),('start_wt_pos',math.nan),
                            ('time_pos',[0.]),('time_pos',[121.]),('coef_pos_x',[0.]*7),
                            ('coef_pos_y',[math.nan]*8)):
            m=trajectory();setattr(m,field,value)
            with self.assertRaises(ValueError):Plan(m)

    def test_zero_speed_at_start_does_not_mean_finished(self):
        r=self.reference()
        self.assertTrue(self.sample(r,0.))
        self.assertEqual(r.phase,'TRACK')
        r2=Reference(r.frame)
        self.assertIsNone(r2.command(100000000000,[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0,1]))
        self.assertEqual(r2.phase,'WAIT_TRAJECTORY')

    def test_verified_endpoint_is_held_across_stationary_restart(self):
        r=self.reference()
        for t in np.arange(0.,2.001,.02):self.assertTrue(self.sample(r,float(t)) is not None)
        self.assertEqual(r.phase,'END_HOLD')
        held=r.last[1].copy()
        r.add_plan(trajectory(start=102.))
        self.assertFalse(r.command(102020000000,[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0,1]))
        np.testing.assert_array_equal(r.last[1],held)
        self.assertEqual(r.restarts_blocked,1)
        self.assertEqual(r.phase,'END_HOLD')
        # Losing the terminal sample on a best-effort link must not silently
        # classify the next trajectory's restart as successful completion.
        missing=self.reference()
        for t in np.arange(0.,1.961,.02):self.sample(missing,float(t))
        self.assertEqual(missing.phase,'TRACK')
        missing.add_plan(trajectory(start=102.01))
        p,v,a=missing.plans[-1].evaluate(.02)
        with self.assertRaisesRegex(ValueError,'discontinuity'):
            missing.command(102030000000,p,v,a,p,[0,0,0,1])

    def test_moving_jump_and_stamp_reorder_are_rejected(self):
        r=self.reference();self.sample(r,0.)
        previous=r.last[1].copy()
        with self.assertRaisesRegex(ValueError,'discontinuity') as raised:self.sample(r,.02,(.06,0,0))
        details=str(raised.exception)
        self.assertIn('(position step)',details)
        self.assertIn('dt=0.020000s',details)
        self.assertIn('allowed 0.021000m',details)
        self.assertIn('previous/current raw xyz=',details)
        self.assertIn('previous raw endpoint distance=',details)
        np.testing.assert_array_equal(r.last[1],previous)
        self.assertEqual(r.accepted,1)
        with self.assertRaisesRegex(ValueError,'Nonincreasing'):self.sample(r,0.)
        r.fault('jump')
        self.assertFalse(self.sample(r,.02))
        self.assertEqual(r.phase,'FAULT')

    def test_gap_diagnostic_is_distinct_and_cannot_accept_or_hold(self):
        r=self.reference();self.sample(r,0.)
        with self.assertRaisesRegex(ValueError,'discontinuity') as raised:self.sample(r,.3)
        details=str(raised.exception)
        self.assertIn('(stamp gap)',details)
        self.assertIn('dt=0.300000s',details)
        self.assertIn('max 0.250000s',details)
        self.assertNotIn('(stamp gap, position step)',details)
        self.assertEqual(r.accepted,1)
        self.assertEqual(r.phase,'TRACK')
        self.assertEqual(r.last[0],100000000000)

    def test_caps_and_changed_goal_are_not_silently_accepted(self):
        r=self.reference();self.sample(r,0.)
        with self.assertRaisesRegex(ValueError,'derivative'):
            r.command(100020000000,[0,0,0],[.31,0,0],[0,0,0],[0,0,0],[0,0,0,1])
        with self.assertRaisesRegex(ValueError,'endpoint changed'):r.add_plan(trajectory(distance=.2))

    def test_time_and_position_must_match_committed_polynomial(self):
        p=Plan(trajectory())
        xyz,v,a=p.evaluate(.5)
        self.assertTrue(p.matches(100500000000,xyz,v,a))
        self.assertFalse(p.matches(105000000000,xyz,v,a))
        self.assertFalse(p.matches(100500000000,xyz+[.1,0,0],v,a))


if __name__=='__main__':unittest.main()
