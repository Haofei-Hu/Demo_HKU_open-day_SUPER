"""Committed rest endpoints can wait and resume without a planning deadline."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'scripts/flight'),str(ROOT/'scripts/planning'),str(ROOT/'scripts/maintenance')]
from super_demo_control import DemoMissionControl,VenuePlan,VenueReference,VenueFrame
from bounded_reference_preview import Plan
from first_hover_terminal import FirstHoverControl
from demo_mission import DemoMission
from test_demo_mission import fixture
from test_super_flight_control_ros import stage_plan


class Tests(unittest.TestCase):
    def follower(self):
        self.begin=np.array([0.,0.,1.415]);self.stop=np.array([.8,.2,1.415])
        self.goal=np.array([4.5,0.,1.415])
        frame=VenueFrame([0,0,0],[0,0,0,1],DemoMission(fixture()),lambda:False)
        return VenueReference(frame,lambda plan:True,
            lambda plan,p:np.linalg.norm(p-self.goal)<.01,'observe')

    def sample(self,follower,msg,elapsed):
        p,v,a=Plan(msg).evaluate(elapsed)
        return follower.command(round((msg.start_wt_pos+elapsed)*1e9),p,v,a,p,[0,0,0,1],
                                measured_position=self.begin)

    def test_fresh_stationary_commands_continue_beyond_polynomial_end(self):
        follower=self.follower();msg=stage_plan(self.begin,self.stop,100.,3.)
        follower.add_plan(msg)
        for t in np.arange(0.,6.001,.02):self.assertTrue(self.sample(follower,msg,float(t)))
        self.assertEqual(follower.phase,'TRACK')
        self.assertTrue(follower.resting_at_endpoint())
        np.testing.assert_allclose(follower.last[1],self.stop)
        self.assertFalse(Plan(msg).matches(106000000000,self.stop,np.zeros(3),np.zeros(3)),
                         'Legacy diagnostics must retain their original matching rule')

    def test_terminal_completion_does_not_hide_a_remaining_loop_or_unknown_goal(self):
        follower=self.follower();msg=stage_plan(self.goal,self.goal,100.,1.)
        # x = goal + .2*t^3*(1-t)^3: stationary at the goal at both ends,
        # but the unobserved remaining curve contains a real excursion.
        msg.coef_pos_x=[0.,-.2,.6,-.6,.2,0.,0.,float(self.goal[0])]
        follower.add_plan(msg);self.sample(follower,msg,0.)
        self.assertIsNone(follower.finish_final_rest(101_000_000_000))
        self.assertEqual(follower.phase,'TRACK')
        follower=self.follower();msg=stage_plan(self.begin,self.goal,100.,3.)
        follower.add_plan(msg);self.sample(follower,msg,2.999)
        follower.final_goal_check=lambda plan,p:False
        self.assertIsNone(follower.finish_final_rest(103_010_000_000))
        self.assertEqual(follower.phase,'TRACK')

    def test_terminal_completion_rejects_moving_endpoints(self):
        follower=self.follower();msg=stage_plan(self.goal,self.goal,100.,1.)
        msg.coef_pos_x[-2]=.2
        follower.add_plan(msg);self.sample(follower,msg,.999)
        self.assertIsNone(follower.finish_final_rest(101_010_000_000))
        self.assertEqual(follower.phase,'TRACK')

    def test_silent_rest_wait_has_no_deadline_and_resumes_continuously(self):
        follower=self.follower();msg=stage_plan(self.begin,self.stop,100.,3.)
        follower.add_plan(msg)
        for t in np.arange(0.,3.021,.02):self.sample(follower,msg,float(t))
        control=NS(follower=follower,native_velocity=np.ones(3),leg='outbound',round_index=1,accepted_target=self.goal,fresh_ns=lambda ns:False,
                   mission_report=NS(data={}),get_logger=lambda:NS(warning=lambda text:None))
        before=follower.last[0]
        for _ in range(3):DemoMissionControl.check_command_stream(control)
        np.testing.assert_array_equal(control.native_velocity,np.zeros(3))
        self.assertIsNone(control.mission_report.data['intermediate_rest_wait']['timeout_s'])
        self.assertEqual(follower.last[0],before,'Do not retime a received command to keep it fresh')
        next_msg=stage_plan(self.stop,self.goal,5000.,6.)
        follower.add_plan(next_msg)
        self.assertTrue(self.sample(follower,next_msg,.01))
        self.assertEqual(follower.max_command_gap(),float('inf'))
        self.assertTrue(self.sample(follower,next_msg,.4))

    def test_rest_matching_rejects_motion_and_uncommitted_positions(self):
        self.follower();msg=stage_plan(self.begin,self.stop,100.,3.);plan=VenuePlan(msg)
        self.assertTrue(plan.matches(200000000000,self.stop,np.zeros(3),np.zeros(3)))
        self.assertFalse(plan.matches(200000000000,self.stop+[.1,0,0],np.zeros(3),np.zeros(3)))
        self.assertFalse(plan.matches(200000000000,self.stop,[.2,0,0],np.zeros(3)))
        self.assertFalse(plan.matches(200000000000,self.stop,np.zeros(3),[.2,0,0]))
        msg.coef_pos_x=[0.,0.,0.,0.,0.,0.,.2,0.]
        moving=VenuePlan(msg)
        self.assertFalse(moving.matches(200000000000,moving.endpoint,np.zeros(3),np.zeros(3)))

    def test_callback_gap_is_advisory_in_complete_demo_only(self):
        messages=[]
        control=NS(mission_report=NS(data={}),get_logger=lambda:NS(warning=messages.append))
        for gap in (.37561,5.,1000.):DemoMissionControl.check_timer_gap(control,gap)
        self.assertEqual(control.mission_report.data['callback_gap_advisories'],3)
        self.assertEqual(control.mission_report.data['largest_callback_gap_s'],1000.)
        self.assertEqual(len(messages),3)
        with self.assertRaisesRegex(ValueError,'Setpoint timer interrupted'):
            FirstHoverControl.check_timer_gap(control,.37561)

    def test_resume_cannot_jump_or_cross_venue_boundary(self):
        follower=self.follower();msg=stage_plan(self.begin,self.stop,100.,3.)
        follower.add_plan(msg)
        for t in np.arange(0.,3.021,.02):self.sample(follower,msg,float(t))
        jumped=stage_plan(self.stop+[.2,0,0],self.goal,5000.,6.)
        follower.add_plan(jumped)
        with self.assertRaisesRegex(ValueError,'position step'):self.sample(follower,jumped,.01)
        follower=self.follower();outside=stage_plan(self.begin,[.8,1.5,1.415],100.,3.)
        follower.add_plan(outside)
        with self.assertRaisesRegex(ValueError,'outside venue'):self.sample(follower,outside,3.1)

    def test_logged_two_centimetre_rest_reanchor_uses_new_polynomial(self):
        follower=self.follower()
        previous=self.stop+np.array([.0072,-.0036,.0201])
        msg=stage_plan(self.begin,previous,100.,.3)
        follower.add_plan(msg)
        self.assertTrue(self.sample(follower,msg,.305304))
        self.assertTrue(follower.resting_at_endpoint())
        next_msg=stage_plan(self.stop,self.goal,100.334788,4.121009)
        follower.add_plan(next_msg)
        self.assertTrue(self.sample(follower,next_msg,.030373))
        self.assertEqual(len(follower.rest_handoffs),1)
        self.assertAlmostEqual(follower.rest_handoffs[0]['seam_m'],.02165,places=5)
        self.assertTrue(self.sample(follower,next_msg,.1))
        self.assertEqual(len(follower.rest_handoffs),1)

    def test_recorded_five_cm_snap_is_rejected_but_exact_rest_start_resumes(self):
        # The rounded raw-coordinate delta from the third outbound failure.
        snap_delta=np.array([-.0237,-.0488,.0036])
        for relocated in (True,False):
            follower=self.follower();msg=stage_plan(self.begin,self.stop,100.,3.)
            follower.add_plan(msg);self.sample(follower,msg,3.0018)
            next_start=self.stop+snap_delta if relocated else self.stop
            next_msg=stage_plan(next_start,self.goal,103.111194,2.1605)
            follower.add_plan(next_msg)
            if relocated:
                with self.assertRaisesRegex(ValueError,'position step'):
                    self.sample(follower,next_msg,.028)
            else:
                self.assertTrue(self.sample(follower,next_msg,.028))
                self.assertLess(follower.rest_handoffs[-1]['seam_m'],1e-12)
                for t in np.arange(.048,2.15,.02):
                    self.assertTrue(self.sample(follower,next_msg,float(t)))

    def test_stationary_handoff_cannot_hide_a_larger_reanchor_or_moving_restart(self):
        for offset,moving in ((.041,False),(.02,True)):
            follower=self.follower();msg=stage_plan(self.begin,self.stop,100.,3.)
            follower.add_plan(msg);self.sample(follower,msg,3.02)
            next_msg=stage_plan(self.stop+[offset,0,0],self.goal,104.,6.)
            if moving:
                next_msg.coef_pos_x[-2]=.2
            follower.add_plan(next_msg)
            with self.assertRaisesRegex(ValueError,'position step'):
                self.sample(follower,next_msg,0.)
            self.assertFalse(follower.rest_handoffs)

    def test_first_sample_after_a_rest_uses_motion_already_in_the_new_plan(self):
        follower=self.follower();msg=stage_plan(self.begin,self.stop,100.,3.)
        follower.add_plan(msg);self.sample(follower,msg,3.02)
        next_msg=stage_plan(self.stop,self.goal,104.,6.)
        follower.add_plan(next_msg)
        self.assertTrue(self.sample(follower,next_msg,.8))
        self.assertGreater(follower.rest_handoffs[0]['new_plan_displacement_bound_m'],.015)


if __name__=='__main__':unittest.main()
