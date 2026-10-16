"""Ground-only committed-stage semantics; no planner configuration changes."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS,MethodType
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts/flight'),str(ROOT/'scripts/planning')]
from ground_speed_audit import GroundSpeedAudit
from super_flight_control import SuperFlightControl
from bounded_reference_preview import Plan,Reference,TakeoffFrame
from control_bridge_preview import Mapping,quaternion_rpy
from test_super_flight_control_ros import stage_plan


def controller(yaw=0.):
    mapping=Mapping(lambda p,q,c:(np.array(p),q,c));mapping.origin=np.zeros(3);mapping.pairs=20
    frame=TakeoffFrame([0,0,0],quaternion_rpy(0,0,yaw),max_forward=5.1)
    requested=frame.axes.T@np.array([5.,0.,0.])
    logs=[]
    node=NS(mapping=mapping,frame=frame,requested_target=requested,requested_raw=requested,
            goal_q=quaternion_rpy(0,0,yaw),execute=False,distance=5.,hover_fc_height=1.,
            committed_plans=0,last_endpoint_kind=None,get_logger=lambda:NS(info=logs.append))
    for name in ('endpoint_evidence','target_ok'):
        setattr(node,name,MethodType(getattr(SuperFlightControl,name),node))
    for name in ('check_stage_endpoint','make_follower','accept_plan','supported_command','final_goal_reached'):
        setattr(node,name,MethodType(getattr(GroundSpeedAudit,name),node))
    node.follower=node.make_follower()
    return node,logs


def sample(reference,msg,t):
    plan=Plan(msg);p,v,a=plan.evaluate(t)
    return reference.command(round((plan.start+t)*1e9),p,v,a,p,[0,0,0,1])


class Tests(unittest.TestCase):
    def test_recorded_goal_snap_cannot_account_for_ten_cm_error(self):
        delta=np.array([3.550,-3.550,.750])-np.array([3.5201,-3.5759,.7532])
        self.assertLess(np.linalg.norm(delta),.04)
        # A rigid rotation cannot turn this vector into >10 cm on any axis.
        for yaw in (0.,.6,2.6):
            axes=TakeoffFrame([0,0,0],quaternion_rpy(0,0,yaw)).axes
            self.assertLess(max(abs(axes@delta)),.04)

    def test_intermediate_braking_endpoint_is_not_a_final_goal(self):
        node,logs=controller()
        msg=stage_plan(np.zeros(3),np.array([1.,0.,0.]),100.,3.)
        with self.assertRaisesRegex(ValueError,'does not establish map relocation') as raised:
            SuperFlightControl.accept_plan(node,Plan(msg),msg)
        self.assertIn('endpoint error forward/left/up=[-4.0, 0.0, 0.0]',str(raised.exception))
        node.accept_plan(Plan(msg),msg)
        for t in np.arange(0.,3.001,.02):sample(node.follower,msg,float(t))
        self.assertEqual(node.follower.phase,'TRACK','Intermediate stop was declared a completed 5 m goal')
        self.assertTrue(any('INTERMEDIATE_STOP' in text for text in logs))
        self.assertTrue(node.supported_command(2))
        self.assertFalse(SuperFlightControl.supported_command(node,2))

    def test_continuous_new_commits_reach_only_the_original_final_goal(self):
        node,logs=controller()
        first=stage_plan(np.zeros(3),np.array([1.,0.,0.]),100.,3.)
        node.accept_plan(Plan(first),first)
        for t in np.arange(0.,3.001,.02):sample(node.follower,first,float(t))
        second=stage_plan(np.array([1.,0.,0.]),np.array([5.,0.,0.]),103.,5.)
        node.accept_plan(Plan(second),second)
        for t in np.arange(.02,5.001,.02):sample(node.follower,second,float(t))
        self.assertEqual(node.follower.phase,'END_HOLD')
        np.testing.assert_allclose(node.follower.last[1],[5.,0.,0.],atol=1e-10)
        self.assertTrue(any('FINAL_GOAL_CANDIDATE' in text for text in logs))
        default=Reference(node.frame,2.,1.);default.add_plan(first)
        sample(default,first,0.)
        with self.assertRaisesRegex(ValueError,'endpoint changed'):default.add_plan(second)

    def test_off_corridor_and_wrong_final_endpoints_do_not_pass(self):
        node,_=controller(yaw=.6)
        for forward,left,up in ((1.,.10001,0.),(1.,0.,.10001),(5.10001,0.,0.),(-.10001,0.,0.)):
            endpoint=node.frame.axes.T@np.array([forward,left,up])
            msg=stage_plan(np.zeros(3),endpoint,100.,8.)
            with self.assertRaises(ValueError) as raised:node.accept_plan(Plan(msg),msg)
            self.assertIn('committed planner endpoint=',str(raised.exception))
            self.assertIn('endpoint error forward/left/up=',str(raised.exception))
        msg=stage_plan(np.zeros(3),node.frame.axes.T@np.array([4.,0.,0.]),100.,8.)
        node.accept_plan(Plan(msg),msg)
        for t in np.arange(0.,8.001,.02):sample(node.follower,msg,float(t))
        self.assertEqual(node.follower.phase,'TRACK')

    def test_rollover_does_not_hide_discontinuity_or_overspeed(self):
        node,_=controller()
        first=stage_plan(np.zeros(3),np.array([1.,0.,0.]),100.,3.)
        node.accept_plan(Plan(first),first);sample(node.follower,first,0.)
        jump=stage_plan(np.array([.2,0.,0.]),np.array([5.,0.,0.]),100.02,8.)
        node.accept_plan(Plan(jump),jump)
        with self.assertRaisesRegex(ValueError,'discontinuity'):sample(node.follower,jump,0.)
        with self.assertRaisesRegex(ValueError,'derivative'):
            node.follower.command(100020000000,[0,0,0],[2.001,0,0],[0,0,0],[0,0,0],[0,0,0,1])


    def test_held_position_cannot_stack_matching_error_beyond_goal_allowance(self):
        node,_=controller()
        msg=stage_plan(np.zeros(3),np.array([4.901,0.,0.]),100.,8.)
        node.accept_plan(Plan(msg),msg)
        for t in np.arange(0.,7.981,.02):sample(node.follower,msg,float(t))
        # A 9.9 cm polynomial offset plus 2 mm matching offset cannot pass.
        p,v,a=Plan(msg).evaluate(8.)
        node.follower.command(108000000000,p,v,a,p-np.array([.002,0.,0.]),[0,0,0,1])
        self.assertEqual(node.follower.phase,'TRACK')
        self.assertFalse(node.final_goal_reached(Plan(msg),[4.899,0.,0.]))


if __name__=='__main__':unittest.main()
