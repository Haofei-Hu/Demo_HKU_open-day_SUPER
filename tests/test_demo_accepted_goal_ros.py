"""SUPER accepted goals control leg completion without changing requested targets."""
from collections import deque
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'scripts/flight'),str(ROOT/'scripts/planning'),str(ROOT/'scripts/maintenance'),str(ROOT/'scripts/simulation')]
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from mars_quadrotor_msgs.msg import PositionCommand
from super_demo_control import DemoMissionControl,VenueFrame,planner_goal_marker
from demo_mission import DemoMission
from demo_mission_simulation import synthetic_goal_markers
from test_demo_mission import fixture
from test_super_flight_control_ros import stage_plan
from bounded_reference_preview import Plan


class Harness(DemoMissionControl):
    def __init__(self):
        self.motion_policy='observe'
        self.round_index=1
        self.phase='TRACK';self.leg='outbound';self.goal_issued_ns=100_000_000_000;self.now_ns=101_000_000_000
        self.accepted_raw=self.accepted_target=self.previous_accepted_raw=None
        self.requested_raw=np.array([3.194326,-3.172474,1.651302])
        self.requested_target=np.array([4.5,0.,1.415]);self.goal_q=np.array([0.,0.,0.,1.])
        self.mapping=NS(converter=lambda p,q,c:(np.asarray(p),q,c),origin=self.requested_raw-self.requested_target,
                        failed=False,pairs=20)
        self.mission=DemoMission(fixture())
        self.frame=VenueFrame([0,0,0],[0,0,0,1],self.mission,lambda:False)
        self.mission_report=NS(data=dict(accepted_goals={},planner_motion_by_leg={}),save=Mock(),event=Mock(),completed=Mock())
        self.logs=[];self.errors=[];self.follower=self.make_follower();self.pending=deque();self.poly_pending=deque()
        self.source=NS(node_name='fsm_node',node_namespace='/',endpoint_gid=bytes(range(24)))
        self.planner_sources={'/visualization/goal':self.source.endpoint_gid}
        self.plan_source=NS(node_name='fsm_node',node_namespace='/',endpoint_gid=bytes(range(12))+b'p'*12)
        self.goal_count=1;self.goal_pub=NS(get_subscription_count=lambda:1,publish=Mock())
        self.native_velocity=np.zeros(3);self.velocity=NS(twist=NS(linear=NS(x=0.,y=0.,z=0.)))
        self.home=np.array([0.,0.,1.415]);self.far=self.requested_target.copy();self.reference=NS(yaw=0.)
        self.distance=4.5;self.state=NS(armed=True);self.execute=True;self.leg_start_target=None
        self.arrived_since=None;self.return_hold_target=None
        self.raw_pose=Pose();self.raw_pose.orientation.w=1.
        self.published=[];self.tracking_ok=Mock()
    def get_publishers_info_by_topic(self,topic):return [self.source if topic=='/visualization/goal' else self.plan_source]
    def get_logger(self):return NS(info=self.logs.append,warning=self.logs.append)
    def get_clock(self):
        def now():
            sec,ns=divmod(self.now_ns,1_000_000_000)
            return NS(nanoseconds=self.now_ns,to_msg=lambda:Time(sec=sec,nanosec=ns))
        return NS(now=now)
    def abort(self,error):self.errors.append(error);self.phase='STOPPED'
    def count_publishers(self,topic):return 1
    def change(self,phase):self.phase=phase
    def publish(self,target,yaw,native=False):self.published.append(np.array(target,copy=True))
    def land(self,reason):self.phase='LAND_REQUEST'
    def feedback(self,raw,stamp_ns=None):
        ns=self.now_ns if stamp_ns is None else stamp_ns
        sec,nanosec=divmod(ns,1_000_000_000)
        msg=synthetic_goal_markers(raw,self.mapping.origin+self.home,Time(sec=sec,nanosec=nanosec))
        self.planner_goal_received(msg)
        return msg


class Tests(unittest.TestCase):
    def test_five_round_trips_keep_fixed_endpoints_and_land_only_after_tenth_goal(self):
        node=Harness();config=fixture();config['flight']['round_trips']=5
        node.mission=DemoMission(config);node.goal_count=0
        node.request_leg('outbound',node.far)
        for number in range(1,6):
            for direction in ('outbound','return'):
                self.assertEqual(node.round_index,number);self.assertEqual(node.leg,direction)
                expected=node.far if direction=='outbound' else node.home
                np.testing.assert_allclose(node.requested_target,expected)
                node.now_ns+=2_000_000_000;node.feedback(node.requested_raw)
                target=node.accepted_target.copy()
                node.follower.phase='END_HOLD';node.follower.last=(node.now_ns,target,node.goal_q)
                node.phase='END_HOLD';now=node.now_ns/1e9;node.arrived_since=now-1.1
                node.mission_tick(now,target,0.)
                if direction=='return':
                    self.assertEqual(node.mission_report.data['round_trips_completed'],number)
                    self.assertEqual(node.phase,'FINAL_HOLD' if number==5 else 'WAIT_PLAN')
        self.assertEqual(node.goal_count,10)
        self.assertEqual(len(node.mission_report.data['accepted_goals']),10)
        self.assertEqual(len(node.mission_report.data['goal_arrivals']),10)
        self.assertIn('round_01_outbound',node.mission_report.data['goal_arrivals'])
        self.assertIn('round_05_return',node.mission_report.data['goal_arrivals'])
        node.mission_tick(now+5.,node.return_hold_target,0.)
        self.assertEqual(node.phase,'FINAL_HOLD')
        node.mission_tick(now+6.1,node.return_hold_target,0.)
        self.assertEqual(node.phase,'LAND_REQUEST')
        with self.assertRaisesRegex(ValueError,'already been sent'):node.request_leg('outbound',node.far)

    def test_stream_queue_uses_latest_fresh_sample_without_retiming(self):
        node=Harness();raw=np.array([3.25,-3.25,1.85]);node.feedback(raw)
        msg=stage_plan(node.mapping.origin+node.home,raw,100.,5.)
        node.follower.add_plan(msg);plan=Plan(msg);node.reference_accepted=Mock()
        def command(elapsed):
            p,v,a=plan.evaluate(elapsed);cmd=PositionCommand();cmd.header.frame_id='world'
            ns=round((100.+elapsed)*1e9);cmd.header.stamp.sec,cmd.header.stamp.nanosec=divmod(ns,1000000000)
            cmd.trajectory_flag=1
            for name,values in (('position',p),('velocity',v),('acceleration',a)):
                for axis,value in zip(('x','y','z'),values):setattr(getattr(cmd,name),axis,float(value))
            return cmd
        node.now_ns=102_050_000_000;old=command(.5);latest=command(2.04)
        node.pending.extend([old,latest]);node.process_reference(node.home)
        self.assertEqual(node.follower.last[0],102_040_000_000)
        self.assertEqual(node.mission_report.data['superseded_command_samples'],1)
        node.reference_accepted.assert_called_once_with(latest)
        node.pending.extend([command(.5),command(.6)])
        with self.assertRaisesRegex(ValueError,'Stale command'):
            node.process_reference(node.home)

    def test_pinned_layout_extracts_reported_goal_and_ignores_delete_and_stale(self):
        stamp=Time(sec=101);goal=[3.25,-3.25,1.85]
        msg=synthetic_goal_markers(goal,[0,0,0],stamp)
        np.testing.assert_array_equal(planner_goal_marker(msg,100_000_000_000),goal)
        self.assertIsNone(planner_goal_marker(msg,102_000_000_000))
        deleted=deepcopy(msg)
        for marker in deleted.markers:marker.action=Marker.DELETEALL
        self.assertIsNone(planner_goal_marker(deleted,0))
        wrong=deepcopy(msg);wrong.markers[2].points[0].x+=1.
        self.assertIsNone(planner_goal_marker(wrong,0))
        wrong=deepcopy(msg);wrong.markers[0].header.frame_id='map'
        self.assertIsNone(planner_goal_marker(wrong,0))

    def test_reported_relocation_completes_against_confirmed_goal(self):
        node=Harness();requested=node.requested_target.copy();raw=np.array([3.25,-3.25,1.85])
        node.feedback(raw)
        plan=Plan(stage_plan(node.mapping.origin+node.home,raw,100.,3.))
        self.assertFalse(node.errors);self.assertTrue(node.check_stage_endpoint(plan))
        self.assertTrue(node.final_goal_reached(plan,node.accepted_target))
        self.assertGreater(node.accepted_target[2]-requested[2],.19)
        np.testing.assert_array_equal(node.requested_target,requested)
        self.assertIn('outbound',node.mission_report.data['accepted_goals'])

    def test_arbitrary_polynomial_endpoint_is_not_a_goal_acknowledgement(self):
        node=Harness();raw=np.array([3.25,-3.25,1.85])
        plan=Plan(stage_plan(node.mapping.origin+node.home,raw,100.,3.))
        self.assertFalse(node.check_stage_endpoint(plan))
        node.feedback(raw)
        middle=Plan(stage_plan(node.mapping.origin+node.home,node.mapping.origin+[2.,0.,1.415],100.,3.))
        self.assertFalse(node.check_stage_endpoint(middle))

    def test_wrong_publisher_and_previous_leg_feedback_are_not_adopted(self):
        node=Harness();raw=np.array([3.25,-3.25,1.85])
        msg=synthetic_goal_markers(raw,[0,0,0],Time(sec=101))
        node.source.endpoint_gid=b'x'*24
        node.planner_goal_received(msg)
        node.source.endpoint_gid=node.planner_sources['/visualization/goal']
        self.assertIsNone(node.accepted_target)
        node.plan_source.node_name='another_planner'
        node.planner_goal_received(msg)
        self.assertIsNone(node.accepted_target)
        node.plan_source.node_name='fsm_node';node.previous_accepted_raw=raw.copy()
        node.feedback(raw)
        self.assertIsNone(node.accepted_target)
        node.previous_accepted_raw=None;node.feedback(raw,99_000_000_000)
        self.assertIsNone(node.accepted_target)

    def test_accepted_goal_outside_vehicle_inclusive_venue_is_rejected(self):
        node=Harness();node.feedback(node.mapping.origin+np.array([5.5,0.,1.415]))
        self.assertEqual(node.phase,'STOPPED');self.assertIsNone(node.accepted_target)
        self.assertIn('outside venue',node.errors[0])

    def test_late_goal_feedback_promotes_verified_rest_without_new_commands(self):
        node=Harness();raw=np.array([3.25,-3.25,1.85])
        msg=stage_plan(node.mapping.origin+node.home,raw,100.,3.)
        node.follower.add_plan(msg);plan=node.follower.plans[-1]
        for t in np.arange(0.,3.021,.02):
            p,v,a=plan.evaluate(t);position=p-node.mapping.origin
            node.follower.command(round((100.+t)*1e9),p,v,a,position,node.goal_q,measured_position=node.home)
        self.assertEqual(node.follower.phase,'TRACK')
        node.now_ns=104_000_000_000;node.feedback(raw)
        node.process_reference(node.accepted_target)
        self.assertEqual(node.follower.phase,'END_HOLD')

    def test_outbound_arrival_sends_return_home_and_holds_actual_far_point(self):
        node=Harness();node.feedback([3.25,-3.25,1.85]);far=node.accepted_target.copy()
        node.follower.phase='END_HOLD';node.follower.last=(101_000_000_000,far,node.goal_q)
        node.phase='END_HOLD';node.arrived_since=100.;node.now_ns=102_000_000_000
        node.mission_tick(102.,far,0.)
        self.assertEqual(node.leg,'return');self.assertEqual(node.goal_count,2)
        np.testing.assert_array_equal(node.requested_target,node.home)
        np.testing.assert_array_equal(node.leg_start_target,far)
        node.mission_tick(103.,far,0.)
        np.testing.assert_array_equal(node.published[-1],far)
        self.assertIsNone(node.accepted_target)

    def test_return_hover_uses_confirmed_return_point_without_unplanned_home_jump(self):
        node=Harness();node.leg='return';node.requested_target=node.home.copy()
        node.requested_raw=node.mapping.origin+node.home
        node.feedback(node.requested_raw+np.array([-.03,.02,.12]));returned=node.accepted_target.copy()
        node.follower.phase='END_HOLD';node.follower.last=(101_000_000_000,returned,node.goal_q)
        node.phase='END_HOLD';node.arrived_since=100.
        node.mission_tick(102.,returned,0.);self.assertEqual(node.phase,'FINAL_HOLD')
        node.mission_tick(103.,returned,0.)
        np.testing.assert_array_equal(node.published[-1],returned)
        node.mission_tick(108.1,returned,0.);self.assertEqual(node.phase,'LAND_REQUEST')


if __name__=='__main__':unittest.main()
