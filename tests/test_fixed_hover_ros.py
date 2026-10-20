"""Fixed hover uses the maintained physical controller without navigation goals."""
from collections import deque
from contextlib import contextmanager, redirect_stderr
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),*(str(ROOT/'scripts'/p) for p in
              ('flight','planning','maintenance','localization','simulation'))]
from demo_mission import DemoMission
from first_hover_terminal import HoverReference
from super_demo_control import DemoMissionControl,VenueFrame,arguments
import super_demo
from test_demo_mission import fixture
from test_live_venue import write_reference


@contextmanager
def physical_controller(hover_only=True):
    with tempfile.TemporaryDirectory() as directory:
        folder=Path(directory)
        config=fixture();config['flight']['round_trips']=5
        config=DemoMission.physical_height(config,1.)
        mission=folder/'mission.json';mission.write_text(json.dumps(config))
        before=mission.read_bytes()
        record,_,alignment,planner=write_reference(folder,mission=mission)
        rclpy.init();node=None
        try:
            node=DemoMissionControl(mission,folder/'report.json',execute=True,
                on_site_supervised=True,alignment_path=alignment,planner_config=planner,
                venue_reference=record,motion_policy='observe',hover_only=hover_only,
                hover_seconds=20.)
            yield node,folder
            assert mission.read_bytes()==before,'Controller changed the measured mission file'
        finally:
            if node is not None:node.destroy_node()
            rclpy.shutdown()


def airborne(node):
    node.reference=HoverReference([0.,0.,0.],.4,hover_fc_height=1.,hold_seconds=6.)
    node.frame=VenueFrame(node.reference.ground,[0.,0.,0.,1.],node.mission)
    node.home=np.array([0.,0.,node.reference.lift]);node.far=node.home+[4.5,0.,0.]
    node.velocity=NS(twist=NS(linear=NS(x=0.,y=0.,z=0.)))
    node.local=PoseStamped();node.local.header.stamp.sec=100
    node.sent_references=deque([(100_000_000_000,node.home.copy(),np.zeros(3))])
    node.stages_started=True;node.phase='HOLD'
    node.mission_report.completed(node.mission.PHASES[0])


class Tests(unittest.TestCase):
    def test_physical_constructor_keeps_start_venue_and_parameter_guards(self):
        with physical_controller() as (node,folder):
            self.assertEqual(node.phase,'CHECKING');self.assertFalse(node.started)
            self.assertEqual((node.sent,node.goal_count),(0,0))
            self.assertEqual(node.START_SERVICE,'/super_demo/start')
            self.assertFalse(node.start(Trigger.Request(),Trigger.Response()).success)
            self.assertIsNotNone(node.live_venue)
            self.assertEqual(node.NATIVE_TOPIC,'/mavros/setpoint_raw/local')
            self.assertEqual(node.EXPECTED_PARAMETERS['MPC_XY_VEL_MAX'],2.)
            self.assertEqual(node.EXPECTED_PARAMETERS['MPC_Z_VEL_MAX_UP'],.5)
            self.assertEqual(node.offboard_budget,float('inf'))
            self.assertEqual(node.mission.hover_height,1.)
            self.assertEqual(node.mission.round_trips,0)
            self.assertEqual(node.mission.PHASES,('TAKEOFF_TO_1_0M','STABILIZE_AT_HOME',
                'HOLD_HOME_20S','AUTO_LAND','CONFIRM_LANDED_AND_DISARMED'))
            report=json.loads((folder/'report.json').read_text())
            self.assertEqual(report['mission']['navigation_goals'],0)
            self.assertEqual(report['source_demonstration_mission']['flight']['round_trips'],5)
            self.assertEqual(report['round_trips_requested'],0)
            self.assertFalse(report['fixed_hover_completed'])
            self.assertFalse(report['physical_flight_verified'])
            policy=report['post_takeoff_position_policy']
            self.assertIsNone(policy['straight_line_corridor_m'])
            self.assertIsNone(policy['tracking_error_abort_m'])
            with self.assertRaisesRegex(ValueError,'cannot issue navigation goals'):
                node.request_leg('outbound',np.zeros(3))

    def test_twenty_second_hover_has_fixed_target_and_ordered_completion(self):
        with physical_controller() as (node,folder):
            airborne(node)
            node.request_leg=Mock(side_effect=AssertionError('Hover requested a route'))
            original_publish=node.publish
            node.publish=Mock(wraps=original_publish);node.land=Mock()
            node.mission_tick(10.,node.home,.4)
            node.mission_tick(11.99,node.home,.4)
            self.assertEqual(node.phase,'HOLD')
            node.mission_tick(12.,node.home,.4)
            self.assertEqual(node.phase,'FINAL_HOLD')
            self.assertEqual(node.final_hold_since,12.)
            # In-venue tracking error remains observed, and does not restart
            # the planned twenty seconds after initial arrival stabilization.
            excursion=node.home+np.array([.5,.3,0.])
            node.mission_tick(31.99,excursion,.4)
            node.land.assert_not_called()
            node.mission_tick(32.,excursion,.4)
            node.land.assert_called_once_with('Fixed hover complete')
            node.request_leg.assert_not_called()
            self.assertEqual(node.goal_count,0)
            self.assertEqual(node.final_hold_since,12.)
            self.assertAlmostEqual(node.max_tracking_error[0],.5)
            for call in node.publish.call_args_list:
                np.testing.assert_array_equal(call.args[0],node.home)
                self.assertEqual(call.args[1],.4)
                self.assertTrue(call.kwargs['native'])
            np.testing.assert_array_equal(node.native_velocity,np.zeros(3))
            self.assertEqual(node.mission_report.data['completed_stages'],
                             list(node.mission.PHASES[:3]))
            self.assertFalse(node.mission_report.data['fixed_hover_completed'])
            node.change('LANDING');node.change('DONE')
            report=json.loads((folder/'report.json').read_text())
            self.assertEqual(report['result'],'SUCCESS')
            self.assertTrue(report['fixed_hover_completed'])
            self.assertFalse(report['demonstration_completed'])
            self.assertFalse(report['physical_flight_verified'])
            self.assertEqual(report['completed_stages'],report['expected_stages'])
            self.assertEqual(report['planner_goals_sent'],0)

    def test_stabilization_requires_two_continuous_seconds_at_home(self):
        with physical_controller() as (node,_):
            airborne(node);node.publish=Mock();node.request_leg=Mock()
            node.mission_tick(10.,node.home,.4)
            node.mission_tick(11.,node.home+[.2,0.,0.],.4)
            self.assertIsNone(node.arrived_since)
            node.mission_tick(12.,node.home,.4)
            node.mission_tick(13.99,node.home,.4)
            self.assertEqual(node.phase,'HOLD')
            node.mission_tick(14.,node.home,.4)
            self.assertEqual(node.phase,'FINAL_HOLD')
            self.assertEqual(node.final_hold_since,14.)
            node.request_leg.assert_not_called()

    def test_fixed_hover_retains_venue_and_sent_reference_checks(self):
        with physical_controller() as (node,_):
            airborne(node);node.phase='FINAL_HOLD';node.final_hold_since=0.
            node.publish=Mock();node.land=Mock()
            for sent,point,error in (
                ([],node.home,'No sent-reference'),
                ([(99_000_000_000,node.home,np.zeros(3))],node.home,'reference is stale'),
                ([(100_000_000_000,node.home,np.zeros(3))],node.home+[0.,2.,0.],'outside venue')):
                with self.subTest(error=error):
                    node.sent_references=deque(sent)
                    with self.assertRaisesRegex(ValueError,error):node.mission_tick(25.,point,.4)
                    node.publish.assert_not_called();node.land.assert_not_called()
            with self.assertRaisesRegex(ValueError,'outside venue'):
                node.target_ok(node.home+[0.,2.,0.])
            node.fresh=Mock(return_value=False)
            with self.assertRaisesRegex(ValueError,'stale local pose during'):
                node.check_landing_feedback()

    def test_default_full_mission_keeps_rounds_and_six_second_arrival_reset(self):
        with physical_controller(hover_only=False) as (node,_):
            self.assertEqual(node.mission.round_trips,5)
            self.assertEqual(len(node.mission.PHASES),20)
            self.assertFalse(node.hover_only)
            airborne(node);node.publish=Mock();node.request_leg=Mock()
            node.mission_tick(10.,node.home,.4);node.mission_tick(12.,node.home,.4)
            node.request_leg.assert_called_once_with('outbound',node.far)
            node.phase='FINAL_HOLD';node.final_hold_since=12.
            node.mission_report=NS(data={},completed=Mock());node.land=Mock()
            node.mission_tick(18.,node.home+[.2,0.,0.],.4)
            self.assertEqual(node.final_hold_since,18.)
            node.land.assert_not_called()
            node.mission_tick(23.99,node.home,.4);node.land.assert_not_called()
            node.mission_tick(24.,node.home,.4)
            node.land.assert_called_once_with('Demonstration return hover complete')
            node.mission_report.completed.assert_called_once_with('HOLD_HOME_6S')

    def test_cli_flags_preserve_readiness_default_and_validate_duration(self):
        required=['--mission-file','mission.json','--alignment-path','alignment.py',
                  '--planner-config','planner.yaml']
        for parser,base in ((arguments,required),(super_demo.arguments,required[:2])):
            defaults=parser(base)
            self.assertFalse(defaults.hover_only);self.assertFalse(defaults.execute)
            selected=parser(base+['--hover-only','--hover-seconds','20'])
            self.assertTrue(selected.hover_only);self.assertEqual(selected.hover_seconds,20.)
        for value in ('0','-1','nan','inf'):
            with self.subTest(value=value),redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    super_demo.arguments(required[:2]+['--hover-seconds',value])
            with self.assertRaisesRegex(ValueError,'finite and positive'):
                DemoMissionControl('unused.json','unused-report.json',hover_only=True,
                                   hover_seconds=float(value))
        with self.assertRaisesRegex(ValueError,'explicit on-site supervision'):
            DemoMissionControl('unused.json','unused-report.json',execute=True,hover_only=True)

    def test_preparation_forwards_hover_mode_and_duration(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source=folder/'source.json'
            source.write_text(json.dumps(fixture()))
            for name in ('super_demo_control.py','demo_mission.py','create_live_venue_profile.py',
                         'live_venue.py','venue_geometry.py','apply_super_venue_bounds.py'):
                (folder/name).touch()
            args=super_demo.arguments(['--mission-file',str(source),'--scripts-dir',str(folder),
                '--flight-height','1.0','--hover-only','--hover-seconds','20','--ros-env'])
            runner=NS(folder=folder,path=folder/'preparation.json',report={},
                session=NS(load=Mock(),config=dict(planner_config_path='planner.yaml',
                    venue_reference_path='venue.json')),open_qgc=Mock(),prepare=Mock())
            with patch.object(super_demo,'arguments',return_value=args),\
                 patch.object(super_demo,'DemoPreparation',return_value=runner),\
                 patch.object(Path,'home',return_value=folder),\
                 patch.object(super_demo.subprocess,'run',return_value=NS(returncode=0)) as run:
                self.assertEqual(super_demo.main(),0)
            command=run.call_args.args[0]
            self.assertIn('--hover-only',command)
            self.assertEqual(command[command.index('--hover-seconds')+1],'20.0')
            self.assertNotIn('--execute',command)
            self.assertEqual(json.loads((folder/'flight_mission.json').read_text())
                             ['flight']['height_fc_above_floor_m'],1.)


if __name__=='__main__':unittest.main()
