"""Complete route geometry and honest whole-mission completion reporting."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/flight'))
from demo_mission import DemoMission,MissionReport


def fixture():
    config=json.loads((ROOT/'configs/demonstration/open_day_mission.json').read_text())
    config['flight']['round_trips']=1
    config['sequence']=list(DemoMission.PHASES)
    config['takeoff'].update(forward_from_rear_m=.5,distance_from_left_m=1.4)
    for side in ('front','rear','left','right'):config['vehicle']['centre_to_'+side+'_m']=.2
    config['vehicle'].update(centre_to_top_m=.1,centre_to_bottom_m=.085)
    return config


class Tests(unittest.TestCase):
    def test_five_round_trips_require_all_returns_before_success(self):
        config=fixture();config['flight']['round_trips']=5
        mission=DemoMission(config)
        self.assertEqual(len(mission.PHASES),20)
        self.assertEqual(mission.PHASES[2],'ROUND_01_SUPER_OUTBOUND_4_5M')
        self.assertEqual(mission.PHASES[-4],'ROUND_05_SUPER_RETURN_HOME')
        with tempfile.TemporaryDirectory() as folder:
            report=MissionReport(Path(folder)/'report.json',mission,'simulation')
            for stage in mission.PHASES[:14]:report.completed(stage)
            report.finish()
            self.assertEqual(report.data['round_trips_completed'],4)
            self.assertEqual(report.data['result'],'FAIL')
            for stage in mission.PHASES[14:]:report.completed(stage)
            report.finish()
            self.assertEqual(report.data['round_trips_completed'],5)
            self.assertEqual(report.data['result'],'SUCCESS')
        for bad in (0,-1,True,1.5):
            config['flight']['round_trips']=bad
            with self.assertRaisesRegex(ValueError,'positive integer'):DemoMission(config)

    def test_rehearsal_height_is_half_metre_without_editing_flight_specification(self):
        original=fixture();before=deepcopy(original)
        config=DemoMission.ground_rehearsal(original);mission=DemoMission(config)
        self.assertEqual(original,before)
        self.assertEqual(DemoMission(original).hover_height,1.5)
        self.assertEqual(mission.hover_height,.5)
        self.assertEqual(mission.PHASES[0],'TAKEOFF_TO_0_5M')
        self.assertEqual(mission.waypoints()['return_hover'],[0.,0.,.5])
        self.assertEqual(config['vehicle'],original['vehicle'])
        self.assertEqual(config['takeoff'],original['takeoff'])
        self.assertEqual(config['flight']['outbound_forward_m'],4.5)
        self.assertEqual(DemoMission.ground_rehearsal(config),config)
        with tempfile.TemporaryDirectory() as directory:
            report=MissionReport(Path(directory)/'virtual.json',mission,'simulation')
            self.assertEqual(report.data['requested_fc_floor_height_m'],.5)
            with self.assertRaisesRegex(ValueError,'virtual execution only'):
                MissionReport(Path(directory)/'flight.json',mission,'flight')

    def test_physical_1_8m_copy_retains_route_survey_and_ground_height(self):
        original=fixture();original['flight']['round_trips']=5
        before=deepcopy(original)
        config=DemoMission.physical_height(original,1.8)
        mission=DemoMission(config)
        self.assertEqual(original,before)
        self.assertEqual(config['vehicle'],original['vehicle'])
        self.assertEqual(config['takeoff'],original['takeoff'])
        self.assertEqual(mission.waypoints()['home_hover'],[0.,0.,1.8])
        self.assertEqual(mission.waypoints()['far_hover'],[4.5,0.,1.8])
        self.assertEqual(config['sequence'][0],'TAKEOFF_TO_1_8M')
        self.assertEqual(len(mission.PHASES),20)
        self.assertEqual(DemoMission(DemoMission.ground_rehearsal(config)).hover_height,.5)
        with tempfile.TemporaryDirectory() as folder:
            report=MissionReport(Path(folder)/'report.json',mission,'simulation')
            for stage in mission.PHASES:report.completed(stage)
            report.finish()
            self.assertEqual(report.data['requested_fc_floor_height_m'],1.8)
            self.assertEqual(report.data['round_trips_completed'],5)
            self.assertFalse(report.data['physical_flight_verified'])

    def test_physical_height_copy_rejects_fixture_missing_survey_and_ceiling_violation(self):
        for config in (DemoMission.ground_rehearsal(fixture()),
                       dict(fixture(),status='SYNTHETIC_FIXTURE')):
            with self.assertRaisesRegex(ValueError,'measured physical'):
                DemoMission.physical_height(config,1.8)
        config=fixture();config['takeoff']['distance_from_left_m']=None
        with self.assertRaisesRegex(ValueError,'measured geometry'):
            DemoMission.physical_height(config,1.8)
        config=fixture();config['vehicle']['centre_to_top_m']=1.01
        DemoMission(config)  # Fits at 1.5 m, but not at 1.8 m.
        with self.assertRaisesRegex(ValueError,'outside venue'):
            DemoMission.physical_height(config,1.8)

    def test_lower_height_requires_explicit_rehearsal_context(self):
        config=fixture();config['flight']['height_fc_above_floor_m']=.5
        with self.assertRaisesRegex(ValueError,'settings differ'):DemoMission(config)
        config=DemoMission.ground_rehearsal(fixture())
        config['flight']['height_fc_above_floor_m']=1.2
        with self.assertRaisesRegex(ValueError,'settings differ'):DemoMission(config)

    def test_explicit_one_point_five_rehearsal_survives_repeated_conversion(self):
        source=fixture();before=deepcopy(source)
        config=DemoMission.ground_rehearsal(source,1.5)
        self.assertEqual(source,before)
        self.assertEqual(DemoMission.ground_rehearsal(config),config)
        mission=DemoMission(config)
        self.assertEqual(mission.hover_height,1.5)
        self.assertEqual(mission.PHASES[0],'TAKEOFF_TO_1_5M')
        self.assertEqual(mission.waypoints()['far_hover'],[4.5,0.,1.5])
        with self.assertRaises(ValueError):DemoMission.physical_height(config,1.5)

    def test_unmeasured_template_cannot_become_a_flight_profile(self):
        template=json.loads((ROOT/'configs/demonstration/open_day_mission.json').read_text())
        with self.assertRaisesRegex(ValueError,'Missing/invalid measured geometry'):DemoMission(template)

    def test_entire_vehicle_fits_on_both_legs_and_variable_start(self):
        for rear,left in ((.5,1.4),(.31,.31),(.8,2.49)):
            cfg=fixture();cfg['takeoff'].update(forward_from_rear_m=rear,distance_from_left_m=left)
            mission=DemoMission(cfg)
            for forward in (0,2.25,4.5,2.25,0):mission.check((forward,0,2.))
            self.assertEqual(mission.waypoints()['return_hover'],[0,0,1.5])
            self.assertEqual(cfg['flight']['return_hover_s'],6.)

    def test_short_edge_does_not_allow_a_centre_on_a_wall(self):
        for key,value in (('forward_from_rear_m',0.),('distance_from_left_m',0.),
                          ('forward_from_rear_m',1.1001),('distance_from_left_m',2.8)):
            cfg=fixture();cfg['takeoff'][key]=value
            with self.assertRaisesRegex(ValueError,'outside venue'):DemoMission(cfg)
        cfg=fixture();cfg['vehicle']['centre_to_top_m']=1.3001
        with self.assertRaisesRegex(ValueError,'outside venue'):DemoMission(cfg)

    def test_body_bounds_and_source_geometry_are_preserved(self):
        cfg=fixture();before=deepcopy(cfg);mission=DemoMission(cfg)
        self.assertEqual(cfg,before)
        for p in ((-.301,0,2),(5.101,0,2),(0,1.201,2),(0,0,2.701)):
            with self.assertRaises(ValueError):mission.check(p)

    def test_no_extra_ten_cm_corridor_or_inset_is_added_to_venue_geometry(self):
        cfg=fixture();cfg['takeoff'].update(forward_from_rear_m=1.1,distance_from_left_m=.2)
        mission=DemoMission(cfg)
        mission.check((4.5,0.,2.));mission.check((1.,-.8,2.4))

    def test_partial_or_out_of_order_sequence_cannot_succeed(self):
        with tempfile.TemporaryDirectory() as folder:
            report=MissionReport(Path(folder)/'partial.json',DemoMission(fixture()),'simulation')
            with self.assertRaisesRegex(ValueError,'out of order'):report.completed('SUPER_RETURN_HOME')
            for stage in DemoMission.PHASES[:-1]:report.completed(stage)
            report.finish()
            self.assertEqual(report.data['result'],'FAIL')
            self.assertFalse(report.data['physical_flight_verified'])

    def test_complete_simulation_and_flight_evidence_are_distinct(self):
        with tempfile.TemporaryDirectory() as folder:
            for mode in ('simulation','flight'):
                path=Path(folder)/(mode+'.json');report=MissionReport(path,DemoMission(fixture()),mode)
                for stage in DemoMission.PHASES:report.completed(stage)
                report.finish();saved=json.loads(path.read_text())
                self.assertEqual(saved['result'],'SUCCESS')
                self.assertEqual(saved['physical_flight_verified'],mode=='flight')
                with self.assertRaises(ValueError):MissionReport(path,DemoMission(fixture()),mode)

    def test_short_site_route_is_separate_and_cannot_pass_as_demonstration(self):
        original=fixture();before=deepcopy(original)
        short=DemoMission.short_site_fixture(original);mission=DemoMission(short)
        self.assertEqual(original,before)
        self.assertEqual(original['flight']['outbound_forward_m'],4.5)
        self.assertEqual(mission.distance,1.)
        self.assertEqual(mission.waypoints()['far_hover'],[1.,0.,1.5])
        self.assertEqual(mission.PHASES[2],'SUPER_OUTBOUND_1M')
        self.assertEqual(mission.config['flight']['horizontal_speed_limit_mps'],2.)
        self.assertEqual(mission.config['flight']['axis_error_limit_m'],.1)
        with tempfile.TemporaryDirectory() as folder:
            report=MissionReport(Path(folder)/'short.json',mission,'simulation')
            with self.assertRaisesRegex(ValueError,'out of order'):
                report.completed('SUPER_OUTBOUND_4_5M')
            for stage in mission.PHASES:report.completed(stage)
            report.finish();self.assertEqual(report.data['result'],'SUCCESS')
            self.assertFalse(report.data['demonstration_completed'])
            self.assertFalse(report.data['physical_flight_verified'])


if __name__=='__main__':unittest.main()
