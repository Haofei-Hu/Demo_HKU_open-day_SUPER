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
    config['takeoff'].update(forward_from_rear_m=.5,distance_from_left_m=1.4)
    for side in ('front','rear','left','right'):config['vehicle']['centre_to_'+side+'_m']=.2
    config['vehicle'].update(centre_to_top_m=.1,centre_to_bottom_m=.085)
    return config


class Tests(unittest.TestCase):
    def test_unmeasured_template_cannot_become_a_flight_profile(self):
        template=json.loads((ROOT/'configs/demonstration/open_day_mission.json').read_text())
        with self.assertRaisesRegex(ValueError,'Missing/invalid measured geometry'):DemoMission(template)

    def test_entire_vehicle_fits_on_both_legs_and_variable_start(self):
        for rear,left in ((.5,1.4),(.31,.31),(.8,2.49)):
            cfg=fixture();cfg['takeoff'].update(forward_from_rear_m=rear,distance_from_left_m=left)
            mission=DemoMission(cfg)
            for forward in (0,2.25,4.5,2.25,0):mission.check((forward,0,2.))
            self.assertEqual(mission.waypoints()['return_hover'],[0,0,2])
            self.assertEqual(cfg['flight']['return_hover_s'],6.)

    def test_short_edge_does_not_allow_a_centre_on_a_wall(self):
        for key,value in (('forward_from_rear_m',0.),('distance_from_left_m',0.),
                          ('forward_from_rear_m',1.1),('distance_from_left_m',2.8)):
            cfg=fixture();cfg['takeoff'][key]=value
            with self.assertRaisesRegex(ValueError,'outside venue'):DemoMission(cfg)
        cfg=fixture();cfg['vehicle']['centre_to_top_m']=.71
        with self.assertRaisesRegex(ValueError,'outside venue'):DemoMission(cfg)

    def test_body_and_ten_cm_allowance_are_not_silently_removed(self):
        cfg=fixture();before=deepcopy(cfg);mission=DemoMission(cfg)
        self.assertEqual(cfg,before)
        for p in ((-.301,0,2),(5.101,0,2),(0,1.201,2),(0,0,2.701)):
            with self.assertRaises(ValueError):mission.check(p)

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
        self.assertEqual(mission.waypoints()['far_hover'],[1.,0.,2.])
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
