"""Ground contact is a measured-state transition, not a change to venue geometry."""
from copy import deepcopy
import unittest

from test_demo_mission import fixture
from demo_mission import DemoMission,TakeoffFloorPolicy


def incident_mission():
    import numpy as np
    from rviz_venue_calibration import selected_geometry,cap_selected_box,apply_geometry
    config=fixture()
    for side in ('front','rear','left','right'):
        config['vehicle']['centre_to_'+side+'_m']=.136
    config['vehicle']['centre_to_top_m']=.165
    observation=dict(ground_fc_xyz=[0.,0.,0.],ground_fc_yaw_rad=0.,origin=[0.,0.,0.])
    geometry=cap_selected_box(
        selected_geometry([0.,1.,1.],[0.,-1.,1.],observation,np.eye(3),.5),2.,2.)
    config=apply_geometry(config,geometry,observation,'regression-fixture')
    return DemoMission(config)


INCIDENT_POINT=[-.015282238004557294,-.01355472174047951,.06397434327751399]


class Tests(unittest.TestCase):
    def test_recorded_ground_to_rise_transition_accepts_small_height_offset(self):
        mission=incident_mission();original=deepcopy(mission.config)
        for landed_state in (1,2,3):
            with self.subTest(landed_state=landed_state):
                policy=TakeoffFloorPolicy()
                # READY FC Z=-.0345406942 versus captured ground Z=-.0070978897.
                policy.check(mission,[0.,0.,.085-.02744280453771353],'READY',1)
                clearances=policy.check(mission,INCIDENT_POINT,'RISE',landed_state)
                self.assertAlmostEqual(clearances['floor'],-.02102565672248602)
                self.assertEqual(policy.last_check['measured_floor_lower_bound_m'],-.10)
                self.assertFalse(policy.takeoff_complete)
        self.assertEqual(mission.config,original)

    def test_floor_allowance_is_bounded_and_never_bypasses_sides_or_ceiling(self):
        mission=incident_mission()
        for point in ([0.,0.,.085-.10001],[-.365,0.,.085],[5.165,0.,.085],
                      [0.,.865,.085],[0.,-.865,.085],[0.,0.,1.836]):
            with self.subTest(point=point),self.assertRaisesRegex(ValueError,'outside venue'):
                TakeoffFloorPolicy().check(mission,point,'RISE',2)

    def test_unknown_or_landing_state_cannot_enable_initial_contact(self):
        for state in (None,0,4):
            with self.subTest(state=state),self.assertRaisesRegex(ValueError,'outside venue'):
                TakeoffFloorPolicy().check(incident_mission(),INCIDENT_POINT,'RISE',state)

    def test_positive_climb_latches_strict_floor_even_if_fc_state_flaps(self):
        mission=incident_mission();policy=TakeoffFloorPolicy()
        policy.check(mission,[0.,0.,.186],'RISE',2)
        self.assertTrue(policy.takeoff_complete)
        for state in (1,2,3):
            with self.subTest(state=state),self.assertRaisesRegex(ValueError,'outside venue'):
                policy.check(mission,INCIDENT_POINT,'RISE',state)

    def test_leaving_rise_closes_allowance_even_without_positive_height_sample(self):
        mission=incident_mission()
        for phase in ('HOLD','WAIT_PLAN','TRACK','END_HOLD','FINAL_HOLD'):
            policy=TakeoffFloorPolicy()
            with self.subTest(phase=phase),self.assertRaisesRegex(ValueError,'outside venue'):
                policy.check(mission,INCIDENT_POINT,phase,1)
            with self.assertRaisesRegex(ValueError,'outside venue'):
                policy.check(mission,INCIDENT_POINT,'RISE',1)

    def test_landing_needs_on_ground_for_bounded_contact_allowance(self):
        mission=incident_mission();policy=TakeoffFloorPolicy()
        policy.check(mission,[0.,0.,1.],'TRACK',2)
        with self.assertRaises(ValueError):policy.check(mission,INCIDENT_POINT,'LANDING',2)
        policy.check(mission,INCIDENT_POINT,'LANDING',1)
        with self.assertRaises(ValueError):policy.check(mission,[0.,0.,-.016],'LANDING',1)

    def test_reference_geometry_remains_strict_after_allowed_measurement(self):
        mission=incident_mission()
        TakeoffFloorPolicy().check(mission,INCIDENT_POINT,'RISE',2)
        with self.assertRaisesRegex(ValueError,'outside venue'):mission.check(INCIDENT_POINT)
        mission.check([0.,0.,.085])


if __name__=='__main__':unittest.main()
