"""Exercise measured/reference dispatch in the real controller classes."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import unittest
import numpy as np

from test_demo_tracking_ros import DemoMissionControl,SuperFlightControl,VenueFrame
from test_takeoff_floor import incident_mission,INCIDENT_POINT


class Tests(unittest.TestCase):
    def control(self):
        control=object.__new__(DemoMissionControl)
        control.phase='RISE';control.extended=NS(landed_state=2)
        control.reference=NS(ground=[0.,0.,0.],yaw=0.)
        control.frame=VenueFrame([0.,0.,0.],[0.,0.,0.,1.],incident_mission())
        control.mission_report=NS(data={})
        control.execute=True
        return control

    def position(self):
        # Venue coordinates include the FC's 8.5 cm ground clearance.
        return np.array(INCIDENT_POINT)-[0.,0.,.085]

    def test_inherited_validation_uses_measured_policy(self):
        control=self.control()
        SuperFlightControl.check_envelope(control,self.position(),0.,False)
        record=control.mission_report.data['measured_floor_check']
        self.assertEqual(record['phase'],'RISE');self.assertEqual(record['landed_state'],2)
        self.assertAlmostEqual(record['clearance_m']['floor'],-.02102565672248602)
        self.assertEqual(record['measured_floor_lower_bound_m'],-.1)

    def test_outgoing_reference_cannot_use_measured_floor_allowance(self):
        control=self.control();control.debug_pub=Mock();control.native_pub=Mock()
        control.check_measured_position(self.position())
        with self.assertRaisesRegex(ValueError,'outside venue'):
            control.publish(self.position(),0.,native=True)
        control.debug_pub.publish.assert_not_called()
        control.native_pub.publish.assert_not_called()

    def test_failure_records_phase_landed_state_and_measurement_source(self):
        control=self.control()
        with self.assertRaises(ValueError):control.check_measured_position([0.,0.,-.101])
        record=control.mission_report.data['venue_envelope_failure']
        self.assertEqual(record['source'],'measured FC position')
        self.assertEqual(record['phase'],'RISE')
        self.assertEqual(record['landed_state'],2)
        self.assertAlmostEqual(record['clearance_m']['floor'],-.101)

    def test_latched_floor_cannot_be_reopened_by_an_on_ground_message(self):
        control=self.control();control.check_measured_position([0.,0.,.101])
        control.extended.landed_state=1
        with self.assertRaisesRegex(ValueError,'outside venue'):
            SuperFlightControl.check_envelope(control,self.position(),0.,False)

    def test_target_evaluation_does_not_latch_measured_airborne_state(self):
        control=self.control()
        control.target_ok([0.,0.,.915])
        self.assertFalse(control.frame.floor_policy.takeoff_complete)
        control.check_measured_position(self.position())


if __name__=='__main__':unittest.main()
