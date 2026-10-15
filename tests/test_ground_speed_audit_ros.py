"""Ground-only high-speed controller checks with simulated ROS telemetry."""
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/planning'),str(ROOT/'scripts/flight')]
from ground_speed_audit import GroundSpeedAudit
from super_flight_control import SuperFlightControl
from bounded_reference_preview import Plan
from test_super_flight_control_ros import run,speed_plan


def guards():
    fast=object.__new__(GroundSpeedAudit)
    try:SuperFlightControl.__init__(fast,execute=True,distance=5.)
    except ValueError as exc:assert 'ground-audit only' in str(exc)
    else:raise AssertionError('High-speed profile permitted flight initialization')
    base=object.__new__(SuperFlightControl)
    try:SuperFlightControl.__init__(base,execute=True,distance=5.)
    except ValueError:pass
    else:raise AssertionError('Standard flight permitted a 5 m goal')
    plan=Plan(speed_plan(np.zeros(3),np.array([5.,0.,0.]),100.))
    samples=[plan.evaluate(float(t)) for t in np.linspace(0,plan.duration,1001)]
    speed=max(np.linalg.norm(v) for _,v,_ in samples)
    acceleration=max(np.linalg.norm(a) for _,_,a in samples)
    assert abs(speed-2.)<1e-8 and abs(acceleration-1.)<1e-8
    assert np.allclose(plan.endpoint,[5.,0.,0.]) and np.linalg.norm(plan.end_velocity)<1e-8
    assert np.linalg.norm(plan.end_acceleration)<1e-8
    print('PASS: unchanged flight distance/speed guards and smooth 5 m S-curve 2/1 boundary',flush=True)


if __name__=='__main__':
    guards()
    for case in ('audit_speed5_hold','audit_speed5_overspeed','audit_speed5_corridor','audit_speed5_restart'):
        run(case)
    for case in ('audit_hold','flight_complete','missing_rc'):
        run(case)
