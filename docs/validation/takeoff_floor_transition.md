# Ground contact during takeoff

The physical run `20261020_205941_900277` stopped 2.60 seconds into `RISE`.
Its measured FC height in venue coordinates was 0.0639743433 m. With the
configured 0.085 m FC-to-bottom extent, the calculated floor clearance was
−0.0210256567 m. The operator reported that the aircraft had not lifted off.
The old error did not record the contemporaneous PX4 landed-state value.

At READY, measured FC Z was −0.0345406942 m while the immutable venue ground
anchor was −0.0070978897 m: a −0.0274428045 m offset. This passed the existing
0.10 m grounded position check. However, the previous floor exemption depended
only on `ON_GROUND`. A transition away from that flag immediately rejected the
previously admitted ground-height offset.

## Corrected behavior

- Measured positions use the existing 0.10 m ground-offset band during initial
  arming/rise. `IN_AIR` or `TAKEOFF` alone does not establish positive clearance.
- Measured clearance above 0.10 m, or departure from the initial takeoff phases,
  permanently closes that initial allowance. Later `ON_GROUND` messages cannot
  reopen it during flight. Confirmed on-ground landing has bounded contact
  allowance.
- Side walls and ceiling remain strict throughout. Below-ground estimates
  beyond the ground band are rejected. Missing/unknown landed-state values
  cannot enable the initial allowance.
- Outgoing positions, accepted goals and polynomial references always use the
  strict floor. Checking a rising reference cannot change the measured-state
  latch.
- The captured venue anchor, planner halfspaces, hull extents, target height,
  sensor timestamps, freshness limits and PX4 parameters are unchanged.
- Reports include the phase, landed state, measured clearance, active lower
  bound and measured/reference distinction. Both READY height messages now
  print the final bound control reference; current measured position is separate.

## Verification

The regression suite replays the reported position with ON_GROUND, IN_AIR and
TAKEOFF feedback, and checks excessive negative offsets, all side/ceiling
bounds, strict airborne behavior, landing contact and reference rejection.
ROS-controller dispatch tests exercise the inherited envelope check and prevent
publication of a reference below the actual floor.

All **121 tests passed** in the isolated ROS 2 Humble test container:

```bash
python3 -m unittest \
  test_takeoff_floor test_takeoff_floor_ros test_demo_tracking_ros \
  test_live_venue test_demo_mission test_demo_test_release \
  test_demo_accepted_goal_ros test_demo_rest_handoff_ros \
  test_left_rehearsal_ros test_rviz_venue_calibration test_demo_venue
```

The inherited handover numerical checks also passed, and the generated
`first_hover_terminal.py` matches its canonical source. The release manifest
hashes and installer's measurement-preservation tests passed in the same suite.

These are software tests with synthetic telemetry and recorded numeric inputs.
They do not establish physical takeoff, estimator accuracy or hover stability.
The earlier READY vision-freshness failures are a separate issue.
