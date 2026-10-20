# RViz lateral venue calibration validation

The calibration path was tested in an isolated ROS 2 Humble Docker environment.
No aircraft, PX4 flight, or physical boundary survey was exercised.

## Automated checks

The following unittest invocation passed **67 tests**:

```bash
python3 -m unittest \
  test_rviz_venue_calibration test_demo_mission test_live_venue \
  test_demo_tracking_ros test_demo_test_release test_demo_venue
```

Coverage includes environmental point conversion without a vehicle lever arm,
tilted localization axes, unequal pick heights, narrower/capped venue widths,
left/right ordering, immutable session anchors, source/origin changes, release
checksums, existing mission behavior, and calibration-before-planner ordering.
Calibration-only preparation does not launch the planner.

The live integration command also completed successfully:

```bash
xvfb-run -a python3 tests/test_live_venue_ros.py
```

Its four cases cover:

- Fresh capture, planner configuration readback, and check-only READY.
- Rejection of a mismatched running planner configuration.
- Rejection of a native control publisher.
- Actual RViz startup under Xvfb, DDS point messages, preview markers, confirmation
  service, saved calibration, and check-only READY with the same fixed anchor.

The RViz case programmatically sends clicks. It verifies the message and
preparation integration, not a human operator's ability to identify a particular
physical wall or the accuracy of a live cloud. Those remain onsite checks.
The source patch/mission regressions do not establish physical flight performance.
