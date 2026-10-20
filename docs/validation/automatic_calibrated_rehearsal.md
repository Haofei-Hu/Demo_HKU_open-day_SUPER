# Automatic rehearsal after RViz venue selection

The rehearsal entrypoint now accepts `--calibrate-venue --rear-offset 0.5`.
After the operator confirms the two sides, the same run continues through fresh
map capture, the isolated real-SUPER mission and its final report. The selected
width is retained exactly. Closing the selection viewer is a handoff to the
rehearsal viewer, not a calibration-only exit.

## Verification

The following suite passed **96 tests** in ROS 2 Humble:

```bash
python3 -m unittest \
  test_rviz_venue_calibration test_super_demo_test test_parallel_startup \
  test_demo_mission test_live_venue test_demo_tracking_ros \
  test_demo_test_release test_demo_venue
```

New orchestration cases check automatic continuation, cancellation before planner
startup, preservation of the source mission, five-round completion, the private
ROS domain and no real-aircraft mode/arming requests.

Geometry tests use tilted world axes, an attitude-dependent lever arm and a small
FC/vision heading difference. The frozen scene retains the calibrated FC anchor,
while raw and aligned quaternion pairs remain consistent with their converter.
Both the planning planes and controller bounds match the same selected sides.
A changed source/session or edited frozen anchor is rejected.

`tests/test_live_venue_ros.py` passed all four live DDS cases. Its RViz case was
also exercised with the new continuation: confirm a 4.2 m venue, invoke the real
`capture_demo_scene.py --venue-mission ...` subprocess against live test publishers,
validate the saved scene, and reach physical **check-only** READY with no native
flight output.

## Real-SUPER frozen-map replay

An existing captured ceiling-regression map was replayed in private domain 197.
Synthetic test picks defined left clearance 1.690 m and right clearance 1.317 m,
giving exactly **3.007 m** width. These are test inputs, not a new physical survey.
The FC heading differed from the external-vision heading by 0.8 degrees to
exercise the separate orientation handling. The cloud was not altered.

The real SUPER planner and ideal virtual vehicle completed:

- All 20 ordered stages, including five outbound and five return legs.
- A 0.5 m takeoff target and 4.5 m requested outbound distance.
- The final six-second hover, virtual AUTO.LAND, and confirmed virtual landed/
  disarmed feedback.
- Final result `SUCCESS`, stage `DONE`, and `round_trips_completed: 5`.

The cloud evidence was retained under
`/workspace/.super-test-env/calibrated_replay_vpj_wqej/`; its preparation helper is
`/workspace/.super-test-env/validate_calibrated_replay.py`. The related logs are
`auto_venue_replay.log`, `auto_venue_regression.log`, and `auto_venue_scene_ros.log`.
These paths belong to the test environment, not the onboard installation.

No physical aircraft or PX4 dynamics were used. Hardware startup, manual onsite
point accuracy and physical tracking were not established by this validation.
The physical entrypoint retains its measured-body requirements, supervision
checks and separate `/super_demo/start` request.
