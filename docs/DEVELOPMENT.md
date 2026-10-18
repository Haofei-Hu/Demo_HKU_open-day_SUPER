# Development and verification

Run these checks in an isolated development ROS domain, not on the active aircraft graph.

Pure repository checks run from the checkout. ROS graph tests require Humble, MAVROS message/service definitions, and SUPER `PositionCommand`; use an environment and ROS domain isolated from the aircraft.

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:$PWD/scripts/localization:$PWD/scripts/flight:$PWD/scripts/planning:$PWD/scripts/analysis:$PWD/scripts/maintenance${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=180
export ROS_LOCALHOST_ONLY=1
python3 tests/test_repository_layout.py
python3 tests/test_super_session.py
python3 tests/test_super_motion_profiles.py
python3 tests/test_super_3d_goal_profile.py
python3 tests/test_ground_height_goal.py
python3 tests/test_bounded_reference.py
python3 tests/test_super_exact_start.py
python3 tests/test_control_chain_metrics.py
python3 tests/test_control_chain_metrics_ros.py
python3 tests/test_hover_reference.py
python3 tests/test_bag_to_xyz.py
python3 scripts/maintenance/build_first_hover_terminal.py --check
python3 tests/test_aligned_pose_preview.py
python3 tests/test_control_bridge_preview.py
python3 tests/test_flight_pose_cov_transport.py
python3 tests/test_first_hover_control.py
FIRST_HOVER_TEST_MODULE=first_hover_terminal python3 tests/test_first_hover_control.py
```


## Verification scope

The current scripts have offline geometry/report checks, private-domain ROS controller checks, and real-SUPER virtual replay evidence. These are different from physical flight validation. See [verification and known issues](VALIDATION.md).
