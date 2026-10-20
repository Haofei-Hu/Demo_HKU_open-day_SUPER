# Fixed-position 1 m hover

Use this mode for a takeoff, fixed-position hover and landing. It uses the current
physical-flight controller, raw position/velocity interface, localization and
vehicle-inclusive venue checks. It sends no outbound or return navigation goals.
The default demonstration remains five round trips unless `--hover-only` is given.

Install the [matching release](QUICK_START.md#install-or-update-once) once. Use the
existing [aircraft configuration](PHYSICAL_FLIGHT.md#2-prepare-the-aircraft-configuration-once),
with the aircraft landed and disarmed and an on-site RC operator ready to take over.
QGC must release its direct Serial connection when MAVROS owns the USB device.

## Terminal 1: prepare and wait for READY

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_nominal.json" \
  --flight-height 1.0 --hover-only --hover-seconds 20 \
  --calibrate-venue --rear-offset 0.5 \
  --venue-width 2 --venue-ceiling 2 \
  --motion-policy observe \
  --execute --on-site-supervised
```

Use the measured mission filename instead if it contains more accurate aircraft
dimensions. Preparation starts QGC and the stack, measures fresh alignment and
opens the frozen RViz selection. Pick LEFT, then RIGHT, and type `confirm`.
Wait for `FIXED HOVER` and `Phase: READY`; keep terminal 1 running. A failed FC
connection or stale localization cannot enter READY. Preparation does not start flight.

## Terminal 2: start after READY

```bash
source /opt/ros/humble/setup.zsh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 service call /super_demo/start std_srvs/srv/Trigger '{}'
```

This requests OFFBOARD, arming, a rise to **1 m FC-centre height above the floor**,
fixed XY/yaw hover for **20 seconds**, then AUTO.LAND and confirmation of landed
and disarmed state. With the configured 8.5 cm resting FC height, commanded rise
is 0.915 m. Initial stabilization uses the existing two seconds of arrival and
low-speed feedback. Once hovering starts, in-venue tracking error is recorded
without restarting the twenty seconds. There is no whole-flight execution deadline.

To request an earlier landing from terminal 2:

```bash
ros2 service call /super_demo/land std_srvs/srv/Trigger '{}'
```

## Results

Reports remain under `~/Downloads/super_demo_reports/<timestamp>/`. A completed
hover has `flight_mode: fixed_hover`, `fixed_hover_completed: true`, and
`planner_goals_sent: 0`. It does not count as completion of the full demonstration.
The source mission file and PX4 parameters are not changed by this mode; the
existing grounded vision initialization still performs its usual EV_CTRL setup.

Verification: 83 focused checks passed. An isolated ROS backend completed the
actual supervisor lifecycle with a 1 m target, 20.05-second hover, zero navigation
goals and confirmed synthetic landing/disarming. That backend used an ideal
virtual vehicle, with no physical aircraft or PX4 dynamics. It does not validate
the physical aircraft's hover-thrust setting or sway response.
