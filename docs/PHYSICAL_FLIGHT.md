# Laboratory flight: 1 m hover target

Use `super_demo.py` for the actual aircraft. This is separate from the virtual
`super_demo_test.py` rehearsal.

| Setting | Laboratory trial |
|---|---|
| Takeoff, outbound and return hover targets | FC centre **1.0 m above the floor** |
| Outer ceiling | **2.0 m above the floor**, including the aircraft's top extent |
| Total lateral operating width | At most **2.0 m**, centred on takeoff and clipped to the selected sides |
| Forward venue length / rear placement | 5.8 m / FC centre 0.5 m from the rear boundary |
| Route | 4.5 m outbound and return, five round trips |
| Finish | Hover at home for 6 seconds, land, confirm disarmed |
| Planning targets / acceptance bounds | 2 m/s and 1 m/s² / 4 m/s and 2 m/s² |

**The 1 m hover target is not a 1 m altitude limit.** SUPER can change altitude
and follow curved paths to avoid obstacles inside the operating box. The same
vehicle-inclusive boundaries constrain search, corridors and the controller.
Returning means planning back to the original hover point, not replaying the
outbound path; the initial heading is retained.

## 1. Install the matching release once

Finish any running task and keep the aircraft landed and disarmed before updating.
This release fixes final-reference completion in the Python supervisor. No new
C++ build is required if all ten venue markers are already installed, including
`VENUE_BACKUP_WAYPOINTS_V1`. Older installations need the
[one-time patch and build commands](QUICK_START.md#install-or-update-once).

```bash
(
set -e
revision='3256ff8f87faa8d3c124f49b4cef43fbcdc95071'
update_dir=$(mktemp -d)

curl -q --noproxy '*' -fL --connect-timeout 10 --max-time 30 \
  "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/$revision/scripts/maintenance/install_demo_test.py" \
  -o "$update_dir/install_demo_test.py"

python3 "$update_dir/install_demo_test.py" --revision "$revision"
)
```

The installer backs up replaced scripts and preserves local mission geometry and
alignment. A normal reboot does not require another download or build.

## 2. Prepare the aircraft configuration once

For the discussed aircraft, the operator confirmed a symmetric X frame, 205 mm
diagonal wheelbase, centred FC and 5-inch props. The nominal horizontal extent is:

```text
FC to outer propeller edge = 205 / (2 × sqrt(2)) + 127 / 2
                          ≈ 136 mm in each horizontal direction
```

The confirmed vertical extents are 16.5 cm above and 8.5 cm below the FC centre.
The horizontal values remain calculated estimates. Confirm that the actual
propeller sweep, LiDAR, battery and other equipment fit within the configured
envelope; increase any extent that does not cover the aircraft.

This command creates `open_day_mission_nominal.json` and labels the estimates.
It does not modify the canonical mission or start ROS or flight. If an existing
measured file is more accurate, use that file in step 3 instead; do not replace
measured dimensions with estimates.

```bash
python3 - <<'PY'
from pathlib import Path
from datetime import datetime
import json
import shutil

base = Path.home() / 'Downloads'
c = json.loads((base / 'open_day_mission.json').read_text())
if c.get('execution_context') is not None or 'SYNTHETIC' in str(c.get('status', '')).upper():
    raise SystemExit('A rehearsal fixture cannot be used as a physical mission')
for side in ('front', 'rear', 'left', 'right'):
    c['vehicle']['centre_to_' + side + '_m'] = 0.136
c['vehicle'].update(centre_to_top_m=0.165, centre_to_bottom_m=0.085)
c['vehicle']['note'] = (
    'Nominal horizontal extents: symmetric 205 mm X frame, 5-inch props, '
    'centred FC. Verify equipment fits before flight. '
    'Vertical extents confirmed by operator.'
)
c['takeoff'].update(
    forward_from_rear_m=0.5,
    distance_from_left_m=None,
    heading_along_length_confirmed=True
)
c['flight'].update(height_fc_above_floor_m=1.0, round_trips=5)
c['sequence'][0] = 'TAKEOFF_TO_1_0M'
c['status'] = 'NOMINAL_ENVELOPE_REQUIRES_ON_SITE_VERIFICATION'
p = base / 'open_day_mission_nominal.json'
if p.exists():
    backup = p.with_name(p.name + '.backup_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(p, backup)
p.write_text(json.dumps(c, indent=2) + '\n')
print('Saved:', p)
print('Horizontal geometry is estimated. No flight was started.')
PY
```

The lateral offset deliberately remains unset: fresh RViz selection supplies it.
Point the nose along the 5.8 m length and use the actual rear offset; side clicks
do not measure the rear boundary. The floor conversion assumes the FC is 0.085 m
above the takeoff floor. See [aircraft geometry](FLIGHT_SITE_MEASUREMENTS.md) if
the installation changes.

## 3. Prepare after each reboot or new placement

Keep the aircraft stationary and disarmed. Have an on-site RC operator with
POSCTL/manual control and takeover available. Preparation checks localization,
source identities, PX4 parameter readbacks and operating geometry. For example,
the physical profile expects `MPC_XY_VEL_MAX=2.0`; a mismatch is reported rather
than silently changed.

In **terminal A**:

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_nominal.json" \
  --flight-height 1.0 \
  --calibrate-venue --rear-offset 0.5 \
  --venue-width 2 --venue-ceiling 2 \
  --motion-policy observe \
  --execute --on-site-supervised
```

This opens or reuses QGC, starts localization, captures a fresh alignment
quaternion and prepares external vision. In the frozen RViz view:

1. Select the actual **LEFT**, then **RIGHT** boundary, looking along the nose.
2. Check the cyan box: up to 1 m on either side of takeoff, clipped to nearer
   selected sides, and a 2 m outer ceiling.
3. Type `confirm` in terminal A. Use `reset` for new picks on the same snapshot,
   or `refresh` for a new snapshot.
4. Let preparation continue to **`Phase: READY`**. Keep terminal A running.

The selection viewer closing after confirmation does not end preparation.
Each restarted localization session needs new picks. The source mission is
preserved; `flight_mission.json` records this run's effective 1 m target and box.
Selecting points does not arm the aircraft.

Always include `--flight-height 1.0`: the existing software default is still
1.5 m. With the 0.085 m ground FC height, the nominal rise is 0.915 m.
**Do not change `--venue-ceiling 2` to 1.**

For preparation checks only, omit `--execute --on-site-supervised`. That mode
ends after readiness checks without flight. Do not add `--calibration-only` to
the complete preparation flow.

## 4. Initiate physical flight after READY

After confirming the configured envelope, clear route and RC takeover, use
**terminal B**:

```bash
source /opt/ros/humble/setup.zsh
ros2 service call /super_demo/start std_srvs/srv/Trigger '{}'
```

**This call initiates physical flight:** when accepted, the controller requests
OFFBOARD, arms and executes the route. A rejected request does not authorize
bypassing preparation with a separate mode or arming command.

## 5. Monitor, land early and save results

Inspect status from terminal B:

```bash
timeout 10s ros2 topic echo \
  /super_debug/super_flight_status std_msgs/msg/String \
  --no-daemon --qos-reliability best_effort \
  --qos-durability volatile --full-length --field data
```

Request early landing while the controller is running:

```bash
ros2 service call /super_demo/land std_srvs/srv/Trigger '{}'
```

Use RC takeover if control is abnormal. `Ctrl+C` is not a hover or landing
command. Only stop the background stack after landing and disarming:

```bash
python3 ~/Downloads/super_session.py stop
```

Keep the complete `~/Downloads/super_demo_reports/<timestamp>/` directory:

- `preparation.json`: preparation steps, errors and log paths.
- `venue_calibration.json`: clicked boundaries, chosen caps and session anchor.
- `flight_mission.json`: effective aircraft geometry, 1 m target and operating box.
- `report.json`: flight stages, completed round trips and final result.

Configuration, controller startup gates and live check-only venue handoff have
been tested. **The complete 1 m physical mission on this aircraft has not yet
been verified.** A successful virtual rehearsal or READY state is not a physical
flight completion record.
