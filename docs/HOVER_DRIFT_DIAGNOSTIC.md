# Hover drift diagnosis: ground observations

Use this first after a hover with lateral sway. Keep the aircraft physically
stationary on the floor, disarmed, with motors stopped. The measurements describe
the ground baseline; they do not reproduce propeller vibration in flight.

`hover_drift_check.py` is a standalone, read-only ROS 2 Humble observer. It reads
live FAST-LIO/PX4 parameters, observes raw odometry and optional downstream poses,
IMU streams and registered-cloud metadata, and writes a local diagnostic bundle.
It never publishes flight setpoints, arms, switches mode, changes parameters,
starts/restarts the stack, or recalibrates alignment. It does not require the
vision bridge to be running. A bounded wait and error report cover missing data.

## Download only the diagnostic

In the onboard computer's NoMachine terminal, run:

```bash
bash --noprofile --norc <<'BASH'
set -e
mkdir -p "$HOME/Downloads"
diagnostic_dir=$(mktemp -d "$HOME/Downloads/hover_drift_tool_XXXXXX")
curl -q -fL --connect-timeout 10 --max-time 60 \
  'https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/a252be4165bf0f0c64217fec5a84a6ae5b578b85/scripts/analysis/hover_drift_check.py' \
  -o "$diagnostic_dir/hover_drift_check.py"
/usr/bin/python3 -m py_compile "$diagnostic_dir/hover_drift_check.py"
install -m 0644 "$diagnostic_dir/hover_drift_check.py" "$HOME/Downloads/hover_drift_check.py"
printf 'Diagnostic installed: %s\n' "$HOME/Downloads/hover_drift_check.py"
BASH
```

The download is pinned to the reviewed diagnostic source commit. The script's SHA-256 is included in its report. No complete demo
update or C++ rebuild is required for this additional observer.

## Start or reuse the existing core, then observe

Finish any active flight/controller session first. QGC must release its direct
serial connection when MAVROS owns the flight controller. Leave a working
localization session running so its current settings can be measured.

```bash
bash --noprofile --norc <<'BASH'
set -e
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/fastlio_ws/install/setup.bash"
source "$HOME/super_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
/usr/bin/python3 "$HOME/Downloads/super_session.py" start
/usr/bin/python3 "$HOME/Downloads/hover_drift_check.py" --duration 120 --wait 30
BASH
```

The existing manager's `start` action reuses a managed session, or starts only
LiDAR, FAST-LIO, MAVROS and the body-pose helper. It does not initialize external
vision or start a controller. Do not follow its generic suggestion to run
`align` for this diagnostic: alignment/vision initialization can change
`EKF2_EV_CTRL` and would change the baseline being investigated.

A new core uses the installed FAST-LIO YAML; an existing session retains its
current overrides. This is not automatically the configuration used in a prior
flight. Read the captured live parameters before comparing runs. This command
deliberately does not request a restart or a new extrinsics override.

If `start` reports unmanaged processes, missing files, an exited component or an
environment mismatch, keep that output and resolve it before continuing. Do not
kill processes or start duplicate nodes just to get a report. If no full flight
workflow is installed, a manually running LiDAR/LIO/MAVROS stack can instead be
observed by omitting the manager command.

The observer waits for raw `/Odometry` and fresh connected, disarmed, on-ground
MAVROS telemetry. It stops if the aircraft arms, leaves the ground or loses that
telemetry. The observation ends after 120 seconds, without operator input.

## Send the result before changing parameters

The terminal prints `RESULT`, the summary and `BUNDLE`. Return that output and
the printed `diagnostic.zip` file. A completed capture is not a flight-readiness
or localization-accuracy pass. Missing optional streams/parameter services are
reported explicitly; a core-only session normally has no aligned/vision poses.
PX4 parameter readbacks come from the MAVROS parameter cache; the observer does
not force a refresh or claim that this cache is an independent live FC query.

The bundle contains raw scalar observations, summaries and parameter-read
results. It does not contain full point clouds or a rosbag. Header-age estimates
assume compatible clocks and include callback scheduling; IMU variation here
cannot establish airborne vibration severity.

After reviewing this baseline, isolate the RViz display and, if indicated,
perform a controlled comparison of localization settings or investigate
mechanical vibration. Change one condition at a time. Do not infer FAST-LIO
failure merely from sparse points or small EKF innovations.

## Step 2: compare the same cloud with two display durations

After reviewing the stationary capture, keep the same localization session and
the aircraft disarmed and physically fixed. Read any missing filter parameters
one at a time; a parameter-service timeout does not itself mean odometry stopped:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
for parameter in point_filter_num filter_size_surf filter_size_map; do
  printf '\n%s\n' "$parameter"
  timeout 15s ros2 param get /laser_mapping "$parameter" || \
    printf 'Read failed; retain the output. No value was assumed or changed.\n'
done
```

Download [the cloud comparison configuration](../configs/rviz/hover_drift_cloud_compare.rviz)
from the diagnostic revision into `~/Downloads/hover_drift_cloud_compare.rviz`.
Open it in an onboard desktop terminal with the same ROS environment:

```bash
rviz2 -d "$HOME/Downloads/hover_drift_cloud_compare.rviz"
```

This viewer uses `camera_init`, the observed `/cloud_registered` frame, directly.
It needs no auxiliary TF publisher and contains no occupancy-map overlay or goal
tool. Both displays use the same topic, color, point size and QoS; only display
history differs. The configuration changes no FAST-LIO or PX4 parameter.

1. Leave `SingleFrame` checked and `Accumulated1s` unchecked. Select a view that
   includes a stationary wall or other rigid structure, and capture a screenshot.
2. Uncheck `SingleFrame`, check `Accumulated1s`, and wait at least two seconds.
   Keep the same viewpoint and capture a second screenshot. Use only one display
   at a time.

If the accumulated view fills in a stable, coherent surface, limited points per
frame can explain the sparse appearance. If a fixed wall produces persistent
double edges, thick layers or overall motion, retain that evidence for checking
registration, geometry and timing. Accumulation can also hide temporal jitter;
a visually clean result is not a flight-localization acceptance test.

Return the three parameter readbacks and both screenshots, including the display
checkboxes. If the view is empty or a display reports an error, return that status
instead. Do not increase point density, alter extrinsics or restart the LIO merely
to make the comparison look better. A later measured-motion test is still needed
to examine axis directions, scale and behavior while moving.
