# Step 1: stationary hover-drift diagnosis

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
  'https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/main/scripts/analysis/hover_drift_check.py' \
  -o "$diagnostic_dir/hover_drift_check.py"
/usr/bin/python3 -m py_compile "$diagnostic_dir/hover_drift_check.py"
install -m 0644 "$diagnostic_dir/hover_drift_check.py" "$HOME/Downloads/hover_drift_check.py"
printf 'Diagnostic installed: %s\n' "$HOME/Downloads/hover_drift_check.py"
BASH
```

For a reproducible run, replace `main` in the download URL with the reviewed
commit SHA. The script's SHA-256 is included in its report. No complete demo
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
