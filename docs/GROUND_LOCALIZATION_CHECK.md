# Disarmed localization and QoS measurements

Use this diagnostic to investigate takeoff drift before changing estimator noise
or controller gains. It records the raw LIO pose, aligned pose, native vision
input and FC local pose while an operator moves the disarmed aircraft through
known displacements. It does not publish goals, setpoints, arming or mode requests.

## Install the diagnostic once

This adds only the diagnostic script to an existing demonstration installation:

```bash
(
set -e
check_revision='394b8dce2a80f3cf7e5a9a2b7cda9ebd1b970ba1'
check_download=$(mktemp -d "$HOME/Downloads/localization_check_install_XXXXXX")
curl -q --noproxy '*' -fL --connect-timeout 10 --max-time 30 \
  "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/$check_revision/scripts/analysis/ground_localization_check.py" \
  -o "$check_download/ground_localization_check.py"
python3 -m py_compile "$check_download/ground_localization_check.py"
if [ -f "$HOME/Downloads/ground_localization_check.py" ]; then
  cp "$HOME/Downloads/ground_localization_check.py" "$check_download/previous.py"
fi
cp "$check_download/ground_localization_check.py" "$HOME/Downloads/"
)
```

## Run from the current installation

Keep the aircraft stationary during automatic preparation. Run in an onboard
desktop terminal:

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

python3 ~/Downloads/ground_localization_check.py --prepare --restart
```

`--prepare --restart` uses the existing session manager's landed/disarmed stop
check, starts the core with fixed LIO extrinsics, measures and validates a fresh
alignment, and starts the existing vision relay. It uses the manager's existing
vision-parameter policy. It does not start SUPER, a rehearsal, or a flight
controller. The diagnostic itself performs no parameter writes.

If the complete localization chain is already running and no controller owns
native setpoints, omit `--prepare --restart`.

After all four pose topics and a connected/disarmed state are observed:

1. Mark the FC centre's starting position and original nose direction. Press
   Enter while stationary to capture the baseline.
2. Move forward without turning (about 0.5 m is convenient). Once stationary,
   type the **actual measured distance in metres** and press Enter.
3. Return to the original position and heading. Press Enter.
4. Repeat for leftward translation, upward translation and leftward rotation.
   Rotation is entered in degrees. Rotate about the FC centre; movement of the
   physical pivot is included in the measured displacement.

Each stationary measurement lasts three seconds. `--window` changes the
measurement duration; it is not a mission timeout. The script waits for operator
input without a movement deadline. `Ctrl+C` saves a partial report.

For an initial stationary-only observation:

```bash
python3 ~/Downloads/ground_localization_check.py --stationary-only --window 30
```

Control publishers, an armed state, changed localization sources or loss of
fresh connected/disarmed state stop the diagnostic. The tool never commands
disarming. After manually handling the aircraft, prepare a fresh alignment and
venue session before flight.

## Reports and interpretation

Outputs are under `~/Downloads/ground_localization_checks/<timestamp>/`:

- `report.json`: each stationary window, manual expected displacement, measured
  forward/left/up displacement, return-to-start differences, covariance and
  message age/gap statistics.
- `samples.jsonl`: original stamps, callback times, poses, full covariance and
  observer QoS during both movement and stationary periods.
- `alignment_snapshot.py.txt`: the installed alignment used for this session,
  when available.
- `diagnostic.zip`: the report, samples and preparation evidence for analysis.

`CAPTURE_COMPLETE` means the observation sequence completed. It does **not**
mean the geometry, localization accuracy or aircraft is flight-ready. No new
geometric acceptance threshold is imposed. `INCOMPLETE_DATA`, `ERROR` and
`CANCELLED` retain the collected evidence.

All aligned, vision and FC displacements use the **same baseline FC heading**.
Separate position origins are removed, but differing source headings are not
rotated away. Raw LIO displacements remain explicitly labelled in raw coordinates.
Return differences include both localization drift and manual placement error.

Best Effort observers record every source. A second Reliable observer is created
only when a single publisher advertises Reliable, avoiding an incompatible
Reliable subscription to a Best Effort publisher. Matching original stamps are
compared over the overlapping observation interval. These are additional
observers: their delivery statistics do not prove delivery to an existing
controller, and receipt-time differences include executor scheduling.

Use the evidence to choose the next investigation:

| Observation | Follow-up |
|---|---|
| Manual displacement has incorrect direction or scale | Check mounting geometry and coordinate transforms before control tuning. |
| FC-centre position changes excessively during a fixed-centre rotation | Check the lever arm, mounting rotation, LIO response and actual pivot motion. |
| Raw LIO remains timely but later topics stall | Inspect the corresponding relay, subscriber scheduling and publisher discovery. |
| Both observers of raw LIO show gaps | Investigate the upstream source or shared scheduling; changing one subscription is not sufficient evidence of a fix. |
| FC feedback and vision disagree | Check fusion state, timing, frames and estimator resets. |
| Geometry agrees, but flight references lead measured motion and overshoot | Examine velocity/attitude response and reference feedforward with flight logs. |

A stationary covariance sample is not an accuracy measurement. On the inspected
PX4 revision, `EKF2_EV_NOISE_MD=0` uses the larger of the supplied position
variance and the squared `EKF2_EVP_NOISE` parameter. Changing the noise mode alone
does not change position noise when the message variance is already below that
floor.
