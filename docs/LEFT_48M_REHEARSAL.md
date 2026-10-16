# Independent left-facing 4.8 m rehearsal

This optional virtual mission turns the captured vehicle's initial heading 90 degrees left, rises to 0.5 m, performs five 4.8 m out-and-back rounds, holds six seconds after the fifth return and lands. The physical aircraft remains untouched. Heading stays fixed at the new orientation throughout; returns travel backward unless a curved SUPER path changes the velocity direction.

The original forward demonstration, installed scripts, measured mission and reports remain separate. The original quick-start release pin is unchanged. The left installer writes a checksum-verified immutable bundle beneath `~/Downloads/super_left_48_rehearsal/<revision>/`; it does not start, stop or update any running aircraft session. Reports go to `~/Downloads/super_left_48_tests/<timestamp>/`.

## Install once

```bash
(
set -e
left_revision='184b36312888c89a0d97326fe069e8ab9664a29c'
left_download_dir=$(mktemp -d)
curl -q --noproxy '*' -fL --connect-timeout 10 --max-time 30 \
  "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/$left_revision/scripts/maintenance/install_left_rehearsal.py" \
  -o "$left_download_dir/install_left_rehearsal.py"
python3 "$left_download_dir/install_left_rehearsal.py" --revision "$left_revision"
)
```

The installer prints the exact launch command. After the current original rehearsal finishes, use:

```bash
python3 ~/Downloads/super_left_48_rehearsal/184b36312888c89a0d97326fe069e8ab9664a29c/super_left_test.py
```

No `--restart` or new alignment is needed. The launcher automatically selects the newest **successful real-SUPER** report with a saved scene beneath `~/Downloads/super_demo_tests`. It ignores active, failed and synthetic-only runs. To select a particular original capture, append `--scene-file /absolute/path/to/original/run/scene.json`.

The saved geometry hashes are checked, and the occupied cloud is copied unchanged. A separate scene copy rotates only the virtual body attitude about aligned up; the mounting lever arm is compensated so the FC centre stays at the same point. Neither the obstacle map nor the physical aircraft rotates. The virtual 5.8 m long venue axis follows this new heading, with a 2.8 m width and height. This is labelled fixture geometry, not a surveyed physical clearance claim.

Real SUPER plans through the original captured obstacles, in isolated localhost-only ROS domain **198**. The original rehearsal uses domain 197; the actual aircraft stack uses its existing domain. The new runner never calls the actual session's start/stop, publishes aircraft flight commands, rewrites measured alignment or repeats real localization preparation. The scene is a reused frozen snapshot, not a fresh scan of a changed room. Run the two graphical rehearsals sequentially to avoid competing for onboard CPU.

QGC opens or reuses its existing window before the virtual mission. RViz opens automatically in domain 198 and displays the newly turned virtual heading, original obstacle cloud and live SUPER paths. Append `--no-rviz` or `--no-qgc` to skip the relevant application.

The left-only planner limits are 2 m/s and 1.5 m/s². Each run writes a new derived YAML with this acceleration limit; the saved source planner YAML and the original forward mission retain their existing values. Mission validation, planner configuration and the left reference monitor all use the same limits. Observe mode retains the existing motion advisories; curved paths are allowed inside the vehicle-inclusive virtual venue. No first-plan or total-mission deadline is added. Source validity, freshness and polynomial continuity checks remain active. Reports print `RESULT: SUCCESS/FAIL`, preserve all ten goal/arrival records and require all five returns plus landing for success. No complete onboard left-facing run is claimed before its report is obtained.
