# Script organization and aircraft deployment

| Directory | Contents | Use |
| --- | --- | --- |
| [localization/](localization/) | Pose conversion, quaternion pairing, and vision relays | Terminals 4, 6, 7 and localization diagnostics |
| [flight/](flight/) | Hover state machine, reference, preview, parameter helper, ground setpoints | Terminal 11 implementation and flight preparation |
| [planning/](planning/) | SUPER command-coordinate preview and ground goals | Terminals 9/10 diagnostics |
| [analysis/](analysis/) | Bag-to-XYZ export, timing, and pose snapshots | Recorded-data assessment |
| [maintenance/](maintenance/) | FAST_LIO patches, historical converters, generator, deployment | Workspace maintenance |

The root retains three complete standalone download entry points: `first_hover_terminal.py`, `odom_age_check.py`, and `update_preview_alignment.py`. [layout.json](layout.json) maps aircraft filenames to categorized sources.

## Deploy into Downloads

Run from this checkout on the onboard computer after landing, disarming, and stopping affected processes:

```bash
python3 scripts/maintenance/install_scripts.py
```

The default installs missing files and preserves existing scripts and session alignment. Explicit updates back up changed files:

```bash
python3 scripts/maintenance/install_scripts.py --overwrite
```

Backups are stored under `~/Downloads/super_scripts_backup_*`. This helper copies files only; it does not start ROS nodes, arm, or write PX4 parameters. Redo session pairing after replacing localization/alignment sources.

Aircraft files remain flat under `~/Downloads`, preserving existing imports and startup paths. To update only terminal 11, use the standalone download in the [flight runbook](../docs/FIRST_HOVER_2026-10-09.md#download-and-install).

The standalone generator requires the checkout and is not deployed into Downloads:

```bash
python3 scripts/maintenance/build_first_hover_terminal.py --check
```

## Development imports

From the repository root:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:$PWD/scripts/localization:$PWD/scripts/flight:$PWD/scripts/planning:$PWD/scripts/analysis:$PWD/scripts/maintenance${PYTHONPATH:+:$PYTHONPATH}"
python3 tests/test_repository_layout.py
python3 tests/test_hover_reference.py
python3 tests/test_bag_to_xyz.py
```

Use an isolated environment/domain for ROS graph tests. See [development validation](../docs/FIRST_HOVER_2026-10-09.md#development-validation). Offline bag export creates no ROS publishers.
