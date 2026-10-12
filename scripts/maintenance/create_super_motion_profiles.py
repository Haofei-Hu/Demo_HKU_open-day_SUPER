#!/usr/bin/env python3
"""Create separate SUPER motion candidates; never start nodes or flight.

Only translational velocity/acceleration limits change. Planner-frame height,
geometry, and map settings are preserved and still need flight integration.
Existing different candidates are refused rather than overwritten.
"""
import argparse
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import shutil
import tempfile

import yaml

PROFILES = {
    'mid360_low_speed_preview_ros2.yaml': (0.3, 0.5),
    'mid360_demo_candidate_ros2.yaml': (2.0, 1.0),
}


def candidate(text, velocity, acceleration):
    data = yaml.safe_load(text)
    for name in ('max_vel', 'max_acc'):
        value = data['traj_opt']['boundary'][name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid source motion limit: '+name)
    tree = yaml.compose(text)
    edits = []
    for name, value in (('max_vel', velocity), ('max_acc', acceleration)):
        node = tree
        for key in ('traj_opt', 'boundary', name):
            if not isinstance(node, yaml.MappingNode):
                raise ValueError('Unexpected configuration structure')
            matches = [v for k, v in node.value if k.value == key]
            if len(matches) != 1:
                raise ValueError('Missing/duplicate YAML key: '+key)
            node = matches[0]
        if not isinstance(node, yaml.ScalarNode):
            raise ValueError('Motion limit must be a scalar')
        edits.append((node.start_mark.index, node.end_mark.index, str(value)))
    updated = text
    for start, end, value in sorted(edits, reverse=True):
        updated = updated[:start] + value + updated[end:]
    expected = deepcopy(data)
    expected['traj_opt']['boundary'].update(max_vel=velocity, max_acc=acceleration)
    if yaml.safe_load(updated) != expected:
        raise ValueError('Unexpected configuration differences')
    return updated


def create(source):
    source = Path(source)
    text = source.read_text()
    provenance = '# Motion candidate from '+source.name+'; source SHA256: '+hashlib.sha256(source.read_bytes()).hexdigest()+'\n'
    staged = {source.with_name(name): provenance+candidate(text, *limits)
              for name, limits in PROFILES.items()}
    for path, content in staged.items():
        if path.is_symlink() or (path.exists() and (not path.is_file() or path.read_text() != content)):
            raise ValueError('Existing candidate differs; no files written: '+str(path))
    missing = [path for path in staged if not path.exists()]
    if missing:
        backup = Path(tempfile.mkdtemp(prefix='motion_profile_source_', dir=source.parent))
        shutil.copy2(source, backup/source.name)
        print('Source backup:', backup/source.name)
        for path in missing:
            with path.open('x') as handle:
                handle.write(staged[path])
    for path in staged:
        velocity, acceleration = PROFILES[path.name]
        print(f'Candidate: {path}; max_vel={velocity:g} m/s, max_acc={acceleration:g} m/s^2')
    print('Only motion limits changed; height, origin, bounds and flight bridge remain pending. No nodes started.')
    return list(staged)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path.home()/'super_ws/src/SUPER/super_planner/config/mid360_ground_diagnostic_ros2.yaml')
    args = parser.parse_args()
    create(args.source)
