#!/usr/bin/env python3
"""Create a separate low-speed 3D-goal profile; no ROS calls or node startup."""
import argparse
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import shutil
import tempfile

import yaml


def create(source):
    source = Path(source)
    text = source.read_text()
    data = yaml.safe_load(text)
    limits = data['traj_opt']['boundary']
    if limits['max_vel'] != .25 or limits['max_acc'] != .45:
        raise ValueError('Expected reserved 0.25 m/s, 0.45 m/s^2 source; no files written.')
    height = data['fsm']['click_height']
    if isinstance(height, bool) or not isinstance(height, (int,float)) or not math.isfinite(height):
        raise ValueError('Invalid click_height; no files written.')
    node = yaml.compose(text)
    for key in ('fsm','click_height'):
        if not isinstance(node,yaml.MappingNode):
            raise ValueError('Unexpected goal configuration; no files written.')
        matches = [v for k,v in node.value if k.value == key]
        if len(matches) != 1:
            raise ValueError('Missing/duplicate goal key; no files written.')
        node = matches[0]
    if not isinstance(node,yaml.ScalarNode):
        raise ValueError('Expected scalar click_height; no files written.')
    # Pinned SUPER overrides message Z only when click_height > -5.
    updated = text[:node.start_mark.index]+'-6.0'+text[node.end_mark.index:]
    expected = deepcopy(data)
    expected['fsm']['click_height'] = -6.0
    if yaml.safe_load(updated) != expected:
        raise ValueError('Unexpected configuration differences; no files written.')
    content = '# 3D goal profile; source SHA256: '+hashlib.sha256(source.read_bytes()).hexdigest()+'\n'+updated
    target = source.with_name('mid360_low_speed_3d_goal_ros2.yaml')
    if target.is_symlink() or (target.exists() and (not target.is_file() or target.read_text() != content)):
        raise ValueError('Existing 3D goal profile differs; no files written: '+str(target))
    if not target.exists():
        backup = Path(tempfile.mkdtemp(prefix='three_d_goal_source_',dir=source.parent))
        shutil.copy2(source,backup/source.name)
        with target.open('x') as handle:
            handle.write(content)
        print('Source backup:',backup/source.name)
    print('3D goal profile:',target)
    print('Only fixed-Z override disabled; message Z retained before map relocation. No nodes started.')
    return target


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=Path.home()/'super_ws/src/SUPER/super_planner/config/mid360_low_speed_reserved_ros2.yaml')
    args = parser.parse_args()
    create(args.source)
