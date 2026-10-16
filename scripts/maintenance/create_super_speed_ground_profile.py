#!/usr/bin/env python3
"""Create separate 2 m/s planning profiles; no ROS or flight operations."""
import argparse
import hashlib
from pathlib import Path

import yaml
from create_super_motion_profiles import candidate


def create(source,mission=False):
    source=Path(source)
    text=source.read_text()
    data=yaml.safe_load(text)
    if (data['fsm']['click_height']!=-6.
            or data['traj_opt']['boundary']['max_vel']!=.25
            or data['traj_opt']['boundary']['max_acc']!=.45):
        raise ValueError('Need the existing reserved low-speed-3d source profile')
    content=(('# Demonstration motion profile, 2 m/s and 1 m/s^2; source SHA256: ' if mission else
              '# Ground audit only, 2 m/s and 1 m/s^2; source SHA256: ')
             +hashlib.sha256(source.read_bytes()).hexdigest()+'\n'+candidate(text,2.,1.))
    target=source.with_name('mid360_demo_mission_ros2.yaml' if mission else 'mid360_forward_5m_ground_ros2.yaml')
    if target.is_symlink() or (target.exists() and target.read_text()!=content):
        raise ValueError('Existing speed-ground profile differs; refusing overwrite: '+str(target))
    if not target.exists():
        with target.open('x') as handle:handle.write(content)
    print('Demonstration motion profile:' if mission else 'Ground-only 5 m / 2 m/s planner profile:',target)
    print('Only planner velocity/acceleration limits changed; source and PX4 parameters unchanged.')
    return target


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--mission',action='store_true',help='separate full-demonstration motion configuration')
    args=parser.parse_args();create(args.source,args.mission)
