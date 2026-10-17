#!/usr/bin/env python3
"""Keep 2/1 optimization targets and add separate 4/2 acceptance bounds."""
import argparse
from copy import deepcopy
import hashlib
from pathlib import Path

import yaml

from apply_super_motion_acceptance import CONFIG,BACKUP,EXPLORATION,VERSION,patched

TARGETS=(2.,1.)
ACCEPTANCE=(4.,2.)


def require_support(source_root):
    root=Path(source_root).expanduser().resolve()
    for relative in (CONFIG,BACKUP,EXPLORATION):
        text=(root/relative).read_text()
        if VERSION not in text or patched(relative,text)!=text:
            raise ValueError('Apply apply_super_motion_acceptance.py and rebuild super_planner first')
    workspace=root.parent.parent
    candidates=[workspace/'install/super_planner/lib/super_planner/fsm_node',
                workspace/'install/lib/super_planner/fsm_node']
    for path in candidates:
        if path.is_file():
            with path.open('rb') as handle:
                if handle.read(4)!=b'\x7fELF':continue
                handle.seek(0)
                previous=b''
                while chunk:=handle.read(1024*1024):
                    if VERSION.encode() in previous+chunk:return path
                    previous=chunk[-len(VERSION):]
    raise ValueError('Installed fsm_node has no motion-acceptance support; rebuild super_planner before startup')


def create(source,source_root=None):
    source=Path(source).expanduser().resolve()
    if source_root is not None:require_support(source_root)
    original=source.read_bytes();data=yaml.safe_load(original)
    boundary=data['traj_opt']['boundary']
    if (boundary['max_vel'],boundary['max_acc'])!=TARGETS:
        raise ValueError('Need unchanged 2 m/s and 1 m/s^2 optimization targets')
    expected=deepcopy(data)
    expected['traj_opt']['boundary'].update(acceptance_max_vel=ACCEPTANCE[0],acceptance_max_acc=ACCEPTANCE[1])
    token=hashlib.sha256(original).hexdigest()[:12]
    target=source.with_name('mid360_demo_acceptance_'+token+'.yaml')
    text=('# Optimization targets 2/1; explicit acceptance bounds 4/2. Source YAML unchanged.\n'+
          yaml.safe_dump(expected,sort_keys=False))
    if target.is_symlink() or target.exists() and target.read_text()!=text:
        raise ValueError('Existing acceptance profile differs; no file replaced: '+str(target))
    if not target.exists():
        with target.open('x') as handle:handle.write(text)
    print('DEMO MOTION: target=2 m/s, 1 m/s^2; acceptance=4 m/s, 2 m/s^2')
    print('Acceptance profile:',target)
    return target


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--source-root',type=Path,default=Path.home()/'super_ws/src/SUPER')
    args=p.parse_args();create(args.source,args.source_root)


if __name__=='__main__':main()
