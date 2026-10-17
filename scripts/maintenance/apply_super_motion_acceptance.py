#!/usr/bin/env python3
"""Separate SUPER optimization targets from explicit trajectory acceptance bounds.

Supports the recorded SUPER 2ad3419 source layout. All files are validated before
writing, original files are backed up, and absent YAML bounds preserve upstream
checks. This edits source only; rebuild super_planner before using new bounds.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

VERSION='MOTION_ACCEPTANCE_V1'
CONFIG='super_planner/include/traj_opt/config.hpp'
BACKUP='super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp'
EXPLORATION='super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp'

MEMBERS='''        // MOTION_ACCEPTANCE_V1: optimization bounds above remain unchanged.
        double acceptance_max_vel{-1.0}, acceptance_max_acc{-1.0};

        double velocityViolationLimit() const {
            return acceptance_max_vel > 0 ?
                std::max(0.0, acceptance_max_vel * acceptance_max_vel - max_vel * max_vel) :
                max_vel * penna_margin;
        }
        double accelerationViolationLimit() const {
            return acceptance_max_acc > 0 ?
                std::max(0.0, acceptance_max_acc * acceptance_max_acc - max_acc * max_acc) :
                max_acc * penna_margin;
        }
        double velocityAcceptanceBound() const {
            return acceptance_max_vel > 0 ? acceptance_max_vel : 1.2 * max_vel;
        }
        double accelerationAcceptanceBound() const {
            return acceptance_max_acc > 0 ? acceptance_max_acc : 1.2 * max_acc;
        }
'''

LOAD='''            loader.LoadParam("traj_opt/boundary/acceptance_max_vel", acceptance_max_vel, -1.0);
            loader.LoadParam("traj_opt/boundary/acceptance_max_acc", acceptance_max_acc, -1.0);
            if ((acceptance_max_vel != -1.0 &&
                 (!std::isfinite(acceptance_max_vel) || acceptance_max_vel < max_vel || acceptance_max_vel <= 0)) ||
                (acceptance_max_acc != -1.0 &&
                 (!std::isfinite(acceptance_max_acc) || acceptance_max_acc < max_acc || acceptance_max_acc <= 0))) {
                throw std::invalid_argument("Motion acceptance bounds must be finite and at least optimization bounds");
            }
            if (acceptance_max_vel > 0 || acceptance_max_acc > 0) {
                std::cout << "MOTION_ACCEPTANCE_V1 " << ns
                          << " target=" << max_vel << "/" << max_acc
                          << " accept=" << velocityAcceptanceBound() << "/" << accelerationAcceptanceBound()
                          << std::endl;
            }
'''

EXP_FINAL='''        // MOTION_ACCEPTANCE_V1: check full polynomial extrema, not only samples.
        if ((cfg_.acceptance_max_vel > 0 &&
             (!std::isfinite(traj.getMaxVelRate()) || traj.getMaxVelRate() > cfg_.velocityAcceptanceBound() + 1e-9)) ||
            (cfg_.acceptance_max_acc > 0 &&
             (!std::isfinite(traj.getMaxAccRate()) || traj.getMaxAccRate() > cfg_.accelerationAcceptanceBound() + 1e-9))) {
            ros_ptr_->warn(" -- [ExpOpt] Trajectory exceeds explicit motion acceptance bounds.");
            traj.clear();
            minCostFunctional = INFINITY;
            ret = -1;
        }
'''

EDITS={
    CONFIG:[
        ('#include <string>\n','#include <string>\n#include <algorithm>\n#include <cmath>\n#include <iostream>\n#include <stdexcept>\n'),
        ('        double penna_margin{0.05};\n','        double penna_margin{0.05};\n\n'+MEMBERS),
        ('            loader.LoadParam("traj_opt/boundary/penna_margin", penna_margin, 0.05);\n',
         '            loader.LoadParam("traj_opt/boundary/penna_margin", penna_margin, 0.05);\n'+LOAD),
    ],
    BACKUP:[
        ('opt_vars.penalty_log(2) > cfg_.max_vel * cfg_.penna_margin','opt_vars.penalty_log(2) > cfg_.velocityViolationLimit()'),
        ('opt_vars.penalty_log(3) > cfg_.max_acc * cfg_.penna_margin','opt_vars.penalty_log(3) > cfg_.accelerationViolationLimit()'),
        ('out_traj.getMaxVelRate() > 1.2 * cfg_.max_vel','out_traj.getMaxVelRate() > cfg_.velocityAcceptanceBound()'),
        ('out_traj.getMaxAccRate() > 1.2 * cfg_.max_acc','out_traj.getMaxAccRate() > cfg_.accelerationAcceptanceBound()'),
        (' -- [BackOpt] Opt failed, Omg or thr or Pos violation.',
         ' -- [BackOpt] MOTION_ACCEPTANCE_V1: position/velocity/acceleration/body-rate/thrust violation.'),
    ],
    EXPLORATION:[
        ('// (cfg_.penna_vel > 0 && opt_vars.penalty_log(2) > cfg_.max_vel * cfg_.penna_margin) ||',
         '(cfg_.acceptance_max_vel > 0 && cfg_.penna_vel > 0 && opt_vars.penalty_log(2) > cfg_.velocityViolationLimit()) ||'),
        ('opt_vars.penalty_log(3) > cfg_.max_acc * cfg_.penna_margin','opt_vars.penalty_log(3) > cfg_.accelerationViolationLimit()'),
        (' -- [ExpOpt] Opt failed, Omg or thr or Pos violation.',
         ' -- [ExpOpt] Opt failed: position/velocity/acceleration/body-rate/thrust violation.'),
        ('        opt_vars.minco.getTrajectory(traj);\n    } else {',
         '        opt_vars.minco.getTrajectory(traj);\n'+EXP_FINAL+'    } else {'),
    ],
}


def patched(relative,text):
    edits=EDITS[relative]
    if VERSION in text:
        if not all(new in text for _,new in edits):
            raise ValueError('Incomplete or modified motion acceptance patch: '+relative)
        return text
    for old,new in edits:
        if text.count(old)!=1:raise ValueError('Unsupported source context; no files written: '+relative)
        text=text.replace(old,new,1)
    return text


def apply(root,dry_run=False):
    root=Path(root).expanduser().resolve();staged={}
    for relative in EDITS:
        path=root/relative
        if path.is_symlink():raise ValueError('Refusing symlinked source: '+str(path))
        original=path.read_text();replacement=patched(relative,original)
        if replacement!=original:staged[relative]=(original,replacement)
    if not staged:
        print('Motion acceptance source patch already applied; rebuild if not yet done.')
        return None
    if dry_run:
        for relative,(before,after) in staged.items():
            print(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),relative,relative)),end='')
        return None
    backup=Path(tempfile.mkdtemp(prefix='motion_acceptance_backup_',dir=root))
    record=dict(version=VERSION,root=str(root),files={})
    for relative,(_,after) in staged.items():
        saved=backup/relative;saved.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(root/relative,saved)
        record['files'][relative]=dict(before=hashlib.sha256(saved.read_bytes()).hexdigest(),
                                      after=hashlib.sha256(after.encode()).hexdigest())
    (backup/'record.json').write_text(json.dumps(record,indent=2)+'\n')
    for relative,(_,after) in staged.items():
        (root/relative).write_text(after)
    print('Motion acceptance source patch applied. Backup:',backup)
    print('Rebuild super_planner before using acceptance_max_vel/acceptance_max_acc.')
    return backup


def restore(backup):
    backup=Path(backup).expanduser().resolve();record=json.loads((backup/'record.json').read_text())
    if record.get('version')!=VERSION:raise ValueError('Unsupported backup')
    root=Path(record['root'])
    for relative,hashes in record['files'].items():
        if relative not in EDITS:raise ValueError('Unexpected backup source')
        if hashlib.sha256((root/relative).read_bytes()).hexdigest()!=hashes['after']:
            raise ValueError('Source changed after patch; refusing restore: '+relative)
        if hashlib.sha256((backup/relative).read_bytes()).hexdigest()!=hashes['before']:
            raise ValueError('Backup changed: '+relative)
    for relative in record['files']:shutil.copy2(backup/relative,root/relative)
    print('Original source restored; rebuild super_planner before restarting.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root',type=Path,default=Path.home()/'super_ws/src/SUPER')
    p.add_argument('--dry-run',action='store_true')
    p.add_argument('--restore',type=Path)
    args=p.parse_args()
    restore(args.restore) if args.restore else apply(args.source_root,args.dry_run)


if __name__=='__main__':main()
