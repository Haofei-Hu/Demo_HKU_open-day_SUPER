#!/usr/bin/env python3
"""Preserve measured SUPER rest-start position; back up and patch pinned source.

No ROS calls, configuration changes, build or process start. Occupied/invalid
start points are refused rather than relocating a trajectory away from the robot.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


REVISION = '2ad3419c127a617c6d7df6925e81a14175a9c096'
RELATIVE = 'super_planner/src/super_core/super_planner.cpp'
OLD = b'''        /// 1) First, shift the start_point to free space.
        Vec3f local_star_pt;
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0)) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        latest_replan.setLocalStartP(local_star_pt);
'''
NEW = b'''        // Exact rest start: retain odometry; never replace it with a voxel centre.
        Vec3f local_star_pt = robot_state_.p;
        if (!local_star_pt.allFinite()) {
            ros_ptr_->error(" -- [SUPER] [PlanFromRest] Exact start rejected: nonfinite odometry.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        const auto start_grid_type = map_ptr_->getGridType(local_star_pt);
        if (start_grid_type != GridType::KNOWN_FREE && start_grid_type != GridType::UNKNOWN) {
            ros_ptr_->error(" -- [SUPER] [PlanFromRest] Exact start rejected: grid type {}; no relocation.",
                            static_cast<int>(start_grid_type));
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        ros_ptr_->info(" -- [SUPER] [PlanFromRest] Exact odometry start retained; grid type {}.",
                       static_cast<int>(start_grid_type));
        latest_replan.setLocalStartP(local_star_pt);
'''


def patch(root, backup_root):
    root = Path(root).expanduser().resolve()
    source = root/RELATIVE
    revision = subprocess.run(['git','-C',str(root),'rev-parse','HEAD'],check=True,
                              text=True,stdout=subprocess.PIPE).stdout.strip()
    if revision != REVISION:
        raise ValueError('Unsupported SUPER revision; no files written: '+revision)
    before = source.read_bytes()
    if before.count(NEW) == 1 and OLD not in before:
        print('Exact rest-start patch already present; no files written.')
        return None
    if before.count(OLD) != 1 or NEW in before:
        raise ValueError('Expected rest-start source block not found once; no files written.')
    after = before.replace(OLD,NEW,1)
    backup_root = Path(backup_root).expanduser()
    backup_root.mkdir(parents=True,exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix='super_exact_start_backup_',dir=backup_root))
    shutil.copy2(source,backup/source.name)
    manifest = dict(source=str(source),revision=revision,
                    before_sha256=hashlib.sha256(before).hexdigest(),
                    after_sha256=hashlib.sha256(after).hexdigest())
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    if source.read_bytes() != before or (backup/source.name).read_bytes() != before:
        raise ValueError('Source changed during backup; no patch written.')
    source.write_bytes(after)
    print('Patched:',source)
    print('Backup:',backup)
    print('Rebuild super_planner before restarting only the ground planning windows.')
    return backup


def restore(backup):
    backup = Path(backup).expanduser()
    manifest = json.loads((backup/'manifest.json').read_text())
    source = Path(manifest['source'])
    old = (backup/source.name).read_bytes()
    current = source.read_bytes()
    if hashlib.sha256(old).hexdigest() != manifest['before_sha256']:
        raise ValueError('Backup checksum mismatch; no files written.')
    if current == old:
        print('Original source already restored; no files written.')
        return
    if hashlib.sha256(current).hexdigest() != manifest['after_sha256']:
        raise ValueError('Source changed after patch; refusing to overwrite later edits.')
    source.write_bytes(old)
    print('Restored:',source)
    print('Rebuild super_planner before restarting ground planning.')


def restore_applied(root, backup_root):
    """Restore only a matching backed-up patch; leave an original source alone."""
    source = Path(root).expanduser().resolve()/RELATIVE
    current = source.read_bytes()
    if current.count(OLD) == 1 and NEW not in current:
        print('Original rest-start source present; no files written.')
        print('Rebuild only if a patched executable was previously built.')
        return False
    if current.count(NEW) != 1 or OLD in current:
        raise ValueError('Unrecognized rest-start source; no files written.')
    current_hash = hashlib.sha256(current).hexdigest()
    for backup in sorted(Path(backup_root).expanduser().glob('super_exact_start_backup_*')):
        try:
            manifest = json.loads((backup/'manifest.json').read_text())
            matches = (Path(manifest['source']).resolve() == source
                       and manifest['revision'] == REVISION
                       and manifest['after_sha256'] == current_hash)
        except (OSError, ValueError, KeyError, TypeError):
            continue
        if matches:
            restore(backup)  # Also verifies the original file checksum.
            return True
    raise ValueError('No matching backup for the current source; no files written. '
                     'Inspect the original backup and any later source edits.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path.home()/'super_ws/src/SUPER')
    parser.add_argument('--backup-root',type=Path,default=Path.home()/'Downloads')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--restore',type=Path,help='restore only if both stored hashes match')
    mode.add_argument('--restore-applied',action='store_true',
                      help='find a matching backup and restore; leave unpatched source unchanged')
    args = parser.parse_args()
    if args.restore_applied:
        restore_applied(args.root,args.backup_root)
    elif args.restore is not None:
        restore(args.restore)
    else:
        patch(args.root,args.backup_root)


if __name__=='__main__':
    main()
