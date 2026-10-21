#!/usr/bin/env python3
"""Guard the reviewed FAST-LIO IMU startup state; default is a read-only check.

Only the exact uploaded a4743 IMU header or its exact patched form is accepted.
This does not change parameters, flush queues, start nodes, or build the binary.
"""
import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile

BEFORE = '2b786c50c255a118fe95c9b017ef13af9e27a79b0624a718d4e192c3e5ca138d'
AFTER = 'f2a4da66cab6d537d3b584dd5989419ec4cd7f30b72be817a184d9f292565849'
CHANGES = (
    ('  V3D acc_s_last;', '  V3D acc_s_last = Zero3d; // FASTLIO_IMU_INIT_GUARD_V1'),
    ('  double last_lidar_end_time_;', '  double last_lidar_end_time_ = -1.0;'),
    ('  start_timestamp_  = -1;\n  init_iter_num',
     '  start_timestamp_  = -1;\n  last_lidar_end_time_ = -1.0;\n  acc_s_last = Zero3d;\n  init_iter_num'),
    ('  if(meas.imu.empty()) {return;};',
     '  // Never expose the previous scan when initialization/input is incomplete.\n'
     '  cur_pcl_un_->clear();\n  if(meas.imu.empty()) {return;};'),
    ('      imu_need_init_ = false;',
     '      imu_need_init_ = false;\n'
     '      // The next propagation starts at this initialized scan boundary.\n'
     '      last_lidar_end_time_ = meas.lidar_end_time;\n      acc_s_last = Zero3d;'),
)

def sha(data):
    return hashlib.sha256(data).hexdigest()

def render(data):
    digest = sha(data)
    if digest == AFTER:
        return data
    if digest != BEFORE:
        raise ValueError('Source differs from reviewed header; no changes made. SHA256='+digest)
    text = data.decode('utf-8')
    for old, new in CHANGES:
        if text.count(old) != 1:
            raise ValueError('Source anchor mismatch; no changes made')
        text = text.replace(old, new, 1)
    result = text.encode('utf-8')
    if sha(result) != AFTER:
        raise ValueError('Patch output checksum mismatch; no changes made')
    return result

def mapping_running():
    for entry in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            words = entry.read_bytes().split(b'\0')
            if any(Path(os.fsdecode(word)).name == 'fastlio_mapping' for word in words if word):
                return True
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as exc:
            raise RuntimeError('Cannot verify process state; stop and inspect processes first') from exc
    return False

def apply(source, backup_root):
    source = source.expanduser().resolve(strict=True)
    original = source.read_bytes()
    updated = render(original)
    if updated == original:
        print('Exact reviewed patch already present. No source changes.')
        return None
    if mapping_running():
        raise RuntimeError('fastlio_mapping is running; use grounded managed stop first')
    backup_root = backup_root.expanduser()
    backup_root.mkdir(parents=True, exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix='imu_init_', dir=backup_root))
    shutil.copy2(source, backup/source.name)
    (backup/'manifest.json').write_text(json.dumps(dict(source=str(source), before=BEFORE, after=AFTER), indent=2)+'\n')
    (backup/'change.patch').write_text(''.join(difflib.unified_diff(
        original.decode().splitlines(True), updated.decode().splitlines(True),
        fromfile='a/src/IMU_Processing.hpp', tofile='b/src/IMU_Processing.hpp')))
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=source.parent, prefix='.imu_guard_', delete=False) as f:
            temporary = Path(f.name)
            f.write(updated)
            f.flush()
            os.fsync(f.fileno())
        temporary.chmod(source.stat().st_mode & 0o777)
        if source.read_bytes() != original:
            raise RuntimeError('Source changed during patch preparation; not overwritten')
        if mapping_running():
            raise RuntimeError('fastlio_mapping started during patch preparation; not overwritten')
        temporary.replace(source)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    print('PATCH_APPLIED:', source)
    print('BACKUP:', backup/source.name)
    print('Rebuild fast_lio while stopped. Source editing alone does not update the running binary.')
    return backup

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path.home()/'fastlio_ws/src/FAST_LIO/src/IMU_Processing.hpp')
    parser.add_argument('--backup-root', type=Path, default=Path.home()/'Downloads/fastlio_init_backups')
    parser.add_argument('--apply', action='store_true', help='apply after grounded shutdown; otherwise check only')
    args = parser.parse_args()
    if args.apply:
        apply(args.source, args.backup_root)
    else:
        before = args.source.expanduser().read_bytes()
        after = render(before)
        print('ALREADY_PATCHED' if before == after else 'REVIEWED_SOURCE_MATCH; --apply required to change it')

if __name__ == '__main__':
    main()
