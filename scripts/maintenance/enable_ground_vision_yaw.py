#!/usr/bin/env python3
"""Allow XY plus vision yaw in the disarmed ground relay, with a source backup."""
from pathlib import Path
import shutil
import sys
import tempfile


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        Path.home() / 'Downloads/ground_xy_fusion_transport.py')
    text = path.read_text()
    old = 'response.values[0].integer_value in (0, 1)'
    new = 'response.values[0].integer_value in (0, 1, 9)'
    if new in text:
        print('Ground relay already allows EV_CTRL 0/1/9; no changes made.')
        return
    if text.count(old) != 1:
        raise SystemExit('Parameter guard source mismatch; no file written.')
    updated = text.replace(old, new, 1)
    updated = updated.replace('EV_CTRL 0 or 1', 'EV_CTRL 0/1/9')
    updated = updated.replace('DISARMED GROUND XY TEST', 'DISARMED GROUND XY/YAW TEST')
    compile(updated, str(path), 'exec')
    backup = Path(tempfile.mkdtemp(
        prefix='ground_xy_backup_', dir=str(path.parent))) / path.name
    shutil.copy2(path, backup)
    path.write_text(updated)
    print('Ground relay now allows EV_CTRL 0/1/9.')
    print('Backup:', backup)


if __name__ == '__main__':
    main()
