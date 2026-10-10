#!/usr/bin/env python3
"""Copy categorized sources to a flat aircraft Downloads directory.

Default: install missing files only. --overwrite backs up changed existing files.
Run from this checkout on a landed, disarmed aircraft, with affected nodes stopped.
This helper never imports ROS, runs a copied script, or changes PX4 parameters.
"""
import argparse
import json
import os
import shutil
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def sources(root=ROOT):
    layout = json.loads((root / 'scripts/layout.json').read_text())
    result = {}
    for name, relative in layout.items():
        if Path(name).name != name or not name.endswith('.py'):
            raise ValueError(f'Invalid deployment filename: {name}')
        path = (root / relative).resolve()
        if root.resolve() not in path.parents or not path.is_file():
            raise ValueError(f'Missing/outside-checkout source: {relative}')
        if name == 'build_first_hover_terminal.py':
            continue  # The generator needs the checkout and belongs on the developer machine.
        data = path.read_bytes()
        compile(data, str(path), 'exec')
        result[name] = data
    return result


def install(destination, overwrite=False, root=ROOT):
    files = sources(root)  # Validate ALL files before touching the destination.
    destination = destination.expanduser().resolve()
    if destination == root.resolve() or root.resolve() in destination.parents:
        raise ValueError('Deployment directory must be outside the checkout')
    destination.mkdir(parents=True, exist_ok=True)
    backup = None
    written, retained = [], []
    for name, data in files.items():
        target = destination / name
        if target.is_symlink() or (target.exists() and not target.is_file()):
            raise ValueError(f'Refusing to replace symlink/non-file: {target}')
        if target.exists():
            if target.read_bytes() == data or not overwrite:
                retained.append(name)
                continue
            if backup is None:
                backup = Path(tempfile.mkdtemp(prefix='super_scripts_backup_', dir=destination))
            shutil.copy2(target, backup / name)
        descriptor, staged = tempfile.mkstemp(prefix='.' + name + '.', dir=destination)
        try:
            with os.fdopen(descriptor, 'wb') as handle:
                handle.write(data)
            os.chmod(staged, 0o644)
            os.replace(staged, target)
        finally:
            Path(staged).unlink(missing_ok=True)
        written.append(name)
    print(f'Installed {len(written)} files in {destination}; retained {len(retained)} existing files.')
    if retained and not overwrite:
        print('Existing session/calibration files were retained. --overwrite is an explicit, backed-up update.')
    if backup:
        print('Backup:', backup)
    print('If alignment/localization sources were replaced, redo session pairing before restarting flight processes.')
    return written, retained, backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path, default=Path.home() / 'Downloads')
    parser.add_argument('--overwrite', action='store_true', help='back up and replace changed existing files')
    args = parser.parse_args()
    install(args.destination, args.overwrite)


if __name__ == '__main__':
    main()
