#!/usr/bin/env python3
"""Install an immutable, separate left-facing rehearsal bundle; never start ROS."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import urllib.request


def install(revision,root=None,fetch=None):
    if not re.fullmatch('[0-9a-f]{40}',revision):raise ValueError('Use a full pinned commit SHA')
    root=Path(root or Path.home()/'Downloads/super_left_48_rehearsal').expanduser()
    if root.is_symlink():raise ValueError('Refusing a symlinked installation root')
    destination=root/revision
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    base='https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/'+revision+'/'
    def download(path):
        with opener.open(base+path,timeout=30) as response:data=response.read(4*1024*1024+1)
        if len(data)>4*1024*1024:raise ValueError('Release file exceeds 4 MiB')
        return data
    fetch=fetch or download
    manifest=json.loads(fetch('configs/demonstration/left_rehearsal_release.json'))
    if manifest.get('schema_version')!=1:raise ValueError('Invalid left rehearsal manifest')
    root.mkdir(parents=True,exist_ok=True)
    staged=Path(tempfile.mkdtemp(prefix='.left_install_',dir=root))
    try:
        for name,entry in manifest['files'].items():
            path=entry['path']
            if Path(name).name!=name or not name.endswith(('.py','.json')):
                raise ValueError('Invalid release filename')
            if Path(path).is_absolute() or '..' in Path(path).parts:raise ValueError('Invalid release path')
            data=fetch(path)
            if hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('Checksum mismatch: '+name)
            if name.endswith('.py'):ast.parse(data,filename=name)
            else:json.loads(data)
            (staged/name).write_bytes(data)
        if destination.is_symlink():raise ValueError('Refusing a symlinked release')
        if destination.exists():
            for name in manifest['files']:
                target=destination/name
                if target.is_symlink() or not target.is_file() or target.read_bytes()!=(staged/name).read_bytes():
                    raise ValueError('Existing immutable release differs: '+str(target))
            print('Separate release already verified:',destination)
        else:os.rename(staged,destination)
    finally:
        if staged.exists():shutil.rmtree(staged)
    print('Original scripts, mission files, reports and running sessions were not modified.')
    print('RUN AFTER THE CURRENT REHEARSAL FINISHES:')
    print('python3 '+str(destination/'super_left_test.py'))
    return destination


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--revision',required=True)
    args=p.parse_args();install(args.revision)


if __name__=='__main__':main()
