#!/usr/bin/env python3
"""Install one pinned automatic-demo-test release; never start ROS or flight."""
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


def install(revision,destination,fetch=None):
    if not re.fullmatch('[0-9a-f]{40}',revision):raise ValueError('Use a full pinned commit SHA')
    destination=Path(destination).expanduser();destination.mkdir(parents=True,exist_ok=True)
    base='https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/'+revision+'/'
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def download(path):
        with opener.open(base+path,timeout=30) as response:
            data=response.read(4*1024*1024+1)
        if len(data)>4*1024*1024:raise ValueError('Release file exceeds 4 MiB: '+path)
        return data
    fetch=fetch or download
    manifest=json.loads(fetch('configs/demonstration/demo_test_release.json'))
    if manifest.get('schema_version')!=1:raise ValueError('Invalid release manifest')
    staged=Path(tempfile.mkdtemp(prefix='demo_test_update_',dir=destination))
    entries=manifest['files']
    for name,entry in entries.items():
        if Path(name).name!=name or not name.endswith(('.py','.json')):raise ValueError('Invalid release filename')
        path=entry['path']
        if '..' in Path(path).parts or Path(path).is_absolute():raise ValueError('Invalid release path')
        target=destination/name
        if target.is_symlink() or target.exists() and not target.is_file():raise ValueError('Refusing unsafe target: '+str(target))
        data=fetch(path)
        if hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('Checksum mismatch: '+name)
        if name.endswith('.py'):ast.parse(data,filename=name)
        else:json.loads(data)
        (staged/name).write_bytes(data)
    # Nothing is replaced until the entire release validates.
    changed=[];retained=[]
    for name,entry in entries.items():
        target=destination/name;data=(staged/name).read_bytes()
        if target.exists() and (target.read_bytes()==data or entry.get('preserve_existing')):
            retained.append(name);continue
        if target.exists():shutil.copy2(target,staged/('previous_'+name))
        tmp=destination/('.demo_update_'+name);tmp.write_bytes(data);os.replace(tmp,target);changed.append(name)
    (staged/'release.json').write_text(json.dumps(dict(revision=revision,updated=changed,retained=retained),indent=2)+'\n')
    print('Demo test scripts updated:',len(changed),'; retained:',len(retained))
    print('Backups:',staged)
    print('Measured alignment and existing mission measurements were preserved. No ROS or flight was started.')
    return staged


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--revision',required=True)
    p.add_argument('--destination',type=Path,default=Path.home()/'Downloads')
    args=p.parse_args();install(args.revision,args.destination)


if __name__=='__main__':main()
