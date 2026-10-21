#!/usr/bin/env python3
"""Reversible, grounded-start IMU selection; no ROS publication or FC writes.

PX4 mode uses the repository's NOMINAL LiDAR mounting, not a calibration:
LiDAR +45 deg yaw / +0.10 m Z in body FLU; PX4 IMU origin assumed at FC centre;
zero time offset. MAVROS data_raw already converts aircraft FRD to body FLU.
"""
import argparse
import ast
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile

import yaml

STATE='lio_imu_mount_state.json'
FILES=('fastlio_body_debug.py','aligned_pose_preview.py')
FIELDS=('Q_BI','T_BI')
PX4_MOUNT={'Q_BI':'np.array([0., 0., 0., 1.])','T_BI':'np.array([0., 0., 0.])'}


def assignments(text):
    result={}
    for node in ast.parse(text).body:
        if isinstance(node,ast.Assign):
            for target in node.targets:
                if isinstance(target,ast.Name) and target.id in FIELDS:
                    if target.id in result:raise ValueError('Duplicate mounting assignment: '+target.id)
                    result[target.id]=(node,ast.get_source_segment(text,node.value))
    if set(result)!=set(FIELDS):raise ValueError('Missing Q_BI/T_BI mounting assignments')
    return result


def expressions(text):
    return {name:value for name,(_,value) in assignments(text).items()}


def equal_expressions(a,b):
    return all(ast.dump(ast.parse(a[k],mode='eval'))==ast.dump(ast.parse(b[k],mode='eval')) for k in FIELDS)


def rewrite(text,values,source):
    nodes=assignments(text);lines=text.splitlines(keepends=True)
    for name,(node,_) in sorted(nodes.items(),key=lambda item:item[1][0].lineno,reverse=True):
        lines[node.lineno-1:node.end_lineno]=[f'{name} = {values[name]}\n']
    result=''.join(lines)
    # Keep the session world quaternion and all conversion/covariance code.
    compile(result,str(source),'exec')
    return result


def parameters(config):
    blocks=[v['ros__parameters'] for v in config.values()
            if isinstance(v,dict) and isinstance(v.get('ros__parameters'),dict)]
    if len(blocks)!=1:raise ValueError('Expected one FAST-LIO ROS parameter block')
    return blocks[0]


def set_parameter(params,name,value):
    if name in params:params[name]=value;return
    group,key=name.split('.',1)
    if not isinstance(params.get(group),dict):raise ValueError('Missing FAST-LIO parameter group: '+group)
    params[group][key]=value


def px4_config(config):
    result=deepcopy(config);p=parameters(result)
    c=math.sqrt(.5)
    for key,value in {
        'common.imu_topic':'/mavros/imu/data_raw',
        'common.time_sync_en':False,'common.time_offset_lidar_to_imu':0.,
        'mapping.extrinsic_est_en':False,
        # p_body = R_body_lidar * p_lidar + t_body_lidar
        'mapping.extrinsic_R':[c,-c,0.,c,c,0.,0.,0.,1.],
        'mapping.extrinsic_T':[0.,0.,.10],
    }.items():set_parameter(p,key,value)
    return result


def verify_mount(scripts,mode):
    scripts=Path(scripts);path=scripts/STATE
    if not path.exists():
        if mode=='px4':raise ValueError('PX4 IMU mounting state missing; prepare a fresh session')
        return
    state=json.loads(path.read_text())
    if state.get('schema_version')!=1 or state.get('mode')!=mode:
        raise ValueError('IMU mode and coordinate conversion differ; prepare a fresh session')
    for name in FILES:
        expected=PX4_MOUNT if mode=='px4' else state['original_mount'][name]
        if not equal_expressions(expressions((scripts/name).read_text()),expected):
            raise ValueError('IMU coordinate conversion changed: '+name)


def configure(scripts,mode,record_path,source=None,target=None):
    if mode not in ('livox','px4'):raise ValueError('Unknown IMU source')
    scripts=Path(scripts);record_path=Path(record_path);state_path=scripts/STATE
    if record_path.exists():raise ValueError('Refusing existing IMU selection record')
    originals={name:(scripts/name).read_text() for name in FILES}
    previous_state=state_path.read_bytes() if state_path.exists() else None
    if previous_state is not None:
        state=json.loads(previous_state)
        if state.get('schema_version')!=1 or state.get('mode') not in ('livox','px4'):
            raise ValueError('Unknown saved IMU mounting state')
        for name,text in originals.items():
            actual=expressions(text)
            # A matching installer may restore a script's original mounting.
            # At a new grounded start, accept only the original or known PX4
            # assignments, then install the requested mode consistently.
            if not (equal_expressions(actual,state['original_mount'][name]) or
                    equal_expressions(actual,PX4_MOUNT)):
                raise ValueError('Custom mounting changed; refusing automatic replacement: '+name)
    else:
        state=dict(schema_version=1,mode='livox',original_mount={n:expressions(t) for n,t in originals.items()})
    replacements={name:rewrite(text,PX4_MOUNT if mode=='px4' else state['original_mount'][name],scripts/name)
                  for name,text in originals.items()}
    record=dict(schema_version=1,imu_source=mode,calibrated=False,
        imu_topic='/mavros/imu/data_raw' if mode=='px4' else 'original FAST-LIO YAML',
        assumptions=['Repository nominal mounting, not calibrated',
                     'LiDAR +45 deg yaw and +0.10 m Z relative to FC body FLU',
                     'PX4 IMU measurement origin assumed at FC centre',
                     'Zero LiDAR-to-IMU time offset; not a measured delay'] if mode=='px4' else [],
        nominal_mount_only=mode=='px4',flight_verified=False,
        lidar_topic_changed=False,fc_parameters_written=False,original_mount=state['original_mount'])
    yaml_text=None
    if mode=='px4':
        if source is None or target is None:raise ValueError('PX4 mode needs source and target YAML')
        source=Path(source);target=Path(target)
        if target.exists() or source.resolve()==target.resolve():raise ValueError('Need a new separate PX4 profile')
        data=source.read_bytes();config=px4_config(yaml.safe_load(data))
        yaml_text='# NOMINAL PX4 IMU comparison profile; extrinsics/time offset are NOT calibrated.\n'+yaml.safe_dump(config,sort_keys=False)
        record.update(source_yaml=str(source.resolve()),source_sha256=hashlib.sha256(data).hexdigest(),
                      profile=str(target.resolve()),profile_sha256=hashlib.sha256(yaml_text.encode()).hexdigest(),
                      fastlio_overrides={k:v for k,v in parameters(config).items() if k in ('common','mapping')})
    backup=Path(tempfile.mkdtemp(prefix='lio_imu_backup_',dir=scripts))
    for name in FILES:shutil.copy2(scripts/name,backup/name)
    if previous_state is not None:(backup/STATE).write_bytes(previous_state)
    state.update(mode=mode,latest_backup=str(backup))
    created=[]
    try:
        for name,text in replacements.items():
            if text!=originals[name]:(scripts/name).write_text(text)
        if yaml_text is not None:
            with target.open('x') as handle:
                created.append(target);handle.write(yaml_text)
        state_path.write_text(json.dumps(state,indent=2)+'\n')
        record.update(backup=str(backup),conversion_sha256={n:hashlib.sha256((scripts/n).read_bytes()).hexdigest() for n in FILES})
        with record_path.open('x') as handle:
            created.append(record_path);json.dump(record,handle,indent=2);handle.write('\n')
    except BaseException:
        for path in created:path.unlink(missing_ok=True)
        for name,text in originals.items():(scripts/name).write_text(text)
        if previous_state is None:state_path.unlink(missing_ok=True)
        else:state_path.write_bytes(previous_state)
        raise
    print('LIO IMU SOURCE:',mode,'; coordinate conversion:', 'body FLU / FC-centre assumption' if mode=='px4' else 'restored original Livox mounting')
    if mode=='px4':print('NOMINAL ONLY: +45 deg LiDAR yaw, +0.10 m Z, zero time offset; ground validation required. No PX4 parameter writes.')
    print('IMU selection report:',record_path)
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scripts-dir',type=Path,required=True)
    p.add_argument('--source',type=Path)
    p.add_argument('--target',type=Path)
    p.add_argument('--record',type=Path,required=True)
    p.add_argument('--imu-source',choices=('livox','px4'),required=True)
    args=p.parse_args()
    if args.imu_source=='px4' and args.source is None:
        from ament_index_python.packages import get_package_share_directory
        args.source=Path(get_package_share_directory('fast_lio'))/'config/mid360.yaml'
    configure(args.scripts_dir,args.imu_source,args.record,args.source,args.target)


if __name__=='__main__':main()
