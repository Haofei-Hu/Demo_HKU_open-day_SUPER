#!/usr/bin/env python3
"""Freeze a ground-planning map's vertical offset in the current LIO session.

Only virtual ground/ceiling and initial map-centre Z are translated. The source
YAML, clouds, motion limits, radius and physical flight envelope are preserved.
This is not a floor survey, a drift correction or a flight command.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import time

import yaml


def effective_bounds(config):
    """Pinned SUPER/ROG-Map ORIGIN_AT_CORNER resetMapSize convention."""
    m=config['rog_map']
    resolution=float(m['resolution']);inflation=float(m['inflation_resolution'])
    step=m['inflation_step']
    values=[resolution,inflation,m['virtual_ground_height'],m['virtual_ceil_height'],*m['fix_map_origin']]
    if (not all(math.isfinite(v) for v in values) or resolution<=0 or inflation<=0
            or isinstance(step,bool) or not isinstance(step,int) or step<0):
        raise ValueError('Invalid map resolution, inflation or vertical bounds')
    inflation=math.ceil(inflation/resolution)*resolution
    floor=(math.floor(m['virtual_ground_height']/inflation)+step)*inflation+inflation/2
    ceiling=(math.floor(m['virtual_ceil_height']/inflation)-step)*inflation-inflation/2
    if floor>=ceiling:raise ValueError('Inflated virtual map bounds have no free height')
    return floor,ceiling,inflation


def translate(config,reference_z):
    if not math.isfinite(reference_z):raise ValueError('Nonfinite session height')
    floor,ceiling,grid=effective_bounds(config)
    shift=math.floor(reference_z/grid)*grid
    result=deepcopy(config);m=result['rog_map']
    m['virtual_ground_height']+=shift
    m['virtual_ceil_height']+=shift
    m['fix_map_origin'][2]+=shift
    new_floor,new_ceiling,_=effective_bounds(result)
    if not (math.isclose(new_floor-floor,shift,abs_tol=1e-8) and
            math.isclose(new_ceiling-ceiling,shift,abs_tol=1e-8)):
        raise ValueError('Grid rounding changed the translated virtual clearance')
    return result,shift,(new_floor,new_ceiling)


def create(source,target,record,observation,alignment_digest):
    source,target,record=map(Path,(source,target,record))
    if source.resolve()==target.resolve() or source.parent.resolve()!=target.parent.resolve():
        raise ValueError('Use a separate profile in the source configuration directory')
    if target.is_symlink() or record.is_symlink():raise ValueError('Refusing symlink output')
    config=yaml.safe_load(source.read_text())
    z=float(observation['raw_z']);gid=observation['raw_source_gid']
    if not math.isfinite(z) or not gid:raise ValueError('Need finite observed height and source identity')
    if record.exists():
        anchor=json.loads(record.read_text())
        if (anchor['raw_source_gid']!=gid or anchor['alignment_sha256']!=alignment_digest):
            raise ValueError('Session localization source/alignment changed; start a fresh core session')
        if abs(z-anchor['raw_reference_z'])>.10:
            raise ValueError('Raw ground height changed by over 0.10 m; map reference will not follow drift/movement')
    else:
        anchor=dict(version=1,raw_reference_z=z,raw_source_gid=gid,alignment_sha256=alignment_digest)
    updated,shift,bounds=translate(config,anchor['raw_reference_z'])
    corridor=float(config['super_planner']['corridor_bound_dis'])
    if not math.isfinite(corridor) or corridor<=0:
        raise ValueError('Invalid source corridor distance')
    if not (bounds[0]+corridor<z<bounds[1]-corridor):
        raise ValueError('Session height lacks the existing corridor clearance inside the inflated map bounds')
    content='# Session vertical map translation; source SHA256: '+hashlib.sha256(source.read_bytes()).hexdigest()+'\n'
    content+=yaml.safe_dump(updated,sort_keys=False)
    if target.exists() and target.read_text()!=content:
        raise ValueError('Existing session map profile differs; refusing overwrite')
    record.parent.mkdir(parents=True,exist_ok=True)
    if not record.exists():
        with record.open('x') as handle:json.dump(anchor,handle,indent=2);handle.write('\n')
    if not target.exists():
        with target.open('x') as handle:handle.write(content)
    print(f'Session map: raw reference Z={anchor["raw_reference_z"]:.6f} m; fixed shift={shift:.6f} m')
    print(f'Effective inflated virtual floor/ceiling={bounds[0]:.6f}/{bounds[1]:.6f} m')
    print('Session profile:',target)
    print('Source YAML, point clouds, radius, motion limits and flight margins preserved.')
    return target


def capture_summary(rows):
    """Describe the exact raw-coordinate window used for the map anchor."""
    span=rows[-1][0]-rows[0][0]
    ranges=[max(r[i] for r in rows)-min(r[i] for r in rows) for i in (1,2,3)]
    change=[rows[-1][i]-rows[0][i] for i in (1,2,3)]
    return span,ranges,change


def observe(gate):
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rows=[];last_receipt=None;bad=None;last_stamp=0
    def receive(msg):
        nonlocal last_receipt,bad,last_stamp
        if msg.header.frame_id!='camera_init' or msg.child_frame_id!='body':
            bad='Unexpected raw odometry frames';return
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age=(gate.node.get_clock().now().nanoseconds-stamp)/1e9
        p=msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
            bad='Nonfinite raw position';return
        if stamp<=last_stamp or stamp<=0 or not -.05<=age<=.25:return
        last_stamp=stamp;last_receipt=time.monotonic()
        rows.append((last_receipt,p.x,p.y,p.z))
        if len(rows)>512:rows.pop(0)
    subscription=gate.node.create_subscription(Odometry,'/Odometry',receive,
        QoSProfile(depth=16,reliability=ReliabilityPolicy.BEST_EFFORT))
    identity=None
    def ready():
        nonlocal identity
        if not gate.grounded():raise RuntimeError('Ground state lost while observing map reference')
        if bad:raise RuntimeError(bad)
        if last_receipt is not None and time.monotonic()-last_receipt>.25:
            raise RuntimeError('Raw odometry gap while observing map reference')
        publishers=gate.node.get_publishers_info_by_topic('/Odometry')
        if len(publishers)>1:raise RuntimeError('Multiple raw odometry sources')
        if len(publishers)!=1:return False
        gid=bytes(publishers[0].endpoint_gid).hex()
        if identity is not None and identity!=gid:raise RuntimeError('Raw source changed during observation')
        identity=gid
        if len(rows)<50 or rows[-1][0]-rows[0][0]<3.:return False
        span,ranges,change=capture_summary(rows)
        if any(value>.025 for value in ranges):
            raise RuntimeError(
                'Raw position is not stationary within 0.025 m per axis; '
                f'raw XYZ range cm={[round(value*100,3) for value in ranges]}; '
                f'first-to-last XYZ change cm={[round(value*100,3) for value in change]}; '
                f'window={span:.3f}s; samples={len(rows)}; '
                'capture limit is separate from the 0.10 m flight/reference envelope; '
                'no map profile written')
        return True
    try:
        gate.wait(ready,'three seconds of fresh stationary raw odometry')
        gate.check()
        return dict(raw_z=sum(row[3] for row in rows)/len(rows),raw_source_gid=identity)
    finally:gate.node.destroy_subscription(subscription)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','target','record','alignment-path'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    from super_session import GroundGate,NATIVE_CONTROL_TOPICS
    gate=GroundGate()
    try:
        gate.wait(gate.localization_ready,'valid ground localization before session-map capture')
        if any(gate.node.count_publishers(topic) for topic in NATIVE_CONTROL_TOPICS):
            raise RuntimeError('Native control writer present; no session-map update')
        digest=hashlib.sha256(args.alignment_path.read_bytes()).hexdigest()
        observation=observe(gate)
        if hashlib.sha256(args.alignment_path.read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Alignment changed during map observation')
        gate.check()
        create(args.source,args.target,args.record,observation,digest)
    finally:gate.close()


if __name__=='__main__':
    main()
