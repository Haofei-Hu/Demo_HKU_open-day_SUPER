"""Capture the grounded FC venue anchor and add only its planes to a new profile."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from control_bridge_preview import Mapping,arrays,load_converter
from demo_mission import DemoMission
from live_venue import SCOPE,SOURCE_TOPICS,digest,source_ids
from venue_geometry import planes_for_frame,yaw


def capture(gate,alignment_path):
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import qos_profile_sensor_data
    from super_session import NATIVE_CONTROL_TOPICS
    alignment_path=Path(alignment_path);before=alignment_path.read_bytes()
    mapping=Mapping(load_converter(alignment_path));raw={};aligned={};latest={};locked=None
    def pose(msg,kind):
        if (kind=='raw' and (msg.header.frame_id!='camera_init' or msg.child_frame_id!='body')
                or kind=='aligned' and msg.header.frame_id!='px4_enu_preview'):
            raise ValueError('Unexpected live venue localization frame')
        ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age=(gate.node.get_clock().now().nanoseconds-ns)/1e9
        if ns<=latest.get('stamp',0) or not -.05<=age<=.25:return
        bucket=raw if kind=='raw' else aligned;bucket[ns]=msg.pose.pose
        while len(bucket)>32:bucket.pop(next(iter(bucket)))
        if ns in raw and ns in aligned:
            r,a=raw.pop(ns),aligned.pop(ns);mapping.pair(r,a)
            latest.update(raw=r,aligned=a,stamp=ns)
    subscriptions=[
        gate.node.create_subscription(Odometry,'/Odometry',lambda m:pose(m,'raw'),qos_profile_sensor_data),
        gate.node.create_subscription(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov',
                                      lambda m:pose(m,'aligned'),qos_profile_sensor_data)]
    def ready():
        nonlocal locked
        gate.check()
        if any(gate.node.count_publishers(t) for t in NATIVE_CONTROL_TOPICS):
            raise ValueError('Native writer present during live venue capture')
        # Discovery can still be incomplete. A duplicate is an error; zero waits.
        for topic in SOURCE_TOPICS:
            count=gate.node.count_publishers(topic)
            if count>1:raise ValueError('Multiple live venue sources: '+topic)
            if count==0:return False
        ids=source_ids(gate.node)
        if locked is not None and ids!=locked:raise ValueError('Live venue source changed during capture')
        locked=ids
        if mapping.pairs<10 or not gate.localization_ready():return False
        if (gate.node.get_clock().now().nanoseconds-latest['stamp'])/1e9>.25:return False
        p,q=arrays(gate.messages['local'].pose);aligned_p,aligned_q=arrays(latest['aligned'])
        if np.any(np.abs(p-aligned_p)>.10):raise ValueError('FC/aligned pose discrepancy exceeds 0.10 m per axis')
        delta=(yaw(q)-yaw(aligned_q)+math.pi)%(2*math.pi)-math.pi
        if abs(delta)>math.radians(5):raise ValueError('FC and external heading disagree')
        return True
    try:
        gate.wait(ready,'ten fresh paired poses and grounded live venue sources')
        if alignment_path.read_bytes()!=before:raise ValueError('Alignment changed during live venue capture')
        p,q=arrays(gate.messages['local'].pose);raw_p,raw_q=arrays(latest['raw'])
        return dict(ground_fc_xyz=p.tolist(),ground_fc_yaw_rad=yaw(q),origin=mapping.origin.tolist(),
                    raw_position=raw_p.tolist(),raw_quaternion=raw_q.tolist(),
                    raw_stamp_ns=latest['stamp'],local_stamp_ns=(gate.messages['local'].header.stamp.sec*1_000_000_000+
                                                                gate.messages['local'].header.stamp.nanosec),
                    source_ids=locked)
    finally:
        for sub in subscriptions:gate.node.destroy_subscription(sub)


def create(source,mission_file,alignment_path,record,gate):
    source=Path(source);mission_file=Path(mission_file);alignment_path=Path(alignment_path);record=Path(record)
    if record.exists():raise ValueError('Refusing an existing live venue reference: '+str(record))
    mission=DemoMission.load(mission_file)
    if mission.rehearsal_only or str(mission.config.get('status','')).startswith('SYNTHETIC'):
        raise ValueError('Live preparation requires physical mission measurements')
    mission_sha=digest(mission_file);alignment_sha=digest(alignment_path);original=source.read_bytes()
    observation=capture(gate,alignment_path)
    if (digest(mission_file)!=mission_sha or digest(alignment_path)!=alignment_sha or source.read_bytes()!=original):
        raise ValueError('Live venue inputs changed during capture')
    planes,data=planes_for_frame(mission,load_converter(alignment_path),observation['raw_quaternion'],
                                observation['origin'],observation['ground_fc_xyz'],observation['ground_fc_yaw_rad'])
    config=yaml.safe_load(original);config['super_planner']['venue_halfspaces']=planes.reshape(-1).tolist()
    target=source.with_name(source.stem+'_live_venue.yaml')
    if target.exists():raise ValueError('Refusing an existing live venue profile: '+str(target))
    text='# Live planner and controller share one immutable grounded venue frame.\n'+yaml.safe_dump(config,sort_keys=False)
    data.update(observation,scope=SCOPE,mission_sha256=mission_sha,source_alignment_sha256=alignment_sha,
                source_planner_sha256=digest(source),planner_profile=str(target.resolve()),
                planner_profile_sha256=hashlib.sha256(text.encode()).hexdigest(),
                native_control_or_flight_requests=False)
    record.parent.mkdir(parents=True,exist_ok=True)
    with target.open('x') as handle:handle.write(text)
    with record.open('x') as handle:json.dump(data,handle,indent=2);handle.write('\n')
    print('LIVE VENUE: one captured FC ground/yaw for planner and controller; source configuration and clouds unchanged')
    print('Live venue reference:',record)
    return target
