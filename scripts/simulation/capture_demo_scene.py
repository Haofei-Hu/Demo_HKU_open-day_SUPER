#!/usr/bin/env python3
"""Read one fresh map and stamp-paired localization for isolated rehearsal."""
import argparse
import base64
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--alignment-path',type=Path,required=True)
    p.add_argument('--planner-config',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise ValueError('Scene output already exists')
    from super_session import GroundGate,NATIVE_CONTROL_TOPICS
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from control_bridge_preview import Mapping,load_converter,arrays
    gate=GroundGate();mapping=Mapping(load_converter(args.alignment_path))
    raw={};aligned={};latest={};subs=[]
    def pose(msg,kind):
        if (kind=='raw' and (msg.header.frame_id!='camera_init' or msg.child_frame_id!='body')
                or kind=='aligned' and msg.header.frame_id!='px4_enu_preview'):
            raise ValueError('Unexpected scene localization frame')
        ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age=(gate.node.get_clock().now().nanoseconds-ns)/1e9
        if not -.05<=age<=.25:return
        cache=raw if kind=='raw' else aligned;cache[ns]=msg
        while len(cache)>32:cache.pop(next(iter(cache)))
        if ns in raw and ns in aligned:
            r,a=raw.pop(ns),aligned.pop(ns)
            mapping.pair(r.pose.pose,a.pose.pose);latest.update(raw=r,aligned=a,stamp=ns)
    subs.append(gate.node.create_subscription(Odometry,'/Odometry',lambda m:pose(m,'raw'),qos_profile_sensor_data))
    subs.append(gate.node.create_subscription(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov',
                                            lambda m:pose(m,'aligned'),qos_profile_sensor_data))
    subs.append(gate.node.create_subscription(PointCloud2,'/rog_map/occ',lambda m:latest.update(cloud=m),qos_profile_sensor_data))
    digest=hashlib.sha256(args.alignment_path.read_bytes()).hexdigest()
    def ready():
        gate.check()
        if any(gate.node.count_publishers(t) for t in NATIVE_CONTROL_TOPICS):raise ValueError('Native writer present during scene capture')
        for topic in ('/Odometry','/super_debug/aligned_pose_cov','/rog_map/occ'):
            count=gate.node.count_publishers(topic)
            if count>1:raise ValueError('Multiple scene sources: '+topic)
            if count!=1:return False
        if mapping.pairs<10 or 'cloud' not in latest:return False
        cloud=latest['cloud'];now=gate.node.get_clock().now().nanoseconds
        if cloud.header.frame_id not in ('world','camera_init'):raise ValueError('Unexpected occupied-map frame')
        cloud_ns=cloud.header.stamp.sec*1_000_000_000+cloud.header.stamp.nanosec
        return (gate.localization_ready() and (now-latest['stamp'])/1e9<=.25 and
                -.05<=(now-cloud_ns)/1e9<=2. and cloud.width*cloud.height>0)
    try:
        gate.wait(ready,'fresh nonempty map and ten fixed-geometry pose pairs',timeout=20.)
        if hashlib.sha256(args.alignment_path.read_bytes()).hexdigest()!=digest:raise ValueError('Alignment changed during capture')
        raw_p,raw_q=arrays(latest['raw'].pose.pose);p,q=arrays(latest['aligned'].pose.pose)
        binary=serialize_message(latest['cloud'])
        if len(binary)>32*1024*1024:raise ValueError('Scene exceeds the 32 MiB snapshot budget')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        frozen=args.output.with_name('scene_alignment.py');geometry=args.alignment_path.read_bytes()
        if frozen.exists() and frozen.read_bytes()!=geometry:raise ValueError('Refusing conflicting frozen scene geometry')
        if not frozen.exists():
            with frozen.open('xb') as handle:handle.write(geometry)
        data=dict(schema_version=1,source='fresh aircraft grounded map snapshot',alignment_path=str(frozen),
            alignment_sha256=digest,planner_config=str(args.planner_config),
            planner_sha256=hashlib.sha256(args.planner_config.read_bytes()).hexdigest(),raw_position=raw_p.tolist(),
            raw_quaternion=raw_q.tolist(),aligned_position=p.tolist(),aligned_quaternion=q.tolist(),
            origin=mapping.origin.tolist(),raw_stamp_ns=latest['stamp'],
            cloud_points=latest['cloud'].width*latest['cloud'].height,
            cloud_original_stamp_ns=latest['cloud'].header.stamp.sec*1_000_000_000+latest['cloud'].header.stamp.nanosec,
            cloud_serialized_base64=base64.b64encode(binary).decode('ascii'),
            native_control_or_flight_requests=False)
        with args.output.open('x') as handle:json.dump(data,handle,indent=2);handle.write('\n')
        print('Captured scene:',args.output,'points:',data['cloud_points'],flush=True)
    finally:
        for sub in subs:gate.node.destroy_subscription(sub)
        gate.close()


if __name__=='__main__':main()
