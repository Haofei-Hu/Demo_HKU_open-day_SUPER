#!/usr/bin/env python3
"""One disarmed vertical SUPER goal, expressed as FC-centre floor height.

Uses paired raw/aligned geometry and the terminal flat-output yaw to invert the
FC-centre conversion. Publishes /goal_pose only; no native setpoints or flight.
"""
import argparse
import hashlib
import math
from pathlib import Path
import time

import numpy as np
import yaml
from control_bridge_preview import Mapping, arrays, load_converter, quaternion_rpy


def height_goal(mapping, raw_pose, aligned_pose, fc_height):
    if not math.isfinite(fc_height) or not .085 < fc_height <= 1.:
        raise ValueError('FC floor height must be in (0.085, 1] m')
    if mapping.failed or mapping.origin is None or mapping.pairs < 10:
        raise ValueError('Need ten matched poses with fixed geometry')
    _,raw_q=arrays(raw_pose)
    aligned_p,_=arrays(aligned_pose)
    x,y,z,w=raw_q
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    # SUPER uses yaw and zero terminal derivatives to generate flat attitude.
    goal_q=quaternion_rpy(0.,0.,yaw)
    target=aligned_p+np.array([0.,0.,fc_height-.085])
    zero=mapping.converter(np.zeros(3),goal_q,np.eye(6))[0]
    basis=np.column_stack([mapping.converter(axis,goal_q,np.eye(6))[0]-zero for axis in np.eye(3)])
    if (not np.all(np.isfinite(basis)) or not np.all(np.isfinite(zero))
            or not np.allclose(basis.T@basis,np.eye(3),atol=1e-8,rtol=0)
            or abs(np.linalg.det(basis)-1.)>1e-8):
        raise ValueError('Expected a rigid position conversion')
    raw_goal=np.linalg.solve(basis,target+mapping.origin-zero)
    check=mapping.converter(raw_goal,goal_q,np.eye(6))[0]-mapping.origin
    if not np.all(np.isfinite(raw_goal)) or not np.allclose(check,target,atol=1e-8,rtol=0):
        raise ValueError('Goal conversion round-trip failed')
    return raw_goal,goal_q,target


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--height',type=float,required=True)
    parser.add_argument('--alignment-path',type=Path,default=Path.home()/'Downloads/aligned_pose_preview.py')
    parser.add_argument('--planner-config',type=Path,required=True)
    args=parser.parse_args(argv)
    if not math.isfinite(args.height) or not .085 < args.height <= 1.:
        parser.error('FC floor height must be in (0.085, 1] m')
    config_bytes=args.planner_config.read_bytes()
    config=yaml.safe_load(config_bytes)
    if (config['fsm']['click_height']!=-6. or config['fsm']['click_goal_topic']!='/goal_pose'
            or not config['fsm']['click_goal_en']
            or config['traj_opt']['boundary']['max_vel']!=.25
            or config['traj_opt']['boundary']['max_acc']!=.45):
        raise SystemExit('Need the reserved low-speed-3d configuration; no goal sent.')
    alignment_hash=hashlib.sha256(args.alignment_path.read_bytes()).digest()
    mapping=Mapping(load_converter(args.alignment_path))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from mavros_msgs.msg import State, ExtendedState
    from std_msgs.msg import String
    rclpy.init()
    node=rclpy.create_node('send_ground_height_goal')
    received={};raw={};aligned={};matched=None;last_pair=0
    def store(name,msg):
        received[name]=(msg,time.monotonic())
    def stamp(msg):
        return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
    def fresh(msg,limit):
        ns=stamp(msg);age=(node.get_clock().now().nanoseconds-ns)/1e9
        return ns>0 and -.05<=age<=limit
    def cache(bucket,msg):
        bucket[stamp(msg)]=msg
        while len(bucket)>100:bucket.pop(next(iter(bucket)))
    inputs=[
        node.create_subscription(Odometry,'/Odometry',lambda m:cache(raw,m),qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov',lambda m:cache(aligned,m),qos_profile_sensor_data),
        node.create_subscription(State,'/mavros/state',lambda m:store('state',m),qos_profile_sensor_data),
        node.create_subscription(ExtendedState,'/mavros/extended_state',lambda m:store('ground',m),qos_profile_sensor_data),
        node.create_subscription(String,'/super_debug/bounded_reference_status',lambda m:store('status',m),qos_profile_sensor_data)]
    publisher=node.create_publisher(PoseStamped,'/goal_pose',10)
    try:
        end=time.monotonic()+8
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if 'state' in received and received['state'][0].armed:
                raise ValueError('Arming observed; no goal sent')
            for ns in sorted(raw.keys() & aligned.keys()):
                r,a=raw.pop(ns),aligned.pop(ns)
                if ns<=last_pair or not fresh(r,.25) or not fresh(a,.25):continue
                if r.header.frame_id!='camera_init' or r.child_frame_id!='body' or a.header.frame_id!='px4_enu_preview':
                    raise ValueError('Unexpected pose frames')
                mapping.pair(r.pose.pose,a.pose.pose);matched=(r,a);last_pair=ns
            if matched is None or mapping.pairs<10 or not all(k in received for k in ('state','ground','status')):continue
            s,g,status=(received[k][0] for k in ('state','ground','status'))
            if (not fresh(s,2) or not fresh(g,2) or not fresh(matched[0],.25)
                    or time.monotonic()-received['status'][1]>2):continue
            if not s.connected or g.landed_state!=ExtendedState.LANDED_STATE_ON_GROUND:
                raise ValueError('Need connected/disarmed/on-ground aircraft')
            if not status.data.startswith('WAIT_TRAJECTORY:'):
                raise ValueError('Start a fresh bounded preview and wait for WAIT_TRAJECTORY')
            if publisher.get_subscription_count()<1:continue
            if (hashlib.sha256(args.alignment_path.read_bytes()).digest()!=alignment_hash
                    or args.planner_config.read_bytes()!=config_bytes):
                raise ValueError('Session geometry/config changed; no goal sent')
            p,q,target=height_goal(mapping,matched[0].pose.pose,matched[1].pose.pose,args.height)
            goal=PoseStamped();goal.header.stamp=node.get_clock().now().to_msg();goal.header.frame_id='world'
            goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=map(float,p)
            goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w=map(float,q)
            publisher.publish(goal)
            print(f'GROUND HEIGHT GOAL ONLY: FC floor height={args.height:g} m; lift={args.height-.085:.3f} m; fixed aligned XY.')
            print('Requested aligned FC-centre xyz:',target.round(6).tolist())
            print('Requested planner xyz:',p.round(6).tolist())
            print('Inspect accepted map-snapped goal and END_HOLD height; no flight requests or native output.')
            end=time.monotonic()+.5
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
            return
        raise ValueError('Timed out waiting for fresh ground state, matched geometry and WAIT_TRAJECTORY')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
