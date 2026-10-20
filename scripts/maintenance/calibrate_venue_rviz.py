#!/usr/bin/env python3
"""Select current-session left/right venue bounds in RViz while disarmed."""
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

from control_bridge_preview import load_converter
from create_live_venue_profile import capture
from live_venue import digest,source_ids
from rviz_venue_calibration import selected_geometry,apply_geometry,world_rotation,wireframe,validate_session


def rviz_config():
    return {'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}],
        'Visualization Manager':{'Class':'','Enabled':True,
            'Global Options':{'Fixed Frame':'camera_init','Frame Rate':20},
            'Displays':[
                {'Class':'rviz_default_plugins/PointCloud2','Name':'Live map','Enabled':True,
                 'Topic':{'Value':'/cloud_registered','Reliability Policy':'Best Effort'},
                 'Style':'Points','Size (Pixels)':5,'Decay Time':0,'Position Transformer':'XYZ','Color Transformer':'AxisColor'},
                {'Class':'rviz_default_plugins/MarkerArray','Name':'Selected venue','Enabled':True,
                 'Topic':{'Value':'/super_debug/venue_calibration','Durability Policy':'Transient Local'}}],
            'Tools':[{'Class':'rviz_default_plugins/Interact'},
                     {'Class':'rviz_default_plugins/MoveCamera'},
                     {'Class':'rviz_default_plugins/PublishPoint','Topic':'/super_venue/clicked_point','Single click':False}]}}


def calibrate(config,alignment_path,folder,rear_offset):
    from super_session import GroundGate,NATIVE_CONTROL_TOPICS
    from geometry_msgs.msg import PointStamped,Point
    from visualization_msgs.msg import Marker,MarkerArray
    from std_srvs.srv import Trigger
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise ValueError('RViz calibration needs the onboard desktop display')
    executable=shutil.which('rviz2')
    if not executable:raise ValueError('rviz2 is not available in the sourced ROS environment')
    folder=Path(folder);alignment_path=Path(alignment_path)
    cfgpath=folder/'venue_calibration.rviz';cfgpath.write_text(yaml.safe_dump(rviz_config(),sort_keys=False))
    if config['takeoff'].get('heading_along_length_confirmed') is not True:
        raise ValueError('Point the aircraft along the 5.8 m length before calibration')
    gate=GroundGate();viewer=None;log=None
    state={'points':[],'geometry':None,'confirm':False}
    try:
        observation=capture(gate,alignment_path);alignment_sha=digest(alignment_path)
        rotation=world_rotation(load_converter(alignment_path),observation['raw_quaternion'])
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub=gate.node.create_publisher(MarkerArray,'/super_debug/venue_calibration',qos)
        def draw():
            msg=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;msg.markers.append(clear)
            def marker(kind,index):
                m=Marker();m.header.frame_id='camera_init';m.header.stamp=gate.node.get_clock().now().to_msg()
                m.ns='venue_calibration';m.id=index;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;m.color.a=1.
                return m
            for i,p in enumerate(state['points']):
                m=marker(Marker.SPHERE,i);m.scale.x=m.scale.y=m.scale.z=.08
                m.color.g=1. if i==0 else 0.;m.color.r=1. if i==1 else 0.
                m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,p);msg.markers.append(m)
                label=marker(Marker.TEXT_VIEW_FACING,10+i);label.scale.z=.15;label.color.r=label.color.g=label.color.b=1.
                label.pose.position.x,label.pose.position.y,label.pose.position.z=map(float,p)
                label.pose.position.z+=.15;label.text='LEFT' if i==0 else 'RIGHT';msg.markers.append(label)
            if state['geometry'] is not None:
                vertices,edges=wireframe(state['geometry'],observation,rotation)
                m=marker(Marker.LINE_LIST,20);m.scale.x=.025;m.color.g=m.color.b=1.
                for a,b in edges:
                    for index in (a,b):
                        p=Point();p.x,p.y,p.z=map(float,vertices[index]);m.points.append(p)
                msg.markers.append(m)
            pub.publish(msg)
        def reset():
            state.update(points=[],geometry=None,confirm=False);draw()
            print('SELECT LEFT boundary first, then RIGHT boundary, looking along the nose.',flush=True)
        def click(msg):
            ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            age=(gate.node.get_clock().now().nanoseconds-ns)/1e9
            if msg.header.frame_id not in ('world','camera_init') or ns<=0 or not -.05<=age<=.25:
                print('Ignored click: wrong frame or stale timestamp.',flush=True);return
            if len(state['points'])==2:
                print('Preview ready. Type confirm or reset in this terminal.',flush=True);return
            p=[msg.point.x,msg.point.y,msg.point.z]
            if not np.all(np.isfinite(p)):return
            state['points'].append(p)
            if len(state['points'])==2:
                try:state['geometry']=selected_geometry(*state['points'],observation,rotation,rear_offset)
                except ValueError as exc:
                    print('Selection rejected:',exc,flush=True);reset();return
                g=state['geometry']
                print(f'PREVIEW: left={g["left_offset_m"]:.3f} m, right={g["width_m"]-g["left_offset_m"]:.3f} m, '
                      f'width={g["width_m"]:.3f} m (maximum 2.8), rear={rear_offset:.3f} m. '
                      'Check the complete cyan box against the actual venue. Type confirm to accept, or reset to select again.',flush=True)
            else:print('LEFT selected. Select RIGHT boundary next.',flush=True)
            draw()
        def confirmation(req,res):
            res.success=state['geometry'] is not None
            if res.success:state['confirm']=True
            res.message='Selection queued for current-session checks' if res.success else 'Select both boundaries first'
            return res
        def resetting(req,res):
            reset();res.success=True;res.message='Select left, then right';return res
        gate.node.create_subscription(PointStamped,'/super_venue/clicked_point',click,qos_profile_sensor_data)
        gate.node.create_service(Trigger,'/super_venue_calibration/confirm',confirmation)
        gate.node.create_service(Trigger,'/super_venue_calibration/reset',resetting)
        log=(folder/'venue_calibration_rviz.log').open('w')
        viewer=subprocess.Popen([executable,'-d',str(cfgpath)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        reset()
        print('RVIZ: choose Publish Point. Select wall/table boundary surfaces, not the nearest obstacle. '
              'Sides are parallel to the nose; rear placement is the supplied offset. No flight requests.',flush=True)
        last_check=0.
        while not state['confirm']:
            gate.ros.spin_once(gate.node,timeout_sec=.05);gate.check()
            if viewer.poll() is not None:raise ValueError('Calibration RViz closed before confirmation; see its log')
            if any(gate.node.count_publishers(t) for t in NATIVE_CONTROL_TOPICS):
                raise ValueError('Native control publisher appeared during calibration')
            if time.monotonic()-last_check>1:
                if digest(alignment_path)!=alignment_sha or source_ids(gate.node)!=observation['source_ids']:
                    raise ValueError('Localization changed; calibrate the fresh session again')
                last_check=time.monotonic()
            if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
                command=sys.stdin.readline().strip().lower()
                if command=='reset':reset()
                elif command=='confirm' and state['geometry'] is not None:state['confirm']=True
                else:print('Use confirm after selecting both points, or reset.',flush=True)
        current=capture(gate,alignment_path)
        result=apply_geometry(config,state['geometry'],observation,alignment_sha)
        validate_session(result['venue_calibration'],current,digest(alignment_path))
        # Keep the selected anchor; a fresh sample never translates the marked walls.
        path=folder/'venue_calibration.json'
        with path.open('x') as handle:json.dump(result['venue_calibration'],handle,indent=2);handle.write('\n')
        print('CALIBRATION: CONFIRMED; current session only. REPORT:',path,flush=True)
        return result
    finally:
        if viewer is not None and viewer.poll() is None:
            os.killpg(viewer.pid,signal.SIGINT)
            try:viewer.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(viewer.pid,signal.SIGKILL);viewer.wait()
        if log is not None:log.close()
        gate.close()
