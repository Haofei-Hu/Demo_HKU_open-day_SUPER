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
from rviz_venue_calibration import selected_geometry,apply_geometry,world_rotation,wireframe,validate_session,cap_rehearsal_width,cap_rehearsal_ceiling,cap_selected_box,separate_execution_geometry


class FrozenCloud:
    """Accumulate a short display snapshot, then ignore live updates until refresh."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.parts=[];self.began=None;self.message=None;self.source_stamps=[]

    def feed(self,msg,now_ns,monotonic):
        from sensor_msgs_py import point_cloud2
        if self.message is not None:return
        ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if msg.header.frame_id not in ('camera_init','world') or ns<=0 or not -.05<=(now_ns-ns)/1e9<=.25:return
        if self.source_stamps and ns<=self.source_stamps[-1]:return
        points=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
        if not isinstance(points,np.ndarray):points=np.asarray(list(points))
        if points.dtype.names:
            points=np.column_stack([points[name] for name in ('x','y','z')])
        points=np.asarray(points,dtype=np.float32).reshape(-1,3)
        points=points[np.all(np.isfinite(points),axis=1)]
        if not len(points):return
        if self.began is None:self.began=monotonic
        self.parts.append(points);self.source_stamps.append(ns)
        if monotonic-self.began>=1.:
            from std_msgs.msg import Header
            header=Header();header.frame_id='camera_init';header.stamp=msg.header.stamp
            self.message=point_cloud2.create_cloud_xyz32(header,np.concatenate(self.parts))
            self.parts=[]


def rviz_config(focal_point=(0.,0.,0.)):
    return {'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}],
        'Visualization Manager':{'Class':'','Enabled':True,
            'Global Options':{'Fixed Frame':'camera_init','Frame Rate':20},
            'Displays':[
                {'Class':'rviz_default_plugins/PointCloud2','Name':'Frozen selection map','Enabled':True,
                 'Topic':{'Value':'/super_debug/venue_calibration_cloud','Durability Policy':'Transient Local'},
                 'Style':'Squares','Size (Pixels)':5,'Decay Time':0,'Position Transformer':'XYZ','Color Transformer':'AxisColor'},
                {'Class':'rviz_default_plugins/MarkerArray','Name':'Selected venue','Enabled':True,
                 'Topic':{'Value':'/super_debug/venue_calibration','Durability Policy':'Transient Local'}}],
            # RViz chooses the first loaded tool as both default and current.
            'Tools':[{'Class':'rviz_default_plugins/PublishPoint','Topic':'/super_venue/clicked_point','Single click':False},
                     {'Class':'rviz_default_plugins/MoveCamera'},
                     {'Class':'rviz_default_plugins/Interact'}],
            'Views':{'Current':{'Class':'rviz_default_plugins/Orbit','Distance':8.,
                'Pitch':.9,'Yaw':.8,
                'Focal Point':dict(zip(('X','Y','Z'),map(float,focal_point)))}}}}


def calibrate(config,alignment_path,folder,rear_offset,rehearsal_width=None,rehearsal_ceiling=None,
              *,venue_width=None,venue_ceiling=None,full_size_execution_venue=False):
    if (rehearsal_width is not None or rehearsal_ceiling is not None) and config.get('execution_context')!='ground_rehearsal':
        raise ValueError('Width/ceiling caps are available only for ground rehearsal')
    if (rehearsal_width is not None or rehearsal_ceiling is not None) and (venue_width is not None or venue_ceiling is not None):
        raise ValueError('Use one explicit box policy per calibration')
    from super_session import GroundGate,NATIVE_CONTROL_TOPICS
    from geometry_msgs.msg import PointStamped,Point
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker,MarkerArray
    from std_srvs.srv import Trigger
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise ValueError('RViz calibration needs the onboard desktop display')
    executable=shutil.which('rviz2')
    if not executable:raise ValueError('rviz2 is not available in the sourced ROS environment')
    folder=Path(folder);alignment_path=Path(alignment_path)
    cfgpath=folder/'venue_calibration.rviz'
    if config['takeoff'].get('heading_along_length_confirmed') is not True:
        raise ValueError('Point the aircraft along the 5.8 m length before calibration')
    gate=GroundGate();viewer=None;log=None
    state={'points':[],'geometry':None,'confirm':False,'cloud_published':False,'click_epoch_ns':None}
    snapshot=FrozenCloud()
    try:
        observation=capture(gate,alignment_path);alignment_sha=digest(alignment_path)
        cfgpath.write_text(yaml.safe_dump(rviz_config(observation['raw_position']),sort_keys=False))
        rotation=world_rotation(load_converter(alignment_path),observation['raw_quaternion'])
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub=gate.node.create_publisher(MarkerArray,'/super_debug/venue_calibration',qos)
        cloud_pub=gate.node.create_publisher(PointCloud2,'/super_debug/venue_calibration_cloud',qos)
        def cloud(msg):
            snapshot.feed(msg,gate.node.get_clock().now().nanoseconds,time.monotonic())
        gate.node.create_subscription(PointCloud2,'/cloud_registered',cloud,qos_profile_sensor_data)
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
                label.pose.position.z+=.15;label.text='1 LEFT' if i==0 else '2 RIGHT';msg.markers.append(label)
            if state['geometry'] is not None:
                boxes=[(state['geometry'],20,(0.,1.,1.),'PLANNING')]
                if 'execution_envelope' in state['geometry']:
                    boxes.append((state['geometry']['execution_envelope'],21,(1.,.65,0.),'EXECUTION BOUNDARY'))
                for geometry,identifier,colour,name in boxes:
                    vertices,edges=wireframe(geometry,observation,rotation)
                    m=marker(Marker.LINE_LIST,identifier);m.scale.x=.025
                    m.color.r,m.color.g,m.color.b=colour
                    for a,b in edges:
                        for index in (a,b):
                            p=Point();p.x,p.y,p.z=map(float,vertices[index]);m.points.append(p)
                    msg.markers.append(m)
                    label=marker(Marker.TEXT_VIEW_FACING,identifier+10);label.scale.z=.15
                    label.color.r,label.color.g,label.color.b=colour
                    label.pose.position.x,label.pose.position.y,label.pose.position.z=map(float,vertices[-1])
                    label.text=name;msg.markers.append(label)
            pub.publish(msg)
        def reset():
            state.update(points=[],geometry=None,confirm=False);draw()
            print('SELECT LEFT boundary first, then RIGHT boundary, looking along the nose.',flush=True)
        def click(msg):
            ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            # Clicks reference an immutable displayed snapshot, not live vehicle feedback.
            # Do not apply the 250 ms localization age threshold to desktop interaction.
            if state['click_epoch_ns'] is None or ns<state['click_epoch_ns']:
                print('Ignored click: wait for FROZEN MAP READY, then select again.',flush=True);return
            if msg.header.frame_id not in ('world','camera_init'):
                print('Ignored click: expected camera_init fixed frame.',flush=True);return
            if len(state['points'])==2:
                print('Preview ready. Type confirm or reset in this terminal.' if state['geometry'] is not None
                      else 'Both picks are shown but invalid; type reset to select again.',flush=True);return
            p=[msg.point.x,msg.point.y,msg.point.z]
            if not np.all(np.isfinite(p)):return
            state['points'].append(p)
            if len(state['points'])==2:
                try:
                    state['geometry']=selected_geometry(*state['points'],observation,rotation,rear_offset)
                    if rehearsal_width is not None:
                        state['geometry']=cap_rehearsal_width(state['geometry'],rehearsal_width)
                    if rehearsal_ceiling is not None:
                        state['geometry']=cap_rehearsal_ceiling(state['geometry'],rehearsal_ceiling)
                    state['geometry']=cap_selected_box(state['geometry'],venue_width,venue_ceiling)
                    if full_size_execution_venue:
                        state['geometry']=separate_execution_geometry(state['geometry'])
                except ValueError as exc:
                    state['geometry']=None
                    print('Selection rejected:',exc,flush=True);draw();return
                g=state['geometry']
                width=rehearsal_width if rehearsal_width is not None else venue_width
                policy='exactly selected sides' if width is None else f'centred width cap {width:g} m; clipped to selected sides'
                print(f'PREVIEW: left={g["left_offset_m"]:.3f} m, right={g["width_m"]-g["left_offset_m"]:.3f} m, '
                      f'width={g["width_m"]:.3f} m ({policy}), rear={rear_offset:.3f} m. '
                      f'Outer ceiling={g["floor_height_limits_m"][1]:.3f} m above floor. '
                      'Cyan box is the planning boundary.',flush=True)
                if 'execution_envelope' in g:
                    e=g['execution_envelope']
                    print('EXECUTION PREVIEW: orange box = 5.8 m forward x 2.8 m wide x 2.8 m high; '
                          f'lateral limits={e["lateral_limits_m"]}; same rear, floor, heading and lateral midpoint. '
                          'This is the actual-position stop boundary, not a larger planning area. '
                          'Confirm only if the orange box matches the available venue.',flush=True)
                print('Check the complete box(es). Type confirm to accept, or reset to select again.',flush=True)
            else:print('LEFT selected. Select RIGHT boundary next.',flush=True)
            draw()
        def refresh():
            reset();snapshot.reset();state.update(cloud_published=False,click_epoch_ns=None)
            from std_msgs.msg import Header
            from sensor_msgs_py import point_cloud2
            header=Header();header.frame_id='camera_init'
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(header,[]))
            print('REFRESH: capturing a new one-second display snapshot; previous picks cleared.',flush=True)
        def confirmation(req,res):
            res.success=state['geometry'] is not None
            if res.success:state['confirm']=True
            res.message='Selection queued for current-session checks' if res.success else 'Select two valid boundaries; inspect any Selection rejected message'
            return res
        def resetting(req,res):
            reset();res.success=True;res.message='Select left, then right';return res
        def refreshing(req,res):
            refresh();res.success=True;res.message='New snapshot requested; wait for FROZEN MAP READY';return res
        gate.node.create_subscription(PointStamped,'/super_venue/clicked_point',click,qos_profile_sensor_data)
        gate.node.create_service(Trigger,'/super_venue_calibration/confirm',confirmation)
        gate.node.create_service(Trigger,'/super_venue_calibration/reset',resetting)
        gate.node.create_service(Trigger,'/super_venue_calibration/refresh',refreshing)
        log=(folder/'venue_calibration_rviz.log').open('w')
        viewer=subprocess.Popen([executable,'-d',str(cfgpath)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        reset()
        print('RVIZ: Publish Point is selected automatically (shortcut U). Wait for FROZEN MAP READY. '
              'Click a visible point on wall/table boundary surfaces, not empty space or the nearest obstacle. '
              'Sides are parallel to the nose; rear placement is the supplied offset. No flight requests.',flush=True)
        last_check=0.
        while not state['confirm']:
            gate.ros.spin_once(gate.node,timeout_sec=.05);gate.check()
            if snapshot.message is not None and not state['cloud_published']:
                cloud_pub.publish(snapshot.message)
                state.update(cloud_published=True,click_epoch_ns=gate.node.get_clock().now().nanoseconds)
                print(f'FROZEN MAP READY: {snapshot.message.width} points; live updates hidden. '
                      'Select 1 LEFT then 2 RIGHT. Type refresh for a new snapshot.',flush=True)
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
                elif command=='refresh':refresh()
                elif command=='confirm' and state['geometry'] is not None:state['confirm']=True
                else:print('Use confirm, reset (keep snapshot), or refresh (new snapshot).',flush=True)
        current=capture(gate,alignment_path)
        result=apply_geometry(config,state['geometry'],observation,alignment_sha)
        result['venue_calibration']['display_snapshot']=dict(
            topic='/super_debug/venue_calibration_cloud',points=snapshot.message.width,
            source_topic='/cloud_registered',source_stamp_first_ns=snapshot.source_stamps[0],
            source_stamp_last_ns=snapshot.source_stamps[-1],live_updates_hidden=True)
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
