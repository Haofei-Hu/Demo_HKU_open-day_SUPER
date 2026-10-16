#!/usr/bin/env python3
"""Complete mission controller rehearsal with an ideal synthetic ROS backend.

The default backend uses a synthetic planner. With --scene-file, real SUPER
plans against a captured static occupied map. Both use virtual FC/vehicle
feedback; neither runs PX4 SITL or aircraft hardware. This tests
the mission supervisor, goal sequencing, setpoint transport and reporting.
An isolated ROS domain and explicit fixture-geometry flag prevent confusion
with aircraft preparation or a surveyed venue.
"""
import argparse
import base64
import hashlib
import json
import math
import multiprocessing
import os
import signal
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import time


class RehearsalViewer:
    """Own only this run's optional viewer, after the domain isolation check."""
    def __init__(self,report,enabled=True):
        self.folder=Path(report).parent
        self.process=None;self.log=None
        self.state=dict(requested=enabled,status='pending' if enabled else 'disabled',
                        domain_id=197,localhost_only=True,rendering_verified=False)

    def save(self):
        self.folder.mkdir(parents=True,exist_ok=True)
        (self.folder/'rviz_status.json').write_text(json.dumps(self.state,indent=2)+'\n')

    def start(self,scene):
        if not self.state['requested']:
            self.save();return
        reason=None
        if os.environ.get('ROS_DOMAIN_ID')!='197' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise ValueError('Rehearsal viewer requires isolated localhost domain 197')
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            reason='No graphical desktop is available'
        elif shutil.which('rviz2') is None:reason='rviz2 executable is not installed'
        if reason:
            self.state.update(status='skipped',reason=reason);self.save()
            print('RVIZ: skipped; '+reason,flush=True);return
        # RViz accepts JSON as a YAML subset. The installer already verifies JSON artifacts.
        config=self.folder/'rehearsal_view.rviz'
        config.write_text(json.dumps(rehearsal_view_config(scene),indent=2)+'\n')
        logpath=self.folder/'rviz.log'
        self.state.update(config=str(config),log=str(logpath))
        try:
            self.log=logpath.open('w')
            self.process=subprocess.Popen(['rviz2','-d',str(config),'--ros-args',
                '-r','__node:=super_demo_rviz'],stdout=self.log,stderr=subprocess.STDOUT,
                start_new_session=True,env=dict(os.environ,ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1'))
            self.state.update(status='launched',pid=self.process.pid)
            print('RVIZ: launched for this rehearsal; log: '+str(logpath),flush=True)
        except OSError as exc:
            if self.log is not None:self.log.close();self.log=None
            self.state.update(status='unavailable',reason=str(exc))
            print('RVIZ: unavailable; '+str(exc),flush=True)
        self.save()

    def close(self):
        if self.process is not None:
            code=self.process.poll()
            if code is None:
                try:os.killpg(self.process.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:code=self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:os.killpg(self.process.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                    code=self.process.wait(timeout=3)
                self.state.update(status='closed',closed_by_runner=True,returncode=code)
            else:self.state.update(status='exited',closed_by_runner=False,returncode=code)
        if self.log is not None:self.log.close();self.log=None
        self.save()


def rehearsal_view_config(scene):
    """All visible data uses raw planner/world coordinates, including the vehicle."""
    def topic(name):
        return {'Value':name,'Depth':5,'Reliability Policy':'Best Effort',
                'Durability Policy':'Volatile','History Policy':'Keep Last'}
    def display(kind,name,path,**properties):
        return dict(Class='rviz_default_plugins/'+kind,Name=name,Enabled=True,
                    Topic=topic(path),**properties)
    displays=[display('Pose','VirtualVehicle','/demo_scene/vehicle_pose',Shape='Arrow',Color='255; 85; 0',
                      **{'Alpha':1.,'Shaft Length':.25,'Shaft Radius':.03,'Head Length':.12,'Head Radius':.07})]
    if scene:
        for name,path in (('CapturedScene','/demo_scene/occupied_points'),('OccupiedMap','/rog_map/occ')):
            displays.append(display('PointCloud2',name,path,Style='Points',Alpha=1.,
                **{'Position Transformer':'XYZ','Color Transformer':'AxisColor','Axis':'Z',
                   'Size (Pixels)':2,'Size (m)':.02,'Decay Time':0}))
        for name,path in (('ExpTraj','/visualization/exp_traj'),
                          ('BackupTraj','/visualization/backup_traj'),('PlannerGoal','/visualization/goal')):
            displays.append(display('MarkerArray',name,path))
        displays.append(display('Path','PlannerPath','/fsm/path',Color='245; 121; 0',
                                **{'Line Style':'Lines','Line Width':.03,'Buffer Length':1}))
    return {'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}],
        'Visualization Manager':{'Class':'','Enabled':True,'Displays':displays,
            'Global Options':{'Fixed Frame':'world','Frame Rate':20,'Background Color':'35; 35; 35'},
            'Tools':[{'Class':'rviz_default_plugins/'+tool} for tool in ('Interact','MoveCamera','FocusCamera')],
            'Views':{'Current':{'Class':'rviz_default_plugins/Orbit','Name':'Current View',
                'Target Frame':'world','Distance':8.,'Pitch':.7,'Yaw':.8,
                'Focal Point':dict(zip(('X','Y','Z'),scene['raw_position'] if scene else [.12,-.08,.175]))}}},
        'Window Geometry':{'Width':1100,'Height':750,'Hide Left Dock':False,'Hide Right Dock':False}}


def rehearsal_profile(source,report):
    """Adapt only private snapshot IO; never edit the real planner profile."""
    import yaml
    source=Path(source);cfg=yaml.safe_load(source.read_text())
    cfg['rog_map']['ros_callback']['cloud_topic']='/demo_scene/occupied_points'
    cfg['rog_map']['raycasting']['enable']=False
    token=hashlib.sha256(str(Path(report).resolve()).encode()).hexdigest()[:12]
    target=source.with_name('mid360_demo_rehearsal_'+token+'.yaml')
    if target.exists():raise ValueError('Rehearsal configuration already exists')
    with target.open('x') as handle:
        handle.write('# Private-domain static occupied-map rehearsal; source YAML unchanged\n'+yaml.safe_dump(cfg,sort_keys=False))
    return target


def backend(connection,fault,scene_file=None):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,TwistStamped
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State,ExtendedState,EstimatorStatus,RCIn,PositionTarget
    from mavros_msgs.srv import SetMode,CommandBool
    from mars_quadrotor_msgs.msg import PositionCommand,PolynomialTrajectory
    from first_hover_terminal import PARAMETERS
    from bounded_reference_preview import Plan
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init();node=Node('param',namespace='/mavros')
    for k,v in dict(PARAMETERS,MPC_XY_VEL_MAX=2.).items():node.declare_parameter(k,v)
    frame=Node('setpoint_position',namespace='/mavros');frame.declare_parameter('mav_frame','LOCAL_NED')
    from rclpy.executors import SingleThreadedExecutor
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(frame)
    ground=[.12,-.08,-.74];pose=list(ground);velocity=[0.,0.,0.];yaw=.6
    raw_q=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)];aligned_q=list(raw_q)
    raw_rotation=None;raw_zero=None;origin=None
    if scene_file is not None:
        import numpy as np
        from control_bridge_preview import load_converter
        scene=json.loads(Path(scene_file).read_text());convert=load_converter(Path(scene['alignment_path']))
        ground=scene['aligned_position'];pose=list(ground);raw_q=scene['raw_quaternion'];aligned_q=scene['aligned_quaternion']
        x,y,z,w=aligned_q;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        raw_zero=convert(np.zeros(3),np.array(raw_q),np.eye(6))[0]
        raw_rotation=np.column_stack([convert(v,np.array(raw_q),np.eye(6))[0]-raw_zero for v in np.eye(3)])
        if not np.allclose(raw_rotation.T@raw_rotation,np.eye(3),atol=1e-8,rtol=0):
            raise ValueError('Scene conversion must be rigid')
        origin=np.array(scene['origin'])
    state=dict(mode='POSCTL',armed=False,landed=True,plan=None,goals=[],commands=[],peak_native_speed=0.,
               max_lateral_displacement_m=0.,rest_waits=0,resume_goal=None,
               future_plan=None,future_replans=0,future_replanned_goal=None)
    state['target']=None
    kinds={'/Odometry':Odometry,'/mavros/state':State,'/mavros/extended_state':ExtendedState,
        '/mavros/estimator_status':EstimatorStatus,'/mavros/rc/in':RCIn,
        '/mavros/local_position/pose':PoseStamped,'/mavros/local_position/velocity_local':TwistStamped,
        '/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,'/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped}
    pubs={name:node.create_publisher(kind,name,qos_profile_sensor_data) for name,kind in kinds.items()}
    vehiclepub=node.create_publisher(PoseStamped,'/demo_scene/vehicle_pose',qos_profile_sensor_data)
    polypub=node.create_publisher(PolynomialTrajectory,'/planning_cmd/poly_traj',10) if scene_file is None else None
    cmdpub=node.create_publisher(PositionCommand,'/planning/pos_cmd',10) if scene_file is None else None
    if scene_file is not None:
        from sensor_msgs.msg import PointCloud2
        from rclpy.serialization import deserialize_message
        cloud=deserialize_message(base64.b64decode(scene['cloud_serialized_base64']),PointCloud2)
        cloudpub=node.create_publisher(PointCloud2,'/demo_scene/occupied_points',10)
        def map_snapshot():
            # A frozen obstacle model, not retimed live sensor measurements.
            cloud.header.stamp=node.get_clock().now().to_msg();cloud.header.frame_id='world';cloudpub.publish(cloud)
        node.create_timer(.5,map_snapshot)
    def mode(req,res):
        state['commands'].append(req.custom_mode);state['mode']=req.custom_mode;res.mode_sent=True;return res
    def arm(req,res):
        state['commands'].append('ARM' if req.value else 'DISARM')
        state['armed']=req.value;state['landed']=not req.value;res.success=True;return res
    def target(msg):
        if state['armed'] and state['mode']=='OFFBOARD':
            state['target']=(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                             [msg.position.x,msg.position.y,msg.position.z],
                             [msg.velocity.x,msg.velocity.y,msg.velocity.z])
            pose[:]=state['target'][1]
            velocity[:]=[msg.velocity.x,msg.velocity.y,msg.velocity.z]
            state['peak_native_speed']=max(state['peak_native_speed'],math.hypot(*velocity))
            state['max_lateral_displacement_m']=max(state['max_lateral_displacement_m'],
                abs(-math.sin(yaw)*(pose[0]-ground[0])+math.cos(yaw)*(pose[1]-ground[1])))
    def goal(msg):
        end=[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
        state['goals'].append(end)
        if scene_file is not None:return  # The real isolated SUPER owns all planning output.
        if fault=='initial-plan-delay' and len(state['goals'])==1:
            def delayed_plan():
                node.destroy_timer(timer);build_plan(end)
            timer=node.create_timer(9.,delayed_plan)
            return
        build_plan(end)
    def build_plan(end,resuming=False):
        if fault=='intermediate-rest' and not resuming:
            state['resume_goal']=list(end)
            end=[(a+b)/2 for a,b in zip(pose,end)]
        distance=math.dist(pose,end)
        duration=max(1.875*distance/2.,math.sqrt(5.774*distance),1.)
        if fault=='long-trajectory':duration=40.  # Exercise both former total deadlines.
        if fault=='motion-target-exceedance':duration*=.9  # Synthetic evidence test only.
        if fault=='motion-advisory-exceedance':duration*=.7
        poly=PolynomialTrajectory();poly.header.frame_id='world';poly.header.stamp=node.get_clock().now().to_msg()
        poly.start_wt_pos=node.get_clock().now().nanoseconds/1e9+.1
        poly.type=3;poly.order_pos=7;poly.piece_num_pos=1;poly.time_pos=[duration]
        for axis,name in enumerate(('coef_pos_x','coef_pos_y','coef_pos_z')):
            d=end[axis]-pose[axis]
            coefficients=[0.,0.,6*d/duration**5,-15*d/duration**4,10*d/duration**3,0.,0.,pose[axis]]
            if fault=='lateral-route':
                # Smooth 20 cm left excursion, with zero endpoint velocity/acceleration.
                offset=.2*(-math.sin(yaw),math.cos(yaw),0.)[axis]
                for index,value in ((1,-64.),(2,192.),(3,-192.),(4,64.)):
                    coefficients[index]+=value*offset/duration**(7-index)
            setattr(poly,name,coefficients)
        state['plan']=Plan(poly);polypub.publish(poly)
    node.create_service(SetMode,'/mavros/set_mode',mode);node.create_service(CommandBool,'/mavros/cmd/arming',arm)
    node.create_subscription(PositionTarget,'/mavros/setpoint_raw/local',target,qos_profile_sensor_data)
    node.create_subscription(PoseStamped,'/goal_pose',goal,10)
    last=time.monotonic()
    def telemetry():
        nonlocal last
        now=time.monotonic();dt=now-last;last=now
        if state['armed'] and state['mode']=='OFFBOARD' and state['target'] is not None:
            when,position,feedforward=state['target']
            elapsed=node.get_clock().now().nanoseconds/1e9-when
            pose[:]=[p+v*elapsed for p,v in zip(position,feedforward)]
            velocity[:]=feedforward
        if state['mode']=='AUTO.LAND' and state['armed']:
            pose[2]=max(ground[2],pose[2]-.5*dt);velocity[:]=[0.,0.,-.5]
            if pose[2]==ground[2]:state['armed']=False;state['landed']=True;velocity[:]=[0.,0.,0.]
        stamp=node.get_clock().now().to_msg()
        for topic,kind in kinds.items():
            msg=kind();msg.header.stamp=stamp
            if kind is State:
                msg.connected=True;msg.armed=state['armed'];msg.mode=state['mode'];msg.manual_input=True
            elif kind is ExtendedState:msg.landed_state=1 if state['landed'] else 2
            elif kind is EstimatorStatus:
                msg.attitude_status_flag=msg.velocity_horiz_status_flag=msg.velocity_vert_status_flag=True
                msg.pos_horiz_rel_status_flag=msg.pos_vert_abs_status_flag=True
            elif kind is RCIn:msg.channels=[1500]*16
            elif kind is TwistStamped:
                msg.header.frame_id='map'
                for a,v in zip(('x','y','z'),velocity):setattr(msg.twist.linear,a,v)
            else:
                msg.header.frame_id='camera_init' if kind is Odometry else ('map' if kind is PoseStamped else 'px4_enu_preview')
                if kind is Odometry:msg.child_frame_id='body'
                p=msg.pose if kind is PoseStamped else msg.pose.pose
                values=pose;quaternion=aligned_q
                if kind is Odometry and raw_rotation is not None:
                    values=raw_rotation.T@(np.array(pose)+origin-raw_zero);quaternion=raw_q
                    for a,v in zip(('x','y','z'),raw_rotation.T@np.array(velocity)):
                        setattr(msg.twist.twist.linear,a,float(v))
                for a,v in zip(('x','y','z'),values):setattr(p.position,a,float(v))
                for a,v in zip(('x','y','z','w'),quaternion):setattr(p.orientation,a,float(v))
            pubs[topic].publish(msg)
            if kind is Odometry:
                # Visualization-only alias: SUPER world numerically matches raw camera_init.
                # Never alias the aligned ENU or MAVROS map frame to world.
                visible=PoseStamped();visible.header.stamp=stamp;visible.header.frame_id='world'
                visible.pose=msg.pose.pose;vehiclepub.publish(visible)
    def planning():
        plan=state['plan'];stamp_ns=node.get_clock().now().nanoseconds;now=stamp_ns/1e9
        if plan is None or now<plan.start:return
        if state['future_plan'] is not None and now>=state['future_plan'].start:
            plan=state['plan']=state['future_plan'];state['future_plan']=None
        if (fault=='future-replan' and state['future_replanned_goal']!=len(state['goals'])
                and now>=plan.start+plan.duration*.45):
            # Publish a continuous replacement ahead of its start while the
            # command topic continues executing the original committed plan.
            import numpy as np
            future=PolynomialTrajectory();future.header.frame_id='world'
            future.header.stamp=node.get_clock().now().to_msg()
            future.type=3;future.order_pos=7;future.piece_num_pos=1
            future.start_wt_pos=now+.03;offset=future.start_wt_pos-plan.start
            future.time_pos=[plan.duration-offset]
            for axis,name in enumerate(('coef_pos_x','coef_pos_y','coef_pos_z')):
                shifted=np.poly1d(plan.coefficients[axis,0])(np.poly1d([1.,offset])).c
                setattr(future,name,np.pad(shifted,(8-len(shifted),0)).tolist())
            state['future_plan']=Plan(future);state['future_replans']+=1
            state['future_replanned_goal']=len(state['goals']);polypub.publish(future)
        if state['resume_goal'] is not None:
            if now>=plan.end+5.:
                goal=state['resume_goal'];state['resume_goal']=None;state['rest_waits']+=1
                build_plan(goal,resuming=True);return
            if now>plan.end+2.:return  # Fresh rest samples, then a silent planning wait.
        if fault=='return-dropout' and len(state['goals'])==2 and now>plan.start+1.:return
        p,v,a=plan.evaluate(now-plan.start)
        msg=PositionCommand();msg.header.frame_id='world'
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp_ns,1_000_000_000)
        msg.trajectory_flag=1;msg.yaw=yaw;msg.attitude.z=yaw
        for field,values in (('position',p),('velocity',v),('acceleration',a)):
            for axis,value in zip(('x','y','z'),values):setattr(getattr(msg,field),axis,float(value))
        cmdpub.publish(msg)
    node.create_timer(.02,telemetry)
    if scene_file is None:node.create_timer(.01,planning)
    connection.send('ready')
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=.005)
            if connection.poll():
                request=connection.recv()
                if request=='stop':break
                if request=='report':connection.send({k:state[k] for k in
                    ('goals','commands','peak_native_speed','max_lateral_displacement_m','armed','landed','rest_waits','future_replans')})
    finally:
        executor.shutdown();node.destroy_node();frame.destroy_node();rclpy.shutdown()


def run(mission_file,report,fault=None,scene_file=None,motion_policy='strict',rviz=True):
    import rclpy
    from std_srvs.srv import Trigger
    from super_demo_control import DemoMissionControl
    parent,child=multiprocessing.get_context('spawn').Pipe()
    process=multiprocessing.get_context('spawn').Process(target=backend,args=(child,fault,scene_file))
    node=None;planner_process=None;planner_log=None;viewer=RehearsalViewer(report,rviz);process.start()
    try:
        if not parent.poll(15) or parent.recv()!='ready':raise RuntimeError('Synthetic ROS backend startup failed')
        with tempfile.TemporaryDirectory(prefix='demo_simulation_') as directory:
            alignment=Path(directory)/'alignment.py';alignment.write_text('def convert(p,q,c):\n return p,q,c\n')
            planner=Path(directory)/'planner.json'
            planner.write_text(json.dumps(dict(fsm=dict(click_height=-6.,click_goal_topic='/goal_pose',
                click_goal_en=True,cmd_topic='/planning/pos_cmd'),traj_opt=dict(boundary=dict(max_vel=2.,max_acc=1.)))))
            scene=None
            if scene_file is not None:
                scene=json.loads(Path(scene_file).read_text());alignment=Path(scene['alignment_path'])
                if hashlib.sha256(alignment.read_bytes()).hexdigest()!=scene['alignment_sha256']:
                    raise ValueError('Frozen scene alignment changed')
                if hashlib.sha256(Path(scene['planner_config']).read_bytes()).hexdigest()!=scene['planner_sha256']:
                    raise ValueError('Source planner configuration changed after scene capture')
                planner=rehearsal_profile(scene['planner_config'],report)
                Path(report).parent.mkdir(parents=True,exist_ok=True)
                planner_log=(Path(report).parent/'planner_rehearsal.log').open('w')
                planner_process=subprocess.Popen(['ros2','run','super_planner','fsm_node','--ros-args',
                    '-p','use_sim_time:=false','-p','config_name:='+planner.name],
                    stdout=planner_log,stderr=subprocess.STDOUT,start_new_session=True)
            viewer.start(scene)
            rclpy.init();node=DemoMissionControl(mission_file,report,True,True,alignment,planner,
                                              simulation=True,motion_policy=motion_policy)
            client=node.create_client(Trigger,'/super_demo/start');attempted=False
            while rclpy.ok() and node.phase not in ('DONE','STOPPED'):
                rclpy.spin_once(node,timeout_sec=.02)
                if planner_process is not None and planner_process.poll() is not None:
                    node.abort('Isolated SUPER exited; inspect planner_rehearsal.log')
                if node.phase=='READY' and not attempted and client.service_is_ready():
                    attempted=True;client.call_async(Trigger.Request())
            parent.send('report')
            if not parent.poll(5):raise RuntimeError('Synthetic backend ledger unavailable')
            ledger=parent.recv()
            node.mission_report.data['simulation_backend']=dict(kind='real SUPER / ideal virtual vehicle' if scene_file else
                'ideal synthetic ROS planning and vehicle',actual_SUPER_used=scene_file is not None,
                actual_PX4_used=False,hardware_used=False,ledger=ledger,scene_file=str(scene_file) if scene_file else None,
                execution_timeout_s=None,
                static_snapshot_adaptation=['rog_map.ros_callback.cloud_topic','rog_map.raycasting.enable'] if scene_file else [])
            node.mission_report.save()
            return node.phase=='DONE'
    finally:
        viewer.close()
        if node is not None:
            node.mission_report.data['rehearsal_viewer']=viewer.state
            node.mission_report.save()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process.is_alive():parent.send('stop')
        process.join(timeout=5)
        if process.is_alive():process.terminate();process.join()
        parent.close();child.close()
        if planner_process is not None and planner_process.poll() is None:
            os.killpg(planner_process.pid,signal.SIGINT)
            try:planner_process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(planner_process.pid,signal.SIGTERM);planner_process.wait(timeout=5)
        if planner_log is not None:planner_log.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mission-file',type=Path,required=True)
    p.add_argument('--fixture-geometry',action='store_true',help='use labelled synthetic geometry, not actual aircraft measurements')
    p.add_argument('--report-json',type=Path)
    p.add_argument('--fault',choices=('return-dropout','motion-target-exceedance','motion-advisory-exceedance',
                                     'initial-plan-delay','lateral-route','long-trajectory','intermediate-rest','future-replan'))
    p.add_argument('--scene-file',type=Path,help='captured actual map; use real SUPER instead of the synthetic planner')
    p.add_argument('--motion-policy',choices=('strict','observe'),default='strict')
    p.add_argument('--no-rviz',action='store_true',help='disable the automatically managed rehearsal viewer')
    p.add_argument('--isolated',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args()
    if args.scene_file and args.fault:
        raise SystemExit('Synthetic planner fault injection cannot be used with an actual SUPER scene')
    if not args.isolated:
        import super_session as manager
        env=dict(os.environ,ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',OPENBLAS_NUM_THREADS='1')
        return subprocess.run(['bash','--noprofile','--norc','-c',manager.source_prefix()+'exec '+
            manager.shell_command([sys.executable,Path(__file__).resolve(),'--isolated',*sys.argv[1:]])],env=env).returncode
    import rclpy
    if os.environ.get('ROS_DOMAIN_ID')!='197' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Synthetic backend requires isolated ROS domain 197 and localhost-only transport')
    rclpy.init();probe=rclpy.create_node('demo_isolation_check');began=time.monotonic()
    while time.monotonic()-began<2:rclpy.spin_once(probe,timeout_sec=.05)
    others=[n for n,_ in probe.get_node_names_and_namespaces() if n!='demo_isolation_check']
    probe.destroy_node();rclpy.shutdown()
    if others:raise SystemExit('Simulation domain is already in use; refusing: '+str(others))
    from demo_mission import default_report_path
    report=args.report_json or default_report_path()
    with tempfile.TemporaryDirectory(prefix='demo_geometry_') as folder:
        mission_file=args.mission_file
        if args.fixture_geometry:
            config=json.loads(mission_file.read_text())
            config['takeoff'].update(forward_from_rear_m=.5,distance_from_left_m=1.4)
            for side in ('front','rear','left','right'):config['vehicle']['centre_to_'+side+'_m']=.2
            config['vehicle'].update(centre_to_top_m=.1,centre_to_bottom_m=.085)
            config['status']='SYNTHETIC_TEST_FIXTURE_NOT_AIRCRAFT_GEOMETRY'
            mission_file=Path(folder)/'fixture.json';mission_file.write_text(json.dumps(config))
        print('REHEARSAL ONLY: real SUPER with frozen map' if args.scene_file else
              'SIMULATION ONLY: ideal synthetic planner',flush=True)
        print('Virtual vehicle/FC only; no physical aircraft or PX4 flight',flush=True)
        return 0 if run(mission_file,report,args.fault,args.scene_file,args.motion_policy,not args.no_rviz) else 1


if __name__=='__main__':raise SystemExit(main())
