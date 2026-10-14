#!/usr/bin/env python3
"""Manage the proven aircraft stack in a private tmux session.

start -> align -> vision QX QY QZ QW -> planner / hover [--execute].
No automatic flight initiation, arming, mode changes, or node restart.
Mutable actions require fresh connected/disarmed/on-ground FC telemetry.
"""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

SOCKET = 'super-aircraft'
DEFAULT_FCU = 'serial:///dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00:2000000'
SETUPS = ('/opt/ros/humble/setup.bash', '~/ws_livox/install/setup.bash',
          '~/fastlio_ws/install/setup.bash', '~/super_ws/install/setup.bash')
CORE = ('lidar', 'lio', 'mavros', 'body')
SCRIPTS = ('fastlio_body_debug.py', 'odom_age_check.py', 'frame_alignment_check.py',
           'update_preview_alignment.py', 'aligned_pose_preview.py',
           'flight_pose_cov_transport.py', 'first_hover_terminal.py',
           'control_bridge_preview.py')
NATIVE_CONTROL_TOPICS=('/mavros/setpoint_position/local','/mavros/setpoint_raw/local',
    '/mavros/setpoint_raw/global','/mavros/setpoint_raw/attitude','/mavros/setpoint_position/global',
    '/mavros/setpoint_position/global_to_local','/mavros/setpoint_velocity/cmd_vel',
    '/mavros/setpoint_velocity/cmd_vel_unstamped','/mavros/setpoint_accel/accel',
    '/mavros/setpoint_attitude/attitude','/mavros/setpoint_attitude/cmd_vel','/mavros/setpoint_attitude/thrust')


def ros_environment():
    return {name: os.environ.get(name, default) for name, default in
            (('ROS_DOMAIN_ID', '0'), ('ROS_LOCALHOST_ONLY', '0'), ('RMW_IMPLEMENTATION', ''))}


def shell_command(argv):
    return shlex.join([str(value) for value in argv])


def source_prefix():
    return 'set -e; ' + '; '.join('source ' + shlex.quote(str(Path(p).expanduser())) for p in SETUPS) + '; '


def run(argv, **kwargs):
    return subprocess.run([str(v) for v in argv], check=True, **kwargs)


class GroundGate:
    """Read-only telemetry plus bounded parameter calls; no mode/arming client."""
    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from mavros_msgs.msg import State, ExtendedState, EstimatorStatus
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from rcl_interfaces.srv import GetParameters, SetParameters
        from mavros_msgs.srv import ParamPull
        self.ros = rclpy
        self.GetParameters, self.SetParameters, self.ParamPull = GetParameters, SetParameters, ParamPull
        self.ON_GROUND = ExtendedState.LANDED_STATE_ON_GROUND
        rclpy.init()
        self.node = rclpy.create_node('super_session_check')
        self.messages, self.receipts = {}, {}
        for name, kind, topic in (
            ('state', State, '/mavros/state'), ('ground', ExtendedState, '/mavros/extended_state'),
            ('local', PoseStamped, '/mavros/local_position/pose'),
            ('estimator', EstimatorStatus, '/mavros/estimator_status'),
            ('preview', PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov'),
            ('vision', PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov')):
            self.node.create_subscription(kind, topic, lambda m, key=name: self.store(key, m),
                                          qos_profile_sensor_data)
        try:
            self.wait(self.grounded, 'fresh connected/disarmed/on-ground FC state')
        except Exception:
            self.close()
            raise

    def store(self, name, msg):
        self.messages[name], self.receipts[name] = msg, time.monotonic()

    def fresh(self, name, limit=2.):
        if name not in self.messages or time.monotonic()-self.receipts[name] > limit:
            return False
        stamp = self.messages[name].header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds-ns)/1e9
        return ns > 0 and -.05 <= age <= limit

    def grounded(self):
        if not self.fresh('state') or not self.fresh('ground'):
            return False
        state = self.messages['state']
        return state.connected and not state.armed and self.messages['ground'].landed_state == self.ON_GROUND

    def wait(self, predicate, description, timeout=12.):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError('Timed out waiting for '+description)
            self.ros.spin_once(self.node, timeout_sec=.05)

    def check(self):
        self.ros.spin_once(self.node, timeout_sec=0)
        if not self.grounded():
            raise RuntimeError('FC state is stale, disconnected, armed, or not on ground; action blocked')

    def call(self, kind, topic, request):
        client = self.node.create_client(kind, topic)
        try:
            self.wait(client.service_is_ready, topic)
            self.check()
            future = client.call_async(request)
            self.wait(future.done, topic+' response')
            self.check()
            result = future.result()
            if result is None:
                raise RuntimeError('Empty service response: '+topic)
            return result
        finally:
            self.node.destroy_client(client)

    def pull(self):
        if not self.call(self.ParamPull, '/mavros/param/pull',
                         self.ParamPull.Request(force_pull=True)).success:
            raise RuntimeError('MAVROS parameter pull failed')

    def parameters(self, names):
        from rclpy.parameter import Parameter
        from rcl_interfaces.msg import Parameter as ParameterMsg
        values = self.call(self.GetParameters, '/mavros/param/get_parameters',
                           self.GetParameters.Request(names=names)).values
        if len(values) != len(names):
            raise RuntimeError('Incomplete parameter response')
        return {name: Parameter.from_parameter_msg(ParameterMsg(name=name, value=value)).value for name, value in zip(names, values)}

    def set_ctrl(self, value):
        from rclpy.parameter import Parameter
        response = self.call(self.SetParameters, '/mavros/param/set_parameters',
            self.SetParameters.Request(parameters=[Parameter('EKF2_EV_CTRL', value=value).to_parameter_msg()]))
        if len(response.results) != 1 or not response.results[0].successful:
            raise RuntimeError('EV_CTRL write rejected')
        if self.parameters(['EKF2_EV_CTRL'])['EKF2_EV_CTRL'] != value:
            raise RuntimeError('EV_CTRL readback mismatch')

    def pose_ready(self, name):
        if not self.fresh(name, .25):
            return False
        return self.messages[name].header.frame_id == 'px4_enu_preview'

    def localization_ready(self):
        if not all(self.fresh(name, limit) for name, limit in
                   (('local', .25), ('preview', .25), ('vision', .25), ('estimator', 2.))):
            return False
        status = self.messages['estimator']
        valid = (status.attitude_status_flag and status.velocity_horiz_status_flag
                 and status.velocity_vert_status_flag and status.pos_horiz_rel_status_flag
                 and status.pos_vert_abs_status_flag and not status.accel_error_status_flag)
        return (valid and self.messages['local'].header.frame_id == 'map'
                and self.pose_ready('preview') and self.pose_ready('vision'))

    def flight_check(self):
        """Sample prerequisites without creating a control publisher/client."""
        from first_hover_terminal import PARAMETERS
        from geometry_msgs.msg import PoseStamped
        from ground_setpoint_transport import pose_values
        from mavros_msgs.msg import RCIn
        from rclpy.qos import qos_profile_sensor_data
        rc_subscription=self.node.create_subscription(RCIn,'/mavros/rc/in',
            lambda msg:self.store('rc',msg),qos_profile_sensor_data)
        report={'scope':'Read-only grounded software prerequisites; no flight requests',
                'parameters':{},'errors':[]}
        try:
            end=time.monotonic()+1.5
            while time.monotonic()<end:self.ros.spin_once(self.node,timeout_sec=.05)
            try:
                actual=self.parameters(list(PARAMETERS))
                report['parameters']={name:{'actual':actual[name],'expected':expected,
                    'matches':type(actual[name]) is type(expected) and actual[name]==expected}
                    for name,expected in PARAMETERS.items()}
            except RuntimeError as exc:report['errors'].append(str(exc))
            frame=None
            try:
                values=self.call(self.GetParameters,'/mavros/setpoint_position/get_parameters',
                    self.GetParameters.Request(names=['mav_frame'])).values
                if len(values)==1 and values[0].type==4:frame=values[0].string_value
            except RuntimeError as exc:report['errors'].append(str(exc))
            self.check()
            state=self.messages['state'];rc=self.messages.get('rc')
            report['state']={'connected':state.connected,'armed':state.armed,
                'mode':state.mode,'manual_input':state.manual_input}
            report['rc']={'fresh':self.fresh('rc',.5),
                'channel_count':len(rc.channels) if rc is not None else 0}
            report['mav_frame']=frame
            report['native_publishers']={topic:self.node.count_publishers(topic)
                for topic in NATIVE_CONTROL_TOPICS if self.node.count_publishers(topic)}
            pose_agreement=False
            if self.localization_ready():
                try:
                    local,yaw=pose_values(self.messages['local'])
                    external,ev_yaw=pose_values(PoseStamped(pose=self.messages['vision'].pose.pose))
                    heading=abs((yaw-ev_yaw+math.pi)%(2*math.pi)-math.pi)
                    distance=math.dist(local,external)
                    pose_agreement=distance<=.15 and heading<=math.radians(5)
                    report['pose_agreement']={'distance_m':distance,'yaw_difference_deg':math.degrees(heading)}
                except ValueError as exc:report['errors'].append(str(exc))
            report['checks']={
                'connected_disarmed_on_ground':self.grounded(),
                'fresh_valid_localization':self.localization_ready(),
                'fc_vision_pose_agreement':pose_agreement,
                'required_parameters':len(report['parameters'])==len(PARAMETERS) and
                    all(value['matches'] for value in report['parameters'].values()),
                'local_ned':frame=='LOCAL_NED',
                'posctl_with_manual_input':state.mode=='POSCTL' and state.manual_input,
                'fresh_rc_with_seven_channels':report['rc']['fresh'] and report['rc']['channel_count']>=7,
                'no_native_control_publishers':not report['native_publishers'],
                'single_localization_sources':all(self.node.count_publishers(topic)==1 for topic in
                    ('/Odometry','/super_debug/aligned_pose_cov','/mavros/vision_pose/pose_cov','/mavros/local_position/pose')),
                'single_rc_source':self.node.count_publishers('/mavros/rc/in')==1}
            return report
        finally:self.node.destroy_subscription(rc_subscription)

    def close(self):
        self.node.destroy_node()
        self.ros.shutdown()


class Session:
    def __init__(self, args):
        self.args = args
        self.name = args.session
        self.folder = Path.home()/'.local/state/super-session'/self.name
        self.metadata = self.folder/'session.json'
        self.scripts = args.scripts_dir.expanduser().resolve()
        self.config = None

    def tmux(self, *args, capture=False):
        return run(['tmux', '-L', SOCKET, *args], text=True,
                   stdout=subprocess.PIPE if capture else None,
                   stderr=subprocess.PIPE if capture else None)

    def exists(self):
        return subprocess.run(['tmux', '-L', SOCKET, 'has-session', '-t', self.name],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0

    def load(self):
        if not self.exists() or not self.metadata.is_file():
            raise RuntimeError('No managed session. Run start first.')
        self.config = json.loads(self.metadata.read_text())
        if self.config['environment'] != ros_environment() or self.config['scripts'] != str(self.scripts):
            raise RuntimeError('Session ROS environment/script directory differs; do not split the ROS graph')

    def windows(self):
        output = self.tmux('list-panes', '-s', '-t', self.name, '-F', '#{window_name}:#{pane_dead}', capture=True).stdout
        return dict(line.rsplit(':', 1) for line in output.splitlines())

    def require_core(self):
        windows = self.windows()
        for name in CORE:
            if windows.get(name) != '0':
                raise RuntimeError(f'{name} is not running. Inspect logs; nodes are never auto-restarted.')

    def save(self):
        self.folder.mkdir(parents=True, exist_ok=True)
        self.metadata.write_text(json.dumps(self.config, indent=2)+'\n')

    def launch(self, name, command, first=False):
        if not first and name in self.windows():
            raise RuntimeError(f'Window {name} already exists; no duplicate/restart')
        if first:
            pane = self.tmux('new-session', '-d', '-s', self.name, '-n', name,
                             '-P', '-F', '#{pane_id}', 'bash --noprofile --norc', capture=True).stdout.strip()
            self.tmux('set-window-option', '-g', 'remain-on-exit', 'on')
        else:
            pane = self.tmux('new-window', '-t', self.name, '-n', name,
                             '-P', '-F', '#{pane_id}', 'bash --noprofile --norc', capture=True).stdout.strip()
        logfile = Path(self.config['logs'])/(name+'.log')
        self.tmux('pipe-pane', '-o', '-t', pane, 'cat >> '+shlex.quote(str(logfile)))
        environment = self.config['environment']
        export = '; '.join('export '+key+'='+shlex.quote(value) for key, value in environment.items() if value)
        body = source_prefix()+export+'; exec '+command
        self.tmux('send-keys', '-t', pane, '-l', shell_command(['exec', 'bash', '--noprofile', '--norc', '-c', body]))
        self.tmux('send-keys', '-t', pane, 'Enter')
        print(f'{name}: {logfile}')

    def core_commands(self):
        lio_override = ' -p mapping.extrinsic_est_en:=false' if self.args.fixed_lio_extrinsics else ''
        return {
            'lidar': shell_command(['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py']),
            'lio': 'ros2 run fast_lio fastlio_mapping --ros-args --params-file '
                   '"$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" '
                   '-p use_sim_time:=false -p publish.map_en:=false -p publish.path_en:=false '
                   '-p publish.scan_publish_en:=true -p publish.scan_bodyframe_pub_en:=false'+lio_override,
            'mavros': shell_command(['ros2', 'launch', 'mavros', 'px4.launch',
                       'fcu_url:='+self.args.fcu_url, 'gcs_url:=udp://127.0.0.1:14555@127.0.0.1:14550']),
            'body': shell_command(['python3', '-u', self.scripts/'fastlio_body_debug.py']),
        }

    def start(self):
        if self.exists():
            self.load()
            if self.args.fixed_lio_extrinsics and not self.config.get('fixed_lio_extrinsics',False):
                raise RuntimeError('Existing session has no fixed-LIO override. Stop it while grounded/disarmed before starting the comparison.')
            self.status()
            return
        for path in [Path(p).expanduser() for p in SETUPS]+[self.scripts/name for name in SCRIPTS]:
            if not path.is_file():
                raise RuntimeError('Missing prerequisite: '+str(path))
        existing = unmanaged_processes()
        if existing:
            raise RuntimeError('Existing stack processes detected: '+', '.join(existing)+
                               '. Stop manual nodes only after landing/disarming; no duplicate launch.')
        logdir = self.scripts/'super_session_logs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        logdir.mkdir(parents=True)
        self.config = dict(environment=ros_environment(), scripts=str(self.scripts), logs=str(logdir), hover_execute=False,
                           fixed_lio_extrinsics=self.args.fixed_lio_extrinsics)
        self.save()
        for index, (name, command) in enumerate(self.core_commands().items()):
            self.launch(name, command, first=index == 0)
        print('Core started. Keep stationary; run align in the command terminal. No vision/control was started.')
        if self.args.fixed_lio_extrinsics:
            print('LIO comparison startup override: mapping.extrinsic_est_en=false. YAML unchanged; verify the live parameter.')

    def status(self):
        self.load()
        for name, dead in self.windows().items():
            print(f'{name}: '+('EXITED; inspect log' if dead == '1' else 'process active'))
        print('Logs:', self.config['logs'])
        if self.config.get('fixed_lio_extrinsics',False):
            print('LIO comparison startup override: mapping.extrinsic_est_en=false (verify the live parameter).')
        print('A process-active status does not establish ROS/FC readiness.')

    def interrupt(self, name):
        if self.windows().get(name) == '1':
            return
        self.tmux('send-keys', '-t', self.name+':'+name, 'C-c')
        deadline = time.monotonic()+10
        while self.windows().get(name) != '1':
            if time.monotonic() >= deadline:
                raise RuntimeError(name+' did not stop after Ctrl+C; inspect logs')
            time.sleep(.05)

    def planner_profile(self):
        import yaml
        folder = Path.home()/'super_ws/src/SUPER/super_planner/config'
        diagnostic = folder/'mid360_ground_diagnostic_ros2.yaml'
        if not diagnostic.is_file():
            helper = self.scripts/'create_super_ground_diagnostic.py'
            if not helper.is_file():
                raise RuntimeError('Missing helper: '+str(helper))
            run(['python3', helper])
        target = diagnostic
        motion_profiles = {
            'low-speed': ('mid360_low_speed_preview_ros2.yaml', .3, .5),
            'low-speed-margin': ('mid360_low_speed_margin_ros2.yaml', .3, .45),
            'low-speed-reserved': ('mid360_low_speed_reserved_ros2.yaml', .25, .45),
            'low-speed-3d': ('mid360_low_speed_3d_goal_ros2.yaml', .25, .45),
        }
        expected = motion_profiles.get(self.args.planner_profile)
        if expected is not None:
            filename, expected_velocity, expected_acceleration = expected
            target = folder/filename
            if not target.is_file():
                if self.args.planner_profile == 'low-speed-3d':
                    source = folder/'mid360_low_speed_reserved_ros2.yaml'
                    if not source.is_file():
                        helper = self.scripts/'create_super_motion_profiles.py'
                        if not helper.is_file():
                            raise RuntimeError('Missing helper: '+str(helper))
                        run(['python3',helper,'--source',diagnostic])
                    helper = self.scripts/'create_super_3d_goal_profile.py'
                    if not helper.is_file():
                        raise RuntimeError('Missing helper: '+str(helper))
                    run(['python3',helper,'--source',source])
                else:
                    helper = self.scripts/'create_super_motion_profiles.py'
                    if not helper.is_file():
                        raise RuntimeError('Missing helper: '+str(helper))
                    run(['python3', helper, '--source', diagnostic])
        data = yaml.safe_load(target.read_text())
        if not isinstance(data, dict):
            raise RuntimeError('Invalid diagnostic YAML')
        fsm = data.get('fsm', {})
        inputs = data.get('rog_map', {}).get('ros_callback', {})
        required = {'click_goal_en':True, 'click_goal_topic':'/goal_pose',
                    'cmd_topic':'/planning/pos_cmd', 'timer_en':True}
        if any(fsm.get(name) != value for name, value in required.items()) or not inputs.get('enable'):
            raise RuntimeError('Unexpected diagnostic goal/command/map configuration')
        if inputs.get('cloud_topic') != '/cloud_registered' or inputs.get('odom_topic') != '/Odometry':
            raise RuntimeError('Diagnostic configuration does not use the active LIO topics')
        if expected is not None:
            limits = data.get('traj_opt', {}).get('boundary', {})
            if limits.get('max_vel') != expected_velocity or limits.get('max_acc') != expected_acceleration:
                raise RuntimeError(f'Low-speed preview requires max_vel={expected_velocity} and max_acc={expected_acceleration}')
        if self.args.planner_profile == 'low-speed-3d' and fsm.get('click_height') != -6.0:
            raise RuntimeError('3D goal preview requires click_height=-6 to preserve message Z')
        (folder.parent/'log').mkdir(parents=True, exist_ok=True)
        return target

    def stop_planning(self, gate):
        for name in ('wire_test', 'handover_check', 'reference_preview', 'cmd_preview', 'planner'):
            if name in self.windows():
                gate.check()
                self.interrupt(name)
                gate.wait(gate.grounded, 'fresh ground state during planning shutdown')
                self.tmux('kill-window', '-t', self.name+':'+name)
        print('Ground planning stopped; localization and vision remain running.')

    def mutate(self, action):
        self.load()
        if action not in ('stop', 'planner-stop', 'wire-stop', 'handover-stop'):
            self.require_core()
        windows = self.windows()
        gate = GroundGate()
        try:
            if action == 'flight-check':
                report=gate.flight_check()
                required=(*CORE,'preview','vision','planner','cmd_preview')
                report['checks']['managed_processes_active']=all(windows.get(name)=='0' for name in required)
                report['checks']['reserved_3d_planner']=False
                try:
                    import yaml
                    profile=yaml.safe_load(Path(self.config['planner_config_path']).read_text())
                    boundary=profile['traj_opt']['boundary'];fsm=profile['fsm']
                    report['checks']['reserved_3d_planner']=(self.config.get('planner_profile')=='low-speed-3d' and
                        boundary['max_vel']==.25 and boundary['max_acc']==.45 and
                        fsm['click_height']==-6. and fsm['click_goal_topic']=='/goal_pose' and
                        fsm['cmd_topic']=='/planning/pos_cmd' and fsm['click_goal_en'])
                except (OSError,KeyError,TypeError,ValueError) as exc:
                    report['errors'].append('Planner configuration: '+str(exc))
                report['blockers']=[name for name,passed in report['checks'].items() if not passed]
                report['sampled_software_prerequisites_passed']=not report['blockers']
                site_inputs={'posctl_with_manual_input','fresh_rc_with_seven_channels','single_rc_source'}
                report['remote_preparation_checks_passed']=all(passed for name,passed in
                    report['checks'].items() if name not in site_inputs)
                report['on_site_supervision_and_clearance']='Not assessed by this read-only command'
                print(json.dumps(report,indent=2))
            elif action == 'align':
                if any(name in windows for name in ('preview', 'vision', 'hover')):
                    raise RuntimeError('Alignment requires a fresh core-only session; do not rebase a running relay')
                gate.pull()
                gate.set_ctrl(0)
                run(['python3', '-u', self.scripts/'odom_age_check.py'])
                run(['python3', '-u', self.scripts/'frame_alignment_check.py'])
                print('Use this survey: super_session.py vision QX QY QZ QW. No quaternion was auto-saved.')
            elif action == 'vision':
                if any(name in windows for name in ('preview', 'vision', 'hover')):
                    raise RuntimeError('Preview/vision/control already exists; no session restart')
                names = ['EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z', 'EKF2_EV_QMIN']
                expected = [0, 3, 0., 0., 0., 0]
                values = gate.parameters(names)
                if any(type(values[name]) is not type(value) or values[name] != value for name, value in zip(names, expected)):
                    raise RuntimeError('Need EV_CTRL=0, HGT_REF=3, EV_POS=0, QMIN=0 before session initialization')
                if any(gate.node.count_publishers(topic) for topic in
                       ('/mavros/vision_pose/pose_cov', '/super_debug/aligned_pose_cov')):
                    raise RuntimeError('Another aligned/native vision publisher is present')
                gate.check()
                run(['python3', self.scripts/'update_preview_alignment.py',
                     *[str(v) for v in self.args.quaternion], '--preview', self.scripts/'aligned_pose_preview.py'])
                gate.wait(gate.grounded, 'fresh ground state after alignment update')
                preview_started = time.monotonic()
                self.launch('preview', shell_command(['python3', '-u', self.scripts/'aligned_pose_preview.py']))
                gate.wait(lambda: gate.pose_ready('preview') and gate.receipts['preview'] >= preview_started,
                          'fresh aligned preview')
                gate.set_ctrl(11)
                vision_started = time.monotonic()
                self.launch('vision', shell_command(['python3', '-u', self.scripts/'flight_pose_cov_transport.py',
                                                    '--ros-args', '-p', 'publish_to_mavros:=true',
                                                    '-p', 'alignment_path:='+str(self.scripts/'aligned_pose_preview.py')]))
                gate.wait(lambda: gate.pose_ready('vision') and gate.receipts['vision'] >= vision_started,
                          'fresh native vision')
                print('Vision session started. Check current estimator status and POSCTL preflight.')
            elif action in ('planner', 'goal', 'planner-check', 'reference-preview'):
                if windows.get('preview') != '0' or windows.get('vision') != '0':
                    raise RuntimeError('Need the managed localization/vision session running')
                if 'hover' in windows:
                    raise RuntimeError('Hover window exists; keep planning and hover tasks separate')
                gate.wait(gate.localization_ready, 'fresh valid local/vision/estimator state')
                gate.check()
                if action == 'reference-preview':
                    if windows.get('planner') != '0' or windows.get('cmd_preview') != '0':
                        raise RuntimeError('Start the ground planner and command preview first')
                    if 'reference_preview' in windows or gate.node.count_publishers('/super_debug/bounded_control_target'):
                        raise RuntimeError('A bounded reference preview already exists; no duplicate/restart')
                    from mars_quadrotor_msgs.msg import PolynomialTrajectory
                    PolynomialTrajectory()
                    helper = self.scripts/'bounded_reference_preview.py'
                    if not helper.is_file():
                        raise RuntimeError('Missing ground helper: '+str(helper))
                    gate.check()
                    self.launch('reference_preview',shell_command(['python3','-u',helper,
                        '--ros-args','-p','alignment_path:='+str(self.scripts/'aligned_pose_preview.py')]))
                    print('Bounded ground reference preview launched. Wait for GROUND FRAME LOCKED before one goal. No native output.')
                elif action in ('goal', 'planner-check'):
                    if action=='goal' and 'handover_check' in windows:
                        raise RuntimeError('Handover auditor owns one goal; use /super_flight/audit or handover-stop')
                    if windows.get('planner') != '0' or windows.get('cmd_preview') != '0':
                        raise RuntimeError('Start the ground planner and command preview first')
                    height_goal = action == 'goal' and self.args.goal_height is not None
                    if height_goal and (self.config.get('planner_profile') != 'low-speed-3d'
                                        or windows.get('reference_preview') != '0'):
                        raise RuntimeError('Height goal requires a managed low-speed-3d planner and fresh reference preview')
                    helper = self.scripts/('send_ground_height_goal.py' if height_goal else
                        'send_control_preview_goal.py' if action == 'goal' else 'control_chain_age_check.py')
                    if not helper.is_file():
                        raise RuntimeError('Missing ground helper: '+str(helper))
                    command = ['python3', '-u', helper]
                    if action == 'planner-check':
                        command += ['--check-motion', '--max-speed', '0.3', '--max-acceleration', '0.5']
                    if height_goal:
                        command += ['--height',str(self.args.goal_height),
                                    '--alignment-path',self.scripts/'aligned_pose_preview.py',
                                    '--planner-config',self.config['planner_config_path']]
                    run(command)
                    if action == 'goal':
                        print('Ground diagnostic goal sent. Inspect accepted goal and preview; no flight setpoints.')
                else:
                    if any(name in windows for name in ('planner', 'cmd_preview')):
                        raise RuntimeError('Planning windows already exist; no duplicate/restart')
                    if any(gate.node.count_publishers(topic) for topic in
                           ('/planning/pos_cmd', '/super_debug/control_target')):
                        raise RuntimeError('Another planner/command-preview publisher is present')
                    if any(gate.node.count_publishers(topic) for topic in
                           ('/mavros/setpoint_position/local', '/mavros/setpoint_raw/local')):
                        raise RuntimeError('An existing native setpoint sender is present')
                    from mars_quadrotor_msgs.msg import PositionCommand
                    PositionCommand()  # Verify generated message support before launching.
                    config = self.planner_profile()
                    self.config['planner_profile']=self.args.planner_profile
                    self.config['planner_config_path']=str(config)
                    self.save()
                    gate.check()
                    print('Ground planner profile:', config)
                    self.launch('planner', shell_command(['ros2', 'run', 'super_planner', 'fsm_node',
                        '--ros-args', '-p', 'use_sim_time:=false', '-p', 'config_name:='+config.name]))
                    gate.check()
                    self.launch('cmd_preview', shell_command(['python3', '-u', self.scripts/'control_bridge_preview.py',
                        '--ros-args', '-p', 'alignment_path:='+str(self.scripts/'aligned_pose_preview.py')]))
                    print('Ground planner and coordinate preview launched. No native setpoints, mode or arming requests.')
                    print('Wait for map/preview readiness, then use goal for a disarmed diagnostic target.')
            elif action == 'wire-test':
                if any(windows.get(name)!='0' for name in ('preview','vision','planner','cmd_preview')):
                    raise RuntimeError('Need managed localization, vision, ground planner and coordinate preview running')
                if any(name in windows for name in ('hover','wire_test','handover_check')):
                    raise RuntimeError('Hover/wire window exists; stop it explicitly before a ground wire test')
                gate.wait(gate.localization_ready,'fresh valid ground localization before native wire test')
                def coordinate_source_ready():
                    count=gate.node.count_publishers('/super_debug/control_target')
                    if count>1:
                        raise RuntimeError(f'Need one SUPER coordinate-preview source; discovered {count}')
                    return count==1
                gate.wait(coordinate_source_ready,
                          'one SUPER coordinate-preview source to be discovered (zero found)')
                if any(gate.node.count_publishers(topic) for topic in NATIVE_CONTROL_TOPICS):
                    raise RuntimeError('Another native setpoint publisher is present; no wire sender launched')
                helper=self.scripts/'ground_setpoint_transport.py'
                if not helper.is_file():raise RuntimeError('Missing ground wire helper: '+str(helper))
                gate.check()
                self.launch('wire_test',shell_command(['python3','-u',helper]))
                print('Ground wire sender launched: SUPER coordinate preview -> MAVROS position/yaw, 20 Hz.')
                print('Disarmed AUTO.LOITER/POSCTL only; arming/OFFBOARD latches output off. No mode or arming requests.')
                print('This checks transport; bounded-reference/flight tracking approval remains separate.')
            elif action=='handover-check':
                if self.config.get('planner_profile')!='low-speed-3d' or any(
                        windows.get(name)!='0' for name in ('preview','vision','planner','cmd_preview')):
                    raise RuntimeError('Need the managed low-speed-3d planner and current vision session')
                if any(name in windows for name in ('hover','reference_preview','handover_check')):
                    raise RuntimeError('Another hover/reference/handover window exists; stop it first')
                helper=self.scripts/'super_flight_control.py'
                if not helper.is_file() or not (self.scripts/'bounded_reference_preview.py').is_file():
                    raise RuntimeError('Missing handover controller or bounded reference helper')
                import ast
                tree=ast.parse((self.scripts/'first_hover_terminal.py').read_text())
                classes=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FirstHoverControl']
                if len(classes)!=1 or not {'mission_tick','allowed_modes','check_envelope'}<={
                        n.name for n in classes[0].body if isinstance(n,ast.FunctionDef)}:
                    raise RuntimeError('Update first_hover_terminal.py before handover-check')
                gate.wait(gate.localization_ready,'fresh valid ground localization before handover audit')
                if 'wire_test' in windows:
                    gate.check();self.interrupt('wire_test')
                    gate.wait(gate.grounded,'fresh ground state during wire shutdown')
                    self.tmux('kill-window','-t',self.name+':wire_test')
                    gate.wait(lambda:gate.node.count_publishers('/mavros/setpoint_position/local')==0,
                              'stopped ground sender to disappear from ROS discovery')
                if any(gate.node.count_publishers(topic) for topic in NATIVE_CONTROL_TOPICS):
                    raise RuntimeError('Stop other native control publishers before handover audit')
                if gate.node.count_publishers('/super_debug/super_flight_target'):
                    raise RuntimeError('Another handover controller is present')
                from mars_quadrotor_msgs.msg import PolynomialTrajectory
                PolynomialTrajectory()
                gate.check()
                self.launch('handover_check',shell_command(['python3','-u',helper,
                    '--alignment-path',self.scripts/'aligned_pose_preview.py',
                    '--planner-config',self.config['planner_config_path']]))
                print('Ground handover auditor launched; managed wire sender stopped if present.')
                print('Wait for READY, then /super_flight/audit. One ground goal; debug only; no flight output.')
            elif action=='handover-stop':
                if 'handover_check' not in windows:raise RuntimeError('No managed handover auditor')
                gate.check();self.interrupt('handover_check')
                gate.wait(gate.grounded,'fresh ground state during handover shutdown')
                self.tmux('kill-window','-t',self.name+':handover_check')
                print('Handover auditor stopped; localization, vision and planning remain running.')
            elif action == 'wire-stop':
                if 'wire_test' not in windows:raise RuntimeError('No managed wire-test window')
                gate.check()
                self.interrupt('wire_test')
                gate.wait(gate.grounded,'fresh ground state during wire shutdown')
                self.tmux('kill-window','-t',self.name+':wire_test')
                print('Ground wire sender stopped; planning/localization/vision remain running.')
            elif action == 'planner-stop':
                self.stop_planning(gate)
            elif action == 'hover':
                if any(name in windows for name in ('planner', 'cmd_preview', 'reference_preview', 'wire_test', 'handover_check')):
                    raise RuntimeError('Stop ground planning with planner-stop before starting hover')
                if windows.get('preview') != '0' or windows.get('vision') != '0':
                    raise RuntimeError('Need the managed preview and vision relay running')
                if 'hover' in windows:
                    if not self.args.restart and (self.config['hover_execute'] or not self.args.execute):
                        raise RuntimeError('Hover window already exists; use --restart only after landing/disarming')
                    gate.check()
                    self.interrupt('hover')
                    self.tmux('kill-window', '-t', self.name+':hover')
                gate.check()
                command = ['python3', '-u', self.scripts/'first_hover_terminal.py',
                           '--hover-height', str(self.args.hover_height), '--hold-seconds', str(self.args.hold_seconds)]
                if self.args.execute:
                    command.append('--execute')
                self.config['hover_execute'] = self.args.execute
                self.save()
                self.launch('hover', shell_command(command))
                print('Hover process launched; inspect READY/GROUND LOCK. Flight still requires /first_hover/start.')
            elif action == 'stop':
                gate.check()
                # Stop publishers first; keep MAVROS last so ground-state gates
                # can be checked throughout cooperative Ctrl+C shutdown.
                for name in ('hover', 'wire_test', 'handover_check', 'reference_preview', 'cmd_preview', 'planner', 'vision', 'preview', 'body', 'lio', 'lidar', 'mavros'):
                    if name in windows:
                        gate.check()
                        self.interrupt(name)
                        if name != 'mavros':
                            gate.wait(gate.grounded, 'fresh ground state during shutdown')
                self.tmux('kill-session', '-t', self.name)
                print('Managed session stopped while grounded/disarmed. QGC was not closed.')
        finally:
            gate.close()


def unmanaged_processes():
    binaries = {'fastlio_mapping', 'mavros_node', 'livox_ros_driver2_node', 'livox_lidar_publisher', 'fsm_node'}
    scripts = set(SCRIPTS)-{'odom_age_check.py', 'frame_alignment_check.py', 'update_preview_alignment.py'}
    scripts |= {'raw_pose_cov_transport.py', 'aligned_pose_cov_transport.py', 'ground_xy_fusion_transport.py',
                'ground_height_fusion_transport.py', 'ground_setpoint_transport.py', 'first_hover_control.py', 'first_hover_preview.py',
                'bounded_reference_preview.py','super_flight_control.py'}
    found = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            words = path.read_bytes().split(b'\0')
            names = {Path(word.decode()).name for word in words if word}
        except (OSError, UnicodeError):
            continue
        matched = names & (binaries | scripts | {'msg_MID360_launch.py'})
        if matched:
            found.append(path.parent.name+':'+','.join(sorted(matched)))
    return found


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'align', 'vision', 'planner', 'goal', 'planner-check', 'reference-preview', 'wire-test', 'wire-stop', 'handover-check', 'handover-stop', 'flight-check', 'planner-stop', 'hover', 'status', 'logs', 'stop'))
    parser.add_argument('quaternion', nargs='*', type=float)
    parser.add_argument('--session', default='super-flight')
    parser.add_argument('--scripts-dir', type=Path, default=Path.home()/'Downloads')
    parser.add_argument('--fcu-url', default=DEFAULT_FCU)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--restart', action='store_true', help='replace only the hover window after a fresh ground-state check')
    parser.add_argument('--planner-profile', choices=('diagnostic', 'low-speed', 'low-speed-margin', 'low-speed-reserved', 'low-speed-3d'), default='diagnostic',
                        help='ground profile; low-speed 0.3/0.5, margin 0.3/0.45, reserved/3d 0.25/0.45; 3d preserves message Z before map relocation')
    parser.add_argument('--hover-height', type=float, default=1.)
    parser.add_argument('--goal-height',type=float,help='disarmed vertical goal: FC-centre floor height in (0.085, 1] m')
    parser.add_argument('--fixed-lio-extrinsics',action='store_true',help='start only: disable online IMU-LiDAR extrinsic estimation via a startup override; YAML unchanged')
    parser.add_argument('--hold-seconds', type=float, default=6.)
    parser.add_argument('--ros-env', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not args.session or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.session):
        parser.error('Use letters, digits, underscore or hyphen for session names')
    if args.action == 'vision':
        q = args.quaternion
        if len(q) != 4 or not all(math.isfinite(v) for v in q) or abs(math.sqrt(sum(v*v for v in q))-1) > .02:
            parser.error('vision requires four finite, approximately unit xyzw quaternion values')
    elif args.quaternion:
        parser.error('Quaternion arguments are only valid for vision')
    if args.restart and args.action != 'hover':
        parser.error('--restart is only valid for hover')
    if args.execute and args.action != 'hover':
        parser.error('--execute is only valid for hover')
    if args.fixed_lio_extrinsics and args.action != 'start':
        parser.error('--fixed-lio-extrinsics is only valid for start')
    if args.planner_profile != 'diagnostic' and args.action != 'planner':
        parser.error('--planner-profile is only valid for planner')
    if args.goal_height is not None and (args.action!='goal' or not math.isfinite(args.goal_height)
                                       or not .085<args.goal_height<=1.):
        parser.error('--goal-height is only for goal, with FC floor height in (0.085, 1] m')
    if not (.085 < args.hover_height <= 1. and math.isfinite(args.hover_height)
            and 0 <= args.hold_seconds <= 6 and math.isfinite(args.hold_seconds)):
        parser.error('Hover height must be (.085, 1] m and hold [0, 6] seconds')
    return args


def main():
    args = arguments()
    if subprocess.run(['which', 'tmux'], stdout=subprocess.DEVNULL).returncode:
        raise SystemExit('tmux is missing. Install once on Ubuntu: sudo apt install tmux')
    if args.action in ('align', 'vision', 'planner', 'goal', 'planner-check', 'reference-preview', 'wire-test', 'wire-stop', 'handover-check', 'handover-stop', 'flight-check', 'planner-stop', 'hover', 'stop') and not args.ros_env:
        body = source_prefix()+'exec '+shell_command([sys.executable, Path(__file__).resolve(), *sys.argv[1:], '--ros-env'])
        raise SystemExit(subprocess.run(['bash', '--noprofile', '--norc', '-c', body]).returncode)
    session = Session(args)
    if args.action == 'start':
        session.start()
    elif args.action == 'status':
        session.status()
    elif args.action == 'logs':
        session.load()
        session.tmux('attach-session', '-t', session.name)
    else:
        session.mutate(args.action)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        raise SystemExit(str(exc))
