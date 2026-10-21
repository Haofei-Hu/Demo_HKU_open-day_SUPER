#!/usr/bin/env python3
"""Prepare the canonical complete demonstration, with fresh automatic alignment.

Default is readiness only. No parameter writes beyond the existing guarded
alignment/vision initialization. Execution needs --execute --on-site-supervised
and a separate explicit /super_demo/start call with current RC/POSCTL.
"""
import argparse
from copy import deepcopy
from datetime import datetime
import fcntl
import json
import math
from pathlib import Path
import subprocess
import sys

import super_session as manager
from super_ground_test import GroundTest,write_report
from demo_mission import DemoMission


class DemoPreparation(GroundTest):
    def __init__(self,args):
        self.args=args
        self.session=manager.Session(manager.arguments(['start','--session',args.session,
                                                       '--scripts-dir',str(args.scripts_dir)]))
        self.folder=args.scripts_dir/'super_demo_reports'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.folder.mkdir(parents=True)
        self.path=self.folder/'preparation.json'
        self.report=dict(result='RUNNING',stage='preflight',stages=[],scope='Complete demonstration preparation')

    def action(self,action,*extra):
        if action=='planner':extra=(*extra,'--venue-mission',self.args.mission_file.resolve())
        return super().action(action,*extra)

    def after_vision_ready(self):
        if not self.args.calibrate_venue:return
        from calibrate_venue_rviz import calibrate
        extra={'full_size_execution_venue':True} if self.args.full_size_execution_venue else {}
        config=calibrate(self.calibration_source,self.args.scripts_dir/'aligned_pose_preview.py',
                         self.folder,self.args.rear_offset,venue_width=self.args.venue_width,
                         venue_ceiling=self.args.venue_ceiling,**extra)
        if not self.args.calibration_only:
            config=DemoMission.physical_height(config,self.args.flight_height)
        self.args.mission_file=self.folder/'flight_mission.json'
        self.args.mission_file.write_text(json.dumps(config,indent=2)+'\n')
        self.report.update(effective_mission=str(self.args.mission_file),
            requested_fc_floor_height_m=self.args.flight_height,
            venue_calibration_report=str(self.folder/'venue_calibration.json'))
        write_report(self.path,self.report)


def arguments(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mission-file',type=Path,required=True)
    p.add_argument('--flight-height',type=float,choices=(1.,1.5,1.8),default=1.5,
                   help='FC-centre height above floor (default: 1.5 m); saves a per-run mission copy without editing the survey')
    p.add_argument('--calibrate-venue',action='store_true',help='Select left/right sides in RViz after fresh localization')
    p.add_argument('--calibration-only',action='store_true',help='Stop after selection; permits incomplete aircraft geometry, no planner/controller')
    p.add_argument('--rear-offset',type=float,help='Previously surveyed FC-to-rear offset; lateral selection does not measure the rear boundary')
    p.add_argument('--venue-width',type=float,help='Optional total width cap centred on takeoff and intersected with RViz-selected sides')
    p.add_argument('--venue-ceiling',type=float,help='Optional outer top boundary above floor, at most 2.8 m; includes aircraft top clearance')
    p.add_argument('--full-size-execution-venue',action='store_true',
                   help='Confirm a separate 5.8 x 2.8 x 2.8 m measured-position boundary around the planning box')
    p.add_argument('--scripts-dir',type=Path,default=Path.home()/'Downloads')
    p.add_argument('--session',default='super-flight')
    p.add_argument('--restart',action='store_true')
    p.add_argument('--fixed-lio-extrinsics',action='store_true')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--on-site-supervised',action='store_true')
    p.add_argument('--hover-only',action='store_true',help='Take off, hold fixed XY/yaw and land; no outbound or return flight')
    p.add_argument('--hover-seconds',type=float,default=20.,help='Planned hover duration for --hover-only (default: 20 s)')
    p.add_argument('--motion-policy',choices=('strict','observe'),default='observe')
    p.add_argument('--qgc-path',type=Path,help='installed QGroundControl executable; defaults to the Downloads AppImage')
    p.add_argument('--ros-env',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args(argv)
    if args.full_size_execution_venue and not args.calibrate_venue:
        p.error('--full-size-execution-venue requires --calibrate-venue and preview confirmation')
    if not math.isfinite(args.hover_seconds) or args.hover_seconds<=0:
        p.error('--hover-seconds must be finite and positive')
    if args.hover_only and args.calibration_only:
        p.error('--hover-only cannot be combined with --calibration-only')
    if args.calibration_only and (not args.calibrate_venue or args.execute):
        p.error('--calibration-only requires --calibrate-venue and cannot execute flight')
    if args.venue_width is not None or args.venue_ceiling is not None:
        if not args.calibrate_venue:p.error('Venue caps require --calibrate-venue and preview confirmation')
        from rviz_venue_calibration import cap_selected_box
        try:cap_selected_box({'selected_lateral_limits_m':[-1.4,1.4]},args.venue_width,args.venue_ceiling)
        except ValueError as exc:p.error(str(exc))
    return args


def main():
    args=arguments();args.scripts_dir=args.scripts_dir.expanduser().resolve()
    if not args.ros_env:
        return subprocess.run(['bash','--noprofile','--norc','-c',manager.source_prefix()+'exec '+
            manager.shell_command([sys.executable,Path(__file__).resolve(),'--ros-env',*sys.argv[1:]])]).returncode
    args.test_profile='forward-5m-2mps'  # Shared 2/1 prerequisite set; no 5 m goal is sent.
    args.max_alignment_spread_deg=2.
    runner=DemoPreparation(args)
    lockpath=Path.home()/'.local/state/super-session/automated_ground_test.lock'
    lockpath.parent.mkdir(parents=True,exist_ok=True)
    with lockpath.open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            runner.report.update(result='FAIL',error='Another automatic task owns the stack')
            write_report(runner.path,runner.report);print('RESULT: FAIL\nREPORT:',runner.path);return 1
        try:
            if args.calibrate_venue:
                source=args.mission_file.expanduser().resolve()
                config=json.loads(source.read_text())
                if config.get('execution_context') is not None or 'SYNTHETIC' in str(config.get('status','')).upper():
                    raise ValueError('RViz preparation needs a physical mission template')
                if args.rear_offset is None:args.rear_offset=config['takeoff'].get('forward_from_rear_m')
                args.rear_offset=DemoMission.measure(args.rear_offset,'rear offset (not measured by left/right selection)')
                if config.get('venue',{}).get('length_forward_m')!=5.8 or config['venue'].get('height_m')!=2.8:
                    raise ValueError('RViz calibration currently supports the 5.8 x 2.8 x 2.8 m demonstration')
                config['venue'].pop('calibration_method',None);config.pop('venue_calibration',None)
                config['venue']['width_left_m']=2.8
                config['flight']['height_fc_above_floor_m']=args.flight_height
                if not args.calibration_only:
                    # Check the aircraft fields before starting the stack; these
                    # temporary offsets are never saved or used for flight.
                    check=deepcopy(config);check['takeoff'].update(forward_from_rear_m=args.rear_offset,distance_from_left_m=1.4)
                    DemoMission(check)
                runner.calibration_source=config
                runner.report['source_mission']=str(source)
            elif args.flight_height is not None:
                source=args.mission_file.expanduser().resolve()
                config=DemoMission.physical_height(json.loads(source.read_text()),args.flight_height)
                args.mission_file=runner.folder/'flight_mission.json'
                args.mission_file.write_text(json.dumps(config,indent=2)+'\n')
                runner.report.update(source_mission=str(source),effective_mission=str(args.mission_file),
                                     requested_fc_floor_height_m=args.flight_height)
                print('FLIGHT MISSION:',args.mission_file,flush=True)
            if not args.calibrate_venue:
                mission=DemoMission.load(args.mission_file)
                if mission.rehearsal_only:
                    raise ValueError('Physical preparation requires a measured physical mission, not a frozen ground rehearsal')
            if args.execute and not args.on_site_supervised:
                raise ValueError('Explicit on-site supervision is required for flight execution')
            for name in ('super_demo_control.py','demo_mission.py','create_live_venue_profile.py',
                         'live_venue.py','venue_geometry.py','apply_super_venue_bounds.py'):
                if not (args.scripts_dir/name).is_file():raise ValueError('Missing mission dependency: '+name)
            runner.open_qgc(required=args.execute)
            runner.prepare(planner_profile='demo',prepare_planner=not args.calibration_only)
            if args.calibration_only:
                runner.report.update(result='SUCCESS',stage='venue_calibrated',flight_readiness_verified=False)
                write_report(runner.path,runner.report)
                print('CALIBRATION: SUCCESS; FLIGHT READINESS: NOT ASSESSED; MISSION: NOT RUN\nREPORT:',runner.path,flush=True)
                return 0
            runner.report.update(result='SUCCESS',stage='prepared');write_report(runner.path,runner.report)
            runner.session.load()
            command=[sys.executable,str(args.scripts_dir/'super_demo_control.py'),
                '--mission-file',str(args.mission_file),'--alignment-path',str(args.scripts_dir/'aligned_pose_preview.py'),
                '--planner-config',runner.session.config['planner_config_path'],
                '--venue-reference',runner.session.config['venue_reference_path'],
                '--preparation-report',str(runner.path),'--report-json',str(runner.folder/'report.json'),
                '--motion-policy',args.motion_policy]
            if args.hover_only:command+=['--hover-only','--hover-seconds',str(args.hover_seconds)]
            if args.execute:command+=['--execute','--on-site-supervised']
            return subprocess.run(command).returncode
        except (Exception,KeyboardInterrupt) as exc:
            runner.report.update(result='FAIL',error=type(exc).__name__+': '+str(exc))
            write_report(runner.path,runner.report)
            print('RESULT: FAIL\nERROR:',str(exc),'\nREPORT:',runner.path,flush=True);return 1


if __name__=='__main__':raise SystemExit(main())
