#!/usr/bin/env python3
"""Prepare the canonical complete demonstration, with fresh automatic alignment.

Default is readiness only. No parameter writes beyond the existing guarded
alignment/vision initialization. Execution needs --execute --on-site-supervised
and a separate explicit /super_demo/start call with current RC/POSCTL.
"""
import argparse
from datetime import datetime
import fcntl
import json
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


def arguments(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mission-file',type=Path,required=True)
    p.add_argument('--flight-height',type=float,choices=(1.5,1.8),
                   help='FC-centre height above floor; saves a per-run mission copy without editing the survey')
    p.add_argument('--scripts-dir',type=Path,default=Path.home()/'Downloads')
    p.add_argument('--session',default='super-flight')
    p.add_argument('--restart',action='store_true')
    p.add_argument('--fixed-lio-extrinsics',action='store_true')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--on-site-supervised',action='store_true')
    p.add_argument('--motion-policy',choices=('strict','observe'),default='observe')
    p.add_argument('--qgc-path',type=Path,help='installed QGroundControl executable; defaults to the Downloads AppImage')
    p.add_argument('--ros-env',action='store_true',help=argparse.SUPPRESS)
    return p.parse_args(argv)


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
            if args.flight_height is not None:
                source=args.mission_file.expanduser().resolve()
                config=DemoMission.physical_height(json.loads(source.read_text()),args.flight_height)
                args.mission_file=runner.folder/'flight_mission.json'
                args.mission_file.write_text(json.dumps(config,indent=2)+'\n')
                runner.report.update(source_mission=str(source),effective_mission=str(args.mission_file),
                                     requested_fc_floor_height_m=args.flight_height)
                print('FLIGHT MISSION:',args.mission_file,flush=True)
            mission=DemoMission.load(args.mission_file)
            if mission.rehearsal_only:
                raise ValueError('Physical preparation requires a measured physical mission, not a frozen ground rehearsal')
            if args.execute and not args.on_site_supervised:
                raise ValueError('Explicit on-site supervision is required for flight execution')
            for name in ('super_demo_control.py','demo_mission.py','create_live_venue_profile.py',
                         'live_venue.py','venue_geometry.py','apply_super_venue_bounds.py'):
                if not (args.scripts_dir/name).is_file():raise ValueError('Missing mission dependency: '+name)
            runner.open_qgc(required=args.execute)
            runner.prepare(planner_profile='demo')
            runner.report.update(result='SUCCESS',stage='prepared');write_report(runner.path,runner.report)
            runner.session.load()
            command=[sys.executable,str(args.scripts_dir/'super_demo_control.py'),
                '--mission-file',str(args.mission_file),'--alignment-path',str(args.scripts_dir/'aligned_pose_preview.py'),
                '--planner-config',runner.session.config['planner_config_path'],
                '--venue-reference',runner.session.config['venue_reference_path'],
                '--preparation-report',str(runner.path),'--report-json',str(runner.folder/'report.json'),
                '--motion-policy',args.motion_policy]
            if args.execute:command+=['--execute','--on-site-supervised']
            return subprocess.run(command).returncode
        except (Exception,KeyboardInterrupt) as exc:
            runner.report.update(result='FAIL',error=type(exc).__name__+': '+str(exc))
            write_report(runner.path,runner.report)
            print('RESULT: FAIL\nERROR:',str(exc),'\nREPORT:',runner.path,flush=True);return 1


if __name__=='__main__':raise SystemExit(main())
