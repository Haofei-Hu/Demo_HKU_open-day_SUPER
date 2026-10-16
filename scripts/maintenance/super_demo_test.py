#!/usr/bin/env python3
"""From-scratch automatic test of the complete demonstration, without real flight.

Default: fresh real localization/alignment/map, then real SUPER in a private
domain with virtual FC/vehicle feedback. --logic-only is explicitly synthetic.
No execution, mode-change or arming option exists for the actual aircraft.
"""
import argparse
from collections import deque
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import super_session as manager
from super_ground_test import GroundTest,write_report
from demo_mission import DemoMission


def planner_log_evidence(path):
    """Retain planner evidence without treating an optimizer error as its cause."""
    markers={
        'exploration_success':'GenerateExpTrajectory SUCCESS',
        'backup_optimization_failure':'generateBackupTrajectory return [OPT_FAILED]',
        'backup_constraint_rejection':'Opt failed, Omg or thr or Pos violation',
        'goal_received':'Receive click goal',
        'missing_odometry':'[Fsm] No odom.',
    }
    counts={name:0 for name in markers};tail=deque(maxlen=100)
    with path.open(errors='replace') as handle:
        for line in handle:
            tail.append(line.rstrip('\n'))
            for name,marker in markers.items():
                if marker in line:counts[name]+=1
    return dict(log=str(path),marker_counts=counts,output_tail='\n'.join(tail),
                cause_inferred=False)


class DemoTest(GroundTest):
    def __init__(self,args):
        self.args=args;self.observer=None;self.child=None
        self.session=manager.Session(manager.arguments(['start','--session',args.session,
                                                      '--scripts-dir',str(args.scripts_dir)]))
        self.folder=args.scripts_dir/'super_demo_tests'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.folder.mkdir(parents=True);self.path=self.folder/'report.json'
        self.report=dict(schema_version=1,result='RUNNING',stage='preflight',stages=[],
            mode='logic-simulation' if args.logic_only else 'real-SUPER-ground-rehearsal',
            real_aircraft_native_output=False,real_aircraft_mode_or_arming_requests=False,
            demonstration_completed=False,
            physical_flight_verified=False,physical_venue_clearance_verified=False,
            actual_localization_tested=False,actual_SUPER_tested=False,
            route_profile=getattr(args,'route_profile','demonstration'),
            motion_policy=getattr(args,'motion_policy','strict'),
            fixture_geometry=args.fixture_geometry,started_at=datetime.now().astimezone().isoformat())

    def step(self,name,command,env=None):
        self.report['stage']=name
        log=self.folder/(str(len(self.report['stages']))+'_'+name+'.log')
        stage=dict(action=name,log=str(log));self.report['stages'].append(stage);write_report(self.path,self.report)
        print('['+name+']',log,flush=True)
        try:
            with log.open('w') as out:
                self.child=subprocess.Popen(list(map(str,command)),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                            text=True,start_new_session=True,env=env)
                for line in self.child.stdout:out.write(line);out.flush();print(line,end='',flush=True)
                stage['returncode']=self.child.wait()
            if stage['returncode']:
                raise RuntimeError(name+' failed; inspect '+str(log))
        finally:
            if log.exists():
                stage['output_tail']='\n'.join(log.read_text(errors='replace').splitlines()[-80:])
                if stage.get('returncode')!=0:self.report['failure_output']=stage['output_tail']
            write_report(self.path,self.report)

    def action(self,action,*extra):
        # Full rehearsal preparation uses the same completion-driven subprocess
        # path as mission execution, including an unbounded stable-map wait.
        command=[sys.executable,self.args.scripts_dir/'super_session.py',
                 '--session',self.args.session,'--scripts-dir',self.args.scripts_dir,action]
        if action=='vision':command.append('--')
        command.extend(map(str,extra))
        self.step(action,command)

    def run_complete(self):
        write_report(self.path,self.report);print('REPORT:',self.path,flush=True)
        try:
            required=('demo_mission.py','demo_mission_simulation.py','super_demo_control.py')
            if not self.args.logic_only:required+=('capture_demo_scene.py',)
            for name in required:
                if not (self.args.scripts_dir/name).is_file():raise RuntimeError('Missing dependency: '+name)
            config=json.loads(self.args.mission_file.read_text())
            config=DemoMission.ground_rehearsal(config)
            if getattr(self.args,'route_profile','demonstration')=='short-1m':
                if not self.args.fixture_geometry:raise ValueError('Short-site rehearsal needs explicitly labelled --fixture-geometry')
                config=DemoMission.short_site_fixture(config)
            if self.args.fixture_geometry:
                config['takeoff'].update(forward_from_rear_m=.5,distance_from_left_m=1.4)
                for side in ('front','rear','left','right'):config['vehicle']['centre_to_'+side+'_m']=.2
                config['vehicle'].update(centre_to_top_m=.1,centre_to_bottom_m=.085)
                config['status']='SYNTHETIC_TEST_GEOMETRY_NOT_AIRCRAFT_MEASUREMENTS'
            mission=DemoMission(config)
            frozen=self.folder/'mission.json';frozen.write_text(json.dumps(config,indent=2)+'\n')
            self.report['mission']=config;self.report['waypoints']=mission.waypoints()
            self.report['execution_context']='ground_rehearsal'
            self.report['requested_fc_floor_height_m']=mission.hover_height
            self.report['round_trips_requested']=mission.round_trips
            print(f'ROUTE: {config["mission_id"]}; {mission.hover_height:g} m height, '
                  f'{mission.round_trips} round trips of {mission.distance:g} m each way, final 6 s hover, LAND',flush=True)
            scene=None
            if not self.args.logic_only:
                self.open_qgc()
                self.prepare(planner_profile='demo')
                self.report['vision_preparation_policy']=dict(continuous_window_s=30.,
                    total_wait_deadline_s=None,header_age_band_s=[-.05,.25],receive_gap_limit_s=.25,
                    temporal_interruptions='Record lifetime evidence and restart observation only',
                    integrity_faults='Fail on invalid/reordered stamps, wrong frames or changed/duplicate sources')
                self.step('vision_chain',[sys.executable,self.args.scripts_dir/'control_chain_age_check.py',
                                          '--vision-chain','--wait-stable','--seconds','30'])
                self.report['actual_localization_tested']=True
                self.session.load();scene=self.folder/'scene.json'
                self.step('capture_scene',[sys.executable,self.args.scripts_dir/'capture_demo_scene.py',
                    '--output',scene,'--alignment-path',self.args.scripts_dir/'aligned_pose_preview.py',
                    '--planner-config',self.session.config['planner_config_path']])
            command=[sys.executable,self.args.scripts_dir/'demo_mission_simulation.py',
                     '--isolated','--mission-file',frozen,'--report-json',self.folder/'mission_report.json',
                     '--motion-policy',getattr(self.args,'motion_policy','strict')]
            if getattr(self.args,'no_rviz',False):command+=['--no-rviz']
            if scene is not None:command+=['--scene-file',scene]
            self.step('complete_mission',command,env=dict(os.environ,ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',
                                                        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
            child=json.loads((self.folder/'mission_report.json').read_text())
            self.report['mission_report']=child
            self.report['round_trips_completed']=child.get('round_trips_completed',0)
            if (child.get('result')!='SUCCESS' or child.get('completed_stages')!=list(mission.PHASES)
                    or child.get('round_trips_completed')!=mission.round_trips):
                raise RuntimeError('Complete ordered mission and confirmed landing were not established')
            actual=child.get('simulation_backend',{}).get('actual_SUPER_used') is True
            if not self.args.logic_only and not actual:raise RuntimeError('Real SUPER rehearsal was not used; no synthetic fallback allowed')
            self.report['actual_SUPER_tested']=actual
            if not self.args.logic_only:
                gate=manager.GroundGate()
                try:gate.check();self.report['actual_aircraft_still_grounded_disarmed']=True
                finally:gate.close()
            self.report.update(result='SUCCESS',stage='complete',demonstration_completed=mission.demonstration)
            return 0
        except (Exception,KeyboardInterrupt) as exc:
            self.report.update(result='FAIL',error=type(exc).__name__+': '+str(exc),failed_stage=self.report['stage'])
            childpath=self.folder/'mission_report.json'
            if childpath.exists():
                child=json.loads(childpath.read_text())
                self.report['partial_mission_report']=child
                self.report['round_trips_completed']=child.get('round_trips_completed',0)
                # Planner participation and mission completion are independent facts.
                self.report['actual_SUPER_tested']=child.get('simulation_backend',{}).get('actual_SUPER_used') is True
            print('ERROR:',self.report['error'],flush=True);return 1
        finally:
            if self.child is not None and self.child.poll() is None:
                os.killpg(self.child.pid,signal.SIGINT)
                try:self.child.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(self.child.pid,signal.SIGTERM);self.child.wait(timeout=5)
            planner_log=self.folder/'planner_rehearsal.log'
            if planner_log.is_file():
                self.report['planner_diagnostics']=planner_log_evidence(planner_log)
            self.report['finished_at']=datetime.now().astimezone().isoformat();write_report(self.path,self.report)
            print('RESULT:',self.report['result'],flush=True);print('REPORT:',self.path,flush=True)
            print('SCOPE:',self.report['mode'],'; no real flight was requested',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scripts-dir',type=Path,default=Path.home()/'Downloads')
    p.add_argument('--mission-file',type=Path,default=Path.home()/'Downloads/open_day_mission.json')
    p.add_argument('--session',default='super-flight')
    p.add_argument('--restart',action='store_true')
    p.add_argument('--fixed-lio-extrinsics',action='store_true')
    p.add_argument('--fixture-geometry',action='store_true',help='labelled virtual placement/body for rehearsal only')
    p.add_argument('--route-profile',choices=('demonstration','short-1m'),default='demonstration',
                   help='full 4.5 m demonstration or separate 1 m short-site rehearsal')
    p.add_argument('--motion-policy',choices=('strict','observe'),default='strict',
                   help='observe records derivative target exceedances only in the isolated virtual rehearsal')
    p.add_argument('--logic-only',action='store_true',help='synthetic planner, no actual aircraft stack or real SUPER')
    p.add_argument('--no-rviz',action='store_true',help='disable the automatically managed rehearsal viewer')
    p.add_argument('--qgc-path',type=Path,help='installed QGroundControl executable; defaults to the Downloads AppImage')
    p.add_argument('--no-qgc',action='store_true',help='skip QGroundControl during the disarmed rehearsal')
    p.add_argument('--ros-env',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args();args.scripts_dir=args.scripts_dir.expanduser().resolve()
    if not args.ros_env:
        return subprocess.run(['bash','--noprofile','--norc','-c',manager.source_prefix()+'exec '+
            manager.shell_command([sys.executable,Path(__file__).resolve(),'--ros-env',*sys.argv[1:]])]).returncode
    args.test_profile='forward-5m-2mps';args.max_alignment_spread_deg=2.
    runner=DemoTest(args)
    lockpath=Path.home()/'.local/state/super-session/automated_ground_test.lock';lockpath.parent.mkdir(parents=True,exist_ok=True)
    with lockpath.open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            runner.report.update(result='FAIL',error='Another automatic task owns the stack')
            write_report(runner.path,runner.report);print('RESULT: FAIL\nREPORT:',runner.path);return 1
        return runner.run_complete()


if __name__=='__main__':raise SystemExit(main())
