#!/usr/bin/env python3
"""Rehearse five left-facing 4.8 m round trips from a saved map, without touching the aircraft stack."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback

DOMAIN=198


def check_private_domain():
    import time
    import rclpy
    if os.environ.get('ROS_DOMAIN_ID')!=str(DOMAIN) or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('Left rehearsal requires isolated localhost domain 198')
    rclpy.init();node=rclpy.create_node('left_rehearsal_isolation_check')
    try:
        began=time.monotonic()
        while time.monotonic()-began<2:rclpy.spin_once(node,timeout_sec=.05)
        others=[name for name,_ in node.get_node_names_and_namespaces()
                if name!='left_rehearsal_isolation_check']
        if others:raise ValueError('Left rehearsal domain already in use; no processes stopped: '+str(others))
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene-file',type=Path,help='original saved scene; defaults to latest successful real-SUPER run')
    p.add_argument('--source-reports',type=Path,default=Path.home()/'Downloads/super_demo_tests')
    p.add_argument('--reports-dir',type=Path,default=Path.home()/'Downloads/super_left_48_tests')
    p.add_argument('--no-rviz',action='store_true')
    p.add_argument('--no-qgc',action='store_true')
    p.add_argument('--qgc-path',type=Path)
    p.add_argument('--ros-env',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args()
    if not args.ros_env:
        import super_session as manager
        env=dict(os.environ,ROS_DOMAIN_ID=str(DOMAIN),ROS_LOCALHOST_ONLY='1',
                 OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        command=manager.source_prefix()+'exec '+manager.shell_command(
            [sys.executable,Path(__file__).resolve(),'--ros-env',*sys.argv[1:]])
        return subprocess.run(['bash','--noprofile','--norc','-c',command],env=env).returncode
    from left_rehearsal import LeftMission,mission_fixture,rotated_scene,successful_scene,left_planner_profile
    from left_demo_control import LeftMissionControl
    from demo_mission_simulation import run
    folder=args.reports_dir.expanduser()/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True);report=folder/'report.json'
    print('REPORT:',report,flush=True)
    try:
        # A separate lock and ROS domain leave the original workflow untouched.
        lockpath=Path.home()/'.local/state/super-session/left_rehearsal.lock'
        lockpath.parent.mkdir(parents=True,exist_ok=True)
        with lockpath.open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            check_private_domain()
            source=args.scene_file or successful_scene(args.source_reports.expanduser())
            scene=left_planner_profile(rotated_scene(source),report);scene_path=folder/'scene.json'
            scene_path.write_text(json.dumps(scene,indent=2)+'\n')
            mission=mission_fixture();LeftMission(mission)
            mission_path=folder/'mission.json';mission_path.write_text(json.dumps(mission,indent=2)+'\n')
            print('SOURCE SCENE:',source,flush=True)
            print('VIRTUAL ONLY: initial heading +90 deg left; 0.5 m rise; five 4.8 m round trips; 6 s final hold; LAND',flush=True)
            print('REAL MAP unchanged; saved snapshot reused. Actual aircraft stack and original files are untouched.',flush=True)
            from qgc_startup import open_qgc
            qgc=open_qgc(folder,args.qgc_path,enabled=not args.no_qgc)
            (folder/'qgc_status.json').write_text(json.dumps(qgc,indent=2)+'\n')
            passed=run(mission_path,report,scene_file=scene_path,motion_policy='observe',
                       rviz=not args.no_rviz,controller_class=LeftMissionControl,domain_id=DOMAIN)
            return 0 if passed else 1
    except (Exception,KeyboardInterrupt) as exc:
        traceback.print_exc()
        data=json.loads(report.read_text()) if report.exists() else {}
        data.update(result='FAIL',error=type(exc).__name__+': '+str(exc),
                    physical_flight_verified=False,scope='separate left-facing virtual rehearsal')
        report.write_text(json.dumps(data,indent=2)+'\n')
        return 1
    finally:
        result=json.loads(report.read_text()).get('result','FAIL') if report.exists() else 'FAIL'
        print('RESULT:',result,flush=True);print('REPORT:',report,flush=True)


if __name__=='__main__':raise SystemExit(main())
