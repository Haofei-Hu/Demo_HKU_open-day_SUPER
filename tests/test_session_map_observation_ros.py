"""Live DDS raw-height capture and blocking conditions; no aircraft or writes."""
import importlib.util
from pathlib import Path
import threading
import time
import subprocess
import sys
import tempfile
import yaml

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus

ROOT=Path(__file__).resolve().parents[1]
def load(name,file):
    spec=importlib.util.spec_from_file_location(name,ROOT/file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
m=load('map_profile','scripts/maintenance/create_session_map_profile.py')
manager=load('session','scripts/maintenance/super_session.py')


def run(case,wait_until_stable=False):
    ctx=Context();rclpy.init(context=ctx);probe=Node('map_reference_fixture',context=ctx)
    kinds={'/Odometry':Odometry,'/mavros/state':State,'/mavros/extended_state':ExtendedState,
           '/mavros/estimator_status':EstimatorStatus,'/mavros/local_position/pose':PoseStamped,
           '/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
           '/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped}
    pubs={topic:probe.create_publisher(kind,topic,16) for topic,kind in kinds.items()}
    began=None;extra=None
    def publish():
        nonlocal extra
        elapsed=time.monotonic()-began if began is not None else 0.
        if case=='duplicate' and elapsed>.5 and extra is None:
            extra=probe.create_publisher(Odometry,'/Odometry',16)
        for topic,kind in kinds.items():
            if kind is Odometry and case=='raw_gap' and elapsed>.5:continue
            msg=kind();msg.header.stamp=probe.get_clock().now().to_msg()
            if kind is State:
                msg.connected=True;msg.mode='AUTO.LOITER';msg.armed=case=='armed' and elapsed>.5
            elif kind is ExtendedState:msg.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
            elif kind is EstimatorStatus:
                msg.attitude_status_flag=msg.velocity_horiz_status_flag=msg.velocity_vert_status_flag=True
                msg.pos_horiz_rel_status_flag=msg.pos_vert_abs_status_flag=True
            elif kind is Odometry:
                msg.header.frame_id='camera_init';msg.child_frame_id='body';msg.pose.pose.orientation.w=1.
                if case=='settles_late':
                    # Alternate full 4 cm excursions until 13 s, then remain
                    # exactly still. A qualifying window must age them all out.
                    offset=.04 if elapsed<13. and int(elapsed*2)%2 else 0.
                else:
                    offset=.02*elapsed if case=='movement' else (
                        .02676 if case=='settles' and .5<elapsed<1.5 else 0.)
                msg.pose.pose.position.z=-1.1827+offset
            elif kind is PoseStamped:msg.header.frame_id='map';msg.pose.orientation.w=1.
            else:msg.header.frame_id='px4_enu_preview';msg.pose.pose.orientation.w=1.
            pubs[topic].publish(msg)
    probe.create_timer(.02,publish)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(probe)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start();gate=None
    try:
        gate=manager.GroundGate();gate.wait(gate.localization_ready,'fresh fixture localization')
        began=time.monotonic()
        if case in ('healthy','settles','settles_late'):
            observation=m.observe(gate,wait_until_stable=wait_until_stable or case=='settles_late')
            assert abs(observation['raw_z']+1.1827)<1e-8 and observation['raw_source_gid']
            if case=='settles_late':
                assert 15.5<time.monotonic()-began<24.
                print('PASS: capture remained pending beyond 12 s, then accepted a full stable window without relaxing 2.5 cm',flush=True)
                return
            if case=='settles':
                assert 4.<time.monotonic()-began<8.
                print('PASS: initial 2.676 cm excursion aged out; later full stable window accepted',flush=True)
            with tempfile.TemporaryDirectory() as directory:
                folder=Path(directory);source=folder/'base.yaml';target=folder/'session.yaml'
                record=folder/'map_reference.json';alignment=folder/'alignment.py'
                cfg=yaml.safe_load((ROOT/'configs/observed_2026-10-08/mid360_real_ros2.yaml').read_text())
                cfg['rog_map']['virtual_ground_height']=-1.5
                source.write_text(yaml.safe_dump(cfg));alignment.write_text('# fixed geometry\n')
                command=[sys.executable,str(ROOT/'scripts/maintenance/create_session_map_profile.py'),
                    '--source',str(source),'--target',str(target),'--record',str(record),
                    '--alignment-path',str(alignment)]
                subprocess.run(command,check=True,timeout=20)
                created=target.read_bytes();anchor=record.read_bytes()
                subprocess.run(command,check=True,timeout=20)
                assert target.read_bytes()==created and record.read_bytes()==anchor
                assert abs(yaml.safe_load(created)['rog_map']['virtual_ground_height']+2.7)<1e-8
                gate.wait(gate.grounded,'refreshed ground state after helper subprocess')
                gate.wait(gate.localization_ready,'refreshed localization after helper subprocess')
                gate.check()
                print('PASS: real helper subprocess, idempotent frozen anchor and parent telemetry refresh',flush=True)
        else:
            expected={'movement':'not stationary','raw_gap':'odometry gap','armed':'Ground state lost',
                      'duplicate':'Multiple raw'}[case]
            try:m.observe(gate,wait_until_stable=wait_until_stable)
            except RuntimeError as exc:
                assert expected in str(exc),str(exc)
                if case=='movement':
                    assert 'raw XYZ range cm=' in str(exc),str(exc)
                    assert 'no stable 3 s window within the 12 s' in str(exc),str(exc)
                    assert 11.5<time.monotonic()-began<13.5
                    assert 'no map profile written' in str(exc),str(exc)
            else:raise AssertionError('Invalid ground reference accepted: '+case)
        print('PASS: map reference observation',case,flush=True)
    finally:
        if gate:gate.close()
        executor.shutdown();thread.join(timeout=2);probe.destroy_node();rclpy.shutdown(context=ctx)


if __name__=='__main__':
    for case in ('healthy','settles','movement','raw_gap','armed','duplicate'):run(case)
