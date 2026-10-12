"""Isolated DDS fixture: endpoint hold, restart suppression and output gates."""
import math
from pathlib import Path
import sys
import tempfile
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus
from mars_quadrotor_msgs.msg import PositionCommand, PolynomialTrajectory

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/planning'))
from bounded_reference_preview import Plan,node_type


def run(case):
    with tempfile.TemporaryDirectory() as folder:
        alignment=Path(folder)/'aligned_pose_preview.py'
        alignment.write_text('def convert(p,q,covariance):\n    return p,q,covariance\n')
        rclpy.init(args=['--ros-args','-p','alignment_path:='+str(alignment)])
        preview=node_type()();probe=Node('bounded_reference_fixture')
        executor=SingleThreadedExecutor();executor.add_node(preview);executor.add_node(probe)
        kinds={'/Odometry':Odometry,'/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
               '/mavros/state':State,'/mavros/extended_state':ExtendedState,
               '/mavros/estimator_status':EstimatorStatus,'/planning/pos_cmd':PositionCommand,
               '/planning_cmd/poly_traj':PolynomialTrajectory}
        pubs={topic:probe.create_publisher(kind,topic,10) for topic,kind in kinds.items()}
        output=[]
        subscription=probe.create_subscription(PoseStamped,'/super_debug/bounded_control_target',output.append,10)
        settings=dict(armed=False,ground=True,estimator_age=0.,running=False,start=0.,poses=True,finished=False)

        def telemetry():
            stamp=probe.get_clock().now().to_msg()
            state=State();state.header.stamp=stamp;state.connected=True;state.armed=settings['armed']
            pubs['/mavros/state'].publish(state)
            ground=ExtendedState();ground.header.stamp=stamp
            ground.landed_state=(ExtendedState.LANDED_STATE_ON_GROUND if settings['ground'] else ExtendedState.LANDED_STATE_IN_AIR)
            pubs['/mavros/extended_state'].publish(ground)
            est=EstimatorStatus()
            est_ns=probe.get_clock().now().nanoseconds-int(settings['estimator_age']*1e9)
            est.header.stamp.sec,est.header.stamp.nanosec=divmod(est_ns,1000000000)
            for name in ('attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag',
                         'pos_horiz_rel_status_flag','pos_vert_abs_status_flag'):setattr(est,name,True)
            pubs['/mavros/estimator_status'].publish(est)
            if settings['poses']:
                raw=Odometry();raw.header.stamp=stamp;raw.header.frame_id='camera_init';raw.child_frame_id='body'
                raw.pose.pose.orientation.w=1.
                aligned=PoseWithCovarianceStamped();aligned.header.stamp=stamp;aligned.header.frame_id='px4_enu_preview'
                aligned.pose.pose.orientation.w=1.
                pubs['/Odometry'].publish(raw);pubs['/super_debug/aligned_pose_cov'].publish(aligned)

        def commands():
            if not settings['running']:return
            now=probe.get_clock().now().nanoseconds
            elapsed=now/1e9-settings['start']
            if elapsed<0:return
            restart=settings['finished']
            start=settings['start']+(2.02 if restart else 0.)
            poly=PolynomialTrajectory();poly.header.stamp=probe.get_clock().now().to_msg();poly.header.frame_id='world'
            poly.type=3;poly.order_pos=7;poly.piece_num_pos=1;poly.start_wt_pos=float(start);poly.time_pos=[2.]
            poly.coef_pos_x=[0.,0.,.01875,-.09375,.125,0.,0.,0.]
            poly.coef_pos_y=poly.coef_pos_z=[0.]*8
            pubs['/planning_cmd/poly_traj'].publish(poly)
            p,v,a=Plan(poly).evaluate(now/1e9-start)
            msg=PositionCommand();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(now,1000000000)
            msg.header.frame_id='world';msg.trajectory_flag=1
            for name,values in (('position',p),('velocity',v),('acceleration',a)):
                for axis,value in zip(('x','y','z'),values):setattr(getattr(msg,name),axis,float(value))
            pubs['/planning/pos_cmd'].publish(msg)
            if elapsed>=2.:settings['finished']=True

        timers=[probe.create_timer(.02,telemetry),probe.create_timer(.01,commands)]
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        def wait(predicate):
            end=time.monotonic()+6
            while not predicate():
                assert time.monotonic()<end,f'Fixture timed out: {case}; {preview.fault_reason}'
                spin(.02)

        duplicate=None
        try:
            wait(lambda:preview.reference is not None)
            assert not output,'Output before a verified command'
            names=[name for name,_ in preview.get_publisher_names_and_types_by_node(preview.get_name(),preview.get_namespace())]
            assert not any(name.startswith('/mavros/') or name=='/goal_pose' for name in names),names
            settings.update(running=True,start=probe.get_clock().now().nanoseconds/1e9+.1)
            if case=='hold':
                wait(lambda:preview.reference.phase=='END_HOLD')
                wait(lambda:preview.reference.restarts_blocked>2)
                count=len(output);spin(.2)
                assert len(output)>count,'Held preview not emitted'
                assert all(.099<msg.pose.position.x<.101 for msg in output[count:]),'Restart leaked through hold'
                settings['estimator_age']=3.;spin(.15);count=len(output);spin(.1)
                assert len(output)==count,'Stale estimator leaked output'
                settings['estimator_age']=0.;wait(lambda:len(output)>count)
                settings['poses']=False;spin(.35);count=len(output);spin(.1)
                assert len(output)==count,'Stale paired localization leaked output'
                settings['poses']=True;wait(lambda:len(output)>count)
                alignment.write_text(alignment.read_text()+'# changed\n')
                wait(lambda:bool(preview.fault_reason))
                count=len(output);spin(.1);assert len(output)==count
                assert 'alignment' in preview.fault_reason
            elif case=='source':
                wait(lambda:bool(output))
                duplicate=probe.create_publisher(PolynomialTrajectory,'/planning_cmd/poly_traj',10)
                wait(lambda:bool(preview.fault_reason))
                count=len(output);spin(.1);assert len(output)==count
                probe.destroy_publisher(duplicate);duplicate=None
                spin(.3);assert len(output)==count,'Source fault recovered automatically'
            else:
                wait(lambda:bool(output));settings['armed']=True
                wait(lambda:preview.arm_seen and bool(preview.fault_reason))
                count=len(output);settings['armed']=False;spin(.2)
                assert len(output)==count,'Arming latch recovered automatically'
            assert preview.reference.phase=='FAULT'
        finally:
            executor.shutdown();preview.destroy_node();probe.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    for case in ('hold','source','arm'):run(case)
    print('PASS: live committed-polynomial match, endpoint hold and restart suppression; '
          'stale estimator/localization gates, alignment/source/arming latches; no MAVROS/goal publishers.')
