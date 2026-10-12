"""Isolated live DDS fixtures for read-only sampling; no aircraft connection."""
import contextlib
import importlib.util
import io
from pathlib import Path
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mars_quadrotor_msgs.msg import PositionCommand

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('metrics',ROOT/'scripts/analysis/control_chain_age_check.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
ctx=Context();rclpy.init(context=ctx)
probe=Node('metrics_fixture',context=ctx)
types={'/Odometry':Odometry,'/planning/pos_cmd':PositionCommand,
       '/super_debug/control_target':PoseStamped,'/mavros/local_position/pose':PoseStamped,
       '/mavros/state':State}
pubs={name:probe.create_publisher(kind,name,10) for name,kind in types.items()}
settings=dict(speed=.3,acceleration=.5,armed=False,age=0.,flag=1,local_frame='map')


def publish():
    ns=probe.get_clock().now().nanoseconds-int(settings['age']*1e9)
    for name,kind in types.items():
        msg=kind();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1_000_000_000)
        if name=='/Odometry':msg.header.frame_id='camera_init';msg.child_frame_id='body'
        elif name=='/planning/pos_cmd':
            msg.header.frame_id='world';msg.trajectory_flag=settings['flag']
            msg.velocity.x=settings['speed'];msg.acceleration.x=settings['acceleration']
        elif name=='/super_debug/control_target':msg.header.frame_id='px4_enu_preview'
        elif name=='/mavros/local_position/pose':msg.header.frame_id=settings['local_frame']
        else:msg.connected=True;msg.armed=settings['armed']
        pubs[name].publish(msg)


probe.create_timer(.01,publish)
executor=SingleThreadedExecutor(context=ctx);executor.add_node(probe)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
try:
    cases=[({},0),({'speed':.35},2),({'acceleration':.55},2),({'armed':True},2),
           ({'age':.4},2),({'flag':0},2),({'local_frame':'wrong'},2)]
    for change,expected in cases:
        settings.update(speed=.3,acceleration=.5,armed=False,age=0.,flag=1,local_frame='map')
        settings.update(change)
        output=io.StringIO()
        with contextlib.redirect_stdout(output):result=m.monitor(1.5,check_motion=True)
        assert result==expected,(change,result,output.getvalue())
        assert 'sampled planner motion limits' in output.getvalue()
    print('PASS: live DDS sampling accepts bounded fresh references and rejects excessive speed/acceleration, '
          'armed state, stale data, inactive trajectory flags and wrong pose frames.')
finally:
    executor.shutdown();thread.join(timeout=2)
    probe.destroy_node();rclpy.shutdown(context=ctx)
