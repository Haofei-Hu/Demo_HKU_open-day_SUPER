"""Real DDS/parameter-service gates with isolated MAVROS telemetry fixtures."""
import importlib.util
from pathlib import Path
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from mavros_msgs.msg import State,ExtendedState
from mavros_msgs.srv import ParamPull

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('session',ROOT/'scripts/maintenance/super_session.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
context=Context();rclpy.init(context=context)
probe=Node('param',namespace='/mavros',context=context)
for name,value in dict(EKF2_EV_CTRL=0,EKF2_HGT_REF=3,EKF2_EV_POS_X=0.,EKF2_EV_POS_Y=0.,EKF2_EV_POS_Z=0.,EKF2_EV_QMIN=0).items(): probe.declare_parameter(name,value)
state_pub=probe.create_publisher(State,'/mavros/state',10)
ground_pub=probe.create_publisher(ExtendedState,'/mavros/extended_state',10)
settings=dict(armed=False,connected=True,age=0.)

def publish():
    state=State(connected=settings['connected'],armed=settings['armed'],mode='POSCTL')
    ground=ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
    ns=probe.get_clock().now().nanoseconds-int(settings['age']*1e9)
    for msg,publisher in ((state,state_pub),(ground,ground_pub)):
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1_000_000_000)
        publisher.publish(msg)


def pull(request,response):
    assert request.force_pull
    response.success=True;response.param_received=6
    return response

probe.create_timer(.05,publish)
probe.create_service(ParamPull,'/mavros/param/pull',pull)
executor=SingleThreadedExecutor(context=context);executor.add_node(probe)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
gate=None
try:
    gate=m.GroundGate();gate.pull()
    expected=['EKF2_EV_CTRL','EKF2_HGT_REF','EKF2_EV_POS_X','EKF2_EV_QMIN']
    assert gate.parameters(expected)==dict(EKF2_EV_CTRL=0,EKF2_HGT_REF=3,EKF2_EV_POS_X=0.,EKF2_EV_QMIN=0)
    gate.set_ctrl(11);gate.set_ctrl(0)
    assert probe.get_parameter('EKF2_EV_CTRL').value==0
    for change in (dict(armed=True),dict(connected=False),dict(age=3.)):
        settings.update(armed=False,connected=True,age=0.);settings.update(change)
        gate.wait(lambda:not gate.grounded(),'invalid fixture state')
        try: gate.set_ctrl(11)
        except RuntimeError: pass
        else: raise AssertionError('Unsafe state permitted parameter write')
        assert probe.get_parameter('EKF2_EV_CTRL').value==0
        settings.update(armed=False,connected=True,age=0.)
        gate.wait(gate.grounded,'valid fixture state')
    print('PASS: real MAVROS 2.14 ParamPull, typed readback, bounded EV_CTRL writes, armed/disconnected/stale-state rejection.')
finally:
    if gate: gate.close()
    executor.shutdown();thread.join(timeout=2)
    probe.destroy_node();rclpy.shutdown(context=context)
