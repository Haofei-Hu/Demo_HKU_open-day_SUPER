"""Real DDS/parameter-service gates with isolated MAVROS telemetry fixtures."""
import importlib.util
from pathlib import Path
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from mavros_msgs.srv import ParamPull

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('session',ROOT/'scripts/maintenance/super_session.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
context=Context();rclpy.init(context=context)
probe=Node('param',namespace='/mavros',context=context)
for name,value in dict(EKF2_EV_CTRL=0,EKF2_HGT_REF=3,EKF2_EV_POS_X=0.,EKF2_EV_POS_Y=0.,EKF2_EV_POS_Z=0.,EKF2_EV_QMIN=0).items(): probe.declare_parameter(name,value)
state_pub=probe.create_publisher(State,'/mavros/state',10)
ground_pub=probe.create_publisher(ExtendedState,'/mavros/extended_state',10)
local_pub=probe.create_publisher(PoseStamped,'/mavros/local_position/pose',10)
estimator_pub=probe.create_publisher(EstimatorStatus,'/mavros/estimator_status',10)
preview_pub=probe.create_publisher(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov',10)
vision_pub=probe.create_publisher(PoseWithCovarianceStamped,'/mavros/vision_pose/pose_cov',10)
settings=dict(armed=False,connected=True,age=0.,valid=True,frame='map')

def publish():
    state=State(connected=settings['connected'],armed=settings['armed'],mode='AUTO.LOITER',manual_input=False)
    ground=ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
    local=PoseStamped();local.header.frame_id=settings['frame'];local.pose.orientation.w=1.
    estimator=EstimatorStatus(attitude_status_flag=True,velocity_horiz_status_flag=True,
        velocity_vert_status_flag=True,pos_horiz_rel_status_flag=settings['valid'],pos_vert_abs_status_flag=True)
    preview=PoseWithCovarianceStamped();preview.header.frame_id='px4_enu_preview';preview.pose.pose.orientation.w=1.
    vision=PoseWithCovarianceStamped();vision.header.frame_id='px4_enu_preview';vision.pose.pose.orientation.w=1.
    ns=probe.get_clock().now().nanoseconds-int(settings['age']*1e9)
    for msg,publisher in ((state,state_pub),(ground,ground_pub),(local,local_pub),
                          (estimator,estimator_pub),(preview,preview_pub),(vision,vision_pub)):
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
    gate.wait(gate.localization_ready,'valid fixture localization')
    assert gate.messages['state'].mode=='AUTO.LOITER' and not gate.messages['state'].manual_input
    for change in (dict(valid=False),dict(frame='wrong')):
        settings.update(change)
        gate.wait(lambda:not gate.localization_ready(),'invalid fixture localization')
        assert not gate.localization_ready()
        settings.update(valid=True,frame='map')
        gate.wait(gate.localization_ready,'restored fixture localization')
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
    print('PASS: real MAVROS 2.14 ParamPull, typed readback, bounded EV_CTRL writes, armed/disconnected/stale-state rejection; AUTO.LOITER ground localization readiness without RC, invalid estimate/frame rejection.')
finally:
    if gate: gate.close()
    executor.shutdown();thread.join(timeout=2)
    probe.destroy_node();rclpy.shutdown(context=context)
