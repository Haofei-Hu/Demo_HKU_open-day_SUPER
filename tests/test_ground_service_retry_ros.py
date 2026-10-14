"""Isolated DDS service recovery under 50 Hz telemetry; no aircraft connection."""
import importlib.util
from pathlib import Path
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterValue,SetParametersResult
from rcl_interfaces.srv import GetParameters,SetParameters
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('session',ROOT/'scripts/maintenance/super_session.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def run(case):
    context=Context();rclpy.init(context=context)
    probe=Node('service_retry_fixture',context=context)
    group=ReentrantCallbackGroup()
    settings={'armed':False};reads=[];writes=[];gate=None;arm_timer=None
    topics={
        '/mavros/state':State,'/mavros/extended_state':ExtendedState,
        '/mavros/local_position/pose':PoseStamped,'/mavros/estimator_status':EstimatorStatus,
        '/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
        '/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped}
    pubs={topic:probe.create_publisher(kind,topic,10) for topic,kind in topics.items()}

    def publish():
        for topic,kind in topics.items():
            msg=kind();msg.header.stamp=probe.get_clock().now().to_msg()
            if kind is State:msg.connected=True;msg.armed=settings['armed'];msg.mode='AUTO.LOITER'
            elif kind is ExtendedState:msg.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
            elif kind is EstimatorStatus:
                msg.attitude_status_flag=msg.velocity_horiz_status_flag=msg.velocity_vert_status_flag=True
                msg.pos_horiz_rel_status_flag=msg.pos_vert_abs_status_flag=True
            elif kind is PoseStamped:msg.header.frame_id='map';msg.pose.orientation.w=1.
            else:msg.header.frame_id='px4_enu_preview';msg.pose.pose.orientation.w=1.
            pubs[topic].publish(msg)

    def read(request,response):
        reads.append(tuple(request.names));number=len(reads)
        if case in ('delayed_first','armed_during_wait') and number==1:time.sleep(.7)
        if case=='all_timeout':time.sleep(1.5)
        value=99 if case=='delayed_first' and number==1 else 11
        response.values=[ParameterValue(type=2,integer_value=value) for _ in request.names]
        return response

    def write(request,response):
        writes.append(request);time.sleep(1.5)
        response.results=[SetParametersResult(successful=True)]
        return response

    probe.create_timer(.02,publish)
    probe.create_service(GetParameters,'/mavros/param/get_parameters',read,callback_group=group)
    probe.create_service(SetParameters,'/mavros/param/set_parameters',write,callback_group=group)
    executor=MultiThreadedExecutor(num_threads=5,context=context);executor.add_node(probe)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        gate=m.GroundGate();gate.READ_RESPONSE_BUDGET=1.2
        if case=='armed_during_wait':
            arm_timer=threading.Timer(.2,lambda:settings.update(armed=True));arm_timer.start()
        began=time.monotonic()
        if case=='write_timeout':
            try:gate.call(SetParameters,'/mavros/param/set_parameters',SetParameters.Request())
            except RuntimeError as exc:assert '1 attempt(s)' in str(exc),str(exc)
            else:raise AssertionError('Write timeout was not rejected')
            assert len(writes)==1 and not reads,'Write was retried'
        elif case in ('all_timeout','armed_during_wait'):
            try:gate.parameters(['EKF2_EV_CTRL'])
            except RuntimeError as exc:
                if case=='all_timeout':assert '3 attempt(s)' in str(exc),str(exc)
                else:assert 'armed' in str(exc),str(exc)
            else:raise AssertionError('Unsafe/missing reply was accepted')
            assert len(reads)==(3 if case=='all_timeout' else 1)
        else:
            assert gate.parameters(['EKF2_EV_CTRL'])=={'EKF2_EV_CTRL':11}
            assert len(reads)==(2 if case=='delayed_first' else 1)
            assert not writes
        elapsed=time.monotonic()-began
        assert elapsed<1.8,f'Response budget expanded: {elapsed}'
        print(f'PASS: {case}; read requests={len(reads)}, write requests={len(writes)}, elapsed={elapsed:.3f}s',flush=True)
    finally:
        if arm_timer:arm_timer.cancel();arm_timer.join()
        if gate:gate.close()
        executor.shutdown();thread.join(timeout=3)
        probe.destroy_node();rclpy.shutdown(context=context)


if __name__=='__main__':
    for case in ('healthy','delayed_first','all_timeout','write_timeout','armed_during_wait'):
        run(case)
