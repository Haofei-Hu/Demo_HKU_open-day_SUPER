"""Live DDS preparation waits; temporal recovery cannot hide source/frame faults."""
import contextlib
import importlib.util
import io
from pathlib import Path
import signal
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('metrics',ROOT/'scripts/analysis/control_chain_age_check.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def run(case,seconds=2.):
    ctx=Context();rclpy.init(context=ctx);node=Node('stable_vision_fixture',context=ctx)
    kinds={'/Odometry':Odometry,'/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
           '/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped,'/mavros/local_position/pose':PoseStamped}
    pubs={topic:node.create_publisher(kind,topic,10) for topic,kind in kinds.items()}
    began=time.monotonic()
    def publish():
        elapsed=time.monotonic()-began
        age=.36 if case=='startup_old' and elapsed<.9 else 0.
        if case=='mid_header':
            age=.36*max(0.,elapsed-.6)/.7 if elapsed<1.3 else .36 if elapsed<1.5 else 0.
        ns=node.get_clock().now().nanoseconds-int(age*1e9)
        for topic,kind in kinds.items():
            if case=='mid_gap' and topic=='/mavros/vision_pose/pose_cov' and .8<elapsed<1.2:continue
            msg=kind();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1_000_000_000)
            msg.header.frame_id=('camera_init' if kind is Odometry else
                                 'map' if kind is PoseStamped else 'px4_enu_preview')
            if kind is Odometry:
                msg.child_frame_id='body'
                if case=='wrong_frame' and elapsed>1.:msg.header.frame_id='wrong'
            pubs[topic].publish(msg)
    node.create_timer(.02,publish)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        def timeout(*_):raise RuntimeError('Fixture did not finish within its test budget')
        previous=signal.signal(signal.SIGALRM,timeout);signal.alarm(int(seconds)+15)
        output=io.StringIO()
        with contextlib.redirect_stdout(output):result=m.monitor(seconds,vision_chain=True,wait_stable=True)
        elapsed=time.monotonic()-began;text=output.getvalue()
        assert result==(2 if case=='wrong_frame' else 0),text
        assert 'Lifetime acquisition statistics' in text,text
        if case=='startup_old':assert elapsed>=seconds+.8 and 'header_age' in text,text
        if case=='mid_gap':assert elapsed>=seconds+1. and 'receive_gap' in text,text
        if case=='mid_header':assert elapsed>=seconds+1.4 and 'header_age' in text,text
        if case=='wrong_frame':assert 'unexpected frames' in text,text
        print(f'PASS: stable vision {case}; result={result}; observed duration={elapsed:.3f}s',flush=True)
    finally:
        signal.alarm(0);signal.signal(signal.SIGALRM,previous)
        executor.shutdown();thread.join(timeout=2);node.destroy_node();rclpy.shutdown(context=ctx)


if __name__=='__main__':
    for case in ('startup_old','mid_gap','mid_header','wrong_frame'):run(case)
