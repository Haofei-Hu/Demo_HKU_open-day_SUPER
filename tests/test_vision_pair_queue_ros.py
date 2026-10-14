"""Real DDS regression for exact-stamp join starvation; isolated from aircraft.

The aligned stream trails raw by one sample. A short callback pause makes
depth-one readers retain different stamps even though both inputs stay fresh.
Compare that original queue against the production bounded paired queue.
"""
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

from flight_pose_cov_transport import FlightPoseCovTransport, DEBUG_TOPIC, PARAMETERS


def run(depth):
    rclpy.init()
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'alignment.py'
        path.write_text('def convert(p,q,covariance):\n    return p,q,covariance\n')
        probe=Node('paired_queue_fixture')
        executor=SingleThreadedExecutor();executor.add_node(probe)
        raw_pub=probe.create_publisher(Odometry,'/Odometry',32)
        aligned_pub=probe.create_publisher(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov',32)
        state_pub=probe.create_publisher(State,'/mavros/state',10)
        outputs=[];raw_seen=[];aligned_seen=[];inputs={}
        probe.create_subscription(PoseWithCovarianceStamped,DEBUG_TOPIC,
                                  lambda msg:outputs.append((time.monotonic(),msg)),32)
        probe.create_subscription(Odometry,'/Odometry',raw_seen.append,32)
        probe.create_subscription(PoseWithCovarianceStamped,'/super_debug/aligned_pose_cov',aligned_seen.append,32)

        def parameters(request,response):
            assert request.names==PARAMETERS
            response.values=[ParameterValue(type=2,integer_value=11),
                             ParameterValue(type=2,integer_value=3),
                             *[ParameterValue(type=3,double_value=0.) for _ in range(3)],
                             ParameterValue(type=2,integer_value=0)]
            return response
        probe.create_service(GetParameters,'/mavros/param/get_parameters',parameters)

        def state():
            msg=State(connected=True,armed=False,mode='POSCTL')
            msg.header.stamp=probe.get_clock().now().to_msg();state_pub.publish(msg)
        probe.create_timer(.05,state)

        class QueueVariant(FlightPoseCovTransport):
            PAIR_QUEUE_DEPTH=depth
        bridge=QueueVariant(parameter_overrides=[Parameter('alignment_path',value=str(path))])
        executor.add_node(bridge)

        def spin(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:executor.spin_once(timeout_sec=.001)

        def raw(ns):
            msg=Odometry();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1_000_000_000)
            msg.header.frame_id='camera_init';msg.child_frame_id='body'
            msg.pose.pose.orientation.w=1.;raw_pub.publish(msg)

        def aligned(ns):
            msg=PoseWithCovarianceStamped();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1_000_000_000)
            msg.header.frame_id='px4_enu_preview';msg.pose.pose.orientation.w=1.
            msg.pose.covariance=[1e-5 if i%7==0 else 0. for i in range(36)]
            inputs[ns]=msg;aligned_pub.publish(msg)

        def stamp(msg):return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

        try:
            deadline=time.monotonic()+8
            while len(outputs)<3:
                assert time.monotonic()<deadline,'Initialization timed out'
                ns=probe.get_clock().now().nanoseconds
                raw(ns);aligned(ns);spin(.04)
            # One raw sample remains ahead of its aligned counterpart.
            previous=probe.get_clock().now().nanoseconds
            raw(previous);spin(.01)
            before=len(outputs);raw_before=len(raw_seen);aligned_before=len(aligned_seen)
            began=time.monotonic()
            for _ in range(18):
                # 3 samples per 60 ms: nominal 50 Hz streams, 5 ms pause.
                ns=probe.get_clock().now().nanoseconds
                stamps=[ns-40_000_000,ns-20_000_000,ns]
                for value in stamps:raw(value)
                for value in [previous,*stamps[:2]]:aligned(value)
                previous=stamps[-1]
                time.sleep(.005)  # Let DDS fill histories before servicing callbacks.
                spin(.055)
            during=outputs[before:]
            assert len(raw_seen)-raw_before>=50 and len(aligned_seen)-aligned_before>=50
            assert bridge.fault is None
            if depth==1:
                assert not during, f'Expected depth-one join starvation, got {len(during)} poses'
                gap=time.monotonic()-outputs[-1][0]
                assert gap>.25
                print(f'REPRODUCED: depth=1; fresh input bursts, zero paired output for {gap:.3f}s',flush=True)
            else:
                assert len(during)>=40,f'Paired queue lost too many samples: {len(during)}'
                gaps=[b[0]-a[0] for a,b in zip(during,during[1:])]
                assert max(gaps)<.25 and during[0][0]-began<.25
                assert all(stamp(b)>stamp(a) for (_,a),(_,b) in zip(during,during[1:]))
                assert all(msg==inputs[stamp(msg)] for _,msg in during),'Relay changed an input stamp/pose/covariance'
                # Larger histories must never make stale measurements publishable.
                spin(.1);count=len(outputs)
                old=probe.get_clock().now().nanoseconds-400_000_000
                raw(old);aligned(old);spin(.1)
                assert len(outputs)==count,'Stale retained measurement was published'
                print(f'PASS: depth={depth}; {len(during)} unchanged paired poses; '
                      f'max output gap={max(gaps):.3f}s; stale inputs rejected',flush=True)
        finally:
            executor.shutdown();bridge.destroy_node();probe.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    run(1)
    run(FlightPoseCovTransport.PAIR_QUEUE_DEPTH)
