"""Real scene-capture subprocess with source identities and no native output."""
import base64
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus
from rclpy.serialization import deserialize_message

ROOT=Path(__file__).resolve().parents[1]


def run(native=False):
    rclpy.init();node=Node('demo_scene_fixture')
    kinds={'/Odometry':Odometry,'/super_debug/aligned_pose_cov':PoseWithCovarianceStamped,
        '/mavros/vision_pose/pose_cov':PoseWithCovarianceStamped,'/mavros/local_position/pose':PoseStamped,
        '/mavros/state':State,'/mavros/extended_state':ExtendedState,'/mavros/estimator_status':EstimatorStatus,
        '/rog_map/occ':PointCloud2}
    pubs={topic:node.create_publisher(kind,topic,10) for topic,kind in kinds.items()}
    if native:pubs['writer']=node.create_publisher(PoseStamped,'/mavros/setpoint_position/local',10)
    def publish():
        stamp=node.get_clock().now().to_msg()
        for topic,kind in kinds.items():
            m=kind();m.header.stamp=stamp
            if kind is State:m.connected=True;m.mode='AUTO.LOITER'
            elif kind is ExtendedState:m.landed_state=1
            elif kind is EstimatorStatus:
                m.attitude_status_flag=m.velocity_horiz_status_flag=m.velocity_vert_status_flag=True
                m.pos_horiz_rel_status_flag=m.pos_vert_abs_status_flag=True
            elif kind is PointCloud2:
                m.header.frame_id='world';m.width=1;m.height=1;m.point_step=16;m.row_step=16;m.is_dense=True
                m.fields=[PointField(name=a,offset=4*i,datatype=PointField.FLOAT32,count=1)
                          for i,a in enumerate(('x','y','z','intensity'))]
                m.data=list(struct.pack('<ffff',1.,2.,3.,1.))
            else:
                m.header.frame_id='camera_init' if kind is Odometry else ('map' if kind is PoseStamped else 'px4_enu_preview')
                if kind is Odometry:m.child_frame_id='body'
                p=m.pose if kind is PoseStamped else m.pose.pose;p.orientation.w=1.
            pubs[topic].publish(m)
    node.create_timer(.02,publish);executor=SingleThreadedExecutor();executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);alignment=folder/'alignment.py';alignment.write_text('def convert(p,q,c):\n return p,q,c\n')
            planner=folder/'planner.yaml';planner.write_text('snapshot: source\n');original=planner.read_bytes()
            scene=folder/'scene.json'
            result=subprocess.run([sys.executable,str(ROOT/'scripts/simulation/capture_demo_scene.py'),
                '--output',str(scene),'--alignment-path',str(alignment),'--planner-config',str(planner)],
                capture_output=True,text=True,timeout=25)
            if native:
                assert result.returncode!=0 and 'Native writer present' in result.stderr,result.stderr
                assert not scene.exists()
            else:
                assert result.returncode==0,result.stderr
                data=json.loads(scene.read_text())
                cloud=deserialize_message(base64.b64decode(data['cloud_serialized_base64']),PointCloud2)
                assert cloud.width==1 and cloud.header.frame_id=='world'
                assert data['raw_quaternion']==[0.,0.,0.,1.] and data['origin']==[0.,0.,0.]
                assert Path(data['alignment_path']).read_bytes()==alignment.read_bytes()
                assert planner.read_bytes()==original and not data['native_control_or_flight_requests']
            print('PASS: actual scene-capture subprocess', 'native writer rejected' if native else 'map/paired pose/frozen files',flush=True)
    finally:
        executor.shutdown();thread.join(timeout=2);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':run();run(True)
