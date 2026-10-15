"""Real survey subprocess captures a new quaternion and refuses stale output."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/localization/frame_alignment_check.py'


def run(case):
    ctx=Context();rclpy.init(context=ctx);probe=Node('alignment_report_fixture',context=ctx)
    body=probe.create_publisher(PoseStamped,'/super_debug/body_pose',10)
    imu=probe.create_publisher(Imu,'/mavros/imu/data',10)
    def publish():
        pose=PoseStamped();pose.header.stamp=probe.get_clock().now().to_msg()
        pose.header.frame_id='camera_init';pose.pose.orientation.w=1.
        msg=Imu();msg.header=pose.header
        msg.orientation.z=math.sin(.3);msg.orientation.w=math.cos(.3)
        if case=='insufficient':msg.orientation_covariance[0]=-1.
        imu.publish(msg);body.publish(pose)
    probe.create_timer(.02,publish)
    executor=SingleThreadedExecutor(context=ctx);executor.add_node(probe)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'alignment.json'
            command=[sys.executable,str(SCRIPT),'--seconds','5','--output-json',str(path)]
            result=subprocess.run(command,capture_output=True,text=True,timeout=12)
            report=json.loads(path.read_text())
            if case=='healthy':
                assert result.returncode==0,result.stderr
                assert report['estimate_available'] and report['matched']>=100,report
                expected=[0.,0.,math.sin(.3),math.cos(.3)]
                assert abs(sum(a*b for a,b in zip(expected,report['quaternion_xyzw']))-1)<1e-8,report
                assert report['body_age_max_s']<.25,report
            else:
                assert result.returncode==2 and not report['estimate_available'],report
                assert 'quaternion_xyzw' not in report
            before=path.read_bytes()
            again=subprocess.run(command,capture_output=True,text=True,timeout=5)
            assert again.returncode==2 and path.read_bytes()==before
            print('PASS: live alignment report',case,'fresh output and overwrite refusal',flush=True)
    finally:
        executor.shutdown();thread.join(timeout=2);probe.destroy_node();rclpy.shutdown(context=ctx)


if __name__=='__main__':
    for case in ('healthy','insufficient'):run(case)
