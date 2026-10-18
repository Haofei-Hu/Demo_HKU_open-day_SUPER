"""Compare unchanged 15 s collectors sequentially and concurrently on live ROS."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/maintenance')]
import super_session as manager
from super_ground_test import alignment_quaternion


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='194' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Test requires private localhost-only ROS domain 194')
    rclpy.init();node=rclpy.create_node('parallel_alignment_fixture')
    odom=node.create_publisher(Odometry,'/Odometry',10)
    body=node.create_publisher(PoseStamped,'/super_debug/body_pose',10)
    imu=node.create_publisher(Imu,'/mavros/imu/data',10)
    def publish():
        stamp=node.get_clock().now().to_msg()
        raw=Odometry();raw.header.stamp=stamp;raw.header.frame_id='camera_init'
        raw.child_frame_id='body';raw.pose.pose.orientation.w=1.
        pose=PoseStamped();pose.header=raw.header;pose.pose.orientation.w=1.
        inertial=Imu();inertial.header=raw.header
        inertial.orientation.z=math.sin(.3);inertial.orientation.w=math.cos(.3)
        odom.publish(raw);body.publish(pose);imu.publish(inertial)
    timer=node.create_timer(.02,publish)
    executor=SingleThreadedExecutor();executor.add_node(node)
    thread=threading.Thread(target=executor.spin);thread.start()
    durations={};reports={}
    try:
        with tempfile.TemporaryDirectory() as directory:
            for parallel in (False,True):
                name='parallel' if parallel else 'sequential';path=Path(directory)/(name+'.json')
                commands=[[sys.executable,'-u',ROOT/'odom_age_check.py'],
                          [sys.executable,'-u',ROOT/'scripts/localization/frame_alignment_check.py','--output-json',path]]
                began=time.monotonic()
                if parallel:manager.parallel_observations(commands)
                else:
                    for command in commands:subprocess.run(list(map(str,command)),check=True)
                durations[name]=time.monotonic()-began
                report=json.loads(path.read_text());q=alignment_quaternion(report)
                expected=[0.,0.,math.sin(.3),math.cos(.3)]
                assert abs(abs(sum(a*b for a,b in zip(q,expected)))-1)<1e-8,report
                assert report['matched']>=500,report
                assert (report['ros_end_stamp_ns']-report['ros_start_stamp_ns'])/1e9>=14.9
                reports[name]={k:report[k] for k in ('matched','rotation_spread_max_deg','body_age_max_s')}
            assert durations['sequential']-durations['parallel']>12.,durations
            result=dict(scope='Real ROS with synthetic stationary sensor publishers; no actual hardware',
                        sampling_duration_s=15.,durations_s=durations,saved_s=durations['sequential']-durations['parallel'],
                        alignment_evidence=reports,result='PASS')
            if len(sys.argv)>1:Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result,indent=2),flush=True)
    finally:
        executor.shutdown();thread.join();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
