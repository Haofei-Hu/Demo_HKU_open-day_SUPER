"""Headless desktop regression: a real mouse click publishes an RViz point."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import yaml

from calibrate_venue_rviz import rviz_config


def main():
    rclpy.init();node=Node('point_picker_desktop_test');received=[];viewer=None
    pub=node.create_publisher(PointCloud2,'/super_debug/venue_calibration_cloud',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(PointStamped,'/super_venue/clicked_point',received.append,10)
    axis=np.arange(-5.,5.,.025,dtype=np.float32);x,y=np.meshgrid(axis,axis)
    points=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size,dtype=np.float32)])
    header=Header();header.frame_id='camera_init';header.stamp=node.get_clock().now().to_msg()
    pub.publish(point_cloud2.create_cloud_xyz32(header,points))
    try:
        with tempfile.TemporaryDirectory() as folder:
            config=rviz_config()
            config['Window Geometry']={'Width':1200,'Height':800,'X':0,'Y':0}
            path=Path(folder)/'select.rviz';path.write_text(yaml.safe_dump(config))
            with (Path(folder)/'rviz.log').open('w') as log:
                viewer=subprocess.Popen(['rviz2','-d',str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                began=time.monotonic();window=None;clicked=False;ready=None
                while time.monotonic()-began<20 and not received:
                    rclpy.spin_once(node,timeout_sec=.05)
                    if viewer.poll() is not None:raise AssertionError((Path(folder)/'rviz.log').read_text())
                    if ready and time.monotonic()-ready>3:
                        found=subprocess.run(['xdotool','search','--all','--onlyvisible','--pid',str(viewer.pid),
                                              '--name','RViz'],
                            capture_output=True,text=True)
                        if found.returncode==0:window=found.stdout.splitlines()[-1]
                    if pub.get_subscription_count() and ready is None:ready=time.monotonic()
                    if window and ready and time.monotonic()-ready>3 and not clicked:
                        subprocess.run(['xdotool','windowsize',window,'1200','800'],check=True)
                        subprocess.run(['xdotool','mousemove','--window',window,'750','400','click','1'],check=True)
                        clicked=True
                assert clicked,'RViz window/cloud never became available'
                assert received,'Default tool did not publish a point from the real mouse click'
                msg=received[0]
                assert msg.header.frame_id=='camera_init'
                assert abs(msg.point.z)<.03,(msg.point.x,msg.point.y,msg.point.z)
                print('PASS real RViz default Publish Point tool: mouse click produced camera_init point on frozen cloud')
    finally:
        if viewer is not None and viewer.poll() is None:
            os.killpg(viewer.pid,signal.SIGINT)
            try:viewer.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(viewer.pid,signal.SIGKILL);viewer.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
