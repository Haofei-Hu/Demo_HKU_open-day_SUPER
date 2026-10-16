"""The managed viewer receives raw-world virtual poses without an ENU identity TF."""
import base64
import json
import math
import multiprocessing
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2,PointField

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/simulation'))
from demo_mission_simulation import backend


class Tests(unittest.TestCase):
    def test_rotated_scene_viewer_pose_matches_raw_odometry_with_original_stamp(self):
        self.check_scene(legacy_preview=False)

    def test_legacy_preview_with_module_scope_ros_init_is_not_imported(self):
        self.check_scene(legacy_preview=True)

    def check_scene(self,legacy_preview):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            alignment=folder/('aligned_pose_preview.py' if legacy_preview else 'alignment.py')
            alignment.write_text('import numpy as np\ndef convert(p,q,c):\n'
                ' import math\n'
                ' # Raw IMU yaw=0; body yaw=-45deg; aligned world adds +90deg.\n'
                ' return np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])@p+np.array([1.,2.,3.]),np.array([0.,0.,math.sin(math.pi/8),math.cos(math.pi/8)]),c\n')
            if legacy_preview:
                # Deployed previews may initialize ROS at module scope. Geometry
                # extraction must work without importing or rewriting that file.
                with alignment.open('a') as handle:handle.write('import rclpy\nrclpy.init()\n')
            original_alignment=alignment.read_bytes()
            cloud=PointCloud2();cloud.header.frame_id='world';cloud.width=cloud.height=1
            cloud.point_step=cloud.row_step=12
            cloud.fields=[PointField(name=a,offset=i*4,datatype=PointField.FLOAT32,count=1)
                          for i,a in enumerate(('x','y','z'))]
            cloud.data=list(struct.pack('<fff',1.,2.,3.))
            scene=folder/'scene.json';scene.write_text(json.dumps(dict(alignment_path=str(alignment),
                aligned_position=[4.,5.,6.],raw_quaternion=[0.,0.,0.,1.],
                aligned_quaternion=[0.,0.,math.sin(math.pi/8),math.cos(math.pi/8)],
                origin=[.5,.6,.7],cloud_serialized_base64=base64.b64encode(serialize_message(cloud)).decode())))
            parent,child=multiprocessing.get_context('spawn').Pipe()
            process=multiprocessing.get_context('spawn').Process(target=backend,args=(child,None,scene))
            sys.path.insert(0,str(folder))
            process.start();node=None
            try:
                self.assertTrue(parent.poll(15));self.assertEqual(parent.recv(),'ready')
                rclpy.init();node=rclpy.create_node('demo_viewer_pose_test')
                poses={};odometry={};local={}
                def stamp(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                subs=[node.create_subscription(PoseStamped,'/demo_scene/vehicle_pose',
                      lambda msg:poses.update({stamp(msg):msg}),qos_profile_sensor_data),
                      node.create_subscription(Odometry,'/Odometry',
                      lambda msg:odometry.update({stamp(msg):msg}),qos_profile_sensor_data),
                      node.create_subscription(PoseStamped,'/mavros/local_position/pose',
                      lambda msg:local.update({stamp(msg):msg}),qos_profile_sensor_data)]
                deadline=time.monotonic()+8
                while not poses.keys()&odometry.keys()&local.keys() and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.05)
                common=poses.keys()&odometry.keys()&local.keys();self.assertTrue(common,'Missing paired viewer/odometry pose')
                key=max(common);pose=poses[key];raw=odometry[key]
                self.assertEqual(pose.header.frame_id,'world');self.assertEqual(raw.header.frame_id,'camera_init')
                self.assertEqual(pose.pose.position,raw.pose.pose.position)
                np.testing.assert_allclose([pose.pose.orientation.x,pose.pose.orientation.y,
                    pose.pose.orientation.z,pose.pose.orientation.w],
                    [0.,0.,-math.sin(math.pi/8),math.cos(math.pi/8)],atol=1e-10)
                self.assertEqual(raw.pose.pose.orientation.w,1.)
                self.assertEqual(raw.pose.pose.orientation.z,0.)
                self.assertAlmostEqual(local[key].pose.orientation.z,math.sin(math.pi/8))
                self.assertAlmostEqual(local[key].pose.orientation.w,math.cos(math.pi/8))
                np.testing.assert_allclose([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z],
                                           [3.6,-3.5,3.7],atol=1e-10)
                parent.send('report');self.assertTrue(parent.poll(3));ledger=parent.recv()
                self.assertFalse(ledger['armed']);self.assertEqual(ledger['goals'],[])
            finally:
                if node is not None:node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
                if process.is_alive():parent.send('stop')
                process.join(timeout=5)
                if process.is_alive():process.terminate();process.join(timeout=5)
                parent.close();child.close()
                sys.path.remove(str(folder))
            self.assertEqual(process.exitcode,0)
            self.assertEqual(alignment.read_bytes(),original_alignment)


if __name__=='__main__':unittest.main()
