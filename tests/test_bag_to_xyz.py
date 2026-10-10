import argparse
import json
import math
import struct
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField

from bag_to_xyz import cloud_xyz, export, trajectory_report


def stamp(msg, seconds):
    ns = int(round(seconds * 1e9))
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    msg.header.frame_id = "camera_init"


def cloud(big=True, shift=0):
    msg = PointCloud2()
    msg.height, msg.width = 2, 2
    msg.point_step, msg.row_step = 20, 44
    msg.is_bigendian = big
    msg.fields = [PointField(name=name, offset=offset, datatype=7, count=1)
                  for name, offset in [("x", 8), ("y", 0), ("z", 12)]]
    points = [[1+shift, 2, 3], [1000+shift, -1000, 5],
              [float("nan"), 0, 0], [4+shift, 5, 6]]
    raw = bytearray(88)
    for i, (x, y, z) in enumerate(points):
        offset = (i // 2) * 44 + (i % 2) * 20
        for off, value in [(8, x), (0, y), (12, z)]:
            struct.pack_into((">" if big else "<") + "f", raw, offset+off, value)
    msg.data = bytes(raw)
    return msg


class ExportTests(unittest.TestCase):
    def test_padded_cloud_both_endians_keeps_spike(self):
        for big in (True, False):
            points = cloud_xyz(cloud(big))
            np.testing.assert_allclose(points, [[1, 2, 3], [1000, -1000, 5], [4, 5, 6]])

    def test_truncated_cloud_rejected(self):
        msg = cloud()
        msg.data = msg.data[:40]
        with self.assertRaisesRegex(ValueError, "truncated"):
            cloud_xyz(msg)

    def test_overlapping_closure_windows_rejected(self):
        rows = [[i, i, 0, 0, 0, 0, 0, 0, 1] for i in range(5)]
        self.assertFalse(trajectory_report(rows, 5, {"camera_init"})["closure_available"])

    def test_frame_change_cannot_report_closure(self):
        rows = [[i, i, 0, 0, 0, 0, 0, 0, 1] for i in range(30)]
        self.assertFalse(trajectory_report(rows, 5, {"a", "b"})["closure_available"])

    def test_required_message_import_failure_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bag, out = root / "fixture", root / "xyz"
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                        rosbag2_py.ConverterOptions("", ""))
            for name, typ in [("/Odometry", "nav_msgs/msg/Odometry"),
                              ("/cloud_registered", "sensor_msgs/msg/PointCloud2")]:
                writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ,
                                                           serialization_format="cdr"))
            writer.write("/Odometry", serialize_message(Odometry()), 1_700_000_000_000_000_000)
            writer.write("/cloud_registered", serialize_message(cloud()), 1_700_000_000_000_000_000)
            del writer
            args = argparse.Namespace(bag=bag, output=out, cloud_topic="/cloud_registered",
                odom_topic="/Odometry", storage="sqlite3", window=5, gap=.5)
            with patch("rosidl_runtime_py.utilities.get_message", side_effect=ImportError("missing package")):
                with self.assertRaisesRegex(ValueError, "Cannot decode required topic"):
                    export(args)
            self.assertFalse(out.exists())

    def test_real_sqlite_bag_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bag, out = root / "fixture", root / "xyz"
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                        rosbag2_py.ConverterOptions("", ""))
            for name, typ in [("/Odometry", "nav_msgs/msg/Odometry"),
                              ("/cloud_registered", "sensor_msgs/msg/PointCloud2"),
                              ("/livox/imu", "sensor_msgs/msg/Imu")]:
                writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ,
                                                           serialization_format="cdr"))
            base = 1_700_000_000
            for i in range(270):
                if 80 <= i < 90:
                    continue  # Known one-second data dropout.
                t = base + i/10
                if i < 50:
                    position = [.1, .2, .3]
                elif i < 200:
                    position = [.1 + math.sin((i-50)/150*math.pi), .2, .3]
                else:
                    position = [.14, .17, .32]
                odom = Odometry()
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
                odom.pose.pose.orientation.w = -1.0 if i % 2 else 1.0
                stamp(odom, t - (5 if i == 151 else 0))  # Detect reset/non-monotonic stamp.
                pointcloud = cloud(big=bool(i % 2), shift=position[0] - .1)
                stamp(pointcloud, t)
                imu = Imu()
                stamp(imu, t)
                ns = int(round((t + .025) * 1e9))
                for topic, msg in [("/Odometry", odom), ("/cloud_registered", pointcloud), ("/livox/imu", imu)]:
                    writer.write(topic, serialize_message(msg), ns)
            del writer
            args = argparse.Namespace(bag=bag, output=out, cloud_topic="/cloud_registered",
                odom_topic="/Odometry", storage="sqlite3", window=5, gap=.5)
            export(args)
            report = json.loads((out / "report.json").read_text())
            self.assertEqual(report["cloud_frames"], 260)
            self.assertEqual(report["points"], 780)
            np.testing.assert_allclose(report["trajectory"]["return_delta_xyz_m"], [.04, -.03, .02], atol=1e-7)
            self.assertAlmostEqual(report["trajectory"]["return_error_3d_m"], math.sqrt(.0029), places=7)
            self.assertAlmostEqual(report["trajectory"]["return_yaw_delta_deg"], 0)
            self.assertEqual(report["topics"]["/Odometry"]["record_gaps_over_threshold"], 1)
            self.assertEqual(report["topics"]["/Odometry"]["non_increasing_header_stamps"], 1)
            self.assertGreater(report["topics"]["/Odometry"]["max_header_gap_s"], 5)
            self.assertEqual(np.loadtxt(out / "cloud_all.xyz").shape, (780, 3))
            self.assertGreater(np.loadtxt(out / "cloud_all.xyz")[:, 0].max(), 1000)
            self.assertEqual(np.loadtxt(out / "trajectory.xyz").shape, (260, 3))
            self.assertEqual(np.loadtxt(out / "cloud_end.xyz").shape, (153, 3))
            with self.assertRaises(FileExistsError):
                export(args)


if __name__ == "__main__":
    unittest.main()
