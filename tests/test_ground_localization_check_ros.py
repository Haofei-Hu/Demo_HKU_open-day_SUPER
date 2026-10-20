"""Private-domain ROS smoke test: mixed QoS, real callbacks, disarmed/source guards."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('ground_check_ros', ROOT/'scripts/analysis/ground_localization_check.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class RosSmoke(unittest.TestCase):
    def test_cli_saves_report_and_bundle(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from mavros_msgs.msg import State
        rclpy.init()
        node = rclpy.create_node('ground_check_cli_fixture')
        types = {m.RAW: Odometry, m.ALIGNED: PoseWithCovarianceStamped,
                 m.VISION: PoseWithCovarianceStamped, m.LOCAL: PoseStamped}
        pubs = {t: node.create_publisher(cls, t, 10) for t, cls in types.items()}
        state_pub = node.create_publisher(State, '/mavros/state', 10)
        process = None
        try:
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory)/'capture'
                process = subprocess.Popen(
                    [sys.executable, str(ROOT/'scripts/analysis/ground_localization_check.py'),
                     '--stationary-only', '--window', '1', '--output', str(output)],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                process.stdin.write('\n')
                process.stdin.flush()
                deadline = time.monotonic()+12
                while process.poll() is None and time.monotonic() < deadline:
                    state = State()
                    state.connected = True
                    state_pub.publish(state)
                    for t, cls in types.items():
                        msg = cls()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.header.frame_id = m.FRAMES[t]
                        pose = msg.pose if t == m.LOCAL else msg.pose.pose
                        pose.orientation.w = 1.
                        pubs[t].publish(msg)
                    rclpy.spin_once(node, timeout_sec=.01)
                self.assertIsNotNone(process.poll(), 'Diagnostic did not finish')
                stdout = process.communicate(timeout=2)[0]
                self.assertEqual(process.returncode, 0, stdout)
                report = json.loads((output/'report.json').read_text())
                self.assertEqual(report['result'], 'CAPTURE_COMPLETE')
                self.assertEqual(report['flight_readiness'], 'NOT_ASSESSED')
                self.assertEqual(len(report['captures']), 1)
                with zipfile.ZipFile(output/'diagnostic.zip') as archive:
                    self.assertIn('samples.jsonl', archive.namelist())
                    self.assertIn('report.json', archive.namelist())
                for t in m.CONTROL_TOPICS:
                    self.assertEqual(node.count_publishers(t), 0)
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()

    def test_observers_and_guards(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from mavros_msgs.msg import State
        rclpy.init()
        publisher = rclpy.create_node('ground_check_fixture')
        types = {m.RAW: Odometry, m.ALIGNED: PoseWithCovarianceStamped,
                 m.VISION: PoseWithCovarianceStamped, m.LOCAL: PoseStamped}
        pubs = {topic: publisher.create_publisher(
            cls, topic, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT
                                  if topic == m.RAW else ReliabilityPolicy.RELIABLE))
                for topic, cls in types.items()}
        state_pub = publisher.create_publisher(State, '/mavros/state', 10)
        with tempfile.TemporaryDirectory() as folder:
            observer = m.Observer(Path(folder))
            def send(armed=False):
                state = State()
                state.connected, state.armed = True, armed
                state_pub.publish(state)
                for topic, cls in types.items():
                    msg = cls()
                    msg.header.stamp = publisher.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_init' if topic == m.RAW else 'map'
                    pose = msg.pose if topic == m.LOCAL else msg.pose.pose
                    pose.position.x = .5
                    pose.orientation.w = 1.
                    pubs[topic].publish(msg)
            try:
                deadline = time.monotonic()+8
                while time.monotonic() < deadline:
                    send()
                    observer.poll()
                    if all(len(observer.rows[(t, 'best_effort')]) >= 3 for t in types) and all(
                            len(observer.rows[(t, 'reliable')]) >= 3 for t in types if t != m.RAW):
                        break
                self.assertTrue(all(observer.rows[(t, 'best_effort')] for t in types))
                self.assertNotIn(m.RAW, observer.reliable_created)
                self.assertEqual(observer.reliable_created, set(types)-{m.RAW})
                observer.wait_ready()
                self.assertTrue(observer.active)
                self.assertGreater(m.qos_comparison(observer.rows[(m.LOCAL, 'best_effort')],
                                                   observer.rows[(m.LOCAL, 'reliable')])['common_stamps'], 0)
                with self.assertRaisesRegex(RuntimeError, 'Armed FC'):
                    deadline = time.monotonic()+3
                    while time.monotonic() < deadline:
                        send(armed=True)
                        observer.poll()
                observer.armed_seen = False
                control = publisher.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
                with self.assertRaisesRegex(RuntimeError, 'Control publishers'):
                    deadline = time.monotonic()+3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(publisher, timeout_sec=.02)
                        observer.graph_check()
                publisher.destroy_publisher(control)
                observer.source_gids[m.LOCAL] = 'different-origin'
                with self.assertRaisesRegex(RuntimeError, 'source changed'):
                    deadline = time.monotonic()+3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(publisher, timeout_sec=.02)
                        try:
                            observer.graph_check()
                        except RuntimeError as exc:
                            if 'Control publishers' not in str(exc):
                                raise
            finally:
                observer.close()
                publisher.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
