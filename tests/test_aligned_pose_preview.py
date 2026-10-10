import importlib.util
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import numpy as np

def main():
    path = Path(__file__).resolve().parents[1] / 'scripts/localization/aligned_pose_preview.py'
    spec = importlib.util.spec_from_file_location('preview', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    p = np.array([.7, -.2, .3])
    q = np.array([.1, -.2, .3, .9]); q /= np.linalg.norm(q)
    cov = np.diag([.01, .02, .03, .004, .005, .006])
    base, orientation, converted_cov = m.convert(p, q, cov)
    lever = -m.rotation(m.multiply(q, m.Q_IB)) @ m.T_BI
    expected = np.eye(6); expected[:3, 3:] = -m.skew(lever)
    expected = m.AXES @ expected
    numeric = np.zeros((3, 6))
    eps = 1e-7
    for i in range(6):
        dp = p.copy(); dq = q.copy()
        if i < 3:
            dp[i] += eps
        else:
            delta = np.array([0., 0., 0., 1.])
            delta[i-3] = eps/2
            delta /= np.linalg.norm(delta)
            dq = m.multiply(delta, q)
        moved, _, _ = m.convert(dp, dq, cov)
        numeric[:, i] = (moved-base)/eps
    np.testing.assert_allclose(numeric, expected[:3], atol=1e-8)
    np.testing.assert_allclose(converted_cov, expected @ cov @ expected.T)
    assert np.linalg.eigvalsh(converted_cov).min() > 0
    np.testing.assert_allclose(m.rotation(orientation), m.R_WC @ m.rotation(q) @ m.rotation(m.Q_IB), atol=1e-12)
    print('Pose composition, lever-arm Jacobian and covariance checks passed.', flush=True)

    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rclpy.init()
    node = rclpy.create_node('isolated_preview_test')
    pub = node.create_publisher(Odometry, '/Odometry', 10)
    received = []
    sub = node.create_subscription(PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov', received.append, 10)
    sent = {}
    log = Path('/tmp/aligned-preview-test.log').open('w')
    process = subprocess.Popen([sys.executable, str(path)], stdout=log, stderr=subprocess.STDOUT)

    def send(x):
        msg = Odometry()
        t = node.get_clock().now().nanoseconds - 1500000000
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(t, 1000000000)
        msg.header.frame_id = 'camera_init'; msg.child_frame_id = 'body'
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.w = 1.
        msg.pose.covariance = cov.ravel().tolist()
        sent[t] = x
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.05)

    try:
        deadline = time.monotonic() + 10
        while not received and time.monotonic() < deadline:
            send(0.)
        assert received, 'No preview received'
        np.testing.assert_allclose([received[0].pose.pose.position.x, received[0].pose.pose.position.y, received[0].pose.pose.position.z], 0., atol=1e-12)
        for _ in range(30):
            send(.2)
        selected = [msg for msg in received if sent.get(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec) == .2]
        assert selected, 'No translated preview received'
        msg = selected[-1]
        assert msg.header.frame_id == 'px4_enu_preview'
        np.testing.assert_allclose([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], m.R_WC @ [.2, 0., 0.], atol=1e-12)
        np.testing.assert_allclose(msg.pose.covariance, m.convert(np.zeros(3), np.array([0., 0., 0., 1.]), cov)[2].ravel(), atol=1e-12)
        assert not node.count_publishers('/mavros/vision_pose/pose_cov')
        print('Isolated Humble ROS test passed: original stale stamp preserved, zero origin, transformed pose/covariance, no MAVROS publisher.')
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        log.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
