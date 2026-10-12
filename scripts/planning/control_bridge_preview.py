#!/usr/bin/env python3
"""Disarmed planner-to-FC-centre coordinate preview; no MAVROS output.

For SUPER 2ad3419: planner 'world' numerically aliases FAST_LIO camera_init.
Load the running session's aligned_pose_preview.py, not an archived quaternion.
Infer its fixed origin from identical-stamp raw/aligned poses. Publish converted
regular/backup commands only to /super_debug/control_target. No flight control,
completion hold, laboratory bounds or independent mounting validation provided.
"""
import math
from pathlib import Path
import time
import numpy as np


def arrays(pose):
    p, q = pose.position, pose.orientation
    position = np.array([p.x, p.y, p.z])
    quaternion = np.array([q.x, q.y, q.z, q.w])
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)):
        raise ValueError('Nonfinite pose')
    norm = np.linalg.norm(quaternion)
    if abs(norm - 1) > .02:
        raise ValueError('Invalid pose quaternion')
    return position, quaternion / norm


def quaternion_rpy(roll, pitch, yaw):
    if not all(math.isfinite(v) for v in (roll, pitch, yaw)):
        raise ValueError('Nonfinite desired attitude')
    sr, sp, sy = np.sin(np.array([roll, pitch, yaw]) / 2)
    cr, cp, cy = np.cos(np.array([roll, pitch, yaw]) / 2)
    return np.array([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


class Mapping:
    def __init__(self, converter):
        self.converter = converter
        self.origin = None
        self.pairs = 0
        self.failed = False

    def pair(self, raw_pose, aligned_pose):
        p, q = arrays(raw_pose)
        observed_p, observed_q = arrays(aligned_pose)
        predicted_p, predicted_q, _ = self.converter(p, q, np.eye(6))
        angle = 2 * math.acos(float(np.clip(abs(predicted_q @ observed_q), 0, 1)))
        candidate = predicted_p - observed_p
        if (angle > math.radians(.5) or
                (self.origin is not None and np.linalg.norm(candidate-self.origin) > .005)):
            self.failed = True
            raise ValueError('Session rotation/origin mismatch; restart preview after calibration')
        if self.origin is None:
            self.origin = candidate.copy()
        self.pairs += 1

    def command(self, msg):
        if self.failed or self.origin is None or self.pairs < 3:
            raise ValueError('Coordinate mapping not ready')
        if msg.header.frame_id != 'world' or msg.trajectory_flag not in (1, 2):
            raise ValueError('Unexpected planner frame or inactive/invalid trajectory flag')
        p = np.array([msg.position.x, msg.position.y, msg.position.z])
        if not np.all(np.isfinite(p)) or not math.isfinite(msg.yaw):
            raise ValueError('Nonfinite command')
        # The confirmed publisher supplies flat-output attitude in camera_init.
        q = quaternion_rpy(msg.attitude.x, msg.attitude.y, msg.attitude.z)
        position, orientation, _ = self.converter(p, q, np.eye(6))
        return position-self.origin, orientation


def load_converter(path):
    # Older deployed previews start ROS at module scope. Read only geometry.
    import ast
    tree = ast.parse(Path(path).read_text())

    def root(target):
        while isinstance(target, (ast.Subscript, ast.Attribute)):
            target = target.value
        return target.id if isinstance(target, ast.Name) else None

    def names(node):
        if isinstance(node, ast.FunctionDef):
            return {node.name}
        if isinstance(node, ast.Assign):
            return {root(target) for target in node.targets} - {None}
        if isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            return {root(node.target)} - {None}
        return set()

    statements = [node for node in tree.body if names(node)]
    has_convert = any(isinstance(node, ast.FunctionDef) and node.name == 'convert'
                      for node in statements)
    needed = {'convert'} if has_convert else {'Q_WC', 'R_WC', 'Q_IB', 'T_BI'}
    while True:
        previous = needed.copy()
        for node in statements:
            if names(node) & needed:
                needed.update(child.id for child in ast.walk(node)
                              if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load))
        if needed == previous:
            break
    selected = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            aliases = [a for a in node.names if (a.asname or a.name.split('.')[0]) in needed]
            if any(a.name.split('.')[0] not in ('math', 'numpy') for a in aliases):
                raise ValueError('Geometry requires an unsupported import')
            if aliases:
                selected.append(ast.copy_location(ast.Import(names=aliases), node))
        elif isinstance(node, ast.ImportFrom):
            aliases = [a for a in node.names if (a.asname or a.name) in needed]
            if aliases:
                if (node.module or '').split('.')[0] not in ('math', 'numpy'):
                    raise ValueError('Geometry requires an unsupported import')
                selected.append(ast.copy_location(ast.ImportFrom(
                    module=node.module, names=aliases, level=node.level), node))
        elif names(node) & needed:
            selected.append(node)
    namespace = {'__name__': 'session_geometry'}
    geometry = ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[]))
    exec(compile(geometry, str(path), 'exec'), namespace)
    if has_convert:
        return namespace['convert']
    # Some older scripts inline conversion in their ROS callback.
    if not all(name in namespace for name in ('Q_WC', 'R_WC', 'Q_IB', 'T_BI')):
        raise ValueError('Preview lacks required session geometry constants')

    def multiply(a, b):
        x, y, z, w = a
        X, Y, Z, W = b
        return np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                         w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])

    def rotation(q):
        x, y, z, w = q
        return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

    def convert(p, q, covariance):
        body_q = multiply(q, namespace['Q_IB'])
        p = namespace['R_WC'] @ (p-rotation(body_q) @ namespace['T_BI'])
        return p, multiply(namespace['Q_WC'], body_q), covariance

    return convert


def node_type(*, node_name='control_bridge_preview', output_topic='/super_debug/control_target'):
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from mavros_msgs.msg import State
    from mars_quadrotor_msgs.msg import PositionCommand

    class ControlPreview(Node):
        def __init__(self):
            super().__init__(node_name)
            self.declare_parameter('alignment_path', str(Path.home() / 'Downloads/aligned_pose_preview.py'))
            self.declare_parameter('command_topic', '/planning/pos_cmd')
            self.declare_parameter('max_age_seconds', .25)
            self.max_age = float(self.get_parameter('max_age_seconds').value)
            if not math.isfinite(self.max_age) or self.max_age <= 0:
                raise ValueError('max_age_seconds must be finite and positive')
            self.mapping = Mapping(load_converter(Path(self.get_parameter('alignment_path').value)))
            self.raw, self.aligned = {}, {}
            self.last_pair_stamp = 0
            self.arm_seen = False
            self.state_at = -math.inf
            self.state_stamp = 0
            self.connected_disarmed = False
            self.last_command = None
            self.last_warning = -math.inf
            self.publisher = self.create_publisher(PoseStamped, output_topic, 10)
            self.inputs = [
                self.create_subscription(Odometry, '/Odometry', self.raw_received, qos_profile_sensor_data),
                self.create_subscription(PoseWithCovarianceStamped, '/super_debug/aligned_pose_cov',
                                         self.aligned_received, qos_profile_sensor_data),
                self.create_subscription(State, '/mavros/state', self.state_received, qos_profile_sensor_data),
                self.create_subscription(PositionCommand, self.get_parameter('command_topic').value,
                                         self.command_received, qos_profile_sensor_data)]
            self.timer = self.create_timer(1.0, self.report)
            self.get_logger().info('DISARMED CONTROL PREVIEW: output '+output_topic)

        @staticmethod
        def stamp(msg):
            return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        def fresh(self, stamp, limit=None):
            age = (self.get_clock().now().nanoseconds-stamp) / 1e9
            return stamp > 0 and -.05 <= age <= (self.max_age if limit is None else limit)

        def state_received(self, msg):
            self.arm_seen = self.arm_seen or msg.armed
            self.state_at = time.monotonic()
            self.state_stamp = self.stamp(msg)
            self.connected_disarmed = msg.connected and not msg.armed and self.fresh(self.state_stamp, 2)

        def raw_received(self, msg):
            if msg.header.frame_id == 'camera_init' and msg.child_frame_id == 'body':
                self.store(self.raw, msg, msg.pose.pose)

        def aligned_received(self, msg):
            if msg.header.frame_id == 'px4_enu_preview':
                self.store(self.aligned, msg, msg.pose.pose)

        def store(self, bucket, msg, pose):
            stamp = self.stamp(msg)
            if stamp <= self.last_pair_stamp or not self.fresh(stamp):
                return
            bucket[stamp] = pose
            while len(bucket) > 24:
                del bucket[min(bucket)]
            if stamp in self.raw and stamp in self.aligned:
                try:
                    self.mapping.pair(self.raw.pop(stamp), self.aligned.pop(stamp))
                    self.last_pair_stamp = stamp
                except ValueError as exc:
                    self.warn(str(exc))

        def ready(self):
            return (not self.arm_seen and self.connected_disarmed
                    and time.monotonic()-self.state_at <= 2
                    and self.fresh(self.state_stamp, 2)
                    and not self.mapping.failed and self.mapping.pairs >= 3
                    and self.fresh(self.last_pair_stamp))

        def warn(self, reason):
            if time.monotonic()-self.last_warning > 3:
                self.get_logger().warning(reason)
                self.last_warning = time.monotonic()

        def command_received(self, msg):
            stamp = self.stamp(msg)
            if not self.ready() or not self.fresh(stamp):
                self.warn('Waiting for fresh disarmed state, paired localization and command')
                return
            if self.last_command is not None and stamp <= self.last_command[0]:
                return
            try:
                p, q = self.mapping.command(msg)
            except ValueError as exc:
                self.warn(str(exc))
                return
            out = PoseStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = 'px4_enu_preview'
            out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, p)
            out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = map(float, q)
            self.publisher.publish(out)
            self.last_command = (stamp, msg.trajectory_flag, p, q)

        def report(self):
            if not self.ready():
                text = ('BLOCKED: session alignment mismatch' if self.mapping.failed else
                        'WAIT: fresh disarmed state and three matched localization samples')
            elif self.last_command is None or not self.fresh(self.last_command[0]):
                text = 'READY coordinates; WAIT: fresh /planning/pos_cmd'
            else:
                _, flag, p, q = self.last_command
                x, y, z, w = q
                yaw = math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
                text = f'PREVIEW flag={flag} xyz={p.round(3).tolist()} yaw={yaw:.1f} deg'
            self.get_logger().info(text)

    return ControlPreview


def main():
    import rclpy
    rclpy.init()
    node = node_type()()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
