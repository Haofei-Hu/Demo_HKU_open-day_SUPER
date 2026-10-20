#!/usr/bin/env python3
"""One-shot floor-referenced hover. Default: diagnostics only, no flight output.

--execute enables a native setpoint publisher, but motion still requires an
explicit /first_hover/start Trigger. Never use with other setpoint senders.
Run on the actual takeoff floor (FC centre 8.5 cm above it). Keep the existing
localization processes running throughout flight; restarting this node in flight
is deliberately unsupported. Measurement messages are never retimed.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, RCIn
from mavros_msgs.srv import SetMode, CommandBool
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger

from ground_setpoint_transport import pose_values
from first_hover_preview import LIMITS
from hover_reference import HoverReference


NATIVE = '/mavros/setpoint_position/local'
DEBUG = '/super_debug/first_hover_target'
PARAMETERS = dict(LIMITS, EKF2_EV_POS_X=0.0, EKF2_EV_POS_Y=0.0,
                  EKF2_EV_POS_Z=0.0, EKF2_EV_QMIN=0)


class FirstHoverControl(Node):
    MAX_HOVER_HEIGHT = 1.
    EXPECTED_PARAMETERS = PARAMETERS
    NATIVE_TOPIC = NATIVE
    NODE_NAME = 'first_hover'
    DEBUG_TOPIC = DEBUG
    START_SERVICE = '/first_hover/start'
    LAND_SERVICE = '/first_hover/land'
    RC_IN_CHECK = True

    def __init__(self, execute=False, hover_fc_height=.5, hold_seconds=2.):
        profile = HoverReference((0., 0., 0.), 0.,
                                 hover_fc_height=hover_fc_height, hold_seconds=hold_seconds)
        if hover_fc_height > self.MAX_HOVER_HEIGHT:
            raise ValueError(f'Controller supports FC heights up to {self.MAX_HOVER_HEIGHT:g} m')
        super().__init__(self.NODE_NAME)
        self.execute = execute
        self.hover_fc_height = float(hover_fc_height)
        self.hold_seconds = profile.hold_seconds
        self.offboard_budget = math.ceil(profile.ramp_seconds + profile.hold_seconds + 2.)
        self.phase = 'CHECKING'
        self.state = self.local = self.vision = self.estimator = self.extended = self.rc = None
        self.receipts = {}
        self.reference = self.identities_locked = None
        self.stable_since = self.candidate = None
        self.phase_at = time.monotonic()
        self.ready_at = self.armed_at = self.hold_at = None
        self.last_tick = self.last_log = -math.inf
        self.fault = None
        self.started = False
        self.commands = []
        self.command_future = None
        self.sent = 0
        self.last_target = None
        self.confirmed = {}
        self.parameter_futures = {}
        self.debug_pub = self.create_publisher(PoseStamped, self.DEBUG_TOPIC, 10)
        self.native_pub = self.make_native_publisher() if execute else None
        for field, msg_type, topic in (
            ('state', State, '/mavros/state'),
            ('local', PoseStamped, '/mavros/local_position/pose'),
            ('vision', PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov'),
            ('estimator', EstimatorStatus, '/mavros/estimator_status'),
            ('extended', ExtendedState, '/mavros/extended_state'),
            ('rc', RCIn, '/mavros/rc/in'),
        ):
            self.create_subscription(msg_type, topic,
                                     lambda msg, name=field: self.store(name, msg),
                                     qos_profile_sensor_data)
        self.parameter_clients = {
            'fc': self.create_client(GetParameters, '/mavros/param/get_parameters'),
            'frame': self.create_client(GetParameters, '/mavros/setpoint_position/get_parameters'),
        }
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.create_service(Trigger, self.START_SERVICE, self.start)
        self.create_service(Trigger, self.LAND_SERVICE, self.stop_requested)
        self.create_timer(1.0, self.poll_parameters)
        self.create_timer(.05, self.tick)
        self.get_logger().info(self.startup_message())

    def startup_message(self):
        return (
            (f'EXECUTE ENABLED; awaiting explicit {self.START_SERVICE}. ' if self.execute else
             'CHECK ONLY; no native setpoints, mode or arming requests. ') +
            f'Floor FC=0.085 m, hover FC={self.hover_fc_height:g} m, '
            f'hold={self.hold_seconds:g} s, fixed XY/yaw, then AUTO.LAND; '
            f'OFFBOARD budget={self.offboard_budget:g} s; one flight only.')

    def make_native_publisher(self):
        return self.create_publisher(PoseStamped,self.NATIVE_TOPIC,10)

    def store(self, field, msg):
        setattr(self, field, msg)
        self.receipts[field] = time.monotonic()

    def fresh(self, field, limit):
        msg = getattr(self, field)
        if msg is None or time.monotonic() - self.receipts.get(field, -math.inf) > limit:
            return False
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        return stamp > 0 and -.05 <= age <= limit

    def poll_parameters(self):
        for key, client in self.parameter_clients.items():
            pending = self.parameter_futures.get(key)
            if pending and not pending[0].done():
                if time.monotonic() - pending[1] > 2:
                    pending[0].cancel()
                    self.confirmed.pop(key, None)
                continue
            if not client.service_is_ready():
                self.confirmed.pop(key, None)
                continue
            names = list(self.EXPECTED_PARAMETERS) if key == 'fc' else ['mav_frame']
            future = client.call_async(GetParameters.Request(names=names))
            self.parameter_futures[key] = (future, time.monotonic())
            future.add_done_callback(lambda result, name=key: self.parameter_result(name, result))

    def parameter_result(self, key, future):
        try:
            values = future.result().values
            expected = list(self.EXPECTED_PARAMETERS.values()) if key == 'fc' else ['LOCAL_NED']
            if len(values) != len(expected):
                raise ValueError('Incomplete parameter response')
            for value, target in zip(values, expected):
                if isinstance(target, str):
                    good = value.type == 4 and value.string_value == target
                else:
                    number = value.integer_value if isinstance(target, int) else value.double_value
                    good = (value.type == (2 if isinstance(target, int) else 3)
                            and math.isfinite(number) and abs(number - target) <= 1e-6)
                if not good:
                    raise ValueError('Parameter mismatch')
            self.confirmed[key] = time.monotonic()
        except Exception:
            self.confirmed.pop(key, None)

    def sources(self):
        sources = []
        for topic in ('/mavros/local_position/pose', '/mavros/vision_pose/pose_cov'):
            publishers = self.get_publishers_info_by_topic(topic)
            if len(publishers) != 1:
                raise ValueError('Need one FC pose and one vision publisher')
            sources.append(bytes(publishers[0].endpoint_gid))
        if self.identities_locked is not None and tuple(sources) != self.identities_locked:
            raise ValueError('Localization source changed; do not restart localization in flight')
        return tuple(sources)

    def validate(self, ground=False):
        if not self.fresh('state', 2) or not self.state.connected:
            raise ValueError('Waiting for fresh connected FC state')
        for key in ('fc', 'frame'):
            if time.monotonic() - self.confirmed.get(key, -math.inf) > 3:
                raise ValueError('Waiting for confirmed fusion, speed limits, LAND fallback and LOCAL_NED')
        fields = [('local', .25), ('vision', .25), ('estimator', 2), ('extended', 2)]
        if self.execute or self.RC_IN_CHECK:
            fields.append(('rc', .5))
        for field, limit in fields:
            if not self.fresh(field, limit):
                raise ValueError('Missing/stale ' + field)
        if (self.execute or self.RC_IN_CHECK) and len(self.rc.channels) < 7:
            raise ValueError('RC channels unavailable')
        if not (self.estimator.attitude_status_flag and self.estimator.velocity_horiz_status_flag
                and self.estimator.velocity_vert_status_flag and self.estimator.pos_horiz_rel_status_flag
                and self.estimator.pos_vert_abs_status_flag) or self.estimator.accel_error_status_flag:
            raise ValueError('FC attitude/local position/velocity estimate invalid')
        if self.local.header.frame_id != 'map' or self.vision.header.frame_id != 'px4_enu_preview':
            raise ValueError('Unexpected local/vision frame')
        xyz, yaw = pose_values(self.local)
        external, ev_yaw = pose_values(PoseStamped(pose=self.vision.pose.pose))
        if math.dist(xyz, external) > .15 or abs(self.angle(yaw - ev_yaw)) > math.radians(5):
            raise ValueError('FC and external localization disagree')
        identities = self.sources()
        if self.count_publishers(self.DEBUG_TOPIC) != 1:
            raise ValueError('Another hover controller/preview is publishing')
        if self.execute:
            if self.count_publishers(self.NATIVE_TOPIC) != 1 or self.native_pub.get_subscription_count() < 1:
                raise ValueError('Need this node as sole position sender, with MAVROS subscriber')
            for topic in (NATIVE,'/mavros/setpoint_raw/local', '/mavros/setpoint_raw/global',
                          '/mavros/setpoint_raw/attitude', '/mavros/setpoint_position/global',
                          '/mavros/setpoint_position/global_to_local',
                          '/mavros/setpoint_velocity/cmd_vel', '/mavros/setpoint_velocity/cmd_vel_unstamped',
                          '/mavros/setpoint_accel/accel', '/mavros/setpoint_attitude/attitude',
                          '/mavros/setpoint_attitude/cmd_vel', '/mavros/setpoint_attitude/thrust'):
                if topic!=self.NATIVE_TOPIC and self.count_publishers(topic):
                    raise ValueError('Other control publisher: ' + topic)
        if ground:
            rc_required = self.execute or self.RC_IN_CHECK
            modes = ('POSCTL',) if rc_required else ('POSCTL', 'AUTO.LOITER')
            if self.state.armed or self.state.mode not in modes or (rc_required and not self.state.manual_input):
                raise ValueError('Initial state must be disarmed POSCTL with RC input')
            if self.extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                raise ValueError('FC does not report on ground')
        self.check_envelope(xyz, yaw, ground)
        return xyz, yaw, identities

    def check_envelope(self, xyz, yaw, ground):
        if self.reference is not None:
            g = self.reference.ground
            if math.hypot(xyz[0]-g[0], xyz[1]-g[1]) > .3:
                raise ValueError('Horizontal departure exceeds 0.3 m')
            if xyz[2] < g[2] - .1 or xyz[2] > g[2] + self.reference.lift + .235:
                raise ValueError('Height outside first-hover envelope')
            if abs(self.angle(yaw - self.reference.yaw)) > math.radians(15):
                raise ValueError('Heading departure exceeds 15 degrees')
            if ground and math.dist(xyz, g) > .05:
                raise ValueError('Aircraft moved after ground capture')

    def allowed_modes(self):
        return {'PRESTREAM': ('POSCTL',), 'MODE_REQUEST': ('POSCTL', 'OFFBOARD'),
                'ARM_REQUEST': ('OFFBOARD',), 'RISE': ('OFFBOARD',),
                'HOLD': ('OFFBOARD',), 'LAND_REQUEST': ('OFFBOARD', 'AUTO.LAND'),
                'LANDING': ('AUTO.LAND',)}

    def mission_tick(self, now, xyz, yaw):
        return False

    @staticmethod
    def angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    def change(self, phase):
        self.phase, self.phase_at = phase, time.monotonic()
        self.get_logger().info('Phase: ' + phase)

    def start(self, request, response):
        response.success = False
        if not self.execute:
            response.message = 'Check-only mode; no flight action exists'
        elif self.phase != 'READY' or self.started:
            response.message = 'Not READY or session already used; no start'
        elif not (self.mode_client.service_is_ready() and self.arm_client.service_is_ready()):
            response.message = 'MAVROS command services unavailable'
        else:
            try:
                self.validate(ground=True)
                self.started = True
                self.change('PRESTREAM')
                response.success = True
                response.message = 'Accepted: ground prestream, OFFBOARD, arm, rise, hold, LAND'
            except ValueError as exc:
                response.message = str(exc)
        return response

    def stop_requested(self, request, response):
        if self.phase in ('RISE', 'HOLD'):
            self.land('Operator requested landing')
            response.success, response.message = True, 'Landing requested'
        else:
            self.abort('Operator stopped session')
            response.success, response.message = True, 'Stopped; no further flight initiation'
        return response

    def command(self, action):
        # No retry of OFFBOARD/ARM: this flight can never silently restart.
        client = self.arm_client if action == 'ARM' else self.mode_client
        request = (CommandBool.Request(value=True) if action == 'ARM' else
                   SetMode.Request(base_mode=0, custom_mode=action))
        if not client.service_is_ready():
            raise ValueError(action + ' service unavailable')
        self.commands.append(action)
        self.command_future = client.call_async(request)

    def command_rejected(self):
        if self.command_future is None or not self.command_future.done():
            return False
        try:
            response = self.command_future.result()
            return not (response.success if self.commands[-1] == 'ARM' else response.mode_sent)
        except Exception:
            return True

    def abort(self, reason):
        if self.phase in ('DONE', 'STOPPED'):
            return
        self.fault = reason
        self.get_logger().error(reason + '; setpoints stopped, no rearm or mode re-entry')
        # Do not override a pilot's mode change. On input loss, cease the stream
        # even if LAND is rejected/unreachable: PX4's verified OFFBOARD-loss LAND
        # action is the remaining fallback. No in-flight disarm request is made.
        if (self.execute and self.started and self.fresh('state', 2)
                and self.state.connected and self.state.armed and self.state.mode == 'OFFBOARD'):
            try:
                self.command('AUTO.LAND')
            except ValueError:
                pass
        self.change('STOPPED')

    def land(self, reason):
        self.get_logger().info(reason + '; requesting AUTO.LAND, then FC landing detection/auto-disarm')
        try:
            self.command('AUTO.LAND')
            self.change('LAND_REQUEST')
        except ValueError as exc:
            self.abort(str(exc))

    def publish(self, xyz, yaw, native=False):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # new command, not a measurement
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.debug_pub.publish(msg)
        if native:
            self.native_pub.publish(msg)
            self.last_target = list(xyz)
            self.sent += 1

    def check_timer_gap(self,gap):
        raise ValueError('Setpoint timer interrupted for more than 0.2 seconds')

    def tick(self):
        now = time.monotonic()
        gap, self.last_tick = now - self.last_tick, now
        if self.phase in ('DONE', 'STOPPED'):
            return
        try:
            # Pilot takeover has priority over configuration and pose checks.
            if self.started and self.fresh('state', 2):
                allowed = self.allowed_modes()[self.phase]
                if self.state.mode not in allowed:
                    self.abort('Pilot/FC changed mode to ' + self.state.mode)
                    return
            if self.phase == 'LANDING':
                if not self.fresh('state', 2) or not self.state.connected:
                    raise ValueError('Lost FC state during landing; no completion claim')
                if (not self.state.armed and self.fresh('extended', 2)
                        and self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND):
                    self.change('DONE')
                    self.get_logger().info('FC confirmed landed and disarmed. No second flight.')
                return  # PX4 owns the descent; never publish while in AUTO.LAND.
            if self.phase == 'LAND_REQUEST' and self.fresh('state', 2) and self.state.mode == 'AUTO.LAND':
                self.change('LANDING')
                return
            if self.started and gap > .2:
                self.check_timer_gap(gap)
            ground = self.phase in ('CHECKING', 'READY', 'PRESTREAM', 'MODE_REQUEST')
            xyz, yaw, identities = self.validate(ground=ground and self.phase != 'MODE_REQUEST')
            if self.phase == 'MODE_REQUEST':
                if self.state.armed or math.dist(xyz, self.reference.ground) > .05:
                    raise ValueError('Unexpected arming/movement before arm request')
            if self.phase == 'CHECKING':
                if self.candidate is None or math.dist(xyz, self.candidate[0]) > .025 or abs(self.angle(yaw-self.candidate[1])) > math.radians(2):
                    self.candidate, self.stable_since = (xyz, yaw), now
                elif now - self.stable_since >= 3:
                    self.reference = HoverReference(xyz, yaw,
                        hover_fc_height=self.hover_fc_height, hold_seconds=self.hold_seconds)
                    self.identities_locked = identities
                    self.change('READY')
                    self.get_logger().info(
                        f'GROUND LOCK xyz={list(self.reference.ground)}; '
                        f'fixed hover z={self.reference.ground[2]+self.reference.lift:.3f}; '
                        f'current measured xyz={xyz}; '
                        f'FC height above floor={self.hover_fc_height:.3f} m; '
                        f'hold={self.hold_seconds:g} s. Awaiting explicit start; no motion yet.')
                return
            if self.mission_tick(now, xyz, yaw):
                return
            target = list(self.reference.ground)
            if self.phase == 'READY':
                target[2] += self.reference.lift
                self.publish(target, self.reference.yaw)
                return
            if self.phase == 'PRESTREAM' and now - self.phase_at >= 2:
                self.command('OFFBOARD')
                self.change('MODE_REQUEST')
            elif self.phase == 'MODE_REQUEST':
                if self.command_rejected():
                    raise ValueError('OFFBOARD request rejected')
                if self.state.mode == 'OFFBOARD':
                    self.command('ARM')
                    self.change('ARM_REQUEST')
                elif now - self.phase_at > 3:
                    raise ValueError('OFFBOARD transition timed out')
            elif self.phase == 'ARM_REQUEST':
                if self.command_rejected():
                    raise ValueError('Arming rejected; inspect PX4 health, never force arm')
                if self.state.armed:
                    self.armed_at = now
                    self.change('RISE')
                elif now - self.phase_at > 3:
                    raise ValueError('Arming timed out; no retry')
            elif self.phase in ('RISE', 'HOLD'):
                if not self.state.armed:
                    self.abort('Disarming observed')
                    return
                target[2] += self.reference.lift
                elapsed = now - self.armed_at
                if self.phase == 'RISE':
                    u = min(1., elapsed / self.reference.ramp_seconds)
                    target[2] = self.reference.ground[2] + self.reference.lift * (u**3 * (10 - 15*u + 6*u*u))
                    if u == 1 and abs(xyz[2]-target[2]) <= .06:
                        self.hold_at = now
                        self.change('HOLD')
                elif abs(xyz[2]-target[2]) > .12:
                    raise ValueError('Hover altitude tracking error exceeds 0.12 m')
                if self.phase == 'HOLD' and now - self.hold_at >= self.hold_seconds:
                    self.land(f'{self.hold_seconds:g}-second hold finished')
                elif elapsed > self.offboard_budget:
                    self.land(f'{self.offboard_budget:g}-second OFFBOARD flight budget reached')
            elif self.phase == 'LAND_REQUEST':
                target = self.last_target or list(self.reference.ground)
                if self.command_rejected() or now - self.phase_at > 2:
                    raise ValueError('LAND transition failed; cease stream for verified PX4 fallback')
            self.publish(target, self.reference.yaw, native=True)
        except ValueError as exc:
            if self.reference is not None or self.started or (self.state is not None and self.state.armed):
                self.abort(str(exc))
            else:
                self.candidate = self.stable_since = None
                if now - self.last_log > 3:
                    self.last_log = now
                    self.get_logger().warning(str(exc))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='enable flight output; still awaits /first_hover/start')
    parser.add_argument('--hover-height', type=float, default=.5,
                        help='FC centre height above takeoff floor in metres, up to 1.0 (default: 0.5)')
    parser.add_argument('--hold-seconds', type=float, default=2.,
                        help='hold after reaching target height, 0 to 6 seconds (default: 2)')
    args, ros_args = parser.parse_known_args()
    try:
        HoverReference((0., 0., 0.), 0., hover_fc_height=args.hover_height,
                       hold_seconds=args.hold_seconds)
        if args.hover_height > 1.:
            raise ValueError('First-hover controller supports FC heights up to 1 m')
    except ValueError as exc:
        parser.error(str(exc))
    rclpy.init(args=ros_args)
    node = FirstHoverControl(execute=args.execute, hover_fc_height=args.hover_height,
                            hold_seconds=args.hold_seconds)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.abort('Controller interrupted')
        # Let a queued LAND request progress; stream already stopped. OFFBOARD
        # loss fallback also covers process death, when this code cannot run.
        until = time.monotonic() + .3
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
