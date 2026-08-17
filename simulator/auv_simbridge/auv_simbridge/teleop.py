"""Hold-to-move closed-loop keyboard teleop for the AUV.

Flying rule: while a key is held the vehicle moves; the moment every key is
released it FREEZES - position, heading and depth are pinned where they were at
the instant of release, and the controller actively brakes to get there. Nothing
coasts, nothing floats up.

That needs key-release events, which Gazebo's KeyPublisher never sends. The
`AuvTeleop` gz-gui panel in auv_gui supplies them: it filters the main window's
key events and publishes the set of keys held right now, at 50 Hz, on
`/teleop/keys`. If that stream goes stale (panel closed, Gazebo hung) the
vehicle freezes rather than running away.

Without the panel there is a degraded fallback on `/keyboard/keypress`, where a
press is treated as "held" for a short window that OS auto-repeat keeps renewing.
It is jerky for the first half second - that is the auto-repeat delay, not a bug -
but it means teleop still works headless.

    W / S     forward / reverse
    A / D     strafe left / right
    Q / E     yaw left / right
    R / F     rise / dive
    SHIFT     boost   - hold with anything above for the fast gear
    CTRL      precision - hold for the fine gear, for lining up on small objects
    Z / C     trim the persistent gear down / up
    SPACE     re-anchor the freeze point here
    X         cut all thrust (controller disengaged, vehicle free)
    T         toggle teleop on/off

Control is a cascade. Keys give a body-frame velocity command; each axis turns
that into a force with the model's own drag curve fed forward plus PI on the
velocity error; and those four numbers - Fx, Fy, Fz, Mz - are split across the
eight thrusters through the real allocation matrix. The feed-forward is the part
that matters for feel: with P alone the vehicle must hold a standing velocity
error to make any thrust at all, so it always settles short of the commanded
speed, which from the cockpit looks like a low speed cap.

When no key is held the velocity command comes from the position error to the
freeze point instead, so "fly" and "freeze" are the same code path.

Every gain and limit is a ROS parameter, retunable live with
`ros2 param set /auv_teleop <name> <value>`.
"""

import json
import math
import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Float64, Int32, String
from std_srvs.srv import Empty

# Qt key codes, as published by both the AuvTeleop panel and KeyPublisher.
KEY_W, KEY_A, KEY_S, KEY_D = 87, 65, 83, 68
KEY_Q, KEY_E, KEY_R, KEY_F = 81, 69, 82, 70
KEY_T, KEY_X, KEY_Z, KEY_C = 84, 88, 90, 67
KEY_SPACE = 32

DRIVE_KEYS = frozenset({KEY_W, KEY_A, KEY_S, KEY_D, KEY_Q, KEY_E, KEY_R, KEY_F})
TAP_KEYS = frozenset({KEY_T, KEY_X, KEY_Z, KEY_C, KEY_SPACE})

THRUSTERS = ('surge_front_left', 'surge_front_right',
             'surge_back_left', 'surge_back_right',
             'heave_front_left', 'heave_front_right',
             'heave_back_left', 'heave_back_right')

# The four surge thrusters are a vectored X at 45 degrees, 0.402 m fore/aft and
# 0.244 m out; the four heave thrusters point straight up. Read off the joint
# axes and link poses in the AUV model, this maps a per-thruster force to the
# net body wrench (Fx, Fy, Fz, Mz). Note the yaw arm is only 0.112 m - that is
# why yaw feels weak unless it gets its own share of the thrust.
ALLOCATION = np.array([
    [0.707,  0.707,  0.707,  0.707, 0.0, 0.0, 0.0, 0.0],    # Fx
    [-0.707, 0.707,  0.707, -0.707, 0.0, 0.0, 0.0, 0.0],    # Fy
    [0.0,    0.0,    0.0,    0.0,   1.0, 1.0, 1.0, 1.0],    # Fz
    [-0.112, 0.112, -0.112,  0.112, 0.0, 0.0, 0.0, 0.0],    # Mz
])
# Least-norm solution: the thrust split that produces a wanted wrench using as
# little total force as possible.
MIX = np.linalg.pinv(ALLOCATION)


class AuvTeleop(Node):

    def __init__(self):
        super().__init__('auv_teleop')

        p = self.declare_parameter
        p('enabled', True)

        # -- how fast it flies ------------------------------------------------
        p('cruise_speed', 0.50)        # m/s with no modifier held
        p('cruise_yaw', 0.80)          # rad/s
        p('cruise_depth_rate', 0.35)   # m/s vertical
        p('boost_scale', 2.50)         # SHIFT
        p('precision_scale', 0.15)     # CTRL
        p('gear', 1.00)                # persistent trim, Z/C
        # Ceilings after all scaling. Set near what the thrusters can actually
        # deliver at max_thrust=60: asking for a speed the vehicle cannot reach
        # just parks the controller in saturation.
        p('max_linear', 1.25)
        p('max_yaw', 1.10)
        p('max_depth_rate', 0.80)
        # How fast a velocity command may change. Without this a key press is a
        # step input, the loop saturates, and the vehicle overshoots the speed
        # you asked for before settling back to it.
        p('accel_limit', 1.50)         # m/s^2
        p('yaw_accel_limit', 3.00)     # rad/s^2

        # -- control ----------------------------------------------------------
        # Per-thruster force ceiling. This is what really sets the top speed: the
        # four surge thrusters point at 45 degrees, so they deliver 2.83x this
        # much forward force, against 130.7 N per m/s of drag.
        p('max_thrust', 60.0)
        # Velocity loops work in net body wrench: N per m/s, N.m per rad/s.
        # The drag feed-forward does the heavy lifting, so the integral only has
        # to mop up modelling error - keep it small or it winds up during the
        # acceleration transient and overshoots the commanded speed.
        p('surge_kp', 300.0)
        p('surge_ki', 15.0)
        p('yaw_rate_kp', 25.0)
        p('yaw_rate_ki', 3.0)
        p('heave_kp', 300.0)
        p('heave_ki', 20.0)
        # Drag feed-forward, straight out of the model's Hydrodynamics block. A
        # plain P loop needs a standing error to hold a speed, which is exactly
        # the "max speed is capped" problem; feeding the known drag forward means
        # the commanded speed is the speed you get.
        p('drag_surge', [130.7, 14.5])     # linear, quadratic
        p('drag_sway', [116.0, 216.0])
        p('drag_heave', [116.0, 350.0])
        p('drag_yaw', [9.5, 18.0])
        # Outer loops: error -> velocity command.
        p('yaw_kp', 3.0)               # heading error -> yaw rate command
        p('position_kp', 1.6)          # position error -> velocity command
        p('depth_kp', 2.0)             # depth error -> vertical velocity command
        p('brake_speed', 1.20)         # cap on the velocity the hold may ask for
        # Holds the vehicle against its own buoyancy, per heave thruster, so the
        # loop only trims around it rather than fighting it.
        p('buoyancy_feedforward', -1.462)
        p('deadband', 0.02)            # m, inside this the hold stops nudging

        # -- input ------------------------------------------------------------
        p('keys_timeout', 0.40)        # s without a panel heartbeat -> freeze
        p('keypress_hold', 0.35)       # s a fallback keypress counts as held

        self.enabled = self.get_parameter('enabled').value
        self.engaged = True            # X disengages the controller entirely

        self.held = set()              # keys down right now
        self.shift = False
        self.ctrl = False
        self.last_keys_msg = None      # wall time of the last panel heartbeat
        self.pulses = {}               # fallback: key -> wall time it expires
        self.source = 'none'

        # Freeze targets. Live values, not commands - they move only when the
        # corresponding axis is being flown.
        self.hold_xy = None
        self.hold_yaw = None
        self.depth = None
        self.trim = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0, 'yaw': 0.0}
        # Slew-limited velocity commands, so a key press is a ramp not a step.
        self.ramp = {'surge': 0.0, 'sway': 0.0, 'yaw': 0.0}

        # Last commanded velocities, for the status panel.
        self.surge_cmd = self.sway_cmd = self.yaw_cmd = 0.0

        self.state = None
        self.last_time = None

        self.thrusters = {
            name: self.create_publisher(
                Float64, f'/model/auv/joint/{name}_joint/cmd_thrust', 10)
            for name in THRUSTERS
        }
        self.status_pub = self.create_publisher(String, '/teleop/status', 10)

        self.create_subscription(Odometry, '/model/auv/odometry', self.odometry_callback, 10)
        self.create_subscription(String, '/teleop/command', self.command_callback, 10)
        self.create_subscription(String, '/teleop/keys', self.keys_callback, 10)
        self.create_subscription(Int32, '/keyboard/keypress', self.keypress_callback, 10)
        self.create_service(Empty, '/simulator/teleop_on', self.handle_on)
        self.create_service(Empty, '/simulator/teleop_off', self.handle_off)

        self.create_timer(0.02, self.control_step)     # 50 Hz
        self.create_timer(0.20, self.publish_status)
        self.get_logger().info(
            'AUV teleop ready: hold to move, release to freeze. '
            'W/S surge, A/D sway, Q/E yaw, R/F depth, SHIFT boost, CTRL precision, '
            'Z/C gear, SPACE re-anchor, X cut, T toggle.')

    # -- input --------------------------------------------------------------

    def keys_callback(self, msg):
        """Held-key set from the AuvTeleop gz-gui panel."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        keys = {int(k) for k in payload.get('keys', [])}
        self.shift = bool(payload.get('shift'))
        self.ctrl = bool(payload.get('ctrl'))
        self.last_keys_msg = _wall()

        # Taps act on the rising edge only, so holding SPACE does not re-anchor
        # fifty times a second, and X does not fight a still-held drive key.
        pressed = keys - self.held
        self.held = keys
        if pressed & DRIVE_KEYS:
            self.engaged = True
        for key in pressed & TAP_KEYS:
            self.handle_tap(key)

    def keypress_callback(self, msg):
        """Fallback for a plain KeyPublisher, which sends no releases."""
        if self.panel_live():
            return                     # the panel is authoritative
        key = int(msg.data)
        if key in TAP_KEYS:
            self.handle_tap(key)
            return
        if key in DRIVE_KEYS:
            self.pulses[key] = _wall() + self.get_parameter('keypress_hold').value
            self.engaged = True

    def panel_live(self):
        return (self.last_keys_msg is not None and
                _wall() - self.last_keys_msg < self.get_parameter('keys_timeout').value)

    def drive_keys(self):
        """Which drive keys count as held right now, and where they came from."""
        if self.panel_live():
            self.source = 'panel'
            return self.held & DRIVE_KEYS
        # No panel heartbeat: the held set is stale and must not be trusted.
        self.held = set()
        now = _wall()
        self.pulses = {key: until for key, until in self.pulses.items() if until > now}
        self.source = 'keypress' if self.pulses else 'none'
        return set(self.pulses) & DRIVE_KEYS

    def handle_tap(self, key):
        if key == KEY_T:
            self.set_enabled(not self.enabled)
            return
        if not self.enabled:
            return
        if key == KEY_X:
            self.cut_thrust()
        elif key == KEY_SPACE:
            self.freeze(announce=True)
        elif key in (KEY_Z, KEY_C):
            gear = self.get_parameter('gear').value * (0.8 if key == KEY_Z else 1.25)
            gear = float(np.clip(gear, 0.15, 3.0))
            self.set_parameters([rclpy.Parameter(
                'gear', rclpy.Parameter.Type.DOUBLE, gear)])
            self.get_logger().info(f'Gear {gear:.2f}')

    def command_callback(self, msg):
        """JSON from the Gazebo GUI panel."""
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if 'enabled' in command:
            self.set_enabled(bool(command['enabled']))
        for key in ('cruise_speed', 'cruise_yaw', 'cruise_depth_rate', 'gear',
                    'boost_scale', 'precision_scale', 'max_linear', 'max_yaw'):
            if key in command:
                self.set_parameters([rclpy.Parameter(
                    key, rclpy.Parameter.Type.DOUBLE, float(command[key]))])
        if command.get('stop'):
            self.freeze(announce=True)
        if command.get('cut'):
            self.cut_thrust()

    def handle_on(self, request, response):
        self.set_enabled(True)
        return response

    def handle_off(self, request, response):
        self.set_enabled(False)
        return response

    def set_enabled(self, value):
        self.enabled = bool(value)
        self.set_parameters([rclpy.Parameter(
            'enabled', rclpy.Parameter.Type.BOOL, self.enabled)])
        if not self.enabled:
            self.publish_thrust({name: 0.0 for name in self.thrusters})
        else:
            self.freeze()
        self.get_logger().info(f'Teleop {"enabled" if self.enabled else "disabled"}.')

    def freeze(self, announce=False):
        """Pin position, heading and depth wherever the vehicle is now."""
        self.pulses.clear()
        self.engaged = True
        if self.state is not None:
            self.hold_xy = self.state['xyz'][:2].copy()
            self.hold_yaw = self.state['yaw']
            self.depth = float(self.state['xyz'][2])
        self.reset_trim()
        if announce:
            self.get_logger().info('Frozen here.')

    def cut_thrust(self):
        self.held.clear()
        self.pulses.clear()
        self.hold_xy = self.hold_yaw = None
        self.engaged = False
        self.reset_trim()
        self.publish_thrust({name: 0.0 for name in self.thrusters})
        self.get_logger().info('Thrust cut; vehicle is free.')

    # -- state --------------------------------------------------------------

    def odometry_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rot = Rot.from_quat([q.x, q.y, q.z, q.w])
        body_vel = np.array([msg.twist.twist.linear.x,
                             msg.twist.twist.linear.y,
                             msg.twist.twist.linear.z])
        self.state = {
            'xyz': np.array([p.x, p.y, p.z]),
            'yaw': float(rot.as_euler('xyz')[2]),
            'rot': rot,
            # Gazebo's odometry twist is body-frame, which is what we control in.
            'body_vel': body_vel,
            'world_vel': rot.apply(body_vel),
            'yaw_rate': float(msg.twist.twist.angular.z),
        }
        if self.depth is None:
            self.freeze()

    # -- control ------------------------------------------------------------

    def gear_scale(self):
        """Speed multiplier from the modifier keys and the persistent gear."""
        scale = self.get_parameter('gear').value
        if self.ctrl:
            return scale * self.get_parameter('precision_scale').value, 'precision'
        if self.shift:
            return scale * self.get_parameter('boost_scale').value, 'boost'
        return scale, 'normal'

    def velocity_command(self, keys):
        """Body-frame (surge, sway) and yaw rate asked for by the held keys."""
        scale, _ = self.gear_scale()
        speed = self.get_parameter('cruise_speed').value * scale
        speed = float(np.clip(speed, 0.0, self.get_parameter('max_linear').value))
        yaw = self.get_parameter('cruise_yaw').value * scale
        yaw = float(np.clip(yaw, 0.0, self.get_parameter('max_yaw').value))

        surge = speed * ((KEY_W in keys) - (KEY_S in keys))
        sway = speed * ((KEY_A in keys) - (KEY_D in keys))
        yaw_rate = yaw * ((KEY_Q in keys) - (KEY_E in keys))
        return surge, sway, yaw_rate

    def control_step(self):
        if self.state is None or not self.enabled or not self.engaged:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_time is None else now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            return

        state = self.state
        keys = self.drive_keys()
        surge, sway, yaw_rate = self.velocity_command(keys)

        # -- horizontal: fly on command, otherwise brake back to the anchor ---
        flying_xy = bool(keys & {KEY_W, KEY_A, KEY_S, KEY_D})
        accel = self.get_parameter('accel_limit').value
        if flying_xy:
            self.hold_xy = None
            surge_cmd = self.slew('surge', surge, accel, dt)
            sway_cmd = self.slew('sway', sway, accel, dt)
        else:
            if self.hold_xy is None:
                # Just released: the anchor is wherever we are at this instant.
                self.hold_xy = state['xyz'][:2].copy()
            error_world = np.array([self.hold_xy[0] - state['xyz'][0],
                                    self.hold_xy[1] - state['xyz'][1], 0.0])
            if np.linalg.norm(error_world) < self.get_parameter('deadband').value:
                error_world[:] = 0.0
            error_body = state['rot'].inv().apply(error_world)
            brake = self.get_parameter('brake_speed').value
            gain = self.get_parameter('position_kp').value
            # No slew here: braking to the anchor must be allowed to bite at
            # once, otherwise "release to freeze" turns into "release to coast".
            surge_cmd = float(np.clip(error_body[0] * gain, -brake, brake))
            sway_cmd = float(np.clip(error_body[1] * gain, -brake, brake))
            self.ramp['surge'], self.ramp['sway'] = surge_cmd, sway_cmd

        # -- yaw: fly on command, otherwise hold heading ----------------------
        if keys & {KEY_Q, KEY_E}:
            self.hold_yaw = None
            rate_cmd = self.slew('yaw', yaw_rate,
                                 self.get_parameter('yaw_accel_limit').value, dt)
        else:
            if self.hold_yaw is None:
                self.hold_yaw = state['yaw']
            error = math.atan2(math.sin(self.hold_yaw - state['yaw']),
                               math.cos(self.hold_yaw - state['yaw']))
            rate_cmd = float(np.clip(error * self.get_parameter('yaw_kp').value,
                                     -self.get_parameter('max_yaw').value,
                                     self.get_parameter('max_yaw').value))
            self.ramp['yaw'] = rate_cmd

        # -- depth: fly on command, otherwise hold the last depth -------------
        scale, _ = self.gear_scale()
        rate = float(np.clip(self.get_parameter('cruise_depth_rate').value * scale,
                             0.0, self.get_parameter('max_depth_rate').value))
        vertical = rate * ((KEY_R in keys) - (KEY_F in keys))
        if vertical != 0.0:
            self.depth = float(state['xyz'][2])   # follow, so release pins here
            vertical_cmd = vertical
        else:
            depth_error = self.depth - state['xyz'][2]
            if abs(depth_error) < self.get_parameter('deadband').value:
                depth_error = 0.0
            vertical_cmd = float(np.clip(
                depth_error * self.get_parameter('depth_kp').value,
                -self.get_parameter('max_depth_rate').value,
                self.get_parameter('max_depth_rate').value))

        # -- velocity loops: wanted net body wrench ---------------------------
        # Snapshot for anti-windup: if the thrusters cannot deliver what these
        # loops ask for, integrating the leftover error only stores up a lurch
        # for when the demand comes back down.
        before = dict(self.trim)
        wrench = np.array([
            self.axis_effort('surge', surge_cmd, state['body_vel'][0], dt,
                             'surge_kp', 'surge_ki', 'drag_surge', 80.0),
            self.axis_effort('sway', sway_cmd, state['body_vel'][1], dt,
                             'surge_kp', 'surge_ki', 'drag_sway', 80.0),
            self.axis_effort('heave', vertical_cmd, state['world_vel'][2], dt,
                             'heave_kp', 'heave_ki', 'drag_heave', 80.0)
            # Four heave thrusters each carrying the buoyancy feed-forward.
            + 4.0 * self.get_parameter('buoyancy_feedforward').value,
            self.axis_effort('yaw', rate_cmd, state['yaw_rate'], dt,
                             'yaw_rate_kp', 'yaw_rate_ki', 'drag_yaw', 12.0),
        ])

        forces, saturated = self.mix(wrench)
        # Horizontal and vertical use disjoint thrusters, so they wind up - and
        # get held back - independently.
        for axis in ('surge', 'sway', 'yaw'):
            if saturated['horizontal']:
                self.trim[axis] = before[axis]
        if saturated['vertical']:
            self.trim['heave'] = before['heave']

        self.surge_cmd, self.sway_cmd, self.yaw_cmd = surge_cmd, sway_cmd, rate_cmd
        self.publish_thrust(forces)

    def axis_effort(self, axis, command, actual, dt, kp_name, ki_name,
                    drag_name, trim_limit):
        """Force (or moment) for one axis: drag feed-forward + PI on velocity.

        The feed-forward is what makes the commanded speed the achieved speed. A
        P term alone has to hold a standing velocity error to produce any force
        at all, so the vehicle always settles well short of what you asked for -
        which reads, from the cockpit, as a low speed cap.
        """
        linear, quadratic = self.get_parameter(drag_name).value
        error = command - actual
        self.trim[axis] = float(np.clip(
            self.trim[axis] + error * self.get_parameter(ki_name).value * dt,
            -trim_limit, trim_limit))
        return (linear * command + quadratic * abs(command) * command
                + error * self.get_parameter(kp_name).value + self.trim[axis])

    def reset_trim(self):
        for axis in self.trim:
            self.trim[axis] = 0.0
        for axis in self.ramp:
            self.ramp[axis] = 0.0

    def slew(self, axis, target, limit, dt):
        """Move a velocity command towards its target no faster than `limit`."""
        step = limit * dt
        self.ramp[axis] = float(np.clip(target, self.ramp[axis] - step,
                                        self.ramp[axis] + step))
        return self.ramp[axis]

    def mix(self, wrench):
        """Split a net body wrench across the eight thrusters.

        Scaling the whole solution rather than clipping each thruster matters:
        clip them independently and asking for full forward plus full yaw gives
        a curve you never commanded. Horizontal and vertical are scaled
        separately because they use disjoint thrusters, so a saturated dive has
        no business slowing the surge down.
        """
        limit = self.get_parameter('max_thrust').value
        forces = MIX @ np.asarray(wrench, dtype=float)

        horizontal, vertical = forces[:4], forces[4:]
        peak = float(np.max(np.abs(horizontal)))
        saturated = {'horizontal': peak > limit,
                     'vertical': float(np.max(np.abs(vertical))) > limit}
        if peak > limit:
            horizontal = horizontal * (limit / peak)
        vertical = np.clip(vertical, -limit, limit)

        return (dict(zip(THRUSTERS, np.concatenate([horizontal, vertical]))),
                saturated)

    def publish_thrust(self, values):
        limit = self.get_parameter('max_thrust').value
        for name, value in values.items():
            self.thrusters[name].publish(
                Float64(data=float(np.clip(value, -limit, limit))))

    def publish_status(self):
        _, mode = self.gear_scale()
        speed = 0.0
        if self.state is not None:
            speed = float(np.linalg.norm(self.state['body_vel'][:2]))
        self.status_pub.publish(String(data=json.dumps({
            'enabled': bool(self.enabled),
            'engaged': bool(self.engaged),
            'mode': mode,
            'gear': round(self.get_parameter('gear').value, 3),
            'surge_cmd': round(self.surge_cmd, 3),
            'sway_cmd': round(self.sway_cmd, 3),
            'yaw_cmd': round(self.yaw_cmd, 3),
            'speed': round(speed, 3),
            'depth': round(float(self.depth if self.depth is not None else 0.0), 3),
            'holding': bool(self.hold_xy is not None),
            'source': self.source,
            'keys': sorted(self.held),
            'cruise_speed': round(self.get_parameter('cruise_speed').value, 3),
            'cruise_yaw': round(self.get_parameter('cruise_yaw').value, 3),
        })))


def _wall():
    """Wall-clock seconds. Input freshness must not stall when sim time does."""
    return time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = AuvTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
