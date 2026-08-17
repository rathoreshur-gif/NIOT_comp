"""A minimal station-keeping controller for the Matsya AUV.

This is the reference implementation competitors are meant to replace. It does
the smallest thing that closes the loop: PID on the pose error, a fixed mixer to
the eight thrusters, and a saturation cap.

`auv_controller/controller_interpolate.py` in Matsya_ROS2 is the reference for
the *interface* — same topics, same message types. The control law here is
deliberately self-contained rather than shared with it: that node loads its gains
through `get_constants.py`, which hardcodes
`~/ros2_ws/src/Matsya_ROS2/auv_controller/config/` and reads the source tree
rather than the installed share directory, so it cannot run in a container
without mounting that exact path.

Units and frames, which are easy to get wrong here. All of this is set by
`auv_simbridge/simulator_bridge.py`, not by convention:

  * Positions are in CENTIMETRES: `run()` multiplies metres by 100, relative to
    an origin that `/localization/reset_service` resets.
  * Y and Z are negated relative to Gazebo (`odometry_callback`). Two flips is a
    180-degree rotation about X, so the frame stays right-handed and comes out
    NED-like: +x forward, +z DOWN. **A larger z means deeper.** Commanding a
    negative z asks the vehicle to rise.
  * Orientation is in DEGREES, not radians — `quaternion_to_euler` calls
    `as_euler('zyx', degrees=True)`. Roll/pitch/yaw are converted to radians on
    the way in here and everything below is radians; only the logs go back to
    degrees.
  * `velocity` is in cm/s (also scaled by 100) and is already body-frame.
    `angular_velocity` is in rad/s and is NOT scaled.
  * `/controller/setpoint` is interpreted in the same frame and units as the
    pose — centimetres and degrees — matching what `controller_interpolate`
    does.
  * Thruster forces are in newtons, capped at the +-40 N the real thrusters can
    produce (`auv_controller/config/constants.json`).

Everything reaches this node over the domain bridge; see
`Matsya_ROS2_Simulator/docker/domain_bridge.yaml` for what is available.
"""
import math

import numpy as np
import rclpy
from auv_msgs.msg import AuvState, Pose, ThrusterForces
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# Peak force one thruster can deliver in either direction, from
# auv_controller/config/constants.json (max_front_force / max_back_force).
MAX_THRUST_N = 40.0

# Mixer rows: how a unit of each body-frame axis maps onto the eight thrusters.
#
# Derived in closed form from the thruster geometry in constants.json rather
# than by pseudo-inverting a control-effectiveness matrix at runtime, because
# the layout is symmetric enough to make the coefficients exactly +-1.
#
# The four surge thrusters sit in a vectored X-configuration at +-0.785 rad
# (45 degrees) with zero pitch, so each contributes cos(45) to both surge and
# sway; the signs below follow from their yaw angles. The four heave thrusters
# have pitch 1.57, so they point straight up and contribute only to heave, roll
# and pitch. Using the same direction convention as auv_controller/allocator.py:
# [cos(pitch)cos(yaw), sin(yaw)cos(pitch), sin(pitch)].
#
# Verified against the full control-effectiveness matrix built from
# constants.json: a unit command on each row produces a wrench on that axis
# alone. The one real cross-coupling is surge -> pitch (and sway -> roll), worth
# about 0.07 N.m per newton, because the surge thrusters sit 0.075 m below the
# centre of mass. That is physics, not a mixer error, and with roll/pitch left
# uncommanded it is the hull's own righting moment that absorbs it.
#
# Column order throughout:
#   0 surge_front_left   1 surge_front_right   2 surge_back_left   3 surge_back_right
#   4 heave_front_left   5 heave_front_right   6 heave_back_left   7 heave_back_right
MIXER = np.array([
    # surge  (+x forward)
    [1.0,  1.0,  1.0,  1.0,   0.0,  0.0,  0.0,  0.0],
    # sway   (+y right)
    [1.0, -1.0, -1.0,  1.0,   0.0,  0.0,  0.0,  0.0],
    # heave  (+z, sign resolved by the heave_sign parameter)
    [0.0,  0.0,  0.0,  0.0,   1.0,  1.0,  1.0,  1.0],
    # roll   (unused by default: the roll gains are zero, see body_error)
    [0.0,  0.0,  0.0,  0.0,  -1.0,  1.0, -1.0,  1.0],
    # pitch  (unused by default, likewise)
    [0.0,  0.0,  0.0,  0.0,  -1.0, -1.0,  1.0,  1.0],
    # yaw
    [1.0, -1.0,  1.0, -1.0,   0.0,  0.0,  0.0,  0.0],
])

AXES = ('surge', 'sway', 'heave', 'roll', 'pitch', 'yaw')


def wrap_angle(angle: float) -> float:
    """Fold an angle into [-pi, pi] so yaw error takes the short way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


class BasicController(Node):
    """PID station-keeping on /controller/setpoint.

    Four controlled axes — surge, sway, heave and yaw. Roll and pitch are left
    to the vehicle's passive righting moment; see body_error for why.
    """

    def __init__(self) -> None:
        super().__init__('basic_controller')

        # Gains are per-axis in AXES order. The translational ones look small
        # because the position error arrives in centimetres; the angular ones
        # look large because the error is in radians by the time it gets here.
        #
        # Roll and pitch are zero on every term: those axes are left to the
        # vehicle's own righting moment (see body_error). They stay in the arrays
        # rather than being dropped so the six-element AXES ordering holds
        # throughout, and so raising them is all it takes to control them again.
        self.declare_parameter('kp', [0.25, 0.25, 0.45, 0.00, 0.00, 30.0])
        self.declare_parameter('ki', [0.005, 0.005, 0.02, 0.00, 0.00, 0.50])
        self.declare_parameter('kd', [0.90, 0.90, 1.10, 0.00, 0.00, 8.00])
        # Ceiling on each axis' integral contribution, in newtons. Without it
        # the depth integrator winds up while the vehicle is still descending.
        # Heave gets the largest budget because the vehicle is positively
        # buoyant and needs a standing downward force to hold depth: at 12 N the
        # integrator saturated and left ~5 cm of steady-state depth error.
        self.declare_parameter('i_limit', [8.0, 8.0, 25.0, 0.0, 0.0, 4.0])
        self.declare_parameter('rate_hz', 20.0)
        # Resolved by observation on the first run: simulator_bridge negates
        # heave on its way to Gazebo and AuvState z is already sign-flipped, so
        # reading the code is not enough to be sure which way is up.
        self.declare_parameter('heave_sign', 1.0)
        # The controller does not use the camera streams; it only counts frames
        # so that a bridged-camera problem shows up in the log instead of
        # silently looking like a slow simulator.
        self.declare_parameter('log_camera_rate', True)

        self.kp = np.array(self.get_parameter('kp').value, dtype=float)
        self.ki = np.array(self.get_parameter('ki').value, dtype=float)
        self.kd = np.array(self.get_parameter('kd').value, dtype=float)
        self.i_limit = np.array(self.get_parameter('i_limit').value, dtype=float)
        self.heave_sign = float(self.get_parameter('heave_sign').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        # World frame, centimetres and radians: [x, y, z, roll, pitch, yaw].
        # Radians here even though the wire carries degrees; converted on ingest.
        self.pose = np.zeros(6, dtype=float)
        # Body-frame linear velocity in cm/s, straight from AuvState.
        self.velocity = np.zeros(3, dtype=float)
        # Body-frame angular velocity in rad/s, also straight from AuvState.
        self.angular_velocity = np.zeros(3, dtype=float)
        # Hold position until someone sends a setpoint. Staying put is a safer
        # default than driving to the origin from wherever the vehicle spawned.
        self.setpoint = None
        self.integral = np.zeros(6, dtype=float)
        self.have_pose = False

        self.thrust_pub = self.create_publisher(
            ThrusterForces, '/controller/thruster_forces', 10)

        self.create_subscription(AuvState, '/localization/pose', self.cb_pose, 10)
        self.create_subscription(Pose, '/controller/setpoint', self.cb_setpoint, 10)

        self.frame_counts = {'front': 0, 'bottom': 0}
        if self.get_parameter('log_camera_rate').value:
            # Sensor-data QoS on this side is safe regardless of what the bridge
            # republishes with: a best-effort subscription matches both a
            # best-effort and a reliable publisher.
            self.create_subscription(
                Image, '/front_camera/image_raw',
                lambda _msg: self._count('front'), qos_profile_sensor_data)
            self.create_subscription(
                Image, '/bottom_camera/image_raw',
                lambda _msg: self._count('bottom'), qos_profile_sensor_data)
            self.create_timer(5.0, self.log_camera_rate)

        self.dt = 1.0 / rate_hz
        self.create_timer(self.dt, self.run)

        self.get_logger().info(
            f'basic_controller up at {rate_hz:.0f} Hz, heave_sign='
            f'{self.heave_sign:+.0f}. Waiting for /localization/pose.')

    # ---------------------------------------------------------------- inputs

    def cb_pose(self, msg: AuvState) -> None:
        self.pose[0] = msg.position.x
        self.pose[1] = msg.position.y
        self.pose[2] = msg.position.z
        # AuvState carries degrees; everything below this line is radians.
        self.pose[3] = math.radians(msg.orientation.roll)
        self.pose[4] = math.radians(msg.orientation.pitch)
        self.pose[5] = math.radians(msg.orientation.yaw)

        # Already body-frame linear velocity, so it needs no rotation. Same
        # source controller_interpolate uses for its derivative term.
        self.velocity[0] = msg.velocity.x
        self.velocity[1] = msg.velocity.y
        self.velocity[2] = msg.velocity.z

        # Body-frame angular rate, already in rad/s and unscaled.
        self.angular_velocity[0] = msg.angular_velocity.x
        self.angular_velocity[1] = msg.angular_velocity.y
        self.angular_velocity[2] = msg.angular_velocity.z

        if not self.have_pose:
            self.have_pose = True
            self.get_logger().info(
                'First pose received: '
                f'x={self.pose[0]:.1f} y={self.pose[1]:.1f} z={self.pose[2]:.1f} cm, '
                f'yaw={math.degrees(self.pose[5]):.1f} deg')

    def cb_setpoint(self, msg: Pose) -> None:
        # Same units as the pose stream: centimetres, and degrees converted to
        # radians so the rest of the node stays in one system.
        self.setpoint = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z,
            math.radians(msg.orientation.roll),
            math.radians(msg.orientation.pitch),
            math.radians(msg.orientation.yaw),
        ], dtype=float)
        # A new target invalidates whatever the integrators accumulated for the
        # old one; carrying it over shows up as an overshoot on arrival.
        self.integral[:] = 0.0
        self.get_logger().info(
            f'New setpoint: x={self.setpoint[0]:.1f} y={self.setpoint[1]:.1f} '
            f'z={self.setpoint[2]:.1f} cm, yaw={math.degrees(self.setpoint[5]):.1f} deg')

    def _count(self, camera: str) -> None:
        self.frame_counts[camera] += 1

    def log_camera_rate(self) -> None:
        front = self.frame_counts['front'] / 5.0
        bottom = self.frame_counts['bottom'] / 5.0
        self.frame_counts['front'] = 0
        self.frame_counts['bottom'] = 0
        if front == 0.0 and bottom == 0.0:
            self.get_logger().warn(
                'No camera frames in the last 5 s — check the domain bridge.')
        else:
            self.get_logger().info(
                f'Camera rate: front {front:.1f} Hz, bottom {bottom:.1f} Hz')

    # --------------------------------------------------------------- control

    def body_error(self) -> np.ndarray:
        """Pose error expressed in the body frame, in AXES order."""
        error = np.zeros(6, dtype=float)

        world_delta = self.setpoint[0:3] - self.pose[0:3]

        # Rotate the horizontal error into the body frame. Only yaw matters:
        # the vehicle is held level, so roll and pitch stay near zero and a full
        # rotation matrix would buy nothing here.
        yaw = self.pose[5]
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        error[0] = world_delta[0] * cos_yaw + world_delta[1] * sin_yaw   # surge
        error[1] = -world_delta[0] * sin_yaw + world_delta[1] * cos_yaw  # sway
        error[2] = world_delta[2]                                        # heave

        # Roll and pitch are deliberately left uncommanded: error[3] and error[4]
        # stay at the zero they were initialised with, so no roll or pitch torque
        # is ever requested.
        #
        # The vehicle rights itself. Its centre of buoyancy sits above its centre
        # of mass, so the two forces form a righting couple that pulls it level
        # whenever it tips — a passive restoring moment that needs no thrust.
        # Actively levelling on top of that spent the same four heave thrusters
        # on a job gravity was already doing, and those thrusters are the only
        # ones that produce heave, so every newton of roll/pitch correction came
        # straight out of the depth budget.
        #
        # The trade is a small steady pitch offset while surging: the surge
        # thrusters sit 0.075 m below the centre of mass, so hard surge leans the
        # vehicle by roughly 0.07 N.m per newton until the righting moment
        # balances it. That settles on its own and does not accumulate.
        error[5] = wrap_angle(self.setpoint[5] - self.pose[5])

        return error

    def run(self) -> None:
        if not self.have_pose:
            return

        if self.setpoint is None:
            # Nothing commanded yet: hold station where we first saw ourselves,
            # facing the way we started.
            self.setpoint = self.pose.copy()
            self.get_logger().info(
                'No setpoint yet — holding the pose we started at.')

        error = self.body_error()

        # Integral with per-axis clamping. Clamping the accumulated force rather
        # than the raw error keeps the limit meaningful in newtons.
        self.integral += error * self.dt
        i_term = np.clip(self.ki * self.integral, -self.i_limit, self.i_limit)

        # Derivative from measured velocity, not from differencing the error.
        # The pose stream is noisy enough that differencing it produces a term
        # dominated by noise, and AuvState already carries the velocity.
        derivative = np.zeros(6, dtype=float)
        derivative[0:3] = -self.velocity
        derivative[3:6] = -self.angular_velocity

        wrench = self.kp * error + i_term + self.kd * derivative
        wrench[2] *= self.heave_sign

        self.publish_thrust(self.allocate(wrench))

    def allocate(self, wrench: np.ndarray) -> np.ndarray:
        """Map a body-frame wrench onto the eight thrusters, saturation-safe."""
        thrust = wrench @ MIXER

        # Scale the whole vector down rather than clipping element-wise, so a
        # saturated axis does not silently rotate the resulting force.
        peak = np.max(np.abs(thrust))
        if peak > MAX_THRUST_N:
            thrust *= MAX_THRUST_N / peak

        return thrust

    def publish_thrust(self, thrust: np.ndarray) -> None:
        msg = ThrusterForces()
        msg.surge_front_left = float(thrust[0])
        msg.surge_front_right = float(thrust[1])
        msg.surge_back_left = float(thrust[2])
        msg.surge_back_right = float(thrust[3])
        msg.heave_front_left = float(thrust[4])
        msg.heave_front_right = float(thrust[5])
        msg.heave_back_left = float(thrust[6])
        msg.heave_back_right = float(thrust[7])
        self.thrust_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BasicController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the thrusters at zero rather than at whatever the last command
        # was; the simulator holds the previous value indefinitely otherwise.
        try:
            node.publish_thrust(np.zeros(8))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
