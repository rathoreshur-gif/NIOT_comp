"""Vehicle-frame gamepad teleop for the NIOT competition simulator.

Built for driving the AUV by hand at a demo: an Xbox-layout pad flies it like a
game. The left stick moves the vehicle relative to its own nose, the right stick
points the nose, and letting go stops it dead instead of letting it drift.

This is a *demo* node. It shares nothing with basic_controller.py, which stays
the competition entry; both publish /controller/thruster_forces, so only one of
them may run at a time. gamepad.launch.py starts this one on its own.

    Left stick        surge / sway, IN THE VEHICLE FRAME - push forward and it
                      goes where the nose points, push left and it strafes to
                      its own left, whichever way it happens to be facing.
    Right stick X     yaw: swing the nose left / right
    Right stick Y     depth: rise / dive
    RT                boost, analogue - squeeze for more speed
    LT                precision, analogue - squeeze for fine control
    L1+L2+R1+R2       all four together: next gear, SLOW -> NORMAL -> FAST ->
                      TURBO -> SLOW. The name shows on the Gazebo scoreboard.
    A                 drop the marker; press again to pick it back up
    Y                 fire the torpedo; press again to reload it
    X                 gripper: toggle open / closed
    LB                re-anchor: pin the hold point here (unless the triggers
                      are squeezed, when it is half of the gear chord)
    D-pad up/down     one gear up / down, the same four, clamped at the ends
    D-pad left        back to NORMAL
    START             start the run
    BACK              end the run

Resetting the run is deliberately NOT on the pad. It re-poses the vehicle and
re-seeds the whole course, which is a thing to do between competitors and not
with a thumb mid-run; `./demo.sh reset` does it from the operator's keyboard.

The payload buttons need ROS SERVICES (/simulator/drop_marker_left and the two
gripper calls), and domain_bridge carries topics and not services - so they only
work with this node on the simulator's own domain. docker-compose.demo.yml puts
it there; on the competition domain the three buttons log a warning and do
nothing, and everything else still flies.

Release every stick and the vehicle FREEZES - position, heading and depth are
pinned where they were at the instant of release and the controller brakes back
to them. Nothing coasts and nothing floats up, which is what makes it drivable
in front of an audience.

Control is the same cascade auv_simbridge/teleop.py uses for keyboard driving,
because it is already tuned for this hull: the sticks ask for a body-frame
velocity, each axis turns that into a force with the model's own drag curve fed
forward plus PI on the velocity error, and the resulting (Fx, Fy, Fz, Mz) is
split across the eight thrusters through the real allocation matrix. The
feed-forward is what makes the commanded speed the achieved speed; with P alone
the vehicle has to hold a standing velocity error to make any thrust at all,
which from the cockpit reads as a low speed cap.

What is NOT shared with that node is the plumbing at either end. teleop.py runs
inside the simulator and writes /model/auv/joint/*/cmd_thrust directly. This one
runs in the competitor container on ROS_DOMAIN_ID=1 and is only allowed the two
topics the domain bridge carries: /controller/thruster_forces out, and
/simulator/run_control for the run lifecycle. So the frames have to be converted
on the way in and the heave sign flipped on the way out - see FRAMES below.

The pad is read straight from /dev/input/js0 with the Linux joystick API rather
than through the `joy` package, so the competitor image needs no new dependency
and no rebuild of its rosdep layer. Unplugging the pad mid-run is safe: the node
freezes the vehicle and reconnects by itself when it comes back.

FRAMES. Three conventions meet in this file and mixing them up produces a
controller that looks plausible and quietly diverges:

  * AuvState, on the wire, is what simulator_bridge publishes: position in
    CENTIMETRES relative to a reset origin, orientation in DEGREES, and Y and Z
    negated from Gazebo so +z is DOWN.
  * `AuvState.velocity` is in the WORLD frame, not the body frame.
    simulator_bridge builds it as `rotation.apply(twist.linear)` - a body-to-
    world rotation - before negating y and z (simulator_bridge.py, in
    odometry_callback). The README and basic_controller.py both describe it as
    body-frame; they are wrong about it, and this node has to rotate it itself
    because a velocity loop fed world-frame feedback fights itself the moment
    the vehicle is not pointing down the world x axis.
  * Everything below `_ingest` works in the Gazebo convention instead - metres,
    radians, x forward, y LEFT, z UP - so that teleop.py's allocation matrix,
    drag coefficients and gains carry over unchanged. `_publish_thrust` converts
    back on the way out.
"""
import glob
import json
import math
import os
import struct
import time

import numpy as np
import rclpy
from auv_msgs.msg import AuvState, RunState, ThrusterForces
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptySrv

# -- the pad ----------------------------------------------------------------
#
# Axis and button numbers for the Linux joystick interface as the kernel's xpad
# driver presents an Xbox-layout pad. The Zebronics pad this was written for
# enumerates as "Microsoft X-Box 360 pad" in XInput mode, which is this layout;
# check with `jstest /dev/input/js0` if a different pad behaves oddly, and
# override with the axis_* / button_* parameters rather than editing this.
AX_LEFT_X, AX_LEFT_Y = 0, 1
AX_LT = 2
AX_RIGHT_X, AX_RIGHT_Y = 3, 4
AX_RT = 5
AX_DPAD_X, AX_DPAD_Y = 6, 7

BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB = 4, 5
BTN_BACK, BTN_START = 6, 7

# -- speed modes ------------------------------------------------------------
#
# Four gears with names, cycled by squeezing all four shoulders at once. A named
# mode beats a floating trim at a stand: "you are in TURBO" is something a
# competitor can be told across a noisy room, and it is on the scoreboard where
# the crowd can see it too.
#
# The numbers are gear multipliers on cruise_speed (1.35 m/s), and the top one
# is the hull's REAL ceiling, not a wish. Four surge thrusters at 45 degrees and
# max_thrust 75 N give Fx = 4 * 0.707 * 75 = 212 N; against the model's own drag
# curve, 130.7v + 14.5v^2 = 212 solves to v = 1.40 m/s, which is exactly
# max_linear. Asking for more than that does not go faster, it just parks the
# velocity loop in saturation and takes the yaw authority down with it - the
# four surge thrusters do both. So TURBO sits at the ceiling and the rest are
# honest fractions of it.
SPEED_MODES = (
    ('SLOW',   0.35),   # 0.47 m/s - a first-timer, or a crowded table
    ('NORMAL', 0.65),   # 0.88 m/s - the default; enough to get about, hard to crash
    ('FAST',   0.85),   # 1.15 m/s
    ('TURBO',  1.05),   # 1.40 m/s - everything the thrusters have
)
DEFAULT_MODE = 1        # NORMAL

# How far the triggers must be squeezed to count as part of the shoulder chord.
# Well past the deadzone of a worn trigger, and well short of the stop, so the
# gesture is "squeeze both" rather than "squeeze both perfectly".
CHORD_TRIGGER = 0.55

# js_event: __u32 time, __s16 value, __u8 type, __u8 number.
JS_EVENT_FORMAT = '<IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80        # ORed into the type on the synthetic events the
                            # kernel queues at open to report current values

AXIS_MAX = 32767.0

# The triggers rest at -32767 and reach +32767 fully squeezed, so they need
# their own normalisation to 0..1. Confirmed by reading the init events off the
# device: axes 2 and 5 report -32767 at rest while every stick reports 0.
TRIGGER_AXES = (AX_LT, AX_RT)

# The four stick axes, which are the ones that are supposed to rest at zero and
# therefore the ones worth calibrating. The D-pad is a hat - it reports exactly
# 0 or +-32767 and has no centre to find - and the triggers have their own
# resting value of -32767, which is not an error to correct but the bottom of
# their range.
STICK_AXES = (AX_LEFT_X, AX_LEFT_Y, AX_RIGHT_X, AX_RIGHT_Y)

# A resting offset this large is not a worn stick, it is a stick being HELD
# while the node started, or an axis that is not a stick at all on this pad.
# Calibrating to it would bake a permanent lie into the centre, so past this
# the offset is refused and logged instead.
MAX_STICK_OFFSET = 0.80

# -- the vehicle ------------------------------------------------------------
#
# Thruster order, matching the ThrusterForces field order and the columns of
# ALLOCATION below.
THRUSTERS = ('surge_front_left', 'surge_front_right',
             'surge_back_left', 'surge_back_right',
             'heave_front_left', 'heave_front_right',
             'heave_back_left', 'heave_back_right')

# Per-thruster force to net body wrench (Fx, Fy, Fz, Mz), in the Gazebo
# convention. The four surge thrusters are a vectored X at 45 degrees; the four
# heave thrusters point straight up. Lifted from auv_simbridge/teleop.py, which
# read it off the joint axes and link poses in the AUV model. The yaw arm is
# only 0.112 m, which is why yaw is the axis that saturates first.
ALLOCATION = np.array([
    [0.707,  0.707,  0.707,  0.707, 0.0, 0.0, 0.0, 0.0],    # Fx  forward
    [-0.707, 0.707,  0.707, -0.707, 0.0, 0.0, 0.0, 0.0],    # Fy  left
    [0.0,    0.0,    0.0,    0.0,   1.0, 1.0, 1.0, 1.0],    # Fz  up
    [-0.112, 0.112, -0.112,  0.112, 0.0, 0.0, 0.0, 0.0],    # Mz  yaw
])
# Least-norm solution: the thrust split that makes a wanted wrench with as
# little total force as possible.
MIX = np.linalg.pinv(ALLOCATION)


def _wall():
    """Wall-clock seconds. Pad freshness must not stall when sim time does."""
    return time.monotonic()


def _wrap(angle):
    """Fold an angle into [-pi, pi] so heading error takes the short way."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Joystick:
    """Non-blocking reader for a Linux joystick device.

    Deliberately not the `joy` package: this keeps the competitor image's
    dependency list untouched, so switching to the gamepad is a compose overlay
    rather than an image rebuild.

    A pad that is unplugged, or that was never there, is not an error. `alive`
    goes false, the caller freezes the vehicle, and `poll` retries the open
    every `retry_period` seconds until it comes back.

    The retry re-resolves the path rather than reusing it, because a pad that is
    unplugged and plugged back in does not reliably come back on the same node:
    the kernel hands out the lowest free js* number, so js0 becomes js1 if
    anything else claimed it in between. When the configured device is missing,
    any other js* is taken instead - at a stand there is only ever one pad, and
    a demo that needs someone to notice a number changed is a demo that stops.

    STICK CENTRES ARE CALIBRATED, not assumed. The kernel queues the current
    value of every axis as a synthetic event the moment the device is opened, so
    the first drain after an open says where this pad's sticks actually rest -
    and on a pad that has been through a few hundred hands, that is not zero.
    The one this was written against rests its right stick X at -17933 of 32767,
    a permanent 55% deflection, which through the deadzone and the expo curve is
    a standing yaw command of 0.14 rad/s: the vehicle turns on its own, forever,
    with the pad face down on the table. Subtracting the measured centre is what
    makes "hands off" mean stationary. See `calibrate` and `axis`.
    """

    def __init__(self, path, retry_period=1.0, discover=True):
        self.path = path
        self.discover = discover
        self.retry_period = retry_period
        self.fd = None
        self.active_path = None     # what is actually open, which may not be
                                    # `path` after a re-plug
        self.axes = {}
        self.buttons = {}
        self.pressed = set()        # rising edges since the last drain
        self.last_open_attempt = 0.0
        self.was_alive = False
        # Measured resting position of each stick axis, in raw counts. Empty
        # until the first drain after an open; missing means "assume zero",
        # which is what a healthy pad reports anyway.
        self.centres = {}
        self.needs_calibration = False
        # Set by calibrate() for the caller to log - the reader itself has no
        # logger, and a 55% offset is something a booth operator should be told.
        self.calibration_note = None

    @property
    def alive(self):
        return self.fd is not None

    def _resolve(self):
        """The device to open: the configured one, or any pad that is there."""
        if os.path.exists(self.path):
            return self.path
        if not self.discover:
            return None
        candidates = sorted(glob.glob('/dev/input/js*'))
        return candidates[0] if candidates else None

    def _open(self):
        now = _wall()
        if now - self.last_open_attempt < self.retry_period:
            return
        self.last_open_attempt = now
        path = self._resolve()
        if path is None:
            return
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            self.active_path = path
            # The kernel is about to queue the current value of every axis as
            # synthetic events. The first drain that reads them is where the
            # centres come from.
            self.needs_calibration = True
        except OSError:
            self.fd = None

    def _close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        # Do not keep stale stick values around: a pad that vanished mid-push
        # would otherwise leave the vehicle driving at whatever it was last
        # told to do.
        self.axes.clear()
        self.buttons.clear()
        # A different pad may come back on this path, so its centres are not
        # this pad's centres.
        self.centres.clear()

    def poll(self):
        """Drain every event the pad has queued. Returns True if it is live."""
        if self.fd is None:
            self._open()
            if self.fd is None:
                return False

        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break                       # nothing more queued right now
            except OSError:
                self._close()               # unplugged
                return False
            if len(data) < JS_EVENT_SIZE:
                break

            _, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
            initial = bool(event_type & JS_EVENT_INIT)
            event_type &= ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                self.axes[number] = value
            elif event_type == JS_EVENT_BUTTON:
                was = self.buttons.get(number, 0)
                self.buttons[number] = value
                # The synthetic events queued at open carry current state, not
                # a press. Acting on them would fire every bound button once at
                # startup - including, memorably, "reset the run".
                if value and not was and not initial:
                    self.pressed.add(number)

        if self.needs_calibration and self.axes:
            self.calibrate()

        return True

    def calibrate(self):
        """Take the sticks' current position as centre.

        Called once per open, on the drain that reads the kernel's synthetic
        events - so it measures where the sticks rest, not where they were at
        some arbitrary later moment.

        An offset past MAX_STICK_OFFSET is refused rather than stored. At that
        size the reading is not a worn centre, it is somebody holding the stick
        as the node came up, or an axis that is not a stick on this pad; storing
        it would mean full deflection one way and nothing the other, forever,
        and no way to tell from the cockpit.
        """
        self.needs_calibration = False
        notes = []
        for axis in STICK_AXES:
            raw = self.axes.get(axis)
            if raw is None:
                continue
            offset = raw / AXIS_MAX
            if abs(offset) > MAX_STICK_OFFSET:
                notes.append(f'axis {axis} rests at {offset:+.2f}, past the '
                             f'{MAX_STICK_OFFSET:.2f} limit - NOT calibrated; '
                             'let go of the sticks and re-plug the pad')
                continue
            self.centres[axis] = raw
            if abs(offset) > 0.05:
                notes.append(f'axis {axis} centre {offset:+.2f}')
        self.calibration_note = '; '.join(notes) if notes else None

    def axis(self, number):
        """Stick axis as -1.0 .. 1.0, about this pad's measured centre.

        Each side of the centre is scaled to its own remaining travel, so full
        deflection still reads 1.0 in BOTH directions. Dividing by AXIS_MAX
        alone would be simpler and wrong: with a centre at -0.55, pushing that
        stick to its negative stop would only ever reach -0.45, and the vehicle
        would turn markedly better one way than the other.
        """
        raw = self.axes.get(number, 0)
        centre = self.centres.get(number, 0)
        span = (AXIS_MAX - centre) if raw >= centre else (AXIS_MAX + centre)
        if span <= 0:
            return 0.0
        return max(-1.0, min(1.0, (raw - centre) / span))

    def trigger(self, number):
        """Trigger axis as 0.0 .. 1.0, from its -32767 resting value."""
        if number not in self.axes:
            return 0.0
        return (self.axes[number] + AXIS_MAX) / (2.0 * AXIS_MAX)

    def held(self, number):
        return bool(self.buttons.get(number, 0))

    def take_pressed(self):
        """Rising edges since the last call, cleared as they are handed over."""
        edges = self.pressed
        self.pressed = set()
        return edges


class GamepadController(Node):
    """Fly the AUV from a gamepad, in the vehicle's own frame."""

    def __init__(self):
        super().__init__('gamepad_controller')

        p = self.declare_parameter
        p('device', '/dev/input/js0')

        # -- how fast it flies ------------------------------------------------
        # Same shape as teleop.py's gears. Commanding a speed the vehicle cannot
        # make just parks the velocity loop in saturation, so these are solved
        # against the drag curves rather than guessed - which is why raising the
        # speeds meant raising max_thrust with them:
        #   surge  1.35 m/s   -> 130.7v + 14.5v^2 = 203 N   -> 71.7 N per thruster
        #   yaw    0.90 rad/s ->   9.5w + 18w^2   = 23.1 Nm -> 51.6 N per thruster
        #   heave  0.50 m/s   ->  116v + 350v^2   = 146 N   -> 36.5 N per thruster
        # Surge and yaw share the same four thrusters, so full surge AND full
        # yaw together still saturates and the mixer scales it back. That is
        # correct - it turns while driving, just not at both extremes at once.
        p('cruise_speed', 1.35)         # m/s at full stick, no modifier
        p('cruise_yaw', 0.90)           # rad/s
        p('cruise_depth_rate', 0.35)    # m/s
        p('boost_scale', 1.75)          # RT fully squeezed
        p('precision_scale', 0.20)      # LT fully squeezed
        # Set from SPEED_MODES, not by hand: `speed_mode` below is the thing
        # that moves, and it writes this. Still a live parameter, so a curious
        # operator can `ros2 param set` a value between the gears.
        p('gear', SPEED_MODES[DEFAULT_MODE][1])
        p('speed_mode', DEFAULT_MODE)   # index into SPEED_MODES
        p('max_linear', 1.40)
        p('max_yaw', 1.00)
        p('max_depth_rate', 0.50)
        # Stick slop. These pads rest a little off centre and the vehicle must
        # not creep because of it.
        p('deadzone', 0.12)
        # Squaring the stick past the deadzone: fine control near centre, full
        # speed still available at the stop. 1.0 is linear.
        p('stick_expo', 2.0)
        # How fast a velocity command may change. Without it a stick flick is a
        # step input, the loop saturates and the vehicle overshoots.
        p('accel_limit', 1.50)          # m/s^2
        p('yaw_accel_limit', 3.00)      # rad/s^2

        # -- control ----------------------------------------------------------
        # DEMO ONLY, and above the competition envelope: 40 N is the ceiling in
        # the README and constants.json, and basic_controller holds to it. This
        # is 75 N because the 3x surge asked for at a stand needs 71.7 N per
        # thruster and would otherwise sit in saturation looking no faster.
        # teleop.py takes the same liberty at 60 N. Nothing here is scored, but
        # it does mean the demo shows a livelier vehicle than an entry can be.
        p('max_thrust', 75.0)
        # Velocity loops work in net body wrench: N per m/s, N.m per rad/s.
        p('surge_kp', 300.0)
        p('surge_ki', 15.0)
        p('yaw_rate_kp', 25.0)
        p('yaw_rate_ki', 3.0)
        p('heave_kp', 300.0)
        p('heave_ki', 20.0)
        # Drag feed-forward, out of the model's Hydrodynamics block: linear,
        # quadratic.
        p('drag_surge', [130.7, 14.5])
        p('drag_sway', [116.0, 216.0])
        p('drag_heave', [116.0, 350.0])
        p('drag_yaw', [9.5, 18.0])
        # Outer loops: error -> velocity command, for the freeze.
        p('position_kp', 1.6)
        p('yaw_kp', 3.0)
        p('depth_kp', 2.0)
        # Cap on the speed the freeze may ask for on the way back to the anchor.
        # Raised with max_linear: at 1.35 m/s the old 1.0 cap made "release to
        # stop" overshoot, because the brake could not ask for the speed the
        # vehicle was already carrying.
        p('brake_speed', 1.40)
        p('deadband', 0.02)             # m, inside this the hold stops nudging
        # Holds the vehicle against its own buoyancy, per heave thruster, in the
        # sim's cmd_thrust sign. simulator_bridge publishes exactly this value
        # on all four when it parks the vehicle.
        p('buoyancy_feedforward', -1.462)
        # Flip to -1.0 if the vehicle rises when told to dive. simulator_bridge
        # negates heave on its way to Gazebo and AuvState z is already
        # sign-flipped, so this is settled by observation, not by reading code.
        p('heave_sign', 1.0)

        # -- run lifecycle ----------------------------------------------------
        # The thrusters are dead until the run starts, so at a booth the START
        # button is the difference between a demo and a puzzled silence.
        # Pushing a stick is as clear a statement of intent as a button, so at a
        # stand it starts the run itself rather than telling a stranger to find
        # START. Set false to drive the lifecycle by hand.
        p('auto_start', True)
        p('pad_timeout', 0.50)          # s without a readable pad -> freeze
        p('rate_hz', 50.0)

        self.device_path = self.get_parameter('device').value
        self.pad = Joystick(self.device_path)

        # Freeze targets, in the Gazebo convention: metres, radians, z up.
        # Live values, not commands - each moves only while its axis is flown.
        self.hold_xy = None
        self.hold_yaw = None
        self.hold_z = None
        self.engaged = True             # B cuts this; the vehicle goes free
        self.trim = {'surge': 0.0, 'sway': 0.0, 'heave': 0.0, 'yaw': 0.0}
        self.ramp = {'surge': 0.0, 'sway': 0.0, 'yaw': 0.0}

        self.state = None
        self.last_time = None
        self.run_state = None
        self.last_nag = 0.0
        self.last_auto = 0.0
        self.commanded = {'surge': 0.0, 'sway': 0.0, 'yaw': 0.0, 'heave': 0.0}

        # -- payload ----------------------------------------------------------
        # All of it is ROS services on the simulator's domain, and each one
        # TOGGLES: the vehicle carries one marker and one torpedo, so pressing
        # the button again picks the payload back up instead of refusing. That
        # is simulator_bridge's decision, not this node's - it is the one that
        # knows whether the DetachableJoint is currently welded - so a press is
        # the same call either way.
        self.marker_client = self.create_client(
            EmptySrv, '/simulator/drop_marker_left')
        # The impulse behind a shot has to go through Gazebo's ApplyLinkWrench,
        # whose message type lives in ros_gz_interfaces - a package the
        # competitor image does not carry and should not have to.
        # simulator_bridge already imports it, already tracks the vehicle's
        # heading, and so aims the shot itself; firing needs no arguments.
        self.torpedo_client = self.create_client(
            EmptySrv, '/simulator/fire_torpedo_left')
        self.close_gripper_client = self.create_client(
            EmptySrv, '/simulator/close_gripper')
        self.open_gripper_client = self.create_client(
            EmptySrv, '/simulator/open_gripper')
        self.gripper_closed = False
        # Current gear, by index into SPEED_MODES. The `gear` parameter is
        # derived from it; this is the thing the chord moves.
        self.speed_mode = int(self.get_parameter('speed_mode').value) % len(SPEED_MODES)
        self.chord_latched = False      # one cycle per squeeze, not one per tick
        # Best guess at what is still aboard, for the log line and the status
        # topic. simulator_bridge holds the truth; this only ever mirrors the
        # calls this node made.
        self.marker_aboard = True
        self.torpedo_aboard = True

        self.thrust_pub = self.create_publisher(
            ThrusterForces, '/controller/thruster_forces', 10)
        self.run_pub = self.create_publisher(String, '/simulator/run_control', 10)
        # Not bridged to the simulator, but a handy `ros2 topic echo` at a booth
        # when the vehicle is doing something the driver did not expect.
        self.status_pub = self.create_publisher(String, '/gamepad/status', 10)
        # The gear name, for the scoreboard in the Gazebo window. Its own topic
        # rather than a field of the status blob above, because the score keeper
        # subscribes to this one and should not have to parse a demo node's
        # debug JSON to do it. Republished at the status rate, not only on
        # change: Gazebo Transport has no latching, and a panel that connects
        # late would otherwise show nothing until the next gear change.
        self.mode_pub = self.create_publisher(String, '/gamepad/speed_mode', 10)

        self.create_subscription(AuvState, '/localization/pose', self._ingest, 10)
        self.create_subscription(RunState, '/simulator/run_state',
                                 self._on_run_state, 10)

        # `gear` follows the mode, including a mode overridden at launch.
        self._apply_speed_mode()

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.control_step)
        self.create_timer(0.2, self.publish_status)

        self.get_logger().info(
            f'Gamepad teleop on {self.device_path}: left stick moves the '
            'vehicle in its own frame, right stick aims it, RT boost, LT '
            'precision, Y fires the torpedo, START begins the run, BACK ends '
            f'it. Gear {self.mode_name} - squeeze L1+L2+R1+R2 together for the '
            'next one. '
            'Release the sticks and it freezes.')
        if not self.pad.poll():
            self.get_logger().warn(
                f'No pad at {self.device_path} yet - waiting for it. Check the '
                'device is passed into the container.')

    # -- inputs -------------------------------------------------------------

    def _ingest(self, msg: AuvState):
        """AuvState -> the Gazebo convention this node controls in.

        Undoes everything simulator_bridge did on the way out: centimetres back
        to metres, degrees back to radians, and the Y/Z negation that made +z
        point down. See FRAMES in the module docstring.
        """
        x = msg.position.x / 100.0
        y = -msg.position.y / 100.0
        z = -msg.position.z / 100.0                 # +z is up again
        yaw = -math.radians(msg.orientation.yaw)

        # World-frame, despite the name and despite what the README says: the
        # bridge rotates the body twist into the world before negating y and z.
        world_vel = np.array([msg.velocity.x / 100.0,
                              -msg.velocity.y / 100.0,
                              -msg.velocity.z / 100.0])

        # Rotate it into the body frame, which is where the surge and sway loops
        # live and what makes the left stick mean "forward for the vehicle"
        # rather than "along the pool".
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        body_vel = np.array([
            world_vel[0] * cos_yaw + world_vel[1] * sin_yaw,
            -world_vel[0] * sin_yaw + world_vel[1] * cos_yaw,
            world_vel[2],
        ])

        self.state = {
            'xyz': np.array([x, y, z]),
            'yaw': yaw,
            'cos_yaw': cos_yaw,
            'sin_yaw': sin_yaw,
            'body_vel': body_vel,
            'world_vel': world_vel,
            'yaw_rate': -msg.angular_velocity.z,    # negated with y and z
        }
        if self.hold_z is None:
            self.freeze()

    def _on_run_state(self, msg: RunState):
        was = self.run_state
        self.run_state = msg
        if was is None or was.state != msg.state:
            self.get_logger().info(
                f'Run {msg.state_label} (run {msg.run_index}, score {msg.score})')
            self.last_nag = 0.0

    def running(self):
        """Whether thrust actually reaches the water right now."""
        return self.run_state is None or self.run_state.state == RunState.RUNNING

    # -- stick shaping ------------------------------------------------------

    def _stick(self, number, invert=False):
        """One stick axis, deadzoned and expo-shaped, as -1.0 .. 1.0."""
        value = self.pad.axis(number)
        if invert:
            value = -value
        dead = float(self.get_parameter('deadzone').value)
        magnitude = abs(value)
        if magnitude <= dead:
            return 0.0
        # Rescale so the axis still reaches 1.0 at the stop rather than losing
        # the deadzone off the top of its range.
        scaled = (magnitude - dead) / (1.0 - dead)
        shaped = scaled ** float(self.get_parameter('stick_expo').value)
        return math.copysign(min(shaped, 1.0), value)

    def gear_scale(self):
        """Speed multiplier from the triggers and the persistent gear."""
        scale = float(self.get_parameter('gear').value)
        boost = float(self.get_parameter('boost_scale').value)
        precision = float(self.get_parameter('precision_scale').value)
        # Analogue in both directions, so the driver can feather between them.
        scale *= 1.0 + self.pad.trigger(AX_RT) * (boost - 1.0)
        scale *= 1.0 - self.pad.trigger(AX_LT) * (1.0 - precision)
        return scale

    def _handle_buttons(self):
        for button in self.pad.take_pressed():
            if button == BTN_START:
                self.send_run('start')
            elif button == BTN_BACK:
                self.send_run('end')
            elif button == BTN_A:
                self.drop_marker()
            elif button == BTN_Y:
                self.fire_torpedo()
            elif button == BTN_X:
                self.toggle_gripper()
            elif button == BTN_LB:
                # Not while the triggers are in: LB is half of the gear chord,
                # and anchoring on the way into a gear change is confusing at
                # best. Squeezing a trigger is a clear enough statement that
                # this press is part of the gesture and not an anchor.
                if not self._triggers_in():
                    self.freeze(announce=True)

        # THE GEAR CHORD: L1 + L2 + R1 + R2 together, one step per squeeze.
        #
        # Held state rather than edges, because four inputs are never pressed on
        # the same tick - the gesture is "all four are down at once", whatever
        # order they arrived in. Latched until at least one is let go, or a gear
        # change that is held for a second would run through every mode.
        #
        # A two-handed chord is the point: this is the one control that changes
        # how everything else behaves, so it should be impossible to hit with a
        # thumb by accident, and unmistakable when it is meant.
        chord = (self.pad.held(BTN_LB) and self.pad.held(BTN_RB)
                 and self._triggers_in())
        if chord and not self.chord_latched:
            self.chord_latched = True
            self.cycle_speed_mode()
        elif not chord:
            self.chord_latched = False

        # D-pad Y is an axis on this driver: -32767 is up, +32767 is down. Edges
        # only, so holding it does not run through the modes. Same four gears as
        # the chord, one step at a time - the chord is the showpiece, this is the
        # one-handed way to the same place.
        # The edge test is "changed, and not centred" rather than "was zero":
        # a hat flicked straight from up to down without settling at centre in
        # between is one press, and the old test dropped it on the floor.
        dpad = self.pad.axes.get(AX_DPAD_Y, 0)
        previous = getattr(self, '_dpad_y', 0)
        self._dpad_y = dpad
        if dpad and dpad != previous:
            self.set_speed_mode(self.speed_mode + (1 if dpad < 0 else -1),
                                wrap=False)

        # D-pad left goes back to NORMAL, which is the gear anyone should be
        # handed the pad in.
        dpad_x = self.pad.axes.get(AX_DPAD_X, 0)
        previous_x = getattr(self, '_dpad_x', 0)
        self._dpad_x = dpad_x
        if dpad_x < 0 <= previous_x:
            self.set_speed_mode(DEFAULT_MODE, wrap=False)

    def _call(self, client, label):
        """Fire a std_srvs/Empty service and forget it.

        call_async, never a blocking call: the control loop runs at 50 Hz on
        this same executor, so waiting on a response here would stall the
        thrusters for as long as the simulator took to answer.
        """
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{label} is not available. On the competition domain these are '
                'behind the domain bridge, which carries no services - run the '
                'demo overlay to reach them.')
            return False
        client.call_async(EmptySrv.Request())
        return True

    def drop_marker(self):
        """A: drop the marker, or pick it back up if it has already gone."""
        if not self._call(self.marker_client, 'drop_marker'):
            return
        self.marker_aboard = not self.marker_aboard
        self.get_logger().info(
            'Marker reloaded.' if self.marker_aboard else 'Marker away.')

    def fire_torpedo(self):
        """Y: fire the torpedo, or reload it if it has already gone.

        simulator_bridge releases the tube and then applies a forward impulse
        along the vehicle's nose, so the torpedo runs about 3 m and coasts to a
        stop. Aim by pointing the vehicle: the shot goes wherever the nose is
        looking at the moment the trigger is pulled. Press Y again and the
        torpedo is teleported back into its tube for another go.
        """
        if not self._call(self.torpedo_client, 'fire_torpedo'):
            return
        self.torpedo_aboard = not self.torpedo_aboard
        self.get_logger().info(
            'Torpedo reloaded.' if self.torpedo_aboard else 'Torpedo away.')

    def toggle_gripper(self):
        """X: open the gripper if it is closed, close it if it is open.

        The simulator only welds a pickup on an explicit close, so brushing past
        an object with the fingers open never picks it up - which means this
        button is the whole grabbing mechanism.
        """
        closing = not self.gripper_closed
        client = self.close_gripper_client if closing else self.open_gripper_client
        if not self._call(client, 'close_gripper' if closing else 'open_gripper'):
            return
        self.gripper_closed = closing
        self.get_logger().info(f'Gripper {"closing" if closing else "opening"}.')

    def _triggers_in(self):
        """Both analogue triggers squeezed - the other half of the gear chord."""
        return (self.pad.trigger(AX_LT) > CHORD_TRIGGER
                and self.pad.trigger(AX_RT) > CHORD_TRIGGER)

    @property
    def mode_name(self):
        return SPEED_MODES[self.speed_mode][0]

    def cycle_speed_mode(self):
        """Next gear, wrapping TURBO back round to SLOW."""
        self.set_speed_mode(self.speed_mode + 1, wrap=True)

    def set_speed_mode(self, index, wrap=False):
        """Select a gear by index. Writes `gear`, announces, tells the panel.

        `wrap` is what separates the chord from the D-pad. The chord is a cycle
        and has to come back round or a one-gesture control could only ever go
        one way. The D-pad is a trim and clamps instead: pressing up at TURBO
        should stay at TURBO, not drop the vehicle to SLOW under someone's
        thumb mid-run.
        """
        count = len(SPEED_MODES)
        index = index % count if wrap else int(np.clip(index, 0, count - 1))
        if index == self.speed_mode:
            return
        self.speed_mode = index
        self._apply_speed_mode()

    def _apply_speed_mode(self):
        """Push the current mode into `gear`, the log and the panel.

        Separate from set_speed_mode so __init__ can call it: the mode may be
        overridden at launch, and `gear` has to follow it or the vehicle flies
        at one speed while the scoreboard names another.
        """
        name, gear = SPEED_MODES[self.speed_mode]
        self.set_parameters([
            rclpy.Parameter('gear', rclpy.Parameter.Type.DOUBLE, float(gear)),
            rclpy.Parameter('speed_mode', rclpy.Parameter.Type.INTEGER,
                            self.speed_mode),
        ])
        self.publish_speed_mode()
        self.get_logger().info(
            f'Speed mode {name} '
            f'({gear * float(self.get_parameter("cruise_speed").value):.2f} m/s)')

    def publish_speed_mode(self):
        self.mode_pub.publish(String(data=self.mode_name))

    def send_run(self, command):
        self.run_pub.publish(String(data=command))
        self.get_logger().info(f'Run control: {command}')
        if command in ('start', 'reset'):
            # Whatever the integrators built up while the thrusters were dead is
            # meaningless now, and letting it out at the start of a run is a
            # visible lurch.
            self.reset_trim()
            self.freeze()

    def toggle_cut(self):
        """Cut or restore thrust. Not on a button at all any more.

        Kept because it is still reachable over ROS and is the right thing to
        call if the vehicle needs to go limp, and because control_step still
        honours `engaged`. It was bound to B until B, one thumb-slip from A,
        silently killed a demo; B then fired the torpedo, which now lives on Y.
        """
        self.engaged = not self.engaged
        if self.engaged:
            self.freeze()
            self.get_logger().info('Controller re-engaged.')
        else:
            self.reset_trim()
            self.publish_zero()
            self.get_logger().info('Thrust cut; vehicle is free.')

    # -- hold ---------------------------------------------------------------

    def freeze(self, announce=False):
        """Pin position, heading and depth wherever the vehicle is now."""
        self.engaged = True
        if self.state is not None:
            self.hold_xy = self.state['xyz'][:2].copy()
            self.hold_yaw = self.state['yaw']
            self.hold_z = float(self.state['xyz'][2])
        self.reset_trim()
        if announce:
            self.get_logger().info('Anchored here.')

    def reset_trim(self):
        for axis in self.trim:
            self.trim[axis] = 0.0
        for axis in self.ramp:
            self.ramp[axis] = 0.0

    # -- control ------------------------------------------------------------

    def control_step(self):
        pad_live = self.pad.poll()
        if pad_live != self.pad.was_alive:
            self.pad.was_alive = pad_live
            if pad_live:
                self.get_logger().info(f'Pad connected on {self.pad.active_path}.')
            else:
                self.get_logger().warn('Pad gone - freezing until it is back.')
                self.freeze()

        # Whatever calibrate() found on this pad, said out loud once. A stick
        # that rests off centre is invisible from the cockpit - the vehicle just
        # creeps - so the log line is the only place anyone finds out.
        if self.pad.calibration_note is not None:
            self.get_logger().warn(
                f'Pad stick centres: {self.pad.calibration_note}. Corrected for '
                'automatically; if the vehicle still creeps, the pad was being '
                'held as the node started - let go and re-plug it.')
            self.pad.calibration_note = None

        if pad_live:
            self._handle_buttons()

        if self.state is None:
            return

        # A thrust cut is undone by simply flying again. B is one thumb-slip
        # from A on this pad, and a driver who has cut it by accident and then
        # pushes the stick means "go" - not "stay dead until you remember which
        # button un-sticks it". Discovered the hard way, mid-demo.
        if not self.engaged:
            if pad_live and self._any_stick():
                self.freeze()       # sets engaged, and anchors where we drifted to
                self.get_logger().info('Stick input - controller re-engaged.')
            else:
                self.nag('Thrust is CUT - press B, or just push a stick.')
                return

        # While the run is IDLE or FINISHED the simulator drops every thrust
        # message, and the vehicle floats up on its own buoyancy. Re-anchoring
        # each tick means `start` begins holding wherever it has drifted to,
        # instead of hauling back to a boot-time pose nobody asked for.
        if not self.running():
            self.freeze()
            if self._any_stick():
                self.want_to_fly()
            self.publish_zero()
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_time is None else now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            return

        state = self.state
        scale = self.gear_scale()

        # A pad that has stopped reporting is not a pad holding centre: freeze
        # rather than fly on stale values.
        if not pad_live:
            surge_in = sway_in = yaw_in = vertical_in = 0.0
        else:
            # THE VEHICLE FRAME. Left stick up is +surge along the nose; left
            # stick left is +sway to the vehicle's own left (Gazebo body +y is
            # left, and the pad reports right as positive, so it is negated).
            surge_in = self._stick(AX_LEFT_Y, invert=True)
            sway_in = -self._stick(AX_LEFT_X)
            # Right stick right should swing the nose right, which is negative
            # yaw about a z-up axis.
            yaw_in = -self._stick(AX_RIGHT_X)
            # Right stick up rises. Depth is the one axis that is not relative
            # to the vehicle: up is up however the hull is pointing.
            vertical_in = self._stick(AX_RIGHT_Y, invert=True)

        speed = float(np.clip(
            float(self.get_parameter('cruise_speed').value) * scale,
            0.0, float(self.get_parameter('max_linear').value)))
        yaw_speed = float(np.clip(
            float(self.get_parameter('cruise_yaw').value) * scale,
            0.0, float(self.get_parameter('max_yaw').value)))
        depth_speed = float(np.clip(
            float(self.get_parameter('cruise_depth_rate').value) * scale,
            0.0, float(self.get_parameter('max_depth_rate').value)))

        accel = float(self.get_parameter('accel_limit').value)
        brake = float(self.get_parameter('brake_speed').value)
        deadband = float(self.get_parameter('deadband').value)

        # -- horizontal: fly on command, otherwise brake back to the anchor ---
        if surge_in or sway_in:
            self.hold_xy = None
            surge_cmd = self.slew('surge', speed * surge_in, accel, dt)
            sway_cmd = self.slew('sway', speed * sway_in, accel, dt)
        else:
            if self.hold_xy is None:
                # Just released: the anchor is wherever we are at this instant.
                self.hold_xy = state['xyz'][:2].copy()
            error = self.hold_xy - state['xyz'][:2]
            if np.linalg.norm(error) < deadband:
                error = np.zeros(2)
            # Into the vehicle frame, the same rotation the velocity got.
            gain = float(self.get_parameter('position_kp').value)
            forward = error[0] * state['cos_yaw'] + error[1] * state['sin_yaw']
            left = -error[0] * state['sin_yaw'] + error[1] * state['cos_yaw']
            # No slew on the brake: it has to bite at once, or "release to
            # freeze" becomes "release to coast".
            surge_cmd = float(np.clip(forward * gain, -brake, brake))
            sway_cmd = float(np.clip(left * gain, -brake, brake))
            self.ramp['surge'], self.ramp['sway'] = surge_cmd, sway_cmd

        # -- yaw: fly on command, otherwise hold heading ----------------------
        if yaw_in:
            self.hold_yaw = None
            rate_cmd = self.slew('yaw', yaw_speed * yaw_in,
                                 float(self.get_parameter('yaw_accel_limit').value), dt)
        else:
            if self.hold_yaw is None:
                self.hold_yaw = state['yaw']
            error = _wrap(self.hold_yaw - state['yaw'])
            limit = float(self.get_parameter('max_yaw').value)
            rate_cmd = float(np.clip(
                error * float(self.get_parameter('yaw_kp').value), -limit, limit))
            self.ramp['yaw'] = rate_cmd

        # -- depth: fly on command, otherwise hold the last one ---------------
        limit = float(self.get_parameter('max_depth_rate').value)
        if vertical_in:
            self.hold_z = float(state['xyz'][2])    # follow, so release pins here
            vertical_cmd = depth_speed * vertical_in
        else:
            error = self.hold_z - state['xyz'][2]
            if abs(error) < deadband:
                error = 0.0
            vertical_cmd = float(np.clip(
                error * float(self.get_parameter('depth_kp').value), -limit, limit))

        # -- velocity loops: the wanted net body wrench -----------------------
        # Snapshot for anti-windup: if the thrusters cannot deliver what these
        # loops ask for, integrating the leftover only stores up a lurch for
        # when the demand comes back down.
        before = dict(self.trim)
        wrench = np.array([
            self.axis_effort('surge', surge_cmd, state['body_vel'][0], dt,
                             'surge_kp', 'surge_ki', 'drag_surge', 80.0),
            self.axis_effort('sway', sway_cmd, state['body_vel'][1], dt,
                             'surge_kp', 'surge_ki', 'drag_sway', 80.0),
            self.axis_effort('heave', vertical_cmd, state['world_vel'][2], dt,
                             'heave_kp', 'heave_ki', 'drag_heave', 80.0)
            # Four heave thrusters each carrying the buoyancy feed-forward.
            + 4.0 * float(self.get_parameter('buoyancy_feedforward').value),
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

        self.commanded = {'surge': surge_cmd, 'sway': sway_cmd,
                          'yaw': rate_cmd, 'heave': vertical_cmd}
        self.publish_thrust(forces)

    def want_to_fly(self):
        """Someone pushed a stick while the run was not live. Make it live.

        Nobody at a stand is going to read a log line telling them to press
        START - they have already picked the pad up and pushed, which says what
        they want. IDLE starts; FINISHED resets, which lands back in IDLE and
        starts on the next push, so the demo loops without anyone being told how.

        Rate-limited because run_state only arrives at 5 Hz, so without it the
        same command goes out dozens of times before the state catches up.
        """
        finished = (self.run_state is not None
                    and self.run_state.state == RunState.FINISHED)

        if not bool(self.get_parameter('auto_start').value):
            self.nag('Run is FINISHED - ask the operator for a reset.' if finished
                     else 'Thrusters are dead until the run starts - press START.')
            return

        now = _wall()
        if now - self.last_auto < 2.0:
            return
        self.last_auto = now
        self.send_run('reset' if finished else 'start')

    def nag(self, message):
        """Say why nothing is happening, on a wall-clock throttle.

        Throttled on wall time rather than sim time so it still arrives at a
        sensible rate when the real-time factor is low, and repeated rather than
        fired once because at a stand the person holding the pad is usually not
        the person who saw the log line go by.
        """
        now = _wall()
        if now - self.last_nag < 3.0:
            return
        self.last_nag = now
        self.get_logger().warn(message)

    def _any_stick(self):
        return any(self._stick(axis) for axis in
                   (AX_LEFT_X, AX_LEFT_Y, AX_RIGHT_X, AX_RIGHT_Y))

    def axis_effort(self, axis, command, actual, dt, kp_name, ki_name,
                    drag_name, trim_limit):
        """Force (or moment) for one axis: drag feed-forward + PI on velocity.

        The feed-forward is what makes the commanded speed the achieved speed. P
        alone has to hold a standing velocity error to produce any force at all,
        so the vehicle always settles short of what was asked for - which from
        the cockpit reads as a low speed cap.
        """
        linear, quadratic = self.get_parameter(drag_name).value
        error = command - actual
        self.trim[axis] = float(np.clip(
            self.trim[axis] + error * float(self.get_parameter(ki_name).value) * dt,
            -trim_limit, trim_limit))
        return (linear * command + quadratic * abs(command) * command
                + error * float(self.get_parameter(kp_name).value) + self.trim[axis])

    def slew(self, axis, target, limit, dt):
        """Move a velocity command towards its target no faster than `limit`."""
        step = limit * dt
        self.ramp[axis] = float(np.clip(target, self.ramp[axis] - step,
                                        self.ramp[axis] + step))
        return self.ramp[axis]

    def mix(self, wrench):
        """Split a net body wrench across the eight thrusters.

        Scaling the whole solution rather than clipping each thruster matters:
        clip them independently and full forward plus full yaw gives a curve
        nobody commanded. Horizontal and vertical scale separately because they
        use disjoint thrusters, so a saturated dive has no business slowing the
        surge down.
        """
        limit = float(self.get_parameter('max_thrust').value)
        forces = MIX @ np.asarray(wrench, dtype=float)

        horizontal, vertical = forces[:4], forces[4:]
        peak = float(np.max(np.abs(horizontal)))
        saturated = {'horizontal': peak > limit,
                     'vertical': float(np.max(np.abs(vertical))) > limit}
        if peak > limit:
            horizontal = horizontal * (limit / peak)
        vertical = np.clip(vertical, -limit, limit)

        return np.concatenate([horizontal, vertical]), saturated

    # -- output -------------------------------------------------------------

    def publish_thrust(self, forces):
        """Publish eight thruster forces, converting out of the Gazebo sign.

        `forces` is in the sim's own cmd_thrust convention, which is what the
        allocation matrix above produces. simulator_bridge negates the four
        heave values again on their way from /controller/thruster_forces into
        Gazebo, so they are pre-negated here to cancel it.
        """
        limit = float(self.get_parameter('max_thrust').value)
        heave_sign = float(self.get_parameter('heave_sign').value)
        values = np.clip(np.asarray(forces, dtype=float), -limit, limit)

        msg = ThrusterForces()
        msg.surge_front_left = float(values[0])
        msg.surge_front_right = float(values[1])
        msg.surge_back_left = float(values[2])
        msg.surge_back_right = float(values[3])
        msg.heave_front_left = float(-values[4] * heave_sign)
        msg.heave_front_right = float(-values[5] * heave_sign)
        msg.heave_back_left = float(-values[6] * heave_sign)
        msg.heave_back_right = float(-values[7] * heave_sign)
        self.thrust_pub.publish(msg)

    def publish_zero(self):
        self.thrust_pub.publish(ThrusterForces())

    def publish_status(self):
        self.publish_speed_mode()
        speed = 0.0
        depth = 0.0
        heading = 0.0
        if self.state is not None:
            speed = float(np.linalg.norm(self.state['body_vel'][:2]))
            depth = -float(self.state['xyz'][2])        # back to depth-positive
            heading = math.degrees(self.state['yaw'])
        self.status_pub.publish(String(data=json.dumps({
            'pad': self.pad.alive,
            'device': self.pad.active_path,
            # What the sticks were found resting at, so `./demo.sh status` can
            # answer "is it the pad?" without anyone reading /dev/input.
            'stick_centres': {str(a): round(v / AXIS_MAX, 3)
                              for a, v in sorted(self.pad.centres.items())},
            'engaged': self.engaged,
            'run': self.run_state.state_label if self.run_state else 'unknown',
            'score': int(self.run_state.score) if self.run_state else 0,
            'gear': round(float(self.get_parameter('gear').value), 3),
            'speed_mode': self.mode_name,
            'scale': round(self.gear_scale(), 3),
            'surge_cmd': round(self.commanded['surge'], 3),
            'sway_cmd': round(self.commanded['sway'], 3),
            'yaw_cmd': round(self.commanded['yaw'], 3),
            'heave_cmd': round(self.commanded['heave'], 3),
            'speed': round(speed, 3),
            'depth': round(depth, 3),
            'heading': round(heading, 1),
            'holding': self.hold_xy is not None,
            'marker_aboard': self.marker_aboard,
            'torpedo_aboard': self.torpedo_aboard,
            'gripper': 'closed' if self.gripper_closed else 'open',
        })))


def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Gazebo holds the last thrust it was handed, so leaving on a non-zero
        # command means the vehicle carries on across the pool after Ctrl+C.
        try:
            node.publish_zero()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
