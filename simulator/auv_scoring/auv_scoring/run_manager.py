"""Own the run: arm it, time it, stop it, and put the course back.

Nothing else in the stack knows when a run is. The flaggers report crossings
whenever they see them and the score keeper adds up whatever it is sent; this
node draws the boundaries around that, which is what makes an elapsed time mean
anything and a score comparable between teams.

Three services, all std_srvs/Trigger, sitting alongside the other /simulator/*
endpoints:

    /simulator/start_run    arm: zero the clock, unkill the thrusters
    /simulator/end_run      freeze scoring, kill the thrusters, write the result
    /simulator/reset_run    re-pose the vehicle and course, clear the score, re-seed

and one topic out, /simulator/run_state (auv_msgs/RunState), at 5 Hz.

The thrusters are dead until start_run - see `start_disarmed` in
simulator_bridge. Without that the clock effectively starts when Gazebo does,
and a controller that takes fifteen seconds to boot is penalised for its own
startup.

The run is timed on the WALL CLOCK, not on sim time - `elapsed`, `remaining`
and the time limit are all real seconds, so the panel agrees with a stopwatch
even when Gazebo is running at an RTF of 0.6. See `run_clock`; sim time is
still what `wait_sim` settles physics against.

Crossing the domain bridge
--------------------------
domain_bridge forwards topics, not services, so the competitor's domain gets
/simulator/run_state one way and /simulator/run_control (std_msgs/String,
"start" | "end" | "reset") the other. This node is the relay: a control string
runs exactly the code the matching service handler does.

Who computes what
-----------------
This node emits the end of a run as a scoring event like any other, rather than
totalling anything itself:

    {"task": "run", "event": "ended", "remaining": 687.7, ...}

The score keeper turns that into the time bonus and writes the result file,
because it is the one thing holding the score. Splitting it the other way would
mean two nodes racing to write the same number.

Resetting without restarting Gazebo
-----------------------------------
A Gazebo boot is 15-20 s, which is the difference between forty practice runs in
an evening and ten. reset_run re-samples the course from the layout instead,
teleports every prop and the vehicle to the new poses with one Gazebo request,
rewrites the ground truth and tells the flaggers to reload it.

Two things it cannot do, documented rather than hidden. Gazebo's set_pose moves
a body without stopping it, so the thrusters are cut and `run.reset.settle_time`
seconds of sim time are given to drag before the teleport. And a marker that has
already been dropped, or a pickup that has been grasped, stays where it is: both
are DetachableJoints that only a relaunch re-welds, so a reset run cannot score
the bins again.
"""

import json
import os
import random
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
from auv_msgs.msg import RunState
from auv_worlds.world_generator import (LayoutError, make_run_id, place_props,
                                        write_ground_truth)
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
import yaml


class RunManager(Node):

    LABELS = {
        RunState.IDLE: 'IDLE',
        RunState.RUNNING: 'RUNNING',
        RunState.FINISHED: 'FINISHED',
    }

    def __init__(self):
        super().__init__('run_manager')

        share = get_package_share_directory('auv_worlds')
        self.declare_parameter('config_file', f'{share}/config/competition_config.yaml')
        self.declare_parameter('ground_truth_file', '')
        self.declare_parameter('world', 'underwater_pool')
        self.declare_parameter('gz_timeout_ms', 3000)

        self.config_path = self.get_parameter('config_file').value
        self.world = self.get_parameter('world').value
        self.gz_timeout_ms = int(self.get_parameter('gz_timeout_ms').value)
        self.settings = self._load_settings(self.config_path)
        self.course = self._load_course(self.get_parameter('ground_truth_file').value)

        # Held across a whole reset, which teleports models and sleeps on sim
        # time. Everything that reads or writes the state below takes it.
        self.lock = threading.Lock()
        self.state = RunState.IDLE
        self.run_index = 0
        self.started_at = None
        self.elapsed = 0.0
        self.message = ''
        self.score = 0
        self.tasks = {'gate': False, 'slalom': False}

        # Reentrant, and not optional: a reset waits on sim time from inside its
        # own callback, and sim time only advances if /clock can still be
        # serviced while it does.
        group = ReentrantCallbackGroup()

        self.state_pub = self.create_publisher(RunState, '/simulator/run_state', 10)
        self.event_pub = self.create_publisher(String, '/scoring/events', 10)
        self.course_pub = self.create_publisher(String, '/scoring/course', 10)

        self.create_service(Trigger, '/simulator/start_run', self.handle_start_run,
                            callback_group=group)
        self.create_service(Trigger, '/simulator/end_run', self.handle_end_run,
                            callback_group=group)
        self.create_service(Trigger, '/simulator/reset_run', self.handle_reset_run,
                            callback_group=group)

        self.create_subscription(String, '/simulator/run_control', self.control_callback,
                                 10, callback_group=group)
        self.create_subscription(String, '/scoring/display', self.display_callback, 10)

        self.kill_client = self.create_client(
            Empty, '/simulator/kill_thrusters', callback_group=group)
        self.unkill_client = self.create_client(
            Empty, '/simulator/unkill_thrusters', callback_group=group)
        self.localisation_client = self.create_client(
            Empty, '/localization/reset_service', callback_group=group)
        self.service_names = {
            self.kill_client: '/simulator/kill_thrusters',
            self.unkill_client: '/simulator/unkill_thrusters',
            self.localisation_client: '/localization/reset_service',
        }

        # On the STEADY clock, not the node's: with use_sim_time the node clock
        # is Gazebo's, so this timer would fire at the RTF - a fifth of a second
        # of sim time, not of the driver's - and stop dead the moment the world
        # is paused, freezing the panel while real seconds carried on. The run
        # is timed on the wall clock now (see `run_clock`), so the timer that
        # displays it and enforces the limit runs on the wall clock too.
        #
        # The consequence is deliberate: pausing Gazebo does not pause the run.
        self.create_timer(0.2, self.tick, callback_group=group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            f'Run manager up on course {self.course["run_id"]} '
            f'(seed {self.course["seed"]}). Limit {self.settings["time_limit"]:.0f}s, '
            f'reset {"re-seeds" if self.settings["reseed"] else "keeps the layout"}. '
            'Thrusters stay dead until /simulator/start_run.')

    # -- config -------------------------------------------------------------

    def _load_settings(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        run = config.get('run') or {}
        reset = run.get('reset') or {}

        limit = float(run.get('time_limit', 1200.0))
        if limit <= 0.0:
            raise RuntimeError(
                f'{path}: run.time_limit must be a positive number of seconds, got {limit}')
        return {
            'time_limit': limit,
            'reseed': bool(reset.get('reseed', True)),
            'settle_time': float(reset.get('settle_time', 1.0)),
            'results_dir': run.get('results_dir') or None,
        }

    def _load_course(self, truth_path):
        """Learn which course is loaded, and where this run's artefacts live.

        The ground truth file already records the seed, the layout it was drawn
        from and where it was written, so a reset needs no parameters of its own
        beyond the file the flaggers were given.
        """
        if not truth_path:
            raise RuntimeError(
                'ground_truth_file parameter is empty; the run manager needs it to know '
                'which course is loaded and where to re-sample it from.')
        with open(truth_path) as handle:
            truth = json.load(handle)

        truth_path = os.path.abspath(truth_path)
        return {
            'run_id': truth['run_id'],
            'seed': int(truth['seed']),
            'layout': truth.get('layout') or os.path.abspath(self.config_path),
            'base_world': truth.get('base_world', ''),
            'world_file': truth.get('world', ''),
            # <output_dir>/ground_truth/competition_<run_id>.json
            'output_dir': os.path.dirname(os.path.dirname(truth_path)),
            'truth': truth_path,
        }

    def results_dir(self):
        return self.settings['results_dir'] or self.course['output_dir']

    # -- lifecycle ----------------------------------------------------------

    def handle_start_run(self, request, response):
        with self.lock:
            if self.state == RunState.RUNNING:
                return self._reply(
                    response, False, f'run {self.run_index} is already in progress')
            if self.state == RunState.FINISHED:
                return self._reply(
                    response, False,
                    'the last run is finished; call /simulator/reset_run first')

            # The sim clock is still the liveness check even though it no
            # longer times the run: no /clock means Gazebo is down or paused,
            # and starting a wall-clock run against a frozen world would burn
            # the limit on a vehicle that cannot move.
            if self.sim_time() <= 0.0:
                return self._reply(
                    response, False,
                    'no sim clock yet - is Gazebo up and running (not paused)?')

            self.run_index += 1
            self.started_at = self.run_clock()
            self.elapsed = 0.0
            self.message = ''
            self.state = RunState.RUNNING
            # State before thrusters: the score keeper clears its board on the
            # new run index, and nothing should be able to score into the old
            # one in between.
            self.publish_state()
            self._call(self.unkill_client, 'arming the thrusters')

            self.get_logger().info(
                f'Run {self.run_index} started on course {self.course["run_id"]}; '
                f'{self.settings["time_limit"]:.0f}s limit, thrusters live.')
            return self._reply(
                response, True,
                f'run {self.run_index} started; {self.settings["time_limit"]:.0f}s limit')

    def handle_end_run(self, request, response):
        with self.lock:
            if self.state != RunState.RUNNING:
                return self._reply(
                    response, False, f'no run in progress (state {self.LABELS[self.state]})')
            self._finish('ended by the competitor')
            return self._reply(
                response, True,
                f'run {self.run_index} ended after {self.elapsed:.1f}s, '
                f'{max(0.0, self.settings["time_limit"] - self.elapsed):.1f}s unused')

    def _finish(self, reason):
        """Freeze the run and hand the ending to the score keeper. Caller holds the lock."""
        self.elapsed = max(0.0, self.run_clock() - self.started_at)
        self.state = RunState.FINISHED
        self.message = reason
        remaining = max(0.0, self.settings['time_limit'] - self.elapsed)

        # Thrusters first - the vehicle should stop the instant the run does.
        self._call(self.kill_client, 'killing the thrusters')
        self.publish_state()

        # The score keeper freezes on this event, pays the time bonus from
        # `remaining` and writes the result file. It carries everything that
        # identifies the run so nothing has to be correlated after the fact.
        self.event_pub.publish(String(data=json.dumps({
            'task': 'run',
            'event': 'ended',
            'reason': reason,
            'run_index': self.run_index,
            'run_id': self.course['run_id'],
            'seed': self.course['seed'],
            'elapsed': round(self.elapsed, 3),
            'time_limit': self.settings['time_limit'],
            'remaining': round(remaining, 3),
            'results_dir': self.results_dir(),
            'sim_time': self.sim_time(),
        })))
        self.get_logger().info(
            f'Run {self.run_index} finished ({reason}) after {self.elapsed:.1f}s; '
            f'{remaining:.1f}s unused.')

    def handle_reset_run(self, request, response):
        # Non-blocking: a second reset arriving while the first is teleporting
        # models should be told so, not queued behind it.
        if not self.lock.acquire(blocking=False):
            return self._reply(response, False, 'busy - a reset is already running')
        try:
            aborted = self.state == RunState.RUNNING
            self._call(self.kill_client, 'killing the thrusters')
            self.state = RunState.IDLE
            self.started_at = None
            self.elapsed = 0.0
            self.message = 'reset'
            # New index: this is what clears the score keeper's board. An
            # aborted run writes no result - it did not finish.
            self.run_index += 1
            self.publish_state()

            # Let drag take the speed off before teleporting. set_pose moves a
            # body, it does not stop one, so without this the vehicle arrives at
            # the spawn carrying whatever velocity it had at the finish.
            self.wait_sim(self.settings['settle_time'])

            try:
                placed, seed = self.sample_course()
            except (LayoutError, OSError, ValueError) as exc:
                return self._reply(response, False, f'could not re-sample the course: {exc}')

            if not self.set_poses(placed):
                return self._reply(
                    response, False,
                    'Gazebo refused the pose request, so the course has NOT moved; '
                    'see the log for what gz reported. Relaunch to get a clean course.')

            run_id = make_run_id(seed)
            truth = write_ground_truth(
                self.course['output_dir'], run_id, seed, self.course['layout'],
                self.course['base_world'], self.course['world_file'], placed)
            self.course.update({'run_id': run_id, 'seed': seed, 'truth': truth})
            # The flaggers were built around the old poses; this is what makes
            # them rebuild, and it also re-arms their crossing detectors.
            self.course_pub.publish(String(data=truth))

            # The vehicle moved, so the origin simulator_bridge subtracts from
            # every pose it publishes is stale. Let the odometry catch up with
            # the teleport first, or the new origin captures the pose the
            # vehicle had at the finish line rather than the one it has now.
            self.wait_sim(0.5)
            self._call(self.localisation_client, 're-zeroing the localisation origin')

            self.publish_state()
            self.get_logger().info(
                f'Reset to course {run_id} (seed {seed}); {len(placed)} models re-posed, '
                f'score cleared, thrusters dead until /simulator/start_run.')
            return self._reply(
                response, True,
                f'{"aborted the run and reset" if aborted else "reset"} to seed {seed}; '
                f'{len(placed)} models re-posed. Call /simulator/start_run when ready.')
        finally:
            self.lock.release()

    def tick(self):
        """Publish the state, and end the run when the limit is reached."""
        if not self.lock.acquire(blocking=False):
            return          # mid-reset; that path publishes its own state
        try:
            if self.state == RunState.RUNNING:
                self.elapsed = max(0.0, self.run_clock() - self.started_at)
                if self.elapsed >= self.settings['time_limit']:
                    self._finish('time limit reached')
            self.publish_state()
        finally:
            self.lock.release()

    # -- the course ---------------------------------------------------------

    def sample_course(self):
        """Draw fresh poses for every prop and the vehicle. Returns (placed, seed)."""
        with open(self.course['layout']) as handle:
            layout = yaml.safe_load(handle)
        if not isinstance(layout, dict):
            raise LayoutError(f'{self.course["layout"]}: expected a YAML mapping')

        seed = (random.SystemRandom().randrange(2 ** 31)
                if self.settings['reseed'] else self.course['seed'])
        return place_props(layout, seed), seed

    def set_poses(self, placed):
        """Teleport every model to its new pose."""
        request = ' '.join(f'pose {{{self.pose_text(prop)}}}' for prop in placed)
        ok, detail = self.gz_request('set_pose_vector', 'gz.msgs.Pose_V', request)
        if ok:
            return True

        if 'timed out' in detail.lower():
            # Nothing answered, so the world's services are not there at all.
            # Trying again once per prop would be twenty more timeouts and a
            # minute of waiting for the same answer.
            self.get_logger().error(
                f'/world/{self.world}/set_pose_vector did not answer. Is Gazebo running, '
                f'and is its world really called "{self.world}"?')
            return False

        # Some gz-sim builds advertise only the single-entity form. One process
        # per prop is slower, but this is a reset, not a control loop. The list
        # comprehension is deliberate: attempt every prop rather than stopping at
        # the first refusal and leaving the course half-moved.
        self.get_logger().warn(
            'set_pose_vector was refused; falling back to one set_pose per model.')
        return all([self.gz_request('set_pose', 'gz.msgs.Pose', self.pose_text(prop))[0]
                    for prop in placed])

    @staticmethod
    def pose_text(prop):
        """One prop's sampled pose as gz.msgs.Pose text format.

        Ground truth is [x y z roll pitch yaw] in metres and degrees, the
        layout's own units; Gazebo wants metres and a quaternion.
        """
        x, y, z = prop['actual'][:3]
        qx, qy, qz, qw = Rot.from_euler('xyz', prop['actual'][3:], degrees=True).as_quat()
        return (f'name: "{prop["name"]}" '
                f'position {{x: {x:.6f} y: {y:.6f} z: {z:.6f}}} '
                f'orientation {{x: {qx:.9f} y: {qy:.9f} z: {qz:.9f} w: {qw:.9f}}}')

    def gz_request(self, service, request_type, request):
        """Call one of the world's Gazebo services. Returns (succeeded, detail).

        Gazebo Transport is not on the ROS graph, so this goes out through the
        `gz` CLI rather than a client - the same way simulator_bridge drives
        Gazebo's own log recorder. `detail` is whatever gz said, which is the
        only thing that distinguishes "no such service" from "I will not do that".
        """
        command = [
            'gz', 'service', '-s', f'/world/{self.world}/{service}',
            '--reqtype', request_type, '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self.gz_timeout_ms), '--req', request,
        ]
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=self.gz_timeout_ms / 1000.0 + 5.0)
        except (OSError, subprocess.TimeoutExpired) as exc:
            detail = f'{exc}'
            self.get_logger().error(f'gz {service} could not be called: {detail}')
            return False, detail

        detail = (result.stdout + result.stderr).strip() or 'no output'
        if result.returncode != 0 or 'true' not in result.stdout:
            self.get_logger().error(f'gz {service} refused: {detail}')
            return False, detail
        return True, detail

    # -- plumbing -----------------------------------------------------------

    def control_callback(self, msg):
        """Run a command that arrived as a topic, for the competitor's domain."""
        command = msg.data.strip().lower()
        handler = {
            'start': self.handle_start_run,
            'end': self.handle_end_run,
            'stop': self.handle_end_run,
            'reset': self.handle_reset_run,
        }.get(command)
        if handler is None:
            self.get_logger().warn(
                f'/simulator/run_control: ignoring {msg.data!r}; '
                'expected "start", "end" or "reset".')
            return

        response = handler(Trigger.Request(), Trigger.Response())
        # Two call sites rather than one logger chosen by a conditional. rclpy
        # caches severity per call LOCATION, so a single line that logs info on
        # one call and warn on the next raises "Logger severity cannot be
        # changed between calls" - which killed run_manager outright the first
        # time a command was rejected, taking /simulator/run_state with it and
        # leaving the whole run lifecycle dead with no obvious cause.
        detail = f'run_control "{command}": {response.message}'
        if response.success:
            self.get_logger().info(detail)
        else:
            self.get_logger().warn(detail)

    def display_callback(self, msg):
        """Mirror the score keeper's running total into the run state.

        The score keeper owns that number; this only reports it, which is how a
        competitor sees its own score without any part of the scoring domain
        being exposed to it.
        """
        try:
            board = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.score = int(board.get('total', 0))
        tasks = board.get('tasks') or {}
        self.tasks = {'gate': bool(tasks.get('gate')), 'slalom': bool(tasks.get('slalom'))}

    def publish_state(self):
        limit = self.settings['time_limit']
        elapsed = self.elapsed if self.state != RunState.IDLE else 0.0

        msg = RunState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.state_label = self.LABELS[self.state]
        msg.elapsed = elapsed
        msg.time_limit = limit
        msg.remaining = max(0.0, limit - elapsed)
        msg.score = int(self.score)
        msg.gate_complete = self.tasks['gate']
        msg.slalom_complete = self.tasks['slalom']
        msg.run_index = self.run_index
        msg.run_id = self.course['run_id']
        msg.message = self.message
        self.state_pub.publish(msg)

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def run_clock(self):
        """Wall-clock seconds, and the only clock the RUN is timed against.

        Deliberately not sim time. Gazebo rarely holds an RTF of 1.0 on a demo
        laptop, and at 0.6 a "twenty minute" run takes thirty-three real
        minutes while the panel still reads 12:00 - the clock on screen and the
        stopwatch in someone's hand disagree, which at a stand reads as a
        broken timer and in a heat is an argument about whose run was longer.
        Timing on the wall clock makes the displayed time the elapsed time, and
        the limit a real limit, whatever the physics is managing.

        monotonic, not wall time, so an NTP step mid-run cannot run the clock
        backwards or hand somebody a free minute.

        `wait_sim` stays on SIM time and must: it is waiting for drag to take
        the speed off a body, which is physics and happens in sim seconds.
        """
        return time.monotonic()

    def wait_sim(self, seconds):
        """Block until `seconds` of SIM time have passed.

        Sim time is what matters - at an RTF of 0.5 a wall-clock second is half
        a second of settling - but it can also stop dead if Gazebo is paused, so
        the wall clock caps the wait rather than timing it.
        """
        if seconds <= 0.0:
            return
        deadline = self.sim_time() + seconds
        wall_limit = time.monotonic() + max(5.0, seconds * 10.0)
        while self.sim_time() < deadline:
            if time.monotonic() >= wall_limit:
                self.get_logger().warn(
                    f'sim clock advanced less than {seconds:.1f}s in {seconds * 10.0:.0f}s '
                    'of wall time; carrying on. Is Gazebo paused?')
                return
            time.sleep(0.02)

    def _call(self, client, name):
        """Fire an Empty service on simulator_bridge and do not wait for it.

        Waiting would deadlock as often as not: these calls are made from inside
        service and timer callbacks, and nothing in the lifecycle depends on the
        reply. A missing service is loud, though, because a start_run that fails
        to unkill the thrusters looks exactly like a dead controller.
        """
        if not client.service_is_ready():
            self.get_logger().error(
                f'{self.service_names[client]} is not available, so {name} did NOT '
                'happen - is simulator_bridge running?')
            return
        client.call_async(Empty.Request())

    @staticmethod
    def _reply(response, success, message):
        response.success = success
        response.message = message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RunManager()
    # Threaded so a reset can wait on sim time inside its own callback while
    # /clock, the state timer and the other services keep being served.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
