"""Turn task events into a running score, and publish it for the Gazebo panel.

Sits downstream of the task flaggers: they decide what happened, this decides
what it is worth. Keeping the two apart means a new task needs a flagger and a
`points:` block in the config, and nothing here has to change.

Point values come from tasks.<task>.scoring.points in competition_config.yaml,
never from code, so retuning the scoring does not mean a rebuild of anything but
the config.

It is also downstream of the run manager, which reports the boundaries of a run
on /simulator/run_state and the end of one as an event like any other. That is
what makes this the only node that has to know the difference between an event
worth points and one that happened outside a run:

  * a task event only counts while the run state is RUNNING;
  * a new run index clears the board;
  * the end of a run pays the time bonus and writes the result file.

The bonus is the unused half of the time limit at run.time_bonus.points_per_minute,
and it is only paid if the vehicle actually ran the tasks named in
run.time_bonus.requires_tasks - see `eligible_for_bonus` for what "ran" means.
Without that guard, starting a run and immediately ending it would be the
highest-scoring strategy in the competition.

With no run manager on the graph - the simulator on its own, or scoring:=true
with lifecycle:=false - no run state is ever received and everything is counted
as it happens, which is how this node behaved before the lifecycle existed.

Publishes on /scoring/display a JSON document the Gazebo Scoreboard panel
renders:

    {"total": 120, "entries": [{"label": ..., "points": ..., "count": ...}],
     "run": {"state": "RUNNING", "elapsed": 12.4, "remaining": 1187.6},
     "tasks": {"gate": true, "slalom": false},
     "speed_mode": "TURBO"}

`speed_mode` is the demo gamepad's gear, and is present ONLY when that node is
running - a competition entry has no gears, so the panel simply does not draw
the row. See the subscription below.
"""

from datetime import datetime, timezone
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from auv_msgs.msg import RunState
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
import yaml


class ScoreKeeper(Node):

    def __init__(self):
        super().__init__('score_keeper')

        share = get_package_share_directory('auv_worlds')
        self.declare_parameter('config_file', f'{share}/config/competition_config.yaml')
        self.declare_parameter('vehicle_model', 'auv')

        config_path = self.get_parameter('config_file').value
        self.vehicle = self.get_parameter('vehicle_model').value
        self.gate = self._load_gate_scoring(config_path)
        self.slalom = self._load_slalom_scoring(config_path)
        self.bins = self._load_bins_scoring(config_path)
        self.torpedo = self._load_torpedo_scoring(config_path)
        self.octagon = self._load_octagon_scoring(config_path)
        self.bonus = self._load_time_bonus(config_path)

        # Ordered so the panel lists events in the order they were earned.
        self.entries = []
        self.awarded = set()
        # What the vehicle has been THROUGH, whether or not it scored. The time
        # bonus is gated on this rather than on points, so taking the cheap half
        # of the gate or the wrong side of a pole still counts as having run the
        # task.
        self.attempted = {'gate': False, 'slalom': set()}
        # A collision is an episode, not an instant: penalise the rising edge,
        # then stay latched until contacts have been absent for the cooldown.
        # Otherwise a hull resting against the frame is billed over and over.
        self.in_contact = False
        self.last_contact_time = None

        # `state` stays None until a run manager is heard from, which is what
        # keeps this node working on its own.
        self.run = {'state': None, 'label': 'IDLE', 'index': None,
                    'elapsed': 0.0, 'remaining': 0.0}

        self.display_pub = self.create_publisher(String, '/scoring/display', 10)
        self.create_subscription(String, '/scoring/events', self.event_callback, 10)
        self.create_subscription(
            RunState, '/simulator/run_state', self.run_state_callback, 10)
        self.create_subscription(
            Contacts, '/model/task1_gate/contact', self.gate_contact_callback, 10)
        # NOT subscribed to /scoring/payload_reloaded, deliberately. That topic
        # says a marker or torpedo is back on the hull, and this node used to
        # clear the payload's key when it heard it, so the reloaded payload
        # could score again. Since a reload is one button press, that made the
        # bins and the board an infinite supply of points - park over a bin,
        # drop, reload, drop. Scoring is now keyed on the BIN and the HOLE, each
        # of which pays once per run, so there is nothing for a reload to
        # re-arm. The flaggers still listen to it: they need to re-arm their
        # DETECTION so a second drop is seen at all, which is a different
        # question from whether it pays.

        # Republish periodically: the GUI panel may connect after the first
        # events land, and Gazebo Transport has no latching.
        #
        # On the STEADY clock: this node runs with use_sim_time, so on the node
        # clock a "one second" redraw is one SIM second, and at an RTF of 0.6
        # the clock on the panel would advance in visible jumps - 00:01, 00:03 -
        # every 1.7 real seconds. The run is timed on the wall clock (see
        # run_manager.run_clock), so the panel showing it redraws on the wall
        # clock too, once a real second, like a stopwatch.
        self.create_timer(1.0, self.publish_display,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        # Stays on the node clock: this one is about how stale a contact SENSOR
        # reading is, which is physics and belongs in sim seconds.
        self.create_timer(0.25, self.release_contact_latch)

        # The demo gamepad's gear name, straight through to the panel. This is
        # the ONE thing on the scoreboard that scoring does not own, and it is
        # here because the panel has no other way in: it renders whatever JSON
        # this node publishes. A plain mirror - no validation beyond a length
        # cap, no scoring consequence, and the field is absent until something
        # publishes, which in a competition run is never.
        #
        # Not on the whitelist the domain bridge carries either, so a
        # competitor's container cannot reach it: in demo mode the gamepad node
        # sits on the simulator's own domain, which is what makes this work.
        self.speed_mode = None
        self.create_subscription(String, '/gamepad/speed_mode',
                                 self.speed_mode_callback, 10)
        self.publish_display()

        self.get_logger().info(
            f'Score keeper up. Gate points: {self.gate["points"]}, '
            f'collision cooldown {self.gate["collision_cooldown"]}s. '
            f'Slalom: {self.slalom["points"]:+.0f} per layer, '
            f'required side "{self.slalom["required_side"]}". '
            f'Bins: {self.bins["points"]:+.0f} per bin. '
            f'Torpedo: {self.torpedo["points"]:+.0f} per hole. '
            f'Octagon: {self.octagon["points"]:+.0f} per object delivered. '
            f'Everything pays once. '
            f'Time bonus: {self.bonus["points_per_minute"]:+.0f} per minute saved, '
            f'requires {self.bonus["requires"] or "nothing"}.')

    # -- config -------------------------------------------------------------

    def _load_gate_scoring(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get('gate') or {}).get('scoring') or {}
        points = scoring.get('points') or {}
        return {
            'points': {
                'left': float(points.get('left', 0.0)),
                'right': float(points.get('right', 0.0)),
                'collision': float(points.get('collision', 0.0)),
            },
            'collision_cooldown': float(scoring.get('collision_cooldown', 3.0)),
        }

    def _load_slalom_scoring(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get('slalom') or {}).get('scoring') or {}
        required = str(scoring.get('required_side', 'left')).strip().lower()
        # "any" scores a layer taken on either side of the red pole. It is a
        # scoring rule, not a detection one: the flagger still reports which
        # side, and the log still says so.
        if required not in ('left', 'right', 'any'):
            raise RuntimeError(
                f'{path}: tasks.slalom.scoring.required_side must be "left", '
                f'"right" or "any", got {required!r}')
        return {
            'required_side': required,
            'points': float((scoring.get('points') or {}).get('layer', 0.0)),
            # Every one of these has to have been crossed before the slalom
            # counts as run, on either side.
            'layers': list(scoring.get('layers') or []),
        }

    def _load_bins_scoring(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get('bins') or {}).get('scoring') or {}
        return {'points': float((scoring.get('points') or {}).get('marker_in_bin', 0.0))}

    def _load_torpedo_scoring(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get('torpedo') or {}).get('scoring') or {}
        return {'points': float(
            (scoring.get('points') or {}).get('torpedo_in_hole', 0.0))}

    def _load_octagon_scoring(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get('octagon') or {}).get('scoring') or {}
        return {'points': float(
            (scoring.get('points') or {}).get('object_in_box', 0.0))}

    def _load_time_bonus(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        bonus = ((config.get('run') or {}).get('time_bonus')) or {}

        requires = [str(task).strip().lower()
                    for task in (bonus.get('requires_tasks') or [])]
        unknown = sorted(set(requires) - {'gate', 'slalom'})
        if unknown:
            raise RuntimeError(
                f'{path}: run.time_bonus.requires_tasks can only name tasks this node '
                f'tracks completion for ("gate", "slalom"); got {unknown}')
        return {
            'points_per_minute': float(bonus.get('points_per_minute', 0.0)),
            'whole_minutes_only': bool(bonus.get('whole_minutes_only', False)),
            'requires': requires,
        }

    # -- run lifecycle ------------------------------------------------------

    def run_state_callback(self, msg):
        """Follow the run manager: a new index is a new run, so a clean board."""
        if self.run['index'] is not None and msg.run_index != self.run['index']:
            self.clear(f'run {msg.run_index}')
        self.run.update({
            'state': msg.state,
            'label': msg.state_label,
            'index': msg.run_index,
            'elapsed': msg.elapsed,
            'remaining': msg.remaining,
        })

    def scoring_open(self):
        """True when a task event counts.

        None means no run manager has ever been heard from, so there is no
        lifecycle to obey and events count as they arrive - the simulator run
        standalone, or launched with lifecycle:=false.
        """
        return self.run['state'] in (None, RunState.RUNNING)

    def clear(self, why):
        self.entries = []
        self.awarded = set()
        self.attempted = {'gate': False, 'slalom': set()}
        self.in_contact = False
        self.last_contact_time = None
        self.get_logger().info(f'Score cleared for {why}.')
        self.publish_display()

    def task_complete(self, task):
        """Has the vehicle been through this task at all, points or not?"""
        if task == 'gate':
            return self.attempted['gate']
        if task == 'slalom':
            layers = set(self.slalom['layers'])
            return bool(layers) and layers <= self.attempted['slalom']
        return False

    # -- scoring ------------------------------------------------------------

    def award(self, key, label, points):
        """Add points under `key`, collapsing repeats into a count."""
        for entry in self.entries:
            if entry['key'] == key:
                entry['count'] += 1
                entry['points'] += points
                break
        else:
            self.entries.append(
                {'key': key, 'label': label, 'points': points, 'count': 1})
        self.get_logger().info(
            f'{label}: {points:+.0f}  (total {self.total():+.0f})')
        self.publish_display()

    def total(self):
        return sum(e['points'] for e in self.entries)

    def event_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Unparseable scoring event: {msg.data!r}')
            return

        # The end of a run is not a scoring event that happens DURING one, so it
        # is handled before the gate below closes on it.
        if event.get('task') == 'run':
            self.run_event(event)
            return

        if not self.scoring_open():
            self.get_logger().info(
                f'Ignoring a {event.get("task")} event: the run is '
                f'{self.run["label"]}, not RUNNING.')
            return

        if event.get('task') == 'bins':
            self.bins_event(event)
            return
        if event.get('task') == 'torpedo':
            self.torpedo_event(event)
            return
        if event.get('task') == 'octagon':
            self.octagon_event(event)
            return
        if event.get('event') != 'passed':
            return
        if event.get('task') == 'slalom':
            self.slalom_event(event)
            return
        if event.get('task') != 'gate':
            return
        # Only the intended direction scores; swimming back through is not a
        # second completion.
        if event.get('direction') != 'forward':
            return

        # Through the gate at all, which is what the time bonus asks about. The
        # half that earns fewer points is still a gate that was run.
        self.attempted['gate'] = True

        side = event.get('side')
        if side not in self.gate['points']:
            return
        # The gate is completed once. Repeated forward passes are not repeat points.
        if 'gate_pass' in self.awarded:
            return
        self.awarded.add('gate_pass')
        self.award(f'gate_{side}', f'Gate — {side} half', self.gate['points'][side])

    def bins_event(self, event):
        """Award a marker landing in a bin. Each BIN pays once, not each drop.

        Keyed on the bin because the marker is reusable: one button press puts
        it back on the hull, so keying on the marker meant a player could hover
        over a single bin and drop into it all afternoon. Four bins, so this
        task is worth four awards to anyone who visits all four - which is the
        behaviour the task is actually asking for.
        """
        if event.get('event') != 'marker_in_bin':
            return
        bin_name = event.get('bin')
        if not bin_name:
            return
        key = f'bin:{bin_name}'
        if key in self.awarded:
            return
        self.awarded.add(key)
        self.award(f'bin_{bin_name}', f'Marker in {bin_name}', self.bins['points'])

    def torpedo_event(self, event):
        """Award a torpedo through one of the board's holes. Each HOLE once.

        Keyed on the hole for the same reason the bins are keyed on the bin: the
        torpedo reloads on a button press, so keying on the torpedo made one
        hole worth unlimited points. Putting a shot through each of the four
        holes is the skill; putting four shots through the same hole is one
        skill done four times.
        """
        if event.get('event') != 'torpedo_in_hole':
            return
        hole = event.get('hole')
        if not hole:
            return
        key = f'hole:{hole}'
        if key in self.awarded:
            return
        self.awarded.add(key)
        self.award(f'torpedo_{hole}', f'Torpedo through {hole}',
                   self.torpedo['points'])

    def octagon_event(self, event):
        """Award a pickup delivered onto one of the octagon's side pads.

        Keyed on the OBJECT, not the pad: there are four pickups and two pads,
        and the ask is that delivering an object is what pays, whichever pad it
        goes on. Putting the same object down twice is one delivery.
        """
        if event.get('event') != 'object_in_box':
            return
        obj = event.get('object')
        if not obj:
            return
        key = f'octagon:{obj}'
        if key in self.awarded:
            return
        self.awarded.add(key)
        self.award(f'octagon_{obj}', f'{obj} delivered', self.octagon['points'])

    def slalom_event(self, event):
        """Award a layer, but only on the side the config asks for."""
        if event.get('direction') != 'forward':
            return

        layer = event.get('layer')
        side = event.get('side')
        if not layer:
            return

        # Cleared the layer, whichever side of the pole it was taken on.
        self.attempted['slalom'].add(layer)

        required = self.slalom['required_side']
        if required != 'any' and side != required:
            self.get_logger().info(
                f'Slalom {layer}: passed on the {side} side, but '
                f'"{required}" is required - no points.')
            return
        # Each layer counts once, so weaving back and forth earns nothing extra.
        if layer in self.awarded:
            return
        self.awarded.add(layer)
        self.award(f'slalom_{layer}', f'Slalom {layer}', self.slalom['points'])

    # -- end of run ---------------------------------------------------------

    def run_event(self, event):
        """The run manager reporting the end of a run: pay the time, write the card."""
        if event.get('event') != 'ended':
            return

        # Freeze here too, not only on /simulator/run_state. The two arrive on
        # different topics with nothing ordering them against each other, and
        # this is the event the result file is written at.
        self.run['state'] = RunState.FINISHED
        self.run['label'] = 'FINISHED'

        bonus = self.time_bonus(float(event.get('remaining', 0.0)))
        self.write_result(event, bonus)

    def time_bonus(self, remaining):
        """Award the unused time, if the course was actually run. Returns the points."""
        missing = [task for task in self.bonus['requires'] if not self.task_complete(task)]
        if missing:
            self.get_logger().info(
                f'No time bonus: {remaining:.0f}s unused, but the '
                f'{" and ".join(missing)} '
                f'{"task was" if len(missing) == 1 else "tasks were"} never run.')
            return 0.0

        minutes = remaining / 60.0
        if self.bonus['whole_minutes_only']:
            minutes = math.floor(minutes)

        points = minutes * self.bonus['points_per_minute']
        if points <= 0.0:
            self.get_logger().info(
                f'No time bonus: {remaining:.0f}s unused is worth nothing at '
                f'{self.bonus["points_per_minute"]:.0f} points per minute.')
            return 0.0

        shown = f'{minutes:.0f}' if self.bonus['whole_minutes_only'] else f'{minutes:.1f}'
        self.award('time_bonus', f'Time bonus — {shown} min saved', points)
        return points

    def write_result(self, event, bonus):
        """Write the run's scorecard beside its world and ground truth."""
        directory = event.get('results_dir') or '.'
        run_id = event.get('run_id', 'unknown')
        path = os.path.join(directory, f'result_{run_id}_run{event.get("run_index", 0)}.json')

        payload = {
            'run_id': run_id,
            'run_index': event.get('run_index'),
            'seed': event.get('seed'),
            'ended': event.get('reason'),
            'elapsed_s': event.get('elapsed'),
            'time_limit_s': event.get('time_limit'),
            'unused_s': event.get('remaining'),
            'total': round(self.total()),
            'time_bonus': round(bonus),
            'tasks_run': {
                'gate': self.task_complete('gate'),
                'slalom': self.task_complete('slalom'),
                'slalom_layers': sorted(self.attempted['slalom']),
            },
            'entries': [
                {'key': e['key'], 'label': e['label'],
                 'points': round(e['points']), 'count': e['count']}
                for e in self.entries
            ],
            'written_utc': datetime.now(timezone.utc).isoformat(),
        }

        try:
            os.makedirs(directory, exist_ok=True)
            with open(path, 'w') as handle:
                json.dump(payload, handle, indent=2)
        except OSError as exc:
            # The score is still on the panel and in the log, so this is not
            # fatal to the run - but it is the only durable copy, so say so.
            self.get_logger().error(f'Could not write the result to {path}: {exc}')
            return

        self.get_logger().info(
            f'Run {event.get("run_index")} scored {round(self.total()):+d} '
            f'(time bonus {round(bonus):+d}) -> {path}')

    # -- collisions ---------------------------------------------------------

    def gate_contact_callback(self, msg):
        if not msg.contacts:
            return
        if not self.scoring_open():
            return
        if not any(self.vehicle in (c.collision1.name, c.collision2.name) or
                   c.collision1.name.startswith(f'{self.vehicle}::') or
                   c.collision2.name.startswith(f'{self.vehicle}::')
                   for c in msg.contacts):
            return

        self.last_contact_time = self.get_clock().now().nanoseconds * 1e-9
        if self.in_contact:
            return
        self.in_contact = True
        self.award('gate_collision', 'Gate collision', self.gate['points']['collision'])

    def release_contact_latch(self):
        """Re-arm the collision penalty once the hull has been clear a while."""
        if not self.in_contact or self.last_contact_time is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_contact_time >= self.gate['collision_cooldown']:
            self.in_contact = False

    def speed_mode_callback(self, msg):
        label = msg.data.strip()[:16]
        self.speed_mode = label or None

    # -- output -------------------------------------------------------------

    def publish_display(self):
        board = {
            'total': round(self.total()),
            'entries': [
                {'label': e['label'], 'points': round(e['points']), 'count': e['count']}
                for e in self.entries
            ],
            # Read back by the run manager, which reports them to the competitor
            # in RunState: this node is the one that knows what has been run.
            'tasks': {
                'gate': self.task_complete('gate'),
                'slalom': self.task_complete('slalom'),
            },
        }
        if self.run['state'] is not None:
            board['run'] = {
                'state': self.run['label'],
                'index': self.run['index'],
                'elapsed': round(self.run['elapsed'], 1),
                'remaining': round(self.run['remaining'], 1),
            }
        if self.speed_mode is not None:
            board['speed_mode'] = self.speed_mode
        self.display_pub.publish(String(data=json.dumps(board)))


def main(args=None):
    rclpy.init(args=args)
    node = ScoreKeeper()
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
