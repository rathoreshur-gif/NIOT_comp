"""Turn task events into a running score, and publish it for the Gazebo panel.

Sits downstream of the task flaggers: they decide what happened, this decides
what it is worth. Keeping the two apart means a new task needs a flagger and a
`points:` block in the config, and nothing here has to change.

Point values come from tasks.<task>.scoring.points in competition_config.yaml,
never from code, so retuning the scoring does not mean a rebuild of anything but
the config.

Publishes on /scoring/display a JSON document the Gazebo Scoreboard panel
renders:

    {"total": 120, "entries": [{"label": ..., "points": ..., "count": ...}]}
"""

import json

from ament_index_python.packages import get_package_share_directory
import rclpy
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

        # Ordered so the panel lists events in the order they were earned.
        self.entries = []
        self.awarded = set()
        # A collision is an episode, not an instant: penalise the rising edge,
        # then stay latched until contacts have been absent for the cooldown.
        # Otherwise a hull resting against the frame is billed over and over.
        self.in_contact = False
        self.last_contact_time = None

        self.display_pub = self.create_publisher(String, '/scoring/display', 10)
        self.create_subscription(String, '/scoring/events', self.event_callback, 10)
        self.create_subscription(
            Contacts, '/model/task1_gate/contact', self.gate_contact_callback, 10)

        # Republish periodically: the GUI panel may connect after the first
        # events land, and Gazebo Transport has no latching.
        self.create_timer(1.0, self.publish_display)
        self.create_timer(0.25, self.release_contact_latch)
        self.publish_display()

        self.get_logger().info(
            f'Score keeper up. Gate points: {self.gate["points"]}, '
            f'collision cooldown {self.gate["collision_cooldown"]}s. '
            f'Slalom: {self.slalom["points"]:+.0f} per layer, '
            f'required side "{self.slalom["required_side"]}". '
            f'Bins: {self.bins["points"]:+.0f} per marker landed.')

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
        if required not in ('left', 'right'):
            raise RuntimeError(
                f'{path}: tasks.slalom.scoring.required_side must be "left" or '
                f'"right", got {required!r}')
        return {
            'required_side': required,
            'points': float((scoring.get('points') or {}).get('layer', 0.0)),
        }

    def _load_bins_scoring(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get('bins') or {}).get('scoring') or {}
        return {'points': float((scoring.get('points') or {}).get('marker_in_bin', 0.0))}

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

        if event.get('task') == 'bins':
            self.bins_event(event)
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

        side = event.get('side')
        if side not in self.gate['points']:
            return
        # The gate is completed once. Repeated forward passes are not repeat points.
        if 'gate_pass' in self.awarded:
            return
        self.awarded.add('gate_pass')
        self.award(f'gate_{side}', f'Gate — {side} half', self.gate['points'][side])

    def bins_event(self, event):
        """Award a marker landing in a bin. Each marker scores once."""
        if event.get('event') != 'marker_in_bin':
            return
        marker = event.get('marker')
        if not marker or marker in self.awarded:
            return
        self.awarded.add(marker)
        self.award('bin_marker', 'Marker in bin', self.bins['points'])

    def slalom_event(self, event):
        """Award a layer, but only on the side the config asks for."""
        if event.get('direction') != 'forward':
            return

        layer = event.get('layer')
        side = event.get('side')
        if not layer:
            return
        if side != self.slalom['required_side']:
            self.get_logger().info(
                f'Slalom {layer}: passed on the {side} side, but '
                f'"{self.slalom["required_side"]}" is required - no points.')
            return
        # Each layer counts once, so weaving back and forth earns nothing extra.
        if layer in self.awarded:
            return
        self.awarded.add(layer)
        self.award('slalom_layer', 'Slalom layer', self.slalom['points'])

    def gate_contact_callback(self, msg):
        if not msg.contacts:
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

    # -- output -------------------------------------------------------------

    def publish_display(self):
        self.display_pub.publish(String(data=json.dumps({
            'total': round(self.total()),
            'entries': [
                {'label': e['label'], 'points': round(e['points']), 'count': e['count']}
                for e in self.entries
            ],
        })))


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
