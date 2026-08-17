"""Flag the vehicle passing through each slalom layer, and on which side.

The slalom is three rows of three poles. Each row is one "layer", and the red
centre pole of that row is both the layer's plane and its divider: pass to one
side of it or the other. Taking the red pole's own randomised pose as the layer
frame means the detection follows the per-pole jitter exactly, with no need to
reconstruct where the row nominally was.

Published per valid crossing:

    /scoring/slalom  std_msgs/String   "<layer>:<side>"
    /scoring/events  std_msgs/String   JSON, consumed by the score keeper
"""

import json

from ament_index_python.packages import get_package_share_directory
from auv_scoring.crossing import PlaneCrossing, pose_from_ground_truth
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class SlalomFlagger(Node):

    def __init__(self):
        super().__init__('slalom_flagger')

        share = get_package_share_directory('auv_worlds')
        self.declare_parameter('ground_truth_file', '')
        self.declare_parameter('config_file', f'{share}/config/competition_config.yaml')
        self.declare_parameter('task', 'slalom')
        self.declare_parameter('odometry_topic', '/model/auv/odometry')

        self.task = self.get_parameter('task').value
        geometry = self._load_geometry(self.get_parameter('config_file').value)
        self.layers = self._build_layers(
            self.get_parameter('ground_truth_file').value, geometry)

        self.slalom_pub = self.create_publisher(String, '/scoring/slalom', 10)
        self.event_pub = self.create_publisher(String, '/scoring/events', 10)
        self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value,
            self.odometry_callback, 10)

        self.get_logger().info(
            f'Slalom flagger watching {len(self.layers)} layers '
            f'({", ".join(layer.name for layer in self.layers)})')

    def _load_geometry(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        task = (config.get('tasks') or {}).get(self.task) or {}
        scoring = task.get('scoring')
        if not scoring:
            raise RuntimeError(f'{path}: tasks.{self.task} has no `scoring:` block')
        for key in ('layers', 'opening_y', 'opening_z'):
            if key not in scoring:
                raise RuntimeError(f'{path}: tasks.{self.task}.scoring is missing "{key}"')
        return scoring

    def _build_layers(self, truth_path, geometry):
        if not truth_path:
            raise RuntimeError('ground_truth_file parameter is empty')
        with open(truth_path) as handle:
            truth = json.load(handle)

        props = {p['name']: p for p in truth['props']}
        vehicle = next((p for p in truth['props'] if p['task'] == 'vehicle'), None)
        if vehicle is None:
            raise RuntimeError(f'{truth_path}: no vehicle entry')
        start = vehicle['actual'][:3]

        layers = []
        for name in geometry['layers']:
            entry = props.get(name)
            if entry is None:
                raise RuntimeError(
                    f'{truth_path}: no prop named "{name}"; have {sorted(props)}')
            xyz, rot = pose_from_ground_truth(entry)
            layers.append(PlaneCrossing(
                name, xyz, rot, geometry['opening_y'], geometry['opening_z'],
                geometry.get('divider_y', 0.0), start))
        return layers

    def odometry_callback(self, msg):
        p = msg.pose.pose.position
        position = [p.x, p.y, p.z]

        for layer in self.layers:
            event = layer.update(position)
            if event is None:
                continue
            if event['outside']:
                self.get_logger().info(
                    f'{event["name"]}: crossed outside the gap at local '
                    f'y={event["local"][1]:+.2f} z={event["local"][2]:+.2f} - not counted.')
                continue

            self.slalom_pub.publish(String(data=f'{event["name"]}:{event["side"]}'))
            self.event_pub.publish(String(data=json.dumps({
                'task': self.task,
                'event': 'passed',
                'layer': event['name'],
                'side': event['side'],
                'direction': 'forward' if event['forward'] else 'reverse',
                'sim_time': self.get_clock().now().nanoseconds * 1e-9,
                'crossing_world': event['crossing_world'],
            })))
            self.get_logger().info(
                f'Slalom {event["name"]}: {event["side"]} of the centre pole, '
                f'{"forward" if event["forward"] else "reverse"}.')


def main(args=None):
    rclpy.init(args=args)
    node = SlalomFlagger()
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
