"""Flag the vehicle passing through the gate, and through which half.

This is scoring-side code, so it is allowed to use ground truth: it reads the
gate's actual sampled pose out of the run's ground truth file and the vehicle's
world pose off the simulator, neither of which a competitor is meant to see.

Detection works in the gate's own frame. The gate mesh is a thin plane, so its
local X is the swim-through axis: the signed local X of the vehicle flips as it
crosses. On a flip the crossing point is interpolated onto the plane and tested
against the opening rectangle, which rejects a vehicle that goes around the
outside, ducks under the posts, or hops over the top bar.

Which half is reported as "left" is derived, not hardcoded. The gate's own Y
axis has no inherent handedness - it depends on the yaw in the config, which is
currently 180 degrees - so the label comes from the INTENDED forward pass,
spawn side toward the far side:

    left_direction = world_up  x  forward_direction

The two halves are physically different (they carry different task images), so
a half keeps its name whichever way the vehicle actually crossed; `direction`
reports that separately. Turn the gate around in the config and the labels
still come out right.

Published on every valid passage:

    /scoring/gate    std_msgs/String   "left" or "right"
    /scoring/events  std_msgs/String   JSON, the stream a score counter consumes
"""

import json

from ament_index_python.packages import get_package_share_directory
from auv_scoring.crossing import PlaneCrossing, pose_from_ground_truth
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class GateFlagger(Node):

    def __init__(self):
        super().__init__('gate_flagger')

        share = get_package_share_directory('auv_worlds')
        self.declare_parameter('ground_truth_file', '')
        self.declare_parameter('config_file', '')
        self.declare_parameter('task', 'gate')
        self.declare_parameter('odometry_topic', '/model/auv/odometry')

        truth_path = self.get_parameter('ground_truth_file').value
        config_path = self.get_parameter('config_file').value
        self.task = self.get_parameter('task').value

        if not config_path:
            config_path = f'{share}/config/competition_config.yaml'

        self.geometry = self._load_geometry(config_path)
        self.plane = self._load_pose(truth_path)

        self.gate_pub = self.create_publisher(String, '/scoring/gate', 10)
        self.event_pub = self.create_publisher(String, '/scoring/events', 10)
        self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value, self.odometry_callback, 10)

        self.passages = 0

        self.get_logger().info(
            f'Gate flagger watching "{self.geometry["prop"]}" at '
            f'{np.round(self.plane.xyz, 3).tolist()}; opening y '
            f'{self.geometry["opening_y"]} split at {self.geometry["divider_y"]}')

    # -- setup --------------------------------------------------------------

    def _load_geometry(self, path):
        """Read the opening rectangle and divider, in the prop's own frame."""
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        task = (config.get('tasks') or {}).get(self.task) or {}
        scoring = task.get('scoring')
        if not scoring:
            raise RuntimeError(
                f'{path}: tasks.{self.task} has no `scoring:` block, so there is no '
                'opening geometry to test crossings against.')
        for key in ('prop', 'opening_y', 'opening_z', 'divider_y'):
            if key not in scoring:
                raise RuntimeError(f'{path}: tasks.{self.task}.scoring is missing "{key}"')
        return scoring

    def _load_pose(self, path):
        """Build the crossing detector from this run's sampled gate pose."""
        if not path:
            raise RuntimeError(
                "ground_truth_file parameter is empty; the flagger needs the run's "
                'ground truth to know where the gate actually ended up.')
        with open(path) as handle:
            truth = json.load(handle)

        props = {p['name']: p for p in truth['props']}
        gate = props.get(self.geometry['prop'])
        if gate is None:
            raise RuntimeError(
                f'{path}: no prop named "{self.geometry["prop"]}"; '
                f'have {sorted(props)}')

        vehicle = next((p for p in truth['props'] if p['task'] == 'vehicle'), None)
        if vehicle is None:
            raise RuntimeError(f'{path}: no vehicle entry')

        xyz, rot = pose_from_ground_truth(gate)
        return PlaneCrossing(
            self.geometry['prop'], xyz, rot,
            self.geometry['opening_y'], self.geometry['opening_z'],
            self.geometry['divider_y'], vehicle['actual'][:3])

    # -- detection ----------------------------------------------------------

    def odometry_callback(self, msg):
        p = msg.pose.pose.position
        event = self.plane.update([p.x, p.y, p.z])
        if event is None:
            return
        if event['outside']:
            self.get_logger().info(
                f'Gate plane crossed outside the opening at local '
                f'y={event["local"][1]:+.2f} z={event["local"][2]:+.2f} - not counted.')
            return

        self.passages += 1
        self.gate_pub.publish(String(data=event['side']))
        self.event_pub.publish(String(data=json.dumps({
            'task': self.task,
            'event': 'passed',
            'side': event['side'],
            'direction': 'forward' if event['forward'] else 'reverse',
            'passage': self.passages,
            'sim_time': self.get_clock().now().nanoseconds * 1e-9,
            'crossing_world': event['crossing_world'],
        })))
        self.get_logger().info(
            f'Gate passage {self.passages}: {event["side"]} half, '
            f'{"forward" if event["forward"] else "reverse"}.')


def main(args=None):
    rclpy.init(args=args)
    node = GateFlagger()
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
