"""Flag a Task 5 pickup delivered onto one of the octagon's two side pads.

The gripper's whole job is to take an object off the table and put it somewhere
deliberate. This is the node that decides it got there.

WHY POSE AND NOT A CONTACT SENSOR. The bins next door use contact sensors, and
that is right for them: a bin is a cavity, the marker is thrown at it, and the
contact POINT is the only thing that separates "in" from "bounced off the rim".
The octagon's two pads are flat boards, and an object is placed rather than
thrown. A contact sensor there would fire while a still-gripped object is merely
brushed across the board, and would need the table's one mesh collision split in
two to say WHICH pad was touched. The pose says everything instead - which pad's
footprint the object is over, whether it is sitting on the surface or being
carried above it, and whether it has stopped moving - so each pickup carries an
OdometryPublisher (see rs2026_pickup_*/model.sdf) and this node reads that.

DELIVERED means all four of these at once:

  * inside a pad's footprint in the PROP's frame, shrunk by `edge_inset`
  * within `settle_height` of that pad's top face
  * moving slower than `settle_speed`
  * and having been all of the above continuously for `settle_time`

The dwell is what makes it a delivery and not a fly-past. It also means a held
object cannot score: the gripper only releases on an explicit open, and an
object still in the jaw moves with the vehicle, which never holds that still
that long over a 0.54 x 0.366 m pad.

Each object scores once. Which pad it goes on does not matter - the ask was
"either box" - but the pad is reported so the log and the score card say where
it went.

The prop pose comes from the run's ground truth, so this tracks wherever the
course generator put the octagon.

Published per object delivered:

    /scoring/octagon   std_msgs/String   "<object>:<pad>"
    /scoring/events    std_msgs/String   JSON, consumed by the score keeper
"""

import json

from ament_index_python.packages import get_package_share_directory
from auv_scoring.crossing import pose_from_ground_truth
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class OctagonFlagger(Node):

    def __init__(self):
        super().__init__('octagon_flagger')

        share = get_package_share_directory('auv_worlds')
        self.declare_parameter('ground_truth_file', '')
        self.declare_parameter('config_file', f'{share}/config/competition_config.yaml')
        self.declare_parameter('task', 'octagon')

        self.task = self.get_parameter('task').value
        self.geometry = self._load_geometry(self.get_parameter('config_file').value)
        self.prop_xyz, self.prop_rot = self._load_pose(
            self.get_parameter('ground_truth_file').value)

        self.objects = list(self.geometry.get('objects') or [])
        self.pads = list(self.geometry.get('pads') or [])
        self.edge_inset = float(self.geometry.get('edge_inset', 0.0))
        self.settle_height = float(self.geometry.get('settle_height', 0.09))
        self.settle_speed = float(self.geometry.get('settle_speed', 0.05))
        self.settle_time = float(self.geometry.get('settle_time', 0.5))

        # Per object: which pad it has been sitting on and since when. `since`
        # is None whenever it is not settled on anything, so the dwell restarts
        # from scratch rather than accumulating across a bounce.
        self.resting = {name: {'pad': None, 'since': None} for name in self.objects}
        self.delivered = set()

        self.octagon_pub = self.create_publisher(String, '/scoring/octagon', 10)
        self.event_pub = self.create_publisher(String, '/scoring/events', 10)
        self.create_subscription(String, '/scoring/course', self.course_callback, 10)
        for name in self.objects:
            self.create_subscription(
                Odometry, f'/model/{name}/odometry',
                self._make_callback(name), 10)

        self.get_logger().info(
            f'Octagon flagger watching {len(self.objects)} pickups against '
            f'{len(self.pads)} pads on "{self.geometry["prop"]}" at '
            f'{np.round(self.prop_xyz, 3).tolist()}; objects {self.objects}')

    # -- setup --------------------------------------------------------------

    def _load_geometry(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get(self.task) or {}).get('scoring')
        if not scoring:
            raise RuntimeError(f'{path}: tasks.{self.task} has no `scoring:` block')
        for key in ('prop', 'pads', 'objects'):
            if key not in scoring:
                raise RuntimeError(
                    f'{path}: tasks.{self.task}.scoring is missing "{key}"')
        for pad in scoring['pads']:
            for key in ('name', 'x', 'y', 'z'):
                if key not in pad:
                    raise RuntimeError(
                        f'{path}: pad {pad.get("name")!r} is missing "{key}"')
        return scoring

    def _load_pose(self, path):
        if not path:
            raise RuntimeError('ground_truth_file parameter is empty')
        with open(path) as handle:
            truth = json.load(handle)
        entry = next((p for p in truth['props'] if p['name'] == self.geometry['prop']), None)
        if entry is None:
            raise RuntimeError(f'{path}: no prop named "{self.geometry["prop"]}"')
        return pose_from_ground_truth(entry)

    def course_callback(self, msg):
        """Re-read the octagon's pose after a reset re-seeded the course.

        The pickups are teleported back onto the table by the run manager, so
        unlike the bins' markers they really are ready to be delivered again -
        which is why the latch is cleared here without qualification.
        """
        path = msg.data.strip()
        try:
            prop_xyz, prop_rot = self._load_pose(path)
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            self.get_logger().error(
                f'Ignoring course update {path}: {exc}. Still scoring against the '
                'octagon pose this run started with.')
            return

        self.prop_xyz, self.prop_rot = prop_xyz, prop_rot
        self.delivered = set()
        self.resting = {name: {'pad': None, 'since': None} for name in self.objects}
        self.get_logger().info(
            f'Course reloaded from {path}; octagon now at '
            f'{np.round(self.prop_xyz, 3).tolist()}.')

    # -- detection ----------------------------------------------------------

    def _pad_under(self, world_point):
        """The pad this world point is resting on, and the local point.

        Returns (pad_spec_or_None, local_point). The footprint is shrunk by
        `edge_inset` so an object teetering half off the edge is not a
        delivery; the height band runs from the pad's top face up by
        `settle_height`, which is deep enough for the tallest pickup to be
        sitting on the surface and shallow enough that one carried over at
        gripper height is not counted.
        """
        local = self.prop_rot.inv().apply(np.asarray(world_point) - self.prop_xyz)
        for pad in self.pads:
            x_lo, x_hi = pad['x']
            y_lo, y_hi = pad['y']
            if not (x_lo + self.edge_inset <= local[0] <= x_hi - self.edge_inset):
                continue
            if not (y_lo + self.edge_inset <= local[1] <= y_hi - self.edge_inset):
                continue
            top = float(pad['z'])
            if top <= local[2] <= top + self.settle_height:
                return pad, local
        return None, local

    def _make_callback(self, name):
        def callback(msg):
            if name in self.delivered:
                return

            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            speed = float(np.linalg.norm([v.x, v.y, v.z]))
            pad, local = self._pad_under([p.x, p.y, p.z])
            track = self.resting[name]

            if pad is None or speed > self.settle_speed:
                # Off a pad, or still moving: the dwell has to start again.
                track['pad'] = None
                track['since'] = None
                return

            now = self.get_clock().now().nanoseconds * 1e-9
            if track['pad'] != pad['name']:
                track['pad'] = pad['name']
                track['since'] = now
                return
            if track['since'] is None:
                track['since'] = now
                return
            if now - track['since'] >= self.settle_time:
                self._deliver(name, pad['name'], local)
        return callback

    def _deliver(self, name, pad_name, local):
        self.delivered.add(name)
        self.octagon_pub.publish(String(data=f'{name}:{pad_name}'))
        self.event_pub.publish(String(data=json.dumps({
            'task': self.task,
            'event': 'object_in_box',
            'object': name,
            'pad': pad_name,
            'sim_time': self.get_clock().now().nanoseconds * 1e-9,
            'local_point': [round(float(v), 3) for v in local],
        })))
        self.get_logger().info(
            f'{name} delivered onto {pad_name} at local '
            f'{np.round(local, 3).tolist()}.')


def main(args=None):
    rclpy.init(args=args)
    node = OctagonFlagger()
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
