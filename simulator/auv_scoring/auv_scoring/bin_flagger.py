"""Flag a dropped marker landing inside one of the four bins.

Each bin carries its own contact sensor, so a contact message already says which
bin was hit and, from the other collision's name, which marker hit it. What a
bare contact cannot say is whether the marker went IN: a marker glancing off an
outside wall or settling on the rim touches the same collision as one sitting in
the bottom.

So the contact position - which Gazebo reports in world coordinates - is
transformed into the bin prop's frame and tested against that bin's interior,
shrunk by the wall thickness. Only a contact genuinely inside the cavity counts.

The prop pose comes from the run's ground truth, so this tracks wherever the
course generator put the bins.

Published per marker landed:

    /scoring/bins    std_msgs/String   "<marker>:<bin>"
    /scoring/events  std_msgs/String   JSON, consumed by the score keeper
"""

import json

from ament_index_python.packages import get_package_share_directory
from auv_scoring.crossing import pose_from_ground_truth
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
import yaml


class BinFlagger(Node):

    def __init__(self):
        super().__init__('bin_flagger')

        share = get_package_share_directory('auv_worlds')
        self.declare_parameter('ground_truth_file', '')
        self.declare_parameter('config_file', f'{share}/config/competition_config.yaml')
        self.declare_parameter('task', 'bins')

        self.task = self.get_parameter('task').value
        self.geometry = self._load_geometry(self.get_parameter('config_file').value)
        self.prop_xyz, self.prop_rot = self._load_pose(
            self.get_parameter('ground_truth_file').value)

        self.markers = list(self.geometry.get('markers') or [])
        self.landed = set()

        self.bins_pub = self.create_publisher(String, '/scoring/bins', 10)
        self.event_pub = self.create_publisher(String, '/scoring/events', 10)
        self.create_subscription(String, '/scoring/course', self.course_callback, 10)
        # A marker that has been picked back up onto the hull is a new drop, so
        # the "this one has already landed" latch has to go with it.
        self.create_subscription(
            String, '/scoring/payload_reloaded', self.reload_callback, 10)
        for spec in self.geometry['bins']:
            self.create_subscription(
                Contacts, f'/model/{self.geometry["prop"]}/{spec["name"]}/contact',
                self._make_callback(spec), 10)

        self.get_logger().info(
            f'Bin flagger watching {len(self.geometry["bins"])} bins on '
            f'"{self.geometry["prop"]}" at {np.round(self.prop_xyz, 3).tolist()}; '
            f'markers {self.markers}')

    # -- setup --------------------------------------------------------------

    def _load_geometry(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get(self.task) or {}).get('scoring')
        if not scoring:
            raise RuntimeError(f'{path}: tasks.{self.task} has no `scoring:` block')
        for key in ('prop', 'bins'):
            if key not in scoring:
                raise RuntimeError(f'{path}: tasks.{self.task}.scoring is missing "{key}"')
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
        """Re-read the bin pose after a reset re-seeded the course.

        Note what this cannot fix: a marker that has already been dropped stays
        on the pool floor, because its DetachableJoint is only re-welded by a
        relaunch. Clearing `landed` is therefore about the next run being able
        to score at all, not about the markers being back on the vehicle.
        """
        path = msg.data.strip()
        try:
            prop_xyz, prop_rot = self._load_pose(path)
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            self.get_logger().error(
                f'Ignoring course update {path}: {exc}. Still scoring against the '
                'bin pose this run started with.')
            return

        self.prop_xyz, self.prop_rot = prop_xyz, prop_rot
        self.landed = set()
        self.get_logger().info(
            f'Course reloaded from {path}; bins now at '
            f'{np.round(self.prop_xyz, 3).tolist()}.')

    def reload_callback(self, msg):
        """A marker is back on the vehicle, so let it score again."""
        name = msg.data.strip()
        if name in self.landed:
            self.landed.discard(name)
            self.get_logger().info(f'{name} reloaded; ready to land again.')

    # -- detection ----------------------------------------------------------

    def _marker_in(self, contact):
        """Which of our markers this contact involves, if any."""
        for name in (contact.collision1.name, contact.collision2.name):
            for marker in self.markers:
                # Gazebo reports names as model::link::collision, and the marker
                # is a nested model so its own name appears in that chain.
                if marker in name.split('::'):
                    return marker
        return None

    def _inside(self, spec, world_point):
        """Report whether a world contact point is inside this bin's cavity."""
        local = self.prop_rot.inv().apply(np.asarray(world_point) - self.prop_xyz)
        inset = float(self.geometry.get('wall_inset', 0.0))
        for axis, key in enumerate(('x', 'y')):
            lo, hi = spec[key]
            if not (lo + inset <= local[axis] <= hi - inset):
                return False, local
        z_lo, z_hi = spec['z']
        # No inset at the bottom - a marker resting on the floor of the bin is
        # exactly what scoring wants - but it must be under the rim.
        return (z_lo <= local[2] <= z_hi), local

    def _make_callback(self, spec):
        def callback(msg):
            for contact in msg.contacts:
                marker = self._marker_in(contact)
                if marker is None or marker in self.landed:
                    continue
                for position in contact.positions:
                    inside, local = self._inside(spec, [position.x, position.y, position.z])
                    if inside:
                        self._land(marker, spec['name'], local)
                        return
                self.get_logger().info(
                    f'{marker} touched {spec["name"]} outside the cavity at local '
                    f'{np.round(local, 3).tolist()} - not counted.',
                    throttle_duration_sec=2.0)
        return callback

    def _land(self, marker, bin_name, local):
        self.landed.add(marker)
        self.bins_pub.publish(String(data=f'{marker}:{bin_name}'))
        self.event_pub.publish(String(data=json.dumps({
            'task': self.task,
            'event': 'marker_in_bin',
            'marker': marker,
            'bin': bin_name,
            'sim_time': self.get_clock().now().nanoseconds * 1e-9,
            'local_point': [round(float(v), 3) for v in local],
        })))
        self.get_logger().info(f'{marker} landed in {bin_name}.')


def main(args=None):
    rclpy.init(args=args)
    node = BinFlagger()
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
