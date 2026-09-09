"""Flag a fired torpedo passing through one of the target board's holes.

Scoring-side code, so it is allowed ground truth: it reads the board's actual
sampled pose out of the run's ground truth file and the torpedoes' world poses
off the simulator, neither of which a competitor is meant to see.

The board is static and collision-free - the AUV is meant to fire THROUGH it
without any reaction force on the prop - so there is nothing to hang a contact
sensor on and this cannot work the way bin_flagger does. It is a plane crossing
instead, the same idea the gate and the slalom use: the board mesh is thin along
its local X, so a torpedo's signed local X flips as it goes through. On a flip
the crossing point is interpolated onto the plane and tested against each hole
rectangle, which rejects a torpedo that sails past the side of the board, over
the top, or through the wide slot in the frame.

Each torpedo scores at most once per SHOT. The vehicle carries one and the fire
button reloads it, so simulator_bridge announces a reload on
/scoring/payload_reloaded and that clears this node's per-torpedo latch - which
is what lets the next shot score.

The torpedo is a TOP-LEVEL model welded to the hull by a DetachableJoint, not a
model nested inside the AUV, so its OdometryPublisher reports a world pose and
this node can use it as it stands. The vehicle's own odometry is still needed,
but only to tell a fired torpedo from a carried one - see LAUNCH_DISPLACEMENT.

The world's own pose feeds (dynamic_pose/info and pose/info) would carry the
torpedo too, but they reach ROS with their name fields empty, so there is no
telling which pose belongs to which model.

Published on a valid hit:

    /scoring/events  std_msgs/String  JSON, the stream the score counter consumes
"""

import json

from ament_index_python.packages import get_package_share_directory
from auv_scoring.crossing import pose_from_ground_truth
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import String
import yaml

# How far clear of the board a torpedo must get before another crossing counts.
# Matches crossing.REARM_DISTANCE for the same reason: one pass, one event.
REARM_DISTANCE = 0.30

# How far a torpedo must move in the vehicle's frame before it counts as fired.
# A torpedo still bolted to the hull sits at a FIXED offset from the vehicle,
# so this separates "launched" from "carried" without needing to hear about the
# firing at all. It matters: without it, driving the vehicle through the board
# plane would line a carried torpedo up with a hole and score a hit nobody
# fired - and at an outreach stand somebody will absolutely drive through the
# target.
#
# Generous, because the two feeds are not synchronised: the vehicle publishes
# odometry at 20 Hz and the torpedo at 50 Hz, so composing one against the other
# while the vehicle is moving at 1.4 m/s already puts 0.07 m of skew into a
# torpedo that has not moved at all. A fired one clears this in a tenth of a
# second, so nothing is lost by being well clear of the noise.
LAUNCH_DISPLACEMENT = 0.25


class TorpedoFlagger(Node):

    def __init__(self):
        super().__init__('torpedo_flagger')

        share = get_package_share_directory('auv_worlds')
        self.declare_parameter('ground_truth_file', '')
        self.declare_parameter('config_file', '')
        self.declare_parameter('task', 'torpedo')
        self.declare_parameter('odometry_topic', '/model/auv/odometry')

        truth_path = self.get_parameter('ground_truth_file').value
        config_path = self.get_parameter('config_file').value or \
            f'{share}/config/competition_config.yaml'
        self.task = self.get_parameter('task').value

        self.geometry = self._load_geometry(config_path)
        self.xyz, self.rot = self._load_pose(truth_path)

        self.event_pub = self.create_publisher(String, '/scoring/events', 10)

        # Latest vehicle world pose, the frame every torpedo pose is relative
        # to. None until the first odometry message, which is why torpedo poses
        # arriving before it are dropped rather than composed against nothing.
        self.auv = None
        self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value,
            self.odometry_callback, 10)

        # Per-torpedo crossing state: the last local position seen, and whether
        # this torpedo is allowed to score again yet.
        self.tracks = {name: {'previous': None, 'armed': True, 'scored': False,
                              'mounted': None, 'launched': False}
                       for name in self.geometry['torpedoes']}
        for name in self.geometry['torpedoes']:
            self.create_subscription(
                Odometry, f'/model/{name}/odometry',
                lambda msg, n=name: self.pose_callback(n, msg), 10)

        # A reloaded torpedo is back in its tube and is a new shot.
        self.create_subscription(
            String, '/scoring/payload_reloaded', self.reload_callback, 10)

        holes = ', '.join(h['name'] for h in self.geometry['holes'])
        self.get_logger().info(
            f'Torpedo flagger watching "{self.geometry["prop"]}" at '
            f'{np.round(self.xyz, 3).tolist()}; holes: {holes}; '
            f'torpedoes: {", ".join(self.geometry["torpedoes"])}')

    # -- setup --------------------------------------------------------------

    def _load_geometry(self, path):
        with open(path) as handle:
            config = yaml.safe_load(handle) or {}
        scoring = ((config.get('tasks') or {}).get(self.task) or {}).get('scoring') or {}
        holes = scoring.get('holes') or []
        if not holes:
            raise RuntimeError(
                f'{path}: tasks.{self.task}.scoring.holes is empty; the torpedo '
                'flagger has nothing to test a crossing against.')

        # Shrink each hole so clipping the very rim does not count as a clean
        # pass, the same idea as the bins' wall_inset.
        inset = float(scoring.get('edge_inset', 0.0))
        shrunk = []
        for hole in holes:
            y0, y1 = (float(v) for v in hole['y'])
            z0, z1 = (float(v) for v in hole['z'])
            if y1 - y0 <= 2 * inset or z1 - z0 <= 2 * inset:
                raise RuntimeError(
                    f'{path}: hole {hole.get("name")!r} is smaller than twice '
                    f'edge_inset ({inset}), so nothing could ever pass it.')
            shrunk.append({
                'name': str(hole.get('name', 'hole')),
                'y': (y0 + inset, y1 - inset),
                'z': (z0 + inset, z1 - inset),
            })
        return {
            'prop': scoring.get('prop', 'task4_torpedo'),
            'holes': shrunk,
            'torpedoes': list(scoring.get('torpedoes') or ['torpedo_left']),
        }

    def _load_pose(self, truth_path):
        if not truth_path:
            raise RuntimeError(
                'ground_truth_file parameter is empty; the torpedo flagger '
                'needs it to know where the board actually ended up.')
        with open(truth_path) as handle:
            truth = json.load(handle)
        for entry in truth.get('props', []):
            if entry.get('name') == self.geometry['prop']:
                return pose_from_ground_truth(entry)
        raise RuntimeError(
            f'{truth_path}: no prop named {self.geometry["prop"]!r}; the '
            'torpedo task may not be in this course.')

    # -- detection ----------------------------------------------------------

    def reload_callback(self, msg):
        """Forget everything about a torpedo that has just been put back.

        `mounted` is cleared rather than kept: the vehicle may have moved, and
        the next pose message re-latches the tube position anyway.
        """
        name = msg.data.strip()
        if name not in self.tracks:
            return
        self.tracks[name] = {'previous': None, 'armed': True, 'scored': False,
                             'mounted': None, 'launched': False}
        self.get_logger().info(f'{name} reloaded; ready to score again.')

    def odometry_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.auv = (np.array([p.x, p.y, p.z]),
                    Rot.from_quat([q.x, q.y, q.z, q.w]))

    def pose_callback(self, name, msg):
        """A torpedo's world pose, straight off its OdometryPublisher."""
        if self.auv is None:
            return

        p = msg.pose.pose.position
        world = np.array([p.x, p.y, p.z])
        auv_xyz, auv_rot = self.auv
        # Where it sits on the hull. Constant while it is still in the tube,
        # whatever the vehicle is doing, which is what the launch test needs.
        relative = auv_rot.inv().apply(world - auv_xyz)
        track = self.tracks[name]

        # Still in its tube? Then it cannot hit anything, whatever the vehicle
        # happens to be flying through.
        if track['mounted'] is None:
            track['mounted'] = relative
        if not track['launched']:
            if np.linalg.norm(relative - track['mounted']) < LAUNCH_DISPLACEMENT:
                return
            track['launched'] = True

        if track['previous'] is None:
            # One line per torpedo at the moment it leaves, so a frame that has
            # gone wrong is obvious in the log rather than showing up as a hit
            # that never comes: the distance should be the range to the board.
            local = self.rot.inv().apply(world - self.xyz)
            self.get_logger().info(
                f'{name} away: world {np.round(world, 2).tolist()}, '
                f'{np.linalg.norm(world - self.xyz):.1f} m from the board '
                f'(local x={local[0]:+.2f}, y={local[1]:+.2f}, z={local[2]:+.2f}).')

        self.update(name, world)

    def update(self, name, position):
        track = self.tracks[name]
        local = self.rot.inv().apply(position - self.xyz)
        previous, track['previous'] = track['previous'], local
        if previous is None:
            return

        if not track['armed']:
            if abs(local[0]) > REARM_DISTANCE:
                track['armed'] = True
            return

        if np.sign(previous[0]) == np.sign(local[0]):
            return

        span = local[0] - previous[0]
        if abs(span) < 1e-9:
            return
        crossing = previous + (local - previous) * (-previous[0] / span)
        track['armed'] = False

        hole = self.hole_at(crossing)
        if hole is None:
            self.get_logger().info(
                f'{name}: crossed the board at local y={crossing[1]:+.3f} '
                f'z={crossing[2]:+.3f} - not through a hole.')
            return

        if track['scored']:
            return
        track['scored'] = True

        world = self.xyz + self.rot.apply(crossing)
        self.event_pub.publish(String(data=json.dumps({
            'task': self.task,
            'event': 'torpedo_in_hole',
            'torpedo': name,
            'hole': hole['name'],
            'sim_time': self.get_clock().now().nanoseconds * 1e-9,
            'crossing_world': [round(float(v), 3) for v in world],
        })))
        self.get_logger().info(f'HIT: {name} through the {hole["name"]} hole.')

    def hole_at(self, crossing):
        for hole in self.geometry['holes']:
            if (hole['y'][0] <= crossing[1] <= hole['y'][1]
                    and hole['z'][0] <= crossing[2] <= hole['z'][1]):
                return hole
        return None


def main(args=None):
    rclpy.init(args=args)
    node = TorpedoFlagger()
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
