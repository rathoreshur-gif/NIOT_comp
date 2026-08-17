"""Shared plane-crossing detection for task flaggers.

Every "did the vehicle go through it" task in this course reduces to the same
question: did the vehicle cross a prop's plane, inside an opening rectangle, and
on which side of a divider. The gate asks it once; the slalom asks it three
times. This holds that logic so the flaggers only have to describe geometry.

All detectors work in the PROP's frame, and the prop pose handed in is the
per-run randomised one, so detection tracks wherever the course generator
actually put things.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation as Rot

# How far clear of the plane the vehicle must get before another crossing
# counts, so hovering in a gap does not emit a stream of events.
REARM_DISTANCE = 0.30


def pose_from_ground_truth(entry):
    """Split a ground truth prop record into (position, rotation)."""
    return (np.array(entry['actual'][:3]),
            Rot.from_euler('xyz', entry['actual'][3:], degrees=True))


class PlaneCrossing:
    """One crossable plane: a prop pose, an opening rectangle, and a divider.

    `opening_y`, `opening_z` and `divider_y` are all in the prop's own frame.
    The prop's local X is the through axis - true for the gate mesh, and for the
    slalom red poles, whose local X points along the run of the course.
    """

    def __init__(self, name, xyz, rot, opening_y, opening_z, divider_y, vehicle_start):
        self.name = name
        self.xyz = np.asarray(xyz, dtype=float)
        self.rot = rot
        self.opening_y = opening_y
        self.opening_z = opening_z
        self.divider_y = float(divider_y)

        # Which side of the plane the vehicle spawned on. Crossing away from it
        # is "forward"; coming back is "reverse".
        self.start_side = math.copysign(
            1.0, self.rot.inv().apply(np.asarray(vehicle_start, dtype=float) - self.xyz)[0])

        # Handedness of the intended pass, resolved once. Deliberately keyed to
        # the forward direction rather than to how the vehicle happens to be
        # moving: the two halves are physically different, so a half has to keep
        # its name whichever way it was crossed.
        forward = self.rot.apply([1.0, 0.0, 0.0]) * -self.start_side
        left_dir = np.cross([0.0, 0.0, 1.0], forward)
        self.left_sense = float(np.dot(left_dir, self.rot.apply([0.0, 1.0, 0.0])))

        self.previous = None
        self.armed = True

    def update(self, position):
        """Feed a world position. Returns an event dict on a valid crossing.

        Returns None when nothing happened, and a dict with `outside` set when
        the plane was crossed but missed the opening - callers usually just log
        that.
        """
        local = self.rot.inv().apply(np.asarray(position, dtype=float) - self.xyz)
        previous, self.previous = self.previous, local
        if previous is None:
            return None

        if not self.armed:
            if abs(local[0]) > REARM_DISTANCE:
                self.armed = True
            return None

        if np.sign(previous[0]) == np.sign(local[0]):
            return None

        span = local[0] - previous[0]
        if abs(span) < 1e-9:
            return None
        crossing = previous + (local - previous) * (-previous[0] / span)
        self.armed = False

        y_min, y_max = self.opening_y
        z_min, z_max = self.opening_z
        if not (y_min <= crossing[1] <= y_max and z_min <= crossing[2] <= z_max):
            return {'name': self.name, 'outside': True,
                    'local': [float(v) for v in crossing]}

        offset = crossing[1] - self.divider_y
        return {
            'name': self.name,
            'outside': False,
            'side': 'left' if offset * self.left_sense > 0 else 'right',
            'forward': math.copysign(1.0, previous[0]) == self.start_side,
            'crossing_world': [round(float(v), 3)
                               for v in self.xyz + self.rot.apply(crossing)],
        }
