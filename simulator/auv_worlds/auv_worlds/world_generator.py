"""Build a randomised competition world from a layout description.

The layout file (config/competition_config.yaml) fixes the nominal pose of every
prop and how far it is allowed to stray. Each run draws a fresh pose per prop from
a seeded RNG, so no two launches see the same course while any single run stays
reproducible from its seed alone.

The base world supplies only the environment - pool, lighting, physics, plugins.
Every <include> in it is discarded and rebuilt from the layout, so the layout is
the single authority on what is in the course.

Ground truth (the sampled poses) is written to a separate file rather than
published on a topic, so it can be withheld from competitors while still being
available to a scorer.

Run standalone with:

    ros2 run auv_worlds generate_world --layout <path> --seed 42
"""

import argparse
from datetime import datetime, timezone
import json
import os
import random
import sys
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import yaml

DEFAULT_CONFIG = 'competition_config.yaml'
ZERO_RANDOMIZE = {'translation': [0.0, 0.0, 0.0], 'rotation_deg': [0.0, 0.0, 0.0]}


class LayoutError(ValueError):
    """The layout file is malformed. Message is aimed at whoever edits the YAML."""


# ---------------------------------------------------------------------------
# Pose helpers
#
# A pose is [x, y, z, roll, pitch, yaw] in metres and degrees. SDF's rpy is the
# extrinsic X-then-Y-then-Z convention, which is exactly scipy's lowercase 'xyz'.
# ---------------------------------------------------------------------------

def _as_pose(seq, where):
    if seq is None:
        return np.zeros(3), Rot.identity()
    if len(seq) != 6:
        raise LayoutError(f'{where}: pose needs 6 numbers [x y z roll pitch yaw], got {len(seq)}')
    values = [float(v) for v in seq]
    return np.array(values[:3]), Rot.from_euler('xyz', values[3:], degrees=True)


def _as_pose_list(xyz, rot):
    """Inverse of _as_pose, for recording ground truth in the layout's own units."""
    return [round(float(v), 6) for v in xyz] + \
           [round(float(a), 6) for a in rot.as_euler('xyz', degrees=True)]


def _sdf_pose(xyz, rot):
    """SDF wants radians."""
    rpy = rot.as_euler('xyz')
    return ' '.join(f'{v:.6f}' for v in (*xyz, *rpy))


def _perturb(xyz, rot, d_xyz, d_rpy_deg):
    """Offset along world axes, then rotate about world axes.

    Rotating on the left (d_rot * rot) rather than the right means a yaw range of
    +/-5 always means 5 degrees about the pool vertical, whatever orientation the
    prop's own mesh alignment gives it.
    """
    d_rot = Rot.from_euler('xyz', d_rpy_deg, degrees=True)
    return xyz + np.asarray(d_xyz, dtype=float), d_rot * rot


def _sample(rng, spec, where):
    """Draw a uniform offset from a half-range spec. Returns (d_xyz, d_rpy_deg)."""
    if spec is None:
        spec = ZERO_RANDOMIZE
    unknown = set(spec) - {'translation', 'rotation_deg'}
    if unknown:
        raise LayoutError(
            f'{where}: unknown randomize key(s) {sorted(unknown)}; '
            'expected "translation" and/or "rotation_deg"')

    def draw(key):
        limits = spec.get(key, [0.0, 0.0, 0.0])
        if len(limits) != 3:
            raise LayoutError(f'{where}.{key}: needs 3 numbers, got {len(limits)}')
        return [rng.uniform(-abs(float(v)), abs(float(v))) for v in limits]

    return draw('translation'), draw('rotation_deg')


# ---------------------------------------------------------------------------
# Layout -> placed props
# ---------------------------------------------------------------------------

def _placement(rng, task_name, origin, task_rand, jitter, prop, index):
    """Resolve one prop to a world pose, alongside its unrandomised nominal."""
    where = f'tasks.{task_name}.props[{index}]'
    for key in ('uri', 'name'):
        if not prop.get(key):
            raise LayoutError(f'{where}: missing required key "{key}"')

    origin_xyz, origin_rot = origin
    local_xyz, local_rot = _as_pose(prop.get('pose'), where)

    nominal_xyz = origin_xyz + origin_rot.apply(local_xyz)
    nominal_rot = origin_rot * local_rot

    # Task transform first (moves the whole group), then per-prop jitter.
    rand_origin_xyz, rand_origin_rot = _perturb(origin_xyz, origin_rot, *task_rand)
    jitter_sample = _sample(rng, jitter, f'{where}.jitter')
    jitter_xyz, jitter_rot = _perturb(local_xyz, local_rot, *jitter_sample)

    world_xyz = rand_origin_xyz + rand_origin_rot.apply(jitter_xyz)
    world_rot = rand_origin_rot * jitter_rot

    return {
        'task': task_name,
        'name': prop['name'],
        'uri': prop['uri'],
        'nominal': _as_pose_list(nominal_xyz, nominal_rot),
        'actual': _as_pose_list(world_xyz, world_rot),
        '_sdf_pose': _sdf_pose(world_xyz, world_rot),
    }


def _payload_placements(vehicle, placement):
    """Where the vehicle's carried payloads have to spawn.

    A DetachableJoint welds child to parent at whatever relative pose it finds
    at world load, so a payload that spawns anywhere but its mount point is
    welded on crooked and stays that way. The vehicle's own pose is sampled, so
    these have to be composed against the sample rather than written into the
    world by hand.

    They are NOT returned as placed props. Ground truth does not want them - a
    payload is not a course prop - and, more importantly, reset_run teleports
    everything in `placed`: a welded payload and its vehicle are one body to the
    physics engine, so moving them separately fights the joint.
    """
    payloads = vehicle.get('payloads') or []
    if not payloads:
        return []

    origin_xyz, origin_rot = _as_pose(placement['actual'], 'vehicle.actual')
    out = []
    for index, payload in enumerate(payloads):
        where = f'vehicle.payloads[{index}]'
        for key in ('uri', 'name'):
            if not payload.get(key):
                raise LayoutError(f'{where}: missing required key "{key}"')
        local_xyz, local_rot = _as_pose(payload.get('pose'), where)
        out.append({
            'name': payload['name'],
            'uri': payload['uri'],
            '_sdf_pose': _sdf_pose(origin_xyz + origin_rot.apply(local_xyz),
                                   origin_rot * local_rot),
        })
    return out


def place_props(layout, seed):
    """Sample a full course. Returns the list of placed props, in layout order."""
    rng = random.Random(seed)
    defaults = (layout.get('defaults') or {}).get('randomize')
    placed = []

    vehicle = layout.get('vehicle')
    if vehicle:
        origin = _as_pose(vehicle.get('pose'), 'vehicle')
        task_rand = _sample(rng, vehicle.get('randomize', defaults), 'vehicle.randomize')
        placement = _placement(rng, 'vehicle', origin, task_rand, None, vehicle, 0)
        # Private key: stripped from ground truth, read by build_world_tree.
        placement['_payloads'] = _payload_placements(vehicle, placement)
        placed.append(placement)

    tasks = layout.get('tasks') or {}
    if not isinstance(tasks, dict):
        raise LayoutError('tasks: expected a mapping of task name -> task definition')

    for task_name, task in tasks.items():
        if not isinstance(task, dict):
            raise LayoutError(f'tasks.{task_name}: expected a mapping')
        props = task.get('props')
        if not props:
            raise LayoutError(f'tasks.{task_name}: no props listed')

        origin = _as_pose(task.get('origin'), f'tasks.{task_name}.origin')
        # Sampled once per task so every prop in it moves as a rigid group.
        task_rand = _sample(rng, task.get('randomize', defaults),
                            f'tasks.{task_name}.randomize')
        jitter = task.get('jitter')

        for index, prop in enumerate(props):
            placed.append(_placement(rng, task_name, origin, task_rand, jitter,
                                     prop, index))

    names = [p['name'] for p in placed]
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        raise LayoutError(f'duplicate model name(s) {duplicates}; Gazebo needs them unique')

    return placed


# ---------------------------------------------------------------------------
# World assembly
# ---------------------------------------------------------------------------

def _absolutise_paths(root, base_dir):
    """Rewrite world-file-relative asset paths to absolute ones.

    The base world points at its textures with paths like
    ../media/textures/foo.png, which Gazebo resolves relative to the directory
    the world file sits in. The generated world lives somewhere else entirely, so
    those would silently resolve to nothing and the pool would render untextured.
    """
    rewritten = 0
    for element in root.iter():
        text = (element.text or '').strip()
        if not text.startswith(('./', '../')):
            continue
        resolved = os.path.normpath(os.path.join(base_dir, text))
        if os.path.exists(resolved):
            element.text = resolved
            rewritten += 1
    return rewritten


def build_world_tree(base_world_path, placed, seed):
    """Strip the base world's includes and rebuild them from the placed props."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(base_world_path, parser=parser)
    world = tree.getroot().find('world')
    if world is None:
        raise LayoutError(f'{base_world_path}: no <world> element')

    _absolutise_paths(tree.getroot(), os.path.dirname(os.path.abspath(base_world_path)))

    for include in world.findall('include'):
        world.remove(include)

    world.append(ET.Comment(
        f' Course generated by auv_worlds.world_generator from seed {seed}.\n'
        '       Edit config/competition_config.yaml, not this file - it is overwritten\n'
        '       on every launch. '))

    for prop in placed:
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = prop['uri']
        ET.SubElement(include, 'name').text = prop['name']
        ET.SubElement(include, 'pose').text = prop['_sdf_pose']

        # The vehicle's carried payloads, spawned at their mount points so the
        # AUV's DetachableJoints weld them on square. Emitted here rather than
        # as placed props of their own so nothing else treats them as course
        # geometry - see _payload_placements.
        for payload in prop.get('_payloads') or []:
            include = ET.SubElement(world, 'include')
            ET.SubElement(include, 'uri').text = payload['uri']
            ET.SubElement(include, 'name').text = payload['name']
            ET.SubElement(include, 'pose').text = payload['_sdf_pose']

    ET.indent(tree, space='  ')
    return tree


def make_run_id(seed):
    """Name one sampling of the course. Unique per second, and self-describing."""
    return f'{datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")}_seed{seed}'


def write_ground_truth(output_dir, run_id, seed, layout_path, base_world, world_path,
                       placed):
    """Record the sampled poses beside the run's world. Returns the path written.

    Kept out of the world file and off the ROS graph so it can be withheld from
    competitors; the flaggers read it. Separate from world generation because
    reset_run re-samples the course inside a Gazebo that is already running -
    new poses and new ground truth, but no new world file.
    """
    truth_dir = os.path.join(output_dir, 'ground_truth')
    os.makedirs(truth_dir, exist_ok=True)
    truth_path = os.path.join(truth_dir, f'competition_{run_id}.json')
    with open(truth_path, 'w') as handle:
        json.dump({
            'run_id': run_id,
            'seed': seed,
            'generated_utc': datetime.now(timezone.utc).isoformat(),
            'layout': os.path.abspath(layout_path),
            'base_world': base_world,
            'world': world_path,
            'props': [{k: v for k, v in prop.items() if not k.startswith('_')}
                      for prop in placed],
        }, handle, indent=2)
    return truth_path


def generate_world(layout_path, seed=None, output_dir=None, models_dir=None):
    """Generate one world. Returns a dict describing the run.

    seed: None draws a fresh one, so each call produces a different course.
    """
    with open(layout_path) as handle:
        layout = yaml.safe_load(handle)
    if not isinstance(layout, dict):
        raise LayoutError(f'{layout_path}: expected a YAML mapping at the top level')

    if seed is None:
        seed = layout.get('seed')
    if seed is None:
        seed = random.SystemRandom().randrange(2 ** 31)
    seed = int(seed)

    share = get_package_share_directory('auv_worlds')
    base_world = layout.get('base_world', 'robosub.sdf')
    if not os.path.isabs(base_world):
        base_world = os.path.join(share, 'worlds', base_world)
    if not os.path.isfile(base_world):
        raise LayoutError(f'base_world not found: {base_world}')

    placed = place_props(layout, seed)
    tree = build_world_tree(base_world, placed, seed)

    output_dir = output_dir or os.path.join(tempfile.gettempdir(), 'matsya_competition')
    os.makedirs(output_dir, exist_ok=True)
    run_id = make_run_id(seed)

    world_path = os.path.join(output_dir, f'competition_{run_id}.sdf')
    tree.write(world_path, encoding='utf-8', xml_declaration=True)

    truth_path = write_ground_truth(output_dir, run_id, seed, layout_path,
                                    base_world, world_path, placed)

    return {
        'seed': seed,
        'run_id': run_id,
        'world': world_path,
        'ground_truth': truth_path,
        'models_dir': models_dir or os.path.join(share, 'models'),
        'props': placed,
    }


def main(argv=None):
    """Command line entry point: generate one world and report where it went."""
    share = get_package_share_directory('auv_worlds')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--layout', default=os.path.join(share, 'config', DEFAULT_CONFIG),
                        help='layout YAML (default: the installed competition_config.yaml)')
    parser.add_argument('--seed', type=int, default=None,
                        help='pin the course; omit for a fresh random one')
    parser.add_argument('--output-dir', default=None,
                        help='where to write the world (default: $TMPDIR/matsya_competition)')
    parser.add_argument('--quiet', action='store_true', help='print only the world path')
    args = parser.parse_args(argv)

    try:
        run = generate_world(args.layout, args.seed, args.output_dir)
    except LayoutError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1

    if args.quiet:
        print(run['world'])
        return 0

    print(f'seed         : {run["seed"]}')
    print(f'world        : {run["world"]}')
    print(f'ground truth : {run["ground_truth"]}')
    print(f'props        : {len(run["props"])}')
    for prop in run['props']:
        dx = [a - n for a, n in zip(prop['actual'], prop['nominal'])]
        print(f'  {prop["task"]:>8s}/{prop["name"]:<24s} '
              f'offset {dx[0]:+.3f} {dx[1]:+.3f} {dx[2]:+.3f} m  yaw {dx[5]:+.2f} deg')
    return 0


if __name__ == '__main__':
    sys.exit(main())
