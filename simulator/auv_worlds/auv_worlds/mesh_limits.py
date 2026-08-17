"""Randomisation limits for competition_config.yaml, derived from the meshes.

The layout file says how far each task may stray from its nominal pose. Those
half-ranges are not free: push a prop far enough and it leaves the pool, sinks
into the floor, breaks the surface, or closes the gap to its neighbour below
what the vehicle needs to swim through. This module reads the actual mesh
geometry and reports the largest half-range that still satisfies all of it, so
the numbers in the YAML can be justified instead of guessed.

    ros2 run auv_worlds mesh_limits
    ros2 run auv_worlds mesh_limits --fraction 0.8 --yaml

Mesh reading is deliberately self-contained - it resolves SDF `relative_to`
pose chains and reads STL, COLLADA and binary glTF - because the alternative
is standing up a renderer just to ask how big a prop is.
"""

import argparse
import itertools
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as Rot
import yaml

from ament_index_python.packages import get_package_share_directory



# --------------------------------------------------------------------- meshes

def stl_points(path):
    size = os.path.getsize(path)
    with open(path, 'rb') as fh:
        head = fh.read(84)
        n = struct.unpack('<I', head[80:84])[0] if len(head) >= 84 else 0
        # Binary STL is exactly 84 + 50n bytes. Anything else is ascii, whatever
        # the leading keyword says - some exporters write "solid" into binary files.
        if size != 84 + 50 * n:
            fh.seek(0)
            txt = fh.read().decode('ascii', 'replace').split()
            pts = []
            for i, tok in enumerate(txt):
                if tok == 'vertex':
                    pts.append(tuple(float(v) for v in txt[i + 1:i + 4]))
            return np.array(pts) if pts else np.zeros((0, 3))
        raw = fh.read(n * 50)
    pts = np.zeros((n * 3, 3))
    for i in range(n):
        off = i * 50 + 12
        for k in range(3):
            pts[i * 3 + k] = struct.unpack('<3f', raw[off + k * 12: off + k * 12 + 12])
    return pts


def dae_points(path):
    root = ET.parse(path).getroot()
    ns = root.tag.split('}')[0].strip('{')
    q = lambda t: '{%s}%s' % (ns, t)

    unit = root.find(q('asset') + '/' + q('unit'))
    scale = float(unit.get('meter', 1)) if unit is not None else 1.0

    geo = {}
    for g in root.iter(q('geometry')):
        verts = g.find('.//' + q('vertices'))
        want = None
        if verts is not None:
            inp = verts.find(q('input'))
            if inp is not None:
                want = inp.get('source', '').lstrip('#')
        pts = []
        for src in g.iter(q('source')):
            fa = src.find(q('float_array'))
            if fa is None:
                continue
            acc = src.find('.//' + q('accessor'))
            if acc is None or acc.get('stride') != '3':
                continue
            if [p.get('name') for p in acc.findall(q('param'))][:3] != ['X', 'Y', 'Z']:
                continue
            if want and src.get('id') != want:
                continue
            v = [float(x) for x in fa.text.split()]
            pts.extend((v[i], v[i + 1], v[i + 2]) for i in range(0, len(v), 3))
        geo[g.get('id')] = np.array(pts) if pts else np.zeros((0, 3))

    def node_matrix(node):
        M = np.eye(4)
        for child in node:
            tag = child.tag.split('}')[-1]
            if tag == 'matrix':
                M = M @ np.array([float(x) for x in child.text.split()]).reshape(4, 4)
            elif tag == 'translate':
                T = np.eye(4)
                T[:3, 3] = [float(x) for x in child.text.split()]
                M = M @ T
            elif tag == 'scale':
                S = np.eye(4)
                S[:3, :3] = np.diag([float(x) for x in child.text.split()])
                M = M @ S
            elif tag == 'rotate':
                v = [float(x) for x in child.text.split()]
                R = np.eye(4)
                axis = np.array(v[:3], dtype=float)
                if np.linalg.norm(axis) > 0:
                    R[:3, :3] = Rot.from_rotvec(
                        axis / np.linalg.norm(axis) * math.radians(v[3])).as_matrix()
                M = M @ R
        return M

    out = []

    def walk(node, M):
        M = M @ node_matrix(node)
        for ig in node.findall(q('instance_geometry')):
            pts = geo.get(ig.get('url').lstrip('#'))
            if pts is not None and len(pts):
                out.append(pts @ M[:3, :3].T + M[:3, 3])
        for ig in node.findall(q('instance_node')):
            tgt = ig.get('url', '').lstrip('#')
            for cand in root.iter(q('node')):
                if cand.get('id') == tgt:
                    walk(cand, M)
        for child in node.findall(q('node')):
            walk(child, M)

    for vs in root.iter(q('visual_scene')):
        for node in vs.findall(q('node')):
            walk(node, np.eye(4))

    if not out:
        return np.zeros((0, 3))
    return np.vstack(out) * scale


def glb_points(path):
    """Binary glTF: POSITION accessors, pushed through the node hierarchy."""
    import json
    with open(path, 'rb') as fh:
        magic, _, _ = struct.unpack('<4sII', fh.read(12))
        if magic != b'glTF':
            raise ValueError(f'{path}: not a GLB')
        js, bin_chunk = None, b''
        while True:
            hdr = fh.read(8)
            if len(hdr) < 8:
                break
            clen, ctype = struct.unpack('<II', hdr)
            data = fh.read(clen)
            if ctype == 0x4E4F534A:
                js = json.loads(data)
            elif ctype == 0x004E4942:
                bin_chunk = data
    g = js
    views = g.get('bufferViews', [])
    CT = {5120: ('b', 1), 5121: ('B', 1), 5122: ('h', 2),
          5123: ('H', 2), 5125: ('I', 4), 5126: ('f', 4)}

    def read_accessor(i):
        acc = g['accessors'][i]
        fmt, sz = CT[acc['componentType']]
        ncomp = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4,
                 'MAT4': 16}[acc['type']]
        bv = views[acc['bufferView']]
        base = bv.get('byteOffset', 0) + acc.get('byteOffset', 0)
        stride = bv.get('byteStride') or ncomp * sz
        out = np.empty((acc['count'], ncomp))
        for k in range(acc['count']):
            off = base + k * stride
            out[k] = struct.unpack_from('<' + fmt * ncomp, bin_chunk, off)
        return out

    def node_M(node):
        if 'matrix' in node:
            return np.array(node['matrix']).reshape(4, 4).T  # glTF is column-major
        M = np.eye(4)
        if 'scale' in node:
            M[:3, :3] = np.diag(node['scale'])
        if 'rotation' in node:
            M[:3, :3] = Rot.from_quat(node['rotation']).as_matrix() @ M[:3, :3]
        if 'translation' in node:
            M[:3, 3] = node['translation']
        return M

    out = []

    def walk(idx, M):
        node = g['nodes'][idx]
        M = M @ node_M(node)
        if 'mesh' in node:
            for prim in g['meshes'][node['mesh']].get('primitives', []):
                pos = prim.get('attributes', {}).get('POSITION')
                if pos is None:
                    continue
                pts = read_accessor(pos)[:, :3]
                out.append(pts @ M[:3, :3].T + M[:3, 3])
        for child in node.get('children', []):
            walk(child, M)

    scene = g.get('scene', 0)
    for idx in g['scenes'][scene].get('nodes', []):
        walk(idx, np.eye(4))
    if not out:
        return np.zeros((0, 3))
    return np.vstack(out)


def mesh_points(uri, model_dir, models_dir):
    rel = uri.strip()
    if rel.startswith('model://'):
        rel = rel[len('model://'):]
        name, _, tail = rel.partition('/')
        path = os.path.join(models_dir, name, tail)
    else:
        path = os.path.join(model_dir, rel)
    if not os.path.isfile(path):
        stem, ext = os.path.splitext(path)
        for alt in ('.dae', '.DAE', '.stl', '.STL'):
            if os.path.isfile(stem + alt):
                path = stem + alt
                break
        else:
            raise FileNotFoundError(path)
    low = path.lower()
    if low.endswith('.dae'):
        return dae_points(path), path
    if low.endswith('.glb') or low.endswith('.gltf'):
        return glb_points(path), path
    return stl_points(path), path


# ------------------------------------------------------------------ model sdf

def pose_of(el):
    p = el.find('pose')
    if p is None or not (p.text or '').strip():
        return np.zeros(3), Rot.identity()
    v = [float(x) for x in p.text.split()]
    return np.array(v[:3]), Rot.from_euler('xyz', v[3:6])


def _frame_graph(model):
    """Resolve every link/joint/frame pose to the model frame.

    SDF 1.7+ poses carry `relative_to`, and these models chain through it:
    a joint is posed relative to the parent link, the child link relative to
    that joint. Reading <pose> as if it were model-relative stacks every bin
    at the origin.
    """
    raw = {}
    for link in model.findall('link'):
        raw[link.get('name')] = (pose_of(link), link.find('pose'))
    for joint in model.findall('joint'):
        child = joint.find('child')
        default = child.text.strip() if child is not None else '__model__'
        raw[joint.get('name')] = (pose_of(joint), joint.find('pose'), default)
    for frame in model.findall('frame'):
        raw[frame.get('name')] = (pose_of(frame), frame.find('pose'),
                                  frame.get('attached_to') or '__model__')

    resolved = {'__model__': (np.zeros(3), Rot.identity())}
    resolving = set()

    def resolve(name):
        if name in resolved:
            return resolved[name]
        if name in resolving:
            raise RuntimeError(f'pose cycle at frame {name!r}')
        entry = raw.get(name)
        if entry is None:
            return resolved['__model__']
        resolving.add(name)
        (xyz, rot), pose_el = entry[0], entry[1]
        default = entry[2] if len(entry) > 2 else '__model__'
        ref = (pose_el.get('relative_to') if pose_el is not None else None) or default
        p_xyz, p_rot = resolve(ref)
        out = (p_xyz + p_rot.apply(xyz), p_rot * rot)
        resolving.discard(name)
        resolved[name] = out
        return out

    for name in raw:
        resolve(name)
    return resolved


def model_points(model_name, models_dir, kinds=('visual',)):
    """Every point of the model, expressed in the model's own frame."""
    model_dir = os.path.join(models_dir, model_name)
    root = ET.parse(os.path.join(model_dir, 'model.sdf')).getroot()
    model = root.find('model')
    m_xyz, m_rot = pose_of(model)
    frames = _frame_graph(model)

    chunks, srcs = [], []
    for link in model.findall('link'):
        l_xyz, l_rot = frames[link.get('name')]
        for kind in kinds:
            for vis in link.findall(kind):
                v_xyz, v_rot = pose_of(vis)
                geom = vis.find('geometry')
                if geom is None:
                    continue
                mesh = geom.find('mesh')
                if mesh is not None:
                    pts, path = mesh_points(mesh.find('uri').text, model_dir, models_dir)
                    sc = mesh.find('scale')
                    if sc is not None:
                        pts = pts * np.array([float(x) for x in sc.text.split()])
                    srcs.append(os.path.basename(path))
                else:
                    box = geom.find('box')
                    cyl = geom.find('cylinder')
                    sph = geom.find('sphere')
                    if box is not None:
                        s = np.array([float(x) for x in box.find('size').text.split()]) / 2
                        pts = np.array([[sx * s[0], sy * s[1], sz * s[2]]
                                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
                    elif cyl is not None:
                        r = float(cyl.find('radius').text)
                        h = float(cyl.find('length').text) / 2
                        pts = np.array([[sx * r, sy * r, sz * h]
                                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
                    elif sph is not None:
                        r = float(sph.find('radius').text)
                        pts = np.array([[sx * r, sy * r, sz * r]
                                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
                    else:
                        continue
                    srcs.append(f'<{geom[0].tag}>')
                pts = pts @ v_rot.as_matrix().T + v_xyz
                pts = pts @ l_rot.as_matrix().T + l_xyz
                pts = pts @ m_rot.as_matrix().T + m_xyz
                chunks.append(pts)
    if not chunks:
        raise RuntimeError(f'{model_name}: no geometry found')
    return np.vstack(chunks), srcs

# ---------------------------------------------------------------------- limits

AXES = 'xyz'

# Pool interior. Read off the base world rather than restated here: the floor
# box's top face and the wall spans are what actually bound a prop.
def pool_bounds(world_path):
    """(min, max) of the water volume, from the base world's own geometry."""
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    floor = None
    for model in world.findall('model'):
        if model.get('name') == 'pool_floor':
            floor = model
            break
    if floor is None:
        raise RuntimeError(f'{world_path}: no pool_floor model to measure')
    f_xyz, _ = pose_of(floor)
    box = floor.find('.//collision/geometry/box/size')
    size = np.array([float(v) for v in box.text.split()])
    lo = np.array([f_xyz[0] - size[0] / 2, f_xyz[1] - size[1] / 2,
                   f_xyz[2] + size[2] / 2])
    hi = np.array([f_xyz[0] + size[0] / 2, f_xyz[1] + size[1] / 2, 0.0])
    return lo, hi


_hull_cache = {}


def hull(models_dir, uri):
    """AABB corners of a model in its own frame, visual and collision unioned.

    Only yaw is ever applied to a task, and yaw leaves Z alone while taking any
    horizontal extreme at a corner of the footprint - so the eight corners carry
    all the information the sweep needs, at a fraction of the vertex count.
    """
    name = uri.split('model://')[-1].strip('/').split('/')[0]
    if name not in _hull_cache:
        pts, _ = model_points(name, models_dir, kinds=('visual', 'collision'))
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        _hull_cache[name] = np.array(list(itertools.product(*zip(lo, hi))))
    return _hull_cache[name]


def task_envelope(models_dir, task, yaw_steps=73):
    """World AABB covering a task over its whole rotation and jitter range.

    Excludes the translation randomisation, which is a pure additive shift and
    so can be solved for afterwards.
    """
    origin = np.array(task['origin'][:3], float)
    origin_rot = Rot.from_euler('xyz', task['origin'][3:6], degrees=True)

    rot_rng = (task.get('randomize') or {}).get('rotation_deg') or [0, 0, 0]
    jit = task.get('jitter') or {}
    j_t = np.array(jit.get('translation', [0, 0, 0]), float)
    j_yaw = abs(float((jit.get('rotation_deg') or [0, 0, 0])[2]))

    def span(half, n):
        return np.linspace(-abs(half), abs(half), n) if half else [0.0]

    rolls, pitches = span(rot_rng[0], 3), span(rot_rng[1], 3)
    yaws, j_yaws = span(rot_rng[2], yaw_steps), span(j_yaw, 9)

    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for prop in task['props']:
        pts = hull(models_dir, prop['uri'])
        pose = prop.get('pose') or [0.0] * 6
        local = np.array(pose[:3], float)
        prop_rot = Rot.from_euler('xyz', pose[3:6], degrees=True)

        for r, p, y in itertools.product(rolls, pitches, yaws):
            frame = Rot.from_euler('xyz', [r, p, y], degrees=True) * origin_rot
            base = origin + frame.apply(local)
            # jitter is applied in the task frame, so the frame rotation spreads
            # its box over the world axes
            jt = np.abs(frame.as_matrix()) @ j_t
            for jy in j_yaws:
                world_rot = frame * prop_rot * Rot.from_euler('z', jy, degrees=True)
                body = pts @ world_rot.as_matrix().T
                lo = np.minimum(lo, base + body.min(axis=0) - jt)
                hi = np.maximum(hi, base + body.max(axis=0) + jt)
    return lo, hi


def compute(layout, models_dir, world_path):
    """Per-task, per-axis translation limits. Returns {task: {...}}."""
    p_lo, p_hi = pool_bounds(world_path)

    tasks = dict(layout['tasks'])
    veh = layout.get('vehicle')
    if veh:
        tasks['vehicle'] = {'origin': veh['pose'],
                            'randomize': veh.get('randomize'),
                            'props': [{'uri': veh['uri'], 'pose': [0.0] * 6}]}

    env = {n: task_envelope(models_dir, t) for n, t in tasks.items()}

    # Swim clearance is the vehicle's own footprint: two props must never be
    # closer than the AUV can fit through, whatever heading it approaches on.
    clear = np.zeros(3)
    if veh:
        v_lo, v_hi = env['vehicle']
        vs = v_hi - v_lo
        clear = np.array([float(np.hypot(vs[0], vs[1]))] * 2 + [float(vs[2])])

    budget = {n: np.full(3, np.inf) for n in tasks}
    against = {n: [''] * 3 for n in tasks}
    for a, b in itertools.combinations(tasks, 2):
        la, ha = env[a]
        lb, hb = env[b]
        gap = np.maximum(lb - ha, la - hb)
        # The vehicle swims the course, so it only has to not spawn inside a
        # prop; prop pairs must additionally leave room to pass between.
        need = np.zeros(3) if 'vehicle' in (a, b) else clear
        slack = gap - need
        k = int(np.argmax(slack))
        if slack[k] <= 0:
            continue
        for name, other in ((a, b), (b, a)):
            if slack[k] / 2 < budget[name][k]:
                budget[name][k] = slack[k] / 2
                against[name][k] = other

    out = {}
    for name in tasks:
        lo, hi = env[name]
        rows = []
        for i, ax in enumerate(AXES):
            wall = min(lo[i] - p_lo[i], p_hi[i] - hi[i])
            limit = max(0.0, min(wall, budget[name][i]))
            rows.append({
                'axis': ax, 'lo': float(lo[i]), 'hi': float(hi[i]),
                'wall': float(wall), 'task': float(budget[name][i]),
                'limit': float(limit), 'against': against[name][i],
                'reason': ('vs ' + against[name][i]
                           if budget[name][i] < wall and against[name][i]
                           else ('floor/surface' if ax == 'z' else 'pool wall')),
            })
        out[name] = rows
    return out, clear


def seat_offsets(layout, models_dir, world_path):
    """Prop pose Z that puts each task's lowest mesh point exactly on the floor."""
    p_lo, _ = pool_bounds(world_path)
    out = {}
    for name, task in layout['tasks'].items():
        for prop in task['props']:
            pts = hull(models_dir, prop['uri'])
            drop = task['origin'][2] + float(pts[:, 2].min())
            out[f'{name}/{prop["name"]}'] = {
                'mesh_min_z': float(pts[:, 2].min()),
                'pose_z_for_floor': float(p_lo[2] - task['origin'][2]
                                          - float(pts[:, 2].min())),
                'current_gap_to_floor': float(drop + (prop.get('pose') or [0] * 6)[2]
                                              - p_lo[2]),
            }
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--layout', default=None)
    parser.add_argument('--models', default=None)
    parser.add_argument('--world', default=None)
    parser.add_argument('--fraction', type=float, default=1.0,
                        help='report this fraction of each hard limit')
    parser.add_argument('--yaml', action='store_true',
                        help='emit randomize blocks ready to paste')
    args = parser.parse_args(argv)

    share = get_package_share_directory('auv_worlds')
    layout_path = args.layout or os.path.join(share, 'config',
                                              'competition_config.yaml')
    layout = yaml.safe_load(open(layout_path))
    models_dir = args.models or os.path.join(share, 'models')
    world = args.world or os.path.join(share, 'worlds',
                                       layout.get('base_world', 'robosub.sdf'))

    rows, clear = compute(layout, models_dir, world)
    f = args.fraction

    if args.yaml:
        for name, rs in rows.items():
            vals = [round(r['limit'] * f, 3) for r in rs]
            print(f'  # {name}: {f:.0%} of the geometric limit')
            print(f'  translation: [{vals[0]}, {vals[1]}, {vals[2]}]')
        return 0

    lo_hi = lambda r: f"[{r['lo']:+7.3f},{r['hi']:+7.3f}]"
    print(f'swim clearance from AUV mesh: {clear[0]:.3f} m horizontal, '
          f'{clear[2]:.3f} m vertical\n')
    print(f'{"task":9s} {"ax":3s} {"swept AABB":>19s} {"wall":>8s} {"vs task":>9s} '
          f'{"limit":>8s} {"x" + str(f):>7s}  reason')
    print('-' * 88)
    for name, rs in rows.items():
        for i, r in enumerate(rs):
            t = '    -   ' if not np.isfinite(r['task']) else f"{r['task']:8.3f}"
            print(f'{name if i == 0 else "":9s} {r["axis"]:3s} {lo_hi(r)} '
                  f'{r["wall"]:8.3f} {t} {r["limit"]:8.3f} '
                  f'{r["limit"] * f:7.3f}  {r["reason"]}')
        print()

    print('floor seating (pose Z that lands the mesh exactly on the floor):')
    for key, s in seat_offsets(layout, models_dir, world).items():
        print(f'  {key:28s} pose_z {s["pose_z_for_floor"]:+.6f}   '
              f'currently {s["current_gap_to_floor"]:+.6f} m off the floor')
    return 0


if __name__ == '__main__':
    sys.exit(main())
