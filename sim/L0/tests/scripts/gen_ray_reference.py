#!/usr/bin/env python3
"""Generate the ray golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_ray_reference.py \\
        sim/L0/tests/assets/golden/ray

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes ray.json:

- `hfield_fields`: each height field as MuJoCo compiles it (`hfield_data`: its
  elevations normalised to [0, 1], rows from -y, in single precision), with its
  size (half-extents, top, base), nrow and ncol;
- `hfield`: rays against those fields through `mj_rayHfield` (`engine_ray.c`),
  each the field's pose (`geom_xpos`, `geom_xmat`, set on the data before the
  call), the ray's start and direction, and MuJoCo's distance, -1 where it
  misses. `hfield_rays` says how they are drawn; `HFIELD_EDGE_RAYS` adds rays
  on the edges of its tests; `LINE_FIELDS` adds fields of one row and of one
  column, drawn as the others; `hfield_side_rays` adds rays across each side
  at its edge's height, to within a few ulps, and refuses a set of them that
  the side test's single-precision reading decides none of;
- `mesh_models`: each mesh in a model of one mesh geom on the world, as MuJoCo
  compiles it: the model's text, the mesh's vertices (`mesh_vert`, single
  precision) and faces, the geom's size, whether MuJoCo kept the mesh in its
  written frame (`authored_frame`: the geom unturned at the origin and the
  vertices as written), and the model's hierarchies (`nbvhstatic`,
  `body_bvhnum`, `mesh_bvhadr`, `mesh_bvhnum`, `bvh_*`);
- `mesh`: rays against those meshes through `mj_rayMesh`, as `hfield`'s.
  `mesh_rays` says how they are drawn; `MESH_EDGE_RAYS` adds rays on the edges
  of its tests.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def elevation(rng, nrow, ncol, kind):
    if kind == 'flat':
        return np.full(nrow * ncol, 0.3)
    if kind == 'bumpy':
        r, c = np.meshgrid(np.arange(nrow), np.arange(ncol), indexing='ij')
        return (np.sin(1.3 * c) * np.cos(0.9 * r) + 0.2 * rng.normal(size=(nrow, ncol))).ravel()
    return rng.uniform(-1.0, 2.0, nrow * ncol)


# Each field: (name, nrow, ncol, size, elevations). Square and other cells, the
# fewest rows and columns MuJoCo's ray reads (2), a flat one (its elevations all
# 0 once normalised) and a tall narrow one, whose sides a ray crosses often.
FIELDS = [
    ('bumpy', 9, 9, [1.0, 1.0, 0.5, 0.1], 'bumpy'),
    ('cells', 5, 7, [0.75, 0.4, 0.3, 0.2], 'random'),
    ('two', 2, 2, [0.5, 0.25, 0.4, 0.1], 'random'),
    ('flat', 4, 3, [0.6, 0.5, 0.2, 0.3], 'flat'),
    ('tall', 3, 4, [0.2, 0.15, 1.0, 0.05], 'random'),
]


def field_model(name, nrow, ncol, size, elev, pad=False):
    """A model of one height-field geom on the world. With `pad`, a second field follows the
    first in `hfield_data`, so that MuJoCo's side test, which on a field of one row or column
    reads one row or column past the data (and multiplies it by 0), reads a number there."""
    after = ('<hfield name="pad" nrow="2" ncol="2" size="1 1 1 1" elevation="0 1 0.5 0.25"/>'
             if pad else '')
    xml = (f'<mujoco><asset><hfield name="{name}" nrow="{nrow}" ncol="{ncol}" '
           f'size="{" ".join(repr(s) for s in size)}" '
           f'elevation="{" ".join(repr(float(e)) for e in elev)}"/>{after}</asset>'
           f'<worldbody><geom type="hfield" hfield="{name}"/></worldbody></mujoco>')
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def turn(rng):
    quat = rng.normal(size=4)
    quat /= np.linalg.norm(quat)
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, quat)
    return mat


def cast(m, d, pos, mat, pnt, vec):
    d.geom_xpos[0] = pos
    d.geom_xmat[0] = mat
    normal = np.zeros(3)
    x = mujoco.mj_rayHfield(m, d, 0, pnt, vec, normal)
    return float(x), normal


def hfield_rays(name, m, d, size):
    """400 rays a field from one generator: the field unturned at the origin in even cases,
    else turned at random and moved by up to 0.5 in each place. The start, in the field's
    frame, by case of five: above it (up to 1 over its top, over its rectangle grown by a
    fifth) aimed down at random; below its base aimed up; beside it (past one side, between
    its base and top) aimed in; inside its top box at random; above it along its own -z
    exactly, or across it along its own x or y. The direction of length 1 in half the cases,
    0.1 to 3 in the rest. It refuses a field whose rays never hit its triangles, its base
    box, or, but for the flat field, its top box's sides above the base."""
    rng = np.random.default_rng(267 + sum(map(ord, name)))
    sx, sy, top, base = size
    out = []
    kinds = {'triangle': 0, 'side': 0, 'base': 0, 'miss': 0}
    for i in range(400):
        if i % 2 == 0:
            pos, mat = np.zeros(3), np.eye(3).ravel()
        else:
            pos, mat = rng.uniform(-0.5, 0.5, 3), turn(rng)
        case = i % 5
        if case == 0:
            lp = [rng.uniform(-1.2, 1.2) * sx, rng.uniform(-1.2, 1.2) * sy, top + rng.uniform(0, 1)]
            lv = rng.normal(size=3)
            lv[2] = -abs(lv[2]) - 0.3
        elif case == 1:
            lp = [rng.uniform(-1.2, 1.2) * sx, rng.uniform(-1.2, 1.2) * sy, -base - rng.uniform(0.05, 1)]
            lv = rng.normal(size=3)
            lv[2] = abs(lv[2]) + 0.3
        elif case == 2:
            axis, sign = rng.integers(2), rng.choice([-1.0, 1.0])
            lp = [rng.uniform(-1, 1) * sx, rng.uniform(-1, 1) * sy, rng.uniform(-base, top)]
            lp[axis] = sign * ((sx, sy)[axis] + rng.uniform(0.05, 1))
            lv = rng.normal(size=3)
            lv[axis] = -sign * (abs(lv[axis]) + 0.3)
        elif case == 3:
            lp = [rng.uniform(-1, 1) * sx, rng.uniform(-1, 1) * sy, rng.uniform(0, top)]
            lv = rng.normal(size=3)
        else:
            lp = [rng.uniform(-1, 1) * sx, rng.uniform(-1, 1) * sy, top + rng.uniform(0, 1)]
            lv = [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]][rng.integers(3)]
            if lv[2] == 0.0:
                lp[2] = rng.uniform(0, top)
                lp[0 if lv[0] else 1] = -2.0 * (sx if lv[0] else -sy)
        lv = np.array(lv, dtype=float)
        lv /= np.linalg.norm(lv)
        if i % 4 >= 2:
            lv = lv * rng.uniform(0.1, 3.0)
        r = np.array(mat).reshape(3, 3)
        pnt = pos + r @ np.array(lp, dtype=float)
        vec = r @ lv
        x, normal = cast(m, d, pos, mat, pnt, vec)
        if x < 0:
            kinds['miss'] += 1
        else:
            hit_z = float(lp[2] + x * lv[2])
            local = r.T @ normal
            if hit_z <= 0:
                kinds['base'] += 1
            elif local[2] == 0 and sorted(abs(local)) == [0.0, 0.0, 1.0]:
                kinds['side'] += 1
            else:
                kinds['triangle'] += 1
        out.append({'field': name, 'pos': floats(pos), 'mat': floats(mat), 'pnt': floats(pnt),
                    'vec': floats(vec), 'x': x})
    line = name in ('row', 'column')
    wanted = (['base', 'miss'] + ([] if line else ['triangle'])
              + ([] if name == 'flat' else ['side']))
    if any(kinds[k] == 0 for k in wanted):
        sys.exit(f'hfield rays {name}: {kinds}')
    print(f'hfield {name}: {kinds}', file=sys.stderr)
    return out


# Rays on the edges of mj_rayHfield's tests, the field unturned at the origin: (field, start,
# direction), in numbers a double holds exactly but the short direction's. On `bumpy` (cells 0.25 wide): straight down
# onto a grid point, the middle of a cell's diagonal, a cell's edge, the field's edge, and
# past it; across the field along x and along y, and along its top; up from under its base,
# and from inside the base; down from on its top box's top face; a direction so short that
# the boxes' bounding spheres miss; along (1, 1, -1), so that the basis's first branch takes
# a tie. On `two`: down its one cell's diagonal and across its two sides. Then one from very
# far.
HFIELD_EDGE_RAYS = [
    ('bumpy', [0.25, 0.5, 2.0], [0.0, 0.0, -1.0]),
    ('bumpy', [0.125, 0.125, 2.0], [0.0, 0.0, -1.0]),
    ('bumpy', [0.25, 0.125, 2.0], [0.0, 0.0, -1.0]),
    ('bumpy', [1.0, 0.375, 2.0], [0.0, 0.0, -1.0]),
    ('bumpy', [1.5, 0.0, 2.0], [0.0, 0.0, -1.0]),
    ('bumpy', [-2.0, 0.375, 0.25], [1.0, 0.0, 0.0]),
    ('bumpy', [0.375, -2.0, 0.125], [0.0, 1.0, 0.0]),
    ('bumpy', [-2.0, 0.0, 0.5], [1.0, 0.0, 0.0]),
    ('bumpy', [0.375, 0.375, -1.0], [0.0, 0.0, 1.0]),
    ('bumpy', [0.375, 0.375, -0.0625], [0.0, 0.0, 1.0]),
    ('bumpy', [0.375, 0.375, 0.5], [0.0, 0.0, -1.0]),
    ('bumpy', [0.375, 0.375, 2.0], [0.0, 0.0, -1e-9]),
    ('bumpy', [-1.0, -1.0, 1.0], [1.0, 1.0, -1.0]),
    ('two', [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]),
    ('two', [-1.0, -0.25, 0.25], [1.0, 0.0, 0.0]),
    ('two', [0.0, -1.0, 0.125], [0.0, 1.0, 0.0]),
    # From 2^64 away along a diagonal: the cells under the path run past what an integer
    # holds, and the cell more each way past that.
    ('bumpy', [-2.0 ** 64, 2.0 ** 64, 0.25], [1.0, -1.0, 0.0]),
]


def box(hx, hy, hz, n=1):
    """A box of half-sizes hx, hy, hz, each face an n by n grid of squares, two triangles
    each, wound outward."""
    verts, index, faces = [], {}, []

    def vertex(p):
        key = tuple(p)
        if key not in index:
            index[key] = len(verts)
            verts.append(p)
        return index[key]

    h = [hx, hy, hz]
    for axis in range(3):
        u, w = [a for a in range(3) if a != axis]
        for sign in (-1.0, 1.0):
            for i in range(n):
                for j in range(n):
                    def corner(a, b):
                        p = [0.0, 0.0, 0.0]
                        p[axis] = sign * h[axis]
                        p[u] = -h[u] + 2 * h[u] * a / n
                        p[w] = -h[w] + 2 * h[w] * b / n
                        return vertex(p)
                    c00, c10, c11, c01 = corner(i, j), corner(i + 1, j), corner(i + 1, j + 1), corner(i, j + 1)
                    # (u, w, axis) is right-handed for axis 0 and 2, left-handed for 1.
                    if (sign > 0) != (axis == 1):
                        faces += [(c00, c10, c11), (c00, c11, c01)]
                    else:
                        faces += [(c00, c11, c10), (c00, c01, c11)]
    return verts, faces


def octahedron(a, b, c):
    verts = [[a, 0, 0], [-a, 0, 0], [0, b, 0], [0, -b, 0], [0, 0, c], [0, 0, -c]]
    faces = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)]
    return verts, faces


def star(rng):
    """An icosahedron's vertices pushed out to radii from 0.3 to 0.8, its faces kept: a
    mesh that is not convex."""
    t = (1 + 5 ** 0.5) / 2
    base = [[-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0], [0, -1, t], [0, 1, t],
            [0, -1, -t], [0, 1, -t], [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1]]
    verts = [list(np.array(v) / np.linalg.norm(v) * rng.uniform(0.3, 0.8)) for v in base]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
             (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8),
             (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    return verts, faces


SLAB = ([[-0.5, -0.4, -0.1], [0.5, -0.4, -0.1], [0.5, 0.4, -0.1], [-0.5, 0.4, -0.1],
         [-0.5, -0.4, 0.1], [0.5, -0.4, 0.1], [0.5, 0.4, 0.1], [-0.5, 0.4, 0.1]],
        [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
         (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)])


def meshes(rng):
    """Each mesh: (name, vertices, faces or None for MuJoCo's convex hull). The first three
    MuJoCo keeps in their written frame (their principal axes along x, y, z, the largest
    moment about x, centred, in numbers a float holds); the slab (the rangefinder golden's)
    MuJoCo turns; the hull of 20 points it turns and moves; the star is not convex."""
    return [('box', *box(0.125, 0.375, 0.5)),
            ('box4', *box(0.125, 0.375, 0.5, 4)),
            ('octahedron', *octahedron(0.25, 0.5, 0.75)),
            ('slab', *SLAB),
            ('hull', [list(rng.uniform(-0.5, 0.5, 3)) for _ in range(20)], None),
            ('star', *star(rng))]


def mesh_model(name, verts, faces):
    text = ' '.join(repr(float(x)) for p in verts for x in p)
    face = '' if faces is None else ' face="' + ' '.join(str(i) for f in faces for i in f) + '"'
    xml = (f'<mujoco><asset><mesh name="{name}" vertex="{text}"{face}/></asset>'
           f'<worldbody><geom type="mesh" mesh="{name}"/></worldbody></mujoco>')
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    written = np.array(verts, dtype=np.float32)
    authored = bool(np.array_equal(m.geom_quat[0], [1, 0, 0, 0]) and not m.geom_pos[0].any()
                    and m.mesh_vert.shape == written.shape and np.array_equal(m.mesh_vert, written))
    model = {'xml': xml, 'vert': floats(m.mesh_vert), 'face': [int(i) for i in m.mesh_face.ravel()],
             'size': floats(m.geom_size[0]), 'authored_frame': authored,
             'nbvhstatic': int(m.nbvhstatic), 'body_bvhnum': [int(x) for x in m.body_bvhnum],
             'mesh_bvhadr': [int(x) for x in m.mesh_bvhadr], 'mesh_bvhnum': [int(x) for x in m.mesh_bvhnum],
             'bvh_aabb': [floats(b) for b in m.bvh_aabb], 'bvh_child': [[int(c) for c in b] for b in m.bvh_child],
             'bvh_nodeid': [int(x) for x in m.bvh_nodeid], 'bvh_depth': [int(x) for x in m.bvh_depth]}
    return m, d, model


def mesh_rays(name, m, d):
    """300 rays a mesh from one generator: the mesh unturned at the origin in even cases,
    else turned at random and moved by up to 0.5 in each place; the start within 1.5 of the
    mesh's centre in each place, aimed at a point within the mesh's bounding radius of its
    centre (four cases of five) or at random; in one case of five the direction one of the
    mesh's own axes, the start moved back along it. The direction of length 1 in half the
    cases, 0.1 to 3 in the rest. It refuses a mesh whose rays never hit or never miss."""
    rng = np.random.default_rng(2671 + sum(map(ord, name)))
    rbound = float(m.geom_rbound[0])
    out, hits = [], 0
    for i in range(300):
        if i % 2 == 0:
            pos, mat = np.zeros(3), np.eye(3).ravel()
        else:
            pos, mat = rng.uniform(-0.5, 0.5, 3), turn(rng)
        r = np.array(mat).reshape(3, 3)
        lp = rng.uniform(-1.5, 1.5, 3)
        if i % 5 == 4:
            lv = np.zeros(3)
            lv[rng.integers(3)] = rng.choice([-1.0, 1.0])
            lp = rng.uniform(-0.5, 0.5, 3) * rbound - 2.0 * lv
        elif i % 5 == 3:
            lv = rng.normal(size=3)
        else:
            lv = rng.uniform(-1, 1, 3) * rbound - lp
        lv = lv / np.linalg.norm(lv)
        if i % 4 >= 2:
            lv = lv * rng.uniform(0.1, 3.0)
        pnt, vec = pos + r @ lp, r @ lv
        d.geom_xpos[0], d.geom_xmat[0] = pos, mat
        x = float(mujoco.mj_rayMesh(m, d, 0, pnt, vec, None))
        hits += x >= 0
        out.append({'mesh': name, 'pos': floats(pos), 'mat': floats(mat), 'pnt': floats(pnt),
                    'vec': floats(vec), 'x': x})
    if not 0 < hits < 300:
        sys.exit(f'mesh rays {name}: {hits} hits of 300')
    print(f'mesh {name}: {hits} hits of 300', file=sys.stderr)
    return out


# Rays on the edges of mj_rayMesh's tests, the mesh unturned at the origin: (mesh, start,
# direction), in numbers a double holds exactly. On `box` (half-sizes 0.125, 0.375, 0.5):
# down its z axis onto a face's diagonal; down onto a corner, an edge and a face's middle;
# across its top face within it, and along its top face's plane from outside; from inside
# it; from on a face, outward and inward; past it by a hair; a direction so short that the
# bounding box test misses. On `box4`: down onto a grid point of its top face. On
# `octahedron`: down onto its apex, and along an edge's line.
MESH_EDGE_RAYS = [
    ('box', [0.0, 0.0, 2.0], [0.0, 0.0, -1.0]),
    ('box', [0.125, 0.375, 2.0], [0.0, 0.0, -1.0]),
    ('box', [0.125, 0.0, 2.0], [0.0, 0.0, -1.0]),
    ('box', [0.0625, 0.1875, 2.0], [0.0, 0.0, -1.0]),
    ('box', [-1.0, 0.0, 0.25], [1.0, 0.0, 0.0]),
    ('box', [-1.0, 0.0, 0.5], [1.0, 0.0, 0.0]),
    ('box', [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    ('box', [0.0, 0.0, 0.5], [0.0, 0.0, 1.0]),
    ('box', [0.0, 0.0, 0.5], [0.0, 0.0, -1.0]),
    ('box', [0.125000001, 0.0, 2.0], [0.0, 0.0, -1.0]),
    ('box', [0.0, 0.0, 2.0], [0.0, 0.0, -1e-9]),
    ('box4', [0.0625, 0.1875, 2.0], [0.0, 0.0, -1.0]),
    ('octahedron', [0.0, 0.0, 2.0], [0.0, 0.0, -1.0]),
    ('octahedron', [-0.25, -0.5, 0.0], [1.0, 2.0, 0.0]),
]


# Fields of one row and of one column, each followed by a padding field (`field_model`). MuJoCo
# compiles them; with one row the y spacing is infinite (one column: x), so a ray's place
# along that axis is 0 and the side test's second elevation is multiplied by 0.
LINE_FIELDS = [
    ('row', 1, 4, [1.0, 0.5, 0.7, 0.5], 'random'),
    ('column', 4, 1, [0.5, 1.0, 0.6, 0.25], 'random'),
]


def field_record(m):
    """The first field as MuJoCo compiles it: its size, rows, columns and elevations."""
    nrow, ncol, adr = int(m.hfield_nrow[0]), int(m.hfield_ncol[0]), int(m.hfield_adr[0])
    return {'size': floats(m.hfield_size[0]), 'nrow': nrow, 'ncol': ncol,
            'data': floats(m.hfield_data[adr:adr + nrow * ncol])}


def hfield_side_rays(name, m, d):
    """Rays across each of a field's four sides, unturned at the origin, level (along the
    side's normal), at the height of the side's edge where they cross it and 1 and 2 ulps
    either way: 6 crossings a side from one generator, each at a place along the side drawn
    at random. The edge's height is MuJoCo's: the elevations either side of the crossing
    (single precision) interpolated, times the field's top. Where the start's height over the
    top falls between that edge and the same edge read from the elevations scaled and
    unscaled in double precision, the side test's single-precision reading decides the ray;
    it returns the rays and how many of them that reading decides."""
    rng = np.random.default_rng(2673 + sum(map(ord, name)))
    rec = field_record(m)
    sx, sy, top, _ = rec['size']
    nrow, ncol = rec['nrow'], rec['ncol']
    data = np.array(rec['data'])
    with np.errstate(divide='ignore'):
        dx = np.float64(2.0 * sx) / np.float64(ncol - 1)
        dy = np.float64(2.0 * sy) / np.float64(nrow - 1)
    out, decided = [], 0
    for i in range(4):
        for _ in range(6):
            t = rng.uniform(-0.95, 0.95)
            if i < 2:
                along, step, count = t * sy, dy, nrow
                y = (along + sy) / dy
                index = lambda k: int(k) * ncol + (ncol - 1 if i == 1 else 0)
            else:
                along, step, count = t * sx, dx, ncol
                y = (along + sx) / dx
                index = lambda k: int(k) + ((nrow - 1) * ncol if i == 3 else 0)
            y0 = max(0.0, min(float(count - 2), np.floor(y)))
            z0 = data[index(np.round(y0))]
            z1 = data[index(np.round(y0 + 1))] if count > 1 else 0.0
            edge = z0 * (y0 + 1 - y) + z1 * (y - y0)
            scaled0 = (z0 * top) / top
            scaled1 = (z1 * top) / top if count > 1 else 0.0
            edge_double = scaled0 * (y0 + 1 - y) + scaled1 * (y - y0)
            height = edge * top
            for k in (-2, -1, 0, 1, 2):
                h = height
                for _ in range(abs(k)):
                    h = np.nextafter(h, np.inf if k > 0 else -np.inf)
                z = h / top
                decided += (z < edge) != (z < edge_double)
                pnt = [0.0, 0.0, float(h)]
                vec = [0.0, 0.0, 0.0]
                axis, sign = (0, -1.0 if i == 0 else 1.0) if i < 2 else (1, -1.0 if i == 2 else 1.0)
                pnt[axis] = sign * (2.0 * (sx if axis == 0 else sy))
                pnt[1 - axis] = float(along)
                vec[axis] = -sign
                x, _ = cast(m, d, np.zeros(3), np.eye(3).ravel(), np.array(pnt), np.array(vec))
                out.append({'field': name, 'pos': [0.0] * 3, 'mat': floats(np.eye(3)), 'pnt': pnt,
                            'vec': vec, 'x': x})
    print(f'hfield side rays {name}: {len(out)}, {decided} decided by the single-precision reading',
          file=sys.stderr)
    return out, decided


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    rng = np.random.default_rng(2670)
    fields, models, rays = {}, {}, []
    for name, nrow, ncol, size, kind in FIELDS:
        m, d = field_model(name, nrow, ncol, size, elevation(rng, nrow, ncol, kind))
        assert (m.hfield_nrow[0], m.hfield_ncol[0]) == (nrow, ncol)
        fields[name] = field_record(m)
        models[name] = (m, d)
        rays += hfield_rays(name, m, d, size)
    for name, pnt, vec in HFIELD_EDGE_RAYS:
        m, d = models[name]
        pos, mat = np.zeros(3), np.eye(3).ravel()
        x, _ = cast(m, d, pos, mat, np.array(pnt), np.array(vec))
        rays.append({'field': name, 'pos': floats(pos), 'mat': floats(mat), 'pnt': pnt,
                     'vec': vec, 'x': x})
    for name, nrow, ncol, size, kind in LINE_FIELDS:
        m, d = field_model(name, nrow, ncol, size, elevation(rng, nrow, ncol, kind), pad=True)
        assert (m.hfield_nrow[0], m.hfield_ncol[0]) == (nrow, ncol)
        fields[name] = field_record(m)
        models[name] = (m, d)
        rays += hfield_rays(name, m, d, size)
    decided = 0
    for name in ('cells', 'two', 'row', 'column'):
        m, d = models[name]
        side, count = hfield_side_rays(name, m, d)
        rays += side
        decided += count
    if decided == 0:
        sys.exit('hfield side rays: none decided by the single-precision reading')
    mesh_models, mesh_ray_list, compiled = {}, [], {}
    for name, verts, faces in meshes(np.random.default_rng(2672)):
        m, d, model = mesh_model(name, verts, faces)
        mesh_models[name], compiled[name] = model, (m, d)
        mesh_ray_list += mesh_rays(name, m, d)
    for name, pnt, vec in MESH_EDGE_RAYS:
        m, d = compiled[name]
        d.geom_xpos[0], d.geom_xmat[0] = np.zeros(3), np.eye(3).ravel()
        x = float(mujoco.mj_rayMesh(m, d, 0, np.array(pnt), np.array(vec), None))
        mesh_ray_list.append({'mesh': name, 'pos': [0.0] * 3, 'mat': floats(np.eye(3)), 'pnt': pnt,
                              'vec': vec, 'x': x})
    authored = [k for k, v in mesh_models.items() if v['authored_frame']]
    if authored != ['box', 'box4', 'octahedron']:
        sys.exit(f'meshes MuJoCo kept in their written frame: {authored}')
    os.makedirs(sys.argv[1], exist_ok=True)
    doc = {'oracle': marker, 'hfield_fields': fields, 'hfield': rays, 'mesh_models': mesh_models,
           'mesh': mesh_ray_list}
    with open(os.path.join(sys.argv[1], 'ray.json'), 'w', encoding='utf-8') as f:
        json.dump(finite_or_string(doc), f, separators=(',', ':'))
        f.write('\n')
    print(f'{len(rays)} hfield rays, {len(mesh_ray_list)} mesh rays -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
