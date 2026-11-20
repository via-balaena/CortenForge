#!/usr/bin/env python3
"""Generate the convex hull graph golden for sim-qhull from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_qhull_graph_reference.py \\
        sim/L0/qhull/tests/data/mesh_graph_golden.txt

It refuses any other interpreter, as gen_census_golden.py does.

Each case is a set of float32 points (MuJoCo reads a mesh's `vertex` as float)
built here from a fixed seed, and a mesh's `maxhullvert`. MuJoCo loads a model
holding them as a mesh with no faces on a static geom, so its compiler makes
the mesh's graph (`mjCMesh::MakeGraph`: qhull over the vertices) and its faces
from the graph. The golden holds, per case, MuJoCo's `mesh_graph` for that
mesh, or that MuJoCo refused the model with "qhull error".

The point sets are the degenerate kinds a mesh's vertices come in: random
points in a cube, points on a sphere, points on a sphere with points inside,
a cube's corners with 1-ulp neighbours, thin slabs, a rotated lattice on a
cube's surface, cylinders, cones and capped cylinders with many segments,
axis-aligned lattices (solid and surface), a cube's corners with face centres
just inside and outside, duplicates, a tetrahedron, an octahedron, and sets
qhull refuses (coplanar, collinear, three distinct points). Some are repeated
with `maxhullvert` 4, 8 and 16. Then a few sets of three streams, each set
chosen because a measured change to the port changes its graph or its
refusal: a fuzz stream (`fuzz_sets`, FUZZ_CHOSEN), degenerate
sets in the ways a CAD or scanned mesh is degenerate; a CAD stream (`cad_sets`,
CAD_CHOSEN), parts of axis-aligned boxes, some corners cut, faces exactly
coplanar, coordinates a float32 ulp apart; and a grid stream (`grid_sets`,
GRID_CHOSEN), boxes whose faces are sampled on grids, some turned.

The file is one case per line, '#' lines aside:

    <name> <maxhullvert> <point count> <x y z as float32 bits in hex>... ; G <graph ints>...
    <name> <maxhullvert> <point count> <hex>... ; ERR

The graph is `mesh_graph` from `mesh_graphadr`: numvert, numface,
vert_edgeadr, vert_globalid, edge_localid, face_globalid.
"""

import math
import os
import struct
import sys
import tempfile

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402


def rotation(r):
    """A rotation matrix from a random unit quaternion."""
    q = r.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def fuzz_sets(seed, count):
    """The fuzz stream: `count` small float32 point sets from one generator, seeded `seed`, cycling through seven
    kinds (a polyhedron's integer faces, a lattice, duplicates and 1-ulp neighbours, an offset set, rings, a rounded
    sphere, a box's gridded faces). Yields (kind, index, points)."""
    r = np.random.default_rng(seed)

    def rot():
        return rotation(r)

    def polyhedron():
        m = r.integers(4, 12)
        pts = []
        for _ in range(m):
            n = r.integers(-3, 4, 3)
            if not n.any():
                n[0] = 1
            u = np.cross(n, [1, 0, 0] if n[0] == 0 else [0, 1, 0])
            v = np.cross(n, u)
            c = n * r.integers(1, 4)
            for _ in range(r.integers(3, 12)):
                pts.append(c + u * r.integers(-4, 5) + v * r.integers(-4, 5))
        return np.array(pts, dtype=np.float64)

    def lattice():
        k = r.integers(2, 7)
        g = r.integers(0, k, (r.integers(8, 120), 3)).astype(np.float64)
        return g if r.random() < 0.5 else g @ rot().T

    def dups():
        p = r.uniform(-1, 1, (r.integers(4, 40), 3)).astype(np.float32)
        reps = [p] + [p[r.permutation(len(p))[: r.integers(1, len(p) + 1)]] for _ in range(r.integers(1, 4))]
        q = np.vstack(reps)
        if r.random() < 0.5:
            idx = r.integers(0, len(q), r.integers(1, len(q) + 1))
            q[idx] = np.nextafter(q[idx], np.float32(2))
        return q

    def offset():
        b = polyhedron() if r.random() < 0.5 else r.uniform(-1, 1, (r.integers(5, 80), 3))
        return b * 10.0 ** r.integers(-4, 1) + 10.0 ** r.integers(1, 4)

    def ring():
        seg = int(r.integers(16, 400))
        rings = int(r.integers(2, 12))
        a = np.arange(seg) * 2 * np.pi / seg
        z = np.sort(r.uniform(-1, 1, rings))
        rad = r.uniform(0.2, 1, rings) if r.random() < 0.5 else np.ones(rings)
        p = np.vstack([np.stack([np.cos(a) * rr, np.sin(a) * rr, np.full(seg, zz)], 1) for zz, rr in zip(z, rad)])
        return p @ rot().T if r.random() < 0.5 else p

    def sphere():
        n = int(r.integers(50, 3000))
        v = r.normal(size=(n, 3))
        v /= np.linalg.norm(v, axis=1, keepdims=True)
        return np.round(v * 10.0 ** r.integers(1, 6)) / 10.0 ** r.integers(0, 3)

    def box_grid():
        nx, ny, nz = r.integers(2, 9, 3)
        s = r.uniform(0.01, 2, 3)
        g = [[i / (nx - 1), j / (ny - 1), k / (nz - 1)] for i in range(nx) for j in range(ny) for k in range(nz)
             if i in (0, nx - 1) or j in (0, ny - 1) or k in (0, nz - 1)]
        p = (np.array(g) - 0.5) * s
        if r.random() < 0.5:
            p = p @ rot().T
        return p + r.uniform(-1, 1, 3) * r.random()

    kinds = [polyhedron, lattice, dups, offset, ring, sphere, box_grid]
    for i in range(count):
        k = kinds[i % len(kinds)]
        yield k.__name__, i, np.asarray(k(), dtype=np.float32).reshape(-1, 3)


# The fuzz sets the golden takes, by index into fuzz_sets(1, ...), each chosen by a measured change to the port that
# no other case caught (RL-9d2 C1's mutants, over the stream): 616, 1009, 1345, whose graph changes when qhull's
# point-plane distance is summed in another order; 1049, 1896, 2774, which qhull refuses (a wide max_outside) when a
# facet's plane is taken from its determinants where they come out near zero, not from Gaussian elimination.
# 571 (RL-9d2 C2's mutants) changes when qhull's merge count is not kept (qh_USEfindbestnew reads it).
FUZZ_CHOSEN = (571, 616, 1009, 1049, 1345, 1896, 2774)


def cad_sets(seed, count):
    """The CAD stream: `count` float32 parts from one generator seeded `seed`: one to
    three axis-aligned boxes, some with a corner cut by a slanted plane, points along their edges and on their faces,
    scaled to metres, some turned a quarter turn about an axis; then a random share of coordinates moved by one ulp,
    zeros made 1e-10-sized, some vertices duplicated. Yields (index, points), the index counting every part made."""
    r = np.random.default_rng(seed)

    def box(lo, hi):
        return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])],
                        dtype=np.float64)

    def part():
        pts = []
        for _ in range(int(r.integers(1, 4))):
            lo = r.uniform(-1, 0.5, 3)
            hi = lo + r.uniform(0.1, 1.5, 3)
            b = box(lo, hi)
            if r.random() < 0.5:
                n = r.normal(size=3)
                n /= np.linalg.norm(n)
                c = b.mean(0) + n * r.uniform(0.2, 0.6) * (hi - lo).max()
                keep = [p for p in b if (p - c) @ n <= 0]
                for i in range(len(b)):
                    for j in range(i + 1, len(b)):
                        a, e = b[i], b[j]
                        da, de = (a - c) @ n, (e - c) @ n
                        if da * de < 0 and np.count_nonzero(a != e) == 1:
                            keep.append(a + (e - a) * (da / (da - de)))
                b = np.array(keep) if len(keep) >= 4 else b
            pts.append(b)
            for _ in range(int(r.integers(0, 6))):
                i, j = r.integers(0, len(b), 2)
                t = r.choice([0.25, 0.5, 0.75, r.random()])
                pts.append([b[i] + (b[j] - b[i]) * t])
        p = np.vstack(pts) * 10.0 ** r.integers(-2, 0) + (r.uniform(-0.1, 0.1, 3) if r.random() < 0.5 else 0)
        if r.random() < 0.3:
            k = int(r.integers(0, 3))
            p[:, [(k + 1) % 3, (k + 2) % 3]] = p[:, [(k + 2) % 3, (k + 1) % 3]] * [1, -1]
        q = p.astype(np.float32)
        m = r.random(q.shape) < r.uniform(0, 0.3)
        q[m] = np.nextafter(q[m], np.where(r.random(q.shape) < 0.5, np.float32(np.inf), np.float32(-np.inf))[m])
        z = (q == 0) & (r.random(q.shape) < 0.5)
        q[z] = np.float32(1.16415322e-10) * r.choice([-1, 1, 4, 8], size=q.shape)[z]
        if r.random() < 0.3:
            q = np.vstack([q, q[r.integers(0, len(q), int(r.integers(1, 4)))]])
        return q

    for i in range(count):
        yield i, part()


# The CAD parts the golden takes, by index into cad_sets(2, ...), each chosen by a measured change to the merges that
# no other case caught (RL-9d2 C2's mutants): 16, a vertex renamed after a merge and its ridges re-oriented; 225, the
# merged facet's ridges marked untested; 851 and 2500, the merges taken in another order, coplanar merges of
# untested new facets not put off, and a tie between two facets' distances merged the other way. And three that a C1
# mutant of the build changes, chosen when the golden missed it (the RL-9d2 review's rerun): 2374, qhull's round-off bound without its 1.01;
# 4005, a horizon facet's distance not checked before merging is skipped (`qh_checkzero`); 4083, a point's best
# facet switched on an equal distance (`qh_findbest`).
CAD_CHOSEN = (16, 225, 851, 2374, 2500, 4005, 4083)


def grid_sets(seed, count):
    """The grid stream: `count` float32 point sets from one xorshift generator seeded `seed`: one to three boxes,
    each face sampled on a k x k grid (k from 3 to 12), half of the sets turned by a random rotation; each coordinate
    rounded to float32 and a random share of them moved one ulp; the points shuffled. Yields (index, points), the
    index counting every set made."""
    mask = (1 << 64) - 1
    state = [(seed * 0x9E3779B97F4A7C15) & mask | 1]

    def bits():
        x = state[0]
        x ^= (x << 13) & mask
        x ^= x >> 7
        x ^= (x << 17) & mask
        state[0] = x
        return x

    def unit():
        return (bits() >> 11) / float(1 << 53)

    def below(lo, hi):
        return lo + bits() % (hi - lo)

    def turn():
        a, b, c = unit() * 6.283, unit() * 6.283, unit() * 6.283
        ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
        return [[ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc],
                [sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc],
                [-sb, cb * sc, cb * cc]]

    def boxes():
        p = []
        for _ in range(below(1, 4)):
            lo = [unit() - 0.5, unit() - 0.5, unit() - 0.5]
            size = [0.1 + unit(), 0.1 + unit(), 0.1 + unit()]
            k = below(3, 13)
            for ax in range(3):
                for side in range(2):
                    for a in range(k):
                        for b in range(k):
                            q = [0.0, 0.0, 0.0]
                            q[ax] = lo[ax] + side * size[ax]
                            q[(ax + 1) % 3] = lo[(ax + 1) % 3] + size[(ax + 1) % 3] * a / (k - 1)
                            q[(ax + 2) % 3] = lo[(ax + 2) % 3] + size[(ax + 2) % 3] * b / (k - 1)
                            p.append(q)
        if unit() < 0.5:
            m = turn()
            p = [[m[i][0] * q[0] + m[i][1] * q[1] + m[i][2] * q[2] for i in range(3)] for q in p]
        return p

    for i in range(count):
        p = boxes()
        share = unit() * 0.4
        out = []
        for q in p:
            row = []
            for x in q:
                f = np.float32(x)
                if unit() < share and f != 0:
                    u = int(np.array(f, dtype=np.float32).view(np.uint32))
                    u = u + 1 if unit() < 0.5 else u - 1
                    f = np.array(u, dtype=np.uint32).view(np.float32)[()]
                row.append(f)
            out.append(row)
        for j in range(len(out) - 1, 0, -1):
            k = bits() % (j + 1)
            out[j], out[k] = out[k], out[j]
        yield i, np.array(out, dtype=np.float32)


# The grid sets the golden takes, (seed, index into grid_sets(seed, ...)), each reaching a merge into the first new
# facet before the merge set is first filled, so qhull's list of new facets starts elsewhere after it (RL-9d2 review,
# RB-1): (20, 13) gives another graph and (24, 993) a refusal when the list is read before the merge.
GRID_CHOSEN = ((20, 13), (24, 993))


def cases():
    """Each case's name, maxhullvert and float32 points (n×3), in a fixed order."""
    out = []

    def add(name, pts, mhvs=(-1,)):
        p = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        for mhv in mhvs:
            out.append((name if mhv == -1 else f'{name}_mhv{mhv}', mhv, p))

    for seed in range(4):
        r = np.random.default_rng(seed)
        for n in (8, 30, 200, 1500):
            add(f'cube_s{seed}_n{n}', r.uniform(-1, 1, (n, 3)), (-1, 8) if n == 200 else (-1,))
        for n in (20, 100, 600):
            v = r.normal(size=(n, 3))
            v /= np.linalg.norm(v, axis=1, keepdims=True)
            add(f'sphere_s{seed}_n{n}', v * 0.05, (-1, 16) if n == 100 else (-1,))
        v = r.normal(size=(300, 3))
        v /= np.linalg.norm(v, axis=1, keepdims=True)
        add(f'sphin_s{seed}', np.vstack([v, r.uniform(-0.5, 0.5, (300, 3))]))
        c = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=np.float32)
        jit = np.vstack([np.nextafter(c, c * 2), np.nextafter(c, c * 0), c, c])
        add(f'cubejit_s{seed}', jit[r.permutation(len(jit))])
        for t in (1e-3, 1e-5, 1e-7):
            s = r.uniform(-1, 1, (200, 3))
            s[:, 2] *= t
            add(f'slab_s{seed}_t{t:g}', s)
        g = np.array([[i, j, k] for i in range(5) for j in range(5) for k in range(5)
                      if 0 in (i, j, k) or 4 in (i, j, k)], dtype=np.float64) / 4 - 0.5
        add(f'latsurf_rot_s{seed}', g @ rotation(r).T)
        seg = 8 + 4 * seed
        a = np.arange(seg) * 2 * np.pi / seg
        ring = np.stack([np.cos(a), np.sin(a), np.zeros(seg)], 1)
        add(f'cyl_seg{seg}', np.vstack([ring * 0.03 + [0, 0, -0.1], ring * 0.03 + [0, 0, 0.1]]), (-1, 4))
        add(f'cone_seg{seg}', np.vstack([ring * 0.03, [[0, 0, 0.07]]]))
        add(f'cylcap_seg{seg}', np.vstack([ring * 0.03 + [0, 0, -0.1], ring * 0.03 + [0, 0, 0.1],
                                           [[0, 0, -0.1]], [[0, 0, 0.1]]]))
    for m in (2, 3, 4, 6):
        g = np.array([[i, j, k] for i in range(m) for j in range(m) for k in range(m)], dtype=np.float64)
        add(f'latsolid_m{m}', g)
        add(f'latsurf_m{m}', g[[(0 in p) or (m - 1 in p) for p in g.astype(int).tolist()]])
    for e in (0, 1e-7, -1e-7, 1e-5, -1e-5):
        c = [[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
        f = [[s * (1 + e) if i == ax else 0 for i in range(3)] for ax in range(3) for s in (-1, 1)]
        add(f'cubeface_e{e:g}', c + f)
    add('dup_tet', [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]] * 3)
    add('dup_cube', [[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)] * 2)
    add('flat_xy', [[x, y, 0] for x in range(4) for y in range(4)])
    add('flat_tilted', [[x, y, 0.5 * x + 0.25 * y] for x in range(4) for y in range(4)])
    add('collinear', [[t, 2 * t, 3 * t] for t in range(8)])
    add('three_plus', [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 1, 0]])
    add('tet', [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    add('octa', [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], (-1, 4))
    for kind, i, pts in fuzz_sets(1, max(FUZZ_CHOSEN) + 1):
        if i in FUZZ_CHOSEN:
            add(f'fuzz_{kind}_{i:05d}', pts)
    for i, pts in cad_sets(2, max(CAD_CHOSEN) + 1):
        if i in CAD_CHOSEN:
            add(f'cad_{i:05d}', pts)
    for seed, index in GRID_CHOSEN:
        for i, pts in grid_sets(seed, index + 1):
            if i == index:
                add(f'grid_{seed}_{i:05d}', pts)
    return out


def graph(points, mhv, tmp):
    """MuJoCo's mesh_graph for the points as a faceless mesh, or None where it refuses with 'qhull error'."""
    vs = ' '.join(repr(float(x)) for x in points.reshape(-1))
    extra = f' maxhullvert="{mhv}"' if mhv != -1 else ''
    xml = (f'<mujoco><asset><mesh name="m" vertex="{vs}"{extra}/></asset>'
           f'<worldbody><body><geom type="mesh" mesh="m"/></body></worldbody></mujoco>')
    path = os.path.join(tmp, 'case.xml')
    with open(path, 'w') as f:
        f.write(xml)
    try:
        m = mujoco.MjModel.from_xml_path(path)
    except Exception as e:  # MuJoCo raises ValueError with its compiler's words
        if 'qhull error' in str(e):
            return None
        raise
    adr = int(m.mesh_graphadr[0])
    if adr < 0:
        sys.exit('MuJoCo loaded the mesh with no graph')
    nv, nf = int(m.mesh_graph[adr]), int(m.mesh_graph[adr + 1])
    return [int(x) for x in m.mesh_graph[adr:adr + 2 + 3 * nv + 6 * nf]]


def main():
    args = sys.argv[1:]
    if len(args) != 1:
        sys.exit(__doc__)
    marker = oracle()
    lines = [f'# MuJoCo {marker["mujoco"]} ({marker["platform"]}), mesh_graph of faceless meshes, '
             'from sim/L0/tests/scripts/gen_qhull_graph_reference.py']
    with tempfile.TemporaryDirectory() as tmp:
        for name, mhv, pts in cases():
            g = graph(pts, mhv, tmp)
            hexes = ' '.join(f'{struct.unpack("<I", struct.pack("<f", float(x)))[0]:08x}' for x in pts.reshape(-1))
            tail = 'ERR' if g is None else 'G ' + ' '.join(str(x) for x in g)
            lines.append(f'{name} {mhv} {len(pts)} {hexes} ; {tail}')
    with open(args[0], 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(len(lines) - 1, 'cases')


if __name__ == '__main__':
    main()
