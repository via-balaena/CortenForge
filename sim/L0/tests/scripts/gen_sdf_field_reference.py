#!/usr/bin/env python3
"""Generate sim-core's signed-distance-field golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I -B sim/L0/tests/scripts/gen_sdf_field_reference.py \\
        sim/L0/core/tests/data/sdf_fields.txt

It refuses any other interpreter, as gen_census_golden.py does. It calls MuJoCo's own compiled functions: `mju_Halton`,
`mju_mat2Quat`, `mju_negPose`, `mju_mulPose` and `mju_quat2Mat` (in the order `mapPose` calls them,
engine_collision_sdf.c:535-542) through the bindings, and `mjc_distance` and `mjc_gradient` (engine_collision_sdf.c:
433-517, exported) through ctypes on the library the bindings loaded, with an `mjSDF` built here, for one geom of
each primitive type `mjc_SDF` takes (a sphere, a capsule, an ellipsoid, a cylinder, a box) alone (`mjSDFTYPE_SINGLE`,
`geomDistance` and `geomGradient`) and for pairs of them under each other combination (intersection, midsurface,
collision), the second turned and moved by `relmat` and `relpos`.

Each line, doubles as their bits in hex, '#' lines aside:

    halton INDEX BASE VALUE
    mat2quat M0..M8 Q0..Q3
    mappose POS1(3) QUAT1(4) POS2(3) QUAT2(4) POS12(3) MAT12(9)
    geom ID TYPE SIZE(3)
    single ID X(3) DIST GRAD(3)
    field KIND ID0 ID1 RELPOS(3) RELMAT(9) X(3) DIST GRAD(3)

TYPE is `mjtGeom`'s name; a `single` line is one geom's field alone, a `field` line two geoms' under KIND
(`intersection`, `midsurface` or `collision`); ID, ID0 and ID1 name `geom` lines (the first field's and the second's).

The points: random ones within twice each geom's size, each axis, face, edge and corner of its box, the origin, a
point on the capsule's and the cylinder's axis, and points with a coordinate -0.0 or two coordinates of equal
magnitude (the box's `radialField3d` and its gradient's ties), from a seeded generator.
"""
import ctypes
import math
import os
import random
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402
from gen_sdf_reference import MjSDF, hx, library  # noqa: E402

GEOMS = [
    ("sphere", (0.3, 0, 0)),
    ("capsule", (0.2, 0.5, 0)),
    ("ellipsoid", (0.4, 0.25, 0.15)),
    ("cylinder", (0.3, 0.2, 0)),
    ("box", (0.3, 0.2, 0.1)),
    ("box", (0.25, 0.25, 0.25)),
    ("ellipsoid", (0.2, 0.2, 0.2)),
]
TYPES = {"sphere": 2, "capsule": 3, "ellipsoid": 4, "cylinder": 5, "box": 6}
KINDS = {"single": 0, "intersection": 1, "midsurface": 2, "collision": 3}


def unit_quat(rng):
    q = [rng.gauss(0, 1) for _ in range(4)]
    n = math.sqrt(sum(x * x for x in q))
    return [x / n for x in q]


def points(size, rng):
    """Points around a geom of half sizes `size` (the box over it)."""
    h = [s if s > 0 else size[0] for s in size]
    if size[1] == 0:
        h = [size[0], size[0], size[0]]
    pts = [[0.0, 0.0, 0.0], [-0.0, 0.0, -0.0]]
    for sx in (-1, 0, 1):
        for sy in (-1, 0, 1):
            for sz in (-1, 0, 1):
                if sx or sy or sz:
                    pts.append([sx * h[0], sy * h[1], sz * h[2]])
                    pts.append([1.3 * sx * h[0], 0.7 * sy * h[1], 1.1 * sz * h[2]])
    pts += [[0, 0, 0.9 * h[2]], [0, 0, 1.7 * h[2]], [0.05, -0.0, 0.2], [-0.0, 0.1, -0.0]]
    pts += [[0.1, 0.1, 0.05], [-0.07, 0.07, 0.07], [0.12, -0.12, 0.3], [h[0] / 2, h[0] / 2, h[0] / 2]]
    for _ in range(12):
        pts.append([rng.uniform(-2 * h[k], 2 * h[k]) for k in range(3)])
    return pts


def main():
    oracle()
    if len(sys.argv) != 2:
        sys.exit("usage: gen_sdf_field_reference.py <out.txt>")
    rng = random.Random(9042)
    lines = ["# sim-core's signed-distance-field golden: MuJoCo 3.5.0's compiled functions",
             "# (written by sim/L0/tests/scripts/gen_sdf_field_reference.py; its docstring says how)."]

    for base in (2, 3, 5):
        for i in range(70):
            lines.append(f"halton {i} {base} {hx(mujoco.mju_Halton(i, base))}")

    quats = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [0.5, 0.5, 0.5, 0.5],
             [0.7071067811865476, 0.7071067811865476, 0, 0], [0, 0.7071067811865476, 0.7071067811865476, 0],
             [0.1, 0.9, 0.3, -0.2], [0.05, 0.2, -0.95, 0.1], [0.02, -0.1, 0.15, 0.98]]
    quats += [unit_quat(rng) for _ in range(30)]
    for q in quats:
        q = np.array(q, dtype=float)
        mujoco.mju_normalize4(q)
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, q)
        res = np.zeros(4)
        mujoco.mju_mat2Quat(res, mat)
        lines.append("mat2quat " + " ".join(hx(x) for x in mat) + " " + " ".join(hx(x) for x in res))

    for _ in range(30):
        pos1, pos2 = [np.array([rng.uniform(-1, 1) for _ in range(3)]) for _ in range(2)]
        q1, q2 = np.array(unit_quat(rng)), np.array(unit_quat(rng))
        if rng.random() < 0.2:
            q1 = np.array([1.0, 0, 0, 0])
        negpos, negquat, pos12, quat12, mat12 = np.zeros(3), np.zeros(4), np.zeros(3), np.zeros(4), np.zeros(9)
        mujoco.mju_negPose(negpos, negquat, pos2, q2)
        mujoco.mju_mulPose(pos12, quat12, negpos, negquat, pos1, q1)
        mujoco.mju_quat2Mat(mat12, quat12)
        lines.append("mappose " + " ".join(hx(x) for x in [*pos1, *q1, *pos2, *q2, *pos12, *mat12]))

    xml = "<mujoco><worldbody>" + "".join(
        f'<geom type="{t}" size="{" ".join(repr(float(s)) for s in sz if s)}"/>' for t, sz in GEOMS) \
        + "</worldbody></mujoco>"
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    lib = library()
    for g, (t, _) in enumerate(GEOMS):
        assert m.geom_type[g] == TYPES[t]
        lines.append(f"geom {g} {t} " + " ".join(hx(x) for x in m.geom_size[g]))

    dbl3 = ctypes.c_double * 3

    def evaluate(kind, ids, relpos, relmat, x):
        idarr = (ctypes.c_int * 2)(*ids)
        types = (ctypes.c_int * 2)(*[int(m.geom_type[i]) for i in ids])
        plugins = (ctypes.c_void_p * 2)(None, None)
        rp = (ctypes.c_double * 3)(*relpos)
        rm = (ctypes.c_double * 9)(*relmat)
        s = MjSDF(plugins, idarr, KINDS[kind], rp, rm, types)
        dist = lib.mjc_distance(m._address, d._address, ctypes.byref(s), dbl3(*x))
        grad = dbl3()
        lib.mjc_gradient(m._address, d._address, ctypes.byref(s), grad, dbl3(*x))
        return dist, list(grad)

    ident = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    for g, (_, size) in enumerate(GEOMS):
        for x in points(size, rng):
            dist, grad = evaluate("single", (g, g), [0, 0, 0], ident, x)
            lines.append(f"single {g} " + " ".join(hx(v) for v in [*x, dist, *grad]))
    for kind in ("intersection", "midsurface", "collision"):
        for _ in range(5):
            g0, g1 = rng.randrange(len(GEOMS)), rng.randrange(len(GEOMS))
            q = np.array(unit_quat(rng))
            relmat = np.zeros(9)
            mujoco.mju_quat2Mat(relmat, q)
            relpos = [rng.uniform(-0.3, 0.3) for _ in range(3)]
            for x in points(GEOMS[g0][1], rng)[::3]:
                dist, grad = evaluate(kind, (g0, g1), relpos, list(relmat), x)
                lines.append(f"field {kind} {g0} {g1} " + " ".join(hx(v) for v in [*relpos, *relmat, *x, dist, *grad]))

    with open(sys.argv[1], "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"{len(lines)} lines -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
