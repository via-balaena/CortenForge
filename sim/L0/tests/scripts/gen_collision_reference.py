#!/usr/bin/env python3
"""Generate the collision golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_collision_reference.py \\
        sim/L0/tests/assets/golden/collision

It refuses any other interpreter, as gen_census_golden.py does.

It writes collision.json. `rest` holds primitives resting on a slab whose top
face is at z = 0, the slab a box (`box_*`) or a convex mesh of eight vertices
(`mesh_*`). MuJoCo collides these pairs through mjc_Convex, whose contact is
the midpoint of the two witness points its EPA recovers from the polytope's
closest face (`epaWitness`, engine_collision_gjk.c:1273-1287;
engine_collision_convex.c:807-812). Each case holds:

- `xml`; `z0`, the free body's start height, upright, at rest;
- `z`: the body's height after every 50th step of `nstep` (from step 49);
- `pose`: a `qpos` 1 mm (deepest point) into the slab, turned so that one
  point of the body is deepest, and `contacts`, MuJoCo's contacts after
  mj_forward there (`pos`, the frame's normal and `dist`). It refuses a pose
  with no contact.
"""

import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

# A slab 1 x 0.8 x 0.2 (unequal sides, so the mesh's principal axes are the
# coordinate axes), its top face at z = 0.
BOX_SLAB = '<geom name="slab" type="box" size="0.5 0.4 0.1" pos="0 0 -0.1"/>'
MESH_ASSET = """<asset><mesh name="slab" vertex="-0.5 -0.4 -0.1  0.5 -0.4 -0.1  0.5 0.4 -0.1  -0.5 0.4 -0.1
  -0.5 -0.4 0.1  0.5 -0.4 0.1  0.5 0.4 0.1  -0.5 0.4 0.1"
  face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7"/></asset>"""
MESH_SLAB = '<geom name="slab" type="mesh" mesh="slab" pos="0 0 -0.1"/>'

# (geom, the height of its lowest point above its centre when upright)
BODIES = {
    "ellipsoid": ('<geom type="ellipsoid" size="0.05 0.05 0.05" mass="0.5"/>', 0.05),
    "ellipsoid3": ('<geom type="ellipsoid" size="0.06 0.045 0.03" mass="0.5"/>', 0.03),
    "cylinder": ('<geom type="cylinder" size="0.05 0.05" mass="0.5"/>', 0.05),
    "box": ('<geom type="box" size="0.05 0.04 0.03" mass="0.5"/>', 0.03),
}

CASES = {
    "box_ellipsoid": ("box", "ellipsoid"),
    "box_cylinder": ("box", "cylinder"),
    "mesh_box": ("mesh", "box"),
    "mesh_cylinder": ("mesh", "cylinder"),
    "mesh_ellipsoid3": ("mesh", "ellipsoid3"),
}

NSTEP = 1000
EVERY = 50
# The turn of the static pose: about an axis off every coordinate plane, so a
# box or cylinder has one lowest point.
TURN_AXIS = np.array([1.0, 0.6, 0.2]) / np.linalg.norm([1.0, 0.6, 0.2])
TURN_ANGLE = 0.3


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def xml_of(slab, body):
    geom, _ = BODIES[body]
    asset = MESH_ASSET if slab == "mesh" else ""
    floor = MESH_SLAB if slab == "mesh" else BOX_SLAB
    return f"""<mujoco>{asset}
<worldbody>{floor}
<body name="b" pos="0.05 -0.03 0"><freejoint/>{geom}</body>
</worldbody></mujoco>"""


def lowest(m, d, body_id):
    """The lowest point of the body's geom: its support along -z."""
    g = int(np.flatnonzero(m.geom_bodyid == body_id)[0])
    size, xmat, xpos = m.geom_size[g], d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    local = xmat.T @ np.array([0.0, 0.0, -1.0])
    t = m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        p = size * np.sign(local)
    elif t == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        s = size * size * local
        p = s / np.sqrt(np.dot(local, s))
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        radial = np.array([local[0], local[1], 0.0])
        n = np.linalg.norm(radial)
        p = np.array([0.0, 0.0, size[1] * np.sign(local[2])])
        if n > 0:
            p[:2] = size[0] * radial[:2] / n
    else:
        sys.exit(f"no support for geom type {t}")
    return xpos + xmat @ p


def case(slab, body):
    xml = xml_of(slab, body)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    b = m.body("b").id
    z0 = BODIES[body][1] + 0.001
    d.qpos[2] = z0
    z = []
    for k in range(NSTEP):
        mujoco.mj_step(m, d)
        if k % EVERY == EVERY - 1:
            z.append(float(d.qpos[2]))

    # The static pose: turned, then lowered so the lowest point is 1 mm in.
    p = mujoco.MjData(m)
    quat = np.zeros(4)
    mujoco.mju_axisAngle2Quat(quat, TURN_AXIS, TURN_ANGLE)
    p.qpos[:3] = [0.05, -0.03, 1.0]
    p.qpos[3:7] = quat
    mujoco.mj_kinematics(m, p)
    p.qpos[2] -= lowest(m, p, b)[2] + 0.001
    mujoco.mj_forward(m, p)
    if p.ncon == 0:
        sys.exit(f"{slab}_{body}: no contact at the static pose")
    contacts = [{"geom": [int(c.geom1), int(c.geom2)], "pos": floats(c.pos),
                 "normal": floats(c.frame[:3]), "dist": float(c.dist)}
                for c in p.contact[:p.ncon]]
    return {"xml": xml, "z0": z0, "nstep": NSTEP, "every": EVERY, "z": z,
            "pose": floats(p.qpos), "contacts": contacts}


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    doc = {"oracle": marker,
           "rest": finite_or_string({name: case(*c) for name, c in CASES.items()})}
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], "collision.json"), doc)
    print(f"{len(CASES)} rest cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
