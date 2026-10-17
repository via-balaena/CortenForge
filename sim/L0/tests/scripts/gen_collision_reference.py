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

`order` holds MuJoCo's contact list after mj_forward (each contact's geoms
as listed, normal, `dist` and `dim`) for scenes that show its order
(engine_collision_driver.c:226-470): a body of two geoms (a box and a
sphere) against two single-sphere bodies, a sphere on a plane and a sphere
against a box, with the midphase on and off (`bodies`,
`bodies_no_midphase`), the same with four `<pair>`s (`pairs`,
`pairs_no_midphase`), and a sphere on a box (`sphere_box`). MuJoCo lists
the lower geom type first; the body pairs by signature, a pair ahead of the
broadphase's contacts at its signature; within a pair of bodies not both
single-geom, by the listed geoms under the midphase, by the first body's
geoms in turn without it. `flex` holds a 3 x 3 flex grid on a plane
between two spheres: MuJoCo lists the plane's flex contacts after its
sphere contacts (a body-flex pair's signature is past every body pair's of
the same first body), and the flex's vertex positions and elements. It
refuses scenes that do not show that.
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


# The contact list's order. World: a plane (geom 0). Body A: a box (1) and a
# sphere (2); body B, a sphere (3) above, and body C, a sphere (4) below,
# each touching both of A's geoms; body D, a sphere (5) on the plane; body E,
# a box (6) beside D.
ORDER_XML = """<mujoco><option><flag gravity="disable"{flags}/></option><worldbody>
<geom name="floor" type="plane" size="2 2 0.1"/>
<body name="A" pos="0 0 0.3"><freejoint/>
  <geom name="Abox" type="box" size="0.05 0.05 0.05" pos="-0.1 0 0"/>
  <geom name="Asph" type="sphere" size="0.05" pos="0.1 0 0"/>
</body>
<body name="B" pos="0 0 0.405"><freejoint/><geom name="Bsph" type="sphere" size="0.1"/></body>
<body name="C" pos="0 0 0.195"><freejoint/><geom name="Csph" type="sphere" size="0.1"/></body>
<body name="D" pos="0.5 0 0.045"><freejoint/><geom name="Dsph" type="sphere" size="0.05"/></body>
<body name="E" pos="0.5 0.095 0.06"><freejoint/><geom name="Ebox" type="box" size="0.05 0.05 0.05"/></body>
</worldbody>{pairs}</mujoco>"""
# Written against MuJoCo's compiled order: each pair names the geom of the
# higher body first (the compiler swaps them), out of signature order (the
# compiler sorts them); the floor pairs and B-C are outside the
# broadphase's pairs, within their margins. The A-B pair is declared sixth,
# so its index exceeds the broadphase's A-B contact's geom ids and only its
# signature's rule lists it first.
ORDER_PAIRS = """<contact><pair geom1="Ebox" geom2="Dsph" condim="1"/>
<pair geom1="Csph" geom2="floor" margin="0.2" condim="1"/><pair geom1="Csph" geom2="Bsph" margin="0.05" condim="1"/>
<pair geom1="Abox" geom2="floor" margin="0.3" condim="1"/><pair geom1="Asph" geom2="floor" margin="0.3" condim="1"/>
<pair geom1="Bsph" geom2="Asph" condim="1"/><pair geom1="Bsph" geom2="floor" margin="0.4" condim="1"/></contact>"""
# A 3 x 3 flex grid on a plane between two spheres, the flex declared
# between their bodies. The plane is on a static body: sim-core skips a
# vertex's contacts with geoms on its body's parent, the world here
# (ledger L171).
FLEX_XML = """<mujoco><option><flag gravity="disable"/></option><worldbody>
<body name="ground"><geom name="floor" type="plane" size="2 2 0.1"/></body>
<body name="ball" pos="0.8 0 0.045"><freejoint/><geom name="ball" type="sphere" size="0.05"/></body>
<flexcomp name="cloth" type="grid" count="3 3 1" spacing="0.05 0.05 0.05" pos="0 0 0.005" radius="0.01"
  dim="2" mass="0.1"><contact selfcollide="none"/></flexcomp>
<body name="ball2" pos="-0.8 0 0.045"><freejoint/><geom name="ball2" type="sphere" size="0.05"/></body>
</worldbody></mujoco>"""
SPHERE_BOX_XML = """<mujoco><option><flag gravity="disable"/></option><worldbody>
<geom name="box" type="box" size="0.2 0.2 0.05"/>
<body name="ball" pos="0.03 -0.02 0.14"><freejoint/><geom name="ball" type="sphere" size="0.1"/></body>
</worldbody></mujoco>"""


def contact_list(xml):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return {"xml": xml, "contacts": [
        {"geom": [int(c.geom[0]), int(c.geom[1])], "normal": floats(c.frame[:3]),
         "dist": float(c.dist), "dim": int(c.dim)} for c in d.contact[:d.ncon]]}


def flex_list(xml):
    """MuJoCo's list for FLEX_XML: each contact's geoms (-1 for the flex
    side), flex vertex (-1 for a geom side) and `dist`; with the flex's
    vertex positions and elements, from which a test builds the same flex in
    its own form."""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    if m.nflex != 1:
        sys.exit("order: the flex scene has no flex")
    adr, num = int(m.flex_elemadr[0]), int(m.flex_elemnum[0])
    dim = int(m.flex_dim[0])
    elem = m.flex_elem[int(m.flex_elemdataadr[0]):int(m.flex_elemdataadr[0]) + num * (dim + 1)]
    return {"xml": xml,
            "flexvert_xpos": floats(d.flexvert_xpos),
            "flex_elem": [int(e) for e in elem],
            "contacts": [{"geom": [int(c.geom[0]), int(c.geom[1])],
                          "vert": [int(c.vert[0]), int(c.vert[1])],
                          "dist": float(c.dist)} for c in d.contact[:d.ncon]]}


def order_cases():
    off = ' midphase="disable"'
    cases = {
        "bodies": contact_list(ORDER_XML.format(flags="", pairs="")),
        "bodies_no_midphase": contact_list(ORDER_XML.format(flags=off, pairs="")),
        "pairs": contact_list(ORDER_XML.format(flags="", pairs=ORDER_PAIRS)),
        "pairs_no_midphase": contact_list(ORDER_XML.format(flags=off, pairs=ORDER_PAIRS)),
        "sphere_box": contact_list(SPHERE_BOX_XML),
        "flex": flex_list(FLEX_XML),
    }
    # The fixture must show what it is for: a multi-geom body pair whose
    # midphase order differs from the index order, and a pair contact placed
    # by its signature ahead of a broadphase one.
    mid = [tuple(c["geom"]) for c in cases["bodies"]["contacts"]]
    flat = [tuple(c["geom"]) for c in cases["bodies_no_midphase"]["contacts"]]
    if mid == flat:
        sys.exit("order: midphase does not reorder the multi-geom body pairs")
    listed = [tuple(c["geom"]) for c in cases["pairs"]["contacts"]]
    if listed.index((2, 3)) > listed.index((3, 1)):
        sys.exit("order: the A-B pair is not listed ahead of the broadphase's A-B contact")
    if cases["sphere_box"]["contacts"][0]["geom"] != [1, 0]:
        sys.exit("order: MuJoCo does not list the sphere first")
    flex = [c["vert"][1] >= 0 for c in cases["flex"]["contacts"]]
    if not (any(flex) and flex == sorted(flex) and not flex[0]):
        sys.exit("order: the flex contacts do not follow the rigid ones")
    return cases


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    doc = {"oracle": marker,
           "rest": finite_or_string({name: case(*c) for name, c in CASES.items()}),
           "order": finite_or_string(order_cases())}
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], "collision.json"), doc)
    print(f"{len(CASES)} rest cases, {len(doc['order'])} order cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
