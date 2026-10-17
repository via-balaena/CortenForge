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
as listed, normal, tangents, `dist` and `dim`) for scenes that show its order
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

`frame` holds MuJoCo's contact frames after mj_forward (all nine entries,
the normal first, then the two tangents: `mju_makeFrame` from
`mj_setContact`, engine_util_spatial.c:508-534) for two spheres along x,
two spheres along a diagonal whose normal's y is past 0.5, and a
horizontal capsule on a plane, whose frame takes the capsule's axis as its
first tangent (mjc_PlaneCapsule's hint).

`inclusion` holds, for scenes at the edge of the margin and the gap, MuJoCo's
contacts after mj_forward (geoms, `dist`, `includemargin`, `exclude`, `pos`)
and `nefc`: a contact is listed at `dist <= margin` and makes rows only below
`margin - gap`, and none with no dofs (engine_collision_driver.c:1414-1415,
engine_core_constraint.c:996-1019). `box_plane_tumble` holds census document
830708de's box under the census's first excitation for 100 steps: `ncon`,
`qpos` and `qvel` after each, the box landing corner by corner (mjc_PlaneBox).
`adhesion` holds an adhesion actuator's moment, from a box with corners
active, in the gap and past the margin, under each cone (mjTRN_BODY,
engine_core_smooth.c:1621-1731). `touching_spheres` holds census document
31833131's two touching spheres after 7 steps, 1.1e-16 into each other, and
MuJoCo's contact count from that state at geom margins 0, 1e-9 and 1e-6:
its broadphase pairs them only with the margin. `bounds` holds a pyramid
mesh, its apex 0.01 into a plane, and two height fields: MuJoCo's bounding
radii, each about its geom's origin (mjCGeom::GetRBound,
user_objects.cc:3418-3454), and its contacts after mj_forward (geoms,
`dist`, `pos`). The apex is past the half-diagonal of the pyramid's box
from its centroid, within the bounding radius; it refuses a scene that is
not. `plane_mesh` holds a pyramid mesh whose centre of mass is its origin
(so MuJoCo keeps its frame), its base 0.001 into a plane, the plane in the
world body and in a static body after the pyramid's (so the mesh is the
pair's first geom as it is tested): MuJoCo's bounding radius and its
contacts (geoms, `dist`, `pos`). `addplanemesh` skips a vertex
nearer the first contact than 0.3 of the bounding radius
(engine_collision_convex.c:1018), which takes two of the base's four
corners; it refuses a scene where the half-diagonal of the mesh's box would
take the same.
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
         "tangents": floats(c.frame[3:]), "dist": float(c.dist), "dim": int(c.dim)}
        for c in d.contact[:d.ncon]]}


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


# Contact frames (mju_makeFrame from mj_setContact): two spheres along x
# (the default tangent (0,1,0)); two spheres along a diagonal with |n_y| >=
# 0.5 (the default (0,0,1)); and a horizontal capsule on a plane (its axis,
# the hint mjc_PlaneCapsule sets).
FRAME_XML = {
    "sphere_sphere_x": """<mujoco><option><flag gravity="disable"/></option><worldbody>
<body pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
<body pos="0.19 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
</worldbody></mujoco>""",
    "sphere_sphere_diagonal": """<mujoco><option><flag gravity="disable"/></option><worldbody>
<body pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
<body pos="0.03 0.15 1.11"><freejoint/><geom type="sphere" size="0.1"/></body>
</worldbody></mujoco>""",
    "plane_capsule": """<mujoco><option><flag gravity="disable"/></option><worldbody>
<geom type="plane" size="2 2 0.1"/>
<body pos="0 0 0.095" euler="0 0 30"><freejoint/><geom type="capsule" size="0.1 0.3" euler="0 90 0"/></body>
</worldbody></mujoco>""",
}


def frame_cases():
    out = {}
    for name, xml in FRAME_XML.items():
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        if d.ncon == 0:
            sys.exit(f"frame: {name} has no contact")
        out[name] = {"xml": xml, "contacts": [
            {"geom": [int(c.geom[0]), int(c.geom[1])], "frame": floats(c.frame),
             "dist": float(c.dist)} for c in d.contact[:d.ncon]]}
    return out


# Which listed contacts get rows (mj_setContact's `exclude`,
# engine_collision_driver.c:1414-1415; mj_instantiateContact,
# engine_core_constraint.c:996-1019): a sphere, an upright cylinder and a box
# resting on a plane at distance 0; a box tilted 10 degrees within a plane's
# margin 0.1 and gap 0.08, two of its corners in the gap; an explicit pair
# of two static spheres (nv 0).
INCLUSION_XML = {
    "sphere_touching_plane": """<mujoco><worldbody><geom type="plane" size="2 2 0.1"/>
<body pos="0 0 0.1"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>""",
    "cylinder_touching_plane": """<mujoco><worldbody><geom type="plane" size="2 2 0.1"/>
<body pos="0 0 0.1"><freejoint/><geom type="cylinder" size="0.05 0.1"/></body></worldbody></mujoco>""",
    "box_touching_plane": """<mujoco><worldbody><geom type="plane" size="2 2 0.1"/>
<body pos="0 0 0.1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body></worldbody></mujoco>""",
    "box_plane_gap": """<mujoco><worldbody><geom type="plane" size="2 2 0.1" margin="0.1" gap="0.08"/>
<body pos="0 0 0.12" euler="10 0 0"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body></worldbody></mujoco>""",
    "static_pair": """<mujoco><worldbody><geom name="a" type="sphere" size="0.1"/>
<body pos="0.15 0 0"><geom name="b" type="sphere" size="0.1"/></body></worldbody>
<contact><pair geom1="a" geom2="b"/></contact></mujoco>""",
}


def inclusion_cases():
    out = {}
    for name, xml in INCLUSION_XML.items():
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        out[name] = {"xml": xml, "nefc": int(d.nefc), "contacts": [
            {"geom": [int(c.geom[0]), int(c.geom[1])], "dist": float(c.dist),
             "includemargin": float(c.includemargin), "exclude": int(c.exclude),
             "pos": floats(c.pos)} for c in d.contact[:d.ncon]]}
    for name in ("sphere_touching_plane", "cylinder_touching_plane", "box_touching_plane"):
        if out[name]["nefc"] != 0 or not out[name]["contacts"] or any(
                c["exclude"] != 1 for c in out[name]["contacts"]):
            sys.exit(f"inclusion: {name}'s contacts are not all listed and excluded")
    if sorted(c["exclude"] for c in out["box_plane_gap"]["contacts"]) != [0, 0, 1, 1]:
        sys.exit("inclusion: the box does not have two corners in the gap")
    if out["static_pair"]["nefc"] != 0:
        sys.exit("inclusion: the static pair has rows")
    return out


# Census document 830708de (collision_plane.rs's `box_plane_stable`): a box
# dropped on a plane under the census's first excitation, qvel_i =
# 0.1 (1 + i mod 3), so it lands on one corner first (mjc_PlaneBox lists the
# corners within the margin, each by its own distance,
# engine_collision_primitive.c:196-239).
BOX_PLANE_TUMBLE_XML = """<mujoco model="box_plane_stable">
<option gravity="0 0 -9.81" solver="Newton" cone="elliptic" iterations="100" tolerance="1e-8"/>
<worldbody><geom type="plane" size="5 5 0.1"/>
<body name="box" pos="0 0 0.11"><joint type="free"/>
<geom type="box" size="0.1 0.1 0.1" mass="1.0" friction="0.5 0.5 0.005"/></body>
</worldbody></mujoco>"""


def box_plane_tumble():
    m = mujoco.MjModel.from_xml_string(BOX_PLANE_TUMBLE_XML)
    d = mujoco.MjData(m)
    for i in range(m.nv):
        d.qvel[i] = 0.1 * (1 + i % 3)
    rows = []
    for _ in range(100):
        mujoco.mj_step(m, d)
        rows.append({"qpos": floats(d.qpos), "qvel": floats(d.qvel), "ncon": int(d.ncon)})
    if not any(0 < r["ncon"] < 4 for r in rows):
        sys.exit("box_plane_tumble: no step lists some corners and not others")
    return {"xml": BOX_PLANE_TUMBLE_XML, "rows": rows}


# An adhesion actuator on a tilted box within a plane's margin 0.04 and gap
# 0.02: three corners listed, two with rows and one in the gap, the fourth
# past the margin. MuJoCo's body transmission averages the active contacts'
# rows (the normal row, or the pyramid's rows at 0.5/(dim-1) each) and the
# excluded contacts' normal Jacobians, negated (mjTRN_BODY,
# engine_core_smooth.c:1621-1731).
ADHESION_XML = """<mujoco><option cone="{cone}"/><worldbody><geom type="plane" size="2 2 0.1" margin="0.04" gap="0.02"/>
<body name="box" pos="0 0 0.08" euler="12 7 0"><freejoint/><geom type="box" size="0.1 0.08 0.06"/></body></worldbody>
<actuator><adhesion body="box" ctrlrange="0 1" gain="10"/></actuator></mujoco>"""


def adhesion_cases():
    out = {}
    for cone in ("pyramidal", "elliptic"):
        xml = ADHESION_XML.format(cone=cone)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        moment = np.zeros(m.nv)
        adr, nnz = d.moment_rowadr[0], d.moment_rownnz[0]
        for k in range(nnz):
            moment[d.moment_colind[adr + k]] = d.actuator_moment[adr + k]
        excluded = [int(c.exclude) for c in d.contact[:d.ncon]]
        if sorted(excluded) != [0, 0, 1]:
            sys.exit(f"adhesion {cone}: not two active corners and one in the gap: {excluded}")
        out[cone] = {"xml": xml, "ncon": int(d.ncon), "nefc": int(d.nefc), "moment": floats(moment)}
    return out


# Census document 31833131 (collision_primitives.rs's `sphere_sphere_touch`):
# two touching free spheres moving together under the census's first
# excitation, 1.1e-16 into each other after 7 steps. MuJoCo's broadphase
# sorts the bodies' bounds along its first axis cast to float, so bounds
# that meet there pair the bodies only with a margin (mj_SAP,
# engine_collision_driver.c:1021-1114).
TOUCHING_SPHERES_XML = """<mujoco model="sphere_sphere_touch">
<option gravity="0 0 0" timestep="0.001"/>
<worldbody>
<body name="s1" pos="0 0 0"><joint type="free"/><geom type="sphere" size="0.5"/></body>
<body name="s2" pos="0.8 0 0"><joint type="free"/><geom type="sphere" size="0.3"/></body>
</worldbody></mujoco>"""


def touching_spheres():
    m = mujoco.MjModel.from_xml_string(TOUCHING_SPHERES_XML)
    d = mujoco.MjData(m)
    for i in range(m.nv):
        d.qvel[i] = 0.1 * (1 + i % 3)
    for _ in range(7):
        mujoco.mj_step(m, d)
    gap = float(np.linalg.norm(d.geom_xpos[1] - d.geom_xpos[0]) - 0.8)
    if not gap < 0:
        sys.exit(f"touching_spheres: the spheres do not overlap ({gap})")
    ncon = {}
    for margin in (0.0, 1e-9, 1e-6):
        mm = mujoco.MjModel.from_xml_string(TOUCHING_SPHERES_XML)
        mm.geom_margin[:] = margin
        dd = mujoco.MjData(mm)
        dd.qpos[:] = d.qpos
        dd.qvel[:] = d.qvel
        mujoco.mj_forward(mm, dd)
        ncon[repr(margin)] = int(dd.ncon)
    if ncon["0.0"] != 0 or ncon["1e-06"] != 1:
        sys.exit(f"touching_spheres: MuJoCo's contacts {ncon}")
    return {"xml": TOUCHING_SPHERES_XML, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
            "overlap": -gap, "ncon": ncon}


# A pyramid mesh, its apex 0.01 into a plane, and two height fields, one
# reaching farther above its origin than below and one farther below. A geom's
# bounding radius is about its origin (mjCGeom::GetRBound,
# user_objects.cc:3418-3454): for a mesh, the norm of each axis's larger
# bound in magnitude, about the centroid where MuJoCo puts the geom's
# origin; for a height field, sqrt(x^2 + y^2 + max(top^2, base^2)) from its
# size. The apex is 4.5 below the centroid and the base 1.5 above, so the
# half-diagonal of the pyramid's box, about the box's centre, does not
# reach the apex; MuJoCo's plane filter passes the pair (mj_filterSphere,
# engine_collision_driver.c:139-156).
BOUNDS_XML = """<mujoco><asset>
<mesh name="pyramid" vertex="-1 -0.5 0  1 -0.5 0  1 0.5 0  -1 0.5 0  0 0 -6"
      face="0 1 2  0 2 3  0 4 1  1 4 2  2 4 3  3 4 0"/>
<hfield name="field" nrow="2" ncol="2" size="1 0.5 0.3 0.1" elevation="0 0.5 1 0.25"/>
<hfield name="deep" nrow="2" ncol="2" size="0.5 1 0.1 0.3" elevation="0 0.5 1 0.25"/>
</asset><worldbody><geom name="floor" type="plane" size="20 20 0.1"/>
<geom name="field" type="hfield" hfield="field" pos="10 10 0"/>
<geom name="deep" type="hfield" hfield="deep" pos="-10 -10 0"/>
<body name="pyramid" pos="0 0 5.99"><freejoint/><geom name="pyramid" type="mesh" mesh="pyramid"/></body>
</worldbody></mujoco>"""


def bounds():
    m = mujoco.MjModel.from_xml_string(BOUNDS_XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    p = m.geom("pyramid").id
    half = float(np.linalg.norm(m.geom_aabb[p][3:]))
    height = float(d.geom_xpos[p][2])
    if not half < height < m.geom_rbound[p]:
        sys.exit(f"bounds: the centroid's height {height} is not between the box's "
                 f"half-diagonal {half} and the bounding radius {m.geom_rbound[p]}")
    contacts = [{"geom": [int(c.geom[0]), int(c.geom[1])], "dist": float(c.dist),
                 "pos": floats(c.pos)} for c in d.contact[:d.ncon]]
    if not any(c["geom"] == [0, p] for c in contacts):
        sys.exit("bounds: MuJoCo lists no contact between the plane and the pyramid")
    return {"xml": BOUNDS_XML, "rbound": floats(m.geom_rbound), "half_diagonal": half,
            "height": height, "contacts": contacts}


PLANE_MESH_XML = """<mujoco><asset>
<mesh name="pyr" vertex="-1 -1 -2.5  1 -1 -2.5  1 1 -2.5  -1 1 -2.5  0 0 7.5"
      face="0 2 1  0 3 2  0 1 4  1 2 4  2 3 4  3 0 4"/>
</asset><worldbody><geom name="floor" type="plane" size="20 20 0.1"/>
<body name="p" pos="0 0 2.499"><freejoint/><geom name="pyr" type="mesh" mesh="pyr"/></body>
</worldbody></mujoco>"""
# The same with the plane in a static body after the pyramid's, so the mesh is
# the pair's first geom as it is tested.
PLANE_MESH_LATER_XML = PLANE_MESH_XML.replace(
    '<geom name="floor" type="plane" size="20 20 0.1"/>\n', "", 1).replace(
    "</worldbody>", '<body name="ground"><geom name="floor" type="plane" size="20 20 0.1"/></body>\n</worldbody>', 1)


def plane_mesh():
    out = {}
    for name, xml in (("world", PLANE_MESH_XML), ("later_body", PLANE_MESH_LATER_XML)):
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        p = m.geom("pyr").id
        half = float(np.linalg.norm(m.geom_aabb[p][3:]))
        # The base's corners are 2 apart along an edge, 2*sqrt(2) across.
        if not 0.3 * half < 2 < 0.3 * m.geom_rbound[p] < 2 * np.sqrt(2):
            sys.exit(f"plane_mesh {name}: half-diagonal {half}, bounding radius {m.geom_rbound[p]}")
        contacts = [{"geom": [int(c.geom[0]), int(c.geom[1])], "dist": float(c.dist),
                     "pos": floats(c.pos)} for c in d.contact[:d.ncon]]
        if len(contacts) != 2:
            sys.exit(f"plane_mesh {name}: {len(contacts)} contacts")
        out[name] = {"xml": xml, "mesh_geom": p, "rbound": float(m.geom_rbound[p]),
                     "half_diagonal": half, "contacts": contacts}
    if out["later_body"]["mesh_geom"] != 0:
        sys.exit("plane_mesh later_body: the mesh is not geom 0")
    return out


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
           "order": finite_or_string(order_cases()),
           "frame": finite_or_string(frame_cases()),
           "inclusion": finite_or_string(inclusion_cases()),
           "box_plane_tumble": finite_or_string(box_plane_tumble()),
           "adhesion": finite_or_string(adhesion_cases()),
           "touching_spheres": finite_or_string(touching_spheres()),
           "bounds": finite_or_string(bounds()),
           "plane_mesh": finite_or_string(plane_mesh())}
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], "collision.json"), doc)
    print(f"{len(CASES)} rest cases, {len(doc['order'])} order cases, "
          f"{len(doc['frame'])} frame cases, {len(doc['inclusion'])} inclusion cases, "
          f"{len(doc['adhesion'])} adhesion cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
