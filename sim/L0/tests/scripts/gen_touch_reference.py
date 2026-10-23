#!/usr/bin/env python3
"""Generate the touch golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_touch_reference.py \\
        sim/L0/tests/assets/golden/touch

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's touch sensor adds, for each contact of its site's body in contact
order, the normal force `mj_contactForce` decodes for that contact (a
pyramidal contact's facets summed first, `mju_decodePyramid`), skipping a
normal force not above 0 and a contact whose normal's ray misses the site's
zone (`engine_sensor.c:887-930`). The cases: a free box resting on a plane
(four contacts) and a free sphere resting beside it (one), each with a touch
sensor on a site that encloses it, at condim 1, 3, 4 and 6 under the
pyramidal cone and condim 3 under the elliptic one. For each, MuJoCo's state
(qpos, qvel, qacc_warmstart, time) before steps 100 to 119, and after
mj_forward at that state: ncon, efc_force and sensordata; and one more
state, step 100's with the sphere's contact rows' forces negated and
mj_sensorAcc run again, so that contact's normal force is below 0 and the
sphere's touch reads 0. `site_sizes`: the size MuJoCo compiles for each
site of a document whose sites write their sizes in part or not at all,
over default classes (a site's size is its touch sensor's zone). `rays`:
mju_rayGeom on each type it takes (`rays` says how they are drawn), and
rays on the edges of its tests (`EDGE_RAYS`), each with MuJoCo's distance. `zones`: a sphere on a plane with touch sensors on
eleven sites of the sphere and two of the world, a box on a plane with
five, a sphere on a box on a plane with two on each, whose zones take some
of their bodies' contacts and not others (`BALL_SITES` and the rest say
where they lie), at the same states as the cases above. `zone_edges`: the
zone test at its edges, with forces and a contact's position written over
MuJoCo's (`EDGE_BALL_XML` and `EDGE_PAIR_XML` say how).
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

STEPS = range(100, 120)
CASES = (("pyramidal", 1), ("pyramidal", 3), ("pyramidal", 4), ("pyramidal", 6), ("elliptic", 3))


def xml(cone, condim):
    return f"""<mujoco model="touch_{cone}_{condim}">
  <option timestep="0.002" cone="{cone}"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1" condim="{condim}"/>
    <body name="box" pos="0 0 0.12" euler="0.05 -0.03 0.2">
      <freejoint/>
      <geom type="box" size="0.2 0.15 0.1" mass="2" condim="{condim}"/>
      <site name="zone" type="box" size="0.5 0.5 0.5"/>
    </body>
    <body name="ball" pos="0.6 0 0.06">
      <freejoint/>
      <geom name="ball_geom" type="sphere" size="0.05" mass="0.3" condim="{condim}"/>
      <site name="ballzone" type="sphere" size="0.2"/>
    </body>
  </worldbody>
  <sensor>
    <touch site="zone"/>
    <touch site="ballzone"/>
  </sensor>
</mujoco>"""


def floats(a):
    return [float(x) for x in a]


def case(cone, condim):
    text = xml(cone, condim)
    m = mujoco.MjModel.from_xml_string(text)
    d = mujoco.MjData(m)
    states = []
    for step in range(max(STEPS) + 1):
        if step in STEPS:
            st = {"step": step, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
                  "qacc_warmstart": floats(d.qacc_warmstart), "time": float(d.time)}
            mujoco.mj_forward(m, d)
            if d.ncon != 5:
                sys.exit(f"touch_{cone}_{condim} step {step}: {d.ncon} contacts")
            st.update({"ncon": int(d.ncon), "efc_force": floats(d.efc_force),
                       "sensordata": floats(d.sensordata)})
            states.append(st)
            if step == min(STEPS):
                states.append(negated(m, d, dict(st)))
        mujoco.mj_step(m, d)
    return {"xml": text, "states": states}


def negated(m, d, st):
    """`st`, after its mj_forward, with the sphere's contact rows' forces
    negated and mj_sensorAcc run again; `d` is left as it was."""
    saved = d.efc_force.copy()
    ball = m.geom("ball_geom").id
    rows = [i for i in range(d.nefc)
            if d.efc_type[i] in (mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL,
                                 mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC,
                                 mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS)
            and ball in d.contact[d.efc_id[i]].geom]
    if not rows:
        sys.exit("no rows of the sphere's contact")
    d.efc_force[rows] = -d.efc_force[rows]
    mujoco.mj_sensorAcc(m, d)
    st.update({"negated_sphere_rows": rows, "efc_force": floats(d.efc_force),
               "sensordata": floats(d.sensordata)})
    if st["sensordata"][1] != 0:
        sys.exit(f"the sphere's touch with its normal negated: {st['sensordata'][1]}")
    d.efc_force[:] = saved
    return st


# Sites whose sizes are written in part, or not at all, over default classes three deep, a
# body's childclass among them, and sites given by a segment (`fromto`, on the site, from a
# class, from a child class over its parent's, nearly along z, and in a moved frame, its own and
# its class's), and a class's position in a moved frame: the zone a touch sensor reads is its
# site's size and pose.
SITE_SIZES_XML = """<mujoco model="site_sizes">
  <default>
    <site size="0.02 0.03"/>
    <default class="a">
      <site size="0.04"/>
      <default class="b">
        <site type="box"/>
      </default>
    </default>
    <default class="c">
      <site type="ellipsoid" size="0.06 0.07 0.08"/>
    </default>
    <default class="d">
      <site type="box" size="0.03" fromto="0 0 0 0.3 0.1 0.2"/>
      <default class="e">
        <site size="0.04"/>
      </default>
      <default class="g">
        <site fromto="0 0.1 0 0 0.1 0.2"/>
      </default>
    </default>
    <default class="p">
      <site pos="0.1 0 0"/>
    </default>
  </default>
  <worldbody>
    <site name="world_plain" type="box"/>
    <body name="b0">
      <freejoint/>
      <geom size="0.1"/>
      <site name="main_class"/>
      <site name="one_written" size="0.5"/>
      <site name="class_a" class="a"/>
      <site name="class_b_two_written" class="b" size="0.1 0.2"/>
      <site name="class_c_one_written" class="c" size="0.9"/>
      <site name="all_written" type="capsule" size="0.11 0.12 0.13"/>
      <site name="fromto_capsule" type="capsule" size="0.02" fromto="0 0 0 0 0 0.4"/>
      <site name="fromto_box_class" class="d"/>
      <site name="fromto_box_nested_class" class="e"/>
      <site name="fromto_ellipsoid" type="ellipsoid" size="0.02 0.07" fromto="0.1 -0.2 0.05 -0.3 0.4 0.25"/>
      <site name="fromto_cylinder" type="cylinder" size="0.01" fromto="0.2 0.1 0 -0.1 0.3 0.4"/>
      <frame pos="0.5 -0.25 1">
        <site name="fromto_in_a_frame" type="capsule" size="0.01" fromto="0 0 0 0.1 0.2 0.3"/>
      </frame>
      <site name="fromto_child_class" class="g"/>
      <site name="fromto_near_vertical" type="box" size="0.02" fromto="0 0 0 1e-9 0 0.4"/>
      <frame pos="0.5 0 0">
        <site name="class_fromto_in_a_frame" class="d"/>
        <site name="class_pos_in_a_frame" class="p"/>
      </frame>
      <body name="b1" childclass="a">
        <joint type="hinge"/>
        <geom size="0.1"/>
        <site name="childclass_a"/>
        <site name="childclass_a_two_written" size="0.3 0.4"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""


def site_sizes():
    """Each named site's size, position and orientation as MuJoCo compiles `SITE_SIZES_XML`."""
    m = mujoco.MjModel.from_xml_string(SITE_SIZES_XML)
    return {"xml": SITE_SIZES_XML,
            "site_size": {m.site(i).name: floats(m.site_size[i]) for i in range(m.nsite)},
            "site_pos": {m.site(i).name: floats(m.site_pos[i]) for i in range(m.nsite)},
            "site_quat": {m.site(i).name: floats(m.site_quat[i]) for i in range(m.nsite)}}


# mju_rayGeom's types and their mjtGeom numbers.
RAY_TYPES = {"plane": 0, "sphere": 2, "capsule": 3, "ellipsoid": 4, "cylinder": 5, "box": 6}


def rays():
    """mju_rayGeom on each type it takes, 300 rays a type from one generator: sizes of 0.05 to
    0.5 in each place (a plane's two of 0, unbounded, in one case of five), each place of the
    centre within 1 of the origin's, a turn at random; the ray's start the centre moved by 0.3
    times a normal draw in each place, inside the geom or not; its direction at random or, in
    one case of five, along one of the geom's own axes (so that, in the geom's frame, its other
    two parts are near 0), of length 1 in half the cases and 0.1 to 3 in the rest. Each with
    MuJoCo's distance, -1 where it misses. It refuses a type with no hit or no miss."""
    rng = np.random.default_rng(150)
    out = []
    for kind, geomtype in RAY_TYPES.items():
        hits = 0
        for i in range(300):
            size = rng.uniform(0.05, 0.5, 3)
            if kind == "plane" and i % 5 == 0:
                size[:2] = 0.0
            pos = rng.uniform(-1.0, 1.0, 3)
            quat = rng.normal(size=4)
            quat /= np.linalg.norm(quat)
            mat = np.zeros(9)
            mujoco.mju_quat2Mat(mat, quat)
            pnt = pos + 0.3 * rng.normal(size=3)
            if i % 5 == 1:
                direction = mat.reshape(3, 3)[:, rng.integers(3)] * rng.choice([-1.0, 1.0])
            else:
                direction = rng.normal(size=3)
            vec = direction / np.linalg.norm(direction)
            if i % 2:
                vec = vec * rng.uniform(0.1, 3.0)
            x = mujoco.mju_rayGeom(pos, mat, size, pnt, vec, geomtype, None)
            hits += x >= 0
            out.append({"type": kind, "size": floats(size), "pos": floats(pos), "mat": floats(mat),
                        "pnt": floats(pnt), "vec": floats(vec), "x": float(x)})
        if not 0 < hits < 300:
            sys.exit(f"rays {kind}: {hits} hits of 300")
    return out + edge_rays()


# Rays on the edges of mju_rayGeom's tests, each geom unturned at the origin: (type, size,
# start, direction). The first ten in numbers a double holds exactly, but the short and the
# near-parallel directions; then a start on a surface aimed outward, rays along a plane's
# edge and a cylinder end's rim; then rays at a capsule's rim where a cap or the side is
# taken, found by a search of review round 1 (RC-2) that checked each against mju_rayGeom.
EDGE_RAYS = [
    # From a sphere's surface inward: the lower root is 0 exactly, and taken.
    ("sphere", [0.5, 0, 0], [0.5, 0, 0], [-1, 0, 0]),
    # A direction so short that ray_quad's a is below mjMINVAL: a miss, though the line crosses.
    ("sphere", [0.5, 0, 0], [-1, 0, 0], [1e-9, 0, 0]),
    # Toward an unbounded plane, but within mjMINVAL of parallel to it: a miss.
    ("plane", [0, 0, 0], [0, 0, 1], [1, 0, -1e-16]),
    # From a box's top face, across it within mjMINVAL of parallel: that face is passed over.
    ("box", [0.5, 0.5, 0.5], [0.125, 0.25, 0.5], [1, 0, 1e-16]),
    # From a cylinder's top end, across it within mjMINVAL of parallel: the end is passed over.
    ("cylinder", [0.5, 0.5, 0], [0.125, 0.25, 0.5], [1, 0, 1e-16]),
    # Along a capsule's equator where its side meets its top cap.
    ("capsule", [0.25, 0.5, 0], [-1, 0, 0.5], [1, 0, 0]),
    # Along a capsule's axis from below, and from inside its bottom cap.
    ("capsule", [0.25, 0.5, 0], [0, 0, -2], [0, 0, 1]),
    ("capsule", [0.25, 0.5, 0], [0, 0, -0.625], [0, 0, 1]),
    # A cylinder's axis from below; an ellipsoid's longest axis from inside.
    ("cylinder", [0.25, 0.5, 0], [0, 0, -2], [0, 0, 1]),
    ("ellipsoid", [0.25, 0.5, 0.125], [0, 0.25, 0], [0, 1, 0]),
    # From a sphere's surface outward; from a plane, down; down a plane's rectangle's edge.
    ("sphere", [0.5, 0, 0], [0.5, 0, 0], [1, 0, 0]),
    ("plane", [0, 0, 0], [0, 0, 0], [0, 0, -1]),
    ("plane", [0.5, 0.5, 0], [0.5, 0, 1], [0, 0, -1]),
    # From a capsule's poles outward; up a cylinder end's rim; from a cylinder's top end and a
    # box's top face outward.
    ("capsule", [0.25, 0.5, 0], [0, 0, 0.75], [0, 0, 1]),
    ("capsule", [0.25, 0.5, 0], [0, 0, -0.75], [0, 0, -1]),
    ("cylinder", [0.25, 0.5, 0], [0.25, 0, -2], [0, 0, 1]),
    ("cylinder", [0.25, 0.5, 0], [0, 0, 0.5], [0, 0, 1]),
    ("box", [0.5, 0.5, 0.5], [0, 0, 0.5], [0, 0, 1]),
    # At a capsule's rim: the top cap, the bottom cap, the side.
    ("capsule", [0.28581561264326383, 0.3854755773473887, 0],
     [-0.42133655551772764, -0.17087761266099769, 0.38228095386256383],
     [0.48948008544190075, 0.8719577026456636, 0.009950414699187995]),
    ("capsule", [0.27854130571008195, 0.12442899540082805, 0],
     [0.5149328859586134, 1.1373322999782516, 0.14069203069059408],
     [-0.3419724585521947, -0.9395643606360015, -0.016542364233362437]),
    ("capsule", [0.3904362406201048, 0.12462678309589756, 0],
     [1.2792616707214886, -0.7051566956221821, -0.00042264592356366926],
     [-0.8703267032758725, 0.4787940684926178, -0.1152721542317907]),
    ("capsule", [0.2513820758992696, 0.46759846023053947, 0],
     [0.008403447800340788, 0.33413499048511264, -0.4983238407255258],
     [-0.49941561646767807, -0.8166522198482731, 0.2892459054940916]),
    ("capsule", [0.43826415571343064, 0.4087071107807584, 0],
     [0.272670596943813, 0.9669635161305612, 0.7329665029181224],
     [0.02212801517404295, -0.4941382936941811, -0.8691016612856407]),
    ("capsule", [0.4237486395184328, 0.21958420163395212, 0],
     [-0.1626371786853277, 0.5217001065307303, -0.18437390091586311],
     [-0.5129956918839691, -0.8472047816302339, -0.13812848399716673]),
]


def edge_rays():
    out = []
    for kind, size, pnt, vec in EDGE_RAYS:
        pos, mat = np.zeros(3), np.eye(3).ravel()
        size, pnt, vec = (np.array(v, dtype=float) for v in (size, pnt, vec))
        x = mujoco.mju_rayGeom(pos, mat, size, pnt, vec, RAY_TYPES[kind], None)
        out.append({"type": kind, "size": floats(size), "pos": floats(pos), "mat": floats(mat),
                    "pnt": floats(pnt), "vec": floats(vec), "x": float(x)})
    return out


def touch_sites(sites):
    """Sites written into a body, and a touch sensor on each."""
    return ("".join(f'<site name="{n}" {a}/>' for n, a in sites),
            "".join(f'<touch site="{n}"/>' for n, _ in sites))


# The sphere's sites: the ray from its contact runs down, from the sphere toward the plane
# (`engine_sensor.c:917-920`). Enclosing it, about it, beside it and off it, turned and not;
# an ellipsoid of one size written (0.2, 0.005, 0.005: flat across the ray, 0.01 off it).
BALL_SITES = [("centre", 'type="sphere" size="0.01"'),
              ("enclosing", 'type="sphere" size="0.12"'),
              ("top", 'type="box" pos="0 0 0.1" size="0.05 0.05 0.01"'),
              ("bottom", 'type="box" pos="0 0 -0.1" size="0.05 0.05 0.01"'),
              ("below_cylinder", 'type="cylinder" pos="0 0 -0.15" size="0.02 0.01"'),
              ("below_capsule", 'type="capsule" pos="0 0 -0.2" size="0.01 0.02"'),
              ("below_ellipsoid", 'type="ellipsoid" pos="0 0 -0.3" size="0.01 0.02 0.03"'),
              ("off_ray", 'type="sphere" pos="0.05 0 -0.2" size="0.01"'),
              ("turned_box", 'type="box" pos="0 0 -0.2" euler="45 30 0" size="0.01 0.02 0.005"'),
              ("turned_capsule", 'type="capsule" pos="0.04 0 -0.2" euler="0 90 0" size="0.01 0.03"'),
              ("flat_ellipsoid", 'type="ellipsoid" pos="0 0.01 -0.12" size="0.2"'),
              ("fromto_capsule", 'type="capsule" size="0.01" fromto="0 0 -0.2 0 0 0.2"')]
# The plane's sites: its contact's ray runs up, from the plane toward the sphere.
WORLD_SITES = [("world_above", 'type="box" pos="0 0 0.05" size="0.05 0.05 0.05"'),
               ("world_below", 'type="box" pos="0 0 -0.2" size="0.05 0.05 0.05"')]
# The box's sites: its four contacts' rays run down. Under one half, one corner, one edge;
# about all four; above it.
BOX_SITES = [("half", 'type="box" pos="0.1 0 -0.15" size="0.1 0.2 0.06"'),
             ("corner", 'type="sphere" pos="0.2 0.15 -0.15" size="0.03"'),
             ("edge", 'type="capsule" pos="-0.2 0 -0.12" euler="90 0 0" size="0.02 0.2"'),
             ("all", 'type="box" size="0.3 0.3 0.3"'),
             ("above", 'type="box" pos="0 0 0.2" size="0.1 0.1 0.05"')]
# A sphere on a box on the plane: the sphere's contact with the box is listed sphere first, so
# the box's sensor turns its ray (up, into the sphere); the box's other sensor sees only the
# plane's four.
STACK_SITES_BOX = [("box_top", 'type="box" pos="0 0 0.15" size="0.05 0.05 0.05"'),
                   ("box_under", 'type="box" pos="0 0 -0.15" size="0.25 0.25 0.06"')]
STACK_SITES_BALL = [("ball_under", 'type="sphere" pos="0 0 -0.12" size="0.03"'),
                    ("ball_over", 'type="sphere" pos="0 0 0.12" size="0.03"')]


def zone_documents():
    """The zones' documents. `segments`: a sphere on a plane under fusestatic, with sites given
    by a segment along z through the contact: a class's in a frame turned to lie along x (it
    misses the ray), a class's in a frame moved down 0.1 (it holds the contact), and the site's
    own in two jointless child bodies the compiler fuses into the sphere's, one moved down 0.1
    (it holds the contact) and one moved 0.3 aside (it misses)."""
    ball, ball_sensors = touch_sites(BALL_SITES)
    world, world_sensors = touch_sites(WORLD_SITES)
    box, box_sensors = touch_sites(BOX_SITES)
    stack_box, stack_box_sensors = touch_sites(STACK_SITES_BOX)
    stack_ball, stack_ball_sensors = touch_sites(STACK_SITES_BALL)
    return {
        "segments": """<mujoco model="zones_segments">
  <compiler fusestatic="true"/>
  <option timestep="0.002"/>
  <default>
    <default class="seg">
      <site type="capsule" size="0.01" fromto="0 0 -0.2 0 0 0.2"/>
    </default>
  </default>
  <worldbody>
    <geom type="plane" size="2 2 0.1"/>
    <body pos="0 0 0.099"><freejoint/><geom type="sphere" size="0.1" mass="0.3"/>
      <frame euler="0 90 0"><site name="class_turned" class="seg"/></frame>
      <frame pos="0 0 -0.1"><site name="class_moved" class="seg"/></frame>
      <body pos="0 0 -0.1">
        <site name="fused_own" type="capsule" size="0.01" fromto="0 0 -0.1 0 0 0.1"/>
      </body>
      <body pos="0.3 0 0">
        <site name="fused_aside" type="capsule" size="0.01" fromto="0 0 -0.2 0 0 0.2"/>
      </body>
    </body>
  </worldbody>
  <sensor><touch site="class_turned"/><touch site="class_moved"/><touch site="fused_own"/>
    <touch site="fused_aside"/></sensor>
</mujoco>""",
        "ball": f"""<mujoco model="zones_ball">
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1"/>
    {world}
    <body pos="0 0 0.099"><freejoint/><geom type="sphere" size="0.1" mass="0.3"/>{ball}</body>
  </worldbody>
  <sensor>{ball_sensors}{world_sensors}</sensor>
</mujoco>""",
        "box": f"""<mujoco model="zones_box">
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1"/>
    <body pos="0 0 0.099"><freejoint/><geom type="box" size="0.2 0.15 0.1" mass="2"/>{box}</body>
  </worldbody>
  <sensor>{box_sensors}</sensor>
</mujoco>""",
        "stack": f"""<mujoco model="zones_stack">
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1"/>
    <body pos="0 0 0.099"><freejoint/><geom type="box" size="0.2 0.15 0.1" mass="2"/>{stack_box}</body>
    <body pos="0 0 0.298"><freejoint/><geom type="sphere" size="0.1" mass="0.3"/>{stack_ball}</body>
  </worldbody>
  <sensor>{stack_box_sensors}{stack_ball_sensors}</sensor>
</mujoco>"""}


def zone_case(name, text):
    """MuJoCo's state before each of `STEPS` and, after mj_forward there, its ncon, efc_force
    and sensordata. It refuses a document whose sensors do not read both 0 and above 0."""
    m = mujoco.MjModel.from_xml_string(text)
    d = mujoco.MjData(m)
    states = []
    for step in range(max(STEPS) + 1):
        if step in STEPS:
            st = {"step": step, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
                  "qacc_warmstart": floats(d.qacc_warmstart), "time": float(d.time)}
            mujoco.mj_forward(m, d)
            st.update({"ncon": int(d.ncon), "efc_force": floats(d.efc_force),
                       "sensordata": floats(d.sensordata)})
            states.append(st)
        mujoco.mj_step(m, d)
    readings = [x for st in states for x in st["sensordata"]]
    if not (any(x == 0 for x in readings) and any(x > 0 for x in readings)):
        sys.exit(f"zones {name}: the sensors read {sorted(set(readings))[:4]}")
    return {"xml": text, "states": states}


# The edges of the zone test (review round 1, RC-3, RC-12), at one state each with MuJoCo's
# own efc_force written over and the acceleration sensors rerun: a sphere on a plane under
# condim 1, with sites on the world and on the sphere, its contact's normal force set to 2 and
# its position to (0, 0, 0.5), on a sphere site's surface and a box site's top face (each ray
# meets its zone at distance 0); set to 1e-16, where mju_normalize3 takes the ray as (1, 0, 0),
# turned on the sphere's side; to 1e-15, mjMINVAL itself; and the position to (0.3, 0, 0.5), on
# the bottom of a sphere site off the contact's own line, which only that position's ray meets. And a body whose two geoms an
# explicit <pair> sets against each other, so that the sensor's body is the contact's second
# side as well as its first, and the ray is turned.
EDGE_BALL_XML = """<mujoco model="zone_edges_ball">
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1" condim="1"/>
    <site name="w_surface" type="sphere" size="0.5"/>
    <site name="w_box_face" type="box" pos="0 0 0.25" size="0.25 0.25 0.25"/>
    <site name="w_px" type="sphere" pos="0.5 0 0" size="0.1"/>
    <site name="w_pz" type="sphere" pos="0 0 0.5" size="0.1"/>
    <site name="w_off" type="sphere" pos="0.3 0 0.75" size="0.25"/>
    <body pos="0 0 0.099"><freejoint/><geom type="sphere" size="0.1" mass="0.3" condim="1"/>
      <site name="b_mx" type="sphere" pos="-0.5 0 -0.1" size="0.1"/>
      <site name="b_mz" type="sphere" pos="0 0 -0.6" size="0.1"/>
    </body>
  </worldbody>
  <sensor><touch site="w_surface"/><touch site="w_box_face"/><touch site="w_px"/>
    <touch site="w_pz"/><touch site="b_mx"/><touch site="b_mz"/><touch site="w_off"/></sensor>
</mujoco>"""
EDGE_PAIR_XML = """<mujoco model="zone_edges_pair">
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body pos="0 0 1"><freejoint/>
      <geom name="a" type="sphere" size="0.1" mass="0.3"/>
      <geom name="b" type="sphere" size="0.1" pos="0 0 0.15" mass="0.3"/>
      <site name="up" type="sphere" pos="0 0 0.4" size="0.05"/>
      <site name="down" type="sphere" pos="0 0 -0.3" size="0.05"/>
    </body>
  </worldbody>
  <contact><pair geom1="a" geom2="b"/></contact>
  <sensor><touch site="up"/><touch site="down"/></sensor>
</mujoco>"""


def edge_state(m, d, force, contact_pos=None):
    """`d`'s state, and after mj_forward there each row's force set to `force` (and the first
    contact's position to `contact_pos`) and mj_sensorAcc rerun: ncon, efc_force, sensordata."""
    st = {"step": int(round(d.time / m.opt.timestep)), "qpos": floats(d.qpos),
          "qvel": floats(d.qvel), "qacc_warmstart": floats(d.qacc_warmstart),
          "time": float(d.time)}
    mujoco.mj_forward(m, d)
    st["ncon"] = int(d.ncon)
    d.efc_force[:] = force
    if contact_pos is not None:
        d.contact.pos[0] = contact_pos
        st["contact_pos"] = [floats(d.contact.pos[i]) for i in range(d.ncon)]
    mujoco.mj_sensorAcc(m, d)
    st.update({"efc_force": floats(d.efc_force), "sensordata": floats(d.sensordata)})
    return st


def zone_edges():
    m = mujoco.MjModel.from_xml_string(EDGE_BALL_XML)
    d = mujoco.MjData(m)
    for _ in range(100):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    if d.ncon != 1 or d.nefc != 1:
        sys.exit(f"zone_edges ball: {d.ncon} contacts, {d.nefc} rows")
    ball = [edge_state(m, d, f, p) for f, p in ((2.0, [0.0, 0.0, 0.5]), (1e-16, None),
                                                 (1e-15, None), (2.0, [0.3, 0.0, 0.5]))]
    m = mujoco.MjModel.from_xml_string(EDGE_PAIR_XML)
    d = mujoco.MjData(m)
    pair = [edge_state(m, d, 2.0)]
    return {"ball": {"xml": EDGE_BALL_XML, "states": ball},
            "pair": {"xml": EDGE_PAIR_XML, "states": pair}}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {f"{cone}_{condim}": case(cone, condim) for cone, condim in CASES}
    out = {"oracle": marker, "cases": finite_or_string(cases),
           "site_sizes": finite_or_string(site_sizes()),
           "rays": finite_or_string(rays()),
           "zones": finite_or_string({n: zone_case(n, t) for n, t in zone_documents().items()}),
           "zone_edges": finite_or_string(zone_edges())}
    with open(os.path.join(sys.argv[1], "touch.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
