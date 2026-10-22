#!/usr/bin/env python3
"""Generate the geom-against-flex collision golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_rigid_reference.py \\
        sim/L0/tests/assets/golden/flex

It refuses any other interpreter, as gen_census_golden.py does, and writes
flex_rigid.json.

A body's geoms against a flex (engine_collision_driver.c:264-482): the
broadphase pairs a body with a flex whose box over its vertices, grown by
its radius and margin, overlaps the body's (mj_broadphase, :1154-1299, the
world body and a world-welded body holding a plane with every flex); the
midphase descends the body's and the flex's hierarchies together
(mj_collideTree, :622-847), a world-welded body's planes first against
every vertex (mj_collidePlaneFlex, :1658-1710); without it each geom
against every element; each geom against an element by type
(mj_collideGeomElem, :1866-1998). Each case: rigid geoms against a flex,
most after a cable that collides with nothing, so the flex's vertices and
elements start past 0; a case may end with a fragment after the flexes (an
actuator). Each vertex is a body with three slides and mass
0.125, so the document is written twice: MuJoCo's <flex body=...> over
declared bodies, and sim-mjcf's <deformable><flex> with its vertices and
the flex's whole mass, whose bodies sim-mjcf adds after the others, in the
same order; equality constraints disabled on both, as sim-mjcf makes a
flex's edges equality constraints. A rigid body's mass and inertia are
given and its quat is MuJoCo's compiled one, on both. Recorded: qpos0, each
control (0.5, when the model has any), and after mj_forward each contact
(sides, distance, position, frame, dim, includemargin, friction, solref,
solimp, exclude, efc_address), the constraint rows (efc_J at the dense
size, each row's columns and values at the sparse one; efc_R, efc_D),
qacc, qfrc_constraint, qfrc_spring and qfrc_passive; then qpos, qvel and ncon at checkpoints of a
short run.
"""
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import floats, ints, oracle  # noqa: E402

MASS = 0.125
CHECKPOINTS = (1, 2, 5, 10, 20)


def grid(n, spacing, z):
    verts = [[(i - (n - 1) / 2) * spacing, (j - (n - 1) / 2) * spacing, z] for j in range(n) for i in range(n)]
    elems = []
    for j in range(n - 1):
        for i in range(n - 1):
            a, b, c, d = j * n + i, j * n + i + 1, (j + 1) * n + i, (j + 1) * n + i + 1
            elems += [a, b, c, b, d, c]
    return verts, elems, 2


# A flex: vertices, element data, dim, and its <contact> attributes.
DUMMY = ([[-0.9, -0.9, 0.3], [-0.8, -0.9, 0.3], [-0.7, -0.9, 0.3]], [0, 1, 1, 2], 1,
         'contype="0" conaffinity="0"')
NONE = 'selfcollide="none" internal="false"'
CLOTH = (*grid(3, 0.1, 0.0), NONE)
CABLE = ([[-0.1, 0.0, 0.0], [0.0, 0.0, 0.0], [0.1, 0.02, 0.01]], [0, 1, 1, 2], 1, NONE)
TET = ([[-0.1, -0.1, -0.05], [0.12, -0.08, -0.04], [-0.02, 0.13, -0.06], [0.0, 0.0, 0.08]],
       [0, 1, 2, 3], 3, NONE)
CUBE = ([[(i & 1) * 0.1, ((i >> 1) & 1) * 0.1, ((i >> 2) & 1) * 0.1] for i in range(8)],
        [0, 1, 2, 4, 3, 2, 1, 7, 5, 1, 4, 7, 6, 4, 2, 7, 1, 2, 4, 7], 3, NONE)
PLANE_BITS = ('<geom type="plane" size="2 2 0.1" contype="2" conaffinity="2"/>'
              '<geom type="box" size="0.05 0.05 0.01" pos="0.06 0 0"/>')
# Elevations exact in float32, rows symmetric and the highest 1: sim-mjcf
# loads them as MuJoCo does (book Rigid-loading L45 makes its data MuJoCo's
# otherwise: float32, rows bottom to top, normalised).
FIELD = ('<hfield name="hf" nrow="5" ncol="6" size="0.3 0.25 0.08 0.05" '
         'elevation="0 0.25 0.5 0.25 0 0  0.25 0.5 0.75 0.5 0.25 0  0.25 0.75 1 0.75 0.5 0.25  '
         '0.25 0.5 0.75 0.5 0.25 0  0 0.25 0.5 0.25 0 0"/>')


# A rigid body's mass and inertia are given, its frame its own: sim-mjcf's
# geom volumes and principal axes round otherwise than MuJoCo's (ledger L198),
# which the golden does not measure.
INERTIAL = '<inertial pos="0 0 0" mass="0.1" diaginertia="2e-5 3e-5 4e-5"/>'


def body(pos, geoms, quat="1 0 0 0"):
    return f'<body name="b" pos="{pos}" quat="{quat}"><freejoint/>{INERTIAL}{geoms}</body>'


def settled(rigid, flex, option, flag, asset, tail=""):
    """The rigid text with body b's quat MuJoCo's compiled one, which sim-mjcf
    keeps as it is: MuJoCo's normalisation of another quat can differ from
    sim-mjcf's in the last bit."""
    _, _, m = documents(rigid, flex, option, flag, asset, tail)
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "b")
    if b < 0:
        return rigid
    quat = " ".join(repr(float(x)) for x in m.body_quat[b])
    head, rest = rigid.split(' quat="', 1)
    return f'{head} quat="{quat}"{rest[rest.index(chr(34)) + 1:]}'


SPHERE = body("0.025 0.025 0.035", '<geom type="sphere" size="0.03"/>')
# Each case: rigid worldbody text, the flex, options, flags, assets.
CASES = {
    "sphere_cloth": (SPHERE, CLOTH, "", "", ""),
    "sphere_cloth_no_midphase": (SPHERE, CLOTH, "", 'midphase="disable"', ""),
    "sphere_cloth_sparse": (SPHERE, CLOTH, 'jacobian="sparse"', "", ""),
    "sphere_cloth_diagonal": (body("0.05 0.05 0.035", '<geom type="sphere" size="0.03"/>'), CLOTH, "", "", ""),
    # No cable: the broadphase's axes are the cloth's and the sphere's, and the
    # sphere's box clears the cloth's vertices by less than the flex's radius.
    "sphere_cloth_alone": (SPHERE, [CLOTH], "", "", ""),
    "box_cloth": (body("0.03 0.02 0.025", '<geom type="box" size="0.03 0.02 0.02" condim="4"/>',
                       "0.98 0.1 0.15 0.05"), CLOTH, "", "", ""),
    "capsule_cloth": (body("0.01 0.03 0.025", '<geom type="capsule" size="0.02 0.04"/>', "0.7071 0.7071 0 0"),
                      CLOTH, "", "", ""),
    "ellipsoid_cloth_condim6": (body("0.02 0.01 0.015", '<geom type="ellipsoid" size="0.04 0.03 0.02" condim="6"/>'),
                                CLOTH, "", "", ""),
    "cylinder_cloth": (body("0.02 0.02 0.025", '<geom type="cylinder" size="0.03 0.02"/>', "0.95 0.2 0.2 0.1"),
                       CLOTH, "", "", ""),
    "sphere_cable": (body("0.0 0.01 0.035", '<geom type="sphere" size="0.03" condim="4"/>'), CABLE, "", "", ""),
    "capsule_cable": (body("0.02 0.01 0.03", '<geom type="capsule" size="0.02 0.03"/>', "0.7071 0 0.7071 0"),
                      CABLE, "", "", ""),
    "box_cable": (body("-0.03 0.0 0.025", '<geom type="box" size="0.03 0.02 0.02"/>', "0.98 0.05 0.1 0.15"),
                  CABLE, "", "", ""),
    "ellipsoid_cable": (body("0.0 0.01 0.025", '<geom type="ellipsoid" size="0.04 0.03 0.02"/>'), CABLE, "", "", ""),
    "sphere_tet": (body("0.02 0.02 0.12", '<geom type="sphere" size="0.05"/>'), TET, "", "", ""),
    "box_tet": (body("0.0 0.01 0.11", '<geom type="box" size="0.03 0.02 0.02"/>', "0.5 0.5 0.5 0.5"),
                TET, "", "", ""),
    "two_geoms_cloth": (body("0.03 0.02 0.035", '<geom type="sphere" size="0.03"/>'
                             '<geom type="box" size="0.02 0.02 0.01" pos="-0.06 0.03 -0.025"/>'), CLOTH, "", "", ""),
    "two_geoms_cloth_no_midphase": (body("0.03 0.02 0.035", '<geom type="sphere" size="0.03"/>'
                                         '<geom type="box" size="0.02 0.02 0.01" pos="-0.06 0.03 -0.025"/>'),
                                    CLOTH, "", 'midphase="disable"', ""),
    # The sphere's bits miss the flex's; the box meets it by the flex's contype only.
    "bitmask_cloth": (body("0.03 0.02 0.035", '<geom type="sphere" size="0.03" contype="2" conaffinity="2"/>'
                           '<geom type="box" size="0.02 0.02 0.01" pos="-0.06 0.03 -0.025" contype="0" '
                           'conaffinity="1"/>'), CLOTH, "", "", ""),
    "params_cloth": (body("0.025 0.025 0.04", '<geom type="sphere" size="0.03" margin="0.004" gap="0.001" '
                          'solmix="0.3" friction="0.7 0.01 0.0002" solref="0.03 0.9" '
                          'solimp="0.8 0.9 0.002 0.4 3" condim="4"/>'),
                     (*grid(3, 0.1, 0.0)[:3], NONE + ' margin="0.002" gap="0.0005" solmix="0.7" '
                      'friction="0.9 0.003 0.0003" solref="0.025 1.1" solimp="0.85 0.97 0.003 0.6 2.5"'),
                     # Elliptic: a pyramidal row's diagApprox with a friction
                     # other than 1 rounds otherwise in sim-core (ledger L227).
                     'cone="elliptic"', "", ""),
    "priority_cloth": (body("0.025 0.025 0.035", '<geom type="sphere" size="0.03" friction="0.7 0.01 0.0002"/>'),
                       (*grid(3, 0.1, 0.0)[:3], NONE + ' priority="1" condim="1"'), "", "", ""),
    "plane_cloth": ('<geom type="plane" size="2 2 0.1"/>', (*grid(3, 0.05, 0.005)[:3], NONE), "", "", ""),
    "plane_cloth_no_midphase": ('<geom type="plane" size="2 2 0.1"/>', (*grid(3, 0.05, 0.005)[:3], NONE),
                                "", 'midphase="disable"', ""),
    "plane_gap_cloth": ('<body name="ground"><geom type="plane" size="2 2 0.1"/></body>',
                        (*grid(3, 0.05, 0.02)[:3], NONE + ' margin="0.02" gap="0.015"'), "", "", ""),
    # A passive flex (mj_nc's exclude 4, mj_contactPassive): its contacts make
    # no rows and push as springs into qfrc_spring, a gap's too.
    "passive_sphere_cloth": (SPHERE, (*grid(3, 0.1, 0.0)[:3], NONE + ' passive="true"'), "", "", ""),
    "passive_box_cloth": (body("0.03 0.02 0.025", '<geom type="box" size="0.03 0.02 0.02" condim="4"/>',
                               "0.98 0.1 0.15 0.05"), (*grid(3, 0.1, 0.0)[:3], NONE + ' passive="true"'),
                          'jacobian="sparse"', "", ""),
    "passive_plane_gap_cloth": ('<body name="ground"><geom type="plane" size="2 2 0.1"/></body>',
                                (*grid(3, 0.05, 0.02)[:3], NONE + ' margin="0.02" gap="0.015" passive="true"'),
                                "", "", ""),
    "hfield_cloth": ('<geom type="hfield" hfield="hf" pos="0.01 -0.02 0.0" quat="0.99 0.02 0.03 0.1"/>',
                     (*grid(3, 0.1, 0.075)[:3], NONE), "", "", FIELD),
    "hfield_cable": ('<geom type="hfield" hfield="hf" pos="0.01 -0.02 0.0" quat="0.99 0.02 0.03 0.1"/>',
                     ([[-0.1, 0.0, 0.075], [0.0, 0.0, 0.075], [0.1, 0.02, 0.08]], [0, 1, 1, 2], 1, NONE),
                     "", "", FIELD),
    "hfield_tet": ('<geom type="hfield" hfield="hf" pos="0.01 -0.02 0.0" quat="0.99 0.02 0.03 0.1"/>',
                   ([[-0.1, -0.1, 0.07], [0.12, -0.08, 0.075], [-0.02, 0.13, 0.07], [0.0, 0.0, 0.2]],
                    [0, 1, 2, 3], 3, NONE), "", "", FIELD),
    # Five tetrahedra in a cube, the middle one on layer 1, inactive: out of
    # the hierarchy, so the midphase never tests it; without it, every
    # element is tested.
    "cube_sphere": (body("0.05 0.05 0.09", '<geom type="sphere" size="0.04"/>'), CUBE, "", "", ""),
    "cube_sphere_no_midphase": (body("0.05 0.05 0.09", '<geom type="sphere" size="0.04"/>'), CUBE,
                                "", 'midphase="disable"', ""),
    "cube_box": (body("0.06 0.06 0.06", '<geom type="box" size="0.03 0.03 0.03"/>'), CUBE, "", "", ""),
    "cube_box_no_midphase": (body("0.06 0.06 0.06", '<geom type="box" size="0.03 0.03 0.03"/>'), CUBE,
                             "", 'midphase="disable"', ""),
    # A world plane whose bits miss the flex's, beside a box: under the
    # midphase the plane is tested against every vertex without its bitmask;
    # without it, the bitmask drops it.
    "plane_bitmask_cloth": (PLANE_BITS, (*grid(3, 0.05, 0.012)[:3], NONE), "", "", ""),
    "plane_bitmask_cloth_no_midphase": (PLANE_BITS, (*grid(3, 0.05, 0.012)[:3], NONE), "",
                                        'midphase="disable"', ""),
    # The override's margin, halved, grows each side's box in the broadphase
    # (engine_collision_driver.c:915): a gap of 8.5 mm over the cloth's middle
    # vertex, along an axis of the broadphase's frame, which the margin's half
    # on one side and a quarter on the other would not pair.
    "override_sphere_cloth_alone": (body("0 0 0.0485", '<geom type="sphere" size="0.03"/>'), [CLOTH],
                                    'o_margin="0.01"', 'override="enable" gravity="disable"', ""),
    # An adhesion actuator counts the body's contacts with geoms, not with a
    # flex (engine_core_smooth.c:1653-1656): a sphere on a cloth, touching a
    # world sphere.
    "adhesion_sphere_cloth": (body("0.025 0.025 0.035", '<geom type="sphere" size="0.03"/>')
                              + '<geom type="sphere" size="0.025" pos="0.025 0.075 0.04"/>', CLOTH, "", "", "",
                              '<actuator><adhesion body="b" ctrlrange="0 1" gain="2"/></actuator>'),
}


def documents(rigid, flex, option, flag, asset, tail=""):
    head = (f'<mujoco><option timestep="0.002" {option}><flag equality="disable" {flag}/></option>'
            f'<asset>{asset}</asset><worldbody>{rigid}')
    mj_bodies, mj_flexes, ours = "", "", ""
    flexes = flex if isinstance(flex, list) else [DUMMY, flex]
    for f, (verts, elems, dim, contact) in enumerate(flexes):
        names = []
        for i, v in enumerate(verts):
            name = f"f{f}v{i}"
            names.append(name)
            mj_bodies += (f'<body name="{name}" pos="{v[0]!r} {v[1]!r} {v[2]!r}">'
                          '<joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/>'
                          '<joint type="slide" axis="0 0 1"/>'
                          f'<inertial pos="0 0 0" mass="{MASS}" diaginertia="1e-5 1e-5 1e-5"/></body>')
        mj_flexes += (f'<flex name="f{f}" dim="{dim}" radius="0.01" body="{" ".join(names)}" '
                      f'vertex="{" ".join(["0 0 0"] * len(verts))}" '
                      f'element="{" ".join(map(str, elems))}"><contact {contact}/></flex>')
    mj = f'{head}{mj_bodies}</worldbody><deformable>{mj_flexes}</deformable>{tail}</mujoco>'
    m = mujoco.MjModel.from_xml_string(mj)
    for f, (verts, _, dim, contact) in enumerate(flexes):
        va, vn = m.flex_vertadr[f], m.flex_vertnum[f]
        da, en = m.flex_elemdataadr[f], m.flex_elemnum[f]
        vtxt = "  ".join(" ".join(repr(float(x)) for x in m.body_pos[b]) for b in m.flex_vertbodyid[va:va + vn])
        etxt = " ".join(str(int(i)) for i in m.flex_elem[da:da + en * (dim + 1)])
        total = MASS * len(verts)
        assert total / len(verts) == MASS, "sim-mjcf's vertex mass must be exactly MASS"
        ours += (f'<flex name="f{f}" dim="{dim}" radius="0.01" mass="{total!r}"><contact {contact}/>'
                 f'<vertex pos="{vtxt}"/><element data="{etxt}"/></flex>')
    ours = f'{head}</worldbody><deformable>{ours}</deformable>{tail}</mujoco>'
    return mj, ours, m


def forward(m, d):
    contacts = []
    for c in d.contact[:d.ncon]:
        contacts.append({
            "geom": ints(c.geom), "flex": ints(c.flex), "elem": ints(c.elem), "vert": ints(c.vert),
            "dist": float(c.dist), "pos": floats(c.pos), "frame": floats(c.frame), "dim": int(c.dim),
            "includemargin": float(c.includemargin), "friction": floats(c.friction),
            "solref": floats(c.solref), "solimp": floats(c.solimp), "exclude": int(c.exclude),
            "efc_address": int(c.efc_address)})
    out = {"contacts": contacts, "nefc": int(d.nefc), "efc_type": ints(d.efc_type[:d.nefc]),
           "efc_R": floats(d.efc_R[:d.nefc]), "efc_D": floats(d.efc_D[:d.nefc]),
           "qacc": floats(d.qacc), "qfrc_constraint": floats(d.qfrc_constraint),
           "qfrc_spring": floats(d.qfrc_spring), "qfrc_passive": floats(d.qfrc_passive)}
    if mujoco.mj_isSparse(m):
        rows = []
        for r in range(d.nefc):
            a, n = d.efc_J_rowadr[r], d.efc_J_rownnz[r]
            rows.append({"colind": ints(d.efc_J_colind[a:a + n]), "values": floats(d.efc_J[a:a + n])})
        out["efc_J_rows"] = rows
    else:
        out["efc_J"] = floats(d.efc_J[:d.nefc * m.nv])
    return out


def case(name, rigid, flex, option, flag, asset, tail=""):
    rigid = settled(rigid, flex, option, flag, asset, tail)
    mj, ours, m = documents(rigid, flex, option, flag, asset, tail)
    flexes = flex if isinstance(flex, list) else [DUMMY, flex]
    passive = [int('passive="true"' in contact) for (_, _, _, contact) in flexes]
    return record(name, mj, ours, m, passive)


def record(name, mj, ours, m, passive=()):
    """A case's record of the compiled MuJoCo model `m` of document `mj`,
    sim-mjcf's being `ours`; `passive` each flex's passive flag as written."""
    out = {"name": name, "xml": mj, "ours": ours, "nv": int(m.nv), "nbody": int(m.nbody)}
    if any(passive):
        # MuJoCo 3.5.0's reader overwrites a flex's passive flag with a stale
        # variable whenever `passive` is given (xml_native_reader.cc:1457-1523;
        # registry D-FLEX-PASSIVE-READ): the attribute is set on the compiled
        # model, and what the reader gave is recorded.
        out["flex_passive_read"] = ints(m.flex_passive)
        m.flex_passive[:] = passive
    d = mujoco.MjData(m)
    out["qpos0"] = floats(d.qpos)
    if m.nu:
        d.ctrl[:] = 0.5
        out["ctrl"] = floats(d.ctrl)
    mujoco.mj_forward(m, d)
    out.update(forward(m, d))
    trajectory = {}
    for step in range(1, max(CHECKPOINTS) + 1):
        mujoco.mj_step(m, d)
        if step in CHECKPOINTS:
            trajectory[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel), "ncon": int(d.ncon)}
    out["trajectory"] = trajectory
    return out


# A world plane under a 3x3 cloth 5 mm up, its first vertex pinned: on the
# world body in MuJoCo's document, a <pin> in sim-mjcf's; the others bodies of
# mass MASS with three slides.
PINNED_VERTS = [[x * 0.05, y * 0.05, 0.005] for y in range(3) for x in range(3)]
PINNED_ELEMS = grid(3, 0.05, 0.005)[1]


def pinned_cloth(option, flag):
    """MuJoCo's and sim-mjcf's documents of the pinned cloth over a plane."""
    head = (f'<mujoco><option timestep="0.002" {option}><flag equality="disable" {flag}/></option>'
            '<worldbody><geom type="plane" size="1 1 0.1"/>')
    v0 = PINNED_VERTS[0]
    bodies = "".join(f'<body name="v{i}" pos="{v[0]!r} {v[1]!r} {v[2]!r}"><joint type="slide" axis="1 0 0"/>'
                     '<joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>'
                     f'<inertial pos="0 0 0" mass="{MASS}" diaginertia="1e-5 1e-5 1e-5"/></body>'
                     for i, v in enumerate(PINNED_VERTS) if i)
    names = "world " + " ".join(f"v{i}" for i in range(1, len(PINNED_VERTS)))
    vertex = f"{v0[0]!r} {v0[1]!r} {v0[2]!r}  " + "  ".join(["0 0 0"] * (len(PINNED_VERTS) - 1))
    elems = " ".join(map(str, PINNED_ELEMS))
    mj = (f'{head}{bodies}</worldbody><deformable><flex name="c" dim="2" radius="0.01" body="{names}" '
          f'vertex="{vertex}" element="{elems}"><contact {NONE}/></flex></deformable></mujoco>')
    verts = "  ".join(" ".join(repr(x) for x in v) for v in PINNED_VERTS)
    ours = (f'{head}</worldbody><deformable><flex name="c" dim="2" radius="0.01" mass="{MASS * len(PINNED_VERTS)!r}">'
            f'<contact {NONE}/><vertex pos="{verts}"/><element data="{elems}"/><pin id="0"/></flex></deformable></mujoco>')
    return mj, ours


def sleeping_sphere(body_last):
    """MuJoCo's document of a sphere asleep under a cloth (sleep="init"), the
    sphere's body declared first or last."""
    rigid = ('<body name="b" pos="0.025 0.025 -0.0405" quat="1 0 0 0" sleep="init"><freejoint/>' + INERTIAL
             + '<geom type="sphere" size="0.03"/></body>')
    mj, _, _ = documents(rigid, CLOTH, "", 'sleep="enable"', "")
    if body_last:
        assert mj.count(rigid) == 1
        mj = mj.replace(rigid, "").replace("</worldbody>", rigid + "</worldbody>")
    return mj


def mujoco_stops():
    """Registry D-FLEX-NO-TREE and D-FLEX-SLEEP-ISLAND, where MuJoCo 3.5.0
    raises an error its own source marks SHOULD NOT OCCUR. The pinned cloth:
    at the dense size with islands mj_forward raises (treeFirst, "no tree
    found", engine_island.c:271-277; the pinned vertex's contact has no dof
    on either side); without islands, and at the sparse size (the contact
    excluded, mj_instantiateContact's exclude 3), it runs. A sphere asleep
    under an awake cloth: its body declared first, mj_step raises at the
    sixth step ("found sleeping tree", engine_sleep.c:542; mj_wakeCollision
    skips a contact with a flex side); declared last, 59 steps run, the
    sphere asleep. Asserted each time the golden is made."""
    def raises(f):
        try:
            f()
        except Exception as e:  # noqa: BLE001 - MuJoCo's mjERROR arrives as a plain Exception
            return str(e)
        return None
    for option, flag, stops in (("", "", True), ("", 'island="disable"', False), ('jacobian="sparse"', "", False)):
        m = mujoco.MjModel.from_xml_string(pinned_cloth(option, flag)[0])
        d = mujoco.MjData(m)
        assert not mujoco.mj_isSparse(m) or option, (option, flag)
        err = raises(lambda: mujoco.mj_forward(m, d))
        assert (err is not None and "no tree found" in err) == stops, (option, flag, err)
    for body_last, stop_step in ((False, 6), (True, None)):
        m = mujoco.MjModel.from_xml_string(sleeping_sphere(body_last))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "b")
        stopped = None
        for step in range(1, 60):
            if raises(lambda: mujoco.mj_step(m, d)) is not None:
                stopped = step
                break
        assert stopped == stop_step, (body_last, stopped)
        if stopped is None:
            assert d.tree_asleep[m.body_treeid[b]] >= 0, "the sphere asleep"


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    import json
    marker = oracle()
    mujoco_stops()
    mj, ours = pinned_cloth("", 'island="disable"')
    pinned = record("pinned_cloth_no_islands", mj, ours, mujoco.MjModel.from_xml_string(mj))
    doc = {"oracle": marker, "cases": [case(n, *c) for n, c in CASES.items()] + [pinned]}
    for c in doc["cases"]:
        kinds = sorted({("elem" if x["elem"][1] >= 0 else "vert", x["dim"], x["exclude"]) for x in c["contacts"]})
        steps = [c["trajectory"][k]["ncon"] for k in sorted(c["trajectory"], key=int)]
        print(f'{c["name"]:30s} ncon {len(c["contacts"]):3d} nefc {c["nefc"]:3d} {kinds} ncon@steps {steps}')
    with open(os.path.join(sys.argv[1], "flex_rigid.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


if __name__ == "__main__":
    main()
