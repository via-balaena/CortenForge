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
all but one after a cable that collides with nothing, so the flex's
vertices and elements start past 0. Each vertex is a body with three slides and mass
0.125, so the document is written twice: MuJoCo's <flex body=...> over
declared bodies, and sim-mjcf's <deformable><flex> with its vertices and
the flex's whole mass, whose bodies sim-mjcf adds after the others, in the
same order; equality constraints disabled on both, as sim-mjcf makes a
flex's edges equality constraints. A rigid body's mass and inertia are
given and its quat is MuJoCo's compiled one, on both. Recorded: qpos0, and
after mj_forward each contact
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


def settled(rigid, flex, option, flag, asset):
    """The rigid text with body b's quat MuJoCo's compiled one, which sim-mjcf
    keeps as it is: MuJoCo's normalisation of another quat can differ from
    sim-mjcf's in the last bit."""
    _, _, m = documents(rigid, flex, option, flag, asset)
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
}


def documents(rigid, flex, option, flag, asset):
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
    mj = f'{head}{mj_bodies}</worldbody><deformable>{mj_flexes}</deformable></mujoco>'
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
    ours = f'{head}</worldbody><deformable>{ours}</deformable></mujoco>'
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


def case(name, rigid, flex, option, flag, asset):
    rigid = settled(rigid, flex, option, flag, asset)
    mj, ours, m = documents(rigid, flex, option, flag, asset)
    out = {"name": name, "xml": mj, "ours": ours, "nv": int(m.nv), "nbody": int(m.nbody)}
    flexes = flex if isinstance(flex, list) else [DUMMY, flex]
    passive = [int('passive="true"' in contact) for (_, _, _, contact) in flexes]
    if any(passive):
        # MuJoCo 3.5.0's reader sets a flex's passive flag to its internal
        # flag's value whenever `passive` is given (xml_native_reader.cc:
        # 1521-1523; registry D-FLEX-PASSIVE-READ): the attribute is set on
        # the compiled model, and what the reader gave is recorded.
        out["flex_passive_read"] = ints(m.flex_passive)
        m.flex_passive[:] = passive
    d = mujoco.MjData(m)
    out["qpos0"] = floats(d.qpos)
    mujoco.mj_forward(m, d)
    out.update(forward(m, d))
    trajectory = {}
    for step in range(1, max(CHECKPOINTS) + 1):
        mujoco.mj_step(m, d)
        if step in CHECKPOINTS:
            trajectory[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel), "ncon": int(d.ncon)}
    out["trajectory"] = trajectory
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    import json
    marker = oracle()
    doc = {"oracle": marker, "cases": [case(n, *c) for n, c in CASES.items()]}
    for c in doc["cases"]:
        kinds = sorted({("elem" if x["elem"][1] >= 0 else "vert", x["dim"], x["exclude"]) for x in c["contacts"]})
        steps = [c["trajectory"][k]["ncon"] for k in sorted(c["trajectory"], key=int)]
        print(f'{c["name"]:30s} ncon {len(c["contacts"]):3d} nefc {c["nefc"]:3d} {kinds} ncon@steps {steps}')
    with open(os.path.join(sys.argv[1], "flex_rigid.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


if __name__ == "__main__":
    main()
