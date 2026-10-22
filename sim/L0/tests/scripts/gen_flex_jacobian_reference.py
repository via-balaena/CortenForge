#!/usr/bin/env python3
"""Generate the flex contact Jacobian golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_jacobian_reference.py \\
        sim/L0/tests/assets/golden/flex

It refuses any other interpreter, as gen_census_golden.py does, and writes
flex_jacobian.json.

A contact with a flex element on a side takes MuJoCo's general Jacobian
(engine_core_constraint.c:932-985): each side's bodies with weights, an
element's vertices' bodies weighted by their inverse distance to the
contact (mj_elemBodyWeight, :142-182), the vertex on the other side left
out of the element; summed (mj_jacSum, engine_core_util.c:503-573) at the
dense size or over the bodies' merged chains at the sparse one. Each case:
a free geom (its condim set) against a flex, or a tetrahedron's internal
contact (its element on side 0, one of its own vertices on side 1), with
the elliptic cone, whose rows are the Jacobian's in the contact frame; a
cable first, so the second flex's vertices and elements start past 0. Each
vertex is a body with three slides, so the document is written twice:
MuJoCo's <flex body=...> over declared bodies, and sim-mjcf's
<deformable><flex> with its vertices, whose bodies sim-mjcf adds after the
others, in the same order. Recorded: each contact's sides (geom, flex,
elem, vert), position, frame and dim, and its rows: at the dense size
efc_J's, at the sparse one each row's columns (efc_J_colind) and values.
"""
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import floats, ints, oracle  # noqa: E402

CABLE = ([[-0.5, -0.5, 0.3], [-0.4, -0.5, 0.3], [-0.3, -0.5, 0.3]], [0, 1, 1, 2], 1)


def grid(n, spacing, z):
    verts = [[(i - (n - 1) / 2) * spacing, (j - (n - 1) / 2) * spacing, z] for j in range(n) for i in range(n)]
    elems = []
    for j in range(n - 1):
        for i in range(n - 1):
            a, b, c, d = j * n + i, j * n + i + 1, (j + 1) * n + i, (j + 1) * n + i + 1
            elems += [a, b, c, b, d, c]
    return verts, elems, 2


CLOTH = grid(3, 0.1, 0.0)
SEGMENT = ([[-0.1, 0.0, 0.0], [0.1, 0.02, 0.01]], [0, 1], 1)
TET_THIN = ([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0.25, 0.25, 0.004]], [0, 1, 2, 3], 3)

CASES = {
    "sphere_on_cell_condim3": ('<body name="b" pos="0.025 0.025 0.015"><freejoint/>'
                               '<geom type="sphere" size="0.01" condim="3"/></body>', [CABLE, CLOTH], "dense"),
    "sphere_on_cell_condim6": ('<body name="b" pos="0.025 0.025 0.015" quat="0.9 0.2 0.3 0.1"><freejoint/>'
                               '<geom type="sphere" size="0.01" condim="6"/></body>', [CABLE, CLOTH], "dense"),
    "sphere_on_cell_condim6_sparse": ('<body name="b" pos="0.025 0.025 0.015" quat="0.9 0.2 0.3 0.1"><freejoint/>'
                                      '<geom type="sphere" size="0.01" condim="6"/></body>', [CABLE, CLOTH], "sparse"),
    "box_on_cell": ('<body name="b" pos="0.03 0.02 0.025"><freejoint/>'
                    '<geom type="box" size="0.02 0.02 0.02" condim="4"/></body>', [CABLE, CLOTH], "dense"),
    "sphere_on_segment": ('<body name="b" pos="0.0 0.01 0.035"><freejoint/>'
                          '<geom type="sphere" size="0.03" condim="4"/></body>', [CABLE, SEGMENT], "dense"),
    "tet_internal": ('', [CABLE, TET_THIN], "dense"),
    "tet_internal_sparse": ('', [CABLE, TET_THIN], "sparse"),
}


def documents(rigid, flexes, size):
    head = (f'<mujoco><option gravity="0 0 0" cone="elliptic" jacobian="{size}"><flag equality="disable"/>'
            f'</option><worldbody>{rigid}')
    mj_bodies, mj_flexes, ours = "", "", ""
    for f, (verts, elems, dim) in enumerate(flexes):
        names = []
        for i, v in enumerate(verts):
            name = f"f{f}v{i}"
            names.append(name)
            mj_bodies += (f'<body name="{name}" pos="{v[0]!r} {v[1]!r} {v[2]!r}">'
                          '<joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/>'
                          '<joint type="slide" axis="0 0 1"/>'
                          '<inertial pos="0 0 0" mass="0.1" diaginertia="1e-5 1e-5 1e-5"/></body>')
        contact = '<contact selfcollide="none" internal="true"/>' if dim == 3 else \
            '<contact selfcollide="none" internal="false"/>'
        mj_flexes += (f'<flex name="f{f}" dim="{dim}" radius="0.01" body="{" ".join(names)}" '
                      f'vertex="{" ".join(["0 0 0"] * len(verts))}" '
                      f'element="{" ".join(map(str, elems))}">{contact}</flex>')
    mj = f'{head}{mj_bodies}</worldbody><deformable>{mj_flexes}</deformable></mujoco>'
    m = mujoco.MjModel.from_xml_string(mj)
    for f, (_, _, dim) in enumerate(flexes):
        va, vn = m.flex_vertadr[f], m.flex_vertnum[f]
        da, en = m.flex_elemdataadr[f], m.flex_elemnum[f]
        vtxt = "  ".join(" ".join(repr(float(x)) for x in m.body_pos[b]) for b in m.flex_vertbodyid[va:va + vn])
        etxt = " ".join(str(int(i)) for i in m.flex_elem[da:da + en * (dim + 1)])
        contact = '<contact selfcollide="none" internal="true"/>' if dim == 3 else \
            '<contact selfcollide="none" internal="false"/>'
        ours += (f'<flex name="f{f}" dim="{dim}" radius="0.01">{contact}'
                 f'<vertex pos="{vtxt}"/><element data="{etxt}"/></flex>')
    ours = f'{head}</worldbody><deformable>{ours}</deformable></mujoco>'
    return mj, ours, m


def case(name, rigid, flexes, size):
    mj, ours, m = documents(rigid, flexes, size)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    contacts = []
    for c in d.contact[:d.ncon]:
        adr, dim = int(c.efc_address), int(c.dim)
        out = {"geom": ints(c.geom), "flex": ints(c.flex), "elem": ints(c.elem), "vert": ints(c.vert),
               "pos": floats(c.pos), "frame": floats(c.frame), "dim": dim, "efc_address": adr}
        if adr >= 0:
            if mujoco.mj_isSparse(m):
                rows = []
                for r in range(adr, adr + dim):
                    a, n = d.efc_J_rowadr[r], d.efc_J_rownnz[r]
                    rows.append({"colind": ints(d.efc_J_colind[a:a + n]), "values": floats(d.efc_J[a:a + n])})
                out["rows"] = rows
            else:
                out["efc_J"] = floats(d.efc_J.reshape(d.nefc, m.nv)[adr:adr + dim])
        contacts.append(out)
    assert contacts, name
    return {"name": name, "xml": mj, "ours": ours, "nv": int(m.nv), "contacts": contacts}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    import json
    marker = oracle()
    doc = {"oracle": marker, "cases": [case(n, *c) for n, c in CASES.items()]}
    for c in doc["cases"]:
        kinds = sorted({(tuple(x["flex"]), x["dim"]) for x in c["contacts"]})
        print(f'{c["name"]:32s} contacts {len(c["contacts"])} {kinds}')
    with open(os.path.join(sys.argv[1], "flex_jacobian.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


if __name__ == "__main__":
    main()
