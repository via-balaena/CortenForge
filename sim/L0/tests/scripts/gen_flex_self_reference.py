#!/usr/bin/env python3
"""Generate the flex-against-flex collision golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_self_reference.py \\
        sim/L0/tests/assets/golden/flex

It refuses any other interpreter, as gen_census_golden.py does, and writes
flex_self.json.

Two flexes against each other (engine_collision_driver.c:264-482): the
broadphase pairs two flexes whose boxes over their vertices, grown by their
radii and margins, overlap; the midphase descends the two hierarchies
together (mj_collideTree, :622-847), each pair of leaves to
mj_collideElems (:2002-2098), and sorts the pair's contacts by element
(contactSort, :366-371); without it, every element of the first against
every element of the second. Two cables' elements meet as capsules, any
other two through the convex route. Each case is written as
gen_flex_rigid_reference.py writes its cases (each vertex a body of mass
0.125 with three slides; MuJoCo's <flex body=...> and sim-mjcf's
<deformable><flex>; equality constraints disabled on both), most after the
same dummy cable that collides with nothing, and records the same values.
"""
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402
from gen_flex_rigid_reference import DUMMY, case, documents, grid  # noqa: E402

NONE = 'selfcollide="none" internal="false"'


def moved(flex, dx, dy, dz, contact=None):
    verts, elems, dim, own = flex
    return [[v[0] + dx, v[1] + dy, v[2] + dz] for v in verts], elems, dim, own if contact is None else contact


CLOTH = (*grid(3, 0.1, 0.0), NONE)
# The second cloth half a cell over and 15 mm up: its vertices over the
# first's cell centres.
OVER = moved(CLOTH, 0.05, 0.05, 0.015)
CABLE = ([[-0.1, 0.0, 0.0], [0.0, 0.0, 0.0], [0.1, 0.0, 0.0]], [0, 1, 1, 2], 1, NONE)
CROSS = ([[0.01, -0.1, 0.015], [0.0, 0.0, 0.015], [0.0, 0.1, 0.012]], [0, 1, 1, 2], 1, NONE)
TET = ([[-0.1, -0.1, -0.05], [0.12, -0.08, -0.04], [-0.02, 0.13, -0.06], [0.0, 0.0, 0.08]],
       [0, 1, 2, 3], 3, NONE)

# Each case: the flexes, options, flags. Two cloths and the dummy have 63
# dofs, so MuJoCo's automatic Jacobian is sparse (60 or more).
CASES = {
    "two_cloths": ([DUMMY, CLOTH, OVER], "", ""),
    "two_cloths_no_midphase": ([DUMMY, CLOTH, OVER], "", 'midphase="disable"'),
    "two_cloths_dense": ([DUMMY, CLOTH, OVER], 'jacobian="dense"', ""),
    # 25 mm apart: every contact within the margin; in the gap (all excluded),
    # and with a smaller gap, past it (rows).
    "two_cloths_gap": ([DUMMY, moved(CLOTH, 0, 0, 0, NONE + ' margin="0.01" gap="0.008"'),
                        moved(CLOTH, 0.05, 0.05, 0.025)], "", ""),
    "two_cloths_margin": ([DUMMY, moved(CLOTH, 0, 0, 0, NONE + ' margin="0.01" gap="0.002"'),
                           moved(CLOTH, 0.05, 0.05, 0.025)], "", ""),
    "two_cloths_params": ([DUMMY, moved(CLOTH, 0, 0, 0, NONE + ' solmix="0.3" friction="0.7 0.01 0.0002" '
                                        'solref="0.03 0.9" solimp="0.8 0.9 0.002 0.4 3"'),
                           moved(OVER, 0, 0, 0, NONE + ' solmix="0.7" friction="0.9 0.003 0.0003" '
                                 'solref="0.025 1.1" solimp="0.85 0.97 0.003 0.6 2.5"')],
                          'cone="elliptic"', ""),
    "two_cloths_priority": ([DUMMY, moved(CLOTH, 0, 0, 0, NONE + ' priority="1" condim="1"'),
                             moved(OVER, 0, 0, 0, NONE + ' friction="0.7 0.01 0.0002"')], "", ""),
    # Dense: at the sparse size MuJoCo's rotational rows of an element
    # contact change from one call to the next (sparse_rotation_rows_vary).
    "two_cloths_condim6": ([DUMMY, CLOTH, moved(OVER, 0, 0, 0, NONE + ' condim="6"')], 'jacobian="dense"', ""),
    # The override's margin replaces the flexes' in the narrowphase and the
    # hierarchies' boxes: 25 mm apart, contacts only within it.
    "two_cloths_override": ([DUMMY, CLOTH, moved(CLOTH, 0.05, 0.05, 0.025)], 'o_margin="0.01"',
                            'override="enable"'),
    "two_cables": ([DUMMY, CABLE, CROSS], "", ""),
    "two_cables_no_midphase": ([DUMMY, CABLE, CROSS], "", 'midphase="disable"'),
    "cable_cloth": ([DUMMY, moved(CROSS, 0, 0, -0.003), CLOTH], "", ""),
    "tet_tet": ([DUMMY, TET, moved(TET, 0.05, 0.02, 0.1)], "", ""),
    "cloth_tet": ([DUMMY, moved(CLOTH, 0, 0, 0.085), TET], "", ""),
    # A cloth with a cloth beside it in each of six directions, 35 mm off:
    # each pair's element gap 15 mm, within the margins' 20 mm, and their
    # boxes 15 mm apart along one axis (filterBox's six tests, the first
    # cloth's box below the other's and above it, on each axis); the six,
    # 49.5 mm apart corner to corner, meet only the first.
    "margin_ring": ([DUMMY, moved(CLOTH, 0, 0, 0, NONE + ' margin="0.01"')]
                    + [moved(CLOTH, *shift, contact=NONE + ' margin="0.01"')
                       for shift in ((0.235, 0, 0), (-0.235, 0, 0), (0, 0.235, 0), (0, -0.235, 0),
                                     (0, 0, 0.035), (0, 0, -0.035))], "", ""),
    # Two 3x3 grids 1 apart in one plane, the second half a cell over, with
    # flex_flex_collision.rs t02's bits and margins.
    "coplanar_grids": ([DUMMY, ([[x, y, 0.0] for y in (0, 1, 2) for x in (0, 1, 2)], CLOTH[1], 2,
                                NONE + ' contype="1" conaffinity="3" margin="0.01"'),
                        ([[x + 0.5, y + 0.5, 0.0] for y in (0, 1, 2) for x in (0, 1, 2)], CLOTH[1], 2,
                         NONE + ' contype="2" conaffinity="1" margin="0.01"')], "", ""),
    # Three flexes, each pair's contacts in signature order: a cable between
    # two cloths, within reach of both, whose bits miss the first cloth's.
    "three_flexes": ([DUMMY, CLOTH, moved(OVER, 0, 0, 0, NONE + ' contype="3" conaffinity="3"'),
                      moved(CROSS, 0, 0, -0.0075, NONE + ' contype="2" conaffinity="2"')], "", ""),
}


def sparse_rotation_rows_vary():
    """Registry D-FLEX-SPARSE-ROT: at the sparse size MuJoCo 3.5.0's rotational
    rows of an element contact (rows 4-9 of each condim-6 pyramid of
    two_cloths_condim6) change from one mj_forward to the next at the same
    state, the first call's equal to the dense size's; at the dense size they
    do not change. Asserted each time the golden is made."""
    flexes, option, flag = CASES["two_cloths_condim6"]
    rows = {}
    for size in ("sparse", "dense"):
        mj, _, _ = documents("", flexes, option.replace("dense", size), flag, "")
        m = mujoco.MjModel.from_xml_string(mj)
        d = mujoco.MjData(m)
        calls = []
        for _ in range(3):
            mujoco.mj_forward(m, d)
            j = np.zeros((d.nefc, m.nv))
            for r in range(d.nefc):
                if mujoco.mj_isSparse(m):
                    a, n = d.efc_J_rowadr[r], d.efc_J_rownnz[r]
                    j[r, d.efc_J_colind[a:a + n]] = d.efc_J[a:a + n]
                else:
                    j[r] = d.efc_J[r * m.nv:(r + 1) * m.nv]
            calls.append(j)
        rows[size] = calls
    sparse, dense = rows["sparse"], rows["dense"]
    changed = sorted({int(r) % 10 for r in np.nonzero(np.any(sparse[0] != sparse[1], axis=1))[0]})
    assert changed == [4, 5, 6, 7, 8, 9], changed
    assert not np.array_equal(sparse[1], sparse[2])
    assert np.array_equal(sparse[0], dense[0])
    assert np.array_equal(dense[0], dense[1]) and np.array_equal(dense[1], dense[2])


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    import json
    marker = oracle()
    sparse_rotation_rows_vary()
    doc = {"oracle": marker, "cases": [case(n, "", flexes, option, flag, "") for n, (flexes, option, flag)
                                       in CASES.items()]}
    for c in doc["cases"]:
        kinds = sorted({("vert" if x["vert"][0] >= 0 else "elem", "self" if x["flex"][0] == x["flex"][1] else "pair",
                         x["dim"], x["exclude"]) for x in c["contacts"]})
        steps = [c["trajectory"][k]["ncon"] for k in sorted(c["trajectory"], key=int)]
        print(f'{c["name"]:30s} ncon {len(c["contacts"]):3d} nefc {c["nefc"]:3d} {kinds} ncon@steps {steps}')
    with open(os.path.join(sys.argv[1], "flex_self.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


if __name__ == "__main__":
    main()
