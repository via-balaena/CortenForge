#!/usr/bin/env python3
"""Generate the collision golden of a flex against a flex and against itself from the
unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_self_reference.py \\
        sim/L0/tests/assets/golden/flex

It refuses any other interpreter, as gen_census_golden.py does, and writes
flex_self.json.

Two flexes against each other (engine_collision_driver.c:264-482): the
broadphase pairs two flexes whose boxes over their vertices, grown by their
radii and margins, overlap; the midphase descends the two hierarchies
together (mj_collideTree, :622-847), each pair of leaves to mj_collideElems
(:2002-2098), and sorts the pair's contacts by element (contactSort,
:366-371); without it, every element of the first against every element of
the second. A flex against itself (:428-475), after every pair: a flex not
rigid whose contype meets its own conaffinity, its active elements
(mj_isElemActive, :214-220) through its own hierarchy (selfcollide bvh, or
auto for a tetrahedral flex), by sweep and prune along the hierarchy's
longest axis (sap, or auto otherwise; mj_collideFlexSAP, :1809-1861), or
each pair in turn (narrow, or without the midphase), its margin and gap left
out of the narrow phase, its contacts in the order made. Two cables'
elements meet as capsules, any other two through the convex route. Each case
is written as gen_flex_rigid_reference.py writes its cases (each vertex a
body of mass 0.125 with three slides; MuJoCo's <flex body=...> and
sim-mjcf's <deformable><flex>; equality constraints disabled on both), most
after the same dummy cable that collides with nothing, and records the same
values.
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


def folded(contact):
    """A strip of four cells folded over itself: the top half 12 mm above the
    bottom and 12 mm along, its elements within reach of the bottom's."""
    verts, elems = [], []
    for i in range(5):
        verts += [[i * 0.05, 0.0, 0.0], [i * 0.05, 0.05, 0.0]]
    for i in range(5):
        verts += [[0.2 - i * 0.05 + 0.012, 0.012, 0.012], [0.2 - i * 0.05 + 0.012, 0.062, 0.012]]
    for i in range(9):
        a, b, c, d = 2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3
        elems += [a, c, b, c, d, b]
    return verts, elems, 2, contact


def stacked(contact):
    """Four triangles of one flex, each 12 mm above the last (ledger L158's
    scene)."""
    verts, elems = [], []
    for k in range(4):
        z, base = 0.012 * k, 3 * k
        verts += [[0.0, 0.0, z], [0.1, 0.0, z], [0.0, 0.1, z]]
        elems += [base, base + 1, base + 2]
    return verts, elems, 2, contact


def staircase(contact):
    """Four triangles of one flex, each 12 mm above the last and 20 mm along x
    and back along y: its hierarchy's root box as long in x as in y, exactly,
    and the triangles in one order along x and the other along y."""
    verts, elems = [], []
    for k in range(4):
        x, y, z, base = 0.02 * k, -0.02 * k, 0.012 * k, 3 * k
        verts += [[x, y, z], [x + 0.1, y, z], [x, y + 0.1, z]]
        elems += [base, base + 1, base + 2]
    return verts, elems, 2, contact


# A cable bent back on itself, its return leg 15 mm above.
CABLE_FOLD = [[0.0, 0, 0], [0.05, 0, 0], [0.1, 0, 0], [0.1, 0, 0.015], [0.05, 0.002, 0.015], [0.0, 0.004, 0.015]]
# Two tetrahedra of one flex, overlapping, sharing no vertex.
TWO_TETS = [[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1],
            [0.03, 0.03, 0.03], [0.13, 0.03, 0.03], [0.03, 0.13, 0.03], [0.03, 0.03, 0.13]]
# Two cubes of five tetrahedra each in one flex, the second a third of the
# way into the first: the middle tetrahedron of each is on layer 1.
CUBE = [[(i & 1) * 0.1, ((i >> 1) & 1) * 0.1, ((i >> 2) & 1) * 0.1] for i in range(8)]
CUBE_TETS = [0, 1, 2, 4, 3, 2, 1, 7, 5, 1, 4, 7, 6, 4, 2, 7, 1, 2, 4, 7]
TWO_CUBES = (CUBE + [[x + 0.03, y + 0.035, z + 0.04] for x, y, z in CUBE], CUBE_TETS + [i + 8 for i in CUBE_TETS], 3)


INTERNAL = 'selfcollide="none" internal="true"'
# A cable bent sharply at its middle vertex: its last vertex 12 mm from its
# first element.
CABLE_V = ([[0, 0, 0], [0.1, 0, 0], [0.005, 0.012, 0.0]], [0, 1, 1, 2], 1)
# The same, its last vertex 25 mm from its first element: 5 mm clear of it,
# within a 10 mm margin; 1 m along x.
CABLE_V_CLEAR = ([[1.0, 0, 0], [1.1, 0, 0], [1.005, 0.025, 0.0]], [0, 1, 1, 2], 1)
# Five tetrahedra of a cube 15 mm on a side: every vertex within two radii of
# its tetrahedron's opposite faces.
SMALL_CUBE = ([[(i & 1) * 0.015, ((i >> 1) & 1) * 0.015, ((i >> 2) & 1) * 0.015] for i in range(8)], CUBE_TETS, 3)
# A tetrahedron 6 mm high, and turned inside out.
SQUASHED = ([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0.03, 0.03, 0.006]], [0, 1, 2, 3], 3)
INVERTED = ([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0.03, 0.03, -0.006]], [0, 1, 2, 3], 3)
# A tetrahedron whose last vertex is two radii behind its first face, exactly
# (planeVertex's `dst <= -2*rad`, no contact there), one face in contact.
TWO_RADII = ([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0, 0, -0.02]], [0, 1, 2, 3], 3)


def self_flex(shape, mode, extra=""):
    verts, elems, dim = shape
    return verts, elems, dim, f'selfcollide="{mode}" internal="false"{extra}'

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
    # A flex against itself (engine_collision_driver.c:428-475), after every
    # pair: a cloth's active elements by sweep and prune along its
    # hierarchy's longest axis (auto, sap), its hierarchy (bvh), or each pair
    # in turn (narrow, or without the midphase); none, nothing.
    "folded_auto": ([DUMMY, folded('selfcollide="auto" internal="false"')], "", ""),
    "folded_sap": ([DUMMY, folded('selfcollide="sap" internal="false"')], "", ""),
    "folded_bvh": ([DUMMY, folded('selfcollide="bvh" internal="false"')], "", ""),
    "folded_narrow": ([DUMMY, folded('selfcollide="narrow" internal="false"')], "", ""),
    "folded_auto_no_midphase": ([DUMMY, folded('selfcollide="auto" internal="false"')], "", 'midphase="disable"'),
    "folded_none": ([DUMMY, folded('selfcollide="none" internal="false"')], "", ""),
    # The margin and gap left out of a flex's collisions with itself.
    "folded_margin_gap": ([DUMMY, folded('selfcollide="auto" internal="false" margin="0.01" gap="0.004"')], "", ""),
    # Its own bits miss each other: no collisions with itself; a cable whose
    # bits meet the strip's lies across it.
    "folded_bits": ([DUMMY, folded('selfcollide="auto" internal="false" contype="1" conaffinity="2"'),
                     ([[0.1, -0.05, 0.022], [0.1, 0.0, 0.022], [0.1, 0.1, 0.022]], [0, 1, 1, 2], 1,
                      NONE + ' contype="2" conaffinity="1"')], "", ""),
    # A cloth against a cloth, then each against itself.
    "pairs_then_self": ([DUMMY, folded('selfcollide="sap" internal="false"'),
                         moved(CLOTH, 0.1, 0.03, 0.027, 'selfcollide="narrow" internal="false"')], "", ""),
    "stacked_auto": ([DUMMY, stacked('selfcollide="auto" internal="false"')], "", ""),
    "stacked_auto_no_midphase": ([DUMMY, stacked('selfcollide="auto" internal="false"')], "", 'midphase="disable"'),
    "stacked_bvh": ([DUMMY, stacked('selfcollide="bvh" internal="false"')], "", ""),
    # Swept along y, not x: the first of the root box's equal longest axes
    # after x (engine_collision_driver.c:1844).
    "staircase_sap": ([DUMMY, staircase('selfcollide="sap" internal="false"')], "", ""),
    "cable_fold_auto": ([DUMMY, (CABLE_FOLD, [0, 1, 1, 2, 2, 3, 3, 4, 4, 5], 1, 'selfcollide="auto" internal="false"')],
                        "", ""),
    "cable_fold_bvh": ([DUMMY, (CABLE_FOLD, [0, 1, 1, 2, 2, 3, 3, 4, 4, 5], 1, 'selfcollide="bvh" internal="false"')],
                       "", ""),
    "two_tets_auto": ([DUMMY, (TWO_TETS, [0, 1, 2, 3, 4, 5, 6, 7], 3, 'selfcollide="auto" internal="false"')], "", ""),
    "two_tets_sap": ([DUMMY, (TWO_TETS, [0, 1, 2, 3, 4, 5, 6, 7], 3, 'selfcollide="sap" internal="false"')], "", ""),
    "two_tets_narrow": ([DUMMY, (TWO_TETS, [0, 1, 2, 3, 4, 5, 6, 7], 3, 'selfcollide="narrow" internal="false"')],
                        "", ""),
    # A tetrahedral flex's active layers: the middle tetrahedra out (1, the
    # default) or in (2), in the hierarchy and in each pair taken in turn.
    "two_cubes_auto": ([DUMMY, self_flex(TWO_CUBES, "auto")], "", ""),
    "two_cubes_layers2": ([DUMMY, self_flex(TWO_CUBES, "auto", ' activelayers="2"')], "", ""),
    "two_cubes_narrow": ([DUMMY, self_flex(TWO_CUBES, "narrow")], "", ""),
    "two_cubes_sap_layers2": ([DUMMY, self_flex(TWO_CUBES, "sap", ' activelayers="2"')], "", ""),
    # A flex's internal collisions (mj_collideFlexInternal, :1745-1806), before
    # its elements against each other: each element against the vertex of
    # its neighbour across a shared fragment, a border element's (a cable's
    # or a cloth's; mj_collideElemVert, :2101-2180), and each tetrahedron's
    # faces against its opposite vertices (planeVertex, :1714-1740, condim
    # 1), the margin and gap left out.
    "cable_v_internal": ([DUMMY, (*CABLE_V, INTERNAL)], "", ""),
    "cable_v_internal_condim4": ([DUMMY, (*CABLE_V, INTERNAL + ' condim="4"')], "", ""),
    "folded_internal": ([DUMMY, folded(INTERNAL)], "", ""),
    # Its margin not in the element against its neighbour's vertex: a cable
    # whose vertex is within its margin and clear of contact meets nothing,
    # beside one in contact, and a folded strip with a margin.
    "cable_v_internal_margin": ([DUMMY, (*CABLE_V, INTERNAL),
                                 (*CABLE_V_CLEAR, INTERNAL + ' margin="0.01" gap="0.003"')], "", ""),
    "folded_internal_margin": ([DUMMY, folded(INTERNAL + ' margin="0.01" gap="0.003"')], "", ""),
    "folded_internal_and_self": ([DUMMY, folded('selfcollide="sap" internal="true"')], "", ""),
    "small_cube_internal": ([DUMMY, (*SMALL_CUBE, INTERNAL)], "", ""),
    "squashed_tet_internal": ([DUMMY, (*SQUASHED, INTERNAL)], "", ""),
    "inverted_tet_internal": ([DUMMY, (*INVERTED, INTERNAL)], "", ""),
    "two_radii_tet_internal": ([DUMMY, (*TWO_RADII, INTERNAL)], "", ""),
    "squashed_internal_margin_gap": ([DUMMY, (*SQUASHED, INTERNAL + ' margin="0.01" gap="0.003" condim="4"')],
                                     "", ""),
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
