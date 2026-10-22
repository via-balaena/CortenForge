#!/usr/bin/env python3
"""Generate the flex data golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_data_reference.py \\
        sim/L0/tests/assets/golden/flex

It refuses any other interpreter, as gen_census_golden.py does, and writes
flex_data.json.

What the compiler makes for a flex's collisions and what mj_flex makes of it
each step: each flex's radius, its elements' layers (their distance from the
border, user_mesh.cc:4613-4642), its element-vertex pairs (user_mesh.cc:
4644-4670) and its bounding-volume hierarchy over its active elements
(user_mesh.cc:4423-4468; children, leaf elements and depths, numbered within
the hierarchy, and its first node after the bodies'); then, with every dof
moved (qpos[i] = qpos0[i] + 0.01 sin(i + 1)), each element's box and each
hierarchy node's box after mj_forward (engine_core_smooth.c:610-662).

Each case is a MuJoCo document, a free sphere then flexcomps under
<worldbody>, the first a cable so the second's vertices, elements and nodes
start past 0; with its vertex positions and elements, from which the test
writes the same flexes in sim-mjcf's <deformable><flex> form (sim-mjcf adds
a flex's vertex bodies after every other body, so the flexcomps come last).
"""
import math
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import floats, ints, oracle  # noqa: E402

CABLE = '<flexcomp name="cable" type="grid" count="4 1 1" spacing="0.1 0.1 0.1" pos="0 0 -1" ' \
        'radius="0.01" dim="1" mass="0.1"><contact selfcollide="none" internal="false"/></flexcomp>'
CLOTH = '<flexcomp name="cloth" type="grid" count="4 3 1" spacing="0.1 0.07 0.1" pos="0 0 0.5" ' \
        'radius="0.02" dim="2" mass="0.1"><contact selfcollide="none" internal="false"/></flexcomp>'
TETS = '<flexcomp name="tets" type="grid" count="4 4 4" spacing="0.1 0.1 0.1" pos="0 0 0.5" ' \
       'radius="0.005" dim="3" mass="1"><contact selfcollide="none" internal="false"{layers}/></flexcomp>'
HEAD = '<mujoco><option gravity="0 0 0"><flag equality="disable"/></option><worldbody>' \
       '<body name="ball" pos="1 1 1"><freejoint/><geom type="sphere" size="0.05"/></body>'
CASES = {
    "cable_and_cloth": HEAD + CABLE + CLOTH + '</worldbody></mujoco>',
    "cable_and_tets": HEAD + CABLE + TETS.format(layers='') + '</worldbody></mujoco>',
    "cable_and_tets_two_layers": HEAD + CABLE + TETS.format(layers=' activelayers="2"') + '</worldbody></mujoco>',
    # A flex of dim below 3 has every element active, whatever activelayers says.
    "cloth_no_layer_active": HEAD + CLOTH.replace('internal="false"/>', 'internal="false" activelayers="0"/>')
    + '</worldbody></mujoco>',
}


def flex_case(m, d, f):
    va, vn = m.flex_vertadr[f], m.flex_vertnum[f]
    ea, en, dim = m.flex_elemadr[f], m.flex_elemnum[f], m.flex_dim[f]
    da = m.flex_elemdataadr[f]
    pa, pn = m.flex_evpairadr[f], m.flex_evpairnum[f]
    ba, bn = m.flex_bvhadr[f], m.flex_bvhnum[f]
    out = {
        "dim": int(dim), "radius": float(m.flex_radius[f]),
        "activelayers": int(m.flex_activelayers[f]),
        "vertices": [floats(v) for v in d.flexvert_xpos[va:va + vn]],
        "elements": ints(m.flex_elem[da:da + en * (dim + 1)]),
        "elemlayer": ints(m.flex_elemlayer[ea:ea + en]),
        "evpair": ints(m.flex_evpair[pa:pa + pn]) if pn else [],
        "bvhnum": int(bn),
    }
    if ba >= 0:
        out["bvhadr_past_static"] = int(ba - m.nbvhstatic)
        out["bvh_child"] = ints(m.bvh_child[ba:ba + bn].ravel())
        out["bvh_nodeid"] = ints(m.bvh_nodeid[ba:ba + bn])
        out["bvh_depth"] = ints(m.bvh_depth[ba:ba + bn])
    return out


def case(name, xml):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    flexes = [flex_case(m, d, f) for f in range(m.nflex)]
    d.qpos[:] = [q + 0.01 * math.sin(i + 1) for i, q in enumerate(m.qpos0)]
    mujoco.mj_forward(m, d)
    return {"name": name, "xml": xml, "nflexelem": int(m.nflexelem), "nbvhdynamic": int(m.nbvhdynamic),
            "flexes": flexes, "moved_qpos": floats(d.qpos),
            "flexelem_aabb": floats(d.flexelem_aabb), "bvh_aabb_dyn": floats(d.bvh_aabb_dyn)}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    import json
    marker = oracle()
    doc = {"oracle": marker, "cases": [case(n, x) for n, x in CASES.items()]}
    with open(os.path.join(sys.argv[1], "flex_data.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(doc['cases'])} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
