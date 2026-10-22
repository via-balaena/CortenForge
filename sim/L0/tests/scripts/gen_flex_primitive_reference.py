#!/usr/bin/env python3
"""Generate the flex primitive golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_primitive_reference.py \\
        sim/L0/tests/assets/golden/flex

It refuses any other interpreter, as gen_census_golden.py does, and writes
flex_primitives.json.

The raw functions a geom meets a flex element through
(engine_collision_driver.c:1890-1963) are not in MuJoCo's API, but a scene
holding one geom and one element calls exactly one of them, so its contact
list is that function's output after mj_setContact (which normalises the
normal, mju_makeFrame). Each case: a geom (sphere, box or capsule, turned
by a quaternion) and a flex of one element, a triangle (mjraw_SphereTriangle,
mjraw_BoxTriangle, mjraw_CapsuleTriangle) or a cable's segment, the capsule
mj_makeCapsule builds from its two vertices (with mju_quatZ2Vec), met by
mjraw_SphereCapsule, mjraw_CapsuleCapsule or mjraw_CapsuleBox; the vertices
on three (two) static bodies, so the element moves with nothing. Recorded:
the geom's position, matrix and size, the vertices, the flex radius, the
margin the driver passes (geom margin + flex margin), and each contact's
distance, position and normal. Also mju_quatZ2Vec on its own, on vectors
along +z, -z, tiny, and general.
"""
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import floats, oracle  # noqa: E402

# (name, geom xml, vertices, flex radius, flex margin)
TRI = [[-0.1, -0.1, 0.0], [0.1, -0.08, 0.01], [-0.02, 0.12, -0.005]]
CASES = [
    ("sphere_interior", '<geom type="sphere" size="0.03" pos="0.0 0.0 0.025"/>', TRI, 0.01, 0.0),
    ("sphere_below", '<geom type="sphere" size="0.03" pos="0.01 -0.01 -0.03"/>', TRI, 0.01, 0.0),
    ("sphere_edge", '<geom type="sphere" size="0.03" pos="0.0 -0.12 0.01"/>', TRI, 0.01, 0.0),
    ("sphere_vertex", '<geom type="sphere" size="0.03" pos="0.12 -0.1 0.03"/>', TRI, 0.01, 0.0),
    ("sphere_in_margin", '<geom type="sphere" size="0.03" pos="0.0 0.0 0.045" margin="0.004"/>', TRI, 0.01, 0.003),
    ("sphere_apart", '<geom type="sphere" size="0.03" pos="0.0 0.0 0.2"/>', TRI, 0.01, 0.0),
    ("sphere_degenerate", '<geom type="sphere" size="0.03" pos="0.0 0.0 0.02"/>',
     [[-0.1, 0.0, 0.0], [0.0, 0.0, 0.0], [0.1, 0.0, 0.0]], 0.01, 0.0),
    ("sphere_degenerate_near", '<geom type="sphere" size="0.03" pos="-0.09 0.0 0.02"/>',
     [[-0.1, 0.0, 0.0], [0.0, 0.0, 0.0], [0.1, 0.0, 0.0]], 0.01, 0.0),
    ("box_resting", '<geom type="box" size="0.03 0.02 0.01" pos="0.0 0.0 0.015" quat="0.98 0.1 0.15 0.05"/>', TRI, 0.01, 0.0),
    ("box_through", '<geom type="box" size="0.05 0.05 0.02" pos="0.01 0.0 0.0" quat="0.9 0.3 -0.2 0.1"/>', TRI, 0.01, 0.0),
    ("box_margin", '<geom type="box" size="0.03 0.03 0.03" pos="0.0 0.0 0.045" margin="0.01"/>', TRI, 0.01, 0.0),
    ("box_vertex_inside", '<geom type="box" size="0.03 0.03 0.03" pos="0.0 0.0 0.0"/>',
     [[0.005, -0.004, 0.02], [0.15, 0.0, 0.08], [0.0, 0.15, 0.08]], 0.01, 0.0),
    ("box_vertex_in_radius", '<geom type="box" size="0.03 0.03 0.03" pos="0.0 0.0 0.0"/>',
     [[0.035, 0.0, 0.02], [0.15, 0.0, 0.08], [0.1, 0.15, 0.08]], 0.01, 0.0),
    ("capsule_lying", '<geom type="capsule" size="0.02 0.05" pos="0.0 0.0 0.025" quat="0.7071 0 0.7071 0"/>', TRI, 0.01, 0.0),
    ("capsule_upright", '<geom type="capsule" size="0.02 0.05" pos="0.01 0.0 0.075"/>', TRI, 0.01, 0.0),
    ("capsule_vertex_on_axis", '<geom type="capsule" size="0.02 0.2" pos="0.11 -0.1 0.02" quat="0.7071 0.7071 0 0"/>', TRI, 0.01, 0.0),
    ("capsule_vertex_past_end", '<geom type="capsule" size="0.02 0.09" pos="0.22 -0.08 0.02" quat="0.7071 0 0.7071 0"/>', TRI, 0.01, 0.0),
    ("cable_sphere", '<geom type="sphere" size="0.03" pos="0.0 0.01 0.035"/>', [[-0.1, 0.0, 0.0], [0.1, 0.02, 0.01]], 0.01, 0.0),
    ("cable_capsule", '<geom type="capsule" size="0.02 0.05" pos="0.0 0.0 0.025" quat="0.9 0.1 0.4 0.1"/>', [[-0.1, 0.0, 0.0], [0.1, 0.02, 0.01]], 0.01, 0.0),
    ("cable_box", '<geom type="box" size="0.03 0.02 0.01" pos="0.0 0.0 0.02" quat="0.98 0.1 0.15 0.05"/>', [[-0.1, 0.0, 0.0], [0.1, 0.02, 0.01]], 0.01, 0.0),
    ("cable_past_end", '<geom type="sphere" size="0.03" pos="0.135 0.02 0.01"/>', [[-0.1, 0.0, 0.0], [0.1, 0.02, 0.01]], 0.01, 0.0),
    ("cable_vertical", '<geom type="sphere" size="0.03" pos="0.035 0.0 0.0"/>', [[0.0, 0.0, -0.1], [0.0, 0.0, 0.1]], 0.01, 0.0),
    ("cable_downward", '<geom type="sphere" size="0.03" pos="0.035 0.0 0.0"/>', [[0.0, 0.0, 0.1], [0.0, 0.0, -0.1]], 0.01, 0.0),
]
Z2VEC = [[0, 0, 1], [0, 0, -2], [1e-17, 0, 0], [0.3, -0.4, 0.5], [0, 1e-20, 1], [-0.2, 0.1, -0.9]]

# The convex route (mjc_ConvexElem, mjc_HFieldElem): the geoms and elements the
# raw functions do not take.
TET = [[-0.1, -0.1, -0.05], [0.12, -0.08, -0.04], [-0.02, 0.13, -0.06], [0.0, 0.0, 0.08]]
SEG = [[-0.1, 0.0, 0.0], [0.1, 0.02, 0.01]]
FIELD = ('<asset><hfield name="hf" nrow="5" ncol="6" size="0.3 0.25 0.08 0.05" '
         'elevation="0 0.2 0.4 0.3 0.1 0  0.1 0.5 0.7 0.6 0.2 0.1  0.2 0.6 1 0.8 0.3 0.1  '
         '0.1 0.4 0.6 0.5 0.2 0  0 0.1 0.3 0.2 0.1 0"/></asset>')
CONVEX = [
    ("ellipsoid_triangle", '<geom type="ellipsoid" size="0.04 0.03 0.02" pos="0.0 0.0 0.015" quat="0.95 0.1 0.2 0.1"/>', TRI, 0.01, 0.0),
    ("ellipsoid_triangle_margin", '<geom type="ellipsoid" size="0.04 0.03 0.02" pos="0.0 0.0 0.036" margin="0.01"/>', TRI, 0.01, 0.0),
    ("cylinder_triangle_wall", '<geom type="cylinder" size="0.03 0.05" pos="0.0 0.0 0.035" quat="0.7071 0.7071 0 0"/>', TRI, 0.01, 0.0),
    ("cylinder_triangle_cap", '<geom type="cylinder" size="0.03 0.05" pos="0.0 0.0 0.055"/>', TRI, 0.01, 0.0),
    ("sphere_tet", '<geom type="sphere" size="0.03" pos="0.02 0.02 0.09"/>', TET, 0.01, 0.0),
    ("capsule_tet", '<geom type="capsule" size="0.02 0.05" pos="0.0 0.0 0.1" quat="0.9 0.3 0.2 0.1"/>', TET, 0.01, 0.0),
    ("box_tet", '<geom type="box" size="0.03 0.02 0.02" pos="0.0 0.01 0.1" quat="0.95 0.1 0.2 0.15"/>', TET, 0.01, 0.003),
    ("ellipsoid_tet", '<geom type="ellipsoid" size="0.04 0.03 0.02" pos="0.0 0.0 0.095"/>', TET, 0.01, 0.0),
    ("cylinder_tet", '<geom type="cylinder" size="0.03 0.04" pos="0.0 0.0 0.12" quat="0.9 0.2 0.3 0.1"/>', TET, 0.01, 0.0),
    ("ellipsoid_cable", '<geom type="ellipsoid" size="0.04 0.03 0.02" pos="0.0 0.01 0.025"/>', SEG, 0.01, 0.0),
    ("cylinder_cable", '<geom type="cylinder" size="0.03 0.05" pos="0.0 0.01 0.035" quat="0.7071 0.7071 0 0"/>', SEG, 0.01, 0.0),
    ("hfield_triangle", "HFIELD", [[-0.05, -0.05, 0.075], [0.08, -0.03, 0.07], [0.0, 0.07, 0.072]], 0.01, 0.0),
    ("hfield_tet", "HFIELD", [[-0.1, -0.05, 0.05], [0.1, -0.06, 0.06], [0.0, 0.09, 0.055], [0.0, 0.0, 0.2]], 0.01, 0.0),
    ("hfield_triangle_deep", "HFIELD", [[-0.05, -0.05, 0.01], [0.08, -0.03, 0.012], [0.0, 0.07, 0.008]], 0.01, 0.0),
    ("hfield_triangle_margin", "HFIELD_MARGIN", [[-0.05, -0.05, 0.09], [0.08, -0.03, 0.085], [0.0, 0.07, 0.088]], 0.01, 0.002),
]


def case(name, geom, verts, radius, fmargin):
    bodies = "".join(f'<body name="v{i}" pos="{v[0]} {v[1]} {v[2]}"/>' for i, v in enumerate(verts))
    names = " ".join(f"v{i}" for i in range(len(verts)))
    dim = len(verts) - 1
    asset = ""
    if geom.startswith("HFIELD"):
        margin = ' margin="0.006"' if geom == "HFIELD_MARGIN" else ""
        asset, rigid = FIELD, f'<geom name="geom" type="hfield" hfield="hf" pos="0.01 -0.02 0.0" quat="0.99 0.02 0.03 0.1"{margin}/>'
    else:
        named = geom.replace(' pos=', ' name="geom" pos=')
        rigid = f'<body name="g" pos="0 0 0"><freejoint/>{named}</body>'
    xml = (f'<mujoco><option gravity="0 0 0"/>{asset}<worldbody>{bodies}'
           f'{rigid}'
           f'</worldbody><deformable><flex name="f" dim="{dim}" body="{names}" '
           f'vertex="{" ".join(["0 0 0"] * len(verts))}" element="{" ".join(map(str, range(len(verts))))}" '
           f'radius="{radius}"><contact margin="{fmargin}" selfcollide="none" internal="false"/></flex>'
           f'</deformable></mujoco>')
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "geom")
    assert m.flex_elemnum[0] == 1 and m.ngeom == 1, name
    cons = [{"dist": float(c.dist), "pos": floats(c.pos), "normal": floats(c.frame[:3]),
             "elem": int(c.elem[1]), "geom": int(c.geom[0])} for c in d.contact[:d.ncon]]
    assert all(c["geom"] == g and c["elem"] == 0 for c in cons), name
    out = {"name": name, "type": int(m.geom_type[g]), "pos": floats(d.geom_xpos[g]),
           "mat": floats(d.geom_xmat[g]), "size": floats(m.geom_size[g]),
           "vertices": [floats(v) for v in d.flexvert_xpos], "radius": float(m.flex_radius[0]),
           "margin": float(m.geom_margin[g] + m.flex_margin[0]), "contacts": cons,
           "center": floats(d.flexelem_aabb[0][:3]),
           "ccd": [int(m.opt.ccd_iterations), float(m.opt.ccd_tolerance)]}
    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_HFIELD:
        h = m.geom_dataid[g]
        nrow, ncol = int(m.hfield_nrow[h]), int(m.hfield_ncol[h])
        out["hfield"] = {"nrow": nrow, "ncol": ncol, "size": floats(m.hfield_size[h]),
                         "data": floats(m.hfield_data[m.hfield_adr[h]:m.hfield_adr[h] + nrow * ncol])}
    return out


def z2vec(v):
    q = np.zeros(4)
    mujoco.mju_quatZ2Vec(q, np.array(v, dtype=float))
    return {"vec": floats(v), "quat": floats(q)}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    import json
    marker = oracle()
    doc = {"oracle": marker, "cases": [case(*c) for c in CASES], "quat_z2vec": [z2vec(v) for v in Z2VEC],
           "convex": [case(*c) for c in CONVEX]}
    for c in doc["cases"] + doc["convex"]:
        print(f'{c["name"]:24s} type {c["type"]} contacts {len(c["contacts"])}')
    with open(os.path.join(sys.argv[1], "flex_primitives.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


if __name__ == "__main__":
    main()
