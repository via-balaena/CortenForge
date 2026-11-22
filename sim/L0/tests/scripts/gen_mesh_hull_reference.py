#!/usr/bin/env python3
"""Generate sim-mjcf's mesh-hull golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_mesh_hull_reference.py \\
        sim/L0/mjcf/tests/assets/mesh_hull/mesh_hull.txt

It refuses any other interpreter, as gen_census_golden.py does.

Each case is one document, every mesh embedded (vertices, faces, or a
built-in), on a free body, which MuJoCo loads from its text with
MjModel.from_xml_string: its convex hull graph (`mesh_graph`), faces,
polygons and body 1's mass, or its refusal. The cases: convex meshes moved,
turned and scaled (MuJoCo makes the hull of the vertices as written),
`maxhullvert` (a mesh's, a class's, and a -1 written over a class's),
meshes written without faces (MuJoCo gives them the hull's),
each way a mesh comes to need a hull (a mesh or SDF geom naming it that
collides or is in a pair, in a body, a frame or the world, its type written
or a class's; a convex mesh) and each way it does not, and flat meshes qhull
refuses where a hull is needed. `mesh_hull.rs`'s test loads each document
and compares (the file's header says how).
"""
import math
import os
import struct
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402


def orient(verts, faces):
    c = [sum(v[k] for v in verts) / len(verts) for k in range(3)]
    res = []
    for a, b, d in faces:
        p, q, r = verts[a], verts[b], verts[d]
        u = [q[k] - p[k] for k in range(3)]; w = [r[k] - p[k] for k in range(3)]
        n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
        m = [(p[k] + q[k] + r[k]) / 3 - c[k] for k in range(3)]
        res.append((a, b, d) if sum(n[k] * m[k] for k in range(3)) > 0 else (a, d, b))
    return res


def vs(verts):
    return "  ".join(" ".join(repr(float(x)) for x in v) for v in verts)


def fs(faces):
    return "  ".join(" ".join(str(i) for i in f) for f in faces)


TET = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
BOX = [(x, y, z) for z in (0, 3) for y in (0, 2) for x in (0, 1)]
HEX = [(0, 0, 0), (1.1, 0, 0), (0, 0.9, 0), (1, 1, 0.1), (0, 0, 1.3), (1.2, 0.1, 1), (0.1, 1, 1.1), (1, 1.05, 0.95)]
CUBE_F = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2),
          (1, 3, 7), (1, 7, 5)]
LP = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
L_V = [(x, y, 0) for x, y in LP] + [(x, y, 1) for x, y in LP]
L_F = [(0, 2, 1), (0, 3, 2), (0, 4, 3), (0, 5, 4), (6, 7, 8), (6, 8, 9), (6, 9, 10), (6, 10, 11)]
for i in range(6):
    j = (i + 1) % 6
    L_F += [(i, j, j + 6), (i, j + 6, i + 6)]
PHI = (1 + 5 ** 0.5) / 2
ICO = [(0, s1, s2 * PHI) for s1 in (-1, 1) for s2 in (-1, 1)]
ICO = ICO + [(y, z, x) for x, y, z in ICO] + [(z, x, y) for x, y, z in ICO]
ICO_F = orient(ICO, [(a, b, d) for a in range(12) for b in range(a + 1, 12) for d in range(b + 1, 12)
                     if all(abs(math.dist(ICO[i], ICO[j]) - 2) < 1e-9 for i, j in ((a, b), (b, d), (a, d)))])
assert len(ICO_F) == 20
SPH = [(0.0, 0.0, 1.0), (0.0, 0.0, -1.0)] + [
    (math.sin(math.pi * i / 9) * math.cos(2 * math.pi * j / 8), math.sin(math.pi * i / 9) * math.sin(2 * math.pi * j / 8),
     math.cos(math.pi * i / 9)) for i in range(1, 9) for j in range(8)]
PLATE = [(0, 0, 0), (2, 0, 0), (0, 1, 0), (2, 1, 0)]
PLATE_F = [(0, 1, 3), (0, 3, 2)]
TURN = ' refquat="0.9 0.1 0.3 0.2"'
SCALE = ' scale="1 2 0.5"'
MOVE = ' refpos="0.3 0.1 -0.2"'
NOCOL = ' contype="0" conaffinity="0"'


def mesh(verts=None, faces=None, attrs=""):
    v = f' vertex="{vs(verts)}"' if verts is not None else ""
    f = f' face="{fs(faces)}"' if faces else ""
    return f'<mesh name="m"{v}{f}{attrs}/>'


def doc(asset, geoms, extra="", top=""):
    return (f'<mujoco>{top}<asset>{asset}</asset><worldbody><body name="b" pos="0 0 1"><freejoint/>{geoms}'
            f'</body></worldbody>{extra}</mujoco>')


G = '<geom type="mesh" mesh="m"{}/>'
SIX = ('<default><default class="six"><mesh maxhullvert="6"/>'
       '<default class="unlimited"><mesh maxhullvert="-1"/></default></default></default>')
PAIR = '<contact><pair geom1="g" geom2="s"/></contact>'
SPHERE = '<geom name="s" type="sphere" size="0.1" pos="0 0 2"' + NOCOL + '/>'
CASES = {
    "hex_convex": doc(mesh(HEX, CUBE_F, ' inertia="convex"'), G.format("")),
    "hex_convex_moved": doc(mesh(HEX, CUBE_F, MOVE + ' inertia="convex"'), G.format("")),
    "hex_convex_turned": doc(mesh(HEX, CUBE_F, TURN + ' inertia="convex"'), G.format("")),
    "hex_convex_scaled": doc(mesh(HEX, CUBE_F, SCALE + ' inertia="convex"'), G.format("")),
    "hex_convex_moved_turned_scaled": doc(mesh(HEX, CUBE_F, MOVE + TURN + SCALE + ' inertia="convex"'), G.format("")),
    "hex_convex_mirrored": doc(mesh(HEX, CUBE_F, ' scale="-1 1 1" inertia="convex"'), G.format("")),
    "hex_legacy_turned_scaled": doc(mesh(HEX, CUBE_F, TURN + SCALE), G.format("")),
    "lprism_convex_turned": doc(mesh(L_V, L_F, TURN + ' inertia="convex"'), G.format("")),
    "lprism_convex_moved_turned_scaled": doc(mesh(L_V, L_F, MOVE + TURN + SCALE + ' inertia="convex"'), G.format("")),
    "ico_convex_turned": doc(mesh(ICO, ICO_F, TURN + ' inertia="convex"'), G.format("")),
    "sphere66_no_faces_maxhullvert8": doc(mesh(SPH, None, ' maxhullvert="8" inertia="convex"'), G.format("")),
    "ico_convex_maxhullvert6": doc(mesh(ICO, ICO_F, ' maxhullvert="6" inertia="convex"'), G.format("")),
    "ico_legacy_maxhullvert6_turned": doc(mesh(ICO, ICO_F, ' maxhullvert="6"' + TURN), G.format("")),
    "lprism_convex_maxhullvert5_and_a_fitted_box": doc(mesh(L_V, L_F, ' maxhullvert="5" inertia="convex"'),
                                                        G.format("") + '<geom type="box" mesh="m"/>'),
    "ico_no_faces_class_maxhullvert6": doc(mesh(ICO, None, ' class="six"'), G.format(""), top=SIX),
    "ico_no_faces_class_maxhullvert6_mesh_minus_one": doc(mesh(ICO, None, ' class="six" maxhullvert="-1"'), G.format(""),
                                                           top=SIX),
    "ico_no_faces_child_class_minus_one": doc(mesh(ICO, None, ' class="unlimited"'), G.format(""), top=SIX),
    "tet_no_faces": doc(mesh(TET), G.format("")),
    "box_no_faces": doc(mesh(BOX), G.format("")),
    "box_and_an_inner_vertex_no_faces": doc(mesh(BOX + [(0.5, 1, 1.5)]), G.format("")),
    "box_no_faces_no_collision": doc(mesh(BOX), G.format(NOCOL)),
    "hex_no_faces_turned_scaled_convex": doc(mesh(HEX, None, TURN + SCALE + ' inertia="convex"'), G.format("")),
    "hex_no_faces_exact": doc(mesh(HEX, None, ' inertia="exact"'), G.format("")),
    "hex_no_faces_shell": doc(mesh(HEX, None, ' inertia="shell"'), G.format("")),
    "ico_no_faces_only_a_fitted_box": doc(mesh(ICO), '<geom type="box" mesh="m"/>'),
    "builtin_cone": doc('<mesh name="m" builtin="cone" params="6 0.5"/>', G.format("")),
    "builtin_wedge": doc('<mesh name="m" builtin="wedge" params="3 3 60 60 0"/>', G.format("")),
    "lprism_collides": doc(mesh(L_V, L_F), G.format("")),
    "lprism_no_collision": doc(mesh(L_V, L_F), G.format(NOCOL)),
    "lprism_no_collision_in_a_pair": doc(mesh(L_V, L_F), G.format(NOCOL + ' name="g"') + SPHERE, extra=PAIR),
    "lprism_only_a_fitted_box": doc(mesh(L_V, L_F), '<geom type="box" mesh="m"/>'),
    "lprism_no_collision_convex": doc(mesh(L_V, L_F, ' inertia="convex"'), G.format(NOCOL)),
    "lprism_unused": doc(mesh(L_V, L_F), '<geom type="sphere" size="0.1"/>'),
    "lprism_contype_only": doc(mesh(L_V, L_F), G.format(' contype="1" conaffinity="0"')),
    "lprism_conaffinity_only": doc(mesh(L_V, L_F), G.format(' contype="0" conaffinity="1"')),
    "lprism_class_no_collision": doc(mesh(L_V, L_F), G.format(' class="v"'),
                                     top='<default><default class="v"><geom contype="0" conaffinity="0"/></default></default>'),
    "lprism_sdf_geom": doc(mesh(L_V, L_F), '<geom type="sdf" mesh="m"/>'),
    "lprism_sdf_geom_no_collision": doc(mesh(L_V, L_F), '<geom type="sdf" mesh="m"' + NOCOL + '/>'),
    "plate_collides_shell": doc(mesh(PLATE, PLATE_F, ' inertia="shell"'), G.format("")),
    "plate_no_collision_convex": doc(mesh(PLATE, PLATE_F, ' inertia="convex"'), G.format(NOCOL)),
    "plate_no_collision_in_a_pair_shell": doc(mesh(PLATE, PLATE_F, ' inertia="shell"'),
                                              G.format(NOCOL + ' name="g"') + SPHERE, extra=PAIR),
    "builtin_plate_collides": doc('<mesh name="m" builtin="plate" params="3 2"/>', G.format("")),
    "builtin_plate_no_collision": doc('<mesh name="m" builtin="plate" params="3 2"/>', G.format(NOCOL)),
    "lprism_collides_in_a_frame": doc(mesh(L_V, L_F), '<frame pos="0.1 0 0">' + G.format("") + '</frame>'),
    "lprism_collides_on_the_world": (f'<mujoco><asset>{mesh(L_V, L_F)}</asset><worldbody>{G.format("")}'
                                     '<body name="b" pos="0 0 3"><freejoint/><geom size="0.1"/></body>'
                                     '</worldbody></mujoco>'),
    "lprism_mesh_type_from_a_class": doc(mesh(L_V, L_F), '<geom class="t" mesh="m"/>',
                                         top='<default><default class="t"><geom type="mesh"/></default></default>'),
}
HEADER = """# MuJoCo 3.5.0 (built from its 3.5.0 tag with -ffp-contract=off, macOS arm64) loading each document with
# MjModel.from_xml_string: `case NAME`, `xml` the document, then `err` MuJoCo's refusal, or per mesh
# `mesh I NVERT NFACE`, `graph` its mesh_graph (`none` where it has none), `face` its faces, `poly` per polygon
# its normal as three doubles' bits and its vertices, and `mass` body 1's mass as a double's bits.
# Written by sim/L0/tests/scripts/gen_mesh_hull_reference.py; the oracle's libraries' sha256 (without UUID):
# {oracle}"""


def case_lines(name, xml):
    lines = [f"case {name}", f"xml {xml}"]
    try:
        m = mujoco.MjModel.from_xml_string(xml)
    except Exception as e:
        return lines + ["err " + str(e).split("\n")[0].replace("Error: ", "")]
    for i in range(m.nmesh):
        nv, nf = int(m.mesh_vertnum[i]), int(m.mesh_facenum[i])
        lines.append(f"mesh {i} {nv} {nf}")
        ga = int(m.mesh_graphadr[i])
        if ga < 0:
            lines.append("graph none")
        else:
            gv, gf = int(m.mesh_graph[ga]), int(m.mesh_graph[ga + 1])
            lines.append("graph " + " ".join(str(int(x)) for x in m.mesh_graph[ga:ga + 2 + 3 * gv + 6 * gf]))
        fa = int(m.mesh_faceadr[i])
        lines.append("face " + " ".join(str(int(x)) for x in m.mesh_face[fa:fa + nf].reshape(-1)))
        pa = int(m.mesh_polyadr[i])
        for p in range(pa, pa + int(m.mesh_polynum[i])):
            va, vn = int(m.mesh_polyvertadr[p]), int(m.mesh_polyvertnum[p])
            normal = " ".join(struct.pack(">d", float(x)).hex() for x in m.mesh_polynormal[p])
            lines.append(f"poly {normal} " + " ".join(str(int(x)) for x in m.mesh_polyvert[va:va + vn]))
    return lines + ["mass " + struct.pack(">d", float(m.body_mass[1])).hex()]


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    out = [HEADER.format(oracle=marker["libraries_sha256_without_uuid"])]
    for name, xml in CASES.items():
        out += case_lines(name, xml)
    with open(sys.argv[1], "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print(f"{len(CASES)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
