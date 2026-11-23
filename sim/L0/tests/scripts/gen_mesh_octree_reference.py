#!/usr/bin/env python3
"""Generate sim-mjcf's mesh-octree golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_mesh_octree_reference.py \\
        sim/L0/mjcf/tests/assets/mesh_octree/mesh_octree.txt

It refuses any other interpreter, as gen_census_golden.py does.

Each case is one document, on a free body, which MuJoCo loads: from its
text with MjModel.from_xml_string where every mesh is embedded (vertices,
faces, or a built-in), or, where a case names mesh files, from a directory
holding the document and those files, which the golden carries byte for
byte. Per mesh, its octree (`mesh_octnum`, and a hash of each of `oct_aabb`,
`oct_child`, `oct_depth`, `oct_coeff` over the mesh's nodes) or none, and for
a file mesh its convex hull graph (`mesh_graph`); or MuJoCo's refusal. The
cases: an SDF geom over meshes of each shape (a box, an L prism, an
icosahedron, meshes written without faces, built-ins), moved, turned, scaled
and mirrored, tiny and huge, under each inertia mode, an open mesh, a flat
mesh and two parallel flat ones, a box whose faces are wound at random; each way
a mesh comes to have an octree (an SDF geom naming it, colliding or not, its
type a class's, in the world, beside a mesh geom, read before discardvisual
drops a visual one) and each way it does not; and an STL, an OBJ and an MSH
file. `mesh_octree.rs`'s test loads each document and compares (the file's
header says how).
"""
import math
import os
import random
import struct
import sys
import tempfile

import mujoco
import numpy as np

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
TET_F = orient(TET, [(0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)])
BOX = [(x, y, z) for z in (0, 3) for y in (0, 2) for x in (0, 1)]
CUBE = [(x, y, z) for z in (-.1, .1) for y in (-.1, .1) for x in (-.1, .1)]
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
# Two plates a unit apart, wound alike: a corner midway between them is as far from each, and which face
# TriangleMeshDistance's query meets first gives the corner's sign.
TWO_PLATES = [(x, y, -0.5) for x, y, _ in PLATE] + [(x, y, 0.5) for x, y, _ in PLATE]
TWO_PLATES_F = PLATE_F + [(a + 4, b + 4, d + 4) for a, b, d in PLATE_F]


def quartered_box(extents, seed):
    """A box centred on the origin, each face split into four squares of two triangles, each triangle wound
    one way or the other at random (Python's `random.Random(seed).random()`, whose sequence for a seed does not
    change between Python versions)."""
    rng = random.Random(seed)
    half = [e / 2 for e in extents]
    verts, index, faces = [], {}, []

    def vid(p):
        if p not in index:
            index[p] = len(verts)
            verts.append(p)
        return index[p]

    for axis in range(3):
        u, w = (axis + 1) % 3, (axis + 2) % 3
        for side in (-1, 1):
            for j in range(2):
                for i in range(2):
                    def corner(a, b):
                        p = [0.0, 0.0, 0.0]
                        p[axis], p[u], p[w] = side * half[axis], half[u] * (a - 1), half[w] * (b - 1)
                        return vid(tuple(p))
                    q = corner(i, j), corner(i + 1, j), corner(i + 1, j + 1), corner(i, j + 1)
                    for f in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
                        faces.append(f if (side > 0) == (rng.random() >= 0.5) else (f[0], f[2], f[1]))
    return verts, faces


TURN = ' refquat="0.9 0.1 0.3 0.2"'
SCALE = ' scale="1 2 0.5"'
MOVE = ' refpos="0.3 0.1 -0.2"'
NOCOL = ' contype="0" conaffinity="0"'


def mesh(verts=None, faces=None, attrs="", name="m"):
    v = f' vertex="{vs(verts)}"' if verts is not None else ""
    f = f' face="{fs(faces)}"' if faces else ""
    return f'<mesh name="{name}"{v}{f}{attrs}/>'


def doc(asset, geoms, extra="", top=""):
    return (f'<mujoco>{top}<asset>{asset}</asset><worldbody><body name="b" pos="0 0 1"><freejoint/>{geoms}'
            f'</body></worldbody>{extra}</mujoco>')


def world(asset, geoms):
    """A document with its geoms on the world body, which needs no mass or inertia from them."""
    return f'<mujoco><asset>{asset}</asset><worldbody>{geoms}</worldbody></mujoco>'


def stl(verts, faces):
    """A binary STL of the faces, each triangle its own three vertices, as STL files write them."""
    out = bytearray(80) + struct.pack("<I", len(faces))
    for f in faces:
        out += struct.pack("<3f", 0, 0, 0)
        for i in f:
            out += struct.pack("<3f", *verts[i])
        out += b"\0\0"
    return bytes(out)


def obj(shapes):
    """An OBJ of shapes, each `o` with its vertices and faces (indices counted over the file)."""
    lines, base = [], 0
    for k, (verts, faces) in enumerate(shapes):
        lines.append(f"o shape{k}")
        lines += [f"v {x!r} {y!r} {z!r}" for x, y, z in verts]
        lines += ["f " + " ".join(str(base + i + 1) for i in f) for f in faces]
        base += len(verts)
    return ("\n".join(lines) + "\n").encode()


def msh(verts, faces):
    """MuJoCo's binary MSH: four counts (vertices, normals, texture coordinates, faces), then the data."""
    out = struct.pack("<4i", len(verts), 0, 0, len(faces))
    for v in verts:
        out += struct.pack("<3f", *v)
    for f in faces:
        out += struct.pack("<3i", *f)
    return out


S = '<geom type="sdf" mesh="m"{}/>'
M = '<geom type="mesh" mesh="m"{}/>'
DISCARD = '<compiler discardvisual="true"/>'
CASES = {
    "box": doc(mesh(BOX, CUBE_F), S.format("")),
    "box_no_collision": doc(mesh(BOX, CUBE_F), S.format(NOCOL)),
    "cube_no_faces": doc(mesh(CUBE), S.format("")),
    "lprism": doc(mesh(L_V, L_F), S.format("")),
    "lprism_shell": doc(mesh(L_V, L_F, ' inertia="shell"'), S.format("")),
    "lprism_convex": doc(mesh(L_V, L_F, ' inertia="convex"'), S.format("")),
    "lprism_legacy": doc(mesh(L_V, L_F, ' inertia="legacy"'), S.format("")),
    "ico": doc(mesh(ICO, ICO_F), S.format("")),
    "sphere66_no_faces": doc(mesh(SPH), S.format("")),
    "hex_no_faces": doc(mesh(HEX), S.format("")),
    "hex_moved_turned_scaled": doc(mesh(HEX, CUBE_F, MOVE + TURN + SCALE), S.format("")),
    "hex_mirrored": doc(mesh(HEX, CUBE_F, ' scale="-1 1 1"'), S.format("")),
    "box_turned": doc(mesh(BOX, CUBE_F, TURN), S.format("")),
    "box_tiny": doc(mesh(BOX, CUBE_F, ' scale="1e-3 1e-3 1e-3"'), S.format("")),
    "box_huge": doc(mesh(BOX, CUBE_F, ' scale="1e3 1e3 1e3"'), S.format("")),
    "open_box_shell": doc(mesh(BOX, CUBE_F[2:], ' inertia="shell"'), S.format("")),
    "builtin_sphere": doc('<mesh name="m" builtin="sphere" params="2"/>', S.format("")),
    "builtin_cone": doc('<mesh name="m" builtin="cone" params="6 0.5"/>', S.format("")),
    "builtin_supertorus": doc('<mesh name="m" builtin="supertorus" params="24 0.3 1 1"/>', S.format("")),
    "plate_collides_shell": doc(mesh(PLATE, PLATE_F, ' inertia="shell"'), S.format("")),
    # A flat mesh on the world, colliding with nothing (a colliding one needs a hull, which qhull refuses a
    # flat mesh): each corner in its plane is where the side of a face's pseudonormal is exactly 0.
    "plate_on_the_world_colliding_with_nothing": world(mesh(PLATE, PLATE_F, ' inertia="shell"'), S.format(NOCOL)),
    "two_plates_wound_alike_on_the_world": world(mesh(TWO_PLATES, TWO_PLATES_F, ' inertia="shell"'),
                                                 S.format(NOCOL)),
    # Faces wound either way: TriangleMeshDistance's sort of the faces, and its query's order among faces at one
    # distance, decide a corner's sign.
    "quartered_box_wound_at_random_on_the_world": world(mesh(*quartered_box((1, 2, 3), 3), ' inertia="legacy"'),
                                                        S.format("")),
    "lprism_mesh_and_sdf_geoms": doc(mesh(L_V, L_F), M.format("") + S.format("")),
    "lprism_two_sdf_geoms": doc(mesh(L_V, L_F), S.format("") + S.format(' pos="3 0 0"')),
    "two_meshes_the_second_an_sdf_geoms": doc(mesh(BOX, CUBE_F, name="a") + mesh(L_V, L_F),
                                              '<geom type="mesh" mesh="a"/>' + S.format("")),
    "two_meshes_both_sdf_geoms": doc(mesh(ICO, ICO_F, name="a") + mesh(L_V, L_F),
                                     '<geom type="sdf" mesh="a"/>' + S.format("")),
    "lprism_sdf_type_from_a_class": doc(mesh(L_V, L_F), '<geom class="s" mesh="m"/>',
                                        top='<default><default class="s"><geom type="sdf"/></default></default>'),
    "lprism_sdf_on_the_world": (f'<mujoco><asset>{mesh(L_V, L_F)}</asset><worldbody>{S.format("")}'
                                '<body name="b" pos="0 0 3"><freejoint/><geom size="0.1"/></body>'
                                '</worldbody></mujoco>'),
    "lprism_discardvisual_a_visual_sdf_geom_and_a_mesh_geom": doc(mesh(L_V, L_F), S.format(NOCOL) + M.format(""),
                                                                   top=DISCARD),
    "lprism_discardvisual_only_a_visual_sdf_geom": doc(mesh(L_V, L_F),
                                                       S.format(NOCOL) + '<geom type="sphere" size="0.1"/>',
                                                       top=DISCARD),
    "lprism_a_visual_sdf_geom_kept": doc(mesh(L_V, L_F), S.format(NOCOL)),
    "lprism_only_a_mesh_geom": doc(mesh(L_V, L_F), M.format("")),
    "lprism_only_a_fitted_box": doc(mesh(L_V, L_F), '<geom type="box" mesh="m"/>'),
}
# Cases whose meshes are files: the document, and each file's name and bytes.
FILE_CASES = {
    "ico_stl_file": (doc('<mesh name="m" file="ico.stl"/>', S.format("")), {"ico.stl": stl(ICO, ICO_F)}),
    "obj_file_two_shapes_the_first_taken": (doc('<mesh name="m" file="two.obj"/>', S.format("")),
                                            {"two.obj": obj([(L_V, L_F), (ICO, ICO_F)])}),
    "lprism_msh_file_scaled": (doc('<mesh name="m" file="lprism.msh" scale="2 1 0.5"/>', S.format("")),
                               {"lprism.msh": msh(L_V, L_F)}),
}
HEADER = """# MuJoCo 3.5.0 (built from its 3.5.0 tag with -ffp-contract=off, macOS arm64) loading each document:
# `case NAME`, `xml` the document, `file NAME HEX` each mesh file it names (its bytes; MuJoCo loaded the
# document from a directory holding them), then `err` MuJoCo's refusal, or `meshes N` and per mesh
# `mesh I NVERT NFACE`, for a file case `graph` its mesh_graph (`none` where it has none), and `octree none` or
# `octree NNODE AABB CHILD DEPTH COEFF ABSCOEFF NEGATIVE`: its mesh_octnum, the hash of its nodes' oct_aabb,
# oct_child, oct_depth, oct_coeff and the coefficients' magnitudes, and how many coefficients are negative. A
# hash is over 64-bit words (a double's bits; an int sign-extended), from 0: each word xored in, then
# splitmix64's finalizer (z ^= z >> 30, z *= 0xbf58476d1ce4e5b9, z ^= z >> 27, z *= 0x94d049bb133111eb,
# z ^= z >> 31), modulo 2^64.
# Written by sim/L0/tests/scripts/gen_mesh_octree_reference.py; the oracle's libraries' sha256 (without UUID):
# {oracle}"""


def mix_hash(words):
    """A 64-bit hash of a list of 64-bit words: from 0, each word xored in, then splitmix64's finalizer, whose
    shifts carry every bit into every other (an FNV step over whole words cannot move a high bit down: a sign
    bit flipped twice would leave the hash as it was)."""
    m = 0xFFFFFFFFFFFFFFFF
    h = 0
    for w in words:
        z = h ^ w
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & m
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & m
        h = z ^ (z >> 31)
    return f"{h:016x}"


def doubles(a):
    return np.ascontiguousarray(a, dtype="<f8").reshape(-1).view("<u8").tolist()


def ints(a):
    return np.ascontiguousarray(a, dtype="<i8").reshape(-1).view("<u8").tolist()


def model_lines(m, graphs):
    lines = [f"meshes {m.nmesh}"]
    for i in range(m.nmesh):
        lines.append(f"mesh {i} {int(m.mesh_vertnum[i])} {int(m.mesh_facenum[i])}")
        if graphs:
            ga = int(m.mesh_graphadr[i])
            if ga < 0:
                lines.append("graph none")
            else:
                gv, gf = int(m.mesh_graph[ga]), int(m.mesh_graph[ga + 1])
                lines.append("graph " + " ".join(str(int(x)) for x in m.mesh_graph[ga:ga + 2 + 3 * gv + 6 * gf]))
        n, a = int(m.mesh_octnum[i]), int(m.mesh_octadr[i])
        if n == 0:
            lines.append("octree none")
            continue
        s = slice(a, a + n)
        coeff = m.oct_coeff[s]
        lines.append(f"octree {n} {mix_hash(doubles(m.oct_aabb[s]))} {mix_hash(ints(m.oct_child[s]))} "
                     f"{mix_hash(ints(m.oct_depth[s]))} {mix_hash(doubles(coeff))} "
                     f"{mix_hash(doubles(np.abs(coeff)))} {int((coeff < 0).sum())}")
    return lines


def refusal(e):
    return "err " + str(e).split("\n")[0].replace("Error: ", "")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    out = [HEADER.format(oracle=marker["libraries_sha256_without_uuid"])]
    for name, xml in CASES.items():
        out += [f"case {name}", f"xml {xml}"]
        try:
            out += model_lines(mujoco.MjModel.from_xml_string(xml), graphs=False)
        except Exception as e:
            out.append(refusal(e))
    for name, (xml, files) in FILE_CASES.items():
        out += [f"case {name}", f"xml {xml}"] + [f"file {f} {data.hex()}" for f, data in files.items()]
        with tempfile.TemporaryDirectory() as d:
            for f, data in files.items():
                with open(os.path.join(d, f), "wb") as h:
                    h.write(data)
            path = os.path.join(d, "doc.xml")
            with open(path, "w") as h:
                h.write(xml)
            try:
                out += model_lines(mujoco.MjModel.from_xml_path(path), graphs=True)
            except Exception as e:
                out.append(refusal(e))
    with open(sys.argv[1], "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print(f"{len(CASES) + len(FILE_CASES)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
