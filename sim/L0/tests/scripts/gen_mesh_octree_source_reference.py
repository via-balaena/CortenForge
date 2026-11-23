#!/usr/bin/env python3
"""Generate sim-core's octree golden from MuJoCo 3.5.0's own octree source, compiled.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it and keeps the source it builds from):

    <workdir>/venv/bin/python -I -B sim/L0/tests/scripts/gen_mesh_octree_source_reference.py \\
        sim/L0/core/tests/data/mesh_octree_source.txt

It refuses any other interpreter, as gen_census_golden.py does, a source tree at another commit than the oracle's
or with changes, and a TriangleMeshDistance at another commit than MuJoCo 3.5.0's or changed otherwise than by
MuJoCo's own patch (`TMD_PATCH_SHA256`, the sha256 of `git diff` there). It copies MuJoCo's octree line for line
(`mjCOctree`: `user_objects.h:217-317`, `user_objects.cc:549-1000`), the coefficients `mjCMesh::TryCompile` gives its
vertices (`user_mesh.cc:783-828`, inside a struct that names the mesh's `vert_`, `face_` and `octree_`), and the four
helpers they call (`mjuu_dot3`, `mjuu_crossvec`, `mjuu_rotVecQuat` with `mjuu_copyvec`, `mju_max`), each checked
where it begins and ends, into a C++ program built against the TriangleMeshDistance header MuJoCo's build fetched,
with `/usr/bin/c++ -std=c++17 -O2 -ffp-contract=off` (the oracle's flag), and runs it on each case: a mesh's
vertices, faces and box (min, max) as sim-core's `make_octree` takes them, with no loader between, so no frame
moves a vertex off the grid it is written on.

The cases are `cases()` below, each named for what it holds, the meshes where an octree's corners tie (on grid
planes, at signed zeros, faces wound at random or duplicated) as well as plain ones; `FOUND` holds meshes a search
found, each because a change to the port that every other case passed changes its octree. The vertices are written as
built here (Python floats), the faces as indices.

The file is a `hash` line per point of `HASH_POINTS` (its coordinates' bits, then MuJoCo's `hash<Point>` of it, in
hex), then one block per case, '#' lines aside:

    hash <x y z as a double's bits in hex> <hash in hex>

    case NAME
    vertices <x y z as a double's bits in hex>...
    faces <a b c>...
    box <min x y z, max x y z as a double's bits in hex>
    octree NNODE AABB CHILD DEPTH COEFF

NNODE is the octree's node count and AABB, CHILD, DEPTH, COEFF the hashes of its nodes' boxes (centre, half sizes),
children, depths and coefficients, as gen_mesh_octree_reference.py hashes them (its `mix_hash`).
"""
import hashlib
import math
import os
import random
import struct
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402
from gen_mesh_octree_reference import mix_hash  # noqa: E402

# Meshes a search found (RL-9d4 review FA): the vertices as found, the faces, each named for what it holds.
FOUND = {
    "a_face_along_x_touching_a_node_s_corner": (
        [(0.09920441922915929, 0.22767018752819249, 0.730396428327261), (0.5501651539476968, 0.22767018752819249, 0.730396428327261), (0.4256719685644904, 0.4833709403051226, 0.5474738710312795), (-0.9090909090909091, -0.9090909090909091, -0.9090909090909091), (-0.9080909090909091, -0.9090909090909091, -0.9090909090909091), (-0.9090909090909091, -0.9080909090909091, -0.9090909090909091), (0.9090909090909091, 0.9090909090909091, 0.9090909090909091), (0.9080909090909091, 0.9090909090909091, 0.9090909090909091), (0.9090909090909091, 0.9080909090909091, 0.9090909090909091)],
        [(0, 1, 2), (3, 4, 5), (6, 7, 8)]),
    "a_face_and_its_mirror_tied_at_the_origin": (
        [(-1.1643518675434614, 0.053428061579287735, -0.6476321910006855), (-2.34603742445167, -0.5795357220704956, -1.5778259448329606), (-2.529448495964222, 0.8032548307403732, -1.1340017396054618), (1.1643518675434614, -0.053428061579287735, 0.6476321910006855), (2.34603742445167, 0.5795357220704956, 1.5778259448329606), (2.529448495964222, -0.8032548307403732, 1.1340017396054618)],
        [(0, 1, 2), (3, 4, 5)]),
    "a_flat_fan_at_negative_zero": (
        [(-0.0, 0.16850358594039783, 0.8084035416955502), (-0.0, 0.3639642732699333, 0.857891202400034), (-0.0, 0.7128011327935113, 0.9819792897376303), (-0.0, 0.34254708432503644, -0.6738007560578605), (-0.0, 0.7212750662325365, 0.9292658946181227), (-0.0, 0.8093919690244733, 0.13821500694864697), (-0.0, 0.42763404034839847, -0.5777500326488034), (-0.0, 0.6632158605467084, 0.1470647047025695), (-0.0, -0.4300850760275896, -0.8730788457095413), (-0.0, 0.7078849768453603, 0.9796120298431625), (-0.0, -0.8229638137805433, 0.6011906425150038), (-0.0, -0.17907634530818228, -0.6984692510943808)],
        [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 5), (0, 5, 6), (0, 6, 7), (0, 7, 8), (0, 8, 9), (0, 9, 10), (0, 10, 11)]),
}

# The commit MuJoCo 3.5.0 fetches TriangleMeshDistance at (`cmake/MujocoDependencies.cmake`), and the sha256 of the
# `git diff` its build leaves there (MuJoCo's patch: `for (const auto& edge_count : edges_count)`).
TMD_COMMIT = "2cb643de"
TMD_PATCH_SHA256 = "f31cc72b77d13db69a4c30160e6bb7dd7f0d47d8aeb86647bc5e696fe762524b"
FLAGS = ["-std=c++17", "-O2", "-ffp-contract=off"]


def lines_of(path):
    with open(path) as f:
        return f.read().split("\n")


def line_range(path, first, last, starts, ends):
    """Lines `first..=last` (1-based) of `path`, whose first must start with `starts` and last with `ends`, and
    whose braces must balance."""
    lines = lines_of(path)[first - 1:last]
    text = "\n".join(lines)
    if not lines[0].strip().startswith(starts) or not lines[-1].startswith(ends) or \
            text.count("{") != text.count("}"):
        sys.exit(f"{path}:{first}-{last} is not the code this copies: {lines[0]!r} ... {lines[-1]!r}")
    return text


def function(path, signature):
    """The function whose definition line starts with `signature`, through the first `}` at column 0 after it."""
    lines = lines_of(path)
    starts = [i for i, l in enumerate(lines) if l.startswith(signature)]
    if len(starts) != 1:
        sys.exit(f"{path}: {len(starts)} definitions start {signature!r}")
    i = starts[0]
    if i > 0 and lines[i - 1].startswith("template"):
        i -= 1
    end = next(j for j in range(starts[0], len(lines)) if lines[j] == "}")
    return "\n".join(lines[i:end + 1])


MAIN = r"""
struct mjCError {
  mjCError(const void*, const char* message) : message(message) {}
  const char* message;
};

// The mesh `mjCMesh::TryCompile`'s coefficient block reads.
struct Mesh {
  std::vector<double> vert_;
  std::vector<int> face_;
  mjCOctree octree_;
  int nvert() const { return vert_.size() / 3; }
  int nface() const { return face_.size() / 3; }
  void Coefficients() {
%(coefficients)s
  }
};

int main(int argc, char** argv) {
  if (argc == 2) {  // `hash`: each stdin line three doubles' bits in hex, each answered by MuJoCo's hash<Point>
    unsigned long long bits[3];
    while (std::scanf("%%llx %%llx %%llx", &bits[0], &bits[1], &bits[2]) == 3) {
      Point point;
      std::memcpy(point.p.data(), bits, sizeof bits);
      std::printf("%%016zx\n", std::hash<Point>()(point));
    }
    return 0;
  }
  FILE* f = std::fopen(argv[1], "rb");
  int nv, nf;
  if (std::fread(&nv, 4, 1, f) != 1) return 2;
  Mesh mesh;
  mesh.vert_.resize(3 * nv);
  if (std::fread(mesh.vert_.data(), 8, 3 * nv, f) != size_t(3 * nv)) return 2;
  if (std::fread(&nf, 4, 1, f) != 1) return 2;
  mesh.face_.resize(3 * nf);
  if (std::fread(mesh.face_.data(), 4, 3 * nf, f) != size_t(3 * nf)) return 2;
  double aamm[6];
  if (std::fread(aamm, 8, 6, f) != 6) return 2;
  std::fclose(f);
  // `mjCMesh::TryCompile` (`user_mesh.cc:778-779`), then its coefficients.
  mesh.octree_.SetFace(mesh.vert_, mesh.face_);
  mesh.octree_.CreateOctree(aamm);
  try {
    mesh.Coefficients();
  } catch (const mjCError& e) {
    std::printf("err %%s\n", e.message);
    return 0;
  }
  int n = mesh.octree_.NumNodes();
  std::vector<double> aabb(6 * n), coeff(8 * n);
  std::vector<int> child(8 * n), depth(n);
  mesh.octree_.CopyAabb(aabb.data());
  mesh.octree_.CopyChild(child.data());
  mesh.octree_.CopyLevel(depth.data());
  mesh.octree_.CopyCoeff(coeff.data());
  FILE* g = std::fopen(argv[2], "wb");
  std::fwrite(&n, 4, 1, g);
  std::fwrite(aabb.data(), 8, 6 * n, g);
  std::fwrite(child.data(), 4, 8 * n, g);
  std::fwrite(depth.data(), 4, n, g);
  std::fwrite(coeff.data(), 8, 8 * n, g);
  std::fclose(g);
  std::printf("ok %%ld\n", (long)_LIBCPP_VERSION);
  return 0;
}
"""


def program(src, tmd):
    user = os.path.join(src, "src", "user")
    parts = [
        "#include <algorithm>", "#include <array>", "#include <cmath>", "#include <cstdio>", "#include <cstdlib>",
        "#include <cstring>",
        "#include <deque>", "#include <stdexcept>", "#include <unordered_map>", "#include <vector>",
        "#include <TriangleMeshDistance/include/tmd/TriangleMeshDistance.h>",
        "typedef double mjtNum;",
        function(os.path.join(user, "user_util.h"), "void mjuu_copyvec("),
        function(os.path.join(user, "user_util.cc"), "double mjuu_dot3("),
        function(os.path.join(user, "user_util.cc"), "void mjuu_crossvec("),
        function(os.path.join(user, "user_util.cc"), "void mjuu_rotVecQuat("),
        function(os.path.join(src, "src", "engine", "engine_util_misc.c"), "mjtNum mju_max("),
        line_range(os.path.join(user, "user_objects.h"), 217, 317, "//------------------------- class mjCOctree", "};"),
        line_range(os.path.join(user, "user_objects.cc"), 549, 1000, "void mjCOctree::CopyLevel(int* level) const {",
                   "//------------------------- class mjCDef implementation"),
        MAIN % {"coefficients": line_range(os.path.join(user, "user_mesh.cc"), 783, 828,
                                           "tmd::TriangleMeshDistance sdf(vert_.data(), nvert(), face_.data(), "
                                           "nface());", "      }")},
    ]
    return "\n\n".join(parts) + "\n"


def box_of(vs):
    """The box over `vs` (min, max), each from +-1e10, as MuJoCo's compiler makes a mesh's."""
    lo, hi = [1e10] * 3, [-1e10] * 3
    for v in vs:
        for k in range(3):
            if v[k] < lo[k]:
                lo[k] = v[k]
            if hi[k] < v[k]:
                hi[k] = v[k]
    return lo + hi


def cuboid(lo, hi):
    vs = [(x, y, z) for z in (lo[2], hi[2]) for y in (lo[1], hi[1]) for x in (lo[0], hi[0])]
    fs = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6),
          (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    return vs, fs


def centred(vs):
    c = [sum(v[k] for v in vs) / len(vs) for k in range(3)]
    return [tuple(v[k] - c[k] for k in range(3)) for v in vs]


def turned(v, a, b):
    ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
    x, y, z = v
    x, y = ca * x - sa * y, sa * x + ca * y
    y, z = cb * y - sb * z, sb * y + cb * z
    return (x, y, z)


def icospheres():
    """Icosahedra subdivided twice, each vertex's radius perturbed, the even ones on a 1/8 grid; the faces as the
    subdivision leaves them, which winds them inconsistently."""
    rng = random.Random(7)
    phi = (1 + 5 ** 0.5) / 2
    out = []
    for k in range(6):
        vs = [(0, s1, s2 * phi) for s1 in (-1, 1) for s2 in (-1, 1)]
        vs = vs + [(y, z, x) for x, y, z in vs] + [(z, x, y) for x, y, z in vs]
        fs = [(a, b, d) for a in range(12) for b in range(a + 1, 12) for d in range(b + 1, 12)
              if all(abs(math.dist(vs[i], vs[j]) - 2) < 1e-9 for i, j in ((a, b), (b, d), (a, d)))]
        for _ in range(2):
            mid, nf = {}, []

            def m(i, j):
                key = (min(i, j), max(i, j))
                if key not in mid:
                    p = [(vs[i][c] + vs[j][c]) / 2 for c in range(3)]
                    n = math.sqrt(sum(x * x for x in p))
                    vs.append(tuple(x / n * phi for x in p))
                    mid[key] = len(vs) - 1
                return mid[key]
            for a, b, d in fs:
                ab, bd, da = m(a, b), m(b, d), m(d, a)
                nf += [(a, ab, da), (b, bd, ab), (d, da, bd), (ab, bd, da)]
            fs = nf
        vs = [tuple(x * (1 + 0.1 * rng.random()) for x in v) for v in vs]
        if k % 2 == 0:
            vs = [tuple(round(x * 8) / 8 for x in v) for v in vs]
        out.append((f"icosphere{k}", vs, fs))
    return out


def notched(cx, cz):
    """The cube [-1, 1]^3 less the block x > cx, z > cz: an L in the xz plane extruded along y."""
    ell = [(-1, -1), (1, -1), (1, cz), (cx, cz), (cx, 1), (-1, 1)]
    vs = [(x, -1, z) for x, z in ell] + [(x, 1, z) for x, z in ell]
    n, fs = len(ell), []
    for i in range(1, n - 1):
        fs += [(0, i, i + 1), (n, n + i + 1, n + i)]
    for i in range(n):
        j = (i + 1) % n
        fs += [(i, i + n, j + n), (i, j + n, j)]
    return vs, fs


def grid(n, z, flip=False):
    vs = [(-1 + 2 * i / n, -1 + 2 * j / n, z) for j in range(n + 1) for i in range(n + 1)]
    fs = []
    for j in range(n):
        for i in range(n):
            a = j * (n + 1) + i
            b, c, d = a + 1, a + n + 1, a + n + 2
            fs += [(a, b, d), (a, d, c)] if not flip else [(a, d, b), (a, c, d)]
    return vs, fs


def slab(n):
    """Two grids a half apart, the lower reversed, their rims joined: a closed slab of many equal sort keys."""
    top_v, top_f = grid(n, 0.25)
    bot_v, bot_f = grid(n, -0.25, flip=True)
    off = len(top_v)
    vs, fs = top_v + bot_v, top_f + [(a + off, b + off, c + off) for a, b, c in bot_f]
    rim = [(i, 0) for i in range(n)] + [(n, j) for j in range(n)] + [(n - i, n) for i in range(n)] + \
        [(0, n - j) for j in range(n)]
    for k in range(len(rim)):
        (i0, j0), (i1, j1) = rim[k], rim[(k + 1) % len(rim)]
        a, b = j0 * (n + 1) + i0, j1 * (n + 1) + i1
        fs += [(a, a + off, b + off), (a, b + off, b)]
    return vs, fs


def quantised_sphere(rng, nu, nv):
    vs = [(0.0, 0.0, 1.0)]
    for j in range(1, nv):
        th = math.pi * j / nv
        for i in range(nu):
            ph, r = 2 * math.pi * i / nu, 1 + 0.3 * rng.random()
            p = (r * math.sin(th) * math.cos(ph), r * math.sin(th) * math.sin(ph), r * math.cos(th))
            vs.append(tuple(round(c * 4) / 4 for c in p))
    vs.append((0.0, 0.0, -1.0))
    fs = [(0, 1 + i, 1 + (i + 1) % nu) for i in range(nu)]
    for j in range(nv - 2):
        for i in range(nu):
            a, b = 1 + j * nu + i, 1 + j * nu + (i + 1) % nu
            fs += [(a, a + nu, b + nu), (a, b + nu, b)]
    last, base = len(vs) - 1, 1 + (nv - 2) * nu
    fs += [(last, base + (i + 1) % nu, base + i) for i in range(nu)]
    return vs, fs


def subdivided_cube(n, lo, hi):
    """A box each face of which is split into an n by n grid of cells of two triangles, wound outward."""
    vs, index, fs = [], {}, []

    def v(p):
        if p not in index:
            index[p] = len(vs)
            vs.append(p)
        return index[p]
    for axis in range(3):
        for side in (0, n):
            a, b = (axis + 1) % 3, (axis + 2) % 3
            for i in range(n):
                for j in range(n):
                    def corner(ii, jj):
                        g = [0, 0, 0]
                        g[axis], g[a], g[b] = side, ii, jj
                        return v(tuple(lo[k] + (hi[k] - lo[k]) * g[k] / n for k in range(3)))
                    p00, p10, p01, p11 = corner(i, j), corner(i + 1, j), corner(i, j + 1), corner(i + 1, j + 1)
                    tris = [(p00, p10, p11), (p00, p11, p01)]
                    if side == 0:
                        tris = [(x, z, y) for x, y, z in tris]
                    fs += tris
    return vs, fs


def scrambled(fs, rng, flip=0.0):
    """The faces each turned to start at a random vertex, wound the other way with probability `flip`, shuffled."""
    out = []
    for t in fs:
        r = rng.randrange(3)
        t = t[r:] + t[:r]
        if rng.random() < flip:
            t = (t[0], t[2], t[1])
        out.append(t)
    rng.shuffle(out)
    return out


def cases():
    out = []
    box = [(x, y, z) for z in (0, 3) for y in (0, 2) for x in (0, 1)]
    _, box_f = cuboid((0, 0, 0), (1, 1, 1))
    cube = [(x, y, z) for z in (-.1, .1) for y in (-.1, .1) for x in (-.1, .1)]
    ell = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
    lprism = [(x, y, 0) for x, y in ell] + [(x, y, 1) for x, y in ell]
    lprism_f = [(0, 2, 1), (0, 3, 2), (0, 4, 3), (0, 5, 4), (6, 7, 8), (6, 8, 9), (6, 9, 10), (6, 10, 11)]
    for i in range(6):
        j = (i + 1) % 6
        lprism_f += [(i, j, j + 6), (i, j + 6, i + 6)]
    out += [("box", box, box_f), ("small_cube", cube, box_f), ("lprism", lprism, lprism_f),
            ("box_centred", centred(box), box_f), ("lprism_centred", centred(lprism), lprism_f),
            ("box_turned", [turned(v, 0.3, 0.7) for v in centred(box)], box_f),
            ("lprism_turned", [turned(v, 1.1, -0.4) for v in centred(lprism)], lprism_f),
            ("open_box", centred(box), box_f[2:]),
            ("degenerate_face", centred(box) + [(0.0, 0.0, 0.0)], box_f + [(0, 0, 8)])]
    out += icospheres()
    vs, fs = cuboid((-1, -1, -1), (1, 1, 1))
    out += [("a_face_twice_once_reversed", vs, fs + [(fs[0][0], fs[0][2], fs[0][1])]),
            ("every_face_twice_reversed", vs, fs + [(a, c, b) for a, b, c in fs]),
            ("notched_on_the_root_centre", *notched(0.0, 0.0)),
            ("notched_on_finer_grid_planes", *notched(1.1 / 2, -1.1 / 4)),
            ("flat_grid", *grid(12, 0.0)),
            ("slab", *slab(12)),
            ("tiny_box", *cuboid((-5e-10, -5e-10, -5e-10), (5e-10, 5e-10, 5e-10))),
            ("thin_box", *cuboid((-1, -1, -5e-9), (1, 1, 5e-9))),
            ("small_box", *cuboid((-1e-7, -1e-7, -1e-7), (1e-7, 1e-7, 1e-7))),
            ("tetrahedron_through_grid_vertices", [(1, -1, 0), (-1, 1, 0), (0, 1, -1), (1, 1, 1)],
             [(0, 1, 2), (0, 3, 1), (1, 3, 2), (2, 3, 0)]),
            ("octahedron", [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)],
             [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)])]
    square = [(-1.0, -1.0, -0.0), (1.0, -1.0, -0.0), (1.0, 1.0, -0.0), (-1.0, 1.0, -0.0)]
    mixed = [(-1.0, -1.0, -0.0), (1.0, -1.0, 0.0), (1.0, 1.0, -0.0), (-1.0, 1.0, 0.0)]
    out += [("flat_square_at_negative_zero", square, [(0, 1, 2), (0, 2, 3)]),
            ("flat_square_zeros_mixed", mixed, [(0, 1, 2), (0, 2, 3)]),
            ("cube_from_negative_zero", *cuboid((-0.0, -0.0, -0.0), (1, 1, 1)))]
    rng = random.Random(11)
    out += [(f"quantised_sphere{k}", *quantised_sphere(rng, 12 + 4 * k, 8 + 2 * k)) for k in range(2)]
    vs, fs = cuboid((-1, -1, -1), (1, 1, 1))
    out += [("a_degenerate_face_at_a_corner", vs + [(1.0, 1.0, 1.0)], fs + [(7, 8, 7)]),
            ("a_vertex_no_face_names", vs + [(5.0, 0.0, 0.0)], fs)]
    rng = random.Random(2026)
    for n in (4, 8):
        vs, fs = subdivided_cube(n, (-1.0, -0.5, -0.25), (1.0, 0.5, 0.25))
        out += [(f"subdivided_box{n}_turned_and_shuffled", vs, scrambled(fs, rng)),
                (f"subdivided_box{n}_wound_at_random", vs, scrambled(fs, rng, flip=0.5))]
    a, af = subdivided_cube(2, (-1.0, -0.5, -0.5), (0.0, 0.5, 0.5))
    b, bf = subdivided_cube(2, (0.0, -0.5, -0.5), (1.0, 0.5, 0.5))
    out.append(("two_boxes_sharing_a_face", a + b,
                scrambled(af + [(x + len(a), y + len(a), z + len(a)) for x, y, z in bf], rng)))
    o, of = subdivided_cube(2, (-1.0, -1.0, -1.0), (1.0, 1.0, 1.0))
    i, inner = subdivided_cube(2, (-0.5, -0.5, -0.5), (0.5, 0.5, 0.5))
    out.append(("a_box_in_a_box", o + i, scrambled(of + [(x + len(o), z + len(o), y + len(o)) for x, y, z in inner],
                                                    rng)))
    # Cubes so small that two of their octree's corners within 1e-9 of each other share MuJoCo's hash of a point.
    out += [(f"cube_{t!r}_across_half", *cuboid((-t, -t, -t), (t, t, t))) for t in (1.2e-9, 7e-10)]
    out += [(name, vs, fs) for name, (vs, fs) in FOUND.items()]
    return [(name, [tuple(float(c) for c in v) for v in vs], fs) for name, vs, fs in out]


def hash_points():
    """Points whose hash the golden holds: each zero's sign on each axis, a few magnitudes and signs, and a grid at
    the 1e-9 scale where near points share a hash."""
    rng = random.Random(5)
    pts = [(a, b, c) for a in (0.0, -0.0) for b in (0.0, -0.0) for c in (0.0, -0.0)]
    pts += [tuple(rng.choice((-1, 1)) * 10 ** rng.uniform(-12, 12) for _ in range(3)) for _ in range(120)]
    pts += [tuple(rng.uniform(-2, 2) for _ in range(3)) for _ in range(120)]
    pts += [(1.2e-9 * i / 4, -1.2e-9 * j / 4, 1.2e-9 * k / 4) for i in range(4) for j in range(4) for k in range(4)]
    return pts


HASH_POINTS = hash_points()


def bits(x):
    return f"{struct.unpack('<Q', struct.pack('<d', x))[0]:016x}"


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    work = os.path.dirname(sys.prefix)
    src, tmd = os.path.join(work, "src"), os.path.join(work, "build", "_deps", "trianglemeshdistance-src")
    head = subprocess.run(["git", "-C", src, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    if head != marker["commit"]:
        sys.exit(f"{src} is at {head!r}, the oracle was built from {marker['commit']}")
    if subprocess.run(["git", "-C", src, "status", "--porcelain"], capture_output=True, text=True).stdout.strip():
        sys.exit(f"{src} has changes: the oracle was built from its commit as it is")
    tmd_head = subprocess.run(["git", "-C", tmd, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    if not tmd_head.startswith(TMD_COMMIT):
        sys.exit(f"{tmd} is at {tmd_head!r}, not {TMD_COMMIT}")
    tmd_diff = subprocess.run(["git", "-C", tmd, "diff"], capture_output=True).stdout
    if hashlib.sha256(tmd_diff).hexdigest() != TMD_PATCH_SHA256:
        sys.exit(f"{tmd} differs from {TMD_COMMIT} otherwise than by MuJoCo's patch")
    compiler = subprocess.run(["/usr/bin/c++", "--version"], capture_output=True, text=True).stdout.split("\n")[0]
    out = [f"# MuJoCo {marker['mujoco']}'s octree source (commit {marker['commit']}, TriangleMeshDistance {tmd_head}),",
           f"# built with /usr/bin/c++ {' '.join(FLAGS)} ({compiler}, libc++ %(libcxx)s) on {marker['platform']},",
           "# over each case's mesh; from",
           "# sim/L0/tests/scripts/gen_mesh_octree_source_reference.py, whose docstring gives the format."]
    with tempfile.TemporaryDirectory() as d:
        cc, exe = os.path.join(d, "octree.cc"), os.path.join(d, "octree")
        with open(cc, "w") as f:
            f.write(program(src, tmd))
        subprocess.run(["/usr/bin/c++", *FLAGS, "-I", tmd, "-o", exe, cc], check=True)
        run = subprocess.run([exe, "hash"], input="\n".join(" ".join(bits(c) for c in p) for p in HASH_POINTS) + "\n",
                             capture_output=True, text=True, check=True).stdout.split()
        assert len(run) == len(HASH_POINTS)
        out += [f"hash {' '.join(bits(c) for c in p)} {h}" for p, h in zip(HASH_POINTS, run)]
        all_cases = cases()
        for name, vs, fs in all_cases:
            aamm = box_of(vs)
            inp, outp = os.path.join(d, "in.bin"), os.path.join(d, "out.bin")
            with open(inp, "wb") as f:
                f.write(struct.pack("<i", len(vs)) + struct.pack(f"<{3 * len(vs)}d", *[c for v in vs for c in v]))
                f.write(struct.pack("<i", len(fs)) + struct.pack(f"<{3 * len(fs)}i", *[k for t in fs for k in t]))
                f.write(struct.pack("<6d", *aamm))
            run = subprocess.run([exe, inp, outp], capture_output=True, text=True, check=True).stdout.strip()
            out += [f"case {name}", "vertices " + " ".join(bits(c) for v in vs for c in v),
                    "faces " + " ".join(str(k) for t in fs for k in t), "box " + " ".join(bits(c) for c in aamm)]
            if run.startswith("err "):
                out.append(run)
                continue
            libcxx = run.split()[1]
            data = open(outp, "rb").read()
            n = struct.unpack_from("<i", data)[0]
            o = 4
            aabb = struct.unpack_from(f"<{6 * n}Q", data, o)
            o += 48 * n
            child = struct.unpack_from(f"<{8 * n}i", data, o)
            o += 32 * n
            depth = struct.unpack_from(f"<{n}i", data, o)
            o += 4 * n
            coeff = struct.unpack_from(f"<{8 * n}Q", data, o)
            word = lambda x: x & 0xFFFFFFFFFFFFFFFF  # noqa: E731 - an int sign-extended to 64 bits
            out.append(f"octree {n} {mix_hash(aabb)} {mix_hash(map(word, child))} {mix_hash(map(word, depth))} "
                       f"{mix_hash(coeff)}")
    out[1] = out[1] % {"libcxx": libcxx}
    with open(sys.argv[1], "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print(f"{len(all_cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
