#!/usr/bin/env python3
"""Generate the native convex collision golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_ccd_reference.py \\
        sim/L0/tests/assets/golden/ccd

It refuses any other interpreter, as gen_census_golden.py does.

It writes ccd.json: MuJoCo's `mjc_ccd` (engine_collision_gjk.c:2208-2357)
called through ctypes on two geoms of a compiled model, each object set up by
`mjc_initCCDObj` (engine_collision_convex.c:709-767) after the geoms' poses
are written into `geom_xpos` and `geom_xmat`. `meshes` holds each mesh's
vertices as MuJoCo compiled them (in its frame, as floats) and whether MuJoCo
hill-climbs its graph (at least `mjMESH_HILLCLIMB_MIN` vertices). Each case
holds the two objects (geom type, size, position, row-major turn, margin,
mesh), the configuration, and MuJoCo's result: the distance returned, the
`nx` witness pairs, the GJK and EPA iteration counts, the EPA status and the
simplex size.

The cases are the calls MuJoCo makes. `contact`: as mjc_Convex makes them
(engine_collision_convex.c:917-926), the pair's margin on both objects, no
distance cutoff, one contact, for every pair the collision table sends to
mjc_Convex. `distance`: as mj_geomDistance makes them
(engine_support.c:527-553), no margin, one contact, the cutoff its distmax,
for those pairs and two boxes. `deep`: as `contact`, the geoms overlapping
deeply, where EPA also starts from GJK's segment or triangle. `aligned`: the
geoms unturned, one offset from the other along a coordinate axis (by 0, 0.3
and 0.7 of their bounding radii), whose simplices are symmetric; at offset 0
GJK starts at the origin and MuJoCo lists no contact. `reversed`: a sphere or
capsule as the second geom, and pairs the collision table gives functions of
their own (two spheres, a sphere or capsule against a box), both ways round.
`first_step`: two spheres, and a sphere square to a capsule's middle, apart
and overlapping, where GJK stops at its first step. `probe`: no contacts asked
(MuJoCo's callers always ask for one or more). `cutoff`: a sphere or capsule
against another geom, the cutoff just below their distance, so that GJK on the
centre or segment ends within it and the inflated distance is past it.
`multi`: two boxes face to face (the upper one turned about the common
normal) and edge to face (turned 45 degrees about a horizontal axis, then
about the vertical), the lower one touching with its +z, -z, -y or -x face,
tilted within the face and edge tolerances and, in two of every eight, past
them, either box first, with four and eight contacts, which `multicontact`
clips (engine_collision_gjk.c:2063-2184).
`census`: the capsule–cylinder pairs of census documents 3a13c90a and
6369216c at their start, the second a three-way tie of EPA's starting faces.
`nan`: an ellipsoid and a box, one turned by a matrix holding a NaN, which
MuJoCo returns from (NaN, after GJK's last iteration); a NaN is written as
the string "nan", as gen_census_golden.py writes one. `multi_mesh`: a box and
a mesh, or two meshes, of polygonal faces (a slab of four-cornered faces, a
prism on an irregular hexagon), face to face and edge to face, each held with
its thinnest axis vertical, tilted within the tolerances and, in two of every
eight, past them, with four contacts as mjc_Convex asks under MULTICCD; the
multi-contact clips the meshes' polygons, which `meshes` records.
It refuses a run whose cases miss a path the port must be checked on (each
starting polytope, an EPA failure, multi-contact, the sphere and capsule
inflation, the cutoff, a separating plane found by gjkIntersect, both mesh
searches).
"""

import ctypes
import json
import math
import os
import sys
from ctypes import POINTER, byref, c_double, c_int, c_void_p

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

CENSUS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "census", "docs")
MAXCONPAIR = 50
DBL_MAX = sys.float_info.max


class CCDObj(ctypes.Structure):
    _fields_ = [("model", c_void_p), ("data", c_void_p), ("geom", c_int), ("geom_type", c_int),
                ("vertindex", c_int), ("meshindex", c_int), ("flex", c_int), ("elem", c_int),
                ("vert", c_int), ("margin", c_double), ("rotate", c_double * 4),
                ("center", c_void_p), ("support", c_void_p), ("prism", c_double * 18),
                ("size", c_void_p), ("hfield_data", c_void_p), ("hfield_nrow", c_int),
                ("hfield_ncol", c_int)]


class CCDConfig(ctypes.Structure):
    _fields_ = [("max_iterations", c_int), ("tolerance", c_double), ("max_contacts", c_int),
                ("dist_cutoff", c_double), ("context", c_void_p), ("alloc", c_void_p),
                ("free", c_void_p)]


class Vertex(ctypes.Structure):
    _fields_ = [("vert", c_double * 3), ("vert1", c_double * 3), ("vert2", c_double * 3),
                ("index1", c_int), ("index2", c_int)]


class CCDStatus(ctypes.Structure):
    _fields_ = [("dist", c_double), ("x1", c_double * (3 * MAXCONPAIR)),
                ("x2", c_double * (3 * MAXCONPAIR)), ("nx", c_int), ("max_iterations", c_int),
                ("tolerance", c_double), ("max_contacts", c_int), ("dist_cutoff", c_double),
                ("gjk_iterations", c_int), ("epa_iterations", c_int), ("epa_status", c_int),
                ("simplex", Vertex * 4), ("nsimplex", c_int)]


LIB = ctypes.CDLL(os.path.join(os.path.dirname(mujoco.__file__), "libmujoco.3.5.0.dylib"))
LIB.mjc_initCCDObj.argtypes = [POINTER(CCDObj), c_void_p, c_void_p, c_int, c_double]
LIB.mjc_initCCDObj.restype = None
LIB.mjc_ccd.argtypes = [POINTER(CCDConfig), POINTER(CCDStatus), POINTER(CCDObj), POINTER(CCDObj)]
LIB.mjc_ccd.restype = c_double

TYPES = {"sphere": 2, "capsule": 3, "ellipsoid": 4, "cylinder": 5, "box": 6, "mesh": 7}

# The pairs the collision table sends to mjc_Convex (engine_collision_driver.c:41-52),
# in MuJoCo's type order.
CONVEX_PAIRS = [("sphere", "ellipsoid"), ("sphere", "mesh"), ("capsule", "ellipsoid"),
                ("capsule", "cylinder"), ("capsule", "mesh"), ("ellipsoid", "ellipsoid"),
                ("ellipsoid", "cylinder"), ("ellipsoid", "box"), ("ellipsoid", "mesh"),
                ("cylinder", "cylinder"), ("cylinder", "box"), ("cylinder", "mesh"),
                ("box", "mesh"), ("mesh", "mesh")]
# mj_geomDistance also takes mjc_ccd for two boxes (engine_support.c:569-580).
DISTANCE_PAIRS = CONVEX_PAIRS + [("box", "box")]
# A sphere or capsule second, and pairs MuJoCo's table gives functions of their own: mjc_ccd
# takes them all.
REVERSED_PAIRS = [("ellipsoid", "sphere"), ("mesh", "sphere"), ("ellipsoid", "capsule"),
                  ("cylinder", "capsule"), ("mesh", "capsule"), ("sphere", "sphere"),
                  ("sphere", "capsule"), ("capsule", "sphere"), ("capsule", "capsule"),
                  ("sphere", "cylinder"), ("cylinder", "sphere"), ("sphere", "box"),
                  ("box", "sphere"), ("capsule", "box"), ("box", "capsule")]
GROUPS = ("contact", "distance", "deep", "aligned", "reversed", "first_step", "probe", "cutoff",
          "multi", "census", "nan", "multi_mesh")


def gem8():
    """Eight corners of a box, each moved a little: below the hill-climb threshold."""
    rng = np.random.default_rng(80)
    half = np.array([0.05, 0.04, 0.03])
    corners = [half * np.array([sx, sy, sz]) * (1 + 0.15 * rng.uniform(-1, 1, 3))
               for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
    return np.array(corners)


def pebble24():
    """Twenty-four points on an ellipsoid, all on their hull: hill-climbed."""
    rng = np.random.default_rng(24)
    d = rng.normal(size=(24, 3))
    d /= np.linalg.norm(d, axis=1)[:, None]
    return d * np.array([0.06, 0.045, 0.035])


def slab8():
    """A box: polygonal faces of four corners, below the hill-climb threshold."""
    half = np.array([0.09, 0.07, 0.02])
    return np.array([half * np.array([sx, sy, sz])
                     for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])


def prism12():
    """A prism on an irregular hexagon: two six-cornered faces, hill-climbed."""
    angles = np.array([0.0, 1.1, 2.0, 3.2, 4.1, 5.3])
    radii = np.array([0.060, 0.055, 0.062, 0.058, 0.061, 0.056])
    ring = np.stack([radii * np.cos(angles), radii * np.sin(angles)], axis=1)
    return np.array([[x, y, z] for z in (-0.015, 0.015) for x, y in ring])


MESHES = {"gem8": gem8(), "pebble24": pebble24(), "slab8": slab8(), "prism12": prism12()}


def mesh_assets():
    out = []
    for name, v in MESHES.items():
        out.append(f'<mesh name="{name}" vertex="{" ".join(repr(float(x)) for x in v.ravel())}"/>')
    return "<asset>" + "".join(out) + "</asset>"


def geom_xml(kind, size, mesh):
    if kind == "mesh":
        return f'<geom type="mesh" mesh="{mesh}"/>'
    return f'<geom type="{kind}" size="{" ".join(repr(float(s)) for s in size)}"/>'


def random_size(rng, kind):
    if kind == "sphere":
        return [rng.uniform(0.03, 0.08)]
    if kind == "capsule":
        return [rng.uniform(0.02, 0.05), rng.uniform(0.02, 0.08)]
    if kind == "cylinder":
        return [rng.uniform(0.02, 0.07), rng.uniform(0.02, 0.07)]
    if kind in ("ellipsoid", "box"):
        return list(rng.uniform(0.02, 0.08, 3))
    return []


def rbound(kind, size, mesh):
    if kind == "sphere":
        return size[0]
    if kind == "capsule":
        return size[0] + size[1]
    if kind == "cylinder":
        return math.hypot(size[0], size[1])
    if kind == "ellipsoid":
        return max(size)
    if kind == "box":
        return math.sqrt(sum(s * s for s in size))
    return float(np.max(np.linalg.norm(MESHES[mesh], axis=1)))


def random_mat(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, q)
    return mat


def axis_mat(axis, angle):
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.asarray(axis, float), angle)
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, q)
    return mat


def compiled_meshes():
    """Each mesh's vertices as MuJoCo compiles them and as authored, whether it hill-climbs, and
    its polygons (`mjCMesh::MakePolygons`, which merges them in the authored frame): each one's
    vertex path and normal (from the compiled vertices, before they are rounded to floats), and
    each vertex's polygons."""
    m = mujoco.MjModel.from_xml_string(
        f"<mujoco>{mesh_assets()}<worldbody>"
        + "".join(f'<geom type="mesh" mesh="{n}"/>' for n in MESHES) + "</worldbody></mujoco>")
    out = {}
    for name in MESHES:
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MESH, name)
        adr, num = m.mesh_vertadr[i], m.mesh_vertnum[i]
        verts = np.asarray(m.mesh_vert[adr:adr + num], dtype=np.float64)
        pa, pn = m.mesh_polyadr[i], m.mesh_polynum[i]
        paths = [[int(m.mesh_polyvert[m.mesh_polyvertadr[pa + k] + j])
                  for j in range(m.mesh_polyvertnum[pa + k])] for k in range(pn)]
        normals = [[float(x) for x in m.mesh_polynormal[pa + k]] for k in range(pn)]
        polymap = [[int(m.mesh_polymap[m.mesh_polymapadr[adr + v] + j])
                    for j in range(m.mesh_polymapnum[adr + v])] for v in range(num)]
        out[name] = {"vert": [float(x) for x in verts.ravel()],
                     "authored": [float(x) for x in MESHES[name].ravel()],
                     "hillclimb": bool(m.mesh_graphadr[i] >= 0 and num >= 10),
                     "polygons": paths, "polynormals": normals, "polymap": polymap}
    return out


def call(kinds, sizes, meshes, poses, margin, config):
    """MuJoCo's mjc_ccd on two geoms at the given poses; the case as the golden holds it."""
    xml = (f"<mujoco>{mesh_assets()}<worldbody>"
           + "".join(geom_xml(k, s, me) for k, s, me in zip(kinds, sizes, meshes))
           + "</worldbody></mujoco>")
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for g, (pos, mat) in enumerate(poses):
        d.geom_xpos[g] = pos
        d.geom_xmat[g] = mat
    return run(m, d, (0, 1), margin, config, meshes)


def run(m, d, geoms, margin, config, meshes):
    objs = [CCDObj(), CCDObj()]
    for obj, g in zip(objs, geoms):
        LIB.mjc_initCCDObj(byref(obj), m._address, d._address, g, margin)
        # The layout is right only if mjc_initCCDObj's writes land where they are read.
        if (obj.geom != g or obj.geom_type != m.geom_type[g] or obj.vertindex != -1
                or obj.meshindex != -1 or obj.flex != -1 or obj.margin != margin
                or list(obj.rotate) != [1, 0, 0, 0] or not obj.support):
            sys.exit(f"mjCCDObj layout: geom {obj.geom} type {obj.geom_type} margin {obj.margin}")
    cfg = CCDConfig(config["max_iterations"], config["tolerance"], config["max_contacts"],
                    config["dist_cutoff"], None, None, None)
    st = CCDStatus()
    ret = LIB.mjc_ccd(byref(cfg), byref(st), byref(objs[0]), byref(objs[1]))
    if st.max_iterations != config["max_iterations"] or st.tolerance != config["tolerance"]:
        sys.exit("mjCCDStatus layout: the configuration did not come back")
    if ret != st.dist and not (math.isnan(ret) and math.isnan(st.dist)):
        sys.exit(f"mjc_ccd returned {ret}, its status holds {st.dist}")
    nx = st.nx
    obj = []
    for g, me in zip(geoms, meshes):
        kind = [k for k, t in TYPES.items() if t == m.geom_type[g]][0]
        obj.append({"type": kind, "size": [float(x) for x in m.geom_size[g]],
                    "pos": [float(x) for x in d.geom_xpos[g]],
                    "mat": [float(x) for x in d.geom_xmat[g]], "margin": margin, "mesh": me})
    return {"obj": obj, "config": config,
            "out": {"dist": ret, "nx": nx,
                    "x1": [[st.x1[3 * i + j] for j in range(3)] for i in range(nx)],
                    "x2": [[st.x2[3 * i + j] for j in range(3)] for i in range(nx)],
                    "gjk_iterations": st.gjk_iterations, "epa_iterations": st.epa_iterations,
                    "epa_status": st.epa_status, "nsimplex": st.nsimplex}}


def config(max_contacts, dist_cutoff):
    return {"max_iterations": 35, "tolerance": 1e-6, "max_contacts": max_contacts,
            "dist_cutoff": dist_cutoff}


def mesh_names(pair, i):
    """The meshes a pair's mesh geoms take in case i: both searches, and both on one pair."""
    first = "gem8" if i % 2 == 0 else "pebble24"
    second = "pebble24" if (i // 2) % 2 == 0 else "gem8"
    return [first if pair[0] == "mesh" else None,
            (second if pair[0] == "mesh" else first) if pair[1] == "mesh" else None]


def random_setup(rng, pair, i, spread):
    """Sizes, meshes and poses: the second geom's centre off the first's by `spread` (a range)
    times the sum of their bounding radii, in a random direction; both turned at random."""
    sizes = [random_size(rng, k) for k in pair]
    meshes = mesh_names(pair, i)
    reach = sum(rbound(k, s, me) for k, s, me in zip(pair, sizes, meshes))
    pos1 = rng.uniform(-0.1, 0.1, 3)
    u = rng.normal(size=3)
    u /= np.linalg.norm(u)
    pos2 = pos1 + u * reach * rng.uniform(*spread)
    return sizes, meshes, [(pos1, random_mat(rng)), (pos2, random_mat(rng))]


def random_case(rng, pair, i, margin, cfg, spread):
    sizes, meshes, poses = random_setup(rng, pair, i, spread)
    return call(pair, sizes, meshes, poses, margin, cfg)


def contact_cases(rng):
    out = {}
    for pair in CONVEX_PAIRS:
        for i in range(24):
            margin = 0.01 if i % 3 == 2 else 0.0
            out[f"{pair[0]}_{pair[1]}_{i}"] = random_case(
                rng, pair, i, margin, config(1, 0.0), (0.15, 1.0))
    return out


def distance_cases(rng):
    out = {}
    for pair in DISTANCE_PAIRS:
        for i in range(24):
            # Mostly apart, some overlapping; every fourth with a cutoff below the gap's scale.
            cutoff = 0.01 if i % 4 == 3 else 10.0
            out[f"{pair[0]}_{pair[1]}_{i}"] = random_case(
                rng, pair, i, 0.0, config(1, cutoff), (0.3, 1.8))
    return out


def deep_cases(rng):
    out = {}
    for pair in CONVEX_PAIRS:
        for i in range(8):
            out[f"{pair[0]}_{pair[1]}_{i}"] = random_case(
                rng, pair, i, 0.0, config(1, 0.0), (0.0, 0.2))
    return out


def aligned_cases(rng):
    out = {}
    eye = np.eye(3).ravel()
    for pair in DISTANCE_PAIRS:
        for axis in range(3):
            for depth in (0.0, 0.3, 0.7):
                i = 3 * axis + int(10 * depth) % 3
                sizes = [random_size(rng, k) for k in pair]
                meshes = mesh_names(pair, i)
                reach = sum(rbound(k, s, me) for k, s, me in zip(pair, sizes, meshes))
                pos2 = np.zeros(3)
                pos2[axis] = depth * reach
                out[f"{pair[0]}_{pair[1]}_{axis}_{depth}"] = call(
                    pair, sizes, meshes, [(np.zeros(3), eye), (pos2, eye)], 0.0, config(1, 0.0))
    return out


def reversed_cases(rng):
    out = {}
    for pair in REVERSED_PAIRS:
        for i in range(8):
            if i < 4:
                margin, cfg, spread = (0.01 if i % 2 else 0.0), config(1, 0.0), (0.15, 1.0)
            else:
                margin, cfg, spread = 0.0, config(1, 10.0), (0.3, 1.8)
            out[f"{pair[0]}_{pair[1]}_{i}"] = random_case(rng, pair, i, margin, cfg, spread)
    return out


def first_step_cases():
    out = {}
    eye = np.eye(3).ravel()
    u = np.array([0.6, 0.48, 0.64])
    for gap in (0.02, -0.01):
        for mode, cfg in (("contact", config(1, 0.0)), ("distance", config(1, 10.0))):
            tag = f"{'apart' if gap > 0 else 'overlap'}_{mode}"
            # Two points: the support point's gap is 0 at the first step.
            out[f"spheres_{tag}"] = call(("sphere", "sphere"), ([0.05], [0.04]), (None, None),
                                         [(np.zeros(3), eye), (u * (0.09 + gap), eye)], 0.0, cfg)
            # A point square to a segment's middle: the same.
            out[f"sphere_capsule_{tag}"] = call(
                ("sphere", "capsule"), ([0.04], [0.03, 0.06]), (None, None),
                [(np.array([0.07 + gap, 0.0, 0.0]), eye), (np.zeros(3), eye)], 0.0, cfg)
    return out


def probe_cases(rng):
    out = {}
    for pair in CONVEX_PAIRS[::2] + [("sphere", "sphere"), ("capsule", "box"), ("box", "box")]:
        for i in range(4):
            cfg = config(0, 0.0 if i % 2 == 0 else 10.0)
            out[f"{pair[0]}_{pair[1]}_{i}"] = random_case(rng, pair, i, 0.0, cfg, (0.15, 1.2))
    return out


def cutoff_cases(rng):
    out = {}
    for pair in [("sphere", "ellipsoid"), ("sphere", "mesh"), ("capsule", "ellipsoid"),
                 ("capsule", "cylinder"), ("capsule", "mesh")]:
        found = 0
        for i in range(200):
            sizes, meshes, poses = random_setup(rng, pair, i, (1.2, 1.8))
            dist = call(pair, sizes, meshes, poses, 0.0, config(1, 10.0))["out"]["dist"]
            if not 0 < dist < 10:
                continue
            for delta in (1e-7, 1e-9, 1e-10, 1e-11, 1e-12, 1e-13, 1e-14):
                case = call(pair, sizes, meshes, poses, 0.0, config(1, dist - delta))
                if case["out"]["dist"] == DBL_MAX and case["out"]["nx"] == 1:
                    out[f"{pair[0]}_{pair[1]}_{found}"] = case
                    found += 1
                    break
            if found == 2:
                break
    return out


def nan_cases():
    out = {}
    eye = np.eye(3).ravel()
    bad = eye.copy()
    bad[0] = math.nan
    sizes = ([0.05, 0.04, 0.03], [0.05, 0.05, 0.05])
    for name, mats, cfg in [("turn2_contact", (eye, bad), config(1, 0.0)),
                            ("turn1_contact", (bad, eye), config(1, 0.0)),
                            ("turn2_distance", (eye, bad), config(1, 10.0)),
                            ("turn2_multi", (eye, bad), config(4, 0.0))]:
        out[name] = call(("ellipsoid", "box"), sizes, (None, None),
                         [(np.zeros(3), mats[0]), (np.array([0.0, 0.0, 0.07]), mats[1])], 0.0, cfg)
    return out


def mat_mul(a, b):
    out = np.zeros((3, 3))
    mujoco.mju_mulMatMat(out, a.reshape(3, 3), b.reshape(3, 3))
    return out.ravel()


def multi_cases(rng):
    out = {}
    # The lower box's turn and the axis it then holds up: +z, -z, -y, -x.
    lower = [(np.eye(3).ravel(), 2), (axis_mat([1, 0, 0], math.pi), 2),
             (axis_mat([1, 0, 0], -math.pi / 2), 1), (axis_mat([0, 1, 0], math.pi / 2), 0)]
    for i in range(128):
        s1 = list(rng.uniform(0.04, 0.08, 3))
        s2 = list(rng.uniform(0.02, 0.05, 3))
        pen = rng.uniform(1e-4, 5e-3)
        flip, up = lower[(i // 2) % 4]
        tilt1 = mat_mul(axis_mat(rng.normal(size=3), rng.uniform(0, 5e-4)), flip)
        yaw = axis_mat([0, 0, 1], rng.uniform(0, 2 * math.pi))
        if i % 2 == 0:
            # Face to face: the upper box turned about z (its faces stay parallel).
            turn = yaw
            z = s1[up] + s2[2] - pen
            kind = "face"
        else:
            # Edge to face: the upper box turned 45 degrees about x, then about z, an edge lowest.
            turn = mat_mul(yaw, axis_mat([1, 0, 0], math.pi / 4))
            z = s1[up] + math.hypot(s2[1], s2[2]) * math.cos(
                math.pi / 4 - math.atan2(s2[2], s2[1])) - pen
            kind = "edge"
        # Tilted within the face and edge tolerances (1.6e-3 rad), past them in a face and an
        # edge case of every eight.
        tilt = rng.uniform(2e-3, 5e-2) if i % 8 in (6, 7) else rng.uniform(0, 5e-4)
        mat2 = mat_mul(axis_mat(rng.normal(size=3), tilt), turn)
        off = rng.uniform(-0.5, 0.5, 2) * min(s1)
        boxes = [(s1, (np.zeros(3), tilt1)), (s2, (np.array([off[0], off[1], z]), mat2))]
        if (i // 16) % 2:
            boxes.reverse()  # the upper box first
        max_contacts = 4 if (i // 8) % 2 == 0 else 8
        out[f"{kind}_{i}"] = call(("box", "box"), [b[0] for b in boxes], (None, None),
                                  [b[1] for b in boxes], 0.0, config(max_contacts, 0.0))
    return out


def compiled_vertices(compiled, shape, size):
    """A shape's vertices in its frame: a box's corners, or a mesh's as MuJoCo compiled them."""
    if shape == "box":
        return np.array([np.array(size) * np.array([sx, sy, sz])
                         for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])
    return np.array(compiled[shape]["vert"]).reshape(-1, 3)


def thin_up(verts):
    """The turn that holds a shape's thinnest axis vertical."""
    up = int(np.argmin(verts.max(axis=0) - verts.min(axis=0)))
    if up == 2:
        return np.eye(3).ravel()
    return axis_mat([1, 0, 0], math.pi / 2) if up == 1 else axis_mat([0, 1, 0], -math.pi / 2)


def multi_mesh_cases(rng, compiled):
    out = {}
    combos = [("box", "slab8"), ("box", "prism12"), ("slab8", "box"), ("prism12", "box"),
              ("slab8", "prism12"), ("prism12", "slab8"), ("slab8", "slab8"),
              ("prism12", "prism12")]
    for lower, upper in combos:
        for i in range(8):
            sizes = {name: list(rng.uniform(0.02, 0.05, 3)) for name in ("lower", "upper")}
            v_low = compiled_vertices(compiled, lower, sizes["lower"])
            v_up = compiled_vertices(compiled, upper, sizes["upper"])
            mat1 = mat_mul(axis_mat(rng.normal(size=3), rng.uniform(0, 5e-4)), thin_up(v_low))
            yaw = axis_mat([0, 0, 1], rng.uniform(0, 2 * math.pi))
            # Face to face, or an edge (along x before the yaw) down; tilted within the face
            # and edge tolerances, past them in one face and one edge case of the eight.
            lean = np.eye(3).ravel() if i % 2 == 0 else axis_mat([1, 0, 0], math.pi / 4)
            tilt = rng.uniform(2e-3, 5e-2) if i >= 6 else rng.uniform(0, 5e-4)
            mat2 = mat_mul(axis_mat(rng.normal(size=3), tilt),
                           mat_mul(yaw, mat_mul(lean, thin_up(v_up))))
            top = max((mat1.reshape(3, 3) @ v)[2] for v in v_low)
            bottom = min((mat2.reshape(3, 3) @ v)[2] for v in v_up)
            reach = 0.3 * min(np.ptp(v_low[:, 0]), np.ptp(v_low[:, 1]), np.ptp(v_up[:, 0]))
            off = rng.uniform(-1, 1, 2) * reach
            pos2 = np.array([off[0], off[1], top - bottom - rng.uniform(1e-4, 3e-3)])
            geoms = [(lower, sizes["lower"], (np.zeros(3), mat1)),
                     (upper, sizes["upper"], (pos2, mat2))]
            # MuJoCo's type order puts a box first; two meshes either way.
            if upper == "box" or (lower != "box" and i % 4 >= 2):
                geoms.reverse()
            kinds = [("box" if g[0] == "box" else "mesh") for g in geoms]
            meshes = [(None if g[0] == "box" else g[0]) for g in geoms]
            out[f"{lower}_{upper}_{i}"] = call(kinds, [g[1] for g in geoms], meshes,
                                               [g[2] for g in geoms], 0.0, config(4, 0.0))
    return out


def census_cases():
    out = {}
    for prefix in ("3a13c90a", "6369216c"):
        (name,) = [f for f in os.listdir(CENSUS) if f.startswith(prefix)]
        with open(os.path.join(CENSUS, name), encoding="utf-8") as f:
            m = mujoco.MjModel.from_xml_string(f.read())
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        pairs = [(c.geom1, c.geom2) for c in d.contact[:d.ncon]
                 if {m.geom_type[c.geom1], m.geom_type[c.geom2]} == {3, 5}]
        if len(pairs) != 1:
            sys.exit(f"census {prefix}: {len(pairs)} capsule-cylinder contacts at the start")
        g1, g2 = pairs[0]
        if m.opt.enableflags & mujoco.mjtEnableBit.mjENBL_OVERRIDE:
            sys.exit(f"census {prefix}: the margin override is on")
        margin = max(m.geom_margin[g1], m.geom_margin[g2])
        cfg = {"max_iterations": int(m.opt.ccd_iterations), "tolerance": float(m.opt.ccd_tolerance),
               "max_contacts": 1, "dist_cutoff": 0.0}
        out[prefix] = run(m, d, (g1, g2), float(margin), cfg, (None, None))
    return out


def coverage(cases):
    """Refuse a golden that misses a path the port must be checked on."""
    outs = [c["out"] for c in cases]
    shrink = [c for c in cases if {"sphere", "capsule"} & {o["type"] for o in c["obj"]}]
    need = {
        "polytope2 start": any(o["epa_status"] == 0 and o["nsimplex"] == 2 for o in outs),
        "polytope3 start": any(o["epa_status"] == 0 and o["nsimplex"] == 3 for o in outs),
        "polytope4 start": any(o["epa_status"] == 0 and o["nsimplex"] == 4 for o in outs),
        "an EPA failure": any(o["epa_status"] > 0 for o in outs),
        "multi-contact": any(o["nx"] > 1 for o in outs),
        "polygonQuad (four of more)": any(c["out"]["nx"] == 4 and c["config"]["max_contacts"] == 4
                                          for c in cases),
        "inflation": any(0 < c["out"]["dist"] < DBL_MAX for c in shrink),
        "the cutoff": any(c["out"]["dist"] == DBL_MAX and c["config"]["dist_cutoff"] > 0
                          for c in cases),
        "gjkIntersect apart": any(c["out"]["dist"] == DBL_MAX and c["config"]["dist_cutoff"] == 0
                                  and c["out"]["gjk_iterations"] > 0 for c in cases),
        "a second geom shrunk": any(c["obj"][1]["type"] in ("sphere", "capsule")
                                    and 0 < c["out"]["dist"] < DBL_MAX for c in cases),
        "GJK's first step": any(o["gjk_iterations"] == 0 and o["nsimplex"] == 1 and o["nx"] == 1
                                for o in outs),
        "no contacts asked": any(c["config"]["max_contacts"] == 0 for c in cases),
        "the cutoff after inflation": any(c["out"]["dist"] == DBL_MAX and c["out"]["nx"] == 1
                                          and c["config"]["dist_cutoff"] > 0 for c in cases),
        "exhaustive mesh": any(o.get("mesh") == "gem8" for c in cases for o in c["obj"]),
        "a mesh's multi-contact": any(c["out"]["nx"] > 1 and any(o["type"] == "mesh"
                                                                 for o in c["obj"])
                                      for c in cases),
        "hill-climbed mesh": any(o.get("mesh") == "pebble24" for c in cases for o in c["obj"]),
    }
    missing = [k for k, v in need.items() if not v]
    if missing:
        sys.exit(f"the cases miss: {', '.join(missing)}")
    hist = {}
    for o in outs:
        key = (o["epa_status"], o["nsimplex"], o["nx"])
        hist[key] = hist.get(key, 0) + 1
    return hist


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"), allow_nan=False)
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    rng = np.random.default_rng(48)
    doc = {"oracle": marker, "meshes": compiled_meshes(), "contact": contact_cases(rng),
           "distance": distance_cases(rng), "deep": deep_cases(rng),
           "aligned": aligned_cases(rng), "reversed": reversed_cases(rng),
           "first_step": first_step_cases(), "probe": probe_cases(rng), "cutoff": cutoff_cases(rng),
           "multi": multi_cases(rng), "census": census_cases(), "nan": nan_cases()}
    doc["multi_mesh"] = multi_mesh_cases(rng, doc["meshes"])
    cases = [c for k in GROUPS for c in doc[k].values()]
    hist = coverage(cases)
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], "ccd.json"), finite_or_string(doc))
    print(f"{len(cases)} cases -> {sys.argv[1]}")
    for (status, nsimplex, nx), n in sorted(hist.items()):
        print(f"  epa_status {status:2} nsimplex {nsimplex} nx {nx}: {n}")


if __name__ == "__main__":
    main()
