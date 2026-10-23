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
clips (engine_collision_gjk.c:2063-2184). Its tilts' axes, and
`multi_mesh`'s, are not of unit length, which `mju_axisAngle2Quat` does not
correct, so those turns are not quite rotations; both sides read the same
matrices.
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
`merge`: meshes whose hulls have neighbouring faces nearer than the merge
tolerance of `PolygonCmp` (user_mesh.cc:2776-2799), where it is not a strict
weak order, so the polygons `MakePolygons` (:2949-3005) makes depend on the
tree of its `std::set`: a slab under a shallow dome of random points, two
ways, a slab whose top is lifted by noise of 2e-5, and points on an
ellipsoid. Each holds the vertices as `MakePolygons` merges them (as
authored, rounded to floats), MuJoCo's hull faces (its graph's
`face_globalid`, in qhull's order) and its polygons in its order.

The groups after `merge` each draw from their own generator, so that the
groups before them are as they were. `graph_meshes`: meshes whose cases are
compared on MuJoCo's own hull graph, each with its vertices as MuJoCo
compiled them, as authored rounded to floats, whether it hill-climbs, its
graph (`vert_globalid` and each hull vertex's neighbours, as
`mjc_hillclimbSupport` walks them), its hull faces in qhull's order and its
polygons: a wedge of ten vertices (mjMESH_HILLCLIMB_MIN), its two top faces
at 1e-3 to each other; a cone on a base of 151 corners (more than
mjMAX_POLYVERT); a slab with a vertex raised 1e-5 above its top, 2e-3 in
from one edge, whose sliver of top to that edge is a polygon of three
corners all of which the polygon of the rest of the top holds; and a fan, a
cone on a base of 149 corners crowded into 30 degrees and six spread over
the rest, whose apex lies on 155 polygons, six of the seven wide ones at
the last six of the apex's polymap positions, 149 to 154. `small`: the
pairs of primitives mjc_ccd takes, shrunk by 1e-2 to 1e-4 and overlapping,
as mjc_Convex calls them. `large`: a box grown by 1 to 1e6 against a box, a
mesh or an ellipsoid, overlapping, with one contact, with four (as
mjc_Convex asks under MULTICCD), or as mj_geomDistance calls them. `touch`:
a primitive resting exactly on a box (sizes and heights dyadic), unturned or
turned about the vertical, centred or off, either first. `degenerate`: geoms
turned by a matrix of rank 2 or 1, and geoms of a size 0 (a box or an
ellipsoid flat, an ellipsoid a point, a cylinder a disc or a segment) on one
turn, offset along one or two of its axes. `long_epa`: deep overlaps under
`ccd_iterations` of 60 to 400 (past mjMAX_EPA_ITERATIONS mjc_ccd allocates
the polytope through the configuration's alloc and free) and
`ccd_tolerance` of 0 to 1e-10, with one contact or four. `overflow`:
primitives grown by 1e150 to 1e154 or shrunk by 1e-150 to 1e-300, where
products of three of their coordinates overflow or underflow a double, every
number of each case finite. `flush`: two unturned boxes, the upper pressed
into the lower with sides flush, so that corners lie on the clip planes,
with four and eight contacts. `graph`: a box against the wedge and the cone,
and those meshes against each other: faces pressed together, an edge or a
vertex against a face, within the face and edge tolerances or past them,
with 4, 5, 8 or 50 contacts. `notch`: a box's corner pressed onto the slab's
sliver, the box's face within the face tolerance of it, with four and eight
contacts. `s3d`: a sphere against an ellipsoid, and a capsule against an
ellipsoid or a cylinder, overlapping, at the chunk 3e-3 review's poses where S3D's stale fourth
weight changes mjc_Convex's contact (registry D-GJK-S3D), as mjc_Convex
calls mjc_ccd; a sphere and an ellipsoid with the exact distance.
`polyvert_cap`: a box laid on a side of the cone at its apex, on the polygons
at the apex's polymap positions 148, 149 and 150, and a box holding the edge
the fan's polygon at position 149, 150 or 151 contributes at its apex,
turned halfway between that edge's polygons; each tilted so that the apex
is deepest, with four and eight contacts, either first. Past position 149
the multi-contact reads no face (the cone) and no edge (the fan) of that
polygon at the apex (mjMAX_POLYVERT, engine_collision_gjk.c:1751, :1787),
and makes fewer contacts; it refuses a case of another count.

`plane_meshes` and `plane_convex` hold MuJoCo's mjc_PlaneConvex
(engine_collision_convex.c:1042-1138), called on a plane (geom 0) and an
ellipsoid or a mesh (geom 1) at poses set in mjData: `plane_meshes` each mesh
above and a tetrahedron as MuJoCo compiles them (their vertices; their hull
graphs where the geom collides, `vert_globalid` and each hull vertex's
neighbours; their radii, `geom_rbound`), `plane_convex` the cases, each with
the plane's pose, the geom's type, size, mesh and pose, the margin and the
contacts listed (`dist`, `pos`, the frame's normal): `ellipsoid`, ellipsoids
of random sizes and turns within 0.05 of planes of random poses, at margins
0, 0.01 and 0.1; `ellipsoid_at_margin`, those apart at a margin equal to
MuJoCo's own distance (each listed: the comparison is inclusive); `mesh`,
each mesh within 0.02 of a plane at random turns and with its local -z face
down, at margins 0 and 0.02; `mesh_bare`, the same draws on the mesh compiled
without a graph (its geom not colliding), where mjc_PlaneConvex searches
every vertex; `edge`, an unturned mesh over an unturned plane at a margin
equal to its lowest vertices' distance exactly (the first listed, the rest
not), with a graph and without, and a mesh without a graph apart by 0.7 of
its radius at a margin of its radius (its first vertex farther from the first
contact than the spacing); `nan`, a mesh and an ellipsoid turned by a matrix
holding a NaN, and a plane whose normal holds one; and `tetra`, the
tetrahedron laid on each of its faces at three spins, at six random turns and
with its local -z face down at 12 (margins up to 1, every vertex within it),
with its graph and without, from its own generator, and one pose where two
base corners tie exactly for the deepest. It refuses plane cases that miss
an ellipsoid listed and one not, 50 ellipsoids at their own distance, 0 to 3
mesh contacts with a graph and without, a NaN contact, a vertex within the
margin skipped as nearer the first contact than 0.3 of the radius, each
lowest vertex at the margin exactly with the first listed alone, a first
vertex past the spacing without a graph, or the tetrahedron face down listed
with its graph.

`hfield_fields` and `hfield_convex` hold MuJoCo's mjc_ConvexHField
(engine_collision_convex.c:1185-1363), called on a height field (geom 0) and
a convex geom (geom 1) at poses set in mjData: `hfield_fields` each field as
MuJoCo compiles it (rows, columns, size and its data normalised and turned
bottom to top), `hfield_convex` the cases, each with the field, the geom's
type, size, mesh, bounding radius and pose, the field's pose, the margin,
`ccd_iterations` and `ccd_tolerance`, and the contacts listed: `pose`, each
of a sphere, capsule, ellipsoid, cylinder, box and two meshes at 12 random
turns over random points of four fields (9 x 9 bumpy, 5 x 9 of uneven
cells, 30 x 30 fine, one cell), reaching 0.03 below to 0.02 above the
surface, the field unturned or turned, at margins 0 and 0.01; `edge`, a
broad flat box pressed into the fine field (mjMAXCONPAIR contacts), a sphere
clear of the field by its bounding sphere, a capsule clear by its box, a
sphere apart within the margin (listed, at a distance below 0), a sphere past
the field's +x edge within the margin (unlisted), boxes resting exactly on
the flat part of a 5 x 5 field of dyadic cells (on a grid vertex, a grid line
and mid-cell: unlisted), a sphere under that field's base within the margin
(listed), and a sphere on a 3 x 3 field scaled to 1e153 (listed) and to 1e154
(unlisted); `nan`, a sphere, a box and a mesh turned by a matrix holding a
NaN, and the field so turned. It refuses height-field cases
that miss a contact for every geom, three or more contacts, none, a geom
apart listed within the margin, mjMAXCONPAIR contacts, the far and past-edge
geoms with none, a NaN turn listing a contact, the touching boxes unlisted,
the sphere under the base listed, or contacts at 1e153 and none at
1e154.

It refuses a run whose cases miss a path the port must be checked on (each
starting polytope, an EPA failure, multi-contact, the sphere and capsule
inflation, the cutoff, a separating plane found by gjkIntersect, both mesh
searches; and what the later groups are for: polytope2's and polytope3's
failures, an exact touch inflated to the cutoff, a geom of size 0, a
singular turn, a box past size 1 through EPA, geoms below 1e-6 and past
1e150, EPA past mjMAX_EPA_ITERATIONS, mjMAXCONPAIR contacts, a
multi-contact on a mesh with a face past mjMAX_POLYVERT corners, a hill climb on
mjMESH_HILLCLIMB_MIN vertices, and a multi-contact on the notched slab).
"""

import ctypes
import json
import math
import os
import sys
from ctypes import CFUNCTYPE, POINTER, byref, c_double, c_int, c_size_t, c_void_p

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

# `mjCCDConfig`'s alloc and free, which mjc_ccd calls for the polytope only past
# mjMAX_EPA_ITERATIONS (engine_collision_gjk.c:2314-2333, :2353-2355); each buffer is held
# until it is freed.
EPA_BUFFERS = {}


@CFUNCTYPE(c_void_p, c_void_p, c_size_t)
def epa_alloc(_context, nbytes):
    buffer = ctypes.create_string_buffer(nbytes)
    EPA_BUFFERS[ctypes.addressof(buffer)] = buffer
    return ctypes.addressof(buffer)


@CFUNCTYPE(None, c_void_p, c_void_p)
def epa_free(_context, address):
    EPA_BUFFERS.pop(address, None)

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
          "multi", "census", "nan", "multi_mesh", "small", "large", "touch", "degenerate",
          "long_epa", "overflow", "flush", "graph", "notch", "s3d", "polyvert_cap")


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
                    config["dist_cutoff"], None, ctypes.cast(epa_alloc, c_void_p),
                    ctypes.cast(epa_free, c_void_p))
    st = CCDStatus()
    ret = LIB.mjc_ccd(byref(cfg), byref(st), byref(objs[0]), byref(objs[1]))
    if st.max_iterations != config["max_iterations"] or st.tolerance != config["tolerance"]:
        sys.exit("mjCCDStatus layout: the configuration did not come back")
    if EPA_BUFFERS:
        sys.exit("mjc_ccd did not free the polytope it allocated")
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


def dome(seed, n, curve):
    """A slab whose top is a shallow dome of `n` random points."""
    rng = np.random.default_rng(seed)
    hx, hy = 0.05, 0.04
    pts = [[sx * hx, sy * hy, -0.02] for sx in (-1, 1) for sy in (-1, 1)]
    top = [(sx * hx, sy * hy) for sx in (-1, 1) for sy in (-1, 1)]
    top += [(rng.uniform(-hx, hx), rng.uniform(-hy, hy)) for _ in range(n)]
    pts += [[x, y, 0.02 - curve * (x * x + y * y)] for x, y in top]
    return np.array(pts)


def flat_noise(seed, n, amplitude):
    """A slab whose top holds `n` random points lifted by up to `amplitude`."""
    rng = np.random.default_rng(seed)
    hx, hy = 0.05, 0.04
    pts = [[sx * hx, sy * hy, sz] for sz in (-0.02, 0.02) for sx in (-1, 1) for sy in (-1, 1)]
    pts += [[rng.uniform(-hx, hx), rng.uniform(-hy, hy), 0.02 + amplitude * rng.uniform(0, 1)]
            for _ in range(n)]
    return np.array(pts)


def ellipsoid_points(seed, n):
    """`n` random points on an ellipsoid."""
    rng = np.random.default_rng(seed)
    u, v = rng.uniform(-1, 1, n), rng.uniform(0, 2 * math.pi, n)
    w = np.sqrt(1 - u * u)
    return np.stack([0.06 * w * np.cos(v), 0.045 * w * np.sin(v), 0.035 * u], axis=1)


MERGE_MESHES = {"dome": dome(1, 220, 0.04), "dome_steep": dome(7, 400, 0.06),
                "flat_noise": flat_noise(2, 150, 2e-5), "ellipsoid1000": ellipsoid_points(3, 1000)}


def merge_meshes():
    """`MakePolygons` where `PolygonCmp` is not a strict weak order: each mesh's vertices as it
    merges them, MuJoCo's hull faces in qhull's order, its polygons and each vertex's polygons."""
    assets = "".join(f'<mesh name="{n}" vertex="{" ".join(repr(float(x)) for x in v.ravel())}"/>'
                     for n, v in MERGE_MESHES.items())
    m = mujoco.MjModel.from_xml_string(
        f"<mujoco><asset>{assets}</asset><worldbody>"
        + "".join(f'<geom type="mesh" mesh="{n}"/>' for n in MERGE_MESHES)
        + "</worldbody></mujoco>")
    out = {}
    for name, authored in MERGE_MESHES.items():
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MESH, name)
        adr, num = m.mesh_vertadr[i], m.mesh_vertnum[i]
        if num != len(authored):
            sys.exit(f"merge {name}: MuJoCo kept {num} of {len(authored)} vertices")
        ga = m.mesh_graphadr[i]
        nv, nf = int(m.mesh_graph[ga]), int(m.mesh_graph[ga + 1])
        fa = ga + 2 + 3 * nv + 3 * nf
        faces = [[int(m.mesh_graph[fa + 3 * k + j]) for j in range(3)] for k in range(nf)]
        pa, pn = m.mesh_polyadr[i], m.mesh_polynum[i]
        paths = [[int(m.mesh_polyvert[m.mesh_polyvertadr[pa + k] + j])
                  for j in range(m.mesh_polyvertnum[pa + k])] for k in range(pn)]
        polymap = [[int(m.mesh_polymap[m.mesh_polymapadr[adr + v] + j])
                    for j in range(m.mesh_polymapnum[adr + v])] for v in range(num)]
        out[name] = {"vert": [float(np.float32(x)) for x in authored.ravel()], "faces": faces,
                     "polygons": paths, "polymap": polymap}
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


# The groups after `merge` each draw from their own generator, so that the groups before them
# stay as they were.


def sized_call(kinds, sizes, meshes, poses, margin, cfg):
    """`call`, each primitive compiled with sizes of 1 and its sizes then written into
    `geom_size`, which mjc_ccd's support functions read (the compiler refuses a size of 0)."""
    xml = (f"<mujoco>{mesh_assets()}<worldbody>"
           + "".join(geom_xml(k, [1.0] * len(s), me) for k, s, me in zip(kinds, sizes, meshes))
           + "</worldbody></mujoco>")
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for g, (kind, size) in enumerate(zip(kinds, sizes)):
        if kind != "mesh":
            m.geom_size[g] = (list(size) + [0.0, 0.0, 0.0])[:3]
    for g, (pos, mat) in enumerate(poses):
        d.geom_xpos[g] = pos
        d.geom_xmat[g] = mat
    return run(m, d, (0, 1), margin, cfg, meshes)


def ccd_config(max_iterations, tolerance, max_contacts, dist_cutoff):
    return {"max_iterations": max_iterations, "tolerance": tolerance,
            "max_contacts": max_contacts, "dist_cutoff": dist_cutoff}


# The pairs of primitives mjc_ccd takes in the collision table and mj_geomDistance.
PRIMITIVE_PAIRS = [p for p in DISTANCE_PAIRS if "mesh" not in p]


def small_cases():
    """The geoms shrunk by 1e-2 to 1e-4, overlapping, as mjc_Convex calls them."""
    rng = np.random.default_rng(101)
    out = {}
    for pair in PRIMITIVE_PAIRS:
        for i, scale in enumerate([1e-2, 1e-3, 3e-4, 1e-4]):
            sizes, meshes, poses = random_setup(rng, pair, i, (0.0, 0.5))
            sizes = [[x * scale for x in s] for s in sizes]
            poses = [(pos * scale, mat) for pos, mat in poses]
            out[f"{pair[0]}_{pair[1]}_{i}"] = call(pair, sizes, meshes, poses, 0.0,
                                                   config(1, 0.0))
    return out


def large_cases():
    """A box, grown by 1 to 1e6, against a box, a mesh or an ellipsoid, overlapping."""
    rng = np.random.default_rng(102)
    out = {}
    for pair in [("box", "box"), ("box", "mesh"), ("mesh", "box"), ("ellipsoid", "box")]:
        for i, scale in enumerate([1.0, 1e2, 1e3, 1e4, 1e5, 1e6]):
            meshes = mesh_names(pair, i)
            sizes = [[x * scale for x in random_size(rng, k)] for k in pair]
            reach = sum(rbound(k, s, me) for k, s, me in zip(pair, sizes, meshes))
            pos1 = rng.uniform(-0.1, 0.1, 3) * scale
            u = rng.normal(size=3)
            u /= np.linalg.norm(u)
            pos2 = pos1 + u * reach * rng.uniform(0.0, 0.7)
            poses = [(pos1, random_mat(rng)), (pos2, random_mat(rng))]
            cfg = [config(1, 0.0), config(4, 0.0), config(1, 10.0 * scale)][i % 3]
            out[f"{pair[0]}_{pair[1]}_{i}"] = call(pair, sizes, meshes, poses, 0.0, cfg)
    return out


def z_turn(rng):
    return axis_mat([0, 0, 1], rng.uniform(0, 2 * math.pi))


def touch_cases():
    """A geom resting exactly on a box's top: sizes and heights dyadic, so that the touch is
    exact; unturned or turned about the vertical, centred or off; either geom first."""
    rng = np.random.default_rng(103)
    out = {}
    eye = np.eye(3).ravel()
    low, top = [0.25, 0.1875, 0.125], 0.125
    for up in ["cylinder", "ellipsoid", "capsule", "sphere", "box"]:
        for i in range(8):
            if up == "cylinder":
                usize = [[0.125, 0.0625], [0.0625, 0.125]][i % 2]
                h = usize[1]
            elif up == "ellipsoid":
                usize = [[0.25, 0.25, 0.0625], [0.125, 0.0625, 0.03125]][i % 2]
                h = usize[2]
            elif up == "capsule":
                usize = [0.0625, 0.125]
                h = usize[0] + (usize[1] if i % 2 == 0 else 0)
            elif up == "sphere":
                usize = [0.0625]
                h = 0.0625
            else:
                usize = [0.0625, 0.03125, 0.046875]
                h = 0.046875
            umat = eye
            if up == "capsule" and i % 2 == 1:
                umat = axis_mat([1, 0, 0], math.pi / 2)  # lying down
            if i % 4 >= 2:
                umat = mat_mul(z_turn(rng), umat)
            off = [0.0, 0.0] if i % 3 == 0 else [0.03125, -0.015625]
            pos2 = np.array([off[0], off[1], top + h])
            cfg = [config(1, 0.0), config(4, 0.0), config(1, 10.0)][i % 3]
            for order in (0, 1):
                geoms = [("box", low, (np.zeros(3), eye)), (up, usize, (pos2, umat))]
                if order:
                    geoms.reverse()
                out[f"box_{up}_{i}_{order}"] = call(
                    [g[0] for g in geoms], [g[1] for g in geoms], (None, None),
                    [g[2] for g in geoms], 0.0, cfg)
    return out


def singular_mat(rng, rank):
    """A turn squashed to rank 2 or 1: a geom flattened to a plane or a line."""
    r1 = random_mat(rng).reshape(3, 3)
    r2 = random_mat(rng).reshape(3, 3)
    return (r1 @ np.diag([1.0, 1.0 if rank > 1 else 0.0, 0.0]) @ r2.T).ravel()


def degenerate_cases():
    """Geoms MuJoCo's compiler and kinematics do not make: turned by a matrix of rank 2 or 1;
    or of a size 0 (a box or an ellipsoid flat, an ellipsoid a point, a cylinder a disc or a
    segment), the two on one turn, the second offset along one or two of its axes. The geoms of
    a size 0 draw from a generator of their own."""
    rng = np.random.default_rng(104)
    out = {}
    for pair in [("ellipsoid", "sphere"), ("capsule", "ellipsoid"), ("cylinder", "mesh")]:
        for i in range(6):
            sizes = [random_size(rng, k) for k in pair]
            meshes = mesh_names(pair, i)
            reach = sum(rbound(k, s, me) for k, s, me in zip(pair, sizes, meshes))
            pos1 = rng.uniform(-0.05, 0.05, 3)
            u = rng.normal(size=3)
            u /= np.linalg.norm(u)
            pos2 = pos1 + u * reach * rng.uniform(0.0, 0.6)
            rank = 2 if i < 3 else 1
            mats = [singular_mat(rng, rank) if i % 3 in (0, 2) else random_mat(rng),
                    singular_mat(rng, rank) if i % 3 in (1, 2) else random_mat(rng)]
            cfg = [config(1, 0.0), config(4, 0.0), config(1, 10.0)][i % 3]
            out[f"singular_{pair[0]}_{pair[1]}_{i}"] = call(
                pair, sizes, meshes, [(pos1, mats[0]), (pos2, mats[1])], 0.0, cfg)
    rng = np.random.default_rng(204)
    for pair in [("box", "box"), ("box", "ellipsoid"), ("ellipsoid", "box"),
                 ("ellipsoid", "ellipsoid"), ("cylinder", "cylinder"), ("box", "cylinder"),
                 ("cylinder", "ellipsoid")]:
        for i in range(12):
            sizes = []
            for kind in pair:
                size = random_size(rng, kind)
                if kind in ("box", "ellipsoid"):
                    if i % 2 == 0:
                        size[i % 3] = 0.0
                    if i % 5 == 0 and kind == "ellipsoid":
                        size = [0.0, 0.0, 0.0]
                elif kind == "cylinder":
                    size[1 if i % 2 == 0 else 0] = 0.0
                sizes.append(size)
            turn = random_mat(rng) if i % 4 >= 2 else np.eye(3).ravel()
            off = np.zeros(3)
            off[(i + 1) % 3] = rng.uniform(-0.03, 0.03)
            if i % 3 == 0:
                off[(i + 2) % 3] = rng.uniform(-0.03, 0.03)
            pos2 = turn.reshape(3, 3) @ off
            cfg = [config(1, 0.0), config(4, 0.0), config(1, 10.0), config(1, 0.0)][i % 4]
            out[f"flat_{pair[0]}_{pair[1]}_{i}"] = sized_call(
                pair, sizes, (None, None), [(np.zeros(3), turn), (pos2, turn)], 0.0, cfg)
    return out


def long_epa_cases():
    """Deep overlaps under 60 to 400 iterations (past mjMAX_EPA_ITERATIONS mjc_ccd allocates
    the polytope) and tolerances of 0 to 1e-10, with one contact or four."""
    rng = np.random.default_rng(105)
    out = {}
    for pair in [("ellipsoid", "ellipsoid"), ("capsule", "ellipsoid"), ("ellipsoid", "cylinder"),
                 ("ellipsoid", "box"), ("cylinder", "box"), ("box", "mesh")]:
        for i, (n, tol) in enumerate([(170, 0.0), (300, 0.0), (170, 1e-15), (400, 1e-14),
                                      (120, 1e-12), (60, 1e-10)]):
            sizes, meshes, poses = random_setup(rng, pair, i, (0.0, 0.4))
            cfg = ccd_config(n, tol, 4 if i % 4 == 1 else 1, 0.0)
            out[f"{pair[0]}_{pair[1]}_{i}"] = call(pair, sizes, meshes, poses, 0.0, cfg)
    return out


def finite(o):
    if isinstance(o, float):
        return math.isfinite(o)
    if isinstance(o, list):
        return all(finite(x) for x in o)
    if isinstance(o, dict):
        return all(finite(v) for v in o.values())
    return True


def overflow_cases():
    """Primitives grown or shrunk by 1e±150 and more, where products of their coordinates
    leave the range of a double, every number of each case still finite."""
    rng = np.random.default_rng(106)
    out = {}
    eye = np.eye(3).ravel()
    for pair in [("box", "box"), ("ellipsoid", "box"), ("capsule", "cylinder"),
                 ("cylinder", "ellipsoid")]:
        for i, scale in enumerate([1e150, 1e152, 1e-160, 1e-300, 1e154, 1e-150]):
            sizes = [[x * scale for x in random_size(rng, k)] for k in pair]
            u = rng.normal(size=3)
            pos2 = u / np.linalg.norm(u) * scale * 0.03
            mats = [random_mat(rng), random_mat(rng)] if i % 2 else [eye, eye]
            cfg = [config(1, 0.0), config(1, 10.0)][i % 2]
            case = sized_call(pair, sizes, (None, None), [(np.zeros(3), mats[0]), (pos2, mats[1])],
                              0.0, cfg)
            if not finite(case):
                sys.exit(f"overflow {pair} {i}: a number past a double's range")
            out[f"{pair[0]}_{pair[1]}_{i}"] = case
    return out


def flush_cases():
    """Two boxes unturned, the upper pressed 1e-3 or 5e-4 into the lower with sides flush, so
    that corners lie on the clip planes: the same footprint, offset along x or both axes, and
    at a corner; four and eight contacts; either box first."""
    out = {}
    eye = np.eye(3).ravel()
    for name, s1, s2, p2 in [
            ("same", [0.05, 0.04, 0.03], [0.05, 0.04, 0.02], [0.0, 0.0, 0.049]),
            ("offset_x", [0.05, 0.04, 0.03], [0.05, 0.04, 0.02], [0.01, 0.0, 0.049]),
            ("offset_xy", [0.05, 0.04, 0.03], [0.03, 0.04, 0.02], [0.02, 0.0, 0.0495]),
            ("corner", [0.05, 0.04, 0.03], [0.02, 0.02, 0.02], [0.03, 0.02, 0.0495])]:
        for max_contacts in (4, 8):
            for order in (0, 1):
                boxes = [(s1, (np.zeros(3), eye)), (s2, (np.array(p2), eye))]
                if order:
                    boxes.reverse()
                out[f"{name}_{max_contacts}_{order}"] = call(
                    ("box", "box"), [b[0] for b in boxes], (None, None), [b[1] for b in boxes],
                    0.0, config(max_contacts, 0.0))
    return out


def wedge10(angle):
    """A slab whose top is two faces meeting at a ridge, `angle` apart: ten vertices."""
    hx, hy = 0.05, 0.04
    t = math.tan(angle / 2)
    pts = [[sx * hx, sy * hy, -0.02] for sx in (-1, 1) for sy in (-1, 1)]
    pts += [[sx * hx, sy * hy, 0.02 - t * hx] for sx in (-1, 1) for sy in (-1, 1)]
    pts += [[0.0, sy * hy, 0.02] for sy in (-1, 1)]
    return np.array(pts)


def cone(n):
    """A cone on an `n`-cornered base (on an ellipse), its apex off the axis."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False) + 0.013
    base = [[0.05 * math.cos(t), 0.045 * math.sin(t), -0.04 / 3] for t in a]
    return np.array([[0.002, -0.001, 0.08 / 3]] + base)


def fan155():
    """A cone whose base has 149 corners crowded into 30 degrees of a circle and six spread
    over the other 330: its apex lies on 155 polygons, seven of them wide. Six of those take
    the apex's last six polymap positions, 149 to 154, and the edge each contributes at the apex
    is 36 degrees or more from every other edge there; position 149 is the last that
    mjMAX_POLYVERT lets the multi-contact read. The seventh, at position 75, contributes an
    edge 0.16 degrees from a crowded one."""
    crowd = [math.radians(30) * k / 148 for k in range(149)]
    spread = [math.radians(30) + math.radians(330) * k / 7 for k in range(1, 7)]
    base = [[0.05 * math.cos(t), 0.05 * math.sin(t), -0.0133] for t in crowd + spread]
    return np.array([[0.0, 0.0, 0.0267]] + base)


def notch9():
    """A slab with a vertex raised 1e-5 above its top, 2e-3 in from one of its edges: nine
    vertices. The sliver of top to that edge is tilted past the merge tolerance and the rest
    of the top lies within it, so that one polygon runs along two of the sliver's edges."""
    hx, hy, hz = 0.05, 0.04, 0.02
    pts = [[sx * hx, sy * hy, sz * hz] for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
    return np.array(pts + [[0.0, hy - 2e-3, hz + 1e-5]])


# The meshes whose cases are compared on MuJoCo's hull graph: a wedge of mjMESH_HILLCLIMB_MIN
# vertices, its top nearly flat; a cone whose base has more than mjMAX_POLYVERT corners; the
# notched slab.
GRAPH_MESHES = {"wedge10": wedge10(1e-3), "cone152": cone(151), "notch9": notch9()}
# The fan, for `polyvert_cap` alone: compiled with the meshes above into `graph_meshes`, but
# not one of them, whose plane cases draw from one generator mesh after mesh.
CAP_MESHES = {"fan155": fan155()}


def graph_assets(meshes=GRAPH_MESHES):
    return "<asset>" + "".join(
        f'<mesh name="{n}" vertex="{" ".join(repr(float(x)) for x in v.ravel())}"/>'
        for n, v in meshes.items()) + "</asset>"


def graph_meshes():
    """Each of `GRAPH_MESHES` and `CAP_MESHES` as MuJoCo compiles it: its vertices, as
    authored rounded to floats (as `MakePolygons` merges them), whether it hill-climbs, its
    hull graph (each hull vertex's index among the mesh's vertices, `vert_globalid`, and its
    neighbours among the hull's vertices, as `mjc_hillclimbSupport` walks them,
    engine_collision_convex.c:383-417), its hull faces in qhull's order, and its polygons."""
    meshes = {**GRAPH_MESHES, **CAP_MESHES}
    m = mujoco.MjModel.from_xml_string(
        f"<mujoco>{graph_assets(meshes)}<worldbody>"
        + "".join(f'<geom type="mesh" mesh="{n}"/>' for n in meshes)
        + "</worldbody></mujoco>")
    out = {}
    for name, authored in meshes.items():
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MESH, name)
        adr, num = m.mesh_vertadr[i], m.mesh_vertnum[i]
        if num != len(authored):
            sys.exit(f"graph mesh {name}: MuJoCo kept {num} of {len(authored)} vertices")
        verts = np.asarray(m.mesh_vert[adr:adr + num], dtype=np.float64)
        ga = m.mesh_graphadr[i]
        nv, nf = int(m.mesh_graph[ga]), int(m.mesh_graph[ga + 1])
        edgeadr = [int(x) for x in m.mesh_graph[ga + 2:ga + 2 + nv]]
        globalid = [int(x) for x in m.mesh_graph[ga + 2 + nv:ga + 2 + 2 * nv]]
        localid = m.mesh_graph[ga + 2 + 2 * nv:ga + 2 + 3 * nv + 3 * nf]
        neighbours = []
        for v in range(nv):
            k, around = edgeadr[v], []
            while localid[k] >= 0:
                around.append(int(localid[k]))
                k += 1
            neighbours.append(around)
        fa = ga + 2 + 3 * nv + 3 * nf
        faces = [[int(m.mesh_graph[fa + 3 * k + j]) for j in range(3)] for k in range(nf)]
        pa, pn = m.mesh_polyadr[i], m.mesh_polynum[i]
        paths = [[int(m.mesh_polyvert[m.mesh_polyvertadr[pa + k] + j])
                  for j in range(m.mesh_polyvertnum[pa + k])] for k in range(pn)]
        normals = [[float(x) for x in m.mesh_polynormal[pa + k]] for k in range(pn)]
        polymap = [[int(m.mesh_polymap[m.mesh_polymapadr[adr + v] + j])
                    for j in range(m.mesh_polymapnum[adr + v])] for v in range(num)]
        out[name] = {"vert": [float(x) for x in verts.ravel()],
                     "authored": [float(np.float32(x)) for x in authored.ravel()],
                     "hillclimb": bool(ga >= 0 and num >= 10),
                     "graph": {"vertex": globalid, "neighbours": neighbours}, "faces": faces,
                     "polygons": paths, "polynormals": normals, "polymap": polymap}
    return out


def unit_turn(axis, angle):
    """`axis_mat` about the unit `axis` (`mju_axisAngle2Quat` takes the axis as it is)."""
    return axis_mat(np.asarray(axis, float) / np.linalg.norm(axis), angle)


def align_mat(a, b):
    """The turn taking the direction `a` to `b`."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    axis = np.cross(a, b)
    if np.linalg.norm(axis) < 1e-12:
        if a @ b > 0:
            return np.eye(3).ravel()
        axis = np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
        return unit_turn(axis, math.pi)
    return unit_turn(axis, math.atan2(np.linalg.norm(axis), a @ b))


def shape_faces(meshes, shape, size):
    """A box's or mesh's faces in its frame: each face's normal and corners."""
    if shape == "box":
        out = []
        for ax in range(3):
            for sgn in (1, -1):
                n = np.zeros(3)
                n[ax] = sgn
                u = np.zeros(3)
                u[(ax + 1) % 3] = size[(ax + 1) % 3]
                w = np.zeros(3)
                w[(ax + 2) % 3] = size[(ax + 2) % 3]
                c = n * size[ax]
                out.append((n, [c + u + w, c - u + w, c - u - w, c + u - w]))
        return out
    m = meshes[shape]
    verts = np.array(m["vert"]).reshape(-1, 3)
    return [(np.array(n), [verts[v] for v in path])
            for n, path in zip(m["polynormals"], m["polygons"])]


def place_dist(m, d, pos2, meshes):
    """MuJoCo's distance between the two geoms, the second at `pos2`."""
    d.geom_xpos[1] = pos2
    return run(m, d, (0, 1), 0.0, config(1, 10.0), meshes)["out"]["dist"]


def pressed_case(rng, meshes, assets, pair, faces, mode, tilted, max_contacts):
    """Two shapes (a box or a mesh of `meshes`) pressed together: a face of each (`faces`: the
    one of most corners, or one at random) turned to face the other's, or an edge or a vertex
    of the second against the first's face; the second tilted within the face and edge
    tolerances or past them; brought to touch on MuJoCo's own distance (by bisection), then
    pressed in by 5e-5 to 2e-3; either first. None where the pose does not touch within the
    bisection's bracket."""
    sizes = [list(rng.uniform(0.015, 0.05, 3)) if s == "box" else [] for s in pair]
    kinds = ["box" if s == "box" else "mesh" for s in pair]
    names = [None if s == "box" else s for s in pair]
    xml = (f"<mujoco>{assets}<worldbody>"
           + "".join(geom_xml(k, s, me) for k, s, me in zip(kinds, sizes, names))
           + "</worldbody></mujoco>")
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    fa = shape_faces(meshes, pair[0], sizes[0])
    fb = shape_faces(meshes, pair[1], sizes[1])
    if faces == "any":
        ia, ib = int(rng.integers(len(fa))), int(rng.integers(len(fb)))
    else:
        ia, ib = (int(np.argmax([len(c) for _, c in f])) for f in (fa, fb))
    na, ca = fa[ia]
    nb, cb = fb[ib]
    r1 = random_mat(rng)
    n_a = r1.reshape(3, 3) @ na
    r2 = mat_mul(unit_turn(n_a, rng.uniform(0, 2 * math.pi)), align_mat(nb, -n_a))
    anchor = np.mean(cb, axis=0)
    if mode == "edge":
        # Leaning about one of its edges, away from the first's face.
        k = int(rng.integers(len(cb)))
        e0, e1 = cb[k], cb[(k + 1) % len(cb)]
        axis = r2.reshape(3, 3) @ (e1 - e0)
        lean = rng.uniform(0.15, 0.7)
        for sgn in (1, -1):
            turned = mat_mul(unit_turn(axis, sgn * lean), r2)
            if (turned.reshape(3, 3) @ (np.mean(cb, axis=0) - (e0 + e1) / 2)) @ n_a > 0:
                break
        r2 = turned
        anchor = (e0 + e1) / 2
    elif mode == "vertex":
        # Turned at random, its lowest corner down.
        r2 = random_mat(rng)
        corners = np.array([v for _, c in fb for v in c])
        anchor = corners[np.argmin([(r2.reshape(3, 3) @ v) @ n_a for v in corners])]
    tilt = rng.uniform(2e-3, 3e-2) if tilted else rng.uniform(0, 5e-4)
    r2 = mat_mul(unit_turn(rng.normal(size=3), tilt), r2)
    reach = max(np.ptp(np.array(ca), axis=0))
    t1 = np.cross(n_a, rng.normal(size=3))
    t1 /= np.linalg.norm(t1)
    centre = r1.reshape(3, 3) @ np.mean(ca, axis=0) + t1 * rng.uniform(0, 0.3) * reach
    base = centre - r2.reshape(3, 3) @ anchor
    d.geom_xpos[0] = 0.0
    d.geom_xmat[0] = r1
    d.geom_xmat[1] = r2
    lo, hi = -0.02, 0.06
    if not place_dist(m, d, base + lo * n_a, names) < 0 < place_dist(m, d, base + hi * n_a,
                                                                     names):
        return None
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if place_dist(m, d, base + mid * n_a, names) < 0:
            lo = mid
        else:
            hi = mid
    d.geom_xpos[1] = base + (hi - rng.uniform(5e-5, 2e-3)) * n_a
    geoms = (1, 0) if rng.uniform() < 0.5 else (0, 1)
    return run(m, d, geoms, 0.0, config(max_contacts, 0.0), [names[g] for g in geoms])


# The poses of `graph`: the faces, the mode, tilted past the tolerances, and the contacts asked.
GRAPH_POSES = [("most", "face", False, 50), ("most", "face", False, 4), ("most", "face", True, 8),
               ("most", "face", True, 5), ("most", "edge", False, 8), ("most", "vertex", False, 4),
               ("any", "face", False, 50), ("any", "face", False, 4), ("any", "face", True, 5),
               ("any", "edge", False, 8), ("any", "edge", True, 50), ("any", "vertex", False, 5)]


def graph_cases(meshes):
    """A box against the wedge and the cone, and the wedge and the cone against each other and
    themselves, in each pose of `GRAPH_POSES` (`pressed_case`)."""
    rng = np.random.default_rng(107)
    out = {}
    assets = graph_assets()
    for pair in [("box", "wedge10"), ("box", "cone152"), ("wedge10", "wedge10"),
                 ("wedge10", "cone152"), ("cone152", "cone152")]:
        for i, (faces, mode, tilted, max_contacts) in enumerate(GRAPH_POSES):
            case = pressed_case(rng, meshes, assets, pair, faces, mode, tilted, max_contacts)
            if case is not None:
                out[f"{pair[0]}_{pair[1]}_{i}"] = case
    return out


def notch_cases(meshes):
    """A box against notch9's sliver (its polygon of three corners): the box's face turned to
    face it, then 2e-4 to 1.4e-3 off it (within the face tolerance) about an axis in the face,
    its lowest corner over a point of the sliver at random; brought to touch on MuJoCo's own
    distance (by bisection), then pressed in by up to 2e-5; four and eight contacts, either
    first. A pose that does not touch within the bisection's bracket is passed over."""
    rng = np.random.default_rng(108)
    out = {}
    notch = meshes["notch9"]
    verts = np.array(notch["vert"]).reshape(-1, 3)
    (sliver,) = [k for k, path in enumerate(notch["polygons"]) if len(path) == 3]
    normal = np.array(notch["polynormals"][sliver])
    corners = verts[notch["polygons"][sliver]]
    assets = graph_assets()
    for i in range(8):
        size = list(rng.uniform(0.015, 0.05, 3))
        names = ["notch9", None]
        xml = (f"<mujoco>{assets}<worldbody>{geom_xml('mesh', [], 'notch9')}"
               f"{geom_xml('box', size, None)}</worldbody></mujoco>")
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        r1 = random_mat(rng)
        n_a = r1.reshape(3, 3) @ normal
        point = r1.reshape(3, 3) @ (rng.dirichlet([1.0, 1.0, 1.0]) @ corners)
        r2 = mat_mul(unit_turn(n_a, rng.uniform(0, 2 * math.pi)),
                     align_mat(np.array([0.0, 0.0, -1.0]), -n_a))
        r2 = mat_mul(unit_turn(np.cross(n_a, rng.normal(size=3)), rng.uniform(2e-4, 1.4e-3)), r2)
        box = np.array([np.array(size) * np.array([sx, sy, sz])
                        for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])
        lowest = box[np.argmin([(r2.reshape(3, 3) @ c) @ n_a for c in box])]
        base = point - r2.reshape(3, 3) @ lowest
        d.geom_xpos[0] = 0.0
        d.geom_xmat[0] = r1
        d.geom_xmat[1] = r2
        lo, hi = -0.005, 0.01
        if not place_dist(m, d, base + lo * n_a, names) < 0 < place_dist(m, d, base + hi * n_a,
                                                                         names):
            continue
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if place_dist(m, d, base + mid * n_a, names) < 0:
                lo = mid
            else:
                hi = mid
        d.geom_xpos[1] = base + (hi - rng.uniform(1e-7, 2e-5)) * n_a
        geoms = (1, 0) if i % 2 else (0, 1)
        out[f"box_{i}"] = run(m, d, geoms, 0.0, config([4, 8][(i // 2) % 2], 0.0),
                              [names[g] for g in geoms])
    return out


# The chunk 3e-3 review's poses, from a random sweep of mjc_Convex run with
# S3D's fourth weight kept and reset, where the two differ: (first type, size,
# turn, second type, size, position, turn, margin), the first geom at the
# origin, turns row-major.
S3D_POSES = [
    ('sphere', [0.19431643263362547], [-0.14066240579386236, 0.979904636996789, -0.1414248563318145, 0.8021439870650064, 0.19652516691946387, 0.5638642414471164, 0.5803267283278775, -0.03412859738501467, -0.8136683152420241],
     'ellipsoid', [0.18137875349531246, 0.4007446788351061, 0.14571340680083741], [-0.18822418909450248, 0.1609455439820394, -0.12743465562536366], [0.009172830441457877, 0.6660796527733304, 0.7458242120922679, -0.08182457488085249, 0.743854314909841, -0.663314026035617, -0.9966045344705493, -0.054942282000031095, 0.06132493392877837], 0.0),
    ('sphere', [0.3483822396013471], [0.27446136593595016, 0.7505871322110216, 0.6010739684662835, 0.6107379257078698, -0.6188807756081062, 0.49394915900804737, 0.7427450065201521, 0.23152870781383292, -0.6282708912144791],
     'ellipsoid', [0.29794101115611127, 0.35113609179488564, 0.07609180867010275], [-0.3223250275083347, 0.1839395212355326, -0.09656603979586885], [-0.15018593003650826, 0.804793694111978, -0.5742397550994403, 0.9748650646090378, 0.023868123081255202, -0.22151392395376543, -0.16456698400312736, -0.5930745506411265, -0.7881499128706038], 0.02),
    ('sphere', [0.36461681799750456], [0.37266141179267126, 0.6602625341288096, -0.6520558704486112, -0.888005703922441, 0.049751662438963895, -0.45713306802917864, -0.26938697437311887, 0.7493851867433478, 0.6048574211562049],
     'ellipsoid', [0.11500982658916105, 0.34519283135196294, 0.3434037362703882], [0.13250514923939288, -0.14909811572557063, -0.4067405963349228], [0.6942382796698352, -0.7052462975373064, 0.14373889818343855, 0.37694208855724076, 0.18612960429051734, -0.9073425110071504, 0.6131458822459571, 0.6840931444010944, 0.3950553086192495], 0.02),
    ('sphere', [0.3804227362795654], [-0.8387991360871244, 0.3215243910876274, 0.4393609851081727, -0.5429631073662189, -0.4345947989822558, -0.7185530076040265, -0.04008831923033576, -0.841278447740879, 0.5391136244132433],
     'ellipsoid', [0.27380710526333246, 0.23438394712150545, 0.41910560783592204], [0.31492508732437496, -0.22010214450048385, -0.06911072294721943], [-0.24712225018995182, 0.40407940347762034, 0.8807101845364487, 0.30955981873491717, -0.8283489408788393, 0.4669162138646619, 0.9182065737544358, 0.3880178705035065, 0.07961670731739645], 0.02),
    ('sphere', [0.4271688823992272], [0.18094888154167785, -0.9802930235745597, -0.07926594602894721, -0.4054428895845415, -0.14778223971629548, 0.902095600759572, -0.8960321230592982, -0.13109537579748382, -0.42419386710602425],
     'ellipsoid', [0.42714460776966034, 0.40132354579129037, 0.46742434580403197], [0.34145331418173536, 0.3972985745213519, -0.1167410027943267], [-0.03229074797136894, -0.4637933130793133, 0.8853547708903835, -0.32779623869768654, -0.831907100772121, -0.44775015531049606, 0.9441964485694165, -0.3046741512301923, -0.1251668010339127], 0.02),
    ('sphere', [0.058287239655370936], [0.3447734057825807, -0.7573889240014136, 0.5545207989472175, -0.3606815138811969, 0.438502304706172, 0.8231795516846511, -0.8666257232562667, -0.48381581884778846, -0.12199225066862272],
     'ellipsoid', [0.4126756042842923, 0.4170383748048545, 0.22510650431108464], [0.2780364801837688, -0.3122662538041742, -0.022046996541299074], [0.3729861126837072, 0.921864656506072, 0.10510430452668695, -0.844024874346228, 0.3841585569546099, -0.3742248182622121, -0.38536135150003137, 0.05087001281200473, 0.9213625076844486], 0.02),
    ('capsule', [0.32278816092237844, 0.05579297750806402], [0.4245235177295487, 0.42864546262823816, 0.7975229465430782, 0.18499198876245254, -0.9033171260441537, 0.3870350551165145, 0.8863169562793225, -0.01677012711801734, -0.4627753405792096],
     'ellipsoid', [0.45994680136291566, 0.23350642375677305, 0.2673899847201243], [-0.49174881412297, 0.215281667622043, -0.039071822591319694], [-0.6287069224387408, 0.7769540884937132, 0.03271009095846322, 0.08127588451077133, 0.023818843558727487, 0.9964069917902609, 0.7733833695358333, 0.6291065148799755, -0.078122702596444], 0.0),
    ('capsule', [0.3146805989155094, 0.37078012772969576], [-0.8095957985432566, 0.08170627190514514, -0.5812733677990672, 0.05447724877114697, -0.9755321232743206, -0.21300071790072173, -0.5844543372646266, -0.2041106601607539, 0.7853355754461324],
     'cylinder', [0.3388141940927644, 0.48194367674319916], [-0.5440039770816825, -0.5526569472950295, 0.5391841799757686], [0.44905842809762453, -0.40150374777951225, 0.7982112932510395, 0.5877854536687582, -0.5400853578763118, -0.6023421508271098, 0.672944862979058, 0.7396638065544525, -0.006531819308192244], 0.0),
]


def point_ellipsoid(p, radii):
    """The signed distance from `p` to the ellipsoid of semi-axes `radii`
    (negative inside) and the outward unit normal at the nearest surface
    point: the root t of sum((r_i y_i / (t + r_i^2))^2) = 1 (y = |p|) by
    bisection to the last bit."""
    p = np.asarray(p, float)
    y, r = np.abs(p), np.asarray(radii, float)
    outside = float(np.sum((y / r) ** 2)) > 1.0
    lo, hi = -float(np.min(r)) ** 2, (float(np.linalg.norm(r * y)) if outside else 0.0)
    while True:
        mid = 0.5 * (lo + hi)
        if mid in (lo, hi):
            break
        if float(np.sum((r * y / (mid + r * r)) ** 2)) > 1.0:
            lo = mid
        else:
            hi = mid
    q = r * r * y / (mid + r * r) * np.where(p < 0, -1.0, 1.0)
    n = q / (r * r)
    dist = float(np.linalg.norm(p - q))
    return (dist if outside else -dist), n / np.linalg.norm(n)


def s3d_cases():
    """Registry D-GJK-S3D in a contact: a sphere against an ellipsoid, and a
    capsule against an ellipsoid or a cylinder, overlapping, at S3D_POSES, called as mjc_Convex
    calls mjc_ccd (the pair's margin on both, one contact, no cutoff). A
    sphere and an ellipsoid also hold `exact_dist`, the sphere centre's
    signed distance from the ellipsoid less the radius and the margin (each
    geom is inflated by half of it)."""
    out = {}
    for k, (k1, s1, m1, k2, s2, p2, m2, margin) in enumerate(S3D_POSES):
        case = call([k1, k2], [s1, s2], [None, None], [([0.0, 0.0, 0.0], m1), (p2, m2)], margin,
                    config(1, 0.0))
        if case["out"]["nx"] != 1 or not case["out"]["dist"] < 0:
            sys.exit(f"s3d {k1}_{k2}_{k}: {case['out']}")
        if (k1, k2) == ("sphere", "ellipsoid"):
            mat = np.array(m2).reshape(3, 3)
            sd, _ = point_ellipsoid(mat.T @ -np.array(p2), s2)
            case["exact_dist"] = sd - s1[0] - margin
        out[f"{k1}_{k2}_{k}"] = case
    return out


def capped_case(meshes, assets, mesh, normal, point, toward, others, max_contacts, box_first):
    """A box of half sizes 0.02, 0.02, 0.01 whose -z face is turned to face `normal` at the
    apex of `mesh`, then 5e-4 about the axis in that face across `toward` (within the face and
    edge tolerances), the way that puts the apex deeper than each of `others`; its face's
    centre over `point`; brought to touch on MuJoCo's own distance (by bisection, from 5 mm in
    to 0.1 mm out), then pressed in by 1e-5."""
    verts = np.array(meshes[mesh]["vert"]).reshape(-1, 3)
    apex = max(range(len(verts)), key=lambda v: len(meshes[mesh]["polymap"][v]))
    size = [0.02, 0.02, 0.01]
    names = [mesh, None]
    m = mujoco.MjModel.from_xml_string(f"<mujoco>{assets}<worldbody>{geom_xml('mesh', [], mesh)}"
                                       f"{geom_xml('box', size, None)}</worldbody></mujoco>")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    d.geom_xpos[0] = 0.0
    d.geom_xmat[0] = np.eye(3).ravel()
    face_up = align_mat(np.array([0.0, 0.0, 1.0]), normal)
    for sign in (1, -1):
        turn = mat_mul(unit_turn(np.cross(normal, toward), sign * 5e-4), face_up)
        down = np.array(turn).reshape(3, 3) @ np.array([0.0, 0.0, -1.0])
        if all((verts[apex] - verts[o]) @ down < 0 for o in others):
            break
    else:
        sys.exit(f"polyvert_cap {mesh}: no turn puts the apex deepest")
    d.geom_xmat[1] = turn
    centre = point + np.array(turn).reshape(3, 3) @ np.array([0.0, 0.0, size[2]])
    lo, hi = -0.005, 1e-4
    if not place_dist(m, d, centre + lo * normal, names) < 0 < place_dist(m, d, centre + hi * normal,
                                                                           names):
        sys.exit(f"polyvert_cap {mesh}: no touch within the bracket")
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if place_dist(m, d, centre + mid * normal, names) < 0:
            lo = mid
        else:
            hi = mid
    d.geom_xpos[1] = centre + (hi - 1e-5) * normal
    geoms = (1, 0) if box_first else (0, 1)
    return run(m, d, geoms, 0.0, config(max_contacts, 0.0),
               [names[g] for g in geoms])


def polyvert_cap_cases(meshes):
    """The multi-contact's caps at a vertex: it reads the normals of at most mjMAX_POLYVERT
    faces there, and the edges of at most as many (engine_collision_gjk.c:1751, :1787). A box
    laid on the cone152 apex's polygon at polymap position 148 or 149 (both among those read)
    meets it face to face, three contacts; on the polygon at 150, past the cap, the face is not
    read, and the multi-contact falls back to an edge at the apex, two. A box holding the edge
    the fan155 apex's polygon at 149 contributes, turned halfway between that edge's two
    polygons, meets it edge to face, two contacts (that edge is read last, and no other at the
    apex is aligned); holding the edge of the polygon at 150 or 151, past the cap, nothing at
    the apex is read as aligned, one. Each with four and eight contacts, either geom first.
    It refuses a case of another count."""
    assets = graph_assets({**GRAPH_MESHES, **CAP_MESHES})
    out = {}
    cone = meshes["cone152"]
    verts = np.array(cone["vert"]).reshape(-1, 3)
    apex = max(range(len(verts)), key=lambda v: len(cone["polymap"][v]))
    for pos, want in ((148, 3), (149, 3), (150, 2)):
        poly = cone["polymap"][apex][pos]
        path = cone["polygons"][poly]
        base = [v for v in path if v != apex]
        mid = 0.5 * (verts[base[0]] + verts[base[1]])
        toward = (verts[apex] - mid) / np.linalg.norm(verts[apex] - mid)
        point = verts[apex] + 0.2 * (mid - verts[apex])
        for k in (4, 8):
            for box_first in (False, True):
                case = capped_case(meshes, assets, "cone152", np.array(cone["polynormals"][poly]),
                                   point, toward, base, k, box_first)
                if case["out"]["nx"] != want:
                    sys.exit(f"polyvert_cap face {pos}: {case['out']['nx']} contacts, not {want}")
                out[f"face_{pos}_{k}_{'box' if box_first else 'cone'}_first"] = case
    fan = meshes["fan155"]
    verts = np.array(fan["vert"]).reshape(-1, 3)
    apex = max(range(len(verts)), key=lambda v: len(fan["polymap"][v]))
    for pos, want in ((149, 2), (150, 1), (151, 1)):
        poly = fan["polymap"][apex][pos]
        path = fan["polygons"][poly]
        prev = path[path.index(apex) - 1]
        (other,) = [q for q in fan["polymap"][apex] if q != poly and prev in fan["polygons"][q]]
        edge = (verts[prev] - verts[apex]) / np.linalg.norm(verts[prev] - verts[apex])
        normal = np.array(fan["polynormals"][poly]) + np.array(fan["polynormals"][other])
        normal -= (normal @ edge) * edge
        normal /= np.linalg.norm(normal)
        point = verts[apex] + 0.2 * (verts[prev] - verts[apex])
        for k in (4, 8):
            for box_first in (False, True):
                case = capped_case(meshes, assets, "fan155", normal, point, edge, [prev], k,
                                   box_first)
                if case["out"]["nx"] != want:
                    sys.exit(f"polyvert_cap edge {pos}: {case['out']['nx']} contacts, not {want}")
                out[f"edge_{pos}_{k}_{'box' if box_first else 'fan'}_first"] = case
    return out


def coverage(cases, graph):
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
        # What the groups after `merge` are for.
        "P2_NONCONVEX": any(o["epa_status"] == 2 for o in outs),
        "P3_INVALID_V4": any(o["epa_status"] == 5 for o in outs),
        "P3_INVALID_V5": any(o["epa_status"] == 6 for o in outs),
        "P3_ORIGIN_ON_FACE": any(o["epa_status"] == 8 for o in outs),
        "an exact touch inflated to the cutoff": any(
            c["out"]["dist"] == 0.0 == c["config"]["dist_cutoff"] and c["out"]["nx"] == 1
            and c["out"]["epa_status"] == -1 for c in shrink),
        "a geom of size 0": any(0.0 in o["size"][:{"cylinder": 2}.get(o["type"], 3)]
                                for c in cases for o in c["obj"]
                                if o["type"] in ("ellipsoid", "cylinder", "box")),
        "a turn of rank below 3": any(np.isfinite(o["mat"]).all()
                                      and abs(np.linalg.det(np.reshape(o["mat"], (3, 3)))) < 1e-9
                                      for c in cases for o in c["obj"]),
        "a box past size 1 in EPA": any(
            c["out"]["epa_status"] == 0 and any(o["type"] == "box" and max(o["size"]) > 1
                                                for o in c["obj"]) for c in cases),
        "geoms below 1e-6": any(max(abs(x) for o in c["obj"] for x in o["size"]) < 1e-6
                                and c["out"]["nx"] == 1 for c in cases),
        "geoms past 1e150": any(max(abs(x) for o in c["obj"] for x in o["size"]) > 1e150
                                and c["out"]["nx"] == 1 for c in cases),
        "EPA past mjMAX_EPA_ITERATIONS": any(o["epa_iterations"] > 170 for o in outs),
        "mjMAXCONPAIR contacts": any(o["nx"] == MAXCONPAIR for o in outs),
        "a multi-contact on a mesh with a face past mjMAX_POLYVERT corners": any(
            c["out"]["nx"] > 1 and any(max(map(len, graph[o["mesh"]]["polygons"])) > 150
                                       for o in c["obj"] if o["mesh"] in graph)
            for c in cases),
        "a mesh of mjMESH_HILLCLIMB_MIN vertices hill-climbed": any(
            len(graph[o["mesh"]]["vert"]) // 3 == 10 and graph[o["mesh"]]["hillclimb"]
            for c in cases for o in c["obj"] if o["mesh"] in graph),
        "a multi-contact on the notched slab": any(
            c["out"]["nx"] > 1 and any(o["mesh"] == "notch9" for o in c["obj"]) for c in cases),
    }
    missing = [k for k, v in need.items() if not v]
    if missing:
        sys.exit(f"the cases miss: {', '.join(missing)}")
    hist = {}
    for o in outs:
        key = (o["epa_status"], o["nsimplex"], o["nx"])
        hist[key] = hist.get(key, 0) + 1
    return hist


class Contact(ctypes.Structure):
    """`mjContact` (mjdata.h:127-158)."""
    _fields_ = [("dist", c_double), ("pos", c_double * 3), ("frame", c_double * 9),
                ("includemargin", c_double), ("friction", c_double * 5), ("solref", c_double * 2),
                ("solreffriction", c_double * 2), ("solimp", c_double * 5), ("mu", c_double),
                ("H", c_double * 36), ("dim", c_int), ("geom1", c_int), ("geom2", c_int),
                ("geom", c_int * 2), ("flex", c_int * 2), ("elem", c_int * 2), ("vert", c_int * 2),
                ("exclude", c_int), ("efc_address", c_int)]


# mjc_PlaneConvex is not exported; MuJoCo's collision table is (engine_collision_driver.c:41-52),
# and holds it for a plane with an ellipsoid and with a mesh.
COLLISION = (c_void_p * 81).in_dll(LIB, "mjCOLLISIONFUNC")
if COLLISION[0 * 9 + 4] is None or COLLISION[0 * 9 + 4] != COLLISION[0 * 9 + 7]:
    sys.exit("mjCOLLISIONFUNC: no one function for a plane with an ellipsoid and a mesh")
PLANE_CONVEX = CFUNCTYPE(c_int, c_void_p, c_void_p, POINTER(Contact), c_int, c_int, c_double)(
    COLLISION[0 * 9 + 4])
PLANE_MESHES = {**MESHES, **GRAPH_MESHES}
# A tetrahedron on a base of three corners in its plane z = -0.03, for the `tetra` cases
# (kept out of `PLANE_MESHES`, whose cases draw from one generator).
TETRA_MESHES = {"tetra4": np.array([[0.0, 0.0, 0.08], [0.06, 0.0, -0.03], [-0.03, 0.05, -0.03],
                                    [-0.03, -0.05, -0.03]])}
ALL_PLANE_MESHES = {**PLANE_MESHES, **TETRA_MESHES}


def tetra_assets():
    return "<asset>" + "".join(
        f'<mesh name="{n}" vertex="{" ".join(repr(float(x)) for x in v.ravel())}"/>'
        for n, v in TETRA_MESHES.items()) + "</asset>"


def plane_mesh_model(name, faces=None):
    """A plane (geom 0) and the mesh `name` (geom 1): colliding, so MuJoCo makes its hull graph;
    or, given `faces`, with those faces and not colliding (contype and conaffinity 0, in no
    pair), so it makes none (user_model.cc:4915-4921, user_mesh.cc:1567)."""
    if faces is None:
        return mujoco.MjModel.from_xml_string(
            f'<mujoco>{mesh_assets()}{graph_assets()}{tetra_assets()}<worldbody>'
            f'<geom type="plane" size="5 5 0.1"/><geom type="mesh" mesh="{name}"/>'
            '</worldbody></mujoco>')
    vert = " ".join(repr(float(x)) for x in ALL_PLANE_MESHES[name].ravel())
    face = " ".join(str(i) for f in faces for i in f)
    return mujoco.MjModel.from_xml_string(
        f'<mujoco><asset><mesh name="{name}" vertex="{vert}" face="{face}"/></asset><worldbody>'
        f'<geom type="plane" size="5 5 0.1"/><geom type="mesh" mesh="{name}" contype="0"'
        ' conaffinity="0"/></worldbody></mujoco>')


def plane_meshes():
    """Each of `PLANE_MESHES` as MuJoCo compiles it: its vertices, its hull graph where its geom
    collides (`vert_globalid` and each hull vertex's neighbours, as `mjccd_support` and
    `mjc_PlaneConvex` walk them), its hull's faces (in the authored vertices' numbering) and its
    bounding radius (`geom_rbound`, in its mesh frame). Given those faces and not colliding, it
    compiles to the same vertices and radius and no graph."""
    out = {}
    for name in ALL_PLANE_MESHES:
        m = plane_mesh_model(name)
        mid = m.geom_dataid[1]
        ga = m.mesh_graphadr[mid]
        nv, nf = int(m.mesh_graph[ga]), int(m.mesh_graph[ga + 1])
        fa = ga + 2 + 3 * nv + 3 * nf
        faces = [[int(m.mesh_graph[fa + 3 * k + j]) for j in range(3)] for k in range(nf)]
        bare = plane_mesh_model(name, faces)
        bid = bare.geom_dataid[1]
        adr, num = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        verts = np.asarray(m.mesh_vert[adr:adr + num], dtype=np.float64)
        bare_verts = np.asarray(bare.mesh_vert[bare.mesh_vertadr[bid]:][:bare.mesh_vertnum[bid]],
                                dtype=np.float64)
        if bare.mesh_graphadr[bid] >= 0 or ga < 0:
            sys.exit(f"plane mesh {name}: graphs {ga}, {bare.mesh_graphadr[bid]}")
        if bare_verts.shape != verts.shape or (bare_verts != verts).any():
            sys.exit(f"plane mesh {name}: compiled without a graph, other vertices")
        if m.geom_rbound[1] != bare.geom_rbound[1]:
            sys.exit(f"plane mesh {name}: another radius without a graph")
        edgeadr = [int(x) for x in m.mesh_graph[ga + 2:ga + 2 + nv]]
        globalid = [int(x) for x in m.mesh_graph[ga + 2 + nv:ga + 2 + 2 * nv]]
        localid = m.mesh_graph[ga + 2 + 2 * nv:]
        neighbours = []
        for v in range(nv):
            k, around = edgeadr[v], []
            while localid[k] >= 0:
                around.append(int(localid[k]))
                k += 1
            neighbours.append(around)
        out[name] = {"vert": [float(x) for x in verts.ravel()],
                     "graph": {"vertex": globalid, "neighbours": neighbours}, "faces": faces,
                     "rbound": float(m.geom_rbound[1])}
    return out


SENTINEL = -1.0e300


def plane_call(m, d, plane, obj, margin):
    """MuJoCo's mjc_PlaneConvex on the plane (geom 0) and geom 1 of `m` at the given poses; the
    contacts it lists (`dist`, `pos`, the frame's normal)."""
    for g, (pos, mat) in enumerate((plane, obj)):
        d.geom_xpos[g] = pos
        d.geom_xmat[g] = mat
    con = (Contact * 4)()
    for c in con:
        c.dist = SENTINEL
        c.frame[3] = SENTINEL
    n = PLANE_CONVEX(m._address, d._address, con, 0, 1, margin)
    normal = [plane[1][2], plane[1][5], plane[1][8]]
    # The layout is right only if each contact's writes land where they are read: the normal
    # in the first three of the frame and zeros in the next three, nothing past the last.
    for k in range(n):
        if list(con[k].frame[3:6]) != [0.0, 0.0, 0.0] or not all(
                a == b or (a != a and b != b) for a, b in zip(con[k].frame[:3], normal)):
            sys.exit(f"mjContact layout: contact {k} of {n}")
    if n >= 4 or con[n].dist != SENTINEL or con[n].frame[3] != SENTINEL:
        sys.exit(f"mjc_PlaneConvex wrote past its {n} contacts")
    return [{"dist": con[k].dist, "pos": list(con[k].pos), "normal": list(con[k].frame[:3])}
            for k in range(n)]


def plane_case(m, d, kind, size, mesh, plane, obj, margin):
    return {"type": kind, "size": [float(x) for x in size], "mesh": mesh,
            "plane": {"pos": [float(x) for x in plane[0]], "mat": [float(x) for x in plane[1]]},
            "pos": [float(x) for x in obj[0]], "mat": [float(x) for x in obj[1]],
            "margin": margin, "contacts": plane_call(m, d, plane, obj, margin)}


def random_plane(rng):
    return rng.uniform(-1, 1, 3), random_mat(rng)


def in_plane(rng, plane_mat, spread):
    """A random offset along the plane's two tangents."""
    t = np.reshape(plane_mat, (3, 3))
    return spread * (rng.uniform(-1, 1) * t[:, 0] + rng.uniform(-1, 1) * t[:, 1])


def face_down(rng, plane_mat, mesh_mat):
    """`mesh_mat` turned so that the mesh's local -z faces the plane, about the plane's normal
    by a random angle."""
    spin = np.reshape(axis_mat([0, 0, 1], rng.uniform(0, 2 * np.pi)), (3, 3))
    return (np.reshape(plane_mat, (3, 3)) @ spin @ np.reshape(mesh_mat, (3, 3))).ravel()


def plane_convex_cases(meshes):
    """The plane cases (module docs): `ellipsoid`, `ellipsoid_at_margin`, `mesh`, `mesh_bare`,
    `edge`, `nan`."""
    rng = np.random.default_rng(51)
    out = {"ellipsoid": {}, "ellipsoid_at_margin": {}, "mesh": {}, "mesh_bare": {}, "edge": {},
           "nan": {}}
    m = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="plane" size="5 5 0.1"/>'
        '<geom type="ellipsoid" size="0.1 0.1 0.1"/></worldbody></mujoco>')
    d = mujoco.MjData(m)
    for i in range(200):
        size = rng.uniform(0.02, 0.5, 3)
        m.geom_size[1] = size
        plane, mat = random_plane(rng), random_mat(rng)
        normal = plane[1][[2, 5, 8]]
        reach = np.linalg.norm(size * (np.reshape(mat, (3, 3)).T @ normal))
        pos = plane[0] + normal * (reach + rng.uniform(-0.05, 0.05)) + in_plane(rng, plane[1], 1.0)
        margin = float(rng.choice([0.0, 0.01, 0.1]))
        out["ellipsoid"][f"e{i}"] = plane_case(m, d, "ellipsoid", size, None, plane,
                                               (pos, mat), margin)
        # The same pose at a margin equal to MuJoCo's own distance, listed (inclusive).
        dist = plane_call(m, d, plane, (pos, mat), 1.0)[0]["dist"]
        if dist > 0:
            case = plane_case(m, d, "ellipsoid", size, None, plane, (pos, mat), dist)
            if len(case["contacts"]) != 1:
                sys.exit(f"ellipsoid at margin e{i}: {len(case['contacts'])} contacts")
            out["ellipsoid_at_margin"][f"e{i}"] = case
    for name in PLANE_MESHES:
        mesh = meshes[name]
        verts = np.reshape(mesh["vert"], (-1, 3))
        for group, faces in (("mesh", None), ("mesh_bare", mesh["faces"])):
            pm = plane_mesh_model(name, faces)
            pd = mujoco.MjData(pm)
            for i in range(24):
                plane = random_plane(rng)
                normal = plane[1][[2, 5, 8]]
                mat = random_mat(rng) if i % 3 else face_down(rng, plane[1], np.eye(3).ravel())
                low = (np.reshape(mat, (3, 3)) @ verts.T).T @ normal
                depth = rng.uniform(-0.01, 0.02)
                pos = plane[0] + normal * (-depth - low.min()) + in_plane(rng, plane[1], 0.5)
                margin = float(rng.choice([0.0, 0.02]))
                out[group][f"{name}_{i}"] = plane_case(pm, pd, "mesh", [0, 0, 0], name, plane,
                                                       (pos, mat), margin)
    # Edges of the comparisons. An unturned mesh over an unturned plane at height 0.25, at a
    # margin of 0.25 plus its lowest vertex's z: each lowest vertex's distance is the margin
    # exactly (each sum and difference is exact here), so the first is listed (`dist <= margin`)
    # and the others are not (`vdot > threshold`), with a graph and without. And a mesh without a
    # graph apart from the plane by 0.7 of its radius at a margin of its radius: its first vertex
    # is farther than 0.3 of the radius from the first contact (half its distance), so only its
    # index keeps it from being listed again.
    out["edge"] = {}
    eye = np.eye(3).ravel()
    for name in ("slab8", "prism12"):
        verts = np.reshape(meshes[name]["vert"], (-1, 3))
        low = float(verts[:, 2].min())
        for tag, faces in (("graph", None), ("bare", meshes[name]["faces"])):
            pm = plane_mesh_model(name, faces)
            pd = mujoco.MjData(pm)
            out["edge"][f"{name}_{tag}_at_margin"] = plane_case(
                pm, pd, "mesh", [0, 0, 0], name, (np.zeros(3), eye),
                (np.array([0.0, 0.0, 0.25]), eye), 0.25 + low)
    for name in ("gem8", "pebble24"):
        verts = np.reshape(meshes[name]["vert"], (-1, 3))
        r = meshes[name]["rbound"]
        pm = plane_mesh_model(name, meshes[name]["faces"])
        pd = mujoco.MjData(pm)
        for i in range(4):
            plane, mat = random_plane(rng), random_mat(rng)
            normal = plane[1][[2, 5, 8]]
            low = ((np.reshape(mat, (3, 3)) @ verts.T).T @ normal).min()
            pos = plane[0] + normal * (0.7 * r - low)
            out["edge"][f"{name}_bare_apart_{i}"] = plane_case(pm, pd, "mesh", [0, 0, 0], name,
                                                               plane, (pos, mat), r)
    bad = eye.copy()
    bad[1] = math.nan
    # A plane's normal is its matrix's third column (mat[2], mat[5], mat[8]).
    bad_normal = eye.copy()
    bad_normal[2] = math.nan
    up = np.array([0.0, 0.0, 0.01])
    for name, bare in (("slab8", False), ("pebble24", False), ("slab8", True)):
        pm = plane_mesh_model(name, meshes[name]["faces"] if bare else None)
        pd = mujoco.MjData(pm)
        tag = f"{name}_bare" if bare else name
        out["nan"][f"{tag}_turn"] = plane_case(pm, pd, "mesh", [0, 0, 0], name,
                                               (np.zeros(3), eye), (up, bad), 0.0)
        out["nan"][f"{tag}_plane"] = plane_case(pm, pd, "mesh", [0, 0, 0], name,
                                                (np.zeros(3), bad_normal), (up, eye), 0.0)
    m.geom_size[1] = [0.05, 0.04, 0.03]
    out["nan"]["ellipsoid_turn"] = plane_case(m, d, "ellipsoid", m.geom_size[1], None,
                                              (np.zeros(3), eye), (up, bad), 0.0)
    out["tetra"] = tetra_cases(meshes)
    plane_coverage(out, meshes)
    return out


def tetra_cases(meshes):
    """The tetrahedron with each of its faces down on a plane at three spins, at six random
    turns (margins 0 and 0.02), and with its local -z face down at 12 (margins 0, 0.05 and 1,
    the last taking every vertex within it); with its graph and without. Its own
    generator."""
    rng = np.random.default_rng(53)
    out = {}
    mesh = meshes["tetra4"]
    verts = np.reshape(mesh["vert"], (-1, 3))
    centre = verts.mean(axis=0)
    for group, faces in (("graph", None), ("bare", mesh["faces"])):
        pm = plane_mesh_model("tetra4", faces)
        pd = mujoco.MjData(pm)
        poses = []
        for k, f in enumerate(mesh["faces"]):
            a, b, c = verts[f[0]], verts[f[1]], verts[f[2]]
            n = np.cross(b - a, c - a)
            if n @ (a - centre) < 0:
                n = -n
            for spin in range(3):
                poses.append((f"face{k}_{spin}", n))
        poses += [(f"random{i}", None) for i in range(6)]
        poses += [(f"local_down{i}", "local") for i in range(12)]
        for tag, n in poses:
            plane = random_plane(rng)
            normal = plane[1][[2, 5, 8]]
            if n is None:
                mat = random_mat(rng)
            elif isinstance(n, str):
                mat = face_down(rng, plane[1], np.eye(3).ravel())
            else:
                down = np.reshape(align_mat(n, np.array([0.0, 0.0, -1.0])), (3, 3))
                spin = np.reshape(axis_mat([0, 0, 1], rng.uniform(0, 2 * np.pi)), (3, 3))
                mat = (np.reshape(plane[1], (3, 3)) @ spin @ down).ravel()
            low = (np.reshape(mat, (3, 3)) @ verts.T).T @ normal
            depth = rng.uniform(-0.004, 0.01)
            pos = plane[0] + normal * (-depth - low.min()) + in_plane(rng, plane[1], 0.5)
            margin = float(rng.choice([0.0, 0.02] if n is None or not isinstance(n, str)
                                      else [0.0, 0.05, 1.0]))
            out[f"{group}_{tag}"] = plane_case(pm, pd, "mesh", [0, 0, 0], "tetra4", plane,
                                               (pos, mat), margin)
    # A pose (found by chunk 3e's fourth review) where two base corners tie exactly for the
    # deepest and the third is a little higher: MuJoCo's climb from the apex sweeps the apex's
    # neighbours in full and ends at the tie it meets first; one that moved on at the first
    # better neighbour would end at the other.
    pm = plane_mesh_model("tetra4")
    plane = (np.array([-0.2603940917362395, 0.41086838762342937, 0.36293235617641084]),
             np.array([0.34719917119006816, 0.6923025713999409, 0.6325898237862821,
                       -0.8711983857691097, -0.01154522591796997, 0.4907953549003909,
                       0.34708227865770863, -0.7215149737819256, 0.5991243897975109]))
    obj = (np.array([-0.07339101648023977, 0.04491025035578622, 0.49002397306165996]),
           np.array([-0.7690295370629232, 0.0917806404794969, 0.6325898237862821,
                     0.3063771337310541, -0.8156304135666739, 0.4907953549003909,
                     0.5610050116099945, 0.5712471816107544, 0.5991243897975109]))
    out["graph_tie"] = plane_case(pm, mujoco.MjData(pm), "mesh", [0, 0, 0], "tetra4", plane,
                                  obj, 1.0)
    return out


def plane_coverage(out, meshes):
    """Refuse plane cases that miss a path the port must be checked on."""
    counts = {g: {len(c["contacts"]) for c in out[g].values()} for g in out}
    need = {"an ellipsoid listed and not": counts["ellipsoid"] >= {0, 1},
            "an ellipsoid at its distance": len(out["ellipsoid_at_margin"]) >= 50,
            "1, 2 and 3 mesh contacts on the graph": counts["mesh"] >= {0, 1, 2, 3},
            "1, 2 and 3 mesh contacts without one": counts["mesh_bare"] >= {0, 1, 2, 3},
            "a NaN contact": any(c["contacts"] and c["contacts"][0]["dist"] != c["contacts"][0]["dist"]
                                 for c in out["nan"].values())}
    # A vertex closer than the margin but nearer the first contact than 0.3 of the radius, so
    # skipped (with 1e-6 of the radius to spare either way).
    near = 0
    for group in ("mesh", "mesh_bare"):
        for c in out[group].values():
            if not c["contacts"] or len(c["contacts"]) == 3:
                continue
            verts = np.reshape(meshes[c["mesh"]]["vert"], (-1, 3))
            mat = np.reshape(c["mat"], (3, 3))
            pts = (mat @ verts.T).T + c["pos"]
            normal = np.reshape(c["plane"]["mat"], (3, 3))[:, 2]
            dist = (pts - c["plane"]["pos"]) @ normal
            gap = np.linalg.norm(pts - c["contacts"][0]["pos"], axis=1)
            r = meshes[c["mesh"]]["rbound"]
            near += int(any((dist < c["margin"] - 1e-9) & (gap < 0.3 * r - 1e-6 * r) & (gap > 0)))
    need["a vertex skipped as too near the first"] = near > 0
    edge = out["edge"]
    need["each lowest vertex at the margin exactly, the first listed alone"] = all(
        len(c["contacts"]) == 1 and c["contacts"][0]["dist"] == c["margin"]
        for k, c in edge.items() if k.endswith("_at_margin"))
    need["a first vertex past the spacing without a graph"] = all(
        c["contacts"] and 0.5 * abs(c["contacts"][0]["dist"]) > 0.3 * meshes[c["mesh"]]["rbound"]
        for k, c in edge.items() if "_apart_" in k)
    need["the tetrahedron face down, listed with its graph"] = any(
        k.startswith("graph_face") and len(c["contacts"]) >= 2 for k, c in out["tetra"].items())
    missing = [k for k, v in need.items() if not v]
    if missing:
        sys.exit(f"the plane cases miss: {', '.join(missing)}")


# mjc_ConvexHField is not exported either; the collision table holds it for a height field with
# every convex type (engine_collision_driver.c:44).
if len({COLLISION[1 * 9 + t] for t in range(2, 8)}) != 1 or COLLISION[1 * 9 + 2] is None:
    sys.exit("mjCOLLISIONFUNC: no one function for a height field with the convex types")
HFIELD_CONVEX = CFUNCTYPE(c_int, c_void_p, c_void_p, POINTER(Contact), c_int, c_int, c_double)(
    COLLISION[1 * 9 + 2])


def bumpy9():
    """9 x 9, elevations multiples of 1/8 from 0 to 1, rows symmetric."""
    e = np.zeros((9, 9))
    for r in range(9):
        for c in range(9):
            e[r, c] = ((3 * min(r, 8 - r) + 5 * c) % 9) / 8.0
    e[0, 0], e[4, 4] = 0.0, 1.0
    return np.minimum(e, 1.0)


def noise(seed, nrow, ncol):
    return np.random.default_rng(seed).uniform(0, 1, (nrow, ncol))


# name: (nrow, ncol, size, elevation as authored): square cells, cells of two spacings, a fine
# field, and a single cell.
HFIELDS = {"bumpy9": (9, 9, (1.0, 1.0, 0.3, 0.1), bumpy9()),
           "uneven5x9": (5, 9, (0.8, 0.5, 0.2, 0.1), noise(59, 5, 9)),
           "fine30": (30, 30, (1.0, 1.0, 0.05, 0.1), noise(30, 30, 30)),
           "cell2": (2, 2, (0.5, 0.5, 0.2, 0.1), np.array([[0.0, 0.5], [1.0, 0.25]])),
           "flat5": (5, 5, (0.5, 0.5, 0.25, 0.125),
                     np.array([[0.0] * 5] * 4 + [[0.0, 0.0, 0.0, 0.0, 1.0]])),
           "large3": (3, 3, (1e153, 1e153, 1e153, 1e153), np.array([[0, 0, 0], [0, 1.0, 0], [0, 0, 0]])),
           "huge3": (3, 3, (1e154, 1e154, 1e154, 1e154), np.array([[0, 0, 0], [0, 1.0, 0], [0, 0, 0]]))}
# The fields the `pose` cases draw over (the others serve the `edge` cases only).
POSE_FIELDS = ("bumpy9", "uneven5x9", "fine30", "cell2")
HFIELD_GEOMS = ("sphere", "capsule", "ellipsoid", "cylinder", "box", "gem8", "pebble24")


def hfield_model(field, geom, size):
    """A height field (geom 0) and a convex geom (geom 1)."""
    nrow, ncol, hsize, elev = HFIELDS[field]
    body = geom_xml("mesh", None, geom) if geom in PLANE_MESHES else geom_xml(geom, size, None)
    return mujoco.MjModel.from_xml_string(
        f'<mujoco>{mesh_assets()}{graph_assets()}<asset><hfield name="f" nrow="{nrow}" '
        f'ncol="{ncol}" size="{" ".join(repr(x) for x in hsize)}" '
        f'elevation="{" ".join(repr(float(x)) for x in elev.ravel())}"/></asset><worldbody>'
        f'<geom type="hfield" hfield="f"/>{body}</worldbody></mujoco>')


def hfield_fields():
    """Each field as MuJoCo compiles it: its rows and columns, size and data (normalised to
    [0, 1] and its rows turned bottom to top, as floats)."""
    out = {}
    for field, (nrow, ncol, hsize, _) in HFIELDS.items():
        m = hfield_model(field, "sphere", [0.1])
        if (m.hfield_nrow[0], m.hfield_ncol[0]) != (nrow, ncol):
            sys.exit(f"hfield {field}: compiled to {m.hfield_nrow[0]} x {m.hfield_ncol[0]}")
        data = m.hfield_data[m.hfield_adr[0]:m.hfield_adr[0] + nrow * ncol]
        out[field] = {"nrow": nrow, "ncol": ncol, "size": [float(x) for x in m.hfield_size[0]],
                      "data": [float(x) for x in data]}
    return out


def lowest(kind, size, rel, verts):
    """How far below its centre a geom turned by `rel` (in the field's frame) reaches."""
    if verts is not None:
        return -float(((rel @ verts.T).T)[:, 2].min())
    z = rel[2]
    if kind == "sphere":
        return size[0]
    if kind == "capsule":
        return size[0] + size[1] * abs(z[2])
    if kind == "ellipsoid":
        return float(np.sqrt(sum((z[i] * size[i]) ** 2 for i in range(3))))
    if kind == "cylinder":
        return size[0] * math.sqrt(max(0.0, 1 - z[2] ** 2)) + size[1] * abs(z[2])
    return float(sum(abs(z[i]) * size[i] for i in range(3)))


def hfield_call(m, d, field_pose, obj_pose, margin):
    """MuJoCo's mjc_ConvexHField on the field (geom 0) and geom 1 at the given poses: its
    contacts (`dist`, `pos`, the frame's normal)."""
    for g, (pos, mat) in enumerate((field_pose, obj_pose)):
        d.geom_xpos[g] = pos
        d.geom_xmat[g] = mat
    con = (Contact * (MAXCONPAIR + 1))()
    for c in con:
        c.dist = SENTINEL
        c.frame[3] = SENTINEL
    n = HFIELD_CONVEX(m._address, d._address, con, 0, 1, margin)
    if n > MAXCONPAIR or con[n].dist != SENTINEL or con[n].frame[3] != SENTINEL:
        sys.exit(f"mjc_ConvexHField wrote past its {n} contacts")
    for k in range(n):
        if list(con[k].frame[3:6]) != [0.0, 0.0, 0.0]:
            sys.exit(f"mjContact layout: contact {k} of {n}")
    # Its poses back as they were: mjc_ConvexHField moves geom 1 into the field's frame and back.
    if (d.geom_xpos[1] != obj_pose[0]).any() and not np.isnan(obj_pose[0]).any():
        sys.exit("mjc_ConvexHField left geom 1 moved")
    return [{"dist": con[k].dist, "pos": list(con[k].pos), "normal": list(con[k].frame[:3])}
            for k in range(n)]


def hfield_case(field, geom, size, field_pose, obj_pose, margin):
    m = hfield_model(field, geom, size)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mesh = geom if geom in PLANE_MESHES else None
    return {"field": field, "type": "mesh" if mesh else geom,
            "size": [float(x) for x in m.geom_size[1]], "mesh": mesh,
            "rbound": float(m.geom_rbound[1]),
            "hfield": {"pos": [float(x) for x in field_pose[0]],
                       "mat": [float(x) for x in field_pose[1]]},
            "pos": [float(x) for x in obj_pose[0]], "mat": [float(x) for x in obj_pose[1]],
            "margin": margin, "ccd": [int(m.opt.ccd_iterations), float(m.opt.ccd_tolerance)],
            "contacts": hfield_call(m, d, field_pose, obj_pose, margin)}


def hfield_convex_cases(meshes, fields):
    """The height-field cases (module docs): `pose`, `edge`, `nan`."""
    rng = np.random.default_rng(52)
    out = {"pose": {}, "edge": {}, "nan": {}}
    for field in POSE_FIELDS:
        nrow, ncol, hsize, _ = HFIELDS[field]
        data = np.reshape(fields[field]["data"], (nrow, ncol))
        for geom in HFIELD_GEOMS:
            verts = np.reshape(meshes[geom]["vert"], (-1, 3)) if geom in PLANE_MESHES else None
            for i in range(12):
                size = random_size(rng, geom) if verts is None else []
                field_pose = (rng.uniform(-1, 1, 3), random_mat(rng) if i % 2 else np.eye(3).ravel())
                rel = np.reshape(random_mat(rng), (3, 3))
                x, y = rng.uniform(-1.1, 1.1) * hsize[0], rng.uniform(-1.1, 1.1) * hsize[1]
                c = int(np.clip(round((x + hsize[0]) / (2 * hsize[0]) * (ncol - 1)), 0, ncol - 1))
                r = int(np.clip(round((y + hsize[1]) / (2 * hsize[1]) * (nrow - 1)), 0, nrow - 1))
                z = data[r, c] * hsize[2] + lowest(geom, size, rel, verts) + rng.uniform(-0.03, 0.02)
                fmat = np.reshape(field_pose[1], (3, 3))
                obj_pose = (field_pose[0] + fmat @ np.array([x, y, z]), (fmat @ rel).ravel())
                margin = float(rng.choice([0.0, 0.01]))
                out["pose"][f"{field}_{geom}_{i}"] = hfield_case(field, geom, size, field_pose,
                                                                 obj_pose, margin)
    eye = np.eye(3).ravel()
    origin = np.zeros(3)
    # A broad flat box pressed into the fine field: more prisms overlap than mjMAXCONPAIR.
    out["edge"]["fine30_broad_box"] = hfield_case(
        "fine30", "box", [0.6, 0.6, 0.05], (origin, eye), (np.array([0.0, 0.0, 0.07]), eye), 0.0)
    # Clear of the field's box by its bounding sphere; within it but clear by its box (a long
    # capsule lying past the field's edge); above the field within the margin only.
    out["edge"]["bumpy9_far"] = hfield_case("bumpy9", "sphere", [0.1], (origin, eye),
                                            (np.array([3.0, 0.0, 0.2]), eye), 0.0)
    out["edge"]["bumpy9_capsule_past_edge"] = hfield_case(
        "bumpy9", "capsule", [0.05, 0.6], (origin, eye),
        (np.array([1.2, 0.0, 0.2]), axis_mat([1, 0, 0], np.pi / 2)), 0.0)
    out["edge"]["cell2_sphere_in_margin"] = hfield_case(
        "cell2", "sphere", [0.1], (origin, eye), (np.array([0.0, 0.0, 0.42]), eye), 0.2)
    # Past the field's +x edge, its box within the margin of the field's: the sub-grid starts
    # at the last column, and MuJoCo lists nothing.
    out["edge"]["bumpy9_sphere_past_edge_in_margin"] = hfield_case(
        "bumpy9", "sphere", [0.1], (origin, eye), (np.array([1.15, 0.0, 0.3]), eye), 0.3)
    # A sphere sunk into a field's centre bump, at 1e153 and at 1e154: at the latter the
    # squared norm of the witness points' difference overflows, and MuJoCo skips the
    # contacts whose normal is then zero.
    for field in ("large3", "huge3"):
        s = HFIELDS[field][2][0]
        out["edge"][f"{field}_sphere"] = hfield_case(
            field, "sphere", [0.6 * s], (origin, eye), (np.array([0.0, 0.0, 0.5 * s]), eye), 0.0)
    # Boxes resting exactly on the flat part of a dyadic field (height 0, cells 0.25), on a
    # grid vertex, on a grid line and mid-cell: they touch, and MuJoCo lists no contact (its
    # distance must be below 0). And a sphere under the field's base by less than the margin.
    for x, y in ((-0.25, 0.0), (-0.125, 0.0), (-0.125, 0.125), (-0.25, 0.25)):
        out["edge"][f"flat5_box_touching_{x}_{y}"] = hfield_case(
            "flat5", "box", [0.0625, 0.0625, 0.125], (origin, eye),
            (np.array([x, y, 0.125]), eye), 0.0)
    out["edge"]["flat5_sphere_under_base"] = hfield_case(
        "flat5", "sphere", [0.05], (origin, eye), (np.array([-0.125, 0.125, -0.18]), eye),
        0.02)
    bad = eye.copy()
    bad[1] = math.nan
    up = np.array([0.1, 0.05, 0.25])
    for geom, size in (("sphere", [0.1]), ("box", [0.08, 0.06, 0.05]), ("gem8", [])):
        out["nan"][f"{geom}_turn"] = hfield_case("bumpy9", geom, size, (origin, eye), (up, bad), 0.0)
    out["nan"]["field_turn"] = hfield_case("bumpy9", "sphere", [0.1], (origin, bad), (up, eye), 0.0)
    hfield_coverage(out)
    return out


def hfield_coverage(out):
    """Refuse height-field cases that miss a path the port must be checked on."""
    pose = out["pose"].values()
    counts = [len(c["contacts"]) for c in pose]
    edge = out["edge"]
    need = {"every geom with a contact": all(any(c["contacts"] for c in pose if (c["mesh"] or c["type"]) == g)
                                             for g in HFIELD_GEOMS),
            "several contacts": max(counts) >= 3,
            "none": 0 in counts,
            # Apart from the field by more than its radius and less than the margin: listed, at
            # a distance below 0 (the prism raised by the margin, the geom inflated by half of
            # it, and the distance not raised back).
            "a geom apart listed within the margin": bool(edge["cell2_sphere_in_margin"]["contacts"])
            and all(x["dist"] < 0 for x in edge["cell2_sphere_in_margin"]["contacts"]),
            "mjMAXCONPAIR contacts": len(edge["fine30_broad_box"]["contacts"]) == MAXCONPAIR,
            "the far and past-edge geoms with none": not edge["bumpy9_far"]["contacts"]
            and not edge["bumpy9_capsule_past_edge"]["contacts"]
            and not edge["bumpy9_sphere_past_edge_in_margin"]["contacts"],
            "a NaN turn listing a contact": any(c["contacts"] for c in out["nan"].values()),
            "boxes touching exactly, unlisted": all(
                not c["contacts"] for k, c in edge.items() if "_touching_" in k),
            "a sphere under the base listed within the margin":
            bool(edge["flat5_sphere_under_base"]["contacts"]),
            "contacts at 1e153 and none at 1e154":
            bool(edge["large3_sphere"]["contacts"]) and not edge["huge3_sphere"]["contacts"]}
    missing = [k for k, v in need.items() if not v]
    if missing:
        sys.exit(f"the height-field cases miss: {', '.join(missing)}")


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
    doc["merge"] = merge_meshes()
    doc["graph_meshes"] = graph_meshes()
    doc.update({"small": small_cases(), "large": large_cases(), "touch": touch_cases(),
                "degenerate": degenerate_cases(), "long_epa": long_epa_cases(),
                "overflow": overflow_cases(), "flush": flush_cases(),
                "graph": graph_cases(doc["graph_meshes"]),
                "notch": notch_cases(doc["graph_meshes"])})
    doc["s3d"] = s3d_cases()
    doc["polyvert_cap"] = polyvert_cap_cases(doc["graph_meshes"])
    doc["plane_meshes"] = plane_meshes()
    doc["plane_convex"] = plane_convex_cases(doc["plane_meshes"])
    doc["hfield_fields"] = hfield_fields()
    doc["hfield_convex"] = hfield_convex_cases(doc["plane_meshes"], doc["hfield_fields"])
    cases = [c for k in GROUPS for c in doc[k].values()]
    hist = coverage(cases, doc["graph_meshes"])
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], "ccd.json"), finite_or_string(doc))
    print(f"{len(cases)} cases -> {sys.argv[1]}")
    for (status, nsimplex, nx), n in sorted(hist.items()):
        print(f"  epa_status {status:2} nsimplex {nsimplex} nx {nx}: {n}")


if __name__ == "__main__":
    main()
