#!/usr/bin/env python3
"""Generate the signed-distance-field golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I -B sim/L0/tests/scripts/gen_sdf_reference.py \\
        sim/L0/tests/assets/golden/sdf/sdf.json

It refuses any other interpreter, as gen_census_golden.py does. Every scene holds a geom of type `sdf` over a mesh
with no plugin (a loaded field: MuJoCo's compiler makes the mesh an octree, `mjCOctree`), and every moving body is
free, placed by its body's `pos`, so its first contacts are those of the written pose. Each scene's name says what it
holds: a field against each primitive `mjc_SDF` takes, turned and moved; a field that is not convex; two fields; a
mesh geom against a field (MuJoCo reads mesh number g1's octree, g1 the mesh geom's id); a plane against a field
(`mjc_PlaneConvex` over the mesh); a height field against a field (none); `sdf_initpoints` and `sdf_iterations`;
explicit pairs; rangefinders; the geom-distance sensors; a flex against a field.

`scenes` holds, per scene, its `xml` and, after `mj_forward`, MuJoCo's contacts (geoms, flexes, vertices, elements,
`dist`, `pos`, the 9-entry `frame`) and `sensordata`, then each of 5 `mj_step`s' `ncon` and `qpos`; or its refusal.
`fields` holds, per mesh, a scene with one field over it and, at each of its points (a grid over 1.3 times the
octree's root box, random points within three times it, points far along each axis and on the box's faces),
`mjc_distance` and `mjc_gradient` of the field alone (`mjSDFTYPE_SINGLE`, exported, called through ctypes on the
library the bindings loaded): `oct_distance` and `oct_gradient`. Doubles are their bits as 64-bit integers in hex.
"""
import ctypes
import json
import os
import random
import struct
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402

class MjSDF(ctypes.Structure):
    _fields_ = [("plugin", ctypes.POINTER(ctypes.c_void_p)), ("id", ctypes.POINTER(ctypes.c_int)),
                ("type", ctypes.c_int), ("relpos", ctypes.POINTER(ctypes.c_double)),
                ("relmat", ctypes.POINTER(ctypes.c_double)), ("geomtype", ctypes.POINTER(ctypes.c_int))]


def library():
    """The MuJoCo library the bindings loaded (the oracle's)."""
    d = os.path.dirname(mujoco.__file__)
    names = [n for n in os.listdir(d) if n.startswith("libmujoco") and (n.endswith(".dylib") or ".so" in n)]
    if len(names) != 1:
        sys.exit(f"expected one libmujoco beside the bindings, found {names}")
    lib = ctypes.CDLL(os.path.join(d, names[0]))
    dbl3 = ctypes.c_double * 3
    lib.mjc_distance.restype = ctypes.c_double
    lib.mjc_distance.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(MjSDF), dbl3]
    lib.mjc_gradient.restype = None
    lib.mjc_gradient.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(MjSDF), dbl3, dbl3]
    return lib


SCENES = {}

CUBE_V = "-0.1 -0.1 -0.1  0.1 -0.1 -0.1  0.1 0.1 -0.1  -0.1 0.1 -0.1  -0.1 -0.1 0.1  0.1 -0.1 0.1  0.1 0.1 0.1  -0.1 0.1 0.1"
CUBE_F = "0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  1 2 6  1 6 5  0 4 7  0 7 3"
# An L prism (not convex): the union of [0,0.2]x[0,0.1]x[0,0.1] and [0,0.1]x[0.1,0.2]x[0,0.1], 12 vertices.
L_V = ("0 0 0  0.2 0 0  0.2 0.1 0  0.1 0.1 0  0.1 0.2 0  0 0.2 0  "
       "0 0 0.1  0.2 0 0.1  0.2 0.1 0.1  0.1 0.1 0.1  0.1 0.2 0.1  0 0.2 0.1")
L_F = ("0 2 1  0 3 2  0 5 3  3 5 4  6 7 8  6 8 9  6 9 11  9 10 11  "
       "0 1 7  0 7 6  1 2 8  1 8 7  2 3 9  2 9 8  3 4 10  3 10 9  4 5 11  4 11 10  5 0 6  5 6 11")


def box_grid(n, h):
    """A box of half size h, each face an n x n grid of quads: more vertices than mjMESH_HILLCLIMB_MIN."""
    verts, index, faces = [], {}, []

    def vid(p):
        key = tuple(round(c, 12) for c in p)
        if key not in index:
            index[key] = len(verts)
            verts.append(p)
        return index[key]

    for axis in range(3):
        for sign in (-1, 1):
            u, v = [k for k in range(3) if k != axis]
            for i in range(n):
                for j in range(n):
                    def pt(a, b):
                        p = [0.0, 0.0, 0.0]
                        p[axis] = sign * h
                        p[u] = -h + 2 * h * a / n
                        p[v] = -h + 2 * h * b / n
                        return vid(p)
                    q = [pt(i, j), pt(i + 1, j), pt(i + 1, j + 1), pt(i, j + 1)]
                    # outward winding: (u, v, axis) right-handed for sign +1
                    rh = (u, v, axis) in ((0, 1, 2), (1, 2, 0), (2, 0, 1))
                    if (sign > 0) == rh:
                        faces += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
                    else:
                        faces += [(q[0], q[2], q[1]), (q[0], q[3], q[2])]
    vs = "  ".join(" ".join(f"{c:.17g}" for c in p) for p in verts)
    fs = "  ".join(" ".join(str(i) for i in f) for f in faces)
    return vs, fs


GRID_V, GRID_F = box_grid(3, 0.1)

ASSETS = {
    "cube": f'<mesh name="cube" vertex="{CUBE_V}" face="{CUBE_F}"/>',
    "ell": f'<mesh name="ell" vertex="{L_V}" face="{L_F}"/>',
    "grid": f'<mesh name="grid" vertex="{GRID_V}" face="{GRID_F}"/>',
}


def doc(name, assets, world, option="", extra="", flag=""):
    a = "\n    ".join(ASSETS[k] if k in ASSETS else k for k in assets)
    xml = f"""<mujoco model="{name}">
  <option {option}>{flag}</option>
  <asset>
    {a}
  </asset>
  <worldbody>
    {world}
  </worldbody>
  {extra}
</mujoco>
"""
    SCENES[name] = xml


SDF_CUBE = '<geom name="field" type="sdf" mesh="cube"/>'


def free(name, pos, geom, quat=None):
    q = f' quat="{quat}"' if quat else ""
    return f'<body name="{name}" pos="{pos}"{q}><freejoint/>{geom}</body>'


# --- primitives against a cube field on the world
doc("p01_sphere_into_top", ["cube"], SDF_CUBE + free("b", "0 0 0.14", '<geom type="sphere" size="0.05"/>'))
doc("p02_sphere_off_centre", ["cube"], SDF_CUBE + free("b", "0.07 -0.03 0.135", '<geom type="sphere" size="0.04"/>'))
doc("p03_sphere_at_corner", ["cube"], SDF_CUBE + free("b", "0.12 0.12 0.12", '<geom type="sphere" size="0.04"/>'))
doc("p04_sphere_apart", ["cube"], SDF_CUBE + free("b", "0 0 0.2", '<geom type="sphere" size="0.05"/>'))
doc("p05_sphere_deep", ["cube"], SDF_CUBE + free("b", "0 0 0.05", '<geom type="sphere" size="0.05"/>'))
doc("p06_capsule_flat", ["cube"], SDF_CUBE + free("b", "0 0 0.125", '<geom type="capsule" size="0.03 0.08"/>', "0.7071067811865476 0 0.7071067811865476 0"))
doc("p07_capsule_tilted", ["cube"], SDF_CUBE + free("b", "0.02 0.01 0.17", '<geom type="capsule" size="0.03 0.06"/>', "0.9238795325112867 0.3826834323650898 0 0"))
doc("p08_ellipsoid", ["cube"], SDF_CUBE + free("b", "0 0.03 0.14", '<geom type="ellipsoid" size="0.06 0.03 0.045"/>', "0.9659258262890683 0 0.25881904510252074 0"))
doc("p09_cylinder", ["cube"], SDF_CUBE + free("b", "-0.02 0 0.135", '<geom type="cylinder" size="0.05 0.04"/>'))
doc("p10_cylinder_on_side", ["cube"], SDF_CUBE + free("b", "0 0 0.14", '<geom type="cylinder" size="0.045 0.07"/>', "0.7071067811865476 0.7071067811865476 0 0"))
doc("p11_box", ["cube"], SDF_CUBE + free("b", "0.01 0.02 0.145", '<geom type="box" size="0.05 0.04 0.05"/>', "0.9659258262890683 0 0 0.25881904510252074"))
doc("p12_box_on_edge", ["cube"], SDF_CUBE + free("b", "0 0 0.16", '<geom type="box" size="0.04 0.04 0.04"/>', "0.9238795325112867 0.3826834323650898 0 0"))
# --- a turned, moved field
doc("t01_field_turned_sphere", ["cube"],
    '<body name="fb" pos="0.3 -0.2 0.1" quat="0.8660254037844387 0.2 0.3 0.3464101615137755"><geom name="field" type="sdf" mesh="cube"/></body>'
    + free("b", "0.3 -0.2 0.26", '<geom type="sphere" size="0.05"/>'))
doc("t02_field_free_box", ["cube"],
    free("fb", "0 0 0.2", '<geom name="field" type="sdf" mesh="cube"/>', "0.9659258262890683 0.25881904510252074 0 0")
    + '<geom type="box" size="0.2 0.2 0.05" pos="0 0 0.02"/>')
# --- a field not convex
doc("n01_ell_sphere_in_notch", ["ell"], '<geom name="field" type="sdf" mesh="ell"/>'
    + free("b", "0.15 0.15 0.05", '<geom type="sphere" size="0.06"/>'))
doc("n02_ell_capsule", ["ell"], '<geom name="field" type="sdf" mesh="ell"/>'
    + free("b", "0.1 0.05 0.13", '<geom type="capsule" size="0.03 0.1"/>', "0.7071067811865476 0.7071067811865476 0 0"))
doc("n03_grid_sphere", ["grid"], '<geom name="field" type="sdf" mesh="grid"/>'
    + free("b", "0.03 0 0.14", '<geom type="sphere" size="0.05"/>'))
# --- two fields
doc("s01_two_fields_same_mesh", ["cube"], SDF_CUBE + free("b", "0.05 0.02 0.19", '<geom type="sdf" mesh="cube"/>'))
doc("s02_two_fields_turned", ["cube"], SDF_CUBE + free("b", "0 0 0.22", '<geom type="sdf" mesh="cube"/>', "0.9238795325112867 0.2 0.3 0.1"))
doc("s03_two_fields_two_meshes", ["cube", "ell"], SDF_CUBE + free("b", "-0.05 -0.05 0.19", '<geom type="sdf" mesh="ell"/>'))
doc("s04_field_order_swapped", ["cube", "ell"], free("b", "-0.05 -0.05 0.19", '<geom type="sdf" mesh="ell"/>')
    + '<body name="w" pos="0 0 0"><geom name="field" type="sdf" mesh="cube"/></body>')
# --- a mesh geom against a field (ledger L442: MuJoCo reads mesh number g1's octree, g1 the mesh GEOM)
doc("m01_mesh_vs_field_same_mesh", ["cube"], SDF_CUBE + free("b", "0.02 0.01 0.195", '<geom type="mesh" mesh="cube"/>'))
doc("m02_mesh_geom0_mesh0_no_octree", ["grid", "cube"],
    free("b", "0.02 0.01 0.195", '<geom type="mesh" mesh="grid"/>') + '<geom name="field" type="sdf" mesh="cube"/>')
doc("m03_mesh_geom1_reads_mesh1", ["cube", "grid"],
    '<geom name="field" type="sdf" mesh="grid"/>' + free("b", "0.02 0.01 0.195", '<geom type="mesh" mesh="cube"/>'))
doc("m04_mesh_geom1_mesh1_no_octree", ["grid", "cube"],
    '<geom name="field" type="sdf" mesh="grid"/>' + free("b", "0.02 0.01 0.195", '<geom type="mesh" mesh="cube"/>'))
doc("m05_mesh_geom_past_meshes", ["cube"],
    '<geom name="floor" type="plane" size="1 1 0.1" pos="0 0 -1"/><geom name="g1" type="sphere" size="0.01" pos="0 0 -0.9"/>'
    + SDF_CUBE + free("b", "0.02 0.01 0.195", '<geom type="mesh" mesh="cube"/>'))
doc("m06_ell_mesh_vs_cube_field", ["cube", "ell"],
    '<geom name="field" type="sdf" mesh="cube"/>' + free("b", "-0.05 -0.05 0.14", '<geom type="mesh" mesh="ell"/>'))
# --- a plane against a field (mjc_PlaneConvex over the mesh)
doc("l01_cube_into_plane", ["cube"], '<geom type="plane" size="1 1 0.1"/>'
    + free("b", "0 0 0.09", '<geom type="sdf" mesh="cube"/>'))
doc("l02_cube_into_plane_turned", ["cube"], '<geom type="plane" size="1 1 0.1"/>'
    + free("b", "0 0 0.12", '<geom type="sdf" mesh="cube"/>', "0.9238795325112867 0.3826834323650898 0 0"))
doc("l03_grid_into_plane", ["grid"], '<geom type="plane" size="1 1 0.1"/>'
    + free("b", "0 0 0.095", '<geom type="sdf" mesh="grid"/>', "0.9961946980917455 0.08715574274765817 0 0"))
doc("l04_field_apart_within_margin", ["cube"], '<geom type="plane" size="1 1 0.1"/>'
    + free("b", "0 0 0.105", '<geom type="sdf" mesh="cube" margin="0.01"/>'))
doc("l05_visual_field_no_contype", ["cube"], '<geom type="plane" size="1 1 0.1"/>'
    + free("b", "0 0 0.09", '<geom type="sdf" mesh="cube" contype="0" conaffinity="0"/><geom type="sphere" size="0.02" contype="0" conaffinity="0"/>'))
# --- a height field against a field: MuJoCo warns and lists none
doc("h01_field_on_hfield", ['<hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.05" elevation="0 0 0 0 0 0 0 0 0"/>', "cube"],
    '<geom type="hfield" hfield="hf"/>' + free("b", "0 0 0.12", '<geom type="sdf" mesh="cube"/>'))
# --- options
doc("o01_initpoints_8", ["cube"], SDF_CUBE + free("b", "0 0 0.14", '<geom type="sphere" size="0.05"/>'), option='sdf_initpoints="8"')
doc("o02_iterations_2", ["cube"], SDF_CUBE + free("b", "0 0 0.14", '<geom type="sphere" size="0.05"/>'), option='sdf_iterations="2"')
doc("o06_initpoints_50_deep", ["cube"], SDF_CUBE + free("b", "0 0 0.06", '<geom type="box" size="0.15 0.15 0.08"/>'),
    option='sdf_initpoints="50"')
doc("o03_initpoints_60", ["cube"], SDF_CUBE + free("b", "0 0 0.12", '<geom type="box" size="0.09 0.09 0.03"/>'), option='sdf_initpoints="60"')
# --- an explicit pair
doc("x01_explicit_pair", ["cube"], SDF_CUBE + free("b", "0 0 0.14", '<geom name="ball" type="sphere" size="0.05" contype="0" conaffinity="0"/>'),
    extra='<contact><pair geom1="field" geom2="ball"/></contact>')
doc("x02_explicit_pair_reversed", ["cube"], SDF_CUBE + free("b", "0 0 0.14", '<geom name="ball" type="sphere" size="0.05" contype="0" conaffinity="0"/>'),
    extra='<contact><pair geom1="ball" geom2="field"/></contact>')
# --- rays
RF = '<site name="s" pos="{p}" zaxis="{z}"/>'
doc("r01_rangefinder_above", ["cube"], SDF_CUBE + '<body name="eye" pos="0 0 0.5">' + RF.format(p="0 0 0", z="0 0 -1") + '</body>',
    extra='<sensor><rangefinder site="s"/></sensor>')
doc("r02_rangefinder_slanted", ["cube"], SDF_CUBE + '<body name="eye" pos="0.3 0.1 0.4">' + RF.format(p="0 0 0", z="-0.6 -0.2 -0.7") + '</body>',
    extra='<sensor><rangefinder site="s"/></sensor>')
doc("r03_rangefinder_miss", ["cube"], SDF_CUBE + '<body name="eye" pos="0.3 0 0.5">' + RF.format(p="0 0 0", z="0 0 -1") + '</body>',
    extra='<sensor><rangefinder site="s"/></sensor>')
doc("r04_rangefinder_turned_field", ["ell"],
    '<body name="fb" pos="0 0 0" quat="0.9238795325112867 0 0 0.3826834323650898"><geom name="field" type="sdf" mesh="ell"/></body>'
    + '<body name="eye" pos="0.05 0.15 0.4">' + RF.format(p="0 0 0", z="0 0 -1") + '</body>',
    extra='<sensor><rangefinder site="s"/></sensor>')
doc("r05_rangefinder_from_inside", ["cube"], SDF_CUBE + '<body name="eye" pos="0 0 0">' + RF.format(p="0 0 0", z="1 0 0") + '</body>',
    extra='<sensor><rangefinder site="s"/></sensor>')
# --- geom-distance sensors
doc("d01_distance_sphere", ["cube"], SDF_CUBE + free("b", "0 0 0.14", '<geom name="ball" type="sphere" size="0.05"/>'),
    extra='<sensor><distance geom1="ball" geom2="field" cutoff="1"/><normal geom1="ball" geom2="field" cutoff="1"/><fromto geom1="field" geom2="ball" cutoff="1"/></sensor>')
doc("d02_distance_plane", ["cube"], '<geom name="floor" type="plane" size="1 1 0.1"/>'
    + free("b", "0 0 0.2", '<geom name="field" type="sdf" mesh="cube"/>'),
    extra='<sensor><distance geom1="floor" geom2="field" cutoff="1"/></sensor>')
# --- a mesh written moved and turned (MuJoCo keeps it in its own frame; the octree is made there)
ASSETS["cube_moved"] = (f'<mesh name="cube_moved" vertex="{CUBE_V}" face="{CUBE_F}" refpos="0.02 -0.01 0.03" '
                        'refquat="0.9659258262890683 0 0.25881904510252074 0"/>')
doc("t03_mesh_frame_moved_turned", ["cube_moved"], '<geom name="field" type="sdf" mesh="cube_moved"/>'
    + free("b", "0.03 0 0.15", '<geom type="sphere" size="0.05"/>'))
# --- a flex against a field
doc("f01_flex_on_field", ["cube"], SDF_CUBE
    + '<flexcomp name="f" type="grid" count="3 3 1" spacing="0.06 0.06 0.06" pos="0 0 0.105" radius="0.01" dim="2"><edge equality="true"/></flexcomp>')
doc("f02_flex_on_field_no_midphase", ["cube"], SDF_CUBE
    + '<flexcomp name="f" type="grid" count="3 3 1" spacing="0.06 0.06 0.06" pos="0 0 0.105" radius="0.01" dim="2"><edge equality="true"/></flexcomp>',
    flag='<flag midphase="disable"/>')


def hx(x):
    """A double's bits, as a 64-bit integer in hex."""
    return format(struct.unpack("<Q", struct.pack("<d", float(x)))[0], "016x")


def scene(xml):
    try:
        m = mujoco.MjModel.from_xml_string(xml)
    except Exception as e:  # MuJoCo's refusal, kept as its words
        return {"xml": xml, "err": str(e)}
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    contacts = [{"geom": [int(c.geom[0]), int(c.geom[1])], "flex": [int(c.flex[0]), int(c.flex[1])],
                 "vert": [int(c.vert[0]), int(c.vert[1])], "elem": [int(c.elem[0]), int(c.elem[1])],
                 "dist": hx(c.dist), "pos": [hx(x) for x in c.pos], "frame": [hx(x) for x in c.frame]}
                for c in (d.contact[i] for i in range(d.ncon))]
    out = {"xml": xml, "ncon": int(d.ncon), "contacts": contacts, "sensordata": [hx(x) for x in d.sensordata]}
    steps = []
    for _ in range(5):
        mujoco.mj_step(m, d)
        steps.append({"ncon": int(d.ncon), "qpos": [hx(x) for x in d.qpos]})
    out["steps"] = steps
    return out


def field(name, rng, lib):
    xml = f'<mujoco><asset>{ASSETS[name]}</asset><worldbody><geom type="sdf" mesh="{name}"/></worldbody></mujoco>'
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    adr = int(m.mesh_octadr[0])
    c, h = m.oct_aabb[adr][:3], m.oct_aabb[adr][3:]
    pts = []
    for i in range(7):
        for j in range(7):
            for k in range(7):
                pts.append([c[a] + 1.3 * h[a] * (t / 3 - 1) for a, t in enumerate((i, j, k))])
    pts += [[c[a] + rng.uniform(-3, 3) * h[a] for a in range(3)] for _ in range(80)]
    for a in range(3):
        for s in (-1, 1):
            for r in (10.0, 1e3):
                p = list(c)
                p[a] += s * r
                pts.append(p)
            p = list(c)
            p[a] += s * h[a]
            pts.append(p)
    dbl3 = ctypes.c_double * 3
    ident = (ctypes.c_double * 9)(1, 0, 0, 0, 1, 0, 0, 0, 1)
    sdf = MjSDF((ctypes.c_void_p * 2)(None, None), (ctypes.c_int * 2)(0, 0), 0, (ctypes.c_double * 3)(0, 0, 0),
                ident, (ctypes.c_int * 2)(8, 8))
    dist, grad = [], []
    for p in pts:
        dist.append(hx(lib.mjc_distance(m._address, d._address, ctypes.byref(sdf), dbl3(*p))))
        g = dbl3()
        lib.mjc_gradient(m._address, d._address, ctypes.byref(sdf), g, dbl3(*p))
        grad.append([hx(x) for x in g])
    return {"xml": xml, "octnum": int(m.mesh_octnum[0]), "points": [[hx(x) for x in p] for p in pts],
            "dist": dist, "grad": grad}


def main():
    oracle()
    if len(sys.argv) != 2:
        sys.exit("usage: gen_sdf_reference.py <out.json>")
    lib = library()
    rng = random.Random(9043)
    out = {"scenes": {name: scene(xml) for name, xml in SCENES.items()},
           "fields": {name: field(name, rng, lib) for name in ("cube", "ell", "grid")}}
    os.makedirs(os.path.dirname(os.path.abspath(sys.argv[1])), exist_ok=True)
    with open(sys.argv[1], "w") as f:
        json.dump(out, f, indent=1, sort_keys=True)
        f.write("\n")
    print(f"{len(out['scenes'])} scenes, {len(out['fields'])} fields -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
