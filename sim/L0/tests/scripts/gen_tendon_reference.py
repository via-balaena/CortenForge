#!/usr/bin/env python3
"""Generate the tendon golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_tendon_reference.py \\
        sim/L0/tests/assets/golden/tendon

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo replaces a tendon's `tendon_lengthspring` of [-1, -1] with its length
at `qpos_spring` (`setSpring`, `engine_setconst.c:1069-1084`), and wraps a
tendon over a sphere in the plane through the sphere's centre and the two
points; when they lie on one line (|p0 x p1| < mjMINVAL) it takes the plane
whose normal is p0 x a, a being 1 in every component but one, which is 0:
y's if |p0.y| exceeds both other components, z's if |p0.z| does, x's
otherwise; a normal shorter than mjMINVAL becomes the x axis (`mju_wrap`,
`engine_util_misc.c:302-340`; `mju_normalize3`).
The cases:

- spring_length: two hinges and a slide whose `ref` and `springref` differ,
  a spatial tendon with stiffness and no `springlength`, one with a
  `springlength`, and a fixed tendon with stiffness; the model's spring
  fields, `qfrc_spring` at qpos0, and qpos and qvel after steps 1, 10 and 100.
- spatial_path, tendon_limits: two in-tree examples (the census docs
  251c59049dab062a and 9a9380fcbf3eacdc) whose hinges have a `ref`;
  `tendon_lengthspring`.
- sphere_wrap: the in-tree sphere-wrap example (census doc
  11976cd3f27cf755), whose two sites lie on a line through the sphere's
  centre at qpos0, from qvel 0.1; each tendon's length, velocity, Jacobian
  and wrap points after mj_forward, and the trajectory.
- sphere_wrap_near: the same with one site 1e-12 off that line, so
  |p0 x p1| (2e-13) is above mjMINVAL.
- sphere_wrap_tie: a line through the centre along (1, 1, 0), whose
  direction has two largest components; one site turns about (1, -1, 0).
- sphere_wrap_ties: three tendons between fixed sites on lines through the
  centre along (0, 1, 1), (1, 0, 1) and (-1, 1, 0); along (0, 1, 1), p0 x a
  is 0.
- collinear_slides: two sites on either side of a sphere on one line, each on
  a slide across it, at qpos0.
- wraps: a sphere and a cylinder on a moving body, each wrapped by three
  tendons between sites on two other moving bodies: one with no sidesite,
  one with a sidesite outside the geom, one with a sidesite inside it (an
  inside wrap); each tendon's length, velocity, Jacobian and wrap points
  after mj_forward at configurations drawn from a seeded generator, the
  first four where each tendon wraps and the first four where it does not.
  The joints' axes are ones whose normalisation sim-mjcf rounds as MuJoCo
  does (ledger L146).
- wraps_sparse: wraps with `jacobian="sparse"`, so the tendons' velocity
  is MuJoCo's sparse product over each tendon's chain
  (`mju_mulMatVecSparse`, `engine_forward.c:232-235`), at the same
  configurations.
- fixed_velocity, fixed_velocity_sparse: a fixed tendon over six hinges
  of a chain with coefficients that are not round, dense and sparse; its
  velocity at ten velocities from a seeded generator.
- fixed_repeated, fixed_repeated_sparse: a fixed tendon that names joint a
  twice (coefficients 1, then 2) and joint b once, with stiffness, damping
  and a motor, dense and sparse. MuJoCo's dense Jacobian row assigns each
  joint's coefficient (a's is 2), its sparse row adds them (3)
  (`engine_core_smooth.c:979-982`, `:971-976`); the length adds every entry
  at both.
  The motor's acc0; the length, Jacobian row, velocity, passive and
  actuator forces after mj_forward, and the trajectory.
- tendon_motors: motors with gears 1.37 and -2.71 on a spatial and a fixed
  tendon; qfrc_actuator at ten states from a seeded generator. MuJoCo's
  moment is the tendon's row scaled by the gear (`mj_transmission`), then
  times the force.
- inside_fallback: a sphere and a cylinder at the origin, each with a
  sidesite inside it, and tendons between world sites a and 2a on six rays
  from the origin: the ends point one way (cos G within mjMINVAL of 1), so
  each inside wrap takes the ends' midpoint's direction at the radius
  (`wrap_inside`'s fallback, `engine_util_misc.c:195-210`).
- wrap_edges: six scenes at the edges of `mju_wrap`'s tests, from a
  review's sweep of them: an end on the surface; a segment grazing it; a
  radius of 1e-14, axis-aligned and turned; a cylinder wrap near pi; a
  radius of 1e-15, turned. Each
  a sphere and a cylinder in the world with sites x0, x1 and a sidesite,
  four tendons over them (with and without the sidesite); each tendon's
  state after mj_forward. It refuses a scene with a length not finite
  (MuJoCo's `acos` gives NaN past |cos| 1 on some grazing segments).
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

STEPS = (1, 10, 100)
DOCS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "census", "docs")

SPRING_LENGTH = """<mujoco model="spring_length">
  <option timestep="0.002"/>
  <compiler angle="radian"/>
  <worldbody>
    <site name="w0" pos="0.3 0 1.2"/>
    <body name="u" pos="0 0 1">
      <joint name="h1" type="hinge" axis="0 1 0" ref="0.8" springref="-0.3" damping="0.2"/>
      <geom type="box" size="0.02 0.02 0.25" pos="0 0 -0.25" mass="1"/>
      <site name="u1" pos="0.04 0 -0.1"/>
      <site name="u2" pos="0.04 0 -0.45"/>
      <body name="l" pos="0 0 -0.5">
        <joint name="h2" type="hinge" axis="0 1 0" ref="-0.5" damping="0.2"/>
        <geom type="box" size="0.02 0.02 0.2" pos="0 0 -0.2" mass="0.5"/>
        <site name="l1" pos="0.03 0 -0.2"/>
        <body name="s" pos="0 0 -0.4">
          <joint name="sl" type="slide" axis="1 0 0" ref="0.05" springref="0.12" damping="0.5"/>
          <geom type="sphere" size="0.03" mass="0.2"/>
          <site name="s1" pos="0 0.02 0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_auto" stiffness="30" damping="1">
      <site site="w0"/>
      <site site="u1"/>
      <site site="u2"/>
      <site site="l1"/>
      <site site="s1"/>
    </spatial>
    <spatial name="t_given" stiffness="20" springlength="0.4">
      <site site="u1"/>
      <site site="s1"/>
    </spatial>
    <fixed name="t_fixed" stiffness="5">
      <joint joint="h1" coef="1"/>
      <joint joint="sl" coef="2"/>
    </fixed>
  </tendon>
</mujoco>"""

SPHERE_WRAP_TIE = """<mujoco model="sphere_wrap_tie">
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <geom name="ball" type="sphere" size="0.1" contype="0" conaffinity="0"/>
    <site name="far" pos="-0.15 -0.15 0"/>
    <body name="arm">
      <joint name="j" type="hinge" axis="1 -1 0" damping="1"/>
      <geom type="sphere" size="0.02" pos="0.15 0.15 0" mass="0.5" contype="0" conaffinity="0"/>
      <site name="near" pos="0.15 0.15 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t">
      <site site="near"/>
      <geom geom="ball"/>
      <site site="far"/>
    </spatial>
  </tendon>
</mujoco>"""

SPHERE_WRAP_TIES = """<mujoco model="sphere_wrap_ties">
  <worldbody>
    <geom name="ball" type="sphere" size="0.1" contype="0" conaffinity="0"/>
    <site name="yz0" pos="0 0.15 0.15"/>
    <site name="yz1" pos="0 -0.15 -0.15"/>
    <site name="xz0" pos="0.15 0 0.15"/>
    <site name="xz1" pos="-0.15 0 -0.15"/>
    <site name="xy0" pos="-0.15 0.15 0"/>
    <site name="xy1" pos="0.15 -0.15 0"/>
  </worldbody>
  <tendon>
    <spatial name="t_yz">
      <site site="yz0"/>
      <geom geom="ball"/>
      <site site="yz1"/>
    </spatial>
    <spatial name="t_xz">
      <site site="xz0"/>
      <geom geom="ball"/>
      <site site="xz1"/>
    </spatial>
    <spatial name="t_xy">
      <site site="xy0"/>
      <geom geom="ball"/>
      <site site="xy1"/>
    </spatial>
  </tendon>
</mujoco>"""

COLLINEAR_SLIDES = """<mujoco model="collinear_slides">
  <worldbody>
    <body name="b1" pos="-0.3 0 0">
      <joint name="j1" type="slide" axis="0 1 0"/>
      <geom type="sphere" size="0.02"/>
      <site name="s1" pos="0 0 0"/>
    </body>
    <body name="wrap_body" pos="0 0 0">
      <geom name="wrap_sphere" type="sphere" size="0.1"/>
    </body>
    <body name="b2" pos="0.3 0 0">
      <joint name="j2" type="slide" axis="0 1 0"/>
      <geom type="sphere" size="0.02"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_collinear">
      <site site="s1"/>
      <geom geom="wrap_sphere"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>"""


WRAPS = """<mujoco model="wraps">
  <compiler angle="radian"/>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="w" pos="0.01 -0.02 0.03" quat="0.9 0.1 -0.3 0.2">
      <joint name="jw" type="hinge" axis="0.3 1 0.2"/>
      <joint name="jw2" type="slide" axis="0.3 -0.7 1.1"/>
      <geom name="ball" type="sphere" size="0.1" mass="1" contype="0" conaffinity="0"/>
      <geom name="cyl" type="cylinder" size="0.08 0.3" pos="0.6 0 0" quat="0.8 0.3 0.4 -0.2" mass="1" contype="0" conaffinity="0"/>
      <site name="ss_out" pos="0 0.3 0.05"/>
      <site name="ss_in" pos="0.01 0.02 -0.03"/>
      <site name="cs_out" pos="0.6 0.25 0.05"/>
      <site name="cs_in" pos="0.61 0.01 0.1"/>
    </body>
    <body name="a" pos="-0.25 0.05 0.02">
      <joint name="ja" type="hinge" axis="0 0 1"/>
      <joint name="ja2" type="slide" axis="0.3 1 0.2"/>
      <geom type="sphere" size="0.01" mass="0.1" contype="0" conaffinity="0"/>
      <site name="a0" pos="0.02 0.01 0"/>
      <site name="ca0" pos="0.62 -0.03 0.1"/>
    </body>
    <body name="b" pos="0.25 -0.03 -0.01">
      <joint name="jb" type="ball"/>
      <geom type="sphere" size="0.01" mass="0.1" contype="0" conaffinity="0"/>
      <site name="b0" pos="0.01 -0.02 0.03"/>
      <site name="cb0" pos="0.55 0.04 -0.12"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="s_none"><site site="a0"/><geom geom="ball"/><site site="b0"/></spatial>
    <spatial name="s_out"><site site="a0"/><geom geom="ball" sidesite="ss_out"/><site site="b0"/></spatial>
    <spatial name="s_in"><site site="a0"/><geom geom="ball" sidesite="ss_in"/><site site="b0"/></spatial>
    <spatial name="c_none"><site site="ca0"/><geom geom="cyl"/><site site="cb0"/></spatial>
    <spatial name="c_out"><site site="ca0"/><geom geom="cyl" sidesite="cs_out"/><site site="cb0"/></spatial>
    <spatial name="c_in"><site site="ca0"/><geom geom="cyl" sidesite="cs_in"/><site site="cb0"/></spatial>
  </tendon>
</mujoco>"""


WRAPS_SPARSE = WRAPS.replace('<option gravity="0 0 0"/>', '<option gravity="0 0 0" jacobian="sparse"/>', 1)
assert WRAPS_SPARSE != WRAPS

FIXED_VELOCITY = """<mujoco model="fixed_velocity">
  <worldbody>
    <body name="b1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="capsule" size="0.02" fromto="0 0 0 0.1 0 0" mass="0.1"/>
    <body name="b2" pos="0.1 0 0"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" size="0.02" fromto="0 0 0 0.1 0 0" mass="0.1"/>
    <body name="b3" pos="0.1 0 0"><joint name="j3" type="hinge" axis="0 1 0"/><geom type="capsule" size="0.02" fromto="0 0 0 0.1 0 0" mass="0.1"/>
    <body name="b4" pos="0.1 0 0"><joint name="j4" type="hinge" axis="0 1 0"/><geom type="capsule" size="0.02" fromto="0 0 0 0.1 0 0" mass="0.1"/>
    <body name="b5" pos="0.1 0 0"><joint name="j5" type="hinge" axis="0 1 0"/><geom type="capsule" size="0.02" fromto="0 0 0 0.1 0 0" mass="0.1"/>
    <body name="b6" pos="0.1 0 0"><joint name="j6" type="hinge" axis="0 1 0"/><geom type="capsule" size="0.02" fromto="0 0 0 0.1 0 0" mass="0.1"/>
    </body></body></body></body></body></body>
  </worldbody>
  <tendon>
    <fixed name="t">
      <joint joint="j1" coef="0.37"/><joint joint="j2" coef="-1.13"/><joint joint="j3" coef="0.71"/>
      <joint joint="j4" coef="2.9"/><joint joint="j5" coef="-0.23"/><joint joint="j6" coef="1.7"/>
    </fixed>
  </tendon>
</mujoco>"""

FIXED_VELOCITY_SPARSE = FIXED_VELOCITY.replace('<mujoco model="fixed_velocity">',
                                               '<mujoco model="fixed_velocity">\n  <option jacobian="sparse"/>', 1)
assert FIXED_VELOCITY_SPARSE != FIXED_VELOCITY

FIXED_REPEATED = """<mujoco model="fixed_repeated">
  <option timestep="0.002"/>
  <worldbody>
    <body name="b1"><joint name="a" type="hinge" axis="0 1 0" damping="0.1"/><geom type="capsule" size="0.03" fromto="0 0 0 0.3 0 0" mass="1"/>
    <body name="b2" pos="0.3 0 0"><joint name="b" type="hinge" axis="0 1 0" damping="0.1"/><geom type="capsule" size="0.03" fromto="0 0 0 0.3 0 0" mass="1"/>
    </body></body>
  </worldbody>
  <tendon>
    <fixed name="t" stiffness="0.5" damping="0.1" springlength="0.1">
      <joint joint="a" coef="1"/><joint joint="b" coef="0.5"/><joint joint="a" coef="2"/>
    </fixed>
  </tendon>
  <actuator><motor tendon="t"/></actuator>
</mujoco>"""

FIXED_REPEATED_SPARSE = FIXED_REPEATED.replace('<option timestep="0.002"/>',
                                               '<option timestep="0.002" jacobian="sparse"/>', 1)
assert FIXED_REPEATED_SPARSE != FIXED_REPEATED

TENDON_MOTORS = """<mujoco model="tendon_motors">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="a" pos="-0.25 0.05 0.02"><joint name="ja" type="hinge" axis="0 0 1"/><joint name="ja2" type="slide" axis="0.3 1 0.2"/>
      <geom type="sphere" size="0.01" mass="0.1"/><site name="a0" pos="0.02 0.01 0"/></body>
    <body name="b" pos="0.25 -0.03 -0.01"><joint name="jb" type="ball"/>
      <geom type="sphere" size="0.01" mass="0.1"/><site name="b0" pos="0.01 -0.02 0.03"/>
      <body pos="0.1 0 0"><joint name="jc" type="hinge" axis="0.3 1 0.2"/><geom type="sphere" size="0.01" mass="0.1"/>
        <site name="c0" pos="0.03 0.01 0"/></body></body>
  </worldbody>
  <tendon>
    <spatial name="s"><site site="a0"/><site site="b0"/><site site="c0"/></spatial>
    <fixed name="f"><joint joint="ja" coef="0.37"/><joint joint="ja2" coef="-1.13"/><joint joint="jc" coef="0.71"/></fixed>
  </tendon>
  <actuator><motor tendon="s" gear="1.37"/><motor tendon="f" gear="-2.71"/></actuator>
</mujoco>"""

RAYS = ((0.2, 0.1, 0.05), (-0.13, 0.07, 0.11), (0.03, -0.17, 0.09), (0.11, 0.13, -0.07),
        (-0.05, -0.09, -0.16), (0.17, -0.02, 0.12))

INSIDE_FALLBACK = """<mujoco model="inside_fallback">
  <worldbody>
    <geom name="ball" type="sphere" size="0.1" contype="0" conaffinity="0"/>
    <geom name="cyl" type="cylinder" size="0.08 0.3" contype="0" conaffinity="0"/>
    <site name="in_s" pos="0.01 -0.02 0.03"/>
    <site name="in_c" pos="0.02 0.01 -0.04"/>
""" + "".join(f'    <site name="a{i}" pos="{x} {y} {z}"/>\n    <site name="b{i}" pos="{2 * x} {2 * y} {2 * z}"/>\n'
              for i, (x, y, z) in enumerate(RAYS)) + """  </worldbody>
  <tendon>
""" + "".join(f'    <spatial name="s{i}"><site site="a{i}"/><geom geom="ball" sidesite="in_s"/><site site="b{i}"/></spatial>\n'
              f'    <spatial name="c{i}"><site site="a{i}"/><geom geom="cyl" sidesite="in_c"/><site site="b{i}"/></spatial>\n'
              for i in range(len(RAYS))) + """  </tendon>
</mujoco>"""


WRAP_EDGES = {
    "on_surface_0": dict(x0=(0.1, 0.0, 0.0), x1=(-0.5, 0.01, 0.02),
                xpos=(0.0, 0.0, 0.0), quat=None,
                r=0.1, side=(0.0, 0.3, 0.0)),
    "graze_0.0": dict(x0=(0.5, 0.1, 0.0), x1=(-0.5, 0.1, 0.0),
                xpos=(0.0, 0.0, 0.0), quat=None,
                r=0.1, side=(0.0, 0.3, 0.0)),
    "radius_1e-14": dict(x0=(0.5, 0.0, 0.0), x1=(-0.5, 1e-17, 0.0),
                xpos=(0.0, 0.0, 0.0), quat=None,
                r=1e-14, side=(0.0, 0.3, 0.0)),
    "near_pi_z_1e-09": dict(x0=(0.10000000010000001, 0.0, 0.1), x1=(-0.10000000010000001, 1e-07, -0.1),
                xpos=(0.0, 0.0, 0.0), quat=None,
                r=0.1, side=(0.0, 0.3, 0.0)),
    "rot_radius_1e-15": dict(x0=(-2.124136895180067, 0.5135682598534345, 0.01719912620783659), x1=(-1.5076145515219104, 0.2287729267604888, -0.7168252725819669),
                xpos=(-1.8158757233509886, 0.3711705933069616, -0.34981307318706517), quat=(-0.4192038564751245, 0.12651859502581073, 0.8963697322326629, -0.06915544061187426),
                r=1e-15, side=(-1.765225396303986, 0.6586969353948451, -0.4188286292276643)),
    "rot_radius_1e-14": dict(x0=(1.9970641950622972, 0.1704844136589891, 2.1744182408273915), x1=(2.1723690524171575, -0.6113447126940592, 2.772758799517692),
                xpos=(2.084716623739727, -0.22043014951753503, 2.473588520172542), quat=(0.6397294299222246, -0.055622188144259406, 0.41141144188066814, 0.6468330960665971),
                r=1e-14, side=(1.8227075602838276, -0.1733222787061681, 2.611907351770347)),
}


def wrap_edge_xml(name, x0, x1, xpos, quat, r, side):
    """A sphere and a cylinder of radius r at xpos, turned by quat, with
    sites x0, x1 and side in the world: four tendons from x0 to x1 over
    each, with no sidesite and with side."""
    def v(p):
        return " ".join(repr(float(c)) for c in p)
    turn = "" if quat is None else f' quat="{v(quat)}"'
    return f"""<mujoco model="wrap_edge_{name}">
  <worldbody>
    <geom name="gs" type="sphere" size="{r!r}" pos="{v(xpos)}"{turn} contype="0" conaffinity="0"/>
    <geom name="gc" type="cylinder" size="{r!r} 0.3" pos="{v(xpos)}"{turn} contype="0" conaffinity="0"/>
    <site name="x0" pos="{v(x0)}"/>
    <site name="x1" pos="{v(x1)}"/>
    <site name="side" pos="{v(side)}"/>
  </worldbody>
  <tendon>
    <spatial name="s_none"><site site="x0"/><geom geom="gs"/><site site="x1"/></spatial>
    <spatial name="s_side"><site site="x0"/><geom geom="gs" sidesite="side"/><site site="x1"/></spatial>
    <spatial name="c_none"><site site="x0"/><geom geom="gc"/><site site="x1"/></spatial>
    <spatial name="c_side"><site site="x0"/><geom geom="gc" sidesite="side"/><site site="x1"/></spatial>
  </tendon>
</mujoco>"""


def wrap_edges():
    """WRAP_EDGES as scenes: each tendon's state after mj_forward. It refuses
    a scene with a tendon whose length is not finite."""
    out = {}
    for name, spec in WRAP_EDGES.items():
        case = sphere_wrap(wrap_edge_xml(name, **spec), name, [], False)
        if not all(np.isfinite(t["ten_length"]) for t in case["forward"]):
            sys.exit(f"wrap_edges {name}: a length is not finite")
        out[name] = case
    return out


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def doc(name):
    with open(os.path.join(DOCS, name + ".xml"), encoding="utf-8") as f:
        return f.read()


def stable(d, what):
    for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
              mujoco.mjtWarning.mjWARN_BADQPOS):
        if d.warning[w].number:
            sys.exit(f"{what}: MuJoCo warned {w.name}")


def trajectory(m, d, what):
    out = {}
    for step in range(1, max(STEPS) + 1):
        mujoco.mj_step(m, d)
        if step in STEPS:
            out[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    stable(d, what)
    return out


def spring_length():
    m = mujoco.MjModel.from_xml_string(SPRING_LENGTH)
    assert np.any(m.qpos0 != m.qpos_spring), "ref and springref differ"
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    out = {"xml": SPRING_LENGTH, "qpos0": floats(m.qpos0), "qpos_spring": floats(m.qpos_spring),
           "tendon_length0": floats(m.tendon_length0),
           "tendon_lengthspring": floats(m.tendon_lengthspring),
           "qfrc_spring": floats(d.qfrc_spring)}
    d.qvel[:] = [0.4, -0.3, 0.2]
    out["qvel"] = floats(d.qvel)
    out["traj"] = trajectory(m, d, "spring_length")
    return out


def lengthspring_of(name):
    xml = doc(name)
    m = mujoco.MjModel.from_xml_string(xml)
    return {"xml": xml, "tendon_lengthspring": floats(m.tendon_lengthspring)}


def ten_j(m, d, t):
    """Tendon t's Jacobian row, dense: at the sparse size MuJoCo keeps
    `ten_J` compressed, the row's values at `ten_J_rowadr[t]` for the dofs
    `ten_J_colind` lists."""
    if not mujoco.mj_isSparse(m):
        return floats(d.ten_J[t])
    row = np.zeros(m.nv)
    adr, nnz = d.ten_J_rowadr[t], d.ten_J_rownnz[t]
    row[d.ten_J_colind.ravel()[adr:adr + nnz]] = d.ten_J.ravel()[adr:adr + nnz]
    return floats(row)


def wrap_state(m, d):
    """Each tendon's length, velocity, Jacobian row and wrap points."""
    out = []
    for t in range(m.ntendon):
        adr, num = d.ten_wrapadr[t], d.ten_wrapnum[t]
        out.append({"ten_length": float(d.ten_length[t]), "ten_velocity": float(d.ten_velocity[t]),
                    "ten_J": ten_j(m, d, t),
                    "wrap_obj": [int(x) for x in d.wrap_obj.ravel()[adr:adr + num]],
                    "wrap_xpos": floats(d.wrap_xpos.reshape(-1, 3)[adr:adr + num])})
    return out


def sphere_wrap(xml, what, qvel, steps):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    out = {"xml": xml, "qvel": floats(d.qvel), "forward": wrap_state(m, d)}
    if steps:
        out["traj"] = trajectory(m, d, what)
    return out


def wraps(xml):
    """`xml` (WRAPS or its sparse copy) at configurations from a seeded
    generator: for each tendon the first four where it wraps and the first
    four where it does not."""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(135)
    drawn = []
    for _ in range(200):
        qpos = m.qpos0.copy()
        qpos[0] = rng.uniform(-3, 3)
        qpos[1] = rng.uniform(-0.15, 0.15)
        qpos[2] = rng.uniform(-3, 3)
        qpos[3] = rng.uniform(-0.2, 0.2)
        q = rng.normal(size=4)
        qpos[4:8] = q / np.linalg.norm(q)
        d.qpos[:] = qpos
        d.qvel[:] = rng.normal(size=m.nv)
        mujoco.mj_forward(m, d)
        drawn.append({"qpos": floats(d.qpos), "qvel": floats(d.qvel), "forward": wrap_state(m, d)})
    keep = set()
    for t in range(m.ntendon):
        for wrapped in (True, False):
            hits = [k for k, c in enumerate(drawn) if (len(c["forward"][t]["wrap_obj"]) == 4) == wrapped]
            if len(hits) < 4:
                sys.exit(f"wraps: tendon {t} wrapped={wrapped} at {len(hits)} configurations")
            keep.update(hits[:4])
    return {"xml": xml, "configs": [drawn[k] for k in sorted(keep)]}


def fixed_velocity(xml):
    """`xml`'s fixed tendon's velocity at ten velocities from a seeded
    generator, at qpos0."""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(149)
    configs = []
    for _ in range(10):
        d.qvel[:] = rng.normal(size=m.nv)
        mujoco.mj_forward(m, d)
        configs.append({"qvel": floats(d.qvel), "ten_velocity": floats(d.ten_velocity),
                        "ten_J": ten_j(m, d, 0)})
    return {"xml": xml, "configs": configs}


def fixed_repeated(xml):
    """`xml`'s motor's acc0 (from its moment, the tendon's row at qpos0); the
    tendon at qpos (0.3, -0.2), qvel (0.7, 0.4) and ctrl 0.8: its length,
    Jacobian row and velocity, the passive and actuator forces after
    mj_forward; then the trajectory."""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[:] = [0.3, -0.2]
    d.qvel[:] = [0.7, 0.4]
    d.ctrl[:] = [0.8]
    out = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel), "ctrl": floats(d.ctrl),
           "actuator_acc0": floats(m.actuator_acc0)}
    mujoco.mj_forward(m, d)
    out["forward"] = {"ten_length": float(d.ten_length[0]), "ten_J": ten_j(m, d, 0),
                      "ten_velocity": float(d.ten_velocity[0]), "qfrc_passive": floats(d.qfrc_passive),
                      "qfrc_actuator": floats(d.qfrc_actuator)}
    out["traj"] = trajectory(m, d, "fixed_repeated")
    return out


def tendon_motors():
    """TENDON_MOTORS at ten states (qpos, qvel, ctrl) from a seeded
    generator: qfrc_actuator after mj_forward."""
    m = mujoco.MjModel.from_xml_string(TENDON_MOTORS)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(151)
    configs = []
    for _ in range(10):
        qpos = m.qpos0.copy()
        qpos[0] = rng.uniform(-2, 2)
        qpos[1] = rng.uniform(-0.1, 0.1)
        q = rng.normal(size=4)
        qpos[2:6] = q / np.linalg.norm(q)
        qpos[6] = rng.uniform(-1, 1)
        d.qpos[:] = qpos
        d.qvel[:] = rng.normal(size=m.nv)
        d.ctrl[:] = rng.uniform(-1, 1, size=m.nu)
        mujoco.mj_forward(m, d)
        configs.append({"qpos": floats(d.qpos), "qvel": floats(d.qvel), "ctrl": floats(d.ctrl),
                        "qfrc_actuator": floats(d.qfrc_actuator)})
    return {"xml": TENDON_MOTORS, "configs": configs}


def near(xml):
    """The sphere-wrap example with its far site 1e-12 off the line. The site
    sits 0.1 from its body's origin: MuJoCo copies the body's frame for a site
    within 1e-6 of it (`site_sameframe`), which would put it back on the line."""
    out = xml
    for old, new in (('<body name="anchor" pos="-0.2 0 0.8">', '<body name="anchor" pos="-0.1 0 0.8">'),
                     ('<site name="s_insert" pos="0 0 0"', '<site name="s_insert" pos="-0.1 0 1e-12"'),
                     ('model="sphere-wrap"', 'model="sphere-wrap-near"')):
        assert out.count(old) == 1, old
        out = out.replace(old, new)
    return out


def check_planes(cases):
    """Each wrap case is where it is meant to be: on the line or off it."""
    for name, on_line in (("sphere_wrap", True), ("sphere_wrap_near", False), ("sphere_wrap_tie", True),
                          ("sphere_wrap_ties", True), ("collinear_slides", True)):
        m = mujoco.MjModel.from_xml_string(cases[name]["xml"])
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        g = next(i for i in range(m.ngeom) if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_SPHERE
                 and m.geom_size[i][0] == 0.1)
        for t in range(m.ntendon):
            sites = [m.wrap_objid[i] for i in range(m.tendon_adr[t], m.tendon_adr[t] + m.tendon_num[t])
                     if m.wrap_type[i] == mujoco.mjtWrap.mjWRAP_SITE]
            p = [d.geom_xmat[g].reshape(3, 3).T @ (d.site_xpos[s] - d.geom_xpos[g]) for s in sites]
            cross = np.linalg.norm(np.cross(p[0], p[1]))
            if (cross < 1e-15) != on_line:
                sys.exit(f"{name} tendon {t}: |p0 x p1| = {cross}")
    for t, tendon in enumerate(cases["inside_fallback"]["forward"]):
        if tendon["wrap_obj"][1:3] != [t % 2, t % 2]:
            sys.exit(f"inside_fallback tendon {t}: no inside wrap, {tendon['wrap_obj']}")
    tie = cases["sphere_wrap_tie"]["xml"]
    m = mujoco.MjModel.from_xml_string(tie)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    a = np.abs(d.site_xpos[m.site("near").id])
    if not (a[0] == a[1] and a[0] > a[2]):
        sys.exit(f"sphere_wrap_tie: no tie, {a}")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    wrap = doc("11976cd3f27cf755")
    cases = {
        "spring_length": spring_length(),
        "spatial_path": lengthspring_of("251c59049dab062a"),
        "tendon_limits": lengthspring_of("9a9380fcbf3eacdc"),
        "sphere_wrap": sphere_wrap(wrap, "sphere_wrap", [0.1], True),
        "sphere_wrap_near": sphere_wrap(near(wrap), "sphere_wrap_near", [0.1], True),
        "sphere_wrap_tie": sphere_wrap(SPHERE_WRAP_TIE, "sphere_wrap_tie", [0.3], True),
        "sphere_wrap_ties": sphere_wrap(SPHERE_WRAP_TIES, "sphere_wrap_ties", [], False),
        "collinear_slides": sphere_wrap(COLLINEAR_SLIDES, "collinear_slides", [0.0, 0.0], False),
        "wraps": wraps(WRAPS),
        "wraps_sparse": wraps(WRAPS_SPARSE),
        "fixed_velocity": fixed_velocity(FIXED_VELOCITY),
        "fixed_velocity_sparse": fixed_velocity(FIXED_VELOCITY_SPARSE),
        "inside_fallback": sphere_wrap(INSIDE_FALLBACK, "inside_fallback", [], False),
        "fixed_repeated": fixed_repeated(FIXED_REPEATED),
        "fixed_repeated_sparse": fixed_repeated(FIXED_REPEATED_SPARSE),
        "tendon_motors": tendon_motors(),
        "wrap_edges": wrap_edges(),
    }
    check_planes(cases)
    out = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "tendon.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
