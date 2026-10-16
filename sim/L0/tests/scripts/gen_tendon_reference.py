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
- inside_fallback: a sphere and a cylinder at the origin, each with a
  sidesite inside it, and tendons between world sites a and 2a on six rays
  from the origin: the ends point one way (cos G within mjMINVAL of 1), so
  each inside wrap takes the ends' midpoint's direction at the radius
  (`wrap_inside`'s fallback, `engine_util_misc.c:195-210`).
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


def wrap_state(m, d):
    """Each tendon's length, velocity, Jacobian row and wrap points."""
    out = []
    for t in range(m.ntendon):
        adr, num = d.ten_wrapadr[t], d.ten_wrapnum[t]
        out.append({"ten_length": float(d.ten_length[t]), "ten_velocity": float(d.ten_velocity[t]),
                    "ten_J": floats(d.ten_J[t]),
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


def wraps():
    """WRAPS at configurations from a seeded generator: for each tendon the
    first four where it wraps and the first four where it does not."""
    m = mujoco.MjModel.from_xml_string(WRAPS)
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
    return {"xml": WRAPS, "configs": [drawn[k] for k in sorted(keep)]}


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
        "wraps": wraps(),
        "inside_fallback": sphere_wrap(INSIDE_FALLBACK, "inside_fallback", [], False),
    }
    check_planes(cases)
    out = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "tendon.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
