#!/usr/bin/env python3
"""Generate the implicit-integrator golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_implicit_reference.py \\
        sim/L0/tests/assets/golden/implicit

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's implicit integrators take M - h*qDeriv on a pattern
(`mj_implicitSkip`, `engine_forward.c:1128-1353`): implicitfast on the mass
matrix's lower triangle, a simple dof's row its diagonal only, factored with
no pivot check (`mj_factorI`); implicit on every pair of dofs on one branch
(`mju_factorLUSparse`). Which bodies are simple and which frames are copied
comes from the compiler (`body_simple`, `dof_simplenum`, `*_sameframe`,
`user_model.cc:2739-2860`, `:2899-2948`, `:3942-3951`). The cases:

- fluid: a free ellipsoid in a fluid under implicitfast (the census doc
  d58bb68a2ae92f44), whose dofs are simple: qpos and qvel after steps 1, 10
  and 100 from a moving start.
- cross_branch: two hinged arms off the world joined by a damped fixed
  tendon, under implicit and implicitfast: the trajectory.
- negative_damping: a double pendulum with a fixed tendon whose stiffness is
  -2 and damping -50 (set at runtime; the loader refuses them), so M - h*D is
  not positive definite under implicitfast: the state after 1 and 20 steps.
Each trajectory also records the factor its first step left (`qH` and
`qHDiagInv`, or `qLU`).
- feedback: a hinge with an affine velocity gain of 1000 at ctrl 10, under
  implicitfast and implicit: qvel after one step.
- factor: a hinge-hinge-ball chain with a hinge off its root, a fixed
  tendon with damping across both branches and a velocity actuator; after
  one step under each integrator, the inputs MuJoCo factored, rebuilt from
  `d.M` and `d.qDeriv` as `mj_implicitSkip` gathers them (`M + qDeriv*(-h)`
  on M's pattern for implicitfast, on D's for implicit), and its factors
  (`qH` and `qHDiagInv`, `qLU`); and `mj_solveM` of a vector after
  `mj_factorM` of the implicitfast input, which runs `mj_factorI` and
  `mj_solveLD` on M's pattern. sim-core's unit tests take these as literals.
- sleeping: two double pendulums, one moving and one asleep from the start
  (`sleep="init"`), under each integrator: MuJoCo fills `qH` or `qLU` over
  every row but factors the awake ones only, so a sleeping row keeps its
  unfactored entries and `qHDiagInv` there stays 0; the factor after the
  first step and the trajectory.
- flags: bodies of each kind (simple or not, and why), geoms and sites at,
  near (1e-7) and off their body's frame and inertial frame: the flags, the
  two patterns, the model's frame fields, every pose `mj_kinematics` writes
  at a moved state, and `body_invweight0` and `dof_invweight0`
  (`engine_setconst.c:620-700`, which reads `body_simple`).
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

STEPS = (1, 10, 100)

FLUID = """<mujoco model="fluid">
  <option density="1.2" viscosity="0.001" integrator="implicitfast"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1" fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>"""

CROSS_BRANCH = """<mujoco model="cross_branch">
  <option timestep="0.002" integrator="{integrator}"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="ja" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="box" size="0.02 0.02 0.2" pos="0 0 -0.2" mass="1"/>
    </body>
    <body name="b" pos="0.5 0 1">
      <joint name="jb" type="hinge" axis="0 1 0"/>
      <geom type="box" size="0.02 0.02 0.25" pos="0 0 -0.25" mass="0.6"/>
      <body pos="0 0 -0.5">
        <joint name="jc" type="hinge" axis="1 0 0"/>
        <geom type="box" size="0.02 0.02 0.1" pos="0 0 -0.1" mass="0.3"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t" damping="4">
      <joint joint="ja" coef="1"/>
      <joint joint="jb" coef="-0.7"/>
      <joint joint="jc" coef="0.4"/>
    </fixed>
  </tendon>
</mujoco>"""

NEGATIVE_DAMPING = """<mujoco model="negative_damping">
  <option timestep="0.01" integrator="implicitfast"/>
  <worldbody>
    <body>
      <joint name="j0" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.05" mass="1"/>
      <body pos="0 0 -1">
        <joint name="j1" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.05" mass="1"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t" stiffness="2" damping="0.5" springlength="0.1">
      <joint joint="j0" coef="1"/>
      <joint joint="j1" coef="-0.5"/>
    </fixed>
  </tendon>
</mujoco>"""

FEEDBACK = """<mujoco model="feedback">
  <option timestep="0.01" integrator="{integrator}" gravity="0 0 0"/>
  <worldbody>
    <body name="b1" pos="0 0 0">
      <joint name="j0" type="hinge" axis="0 1 0" damping="0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <general name="a0" joint="j0" gaintype="affine" gainprm="0 0 1000" biasprm="0 0 0"/>
  </actuator>
</mujoco>"""

FACTOR = """<mujoco model="factor">
  <option timestep="0.005" integrator="{integrator}"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="j0" type="hinge" axis="0 1 0" damping="0.3"/>
      <geom type="box" size="0.03 0.03 0.15" pos="0 0 -0.15" mass="1"/>
      <body name="b" pos="0 0 -0.3">
        <joint name="j1" type="hinge" axis="1 0 0" damping="0.2"/>
        <geom type="box" size="0.03 0.03 0.12" pos="0.02 0 -0.12" mass="0.7"/>
        <body name="c" pos="0 0 -0.25">
          <joint name="j2" type="ball" damping="0.1"/>
          <geom type="box" size="0.05 0.03 0.08" pos="0.03 0.01 -0.08" mass="0.4"/>
        </body>
      </body>
      <body name="d" pos="0.2 0 0">
        <joint name="j3" type="hinge" axis="0 0 1" damping="0.25"/>
        <geom type="box" size="0.1 0.02 0.02" pos="0.1 0 0" mass="0.5"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t" damping="3">
      <joint joint="j1" coef="1"/>
      <joint joint="j3" coef="-0.6"/>
    </fixed>
  </tendon>
  <actuator>
    <velocity joint="j1" kv="2"/>
  </actuator>
</mujoco>"""

SLEEPING = """<mujoco model="sleeping">
  <option timestep="0.002" integrator="{integrator}">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <body name="moving" pos="0 0 1">
      <joint type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="box" size="0.02 0.02 0.2" pos="0 0 -0.2" mass="1"/>
      <body pos="0 0 -0.4">
        <joint type="hinge" axis="1 0 0" damping="0.3"/>
        <geom type="box" size="0.02 0.02 0.1" pos="0 0 -0.1" mass="0.5"/>
      </body>
    </body>
    <body name="resting" pos="1 0 1" sleep="init">
      <joint type="hinge" axis="0 1 0" damping="0.4"/>
      <geom type="box" size="0.02 0.02 0.2" pos="0 0 -0.2" mass="1"/>
      <body pos="0 0 -0.4">
        <joint type="hinge" axis="0 1 0" damping="0.3"/>
        <geom type="box" size="0.03 0.02 0.1" pos="0.01 0 -0.1" mass="0.5"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

FLAGS = """<mujoco model="flags">
  <option timestep="0.002"/>
  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.1" contype="0" conaffinity="0"/>
    <body name="free" pos="0 0 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.2 0.3"/>
      <geom name="free_at_body" type="sphere" size="0.1" contype="0" conaffinity="0"/>
      <site name="free_at_body"/>
      <site name="free_near_body" pos="1e-7 0 0"/>
      <site name="free_near_rot" pos="0.3 0 0" quat="1 4e-7 0 0"/>
      <site name="free_off" pos="0.2 0.1 0" quat="0.9 0.2 0.1 0.3"/>
    </body>
    <body name="sliders" pos="1 0 0">
      <joint type="slide" axis="1 0 0" armature="0.3"/>
      <joint type="slide" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      <geom name="sliders_near_body" type="sphere" size="0.05" pos="0 -2e-7 0" contype="0" conaffinity="0"/>
    </body>
    <body name="slide_then_hinge" pos="2 0 0">
      <joint type="slide" axis="0 0 1"/>
      <joint type="hinge" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.2 0.3"/>
    </body>
    <body name="tilted_hinge" pos="3 0 0">
      <joint type="hinge" axis="0 1 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.2 0.3"/>
    </body>
    <body name="two_hinges" pos="4 0 0">
      <joint type="hinge" axis="1 0 0"/>
      <joint type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.2 0.3"/>
    </body>
    <body name="hinge_off_origin" pos="5 0 0">
      <joint type="hinge" axis="0 0 1" pos="0.1 0 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.2 0.3"/>
    </body>
    <body name="offset_inertia" pos="6 0 0">
      <joint type="hinge" axis="0 0 1"/>
      <inertial pos="0.1 0 0" quat="0.8 0 0.6 0" mass="1" diaginertia="0.1 0.2 0.3"/>
      <geom name="at_inertia" type="sphere" size="0.05" pos="0.1 0 0" quat="0.8 0 0.6 0" contype="0" conaffinity="0" mass="0"/>
      <geom name="near_inertia" type="sphere" size="0.05" pos="0.1000001 0 0" quat="0.8 0 0.6 0" contype="0" conaffinity="0" mass="0"/>
      <site name="off_inertia" pos="0.3 0 0" quat="0.6 0.8 0 0"/>
    </body>
    <body name="turned_inertia" pos="7 0 0">
      <joint type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" quat="0.9 0.1 0 0" mass="1" diaginertia="0.1 0.2 0.3"/>
      <site name="body_rot" pos="0.2 0 0"/>
      <site name="inertia_rot" pos="0.2 0 0" quat="0.9 0.1 0 0"/>
      <site name="near_inertia_rot" pos="0.2 0 0" quat="0.9 0.1000004 0 0"/>
      <site name="negated_inertia_rot" pos="0.2 0 0" quat="-0.9 -0.1 0 0"/>
    </body>
    <body name="parent" pos="8 0 0">
      <joint type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      <body name="child" pos="0 0 -0.5">
        <joint type="slide" axis="1 0 0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      </body>
    </body>
    <body name="mount" pos="9 0 0">
      <geom type="box" size="0.1 0.1 0.1" contype="0" conaffinity="0"/>
      <body name="on_mount" pos="0 0 -0.5">
        <joint type="slide" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def stable(d, what):
    for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
              mujoco.mjtWarning.mjWARN_BADQPOS):
        if d.warning[w].number:
            sys.exit(f"{what}: MuJoCo warned {w.name}")


def trajectory(m, d, what, steps=STEPS):
    out = {}
    for step in range(1, max(steps) + 1):
        mujoco.mj_step(m, d)
        if step == 1:
            # the factor the first step left: qH and qHDiagInv, or qLU
            out["factor"] = ({"qH": floats(d.qH), "qHDiagInv": floats(d.qHDiagInv)}
                             if m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
                             else {"qLU": floats(d.qLU)})
        if step in steps:
            out[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    stable(d, what)
    return out


def fluid():
    m = mujoco.MjModel.from_xml_string(FLUID)
    assert list(m.M_rownnz) == [1] * 6 and all(m.dof_simplenum), "the free dofs are simple"
    d = mujoco.MjData(m)
    d.qvel[:] = [0.3, -0.2, 0.4, 2.0, -1.5, 3.0]
    qvel = floats(d.qvel)
    return {"xml": FLUID, "qvel": qvel, "traj": trajectory(m, d, "fluid")}


def cross_branch():
    out = {}
    for integrator in ("implicit", "implicitfast"):
        xml = CROSS_BRANCH.format(integrator=integrator)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.qpos[:] = [0.4, -0.3, 0.2]
        d.qvel[:] = [1.0, 0.5, -0.8]
        out[integrator] = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
                           "traj": trajectory(m, d, integrator)}
    return out


def negative_damping():
    m = mujoco.MjModel.from_xml_string(NEGATIVE_DAMPING)
    m.tendon_stiffness[0] = -2.0
    m.tendon_damping[0] = -50.0
    d = mujoco.MjData(m)
    d.qpos[:] = [0.3, -0.2]
    d.qvel[:] = [0.7, -0.4]
    # M - h*D is not positive definite: a diagonal of its factor is negative.
    probe = mujoco.MjData(m)
    probe.qpos[:], probe.qvel[:] = d.qpos, d.qvel
    mujoco.mj_step(m, probe)
    if not np.any(probe.qHDiagInv < 0):
        sys.exit(f"negative_damping: M - h*D is positive definite, {probe.qHDiagInv}")
    out = {"xml": NEGATIVE_DAMPING, "tendon_stiffness": -2.0, "tendon_damping": -50.0,
           "qpos": [0.3, -0.2], "qvel": [0.7, -0.4],
           "qHDiagInv": floats(probe.qHDiagInv)}
    out["traj"] = trajectory(m, d, "negative_damping", steps=(1, 20))
    return out


def feedback():
    out = {}
    for integrator in ("implicitfast", "implicit"):
        xml = FEEDBACK.format(integrator=integrator)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.ctrl[0] = 10.0
        d.qvel[0] = 1.0
        mujoco.mj_step(m, d)
        out[integrator] = {"xml": xml, "ctrl": 10.0, "qvel0": 1.0, "qvel": floats(d.qvel)}
    return out


def lu_divisions_tell(lu, rownnz, rowadr, colind):
    """Whether mju_factorLUSparse's divisions x/y, replayed, round otherwise
    than x*(1/y) at one of them at least: a test of the factor then tells the
    two apart."""
    lu, rem = list(lu), list(rownnz)
    for i in range(len(rownnz) - 1, -1, -1):
        rem[i] -= 1
        ii = rowadr[i] + rem[i]
        for j in range(i - 1, -1, -1):
            ji = rowadr[j] + rem[j] - 1
            if colind[ji] != i:
                continue
            rem[j] -= 1
            if lu[ji] / lu[ii] != lu[ji] * (1.0 / lu[ii]):
                return True
            lu[ji] /= lu[ii]
            icnt, jcnt = rowadr[i], rowadr[j]
            while jcnt < rowadr[j] + rem[j]:
                if colind[icnt] == colind[jcnt]:
                    lu[jcnt] -= lu[icnt] * lu[ji]
                    icnt, jcnt = icnt + 1, jcnt + 1
                elif colind[icnt] > colind[jcnt]:
                    jcnt += 1
                else:
                    icnt += 1
    return False


def factor():
    out = {}
    y = np.array([0.7, -1.3, 0.4, 2.1, -0.9, 1.6])
    for integrator in ("implicitfast", "implicit"):
        xml = FACTOR.format(integrator=integrator)
        m = mujoco.MjModel.from_xml_string(xml)
        assert m.nv == 6 and max(m.M_rownnz) == 5, (m.nv, list(m.M_rownnz))
        d = mujoco.MjData(m)
        d.qpos[:] = [0.6, -0.5, 0.7, -0.4, 0.3, 0.5, -0.2]
        d.qpos[2:6] /= np.linalg.norm(d.qpos[2:6])
        d.qvel[:] = [1.3, -0.9, 0.6, -1.7, 1.1, 0.4]
        d.ctrl[0] = 0.3
        mujoco.mj_step(m, d)
        stable(d, integrator)
        neg_h = -m.opt.timestep
        case = {"xml": xml}
        if integrator == "implicitfast":
            qh_in = np.array([d.M[k] + d.qDeriv[m.mapD2M[k]] * neg_h for k in range(m.nC)])
            case.update({"rownnz": ints(m.M_rownnz), "rowadr": ints(m.M_rowadr),
                         "colind": ints(m.M_colind), "qH_in": floats(qh_in),
                         "qH": floats(d.qH), "qHDiagInv": floats(d.qHDiagInv)})
            # mj_factorM copies d.M into qLD and runs mj_factorI on M's pattern
            d.M[:] = qh_in
            mujoco.mj_factorM(m, d)
            assert np.array_equal(d.qLD, d.qH) and np.array_equal(d.qLDiagInv, d.qHDiagInv)
            x = np.zeros(m.nv)
            mujoco.mj_solveM(m, d, x.reshape(1, -1), y.reshape(1, -1))
            case.update({"y": floats(y), "x": floats(x)})
        else:
            qlu_in = np.array([(d.M[m.mapM2D[k]] if m.mapM2D[k] >= 0 else 0.0) + d.qDeriv[k] * neg_h
                               for k in range(m.nD)])
            if not lu_divisions_tell(qlu_in, m.D_rownnz, m.D_rowadr, m.D_colind):
                sys.exit("factor: every LU division rounds as a product with the reciprocal")
            case.update({"rownnz": ints(m.D_rownnz), "rowadr": ints(m.D_rowadr),
                         "diag": ints(m.D_diag), "colind": ints(m.D_colind),
                         "qLU_in": floats(qlu_in), "qLU": floats(d.qLU)})
        out[integrator] = case
    return out


def sleeping():
    out = {}
    for integrator in ("implicitfast", "implicit"):
        xml = SLEEPING.format(integrator=integrator)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        if d.nv_awake != 2:
            sys.exit(f"sleeping: {d.nv_awake} dofs awake at the start")
        d.qpos[0:2] = [0.5, -0.4]
        d.qvel[0:2] = [1.2, -0.8]
        case = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
                "traj": trajectory(m, d, integrator, steps=(1, 10))}
        if d.nv_awake != 2:
            sys.exit(f"sleeping: {d.nv_awake} dofs awake after 10 steps")
        # the sleeping rows couple: factoring them would change an entry
        if m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICIT:
            sleeping_offdiag = d.qLU[m.D_rowadr[2] + 1]
        else:
            sleeping_offdiag = d.qH[m.M_rowadr[3]]
        if sleeping_offdiag == 0:
            sys.exit("sleeping: the sleeping tree's rows do not couple")
        out[integrator] = case
    return out


def flags():
    m = mujoco.MjModel.from_xml_string(FLAGS)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(29)
    for j in range(m.njnt):
        a, t = m.jnt_qposadr[j], m.jnt_type[j]
        if t == mujoco.mjtJoint.mjJNT_FREE:
            d.qpos[a:a + 3] = [0.2, -0.1, 1.3]
            q = rng.normal(size=4)
            d.qpos[a + 3:a + 7] = q / np.linalg.norm(q)
        else:
            d.qpos[a] = rng.uniform(-0.8, 0.8)
    qpos = floats(d.qpos)
    mujoco.mj_kinematics(m, d)
    model = {k: floats(getattr(m, k)) for k in (
        "body_pos", "body_quat", "body_ipos", "body_iquat", "jnt_pos", "jnt_axis",
        "geom_pos", "geom_quat", "site_pos", "site_quat")}
    derived = {k: ints(getattr(m, k)) for k in (
        "body_sameframe", "geom_sameframe", "site_sameframe", "body_simple", "dof_simplenum",
        "M_rownnz", "M_rowadr", "M_colind", "D_rownnz", "D_rowadr", "D_diag", "D_colind")}
    poses = {k: floats(getattr(d, k)) for k in (
        "xipos", "ximat", "geom_xpos", "geom_xmat", "site_xpos", "site_xmat")}
    kinds = set(derived["body_sameframe"]) | set(derived["geom_sameframe"]) | set(derived["site_sameframe"])
    near_rot = m.site("free_near_rot").id
    if derived["site_sameframe"][near_rot] != 3 or m.site_quat[near_rot][1] == 0:
        sys.exit("flags: no turned frame within 1e-6 of its body's rotation")
    negated = m.site("negated_inertia_rot").id
    if derived["site_sameframe"][negated] != 4:
        sys.exit("flags: a quaternion negated is not taken as the inertial frame's")
    if kinds != {0, 1, 2, 3, 4} or set(derived["body_simple"]) != {0, 1, 2}:
        sys.exit(f"flags: not every kind, {kinds} {set(derived['body_simple'])}")
    invweight = {k: floats(getattr(m, k)) for k in ("body_invweight0", "dof_invweight0")}
    return {"xml": FLAGS, "qpos": qpos, "model": model, "derived": derived, "poses": poses,
            "invweight": invweight}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "fluid": fluid(),
        "cross_branch": cross_branch(),
        "negative_damping": negative_damping(),
        "feedback": feedback(),
        "factor": factor(),
        "sleeping": sleeping(),
        "flags": flags(),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "implicit.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
