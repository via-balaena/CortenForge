#!/usr/bin/env python3
"""Generate the quaternion-joint transition golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_transition_reference.py \\
        sim/L0/tests/assets/golden/transition

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's `mjd_transitionFD` differences the perturbed next states with each
other (`mjd_stepFD`, `engine_derivative_fd.c:295-531`; `stateDiff`, `:55-64`),
so a quaternion joint's position rows are tangents at the next state
(`mj_differentiatePos`, `engine_support.c:608-638`). The cases:

- transitions: a ball pendulum, a hinge with a ball pendulum below it, and a
  free body, each spinning, without contact: the state and
  `mjd_transitionFD`'s A (centred, eps 1e-6), read on a copy of the data.
- actuated: `mjd_transitionFD`'s A and B with actuators: a hinge driven
  by a filter, and by an exact filter, with a time constant of 1e-12 (under
  the old 1e-10 floor and above mjMINVAL, so MuJoCo uses it as it is), under
  Euler, implicit and implicitfast (centred), the exact filter also at a
  timestep of 1e-11, where its decay over a step, exp(-h/tau), is e^-10 and
  not 0, a filter with 1e-16, which MuJoCo raises to mjMINVAL, and an exact
  filter with 1e-16 at a timestep of 1e-15, where the raised time constant
  makes the decay e^-1; and a free body driven by a motor at its site, its
  control at the lower end of its range, forward and centred (a forward
  column, and a centred one at a control's lower bound, difference the
  nominal next state with the perturbed one); and a hinge with a ball
  pendulum below it, driven at a site by an actuator whose gain has a
  velocity term (affine), under implicit and implicitfast (centred). Its
  velocity derivative moves with the control, and MuJoCo's control columns
  step from the velocity stage (`mj_stepSkip`, `engine_derivative_fd.c:350`),
  whose `mj_implicitSkip` keeps the nominal step's factor (`:140`); the
  generator refuses the case unless that B differs from one whose columns
  factor (full `mj_step`s) by more than 1e-4.
- differentiate: a model with a hinge, a slide, a ball and a free joint, and
  pairs of positions: a 1e-12 rad rotation, a rotation past pi between them,
  quaternions not of unit length, and a pair whose free joint moves, at dt
  0.002, 2e-6 and 7e-4 (where a division by dt rounds otherwise than a
  product with 1/dt, on a hinge or a slide and on the free joint's
  position, each): `mj_differentiatePos`.
- quat_integrate: `mjd_quatIntegrate`'s Dquat and Dvel for velocities and
  scales above and below its Taylor threshold (|scale*vel| = 1/32) and at 0.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

BALL = """<mujoco model="ball">
  <option timestep="0.002"/>
  <worldbody>
    <body pos="0 0 1">
      <joint type="ball" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0 0.1 0.05 -0.4" size="0.03" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""

HINGE_BALL = """<mujoco model="hinge_ball">
  <option timestep="0.002"/>
  <worldbody>
    <body pos="0 0 1">
      <joint type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="box" size="0.02 0.02 0.15" pos="0 0 -0.15" mass="0.8" contype="0" conaffinity="0"/>
      <body pos="0 0 -0.3">
        <joint type="ball" damping="0.05"/>
        <geom type="box" size="0.03 0.05 0.12" pos="0.02 0 -0.12" mass="0.5" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

FREE = """<mujoco model="free">
  <option timestep="0.002"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.06 0.03" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""

JOINTS = """<mujoco model="joints">
  <worldbody>
    <body>
      <joint type="hinge" axis="0 0 1"/>
      <geom size="0.1"/>
      <body pos="0.3 0 0">
        <joint type="slide" axis="1 0 0"/>
        <geom size="0.1"/>
        <body pos="0.3 0 0">
          <joint type="ball"/>
          <geom size="0.1"/>
        </body>
      </body>
    </body>
    <body pos="0 1 0">
      <freejoint/>
      <geom size="0.1"/>
    </body>
  </worldbody>
</mujoco>"""


FILTER = """<mujoco model="filter_tau">
  <option timestep="{timestep}" integrator="{integrator}"/>
  <worldbody>
    <body pos="0 0 1">
      <joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator>
    <general joint="j" gainprm="1" dyntype="{dyntype}" dynprm="{tau}"/>
  </actuator>
</mujoco>"""

MOTOR_FREE = """<mujoco model="motor_free">
  <option timestep="0.002"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.06 0.03" mass="1" contype="0" conaffinity="0"/>
      <site name="s" pos="0.05 0.02 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor site="s" gear="1 0.5 0 0 0 0.2" ctrllimited="true" ctrlrange="0 1"/>
  </actuator>
</mujoco>"""


AFFINE_SITE = """<mujoco model="affine_site">
  <option timestep="0.002" integrator="{integrator}"/>
  <worldbody>
    <body pos="0 0 1">
      <joint type="hinge" axis="0 1 0" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.03" mass="0.6" contype="0" conaffinity="0"/>
      <body pos="0.2 0 0">
        <joint type="ball" damping="0.05"/>
        <geom type="capsule" fromto="0 0 0 0.1 0.05 -0.3" size="0.03" mass="0.8" contype="0" conaffinity="0"/>
        <site name="s" pos="0.05 0.02 -0.15"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <general site="s" gear="1 0.5 0 0 0.2 0" gaintype="affine" gainprm="2 0 -3"/>
  </actuator>
</mujoco>"""

AFFINE_QPOS = [0.3, 0.8, -0.3, 0.4, 0.2]


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def unit(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def transition(xml, qpos, qvel):
    m = mujoco.MjModel.from_xml_string(xml)
    assert m.nq != m.nv, "a quaternion joint"
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    d.qvel[:] = qvel
    for j in range(m.njnt):
        if m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_BALL, mujoco.mjtJoint.mjJNT_FREE):
            a = m.jnt_qposadr[j] + (3 if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE else 0)
            d.qpos[a:a + 4] = unit(d.qpos[a:a + 4])
    mujoco.mj_forward(m, d)
    state = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    nx = 2 * m.nv + m.na
    a = np.zeros((nx, nx))
    # mjd_transitionFD leaves a perturbed evaluation in its data: a copy.
    fd = mujoco.MjData(m)
    fd.qpos[:], fd.qvel[:] = d.qpos, d.qvel
    mujoco.mj_forward(m, fd)
    mujoco.mjd_transitionFD(m, fd, 1e-6, 1, a, None, None, None)
    state["A"] = floats(a)
    return state


def actuated(xml, qpos, qvel, act, ctrl, centered):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[:], d.qvel[:], d.ctrl[:] = qpos, qvel, ctrl
    if m.na:
        d.act[:] = act
    for j in range(m.njnt):
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            a = m.jnt_qposadr[j] + 3
            d.qpos[a:a + 4] = unit(d.qpos[a:a + 4])
    mujoco.mj_forward(m, d)
    nx = 2 * m.nv + m.na
    a, b = np.zeros((nx, nx)), np.zeros((nx, m.nu))
    fd = mujoco.MjData(m)
    fd.qpos[:], fd.qvel[:], fd.ctrl[:] = d.qpos, d.qvel, d.ctrl
    if m.na:
        fd.act[:] = d.act
    mujoco.mj_forward(m, fd)
    mujoco.mjd_transitionFD(m, fd, 1e-6, int(centered), a, b, None, None)
    return {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel), "act": floats(d.act),
            "ctrl": floats(d.ctrl), "centered": centered, "A": floats(a), "B": floats(b)}


def refactoring_b(xml, qpos, qvel, ctrl, eps=1e-6):
    """B by centred differences of full `mj_step`s, each of which factors."""
    m = mujoco.MjModel.from_xml_string(xml)
    nx = 2 * m.nv + m.na
    b = np.zeros((nx, m.nu))

    def next_state(u):
        d = mujoco.MjData(m)
        d.qpos[:], d.qvel[:], d.ctrl[:] = qpos, qvel, u
        mujoco.mj_step(m, d)
        return d

    for i in range(m.nu):
        up, um = np.array(ctrl, dtype=float), np.array(ctrl, dtype=float)
        up[i] += eps
        um[i] -= eps
        dp, dm = next_state(up), next_state(um)
        dq = np.zeros(m.nv)
        mujoco.mj_differentiatePos(m, dq, 2 * eps, dm.qpos, dp.qpos)
        b[:m.nv, i] = dq
        b[m.nv:2 * m.nv, i] = (dp.qvel - dm.qvel) / (2 * eps)
    return b


def affine_site(integrator):
    xml = AFFINE_SITE.format(integrator=integrator)
    qpos = list(AFFINE_QPOS[:1]) + list(unit(AFFINE_QPOS[1:]))
    case = actuated(xml, qpos, [0.7, -0.4, 0.6, 0.5], [], [0.5], True)
    b = np.array(case["B"]).reshape(-1, 1)
    gap = np.max(np.abs(b - refactoring_b(xml, qpos, [0.7, -0.4, 0.6, 0.5], [0.5])))
    if gap <= 1e-4:
        sys.exit(f"affine_site {integrator}: B does not tell the nominal factor apart ({gap})")
    return case


def differentiate():
    m = mujoco.MjModel.from_xml_string(JOINTS)
    assert list(m.jnt_type) == [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE,
                                mujoco.mjtJoint.mjJNT_BALL, mujoco.mjtJoint.mjJNT_FREE]
    base = np.array([0.4, -0.2] + list(unit([0.8, 0.3, -0.4, 0.2])) + [0.1, 0.2, 0.3]
                    + list(unit([0.6, -0.2, 0.7, 0.3])))

    def turned(q, axis, angle):
        quat = np.zeros(4)
        mujoco.mju_axisAngle2Quat(quat, np.asarray(axis, dtype=float), angle)
        res = np.zeros(4)
        mujoco.mju_mulQuat(res, np.asarray(q, dtype=float), quat)
        return res

    pairs = {}
    q2 = base.copy()
    q2[0] += 1e-9
    q2[1] -= 3e-9
    q2[2:6] = turned(base[2:6], [0.3, -0.5, 0.8], 1e-12)
    q2[6:9] += [1e-9, 0, -2e-9]
    q2[9:13] = turned(base[9:13], [1, 0, 0], 1e-12)
    pairs["tiny"] = (base, q2)
    q2 = base.copy()
    q2[2:6] = -turned(base[2:6], [0, 1, 0], 2.9)     # the same rotation, the quaternion negated
    q2[9:13] = turned(base[9:13], [0.6, 0.0, -0.8], 3.6)  # past pi: the short way back
    pairs["past_pi"] = (base, q2)
    q1 = base.copy()
    q2 = base.copy()
    q1[2:6] *= 1.1
    q2[9:13] = 0.9 * turned(base[9:13], [0.2, 0.7, 0.1], 0.3)
    pairs["not_unit"] = (q1, q2)
    q2 = base.copy()
    q2[6:9] += [0.0123, -0.0456, 0.0789]
    pairs["free_moves"] = (base, q2)
    out = {"xml": JOINTS, "cases": []}
    told = {"hinge_slide": False, "free": False}
    for name, (q1, q2) in pairs.items():
        for dt in (0.002, 2e-6, 7e-4):
            # a test of a division by dt tells it from a product with 1/dt
            for cls, idx in (("hinge_slide", (0, 1)), ("free", (6, 7, 8))):
                told[cls] |= any((q2[i] - q1[i]) / dt != (q2[i] - q1[i]) * (1.0 / dt) for i in idx)
            v = np.zeros(m.nv)
            mujoco.mj_differentiatePos(m, v, dt, q1, q2)
            out["cases"].append({"name": name, "dt": dt, "qpos1": floats(q1), "qpos2": floats(q2),
                                 "qvel": floats(v)})
    if not all(told.values()):
        sys.exit(f"differentiate: a class whose divisions by dt all round as products: {told}")
    return out


def quat_integrate():
    out = []
    for vel, scale in (([0.8, -1.3, 0.5], 0.002), ([0.8, -1.3, 0.5], 0.5), ([3.0, 1.0, -2.0], 0.01),
                       ([0.0, 0.0, 0.0], 0.002), ([1e-3, 2e-3, -1e-3], 1.0)):
        vel = np.array(vel)
        dquat, dvel, dscale = np.zeros(9), np.zeros(9), np.zeros(3)
        mujoco.mjd_quatIntegrate(vel, scale, dquat, dvel, dscale)
        out.append({"vel": floats(vel), "scale": scale, "Dquat": floats(dquat), "Dvel": floats(dvel),
                    "taylor": bool(np.linalg.norm(scale * vel) <= 1 / 32)})
    if {c["taylor"] for c in out} != {True, False}:
        sys.exit("quat_integrate: not both branches")
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "transitions": {
            "ball": transition(BALL, [0.9, 0.2, -0.3, 0.1], [0.6, -0.5, 0.4]),
            "hinge_ball": transition(HINGE_BALL, [0.3, 0.8, -0.3, 0.4, 0.2], [0.7, -0.4, 0.6, 0.5]),
            "free": transition(FREE, [0.1, -0.2, 1.0, 0.7, 0.3, -0.5, 0.4],
                               [0.2, -0.1, 0.3, 0.9, -0.6, 0.7]),
        },
        "actuated": {
            **{f"{t}_{i}": actuated(FILTER.format(timestep=0.002, integrator=i, dyntype=t, tau=1e-12),
                                    [0.3], [0.2], [0.1], [0.5], True)
               for t in ("filter", "filterexact") for i in ("Euler", "implicit", "implicitfast")},
            "filterexact_short_step": actuated(FILTER.format(timestep=1e-11, integrator="Euler",
                                                             dyntype="filterexact", tau=1e-12),
                                               [0.3], [0.2], [0.1], [0.5], True),
            "filter_floor": actuated(FILTER.format(timestep=0.002, integrator="Euler",
                                                   dyntype="filter", tau=1e-16),
                                     [0.3], [0.2], [0.1], [0.5], True),
            "filterexact_floor": actuated(FILTER.format(timestep=1e-15, integrator="Euler",
                                                        dyntype="filterexact", tau=1e-16),
                                          [0.3], [0.2], [0.1], [0.5], True),
            "motor_free_forward": actuated(MOTOR_FREE, [0.1, -0.2, 1.0, 0.7, 0.3, -0.5, 0.4],
                                           [0.2, -0.1, 0.3, 0.9, -0.6, 0.7], [], [0.0], False),
            "motor_free_centred": actuated(MOTOR_FREE, [0.1, -0.2, 1.0, 0.7, 0.3, -0.5, 0.4],
                                           [0.2, -0.1, 0.3, 0.9, -0.6, 0.7], [], [0.0], True),
            **{f"affine_site_{i}": affine_site(i) for i in ("implicit", "implicitfast")},
        },
        "differentiate": differentiate(),
        "quat_integrate": quat_integrate(),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "transition.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
