#!/usr/bin/env python3
"""Generate the mass-matrix factor golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_mass_factor_reference.py \\
        sim/L0/tests/assets/golden/mass_factor

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo factors the mass matrix on its own pattern, a simple dof's row holding
its diagonal only (`mj_factorM`, `engine_core_smooth.c:1961-1986`): it copies
the awake rows of `d.M` into `d.qLD` and runs `mj_factorI` over them; a
sleeping row keeps what it held. `mj_solveM` solves with that factor over
every row (`mj_solveLD`, `:2131-2224`). Euler with dof damping factors
`M + h·diag(damping)` the same way into `d.qH` (`mj_EulerSkip`,
`engine_forward.c:943-1009`). The cases:

- chain: a hinge-hinge-ball chain with a hinge off its root, no damping:
  after mj_forward at a moved state, `d.M` and the factor `d.qLD` and
  `d.qLDiagInv` made of it, and `mj_solveM` of a vector `y`. sim-core's
  mass matrix rounds otherwise than MuJoCo's here, so the test factors
  MuJoCo's.
- eulerdamp: the chain with dof damping, under Euler: `d.qH` and
  `d.qHDiagInv` after the first step, and qpos and qvel after steps 1, 10
  and 100.
- eulerdamp_sleeping: two damped double pendulums, one asleep from the
  start, under Euler: `d.qH` and `d.qHDiagInv` after the first step, whose
  sleeping rows MuJoCo has not filled (0).
- two_slides: a simple body with two slides along one axis and one across
  it, under Euler. MuJoCo stores the body's block of the mass matrix as its
  diagonal only, though the two slides couple: after mj_forward, `d.M`,
  `d.qLD`, `d.qLDiagInv` and `d.qacc`; then qvel after steps 1 and 10.
- sleeping: two double pendulums, one asleep from the start
  (`sleep="init"`), under Euler: after 10 steps, `d.M` and `d.qLD` and
  `d.qLDiagInv`, whose awake rows are the factor of that `d.M` and whose
  sleeping rows were factored before the tree fell asleep; and which dofs
  are awake.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

CHAIN = """<mujoco model="chain">
  <option timestep="0.005" integrator="Euler"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="j0" type="hinge" axis="0 1 0"{d0}/>
      <geom type="box" size="0.03 0.03 0.15" pos="0 0 -0.15" mass="1"/>
      <body name="b" pos="0 0 -0.3">
        <joint name="j1" type="hinge" axis="1 0 0"{d1}/>
        <geom type="box" size="0.03 0.03 0.12" pos="0.02 0 -0.12" mass="0.7"/>
        <body name="c" pos="0 0 -0.25">
          <joint name="j2" type="ball"{d2}/>
          <geom type="box" size="0.05 0.03 0.08" pos="0.03 0.01 -0.08" mass="0.4"/>
        </body>
      </body>
      <body name="d" pos="0.2 0 0">
        <joint name="j3" type="hinge" axis="0 0 1"{d3}/>
        <geom type="box" size="0.1 0.02 0.02" pos="0.1 0 0" mass="0.5"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

TWO_SLIDES = """<mujoco model="two_slides">
  <option timestep="0.002" integrator="Euler" gravity="2 0 -9.81"/>
  <worldbody>
    <body name="slider" pos="0 0 1">
      <joint type="slide" axis="1 0 0" armature="0.1"/>
      <joint type="slide" axis="1 0 0" armature="0.2"/>
      <joint type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>"""

SLEEPING = """<mujoco model="sleeping">
  <option timestep="0.002" integrator="Euler">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <body name="moving" pos="0 0 1">
      <joint type="hinge" axis="0 1 0"{d0}/>
      <geom type="box" size="0.02 0.02 0.2" pos="0 0 -0.2" mass="1"/>
      <body pos="0 0 -0.4">
        <joint type="hinge" axis="1 0 0"{d1}/>
        <geom type="box" size="0.02 0.02 0.1" pos="0 0 -0.1" mass="0.5"/>
      </body>
    </body>
    <body name="resting" pos="1 0 1" sleep="init">
      <joint type="hinge" axis="0 1 0"{d2}/>
      <geom type="box" size="0.02 0.02 0.2" pos="0 0 -0.2" mass="1"/>
      <body pos="0 0 -0.4">
        <joint type="hinge" axis="0 1 0"{d3}/>
        <geom type="box" size="0.03 0.02 0.1" pos="0.01 0 -0.1" mass="0.5"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

STEPS = (1, 10, 100)
CHAIN_QPOS = [0.6, -0.5, 0.7, -0.4, 0.3, 0.5, -0.2]
CHAIN_QVEL = [1.3, -0.9, 0.6, -1.7, 1.1, 0.4]


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def stable(d, what):
    for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
              mujoco.mjtWarning.mjWARN_BADQPOS):
        if d.warning[w].number:
            sys.exit(f"{what}: MuJoCo warned {w.name}")


def chain_state(m, d):
    d.qpos[:] = CHAIN_QPOS
    d.qpos[2:6] /= np.linalg.norm(d.qpos[2:6])
    d.qvel[:] = CHAIN_QVEL
    return floats(d.qpos), floats(d.qvel)


def forward(m, d):
    """What mj_forward leaves of the factor and the acceleration it solved."""
    mujoco.mj_forward(m, d)
    return {"M": floats(d.M), "qLD": floats(d.qLD), "qLDiagInv": floats(d.qLDiagInv),
            "qacc": floats(d.qacc)}


def trajectory(m, d, what, steps, record=None):
    out = {}
    for step in range(1, max(steps) + 1):
        mujoco.mj_step(m, d)
        if step == 1 and record:
            out["first"] = record(d)
        if step in steps:
            out[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    stable(d, what)
    return out


def chain():
    xml = CHAIN.format(d0="", d1="", d2="", d3="")
    m = mujoco.MjModel.from_xml_string(xml)
    if max(m.M_rownnz) != 5 or any(m.dof_simplenum):
        sys.exit(f"chain: not the pattern meant, {list(m.M_rownnz)} {list(m.dof_simplenum)}")
    d = mujoco.MjData(m)
    qpos, qvel = chain_state(m, d)
    mujoco.mj_forward(m, d)
    y = np.array([0.7, -1.3, 0.4, 2.1, -0.9, 1.6])
    x = np.zeros(m.nv)
    mujoco.mj_solveM(m, d, x.reshape(1, -1), y.reshape(1, -1))
    return {"xml": xml, "qpos": qpos, "qvel": qvel, "M": floats(d.M), "qLD": floats(d.qLD),
            "qLDiagInv": floats(d.qLDiagInv), "y": floats(y), "x": floats(x)}


def eulerdamp():
    xml = CHAIN.format(d0=' damping="0.3"', d1=' damping="0.2"', d2=' damping="0.1"',
                       d3=' damping="0.25"')
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    qpos, qvel = chain_state(m, d)
    case = {"xml": xml, "qpos": qpos, "qvel": qvel}
    case["traj"] = trajectory(
        m, d, "eulerdamp", STEPS,
        record=lambda d: {"qH": floats(d.qH), "qHDiagInv": floats(d.qHDiagInv)})
    return case


def two_slides():
    m = mujoco.MjModel.from_xml_string(TWO_SLIDES)
    if m.body_simple[1] != 2 or list(m.M_rownnz) != [1, 1, 1]:
        sys.exit(f"two_slides: {list(m.body_simple)} {list(m.M_rownnz)}")
    d = mujoco.MjData(m)
    d.qvel[:] = [0.3, -0.2, 0.1]
    qvel = floats(d.qvel)
    case = {"xml": TWO_SLIDES, "qvel": qvel, "forward": forward(m, d)}
    case["traj"] = trajectory(m, d, "two_slides", (1, 10))
    return case


def sleeping_start(xml, what):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if d.nv_awake != 2:
        sys.exit(f"{what}: {d.nv_awake} dofs awake at the start")
    d.qpos[0:2] = [0.5, -0.4]
    d.qvel[0:2] = [1.2, -0.8]
    return m, d


def eulerdamp_sleeping():
    xml = SLEEPING.format(d0=' damping="0.5"', d1=' damping="0.3"', d2=' damping="0.4"',
                          d3=' damping="0.3"')
    m, d = sleeping_start(xml, "eulerdamp_sleeping")
    case = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    mujoco.mj_step(m, d)
    stable(d, "eulerdamp_sleeping")
    if d.nv_awake != 2 or np.any(d.qHDiagInv[2:]) or not np.all(d.qHDiagInv[:2]):
        sys.exit(f"eulerdamp_sleeping: not only the awake rows factored, {list(d.qHDiagInv)}")
    case.update({"qH": floats(d.qH), "qHDiagInv": floats(d.qHDiagInv)})
    return case


def sleeping():
    xml = SLEEPING.format(d0="", d1="", d2="", d3="")
    m, d = sleeping_start(xml, "sleeping")
    case = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    for _ in range(10):
        mujoco.mj_step(m, d)
    stable(d, "sleeping")
    if d.nv_awake != 2:
        sys.exit(f"sleeping: {d.nv_awake} dofs awake after 10 steps")
    awake = [int(i) for i in d.dof_awake_ind[:d.nv_awake]]
    # the awake rows are the factor of d.M: mj_factorM of it changes nothing
    again = mujoco.MjData(m)
    mujoco.mj_copyData(again, m, d)
    mujoco.mj_factorM(m, again)
    if not (np.array_equal(again.qLD, d.qLD) and np.array_equal(again.qLDiagInv, d.qLDiagInv)):
        sys.exit("sleeping: d.qLD is not the factor of d.M")
    case.update({"steps": 10, "awake": awake, "M": floats(d.M), "qLD": floats(d.qLD),
                 "qLDiagInv": floats(d.qLDiagInv)})
    return case


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "chain": chain(),
        "eulerdamp": eulerdamp(),
        "eulerdamp_sleeping": eulerdamp_sleeping(),
        "two_slides": two_slides(),
        "sleeping": sleeping(),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "mass_factor.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
