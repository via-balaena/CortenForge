#!/usr/bin/env python3
"""Generate the RK4 golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_rk4_reference.py \\
        sim/L0/tests/assets/golden/rk4

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes rk4.json: for each fixture below, under RK4, the controls set
before the first step and act, qpos and qvel after each of `nstep` mj_step
calls. mj_RungeKutta's stages take each activation as the step's start plus
h times the weighted sum of the earlier stages' act_dot, whatever the
actuator's dynamics, and clamp nothing (engine_forward.c:1086-1089); the
advance takes the activation through mj_nextActivation, which does
(mj_advance, engine_forward.c:886-893).

Fixtures, a damped hinge with one actuator each:
- filterexact: `dyntype="filterexact" dynprm="0.03"`, where the exact filter
  advances an activation by tau(1 - exp(-h/tau)) = 0.00193 times act_dot
  over a step of h = 0.002;
- actrange: an integrator with `actlimited="true" actrange="0 0.1"` and a
  control of 1, whose activation reaches its limit and stays there, so that
  every later step's stages take it past the limit. The generator refuses
  the fixture if it does not.
"""

import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

HINGE = """<mujoco><option timestep="0.002" integrator="RK4"/>
<worldbody>
<body name="l" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
</worldbody>
<actuator><general name="a" joint="j" gainprm="1" {dyn}/></actuator></mujoco>"""

FIXTURES = {
    "filterexact": (HINGE.format(dyn='dyntype="filterexact" dynprm="0.03"'), [0.7], 200),
    "actrange": (HINGE.format(dyn='dyntype="integrator" actlimited="true" actrange="0 0.1"'), [1.0], 200),
}


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def fixture(name, xml, ctrl, nstep):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.ctrl[:] = ctrl
    steps = []
    for _ in range(nstep):
        mujoco.mj_step(m, d)
        steps.append({"act": floats(d.act), "qpos": floats(d.qpos), "qvel": floats(d.qvel)})
    if name == "actrange" and sum(s["act"][0] == 0.1 for s in steps) < nstep // 4:
        sys.exit("actrange: the activation does not stay at its limit")
    return {"name": name, "xml": xml, "ctrl": ctrl, "nstep": nstep, "steps": steps}


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    doc = {"oracle": marker,
           "fixtures": finite_or_string([fixture(name, *f) for name, f in FIXTURES.items()])}
    write(os.path.join(sys.argv[1], "rk4.json"), doc)
    print(f"{len(FIXTURES)} fixtures -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
