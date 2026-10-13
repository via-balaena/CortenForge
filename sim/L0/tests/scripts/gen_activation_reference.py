#!/usr/bin/env python3
"""Generate the activation golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_activation_reference.py \\
        sim/L0/tests/assets/golden/activation

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes activation.json: for each fixture below, a hinge with one actuator
and a control of 1, its act_dot and actuator_force after mj_forward, and act,
act_dot, qpos and qvel after each of `nstep` mj_step calls. MuJoCo's
mj_fwdActuation computes every awake actuator's act_dot, a group-disabled
one's included (engine_forward.c:321-372), and gives a disabled actuator no
force; mj_advance gives it an act_dot of 0 (engine_forward.c:893). A filter's
time constant is clamped at mjMINVAL, in act_dot (:352) and in
mj_nextActivation (engine_support.c:718).

Fixtures:
- disabled: a filter (tau 0.2) in group 2, with `actuatorgroupdisable="2"`,
  under Euler and under RK4;
- tiny_tau: a filter with tau 1e-12;
- tiny_tau_exact: an exact filter with tau 1e-12, under Euler and under RK4;
- muscle_smooth: muscle dynamics with a smoothing width of 0.3, 60 steps,
  whose activation passes through the width (mju_muscleDynamics,
  engine_util_misc.c:752-770: the sigmoid of dctrl / width + 0.5);
- muscle_tiny_tau: muscle dynamics with an activation time constant of
  1e-13, which MuJoCo clamps at mjMINVAL (:769).
"""

import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

HINGE = """<mujoco><option timestep="0.002" integrator="{integrator}"{disable}/>
<worldbody>
<body name="l" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" damping="0.1"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
</worldbody>
<actuator><general name="a" joint="j" group="{group}" gainprm="1" {dyn}/></actuator></mujoco>"""

FIXTURES = {
    "disabled_Euler": (HINGE.format(integrator="Euler", disable=' actuatorgroupdisable="2"', group=2,
                                    dyn='dyntype="filter" dynprm="0.2"'), 5),
    "disabled_RK4": (HINGE.format(integrator="RK4", disable=' actuatorgroupdisable="2"', group=2,
                                  dyn='dyntype="filter" dynprm="0.2"'), 5),
    "tiny_tau": (HINGE.format(integrator="Euler", disable="", group=0,
                              dyn='dyntype="filter" dynprm="1e-12"'), 1),
    "tiny_tau_exact": (HINGE.format(integrator="Euler", disable="", group=0,
                                    dyn='dyntype="filterexact" dynprm="1e-12"'), 1),
    "tiny_tau_exact_RK4": (HINGE.format(integrator="RK4", disable="", group=0,
                                        dyn='dyntype="filterexact" dynprm="1e-12"'), 1),
    "muscle_smooth": (HINGE.format(integrator="Euler", disable="", group=0,
                                   dyn='dyntype="muscle" dynprm="0.01 0.04 0.3"'), 60),
    "muscle_tiny_tau": (HINGE.format(integrator="Euler", disable="", group=0,
                                     dyn='dyntype="muscle" dynprm="1e-13 0.04 0"'), 1),
}


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def fixture(name, xml, nstep):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.ctrl[0] = 1.0
    mujoco.mj_forward(m, d)
    out = {"name": name, "xml": xml, "nstep": nstep,
           "forward": {"act_dot": floats(d.act_dot), "actuator_force": floats(d.actuator_force)}, "steps": []}
    for _ in range(nstep):
        mujoco.mj_step(m, d)
        out["steps"].append({"act": floats(d.act), "act_dot": floats(d.act_dot),
                             "qpos": floats(d.qpos), "qvel": floats(d.qvel)})
    return out


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
    write(os.path.join(sys.argv[1], "activation.json"), doc)
    print(f"{len(FIXTURES)} fixtures -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
