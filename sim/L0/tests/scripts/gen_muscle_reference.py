#!/usr/bin/env python3
"""Generate the muscle golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_muscle_reference.py \\
        sim/L0/tests/assets/golden/muscle

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes muscle.json: for each fixture below, a hinge with one
`<general gaintype="muscle" biastype="muscle" dyntype="none">` actuator,
MuJoCo's actuator_lengthrange and actuator_acc0, and, at each state (qpos,
qvel, ctrl), actuator_force and qfrc_actuator after mj_forward. MuJoCo keeps a
muscle's negative force in the model and resolves it as
scale / max(mjMINVAL, acc0) at each call, the gain from gainprm and the bias
from biasprm (mju_muscleGain, mju_muscleBias, engine_util_misc.c:653-725),
clamping each divisor at mjMINVAL.

Fixtures:
- auto_force: both forces -1 (A20 §1A.3.8's case: a general actuator with
  muscle gain and bias but no muscle dynamics), lengthrange -1 1;
- bias_from_biasprm: the gain's force 300 and the bias's 100, at three
  states (two past the optimal length, where the passive force is not 0)
  and, with a control of 0, seven where the force is the passive force
  alone, five on its quadratic branch and two past its midpoint, on its
  linear one (MuJoCo multiplies it left to right, -force*fpmax*0.5*x*x
  and -force*fpmax*(0.5 + x));
- narrow_range: both forces 300 and a lengthrange 1e-12 wide, so that the
  optimal length is 3.3e-12, at states 1e-13 from the range's end, one
  moving at 1e-12, a normalized velocity of 0.2;
- tiny_acc0: auto_force with a gear of 1e-18, so that acc0 is 3.1e-17:
  MuJoCo stores acc0 unclamped (engine_setconst.c:776) and clamps it at
  mjMINVAL where the force is resolved;
- acc0_order: a muscle on a fixed tendon over five slides, each on its own
  body, whose acc0 MuJoCo's `mju_norm` (four interleaved sums,
  engine_util_blas.c:698-765) gives 1.754197806780044 and a sum of squares
  left to right 1.7541978067800443;
- tiny_fv: explicit forces and an fvmax of 1 + 5e-11, at normalized
  velocities between 0 and 5e-11, where the force-velocity curve divides by
  max(mjMINVAL, fvmax - 1);
- flat_lmax: explicit forces and an lmax of 1 + 1e-10, so that the length
  curve's and the passive force's midpoint b is 5e-11 past 1, at normalized
  lengths 1 + 3e-11, where both divide by max(mjMINVAL, b - 1), and
  1 + 2e-10, past lmax, where only the passive force's linear branch does;
- zero_force: both forces 0, which MuJoCo leaves at 0 (only a negative
  force is resolved from acc0).

The fixtures named in STEPPED also step once from each of their states under
implicit and implicitfast, whose velocity derivative of the muscle force
(`mjd_muscleGain_vel`, engine_derivative.c:781-826) divides by the same
clamped quantities, and record qpos and qvel.

Each fixture isolates one difference: auto_force the resolution of the force
(its range is wide and its gain and bias parameters equal), bias_from_biasprm
the bias's parameters, narrow_range the clamps (its forces are explicit),
tiny_acc0 the acc0 floor, acc0_order its summation order, tiny_fv and
flat_lmax their clamps, zero_force a comparison.
"""

import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

HINGE = """<mujoco>
<worldbody>
<body name="l" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>
</worldbody>
<actuator><general name="m" joint="j" gaintype="muscle" biastype="muscle" dyntype="none"
  gainprm="0.75 1.05 {gain} 200 0.5 1.6 1.5 1.3 1.2" biasprm="0.75 1.05 {bias} 200 0.5 1.6 1.5 1.3 1.2"
  lengthrange="{lr}"/></actuator></mujoco>"""

SLIDES = """<mujoco>
<worldbody>
""" + "".join(f"""<body pos="{i} 0 0"><joint name="s{i}" type="slide" axis="1 0 0"/>
  <geom type="sphere" size="0.1" mass="{mass}"/></body>
""" for i, mass in enumerate([1.78, 0.703, 2.276, 0.631, 2.018])) + """</worldbody>
<tendon><fixed name="t">""" + "".join(f'<joint joint="s{i}" coef="{c}"/>' for i, c in enumerate([1.107, 0.622, 2.023, 0.403, 1.7])) + """</fixed></tendon>
<actuator><general name="m" tendon="t" gaintype="muscle" biastype="muscle" dyntype="none"
  gainprm="0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2" biasprm="0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2"
  lengthrange="-1 1"/></actuator></mujoco>"""

FIXTURES = {
    "auto_force": (HINGE.format(gain=-1, bias=-1, lr="-1 1"),
                   [(0.0, 0.0, 0.6), (0.2, 0.3, 0.6), (-0.4, -0.5, 1.0), (0.7, 0.1, 0.3)]),
    "bias_from_biasprm": (HINGE.format(gain=300, bias=100, lr="-1 1"),
                          [(0.5, 0.0, 0.6), (0.8, 0.2, 0.6), (0.95, -0.1, 0.2),
                           (0.67, 0.0, 0.0), (0.75, 0.0, 0.0), (0.78, 0.0, 0.0), (0.87, 0.0, 0.0),
                           (0.95, 0.0, 0.0), (2.702, 0.0, 0.0), (2.708, 0.0, 0.0)]),
    "narrow_range": (HINGE.format(gain=300, bias=300, lr="0 1e-12"),
                     [(1e-13, 0.0, 0.6), (3e-13, 0.0, 0.6), (5e-13, 0.0, 1.0), (1e-13, 1e-12, 0.6)]),
    "tiny_acc0": (HINGE.format(gain=-1, bias=-1, lr="-1 1").replace('dyntype="none"', 'dyntype="none" gear="1e-18"'),
                  [(0.0, 0.0, 0.6), (0.2, 0.3, 0.6)]),
    "acc0_order": (SLIDES, [(0.0, 0.0, 0.6), (0.3, 0.2, 0.6)]),
    "tiny_fv": (HINGE.format(gain=300, bias=300, lr="-1 1").replace(" 1.3 1.2\"", " 1.3 1.00000000005\""),
                [(0.1, 2.5e-10, 0.6), (0.2, 4e-10, 1.0)]),
    "flat_lmax": (HINGE.format(gain=300, bias=300, lr="-1 1").replace(" 0.5 1.6 1.5", " 0.5 1.0000000001 1.5"),
                  [(-1.0 + (1.0 + 3e-11 - 0.75) * (2.0 / 0.3), 0.0, 0.6),
                   (-1.0 + (1.0 + 2e-10 - 0.75) * (2.0 / 0.3), 0.0, 0.6)]),
    "zero_force": (HINGE.format(gain=0, bias=0, lr="-1 1"), [(0.2, 0.3, 0.6), (0.9, 0.0, 0.0)]),
}

# Fixtures that also step once under each implicit integrator.
STEPPED = ("narrow_range", "tiny_fv")


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def passive(m, d, qpos, qvel):
    """The actuator's force with a control of 0: its bias alone."""
    mujoco.mj_resetData(m, d)
    d.qpos[0], d.qvel[0] = qpos, qvel
    mujoco.mj_forward(m, d)
    return float(d.actuator_force[0])


def implicit_step(xml, integrator, state):
    """One step under `integrator` from `state`: qpos and qvel after it."""
    xml = xml.replace("<mujoco>", f'<mujoco><option integrator="{integrator}"/>', 1)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[0], d.qvel[0], d.ctrl[0] = state
    mujoco.mj_step(m, d)
    return {"integrator": integrator, "xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel)}


def fixture(name, xml, states):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    out = []
    for qpos, qvel, ctrl in states:
        mujoco.mj_resetData(m, d)
        d.qpos[0], d.qvel[0], d.ctrl[0] = qpos, qvel, ctrl
        mujoco.mj_forward(m, d)
        out.append({"qpos": qpos, "qvel": qvel, "ctrl": ctrl,
                    "actuator_force": floats(d.actuator_force), "qfrc_actuator": floats(d.qfrc_actuator)})
    if name == "bias_from_biasprm" and not any(passive(m, d, s[0], s[1]) for s in states):
        sys.exit("bias_from_biasprm: the passive force is 0 at every state")
    if name == "tiny_fv" and not all(0 < s[1] / 10.0 < 5e-11 for s in states):
        sys.exit("tiny_fv: a normalized velocity outside (0, 5e-11)")
    steps = [dict(implicit_step(xml, i, state), state=k) for k, state in enumerate(states)
             for i in ("implicit", "implicitfast")] if name in STEPPED else []
    return {"name": name, "xml": xml, "lengthrange": floats(m.actuator_lengthrange[0]),
            "acc0": float(m.actuator_acc0[0]), "gainprm": floats(m.actuator_gainprm[0]),
            "biasprm": floats(m.actuator_biasprm[0]), "states": out, "implicit_steps": steps}


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
    write(os.path.join(sys.argv[1], "muscle.json"), doc)
    print(f"{len(FIXTURES)} fixtures -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
