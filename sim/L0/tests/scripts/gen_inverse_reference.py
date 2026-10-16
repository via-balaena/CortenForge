#!/usr/bin/env python3
"""Generate the inverse dynamics golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_inverse_reference.py \\
        sim/L0/tests/assets/golden/inverse

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's `mj_inverse` takes the constraint force from `qacc`
(`mj_invConstraint`, `engine_inverse.c:157-180`): each row's state and force
as `mj_constraintUpdate` makes them at `jar = J·qacc − aref`, then
`qfrc_constraint = J'·force`. The cases, each after mj_forward at its
state (qpos, qvel):

- limit: a hinge held past its limit, with damping, carrying a second
  hinge; contact: a free box resting on a plane, pyramidal, condim 3;
  elliptic: the same box, elliptic. For each: mj_inverse at the forward's
  qacc and at that qacc plus a shift, giving qfrc_inverse, efc_force,
  qfrc_constraint and efc_state; and mjd_inverseFD (forward differences,
  eps 1e-6, no actuation), DfDq, DfDv and DfDa, row-major as MuJoCo
  writes them.
- limit_fwdinv, contact_fwdinv, free_fwdinv: the limit and contact cases,
  the contact one with a wrench on the box (xfrc_applied), and a damped
  three-hinge chain with no rows, with the fwdinv flag enabled: their state (qpos,
  qvel, qacc_warmstart, time), then after mj_forward solver_fwdinv (both 0
  with no rows), efc_force, qfrc_constraint and efc_state
  (`mj_compareFwdInv`, `engine_inverse.c:262-302`, puts back the forward's
  qfrc_constraint and efc_force, not efc_state).
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

LIMIT = """<mujoco model="limit">
  <option timestep="0.002"/>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0" limited="true" range="-0.5 0.5" damping="0.2"/>
      <geom type="box" size="0.2 0.03 0.03" pos="0.2 0 0" mass="1"/>
      <body pos="0.4 0 0">
        <joint name="h2" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.05" mass="0.3"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""


def box(cone):
    return f"""<mujoco model="box_{cone}">
  <option timestep="0.002" cone="{cone}"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1"/>
    <body name="box" pos="0 0 0.0995" euler="0.02 -0.01 0.3">
      <freejoint/>
      <geom type="box" size="0.2 0.15 0.1" mass="2"/>
    </body>
  </worldbody>
</mujoco>"""


CASES = {
    "limit": (LIMIT, [0.52, -0.3], [0.4, -0.2], [0.7, -1.3]),
    "contact": (box("pyramidal"), None, None, [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]),
    "elliptic": (box("elliptic"), None, None, [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]),
}


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def state(m, qpos, qvel):
    d = mujoco.MjData(m)
    if qpos is not None:
        d.qpos[:] = qpos
    else:
        # A box settled on the plane: 100 steps from rest.
        for _ in range(100):
            mujoco.mj_step(m, d)
    if qvel is not None:
        d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    return d


def inverse_at(m, d, qacc):
    d.qacc[:] = qacc
    mujoco.mj_inverse(m, d)
    return {"qacc": floats(d.qacc), "qfrc_inverse": floats(d.qfrc_inverse),
            "efc_force": floats(d.efc_force), "qfrc_constraint": floats(d.qfrc_constraint),
            "efc_state": ints(d.efc_state)}


def case(xml, qpos, qvel, shift):
    m = mujoco.MjModel.from_xml_string(xml)
    d = state(m, qpos, qvel)
    if d.nefc == 0:
        sys.exit(f"{m.names[:20]}: no constraint rows")
    out = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel), "qacc_warmstart": floats(d.qacc_warmstart),
           "time": float(d.time), "nefc": int(d.nefc), "forward_qacc": floats(d.qacc),
           "forward_efc_force": floats(d.efc_force)}
    qacc = d.qacc.copy()
    out["at_forward_qacc"] = inverse_at(m, d, qacc)
    out["at_shifted_qacc"] = inverse_at(m, d, qacc + np.array(shift))
    d = state(m, out["qpos"], out["qvel"]) if qpos is not None else d
    d.qacc[:] = qacc
    mujoco.mj_forward(m, d)
    nv = m.nv
    dfdq, dfdv, dfda = (np.zeros((nv, nv)) for _ in range(3))
    mujoco.mjd_inverseFD(m, d, 1e-6, 0, dfdq, dfdv, dfda, None, None, None, None)
    out.update({"DfDq": floats(dfdq), "DfDv": floats(dfdv), "DfDa": floats(dfda)})
    return out


FREE = """<mujoco model="free">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1"><joint type="hinge" axis="0 1 0" damping="0.3"/>
      <geom type="box" size="0.15 0.02 0.02" pos="0.15 0 0" mass="0.7"/>
      <body pos="0.3 0 0"><joint type="hinge" axis="0.3 1 0.2" damping="0.1"/>
        <geom type="box" size="0.1 0.02 0.02" pos="0.1 0 0" mass="0.4"/>
        <body pos="0.2 0 0"><joint type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.04" pos="0.05 0.01 0" mass="0.2"/></body></body></body>
  </worldbody>
</mujoco>"""


def fwdinv(name, xfrc=None):
    """Case `name` with the fwdinv flag enabled, at its state, with `xfrc`
    (body, wrench) applied: after mj_forward, solver_fwdinv, efc_force,
    qfrc_constraint and efc_state."""
    xml, qpos, qvel, _ = CASES[name] if name in CASES else (FREE, [0.4, -0.7, 1.1], [0.3, -0.2, 0.5], None)
    on = xml.replace('<option timestep="0.002"', '<option timestep="0.002" fwdinv_flag', 1)
    on = on.replace(' fwdinv_flag/>', '>\n    <flag fwdinv="enable"/>\n  </option>', 1)
    on = on.replace(' fwdinv_flag cone="pyramidal"/>',
                    ' cone="pyramidal">\n    <flag fwdinv="enable"/>\n  </option>', 1)
    assert 'fwdinv="enable"' in on and 'fwdinv_flag' not in on, on
    m = mujoco.MjModel.from_xml_string(on)
    reached = state(m, qpos, qvel)
    before = {"qpos": floats(reached.qpos), "qvel": floats(reached.qvel),
              "qacc_warmstart": floats(reached.qacc_warmstart), "time": float(reached.time)}
    # The forward on a fresh MjData from that state, as the test runs it.
    d = mujoco.MjData(m)
    d.qpos[:] = before["qpos"]
    d.qvel[:] = before["qvel"]
    d.qacc_warmstart[:] = before["qacc_warmstart"]
    d.time = before["time"]
    if xfrc is not None:
        d.xfrc_applied[xfrc[0]] = xfrc[1]
        before["xfrc_applied"] = [xfrc[0], list(xfrc[1])]
    mujoco.mj_forward(m, d)
    return {"xml": on, **before, "solver_fwdinv": floats(d.solver_fwdinv),
            "efc_force": floats(d.efc_force), "qfrc_constraint": floats(d.qfrc_constraint),
            "efc_state": ints(d.efc_state)}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {name: case(*args) for name, args in CASES.items()}
    cases["limit_fwdinv"] = fwdinv("limit")
    cases["contact_fwdinv"] = fwdinv("contact", (1, [0.3, -0.2, 0.5, 0.01, 0.02, -0.03]))
    cases["free_fwdinv"] = fwdinv("free")
    out = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "inverse.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
