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
  qfrc_constraint and efc_state, and then mj_rnePostConstraint's cfrc_ext,
  from the inverse's force, moved to each body's xpos; and mjd_inverseFD (forward differences,
  eps 1e-6, no actuation), DfDq, DfDv and DfDa, row-major as MuJoCo
  writes them.
- limit_fwdinv, contact_fwdinv, free_fwdinv, pgs_fwdinv, floss_fwdinv,
  noslip_fwdinv, rk4_fwdinv, with the fwdinv flag enabled: the limit and
  contact cases, the contact one with a wrench on the box (xfrc_applied);
  a damped three-hinge chain with no rows; the box under PGS at 4
  iterations, settled 100 steps, where the inverse's constraint force is
  not the forward's; the limit case with friction loss on both hinges and
  the elliptic box with 5 noslip iterations, where the inverse's row states
  are not the forward's; and the PGS box under RK4.
  Their state (qpos, qvel, qacc_warmstart, time), then from it, each on a
  fresh MjData: after mj_forward, solver_fwdinv (mj_forward does not
  compare); after mj_step, which runs `mj_compareFwdInv`
  (`engine_inverse.c:262-302`) after its forward pass and mj_checkAcc
  (`engine_forward.c:1431-1434`), solver_fwdinv and efc_state (the
  comparison puts back the forward's qfrc_constraint and efc_force, not
  efc_state). It refuses the file unless the PGS case's solver_fwdinv[0]
  and the RK4 case's are above 1e-6 and the friction-loss and noslip
  cases' states after mj_step differ from those after mj_forward.
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


def cfrc_ext_at_origin(m, d):
    """cfrc_ext ([torque; force]) moved from the subtree COM to each body's
    xpos, where sim-core keeps it (gen_accumulators_reference.py's)."""
    out = []
    for b in range(m.nbody):
        c = d.subtree_com[m.body_rootid[b]]
        tq, f = d.cfrc_ext[b, :3], d.cfrc_ext[b, 3:]
        out.append(floats(np.concatenate([tq - np.cross(d.xpos[b] - c, f), f])))
    return out


def inverse_at(m, d, qacc):
    d.qacc[:] = qacc
    mujoco.mj_inverse(m, d)
    out = {"qacc": floats(d.qacc), "qfrc_inverse": floats(d.qfrc_inverse),
           "efc_force": floats(d.efc_force), "qfrc_constraint": floats(d.qfrc_constraint),
           "efc_state": ints(d.efc_state)}
    # The body accumulators from the inverse's force.
    mujoco.mj_rnePostConstraint(m, d)
    out["cfrc_ext"] = cfrc_ext_at_origin(m, d)
    return out


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


FLOSS = LIMIT.replace('damping="0.2"/>', 'damping="0.2" frictionloss="0.3"/>', 1).replace(
    '<joint name="h2" type="hinge" axis="0 1 0"/>',
    '<joint name="h2" type="hinge" axis="0 1 0" frictionloss="0.05"/>', 1)
assert FLOSS.count("frictionloss") == 2, FLOSS


def fwdinv(xml, qpos, qvel, xfrc=None, opt=""):
    """`xml` with the fwdinv flag enabled (and `opt` added to its option),
    at its state, with `xfrc` (body, wrench) applied: after mj_forward,
    solver_fwdinv (and, not written, efc_state); after mj_step,
    solver_fwdinv and efc_state."""
    assert xml.count("<option ") == 1, xml
    start = xml.index("<option ")
    end = xml.index("/>", start)
    on = xml[:end] + opt + '>\n    <flag fwdinv="enable"/>\n  </option>' + xml[end + 2:]
    m = mujoco.MjModel.from_xml_string(on)
    reached = state(m, qpos, qvel)
    before = {"qpos": floats(reached.qpos), "qvel": floats(reached.qvel),
              "qacc_warmstart": floats(reached.qacc_warmstart), "time": float(reached.time)}
    if xfrc is not None:
        before["xfrc_applied"] = [xfrc[0], list(xfrc[1])]

    def at_state():
        # A fresh MjData at that state, as the test makes one.
        d = mujoco.MjData(m)
        d.qpos[:] = before["qpos"]
        d.qvel[:] = before["qvel"]
        d.qacc_warmstart[:] = before["qacc_warmstart"]
        d.time = before["time"]
        if xfrc is not None:
            d.xfrc_applied[xfrc[0]] = xfrc[1]
        return d

    d = at_state()
    mujoco.mj_forward(m, d)
    forward = {"solver_fwdinv": floats(d.solver_fwdinv)}
    forward_states = ints(d.efc_state)
    d = at_state()
    mujoco.mj_step(m, d)
    step = {"solver_fwdinv": floats(d.solver_fwdinv), "efc_state": ints(d.efc_state)}
    return {"xml": on, **before, "after_forward": forward, "after_step": step}, forward_states


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {name: case(*args) for name, args in CASES.items()}
    limit, contact = CASES["limit"], CASES["contact"]
    pgs = ' solver="PGS" iterations="4"'
    forward_states = {}
    for name, args, kwargs in [
        ("limit_fwdinv", (limit[0], limit[1], limit[2]), {}),
        ("contact_fwdinv", (contact[0], None, None, (1, [0.3, -0.2, 0.5, 0.01, 0.02, -0.03])), {}),
        ("free_fwdinv", (FREE, [0.4, -0.7, 1.1], [0.3, -0.2, 0.5]), {}),
        ("pgs_fwdinv", (contact[0], None, None), {"opt": pgs}),
        ("floss_fwdinv", (FLOSS, limit[1], limit[2]), {}),
        ("noslip_fwdinv", (CASES["elliptic"][0], None, None), {"opt": ' noslip_iterations="5"'}),
        ("rk4_fwdinv", (contact[0], None, None), {"opt": pgs + ' integrator="RK4"'}),
    ]:
        cases[name], forward_states[name] = fwdinv(*args, **kwargs)
    for name in ("pgs_fwdinv", "rk4_fwdinv"):
        if not cases[name]["after_step"]["solver_fwdinv"][0] > 1e-6:
            sys.exit(f"{name}: solver_fwdinv[0] {cases[name]['after_step']['solver_fwdinv']}")
    for name in ("floss_fwdinv", "noslip_fwdinv"):
        if cases[name]["after_step"]["efc_state"] == forward_states[name]:
            sys.exit(f"{name}: the inverse's states are the forward's {forward_states[name]}")
    out = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "inverse.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
