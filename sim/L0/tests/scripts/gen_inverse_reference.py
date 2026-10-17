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
- stages: the limit case's arm with sensors at every stage (joint position,
  a site's position, joint velocity, a velocimeter, an accelerometer, a
  force and a torque) and the energy flag, after mj_forward at a state with
  no constraint row, then qpos and qvel moved to one past the limit and
  qacc set: mj_inverse runs its own position and velocity stages
  (`mj_invPosition`, `mj_invVelocity`, `engine_inverse.c:38-72`) with their
  sensors and energy, then the acceleration sensors. qfrc_inverse,
  efc_force, qfrc_constraint, sensordata, energy and nisland after it (its
  position stage makes no islands: nisland stays the forward's 0). It
  refuses the case unless the first state has no row and the second has.
- sleep: a box resting on a plane and a damped hanging arm, sleep enabled,
  3,000 steps from rest (both trees asleep), then mj_forward and mj_inverse
  three times: qacc and qfrc_bias after the forward, qfrc_applied and
  qfrc_actuator, and qfrc_inverse after each call. MuJoCo's `mj_rne` writes
  the awake dofs only (`engine_core_smooth.c:2481-2484`) and the sum adds to
  `qfrc_inverse` (`engine_inverse.c:238-241`), so a sleeping dof's value
  grows call by call. Then, from a fresh MjData at the same state (the same
  steps and mj_forward), mjd_inverseFD (forward differences, eps 1e-6):
  its centre and every point add to the sleeping dofs' qfrc_inverse in
  turn, and a qpos nudge that wakes a tree carries into the later points.
  It refuses the case unless every tree sleeps and the third call's value
  differs from the first's.
- wake: a box resting on a plane that starts asleep (sleep="init") and an
  awake sphere away from it; the sphere's qpos moved to overlap the box.
  mj_inverse there stops with an error: its position stage wakes no tree a
  contact touches (no `mj_wakeCollision`), and its constraint counting
  refuses a contact with a sleeping geom (`mj_nc`,
  `engine_core_constraint.c:1944-1956`); the message. From a fresh MjData
  at that state, mj_forward wakes the box and mj_inverse then runs:
  tree_asleep, ncon and nefc after the forward, qfrc_inverse after the
  inverse. It refuses the case unless mj_inverse stops with that error and
  mj_forward wakes the box.
- discrete: a chain of three hinges on turned axes (damped, one past its
  limit), a velocity actuator on the first and a damped fixed tendon over
  the first two, with invdiscrete enabled: under Euler (eulerdamp on and
  off, and the damper flag off, which mj_discreteAcc does not read),
  implicit and implicitfast, after mj_forward at its state,
  mj_inverse at the forward's qacc shifted: `mj_discreteAcc` turns qacc
  into the continuous-time one first (`engine_inverse.c:76-153`), and
  qacc is put back after. qfrc_inverse, efc_force and qacc after it, and
  the same without the flag. Under RK4, the error mj_inverse stops with,
  and the one mj_step stops with when fwdinv is on too; and from rest,
  with both flags, mj_step's runs until a step's forward pass makes a
  constraint row, since the comparison returns before the inverse when
  there is none (`engine_inverse.c:267-270`): qpos after each step that
  ran, and the step that stopped. It refuses the case unless the flag
  moves qfrc_inverse under each integrator but Euler with eulerdamp off,
  which it must leave bit for bit, and unless from rest a step runs
  before one stops.
- wake_gap: the same with margin 0.02 and gap 0.02 on the ball's geom and
  the ball 0.01 from the box: the contact is listed (dist <= margin) and
  excluded (dist >= margin - gap), so `mj_nc` skips it and mj_inverse runs;
  the box sleeps on. ncon and tree_asleep after it. It refuses the case
  unless the contact is listed and excluded and the box sleeps.
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


STAGES = LIMIT.replace('<mujoco model="limit">', '<mujoco model="stages">', 1).replace(
    '<option timestep="0.002"/>',
                      '<option timestep="0.002">\n    <flag energy="enable"/>\n  </option>', 1).replace(
    '<geom type="box" size="0.2 0.03 0.03" pos="0.2 0 0" mass="1"/>',
    '<geom type="box" size="0.2 0.03 0.03" pos="0.2 0 0" mass="1"/>\n      <site name="tip" pos="0.4 0 0"/>', 1).replace(
    '<geom type="sphere" size="0.05" mass="0.3"/>',
    '<geom type="sphere" size="0.05" mass="0.3"/>\n        <site name="ball" pos="0.02 0 0.01"/>', 1).replace(
    "  </worldbody>",
    """  </worldbody>
  <sensor>
    <jointpos joint="h2"/>
    <framepos objtype="site" objname="ball"/>
    <jointvel joint="h"/>
    <velocimeter site="ball"/>
    <accelerometer site="ball"/>
    <force site="tip"/>
    <torque site="tip"/>
  </sensor>""", 1)
assert STAGES.count("<sensor>") == 1 and STAGES.count("site name=") == 2 and "energy" in STAGES \
    and 'model="stages"' in STAGES, STAGES


def stages():
    """mj_forward at a state with no row, then qpos, qvel and qacc moved and
    mj_inverse: its own stages, sensors and energy at the moved state."""
    m = mujoco.MjModel.from_xml_string(STAGES)
    d = mujoco.MjData(m)
    d.qpos[:] = [0.003, 0.4]
    d.qvel[:] = [-0.3, 0.6]
    mujoco.mj_forward(m, d)
    if d.nefc != 0:
        sys.exit(f"stages: {d.nefc} rows at the first state")
    out = {"xml": STAGES, "forward_qpos": floats(d.qpos), "forward_qvel": floats(d.qvel)}
    d.qpos[:] = [0.52, -0.3]
    d.qvel[:] = [0.4, -0.2]
    d.qacc[:] = [0.7, -1.3]
    mujoco.mj_inverse(m, d)
    if d.nefc == 0:
        sys.exit("stages: no row at the moved state")
    out.update({"qpos": floats(d.qpos), "qvel": floats(d.qvel), "qacc": floats(d.qacc), "nefc": int(d.nefc),
                "qfrc_inverse": floats(d.qfrc_inverse), "efc_force": floats(d.efc_force),
                "qfrc_constraint": floats(d.qfrc_constraint), "sensordata": floats(d.sensordata),
                "energy": floats(d.energy), "nisland": int(d.nisland)})
    return out


SLEEP = """<mujoco model="sleep">
  <option timestep="0.002">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="box" pos="0 0 0.1"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="2"/></body>
    <body name="arm" pos="1 0 1"><joint name="h" type="hinge" axis="0 1 0" damping="0.3"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="0.5"/></body>
  </worldbody>
</mujoco>"""
SLEEP_STEPS = 3000


def sleep():
    """Every tree asleep after SLEEP_STEPS steps, then mj_forward and three
    mj_inverse calls."""
    m = mujoco.MjModel.from_xml_string(SLEEP)
    d = mujoco.MjData(m)
    for _ in range(SLEEP_STEPS):
        mujoco.mj_step(m, d)
    if np.any(d.tree_asleep < 0):
        sys.exit(f"sleep: a tree is awake, {d.tree_asleep}")
    mujoco.mj_forward(m, d)
    out = {"xml": SLEEP, "steps": SLEEP_STEPS, "qpos": floats(d.qpos), "qacc": floats(d.qacc),
           "qfrc_bias": floats(d.qfrc_bias), "qfrc_applied": floats(d.qfrc_applied),
           "qfrc_actuator": floats(d.qfrc_actuator), "qfrc_inverse": []}
    for _ in range(3):
        mujoco.mj_inverse(m, d)
        out["qfrc_inverse"].append(floats(d.qfrc_inverse))
    if out["qfrc_inverse"][2] == out["qfrc_inverse"][0]:
        sys.exit("sleep: qfrc_inverse does not grow")
    d = mujoco.MjData(m)
    for _ in range(SLEEP_STEPS):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    nv = m.nv
    dfdq, dfdv, dfda = (np.zeros((nv, nv)) for _ in range(3))
    mujoco.mjd_inverseFD(m, d, 1e-6, 0, dfdq, dfdv, dfda, None, None, None, None)
    out.update({"DfDq": floats(dfdq), "DfDv": floats(dfdv), "DfDa": floats(dfda)})
    return out


WAKE = """<mujoco model="wake">
  <option timestep="0.002">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="box" pos="0 0 0.1" sleep="init"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="2"/></body>
    <body name="ball" pos="1 0 0.1"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
  </worldbody>
</mujoco>"""
WAKE_BALL_X = 0.15


def wake():
    """The box asleep from the reset, the ball moved onto it: mj_inverse's
    error; then, from a fresh MjData, mj_forward and mj_inverse."""
    m = mujoco.MjModel.from_xml_string(WAKE)
    box = m.body_treeid[m.body("box").id]
    ball = m.jnt_qposadr[m.body_jntadr[m.body("ball").id]]

    def at_state():
        d = mujoco.MjData(m)
        if d.tree_asleep[box] < 0:
            sys.exit("wake: the box is awake after the reset")
        d.qpos[ball] = WAKE_BALL_X
        return d

    d = at_state()
    out = {"xml": WAKE, "qpos": floats(d.qpos), "box_geom": int(m.body_geomadr[m.body("box").id])}
    try:
        mujoco.mj_inverse(m, d)
        sys.exit("wake: mj_inverse ran")
    except mujoco.FatalError as e:
        out["inverse_error"] = str(e)
    if "involves sleeping geom" not in out["inverse_error"]:
        sys.exit(f"wake: {out['inverse_error']}")
    d = at_state()
    mujoco.mj_forward(m, d)
    if d.tree_asleep[box] >= 0:
        sys.exit("wake: mj_forward left the box asleep")
    out["after_forward"] = {"tree_asleep": ints(d.tree_asleep), "ncon": int(d.ncon), "nefc": int(d.nefc)}
    mujoco.mj_inverse(m, d)
    out["qfrc_inverse"] = floats(d.qfrc_inverse)
    return out


WAKE_GAP = WAKE.replace('<geom type="sphere" size="0.1" mass="1"/>',
                        '<geom type="sphere" size="0.1" mass="1" margin="0.02" gap="0.02"/>', 1)
assert WAKE_GAP.count('gap="0.02"') == 1, WAKE_GAP
WAKE_GAP_BALL_X = 0.21


def wake_gap():
    """The ball 0.01 from the sleeping box, inside its margin and its gap:
    mj_inverse runs and the box sleeps on."""
    m = mujoco.MjModel.from_xml_string(WAKE_GAP)
    d = mujoco.MjData(m)
    box = m.body_treeid[m.body("box").id]
    ball = m.jnt_qposadr[m.body_jntadr[m.body("ball").id]]
    d.qpos[ball] = WAKE_GAP_BALL_X
    mujoco.mj_inverse(m, d)
    geoms = {int(m.body_geomadr[m.body(n).id]) for n in ("box", "ball")}
    near = [c for c in d.contact[:d.ncon] if {int(c.geom[0]), int(c.geom[1])} == geoms]
    if len(near) != 1 or not near[0].exclude or d.tree_asleep[box] < 0:
        sys.exit(f"wake_gap: {near}, tree_asleep {d.tree_asleep}")
    return {"xml": WAKE_GAP, "qpos": floats(d.qpos), "ncon": int(d.ncon), "tree_asleep": ints(d.tree_asleep),
            "dist": float(near[0].dist)}


DISCRETE = """<mujoco model="discrete">
  <option timestep="0.01" integrator="{integrator}"{extra}/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="h1" type="hinge" axis="0 1 0" damping="0.7"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="0.5"/>
      <body pos="0.3 0 0">
        <joint name="h2" type="hinge" axis="1 0.3 0" damping="0.2" limited="true" range="-10 10"/>
        <geom type="box" size="0.05 0.03 0.02" pos="0.05 0 0" mass="0.3"/>
        <body pos="0.1 0 0">
          <joint name="h3" type="hinge" axis="0.2 0.1 1" damping="0.4"/>
          <geom type="sphere" size="0.03" pos="0.04 0.01 0" mass="0.2"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t" damping="0.9"><joint joint="h1" coef="1"/><joint joint="h2" coef="-0.6"/></fixed>
  </tendon>
  <actuator><velocity joint="h1" kv="0.4"/></actuator>
</mujoco>"""
DISCRETE_STATE = ([0.3, 0.25, -0.4], [1.1, -0.7, 0.9], [0.5, -1.2, 0.8])


def discrete():
    """The discrete chain under each integrator, mj_inverse with and without
    invdiscrete; under RK4 the errors."""
    qpos, qvel, shift = DISCRETE_STATE
    out = {}
    for name, integrator, extra in (("euler", "Euler", ""),
                                    ("euler_no_eulerdamp", "Euler", ""),
                                    ("euler_no_damper", "Euler", ""),
                                    ("implicit", "implicit", ""),
                                    ("implicitfast", "implicitfast", "")):
        xml = DISCRETE.format(integrator=integrator, extra=extra)
        if name in ("euler_no_eulerdamp", "euler_no_damper"):
            flag = name.removeprefix("euler_no_")
            xml = xml.replace('integrator="Euler"/>', f'integrator="Euler">\n    <flag {flag}="disable"/>\n  </option>', 1)
            assert flag in xml, xml
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.qpos[:] = qpos
        d.qvel[:] = qvel
        d.ctrl[:] = [0.25]
        mujoco.mj_forward(m, d)
        if d.nefc == 0:
            sys.exit(f"discrete {name}: no constraint row")
        qacc = d.qacc + np.array(shift)
        runs = {}
        for flagged in (False, True):
            if flagged:
                m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_INVDISCRETE
            d.qacc[:] = qacc
            mujoco.mj_inverse(m, d)
            if not np.array_equal(d.qacc, qacc):
                sys.exit(f"discrete {name}: qacc not put back")
            runs["flagged" if flagged else "plain"] = {
                "qfrc_inverse": floats(d.qfrc_inverse), "efc_force": floats(d.efc_force)}
        moved = runs["flagged"]["qfrc_inverse"] != runs["plain"]["qfrc_inverse"]
        if moved != (name != "euler_no_eulerdamp"):
            sys.exit(f"discrete {name}: the flag {'moved' if moved else 'left'} qfrc_inverse")
        out[name] = {"xml": xml, "qpos": qpos, "qvel": qvel, "ctrl": [0.25], "qacc": floats(qacc),
                     "nefc": int(d.nefc), **runs}
    xml = DISCRETE.format(integrator="RK4", extra="")
    m = mujoco.MjModel.from_xml_string(xml)
    m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_INVDISCRETE
    errors = {}
    for what in ("inverse", "step"):
        d = mujoco.MjData(m)
        d.qpos[:] = qpos
        d.qvel[:] = qvel
        try:
            if what == "inverse":
                mujoco.mj_forward(m, d)
                mujoco.mj_inverse(m, d)
            else:
                m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_FWDINV
                mujoco.mj_step(m, d)
            sys.exit(f"discrete rk4: {what} ran")
        except mujoco.FatalError as e:
            errors[what] = str(e)
        if "not supported by RK4" not in errors[what]:
            sys.exit(f"discrete rk4: {errors[what]}")
    m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_FWDINV
    d = mujoco.MjData(m)
    ran = []
    for k in range(100):
        try:
            mujoco.mj_step(m, d)
        except mujoco.FatalError as e:
            if "not supported by RK4" not in str(e) or d.nefc == 0:
                sys.exit(f"discrete rk4 from rest: step {k}: {e}, nefc {d.nefc}")
            break
        ran.append(floats(d.qpos))
    else:
        sys.exit("discrete rk4 from rest: no step stopped")
    if not ran:
        sys.exit("discrete rk4 from rest: the first step stopped")
    out["rk4"] = {"xml": xml, "errors": errors, "from_rest": {"qpos_after": ran, "stopped_at": len(ran)}}
    return out


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
    cases["stages"] = stages()
    cases["sleep"] = sleep()
    cases["wake"] = wake()
    cases["wake_gap"] = wake_gap()
    cases["discrete"] = discrete()
    out = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "inverse.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
