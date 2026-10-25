#!/usr/bin/env python3
"""Generate the user actuator golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_user_actuator_reference.py \\
        sim/L0/tests/assets/golden/user_actuator

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes user_actuator.json, one entry per case: two damped hinges under
implicitfast and one actuator on the second whose gain or bias is a user's,
with MuJoCo's callback set (`mjcb_act_gain`, `mjcb_act_bias`) or unset; and
for each of 12 states (qpos, qvel, ctrl and act drawn from a seeded
generator) after mj_forward the actuator's force, qfrc_actuator and act_dot,
and after one mj_step qDeriv on D's pattern (D_rownnz, D_rowadr, D_colind),
qpos, qvel and act.

sim-mjcf refuses a user gain, bias or dynamics, so each case also holds a
stand-in (`xml`): the same model with a type sim-mjcf loads in place of the
user's, which the test changes to the user's (`patch`) and checks against
MuJoCo's compiled types (`gaintype`, `biastype`, `dyntype`, `actnum`). The
test's callbacks are GAIN, BIAS and DYN below, in the same operations; the
forces compared pin their values at every state. The models' joint axes are
unit, their bodies' inertias written out and their geoms boxes, so what
sim-mjcf compiles is MuJoCo's bit for bit (ledgers L146, L198 hold where it
is not).
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

STATES = 12

MODEL = """<mujoco>
  <option timestep="0.002" integrator="implicitfast"/>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="h1" axis="0 1 0" damping="0.37"/>
      <inertial pos="0.15 0 0" mass="0.9" diaginertia="0.004 0.03 0.03"/>
      <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
      <body name="b2" pos="0.3 0 0">
        <joint name="h2" axis="0 0 1" damping="1.13"/>
        <inertial pos="0.15 0 0" mass="0.7" diaginertia="0.003 0.02 0.02"/>
        <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    {actuator}
  </actuator>
</mujoco>"""

# (name, MuJoCo's actuator, sim-mjcf's stand-in, the patch)
CASES = [
    ('a user gain with its callback, an affine bias',
     '<general joint="h2" gaintype="user" biastype="affine" biasprm="0.1 -1 -0.3"/>',
     '<general joint="h2" gaintype="fixed" biastype="affine" biasprm="0.1 -1 -0.3"/>',
     {'gain': 'callback'}),
    ('an affine gain, a user bias with its callback',
     '<general joint="h2" gaintype="affine" gainprm="2 0 0.5" biastype="user"/>',
     '<general joint="h2" gaintype="affine" gainprm="2 0 0.5" biastype="none"/>',
     {'bias': 'callback'}),
    ('a user gain without a callback, an affine bias',
     '<general joint="h2" gaintype="user" biastype="affine" biasprm="0.1 -1 -0.3"/>',
     '<general joint="h2" gaintype="fixed" biastype="affine" biasprm="0.1 -1 -0.3"/>',
     {'gain': 'unset'}),
    ('an affine gain, a user bias without a callback',
     '<general joint="h2" gaintype="affine" gainprm="2 0 0.5" biastype="user"/>',
     '<general joint="h2" gaintype="affine" gainprm="2 0 0.5" biastype="none"/>',
     {'bias': 'unset'}),
]


def GAIN(m, d, i):
    return 1.7 + 0.2 * d.actuator_length[i] - 0.3 * d.actuator_velocity[i]


def BIAS(m, d, i):
    return -0.4 * d.actuator_length[i] + 0.25 * d.actuator_velocity[i]


def DYN(m, d, i):
    return (d.ctrl[i] - d.act[m.actuator_actadr[i]]) / 0.07


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def set_callbacks(patch):
    mujoco.set_mjcb_act_gain(GAIN if patch.get('gain') == 'callback' else None)
    mujoco.set_mjcb_act_bias(BIAS if patch.get('bias') == 'callback' else None)
    mujoco.set_mjcb_act_dyn(DYN if patch.get('dyn') == 'callback' else None)


def case(name, actuator, stand_in, patch):
    # MuJoCo's compiler runs the model once; the callbacks are set after it.
    set_callbacks({})
    m = mujoco.MjModel.from_xml_string(MODEL.format(actuator=actuator))
    d = mujoco.MjData(m)
    set_callbacks(patch)
    rng = np.random.default_rng(7)
    states = []
    for _ in range(STATES):
        qpos = m.qpos0 + rng.uniform(-0.8, 0.8, m.nq)
        qvel = rng.uniform(-2.0, 2.0, m.nv)
        ctrl = rng.uniform(0.0, 1.0, m.nu)
        act = rng.uniform(0.0, 1.0, m.na)

        def start():
            mujoco.mj_resetData(m, d)
            d.qpos[:], d.qvel[:], d.ctrl[:], d.act[:] = qpos, qvel, ctrl, act

        start()
        mujoco.mj_forward(m, d)
        state = {'qpos': floats(qpos), 'qvel': floats(qvel), 'ctrl': floats(ctrl),
                 'act': floats(act), 'actuator_force': floats(d.actuator_force),
                 'qfrc_actuator': floats(d.qfrc_actuator), 'act_dot': floats(d.act_dot)}
        start()
        mujoco.mj_step(m, d)
        state.update({'qDeriv': floats(d.qDeriv), 'qpos_after': floats(d.qpos),
                      'qvel_after': floats(d.qvel), 'act_after': floats(d.act)})
        states.append(state)
    return {'name': name, 'mujoco_xml': MODEL.format(actuator=actuator),
            'xml': MODEL.format(actuator=stand_in), 'patch': patch,
            'gaintype': mujoco.mjtGain(m.actuator_gaintype[0]).name,
            'biastype': mujoco.mjtBias(m.actuator_biastype[0]).name,
            'dyntype': mujoco.mjtDyn(m.actuator_dyntype[0]).name,
            'actnum': int(m.actuator_actnum[0]),
            'D_rownnz': ints(m.D_rownnz), 'D_rowadr': ints(m.D_rowadr),
            'D_colind': ints(m.D_colind), 'states': states}


def write(path, doc):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, separators=(',', ':'))
        f.write('\n')


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    try:
        cases = [case(*c) for c in CASES]
    finally:
        set_callbacks({})
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], 'user_actuator.json'),
          {'oracle': marker, 'cases': finite_or_string(cases)})
    print(f'{len(cases)} cases -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
