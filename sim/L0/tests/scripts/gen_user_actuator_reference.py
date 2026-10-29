#!/usr/bin/env python3
"""Generate the user actuator golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_user_actuator_reference.py \\
        sim/L0/tests/assets/golden/user_actuator

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes user_actuator.json, one entry per case: two damped hinges under
implicitfast (one case under implicit) and one actuator on the second whose
gain or bias is a user's, its own parameters written (MuJoCo's reads none of
them), or whose dynamics are a user's with one activation or two (`actdim`,
with and without `actearly` and an activation range), with MuJoCo's callback
set (`mjcb_act_gain`, `mjcb_act_bias`, `mjcb_act_dyn`; with two activations
the dynamics callback writes the last one's act_dot, or both) or unset; and for each
of 12 states (qpos, qvel, ctrl and act drawn from a
seeded generator, and act_dot from a second one, set before each pass:
MuJoCo zeroes an actuator's act_dot before its dynamics write it) after
mj_forward the actuator's force, qfrc_actuator and act_dot, and after one
mj_step qDeriv on D's pattern (D_rownnz, D_rowadr, D_colind), qpos, qvel
and act.

sim-mjcf refuses `actdim` (and refused a user gain, bias or dynamics when
this golden was made), so each case also holds a stand-in (`xml`): the same
model with a type sim-mjcf loads in place of the
user's, which the test changes to the user's (`patch`) and checks against
MuJoCo's compiled types (`gaintype`, `biastype`, `dyntype`, `actnum`). The
test's callbacks are GAIN, BIAS and DYN below, in the same operations; the
forces compared pin GAIN's and BIAS's values at every state, act_dot
DYN's. The models' joint axes are
unit, their bodies' inertias written out and their geoms boxes, so what
sim-mjcf compiles is MuJoCo's bit for bit (ledger L198 holds where it
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
  <option timestep="0.002" integrator="{integrator}"/>
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

# A user gain's and a user bias's own parameters, which MuJoCo's reads not.
USER_GAIN = 'gainprm="2.5 0.4 -0.6"'
USER_BIAS = 'biasprm="0.3 -0.2 0.45"'

# (name, MuJoCo's actuator, sim-mjcf's stand-in, the patch[, the integrator])
CASES = [
    ('a user gain with its callback, an affine bias',
     f'<general joint="h2" gaintype="user" {USER_GAIN} biastype="affine" biasprm="0.1 -1 -0.3"/>',
     f'<general joint="h2" gaintype="fixed" {USER_GAIN} biastype="affine" biasprm="0.1 -1 -0.3"/>',
     {'gain': 'callback'}),
    ('a user gain with its callback, an affine bias, under implicit',
     f'<general joint="h2" gaintype="user" {USER_GAIN} biastype="affine" biasprm="0.1 -1 -0.3"/>',
     f'<general joint="h2" gaintype="fixed" {USER_GAIN} biastype="affine" biasprm="0.1 -1 -0.3"/>',
     {'gain': 'callback'}, 'implicit'),
    ('an affine gain, a user bias with its callback',
     f'<general joint="h2" gaintype="affine" gainprm="2 0 0.5" biastype="user" {USER_BIAS}/>',
     f'<general joint="h2" gaintype="affine" gainprm="2 0 0.5" biastype="none" {USER_BIAS}/>',
     {'bias': 'callback'}),
    ('a user gain without a callback, an affine bias',
     f'<general joint="h2" gaintype="user" {USER_GAIN} biastype="affine" biasprm="0.1 -1 -0.3"/>',
     f'<general joint="h2" gaintype="fixed" {USER_GAIN} biastype="affine" biasprm="0.1 -1 -0.3"/>',
     {'gain': 'unset'}),
    ('an affine gain, a user bias without a callback',
     f'<general joint="h2" gaintype="affine" gainprm="2 0 0.5" biastype="user" {USER_BIAS}/>',
     f'<general joint="h2" gaintype="affine" gainprm="2 0 0.5" biastype="none" {USER_BIAS}/>',
     {'bias': 'unset'}),
    ('user dynamics with two activations, without a callback',
     '<general joint="h2" dyntype="user" actdim="2" gaintype="affine" gainprm="2 0 0.5" '
     'biastype="affine" biasprm="0 -1 -0.3"/>',
     '<general joint="h2" dyntype="integrator" gaintype="affine" gainprm="2 0 0.5" '
     'biastype="affine" biasprm="0 -1 -0.3"/>',
     {'dyn': 'unset', 'actnum': 2}),
    ('user dynamics with one activation and its callback',
     '<general joint="h2" dyntype="user" gaintype="affine" gainprm="2 0 0.5" '
     'biastype="affine" biasprm="0 -1 -0.3"/>',
     '<general joint="h2" dyntype="integrator" gaintype="affine" gainprm="2 0 0.5" '
     'biastype="affine" biasprm="0 -1 -0.3"/>',
     {'dyn': 'callback'}),
    ('user dynamics with two activations, actearly and its callback',
     '<general joint="h2" dyntype="user" actdim="2" actearly="true" gaintype="affine" '
     'gainprm="2 0 0.5" biastype="affine" biasprm="0 -1 -0.3"/>',
     '<general joint="h2" dyntype="integrator" actearly="true" gaintype="affine" '
     'gainprm="2 0 0.5" biastype="affine" biasprm="0 -1 -0.3"/>',
     {'dyn': 'callback', 'actnum': 2}),
    ('user dynamics with two activations, actearly, an activation range and its callback',
     '<general joint="h2" dyntype="user" actdim="2" actearly="true" actlimited="true" '
     'actrange="0.1 0.5" gaintype="affine" gainprm="2 0 0.5" biastype="affine" '
     'biasprm="0 -1 -0.3"/>',
     '<general joint="h2" dyntype="integrator" actearly="true" actlimited="true" '
     'actrange="0.1 0.5" gaintype="affine" gainprm="2 0 0.5" biastype="affine" '
     'biasprm="0 -1 -0.3"/>',
     {'dyn': 'callback', 'actnum': 2}),
    ('user dynamics with two activations, actearly and a callback writing both',
     '<general joint="h2" dyntype="user" actdim="2" actearly="true" gaintype="affine" '
     'gainprm="2 0 0.5" biastype="affine" biasprm="0 -1 -0.3"/>',
     '<general joint="h2" dyntype="integrator" actearly="true" gaintype="affine" '
     'gainprm="2 0 0.5" biastype="affine" biasprm="0 -1 -0.3"/>',
     {'dyn': 'callback_both', 'actnum': 2}),
]


def GAIN(m, d, i):
    return 1.7 + 0.2 * d.actuator_length[i] - 0.3 * d.actuator_velocity[i]


def BIAS(m, d, i):
    return -0.4 * d.actuator_length[i] + 0.25 * d.actuator_velocity[i]


def DYN(m, d, i):
    # The last activation's; its act_dot is 0 here, MuJoCo zeroing it first.
    a = m.actuator_actadr[i] + m.actuator_actnum[i] - 1
    act_dot = (d.ctrl[i] - d.act[a]) / 0.07 + d.act_dot[a]
    if m.actuator_actnum[i] == 1:
        return act_dot
    # With several activations MuJoCo's callback writes act_dot itself
    # (engine_forward.c:361-370).
    d.act_dot[a] = act_dot
    return 0.0


def DYN_BOTH(m, d, i):
    # Writes both activations' act_dot; MuJoCo ignores the return with two.
    first = m.actuator_actadr[i]
    a = first + m.actuator_actnum[i] - 1
    d.act_dot[first] = 0.5 * d.ctrl[i] - d.act[first]
    d.act_dot[a] = (d.ctrl[i] - d.act[a]) / 0.07
    return 9.0


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def set_callbacks(patch):
    mujoco.set_mjcb_act_gain(GAIN if patch.get('gain') == 'callback' else None)
    mujoco.set_mjcb_act_bias(BIAS if patch.get('bias') == 'callback' else None)
    mujoco.set_mjcb_act_dyn({'callback': DYN, 'callback_both': DYN_BOTH}.get(patch.get('dyn')))


def case(name, actuator, stand_in, patch, integrator='implicitfast'):
    # MuJoCo's compiler runs the model once; the callbacks are set after it.
    set_callbacks({})
    text = MODEL.format(integrator=integrator, actuator=actuator)
    m = mujoco.MjModel.from_xml_string(text)
    d = mujoco.MjData(m)
    set_callbacks(patch)
    rng = np.random.default_rng(7)
    rng_act_dot = np.random.default_rng(8)
    states = []
    for _ in range(STATES):
        qpos = m.qpos0 + rng.uniform(-0.8, 0.8, m.nq)
        qvel = rng.uniform(-2.0, 2.0, m.nv)
        ctrl = rng.uniform(0.0, 1.0, m.nu)
        act = rng.uniform(0.0, 1.0, m.na)
        act_dot = rng_act_dot.uniform(-1.0, 1.0, m.na)

        def start():
            mujoco.mj_resetData(m, d)
            d.qpos[:], d.qvel[:], d.ctrl[:], d.act[:] = qpos, qvel, ctrl, act
            d.act_dot[:] = act_dot

        start()
        mujoco.mj_forward(m, d)
        state = {'qpos': floats(qpos), 'qvel': floats(qvel), 'ctrl': floats(ctrl),
                 'act': floats(act), 'act_dot_before': floats(act_dot),
                 'actuator_force': floats(d.actuator_force),
                 'qfrc_actuator': floats(d.qfrc_actuator), 'act_dot': floats(d.act_dot)}
        start()
        mujoco.mj_step(m, d)
        state.update({'qDeriv': floats(d.qDeriv), 'qpos_after': floats(d.qpos),
                      'qvel_after': floats(d.qvel), 'act_after': floats(d.act)})
        states.append(state)
    return {'name': name, 'mujoco_xml': text,
            'xml': MODEL.format(integrator=integrator, actuator=stand_in), 'patch': patch,
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
