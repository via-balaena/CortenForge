#!/usr/bin/env python3
"""Generate the actuator pass golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_actuator_pass_reference.py \\
        sim/L0/tests/assets/golden/actuator_pass

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes actuator_pass.json, one entry per case: two damped hinges under
implicitfast with two or three actuators, each case showing what MuJoCo's
mj_fwdActuation does in its passes (engine_forward.c:277-570): every
actuator's act_dot, then the plugins', then the forces, then the plugins',
then the clamp and the mapping over every actuator. A callback reads another
actuator's act_dot or force, or its own force before its bias; a dynamics
callback writes an earlier actuator's act_dot; a `mujoco.pid` plugin's
actuator, with an integral term, a filter, a force range, and MuJoCo's
dynamics callback called for it; a force range that excludes 0 on a disabled
actuator; two actuators sharing an activation; an infinite gear on actuators
with no force. For each of 12 states (qpos, qvel, ctrl and act drawn from a
seeded generator, and act_dot from a second one, set before each pass) after
mj_forward each actuator's force, qfrc_actuator and act_dot, and after one
mj_step qDeriv on D's pattern (D_rownnz, D_rowadr, D_colind), qpos, qvel and
act.

sim-mjcf refuses a user gain, bias or dynamics and loads no plugin, so each
case also holds a stand-in (`xml`): the same model with types sim-mjcf loads.
The test changes it to MuJoCo's compiled actuators (`compiled`: types,
activation counts and addresses, plugins, actearly, force ranges, gears),
including what the case sets after compiling (`after_compile`), and checks the
result against them. The test's callbacks are GAIN, BIAS and DYN below, in the
same operations, and its plugin is a port of `mujoco.pid` (`pid.cc`) built
from `pid`. The models' joint axes are unit, their bodies' inertias written
out and their geoms boxes, so what sim-mjcf compiles is MuJoCo's bit for bit
(ledgers L146, L198 hold where it is not).
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
  <option timestep="0.002" integrator="implicitfast"{option}/>
{extension}  <worldbody>
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

PID_KD = {'kp': 3.0, 'kd': 0.2}
PID_KI = {'kp': 3.0, 'ki': 0.5, 'kd': 0.2}


def extension(pid):
    if not pid:
        return ''
    keys = ''.join(f'<config key="{k}" value="{v}"/>' for k, v in pid.items())
    return (f'  <extension><plugin plugin="mujoco.pid"><instance name="pid">{keys}'
            '</instance></plugin></extension>\n')


# (name, MuJoCo's actuators, sim-mjcf's stand-in, the case's options)
CASES = [
    ('a user gain reading a later actuator act_dot',
     '<general joint="h1" gaintype="user"/>\n    <general joint="h2" dyntype="integrator"/>',
     '<general joint="h1"/>\n    <general joint="h2" dyntype="integrator"/>',
     {'gain': 'later_act_dot'}),
    ('a user gain reading an earlier actuator force, which its range clamps after',
     '<motor joint="h1" forcelimited="true" forcerange="-0.1 0.1"/>\n'
     '    <general joint="h2" gaintype="user" biastype="affine" biasprm="0 -1 -0.3"/>',
     '<motor joint="h1" forcelimited="true" forcerange="-0.1 0.1"/>\n'
     '    <general joint="h2" biastype="affine" biasprm="0 -1 -0.3"/>',
     {'gain': 'earlier_force'}),
    ('a user bias reading its own actuator force',
     '<general joint="h1" gaintype="affine" gainprm="2 0 0.5" biastype="user"/>\n'
     '    <general joint="h2" dyntype="filter" dynprm="0.05" biastype="affine" biasprm="0 -1 -0.3"/>',
     '<general joint="h1" gaintype="affine" gainprm="2 0 0.5"/>\n'
     '    <general joint="h2" dyntype="filter" dynprm="0.05" biastype="affine" biasprm="0 -1 -0.3"/>',
     {'bias': 'own_force'}),
    ('a user dynamics callback reading an earlier actuator force',
     '<general joint="h1" gainprm="3"/>\n    <general joint="h2" dyntype="user"/>',
     '<general joint="h1" gainprm="3"/>\n    <general joint="h2" dyntype="integrator"/>',
     {'dyn': 'earlier_force'}),
    ('a user dynamics callback writing an earlier actuator act_dot, which acts early',
     '<general joint="h1" dyntype="integrator" actearly="true"/>\n    <general joint="h2" dyntype="user"/>',
     '<general joint="h1" dyntype="integrator" actearly="true"/>\n    <general joint="h2" dyntype="integrator"/>',
     {'dyn': 'write_earlier'}),
    ('a PID plugin actuator beside a motor',
     '<motor joint="h1"/>\n    <plugin joint="h2" plugin="mujoco.pid" instance="pid"/>',
     '<motor joint="h1"/>\n    <general joint="h2"/>',
     {'pid': PID_KD}),
    ('a PID with an integral term and a filter, acting early',
     '<general joint="h1" dyntype="integrator"/>\n'
     '    <plugin joint="h2" plugin="mujoco.pid" instance="pid" dyntype="filter" dynprm="0.05" '
     'actdim="2" actearly="true"/>',
     '<general joint="h1" dyntype="integrator"/>\n'
     '    <general joint="h2" dyntype="filter" dynprm="0.05" actearly="true"/>',
     {'pid': PID_KI}),
    ('a PID with an integral term and no dynamics, and a dynamics callback',
     '<general joint="h1" dyntype="integrator"/>\n'
     '    <plugin joint="h2" plugin="mujoco.pid" instance="pid" actdim="1"/>',
     '<general joint="h1" dyntype="integrator"/>\n    <general joint="h2"/>',
     {'pid': PID_KI, 'dyn': 'write_earlier'}),
    ('a PID with a force range',
     '<motor joint="h1"/>\n'
     '    <plugin joint="h2" plugin="mujoco.pid" instance="pid" forcelimited="true" forcerange="-0.1 0.1"/>',
     '<motor joint="h1"/>\n    <general joint="h2" forcelimited="true" forcerange="-0.1 0.1"/>',
     {'pid': PID_KD}),
    ('a force range excluding 0 on a disabled group',
     '<motor joint="h1" group="1" forcelimited="true" forcerange="1 2"/>\n'
     '    <motor joint="h2" forcelimited="true" forcerange="-2 -1"/>',
     '<motor joint="h1" group="1" forcelimited="true" forcerange="1 2"/>\n'
     '    <motor joint="h2" forcelimited="true" forcerange="-2 -1"/>',
     {'option': ' actuatorgroupdisable="1"'}),
    ('two filters sharing an activation',
     '<general joint="h1" dyntype="filter" dynprm="0.5"/>\n'
     '    <general joint="h2" dyntype="filter" dynprm="0.25"/>',
     '<general joint="h1" dyntype="filter" dynprm="0.5"/>\n'
     '    <general joint="h2" dyntype="filter" dynprm="0.25"/>',
     {'actadr': [[1, 0]]}),
    ('an infinite gear on actuators with no force',
     '<motor joint="h1"/>\n    <general joint="h2" gainprm="0"/>\n    <motor joint="h2" group="1"/>',
     '<motor joint="h1"/>\n    <general joint="h2" gainprm="0"/>\n    <motor joint="h2" group="1"/>',
     {'option': ' actuatorgroupdisable="1"', 'gear': [[1, float('inf')], [2, float('inf')]]}),
]


def GAIN_LATER_ACT_DOT(m, d, i):
    return 1.0 + d.act_dot[0]


def GAIN_EARLIER_FORCE(m, d, i):
    return 1.0 + d.actuator_force[0]


def BIAS_OWN_FORCE(m, d, i):
    return 0.5 * d.actuator_force[i]


def DYN_EARLIER_FORCE(m, d, i):
    a = m.actuator_actadr[i] + m.actuator_actnum[i] - 1
    return d.actuator_force[0] + (d.ctrl[i] - d.act[a]) / 0.07


def DYN_WRITE_EARLIER(m, d, i):
    a = m.actuator_actadr[i] + m.actuator_actnum[i] - 1
    d.act_dot[0] += 0.5 * d.ctrl[i]
    return (d.ctrl[i] - d.act[a]) / 0.07


GAINS = {'later_act_dot': GAIN_LATER_ACT_DOT, 'earlier_force': GAIN_EARLIER_FORCE}
BIASES = {'own_force': BIAS_OWN_FORCE}
DYNS = {'earlier_force': DYN_EARLIER_FORCE, 'write_earlier': DYN_WRITE_EARLIER}


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def set_callbacks(opts):
    mujoco.set_mjcb_act_gain(GAINS.get(opts.get('gain')))
    mujoco.set_mjcb_act_bias(BIASES.get(opts.get('bias')))
    mujoco.set_mjcb_act_dyn(DYNS.get(opts.get('dyn')))


def case(name, actuator, stand_in, opts):
    # MuJoCo's compiler runs the model once; the callbacks are set after it.
    set_callbacks({})
    fill = {'option': opts.get('option', ''), 'extension': extension(opts.get('pid'))}
    text = MODEL.format(actuator=actuator, **fill)
    m = mujoco.MjModel.from_xml_string(text)
    for a, adr in opts.get('actadr', []):
        m.actuator_actadr[a] = adr
    for a, gear in opts.get('gear', []):
        m.actuator_gear[a, 0] = gear
    d = mujoco.MjData(m)
    set_callbacks(opts)
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
    compiled = {
        'dyntype': [mujoco.mjtDyn(t).name for t in m.actuator_dyntype],
        'gaintype': [mujoco.mjtGain(t).name for t in m.actuator_gaintype],
        'biastype': [mujoco.mjtBias(t).name for t in m.actuator_biastype],
        'actnum': ints(m.actuator_actnum), 'actadr': ints(m.actuator_actadr), 'na': int(m.na),
        'plugin': ints(m.actuator_plugin), 'actearly': ints(m.actuator_actearly),
        'forcelimited': ints(m.actuator_forcelimited), 'forcerange': floats(m.actuator_forcerange),
        'gear0': floats(m.actuator_gear[:, 0]),
    }
    return {'name': name, 'mujoco_xml': text, 'xml': MODEL.format(actuator=stand_in, **fill),
            'callbacks': {k: opts[k] for k in ('gain', 'bias', 'dyn') if k in opts},
            'pid': opts.get('pid'),
            'after_compile': {k: opts[k] for k in ('actadr', 'gear') if k in opts},
            'compiled': compiled,
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
    write(os.path.join(sys.argv[1], 'actuator_pass.json'),
          {'oracle': marker, 'cases': finite_or_string(cases)})
    print(f'{len(cases)} cases -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
