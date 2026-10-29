#!/usr/bin/env python3
"""Generate the actuator velocity golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_actuator_velocity_reference.py \\
        sim/L0/tests/assets/golden/actuator_velocity

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes actuator_velocity.json, one entry per case: a model whose velocity
actuators act through a fixed tendon, a spatial tendon, a site (with and
without a reference site), a slider-crank and a hinge with a negative gear,
each at the dense and the sparse Jacobian; and for each of 12 states (qpos and
qvel drawn from a seeded generator) after mj_forward each actuator's moment
(dense rows) and its stored columns (MuJoCo's moment_rownnz and
moment_colind, which set mju_dotSparse's grouping), its velocity and force,
qfrc_actuator and the actuatorvel sensors. One case disables actuation. Two
more: a free joint's joint and jointinparent actuators, gears with zero
entries (MuJoCo stores a ball's or free joint's three or six entries, zeros
kept), dense and sparse; and a tree asleep from the start whose qvel is set
after a forward pass, then a forward pass from the velocity stage
(mj_forwardSkip at mjSTAGE_POS), where MuJoCo's row for the sleeping
actuator is empty. Each case names the actuators whose moment rows sim-core
fills (`moment_rows`). The models' joint axes are unit, their bodies'
inertias written out and their geoms boxes, so what sim-mjcf compiles is
MuJoCo's bit for bit (ledgers L198 and L195 hold where it is not).
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
  <option timestep="0.002" jacobian="{jacobian}">{flags}</option>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="h1" axis="0 1 0"/>
      <inertial pos="0.15 0 0" mass="0.9" diaginertia="0.004 0.03 0.03"/>
      <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
      <site name="s1" pos="0.15 0 0.03"/>
      <body name="b2" pos="0.3 0 0">
        <joint name="h2" axis="0 0 1"/>
        <inertial pos="0.15 0 0" mass="0.7" diaginertia="0.003 0.02 0.02"/>
        <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
        <site name="s2" pos="0.15 0.01 0.03"/>
        <body name="b3" pos="0.3 0 0">
          <joint name="h3" axis="1 0 0"/>
          <joint name="sl" type="slide" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="0.4" diaginertia="0.0004 0.0005 0.0006"/>
          <geom type="box" size="0.05 0.04 0.03"/>
          <site name="tip" pos="0.05 0 0" euler="0.3 -0.2 0.5"/>
        </body>
      </body>
    </body>
    <body name="free" pos="0.5 0.5 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="2" diaginertia="0.03 0.025 0.012"/>
      <geom type="box" size="0.1 0.2 0.3"/>
      <site name="fs" pos="0.05 -0.1 0.2" euler="-0.4 0.1 0.7"/>
    </body>
    <body name="rail" pos="-0.5 0 1">
      <joint name="rh" axis="0 0 1"/>
      <inertial pos="0.1 0 0" mass="0.3" diaginertia="0.0002 0.001 0.001"/>
      <geom type="box" size="0.1 0.02 0.02" pos="0.1 0 0"/>
      <site name="crank" pos="0.2 0 0"/>
    </body>
    <site name="slider" pos="-0.2 0.3 1" euler="1.5707963267948966 0 0"/>
  </worldbody>
  <tendon>
    <fixed name="tf">
      <joint joint="h1" coef="0.5"/>
      <joint joint="h2" coef="-1.2"/>
      <joint joint="h3" coef="0.7"/>
    </fixed>
    <spatial name="ts">
      <site site="s1"/>
      <site site="s2"/>
      <site site="tip"/>
    </spatial>
  </tendon>
  <actuator>
    <velocity name="fixed" tendon="tf" gear="1.37" kv="3"/>
    <velocity name="spatial" tendon="ts" gear="-2.71" kv="2"/>
    <velocity name="site" site="fs" gear="0.3 -0.2 0.9 0.1 0.4 -0.5" kv="1.5"/>
    <velocity name="site_tip" site="tip" gear="0.93 0.1 -0.4 0.2 0.3 0.6" kv="0.7"/>
    <velocity name="refsite" site="tip" refsite="s1" gear="0.5 -0.3 0.2 0.1 -0.6 0.4" kv="1.1"/>
    <velocity name="crank" cranksite="crank" slidersite="slider" cranklength="0.35" gear="1.7" kv="0.9"/>
    <velocity name="hinge" joint="h1" gear="-1.5" kv="2.5"/>
  </actuator>
  <sensor>
    <actuatorvel actuator="fixed"/>
    <actuatorvel actuator="spatial"/>
    <actuatorvel actuator="site"/>
    <actuatorvel actuator="site_tip"/>
    <actuatorvel actuator="refsite"/>
    <actuatorvel actuator="crank"/>
    <actuatorvel actuator="hinge"/>
  </sensor>
</mujoco>"""

CASES = [
    ('dense', 'dense', ''),
    ('sparse', 'sparse', ''),
    ('actuation disabled', 'dense', '<flag actuation="disable"/>'),
]

FREE = """<mujoco>
  <option timestep="0.002" jacobian="{jacobian}"/>
  <worldbody>
    <body name="free" pos="0 0 1">
      <freejoint name="f"/>
      <inertial pos="0 0 0" mass="2" diaginertia="0.03 0.025 0.012"/>
      <geom type="box" size="0.1 0.2 0.3"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="joint" joint="f" gear="0.3 0 -0.7 0.4 0.9 -0.2" kv="1.3"/>
    <velocity name="jointinparent" jointinparent="f" gear="0.5 -0.4 0 0.2 0 0.8" kv="0.6"/>
  </actuator>
  <sensor>
    <actuatorvel actuator="joint"/>
    <actuatorvel actuator="jointinparent"/>
  </sensor>
</mujoco>"""

SLEEPING = """<mujoco>
  <option timestep="0.002"><flag sleep="enable"/></option>
  <worldbody>
    <body name="still" pos="0 0 1" sleep="init">
      <joint name="hs" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.1 0.1 0.1"/>
    </body>
    <body name="moving" pos="1 0 1">
      <joint name="hm" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="asleep" joint="hs" gear="-1.5" kv="1"/>
    <velocity name="awake" joint="hm" gear="2" kv="1"/>
  </actuator>
  <sensor>
    <actuatorvel actuator="asleep"/>
    <actuatorvel actuator="awake"/>
  </sensor>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def draw(m, rng, k):
    """qpos (unit quaternion for the free joint) and qvel; the last state holds
    the hinge's velocity at 0, so its negative gear makes a negative zero."""
    qpos = m.qpos0.copy()
    for j in range(m.njnt):
        a = m.jnt_qposadr[j]
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            qpos[a:a + 3] += rng.uniform(-0.2, 0.2, 3)
            q = rng.normal(size=4)
            qpos[a + 3:a + 7] = q / np.linalg.norm(q)
        else:
            qpos[a] += rng.uniform(-0.8, 0.8)
    qvel = rng.uniform(-2.0, 2.0, m.nv)
    if k == STATES - 1:
        qvel[0] = 0.0
    return qpos, qvel


def case(name, jacobian, flags):
    text = MODEL.format(jacobian=jacobian, flags=flags)
    m = mujoco.MjModel.from_xml_string(text)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(162)
    states = []
    for k in range(STATES):
        qpos, qvel = draw(m, rng, k)
        mujoco.mj_resetData(m, d)
        d.qpos[:] = qpos
        d.qvel[:] = qvel
        d.ctrl[:] = np.linspace(-0.6, 0.9, m.nu)
        mujoco.mj_forward(m, d)
        moment = np.zeros((m.nu, m.nv))
        mujoco.mju_sparse2dense(moment, d.actuator_moment, d.moment_rownnz,
                                d.moment_rowadr, d.moment_colind)
        colind = [ints(d.moment_colind[d.moment_rowadr[i]:d.moment_rowadr[i] + d.moment_rownnz[i]])
                  for i in range(m.nu)]
        states.append({
            'qpos': floats(qpos),
            'qvel': floats(qvel),
            'ctrl': floats(d.ctrl),
            'actuator_moment': floats(moment),
            'moment_colind': colind,
            'actuator_velocity': floats(d.actuator_velocity),
            'actuator_force': floats(d.actuator_force),
            'qfrc_actuator': floats(d.qfrc_actuator),
            'sensordata': floats(d.sensordata),
        })
    return {'name': name, 'xml': text, 'nv': int(m.nv), 'nu': int(m.nu),
            'sparse': bool(mujoco.mj_isSparse(m)), 'moment_rows': [2, 3, 4, 5], 'states': states}


def record(m, d, qpos, qvel):
    moment = np.zeros((m.nu, m.nv))
    mujoco.mju_sparse2dense(moment, d.actuator_moment, d.moment_rownnz,
                            d.moment_rowadr, d.moment_colind)
    colind = [ints(d.moment_colind[d.moment_rowadr[i]:d.moment_rowadr[i] + d.moment_rownnz[i]])
              for i in range(m.nu)]
    return {'qpos': floats(qpos), 'qvel': floats(qvel), 'ctrl': floats(d.ctrl),
            'actuator_moment': floats(moment), 'moment_colind': colind,
            'actuator_velocity': floats(d.actuator_velocity),
            'actuator_force': floats(d.actuator_force),
            'qfrc_actuator': floats(d.qfrc_actuator), 'sensordata': floats(d.sensordata)}


def free_case(name, jacobian):
    text = FREE.format(jacobian=jacobian)
    m = mujoco.MjModel.from_xml_string(text)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(163)
    states = []
    for _ in range(STATES):
        mujoco.mj_resetData(m, d)
        q = rng.normal(size=4)
        qpos = np.concatenate([rng.uniform(-0.2, 0.2, 3) + [0, 0, 1], q / np.linalg.norm(q)])
        qvel = rng.uniform(-2.0, 2.0, m.nv)
        d.qpos[:] = qpos
        d.qvel[:] = qvel
        d.ctrl[:] = [0.4, -0.7]
        mujoco.mj_forward(m, d)
        states.append(record(m, d, qpos, qvel))
    return {'name': name, 'xml': text, 'nv': int(m.nv), 'nu': int(m.nu),
            'sparse': bool(mujoco.mj_isSparse(m)), 'moment_rows': [0, 1], 'states': states}


def sleeping_case():
    """A forward pass, then qvel written into the sleeping tree and a forward
    pass from the velocity stage: MuJoCo's sleeping actuator has an empty row."""
    m = mujoco.MjModel.from_xml_string(SLEEPING)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)
    qvel = np.array([0.25, 0.25])
    d.qvel[:] = qvel
    mujoco.mj_forwardSkip(m, d, mujoco.mjtStage.mjSTAGE_POS, 0)
    if d.moment_rownnz[0] != 0 or d.actuator_velocity[0] != 0.0:
        sys.exit('sleeping: MuJoCo kept a row for the sleeping actuator')
    state = record(m, d, d.qpos.copy(), qvel)
    return {'name': 'sleeping', 'xml': SLEEPING, 'nv': int(m.nv), 'nu': int(m.nu),
            'sparse': bool(mujoco.mj_isSparse(m)), 'moment_rows': [], 'skip_pos': True, 'states': [state]}


def write(path, doc):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, separators=(',', ':'))
        f.write('\n')


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = ([case(*c) for c in CASES] + [free_case('free dense', 'dense'), free_case('free sparse', 'sparse')]
             + [sleeping_case()])
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], 'actuator_velocity.json'),
          {'oracle': marker, 'cases': finite_or_string(cases)})
    print(f'{len(cases)} cases -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
