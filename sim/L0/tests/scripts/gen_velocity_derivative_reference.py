#!/usr/bin/env python3
"""Generate the velocity derivative golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_velocity_derivative_reference.py \\
        sim/L0/tests/assets/golden/velocity_derivative

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes velocity_derivative.json: a chain of three hinges and a slide, each
damped, with a damped fixed tendon and a damped spatial tendon over them and
velocity actuators on a hinge and on both tendons, so the dofs' entries of
qDeriv each sum an actuator's term and more than one passive term
(`mjd_smooth_vel`, `engine_derivative.c:1792-1812`). For each of 8 states
(drawn from a seeded generator) under implicit and under implicitfast, one
mj_step: qDeriv on D's pattern (D_rownnz, D_rowadr, D_colind), as the step
made it at the state, and qpos and qvel after it; and one undamped slide
whose velocity actuator's kv is 1e-31, whose qDeriv MuJoCo adds however small
it is, under implicitfast. The chain's joint axes
are unit, its bodies' inertias written out and its geoms boxes, so what
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

STATES = 8

MODEL = """<mujoco>
  <option timestep="0.002" integrator="{integrator}"/>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="h1" axis="0 1 0" damping="0.37"/>
      <inertial pos="0.15 0 0" mass="0.9" diaginertia="0.004 0.03 0.03"/>
      <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
      <site name="s1" pos="0.15 0 0.03"/>
      <body name="b2" pos="0.3 0 0">
        <joint name="h2" axis="0 0 1" damping="1.13"/>
        <inertial pos="0.15 0 0" mass="0.7" diaginertia="0.003 0.02 0.02"/>
        <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
        <site name="s2" pos="0.15 0.01 0.03"/>
        <body name="b3" pos="0.3 0 0">
          <joint name="h3" axis="1 0 0" damping="0.29"/>
          <joint name="sl" type="slide" axis="0 1 0" damping="2.7"/>
          <inertial pos="0 0 0" mass="0.4" diaginertia="0.0004 0.0005 0.0006"/>
          <geom type="box" size="0.05 0.04 0.03"/>
          <site name="tip" pos="0.05 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="tf" damping="0.83">
      <joint joint="h1" coef="0.5"/>
      <joint joint="h2" coef="-1.2"/>
      <joint joint="h3" coef="0.7"/>
    </fixed>
    <spatial name="ts" damping="1.9">
      <site site="s1"/>
      <site site="s2"/>
      <site site="tip"/>
    </spatial>
  </tendon>
  <actuator>
    <velocity joint="h2" gear="1.3" kv="0.71"/>
    <velocity tendon="tf" gear="1.37" kv="3.1"/>
    <velocity tendon="ts" gear="-2.71" kv="0.53"/>
  </actuator>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


TINY = """<mujoco>
  <option timestep="0.002" integrator="implicitfast"/>
  <worldbody>
    <body name="b" pos="0 0 1">
      <joint name="s" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>
  <actuator>
    <velocity joint="s" kv="1e-31"/>
  </actuator>
</mujoco>"""


def case(integrator, text=None, nstates=STATES):
    text = text or MODEL.format(integrator=integrator)
    m = mujoco.MjModel.from_xml_string(text)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(170)
    states = []
    for _ in range(nstates):
        mujoco.mj_resetData(m, d)
        d.qpos[:] = m.qpos0 + rng.uniform(-0.8, 0.8, m.nq)
        d.qvel[:] = rng.uniform(-2.0, 2.0, m.nv)
        d.ctrl[:] = rng.uniform(-1.0, 1.0, m.nu)
        start = {'qpos': floats(d.qpos), 'qvel': floats(d.qvel), 'ctrl': floats(d.ctrl)}
        mujoco.mj_step(m, d)
        start.update({'qDeriv': floats(d.qDeriv), 'qpos_after': floats(d.qpos),
                      'qvel_after': floats(d.qvel)})
        states.append(start)
    return {'integrator': integrator, 'xml': text,
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
    cases = [case(i) for i in ('implicit', 'implicitfast')] + [case('implicitfast', TINY, 1)]
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], 'velocity_derivative.json'),
          {'oracle': marker, 'cases': finite_or_string(cases)})
    print(f'{len(cases)} cases -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
