#!/usr/bin/env python3
"""Generate the smooth force golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_smooth_force_reference.py \\
        sim/L0/tests/assets/golden/smooth_force

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes smooth_force.json: a chain of three damped hinges and a damped
slide under gravity, with a velocity actuator on each, so that at a moving
state every dof's passive force, bias force, applied force and actuator
force are all nonzero; for each of 8 states (qpos, qvel, ctrl and
qfrc_applied drawn from a seeded generator) after mj_forward those four,
qfrc_smooth (`mj_fwdAcceleration`, `engine_forward.c`) and qacc_smooth. The
model's joint axes are unit and its bodies' inertias written out, so what
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
  <option timestep="0.002"/>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="h1" axis="0 1 0" damping="0.37"/>
      <inertial pos="0.15 0 0" mass="0.9" diaginertia="0.004 0.03 0.03"/>
      <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
      <body name="b2" pos="0.3 0 0">
        <joint name="h2" axis="0 0 1" damping="1.13"/>
        <inertial pos="0.15 0 0" mass="0.7" diaginertia="0.003 0.02 0.02"/>
        <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
        <body name="b3" pos="0.3 0 0">
          <joint name="h3" axis="1 0 0" damping="0.29"/>
          <joint name="sl" type="slide" axis="0 1 0" damping="2.7"/>
          <inertial pos="0 0 0" mass="0.4" diaginertia="0.0004 0.0005 0.0006"/>
          <geom type="box" size="0.05 0.04 0.03"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity joint="h1" kv="0.61"/>
    <velocity joint="h2" kv="0.71"/>
    <velocity joint="h3" kv="0.43"/>
    <velocity joint="sl" kv="1.9"/>
  </actuator>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    m = mujoco.MjModel.from_xml_string(MODEL)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(266)
    states = []
    for _ in range(STATES):
        mujoco.mj_resetData(m, d)
        d.qpos[:] = m.qpos0 + rng.uniform(-0.8, 0.8, m.nq)
        d.qvel[:] = rng.uniform(-2.0, 2.0, m.nv)
        d.ctrl[:] = rng.uniform(-1.0, 1.0, m.nu)
        d.qfrc_applied[:] = rng.uniform(-3.0, 3.0, m.nv)
        mujoco.mj_forward(m, d)
        terms = [d.qfrc_passive, d.qfrc_bias, d.qfrc_applied, d.qfrc_actuator]
        if any((t == 0).any() for t in terms):
            sys.exit('a term is zero at a drawn state')
        states.append({key: floats(getattr(d, key)) for key in
                       ('qpos', 'qvel', 'ctrl', 'qfrc_applied', 'qfrc_passive', 'qfrc_bias',
                        'qfrc_actuator', 'qfrc_smooth', 'qacc_smooth')})
    os.makedirs(sys.argv[1], exist_ok=True)
    with open(os.path.join(sys.argv[1], 'smooth_force.json'), 'w', encoding='utf-8') as f:
        json.dump({'oracle': marker, 'xml': MODEL, 'states': finite_or_string(states)}, f,
                  separators=(',', ':'))
        f.write('\n')
    print(f'{STATES} states -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
