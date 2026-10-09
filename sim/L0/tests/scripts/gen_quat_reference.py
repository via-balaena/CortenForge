#!/usr/bin/env python3
"""Generate the quaternion-integration golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_quat_reference.py \\
        sim/L0/tests/assets/golden/quat

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes quat.json:

- `spins`: a box on a free joint and on a ball joint, in zero gravity, under
  Euler and RK4, from an angular velocity about the body's z axis, a principal
  axis of the box, so the gyroscopic force is 0 up to rounding, and a start
  quaternion turned 0.3 rad about (1, 1, 0)/sqrt(2); qpos and qvel after each
  step of `checkpoints`. Two speeds: 3.7e-9 rad/s (`slow`, an angle per step
  far below 1e-10) and 2.3 rad/s (`fast`).
- `integrate_pos`: mj_integratePos on a ball joint, for each case's qpos
  (a quaternion, unit or not), qvel and dt: the qpos it returns.
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

CHECKPOINTS = [1, 2, 10, 100, 1000]

ROOTS = {
    'free': '<freejoint/>',
    'ball': '<joint type="ball"/>',
}


def spin_xml(root, integrator):
    return f"""<mujoco>
  <option timestep="0.002" gravity="0 0 0" integrator="{integrator}"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      {ROOTS[root]}
      <geom type="box" size="0.1 0.2 0.3" mass="1"/>
    </body>
  </worldbody>
</mujoco>"""


def start_quat():
    s, c = math.sin(0.15), math.cos(0.15)
    k = s / math.sqrt(2.0)
    return [c, k, k, 0.0]


def spin_case(root, integrator, speed):
    xml = spin_xml(root, integrator)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    q = 3 if root == 'free' else 0
    v = 3 if root == 'free' else 0
    d.qpos[q:q + 4] = start_quat()
    d.qvel[v:v + 3] = [0.0, 0.0, speed]
    start = {'qpos': [float(x) for x in d.qpos], 'qvel': [float(x) for x in d.qvel]}
    out = []
    for step in range(1, CHECKPOINTS[-1] + 1):
        mujoco.mj_step(m, d)
        if step in CHECKPOINTS:
            out.append({'step': step, 'qpos': [float(x) for x in d.qpos],
                        'qvel': [float(x) for x in d.qvel]})
    return {'root': root, 'integrator': integrator, 'speed': speed, 'xml': xml,
            'start': start, 'states': out}


BALL = """<mujoco>
  <worldbody>
    <body name="b" pos="0 0 1">
      <joint type="ball"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>"""

INTEGRATE_POS = [
    # (name, quaternion, qvel, dt)
    ('below mjMINVAL about y, negative dt', [1.0, 0.0, 0.0, 0.0], [0.0, 1e-16, 0.0], -1.0),
    ('below mjMINVAL about z, negative dt', [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 2e-16], -1.0),
    ('zero velocity', start_quat(), [0.0, 0.0, 0.0], 0.01),
    ('turn', start_quat(), [0.3, -0.2, 0.5], 0.01),
    ('turn back', start_quat(), [0.3, -0.2, 0.5], -0.01),
    ('tiny angle', start_quat(), [1e-12, 2e-12, -3e-12], 0.002),
    ('norm 1 + 1e-12', [x * (1.0 + 1e-12) for x in start_quat()], [0.3, -0.2, 0.5], 0.01),
    ('norm within mjMINVAL of 1', [x * (1.0 + 4e-16) for x in start_quat()], [0.3, -0.2, 0.5], 0.01),
    ('norm 2', [2.0 * x for x in start_quat()], [0.3, -0.2, 0.5], 0.01),
    ('norm 1e-12', [1e-12 * x for x in start_quat()], [0.3, -0.2, 0.5], 0.01),
    ('zero quaternion', [0.0, 0.0, 0.0, 0.0], [0.3, -0.2, 0.5], 0.01),
]


def integrate_pos_case(name, quat, qvel, dt):
    m = mujoco.MjModel.from_xml_string(BALL)
    q = np.array(quat, dtype=float)
    mujoco.mj_integratePos(m, q, np.array(qvel, dtype=float), dt)
    return {'name': name, 'qpos': list(quat), 'qvel': list(qvel), 'dt': dt,
            'result': [float(x) for x in q]}


def write(path, doc):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, separators=(',', ':'))
        f.write('\n')


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    spins = [spin_case(root, integrator, speed)
             for speed in (3.7e-9, 2.3)
             for root in ROOTS
             for integrator in ('Euler', 'RK4')]
    cases = [integrate_pos_case(*c) for c in INTEGRATE_POS]
    doc = {'oracle': marker, 'checkpoints': CHECKPOINTS,
           'spins': finite_or_string(spins), 'integrate_pos': finite_or_string(cases)}
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], 'quat.json'), doc)
    print(f'{len(spins)} spins, {len(cases)} integrate_pos cases -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
