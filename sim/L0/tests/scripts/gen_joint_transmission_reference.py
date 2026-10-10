#!/usr/bin/env python3
"""Generate the ball and free joint transmission golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_joint_transmission_reference.py \\
        sim/L0/tests/assets/golden/joint_transmission

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes joint_transmission.json, one entry per case: an actuator with a
`joint` or `jointinparent` transmission on a ball joint or a free joint
(a box in zero gravity), its kind (motor, position, velocity), its gear, the
joint's position (one quaternion 1.5 times a unit one) and velocity; the
model's actuator_acc0; then after mj_forward the actuator's length,
velocity, moment (dense, one row) and force, qfrc_actuator, qacc and the
actuatorpos and actuatorvel sensors; and qpos and qvel after each of 10
steps under Euler and under implicitfast. Its `forward_only` cases hold the
forward quantities alone.
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

KINDS = {
    'motor': '<motor name="a" {trn} gear="{gear}"/>',
    'position': '<position name="a" {trn} gear="{gear}" kp="20"/>',
    'velocity': '<velocity name="a" {trn} gear="{gear}" kv="3"/>',
}


def quat(angle, axis):
    n = math.sqrt(sum(a * a for a in axis))
    s = math.sin(angle / 2.0)
    return [math.cos(angle / 2.0)] + [s * a / n for a in axis]


# (name, joint, transmission, gear, quaternion, angular velocity[, linear velocity])
CASES = [
    ('ball joint scalar', 'ball', 'joint', '1.5', quat(0.3, [1, 1, 0]), [0.7, -0.4, 0.3]),
    ('ball jointinparent scalar', 'ball', 'jointinparent', '1.5', quat(0.3, [1, 1, 0]), [0.7, -0.4, 0.3]),
    ('ball joint 3d', 'ball', 'joint', '1.5 0.2 -0.3', quat(0.3, [1, 1, 0]), [0.7, -0.4, 0.3]),
    ('ball jointinparent 3d', 'ball', 'jointinparent', '1.5 0.2 -0.3', quat(0.3, [1, 1, 0]),
     [0.7, -0.4, 0.3]),
    ('ball joint past pi', 'ball', 'joint', '1.5 0.2 -0.3', quat(4.0, [1, -2, 0.5]), [0.7, -0.4, 0.3]),
    ('ball jointinparent past pi', 'ball', 'jointinparent', '1.5 0.2 -0.3', quat(4.0, [1, -2, 0.5]),
     [0.7, -0.4, 0.3]),
    # MuJoCo's length, the rotation vector along the rotated gear, rounds
    # otherwise than along the gear itself here (equal before rounding)
    ('ball jointinparent rounding', 'ball', 'jointinparent', '-0.27 1.05 -1.99',
     quat(2.3872977182929884, [-0.8122808264515302, -0.9433050469559874, 0.6715302078397394]),
     [0.7, -0.4, 0.3]),
    ('ball jointinparent off unit norm', 'ball', 'jointinparent', '1.5 0.2 -0.3',
     [1.5 * x for x in quat(0.3, [1, 1, 0])], [0.7, -0.4, 0.3]),
    ('free joint 6d', 'free', 'joint', '1.5 0.2 -0.3 0.4 -0.5 0.6', quat(0.3, [1, 1, 0]),
     [0.7, -0.4, 0.3]),
    ('free jointinparent 6d', 'free', 'jointinparent', '1.5 0.2 -0.3 0.4 -0.5 0.6', quat(0.3, [1, 1, 0]),
     [0.7, -0.4, 0.3]),
]

# After mj_forward only: the moment row times qvel gives products 1e16, 1,
# -1e16, 1, which sum to 2 in mju_dotSparse's order and to 1 left to right.
FORWARD_ONLY = [
    ('free joint summation order', 'free', 'joint', '1e8 1 -1e8 1 0 0', quat(0.3, [1, 1, 0]),
     [1.0, 0.0, 0.0], [1e8, 1.0, 1e8]),
]


def xml(joint, trn, kind, gear, integrator):
    jnt = '<freejoint name="j"/>' if joint == 'free' else '<joint name="j" type="ball"/>'
    act = KINDS[kind].format(trn=f'{trn}="j"', gear=gear)
    return f"""<mujoco>
  <option timestep="0.002" gravity="0 0 0" integrator="{integrator}"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      {jnt}
      <geom type="box" size="0.1 0.2 0.3" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    {act}
  </actuator>
  <sensor>
    <actuatorpos actuator="a"/>
    <actuatorvel actuator="a"/>
  </sensor>
</mujoco>"""


def state(m, d, joint, q, w, v=(0.3, 0.1, -0.2)):
    mujoco.mj_resetData(m, d)
    if joint == 'free':
        d.qpos[0:3] = [0.1, -0.2, 1.0]
        d.qpos[3:7] = q
        d.qvel[0:3] = v
        d.qvel[3:6] = w
    else:
        d.qpos[0:4] = q
        d.qvel[0:3] = w
    d.ctrl[0] = 0.3


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def case(name, joint, trn, gear, q, w, kind, v=(0.3, 0.1, -0.2), steps=True):
    out = {'name': name, 'joint': joint, 'trn': trn, 'kind': kind, 'gear': gear}
    for integrator in ('Euler', 'implicitfast') if steps else ('Euler',):
        text = xml(joint, trn, kind, gear, integrator)
        m = mujoco.MjModel.from_xml_string(text)
        d = mujoco.MjData(m)
        state(m, d, joint, q, w, v)
        if integrator == 'Euler':
            out['xml'] = text
            out['qpos'] = floats(d.qpos)
            out['qvel'] = floats(d.qvel)
            out['ctrl'] = floats(d.ctrl)
            mujoco.mj_forward(m, d)
            moment = np.zeros((m.nu, m.nv))
            mujoco.mju_sparse2dense(moment, d.actuator_moment, d.moment_rownnz,
                                    d.moment_rowadr, d.moment_colind)
            out['forward'] = {
                'actuator_acc0': floats(m.actuator_acc0),
                'actuator_length': floats(d.actuator_length),
                'actuator_velocity': floats(d.actuator_velocity),
                'actuator_moment': floats(moment),
                'actuator_force': floats(d.actuator_force),
                'qfrc_actuator': floats(d.qfrc_actuator),
                'qacc': floats(d.qacc),
                'sensordata': floats(d.sensordata),
            }
            state(m, d, joint, q, w, v)
        if steps:
            out[integrator] = []
            for _ in range(10):
                mujoco.mj_step(m, d)
                out[integrator].append({'qpos': floats(d.qpos), 'qvel': floats(d.qvel)})
    return out


def write(path, doc):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, separators=(',', ':'))
        f.write('\n')


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = [case(*c, kind) for c in CASES for kind in KINDS]
    forward_only = [case(*c[:6], kind, c[6], steps=False) for c in FORWARD_ONLY for kind in KINDS]
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], 'joint_transmission.json'),
          {'oracle': marker, 'cases': finite_or_string(cases),
           'forward_only': finite_or_string(forward_only)})
    print(f'{len(cases)} cases, {len(forward_only)} forward-only -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
