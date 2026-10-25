#!/usr/bin/env python3
"""Generate the trajectory golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_trajectory_reference.py \\
        sim/L0/tests/assets/golden/trajectory

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes trajectory.json: runs whose model sim-mjcf compiles as MuJoCo does,
so that what a run shows is the forward pass and the step. Each body's
inertial (position, orientation, mass, inertia) is written out as MuJoCo
compiles it, which leaves out the masses and inertias sim-mjcf compiles an
ulp off (ledger L198), and a mesh is written in the frame MuJoCo moves it to,
with the geom's quaternion one both normalise alike (Rigid-loading's L35).
The cases:

- limit: a hinge held past its limit, with damping, carrying a second hinge
  (the inverse golden's), from a moving start: every step of 40;
- box_rest: a free box resting on a plane, pyramidal: every 10th of 150 steps;
- box_cylinder, mesh_cylinder: a cylinder dropped from 1 mm onto a box slab
  and onto a convex mesh slab (the collision golden's rest scenes): every
  10th of 120 steps;
- bumpy_<body>_<spot>: a sphere, capsule, ellipsoid, cylinder and box dropped
  at two spots of the collision golden's bumpy height field: every 50th of
  1000 steps.

Each checkpoint holds qpos and qvel after that many steps.
"""
import json
import os
import re
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402
import gen_collision_reference as collision  # noqa: E402
import gen_inverse_reference as inverse  # noqa: E402


def text(a):
    return ' '.join(repr(float(x)) for x in np.asarray(a).ravel())


def inertia_written(xml):
    """The document with each body's inertial as MuJoCo compiles it."""
    m = mujoco.MjModel.from_xml_string(xml)
    opens = list(re.finditer(r'<body\b[^>]*?(?<!/)>', xml))
    if len(opens) != m.nbody - 1:
        sys.exit(f'{len(opens)} body tags for {m.nbody - 1} bodies')
    for b, tag in reversed(list(enumerate(opens, start=1))):
        inertial = (f'<inertial pos="{text(m.body_ipos[b])}" quat="{text(m.body_iquat[b])}" '
                    f'mass="{repr(float(m.body_mass[b]))}" diaginertia="{text(m.body_inertia[b])}"/>')
        xml = xml[:tag.end()] + inertial + xml[tag.end():]
    m2 = mujoco.MjModel.from_xml_string(xml)
    for k in ('body_ipos', 'body_iquat', 'body_mass', 'body_inertia'):
        if not (getattr(m2, k) == getattr(m, k)).all():
            sys.exit(f'{k}: MuJoCo compiles the written inertial otherwise')
    return xml


def mesh_in_mujocos_frame(xml):
    """The slab mesh's vertices as MuJoCo stores them, and its geom turned by a
    quaternion both sides normalise alike, so the geom holds the same
    vertices and frame in both."""
    m = mujoco.MjModel.from_xml_string(xml)
    g = m.geom('slab').id
    mesh = m.geom_dataid[g]
    vert = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
    xml, n = re.subn(r'vertex="[^"]*"', f'vertex="{text(vert)}"', xml, count=1)
    old = '<geom name="slab" type="mesh" mesh="slab" pos="0 0 -0.1"/>'
    if n != 1 or xml.count(old) != 1:
        sys.exit('mesh slab not found')
    xml = xml.replace(old, f'<geom name="slab" type="mesh" mesh="slab" pos="{text(m.geom_pos[g])}" '
                           'quat="0 1 0 1"/>')
    m2 = mujoco.MjModel.from_xml_string(xml)
    if not (m2.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]] == vert).all():
        sys.exit('MuJoCo moves the written vertices again')
    return xml


def run(xml, steps, every, qpos=None, qvel=None):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if qpos is not None:
        d.qpos[:] = qpos
    if qvel is not None:
        d.qvel[:] = qvel
    start = {'qpos': [float(x) for x in d.qpos], 'qvel': [float(x) for x in d.qvel]}
    checkpoints = []
    for k in range(1, steps + 1):
        mujoco.mj_step(m, d)
        if k % every == 0:
            checkpoints.append({'step': k, 'qpos': [float(x) for x in d.qpos],
                                'qvel': [float(x) for x in d.qvel]})
    return {'xml': xml, 'start': start, 'every': every, 'checkpoints': checkpoints}


def cases():
    out = {}
    limit_xml, qpos, qvel, _ = inverse.CASES['limit']
    out['limit'] = run(inertia_written(limit_xml), 40, 1, qpos, qvel)
    out['box_rest'] = run(inertia_written(inverse.box('pyramidal')), 150, 10)
    for slab in ('box', 'mesh'):
        xml = inertia_written(collision.xml_of(slab, 'cylinder'))
        if slab == 'mesh':
            xml = mesh_in_mujocos_frame(xml)
        start = [0.05, -0.03, collision.BODIES['cylinder'][1] + 0.001, 1.0, 0.0, 0.0, 0.0]
        out[f'{slab}_cylinder'] = run(xml, 120, 10, start)
    elevation = ' '.join(repr(e) for e in collision.BUMPY_ELEVATION)
    for body, (geom, euler) in collision.BUMPY_BODIES.items():
        for spot, (x, y) in collision.BUMPY_SPOTS.items():
            xml = (f'<mujoco><asset><hfield name="f" nrow="9" ncol="9" size="1 1 0.3 0.1" '
                   f'elevation="{elevation}"/></asset><worldbody><geom type="hfield" hfield="f"/>'
                   f'<body pos="{x} {y} 0.6" euler="{euler}"><freejoint/>{geom}</body>'
                   '</worldbody></mujoco>')
            out[f'bumpy_{body}_{spot}'] = run(inertia_written(xml), 1000, 50)
    return out


def write(path, doc):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, separators=(',', ':'))
        f.write('\n')


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    out = cases()
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], 'trajectory.json'), {'oracle': marker, 'cases': finite_or_string(out)})
    print(f'{len(out)} cases -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
