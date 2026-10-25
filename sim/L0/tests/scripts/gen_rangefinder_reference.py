#!/usr/bin/env python3
"""Generate the rangefinder golden from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_rangefinder_reference.py \\
        sim/L0/tests/assets/golden/rangefinder

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes rangefinder.json, one entry per case, each a model, the states its
free body is put at (qpos), and the sensordata after mj_forward at each. A
site's rangefinder casts `mj_ray` (`engine_ray.c`) from the site along its z
axis over the geoms its body does not hold. The cases:

- random: a sphere, a capsule, an ellipsoid, a cylinder and a box on one free
  body and an unbounded plane under them, and a rangefinder on a second free
  body, at 240 states drawn from a seeded generator (hits and misses);
- invisible: a box whose rgba alpha is 0 in front of a sphere, and a box on
  the sensor's own body in front of another sphere;
- translucent: a box whose rgba alpha is 0.25 in front of a sphere (the ray
  stops at the box);
- cutoff: a hit at 2 under a cutoff of 0.5, a miss under a cutoff of 0.5,
  and a hit at 150 with no cutoff;
- mesh: a convex mesh slab below a free body turned to 80 drawn states, one
  rangefinder on it;
- hfield: a height field below the same turning body and one above it, at the
  same 80 states;
- mesh_written_frame: a box mesh below the same turning body and an octahedron
  above it, two meshes MuJoCo keeps in their written frame, at the same 80
  states.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

NSTATE = 240


def num(rng, lo, hi):
    return repr(float(rng.uniform(lo, hi)))


def vec(rng, lo, hi, n=3):
    return ' '.join(num(rng, lo, hi) for _ in range(n))


def random_model():
    """Five primitives on one free body, unturned in it, and an unbounded plane;
    one rangefinder on an unturned site of a second free body. Every turn comes
    from the states' qpos, so none goes through the compiler's orientations
    (its Euler angles and written quaternions; ledgers L255 and L264); the
    plane's size is 0 in x and y, which MuJoCo reads unbounded, as sim-core
    reads every plane until Rigid-loading's L14."""
    rng = np.random.default_rng(236)
    geoms = [
        f'<geom type="sphere" size="{num(rng, 0.1, 0.3)}" pos="{vec(rng, -1.5, 1.5)}"/>',
        f'<geom type="capsule" size="{num(rng, 0.05, 0.2)} {num(rng, 0.1, 0.4)}" pos="{vec(rng, -1.5, 1.5)}"/>',
        f'<geom type="ellipsoid" size="{vec(rng, 0.1, 0.4)}" pos="{vec(rng, -1.5, 1.5)}"/>',
        f'<geom type="cylinder" size="{num(rng, 0.1, 0.3)} {num(rng, 0.1, 0.4)}" pos="{vec(rng, -1.5, 1.5)}"/>',
        f'<geom type="box" size="{vec(rng, 0.1, 0.4)}" pos="{vec(rng, -1.5, 1.5)}"/>',
    ]
    xml = ('<mujoco><option gravity="0 0 0"/><worldbody><geom type="plane" size="0 0 0.1" pos="0 0 -2"/>'
           f'<body name="targets"><freejoint/>{"".join(geoms)}</body>'
           '<body name="eye"><freejoint/><geom type="sphere" size="0.01" contype="0" conaffinity="0"/>'
           '<site name="r"/></body></worldbody><sensor><rangefinder site="r"/></sensor></mujoco>')
    states = []
    for _ in range(NSTATE):
        qt = rng.normal(size=4)
        qe = rng.normal(size=4)
        states.append([float(x) for x in rng.uniform(-0.3, 0.3, 3)] + [float(x) for x in qt / np.linalg.norm(qt)]
                      + [float(x) for x in rng.uniform(-0.2, 0.2, 3)] + [float(x) for x in qe / np.linalg.norm(qe)])
    return xml, states


INVISIBLE = """<mujoco><option gravity="0 0 0"/><worldbody>
  <geom type="box" size="0.1 0.1 0.1" pos="0 0 1" rgba="1 0 0 0"/>
  <geom type="sphere" size="0.2" pos="0 0 2"/>
  <geom type="sphere" size="0.2" pos="1 0 2"/>
  <body name="eye"><freejoint/>
    <geom type="box" size="0.1 0.1 0.1" pos="1 0 1" contype="0" conaffinity="0"/>
    <site name="through_invisible"/>
    <site name="through_own" pos="1 0 0"/>
  </body>
</worldbody><sensor>
  <rangefinder site="through_invisible"/>
  <rangefinder site="through_own"/>
</sensor></mujoco>"""

CUTOFF = """<mujoco><option gravity="0 0 0"/><worldbody>
  <geom type="sphere" size="0.2" pos="0 0 2.2"/>
  <geom type="sphere" size="1" pos="5 0 151"/>
  <body name="eye"><freejoint/><geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
    <site name="near"/>
    <site name="miss" pos="-3 0 0"/>
    <site name="far" pos="5 0 0"/>
  </body>
</worldbody><sensor>
  <rangefinder site="near" cutoff="0.5"/>
  <rangefinder site="miss" cutoff="0.5"/>
  <rangefinder site="far"/>
</sensor></mujoco>"""

TRANSLUCENT = """<mujoco><option gravity="0 0 0"/><worldbody>
  <geom type="box" size="0.1 0.1 0.1" pos="0 0 1" rgba="0 0 1 0.25"/>
  <geom type="sphere" size="0.2" pos="0 0 2"/>
  <body name="eye"><freejoint/><geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
    <site name="r"/>
  </body>
</worldbody><sensor><rangefinder site="r"/></sensor></mujoco>"""

REST = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

# A mesh slab below a turning eye. MuJoCo turns the slab to its principal frame, which
# sim-mjcf does not (book Rigid-loading L35), and keeps its vertices in single precision,
# sim-core in double (ledger L281).
MESH = """<mujoco><option gravity="0 0 0"/>
  <asset>
    <mesh name="slab" vertex="-0.5 -0.4 -0.1  0.5 -0.4 -0.1  0.5 0.4 -0.1  -0.5 0.4 -0.1
      -0.5 -0.4 0.1  0.5 -0.4 0.1  0.5 0.4 0.1  -0.5 0.4 0.1"
      face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="slab" pos="0 0 -0.3"/>
    <body name="eye"><freejoint/><geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
      <site name="r"/>
    </body>
  </worldbody>
  <sensor><rangefinder site="r"/></sensor>
</mujoco>"""


# A height field below a turning eye and another above it, base down toward it. Their
# elevations load in sim-mjcf as MuJoCo compiles them: numbers a float holds, 0 to 1 (so
# MuJoCo's normalising leaves them), the rows the same read from either end (so its flip
# does), the cells square (so sim-mjcf does not resample them); book Rigid-loading L45
# loads any field so.
HFIELD_ROWS = ['0 0.25 0.5 0.75 1 0.625', '0.125 0.5 1 0.25 0 0.375', '0.5 0.75 0 0.125 0.25 1']
HFIELD = f"""<mujoco><option gravity="0 0 0"/>
  <asset>
    <hfield name="field" nrow="5" ncol="6" size="0.5 0.4 0.3 0.1"
      elevation="{' '.join(HFIELD_ROWS + HFIELD_ROWS[1::-1])}"/>
  </asset>
  <worldbody>
    <geom type="hfield" hfield="field" pos="0 0 -0.4"/>
    <geom type="hfield" hfield="field" pos="0.125 -0.25 0.6"/>
    <body name="eye"><freejoint/><geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
      <site name="r"/>
    </body>
  </worldbody>
  <sensor><rangefinder site="r"/></sensor>
</mujoco>"""


# A box below a turning eye and an octahedron above it, two meshes MuJoCo keeps in their
# written frame (their principal axes along x, y, z, the largest moment about x, centred, in
# numbers a float holds), so sim-mjcf loads them as MuJoCo compiles them, which it does not
# do for a mesh MuJoCo turns until book Rigid-loading L35.
MESH_BOX = """<mujoco><option gravity="0 0 0"/>
  <asset>
    <mesh name="box" vertex="-0.25 -0.75 -1  0.25 -0.75 -1  0.25 0.75 -1  -0.25 0.75 -1
      -0.25 -0.75 1  0.25 -0.75 1  0.25 0.75 1  -0.25 0.75 1"
      face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7"/>
    <mesh name="octahedron" vertex="0.5 0 0  -0.5 0 0  0 1 0  0 -1 0  0 0 1.5  0 0 -1.5"
      face="0 2 4  2 1 4  1 3 4  3 0 4  2 0 5  1 2 5  3 1 5  0 3 5"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="box" pos="0.0625 0 -1.25"/>
    <geom type="mesh" mesh="octahedron" pos="0 0.125 1.75"/>
    <body name="eye"><freejoint/><geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
      <site name="r"/>
    </body>
  </worldbody>
  <sensor><rangefinder site="r"/></sensor>
</mujoco>"""


def turning_states(n):
    rng = np.random.default_rng(267)
    states = []
    for _ in range(n):
        q = rng.normal(size=4)
        states.append([0.0, 0.0, 0.0] + [float(x) for x in q / np.linalg.norm(q)])
    return states


def case(name, xml, states):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    out = []
    for qpos in states:
        mujoco.mj_resetData(m, d)
        d.qpos[:] = qpos
        mujoco.mj_forward(m, d)
        out.append({'qpos': [float(x) for x in qpos], 'sensordata': [float(x) for x in d.sensordata]})
    return {'name': name, 'xml': xml, 'states': out}


def write(path, doc):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, separators=(',', ':'))
        f.write('\n')


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    xml, states = random_model()
    cases = [case('random', xml, states), case('invisible', INVISIBLE, [REST]),
             case('cutoff', CUTOFF, [REST]), case('mesh', MESH, turning_states(80)),
             case('translucent', TRANSLUCENT, [REST]), case('hfield', HFIELD, turning_states(80)),
             case('mesh_written_frame', MESH_BOX, turning_states(80))]
    os.makedirs(sys.argv[1], exist_ok=True)
    write(os.path.join(sys.argv[1], 'rangefinder.json'), {'oracle': marker, 'cases': finite_or_string(cases)})
    print(f'{len(cases)} cases -> {sys.argv[1]}')


if __name__ == '__main__':
    main()
