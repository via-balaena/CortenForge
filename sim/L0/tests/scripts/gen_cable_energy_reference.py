#!/usr/bin/env python3
"""Print MuJoCo 3.5.0's cable energies that `flex_unified.rs` holds as literals.

Run it with the unfused oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_cable_energy_reference.py

It refuses any other interpreter, as gen_census_golden.py does.

A three-vertex cable, its first vertex a jointless anchor body and the other
two three-slide bodies, both edges stretched (qpos 0.02, 0, 0, 0.02, 0.03, 0),
edge stiffness 50, gravity off: the potential energy after mj_forward as
loaded, and with the first edge marked rigid, the flex marked rigid and the
flex's dim set to 2 after loading (MuJoCo's `mj_energyPos` skips each,
`engine_sensor.c:1701-1718`; ledger L107). The compiler cannot produce the last
three: it marks every edge of a rigid flex rigid and refuses an edge stiffness
on a flex of dim above 1.
"""
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402

XML = """<mujoco>
  <option timestep="0.002"><flag equality="disable" gravity="disable" energy="enable"/></option>
  <worldbody>
    <body name="anchor" pos="0 0 1"/>
    <body name="v1" pos="0.1 0 1">
      <joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
    </body>
    <body name="v2" pos="0.2 0 1">
      <joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
    </body>
  </worldbody>
  <deformable>
    <flex name="c" dim="1" body="anchor v1 v2" vertex="0 0 0 0 0 0 0 0 0" element="0 1 1 2">
      <edge stiffness="50"/>
      <contact contype="0" conaffinity="0"/>
    </flex>
  </deformable>
</mujoco>"""
QPOS = [0.02, 0.0, 0.0, 0.02, 0.03, 0.0]


def energy(mutate=None):
    m = mujoco.MjModel.from_xml_string(XML)
    if mutate:
        mutate(m)
    d = mujoco.MjData(m)
    d.qpos[:] = QPOS
    mujoco.mj_forward(m, d)
    return float(d.energy[0])


def main():
    oracle()
    print("as loaded", repr(energy()))
    print("first edge rigid", repr(energy(lambda m: m.flexedge_rigid.__setitem__(0, 1))))
    print("flex rigid", repr(energy(lambda m: m.flex_rigid.__setitem__(0, 1))))
    print("dim 2", repr(energy(lambda m: m.flex_dim.__setitem__(0, 2))))


if __name__ == "__main__":
    main()
