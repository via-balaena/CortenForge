#!/usr/bin/env python3
"""Print MuJoCo 3.5.0's flex edge velocities that `flex_unified.rs` holds as literals.

Run it with the unfused oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_edge_velocity_reference.py

It refuses any other interpreter, as gen_census_golden.py does.

A free 2 x 2 grid (four three-slide bodies under `<flex body=...>`, two
triangles, five edges), each dof moving at its own rate, after mj_forward:
each edge's vertices and velocity, first as loaded (no edge equality, edge
stiffness, edge damping or flex damping, so `mj_flex` skips the edges'
Jacobians and their velocities stay 0, `engine_core_smooth.c:682-686`), then
with an edge damping, which makes the Jacobians (ledger L153).
"""
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402

BODY = """<body name="v{i}" pos="{pos}">
      <joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
    </body>"""
XML = """<mujoco>
  <option><flag gravity="disable"/></option>
  <worldbody>
    {bodies}
  </worldbody>
  <deformable>
    <flex name="grid" dim="2" body="v0 v1 v2 v3" vertex="0 0 0 0 0 0 0 0 0 0 0 0" element="0 1 2 1 3 2">
      {edge}
      <contact contype="0" conaffinity="0"/>
    </flex>
  </deformable>
</mujoco>"""
POSITIONS = ["0 0 0", "1 0 0", "0 1 0", "1 1 0"]
# qvel[i] = 0.1 * (i + 1) * (-1)^i: every dof at its own rate, signs mixed.
QVEL = [0.1 * (i + 1) * (-1) ** i for i in range(12)]


def velocities(edge):
    bodies = "\n    ".join(BODY.format(i=i, pos=p) for i, p in enumerate(POSITIONS))
    m = mujoco.MjModel.from_xml_string(XML.format(bodies=bodies, edge=edge))
    d = mujoco.MjData(m)
    d.qvel[:] = QVEL
    mujoco.mj_forward(m, d)
    return [(tuple(int(v) for v in m.flex_edge[e]), float(d.flexedge_velocity[e]))
            for e in range(m.nflexedge)]


def main():
    oracle()
    print("as loaded (Jacobians skipped)", velocities(""))
    print("edge damping 1", velocities('<edge damping="1"/>'))


if __name__ == "__main__":
    main()
