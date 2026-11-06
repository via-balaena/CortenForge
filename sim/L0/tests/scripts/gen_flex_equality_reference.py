#!/usr/bin/env python3
"""Print MuJoCo 3.5.0's trajectories that `flex_equality_rows.rs` holds as literals.

Run it with the unfused oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_equality_reference.py

It refuses any other interpreter, as gen_census_golden.py does.

A 3 x 3 cloth whose vertices are bodies, the first row of them without joints,
under gravity with contacts off: with `<equality><flex flex="g" solref="0.05 1"/>`
and with no equality. For each, the constraint rows at the start (MuJoCo makes
one per edge that is not rigid, engine_core_constraint.c:616-643, and only for a
flex an equality names) and `qpos` after 500 steps of 0.001 s.
"""
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402

XML = """<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <body name="g_v0" pos="0.0 0.0 1"><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v1" pos="0.1 0.0 1"><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v2" pos="0.2 0.0 1"><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v3" pos="0.0 0.1 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v4" pos="0.1 0.1 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v5" pos="0.2 0.1 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v6" pos="0.0 0.2 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v7" pos="0.1 0.2 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
    <body name="g_v8" pos="0.2 0.2 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>
  </worldbody>
  <deformable>
    <flex name="g" dim="2" body="g_v0 g_v1 g_v2 g_v3 g_v4 g_v5 g_v6 g_v7 g_v8" element="0 1 3 1 4 3 1 2 4 2 5 4 3 4 6 4 7 6 4 5 7 5 8 7">
      <contact contype="0" conaffinity="0"/>
    </flex>
  </deformable>
  <equality>
    <flex flex="g" solref="0.05 1"/>
  </equality>
</mujoco>"""

EQUALITY = """  <equality>
    <flex flex="g" solref="0.05 1"/>
  </equality>
"""


def main():
    oracle()
    assert EQUALITY in XML
    for label, xml in (("with the equality", XML), ("without", XML.replace(EQUALITY, ""))):
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        print(label, "nefc at the start", d.nefc)
        for _ in range(500):
            mujoco.mj_step(m, d)
        print(label, "qpos", [float(q) for q in d.qpos])


if __name__ == "__main__":
    main()
