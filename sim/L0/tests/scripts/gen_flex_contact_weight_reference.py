#!/usr/bin/env python3
"""Print MuJoCo 3.5.0's flex contact rows' R that `flex_unified.rs` holds as literals.

Run it with the unfused oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flex_contact_weight_reference.py

It refuses any other interpreter, as gen_census_golden.py does.

A 3 x 3 cloth (mass 0.1, radius 0.01) 5 mm into a plane on a static body,
equality constraints disabled (sim-mjcf gives every flex an edge equality,
Rigid-loading L31, so both sides drop it): its vertex positions and
elements, then each contact's sides and each constraint row's R after
mj_forward. A contact row's R comes from the two sides' bodies' invweight0
(mj_diagApprox, engine_core_constraint.c:1201-1285): the plane's static
body and the vertex's body.
"""
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import oracle  # noqa: E402

XML = """<mujoco>
  <option gravity="0 0 0"><flag equality="disable"/></option>
  <worldbody>
    <body name="ground"><geom name="floor" type="plane" size="2 2 0.1"/></body>
    <flexcomp name="A" type="grid" count="3 3 1" spacing="0.05 0.05 0.05" pos="0 0 0.005"
              radius="0.01" dim="2" mass="0.1">
      <contact selfcollide="none" internal="false"/>
    </flexcomp>
  </worldbody>
</mujoco>"""


def main():
    oracle()
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print("vertices", [[float(x) for x in v] for v in d.flexvert_xpos])
    print("elements", [int(i) for i in m.flex_elem[: 3 * m.flex_elemnum[0]]])
    print("contacts (geom, vert)", [(int(c.geom[0]), int(c.vert[1])) for c in d.contact[: d.ncon]])
    print("nefc", d.nefc, "R", sorted({repr(float(r)) for r in d.efc_R}))


if __name__ == "__main__":
    main()
