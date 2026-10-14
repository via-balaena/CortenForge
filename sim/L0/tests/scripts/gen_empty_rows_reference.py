#!/usr/bin/env python3
"""Generate the empty-constraint golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_empty_rows_reference.py \\
        sim/L0/tests/assets/golden/empty_rows

It refuses any other interpreter, as gen_census_golden.py does.

A constraint other than a contact whose Jacobian rows are all zero adds no rows
(`mj_addConstraint`, `engine_core_constraint.c:237-329`). The cases, each after
mj_forward from qpos0 and rest:

- weld_static: census doc 0ee0d13e (`builder/equality.rs:551`), a weld between two
  bodies without dofs.
- connect_static: census doc f85c405a (`builder/equality.rs:454`), a connect from a
  body without dofs to the world.
- tendon_static: two spatial tendons between two sites on the world body, one with
  friction loss and a range it exceeds, one shorter than its range, beside a hinge
  with friction loss: only the hinge's friction row is made.

Each records nefc and efc_type. A ball joint's limit always has a row inside its
margin, its direction never all zero: below mjMINVAL `mju_normalize3` makes it the
x axis (`engine_core_constraint.c:824-828`). The cases ball_identity (a hanging
ball joint at the identity, a limit of 0.01 degree inside a margin of 0.02) and
ball_tiny (the same at a rotation of about 9.2e-12 rad, above mjMINVAL, so the
row takes the rotation's own axis) also record efc_J, efc_pos and qacc; the
generator refuses either unless it has one limit row.
"""
import json
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

WELD_STATIC = """<mujoco model="m">
  <worldbody>
    <body name="a" pos="1 0 0">
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
    <body name="b" pos="0 1 0">
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <equality>
    <weld body1="a" body2="b" anchor="0 0 0"/>
  </equality>
</mujoco>"""

CONNECT_STATIC = """<mujoco model="m">
  <worldbody>
    <body name="a" pos="0 0 1">
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <equality>
    <connect name="c1" body1="a" anchor="0.2 0 0"/>
  </equality>
</mujoco>"""

TENDON_STATIC = """<mujoco model="tendon_static">
  <worldbody>
    <site name="s1" pos="0 0 1"/>
    <site name="s2" pos="1 0 1"/>
    <body name="arm" pos="0 0 2">
      <joint name="h" type="hinge" axis="0 1 0" frictionloss="0.2"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t" frictionloss="0.5" limited="true" range="0 0.5">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
    <spatial name="t2" limited="true" range="1.5 2">
      <site site="s1"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>"""


BALL_LIMIT = """<mujoco model="ball_limit">
  <worldbody>
    <body name="b" pos="0 0 1">
      <joint name="j" type="ball" range="0 0.01" margin="0.02"/>
      <geom type="capsule" fromto="0 0 0 0.2 0.1 -0.3" size="0.02" mass="1"/>
    </body>
  </worldbody>
</mujoco>"""


def ball_case(quat):
    m = mujoco.MjModel.from_xml_string(BALL_LIMIT)
    d = mujoco.MjData(m)
    d.qpos[:] = quat
    mujoco.mj_forward(m, d)
    if d.nefc != 1 or d.efc_type[0] != int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT):
        sys.exit(f"ball_limit: {d.nefc} rows, not one limit row")
    return {"xml": BALL_LIMIT, "qpos": [float(x) for x in d.qpos], "nefc": int(d.nefc),
            "efc_type": [int(t) for t in d.efc_type[:d.nefc]],
            "efc_J": [float(x) for x in d.efc_J[:d.nefc * m.nv]],
            "efc_pos": [float(x) for x in d.efc_pos[:d.nefc]], "qacc": [float(x) for x in d.qacc]}


def case(xml):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return {"xml": xml, "nefc": int(d.nefc), "efc_type": [int(t) for t in d.efc_type[:d.nefc]]}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "weld_static": case(WELD_STATIC),
        "connect_static": case(CONNECT_STATIC),
        "tendon_static": case(TENDON_STATIC),
        "ball_identity": ball_case([1.0, 0.0, 0.0, 0.0]),
        "ball_tiny": ball_case([1.0, 1e-12, 2e-12, 4e-12]),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "empty_rows.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
