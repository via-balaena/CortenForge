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

At the sparse size (`mj_isSparse`: `<option jacobian="sparse">`, or the default
"auto" from 60 dofs) the rule is the constraint's dof chain, not its values
(`engine_core_constraint.c:273-283`). chain_<kind>_<jacobian>_nv<nv>: a connect,
a weld, and a fixed tendon of coef 0 with friction loss and a limit it exceeds,
each between or on two jointless children of one hinged body, so their rows are
zero and their chain is the hinge; at 61 dofs (ten free spheres beside them),
auto and dense; at 60 and 59 (sliders for the last sphere), auto, either side
of the automatic size; and at 1, sparse, dense and auto. spatial_sparse: a
spatial tendon from a world site to a hinged arm, with friction loss, at
jacobian sparse. <case>_sparse: weld_static, connect_static and tendon_static at
jacobian sparse (their chains are empty, so no rows), and a connect from a body
without dofs to a hinged arm (its chain is the hinge's). tendon_chains records
`ten_J_rownnz` at jacobian sparse: a fixed tendon over two hinges, one of coef 0;
spatial tendons across two hinged bodies and a slider (the last spans all three
dofs); and a fixed tendon naming one hinge twice (one dof).
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


FREE_SPHERES = "".join(
    f'<body pos="{3 + i} 0 1"><freejoint/>'
    '<geom type="sphere" size="0.1" mass="1" contype="0" conaffinity="0"/></body>'
    for i in range(10))

CHAIN = """<mujoco model="chain_{kind}">
  <option{option}/>
  <worldbody>
    <body name="p" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
      <body name="a" pos="0.2 0 0"><geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/></body>
      <body name="b" pos="0.2 0 0"><geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/></body>
    </body>
    {filler}
  </worldbody>
  {constraint}
</mujoco>"""

CHAIN_CONSTRAINTS = {
    "connect": '<equality><connect body1="a" body2="b" anchor="0 0 0"/></equality>',
    "weld": '<equality><weld body1="a" body2="b"/></equality>',
    "tendon": '<tendon><fixed name="t" frictionloss="1" limited="true" range="-1 -0.5">'
              '<joint joint="h" coef="0"/></fixed></tendon>',
    "tendon_equality": '<tendon><fixed name="t"><joint joint="h" coef="0"/></fixed></tendon>'
                       '<equality><tendon tendon1="t"/></equality>',
}


# The hinge, nine free spheres and four or five sliders: 59 and 60 dofs, either
# side of the automatic sparse size.
def sliders(n):
    return "".join(
        f'<body pos="{3 + i} 2 1"><joint type="slide" axis="1 0 0"/>'
        '<geom type="sphere" size="0.1" mass="1" contype="0" conaffinity="0"/></body>'
        for i in range(n))


FILLERS = {61: FREE_SPHERES, 60: FREE_SPHERES.rsplit("<body", 1)[0] + sliders(5),
           59: FREE_SPHERES.rsplit("<body", 1)[0] + sliders(4), 1: ""}


def chain_case(kind, jacobian, nv):
    option = f' jacobian="{jacobian}"' if jacobian != "auto" else ""
    xml = CHAIN.format(kind=kind, option=option, filler=FILLERS[nv],
                       constraint=CHAIN_CONSTRAINTS[kind])
    out = case(xml)
    m = mujoco.MjModel.from_xml_string(xml)
    if m.nv != nv:
        sys.exit(f"chain_{kind}: {m.nv} dofs, not {nv}")
    out["nv"] = nv
    return out


SPATIAL_SPARSE = """<mujoco model="spatial_sparse">
  <option jacobian="sparse"/>
  <worldbody>
    <site name="w" pos="0 0 2"/>
    <body name="arm" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
      <site name="tip" pos="0.3 0 0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t" frictionloss="0.5">
      <site site="w"/>
      <site site="tip"/>
    </spatial>
  </tendon>
</mujoco>"""

TENDON_CHAINS = """<mujoco model="tendon_chains">
  <option jacobian="sparse"/>
  <worldbody>
    <site name="w" pos="0 0 2"/>
    <body name="a" pos="0 0 1">
      <joint name="ha" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
      <site name="sa" pos="0.3 0 0"/>
      <body name="b" pos="0.3 0 0">
        <joint name="hb" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
        <site name="sb" pos="0.3 0 0"/>
      </body>
    </body>
    <body name="c" pos="2 0 1">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
      <site name="sc" pos="0 0 0.3"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="f"><joint joint="ha" coef="0"/><joint joint="hb" coef="1"/></fixed>
    <spatial name="world_to_b"><site site="w"/><site site="sb"/></spatial>
    <spatial name="a_to_c"><site site="sa"/><site site="sc"/></spatial>
    <spatial name="a_to_b"><site site="sa"/><site site="sb"/></spatial>
    <spatial name="c_to_b"><site site="sc"/><site site="sb"/></spatial>
    <fixed name="repeat"><joint joint="ha" coef="1"/><joint joint="ha" coef="2"/></fixed>
  </tendon>
</mujoco>"""


CONNECT_TO_ARM = """<mujoco model="connect_to_arm">
  <option jacobian="sparse"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
    <body name="arm" pos="0 0 2">
      <joint name="h" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
    </body>
  </worldbody>
  <equality>
    <connect body1="a" body2="arm" anchor="0 0 0"/>
  </equality>
</mujoco>"""


def sparse(xml):
    head, rest = xml.split(">", 1)
    return head + '>\n  <option jacobian="sparse"/>' + rest


def tendon_chains():
    m = mujoco.MjModel.from_xml_string(TENDON_CHAINS)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return {"xml": TENDON_CHAINS, "ten_J_rownnz": [int(x) for x in d.ten_J_rownnz]}


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
        "spatial_sparse": case(SPATIAL_SPARSE),
        "weld_static_sparse": case(sparse(WELD_STATIC)),
        "connect_static_sparse": case(sparse(CONNECT_STATIC)),
        "tendon_static_sparse": case(sparse(TENDON_STATIC)),
        "connect_to_arm_sparse": case(CONNECT_TO_ARM),
        "tendon_chains": tendon_chains(),
    }
    for kind in CHAIN_CONSTRAINTS:
        for nv, settings in ((61, ("auto", "dense")), (60, ("auto",)), (59, ("auto",)),
                             (1, ("sparse", "dense", "auto"))):
            for jacobian in settings:
                cases[f"chain_{kind}_{jacobian}_nv{nv}"] = chain_case(kind, jacobian, nv)
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "empty_rows.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
