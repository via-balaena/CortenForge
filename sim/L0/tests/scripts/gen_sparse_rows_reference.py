#!/usr/bin/env python3
"""Generate the sparse-row golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_sparse_rows_reference.py \\
        sim/L0/tests/assets/golden/sparse_rows

It refuses any other interpreter, as gen_census_golden.py does.

At the sparse size (`mj_isSparse`) each constraint row lists its dof chain
(`efc_J_rownnz`, `efc_J_rowadr`, `efc_J_colind`): the dofs the row kind passes
to `mj_addConstraint` (`engine_core_constraint.c:237-329`), whatever their
values. One scene at jacobian="sparse" makes every rigid row kind:

- connects (an arm's forearm to a free ball, and to the arm, whose chains share
  the arm's dof) and a weld (a ball joint's slide-jointed child to the arm), the
  bodies' merged chains (`mj_mergeChain`);
- a joint equality (a hinge and the slide) and a tendon equality (a fixed
  tendon over the first hinge and the slide, the slide's coefficient 0, and one
  over both hinges), their joints' and tendons' chains' unions;
- the first hinge's and the slide's friction loss, the first tendon's;
- the first hinge, the ball joint and the slide past their ranges, the first
  tendon and a spatial tendon (forearm to a world site) past theirs;
- the forearm's sphere and the free ball on the floor, the free ball under a
  second free ball: two simple bodies' chain (`mj_mergeChainSimple`) and a
  general one.

Four cases: pyramidal at condim 3, elliptic at condim 6 and 4, and condim 1
(the two free balls' contact frictionless, the floor's pyramidal at the
floor's default condim 3). Each, after mj_forward at its pose, records nv,
efc_type, efc_id, the three chain arrays and efc_J (the rows' values, row by
row in chain order). The generator refuses a case missing a row kind.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

SCENE = """<mujoco model="sparse_rows">
  <option timestep="0.002" jacobian="sparse" cone="{cone}"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <site name="anchor" pos="0 0 0.6"/>
    <body name="arm" pos="0 0 0.3">
      <joint name="h1" type="hinge" axis="0 1 0" range="-20 20" limited="true" frictionloss="0.05"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
      <body name="fore" pos="0.3 0 0">
        <joint name="h2" type="hinge" axis="0 1 0"/>
        <geom name="hand" type="sphere" pos="0.3 0 -0.2" size="0.05" mass="0.4" condim="{condim}"/>
        <site name="tip" pos="0.3 0 0"/>
      </body>
    </body>
    <body name="ball" pos="0.8 0 0.059">
      <freejoint/>
      <geom name="b" type="sphere" size="0.06" mass="0.5" condim="{condim}"/>
    </body>
    <body name="ball2" pos="0.8 0 0.175">
      <freejoint/>
      <geom name="b2" type="sphere" size="0.06" mass="0.5" condim="{condim}"/>
    </body>
    <body name="pend" pos="-0.5 0 0.5">
      <joint name="bj" type="ball" range="0 10" limited="true"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.02" mass="0.3" contype="0" conaffinity="0"/>
      <body name="link" pos="0 0 -0.3">
        <joint name="s1" type="slide" axis="1 0 0" range="-0.05 0.05" limited="true" frictionloss="0.02"/>
        <geom type="box" size="0.03 0.03 0.03" mass="0.2" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
  <equality>
    <connect body1="fore" body2="ball" anchor="0.3 0 0"/>
    <connect body1="fore" body2="arm" anchor="0.1 0 0"/>
    <weld body1="link" body2="arm"/>
    <joint joint1="h2" joint2="s1"/>
    <tendon tendon1="t1" tendon2="t2"/>
  </equality>
  <tendon>
    <fixed name="t1" frictionloss="0.1" limited="true" range="-0.01 0.01"><joint joint="h1" coef="1"/><joint joint="s1" coef="0"/></fixed>
    <fixed name="t2"><joint joint="h1" coef="0.2"/><joint joint="h2" coef="0.5"/></fixed>
    <spatial name="sp" limited="true" range="0 0.2"><site site="tip"/><site site="anchor"/></spatial>
  </tendon>
</mujoco>"""
H1_QPOS = 0.4
BALL_QUAT = [float(np.cos(0.13)), float(np.sin(0.13)), 0.0, 0.0]
S1_QPOS = 0.06
KINDS = {"mjCNSTR_EQUALITY", "mjCNSTR_FRICTION_DOF", "mjCNSTR_FRICTION_TENDON",
         "mjCNSTR_LIMIT_JOINT", "mjCNSTR_LIMIT_TENDON"}


def case(cone, condim, contacts):
    xml = SCENE.format(cone=cone, condim=condim)
    m = mujoco.MjModel.from_xml_string(xml)
    assert mujoco.mj_isSparse(m), "the sparse size"
    d = mujoco.MjData(m)
    d.qpos[m.jnt_qposadr[m.joint("h1").id]] = H1_QPOS
    adr = m.jnt_qposadr[m.joint("bj").id]
    d.qpos[adr:adr + 4] = BALL_QUAT
    d.qpos[m.jnt_qposadr[m.joint("s1").id]] = S1_QPOS
    mujoco.mj_forward(m, d)
    kinds = {mujoco.mjtConstraint(int(t)).name for t in d.efc_type[:d.nefc]}
    if not KINDS | contacts <= kinds:
        sys.exit(f"{cone} {condim}: rows {sorted(kinds)} miss {sorted(KINDS | contacts - kinds)}")
    pairs = {tuple(sorted(m.geom(int(g)).name for g in c.geom))
             for c in d.contact[:d.ncon] if c.efc_address >= 0}
    if not {("floor", "hand"), ("b", "b2")} <= pairs:
        sys.exit(f"{cone} {condim}: contacts with rows {sorted(pairs)}")
    nnz = int(sum(d.efc_J_rownnz[:d.nefc]))
    return {"xml": xml, "qpos": [float(x) for x in d.qpos], "nv": int(m.nv), "nefc": int(d.nefc),
            "efc_type": [int(x) for x in d.efc_type[:d.nefc]],
            "efc_id": [int(x) for x in d.efc_id[:d.nefc]],
            "efc_J_rownnz": [int(x) for x in d.efc_J_rownnz[:d.nefc]],
            "efc_J_rowadr": [int(x) for x in d.efc_J_rowadr[:d.nefc]],
            "efc_J_colind": [int(x) for x in d.efc_J_colind[:nnz]],
            "efc_J": [float(x) for x in d.efc_J[:nnz]]}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "pyramidal_3": case("pyramidal", 3, {"mjCNSTR_CONTACT_PYRAMIDAL"}),
        "elliptic_6": case("elliptic", 6, {"mjCNSTR_CONTACT_ELLIPTIC"}),
        "elliptic_4": case("elliptic", 4, {"mjCNSTR_CONTACT_ELLIPTIC"}),
        "frictionless_1": case("pyramidal", 1, {"mjCNSTR_CONTACT_FRICTIONLESS",
                                                "mjCNSTR_CONTACT_PYRAMIDAL"}),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    os.makedirs(sys.argv[1], exist_ok=True)
    with open(os.path.join(sys.argv[1], "sparse_rows.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
