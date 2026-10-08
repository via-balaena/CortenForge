#!/usr/bin/env python3
"""Generate the sleep golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_sleep_reference.py \\
        sim/L0/tests/assets/golden/sleep

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes sleep.json. Its `trees` entry holds, for each model below, MuJoCo's
kinematic-tree tables (ntree, body_treeid, tree_bodyadr, tree_bodynum,
tree_dofadr, tree_dofnum, dof_treeid, tendon_treenum, tendon_treeid), the
resolved tree_sleep_policy by name, and body_awake after mj_resetData and
after one mj_step, with sleep enabled and with it disabled. The models, all
with sleep enabled:

- table: a box resting on a static box (A8's fixture);
- static_root: a static body with two hinged children;
- interleaved: static bodies between moving trees, and a static body welded
  to a moving one;
- mocap: a mocap body with a child, a free body and a static body;
- actuators: one tree per transmission (joint, jointinparent, site, body,
  slider-crank with its crank and slider sites on two trees, a fixed tendon
  over two trees) and one untouched tree;
- tendons: two trees with stiffness, two with a limit only, three with
  nothing, two with damping only, and a tendon whose wraps reach tree 3
  before tree 2;
- flex: a free body and a 2 x 2 flexcomp grid, whose four vertex bodies are
  trees. sim-mjcf does not read a `<flexcomp>` under `<worldbody>` yet (the
  book's Rigid-loading L10), so the case also holds `ours_xml`, the same grid
  in the `<deformable><flexcomp>` form sim-mjcf reads, which the test loads;
- gravcomp: a static body and a free body with gravity compensation; also
  qfrc_gravcomp after mj_forward.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

SLEEP = '<option><flag sleep="enable"/></option>'

TREES = {
    "table": f"""<mujoco>{SLEEP}
<worldbody><geom type="plane" size="5 5 0.1"/>
<body name="table" pos="0 0 0.5"><geom type="box" size="0.5 0.5 0.05"/></body>
<body name="b" pos="0 0 0.6495"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
</worldbody></mujoco>""",
    "static_root": f"""<mujoco>{SLEEP}
<worldbody>
<body name="base" pos="0 0 1"><geom type="box" size="0.2 0.2 0.05"/>
  <body name="arm1" pos="0.3 0 0"><joint name="h1" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03"/></body>
  <body name="arm2" pos="-0.3 0 0"><joint name="h2" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 -0.3 0 0" size="0.03"/></body>
</body>
</worldbody></mujoco>""",
    "interleaved": f"""<mujoco>{SLEEP}
<worldbody>
<body name="a" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/>
  <body name="a_weld" pos="0.2 0 0"><geom type="sphere" size="0.05"/></body>
</body>
<body name="s" pos="1 0 0"><geom type="box" size="0.1 0.1 0.1"/>
  <body name="s_child" pos="0 0 0.3"><geom type="box" size="0.05 0.05 0.05"/>
    <body name="c1" pos="0 0 0.3"><joint name="hc1"/>
      <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/>
      <body name="c2" pos="0.2 0 0"><joint name="hc2"/>
        <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/></body>
    </body>
  </body>
</body>
<body name="s2" pos="2 0 0"><geom type="box" size="0.1 0.1 0.1"/></body>
<body name="d" pos="3 0 1"><joint name="sd" type="slide" axis="0 0 1"/>
  <geom type="sphere" size="0.1"/></body>
</worldbody></mujoco>""",
    "mocap": f"""<mujoco>{SLEEP}
<worldbody>
<body name="m" mocap="true" pos="0 0 1">
  <geom type="box" size="0.1 0.1 0.1" contype="0" conaffinity="0"/>
  <body name="m_child" pos="0 0 0.2">
    <geom type="sphere" size="0.05" contype="0" conaffinity="0"/></body>
</body>
<body name="f" pos="1 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
<body name="s" pos="2 0 0"><geom type="box" size="0.1 0.1 0.1"/></body>
</worldbody></mujoco>""",
    "actuators": f"""<mujoco>{SLEEP}
<worldbody>
<body name="b0" pos="0 0 1"><joint name="j0"/><geom type="sphere" size="0.05"/></body>
<body name="b1" pos="1 0 1"><joint name="j1"/><geom type="sphere" size="0.05"/></body>
<body name="b2" pos="2 0 1"><joint name="j2"/><geom type="sphere" size="0.05"/><site name="s2"/></body>
<body name="b3" pos="3 0 1"><joint name="j3" type="slide" axis="0 0 1"/>
  <geom type="box" size="0.05 0.05 0.05"/></body>
<body name="b4" pos="4 0 1"><joint name="j4" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.02"/><site name="crank" pos="0.1 0 0"/></body>
<body name="b5" pos="4.3 0 1"><joint name="j5" type="slide" axis="1 0 0"/>
  <geom type="sphere" size="0.05"/><site name="slider"/></body>
<body name="b6" pos="6 0 1"><joint name="j6"/><geom type="sphere" size="0.05"/></body>
<body name="b7" pos="7 0 1"><joint name="j7"/><geom type="sphere" size="0.05"/></body>
<body name="b8" pos="8 0 1"><joint name="j8"/><geom type="sphere" size="0.05"/></body>
</worldbody>
<tendon><fixed name="t67"><joint joint="j6" coef="1"/><joint joint="j7" coef="1"/></fixed></tendon>
<actuator>
  <motor joint="j0"/>
  <motor jointinparent="j1"/>
  <motor site="s2" gear="0 0 0 0 0 1"/>
  <adhesion body="b3" ctrlrange="0 1"/>
  <general cranksite="crank" slidersite="slider" cranklength="0.3"/>
  <motor tendon="t67"/>
</actuator>
</mujoco>""",
    "tendons": f"""<mujoco>{SLEEP}
<worldbody>
<body name="b0" pos="0 0 1"><joint name="j0"/><geom type="sphere" size="0.05"/><site name="s0"/></body>
<body name="b1" pos="1 0 1"><joint name="j1"/><geom type="sphere" size="0.05"/><site name="s1"/></body>
<body name="b2" pos="2 0 1"><joint name="j2"/><geom type="sphere" size="0.05"/></body>
<body name="b3" pos="3 0 1"><joint name="j3"/><geom type="sphere" size="0.05"/></body>
<body name="b4" pos="4 0 1"><joint name="j4"/><geom type="sphere" size="0.05"/></body>
<body name="b5" pos="5 0 1"><joint name="j5"/><geom type="sphere" size="0.05"/></body>
<body name="b6" pos="6 0 1"><joint name="j6"/><geom type="sphere" size="0.05"/></body>
<body name="b7" pos="7 0 1"><joint name="j7"/><geom type="sphere" size="0.05"/></body>
<body name="b8" pos="8 0 1"><joint name="j8"/><geom type="sphere" size="0.05"/></body>
</worldbody>
<tendon>
  <spatial name="stiff01" stiffness="10"><site site="s0"/><site site="s1"/></spatial>
  <fixed name="limit23" limited="true" range="-1 1">
    <joint joint="j2" coef="1"/><joint joint="j3" coef="1"/></fixed>
  <fixed name="three456"><joint joint="j4" coef="1"/><joint joint="j5" coef="1"/>
    <joint joint="j6" coef="1"/></fixed>
  <fixed name="damp78" damping="1"><joint joint="j7" coef="1"/><joint joint="j8" coef="1"/></fixed>
  <fixed name="order32"><joint joint="j3" coef="1"/><joint joint="j2" coef="1"/></fixed>
</tendon>
</mujoco>""",
    "flex": f"""<mujoco>{SLEEP}
<worldbody>
<body name="f" pos="3 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
<flexcomp name="cloth" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" pos="0 0 1" dim="2"
  radius="0.01" mass="0.1"/>
</worldbody></mujoco>""",
    "gravcomp": f"""<mujoco>{SLEEP}
<worldbody>
<body name="hover" pos="0 0 1" gravcomp="1"><geom type="box" size="0.1 0.1 0.1" mass="2"/></body>
<body name="f" pos="1 0 1" gravcomp="0.5"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
</worldbody></mujoco>""",
}

POLICY = {
    int(mujoco.mjtSleepPolicy.mjSLEEP_AUTO): "Auto",
    int(mujoco.mjtSleepPolicy.mjSLEEP_AUTO_NEVER): "AutoNever",
    int(mujoco.mjtSleepPolicy.mjSLEEP_AUTO_ALLOWED): "AutoAllowed",
    int(mujoco.mjtSleepPolicy.mjSLEEP_NEVER): "Never",
    int(mujoco.mjtSleepPolicy.mjSLEEP_ALLOWED): "Allowed",
    int(mujoco.mjtSleepPolicy.mjSLEEP_INIT): "Init",
}


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def floats(a):
    return [float(x) for x in a]


def awake_after(m, step):
    d = mujoco.MjData(m)
    if step:
        mujoco.mj_step(m, d)
    return ints(d.body_awake)


OURS_XML = {
    "flex": """<mujoco><option><flag sleep="enable"/></option>
<worldbody>
<body name="f" pos="3 0 1"><freejoint/><geom type="sphere" size="0.1"/></body>
</worldbody>
<deformable><flexcomp name="cloth" type="grid" count="2 2 1" spacing="0.1" pos="0 0 1" dim="2"
  radius="0.01" mass="0.1"/></deformable>
</mujoco>""",
}


def tree_case(name, xml):
    m = mujoco.MjModel.from_xml_string(xml)
    out = {
        "name": name,
        "xml": xml,
        "ours_xml": OURS_XML.get(name),
        "ntree": int(m.ntree),
        "body_treeid": ints(m.body_treeid),
        "tree_bodyadr": ints(m.tree_bodyadr),
        "tree_bodynum": ints(m.tree_bodynum),
        "tree_dofadr": ints(m.tree_dofadr),
        "tree_dofnum": ints(m.tree_dofnum),
        "dof_treeid": ints(m.dof_treeid),
        "tendon_treenum": ints(m.tendon_treenum),
        "tendon_treeid": ints(m.tendon_treeid),
        "tree_sleep_policy": [POLICY[int(p)] for p in m.tree_sleep_policy],
        "body_awake_reset": awake_after(m, False),
        "body_awake_step": awake_after(m, True),
    }
    m.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    out["body_awake_reset_nosleep"] = awake_after(m, False)
    out["body_awake_step_nosleep"] = awake_after(m, True)
    m.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    out["qfrc_gravcomp"] = floats(d.qfrc_gravcomp)
    return out


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    trees = [tree_case(name, xml) for name, xml in TREES.items()]
    doc = {"oracle": marker, "trees": finite_or_string(trees)}
    write(os.path.join(sys.argv[1], "sleep.json"), doc)
    print(f"{len(trees)} tree cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
