#!/usr/bin/env python3
"""Generate the constraint impedance golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_impedance_reference.py \\
        sim/L0/tests/assets/golden/impedance

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo takes one impedance for all rows of a connect (3) or a weld (6), from
the norm of their violation (`getposdim`, `engine_core_constraint.c:1366-1395`;
`mj_makeImpedance`, `:1465-1524`). The cases:

- connect_rows: a hinged capsule and a free box joined by a connect, the
  hinge at 0.002 rad from its reference; after mj_forward, the connect rows'
  efc_pos and efc_imp, and qacc.
- weld_hinge: a box on an undamped hinge welded to the world, soft
  (solref "0.1 0.1") so it still swings at step 500, with solimp
  "0.8 0.95 0.01 0.4 3", started 0.008 rad from its weld pose, inside the
  solimp width; qpos and qvel after steps 1, 10, 100 and 500 under Euler,
  implicit and implicitfast.
- cable: the composite cable of `implicit_integration.rs`'s
  `test_implicitfast_connect_ball_chain_stability`, with MuJoCo's straight
  `curve="s 0 0"` for that test's `"l 0 0"` (not a MuJoCo keyword), written
  out by MuJoCo as plain bodies (mj_saveLastXML), with the connect's
  `solref` given both values (MuJoCo writes `"0.002"`, which sim-mjcf reads
  as the default); qpos and qvel after steps 1, 10, 100 and 500.
- limit_margin: a hinge limited to +-0.5 with margin 0.1, at 0.42 rad, so its
  upper limit row is 0.08 from the limit and 0.02 inside the margin: the
  row's efc_pos and efc_imp (an impedance takes `pos - margin`,
  `getimpedance`, `:1413-1461`).
"""
import json
import os
import re
import sys
import tempfile

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

CHECKPOINTS = [1, 10, 100, 500]

CONNECT = """<mujoco model="conn_free2">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="0.35 0 1">
      <freejoint/>
      <geom type="box" size="0.05 0.04 0.03" mass="2" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality>
    <connect body1="a" body2="b" anchor="0.3 0 0"/>
  </equality>
</mujoco>"""


def weld_xml(integrator):
    return f"""<mujoco model="weld_hinge">
  <option timestep="0.002" integrator="{integrator}"/>
  <worldbody>
    <body name="w" pos="0 0 1">
      <joint name="h" type="hinge" axis="0.2 0.3 1"/>
      <geom type="box" size="0.1 0.05 0.03" pos="0.15 0.05 0" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality>
    <weld body1="w" solref="0.1 0.1" solimp="0.8 0.95 0.01 0.4 3"/>
  </equality>
</mujoco>"""


CABLE_COMPOSITE = """<mujoco model="cable-connect-stability">
  <option gravity="0 0 -9.81" timestep="0.001" integrator="implicitfast"
          solver="Newton" iterations="200" tolerance="1e-12"/>
  <worldbody>
    <body name="left" pos="-0.5 0 1">
      <composite type="cable" prefix="A" count="4 1 1"
                 initial="none" curve="s 0 0" size="1.0">
        <joint kind="main" damping="0.5"/>
        <geom type="capsule" size="0.01" density="200"
              contype="0" conaffinity="0"/>
      </composite>
    </body>
    <body name="right" pos="0.5 0 1">
      <geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality>
    <connect body1="AB_last" body2="right" anchor="0.33333 0 0"
             solref="0.002 1.0"/>
  </equality>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def stable(d, what):
    for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
              mujoco.mjtWarning.mjWARN_BADQPOS):
        if d.warning[w].number:
            sys.exit(f"{what}: MuJoCo warned {w.name}")


def trajectory(xml, qpos0_offset, what):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[:] = m.qpos0
    for i, v in qpos0_offset.items():
        d.qpos[i] += v
    out = {}
    for step in range(1, CHECKPOINTS[-1] + 1):
        mujoco.mj_step(m, d)
        if step in CHECKPOINTS:
            out[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    stable(d, what)
    return out


def connect_rows():
    m = mujoco.MjModel.from_xml_string(CONNECT)
    d = mujoco.MjData(m)
    d.qpos[0] = 0.002
    mujoco.mj_forward(m, d)
    stable(d, "connect_rows")
    rows = [i for i in range(d.nefc) if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
    assert len(rows) == 3, rows
    return {"xml": CONNECT, "hinge": 0.002, "efc_pos": floats(d.efc_pos[rows]),
            # efc_KBIP holds K, B, imp, impP per row (mjdata.h).
            "efc_imp": [float(d.efc_KBIP[i, 2]) for i in rows], "qacc": floats(d.qacc)}


def cable():
    m = mujoco.MjModel.from_xml_string(CABLE_COMPOSITE)
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cable.xml")
        mujoco.mj_saveLastXML(path, m)
        xml = open(path).read()
    # MuJoCo writes the connect's solref as one value (its dampratio is the default 1).
    n = len(re.findall(r'solref="0\.002"', xml))
    assert n == 1, f"{n} one-value solrefs"
    xml = xml.replace('solref="0.002"', 'solref="0.002 1"')
    # The composite's name record carries no physics.
    custom = re.findall(r"\n  <custom>.*?</custom>\n", xml, flags=re.S)
    assert len(custom) == 1 and len(custom[0]) < 200, custom
    xml = xml.replace(custom[0], "\n")
    explicit = mujoco.MjModel.from_xml_string(xml)
    for k in ("nq", "nv", "nbody", "neq"):
        assert getattr(explicit, k) == getattr(m, k), k
    return {"xml": xml, "traj": trajectory(xml, {}, "cable")}


LIMIT = """<mujoco model="limit_margin">
  <compiler angle="radian"/>
  <worldbody>
    <body name="p" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0" limited="true" range="-0.5 0.5" margin="0.1"
             solimplimit="0.9 0.95 0.2 0.5 2"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1"/>
    </body>
  </worldbody>
</mujoco>"""


def limit_margin():
    m = mujoco.MjModel.from_xml_string(LIMIT)
    d = mujoco.MjData(m)
    d.qpos[0] = 0.42
    mujoco.mj_forward(m, d)
    stable(d, "limit_margin")
    rows = [i for i in range(d.nefc) if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT]
    assert len(rows) == 1, rows
    i = rows[0]
    # Inside the margin and the solimp width, where `pos - margin` and `pos` differ.
    assert 0 < d.efc_pos[i] < d.efc_margin[i] and d.efc_margin[i] - d.efc_pos[i] < 0.2, d.efc_pos[i]
    return {"xml": LIMIT, "hinge": 0.42, "efc_pos": float(d.efc_pos[i]),
            "efc_margin": float(d.efc_margin[i]), "efc_imp": float(d.efc_KBIP[i, 2])}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "connect_rows": connect_rows(),
        "weld_hinge": {
            "xml": {i: weld_xml(i) for i in ("Euler", "implicit", "implicitfast")},
            "hinge": 0.008,
            "traj": {i: trajectory(weld_xml(i), {0: 0.008}, f"weld {i}")
                     for i in ("Euler", "implicit", "implicitfast")},
        },
        "cable": cable(),
        "limit_margin": limit_margin(),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "impedance.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
