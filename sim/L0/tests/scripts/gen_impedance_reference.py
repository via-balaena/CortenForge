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

`getsolparam` (`:1291-1362`) replaces a mixed `solref` (one value positive,
the other not) by the default (0.02, 1), and a mixed `solreffriction` by
(0, 0), under which an elliptic contact's friction rows take `solref`:

- mixed_solreffriction: census doc d81df1fa (`unified_solvers.rs`'s
  `<pair solreffriction="0.1 0.0">`, elliptic), started sliding (qvel
  0.5, -0.2 along the plane): a friction row's solref reaches it only through
  B times the row's velocity, which is 0 from rest. qpos and qvel after steps
  1, 10, 50 and 100. The generator refuses it unless the friction rows' B is
  the normal row's and their velocity is not 0.
- mixed_solreffriction_own_solref: the same sphere, its `<pair>` giving
  `solref="0.05 1"` (not the default) and `solreffriction="-0.1 0.5"`, mixed
  the other way; the same checkpoints and refusals. With a default solref, a
  mixed solreffriction replaced by the default solref instead of by (0, 0)
  gives the same rows; here it does not.
- mixed_solref_limit: two hinged arms falling onto their limits, whose
  `solreflimit` is mixed each way ("0.05 -2", "-500 2"); qpos and qvel after
  steps 1, 10, 100 and 300. The generator refuses it unless both limit rows'
  K and B are the default's.

`getimpedance` (`:1413-1461`) takes the curve as flat when `solimp[0] ==
solimp[1]` or `solimp[2] <= mjMINVAL`, and as linear when `solimp[4] == 1`,
each compared exactly (`:1416`, `:1439`):

- impedance_edges: three limited hinges, each past its limit, whose
  solimplimit sits at one of those tests: a width of 1e-11 (above mjMINVAL,
  so not flat: past the width it saturates at solimp[1]), solimp[0] and
  solimp[1] 5e-13 apart, and a power of 1 + 1e-13; after mj_forward, each
  limit row's efc_imp and qacc. The generator refuses it unless each
  impedance differs from what a tolerance at those tests gives: the flat
  midpoint for the first two, the linear curve for the third.
"""
import hashlib
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


def trajectory(xml, qpos0_offset, what, checkpoints=CHECKPOINTS, qvel=None):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[:] = m.qpos0
    for i, v in qpos0_offset.items():
        d.qpos[i] += v
    if qvel is not None:
        d.qvel[:] = qvel
    out = {}
    for step in range(1, checkpoints[-1] + 1):
        mujoco.mj_step(m, d)
        if step in checkpoints:
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


CENSUS_DOCS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "census", "docs")

MIXED_SOLREFFRICTION_DOC = "d81df1fabe5612c1"

SLIDING = [0.5, -0.2, 0.0, 0.0, 0.0, 0.0]


OWN_SOLREF = """<mujoco model="mixed_solreffriction_own_solref">
  <option gravity="0 0 -9.81" timestep="0.002" solver="Newton" cone="elliptic"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="ball" pos="0 0 0.1">
      <joint type="free"/>
      <geom name="sphere" type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="floor" geom2="sphere" condim="3" friction="1.0 1.0 0.005 0.001 0.001"
          solref="0.05 1.0" solreffriction="-0.1 0.5"/>
  </contact>
</mujoco>"""


def census_doc(doc):
    with open(os.path.join(CENSUS_DOCS, doc + ".xml"), encoding="utf-8") as f:
        xml = f.read()
    assert hashlib.sha256(xml.encode()).hexdigest()[:16] == doc
    return xml


def mixed_solreffriction(xml, what):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = SLIDING
    friction_vel = 0.0
    for _ in range(100):
        mujoco.mj_step(m, d)
        elliptic = [i for i in range(d.nefc) if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC]
        if elliptic:
            assert len(elliptic) == 3, elliptic
            # efc_KBIP holds K, B, imp, impP per row (mjdata.h): the friction rows take solref's B.
            b = [float(d.efc_KBIP[i, 1]) for i in elliptic]
            assert b[0] > 0 and b[1] == b[0] and b[2] == b[0], b
            friction_vel = max(friction_vel, *(abs(float(d.efc_vel[i])) for i in elliptic[1:]))
    assert friction_vel > 1e-3, friction_vel
    return {"xml": xml, "qvel": SLIDING, "traj": trajectory(xml, {}, what, [1, 10, 50, 100], SLIDING)}


MIXED_SOLREF_LIMIT = """<mujoco model="mixed_solref_limit">
  <compiler angle="radian"/>
  <option timestep="0.002"/>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0" limited="true" range="-0.3 0.3" solreflimit="0.05 -2"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="arm2" pos="0 1 1">
      <joint name="h2" type="hinge" axis="0 1 0" limited="true" range="-0.2 0.2" solreflimit="-500 2"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""


def mixed_solref_limit():
    m = mujoco.MjModel.from_xml_string(MIXED_SOLREF_LIMIT)
    d = mujoco.MjData(m)
    for _ in range(300):
        mujoco.mj_step(m, d)
    assert d.nefc == 2, d.nefc
    # The default solref (0.02, 1) at dmax 0.95 (`mj_makeImpedance`, standard format).
    k, b = 1 / (0.95 * 0.95 * 0.02 * 0.02 * 1 * 1), 2 / (0.95 * 0.02)
    for i in range(2):
        assert d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT, d.efc_type[i]
        assert (float(d.efc_KBIP[i, 0]), float(d.efc_KBIP[i, 1])) == (k, b), d.efc_KBIP[i]
    return {"xml": MIXED_SOLREF_LIMIT,
            "traj": trajectory(MIXED_SOLREF_LIMIT, {}, "mixed_solref_limit", [1, 10, 100, 300])}


IMPEDANCE_EDGES = {
    "narrow_width": [0.9, 0.95, 1e-11, 0.5, 2.0],
    "near_flat": [0.9, 0.9000000000005, 0.01, 0.5, 2.0],
    "near_linear": [0.9, 0.95, 0.01, 0.5, 1.0000000000001],
}


def impedance_edges():
    bodies = "\n".join(
        f"""    <body name="b{k}" pos="{k} 0 1">
      <joint name="{name}" type="hinge" axis="0 1 0" limited="true" range="-0.1 0.1"
             solimplimit="{' '.join(repr(x) for x in si)}"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>"""
        for k, (name, si) in enumerate(IMPEDANCE_EDGES.items()))
    xml = ('<mujoco model="impedance_edges">\n  <compiler angle="radian"/>\n  <worldbody>\n'
           + bodies + "\n  </worldbody>\n</mujoco>")
    qpos = [0.103, 0.104, 0.104]
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    mujoco.mj_forward(m, d)
    stable(d, "impedance_edges")
    assert d.nefc == 3 and list(d.efc_id) == [0, 1, 2], (d.nefc, d.efc_id)
    imp = [float(d.efc_KBIP[i, 2]) for i in range(3)]
    for i, si in enumerate(IMPEDANCE_EDGES.values()):
        assert [float(x) for x in m.jnt_solimp[i]] == si, m.jnt_solimp[i]
    flat = [0.5 * (si[0] + si[1]) for si in IMPEDANCE_EDGES.values()]
    assert imp[0] == IMPEDANCE_EDGES["narrow_width"][1] and imp[0] != flat[0], imp
    assert imp[1] != flat[1], imp
    x = abs(d.efc_pos[2] - d.efc_margin[2]) / 0.01
    si = IMPEDANCE_EDGES["near_linear"]
    assert 0 < x < 1 and imp[2] != si[0] + x * (si[1] - si[0]), (x, imp)
    return {"xml": xml, "qpos": qpos, "efc_imp": imp, "qacc": floats(d.qacc)}


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
        "mixed_solreffriction": mixed_solreffriction(census_doc(MIXED_SOLREFFRICTION_DOC),
                                                     "mixed_solreffriction"),
        "mixed_solreffriction_own_solref": mixed_solreffriction(OWN_SOLREF,
                                                                "mixed_solreffriction_own_solref"),
        "mixed_solref_limit": mixed_solref_limit(),
        "impedance_edges": impedance_edges(),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "impedance.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
