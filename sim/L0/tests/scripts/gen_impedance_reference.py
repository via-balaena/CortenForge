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
- mixed_solref_rows: a mixed solref on each other row kind: a free box's
  weld ("0.05 -2"), a connect ("-100 5"), a joint equality ("0.03 0"), two
  hinges' friction loss ("0.02 -1", "-50 -2"), a fixed tendon's limit
  ("-200 3") and friction loss ("0.03 -1"), a contact whose two geoms'
  solref mix (solmix) to a mixed one, and override's o_solref ("0.03 -1");
  qpos and qvel after steps 1, 10, 50, 100 and 200. The generator refuses a
  scene during which MuJoCo warns of no replacement.

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

`mj_makeImpedance` (`:1465-1579`) gives each row R = (1-imp)/imp *
diagApprox, then for a pyramidal or elliptic contact R[1] = R[0]/impratio
and the regularized cone's mu = friction[0]*sqrt(R[1]/R[0]) (`:1536-1540`);
an elliptic contact's other rows R[j+1] = R[1]*friction[0]^2/friction[j]^2
(`:1544-1548`), a pyramidal contact's rows all 2*mu^2*R[0] (`:1553-1562`);
then D = 1/R, and every row's diagApprox = R*imp/(1-imp) (`:1570-1578`):

- elliptic_rows: one sphere 1 mm into a plane, Newton, elliptic, at condim
  3, 4 and 6, sliding (qvel 0.05, 0.02) and also spinning (0.1 about the
  normal); after mj_forward, efc_R, efc_D, efc_diagApprox, efc_force, qacc
  and the contact's mu.
- impratio: impratio 10, the sphere sliding, spinning and rolling, at
  condim 3 and 6 under an elliptic cone (Newton, CG, PGS) and a pyramidal
  one (Newton); the first forward's rows and mu as above, qpos and qvel
  after steps 1, 10, 100 and 300, each step's solver_niter, and at each of
  those checkpoints MuJoCo's own
  1-ulp spread: the largest change of qpos when one starting velocity is
  moved by 1 ulp, over the six.
- impratio_noslip: the impratio sphere, elliptic, Newton, condim 3 and 6,
  noslip_iterations 10; after mj_forward, solver_niter, each statistic's
  nactive and nchange, and every row's efc_state: noslip's `dualState`
  takes the regularized cone's mu (`engine_solver.c:233-310`).
- pair_condim6: census doc 2cb92700 (`<pair condim="6"
  solreffriction="0.08 0.6">`, elliptic), started sliding and spinning
  (from rest its friction rows have no velocity); qpos and qvel after steps
  1, 10, 50 and 100.
- aniso_rows: the sphere under a `<pair>` of condim 6 whose two sliding
  frictions differ (0.8, 0.4), impratio 2.5, sliding, spinning and rolling,
  under Newton, CG and PGS: the first forward's rows and mu as above, and
  under PGS qpos and qvel after steps 1, 10, 50 and 100. The generator
  refuses it unless the second tangent's R is not the first's.
- diag_approx: census docs a0ff89f4 and 406a1991 (boxes into a plane,
  pyramidal), from qpos0, and 60c23753 (frictionless, a sphere at distance
  0 from its plane at qpos0, so without rows) moved 1 mm into the plane;
  and every_row_kind, a scene with equality, friction-loss (dof and tendon),
  limit and elliptic contact rows, at its state; and frictionloss_rows,
  mixed_solref_rows' two hinges with friction loss, given a velocity, whose
  rows' diagApprox, remade from R, are not their dofs' invweight0 (an ulp
  apart); after mj_forward, every row's efc_diagApprox and efc_R. The
  generator refuses every_row_kind unless each of those kinds makes a
  row, and frictionloss_rows unless one row's diagApprox is not its dof's
  invweight0.

A NaN violation goes on to `getimpedance`'s curve, which gives a NaN impedance,
and `mju_max` keeps a NaN R (`:1422-1461`, `:1484`):

- nan_turn: a free body over a plane turned by a NaN quaternion (mesh,
  ellipsoid, box), its contacts at a NaN distance; after mj_forward, each
  row's impedance (`efc_KBIP`'s third), R, D and diagApprox. The generator
  refuses a case whose rows are not all NaN.

A weld's rotational error and rotational Jacobian rows are scaled by its
torquescale, `eq_data[10]` (`:482`, `:510`, `:530`):

- weld_torquescale: a free box welded to the world, soft (solref "0.05 1"),
  started turned 0.2 rad about (1, 2, 3) and spinning, at torquescale 0,
  0.5 and 2; and two free boxes welded to each other at 0.3, the second
  started turned; qpos and qvel after steps 1, 10, 100 and 500. The
  generator refuses a case unless MuJoCo's `eq_data[10]` is its
  torquescale. sim-mjcf reads no `torquescale` until Rigid-loading's L41, so
  the test writes it into `eq_data[10]` after loading.
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


def sphere_xml(condim, cone, solver, impratio=None, noslip=None):
    option = f' impratio="{impratio}"' if impratio is not None else ""
    option += f' noslip_iterations="{noslip}"' if noslip is not None else ""
    return f"""<mujoco model="sphere_c{condim}_{cone}_{solver}">
  <option timestep="0.002" solver="{solver}" cone="{cone}"{option}/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" condim="{condim}"/>
    <body name="ball" pos="0 0 0.099">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" condim="{condim}"/>
    </body>
  </worldbody>
</mujoco>"""


def rows(m, d):
    """The first forward's rows, as mj_makeImpedance leaves them."""
    out = {k: floats(getattr(d, k)[:d.nefc]) for k in ("efc_R", "efc_D", "efc_diagApprox", "efc_force")}
    out["qacc"] = floats(d.qacc)
    out["mu"] = float(d.contact[0].mu)
    return out


ELLIPTIC_QVEL = {"slide": [0.05, 0.02, 0.0, 0.0, 0.0, 0.0], "spin": [0.05, 0.02, 0.0, 0.0, 0.0, 0.1]}


def elliptic_rows():
    out = {}
    for condim in (3, 4, 6):
        for label, qvel in ELLIPTIC_QVEL.items():
            xml = sphere_xml(condim, "elliptic", "Newton")
            m = mujoco.MjModel.from_xml_string(xml)
            d = mujoco.MjData(m)
            d.qvel[:] = qvel
            mujoco.mj_forward(m, d)
            stable(d, f"elliptic c{condim} {label}")
            assert d.ncon == 1 and d.nefc == condim, (d.ncon, d.nefc)
            out[f"c{condim}_{label}"] = {"xml": xml, "qvel": qvel, **rows(m, d)}
    return out


IMPRATIO_QVEL = [0.3, -0.2, 0.0, 0.5, -0.4, 0.8]
IMPRATIO_STEPS = [1, 10, 100, 300]


def step_niter(xml, qvel, nstep):
    """solver_niter after each of the first `nstep` steps."""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = qvel
    out = []
    for _ in range(nstep):
        mujoco.mj_step(m, d)
        out.append(int(d.solver_niter[0]))
    return out


def ulp_spread(xml, qvel, checkpoints):
    """At each checkpoint, the largest change of qpos over the six runs whose
    starting qvel has one entry moved up by 1 ulp."""
    m = mujoco.MjModel.from_xml_string(xml)
    def run(v):
        d = mujoco.MjData(m)
        d.qvel[:] = v
        out = {}
        for step in range(1, checkpoints[-1] + 1):
            mujoco.mj_step(m, d)
            if step in checkpoints:
                out[step] = d.qpos.copy()
        return out
    base = run(qvel)
    spread = {step: 0.0 for step in checkpoints}
    for i in range(len(qvel)):
        v = np.array(qvel, dtype=float)
        v[i] = np.nextafter(v[i], np.inf)
        for step, q in run(v).items():
            spread[step] = max(spread[step], float(np.abs(q - base[step]).max()))
    return {str(step): s for step, s in spread.items()}


def impratio():
    out = {}
    for cone, solvers in (("elliptic", ("Newton", "CG", "PGS")), ("pyramidal", ("Newton",))):
        for condim in (3, 6):
            for solver in solvers:
                xml = sphere_xml(condim, cone, solver, impratio=10)
                m = mujoco.MjModel.from_xml_string(xml)
                d = mujoco.MjData(m)
                d.qvel[:] = IMPRATIO_QVEL
                mujoco.mj_forward(m, d)
                stable(d, f"impratio {cone} c{condim} {solver}")
                first = rows(m, d)
                # The regularized cone is narrower than the friction's: mu = friction[0]*sqrt(1/impratio).
                assert 0 < first["mu"] < 1, first["mu"]
                out[f"{cone}_c{condim}_{solver}"] = {
                    "xml": xml, "qvel": IMPRATIO_QVEL, "forward": first,
                    "traj": trajectory(xml, {}, f"impratio {cone} c{condim} {solver}", IMPRATIO_STEPS,
                                       IMPRATIO_QVEL),
                    "niter": step_niter(xml, IMPRATIO_QVEL, IMPRATIO_STEPS[-1]),
                    "ulp_spread": ulp_spread(xml, IMPRATIO_QVEL, IMPRATIO_STEPS)}
    return out


def impratio_noslip():
    out = {}
    for condim in (3, 6):
        xml = sphere_xml(condim, "elliptic", "Newton", impratio=10, noslip=10)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.qvel[:] = IMPRATIO_QVEL
        mujoco.mj_forward(m, d)
        stable(d, f"impratio noslip c{condim}")
        n = int(d.solver_niter[0])
        assert d.nefc == condim and n > 0, (d.nefc, n)
        out[f"c{condim}"] = {"xml": xml, "qvel": IMPRATIO_QVEL, "niter": n,
                             "nactive": [int(s.nactive) for s in d.solver[:n]],
                             "nchange": [int(s.nchange) for s in d.solver[:n]],
                             "efc_state": [int(s) for s in d.efc_state[:d.nefc]]}
    return out


PAIR_CONDIM6_DOC = "2cb92700e75eb1ae"
PAIR_CONDIM6_QVEL = [0.5, -0.2, 0.0, 0.3, -0.3, 0.8]


def pair_condim6():
    xml = census_doc(PAIR_CONDIM6_DOC)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = PAIR_CONDIM6_QVEL
    moved = 0.0
    for _ in range(100):
        mujoco.mj_step(m, d)
        if d.nefc:
            assert d.nefc == 6, d.nefc
            moved = max(moved, *(abs(float(v)) for v in d.efc_vel[3:6]))
    assert moved > 1e-3, moved
    return {"xml": xml, "qvel": PAIR_CONDIM6_QVEL,
            "traj": trajectory(xml, {}, "pair_condim6", [1, 10, 50, 100], PAIR_CONDIM6_QVEL)}


MIXED_SOLREF_ROWS = {
    "weld": ("""<mujoco model="weld_box">
  <option timestep="0.002"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.06 0.04" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality>
    <weld body1="box" solref="0.05 -2" />
  </equality>
</mujoco>""",
               [0, 0, 1, 0.9950041652780258, 0.026681602917391797, 0.05336320583478359, 0.08004480875217539], [0.1, 0, 0, 0.5, -0.3, 0.8]),
    "connect": ("""<mujoco model="conn">
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
    <connect body1="a" body2="b" anchor="0.3 0 0" solref="-100 5"/>
  </equality>
</mujoco>""",
               [0.01, 0.35, 0, 1, 1, 0, 0, 0], None),
    "joint": ("""<mujoco model="jointeq">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="0 1 1">
      <joint name="j2" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality>
    <joint joint1="j1" joint2="j2" polycoef="0 1 0 0 0" solref="0.03 0"/>
  </equality>
</mujoco>""",
               [0.1, -0.1], None),
    "frictionloss": ("""<mujoco model="floss">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0" frictionloss="0.3" solreffriction="0.02 -1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="0 1 1">
      <joint name="j2" type="hinge" axis="0 1 0" frictionloss="0.3" solreffriction="-50 -2"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>""",
               None, [1.0, 1.0]),
    "tendon": ("""<mujoco model="tendon">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="0 1 1">
      <joint name="j2" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t1" limited="true" range="-0.2 0.2" solreflimit="-200 3" frictionloss="0.2" solreffriction="0.03 -1">
      <joint joint="j1" coef="1"/>
      <joint joint="j2" coef="0.5"/>
    </fixed>
  </tendon>
</mujoco>""",
               None, [0.8, 0.4]),
    "solmix": ("""<mujoco model="solmix">
  <option timestep="0.002" cone="elliptic"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" solref="0.02 1" solmix="1" condim="3"/>
    <body name="ball" pos="0 0 0.099">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" solref="-1000 5" solmix="1" condim="3"/>
    </body>
  </worldbody>
</mujoco>""",
               None, [0.3, -0.2, 0, 0.5, -0.4, 0.8]),
    "override": ("""<mujoco model="override">
  <option timestep="0.002" cone="elliptic" o_solref="0.03 -1">
    <flag override="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="ball" pos="0 0 0.099">
      <freejoint/>
      <geom name="sphere" type="sphere" size="0.1" mass="1" condim="4"/>
    </body>
  </worldbody>
</mujoco>""",
               None, [0.3, -0.2, 0, 0.5, -0.4, 0.8]),
}


ANISO = """<mujoco model="pair">
  <option timestep="0.002" cone="elliptic" impratio="2.5" solver="PGS"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="ball" pos="0 0 0.099">
      <freejoint/>
      <geom name="sphere" type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="floor" geom2="sphere" condim="6" friction="0.8 0.4 0.02 0.003 0.001" solref="0.02 1"/>
  </contact>
</mujoco>"""
ANISO_QVEL = [0.3, -0.2, 0.0, 0.5, -0.4, 0.8]


EVERY_ROW_KIND = """<mujoco model="every">
  <compiler angle="radian"/>
  <option timestep="0.002" cone="elliptic" impratio="2"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" condim="4"/>
    <body name="a" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0" limited="true" range="-0.1 0.1" frictionloss="0.2"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
      <body name="a2" pos="0.3 0 0">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
      </body>
    </body>
    <body name="b" pos="1 0 1">
      <freejoint/>
      <geom type="box" size="0.05 0.04 0.03" mass="2" contype="0" conaffinity="0"/>
    </body>
    <body name="c" pos="-1 0 1">
      <freejoint/>
      <geom type="box" size="0.05 0.04 0.03" mass="2" contype="0" conaffinity="0"/>
    </body>
    <body name="ball" pos="0 2 0.099">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" condim="4"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t" limited="true" range="-0.05 0.05" frictionloss="0.1">
      <joint joint="j1" coef="1"/>
      <joint joint="j2" coef="1"/>
    </fixed>
  </tendon>
  <equality>
    <connect body1="a2" body2="b" anchor="0.3 0 0"/>
    <weld body1="c"/>
    <joint joint1="j2" polycoef="0.05 0 0 0 0"/>
  </equality>
</mujoco>"""
EVERY_ROW_KIND_STATE = ([0.12, 0.03, 1.01, 0, 1, 1, 0, 0, 0, -1, 0.01, 1, 0.9996875162757026, 0.0, 0.0, 0.024997395914712332, 0, 2, 0.099, 1, 0, 0, 0],
                        [0.5, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0.3, 0, 0, 0, 0, 0.5])


def mixed_solref_rows(warned):
    """MIXED_SOLREF_ROWS, each a row kind with a mixed solref: qpos and qvel
    after steps 1, 10, 50, 100 and 200. It refuses a scene unless MuJoCo
    warns, during it, that it replaces a mixed solref or solreffriction."""
    out = {}
    for name, (xml, qpos, qvel) in MIXED_SOLREF_ROWS.items():
        before = len(warned)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        if qpos is not None:
            d.qpos[:] = qpos
        if qvel is not None:
            d.qvel[:] = qvel
        traj = {}
        for step in range(1, 201):
            mujoco.mj_step(m, d)
            if step in (1, 10, 50, 100, 200):
                traj[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
        stable(d, name)
        if not any("replacing with default" in w for w in warned[before:]):
            sys.exit(f"{name}: MuJoCo replaced no mixed solref")
        out[name] = {"xml": xml, "qpos": qpos, "qvel": qvel, "traj": traj}
    return out


def aniso_rows():
    """ANISO under Newton, CG and PGS: the first forward's rows as rows()
    gives them, and under PGS qpos and qvel after steps 1, 10, 50 and 100.
    It refuses the scene unless the second tangent's R is not the first's."""
    out = {}
    for solver in ("Newton", "CG", "PGS"):
        xml = ANISO.replace('solver="PGS"', f'solver="{solver}"')
        assert solver == "PGS" or xml != ANISO
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.qvel[:] = ANISO_QVEL
        mujoco.mj_forward(m, d)
        stable(d, f"aniso {solver}")
        assert d.ncon == 1 and d.nefc == 6, (d.ncon, d.nefc)
        if d.efc_R[2] == d.efc_R[1]:
            sys.exit("aniso: the tangents' R are equal")
        out[solver] = {"xml": xml, "qvel": ANISO_QVEL, **rows(m, d)}
    out["PGS"]["traj"] = trajectory(out["PGS"]["xml"], {}, "aniso PGS", [1, 10, 50, 100], ANISO_QVEL)
    return out


# Each doc's height offset of its free joint (qpos[2]) from qpos0.
DIAG_APPROX_DOCS = {"a0ff89f48fc8fcc9": 0.0, "406a1991539e1cc2": 0.0, "60c23753e7d2ea34": -0.001}

# A sphere 1 mm into a plane, sliding, pyramidal, its sliding friction not 1:
# a facet pair's diagApprox is tran + friction^2*tran, each product apart
# (`mj_diagApprox`, `engine_core_constraint.c:1276-1280`); at mass 0.3 that
# rounds otherwise than (1 + friction^2)*tran, which the generator checks.
PYRAMIDAL_FRICTION = """<mujoco model="pyramidal_friction">
  <option timestep="0.002" cone="pyramidal"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" friction="0.9 0.02 0.002"/>
    <body pos="0 0 0.099">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="0.3" condim="{condim}" friction="0.9 0.02 0.002"/>
    </body>
  </worldbody>
</mujoco>"""
PYRAMIDAL_FRICTION_QVEL = [0.05, 0.02, 0, 0, 0, 0.1]

# Two slides of masses 1 and 0.75, each under its own fixed tendon, the
# tendons coupled by a tendon equality and started apart: its row's
# diagApprox adds both tendons' invweight0 (`engine_core_constraint.c:1139-1150`).
TENDON_PAIR = """<mujoco model="tendon_pair">
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="a">
      <joint name="ja" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="0 0.5 0">
      <joint name="jb" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.05" mass="0.75" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="ta"><joint joint="ja" coef="1"/></fixed>
    <fixed name="tb"><joint joint="jb" coef="1"/></fixed>
  </tendon>
  <equality>
    <tendon tendon1="ta" tendon2="tb" polycoef="0 1 0 0 0"/>
  </equality>
</mujoco>"""
TENDON_PAIR_QPOS = [0.02, -0.01]

# gen_primal_solver_reference.py's chain (a free sphere, three links on four
# hinges, a second free sphere) at condim 3 under a pyramidal cone, each body's
# inertia written out (`<inertial>`), so the compiled masses, inertias and
# inertial frames are MuJoCo's (sim-mjcf's own round otherwise: ledgers L195,
# L198): its bodies' and dofs' invweight0, made from M at qpos0
# (`engine_setconst.c:620-712`), and its contact rows. The inertias are picked
# so that each average set0 takes, (a + b + c)/3, rounds otherwise grouped as
# (a + (b + c))/3 for a body's translational weight, a body's rotational weight
# and a free joint's rotational dofs, which the generator checks.
INERTIAL_CHAIN = """<mujoco model="inertial_chain">
  <option timestep="0.002" cone="pyramidal" jacobian="sparse">
    <flag island="disable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="root" pos="0 0 0.06">
      <inertial pos="0 0 0" mass="2" diaginertia="0.00452654 0.00459093 0.00467655"/>
      <freejoint/>
      <geom type="sphere" size="0.07" mass="2"/>
      <body name="l1" pos="0.07 0 0">
        <inertial pos="0.1 0 -0.005" mass="0.7" diaginertia="0.000361167 0.00202064 0.00226689"/>
        <joint type="hinge" axis="0 1 0"/>
        <joint type="hinge" axis="0 0 1"/>
        <geom type="capsule" fromto="0 0 0 0.2 0 -0.01" size="0.03" mass="0.7"/>
        <body name="l2" pos="0.2 0 -0.01">
          <inertial pos="0.1 0.01 -0.0075" mass="0.5" diaginertia="0.0002 0.0018 0.0019"/>
          <joint type="hinge" axis="0 1 0"/>
          <geom type="capsule" fromto="0 0 0 0.2 0.02 -0.015" size="0.03" mass="0.5"/>
          <body name="l3" pos="0.2 0.02 -0.015">
            <inertial pos="0 0 0" mass="0.3" diaginertia="0.0001 0.000125 0.00015"/>
            <joint type="hinge" axis="1 0 0"/>
            <geom type="sphere" size="0.035" mass="0.3"/>
          </body>
        </body>
      </body>
    </body>
    <body name="ball2" pos="-0.4 0.1 0.049">
      <inertial pos="0 0 0" mass="0.8" diaginertia="0.00089568 0.00101225 0.00128392"/>
      <freejoint/>
      <geom type="sphere" size="0.05" mass="0.8"/>
    </body>
  </worldbody>
</mujoco>"""


def inertial_chain_orders(m, d):
    """Whether some body's translational weight, some body's rotational weight
    and some free joint's rotational dofs' weight round otherwise as
    (a + (b + c))/3 than as MuJoCo's (a + b + c)/3, with MuJoCo's value the
    latter (`engine_setconst.c:648-649`, `:700-704`)."""
    def split(t, have):
        return (t[0] + t[1] + t[2]) / 3 != (t[0] + (t[1] + t[2])) / 3 and (t[0] + t[1] + t[2]) / 3 == have
    tran = rot = dof = False
    for b in range(1, m.nbody):
        if m.body_weldid[b] == 0 or m.body_simple[b] == 2:
            continue
        jac = np.zeros((6, m.nv))
        mujoco.mj_jacBodyCom(m, d, jac[:3], jac[3:], b)
        tmp = np.zeros((6, m.nv))
        mujoco.mj_solveM(m, d, tmp, jac)
        a = [float(jac[i] @ tmp[i]) for i in range(6)]
        tran = tran or split(a[:3], m.body_invweight0[b][0])
        rot = rot or split(a[3:], m.body_invweight0[b][1])
    for j in range(m.njnt):
        if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            continue
        adr = m.jnt_dofadr[j]
        sel = np.zeros((3, m.nv))
        for k in range(3):
            sel[k, adr + 3 + k] = 1
        tmp = np.zeros((3, m.nv))
        mujoco.mj_solveM(m, d, tmp, sel)
        dof = dof or split([tmp[k, adr + 3 + k] for k in range(3)], m.dof_invweight0[adr + 3])
    return tran, rot, dof


def diag_approx():
    out = {}
    for doc, dz in DIAG_APPROX_DOCS.items():
        xml = census_doc(doc)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.qpos[2] += dz
        mujoco.mj_forward(m, d)
        assert d.nefc > 0, doc
        out[doc] = {"xml": xml, "dz": dz, "efc_diagApprox": floats(d.efc_diagApprox[:d.nefc]),
                    "efc_R": floats(d.efc_R[:d.nefc])}
    m = mujoco.MjModel.from_xml_string(EVERY_ROW_KIND)
    d = mujoco.MjData(m)
    d.qpos[:], d.qvel[:] = EVERY_ROW_KIND_STATE
    mujoco.mj_forward(m, d)
    kinds = {int(t) for t in d.efc_type[:d.nefc]}
    want = {mujoco.mjtConstraint.mjCNSTR_EQUALITY, mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF,
            mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON, mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT,
            mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC}
    if not {int(k) for k in want} <= kinds:
        sys.exit(f"every_row_kind: rows {sorted(kinds)}")
    out["every_row_kind"] = {"xml": EVERY_ROW_KIND, "qpos": EVERY_ROW_KIND_STATE[0],
                             "qvel": EVERY_ROW_KIND_STATE[1],
                             "efc_diagApprox": floats(d.efc_diagApprox[:d.nefc]),
                             "efc_R": floats(d.efc_R[:d.nefc])}
    xml, _, qvel = MIXED_SOLREF_ROWS["frictionloss"]
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    remade = [i for i in range(d.nefc)
              if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
              and d.efc_diagApprox[i] != m.dof_invweight0[d.efc_id[i]]]
    if not remade:
        sys.exit("frictionloss_rows: no row's diagApprox is not its dof's invweight0")
    out["frictionloss_rows"] = {"xml": xml, "qpos": floats(d.qpos), "qvel": qvel,
                                "efc_diagApprox": floats(d.efc_diagApprox[:d.nefc]),
                                "efc_R": floats(d.efc_R[:d.nefc])}
    for condim in (3, 6):
        xml = PYRAMIDAL_FRICTION.format(condim=condim)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.qvel[:] = PYRAMIDAL_FRICTION_QVEL
        mujoco.mj_forward(m, d)
        assert d.ncon == 1 and d.nefc == 2 * (condim - 1), (d.ncon, d.nefc)
        tran = m.body_invweight0[1][0]
        fri = d.contact[0].friction[0]
        if tran + fri * fri * tran == (1 + fri * fri) * tran:
            sys.exit(f"pyramidal_friction_{condim}: the two orders round alike")
        out[f"pyramidal_friction_{condim}"] = {
            "xml": xml, "qpos": floats(d.qpos), "qvel": PYRAMIDAL_FRICTION_QVEL,
            "efc_diagApprox": floats(d.efc_diagApprox[:d.nefc]), "efc_R": floats(d.efc_R[:d.nefc])}
    m = mujoco.MjModel.from_xml_string(TENDON_PAIR)
    d = mujoco.MjData(m)
    d.qpos[:] = TENDON_PAIR_QPOS
    mujoco.mj_forward(m, d)
    assert d.nefc == 1 and d.efc_type[0] == mujoco.mjtConstraint.mjCNSTR_EQUALITY, d.nefc
    out["tendon_pair"] = {"xml": TENDON_PAIR, "qpos": TENDON_PAIR_QPOS, "qvel": [0.0, 0.0],
                          "tendon_invweight0": floats(m.tendon_invweight0),
                          "efc_diagApprox": floats(d.efc_diagApprox[:d.nefc]),
                          "efc_R": floats(d.efc_R[:d.nefc])}
    m = mujoco.MjModel.from_xml_string(INERTIAL_CHAIN)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert d.ncon > 0, "the chain rests on the plane"
    if not all(inertial_chain_orders(m, d)):
        sys.exit("inertial_chain: an average rounds alike grouped either way")
    out["inertial_chain"] = {"xml": INERTIAL_CHAIN, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
                             "body_invweight0": floats(m.body_invweight0.flatten()),
                             "dof_invweight0": floats(m.dof_invweight0),
                             "efc_diagApprox": floats(d.efc_diagApprox[:d.nefc]),
                             "efc_R": floats(d.efc_R[:d.nefc])}
    return out


NAN_TURN_CUBE = ('<mesh name="m" vertex="-.1 -.1 -.1 .1 -.1 -.1 .1 .1 -.1 -.1 .1 -.1 -.1 -.1 .1 .1 -.1 .1 '
                 '.1 .1 .1 -.1 .1 .1" face="0 2 1 0 3 2 4 5 6 4 6 7 0 1 5 0 5 4 2 3 7 2 7 6 0 4 7 0 7 3 1 2 6 '
                 '1 6 5"/>')
NAN_TURN_GEOMS = {"mesh": '<geom type="mesh" mesh="m"/>',
                  "ellipsoid": '<geom type="ellipsoid" size="0.1 0.08 0.06"/>',
                  "box": '<geom type="box" size="0.1 0.1 0.1"/>'}


def nan_turn():
    """A free body over a plane turned by a NaN quaternion (qpos[3]): its
    contacts at a NaN distance make rows whose impedance (`efc_KBIP`'s third),
    R, D and diagApprox are NaN (`getimpedance`, `:1413-1461`, takes a NaN `x`
    to its curve; `mju_max` keeps a NaN). It refuses a case whose rows are not
    all NaN, or that makes none."""
    out = {}
    for name, geom in NAN_TURN_GEOMS.items():
        xml = (f'<mujoco><asset>{NAN_TURN_CUBE}</asset><worldbody><geom type="plane" size="5 5 0.1"/>'
               f'<body pos="0 0 0.09"><freejoint/>{geom}</body></worldbody></mujoco>')
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.qpos[3] = float("nan")
        mujoco.mj_forward(m, d)
        n = d.nefc
        rows = {"efc_imp": floats(d.efc_KBIP[:n, 2]), "efc_R": floats(d.efc_R[:n]),
                "efc_D": floats(d.efc_D[:n]), "efc_diagApprox": floats(d.efc_diagApprox[:n])}
        if n == 0 or not all(np.isnan(v).all() for v in rows.values()):
            sys.exit(f"nan_turn {name}: {n} rows, not all NaN")
        out[name] = {"xml": xml, "nefc": int(n), **rows}
    return out


def weld_box_xml(torquescale):
    return f"""<mujoco model="weld_box_ts">
  <option timestep="0.002"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.06 0.04" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality>
    <weld body1="box" solref="0.05 1" torquescale="{torquescale}"/>
  </equality>
</mujoco>"""


WELD_PAIR_XML = """<mujoco model="weld_pair_ts">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.06 0.04" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="0.3 0 1">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="0.5" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality>
    <weld body1="a" body2="b" solref="0.05 1" torquescale="0.3"/>
  </equality>
</mujoco>"""


def turned(axis, angle):
    a = np.array(axis, dtype=float)
    a /= np.linalg.norm(a)
    return [float(np.cos(angle / 2))] + [float(x) for x in np.sin(angle / 2) * a]


def weld_torquescale():
    """Each case: its free joint at quat_adr (the box, or the pair's second box) turned and spinning."""
    out = {}
    cases = [(f"box_ts_{ts}", weld_box_xml(ts), ts, 3) for ts in ("0", "0.5", "2")]
    cases.append(("pair_ts_0.3", WELD_PAIR_XML, "0.3", 10))
    for name, xml, ts, quat_adr in cases:
        m = mujoco.MjModel.from_xml_string(xml)
        assert m.neq == 1 and float(m.eq_data[0, 10]) == float(ts), m.eq_data[0]
        qpos = [float(x) for x in m.qpos0]
        qpos[quat_adr:quat_adr + 4] = turned([1, 2, 3], 0.2)
        # A free joint's angular velocity follows its 3 linear dofs: dofs (quat_adr - 3) / 7 * 6 + 3.
        dof = (quat_adr - 3) // 7 * 6 + 3
        qvel = [0.0] * m.nv
        qvel[dof:dof + 3] = [0.5, -0.3, 0.8]
        d = mujoco.MjData(m)
        d.qpos[:] = qpos
        d.qvel[:] = qvel
        traj = {}
        for step in range(1, 501):
            mujoco.mj_step(m, d)
            if step in (1, 10, 100, 500):
                traj[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
        stable(d, name)
        out[name] = {"xml": xml, "torquescale": float(ts), "qpos": qpos, "qvel": qvel, "traj": traj}
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    warned = []
    mujoco.set_mju_user_warning(warned.append)
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
        "mixed_solref_rows": mixed_solref_rows(warned),
        "impedance_edges": impedance_edges(),
        "elliptic_rows": elliptic_rows(),
        "impratio": impratio(),
        "impratio_noslip": impratio_noslip(),
        "pair_condim6": pair_condim6(),
        "aniso_rows": aniso_rows(),
        "diag_approx": diag_approx(),
        "nan_turn": nan_turn(),
        "weld_torquescale": weld_torquescale(),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "impedance.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
