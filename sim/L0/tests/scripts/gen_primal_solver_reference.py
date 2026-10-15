#!/usr/bin/env python3
"""Generate the primal solver golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_primal_solver_reference.py \\
        sim/L0/tests/assets/golden/primal_solver

It refuses any other interpreter, as gen_census_golden.py does.

Newton and CG are one solver in MuJoCo, `mj_solPrimal` (`engine_solver.c:1811-1968`),
started from `warmstart` (`engine_forward.c:611-687`). Every case disables islands
(`<flag island="disable"/>`), so MuJoCo solves all rows at once, scaled by
`1/(meaninertia·nv)`: the path sim-core runs. The cases:

- newton_box: census doc 406a1991 (`collision_plane.rs:1076`), a box on a tilted
  plane with no gravity, at the census start (qpos0, qvel_i = 0.1·(1 + i mod 3)):
  after mj_forward, each iteration's statistics, qacc and efc_force; and with
  `iterations=1`, the first iterate's qacc.
- cg_connect: a hinge arm and a free box joined by a connect, under CG: qpos and
  qvel after steps 1, 10, 100 and 500, and each step's iteration count.
- cg_sphere: census doc 443824ba (`noslip.rs:201`), a sphere on a plane, CG with an
  elliptic cone, at the census start: qpos and qvel after steps 1, 10 and 100,
  and each step's iteration count.
- no_rows: census doc 28695e44 (`collision_primitives.rs:802`), two stacked boxes
  under Newton, at the census start: nefc and the iteration count after each of
  the first 10 steps (MuJoCo clears the count when a step has no rows).
- few_iterations: a tilted box sliding and spinning on a plane, every bottom corner
  below it (ledger L81's matched fixture, without its sleeping arm: MuJoCo sleeps
  no tree with islands disabled): the box's qvel after one step under CG and
  Newton at 1 and 3 iterations.
- eulerdamp_asleep: ledger L71's fixture, few_iterations' box beside a damped arm
  that starts asleep, sleep and islands enabled (MuJoCo sleeps no tree without
  islands): the qvel after one step under CG at 1 and 3 iterations and Newton at 1.
  Euler's damped solve takes only the awake dofs' damping (`mj_EulerSkip`,
  `engine_forward.c:948-962`), so it does not run.
- mixed_rows: an arm on a limited hinge with friction loss, a ball joined to its tip
  by a connect, and a sphere sliding and spinning on a plane, so every kind of row is
  present (equality, friction loss, limit, contact); Newton and CG, pyramidal and
  elliptic cones (condim 3 and 6): qpos and qvel after steps 1, 10, 50 and 200,
  each step's iteration count, and the first forward's statistics. (A sphere, not a
  box: sim-core's box-plane collision keeps a corner above the plane that MuJoCo
  drops, ledger L41 / book P43.)
- warmstart_islands: a pendulum swinging free beside a sphere sliding and spinning on
  a plane, noslip on, so MuJoCo makes the islands (the sphere's; the pendulum is in
  none) but solves every row at once (`islands_supported` needs no noslip,
  `engine_forward.c:762-765`), from a warmstart that starts the pendulum's dof at
  `qacc_smooth` (`:670-676`); Newton and CG: qpos and qvel after steps 1, 10, 50 and
  100.
- noslip_rows: mixed_rows' scene at condim 3, pyramidal and elliptic, Newton and CG, with
  noslip_iterations 10: noslip sweeps a friction-loss row and a contact's friction rows,
  counts its sweeps in `solver_niter`, appends their statistics after the solver's and
  leaves `efc_state` as `dualState` (`engine_solver.c:556`, `:716-741`); the first
  forward's statistics and rows' states, qpos and qvel after steps 1, 10, 50 and 200,
  and each step's iteration count. Under the elliptic cone the puck starts at a tenth of
  its speed, so its friction stays inside the cone (by 0.84 of mu*N, measured): sliding,
  noslip leaves it on the cone's edge, where `dualState`'s zone is set by the last bit
  (MuJoCo's own Newton run puts it in the cone and its CG run outside). And a hinge held
  at its limit with noslip on and no friction row: MuJoCo still sweeps (twice: the first
  sweep's improvement is the cost correction over every row, `engine_solver.c:563-567`).
  And the pyramidal scene with the puck slow and the solver stopped after one iteration:
  noslip then changes rows' states on its first sweep (MuJoCo's nchange 1 under Newton,
  2 under CG), which it does in no other case here.
- sparse_spheres: ten free spheres sliding and spinning on a plane under Newton with
  an elliptic cone, 60 dofs: MuJoCo's sparse Hessian (`mj_isSparse`, nv >= 60). qpos
  and qvel after steps 1, 10 and 50, and each step's iteration count.
- sparse_pair and sparse_stack: ten spheres again, two of them in contact with each
  other (sparse_pair: the second overlaps the first by 1 mm, sliding along y and
  spinning; sparse_stack: the second rests on the first, which slides along x), so an
  elliptic contact couples two trees and its cone term pairs columns of different rows:
  the first forward's statistics, qpos and qvel after steps 1, 10, 50 and 100, and each
  step's iteration count. The generator refuses either unless that contact is in the
  cone at the first forward.
- same_input: for newton_box, cg_connect and each mixed_rows variant (the condim-6
  ones also with the sphere rolling about x and y, so all five tangential components
  are nonzero), the first forward's solver inputs as mj_fwdConstraint hands them to
  the solver (`d.M` on the mass matrix's pattern and its factor `d.qLD`,
  `d.qLDiagInv`, `qacc_smooth`, `qfrc_smooth`, the dense `efc_J`, `efc_aref`,
  `efc_D`, `efc_R`) and its outputs (`qacc`, `efc_force`, each iteration's
  statistics), so a test can hand the solver MuJoCo's own inputs and compare its
  outputs bit for bit, whatever the stages before it round. The elliptic condim-3
  variant also runs under CG with `ls_iterations` 1, and under CG and Newton with
  `tolerance` 1e-300 and iterations capped at 40: a mutant of the line search's exit
  at its iteration cap is caught by the first, one of its midpoint exit by CG's
  second. (sim-mjcf refuses a `tolerance` of 0, which MuJoCo takes.)
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

NEWTON_BOX = """<mujoco model="box_plane_tilted_4">
  <option gravity="0 0 0" timestep="0.001" iterations="{iterations}">
    <flag island="disable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" euler="10 0 0"/>
    <body name="box" pos="0 0 0.2">
      <joint type="free"/>
      <geom type="box" size="0.3 0.3 0.3"/>
    </body>
  </worldbody>
</mujoco>"""

CG_CONNECT = """<mujoco model="conn_free2_cg">
  <option timestep="0.002" solver="CG">
    <flag island="disable"/>
  </option>
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

CG_SPHERE = """<mujoco model="cg_noop">
  <option gravity="0 0 -9.81" solver="CG" cone="elliptic" noslip_iterations="0">
    <flag island="disable"/>
  </option>
  <worldbody>
    <body pos="0 0 0.15">
      <joint type="free"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
    <geom type="plane" size="5 5 0.1"/>
  </worldbody>
</mujoco>"""

NO_ROWS = """<mujoco model="box_stack_stable">
  <option gravity="0 0 -9.81" solver="Newton" cone="elliptic" iterations="100" tolerance="1e-8">
    <flag island="disable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="box1" pos="0 0 0.11">
      <joint type="free"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" friction="0.5 0.5 0.005"/>
    </body>
    <body name="box2" pos="0 0 0.31">
      <joint type="free"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" friction="0.5 0.5 0.005"/>
    </body>
  </worldbody>
</mujoco>"""

FEW_ITERATIONS = """<mujoco model="few_iterations">
  <option timestep="0.002" solver="{solver}" iterations="{iterations}">
    <flag island="disable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="box" pos="0 0 0.09" euler="2 1 0">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>"""

WARMSTART_ISLANDS = """<mujoco model="warmstart_islands">
  <option timestep="0.002" solver="{solver}" cone="elliptic" noslip_iterations="10"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body pos="0 0 0.0995">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" friction="0.5 0.005 0.0001"/>
    </body>
    <body pos="2 0 1">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 -0.3" size="0.03" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""
WARMSTART_QVEL = [0.8, 0, 0, 0, 0, 3.0, 1.5]

SPHERE = """    <body pos="{x} {y} 0.0995">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" friction="0.6 0.01 0.001" condim="{condim}"/>
    </body>"""

SPARSE_SPHERES = """<mujoco model="sparse_spheres">
  <option timestep="0.002" solver="Newton" cone="elliptic">
    <flag island="disable"/>
  </option>
  <worldbody>
    <geom type="plane" size="10 10 0.1"/>
{bodies}
  </worldbody>
</mujoco>""".format(bodies="\n".join(
    SPHERE.format(x=0.5 * (i % 5), y=0.5 * (i // 5), condim=3) for i in range(10)))
# Each sphere slides along x at 0.3·(1 + i mod 3) and spins about z.
SPARSE_QVEL = [v for i in range(10) for v in (0.3 * (1 + i % 3), 0.1, 0, 0, 0, 2.0 + i)]

TOP = """    <body pos="0 0 0.2985">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" condim="3"/>
    </body>"""
SPARSE_OTHERS = [SPHERE.format(x=1.0 + 0.5 * (i % 4), y=0.5 * (i // 4), condim=3) for i in range(8)]
TWO_TREES = """<mujoco model="{name}">
  <option timestep="0.002" solver="Newton" cone="elliptic">
    <flag island="disable"/>
  </option>
  <worldbody>
    <geom type="plane" size="10 10 0.1"/>
{bodies}
  </worldbody>
</mujoco>"""
SPARSE_PAIR_XML = TWO_TREES.format(name="sparse_pair", bodies="\n".join(
    [SPHERE.format(x=0.0, y=0.0, condim=3), SPHERE.format(x=0.199, y=0.0, condim=3)] + SPARSE_OTHERS))
# The second sphere slides along y and spins about z against the first; the others
# slide on the plane.
SPARSE_PAIR_QVEL = [0, 0, 0, 0, 0, 0, 0, 0.8, 0, 0, 0, 5.0] + [v for _ in range(8) for v in (0.3, 0.1, 0, 0, 0, 2.0)]
SPARSE_STACK_XML = TWO_TREES.format(name="sparse_stack", bodies="\n".join(
    [SPHERE.format(x=0.0, y=0.0, condim=3), TOP] + SPARSE_OTHERS))
# The lower sphere slides along x under the upper one.
SPARSE_STACK_QVEL = [0.5, 0, 0, 0, 0, 0] + [0.0] * 54

EULERDAMP_ASLEEP = """<mujoco model="eulerdamp_asleep">
  <option timestep="0.002" solver="{solver}" iterations="{iterations}">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="box" pos="0 0 0.09" euler="2 1 0">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
    <body name="arm" pos="2 0 1" sleep="init">
      <joint name="h" type="hinge" axis="0 1 0" damping="5"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""

MIXED_ROWS = """<mujoco model="mixed_rows">
  <option timestep="0.002" solver="{solver}" cone="{cone}">
    <flag island="disable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="arm" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0" limited="true" range="-0.3 0.3"
             frictionloss="0.5" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="ball" pos="0.3 0 1">
      <freejoint/>
      <geom type="sphere" size="0.05" mass="0.5" contype="0" conaffinity="0"/>
    </body>
    <body name="puck" pos="1 0 0.0995">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" friction="0.8 0.02 0.002" condim="{condim}"/>
    </body>
  </worldbody>
  <equality>
    <connect body1="arm" body2="ball" anchor="0.3 0 0"/>
  </equality>
</mujoco>"""

# The arm swings into its limit; the sphere slides along x and spins about z.
MIXED_QVEL = [2.0, 0, 0, 0, 0, 0, 0, 0.5, 0.2, 0, 0, 0, 3.0]
# ... and rolls about x and y too, so a condim-6 contact's five tangential
# components are all nonzero.
ROLLING_QVEL = [2.0, 0, 0, 0, 0, 0, 0, 0.5, 0.2, 0, 1.0, -0.7, 3.0]
# ... and with the sphere at a tenth of that speed, so its friction holds inside the cone.
NOSLIP_STICK_QVEL = [2.0, 0, 0, 0, 0, 0, 0, 0.05, 0.02, 0, 0, 0, 3.0]

NOSLIP_WITHOUT_ITS_ROWS = """<mujoco model="noslip_without_its_rows">
  <option timestep="0.002" solver="{solver}" noslip_iterations="10"/>
  <worldbody>
    <body pos="0 0 1">
      <joint type="hinge" axis="0 1 0" limited="true" range="-0.3 -0.1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def stats(m, d):
    n = int(d.solver_niter[0])
    keys = ("improvement", "gradient", "lineslope", "nactive", "nchange", "neval", "nupdate")
    out = {k: [] for k in keys}
    for s in d.solver[:n]:
        out["improvement"].append(float(s.improvement))
        out["gradient"].append(float(s.gradient))
        out["lineslope"].append(float(s.lineslope))
        out["nactive"].append(int(s.nactive))
        out["nchange"].append(int(s.nchange))
        out["neval"].append(int(s.neval))
        out["nupdate"].append(int(s.nupdate))
    out["niter"] = n
    return out


def census_start(m, d):
    d.qpos[:] = m.qpos0
    for i in range(m.nv):
        d.qvel[i] = 0.1 * (1 + i % 3)


def newton_box():
    m = mujoco.MjModel.from_xml_string(NEWTON_BOX.format(iterations=100))
    d = mujoco.MjData(m)
    census_start(m, d)
    mujoco.mj_forward(m, d)
    out = {"xml": NEWTON_BOX.format(iterations=100), "qvel": floats(d.qvel),
           "meaninertia": float(m.stat.meaninertia), "nefc": int(d.nefc),
           "stats": stats(m, d), "qacc": floats(d.qacc), "efc_force": floats(d.efc_force[:d.nefc])}
    m1 = mujoco.MjModel.from_xml_string(NEWTON_BOX.format(iterations=1))
    d1 = mujoco.MjData(m1)
    census_start(m1, d1)
    mujoco.mj_forward(m1, d1)
    out["one_iteration"] = {"xml": NEWTON_BOX.format(iterations=1), "niter": int(d1.solver_niter[0]),
                            "qacc": floats(d1.qacc)}
    return out


def trajectory(xml, qvel, nstep, checkpoints, start=None):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if start is not None:
        start(m, d)
    else:
        d.qpos[:] = m.qpos0
        d.qvel[:] = qvel
    out = {"xml": xml, "qvel": floats(d.qvel), "niter": [], "traj": {}}
    for k in range(1, nstep + 1):
        mujoco.mj_step(m, d)
        out["niter"].append(int(d.solver_niter[0]))
        if k in checkpoints:
            out["traj"][str(k)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    return out


def no_rows():
    m = mujoco.MjModel.from_xml_string(NO_ROWS)
    d = mujoco.MjData(m)
    census_start(m, d)
    out = {"xml": NO_ROWS, "qvel": floats(d.qvel), "nefc": [], "niter": []}
    for _ in range(10):
        mujoco.mj_step(m, d)
        out["nefc"].append(int(d.nefc))
        out["niter"].append(int(d.solver_niter[0]))
    return out


def few_iterations(template=FEW_ITERATIONS, runs=(("CG", 1), ("CG", 3), ("Newton", 1), ("Newton", 3))):
    out = {}
    for solver, it in runs:
        xml = template.format(solver=solver, iterations=it)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        d.qvel[:6] = [1.0, 0.3, 0.0, 0.0, 0.0, 4.0]
        qvel0 = floats(d.qvel)
        mujoco.mj_step(m, d)
        out[f"{solver}{it}"] = {"xml": xml, "qvel0": qvel0, "ncon": int(d.ncon),
                                "niter": int(d.solver_niter[0]), "qvel": floats(d.qvel)}
    return out


def mixed_rows():
    out = {}
    for solver in ("Newton", "CG"):
        for cone, condim in (("pyramidal", 3), ("elliptic", 3), ("elliptic", 6)):
            xml = MIXED_ROWS.format(solver=solver, cone=cone, condim=condim)
            case = trajectory(xml, MIXED_QVEL, 200, (1, 10, 50, 200))
            m = mujoco.MjModel.from_xml_string(xml)
            d = mujoco.MjData(m)
            d.qvel[:] = MIXED_QVEL
            mujoco.mj_forward(m, d)
            case["forward"] = {"stats": stats(m, d), "qacc": floats(d.qacc), "nefc": int(d.nefc),
                               "efc_type": [int(t) for t in d.efc_type[:d.nefc]]}
            out[f"{solver}_{cone}_{condim}"] = case
    return out


def noslip_case(xml, qvel):
    case = trajectory(xml, qvel, 200, (1, 10, 50, 200))
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    case["forward"] = {"stats": stats(m, d), "nefc": int(d.nefc),
                       "efc_state": [int(x) for x in d.efc_state[:d.nefc]]}
    return case


def noslip_rows():
    out = {}
    for solver in ("Newton", "CG"):
        def mixed(cone, options=""):
            return MIXED_ROWS.format(solver=solver, cone=cone, condim=3).replace(
                f'solver="{solver}"', f'solver="{solver}" noslip_iterations="10"{options}')
        out[f"{solver}_pyramidal"] = noslip_case(mixed("pyramidal"), MIXED_QVEL)
        out[f"{solver}_elliptic"] = noslip_case(mixed("elliptic"), NOSLIP_STICK_QVEL)
        out[f"{solver}_pyramidal_one_iteration"] = noslip_case(
            mixed("pyramidal", ' iterations="1"'), NOSLIP_STICK_QVEL)
        out[f"{solver}_limit_only"] = noslip_case(NOSLIP_WITHOUT_ITS_ROWS.format(solver=solver), [0.5])
    return out


def same_input(xml, qvel, start=None):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if start is not None:
        start(m, d)
    else:
        d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    n = d.nefc
    assert not mujoco.mj_isSparse(m), "a dense case"
    return {"xml": xml, "qvel": floats(d.qvel), "newton": int(m.opt.solver) == 2,
            "M": floats(d.M), "qLD": floats(d.qLD), "qLDiagInv": floats(d.qLDiagInv),
            "M_rowadr": [int(x) for x in m.M_rowadr], "M_rownnz": [int(x) for x in m.M_rownnz],
            "M_colind": [int(x) for x in m.M_colind],
            "qacc_smooth": floats(d.qacc_smooth), "qfrc_smooth": floats(d.qfrc_smooth),
            "efc_J": floats(d.efc_J[:n * m.nv]), "efc_aref": floats(d.efc_aref[:n]),
            "efc_D": floats(d.efc_D[:n]), "efc_R": floats(d.efc_R[:n]),
            "qacc": floats(d.qacc), "efc_force": floats(d.efc_force[:n]), "stats": stats(m, d)}


def two_tree_cone(xml, qvel):
    """trajectory() of a sparse case whose first forward has a contact between two
    bodies in the cone, with that forward's statistics; refuses any other."""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    assert mujoco.mj_isSparse(m) and m.nv >= 60, "MuJoCo's sparse path"
    cone = int(mujoco.mjtConstraintState.mjCNSTRSTATE_CONE)
    coupled = [
        i for i in range(d.nefc)
        if d.efc_type[i] == int(mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC)
        and d.efc_state[i] == cone
        and all(m.geom_bodyid[g] > 0 for g in d.contact[d.efc_id[i]].geom)
    ]
    assert coupled, "a contact between two bodies in the cone at the first forward"
    case = trajectory(xml, qvel, 100, (1, 10, 50, 100))
    case["forward"] = {"stats": stats(m, d), "nefc": int(d.nefc)}
    return case


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "newton_box": newton_box(),
        "cg_connect": trajectory(CG_CONNECT, None, 500, (1, 10, 100, 500), census_start),
        "cg_sphere": trajectory(CG_SPHERE, None, 100, (1, 10, 100), census_start),
        "no_rows": no_rows(),
        "few_iterations": few_iterations(),
        "warmstart_islands": {solver: trajectory(WARMSTART_ISLANDS.format(solver=solver), WARMSTART_QVEL,
                                                   100, (1, 10, 50, 100))
                              for solver in ("Newton", "CG")},
        "sparse_spheres": trajectory(SPARSE_SPHERES, SPARSE_QVEL, 50, (1, 10, 50)),
        "sparse_pair": two_tree_cone(SPARSE_PAIR_XML, SPARSE_PAIR_QVEL),
        "sparse_stack": two_tree_cone(SPARSE_STACK_XML, SPARSE_STACK_QVEL),
        "eulerdamp_asleep": few_iterations(EULERDAMP_ASLEEP, (("CG", 1), ("CG", 3), ("Newton", 1))),
        "mixed_rows": mixed_rows(),
        "noslip_rows": noslip_rows(),
    }
    same = {"newton_box": same_input(NEWTON_BOX.format(iterations=100), None, census_start),
            "cg_connect": same_input(CG_CONNECT, None, census_start)}
    for solver in ("Newton", "CG"):
        for cone, condim in (("pyramidal", 3), ("elliptic", 3), ("elliptic", 6)):
            same[f"{solver}_{cone}_{condim}"] = same_input(
                MIXED_ROWS.format(solver=solver, cone=cone, condim=condim), MIXED_QVEL)
        same[f"{solver}_elliptic_6_rolling"] = same_input(
            MIXED_ROWS.format(solver=solver, cone="elliptic", condim=6), ROLLING_QVEL)
    for solver, tag, opt in (("CG", "ls1", 'ls_iterations="1"'),
                             ("CG", "tiny_tol", 'tolerance="1e-300" iterations="40"'),
                             ("Newton", "tiny_tol", 'tolerance="1e-300" iterations="40"')):
        xml = MIXED_ROWS.format(solver=solver, cone="elliptic", condim=3).replace(
            f'solver="{solver}"', f'solver="{solver}" {opt}')
        same[f"{solver}_elliptic_3_{tag}"] = same_input(xml, MIXED_QVEL)
    cases["same_input"] = same
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "primal_solver.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
