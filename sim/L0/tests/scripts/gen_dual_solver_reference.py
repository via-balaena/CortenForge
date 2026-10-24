#!/usr/bin/env python3
"""Generate the dual solvers' golden data from the unfused MuJoCo 3.5.0 oracle:
`efc_AR`, PGS and noslip.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_dual_solver_reference.py \\
        sim/L0/tests/assets/golden/dual_solver

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's dual solvers, PGS (`mj_solPGS`, `engine_solver.c:316-530`) and noslip
(`mj_solNoSlip`, `:537-748`), read the rows' inverse inertia with their
regularization, `efc_AR = J·M⁻¹·J' + diag(R)`, which the position stage makes
whenever one of them runs (`mj_projectConstraint`, `engine_core_constraint.c:2140-2372`;
`mj_isDual`), dense or at the sparse size. PGS starts from the warmstart's forces
when their dual cost `f·b + ½·f·(AR·f)` is not positive, else from zero
(`warmstart`, `engine_forward.c:611-687`), and both end by mapping the forces to
the dofs and making the acceleration from them (`dualFinish`, `engine_solver.c:70-78`).

Every case is mixed_rows' scene (gen_primal_solver_reference.py: equality, friction
loss, limit and contact rows) or the tilted box (four contacts, each dof in every
row) or the chain (a free sphere, three links on four hinges, a second free sphere),
dense and at jacobian="sparse". Each records, after the case's forward, the
inputs the constraint stage hands its solvers (`M` on its pattern, `qLD`,
`qLDiagInv`, `qacc_smooth`, `qfrc_smooth`, the rows' `efc_J`, `efc_aref`,
`efc_b`, `efc_R`, `efc_D`, `qacc_warmstart`), MuJoCo's `efc_AR` (at the sparse
size its rows' `efc_AR_rownnz`, `efc_AR_rowadr`, `efc_AR_colind`), and the
solve's outputs (`efc_force`, `efc_state`, `qacc`, `qfrc_constraint`, each
iteration's statistics, `solver_niter`, `solver_nnz`), so a test can hand the
solvers MuJoCo's inputs and compare every output bit for bit.

- pgs: PGS over each scene. `warm`: after three steps from the case's start, so
  the warmstart is the last step's acceleration; `cold`: at the start, the
  warmstart zero; `zeroed`: the warmstart set far from the solution, so the
  warmstart's dual cost is positive and PGS starts from zero forces (the
  generator refuses the case otherwise); `nowarm`: the warmstart disabled.
- noslip: noslip_iterations 10 after PGS, Newton and CG, over mixed_rows'
  pyramidal and elliptic scenes and the box, warm.
- nnz: the primal solver's `solver_nnz` after a forward (`engine_solver.c:1953-1964`),
  Newton and CG over mixed_rows at condim 3 and 6, pyramidal and elliptic,
  dense and sparse, islands disabled and enabled (MuJoCo's count per island
  and their sum), the box and the chain under Newton at the sparse size.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402
from gen_primal_solver_reference import (  # noqa: E402
    CHAIN, MIXED_QVEL, MIXED_ROWS, NEWTON_BOX, ROLLING_QVEL, census_start, floats, stats, with_islands)


def mixed(solver, cone, condim, sparse=False, options=""):
    xml = MIXED_ROWS.format(solver=solver, cone=cone, condim=condim)
    extra = (' jacobian="sparse"' if sparse else "") + options
    return xml.replace(f'solver="{solver}"', f'solver="{solver}"{extra}')


def box(solver, sparse=False, options=""):
    xml = NEWTON_BOX.format(iterations=100)
    extra = f' solver="{solver}"' + (' jacobian="sparse"' if sparse else "") + options
    return xml.replace('iterations="100"', 'iterations="100"' + extra)


def chain(solver, cone, condim, sparse=True):
    xml = CHAIN.format(cone=cone, condim=condim).replace('solver="Newton"', f'solver="{solver}"')
    return xml if sparse else xml.replace('jacobian="sparse"', 'jacobian="dense"')


def ar_of(m, d):
    """MuJoCo's efc_AR: the dense nefc×nefc matrix, or at the sparse size each
    row's columns and values."""
    n = d.nefc
    if mujoco.mj_isSparse(m):
        rownnz = [int(x) for x in d.efc_AR_rownnz[:n]]
        rowadr = [int(x) for x in d.efc_AR_rowadr[:n]]
        cols, vals = [], []
        for r in range(n):
            cols += [int(x) for x in d.efc_AR_colind[rowadr[r]:rowadr[r] + rownnz[r]]]
            vals += floats(d.efc_AR[rowadr[r]:rowadr[r] + rownnz[r]])
        return {"efc_AR_rownnz": rownnz, "efc_AR_rowadr": rowadr,
                "efc_AR_colind": cols, "efc_AR": vals, "nA": int(d.nA)}
    assert d.nA == n * n, (d.nA, n)
    return {"efc_AR": floats(d.efc_AR[:n * n]), "nA": int(d.nA)}


def inputs_of(m, d):
    n = d.nefc
    out = {"M": floats(d.M), "qLD": floats(d.qLD), "qLDiagInv": floats(d.qLDiagInv),
           "M_rowadr": [int(x) for x in m.M_rowadr], "M_rownnz": [int(x) for x in m.M_rownnz],
           "M_colind": [int(x) for x in m.M_colind],
           "qacc_smooth": floats(d.qacc_smooth), "qfrc_smooth": floats(d.qfrc_smooth),
           "qacc_warmstart": floats(d.qacc_warmstart),
           "efc_aref": floats(d.efc_aref[:n]), "efc_b": floats(d.efc_b[:n]),
           "efc_D": floats(d.efc_D[:n]), "efc_R": floats(d.efc_R[:n]),
           "efc_type": [int(x) for x in d.efc_type[:n]], "efc_id": [int(x) for x in d.efc_id[:n]],
           "ne": int(d.ne), "nf": int(d.nf),
           # Each contact's regularized cone mu, which MuJoCo makes from its own R.
           "contact_mu": [float(d.contact[k].mu) for k in range(d.ncon)]}
    if mujoco.mj_isSparse(m):
        nnz = int(sum(d.efc_J_rownnz[:n]))
        out.update({"efc_J_rownnz": [int(x) for x in d.efc_J_rownnz[:n]],
                    "efc_J_rowadr": [int(x) for x in d.efc_J_rowadr[:n]],
                    "efc_J_colind": [int(x) for x in d.efc_J_colind[:nnz]],
                    "efc_J": floats(d.efc_J[:nnz])})
    else:
        out["efc_J"] = floats(d.efc_J[:n * m.nv])
    return out


def outputs_of(m, d):
    n = d.nefc
    return {"efc_force": floats(d.efc_force[:n]), "efc_state": [int(x) for x in d.efc_state[:n]],
            "qacc": floats(d.qacc), "qfrc_constraint": floats(d.qfrc_constraint),
            "stats": stats(m, d), "solver_nnz": int(d.solver_nnz[0])}


def warm_cost(m, d):
    """The warmstart's PGS dual cost as `warmstart` makes it, on a copy."""
    c = mujoco.MjData(m)
    c.qpos[:], c.qvel[:], c.qacc_warmstart[:] = d.qpos, d.qvel, d.qacc_warmstart
    mujoco.mj_forward(m, c)
    n = c.nefc
    jar = np.zeros(n)
    mujoco.mj_mulJacVec(m, c, jar, c.qacc_warmstart)
    jar -= c.efc_aref[:n]
    cost = np.zeros(1)
    mujoco.mj_constraintUpdate(m, c, jar, cost, 0)
    f = c.efc_force[:n].copy()
    if mujoco.mj_isSparse(m):
        arf = np.zeros(n)
        for r in range(n):
            a, k = c.efc_AR_rowadr[r], c.efc_AR_rownnz[r]
            arf[r] = sum(c.efc_AR[a + j] * f[c.efc_AR_colind[a + j]] for j in range(k))
    else:
        arf = c.efc_AR[:n * n].reshape(n, n) @ f
    return float(f @ c.efc_b[:n] + 0.5 * f @ arf)


def solve(xml, qvel=None, start=None, warm_steps=0, warmstart=None):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if start is not None:
        start(m, d)
    else:
        d.qvel[:] = qvel
    for _ in range(warm_steps):
        mujoco.mj_step(m, d)
    if warmstart is not None:
        d.qacc_warmstart[:] = warmstart(m, d)
    case = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    mujoco.mj_forward(m, d)
    assert mujoco.mj_isDual(m), "a dual solver runs"
    case.update(inputs_of(m, d))
    case.update(ar_of(m, d))
    case.update(outputs_of(m, d))
    case["nefc"] = int(d.nefc)
    return m, d, case


def pgs():
    scenes = {}
    for cone, condim in (("pyramidal", 3), ("elliptic", 3), ("elliptic", 4), ("elliptic", 6)):
        for sparse in (False, True):
            tag = f"mixed_{cone}_{condim}" + ("_sparse" if sparse else "")
            scenes[tag] = (mixed("PGS", cone, condim, sparse), MIXED_QVEL, None)
    scenes["mixed_elliptic_6_rolling"] = (mixed("PGS", "elliptic", 6), ROLLING_QVEL, None)
    for sparse in (False, True):
        scenes["box" + ("_sparse" if sparse else "")] = (box("PGS", sparse), None, census_start)
    for cone, condim in (("pyramidal", 3), ("elliptic", 3)):
        for sparse in (False, True):
            tag = f"chain_{cone}_{condim}" + ("_sparse" if sparse else "")
            rng = np.random.default_rng(7)
            xml = chain("PGS", cone, condim, sparse)
            nv = mujoco.MjModel.from_xml_string(xml).nv
            scenes[tag] = (xml, list(rng.normal(0, 0.8, nv)), None)
    out, branches = {}, {"kept": 0, "zeroed": 0}
    for tag, (xml, qvel, start) in scenes.items():
        for run, kw in (("warm", {"warm_steps": 3}), ("cold", {}),
                        ("zeroed", {"warm_steps": 3, "warmstart": lambda m, d: -50 * d.qacc_warmstart})):
            m, d, case = solve(xml, qvel, start, **kw)
            cost = warm_cost(m, d)
            case["warm_cost"] = cost
            if run == "zeroed":
                assert cost > 0, f"{tag}: the warmstart's cost {cost} is not positive"
            branches["zeroed" if cost > 0 else "kept"] += 1
            out[f"{tag}_{run}"] = case
        nowarm = xml.replace("<flag island=", '<flag warmstart="disable" island=')
        assert nowarm != xml
        out[f"{tag}_nowarm"] = solve(nowarm, qvel, start, warm_steps=3)[2]
    assert branches["kept"] and branches["zeroed"], branches
    return out


def noslip():
    out = {}
    for solver in ("PGS", "Newton", "CG"):
        opt = ' noslip_iterations="10"'
        for cone in ("pyramidal", "elliptic"):
            for sparse in (False, True):
                tag = f"{solver}_mixed_{cone}" + ("_sparse" if sparse else "")
                out[tag] = solve(mixed(solver, cone, 3, sparse, opt), MIXED_QVEL, warm_steps=3)[2]
        out[f"{solver}_box"] = solve(box(solver, False, opt), None, census_start, warm_steps=3)[2]
    return out


def nnz():
    """The primal solver's solver_nnz: Newton's Hessian at the sparse size
    (its factor's entries, twice under an elliptic cone, and the Hessian's
    count), nv*nv dense, CG's 0 (`engine_solver.c:1953-1964`), per island
    with islands enabled; the case keeps MuJoCo's list and its sum."""
    out = {}
    for solver in ("Newton", "CG"):
        for cone, condim in (("pyramidal", 3), ("elliptic", 3), ("elliptic", 6)):
            for sparse in (False, True):
                for islands in (False, True):
                    xml = mixed(solver, cone, condim, sparse)
                    xml = with_islands(xml) if islands else xml
                    m = mujoco.MjModel.from_xml_string(xml)
                    d = mujoco.MjData(m)
                    d.qvel[:] = MIXED_QVEL
                    mujoco.mj_forward(m, d)
                    n = max(1, min(int(d.nisland), 20))
                    per = [int(x) for x in d.solver_nnz[:n]]
                    tag = f"{solver}_{cone}_{condim}" + ("_sparse" if sparse else "") + ("_islands" if islands else "")
                    out[tag] = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
                                "nisland": int(d.nisland), "solver_nnz_per_island": per,
                                "solver_nnz": sum(per)}
    for tag, xml in (("Newton_box_sparse", box("Newton", True)), ("Newton_chain_elliptic", chain("Newton", "elliptic", 3))):
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        if "chain" in tag:
            d.qvel[:] = list(np.random.default_rng(7).normal(0, 0.8, m.nv))
        else:
            census_start(m, d)
        mujoco.mj_forward(m, d)
        per = [int(d.solver_nnz[0])]
        out[tag] = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel), "nisland": int(d.nisland),
                    "solver_nnz_per_island": per, "solver_nnz": sum(per)}
    assert any(c["nisland"] > 1 for c in out.values()), "a case with islands"
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {"pgs": pgs(), "noslip": noslip(), "nnz": nnz()}
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    os.makedirs(sys.argv[1], exist_ok=True)
    with open(os.path.join(sys.argv[1], "dual_solver.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{sum(len(v) for v in cases.values())} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
