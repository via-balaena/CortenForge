#!/usr/bin/env python3
"""Generate the constraint-reference golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_reference_rows_reference.py \\
        sim/L0/tests/assets/golden/reference_rows

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo takes every row's velocity as `J·qvel` (`mj_mulJacVec`, each row a
`mju_dot`) and its reference acceleration from it (`mj_referenceConstraint`,
`engine_core_constraint.c:2375-2387`), then `efc_b = J·qacc_smooth - aref`
(`mj_fwdConstraint`, `engine_forward.c:755-756`). The case, tendon_rows: six
bodies on vertical slides, two fixed tendons over all six, a tendon equality
between them, the second tendon's friction loss and its lower limit (its range
starts above its length at qpos0), and one slide's friction loss, the slides
along different axes under a gravity with three components, so qacc_smooth
differs between them. Every row's Jacobian is the tendons' coefficients or a
unit, so it is exact in any engine. After mj_forward at qpos0 with the qvel below, per row: efc_type,
efc_J (row-major), efc_pos, efc_vel, efc_aref and efc_b; and qacc_smooth.
tendon_rows_sparse: the same at jacobian="sparse", where each product is a
`mju_dotSparse` over the row's chain (`mju_mulMatVecSparse`); its efc_J is
written out dense from the rows' chains.
"""
import json
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

MASSES = [1.0, 2.0, 0.5, 3.0, 1.5, 0.75]
COEF0 = [0.3, -0.7, 1.1, 0.13, -0.29, 0.61]
COEF1 = [0.47, 0.19, -0.83, 0.71, 0.37, -0.53]
# A power of two scales exactly; this one keeps aref below J·qacc_smooth, so
# efc_b shows the order J·qacc_smooth is summed in.
QVEL = [v / 256 for v in [-0.15, 0.65, -0.75, -0.55, 0.25, 0.9]]
AXES = ["0 0 1", "1 0 0", "0 1 0", "0 0 -1", "-1 0 0", "0 -1 0"]


def tendon_rows_xml():
    friction = {2: ' frictionloss="0.2"'}
    bodies = "".join(
        f'<body name="b{i}" pos="{i} 0 1"><joint name="s{i}" type="slide" axis="{AXES[i]}"'
        f'{friction.get(i, "")}/>'
        f'<geom type="sphere" size="0.1" mass="{m}" contype="0" conaffinity="0"/></body>'
        for i, m in enumerate(MASSES))
    t0 = "".join(f'<joint joint="s{i}" coef="{c}"/>' for i, c in enumerate(COEF0))
    t1 = "".join(f'<joint joint="s{i}" coef="{c}"/>' for i, c in enumerate(COEF1))
    return f"""<mujoco model="tendon_rows">
  <option timestep="0.002" gravity="0.83 -2.1 -9.81"/>
  <worldbody>{bodies}</worldbody>
  <tendon>
    <fixed name="t0">{t0}</fixed>
    <fixed name="t1" frictionloss="0.3" limited="true" range="0.05 1">{t1}</fixed>
  </tendon>
  <equality><tendon tendon1="t0" tendon2="t1" polycoef="0 0.5 0 0 0"/></equality>
</mujoco>"""


def case(xml, qvel):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    n = d.nefc
    if mujoco.mj_isSparse(m):
        jac = [0.0] * (n * m.nv)
        for r in range(n):
            adr = d.efc_J_rowadr[r]
            for k in range(adr, adr + d.efc_J_rownnz[r]):
                jac[r * m.nv + d.efc_J_colind[k]] = d.efc_J[k]
    else:
        jac = d.efc_J.reshape(-1)[:n * m.nv]
    return {
        "xml": xml,
        "qvel": [float(x) for x in qvel],
        "nefc": int(n),
        "efc_type": [int(t) for t in d.efc_type[:n]],
        "efc_J": [float(x) for x in jac],
        "efc_pos": [float(x) for x in d.efc_pos[:n]],
        "efc_vel": [float(x) for x in d.efc_vel[:n]],
        "efc_aref": [float(x) for x in d.efc_aref[:n]],
        "efc_b": [float(x) for x in d.efc_b[:n]],
        "qacc_smooth": [float(x) for x in d.qacc_smooth],
    }


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    sparse = tendon_rows_xml().replace('timestep="0.002"', 'timestep="0.002" jacobian="sparse"', 1)
    cases = {"tendon_rows": case(tendon_rows_xml(), QVEL), "tendon_rows_sparse": case(sparse, QVEL)}
    want = [int(mujoco.mjtConstraint.mjCNSTR_EQUALITY), int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF),
            int(mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON), int(mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)]
    for name, c in cases.items():
        if c["efc_type"] != want:
            sys.exit(f"{name}: rows {c['efc_type']}, not {want}")
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "reference_rows.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
