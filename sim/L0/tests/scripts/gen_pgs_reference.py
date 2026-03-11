#!/usr/bin/env python3
"""Generate PGS solver conformance reference data for Phase 13 Spec B T7.

Usage:
    uv pip install mujoco==3.4.0 numpy
    uv run sim/L0/tests/scripts/gen_pgs_reference.py

Outputs .npy files in sim/L0/tests/assets/golden/pgs_conformance/:
    pgs_efc_force.npy     — (nefc,) float64 — constraint forces after 1 step
    pgs_qacc.npy          — (nv,)  float64 — qacc after 1 step
    pgs_solver_niter.npy  — ()     int64   — solver iteration count after 1 step
"""

from pathlib import Path

import mujoco
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR.parent / "assets" / "golden" / "pgs_conformance"
MODEL_PATH = ASSETS_DIR / "pgs_test_model.xml"


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    # Verify solver is PGS
    assert model.opt.solver == mujoco.mjtSolver.mjSOL_PGS, (
        f"Expected PGS solver, got {model.opt.solver}"
    )

    # Step once
    mujoco.mj_step(model, data)

    # Dump reference data
    efc_force = data.efc_force.copy()
    qacc = data.qacc.copy()
    solver_niter = np.int64(data.solver_niter[0])

    print(f"Model: {MODEL_PATH.name}")
    print(f"nv = {model.nv}, nefc = {data.nefc}")
    print(f"solver_niter = {solver_niter}")
    print(f"efc_force = {efc_force}")
    print(f"qacc = {qacc}")

    np.save(ASSETS_DIR / "pgs_efc_force.npy", efc_force)
    np.save(ASSETS_DIR / "pgs_qacc.npy", qacc)
    np.save(ASSETS_DIR / "pgs_solver_niter.npy", solver_niter)

    print(f"\nSaved reference data to {ASSETS_DIR}")


if __name__ == "__main__":
    main()
