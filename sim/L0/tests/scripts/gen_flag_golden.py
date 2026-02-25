#!/usr/bin/env python3
"""Generate golden-file .npy data for §41 runtime flag conformance tests.

Usage:
    uv pip install mujoco numpy
    uv run sim/L0/tests/scripts/gen_flag_golden.py

Outputs:
    sim/L0/tests/assets/golden/flags/baseline_qacc.npy
    sim/L0/tests/assets/golden/flags/disable_gravity_qacc.npy

Each .npy file contains a (10, nv) float64 array — qacc per step for 10 steps.
"""

from pathlib import Path

import mujoco
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR.parent / "assets" / "golden" / "flags"
MODEL_PATH = ASSETS_DIR / "flag_golden_test.xml"


def generate(model_path: Path, output_dir: Path) -> None:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    nsteps = 10

    # --- Baseline (no flags) ---
    mujoco.mj_resetData(model, data)
    baseline = np.zeros((nsteps, model.nv), dtype=np.float64)
    for i in range(nsteps):
        mujoco.mj_step(model, data)
        baseline[i] = data.qacc[:model.nv].copy()
    np.save(output_dir / "baseline_qacc.npy", baseline)
    print(f"  baseline_qacc.npy  shape={baseline.shape}")

    # --- DISABLE_GRAVITY ---
    mujoco.mj_resetData(model, data)
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_GRAVITY
    gravity_off = np.zeros((nsteps, model.nv), dtype=np.float64)
    for i in range(nsteps):
        mujoco.mj_step(model, data)
        gravity_off[i] = data.qacc[:model.nv].copy()
    np.save(output_dir / "disable_gravity_qacc.npy", gravity_off)
    print(f"  disable_gravity_qacc.npy  shape={gravity_off.shape}")

    # Reset flag for future runs
    model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)


if __name__ == "__main__":
    print(f"Model: {MODEL_PATH}")
    print(f"Output: {ASSETS_DIR}")
    generate(MODEL_PATH, ASSETS_DIR)
    print("Done.")
