#!/usr/bin/env python3
"""Generate golden-file .npy data for DT-97 flag conformance tests.

Usage:
    uv pip install mujoco==3.4.0 numpy
    uv run sim/L0/tests/scripts/gen_flag_golden.py

Outputs 26 .npy files in sim/L0/tests/assets/golden/flags/:
    baseline_qacc.npy              — no flags set
    disable_{name}_qacc.npy        — 19 disable flags (one file each)
    enable_{name}_qacc.npy         — 6 enable flags (one file each)

Each .npy file contains a (10, nv) float64 array — qacc per step for 10 steps.
All simulations set ctrl[0]=2.0 (outside ctrlrange [-1,1]) so DISABLE_CLAMPCTRL
has observable effect.
"""

from pathlib import Path

import mujoco
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR.parent / "assets" / "golden" / "flags"
MODEL_PATH = ASSETS_DIR / "flag_golden_test.xml"

NSTEPS = 10
CTRL_VALUE = 2.0  # Outside ctrlrange [-1, 1] for DISABLE_CLAMPCTRL testing

# Disable flags — bit positions from MuJoCo mjmodel.h (mjtDisableBit).
# Using integer values directly for robustness across MuJoCo Python versions.
DISABLE_FLAGS = [
    ("constraint",    1 << 0),
    ("equality",      1 << 1),
    ("frictionloss",  1 << 2),
    ("limit",         1 << 3),
    ("contact",       1 << 4),
    ("spring",        1 << 5),
    ("damper",        1 << 6),
    ("gravity",       1 << 7),
    ("clampctrl",     1 << 8),
    ("warmstart",     1 << 9),
    ("filterparent",  1 << 10),
    ("actuation",     1 << 11),
    ("refsafe",       1 << 12),
    ("sensor",        1 << 13),
    ("midphase",      1 << 14),
    ("eulerdamp",     1 << 15),
    ("autoreset",     1 << 16),
    ("nativeccd",     1 << 17),
    ("island",        1 << 18),
]

# Enable flags — bit positions from MuJoCo mjmodel.h (mjtEnableBit).
ENABLE_FLAGS = [
    ("override",     1 << 0),
    ("energy",       1 << 1),
    ("fwdinv",       1 << 2),
    ("invdiscrete",  1 << 3),
    ("multiccd",     1 << 4),
    ("sleep",        1 << 5),
]


def run_simulation(model, data, nsteps):
    """Step simulation and capture qacc at each step."""
    qacc = np.zeros((nsteps, model.nv), dtype=np.float64)
    for i in range(nsteps):
        mujoco.mj_step(model, data)
        qacc[i] = data.qacc[: model.nv].copy()
    return qacc


def generate(model_path, output_dir):
    if mujoco.__version__ != "3.4.0":
        raise RuntimeError(
            f"mujoco=={mujoco.__version__} but golden data requires mujoco==3.4.0. "
            "Install with: uv pip install mujoco==3.4.0"
        )

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    print(f"  nv={model.nv}  nu={model.nu}  nq={model.nq}")

    # --- Baseline (no flags) ---
    mujoco.mj_resetData(model, data)
    data.ctrl[:] = CTRL_VALUE
    baseline = run_simulation(model, data, NSTEPS)
    np.save(output_dir / "baseline_qacc.npy", baseline)
    print(f"  {'baseline_qacc.npy':40s} shape={baseline.shape}")

    # --- Disable flags ---
    for name, flag in DISABLE_FLAGS:
        mujoco.mj_resetData(model, data)
        model.opt.disableflags = flag
        data.ctrl[:] = CTRL_VALUE
        qacc = run_simulation(model, data, NSTEPS)
        model.opt.disableflags = 0  # Reset for next iteration
        filename = f"disable_{name}_qacc.npy"
        np.save(output_dir / filename, qacc)
        differs = not np.allclose(baseline, qacc)
        print(f"  {filename:40s} shape={qacc.shape}  differs={differs}")

    # --- Enable flags ---
    for name, flag in ENABLE_FLAGS:
        mujoco.mj_resetData(model, data)
        model.opt.enableflags = flag
        data.ctrl[:] = CTRL_VALUE
        qacc = run_simulation(model, data, NSTEPS)
        model.opt.enableflags = 0  # Reset for next iteration
        filename = f"enable_{name}_qacc.npy"
        np.save(output_dir / filename, qacc)
        differs = not np.allclose(baseline, qacc)
        print(f"  {filename:40s} shape={qacc.shape}  differs={differs}")


if __name__ == "__main__":
    print(f"Model: {MODEL_PATH}")
    print(f"Output: {ASSETS_DIR}")
    generate(MODEL_PATH, ASSETS_DIR)
    print("Done. Generated 26 .npy files.")
