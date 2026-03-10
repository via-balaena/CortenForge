#!/usr/bin/env python3
"""Dump per-constraint-row data from MuJoCo for Phase 13 diagnostic.

Usage:
    uv run sim/L0/tests/scripts/dump_constraint_diagnostic.py

Outputs constraint diagnostic data for the flag_golden_test.xml model
to stdout. This data is used to compare against CortenForge's constraint
assembly and solver output to identify the root cause of the ~1.69
qacc divergence on DOF 0.
"""

from pathlib import Path

import mujoco
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR.parent / "assets" / "golden" / "flags"
MODEL_PATH = ASSETS_DIR / "flag_golden_test.xml"
OUTPUT_DIR = ASSETS_DIR / "diagnostic"

CTRL_VALUE = 2.0


def dump_constraint_data(model, data, label=""):
    """Dump all per-constraint-row data."""
    nefc = data.nefc
    ne = data.ne
    nf = data.nf

    print(f"\n{'='*80}")
    print(f"Constraint Diagnostic: {label}")
    print(f"{'='*80}")
    print(f"  nefc={nefc}  ne={ne}  nf={nf}  nv={model.nv}  nq={model.nq}")
    print(f"  ncon={data.ncon}")
    print(f"  solver_type={model.opt.solver}")
    print()

    # Constraint types from MuJoCo
    type_names = {
        0: "Equality",
        1: "FrictionLoss",
        2: "LimitJoint",
        3: "LimitTendon",
        4: "ContactFrictionless",
        5: "ContactPyramidal",
        6: "ContactElliptic",
    }

    for i in range(nefc):
        ctype = data.efc_type[i]
        type_name = type_names.get(ctype, f"Unknown({ctype})")
        print(f"  Row {i}: type={type_name} ({ctype})")
        print(f"    efc_id     = {data.efc_id[i]}")
        print(f"    efc_pos    = {data.efc_pos[i]:.15e}")
        print(f"    efc_margin = {data.efc_margin[i]:.15e}")

        # Jacobian row (efc_J is stored as flat nefc*nv in MuJoCo)
        j_start = i * model.nv
        j_row = data.efc_J[j_start:j_start + model.nv]
        nonzero = [(j, v) for j, v in enumerate(j_row) if abs(v) > 1e-20]
        print(f"    efc_J      = {nonzero}")

        print(f"    efc_diagApprox = {data.efc_diagApprox[i]:.15e}")
        print(f"    efc_R      = {data.efc_R[i]:.15e}")
        print(f"    efc_D      = {data.efc_D[i]:.15e}")
        print(f"    efc_aref   = {data.efc_aref[i]:.15e}")
        print(f"    efc_b      = {data.efc_b[i]:.15e}")
        print(f"    efc_force  = {data.efc_force[i]:.15e}")
        print(f"    efc_state  = {data.efc_state[i]}")

        # Solver params (not directly accessible as per-row in MuJoCo Python)
        print()

    # qacc
    print(f"  qacc = [", end="")
    for i in range(model.nv):
        if i > 0:
            print(", ", end="")
        print(f"{data.qacc[i]:.15e}", end="")
    print("]")

    # qacc_smooth (unconstrained acceleration)
    print(f"  qacc_smooth = [", end="")
    for i in range(model.nv):
        if i > 0:
            print(", ", end="")
        print(f"{data.qacc_smooth[i]:.15e}", end="")
    print("]")

    # qfrc_constraint
    print(f"  qfrc_constraint = [", end="")
    for i in range(model.nv):
        if i > 0:
            print(", ", end="")
        print(f"{data.qfrc_constraint[i]:.15e}", end="")
    print("]")

    # qfrc_smooth
    print(f"  qfrc_smooth = [", end="")
    for i in range(model.nv):
        if i > 0:
            print(", ", end="")
        print(f"{data.qfrc_smooth[i]:.15e}", end="")
    print("]")

    print()


def dump_model_params(model, data):
    """Dump model-level parameters relevant to constraints."""
    print(f"\n{'='*80}")
    print("Model Parameters")
    print(f"{'='*80}")
    print(f"  timestep = {model.opt.timestep}")
    print(f"  gravity  = {model.opt.gravity}")
    print(f"  solver   = {model.opt.solver}  (0=PGS, 1=CG, 2=Newton)")
    print(f"  cone     = {model.opt.cone}  (0=pyramidal, 1=elliptic)")
    print(f"  niter    = {model.opt.iterations}")
    print(f"  tolerance= {model.opt.tolerance}")
    print(f"  noslip_iterations = {model.opt.noslip_iterations}")

    # Body invweight0
    print(f"\n  body_invweight0:")
    for i in range(model.nbody):
        w = model.body_invweight0[i]
        print(f"    body {i}: [{w[0]:.15e}, {w[1]:.15e}]")

    # DOF invweight0
    print(f"\n  dof_invweight0:")
    for i in range(model.nv):
        print(f"    dof {i}: {model.dof_invweight0[i]:.15e}")

    # Tendon invweight0
    if model.ntendon > 0:
        print(f"\n  tendon_invweight0:")
        for i in range(model.ntendon):
            print(f"    tendon {i}: {model.tendon_invweight0[i]:.15e}")

    # DOF friction/solref/solimp
    print(f"\n  DOF friction parameters:")
    for i in range(model.nv):
        fl = model.dof_frictionloss[i]
        if fl > 0:
            sr = model.dof_solref[i]
            si = model.dof_solimp[i]
            print(f"    dof {i}: frictionloss={fl}")
            print(f"      solref=[{sr[0]:.6f}, {sr[1]:.6f}]")
            print(f"      solimp=[{si[0]:.6f}, {si[1]:.6f}, {si[2]:.6f}, {si[3]:.6f}, {si[4]:.6f}]")

    # In MuJoCo, dof_solref/dof_solimp ARE the friction loss solver params
    # (there is no separate dof_solref_friction in MuJoCo 3.4.0)

    # Equality parameters
    print(f"\n  Equality constraints:")
    for i in range(model.neq):
        print(f"    eq {i}: type={model.eq_type[i]} active={model.eq_active0[i]}")
        print(f"      obj1id={model.eq_obj1id[i]} obj2id={model.eq_obj2id[i]}")
        print(f"      data={model.eq_data[i]}")
        sr = model.eq_solref[i]
        si = model.eq_solimp[i]
        print(f"      solref=[{sr[0]:.6f}, {sr[1]:.6f}]")
        print(f"      solimp=[{si[0]:.6f}, {si[1]:.6f}, {si[2]:.6f}, {si[3]:.6f}, {si[4]:.6f}]")

    # Mass matrix — need to run mj_forward first to populate qM
    mujoco.mj_forward(model, data)
    qm_full = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, qm_full, data.qM)

    print(f"\n  qM diagonal (after mj_forward):")
    for i in range(model.nv):
        print(f"    qM[{i},{i}] = {qm_full[i,i]:.15e}")

    print(f"\n  Full qM:")
    for i in range(model.nv):
        row = [f"{qm_full[i,j]:.10e}" for j in range(model.nv)]
        print(f"    [{', '.join(row)}]")


def save_diagnostic_npy(model, data, output_dir):
    """Save constraint data as .npy files for Rust comparison."""
    output_dir.mkdir(parents=True, exist_ok=True)

    nefc = data.nefc
    nv = model.nv

    np.save(output_dir / "efc_type.npy", data.efc_type[:nefc].astype(np.float64))
    np.save(output_dir / "efc_pos.npy", data.efc_pos[:nefc].copy())
    np.save(output_dir / "efc_margin.npy", data.efc_margin[:nefc].copy())
    # efc_J is flat (nefc*nv), reshape to 2D
    efc_J_flat = data.efc_J[:nefc * nv].copy()
    np.save(output_dir / "efc_J.npy", efc_J_flat.reshape(nefc, nv))
    np.save(output_dir / "efc_diagApprox.npy", data.efc_diagApprox[:nefc].copy())
    np.save(output_dir / "efc_R.npy", data.efc_R[:nefc].copy())
    np.save(output_dir / "efc_D.npy", data.efc_D[:nefc].copy())
    np.save(output_dir / "efc_aref.npy", data.efc_aref[:nefc].copy())
    np.save(output_dir / "efc_b.npy", data.efc_b[:nefc].copy())
    np.save(output_dir / "efc_force.npy", data.efc_force[:nefc].copy())
    np.save(output_dir / "efc_state.npy", data.efc_state[:nefc].astype(np.float64))
    np.save(output_dir / "efc_id.npy", data.efc_id[:nefc].astype(np.float64))
    np.save(output_dir / "qacc.npy", data.qacc[:nv].copy())
    np.save(output_dir / "qacc_smooth.npy", data.qacc_smooth[:nv].copy())
    np.save(output_dir / "qfrc_constraint.npy", data.qfrc_constraint[:nv].copy())
    np.save(output_dir / "qfrc_smooth.npy", data.qfrc_smooth[:nv].copy())

    # Also save qM diagonal and full mass matrix
    qm_full = np.zeros((nv, nv))
    mujoco.mj_fullM(model, qm_full, data.qM)
    np.save(output_dir / "qM_full.npy", qm_full)

    # Save model-level weights
    np.save(output_dir / "body_invweight0.npy", model.body_invweight0[:model.nbody].copy())
    np.save(output_dir / "dof_invweight0.npy", model.dof_invweight0[:nv].copy())

    print(f"\nSaved {18} .npy files to {output_dir}")


def main():
    assert mujoco.__version__ == "3.4.0", f"Need mujoco==3.4.0, got {mujoco.__version__}"

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    print(f"Model: {MODEL_PATH}")
    print(f"  nv={model.nv}  nq={model.nq}  nu={model.nu}  nbody={model.nbody}")
    print(f"  neq={model.neq}  ntendon={model.ntendon}  njnt={model.njnt}")
    print(f"  ncon(initial)={data.ncon}")

    dump_model_params(model, data)

    # Step 1: Reset and set ctrl
    mujoco.mj_resetData(model, data)
    data.ctrl[:] = CTRL_VALUE

    # Run mj_forward to populate all intermediate data WITHOUT stepping
    # This gives us the constraint data at the initial state.
    mujoco.mj_forward(model, data)

    print(f"\n--- After mj_forward (before any step) ---")
    dump_constraint_data(model, data, "initial forward")
    save_diagnostic_npy(model, data, OUTPUT_DIR / "step0_pre")

    # Now do mj_step and dump after step 0
    mujoco.mj_resetData(model, data)
    data.ctrl[:] = CTRL_VALUE
    mujoco.mj_step(model, data)

    print(f"\n--- After step 0 ---")
    dump_constraint_data(model, data, "after step 0")
    save_diagnostic_npy(model, data, OUTPUT_DIR / "step0_post")

    # Also dump disable_filterparent for comparison
    mujoco.mj_resetData(model, data)
    model.opt.disableflags = 1 << 10  # DISABLE_FILTERPARENT
    data.ctrl[:] = CTRL_VALUE
    mujoco.mj_step(model, data)

    print(f"\n--- After step 0 (DISABLE_FILTERPARENT) ---")
    dump_constraint_data(model, data, "DISABLE_FILTERPARENT step 0")
    save_diagnostic_npy(model, data, OUTPUT_DIR / "step0_filterparent")

    model.opt.disableflags = 0  # Reset

    # Dump disable_frictionloss
    mujoco.mj_resetData(model, data)
    model.opt.disableflags = 1 << 2  # DISABLE_FRICTIONLOSS
    data.ctrl[:] = CTRL_VALUE
    mujoco.mj_step(model, data)

    print(f"\n--- After step 0 (DISABLE_FRICTIONLOSS) ---")
    dump_constraint_data(model, data, "DISABLE_FRICTIONLOSS step 0")
    save_diagnostic_npy(model, data, OUTPUT_DIR / "step0_frictionloss")

    model.opt.disableflags = 0  # Reset


if __name__ == "__main__":
    main()
