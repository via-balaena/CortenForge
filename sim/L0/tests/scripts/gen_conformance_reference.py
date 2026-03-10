#!/usr/bin/env python3
"""Generate per-stage conformance reference data from MuJoCo 3.4.0.

Usage:
    uv pip install mujoco==3.4.0 numpy
    uv run sim/L0/tests/scripts/gen_conformance_reference.py

Generates individual .npy files in sim/L0/tests/assets/golden/conformance/reference/
for each canonical conformance model. Each file contains a single field captured
after one mj_forward() call (per-stage reference) or N mj_step() calls (trajectory).

Output format: NumPy v1.0 .npy (individual files, NO .npz).

Models and their ctrl values:
    actuated_system: ctrl = [1.0, 0.5]
    composite_model: ctrl = [1.0]
    all others:      ctrl = None (no actuator inputs)
"""

import hashlib
import json
import time
from pathlib import Path

import mujoco
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
MODELS_DIR = SCRIPT_DIR.parent / "assets" / "golden" / "conformance" / "models"
OUTPUT_DIR = SCRIPT_DIR.parent / "assets" / "golden" / "conformance" / "reference"

# Per-model configuration: ctrl values and trajectory step counts.
# ctrl=None means no actuator inputs (model has nu=0 or ctrl left at zero).
MODEL_CONFIG = {
    "pendulum":         {"ctrl": None,       "traj_steps": 100},
    "double_pendulum":  {"ctrl": None,       "traj_steps": 100},
    "contact_scenario": {"ctrl": None,       "traj_steps": 100},
    "actuated_system":  {"ctrl": [1.0, 0.5], "traj_steps": 100},
    "tendon_model":     {"ctrl": None,       "traj_steps": 100},
    "sensor_model":     {"ctrl": None,       "traj_steps": 100},
    "equality_model":   {"ctrl": None,       "traj_steps": 100},
    "composite_model":  {"ctrl": [1.0],      "traj_steps": 200},
}


def file_md5(path):
    """Compute MD5 checksum of a file."""
    h = hashlib.md5()
    h.update(path.read_bytes())
    return h.hexdigest()


def save_npy(output_dir, filename, array, manifest):
    """Save array as .npy and record in manifest."""
    path = output_dir / filename
    np.save(path, array)
    manifest[filename] = {
        "shape": list(array.shape),
        "dtype": str(array.dtype),
    }


def generate_per_stage(model_name, model, data, output_dir, ctrl, manifest):
    """Generate per-stage reference data from a single forward() call.

    Calls mj_forward() once and captures per-stage output fields.
    Only saves files for stages that produce non-empty data.
    """
    mujoco.mj_resetData(model, data)
    if ctrl is not None:
        for i, v in enumerate(ctrl):
            data.ctrl[i] = v

    mujoco.mj_forward(model, data)

    prefix = model_name

    # --- FK stage ---
    save_npy(output_dir, f"{prefix}_fk_xpos.npy",
             data.xpos.copy(), manifest)
    save_npy(output_dir, f"{prefix}_fk_xquat.npy",
             data.xquat.copy(), manifest)
    save_npy(output_dir, f"{prefix}_fk_xipos.npy",
             data.xipos.copy(), manifest)

    # --- CRBA stage: full dense mass matrix ---
    M = np.zeros((model.nv, model.nv), dtype=np.float64)
    mujoco.mj_fullM(model, M, data.qM)
    save_npy(output_dir, f"{prefix}_crba_qM.npy", M, manifest)

    # --- RNE stage ---
    save_npy(output_dir, f"{prefix}_rne_qfrc_bias.npy",
             data.qfrc_bias[:model.nv].copy(), manifest)

    # --- Passive force stage ---
    save_npy(output_dir, f"{prefix}_passive_qfrc_passive.npy",
             data.qfrc_passive[:model.nv].copy(), manifest)

    # --- Actuator stage (only if model has actuators) ---
    if model.nu > 0:
        save_npy(output_dir, f"{prefix}_actuator_qfrc_actuator.npy",
                 data.qfrc_actuator[:model.nv].copy(), manifest)
        save_npy(output_dir, f"{prefix}_actuator_force.npy",
                 data.actuator_force[:model.nu].copy(), manifest)

    # --- Sensor stage (only if model has sensors) ---
    if model.nsensor > 0:
        save_npy(output_dir, f"{prefix}_sensor_sensordata.npy",
                 data.sensordata[:model.nsensordata].copy(), manifest)

    # --- Tendon stage (only if model has tendons) ---
    if model.ntendon > 0:
        save_npy(output_dir, f"{prefix}_tendon_length.npy",
                 data.ten_length[:model.ntendon].copy(), manifest)
        save_npy(output_dir, f"{prefix}_tendon_velocity.npy",
                 data.ten_velocity[:model.ntendon].copy(), manifest)

    # --- Contact stage (only if contacts were detected) ---
    ncon = data.ncon
    if ncon > 0:
        # data.contact.pos: (nconmax, 3), data.contact.frame: (nconmax, 9)
        # data.contact.dist: (nconmax,), data.contact.geom: (nconmax, 2)
        save_npy(output_dir, f"{prefix}_contact_pos.npy",
                 data.contact.pos[:ncon].copy(), manifest)
        # Contact frame: 9 elements = [normal(3), tangent1(3), tangent2(3)]
        # Extract normal (first 3 elements of each row)
        frame = data.contact.frame[:ncon].copy()
        normal = frame[:, :3]
        save_npy(output_dir, f"{prefix}_contact_normal.npy",
                 normal, manifest)
        save_npy(output_dir, f"{prefix}_contact_depth.npy",
                 data.contact.dist[:ncon].copy(), manifest)
        save_npy(output_dir, f"{prefix}_contact_geom_pairs.npy",
                 data.contact.geom[:ncon].copy().astype(np.int32), manifest)

    # --- Constraint stage (only if constraints were assembled) ---
    nefc = data.nefc
    if nefc > 0:
        # efc_J is stored flat as (nefc * nv,) in MuJoCo Python (dense mode).
        # Reshape to (nefc, nv) for downstream consumption.
        efc_J = data.efc_J[:nefc * model.nv].copy().reshape(nefc, model.nv)
        save_npy(output_dir, f"{prefix}_constraint_efc_J.npy",
                 efc_J, manifest)
        save_npy(output_dir, f"{prefix}_constraint_efc_b.npy",
                 data.efc_b[:nefc].copy(), manifest)
        save_npy(output_dir, f"{prefix}_constraint_efc_force.npy",
                 data.efc_force[:nefc].copy(), manifest)


def generate_trajectory(model_name, model, data, output_dir, ctrl, nsteps, manifest):
    """Generate trajectory reference data from N step() calls.

    Resets data, sets ctrl, then steps N times capturing qpos/qvel/qacc.
    """
    mujoco.mj_resetData(model, data)
    if ctrl is not None:
        for i, v in enumerate(ctrl):
            data.ctrl[i] = v

    qpos_traj = np.zeros((nsteps, model.nq), dtype=np.float64)
    qvel_traj = np.zeros((nsteps, model.nv), dtype=np.float64)
    qacc_traj = np.zeros((nsteps, model.nv), dtype=np.float64)

    for i in range(nsteps):
        mujoco.mj_step(model, data)
        qpos_traj[i] = data.qpos[:model.nq].copy()
        qvel_traj[i] = data.qvel[:model.nv].copy()
        qacc_traj[i] = data.qacc[:model.nv].copy()

    prefix = model_name
    save_npy(output_dir, f"{prefix}_trajectory_qpos.npy",
             qpos_traj, manifest)
    save_npy(output_dir, f"{prefix}_trajectory_qvel.npy",
             qvel_traj, manifest)
    save_npy(output_dir, f"{prefix}_trajectory_qacc.npy",
             qacc_traj, manifest)


def generate_all():
    """Generate all conformance reference data."""
    if mujoco.__version__ != "3.4.0":
        raise RuntimeError(
            f"mujoco=={mujoco.__version__} but reference data requires mujoco==3.4.0. "
            "Install with: uv pip install mujoco==3.4.0"
        )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    metadata = {
        "mujoco_version": mujoco.__version__,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "models": {},
        "files": {},
    }

    for model_name, config in MODEL_CONFIG.items():
        model_path = MODELS_DIR / f"{model_name}.xml"
        if not model_path.exists():
            print(f"  SKIP {model_name}: {model_path} not found")
            continue

        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        ctrl = config["ctrl"]
        traj_steps = config["traj_steps"]

        print(f"\n  {model_name}:")
        print(f"    nq={model.nq}  nv={model.nv}  nu={model.nu}  "
              f"na={model.na}  nsensor={model.nsensor}  ntendon={model.ntendon}")

        model_meta = {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "na": int(model.na),
            "nsensor": int(model.nsensor),
            "nsensordata": int(model.nsensordata),
            "ntendon": int(model.ntendon),
            "ctrl": ctrl,
            "trajectory_steps": traj_steps,
            "model_xml_md5": file_md5(model_path),
        }
        metadata["models"][model_name] = model_meta

        manifest = {}

        # Per-stage reference data (single forward pass)
        generate_per_stage(model_name, model, data, OUTPUT_DIR, ctrl, manifest)

        # Trajectory data (N steps)
        generate_trajectory(
            model_name, model, data, OUTPUT_DIR, ctrl, traj_steps, manifest)

        # Print file summary
        for fname, info in sorted(manifest.items()):
            print(f"    {fname:55s} shape={info['shape']}")

        metadata["files"].update(manifest)

    # Write metadata JSON
    meta_path = OUTPUT_DIR / "reference_metadata.json"
    meta_path.write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"\n  Metadata written: {meta_path}")
    print(f"  Total files: {len(metadata['files'])} .npy + 1 .json")


if __name__ == "__main__":
    print(f"Models: {MODELS_DIR}")
    print(f"Output: {OUTPUT_DIR}")
    generate_all()
    print("\nDone.")
