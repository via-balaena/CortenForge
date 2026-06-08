"""Regenerate `moment_arms_opensim.json` — the real-OpenSim **leg-chain** reference
`cf-osim`'s `opensim_cross_check` grades the oracle against at the unwelded hip
(A2). (The companion `gen_moment_arms.py` regenerates the knee-only ROM reference
`knee_moment_arms_opensim.json` used by the G1 knee study + the knee-ROM check.)

Run (needs the `opensim` PyPI wheel; arm64/x86 macOS + Linux + Windows wheels
exist for CPython 3.11-3.13):

    uv venv --python 3.12 /tmp/osim-venv
    uv pip install --python /tmp/osim-venv opensim
    /tmp/osim-venv/bin/python gen_leg_moment_arms.py gait2392.osim moment_arms_opensim.json

Covers the knee AND the unwelded hip (leg-region A2). Every moment arm is
evaluated at a **multi-DOF base pose** (several non-zero hip rotations + a flexed
knee at once), then one coordinate is swept around its base value while the others
stay at the base. This is deliberate: a single-coordinate sweep from neutral would
leave only one rotation non-zero, so it could NOT exercise the hip's rotation
*composition order* (R-rot). With multiple rotations non-zero throughout, the
reference pins that order against real OpenSim.

OpenSim convention: r = -d(length)/d(coordinate), the total derivative through the
coupled CustomJoint (`computeMomentArm`).
"""
import json
import math
import sys

import opensim as osim

osim.Logger.setLevelString("Error")  # silence missing-geometry warnings

OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]

# Multi-DOF base pose (rad). Distinct signs + magnitudes so every moment arm is
# evaluated with several non-zero hip rotations — this is what exercises the
# rotation-composition order.
BASE = {
    "hip_flexion_r": 0.3,
    "hip_adduction_r": -0.2,
    "hip_rotation_r": 0.15,
    "knee_angle_r": -0.6,
}
SWEEP_COORDS = ["hip_flexion_r", "hip_adduction_r", "hip_rotation_r", "knee_angle_r"]
SWEEP_OFFSETS_DEG = [-20.0, -10.0, 0.0, 10.0, 20.0]  # around each coord's base value

model = osim.Model(OSIM)
state = model.initSystem()
cs = model.getCoordinateSet()


def set_pose(overrides):
    for name, val in BASE.items():
        cs.get(name).setValue(state, val, False)
    for name, val in overrides.items():
        cs.get(name).setValue(state, val, False)
    model.assemble(state)
    model.realizePosition(state)


sweeps = {}
for coord_name in SWEEP_COORDS:
    coord = cs.get(coord_name)
    per_muscle = {m: [] for m in MUSCLES}
    for d in SWEEP_OFFSETS_DEG:
        v = BASE[coord_name] + math.radians(d)
        set_pose({coord_name: v})
        for mname in MUSCLES:
            musc = osim.Muscle.safeDownCast(model.getForceSet().get(mname))
            per_muscle[mname].append({
                "value_rad": v,
                "moment_arm_m": musc.computeMomentArm(state, coord),
                "length_m": musc.getGeometryPath().getLength(state),
            })
    sweeps[coord_name] = per_muscle

with open(OUT, "w") as f:
    json.dump({
        "source": "OpenSim " + osim.GetVersion(),
        "model": OSIM.split("/")[-1],
        "convention": "moment_arm = -d(length)/d(coordinate), meters",
        "base_pose_rad": BASE,
        "sweeps": sweeps,
    }, f, indent=1)

print(f"OpenSim {osim.GetVersion()} moment arms (mm) at the base pose:")
print(f"{'coord':<16} " + " ".join(f"{m:>11}" for m in MUSCLES))
for coord_name in SWEEP_COORDS:
    # the middle sample (offset 0) is the base pose itself
    mid = len(SWEEP_OFFSETS_DEG) // 2
    vals = " ".join(f"{sweeps[coord_name][m][mid]['moment_arm_m']*1000:11.2f}" for m in MUSCLES)
    print(f"{coord_name:<16} {vals}")
