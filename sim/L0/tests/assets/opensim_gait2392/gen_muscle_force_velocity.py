"""Regenerate `muscle_force_velocity_opensim.json` — the real-OpenSim FORCE reference
under non-zero fiber velocity, for the G2-PR2 force-velocity cross-check. This is the
non-isometric companion to `gen_muscle_forces.py` (which holds the joint still).

Run:
    /tmp/osim-venv/bin/python gen_muscle_force_velocity.py gait2392.osim muscle_force_velocity_opensim.json

Same apples-to-apples setup as the isometric reference: `ignore_tendon_compliance=True`
(rigid tendon), activation set directly on the state. Here we additionally set the knee
COORDINATE SPEED so the muscle has a nonzero fiber velocity, then realize Dynamics and
record the force the muscle applies along its path (`getActuation`) together with the
musculotendon length and lengthening speed and the (normalized) fiber velocity. Our Rust
`millard_path_force` recomputes the force from `mtu_length` + `mtu_velocity` and is graded
against `force_along_path`; `normalized_fiber_velocity` independently checks our fiber-
velocity derivation.

A grid of knee angles (muscles near their active range) × knee speeds (shortening through
lengthening) × activations exercises the force-velocity curve + fiber damping.
"""
import json
import math
import sys
import opensim as osim

osim.Logger.setLevelString("Error")

OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]
ANGLES_DEG = [-20.0, -50.0, -80.0]
SPEEDS_RAD_S = [-20.0, -10.0, -4.0, 0.0, 4.0, 10.0, 20.0]
ACTIVATIONS = [1.0, 0.5]

model = osim.Model(OSIM)
muscs = {}
for mname in MUSCLES:
    m = osim.Muscle.safeDownCast(model.getForceSet().get(mname))
    m.set_ignore_tendon_compliance(True)
    muscs[mname] = m

state = model.initSystem()
coord = model.getCoordinateSet().get("knee_angle_r")

params = {}
for mname, m in muscs.items():
    params[mname] = {
        "max_isometric_force_N": m.getMaxIsometricForce(),
        "optimal_fiber_length_m": m.getOptimalFiberLength(),
        "tendon_slack_length_m": m.getTendonSlackLength(),
        "pennation_at_optimal_rad": m.getPennationAngleAtOptimalFiberLength(),
        "max_contraction_velocity_lopt_per_s": m.getMaxContractionVelocity(),
    }

results = {}
for mname, m in muscs.items():
    rows = []
    for adeg in ANGLES_DEG:
        arad = math.radians(adeg)
        for speed in SPEEDS_RAD_S:
            coord.setValue(state, arad, True)
            coord.setSpeedValue(state, speed)
            per_act = {}
            for a in ACTIVATIONS:
                m.setActivation(state, a)
                model.realizeDynamics(state)
                per_act[f"{a:.2f}"] = {
                    "activation": a,
                    "force_along_path_N": m.getActuation(state),
                    "normalized_fiber_velocity": m.getNormalizedFiberVelocity(state),
                    "fiber_velocity_m_per_s": m.getFiberVelocity(state),
                    "normalized_fiber_length": m.getNormalizedFiberLength(state),
                }
            # mtu length + lengthening speed are activation-independent kinematics.
            rows.append({
                "angle_deg": adeg,
                "knee_speed_rad_s": speed,
                "mtu_length_m": m.getLength(state),
                "mtu_velocity_m_per_s": m.getLengtheningSpeed(state),
                "by_activation": per_act,
            })
    results[mname] = rows

with open(OUT, "w") as f:
    json.dump({
        "source": "OpenSim " + osim.GetVersion(),
        "model": OSIM.split("/")[-1],
        "muscle_model": "Millard2012EquilibriumMuscle, ignore_tendon_compliance=True (rigid), fiber_damping=0.1",
        "coordinate": "knee_angle_r",
        "regime": "non-isometric (knee speed set), activation set directly on state",
        "convention": "force_along_path = getActuation (tension>0); mtu_velocity = lengthening speed (>0 lengthening)",
        "angles_deg": ANGLES_DEG,
        "knee_speeds_rad_s": SPEEDS_RAD_S,
        "activations": ACTIVATIONS,
        "params": params,
        "muscles": results,
    }, f, indent=1)

print(f"OpenSim {osim.GetVersion()}  force-velocity grid: "
      f"{len(MUSCLES)} muscles × {len(ANGLES_DEG)} angles × {len(SPEEDS_RAD_S)} speeds × "
      f"{len(ACTIVATIONS)} activations.")
print("rect_fem_r @ -50deg, a=1.0, path force (N) vs knee speed:")
for row in results["rect_fem_r"]:
    if abs(row["angle_deg"] - (-50.0)) < 1e-6:
        b = row["by_activation"]["1.00"]
        print(f"  speed {row['knee_speed_rad_s']:+6.1f}  vN {b['normalized_fiber_velocity']:+.3f}  "
              f"F {b['force_along_path_N']:8.1f}")
