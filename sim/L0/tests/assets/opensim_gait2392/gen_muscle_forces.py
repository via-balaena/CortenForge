"""Regenerate `muscle_forces_opensim.json` — the real-OpenSim FORCE reference for
the G2 (dynamics) spike. This is the muscle-FORCE analogue of `gen_moment_arms.py`
(which records path geometry / moment arms only).

Run (needs the `opensim` PyPI wheel; CPython 3.11-3.13 macOS/Linux/Windows):

    uv venv --python 3.12 /tmp/osim-venv
    VIRTUAL_ENV=/tmp/osim-venv uv pip install opensim
    /tmp/osim-venv/bin/python gen_muscle_forces.py gait2392.osim muscle_forces_opensim.json

Apples-to-apples with our MuJoCo-style Hill engine, which uses a RIGID tendon and
takes a known activation directly:
  - `ignore_tendon_compliance = True`  -> rigid tendon (no fiber-length state /
    equilibrium solve), matching our engine's algebraic fiber geometry.
  - activation set directly on the state (`setActivation`) and realized, so the
    recorded force is at a KNOWN activation (no excitation->activation ODE).
  - velocities left at zero -> ISOMETRIC (force-velocity factor = 1), so the gap
    isolates the force-LENGTH + passive + pennation model differences (the #1 G2
    question: engine-Hill Gaussian-FL/exp-FP vs OpenSim Millard2012).

Sweeps knee_angle_r 0 -> -100 deg (flexion), all other coordinates at neutral, at
two activations. Records, per muscle/angle/activation, the force the muscle applies
along its path (`getActuation`) plus active/passive fiber force, normalized fiber
length, pennation, MTU length, and the moment arm + resulting knee moment. Also
records each muscle's force parameters (F0, L0, Lts, pennation@opt, vmax) so the
Rust side can evaluate our Hill curves on the same parameters.
"""
import json
import math
import sys
import opensim as osim

osim.Logger.setLevelString("Error")  # silence missing-geometry warnings

OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]
ACTIVATIONS = [1.0, 0.5]

model = osim.Model(OSIM)

# Rigid tendon on the target muscles, set BEFORE initSystem so it takes effect.
muscs = {}
for mname in MUSCLES:
    m = osim.Muscle.safeDownCast(model.getForceSet().get(mname))
    m.set_ignore_tendon_compliance(True)
    muscs[mname] = m

state = model.initSystem()
coord = model.getCoordinateSet().get("knee_angle_r")

angles_deg = [-(5.0 * i) for i in range(21)]  # 0, -5, ... -100

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
    for adeg in angles_deg:
        arad = math.radians(adeg)
        coord.setValue(state, arad, True)  # enforce coupled-joint constraints
        # qvel defaults to zero -> isometric (fiber velocity 0).
        per_act = {}
        for a in ACTIVATIONS:
            m.setActivation(state, a)
            model.realizeDynamics(state)
            r = m.computeMomentArm(state, coord)
            f_path = m.getActuation(state)  # force along the path (= tendon force)
            per_act[f"{a:.2f}"] = {
                "activation": a,
                "force_along_path_N": f_path,
                "active_fiber_force_N": m.getActiveFiberForce(state),
                "passive_fiber_force_N": m.getPassiveFiberForce(state),
                "fiber_force_along_tendon_N": m.getFiberForceAlongTendon(state),
                "normalized_fiber_length": m.getNormalizedFiberLength(state),
                "fiber_length_m": m.getFiberLength(state),
                "pennation_angle_rad": m.getPennationAngle(state),
                "moment_arm_m": r,
                "knee_moment_Nm": f_path * r,
            }
        rows.append({
            "angle_deg": adeg,
            "angle_rad": arad,
            "mtu_length_m": m.getLength(state),  # activation-independent
            "by_activation": per_act,
        })
    results[mname] = rows

with open(OUT, "w") as f:
    json.dump({
        "source": "OpenSim " + osim.GetVersion(),
        "model": OSIM.split("/")[-1],
        "muscle_model": "Millard2012EquilibriumMuscle, ignore_tendon_compliance=True (rigid)",
        "coordinate": "knee_angle_r",
        "regime": "isometric (qvel=0), activation set directly on state",
        "convention": "force_along_path = getActuation (tension>0); moment = force*moment_arm",
        "activations": ACTIVATIONS,
        "params": params,
        "muscles": results,
    }, f, indent=1)

print(f"OpenSim {osim.GetVersion()}  rigid-tendon isometric force (N) at a=1.0, "
      "0 / 50 / 100 deg flexion:")
for m in MUSCLES:
    r = results[m]
    g = lambda row: row["by_activation"]["1.00"]["force_along_path_N"]
    print(f"  {m:<11} {g(r[0]):8.1f} {g(r[10]):8.1f} {g(r[20]):8.1f}")
