"""Regenerate `muscle_joint_moments_opensim.json` — the real-OpenSim KNEE JOINT MOMENT
reference for the G2-PR3b muscle-driven-twin dynamics gate. The joint-torque analogue
of the force/moment-arm references: it records what each muscle contributes to the knee
generalized force when driven at a known activation.

Run:
    /tmp/osim-venv/bin/python gen_muscle_joint_moments.py gait2392.osim muscle_joint_moments_opensim.json

Same apples-to-apples setup as gen_muscle_forces.py (rigid tendon, activation set on the
state, isometric). Per muscle / knee angle / activation, the knee joint moment is
`getActuation(state) · computeMomentArm(state, knee)` — the muscle's force along its path
times its knee moment arm = its contribution to the knee generalized force (N·m). Our
muscle-driven twin reproduces this as (engine actuator force) × (engine coupled moment arm).
"""
import json
import math
import sys
import opensim as osim

osim.Logger.setLevelString("Error")
OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]
ACTIVATIONS = [1.0, 0.5]

model = osim.Model(OSIM)
muscs = {}
for mname in MUSCLES:
    m = osim.Muscle.safeDownCast(model.getForceSet().get(mname))
    m.set_ignore_tendon_compliance(True)
    muscs[mname] = m

state = model.initSystem()
coord = model.getCoordinateSet().get("knee_angle_r")
angles_deg = [-(5.0 * i) for i in range(21)]  # 0 … −100

results = {}
for mname, m in muscs.items():
    rows = []
    for adeg in angles_deg:
        arad = math.radians(adeg)
        coord.setValue(state, arad, True)
        per_act = {}
        for a in ACTIVATIONS:
            m.setActivation(state, a)
            model.realizeDynamics(state)
            f = m.getActuation(state)
            r = m.computeMomentArm(state, coord)
            per_act[f"{a:.2f}"] = {
                "activation": a,
                "force_along_path_N": f,
                "moment_arm_m": r,
                "knee_moment_Nm": f * r,
            }
        rows.append({"angle_deg": adeg, "angle_rad": arad, "by_activation": per_act})
    results[mname] = rows

with open(OUT, "w") as fp:
    json.dump({
        "source": "OpenSim " + osim.GetVersion(),
        "model": OSIM.split("/")[-1],
        "muscle_model": "Millard2012EquilibriumMuscle, ignore_tendon_compliance=True (rigid)",
        "coordinate": "knee_angle_r",
        "regime": "isometric, activation set directly on state",
        "convention": "knee_moment = getActuation · computeMomentArm (N·m)",
        "activations": ACTIVATIONS,
        "muscles": results,
    }, fp, indent=1)

print(f"OpenSim {osim.GetVersion()}  knee joint moments (N·m) at a=1.0, 0/50/100 deg flexion:")
for mname in MUSCLES:
    r = results[mname]
    g = lambda row: row["by_activation"]["1.00"]["knee_moment_Nm"]
    print(f"  {mname:<11} {g(r[0]):7.2f} {g(r[10]):7.2f} {g(r[20]):7.2f}")
