"""Regenerate `forward_dynamics_opensim.json` — the real-OpenSim FORWARD-DYNAMICS
reference for the G3 muscle-driven-twin gate (activations -> joint ACCELERATION).

Run:
    /tmp/osim-venv/bin/python gen_forward_dynamics.py gait2392.osim forward_dynamics_opensim.json

This is the end-to-end "activations -> motion" oracle. Apples-to-apples with the
twin's emitted forward dynamics:

- Reduced RIGHT LEG = 5 free DOF (hip flexion/adduction/rotation + knee + ankle),
  matching the emitted twin (pelvis welded; contralateral leg, torso, subtalar and
  mtp all LOCKED, so the foot is rigid at the ankle exactly as the twin's lumped
  talus body).
- Rigid tendon (ignore_tendon_compliance), matching the engine's algebraic fiber.
- GRAVITY OFF, so the comparison isolates muscle force -> joint acceleration (no
  gravity-frame-matching confound).
- Activations set directly on the state (no excitation->activation dynamics), and
  ALL generalized speeds zeroed via `state.setU(0)` (a per-coordinate setSpeedValue
  after setValue(...,True) leaves residual velocity -> garbage udot).
- Only the 4 right-leg muscles apply force; everything else disabled.

The JSON also carries the gait2392 segment inertias these accelerations were
computed with (femur + tibia direct; the foot is the talus+calcn+toes composite
about the talus frame, since subtalar/mtp are locked). The twin now EMITS these
same inertias (`cf_osim::parse_leg_chain` reads femur/tibia and composes the
locked-foot composite), so the gate no longer injects them — it cross-checks that
the emitted inertias equal these, then validates the shipped twin's forward
dynamics end-to-end (solver + coupling + force + anthropometry).
"""
import json
import sys

import numpy as np
import opensim as osim

osim.Logger.setLevelString("Error")
OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]
FREE = ["hip_flexion_r", "hip_adduction_r", "hip_rotation_r", "knee_angle_r", "ankle_angle_r"]
KNEE_ANGLES = [-0.3, -0.55, -0.8, -1.5]  # 3 functional (|knee|<=1) + 1 deep-flexion
ACTIVATION = 1.0


def mat3(R):
    return np.array([[R.get(i, j) for j in range(3)] for i in range(3)])


def vec3(v):
    return np.array([v.get(0), v.get(1), v.get(2)])


def foot_composite(model, state):
    """talus+calcn+toes spatial inertia about the talus body frame (subtalar/mtp=0)."""
    bs = model.getBodySet()
    talus = bs.get("talus_r")
    T = talus.getTransformInGround(state)
    Rt, pt = mat3(T.R()), vec3(T.p())
    M, I_o, Psum = 0.0, np.zeros((3, 3)), np.zeros(3)
    for name in ["talus_r", "calcn_r", "toes_r"]:
        b = bs.get(name)
        mass = b.getMass()
        com = vec3(b.getMassCenter())
        I = b.getInertia()
        mom, pr = vec3(I.getMoments()), vec3(I.getProducts())
        Ib = np.array([[mom[0], pr[0], pr[1]], [pr[0], mom[1], pr[2]], [pr[1], pr[2], mom[2]]])
        Tgb = b.getTransformInGround(state)
        Rgb, pgb = mat3(Tgb.R()), vec3(Tgb.p())
        R_tb = Rt.T @ Rgb
        p = Rt.T @ (pgb + Rgb @ com - pt)
        I_o += R_tb @ Ib @ R_tb.T + mass * (p @ p * np.eye(3) - np.outer(p, p))
        Psum += mass * p
        M += mass
    P = Psum / M
    I_com = I_o - M * (P @ P * np.eye(3) - np.outer(P, P))
    evals, evecs = np.linalg.eigh(I_com)
    if np.linalg.det(evecs) < 0:
        evecs[:, 0] = -evecs[:, 0]
    w = np.sqrt(max(0.0, 1 + evecs.trace())) / 2
    quat = [w, (evecs[2, 1] - evecs[1, 2]) / (4 * w), (evecs[0, 2] - evecs[2, 0]) / (4 * w),
            (evecs[1, 0] - evecs[0, 1]) / (4 * w)]
    return {"mass": M, "com": P.tolist(), "idiag": evals.tolist(), "iquat": quat}


model = osim.Model(OSIM)
model.setGravity(osim.Vec3(0, 0, 0))
for n in MUSCLES:
    osim.Muscle.safeDownCast(model.getForceSet().get(n)).set_ignore_tendon_compliance(True)
cs = model.getCoordinateSet()
for i in range(cs.getSize()):
    if cs.get(i).getName() not in FREE:
        cs.get(i).set_locked(True)
state = model.initSystem()
fs = model.getForceSet()
muscs = {n: osim.Muscle.safeDownCast(fs.get(n)) for n in MUSCLES}
knee = cs.get("knee_angle_r")

# Segment inertias the twin emits + the gate cross-checks (femur/tibia direct, foot
# composite).
bs = model.getBodySet()
inertias = {}
for name in ["femur_r", "tibia_r"]:
    b = bs.get(name)
    inertias[name] = {
        "mass": b.getMass(),
        "com": vec3(b.getMassCenter()).tolist(),
        "idiag": vec3(b.getInertia().getMoments()).tolist(),
        "iquat": [1.0, 0.0, 0.0, 0.0],
    }
inertias["talus_r"] = foot_composite(model, state)


def knee_udot(activate):
    """activate: dict muscle->activation; others get setAppliesForce(False)."""
    for i in range(fs.getSize()):
        fs.get(i).setAppliesForce(state, fs.get(i).getName() in activate)
    state.setU(osim.Vector(state.getNU(), 0.0))
    for n, m in muscs.items():
        if n in activate:
            m.setActivation(state, activate[n])
    model.realizeAcceleration(state)
    return knee.getAccelerationValue(state)


cases = []
for ang in KNEE_ANGLES:
    knee.setValue(state, ang, True)
    full = knee_udot({n: ACTIVATION for n in MUSCLES})
    per_muscle = {}
    for solo in MUSCLES:
        knee.setValue(state, ang, True)
        per_muscle[solo] = knee_udot({solo: ACTIVATION})
    cases.append({
        "knee_rad": ang,
        "activation": ACTIVATION,
        "full_activation_knee_udot": full,
        "solo_knee_udot": per_muscle,
    })

with open(OUT, "w") as fp:
    json.dump({
        "source": "OpenSim " + osim.GetVersion(),
        "model": OSIM.split("/")[-1],
        "muscle_model": "Millard2012EquilibriumMuscle, ignore_tendon_compliance=True (rigid)",
        "setup": "reduced right leg 5-DOF (hip3+knee+ankle), pelvis+contralateral+torso+"
                 "subtalar+mtp LOCKED, GRAVITY OFF, activations set directly, speeds zeroed",
        "free_coords": FREE,
        "muscles": MUSCLES,
        "units": "knee_udot in rad/s^2; inertias in kg, m, kg*m^2; iquat (w,x,y,z) body->principal",
        "inertias": inertias,
        "cases": cases,
    }, fp, indent=1)

print(f"OpenSim {osim.GetVersion()} forward dynamics (gravity off, rigid, act=1.0):")
for c in cases:
    print(f"  knee={c['knee_rad']:+.2f}  full_act knee_udot={c['full_activation_knee_udot']:+.1f} rad/s^2")
