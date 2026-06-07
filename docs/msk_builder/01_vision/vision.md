# Vision — the musculoskeletal builder

*2026-06-07.*

## What we are building toward

A pipeline that ingests a **3D body scan** of a real person and emits a **simulatable
articulated model** — bones as rigid bodies, joints with anatomical centers and axes, and
muscles/tendons routed across them — that loads in our own physics engine and moves correctly.

This is the **"Body in → Twin built"** stretch of the mission loop (`MISSION.md`, thesis steps
1–2). The twin's fidelity caps everything downstream — "every gradient and every device inherits
its error" — so we hold this to a *validated* standard, not a plausible one, and we build it on
the thing closest to the capstone (a powered knee orthosis/exo) rather than the easiest thing.

## The keystone idea — an oracle, for free

The naive reading of "validated against measurement" is that validation needs hardware and a
measured human. It does not — not for the first gate.

The biomechanics field already has **literature-validated** musculoskeletal models: OpenSim's
lower-limb models (gait2392, Rajagopal) carry decades of cadaver- and imaging-derived joint
centers, muscle attachment points, and moment-arm curves. If we **bridge one into our MJCF**,
that converted model plays two roles at once:

1. **Template** — the reference anatomy we scale/morph to fit a particular scan.
2. **Oracle** — the ground truth we grade our result against (its joint center, its moment-arm
   curves), computed in-engine from our own tendon Jacobian.

So the first gate becomes **"does our scan-fitted knee reproduce the validated model's
kinematics?"** — answerable entirely in software. Hardware sim-to-real is a *later* gate, not a
prerequisite to the first one. This is the single decision that makes the program tractable and
honest at the same time.

## The four locked decisions (confirmed 2026-06-07)

1. **First target = the KNEE, modeled as a 1-DOF hinge at G1.** It is *the* exo/orthosis joint
   (capstone-aligned, load-bearing) and the most-validated joint in the OpenSim corpus. The knee
   is also biomechanically the *hardest* major joint (rolling-gliding, moving instant center) —
   so we **deliberately defer** that fidelity: G1 treats it as a fixed hinge and banks the
   moving-center work for a later gate, with the validated ROM bounded to where a fixed hinge is
   honest (see the G1 recon, risk R3).

2. **Fitting method = template registration + scan-driven scaling.** Morph the validated OpenSim
   template using landmarks/measurements detected on the scan — the analogue of OpenSim's own
   Scale tool, driven by scan geometry instead of marker pairs. We do **not** synthesize muscle
   paths from scratch: that would put the research risk precisely where it is hardest to check.

3. **Interop = bridge OpenSim `.osim` → our MJCF, and spike it FIRST.** Converter fidelity is the
   #1 risk, so the very first action is a throwaway spike that converts one knee + four muscles
   and checks the moment arms against the published curves. The converted model is both template
   and oracle (above).

4. **G1 gate = articulated kinematics validated vs the OpenSim oracle** (no hardware). G2 adds
   Hill-muscle-driven motion; physical sim-to-real is a later gate that ties into system
   identification (Mission deliverable #3).

## Why this is the right first bite

- It **recycles the whole front of the repo** — the scan pipeline, the mesh suite, and the
  rigid-body engine — and adds only the genuinely-missing connective tissue (an MJCF emitter,
  the OpenSim bridge, landmark detection, and a scaling solver).
- It produces, at G1, a **personalized articulated knee** — already a useful artifact for the
  orthosis path, and the rigid+tendon scaffold the soft-tissue FEM will later couple to.
- It establishes a **validation harness** (the oracle comparison) that every later joint and the
  whole-body generalization can be graded against.

See `../03_phases/g1_knee_kinematics/recon.md` for the empirical baseline, gap table, and the
S0→S5 execution ladder.
