# Vision — the musculoskeletal builder

*2026-06-07. Reframed 2026-06-07 to **parametric-builder-first**; the original scan-first
framing is preserved verbatim in §"How the framing shifted" at the bottom.*

## What we are building toward

A **parametric musculoskeletal body builder**: it generates a **simulatable articulated body**
— bones as rigid bodies, joints with anatomical centers and axes, and muscles/tendons routed
across them — from an **anatomical part library** (the analogue of `cf-anthro` for *external*
anthropometry, but for the *internal* skeleton + soft routing). The body loads in our own physics
engine and moves correctly. It needs **no scan to exist.**

A **3D body scan, when one is available**, is an **optional downstream layer**: it contributes a
small vector of **parameters** (segment lengths, girths, joint centers — see the tiers below) that
*morph* the canonical body toward a particular person. The scan never supplies raw geometry. It is
an **accuracy upgrade, not a prerequisite.**

This is the **"Body in → Twin built"** stretch of the mission loop (`MISSION.md`, thesis steps
1–2). The twin's fidelity caps everything downstream — "every gradient and every device inherits
its error" — so we hold this to a *validated* standard, not a plausible one, and we build it on
the thing closest to the capstone (a powered knee orthosis/exo) rather than the easiest thing.

### Why builder-first (the emphasis inversion)

The earlier framing led with the scan: *ingest a scan → emit a model.* That makes the **scanner a
blocking dependency on everything downstream** — the exo, the RL controller, system-ID, and
sim-to-real all need a body *now*, and sub-$1000 scanners produce noisy, inconsistent, hole-ridden
meshes. Gating the whole program on bad hardware is the wrong order.

Inverting it — **build the body parametrically, fold the scan in as a parameter source** —
removes that gate. We can drive the entire capstone loop on **canonical / parametric clones** and
add scan-fitting only where it measurably improves fidelity. Two bonuses fall out:

- **Free training data.** A parametric body can be domain-randomized to generate ground-truth
  for the landmark detector, the RL policy, and system-ID priors. `cf-anthro::synthetic` already
  does this in miniature (it validates the knee detector against synthetic legs with known truth).
- **A canonical frame.** Fitting becomes low-dimensional regression onto a strong prior instead
  of raw-mesh processing — which is exactly what makes it robust to a bad scanner.

This is well-trodden ground, not a gamble: it is the **statistical-shape-model + atlas-registration**
approach used across biomechanics (OpenSim's Scale tool, SMPL/STAR for skin, open bone SSMs).
Fitting a strong anatomical prior to sparse, noisy observations is the textbook robust answer to a
low-quality scanner.

## The keystone idea — an oracle, for free

The naive reading of "validated against measurement" is that validation needs hardware and a
measured human. It does not — not for the first gate.

The biomechanics field already has **literature-validated** musculoskeletal models: OpenSim's
lower-limb models (gait2392, Rajagopal) carry decades of cadaver- and imaging-derived joint
centers, muscle attachment points, and moment-arm curves. If we **bridge one into our MJCF**,
that converted model plays *three* roles at once:

1. **Template** — the reference anatomy we scale/morph to fit a particular person.
2. **Oracle** — the ground truth we grade our result against (its joint center, its moment-arm
   curves), computed in-engine from our own tendon Jacobian.
3. **Library seed** — the validated source the parametric part library is built from, so we do
   **not** author anatomy from scratch (which is where the research risk is worst; see decision 2).

So the first gate becomes **"does our (scan-fitted or canonical) knee reproduce the validated
model's kinematics?"** — answerable entirely in software. Hardware sim-to-real is a *later* gate,
not a prerequisite to the first one. This is the single decision that makes the program tractable
and honest at the same time.

## The scan as a parameter source — not the entry point

When a scan does arrive, the design question is **how much it is allowed to deform the body.**
Three tiers, ordered by personalization *and* by fragility against bad scanners:

- **(a) scan → scalar parameters only** — segment lengths, girths, joint centers; scale/morph the
  template. Bulletproof against bad scanners: a noisy mesh can still yield a robust girth. **This
  is what `cf-msk-fit` does today** (a single similarity transform pinning the knee center).
- **(b) scan → a few shape-model PCs** — a low-dimensional SSM fit. More personal, still robust
  *because the shape model regularizes the noise away*.
- **(c) scan → free-form surface deformation to match the mesh** — most personal, least robust;
  it drags us straight back into **trusting bad geometry**, the exact problem builder-first exists
  to escape.

**Locked direction: ship (a), design the parameter interface so (b) drops in later, never do (c)
blind.** The discriminator for any candidate parameter is one question: *could a sub-$1000 scanner
measure this robustly?* Lengths, girths, joint centers — yes. Per-vertex surface truth — no.

## Honesty — what a synthetic body does and does not prove

The discipline is "validated, not plausible," so two claims stay separate:

- A canonical/parametric body **validates the machinery** — that the pipeline composes, the body
  articulates correctly, and RL/control/system-ID train against it. This is real and useful *now*.
- It is **not** a validated *personal* twin. "A clone of something similar to a real body" is
  perfect for proving the system; it **cannot** claim per-person fidelity. Personalization is
  *earned* later by the scan-fit layer (tier a→b) and, eventually, by hardware sim-to-real.

Keep those as two claims and the program stays honest while it moves fast.

## The four locked decisions (confirmed 2026-06-07)

*(Unchanged by the reframe — the inversion is about emphasis and entry point, not method. Decision
2 in particular is what the builder-first library leans on.)*

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
   *(Builder-first reading: the same template is also the canonical body when no scan is present.)*

3. **Interop = bridge OpenSim `.osim` → our MJCF, and spike it FIRST.** Converter fidelity is the
   #1 risk, so the very first action is a throwaway spike that converts one knee + four muscles
   and checks the moment arms against the published curves. The converted model is both template
   and oracle (above).

4. **G1 gate = articulated kinematics validated vs the OpenSim oracle** (no hardware). G2 adds
   Hill-muscle-driven motion; physical sim-to-real is a later gate that ties into system
   identification (Mission deliverable #3).

## Why this is the right first bite

- It **recycles the whole front of the repo** — the mesh suite and the rigid-body engine — and
  adds only the genuinely-missing connective tissue (an MJCF emitter, the OpenSim bridge, the
  anatomical part library, landmark detection, and a scaling solver).
- It produces, *with or without a scan*, a **personalized-or-canonical articulated knee** — already
  a useful artifact for the orthosis path, and the rigid+tendon scaffold the soft-tissue FEM will
  later couple to.
- It establishes a **validation harness** (the oracle comparison) that every later joint and the
  whole-body generalization can be graded against.

See `../03_phases/g1_knee_kinematics/recon.md` for the empirical baseline, gap table, and the
S0→S5 execution ladder.

## How the framing shifted

The original vision (2026-06-07) opened: *"A pipeline that ingests a 3D body scan of a real person
and emits a simulatable articulated model … This is the 'Body in → Twin built' stretch."* Scan-first.

The reframe (same day, after the sub-$1000-scanner bottleneck surfaced) **inverts the entry point,
not the method**: build the body parametrically from the validated library; treat the scan as an
optional tier-(a) parameter source. Every locked decision survives unchanged — the template,
oracle, knee-first, and registration-based scaling all still hold; the scan simply stops being the
thing the program waits on.
