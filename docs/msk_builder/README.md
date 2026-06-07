# Musculoskeletal Builder — parametric bones/tendons movable body

*Established 2026-06-07. Reframed 2026-06-07 to parametric-builder-first (see `01_vision/vision.md`).*

> **Build a simulatable bones/tendons articulated body parametrically from a validated anatomical
> library; fold a 3D scan in — when one exists — as an optional parameter source, not a prerequisite.**

This is **Mission deliverable #4 — the person-specific musculoskeletal model builder**
(`MISSION.md`: "scan → scaled skeleton + muscle routing"). It is the **"Body in → Twin
built"** half of the differentiable body-to-device loop (thesis steps 1–2) and the substrate
the powered knee-orthosis capstone is eventually built on.

**Builder-first.** The body generates from an anatomical part library (seeded from the validated
OpenSim corpus) and **needs no scan to exist** — that removes the sub-$1000 scanner as a blocking
dependency on everything downstream (exo, RL, system-ID). A scan, when available, contributes a
small **parameter** vector (lengths, girths, joint centers) that morphs the canonical body toward a
person; it never supplies raw geometry. It is an **accuracy upgrade, not a prerequisite**. See
`01_vision/vision.md` for the full reframe, the (a)/(b)/(c) scan-deformation fork, and the honesty
caveat (synthetic body validates the *machinery*, not personhood).

It is green-field — no anatomy/skeleton code exists yet — but it stands on a strong recyclable
base (a MuJoCo-aligned rigid-body engine with spatial tendons + a Hill muscle model, and a
mature scan→watertight-twin mesh pipeline). Per the mission's discipline ("quality and ceiling,
not speed"), the program **nails ONE joint — the knee — fully and *validated* before
generalizing.**

## The keystone idea

A literature-validated **OpenSim** lower-limb model, bridged into our **MJCF**, serves *three*
roles at once: the **library seed** the parametric body is built from, the **template** we
scale/morph to a person, *and* an **oracle** (its joint centers and moment-arm curves) to grade
against. That lets the first software gate be checked **with no hardware** — a
*literature model* stands in for a physical measurement at this stage. (Precision: "oracle" means
that validated model's geometry; "validated against measurement" in the mission sense — a real
instrumented body — remains a later gate. The oracle↔OpenSim cross-check is **done** (0.3 mm RMSE;
see the Status section below).)

## Folder map (mirrors `docs/thermo_computing/`)

| Path | Contents |
|---|---|
| `01_vision/vision.md` | Vision (parametric-builder-first), the keystone oracle idea, the scan-as-parameter-source fork, the honesty caveat, the four locked decisions, mission linkage |
| `02_foundations/existing_substrate.md` | What the repo already gives us to recycle, with file paths |
| `02_foundations/library_parameter_architecture.md` | Architecture sketch: the anatomical IR/library, `BodyParams`, the `realize()` morph, the pluggable `ParamSource` interface, crate boundaries, build order |
| `03_phases/g1_knee_kinematics/recon.md` | The active RECON: empirical baseline → thesis → end-state → gap table → decisions → sub-leaf ladder (S0→S5) → risks → G1 validation |

## Phase map

| Phase | Gate | Definition of done |
|---|---|---|
| **G1 — Knee kinematics** *(active)* | software, no hardware | a canonical knee skeleton+tendons in MJCF (scan-fitted via landmarks → scaling when a scan exists) that articulates inside the skin envelope, with joint center + moment-arm curves matching the OpenSim oracle within tolerance |
| **G2 — Muscle-driven** *(future)* | software | Hill muscles/tendons actuate the joint; force-length / moment-arm behavior under load |
| **G3+ — Sim-to-real** *(future)* | physical | validated against a real measurement (ROM / moment arm); ties into system-identification (Mission deliverable #3) |

## Status

**2026-06-07 — S0 + S1 complete and reviewed.** Four decisions locked with the user; gait2392
vendored (Apache-2.0) at `sim/L0/tests/assets/opensim_gait2392/`. Converter lives in `tools/cf-osim`.
**S0**: the bare 1-DOF hinge is rejected by data (misses 5 mm for all four muscles; dominant error =
the coupled tibial translation, *inverting* the prior patella assumption). **S1**: a coupled-knee
model (coupled translation + patella bodies) reproduces the re-derived OpenSim-geometry oracle within
5 mm for all four. A 3-reviewer adversarial pass (2026-06-07) found **no math/physics bugs** (the knee
convention was verified against OpenSim docs and the surprising finding reproduced independently);
its fixes — natural-cubic splines, mis-conversion guards, claim re-scoping — are applied. **The
oracle is now VALIDATED against real OpenSim 4.6** (the `opensim` PyPI wheel, run on this exact model):
our re-derivation reproduces OpenSim's knee moment arms to **0.3 mm RMSE** across all four muscles
(`tools/cf-osim/tests/opensim_cross_check.rs`, runs in CI off the vendored reference JSON). G1's
kinematic foundation is independently anchored. **Next:** S2 (scan landmark detection) + the clean
converter (G1 recon §"S1 remaining").

## Sequencing within the loop

This builder is the front half. The **keystone** (differentiable soft↔rigid coupling) and the
**co-design optimizer** are downstream of a trustworthy twin; this program produces the rigid
+ tendon skeleton that the soft-tissue FEM (`sim/L0/soft`) later couples to. We build the
skeleton to *validated* fidelity first.
