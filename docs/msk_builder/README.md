# Musculoskeletal Builder — scan → bones/tendons movable model

*Established 2026-06-07.*

> **Take a 3D human body scan and produce a simulatable bones/tendons articulated model.**

This is **Mission deliverable #4 — the person-specific musculoskeletal model builder**
(`MISSION.md`: "scan → scaled skeleton + muscle routing"). It is the **"Body in → Twin
built"** half of the differentiable body-to-device loop (thesis steps 1–2) and the substrate
the powered knee-orthosis capstone is eventually built on.

It is green-field — no anatomy/skeleton code exists yet — but it stands on a strong recyclable
base (a MuJoCo-aligned rigid-body engine with spatial tendons + a Hill muscle model, and a
mature scan→watertight-twin mesh pipeline). Per the mission's discipline ("quality and ceiling,
not speed"), the program **nails ONE joint — the knee — fully and *validated* before
generalizing.**

## The keystone idea

A literature-validated **OpenSim** lower-limb model, bridged into our **MJCF**, serves as
*both* the **template** we scale to a scan *and* an **oracle** (its joint centers and moment-arm
curves) to grade against. That lets the first software gate be checked **with no hardware** — a
*literature model* stands in for a physical measurement at this stage. (Precision: "oracle" means
that validated model's geometry; "validated against measurement" in the mission sense — a real
instrumented body — remains a later gate. The oracle↔OpenSim cross-check is **done** (0.3 mm RMSE;
see the Status section below).)

## Folder map (mirrors `docs/thermo_computing/`)

| Path | Contents |
|---|---|
| `01_vision/vision.md` | Vision, the keystone oracle idea, the four locked decisions, mission linkage |
| `02_foundations/existing_substrate.md` | What the repo already gives us to recycle, with file paths |
| `03_phases/g1_knee_kinematics/recon.md` | The active RECON: empirical baseline → thesis → end-state → gap table → decisions → sub-leaf ladder (S0→S5) → risks → G1 validation |

## Phase map

| Phase | Gate | Definition of done |
|---|---|---|
| **G1 — Knee kinematics** *(active)* | software, no hardware | scan → landmarks → scaled knee skeleton+tendons in MJCF that articulates inside the skin envelope, with joint center + moment-arm curves matching the OpenSim oracle within tolerance |
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
