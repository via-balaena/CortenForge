# Thermodynamic Computing — CortenForge initiative

Long-horizon research line: **mechanical p-bits + Langevin thermostat
+ EBM + RL, design → simulate → 3D-print**. The probabilistic computing
element is a bistable mechanical structure (buckling beam, snap-through
latch, Brownian ratchet), not a transistor; the same parameters drive
the geometry (cf-design), the physics (sim-core + thermostat), the
energy-based model (ml-bridge autograd), and the controller (ml-bridge RL).

Branch: `feature/thermo-doc-review`. Run on the
[sharpen-the-axe discipline](./02_foundations/working_principles.md) —
recon depth before commitment, validation with margin, foundations
over scaffolding.

## Where to start

| If you want… | Read |
|---|---|
| The endpoint we're building toward | [`01_vision/vision.md`](./01_vision/vision.md) |
| The five concrete experiments (D1–D5) and their foundation status | [`01_vision/research_directions.md`](./01_vision/research_directions.md) |
| The single-sentence framing | [`01_vision/synthesis.md`](./01_vision/synthesis.md) |
| The operating principles for the initiative | [`02_foundations/working_principles.md`](./02_foundations/working_principles.md) |
| Where the thermo line is right now | [`02_foundations/current_state.md`](./02_foundations/current_state.md) |
| What CortenForge already has and what's missing | [`02_foundations/existing_substrate.md`](./02_foundations/existing_substrate.md) |
| The chassis design for `sim-thermostat` | [`02_foundations/chassis_design.md`](./02_foundations/chassis_design.md) |
| Open questions (Q1–Q6) | [`02_foundations/open_questions.md`](./02_foundations/open_questions.md) |
| The 7-phase build plan | [`03_phases/overview.md`](./03_phases/overview.md) |
| Dated history of *why* the plan looks the way it does | [`04_recon_log/`](./04_recon_log/) |
| The 2026-04-09 doc review pass | [`05_doc_reviews/2026-04-09_doc_review.md`](./05_doc_reviews/2026-04-09_doc_review.md) |
| Cross-cutting findings (currently empty) | [`06_findings/`](./06_findings/) |
| Full mdbook-style index | [`SUMMARY.md`](./SUMMARY.md) |
| The pre-refactor monolithic master plan (now a pointer) | [`MASTER_PLAN.md`](./MASTER_PLAN.md) |

## Spec Index

Child specs spawned from phases of the Gap will be linked here as they are
written.

- [`02_foundations/chassis_design.md`](./02_foundations/chassis_design.md) — bolt-pattern
  design document for the `sim-thermostat` crate. Defines the
  `PassiveComponent` trait, the `PassiveStack` builder + composer, the
  clone-footgun resolution via `install_per_env`, the orthogonal
  `Diagnose` trait, the `test_utils` chassis (Welford's algorithm),
  and the on-disk crate layout. Seven decisions (1-7), all RESOLVED.
  Read this *before* the Phase 1 spec — every Phase 1 implementation
  detail bolts into the chassis defined here.
- *(Phase 1 spec — `03_phases/01_langevin_thermostat.md` — is the next
  artifact to produce, drafted in a fresh session against the chassis
  design above.)*
