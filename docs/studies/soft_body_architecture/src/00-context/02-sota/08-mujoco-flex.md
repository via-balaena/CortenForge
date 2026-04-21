# MuJoCo flex — where we start from

Source: DeepMind MuJoCo 3.0+ release notes. Repo: [`github.com/google-deepmind/mujoco`](https://github.com/google-deepmind/mujoco) · Apache-2.0 (MuJoCo core) · Flex introduced in MuJoCo 3.0.0 (2023-10-18) · Accessed: 2026-04-20

MuJoCo flex is the deformable-object extension added to MuJoCo in the 3.0.0 release: "a new low-level model element used to define deformable objects as simplicial complexes that can be of dimension 1, 2 or 3, corresponding to stretchable lines, triangles or tetrahedra" (3.0.0 release notes). Official release-note qualifier: **"This feature is still under development and subject to change."** MuJoCo is the engine CortenForge is already built on, making flex the **Track 1B baseline** — the MuJoCo-native deformable path that Phase A–I regression-tests against.

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `partial`. Solver per [Part 12 Ch 01 — Track 1B](../../120-roadmap/01-track-1b.md) authoritative framing: **Newton + constraint-projection + penalty-contact**. Elasticity plugins exist; hyperelastic material coverage is not a full catalog. Contact is MuJoCo's signature convex-optimization penalty, **not IPC**. Missing the IPC component of axis 1.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `—`. MuJoCo's default rendering is basic functional visualization (OpenGL). No shader integration at the axis-2 ceiling.
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `✓`. MuJoCo is designed real-time from first principles; flex integrates within the same pipeline. Bounded per-frame cost under the simplicial-complex size budget. Production real-time on consumer hardware.
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `—`. MuJoCo's MJX (JAX port) offers differentiable rollouts for standard MuJoCo; whether flex is fully differentiable through MJX is not verified in the 2026-04-20 fetch (likely partial/beta given the "under development" status) — soft-defer to Pass 2 for a verified MJX-flex scoring.

## What `sim-soft` inherits or learns

MuJoCo flex is the **in-engine regression baseline** against which `sim-soft` validates. It is already in CortenForge's solver stack, so every `sim-soft` PR regression-tests against a flex scene on the same canonical problem. [Part 11 Ch 04 §02 — regression](../../110-crate/04-testing/01-regression.md)'s regression-vs-flex suite pairs flex outputs with `sim-soft` outputs at Phases B, C, D, H, and I on the [`Observable`](../../110-crate/01-traits/00-core.md) layer. The *direction of disagreement* matters more than absolute agreement: where flex and `sim-soft` diverge, the divergence should be explicable via the axis-1 + axis-4 architectural differences (hyperelastic + IPC + differentiable vs MuJoCo flex's constraint-projection + penalty + no-AD). A divergence that doesn't cash out to one of those architectural lines is a bug.

`sim-soft`'s relationship is as an **in-engine regression baseline** and a **direction-of-disagreement oracle** — not a dependency (MuJoCo flex is a sibling engine, not upstream) nor a negative example (MuJoCo flex is beta; judging it harshly would be premature).

## Citation status

- Release date: MuJoCo 3.0.0 on **2023-10-18** (verified via release notes + WebSearch cross-check; one fetch initially returned 2024 but subsequent cross-verification confirmed 2023-10-18).
- "Under development and subject to change" status — confirmed from 3.0.0 release notes verbatim.
- Solver framing "Newton + constraint-projection + penalty-contact" — authoritative per [Part 12 Ch 01](../../120-roadmap/01-track-1b.md) Track 1B commentary.
- MJX-flex differentiability — soft-deferred to Pass 2.
- Hyperelastic constitutive-model coverage in flex — soft-deferred to Pass 2 (elasticity plugins exist; specific catalog not independently verified).
- At-scale flex benchmarks — soft-deferred to Pass 2.
