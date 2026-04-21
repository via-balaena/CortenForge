# NVIDIA Flex — position-based, GPU

Repo: [`github.com/NVIDIAGameWorks/FleX`](https://github.com/NVIDIAGameWorks/FleX) · License: LICENSE.txt present, specific name soft-deferred (likely NVIDIA proprietary / source-available per GameWorks pattern; do NOT claim MIT or Apache without verification) · Last release: **1.1.0 on 2017-04-10** (unmaintained/legacy; superseded-in-practice by [Warp](01-warp.md) for NVIDIA's current GPU-physics roadmap) · Primary paper: Macklin, Müller, Chentanez, Kim 2014 "Unified Particle Physics for Real-Time Applications," SIGGRAPH 2014 · Accessed: 2026-04-20

Flex is NVIDIA GameWorks' unified particle-based physics simulator, shipped for Unity (FleX for Unity 1.0 beta) and Unreal Engine 4 (GameWorks integration). Soft-body is particle-based: cluster-shape-matching + position-based constraints, descended from the Müller et al. PBD lineage. Not a continuum FEM hyperelastic solver; does not attempt axis-1 correctness.

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `—`. Particle-based (PBD/XPBD lineage). No continuum hyperelasticity, no total-potential-energy formulation. Position-based contact, not IPC. Fundamentally a different class from axis-1 correctness.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `✓`. Production-grade integration in game engines (Unity, Unreal): particle outputs feed directly into the engine's rendering pipelines. The physics-to-pixels pathway is tight — no secondary-motion fakery because the physics runs at frame rate and particle clusters drive mesh deformation directly.
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `✓`. Production-mature GPU real-time. Target frame rates are game-engine baseline (60+ FPS) on consumer NVIDIA hardware. Bounded per-frame cost under the particle-count + constraint-iteration budget.
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `—`. No AD. Position-based constraint projection is not smooth in the way axis-4 requires; the step functions of constraint satisfaction are not differentiable at contact boundaries.

## What `sim-soft` inherits or learns

Flex is a **negative example the design has to beat** on axis 1 and axis 4 simultaneously: it demonstrates that a GPU soft-body engine can hit real-time at game-engine budget *and* can deliver visual quality via the production pixel pathway, but it gives up physical correctness and differentiability to do so. [Part 1 Ch 03](../../10-physical/03-thesis.md)'s thesis argues exactly that the tradeoff Flex made — PBD over hyperelastic + IPC — is no longer required in 2026. `sim-soft`'s architectural bet is that the same real-time + visual-quality envelope is reachable with hyperelastic FEM + [IPC-grade contact](../../40-contact/00-why-ipc.md) on [wgpu](../../80-gpu/00-wgpu-layout.md) without the axis-1 + axis-4 sacrifice.

## Citation status

- Repo + last-release-tag (1.1.0 on 2017-04-10) confirmed.
- Exact license name — soft-deferred to Pass 2 (LICENSE.txt present at repo root; content not fetched; do NOT claim MIT or Apache without verification).
- Primary paper: Macklin, Müller, Chentanez, Kim 2014 "Unified Particle Physics for Real-Time Applications," SIGGRAPH 2014 — author + title + venue confirmed; DOI soft-deferred to Pass 2.
- "Superseded-in-practice" characterization — accurate per absence of repo activity since 2017-04 and NVIDIA's current Warp-focused roadmap; "officially deprecated" is NOT a claim supported by fetched sources (repo not formally archived).
- Specific per-scene real-time benchmarks — soft-deferred to Pass 2 (published Flex numbers exist in the game-engine context; not fetched for this pass).
