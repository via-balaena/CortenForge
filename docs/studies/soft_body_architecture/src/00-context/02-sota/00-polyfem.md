# PolyFEM — IPC + hyperelastic, offline

Repo: [`github.com/polyfem/polyfem`](https://github.com/polyfem/polyfem) · MIT · Primary paper: Schneider, Dumas, Hu, Bouaziz, Kaufmann, Jiang, Gao, Zhou, Jacobson, Panozzo 2019 "Decoupling Simulation Accuracy from Mesh Quality," SIGGRAPH Asia 2019 · Accessed: 2026-04-20

PolyFEM is a C++ finite element framework with a specific thesis: simulation accuracy should decouple from mesh quality via higher-order elements and non-conforming basis functions. Contact is integrated through the [IPC Toolkit](03-ipc-toolkit.md) from the same research group. Research-grade focus; offline CPU execution.

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `✓`. Hyperelastic (NeoHookean, SaintVenant-Kirchhoff, NonLinearElasticity in the Python API surface), IPC via `ipc-toolkit` dependency, implicit integration, measured-data parameterizable. Full four-component combination.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `—`. No integrated shader pipeline; offline batch-render only. No commitment to FEM-mesh-as-render-mesh.
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `—`. Offline CPU focus. No GPU kernel path.
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `—`. No documented autograd / IFT adjoint / time-adjoint support in fetched README or Python API docs. Soft-defer below.

## What `sim-soft` inherits or learns

PolyFEM is the reference implementation for **axis 1 (physically correct)** — hyperelastic FEM + IPC + implicit integration composed and shipped as one solver. [Part 4 Ch 00](../../40-contact/00-why-ipc.md)'s IPC commitment traces to [Li et al. 2020](../../appendices/00-references/00-ipc.md) from PolyFEM's research group; [Part 2 Ch 04](../../20-materials/04-hyperelastic.md)'s hyperelastic choices overlap. `sim-soft` inherits this as a dependency-in-spirit (same algorithmic underpinnings, re-implemented in Rust) and as a regression baseline for correctness on static-equilibrium test scenes. What PolyFEM does not commit to — real-time GPU execution, differentiability, integrated shading — is where `sim-soft`'s distinguishing integration lands.

## Citation status

- Repo + license + active-maintenance status confirmed from README (2026-04-20).
- Schneider et al. 2019 SIGGRAPH Asia — citation attribution confirmed; DOI and page numbers soft-deferred to Pass 2 (paper landing page not independently fetched).
- Material-model coverage beyond the three Python-API-listed constitutive laws — soft-deferred to Pass 2 (Mooney-Rivlin / Ogden availability in C++ core unverified from fetched sources).
- Implicit integration scheme (backward Euler vs. other) — soft-deferred to Pass 2 (time-dependent simulations documented; exact scheme not fetched).
- Differentiability add-ons — soft-deferred to Pass 2 (no AD or inverse-design documentation surfaced in 2026-04-20 fetch; possible recent add-ons should be re-checked in Pass 2).

**Scoring-table tension (tracked for Ch 02-close):** this leaf scores PolyFEM `Differentiable: —` based on fetched sources; the parent [SOTA survey table](../02-sota.md) currently shows `partial`. Sub-leaf is authoritative per [§00 rubric](../01-ceiling/00-definitions.md). Parent table to be updated at Ch 02-close unless Pass 2 surfaces a documented AD path.
