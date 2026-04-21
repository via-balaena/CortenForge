# Houdini FEM Solver — production offline

Source: SideFX Houdini documentation (closed-source / commercial) · Primary reference: Houdini docs for Solid Object DOP / Finite Element Solver · Accessed: 2026-04-20

Houdini's FEM Solver (Solid Object DOP / Finite Element Solver) is the volumetric soft-body path in SideFX Houdini — true finite-element simulation on tet meshes with hyperelastic material support, implicit time integration, and deep integration with Houdini's Mantra / Karma rendering pipeline. Commercial closed-source; the solver's internals are not public, but documented features and inputs/outputs allow external characterization. Note: "Jelly" in the parent survey refers to this FEM Solver, not to Houdini's [Vellum](https://www.sidefx.com/docs/houdini/nodes/sop/vellumsolver.html) XPBD solver (which is the engine used for cloth, hair, and grains — see the heterogeneous-stack paragraph below).

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `partial`. Hyperelastic material support confirmed via Houdini docs; implicit integration confirmed; measured-data parameters supported (configurable constitutive parameters). The **contact formulation** is constraint-based, not IPC-grade — the specific algorithmic method (penalty, augmented Lagrangian, or other) is not independently verified in the 2026-04-20 fetch; soft-defer to Pass 2. Missing the IPC component of axis 1.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `✓`. Production-grade physics-to-pixels integration via Mantra/Karma: per-vertex physics attributes (stress, deformation) reach the renderer directly, with subdivision surfaces on the FEM mesh. This is the production ceiling for FEM-driven shading.
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `—`. Offline/production focus — designed for VFX workflows where frame-budget is measured in minutes-to-hours on render farms, not milliseconds on a single GPU.
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `—`. No AD. Not a design-study goal for VFX production.

## What `sim-soft` inherits or learns

Houdini FEM Solver is the reference for **axis 2 (visually great)** at the production ceiling — the proof that per-vertex-physics-attribute + shader integration produces visual quality without a separate rendering hack layer. [Part 9 Ch 00 — SSS](../../90-visual/00-sss.md) and [Part 9 Ch 05 — sim-bevy](../../90-visual/05-sim-bevy.md) cite this production-quality pathway as the target `sim-bevy` approximates. `sim-soft`'s relationship is as a **visual-quality reference baseline** and a **negative example on axes 3 and 4**: Houdini's FEM Solver achieves physical + visual but gives up real-time and differentiability to do so.

**Heterogeneous-stack contrast.** Houdini's production character pipeline does not unify skin, cloth, and hair in one solver. Soft-body flesh / skin runs on the FEM Solver; cloth and hair run on [Vellum](https://www.sidefx.com/docs/houdini/nodes/sop/vellumsolver.html) (XPBD); specialized solvers cover finer effects. The combined visual quality is the product of multiple solvers wired together at the scene level — a heterogeneous stack rather than a single integrated solver. `sim-soft`'s distinguishing architectural bet (per [§02 the-gap](../01-ceiling/02-the-gap.md)) is to unify skin + cloth + hair under one energy minimization with shared IPC contact and shared differentiable tape, rather than parallel specialized solvers coordinated at the scene level.

## Citation status

- Source: SideFX Houdini documentation. No primary research paper — the FEM Solver is in-house SideFX engineering. Cite the Solid Object DOP / Finite Element Solver docs page.
- Specific contact-formulation algorithm in Houdini FEM — soft-deferred to Pass 2 (documented as constraint-based but the specific method — penalty, augmented Lagrangian, or other — not independently verified).
- Houdini version citation — current documentation is for Houdini 21.0 (Vellum available since Houdini 17.0; FEM Solver lineage older); exact FEM Solver version history soft-deferred to Pass 2.

**Scoring-table tension (tracked for Ch 02-close):** this leaf scores Physical `partial`; the parent [SOTA survey table](../02-sota.md) shows `✓` (for the former "Jelly" row). Sub-leaf framing is more honest: missing IPC component disqualifies a full axis-1 `✓`. Parent table to be updated at Ch 02-close — both the row title rename ("Jelly" → "Houdini FEM Solver") and the Physical score adjustment.
