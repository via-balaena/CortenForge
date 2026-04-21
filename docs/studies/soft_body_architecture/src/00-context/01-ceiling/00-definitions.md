# Physically correct, visually great, real-time, differentiable

The [parent Ch 01](../01-ceiling.md) stakes the ceiling on a claim: the ceiling is *measurable*, not aspirational. This sub-chapter names the four axes the claim rests on and gives each a testable criterion — the operational test a candidate solver passes or fails. The four criteria also back the rightmost columns of the [SOTA survey](../02-sota.md); every ✓/partial/— cell in that table is one of the four tests answered for one of the nine solvers.

Each axis below names the components that compose it, the testable criterion that checks them, and the scoring rubric that maps test outcomes to ✓/partial/— for the [Ch 02 cross-compare row](../02-sota/09-cross-compare.md).

## Physically correct

Four components:

- **Hyperelastic constitutive law with near-incompressibility.** An energy density $\Psi(F)$ with a volumetric term that diverges as $J = \det F \to 0$ — neo-Hookean, Mooney-Rivlin, or Ogden per [Part 2 Ch 04](../../20-materials/04-hyperelastic.md). Linear elasticity and mass-spring networks fail this component.
- **IPC-grade non-penetration contact.** Contact as a smooth barrier $b(d)$ that diverges as the gap $d \to 0^+$, per [Part 4 Ch 00](../../40-contact/00-why-ipc.md) — not as a penalty spring that stores overlap energy, not as an impulse that jumps configuration. Non-penetration is a guarantee, not a convergence property of the timestep.
- **Implicit time integration on total potential energy.** Backward Euler with Newton on $U_\text{elastic} + U_\text{contact} + U_\text{inertia}$ per [Part 5 Ch 00](../../50-time-integration/00-backward-euler.md). Explicit schemes and PBD projection loops — which have no total potential — fail this component.
- **Measured-data constitutive calibration.** Parameters fit to uniaxial, biaxial, and viscoelastic-DMA data per [Part 1 Ch 04](../../10-physical/04-material-data.md), not hand-tuned for visual plausibility.

Testable criterion: the solver reproduces a measured force-displacement curve on a known benchmark (compressed cube, cantilever under tip load, contact squeeze on Ecoflex 00-30) within the validity regime from [Part 2 Ch 00 §02](../../20-materials/00-trait-hierarchy/02-validity.md).

Scoring: all four components plus criterion passes scores `✓`; most-but-not-all components (e.g., hyperelastic + implicit but penalty contact instead of IPC) scores `partial`; multiple components missing (e.g., linear elasticity + explicit integration, as in most PBD-based stacks) scores `—`.

## Visually great

Three components:

- **The FEM mesh is the render mesh.** No skinning layer that maps a low-resolution physics mesh onto a high-resolution render mesh with per-vertex rigging. [Part 9 Ch 03](../../90-visual/03-subdivision.md)'s subdivision is the allowed resolution pathway — smooth interpolation of the same mesh, not a separate rigged artist asset.
- **Per-vertex physics attributes feed the shader directly.** Stress, temperature, contact pressure, and deformation gradient reach the shader as vertex data, per [Part 9 Ch 05](../../90-visual/05-sim-bevy.md). Contact-pressure-driven micro-wrinkle normals and thermal overlays originate in the physics and render as a consequence — not from baked normal maps or animation-team-tuned keyframes.
- **No secondary-motion module.** Wobble, settling, and jiggle come from the [viscoelastic material model](../../20-materials/07-viscoelastic.md), not from a post-hoc animation layer with its own damping tuning knobs.

Testable criterion: the `SoftScene` description from [Part 1 Ch 03](../../10-physical/03-thesis.md) has four fields — `geometry`, `material`, `boundary`, `partners`. A fifth field for rendering — skinning weights, animation curves, a normal-map bakery — is a failure.

Scoring: all three components plus the four-field scene structure scores `✓`; a pipeline that inserts a rendering layer between physics and pixels but still reads some physics attributes scores `partial`; a pipeline that forgoes physics-driven visuals entirely — offline engineering renders, no shader integration — scores `—`.

## Real-time

Two components:

- **60+ FPS on a consumer GPU at a target tet count.** [Part 11 Ch 03 Phase E](../../110-crate/03-build-order.md#the-committed-order) names the target envelope: ≥ 30 FPS at experience-mode resolution (~5 k tets), ≥ 5 Hz at design-mode resolution (~30 k tets). These are *targets derived by scaling Warp's published per-timestep neo-Hookean benchmark numbers to a 60/5 Hz frame budget on comparable tet counts* — not measurements of `sim-soft`. The ceiling test is whether the architecture can hit the envelope on paper; Phase E's shipping criterion is whether it does in practice.
- **Implicit Newton with bounded inner iterations and bounded CCD cost.** A solver whose per-frame cost varies by orders of magnitude with contact configuration fails this component even if the average-case number lands at 60 FPS.

Testable criterion: a published benchmark at or above the Warp-derived envelope above, with bounded worst-case cost.

Scoring: at or above the envelope with bounded worst-case scores `✓`; on GPU but with no published at-rate benchmark or unbounded worst-case scores `partial`; offline CPU or no published real-time benchmark scores `—`.

## Differentiable

Three components:

- **Implicit-function-theorem gradient on steady-state.** $\partial x^*/\partial \theta = -A^{-1} \partial r / \partial \theta$ per [Part 6 Ch 02](../../60-differentiability/02-implicit-function.md), where $A$ is the stiffness matrix factorization already on the tape from the forward solve — one extra back-substitution per backward pass.
- **Adjoint state on time-series.** Reverse-in-time accumulation per [Part 6 Ch 03](../../60-differentiability/03-time-adjoint.md) for rolled-out trajectories, with checkpointing per [Part 6 Ch 04](../../60-differentiability/04-checkpointing.md) for memory-bounded backprop.
- **Gradcheck at 5 digits plus determinism-in-θ.** The [Part 11 Ch 04 §02 FD-vs-IFT test](../../110-crate/04-testing/03-gradcheck.md) — 5-digit agreement on each composed-scalar gradient component — *and* the [§01 determinism-in-θ test](../../110-crate/04-testing/03-gradcheck.md) — bit-equal `RewardBreakdown` on repeated calls at the same θ, the γ-locked contract carried by `ForwardMap::evaluate`.

Testable criterion: the solver passes §01 + §02 on a 100-tet neo-Hookean cube and on its contact extension.

Scoring: all three components with gradcheck discipline scores `✓`; partial gradient machinery — FD-only wrappers, steady-state-adjoint without time-adjoint, or no gradcheck harness — scores `partial`; no gradient path scores `—`.

## What this commits Ch 02 to

Every entry in the [SOTA survey](../02-sota.md) is scored against the four axes above. The ✓/partial/— cells are not editorial judgments; they are answers to the four tests. A solver that hits three of four gets three cells at their respective ✓/partial/— — not a composite rating. [Ch 02 §09 cross-compare](../02-sota/09-cross-compare.md) is the full side-by-side; [§02 the-gap](02-the-gap.md) reads the empty cells as the integration surface `sim-soft` ships.
