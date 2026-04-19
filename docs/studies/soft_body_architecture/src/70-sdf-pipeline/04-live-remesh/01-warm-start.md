# Warm-start from previous mesh

The [Ch 04 parent's Claim 3](../04-live-remesh.md) names warm-starting as where the interactive-rate story lives: a design edit that does not change the tet topology can reuse the previous mesh's connectivity, the previous converged state $x^\ast_\text{old}$ as the Newton initial guess, and — for parameter-only edits — even the [cached Hessian factorization](../../50-time-integration/00-backward-euler.md). This leaf writes down the two warm-start paths, what each reuses and invalidates, and why the Hessian-factor reuse collapses the parameter-only cost into back-substitutions.

## ParameterOnly: the ≤50 ms hot path

A parameter-only edit moves the geometric SDF's boundary by less than one iso-surface cell (the gate from [§00 change-detection](00-change-detection.md) has already verified that topology is preserved). The per-tet material cache does not need to change either — the `MaterialField` is unchanged for this edit class. What does need to happen:

1. **Replace `scene.sdf` with `new_sdf`.** The field trait objects in `SdfField` swap; the tet mesh and material cache stay.
2. **Update the Newton initial guess to $x^\ast_\text{old}$.** The previous converged equilibrium is the warm start — for small boundary perturbations, the new equilibrium $x^\ast_\text{new}$ is close, and Newton converges in a handful of iterations from this initial guess rather than from a cold start.
3. **Reuse the cached Hessian factor.** The parameter-only edit moves boundary *positions* only; the per-tet stiffness blocks $K^e$ are functions of the tet's reference-frame geometry (via the strain-displacement matrix $B$) and material parameters, both of which are unchanged. The full Newton Hessian $H = K + H_\text{contact} + M/\Delta t^2$ therefore sees no change in $K$ or $M$, and the contact block $H_\text{contact}$ changes only if the new boundary moves an active contact pair. For the common case (geometric perturbation not crossing the active set), the cached $L L^T$ factor from the previous step is still valid.
4. **Solve Newton iterates as back-substitutions.** Each Newton step is $\Delta x^{(k)} = -L^{-T} L^{-1} g^{(k)}$ — two triangular solves against the cached factor, no refactorization. A handful of iterates ≤ convergence tolerance; the wall-time cost is dominated by the residual evaluation (per-element constitutive pass) rather than the linear solve.

The ≤50 ms budget is achievable because steps (1)–(2) are microseconds (pointer swap + state copy), step (3) is zero-cost (the factor is already there), and step (4) is O(Newton iterations × (back-substitution cost + per-element residual cost)). Back-substitutions against a sparse SPD factor and the per-element constitutive residual evaluation are both bounded by canonical-scale mesh size; the handful of Newton iterates ([Ch 04 parent](../04-live-remesh.md)'s "converges in a handful of iterations" phrasing) times the per-iterate cost lands inside the 50 ms target in practice, though the specific factor is a Phase-B benchmark deliverable.

## MaterialChanging: the ≤200 ms path

A material-changing edit (the `MaterialField` moved, with or without a parameter-only geometric move) preserves topology but invalidates the per-tet material cache. What needs to happen:

1. **Replace `scene.sdf` and `scene.material` with new values.**
2. **Re-sample the `MaterialField` at every tet's evaluation point** (per [Ch 02 §00 sampling](../02-material-assignment/00-sampling.md)). For 30k Tet4 tets, this is ~30k × (slots) evaluations, typically low-millisecond-scale.
3. **Invalidate the Hessian cache.** The per-element stiffness blocks $K^e$ are now different (materials changed), so the cached factor no longer applies. Schedule re-assembly + re-factor on the next Newton step.
4. **Warm-start Newton from $x^\ast_\text{old}$.** The position is still close to the new equilibrium for modest material changes; Newton convergence is faster than cold-starting.
5. **Assemble and factor the new Hessian.** The full-mesh assembly walks the new per-tet stiffness blocks and accumulates into a sparse CSR matrix; the factorization is via `faer::sparse::linalg::solvers::Llt` (the standard SPD sparse factor path from [Part 5 Ch 00](../../50-time-integration/00-backward-euler.md) and the [factor-on-tape pattern](../../60-differentiability/02-implicit-function.md)'s forward side).
6. **Solve Newton iterates.** First iterate pays the factorization cost; subsequent iterates reuse the new factor via back-substitutions.

The ≤200 ms budget accommodates steps (2), (5), and Newton iteration at a 30k-tet scale. Step (5) — the sparse factorization — is typically the single most expensive item on this path, and is what makes material-changing edits ~4× slower than parameter-only.

## The cached-factor pattern

`SoftScene` carries an `Option<Llt<f64>>` field (the cached sparse-SPD factor from `faer::sparse::linalg::solvers`) alongside its mesh, materials, and converged state. The warm-edit arm of [Ch 04 spine's `apply_design_edit`](../04-live-remesh.md) consumes the cache:

- **ParameterOnly:** the cache is `Some(factor)`; Newton solves via back-substitution only; the factor remains cached for the next edit.
- **MaterialChanging:** invalidate the cache to `None`; Newton's first iterate assembles + factors a fresh matrix; the new factor is stored for subsequent iterates and the next edit.

The cached-factor pattern mirrors the [Part 5 Ch 00 factor-on-tape commitment](../../50-time-integration/00-backward-euler.md) at the design-edit timescale. Inside a single backward-Euler step, the factor is reused across Newton iterates; across design edits, the factor is reused across entire re-solves when the stiffness is unchanged. The two reuse cadences layer cleanly.

## When the cached factor is invalidated

Four invalidation triggers, in order of granularity:

- **Material parameters changed** (`MaterialChanging` edit class). Invalidates because $K^e$ depends on material parameters.
- **Active contact set changed** ([Part 4 Ch 01 §01 adaptive-$\kappa$](../../40-contact/01-ipc-internals/01-adaptive-kappa.md)'s barrier activation crossing). Invalidates because $H_\text{contact}$ changes structure when a pair enters or leaves the active set.
- **Timestep $\Delta t$ changed** ([Part 5 Ch 02 adaptive-$\Delta t$](../../50-time-integration/02-adaptive-dt.md)). Invalidates because $M/\Delta t^2$ scales with $\Delta t$; the inertial block changes.
- **Topology changed** (`TopologyChanging` edit class). The mesh and cache are replaced entirely; this is the [§02 state-transfer](02-state-transfer.md) path, not a cache-invalidation-then-refactor path.

Outside these triggers, the factor persists. The common case — parameter-only edits that do not touch the contact set — hits all triggers negative and the factor is reused across many consecutive edits. This is the interactive-rate path's structural advantage.

## What this sub-leaf commits the book to

- **ParameterOnly edits are back-substitutions against a cached factor.** The ≤50 ms budget rests on this; no Hessian assembly, no factorization, just residual evaluation + triangular solves.
- **MaterialChanging edits pay the assembly + factorization cost but warm-start Newton from $x^\ast_\text{old}$.** The ≤200 ms budget accommodates the ~4× step cost difference from factor re-computation.
- **The cached factor is a `faer::sparse::linalg::solvers::Llt<f64>`.** Same type as the [Part 5 Ch 00 factor-on-tape](../../50-time-integration/00-backward-euler.md) primary path.
- **Four invalidation triggers: material-changed, contact-set-changed, timestep-changed, topology-changed.** The first three are in-place invalidations on the same mesh; the fourth routes through [§02 state-transfer](02-state-transfer.md) with a fresh mesh.
- **Invariant: while cached, the factor corresponds to the current $(K, H_\text{contact}, M/\Delta t^2)$ triple.** Invalidation on any of the first three (in-place) triggers above is the mechanism preserving this invariant; the fourth (topology-changing) trigger drops the cache entirely with the old mesh.
