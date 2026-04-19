# State transfer across re-meshes

The [Ch 04 parent's Claim 4](../04-live-remesh.md) names state transfer as where differentiability breaks: a topology-changing edit produces a new mesh whose combinatorics differ from the old, and the transfer of the previous converged state $x^\ast_\text{old}$ onto the new mesh's vertices requires looking up each new vertex's enclosing old-mesh tet — a piecewise-constant operation that is not smooth in the design parameter. This leaf writes out the transfer rules, the non-smoothness argument, and the FD-wrapper handoff.

## Barycentric interpolation from the old mesh

For each new-mesh vertex at reference-space position $\mathbf{x}_\text{ref}^\text{new}$:

1. **Enclosing-tet query.** Find the old-mesh tet whose interior contains $\mathbf{x}_\text{ref}^\text{new}$. Standard spatial-indexing machinery — an AABB tree or BVH over the old mesh's tet bounding boxes, then a point-in-tet test on the candidates. $O(\log N)$ per query, built once per old mesh at state-transfer time.
2. **Barycentric weights.** For the enclosing tet with vertices $\mathbf{x}_1, \mathbf{x}_2, \mathbf{x}_3, \mathbf{x}_4$ and their state values $\mathbf{x}^\ast_1, \mathbf{x}^\ast_2, \mathbf{x}^\ast_3, \mathbf{x}^\ast_4$, the barycentric weights $(w_1, w_2, w_3, w_4)$ satisfying $\mathbf{x}_\text{ref}^\text{new} = \sum_i w_i \mathbf{x}_i$ with $\sum_i w_i = 1$ give the per-vertex interpolation.
3. **Interpolated per-vertex state.** The new vertex inherits $\mathbf{x}^\ast_\text{new} = \sum_i w_i \mathbf{x}^\ast_i$ for position, $\mathbf{v}^\ast_\text{new} = \sum_i w_i \mathbf{v}^\ast_i$ for velocity. Both are linear-operator-smooth in the $\mathbf{x}^\ast_i, \mathbf{v}^\ast_i$ inputs.

When a new vertex lies outside the old mesh entirely — the topology edit grew the body — the query returns no enclosing tet, and the new vertex's state defaults to a rest-configuration value ($\mathbf{x}^\ast_\text{new} = \mathbf{x}_\text{ref}^\text{new}$, $\mathbf{v}^\ast_\text{new} = \mathbf{0}$). This is the only cold region on the warm-started state; Newton has to converge the rest-region into equilibrium with the rest of the body, which is why the topology-changing path allows a full Newton re-solve rather than a few-iterate warm-up.

## Per-tet state inheritance

Per-element state (per-tet stress tensor $\sigma^e$, per-element Prony over-stress $Q_i^{n+1,*}$, per-element constitutive history) is inherited by assignment from the enclosing old tet, not by barycentric blend. For a new tet with centroid at $\mathbf{x}_\text{ref}^c$, find the enclosing old-mesh tet via the same spatial query, and copy the per-tet state directly.

Per-tet state is piecewise-constant on the old mesh (each tet carries one value), and blending per-tet values across four enclosing old tets would introduce interpolation artifacts in the rheological state (a non-physical blend of stress tensors from different material regions at a material-interface crossing). Direct assignment preserves the per-tet semantics.

The [Ch 02 interface-tet flag](../02-material-assignment/01-composition.md) is *not* inherited — it is a mesh-geometry-plus-SDF characterization (whether the new tet's centroid sits within one edge length of a material-interface SDF), and the new tet's geometry is different from any old tet's. The flag is re-evaluated from scratch during the [Ch 02 §00 sampling pass](../02-material-assignment/00-sampling.md) that the topology-changing path runs after state transfer.

## Where differentiability breaks

The enclosing-tet query is **piecewise-constant in the new-mesh vertex position**: two infinitesimally-close new vertices can resolve to different old tets if they straddle an old-mesh face, and the interpolation operator switches discretely. Pushing this back to the design-parameter level: consider a design-parameter sweep that continuously moves a primitive boundary across the canonical problem. At the continuous design-parameter value where the iso-surface topology changes, state transfer runs, and the enclosing-tet lookup for each new-mesh vertex is piecewise-constant in the post-topology-change design parameter.

The chain is therefore smooth in pieces — positions and velocities interpolate linearly in the old-mesh state; the enclosing-tet assignment is the one non-smooth step. [Part 6 Ch 05's three-part compromise](../../60-differentiability/05-diff-meshing.md) is what applies here:

- **Inside one topology epoch** (no topology changes during the sweep), state transfer does not run; the mesh is held constant, gradients flow through the IFT adjoint cleanly.
- **Across a topology-change boundary**, state transfer runs, and [Part 6 Ch 05 §03's FD wrapper](../../60-differentiability/05-diff-meshing/03-fd-wrappers.md) takes over. Finite-difference gradients are computed across the topology change, the result is flagged with `GradientEstimate::Noisy { variance }`, and the optimizer handles the noise.

The [stochastic-adjoint upgrade](../../60-differentiability/03-time-adjoint/02-stochastic.md) from Part 6 Ch 03 §02 is the Phase-H path to replace FD with a proper stochastic-gradient treatment; the deferral is explicit per the Ch 04 parent's commitment.

## The full cold-start

Once state has been transferred onto the new mesh, the Newton re-solve runs cold — no cached Hessian (the mesh connectivity changed, so $K$'s sparsity pattern changed), full assembly + factorization on the first iterate, full Newton loop until convergence. The target is ≤500 ms end-to-end at design-mode resolution per [Ch 04 Claim 1](../04-live-remesh.md); the realistic timing depends on how close the transferred state is to the new equilibrium, which in turn depends on how much the topology change actually moved the body.

A design edit that adds a small cavity far from the loaded region produces a transferred state that is already close to the new equilibrium everywhere except in the new cavity region; Newton converges quickly. A design edit that changes the load path (e.g., opens a through-hole between a previously-sealed cavity and the external boundary) produces a transferred state that is far from equilibrium across the whole body; Newton takes the full cold-start budget.

## What this sub-leaf commits the book to

- **Vertex state (position, velocity) transfers via barycentric interpolation from the enclosing old-mesh tet.** Linear-smooth in the old-mesh state; non-smooth in the enclosing-tet assignment.
- **Per-tet state (stress, Prony history, interface flag) transfers by assignment, not blending.** Preserves per-tet semantics; avoids non-physical blends across material regions.
- **New vertices outside the old mesh get rest-configuration state.** Position = reference position, velocity = 0; Newton converges the cold region on first re-solve.
- **Differentiability breaks at the enclosing-tet assignment.** [Part 6 Ch 05 §03 FD wrapper](../../60-differentiability/05-diff-meshing/03-fd-wrappers.md) is invoked across topology-change boundaries; `GradientEstimate::Noisy { variance }` flag propagates to the optimizer.
- **Stochastic-adjoint upgrade in Phase H is the planned long-term replacement.** Scheduled per [Part 6 Ch 03 §02](../../60-differentiability/03-time-adjoint/02-stochastic.md); FD is the Phase-D interim.
- **The ≤500 ms topology-change budget assumes the transferred state is close to the new equilibrium.** Large topology changes that alter load paths may exceed it; [§00 change-detection](00-change-detection.md) does not distinguish small from large topology edits, so the budget is a best-effort target rather than a hard guarantee.
