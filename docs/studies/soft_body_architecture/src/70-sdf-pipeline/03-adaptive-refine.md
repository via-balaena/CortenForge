# Adaptive refinement from stress feedback

This chapter is an inherent leaf — no sub-chapters — because adaptive refinement is one well-scoped capability. It ships in [Phase H](../110-crate/03-build-order.md#the-committed-order) as a fidelity upgrade, not a Phase A–D default, and is independent of the [Ch 01 tet-generation strategy](01-tet-strategies.md): whichever generator produced the initial mesh, refinement subdivides specific tets in-place without re-running the generator.

## The trigger: stress, not strain, not geometry

After a Newton solve converges, `sim-soft::readout/` produces a per-tet stress tensor $\sigma_e$ (see [Part 11 Ch 00 readout](../110-crate/00-module-layout/09-readout.md)). The refinement criterion is a scalar derived from the stress gradient across tet edges:

$$ r_e = \max_{e' \in \mathcal{N}(e)} \frac{\|\sigma_e - \sigma_{e'}\|_F}{\|\sigma_e\|_F + \epsilon} $$

where $\mathcal{N}(e)$ is the set of face-neighbors of tet $e$ and $\|\cdot\|_F$ is Frobenius norm. A tet is marked for refinement if $r_e > r_\text{threshold}$ (default 0.3). The $\epsilon$ floor prevents division-by-zero in low-stress regions and picks up a natural "ignore tets in the resting region" behavior.

Three design choices implicit in this trigger.

**Stress, not strain.** For hyperelastic materials, stress and strain carry the same information in the low-strain limit but diverge at finite strain — stress localizes at contact interfaces where strain may not. `sim-soft`'s refinement target is specifically the stress-localization zone (contact bands, sharp corners, material interface bands), not the strain-distribution zone, so stress is the right signal.

**Gradient across neighbors, not absolute magnitude.** Absolute stress magnitude would refine every tet in a uniformly-loaded region, which is not the failure mode we are fixing. We want to refine where the *stress field is changing fast* — the places where the [rim-deformation failure from Part 1 Ch 02](../10-physical/02-what-goes-wrong/04-rim.md) lives. Gradient-across-neighbors picks up exactly that.

**Per-converged-solve, not per-Newton-iterate.** Refinement runs after Newton converges on the current mesh, not mid-iteration. Refining mid-iteration would invalidate the Hessian factorization ([Part 5 Ch 00 Claim 3](../50-time-integration/00-backward-euler.md)), destroy the warm-start, and turn the adjoint into a non-deterministic function of which iterate triggered the split. The loss of solver-state stability outweighs the accuracy gain. Refinement is a between-solves operation.

## The operation: red-green subdivision, not remeshing

Given a tet marked for refinement, `sim-soft` applies red-green subdivision (Bey 1995): split the tet into 8 sub-tets by inserting new vertices at edge midpoints (*red* refinement), then propagate green closures across the neighbors to maintain conformity. This is the standard FEM adaptive-refinement operation and has three properties that matter for `sim-soft`:

1. **Topology-preserving within the mesh.** The SDF boundary does not move; refinement subdivides interior tets, and the surface triangulation on boundary tets is updated consistently. No SDF → tet re-generation is required; [Ch 01's generator](01-tet-strategies.md) is not re-invoked.
2. **State-transferable.** Positions of the new midpoint vertices are interpolated linearly from their parent edge; velocities and stresses are interpolated from the parent tet. The [Ch 04 warm-start machinery](04-live-remesh.md) applies, specialized to the refinement-induced change.
3. **Bounded quality loss.** Red subdivision produces 8 children — 4 corner tets similar to the parent at half scale, and 4 interior tets from an octahedral split whose quality depends on which diagonal is chosen. Bey's result bounds the worst-case aspect-ratio degradation across any choice; green closures are bounded by the same result. `sim-soft` picks the diagonal that minimizes the resulting aspect-ratio spread and enforces a quality floor, aborting refinement on any tet whose children would fall below it — a rare corner case for well-conditioned initial meshes.

Unrefinement (coalescing four children back into a parent) is also supported, triggered when the reverse criterion $r_e < r_\text{threshold}/4$ holds on all children of a parent tet. The hysteresis factor of 4 prevents flapping between refined and coarse states across the optimizer's design sweeps.

## Integration with the optimization loop

Adaptive refinement interacts non-trivially with [Part 10 Ch 00 — forward](../100-optimization/00-forward.md)'s optimization loop. An optimizer that sweeps design parameters may trigger refinement at different design points, producing meshes of different tet counts and different degree-of-freedom dimensions. The gradient API from [Part 6 Ch 02](../60-differentiability/02-implicit-function.md) projects gradients through the refinement operation by treating the subdivision as a linear interpolation operator; the projection is differentiable, so the optimizer sees a smooth reward surface modulo the piecewise changes at refinement boundaries. Those boundaries are flagged with `GradientEstimate::Noisy { variance }` the same way [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md) handles mesh-topology changes; the noise scales with the stress-gradient tolerance $r_\text{threshold}$ — tighter tolerances produce more-frequent but smaller-jump refinement events. Replacing the FD wrapper with a proper stochastic-adjoint treatment is scheduled alongside the other stochastic-adjoint work in [Phase H](../110-crate/03-build-order.md#the-committed-order) (per [Part 6 Ch 03](../60-differentiability/03-time-adjoint.md)); the deferral is explicit and tied to a phase, not indefinite.

## Alternatives rejected

**Uniform refinement.** Subdividing every tet uniformly is simpler but produces a mesh 8× larger than adaptive refinement for the same worst-case accuracy. On a 30k-tet starting mesh, uniform refinement produces 240k tets and sends Newton-per-step cost up 8× (sparse-factorization-dominated, so near-linear in DOF count at this scale); adaptive refinement on the same problem typically produces ≈60k tets concentrated in contact bands. Rejected.

**p-refinement (increase element order).** Tet10 is [Phase H](../110-crate/03-build-order.md#the-committed-order) and addresses the shape-function-poverty half of the rim-deformation failure; adaptive-h-refinement addresses the mesh-resolution half. They are complementary, not alternatives — the canonical problem benefits from both (Tet10 in the contact band, h-refinement in the stress-gradient zones). Neither is rejected; the chapter is specifically about h-refinement.

**Contact-proximity-based refinement.** Refining tets near active IPC contact pairs (regardless of stress magnitude) is simpler than stress-gradient-based and popular in production IPC implementations. Rejected as the default because it refines too aggressively on glancing contact where stress is low, and not aggressively enough at internal stress concentrations (e.g., the shoulder where a rigid probe's diameter steps up). Stress-gradient picks up both cases; contact-proximity picks up only the first.

## When to enable

Adaptive refinement is gated behind a config flag, off by default, for a reason: the canonical problem at Phase D mesh resolution is already fine enough that the refinement rarely triggers, and the added code path is not worth the complexity when simpler paths work. The flag flips on in Phase H when the fidelity-upgrade wave lands (Tet10, viscoelasticity, HGO anisotropy); adaptive refinement in that phase handles the cases where the Tet10 + fine baseline is still insufficient. Experience-mode runs it off; design-mode in Phase H runs it on.
