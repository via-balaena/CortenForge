# Live re-meshing under design edits

The [Ch 00 thesis](00-sdf-primitive.md) commits `sim-soft` to an SDF-valued design surface, and [the Part 1 Ch 03 thesis](../10-physical/03-thesis.md) commits to `cf-design`-driven interactive authoring where the designer edits geometry and sees the simulation respond. This chapter names the machinery that closes the loop: **SDF edit → change detection → re-mesh decision → warm-started re-solve**, run at interactive rates inside a single design-mode session. Phase G's build-order deliverable is this chapter's machinery working end-to-end on the canonical problem.

| Section | What it covers |
|---|---|
| [Change detection](04-live-remesh/00-change-detection.md) | Content-hash the composed `SdfField` + `MaterialField`; classify edits as (a) parameter-only (warm-start, no re-mesh), (b) topology-preserving-but-material-changing (warm-start with material re-sample), (c) topology-changing (full re-mesh) |
| [Warm-start from previous mesh](04-live-remesh/01-warm-start.md) | For parameter-only and material-changing edits: reuse the previous mesh, re-sample materials, use the previous converged state $x^\ast_\text{old}$ as the Newton initial guess. Target: ≤50 ms per edit for design-mode |
| [State transfer across re-meshes](04-live-remesh/02-state-transfer.md) | For topology-changing edits: interpolate position, velocity, and stress from the old mesh onto the new mesh via barycentric sampling; re-assemble and re-factor the Hessian. Target: ≤500 ms per edit for design-mode |

Four claims.

## 1. The three edit classes have three different cost regimes

Not every design edit is created equal. A designer adjusting a probe-radius slider triggers a `SdfField` change, but the tet mesh and the material assignment do not have to change — the geometric SDF is sampled differently, but the tet topology is preserved by the iso-surface reconstruction when the boundary moves by less than one tet edge. `sim-soft`'s change detector distinguishes:

- **Parameter-only edit** (probe radius nudged by <1 edge length, blend radius $k$ adjusted, material stiffness value changed on an existing field). No re-mesh. The previous mesh's tet topology is reused; per-tet material values are re-sampled; the previous converged position is the new Newton initial guess. End-to-end: **≤50 ms** on canonical-scale (~30k tets). This is the hot path; 90% of interactive edits land here.

- **Material-changing / topology-preserving edit** (designer adds a new material field, changes a field's composition tree, swaps a material to one with different Prony terms). No re-mesh. Per-tet material values are re-sampled from the new `MaterialField`; the Hessian is re-assembled (material changed, so stiffness changed) and re-factored; Newton uses the previous position as initial guess. End-to-end: **≤200 ms**.

- **Topology-changing edit** (designer changes the composition tree in a way that adds or removes primitives, moves a primitive enough that the iso-surface topology changes, or crosses a bifurcation like a cavity opening into a through-hole). Full re-mesh via [Ch 01](01-tet-strategies.md); state transfer via barycentric interpolation ([state-transfer sub-chapter](04-live-remesh/02-state-transfer.md)); full Newton cold-start. End-to-end: **≤500 ms** for design-mode resolution, longer for larger meshes.

Interactive design productivity lives or dies on the ≤50 ms target for parameter-only edits. The whole change-detection layer exists specifically to keep the common case cheap.

## 2. Change detection is content-hash-based, not structural

The change detector hashes the composed `SdfField` and `MaterialField` at a fixed set of probe points (sparse, ~100 samples over the bounding box) and compares the hash to the previous frame's. If the hash matches, no re-solve is needed. If the hash differs, a secondary classifier runs: re-hash the topology (iso-surface extraction at coarse resolution, count connected components + genus + cavity count) and compare to the previous topology. If topology matches, the edit is parameter-only or material-changing; a third classifier distinguishes those by checking which field's hash changed. If topology differs, the edit is topology-changing.

This is cheap — the primary hash is ≈100 SDF evaluations (μs-scale), and the topology hash is ≈10k evaluations at a 30³ grid plus connected-components labeling (ms-scale). Only the topology-changing case pays the full re-mesh cost, and the classifier runs only once per detected edit.

Structural change detection (walking the composition tree and diffing node-by-node) would be more precise but requires `sim-soft` to understand `cf-design`'s composition-tree internals. The content-hash approach treats `cf-design` as opaque — it produces an `SdfField`+`MaterialField`, `sim-soft` hashes it, the boundary stays at the `Sdf` trait. That matches [Ch 00's commitment](00-sdf-primitive.md) to the boundary being trait-object-based.

## 3. Warm-starting is where the interactive-rate story lives

A parameter-only edit reuses the previous mesh and previous converged position $x^\ast_\text{old}$. The new equilibrium $x^\ast_\text{new}$ is close to $x^\ast_\text{old}$ when the edit is small, and Newton converges in a handful of iterations from the warm-started initial guess rather than from a cold start — the gap is large enough to be what makes the interactive-rate budget feasible. The hot-path pattern:

```rust
use faer::sparse::linalg::solvers::Llt;
use sim_ml_chassis::Tape;

pub fn apply_design_edit(
    scene: &mut SoftScene,
    new_sdf: SdfField,
    new_material: MaterialField,
    tape: &mut Tape,
) -> EditResult {
    let classification = change_detector.classify(&scene.sdf, &new_sdf,
                                                   &scene.material, &new_material);
    match classification {
        EditClass::ParameterOnly => {
            scene.sdf = new_sdf;
            scene.material = new_material;
            scene.resample_materials_per_tet();     // no mesh change
            // previous x* is still a valid initial guess; Hessian may need partial re-assembly
            let step = newton_solve_from(tape, &scene.x_star, /* warm */ true);
            EditResult::Warm { iters: step.newton_iters, wall_ms: step.elapsed_ms }
        }
        EditClass::MaterialChanging => {
            scene.material = new_material;
            scene.resample_materials_per_tet();
            scene.hessian_cache.invalidate();       // material changed, re-assemble + re-factor
            let step = newton_solve_from(tape, &scene.x_star, /* warm */ true);
            EditResult::Warm { iters: step.newton_iters, wall_ms: step.elapsed_ms }
        }
        EditClass::TopologyChanging => {
            let old_mesh = std::mem::take(&mut scene.mesh);
            let old_state = std::mem::take(&mut scene.x_star);
            scene.sdf = new_sdf;
            scene.material = new_material;
            scene.mesh = tet_generator.mesh_sdf(&scene.sdf, scene.resolution);   // Ch 01
            scene.resample_materials_per_tet();
            scene.x_star = state_transfer_barycentric(&old_mesh, &old_state, &scene.mesh);
            let step = newton_solve_from(tape, &scene.x_star, /* warm */ false);
            EditResult::Cold { iters: step.newton_iters, wall_ms: step.elapsed_ms }
        }
    }
}
```

The code shape is deliberate: three cases, three cost regimes, one trait-boundary-respecting API. The `hessian_cache` carries the forward-Newton factorization from the previous edit across to the next, consistent with [Part 5 Ch 00's factor-on-tape pattern](../50-time-integration/00-backward-euler.md). For the parameter-only path — the common case — the Hessian is often unchanged (the edit is a geometric-SDF boundary shift too small to alter tet Jacobians), the cached Cholesky factor is re-used, and Newton converges on back-substitutions alone.

## 4. State transfer is where differentiability breaks, and the FD wrapper lives

Topology-changing edits interpolate state from the old mesh to the new mesh via barycentric sampling: each new-mesh vertex finds its enclosing old-mesh tet, and its position and velocity are interpolated from the old tet's four vertices via barycentric weights. Per-tet quantities (stress, per-element material state) are inherited from the enclosing old tet directly — no barycentric blend, just assignment. Both paths are linear-operator-smooth in their inputs — but the *choice of which old-mesh tet encloses each new-mesh vertex* is not smooth, because the enclosing-tet query is piecewise-constant across new-mesh vertex positions.

Concretely: consider a design-parameter sweep that gradually moves a primitive until its boundary crosses through a tet. At the crossing, the enclosing-tet assignment for any new-mesh vertex near the boundary flips discontinuously. State transfer at that design parameter is therefore not differentiable by a smooth operator.

[Part 6 Ch 05's FD wrapper](../60-differentiability/05-diff-meshing.md) addresses this. Gradients w.r.t. design parameters that trigger topology changes are computed by finite difference across the change, with the resulting noise reported via `GradientEstimate::Noisy { variance }`. Replacing the FD wrapper with a proper stochastic-adjoint treatment is scheduled alongside the other stochastic-adjoint work in [Phase H](../110-crate/03-build-order.md#the-committed-order) (per [Part 6 Ch 03](../60-differentiability/03-time-adjoint.md)); the deferral is explicit and tied to a phase, not indefinite. Parameter-only and material-changing edits do not cross this boundary and retain exact IFT gradients through [Part 6 Ch 02's machinery](../60-differentiability/02-implicit-function.md). In practice:

- **Design-mode gradient-based optimizers** (BayesOpt, gradient descent, [Part 10 Ch 00 — forward](../100-optimization/00-forward.md)) get exact gradients for ≈95% of steps (parameter-only ≈90% + material-changing ≈5%) and noisy gradients for the remaining ≈5% that cross topology changes. The noise is flagged, and the optimizer handles it by falling back to a FD-noise-tolerant acquisition (e.g., UCB with inflated variance on noisy samples).
- **Active-learning loops** (information-theoretic acquisitions) weight noisy samples lower, so the `GradientEstimate::Noisy` flag propagates naturally into the acquisition function.

The overall design consequence: `sim-soft` does not deliver end-to-end smooth differentiability across arbitrary design edits — that is the [Part 6 Ch 05 open problem](../60-differentiability/05-diff-meshing.md), unsolved at the platform level. What it does deliver is exact gradients on the common case (parameter edits within a fixed topology, 90%+ of sweeps) and flagged-noisy gradients on the rare case (topology crossings), with the division between the two determined by the change detector rather than by the optimizer guessing.

## What this commits downstream

- [Part 6 Ch 05 (diff-meshing open problem)](../60-differentiability/05-diff-meshing.md)'s FD wrapper is applied specifically at the state-transfer boundary inside `TopologyChanging` edits. The `GradientEstimate::Noisy` flag is raised exactly when the change detector classifies an edit as topology-changing.
- [Part 10 Ch 00 — forward](../100-optimization/00-forward.md) consumes the `EditResult` + `GradientEstimate` pair to choose acquisition functions that tolerate noise on topology-crossing samples.
- [Part 11 Ch 02 sub-chapter 04 (cf-design coupling)](../110-crate/02-coupling/04-cf-design.md) closes the authoring-side API: `cf-design` emits `SdfField` + `MaterialField` pairs and consumes `EditResult`s; the protocol is specified there.
- [Phase G of build-order](../110-crate/03-build-order.md#the-committed-order) names the deliverable: designer edits SDF in `cf-design`, mesh re-derives, solve warm-starts, reward updates, visible within ≤500 ms for topology-changing edits, ≤50 ms for parameter-only.

## What this does NOT commit

- **The `cf-design` UI.** Whether `cf-design` exposes sliders, node graphs, Python bindings, or a WYSIWYG tool is a `cf-design` concern. `sim-soft` sees only the `SdfField` + `MaterialField` pair at the edit boundary.
- **Persistence.** Saving and loading designs is a `cf-design` concern; `sim-soft` does not serialize the scene state for persistence, only for debugging and regression tests.
- **Multi-designer collaboration.** Real-time collaborative editing (multiple designers on one scene) is not on the Phase A–I roadmap and would require a different synchronization primitive than content-hashing. If it becomes a goal, the hash machinery from §2 is a reasonable starting point, but the rest of the stack would need rework.
