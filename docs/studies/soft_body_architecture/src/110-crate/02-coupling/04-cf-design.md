# With cf-design

`cf-design` is the authoring layer. The designer edits primitive parameters in `cf-design`'s composition-tree UI — probe radius, cavity length, wall thickness, smooth-blend radii, per-region stiffness, fiber directions, diffusion profiles — and `cf-design` emits a resolved `SdfField` + `MaterialField` pair that `sim-soft` consumes. This coupling is **in-only**. No physics state flows back into `cf-design`; no rendered frame, no simulator internal, no optimizer gradient reaches the authoring crate through this boundary. `cf-design` sends geometry + material, `sim-soft` sends back an `EditResult` classifying the edit and reporting its wall-time, and the loop closes. The physical-print loop and everything downstream of it stay on the `sim-opt` side of the [§03 ml-chassis boundary](03-ml-chassis.md), not here.

## What crosses the boundary

| Field | Direction | Representation | Required? |
|---|---|---|---|
| `SdfField` — composed geometric SDF + bounding box + resolution hint | `cf-design` → `sim-soft` | `Box<dyn Sdf>` + `Aabb3` + `f64` per [Part 7 Ch 00 §04](../../70-sdf-pipeline/00-sdf-primitive.md) | yes |
| `MaterialField` — stiffness / density / fiber / Prony / thermal / layer-id / diffusion-profile fields | `cf-design` → `sim-soft` | struct of `Box<dyn Field<Output = T>>` entries per field; layer-id and diffusion-profile are renderer-facing (optional) | yes |
| `EditResult { class, wall_time_ms }` | `sim-soft` → `cf-design` | `EditClass::{ParameterOnly, MaterialChanging, TopologyChanging}` + wall-time-ms | yes |

The `SdfField` + `MaterialField` pair is re-sent on every design edit. [Part 7 Ch 04](../../70-sdf-pipeline/04-live-remesh.md) commits three edit-class cost regimes — ≤50 ms parameter-only, ≤200 ms material-changing, ≤500 ms topology-changing — which is the interactive-rate budget this coupling lives against. The [`sdf_bridge/` module](../00-module-layout/08-sdf-bridge.md) on the `sim-soft` side owns change detection, tetrahedralization, material sampling, warm-start, and FD-wrapper triggering; `cf-design` does not see any of that machinery, only the `EditResult` summary.

## The coupling is in-only

No `sim-soft` state, no physics snapshot, no sim-opt-side GP posterior, no rendered mesh, no gradient flows back across this boundary. `cf-design` authors; `sim-soft` consumes. Whatever `cf-design` needs to show the designer is driven by [`sim-bevy`'s visual layer](02-bevy.md) out of `sim-soft`'s `MeshSnapshot`, not out of a back-channel. If a Bevy-based preview renders inside `cf-design`'s own UI, that preview's state lives on the `sim-bevy` side of the pipeline, not on this boundary.

This is the narrowest asymmetry `sim-soft` has with any sibling crate: the one input (SDF + material) and the one return (`EditResult`) are the entire surface. It is narrow on purpose. [Part 7 Ch 00 §04](../../70-sdf-pipeline/00-sdf-primitive.md) commits the handshake to be in-memory rather than file-format-mediated so interactive-rate design-mode budgets are achievable; it does not commit anything about state flowing back, because nothing does.

## Differentiability flows through authoring parameters, not through the mesh

The designer's authoring state is a composition tree of primitives and operators, differentiable by construction per [Part 7 Ch 00 §01 operations](../../70-sdf-pipeline/00-sdf-primitive.md). When an optimizer adjusts $\theta$ (SDF parameters, material-field coefficients, reward-composition weights), the SDF and material fields change smoothly, `sdf_bridge/` classifies the edit, and:

- **Parameter-only edits** (≈90% of design-space steps per [Part 10 Ch 00 split](../../100-optimization/00-forward.md)) — warm-start the previous converged state, skip tetrahedralization, re-sample per-tet material parameters, reuse the cached Cholesky factor where possible. IFT gradients through [`autograd/`](../00-module-layout/07-autograd.md) are exact.
- **Material-changing edits** (≈5%) — warm-start position, re-assemble and re-factor the Hessian (material changed, stiffness changed), IFT gradients remain exact.
- **Topology-changing edits** (≈5%) — full re-mesh via [Ch 01](../../70-sdf-pipeline/01-tet-strategies.md), barycentric state transfer via [Ch 04 §02](../../70-sdf-pipeline/04-live-remesh/02-state-transfer.md), [FD-wrapper](../00-module-layout/08-sdf-bridge.md) returns `GradientEstimate::Noisy { variance }`.

The non-differentiable step — SDF → tet mesh — is confined to [`sdf_bridge/`](../00-module-layout/08-sdf-bridge.md) by design. This is what [Part 7 Ch 00 §01 claim 3](../../70-sdf-pipeline/00-sdf-primitive.md) commits: the SDF abstraction localizes the non-differentiable step at one well-defined boundary and lets the FD wrapper have one clean thing to wrap. Had the design surface been mesh-valued, differentiability would bleed into every vertex-move, and the FD wrapper would have nothing clean to wrap.

The 95/5 exact/noisy split is a property of `sim-soft`'s canonical design space, not a universal claim. Design spaces dominated by big geometric moves (adding or removing a primitive per step) shift toward 70/30, and [Part 10 Ch 02](../../100-optimization/02-bayesopt.md)'s noise-tolerant acquisition machinery (UCB with variance-inflated noisy samples, EI with posterior truncation, info-criteria down-weighting noisy samples by $1 / (1 + \text{var}/\sigma_\text{signal}^2)$) becomes load-bearing in those regimes rather than an edge case.

## `MaterialField` carries both solver-consumed and renderer-only channels

The [`MaterialField` struct from Part 7 Ch 00 §04](../../70-sdf-pipeline/00-sdf-primitive.md) has seven channels; four are solver-consumed, one is used by both solver and renderer, and two are renderer-only:

```rust
pub struct MaterialField {
    pub stiffness:           Box<dyn Field<Output = f64>>,                      // solver
    pub density:             Box<dyn Field<Output = f64>>,                      // solver
    pub fiber_dir:           Option<Box<dyn Field<Output = Vec3>>>,             // solver + renderer
    pub prony_terms:         Vec<Box<dyn Field<Output = f64>>>,                 // solver
    pub thermal_conductivity: Option<Box<dyn Field<Output = f64>>>,             // solver
    pub layer_id:            Option<Box<dyn Field<Output = u8>>>,               // renderer only
    pub diffusion_profile:   Option<Box<dyn Field<Output = DiffusionProfile>>>, // renderer only
}
```

`diffusion_profile` is the γ-locked [`DiffusionProfile`](../../90-visual/00-sss.md) type — sum-of-exponentials by default (Christensen-Burley 2015 lineage per [Part 9 Ch 00 §01](../../90-visual/00-sss/01-bssrdf.md)), refit to sum-of-Gaussians at scene ingest when `layer_id != None` so multi-layer compounds via variance addition. [`material/` claim 3](../00-module-layout/00-material.md) commits that `MaterialField` owns the storage and composition of both channels; [`sim-bevy`](02-bevy.md) consumes them for SSS rendering via the [`Observable`-surfaced snapshot](02-bevy.md). Single-material or physics-only runs leave both `None` and pay no cost.

From `cf-design`'s authoring perspective, `diffusion_profile` is a field the designer either paints directly (per-material curve authoring) or inherits from the [Appendix 02 material database](../../appendices/02-material-db.md)'s reference profiles. The authoring UI on the `cf-design` side is out of scope for this sub-leaf; the cross-crate type is the `DiffusionProfile` field on `MaterialField`, and the appendix row-set expansion below makes the material-database cross-reference concrete.

## Appendix 02 material-DB — `DiffusionProfile` row extension

[Appendix 02's material-database tables](../../appendices/02-material-db.md) carry per-material numerical constants for every shipped `impl Material` — Ecoflex 00-10/00-20/00-30/00-50, Dragon Skin 10A/20A/30A, carbon-black composites at 5/10/15/20 wt%. Each family's table already has a "Diffusion profile reference" row marked "Pass 3 measured"; the γ-locked `DiffusionProfile` schema is now available, so the row-set is extended inline here to name the data the appendix will carry once the measurements land. The schema binds now even though the numerical values stay Pass 3 — having the column structure pinned means a designer or downstream tool can wire against the appendix today and fill in per-batch calibration values later without a schema migration.

The extension lands in [Appendix 02](../../appendices/02-material-db.md) as an adjacent authoring-side check: the nominal-column row "Diffusion profile reference: Pass 3 measured" becomes a two-level entry — the γ-locked `DiffusionProfile` schema reference (default sum-of-exponentials, Christensen-Burley 2015) plus the per-material measured-curve slot (Pass 3). No value changes in Pass 1; only the schema is pinned. The [calibration loop in Part 10 Ch 05](../../100-optimization/05-sim-to-real.md)'s `diffusion_gp` consumes per-batch measured profiles as residual training data, and the appendix is the anchor reference those calibrations start from.

## No physical-print-loop machinery crosses this boundary

Per [Part 12 Ch 07 priority-2](../../120-roadmap/07-open-questions.md), the full design-print-rate loop — `MeasurementReport` ingestion, residual-GP online updates, batch-rating workflow, cross-print drift detection — is post-Phase-I and `sim-opt`-scoped, not `cf-design`-scoped. `cf-design` does not ingest `MeasurementReport`; `cf-design` does not see `SimToRealCorrection` or `OnlineUpdateOutcome` or printer IDs. Those types cross the `sim-opt` boundary only per [§03 ml-chassis "Where `sim-opt` sits"](03-ml-chassis.md) and [Part 10 Ch 05 §01](../../100-optimization/05-sim-to-real/01-online.md).

What `cf-design` could eventually see — post-Phase-I — is designer-rating UI for the [preference GP from Part 10 Ch 03](../../100-optimization/03-preference.md), because rating pairs of physically printed samples requires a designer-in-the-loop interface. That machinery lands in a future `cf-design` extension alongside the Part 12 Ch 07 deferred work; it is not a Phase A–I deliverable and does not cross this boundary during Phases A–I. Honoring the deferral discipline explicitly here matters because the authoring-side UI is where a future designer-rating hook would live, and naming the boundary makes sure the seam between "what `cf-design` does in Phase A–I" and "what a post-Phase-I extension would add" is a named seam, not a silent one.

## Determinism-in-θ across the authoring loop

The [`ForwardMap` γ-locked contract](../../100-optimization/00-forward.md) holds across this coupling: repeated `evaluate` calls at the same $\theta$ produce the same `RewardBreakdown` modulo cache state. The `cf-design` loop preserves this because (a) `cf-design`'s composition-tree resolution is a pure function of $\theta$ and the tree's structural state, (b) the `SdfField` and `MaterialField` hashes [`sdf_bridge/`'s change classifier](../00-module-layout/08-sdf-bridge.md) computes are deterministic functions of the resolved field content, and (c) the mesh generator ([fTetWild, Ch 01](../../70-sdf-pipeline/01-tet-strategies.md)) is deterministic given fixed SDF + resolution hint. There is no RNG in the authoring-to-physics path; once `cf-design` commits a $\theta$, every downstream consequence reproduces bit-identically.

The topology-changing case is still deterministic — the same topology edit at the same $\theta$ lands on the same new mesh with the same state-transfer interpolation — but its gradient is `GradientEstimate::Noisy` because FD evaluation perturbs $\theta$ across the topology boundary; the FD noise is in the gradient estimate's variance, not in the `RewardBreakdown` itself. [Part 10 Ch 02's noise-tolerant acquisitions](../../100-optimization/02-bayesopt.md) consume the variance directly.

## What the `cf-design` coupling does not carry

- **No back-flow of physics state.** `cf-design` does not see `MeshSnapshot`, `ForwardMap` returns, `Observable` output, or any solver internal. Authoring-side preview lives on the `sim-bevy` pipeline separately.
- **No SDF-primitive-library implementation.** `cf-design` ships the primitive library and the operator algebra internally; `sim-soft` queries the composed SDF via `Sdf::eval` / `Sdf::grad` and is agnostic to the underlying representation (grid-sampled, procedural tree, neural SDF, sparse voxel hash) per [Part 7 Ch 00 "What this does NOT commit"](../../70-sdf-pipeline/00-sdf-primitive.md).
- **No manufacturing export.** STL / STEP / slicer-input pipelines live in `cf-design` but are outside `sim-soft`'s boundary; this sub-leaf does not specify them.
- **No designer-rating machinery.** Preference GP ratings are a post-Phase-I addition on the `cf-design` side, not a Phase A–I deliverable crossing this boundary.
- **No physical-print ingestion.** `MeasurementReport` and friends cross `sim-opt`'s boundary only; `cf-design` does not see them.

## What this sub-leaf commits the crate to

- **`cf-design` coupling is in-only — `SdfField` + `MaterialField` in, `EditResult` out.** No physics state, no simulator internal, no gradient flows back. The narrowest cross-crate surface `sim-soft` has.
- **The boundary is the trait pair `Sdf` + `Field<Output = T>` plus the `EditResult` return.** `cf-design` owns the authoring layer; `sim-soft::sdf_bridge/` owns the ingest layer. `sim-soft` treats the SDF representation as opaque.
- **`MaterialField` carries both solver-consumed and renderer-only channels.** `stiffness` / `density` / `fiber_dir` / `prony_terms` / `thermal_conductivity` feed the solver; `layer_id` and γ-locked `diffusion_profile` feed the renderer per [Part 9 Ch 00](../../90-visual/00-sss.md). `material/` owns storage and composition per [Ch 00 `material/` claim 3](../00-module-layout/00-material.md).
- **Appendix 02 material-database schema is extended inline** to pin the γ-locked `DiffusionProfile` schema reference per resolved decision 3 of the Phase δ plan. Per-material measured values remain Pass 3; schema binds now.
- **Differentiability flows through authoring parameters.** Exact IFT gradients for parameter-only and material-changing edits; `GradientEstimate::Noisy` for topology-changing edits via [FD wrapper in `sdf_bridge/`](../00-module-layout/08-sdf-bridge.md). 95/5 exact/noisy split holds on the canonical design space.
- **No physical-print-loop machinery crosses.** `MeasurementReport` / `SimToRealCorrection` / `OnlineUpdateOutcome` / preference-rating UI are post-Phase-I per [Part 12 Ch 07](../../120-roadmap/07-open-questions.md) and stay `sim-opt`-scoped.
- **Determinism-in-θ holds.** Authoring-to-physics path has no RNG; [`ForwardMap`'s](../../100-optimization/00-forward.md) determinism contract is preserved.
- **Phase G is when this coupling closes.** Phases A–F use hand-authored meshes or MJCF imports; [Phase G](../03-build-order.md#the-committed-order) is when `cf-design`-driven live authoring becomes the primary design-mode workflow.
