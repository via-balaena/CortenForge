# SDF sampling per tet

After [Ch 01's tet generator](../01-tet-strategies.md) produces a mesh, `sim-soft`'s `sdf_bridge/` walks the mesh once to assign per-tet material parameters by sampling the [`MaterialField`](../00-sdf-primitive.md) at each element's evaluation point. This leaf names the evaluation-point policy, the caching strategy, and the re-sampling trigger — complementary to [Part 2 Ch 09 §01's trait-surface-side sampling contract](../../20-materials/09-spatial-fields/01-sampling.md).

## The per-tet evaluation point

The [Ch 02 parent's Claim 1](../02-material-assignment.md) commits `sim-soft` to per-tet sampling, not per-vertex. The evaluation point depends on the element type:

- **Tet4 (constant-strain) — sample at the centroid.** For a tet with vertices $\mathbf{x}_1, \mathbf{x}_2, \mathbf{x}_3, \mathbf{x}_4$, the centroid $\mathbf{x}_c = \tfrac{1}{4}(\mathbf{x}_1 + \mathbf{x}_2 + \mathbf{x}_3 + \mathbf{x}_4)$ is the unique point that the linear shape functions weight equally. One call to [`Field::sample(x_ref)`](../../20-materials/09-spatial-fields/00-sdf-valued.md) per scalar parameter slot produces one per-element $\mathbf{p}_e$ vector.
- **Tet10 (higher-order, Phase H) — sample per Gauss point.** Tet10's four-point Gauss quadrature rule distributes quadrature points across the element interior; sampling the `MaterialField` at each quadrature point produces a per-Gauss-point material vector, which the assembly pass consumes for the four-point integration of the element residual and tangent. Per-Gauss-point sampling resolves within-element material gradients that centroid-sampled Tet10 would miss.

The Tet10 rule matches the [Gauss-quadrature weights $w_q$](../../appendices/03-notation.md) from [Part 3 Ch 00 Tet10](../../30-discretization/00-element-choice/01-tet10.md); the [Hughes 2000](../../appendices/00-references/02-adjoint.md#hughes-2000) standard-FEM reference grounds the Gauss-point-per-assembly-pass convention.

## When sampling happens

Material sampling is a mesh-build-time pass, not a per-Newton-iteration pass. Concretely:

1. **Initial mesh build** ([Ch 01](../01-tet-strategies.md) produces the tet mesh). `sdf_bridge/` walks the new mesh and samples the `MaterialField` at each element's evaluation point. The per-element material vectors are stored in the mesh's per-tet material cache.
2. **Warm-started re-mesh** ([Ch 04 §01 warm-start](../04-live-remesh/01-warm-start.md) handles parameter-only or material-changing edits). The tet topology is unchanged; the per-element sample points are the same; but the `MaterialField` may have changed (for material-changing edits). `sdf_bridge/` re-walks the cached mesh and re-samples the `MaterialField` at the same evaluation points, updating the per-tet material cache.
3. **Cold-started re-mesh** ([Ch 04 §02 state-transfer](../04-live-remesh/02-state-transfer.md) handles topology-changing edits). The tet topology has changed; the sample points have changed; the `MaterialField` may or may not have changed. `sdf_bridge/` walks the newly-meshed tet set and samples at each new evaluation point from scratch.

The [Newton hot path](../../50-time-integration/00-backward-euler.md) reads from the per-tet material cache; the sampling cost is amortized across all Newton iterations of all timesteps between re-samples. For the parameter-only hot path (~90% of design-mode edits per [Ch 04 §00](../04-live-remesh/00-change-detection.md)), the cache is not invalidated and the sampling pass does not run at all.

## The cache

Per-tet material vectors live alongside the tet connectivity in a `TetMesh::material_cache` field indexed by tet id. The cache's shape mirrors the `MaterialField`'s slot set (stiffness, density, fiber direction, Prony terms, thermal conductivity), with the concrete per-tet or per-Gauss-point storage depending on element type. Specific layout (struct-of-arrays vs. array-of-structs, SIMD-alignment, cacheline packing) is a Phase-B implementation decision tied to the assembly kernel's access pattern.

For Tet4, the cache holds one entry per tet per slot. For Tet10, it holds four entries per tet per slot (one per Gauss point). The distinction is encoded in the tet-element-type tag that every tet carries from mesh-build time; the assembly pass switches on the tag without branching on per-tet metadata at per-iteration granularity.

## Re-sampling is not re-meshing

The distinction is load-bearing: re-sampling walks an existing mesh and updates its material cache; re-meshing replaces the mesh entirely. A material-only design change (the designer moves a material-field boundary without moving any geometric boundary) triggers re-sampling but not re-meshing, and the Hessian re-assembly + re-factorization cost is the dominant latency on that path (per [Ch 04 §01 warm-start](../04-live-remesh/01-warm-start.md)'s ≤200 ms budget for material-changing edits, longer than the parameter-only ≤50 ms because the per-element stiffness blocks depend on the material parameters).

The sample-cache update itself is cheap — one `Field::sample` call per slot per tet per Gauss point, at most a few thousand evaluations total per mesh. The dominant post-resampling cost is the Hessian re-assembly downstream, not the sample pass itself.

## What this sub-leaf commits the book to

- **Tet4 sampling: centroid.** One `Field::sample` call per slot per tet. Matches the one-Gauss-point integration rule.
- **Tet10 sampling: per Gauss point.** Four calls per slot per tet for the four-Gauss-point rule; resolves within-element material gradients that Tet10's quadratic shape functions exercise.
- **Sampling is a mesh-build-time pass.** Stored in a per-tet material cache alongside the connectivity; read from the Newton hot path, written only on (re-)mesh or material-changing edits.
- **Re-sampling ≠ re-meshing.** Material-only edits re-sample the existing mesh; topology-changing edits re-mesh and re-sample from scratch ([Ch 04](../04-live-remesh.md) distinguishes).
- **Hughes 2000 Gauss-quadrature rule grounds the Tet10 per-quadrature-point sampling convention.** No new Gauss rule is derived here; `sim-soft` consumes the standard.
