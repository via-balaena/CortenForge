# Per-element sampling

The [Ch 09 parent's table](../09-spatial-fields.md) names per-element sampling as the bridge from continuous fields to discrete element properties. This leaf writes the sampling-point choice for [Tet4](../../30-discretization/00-element-choice/00-tet4.md) and [Tet10](../../30-discretization/00-element-choice/01-tet10.md), the trade-off between centroid and per-Gauss-point sampling, and the alternatives `sim-soft` does not ship.

## The default rule

For Tet4 (one Gauss point at the element centroid), the default is to sample [`MaterialField`](00-sdf-valued.md) at the centroid: one call to `MaterialField::sample(x_ref)` per element, one resulting per-element scalar `Material` instance. The Tet4 element is constant-strain by construction and gives one Gauss-point evaluation per assembly pass; matching the sample rate to the integration rate is the natural choice and incurs no over- or under-resolution.

For Tet10 (four Gauss points distributed across the element interior), `sim-soft` ships per-Gauss-point sampling as the default: four calls to `MaterialField::sample` per element, four per-Gauss-point scalar parameter sets that the assembly pass consumes for the four-Gauss-point quadrature rule. Per-Gauss-point sampling resolves within-element material gradients — a stiffness ramp across an element produces different evaluations at the four Gauss points, and the assembled per-element residual reflects the gradient.

The user can opt down to centroid-sampled Tet10 (one sample per element, shared across the four Gauss points) for performance-sensitive scenes where the material field is known to be coarse-scale relative to the mesh resolution. This is a per-mesh-region option set at construction; it is not the default.

## Why centroid is the natural sample point

For a tetrahedron with vertices $\mathbf{x}_1, \mathbf{x}_2, \mathbf{x}_3, \mathbf{x}_4$, the centroid $\mathbf{x}_c = \tfrac{1}{4}(\mathbf{x}_1 + \mathbf{x}_2 + \mathbf{x}_3 + \mathbf{x}_4)$ is the unique point that the linear shape functions weight equally. For a smoothly varying material field, the centroid sample is the leading-order approximation of the per-element-averaged parameter value: the higher-order corrections involve gradients of the field across the element, which the linear-element interpolation drops anyway.

For a discontinuous field (sharp material interface across the element), the centroid sample is meaningless — the centroid lies on one side or the other and gives the parameter from that side, ignoring the interface entirely. Per-element sampling cannot resolve sub-element discontinuities; the [Part 3 multi-material interfaces leaf](../../30-discretization/03-interfaces.md) handles this by requiring element edges to lie on material-interface SDFs, ensuring no element straddles a discontinuity.

## Alternatives `sim-soft` does not ship

Two alternatives to per-element sampling appear in the literature:

- **Vertex-sampled with linear interpolation.** Sample at the four element vertices and interpolate per Gauss point via the linear shape functions. The interpolation respects vertex-to-vertex gradients and gives a smooth-per-element field, but the assembly cost rises (four samples per Tet4 instead of one) and the interpolation introduces artifacts at vertex-shared materially-discontinuous interfaces. Not the default.
- **Subgrid or higher-order sampling.** Sample at points denser than the integration rule's Gauss points and average. Useful for capturing very-fine field structure when re-meshing is not an option, but introduces a separate tuning parameter (sub-grid density) and breaks the one-step-equals-one-pass assembly pattern. Not the default; reserved for post-Phase-I extensions if specific scenes need it.

The per-element-centroid (Tet4) and per-Gauss-point (Tet10) defaults match the standard FEM practice for material assignment from spatial fields and align with the [Phase D differentiability validation](../../110-crate/03-build-order.md) the [gradcheck tests](../../110-crate/04-testing/03-gradcheck.md) target.

## What this sub-leaf commits the book to

- **Tet4 default: sample at the element centroid (one call per element).** Matches the one-Gauss-point integration rule; no over- or under-resolution.
- **Tet10 default: sample per Gauss point (four calls per element).** Resolves within-element material gradients; aligns with the four-Gauss-point quadrature rule.
- **Centroid-sampled Tet10 is an opt-in for coarse-field scenes.** Performance-sensitive scenes where the material field is known to be coarser than the mesh resolution can drop to one sample per element.
- **Discontinuous fields require mesh-aligned interfaces.** Per-element sampling cannot resolve sub-element discontinuities; the meshing step ([Part 3 Ch 03](../../30-discretization/03-interfaces.md)) handles material-interface SDFs by requiring element edges to lie on them.
- **Vertex-sampled and subgrid alternatives are not the default.** Either is possible; neither is on the Phase A–I roadmap.
