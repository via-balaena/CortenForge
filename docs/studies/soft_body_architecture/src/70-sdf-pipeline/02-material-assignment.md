# Material field assignment

Once [Ch 01](01-tet-strategies.md) has produced a tet mesh from the geometric SDF, each tet needs its material parameters — stiffness, density, fiber direction, Prony-series relaxation times, thermal conductivity — assigned from the `MaterialField` that accompanied the `SdfField` across the [Ch 00 boundary](00-sdf-primitive.md). This chapter names how.

| Section | What it covers |
|---|---|
| [SDF sampling per tet](02-material-assignment/00-sampling.md) | Evaluation point policy: centroid for linear tets (Tet4), Gauss-point quadrature for higher-order tets (Tet10, [Phase H](../110-crate/03-build-order.md#the-committed-order)). One sample per tet, cached at mesh-build time, re-sampled on re-mesh |
| [Multi-field composition](02-material-assignment/01-composition.md) | Composition rules when the `MaterialField` is an SDF-distance-weighted blend (e.g., stiff skin over soft core): tet sits in a graded band, sample resolves to the correct per-tet weighted mean; interface bands can trigger refinement ([Ch 03](03-adaptive-refine.md)) |

Two claims.

**Per-tet sampling, not per-vertex.** `sim-soft`'s [`Material` trait](../20-materials/00-trait-hierarchy/00-trait-surface.md) takes per-tet parameters; the material is piecewise-constant across tets, matching the Tet4 linear-shape-function assumption. Per-vertex material painting (common in games rigging pipelines) would mismatch the constitutive-law-per-element model and require averaging within each tet anyway. Centroid sampling is the simplest policy that respects the discretization. Higher-order tets (Tet10, Phase H) require per-Gauss-point sampling because the material variation is integrated through the element; the [sampling sub-chapter](02-material-assignment/00-sampling.md) covers the quadrature rule.

**The interface band is where multi-material composition bites.** A stiff skin over a soft core passes through a graded transition zone where the SDF-weighted blend assigns a continuous stiffness value to each tet. If a tet straddles the transition band and samples the centroid, it averages-out the gradient and misses it; this is a known source of material-band-smearing errors in FEM-on-SDF workflows. The `sim-soft` response is to (a) flag tets whose SDF distance to the nearest material interface is below their own edge length as "interface tets" at mesh-build time, and (b) allow [adaptive refinement](03-adaptive-refine.md) to subdivide them if stress resolution demands it. The flagging is cheap (one SDF query per tet); the refinement is only triggered when the stress gradient across the band is high enough to matter.
