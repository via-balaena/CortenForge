# SDF-valued material parameters

The [Ch 09 parent's Claim 1](../09-spatial-fields.md) commits to material parameters as fields rather than constants — a Young's modulus of 500 kPa everywhere is the degenerate case, and a realistic soft part has parameters varying smoothly (molded gradients) or abruptly (bonded multi-material). This leaf names the public API for specifying those fields, the type-level generalization that makes a uniform material a special case of a graded one, and the bridge to [`cf-design`](../../110-crate/02-coupling/04-cf-design.md)'s authoring layer.

## The `Sdf<T>` generic

The [`SdfField` and `MaterialField` types committed in Part 7 Ch 00](../../70-sdf-pipeline/00-sdf-primitive.md) are the foundation: `SdfField` is a scalar-valued spatial field, and `MaterialField` aggregates per-material-parameter `SdfField` instances. Ch 09 generalizes the scalar case to typed-valued spatial fields via the `Sdf<T>` generic:

- `Sdf<f64>` — scalar parameters (Lamé $\mu, \lambda$; Mooney-Rivlin $C_{10}, C_{01}$; Ogden $\mu_i, \alpha_i$; HGO $k_1, k_2$; Prony $\tau_i, g_i$; thermal expansion $\alpha$).
- `Sdf<UnitVec3>` — unit-vector parameters (HGO fiber direction $a$, single- and two-fiber families).
- `Sdf<Tensor3>` — tensor parameters (full orthotropy moduli, anisotropic thermal expansion). A Pass-3-or-later extension, not on the Phase A–I roadmap.

Each `Sdf<T>` exposes a sample method: `fn sample(&self, x_ref: Vec3) -> T`. The implementation is type-specific — scalar fields evaluate the scalar SDF; unit-vector fields evaluate and normalize; tensor fields evaluate and project onto the appropriate symmetric or PSD subspace. Consumers of the trait surface call `sample(x_ref)` without knowing the underlying SDF representation.

`SdfField` from Part 7 Ch 00 is the established type alias for the scalar case `Sdf<f64>`; the typed generic `Sdf<T>` extends the same authoring and evaluation machinery to non-scalar parameter types.

## `MaterialField` aggregates per-parameter SDFs

The `MaterialField` struct from [Part 7 Ch 00](../../70-sdf-pipeline/00-sdf-primitive.md) holds one `Sdf<T>` per material parameter the wrapped `Material` trait surface exposes. A constant-everywhere material is the degenerate case — every parameter slot holds an `Sdf::Constant(...)`. A graded material puts a non-constant `Sdf` in one or more slots. The solver does not know or care: every per-element material evaluation goes through `sample(x_ref)`, and a constant-SDF `sample` returns the same value at every $x_\text{ref}$.

This is the type-level generalization the parent's Claim 1 names: the constant case and the graded case share one trait surface, with the discrimination in the SDF type rather than in the consuming impl. There is no runtime branch on "is this material uniform or graded?" anywhere in the [`element/`](../../110-crate/00-module-layout/01-element.md) assembly path.

## Authoring path

`MaterialField` instances are authored in [`cf-design`](../../110-crate/02-coupling/04-cf-design.md), which exposes the SDF-construction primitives (CSG, smooth blends, displacement) committed in [Part 7 Ch 00 sub-chapter 01](../../70-sdf-pipeline/00-sdf-primitive/01-operations.md). A designer building a multi-layer compliant cavity assembles SDFs for the geometry (which the [meshing pipeline](../../70-sdf-pipeline/01-tet-strategies.md) consumes) and SDFs for the material parameters (which `MaterialField` aggregates and Ch 09 sampling consumes per element).

The two SDF channels — geometry and material — are independent: the geometry SDF determines where the mesh lives, the material SDF determines what each per-element parameter set is. A material-only design change can move material boundaries without re-meshing if the geometry envelope is unchanged; a geometry change forces a re-mesh ([Part 7 Ch 04 live re-mesh](../../70-sdf-pipeline/04-live-remesh.md)) but the material SDFs sample cleanly into the new mesh's elements.

## Trait-surface contract

The `Material` trait stays scalar-parameterized at the trait level — `mu: f64`, not `mu: Sdf<f64>`. The SDF-to-scalar conversion happens in [`element/`](../../110-crate/00-module-layout/01-element.md) at element construction (and warm-restart from re-mesh): for each element, the assembly pass calls `MaterialField::sample` at the [per-element sample point](01-sampling.md), produces a per-element `Material` instance with scalar parameters, and stores it alongside the element's deformation gradient.

The Newton loop iterates over per-element scalar materials. The spatial-field machinery does not enter the per-iteration hot path; the per-element scalar materials are static within a step and the Newton iteration's nonlinearity is purely in $F$. This separation is what keeps the [factor-on-tape Cholesky path](../../50-time-integration/00-backward-euler.md) clean — the spatial-field evaluation cost is a one-time per-step overhead, not a per-iteration multiplier.

## What this sub-leaf commits the book to

- **`Sdf<T>` generalizes the scalar `SdfField` to typed-valued spatial fields.** `Sdf<f64>` for scalars, `Sdf<UnitVec3>` for unit vectors, `Sdf<Tensor3>` as a future extension. The `sample(x_ref)` method is the only consumer-side API.
- **`MaterialField` aggregates per-parameter `Sdf<T>` instances.** Constant materials are `Sdf::Constant(...)` in each slot; graded materials replace specific slots with non-constant SDFs. No runtime branch on uniformity.
- **The `Material` trait stays scalar-parameterized.** SDF-to-scalar conversion happens in [`element/`](../../110-crate/00-module-layout/01-element.md) at construction (or warm-restart); the Newton hot path sees per-element scalar materials.
- **Geometry and material SDFs are independent channels.** Material boundaries can move without re-meshing when the geometry envelope is unchanged; geometry changes force re-mesh but material SDFs sample cleanly into the new mesh.
- **Authoring goes through [`cf-design`](../../110-crate/02-coupling/04-cf-design.md).** Same SDF-construction primitives as the geometry channel; `MaterialField` is the dispatch-aggregator at the `sim-soft` boundary.
