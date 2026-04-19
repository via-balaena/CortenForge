# SDF-valued material parameters

The [Ch 09 parent's Claim 1](../09-spatial-fields.md) commits to material parameters as fields rather than constants — a Young's modulus of 500 kPa everywhere is the degenerate case, and a realistic soft part has parameters varying smoothly (molded gradients) or abruptly (bonded multi-material). This leaf names the public API for specifying those fields, the type-level generalization that makes a uniform material a special case of a graded one, and the bridge to [`cf-design`](../../110-crate/02-coupling/04-cf-design.md)'s authoring layer.

## The `Field<Output = T>` trait

[Part 7 Ch 00](../../70-sdf-pipeline/00-sdf-primitive.md) introduces two distinct trait abstractions: the `Sdf` trait for the *geometric* signed-distance field (scalar-valued with a `grad` method that returns a unit normal), and the `Field<Output = T>` trait for *typed spatial fields* over reference space (method `sample(x_ref) -> T`). `Field<Output = T>` is the trait this chapter uses for spatially-varying material parameters:

- `Field<Output = f64>` — scalar parameters (Lamé $\mu, \lambda$; Mooney-Rivlin $C_{10}, C_{01}$; Ogden $\mu_i, \alpha_i$; HGO $k_1, k_2$; Prony $\tau_i, g_i$; thermal expansion $\alpha$).
- `Field<Output = Vec3>` — vector parameters (HGO fiber direction $a$; fiber resolvers normalize to unit length at the [`FiberDirection`](../06-anisotropic/02-direction-fields.md) consumer boundary, not at the `Field` boundary).
- `Field<Output = Tensor3>` — tensor parameters (full orthotropy moduli, anisotropic thermal expansion). A Pass-3-or-later extension, not on the Phase A–I roadmap.

Each `Field<Output = T>` exposes `fn sample(&self, x_ref: Vec3) -> T`. The implementation is type-specific — scalar fields evaluate a scalar SDF (or any scalar-valued function over reference space); vector fields evaluate component-wise or via a vector-valued parametric form; tensor fields evaluate and project onto the appropriate symmetric or PSD subspace if the constitutive law requires it. Consumers of the trait surface call `sample(x_ref)` without knowing the underlying representation.

The geometric `SdfField` from [Part 7 Ch 00](../../70-sdf-pipeline/00-sdf-primitive.md) is a *struct* (carrying the composed geometric SDF plus a bounding box and resolution hint), not a type alias; the material-side typed fields above compose *inside* `MaterialField`, a separate struct that aggregates one `Box<dyn Field<Output = T>>` per material parameter slot. The two structs serve different roles (geometry vs. per-parameter material) and do not substitute for one another.

## `MaterialField` aggregates per-parameter typed fields

The `MaterialField` struct from [Part 7 Ch 00](../../70-sdf-pipeline/00-sdf-primitive.md) holds one `Box<dyn Field<Output = T>>` per material parameter the wrapped `Material` trait surface exposes. A constant-everywhere material is the degenerate case — every parameter slot holds a `Field` implementation that returns the same value at every $x_\text{ref}$. A graded material puts a non-constant `Field` in one or more slots. The solver does not know or care: every per-element material evaluation goes through `sample(x_ref)`, and a constant-field `sample` returns the same value at every reference-space point.

This is the type-level generalization the parent's Claim 1 names: the constant case and the graded case share one trait surface, with the discrimination in the `Field` implementation rather than in the consuming code. There is no runtime branch on "is this material uniform or graded?" anywhere in the [`element/`](../../110-crate/00-module-layout/01-element.md) assembly path.

## Authoring path

`MaterialField` instances are authored in [`cf-design`](../../110-crate/02-coupling/04-cf-design.md), which exposes the SDF-construction primitives (CSG, smooth blends, displacement) committed in [Part 7 Ch 00 §01 operations](../../70-sdf-pipeline/00-sdf-primitive/01-operations.md). A designer building a multi-layer compliant cavity assembles a geometric `Sdf` for the shell (which the [meshing pipeline](../../70-sdf-pipeline/01-tet-strategies.md) consumes as `SdfField`) and per-parameter `Field<Output = T>` instances for the material parameters (which `MaterialField` aggregates and Ch 09 sampling consumes per element).

The two channels — geometric SDF and per-parameter typed field — share authoring primitives but are structurally distinct: the geometric SDF determines where the mesh lives, the typed fields determine what each per-element parameter set is. A material-only design change can move material boundaries without re-meshing if the geometry envelope is unchanged; a geometry change forces a re-mesh ([Part 7 Ch 04 live re-mesh](../../70-sdf-pipeline/04-live-remesh.md)) but the `MaterialField`'s typed fields sample cleanly into the new mesh's elements.

## Trait-surface contract

The `Material` trait stays scalar-parameterized at the trait level — `mu: f64`, not `mu: Box<dyn Field<Output = f64>>`. The field-to-scalar conversion happens in [`element/`](../../110-crate/00-module-layout/01-element.md) at element construction (and warm-restart from re-mesh): for each element, the assembly pass calls `MaterialField::sample` at the [per-element sample point](01-sampling.md), produces a per-element `Material` instance with scalar parameters, and stores it alongside the element's deformation gradient.

The Newton loop iterates over per-element scalar materials. The spatial-field machinery does not enter the per-iteration hot path; the per-element scalar materials are static within a step and the Newton iteration's nonlinearity is purely in $F$. This separation is what keeps the [factor-on-tape Cholesky path](../../50-time-integration/00-backward-euler.md) clean — the spatial-field evaluation cost is a one-time per-step overhead, not a per-iteration multiplier.

## What this sub-leaf commits the book to

- **`Field<Output = T>` is the trait for typed spatial material fields.** `f64` for scalars, `Vec3` for vectors (fiber direction), `Tensor3` as a future extension. The `sample(x_ref)` method is the only consumer-side API. The geometric `Sdf` trait is a distinct abstraction (scalar-valued with `grad`), not a specialization of `Field`.
- **`SdfField` is a struct for the geometric SDF, not a type alias.** It carries the composed `Sdf` implementation plus a bounding box and resolution hint. The per-material-parameter typed fields aggregate inside the separate `MaterialField` struct.
- **`MaterialField` aggregates per-parameter `Box<dyn Field<Output = T>>` instances.** Constant materials hold `Field` implementations that return constants; graded materials replace specific slots with non-constant fields. No runtime branch on uniformity.
- **The `Material` trait stays scalar-parameterized.** Field-to-scalar conversion happens in [`element/`](../../110-crate/00-module-layout/01-element.md) at construction (or warm-restart); the Newton hot path sees per-element scalar materials.
- **Geometry and material fields are independent channels.** Material boundaries can move without re-meshing when the geometry envelope is unchanged; geometry changes force re-mesh but typed fields sample cleanly into the new mesh.
- **Authoring goes through [`cf-design`](../../110-crate/02-coupling/04-cf-design.md).** Same SDF-construction primitives as the geometry channel; `MaterialField` is the dispatch-aggregator at the `sim-soft` boundary.
