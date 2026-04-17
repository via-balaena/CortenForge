# Spatial fiber direction fields

The [Ch 06 parent's Claim 2](../06-anisotropic.md) commits to fiber direction as a field, not a parameter — molded silicone has flow-aligned directions, woven textiles have orthogonal families that rotate around curved surfaces, biological tissue follows anatomical axes. This leaf names the three resolver patterns `sim-soft` ships, the per-element sampling rule, and the bridge to [Ch 09's spatial material fields](../09-spatial-fields.md).

## The `FiberDirection` trait surface

A fiber-direction resolver `R: FiberDirection` is a function from reference-frame position to a unit vector (or a pair of unit vectors for two-fiber-family materials):

```rust
pub trait FiberDirection: Send + Sync {
    type Output;  // UnitVec3 for single-fiber, [UnitVec3; 2] for two-fiber
    fn at(&self, x_ref: Vec3) -> Self::Output;
}
```

The `at(x_ref)` method evaluates the field at a reference-configuration point. The output is normalized to unit length by the resolver itself; consumers of the trait can rely on $\|a\| = 1$ without a defensive normalization. The [`Anisotropic<M, R>` decorator](../00-trait-hierarchy/01-composition.md) holds the resolver as its `fiber: R` field and queries it once per element at construction time.

## Three resolver patterns

`sim-soft` ships three concrete resolvers covering the dominant authoring patterns:

**Uniform.** A constant unit vector across the entire mesh. Useful for thin sheets with a single weave direction or laminate plies with one fiber orientation. The simplest resolver:

```rust
pub struct UniformFiber(pub UnitVec3);

impl FiberDirection for UniformFiber {
    type Output = UnitVec3;
    fn at(&self, _: Vec3) -> UnitVec3 { self.0 }
}
```

**Axisymmetric.** Fiber direction wraps around an axis — circumferential, helical, or axial. Useful for tubular reinforcement (fiber-reinforced silicone tubes, braided sleeves, blood-vessel analogs). Parameterized by the axis line and the helix angle (zero for purely circumferential, $\pm \pi/2$ for purely axial):

```rust
pub struct AxisymmetricFiber {
    pub axis_origin: Vec3,
    pub axis_direction: UnitVec3,
    pub helix_angle: f64,  // radians off the circumferential plane
}
```

The `at(x_ref)` evaluation computes the radial offset from the axis (the component of $x_\text{ref} - \text{axis\_origin}$ perpendicular to `axis_direction`), takes the cross product of the axis with the radial direction to get the local circumferential vector, then mixes the circumferential vector with the axis direction by `helix_angle`. The result is a unit vector tangent to the helical path through the point.

**SDF-valued.** A general `Sdf<UnitVec3>` field authored in [`cf-design`](../../110-crate/02-coupling/04-cf-design.md), evaluated at the per-element sample point. Useful for arbitrary patterns — non-axisymmetric weaves, anatomical fiber paths from imaging, designed gradients. This is the resolver that makes fiber direction a first-class design variable:

```rust
pub struct SdfFiber(pub Sdf<UnitVec3>);

impl FiberDirection for SdfFiber {
    type Output = UnitVec3;
    fn at(&self, x_ref: Vec3) -> UnitVec3 {
        self.0.sample(x_ref).normalize()
    }
}
```

The defensive normalization protects against floating-point drift in the underlying SDF representation; the SDF authoring layer should produce near-unit vectors but the resolver does not assume it.

Two-fiber resolvers follow the same patterns with `type Output = [UnitVec3; 2]`. For an axisymmetric two-fiber HGO (e.g., $\pm 45°$ helical winding around an axis), the resolver returns the two helically-rotated vectors as a pair from a single `at(x_ref)` call.

## Per-element sampling

The resolver is queried once per element at the sample point committed to by [Ch 09 sampling](../09-spatial-fields/01-sampling.md). The result is stored alongside the per-element material descriptor in [`element/`](../../110-crate/00-module-layout/01-element.md), not on the resolver state.

The sampling is set at element construction time and refreshed on warm-restart from a [re-mesh](../../70-sdf-pipeline/04-live-remesh.md). Within a single Newton step the fiber direction is constant per element; deformation does not rotate it because the convention is reference-frame ([Ch 01 invariants leaf](../01-strain-measures/03-invariants.md), Ch 06 Claim 2).

## Smooth versus discontinuous transitions

Fiber direction can vary smoothly across the mesh (molded silicone with flow-aligned anisotropy, braided sleeves with continuously rotating fibers) or step discontinuously at material interfaces (layered composites with distinct fiber orientations per ply). Both are accommodated:

- **Smooth fields** sample cleanly at element sample points; the per-element fiber directions vary slowly across neighbouring elements, and the discrete fiber-direction field is a faithful representation of the continuous one at the mesh resolution.
- **Discontinuous fields** require the mesh to have element boundaries aligned with the fiber-direction discontinuity. A sharp transition sampled across an unaligned element produces a sample value that interpolates between the two sides, which is not physically meaningful — averaging $+45°$ and $-45°$ fibers gives a $0°$ direction that neither side actually contains. The [Part 3 multi-material interfaces leaf](../../30-discretization/03-interfaces.md) handles this by requiring element edges to lie on material-interface SDFs, which the [SDF tetrahedralization step](../../70-sdf-pipeline/01-tet-strategies.md) enforces during meshing.

The same machinery serves both regimes; the distinction is a meshing concern, not a fiber-resolver concern.

## Differentiability

The fiber direction enters the gradient chain through $I_4 = a \cdot C\, a$. For SDF-valued resolvers, the [reward gradient](../../100-optimization/00-forward.md) flows back through $a$ to the SDF parameters $\theta$ that define the fiber field — a designer choosing helix angle, ply orientation, or fiber-direction blend weights sees the gradient at the optimization step. The chain is per-element-sample, identical to the [Ch 09 gradient flow argument](../09-spatial-fields/02-gradient-flow.md): no custom VJP is needed unless the SDF evaluation goes through meshing, which is [Part 6 Ch 05's open problem](../../60-differentiability/05-diff-meshing.md) not unique to fiber fields.

For uniform and axisymmetric resolvers the parameters are scalars (axis origin, axis direction, helix angle); gradients flow through the resolver's `at(x_ref)` evaluation by reverse-mode autograd directly, no special handling.

## What this sub-leaf commits the book to

- **`FiberDirection` is a trait, not a single struct.** Three concrete resolvers ship: `UniformFiber`, `AxisymmetricFiber`, `SdfFiber`. Two-fiber-family variants follow with `type Output = [UnitVec3; 2]`.
- **Per-element sampling at the rule [Ch 09 sampling](../09-spatial-fields/01-sampling.md) commits to.** Resolved once at element construction and refreshed on warm-restart from re-mesh; constant per element within a Newton step.
- **Reference-frame convention is non-negotiable.** Fiber direction does not rotate with deformation. $I_4 = a \cdot C\, a$ remains rotation-invariant per the [Ch 01 invariants leaf](../01-strain-measures/03-invariants.md).
- **Discontinuous fiber transitions require mesh-aligned interfaces.** The fiber resolver does not handle the discontinuity; the meshing step does, via [Part 3 multi-material interface alignment](../../30-discretization/03-interfaces.md).
- **SDF-valued resolvers make fiber direction a first-class design variable.** Gradients flow through `Sdf<UnitVec3>` parameters at optimization time, identical to Ch 09's scalar-field path. No meshing-gradient barrier unless the SDF itself drives topology change.
