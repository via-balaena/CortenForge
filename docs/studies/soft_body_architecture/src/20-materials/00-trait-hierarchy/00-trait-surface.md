# The core trait surface

The `Material` trait is what every constitutive law in Part 2 — linear through Ogden-plus-HGO-plus-Prony-plus-thermal — presents to `sim-soft`'s Newton loop, gradcheck harness, and autograd tape. The parent [Ch 00 — trait hierarchy](../00-trait-hierarchy.md) committed to keeping the surface thin; this leaf writes the trait down in Rust, names what each of the four items is required to return, and states what the surface deliberately leaves out.

## The four items

```rust
use nalgebra::{Matrix3, SMatrix};

pub trait Material: Send + Sync {
    /// Strain-energy density psi(F) at deformation gradient F.
    fn energy(&self, f: &Matrix3<f64>) -> f64;

    /// First Piola-Kirchhoff stress P = d psi / dF.
    fn first_piola(&self, f: &Matrix3<f64>) -> Matrix3<f64>;

    /// Fourth-order tangent dP/dF as a 9x9 matrix in row-major index order.
    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9>;

    /// Validity-domain descriptor — see the validity sibling.
    fn validity(&self) -> ValidityDomain;
}
```

Four items. No lifetimes, no generic parameters on the methods, no interior mutability. `Send + Sync` so the same `Material` instance can serve every Gauss point of a parallel per-element loop without cloning.

**`energy(F) → ψ`.** The scalar strain-energy density at a single Gauss point. The [Newton loop of Part 5 Ch 00](../../50-time-integration/00-backward-euler.md) aggregates this across quadrature points into the total potential energy whose minimizer is the step's equilibrium.

**`first_piola(F) → P`.** The first Piola-Kirchhoff stress, equal to $\partial \psi / \partial F$. This is the quantity the [FEM assembly VJP](../../60-differentiability/01-custom-vjps/01-fem-assembly.md) evaluates per element and fans out into the global residual. Per the [notation-appendix convention](../../appendices/03-notation.md), $P$ is first Piola; Cauchy stress $\sigma = P F^T / J$ is derived from it rather than required on the trait.

**`tangent(F) → ∂P/∂F`.** The fourth-order tangent stored as a flattened 9×9 matrix — row-major in both the row-index $F_{ij}$ and the column-index $F_{kl}$. The per-element stiffness block in the global Newton tangent is $B^T \cdot \text{tangent}(F) \cdot B$ after the strain-displacement pull-back ([Part 6 Ch 01 — FEM assembly VJP](../../60-differentiability/01-custom-vjps/01-fem-assembly.md)); that pull-back lives in `element/`, not `material/`, which is what keeps this trait clean of element-shape knowledge.

**`validity(&self) → ValidityDomain`.** A pure-function descriptor of the strain / rotation / temperature / stretch regime in which the impl's closed-form expressions match the underlying physics to a declared tolerance. The descriptor itself is covered in the [validity sibling](02-validity.md); its presence on the trait is what makes "A-grade or it doesn't ship" testable for constitutive laws.

## Derived scalars via blanket defaults

Two downstream scalars are computed from the three required items and shipped as trait-method defaults — no impl needs to override unless there is a closed-form shortcut.

**Volumetric strain.** `fn volumetric_strain(&self, f) -> f64 { f.determinant() - 1.0 }`. Used by the [volume-loss failure detector](../../10-physical/02-what-goes-wrong/00-volume-loss.md) and by the near-incompressibility diagnostics in [Ch 05](../05-incompressibility.md).

**von Mises stress.** Derived from the Cauchy stress $\sigma = P F^T / J$ via the standard deviatoric-norm formula $\sigma_{\text{vM}} = \sqrt{\tfrac{3}{2}\, s : s}$ with $s = \sigma - \tfrac{1}{3}(\mathrm{tr}\,\sigma)\, I$. Used by the [stress-driven h-refinement criterion](../../30-discretization/02-adaptive/00-h-refinement.md) and by the [thermal visualization overlays](../../90-visual/04-thermal-viz.md).

The blanket default is what makes "every `Material` produces a von Mises field" structural rather than a convention every impl has to remember.

## What the trait does not carry

The four items fix the trait to an instantaneous, spatially local, rate-independent elastic response at a single material point.

- **No internal state.** No history variables, no per-element scratch. Rate-dependent behavior lives in the [composition sibling](01-composition.md)'s `Viscoelastic<M>` decorator, which owns the Prony-series history separately from the base material's parameters.
- **No fiber direction or temperature.** Anisotropic and thermal dependence are compositions covered in the same leaf; neither leaks onto the base trait surface.
- **No element-shape knowledge.** The trait never sees tet versus hex, nor shape-function gradients. Those are `element/`-layer concerns pulled back to the material trait via the strain-displacement matrix $B$.
- **No sparse-assembly knowledge.** The trait returns per-Gauss-point dense blocks; CSR pattern construction and per-element-to-global indexing live in `solver/`.

This separation is load-bearing for two downstream commitments: the [spatial-material-field composition](../09-spatial-fields.md) mixes different `Material` types in the same mesh without branching because the trait has no element-shape coupling; and the [GPU autograd tape](../../80-gpu/03-gpu-autograd.md) records per-Gauss-point calls without per-tet material-specific scratch because the trait is pure in $F$.

## What this sub-leaf commits the book to

- **Every constitutive law in Part 2 is one `impl Material`.** Linear, corotational, neo-Hookean, Mooney-Rivlin, Ogden, HGO — each one is a struct plus three closed-form methods plus a validity descriptor. The [gradcheck suite](../../110-crate/04-testing/03-gradcheck.md) hits every impl with the same test harness.
- **Rate-dependent, temperature-dependent, and anisotropic behavior are compositions, not trait variants.** No `RateDependentMaterial` or `ThermalMaterial` trait; those land as three decorators in the [composition sibling](01-composition.md).
- **The four-item surface is what downstream code is generic over.** [`element/`](../../110-crate/00-module-layout/01-element.md)'s per-tet assembly loop takes `M: Material` as a single type parameter. If the surface grew to eight items tomorrow, every downstream module would have to be audited; staying at four is a conscious budget.
