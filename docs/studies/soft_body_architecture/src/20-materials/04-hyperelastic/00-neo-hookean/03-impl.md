# Implementation pseudocode

The [energy-and-stress](00-energy.md), [tangent](01-tangent.md), and [incompressible](02-incompressible.md) siblings derived the closed forms; this leaf collapses them into the Rust `impl Material for NeoHookean` that `sim-soft`'s [`material/`](../../../110-crate/00-module-layout/00-material.md) module ships. The form is pseudocode tied to real types — `nalgebra::Matrix3<f64>` on the hot path, the [Ch 00 `Material` trait surface](../../00-trait-hierarchy/00-trait-surface.md) at the public edge.

## Struct and constructors

```rust
use nalgebra::{Matrix3, SMatrix};

pub struct NeoHookean {
    mu: f64,
    lambda: f64,
}

impl NeoHookean {
    pub fn from_lame(mu: f64, lambda: f64) -> Self { Self { mu, lambda } }

    pub fn from_young_poisson(young: f64, nu: f64) -> Self {
        assert!(
            nu < 0.45,
            "standalone NeoHookean requires nu < 0.45; use the Ch 05 locking-fix decorator for higher Poisson ratios"
        );
        let mu = young / (2.0 * (1.0 + nu));
        let lambda = young * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
        Self { mu, lambda }
    }
}
```

Two constructors cover the two input conventions. [Part 1 Ch 04 material data](../../../10-physical/04-material-data.md) supplies $(E, \nu)$ from manufacturer sheets; internal code that computes Lamé directly uses the `from_lame` form. The Poisson-upper-bound assertion mirrors the standalone validity declaration below; going above $\nu = 0.45$ requires the Ch 05 locking-fix decorator, which carries its own constructor path that widens the Poisson bound to $0.499$.

## Hot-path evaluation

```rust
impl Material for NeoHookean {
    fn energy(&self, f: &Matrix3<f64>) -> f64 {
        let j = f.determinant();
        let i1 = f.norm_squared();  // Frobenius^2 = tr(F^T F)
        let ln_j = j.ln();
        0.5 * self.mu * (i1 - 3.0) - self.mu * ln_j + 0.5 * self.lambda * ln_j * ln_j
    }

    fn first_piola(&self, f: &Matrix3<f64>) -> Matrix3<f64> {
        let f_inv_t = f.try_inverse()
            .expect("singular F in NeoHookean (det F == 0); orientation is gated at the step boundary")
            .transpose();
        let ln_j = f.determinant().ln();
        self.mu * (f - f_inv_t) + self.lambda * ln_j * f_inv_t
    }

    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9> {
        let f_inv_t = f.try_inverse().unwrap().transpose();
        let ln_j = f.determinant().ln();
        // C_ijkl = mu * delta_ik delta_jl
        //        + (mu - lambda * ln_j) * F^{-T}_il * F^{-T}_kj
        //        + lambda * F^{-T}_ij * F^{-T}_kl
        // Flattened row-major into a 9x9. See the tangent sibling.
        assemble_tangent_9x9(self.mu, self.lambda, &f_inv_t, ln_j)
    }

    // fn validity(&self) below — separated into its own block for readability
}
```

Three callsite observations match the [tangent sibling](01-tangent.md)'s cache argument. First, `f_inv_t` is computed in both `first_piola` and `tangent`; `sim-soft`'s actual hot path threads an `EvaluationScratch` through the per-element loop so the inverse is computed once per Gauss-point visit rather than twice. Second, `ln_j` is shared too; same scratch. Third, the `try_inverse().expect(...)` is *not* the inversion-handling enforcement, though it reads like it. It tests **invertibility** (`det F != 0`), not **orientation** (`det F > 0`), and an inverted-but-invertible `F` passes it and reaches `det F.ln()`, which returns `NaN` rather than panicking. `RequireOrientation` declares the domain the closed form is valid on; the shipped enforcement is a step-boundary sweep in the solver (`CpuNewtonSolver::check_validity_at_step_start`), which requires a finite, strictly positive `det F` at every Gauss point. Nor is inversion a contact concern: the case that motivated the sweep arises from a prescribed-displacement warm start with no contact in it, so no barrier — IPC or penalty — covers it.

## Validity-domain declaration

```rust
impl Material for NeoHookean {
    // ... energy, first_piola, tangent above ...

    fn validity(&self) -> ValidityDomain {
        ValidityDomain {
            max_stretch_deviation: 1.0,          // approximately 100% stretch
            max_rotation: f64::INFINITY,         // rotation-invariant by construction
            poisson_range: (-1.0, 0.45),         // standalone bound; widened by Ch 05 composition
            temperature_range: None,             // rate-independent, no thermal coupling
            strain_rate_range: None,             // rate-independent
            inversion: InversionHandling::RequireOrientation,
        }
    }
}
```

The standalone Poisson upper bound is $0.45$ (matching [Ch 02 linear](../../02-linear.md) standalone). [Ch 05's mixed u-p](../../05-incompressibility/01-mixed-up.md) and [F-bar](../../05-incompressibility/02-f-bar.md) decorators rewrite the descriptor to widen the upper bound to $0.499$ when they wrap this impl; see the [validity sub-leaf](../../00-trait-hierarchy/02-validity.md) for the composition-widening pattern.

## Cancellation-safe small-strain path

At $F \approx I$, direct evaluation of $I_1 - 3$ and $\ln J$ loses digits to catastrophic cancellation. `NeoHookean` branches to a cancellation-safe path when $\|F - I\|_F$ falls below a threshold:

- $I_1 - 3 = 2\,\mathrm{tr}(\varepsilon) + \|\nabla u\|_F^2$ where $\varepsilon = \tfrac{1}{2}(\nabla u + (\nabla u)^T)$ and $\nabla u = F - I$
- $\ln J$ via `f64::ln_1p(j - 1.0)` when $|j - 1| < 10^{-2}$

This keeps the relative error below machine $\varepsilon$ at small strain — the regime where the [Phase B gradcheck](../../../110-crate/03-build-order.md) asserts six-digit agreement with the linear-elastic reference.

## Gradcheck hooks

The [gradcheck suite](../../../110-crate/04-testing/03-gradcheck.md) runs four assertions per `Material` impl; `NeoHookean` hooks into all four the same way:

- **Rotation invariance.** $\psi(R F) = \psi(F)$ for random $R \in SO(3)$ — passes structurally from the $(I_1, J)$ parameterization; the test is a sanity check that the impl did not regress.
- **Small-strain reduction.** Six-digit match against [`impl Material for Linear`](../../02-linear.md) at $F = I + 10^{-3}\, G$ for random $G$.
- **First Piola = $\partial \psi / \partial F$.** Finite-difference comparison of $P$ against a central-difference gradient of `energy`.
- **Tangent = $\partial P / \partial F$.** Finite-difference comparison of `tangent` against a central-difference Jacobian of `first_piola`.

The last two are where a bug in the hand derivation shows up. Passing all four is the Phase B gate for `NeoHookean`'s landing in `material/`.

## What this sub-leaf commits the book to

- **`NeoHookean` is roughly 100 lines of Rust.** Struct + two constructors + four trait methods + validity declaration + small-strain branch + gradcheck hooks. No internal state, no history, no temperature coupling. The density matches Ch 00's "thin trait surface" claim.
- **`EvaluationScratch` shares $F^{-T}$ and $\ln J$ between stress and tangent calls.** Listed here rather than on the base trait because it is a per-impl optimization, not a trait concern; the Ch 00 `Material` trait is pure in $F$.
- **Standalone `NeoHookean` caps at $\nu < 0.45$.** Higher Poisson ratios require wrapping in the Ch 05 mixed-u-p or F-bar decorator; the `from_young_poisson` constructor asserts on $\nu \geq 0.45$ rather than silently constructing an instance outside its declared regime.
- **Inversion handling is `RequireOrientation`, and the impl does not enforce it.** $\det F \le 0$ reaching the evaluation methods yields `NaN` (via $\ln$ of a non-positive determinant), not a panic. Enforcement lives in the solver's step-boundary validity sweep (`CpuNewtonSolver::check_validity_at_step_start` in `sim-soft`, whose `check_orientation` helper walks the Gauss points and the corner block), which rejects a non-finite or non-positive $\det F$ and names the offending element. It runs at the two step boundaries only — [the Newton loop](../../../50-time-integration/00-backward-euler.md)'s intermediate trial states are not gated. Treating this as an IPC-barrier concern was the earlier framing and is wrong in both halves: the impl does not panic, and the motivating inversions are prescribed-displacement, not contact.
