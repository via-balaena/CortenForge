//! Shared test helpers for `sim-soft` integration tests.
//!
//! Per Rust integration-test convention, each `tests/<name>.rs` is its
//! own crate. Files that need shared helpers declare `mod common;`,
//! which compiles only this `tests/common/mod.rs` module into that
//! crate. Files that don't declare it never see this module.

// Each declaring crate compiles this whole module but uses only the helpers it
// needs, so every helper is dead code in some binary — the standard
// `tests/common` friction, unavoidable once more than one helper lives here.
// (It became visible when `garcia_bonded_correction` landed alongside
// `assert_neo_hookean_bit_equal`: no single test file uses both.) Scoped to
// this shared-helper module; production `src/` carries no such blanket.
#![allow(dead_code)]

use nalgebra::Matrix3;
use sim_soft::{Material, NeoHookean};

/// Compare two [`NeoHookean`] instances by bit-equal first-Piola at a
/// fixed deformation gradient that decouples both Lamé parameters.
///
/// At `F = diag(2, 1, 1)`:
///
/// - `F⁻ᵀ = diag(0.5, 1, 1)`, `J = 2`, `ln J = ln 2`.
/// - `P = μ (F − F⁻ᵀ) + λ (ln J) F⁻ᵀ`
///   `= μ · diag(1.5, 0, 0) + λ · ln 2 · diag(0.5, 1, 1)`.
///
/// So `P[(1,1)] = λ · ln 2`: bit-equal `P[(1,1)]` ⇒ bit-equal `λ`,
/// since the `ln 2` factor is the same `f64` in both calls. With `λ`
/// pinned, `P[(0,0)] = 1.5 μ + 0.5 λ · ln 2`: bit-equal `P[(0,0)]` ⇒
/// bit-equal `μ`. Bit-equality of the full `P` matrix at this `F` is
/// therefore necessary and sufficient for bit-equality of the
/// underlying `(μ, λ)` pair through the [`Material`] trait surface.
///
/// `NeoHookean`'s scalar parameters are private (commit 1 surface
/// lock; widening accessors is a Phase H concern), so the comparison
/// goes through the trait surface rather than reading the fields
/// directly.
pub fn assert_neo_hookean_bit_equal(actual: &NeoHookean, expected: &NeoHookean) {
    let f = Matrix3::new(2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    let p_actual = actual.first_piola(&f);
    let p_expected = expected.first_piola(&f);
    for i in 0..3 {
        for j in 0..3 {
            assert_eq!(
                p_actual[(i, j)].to_bits(),
                p_expected[(i, j)].to_bits(),
                "first_piola[({i},{j})] differs",
            );
        }
    }
}

/// Garcia bonded bottom-effect correction `Δ_G(ν, χ)` (arXiv:2406.17157
/// Note S2 Eq. S4), `χ = √(Rδ)/h`.
///
/// **Single source of truth**, shared by `bonded_layer_indentation.rs` (the
/// `#676` demand-#1 gate) and `tet10_indentation_demand1.rs` (its Tet10 arm).
/// It lives here because the two must compare against the *same* oracle: a
/// second copy was written for the Tet10 arm and was wrong — a plausible-
/// looking polynomial rather than this expansion — which put the measured
/// `RATIO` at 3.43 instead of 1.13. Verified against the source's published
/// coefficients at ν = 0.5 / 0.49; see the `#676` module docstring.
#[must_use]
pub fn garcia_bonded_correction(nu: f64, chi: f64) -> f64 {
    let a0 = -(1.3442f64.mul_add(nu * nu, 1.4678f64.mul_add(-nu, 1.2876))) / (1.0 - nu);
    let b0 = 1.5164f64.mul_add(nu * nu, 1.0277f64.mul_add(-nu, 0.6387)) / (1.0 - nu);
    let pi = std::f64::consts::PI;
    let ca = (2.0 * a0 / pi).abs(); // leading, analytically exact
    let cb = 301.0 * pi * a0 * a0 / 2000.0;
    let cc = -pi * (31.0 * a0.powi(3) / 255.0 + 106.0 * b0 / 491.0);
    let cd = pi * (24.0 * a0.powi(4) / 245.0 + 40.0 * b0 * a0 / 97.0);
    let ce = -pi * (a0.powi(5) / 22.0 + 2.0 * b0 * a0 * a0 / 9.0);
    // Horner in χ.
    ce.mul_add(chi, cd)
        .mul_add(chi, cc)
        .mul_add(chi, cb)
        .mul_add(chi, ca)
        .mul_add(chi, 1.0)
}
