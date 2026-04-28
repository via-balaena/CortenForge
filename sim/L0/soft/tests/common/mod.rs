//! Shared test helpers for `sim-soft` integration tests.
//!
//! Per Rust integration-test convention, each `tests/<name>.rs` is its
//! own crate. Files that need shared helpers declare `mod common;`,
//! which compiles only this `tests/common/mod.rs` module into that
//! crate. Files that don't declare it never see this module.

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
