//! Smoke tests for the `(E, ν)` material constructors
//! [`NeoHookean::from_young_poisson`] and
//! [`Yeoh::from_young_poisson_and_c2`].
//!
//! These are the textbook engineering entry points (parallel to the
//! Lamé-parameter `from_lame` / `from_lame_and_c2` constructors the rest
//! of the suite drives). They pin the isotropic-elasticity conversion
//! `μ = E / 2(1+ν)`, `λ = E ν / [(1+ν)(1−2ν)]` against independently-known
//! values and exercise the `ν ≥ 0.45` validity-cap panic.

use sim_soft::{NeoHookean, Yeoh};

/// `(E = 260 kPa, ν = 0.3)` converts to the exact Lamé pair
/// `μ = 100 kPa, λ = 150 kPa`:
/// `μ = 260000 / 2.6 = 100000`, `λ = 260000·0.3 / (1.3·0.4) = 150000`.
#[test]
fn neo_hookean_from_young_poisson_matches_known_lame() {
    let nh = NeoHookean::from_young_poisson(260_000.0, 0.3);
    assert!((nh.mu() - 100_000.0).abs() < 1e-6, "μ = {}", nh.mu());
    assert!(
        (nh.lambda() - 150_000.0).abs() < 1e-6,
        "λ = {}",
        nh.lambda()
    );
}

/// `(E = 280 kPa, ν = 0.4)` converts to `μ = 100 kPa, λ = 400 kPa` — the
/// skeleton's own material point (`μ = 1e5`, `λ = 4e5`, ν ≈ 0.40). The
/// `(E, ν)` path must reproduce the Lamé literal the scenes use directly.
#[test]
fn neo_hookean_from_young_poisson_reproduces_skeleton_point() {
    let nh = NeoHookean::from_young_poisson(280_000.0, 0.4);
    assert!((nh.mu() - 100_000.0).abs() < 1e-6, "μ = {}", nh.mu());
    assert!(
        (nh.lambda() - 400_000.0).abs() < 1e-6,
        "λ = {}",
        nh.lambda()
    );
}

/// The standalone compressible law caps Poisson at 0.45.
#[test]
#[should_panic(expected = "nu < 0.45")]
fn neo_hookean_from_young_poisson_panics_above_cap() {
    #[allow(clippy::let_underscore_must_use)]
    let _ = NeoHookean::from_young_poisson(280_000.0, 0.45);
}

/// Yeoh's `(E, ν, C₂)` constructor shares the same conversion and carries
/// `C₂` through unchanged: `(280 kPa, 0.4, 5 kPa) → μ = 100 kPa,
/// λ = 400 kPa, C₂ = 5 kPa`.
#[test]
fn yeoh_from_young_poisson_and_c2_matches_known_lame_and_carries_c2() {
    let y = Yeoh::from_young_poisson_and_c2(280_000.0, 0.4, 5_000.0);
    assert!((y.mu() - 100_000.0).abs() < 1e-6, "μ = {}", y.mu());
    assert!((y.lambda() - 400_000.0).abs() < 1e-6, "λ = {}", y.lambda());
    assert!((y.c2() - 5_000.0).abs() < 1e-12, "C₂ = {}", y.c2());
}

/// Same 0.45 cap as the `NeoHookean` sibling.
#[test]
#[should_panic(expected = "nu < 0.45")]
fn yeoh_from_young_poisson_and_c2_panics_above_cap() {
    #[allow(clippy::let_underscore_must_use)]
    let _ = Yeoh::from_young_poisson_and_c2(280_000.0, 0.5, 5_000.0);
}
