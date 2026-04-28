//! Unit tests for [`MaterialField`] (Phase 4 commit 3).
//!
//! Five cases per `phase_4_multi_material_scope.md` §8 commit 3 test
//! plan:
//!
//! 1. **`uniform` consistent across probes** — five probes, each
//!    `sample` is bit-equal to a directly-constructed
//!    `NeoHookean::from_lame(mu, lambda)`.
//! 2. **`from_fields` with two `ConstantField`s matches `uniform`** —
//!    proves the two construction paths converge bit-exactly.
//! 3. **`from_fields` with two `LayeredScalarField`s, per-shell** —
//!    proves the two slots sample independently against the same SDF.
//! 4. **Mixed slot types** — `μ` as `ConstantField`, `λ` as
//!    `LayeredScalarField` — proves heterogeneous compositions work.
//! 5. **Compile-time `Send + Sync` witness** for `MaterialField`.
//!
//! `NeoHookean`'s scalar parameters are private (commit 1 surface
//! lock; widening accessors is a Phase H concern), so the bit-equality
//! comparison goes through the `Material::first_piola` trait surface
//! at a fixed deformation gradient — see
//! [`common::assert_neo_hookean_bit_equal`] for the algebraic argument.

mod common;

use common::assert_neo_hookean_bit_equal;
use sim_soft::{
    ConstantField, Field, LayeredScalarField, MaterialField, NeoHookean, SphereSdf, Vec3,
};

// ---------------------------------------------------------------------
// 1. Uniform consistency
// ---------------------------------------------------------------------

#[test]
fn material_field_uniform_returns_consistent_neo_hookean() {
    // Skeleton-scene parameters from `material/neo_hookean.rs:25-26`
    // (Ecoflex-class, `ν ≈ 0.40`).
    let mu = 1.0e5;
    let lambda = 4.0e5;
    let field = MaterialField::uniform(mu, lambda);
    let reference = NeoHookean::from_lame(mu, lambda);
    let probes = [
        Vec3::zeros(),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.5, -0.25, 0.75),
        Vec3::new(-1.0e-3, 2.0e3, 0.0),
    ];
    for p in probes {
        assert_neo_hookean_bit_equal(&field.sample(p), &reference);
    }
}

// ---------------------------------------------------------------------
// 2. `from_fields` with constants matches `uniform`
// ---------------------------------------------------------------------

#[test]
fn material_field_from_fields_with_constants_matches_uniform() {
    let mu = 1.0e5;
    let lambda = 4.0e5;
    let uniform = MaterialField::uniform(mu, lambda);
    let from_fields = MaterialField::from_fields(
        Box::new(ConstantField::new(mu)),
        Box::new(ConstantField::new(lambda)),
    );
    let probes = [
        Vec3::zeros(),
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(-2.0, 3.0, -4.0),
    ];
    for p in probes {
        assert_neo_hookean_bit_equal(&from_fields.sample(p), &uniform.sample(p));
    }
}

// ---------------------------------------------------------------------
// 3. `from_fields` with two `LayeredScalarField`s
// ---------------------------------------------------------------------

#[test]
fn material_field_from_fields_with_layered_returns_per_shell() {
    // Two 2-shell layered fields keyed on the same `SphereSdf` of
    // radius 1.0. `phi(x) = ‖x‖ − 1`; threshold `-0.5` partitions:
    //
    // - `phi <  -0.5` (‖x‖ < 0.5) → inner shell  → μ=1.0e5, λ=4.0e5
    // - `phi >= -0.5` (‖x‖ ≥ 0.5) → outer shell  → μ=2.0e5, λ=8.0e5
    let mu_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        Box::new(SphereSdf { radius: 1.0 }),
        vec![-0.5],
        vec![1.0e5, 2.0e5],
    ));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        Box::new(SphereSdf { radius: 1.0 }),
        vec![-0.5],
        vec![4.0e5, 8.0e5],
    ));
    let field = MaterialField::from_fields(mu_field, lambda_field);

    // ‖x‖ = 0.1 → inner shell.
    let inner = Vec3::new(0.1, 0.0, 0.0);
    assert_neo_hookean_bit_equal(&field.sample(inner), &NeoHookean::from_lame(1.0e5, 4.0e5));

    // ‖x‖ = 0.9 → outer shell.
    let outer = Vec3::new(0.9, 0.0, 0.0);
    assert_neo_hookean_bit_equal(&field.sample(outer), &NeoHookean::from_lame(2.0e5, 8.0e5));
}

// ---------------------------------------------------------------------
// 4. Mixed slot types
// ---------------------------------------------------------------------

#[test]
fn material_field_mixed_field_types() {
    // `μ` as a constant; `λ` as a 2-shell layered field on the radius-1
    // `SphereSdf`. Same partition as case 3:
    //
    // - inner: μ = 1.5e5 (constant), λ = 4.0e5 (inner shell)
    // - outer: μ = 1.5e5 (constant), λ = 8.0e5 (outer shell)
    let mu_field: Box<dyn Field<f64>> = Box::new(ConstantField::new(1.5e5));
    let lambda_field: Box<dyn Field<f64>> = Box::new(LayeredScalarField::new(
        Box::new(SphereSdf { radius: 1.0 }),
        vec![-0.5],
        vec![4.0e5, 8.0e5],
    ));
    let field = MaterialField::from_fields(mu_field, lambda_field);

    let inner = Vec3::new(0.1, 0.0, 0.0);
    assert_neo_hookean_bit_equal(&field.sample(inner), &NeoHookean::from_lame(1.5e5, 4.0e5));

    let outer = Vec3::new(0.9, 0.0, 0.0);
    assert_neo_hookean_bit_equal(&field.sample(outer), &NeoHookean::from_lame(1.5e5, 8.0e5));
}

// ---------------------------------------------------------------------
// 5. Compile-time `Send + Sync` witness
// ---------------------------------------------------------------------

// If `MaterialField` ever grows a non-Send field (or one of its slot
// trait objects loses the `Send + Sync` supertrait), the call below
// fails to type-check — exactly the regression the witness catches.
#[test]
fn material_field_is_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<MaterialField>();
}
