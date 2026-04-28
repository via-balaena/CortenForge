//! Unit tests for the `Field<T>` trait, `ConstantField<T>`,
//! `LayeredScalarField`, and `BlendedScalarField`.
//!
//! Phase 4 commit 1 covered `Field<T>` + `ConstantField<T>`. Commit 2
//! extends this file with concentric-shell (`LayeredScalarField`) and
//! smoothstep-blend (`BlendedScalarField`) cases per
//! `phase_4_multi_material_scope.md` §0 net-new files row + §8 commit 2
//! gate.
//!
//! Layout:
//!
//! 1. **`ConstantField` cases** (commit 1) — scalar bit-equality across
//!    five probes; `Vec3`-typed witness; trait surface is `Send + Sync`.
//! 2. **`LayeredScalarField` cases** (commit 2) — per-shell correctness
//!    over a 3-shell concentric `SphereSdf`; boundary convention at
//!    `phi == threshold` falls to the outer shell; constructor panics
//!    on non-monotone thresholds and on `values.len()` mismatch.
//! 3. **`BlendedScalarField` cases** (commit 2) — bit-exact clamp at
//!    both band edges and at the SDF zero set; cubic Hermite kernel
//!    midpoint is exactly `0.5`; constructor panics on non-positive
//!    `band_half_width`.
//! 4. **Compile-time `Send + Sync` witness** for every concrete field
//!    type currently in this crate.
//!
//! Bit-equality (`f64::to_bits`) is the assertion style throughout the
//! cases that *must* be exact — both fields are pure functions over
//! `x_ref` with no transcendentals on the hot paths the tests exercise,
//! so any drift would be a real bug.

use sim_soft::{BlendedScalarField, ConstantField, Field, LayeredScalarField, SphereSdf, Vec3};

// ---------------------------------------------------------------------
// 1. ConstantField cases (commit 1)
// ---------------------------------------------------------------------

#[test]
fn constant_field_f64_returns_same_value_everywhere() {
    let field: ConstantField<f64> = ConstantField::new(1.234e5);
    let probes = [
        Vec3::zeros(),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-3.7, 2.1, 0.05),
        Vec3::new(1e-12, -1e-12, 1e-12),
        Vec3::new(1e12, 1e12, 1e12),
    ];
    let expected_bits = 1.234e5_f64.to_bits();
    for p in probes {
        // Bit-equality via `to_bits`: `ConstantField::sample` clones a
        // stored scalar — no arithmetic, no drift — so any per-probe
        // mismatch would be a real bug, not float noise. Comparing bit
        // patterns sidesteps `clippy::float_cmp` while expressing the
        // exact contract.
        assert_eq!(field.sample(p).to_bits(), expected_bits, "probe = {p:?}");
    }
}

#[test]
fn constant_field_vec3_returns_same_vector_everywhere() {
    let direction = Vec3::new(1.0, 0.0, 0.0);
    let field: ConstantField<Vec3> = ConstantField::new(direction);
    let probes = [
        Vec3::zeros(),
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(-2.0, 3.0, -4.0),
    ];
    for p in probes {
        assert_eq!(field.sample(p), direction, "probe = {p:?}");
    }
}

// ---------------------------------------------------------------------
// 2. LayeredScalarField cases (commit 2)
// ---------------------------------------------------------------------

/// 3-shell concentric `SphereSdf` of radius `R = 1.0`. Thresholds are
/// chosen well inside the sphere so every probe lands cleanly in one
/// shell. The geometry mirrors the IV-4 / IV-5 layered silicone device's
/// outer / middle / inner partition, just at a unit-radius scale that
/// keeps the test arithmetic easy to read. `phi(x) = ‖x‖ − 1`, so:
///
/// - `phi < -0.6` (innermost ~ ‖x‖ < 0.4) → `values[0] = 1.0`
/// - `-0.6 ≤ phi < -0.3` (middle ~ 0.4 ≤ ‖x‖ < 0.7) → `values[1] = 2.0`
/// - `phi ≥ -0.3` (outer + exterior ~ ‖x‖ ≥ 0.7) → `values[2] = 3.0`
///
/// Returns the field plus the three known-shell probe points + their
/// expected sample values.
fn three_shell_concentric() -> (LayeredScalarField, [(Vec3, f64); 3]) {
    let sdf = Box::new(SphereSdf { radius: 1.0 });
    let thresholds = vec![-0.6, -0.3];
    let values = vec![1.0, 2.0, 3.0];
    let field = LayeredScalarField::new(sdf, thresholds, values);

    let probes = [
        // ‖x‖ = 0.1 → phi = -0.9 → innermost
        (Vec3::new(0.1, 0.0, 0.0), 1.0),
        // ‖x‖ = 0.5 → phi = -0.5 → middle
        (Vec3::new(0.5, 0.0, 0.0), 2.0),
        // ‖x‖ = 0.9 → phi = -0.1 → outer
        (Vec3::new(0.9, 0.0, 0.0), 3.0),
    ];
    (field, probes)
}

#[test]
fn layered_scalar_field_returns_per_shell_value() {
    let (field, probes) = three_shell_concentric();
    for (p, expected) in probes {
        // Bit-equality: `sample` looks up a stored f64 — no arithmetic,
        // no drift — so per-shell values come back to the bit.
        assert_eq!(
            field.sample(p).to_bits(),
            expected.to_bits(),
            "probe = {p:?}",
        );
    }
}

#[test]
fn layered_scalar_field_boundary_belongs_to_outer_shell() {
    // At ‖x‖ = 0.4 → phi = -0.6, exactly on the inner threshold. The
    // documented convention (`partition_point(|&t| t <= phi)`) puts the
    // boundary point in the *outer* shell, i.e., values[1] = 2.0.
    let (field, _) = three_shell_concentric();
    let on_inner_boundary = Vec3::new(0.4, 0.0, 0.0);
    assert_eq!(
        field.sample(on_inner_boundary).to_bits(),
        2.0_f64.to_bits(),
        "phi == threshold[0] should bucket into the outer shell (values[1]=2.0)",
    );
}

#[test]
#[should_panic(expected = "strictly monotone-increasing")]
fn layered_scalar_field_panics_on_non_monotone_thresholds() {
    let sdf = Box::new(SphereSdf { radius: 1.0 });
    // thresholds[1] == thresholds[0] is non-strict: must panic.
    let _field = LayeredScalarField::new(sdf, vec![-0.5, -0.5], vec![1.0, 2.0, 3.0]);
}

#[test]
#[should_panic(expected = "values.len()")]
fn layered_scalar_field_panics_on_value_count_mismatch() {
    let sdf = Box::new(SphereSdf { radius: 1.0 });
    // Two thresholds need three values; supplying two must panic.
    let _field = LayeredScalarField::new(sdf, vec![-0.6, -0.3], vec![1.0, 2.0]);
}

// ---------------------------------------------------------------------
// 3. BlendedScalarField cases (commit 2)
// ---------------------------------------------------------------------

/// 1.0-radius `SphereSdf` blending `inside_field = 10.0` and
/// `outside_field = 30.0` over a band of half-width `0.2` around the
/// sphere surface. `phi(x) = ‖x‖ − 1`.
fn unit_sphere_blend() -> BlendedScalarField {
    let sdf = Box::new(SphereSdf { radius: 1.0 });
    let inside: Box<dyn Field<f64>> = Box::new(ConstantField::new(10.0));
    let outside: Box<dyn Field<f64>> = Box::new(ConstantField::new(30.0));
    BlendedScalarField::new(sdf, inside, outside, 0.2)
}

#[test]
fn blended_scalar_field_deep_inside_returns_inside_value() {
    let field = unit_sphere_blend();
    // ‖x‖ = 0.1 → phi = -0.9 ≪ -0.2; smoothstep clamps s = 0,
    // outside_weight = 0 exactly, blend returns inside value bit-exact.
    let p = Vec3::new(0.1, 0.0, 0.0);
    assert_eq!(field.sample(p).to_bits(), 10.0_f64.to_bits());
}

#[test]
fn blended_scalar_field_deep_outside_returns_outside_value() {
    let field = unit_sphere_blend();
    // ‖x‖ = 2.0 → phi = +1.0 ≫ +0.2; smoothstep clamps s = 1,
    // outside_weight = 1 exactly, blend returns outside value bit-exact.
    let p = Vec3::new(2.0, 0.0, 0.0);
    assert_eq!(field.sample(p).to_bits(), 30.0_f64.to_bits());
}

#[test]
fn blended_scalar_field_at_zero_set_returns_midpoint() {
    let field = unit_sphere_blend();
    // ‖x‖ = 1.0 → phi = 0; s = 0.5; cubic smoothstep s²(3 − 2s) =
    // 0.25 · 2 = 0.5 exactly. Blend = 0.5 · 10 + 0.5 · 30 = 20 exactly.
    let p = Vec3::new(1.0, 0.0, 0.0);
    assert_eq!(field.sample(p).to_bits(), 20.0_f64.to_bits());
}

#[test]
fn blended_scalar_field_clamps_bit_exact_at_band_edges() {
    let field = unit_sphere_blend();
    // ‖x‖ = 0.8 → phi = -0.2 exactly → s = 0 exactly → inside.
    let on_inner_band = Vec3::new(0.8, 0.0, 0.0);
    assert_eq!(field.sample(on_inner_band).to_bits(), 10.0_f64.to_bits());
    // ‖x‖ = 1.2 → phi = +0.2 exactly → s = 1 exactly → outside.
    let on_outer_band = Vec3::new(1.2, 0.0, 0.0);
    assert_eq!(field.sample(on_outer_band).to_bits(), 30.0_f64.to_bits());
}

#[test]
#[should_panic(expected = "strictly positive and finite")]
fn blended_scalar_field_panics_on_zero_band() {
    let sdf = Box::new(SphereSdf { radius: 1.0 });
    let inside: Box<dyn Field<f64>> = Box::new(ConstantField::new(10.0));
    let outside: Box<dyn Field<f64>> = Box::new(ConstantField::new(30.0));
    let _field = BlendedScalarField::new(sdf, inside, outside, 0.0);
}

#[test]
#[should_panic(expected = "strictly positive and finite")]
fn blended_scalar_field_panics_on_negative_band() {
    let sdf = Box::new(SphereSdf { radius: 1.0 });
    let inside: Box<dyn Field<f64>> = Box::new(ConstantField::new(10.0));
    let outside: Box<dyn Field<f64>> = Box::new(ConstantField::new(30.0));
    let _field = BlendedScalarField::new(sdf, inside, outside, -0.1);
}

// ---------------------------------------------------------------------
// 4. Compile-time Send + Sync witness
// ---------------------------------------------------------------------

// Compile-time witness: every concrete `Field` impl in this crate is
// `Send + Sync`. If the trait's `Send + Sync` supertrait is dropped, or
// any concrete type grows a non-Send field, the call below fails to
// type-check, which is exactly the regression we want.
#[test]
fn field_impls_are_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<ConstantField<f64>>();
    assert_send_sync::<ConstantField<Vec3>>();
    assert_send_sync::<LayeredScalarField>();
    assert_send_sync::<BlendedScalarField>();
}
