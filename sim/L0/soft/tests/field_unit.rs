//! Unit tests for the `Field<T>` trait + `ConstantField<T>` impl.
//!
//! Phase 4 commit 1 scope per `phase_4_multi_material_scope.md` §8:
//! `ConstantField` only. Commit 2 extends this file with concentric-shell
//! (`LayeredScalarField`) + smoothstep-blend (`BlendedScalarField`) cases.
//!
//! Three checks:
//!
//! 1. `ConstantField<f64>::sample` returns the stored scalar at every
//!    reference-space point — the load-bearing case for Phase 4's
//!    uniform-material default and the IV-1 regression net.
//! 2. `ConstantField<Vec3>::sample` returns the stored vector at every
//!    reference-space point — exercises the `T: Clone` generic surface
//!    Phase 4 ships day-one per scope memo Decision A. `Vec3` is the
//!    Phase H HGO fiber-direction type; testing it now confirms the
//!    `Field<T>` trait is genuinely generic, not f64-baked.
//! 3. The trait surface is `Send + Sync` — required so the upcoming
//!    `MaterialField` can hold `Box<dyn Field<f64>>` slots that thread
//!    across rayon-style parallel builds (forward-looking; Phase 4's
//!    mesher walks single-threaded).

use sim_soft::{ConstantField, Field, Vec3};

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

// Compile-time witness: `ConstantField<f64>` is `Send + Sync`. This is the
// real assertion — if `Field<T>: Send + Sync` is dropped from the trait
// definition or `ConstantField<T>` grows a non-Send field, the call below
// fails to type-check, which is exactly the regression we want.
#[test]
fn constant_field_is_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<ConstantField<f64>>();
    assert_send_sync::<ConstantField<Vec3>>();
}
