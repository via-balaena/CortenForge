//! Unit tests for kinematic rigid primitives — the rigid side of
//! one-way soft↔rigid penalty contact (Phase 5 commit 1).
//!
//! Phase 5 ships [`RigidPlane`] only (`RigidSphere` deferred to Phase H
//! per `phase_5_penalty_contact_scope.md` Decision B). The trait surface
//! and its sign convention (positive outside, negative inside) is pinned
//! here so future `RigidSphere` / `RigidBox` impls inherit the same
//! contract; V-2 / V-3 / V-3a / V-4 / V-5 all depend on it.
//!
//! Penalty `ContactModel` cases (V-2 unit + FD) land in this same file
//! at commit 4.

use approx::assert_relative_eq;
use sim_soft::{RigidPlane, Vec3};

// ---------------------------------------------------------------------
// Sign convention — positive outside, negative inside, zero on surface
// ---------------------------------------------------------------------

#[test]
fn rigid_plane_signed_distance_above_is_positive() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let p = Vec3::new(2.7, -1.3, 5.0);
    // Bit-equality: `signed_distance` is `p · n - offset` with n = ẑ
    // and offset = 0, so the result is exactly `p.z` — the x and y
    // dot-product terms drop because n has zero x and y components.
    assert_eq!(plane.signed_distance(p).to_bits(), 5.0_f64.to_bits());
}

#[test]
fn rigid_plane_signed_distance_below_is_negative() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let p = Vec3::new(2.7, -1.3, -3.0);
    assert_eq!(plane.signed_distance(p).to_bits(), (-3.0_f64).to_bits());
}

#[test]
fn rigid_plane_signed_distance_on_surface_is_zero() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let p = Vec3::new(2.7, -1.3, 0.0);
    assert_eq!(plane.signed_distance(p).to_bits(), 0.0_f64.to_bits());
}

#[test]
fn rigid_plane_signed_distance_with_nonzero_offset() {
    // Plane at z = 5.
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 5.0);
    let above = Vec3::new(1.0, 1.0, 7.5);
    let below = Vec3::new(1.0, 1.0, 4.0);
    assert_eq!(plane.signed_distance(above).to_bits(), 2.5_f64.to_bits());
    assert_eq!(plane.signed_distance(below).to_bits(), (-1.0_f64).to_bits());
}

#[test]
fn rigid_plane_signed_distance_oblique_normal() {
    // Plane through origin with normal (1, 1, 0) — distance from probe
    // (1, 1, 7) is √2 (z-component drops because the normal has no z).
    let plane = RigidPlane::new(Vec3::new(1.0, 1.0, 0.0), 0.0);
    let probe = Vec3::new(1.0, 1.0, 7.0);
    // Constructor normalize involves a sqrt — relative tolerance, not
    // bit-equality.
    assert_relative_eq!(
        plane.signed_distance(probe),
        2.0_f64.sqrt(),
        max_relative = 1e-15,
        epsilon = 1e-15
    );
}

// ---------------------------------------------------------------------
// outward_normal is constant for a plane (position-independent)
// ---------------------------------------------------------------------

#[test]
fn rigid_plane_outward_normal_is_constructor_normal_everywhere() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 3.0);
    let probes = [
        Vec3::zeros(),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-3.7, 2.1, 0.05),
        Vec3::new(1e-12, -1e-12, 1e12),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let expected = plane.normal();
    for p in probes {
        let got = plane.outward_normal(p);
        // Bit-equality: `outward_normal` clones the stored unit normal
        // with no arithmetic, so any drift would be a real bug.
        assert_eq!(got.x.to_bits(), expected.x.to_bits(), "x at {p:?}");
        assert_eq!(got.y.to_bits(), expected.y.to_bits(), "y at {p:?}");
        assert_eq!(got.z.to_bits(), expected.z.to_bits(), "z at {p:?}");
    }
}

// ---------------------------------------------------------------------
// Constructor robustness
// ---------------------------------------------------------------------

#[test]
fn rigid_plane_constructor_normalizes_non_unit_normal() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 2.0), 1.5);
    let n = plane.normal();
    assert_relative_eq!(n.x, 0.0, epsilon = 1e-15);
    assert_relative_eq!(n.y, 0.0, epsilon = 1e-15);
    assert_relative_eq!(n.z, 1.0, max_relative = 1e-15, epsilon = 1e-15);
    // Offset is stored verbatim — no arithmetic.
    assert_eq!(plane.offset().to_bits(), 1.5_f64.to_bits());
}

#[test]
#[should_panic(expected = "normal must have strictly positive finite length")]
fn rigid_plane_constructor_panics_on_zero_normal() {
    let _plane = RigidPlane::new(Vec3::zeros(), 0.0);
}

#[test]
#[should_panic(expected = "normal must have strictly positive finite length")]
fn rigid_plane_constructor_panics_on_nan_normal() {
    let _plane = RigidPlane::new(Vec3::new(f64::NAN, 0.0, 1.0), 0.0);
}

// ---------------------------------------------------------------------
// Compile-time Send + Sync witness
// ---------------------------------------------------------------------

#[test]
fn rigid_plane_is_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<RigidPlane>();
}
