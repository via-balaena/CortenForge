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
use sim_soft::{
    ContactModel, ContactPair, MaterialField, PenaltyRigidContact, RigidPlane, SingleTetMesh, Vec3,
    VertexId,
};

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

// =====================================================================
// PenaltyRigidContact (V-2 unit) — Phase 5 commit 4
// =====================================================================
//
// V-2 invariant per `phase_5_penalty_contact_scope.md` §1: penalty
// energy / gradient / Hessian unit-level + FD-consistent. This file
// covers the unit-level cases (formula correctness, gradient
// direction, Hessian eigenstructure, ccd_toi sentinel, active-pair
// filtering and ordering); FD self-consistency lives in
// `tests/contact_fd.rs`.

const KAPPA: f64 = 1.0e4;
const D_HAT: f64 = 1.0e-3;

fn penalty_z_floor() -> PenaltyRigidContact {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT)
}

const fn pair_v0_p0() -> ContactPair {
    ContactPair::Vertex {
        vertex_id: 0,
        primitive_id: 0,
    }
}

#[test]
fn penalty_energy_outside_band_is_zero() {
    let c = penalty_z_floor();
    // d = 5·d̂ — well outside band; formula branch returns bit-exact 0.
    let positions = [Vec3::new(0.0, 0.0, 5.0 * D_HAT)];
    assert_eq!(
        c.energy(&pair_v0_p0(), &positions).to_bits(),
        0.0_f64.to_bits()
    );
}

#[test]
fn penalty_energy_at_band_boundary_is_zero() {
    let c = penalty_z_floor();
    // d = d̂ exactly — boundary case; the `>= d_hat` branch picks zero.
    let positions = [Vec3::new(0.0, 0.0, D_HAT)];
    assert_eq!(
        c.energy(&pair_v0_p0(), &positions).to_bits(),
        0.0_f64.to_bits()
    );
}

#[test]
fn penalty_energy_at_known_overlap() {
    let c = penalty_z_floor();
    let d = 0.5 * D_HAT;
    let positions = [Vec3::new(0.0, 0.0, d)];
    let expected = 0.5 * KAPPA * (D_HAT - d) * (D_HAT - d);
    assert_relative_eq!(
        c.energy(&pair_v0_p0(), &positions),
        expected,
        max_relative = 1.0e-15,
        epsilon = 1.0e-15
    );
}

#[test]
fn penalty_energy_at_full_overlap_on_surface() {
    let c = penalty_z_floor();
    // d = 0 on the plane — gap = d̂ → E = ½ κ d̂².
    let positions = [Vec3::new(0.0, 0.0, 0.0)];
    let expected = 0.5 * KAPPA * D_HAT * D_HAT;
    assert_relative_eq!(
        c.energy(&pair_v0_p0(), &positions),
        expected,
        max_relative = 1.0e-15,
        epsilon = 1.0e-15
    );
}

#[test]
fn penalty_energy_below_surface_at_known_penetration() {
    let c = penalty_z_floor();
    // d = -d̂ (penetrating) — gap = 2·d̂ → E = ½ κ (2 d̂)² = 2 κ d̂².
    let d = -D_HAT;
    let positions = [Vec3::new(0.0, 0.0, d)];
    let expected = 0.5 * KAPPA * (D_HAT - d) * (D_HAT - d);
    assert_relative_eq!(
        c.energy(&pair_v0_p0(), &positions),
        expected,
        max_relative = 1.0e-15,
        epsilon = 1.0e-15
    );
}

#[test]
fn penalty_gradient_outside_band_is_empty() {
    let c = penalty_z_floor();
    let positions = [Vec3::new(0.0, 0.0, 5.0 * D_HAT)];
    let g = c.gradient(&pair_v0_p0(), &positions);
    assert!(g.contributions.is_empty());
}

#[test]
fn penalty_gradient_direction_at_active_config() {
    // R-5 lens (v): the penalty gradient at active config points
    // *opposite* the outward normal, mirroring the elastic case where
    // f_int points further into the deformed configuration. With the
    // plane's outward normal n = +ẑ and a vertex above the plane at
    // d = 0.4·d̂, the gradient z-component is −κ·(d̂−d) < 0, so the
    // restoring force on the vertex (= −gradient) is along +ẑ —
    // pushing the vertex away from the rigid body.
    let c = penalty_z_floor();
    let d = 0.4 * D_HAT;
    let positions = [Vec3::new(0.0, 0.0, d)];
    let g = c.gradient(&pair_v0_p0(), &positions);
    assert_eq!(g.contributions.len(), 1);
    let (vid, force) = g.contributions[0];
    assert_eq!(vid, 0);
    let expected_z = -KAPPA * (D_HAT - d);
    // x and y are zero (IEEE may produce ±0 from the `−κ·(d̂−d)·0`
    // multiplication; `abs()` normalizes the sign so the bit-equality
    // assertion holds for both representations of zero — same magnitude
    // claim either way).
    assert_eq!(force.x.abs().to_bits(), 0_u64);
    assert_eq!(force.y.abs().to_bits(), 0_u64);
    assert_relative_eq!(
        force.z,
        expected_z,
        max_relative = 1.0e-15,
        epsilon = 1.0e-15
    );
}

#[test]
fn penalty_gradient_magnitude_scales_with_overlap() {
    let c = penalty_z_floor();
    for d_factor in [0.75, 0.25, -1.0_f64] {
        let d = d_factor * D_HAT;
        let positions = [Vec3::new(0.0, 0.0, d)];
        let g = c.gradient(&pair_v0_p0(), &positions);
        let expected_mag = KAPPA * (D_HAT - d);
        let actual_mag = g.contributions[0].1.norm();
        assert_relative_eq!(
            actual_mag,
            expected_mag,
            max_relative = 1.0e-12,
            epsilon = 1.0e-12
        );
    }
}

#[test]
fn penalty_hessian_outside_band_is_empty() {
    let c = penalty_z_floor();
    let positions = [Vec3::new(0.0, 0.0, 5.0 * D_HAT)];
    let h = c.hessian(&pair_v0_p0(), &positions);
    assert!(h.contributions.is_empty());
}

#[test]
#[allow(clippy::expect_used)]
fn penalty_hessian_is_rank_one_along_normal() {
    // Oblique normal so all 9 entries of κ·n⊗n are nonzero — exercises
    // every term of the outer product. Hessian eigenstructure: λ ∈
    // {0, 0, κ}, eigenvector of nonzero eigenvalue parallel to n.
    let normal = Vec3::new(1.0, 1.0, 1.0).normalize();
    let plane = RigidPlane::new(normal, 0.0);
    let penalty = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT);
    let positions = [0.4 * D_HAT * normal];
    let hessian = penalty.hessian(&pair_v0_p0(), &positions);
    assert_eq!(hessian.contributions.len(), 1);
    let (row_vid, col_vid, block) = hessian.contributions[0];
    assert_eq!(row_vid, 0);
    assert_eq!(col_vid, 0);

    let eig = block.symmetric_eigen();
    let mut eigenvalues: Vec<f64> = eig.eigenvalues.iter().copied().collect();
    // `total_cmp` is total over `f64` — eigenvalues of a real
    // symmetric matrix can never be NaN, but using `total_cmp`
    // sidesteps `partial_cmp.unwrap()` either way.
    eigenvalues.sort_by(f64::total_cmp);
    assert_relative_eq!(eigenvalues[0], 0.0, epsilon = 1.0e-9);
    assert_relative_eq!(eigenvalues[1], 0.0, epsilon = 1.0e-9);
    assert_relative_eq!(eigenvalues[2], KAPPA, max_relative = 1.0e-12);

    // Eigenvector of the nonzero eigenvalue is ±n (sign is arbitrary
    // per nalgebra's eigen convention; magnitude of the dot product is
    // 1 either way).
    let (max_idx, _) = eig
        .eigenvalues
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.total_cmp(b))
        .expect("3x3 symmetric matrix has 3 eigenvalues");
    let eigvec = eig.eigenvectors.column(max_idx);
    let dot_with_normal = eigvec.dot(&normal).abs();
    assert_relative_eq!(dot_with_normal, 1.0, max_relative = 1.0e-12);
}

#[test]
fn penalty_hessian_block_is_diagonal_at_vertex_id() {
    // Different vertex_id propagates through the contributions tuple
    // — verifies the implementation reads vertex_id from the pair, not
    // from a hardcoded 0.
    let c = penalty_z_floor();
    let positions = [
        Vec3::new(0.0, 0.0, 5.0 * D_HAT), // out of band
        Vec3::new(0.0, 0.0, 5.0 * D_HAT), // out of band
        Vec3::new(0.0, 0.0, 0.4 * D_HAT), // active
    ];
    let pair = ContactPair::Vertex {
        vertex_id: 2,
        primitive_id: 0,
    };
    let h = c.hessian(&pair, &positions);
    assert_eq!(h.contributions.len(), 1);
    assert_eq!(h.contributions[0].0, 2);
    assert_eq!(h.contributions[0].1, 2);
}

#[test]
fn penalty_ccd_toi_returns_infinity() {
    let c = penalty_z_floor();
    let x0 = [Vec3::zeros()];
    let x1 = [Vec3::new(1.0, 1.0, 1.0)];
    let toi = c.ccd_toi(&pair_v0_p0(), &x0, &x1);
    assert!(toi.is_infinite() && toi.is_sign_positive());
}

#[test]
fn penalty_active_pairs_filters_by_d_hat() {
    let c = penalty_z_floor();
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    // Three vertices: above-band, in-band, penetrating. The mesh
    // argument is unused by the Phase 5 vertex-vs-primitive walk
    // (Mesh is part of the trait surface for Phase H IPC's edge-edge
    // / face-face cases that need topology); only `positions.len()`
    // governs the iteration.
    let positions = [
        Vec3::new(0.0, 0.0, 5.0 * D_HAT),  // above band — filtered out
        Vec3::new(0.0, 0.0, 0.5 * D_HAT),  // in band — included
        Vec3::new(0.0, 0.0, -0.5 * D_HAT), // penetrating — included
    ];
    let pairs = c.active_pairs(&mesh, &positions);
    assert_eq!(pairs.len(), 2);
    let &ContactPair::Vertex {
        vertex_id: v0,
        primitive_id: p0,
    } = &pairs[0];
    let &ContactPair::Vertex {
        vertex_id: v1,
        primitive_id: p1,
    } = &pairs[1];
    assert_eq!((v0, p0), (1, 0));
    assert_eq!((v1, p1), (2, 0));
}

#[test]
fn penalty_active_pairs_deterministic_ordering() {
    // 4 vertices × 2 primitives, all active — pins the outer-vertex /
    // inner-primitive walk per Decision M.
    let plane_z0 = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let plane_z2 = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 2.0 * D_HAT);
    let c = PenaltyRigidContact::with_params(vec![plane_z0, plane_z2], KAPPA, D_HAT);
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    // Each vertex at z = 0.5·d̂ — d = 0.5·d̂ for plane_z0 (in band) and
    // d = −1.5·d̂ for plane_z2 (penetrating, also < d̂). All 8 pairs
    // active.
    let positions = [
        Vec3::new(0.0, 0.0, 0.5 * D_HAT),
        Vec3::new(1.0, 0.0, 0.5 * D_HAT),
        Vec3::new(0.0, 1.0, 0.5 * D_HAT),
        Vec3::new(1.0, 1.0, 0.5 * D_HAT),
    ];
    let pairs = c.active_pairs(&mesh, &positions);
    assert_eq!(pairs.len(), 8);
    let expected: Vec<(VertexId, u32)> = (0..4_u32)
        .flat_map(|v| (0..2_u32).map(move |p| (v, p)))
        .collect();
    let actual: Vec<(VertexId, u32)> = pairs
        .iter()
        .map(|p| {
            let &ContactPair::Vertex {
                vertex_id,
                primitive_id,
            } = p;
            (vertex_id, primitive_id)
        })
        .collect();
    assert_eq!(actual, expected);
}

#[test]
fn penalty_is_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<PenaltyRigidContact>();
}
