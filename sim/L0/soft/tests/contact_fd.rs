//! Finite-difference vs. analytic derivative tests for
//! [`PenaltyRigidContact`] (Phase 5 commit 4, V-2 cont.).
//!
//! Two FD checks per `phase_5_penalty_contact_scope.md` §1 V-2:
//!
//! 1. `gradient` vs. central FD of `energy`.
//! 2. `hessian` vs. central FD of `gradient`.
//!
//! Both at `H = 1.5e-8` per `material_fd.rs:21` — central FD, square
//! root of f64 eps; relative bar `1.0e-5` per scope memo §1's
//! "5-digit relative-error bar." The penalty energy is *exactly*
//! quadratic in position when `d < d̂` (the SDF is linear in `p` for a
//! `RigidPlane`, so `E(p) = ½ κ (d̂ − p·n + offset)²` is a polynomial
//! of degree 2), so the central-FD truncation error is identically
//! zero — the floor is roundoff `|y|·ε_f64/h ≈ |y|·1.5e-8`.
//!
//! Test config uses an oblique plane normal `n = (1, 1, 1) / √3` so
//! all three spatial components of the gradient are nonzero —
//! axis-aligned `n = ẑ` would set two of three components to zero and
//! force absolute-tolerance comparisons that don't exercise the FD
//! machinery. The vertex sits at `d = 0.4·d̂` within the active band;
//! at `H = 1.5e-8 ≪ 0.4·d̂ = 4e-4`, FD perturbations stay five orders
//! of magnitude inside the band so the formula is smooth across the
//! probe — boundary discontinuity at `d = d̂` is not the regime here.
//! V-7 (commit 11) handles the FD-vs-κ direction; this file is purely
//! position-derivative coverage.

use approx::assert_relative_eq;
use sim_soft::{ContactModel, ContactPair, PenaltyRigidContact, RigidPlane, Vec3};

const KAPPA: f64 = 1.0e4;
const D_HAT: f64 = 1.0e-3;
const H: f64 = 1.5e-8;
const MAX_RELATIVE: f64 = 1.0e-5;

/// Absolute slack covers FD truncation + roundoff at `h = √eps`.
/// Central FD noise is bounded by `|y| · eps_f64 / h ≈ |y| · 1.5e-8`.
/// At the chosen config `|gradient| ≈ 6 N` → noise ≈ 9e-8;
/// `|Hessian entry| ≈ κ/3 ≈ 3.3e3` → noise ≈ 9e-8 ÷ 2H ≈ 3 (relative
/// to entry: ~1e-3). The `1.0e-3` bar covers both with comfortable
/// headroom — same regime as `material_fd.rs:28`.
const EPSILON: f64 = 1.0e-3;

fn oblique_penalty() -> (PenaltyRigidContact, Vec3) {
    let n = Vec3::new(1.0, 1.0, 1.0).normalize();
    let plane = RigidPlane::new(n, 0.0);
    let c = PenaltyRigidContact::with_params(vec![plane], KAPPA, D_HAT);
    (c, n)
}

const fn pair_v0_p0() -> ContactPair {
    ContactPair::Vertex {
        vertex_id: 0,
        primitive_id: 0,
    }
}

#[test]
fn penalty_gradient_matches_central_fd_of_energy() {
    let (c, n) = oblique_penalty();
    let p0 = 0.4 * D_HAT * n;
    let pair = pair_v0_p0();
    let force = c.gradient(&pair, &[p0]).contributions[0].1;

    for i in 0..3 {
        let mut e_i = Vec3::zeros();
        e_i[i] = 1.0;
        let p_pos = [p0 + H * e_i];
        let p_neg = [p0 - H * e_i];
        let fd = (c.energy(&pair, &p_pos) - c.energy(&pair, &p_neg)) / (2.0 * H);
        assert_relative_eq!(fd, force[i], max_relative = MAX_RELATIVE, epsilon = EPSILON);
    }
}

#[test]
fn penalty_hessian_matches_central_fd_of_gradient() {
    let (c, n) = oblique_penalty();
    let p0 = 0.4 * D_HAT * n;
    let pair = pair_v0_p0();
    let block = c.hessian(&pair, &[p0]).contributions[0].2;

    for j in 0..3 {
        let mut e_j = Vec3::zeros();
        e_j[j] = 1.0;
        let p_pos = [p0 + H * e_j];
        let p_neg = [p0 - H * e_j];
        let g_pos = c.gradient(&pair, &p_pos).contributions[0].1;
        let g_neg = c.gradient(&pair, &p_neg).contributions[0].1;
        let fd_col = (g_pos - g_neg) / (2.0 * H);
        for i in 0..3 {
            assert_relative_eq!(
                fd_col[i],
                block[(i, j)],
                max_relative = MAX_RELATIVE,
                epsilon = EPSILON
            );
        }
    }
}
