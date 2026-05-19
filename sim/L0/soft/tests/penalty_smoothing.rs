//! Candidate-C smoothed-penalty unit tests — verify the
//! `with_params_and_smoothing` family + the quintic-Hermite ramp over
//! `[d̂, d̂+ε]` per `docs/CANDIDATE_C_SMOOTHED_CONTACT_SPEC.md`.
//!
//! These tests target the three load-bearing properties of the
//! smoothing prescription:
//!
//! 1. **Bit-equal-when-dormant** (spec §3). `smoothing_eps_m == 0.0`
//!    reduces to the hard penalty bit-exactly at any `sd`. The full
//!    sim-soft regression net (`contact_unit`, `penalty_*`,
//!    `non_interpenetration`, `hertz_sphere_plane`, etc.) is the
//!    primary verification — these tests pin a small additional
//!    surface that the contract holds at arbitrary `sd`, not just at
//!    the fixture-checked points.
//!
//! 2. **C² energy at the active-pair boundary** (spec §2.2). The
//!    quintic-Hermite polynomial `p(τ) = 1 − 10τ³ + 15τ⁴ − 6τ⁵`
//!    has `(value, 1st_deriv, 2nd_deriv) = (1, 0, 0)` at `τ = 0` and
//!    `(0, 0, 0)` at `τ = 1`. The continuity tests sample the
//!    one-sided limits and assert they match the gate-side value to
//!    machine precision.
//!
//! 3. **Hessian continuity across pair flip** (spec §7 open-question
//!    structural argument). The class-2 chattering bookmarked at
//!    `docs/CAVITY_5MM_CHATTERING_BOOKMARK.md` traces back to a finite
//!    jump in the per-pair Hessian when a pair crosses `sd = d̂`;
//!    quintic-Hermite smoothing converts that discrete jump into a
//!    continuous-by-composition mechanism (per-pair `H_pair(sd)` is
//!    C⁰; sums of C⁰ are C⁰). The cross-pair-flip Hessian test pins
//!    the per-pair half of that argument; the multi-pair compositional
//!    sum follows by additivity of the assembled `H_contact(x)`.

use approx::assert_relative_eq;
use sim_soft::{
    ActivePairsFor, ContactModel, ContactPair, MaterialField, PenaltyRigidContact, RigidPlane,
    SingleTetMesh, Vec3,
};

/// Fixture-local penalty stiffness — mirrors `PENALTY_KAPPA_DEFAULT`.
const KAPPA: f64 = 1.0e4;
/// Fixture-local contact band — 1 mm, same as `PENALTY_DHAT_DEFAULT`.
const D_HAT: f64 = 1.0e-3;
/// Smoothing window — 0.5 mm. Inside the
/// [0.1 mm, 2 mm] sweep envelope spec §5 mandates for the C.2
/// calibration; a fixture-local choice that produces clean numbers.
const EPS: f64 = 0.5e-3;

/// A `+z`-normal plane through the origin with smoothing enabled —
/// vertex at `(_, _, z)` has `sd = z`, so the test fixture controls
/// `sd` directly through the vertex's z-coordinate.
fn penalty_z_floor_smoothed(eps: f64) -> PenaltyRigidContact {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    PenaltyRigidContact::with_params_and_smoothing(vec![plane], KAPPA, D_HAT, eps)
}

const fn pair_v0_p0() -> ContactPair {
    ContactPair::Vertex {
        vertex_id: 0,
        primitive_id: 0,
    }
}

/// Bit-equal-when-dormant contract verification at arbitrary `sd`.
/// The fixture-pinned regression tests (`contact_unit`,
/// `penalty_compressive_block`, etc.) cover specific known points;
/// this test sweeps a range to catch any `ε = 0` code-path bug that
/// might slip past the fixture-pinned points.
#[test]
fn penalty_smoothing_zero_eps_is_bit_equal_to_hard_penalty() {
    let plane_hard = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let c_hard = PenaltyRigidContact::with_params(vec![plane_hard], KAPPA, D_HAT);
    let c_smooth_zero = penalty_z_floor_smoothed(0.0);

    // Sweep sd over a representative range:
    //   - Inside the active band (sd < d̂)
    //   - At the gate exactly (sd = d̂)
    //   - Past the gate (sd > d̂)
    let sd_samples = [
        -2.0 * D_HAT,
        -0.5 * D_HAT,
        0.0,
        0.25 * D_HAT,
        0.99 * D_HAT,
        D_HAT,
        1.01 * D_HAT,
        2.0 * D_HAT,
    ];
    for &sd in &sd_samples {
        let positions = [Vec3::new(0.0, 0.0, sd)];
        let p = &pair_v0_p0();
        assert_eq!(
            c_hard.energy(p, &positions).to_bits(),
            c_smooth_zero.energy(p, &positions).to_bits(),
            "energy bit-mismatch at sd = {sd}",
        );
        let g_hard = c_hard.gradient(p, &positions);
        let g_smooth = c_smooth_zero.gradient(p, &positions);
        assert_eq!(
            g_hard.contributions.len(),
            g_smooth.contributions.len(),
            "gradient contribution count mismatch at sd = {sd}",
        );
        for (h, s) in g_hard
            .contributions
            .iter()
            .zip(g_smooth.contributions.iter())
        {
            assert_eq!(h.0, s.0, "gradient vertex_id mismatch at sd = {sd}");
            for axis in 0..3 {
                assert_eq!(
                    h.1[axis].to_bits(),
                    s.1[axis].to_bits(),
                    "gradient axis {axis} bit-mismatch at sd = {sd}",
                );
            }
        }
        let h_hard = c_hard.hessian(p, &positions);
        let h_smooth = c_smooth_zero.hessian(p, &positions);
        assert_eq!(
            h_hard.contributions.len(),
            h_smooth.contributions.len(),
            "hessian contribution count mismatch at sd = {sd}",
        );
        for (h, s) in h_hard
            .contributions
            .iter()
            .zip(h_smooth.contributions.iter())
        {
            assert_eq!(h.0, s.0, "hessian row-vertex mismatch at sd = {sd}");
            assert_eq!(h.1, s.1, "hessian col-vertex mismatch at sd = {sd}");
            for row in 0..3 {
                for col in 0..3 {
                    assert_eq!(
                        h.2[(row, col)].to_bits(),
                        s.2[(row, col)].to_bits(),
                        "hessian block ({row},{col}) bit-mismatch at sd = {sd}",
                    );
                }
            }
        }
    }
}

/// Energy continuous at `sd = d̂`: the one-sided limit from the active
/// side matches the one-sided limit from the transition side.
#[test]
fn penalty_smoothing_energy_continuous_at_d_hat() {
    let c = penalty_z_floor_smoothed(EPS);
    let p = &pair_v0_p0();
    // Sample close on both sides of the boundary. With quintic Hermite
    // the energy is C² at sd = d̂, so the one-sided values differ by
    // O(delta²) (the gap²·ramp expansion's lowest-order contribution).
    let delta = 1.0e-9;
    let e_below = c.energy(p, &[Vec3::new(0.0, 0.0, D_HAT - delta)]);
    let e_above = c.energy(p, &[Vec3::new(0.0, 0.0, D_HAT + delta)]);
    // Both should be ≈ 0.5·κ·delta² (with the smoothed energy being
    // bit-equal to the hard energy at the band boundary because both
    // gap = ±delta and ramp = 1 at sd = d̂⁺).
    let expected = 0.5 * KAPPA * delta * delta;
    assert_relative_eq!(e_below, expected, max_relative = 1.0e-10);
    assert_relative_eq!(e_above, expected, max_relative = 1.0e-3);
    // Same-magnitude continuity check across the boundary.
    assert_relative_eq!(e_below, e_above, max_relative = 1.0e-3);
}

/// Gradient C⁰ at `sd = d̂`: the gradient is zero at the boundary
/// itself, and both one-sided limits go to zero as `δ → 0`. With a
/// continuous nonzero Hessian `κ` at the boundary, the one-sided
/// gradient values at `sd = d̂ ± δ` evaluate to roughly `±κδ`
/// (the Taylor first-order term — `E'(d̂) = 0`, `E''(d̂) = κ`), so the
/// `||above − below|| ≈ 2κδ` Lipschitz scaling at finite `δ` is the
/// expected continuous-function behavior, not a discontinuity.
#[test]
fn penalty_smoothing_gradient_continuous_at_d_hat() {
    let c = penalty_z_floor_smoothed(EPS);
    let p = &pair_v0_p0();
    // Gradient at sd = d̂ exactly is zero by the active-side formula
    // (gap = 0 → d_energy_d_sd = -κ·gap = 0). Pair is included (gate
    // is sd < d̂ + ε, true at sd = d̂ when ε > 0).
    let g_at = c.gradient(p, &[Vec3::new(0.0, 0.0, D_HAT)]);
    assert_eq!(g_at.contributions.len(), 1);
    let f_at = g_at.contributions[0].1;
    assert_eq!(
        f_at.norm().to_bits(),
        0.0_f64.to_bits(),
        "gradient at sd = d̂ exactly must be bit-zero, got ||f|| = {}",
        f_at.norm(),
    );

    // Linear scaling check: ||f(d̂ ± δ)|| ≈ κ·δ at small δ (the Lipschitz
    // scale set by E''(d̂) = κ). Test that the magnitude is in the
    // expected band [0.5·κδ, 2·κδ] — confirms gradient is bounded
    // linearly, not jumping discontinuously.
    let delta = 1.0e-9;
    let g_below = c.gradient(p, &[Vec3::new(0.0, 0.0, D_HAT - delta)]);
    let g_above = c.gradient(p, &[Vec3::new(0.0, 0.0, D_HAT + delta)]);
    let f_below = g_below.contributions[0].1;
    let f_above = g_above.contributions[0].1;
    let expected_scale = KAPPA * delta;
    assert!(
        f_below.norm() > 0.5 * expected_scale && f_below.norm() < 2.0 * expected_scale,
        "gradient below band-boundary out of Lipschitz envelope: ||f|| = {}, expected ≈ κ·δ = {}",
        f_below.norm(),
        expected_scale,
    );
    assert!(
        f_above.norm() > 0.5 * expected_scale && f_above.norm() < 2.0 * expected_scale,
        "gradient above band-boundary out of Lipschitz envelope: ||f|| = {}, expected ≈ κ·δ = {}",
        f_above.norm(),
        expected_scale,
    );
}

/// Hessian continuous at `sd = d̂` — the class-2 chattering fix. Hard
/// penalty would show a `κ`-magnitude jump here (the limit-from-left
/// is `κ·n⊗n`, the gate-returned value at `sd ≥ d̂` is 0); quintic
/// smoothing collapses that to zero.
#[test]
fn penalty_smoothing_hessian_continuous_at_d_hat() {
    let c = penalty_z_floor_smoothed(EPS);
    let p = &pair_v0_p0();
    let delta = 1.0e-9;
    let h_below = c.hessian(p, &[Vec3::new(0.0, 0.0, D_HAT - delta)]);
    let h_above = c.hessian(p, &[Vec3::new(0.0, 0.0, D_HAT + delta)]);
    assert_eq!(h_below.contributions.len(), 1);
    assert_eq!(h_above.contributions.len(), 1);
    let block_below = h_below.contributions[0].2;
    let block_above = h_above.contributions[0].2;
    // For quintic Hermite the Hessian is C⁰ at sd = d̂. The two
    // one-sided limits should agree at the κ·n⊗n level (modulo
    // O(δ³/ε³) higher-order terms from the quintic expansion).
    let diff_frobenius = (block_above - block_below).norm();
    assert!(
        diff_frobenius < 1.0e-6 * KAPPA,
        "hessian not continuous at sd = d̂: ||above - below||_F = {diff_frobenius}, \
         expected ≪ {} (= 1e-6·κ)",
        1.0e-6 * KAPPA,
    );
    // For contrast: the hard penalty would jump by κ·n⊗n here. The
    // smoothed jump should be many orders of magnitude smaller.
    assert!(
        diff_frobenius < 1.0e-3 * KAPPA,
        "hessian jump much larger than the expected κ·n⊗n active→smoothed gap",
    );
}

/// Energy, gradient, and Hessian all reach zero at the upper edge of
/// the smoothing window `sd = d̂ + ε`. Tested at exactly `sd = d̂ + ε`
/// — `pair_is_active` returns false (`sd < d̂ + ε` is strict), so
/// energy / gradient / Hessian all return bit-zero defaults. Also
/// tested at `sd > d̂ + ε` for the inactive half.
#[test]
fn penalty_smoothing_energy_gradient_hessian_zero_at_band_top() {
    let c = penalty_z_floor_smoothed(EPS);
    let p = &pair_v0_p0();

    // At the upper edge exactly: gate's `sd < d̂ + ε` is strict, so
    // pair_is_active returns false, and the three trait methods return
    // their zero defaults bit-exactly.
    let positions_edge = [Vec3::new(0.0, 0.0, D_HAT + EPS)];
    assert_eq!(
        c.energy(p, &positions_edge).to_bits(),
        0.0_f64.to_bits(),
        "energy at sd = d̂ + ε exactly must be bit-zero",
    );
    assert!(
        c.gradient(p, &positions_edge).contributions.is_empty(),
        "gradient at sd = d̂ + ε exactly must have no contributions",
    );
    assert!(
        c.hessian(p, &positions_edge).contributions.is_empty(),
        "hessian at sd = d̂ + ε exactly must have no contributions",
    );

    // Past the upper edge: same — gate excludes the pair.
    let positions_above = [Vec3::new(0.0, 0.0, 2.0_f64.mul_add(EPS, D_HAT))];
    assert_eq!(c.energy(p, &positions_above).to_bits(), 0.0_f64.to_bits());
    assert!(c.gradient(p, &positions_above).contributions.is_empty());
    assert!(c.hessian(p, &positions_above).contributions.is_empty());

    // Verify the approach is C⁰ from below: at sd = d̂+ε with the limit
    // strictly approached (very small δ that resolves the quintic's
    // cubic-cuspy convergence rate, R''(τ) ∝ (1−τ) → linear approach
    // in δ for the gap²·R''(sd) term, which dominates near the upper
    // edge). δ = 1e-15 makes all terms truly negligible (κ·δ/ε ≈ 1e-8).
    let delta = 1.0e-15;
    let positions_near = [Vec3::new(0.0, 0.0, D_HAT + EPS - delta)];
    let h_near = c.hessian(p, &positions_near);
    let h_near_norm = h_near
        .contributions
        .first()
        .map_or(0.0, |(_, _, block)| block.norm());
    assert!(
        h_near_norm < 1.0e-5 * KAPPA,
        "hessian at sd = (d̂ + ε)⁻ should approach 0: got {h_near_norm}",
    );
}

/// Quintic Hermite ramp sanity — value at `τ = 0` is 1, at `τ = 0.5`
/// is in `(0, 1)`, and at `τ = 1` is 0. Recovered from the energy as
/// `R(sd) = E(sd) / (0.5·κ·(d̂−sd)²)`; the formula is exact at the
/// transition-band evaluation points.
#[test]
fn penalty_smoothing_quintic_ramp_polynomial_sanity() {
    let c = penalty_z_floor_smoothed(EPS);
    let p = &pair_v0_p0();
    // Recover R(sd) from energy: E = 0.5κ·(d̂-sd)²·R → R = 2E/(κ·(d̂-sd)²).
    let recover = |sd: f64| -> f64 {
        let positions = [Vec3::new(0.0, 0.0, sd)];
        let e = c.energy(p, &positions);
        let gap = D_HAT - sd;
        2.0 * e / (KAPPA * gap * gap)
    };
    // Near τ = 0⁺ (just inside the transition band) — ramp ≈ 1.
    let r_near_zero = recover(0.001_f64.mul_add(EPS, D_HAT));
    assert_relative_eq!(r_near_zero, 1.0, max_relative = 1.0e-3);

    // At τ = 0.5 — ramp = 1 - 10·(1/8) + 15·(1/16) - 6·(1/32)
    //                  = 1 - 1.25 + 0.9375 - 0.1875 = 0.5.
    let r_mid = recover(0.5_f64.mul_add(EPS, D_HAT));
    assert_relative_eq!(r_mid, 0.5, max_relative = 1.0e-12);

    // At τ = 0.25 — ramp = 1 - 10·(1/64) + 15·(1/256) - 6·(1/1024)
    //                    = 1 - 0.15625 + 0.05859375 - 0.005859375
    //                    = 0.896484375.
    let r_quarter = recover(0.25_f64.mul_add(EPS, D_HAT));
    assert_relative_eq!(r_quarter, 0.896_484_375, max_relative = 1.0e-12);

    // Near τ = 1⁻ — ramp ≈ 0 (with O((1-τ)³) tail).
    let r_near_one = recover(0.999_f64.mul_add(EPS, D_HAT));
    assert!(
        r_near_one.abs() < 1.0e-7,
        "ramp at τ ≈ 1⁻ should be near zero, got {r_near_one}",
    );
}

/// Ramp monotonically decreasing in the transition band per spec §6
/// C.1 ladder. `p'(τ) = -30τ²(1-τ)²` is strictly negative on `(0, 1)`,
/// so the ramp is strictly monotonically decreasing on `(d̂, d̂+ε)`.
#[test]
fn penalty_smoothing_ramp_monotonically_decreasing_in_band() {
    let c = penalty_z_floor_smoothed(EPS);
    let p = &pair_v0_p0();
    let recover = |sd: f64| -> f64 {
        let positions = [Vec3::new(0.0, 0.0, sd)];
        let e = c.energy(p, &positions);
        let gap = D_HAT - sd;
        2.0 * e / (KAPPA * gap * gap)
    };
    // Sweep τ at 100 uniform points strictly inside (0, 1).
    let n_samples: i32 = 100;
    let mut prev = f64::INFINITY;
    for i in 1..n_samples {
        let tau = f64::from(i) / f64::from(n_samples);
        let sd = tau.mul_add(EPS, D_HAT);
        let r = recover(sd);
        assert!(
            r < prev,
            "ramp not strictly decreasing at τ = {tau}: ramp = {r}, prev = {prev}",
        );
        prev = r;
    }
}

/// Per-pair gradient Lipschitz across the active boundary `sd = d̂`.
/// The gradient is continuous in both the hard-penalty and smoothed
/// cases (both branches' formulas give zero at exactly `sd = d̂`); the
/// load-bearing chattering fix lives at the *Hessian* level (see
/// [`penalty_smoothing_hessian_continuous_across_pair_flip`]). This
/// test pins that the gradient stays Lipschitz with constant ≈ κ
/// across the boundary — i.e., no discontinuity in the gradient is
/// introduced by the smoothing prescription.
///
/// Concretely: gradient at `sd = d̂ − δ` is `−κδ` (active-side Taylor);
/// gradient at `sd = d̂ + δ` is `+κδ` (transition-side Taylor, with
/// the quintic-Hermite ramp's `(1, 0, 0)` boundary giving the same
/// leading-order linear-in-δ term). Both go to zero as `δ → 0`, but
/// their slopes have opposite signs (the energy is locally `0.5κ(sd−d̂)²`
/// on the right too — a local minimum at `sd = d̂` with value zero).
/// `||right − left||` is therefore `≈ 2κδ`, the Lipschitz bound.
#[test]
fn penalty_smoothing_gradient_continuous_across_pair_flip() {
    let c = penalty_z_floor_smoothed(EPS);
    let p = &pair_v0_p0();
    let delta = 1.0e-9;
    let g_left = c.gradient(p, &[Vec3::new(0.0, 0.0, D_HAT - delta)]);
    let g_right = c.gradient(p, &[Vec3::new(0.0, 0.0, D_HAT + delta)]);
    let f_left = g_left.contributions[0].1;
    let f_right = g_right.contributions[0].1;
    let diff = (f_right - f_left).norm();
    // Lipschitz envelope: diff in [κδ, 4κδ] — 2κδ is the expected
    // Taylor value; the envelope catches both the C⁰ contract (no
    // unbounded jump) and discriminates against the hard-penalty
    // alternative-formulation where the right side would be 0 (giving
    // diff = κδ).
    let lipschitz_scale = KAPPA * delta;
    assert!(
        diff > 1.5 * lipschitz_scale && diff < 4.0 * lipschitz_scale,
        "gradient out of Lipschitz envelope across pair flip: \
         diff = {diff}, expected ≈ 2·κ·δ = {}",
        2.0 * lipschitz_scale,
    );
}

/// Per-pair Hessian continuous across the active boundary `sd = d̂` —
/// the load-bearing test for the C.0 spec's chattering fix. The
/// hard-penalty Hessian jumps by `κ·n⊗n` here; quintic smoothing
/// collapses that to ≈ 0 (modulo O((δ/ε)³)·κ higher-order terms).
#[test]
fn penalty_smoothing_hessian_continuous_across_pair_flip() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let c_smooth = PenaltyRigidContact::with_params_and_smoothing(vec![plane], KAPPA, D_HAT, EPS);
    let plane_hard = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let c_hard = PenaltyRigidContact::with_params(vec![plane_hard], KAPPA, D_HAT);
    let p = &pair_v0_p0();

    let delta = 1.0e-9;
    let positions_left = [Vec3::new(0.0, 0.0, D_HAT - delta)];
    let positions_right = [Vec3::new(0.0, 0.0, D_HAT + delta)];

    // Smoothed: Hessian C⁰ at sd = d̂. ||right - left|| ≈ O((δ/ε)³)·κ.
    let h_smooth_left = c_smooth.hessian(p, &positions_left);
    let h_smooth_right = c_smooth.hessian(p, &positions_right);
    let block_left = h_smooth_left.contributions[0].2;
    let block_right = h_smooth_right.contributions[0].2;
    let smooth_diff = (block_right - block_left).norm();

    // Hard penalty: Hessian discontinuous at sd = d̂. Left = κ·n⊗n,
    // right = 0 (the gate returns no contribution). ||right - left||
    // is κ (with n along +z, ||n⊗n||_F = 1).
    let h_hard_left = c_hard.hessian(p, &positions_left);
    let h_hard_right = c_hard.hessian(p, &positions_right);
    let hard_left = h_hard_left.contributions[0].2;
    assert!(
        h_hard_right.contributions.is_empty(),
        "hard penalty should have zero hessian contribution at sd > d̂",
    );
    let hard_diff = hard_left.norm(); // ||right - left|| with right = 0.
    assert_relative_eq!(hard_diff, KAPPA, max_relative = 1.0e-12);

    // The smoothed jump should be many orders of magnitude smaller
    // than the hard-penalty jump — this is the chattering fix.
    assert!(
        smooth_diff < 1.0e-9 * hard_diff,
        "quintic smoothing failed to suppress hessian jump: smooth_diff = {smooth_diff}, \
         hard_diff = {hard_diff}, ratio = {}",
        smooth_diff / hard_diff,
    );
}

/// `active_pairs` includes the transition band when smoothing is on —
/// gate at `sd < d̂ + smoothing_eps_m`, not the hard `sd < d̂`.
#[test]
fn penalty_smoothing_active_pairs_includes_transition_band() {
    let c = penalty_z_floor_smoothed(EPS);
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    // Place one vertex inside the transition band — sd = d̂ + ε/2.
    let positions = [Vec3::new(0.0, 0.0, 0.5_f64.mul_add(EPS, D_HAT))];
    let pairs = c.active_pairs(&mesh, &positions);
    assert_eq!(
        pairs.len(),
        1,
        "smoothing-on `active_pairs` must include transition-band pairs (sd ∈ (d̂, d̂+ε))",
    );
}

/// `active_pairs` excludes pairs above the smoothing window
/// (`sd ≥ d̂ + ε`) — the upper edge of the active band with smoothing
/// on.
#[test]
fn penalty_smoothing_active_pairs_excludes_above_band() {
    let c = penalty_z_floor_smoothed(EPS);
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    // Place one vertex above the transition band — sd = d̂ + 2ε.
    let positions = [Vec3::new(0.0, 0.0, 2.0_f64.mul_add(EPS, D_HAT))];
    let pairs = c.active_pairs(&mesh, &positions);
    assert!(
        pairs.is_empty(),
        "smoothing-on `active_pairs` must exclude pairs above d̂ + smoothing_eps_m, got {} pairs",
        pairs.len(),
    );
}

/// Constructor rejects negative `smoothing_eps_m` — defensive surface
/// for the C.2 calibration sweep + a future tuning UI.
#[test]
#[should_panic(expected = "smoothing_eps_m must be non-negative and finite")]
fn penalty_smoothing_constructor_rejects_negative_eps() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    drop(PenaltyRigidContact::with_params_and_smoothing(
        vec![plane],
        KAPPA,
        D_HAT,
        -1.0e-3,
    ));
}

/// Constructor rejects non-finite `smoothing_eps_m`.
#[test]
#[should_panic(expected = "smoothing_eps_m must be non-negative and finite")]
fn penalty_smoothing_constructor_rejects_nan_eps() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    drop(PenaltyRigidContact::with_params_and_smoothing(
        vec![plane],
        KAPPA,
        D_HAT,
        f64::NAN,
    ));
}

/// `with_params_and_smoothing_and_interior_cutoff` composes the two
/// constructors orthogonally — interior cutoff filters deep-interior
/// pairs, smoothing applies in the transition band.
#[test]
fn penalty_smoothing_and_interior_cutoff_compose_orthogonally() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let interior_cutoff = 5.0e-3;
    let c = PenaltyRigidContact::with_params_and_smoothing_and_interior_cutoff(
        vec![plane],
        KAPPA,
        D_HAT,
        EPS,
        interior_cutoff,
    );
    let mesh = SingleTetMesh::new(&MaterialField::skeleton_default());
    // Three vertices probe the gate:
    //   - sd = -10 mm: deep-interior, excluded by cutoff.
    //   - sd = -0.5 mm: in active band (sd < d̂), included.
    //   - sd = d̂ + ε/2 = 1.25 mm: transition band, included.
    //   - sd = d̂ + 2ε = 2 mm: above smoothing window, excluded.
    let positions = [
        Vec3::new(0.0, 0.0, -10.0e-3),
        Vec3::new(0.0, 0.0, -0.5e-3),
        Vec3::new(0.0, 0.0, 0.5_f64.mul_add(EPS, D_HAT)),
        Vec3::new(0.0, 0.0, 2.0_f64.mul_add(EPS, D_HAT)),
    ];
    let pairs = c.active_pairs(&mesh, &positions);
    assert_eq!(
        pairs.len(),
        2,
        "expected 2 active pairs, got {}",
        pairs.len()
    );
    // Verify the two included vertices are indices 1 and 2.
    let vids: Vec<_> = pairs
        .iter()
        .map(|p| match p {
            ContactPair::Vertex { vertex_id, .. } => *vertex_id,
        })
        .collect();
    assert_eq!(vids, vec![1, 2], "wrong vertex set included");
}

/// `with_params_and_smoothing_and_interior_cutoff` rejects an interior
/// cutoff that doesn't cover the smoothed active band.
#[test]
#[should_panic(expected = "must be > d_hat + smoothing_eps_m")]
fn penalty_smoothing_and_interior_cutoff_rejects_cutoff_inside_band() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    // d_hat = 1 mm + smoothing = 0.5 mm → upper band = 1.5 mm.
    // Cutoff at 1.25 mm is inside the band, should panic.
    drop(
        PenaltyRigidContact::with_params_and_smoothing_and_interior_cutoff(
            vec![plane],
            KAPPA,
            D_HAT,
            EPS,
            1.25e-3,
        ),
    );
}
