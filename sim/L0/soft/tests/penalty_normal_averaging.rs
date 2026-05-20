//! Candidate-E.b normal-averaging unit tests — verify the
//! `with_params_and_smoothing_and_normal_averaging` family + the
//! per-pair normal averaging helper per
//! `docs/CANDIDATE_E_B_NORMAL_AVERAGING_SPEC.md`.
//!
//! Three load-bearing properties pinned here:
//!
//! 1. **Bit-equal-when-disabled** (spec §3.1, §3.4). `normal_avg_k == 1`
//!    short-circuits the helper to `prim.grad(p)` exactly — regardless
//!    of `normal_avg_radius_m`. The full sim-soft regression net is
//!    the primary verification; these tests pin the contract holds at
//!    the new constructor's `k = 1` entry point.
//!
//! 2. **Averaging preserves constant normals** (spec §2.1). On any
//!    SDF whose grad is constant in a neighborhood of `p` (e.g.,
//!    `RigidPlane`), the `k = 7` averaged normal equals the center
//!    grad bit-exactly — the averaging mechanism is a strict
//!    smoothing, not a perturbation in constant-grad regions.
//!
//! 3. **Averaging preserves radial symmetry** (spec §2.1, hyp 1
//!    rationale). On a smooth-curved SDF (`SphereSdf`) queried at an
//!    axis-aligned surface point, the 6-face offsets land at
//!    symmetric positions and the averaged normal still points
//!    outward radially — the helper smooths normal-direction kinks
//!    without changing the well-defined direction on smooth SDFs.
//!
//! Constructor-validation tests verify each assert fires on the
//! pre-imagined caller mistakes (invalid `k`, `r = 0` with `k > 1`,
//! interior cutoff inconsistent with `d_hat + smoothing_eps_m`).

use approx::assert_relative_eq;
use sim_soft::{ContactModel, ContactPair, PenaltyRigidContact, RigidPlane, SphereSdf, Vec3};

/// Fixture-local penalty stiffness — mirrors `PENALTY_KAPPA_DEFAULT`.
const KAPPA: f64 = 1.0e4;
/// Fixture-local contact band — 1 mm.
const D_HAT: f64 = 1.0e-3;
/// Gap-smoothing window — 0.075 mm, the C′.a-pinned value (mirrors
/// the cf-device-design const). Used to confirm normal-averaging
/// composes orthogonally with the C′.a gap smoothing.
const EPS: f64 = 0.075e-3;
/// Normal-averaging offset radius — 0.5 mm (matches the spec §2.3
/// sweep range's lower end).
const R_AVG: f64 = 0.5e-3;

const fn pair_v0_p0() -> ContactPair {
    ContactPair::Vertex {
        vertex_id: 0,
        primitive_id: 0,
    }
}

/// `+z`-plane smoothing contact via the original (no-averaging) ctor.
fn smoothed_z_floor() -> PenaltyRigidContact {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    PenaltyRigidContact::with_params_and_smoothing(vec![plane], KAPPA, D_HAT, EPS)
}

/// `+z`-plane smoothing + averaging contact at the given `(k, r)`.
fn smoothed_avg_z_floor(k: u8, r: f64) -> PenaltyRigidContact {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging(
        vec![plane],
        KAPPA,
        D_HAT,
        EPS,
        k,
        r,
    )
}

/// `SphereSdf` radius 0.05 m centered at origin, smoothing + averaging
/// at the given `(k, r)`.
fn smoothed_avg_sphere(k: u8, r: f64) -> PenaltyRigidContact {
    let sphere = SphereSdf { radius: 0.05 };
    PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging(
        vec![sphere],
        KAPPA,
        D_HAT,
        EPS,
        k,
        r,
    )
}

// ── 1. Bit-equal-when-disabled ─────────────────────────────────────────

#[test]
fn normal_avg_k_one_zero_r_bit_equal_to_smoothing_only() {
    let baseline = smoothed_z_floor();
    let avg_disabled = smoothed_avg_z_floor(1, 0.0);

    // Sweep sd over the active band — same range as
    // penalty_smoothing.rs's bit-equal test.
    let sd_samples = [
        -2.0 * D_HAT,
        -0.5 * D_HAT,
        0.0,
        0.25 * D_HAT,
        0.99 * D_HAT,
        D_HAT,
        0.5_f64.mul_add(EPS, D_HAT),
        D_HAT + EPS,
        2.0_f64.mul_add(EPS, D_HAT),
    ];
    let p = &pair_v0_p0();
    for &sd in &sd_samples {
        let positions = [Vec3::new(0.0, 0.0, sd)];
        // Energy is identical (averaging only affects the normal).
        assert_eq!(
            baseline.energy(p, &positions).to_bits(),
            avg_disabled.energy(p, &positions).to_bits(),
            "energy bit-mismatch at sd = {sd}",
        );
        // Gradient direction equals the plane normal at k=1.
        let g_baseline = baseline.gradient(p, &positions);
        let g_avg = avg_disabled.gradient(p, &positions);
        assert_eq!(
            g_baseline.contributions.len(),
            g_avg.contributions.len(),
            "gradient contribution count at sd = {sd}",
        );
        for ((vid_b, f_b), (vid_a, f_a)) in g_baseline
            .contributions
            .iter()
            .zip(g_avg.contributions.iter())
        {
            assert_eq!(vid_b, vid_a);
            for i in 0..3 {
                assert_eq!(
                    f_b[i].to_bits(),
                    f_a[i].to_bits(),
                    "gradient component {i} at sd = {sd}",
                );
            }
        }
        // Hessian bit-equal — n_avg ⊗ n_avg with n_avg = prim.grad(p)
        // at k=1 is the same rank-1 block.
        let h_baseline = baseline.hessian(p, &positions);
        let h_avg = avg_disabled.hessian(p, &positions);
        assert_eq!(h_baseline.contributions.len(), h_avg.contributions.len());
        for ((r_b, c_b, m_b), (r_a, c_a, m_a)) in h_baseline
            .contributions
            .iter()
            .zip(h_avg.contributions.iter())
        {
            assert_eq!(r_b, r_a);
            assert_eq!(c_b, c_a);
            for i in 0..3 {
                for j in 0..3 {
                    assert_eq!(
                        m_b[(i, j)].to_bits(),
                        m_a[(i, j)].to_bits(),
                        "hessian block ({i}, {j}) at sd = {sd}",
                    );
                }
            }
        }
    }
}

#[test]
fn normal_avg_k_one_nonzero_r_bit_equal_to_smoothing_only() {
    // The helper short-circuits on k == 1 BEFORE reading r — so
    // r > 0 with k = 1 is silently equivalent to r = 0. Pins the
    // short-circuit isn't dependent on the assertion ordering.
    let baseline = smoothed_z_floor();
    let avg_with_r = smoothed_avg_z_floor(1, R_AVG);
    let positions = [Vec3::new(0.0, 0.0, 0.5 * D_HAT)];
    let p = &pair_v0_p0();
    let g_baseline = baseline.gradient(p, &positions);
    let g_avg = avg_with_r.gradient(p, &positions);
    for ((_, f_b), (_, f_a)) in g_baseline
        .contributions
        .iter()
        .zip(g_avg.contributions.iter())
    {
        for i in 0..3 {
            assert_eq!(f_b[i].to_bits(), f_a[i].to_bits());
        }
    }
}

// ── 2. Averaging preserves constant normals ─────────────────────────────

#[test]
fn normal_avg_k_seven_on_plane_preserves_constant_normal() {
    // The plane's grad is +z everywhere; averaging 7 identical vectors
    // and renormalizing returns +z exactly. The k=7 gradient/hessian
    // are bit-equal to the k=1 gradient/hessian on a plane SDF —
    // the averaging mechanism is a strict no-op in constant-grad
    // regions.
    let avg_disabled = smoothed_avg_z_floor(1, 0.0);
    let avg_k7 = smoothed_avg_z_floor(7, R_AVG);
    let positions = [Vec3::new(0.0, 0.0, 0.5 * D_HAT)];
    let p = &pair_v0_p0();

    let g_disabled = avg_disabled.gradient(p, &positions);
    let g_k7 = avg_k7.gradient(p, &positions);
    for ((_, f_d), (_, f_k7)) in g_disabled
        .contributions
        .iter()
        .zip(g_k7.contributions.iter())
    {
        for i in 0..3 {
            assert_eq!(
                f_d[i].to_bits(),
                f_k7[i].to_bits(),
                "gradient component {i} drifts on constant-grad plane",
            );
        }
    }

    let h_disabled = avg_disabled.hessian(p, &positions);
    let h_k7 = avg_k7.hessian(p, &positions);
    for ((_, _, m_d), (_, _, m_k7)) in h_disabled
        .contributions
        .iter()
        .zip(h_k7.contributions.iter())
    {
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(
                    m_d[(i, j)].to_bits(),
                    m_k7[(i, j)].to_bits(),
                    "hessian block ({i}, {j}) drifts on constant-grad plane",
                );
            }
        }
    }
}

// ── 3. Averaging preserves radial symmetry ─────────────────────────────

#[test]
fn normal_avg_k_seven_on_sphere_axis_query_preserves_radial_direction() {
    // Query at (0.05 + 0.0005, 0, 0) — 0.5 mm outside a 50 mm sphere,
    // sd = 0.5 mm < d_hat = 1 mm so the pair is active.
    //
    // The 6 axis-aligned offsets at r = 0.5 mm land at:
    //   (0.0505 ± 0.0005, 0, 0)        → sphere grad still ≈ +x
    //   (0.0505, ±0.0005, 0)           → sphere grad has small ±y
    //                                    component but the ±y pair
    //                                    cancels in the sum
    //   (0.0505, 0, ±0.0005)           → same cancellation in z
    //
    // By the symmetry, the averaged normal direction stays on the
    // +x axis. The magnitude (before renormalization) shrinks slightly
    // due to the inward angle of the off-axis offsets, but renorm
    // returns a unit vector on +x.
    let avg_k7 = smoothed_avg_sphere(7, R_AVG);
    // Sphere radius 0.05; query 0.5 mm outside on +x axis.
    let positions = [Vec3::new(0.05 + 0.5e-3, 0.0, 0.0)];
    let p = &pair_v0_p0();
    let g = avg_k7.gradient(p, &positions);
    assert_eq!(g.contributions.len(), 1);
    let (_, force) = g.contributions[0];
    // The gradient direction is `c.d_energy_d_sd · n_avg`. c.d_energy_d_sd
    // is non-zero (active pair); divide out to recover n_avg.
    // We don't have direct access to the scalar — instead, normalize
    // the force vector and verify it points along +x (or -x — sign
    // depends on the active-band gradient convention).
    let force_norm = force.norm();
    assert!(
        force_norm > 0.0,
        "non-zero force expected at sd = 0.5 mm active pair",
    );
    let force_unit = force / force_norm;
    // By radial symmetry the unit force is along ±x. The y and z
    // components must be near zero (limited by f64 round-off on the
    // sphere grad evaluations).
    assert_relative_eq!(force_unit.y.abs(), 0.0, epsilon = 1e-10);
    assert_relative_eq!(force_unit.z.abs(), 0.0, epsilon = 1e-10);
    assert_relative_eq!(force_unit.x.abs(), 1.0, epsilon = 1e-10);
}

// ── 4. Constructor validation ─────────────────────────────────────────

#[test]
#[should_panic(expected = "normal_avg_k must be 1")]
fn normal_avg_constructor_panics_on_invalid_k() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let _contact = PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging(
        vec![plane],
        KAPPA,
        D_HAT,
        EPS,
        3, // not in {1, 7}
        R_AVG,
    );
}

#[test]
#[should_panic(expected = "normal_avg_radius_m must be strictly positive")]
fn normal_avg_constructor_panics_on_r_zero_when_k_seven() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let _contact = PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging(
        vec![plane],
        KAPPA,
        D_HAT,
        EPS,
        7,
        0.0, // forbidden when k > 1
    );
}

#[test]
#[should_panic(expected = "normal_avg_radius_m must be non-negative")]
fn normal_avg_constructor_panics_on_negative_r() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let _contact = PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging(
        vec![plane],
        KAPPA,
        D_HAT,
        EPS,
        1,
        -1.0e-3,
    );
}

#[test]
#[should_panic(expected = "interior_cutoff")]
fn normal_avg_with_cutoff_constructor_panics_on_too_small_cutoff() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    let _contact =
        PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging_and_interior_cutoff(
            vec![plane],
            KAPPA,
            D_HAT,
            EPS,
            7,
            R_AVG,
            D_HAT, // must be > d_hat + eps; this is just = d_hat
        );
}

// ── 5. with_interior_cutoff variant happy path ────────────────────────

#[test]
fn normal_avg_with_cutoff_constructor_succeeds_at_valid_params() {
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, 1.0), 0.0);
    // cutoff = 5 mm > d_hat + eps = 1.075 mm — valid.
    let _contact =
        PenaltyRigidContact::with_params_and_smoothing_and_normal_averaging_and_interior_cutoff(
            vec![plane],
            KAPPA,
            D_HAT,
            EPS,
            7,
            R_AVG,
            5.0e-3,
        );
    // Smoke test: constructor returned without panic.
}
