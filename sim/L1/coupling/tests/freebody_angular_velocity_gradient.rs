//! Free-body contact-MOMENT gradient gate (Layer-2 keystone, differentiable wrench carry).
//!
//! [`StaggeredCoupling::coupled_trajectory_angular_velocity_gradient`] differentiates a free
//! body's final spin `ω_N` (the angular velocity an off-centre strike drives) w.r.t. the soft
//! block's material parameter — the contact moment made differentiable. It routes the full
//! contact wrench `[τ; f]` through the multi-DOF `RigidStateCarryVjp` carry (the free joint's
//! tangent-space loaded `J_state` + the SO(3) `G_pos`), the orientation-aware successor to the
//! scalar `z_N` path.
//!
//! ## Why ω, not z
//! The free body's z-translation and the flat-plane contact never see its orientation (the
//! mass matrix is block-diagonal for a centred geom; `plane_height` depends only on `xpos.z`),
//! so the moment leaves `z_N`/`peak_fz` bit-identical — `coupled_trajectory_material_gradient`
//! is already exact there, and *cannot* exercise the wrench carry. The final spin `ω_N` is the
//! target that depends on the moment, so it is the one that gates it.
//!
//! Gate: the one-tape gradient matches a central FD of the FULL real coupled rollout (a fresh
//! `step()` re-roll reading `ω_N = qvel[3 + axis]`). Machine-exact (~1e-7 rel, the FD floor)
//! at every rollout length, on an OFF-CENTRE scene where the moment — and hence `ω_N` — is large.

#![allow(
    // A missing/malformed fixture (MJCF load, body index) surfaces as a test panic —
    // the canonical fixture idiom in this workspace's integration tests.
    clippy::expect_used
)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const EDGE: f64 = 0.1;
const MASS: f64 = 0.2;
const HALF_X: f64 = 0.06;
const HALF_Z: f64 = 0.005;
const DT: f64 = 1.0e-3;
const PEN: f64 = 1.0e-4;
const MU0: f64 = 3.0e4;
const AXIS: usize = 1; // ω about body-y (an off-centre +x strike spins about y)

/// An off-centre platen (COM at x = 0.07, block centroid at 0.05) so the contact moment — and
/// thus `ω_N` — is large, pre-penetrated and released from rest. `rigid_damping = 0` (the wrench
/// carry's v1 scope) and the moment routed (the FD oracle must see the spin).
fn build(mu: f64) -> StaggeredCoupling {
    let com_z = EDGE + HALF_Z - PEN;
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="{DT}"/>
  <worldbody>
    <body name="platen" pos="0.07 0.05 {com_z}">
      <freejoint/>
      <geom type="box" size="{HALF_X} {HALF_X} {HALF_Z}" mass="{MASS}"/>
    </body>
  </worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(model, data, 1, HALF_Z, 4, EDGE, mu, DT, 3.0e4, 1.0e-2, 0.0)
        .with_contact_moment(true)
}

/// Final spin `ω_N = qvel[3 + AXIS]` after `n` steps of the REAL coupled (moment-on) dynamics.
fn final_omega(mu: f64, n: usize) -> f64 {
    let mut c = build(mu);
    for _ in 0..n {
        c.step();
    }
    c.data().qvel[3 + AXIS]
}

/// The tape's forward rollout reproduces the real coupled dynamics exactly (it routes the same
/// wrench `step()` does), so its `ω_N` equals the real rollout's.
#[test]
fn angular_velocity_tape_forward_matches_real_rollout() {
    let n = 16;
    let (omega, _grad) = build(MU0).coupled_trajectory_angular_velocity_gradient(n, 0, AXIS);
    let omega_ref = final_omega(MU0, n);
    assert!(
        (omega - omega_ref).abs() < 1e-12,
        "tape forward ω_N {omega} != real rollout {omega_ref}"
    );
}

/// One `tape.backward` over the off-centre coupled rollout matches the full-coupled FD oracle.
/// The block ties `λ = 4μ`, so the constructor-FD measures `d/dμ|_{λ=4μ} = ∂/∂μ + 4·∂/∂λ`.
#[test]
fn angular_velocity_gradient_matches_full_fd() {
    let n = 16;
    let (_omega, grad_mu) = build(MU0).coupled_trajectory_angular_velocity_gradient(n, 0, AXIS);
    let grad_lambda = build(MU0)
        .coupled_trajectory_angular_velocity_gradient(n, 1, AXIS)
        .1;
    let grad_total = grad_mu + 4.0 * grad_lambda;

    let eps = MU0 * 5e-4;
    let fd = (final_omega(MU0 + eps, n) - final_omega(MU0 - eps, n)) / (2.0 * eps);
    let rel = (grad_total - fd).abs() / fd.abs().max(1e-30);
    eprintln!(
        "∂ω_N/∂μ={grad_mu:.6e} ∂ω_N/∂λ={grad_lambda:.6e} total(tape)={grad_total:.6e} FD={fd:.6e} rel={rel:.3e}"
    );
    assert!(
        grad_total.abs() > 1e-9,
        "gradient implausibly ~0 — moment not exercised"
    );
    assert!(
        rel < 1e-6,
        "one-tape dω_N/dμ {grad_total} disagrees with full-coupled FD {fd} (rel {rel:e})"
    );
}

/// Machine-exact at every rollout length — the spin's gradient is well-defined as the off-centre
/// platen reaches its steady spin rate, not only asymptotically.
#[test]
fn angular_velocity_gradient_machine_exact_at_all_lengths() {
    for &n in &[4_usize, 8, 16, 24] {
        let g = build(MU0)
            .coupled_trajectory_angular_velocity_gradient(n, 0, AXIS)
            .1
            + 4.0
                * build(MU0)
                    .coupled_trajectory_angular_velocity_gradient(n, 1, AXIS)
                    .1;
        let eps = MU0 * 5e-4;
        let fd = (final_omega(MU0 + eps, n) - final_omega(MU0 - eps, n)) / (2.0 * eps);
        let rel = (g - fd).abs() / fd.abs().max(1e-30);
        let abs = (g - fd).abs();
        eprintln!("n={n}: rel={rel:.3e} abs={abs:.3e}");
        assert!(
            rel < 1e-6 || abs < 1e-11,
            "dω_N/dμ at n={n} should be machine-exact vs full-coupled FD, got rel {rel:e} abs {abs:e}"
        );
    }
}

const SPHERE_R: f64 = 0.08;

/// The off-centre platen above, but with a finite SPHERE collider (the curved indenter the
/// de-escalation viz uses) — the path the stale-FK lagged-attribution fix is FOR.
fn build_sphere(mu: f64) -> StaggeredCoupling {
    let com_z = EDGE + HALF_Z - PEN;
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="{DT}"/>
  <worldbody>
    <body name="platen" pos="0.07 0.05 {com_z}">
      <freejoint/>
      <geom type="box" size="{HALF_X} {HALF_X} {HALF_Z}" mass="{MASS}"/>
    </body>
  </worldbody>
</mujoco>"#
    );
    let model = load_model(&mjcf).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(model, data, 1, HALF_Z, 4, EDGE, mu, DT, 3.0e4, 1.0e-2, 0.0)
        .with_sphere_collider(SPHERE_R)
        .with_contact_moment(true)
}

fn final_omega_sphere(mu: f64, n: usize) -> f64 {
    let mut c = build_sphere(mu);
    for _ in 0..n {
        c.step();
    }
    c.data().qvel[3 + AXIS]
}

/// **Curved-collider regression gate.** The free-body ω gradient under a finite SPHERE exercises
/// the curved-pose contact-moment term `f_mag·H`, which the stale-FK lagged-attribution fix makes
/// correctly per-step. Without the fix this is ~5.86% off the full-coupled FD; with it, ~1e-5.
///
/// The threshold is `1e-3`, NOT the plane's `1e-6`: the free body's loaded `J_state` is the FD
/// `loaded_state_jacobian`, and the sphere's curved contact carries a small residual moment term
/// beyond the lagged-attribution fix — together a documented ~1e-5 floor (machine-exactness via an
/// analytic free-body `J_state` + the residual curved term is a recorded follow-on). `1e-3` guards
/// against the 5.86%-class regression the fix prevents while tolerating that floor with margin.
#[test]
fn sphere_angular_velocity_gradient_tracks_fd() {
    for &n in &[4_usize, 8, 16] {
        let g = build_sphere(MU0)
            .coupled_trajectory_angular_velocity_gradient(n, 0, AXIS)
            .1
            + 4.0
                * build_sphere(MU0)
                    .coupled_trajectory_angular_velocity_gradient(n, 1, AXIS)
                    .1;
        let eps = MU0 * 5e-4;
        let fd =
            (final_omega_sphere(MU0 + eps, n) - final_omega_sphere(MU0 - eps, n)) / (2.0 * eps);
        let rel = (g - fd).abs() / fd.abs().max(1e-30);
        eprintln!("sphere n={n}: rel={rel:.3e}");
        assert!(
            fd.abs() > 1e-9,
            "degenerate sphere gate at n={n}: FD ≈ 0 — the sphere should spin the body"
        );
        assert!(
            rel < 1e-3,
            "sphere dω_N/dμ at n={n} regressed vs full-coupled FD (the lagged-attribution fix): \
             rel {rel:e} (guard 1e-3, ~1e-5 expected)"
        );
    }
}
