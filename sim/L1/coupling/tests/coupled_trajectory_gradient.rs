//! Keystone time-adjoint (PR2) — the coupled multi-step trajectory gradient.
//!
//! `StaggeredCoupling::coupled_trajectory_material_gradient` rolls the coupled
//! soft↔rigid system forward `n_steps` on ONE chassis tape, then a single
//! `tape.backward` gives `∂z_N/∂p` — the platen's final height vs the soft
//! block's Neo-Hookean material — with the reverse pass crossing BOTH step
//! boundaries AND the soft↔rigid interface. It threads the full coupled
//! recurrence: the prev-state / material / contact-pose adjoints fused in
//! `TrajectoryStepVjp` (PR1 + S5 + S3), the velocity readout, the contact-force
//! readout, and the rigid semi-implicit carry.
//!
//! Gate: the one-tape gradient matches a central FD of the FULL real coupled
//! re-rollout (a fresh coupling at μ±ε, the real `step` loop) — an INDEPENDENT
//! oracle (it re-runs the real Newton solves and sim-core steps), in the
//! deeply-engaged, stable-active-set regime the keystone is scoped to.
//!
//! **Scope (penalty R3).** With penalty contact, static force balance settles at
//! the band edge (`sd ≈ d̂`, where the C⁰ force → 0). The C⁰ force keeps `z_N(μ)`
//! smooth (the FD converges across decades of ε), but its derivative kinks at the
//! active-set boundary, so a per-step linearization (active set frozen at μ) is
//! exact only while the contact is firmly engaged (`sd ≪ d̂`). The gradient is
//! validated there to ~1e-3; it degrades as the platen reaches marginal contact
//! or bounces through make/break events — the documented limit IPC would lift.
//! The forward dynamics are validated at any length (the tape replays the real
//! coupled rollout exactly).

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A platen started already in contact (plane at z−clearance = 0.103, the soft
// block's top face at z = 0.1) so a short rollout stays firmly engaged before it
// settles to the marginal band edge — the regime the gradient is scoped to.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e4;
const DAMPING: f64 = 60.0; // settles into contact without a violent rebound

fn build(mu: f64) -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    // body=1, clearance, n_per_edge=4, edge=0.1, mu, dt=1e-3, kappa, d_hat, damping.
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, DAMPING,
    )
}

/// Final platen height after `n_steps` of the REAL coupled dynamics at `mu`.
fn final_z(mu: f64, n_steps: usize) -> f64 {
    let mut c = build(mu);
    let mut z = c.data().xpos[1].z;
    for _ in 0..n_steps {
        z = c.step().rigid_z;
    }
    z
}

/// The tape's forward rollout reproduces the real coupled dynamics exactly (the
/// nodes carry real `step` values; only the backward pass is analytic).
#[test]
fn trajectory_forward_matches_real_rollout() {
    let n = 20;
    let (z_n, _grad) = build(MU0).coupled_trajectory_material_gradient(n, 0);
    let z_ref = final_z(MU0, n);
    assert!(
        (z_n - z_ref).abs() < 1e-12,
        "tape forward z_N {z_n} != real rollout {z_ref}"
    );
}

/// One `tape.backward` over a deeply-engaged coupled rollout matches the
/// full-coupled FD oracle. The block ties `λ = 4μ`, so the constructor-FD (which
/// moves λ = 4μ with μ) measures the total `d/dμ|_{λ=4μ} = ∂/∂μ + 4·∂/∂λ` — the
/// documented S5 linear combination of the two `param_idx` gradients.
#[test]
fn trajectory_gradient_engaged_matches_full_fd() {
    let n = 20;
    let (_z, grad_mu) = build(MU0).coupled_trajectory_material_gradient(n, 0);
    let (_z2, grad_lambda) = build(MU0).coupled_trajectory_material_gradient(n, 1);
    let grad_total = grad_mu + 4.0 * grad_lambda;

    let eps = MU0 * 5e-4;
    let fd = (final_z(MU0 + eps, n) - final_z(MU0 - eps, n)) / (2.0 * eps);
    let rel = (grad_total - fd).abs() / fd.abs().max(1e-30);
    eprintln!(
        "engaged: ∂/∂μ={grad_mu:.6e} ∂/∂λ={grad_lambda:.6e} \
         total(tape)={grad_total:.6e} FD={fd:.6e} rel={rel:.3e}"
    );
    assert!(grad_total.abs() > 1e-9, "gradient implausibly ~0");
    assert!(
        rel < 2e-3,
        "one-tape total dz_N/dμ {grad_total} disagrees with full-coupled FD {fd} (rel {rel:e})"
    );
}

/// The gradient ACCURACY improves as the rollout drives the contact from its
/// initial light touch into firm engagement — direct evidence the per-step
/// factors compose correctly (the residual is the penalty non-smoothness, which
/// shrinks as `sd` moves below `d̂`, not a formula error).
#[test]
fn trajectory_gradient_converges_with_engagement() {
    let fd_rel = |n: usize| {
        let g = build(MU0).coupled_trajectory_material_gradient(n, 0).1
            + 4.0 * build(MU0).coupled_trajectory_material_gradient(n, 1).1;
        let eps = MU0 * 5e-4;
        let fd = (final_z(MU0 + eps, n) - final_z(MU0 - eps, n)) / (2.0 * eps);
        (g - fd).abs() / fd.abs().max(1e-30)
    };
    let early = fd_rel(8); // light touch
    let deep = fd_rel(20); // firmly engaged
    eprintln!("rel: n=8 {early:.3e} → n=20 {deep:.3e}");
    assert!(
        deep < early && deep < 2e-3,
        "deep-engagement gradient ({deep:e}) should be more accurate than the early touch ({early:e})"
    );
}
