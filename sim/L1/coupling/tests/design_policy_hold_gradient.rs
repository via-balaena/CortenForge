//! The trajectory-integrated HOLDING gradient matches a finite difference of the forward holding
//! cost — the first sustained-behavior co-design lever (H1 of the R5 grip-viewer arc).
//!
//! Where `coupled_trajectory_design_policy_friction_gradient` differentiates the TERMINAL `tip_x`,
//! `coupled_trajectory_design_policy_hold_gradient` differentiates a cost summed over the whole
//! rollout — `L = Σₖ (qₖ − q_hold)²` (the gripped limb's hinge angle vs a held setpoint). Both share
//! one per-step coupled tape (`build_design_policy_tape`, the #406 machinery); only the seeding
//! differs. The forward oracle reads the per-step `qpos0` from
//! `coupled_trajectory_policy_gripped_capture` (an independent forward), so the FD never touches the
//! gradient's tape.
//!
//! Tolerance is the friction-gradient's (rel ~5e-3) — the underlying tape carries the #422 frozen-lag
//! friction residual. Gated on BOTH the centroid sphere and the moving-EE (tip-posed) grip.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

const SPHERE_R: f64 = 0.08;
const PARAMS: [f64; 3] = [0.05, -0.02, 0.01];
const Q_HOLD: f64 = 0.25;

// Centroid friction grip (gravity 2.0; sphere posed over the block centroid).
fn centroid_scene() -> &'static str {
    r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#
}

// Moving-EE friction grip (gravity 1.5; sphere tracks the arm-tip geom — the gated R5 scene).
fn moving_ee_scene() -> &'static str {
    r#"<mujoco>
  <option gravity="1.5 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.344">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.17" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#
}

fn build(scene: &str, qpos0: f64, soft_mu: f64, moving_ee: bool) -> StaggeredCoupling {
    let model = load_model(scene).expect("scene loads");
    let mut data = model.make_data();
    data.qpos[0] = qpos0;
    data.forward(&model).expect("forward");
    let c = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R);
    let c = if moving_ee { c.with_contact_geom(0) } else { c };
    c.with_friction(2.5, 0.1)
}

/// Forward holding cost `Σ_{k=0}^{N-1} (qₖ − q_hold)²` — an INDEPENDENT forward (the capture is
/// byte-identical to `coupled_trajectory_policy_gripped_x`, not the gradient's tape). The first N
/// frames carry the hinge angle at the start of steps 0…N−1 — exactly the gradient's `qpos_steps`.
fn hold_cost(mut c: StaggeredCoupling, q_hold: f64, n: usize) -> f64 {
    let frames = c
        .coupled_trajectory_policy_gripped_capture(&LinearFeedback, &PARAMS, n)
        .1;
    (0..n)
        .map(|k| (frames[k].qpos0 - q_hold) * (frames[k].qpos0 - q_hold))
        .sum()
}

fn check(scene: &str, qpos0: f64, moving_ee: bool, n: usize, label: &str) {
    let mu0 = 3.0e3;
    let (cost, mu_grad, theta) = build(scene, qpos0, mu0, moving_ee)
        .coupled_trajectory_design_policy_hold_gradient(&LinearFeedback, &PARAMS, n, Q_HOLD);

    // The gradient's reported cost equals the independent forward cost (same rollout physics).
    let cost0 = hold_cost(build(scene, qpos0, mu0, moving_ee), Q_HOLD, n);
    assert!(
        (cost - cost0).abs() < 1e-9,
        "{label}: gradient cost {cost} vs forward {cost0}"
    );
    assert!(cost0 > 1e-6, "{label}: degenerate cost — no holding signal");

    // θ channel.
    let eps = 1e-6;
    for k in 0..PARAMS.len() {
        let (mut up, mut dn) = (PARAMS, PARAMS);
        up[k] += eps;
        dn[k] -= eps;
        let lp = {
            let mut c = build(scene, qpos0, mu0, moving_ee);
            let frames = c
                .coupled_trajectory_policy_gripped_capture(&LinearFeedback, &up, n)
                .1;
            (0..n)
                .map(|j| (frames[j].qpos0 - Q_HOLD).powi(2))
                .sum::<f64>()
        };
        let ln = {
            let mut c = build(scene, qpos0, mu0, moving_ee);
            let frames = c
                .coupled_trajectory_policy_gripped_capture(&LinearFeedback, &dn, n)
                .1;
            (0..n)
                .map(|j| (frames[j].qpos0 - Q_HOLD).powi(2))
                .sum::<f64>()
        };
        let fd = (lp - ln) / (2.0 * eps);
        assert!(fd.abs() > 1e-9, "{label} θ[{k}]: degenerate FD ({fd:e})");
        let rel = (theta[k] - fd).abs() / fd.abs().max(1e-9);
        assert!(
            rel < 5e-3,
            "{label} θ[{k}]: hold gradient {} vs FD {fd} (rel {rel:e})",
            theta[k]
        );
    }

    // μ (design) channel — the λ = 4μ tie moves with the build perturbation.
    let eps_mu = mu0 * 1e-4;
    let fd_mu = (hold_cost(build(scene, qpos0, mu0 + eps_mu, moving_ee), Q_HOLD, n)
        - hold_cost(build(scene, qpos0, mu0 - eps_mu, moving_ee), Q_HOLD, n))
        / (2.0 * eps_mu);
    assert!(fd_mu.abs() > 1e-9, "{label} μ: degenerate FD ({fd_mu:e})");
    let rel_mu = (mu_grad - fd_mu).abs() / fd_mu.abs().max(1e-9);
    assert!(
        rel_mu < 5e-3,
        "{label} μ: hold gradient {mu_grad} vs FD {fd_mu} (rel {rel_mu:e})"
    );
}

/// Holding gradient (μ + θ) vs FD on the CENTROID grip (short horizon).
#[test]
fn hold_gradient_centroid_matches_fd() {
    check(centroid_scene(), 0.3, false, 5, "centroid n5");
}

/// Holding gradient on the CENTROID grip at a LONGER horizon — exercises the trajectory
/// *accumulation* (the novelty: the cost sums 20 per-step terms through the carry recurrence,
/// where the #422 frozen-lag friction residual compounds).
#[test]
fn hold_gradient_centroid_n20_matches_fd() {
    check(centroid_scene(), 0.3, false, 20, "centroid n20");
}

/// Holding gradient (μ + θ) vs FD on the MOVING-EE (tip-posed) grip — the R5 scene.
#[test]
fn hold_gradient_moving_ee_matches_fd() {
    check(moving_ee_scene(), 0.05, true, 5, "moving-EE n5");
}
