//! Moving-end-effector GRIP CO-DESIGN gradient on a tip-posed sphere. A policy drives a hinge arm
//! carrying a sphere end-effector that rides the arm-tip geom (`with_contact_geom`), so the contact
//! centre moves as the controls swing the arm; sideways gravity turns the press into a tangential
//! grip.
//!
//! The pose-SENSITIVE `tip_z` actuator gradient (the moving-EE centre-channel discriminator) is the
//! `moving-ee·actuator(motor)` row of `coupling_grad_harness.rs`; the moving-EE ACTUATOR-friction
//! and POLICY-friction control gradients are now the `moving-ee·actuator-friction[u]` /
//! `moving-ee·policy-friction[θ]` rows there (folded via `powered_friction_moving_ee`). What remains
//! HERE is the DESIGN+POLICY-friction (#406's gradient, R5's) channel — the μ + θ grip CO-DESIGN
//! gradient, objective `tip_x` (the tangential drag, which is pose-INSENSITIVE, #429: zeroing the
//! pose-centre seam leaves `∂tip_x` machine-exact). It stays bespoke until the grip co-design family
//! is folded into the harness as its own row (the fallible `try_`/`RolloutError` path); tolerance
//! 5e-3 = the curved-friction floor.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

const SPHERE_R: f64 = 0.08;

// A long Y-hinge arm with a motor and a sphere end-effector, under SIDEWAYS gravity so the tip
// sweeps tangentially as the motor/policy drives it (the grip drag the friction gates read).
const FRIC_MJCF: &str = r#"<mujoco>
  <option gravity="1.5 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.344">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.17" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;

const PARAMS: [f64; 3] = [0.05, -0.02, 0.01];

fn build_fric(soft_mu: f64) -> StaggeredCoupling {
    let model = load_model(FRIC_MJCF).expect("friction scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.05;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
    .with_contact_geom(0)
    .with_friction(2.5, 0.1)
}

/// Moving-EE DESIGN+POLICY-friction gradient (#406, R5's gradient) — μ + θ channels vs the FD of
/// the geom-posed `policy_gripped_x`. **This is #406's gradient running on a TIP-posed sphere.**
#[test]
fn design_policy_friction_moving_ee_gradient_matches_fd() {
    let n = 4;
    let mu0 = 3.0e3;
    let (tip_x, mu_grad, theta) = build_fric(mu0)
        .coupled_trajectory_design_policy_friction_gradient(&LinearFeedback, &PARAMS, n);
    let x0 = build_fric(mu0)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
        .x;
    assert!(
        (tip_x - x0).abs() < 1e-10,
        "forward mismatch {tip_x} vs {x0}"
    );
    let eps = 1e-6;
    for k in 0..PARAMS.len() {
        let (mut up, mut dn) = (PARAMS, PARAMS);
        up[k] += eps;
        dn[k] -= eps;
        let fd = (build_fric(mu0)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &up, n)
            .x
            - build_fric(mu0)
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &dn, n)
                .x)
            / (2.0 * eps);
        assert!(fd.abs() > 1e-9, "θ[{k}]: degenerate FD");
        let rel = (theta[k] - fd).abs() / fd.abs().max(1e-9);
        assert!(
            rel < 5e-3,
            "θ[{k}]: design+policy moving-EE {} vs FD {fd} (rel {rel:e})",
            theta[k]
        );
    }
    let eps_mu = mu0 * 1e-4;
    let fd_mu = (build_fric(mu0 + eps_mu)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
        .x
        - build_fric(mu0 - eps_mu)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
            .x)
        / (2.0 * eps_mu);
    assert!(fd_mu.abs() > 1e-9, "μ: degenerate FD ({fd_mu:e})");
    let rel_mu = (mu_grad - fd_mu).abs() / fd_mu.abs().max(1e-9);
    assert!(
        rel_mu < 5e-3,
        "μ: design+policy moving-EE {mu_grad} vs FD {fd_mu} (rel {rel_mu:e})"
    );
}
