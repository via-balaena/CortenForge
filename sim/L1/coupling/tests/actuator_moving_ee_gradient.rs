//! Moving-end-effector ACTUATOR control gradient on a tip-posed sphere — the actuator/policy
//! successor to #428 (moving-EE material-normal). A motor drives the hinge arm; the contact sphere
//! rides the arm-tip geom (`with_contact_geom`), so the contact centre moves as the controls swing
//! the arm. `∂tip_z/∂u_k` from one tape vs a central FD of the geom-posed actuated rollout.
//!
//! Why it composes: the moving-EE pose channel (PoseCentreVjp + WrenchPose::Centre + the
//! twist-translation soft node) is the SAME machinery #427/#428 validated single-step; the actuator
//! `g_act` channel is contact-independent (Δt·M⁻¹·∂qfrc_actuator/∂ctrl) and orthogonal to the pose
//! channel. So this gate is the composition check (g_act + moving-EE centre on one tape). The tip_z
//! objective is POSE-SENSITIVE (unlike the friction tip_x drag, #429), so this gate DOES discriminate
//! the centre channel — a height-only adjoint disagrees with the geom-posed FD.
//!
//! n = 1 is the single-step leaf (isolate the moving-EE + g_act composition before the multi-step
//! rollout); n = 2, 6 are the multi-step composition.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

// A long Y-hinge arm with a motor, sphere end-effector pressing the block top from directly above
// (body at xy = block centroid; the link reaches down so the south pole indents ~4 mm). The motor
// control swings the arm → the contact centre translates as the tip arcs.
const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0.05 0.05 0.344">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.17" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor name="a" joint="j" gear="1"/></actuator>
</mujoco>"#;

const SPHERE_R: f64 = 0.08;

fn build() -> StaggeredCoupling {
    let model = load_model(MJCF).expect("actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.15;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
    .with_contact_geom(0)
}

/// The tip-posed actuated rollout engages the block and stays finite.
#[test]
fn actuator_moving_ee_engages_and_is_stable() {
    let zn = build().coupled_trajectory_actuated_z(&[0.02_f64; 8]);
    assert!(zn.is_finite() && (0.10..0.40).contains(&zn), "got zN={zn}");
}

/// `∂tip_z/∂u_k` (one tape) vs a central FD of the geom-posed `coupled_trajectory_actuated_z`.
/// n = 1 is the single-step leaf; the centre channel (pose-sensitive tip_z) is what makes it match.
#[test]
fn actuator_moving_ee_gradient_matches_fd() {
    let controls = [0.03_f64, -0.02, 0.04, 0.01, -0.015, 0.02];
    for &n in &[1usize, 2, 6] {
        let ctl = &controls[..n];
        let (tip_z, grad) = build().coupled_trajectory_actuator_gradient(ctl);
        let z0 = build().coupled_trajectory_actuated_z(ctl);
        assert!(
            (tip_z - z0).abs() < 1e-10,
            "n={n}: forward mismatch {tip_z} vs {z0}"
        );
        let eps = 1e-3;
        let mut worst = 0.0_f64;
        for k in 0..n {
            let (mut up, mut dn) = (ctl.to_vec(), ctl.to_vec());
            up[k] += eps;
            dn[k] -= eps;
            let fd = (build().coupled_trajectory_actuated_z(&up)
                - build().coupled_trajectory_actuated_z(&dn))
                / (2.0 * eps);
            assert!(fd.abs() > 1e-9, "n={n} k={k}: degenerate FD");
            let rel = (grad[k] - fd).abs() / fd.abs().max(1e-9);
            worst = worst.max(rel);
        }
        eprintln!("actuator moving-EE (n={n}): worst rel = {worst:.3e}");
        assert!(
            worst < 1e-5,
            "n={n}: moving-EE actuator ∂tip_z/∂u disagrees with FD (worst rel {worst:e})"
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────────────────
// FRICTION + POLICY moving-EE gradients. Sideways-gravity grip; the tip sweeps tangentially as the
// motor/policy drives it. Objective = tip_x (the tangential drag), which is POSE-INSENSITIVE (#429:
// zeroing the pose-centre seam leaves ∂tip_x machine-exact) — so these are COMPOSITION/engagement
// gates (the g_act + friction + moving-EE centre composition on one geom-posed tape), NOT pose
// discriminators. The moving-EE friction-pose pieces are proven single-step (#429 leaf gates +
// friction_wrench_node_centre); the pose-sensitive discriminator for the actuator path is the tip_z
// `actuator_moving_ee_gradient_matches_fd` gate above. Tolerance 5e-3 = the curved-friction floor.
// ─────────────────────────────────────────────────────────────────────────────────────────

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

/// Moving-EE ACTUATOR-FRICTION control gradient vs the FD of the geom-posed `actuated_gripped_x`.
#[test]
fn actuator_friction_moving_ee_gradient_matches_fd() {
    let controls = [0.04_f64, -0.03, 0.05, 0.02];
    let (tip_x, grad) = build_fric(3.0e3).coupled_trajectory_actuator_friction_gradient(&controls);
    let x0 = build_fric(3.0e3)
        .coupled_trajectory_actuated_gripped_x(&controls)
        .x;
    assert!(
        (tip_x - x0).abs() < 1e-10,
        "forward mismatch {tip_x} vs {x0}"
    );
    let eps = 1e-3;
    for k in 0..controls.len() {
        let (mut up, mut dn) = (controls, controls);
        up[k] += eps;
        dn[k] -= eps;
        let fd = (build_fric(3.0e3)
            .coupled_trajectory_actuated_gripped_x(&up)
            .x
            - build_fric(3.0e3)
                .coupled_trajectory_actuated_gripped_x(&dn)
                .x)
            / (2.0 * eps);
        assert!(fd.abs() > 1e-9, "k={k}: degenerate FD");
        let rel = (grad[k] - fd).abs() / fd.abs().max(1e-9);
        assert!(
            rel < 5e-3,
            "k={k}: actuator-friction moving-EE {} vs FD {fd} (rel {rel:e})",
            grad[k]
        );
    }
}

/// Moving-EE POLICY-FRICTION gradient (θ) vs the FD of the geom-posed `policy_gripped_x`.
#[test]
fn policy_friction_moving_ee_gradient_matches_fd() {
    let n = 4;
    let (tip_x, theta) =
        build_fric(3.0e3).coupled_trajectory_policy_friction_gradient(&LinearFeedback, &PARAMS, n);
    let x0 = build_fric(3.0e3)
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
        let fd = (build_fric(3.0e3)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &up, n)
            .x
            - build_fric(3.0e3)
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &dn, n)
                .x)
            / (2.0 * eps);
        assert!(fd.abs() > 1e-9, "θ[{k}]: degenerate FD");
        let rel = (theta[k] - fd).abs() / fd.abs().max(1e-9);
        assert!(
            rel < 5e-3,
            "θ[{k}]: policy-friction moving-EE {} vs FD {fd} (rel {rel:e})",
            theta[k]
        );
    }
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
