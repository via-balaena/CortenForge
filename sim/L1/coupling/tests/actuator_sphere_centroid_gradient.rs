//! The actuator/policy trajectory gradients are curvature-correct on a CENTROID finite sphere.
//!
//! These four gradients (`coupled_trajectory_actuator_gradient`, `..._actuator_friction_gradient`,
//! `..._policy_friction_gradient`, `..._design_policy_friction_gradient`) were `require_plane_collider`-
//! guarded as the "exo follow-on" — but the guard was CONSERVATIVE. The actuator velocity column
//! (`g_act`) is contact-INDEPENDENT (`Δt·M⁻¹·∂qfrc_actuator/∂ctrl`), and the contact nodes these
//! gradients build (`trajectory_step_vjp` / `trajectory_step_vjp_grip`, `ContactWrenchTrajVjp` with
//! `collider_hessian`, `FrictionWrenchTrajVjp`) were all made curvature-correct by #415–#429. So
//! lifting the guard (→ `require_no_moving_ee`) just exposed an already-correct sphere gradient.
//!
//! Each gate is the one-tape gradient vs a central FD of the independent full-coupled rollout
//! oracle, on a `with_sphere_collider` (centroid-posed, no `with_contact_geom`) scene. The moving
//! end-effector (tip-posed) centre channel was the follow-on, now done — `actuator_moving_ee_gradient.rs`.
//!
//! **What these gates prove (and what they do NOT).** They prove the actuator/policy gradient
//! COMPOSITION is machine-exact on a sphere — the `g_act` channel composes correctly with the
//! curved contact nodes (tape-vs-sphere-FD, the FD oracle poses the SAME sphere). They do NOT by
//! themselves DISCRIMINATE the curved-normal term: in this gentle large-sphere regime (r = 0.08,
//! south-pole patch ⇒ `∇²sd` ≈ 0) the curvature contribution to the trajectory gradient is below
//! the gate tolerance (zeroing `SphereSdf::hessian` leaves all four passing) — the same
//! small-curvature regime as #422. The curved-contact term's correctness is gated SINGLE-STEP
//! (`sphere_contact_total_jacobian.rs`, `sim_soft/tests/friction_sphere_tangent.rs`, and the
//! materially-curved articulated trajectory gates `sphere_articulated_{,friction_}trajectory_gradient.rs`).
//! The point here is that lifting the conservative plane guard exposed an already-correct gradient.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

const SPHERE_R: f64 = 0.08;
const PARAMS: [f64; 3] = [0.08, -0.02, 0.01];

// Straight-down gravity actuator scene (the NORMAL actuator gradient): a motor drives the hinge,
// the sphere tip presses the block top.
fn normal_scene() -> &'static str {
    r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor name="a" joint="j" gear="4"/></actuator>
</mujoco>"#
}

// Sideways-gravity scene (the FRICTION / POLICY gradients): the tip sweeps tangentially → a
// controlled friction grip.
fn fric_scene() -> &'static str {
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

fn build(scene: &str, soft_mu: f64, friction: bool) -> StaggeredCoupling {
    let model = load_model(scene).expect("scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    let c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, soft_mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R);
    if friction {
        c.with_friction(2.5, 0.1)
    } else {
        c
    }
}

/// NORMAL actuator control gradient vs the FD of `coupled_trajectory_actuated_z`, on a centroid sphere.
#[test]
fn actuator_normal_sphere_centroid_matches_fd() {
    let controls = [0.3_f64, -0.2, 0.4];
    let (tip_z, grad) =
        build(normal_scene(), 3.0e4, false).coupled_trajectory_actuator_gradient(&controls);
    let z0 = build(normal_scene(), 3.0e4, false).coupled_trajectory_actuated_z(&controls);
    assert!(
        (tip_z - z0).abs() < 1e-10,
        "forward mismatch: {tip_z} vs {z0}"
    );
    let eps = 1e-3;
    for k in 0..controls.len() {
        let (mut up, mut dn) = (controls, controls);
        up[k] += eps;
        dn[k] -= eps;
        let fd = (build(normal_scene(), 3.0e4, false).coupled_trajectory_actuated_z(&up)
            - build(normal_scene(), 3.0e4, false).coupled_trajectory_actuated_z(&dn))
            / (2.0 * eps);
        let rel = (grad[k] - fd).abs() / fd.abs().max(1e-9);
        assert!(fd.abs() > 1e-9, "k={k}: degenerate FD");
        assert!(
            rel < 1e-5,
            "k={k}: actuator-normal sphere gradient {} vs FD {fd} (rel {rel:e})",
            grad[k]
        );
    }
}

/// FRICTION actuator control gradient vs the FD of `coupled_trajectory_actuated_gripped_x`.
#[test]
fn actuator_friction_sphere_centroid_matches_fd() {
    let controls = [0.1_f64, -0.05, 0.08, 0.04];
    let (tip_x, grad) =
        build(fric_scene(), 3.0e3, true).coupled_trajectory_actuator_friction_gradient(&controls);
    let x0 = build(fric_scene(), 3.0e3, true)
        .coupled_trajectory_actuated_gripped_x(&controls)
        .x;
    assert!(
        (tip_x - x0).abs() < 1e-10,
        "forward mismatch: {tip_x} vs {x0}"
    );
    let eps = 1e-3;
    for k in 0..controls.len() {
        let (mut up, mut dn) = (controls, controls);
        up[k] += eps;
        dn[k] -= eps;
        let fd = (build(fric_scene(), 3.0e3, true)
            .coupled_trajectory_actuated_gripped_x(&up)
            .x
            - build(fric_scene(), 3.0e3, true)
                .coupled_trajectory_actuated_gripped_x(&dn)
                .x)
            / (2.0 * eps);
        let rel = (grad[k] - fd).abs() / fd.abs().max(1e-9);
        assert!(fd.abs() > 1e-9, "k={k}: degenerate FD");
        assert!(
            rel < 5e-3,
            "k={k}: actuator-friction sphere gradient {} vs FD {fd} (rel {rel:e})",
            grad[k]
        );
    }
}

/// POLICY-friction gradient (θ channel) vs the FD of `coupled_trajectory_policy_gripped_x`.
#[test]
fn policy_friction_sphere_centroid_matches_fd() {
    let n = 4;
    let (tip_x, theta) = build(fric_scene(), 3.0e3, true)
        .coupled_trajectory_policy_friction_gradient(&LinearFeedback, &PARAMS, n);
    let x0 = build(fric_scene(), 3.0e3, true)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
        .x;
    assert!(
        (tip_x - x0).abs() < 1e-10,
        "forward mismatch: {tip_x} vs {x0}"
    );
    let eps = 1e-6;
    for k in 0..PARAMS.len() {
        let (mut up, mut dn) = (PARAMS, PARAMS);
        up[k] += eps;
        dn[k] -= eps;
        let fd = (build(fric_scene(), 3.0e3, true)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &up, n)
            .x
            - build(fric_scene(), 3.0e3, true)
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &dn, n)
                .x)
            / (2.0 * eps);
        let rel = (theta[k] - fd).abs() / fd.abs().max(1e-9);
        assert!(fd.abs() > 1e-9, "k={k}: degenerate FD");
        assert!(
            rel < 5e-3,
            "θ[{k}]: policy-friction sphere gradient {} vs FD {fd} (rel {rel:e})",
            theta[k]
        );
    }
}

/// DESIGN+POLICY-friction gradient (#406, R5's gradient) — both the design (μ) and policy (θ)
/// channels vs the FD of `coupled_trajectory_policy_gripped_x`, on a centroid sphere.
#[test]
fn design_policy_friction_sphere_centroid_matches_fd() {
    let n = 4;
    let mu0 = 3.0e3;
    let (tip_x, mu_grad, theta) = build(fric_scene(), mu0, true)
        .coupled_trajectory_design_policy_friction_gradient(&LinearFeedback, &PARAMS, n);
    let x0 = build(fric_scene(), mu0, true)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
        .x;
    assert!(
        (tip_x - x0).abs() < 1e-10,
        "forward mismatch: {tip_x} vs {x0}"
    );
    // θ channel.
    let eps = 1e-6;
    for k in 0..PARAMS.len() {
        let (mut up, mut dn) = (PARAMS, PARAMS);
        up[k] += eps;
        dn[k] -= eps;
        let fd = (build(fric_scene(), mu0, true)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &up, n)
            .x
            - build(fric_scene(), mu0, true)
                .coupled_trajectory_policy_gripped_x(&LinearFeedback, &dn, n)
                .x)
            / (2.0 * eps);
        let rel = (theta[k] - fd).abs() / fd.abs().max(1e-9);
        assert!(fd.abs() > 1e-9, "θ[{k}]: degenerate FD");
        assert!(
            rel < 5e-3,
            "θ[{k}]: design+policy sphere gradient {} vs FD {fd} (rel {rel:e})",
            theta[k]
        );
    }
    // μ (design) channel — tied λ = 4μ moves with the build perturbation.
    let eps_mu = mu0 * 1e-4;
    let fd_mu = (build(fric_scene(), mu0 + eps_mu, true)
        .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
        .x
        - build(fric_scene(), mu0 - eps_mu, true)
            .coupled_trajectory_policy_gripped_x(&LinearFeedback, &PARAMS, n)
            .x)
        / (2.0 * eps_mu);
    let rel_mu = (mu_grad - fd_mu).abs() / fd_mu.abs().max(1e-9);
    // The design (μ) lever on a policy-driven tip_x is intrinsically small (the policy adapts), but
    // must be a real signal — else the rel check is vacuously satisfied via the floor.
    assert!(
        fd_mu.abs() > 1e-9,
        "μ: degenerate FD ({fd_mu:e}) — design lever inert?"
    );
    assert!(
        rel_mu < 5e-3,
        "μ: design+policy sphere gradient {mu_grad} vs FD {fd_mu} (rel {rel_mu:e})"
    );
}
