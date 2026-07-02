//! The DESIGN+POLICY (grip co-design) trajectory gradient is curvature-correct on a CENTROID finite
//! sphere.
//!
//! These gradients (`coupled_trajectory_actuator_gradient`, `..._actuator_friction_gradient`,
//! `..._policy_friction_gradient`, `..._design_policy_friction_gradient`) were `require_plane_collider`-
//! guarded as the "exo follow-on" — but the guard was CONSERVATIVE. The actuator velocity column
//! (`g_act`) is contact-INDEPENDENT (`Δt·M⁻¹·∂qfrc_actuator/∂ctrl`), and the contact nodes these
//! gradients build (`trajectory_step_vjp` / `trajectory_step_vjp_grip`, `ContactWrenchTrajVjp` with
//! `collider_hessian`, `FrictionWrenchTrajVjp`) were all made curvature-correct by #415–#429. So
//! lifting the guard (→ `require_no_moving_ee`) just exposed an already-correct sphere gradient.
//!
//! The one-tape actuator (`sphere·actuator`), actuator-friction (`sphere·actuator-friction`), and
//! policy-friction (`sphere·policy-friction`) centroid-sphere gradients are now rows of
//! `coupling_grad_harness.rs` (each a tape-vs-central-FD check of the independent full-coupled
//! rollout oracle on a `with_sphere_collider` centroid-posed scene). What remains HERE is the
//! DESIGN+POLICY-friction grip CO-DESIGN gradient (#406, R5's — the μ + θ channels), which stays
//! bespoke until the grip co-design family is folded into the harness (the fallible
//! `try_`/`RolloutError` path).
//!
//! **What this gate proves (and what it does NOT).** It proves the design+policy gradient COMPOSITION
//! is machine-exact on a sphere — the design (μ) and policy (θ) channels compose correctly with the
//! curved contact nodes (tape-vs-sphere-FD, the FD oracle poses the SAME sphere). It does NOT by
//! itself DISCRIMINATE the curved-normal term: in this gentle large-sphere regime (r = 0.08,
//! south-pole patch ⇒ `∇²sd` ≈ 0) the curvature contribution to the trajectory gradient is below
//! the gate tolerance (zeroing `SphereSdf::hessian` leaves it passing) — the same small-curvature
//! regime as #422. The curved-contact term's correctness is gated SINGLE-STEP
//! (`sphere_contact_total_jacobian.rs`, `sim_soft/tests/friction_sphere_tangent.rs`, and the
//! materially-curved articulated trajectory gates `sphere_articulated_{,friction_}trajectory_gradient.rs`).
//! The point here is that lifting the conservative plane guard exposed an already-correct gradient.

#![allow(clippy::expect_used)]

use sim_coupling::{LinearFeedback, StaggeredCoupling};
use sim_mjcf::load_model;

const SPHERE_R: f64 = 0.08;
const PARAMS: [f64; 3] = [0.08, -0.02, 0.01];

// Sideways-gravity scene (the design+policy grip co-design gradient): the tip sweeps tangentially →
// a controlled friction grip.
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
