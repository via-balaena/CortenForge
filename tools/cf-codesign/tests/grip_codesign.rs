//! The de-escalation co-design loop on the soft↔rigid GRIP — the study's first
//! co-design *result*.
//!
//! [`GripCoDesignTarget`] co-designs the soft buffer stiffness `μ` (λ = 4μ) AND a
//! closed-loop holding policy `θ` together so the gripped limb's tangential drag
//! `tip_x` after the coupled rollout reaches a restraint target, reading BOTH
//! gradient blocks `(∂tip_x/∂μ_total, ∂tip_x/∂θ)` from ONE
//! `StaggeredCoupling::coupled_trajectory_design_policy_friction_gradient` backward
//! (the curvature-correct finite-contact gradient, PR #406, gated on the centroid
//! sphere by #430). Unlike [`JointTarget`]'s static free-platen press, the scene is
//! the keystone de-escalation grip: an actuated hinge limb whose finite sphere tip
//! sweeps tangentially into the soft block, gripped by dry friction (soft holding a
//! MOVING limb).
//!
//! The optimizer works in `p = [ln μ, θ…]` (the target owns the μ log-reparam so
//! the positive μ takes relative steps and the signed θ stay linear — both then
//! conditioned by one `loss_scale` Normalized lever and a single lr).
//!
//! Gates:
//! 1. the joint loss gradient (both blocks) matches a central FD of the loss in
//!    `p`-space — including the log-μ chain-rule on the design component;
//! 2. grip inverse design recovers a (μ*, θ*) behavior — from a perturbed start the
//!    optimizer drives `tip_x` to the restraint target by moving BOTH μ and θ.
//!
//! Tolerances are the friction-gradient's (rel ~5e-3), not the smooth normal
//! channel's: the underlying design+policy-friction gradient carries the frozen-lag
//! friction residual (the ~2e-3 intrinsic floor documented in #422), so the loss
//! gradient inherits it.

#![allow(clippy::expect_used)]

use cf_codesign::{CoDesignProblem, GripCoDesignTarget, OptConfig};

// The de-escalation grip: an actuated hinge arm, its finite sphere tip pressed into
// the soft buffer under a sideways gravity that drives the tangential sweep.
const GRIP_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;

const N_STEPS: usize = 10;
// The reference design whose gripped drag is the restraint target: a softer buffer +
// a holding policy.
const MU_STAR: f64 = 3.0e3;
const THETA_STAR: [f64; 3] = [0.0, 0.0, 0.5];

fn target() -> GripCoDesignTarget<sim_coupling::LinearFeedback> {
    let x = GripCoDesignTarget::linear_for_inverse_design(GRIP_MJCF.to_string(), N_STEPS, 0.0)
        .forward_x(MU_STAR, &THETA_STAR);
    GripCoDesignTarget::linear_for_inverse_design(GRIP_MJCF.to_string(), N_STEPS, x)
}

/// Both gradient blocks match a central FD of the loss in `p = [ln μ, θ]` space —
/// the design component carries the log-μ chain-rule factor, the policy components
/// are linear. Independent: the FD re-rolls the real coupled grip sim (rebuilding
/// the block at the perturbed μ for the design component).
#[test]
fn grip_loss_gradient_matches_fd() {
    let problem = target();
    // A generic non-optimal point: μ = 5e3 (p0 = ln 5e3), a generic holding policy.
    let p = problem.x0(5.0e3, &[0.05, -0.1, 0.2]);
    let (_loss, grad) = problem.evaluate(&p);

    let loss_at = |p: &[f64]| {
        let (loss, _) = problem.evaluate(p);
        loss
    };
    let names = ["ln μ", "w_z", "w_vz", "b"];
    for i in 0..p.len() {
        // ln μ ~ 8.5 (relative eps); weights/bias O(1) but the tip_x lever is small,
        // so use a slightly larger linear eps for a clean central difference.
        let eps = if i == 0 { 1e-5 } else { 1e-4 };
        let mut up = p.clone();
        let mut dn = p.clone();
        up[i] += eps;
        dn[i] -= eps;
        let fd = (loss_at(&up) - loss_at(&dn)) / (2.0 * eps);
        let rel = (grad[i] - fd).abs() / fd.abs().max(1e-30);
        let abs = (grad[i] - fd).abs();
        eprintln!(
            "{}: grad={:.6e} fd={fd:.6e} rel={rel:.3e} abs={abs:.3e}",
            names[i], grad[i]
        );
        // The FD of the loss must be a real signal (not near-zero), else the rel
        // check is vacuously satisfied — mirroring the upstream centroid gate's
        // non-degenerate-FD guard (which the bare template omits). Every component
        // is O(1e-8..1e-6) at this point, so this is a safety net against a future
        // test-point shift, not a live constraint.
        assert!(
            fd.abs() > 1e-9,
            "∂loss/∂{} degenerate FD ({fd:e}) — pick a non-trivial test point",
            names[i]
        );
        // Friction-gradient tolerance (the underlying design+policy-friction gradient
        // is gated at rel ~5e-3, per the #422 intrinsic frozen-lag floor).
        assert!(
            rel < 5e-3,
            "∂loss/∂{} {} vs FD {fd} (rel {rel:e} abs {abs:e})",
            names[i],
            grad[i]
        );
    }
}

/// Grip inverse design recovers a target behavior: from a perturbed start (firmer
/// buffer AND zero policy), the optimizer drives `tip_x` to the restraint target by
/// moving BOTH the design variable and the holding policy. (Under-determined — 1+3
/// params for one scalar target — so this is behavior recovery, not unique (μ*,θ*).)
#[test]
fn grip_inverse_design_recovers_behavior() {
    let problem = target();
    let x_tgt = problem.target_x();
    // Start: firmer buffer, zero policy — far from the reference in BOTH axes.
    let mu0 = 6.0e3;
    let x0 = problem.x0(mu0, &[0.0, 0.0, 0.0]);
    let x_start = {
        let (m0, th0) = problem.to_physical(&x0);
        problem.forward_x(m0, &th0)
    };

    // Dimensionless residual (residual scale = the target drag magnitude); log_space
    // = false (the μ log-reparam is inside the target, θ is linear). Standard eps; a
    // single lr suiting both relative-μ and linear-θ steps.
    let normalized = problem.recommended_normalized(x_tgt.abs().max(1e-3));
    let cfg = OptConfig {
        lr: 0.03,
        max_iters: 800,
        loss_tol: 1e-18,
        ..OptConfig::default()
    };
    assert!(
        (cfg.eps - OptConfig::default().eps).abs() < f64::EPSILON,
        "standard eps"
    );

    let result = normalized.optimize(&x0, &cfg);
    let (mu_rec, th_rec) = problem.to_physical(&result.params);
    let x_final = problem.forward_x(mu_rec, &th_rec);
    eprintln!(
        "x_tgt={x_tgt:.9} x_start={x_start:.9} x_final={x_final:.9} |x-tgt|={:.3e} \
         μ: {mu0} → {mu_rec:.1} (μ*={MU_STAR})  θ_rec={th_rec:?}  iters={} conv={}",
        (x_final - x_tgt).abs(),
        result.iters,
        result.converged,
    );
    assert!(result.converged, "grip inverse design did not converge");
    assert!(
        (x_final - x_tgt).abs() < 1e-8,
        "recovered (μ, θ) should hit the restraint target: x_final {x_final} vs target {x_tgt}"
    );
    // The optimizer genuinely moved the design variable μ off its start (both axes
    // are live, not just the policy carrying the whole correction). The μ lever on a
    // policy-driven tip_x is weaker than the policy's, so the threshold is modest.
    assert!(
        (mu_rec - mu0).abs() > 500.0,
        "μ should move off its {mu0} start (the design axis is live), got {mu_rec}"
    );
    // Sanity: the target differs from the start drag, so there was real work.
    assert!(
        (x_tgt - x_start).abs() > 1e-6,
        "restraint target should differ from the start drag"
    );
}
