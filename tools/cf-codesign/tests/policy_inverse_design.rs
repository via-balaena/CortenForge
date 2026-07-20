//! The CLOSED-LOOP policy half of the co-design loop — feedback-policy inverse design.
//!
//! [`FeedbackPolicyTarget`] tunes the parameters θ of a state-feedback policy
//! `u_k = π_θ(state_k)` ([`LinearFeedback`]: `u = w_z·z + w_vz·vz + b`) so the
//! platen's height `z_N` after the coupled soft↔rigid rollout matches a target,
//! with the gradient `∂z_N/∂θ` flowing through the keystone's multi-step coupled
//! tape via backprop-through-time across the state→control recurrence
//! (`StaggeredCoupling::coupled_trajectory_policy_gradient`). It is the closed-loop
//! analogue of the open-loop [`ControlScheduleTarget`].
//!
//! Gates:
//! 1. the problem's loss gradient matches a central FD of the loss, per policy
//!    parameter — including the feedback weights, whose gradient flows only through
//!    the recurrence (the consumer's `residual · ∂z_N/∂θ` wiring is correct);
//! 2. inverse design recovers a target platen behavior — from a zero policy, the
//!    optimizer finds a feedback policy driving `z_N` to the target;
//! 3. a negative control: the same optimizer WITHOUT the `Normalized` conditioning
//!    (same `lr`, raw loss) does NOT converge and stalls orders of magnitude short
//!    of the normalized run's deep convergence — normalization is load-bearing.
//!    (Refined finding vs the open-loop control half: the linear policy's bias `b`
//!    adds to *every* step's control, so `∂z_N/∂b ≈ N·(per-step) ~ 1e-4` is ~10×
//!    a single open-loop control's sensitivity — the raw loss gradient starts only
//!    marginally above `eps`, so the raw run is *partially* effective rather than
//!    fully dead, but still stalls ~3 orders short of the conditioned run.)

#![allow(clippy::expect_used)]

use cf_codesign::{CoDesignProblem, FeedbackPolicyTarget, OptConfig, StopReason};

// A platen started already in contact (the keystone trajectory fixture); the
// feedback policy pushes the platen up/down against the soft block.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const N_STEPS: usize = 8;

/// A reference feedback policy whose final platen height is the inverse-design
/// target: restoring (negative `w_z`), velocity damping (negative `w_vz`), bias.
const REFERENCE_THETA: [f64; 3] = [-15.0, -3.0, 1.2];

fn setup(target_z: f64) -> FeedbackPolicyTarget<sim_coupling::LinearFeedback> {
    FeedbackPolicyTarget::linear_for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, target_z)
}

/// The problem's analytic loss gradient matches a central FD of the loss w.r.t.
/// each policy parameter — the objective gradient is correct (not just a descent
/// direction). Independent: the FD re-rolls the real closed-loop coupled sim.
#[test]
fn policy_loss_gradient_matches_fd() {
    let target_z = setup(0.0).forward_z(&REFERENCE_THETA);
    let problem = setup(target_z);

    // Evaluate at a generic (non-optimal) policy so the residual is nonzero.
    let theta = [-8.0_f64, -1.5, 0.6];
    let (_loss, grad) = problem.evaluate(&theta);

    let loss_at = |t: &[f64]| {
        let r = problem.forward_z(t) - target_z;
        0.5 * r * r
    };
    let names = ["w_z", "w_vz", "b"];
    for i in 0..3 {
        let eps = if i == 2 { 1e-3 } else { 1e-2 };
        let mut up = theta;
        let mut dn = theta;
        up[i] += eps;
        dn[i] -= eps;
        let fd = (loss_at(&up) - loss_at(&dn)) / (2.0 * eps);
        let rel = (grad[i] - fd).abs() / fd.abs().max(1e-30);
        let abs = (grad[i] - fd).abs();
        eprintln!(
            "{}: grad={:.6e} fd={fd:.6e} rel={rel:.3e} abs={abs:.3e}",
            names[i], grad[i]
        );
        assert!(
            rel < 1e-5 || abs < 1e-16,
            "∂loss/∂{} {} vs FD {fd} (rel {rel:e} abs {abs:e})",
            names[i],
            grad[i]
        );
    }
}

/// Inverse design recovers a target platen behavior: from a zero policy
/// (`θ = 0`, no control), the optimizer finds a feedback policy that drives `z_N`
/// to the target height. (Under-determined — 3 params for one scalar target — so
/// this is behavior recovery, the natural trajectory-optimization objective, not
/// unique-parameter recovery.)
#[test]
fn inverse_design_recovers_target_behavior() {
    let target_z = setup(0.0).forward_z(&REFERENCE_THETA);
    let z_zero = setup(0.0).forward_z(&[0.0; 3]);
    let target = setup(target_z);

    // Dimensionless residual (loss_scale = 1/L², L = 1e-3 m platen-height scale);
    // log_space = false because the linear policy's weights are signed. Standard
    // Adam eps; a state-feedback-appropriate lr.
    let problem = target.recommended_normalized(1e-3);
    let cfg = OptConfig {
        lr: 0.02,
        max_iters: 600,
        loss_tol: 1e-18,
        ..OptConfig::default()
    };
    assert!(
        (cfg.eps - OptConfig::default().eps).abs() < f64::EPSILON,
        "standard eps"
    );

    let result = problem.optimize(&[0.0; 3], &cfg);
    let z_final = target.forward_z(&result.params);
    eprintln!(
        "target_z={target_z:.9} z(0)={z_zero:.9} z_final={z_final:.9} \
         |z-tgt|={:.3e} iters={} stop={:?} params={:?}",
        (z_final - target_z).abs(),
        result.iters,
        result.stop_reason,
        result.params,
    );
    assert!(
        matches!(
            result.stop_reason,
            StopReason::GradTol | StopReason::LossTol
        ),
        "policy inverse design did not converge"
    );
    assert!(
        (z_final - target_z).abs() < 1e-9,
        "recovered policy should hit the target height: z_final {z_final} vs target {target_z}"
    );
    // Sanity: the target genuinely differs from the do-nothing height, so the
    // optimizer had real work to do.
    assert!(
        (target_z - z_zero).abs() > 1e-6,
        "target should differ from the zero-policy height"
    );
}

/// Negative control: the SAME optimizer (same lr, same budget) on the RAW target —
/// without the `Normalized` dimensionless-residual conditioning — does NOT converge
/// and stalls orders of magnitude short of the normalized run's deep convergence
/// (which reaches `|z-tgt| ~ 1e-12`, see `inverse_design_recovers_target_behavior`).
/// As `z_N → target` the raw loss gradient `residual · ∂z_N/∂θ` collapses below
/// Adam's standard `eps = 1e-8` and the run grinds to a halt; the dimensionless
/// residual keeps it scale-invariant, so the conditioning is load-bearing.
#[test]
fn normalization_is_load_bearing() {
    let target_z = setup(0.0).forward_z(&REFERENCE_THETA);
    let target = setup(target_z);

    // Raw target (no Normalized), same feedback-appropriate lr/budget.
    let cfg = OptConfig {
        lr: 0.02,
        max_iters: 600,
        loss_tol: 1e-30,
        ..OptConfig::default()
    };
    let result = cf_codesign::optimize(&target, &[0.0; 3], &cfg);
    let z_final = target.forward_z(&result.params);
    eprintln!(
        "raw: z_final={z_final:.9} target={target_z:.9} |z-tgt|={:.3e} stop={:?}",
        (z_final - target_z).abs(),
        result.stop_reason
    );
    assert_eq!(
        result.stop_reason,
        StopReason::MaxIters,
        "raw run should not converge to loss_tol"
    );
    // Raw stalls ~1e-9 (eps-limited as the gradient collapses); the conditioned run
    // reaches ~1e-12. The 1e-10 bar (>10× the raw stall point's margin) is robust
    // while the normalized run is ~3 orders past it.
    assert!(
        (z_final - target_z).abs() > 1e-10,
        "raw (un-normalized) run should stall well short of deep convergence, \
         got |z-tgt| {}",
        (z_final - target_z).abs()
    );
    // Verify the STATED mechanism (not just the symptom): the raw loss gradient has
    // collapsed below Adam's standard eps, which is why the run stalls. (The
    // `Normalized` run keeps the gradient above eps via the loss_scale lever.)
    let final_grad_inf = result
        .history
        .last()
        .expect("nonempty descent history")
        .grad_inf;
    eprintln!(
        "raw final |grad|∞ = {final_grad_inf:.3e} (Adam eps = {:.0e})",
        cfg.eps
    );
    assert!(
        final_grad_inf < cfg.eps,
        "raw run should stall because its gradient collapsed below eps, \
         got |grad|∞ {final_grad_inf} vs eps {}",
        cfg.eps
    );
}
