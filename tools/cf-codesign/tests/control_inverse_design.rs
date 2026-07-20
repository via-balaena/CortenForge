//! The POLICY half of the co-design loop — open-loop control inverse design.
//!
//! [`ControlScheduleTarget`] tunes a per-step platen control force schedule
//! `u_0 … u_{N−1}` so the platen's height `z_N` after the coupled soft↔rigid
//! rollout matches a target, with the gradient `∂z_N/∂u_k` flowing through the
//! keystone's multi-step coupled tape
//! (`StaggeredCoupling::coupled_trajectory_control_gradient`). It is the control
//! analogue of the material trajectory target — the other half of the mission's
//! "one outer loop differentiating w.r.t. both design AND policy parameters".
//!
//! Gates:
//! 1. the problem's loss gradient matches a central FD of the loss (the
//!    consumer's `residual · ∂z_N/∂u` wiring is the correct objective gradient);
//! 2. inverse design recovers a target platen behavior — from a zero schedule,
//!    the optimizer finds a control schedule driving `z_N` to the target;
//! 3. a negative control: the same optimizer WITHOUT the `Normalized`
//!    conditioning (same `lr`, raw loss) does NOT converge — the weakly-sensitive
//!    control gradient sits below Adam's standard `eps`, so normalization is
//!    load-bearing (a signed-parameter analogue of the v3 material result, using
//!    the dimensionless-residual lever since log-space needs positive params).

#![allow(clippy::expect_used)]

use cf_codesign::{CoDesignProblem, ControlScheduleTarget, OptConfig};

// A platen started already in contact (the keystone trajectory fixture); the
// control schedule pushes the platen up/down against the soft block.
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

/// A reference schedule whose final platen height is the inverse-design target.
fn reference_schedule() -> Vec<f64> {
    (0..N_STEPS)
        .map(|k| if k % 2 == 0 { -1.0 } else { 0.8 })
        .collect()
}

/// The problem's analytic loss gradient matches a central FD of the loss w.r.t.
/// each control input — the objective gradient is correct (not just a descent
/// direction). An independent check: the FD re-rolls the real coupled sim.
#[test]
fn control_loss_gradient_matches_fd() {
    let target_z = ControlScheduleTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, 0.0)
        .forward_z(&reference_schedule());
    let problem =
        ControlScheduleTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, target_z);

    // Evaluate at a generic (non-optimal) schedule so the residual is nonzero.
    let u: Vec<f64> = (0..N_STEPS).map(|k| 0.3 * (k as f64) - 1.0).collect();
    let (_loss, grad) = problem.evaluate(&u);

    let loss_at = |u: &[f64]| {
        let r = problem.forward_z(u) - target_z;
        0.5 * r * r
    };
    for k in 0..N_STEPS {
        let eps = 1e-2;
        let mut up = u.clone();
        let mut dn = u.clone();
        up[k] += eps;
        dn[k] -= eps;
        let fd = (loss_at(&up) - loss_at(&dn)) / (2.0 * eps);
        let rel = (grad[k] - fd).abs() / fd.abs().max(1e-30);
        let abs = (grad[k] - fd).abs();
        eprintln!(
            "k={k}: grad={:.6e} fd={fd:.6e} rel={rel:.3e} abs={abs:.3e}",
            grad[k]
        );
        assert!(
            rel < 1e-5 || abs < 1e-16,
            "∂loss/∂u_{k} {} vs FD {fd} (rel {rel:e} abs {abs:e})",
            grad[k]
        );
    }
}

/// Inverse design recovers a target platen behavior: from a zero control
/// schedule, the optimizer finds a schedule that drives `z_N` to the target
/// height. (The schedule is under-determined — N controls for one scalar target —
/// so this is behavior recovery, the natural open-loop trajectory-optimization
/// objective, not unique-parameter recovery.)
#[test]
fn inverse_design_recovers_target_behavior() {
    let setup = ControlScheduleTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, 0.0);
    let target_z = setup.forward_z(&reference_schedule());
    let z_zero = setup.forward_z(&[0.0; N_STEPS]);
    let target =
        ControlScheduleTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, target_z);

    // Dimensionless residual (loss_scale = 1/L², L = 1e-3 m platen-height scale);
    // log_space = false because control forces are signed. Standard Adam eps;
    // a control-appropriate lr (the default physical-μ lr is far too large for an
    // O(1)-newton step).
    let problem = target.recommended_normalized(1e-3);
    let cfg = OptConfig {
        lr: 0.02,
        max_iters: 400,
        loss_tol: 1e-18,
        ..OptConfig::default()
    };
    assert!(
        (cfg.eps - OptConfig::default().eps).abs() < f64::EPSILON,
        "standard eps"
    );

    let result = problem.optimize(&[0.0; N_STEPS], &cfg);
    let z_final = target.forward_z(&result.params);
    eprintln!(
        "target_z={target_z:.9} z(0)={z_zero:.9} z_final={z_final:.9} \
         |z-tgt|={:.3e} iters={} conv={}",
        (z_final - target_z).abs(),
        result.iters,
        result.converged()
    );
    assert!(
        result.converged(),
        "control inverse design did not converge"
    );
    assert!(
        (z_final - target_z).abs() < 1e-9,
        "recovered schedule should hit the target height: z_final {z_final} vs target {target_z}"
    );
    // Sanity: the target genuinely differs from the do-nothing height, so the
    // optimizer had real work to do.
    assert!(
        (target_z - z_zero).abs() > 1e-6,
        "target should differ from the zero-control height"
    );
}

/// Negative control: the SAME optimizer (same lr) on the RAW target — without the
/// `Normalized` dimensionless-residual conditioning — does NOT reach the target.
/// The control gradient `∂z_N/∂u ~ 1e-5` gives a loss gradient below Adam's
/// standard `eps = 1e-8`, so the raw run crawls; the conditioning is load-bearing.
#[test]
fn normalization_is_load_bearing() {
    let setup = ControlScheduleTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, 0.0);
    let target_z = setup.forward_z(&reference_schedule());
    let target =
        ControlScheduleTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, target_z);

    // Raw target (no Normalized), same control-appropriate lr.
    let cfg = OptConfig {
        lr: 0.02,
        max_iters: 400,
        loss_tol: 1e-30,
        ..OptConfig::default()
    };
    let result = cf_codesign::optimize(&target, &[0.0; N_STEPS], &cfg);
    let z_final = target.forward_z(&result.params);
    eprintln!(
        "raw: z_final={z_final:.9} target={target_z:.9} |z-tgt|={:.3e} conv={}",
        (z_final - target_z).abs(),
        result.converged()
    );
    assert!(
        (z_final - target_z).abs() > 1e-7,
        "raw (un-normalized) run should NOT reach the target — it sits below eps and crawls"
    );
}
