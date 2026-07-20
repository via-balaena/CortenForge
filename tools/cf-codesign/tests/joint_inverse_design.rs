//! The JOINT design+policy co-design loop — the mission's "both in one outer loop".
//!
//! [`JointTarget`] optimizes the soft material design variable `μ` (stiffness
//! scale, λ = 4μ) AND a closed-loop feedback policy `θ` together so the platen's
//! height `z_N` after the coupled rollout hits a target, reading BOTH gradient
//! blocks `(∂z_N/∂μ_total, ∂z_N/∂θ)` from ONE
//! `StaggeredCoupling::coupled_trajectory_joint_gradient` backward. It is the
//! literal realization of `MISSION.md`'s "one outer loop differentiating w.r.t.
//! both design AND policy parameters".
//!
//! The optimizer works in `p = [ln μ, θ…]` (the target owns the μ log-reparam so
//! the positive μ takes relative steps and the signed θ stay linear — both then
//! conditioned by one `loss_scale` Normalized lever and a single lr).
//!
//! Gates:
//! 1. the joint loss gradient (both blocks) matches a central FD of the loss in
//!    `p`-space — including the log-μ chain-rule on the design component;
//! 2. joint inverse design recovers a (μ*, θ*) behavior — from a perturbed start
//!    the optimizer drives `z_N` to the target by moving BOTH μ and θ.
//!
//! (Honest conditioning note: unlike the single-axis policy target, there is no
//! load-bearing `loss_scale` negative control here — the target's internal log-μ
//! reparametrization is what makes the joint loop tractable [it both scales the μ
//! gradient and makes μ's Adam step relative so one `lr` serves μ and θ], and it
//! already lifts the `p`-space gradient near/above the standard `eps`, so even the
//! raw run converges; the `loss_scale` lever is then belt-and-suspenders headroom,
//! not strictly required. And because the problem is under-determined, θ can
//! compensate for a stuck μ, so a `z_N`-based conditioning negative control would
//! be ambiguous anyway. The log-μ reparam being load-bearing is shown directly:
//! gate 2 asserts μ genuinely moves off its start.)

#![allow(clippy::expect_used)]

use cf_codesign::{CoDesignProblem, JointTarget, OptConfig};

const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const N_STEPS: usize = 10;
// The reference design whose rollout height is the target: stiffer block + a
// restoring/damping/bias feedback policy.
const MU_STAR: f64 = 4.0e4;
const THETA_STAR: [f64; 3] = [-15.0, -3.0, 1.2];

fn target() -> JointTarget<sim_coupling::LinearFeedback> {
    let z = JointTarget::linear_for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, 0.0)
        .forward_z(MU_STAR, &THETA_STAR);
    JointTarget::linear_for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, z)
}

/// Both gradient blocks match a central FD of the loss in `p = [ln μ, θ]` space —
/// the design component carries the log-μ chain-rule factor, the policy components
/// are linear. Independent: the FD re-rolls the real coupled sim (rebuilding the
/// block at the perturbed μ for the design component).
#[test]
fn joint_loss_gradient_matches_fd() {
    let problem = target();
    // A generic non-optimal point: μ = 3e4 (p0 = ln 3e4), a generic policy.
    let p = problem.x0(3.0e4, &[-8.0, -1.5, 0.6]);
    let (_loss, grad) = problem.evaluate(&p);

    let loss_at = |p: &[f64]| {
        let (loss, _) = problem.evaluate(p);
        loss
    };
    let names = ["ln μ", "w_z", "w_vz", "b"];
    for i in 0..p.len() {
        // ln μ ~ 10.3 (relative eps); weights O(1); bias O(1).
        let eps = if i == 0 {
            1e-5
        } else if i == 3 {
            1e-3
        } else {
            1e-2
        };
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
        // 1e-5 (matching the policy-loss test and the underlying gradient blocks);
        // the composition is linear so it achieves ~5e-7 here, leaving a healthy
        // margin while still catching a real 1e-5-scale glue regression.
        assert!(
            rel < 1e-5 || abs < 1e-16,
            "∂loss/∂{} {} vs FD {fd} (rel {rel:e} abs {abs:e})",
            names[i],
            grad[i]
        );
    }
}

/// Joint inverse design recovers a target behavior: from a perturbed start
/// (different μ AND different θ), the optimizer drives `z_N` to the target by
/// moving BOTH the design variable and the policy. (Under-determined — 1+3 params
/// for one scalar target — so this is behavior recovery, not unique (μ*, θ*).)
#[test]
fn joint_inverse_design_recovers_behavior() {
    let problem = target();
    let z_tgt = problem.target_z();
    // Start: softer block, zero policy — far from the reference in BOTH axes.
    let x0 = problem.x0(2.0e4, &[0.0, 0.0, 0.0]);
    let z_start = {
        let (mu0, th0) = problem.to_physical(&x0);
        problem.forward_z(mu0, &th0)
    };

    // Dimensionless residual; log_space=false (the μ log-reparam is inside the
    // target, θ is linear). Standard eps; a single lr suiting both relative-μ and
    // linear-θ steps.
    let normalized = problem.recommended_normalized(1e-3);
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
    let z_final = problem.forward_z(mu_rec, &th_rec);
    eprintln!(
        "z_tgt={z_tgt:.9} z_start={z_start:.9} z_final={z_final:.9} |z-tgt|={:.3e} \
         μ: 2e4 → {mu_rec:.1} (μ*={MU_STAR})  θ_rec={th_rec:?}  iters={} conv={}",
        (z_final - z_tgt).abs(),
        result.iters,
        result.converged(),
    );
    assert!(result.converged(), "joint inverse design did not converge");
    assert!(
        (z_final - z_tgt).abs() < 1e-9,
        "recovered (μ, θ) should hit the target height: z_final {z_final} vs target {z_tgt}"
    );
    // The optimizer genuinely moved the design variable μ off its start (both axes
    // are live, not just the policy carrying the whole correction).
    assert!(
        (mu_rec - 2.0e4).abs() > 1.0e3,
        "μ should move off its 2e4 start (the design axis is live), got {mu_rec}"
    );
    // Sanity: the target differs from the start height, so there was real work.
    assert!(
        (z_tgt - z_start).abs() > 1e-6,
        "target should differ from the start height"
    );
}
