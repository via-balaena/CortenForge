//! cf-codesign de-escalation RQ2 gate — the impact co-design loop: inverse design of a
//! protective buffer's stiffness to a PEAK-CONTACT-FORCE (recoverability) spec.
//!
//! Given a target peak strike force `F* = peak_force(μ*)` — a recoverability spec, e.g. below the
//! ISO/TS 15066 ~270 N transient line — the optimizer recovers the buffer stiffness `μ*` that
//! produces it, driving the chassis Adam with the keystone's PEAK-force trajectory gradient
//! `d(peak |fz|)/dμ` (one `tape.backward` from the peak step's contact-force node), the first
//! consumer of `coupled_trajectory_peak_force_gradient`.
//!
//! Two checks: (1) the raw problem's analytic gradient matches a central FD of its peak-force loss
//! (the consumed gradient is correct for the objective, machine-exact); (2) the [`Normalized`]-
//! wrapped optimizer recovers `μ*` to tolerance with the standard `eps` and reports `converged`.
//!
//! Conditioning note. Unlike the position-outcome targets (`z_N ~ 1e-7`, gradient eps-dominated),
//! peak force is `O(100 N)` with a well-scaled gradient — there is no tiny-gradient problem here.
//! `μ` is still a large positive scale, so the recovery wraps the target in [`Normalized`] with
//! `log_space = true` purely for relative (scale-free) steps and a structural `μ > 0`.

#![allow(clippy::expect_used)]

use cf_codesign::{CoDesignProblem, Normalized, PeakForceTarget, StopReason};

// The de-escalation impact fixture: a 1 kg limb at the contact-band top (z = 0.115), struck
// downward into a pinned soft buffer (block top z = 0.10). Mirrors the RQ1 scenario.
const LIMB_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="limb" pos="0 0 0.115">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>"#;

const V_IMPACT: f64 = 2.0;
// The peak strike force lands at step ~2–3 (the first compression); 25 steps is comfortably
// past it while keeping the 200+-iter recovery within the heavy-test budget (the full rollout
// is rebuilt twice per evaluate, hundreds of iters).
const N_STEPS: usize = 25;

#[test]
fn peak_force_gradient_matches_fd() {
    // Target 0 ⇒ loss = ½·|peak|², gradient = |peak|·∂|peak|/∂μ — isolates the gradient check.
    let p = PeakForceTarget::for_impact_design(LIMB_MJCF.to_string(), V_IMPACT, N_STEPS, 0.0);
    let mu = 3.5e4;
    let (_loss, grad) = p.evaluate(&[mu]);

    // FD of the loss along μ (the rebuild follows λ = 4μ — the same line the analytic total
    // ∂/∂μ + 4·∂/∂λ differentiates).
    let eps = mu * 1e-6;
    let loss_at = |m: f64| p.evaluate(&[m]).0;
    let fd = (loss_at(mu + eps) - loss_at(mu - eps)) / (2.0 * eps);

    let rel = (grad[0] - fd).abs() / fd.abs();
    eprintln!(
        "peak-force dLoss/dμ: analytic={:.6e}  FD={fd:.6e}  rel={rel:.3e}",
        grad[0]
    );
    // Machine-exact (~1e-9 in the spike) — gate with margin for cross-platform float drift.
    assert!(
        rel < 1e-5,
        "peak-force problem gradient {} disagrees with FD {fd} (rel {rel:e})",
        grad[0],
    );
    // Genuinely nonzero (a no-contact rollout would leave peak μ-independent ⇒ gradient 0).
    assert!(
        grad[0].abs() > 1e-9,
        "gradient ~0 — did the strike engage contact?"
    );
}

#[test]
fn recovers_known_stiffness_from_target_peak_force() {
    let mu_star = 4.0e4;

    // The recoverability spec = the peak strike force the buffer transmits at μ*.
    let setup = PeakForceTarget::for_impact_design(LIMB_MJCF.to_string(), V_IMPACT, N_STEPS, 0.0);
    let target_force = setup.forward_peak_force(mu_star);
    assert!(target_force > 0.0, "degenerate: no contact force at μ*");

    let target =
        PeakForceTarget::for_impact_design(LIMB_MJCF.to_string(), V_IMPACT, N_STEPS, target_force);

    // log-μ relative steps (μ is a large positive scale); residual scale L = the target force so
    // (peak − target)/L is dimensionless. Standard eps — no tiny-gradient band-aid needed here.
    let problem = Normalized::with_residual_scale(&target, target_force, true);
    let cfg = problem.recommended_config();

    let mu0 = 2.0e4;
    let result = problem.optimize(&[mu0], &cfg);
    let mu = result.params[0];
    let rel_mu = (mu - mu_star).abs() / mu_star;
    eprintln!(
        "peak-force inverse design: μ₀={mu0} → μ={mu:.4} (μ*={mu_star}) rel={rel_mu:.3e}  \
         target_force={target_force:.2} N  loss={:.3e}  iters={}  stop={:?}",
        result.loss, result.iters, result.stop_reason,
    );

    assert_ne!(
        result.stop_reason,
        StopReason::MaxIters,
        "optimizer did not converge in max_iters"
    );
    assert!(
        rel_mu < 1e-3,
        "did not recover μ*: μ={mu} μ*={mu_star} (rel {rel_mu:e})"
    );
    // The descent made real progress (loss dropped by orders).
    let first = result.history.first().expect("history non-empty");
    assert!(
        first.loss > 1e3 * result.loss,
        "loss barely moved: first={:e} final={:e}",
        first.loss,
        result.loss,
    );
}

/// Backs the load-bearing modeling claim that makes target-inverse-design (objective A) the
/// honest choice over unconstrained "minimize peak force": in this scene peak force is MONOTONE
/// increasing in buffer stiffness (a softer buffer is always gentler — no bottoming/U-shape), so
/// minimizing peak force alone would trivially drive μ → floor, while a peak-force *target* has a
/// unique μ*. Forward-only (no gradient), so it is cheap. Also the de-escalation premise itself:
/// softer absorbs better.
#[test]
fn peak_force_is_monotone_in_stiffness() {
    let p = PeakForceTarget::for_impact_design(LIMB_MJCF.to_string(), V_IMPACT, N_STEPS, 0.0);
    // A wide stiffness sweep spanning the Ecoflex→firm range (kPa-scale μ).
    let mus = [4.0e3_f64, 8.0e3, 1.6e4, 3.2e4, 6.4e4];
    let forces: Vec<f64> = mus.iter().map(|&m| p.forward_peak_force(m)).collect();
    eprintln!("peak-force(μ) sweep: {mus:?} → {forces:?}");
    for w in forces.windows(2) {
        assert!(
            w[1] > w[0],
            "peak force not monotone increasing in μ: {forces:?} (a softer buffer should be \
             gentler — if this fails the objective-A rationale and the de-escalation premise break)"
        );
    }
    // Non-degenerate: contact genuinely engaged across the sweep (forces are real, not ~0).
    assert!(
        forces[0] > 1.0,
        "no contact force at the softest μ: {forces:?}"
    );
}
