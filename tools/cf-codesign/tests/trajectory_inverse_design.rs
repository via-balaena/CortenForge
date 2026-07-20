//! cf-codesign v2 gate — the trajectory co-design loop: inverse design over a
//! multi-step coupled rollout.
//!
//! Given a target rigid behavior `z* = z_N(μ*)` (the platen's height after an
//! N-step contact-engaged coupled rollout), the optimizer recovers the soft material
//! `μ*` that produces it — driving the chassis Adam with the keystone's
//! MULTI-step gradient `d z_N/d μ` (one `tape.backward` across both engines and
//! every step boundary), the first consumer of `coupled_trajectory_material_gradient`.
//!
//! Three checks: (1) the raw problem's analytic gradient matches a central FD of
//! its trajectory loss (the consumed keystone gradient is correct for the
//! objective, machine-exact); (2) the [`Normalized`]-wrapped gradient matches an
//! FD in the optimizer's (log-μ) space (the `1/L²` loss-scale and `dμ/dp = μ`
//! chain-rule bookkeeping is correct); (3) the optimizer recovers `μ*` to
//! tolerance **with the standard `eps = 1e-8`** and reports `converged`.
//!
//! **Conditioning note.** `z_N` is a position, so `∂z_N/∂μ ~ 1e-7` and the raw
//! loss gradient is `~2e-10` — below Adam's standard `eps = 1e-8`, so optimizing
//! the raw target crawls. The recovery run therefore wraps the target in
//! [`Normalized`] (a dimensionless residual + log-μ relative steps), which lifts
//! the gradient above the standard `eps` — no special `eps` needed. See
//! `docs/codesign/recon.md` §v3.

#![allow(clippy::expect_used)]

use cf_codesign::{CoDesignProblem, Normalized, OptConfig, SoftMaterialTrajectoryTarget, optimize};

// Platen started already in contact (plane at z − clearance = 0.103 vs the soft
// block's top face at z = 0.1) so the rollout is engaged from step 0 (it deepens
// and settles, it does not break contact) — the keystone trajectory gate's scene.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const N_STEPS: usize = 20; // engaged; slope dz_N/dμ ~ 1e-7, well clear of settling

// A platen started ABOVE contact: it falls under gravity, makes contact, rebounds
// and BREAKS contact, then re-makes — a genuine make/break/re-make (bounce) the
// inverse-design scene above (engaged from step 0) never exercises. Traced active
// set over the 80-step rollout: no contact ~steps 0–63, engaged ~64–67, broken
// ~68–77 (rebound), re-engaged ~78–79. Damping 8 (vs the engaged scene's 60) lets
// the platen actually descend into contact; matches the keystone/IPC make/break
// gates' fixture.
const MAKE_CONTACT_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

#[test]
fn trajectory_gradient_matches_fd() {
    // Target irrelevant to the gradient-of-loss check; use 0 so the loss is
    // ½·z_N² and its gradient is z_N·(d z_N/d μ).
    let p = SoftMaterialTrajectoryTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, 0.0);
    let mu = 3.5e4;
    let (_loss, grad) = p.evaluate(&[mu]);

    // FD of the loss along μ (the rebuild follows λ = 4μ — the same line the
    // analytic total ∂/∂μ + 4·∂/∂λ differentiates).
    let eps = mu * 1e-6;
    let loss_at = |m: f64| p.evaluate(&[m]).0;
    let fd = (loss_at(mu + eps) - loss_at(mu - eps)) / (2.0 * eps);

    let rel = (grad[0] - fd).abs() / fd.abs();
    eprintln!(
        "trajectory dLoss/dμ: analytic={:.6e}  FD={fd:.6e}  rel={rel:.3e}",
        grad[0]
    );
    // Machine-exact (~1e-8) — the keystone trajectory gradient is consumed
    // correctly through N steps; gate with margin for cross-platform float drift.
    assert!(
        rel < 1e-5,
        "trajectory problem gradient {} disagrees with FD {fd} (rel {rel:e})",
        grad[0],
    );
}

/// The [`Normalized`] wrapper's gradient (in the optimizer's log-μ space) matches
/// a central FD of the *normalized* loss in that space — confirming the
/// chain-rule bookkeeping: the `1/L²` loss-scale and the `dμ/dp = μ` log factor.
/// This is the new surface v3 adds; the raw keystone gradient is gated above.
#[test]
fn normalized_gradient_matches_fd() {
    // Residual scale L = the 0.1 m block edge; log-μ relative steps.
    let target =
        SoftMaterialTrajectoryTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, 0.0);
    let problem = Normalized::with_residual_scale(&target, 0.1, true);

    let mu = 3.5e4;
    let p = problem.to_normalized(&[mu]); // log-μ space
    let (_loss, grad) = problem.evaluate(&p);

    // FD of the normalized loss in the SAME (log-μ) space the gradient lives in.
    let eps = 1e-6;
    let loss_at = |dp: f64| problem.evaluate(&[p[0] + dp]).0;
    let fd = (loss_at(eps) - loss_at(-eps)) / (2.0 * eps);

    let rel = (grad[0] - fd).abs() / fd.abs();
    eprintln!(
        "normalized dLoss/d(lnμ): analytic={:.6e}  FD={fd:.6e}  rel={rel:.3e}",
        grad[0]
    );
    assert!(
        rel < 1e-5,
        "normalized gradient {} disagrees with FD {fd} (rel {rel:e})",
        grad[0],
    );

    // Also exercise the log_space = false branch (dx/dp = 1.0, loss-scale only):
    // the gradient must match an FD of the same scaled loss in PHYSICAL μ-space.
    let linear = Normalized::new(&target, 1.0 / (0.1 * 0.1), false);
    let (_, lg) = linear.evaluate(&[mu]);
    let le = mu * 1e-6;
    let lfd = (linear.evaluate(&[mu + le]).0 - linear.evaluate(&[mu - le]).0) / (2.0 * le);
    let lrel = (lg[0] - lfd).abs() / lfd.abs();
    eprintln!(
        "linear (loss-scale only) dLoss/dμ: analytic={:.6e}  FD={lfd:.6e}  rel={lrel:.3e}",
        lg[0]
    );
    assert!(
        lrel < 1e-5,
        "linear-branch gradient {} disagrees with FD {lfd} (rel {lrel:e})",
        lg[0]
    );
}

#[test]
fn recovers_known_material_from_target_trajectory() {
    let mu_star = 4.0e4;

    // The target behavior = the platen's height after the rollout at μ*.
    let setup =
        SoftMaterialTrajectoryTarget::for_inverse_design(PLATEN_MJCF.to_string(), N_STEPS, 0.0);
    let target_z = setup.forward_z(mu_star);

    let target = SoftMaterialTrajectoryTarget::for_inverse_design(
        PLATEN_MJCF.to_string(),
        N_STEPS,
        target_z,
    );

    // NEGATIVE CONTROL — normalization is load-bearing, not incidental: the RAW
    // target with the STANDARD eps does NOT recover (the ~2e-10 gradient is
    // eps-dominated and Adam crawls). Bounded iters keep it cheap; the spike's
    // 300-iter run reached only rel 0.26.
    let mu0 = 2.0e4;
    let crawl = optimize(
        &target,
        &[mu0],
        &OptConfig {
            max_iters: 80,
            ..OptConfig::default()
        },
    );
    let crawl_rel = (crawl.params[0] - mu_star).abs() / mu_star;
    eprintln!("negative control (raw + standard eps, 80 iters): rel={crawl_rel:.3e}");
    assert!(
        crawl_rel > 1e-2,
        "raw target with standard eps unexpectedly converged (rel {crawl_rel:e}) — \
         the negative control is meant to crawl",
    );

    // Condition the (weakly-sensitive) objective: a dimensionless residual
    // (L = 0.1 m block edge) + log-μ relative steps. `recommended_config` uses the
    // STANDARD `eps = 1e-8` — the v3 point: normalization, not a tiny per-scene eps.
    let problem = Normalized::with_residual_scale(&target, 0.1, true);
    let cfg = problem.recommended_config();
    assert_eq!(
        cfg.eps,
        OptConfig::default().eps,
        "must use the standard eps"
    );

    // The bracketing entry point: x0 and result.params are in PHYSICAL μ units.
    let result = problem.optimize(&[mu0], &cfg);
    let mu = result.params[0];

    // The bracketing remap covers history too: the first record starts at μ₀ (a
    // physical ~2e4), NOT its log-space image (~10) — pins the history remap loop.
    let first = result.history.first().expect("history non-empty");
    assert!(
        (first.params[0] - mu0).abs() < 1.0,
        "history params not mapped to physical units: {} (expected ≈ μ₀ {mu0})",
        first.params[0],
    );

    let rel_mu = (mu - mu_star).abs() / mu_star;
    eprintln!(
        "trajectory inverse design (standard eps): μ₀={mu0} → μ={mu:.4} (μ*={mu_star}) \
         rel={rel_mu:.3e}  loss={:.3e}  iters={}  converged={}",
        result.loss,
        result.iters,
        result.converged(),
    );

    assert!(
        result.converged(),
        "optimizer did not converge in max_iters"
    );
    // Robust gate (the measured recovery is far tighter, ~1e-6 or better); loose
    // here so cross-OS float drift in the coupled rollout can't flake it.
    assert!(
        rel_mu < 1e-3,
        "did not recover μ*: μ={mu} μ*={mu_star} (rel {rel_mu:e})",
    );
    // The descent made real progress (the loss dropped by orders).
    assert!(
        first.loss > 1e3 * result.loss,
        "loss barely moved: first={:e} final={:e}",
        first.loss,
        result.loss,
    );
}

/// Showcase — the co-design gradient stays machine-exact when the N-step rollout
/// crosses contact MAKE/BREAK events. The platen starts above contact (z = 0.125),
/// falls under gravity, makes contact (~step 64 of 80), rebounds and BREAKS contact
/// (~steps 68–77), then re-makes (~step 78) — two onsets and a release the
/// inverse-design scene above (engaged from step 0) never exercises. The
/// gradient-of-objective vs a central FD of the same objective stays machine-exact
/// (~4e-8) across all of them.
///
/// Scope: this carries the *gradient-correctness-through-make/break* proof; the
/// engaged scene above carries the *convergence* proof (a full recovery here would
/// need ~80-step rollouts × hundreds of iters — minutes). The substrate's
/// independent make/break validation (vs a re-rolled FD oracle) lives in
/// `sim-coupling`'s `coupled_trajectory_gradient.rs` and the `ipc-traj·material` rows of
/// its `coupling_grad_harness.rs`.
#[test]
fn trajectory_gradient_matches_fd_through_make_break() {
    let n = 80;
    let p = SoftMaterialTrajectoryTarget::new(
        MAKE_CONTACT_MJCF.to_string(),
        1,
        0.005,
        4,
        0.1,
        1.0e-3,
        3.0e4,
        1.0e-2,
        8.0,
        n,
        0.0,
    );
    let mu = 3.5e4;
    let (_loss, grad) = p.evaluate(&[mu]);

    let eps = mu * 1e-6;
    let loss_at = |m: f64| p.evaluate(&[m]).0;
    let fd = (loss_at(mu + eps) - loss_at(mu - eps)) / (2.0 * eps);

    let rel = (grad[0] - fd).abs() / fd.abs();
    eprintln!(
        "through-make/break dLoss/dμ: analytic={:.6e}  FD={fd:.6e}  rel={rel:.3e}",
        grad[0]
    );
    // Proof contact really was made: z_N drops into the band (~0.1145) vs ~0.120 if
    // the platen never reached contact — a ~5 mm margin, drift-proof even if float
    // drift shifts the onset step a little. (Without it the rollout is a no-op and
    // the gradient is identically 0, the failure mode at too-small n.)
    assert!(
        p.forward_z(mu) < 0.115,
        "platen never made contact — z_N {} ≥ 0.115 (raise n)",
        p.forward_z(mu),
    );
    // The gradient is also genuinely nonzero through the make/break (a no-contact
    // rollout leaves z_N μ-independent ⇒ gradient exactly 0).
    assert!(grad[0].abs() > 1e-11, "gradient ~0 through make/break");
    assert!(
        rel < 1e-5,
        "gradient {} disagrees with FD {fd} through make/break (rel {rel:e})",
        grad[0],
    );
}
