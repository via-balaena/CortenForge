//! cf-codesign v1 gate — the first closed co-design loop: inverse design.
//!
//! Given a target rigid behavior `vz* = vz'(μ*)`, the optimizer recovers the
//! soft material `μ*` that produces it — driving the chassis Adam with the
//! keystone gradient `d vz'/d μ` (the total along the `λ = 4μ` stiffness line)
//! that crosses the differentiable soft↔rigid coupling.
//!
//! Two checks: (1) the problem's analytic gradient matches a finite difference
//! of its loss (so the co-design gradient is correct for the objective, not just
//! a descent direction); (2) the optimizer recovers `μ*` to tolerance and
//! reports `converged`.

#![allow(clippy::expect_used)]

use cf_codesign::{CoDesignProblem, OptConfig, SoftMaterialTarget, StopReason, optimize};

const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const HEIGHT: f64 = 0.099; // deeply-engaged: 25 top vertices active, stable

#[test]
fn problem_gradient_matches_fd() {
    // A target is irrelevant to the gradient-of-loss check; use 0 so the loss
    // is ½·vz'² and its gradient is vz'·(d vz'/d μ).
    let p = SoftMaterialTarget::for_inverse_design(PLATEN_MJCF.to_string(), HEIGHT, 0.0);
    let mu = 3.5e4;
    let (_loss, grad) = p.evaluate(&[mu]);

    // FD of the loss along μ (the rebuild follows λ = 4μ — the same line the
    // analytic total ∂/∂μ + 4·∂/∂λ differentiates).
    let eps = mu * 1e-6;
    let loss_at = |m: f64| p.evaluate(&[m]).0;
    let fd = (loss_at(mu + eps) - loss_at(mu - eps)) / (2.0 * eps);

    let rel = (grad[0] - fd).abs() / fd.abs();
    eprintln!(
        "dLoss/dμ: analytic={:.6e}  FD={fd:.6e}  rel={rel:.3e}",
        grad[0]
    );
    assert!(
        rel < 1e-5,
        "problem gradient {} disagrees with FD {fd} (rel {rel:e})",
        grad[0],
    );
}

#[test]
fn recovers_known_material_from_target_behavior() {
    let mu_star = 4.0e4;

    // The target behavior = the rigid outcome the reference material produces.
    let setup = SoftMaterialTarget::for_inverse_design(PLATEN_MJCF.to_string(), HEIGHT, 0.0);
    let target_vz = setup.forward_vz(mu_star);

    // The inverse-design problem: find the material that yields target_vz.
    let problem =
        SoftMaterialTarget::for_inverse_design(PLATEN_MJCF.to_string(), HEIGHT, target_vz);

    // Optimize from a different stiffness.
    let mu0 = 2.0e4;
    let result = optimize(&problem, &[mu0], &OptConfig::default());

    let mu = result.params[0];
    let rel_mu = (mu - mu_star).abs() / mu_star;
    eprintln!(
        "inverse design: μ₀={mu0} → μ={mu:.3} (μ*={mu_star}) rel={rel_mu:.3e}  \
         loss={:.3e}  iters={}  stop={:?}",
        result.loss, result.iters, result.stop_reason,
    );

    assert_ne!(
        result.stop_reason,
        StopReason::MaxIters,
        "optimizer did not converge in max_iters"
    );
    assert!(
        result.loss < 1e-10,
        "final loss {} not below tol",
        result.loss
    );
    assert!(
        rel_mu < 1e-3,
        "did not recover μ*: μ={mu} μ*={mu_star} (rel {rel_mu:e})",
    );
    // The descent made real progress (the trajectory's loss dropped by orders).
    let first_loss = result.history.first().expect("history non-empty").loss;
    assert!(
        first_loss > 1e3 * result.loss,
        "loss barely moved: first={first_loss:e} final={:e}",
        result.loss,
    );
}
