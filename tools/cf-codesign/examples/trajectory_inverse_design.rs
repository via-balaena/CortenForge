//! Worked example — the trajectory co-design loop (inverse design over a rollout).
//!
//! Given a target rigid behavior (the platen's height after an N-step
//! contact-engaged coupled rollout), recover the soft material that produces it, by optimizing
//! with the keystone's MULTI-step gradient that crosses the differentiable
//! soft↔rigid coupling AND every step boundary. Run:
//!
//! ```text
//! cargo run -p cf-codesign --example trajectory_inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use cf_codesign::{Normalized, SoftMaterialTrajectoryTarget};

// Platen started already in contact so the rollout is engaged from step 0.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

fn main() {
    let n_steps = 20;
    let mu_star = 4.0e4_f64;

    // The target is the platen's height after the rollout at a reference material.
    let setup =
        SoftMaterialTrajectoryTarget::for_inverse_design(PLATEN_MJCF.to_string(), n_steps, 0.0);
    let target_z = setup.forward_z(mu_star);
    println!(
        "Target behavior: platen z_N = {target_z:.6} after {n_steps} coupled steps \
         (produced by μ* = {mu_star})"
    );

    let target = SoftMaterialTrajectoryTarget::for_inverse_design(
        PLATEN_MJCF.to_string(),
        n_steps,
        target_z,
    );
    let mu0 = 2.0e4;
    println!("Optimizing soft stiffness from μ₀ = {mu0} (λ = 4μ) ...\n");

    // z_N is a position: ∂z_N/∂μ ~ 1e-7, so the raw loss gradient ~2e-10 is below
    // Adam's standard eps (1e-8) and the optimizer would crawl. Condition the
    // objective with `Normalized` — a dimensionless residual (L = 0.1 m block edge)
    // plus log-μ relative steps — so the STANDARD eps keeps its scale-invariance.
    let problem = Normalized::with_residual_scale(&target, 0.1, true);
    let cfg = problem.recommended_config();
    // `optimize` brackets the physical↔log-μ mapping: x0 and result/history params
    // are all in physical μ units.
    let result = problem.optimize(&[mu0], &cfg);

    // μ is physical; loss/|grad| are in the NORMALIZED (dimensionless / log-μ) space
    // the optimizer worked in.
    for (k, rec) in result.history.iter().enumerate() {
        if k % 40 == 0 {
            println!(
                "  iter {k:3}: μ = {:>9.2}   loss(norm) = {:.3e}   |grad|(norm) = {:.3e}",
                rec.params[0], rec.loss, rec.grad_inf,
            );
        }
    }
    let mu = result.params[0];
    println!(
        "\nRecovered μ = {mu:.3} (μ* = {mu_star}, rel error {:.2e}) in {} iters; stop = {:?}",
        (mu - mu_star).abs() / mu_star,
        result.iters,
        result.stop_reason,
    );
}
