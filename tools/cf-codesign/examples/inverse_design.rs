//! Worked example — the first closed co-design loop (inverse design).
//!
//! Given a target rigid behavior (a platen's next-step velocity), recover the
//! soft material that produces it, by optimizing with the gradient that crosses
//! the differentiable soft↔rigid coupling. Run:
//!
//! ```text
//! cargo run -p cf-codesign --example inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use cf_codesign::{OptConfig, SoftMaterialTarget, optimize};

const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

fn main() {
    let height = 0.099;
    let mu_star = 4.0e4_f64;

    // The target is the rigid outcome a reference soft material produces.
    let setup = SoftMaterialTarget::for_inverse_design(PLATEN_MJCF.to_string(), height, 0.0);
    let target_vz = setup.forward_vz(mu_star);
    println!("Target behavior: platen vz' = {target_vz:.6} (produced by μ* = {mu_star})");

    let problem =
        SoftMaterialTarget::for_inverse_design(PLATEN_MJCF.to_string(), height, target_vz);
    let mu0 = 2.0e4;
    println!("Optimizing soft stiffness from μ₀ = {mu0} (λ = 4μ) ...\n");

    let result = optimize(&problem, &[mu0], &OptConfig::default());

    for (k, rec) in result.history.iter().enumerate() {
        if k % 20 == 0 {
            println!(
                "  iter {k:3}: μ = {:>9.2}   loss = {:.3e}   |grad| = {:.3e}",
                rec.params[0], rec.loss, rec.grad_inf,
            );
        }
    }
    println!(
        "\nRecovered μ = {:.3} (μ* = {mu_star}, rel error {:.2e}) in {} iters; converged = {}",
        result.params[0],
        (result.params[0] - mu_star).abs() / mu_star,
        result.iters,
        result.converged(),
    );
}
