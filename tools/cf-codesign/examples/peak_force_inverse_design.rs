//! Worked example — the de-escalation RQ2 impact co-design loop.
//!
//! Given a peak-contact-force recoverability spec (e.g. keep a limb strike below the ISO/TS 15066
//! ~270 N transient line), recover the soft buffer stiffness that meets it — optimizing with the
//! keystone's PEAK-force trajectory gradient `d(peak |fz|)/dμ`. Run:
//!
//! ```text
//! cargo run -p cf-codesign --example peak_force_inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use cf_codesign::{Normalized, PeakForceTarget};

// A 1 kg limb at the contact-band top (z = 0.115), struck downward into a pinned soft buffer.
const LIMB_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="limb" pos="0 0 0.115">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>"#;

fn main() {
    let v_impact = 2.0_f64;
    let n_steps = 25;

    // The recoverability spec = the peak strike force a reference soft buffer (μ* ≈ 10 kPa, an
    // Ecoflex-grade silicone) transmits — comfortably below the ISO ~270 N line. We then recover
    // it starting from a too-firm buffer, the de-escalation direction (soften until recoverable).
    let mu_star = 1.0e4_f64;
    let setup = PeakForceTarget::for_impact_design(LIMB_MJCF.to_string(), v_impact, n_steps, 0.0);
    let target_force = setup.forward_peak_force(mu_star);

    println!(
        "De-escalation RQ2: find the soft buffer stiffness whose peak strike force = {target_force:.1} N\n\
         (a 1 kg limb at {v_impact} m/s; ISO recoverable line ~270 N, fracture cliff ~450 N)\n"
    );

    let target =
        PeakForceTarget::for_impact_design(LIMB_MJCF.to_string(), v_impact, n_steps, target_force);
    let mu0 = 4.0e4;
    println!(
        "Starting buffer: μ₀ = {mu0} Pa → peak force {:.1} N (too firm)",
        target.forward_peak_force(mu0)
    );
    println!("Optimizing buffer stiffness (λ = 4μ) ...\n");

    // Peak force is well-scaled (O(100 N)), so no tiny-gradient issue; log-μ gives relative
    // (scale-free) steps and a structural μ > 0. Residual scale L = the target force.
    let problem = Normalized::with_residual_scale(&target, target_force, true);
    let cfg = problem.recommended_config();
    let result = problem.optimize(&[mu0], &cfg);

    for (k, rec) in result.history.iter().enumerate() {
        if k % 40 == 0 {
            println!(
                "  iter {k:3}: μ = {:>9.2} Pa   loss(norm) = {:.3e}   |grad|(norm) = {:.3e}",
                rec.params[0], rec.loss, rec.grad_inf,
            );
        }
    }
    let mu = result.params[0];
    println!(
        "\nRecovered buffer μ = {mu:.1} Pa → peak force {:.1} N (target {target_force} N) \
         in {} iters; converged = {}",
        target.forward_peak_force(mu),
        result.iters,
        result.converged(),
    );
    println!("The differentiable co-design loop tuned the buffer to meet the recoverability spec.");
}
