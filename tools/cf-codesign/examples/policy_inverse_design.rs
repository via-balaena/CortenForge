//! Worked example — the CLOSED-LOOP policy half of the co-design loop.
//!
//! Given a target rigid behavior (the platen's height after an N-step coupled
//! soft↔rigid rollout), recover a state-**feedback** policy `u_k = π_θ(state_k)`
//! that produces it, by optimizing the policy weights θ with the keystone's
//! closed-loop policy gradient `∂z_N/∂θ` — one `tape.backward` that backpropagates
//! through time across the state→control recurrence. Unlike the open-loop
//! `control_inverse_design` (a free schedule `u_0 … u_{N−1}`), here the control at
//! each step is a function of the platen state, and the optimized parameters are
//! the *shared* policy weights. (Optimizing design and policy *jointly* in one
//! outer loop is a documented follow-on.) Run:
//!
//! ```text
//! cargo run -p cf-codesign --example policy_inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use cf_codesign::{FeedbackPolicyTarget, OptConfig};

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
    let n_steps = 8;

    // A reference feedback policy u = w_z·z + w_vz·vz + b whose final platen height
    // is the behavior to recover (restoring + velocity damping + bias).
    let theta_star = [-15.0_f64, -3.0, 1.2];
    let setup =
        FeedbackPolicyTarget::linear_for_inverse_design(PLATEN_MJCF.to_string(), n_steps, 0.0);
    let target_z = setup.forward_z(&theta_star);
    let z_zero = setup.forward_z(&[0.0; 3]);
    println!(
        "Target behavior: platen z_N = {target_z:.6} after {n_steps} coupled steps \
         (the do-nothing height is {z_zero:.6}; the policy must close the {:.2e} gap)",
        target_z - z_zero
    );

    let target =
        FeedbackPolicyTarget::linear_for_inverse_design(PLATEN_MJCF.to_string(), n_steps, target_z);
    println!("Optimizing the feedback policy θ = [w_z, w_vz, b] from θ₀ = 0 ...\n");

    // z_N is a position: ∂z_N/∂θ ~ 1e-5..1e-4, so the raw loss gradient is near/below
    // Adam's standard eps (1e-8) and the optimizer would crawl. Condition with
    // `Normalized` — a dimensionless residual (loss_scale = 1/L²). The linear policy's
    // weights are SIGNED, so (unlike the material μ) the log-space lever does not
    // apply: loss_scale only.
    let problem = target.recommended_normalized(1e-3);
    let cfg = OptConfig {
        lr: 0.02,
        max_iters: 600,
        loss_tol: 1e-18,
        ..OptConfig::default()
    };
    let result = problem.optimize(&[0.0; 3], &cfg);

    for (k, rec) in result.history.iter().enumerate() {
        if k % 60 == 0 {
            println!(
                "  iter {k:3}: loss(norm) = {:.3e}   |grad|(norm) = {:.3e}",
                rec.loss, rec.grad_inf,
            );
        }
    }

    let z_final = target.forward_z(&result.params);
    let [w_z, w_vz, b] = [result.params[0], result.params[1], result.params[2]];
    println!("\nRecovered feedback policy  u_k = w_z·z_k + w_vz·vz_k + b :");
    println!("  w_z  = {w_z:>9.4}   (proportional)");
    println!("  w_vz = {w_vz:>9.4}   (derivative)");
    println!("  b    = {b:>9.4}   (bias)");
    println!(
        "\nDriven platen z_N = {z_final:.6} (target {target_z:.6}, |Δ| = {:.2e}) in {} iters; \
         converged = {}",
        (z_final - target_z).abs(),
        result.iters,
        result.converged(),
    );
    println!(
        "(3 policy weights for one scalar target is under-determined — this is a \
         feedback policy that achieves the target behavior, not the unique reference θ*.)"
    );
}
