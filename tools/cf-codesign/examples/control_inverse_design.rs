//! Worked example — the POLICY half of the co-design loop (open-loop control).
//!
//! Given a target rigid behavior (the platen's height after an N-step coupled
//! soft↔rigid rollout), recover an open-loop control-force schedule that
//! produces it, by optimizing with the keystone's MULTI-step CONTROL gradient
//! `∂z_N/∂u_k` — one `tape.backward` across both engines and every step boundary.
//! This is the control analogue of `trajectory_inverse_design` (which recovers a
//! soft MATERIAL): the other half of the mission's "both design and policy". Run:
//!
//! ```text
//! cargo run -p cf-codesign --example control_inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use cf_codesign::{ControlScheduleTarget, OptConfig};

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

    // A reference schedule whose final platen height is the behavior to recover.
    let u_star: Vec<f64> = (0..n_steps)
        .map(|k| if k % 2 == 0 { -1.0 } else { 0.8 })
        .collect();
    let setup = ControlScheduleTarget::for_inverse_design(PLATEN_MJCF.to_string(), n_steps, 0.0);
    let target_z = setup.forward_z(&u_star);
    let z_zero = setup.forward_z(&vec![0.0; n_steps]);
    println!(
        "Target behavior: platen z_N = {target_z:.6} after {n_steps} coupled steps \
         (the do-nothing height is {z_zero:.6}; the schedule must close the {:.2e} gap)",
        target_z - z_zero
    );

    let target =
        ControlScheduleTarget::for_inverse_design(PLATEN_MJCF.to_string(), n_steps, target_z);
    println!("Optimizing the control schedule from u₀ = 0 ...\n");

    // z_N is a position: ∂z_N/∂u_k ~ 1e-5, so the raw loss gradient is below Adam's
    // standard eps (1e-8) and the optimizer would crawl. Condition with `Normalized`
    // — a dimensionless residual (loss_scale = 1/L²). Control forces are SIGNED, so
    // (unlike the material μ) the log-space lever does not apply: loss_scale only.
    let problem = target.recommended_normalized(1e-3);
    // Standard eps; a control-appropriate lr (an O(1)-newton step, not the default
    // physical-μ step), and a deep iteration budget for the weak sensitivity.
    let cfg = OptConfig {
        lr: 0.02,
        max_iters: 400,
        loss_tol: 1e-18,
        ..OptConfig::default()
    };
    let result = problem.optimize(&vec![0.0; n_steps], &cfg);

    for (k, rec) in result.history.iter().enumerate() {
        if k % 40 == 0 {
            println!(
                "  iter {k:3}: loss(norm) = {:.3e}   |grad|(norm) = {:.3e}",
                rec.loss, rec.grad_inf,
            );
        }
    }

    let z_final = target.forward_z(&result.params);
    println!("\nRecovered schedule (N):");
    for (k, u) in result.params.iter().enumerate() {
        println!("  u_{k} = {u:>8.4}");
    }
    println!(
        "\nDriven platen z_N = {z_final:.6} (target {target_z:.6}, |Δ| = {:.2e}) in {} iters; \
         converged = {}",
        (z_final - target_z).abs(),
        result.iters,
        result.converged,
    );
    println!(
        "(The schedule is under-determined — {n_steps} controls for one target — so this is a \
         schedule that achieves the target behavior, not the unique reference u*.)"
    );
}
