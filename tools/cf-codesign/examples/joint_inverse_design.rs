//! Worked example — the JOINT design+policy co-design loop (the mission's "both").
//!
//! Recover BOTH a soft material design variable `μ` AND a closed-loop feedback
//! policy `u_k = π_θ(state_k)` that together drive the platen to a target height
//! after a coupled soft↔rigid rollout — optimizing both axes in ONE outer loop, the
//! gradient `(∂z_N/∂μ_total, ∂z_N/∂θ)` read from a single
//! `StaggeredCoupling::coupled_trajectory_joint_gradient` backward. This is the
//! literal realization of `MISSION.md`'s "one outer loop differentiating w.r.t.
//! both design AND policy parameters". Run:
//!
//! ```text
//! cargo run -p cf-codesign --example joint_inverse_design
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use cf_codesign::{JointTarget, OptConfig};

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
    let n_steps = 10;

    // A reference DESIGN (stiffer block) + POLICY whose rollout height is the target.
    let mu_star = 4.0e4;
    let theta_star = [-15.0_f64, -3.0, 1.2];
    let setup = JointTarget::linear_for_inverse_design(PLATEN_MJCF.to_string(), n_steps, 0.0);
    let target_z = setup.forward_z(mu_star, &theta_star);

    // Start far from the reference in BOTH axes: a softer block, a zero policy.
    let mu0 = 2.0e4;
    let theta0 = [0.0_f64; 3];
    let z_start = setup.forward_z(mu0, &theta0);
    println!(
        "Target behavior: platen z_N = {target_z:.6} after {n_steps} coupled steps.\n\
         Start: μ = {mu0:.0} (μ* = {mu_star:.0}), θ = {theta0:?} → z_N = {z_start:.6} \
         (close the {:.2e} gap by moving BOTH μ and θ)\n",
        target_z - z_start
    );

    let problem =
        JointTarget::linear_for_inverse_design(PLATEN_MJCF.to_string(), n_steps, target_z);
    println!("Optimizing the JOINT vector p = [ln μ, w_z, w_vz, b] in one outer loop ...\n");

    // The target works in p = [ln μ, θ]: the positive μ is log-reparametrized inside
    // (relative steps, μ > 0 by construction), the signed θ are linear — so a single
    // loss_scale-conditioned Adam loop with one lr drives both axes. Standard eps.
    let normalized = problem.recommended_normalized(1e-3);
    let cfg = OptConfig {
        lr: 0.03,
        max_iters: 800,
        loss_tol: 1e-18,
        ..OptConfig::default()
    };
    let result = normalized.optimize(&problem.x0(mu0, &theta0), &cfg);

    for (k, rec) in result.history.iter().enumerate() {
        if k % 80 == 0 {
            println!(
                "  iter {k:3}: loss(norm) = {:.3e}   |grad|(norm) = {:.3e}",
                rec.loss, rec.grad_inf,
            );
        }
    }

    let (mu_rec, th_rec) = problem.to_physical(&result.params);
    let z_final = problem.forward_z(mu_rec, &th_rec);
    println!("\nRecovered JOINT design+policy:");
    println!("  μ    = {mu_rec:>9.1}   (design variable; started at {mu0:.0}, μ* = {mu_star:.0})");
    println!("  w_z  = {:>9.4}   (policy: proportional)", th_rec[0]);
    println!("  w_vz = {:>9.4}   (policy: derivative)", th_rec[1]);
    println!("  b    = {:>9.4}   (policy: bias)", th_rec[2]);
    println!(
        "\nDriven platen z_N = {z_final:.6} (target {target_z:.6}, |Δ| = {:.2e}) in {} iters; \
         converged = {}",
        (z_final - target_z).abs(),
        result.iters,
        result.converged,
    );
    println!(
        "(1 design + 3 policy params for one scalar target is under-determined — this is a \
         (μ, θ) pair that achieves the target behavior, not the unique reference. Both axes \
         moved: the design μ shifted off its start AND the policy was tuned, in one backward.)"
    );
}
