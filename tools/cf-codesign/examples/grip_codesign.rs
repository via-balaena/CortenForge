//! Worked example — the de-escalation co-design loop on the soft↔rigid GRIP.
//!
//! The de-escalation study's first co-design *result*. A soft buffer holds a
//! *moving* limb: an actuated hinge arm whose finite sphere tip sweeps tangentially
//! into a soft block, gripped by dry friction. We co-design BOTH the buffer
//! stiffness `μ` AND a closed-loop holding policy `θ` so the gripped limb's
//! tangential drag `tip_x` after the coupled rollout reaches a RESTRAINT target —
//! holding the limb at a target POSITION. (This objective is `tip_x`, not a force
//! or recoverability metric: the soft buffer's recoverability is RQ1's separate
//! result, and a peak-force objective on the grip is a documented follow-on.) Both
//! gradient blocks `(∂tip_x/∂μ_total, ∂tip_x/∂θ)` come
//! from ONE `coupled_trajectory_design_policy_friction_gradient` backward — the
//! curvature-correct finite-contact gradient (PR #406, gated on the centroid sphere
//! by #430). Run:
//!
//! ```text
//! cargo run -p cf-codesign --example grip_codesign
//! ```

#![allow(clippy::expect_used, clippy::print_stdout)]

use cf_codesign::{GripCoDesignTarget, OptConfig};

// The de-escalation grip: an actuated hinge arm (the "limb"), its finite sphere tip
// pressed into the soft buffer under a sideways gravity that drives the tangential
// sweep — soft holding a MOVING limb, the keystone friction-grip scene.
//
// NOTE: the COLLISION sphere is the r = 0.08 collider set by
// `with_sphere_collider` (the contact physics); the MJCF geom `size="0.004"` only
// contributes the limb's (negligible) hinge inertia and is NOT the contact surface.
// The sphere is centroid-posed over the block, not tracked to this geom's tip (the
// moving-end-effector follow-on). This mirrors the upstream centroid gate fixture.
const GRIP_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;

fn main() {
    let n_steps = 10;

    // A reference DESIGN (buffer μ*) + holding POLICY θ* whose gripped drag is the
    // restraint target — a behavior we KNOW the grip can realize.
    let mu_star = 3.0e3;
    let theta_star = [0.0_f64, 0.0, 0.5];
    let setup = GripCoDesignTarget::linear_for_inverse_design(GRIP_MJCF.to_string(), n_steps, 0.0);
    let target_x = setup.forward_x(mu_star, &theta_star);

    // Start far from the reference in BOTH axes: a firmer buffer, a zero policy.
    let mu0 = 6.0e3;
    let theta0 = [0.0_f64; 3];
    let x_start = setup.forward_x(mu0, &theta0);
    println!(
        "De-escalation grip co-design: restrain the limb to tip_x = {target_x:.6} m after \
         {n_steps} coupled grip steps.\n\
         Start: μ = {mu0:.0} Pa (μ* = {mu_star:.0}), θ = {theta0:?} → tip_x = {x_start:.6} m \
         (close the {:.2e} m gap by moving BOTH the buffer μ and the holding policy θ)\n",
        target_x - x_start
    );

    let problem =
        GripCoDesignTarget::linear_for_inverse_design(GRIP_MJCF.to_string(), n_steps, target_x);
    println!("Optimizing the JOINT vector p = [ln μ, w_z, w_vz, b] in one outer loop ...\n");

    // p = [ln μ, θ]: the positive μ is log-reparametrized inside the target (relative
    // steps, μ > 0 by construction), the signed θ are linear — so one loss_scale-
    // conditioned Adam loop with one lr drives both axes. Residual scale = the target
    // drag magnitude. Standard eps.
    let normalized = problem.recommended_normalized(target_x.abs().max(1e-3));
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
    let x_final = problem.forward_x(mu_rec, &th_rec);
    println!("\nRecovered de-escalation grip co-design:");
    println!(
        "  μ    = {mu_rec:>9.1} Pa (buffer stiffness; started at {mu0:.0}, μ* = {mu_star:.0})"
    );
    println!(
        "  w_z  = {:>9.4}    (holding policy: proportional)",
        th_rec[0]
    );
    println!(
        "  w_vz = {:>9.4}    (holding policy: derivative)",
        th_rec[1]
    );
    println!("  b    = {:>9.4}    (holding policy: bias)", th_rec[2]);
    println!(
        "\nGripped limb tip_x = {x_final:.6} m (target {target_x:.6} m, |Δ| = {:.2e} m) in {} \
         iters; converged = {}",
        (x_final - target_x).abs(),
        result.iters,
        result.converged,
    );
    println!(
        "The differentiable co-design loop tuned the buffer AND the holding policy to restrain \
         the limb — the study's first co-design result on the keystone soft↔rigid grip."
    );
}
