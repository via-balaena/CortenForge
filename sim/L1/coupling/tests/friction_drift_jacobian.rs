//! Keystone friction leaf PR3b-2a — the coupled-step DRIFT JACOBIAN `∂x*/∂v_rigid`.
//!
//! The two-way grip feedback edge in isolation: the rigid body's tangential velocity enters
//! the soft solve only through the collider drift `Δ_surf = v_rigid·dt`, so `∂x*/∂v_rigid =
//! ∂x*/∂Δ_surf · dt` (the sim-soft drift sensitivity, PR3b-1, scaled by dt at the coupling).
//! This gates that composition against a full re-solve FD over the rigid velocity — the new
//! edge the full friction-coupled trajectory gradient (PR3b-2b) will chain. Forward-of-gradient
//! validation; no tape yet.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::Vec3;

// Free platen pushed sideways by tilted gravity (the grip scene) — so after a few steps the
// platen is ENGAGED on the block AND moving tangentially (a non-degenerate v_rigid for the
// drift Jacobian). Start near force balance to avoid a launch spike (PR3a S0 lesson).
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.115">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

fn rel(a: &[f64], b: &[f64]) -> f64 {
    let num: f64 = a
        .iter()
        .zip(b)
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt();
    let den: f64 = b.iter().map(|y| y * y).sum::<f64>().sqrt();
    num / den.max(1e-30)
}

#[test]
fn coupled_drift_jacobian_matches_resolve_fd() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    let mut c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, /* body */ 1, /* contact_clearance */ 0.005,
        /* n_per_edge */ 4, /* edge */ 0.1, /* mu (soft) */ 3.0e4,
        /* dt */ 1.0e-3, /* kappa */ 3.0e4, /* d_hat */ 1.0e-2,
        /* rigid_damping */ 8.0,
    )
    .with_friction(2.0, 0.1);

    // Advance to an engaged, tangentially-moving operating point.
    let _ = c.coupled_trajectory_grip(120);
    let v_tan = c.data().qvel[0];
    assert!(
        v_tan.abs() > 1e-2,
        "degenerate: platen barely moving tangentially (v_x={v_tan}) — no drift to differentiate"
    );

    let dir = Vec3::new(1.0, 0.0, 0.0);
    let an = c.coupled_step_drift_jacobian(dir);
    // FD: re-solve x* with the platen velocity perturbed ±ε along x.
    let eps = 1.0e-4;
    let xp = c.coupled_step_x_at_velocity_perturbation(dir, eps);
    let xm = c.coupled_step_x_at_velocity_perturbation(dir, -eps);
    let fd: Vec<f64> = xp
        .iter()
        .zip(&xm)
        .map(|(a, b)| (a - b) / (2.0 * eps))
        .collect();

    let r = rel(&an, &fd);
    let max: f64 = an.iter().map(|v| v.abs()).fold(0.0, f64::max);
    eprintln!("PR3b-2a ∂x*/∂v_rigid: v_x={v_tan:.4e}  ‖·‖∞={max:.3e}  rel-vs-FD={r:.3e}");
    assert!(max > 1e-7, "drift Jacobian implausibly small");
    assert!(
        r < 1e-5,
        "coupled drift Jacobian disagrees with re-solve FD: {r:e}"
    );
}
