//! Keystone actuator-dynamics leaf — actuated CHAINS (`nv > 1`). The articulated
//! actuator control gradient on a multi-link mechanism, the exo's actual topology.
//!
//! A damped 2-link hinge chain (`nv = 2`) presses a soft block; an actuator on the
//! distal joint drives the tip THROUGH the contact. On a chain the actuator force
//! interacts with the configuration-dependent mass matrix (`∂M⁻¹/∂q`), so the loaded
//! `J_state` must see the control — `coupled_trajectory_actuator_gradient` sets `ctrl`
//! before the carry and `scratch_state_step` replicates it in the FD
//! `loaded_state_jacobian` (a `ctrl`-blind scratch was ≈5e-5 wrong; replicating it →
//! ~1e-8). Joint damping keeps the unconstrained chain settled on the block (an
//! undamped chain swings the distal link up off the block).
//!
//! **Matched pair.** The FD oracle `coupled_trajectory_actuated_z` steps the REAL engine
//! with `data.ctrl` set each step. The gate is the one-tape control gradient vs a central
//! FD of that oracle. The chain has no single-hinge analytic `J_state`, so the carry uses
//! the FD `loaded_state_jacobian` (FD-carry precision).

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const MU0: f64 = 3.0e4;

// A damped 2-link hinge chain (distal tip presses the block) with a swappable actuator
// on the DISTAL joint `j1` (direct authority on the tip).
fn scene(actuator: &str) -> String {
    format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="upper" pos="0 0 0.2">
      <joint name="j0" type="hinge" axis="0 1 0" damping="0.3"/>
      <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
      <body name="lower" pos="0 0 -0.05">
        <joint name="j1" type="hinge" axis="0 1 0" damping="0.3"/>
        <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    {actuator}
  </actuator>
</mujoco>"#
    )
}

fn build(actuator: &str) -> StaggeredCoupling {
    let model = load_model(&scene(actuator)).expect("chain actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.1;
    data.qpos[1] = -0.05;
    data.forward(&model).expect("forward");
    // body = 2 (the distal link, the contacting tip).
    StaggeredCoupling::new(
        model, data, 2, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// The one-tape control gradient vs a per-control central FD of the (independent)
/// full-coupled oracle — FD-carry precision for the `nv = 2` chain.
fn gate(label: &str, actuator: &str, controls: &[f64]) {
    let (tip_z, grad) = build(actuator).coupled_trajectory_actuator_gradient(controls);
    let z_oracle = build(actuator).coupled_trajectory_actuated_z(controls);
    assert!(
        (tip_z - z_oracle).abs() < 1e-12,
        "{label}: tape forward tip_z {tip_z} != actuated rollout {z_oracle}"
    );
    assert!(
        z_oracle > 0.105 && z_oracle < 0.115,
        "{label}: damped chain should stay settled on the block, got {z_oracle}"
    );
    let eps = 1e-3;
    let mut max_rel = 0.0_f64;
    for k in 0..controls.len() {
        let mut cp = controls.to_vec();
        let mut cm = controls.to_vec();
        cp[k] += eps;
        cm[k] -= eps;
        let fd = (build(actuator).coupled_trajectory_actuated_z(&cp)
            - build(actuator).coupled_trajectory_actuated_z(&cm))
            / (2.0 * eps);
        let rel = (grad[k] - fd).abs() / fd.abs().max(1e-30);
        println!(
            "{label} u_{k}: tape={:.6e} FD={fd:.6e} rel={rel:.3e}",
            grad[k]
        );
        assert!(
            fd.abs() > 1e-9,
            "{label}: ∂tip_z/∂u_{k} should be a nonzero, well-posed target"
        );
        max_rel = max_rel.max(rel);
    }
    assert!(
        max_rel < 1e-5,
        "{label}: chain actuator control gradient must match the full-coupled FD \
         (FD-carry precision), worst rel {max_rel:.3e}"
    );
}

/// A MOTOR on the distal joint of the chain — the constant force interacts with
/// `∂M⁻¹/∂q`, the channel a `ctrl`-blind `J_state` would drop.
#[test]
fn chain_motor_control_gradient_matches_fd() {
    gate(
        "chain-motor",
        r#"<motor name="a" joint="j1" gear="2"/>"#,
        &[0.15, -0.1, 0.12, -0.08],
    );
}

/// A POSITION servo on the distal joint of the chain.
#[test]
fn chain_position_servo_control_gradient_matches_fd() {
    gate(
        "chain-position",
        r#"<position name="a" joint="j1" kp="2"/>"#,
        &[0.1, -0.1, 0.05, -0.05],
    );
}

/// A VELOCITY servo on the distal joint of the chain.
#[test]
fn chain_velocity_servo_control_gradient_matches_fd() {
    gate(
        "chain-velocity",
        r#"<velocity name="a" joint="j1" kv="0.3"/>"#,
        &[0.3, -0.2, 0.25, -0.15],
    );
}

/// Materiality: the chain actuator genuinely steers the tip through the contact.
#[test]
fn chain_actuator_moves_the_tip() {
    let actuator = r#"<motor name="a" joint="j1" gear="2"/>"#;
    let n = 4;
    let z_driven = build(actuator).coupled_trajectory_actuated_z(&vec![0.3_f64; n]);
    let z_passive = build(actuator).coupled_trajectory_actuated_z(&vec![0.0_f64; n]);
    let rel = (z_driven - z_passive).abs() / z_passive.abs().max(1e-30);
    println!("chain: driven={z_driven:.6e} passive={z_passive:.6e} rel-diff={rel:.3e}");
    assert!(
        rel > 1e-5,
        "the chain actuator must move the tip through the contact (rel diff {rel:.3e})"
    );
}
