//! Keystone actuator-dynamics leaf — the coupled articulated trajectory gradient w.r.t.
//! a MuJoCo `<actuator>`'s per-step control. PR1 (motor) + PR2 (state-feedback servos).
//!
//! A single-hinge arm with a joint actuator presses a soft block; the actuator torque
//! `qfrc_actuator = moment·(gain·ctrl + bias)` drives the arm THROUGH the contact. One
//! `tape.backward(tip_z_N)` gives `∂tip_z_N/∂u_k` for every per-step control `u_k` — the
//! articulated + real-`<actuator>` successor to the free-platen
//! `coupled_trajectory_control_gradient`. The carry gains the actuator-input channel
//! `s' = J_state·s + G·w + G_act·u`, `G_act_vel = Δt·M_impl⁻¹·∂qfrc_actuator/∂ctrl`.
//!
//! **Affine actuators — motor AND state-feedback servos (PR2).** The same machinery
//! covers any AFFINE actuator (`force = gain·ctrl + bias`, gain/bias affine in
//! length/velocity): the control channel `∂qfrc/∂ctrl = gain` is captured by
//! `actuator_velocity_column`, and a SERVO's state-feedback `∂qfrc/∂(qpos,qvel)`
//! (`−kp` for a position servo, `−kv` for a velocity servo) is a CONSTANT, EXPLICIT
//! (non-eulerdamp) slope, so it is already in the analytic single-hinge `J_state` (the
//! unloaded transition `A` from `transition_derivatives`) — no `ctrl`-replication needed.
//! Position, velocity, and combined PD servos are therefore machine-exact too.
//!
//! **Matched pair.** The FD oracle `coupled_trajectory_actuated_z` steps the REAL engine
//! with `data.ctrl` set each step, so it is the independent ground truth. The gate is the
//! one-tape control gradient vs a central FD of that oracle.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const MU0: f64 = 3.0e4;

// The single-hinge keystone scene (started tilted at qpos = 0.3 so the tip presses the
// block) with a swappable `<actuator>` driving joint `j`.
fn scene(actuator: &str) -> String {
    format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator>
    {actuator}
  </actuator>
</mujoco>"#
    )
}

fn build(actuator: &str) -> StaggeredCoupling {
    let model = load_model(&scene(actuator)).expect("actuator scene loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// The one-tape control gradient vs a per-control central FD of the (independent)
/// full-coupled oracle — machine-exact for any affine single-hinge actuator.
fn gate(label: &str, actuator: &str, controls: &[f64]) {
    let (tip_z, grad) = build(actuator).coupled_trajectory_actuator_gradient(controls);
    let z_oracle = build(actuator).coupled_trajectory_actuated_z(controls);
    assert!(
        (tip_z - z_oracle).abs() < 1e-12,
        "{label}: tape forward tip_z {tip_z} != actuated rollout {z_oracle}"
    );
    assert!(
        z_oracle > 0.10 && z_oracle < 0.115,
        "{label}: arm should stay engaged near the block top, got {z_oracle}"
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
        max_rel < 1e-6,
        "{label}: actuator control gradient must match the full-coupled FD (machine-exact), \
         worst rel {max_rel:.3e}"
    );
}

/// PR1 — a direct-torque MOTOR (`force = gear·ctrl`, no state feedback).
#[test]
fn motor_control_gradient_matches_fd() {
    gate(
        "motor",
        r#"<motor name="a" joint="j" gear="4"/>"#,
        &[0.3, -0.2, 0.25, -0.15, 0.1, 0.2],
    );
}

/// PR2 — a POSITION servo (`force = kp·(ctrl − qpos)`): the state-feedback stiffness
/// `∂force/∂qpos = −kp` is explicit ⇒ already in the analytic `J_state`. Control = target
/// angles near the tilt.
#[test]
fn position_servo_control_gradient_matches_fd() {
    gate(
        "position",
        r#"<position name="a" joint="j" kp="8"/>"#,
        &[0.35, 0.25, 0.4, 0.2, 0.3, 0.28],
    );
}

/// PR2 — a VELOCITY servo (`force = kv·(ctrl − qvel)`): the state-feedback damping
/// `∂force/∂qvel = −kv` is explicit (NOT eulerdamp) ⇒ already in `J_state`. Control =
/// target rates.
#[test]
fn velocity_servo_control_gradient_matches_fd() {
    gate(
        "velocity",
        r#"<velocity name="a" joint="j" kv="0.5"/>"#,
        &[0.5, -0.3, 0.4, -0.2, 0.3, -0.1],
    );
}

/// PR2 — a PD servo (`<position kp= kv=>`: `force = kp·(ctrl − qpos) − kv·qvel`) — BOTH
/// state-feedback channels live in one actuator. The realistic exo joint controller.
#[test]
fn pd_servo_control_gradient_matches_fd() {
    gate(
        "pd",
        r#"<position name="a" joint="j" kp="8" kv="0.4"/>"#,
        &[0.35, 0.25, 0.4, 0.2, 0.3, 0.28],
    );
}

/// Materiality: the actuator genuinely steers the tip through the contact — a nonzero
/// control sequence changes `tip_z_N` vs the unactuated (ctrl = 0) rollout.
#[test]
fn actuator_moves_the_tip() {
    let n = 6;
    let drive = |actuator: &str, u: f64| build(actuator).coupled_trajectory_actuated_z(&vec![u; n]);
    for (label, actuator, u) in [
        ("motor", r#"<motor name="a" joint="j" gear="4"/>"#, 0.4),
        ("position", r#"<position name="a" joint="j" kp="8"/>"#, 0.5),
    ] {
        let rel = (drive(actuator, u) - drive(actuator, 0.0)).abs() / drive(actuator, 0.0).abs();
        println!("{label}: rel-diff={rel:.3e}");
        assert!(
            rel > 1e-4,
            "{label}: the actuator must move the tip through the contact (rel {rel:.3e})"
        );
    }
}
