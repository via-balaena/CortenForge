//! Keystone actuator-dynamics leaf (PR1) — the coupled articulated trajectory gradient
//! w.r.t. a MuJoCo `<motor>` actuator's per-step control.
//!
//! A single-hinge arm with a joint motor presses a soft block; the motor torque
//! `qfrc_actuator = moment·gain·ctrl` drives the arm THROUGH the contact. One
//! `tape.backward(tip_z_N)` gives `∂tip_z_N/∂u_k` for every per-step control `u_k` —
//! the articulated + real-`<actuator>` successor to the free-platen
//! `coupled_trajectory_control_gradient`. The carry gains the actuator-input channel
//! `s' = J_state·s + G·w + G_act·u`, `G_act_vel = Δt·M_impl⁻¹·∂qfrc_actuator/∂ctrl`.
//!
//! **Matched pair.** The FD oracle `coupled_trajectory_actuated_z` steps the REAL engine
//! with `data.ctrl` set each step, so it is the independent ground truth. The gate is the
//! one-tape control gradient vs a central FD of that oracle. Single-hinge motor → the
//! analytic carry is machine-exact.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// The single-hinge keystone scene + a joint motor (gear 4). Started tilted (qpos = 0.3)
// so the tip presses the block; the motor torque modulates the arm through the contact.
const MOTOR_HINGE: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="m" joint="j" gear="4"/>
  </actuator>
</mujoco>"#;

const MU0: f64 = 3.0e4;

fn build() -> StaggeredCoupling {
    let model = load_model(MOTOR_HINGE).expect("motor hinge loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, MU0, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

#[test]
fn actuator_control_gradient_matches_fd() {
    // A varied control sequence so each ∂tip_z/∂u_k is a distinct, nonzero target.
    let controls = [0.3_f64, -0.2, 0.25, -0.15, 0.1, 0.2];

    let (tip_z, grad) = build().coupled_trajectory_actuator_gradient(&controls);
    // Fresh-output consistency: the tape forward reproduces the real actuated rollout.
    let z_oracle = build().coupled_trajectory_actuated_z(&controls);
    assert!(
        (tip_z - z_oracle).abs() < 1e-12,
        "tape forward tip_z {tip_z} != actuated rollout {z_oracle}"
    );
    assert!(
        z_oracle > 0.10 && z_oracle < 0.115,
        "arm should stay engaged near the block top, got {z_oracle}"
    );

    let eps = 1e-3;
    let mut max_rel = 0.0_f64;
    for k in 0..controls.len() {
        let mut cp = controls;
        let mut cm = controls;
        cp[k] += eps;
        cm[k] -= eps;
        let fd = (build().coupled_trajectory_actuated_z(&cp)
            - build().coupled_trajectory_actuated_z(&cm))
            / (2.0 * eps);
        let rel = (grad[k] - fd).abs() / fd.abs().max(1e-30);
        println!("u_{k}: tape={:.6e} FD={fd:.6e} rel={rel:.3e}", grad[k]);
        assert!(
            fd.abs() > 1e-9,
            "∂tip_z/∂u_{k} should be a nonzero, well-posed target"
        );
        max_rel = max_rel.max(rel);
    }
    assert!(
        max_rel < 1e-6,
        "actuator control gradient must match full-coupled FD (single-hinge motor → \
         machine-exact), worst rel {max_rel:.3e}"
    );
}

/// Materiality: the motor genuinely steers the tip through the contact — a nonzero
/// control sequence changes `tip_z_N` vs the unactuated (ctrl = 0) rollout.
#[test]
fn motor_control_moves_the_tip() {
    let n = 6;
    let z_driven = build().coupled_trajectory_actuated_z(&vec![0.4_f64; n]);
    let z_passive = build().coupled_trajectory_actuated_z(&vec![0.0_f64; n]);
    let rel = (z_driven - z_passive).abs() / z_passive.abs().max(1e-30);
    println!("driven={z_driven:.6e} passive={z_passive:.6e} rel-diff={rel:.3e}");
    assert!(
        rel > 1e-4,
        "the motor must move the tip through the contact (rel diff {rel:.3e})"
    );
}
