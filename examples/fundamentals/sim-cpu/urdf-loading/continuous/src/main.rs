//! Continuous Joint Wheel
//!
//! Spinning wheel with no joint limits (unlimited revolute). A constant torque
//! is applied via `qfrc_applied` and the angular acceleration is verified
//! against alpha = tau / I.
//!
//! Run: `cargo run -p example-urdf-continuous --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::suboptimal_flops
)]

use sim_core::MjJointType;

/// Wheel on axle with continuous (unlimited revolute) joint about Z.
const WHEEL_URDF: &str = r#"<?xml version="1.0"?>
<robot name="wheel">
    <link name="axle">
        <inertial>
            <mass value="100.0"/>
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10"/>
        </inertial>
    </link>
    <link name="wheel">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
        </inertial>
    </link>
    <joint name="spin" type="continuous">
        <parent link="axle"/>
        <child link="wheel"/>
        <axis xyz="0 0 1"/>
    </joint>
</robot>
"#;

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn main() {
    println!("=== URDF Loading — Continuous Joint Wheel ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    let model = sim_urdf::load_urdf_model(WHEEL_URDF).expect("URDF should parse");

    // --- Check 1: Joint type is hinge ---
    let is_hinge = model.jnt_type[0] == MjJointType::Hinge;
    if check(
        "Joint type is hinge",
        is_hinge,
        &format!("type={:?}", model.jnt_type[0]),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Joint is NOT limited (continuous = unlimited revolute) ---
    let is_unlimited = !model.jnt_limited[0];
    if check(
        "Joint is unlimited",
        is_unlimited,
        &format!("limited={}", model.jnt_limited[0]),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 3: Angular acceleration matches alpha = tau / I ---
    // I_zz = 0.02 for the wheel about the spin axis.
    // Apply torque tau = 1.0 N·m, expect alpha = 1.0 / 0.02 = 50 rad/s^2.
    let tau = 1.0_f64;
    let i_zz = 0.02_f64;
    let expected_alpha = tau / i_zz;

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Apply constant torque and step once
    data.qfrc_applied[0] = tau;
    data.step(&model).expect("step");

    // After one timestep from rest: omega ≈ alpha * dt
    let dt = model.timestep;
    let omega = data.qvel[0];
    let measured_alpha = omega / dt;
    let rel_err = ((measured_alpha - expected_alpha) / expected_alpha).abs();

    if check(
        "alpha = tau / I",
        rel_err < 0.01,
        &format!(
            "measured={measured_alpha:.2} rad/s^2, expected={expected_alpha:.2}, err={:.3}%",
            rel_err * 100.0
        ),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 4: Angle wraps past 2*pi (no limits clamp it) ---
    let mut data2 = model.make_data();
    data2.forward(&model).expect("forward");

    // Give it a big initial angular velocity so it passes 2*pi
    data2.qvel[0] = 100.0; // 100 rad/s
    for _ in 0..1000 {
        data2.step(&model).expect("step");
    }

    let final_q = data2.qpos[0];
    let passes_2pi = final_q.abs() > 2.0 * std::f64::consts::PI;
    if check(
        "Angle passes 2*pi (no clamping)",
        passes_2pi,
        &format!("q={final_q:.2} rad"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 5: Constant torque → linearly increasing velocity ---
    let mut data3 = model.make_data();
    data3.forward(&model).expect("forward");

    let n_steps = 500;
    for _ in 0..n_steps {
        data3.qfrc_applied[0] = tau;
        data3.step(&model).expect("step");
    }

    let expected_omega = expected_alpha * dt * (n_steps as f64);
    let measured_omega = data3.qvel[0];
    let omega_err = ((measured_omega - expected_omega) / expected_omega).abs();

    if check(
        "Constant torque → linear velocity ramp",
        omega_err < 0.01,
        &format!(
            "omega={measured_omega:.4}, expected={expected_omega:.4}, err={:.3}%",
            omega_err * 100.0
        ),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Summary ---
    println!("\n============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}
