//! Joint Dynamics — Damping and Friction
//!
//! Three identical pendulums with different dynamics parameters:
//! (A) damping=0, friction=0 — no energy loss (conservative)
//! (B) damping=0.5, friction=0 — velocity-dependent dissipation
//! (C) damping=0, friction=0.5 — velocity-independent frictionloss
//!
//! All start at the same angle. Compares energy decay: A stays constant,
//! B and C both decay, but with different profiles.
//!
//! Run: `cargo run -p example-urdf-damping-friction --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops,
    clippy::too_many_lines
)]

/// No damping, no friction (conservative).
const NO_LOSS_URDF: &str = r#"<?xml version="1.0"?>
<robot name="no_loss">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 -0.25"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
    </link>
    <joint name="hinge" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <origin xyz="0 0 1.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
    </joint>
</robot>
"#;

/// Velocity-dependent damping only.
const DAMPED_URDF: &str = r#"<?xml version="1.0"?>
<robot name="damped">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 -0.25"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
    </link>
    <joint name="hinge" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <origin xyz="0 0 1.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
        <dynamics damping="0.5"/>
    </joint>
</robot>
"#;

/// Velocity-independent friction (frictionloss) only.
const FRICTION_URDF: &str = r#"<?xml version="1.0"?>
<robot name="friction">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 -0.25"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
    </link>
    <joint name="hinge" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <origin xyz="0 0 1.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
        <dynamics damping="0" friction="0.5"/>
    </joint>
</robot>
"#;

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

/// Simulate a pendulum for n_steps, return peak velocity magnitude.
fn simulate_peak_velocity(urdf: &str, n_steps: usize) -> f64 {
    let model = sim_urdf::load_urdf_model(urdf).expect("parse");
    let mut data = model.make_data();
    data.qpos[0] = 0.5; // 0.5 rad initial angle
    data.forward(&model).expect("forward");

    let mut peak_vel = 0.0_f64;
    for _ in 0..n_steps {
        data.step(&model).expect("step");
        peak_vel = peak_vel.max(data.qvel[0].abs());
    }
    peak_vel
}

fn main() {
    println!("=== URDF Loading — Damping & Friction ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    // --- Check 1: Damping value propagates to model ---
    let damped_model = sim_urdf::load_urdf_model(DAMPED_URDF).expect("parse");
    let damping = damped_model.jnt_damping[0];
    if check(
        "Damping=0.5 in model",
        (damping - 0.5).abs() < 0.001,
        &format!("jnt_damping={damping:.3}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Frictionloss value propagates to model ---
    let friction_model = sim_urdf::load_urdf_model(FRICTION_URDF).expect("parse");
    let fl = friction_model.dof_frictionloss[0];
    if check(
        "Frictionloss=0.5 in model",
        (fl - 0.5).abs() < 0.001,
        &format!("dof_frictionloss={fl:.3}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 3: MJCF output contains frictionloss ---
    let mjcf = sim_urdf::urdf_to_mjcf(FRICTION_URDF).expect("convert");
    let has_fl = mjcf.contains("frictionloss");
    if check(
        "MJCF contains frictionloss",
        has_fl,
        &format!("found: {has_fl}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 4: Conservative pendulum maintains peak velocity ---
    let steps = 5000;
    let peak_no_loss_early = {
        let model = sim_urdf::load_urdf_model(NO_LOSS_URDF).expect("parse");
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.forward(&model).expect("forward");
        let mut peak = 0.0_f64;
        for _ in 0..500 {
            data.step(&model).expect("step");
            peak = peak.max(data.qvel[0].abs());
        }
        peak
    };
    let peak_no_loss_late = {
        let model = sim_urdf::load_urdf_model(NO_LOSS_URDF).expect("parse");
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.forward(&model).expect("forward");
        for _ in 0..4500 {
            data.step(&model).expect("step");
        }
        let mut peak = 0.0_f64;
        for _ in 0..500 {
            data.step(&model).expect("step");
            peak = peak.max(data.qvel[0].abs());
        }
        peak
    };
    let ratio = peak_no_loss_late / peak_no_loss_early;
    if check(
        "No-loss pendulum: peak velocity stable",
        ratio > 0.95,
        &format!("early={peak_no_loss_early:.4}, late={peak_no_loss_late:.4}, ratio={ratio:.4}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 5: Damped pendulum loses energy ---
    let peak_damped = simulate_peak_velocity(DAMPED_URDF, steps);
    let peak_no_loss = simulate_peak_velocity(NO_LOSS_URDF, steps);
    if check(
        "Damped pendulum: lower peak velocity",
        peak_damped < peak_no_loss * 0.8,
        &format!("damped={peak_damped:.4}, no_loss={peak_no_loss:.4}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 6: Friction pendulum loses energy ---
    let peak_friction = simulate_peak_velocity(FRICTION_URDF, steps);
    if check(
        "Friction pendulum: lower peak velocity",
        peak_friction < peak_no_loss * 0.8,
        &format!("friction={peak_friction:.4}, no_loss={peak_no_loss:.4}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 7: Damped and friction have different decay profiles ---
    // Damping is velocity-proportional; friction is constant-magnitude.
    // They should produce different peak velocities over the same time.
    let diff = (peak_damped - peak_friction).abs();
    if check(
        "Damping ≠ friction (different profiles)",
        diff > 0.001,
        &format!("damped={peak_damped:.4}, friction={peak_friction:.4}, diff={diff:.4}"),
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
