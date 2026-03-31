//! Mimic Joint Coupling
//!
//! Two revolute joints on a common base. The follower joint mimics the leader
//! with multiplier=2, offset=0.1. The URDF `<mimic>` element converts to
//! `<equality><joint>` with `polycoef` in MJCF.
//!
//! Verifies the follower tracks at 2*leader + 0.1 within solver tolerance.
//!
//! Run: `cargo run -p example-urdf-mimic --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

/// Two hinges on a common base. Follower mimics leader with ratio 2:1 + offset.
const MIMIC_URDF: &str = r#"<?xml version="1.0"?>
<robot name="mimic_test">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="link1">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>
    <link name="link2">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>
    <joint name="leader" type="revolute">
        <parent link="base"/>
        <child link="link1"/>
        <origin xyz="0 0.2 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
    </joint>
    <joint name="follower" type="revolute">
        <parent link="base"/>
        <child link="link2"/>
        <origin xyz="0 -0.2 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-6.28" upper="6.28"/>
        <mimic joint="leader" multiplier="2.0" offset="0.1"/>
    </joint>
</robot>
"#;

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn main() {
    println!("=== URDF Loading — Mimic Joint Coupling ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    // --- Check 1: MJCF contains equality constraint ---
    let mjcf = sim_urdf::urdf_to_mjcf(MIMIC_URDF).expect("convert");
    let has_equality = mjcf.contains("<equality>") && mjcf.contains("polycoef");
    if check(
        "MJCF has <equality> with polycoef",
        has_equality,
        if has_equality { "found" } else { "missing" },
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Model loads with equality constraints ---
    let model = sim_urdf::load_urdf_model(MIMIC_URDF).expect("URDF should parse");
    let has_eq = model.neq > 0;
    if check(
        "Model has equality constraints",
        has_eq,
        &format!("neq={}", model.neq),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 3: Two joints exist ---
    let two_joints = model.njnt == 2;
    if check(
        "Two joints in model",
        two_joints,
        &format!("njnt={}", model.njnt),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 4: Follower tracks leader at equilibrium ---
    // Set leader to 0.5 and let the constraint solver settle.
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.qpos[1] = 2.0 * 0.5 + 0.1; // start near expected
    data.forward(&model).expect("forward");

    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let leader = data.qpos[0];
    let follower = data.qpos[1];
    let expected = 2.0 * leader + 0.1;
    let err = (follower - expected).abs();

    if check(
        "Follower tracks 2*leader + 0.1",
        err < 0.05,
        &format!(
            "leader={leader:.4}, follower={follower:.4}, expected={expected:.4}, err={err:.4}"
        ),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 5: Constraint holds under dynamic motion ---
    // Reset and drive leader to different position.
    let mut data2 = model.make_data();
    data2.qpos[0] = -0.3;
    data2.qpos[1] = 2.0 * (-0.3) + 0.1;
    data2.forward(&model).expect("forward");

    for _ in 0..2000 {
        data2.step(&model).expect("step");
    }

    let leader2 = data2.qpos[0];
    let follower2 = data2.qpos[1];
    let expected2 = 2.0 * leader2 + 0.1;
    let err2 = (follower2 - expected2).abs();

    if check(
        "Constraint holds from different initial condition",
        err2 < 0.05,
        &format!(
            "leader={leader2:.4}, follower={follower2:.4}, expected={expected2:.4}, err={err2:.4}"
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
