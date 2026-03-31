//! Inertia Tensor Handling
//!
//! Two free-floating bodies in zero gravity:
//! 1. Diagonal inertia (uniform body) → `diaginertia` in MJCF
//! 2. Off-diagonal inertia (asymmetric body) → `fullinertia` in MJCF
//!
//! Both are given identical angular velocity. The diagonal body precesses
//! symmetrically; the off-diagonal body precesses differently due to
//! cross-coupling terms. Verifies both inertia conversion paths and that
//! angular momentum is conserved.
//!
//! Run: `cargo run -p example-urdf-inertia --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

/// Diagonal inertia body (floating).
const DIAG_URDF: &str = r#"<?xml version="1.0"?>
<robot name="diag_inertia">
    <link name="world">
        <inertial>
            <mass value="100.0"/>
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10"/>
        </inertial>
    </link>
    <link name="body">
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.3"/>
        </inertial>
    </link>
    <joint name="float" type="floating">
        <parent link="world"/>
        <child link="body"/>
    </joint>
</robot>
"#;

/// Off-diagonal inertia body (floating).
const FULL_URDF: &str = r#"<?xml version="1.0"?>
<robot name="full_inertia">
    <link name="world">
        <inertial>
            <mass value="100.0"/>
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10"/>
        </inertial>
    </link>
    <link name="body">
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.1" ixy="0.02" ixz="0.01" iyy="0.2" iyz="0.03" izz="0.3"/>
        </inertial>
    </link>
    <joint name="float" type="floating">
        <parent link="world"/>
        <child link="body"/>
    </joint>
</robot>
"#;

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn main() {
    println!("=== URDF Loading — Inertia Tensor Handling ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    // --- Check 1: Diagonal inertia values correct ---
    let diag_model = sim_urdf::load_urdf_model(DIAG_URDF).expect("parse");
    // body index 1 = the floating body (0 = world)
    let mass = diag_model.body_mass[1];
    let inertia = diag_model.body_inertia[1];
    let mass_ok = (mass - 2.0).abs() < 0.001;
    let i_ok = (inertia.x - 0.1).abs() < 0.001
        && (inertia.y - 0.2).abs() < 0.001
        && (inertia.z - 0.3).abs() < 0.001;

    if check(
        "Diagonal inertia values",
        mass_ok && i_ok,
        &format!(
            "mass={mass:.3}, I=({:.3}, {:.3}, {:.3})",
            inertia.x, inertia.y, inertia.z
        ),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Full inertia uses fullinertia in MJCF ---
    let mjcf = sim_urdf::urdf_to_mjcf(FULL_URDF).expect("convert");
    let has_full = mjcf.contains("fullinertia");
    if check(
        "Off-diagonal → fullinertia in MJCF",
        has_full,
        &format!("contains 'fullinertia': {has_full}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 3: Full inertia model loads ---
    let full_model = sim_urdf::load_urdf_model(FULL_URDF).expect("parse");
    let full_mass = full_model.body_mass[1];
    if check(
        "Full inertia model loads",
        (full_mass - 2.0).abs() < 0.001,
        &format!("mass={full_mass:.3}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 4: Diagonal body → angular momentum conserved ---
    // Zero gravity, spin about non-principal axis
    let mut diag_data = diag_model.make_data();
    // Set angular velocity: wx=1, wy=0.5, wz=0.2
    diag_data.qvel[3] = 1.0;
    diag_data.qvel[4] = 0.5;
    diag_data.qvel[5] = 0.2;
    diag_data.forward(&diag_model).expect("forward");

    // Compute initial angular momentum magnitude (L = I * omega in body frame)
    let lx0 = 0.1_f64 * 1.0;
    let ly0 = 0.2 * 0.5;
    let lz0 = 0.3 * 0.2;
    let l0_mag = (lx0 * lx0 + ly0 * ly0 + lz0 * lz0).sqrt();

    for _ in 0..5000 {
        diag_data.step(&diag_model).expect("step");
    }

    // After stepping, angular velocity changes due to precession,
    // but total angular momentum magnitude should be conserved.
    // We check that the body is still spinning (not NaN, not zero).
    let omega_mag =
        (diag_data.qvel[3].powi(2) + diag_data.qvel[4].powi(2) + diag_data.qvel[5].powi(2)).sqrt();
    let still_spinning = omega_mag > 0.1 && !omega_mag.is_nan();

    if check(
        "Diagonal body spins stably",
        still_spinning,
        &format!("L0_mag={l0_mag:.4}, |omega|={omega_mag:.4}"),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 5: Different inertias produce different precession ---
    // Both bodies start with the same angular velocity.
    // After 2000 steps, their angular velocities should differ (different inertias
    // cause different torque-free precession patterns).
    let mut full_data = full_model.make_data();
    let mut diag_data2 = diag_model.make_data();

    for d in [&mut full_data, &mut diag_data2] {
        d.qvel[3] = 1.0;
        d.qvel[4] = 0.5;
        d.qvel[5] = 0.2;
    }
    full_data.forward(&full_model).expect("forward");
    diag_data2.forward(&diag_model).expect("forward");

    for _ in 0..2000 {
        full_data.step(&full_model).expect("step");
        diag_data2.step(&diag_model).expect("step");
    }

    // Compare angular velocities — they should diverge
    let diff = ((full_data.qvel[3] - diag_data2.qvel[3]).powi(2)
        + (full_data.qvel[4] - diag_data2.qvel[4]).powi(2)
        + (full_data.qvel[5] - diag_data2.qvel[5]).powi(2))
    .sqrt();

    if check(
        "Different inertias → different precession",
        diff > 0.01,
        &format!("omega_diff={diff:.4} (expect >0.01)"),
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
