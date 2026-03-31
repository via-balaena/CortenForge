//! Prismatic Joint Slider
//!
//! Mass on a horizontal spring using a prismatic (slide) joint from URDF.
//! A spring force is applied via `qfrc_applied` (URDF has no stiffness attribute).
//! Verifies oscillation period matches the analytical T = 2*pi*sqrt(m/k).
//!
//! Run: `cargo run -p example-urdf-prismatic --release`

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

/// Horizontal slider: base + sliding mass with prismatic joint along X.
/// No gravity influence on the slide direction (gravity is -Z).
const SLIDER_URDF: &str = r#"<?xml version="1.0"?>
<robot name="slider">
    <link name="base">
        <inertial>
            <mass value="100.0"/>
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10"/>
        </inertial>
    </link>
    <link name="mass">
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>
    <joint name="slide" type="prismatic">
        <parent link="base"/>
        <child link="mass"/>
        <origin xyz="0 0 1.0"/>
        <axis xyz="1 0 0"/>
        <limit lower="-1.0" upper="1.0"/>
    </joint>
</robot>
"#;

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn main() {
    println!("=== URDF Loading — Prismatic Joint Slider ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    // --- Check 1: URDF loads ---
    let model = sim_urdf::load_urdf_model(SLIDER_URDF).expect("URDF should parse");
    if check(
        "URDF loads",
        model.njnt == 1,
        &format!("njnt={}", model.njnt),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Joint type is slide ---
    let is_slide = model.jnt_type[0] == MjJointType::Slide;
    if check(
        "Joint type is slide",
        is_slide,
        &format!("type={:?}", model.jnt_type[0]),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 3: Joint limits correct ---
    let range = model.jnt_range[0];
    let range_ok = (range.0 - (-1.0)).abs() < 0.001 && (range.1 - 1.0).abs() < 0.001;
    if check(
        "Slide limits [-1, 1]",
        model.jnt_limited[0] && range_ok,
        &format!("range=({:.2}, {:.2})", range.0, range.1),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 4: Axis mapped correctly ---
    let axis = model.jnt_axis[0];
    // URDF axis "1 0 0" should map to X-axis in MJCF
    let axis_ok = (axis.x - 1.0).abs() < 0.01 && axis.y.abs() < 0.01 && axis.z.abs() < 0.01;
    if check(
        "Axis maps to X",
        axis_ok,
        &format!("axis=({:.2}, {:.2}, {:.2})", axis.x, axis.y, axis.z),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 5: Period matches T = 2*pi*sqrt(m/k) with manual spring ---
    let k = 50.0_f64; // spring constant (N/m)
    let m = 2.0_f64;
    let dt = model.timestep;

    let mut data = model.make_data();
    data.qpos[0] = 0.1; // 10cm displacement
    data.forward(&model).expect("forward");

    let mut prev_q = data.qpos[0];
    let mut first_crossing = None;
    let mut second_crossing = None;

    for step in 0..50_000 {
        // Apply spring force: F = -k * q
        data.qfrc_applied[0] = -k * data.qpos[0];
        data.step(&model).expect("step");
        let q = data.qpos[0];

        if prev_q > 0.0 && q <= 0.0 {
            let t = (step as f64 + 1.0) * dt;
            if first_crossing.is_none() {
                first_crossing = Some(t);
            } else {
                second_crossing = Some(t);
                break;
            }
        }
        prev_q = q;
    }

    let t1 = first_crossing.expect("first crossing");
    let t2 = second_crossing.expect("second crossing");
    let measured_t = t2 - t1;
    let analytical_t = 2.0 * std::f64::consts::PI * (m / k).sqrt();
    let rel_err = ((measured_t - analytical_t) / analytical_t).abs();

    if check(
        "Period matches T = 2pi*sqrt(m/k)",
        rel_err < 0.02,
        &format!(
            "measured={measured_t:.4}s, analytical={analytical_t:.4}s, err={:.2}%",
            rel_err * 100.0
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
