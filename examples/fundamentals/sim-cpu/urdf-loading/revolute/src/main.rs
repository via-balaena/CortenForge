//! Revolute Joint Pendulum
//!
//! Single hinge pendulum defined in URDF. Swings under gravity. Verifies the
//! oscillation period matches the analytical prediction T = 2*pi*sqrt(L/g)
//! for small angles, and that the URDF → Model pipeline produces the correct
//! joint type with correct limits.
//!
//! Run: `cargo run -p example-urdf-revolute --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::suboptimal_flops,
    clippy::too_many_lines,
    clippy::approx_constant
)]

/// Simple pendulum: base + arm with revolute joint.
/// Arm COM is at 0.25m below the pivot → effective length L = 0.25m.
const PENDULUM_URDF: &str = r#"<?xml version="1.0"?>
<robot name="pendulum">
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

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn main() {
    println!("=== URDF Loading — Revolute Joint Pendulum ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    // --- Check 1: URDF loads successfully ---
    let model = sim_urdf::load_urdf_model(PENDULUM_URDF).expect("URDF should parse");
    let ok = model.njnt == 1;
    if check(
        "URDF loads with 1 joint",
        ok,
        &format!("njnt={}", model.njnt),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Joint type is hinge ---
    let is_hinge = model.jnt_type[0] == sim_core::MjJointType::Hinge;
    if check(
        "Joint type is hinge",
        is_hinge,
        &format!("type={:?}", model.jnt_type[0]),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 3: Joint is limited ---
    let is_limited = model.jnt_limited[0];
    let range = model.jnt_range[0];
    let range_ok = (range.0 - (-3.14)).abs() < 0.01 && (range.1 - 3.14).abs() < 0.01;
    if check(
        "Joint limited with correct range",
        is_limited && range_ok,
        &format!(
            "limited={is_limited}, range=({:.2}, {:.2})",
            range.0, range.1
        ),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 4: Period matches analytical prediction ---
    // For a compound pendulum with point mass at distance L from pivot:
    //   I = m*L^2 + I_cm, T = 2*pi*sqrt(I / (m*g*L))
    // With m=1, L=0.25, I_cm≈0.02 (about y-axis):
    //   I = 1*0.0625 + 0.02 = 0.0825
    //   T = 2*pi*sqrt(0.0825 / (1*9.81*0.25)) ≈ 1.152s
    // For small angles, we measure the half-period (time to first zero-crossing
    // from positive angle).
    let mut data = model.make_data();
    data.qpos[0] = 0.05; // small angle (≈3 degrees)
    data.forward(&model).expect("forward");

    let dt = model.timestep;
    let mut prev_q = data.qpos[0];
    let mut first_crossing = None;
    let mut second_crossing = None;

    for step in 0..20_000 {
        data.step(&model).expect("step");
        let q = data.qpos[0];
        // Consecutive downward zero crossings are exactly one period apart
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

    let t1 = first_crossing.expect("should find first crossing");
    let t2 = second_crossing.expect("should find second crossing");
    let measured_t = t2 - t1;
    let m = 1.0_f64;
    let big_l = 0.25_f64;
    let i_cm = 0.02_f64; // Iyy of the arm
    let i_total = m * big_l * big_l + i_cm;
    let analytical_t = 2.0 * std::f64::consts::PI * (i_total / (m * 9.81 * big_l)).sqrt();
    let rel_err = ((measured_t - analytical_t) / analytical_t).abs();

    if check(
        "Period matches analytical (small angle)",
        rel_err < 0.02,
        &format!(
            "measured={measured_t:.4}s, analytical={analytical_t:.4}s, err={:.2}%",
            rel_err * 100.0
        ),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 5: Energy is approximately conserved (no damping) ---
    let mut data2 = model.make_data();
    data2.qpos[0] = 0.3; // moderate angle
    data2.forward(&model).expect("forward");

    // Collect energy at start and after 2000 steps
    data2.forward(&model).expect("forward");
    let e0 = data2.energy_kinetic + data2.energy_potential;
    for _ in 0..2000 {
        data2.step(&model).expect("step");
    }
    let e_final = data2.energy_kinetic + data2.energy_potential;
    let e_drift = ((e_final - e0) / e0.abs().max(1e-10)).abs();

    if check(
        "Energy approximately conserved",
        e_drift < 0.01,
        &format!(
            "E0={e0:.6}, E_final={e_final:.6}, drift={:.4}%",
            e_drift * 100.0
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
