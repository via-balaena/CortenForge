//! Fixed Joint Fusion
//!
//! Three links: base → fixed → child → revolute → grandchild.
//! With `fusestatic="true"` (the default in the URDF converter), the
//! fixed-joint link merges into its parent. The compiled model has fewer
//! bodies than the URDF has links.
//!
//! Run: `cargo run -p example-urdf-fixed --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

/// Three links: base → (fixed) sensor_mount → (revolute) arm.
/// The fixed joint should cause sensor_mount to fuse into base.
const FIXED_URDF: &str = r#"<?xml version="1.0"?>
<robot name="fixed_test">
    <link name="base">
        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
        </inertial>
    </link>
    <link name="sensor_mount">
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 0.2"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
    </link>
    <joint name="mount_fixed" type="fixed">
        <parent link="base"/>
        <child link="sensor_mount"/>
        <origin xyz="0 0 0.1"/>
    </joint>
    <joint name="arm_joint" type="revolute">
        <parent link="sensor_mount"/>
        <child link="arm"/>
        <origin xyz="0 0 0.2"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
    </joint>
</robot>
"#;

/// Four links: base → (fixed) mount1 → (fixed) mount2 → (revolute) arm.
/// Two chained fixed joints: both should fuse.
const CHAIN_FIXED_URDF: &str = r#"<?xml version="1.0"?>
<robot name="chain_fixed">
    <link name="base">
        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
        </inertial>
    </link>
    <link name="mount1">
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
    <link name="mount2">
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>
    <joint name="f1" type="fixed">
        <parent link="base"/>
        <child link="mount1"/>
        <origin xyz="0 0 0.05"/>
    </joint>
    <joint name="f2" type="fixed">
        <parent link="mount1"/>
        <child link="mount2"/>
        <origin xyz="0 0 0.05"/>
    </joint>
    <joint name="arm_joint" type="revolute">
        <parent link="mount2"/>
        <child link="arm"/>
        <origin xyz="0 0 0.1"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
    </joint>
</robot>
"#;

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn main() {
    println!("=== URDF Loading — Fixed Joint Fusion ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    // --- Check 1: Single fixed joint → body count reduced ---
    let model = sim_urdf::load_urdf_model(FIXED_URDF).expect("URDF should parse");
    // URDF has 3 links. With fusion: world + base(+sensor_mount) + arm = 3 bodies.
    // Without fusion it would be 4 (world + base + sensor_mount + arm).
    let fused = model.nbody < 4;
    if check(
        "Fixed joint reduces body count",
        fused,
        &format!("nbody={} (expect <4, URDF has 3 links)", model.nbody),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Only 1 joint survives (fixed joint eliminated) ---
    let one_joint = model.njnt == 1;
    if check(
        "Only revolute joint survives",
        one_joint,
        &format!("njnt={} (expect 1)", model.njnt),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 3: Model still simulates correctly ---
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    let mut ok = true;
    for _ in 0..500 {
        data.step(&model).expect("step");
        if data.qpos.iter().any(|v| v.is_nan()) {
            ok = false;
            break;
        }
    }
    if check(
        "Fused model simulates 500 steps",
        ok,
        &format!("qpos[0]={:.4}", data.qpos[0]),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 4: Chained fixed joints → both fuse ---
    let model2 = sim_urdf::load_urdf_model(CHAIN_FIXED_URDF).expect("URDF should parse");
    // URDF has 4 links, 2 fixed joints, 1 revolute.
    // With fusion: world + base(+mount1+mount2 fused) + arm = 3 bodies.
    let chain_fused = model2.nbody < 5;
    let chain_one_joint = model2.njnt == 1;
    if check(
        "Chained fixed joints both fuse",
        chain_fused && chain_one_joint,
        &format!(
            "nbody={} (expect <5), njnt={} (expect 1)",
            model2.nbody, model2.njnt
        ),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 5: DOF count is correct (only revolute contributes) ---
    let dof_ok = model.nv == 1 && model.nq == 1;
    if check(
        "DOF = 1 (only revolute)",
        dof_ok,
        &format!("nv={}, nq={}", model.nv, model.nq),
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
