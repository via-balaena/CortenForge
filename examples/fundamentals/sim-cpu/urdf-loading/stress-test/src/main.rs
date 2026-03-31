//! Stress test — headless validation of the entire sim-urdf pipeline.
//!
//! Tests every URDF feature against the engine: joint types, geometry conversion,
//! inertia, limits, damping, frictionloss, mimic joints, planar joints, dynamic
//! equivalence with hand-written MJCF, and error handling.
//!
//! Run: `cargo run -p example-urdf-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::needless_late_init,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::approx_constant
)]

use sim_core::{GeomType, MjJointType};

// ============================================================================
// URDF Models
// ============================================================================

/// Simple single-link robot (base only, no joints).
const SINGLE_LINK_URDF: &str = r#"<?xml version="1.0"?>
<robot name="single_link">
    <link name="base_link">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
    </link>
</robot>
"#;

/// 3-DOF arm: revolute shoulder, revolute elbow, prismatic gripper.
const ARM_URDF: &str = r#"<?xml version="1.0"?>
<robot name="test_arm">
    <link name="base_link">
        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
        </inertial>
    </link>

    <link name="upper_arm">
        <inertial>
            <origin xyz="0 0 0.25"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.005"/>
        </inertial>
        <collision>
            <origin xyz="0 0 0.25"/>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
        </collision>
    </link>

    <link name="forearm">
        <inertial>
            <origin xyz="0 0 0.2"/>
            <mass value="0.5"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.002"/>
        </inertial>
        <collision>
            <origin xyz="0 0 0.2"/>
            <geometry>
                <box size="0.06 0.06 0.4"/>
            </geometry>
        </collision>
    </link>

    <link name="gripper">
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
        <collision>
            <geometry>
                <sphere radius="0.03"/>
            </geometry>
        </collision>
    </link>

    <joint name="shoulder" type="revolute">
        <parent link="base_link"/>
        <child link="upper_arm"/>
        <origin xyz="0 0 0.1"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="2"/>
    </joint>

    <joint name="elbow" type="revolute">
        <parent link="upper_arm"/>
        <child link="forearm"/>
        <origin xyz="0 0 0.5"/>
        <axis xyz="0 1 0"/>
        <limit lower="-2.0" upper="2.0" effort="50" velocity="3"/>
    </joint>

    <joint name="grip" type="prismatic">
        <parent link="forearm"/>
        <child link="gripper"/>
        <origin xyz="0 0 0.4"/>
        <axis xyz="0 0 1"/>
        <limit lower="0" upper="0.05" effort="10" velocity="0.5"/>
    </joint>
</robot>
"#;

/// Equivalent MJCF for the same arm (for dynamic equivalence checks).
const ARM_MJCF: &str = r#"
<mujoco model="test_arm">
    <option gravity="0 0 -9.81" timestep="0.002"/>
    <compiler angle="radian" eulerseq="xyz" fusestatic="true" discardvisual="true"/>

    <worldbody>
        <body name="base_link" pos="0 0 0">
            <inertial mass="5.0" diaginertia="0.5 0.5 0.5"/>
            <body name="upper_arm" pos="0 0 0.1">
                <joint name="shoulder" type="hinge" axis="0 1 0" limited="true" range="-3.14 3.14"/>
                <inertial mass="1.0" pos="0 0 0.25" diaginertia="0.02 0.02 0.005"/>
                <geom type="cylinder" size="0.04 0.25" pos="0 0 0.25"/>
                <body name="forearm" pos="0 0 0.5">
                    <joint name="elbow" type="hinge" axis="0 1 0" limited="true" range="-2.0 2.0"/>
                    <inertial mass="0.5" pos="0 0 0.2" diaginertia="0.01 0.01 0.002"/>
                    <geom type="box" size="0.03 0.03 0.2" pos="0 0 0.2"/>
                    <body name="gripper" pos="0 0 0.4">
                        <joint name="grip" type="slide" axis="0 0 1" limited="true" range="0 0.05"/>
                        <inertial mass="0.2" diaginertia="0.001 0.001 0.001"/>
                        <geom type="sphere" size="0.03"/>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>
</mujoco>
"#;

/// Continuous (unlimited revolute) joint.
const CONTINUOUS_URDF: &str = r#"<?xml version="1.0"?>
<robot name="wheel">
    <link name="axle">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
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

/// Fixed joint — child should fuse into parent with fusestatic.
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
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
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

/// Floating joint (6 DOF).
const FLOATING_URDF: &str = r#"<?xml version="1.0"?>
<robot name="floating_test">
    <link name="world">
        <inertial>
            <mass value="100.0"/>
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10"/>
        </inertial>
    </link>
    <link name="floating_body">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>
    <joint name="float" type="floating">
        <parent link="world"/>
        <child link="floating_body"/>
    </joint>
</robot>
"#;

/// Geometry types: box, sphere, cylinder on the same arm.
const GEOMETRY_URDF: &str = r#"<?xml version="1.0"?>
<robot name="geometry_test">
    <link name="base">
        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
        </inertial>
        <collision>
            <geometry>
                <box size="0.2 0.4 0.6"/>
            </geometry>
        </collision>
    </link>
    <link name="link1">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <geometry>
                <sphere radius="0.15"/>
            </geometry>
        </collision>
    </link>
    <link name="link2">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <geometry>
                <cylinder radius="0.08" length="0.3"/>
            </geometry>
        </collision>
    </link>
    <joint name="j1" type="revolute">
        <parent link="base"/>
        <child link="link1"/>
        <origin xyz="0 0 0.3"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
    </joint>
    <joint name="j2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0 0 0.3"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
    </joint>
</robot>
"#;

/// Off-diagonal inertia (asymmetric body). Uses floating joint to prevent fusion.
const FULL_INERTIA_URDF: &str = r#"<?xml version="1.0"?>
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

/// Diagonal inertia for comparison. Uses floating joint to prevent fusion.
const DIAG_INERTIA_URDF: &str = r#"<?xml version="1.0"?>
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

/// Joint with damping.
const DAMPING_URDF: &str = r#"<?xml version="1.0"?>
<robot name="damping_test">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 0.2"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
    </link>
    <joint name="hinge" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <origin xyz="0 0 0.1"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
        <dynamics damping="0.5"/>
    </joint>
</robot>
"#;

/// Joint with friction (tests frictionloss mapping — Fix 1 in spec).
const FRICTION_URDF: &str = r#"<?xml version="1.0"?>
<robot name="friction_test">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 0.2"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
    </link>
    <joint name="hinge" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <origin xyz="0 0 0.1"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
        <dynamics damping="0" friction="0.5"/>
    </joint>
</robot>
"#;

/// Mimic joint (tests Fix 3 in spec).
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

/// Planar joint (tests Fix 4 in spec — should be 3 DOF).
const PLANAR_URDF: &str = r#"<?xml version="1.0"?>
<robot name="planar_test">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="slider">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>
    <joint name="planar" type="planar">
        <parent link="base"/>
        <child link="slider"/>
        <axis xyz="0 0 1"/>
    </joint>
</robot>
"#;

/// Mesh geometry (tests Fix 2 in spec — currently silently dropped).
const MESH_URDF: &str = r#"<?xml version="1.0"?>
<robot name="mesh_test">
    <link name="base">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <geometry>
                <mesh filename="dummy.stl"/>
            </geometry>
        </collision>
    </link>
</robot>
"#;

// ============================================================================
// Helpers
// ============================================================================

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

// ============================================================================
// Group 1: Basic loading
// ============================================================================

fn check_1_single_link_loads() -> (u32, u32) {
    let result = sim_urdf::load_urdf_model(SINGLE_LINK_URDF);
    let ok = result.is_ok();
    let detail = if ok {
        let m = result.expect("just checked");
        format!("name={}, nbody={}", m.name, m.nbody)
    } else {
        format!("ERROR: {}", result.unwrap_err())
    };
    let p = check("Single link loads", ok, &detail);
    (u32::from(p), 1)
}

fn check_2_arm_loads() -> (u32, u32) {
    let result = sim_urdf::load_urdf_model(ARM_URDF);
    let ok = result.is_ok();
    let detail = if ok {
        let m = result.expect("just checked");
        format!(
            "name={}, nbody={}, njnt={}, nv={}, ngeom={}",
            m.name, m.nbody, m.njnt, m.nv, m.ngeom
        )
    } else {
        format!("ERROR: {}", result.unwrap_err())
    };
    let p = check("3-DOF arm loads", ok, &detail);
    (u32::from(p), 1)
}

fn check_3_arm_steps() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(ARM_URDF).expect("parse");
    let mut data = model.make_data();
    data.qpos[0] = 0.5; // shoulder at 0.5 rad
    data.forward(&model).expect("forward");

    let mut ok = true;
    for i in 0..500 {
        if let Err(e) = data.step(&model) {
            let p = check("Arm steps 500", false, &format!("failed at step {i}: {e}"));
            return (u32::from(p), 1);
        }
        // Check for NaN
        if data.qpos.iter().any(|v| v.is_nan()) {
            ok = false;
            break;
        }
    }

    let p = check(
        "Arm steps 500",
        ok,
        &format!(
            "qpos=[{:.4}, {:.4}, {:.4}]",
            data.qpos[0], data.qpos[1], data.qpos[2]
        ),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 2: Joint types
// ============================================================================

fn check_4_revolute_joint() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(ARM_URDF).expect("parse");

    // First joint should be hinge
    let is_hinge = model.jnt_type[0] == MjJointType::Hinge;
    let is_limited = model.jnt_limited[0];
    let range = model.jnt_range[0];
    let range_ok = (range.0 - (-3.14)).abs() < 0.01 && (range.1 - 3.14).abs() < 0.01;

    let pass = is_hinge && is_limited && range_ok;
    let p = check(
        "Revolute → hinge",
        pass,
        &format!(
            "type={:?}, limited={is_limited}, range=({:.2}, {:.2})",
            model.jnt_type[0], range.0, range.1
        ),
    );
    (u32::from(p), 1)
}

fn check_5_prismatic_joint() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(ARM_URDF).expect("parse");

    // Third joint (index 2) should be slide
    let jnt_idx = 2;
    let is_slide = model.jnt_type[jnt_idx] == MjJointType::Slide;
    let is_limited = model.jnt_limited[jnt_idx];
    let range = model.jnt_range[jnt_idx];
    let range_ok = (range.0).abs() < 0.001 && (range.1 - 0.05).abs() < 0.001;

    let pass = is_slide && is_limited && range_ok;
    let p = check(
        "Prismatic → slide",
        pass,
        &format!(
            "type={:?}, limited={is_limited}, range=({:.4}, {:.4})",
            model.jnt_type[jnt_idx], range.0, range.1
        ),
    );
    (u32::from(p), 1)
}

fn check_6_continuous_joint() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(CONTINUOUS_URDF).expect("parse");

    let is_hinge = model.jnt_type[0] == MjJointType::Hinge;
    let is_unlimited = !model.jnt_limited[0];

    let pass = is_hinge && is_unlimited;
    let p = check(
        "Continuous → unlimited hinge",
        pass,
        &format!(
            "type={:?}, limited={}",
            model.jnt_type[0], model.jnt_limited[0]
        ),
    );
    (u32::from(p), 1)
}

fn check_7_fixed_joint_fuses() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(FIXED_URDF).expect("parse");

    // URDF has 3 links: base, sensor_mount (fixed), arm
    // With fusestatic=true, sensor_mount should fuse into base.
    // Result: world + base(+sensor_mount fused) + arm = 3 bodies
    // But only 1 joint (arm_joint), since mount_fixed is eliminated.
    let joint_ok = model.njnt == 1;
    // nbody should be less than 4 (world + 3 links) due to fusion
    let fused = model.nbody < 4;

    let pass = joint_ok && fused;
    let p = check(
        "Fixed joint fuses",
        pass,
        &format!(
            "nbody={} (expect <4), njnt={} (expect 1)",
            model.nbody, model.njnt
        ),
    );
    (u32::from(p), 1)
}

fn check_8_floating_joint() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(FLOATING_URDF).expect("parse");

    let is_free = model.jnt_type[0] == MjJointType::Free;
    // Free joint: nq=7 (pos3 + quat4), nv=6 (lin3 + ang3)
    let dof_ok = model.nv >= 6;

    let pass = is_free && dof_ok;
    let p = check(
        "Floating → free joint",
        pass,
        &format!(
            "type={:?}, nv={}, nq={}",
            model.jnt_type[0], model.nv, model.nq
        ),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 3: Structural equivalence (URDF vs hand-written MJCF)
// ============================================================================

fn check_9_structural_equivalence() -> (u32, u32) {
    let urdf_model = sim_urdf::load_urdf_model(ARM_URDF).expect("urdf parse");
    let mjcf_model = sim_mjcf::load_model(ARM_MJCF).expect("mjcf parse");

    let nbody_eq = urdf_model.nbody == mjcf_model.nbody;
    let njnt_eq = urdf_model.njnt == mjcf_model.njnt;
    let nv_eq = urdf_model.nv == mjcf_model.nv;
    let nq_eq = urdf_model.nq == mjcf_model.nq;

    let pass = nbody_eq && njnt_eq && nv_eq && nq_eq;
    let p = check(
        "Structural equivalence",
        pass,
        &format!(
            "URDF(nbody={}, njnt={}, nv={}, nq={}) vs MJCF(nbody={}, njnt={}, nv={}, nq={})",
            urdf_model.nbody,
            urdf_model.njnt,
            urdf_model.nv,
            urdf_model.nq,
            mjcf_model.nbody,
            mjcf_model.njnt,
            mjcf_model.nv,
            mjcf_model.nq,
        ),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 4: Dynamic equivalence
// ============================================================================

fn check_10_dynamic_equivalence_qpos() -> (u32, u32) {
    let urdf_model = sim_urdf::load_urdf_model(ARM_URDF).expect("urdf parse");
    let mjcf_model = sim_mjcf::load_model(ARM_MJCF).expect("mjcf parse");

    let mut urdf_data = urdf_model.make_data();
    let mut mjcf_data = mjcf_model.make_data();

    // Same initial conditions
    urdf_data.qpos[0] = 0.5;
    mjcf_data.qpos[0] = 0.5;
    urdf_data.forward(&urdf_model).expect("forward");
    mjcf_data.forward(&mjcf_model).expect("forward");

    // Step 500 times (1 second at dt=0.002)
    for _ in 0..500 {
        urdf_data.step(&urdf_model).expect("step");
        mjcf_data.step(&mjcf_model).expect("step");
    }

    let mut max_diff: f64 = 0.0;
    let nq = urdf_model.nq.min(mjcf_model.nq);
    for i in 0..nq {
        let diff = (urdf_data.qpos[i] - mjcf_data.qpos[i]).abs();
        max_diff = max_diff.max(diff);
    }

    let pass = max_diff < 1e-6;
    let p = check(
        "Dynamic equivalence (qpos)",
        pass,
        &format!("max |qpos_urdf - qpos_mjcf| = {max_diff:.2e} (need <1e-6)",),
    );
    (u32::from(p), 1)
}

fn check_11_dynamic_equivalence_qvel() -> (u32, u32) {
    let urdf_model = sim_urdf::load_urdf_model(ARM_URDF).expect("urdf parse");
    let mjcf_model = sim_mjcf::load_model(ARM_MJCF).expect("mjcf parse");

    let mut urdf_data = urdf_model.make_data();
    let mut mjcf_data = mjcf_model.make_data();

    urdf_data.qpos[0] = 0.5;
    mjcf_data.qpos[0] = 0.5;
    urdf_data.forward(&urdf_model).expect("forward");
    mjcf_data.forward(&mjcf_model).expect("forward");

    for _ in 0..500 {
        urdf_data.step(&urdf_model).expect("step");
        mjcf_data.step(&mjcf_model).expect("step");
    }

    let mut max_diff: f64 = 0.0;
    let nv = urdf_model.nv.min(mjcf_model.nv);
    for i in 0..nv {
        let diff = (urdf_data.qvel[i] - mjcf_data.qvel[i]).abs();
        max_diff = max_diff.max(diff);
    }

    let pass = max_diff < 1e-6;
    let p = check(
        "Dynamic equivalence (qvel)",
        pass,
        &format!("max |qvel_urdf - qvel_mjcf| = {max_diff:.2e} (need <1e-6)",),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 5: Geometry conversion
// ============================================================================

fn check_12_box_half_extents() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(GEOMETRY_URDF).expect("parse");

    // Find box geom (base link collision)
    for i in 0..model.ngeom {
        if model.geom_type[i] == GeomType::Box {
            let s = model.geom_size[i];
            // URDF box size="0.2 0.4 0.6" → MJCF half-extents 0.1, 0.2, 0.3
            let pass =
                (s.x - 0.1).abs() < 0.001 && (s.y - 0.2).abs() < 0.001 && (s.z - 0.3).abs() < 0.001;
            let p = check(
                "Box half-extents",
                pass,
                &format!(
                    "size=({:.3}, {:.3}, {:.3}), expect (0.1, 0.2, 0.3)",
                    s.x, s.y, s.z
                ),
            );
            return (u32::from(p), 1);
        }
    }
    let p = check("Box half-extents", false, "no box geom found");
    (u32::from(p), 1)
}

fn check_13_sphere_radius() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(GEOMETRY_URDF).expect("parse");

    for i in 0..model.ngeom {
        if model.geom_type[i] == GeomType::Sphere {
            let r = model.geom_size[i].x;
            let pass = (r - 0.15).abs() < 0.001;
            let p = check("Sphere radius", pass, &format!("r={r:.3}, expect 0.15"));
            return (u32::from(p), 1);
        }
    }
    let p = check("Sphere radius", false, "no sphere geom found");
    (u32::from(p), 1)
}

fn check_14_cylinder_conversion() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(GEOMETRY_URDF).expect("parse");

    for i in 0..model.ngeom {
        if model.geom_type[i] == GeomType::Cylinder {
            let s = model.geom_size[i];
            // URDF cylinder radius=0.08 length=0.3 → MJCF size="0.08 0.15" (radius, half_length)
            let r_ok = (s.x - 0.08).abs() < 0.001;
            let hl_ok = (s.y - 0.15).abs() < 0.001;
            let pass = r_ok && hl_ok;
            let p = check(
                "Cylinder conversion",
                pass,
                &format!("size=({:.3}, {:.3}), expect (0.08, 0.15)", s.x, s.y),
            );
            return (u32::from(p), 1);
        }
    }
    let p = check("Cylinder conversion", false, "no cylinder geom found");
    (u32::from(p), 1)
}

// ============================================================================
// Group 6: Inertia
// ============================================================================

fn check_15_diagonal_inertia() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(DIAG_INERTIA_URDF).expect("parse");

    // body 0 is world, body 1 is "base"
    let mass = model.body_mass[1];
    let inertia = model.body_inertia[1];
    let mass_ok = (mass - 2.0).abs() < 0.001;
    let i_ok = (inertia.x - 0.1).abs() < 0.001
        && (inertia.y - 0.2).abs() < 0.001
        && (inertia.z - 0.3).abs() < 0.001;

    let pass = mass_ok && i_ok;
    let p = check(
        "Diagonal inertia",
        pass,
        &format!(
            "mass={mass:.3}, diaginertia=({:.3}, {:.3}, {:.3})",
            inertia.x, inertia.y, inertia.z
        ),
    );
    (u32::from(p), 1)
}

fn check_16_full_inertia_loads() -> (u32, u32) {
    let result = sim_urdf::load_urdf_model(FULL_INERTIA_URDF);
    let ok = result.is_ok();
    let detail = if ok {
        let m = result.expect("just checked");
        format!(
            "mass={:.3}, diaginertia=({:.3}, {:.3}, {:.3})",
            m.body_mass[1], m.body_inertia[1].x, m.body_inertia[1].y, m.body_inertia[1].z
        )
    } else {
        format!("ERROR: {}", result.unwrap_err())
    };
    let p = check("Full (off-diagonal) inertia loads", ok, &detail);
    (u32::from(p), 1)
}

fn check_17_full_inertia_mjcf_has_fullinertia() -> (u32, u32) {
    let mjcf = sim_urdf::urdf_to_mjcf(FULL_INERTIA_URDF).expect("convert");
    let has_full = mjcf.contains("fullinertia");
    let p = check(
        "Full inertia → fullinertia in MJCF",
        has_full,
        &format!("MJCF contains 'fullinertia': {has_full}"),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 7: Limits and dynamics
// ============================================================================

fn check_18_limits_propagate() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(ARM_URDF).expect("parse");

    // Check elbow joint (index 1): range should be [-2.0, 2.0]
    let limited = model.jnt_limited[1];
    let range = model.jnt_range[1];
    let pass = limited && (range.0 - (-2.0)).abs() < 0.01 && (range.1 - 2.0).abs() < 0.01;
    let p = check(
        "Joint limits propagate",
        pass,
        &format!("limited={limited}, range=({:.2}, {:.2})", range.0, range.1),
    );
    (u32::from(p), 1)
}

fn check_19_damping_propagates() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(DAMPING_URDF).expect("parse");

    let damping = model.jnt_damping[0];
    let pass = (damping - 0.5).abs() < 0.001;
    let p = check(
        "Joint damping propagates",
        pass,
        &format!("damping={damping:.3}, expect 0.5"),
    );
    (u32::from(p), 1)
}

fn check_20_damping_dissipates_energy() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(DAMPING_URDF).expect("parse");
    let mut data = model.make_data();

    // Start at 45 degrees
    data.qpos[0] = std::f64::consts::FRAC_PI_4;
    data.forward(&model).expect("forward");
    let vel_initial_after_100steps;

    // Step 100 times to get moving
    for _ in 0..100 {
        data.step(&model).expect("step");
    }
    vel_initial_after_100steps = data.qvel[0].abs();

    // Step 1000 more — damping should reduce velocity
    for _ in 0..1000 {
        data.step(&model).expect("step");
    }
    let vel_final = data.qvel[0].abs();

    // Damped system should have lower peak velocity over time
    // (comparing at arbitrary points, the max velocity should decrease)
    let pass = vel_final < vel_initial_after_100steps || vel_final < 0.1;
    let p = check(
        "Damping dissipates energy",
        pass,
        &format!("vel@100steps={vel_initial_after_100steps:.4}, vel@1100steps={vel_final:.4}"),
    );
    (u32::from(p), 1)
}

fn check_21_frictionloss_maps() -> (u32, u32) {
    // Check that URDF friction appears as frictionloss in MJCF output
    let mjcf = sim_urdf::urdf_to_mjcf(FRICTION_URDF).expect("convert");
    let has_frictionloss = mjcf.contains("frictionloss");

    let p = check(
        "Friction → frictionloss in MJCF",
        has_frictionloss,
        &format!("MJCF contains 'frictionloss': {has_frictionloss}"),
    );
    (u32::from(p), 1)
}

fn check_22_frictionloss_in_model() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(FRICTION_URDF).expect("parse");

    // dof_frictionloss should have the friction value
    let fl = model.dof_frictionloss[0];
    let pass = (fl - 0.5).abs() < 0.001;
    let p = check(
        "Frictionloss in model",
        pass,
        &format!("dof_frictionloss[0]={fl:.3}, expect 0.5"),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 8: Mimic joints (Fix 3 — expected to fail before implementation)
// ============================================================================

fn check_23_mimic_parses() -> (u32, u32) {
    // Just check that the URDF loads without error.
    // The mimic element is currently ignored, so this should still load.
    let result = sim_urdf::load_urdf_model(MIMIC_URDF);
    let ok = result.is_ok();
    let p = check(
        "Mimic URDF loads",
        ok,
        &if ok {
            "loaded (mimic may be ignored)".into()
        } else {
            format!("ERROR: {}", result.unwrap_err())
        },
    );
    (u32::from(p), 1)
}

fn check_24_mimic_creates_equality() -> (u32, u32) {
    let mjcf = sim_urdf::urdf_to_mjcf(MIMIC_URDF).expect("convert");
    let has_equality = mjcf.contains("<equality>");
    let has_joint_eq = mjcf.contains("polycoef");

    let pass = has_equality && has_joint_eq;
    let p = check(
        "Mimic → equality constraint in MJCF",
        pass,
        &format!("has <equality>: {has_equality}, has polycoef: {has_joint_eq}"),
    );
    (u32::from(p), 1)
}

fn check_25_mimic_tracks() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(MIMIC_URDF).expect("parse");

    // Check if equality constraints exist
    if model.neq == 0 {
        let p = check(
            "Mimic joint tracks leader",
            false,
            "no equality constraints in model (mimic not wired)",
        );
        return (u32::from(p), 1);
    }

    let mut data = model.make_data();
    // Drive leader joint to 0.5 rad
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    // Step to let constraint settle
    for _ in 0..1000 {
        data.step(&model).expect("step");
    }

    // Follower should be at multiplier * leader + offset = 2.0 * 0.5 + 0.1 = 1.1
    // But dynamics will evolve, so check the constraint relationship
    let leader = data.qpos[0];
    let follower = data.qpos[1];
    let expected = 2.0 * leader + 0.1;
    let err = (follower - expected).abs();

    let pass = err < 0.05; // within solver tolerance
    let p = check(
        "Mimic joint tracks leader",
        pass,
        &format!(
            "leader={leader:.4}, follower={follower:.4}, expected={expected:.4}, err={err:.4}"
        ),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 9: Planar joints (Fix 4 — expected to fail before implementation)
// ============================================================================

fn check_26_planar_dof() -> (u32, u32) {
    let model = sim_urdf::load_urdf_model(PLANAR_URDF).expect("parse");

    // Planar should be 3 DOF (2 slide + 1 hinge)
    // Currently approximated as 1 DOF hinge
    let pass = model.nv == 3;
    let p = check(
        "Planar joint → 3 DOF",
        pass,
        &format!("nv={} (expect 3), njnt={}", model.nv, model.njnt),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 10: Mesh geometry (Fix 2 — expected to fail before implementation)
// ============================================================================

fn check_27_mesh_not_silently_dropped() -> (u32, u32) {
    let mjcf = sim_urdf::urdf_to_mjcf(MESH_URDF).expect("convert");

    // Check that mesh generates an <asset> section and a mesh geom
    let has_asset = mjcf.contains("<asset>");
    let has_mesh_geom = mjcf.contains(r#"type="mesh""#);

    let pass = has_asset && has_mesh_geom;
    let p = check(
        "Mesh not silently dropped",
        pass,
        &format!("has <asset>: {has_asset}, has mesh geom: {has_mesh_geom}"),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Group 11: Error handling
// ============================================================================

fn check_28_error_missing_robot() -> (u32, u32) {
    let result = sim_urdf::parse_urdf_str("<link name='test'/>");
    let pass = result.is_err();
    let p = check(
        "Missing <robot> → error",
        pass,
        &if pass {
            "correct: error returned".to_string()
        } else {
            "BUG: should have failed".to_string()
        },
    );
    (u32::from(p), 1)
}

fn check_29_error_unknown_joint_type() -> (u32, u32) {
    let result = sim_urdf::parse_urdf_str(
        r#"
        <robot name="test">
            <link name="a"/>
            <link name="b"/>
            <joint name="j" type="invalid">
                <parent link="a"/>
                <child link="b"/>
            </joint>
        </robot>
    "#,
    );
    let pass = result.is_err();
    let p = check(
        "Unknown joint type → error",
        pass,
        &if pass {
            format!("correct: {}", result.unwrap_err())
        } else {
            "BUG: should have failed".into()
        },
    );
    (u32::from(p), 1)
}

fn check_30_error_undefined_link() -> (u32, u32) {
    let robot = sim_urdf::UrdfRobot::new("test")
        .with_link(sim_urdf::UrdfLink::new("base"))
        .with_joint(sim_urdf::UrdfJoint::new(
            "j1",
            sim_urdf::UrdfJointType::Fixed,
            "base",
            "nonexistent",
        ));
    let result = sim_urdf::validate(&robot);
    let pass = result.is_err();
    let p = check(
        "Undefined link → error",
        pass,
        &if pass {
            format!("correct: {}", result.unwrap_err())
        } else {
            "BUG: should have failed".into()
        },
    );
    (u32::from(p), 1)
}

fn check_31_urdf_to_mjcf_inspectable() -> (u32, u32) {
    let mjcf = sim_urdf::urdf_to_mjcf(ARM_URDF).expect("convert");

    // Verify the intermediate MJCF is well-formed and inspectable
    let has_mujoco = mjcf.contains("<mujoco");
    let has_worldbody = mjcf.contains("<worldbody>");
    let has_joint = mjcf.contains("<joint");

    let pass = has_mujoco && has_worldbody && has_joint;
    let p = check(
        "urdf_to_mjcf() inspectable",
        pass,
        &format!("has <mujoco>: {has_mujoco}, <worldbody>: {has_worldbody}, <joint>: {has_joint}"),
    );
    (u32::from(p), 1)
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    println!("=== URDF Loading — Stress Test ===\n");

    let mut total = 0u32;
    let mut passed = 0u32;

    macro_rules! run {
        ($label:expr, $fn:expr) => {{
            println!("-- {} --", $label);
            let (p, t) = $fn;
            passed += p;
            total += t;
            println!();
        }};
    }

    // Group 1: Basic loading
    run!("1. Single link loads", check_1_single_link_loads());
    run!("2. 3-DOF arm loads", check_2_arm_loads());
    run!("3. Arm steps 500", check_3_arm_steps());

    // Group 2: Joint types
    run!("4. Revolute → hinge", check_4_revolute_joint());
    run!("5. Prismatic → slide", check_5_prismatic_joint());
    run!(
        "6. Continuous → unlimited hinge",
        check_6_continuous_joint()
    );
    run!("7. Fixed joint fuses", check_7_fixed_joint_fuses());
    run!("8. Floating → free joint", check_8_floating_joint());

    // Group 3: Structural equivalence
    run!(
        "9. Structural equivalence",
        check_9_structural_equivalence()
    );

    // Group 4: Dynamic equivalence
    run!(
        "10. Dynamic equivalence (qpos)",
        check_10_dynamic_equivalence_qpos()
    );
    run!(
        "11. Dynamic equivalence (qvel)",
        check_11_dynamic_equivalence_qvel()
    );

    // Group 5: Geometry
    run!("12. Box half-extents", check_12_box_half_extents());
    run!("13. Sphere radius", check_13_sphere_radius());
    run!("14. Cylinder conversion", check_14_cylinder_conversion());

    // Group 6: Inertia
    run!("15. Diagonal inertia", check_15_diagonal_inertia());
    run!("16. Full inertia loads", check_16_full_inertia_loads());
    run!(
        "17. Full inertia → fullinertia",
        check_17_full_inertia_mjcf_has_fullinertia()
    );

    // Group 7: Limits & dynamics
    run!("18. Joint limits propagate", check_18_limits_propagate());
    run!(
        "19. Joint damping propagates",
        check_19_damping_propagates()
    );
    run!(
        "20. Damping dissipates energy",
        check_20_damping_dissipates_energy()
    );
    run!(
        "21. Friction → frictionloss (MJCF)",
        check_21_frictionloss_maps()
    );
    run!(
        "22. Frictionloss in model",
        check_22_frictionloss_in_model()
    );

    // Group 8: Mimic joints (Fix 3)
    run!("23. Mimic URDF loads", check_23_mimic_parses());
    run!(
        "24. Mimic → equality constraint",
        check_24_mimic_creates_equality()
    );
    run!("25. Mimic joint tracks leader", check_25_mimic_tracks());

    // Group 9: Planar joints (Fix 4)
    run!("26. Planar → 3 DOF", check_26_planar_dof());

    // Group 10: Mesh (Fix 2)
    run!(
        "27. Mesh not silently dropped",
        check_27_mesh_not_silently_dropped()
    );

    // Group 11: Error handling
    run!(
        "28. Missing <robot> → error",
        check_28_error_missing_robot()
    );
    run!(
        "29. Unknown joint type → error",
        check_29_error_unknown_joint_type()
    );
    run!(
        "30. Undefined link → error",
        check_30_error_undefined_link()
    );
    run!(
        "31. urdf_to_mjcf() inspectable",
        check_31_urdf_to_mjcf_inspectable()
    );

    // Summary
    println!("============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!(
            "  {} FAILED — these indicate spec gaps to fix",
            total - passed
        );
        std::process::exit(1);
    }
}
