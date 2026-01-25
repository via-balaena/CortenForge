//! URDF → Physics pipeline integration tests.
//!
//! Tests the full pipeline from URDF robot descriptions to simulated physics.

use approx::assert_relative_eq;
use nalgebra::{Point3, UnitQuaternion};
use sim_core::{Stepper, World};
use sim_types::Pose;
use sim_urdf::load_urdf_str;

/// Test: Simple single-link URDF loads and simulates.
#[test]
fn test_single_link_urdf() {
    let urdf = r#"
        <robot name="single_link">
            <link name="base">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
        </robot>
    "#;

    let robot = load_urdf_str(urdf).expect("should parse URDF");
    assert_eq!(robot.name, "single_link");
    assert_eq!(robot.bodies.len(), 1);

    let mut world = World::default();
    let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");

    assert_eq!(world.body_count(), 1);
    assert!(spawned.link_id("base").is_some());

    // Verify the link can be retrieved and has valid properties
    let base_id = spawned.link_id("base").unwrap();
    let body = world.body(base_id).unwrap();

    // Verify mass properties
    assert_relative_eq!(body.mass_props.mass, 1.0, epsilon = 1e-6);

    // Body state should be valid
    assert!(body.state.pose.position.x.is_finite());
    assert!(body.state.pose.position.y.is_finite());
    assert!(body.state.pose.position.z.is_finite());
}

/// Test: Multi-link articulated robot (2-link pendulum).
#[test]
fn test_multi_link_articulated_robot() {
    let urdf = r#"
        <robot name="two_link_arm">
            <link name="base">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
            <link name="link1">
                <inertial>
                    <origin xyz="0 0 -0.25"/>
                    <mass value="0.5"/>
                    <inertia ixx="0.01" iyy="0.01" izz="0.001" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
            <link name="link2">
                <inertial>
                    <origin xyz="0 0 -0.25"/>
                    <mass value="0.3"/>
                    <inertia ixx="0.005" iyy="0.005" izz="0.001" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
            <joint name="joint1" type="revolute">
                <parent link="base"/>
                <child link="link1"/>
                <origin xyz="0 0 0"/>
                <axis xyz="0 1 0"/>
                <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/>
            </joint>
            <joint name="joint2" type="revolute">
                <parent link="link1"/>
                <child link="link2"/>
                <origin xyz="0 0 -0.5"/>
                <axis xyz="0 1 0"/>
                <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/>
            </joint>
        </robot>
    "#;

    let robot = load_urdf_str(urdf).expect("should parse URDF");
    assert_eq!(robot.name, "two_link_arm");
    assert_eq!(robot.bodies.len(), 3);
    assert_eq!(robot.joints.len(), 2);

    let mut world = World::default();
    let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");

    assert_eq!(world.body_count(), 3);
    assert_eq!(world.joint_count(), 2);

    assert!(spawned.link_id("base").is_some());
    assert!(spawned.link_id("link1").is_some());
    assert!(spawned.link_id("link2").is_some());
    assert!(spawned.joint_id("joint1").is_some());
    assert!(spawned.joint_id("joint2").is_some());

    // Simulate and verify links move (pendulum swings under gravity)
    let link2_id = spawned.link_id("link2").unwrap();
    let initial_z = world.body(link2_id).unwrap().state.pose.position.z;

    let mut stepper = Stepper::new();
    stepper.run_for(&mut world, 0.5).expect("should simulate");

    // After simulation, the pendulum should have moved
    let final_state = world.body(link2_id).unwrap();
    let position_changed = (final_state.state.pose.position.z - initial_z).abs() > 1e-6;

    assert!(position_changed, "Pendulum should move under gravity");
}

/// Test: Robot with revolute and prismatic joints.
#[test]
fn test_revolute_and_prismatic_joints() {
    let urdf = r#"
        <robot name="mixed_joints">
            <link name="base">
                <inertial>
                    <mass value="10.0"/>
                    <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
            <link name="slider">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
            <link name="rotator">
                <inertial>
                    <mass value="0.5"/>
                    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
            <joint name="slide" type="prismatic">
                <parent link="base"/>
                <child link="slider"/>
                <origin xyz="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1.0" upper="1.0" effort="100" velocity="1"/>
            </joint>
            <joint name="rotate" type="revolute">
                <parent link="slider"/>
                <child link="rotator"/>
                <origin xyz="0 0 0.5"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/>
            </joint>
        </robot>
    "#;

    let robot = load_urdf_str(urdf).expect("should parse URDF");
    assert_eq!(robot.joints.len(), 2);

    let mut world = World::default();
    let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");

    assert_eq!(world.body_count(), 3);
    assert_eq!(world.joint_count(), 2);

    // Verify joints are accessible
    assert!(spawned.joint_id("slide").is_some());
    assert!(spawned.joint_id("rotate").is_some());

    // Verify joint types by checking the joints in the world
    let slide_id = spawned.joint_id("slide").unwrap();
    let rotate_id = spawned.joint_id("rotate").unwrap();

    let slide_joint = world.joint(slide_id).unwrap();
    let rotate_joint = world.joint(rotate_id).unwrap();

    assert!(
        matches!(slide_joint.joint_type, sim_types::JointType::Prismatic),
        "slide should be prismatic"
    );
    assert!(
        matches!(rotate_joint.joint_type, sim_types::JointType::Revolute),
        "rotate should be revolute"
    );
}

/// Test: Robot with joint limits.
#[test]
fn test_joint_limits() {
    let urdf = r#"
        <robot name="limited_robot">
            <link name="base">
                <inertial>
                    <mass value="10.0"/>
                    <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
            <link name="arm">
                <inertial>
                    <origin xyz="0.25 0 0"/>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
            <joint name="shoulder" type="revolute">
                <parent link="base"/>
                <child link="arm"/>
                <origin xyz="0 0 0"/>
                <axis xyz="0 1 0"/>
                <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
            </joint>
        </robot>
    "#;

    let robot = load_urdf_str(urdf).expect("should parse URDF");

    let mut world = World::default();
    let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");

    // Get joint and verify limits
    let joint_id = spawned.joint_id("shoulder").unwrap();
    let joint = world.joint(joint_id).unwrap();

    // Joint should have limits
    assert!(joint.limits.is_some(), "Joint should have limits");
    let limits = joint.limits.as_ref().unwrap();
    assert_relative_eq!(limits.lower(), -1.57, epsilon = 0.01);
    assert_relative_eq!(limits.upper(), 1.57, epsilon = 0.01);

    // Joint should start at zero position
    assert_relative_eq!(joint.state.position, 0.0, epsilon = 1e-6);
}

/// Test: URDF with visual and collision geometry (parsing only).
#[test]
fn test_urdf_with_geometry() {
    let urdf = r#"
        <robot name="geometric_robot">
            <link name="base">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
                <visual>
                    <geometry>
                        <box size="0.5 0.5 0.1"/>
                    </geometry>
                </visual>
                <collision>
                    <geometry>
                        <box size="0.5 0.5 0.1"/>
                    </geometry>
                </collision>
            </link>
            <link name="sphere_link">
                <inertial>
                    <mass value="0.5"/>
                    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
                </inertial>
                <visual>
                    <geometry>
                        <sphere radius="0.1"/>
                    </geometry>
                </visual>
                <collision>
                    <geometry>
                        <sphere radius="0.1"/>
                    </geometry>
                </collision>
            </link>
            <joint name="attach" type="fixed">
                <parent link="base"/>
                <child link="sphere_link"/>
                <origin xyz="0 0 0.2"/>
            </joint>
        </robot>
    "#;

    let robot = load_urdf_str(urdf).expect("should parse URDF with geometry");
    assert_eq!(robot.bodies.len(), 2);
    assert_eq!(robot.joints.len(), 1);

    let mut world = World::default();
    let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");

    assert_eq!(world.body_count(), 2);
    assert!(spawned.link_id("base").is_some());
    assert!(spawned.link_id("sphere_link").is_some());
}

/// Test: Spawning robot at custom pose.
#[test]
fn test_spawn_at_custom_pose() {
    let urdf = r#"
        <robot name="positioned">
            <link name="base">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
        </robot>
    "#;

    let robot = load_urdf_str(urdf).expect("should parse");

    let custom_pose = Pose::from_position_rotation(
        Point3::new(5.0, 3.0, 2.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
    );

    let mut world = World::default();
    let spawned = robot
        .spawn_into(&mut world, custom_pose)
        .expect("should spawn at custom pose");

    let base_id = spawned.link_id("base").unwrap();
    let body = world.body(base_id).unwrap();

    assert_relative_eq!(body.state.pose.position.x, 5.0, epsilon = 1e-6);
    assert_relative_eq!(body.state.pose.position.y, 3.0, epsilon = 1e-6);
    assert_relative_eq!(body.state.pose.position.z, 2.0, epsilon = 1e-6);
}
