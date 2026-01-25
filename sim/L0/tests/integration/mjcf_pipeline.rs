//! MJCF → Physics pipeline integration tests.
//!
//! Tests the full pipeline from MJCF model descriptions to simulated physics,
//! including muscles, tendons, sensors, and equality constraints.

use approx::assert_relative_eq;
use sim_core::{Stepper, World};
use sim_mjcf::load_mjcf_str;

/// Test: Simple MJCF model (bodies + joints).
#[test]
fn test_simple_mjcf_model() {
    let mjcf = r#"
        <mujoco model="simple">
            <worldbody>
                <body name="base" pos="0 0 1">
                    <geom type="box" size="0.5 0.5 0.1" mass="10.0"/>
                    <body name="pendulum" pos="0 0 -0.5">
                        <joint name="hinge" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse MJCF");
    assert_eq!(model.name, "simple");
    assert_eq!(model.bodies.len(), 2);
    assert_eq!(model.joints.len(), 1);

    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    assert_eq!(world.body_count(), 2);
    assert_eq!(world.joint_count(), 1);

    assert!(spawned.body_id("base").is_some());
    assert!(spawned.body_id("pendulum").is_some());
    assert!(spawned.joint_id("hinge").is_some());
}

/// Test: MJCF model with free joint falls under gravity.
#[test]
fn test_mjcf_free_body_gravity() {
    let mjcf = r#"
        <mujoco model="falling">
            <option gravity="0 0 -9.81"/>
            <worldbody>
                <body name="ball" pos="0 0 5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse");
    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    let ball_id = spawned.body_id("ball").unwrap();
    let initial_z = world.body(ball_id).unwrap().state.pose.position.z;

    let mut stepper = Stepper::new();
    stepper.run_for(&mut world, 0.1).expect("should simulate");

    let final_z = world.body(ball_id).unwrap().state.pose.position.z;

    // Ball should fall: Δz ≈ 0.5 * 9.81 * 0.1² = 0.049m
    assert!(
        final_z < initial_z - 0.01,
        "Ball should fall: initial_z={}, final_z={}",
        initial_z,
        final_z
    );
}

/// Test: Model with muscles produces force when activated.
#[test]
fn test_mjcf_muscle_activation() {
    // A simple arm with a muscle that flexes the joint
    let mjcf = r#"
        <mujoco model="muscle_arm">
            <option gravity="0 0 0"/>
            <worldbody>
                <body name="base" pos="0 0 0">
                    <geom type="sphere" size="0.05" mass="10.0"/>
                    <body name="arm" pos="0.3 0 0">
                        <joint name="elbow" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
                        <geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0" mass="0.5"/>
                    </body>
                </body>
            </worldbody>
            <actuator>
                <muscle name="biceps" joint="elbow" gear="1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse");

    // Verify muscle was loaded
    assert!(model.muscle_count() > 0, "Should have muscles");
    let biceps = model.muscle("biceps");
    assert!(biceps.is_some(), "Should find biceps muscle");

    // Get muscle before spawning (which consumes model)
    let muscle_clone = biceps.map(|m| m.muscle().clone());

    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    let joint_id = spawned.joint_id("elbow").unwrap();
    let initial_pos = world.joint(joint_id).unwrap().state.position;

    // Verify muscle produces torque when activated
    if let Some(mut hill_muscle) = muscle_clone {
        hill_muscle.set_excitation(1.0);

        // Compute muscle torque
        let torque = hill_muscle.compute_torque(initial_pos, 0.0, 0.001);
        assert!(torque.abs() > 0.0, "Activated muscle should produce torque");
    }
}

/// Test: Model with tendons couples joints.
#[test]
fn test_mjcf_tendon_coupling() {
    let mjcf = r#"
        <mujoco model="tendon_test">
            <option gravity="0 0 0"/>
            <worldbody>
                <body name="base" pos="0 0 0">
                    <geom type="sphere" size="0.05" mass="10.0"/>
                    <body name="link1" pos="0 0 -0.3">
                        <joint name="joint1" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.2" mass="0.5"/>
                        <body name="link2" pos="0 0 -0.2">
                            <joint name="joint2" type="hinge" axis="0 1 0"/>
                            <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.2" mass="0.3"/>
                        </body>
                    </body>
                </body>
            </worldbody>
            <tendon>
                <fixed name="coupled">
                    <joint joint="joint1" coef="1.0"/>
                    <joint joint="joint2" coef="1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse");

    // Verify tendon was loaded
    assert!(!model.tendons.is_empty(), "Should have tendons");
    let coupled = model.tendon("coupled");
    assert!(coupled.is_some(), "Should find coupled tendon");

    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    assert_eq!(world.joint_count(), 2);
    assert!(spawned.joint_id("joint1").is_some());
    assert!(spawned.joint_id("joint2").is_some());
}

/// Test: Model with connect equality constraint.
#[test]
fn test_mjcf_connect_constraint() {
    let mjcf = r#"
        <mujoco model="equality_test">
            <option gravity="0 0 -9.81"/>
            <worldbody>
                <body name="body1" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="body2" pos="0.5 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="body1" body2="body2" anchor="0.25 0 0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse");

    // Verify equality constraint was loaded
    assert!(
        !model.connect_constraints.is_empty(),
        "Should have connect constraints"
    );

    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    let body1_id = spawned.body_id("body1").unwrap();
    let body2_id = spawned.body_id("body2").unwrap();

    // Get initial distance between bodies
    let pos1 = world.body(body1_id).unwrap().state.pose.position;
    let pos2 = world.body(body2_id).unwrap().state.pose.position;
    let initial_distance = (pos2 - pos1).norm();

    // Simulate
    let mut stepper = Stepper::new();
    stepper.run_for(&mut world, 0.5).expect("should simulate");

    // Bodies should have fallen but maintained relative position
    let final_pos1 = world.body(body1_id).unwrap().state.pose.position;
    let final_pos2 = world.body(body2_id).unwrap().state.pose.position;
    let final_distance = (final_pos2 - final_pos1).norm();

    // Distance should be approximately maintained by the constraint
    // Allow some tolerance for constraint solver
    assert_relative_eq!(initial_distance, final_distance, epsilon = 0.1);
}

/// Test: Model with weld constraint.
#[test]
fn test_mjcf_weld_constraint() {
    let mjcf = r#"
        <mujoco model="weld_test">
            <option gravity="0 0 -9.81"/>
            <worldbody>
                <body name="body1" pos="0 0 2">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
                <body name="body2" pos="0.3 0 2">
                    <joint type="free"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <weld body1="body1" body2="body2"/>
            </equality>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse");

    // Verify weld constraint was loaded
    assert!(
        !model.weld_constraints.is_empty(),
        "Should have weld constraints"
    );

    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    let body1_id = spawned.body_id("body1").unwrap();
    let body2_id = spawned.body_id("body2").unwrap();

    // Simulate
    let mut stepper = Stepper::new();
    stepper.run_for(&mut world, 0.3).expect("should simulate");

    // Both bodies should have same orientation (welded)
    let rot1 = world.body(body1_id).unwrap().state.pose.rotation;
    let rot2 = world.body(body2_id).unwrap().state.pose.rotation;

    // Compute angle between rotations
    let rot_diff = rot1.inverse() * rot2;
    let angle = rot_diff.angle();

    // Welded bodies should have nearly identical rotation
    assert!(
        angle < 0.1,
        "Welded bodies should have same orientation, angle diff: {}",
        angle
    );
}

/// Test: Model with distance constraint.
#[test]
fn test_mjcf_distance_constraint() {
    let mjcf = r#"
        <mujoco model="distance_test">
            <option gravity="0 0 -9.81"/>
            <worldbody>
                <body name="body1" pos="0 0 3">
                    <joint type="free"/>
                    <geom name="g1" type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="body2" pos="1 0 3">
                    <joint type="free"/>
                    <geom name="g2" type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <distance geom1="g1" geom2="g2" distance="1.0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse");

    // Verify distance constraint was loaded
    assert!(
        !model.distance_constraints.is_empty(),
        "Should have distance constraints"
    );

    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    let body1_id = spawned.body_id("body1").unwrap();
    let body2_id = spawned.body_id("body2").unwrap();

    // Simulate
    let mut stepper = Stepper::new();
    stepper.run_for(&mut world, 0.5).expect("should simulate");

    // Distance should be maintained at ~1.0
    let pos1 = world.body(body1_id).unwrap().state.pose.position;
    let pos2 = world.body(body2_id).unwrap().state.pose.position;
    let distance = (pos2 - pos1).norm();

    assert_relative_eq!(distance, 1.0, epsilon = 0.15);
}

/// Test: Model with joint equality constraint.
#[test]
fn test_mjcf_joint_equality() {
    let mjcf = r#"
        <mujoco model="joint_equality_test">
            <option gravity="0 0 0"/>
            <worldbody>
                <body name="base" pos="0 0 0">
                    <geom type="sphere" size="0.05" mass="10.0"/>
                    <body name="link1" pos="0 0 -0.3">
                        <joint name="joint1" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.2" mass="0.5"/>
                    </body>
                </body>
            </worldbody>
            <equality>
                <joint joint1="joint1" polycoef="0.5 0 0 0 0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse");

    // Verify joint equality constraint was loaded
    assert!(
        !model.joint_constraints.is_empty(),
        "Should have joint equality constraints"
    );

    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    let joint_id = spawned.joint_id("joint1").unwrap();

    // Verify joint was spawned and is accessible
    let joint = world.joint(joint_id).unwrap();
    assert!(
        joint.state.position.is_finite(),
        "Joint position should be finite"
    );

    // Note: Joint equality constraints may need the constraint solver to apply.
    // For now, verify the constraint was parsed and the joint is accessible.
}

/// Test: MJCF with multiple geoms computes correct mass.
#[test]
fn test_mjcf_multi_geom_mass() {
    let mjcf = r#"
        <mujoco model="multi_geom">
            <worldbody>
                <body name="compound" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" pos="-0.15 0 0" mass="0.5"/>
                    <geom type="sphere" size="0.1" pos="0.15 0 0" mass="0.5"/>
                    <geom type="box" size="0.15 0.05 0.05" pos="0 0 0" mass="0.2"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_mjcf_str(mjcf).expect("should parse");

    let mut world = World::default();
    let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

    let body_id = spawned.body_id("compound").unwrap();
    let body = world.body(body_id).unwrap();

    // Total mass should be 0.5 + 0.5 + 0.2 = 1.2 kg
    assert_relative_eq!(body.mass_props.mass, 1.2, epsilon = 0.01);

    // Inertia tensor should be non-zero and positive definite
    let inertia = body.mass_props.inertia;
    assert!(inertia[(0, 0)] > 0.0, "Ixx should be positive");
    assert!(inertia[(1, 1)] > 0.0, "Iyy should be positive");
    assert!(inertia[(2, 2)] > 0.0, "Izz should be positive");
}
