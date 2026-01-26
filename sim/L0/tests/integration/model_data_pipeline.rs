//! Model/Data pipeline integration tests.
//!
//! Tests the new MuJoCo-aligned Model/Data architecture:
//! - MJCF → Model conversion
//! - Model::make_data() for creating simulation state
//! - Data::step() for time integration
//! - Forward kinematics correctness
//!
//! This validates the Phase 4+ consolidation work.

use approx::assert_relative_eq;
use sim_mjcf::load_model;

/// Test: Simple pendulum loads and simulates correctly.
#[test]
fn test_simple_pendulum_model_data() {
    // The pendulum body has a geom positioned below the joint pivot
    // so gravity can produce torque around the hinge axis.
    let mjcf = r#"
        <mujoco model="pendulum">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" pos="0 0 -0.5" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load MJCF into Model");

    // Verify model dimensions
    assert_eq!(model.nbody, 2, "should have world + 1 body");
    assert_eq!(model.njnt, 1, "should have 1 joint");
    assert_eq!(model.nq, 1, "hinge joint has nq=1");
    assert_eq!(model.nv, 1, "hinge joint has nv=1");

    // Create simulation state
    let mut data = model.make_data();
    assert_eq!(data.qpos.len(), 1);
    assert_eq!(data.qvel.len(), 1);

    // Start from horizontal position (PI/2 from vertical)
    data.qpos[0] = std::f64::consts::FRAC_PI_2;

    // Step the simulation
    let initial_qpos = data.qpos[0];
    for _ in 0..100 {
        data.step(&model);
    }

    // Pendulum should have swung down (qpos decreased towards 0)
    assert!(
        data.qpos[0] < initial_qpos,
        "pendulum should swing down: {} < {}",
        data.qpos[0],
        initial_qpos
    );
}

/// Test: Double pendulum Model/Data pipeline.
#[test]
fn test_double_pendulum_model_data() {
    let mjcf = r#"
        <mujoco model="double_pendulum">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="link1" pos="0 0 0">
                    <joint name="joint1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.02 0.25" mass="0.5"/>
                    <body name="link2" pos="0 0 -0.5">
                        <joint name="joint2" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.02 0.25" mass="0.5"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    assert_eq!(model.nbody, 3, "world + 2 links");
    assert_eq!(model.njnt, 2, "2 hinge joints");
    assert_eq!(model.nq, 2, "2 scalar joint positions");
    assert_eq!(model.nv, 2, "2 DOFs");

    let mut data = model.make_data();

    // Start both links horizontal
    data.qpos[0] = std::f64::consts::FRAC_PI_2;
    data.qpos[1] = 0.0;

    // Simulate
    for _ in 0..500 {
        data.step(&model);
    }

    // Both joints should have moved
    assert!(
        data.qvel[0].abs() > 0.01 || data.qvel[1].abs() > 0.01,
        "double pendulum should be moving"
    );
}

/// Test: Free joint body falls under gravity.
#[test]
fn test_free_joint_gravity() {
    let mjcf = r#"
        <mujoco model="falling_ball">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    assert_eq!(model.nbody, 2);
    assert_eq!(model.njnt, 1);
    assert_eq!(model.nq, 7, "free joint: 3 pos + 4 quat");
    assert_eq!(model.nv, 6, "free joint: 3 linear + 3 angular DOF");

    let mut data = model.make_data();

    // Check initial position (should be at z=5)
    let initial_z = data.qpos[2];
    assert_relative_eq!(initial_z, 5.0, epsilon = 1e-10);

    // Simulate for 0.1 seconds
    for _ in 0..100 {
        data.step(&model);
    }

    // Ball should have fallen: Δz ≈ 0.5 * 9.81 * 0.1² = 0.049m
    let final_z = data.qpos[2];
    assert!(
        final_z < initial_z - 0.04,
        "ball should fall: initial={}, final={}",
        initial_z,
        final_z
    );
}

/// Test: Ball joint (spherical) model.
#[test]
fn test_ball_joint_model_data() {
    let mjcf = r#"
        <mujoco model="spherical_pendulum">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball_link" pos="0 0 0">
                    <joint type="ball"/>
                    <geom type="sphere" size="0.1" pos="0 0 -0.5" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    assert_eq!(model.nq, 4, "ball joint: 4 quaternion components");
    assert_eq!(model.nv, 3, "ball joint: 3 angular DOF");

    let mut data = model.make_data();

    // Apply small angular velocity
    data.qvel[0] = 0.1;

    // Simulate
    for _ in 0..100 {
        data.step(&model);
    }

    // Quaternion should still be normalized
    let q_norm =
        (data.qpos[0].powi(2) + data.qpos[1].powi(2) + data.qpos[2].powi(2) + data.qpos[3].powi(2))
            .sqrt();
    assert_relative_eq!(q_norm, 1.0, epsilon = 1e-6);
}

/// Test: Forward kinematics produces correct body poses.
#[test]
fn test_forward_kinematics_model_data() {
    let mjcf = r#"
        <mujoco model="fk_test">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="link1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="box" size="0.1 0.1 0.5" mass="1.0"/>
                    <body name="link2" pos="0 0 1">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="box" size="0.1 0.1 0.5" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Zero configuration
    data.qpos[0] = 0.0;
    data.qpos[1] = 0.0;
    data.forward(&model);

    // Link1 should be at (0, 0, 1)
    assert_relative_eq!(data.xpos[1].x, 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.xpos[1].y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.xpos[1].z, 1.0, epsilon = 1e-6);

    // Link2 should be at (0, 0, 2)
    assert_relative_eq!(data.xpos[2].x, 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.xpos[2].y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.xpos[2].z, 2.0, epsilon = 1e-6);

    // Rotate first joint 90 degrees around Y axis
    data.qpos[0] = std::f64::consts::FRAC_PI_2;
    data.forward(&model);

    // Link1 body position stays at (0, 0, 1) - the joint rotates the body orientation
    // but the body frame origin (where joint anchor is) doesn't move
    assert_relative_eq!(data.xpos[1].x, 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.xpos[1].y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.xpos[1].z, 1.0, epsilon = 1e-6);

    // Link2 is at pos="0 0 1" relative to Link1's rotated frame
    // After 90 degree rotation around Y, the local Z axis points toward +X
    // So Link2's position in world: (0,0,1) + rot_Y(PI/2) * (0,0,1) = (0,0,1) + (1,0,0) = (1,0,2)
    assert_relative_eq!(data.xpos[2].x, 1.0, epsilon = 1e-6);
    assert_relative_eq!(data.xpos[2].y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.xpos[2].z, 1.0, epsilon = 1e-6);
}

/// Test: Energy conservation in frictionless system.
#[test]
fn test_energy_conservation_model_data() {
    // Pendulum at 45 degrees starts with non-zero potential AND kinetic energy
    // This ensures we're testing actual energy conservation, not zero=zero
    let mjcf = r#"
        <mujoco model="energy_test">
            <option gravity="0 0 -9.81" timestep="0.0001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" pos="0 0 -1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start from 45 degrees with initial velocity
    // At 45 degrees, the COM is at (-0.707, 0, -0.707), giving non-zero potential energy
    // Initial velocity gives non-zero kinetic energy
    data.qpos[0] = std::f64::consts::FRAC_PI_4; // 45 degrees
    data.qvel[0] = 1.0; // Initial angular velocity
    data.forward(&model);

    let initial_energy = data.total_energy();

    // Verify we have meaningful initial energy
    assert!(
        initial_energy.abs() > 0.1,
        "Initial energy should be non-zero for valid test: {}",
        initial_energy
    );

    // Simulate for 1 second (10000 steps at 0.0001s)
    for _ in 0..10000 {
        data.step(&model);
    }

    let final_energy = data.total_energy();

    // Energy should be conserved within 1% for semi-implicit Euler with small timestep
    let energy_drift = (final_energy - initial_energy).abs() / initial_energy.abs();
    assert!(
        energy_drift < 0.01,
        "Energy drift too large: {:.2}% (initial={}, final={})",
        energy_drift * 100.0,
        initial_energy,
        final_energy
    );
}

/// Test: Joint limits are enforced.
#[test]
fn test_joint_limits_model_data() {
    let mjcf = r#"
        <mujoco model="limited_joint">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint name="limited" type="hinge" axis="0 1 0" limited="true" range="-0.5 0.5" damping="10.0"/>
                    <geom type="capsule" size="0.05 0.5" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Verify limit is stored
    assert!(model.jnt_limited[0]);
    assert_relative_eq!(model.jnt_range[0].0, -0.5, epsilon = 1e-6);
    assert_relative_eq!(model.jnt_range[0].1, 0.5, epsilon = 1e-6);

    let mut data = model.make_data();

    // Start at the limit with velocity into limit
    data.qpos[0] = 0.5;
    data.qvel[0] = 5.0; // Moving toward +0.6, past the +0.5 limit

    // Simulate - limits should prevent penetration past 0.5
    for _ in 0..500 {
        data.step(&model);
    }

    // Joint should be within or near limits (with small overshoot allowed)
    // The penalty method allows some penetration but should keep it bounded
    let limit_violation = (data.qpos[0] - 0.5).max(0.0) + (-0.5 - data.qpos[0]).max(0.0);
    assert!(
        limit_violation < 0.2,
        "joint limit violated too much: qpos={}, limits=[-0.5, 0.5], violation={}",
        data.qpos[0],
        limit_violation
    );
}

/// Test: Joint damping dissipates energy.
#[test]
fn test_joint_damping_model_data() {
    let mjcf = r#"
        <mujoco model="damped">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="spinning" pos="0 0 0">
                    <joint type="hinge" axis="0 0 1" damping="1.0"/>
                    <geom type="box" size="0.5 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start with angular velocity
    data.qvel[0] = 10.0;

    let initial_velocity = data.qvel[0].abs();

    // Simulate
    for _ in 0..1000 {
        data.step(&model);
    }

    // Velocity should decrease due to damping
    let final_velocity = data.qvel[0].abs();
    assert!(
        final_velocity < initial_velocity * 0.5,
        "damping should reduce velocity: initial={}, final={}",
        initial_velocity,
        final_velocity
    );
}

/// Test: Joint spring provides restoring force.
#[test]
fn test_joint_spring_model_data() {
    let mjcf = r#"
        <mujoco model="spring">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="sprung" pos="0 0 0">
                    <joint type="hinge" axis="0 1 0" stiffness="10.0" damping="0.5"/>
                    <geom type="box" size="0.5 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Displace from equilibrium
    data.qpos[0] = 1.0;

    // Simulate
    for _ in 0..5000 {
        data.step(&model);
    }

    // Spring + damping should bring it back near equilibrium
    assert!(
        data.qpos[0].abs() < 0.5,
        "spring should restore position: qpos={}",
        data.qpos[0]
    );
}

/// Test: Actuator control produces torque.
#[test]
fn test_actuator_model_data() {
    let mjcf = r#"
        <mujoco model="actuated">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="motor" pos="0 0 0">
                    <joint name="motor_joint" type="hinge" axis="0 0 1"/>
                    <geom type="box" size="0.5 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="motor1" joint="motor_joint" gear="10"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nu, 1, "should have 1 actuator");

    let mut data = model.make_data();

    // Apply control input
    data.ctrl[0] = 1.0;

    // Simulate
    for _ in 0..100 {
        data.step(&model);
    }

    // Joint should have accelerated
    assert!(
        data.qvel[0].abs() > 0.1,
        "actuator should produce motion: qvel={}",
        data.qvel[0]
    );
}

/// Test: Data reset restores initial state.
#[test]
fn test_data_reset() {
    let mjcf = r#"
        <mujoco model="reset_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 5">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    let initial_qpos = data.qpos.clone();

    // Simulate
    for _ in 0..1000 {
        data.step(&model);
    }

    // State should have changed
    assert!((data.qpos[2] - initial_qpos[2]).abs() > 0.1);

    // Reset
    data.reset(&model);

    // State should be restored
    assert_relative_eq!(data.qpos[2], initial_qpos[2], epsilon = 1e-10);
    assert_relative_eq!(data.qvel[0], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.time, 0.0, epsilon = 1e-10);
}

/// Test: Sensor data array is properly allocated.
#[test]
fn test_sensor_data_allocation() {
    let mjcf = r#"
        <mujoco model="sensor_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" pos="0 0 -0.5" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let data = model.make_data();

    // Sensor data should be allocated (even if empty when no sensors defined)
    assert_eq!(data.sensordata.len(), model.nsensordata);
    assert_eq!(model.nsensor, 0, "no sensors defined in MJCF");
}

/// Test: Model sensor fields are initialized correctly.
#[test]
fn test_model_sensor_fields() {
    let mjcf = r#"
        <mujoco model="no_sensor">
            <option gravity="0 0 -9.81"/>
            <worldbody>
                <body name="test">
                    <joint type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // All sensor arrays should be empty
    assert_eq!(model.nsensor, 0);
    assert_eq!(model.nsensordata, 0);
    assert!(model.sensor_type.is_empty());
    assert!(model.sensor_adr.is_empty());
    assert!(model.sensor_dim.is_empty());
}
