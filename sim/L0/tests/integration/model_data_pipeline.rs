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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
    data.forward(&model).expect("forward failed");

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
    data.forward(&model).expect("forward failed");

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
            <option gravity="0 0 -9.81" timestep="0.0001">
                <flag energy="enable"/>
            </option>
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
    data.forward(&model).expect("forward failed");

    let initial_energy = data.total_energy();

    // Verify we have meaningful initial energy
    assert!(
        initial_energy.abs() > 0.1,
        "Initial energy should be non-zero for valid test: {}",
        initial_energy
    );

    // Simulate for 1 second (10000 steps at 0.0001s)
    for _ in 0..10000 {
        data.step(&model).expect("step failed");
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
            <compiler angle="radian"/>
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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

// ============================================================================
// energy_initial: the drift baseline, captured once per reset
// ============================================================================

/// A one-link pendulum under gravity with energy computed.
fn energy_pendulum() -> sim_core::Model {
    let mut model = sim_core::Model::n_link_pendulum(1, 1.0, 0.1);
    model.enableflags |= sim_core::ENABLE_ENERGY;
    model
}

/// `forward` and `forward_skip` both record the first energy, and stepping
/// does not move it.
#[test]
fn forward_captures_energy_initial() {
    let model = energy_pendulum();
    for skip in [false, true] {
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        if skip {
            data.forward_skip(&model, sim_core::MjStage::None, false)
                .unwrap();
        } else {
            data.forward(&model).unwrap();
        }
        let first = data.total_energy();
        assert_ne!(first, 0.0);
        assert_eq!(data.energy_initial, first, "forward_skip: {skip}");
        for _ in 0..100 {
            data.step(&model).unwrap();
        }
        assert_eq!(
            data.energy_initial, first,
            "after 100 steps, forward_skip: {skip}"
        );
    }
}

/// A first energy of exactly 0 is a baseline like any other.
#[test]
fn energy_initial_is_captured_once_even_when_zero() {
    let mut model = energy_pendulum();
    model.gravity = nalgebra::Vector3::zeros();
    let mut data = model.make_data();
    data.forward_skip(&model, sim_core::MjStage::None, false)
        .unwrap(); // at rest: E = 0
    assert_eq!(data.total_energy(), 0.0);
    data.qvel[0] = 1.0;
    data.forward_skip(&model, sim_core::MjStage::None, false)
        .unwrap();
    assert_ne!(data.total_energy(), 0.0);
    assert_eq!(
        data.energy_initial, 0.0,
        "baseline moved to {}",
        data.energy_initial
    );
}

/// `reset` clears the baseline; the next forward pass records it again.
#[test]
fn reset_clears_the_energy_baseline() {
    let model = energy_pendulum();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();
    data.reset(&model);
    data.qpos[0] = 0.6;
    data.forward(&model).unwrap();
    assert_eq!(data.energy_initial, data.total_energy());
}

/// An auto-reset is a reset: the baseline is the energy at the reset state.
#[test]
fn an_auto_reset_recaptures_energy_initial() {
    let model = energy_pendulum();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();
    data.qpos[0] = f64::NAN;
    data.step(&model).unwrap();
    assert!(data.divergence_detected());
    let mut at_reset = model.make_data();
    at_reset.forward(&model).unwrap();
    assert_ne!(at_reset.total_energy(), 0.0);
    assert_eq!(data.energy_initial, at_reset.total_energy());
}

/// Building a model derives values from a `Data` (`acc0`, a muscle's
/// `lengthrange`, `invweight0`, `stat_meaninertia`, a spatial tendon's
/// `length0`) without `try_make_data`'s checks, so a model they refuse still
/// loads, and is refused when its `Data` is made.
#[test]
fn derivation_loads_a_model_try_make_data_refuses() {
    let model = load_model(
        r#"<mujoco model="derivation">
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="locked" type="hinge" axis="0 1 0" limited="true" range="0 0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1"/>
      <site name="s0"/>
      <body name="b" pos="0 0 -0.5">
        <joint name="swing" type="hinge" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1"/>
        <site name="s1" pos="0.1 0 -0.4"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t">
      <site site="s0"/>
      <site site="s1"/>
    </spatial>
  </tendon>
  <actuator>
    <muscle name="m" tendon="t"/>
  </actuator>
</mujoco>"#,
    )
    .expect("building the model runs no try_make_data check");
    assert!(model.tendon_length0[0] > 0.0);
    assert!(model.actuator_acc0[0] > 0.0);
    let refused = model.try_make_data().err();
    assert!(
        matches!(
            &refused,
            Some(sim_core::MakeDataError::Range(e)) if e.field == "jnt_range" && e.index == 0
        ),
        "{refused:?}"
    );
}

/// A clone keeps the baseline it was cloned with.
#[test]
fn a_clone_keeps_the_energy_baseline() {
    let model = energy_pendulum();
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).unwrap();
    let first = data.total_energy();
    let mut clone = data.clone();
    clone.qpos[0] = 0.6;
    clone.forward(&model).unwrap();
    assert_ne!(clone.total_energy(), first);
    assert_eq!(clone.energy_initial, first);
}

// ============================================================================
// Data::reset leaves what make_data leaves
// ============================================================================

/// Write values `make_data` never leaves into a `Data`'s state, derived
/// arrays, flags and statistics.
pub(super) fn scramble(data: &mut sim_core::Data) {
    data.time = 7.0;
    data.qpos.fill(0.7);
    data.qvel.fill(7.0);
    data.qacc_warmstart.fill(7.0);
    data.xpos.fill(nalgebra::Vector3::repeat(7.0));
    data.xipos.fill(nalgebra::Vector3::repeat(7.0));
    data.subtree_com.fill(nalgebra::Vector3::repeat(7.0));
    data.cinert.fill(nalgebra::Matrix6::repeat(7.0));
    data.qM.fill(7.0);
    data.qLD_valid = true;
    data.qfrc_bias.fill(7.0);
    data.qfrc_applied.fill(7.0);
    data.stat_meaninertia = 7.0;
    data.solver_niter = 7;
    data.energy_potential = 7.0;
    data.sensordata.fill(7.0);
}

/// After any history, `reset` leaves the `Data` a fresh `make_data` leaves:
/// MuJoCo's `_resetData` zeroes the whole `mjData` before it sets the
/// defaults, and `mj_makeData` ends in it.
#[test]
fn reset_leaves_the_data_make_data_leaves() {
    let model = energy_pendulum();
    let want = format!("{:#?}", model.make_data());
    let mut data = model.make_data();
    data.qpos[0] = f64::NAN;
    for _ in 0..6 {
        data.step(&model).unwrap(); // the first auto-resets, with a bad-qpos warning
    }
    scramble(&mut data);
    data.reset(&model);
    let got = format!("{data:#?}");
    assert!(
        got == want,
        "reset differs from make_data in: {:?}",
        super::keyframes::differing_fields(&got, &want)
    );
}
