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

/// A model with a range `try_make_data` refuses still loads, with a muscle and
/// a spatial tendon whose derivations would build a `Data`: building skips
/// those derivations, and making its `Data` refuses it.
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

// ============================================================================
// Model::recompute_derived and the derived fields
// ============================================================================

/// `compute_dof_lengths` sizes `dof_length` itself.
#[test]
fn compute_dof_lengths_sizes_dof_length() {
    let mut model = sim_core::Model::n_link_pendulum(2, 1.0, 0.1);
    model.dof_length.clear();
    sim_core::compute_dof_lengths(&mut model);
    assert_eq!(model.dof_length.len(), 2);
}

/// Two hinged links with explicit inertials, a fixed tendon, a spatial tendon,
/// a muscle and a position actuator with a damping ratio: every derivation has
/// work to do, and each edit below is one `Model` field.
const DERIVED: &str = r#"<mujoco model="derived">
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="j0" type="hinge" axis="0 1 0" damping="0.5" ref="0.3"/>
      <inertial pos="0 0 -0.25" mass="1" diaginertia="0.02 0.02 0.001"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02"/>
      <site name="s0" pos="0.05 0 -0.1"/>
      <body name="b" pos="0 0 -0.5">
        <joint name="j1" type="hinge" axis="0 1 0" stiffness="2"/>
        <inertial pos="0 0 -0.1" mass="0.5" diaginertia="0.005 0.005 0.001"/>
        <geom type="sphere" size="0.05"/>
        <site name="s1" pos="0.05 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="sum">
      <joint joint="j0" coef="1"/>
      <joint joint="j1" coef="0.5"/>
    </fixed>
    <spatial name="cable">
      <site site="s0"/>
      <site site="s1"/>
    </spatial>
  </tendon>
  <actuator>
    <muscle name="m" tendon="cable"/>
    <position name="p" joint="j0" kp="10" dampratio="1"/>
  </actuator>
</mujoco>"#;

/// On a model the builder made, `recompute_derived` changes nothing.
#[test]
fn recompute_derived_changes_nothing_on_a_built_model() {
    let mut model = load_model(DERIVED).unwrap();
    let before = format!("{model:#?}");
    model.recompute_derived().unwrap();
    let after = format!("{model:#?}");
    assert!(after == before, "recompute_derived changed a built model");
}

/// The derived fields an edit reaches, printed for comparison.
fn derived_fields(m: &sim_core::Model) -> Vec<(&'static str, String)> {
    vec![
        ("body_subtreemass", format!("{:?}", m.body_subtreemass)),
        ("body_invweight0", format!("{:?}", m.body_invweight0)),
        ("dof_invweight0", format!("{:?}", m.dof_invweight0)),
        ("tendon_invweight0", format!("{:?}", m.tendon_invweight0)),
        ("stat_meaninertia", format!("{:?}", m.stat_meaninertia)),
        ("actuator_acc0", format!("{:?}", m.actuator_acc0)),
        (
            "actuator_lengthrange",
            format!("{:?}", m.actuator_lengthrange),
        ),
        ("tendon_length0", format!("{:?}", m.tendon_length0)),
        ("geom_rbound", format!("{:?}", m.geom_rbound)),
        ("geom_aabb", format!("{:?}", m.geom_aabb)),
        ("implicit_damping", format!("{:?}", m.implicit_damping)),
    ]
}

/// An edit of one primary field, then `recompute_derived`, gives the derived
/// fields that building the edited MJCF gives.
#[test]
fn recompute_derived_after_an_edit_matches_building_the_edited_model() {
    type Edit = fn(&mut sim_core::Model);
    let edits: [(&str, &str, &str, Edit); 5] = [
        ("body b's mass", r#"mass="0.5""#, r#"mass="1.5""#, |m| {
            let b = m.body_id("b").unwrap();
            m.body_mass[b] = 1.5;
        }),
        (
            "site s1's position",
            r#"pos="0.05 0 -0.2""#,
            r#"pos="0.08 0 -0.2""#,
            |m| {
                let s1 = m.site_id("s1").unwrap();
                m.site_pos[s1] = nalgebra::Vector3::new(0.08, 0.0, -0.2);
            },
        ),
        (
            "the fixed tendon's coefficient on j0",
            r#"coef="1""#,
            r#"coef="1.25""#,
            |m| {
                let sum = m.tendon_id("sum").unwrap();
                m.wrap_prm[m.tendon_adr[sum]] = 1.25;
            },
        ),
        (
            "the rod's radius",
            r#"size="0.02""#,
            r#"size="0.04""#,
            |m| {
                let rod = m.geom_id("rod").unwrap();
                m.geom_size[rod].x = 0.04;
            },
        ),
        ("j0's damping", r#"damping="0.5""#, r#"damping="2""#, |m| {
            let j0 = m.joint_id("j0").unwrap();
            m.jnt_damping[j0] = 2.0;
            m.dof_damping[m.jnt_dof_adr[j0]] = 2.0;
        }),
    ];
    for (what, from, to, edit) in edits {
        assert_eq!(DERIVED.matches(from).count(), 1, "{what}");
        let built = load_model(&DERIVED.replace(from, to)).unwrap();
        let mut edited = load_model(DERIVED).unwrap();
        edit(&mut edited);
        edited.recompute_derived().unwrap();
        let unedited = derived_fields(&load_model(DERIVED).unwrap());
        let (want, got) = (derived_fields(&built), derived_fields(&edited));
        assert!(
            want != unedited,
            "{what}: the edit reaches no derived field"
        );
        for ((field, w), (_, g)) in want.iter().zip(&got) {
            assert_eq!(g, w, "{what}: {field}");
        }
    }
}

/// `recompute_derived` follows a damping edit into the implicit parameters.
#[test]
fn recompute_derived_follows_a_damping_edit() {
    let mut model = sim_core::Model::n_link_pendulum(1, 1.0, 0.1);
    model.jnt_damping[0] = 5.0;
    model.dof_damping[0] = 5.0;
    model.recompute_derived().unwrap();
    assert_eq!(model.implicit_damping[0], 5.0);
}

/// The derived fields no one-field edit above reaches: cleared, then
/// recomputed, they are the built model's again.
#[test]
fn recompute_derived_restores_the_structural_fields() {
    let structural = |m: &sim_core::Model| {
        format!(
            "{:?} {:?} {:?} {:?} {:?} {:?} {} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
            m.body_ancestor_joints,
            m.body_ancestor_mask,
            m.body_weldid,
            m.qLD_rowadr,
            m.qLD_rownnz,
            m.qLD_colind,
            m.ntree,
            m.tree_body_adr,
            m.tree_dof_num,
            m.body_treeid,
            m.dof_treeid,
            m.tendon_tree,
            m.tree_sleep_policy,
            m.dof_length
        )
    };
    let built = load_model(DERIVED).unwrap();
    let mut cleared = load_model(DERIVED).unwrap();
    cleared.body_ancestor_joints.clear();
    cleared.body_ancestor_mask.clear();
    cleared.body_weldid.fill(0);
    cleared.qLD_rowadr.clear();
    cleared.qLD_rownnz.clear();
    cleared.qLD_colind.clear();
    cleared.ntree = 0;
    cleared.tree_body_adr.clear();
    cleared.tree_dof_num.clear();
    cleared.body_treeid.clear();
    cleared.dof_treeid.clear();
    cleared.tendon_tree.clear();
    cleared.tree_sleep_policy.clear();
    cleared.dof_length.clear();
    assert_ne!(structural(&cleared), structural(&built));
    cleared.recompute_derived().unwrap();
    assert_eq!(structural(&cleared), structural(&built));
}

/// A range `check_ranges` refuses leaves the model as it was.
#[test]
fn recompute_derived_refuses_a_backwards_limited_range() {
    let mut model = sim_core::Model::n_link_pendulum(1, 1.0, 0.1);
    model.jnt_limited[0] = true;
    model.jnt_range[0] = (1.0, -1.0);
    model.jnt_damping[0] = 5.0;
    let before = format!("{model:#?}");
    let refused = model.recompute_derived();
    assert!(
        matches!(
            &refused,
            Err(sim_core::ModelError::Range(e)) if e.field == "jnt_range" && e.index == 0
        ),
        "{refused:?}"
    );
    assert!(
        format!("{model:#?}") == before,
        "a refused recompute changed the model"
    );
}
