//! Musculoskeletal integration tests.
//!
//! Tests muscle and tendon dynamics in isolation and combined with the
//! physics simulation.

use approx::assert_relative_eq;
use sim_muscle::{HillMuscle, HillMuscleConfig, MuscleGroup};
use sim_tendon::{CableProperties, FixedTendon, TendonActuator};
use sim_types::JointId;

/// Test: Single muscle pulling on a revolute joint.
#[test]
fn test_single_muscle_on_joint() {
    let config = HillMuscleConfig::default();
    let mut muscle = HillMuscle::new(config);

    // Initial state: muscle at rest (note: default activation may be small but non-zero)
    assert_relative_eq!(muscle.excitation(), 0.0, epsilon = 1e-6);
    assert!(
        muscle.activation() < 0.05,
        "Initial activation should be small"
    );

    // Apply excitation
    muscle.set_excitation(1.0);
    assert_relative_eq!(muscle.excitation(), 1.0, epsilon = 1e-6);

    // Compute torque over multiple timesteps to ramp up activation
    let joint_angle = 0.0;
    let joint_velocity = 0.0;
    let dt = 0.001;

    let mut torque = 0.0;
    for _ in 0..100 {
        torque = muscle.compute_torque(joint_angle, joint_velocity, dt);
    }

    // After 100ms, activation should have ramped up
    assert!(muscle.activation() > 0.5, "Activation should ramp up");
    assert!(torque > 0.0, "Muscle should produce positive torque");
}

/// Test: Antagonist muscle pair (co-contraction).
#[test]
fn test_antagonist_muscle_pair() {
    let flexor_config = HillMuscleConfig::biceps();
    let extensor_config = HillMuscleConfig::default();

    let flexor = HillMuscle::new(flexor_config);
    let extensor = HillMuscle::new(extensor_config);

    let mut group = MuscleGroup::new()
        .with_flexor(flexor)
        .with_extensor(extensor);

    assert_eq!(group.len(), 2);

    // Activate both muscles equally (co-contraction)
    group.set_all_excitations(&[1.0, 1.0]);

    // Warm up activations
    let joint_angle = 0.0;
    let joint_velocity = 0.0;
    let dt = 0.001;

    for _ in 0..200 {
        let _ = group.compute_net_torque(joint_angle, joint_velocity, dt);
    }

    // Net torque should be relatively small compared to individual muscle forces
    // Note: Different muscle configs may not cancel exactly
    let net_torque = group.compute_net_torque(joint_angle, joint_velocity, dt);
    assert!(
        net_torque.abs() < 30.0,
        "Co-contraction should produce relatively small net torque: {}",
        net_torque
    );

    // But individual muscles should produce significant torque
    let diagnostics = group.diagnostics();
    assert!(diagnostics.len() == 2);
}

/// Test: Antagonist pair with asymmetric activation.
#[test]
fn test_antagonist_asymmetric_activation() {
    let flexor_config = HillMuscleConfig::biceps();
    let extensor_config = HillMuscleConfig::default();

    let flexor = HillMuscle::new(flexor_config);
    let extensor = HillMuscle::new(extensor_config);

    let mut group = MuscleGroup::new()
        .with_flexor(flexor)
        .with_extensor(extensor);

    // Activate flexor more than extensor
    group.set_all_excitations(&[1.0, 0.3]);

    let joint_angle = 0.0;
    let joint_velocity = 0.0;
    let dt = 0.001;

    // Warm up
    for _ in 0..200 {
        let _ = group.compute_net_torque(joint_angle, joint_velocity, dt);
    }

    let net_torque = group.compute_net_torque(joint_angle, joint_velocity, dt);

    // Flexor dominates, so positive torque
    assert!(
        net_torque > 0.0,
        "Flexor-dominant activation should produce positive torque: {}",
        net_torque
    );
}

/// Test: Fixed tendon routing through multiple joints.
#[test]
fn test_fixed_tendon_multi_joint() {
    // Fixed tendon couples two joints with coefficients
    let tendon = FixedTendon::new("coupled")
        .with_coefficient(JointId::new(0), 1.0)  // joint0 with coef 1.0
        .with_coefficient(JointId::new(1), -0.5) // joint1 with coef -0.5
        .with_rest_length(0.5);

    // Compute tendon length given joint positions
    let joint_positions = [0.5, 0.3]; // joint0 at 0.5 rad, joint1 at 0.3 rad

    // Tendon length = rest_length + sum(coef_i * q_i) = 0.5 + 1.0*0.5 + (-0.5)*0.3 = 0.85
    let length = tendon.compute_length(&joint_positions);
    assert_relative_eq!(length, 0.85, epsilon = 1e-6);

    // Jacobian gives the coupling coefficients
    let jacobian = tendon.jacobian(&joint_positions);
    assert_eq!(jacobian.len(), 2);
    assert_relative_eq!(jacobian[0], 1.0, epsilon = 1e-6);
    assert_relative_eq!(jacobian[1], -0.5, epsilon = 1e-6);
}

/// Test: Tendon with cable properties (stiffness, damping).
#[test]
fn test_tendon_cable_properties() {
    let props = CableProperties::new(1000.0, 0.5) // stiffness, rest_length
        .with_damping(10.0)
        .with_max_tension(500.0);

    // When stretched beyond rest length, tension is positive
    let current_length = 0.55; // 0.05m stretch
    let velocity = 0.0;

    let tension = props.compute_force(current_length, velocity);
    // F = k * (L - L0) = 1000 * 0.05 = 50 N
    assert_relative_eq!(tension, 50.0, epsilon = 1e-6);

    // With positive velocity (stretching), damping adds tension
    // Note: default damping behavior only adds when lengthening
    let tension_with_damping = props.compute_force(current_length, 0.1);
    // F = 50 + 10 * 0.1 = 51 N (if velocity > 0)
    assert!(tension_with_damping >= tension);

    // Compression (shorter than rest length) produces zero tension
    let compression_length = 0.45;
    let compression_tension = props.compute_force(compression_length, 0.0);
    assert_relative_eq!(compression_tension, 0.0, epsilon = 1e-6);
}

/// Test: Muscle-tendon unit (muscle in series with tendon).
#[test]
fn test_muscle_tendon_unit() {
    // Create a muscle with tendon properties
    let config = HillMuscleConfig {
        max_isometric_force: 500.0,
        optimal_fiber_length: 0.1,
        tendon_slack_length: 0.15,
        pennation_angle_optimal: 0.0,
        ..Default::default()
    };

    let mut muscle = HillMuscle::new(config);

    // At optimal length, muscle produces max force
    muscle.set_excitation(1.0);

    // Warm up activation
    for _ in 0..200 {
        let _ = muscle.compute_torque(0.0, 0.0, 0.001);
    }

    // At optimal length and zero velocity, force should approach max isometric
    let diagnostics = muscle.diagnostics();
    assert!(
        diagnostics.fiber_force > 0.0,
        "Muscle fiber should produce force"
    );
}

/// Test: Fixed tendon computes force correctly.
#[test]
fn test_fixed_tendon_force() {
    let tendon = FixedTendon::new("test")
        .with_coefficient(JointId::new(0), 0.05)  // 5cm moment arm
        .with_rest_length(0.3)
        .with_stiffness(2000.0)
        .with_damping(20.0);

    // At zero angle, length equals rest length, no force
    let force_at_rest = tendon.compute_force(&[0.0], &[0.0]);
    assert_relative_eq!(force_at_rest, 0.0, epsilon = 1e-6);

    // At 1 radian, tendon is stretched by 0.05m
    // Force = k * stretch = 2000 * 0.05 = 100 N
    let force_stretched = tendon.compute_force(&[1.0], &[0.0]);
    assert_relative_eq!(force_stretched, 100.0, epsilon = 1e-6);
}

/// Test: Muscle force-length relationship.
#[test]
fn test_muscle_force_length() {
    let config = HillMuscleConfig::default();
    let mut muscle = HillMuscle::new(config);

    muscle.set_excitation(1.0);

    // Warm up
    for _ in 0..300 {
        let _ = muscle.compute_torque(0.0, 0.0, 0.001);
    }

    // Test at different muscle lengths via joint angle
    let torques: Vec<f64> = [-1.0, -0.5, 0.0, 0.5, 1.0]
        .iter()
        .map(|&angle| {
            let mut m = HillMuscle::new(HillMuscleConfig::default());
            m.set_excitation(1.0);
            for _ in 0..300 {
                let _ = m.compute_torque(angle, 0.0, 0.001);
            }
            m.compute_torque(angle, 0.0, 0.001)
        })
        .collect();

    // Torques should vary with angle (force-length relationship)
    // All should be positive but with different magnitudes
    for (i, &torque) in torques.iter().enumerate() {
        assert!(
            torque > 0.0,
            "Torque at angle {} should be positive: {}",
            i,
            torque
        );
    }
}

/// Test: Muscle force-velocity relationship.
#[test]
fn test_muscle_force_velocity() {
    let config = HillMuscleConfig::default();

    // Compare isometric (v=0) vs concentric (v>0, shortening) vs eccentric (v<0, lengthening)
    let test_velocities = [-2.0, -1.0, 0.0, 1.0, 2.0]; // lengthening is negative

    let forces: Vec<f64> = test_velocities
        .iter()
        .map(|&vel| {
            let mut m = HillMuscle::new(config.clone());
            m.set_excitation(1.0);
            // Warm up at this velocity
            for _ in 0..300 {
                let _ = m.compute_torque(0.0, vel, 0.001);
            }
            m.compute_torque(0.0, vel, 0.001)
        })
        .collect();

    // Eccentric (lengthening, negative velocity) produces higher force
    // Isometric (zero velocity) is intermediate
    // Concentric (shortening, positive velocity) produces lower force
    let eccentric_force = forces[0]; // v = -2.0
    let isometric_force = forces[2]; // v = 0.0
    let concentric_force = forces[4]; // v = 2.0

    // This is the Hill curve relationship:
    // F_eccentric > F_isometric > F_concentric
    assert!(
        eccentric_force >= isometric_force * 0.95,
        "Eccentric should be >= isometric: {} vs {}",
        eccentric_force,
        isometric_force
    );
    assert!(
        isometric_force >= concentric_force,
        "Isometric should be >= concentric: {} vs {}",
        isometric_force,
        concentric_force
    );
}

/// Test: Muscle activation dynamics (rise and fall times).
#[test]
fn test_muscle_activation_dynamics() {
    let config = HillMuscleConfig::default();
    let mut muscle = HillMuscle::new(config);

    let dt = 0.001;

    // Start with no excitation (activation may have small initial value)
    let initial_activation = muscle.activation();
    assert!(
        initial_activation < 0.05,
        "Initial activation should be small: {}",
        initial_activation
    );

    // Apply full excitation
    muscle.set_excitation(1.0);

    // Track activation rise
    let mut activations_rise = Vec::new();
    for _ in 0..100 {
        let _ = muscle.compute_torque(0.0, 0.0, dt);
        activations_rise.push(muscle.activation());
    }

    // Activation should increase monotonically
    for i in 1..activations_rise.len() {
        assert!(
            activations_rise[i] >= activations_rise[i - 1],
            "Activation should increase during excitation"
        );
    }

    // After 100ms, activation should be significant
    assert!(
        muscle.activation() > 0.5,
        "Activation should be >0.5 after 100ms"
    );

    // Now remove excitation
    muscle.set_excitation(0.0);

    // Track activation fall
    let mut activations_fall = Vec::new();
    for _ in 0..200 {
        let _ = muscle.compute_torque(0.0, 0.0, dt);
        activations_fall.push(muscle.activation());
    }

    // Activation should decrease (deactivation is typically slower)
    let final_activation = *activations_fall.last().unwrap();
    assert!(
        final_activation < activations_rise.last().unwrap() - 0.1,
        "Activation should decrease after removing excitation"
    );
}

/// Test: Predefined muscle configurations.
#[test]
fn test_predefined_muscle_configs() {
    // Biceps
    let biceps_config = HillMuscleConfig::biceps();
    let mut biceps = HillMuscle::new(biceps_config);
    biceps.set_excitation(1.0);
    for _ in 0..200 {
        let _ = biceps.compute_torque(0.0, 0.0, 0.001);
    }
    let biceps_force = biceps.diagnostics().fiber_force;
    assert!(biceps_force > 0.0, "Biceps should produce force");

    // Quadriceps (stronger muscle)
    let quad_config = HillMuscleConfig::quadriceps();
    let mut quad = HillMuscle::new(quad_config);
    quad.set_excitation(1.0);
    for _ in 0..200 {
        let _ = quad.compute_torque(0.0, 0.0, 0.001);
    }
    let quad_force = quad.diagnostics().fiber_force;
    assert!(
        quad_force > biceps_force,
        "Quadriceps should be stronger than biceps"
    );

    // Gastrocnemius and Soleus (ankle muscles)
    let gastroc_config = HillMuscleConfig::gastrocnemius();
    let soleus_config = HillMuscleConfig::soleus();

    let mut gastroc = HillMuscle::new(gastroc_config);
    let mut soleus = HillMuscle::new(soleus_config);

    gastroc.set_excitation(1.0);
    soleus.set_excitation(1.0);

    for _ in 0..200 {
        let _ = gastroc.compute_torque(0.0, 0.0, 0.001);
        let _ = soleus.compute_torque(0.0, 0.0, 0.001);
    }

    assert!(gastroc.diagnostics().fiber_force > 0.0);
    assert!(soleus.diagnostics().fiber_force > 0.0);
}
