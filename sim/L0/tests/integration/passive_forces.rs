//! Phase 0: Passive Force Tests for MuJoCo Parity.
//!
//! These tests verify the correctness of passive force implementation:
//! - **Spring Reference**: `springref` as equilibrium position (not `qpos0`)
//! - **Friction Loss**: Coulomb friction opposing joint velocity
//! - **Damping**: Viscous damping proportional to velocity
//!
//! Reference: MuJoCo passive force semantics (springref, damping, frictionloss)
//!
//! # Numerical Stability Notes
//!
//! These tests use small timesteps and conservative stiffness values to ensure
//! stability with the explicit Euler integrator. For a spring-mass system:
//! - Critical timestep: dt < 2 * sqrt(m/k)
//! - With m=1, k=100: dt_crit ≈ 0.2s, so dt=0.001 is very safe

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// Spring Reference Position Tests (Phase 0.1)
// ============================================================================

/// Test: Spring equilibrium is at `springref`, NOT `qpos0`.
///
/// MuJoCo Semantics:
/// - `qpos0`: Initial joint position at model load (for `mj_resetData()`)
/// - `springref`: Spring equilibrium position (where spring force is zero)
///
/// A joint with `ref="0"` (qpos0=0) and `springref="0.5"` should:
/// - Start at q=0
/// - Have spring force pulling toward q=0.5
/// - Settle near q=0.5 (with damping)
#[test]
fn test_springref_shifts_equilibrium() {
    // Joint starts at ref=0 (qpos0), but spring equilibrium is at springref=0.5
    // Use small timestep for stability with explicit integrator
    // Critical damping ratio: ζ = b / (2 * sqrt(k * I))
    // For hinge with sphere at origin, I ≈ 2/5 * m * r² = 0.004 kg⋅m²
    // With k=50, b=2: ζ ≈ 2 / (2 * sqrt(50 * 0.004)) ≈ 2.2 (overdamped)
    let mjcf = r#"
        <mujoco model="springref_test">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.0005"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="50" damping="2" springref="0.5" ref="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Verify initial position is qpos0=0, NOT springref
    assert_relative_eq!(data.qpos[0], 0.0, epsilon = 1e-10);

    // Verify springref was loaded correctly
    assert_relative_eq!(model.jnt_springref[0], 0.5, epsilon = 1e-10);

    // Step simulation until it settles (with damping, it should reach equilibrium)
    // Small timestep means more steps needed
    for _ in 0..20000 {
        data.step(&model).expect("step failed");
    }

    // Should settle near springref=0.5, not qpos0=0
    // Allow for some settling tolerance
    assert_relative_eq!(data.qpos[0], 0.5, epsilon = 0.05);
}

/// Test: Spring at equilibrium produces zero force.
///
/// When q == springref, the spring force should be exactly zero.
#[test]
fn test_springref_zero_force_at_equilibrium() {
    let mjcf = r#"
        <mujoco model="spring_equilibrium">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="1000" damping="0" springref="0.3"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Set position to springref exactly
    data.qpos[0] = 0.3;
    data.qvel[0] = 0.0; // Zero velocity (no damping force)

    // Compute passive forces
    data.forward(&model).expect("forward failed");

    // At equilibrium with zero velocity, passive force should be zero
    // (Need to call step to populate qfrc_passive, or use internal API)
    // For now, verify by stepping and checking that position doesn't change
    let initial_pos = data.qpos[0];
    data.step(&model).expect("step failed");
    data.step(&model).expect("step failed");
    data.step(&model).expect("step failed");

    // Position should remain unchanged (no net force)
    assert_relative_eq!(data.qpos[0], initial_pos, epsilon = 1e-8);
}

/// Test: Spring force direction is correct.
///
/// - When q < springref: force should be positive (toward springref)
/// - When q > springref: force should be negative (toward springref)
#[test]
fn test_springref_force_direction() {
    let mjcf = r#"
        <mujoco model="spring_direction">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="100" damping="50" springref="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Test: q < springref => should accelerate toward springref (positive direction)
    {
        let mut data = model.make_data();
        data.qpos[0] = -0.5; // Below equilibrium
        data.qvel[0] = 0.0;

        // One step should produce positive acceleration
        data.step(&model).expect("step failed");
        assert!(
            data.qvel[0] > 0.0,
            "Spring should accelerate toward springref when q < springref"
        );
    }

    // Test: q > springref => should accelerate toward springref (negative direction)
    {
        let mut data = model.make_data();
        data.qpos[0] = 0.5; // Above equilibrium
        data.qvel[0] = 0.0;

        // One step should produce negative acceleration
        data.step(&model).expect("step failed");
        assert!(
            data.qvel[0] < 0.0,
            "Spring should accelerate toward springref when q > springref"
        );
    }
}

/// Test: Spring stiffness scales force correctly.
///
/// F = -k * (q - springref)
/// For displacement d = q - springref:
/// - Doubling stiffness should double the force (and thus acceleration)
#[test]
fn test_spring_stiffness_scaling() {
    let make_model = |stiffness: f64| {
        format!(
            r#"
            <mujoco model="spring_stiffness">
                <compiler angle="radian"/>
                <option gravity="0 0 0" timestep="0.001"/>
                <worldbody>
                    <body name="pendulum" pos="0 0 0">
                        <joint name="hinge" type="hinge" axis="0 1 0"
                               stiffness="{}" damping="0" springref="0"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
            stiffness
        )
    };

    // Low stiffness
    let model_low = load_model(&make_model(50.0)).expect("should load");
    let mut data_low = model_low.make_data();
    data_low.qpos[0] = 0.1; // Displacement from equilibrium
    data_low.qvel[0] = 0.0;
    data_low.step(&model_low).expect("step failed");
    let accel_low = data_low.qvel[0] / 0.001; // v = a*t for first step

    // High stiffness (2x)
    let model_high = load_model(&make_model(100.0)).expect("should load");
    let mut data_high = model_high.make_data();
    data_high.qpos[0] = 0.1; // Same displacement
    data_high.qvel[0] = 0.0;
    data_high.step(&model_high).expect("step failed");
    let accel_high = data_high.qvel[0] / 0.001;

    // Acceleration should scale with stiffness (2x stiffness => 2x acceleration)
    assert_relative_eq!(accel_high / accel_low, 2.0, epsilon = 0.01);
}

/// Test: Default springref is zero.
///
/// When springref is not specified in MJCF, it should default to 0.
#[test]
fn test_springref_default_is_zero() {
    let mjcf = r#"
        <mujoco model="spring_default">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="100" damping="10"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Default springref should be 0
    assert_relative_eq!(model.jnt_springref[0], 0.0, epsilon = 1e-10);
}

/// Test: Slide joint springref works correctly.
///
/// Slide joints use linear displacement, not angular.
#[test]
fn test_springref_slide_joint() {
    let mjcf = r#"
        <mujoco model="slide_spring">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.002"/>
            <worldbody>
                <body name="slider" pos="0 0 0">
                    <joint name="slide" type="slide" axis="1 0 0"
                           stiffness="500" damping="20" springref="0.2"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Verify springref loaded
    assert_relative_eq!(model.jnt_springref[0], 0.2, epsilon = 1e-10);

    // Start at origin, should settle at springref
    data.qpos[0] = 0.0;
    for _ in 0..5000 {
        data.step(&model).expect("step failed");
    }

    // Should settle near springref=0.2
    assert_relative_eq!(data.qpos[0], 0.2, epsilon = 0.01);
}

// ============================================================================
// Friction Loss Tests (Phase 0.2)
// ============================================================================

/// Test: Friction loss opposes motion direction.
///
/// Coulomb friction: τ = -frictionloss * sign(qvel)
/// - Positive velocity => negative friction torque
/// - Negative velocity => positive friction torque
#[test]
fn test_frictionloss_opposes_motion() {
    let mjcf = r#"
        <mujoco model="friction_test">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0" frictionloss="10"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Test: Positive velocity should decelerate (friction opposes)
    {
        let mut data = model.make_data();
        data.qvel[0] = 1.0; // Moving in positive direction

        // Step and check that velocity decreases
        let v0 = data.qvel[0];
        data.step(&model).expect("step failed");
        assert!(
            data.qvel[0] < v0,
            "Friction should slow down positive velocity"
        );
    }

    // Test: Negative velocity should decelerate toward zero
    {
        let mut data = model.make_data();
        data.qvel[0] = -1.0; // Moving in negative direction

        let v0 = data.qvel[0];
        data.step(&model).expect("step failed");
        assert!(
            data.qvel[0] > v0,
            "Friction should slow down negative velocity (toward zero)"
        );
    }
}

/// Test: Friction loss reduces velocity over time.
///
/// With friction (and some damping for numerical stability), a moving joint
/// should slow down. Pure Coulomb friction with smooth approximation won't
/// bring velocity exactly to zero (tanh(small) ≈ small), so we add damping
/// to help convergence.
#[test]
fn test_frictionloss_reduces_velocity() {
    let mjcf = r#"
        <mujoco model="friction_rest">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="0" damping="1.0" frictionloss="5"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start with velocity
    data.qvel[0] = 2.0;
    let initial_vel = data.qvel[0];

    // Run simulation
    for _ in 0..10000 {
        data.step(&model).expect("step failed");
    }

    // Velocity should be significantly reduced (at least 50% reduction)
    assert!(
        data.qvel[0].abs() < initial_vel * 0.5,
        "Friction + damping should slow the joint, got {}",
        data.qvel[0]
    );
}

/// Test: Friction loss magnitude scales correctly.
///
/// Higher frictionloss should produce faster deceleration.
#[test]
fn test_frictionloss_scaling() {
    // Use PGS solver: the penalty model produces force = frictionloss * sign(vel),
    // so doubling frictionloss doubles deceleration.  Newton solver's Huber cost
    // has a quadratic zone whose slope (D) is independent of frictionloss, making
    // the scaling test meaningful only under PGS.
    let make_model = |frictionloss: f64| {
        format!(
            r#"
            <mujoco model="friction_scaling">
                <compiler angle="radian"/>
                <option gravity="0 0 0" timestep="0.001" solver="PGS"/>
                <worldbody>
                    <body name="pendulum" pos="0 0 0">
                        <joint name="hinge" type="hinge" axis="0 1 0"
                               stiffness="0" damping="0" frictionloss="{}"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
            frictionloss
        )
    };

    // Low friction
    let model_low = load_model(&make_model(5.0)).expect("should load");
    let mut data_low = model_low.make_data();
    data_low.qvel[0] = 1.0;
    data_low.step(&model_low).expect("step failed");
    let decel_low = 1.0 - data_low.qvel[0];

    // High friction (2x)
    let model_high = load_model(&make_model(10.0)).expect("should load");
    let mut data_high = model_high.make_data();
    data_high.qvel[0] = 1.0;
    data_high.step(&model_high).expect("step failed");
    let decel_high = 1.0 - data_high.qvel[0];

    // Higher friction should produce greater deceleration
    assert!(
        decel_high > decel_low,
        "Higher frictionloss should produce greater deceleration"
    );
    // Should be approximately 2x (within tolerance for smooth approximation)
    assert_relative_eq!(decel_high / decel_low, 2.0, epsilon = 0.1);
}

/// Test: Zero velocity produces zero friction force.
///
/// Friction is velocity-dependent: τ = -frictionloss * sign(qvel)
/// At zero velocity, the smooth approximation should produce negligible force.
#[test]
fn test_frictionloss_zero_at_rest() {
    let mjcf = r#"
        <mujoco model="friction_rest_test">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="0" damping="0" frictionloss="100"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start at rest
    data.qpos[0] = 0.5; // Arbitrary position
    data.qvel[0] = 0.0;

    // Step should not change velocity (no force)
    data.step(&model).expect("step failed");

    // Velocity should remain effectively zero
    assert!(
        data.qvel[0].abs() < 1e-10,
        "Friction should not accelerate a stationary joint, got {}",
        data.qvel[0]
    );
}

/// Test: Friction loss on slide joint.
#[test]
fn test_frictionloss_slide_joint() {
    let mjcf = r#"
        <mujoco model="slide_friction">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="slider" pos="0 0 0">
                    <joint name="slide" type="slide" axis="1 0 0"
                           stiffness="0" damping="0" frictionloss="10"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start with velocity
    data.qvel[0] = 0.5;
    let v0 = data.qvel[0];

    data.step(&model).expect("step failed");

    // Friction should reduce velocity
    assert!(data.qvel[0] < v0, "Slide joint friction should slow motion");
}

// ============================================================================
// Combined Passive Force Tests
// ============================================================================

/// Test: Spring + damper + friction work together correctly.
///
/// A joint with all three passive forces should:
/// - Oscillate toward springref (spring)
/// - Have damped oscillation (damper)
/// - Come to rest faster with friction
#[test]
fn test_combined_passive_forces() {
    // Use smaller stiffness and adequate damping for stability
    let mjcf = r#"
        <mujoco model="combined_passive">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.0005"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="50" damping="2" frictionloss="0.5" springref="0.3"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start displaced from equilibrium
    data.qpos[0] = 0.0;
    data.qvel[0] = 0.0;

    // Run simulation (more steps due to smaller timestep)
    for _ in 0..40000 {
        data.step(&model).expect("step failed");
    }

    // Should settle near springref
    assert_relative_eq!(data.qpos[0], 0.3, epsilon = 0.05);
    // Should be slow (may not be exactly at rest due to friction approximation)
    assert!(
        data.qvel[0].abs() < 0.1,
        "Joint should be nearly at rest, got {}",
        data.qvel[0]
    );
}

/// Test: Damping force scales with velocity.
///
/// Damping: τ = -b * qvel (linear in velocity)
/// At higher velocity, damping force is proportionally higher.
#[test]
fn test_damping_scales_with_velocity() {
    let mjcf = r#"
        <mujoco model="damping_scaling">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="0" damping="10" frictionloss="0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Test at two different velocities
    let mut data_low = model.make_data();
    data_low.qvel[0] = 1.0;
    let v0_low = data_low.qvel[0];
    data_low.step(&model).expect("step failed");
    let decel_low = v0_low - data_low.qvel[0];

    let mut data_high = model.make_data();
    data_high.qvel[0] = 2.0;
    let v0_high = data_high.qvel[0];
    data_high.step(&model).expect("step failed");
    let decel_high = v0_high - data_high.qvel[0];

    // Deceleration should scale linearly with velocity (2x velocity => 2x deceleration)
    assert_relative_eq!(decel_high / decel_low, 2.0, epsilon = 0.1);
}

// ============================================================================
// Edge Cases and Numerical Stability
// ============================================================================

/// Test: Moderate stiffness with appropriate timestep remains stable.
///
/// For explicit integration, stability requires dt < 2*sqrt(m/k).
/// With k=500, m≈0.004 (sphere inertia), dt_crit ≈ 0.006s.
/// We use dt=0.0002 which is safely below this threshold.
#[test]
fn test_moderate_stiffness_stability() {
    let mjcf = r#"
        <mujoco model="moderate_stiffness">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.0002"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="500" damping="5" springref="0.1"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    data.qpos[0] = 0.0;

    // Run simulation - should not explode
    for i in 0..50000 {
        data.step(&model).expect("step failed");

        // Check for numerical explosion
        assert!(
            data.qpos[0].abs() < 10.0,
            "Position exploded at step {}: {}",
            i,
            data.qpos[0]
        );
        assert!(
            data.qvel[0].abs() < 1000.0,
            "Velocity exploded at step {}: {}",
            i,
            data.qvel[0]
        );
    }

    // Should settle near springref
    assert_relative_eq!(data.qpos[0], 0.1, epsilon = 0.02);
}

/// Test: Multiple joints with different springref values.
#[test]
fn test_multiple_joints_different_springref() {
    let mjcf = r#"
        <mujoco model="multi_joint_spring">
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="link1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           stiffness="100" damping="10" springref="0.2"/>
                    <geom type="capsule" size="0.05 0.4" mass="1.0"/>
                    <body name="link2" pos="0 0 -1">
                        <joint name="j2" type="hinge" axis="0 1 0"
                               stiffness="100" damping="10" springref="-0.3"/>
                        <geom type="capsule" size="0.05 0.4" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Verify different springref values loaded
    assert_relative_eq!(model.jnt_springref[0], 0.2, epsilon = 1e-10);
    assert_relative_eq!(model.jnt_springref[1], -0.3, epsilon = 1e-10);

    // Run simulation
    for _ in 0..10000 {
        data.step(&model).expect("step failed");
    }

    // Each joint should settle near its own springref
    assert_relative_eq!(data.qpos[0], 0.2, epsilon = 0.02);
    assert_relative_eq!(data.qpos[1], -0.3, epsilon = 0.02);
}

/// Test: springref with gravity interaction.
///
/// With gravity, the joint should settle at a position where
/// spring torque balances gravitational torque. For a pendulum
/// rotating around Y-axis with mass below the joint, gravity
/// creates a torque that the spring must balance.
#[test]
fn test_springref_with_gravity() {
    // Pendulum with spring - geom offset creates moment arm for gravity
    // The hinge rotates around Y-axis, so gravity (in -Z) creates torque
    // when the pendulum is rotated away from vertical.
    //
    // At equilibrium: spring_torque = gravity_torque
    // k * (θ - springref) = m * g * L * sin(θ)
    //
    // With springref=0, the pendulum hangs straight down (θ=0) at equilibrium
    // since sin(0)=0 means no gravity torque, and spring at rest too.
    //
    // To test the spring actually works, we start displaced and verify
    // it returns toward springref=0.
    let mjcf = r#"
        <mujoco model="spring_gravity">
            <compiler angle="radian"/>
            <option gravity="0 0 -9.81" timestep="0.0005"/>
            <worldbody>
                <body name="pendulum" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="100" damping="5" springref="0"/>
                    <geom type="sphere" size="0.1" pos="0 0 -0.5" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start displaced from equilibrium
    data.qpos[0] = 0.5; // About 30 degrees

    // Run simulation
    for _ in 0..40000 {
        data.step(&model).expect("step failed");
    }

    // Should settle near springref=0 (straight down), not at 0.5
    // The spring pulls back toward springref, gravity assists for θ>0
    assert!(
        data.qpos[0].abs() < 0.1,
        "Spring should pull pendulum toward springref=0. Got q={}",
        data.qpos[0]
    );
}
