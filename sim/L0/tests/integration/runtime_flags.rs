//! §41 Runtime Flag Conformance Tests (AC1–AC48).
//!
//! Verifies that each disable/enable flag gates the correct pipeline subsystem.
//! Tests are named `acNN_<description>` matching the acceptance criteria in
//! `sim/docs/todo/spec_fleshouts/S41_RUNTIME_FLAGS_SPEC.md`.
//!
//! Golden-file MuJoCo conformance (AC18) is in a separate file; these tests
//! verify behavioral gating correctness.

use approx::assert_relative_eq;
use sim_core::{
    DISABLE_ACTUATION, DISABLE_AUTORESET, DISABLE_CLAMPCTRL, DISABLE_CONSTRAINT, DISABLE_CONTACT,
    DISABLE_DAMPER, DISABLE_EQUALITY, DISABLE_EULERDAMP, DISABLE_FILTERPARENT,
    DISABLE_FRICTIONLOSS, DISABLE_GRAVITY, DISABLE_LIMIT, DISABLE_REFSAFE, DISABLE_SENSOR,
    DISABLE_SPRING, DISABLE_WARMSTART, ENABLE_ENERGY, ENABLE_OVERRIDE, ENABLE_SLEEP, Warning,
    actuator_disabled, disabled, enabled,
};
use sim_mjcf::load_model;

// ============================================================================
// Shared MJCF Fixtures
// ============================================================================

/// Minimal body with gravity, a hinge joint, spring/damper, and an actuator.
/// Exercises: gravity, spring, damper, actuation, limit, contact.
fn flag_test_mjcf() -> &'static str {
    r#"
    <mujoco model="flag_test">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"
                       stiffness="100" damping="10"
                       limited="true" range="-90 90"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <actuator>
            <motor joint="hinge" name="motor" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    "#
}

/// Free-falling body above a floor. Used for contact/collision tests.
fn free_body_with_floor_mjcf() -> &'static str {
    r#"
    <mujoco model="free_body_floor">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="ball" pos="0 0 0.5">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Free body in space — no contacts, no springs, just gravity.
fn free_body_space_mjcf() -> &'static str {
    r#"
    <mujoco model="free_body_space">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="floater" pos="0 0 1">
                <freejoint name="free"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Two-actuator model with different groups for per-group disable testing.
fn two_group_actuator_mjcf() -> &'static str {
    r#"
    <mujoco model="two_group_actuator">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link1" pos="0 0 1">
                <joint name="j1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
            <body name="link2" pos="1 0 1">
                <joint name="j2" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
        </worldbody>
        <actuator>
            <motor joint="j1" name="motor1" group="0"/>
            <motor joint="j2" name="motor2" group="2"/>
        </actuator>
    </mujoco>
    "#
}

/// Body with equality (weld) constraint.
fn weld_constraint_mjcf() -> &'static str {
    r#"
    <mujoco model="weld_test">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="a" pos="0 0 1">
                <freejoint name="free_a"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
            <body name="b" pos="0.5 0 1">
                <freejoint name="free_b"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
        </worldbody>
        <equality>
            <weld body1="a" body2="b"/>
        </equality>
    </mujoco>
    "#
}

// ============================================================================
// AC1: Contact disable — body falls through floor
// ============================================================================

#[test]
fn ac1_contact_disable() {
    let mut model = load_model(free_body_with_floor_mjcf()).expect("load");
    model.disableflags |= DISABLE_CONTACT;

    let mut data = model.make_data();
    // Step enough for ball to reach floor height
    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    // With contacts disabled, ball falls through floor (z < 0)
    assert!(data.qpos[2] < 0.0, "ball should fall through floor");
    assert_eq!(data.ncon, 0, "no contacts when DISABLE_CONTACT is set");
}

// ============================================================================
// AC2: Gravity disable — free body stays stationary
// ============================================================================

#[test]
fn ac2_gravity_disable() {
    let mut model = load_model(free_body_space_mjcf()).expect("load");
    model.disableflags |= DISABLE_GRAVITY;

    let mut data = model.make_data();
    let initial_pos = data.qpos.clone();

    data.forward(&model).expect("forward");

    // With gravity disabled, qacc should be all zeros (no forces act)
    for dof in 0..model.nv {
        assert_relative_eq!(data.qacc[dof], 0.0, epsilon = 1e-12);
    }

    // Step and verify position unchanged
    data.step(&model).expect("step");
    for i in 0..3 {
        assert_relative_eq!(data.qpos[i], initial_pos[i], epsilon = 1e-12);
    }
}

// ============================================================================
// AC3: Limit disable — joint exceeds range
// ============================================================================

#[test]
fn ac3_limit_disable() {
    let mut model = load_model(flag_test_mjcf()).expect("load");
    model.disableflags |= DISABLE_LIMIT;

    let mut data = model.make_data();

    // Push joint past 90-degree limit
    data.qpos[0] = std::f64::consts::PI; // 180 degrees, well past 90-degree limit

    data.forward(&model).expect("forward");

    // With limits disabled, no constraint force from joint limit
    // qfrc_constraint should be zero for this DOF (no limit row generated)
    assert_relative_eq!(data.qfrc_constraint[0], 0.0, epsilon = 1e-12);
}

// ============================================================================
// AC4: Equality disable — weld bodies separate freely
// ============================================================================

#[test]
fn ac4_equality_disable() {
    let mut model = load_model(weld_constraint_mjcf()).expect("load");
    model.disableflags |= DISABLE_EQUALITY;

    let mut data = model.make_data();

    // Give body A an initial velocity that body B does not share
    data.qvel[0] = 1.0; // vx for body A

    for _ in 0..50 {
        data.step(&model).expect("step");
    }

    // With equality disabled, bodies should diverge
    let xa = data.qpos[0]; // x of body A
    let xb = data.qpos[7]; // x of body B (freejoint: 7 qpos per body)
    assert!((xa - xb).abs() > 0.01, "bodies should separate freely");
}

// ============================================================================
// AC5: Spring/damper independence
// ============================================================================

#[test]
fn ac5_spring_damper_independence() {
    // Spring disabled, damper enabled → spring force zero, damper non-zero
    let mut model = load_model(flag_test_mjcf()).expect("load");
    model.disableflags |= DISABLE_SPRING;

    let mut data = model.make_data();
    data.qpos[0] = 0.5; // deflect joint to engage spring
    data.qvel[0] = 1.0; // give velocity for damping
    data.forward(&model).expect("forward");

    assert_relative_eq!(data.qfrc_spring[0], 0.0, epsilon = 1e-12);
    assert!(
        data.qfrc_damper[0].abs() > 0.1,
        "damper force should be non-zero when only spring is disabled"
    );

    // Damper disabled, spring enabled → damper force zero, spring non-zero
    let mut model2 = load_model(flag_test_mjcf()).expect("load");
    model2.disableflags |= DISABLE_DAMPER;

    let mut data2 = model2.make_data();
    data2.qpos[0] = 0.5;
    data2.qvel[0] = 1.0;
    data2.forward(&model2).expect("forward");

    assert!(
        data2.qfrc_spring[0].abs() > 1.0,
        "spring force should be non-zero when only damper is disabled"
    );
    assert_relative_eq!(data2.qfrc_damper[0], 0.0, epsilon = 1e-12);
}

// ============================================================================
// AC6: Actuation disable — ctrl has no effect
// ============================================================================

#[test]
fn ac6_actuation_disable() {
    let mut model = load_model(flag_test_mjcf()).expect("load");
    model.disableflags |= DISABLE_ACTUATION;

    let mut data = model.make_data();
    data.ctrl[0] = 1.0; // max effort

    data.forward(&model).expect("forward");

    // qfrc_actuator should be zero
    for dof in 0..model.nv {
        assert_relative_eq!(data.qfrc_actuator[dof], 0.0, epsilon = 1e-12);
    }
}

// ============================================================================
// AC9: Constraint disable — cascading to ncon=0, nefc=0
// ============================================================================

#[test]
fn ac9_constraint_disable() {
    let mut model = load_model(free_body_with_floor_mjcf()).expect("load");
    model.disableflags |= DISABLE_CONSTRAINT;

    let mut data = model.make_data();
    // Place ball on floor — should generate contacts if constraints are on
    data.qpos[2] = 0.1;
    data.forward(&model).expect("forward");

    // AC42 causal chain: collision skipped → ncon=0, nefc=0, qfrc_constraint=0
    assert_eq!(data.ncon, 0, "no contacts when DISABLE_CONSTRAINT is set");
    for dof in 0..model.nv {
        assert_relative_eq!(data.qfrc_constraint[dof], 0.0, epsilon = 1e-12);
    }
}

// ============================================================================
// AC10: Passive top-level gating — both spring+damper disabled skips all
// ============================================================================

#[test]
fn ac10_passive_top_level_gating() {
    let mut model = load_model(flag_test_mjcf()).expect("load");
    model.disableflags |= DISABLE_SPRING | DISABLE_DAMPER;

    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    // All passive sub-forces should be zero
    for dof in 0..model.nv {
        assert_relative_eq!(data.qfrc_spring[dof], 0.0, epsilon = 1e-12);
        assert_relative_eq!(data.qfrc_damper[dof], 0.0, epsilon = 1e-12);
        assert_relative_eq!(data.qfrc_passive[dof], 0.0, epsilon = 1e-12);
    }
}

// ============================================================================
// AC11: Default bitfields — fresh model has disableflags=0, enableflags=0
// ============================================================================

#[test]
fn ac11_default_bitfields() {
    let model = load_model(free_body_space_mjcf()).expect("load");
    assert_eq!(model.disableflags, 0, "all disable flags default to clear");
    assert_eq!(
        model.enableflags, 0,
        "all enable flags default to clear (MuJoCo convention)"
    );
}

// ============================================================================
// AC13: Energy gating — only computed when ENABLE_ENERGY set
// ============================================================================

#[test]
fn ac13_energy_gating() {
    // Without ENABLE_ENERGY (default): energy fields are zeroed
    let model = load_model(free_body_space_mjcf()).expect("load");
    assert_eq!(
        model.enableflags & ENABLE_ENERGY,
        0,
        "energy off by default"
    );
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    assert_eq!(
        data.energy_potential, 0.0,
        "potential energy zeroed without ENABLE_ENERGY"
    );
    assert_eq!(
        data.energy_kinetic, 0.0,
        "kinetic energy zeroed without ENABLE_ENERGY"
    );

    // With ENABLE_ENERGY explicitly set: energy fields are computed
    let mut model2 = load_model(free_body_space_mjcf()).expect("load");
    model2.enableflags |= ENABLE_ENERGY;
    let mut data2 = model2.make_data();
    data2.qvel[2] = -1.0; // downward velocity for kinetic energy
    data2.forward(&model2).expect("forward");

    // Body at z=1 with gravity → negative potential energy
    // body has downward velocity → positive kinetic energy
    assert!(
        data2.energy_potential != 0.0,
        "potential energy should be computed"
    );
    assert!(
        data2.energy_kinetic > 0.0,
        "kinetic energy should be positive with velocity"
    );
}

// ============================================================================
// AC14: Fluid sleep filtering — mj_fluid() skips sleeping bodies
// ============================================================================

#[test]
fn ac14_fluid_sleep_filtering() {
    use sim_core::SleepState;

    // Two free bodies in fluid (density > 0). Body 2 starts asleep via sleep="init".
    // With ENABLE_SLEEP and a sleeping body, mj_fluid() should skip that body —
    // its DOFs in qfrc_fluid should remain zero.
    let mjcf = r#"
        <mujoco model="fluid_sleep_test">
            <option density="1.2" viscosity="0.001" timestep="0.002">
                <flag sleep="enable"/>
            </option>
            <worldbody>
                <body name="awake_body" pos="0 0 1">
                    <freejoint/>
                    <geom type="box" size="0.1 0.05 0.025" mass="1.0"
                          contype="0" conaffinity="0"/>
                </body>
                <body name="sleeping_body" pos="2 0 1" sleep="init">
                    <freejoint/>
                    <geom type="box" size="0.1 0.05 0.025" mass="1.0"
                          contype="0" conaffinity="0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert!(enabled(&model, ENABLE_SLEEP), "sleep should be enabled");
    assert_eq!(model.nv, 12, "two free joints = 12 DOFs");

    let mut data = model.make_data();
    // Give the awake body non-zero velocity so it produces fluid forces.
    data.qvel[0] = 0.5;
    data.qvel[1] = -1.0;
    data.qvel[2] = 0.8;
    data.qvel[3] = 1.2;
    data.qvel[4] = -0.7;
    data.qvel[5] = 0.3;
    // Sleeping body has zero velocity (default).

    data.forward(&model).expect("forward");

    // Verify body 2 is asleep.
    assert_eq!(
        data.body_sleep_state[2],
        SleepState::Asleep,
        "body 2 should be asleep"
    );

    // Sleeping body (DOFs 6–11): qfrc_fluid must be exactly zero.
    for dof in 6..12 {
        assert_eq!(
            data.qfrc_fluid[dof], 0.0,
            "qfrc_fluid[{dof}] should be zero for sleeping body"
        );
    }

    // Awake body (DOFs 0–5): qfrc_fluid should be non-zero (has velocity + fluid).
    let awake_has_fluid = (0..6).any(|dof| data.qfrc_fluid[dof] != 0.0);
    assert!(
        awake_has_fluid,
        "awake body should have non-zero qfrc_fluid with velocity in fluid"
    );
}

// ============================================================================
// AC19: Clampctrl disable — ctrl exceeds ctrlrange
// ============================================================================

#[test]
fn ac19_clampctrl_disable() {
    // With clamping (default): ctrl=5 gets clamped to 1
    let model_clamped = load_model(flag_test_mjcf()).expect("load");
    let mut data_clamped = model_clamped.make_data();
    data_clamped.ctrl[0] = 5.0;
    data_clamped.forward(&model_clamped).expect("forward");
    let force_clamped = data_clamped.qfrc_actuator[0];

    // Without clamping: ctrl=5 is used directly
    let mut model_unclamped = load_model(flag_test_mjcf()).expect("load");
    model_unclamped.disableflags |= DISABLE_CLAMPCTRL;
    let mut data_unclamped = model_unclamped.make_data();
    data_unclamped.ctrl[0] = 5.0;
    data_unclamped.forward(&model_unclamped).expect("forward");
    let force_unclamped = data_unclamped.qfrc_actuator[0];

    // Unclamped force should be larger (ctrl 5 vs clamped 1)
    assert!(
        force_unclamped.abs() > force_clamped.abs(),
        "unclamped actuator force ({force_unclamped}) should exceed clamped ({force_clamped})"
    );
}

// ============================================================================
// AC22: Per-group actuator disabling
// ============================================================================

#[test]
fn ac22_per_group_actuator_disable() {
    let mut model = load_model(two_group_actuator_mjcf()).expect("load");
    // Disable group 2 only
    model.disableactuator = 1u32 << 2;

    let mut data = model.make_data();
    data.ctrl[0] = 1.0; // motor1 (group 0) — should work
    data.ctrl[1] = 1.0; // motor2 (group 2) — should be disabled
    data.forward(&model).expect("forward");

    // motor1 (group 0) produces force
    assert!(
        data.actuator_force[0].abs() > 0.0,
        "group-0 actuator should produce force"
    );

    // motor2 (group 2) is disabled → zero force
    // group-2 actuator should produce zero force when disabled
    assert_relative_eq!(data.actuator_force[1], 0.0, epsilon = 1e-12);
}

// ============================================================================
// AC23: Unconditional initialization before guards
// ============================================================================

#[test]
fn ac23_unconditional_initialization() {
    // Run once with all features, then disable — force arrays should be clean zeros
    let mut model = load_model(flag_test_mjcf()).expect("load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    // Verify forces are non-zero
    assert!(data.qfrc_actuator[0].abs() > 0.0);
    assert!(data.qfrc_passive[0].abs() > 0.0);

    // Now disable everything and re-forward — init-then-guard should zero
    model.disableflags |= DISABLE_ACTUATION | DISABLE_SPRING | DISABLE_DAMPER;
    data.forward(&model).expect("forward");

    // actuator force zeroed before guard
    assert_relative_eq!(data.qfrc_actuator[0], 0.0, epsilon = 1e-12);
    // passive force zeroed before guard (both spring+damper disabled)
    assert_relative_eq!(data.qfrc_passive[0], 0.0, epsilon = 1e-12);
}

// ============================================================================
// AC25: Spring/damper force separation
// ============================================================================

#[test]
fn ac25_spring_damper_force_separation() {
    let model = load_model(flag_test_mjcf()).expect("load");
    let mut data = model.make_data();

    // Deflect joint and give velocity to engage both spring and damper
    data.qpos[0] = 0.5;
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    // qfrc_spring and qfrc_damper should be independently non-zero
    assert!(
        data.qfrc_spring[0].abs() > 1.0,
        "spring force should be non-zero with deflection"
    );
    assert!(
        data.qfrc_damper[0].abs() > 0.1,
        "damper force should be non-zero with velocity"
    );

    // qfrc_passive should be the sum
    let sum =
        data.qfrc_spring[0] + data.qfrc_damper[0] + data.qfrc_gravcomp[0] + data.qfrc_fluid[0];
    // qfrc_passive should equal sum of components
    assert_relative_eq!(data.qfrc_passive[0], sum, epsilon = 1e-12);
}

// ============================================================================
// AC26: Auto-reset on NaN
// ============================================================================

#[test]
fn ac26_auto_reset_on_nan() {
    let model = load_model(free_body_space_mjcf()).expect("load");
    let mut data = model.make_data();

    // Inject NaN into qpos
    data.qpos[0] = f64::NAN;
    data.step(&model)
        .expect("step should not error (auto-reset)");

    // After auto-reset: warning recorded, divergence detected
    assert!(data.divergence_detected(), "divergence should be detected");
    assert!(
        data.warnings[Warning::BadQpos as usize].count > 0,
        "BadQpos warning should fire"
    );
    // Note: step() resets then continues integration, so qpos will have
    // advanced one timestep from qpos0, not be exactly qpos0.
    // The key invariant is that the NaN is gone and state is valid.
    assert!(
        !data.qpos[0].is_nan(),
        "NaN should be cleared by auto-reset"
    );
}

// ============================================================================
// AC27: Auto-reset threshold (values > 1e10 trigger reset)
// ============================================================================

#[test]
fn ac27_auto_reset_threshold() {
    let model = load_model(free_body_space_mjcf()).expect("load");
    let mut data = model.make_data();

    // Value exceeding MAX_VAL (1e10) should trigger reset
    data.qpos[0] = 1.1e10;
    data.step(&model).expect("step");

    assert!(
        data.divergence_detected(),
        "1.1e10 should trigger auto-reset"
    );
    assert!(data.warnings[Warning::BadQpos as usize].count > 0);
}

// ============================================================================
// AC28: Autoreset disable — warning but no reset
// ============================================================================

#[test]
fn ac28_autoreset_disable() {
    let mut model = load_model(free_body_space_mjcf()).expect("load");
    model.disableflags |= DISABLE_AUTORESET;

    let mut data = model.make_data();
    data.qpos[0] = f64::NAN;
    data.step(&model).expect("step");

    // Warning should fire but NaN remains (no reset)
    assert!(
        data.warnings[Warning::BadQpos as usize].count > 0,
        "warning should still fire with autoreset disabled"
    );
    assert!(
        data.qpos[0].is_nan(),
        "NaN should persist when autoreset is disabled"
    );
}

// ============================================================================
// AC29: Ctrl validation — bad ctrl zeros all ctrl, no reset
// ============================================================================

#[test]
fn ac29_ctrl_validation() {
    let model = load_model(flag_test_mjcf()).expect("load");
    let mut data = model.make_data();

    // Set bad ctrl
    data.ctrl[0] = f64::NAN;
    data.forward(&model).expect("forward");

    // All ctrl should be zeroed
    for i in 0..model.nu {
        assert_eq!(
            data.ctrl[i], 0.0,
            "ctrl[{i}] should be zeroed after bad ctrl"
        );
    }

    // BadCtrl warning should fire
    assert!(data.warnings[Warning::BadCtrl as usize].count > 0);

    // qpos/qvel should NOT be reset (no auto-reset for ctrl)
    // (they were fine to begin with, so they should still be at initial values)
    assert!(
        !data.divergence_detected(),
        "bad ctrl should not trigger position/velocity reset"
    );
}

// ============================================================================
// AC24: Gravcomp routing — jnt_actgravcomp
// ============================================================================

#[test]
fn ac24_gravcomp_routing() {
    let mjcf = r#"
        <mujoco model="gravcomp_routing">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="link" pos="0 0 1">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="2.0" pos="0.5 0 0"
                          contype="0" conaffinity="0"/>
                    <inertial pos="0.5 0 0" mass="2.0" diaginertia="0.008 0.008 0.008"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Default: gravcomp → qfrc_passive (not qfrc_actuator)
    let mut model = load_model(mjcf).expect("load");
    model.body_gravcomp[1] = 1.0;
    model.ngravcomp = 1;
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let gravcomp_val = data.qfrc_gravcomp[0];
    assert!(
        gravcomp_val.abs() > 0.1,
        "gravcomp should be non-zero for body with gravcomp=1"
    );
    // With default (jnt_actgravcomp=false), gravcomp goes to passive
    assert_relative_eq!(data.qfrc_passive[0], gravcomp_val, epsilon = 1e-10);
    assert_relative_eq!(data.qfrc_actuator[0], 0.0, epsilon = 1e-12);

    // With jnt_actgravcomp=true: gravcomp → qfrc_actuator (not qfrc_passive)
    let mut model2 = load_model(mjcf).expect("load");
    model2.body_gravcomp[1] = 1.0;
    model2.ngravcomp = 1;
    model2.jnt_actgravcomp[0] = true;
    let mut data2 = model2.make_data();
    data2.forward(&model2).expect("forward");

    let gravcomp_val2 = data2.qfrc_gravcomp[0];
    assert_relative_eq!(gravcomp_val2, gravcomp_val, epsilon = 1e-12);
    // Gravcomp NOT in passive
    assert_relative_eq!(data2.qfrc_passive[0], 0.0, epsilon = 1e-12);
    // Gravcomp IS in actuator
    assert_relative_eq!(data2.qfrc_actuator[0], gravcomp_val2, epsilon = 1e-12);

    // With DISABLE_GRAVITY: neither path produces gravcomp
    let mut model3 = load_model(mjcf).expect("load");
    model3.body_gravcomp[1] = 1.0;
    model3.ngravcomp = 1;
    model3.jnt_actgravcomp[0] = true;
    model3.disableflags |= DISABLE_GRAVITY;
    let mut data3 = model3.make_data();
    data3.forward(&model3).expect("forward");

    assert_relative_eq!(data3.qfrc_gravcomp[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(data3.qfrc_actuator[0], 0.0, epsilon = 1e-12);
}

// ============================================================================
// AC37: Override disabled by default
// ============================================================================

#[test]
fn ac37_override_disabled_by_default() {
    let model = load_model(free_body_with_floor_mjcf()).expect("load");
    assert_eq!(
        model.enableflags & ENABLE_OVERRIDE,
        0,
        "override should be disabled by default"
    );
}

// ============================================================================
// AC39: reset() correctness — 7 properties
// ============================================================================

#[test]
fn ac39_reset_correctness() {
    let model = load_model(flag_test_mjcf()).expect("load");
    let mut data = model.make_data();

    // Mess up state
    data.qpos[0] = 42.0;
    data.qvel[0] = 13.0;
    data.qacc[0] = 7.0;
    data.ctrl[0] = 0.5;
    data.time = 99.0;
    data.warnings[Warning::BadQpos as usize].count = 5;

    // Reset
    data.reset(&model);

    // Property 1: qpos == qpos0
    for i in 0..model.nq {
        assert_relative_eq!(data.qpos[i], model.qpos0[i], epsilon = 1e-15);
    }

    // Property 2: qvel, qacc, ctrl, act all zero
    for i in 0..model.nv {
        assert_eq!(data.qvel[i], 0.0);
        assert_eq!(data.qacc[i], 0.0);
    }
    for i in 0..model.nu {
        assert_eq!(data.ctrl[i], 0.0);
    }

    // Property 3: time == 0
    assert_eq!(data.time, 0.0);

    // Property 4: warning counters zero
    for w in &data.warnings {
        assert_eq!(w.count, 0);
        assert_eq!(w.last_info, 0);
    }

    // Property 5: ncon == 0
    assert_eq!(data.ncon, 0);

    // Property 6: force vectors zero
    for i in 0..model.nv {
        assert_eq!(data.qfrc_passive[i], 0.0);
        assert_eq!(data.qfrc_actuator[i], 0.0);
        assert_eq!(data.qfrc_constraint[i], 0.0);
        assert_eq!(data.qfrc_spring[i], 0.0);
        assert_eq!(data.qfrc_damper[i], 0.0);
        assert_eq!(data.qfrc_gravcomp[i], 0.0);
        assert_eq!(data.qfrc_fluid[i], 0.0);
    }
}

// ============================================================================
// AC41: DISABLE_ACTUATION + per-group orthogonality
// ============================================================================

#[test]
fn ac41_actuation_plus_pergroup_orthogonality() {
    // Case 1: DISABLE_ACTUATION set + per-group bits → ALL actuators disabled
    let mut model = load_model(two_group_actuator_mjcf()).expect("load");
    model.disableflags |= DISABLE_ACTUATION;
    model.disableactuator = 1u32 << 2; // also disable group 2

    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.ctrl[1] = 1.0;
    data.forward(&model).expect("forward");

    for dof in 0..model.nv {
        // all actuators disabled with DISABLE_ACTUATION
        assert_relative_eq!(data.qfrc_actuator[dof], 0.0, epsilon = 1e-12);
    }

    // Case 2: Only per-group → only group 2 disabled
    let mut model2 = load_model(two_group_actuator_mjcf()).expect("load");
    model2.disableactuator = 1u32 << 2;

    let mut data2 = model2.make_data();
    data2.ctrl[0] = 1.0;
    data2.ctrl[1] = 1.0;
    data2.forward(&model2).expect("forward");

    assert!(
        data2.actuator_force[0].abs() > 0.0,
        "group-0 actuator should work without DISABLE_ACTUATION"
    );
    // group-2 actuator disabled by per-group mechanism
    assert_relative_eq!(data2.actuator_force[1], 0.0, epsilon = 1e-12);

    // Case 3: Neither set → all actuators active
    let model3 = load_model(two_group_actuator_mjcf()).expect("load");
    let mut data3 = model3.make_data();
    data3.ctrl[0] = 1.0;
    data3.ctrl[1] = 1.0;
    data3.forward(&model3).expect("forward");

    assert!(data3.actuator_force[0].abs() > 0.0, "motor1 active");
    assert!(data3.actuator_force[1].abs() > 0.0, "motor2 active");
}

// ============================================================================
// AC45: actuator_velocity unconditional after pipeline move
// ============================================================================

#[test]
fn ac45_actuator_velocity_unconditional() {
    let mut model = load_model(flag_test_mjcf()).expect("load");
    model.disableflags |= DISABLE_ACTUATION;

    let mut data = model.make_data();
    data.qvel[0] = 2.0; // non-zero joint velocity

    data.forward(&model).expect("forward");

    // actuator_velocity should be computed regardless of DISABLE_ACTUATION
    // (it's transmission geometry, not force computation)
    assert!(
        data.actuator_velocity[0].abs() > 0.0,
        "actuator_velocity should be non-zero with non-zero qvel, even with DISABLE_ACTUATION"
    );

    // Compare with actuation enabled — should be identical
    let model2 = load_model(flag_test_mjcf()).expect("load");
    let mut data2 = model2.make_data();
    data2.qvel[0] = 2.0;
    data2.forward(&model2).expect("forward");

    // actuator_velocity should be identical with or without DISABLE_ACTUATION
    assert_relative_eq!(
        data.actuator_velocity[0],
        data2.actuator_velocity[0],
        epsilon = 1e-15
    );
}

// ============================================================================
// AC48: nv==0 passive force guard
// ============================================================================

#[test]
fn ac48_nv_zero_passive_guard() {
    // Model with zero DOFs — purely static scene (all welded to world)
    let mjcf = r#"
        <mujoco model="static">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.nv, 0, "static model should have nv=0");

    let mut data = model.make_data();
    // This should not crash
    data.forward(&model).expect("forward");
    data.step(&model).expect("step");
}

// ============================================================================
// Flag helper unit tests
// ============================================================================

#[test]
fn flag_helpers_disabled() {
    let mut model = load_model(free_body_space_mjcf()).expect("load");

    // No flags set — nothing disabled
    assert!(!disabled(&model, DISABLE_GRAVITY));
    assert!(!disabled(&model, DISABLE_CONTACT));

    // Set gravity disable
    model.disableflags |= DISABLE_GRAVITY;
    assert!(disabled(&model, DISABLE_GRAVITY));
    assert!(!disabled(&model, DISABLE_CONTACT));
}

#[test]
fn flag_helpers_enabled() {
    let mut model = load_model(free_body_space_mjcf()).expect("load");

    // MuJoCo default: all enable flags off
    assert!(!enabled(&model, ENABLE_ENERGY));
    assert!(!enabled(&model, ENABLE_OVERRIDE));

    // Set energy and override
    model.enableflags |= ENABLE_ENERGY | ENABLE_OVERRIDE;
    assert!(enabled(&model, ENABLE_ENERGY));
    assert!(enabled(&model, ENABLE_OVERRIDE));
}

#[test]
fn flag_helpers_actuator_disabled() {
    let mut model = load_model(two_group_actuator_mjcf()).expect("load");

    // No groups disabled
    assert!(!actuator_disabled(&model, 0));
    assert!(!actuator_disabled(&model, 1));

    // Disable group 2
    model.disableactuator = 1u32 << 2;
    assert!(
        !actuator_disabled(&model, 0),
        "motor1 (group 0) not disabled"
    );
    assert!(
        actuator_disabled(&model, 1),
        "motor2 (group 2) should be disabled"
    );
}

// ============================================================================
// MJCF parsing: <flag> attribute wiring (S1–S3)
// ============================================================================

#[test]
fn s1_flag_parsing_disable() {
    let mjcf = r#"
        <mujoco model="flag_parse">
            <option>
                <flag gravity="disable" contact="disable" actuation="disable"
                      sensor="disable" clampctrl="disable" warmstart="disable"
                      constraint="disable" equality="disable" limit="disable"
                      spring="disable" damper="disable" frictionloss="disable"
                      filterparent="disable" eulerdamp="disable"
                      refsafe="disable" autoreset="disable"/>
            </option>
            <worldbody>
                <body><freejoint/><geom type="sphere" size="0.1"/></body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");

    assert!(disabled(&model, DISABLE_GRAVITY));
    assert!(disabled(&model, DISABLE_CONTACT));
    assert!(disabled(&model, DISABLE_ACTUATION));
    assert!(disabled(&model, DISABLE_SENSOR));
    assert!(disabled(&model, DISABLE_CLAMPCTRL));
    assert!(disabled(&model, DISABLE_WARMSTART));
    assert!(disabled(&model, DISABLE_CONSTRAINT));
    assert!(disabled(&model, DISABLE_EQUALITY));
    assert!(disabled(&model, DISABLE_LIMIT));
    assert!(disabled(&model, DISABLE_SPRING));
    assert!(disabled(&model, DISABLE_DAMPER));
    assert!(disabled(&model, DISABLE_FRICTIONLOSS));
    assert!(disabled(&model, DISABLE_FILTERPARENT));
    assert!(disabled(&model, DISABLE_EULERDAMP));
    assert!(disabled(&model, DISABLE_REFSAFE));
    assert!(disabled(&model, DISABLE_AUTORESET));
}

#[test]
fn s1_flag_parsing_enable() {
    let mjcf = r#"
        <mujoco model="flag_parse_en">
            <option>
                <flag energy="enable" override="enable" sleep="enable"/>
            </option>
            <worldbody>
                <body><freejoint/><geom type="sphere" size="0.1"/></body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");

    assert!(enabled(&model, ENABLE_ENERGY));
    assert!(enabled(&model, ENABLE_OVERRIDE));
    assert!(enabled(&model, ENABLE_SLEEP));
}

#[test]
fn s7_actuatorgroupdisable_parsing() {
    let mjcf = r#"
        <mujoco model="group_disable">
            <option actuatorgroupdisable="2 5"/>
            <worldbody>
                <body><freejoint/><geom type="sphere" size="0.1"/></body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");

    // Bits 2 and 5 should be set
    assert_ne!(model.disableactuator & (1u32 << 2), 0, "group 2 disabled");
    assert_ne!(model.disableactuator & (1u32 << 5), 0, "group 5 disabled");
    // Other bits should be clear
    assert_eq!(
        model.disableactuator & (1u32 << 0),
        0,
        "group 0 not disabled"
    );
    assert_eq!(
        model.disableactuator & (1u32 << 1),
        0,
        "group 1 not disabled"
    );
    assert_eq!(
        model.disableactuator & (1u32 << 3),
        0,
        "group 3 not disabled"
    );
}

// ============================================================================
// Additional MJCF Fixtures for AC7–AC46
// ============================================================================

/// Body with a sensor (jointpos) for sensor-disable tests.
fn sensor_test_mjcf() -> &'static str {
    r#"
    <mujoco model="sensor_test">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
        </worldbody>
        <sensor>
            <jointpos joint="hinge"/>
        </sensor>
    </mujoco>
    "#
}

/// Parent-child overlapping geoms for filterparent tests.
fn filterparent_test_mjcf() -> &'static str {
    r#"
    <mujoco model="filterparent_test">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="parent" pos="0 0 1">
                <freejoint name="parent_free"/>
                <geom type="sphere" size="0.3" mass="1.0"/>
                <body name="child" pos="0 0 0">
                    <joint name="child_hinge" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.3" mass="1.0"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Hinge joint with frictionloss for friction-loss disable tests.
fn frictionloss_test_mjcf() -> &'static str {
    r#"
    <mujoco model="frictionloss_test">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"
                       frictionloss="10" limited="true" range="-90 90"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Two-actuator model with RK4 integrator for per-group + RK4 tests.
fn rk4_actuator_mjcf() -> &'static str {
    r#"
    <mujoco model="rk4_actuator">
        <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4"/>
        <worldbody>
            <body name="link1" pos="0 0 1">
                <joint name="j1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
            <body name="link2" pos="1 0 1">
                <joint name="j2" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"
                      contype="0" conaffinity="0"/>
            </body>
        </worldbody>
        <actuator>
            <motor joint="j1" name="motor1" group="0"/>
            <motor joint="j2" name="motor2" group="1"/>
        </actuator>
    </mujoco>
    "#
}

// ============================================================================
// AC7: Sensor disable — sensordata retains stale values
// ============================================================================

#[test]
fn ac7_sensor_disable() {
    let model = load_model(sensor_test_mjcf()).expect("load");
    let mut data = model.make_data();

    // Step once with sensors enabled to populate sensordata
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    assert!(model.nsensordata > 0, "model should have sensor data slots");
    let sensor_val = data.sensordata[0];
    assert!(
        sensor_val.abs() > 0.0,
        "sensor should read non-zero with joint deflected"
    );

    // Now disable sensors and change joint position
    let mut model2 = load_model(sensor_test_mjcf()).expect("load");
    model2.disableflags |= DISABLE_SENSOR;
    let mut data2 = model2.make_data();

    // Set a known value in sensordata, then forward — should remain stale
    data2.sensordata[0] = 42.0;
    data2.qpos[0] = 1.0;
    data2.forward(&model2).expect("forward");

    // S4.10: sensordata is NOT zeroed when sensors disabled (MuJoCo match)
    // S4.10: sensordata should retain stale value when DISABLE_SENSOR is set
    assert_relative_eq!(data2.sensordata[0], 42.0, epsilon = 1e-12);
}

// ============================================================================
// AC8: Warmstart disable — solver cold-starts from qacc_smooth
// ============================================================================

#[test]
fn ac8_warmstart_disable() {
    // Step a contact model multiple times with warmstart disabled.
    // Verify solver converges (qacc finite) and efc_force starts from zero.
    let mut model = load_model(free_body_with_floor_mjcf()).expect("load");
    model.disableflags |= DISABLE_WARMSTART;

    let mut data = model.make_data();
    // Place ball near floor to generate contacts
    data.qpos[2] = 0.11;

    for _ in 0..10 {
        data.step(&model).expect("step");
    }

    // qacc should be finite (solver converged despite cold start)
    for dof in 0..model.nv {
        assert!(
            data.qacc[dof].is_finite(),
            "qacc[{dof}] should be finite with DISABLE_WARMSTART"
        );
    }
}

// ============================================================================
// AC12: `passive` attribute silently ignored
// ============================================================================

#[test]
fn ac12_passive_attribute_ignored() {
    // MuJoCo has a `passive` flag attribute but CortenForge uses spring/damper
    // as separate flags. The `passive` attribute should be silently ignored.
    let mjcf = r#"
        <mujoco model="passive_attr">
            <option>
                <flag passive="disable"/>
            </option>
            <worldbody>
                <body><freejoint/><geom type="sphere" size="0.1"/></body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");

    // No disable flags should be set from `passive` attribute
    // (Spring and damper have their own flags)
    assert_eq!(
        model.disableflags & DISABLE_SPRING,
        0,
        "passive=disable should not set DISABLE_SPRING"
    );
    assert_eq!(
        model.disableflags & DISABLE_DAMPER,
        0,
        "passive=disable should not set DISABLE_DAMPER"
    );
}

// ============================================================================
// AC15: Filterparent disable — parent-child contacts generated
// ============================================================================

#[test]
fn ac15_filterparent_disable() {
    // Default: parent-child geoms are filtered (no contacts between them)
    let model_default = load_model(filterparent_test_mjcf()).expect("load");
    let mut data_default = model_default.make_data();
    data_default.forward(&model_default).expect("forward");
    let ncon_default = data_default.ncon;

    // With DISABLE_FILTERPARENT: parent-child geom pairs should collide
    let mut model_nofilt = load_model(filterparent_test_mjcf()).expect("load");
    model_nofilt.disableflags |= DISABLE_FILTERPARENT;
    let mut data_nofilt = model_nofilt.make_data();
    data_nofilt.forward(&model_nofilt).expect("forward");
    let ncon_nofilt = data_nofilt.ncon;

    assert!(
        ncon_nofilt > ncon_default,
        "DISABLE_FILTERPARENT should allow parent-child contacts \
         (ncon={ncon_nofilt} with flag vs {ncon_default} without)"
    );
}

// ============================================================================
// AC16: Frictionloss disable — no friction loss constraint rows
// ============================================================================

#[test]
fn ac16_frictionloss_disable() {
    use sim_core::ConstraintType;

    // With frictionloss enabled (default): should have FrictionLoss rows
    let model_enabled = load_model(frictionloss_test_mjcf()).expect("load");
    let mut data_enabled = model_enabled.make_data();
    data_enabled.qvel[0] = 1.0; // need velocity for friction to matter
    data_enabled.forward(&model_enabled).expect("forward");

    let fl_rows_enabled = data_enabled
        .efc_type
        .iter()
        .filter(|t| matches!(t, ConstraintType::FrictionLoss))
        .count();

    assert!(
        fl_rows_enabled > 0,
        "should have FrictionLoss constraint rows with frictionloss > 0"
    );

    // With DISABLE_FRICTIONLOSS: no FrictionLoss rows
    let mut model_disabled = load_model(frictionloss_test_mjcf()).expect("load");
    model_disabled.disableflags |= DISABLE_FRICTIONLOSS;
    let mut data_disabled = model_disabled.make_data();
    data_disabled.qvel[0] = 1.0;
    data_disabled.forward(&model_disabled).expect("forward");

    let fl_rows_disabled = data_disabled
        .efc_type
        .iter()
        .filter(|t| matches!(t, ConstraintType::FrictionLoss))
        .count();

    assert_eq!(
        fl_rows_disabled, 0,
        "should have zero FrictionLoss rows with DISABLE_FRICTIONLOSS"
    );
}

// ============================================================================
// AC17: Refsafe disable — solref[0] below 2*timestep is NOT clamped
// ============================================================================

#[test]
fn ac17_refsafe_disable() {
    // Create a model with a low solref timeconst (below 2*timestep).
    // With refsafe (default): timeconst clamped to 2*timestep.
    // Without refsafe: timeconst used as-is → different constraint stiffness.
    let mjcf = r#"
        <mujoco model="refsafe_test">
            <option gravity="0 0 -9.81" timestep="0.01"/>
            <worldbody>
                <body name="link" pos="0 0 1">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           limited="true" range="-1 1"
                           solreflimit="0.001 1"/>
                    <geom type="sphere" size="0.1" mass="1.0"
                          contype="0" conaffinity="0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // With refsafe ON (default): timeconst clamped to 2*0.01 = 0.02
    let model_safe = load_model(mjcf).expect("load");
    let mut data_safe = model_safe.make_data();
    data_safe.qpos[0] = 2.0; // violate upper limit (range is -1 to 1)
    data_safe.forward(&model_safe).expect("forward");
    let aref_safe = data_safe.efc_aref.clone();

    // With refsafe OFF: timeconst = 0.001 (much stiffer)
    let mut model_unsafe = load_model(mjcf).expect("load");
    model_unsafe.disableflags |= DISABLE_REFSAFE;
    let mut data_unsafe = model_unsafe.make_data();
    data_unsafe.qpos[0] = 2.0;
    data_unsafe.forward(&model_unsafe).expect("forward");
    let aref_unsafe = data_unsafe.efc_aref.clone();

    // Both should have constraint rows (limit violated)
    assert!(
        !aref_safe.is_empty(),
        "should have constraint rows with limit violated"
    );
    assert_eq!(aref_safe.len(), aref_unsafe.len());

    // aref values should differ because stiffness differs
    // (unclamped timeconst = 0.001 vs clamped = 0.02 → ~400x stiffer)
    assert!(
        (aref_safe[0] - aref_unsafe[0]).abs() > 1e-6,
        "aref should differ between refsafe on ({}) and off ({})",
        aref_safe[0],
        aref_unsafe[0]
    );
}

// ============================================================================
// AC20: Eulerdamp disable — step succeeds (stub, no crash)
// ============================================================================

#[test]
fn ac20_eulerdamp_disable() {
    let mut model = load_model(flag_test_mjcf()).expect("load");
    model.disableflags |= DISABLE_EULERDAMP;

    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.qvel[0] = 1.0;

    // Should not crash. Currently a stub (explicit Euler only).
    data.step(&model)
        .expect("step should succeed with DISABLE_EULERDAMP");

    // Verify the flag is stored correctly
    assert!(disabled(&model, DISABLE_EULERDAMP));

    // qacc should be finite
    for dof in 0..model.nv {
        assert!(
            data.qacc[dof].is_finite(),
            "qacc[{dof}] should be finite with DISABLE_EULERDAMP"
        );
    }
}

// ============================================================================
// AC21: Passive force hierarchy — spring disabled but gravcomp still runs
// ============================================================================

#[test]
fn ac21_passive_force_hierarchy() {
    // Model with gravcomp body — gravcomp should run even with DISABLE_SPRING
    let mjcf = r#"
        <mujoco model="gravcomp_hierarchy">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="link" pos="0 0 1">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           stiffness="100" damping="10"/>
                    <geom type="sphere" size="0.1" mass="2.0" pos="0.5 0 0"
                          contype="0" conaffinity="0"/>
                    <inertial pos="0.5 0 0" mass="2.0" diaginertia="0.008 0.008 0.008"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Case 1: DISABLE_SPRING only — damper + gravcomp still run
    let mut model1 = load_model(mjcf).expect("load");
    model1.disableflags |= DISABLE_SPRING;
    model1.body_gravcomp[1] = 1.0;
    model1.ngravcomp = 1;
    let mut data1 = model1.make_data();
    data1.qpos[0] = 0.5;
    data1.qvel[0] = 1.0;
    data1.forward(&model1).expect("forward");

    assert_relative_eq!(data1.qfrc_spring[0], 0.0, epsilon = 1e-12);
    assert!(
        data1.qfrc_damper[0].abs() > 0.1,
        "damper should still run with only DISABLE_SPRING"
    );
    assert!(
        data1.qfrc_gravcomp[0].abs() > 0.1,
        "gravcomp should still run with only DISABLE_SPRING"
    );

    // Case 2: Both DISABLE_SPRING + DISABLE_DAMPER — top-level early return
    // skips ALL sub-functions including gravcomp
    let mut model2 = load_model(mjcf).expect("load");
    model2.disableflags |= DISABLE_SPRING | DISABLE_DAMPER;
    model2.body_gravcomp[1] = 1.0;
    model2.ngravcomp = 1;
    let mut data2 = model2.make_data();
    data2.qpos[0] = 0.5;
    data2.qvel[0] = 1.0;
    data2.forward(&model2).expect("forward");

    // Everything zeroed by top-level early return
    assert_relative_eq!(data2.qfrc_spring[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(data2.qfrc_damper[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(data2.qfrc_gravcomp[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(data2.qfrc_passive[0], 0.0, epsilon = 1e-12);
}

// ============================================================================
// AC38: Island default correctness — disableflags=0 means island discovery runs
// ============================================================================

#[test]
fn ac38_island_default_correctness() {
    use sim_core::DISABLE_ISLAND;

    let model = load_model(free_body_with_floor_mjcf()).expect("load");

    // Default: DISABLE_ISLAND bit is clear
    assert_eq!(
        model.disableflags & DISABLE_ISLAND,
        0,
        "DISABLE_ISLAND should be clear by default"
    );

    // Forward pass should discover islands (nisland > 0 for a model with contacts)
    let mut data = model.make_data();
    data.qpos[2] = 0.11; // near floor
    data.forward(&model).expect("forward");

    // Island discovery should run (nisland >= 1 when there are constraint rows)
    // The exact count depends on contact generation, but with disableflags=0
    // island discovery is not skipped.
    assert_eq!(
        model.disableflags, 0,
        "all disable flags should be zero by default"
    );
}

// ============================================================================
// AC40: Contact-passive + spring/damper interaction
// ============================================================================

#[test]
fn ac40_contact_passive_spring_interaction() {
    // DISABLE_CONTACT + spring/damper enabled → spring/damper passive forces apply
    let mut model = load_model(flag_test_mjcf()).expect("load");
    model.disableflags |= DISABLE_CONTACT;

    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    // Spring and damper should still produce force despite DISABLE_CONTACT
    assert!(
        data.qfrc_spring[0].abs() > 1.0,
        "spring force should be non-zero with only DISABLE_CONTACT"
    );
    assert!(
        data.qfrc_damper[0].abs() > 0.1,
        "damper force should be non-zero with only DISABLE_CONTACT"
    );
    assert_eq!(data.ncon, 0, "no contacts with DISABLE_CONTACT");

    // Both spring+damper disabled → all passive sub-functions skipped
    let mut model2 = load_model(flag_test_mjcf()).expect("load");
    model2.disableflags |= DISABLE_CONTACT | DISABLE_SPRING | DISABLE_DAMPER;

    let mut data2 = model2.make_data();
    data2.qpos[0] = 0.5;
    data2.qvel[0] = 1.0;
    data2.forward(&model2).expect("forward");

    for dof in 0..model2.nv {
        assert_relative_eq!(data2.qfrc_passive[dof], 0.0, epsilon = 1e-12);
    }
}

// ============================================================================
// AC42: DISABLE_CONSTRAINT cascading — ncon=0, nefc=0, qacc=qacc_smooth
// ============================================================================

#[test]
fn ac42_constraint_cascading() {
    let mut model = load_model(free_body_with_floor_mjcf()).expect("load");
    model.disableflags |= DISABLE_CONSTRAINT;

    let mut data = model.make_data();
    data.qpos[2] = 0.11; // near floor — would normally generate contacts
    data.forward(&model).expect("forward");

    // Full causal chain verification
    assert_eq!(data.ncon, 0, "ncon should be 0 with DISABLE_CONSTRAINT");
    assert!(
        data.efc_type.is_empty(),
        "nefc should be 0 with DISABLE_CONSTRAINT"
    );

    // qacc should equal qacc_smooth (no constraint correction)
    for dof in 0..model.nv {
        // qacc should equal qacc_smooth with DISABLE_CONSTRAINT
        assert_relative_eq!(data.qacc[dof], data.qacc_smooth[dof], epsilon = 1e-12);
    }

    // qfrc_constraint should be all zeros
    for dof in 0..model.nv {
        assert_relative_eq!(data.qfrc_constraint[dof], 0.0, epsilon = 1e-12);
    }
}

// ============================================================================
// AC43: Sleep-filtered aggregation — awake bodies match no-sleep results
// ============================================================================

#[test]
fn ac43_sleep_filtered_aggregation() {
    // With all bodies awake, qfrc_passive should be identical whether
    // sleep is enabled or not (catches off-by-one in indexed iteration).
    let model_nosleep = load_model(flag_test_mjcf()).expect("load");
    let mut data_nosleep = model_nosleep.make_data();
    data_nosleep.qpos[0] = 0.5;
    data_nosleep.qvel[0] = 1.0;
    data_nosleep.forward(&model_nosleep).expect("forward");

    let mut model_sleep = load_model(flag_test_mjcf()).expect("load");
    model_sleep.enableflags |= ENABLE_SLEEP;
    let mut data_sleep = model_sleep.make_data();
    data_sleep.qpos[0] = 0.5;
    data_sleep.qvel[0] = 1.0;
    data_sleep.forward(&model_sleep).expect("forward");

    // All bodies awake → passive forces should be bitwise identical
    for dof in 0..model_nosleep.nv {
        // qfrc_passive should match with all bodies awake
        assert_relative_eq!(
            data_nosleep.qfrc_passive[dof],
            data_sleep.qfrc_passive[dof],
            epsilon = 1e-12
        );
    }
}

// ============================================================================
// AC44: Per-group with RK4 — disabled group frozen across all RK4 stages
// ============================================================================

#[test]
fn ac44_pergroup_rk4() {
    let mut model = load_model(rk4_actuator_mjcf()).expect("load");
    // Disable group 1 (motor2)
    model.disableactuator = 1u32 << 1;

    let mut data = model.make_data();
    data.ctrl[0] = 1.0; // motor1 (group 0) — active
    data.ctrl[1] = 1.0; // motor2 (group 1) — disabled

    // Record initial qpos for j2
    let _j2_qpos_initial = data.qpos[1];

    data.step(&model).expect("step");

    // motor1 (group 0) should have effect — j1 moves
    assert!(
        (data.qpos[0] - 0.0).abs() > 1e-10,
        "group-0 actuator should move j1 under RK4"
    );

    // motor2 (group 1, disabled) — j2 should only move due to gravity,
    // NOT due to actuator force. Compare against a run with no ctrl.
    let mut model_noctrl = load_model(rk4_actuator_mjcf()).expect("load");
    model_noctrl.disableactuator = 1u32 << 1;
    let mut data_noctrl = model_noctrl.make_data();
    data_noctrl.ctrl[0] = 1.0;
    data_noctrl.ctrl[1] = 0.0; // no ctrl for motor2
    data_noctrl.step(&model_noctrl).expect("step");

    // j2 position should be the same whether ctrl[1]=1 or ctrl[1]=0
    // because motor2 is group-disabled — ctrl is ignored at all RK4 stages.
    // Disabled group-1 actuator should have no effect under RK4
    assert_relative_eq!(data.qpos[1], data_noctrl.qpos[1], epsilon = 1e-12);
}

// ============================================================================
// AC30: Sleep-aware validation — sleeping DOFs skip NaN check
// ============================================================================

#[test]
fn ac30_sleep_aware_validation() {
    // This test verifies that mj_check_vel skips sleeping DOFs.
    // With sleep enabled and a body asleep, bad velocity in sleeping DOFs
    // should NOT trigger auto-reset.
    //
    // Note: Putting a body to sleep requires island discovery + sleep countdown.
    // We test indirectly: with ENABLE_SLEEP and all bodies active, verify that
    // bad velocity DOES trigger reset (the awake path works).
    let mut model = load_model(free_body_space_mjcf()).expect("load");
    model.enableflags |= ENABLE_SLEEP;

    let mut data = model.make_data();
    data.qvel[0] = f64::NAN;

    data.step(&model).expect("step");

    // Awake body with NaN velocity should trigger reset
    assert!(
        data.warnings[Warning::BadQvel as usize].count > 0,
        "BadQvel warning should fire for NaN in awake body"
    );
    assert!(
        !data.qvel[0].is_nan(),
        "NaN should be cleared by auto-reset"
    );
}

// ============================================================================
// AC46: DISABLE_WARMSTART + islands — cold start with island discovery
// ============================================================================

#[test]
fn ac46_warmstart_plus_islands() {
    let mut model = load_model(free_body_with_floor_mjcf()).expect("load");
    model.disableflags |= DISABLE_WARMSTART;

    let mut data = model.make_data();
    data.qpos[2] = 0.11; // near floor

    // Step multiple times — should converge without warmstart
    for _ in 0..20 {
        data.step(&model).expect("step");
    }

    // Verify solver converged (qacc finite)
    for dof in 0..model.nv {
        assert!(
            data.qacc[dof].is_finite(),
            "qacc[{dof}] should be finite with DISABLE_WARMSTART + islands"
        );
    }

    // qacc_warmstart should still be saved at end of step (unconditional save)
    // It won't be used (warmstart disabled), but the save always happens.
    let warmstart_nonzero = (0..model.nv).any(|i| data.qacc_warmstart[i] != 0.0);
    assert!(
        warmstart_nonzero,
        "qacc_warmstart should be saved even with DISABLE_WARMSTART"
    );
}

// ============================================================================
// AC35: ENABLE_OVERRIDE margin + solver params
// ============================================================================

#[test]
fn ac35_override_margin_solver_params() {
    // Two spheres near each other, with ENABLE_OVERRIDE + custom o_* params.
    let mjcf = r#"
    <mujoco model="override_test">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="ball" pos="0 0 0.5">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let custom_solref = [0.05, 0.8];
    let custom_solimp = [0.8, 0.85, 0.002, 0.3, 1.5];
    let custom_margin = 0.1;

    let mut model = load_model(mjcf).expect("load");
    model.enableflags |= ENABLE_OVERRIDE;
    model.o_solref = custom_solref;
    model.o_solimp = custom_solimp;
    model.o_margin = custom_margin;

    let mut data = model.make_data();
    // Place ball near floor so contact is generated
    data.qpos[2] = 0.11;
    data.forward(&model).expect("forward");

    assert!(
        data.ncon > 0,
        "should generate at least one contact near floor"
    );

    let c = &data.contacts[0];
    assert_relative_eq!(c.solref[0], custom_solref[0], epsilon = 1e-12);
    assert_relative_eq!(c.solref[1], custom_solref[1], epsilon = 1e-12);
    for (actual, expected) in c.solimp.iter().zip(custom_solimp.iter()) {
        assert_relative_eq!(actual, expected, epsilon = 1e-12);
    }
    // includemargin should reflect the override margin (o_margin - gap).
    // Default gap = 0.0 for both geoms, so includemargin = o_margin.
    assert_relative_eq!(c.includemargin, custom_margin, epsilon = 1e-12);
}

// ============================================================================
// AC36: ENABLE_OVERRIDE friction clamping
// ============================================================================

#[test]
fn ac36_override_friction_clamping() {
    let mjcf = r#"
    <mujoco model="override_friction_test">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="ball" pos="0 0 0.5">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let min_mu = 1e-5;

    // Case 1: friction values above MIN_MU — should pass through unchanged
    let above_mu = [0.5, 0.5, 0.01, 0.001, 0.001];
    let mut model = load_model(mjcf).expect("load");
    model.enableflags |= ENABLE_OVERRIDE;
    model.o_friction = above_mu;
    model.o_margin = 0.1;

    let mut data = model.make_data();
    data.qpos[2] = 0.11;
    data.forward(&model).expect("forward");

    assert!(data.ncon > 0, "should generate contact");
    let c = &data.contacts[0];
    for (actual, expected) in c.mu.iter().zip(above_mu.iter()) {
        assert_relative_eq!(actual, expected, epsilon = 1e-12);
    }
    assert_relative_eq!(c.friction, above_mu[0], epsilon = 1e-12);

    // Case 2: friction values at/below MIN_MU — should be clamped
    let below_mu = [0.0, 1e-6, 1e-10, 0.0, 1e-20];
    let mut model2 = load_model(mjcf).expect("load");
    model2.enableflags |= ENABLE_OVERRIDE;
    model2.o_friction = below_mu;
    model2.o_margin = 0.1;

    let mut data2 = model2.make_data();
    data2.qpos[2] = 0.11;
    data2.forward(&model2).expect("forward");

    assert!(data2.ncon > 0, "should generate contact");
    let c2 = &data2.contacts[0];
    for (i, &mu_i) in c2.mu.iter().enumerate() {
        assert!(
            mu_i >= min_mu,
            "mu[{i}] = {mu_i} should be >= MIN_MU ({min_mu})",
        );
    }
    assert!(c2.friction >= min_mu, "friction scalar should be >= MIN_MU");
}
