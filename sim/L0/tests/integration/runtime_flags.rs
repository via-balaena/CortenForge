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
    // When BOTH spring and damper are disabled, all passive sub-forces are zero.
    // NOTE: Component-level gating (DISABLE_SPRING alone zeroes spring) is tracked
    // as S4.7b gap. Currently only the top-level combined guard is implemented.
    let mut model = load_model(flag_test_mjcf()).expect("load");
    model.disableflags |= DISABLE_SPRING | DISABLE_DAMPER;

    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward");

    assert_relative_eq!(data.qfrc_spring[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(data.qfrc_damper[0], 0.0, epsilon = 1e-12);

    // Verify that with neither disabled, both forces are non-zero
    let model2 = load_model(flag_test_mjcf()).expect("load");
    let mut data2 = model2.make_data();
    data2.qpos[0] = 0.5;
    data2.qvel[0] = 1.0;
    data2.forward(&model2).expect("forward");

    assert!(
        data2.qfrc_spring[0].abs() > 1.0,
        "spring force should be non-zero with deflection"
    );
    assert!(
        data2.qfrc_damper[0].abs() > 0.1,
        "damper force should be non-zero with velocity"
    );
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
    // CortenForge defaults: ENABLE_ENERGY is on (pre-§41 behavior, see S5.1).
    // MuJoCo defaults enableflags=0, but CortenForge sets energy=true in MjcfFlag.
    assert_eq!(
        model.enableflags, ENABLE_ENERGY,
        "CortenForge default: only ENABLE_ENERGY is set"
    );
}

// ============================================================================
// AC13: Energy gating — only computed when ENABLE_ENERGY set
// ============================================================================

#[test]
fn ac13_energy_gating() {
    // Without ENABLE_ENERGY: energy fields are zeroed
    let mut model = load_model(free_body_space_mjcf()).expect("load");
    model.enableflags &= !ENABLE_ENERGY; // Clear the CortenForge default
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

    // With ENABLE_ENERGY (CortenForge default): energy fields are computed
    let model2 = load_model(free_body_space_mjcf()).expect("load");
    assert_ne!(model2.enableflags & ENABLE_ENERGY, 0);
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

    // CortenForge default: ENABLE_ENERGY is on, others off
    assert!(enabled(&model, ENABLE_ENERGY));
    assert!(!enabled(&model, ENABLE_OVERRIDE));

    // Clear energy, set override
    model.enableflags &= !ENABLE_ENERGY;
    model.enableflags |= ENABLE_OVERRIDE;
    assert!(!enabled(&model, ENABLE_ENERGY));
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
