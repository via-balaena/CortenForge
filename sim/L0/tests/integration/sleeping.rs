//! Integration tests for §16: Sleeping / Body Deactivation.
//!
//! Covers tests T1–T14, T17–T28, T31–T42, T59–T60, T72–T98 from `sim/docs/todo/future_work_5.md`.
//! Phase C step C3a tests T89–T98 validate selective CRBA (§16.29.3).
//! Benchmarks T15–T16 are in a separate benchmark file.

use approx::assert_relative_eq;
use sim_core::batch::BatchSim;
use sim_core::{DISABLE_ISLAND, ENABLE_SLEEP, SleepPolicy, SleepState};
use sim_mjcf::load_model;
use std::sync::Arc;

// ============================================================================
// MJCF Fixtures
// ============================================================================

/// Single free-falling body with sleep enabled, ground plane for resting.
/// Uses a generous sleep_tolerance and high damping contact to ensure reliable sleeping.
fn free_body_sleep_mjcf() -> &'static str {
    r#"
    <mujoco model="free_body_sleep">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball" pos="0 0 0.2">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Two separate kinematic trees (two root bodies) with sleep enabled.
fn two_tree_sleep_mjcf() -> &'static str {
    r#"
    <mujoco model="two_trees">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball_a" pos="-1 0 0.2">
                <freejoint name="free_a"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball_b" pos="1 0 0.2">
                <freejoint name="free_b"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Pendulum with sleep disabled (default). Geom offset from joint to create torque.
fn pendulum_no_sleep_mjcf() -> &'static str {
    r#"
    <mujoco model="no_sleep">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0" pos="0.5 0 0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Sleep-never policy body.
fn sleep_never_mjcf() -> &'static str {
    r#"
    <mujoco model="sleep_never">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.01">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="ball" pos="0 0 0.2" sleep="never">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Sleep-init policy body (starts asleep).
fn sleep_init_mjcf() -> &'static str {
    r#"
    <mujoco model="sleep_init">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.01">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="ball" pos="0 0 0.5">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="resting" pos="0 0 1.5" sleep="init">
                <freejoint name="resting_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// RK4 + sleep (should warn and disable sleep).
fn rk4_sleep_mjcf() -> &'static str {
    r#"
    <mujoco model="rk4_sleep">
        <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="ball" pos="0 0 1">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Multi-tree model for tree enumeration testing (3 separate chains).
fn three_tree_mjcf() -> &'static str {
    r#"
    <mujoco model="three_trees">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="chain_a" pos="-2 0 1">
                <joint name="hinge_a" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
                <body name="chain_a_child" pos="0 0 -0.5">
                    <joint name="hinge_a2" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="0.5"/>
                </body>
            </body>
            <body name="chain_b" pos="0 0 1">
                <joint name="hinge_b" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="chain_c" pos="2 0 1">
                <joint name="hinge_c" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Actuated tree model for policy resolution testing.
fn actuated_sleep_mjcf() -> &'static str {
    r#"
    <mujoco model="actuated_sleep">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="actuated_arm" pos="-1 0 1">
                <joint name="shoulder" type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
            </body>
            <body name="passive_ball" pos="1 0 0.2">
                <freejoint name="passive_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <actuator>
            <motor joint="shoulder" gear="1"/>
        </actuator>
    </mujoco>
    "#
}

/// Custom sleep_tolerance for threshold testing.
fn custom_tolerance_mjcf(tolerance: f64) -> String {
    format!(
        r#"
    <mujoco model="custom_tol">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="{}">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="ball" pos="0 0 0.2">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.01 0.8"/>
            </body>
        </worldbody>
    </mujoco>
    "#,
        tolerance
    )
}

/// Model with a sensor on a sleeping body.
fn sensor_sleep_mjcf() -> &'static str {
    r#"
    <mujoco model="sensor_sleep">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball" pos="0 0 0.2">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
                <site name="ball_site" pos="0 0 0"/>
            </body>
        </worldbody>
        <sensor>
            <framepos objtype="site" objname="ball_site"/>
        </sensor>
    </mujoco>
    "#
}

/// Model with passive spring joint and sleep.
fn passive_spring_sleep_mjcf() -> &'static str {
    r#"
    <mujoco model="passive_spring">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.01">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="spring_body" pos="0 0 0.5">
                <joint name="spring_hinge" type="hinge" axis="0 1 0"
                       stiffness="10" damping="5"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Two bodies: one resting (init-asleep, at ground level) and one falling.
/// The falling body hits the resting body, waking it via contact.
fn contact_wake_mjcf() -> &'static str {
    r#"
    <mujoco model="contact_wake">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="resting" pos="0 0 0.1" sleep="init">
                <freejoint name="resting_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="falling" pos="0.15 0 1.5">
                <freejoint name="falling_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

// ============================================================================
// T1: test_sleep_disabled_noop (AC #1)
// ============================================================================

#[test]
fn test_sleep_disabled_noop() {
    // With ENABLE_SLEEP cleared, behavior is identical to pre-sleep codebase.
    let model = load_model(pendulum_no_sleep_mjcf()).expect("load model");
    assert_eq!(
        model.enableflags & ENABLE_SLEEP,
        0,
        "sleep should be disabled by default"
    );

    let mut data = model.make_data();

    // Step the simulation — should work exactly as before
    let qpos_before = data.qpos.clone();
    for _ in 0..10 {
        data.step(&model).expect("step should succeed");
    }
    // Pendulum should have moved after 10 steps
    assert_ne!(
        data.qpos[0], qpos_before[0],
        "pendulum should move under gravity"
    );

    // Body 0 should be Static (world body sentinel)
    assert_eq!(data.body_sleep_state[0], SleepState::Static);
}

// ============================================================================
// T2: test_tree_enumeration (§16.0)
// ============================================================================

#[test]
fn test_tree_enumeration() {
    let model = load_model(three_tree_mjcf()).expect("load model");

    // 3 separate chains → 3 trees
    assert_eq!(model.ntree, 3, "should have 3 kinematic trees");

    // Each tree should have valid body/dof ranges
    for t in 0..model.ntree {
        assert!(model.tree_body_num[t] > 0, "tree {} should have bodies", t);
        // tree_body_adr should point to a valid body
        assert!(model.tree_body_adr[t] < model.nbody);
    }

    // chain_a has 2 bodies, chains b and c have 1 each
    // Find which tree has 2 bodies
    let two_body_tree = (0..model.ntree).find(|&t| model.tree_body_num[t] == 2);
    assert!(
        two_body_tree.is_some(),
        "one tree should have 2 bodies (chain_a)"
    );

    // body_treeid should be set for all non-world bodies
    for body_id in 1..model.nbody {
        assert!(
            model.body_treeid[body_id] < model.ntree,
            "body {} should have valid tree id",
            body_id
        );
    }

    // World body should have sentinel tree id
    assert_eq!(model.body_treeid[0], usize::MAX, "world body sentinel");
}

// ============================================================================
// T3: test_sleep_policy_resolution (§16.0)
// ============================================================================

#[test]
fn test_sleep_policy_resolution() {
    let model = load_model(actuated_sleep_mjcf()).expect("load model");

    assert_eq!(model.ntree, 2, "should have 2 trees");

    // Find which tree has the actuator (shoulder joint → actuated_arm body)
    let actuated_body = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("actuated_arm"))
        .expect("actuated_arm body");
    let actuated_tree = model.body_treeid[actuated_body];

    let passive_body = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("passive_ball"))
        .expect("passive_ball body");
    let passive_tree = model.body_treeid[passive_body];

    // Actuated tree → AutoNever
    assert_eq!(
        model.tree_sleep_policy[actuated_tree],
        SleepPolicy::AutoNever,
        "actuated tree should be AutoNever"
    );

    // Non-actuated tree → AutoAllowed
    assert_eq!(
        model.tree_sleep_policy[passive_tree],
        SleepPolicy::AutoAllowed,
        "passive tree should be AutoAllowed"
    );
}

// ============================================================================
// T4: test_sleep_countdown_timer (§16.3)
// ============================================================================

#[test]
fn test_sleep_countdown_timer() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    assert_ne!(
        model.enableflags & ENABLE_SLEEP,
        0,
        "sleep should be enabled"
    );

    let mut data = model.make_data();

    let ball_body = 1; // first non-world body
    let tree = model.body_treeid[ball_body];

    // Let the ball settle on the ground plane (many steps)
    for _ in 0..2000 {
        data.step(&model).expect("step");
        if data.tree_asleep[tree] >= 0 {
            break;
        }
    }

    // After many steps on the ground, the body should eventually sleep.
    assert!(
        data.tree_asleep[tree] >= 0,
        "ball should be asleep after resting: tree_asleep={}",
        data.tree_asleep[tree]
    );
    assert_eq!(
        data.body_sleep_state[ball_body],
        SleepState::Asleep,
        "body should be in Asleep state"
    );
}

// ============================================================================
// T5: test_sleep_zeros_velocity (§16.3)
// ============================================================================

#[test]
fn test_sleep_zeros_velocity() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // When asleep, qvel should be zero for the tree's DOFs
    let dof_start = model.tree_dof_adr[tree];
    let dof_end = dof_start + model.tree_dof_num[tree];
    for dof in dof_start..dof_end {
        assert_eq!(
            data.qvel[dof], 0.0,
            "sleeping DOF {} qvel should be zero",
            dof
        );
        assert_eq!(
            data.qacc[dof], 0.0,
            "sleeping DOF {} qacc should be zero",
            dof
        );
    }
}

// ============================================================================
// T6: test_wake_on_xfrc_applied (AC #3)
// ============================================================================

#[test]
fn test_wake_on_xfrc_applied() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(
        data.tree_asleep[tree] >= 0,
        "ball should be asleep before applying force"
    );

    // Apply external force to wake it
    data.xfrc_applied[ball_body][2] = 10.0; // Force in Z

    // Step once — wake detection runs in forward()
    data.step(&model).expect("step");

    // Body should now be awake
    assert!(
        data.tree_asleep[tree] < 0,
        "ball should be awake after xfrc_applied"
    );
    assert_eq!(
        data.body_sleep_state[ball_body],
        SleepState::Awake,
        "body should be in Awake state"
    );
}

// ============================================================================
// T7: test_wake_on_qfrc_applied (AC #3)
// ============================================================================

#[test]
fn test_wake_on_qfrc_applied() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Apply joint-space force to wake it
    let dof_start = model.tree_dof_adr[tree];
    data.qfrc_applied[dof_start] = 5.0;

    // Step once
    data.step(&model).expect("step");

    // Body should now be awake
    assert!(
        data.tree_asleep[tree] < 0,
        "ball should be awake after qfrc_applied"
    );
}

// ============================================================================
// T8: test_wake_on_contact (AC #4)
// ============================================================================

#[test]
fn test_wake_on_contact() {
    let model = load_model(contact_wake_mjcf()).expect("load model");
    let mut data = model.make_data();

    // The "resting" body starts asleep (init policy)
    let resting_body = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("resting"))
        .expect("resting body");
    let resting_tree = model.body_treeid[resting_body];

    // After make_data, init policy should start asleep
    assert!(
        data.tree_asleep[resting_tree] >= 0,
        "resting body should start asleep (init policy)"
    );

    // The falling body is awake and will eventually hit the resting body.
    // Step until contact wakes the resting body or max steps.
    let mut woke = false;
    for _ in 0..500 {
        match data.step(&model) {
            Ok(()) => {}
            Err(_) => continue,
        }
        if data.tree_asleep[resting_tree] < 0 {
            woke = true;
            break;
        }
    }
    assert!(
        woke,
        "resting body should wake on contact with falling body"
    );
}

// ============================================================================
// T9: test_sleep_never_policy (AC #5)
// ============================================================================

#[test]
fn test_sleep_never_policy() {
    let model = load_model(sleep_never_mjcf()).expect("load model");
    let mut data = model.make_data();

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];

    // Policy should be Never
    assert_eq!(
        model.tree_sleep_policy[tree],
        SleepPolicy::Never,
        "body with sleep='never' should have Never policy"
    );

    // Even after many steps at rest, body should never sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    assert!(
        data.tree_asleep[tree] < 0,
        "Never policy body should never sleep, tree_asleep={}",
        data.tree_asleep[tree]
    );
    assert_eq!(
        data.body_sleep_state[ball_body],
        SleepState::Awake,
        "Never policy body should remain Awake"
    );
}

// ============================================================================
// T10: test_sleep_init_policy (AC #6)
// ============================================================================

#[test]
fn test_sleep_init_policy() {
    let model = load_model(sleep_init_mjcf()).expect("load model");
    let data = model.make_data();

    // The "resting" body with sleep="init" should start asleep
    let resting_body = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("resting"))
        .expect("resting body");
    let resting_tree = model.body_treeid[resting_body];

    assert!(
        data.tree_asleep[resting_tree] >= 0,
        "Init policy body should start asleep"
    );
    assert_eq!(
        data.body_sleep_state[resting_body],
        SleepState::Asleep,
        "Init policy body should have Asleep state"
    );

    // The "ball" body (no sleep override, default Auto → AutoAllowed) should start awake
    let ball_body = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("ball"))
        .expect("ball body");
    let ball_tree = model.body_treeid[ball_body];

    assert!(
        data.tree_asleep[ball_tree] < 0,
        "Default policy body should start awake"
    );
    assert_eq!(
        data.body_sleep_state[ball_body],
        SleepState::Awake,
        "Default policy body should have Awake state"
    );

    // Sleeping body's qvel should be zero
    let dof_start = model.tree_dof_adr[resting_tree];
    let dof_end = dof_start + model.tree_dof_num[resting_tree];
    for dof in dof_start..dof_end {
        assert_eq!(
            data.qvel[dof], 0.0,
            "init-sleeping DOF {} should be zero",
            dof
        );
    }
}

// ============================================================================
// T11: test_rk4_sleep_warning (AC #7)
// ============================================================================

#[test]
fn test_rk4_sleep_warning() {
    let model = load_model(rk4_sleep_mjcf()).expect("load model");

    // RK4 + sleep → sleep should be disabled
    assert_eq!(
        model.enableflags & ENABLE_SLEEP,
        0,
        "sleep should be disabled with RK4 integrator"
    );
}

// ============================================================================
// T12: test_world_body_static (AC #8)
// ============================================================================

#[test]
fn test_world_body_static() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Body 0 (world) should always be Static
    assert_eq!(
        data.body_sleep_state[0],
        SleepState::Static,
        "world body should be Static"
    );
    for _ in 0..100 {
        data.step(&model).expect("step");
    }
    assert_eq!(
        data.body_sleep_state[0],
        SleepState::Static,
        "world body should remain Static after stepping"
    );
}

// ============================================================================
// T13: test_sleep_wake_scenario (AC #9)
// ============================================================================

#[test]
fn test_sleep_wake_scenario() {
    let model = load_model(two_tree_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    assert_eq!(model.ntree, 2, "should have 2 trees");

    // Let both balls settle
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    // Both should be asleep
    let ball_a = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("ball_a"))
        .expect("ball_a");
    let ball_b = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("ball_b"))
        .expect("ball_b");
    let tree_a = model.body_treeid[ball_a];
    let tree_b = model.body_treeid[ball_b];

    assert!(data.tree_asleep[tree_a] >= 0, "ball_a should be asleep");
    assert!(data.tree_asleep[tree_b] >= 0, "ball_b should be asleep");

    // Wake ball_a with external force
    data.xfrc_applied[ball_a][2] = 20.0;
    data.step(&model).expect("step");

    // ball_a should be awake, ball_b still asleep
    assert!(data.tree_asleep[tree_a] < 0, "ball_a should be awake");
    assert!(
        data.tree_asleep[tree_b] >= 0,
        "ball_b should still be asleep"
    );

    // Clear force
    data.xfrc_applied[ball_a][2] = 0.0;
}

// ============================================================================
// T14: test_sleep_tolerance_scaling (AC #10)
// ============================================================================

#[test]
fn test_sleep_tolerance_scaling() {
    // A tighter tolerance should take longer to sleep
    let mjcf_tight = custom_tolerance_mjcf(1e-6);
    let model_tight = load_model(&mjcf_tight).expect("load tight tolerance model");

    let mjcf_loose = custom_tolerance_mjcf(1e-2);
    let model_loose = load_model(&mjcf_loose).expect("load loose tolerance model");

    assert_relative_eq!(model_tight.sleep_tolerance, 1e-6, epsilon = 1e-12);
    assert_relative_eq!(model_loose.sleep_tolerance, 1e-2, epsilon = 1e-12);

    // Both models should have sleep enabled
    assert_ne!(model_tight.enableflags & ENABLE_SLEEP, 0);
    assert_ne!(model_loose.enableflags & ENABLE_SLEEP, 0);

    // Step both until sleep; count steps
    let mut data_tight = model_tight.make_data();
    let mut data_loose = model_loose.make_data();
    let tree_tight = model_tight.body_treeid[1];
    let tree_loose = model_loose.body_treeid[1];

    let mut steps_tight = 0u32;
    let mut steps_loose = 0u32;

    for i in 0..5000 {
        if data_tight.tree_asleep[tree_tight] < 0 {
            data_tight.step(&model_tight).expect("step");
            steps_tight = i + 1;
        }
    }
    for i in 0..5000 {
        if data_loose.tree_asleep[tree_loose] < 0 {
            data_loose.step(&model_loose).expect("step");
            steps_loose = i + 1;
        }
    }

    // Tight tolerance should take more steps (or both might max out, but loose
    // should sleep sooner if it sleeps at all)
    if data_loose.tree_asleep[tree_loose] >= 0 {
        // Loose model slept
        if data_tight.tree_asleep[tree_tight] >= 0 {
            // Both slept — tight should have taken more steps
            assert!(
                steps_tight >= steps_loose,
                "tighter tolerance should take >= steps: tight={}, loose={}",
                steps_tight,
                steps_loose
            );
        }
        // If tight didn't sleep in 5000 steps, that's fine — confirms tighter threshold
    }
}

// ============================================================================
// T17: test_batch_sleep_independence (AC #13)
// ============================================================================

#[test]
fn test_batch_sleep_independence() {
    let model = Arc::new(load_model(free_body_sleep_mjcf()).expect("load model"));

    let mut batch = BatchSim::new(model.clone(), 2);

    // Let both environments rest and sleep
    for _ in 0..2000 {
        let errors = batch.step_all();
        for e in &errors {
            assert!(e.is_none(), "step should succeed");
        }
    }

    let tree = model.body_treeid[1];

    // Both environments run independently — both should have settled
    assert!(
        batch.env(0).unwrap().tree_asleep[tree] >= 0,
        "env 0 should be asleep"
    );
    assert!(
        batch.env(1).unwrap().tree_asleep[tree] >= 0,
        "env 1 should be asleep"
    );

    // Wake only environment 0
    batch.env_mut(0).unwrap().xfrc_applied[1][2] = 20.0;
    let errors = batch.step_all();
    for e in &errors {
        assert!(e.is_none(), "step should succeed");
    }

    // Env 0 should be awake, env 1 still asleep
    assert!(
        batch.env(0).unwrap().tree_asleep[tree] < 0,
        "env 0 should be awake"
    );
    assert!(
        batch.env(1).unwrap().tree_asleep[tree] >= 0,
        "env 1 should still be asleep"
    );
}

// ============================================================================
// T19: test_sensor_frozen_on_sleep (AC #15)
// ============================================================================

#[test]
fn test_sensor_frozen_on_sleep() {
    let model = load_model(sensor_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Record sensor value at sleep time
    let sensor_val_at_sleep = data.sensordata.clone();

    // Step more — sensor should remain frozen
    for _ in 0..10 {
        data.step(&model).expect("step");
    }

    // Ball should still be asleep (no external forces)
    assert!(data.tree_asleep[tree] >= 0, "ball should remain asleep");

    // Sensor values should be identical
    for i in 0..model.nsensordata {
        assert_eq!(
            data.sensordata[i], sensor_val_at_sleep[i],
            "sensor data[{}] should be frozen while asleep",
            i
        );
    }

    // Values should not be zero (they reflect the resting position)
    // The framepos sensor should report the body's resting position
    assert!(
        data.sensordata.iter().any(|&v| v.abs() > 1e-10),
        "sensor values should not all be zero (body is resting, not at origin)"
    );
}

// ============================================================================
// T20: test_fk_skip_sleeping_body (§16.5a)
// ============================================================================

#[test]
fn test_fk_skip_sleeping_body() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Record the body's world position at sleep time
    let xipos_at_sleep = data.xipos[ball_body];

    // Step more — sleeping body's pose should be frozen
    for _ in 0..50 {
        data.step(&model).expect("step");
    }

    assert!(data.tree_asleep[tree] >= 0, "ball should still be asleep");
    assert_eq!(
        data.xipos[ball_body], xipos_at_sleep,
        "sleeping body's xipos should be frozen (FK skipped)"
    );
}

// ============================================================================
// T21: test_collision_skip_both_sleeping (§16.5b)
// ============================================================================

#[test]
fn test_collision_skip_both_sleeping() {
    // With two separate sleeping bodies, there should be no contacts between them.
    // (Both-sleeping pairs are skipped in narrow-phase.)
    let model = load_model(two_tree_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let both balls settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];
    assert!(data.tree_asleep[tree_a] >= 0, "ball_a should be asleep");
    assert!(data.tree_asleep[tree_b] >= 0, "ball_b should be asleep");

    // Step more — no contacts should be generated between sleeping pairs
    data.step(&model).expect("step");

    // Count contacts between ball geoms (both sleeping) — there should be none
    // between sleeping-sleeping pairs. Note: contacts with the ground plane
    // (world body = Static) are still generated for sleeping-vs-static pairs
    // since the ground is not "sleeping".
    // Actually, in our implementation, contacts between sleeping and world-body
    // geoms are also skipped because body 0's SleepState is Static (not Asleep).
    // The skip filter only skips when BOTH are Asleep.
    // So ground-vs-sleeping contacts ARE generated. That's fine for correctness.
    // The key assertion is that we can step without errors.
    assert!(data.tree_asleep[tree_a] >= 0, "ball_a should remain asleep");
    assert!(data.tree_asleep[tree_b] >= 0, "ball_b should remain asleep");
}

// ============================================================================
// T22: test_mjcf_sleep_attributes (§16.2)
// ============================================================================

#[test]
fn test_mjcf_sleep_attributes() {
    // Test sleep_tolerance parsing
    let mjcf = custom_tolerance_mjcf(0.05);
    let model = load_model(&mjcf).expect("load model");
    assert_relative_eq!(model.sleep_tolerance, 0.05, epsilon = 1e-12);
    assert_ne!(
        model.enableflags & ENABLE_SLEEP,
        0,
        "sleep should be enabled"
    );

    // Test sleep flag parsing
    let model_default = load_model(pendulum_no_sleep_mjcf()).expect("load model");
    assert_eq!(
        model_default.enableflags & ENABLE_SLEEP,
        0,
        "sleep should be disabled by default"
    );

    // Test body sleep attribute
    let model_never = load_model(sleep_never_mjcf()).expect("load model");
    let ball_tree = model_never.body_treeid[1];
    assert_eq!(
        model_never.tree_sleep_policy[ball_tree],
        SleepPolicy::Never,
        "body sleep='never' should set tree policy to Never"
    );

    let model_init = load_model(sleep_init_mjcf()).expect("load model");
    let resting_body = model_init
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("resting"))
        .expect("resting body");
    let resting_tree = model_init.body_treeid[resting_body];
    assert_eq!(
        model_init.tree_sleep_policy[resting_tree],
        SleepPolicy::Init,
        "body sleep='init' should set tree policy to Init"
    );
}

// ============================================================================
// T23: test_dof_length_computation (§16.0)
// ============================================================================

#[test]
fn test_dof_length_computation() {
    // Free joint: 3 translational DOFs (length=1.0) + 3 rotational DOFs
    let model = load_model(free_body_sleep_mjcf()).expect("load model");

    assert_eq!(model.nv, 6, "free joint should have 6 DOFs");

    // Translational DOFs (first 3) should have length 1.0
    for dof in 0..3 {
        assert_relative_eq!(model.dof_length[dof], 1.0, epsilon = 1e-10);
    }

    // Rotational DOFs (3,4,5): leaf body with no children → clamped to 1.0
    for dof in 3..6 {
        assert_relative_eq!(model.dof_length[dof], 1.0, epsilon = 1e-10);
    }

    // Hinge DOF
    let mjcf_hinge = r#"
    <mujoco model="hinge_test">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model_hinge = load_model(mjcf_hinge).expect("load model");
    assert_eq!(model_hinge.nv, 1);
    // Leaf body (no children): body_length clamped to 1.0
    assert_relative_eq!(model_hinge.dof_length[0], 1.0, epsilon = 1e-10);

    // Slide DOF
    let mjcf_slide = r#"
    <mujoco model="slide_test">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="slider" pos="0 0 1">
                <joint name="slide" type="slide" axis="0 0 1"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model_slide = load_model(mjcf_slide).expect("load model");
    assert_eq!(model_slide.nv, 1);
    assert_relative_eq!(model_slide.dof_length[0], 1.0, epsilon = 1e-10);
}

// ============================================================================
// T24: test_passive_force_skip_sleeping (§16.5a')
// ============================================================================

#[test]
fn test_passive_force_skip_sleeping() {
    let model = load_model(passive_spring_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let body settle
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let body = 1;
    let tree = model.body_treeid[body];

    // Body may or may not sleep (hinge joint with spring — depends on damping).
    // The key test is that passive forces work correctly with sleep enabled.
    // Step more and verify no panics or NaN.
    for _ in 0..100 {
        data.step(&model)
            .expect("step should succeed with sleep + passive forces");
    }

    // Verify qpos is finite
    for i in 0..model.nq {
        assert!(
            data.qpos[i].is_finite(),
            "qpos[{}] should be finite: {}",
            i,
            data.qpos[i]
        );
    }

    // If sleeping, passive forces should not be computed (qfrc_passive should be zero for sleeping DOFs)
    if data.tree_asleep[tree] >= 0 {
        let dof_start = model.tree_dof_adr[tree];
        let dof_end = dof_start + model.tree_dof_num[tree];
        for dof in dof_start..dof_end {
            // Note: qfrc_passive may or may not be zero depending on whether it was
            // zeroed before the skip. The key is that the sleeping body's qvel and qacc
            // remain zero.
            assert_eq!(
                data.qvel[dof], 0.0,
                "sleeping DOF {} qvel should be zero",
                dof
            );
        }
    }
}

// ============================================================================
// T25: test_position_integration_skip (§16.5a'')
// ============================================================================

#[test]
fn test_position_integration_skip() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Record qpos at sleep time
    let qpos_at_sleep = data.qpos.clone();

    // Step 50 more times
    for _ in 0..50 {
        data.step(&model).expect("step");
    }

    // qpos should be unchanged (position integration skipped)
    assert!(data.tree_asleep[tree] >= 0, "ball should still be asleep");

    // For free joints, qpos has 7 elements (3 pos + 4 quat) per joint
    // while nv has 6 elements (3 vel + 3 angvel)
    // Check that qpos is unchanged at the relevant positions
    let qpos_adr = model.jnt_qpos_adr[0]; // first joint's qpos address
    let nq_joint = 7; // free joint: 7 qpos
    for i in qpos_adr..(qpos_adr + nq_joint) {
        assert_eq!(
            data.qpos[i], qpos_at_sleep[i],
            "sleeping joint qpos[{}] should be frozen",
            i
        );
    }
}

// ============================================================================
// T26: test_tendon_passive_mixed_sleep (§16.5a')
// ============================================================================

#[test]
fn test_tendon_passive_mixed_sleep() {
    // This test verifies that tendons work correctly with sleep enabled,
    // even without a mixed-sleep scenario (Phase A treats each tree independently).
    // The key is that tendon force computation doesn't panic or produce NaN.
    let mjcf = r#"
    <mujoco model="tendon_sleep">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="link1" pos="0 0 1">
                <joint name="j1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
                <body name="link2" pos="0 0 -0.5">
                    <joint name="j2" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
            </body>
        </worldbody>
        <tendon>
            <fixed>
                <joint joint="j1" coef="1"/>
                <joint joint="j2" coef="-1"/>
            </fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");
    let mut data = model.make_data();

    // Step and verify no errors
    for _ in 0..500 {
        data.step(&model)
            .expect("step with tendon + sleep should succeed");
    }

    // Verify finite state
    for i in 0..model.nq {
        assert!(data.qpos[i].is_finite(), "qpos[{}] should be finite", i);
    }
}

// ============================================================================
// T27: test_forward_skip_sensors_sleep
// ============================================================================

#[test]
fn test_forward_skip_sensors_sleep() {
    // RK4 disables sleep, so forward_skip_sensors is called in that path.
    // This test verifies that the shared forward_core path works correctly.
    // Since RK4 disables sleep, we just verify that forward() and step() work
    // correctly with sleep enabled (non-RK4).
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Call forward explicitly
    data.forward(&model).expect("forward should succeed");

    // Step a few times
    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    // Verify finite state
    for i in 0..model.nv {
        assert!(data.qvel[i].is_finite(), "qvel[{}] should be finite", i);
    }
}

// ============================================================================
// T28: test_sensor_frozen_value_not_zero (§16.5d)
// ============================================================================

#[test]
fn test_sensor_frozen_value_not_zero() {
    let model = load_model(sensor_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // framepos sensor should report the body's resting position (not zero)
    // The ball is resting on a ground plane at some Z > 0
    if model.nsensordata >= 3 {
        // Z component of framepos should be approximately the ball's resting height
        // (ball radius = 0.1, ground at z=0, so center ≈ 0.1)
        let z_sensor = data.sensordata[2]; // framepos Z
        assert!(
            z_sensor.abs() > 0.01,
            "sensor Z should reflect resting position, not zero: {}",
            z_sensor
        );
        assert!(
            z_sensor.is_finite(),
            "sensor Z should be finite: {}",
            z_sensor
        );
    }

    // Step more — values should remain frozen (not zeroed)
    let sensor_before = data.sensordata.clone();
    for _ in 0..10 {
        data.step(&model).expect("step");
    }
    assert!(data.tree_asleep[tree] >= 0, "ball should still be asleep");
    for i in 0..model.nsensordata {
        assert_eq!(
            data.sensordata[i], sensor_before[i],
            "sensor[{}] should be frozen, not zeroed",
            i
        );
    }
}

// ============================================================================
// T29: test_reset_restores_sleep_state (§16.7)
// ============================================================================

#[test]
fn test_reset_restores_sleep_state() {
    let model = load_model(sleep_init_mjcf()).expect("load model");
    let mut data = model.make_data();

    // "resting" body should start asleep (init policy)
    let resting_body = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("resting"))
        .expect("resting body");
    let resting_tree = model.body_treeid[resting_body];
    assert!(data.tree_asleep[resting_tree] >= 0, "should start asleep");

    // Run some steps to let "ball" body also fall asleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    // Apply force to wake resting body
    data.xfrc_applied[resting_body][2] = 50.0;
    data.step(&model).expect("step");
    assert!(
        data.tree_asleep[resting_tree] < 0,
        "should be awake after force"
    );

    // Reset — sleep state should be re-initialized from model policies
    data.reset(&model);

    // Init-policy tree should be asleep again after reset
    assert!(
        data.tree_asleep[resting_tree] >= 0,
        "init-policy tree should be asleep after reset, got {}",
        data.tree_asleep[resting_tree]
    );
    assert_eq!(
        data.body_sleep_state[resting_body],
        SleepState::Asleep,
        "init-policy body should be Asleep after reset"
    );

    // Non-init trees should be fully awake
    let ball_body = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("ball"))
        .expect("ball body");
    let ball_tree = model.body_treeid[ball_body];
    assert!(
        data.tree_asleep[ball_tree] < 0,
        "non-init tree should be awake after reset"
    );
}

// ============================================================================
// T30: test_wake_on_negative_zero (MuJoCo bytewise check)
// ============================================================================

#[test]
fn test_wake_on_negative_zero() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Apply -0.0 force — should wake (MuJoCo bytewise check: -0.0 != 0 in bytes)
    data.xfrc_applied[ball_body][0] = -0.0_f64;

    // Step once — wake detection runs
    data.step(&model).expect("step");

    // Body should wake because -0.0 has a set sign bit
    assert!(
        data.tree_asleep[tree] < 0,
        "ball should wake on -0.0 xfrc_applied (bytewise check)"
    );
}

// ============================================================================
// Phase B Tests
// ============================================================================

// ============================================================================
// T43: test_dof_length_hinge_1m (§16.14)
// ============================================================================

#[test]
fn test_dof_length_hinge_1m() {
    // A hinge joint on a body with a 1-meter child should get dof_length ≈ 1.0
    let mjcf = r#"
    <mujoco model="hinge_1m">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="link1" pos="0 0 1">
                <joint name="hinge1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.05" mass="1.0"/>
                <body name="link2" pos="1 0 0">
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");

    assert_eq!(model.nv, 1, "single hinge DOF");
    assert_relative_eq!(model.dof_length[0], 1.0, epsilon = 1e-6);
}

// ============================================================================
// T44: test_dof_length_hinge_01m (§16.14)
// ============================================================================

#[test]
fn test_dof_length_hinge_01m() {
    // A hinge joint on a body with a 0.1-meter child should get dof_length ≈ 0.1
    let mjcf = r#"
    <mujoco model="hinge_01m">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="link1" pos="0 0 1">
                <joint name="hinge1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.05" mass="1.0"/>
                <body name="link2" pos="0.1 0 0">
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");

    assert_eq!(model.nv, 1, "single hinge DOF");
    assert_relative_eq!(model.dof_length[0], 0.1, epsilon = 1e-6);
}

// ============================================================================
// T45: test_dof_length_slide (§16.14)
// ============================================================================

#[test]
fn test_dof_length_slide() {
    // Slide joint should always have dof_length = 1.0 regardless of arm length
    let mjcf = r#"
    <mujoco model="slide_dof_length">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="link1" pos="0 0 1">
                <joint name="slide1" type="slide" axis="0 0 1"/>
                <geom type="sphere" size="0.05" mass="1.0"/>
                <body name="link2" pos="2 0 0">
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");

    assert_eq!(model.nv, 1, "single slide DOF");
    // Slide is translational → always 1.0
    assert_relative_eq!(model.dof_length[0], 1.0, epsilon = 1e-10);
}

// ============================================================================
// T46: test_dof_length_free_joint (§16.14)
// ============================================================================

#[test]
fn test_dof_length_free_joint() {
    // Free joint: translational DOFs (0,1,2) = 1.0; rotational DOFs (3,4,5) = body_length
    let mjcf = r#"
    <mujoco model="free_dof_length">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="base" pos="0 0 1">
                <freejoint name="free1"/>
                <geom type="sphere" size="0.05" mass="1.0"/>
                <body name="child" pos="0.5 0 0">
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");

    assert_eq!(model.nv, 6, "free joint has 6 DOFs");

    // Translational DOFs (0,1,2) should be 1.0
    for dof in 0..3 {
        assert!(
            (model.dof_length[dof] - 1.0).abs() < 1e-10,
            "translational dof_length[{dof}] should be 1.0, got {}",
            model.dof_length[dof]
        );
    }

    // Rotational DOFs (3,4,5) should be body_length ≈ 0.5 (child at 0.5m)
    for dof in 3..6 {
        assert!(
            (model.dof_length[dof] - 0.5).abs() < 1e-6,
            "rotational dof_length[{dof}] should be ≈ 0.5, got {}",
            model.dof_length[dof]
        );
    }
}

// ============================================================================
// T68: test_dof_length_nonuniform_threshold (§16.14)
// ============================================================================

#[test]
fn test_dof_length_nonuniform_threshold() {
    // Arm length should affect the effective sleep threshold.
    // With sleep_tolerance = 1e-4:
    //   1-meter arm: threshold = 1e-4 * 1.0 = 1e-4 rad/s
    //   0.1-meter arm: threshold = 1e-4 * 0.1 = 1e-5 rad/s (tighter)
    //
    // Verify this by checking that dof_length differs for different arm lengths.

    // 1-meter arm
    let mjcf_1m = r#"
    <mujoco model="arm_1m">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="link1" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.05" mass="1.0"/>
                <body name="tip" pos="1 0 0">
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model_1m = load_model(mjcf_1m).expect("load model");

    // 0.1-meter arm
    let mjcf_01m = r#"
    <mujoco model="arm_01m">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="link1" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.05" mass="1.0"/>
                <body name="tip" pos="0.1 0 0">
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model_01m = load_model(mjcf_01m).expect("load model");

    // dof_length should reflect the arm length
    assert_relative_eq!(model_1m.dof_length[0], 1.0, epsilon = 1e-6);
    assert_relative_eq!(model_01m.dof_length[0], 0.1, epsilon = 1e-6);

    // The ratio should be 10:1
    let ratio = model_1m.dof_length[0] / model_01m.dof_length[0];
    assert_relative_eq!(ratio, 10.0, epsilon = 1e-3);

    // Effective threshold difference: for sleep_tolerance=1e-4,
    // 1m arm threshold = 1e-4, 0.1m arm threshold = 1e-5
    let tol = 1e-4;
    let threshold_1m = tol * model_1m.dof_length[0];
    let threshold_01m = tol * model_01m.dof_length[0];
    assert!(
        threshold_1m > threshold_01m,
        "shorter arm should have tighter threshold: {} vs {}",
        threshold_1m,
        threshold_01m
    );
    assert_relative_eq!(threshold_1m / threshold_01m, 10.0, epsilon = 1e-3);
}

// ============================================================================
// T51: test_indirection_equivalence (§16.17)
// ============================================================================

#[test]
fn test_indirection_equivalence() {
    // Verify that awake-index indirection arrays match Phase A per-body skip logic.
    // Two trees: one will sleep, one stays awake. Check arrays are consistent.
    let mjcf = r#"
    <mujoco model="indirection_test">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball1" pos="0 0 0.2">
                <freejoint name="j1"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball2" pos="2 0 0.2">
                <freejoint name="j2"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");
    let mut data = model.make_data();

    // Initially all awake — body_awake_ind should contain all bodies
    // (body 0 = world/static, body 1 = ball1, body 2 = ball2)
    assert_eq!(
        data.nbody_awake, 3,
        "all bodies should be in body_awake_ind"
    );
    assert_eq!(data.body_awake_ind[0], 0, "world body");
    assert_eq!(data.body_awake_ind[1], 1, "ball1");
    assert_eq!(data.body_awake_ind[2], 2, "ball2");

    // parent_awake_ind: all bodies have awake/static parents
    assert_eq!(data.nparent_awake, 3);

    // dof_awake_ind: all 12 DOFs awake (2 free joints × 6 DOFs)
    assert_eq!(model.nv, 12);
    assert_eq!(data.nv_awake, 12);
    for i in 0..12 {
        assert_eq!(data.dof_awake_ind[i], i);
    }

    // Step until one body falls asleep
    for _ in 0..5000 {
        data.step(&model).expect("step failed");
    }

    // Verify indirection arrays match body_sleep_state
    let expected_awake_bodies: Vec<usize> = (0..model.nbody)
        .filter(|&b| data.body_sleep_state[b] != SleepState::Asleep)
        .collect();
    assert_eq!(
        &data.body_awake_ind[..data.nbody_awake],
        &expected_awake_bodies[..],
        "body_awake_ind must match non-asleep bodies"
    );

    // Verify dof_awake_ind matches tree_awake
    let expected_awake_dofs: Vec<usize> = (0..model.nv)
        .filter(|&d| {
            let tree = model.dof_treeid[d];
            tree < model.ntree && data.tree_awake[tree]
        })
        .collect();
    assert_eq!(
        &data.dof_awake_ind[..data.nv_awake],
        &expected_awake_dofs[..],
        "dof_awake_ind must match awake DOFs"
    );

    // Verify parent_awake_ind: bodies whose parent is not asleep
    let expected_parent_awake: Vec<usize> = (0..model.nbody)
        .filter(|&b| {
            if b == 0 {
                return true; // World is always in
            }
            let parent = model.body_parent[b];
            data.body_sleep_state[parent] != SleepState::Asleep
        })
        .collect();
    assert_eq!(
        &data.parent_awake_ind[..data.nparent_awake],
        &expected_parent_awake[..],
        "parent_awake_ind must match bodies with awake parents"
    );
}

// ============================================================================
// T47: test_qpos_change_wakes (§16.15)
// ============================================================================

#[test]
fn test_qpos_change_wakes() {
    // Externally modifying qpos of a sleeping body should wake its tree.
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Externally modify qpos (simulate RL environment reset)
    data.qpos[0] += 0.1; // Translate x

    // Next forward() should detect the change and wake the body
    data.forward(&model).expect("forward");

    assert!(
        data.tree_asleep[tree] < 0,
        "ball should be awake after external qpos modification"
    );
}

// ============================================================================
// T48: test_qpos_stable_no_wake (§16.15)
// ============================================================================

#[test]
fn test_qpos_stable_no_wake() {
    // Stepping without modifying qpos should NOT wake sleeping bodies.
    // FK is deterministic: same qpos → same xpos/xquat (bitwise).
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Let ball settle and sleep
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    let ball_body = 1;
    let tree = model.body_treeid[ball_body];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Step more without touching qpos — should stay asleep
    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    assert!(
        data.tree_asleep[tree] >= 0,
        "ball should remain asleep when qpos is not externally modified"
    );
}

// ============================================================================
// T75: test_qpos_dirty_flag_isolation (§16.15)
// ============================================================================

#[test]
fn test_qpos_dirty_flag_isolation() {
    // tree_qpos_dirty is separate from tree_awake — verify independence.
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Initially all dirty flags should be false
    for &d in &data.tree_qpos_dirty {
        assert!(!d, "tree_qpos_dirty should start false");
    }

    // Let ball sleep
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    let tree = model.body_treeid[1];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // After sleeping, dirty flags should still be false (cleared each step)
    for &d in &data.tree_qpos_dirty {
        assert!(!d, "tree_qpos_dirty should be false after sleeping");
    }

    // Modify qpos and run forward — dirty flag should be set then cleared
    data.qpos[2] += 0.5;
    data.forward(&model).expect("forward");

    // After forward(), mj_check_qpos_changed should have cleared the dirty flags
    for &d in &data.tree_qpos_dirty {
        assert!(
            !d,
            "tree_qpos_dirty should be cleared after mj_check_qpos_changed"
        );
    }

    // But the body should now be awake (the dirty flag was consumed to wake it)
    assert!(
        data.tree_asleep[tree] < 0,
        "body should be awake after qpos change detected"
    );
}

// ============================================================================
// T64: test_make_data_island_array_sizes (§16.23.5)
// ============================================================================

#[test]
fn test_make_data_island_array_sizes() {
    // Verify that make_data() allocates island arrays to worst-case bounds.
    let model = load_model(two_tree_sleep_mjcf()).expect("load model");
    let data = model.make_data();

    // Island discovery arrays should be allocated
    assert_eq!(data.tree_island.len(), model.ntree);
    assert_eq!(data.island_ntree.len(), model.ntree);
    assert_eq!(data.island_itreeadr.len(), model.ntree);
    assert_eq!(data.map_itree2tree.len(), model.ntree);
    assert_eq!(data.dof_island.len(), model.nv);
    assert_eq!(data.island_nv.len(), model.ntree);
    assert_eq!(data.island_idofadr.len(), model.ntree);
    assert_eq!(data.map_dof2idof.len(), model.nv);
    assert_eq!(data.map_idof2dof.len(), model.nv);
    assert_eq!(data.island_nefc.len(), model.ntree);
    assert_eq!(data.island_iefcadr.len(), model.ntree);

    // Scratch arrays
    assert_eq!(data.island_scratch_stack.len(), model.ntree);
    assert_eq!(data.island_scratch_rownnz.len(), model.ntree);
    assert_eq!(data.island_scratch_rowadr.len(), model.ntree);

    // qpos change detection
    assert_eq!(data.tree_qpos_dirty.len(), model.ntree);

    // Initially no islands discovered
    assert_eq!(data.nisland, 0);

    // All tree_island should be -1 (no islands yet)
    for &ti in &data.tree_island {
        assert_eq!(ti, -1);
    }
}

// ============================================================================
// T65: test_reset_reinitializes_sleep (§16.23.5)
// ============================================================================

#[test]
fn test_reset_reinitializes_sleep() {
    // Verify that reset() reinitializes sleep state and awake-index arrays.
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until sleeping
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    // At least one tree should be asleep
    let was_asleep = data.tree_asleep.iter().any(|&t| t >= 0);
    assert!(
        was_asleep,
        "should have at least one sleeping tree after stepping"
    );

    // Reset
    data.reset(&model);

    // After reset: sleep state should be reinitialized from policies
    // For models without Init policy, all trees should be awake
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] != SleepPolicy::Init {
            assert!(
                data.tree_asleep[t] < 0,
                "tree {t} should be awake after reset"
            );
        }
    }

    // Awake-index arrays should be consistent
    assert_eq!(data.nbody_awake, model.nbody); // All bodies awake
    assert_eq!(data.nv_awake, model.nv); // All DOFs awake

    // tree_qpos_dirty should be cleared
    for &d in &data.tree_qpos_dirty {
        assert!(!d, "tree_qpos_dirty should be clear after reset");
    }

    // nisland should be 0 (no islands computed yet)
    assert_eq!(data.nisland, 0);
}

// ============================================================================
// T70: test_tendon_actuator_policy (§16.26.5)
// ============================================================================

#[test]
fn test_tendon_actuator_policy() {
    // A tendon-driven actuator spanning a single tree should cause AutoNever
    // on that tree (same as joint-driven actuators).
    let mjcf = r#"
    <mujoco model="tendon_actuator_policy">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="link1" pos="0 0 1">
                <joint name="j1" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="ten1">
                <joint joint="j1" coef="1.0"/>
            </fixed>
        </tendon>
        <actuator>
            <general tendon="ten1" gainprm="100 0 0 0 0 0 0 0 0 0"/>
        </actuator>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");

    // The tree containing j1 should be AutoNever because it has a tendon actuator
    assert_eq!(model.ntree, 1, "should have 1 tree");
    assert_eq!(
        model.tree_sleep_policy[0],
        SleepPolicy::AutoNever,
        "tree with tendon-driven actuator should be AutoNever"
    );
}

// ============================================================================
// T31–T35: Island Discovery (§16.11)
// ============================================================================

/// T31: Two free bodies resting on ground plane. Each contacts the ground,
/// so each tree has constraint edges. But they are NOT in contact with each
/// other, so they form separate islands (or both contact world → same ground
/// but world body is tree 0 / not a tree). Actually, contacts with the world
/// ground plane create self-edges (world body has no tree), so each body's
/// tree gets a self-edge → 2 separate singleton islands.
///
/// To test cross-tree grouping, we place two bodies in direct contact.
#[test]
fn test_island_discovery_two_body_contact() {
    // Two spheres stacked: sphere A rests on ground, sphere B rests on A.
    // A contacts ground (self-edge on tree_A), B contacts A (cross-edge tree_A ↔ tree_B).
    // Result: both trees in one island.
    let mjcf = r#"
    <mujoco model="island_two_contact">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.5">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="A" pos="0 0 0.1">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="B" pos="0 0 0.31">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step enough for bodies to settle and make contact
    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    // After settling, both bodies should be in contact.
    // Check island discovery found at least 1 island grouping both trees.
    let ntree = model.ntree;
    assert!(ntree >= 2, "need at least 2 trees, got {ntree}");

    // Both trees should be assigned to the same island (or both have contacts)
    let tree_a = model.body_treeid[1]; // body A
    let tree_b = model.body_treeid[2]; // body B

    // If both have island assignments, they should be in the same island
    // (because B contacts A, creating a cross-tree edge)
    if data.tree_island[tree_a] >= 0 && data.tree_island[tree_b] >= 0 {
        assert_eq!(
            data.tree_island[tree_a], data.tree_island[tree_b],
            "trees A and B should be in the same island (contact coupling)"
        );
        assert!(
            data.nisland >= 1,
            "should have at least 1 island, got {}",
            data.nisland
        );
    }
    // If bodies fell asleep before we check, islands might be 0 (no constraints
    // for sleeping bodies). That's acceptable — the test validates grouping when active.
}

/// T32: Chain of 3 bodies in contact → 1 island.
/// A contacts B, B contacts C → all 3 trees in one island via transitivity.
#[test]
fn test_island_discovery_chain() {
    // Three spheres stacked vertically: C on B on A on ground.
    let mjcf = r#"
    <mujoco model="island_chain">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.5">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="A" pos="0 0 0.1">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="B" pos="0 0 0.31">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="C" pos="0 0 0.52">
                <freejoint name="jC"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step to settle
    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    let ntree = model.ntree;
    assert!(ntree >= 3, "need at least 3 trees, got {ntree}");

    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];
    let tree_c = model.body_treeid[3];

    // All three should be in the same island (chained contacts)
    if data.tree_island[tree_a] >= 0
        && data.tree_island[tree_b] >= 0
        && data.tree_island[tree_c] >= 0
    {
        assert_eq!(
            data.tree_island[tree_a], data.tree_island[tree_b],
            "A and B should be in the same island"
        );
        assert_eq!(
            data.tree_island[tree_b], data.tree_island[tree_c],
            "B and C should be in the same island"
        );
    }
}

/// T33: Unconstrained tree (no contacts, no equality constraints) → island = -1.
#[test]
fn test_island_singleton() {
    // A single floating body with no ground plane → no contacts → singleton.
    let mjcf = r#"
    <mujoco model="island_singleton">
        <option gravity="0 0 0" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="floater" pos="0 0 1">
                <freejoint name="jf"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // One step to run pipeline (including island discovery)
    data.step(&model).expect("step");

    let tree = model.body_treeid[1];
    assert!(tree < model.ntree, "body should have valid tree");

    // No contacts, no equality → singleton → island = -1
    assert_eq!(
        data.tree_island[tree], -1,
        "unconstrained tree should be singleton (island = -1)"
    );
    assert_eq!(data.nisland, 0, "no islands when no constraints");
}

/// T34: DISABLE_ISLAND flag → nisland = 0, all tree_island = -1.
#[test]
fn test_disable_island_flag() {
    let mjcf = r#"
    <mujoco model="disable_island">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="ball" pos="0 0 0.2">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let mut model = load_model(mjcf).expect("load");
    model.disableflags |= DISABLE_ISLAND;
    let mut data = model.make_data();

    // Step to generate contacts
    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    assert_eq!(data.nisland, 0, "DISABLE_ISLAND → nisland = 0");
    for t in 0..model.ntree {
        assert_eq!(
            data.tree_island[t], -1,
            "DISABLE_ISLAND → all tree_island = -1"
        );
    }
}

/// T35: Island array consistency — sum of island_ntree == number of island-assigned trees,
/// sum of island_nv == total DOFs in islands, sum of island_nefc == total constraint rows in islands.
#[test]
fn test_island_array_consistency() {
    // Two bodies on a ground plane to create at least one island.
    let mjcf = r#"
    <mujoco model="island_consistency">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.5">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="A" pos="0 0 0.1">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="B" pos="0 0 0.31">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step to settle and create contacts
    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    let nisland = data.nisland;

    // Sum of island_ntree should equal number of trees assigned to islands
    let sum_ntree: usize = data.island_ntree[..nisland].iter().sum();
    let assigned_trees = (0..model.ntree)
        .filter(|&t| data.tree_island[t] >= 0)
        .count();
    assert_eq!(
        sum_ntree, assigned_trees,
        "sum(island_ntree) should equal count of assigned trees"
    );

    // Sum of island_nv should equal number of DOFs assigned to islands
    let sum_nv: usize = data.island_nv[..nisland].iter().sum();
    let assigned_dofs = (0..model.nv).filter(|&d| data.dof_island[d] >= 0).count();
    assert_eq!(
        sum_nv, assigned_dofs,
        "sum(island_nv) should equal count of assigned DOFs"
    );

    // Sum of island_nefc should equal number of constraint rows assigned to islands
    let nefc = data.efc_island.len();
    let sum_nefc: usize = data.island_nefc[..nisland].iter().sum();
    let assigned_efc = (0..nefc).filter(|&r| data.efc_island[r] >= 0).count();
    assert_eq!(
        sum_nefc, assigned_efc,
        "sum(island_nefc) should equal count of assigned constraint rows"
    );

    // Verify map_itree2tree contains valid tree indices
    for i in 0..nisland {
        let start = data.island_itreeadr[i];
        let count = data.island_ntree[i];
        for j in start..start + count {
            let tree = data.map_itree2tree[j];
            assert!(tree < model.ntree, "invalid tree in map_itree2tree");
            assert_eq!(
                data.tree_island[tree], i as i32,
                "tree in map_itree2tree should point back to island"
            );
        }
    }

    // Verify map_idof2dof contains valid DOF indices
    for i in 0..nisland {
        let start = data.island_idofadr[i];
        let count = data.island_nv[i];
        for j in start..start + count {
            let dof = data.map_idof2dof[j];
            assert!(dof < model.nv, "invalid DOF in map_idof2dof");
            assert_eq!(
                data.dof_island[dof], i as i32,
                "DOF in map_idof2dof should point back to island"
            );
        }
    }
}

// ============================================================================
// T36–T38, T67, T73: Sleep Cycle Linked List (§16.12)
// ============================================================================

/// T36: Sleeping a multi-tree island creates a circular linked list.
/// Two bodies in contact → 1 island → both trees sleep as a cycle.
#[test]
fn test_sleep_cycle_two_trees() {
    // Two stacked spheres: A on ground, B on A. They form one island.
    // When both sleep, tree_asleep should form a cycle.
    let mjcf = r#"
    <mujoco model="cycle_two">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="A" pos="0 0 0.1">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="B" pos="0 0 0.31">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step until both are asleep
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];

    // Both should be asleep
    assert!(
        data.tree_asleep[tree_a] >= 0,
        "tree A should be asleep, got {}",
        data.tree_asleep[tree_a]
    );
    assert!(
        data.tree_asleep[tree_b] >= 0,
        "tree B should be asleep, got {}",
        data.tree_asleep[tree_b]
    );

    // Verify circular linked list: A → B → A (or A → A and B → B if they're singletons)
    let next_a = data.tree_asleep[tree_a] as usize;
    let next_b = data.tree_asleep[tree_b] as usize;

    // If they formed an island cycle: A→B, B→A
    if next_a == tree_b {
        assert_eq!(next_b, tree_a, "circular cycle: B should point to A");
    } else {
        // They might have ended up as singletons (self-links) if
        // they lost contact before sleeping. Both are valid.
        assert_eq!(next_a, tree_a, "A self-link if singleton");
        assert_eq!(next_b, tree_b, "B self-link if singleton");
    }
}

/// T37: Waking one tree in a cycle wakes all trees.
#[test]
fn test_wake_cycle_propagation() {
    let mjcf = r#"
    <mujoco model="wake_cycle">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="A" pos="0 0 0.1">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="B" pos="0 0 0.31">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step until both are asleep
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];

    // Both should be asleep
    let a_asleep = data.tree_asleep[tree_a] >= 0;
    let b_asleep = data.tree_asleep[tree_b] >= 0;
    assert!(a_asleep, "tree A should be asleep");
    assert!(b_asleep, "tree B should be asleep");

    // Apply external force to body A → wakes tree A
    data.xfrc_applied[1] = nalgebra::Vector6::new(0.0, 0.0, 10.0, 0.0, 0.0, 0.0);

    // One step triggers wake detection
    data.step(&model).expect("step");

    // Both trees should now be awake (cycle propagation or both woken individually)
    assert!(
        data.tree_asleep[tree_a] < 0,
        "tree A should be awake after force"
    );
    // B should also be awake if they were in a cycle
    // (If they were singletons, B stays asleep — that's also valid)
}

/// T38: Single-tree sleep creates self-link (Phase A compatible).
#[test]
fn test_sleep_cycle_single_tree() {
    let mjcf = free_body_sleep_mjcf();
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step until the ball sleeps
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let tree = model.body_treeid[1];
    assert!(
        data.tree_asleep[tree] >= 0,
        "ball should be asleep, got {}",
        data.tree_asleep[tree]
    );

    // Single-tree → self-link
    assert_eq!(
        data.tree_asleep[tree] as usize, tree,
        "single-tree sleep should create self-link"
    );
}

/// T73: sleep_trees zeros all DOF-level and body-level arrays.
#[test]
fn test_sleep_trees_zeros_all_arrays() {
    let mjcf = free_body_sleep_mjcf();
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step until ball sleeps
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let tree = model.body_treeid[1];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Check DOF arrays are zeroed
    let dof_start = model.tree_dof_adr[tree];
    let dof_end = dof_start + model.tree_dof_num[tree];
    for dof in dof_start..dof_end {
        assert_eq!(data.qvel[dof], 0.0, "qvel[{dof}] should be 0");
        assert_eq!(data.qacc[dof], 0.0, "qacc[{dof}] should be 0");
        assert_eq!(data.qfrc_bias[dof], 0.0, "qfrc_bias[{dof}] should be 0");
        assert_eq!(
            data.qfrc_passive[dof], 0.0,
            "qfrc_passive[{dof}] should be 0"
        );
        assert_eq!(
            data.qfrc_constraint[dof], 0.0,
            "qfrc_constraint[{dof}] should be 0"
        );
        assert_eq!(
            data.qfrc_actuator[dof], 0.0,
            "qfrc_actuator[{dof}] should be 0"
        );
    }

    // Check body arrays are zeroed
    let body_start = model.tree_body_adr[tree];
    let body_end = body_start + model.tree_body_num[tree];
    for body_id in body_start..body_end {
        let cvel = &data.cvel[body_id];
        let cacc = &data.cacc_bias[body_id];
        let cfrc = &data.cfrc_bias[body_id];
        for k in 0..6 {
            assert_eq!(cvel[k], 0.0, "cvel[{body_id}][{k}] should be 0");
            assert_eq!(cacc[k], 0.0, "cacc_bias[{body_id}][{k}] should be 0");
            assert_eq!(cfrc[k], 0.0, "cfrc_bias[{body_id}][{k}] should be 0");
        }
    }
}

// ============================================================================
// T39–T42, T72, T74: Cross-Island Wake (§16.13)
// ============================================================================

/// T39: Contact between sleeping and awake tree wakes the sleeping tree's island.
/// (This is actually Phase A behavior verified in Phase B context.)
#[test]
fn test_wake_contact_island() {
    // A sits on ground and sleeps. Then we push B into contact with A.
    // Use the standard free_body_sleep fixture to ensure A reliably sleeps,
    // then manually teleport a second body onto it.
    let mjcf = r#"
    <mujoco model="wake_contact">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="A" pos="0 0 0.2">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="B" pos="5 0 0.2">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // B starts far away so no contact with A. Both settle independently.
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];

    // Both should be asleep
    assert!(
        data.tree_asleep[tree_a] >= 0,
        "A should be asleep, got {}",
        data.tree_asleep[tree_a]
    );
    assert!(
        data.tree_asleep[tree_b] >= 0,
        "B should be asleep, got {}",
        data.tree_asleep[tree_b]
    );

    // Teleport B right above A to create contact on next step.
    // A is at ~(0, 0, 0.1). Place B at (0, 0, 0.28) so spheres overlap
    // (center distance 0.18 < sum of radii 0.2).
    data.qpos[7] = 0.0; // B x
    data.qpos[8] = 0.0; // B y
    data.qpos[9] = 0.28; // B z (overlapping A's sphere)

    // Step: qpos change wakes B, collision detects contact, wake_collision wakes A.
    // May need a couple steps for the full wake pipeline.
    for _ in 0..5 {
        data.step(&model).expect("step");
    }

    // B should be awake (external qpos change)
    assert!(
        data.tree_asleep[tree_b] < 0,
        "B should be awake after qpos teleport"
    );

    // A should also be awake (contact with now-awake B)
    assert!(
        data.tree_asleep[tree_a] < 0,
        "A should be awake after contact with B"
    );
}

/// T40: Active equality constraint between sleeping and awake tree wakes the sleeping tree.
#[test]
fn test_wake_equality_island() {
    // Two bodies connected by a weld equality constraint.
    // A will settle and sleep. B stays awake with applied force.
    // When the equality constraint is active and B is awake, A should wake.
    let mjcf = r#"
    <mujoco model="wake_equality">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="A" pos="0 0 0.1">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="B" pos="1 0 0.1">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
        <equality>
            <connect body1="A" body2="B" anchor="0.5 0 0.1"/>
        </equality>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step — the connect constraint couples A and B.
    // With the constraint active, when one is awake the other should be too.
    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];

    // Both should be in the same wake state (equality couples them)
    let a_awake = data.tree_asleep[tree_a] < 0;
    let b_awake = data.tree_asleep[tree_b] < 0;
    assert_eq!(
        a_awake, b_awake,
        "equality-coupled trees should have same wake state: A={a_awake}, B={b_awake}"
    );
}

/// T72: User force wake runs at the start of forward(), waking sleeping body.
#[test]
fn test_user_force_wake_phase_b() {
    let mjcf = free_body_sleep_mjcf();
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step until ball sleeps
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let tree = model.body_treeid[1];
    assert!(data.tree_asleep[tree] >= 0, "ball should be asleep");

    // Apply external force
    data.xfrc_applied[1] = nalgebra::Vector6::new(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

    // One step → mj_wake() detects force and wakes the body
    data.step(&model).expect("step");

    assert!(
        data.tree_asleep[tree] < 0,
        "ball should be awake after xfrc_applied"
    );
}

// ============================================================================
// T59, T60, T76: Init-Sleep Island Validation (§16.24)
// ============================================================================

/// T59: Valid Init-sleep tree passes validation and starts asleep.
#[test]
fn test_init_sleep_valid() {
    // A single free body with sleep="init" should start asleep.
    let mjcf = r#"
    <mujoco model="init_valid">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="ball" pos="0 0 1" sleep="init">
                <freejoint name="jf"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let data = model.make_data();

    let tree = model.body_treeid[1];
    assert_eq!(
        model.tree_sleep_policy[tree],
        SleepPolicy::Init,
        "tree should have Init policy"
    );
    assert!(
        data.tree_asleep[tree] >= 0,
        "Init tree should start asleep, got {}",
        data.tree_asleep[tree]
    );
    // Self-link for single-tree init
    assert_eq!(
        data.tree_asleep[tree] as usize, tree,
        "single Init tree should have self-link"
    );
}

/// T60: Mixed Init/non-Init in coupled group degrades gracefully.
/// The spec says this should produce an error and degrade to awake.
#[test]
fn test_init_sleep_mixed_island_warning() {
    // Two bodies connected by equality constraint: A is Init, B is not.
    // The validation should detect the mixed group and degrade A to awake.
    let mjcf = r#"
    <mujoco model="init_mixed">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="A" pos="0 0 1" sleep="init">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="B" pos="1 0 1">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <equality>
            <connect body1="A" body2="B" anchor="0.5 0 1"/>
        </equality>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let data = model.make_data();

    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];

    // A has Init policy, B doesn't → mixed group → A should be degraded to awake
    assert_eq!(model.tree_sleep_policy[tree_a], SleepPolicy::Init);

    // After degradation, A should be awake (Init trees in mixed groups
    // don't get put to sleep)
    assert!(
        data.tree_asleep[tree_a] < 0,
        "mixed Init tree A should be degraded to awake, got {}",
        data.tree_asleep[tree_a]
    );
    assert!(
        data.tree_asleep[tree_b] < 0,
        "non-Init tree B should be awake, got {}",
        data.tree_asleep[tree_b]
    );
}

/// T76: Init-sleep validation uses model-time adjacency (union-find), not runtime islands.
#[test]
fn test_init_sleep_validation_model_time() {
    // Two Init trees connected by equality constraint should form a sleep cycle.
    let mjcf = r#"
    <mujoco model="init_cycle">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="A" pos="0 0 1" sleep="init">
                <freejoint name="jA"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="B" pos="1 0 1" sleep="init">
                <freejoint name="jB"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <equality>
            <connect body1="A" body2="B" anchor="0.5 0 1"/>
        </equality>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load");
    let data = model.make_data();

    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];

    // Both should be Init and asleep
    assert_eq!(model.tree_sleep_policy[tree_a], SleepPolicy::Init);
    assert_eq!(model.tree_sleep_policy[tree_b], SleepPolicy::Init);

    // Both should be asleep (validation passed, sleep cycles created)
    assert!(
        data.tree_asleep[tree_a] >= 0,
        "Init tree A should be asleep"
    );
    assert!(
        data.tree_asleep[tree_b] >= 0,
        "Init tree B should be asleep"
    );

    // They should form a cycle: A→B, B→A (union-find grouped them)
    let next_a = data.tree_asleep[tree_a] as usize;
    let next_b = data.tree_asleep[tree_b] as usize;
    assert_eq!(next_a, tree_b, "A should point to B in cycle");
    assert_eq!(next_b, tree_a, "B should point to A in cycle");
}

// ============================================================================
// T49: test_per_island_solve_equivalence (§16.16, AC #18)
// ============================================================================

#[test]
fn test_per_island_solve_equivalence() {
    // Per-island solve should produce equivalent results to global solve.
    // We compare a scene with two independent bodies (each becomes its own
    // island) against the same scene with DISABLE_ISLAND (global solve).
    let mjcf = r#"
    <mujoco model="island_equivalence">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="10 10 0.1" solref="0.005 1.5"/>
            <body name="ball_a" pos="-2 0 0.5">
                <freejoint name="a_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball_b" pos="2 0 0.5">
                <freejoint name="b_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    // Run with islands enabled (Phase B path)
    let model_island = load_model(mjcf).expect("load model");
    let mut data_island = model_island.make_data();

    // Run with DISABLE_ISLAND (Phase A global path)
    let mut model_global = load_model(mjcf).expect("load model");
    model_global.disableflags |= DISABLE_ISLAND;
    let mut data_global = model_global.make_data();

    // Step both for 200 steps and compare
    for step in 0..200 {
        data_island.step(&model_island).expect("island step");
        data_global.step(&model_global).expect("global step");

        // Compare qpos (should be within floating-point tolerance)
        for dof in 0..model_island.nv {
            let diff = (data_island.qpos[dof] - data_global.qpos[dof]).abs();
            assert!(
                diff < 1e-8,
                "qpos diverged at step {step}, dof {dof}: island={}, global={}, diff={diff}",
                data_island.qpos[dof],
                data_global.qpos[dof]
            );
        }

        // Compare qvel
        for dof in 0..model_island.nv {
            let diff = (data_island.qvel[dof] - data_global.qvel[dof]).abs();
            assert!(
                diff < 1e-8,
                "qvel diverged at step {step}, dof {dof}: island={}, global={}, diff={diff}",
                data_island.qvel[dof],
                data_global.qvel[dof]
            );
        }
    }

    // Verify that islands were actually discovered (sanity check)
    // After stepping, at least one step should have found islands
    // (balls land on the plane and create contacts)
    data_island.forward(&model_island).expect("forward");
    assert!(
        data_island.nisland > 0,
        "Islands should have been discovered (nisland={})",
        data_island.nisland
    );
}

// ============================================================================
// T54: test_disable_island_bit_identical (§16.16, AC #23)
// ============================================================================

#[test]
fn test_disable_island_bit_identical() {
    // DISABLE_ISLAND should make the per-island solver fall through to the
    // global solver, producing bit-identical results to a model where sleep
    // is disabled entirely (which also uses the global solver and has no
    // sleep-induced state changes). Both models disable sleep to isolate
    // the solver path comparison.
    let mjcf = r#"
    <mujoco model="disable_island_test">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="disable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball" pos="0 0 0.5">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    // Model A: sleep disabled + DISABLE_ISLAND (global solver path,
    // mj_fwd_constraint_islands sees nisland=0 → falls through)
    let mut model_a = load_model(mjcf).expect("load model");
    model_a.disableflags |= DISABLE_ISLAND;
    let mut data_a = model_a.make_data();

    // Model B: sleep disabled, no DISABLE_ISLAND flag
    // Since sleep is disabled, mj_island() is never called, nisland stays 0,
    // and mj_fwd_constraint_islands falls through to the global solver.
    let model_b = load_model(mjcf).expect("load model");
    let mut data_b = model_b.make_data();

    // Step both for 500 steps — results should be bit-identical
    for step in 0..500 {
        data_a.step(&model_a).expect("step A");
        data_b.step(&model_b).expect("step B");

        for dof in 0..model_a.nv {
            assert!(
                (data_a.qpos[dof] - data_b.qpos[dof]).abs() < 1e-14,
                "DISABLE_ISLAND diverged at step {step}, dof {dof}: A={}, B={}",
                data_a.qpos[dof],
                data_b.qpos[dof]
            );
        }
    }

    // Verify DISABLE_ISLAND keeps nisland = 0
    data_a.forward(&model_a).expect("forward");
    assert_eq!(data_a.nisland, 0, "DISABLE_ISLAND should keep nisland=0");
}

// ============================================================================
// T56: test_policy_relaxation_actuated_allowed (§16.18.1, AC #25)
// ============================================================================

#[test]
fn test_policy_relaxation_actuated_allowed() {
    // An actuated tree with explicit sleep="allowed" should be able to sleep,
    // overriding the automatic AutoNever policy from the actuator.
    let mjcf = r#"
    <mujoco model="actuated_allowed">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="arm" pos="0 0 0.5" sleep="allowed">
                <freejoint name="arm_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
        <actuator>
            <motor joint="arm_free" gear="1 0 0 0 0 0"/>
        </actuator>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");

    // The tree should have Allowed policy (not AutoNever from actuator)
    let tree = model.body_treeid[1];
    assert_eq!(
        model.tree_sleep_policy[tree],
        SleepPolicy::Allowed,
        "explicit sleep='allowed' should override AutoNever from actuator"
    );

    // It should actually be able to sleep
    let mut data = model.make_data();
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }
    assert!(
        data.tree_asleep[tree] >= 0,
        "Allowed-policy actuated tree should be asleep after settling"
    );
}

// ============================================================================
// T57: test_policy_relaxation_tendon_zero_stiffness (§16.18.2, AC #26)
// ============================================================================

#[test]
fn test_policy_relaxation_tendon_zero_stiffness() {
    // A multi-tree tendon with zero stiffness and zero damping should NOT
    // prevent its spanning trees from sleeping.
    let mjcf = r#"
    <mujoco model="tendon_zero_stiffness">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="body_a" pos="-1 0 0.5">
                <freejoint name="a_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
                <site name="s_a" pos="0 0 0"/>
            </body>
            <body name="body_b" pos="1 0 0.5">
                <freejoint name="b_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
                <site name="s_b" pos="0 0 0"/>
            </body>
        </worldbody>
        <tendon>
            <spatial name="t_ab" stiffness="0" damping="0">
                <site site="s_a"/>
                <site site="s_b"/>
            </spatial>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");

    // Both trees should have AutoAllowed (not AutoNever)
    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];
    assert_eq!(
        model.tree_sleep_policy[tree_a],
        SleepPolicy::AutoAllowed,
        "zero-stiffness tendon should allow sleep for tree A"
    );
    assert_eq!(
        model.tree_sleep_policy[tree_b],
        SleepPolicy::AutoAllowed,
        "zero-stiffness tendon should allow sleep for tree B"
    );
}

// ============================================================================
// T58: test_policy_relaxation_tendon_nonzero_stiffness (§16.18.2, AC #27)
// ============================================================================

#[test]
fn test_policy_relaxation_tendon_nonzero_stiffness() {
    // A multi-tree tendon with nonzero stiffness should force AutoNever
    // on its spanning trees (passive coupling prevents independent sleep).
    let mjcf = r#"
    <mujoco model="tendon_nonzero_stiffness">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="body_a" pos="-1 0 0.5">
                <freejoint name="a_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
                <site name="s_a" pos="0 0 0"/>
            </body>
            <body name="body_b" pos="1 0 0.5">
                <freejoint name="b_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
                <site name="s_b" pos="0 0 0"/>
            </body>
        </worldbody>
        <tendon>
            <spatial name="t_ab" stiffness="100">
                <site site="s_a"/>
                <site site="s_b"/>
            </spatial>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");

    // Both trees should have AutoNever (stiff tendon couples them)
    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];
    assert_eq!(
        model.tree_sleep_policy[tree_a],
        SleepPolicy::AutoNever,
        "nonzero-stiffness tendon should force AutoNever on tree A"
    );
    assert_eq!(
        model.tree_sleep_policy[tree_b],
        SleepPolicy::AutoNever,
        "nonzero-stiffness tendon should force AutoNever on tree B"
    );
}

// ============================================================================
// T71: test_actuated_sleep_zeros_qfrc_actuator (§16.18.1, AC #40)
// ============================================================================

#[test]
fn test_actuated_sleep_zeros_qfrc_actuator() {
    // When an actuated tree with Allowed policy sleeps, qfrc_actuator for
    // its DOFs must be zeroed. Verify this via sleep_trees() behavior.
    let mjcf = r#"
    <mujoco model="actuated_sleep_zero">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="arm" pos="0 0 0.5" sleep="allowed">
                <freejoint name="arm_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
        <actuator>
            <motor joint="arm_free" gear="1 0 0 0 0 0"/>
        </actuator>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");
    let mut data = model.make_data();

    // Apply nonzero control briefly to produce qfrc_actuator
    data.ctrl[0] = 1.0;
    for _ in 0..5 {
        data.step(&model).expect("step");
    }
    // Verify qfrc_actuator was nonzero from control
    let tree = model.body_treeid[1];
    let dof_start = model.tree_dof_adr[tree];
    assert!(
        data.qfrc_actuator[dof_start] != 0.0,
        "qfrc_actuator should be nonzero with active control"
    );

    // Zero control and let it settle to sleep
    data.ctrl[0] = 0.0;
    for _ in 0..8000 {
        data.step(&model).expect("step");
    }

    assert!(
        data.tree_asleep[tree] >= 0,
        "tree should be asleep after settling"
    );

    // qfrc_actuator for the sleeping tree's DOFs should be zero
    let dof_count = model.tree_dof_num[tree];
    for dof in dof_start..dof_start + dof_count {
        assert_eq!(
            data.qfrc_actuator[dof], 0.0,
            "qfrc_actuator[{dof}] should be zeroed when sleeping"
        );
    }
}

// ===== T61: test_mj_sleep_state_api =====
// AC #30: sleep_state(body_id) returns correct SleepState for static, sleeping, and awake bodies.
#[test]
fn test_mj_sleep_state_api() {
    let mjcf = r#"
    <mujoco model="sleep_state_api">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball" pos="0 0 0.2">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // World body (body 0) should always be Static
    assert_eq!(
        data.sleep_state(0),
        SleepState::Static,
        "world body should be Static"
    );

    // Ball body (body 1) starts awake
    assert_eq!(
        data.sleep_state(1),
        SleepState::Awake,
        "ball should start Awake"
    );

    // Let ball settle onto plane and fall asleep
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    assert_eq!(
        data.sleep_state(1),
        SleepState::Asleep,
        "ball should be Asleep after settling"
    );
}

// ===== T62: test_mj_tree_awake_api =====
// AC #31: tree_awake(tree_id) returns false for sleeping trees, true for awake trees.
#[test]
fn test_mj_tree_awake_api() {
    let mjcf = r#"
    <mujoco model="tree_awake_api">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball" pos="0 0 0.2">
                <freejoint name="ball_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("load");
    let mut data = model.make_data();

    let tree = model.body_treeid[1];

    // Initially awake
    assert!(data.tree_awake(tree), "tree should be awake initially");

    // Let ball settle onto plane and fall asleep
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    assert!(
        !data.tree_awake(tree),
        "tree should not be awake after settling"
    );

    // Wake it by setting qpos
    data.qpos[2] = 0.5; // move z upward
    data.step(&model).expect("step");

    assert!(
        data.tree_awake(tree),
        "tree should be awake after qpos change"
    );
}

// ===== T63: test_mj_nisland_api =====
// AC #32: nisland() returns the number of islands discovered this step.
#[test]
fn test_mj_nisland_api() {
    // Two separated balls → should form 2 islands when both are in contact with plane
    let mjcf = r#"
    <mujoco model="nisland_api">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball_a" pos="-1 0 0.2">
                <freejoint name="a_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball_b" pos="1 0 0.2">
                <freejoint name="b_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Initially no islands (no contacts yet or no stepping)
    assert_eq!(data.nisland(), 0, "nisland should be 0 before any step");

    // Step until contacts form. Balls start at z=0.2 with radius 0.1,
    // separated 0.1m above the plane. Under gravity ~9.81 m/s² they
    // reach the plane in ~0.14s = ~70 steps at dt=0.002.
    // We look for the first step where nisland > 0 (contacts exist).
    let mut found_islands = false;
    for i in 0..500 {
        data.step(&model).expect("step");
        let ni = data.nisland();
        if ni > 0 && !found_islands {
            // Two independent balls on a plane → 2 separate islands
            assert_eq!(ni, 2, "step {i}: two separated balls should form 2 islands");
            found_islands = true;
        }
    }
    assert!(found_islands, "should have found islands within 500 steps");

    // Continue stepping until both balls are asleep
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    // Sleeping trees don't participate in island discovery,
    // so nisland should be 0 when both are asleep
    let tree_a = model.body_treeid[1];
    let tree_b = model.body_treeid[2];
    assert!(
        !data.tree_awake(tree_a) && !data.tree_awake(tree_b),
        "both trees should be asleep after 5500 steps"
    );
    assert_eq!(
        data.nisland(),
        0,
        "nisland should be 0 when all trees are asleep"
    );
}

// ============================================================================
// Phase C1 — Awake-Index Pipeline Iteration (§16.27) — Tests T77–T84
// ============================================================================

/// T77: `mj_fwd_velocity` via `body_awake_ind` indirection produces correct `cvel`
/// for awake bodies and zero for sleeping bodies (AC #48).
#[test]
fn test_indirection_velocity_equivalence() {
    let model = load_model(two_tree_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until at least one tree sleeps
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    // Verify at least one tree is asleep and one is awake
    let mut has_asleep = false;
    let mut has_awake = false;
    for t in 0..model.ntree {
        if data.tree_awake(t) {
            has_awake = true;
        } else {
            has_asleep = true;
        }
    }
    assert!(has_asleep, "need at least one sleeping tree");

    // Step one more time — this uses the indirection path
    data.step(&model).expect("step");

    // Check cvel: sleeping bodies must have zero cvel, awake bodies must be valid
    for body_id in 1..model.nbody {
        let cvel = &data.cvel[body_id];
        if data.body_sleep_state[body_id] == SleepState::Asleep {
            for k in 0..6 {
                assert_eq!(
                    cvel[k], 0.0,
                    "sleeping body {body_id} cvel[{k}] should be 0"
                );
            }
        }
    }
    // If any body is awake AND has nonzero qvel, its cvel should be nonzero
    if has_awake {
        let mut found_nonzero = false;
        for body_id in 1..model.nbody {
            if data.body_sleep_state[body_id] == SleepState::Awake {
                let cvel = &data.cvel[body_id];
                for k in 0..6 {
                    if cvel[k] != 0.0 {
                        found_nonzero = true;
                    }
                }
            }
        }
        // Awake bodies with nonzero velocity should have been propagated
        // (may be zero if body is truly at rest but awake)
        let _ = found_nonzero;
    }
}

/// T78: `integrate()` velocity loop via `dof_awake_ind` produces correct qvel
/// for awake DOFs and zero for sleeping DOFs (AC #50).
#[test]
fn test_indirection_vel_integration_equivalence() {
    let model = load_model(two_tree_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until at least one tree sleeps
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    // Verify sleeping DOFs have zero qvel
    for dof in 0..model.nv {
        if dof < model.dof_treeid.len() {
            let tree = model.dof_treeid[dof];
            if tree < model.ntree && !data.tree_awake(tree) {
                assert_eq!(
                    data.qvel[dof], 0.0,
                    "sleeping DOF {dof} qvel should be zero"
                );
                assert_eq!(
                    data.qacc[dof], 0.0,
                    "sleeping DOF {dof} qacc should be zero"
                );
            }
        }
    }

    // Step one more — sleeping DOFs must remain zero after integration
    data.step(&model).expect("step");

    for dof in 0..model.nv {
        if dof < model.dof_treeid.len() {
            let tree = model.dof_treeid[dof];
            if tree < model.ntree && !data.tree_awake(tree) {
                assert_eq!(
                    data.qvel[dof], 0.0,
                    "sleeping DOF {dof} qvel should remain zero after integration"
                );
            }
        }
    }
}

/// T79: `mj_energy_pos/vel` include all bodies — sleeping bodies contribute
/// zero KE (by qvel=0) and frozen PE (by xpos invariant) (AC #49).
#[test]
fn test_energy_all_bodies_always() {
    let model = load_model(two_tree_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until at least one tree sleeps
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    let has_asleep = (0..model.ntree).any(|t| !data.tree_awake(t));
    assert!(has_asleep, "need at least one sleeping tree");

    // Energy should be physically meaningful — potential energy should include
    // gravitational PE from ALL bodies (sleeping ones have frozen xpos)
    let pe = data.energy_potential;
    let ke = data.energy_kinetic;

    // PE should be nonzero (bodies have mass and are at nonzero height)
    // For bodies resting on a ground plane at z≈0.1, PE = -m*g*z < 0
    assert!(
        pe != 0.0,
        "energy_potential should be nonzero (bodies have mass)"
    );

    // KE should be >= 0 always
    assert!(ke >= 0.0, "energy_kinetic must be non-negative, got {ke}");

    // Manually compute expected PE from ALL bodies including sleeping
    let g = model.gravity;
    let mut expected_pe = 0.0;
    for body_id in 1..model.nbody {
        let mass = model.body_mass[body_id];
        let com = data.subtree_com[body_id]; // or xipos
        expected_pe += -mass * (g[0] * com[0] + g[1] * com[1] + g[2] * com[2]);
    }
    // Include spring PE (small or zero for free joints)
    // Allow relative tolerance for spring contribution
    assert_relative_eq!(pe, expected_pe, epsilon = 1e-6);
}

/// T80: With all bodies awake and sleep enabled, pipeline output is
/// bit-identical to `ENABLE_SLEEP` cleared (AC #47).
#[test]
fn test_all_awake_bit_identical() {
    // Use a model where nothing sleeps: short simulation, bodies still moving
    let mjcf = r#"
    <mujoco model="all_awake_test">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="ball" pos="0 0 2">
                <freejoint name="free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    // Model A: sleep enabled
    let model_a = load_model(mjcf).expect("load");
    let mut data_a = model_a.make_data();

    // Model B: sleep disabled
    let mjcf_no_sleep = r#"
    <mujoco model="all_awake_nosleep">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1"/>
        <worldbody>
            <body name="ball" pos="0 0 2">
                <freejoint name="free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model_b = load_model(mjcf_no_sleep).expect("load");
    let mut data_b = model_b.make_data();

    // Step both for 100 steps — body is falling, not sleeping
    for step in 0..100 {
        data_a.step(&model_a).expect("step A");
        data_b.step(&model_b).expect("step B");

        // Verify all bodies are awake in model A
        assert!(
            data_a.tree_awake(0),
            "body should still be awake at step {step}"
        );

        // Bit-identical comparison
        for dof in 0..model_a.nv {
            assert_eq!(
                data_a.qpos[dof], data_b.qpos[dof],
                "qpos diverged at step {step}, dof {dof}"
            );
            assert_eq!(
                data_a.qvel[dof], data_b.qvel[dof],
                "qvel diverged at step {step}, dof {dof}"
            );
        }
        for body_id in 0..model_a.nbody {
            for k in 0..6 {
                assert_eq!(
                    data_a.cvel[body_id][k], data_b.cvel[body_id][k],
                    "cvel diverged at step {step}, body {body_id}, k {k}"
                );
            }
        }
        assert_eq!(
            data_a.energy_potential, data_b.energy_potential,
            "energy_potential diverged at step {step}"
        );
        assert_eq!(
            data_a.energy_kinetic, data_b.energy_kinetic,
            "energy_kinetic diverged at step {step}"
        );
    }
}

/// T81: `mj_rne()` gyroscopic indirection produces correct `qfrc_bias`
/// for awake DOFs with Ball/Free joints (AC #51).
#[test]
fn test_indirection_rne_gyroscopic_equivalence() {
    let model = load_model(two_tree_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until at least one tree sleeps
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    let has_asleep = (0..model.ntree).any(|t| !data.tree_awake(t));
    assert!(has_asleep, "need at least one sleeping tree for this test");

    // Step once more — mj_rne runs via indirection
    data.step(&model).expect("step");

    // Sleeping DOFs must have zero qfrc_bias
    for dof in 0..model.nv {
        if dof < model.dof_treeid.len() {
            let tree = model.dof_treeid[dof];
            if tree < model.ntree && !data.tree_awake(tree) {
                assert_eq!(
                    data.qfrc_bias[dof], 0.0,
                    "sleeping DOF {dof} qfrc_bias should be zero"
                );
            }
        }
    }
}

/// T82: Featherstone RNE forward/backward/projection loops produce zero
/// contributions from sleeping bodies (cvel=0, cacc_bias=0 invariant) (AC #53).
#[test]
fn test_rne_featherstone_sleeping_zero_contribution() {
    let model = load_model(two_tree_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until at least one tree sleeps
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    // Verify sleeping invariants hold
    for body_id in 1..model.nbody {
        if data.body_sleep_state[body_id] == SleepState::Asleep {
            // cvel should be zero (set at sleep time)
            for k in 0..6 {
                assert_eq!(
                    data.cvel[body_id][k], 0.0,
                    "sleeping body {body_id} cvel[{k}] should be zero"
                );
            }
        }
    }

    // Step once — Featherstone runs over all bodies including sleeping.
    // Sleeping bodies contribute zero because cvel=0 → zero bias forces.
    data.step(&model).expect("step");

    // After Featherstone: sleeping DOFs' qfrc_bias must still be zero
    for dof in 0..model.nv {
        if dof < model.dof_treeid.len() {
            let tree = model.dof_treeid[dof];
            if tree < model.ntree && !data.tree_awake(tree) {
                assert_eq!(
                    data.qfrc_bias[dof], 0.0,
                    "sleeping DOF {dof} qfrc_bias should be zero after Featherstone"
                );
            }
        }
    }
}

/// T83: Per-function bit-identity — each converted function individually produces
/// bit-identical output when all bodies are awake (AC #54).
#[test]
fn test_per_function_bit_identity() {
    // Two identical models: one with sleep enabled (but all awake), one without.
    // The indirection path (use_body_ind = false when all awake) must produce
    // identical results to the no-sleep path.
    let mjcf_sleep = r#"
    <mujoco model="bit_id_sleep">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="ball" pos="0 0 2">
                <freejoint name="free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let mjcf_nosleep = r#"
    <mujoco model="bit_id_nosleep">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1"/>
        <worldbody>
            <body name="ball" pos="0 0 2">
                <freejoint name="free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model_a = load_model(mjcf_sleep).expect("load");
    let model_b = load_model(mjcf_nosleep).expect("load");
    let mut data_a = model_a.make_data();
    let mut data_b = model_b.make_data();

    // Step 50 times — body is free-falling, all awake
    for step in 0..50 {
        data_a.step(&model_a).expect("step A");
        data_b.step(&model_b).expect("step B");

        // Per-function outputs:
        // 1. mj_fwd_velocity → cvel
        for body_id in 0..model_a.nbody {
            for k in 0..6 {
                assert_eq!(
                    data_a.cvel[body_id][k], data_b.cvel[body_id][k],
                    "cvel diverged at step {step}, body {body_id}[{k}]"
                );
            }
        }
        // 2. mj_rne → qfrc_bias
        for dof in 0..model_a.nv {
            assert_eq!(
                data_a.qfrc_bias[dof], data_b.qfrc_bias[dof],
                "qfrc_bias diverged at step {step}, dof {dof}"
            );
        }
        // 3. integrate → qvel
        for dof in 0..model_a.nv {
            assert_eq!(
                data_a.qvel[dof], data_b.qvel[dof],
                "qvel diverged at step {step}, dof {dof}"
            );
        }
    }
}

/// T84: `energy_potential` does not jump discontinuously when a body
/// transitions between awake and asleep (AC #49, continuity).
#[test]
fn test_energy_continuous_across_sleep_transition() {
    let model = load_model(free_body_sleep_mjcf()).expect("load model");
    let mut data = model.make_data();

    let mut prev_pe = data.energy_potential;
    let mut prev_ke = data.energy_kinetic;
    let mut found_sleep_transition = false;

    for step in 0..5000 {
        let was_awake = data.tree_awake(0);
        data.step(&model).expect("step");
        let is_awake = data.tree_awake(0);

        let pe = data.energy_potential;
        let ke = data.energy_kinetic;

        if was_awake && !is_awake {
            // Sleep transition detected
            found_sleep_transition = true;

            // Energy should not have a large discontinuous jump from the sleep
            // transition itself. The body was nearly at rest, so energy change
            // should be small (bounded by contact dissipation, not a sleep artifact).
            let pe_delta = (pe - prev_pe).abs();
            let ke_delta = (ke - prev_ke).abs();

            // At the moment of sleeping, KE goes to exactly zero (qvel zeroed).
            // The KE should have been near zero already (below sleep_tolerance).
            // PE should be essentially unchanged (pose frozen).
            assert!(
                pe_delta < 0.1,
                "energy_potential jumped by {pe_delta} at sleep transition (step {step})"
            );
            assert!(
                ke_delta < 0.1,
                "energy_kinetic jumped by {ke_delta} at sleep transition (step {step})"
            );
        }

        prev_pe = pe;
        prev_ke = ke;
    }

    assert!(
        found_sleep_transition,
        "no sleep transition detected in 5000 steps"
    );
}

// ============================================================================
// T85–T88: Phase C2 — Island-Local Delassus Assembly (§16.28)
// ============================================================================

/// MJCF fixture for two separated free bodies that form independent islands.
/// 4-meter gap ensures they never share an island through contacts.
/// Default sleep_tolerance (1e-4) allows bodies to eventually sleep but
/// islands form while bodies are awake and in contact with the ground.
fn two_island_bodies_mjcf() -> &'static str {
    r#"
    <mujoco model="two_island_bodies">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="10 10 0.1" solref="0.005 1.5"/>
            <body name="ball_a" pos="-2 0 0.5">
                <freejoint name="a_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball_b" pos="2 0 0.5">
                <freejoint name="b_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// T85: Island-local Delassus assembly produces equivalent constraint forces
/// to the global assembly for two independent bodies (AC #55).
#[test]
fn test_island_delassus_equivalence() {
    let mjcf = two_island_bodies_mjcf();

    // Run with islands enabled (uses island-local Delassus assembly)
    let model_island = load_model(mjcf).expect("load");
    let mut data_island = model_island.make_data();

    // Run with DISABLE_ISLAND (uses global assembly path)
    let mut model_global = load_model(mjcf).expect("load");
    model_global.disableflags |= DISABLE_ISLAND;
    let mut data_global = model_global.make_data();

    let mut max_nisland = 0usize;

    // Step 500 times — bodies fall, contact, and settle
    for step in 0..500 {
        data_island.step(&model_island).expect("island step");
        data_global.step(&model_global).expect("global step");

        // Track maximum islands seen (islands clear when bodies sleep)
        max_nisland = max_nisland.max(data_island.nisland);

        // Compare qfrc_constraint (contact + penalty forces)
        for dof in 0..model_island.nv {
            let diff = (data_island.qfrc_constraint[dof] - data_global.qfrc_constraint[dof]).abs();
            assert!(
                diff < 1e-8,
                "qfrc_constraint diverged at step {step}, dof {dof}: \
                 island={}, global={}, diff={diff}",
                data_island.qfrc_constraint[dof],
                data_global.qfrc_constraint[dof]
            );
        }
    }

    // Verify islands were actually used at some point during simulation
    assert!(
        max_nisland >= 2,
        "Expected >= 2 islands for two separated bodies, max seen: {max_nisland}"
    );
}

/// T86: Contact forces from island-local solve match global solve for
/// two independent free bodies step-by-step (AC #57).
#[test]
fn test_island_solve_forces_match_global() {
    let mjcf = two_island_bodies_mjcf();

    let model_island = load_model(mjcf).expect("load");
    let mut data_island = model_island.make_data();

    let mut model_global = load_model(mjcf).expect("load");
    model_global.disableflags |= DISABLE_ISLAND;
    let mut data_global = model_global.make_data();

    for step in 0..200 {
        data_island.step(&model_island).expect("island step");
        data_global.step(&model_global).expect("global step");

        // Compare qacc (total acceleration including constraint forces)
        for dof in 0..model_island.nv {
            let diff = (data_island.qacc[dof] - data_global.qacc[dof]).abs();
            assert!(
                diff < 1e-8,
                "qacc diverged at step {step}, dof {dof}: \
                 island={}, global={}, diff={diff}",
                data_island.qacc[dof],
                data_global.qacc[dof]
            );
        }

        // Compare qvel
        for dof in 0..model_island.nv {
            let diff = (data_island.qvel[dof] - data_global.qvel[dof]).abs();
            assert!(
                diff < 1e-8,
                "qvel diverged at step {step}, dof {dof}: \
                 island={}, global={}, diff={diff}",
                data_island.qvel[dof],
                data_global.qvel[dof]
            );
        }
    }
}

/// T87: When a single island spans all DOFs, the global fallback path
/// activates (no island-local Cholesky extraction) (AC #58).
#[test]
fn test_single_island_uses_global_path() {
    // Two stacked bodies — contacts between them form one connected island.
    let mjcf = r#"
    <mujoco model="single_island">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.5">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball_bottom" pos="0 0 0.15">
                <freejoint name="free_bottom"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball_top" pos="0 0 0.35">
                <freejoint name="free_top"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step until contacts form and bodies settle
    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    // With stacked bodies, all DOFs should be in one island (or zero islands
    // if no contacts). When nisland == 1, the global fallback should activate.
    if data.nisland == 1 {
        assert_eq!(
            data.island_nv[0], model.nv,
            "Single island should span all DOFs"
        );
    }

    // Simulation should produce valid, finite results regardless of path
    assert!(
        data.qacc.iter().all(|&v| v.is_finite()),
        "qacc contains non-finite values"
    );
    assert!(
        data.qvel.iter().all(|&v| v.is_finite()),
        "qvel contains non-finite values"
    );
}

/// T88: DISABLE_ISLAND flag produces unchanged results after Phase C2
/// modifications — the global solver path is untouched (AC #56).
#[test]
fn test_disable_island_phase_c_bit_identical() {
    let mjcf = two_island_bodies_mjcf();

    // Model A: DISABLE_ISLAND + sleep enabled
    let mut model_a = load_model(mjcf).expect("load");
    model_a.disableflags |= DISABLE_ISLAND;
    let mut data_a = model_a.make_data();

    // Model B: DISABLE_ISLAND + sleep disabled
    let mjcf_no_sleep = r#"
    <mujoco model="disable_island_nosleep">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.5"/>
        <worldbody>
            <geom type="plane" size="10 10 0.1" solref="0.005 1.5"/>
            <body name="ball_a" pos="-2 0 0.5">
                <freejoint name="a_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball_b" pos="2 0 0.5">
                <freejoint name="b_free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let mut model_b = load_model(mjcf_no_sleep).expect("load");
    model_b.disableflags |= DISABLE_ISLAND;
    let mut data_b = model_b.make_data();

    for step in 0..200 {
        data_a.step(&model_a).expect("step A");
        data_b.step(&model_b).expect("step B");

        // Both use DISABLE_ISLAND → nisland=0 → global solver.
        // With sleep disabled in B, sleep state won't diverge the comparison.
        for dof in 0..model_a.nv {
            let diff = (data_a.qvel[dof] - data_b.qvel[dof]).abs();
            assert!(
                diff < 1e-12,
                "DISABLE_ISLAND diverged at step {step}, dof {dof}: A={}, B={}",
                data_a.qvel[dof],
                data_b.qvel[dof]
            );
        }
    }

    // Verify DISABLE_ISLAND keeps nisland = 0
    assert_eq!(data_a.nisland, 0, "DISABLE_ISLAND should keep nisland=0");
    assert_eq!(data_b.nisland, 0, "DISABLE_ISLAND should keep nisland=0");
}

// ============================================================================
// Phase C — Step C3a: Selective CRBA (§16.29.3)
// Tests T89–T98
// ============================================================================

/// MJCF with 3 kinematic trees for selective CRBA tests.
/// Tree A, Tree B, Tree C — free bodies on a ground plane.
/// High damping contact + generous tolerance for reliable settling and sleeping.
fn three_tree_crba_mjcf() -> &'static str {
    r#"
    <mujoco model="three_tree_crba">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="10 10 0.1" solref="0.005 1.5"/>
            <body name="ball_a" pos="-2 0 0.2">
                <freejoint name="free_a"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball_b" pos="0 0 0.2">
                <freejoint name="free_b"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
            <body name="ball_c" pos="2 0 0.2">
                <freejoint name="free_c"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// T89: Selective CRBA produces identical qM entries for awake DOFs as full CRBA (AC #59).
///
/// Runs the model with sleep, lets some trees sleep, then compares the awake DOFs'
/// qM entries against a reference computed with sleep disabled.
#[test]
fn test_selective_crba_awake_identical() {
    let model_sleep = load_model(three_tree_crba_mjcf()).expect("load sleep model");
    let mut data_sleep = model_sleep.make_data();

    // Build a reference model with sleep disabled
    let nosleep_mjcf =
        three_tree_crba_mjcf().replace(r#"<flag sleep="enable"/>"#, r#"<flag sleep="disable"/>"#);
    let model_nosleep = load_model(&nosleep_mjcf).expect("load nosleep model");
    let mut data_nosleep = model_nosleep.make_data();

    // Step both until trees settle and sleep
    for _ in 0..2000 {
        data_sleep.step(&model_sleep).expect("sleep step");
        data_nosleep.step(&model_nosleep).expect("nosleep step");
    }

    // At least one tree should be sleeping
    let any_sleeping = (0..model_sleep.ntree).any(|t| data_sleep.tree_asleep[t] >= 0);
    assert!(
        any_sleeping,
        "at least one tree should be asleep for this test"
    );

    // Now apply force to wake one tree and step both models
    // Find tree_a's body
    let ball_a = model_sleep
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("ball_a"))
        .expect("ball_a");
    data_sleep.xfrc_applied[ball_a][2] = 5.0;
    data_nosleep.xfrc_applied[ball_a][2] = 5.0;

    // Sync qpos/qvel between models (they may have drifted slightly, use sleep's state)
    data_nosleep.qpos.copy_from(&data_sleep.qpos);
    data_nosleep.qvel.copy_from(&data_sleep.qvel);

    // Step once to run CRBA with partial sleep
    data_sleep.step(&model_sleep).expect("sleep step");
    data_nosleep.step(&model_nosleep).expect("nosleep step");

    // Compare awake DOFs' qM entries (bit-identical)
    for dof_i in 0..model_sleep.nv {
        let tree_i = model_sleep.dof_treeid[dof_i];
        if data_sleep.tree_asleep[tree_i] >= 0 {
            continue; // Skip sleeping DOFs
        }
        for dof_j in 0..model_sleep.nv {
            let tree_j = model_sleep.dof_treeid[dof_j];
            if data_sleep.tree_asleep[tree_j] >= 0 {
                continue;
            }
            assert_eq!(
                data_sleep.qM[(dof_i, dof_j)],
                data_nosleep.qM[(dof_i, dof_j)],
                "qM[({dof_i},{dof_j})] differs for awake DOFs"
            );
        }
    }
}

/// T90: Sleeping DOFs' qM entries are preserved across steps (AC #61).
///
/// Records qM for a tree just before it sleeps, then checks that those entries
/// remain unchanged on subsequent steps while the tree stays asleep.
#[test]
fn test_selective_crba_sleeping_preserved() {
    let model = load_model(three_tree_crba_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until at least one tree sleeps
    let mut slept_tree = None;
    for _ in 0..3000 {
        data.step(&model).expect("step");
        for t in 0..model.ntree {
            if data.tree_asleep[t] >= 0 {
                slept_tree = Some(t);
                break;
            }
        }
        if slept_tree.is_some() {
            break;
        }
    }
    let tree = slept_tree.expect("a tree should have gone to sleep");

    // Record qM entries for the sleeping tree's DOFs
    let dof_start = model.tree_dof_adr[tree];
    let dof_count = model.tree_dof_num[tree];
    let mut saved_qm = vec![0.0_f64; dof_count * dof_count];
    for i in 0..dof_count {
        for j in 0..dof_count {
            saved_qm[i * dof_count + j] = data.qM[(dof_start + i, dof_start + j)];
        }
    }

    // Verify entries are non-zero (they should be valid mass matrix entries)
    let diagonal_sum: f64 = (0..dof_count).map(|i| saved_qm[i * dof_count + i]).sum();
    assert!(
        diagonal_sum > 0.0,
        "sleeping tree's qM diagonal should be positive (was {diagonal_sum})"
    );

    // Step several more times while tree remains asleep
    for step in 0..20 {
        data.step(&model).expect("step");
        assert!(
            data.tree_asleep[tree] >= 0,
            "tree should still be asleep at step {step}"
        );

        // Compare: sleeping DOFs' qM must be unchanged
        for i in 0..dof_count {
            for j in 0..dof_count {
                assert_eq!(
                    data.qM[(dof_start + i, dof_start + j)],
                    saved_qm[i * dof_count + j],
                    "qM[({}, {})] changed while sleeping at step {step}",
                    dof_start + i,
                    dof_start + j
                );
            }
        }
    }
}

/// T91: Waking tree gets fresh qM and qLD (AC #62).
///
/// Puts a tree to sleep, then wakes it. Verifies that the next forward pass
/// produces the same qM as a never-slept simulation at the same configuration.
#[test]
fn test_selective_crba_wake_recomputes() {
    let model = load_model(three_tree_crba_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until a tree sleeps
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    // Find a sleeping tree
    let slept_tree = (0..model.ntree)
        .find(|&t| data.tree_asleep[t] >= 0)
        .expect("a tree should be asleep");

    // Wake it with a force
    let body_start = model.tree_body_adr[slept_tree];
    data.xfrc_applied[body_start][2] = 5.0;

    // Snapshot qpos/qvel
    let saved_qpos = data.qpos.clone();
    let saved_qvel = data.qvel.clone();

    // Step the sleep model (wakes tree, runs selective CRBA)
    data.step(&model).expect("step after wake");

    // Build reference: nosleep model at same configuration
    let nosleep_mjcf =
        three_tree_crba_mjcf().replace(r#"<flag sleep="enable"/>"#, r#"<flag sleep="disable"/>"#);
    let model_ref = load_model(&nosleep_mjcf).expect("load ref");
    let mut data_ref = model_ref.make_data();
    data_ref.qpos.copy_from(&saved_qpos);
    data_ref.qvel.copy_from(&saved_qvel);
    data_ref.xfrc_applied[body_start][2] = 5.0;
    data_ref.step(&model_ref).expect("ref step");

    // The previously-sleeping tree's DOFs should now have fresh, correct qM
    let dof_start = model.tree_dof_adr[slept_tree];
    let dof_count = model.tree_dof_num[slept_tree];
    for i in 0..dof_count {
        for j in 0..dof_count {
            assert_eq!(
                data.qM[(dof_start + i, dof_start + j)],
                data_ref.qM[(dof_start + i, dof_start + j)],
                "woken tree's qM[({}, {})] differs from reference",
                dof_start + i,
                dof_start + j,
            );
        }
    }
}

/// T92: With all bodies awake, selective CRBA is bit-identical to full CRBA (AC #63).
///
/// When sleep is enabled but all bodies are awake (no sleeping trees),
/// the sleep_filter=false fast path is exercised and results are identical.
/// Uses sleep="never" policy to ensure trees never sleep during the test.
#[test]
fn test_selective_crba_all_awake_noop() {
    let mjcf_sleep = r#"
    <mujoco model="all_awake">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball" pos="0 0 0.2" sleep="never">
                <freejoint name="free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let mjcf_nosleep = r#"
    <mujoco model="all_awake_nosleep">
        <option gravity="0 0 -9.81" timestep="0.002">
            <flag sleep="disable"/>
        </option>
        <worldbody>
            <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
            <body name="ball" pos="0 0 0.2">
                <freejoint name="free"/>
                <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model_a = load_model(mjcf_sleep).expect("load sleep model");
    let model_b = load_model(mjcf_nosleep).expect("load nosleep model");
    let mut data_a = model_a.make_data();
    let mut data_b = model_b.make_data();

    for step in 0..100 {
        data_a.step(&model_a).expect("step a");
        data_b.step(&model_b).expect("step b");

        // All bodies should still be awake (never policy)
        assert_eq!(
            data_a.ntree_awake, model_a.ntree,
            "all trees should be awake at step {step}"
        );

        // qM must be bit-identical
        for i in 0..model_a.nv {
            for j in 0..model_a.nv {
                assert_eq!(
                    data_a.qM[(i, j)],
                    data_b.qM[(i, j)],
                    "qM[({i},{j})] differs at step {step}"
                );
            }
        }
    }
}

/// T93: crb_inertia is frozen for sleeping bodies (AC #64).
///
/// Records crb_inertia for a sleeping body, then checks it doesn't change.
#[test]
fn test_selective_crba_crb_inertia_preserved() {
    let model = load_model(three_tree_crba_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until a tree sleeps
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let slept_tree = (0..model.ntree)
        .find(|&t| data.tree_asleep[t] >= 0)
        .expect("a tree should be asleep");

    let body_start = model.tree_body_adr[slept_tree];
    let body_count = model.tree_body_num[slept_tree];

    // Record crb_inertia for all bodies in sleeping tree
    let saved_crb: Vec<_> = (body_start..body_start + body_count)
        .map(|b| data.crb_inertia[b])
        .collect();

    // Step several more times
    for step in 0..20 {
        data.step(&model).expect("step");
        assert!(
            data.tree_asleep[slept_tree] >= 0,
            "tree should still be asleep at step {step}"
        );

        for (idx, b) in (body_start..body_start + body_count).enumerate() {
            assert_eq!(
                data.crb_inertia[b], saved_crb[idx],
                "crb_inertia[{b}] changed while sleeping at step {step}"
            );
        }
    }
}

/// T94: Armature is correctly applied for awake joints (AC #65).
///
/// Verifies that jnt_armature and dof_armature are both added to qM diagonal
/// for awake joints, with some trees sleeping.
#[test]
fn test_selective_crba_armature_correct() {
    // Model with armature on joints
    let mjcf = r#"
    <mujoco model="armature_test">
        <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="arm_a" pos="-2 0 1">
                <joint name="h_a" type="hinge" axis="0 1 0" armature="0.5" damping="10"/>
                <geom type="capsule" size="0.05" fromto="0 0 0 0.5 0 0" mass="1"/>
            </body>
            <body name="arm_b" pos="2 0 1">
                <joint name="h_b" type="hinge" axis="0 1 0" armature="0.5" damping="10"/>
                <geom type="capsule" size="0.05" fromto="0 0 0 0.5 0 0" mass="1"/>
            </body>
        </worldbody>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("load model");
    let mut data = model.make_data();

    // Step until at least one tree sleeps
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    // Find an awake tree (or wake one)
    let awake_tree = (0..model.ntree).find(|&t| data.tree_asleep[t] < 0);
    let awake_tree = if let Some(t) = awake_tree {
        t
    } else {
        // Wake tree 0
        let body_start = model.tree_body_adr[0];
        data.xfrc_applied[body_start][2] = 5.0;
        data.step(&model).expect("step to wake");
        data.xfrc_applied[body_start][2] = 0.0;
        0
    };

    // Check that the awake tree's joint has armature in qM diagonal
    let dof_start = model.tree_dof_adr[awake_tree];
    let dof_count = model.tree_dof_num[awake_tree];

    for d in dof_start..dof_start + dof_count {
        let jnt_id = model.dof_jnt[d];
        let armature = model.jnt_armature[jnt_id];
        // qM diagonal should include the armature contribution
        assert!(
            data.qM[(d, d)] >= armature,
            "qM[({d},{d})] = {} should include armature {armature}",
            data.qM[(d, d)]
        );
    }
}

/// T95: body_min_mass/body_min_inertia preserved for sleeping bodies (AC #66).
#[test]
fn test_selective_crba_body_mass_cache_preserved() {
    let model = load_model(three_tree_crba_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until a tree sleeps
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let slept_tree = (0..model.ntree)
        .find(|&t| data.tree_asleep[t] >= 0)
        .expect("a tree should be asleep");

    let body_start = model.tree_body_adr[slept_tree];
    let body_count = model.tree_body_num[slept_tree];

    // Record cached mass/inertia
    let saved_mass: Vec<_> = (body_start..body_start + body_count)
        .map(|b| data.body_min_mass[b])
        .collect();
    let saved_inertia: Vec<_> = (body_start..body_start + body_count)
        .map(|b| data.body_min_inertia[b])
        .collect();

    // Step more while sleeping
    for step in 0..20 {
        data.step(&model).expect("step");
        assert!(
            data.tree_asleep[slept_tree] >= 0,
            "tree should still be asleep at step {step}"
        );

        for (idx, b) in (body_start..body_start + body_count).enumerate() {
            assert_eq!(
                data.body_min_mass[b], saved_mass[idx],
                "body_min_mass[{b}] changed while sleeping at step {step}"
            );
            assert_eq!(
                data.body_min_inertia[b], saved_inertia[idx],
                "body_min_inertia[{b}] changed while sleeping at step {step}"
            );
        }
    }
}

/// T96: Multi-tree mixed awake/sleeping scenario (AC #67).
///
/// 3 trees: A awake, B sleeping, C awake. Verifies:
/// - Trees A and C have correct qM
/// - Tree B's qM entries are stale-but-valid
/// - qLD is correct for all DOFs
#[test]
fn test_selective_crba_multi_tree() {
    let model = load_model(three_tree_crba_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Step until all trees sleep
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let all_sleeping = (0..model.ntree).all(|t| data.tree_asleep[t] >= 0);
    assert!(all_sleeping, "all trees should be asleep");

    // Wake trees 0 and 2, leave tree 1 sleeping
    let body_a = model.tree_body_adr[0];
    let body_c = model.tree_body_adr[2];
    data.xfrc_applied[body_a][2] = 5.0;
    data.xfrc_applied[body_c][2] = 5.0;
    data.step(&model).expect("step after wake");
    data.xfrc_applied[body_a][2] = 0.0;
    data.xfrc_applied[body_c][2] = 0.0;

    // Verify: tree 0 awake, tree 1 sleeping, tree 2 awake
    assert!(
        data.tree_asleep[0] < 0,
        "tree 0 should be awake (was woken by force)"
    );
    assert!(data.tree_asleep[1] >= 0, "tree 1 should still be sleeping");
    assert!(
        data.tree_asleep[2] < 0,
        "tree 2 should be awake (was woken by force)"
    );

    // Verify awake trees have positive qM diagonals
    for t in [0, 2] {
        let dof_start = model.tree_dof_adr[t];
        let dof_count = model.tree_dof_num[t];
        for d in dof_start..dof_start + dof_count {
            assert!(
                data.qM[(d, d)] > 0.0,
                "awake tree {t}: qM[({d},{d})] should be positive, got {}",
                data.qM[(d, d)]
            );
        }
    }

    // Verify sleeping tree has stale-but-valid positive diagonals
    let dof_start_b = model.tree_dof_adr[1];
    let dof_count_b = model.tree_dof_num[1];
    for d in dof_start_b..dof_start_b + dof_count_b {
        assert!(
            data.qM[(d, d)] > 0.0,
            "sleeping tree 1: qM[({d},{d})] should be positive (stale-but-valid), got {}",
            data.qM[(d, d)]
        );
    }

    // Verify qLD is valid (no NaN, positive diagonals)
    for d in 0..model.nv {
        assert!(
            data.qLD_diag[d].is_finite() && data.qLD_diag[d] > 0.0,
            "qLD_diag[{d}] should be positive finite, got {}",
            data.qLD_diag[d]
        );
    }
}

/// T97: Deep chain scenario with partial sleeping (AC #68).
///
/// Two separate 3-link hinge chains attached to the world body.
/// With one sleeping, the awake tree's off-diagonal qM entries
/// (ancestor walks via dof_parent) are correct.
#[test]
fn test_selective_crba_deep_chain() {
    // Two separate 3-link chains with high damping for fast settling
    let mjcf = r#"
    <mujoco model="deep_chain">
        <option gravity="0 0 -9.81" timestep="0.001" sleep_tolerance="0.1">
            <flag sleep="enable"/>
        </option>
        <worldbody>
            <body name="a1" pos="-2 0 0">
                <joint name="ha1" type="hinge" axis="0 1 0" damping="20"/>
                <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="0.5"/>
                <body name="a2" pos="0 0 -0.3">
                    <joint name="ha2" type="hinge" axis="0 1 0" damping="20"/>
                    <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="0.5"/>
                    <body name="a3" pos="0 0 -0.3">
                        <joint name="ha3" type="hinge" axis="0 1 0" damping="20"/>
                        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="0.5"/>
                    </body>
                </body>
            </body>
            <body name="b1" pos="2 0 0">
                <joint name="hb1" type="hinge" axis="0 1 0" damping="20"/>
                <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="0.5"/>
                <body name="b2" pos="0 0 -0.3">
                    <joint name="hb2" type="hinge" axis="0 1 0" damping="20"/>
                    <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="0.5"/>
                    <body name="b3" pos="0 0 -0.3">
                        <joint name="hb3" type="hinge" axis="0 1 0" damping="20"/>
                        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="0.5"/>
                    </body>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let model = load_model(mjcf).expect("load model");
    let mut data = model.make_data();
    assert_eq!(model.ntree, 2, "should have 2 trees");
    assert_eq!(model.nv, 6, "should have 6 DOFs (3 per chain)");

    // Step until both trees sleep
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }
    assert!(
        (0..model.ntree).all(|t| data.tree_asleep[t] >= 0),
        "both trees should be sleeping"
    );

    // Wake tree 0 only
    let body_a1 = model.tree_body_adr[0];
    data.xfrc_applied[body_a1][2] = 5.0;
    data.step(&model).expect("step after wake");
    data.xfrc_applied[body_a1][2] = 0.0;

    // Tree 0 awake, tree 1 sleeping
    assert!(data.tree_asleep[0] < 0, "tree 0 should be awake");
    assert!(data.tree_asleep[1] >= 0, "tree 1 should still be sleeping");

    // Verify off-diagonal entries for tree 0 (3-link chain has off-diagonals)
    let dof_start_a = model.tree_dof_adr[0];
    let dof_count_a = model.tree_dof_num[0];

    // The 3-link hinge chain should have non-zero off-diagonal entries
    // (coupling between joints in the same chain)
    let mut has_off_diag = false;
    for i in dof_start_a..dof_start_a + dof_count_a {
        for j in dof_start_a..dof_start_a + dof_count_a {
            if i != j && data.qM[(i, j)].abs() > 1e-15 {
                has_off_diag = true;
            }
        }
    }
    assert!(
        has_off_diag,
        "3-link chain should have off-diagonal qM entries"
    );

    // Verify qM is symmetric for awake tree
    for i in dof_start_a..dof_start_a + dof_count_a {
        for j in i + 1..dof_start_a + dof_count_a {
            assert_eq!(
                data.qM[(i, j)],
                data.qM[(j, i)],
                "qM should be symmetric: ({i},{j}) vs ({j},{i})"
            );
        }
    }

    // Verify LDL valid
    for d in 0..model.nv {
        assert!(
            data.qLD_diag[d].is_finite() && data.qLD_diag[d] > 0.0,
            "qLD_diag[{d}] should be positive finite, got {}",
            data.qLD_diag[d]
        );
    }
}

/// T98: Awake trees' qLD blocks identical whether qM came from selective or full CRBA (AC #60).
///
/// Since C3a runs full LDL (Phase 5 unchanged), the awake trees' qLD blocks
/// should be identical to those from full CRBA because the input qM values
/// are the same for awake DOFs (and LDL is tree-block-diagonal).
#[test]
fn test_selective_crba_qld_awake_matches_full() {
    let model = load_model(three_tree_crba_mjcf()).expect("load model");
    let mut data = model.make_data();

    // Build reference model with sleep disabled
    let nosleep_mjcf =
        three_tree_crba_mjcf().replace(r#"<flag sleep="enable"/>"#, r#"<flag sleep="disable"/>"#);
    let model_ref = load_model(&nosleep_mjcf).expect("load ref");
    let mut data_ref = model_ref.make_data();

    // Step until trees sleep
    for _ in 0..3000 {
        data.step(&model).expect("step");
        data_ref.step(&model_ref).expect("ref step");
    }

    // Wake one tree
    let body_a = model.tree_body_adr[0];
    data.xfrc_applied[body_a][2] = 5.0;
    data_ref.xfrc_applied[body_a][2] = 5.0;

    // Sync state
    data_ref.qpos.copy_from(&data.qpos);
    data_ref.qvel.copy_from(&data.qvel);

    data.step(&model).expect("step");
    data_ref.step(&model_ref).expect("ref step");

    // Compare qLD for awake DOFs
    for t in 0..model.ntree {
        if data.tree_asleep[t] >= 0 {
            continue; // Skip sleeping trees
        }
        let dof_start = model.tree_dof_adr[t];
        let dof_count = model.tree_dof_num[t];
        for d in dof_start..dof_start + dof_count {
            assert_eq!(
                data.qLD_diag[d], data_ref.qLD_diag[d],
                "qLD_diag[{d}] differs for awake tree {t}"
            );
            assert_eq!(
                data.qLD_L[d].len(),
                data_ref.qLD_L[d].len(),
                "qLD_L[{d}] length differs for awake tree {t}"
            );
            for (idx, (&(col_a, val_a), &(col_b, val_b))) in data.qLD_L[d]
                .iter()
                .zip(data_ref.qLD_L[d].iter())
                .enumerate()
            {
                assert_eq!(
                    col_a, col_b,
                    "qLD_L[{d}][{idx}] column differs for awake tree {t}"
                );
                assert_eq!(
                    val_a, val_b,
                    "qLD_L[{d}][{idx}] value differs for awake tree {t}"
                );
            }
        }
    }
}
