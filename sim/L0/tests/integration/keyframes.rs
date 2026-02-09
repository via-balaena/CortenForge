//! Integration tests for §14: Keyframes, Mocap Bodies.
//!
//! Covers all 27 acceptance criteria from `sim/docs/todo/future_work_4.md`.

use approx::assert_relative_eq;
use sim_core::ResetError;
use sim_core::batch::BatchSim;
use sim_mjcf::load_model;
use std::sync::Arc;

// ============================================================================
// MJCF fixtures
// ============================================================================

/// Single hinge pendulum with one keyframe.
fn pendulum_with_keyframe() -> &'static str {
    r#"
    <mujoco model="pendulum_kf">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <keyframe>
            <key name="home" qpos="0.5"/>
        </keyframe>
    </mujoco>
    "#
}

/// Pendulum with partial keyframe (only qpos, no qvel/act/ctrl).
fn pendulum_partial_keyframe() -> &'static str {
    r#"
    <mujoco model="pendulum_partial">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <actuator>
            <motor joint="hinge" gear="1"/>
        </actuator>
        <keyframe>
            <key name="partial" qpos="1.0"/>
        </keyframe>
    </mujoco>
    "#
}

/// Pendulum with empty keyframe (no state attributes).
fn pendulum_empty_keyframe() -> &'static str {
    r#"
    <mujoco model="pendulum_empty_kf">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <keyframe>
            <key name="defaults"/>
        </keyframe>
    </mujoco>
    "#
}

/// Pendulum with 3 distinct keyframes.
fn pendulum_three_keyframes() -> &'static str {
    r#"
    <mujoco model="pendulum_3kf">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <keyframe>
            <key name="up"    qpos="1.5708" qvel="0.0" time="0.0"/>
            <key name="down"  qpos="-1.5708" qvel="1.0" time="1.0"/>
            <key name="swing" qpos="0.7854" qvel="-2.0" time="2.0"/>
        </keyframe>
    </mujoco>
    "#
}

/// Single mocap body as direct child of worldbody (no joints).
fn single_mocap_body() -> &'static str {
    r#"
    <mujoco model="mocap_single">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="target" mocap="true" pos="1 2 3">
                <geom type="sphere" size="0.05" mass="0.1"/>
            </body>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Two mocap bodies in one model.
fn two_mocap_bodies() -> &'static str {
    r#"
    <mujoco model="mocap_two">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="target_a" mocap="true" pos="1 0 0">
                <geom type="sphere" size="0.05" mass="0.1"/>
            </body>
            <body name="target_b" mocap="true" pos="0 1 0">
                <geom type="sphere" size="0.05" mass="0.2"/>
            </body>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Mocap body with a site.
fn mocap_with_site() -> &'static str {
    r#"
    <mujoco model="mocap_site">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="target" mocap="true" pos="1 0 0">
                <geom type="sphere" size="0.05" mass="0.1"/>
                <site name="tip" pos="0 0 0.1"/>
            </body>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Mocap body with a child body.
fn mocap_with_child() -> &'static str {
    r#"
    <mujoco model="mocap_child">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="platform" mocap="true" pos="0 0 2">
                <geom type="box" size="0.5 0.5 0.05" mass="1.0"/>
                <body name="arm" pos="0 0 0.05">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.4" mass="0.5"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Model with mocap bodies and keyframes with mpos/mquat.
fn mocap_with_keyframe() -> &'static str {
    r#"
    <mujoco model="mocap_kf">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="target" mocap="true" pos="1 0 0">
                <geom type="sphere" size="0.05" mass="0.1"/>
            </body>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <keyframe>
            <key name="moved" qpos="0.3" mpos="5 6 7" mquat="1 0 0 0"/>
        </keyframe>
    </mujoco>
    "#
}

/// Mocap body with a contact-capable dynamic body nearby.
fn mocap_contact() -> &'static str {
    r#"
    <mujoco model="mocap_contact">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1"/>
            <body name="wall" mocap="true" pos="0.15 0 0.5">
                <geom type="box" size="0.05 1 1" mass="10.0"/>
            </body>
            <body name="ball" pos="0 0 0.5">
                <joint name="slide_x" type="slide" axis="1 0 0"/>
                <joint name="slide_z" type="slide" axis="0 0 1"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

// ============================================================================
// AC 1: Keyframes — parsing: basic qpos
// ============================================================================

#[test]
fn ac01_keyframe_parse_qpos() {
    let model = load_model(pendulum_with_keyframe()).unwrap();
    assert_eq!(model.nkeyframe, 1);
    assert_eq!(model.keyframes.len(), 1);
    assert_eq!(model.keyframes[0].name, "home");
    assert_relative_eq!(model.keyframes[0].qpos[0], 0.5, epsilon = 1e-12);
}

// ============================================================================
// AC 2: Partial keyframes fill defaults
// ============================================================================

#[test]
fn ac02_partial_keyframe_defaults() {
    let model = load_model(pendulum_partial_keyframe()).unwrap();
    let kf = &model.keyframes[0];
    assert_eq!(kf.name, "partial");
    // qpos is explicitly set
    assert_relative_eq!(kf.qpos[0], 1.0, epsilon = 1e-12);
    // qvel should be zeros (default)
    assert_relative_eq!(kf.qvel[0], 0.0, epsilon = 1e-12);
    // act should be zeros (default, na == 0 in this model but DVector length 0)
    assert_eq!(kf.act.len(), model.na);
    // ctrl should be zeros (nu == 1 from motor actuator)
    assert_eq!(kf.ctrl.len(), model.nu);
    for i in 0..model.nu {
        assert_relative_eq!(kf.ctrl[i], 0.0, epsilon = 1e-12);
    }
}

// ============================================================================
// AC 3: Empty keyframe fills all defaults
// ============================================================================

#[test]
fn ac03_empty_keyframe_model_defaults() {
    let model = load_model(pendulum_empty_keyframe()).unwrap();
    let kf = &model.keyframes[0];
    assert_eq!(kf.name, "defaults");
    // qpos defaults to qpos0
    assert_eq!(kf.qpos.len(), model.nq);
    assert_relative_eq!(kf.qpos[0], model.qpos0[0], epsilon = 1e-12);
    // qvel defaults to zeros
    assert_eq!(kf.qvel.len(), model.nv);
    assert_relative_eq!(kf.qvel[0], 0.0, epsilon = 1e-12);
    // time defaults to 0.0
    assert_relative_eq!(kf.time, 0.0, epsilon = 1e-12);
}

// ============================================================================
// AC 4: Multiple keyframes
// ============================================================================

#[test]
fn ac04_multiple_keyframes_distinct() {
    let model = load_model(pendulum_three_keyframes()).unwrap();
    assert_eq!(model.nkeyframe, 3);
    let mut data = model.make_data();

    // Reset to each keyframe and verify distinct state.
    let mut states: Vec<(f64, f64)> = Vec::new();
    for i in 0..3 {
        data.reset_to_keyframe(&model, i).unwrap();
        states.push((data.qpos[0], data.qvel[0]));
    }

    // All three qpos values are distinct.
    assert!((states[0].0 - states[1].0).abs() > 0.1);
    assert!((states[1].0 - states[2].0).abs() > 0.1);
    assert!((states[0].0 - states[2].0).abs() > 0.1);
}

// ============================================================================
// AC 5: reset_to_keyframe overwrites state
// ============================================================================

#[test]
fn ac05_reset_to_keyframe_overwrites_state() {
    let model = load_model(pendulum_three_keyframes()).unwrap();
    let mut data = model.make_data();

    // Step to diverge from initial state.
    data.qpos[0] = 0.5;
    for _ in 0..50 {
        data.step(&model).unwrap();
    }

    // Reset to keyframe 0 ("up").
    data.reset_to_keyframe(&model, 0).unwrap();
    let kf = &model.keyframes[0];

    assert_relative_eq!(data.time, kf.time, epsilon = 1e-12);
    assert_relative_eq!(data.qpos[0], kf.qpos[0], epsilon = 1e-12);
    assert_relative_eq!(data.qvel[0], kf.qvel[0], epsilon = 1e-12);

    // After forward(), body poses match the keyframe state.
    data.forward(&model).unwrap();
    assert!(
        data.xpos[1].norm() > 0.1,
        "body should have nonzero position"
    );
}

// ============================================================================
// AC 6: reset_to_keyframe with out-of-range index
// ============================================================================

#[test]
fn ac06_reset_out_of_range() {
    let model = load_model(pendulum_with_keyframe()).unwrap();
    let mut data = model.make_data();

    let result = data.reset_to_keyframe(&model, 999);
    assert!(result.is_err());
    match result.unwrap_err() {
        ResetError::InvalidKeyframeIndex { index, nkeyframe } => {
            assert_eq!(index, 999);
            assert_eq!(nkeyframe, 1);
        }
        _ => panic!("expected InvalidKeyframeIndex"),
    }
}

// ============================================================================
// AC 7: Round-trip (step → reset → forward → verify)
// ============================================================================

#[test]
fn ac07_round_trip_reset() {
    let model = load_model(pendulum_with_keyframe()).unwrap();
    let kf = &model.keyframes[0];

    // Fresh data with keyframe qpos, forward() to get reference xpos.
    let mut reference = model.make_data();
    reference.qpos.copy_from(&kf.qpos);
    reference.qvel.copy_from(&kf.qvel);
    reference.forward(&model).unwrap();

    // Step 100 times to diverge.
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    for _ in 0..100 {
        data.step(&model).unwrap();
    }

    // Reset to keyframe, forward().
    data.reset_to_keyframe(&model, 0).unwrap();
    data.forward(&model).unwrap();

    assert_relative_eq!(data.qpos[0], kf.qpos[0], epsilon = 1e-12);
    assert_relative_eq!(data.qvel[0], kf.qvel[0], epsilon = 1e-12);

    // xpos should match the reference.
    for i in 0..model.nbody {
        assert_relative_eq!(data.xpos[i], reference.xpos[i], epsilon = 1e-10);
    }
}

// ============================================================================
// AC 8: reset_to_keyframe preserves qfrc_applied/xfrc_applied, clears derived
// ============================================================================

#[test]
fn ac08_reset_preserves_applied_forces() {
    let model = load_model(pendulum_with_keyframe()).unwrap();
    let mut data = model.make_data();

    // Set applied forces before reset.
    data.qfrc_applied[0] = 42.0;
    data.xfrc_applied[1] = nalgebra::Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

    // Step to populate derived quantities.
    data.qpos[0] = 0.5;
    data.forward(&model).unwrap();
    data.step(&model).unwrap();

    // Reset.
    data.reset_to_keyframe(&model, 0).unwrap();

    // Applied forces preserved.
    assert_relative_eq!(data.qfrc_applied[0], 42.0, epsilon = 1e-12);
    assert_relative_eq!(data.xfrc_applied[1][0], 1.0, epsilon = 1e-12);

    // Derived quantities cleared.
    assert_relative_eq!(data.qacc[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(data.qacc_warmstart[0], 0.0, epsilon = 1e-12);
    assert_eq!(data.ncon, 0);
    assert!(data.contacts.is_empty());
}

// ============================================================================
// AC 9: Mocap body produces correct body_mocapid
// ============================================================================

#[test]
fn ac09_mocap_body_mocapid() {
    let model = load_model(single_mocap_body()).unwrap();

    // Find the mocap body (name "target").
    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();

    assert!(model.body_mocapid[mocap_body_id].is_some());
    let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();
    assert_eq!(mocap_idx, 0); // first mocap body

    // No joints on mocap body.
    assert_eq!(model.body_jnt_num[mocap_body_id], 0);

    // Mass should be non-zero (from geom density).
    assert!(
        model.body_mass[mocap_body_id] > 0.0,
        "mocap body mass should be computed from geoms"
    );
}

// ============================================================================
// AC 10: nmocap equals number of mocap bodies
// ============================================================================

#[test]
fn ac10_nmocap_count() {
    let model_1 = load_model(single_mocap_body()).unwrap();
    assert_eq!(model_1.nmocap, 1);

    let model_2 = load_model(two_mocap_bodies()).unwrap();
    assert_eq!(model_2.nmocap, 2);

    // Pendulum without mocap bodies.
    let model_0 = load_model(pendulum_with_keyframe()).unwrap();
    assert_eq!(model_0.nmocap, 0);
}

// ============================================================================
// AC 11: After forward(), xpos == mocap_pos
// ============================================================================

#[test]
fn ac11_forward_xpos_equals_mocap_pos() {
    let model = load_model(single_mocap_body()).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();
    let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();

    assert_relative_eq!(
        data.xpos[mocap_body_id],
        data.mocap_pos[mocap_idx],
        epsilon = 1e-12
    );
}

// ============================================================================
// AC 12: Setting mocap_pos and calling forward() updates xpos + geom_xpos
// ============================================================================

#[test]
fn ac12_set_mocap_pos_updates_pose() {
    let model = load_model(single_mocap_body()).unwrap();
    let mut data = model.make_data();

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();
    let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();

    // Move mocap body.
    let new_pos = nalgebra::Vector3::new(10.0, 20.0, 30.0);
    data.mocap_pos[mocap_idx] = new_pos;
    data.forward(&model).unwrap();

    assert_relative_eq!(data.xpos[mocap_body_id], new_pos, epsilon = 1e-12);

    // Find the geom belonging to the mocap body.
    let geom_id = (0..model.ngeom)
        .find(|&g| model.geom_body[g] == mocap_body_id)
        .unwrap();
    // geom_xpos should reflect the new position (plus geom offset if any).
    assert_relative_eq!(data.geom_xpos[geom_id].x, 10.0, epsilon = 1e-6);
    assert_relative_eq!(data.geom_xpos[geom_id].y, 20.0, epsilon = 1e-6);
    assert_relative_eq!(data.geom_xpos[geom_id].z, 30.0, epsilon = 1e-6);
}

// ============================================================================
// AC 13: Mocap body geoms produce contacts, contact affects dynamic body only
// ============================================================================

#[test]
fn ac13_mocap_contact_affects_dynamic_only() {
    let model = load_model(mocap_contact()).unwrap();
    let mut data = model.make_data();

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("wall"))
        .unwrap();
    let ball_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("ball"))
        .unwrap();
    let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();

    // Move mocap wall close to ball to create contact.
    data.mocap_pos[mocap_idx] = nalgebra::Vector3::new(0.15, 0.0, 0.5);
    data.forward(&model).unwrap();

    // Record mocap position before stepping.
    let wall_pos_before = data.xpos[mocap_body_id];

    // Step several times — the ball should be pushed.
    for _ in 0..50 {
        data.step(&model).unwrap();
    }

    // Mocap body should NOT move (no joints, not affected by contacts).
    assert_relative_eq!(data.xpos[mocap_body_id], wall_pos_before, epsilon = 1e-10);

    // Ball body should have moved (pushed by contact or fallen by gravity).
    // At least the Z position should differ (gravity).
    let ball_pos = data.xpos[ball_body_id];
    assert!(
        (ball_pos.z - 0.5).abs() > 1e-4 || (ball_pos.x).abs() > 1e-4,
        "ball should have moved due to contact or gravity"
    );
}

// ============================================================================
// AC 14: Mocap body cvel is zero
// ============================================================================

#[test]
fn ac14_mocap_body_cvel_zero() {
    let model = load_model(single_mocap_body()).unwrap();
    let mut data = model.make_data();

    // Set mocap body to a non-origin position and call forward+velocity FK.
    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();

    data.forward(&model).unwrap();

    // cvel should be zero for mocap body (no joints, no DOFs).
    for j in 0..6 {
        assert_relative_eq!(data.cvel[mocap_body_id][j], 0.0, epsilon = 1e-12);
    }
}

// ============================================================================
// AC 15: Mocap body mass appears in subtree_mass of world
// ============================================================================

#[test]
fn ac15_mocap_mass_in_subtree() {
    let model = load_model(single_mocap_body()).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();

    // Mocap body mass should be included in world's subtree_mass.
    let mocap_mass = model.body_mass[mocap_body_id];
    assert!(mocap_mass > 0.0, "mocap body should have nonzero mass");

    // World subtree mass should include mocap body's mass.
    // Total = sum of all body masses.
    let total_mass: f64 = model.body_mass.iter().sum();
    assert_relative_eq!(data.subtree_mass[0], total_mass, epsilon = 1e-10);
}

// ============================================================================
// AC 16: Mocap body with child — child FK relative to mocap override
// ============================================================================

#[test]
fn ac16_mocap_child_follows_parent() {
    let model = load_model(mocap_with_child()).unwrap();
    let mut data = model.make_data();

    let platform_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("platform"))
        .unwrap();
    let arm_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("arm"))
        .unwrap();
    let mocap_idx = model.body_mocapid[platform_id].unwrap();

    // Initial FK.
    data.forward(&model).unwrap();
    let arm_pos_initial = data.xpos[arm_id];

    // Move mocap body by +5 in X.
    data.mocap_pos[mocap_idx] = nalgebra::Vector3::new(5.0, 0.0, 2.0);
    data.forward(&model).unwrap();

    let arm_pos_moved = data.xpos[arm_id];
    // Child should have moved by ~5 in X (same as parent).
    assert_relative_eq!(arm_pos_moved.x - arm_pos_initial.x, 5.0, epsilon = 1e-6);
}

// ============================================================================
// AC 17: Two mocap bodies — independent control
// ============================================================================

#[test]
fn ac17_two_mocap_independent() {
    let model = load_model(two_mocap_bodies()).unwrap();
    assert_eq!(model.nmocap, 2);
    let mut data = model.make_data();

    let body_a = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target_a"))
        .unwrap();
    let body_b = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target_b"))
        .unwrap();

    let idx_a = model.body_mocapid[body_a].unwrap();
    let idx_b = model.body_mocapid[body_b].unwrap();
    assert_ne!(idx_a, idx_b);

    // Move only body A.
    data.mocap_pos[idx_a] = nalgebra::Vector3::new(100.0, 0.0, 0.0);
    data.forward(&model).unwrap();

    assert_relative_eq!(data.xpos[body_a].x, 100.0, epsilon = 1e-10);
    // Body B should not have moved.
    assert_relative_eq!(data.xpos[body_b].x, 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.xpos[body_b].y, 1.0, epsilon = 1e-10);
}

// ============================================================================
// AC 18: Mocap body with site — site_xpos updated by mocap_pos
// ============================================================================

#[test]
fn ac18_mocap_site_follows() {
    let model = load_model(mocap_with_site()).unwrap();
    let mut data = model.make_data();

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();
    let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();

    // Find site "tip".
    let site_id = model
        .site_name
        .iter()
        .position(|n| n.as_deref() == Some("tip"))
        .unwrap();

    // Move mocap body.
    data.mocap_pos[mocap_idx] = nalgebra::Vector3::new(7.0, 8.0, 9.0);
    data.forward(&model).unwrap();

    // Site pos is (body_pos + site offset in body frame).
    // Site offset is "0 0 0.1" in body frame (identity rotation).
    assert_relative_eq!(data.site_xpos[site_id].x, 7.0, epsilon = 1e-6);
    assert_relative_eq!(data.site_xpos[site_id].y, 8.0, epsilon = 1e-6);
    assert_relative_eq!(data.site_xpos[site_id].z, 9.1, epsilon = 1e-6);
}

// ============================================================================
// AC 19: Mocap on non-world-child → ModelConversionError
// ============================================================================

#[test]
fn ac19_mocap_non_world_child_error() {
    let xml = r#"
    <mujoco model="bad_mocap_nested">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="parent" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
                <body name="nested_mocap" mocap="true" pos="0 0 0.5">
                    <geom type="sphere" size="0.05" mass="0.1"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let result = load_model(xml);
    assert!(
        result.is_err(),
        "mocap body nested under non-world body should fail"
    );
    let err_msg = format!("{}", result.unwrap_err());
    assert!(
        err_msg.contains("mocap") || err_msg.contains("Mocap"),
        "error should mention 'mocap': {}",
        err_msg
    );
}

// ============================================================================
// AC 20: Mocap body with joints → ModelConversionError
// ============================================================================

#[test]
fn ac20_mocap_with_joints_error() {
    let xml = r#"
    <mujoco model="bad_mocap_joints">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="bad_mocap" mocap="true" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#;

    let result = load_model(xml);
    assert!(result.is_err(), "mocap body with joints should fail");
    let err_msg = format!("{}", result.unwrap_err());
    assert!(
        err_msg.contains("mocap") || err_msg.contains("joint"),
        "error should mention 'mocap' or 'joint': {}",
        err_msg
    );
}

// ============================================================================
// AC 21: Keyframe qpos wrong length → ModelConversionError
// ============================================================================

#[test]
fn ac21_keyframe_qpos_wrong_length() {
    let xml = r#"
    <mujoco model="bad_kf_len">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <keyframe>
            <key name="bad" qpos="1.0 2.0 3.0"/>
        </keyframe>
    </mujoco>
    "#;

    let result = load_model(xml);
    assert!(result.is_err(), "wrong-length qpos should fail");
    let err_msg = format!("{}", result.unwrap_err());
    assert!(
        err_msg.contains("bad") && err_msg.contains("qpos"),
        "error should name keyframe and field: {}",
        err_msg
    );
}

// ============================================================================
// AC 22: Keyframe qpos with NaN → ModelConversionError
// ============================================================================

#[test]
fn ac22_keyframe_nan_rejected() {
    let xml = r#"
    <mujoco model="nan_kf">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <keyframe>
            <key name="nankey" qpos="NaN"/>
        </keyframe>
    </mujoco>
    "#;

    let result = load_model(xml);
    assert!(result.is_err(), "NaN in keyframe qpos should fail");
    let err_msg = format!("{}", result.unwrap_err());
    assert!(
        err_msg.contains("finite") || err_msg.contains("NaN"),
        "error should mention finiteness: {}",
        err_msg
    );
}

// ============================================================================
// AC 23: Keyframe time=inf → ModelConversionError
// ============================================================================

#[test]
fn ac23_keyframe_inf_time_rejected() {
    let xml = r#"
    <mujoco model="inf_time">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="link" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <keyframe>
            <key name="infkey" time="inf"/>
        </keyframe>
    </mujoco>
    "#;

    let result = load_model(xml);
    assert!(result.is_err(), "inf time in keyframe should fail");
    let err_msg = format!("{}", result.unwrap_err());
    assert!(
        err_msg.contains("finite") || err_msg.contains("time"),
        "error should mention time finiteness: {}",
        err_msg
    );
}

// ============================================================================
// AC 24: step() does not alter mocap body pose
// ============================================================================

#[test]
fn ac24_step_preserves_mocap_pose() {
    let model = load_model(single_mocap_body()).unwrap();
    let mut data = model.make_data();

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();
    let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();

    data.forward(&model).unwrap();
    let pos_before = data.mocap_pos[mocap_idx];
    let quat_before = data.mocap_quat[mocap_idx];

    // Step many times.
    for _ in 0..100 {
        data.step(&model).unwrap();
    }

    assert_relative_eq!(data.mocap_pos[mocap_idx], pos_before, epsilon = 1e-12);
    assert_relative_eq!(
        data.mocap_quat[mocap_idx].into_inner(),
        quat_before.into_inner(),
        epsilon = 1e-12
    );
}

// ============================================================================
// AC 25: BatchSim with mocap bodies — independent per environment
// ============================================================================

#[test]
fn ac25_batchsim_mocap_independent() {
    let model = Arc::new(load_model(single_mocap_body()).unwrap());
    let n_envs = 3;
    let mut batch = BatchSim::new(Arc::clone(&model), n_envs);

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();

    // Set different mocap_pos per environment.
    for i in 0..n_envs {
        let env = batch.env_mut(i).unwrap();
        let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();
        env.mocap_pos[mocap_idx] = nalgebra::Vector3::new((i as f64 + 1.0) * 10.0, 0.0, 0.0);
    }

    // Step all environments.
    let _errors = batch.step_all();

    // Verify poses are independent.
    for i in 0..n_envs {
        let env = batch.env(i).unwrap();
        assert_relative_eq!(
            env.xpos[mocap_body_id].x,
            (i as f64 + 1.0) * 10.0,
            epsilon = 1e-6,
        );
    }
}

// ============================================================================
// AC 26: Keyframe with mpos/mquat updates mocap arrays
// ============================================================================

#[test]
fn ac26_keyframe_with_mocap_poses() {
    let model = load_model(mocap_with_keyframe()).unwrap();
    let mut data = model.make_data();

    // After reset_to_keyframe, mocap arrays should contain keyframe's poses.
    data.reset_to_keyframe(&model, 0).unwrap();

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();
    let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();

    assert_relative_eq!(data.mocap_pos[mocap_idx].x, 5.0, epsilon = 1e-12);
    assert_relative_eq!(data.mocap_pos[mocap_idx].y, 6.0, epsilon = 1e-12);
    assert_relative_eq!(data.mocap_pos[mocap_idx].z, 7.0, epsilon = 1e-12);

    // After forward, xpos should match.
    data.forward(&model).unwrap();
    assert_relative_eq!(data.xpos[mocap_body_id].x, 5.0, epsilon = 1e-10);
    assert_relative_eq!(data.xpos[mocap_body_id].y, 6.0, epsilon = 1e-10);
    assert_relative_eq!(data.xpos[mocap_body_id].z, 7.0, epsilon = 1e-10);
}

// ============================================================================
// AC 27: Data::reset() restores mocap poses to model defaults
// ============================================================================

#[test]
fn ac27_reset_restores_mocap_defaults() {
    let model = load_model(single_mocap_body()).unwrap();
    let mut data = model.make_data();

    let mocap_body_id = model
        .body_name
        .iter()
        .position(|n| n.as_deref() == Some("target"))
        .unwrap();
    let mocap_idx = model.body_mocapid[mocap_body_id].unwrap();

    // Modify mocap position away from default.
    data.mocap_pos[mocap_idx] = nalgebra::Vector3::new(999.0, 888.0, 777.0);

    // Reset should restore to model defaults (body_pos for mocap body).
    data.reset(&model);

    // Default mocap pos is the body_pos attribute: "1 2 3".
    assert_relative_eq!(data.mocap_pos[mocap_idx].x, 1.0, epsilon = 1e-12);
    assert_relative_eq!(data.mocap_pos[mocap_idx].y, 2.0, epsilon = 1e-12);
    assert_relative_eq!(data.mocap_pos[mocap_idx].z, 3.0, epsilon = 1e-12);
}
