//! Integration tests for batched simulation (`BatchSim`).
//!
//! Tests end-to-end behavior with MJCF-loaded models, including models
//! with contacts (ground plane + geoms) to verify that parallel stepping
//! produces identical results to sequential stepping even when collision
//! detection and constraint solving are involved.

use sim_core::batch::BatchSim;
use sim_mjcf::load_model;
use std::sync::Arc;

/// Pendulum with ground plane contact: body can collide with the ground.
fn pendulum_with_contact_mjcf() -> &'static str {
    r#"
    <mujoco model="pendulum_contact">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
            <body name="pendulum" pos="0 0 1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
    </mujoco>
    "#
}

/// Two-body actuated model for testing ctrl input.
fn actuated_mjcf() -> &'static str {
    r#"
    <mujoco model="actuated">
        <option gravity="0 0 -9.81" timestep="0.002"/>
        <worldbody>
            <body name="arm" pos="0 0 0">
                <joint name="shoulder" type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
                <body name="forearm" pos="0 0 -0.5">
                    <joint name="elbow" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4" mass="0.5"/>
                </body>
            </body>
        </worldbody>
        <actuator>
            <motor joint="shoulder" gear="1"/>
            <motor joint="elbow" gear="1"/>
        </actuator>
    </mujoco>
    "#
}

/// Acceptance criterion 1: Bit-exact determinism with a contact model.
/// Batched stepping produces identical results to sequential stepping,
/// even when collision detection and constraint solving are involved.
#[test]
fn batch_matches_sequential_with_contacts() {
    let model = Arc::new(load_model(pendulum_with_contact_mjcf()).unwrap());
    let n = 4;

    let mut batch = BatchSim::new(Arc::clone(&model), n);
    let mut sequential: Vec<_> = (0..n).map(|_| model.make_data()).collect();

    // Set distinct initial angles so envs diverge
    for (i, seq) in sequential.iter_mut().enumerate() {
        let angle = (i as f64 + 1.0) * 0.5;
        batch.env_mut(i).unwrap().qpos[0] = angle;
        seq.qpos[0] = angle;
    }

    // Step both 50 times (enough for some envs to contact the ground)
    for _ in 0..50 {
        let _errors = batch.step_all();
        for data in &mut sequential {
            let _ = data.step(&model);
        }
    }

    for (i, s) in sequential.iter().enumerate() {
        let b = batch.env(i).unwrap();
        assert_eq!(b.qpos, s.qpos, "env {i} qpos mismatch");
        assert_eq!(b.qvel, s.qvel, "env {i} qvel mismatch");
        assert_eq!(b.qacc, s.qacc, "env {i} qacc mismatch");
        assert_eq!(b.time, s.time, "env {i} time mismatch");
        assert_eq!(b.ncon, s.ncon, "env {i} ncon mismatch");
        assert_eq!(
            b.contacts.len(),
            s.contacts.len(),
            "env {i} contacts count mismatch"
        );
        assert_eq!(b.sensordata, s.sensordata, "env {i} sensordata mismatch");
    }
}

/// Acceptance criterion 2: NaN auto-reset with MJCF model.
/// After §41 S8, NaN qpos triggers auto-reset instead of returning Err.
#[test]
fn nan_auto_reset_with_mjcf_model() {
    let model = Arc::new(load_model(pendulum_with_contact_mjcf()).unwrap());
    let mut batch = BatchSim::new(Arc::clone(&model), 4);

    // Env 1 gets NaN, others get valid states
    batch.env_mut(0).unwrap().qpos[0] = 0.3;
    batch.env_mut(1).unwrap().qpos[0] = f64::NAN;
    batch.env_mut(2).unwrap().qpos[0] = 0.7;
    batch.env_mut(3).unwrap().qpos[0] = 1.2;

    let errors = batch.step_all();

    // All envs succeed — NaN env auto-resets instead of erroring.
    for (i, e) in errors.iter().enumerate() {
        assert!(e.is_none(), "env {i} should succeed (got {e:?})");
    }

    // Env 1 should have detected divergence (auto-reset happened).
    assert!(
        batch.env(1).unwrap().divergence_detected(),
        "env 1 should auto-reset on NaN"
    );

    // Healthy envs should have advanced
    assert!(batch.env(0).unwrap().time > 0.0);
    assert!(batch.env(2).unwrap().time > 0.0);
    assert!(batch.env(3).unwrap().time > 0.0);
}

/// Test that per-env ctrl input works correctly with an actuated model.
#[test]
fn per_env_ctrl_produces_different_trajectories() {
    let model = Arc::new(load_model(actuated_mjcf()).unwrap());
    assert!(model.nu >= 2, "actuated model should have actuators");

    let mut batch = BatchSim::new(Arc::clone(&model), 3);

    // Give each env different control inputs
    batch.env_mut(0).unwrap().ctrl[0] = 0.0;
    batch.env_mut(1).unwrap().ctrl[0] = 5.0;
    batch.env_mut(2).unwrap().ctrl[0] = -5.0;

    // Step for a while
    for _ in 0..100 {
        let errors = batch.step_all();
        for (i, e) in errors.iter().enumerate() {
            assert!(e.is_none(), "env {i} should not fail");
        }
    }

    // All envs should have different states due to different controls
    let q0 = batch.env(0).unwrap().qpos[0];
    let q1 = batch.env(1).unwrap().qpos[0];
    let q2 = batch.env(2).unwrap().qpos[0];

    assert!(
        (q0 - q1).abs() > 1e-6,
        "env 0 and 1 should differ: {q0} vs {q1}"
    );
    assert!(
        (q1 - q2).abs() > 1e-6,
        "env 1 and 2 should differ: {q1} vs {q2}"
    );
}

/// Test reset after stepping with an MJCF model.
#[test]
fn reset_restores_initial_state_mjcf() {
    let model = Arc::new(load_model(pendulum_with_contact_mjcf()).unwrap());
    let mut batch = BatchSim::new(Arc::clone(&model), 2);

    // Step forward
    batch.env_mut(0).unwrap().qpos[0] = 1.5;
    batch.env_mut(1).unwrap().qpos[0] = 0.8;

    for _ in 0..20 {
        let _ = batch.step_all();
    }

    // Both should have advanced
    assert!(batch.env(0).unwrap().time > 0.0);
    assert!(batch.env(1).unwrap().time > 0.0);

    // Reset env 0 only
    batch.reset(0);

    assert_eq!(batch.env(0).unwrap().time, 0.0);
    assert_eq!(batch.env(0).unwrap().qpos, model.qpos0);
    assert!(batch.env(1).unwrap().time > 0.0, "env 1 untouched");

    // Env 0 should be steppable again after reset
    let errors = batch.step_all();
    assert!(errors[0].is_none(), "env 0 should step after reset");
}
