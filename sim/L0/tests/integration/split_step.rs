//! Split-step API tests (§53).
//!
//! Verifies that `step1()` + `step2()` produces identical results to `step()`
//! for Euler/Implicit integrators, and that force injection between the two
//! phases modifies the resulting trajectory.

use sim_core::{Integrator, Model};

/// Helper: run N steps with step(), return final qpos/qvel.
fn run_step(model: &Model, n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut data = model.make_data();
    data.qpos[0] = std::f64::consts::FRAC_PI_4;
    data.forward(model).unwrap();
    for _ in 0..n {
        data.step(model).unwrap();
    }
    (data.qpos.as_slice().to_vec(), data.qvel.as_slice().to_vec())
}

/// Helper: run N steps with step1()+step2(), return final qpos/qvel.
fn run_split(model: &Model, n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut data = model.make_data();
    data.qpos[0] = std::f64::consts::FRAC_PI_4;
    data.forward(model).unwrap();
    for _ in 0..n {
        data.step1(model).unwrap();
        data.step2(model);
    }
    (data.qpos.as_slice().to_vec(), data.qvel.as_slice().to_vec())
}

#[test]
fn split_step_euler_equivalence() {
    let model = Model::n_link_pendulum(2, 1.0, 0.1);
    assert_eq!(model.integrator, Integrator::Euler);

    let (qpos_step, qvel_step) = run_step(&model, 100);
    let (qpos_split, qvel_split) = run_split(&model, 100);

    // Bitwise match expected for identical code paths
    for i in 0..qpos_step.len() {
        assert!(
            (qpos_step[i] - qpos_split[i]).abs() < 1e-15,
            "qpos[{i}] mismatch: step={} split={}",
            qpos_step[i],
            qpos_split[i]
        );
    }
    for i in 0..qvel_step.len() {
        assert!(
            (qvel_step[i] - qvel_split[i]).abs() < 1e-15,
            "qvel[{i}] mismatch: step={} split={}",
            qvel_step[i],
            qvel_split[i]
        );
    }
}

#[test]
fn split_step_implicit_equivalence() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.integrator = Integrator::ImplicitSpringDamper;
    model.compute_implicit_params();

    let (qpos_step, qvel_step) = run_step(&model, 100);
    let (qpos_split, qvel_split) = run_split(&model, 100);

    for i in 0..qpos_step.len() {
        assert!(
            (qpos_step[i] - qpos_split[i]).abs() < 1e-12,
            "qpos[{i}] mismatch: step={} split={}",
            qpos_step[i],
            qpos_split[i]
        );
    }
    for i in 0..qvel_step.len() {
        assert!(
            (qvel_step[i] - qvel_split[i]).abs() < 1e-12,
            "qvel[{i}] mismatch: step={} split={}",
            qvel_step[i],
            qvel_split[i]
        );
    }
}

#[test]
fn split_step_force_injection() {
    let model = Model::n_link_pendulum(1, 1.0, 0.1);
    let n = 50;

    // Baseline: no injected force
    let mut data_baseline = model.make_data();
    data_baseline.qpos[0] = std::f64::consts::FRAC_PI_4;
    data_baseline.forward(&model).unwrap();
    for _ in 0..n {
        data_baseline.step1(&model).unwrap();
        data_baseline.step2(&model);
    }

    // Injection: apply constant torque between step1/step2
    let mut data_inject = model.make_data();
    data_inject.qpos[0] = std::f64::consts::FRAC_PI_4;
    data_inject.forward(&model).unwrap();
    for _ in 0..n {
        data_inject.step1(&model).unwrap();
        data_inject.qfrc_applied[0] = 5.0; // Apply torque before integration
        data_inject.step2(&model);
    }

    // Trajectories must differ
    let diff: f64 = data_baseline
        .qpos
        .iter()
        .zip(data_inject.qpos.iter())
        .map(|(a, b)| (a - b).abs())
        .sum();
    assert!(
        diff > 1e-3,
        "Force injection should produce different trajectory, diff={diff}"
    );
}

#[test]
fn split_step_invalid_timestep() {
    let mut model = Model::n_link_pendulum(1, 1.0, 0.1);
    model.timestep = 0.0;
    let mut data = model.make_data();

    let result = data.step1(&model);
    assert!(result.is_err());
}
