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
        data.step2(model).unwrap();
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
        data_baseline.step2(&model).unwrap();
    }

    // Injection: apply constant torque between step1/step2
    let mut data_inject = model.make_data();
    data_inject.qpos[0] = std::f64::consts::FRAC_PI_4;
    data_inject.forward(&model).unwrap();
    for _ in 0..n {
        data_inject.step1(&model).unwrap();
        data_inject.qfrc_applied[0] = 5.0; // Apply torque before integration
        data_inject.step2(&model).unwrap();
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

/// T6/G12: RK4 model with split-step works (Euler fallback) but differs from step().
///
/// step2() emits a tracing::warn! when integrator is RK4. The split path
/// uses Euler, while step() uses full RK4 multi-stage substeps.
#[test]
fn rk4_split_step_euler_fallback() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.integrator = Integrator::RungeKutta4;

    let n = 50;

    // Run step() (uses full RK4)
    let (qpos_step, qvel_step) = run_step(&model, n);

    // Run step1()+step2() (uses Euler inside step2)
    let (qpos_split, qvel_split) = run_split(&model, n);

    // Assertion 1: No panic — step2 works
    // (If we got here, it didn't panic.)

    // Assertion 2: Results differ from step() (RK4 vs Euler)
    let pos_diff: f64 = qpos_step
        .iter()
        .zip(qpos_split.iter())
        .map(|(a, b)| (a - b).abs())
        .sum();
    assert!(
        pos_diff > 1e-6,
        "RK4 step() vs split step1()+step2() should produce different results, pos_diff={pos_diff}"
    );

    let vel_diff: f64 = qvel_step
        .iter()
        .zip(qvel_split.iter())
        .map(|(a, b)| (a - b).abs())
        .sum();
    assert!(
        vel_diff > 1e-6,
        "RK4 step() vs split step1()+step2() should produce different results, vel_diff={vel_diff}"
    );
}

/// T6: step1()+step2() on RK4 does not panic and completes successfully.
///
/// Note: The tracing::warn! emitted by step2() for RK4 cannot be captured
/// without a tracing-test dependency. The warning's presence is verified by
/// code inspection — see `forward/mod.rs` step2() RK4 guard.
#[test]
fn rk4_split_step_no_panic() {
    let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
    model.integrator = Integrator::RungeKutta4;

    let mut data = model.make_data();
    data.qpos[0] = std::f64::consts::FRAC_PI_4;
    data.forward(&model).unwrap();

    // Must not panic — run several steps
    for _ in 0..10 {
        data.step1(&model).unwrap();
        data.step2(&model).unwrap();
    }

    // State should have advanced (not stuck at initial)
    assert!(
        (data.qpos[0] - std::f64::consts::FRAC_PI_4).abs() > 1e-6,
        "State should advance after multiple step1()+step2(), qpos[0]={}",
        data.qpos[0]
    );
}
