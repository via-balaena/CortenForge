//! GPU simulation integration tests.
//!
//! All tests use `n_link_pendulum(3, 1.0, 0.1)` (nv=3, Euler integrator)
//! unless stated otherwise.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation, // f64→f32 intentional in precision tests
    clippy::needless_range_loop,
    clippy::suboptimal_flops,         // tolerance formulas are clearer without mul_add
    clippy::let_underscore_must_use,  // test discards is_available() result
    clippy::panic                     // tests use panic! for clear failure messages
)]

use std::sync::Arc;

use sim_core::{Integrator, Model};
use sim_gpu::{GpuBatchSim, GpuParams, GpuSimContext, GpuSimError};

// ── Always-run tests (no GPU required) ────────────────────────────

#[test]
fn gpu_context_availability() {
    // GpuSimContext::is_available() returns bool without panic.
    let _ = GpuSimContext::is_available();
}

#[test]
fn try_new_fallback_consistency() {
    // try_new() returns Ok(Some) iff is_available(), Ok(None) otherwise.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let result = GpuBatchSim::try_new(model, 4);
    match result {
        Ok(Some(_)) => assert!(GpuSimContext::is_available()),
        Ok(None) => assert!(!GpuSimContext::is_available()),
        Err(e) => panic!("try_new returned unexpected error: {e}"),
    }
}

#[test]
fn gpu_params_layout() {
    assert_eq!(std::mem::size_of::<GpuParams>(), 16);
    assert_eq!(std::mem::align_of::<GpuParams>(), 4);
}

#[test]
fn gpu_params_from_model() {
    let model = Model::n_link_pendulum(3, 1.0, 0.1);
    // model.timestep = 1.0/240.0 ≈ 0.004167 for n_link_pendulum
    let params = GpuParams::from_model(&model, 64);
    assert_eq!(params.num_envs, 64);
    assert_eq!(params.nv, 3);
    // f64→f32 conversion: verify round-trip accuracy at this scale
    let expected_ts = (1.0_f64 / 240.0) as f32;
    assert!((params.timestep - expected_ts).abs() < f32::EPSILON);
}

#[test]
fn gpu_unsupported_integrator() {
    // Checked before GPU availability — must work without a GPU.
    let mut model = Model::n_link_pendulum(3, 1.0, 0.1);

    model.integrator = Integrator::RungeKutta4;
    let err = GpuBatchSim::new(Arc::new(model.clone()), 4).unwrap_err();
    assert!(matches!(
        err,
        GpuSimError::UnsupportedIntegrator(Integrator::RungeKutta4)
    ));

    model.integrator = Integrator::ImplicitSpringDamper;
    let err = GpuBatchSim::new(Arc::new(model), 4).unwrap_err();
    assert!(matches!(
        err,
        GpuSimError::UnsupportedIntegrator(Integrator::ImplicitSpringDamper)
    ));
}

// ── GPU-required tests ────────────────────────────────────────────
// Ignored in CI without GPU. Run locally with: cargo test -- --ignored

#[test]
#[ignore = "Requires GPU"]
fn gpu_euler_ok() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let batch = GpuBatchSim::new(model, 4);
    assert!(batch.is_ok());
    assert_eq!(batch.unwrap().len(), 4);
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_step_empty_batch() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(model, 0).unwrap();
    let result = batch.step_all().unwrap();
    assert!(result.is_empty());
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_step_nv_zero() {
    // Model with nv=0: GPU dispatch is a no-op, no crash.
    let model = Arc::new(Model::empty());
    assert_eq!(model.nv, 0);
    let mut batch = GpuBatchSim::new(model, 4).unwrap();
    let result = batch.step_all().unwrap();
    assert_eq!(result.len(), 4);
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_matches_cpu_single_step() {
    // AC1: GPU matches CPU within f32 tolerance after 1 step.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut gpu_batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    let mut cpu_batch = gpu_batch.cpu_reference();

    gpu_batch.step_all().unwrap();
    cpu_batch.step_all();

    for i in 0..4 {
        let gpu_data = gpu_batch.env(i).unwrap();
        let cpu_data = cpu_batch.env(i).unwrap();
        for j in 0..model.nv {
            let abs_err = (gpu_data.qvel[j] - cpu_data.qvel[j]).abs();
            let rel_bound = 1.2e-7 * cpu_data.qvel[j].abs() + 1e-10;
            assert!(
                abs_err <= rel_bound,
                "env {i} qvel[{j}]: gpu={}, cpu={}, err={abs_err}, bound={rel_bound}",
                gpu_data.qvel[j],
                cpu_data.qvel[j]
            );
        }
    }
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_matches_cpu_100_steps() {
    // AC1: tolerance scales linearly with step count (N=100).
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut gpu_batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    let mut cpu_batch = gpu_batch.cpu_reference();

    for _ in 0..100 {
        gpu_batch.step_all().unwrap();
        cpu_batch.step_all();
    }

    let n = 100.0_f64;
    for i in 0..4 {
        let gpu_data = gpu_batch.env(i).unwrap();
        let cpu_data = cpu_batch.env(i).unwrap();
        for j in 0..model.nv {
            let abs_err = (gpu_data.qvel[j] - cpu_data.qvel[j]).abs();
            let rel_bound = n * 1.2e-7 * cpu_data.qvel[j].abs() + n * 1e-10;
            assert!(
                abs_err <= rel_bound,
                "env {i} qvel[{j}] after 100 steps: err={abs_err}, bound={rel_bound}"
            );
        }
    }
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_determinism() {
    // AC4: 5 identical runs produce bitwise-identical qvel.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut reference: Option<Vec<Vec<f64>>> = None;

    for run in 0..5 {
        let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
        for _ in 0..10 {
            batch.step_all().unwrap();
        }

        let qvels: Vec<Vec<f64>> = (0..4)
            .map(|i| batch.env(i).unwrap().qvel.as_slice().to_vec())
            .collect();

        if let Some(ref expected) = reference {
            assert_eq!(&qvels, expected, "Run {run} diverged from run 0");
        } else {
            reference = Some(qvels);
        }
    }
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_error_isolation() {
    // AC5: NaN in one env doesn't corrupt others.
    // With Euler integrator, forward() does not return Err on NaN qpos —
    // the NaN propagates silently through FK/dynamics to qacc, then to
    // qvel via GPU integration. The key invariant is that NaN stays
    // confined to env 1 and does not leak to other envs.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();

    // Inject NaN into env 1's qpos — NaN will propagate through dynamics
    batch.env_mut(1).unwrap().qpos[0] = f64::NAN;

    let errors = batch.step_all().unwrap();

    // Env 1's qvel should be NaN (NaN propagated through qacc → GPU → qvel)
    let env1 = batch.env(1).unwrap();
    let has_nan = (0..model.nv).any(|j| !env1.qvel[j].is_finite());
    assert!(
        has_nan || errors[1].is_some(),
        "env 1 should have NaN qvel or a StepError"
    );

    // Env 0, 2, 3 must have valid (non-NaN) qvel — error isolation
    for i in [0, 2, 3] {
        assert!(errors[i].is_none(), "env {i} should not have an error");
        let data = batch.env(i).unwrap();
        for j in 0..model.nv {
            assert!(
                data.qvel[j].is_finite(),
                "env {i} qvel[{j}] is not finite after NaN in env 1"
            );
        }
    }
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_reset_single() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    for _ in 0..10 {
        batch.step_all().unwrap();
    }

    batch.reset(2);
    let data = batch.env(2).unwrap();
    // After reset: qvel = 0, time = 0
    for j in 0..model.nv {
        assert_eq!(data.qvel[j], 0.0);
    }
    assert_eq!(data.time, 0.0);
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_reset_where() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    for _ in 0..10 {
        batch.step_all().unwrap();
    }

    let pre_step_time = batch.env(0).unwrap().time;
    batch.reset_where(&[false, true, false, true]);

    // Env 1 and 3 reset, env 0 and 2 unchanged
    assert_eq!(batch.env(1).unwrap().time, 0.0);
    assert_eq!(batch.env(3).unwrap().time, 0.0);
    assert_eq!(batch.env(0).unwrap().time, pre_step_time);
    assert_eq!(batch.env(2).unwrap().time, pre_step_time);
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_reset_all() {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut batch = GpuBatchSim::new(Arc::clone(&model), 4).unwrap();
    for _ in 0..10 {
        batch.step_all().unwrap();
    }

    batch.reset_all();
    for i in 0..4 {
        let data = batch.env(i).unwrap();
        assert_eq!(data.time, 0.0);
        for j in 0..model.nv {
            assert_eq!(data.qvel[j], 0.0);
        }
    }
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_step_single_env() {
    // Single-env batch: no off-by-one in buffer offsets.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    let mut gpu = GpuBatchSim::new(Arc::clone(&model), 1).unwrap();
    let mut cpu = gpu.cpu_reference();

    gpu.step_all().unwrap();
    cpu.step_all();

    let gpu_qvel = gpu.env(0).unwrap().qvel.as_slice();
    let cpu_qvel = cpu.env(0).unwrap().qvel.as_slice();
    for j in 0..model.nv {
        let err = (gpu_qvel[j] - cpu_qvel[j]).abs();
        assert!(
            err <= 1.2e-7 * cpu_qvel[j].abs() + 1e-10,
            "qvel[{j}]: gpu={}, cpu={}, err={err}",
            gpu_qvel[j],
            cpu_qvel[j]
        );
    }
}

#[test]
#[ignore = "Requires GPU"]
fn gpu_batch_too_large() {
    // Construct a batch that exceeds max_storage_buffer_binding_size.
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
    // u32::MAX envs × 3 DOFs × 4 bytes = ~51 GB — exceeds any real GPU limit.
    let err = GpuBatchSim::new(model, u32::MAX as usize).unwrap_err();
    assert!(matches!(err, GpuSimError::BatchTooLarge { .. }));
}
