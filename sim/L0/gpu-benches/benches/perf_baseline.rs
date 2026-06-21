//! Perf/scale baseline: CPU vs GPU-batched physics throughput.
//!
//! Run with: `cargo bench -p sim-gpu-benches`
//!
//! Measures **env-steps per second** — `n_env * steps / wall-second`, the metric
//! MJX/Brax report — across a batch-size sweep, for the same GPU-compatible model
//! run four ways:
//!
//! - `cpu_seq` — single-env `Data::step` in a loop over all envs (naive baseline)
//! - `cpu_batch` — `BatchSim::step_all` (rayon-parallel across envs; the CPU best)
//! - `gpu_rollout` — `GpuPhysicsPipeline::step(num_substeps = steps)`: all substeps
//!   in ONE submit + ONE readback (the GPU's strength)
//! - `gpu_perstep` — `step(num_substeps = 1)` called `steps` times: one upload +
//!   readback PER step (the cost of per-step observations, as in RL)
//!
//! The CPU↔GPU crossover (the `n_env` where `gpu_*` overtakes `cpu_batch`) is the
//! headline number; see `PERF_BASELINE.md` for recorded results + methodology.
//!
//! GPU benches are skipped cleanly when no GPU is available (CPU benches still run).

// This is a benchmark (test-like) crate: `expect`/`unwrap` on setup failures are
// the honest choice — a benchmark that can't construct its pipeline should fail
// loudly, not silently skip. The workspace warns on these for library code; opt
// out here as is conventional for test/bench targets.
#![allow(
    missing_docs,
    clippy::wildcard_imports,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::cast_possible_truncation,
    clippy::ignored_unit_patterns,
    clippy::expect_used
)]

use std::sync::Arc;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use sim_core::batch::BatchSim;
use sim_core::{Data, Model};
use sim_gpu::{GpuPhysicsPipeline, GpuPipelineError};
use sim_gpu_benches::{N_ENVS, STEPS, bench_model, make_datas};

// Both fixtures use the SAME model (free body + ground plane + SDF sphere, nv = 6;
// `bench_model`), differing only in drop height: `no_contact` starts far above the
// plane (free fall; broadphase finds nothing → no constraint solve), `contact`
// starts penetrating (sustained contact → full collision + constraint solve every
// step). See `sim_gpu_benches` for the shared fixtures.

/// True if a GPU pipeline can be created (probes once with a trivial model).
fn gpu_available() -> bool {
    let model = bench_model();
    let data = model.make_data();
    match GpuPhysicsPipeline::new(&model, &data) {
        Err(GpuPipelineError::NoGpu(_)) => false,
        // Ok, or any other error (GPU exists but this model is unsupported) —
        // treat as "available" so the real benches surface the failure.
        _ => true,
    }
}

/// Benchmark one fixture (named `tag`) across all configs and batch sizes.
fn bench_fixture(c: &mut Criterion, tag: &str, build: fn() -> Model, drop_z: f64, gpu: bool) {
    let mut group = c.benchmark_group(tag);
    // GPU submit/readback (and CPU rollouts) per sample are expensive at large
    // n_env; keep sampling + timing modest so the sweep finishes in reasonable
    // wall-time. 10 samples over a short window is enough for a stable baseline.
    group.sample_size(10);
    group.warm_up_time(std::time::Duration::from_millis(800));
    group.measurement_time(std::time::Duration::from_secs(3));

    for &n in N_ENVS {
        let total = (n as u64) * u64::from(STEPS);
        group.throughput(Throughput::Elements(total));

        // ── cpu_seq: single-env step in a loop ────────────────────
        group.bench_with_input(BenchmarkId::new("cpu_seq", n), &n, |b, &n| {
            let model = build();
            let mut datas = make_datas(&model, n, drop_z);
            b.iter(|| {
                for _ in 0..STEPS {
                    for d in &mut datas {
                        d.step(&model).expect("cpu_seq step");
                    }
                }
            });
        });

        // ── cpu_batch: rayon-parallel BatchSim ────────────────────
        group.bench_with_input(BenchmarkId::new("cpu_batch", n), &n, |b, &n| {
            let model = Arc::new(build());
            let mut batch = BatchSim::new(Arc::clone(&model), n);
            for (i, env) in batch.envs_mut().enumerate() {
                env.qpos[0] = (i as f64) * 0.01;
                env.qpos[2] = drop_z;
                env.qpos[3] = 1.0;
            }
            b.iter(|| {
                for _ in 0..STEPS {
                    let _ = batch.step_all();
                }
            });
        });

        if !gpu {
            continue;
        }

        // ── gpu_rollout: STEPS substeps in one submit + one readback ──
        group.bench_with_input(BenchmarkId::new("gpu_rollout", n), &n, |b, &n| {
            let model = build();
            let mut datas = make_datas(&model, n, drop_z);
            let refs: Vec<&Data> = datas.iter().collect();
            let pipe =
                GpuPhysicsPipeline::new_batched(&model, &refs).expect("gpu pipeline (rollout)");
            drop(refs);
            b.iter(|| {
                pipe.step(&model, &mut datas, STEPS);
            });
        });

        // ── gpu_perstep: one submit + readback PER step ───────────
        group.bench_with_input(BenchmarkId::new("gpu_perstep", n), &n, |b, &n| {
            let model = build();
            let mut datas = make_datas(&model, n, drop_z);
            let refs: Vec<&Data> = datas.iter().collect();
            let pipe =
                GpuPhysicsPipeline::new_batched(&model, &refs).expect("gpu pipeline (perstep)");
            drop(refs);
            b.iter(|| {
                for _ in 0..STEPS {
                    pipe.step(&model, &mut datas, 1);
                }
            });
        });
    }

    group.finish();
}

fn perf_baseline(c: &mut Criterion) {
    let gpu = gpu_available();
    if !gpu {
        eprintln!("perf_baseline: no GPU detected — running CPU configs only");
    }
    // no_contact: sphere (radius 5) starts at z=20, far above the z=0 plane →
    // free fall, collision broadphase finds nothing, no constraint solve.
    bench_fixture(c, "no_contact", bench_model, 20.0, gpu);
    // contact: sphere starts penetrating the plane → sustained contact every step.
    bench_fixture(c, "contact", bench_model, 3.0, gpu);
}

criterion_group!(benches, perf_baseline);
criterion_main!(benches);
