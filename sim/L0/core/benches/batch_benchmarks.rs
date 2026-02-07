//! Benchmarks for batched simulation throughput.
//!
//! Run with: `cargo bench -p sim-core --features parallel`
//!
//! Measures throughput (envs * steps / second) for varying batch sizes
//! to verify scaling with CPU core count.

#![allow(
    missing_docs,
    clippy::wildcard_imports,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::ignored_unit_patterns
)]

use std::sync::Arc;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use sim_core::Model;
use sim_core::batch::BatchSim;

/// Create a non-trivial model for benchmarking.
///
/// Uses a 10-link pendulum (nv=10) which exercises FK, CRBA, RNE, and
/// integration without requiring external MJCF files. For contact-heavy
/// benchmarks, load an MJCF model with ground plane + geoms.
fn bench_model() -> Model {
    Model::n_link_pendulum(10, 1.0, 0.1)
}

fn bench_step_all(c: &mut Criterion) {
    let model = Arc::new(bench_model());
    let steps_per_iter = 100;

    let mut group = c.benchmark_group("batch_step_all");

    for &n_envs in &[1, 16, 64, 256, 1024] {
        group.bench_with_input(
            BenchmarkId::from_parameter(n_envs),
            &n_envs,
            |b, &n_envs| {
                let mut batch = BatchSim::new(Arc::clone(&model), n_envs);

                // Give each env a distinct initial state so they diverge
                for (i, env) in batch.envs_mut().enumerate() {
                    env.qpos[0] = (i as f64) * 0.01;
                }

                b.iter(|| {
                    for _ in 0..steps_per_iter {
                        let _errors = batch.step_all();
                    }
                });
            },
        );
    }

    group.finish();
}

criterion_group!(benches, bench_step_all);
criterion_main!(benches);
