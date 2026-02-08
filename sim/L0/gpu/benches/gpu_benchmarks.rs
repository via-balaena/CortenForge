//! Benchmark: `BatchSim` (CPU) vs `GpuBatchSim` (GPU) step throughput.
//!
//! Model: `n_link_pendulum(3, 1.0, 0.1)` — 4 bodies (world+3), 3 hinge joints, nv=3
//! Env counts: 64, 256, 1024, 4096
//! Measurement: 10 consecutive `step_all()` calls per iteration
//! Groups: `cpu/{n}` vs `gpu/{n}`

#![allow(
    missing_docs,                   // criterion_main! generates undocumented items
    clippy::missing_docs_in_private_items,
    clippy::let_underscore_must_use // benchmark intentionally discards step_all results
)]

use std::sync::Arc;

use criterion::{Criterion, criterion_group, criterion_main};
use sim_core::Model;
use sim_core::batch::BatchSim;
use sim_gpu::GpuBatchSim;

fn bench_cpu_vs_gpu(c: &mut Criterion) {
    let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));

    for &n in &[64, 256, 1024, 4096] {
        // CPU benchmark
        c.bench_function(&format!("cpu/{n}"), |b| {
            let mut batch = BatchSim::new(Arc::clone(&model), n);
            b.iter(|| {
                for _ in 0..10 {
                    let _ = batch.step_all();
                }
            });
        });

        // GPU benchmark (skip if no GPU)
        if let Ok(mut batch) = GpuBatchSim::new(Arc::clone(&model), n) {
            c.bench_function(&format!("gpu/{n}"), |b| {
                b.iter(|| {
                    for _ in 0..10 {
                        let _ = batch.step_all();
                    }
                });
            });
        }
    }
}

criterion_group!(benches, bench_cpu_vs_gpu);
criterion_main!(benches);
