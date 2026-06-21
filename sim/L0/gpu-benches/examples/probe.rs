//! Quick scaling probe: call `GpuPhysicsPipeline::step()` in a tight loop and
//! print per-iteration progress (z, vz, wall-time). A lightweight way to sanity
//! check GPU throughput / stability at a given `(substeps, n_env)` before
//! committing to the full criterion sweep. Usage: `cargo run --release --example
//! probe -- <substeps> <n_env>` (defaults: 100 substeps, 1 env).
#![allow(missing_docs, clippy::cast_precision_loss)]

use std::io::Write;
use std::time::Instant;

use sim_core::Data;
use sim_gpu::GpuPhysicsPipeline;
use sim_gpu_benches::bench_model;

fn main() {
    let model = bench_model();
    let n_env: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    let mut datas: Vec<Data> = (0..n_env)
        .map(|i| {
            let mut d = model.make_data();
            d.qpos[0] = i as f64 * 0.01;
            d.qpos[2] = 20.0; // free fall (no_contact case)
            d.qpos[3] = 1.0;
            d
        })
        .collect();
    let refs: Vec<&Data> = datas.iter().collect();
    let Ok(pipe) = GpuPhysicsPipeline::new_batched(&model, &refs) else {
        eprintln!("no GPU");
        return;
    };
    drop(refs);

    let substeps: u32 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(100);
    println!("probe: 30 iterations of step({substeps} substeps)");
    let t0 = Instant::now();
    for i in 0..30 {
        pipe.step(&model, &mut datas, substeps);
        if i % 5 == 0 {
            println!(
                "  iter {i:3}  z={:+.3e}  vz={:+.3e}  t={:.2}s",
                datas[0].qpos[2],
                datas[0].qvel[2],
                t0.elapsed().as_secs_f64()
            );
            std::io::stdout().flush().ok();
        }
    }
    println!("DONE in {:.2}s", t0.elapsed().as_secs_f64());
}
