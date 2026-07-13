//! Benchmarks for `Competition::run_replicates` across-run parallelism.
//!
//! Run with: `cargo bench -p sim-ml-chassis-benches --bench competition_benchmarks`
//!
//! ## What this measures
//!
//! `run_replicates` executes independent `(task, builder, seed)` runs over a
//! rayon pool. This bench pits the real parallel path against a
//! looped-sequential baseline (each run issued as its own one-item
//! `run_replicates`, so no two overlap) on the same total work — a real
//! 6-DOF reaching task with physics-bound rollouts. The ratio of the two
//! timings is the committed, re-runnable speedup figure.
//!
//! On an M-series (12 cores) the parallel case runs ~6–8x faster than the
//! sequential loop for a multi-seed workload; the exact number scales with
//! core count and run count (coarse-grained, so it balances well once runs
//! outnumber cores). A single small competition (few, unequal runs) sees
//! less because wall time is capped by the slowest run.

#![allow(
    missing_docs,
    clippy::wildcard_imports,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::let_underscore_must_use,
    clippy::cast_precision_loss
)]

use std::collections::BTreeMap;

use criterion::{Criterion, criterion_group, criterion_main};
use sim_ml_chassis::artifact::{PolicyArtifact, TrainingCheckpoint};
use sim_ml_chassis::{
    Algorithm, Competition, EpochMetrics, LinearPolicy, TaskConfig, Tensor, TrainingBudget, VecEnv,
    reaching_6dof,
};

// ─── a physics-bound stand-in algorithm ──────────────────────────────────────

/// Minimal [`Algorithm`] whose `train` cost is dominated by real env
/// stepping — the same physics-bound profile as CEM/PPO/etc., without a
/// dependency on the higher-tier `sim-rl`. Each epoch resets and steps the
/// env `max_steps` times, so a run carries genuine parallelizable work.
struct RolloutAlgorithm {
    policy: LinearPolicy,
    max_steps: usize,
}

impl Algorithm for RolloutAlgorithm {
    fn name(&self) -> &'static str {
        "Rollout"
    }

    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        _seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics> {
        let n = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(_) => 1,
        };
        let n_envs = env.n_envs();
        let act_dim = env.model().nu;
        let actions = Tensor::zeros(&[n_envs, act_dim]);
        let mut metrics = Vec::with_capacity(n);
        for epoch in 0..n {
            let _ = env.reset_all();
            let mut total = 0.0;
            for _ in 0..self.max_steps {
                let result = env.step(&actions).unwrap();
                total += result.rewards.iter().sum::<f64>();
            }
            let em = EpochMetrics {
                epoch,
                mean_reward: total / n_envs as f64,
                done_count: 0,
                total_steps: n_envs * self.max_steps,
                wall_time_ms: 0,
                extra: BTreeMap::new(),
            };
            on_epoch(&em);
            metrics.push(em);
        }
        metrics
    }

    fn policy_artifact(&self) -> PolicyArtifact {
        PolicyArtifact::from_policy(&self.policy)
    }

    fn best_artifact(&self) -> PolicyArtifact {
        self.policy_artifact()
    }

    fn checkpoint(&self) -> TrainingCheckpoint {
        TrainingCheckpoint {
            algorithm_name: "Rollout".into(),
            policy_artifact: self.policy_artifact(),
            critics: vec![],
            optimizer_states: vec![],
            algorithm_state: BTreeMap::new(),
            best_params: None,
            best_reward: None,
            best_epoch: 0,
        }
    }
}

// ─── competition parallel vs sequential ──────────────────────────────────────

fn bench_run_replicates(c: &mut Criterion) {
    const N_ENVS: usize = 8;
    const MAX_STEPS: usize = 50;
    let budget = TrainingBudget::Epochs(2);
    let seeds: Vec<u64> = (0..12).collect(); // 12 independent runs
    let task = reaching_6dof();

    let build: &(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync) = &|t: &TaskConfig| {
        Box::new(RolloutAlgorithm {
            policy: LinearPolicy::new(t.obs_dim(), t.act_dim(), t.obs_scale()),
            max_steps: MAX_STEPS,
        })
    };
    let comp = Competition::new(N_ENVS, budget, 0);

    let mut group = c.benchmark_group("competition_run_replicates_6dof");
    // Each iteration runs 12 full training runs — keep the sample count low.
    group.sample_size(10);

    // Real shipping path: all 12 runs dispatched across the rayon pool.
    group.bench_function("parallel_12runs", |b| {
        b.iter(|| {
            let result = comp
                .run_replicates(std::slice::from_ref(&task), &[build], &seeds)
                .unwrap();
            std::hint::black_box(result.runs.len())
        });
    });

    // Sequential baseline: each run issued alone so none overlap; same work.
    group.bench_function("sequential_12runs", |b| {
        b.iter(|| {
            let mut total = 0;
            for &seed in &seeds {
                let result = comp
                    .run_replicates(std::slice::from_ref(&task), &[build], &[seed])
                    .unwrap();
                total += result.runs.len();
            }
            std::hint::black_box(total)
        });
    });

    group.finish();
}

criterion_group!(benches, bench_run_replicates);
criterion_main!(benches);
