//! Competition runner — runs algorithms on tasks, collects standardized results.
//!
//! The [`Competition`] runner is the dynamometer for the ML layer.  It takes a
//! set of [`TaskConfig`]s and a set of algorithm builders, runs every
//! (task, algorithm) combination, and returns structured [`RunResult`]s that
//! competition tests can assert against.
//!
//! Each (task, algorithm) pair gets a fresh [`VecEnv`](crate::VecEnv) — no
//! cross-contamination between runs.

use std::collections::BTreeMap;
use std::path::Path;

use crate::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
use crate::artifact::{ArtifactError, PolicyArtifact, TrainingProvenance};
use crate::error::EnvError;
use crate::task::TaskConfig;

// ── RunResult ───────────────────────────────────────────────────────────────

/// Result from running one algorithm on one task for one replicate.
#[derive(Debug, Clone)]
pub struct RunResult {
    /// Task name (from [`TaskConfig::name()`]).
    pub task_name: String,
    /// Algorithm name (from [`Algorithm::name()`]).
    pub algorithm_name: String,
    /// 0-based index of this replicate within the seeds slice passed to
    /// [`Competition::run_replicates`].  Single-seed calls via
    /// [`Competition::run`] produce `replicate_index == 0`.
    pub replicate_index: usize,
    /// Per-epoch metrics from [`Algorithm::train()`].
    pub metrics: Vec<EpochMetrics>,
    /// Trained policy artifact with provenance (task, seed, metrics, wall time).
    pub artifact: PolicyArtifact,
    /// Best-epoch policy artifact with provenance.
    pub best_artifact: PolicyArtifact,
}

impl RunResult {
    /// Final epoch's mean reward, or `None` if no epochs ran.
    #[must_use]
    pub fn final_reward(&self) -> Option<f64> {
        self.metrics.last().map(|m| m.mean_reward)
    }

    /// Best epoch's mean reward, or `None` if no epochs ran.
    #[must_use]
    pub fn best_reward(&self) -> Option<f64> {
        self.metrics
            .iter()
            .map(|m| m.mean_reward)
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
    }

    /// Total done triggers across all epochs.
    #[must_use]
    pub fn total_dones(&self) -> usize {
        self.metrics.iter().map(|m| m.done_count).sum()
    }

    /// Panics if any epoch has a non-finite `mean_reward`.
    ///
    /// Checks every epoch in `self.metrics`. On failure, the panic message
    /// includes the algorithm name, epoch index, and the offending value.
    ///
    /// # Panics
    ///
    /// Panics when any `EpochMetrics::mean_reward` is `NaN` or infinite.
    pub fn assert_finite(&self) {
        for m in &self.metrics {
            assert!(
                m.mean_reward.is_finite(),
                "{} epoch {} non-finite reward: {}",
                self.algorithm_name,
                m.epoch,
                m.mean_reward
            );
        }
    }
}

// ── CompetitionResult ───────────────────────────────────────────────────────

/// Aggregated results from a full competition run.
#[derive(Debug, Clone)]
pub struct CompetitionResult {
    /// One entry per (task, algorithm) pair, in task-major order.
    pub runs: Vec<RunResult>,
}

impl CompetitionResult {
    /// Find the result for a specific (task, algorithm) pair.
    #[must_use]
    pub fn find(&self, task: &str, algorithm: &str) -> Option<&RunResult> {
        self.runs
            .iter()
            .find(|r| r.task_name == task && r.algorithm_name == algorithm)
    }

    /// All results for a given task, in builder order.
    #[must_use]
    pub fn for_task(&self, task: &str) -> Vec<&RunResult> {
        self.runs.iter().filter(|r| r.task_name == task).collect()
    }

    /// All results for a given algorithm, in task order.
    #[must_use]
    pub fn for_algorithm(&self, algorithm: &str) -> Vec<&RunResult> {
        self.runs
            .iter()
            .filter(|r| r.algorithm_name == algorithm)
            .collect()
    }

    /// Save all artifacts to a directory.
    ///
    /// File naming: `{task}_{algorithm}.artifact.json`.  Creates `dir` if
    /// it doesn't exist.  Each artifact is validated before writing.
    ///
    /// # Errors
    ///
    /// Returns [`ArtifactError`] if validation fails or a file can't be written.
    pub fn save_artifacts(&self, dir: impl AsRef<Path>) -> Result<(), ArtifactError> {
        let dir = dir.as_ref();
        std::fs::create_dir_all(dir)?;
        for run in &self.runs {
            let filename = format!("{}_{}.artifact.json", run.task_name, run.algorithm_name);
            run.artifact.save(dir.join(filename))?;
            let best_filename = format!(
                "{}_{}.best.artifact.json",
                run.task_name, run.algorithm_name
            );
            run.best_artifact.save(dir.join(best_filename))?;
        }
        Ok(())
    }

    /// Best artifact for a task (highest best reward).
    #[must_use]
    pub fn best_for_task(&self, task: &str) -> Option<&PolicyArtifact> {
        self.for_task(task)
            .into_iter()
            .max_by(|a, b| {
                let ra = a.best_reward().unwrap_or(f64::NEG_INFINITY);
                let rb = b.best_reward().unwrap_or(f64::NEG_INFINITY);
                ra.partial_cmp(&rb).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|r| &r.best_artifact)
    }

    /// Print a ranked results table for one task to stderr.
    ///
    /// Sorts algorithms by best reward (best first), prints a formatted
    /// table with algorithm name, final reward, best reward, best epoch,
    /// and total dones, followed by the ordering string.
    pub fn print_ranked(&self, task: &str, title: &str) {
        let runs = self.for_task(task);
        let mut ranked: Vec<(&str, f64, f64, usize, usize)> = runs
            .iter()
            .map(|r| {
                (
                    r.algorithm_name.as_str(),
                    r.final_reward().unwrap_or(f64::NAN),
                    r.best_reward().unwrap_or(f64::NAN),
                    r.metrics
                        .iter()
                        .enumerate()
                        .max_by(|(_, a), (_, b)| {
                            a.mean_reward
                                .partial_cmp(&b.mean_reward)
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .map_or(0, |(i, _)| i),
                    r.total_dones(),
                )
            })
            .collect();
        ranked.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));

        eprintln!("\n=== {title} ===");
        eprintln!(
            "{:<12} {:>14} {:>13} {:>8} {:>10}",
            "Algorithm", "Final Reward", "Best Reward", "Best @", "Dones"
        );
        eprintln!("{}", "-".repeat(61));
        for (name, final_r, best_r, best_at, dones) in &ranked {
            eprintln!("{name:<12} {final_r:>14.2} {best_r:>13.2} {best_at:>8} {dones:>10}");
        }

        let ordering: Vec<&str> = ranked.iter().map(|(n, _, _, _, _)| *n).collect();
        eprintln!(
            "\nOrdering by best (best → worst): {}",
            ordering.join(" > ")
        );
    }

    /// Print a summary table of all results to stderr.
    ///
    /// Uses `eprintln!` so output is visible even under test capture
    /// (shows on failure, or with `--nocapture`).
    pub fn print_summary(&self) {
        eprintln!(
            "\n{:<20} {:<15} {:>14} {:>12} {:>10}",
            "Task", "Algorithm", "Final Reward", "Total Dones", "Wall (ms)"
        );
        eprintln!("{}", "-".repeat(75));
        for run in &self.runs {
            let reward = run
                .final_reward()
                .map_or_else(|| "N/A".to_string(), |r| format!("{r:.2}"));
            let wall: u64 = run.metrics.iter().map(|m| m.wall_time_ms).sum();
            eprintln!(
                "{:<20} {:<15} {:>14} {:>12} {:>10}",
                run.task_name,
                run.algorithm_name,
                reward,
                run.total_dones(),
                wall
            );
        }
        eprintln!();
    }

    // ── Replicate aggregation (Ch 24 Decision 2, PR 2a) ───────────────

    /// Find the specific replicate for `(task, algorithm, replicate_index)`.
    ///
    /// Returns `None` if no [`RunResult`] matches all three fields.
    /// Useful for pulling out a single replicate's artifact after a
    /// [`Competition::run_replicates`] call.
    #[must_use]
    pub fn find_replicate(
        &self,
        task: &str,
        algorithm: &str,
        replicate_index: usize,
    ) -> Option<&RunResult> {
        self.runs.iter().find(|r| {
            r.task_name == task
                && r.algorithm_name == algorithm
                && r.replicate_index == replicate_index
        })
    }

    /// Collect the per-replicate best-reward values for one
    /// `(task, algorithm)` pair.
    ///
    /// Silently filters out replicates whose
    /// [`RunResult::best_reward()`] returns `None` (zero-epoch runs or
    /// runs where every `mean_reward` was non-finite).  Callers that
    /// need to detect missing replicates compare the returned
    /// `Vec<f64>`'s length against the seed-slice length at the call
    /// site.  Ch 24 §4.7 picked the silent-filter-out shape contingent
    /// on PR 2b's REINFORCE/PPO zero-fallback cleanup.
    #[must_use]
    pub fn replicate_best_rewards(&self, task: &str, algorithm: &str) -> Vec<f64> {
        self.runs
            .iter()
            .filter(|r| r.task_name == task && r.algorithm_name == algorithm)
            .filter_map(RunResult::best_reward)
            .collect()
    }

    /// Collect the per-replicate final-epoch-reward values for one
    /// `(task, algorithm)` pair.
    ///
    /// Parallel to [`Self::replicate_best_rewards`] in shape and
    /// filtering contract: silently filters out replicates whose
    /// [`RunResult::final_reward()`] returns `None` (zero-epoch
    /// runs).  Since `best_reward` and `final_reward` are both
    /// `None` iff the replicate ran zero epochs, the two accessors
    /// always return vectors of the same length for the same
    /// `(task, algorithm)` pair, and their indices line up 1:1.
    ///
    /// Added for the ml-chassis-refactor study's Ch 51 dual-metric
    /// amendment — the rematch driver plumbs this through
    /// `sim_opt::analysis::TwoMetricOutcome` so Ch 32's
    /// bootstrap-and-classify pipeline runs once on
    /// `best_reward` and once on `final_reward` per run.  See
    /// Ch 51 §2.1 for the amendment's scope and Ch 51 §2.3 for
    /// the joint-verdict interpretation framework.
    #[must_use]
    pub fn replicate_final_rewards(&self, task: &str, algorithm: &str) -> Vec<f64> {
        self.runs
            .iter()
            .filter(|r| r.task_name == task && r.algorithm_name == algorithm)
            .filter_map(RunResult::final_reward)
            .collect()
    }

    /// Summarize the per-replicate best-reward distribution for one
    /// `(task, algorithm)` pair as a [`SeedSummary`].
    ///
    /// Returns `None` if no replicates survive the filter (see
    /// [`Self::replicate_best_rewards`]).  For `n == 1`, `std_dev` is
    /// returned as `0.0` with the understanding that sample standard
    /// deviation is undefined for a single observation; callers that
    /// need a strict `n >= 2` guard should check the `.n` field
    /// directly.
    #[must_use]
    pub fn describe(&self, task: &str, algorithm: &str) -> Option<SeedSummary> {
        let rewards = self.replicate_best_rewards(task, algorithm);
        SeedSummary::from_rewards(&rewards)
    }
}

// ── SeedSummary ─────────────────────────────────────────────────────────────

/// Across-replicate summary statistics for one `(task, algorithm)` pair.
///
/// Constructed via [`SeedSummary::from_rewards`] from a slice of
/// per-replicate best rewards — typically the output of
/// [`CompetitionResult::replicate_best_rewards`].
///
/// Ch 24 Decision 2's Option M (medium) aggregation surface: just the
/// three fields, sample standard deviation with Bessel's correction.
/// Richer summaries (percentiles, bootstrap CIs) are caller-side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeedSummary {
    /// Number of replicates contributing to `mean` / `std_dev`.
    pub n: usize,
    /// Arithmetic mean of the rewards slice.
    pub mean: f64,
    /// Sample standard deviation (`n-1` denominator, Bessel's
    /// correction).  Returns `0.0` for `n == 1` with the caveat that
    /// sample std is undefined for a single observation.
    pub std_dev: f64,
}

impl SeedSummary {
    /// Build a summary from a slice of per-replicate rewards.
    ///
    /// Returns `None` for an empty slice.  For `n == 1`, returns
    /// `Some(SeedSummary { n: 1, mean: rewards[0], std_dev: 0.0 })`;
    /// sample standard deviation is mathematically undefined but
    /// returning zero is the least-surprising default for pipeline
    /// code that aggregates many pairs and would otherwise have to
    /// special-case every single-replicate result.
    #[must_use]
    pub fn from_rewards(rewards: &[f64]) -> Option<Self> {
        if rewards.is_empty() {
            return None;
        }
        let n = rewards.len();
        // `n` is `usize` bounded by the length of a Rust slice; the
        // usize→f64 cast is lossy only for n > 2^53, which would
        // require the caller to build a ~petabyte-sized rewards slice
        // first.  The `f64` cast is fine here.
        #[allow(clippy::cast_precision_loss)]
        let n_f64 = n as f64;
        let mean = rewards.iter().sum::<f64>() / n_f64;
        let std_dev = if n == 1 {
            0.0
        } else {
            // Sample standard deviation with Bessel's correction
            // (denominator = n - 1), matching RL-literature convention.
            #[allow(clippy::cast_precision_loss)]
            let denom = (n - 1) as f64;
            let variance: f64 = rewards
                .iter()
                .map(|r| {
                    let d = r - mean;
                    d * d
                })
                .sum::<f64>()
                / denom;
            variance.sqrt()
        };
        Some(Self { n, mean, std_dev })
    }
}

// ── Timestamp helper ───────────────────────────────────────────────────────

/// Returns the current UTC time as an ISO 8601 string (e.g. `"2026-04-08T14:30:00Z"`).
///
/// Uses a standard civil-date algorithm (Howard Hinnant) to convert epoch
/// seconds to year/month/day. No external crate needed.
// Hinnant civil-date algorithm uses single-letter names (y/m/d/era/doe/doy/mp)
// from the published reference; epoch arithmetic round-trips through i64/u32 so
// wrap/sign losses are bounded by realistic Unix-epoch ranges.
#[allow(
    clippy::many_single_char_names,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss
)]
fn now_iso8601() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();

    let sec = secs % 60;
    let min = (secs / 60) % 60;
    let hour = (secs / 3600) % 24;

    // Days since 1970-01-01 → civil date (Hinnant algorithm).
    let z = (secs / 86400) as i64 + 719_468;
    let era = (if z >= 0 { z } else { z - 146_096 }) / 146_097;
    let doe = (z - era * 146_097) as u64; // day of era [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let year = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = if month <= 2 { year + 1 } else { year };

    format!("{year:04}-{month:02}-{day:02}T{hour:02}:{min:02}:{sec:02}Z")
}

// ── Competition ─────────────────────────────────────────────────────────────

/// Runs algorithms on tasks and collects standardized results.
///
/// # Example
///
/// ```ignore
/// let comp = Competition::new(50, TrainingBudget::Epochs(30), 42);
/// let result = comp.run(
///     &[reaching_2dof(), reaching_6dof()],
///     &[&|task| build_ppo(task), &|task| build_sac(task)],
/// )?;
///
/// // PPO should beat CEM on 6-DOF
/// let ppo = result.find("reaching-6dof", "PPO").unwrap();
/// let cem = result.find("reaching-6dof", "CEM").unwrap();
/// assert!(ppo.final_reward() > cem.final_reward());
/// ```
pub struct Competition {
    n_envs: usize,
    budget: TrainingBudget,
    seed: u64,
    verbose: bool,
}

impl Competition {
    /// Create a competition runner (silent — no epoch logging).
    ///
    /// - `n_envs`: parallel environments per run.
    /// - `budget`: how long each algorithm trains.
    /// - `seed`: RNG seed passed to [`Algorithm::train()`].
    #[must_use]
    pub const fn new(n_envs: usize, budget: TrainingBudget, seed: u64) -> Self {
        Self {
            n_envs,
            budget,
            seed,
            verbose: false,
        }
    }

    /// Create a competition runner with epoch-level logging to stderr.
    ///
    /// Prints algorithm start/end messages and per-epoch metrics, so you
    /// can watch training progress in real time during multi-minute runs.
    #[must_use]
    pub const fn new_verbose(n_envs: usize, budget: TrainingBudget, seed: u64) -> Self {
        Self {
            n_envs,
            budget,
            seed,
            verbose: true,
        }
    }

    /// Run all algorithm builders on all tasks (single-seed).
    ///
    /// Thin wrapper over [`Self::run_replicates`] with a one-element seed
    /// slice containing `self.seed`, preserving the legacy ergonomics.
    /// Returns one [`RunResult`] per (task, builder) pair, in task-major
    /// order, every result carrying `replicate_index == 0`.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError`] if any [`TaskConfig::build_vec_env()`] fails.
    pub fn run(
        &self,
        tasks: &[TaskConfig],
        builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
    ) -> Result<CompetitionResult, EnvError> {
        self.run_replicates(tasks, builders, &[self.seed])
    }

    /// Run all algorithm builders on all tasks across multiple replicate
    /// seeds.
    ///
    /// Returns `tasks.len() * builders.len() * seeds.len()` [`RunResult`]s
    /// in seeds-outermost, then task-major, then builder-major order.  Each
    /// `(task, builder, seed)` triple gets a fresh
    /// [`VecEnv`](crate::VecEnv) constructed via
    /// [`TaskConfig::build_vec_env`] with the per-replicate seed threaded
    /// through — stochastic-physics tasks (those carrying a
    /// `LangevinThermostat`) vary their physics noise sequence across
    /// replicates, deterministic tasks ignore the seed.
    ///
    /// Execution is sequential (no `rayon` dependency on L0); the first
    /// error aborts the whole call, matching [`Self::run`]'s contract.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError`] if any [`TaskConfig::build_vec_env()`] fails.
    pub fn run_replicates(
        &self,
        tasks: &[TaskConfig],
        builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
        seeds: &[u64],
    ) -> Result<CompetitionResult, EnvError> {
        let mut runs = Vec::with_capacity(tasks.len() * builders.len() * seeds.len());

        for (replicate_index, &seed) in seeds.iter().enumerate() {
            for task in tasks {
                for builder in builders {
                    let mut env = task.build_vec_env(self.n_envs, seed)?;
                    let mut algorithm = builder(task);
                    let name = algorithm.name();

                    if self.verbose {
                        eprintln!(
                            "\n[{name}] training on {} (replicate {replicate_index}, seed {seed})...",
                            task.name()
                        );
                    }

                    let t0 = std::time::Instant::now();
                    let verbose = self.verbose;

                    let metrics = algorithm.train(&mut env, self.budget, seed, &|m| {
                        if verbose {
                            eprintln!(
                                "  epoch {:>3}: reward={:>10.2}, dones={:>3}, {}ms",
                                m.epoch, m.mean_reward, m.done_count, m.wall_time_ms
                            );
                        }
                    });

                    if self.verbose {
                        // Print `best` (max mean_reward across epochs) alongside `final`
                        // (last epoch's mean_reward). The rematch's statistical pipeline
                        // consumes `best_reward()`, not the final, so a reader watching
                        // the live log must see both to interpret the run correctly.
                        // See `docs/studies/ml_chassis_refactor/src/50-d2c-sr-rematch-writeup.md`
                        // for the case study where reading only `final` misled the author.
                        let final_reward = metrics.last().map_or(f64::NAN, |m| m.mean_reward);
                        let best_reward = metrics
                            .iter()
                            .map(|m| m.mean_reward)
                            .fold(f64::NEG_INFINITY, f64::max);
                        let total_dones: usize = metrics.iter().map(|m| m.done_count).sum();
                        eprintln!(
                            "[{name}] done — best={best_reward:.2}, final={final_reward:.2}, {total_dones} dones, {:.1}s",
                            t0.elapsed().as_secs_f64()
                        );
                    }

                    let artifact = algorithm.policy_artifact();
                    let best_artifact = algorithm.best_artifact();

                    // Strict `>` matches BestTracker (§3.3): ties keep the
                    // earlier epoch.  Do NOT replace with
                    // `RunResult::best_reward()` — that uses
                    // `partial_cmp`→`Ordering::Equal` fallback and would
                    // pick the later epoch under a tie, diverging from
                    // `best_epoch`'s "earliest tied epoch" invariant.
                    let (best_epoch, best_reward) = {
                        let mut bi = 0;
                        let mut br = f64::NEG_INFINITY;
                        for (i, m) in metrics.iter().enumerate() {
                            if m.mean_reward > br {
                                br = m.mean_reward;
                                bi = i;
                            }
                        }
                        if metrics.is_empty() {
                            (0, None)
                        } else if br.is_finite() {
                            (bi, Some(br))
                        } else {
                            (0, None)
                        }
                    };

                    let provenance = TrainingProvenance {
                        algorithm: name.to_string(),
                        task: task.name().to_string(),
                        seed,
                        epochs_trained: metrics.len(),
                        final_reward: metrics.last().map_or(0.0, |m| m.mean_reward),
                        best_reward,
                        best_epoch,
                        total_steps: metrics.iter().map(|m| m.total_steps).sum(),
                        wall_time_ms: metrics.iter().map(|m| m.wall_time_ms).sum(),
                        timestamp: now_iso8601(),
                        hyperparams: BTreeMap::new(),
                        metrics: metrics.clone(),
                        parent: None,
                    };

                    let artifact = artifact.with_provenance(provenance.clone());
                    let best_artifact = best_artifact.with_provenance(provenance);

                    runs.push(RunResult {
                        task_name: task.name().to_string(),
                        algorithm_name: name.to_string(),
                        replicate_index,
                        metrics,
                        artifact,
                        best_artifact,
                    });
                }
            }
        }

        Ok(CompetitionResult { runs })
    }
}

// ── tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::let_underscore_must_use
)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use crate::artifact::{PolicyArtifact, TrainingCheckpoint};
    use crate::linear::LinearPolicy;
    use crate::policy::Policy;
    use crate::test_stock_tasks::{reaching_2dof, reaching_6dof};
    use crate::vec_env::VecEnv;

    /// Minimal artifact for tests that don't care about artifact content.
    fn dummy_artifact() -> PolicyArtifact {
        PolicyArtifact::from_policy(&LinearPolicy::new(1, 1, &[1.0]))
    }

    /// Minimal Algorithm implementation for testing.
    struct MockAlgorithm {
        name: &'static str,
        policy: Box<dyn Policy>,
        best: crate::best_tracker::BestTracker,
    }

    impl MockAlgorithm {
        fn new(name: &'static str) -> Self {
            let policy = Box::new(LinearPolicy::new(1, 1, &[1.0]));
            let best = crate::best_tracker::BestTracker::new(policy.params());
            Self { name, policy, best }
        }
    }

    impl Algorithm for MockAlgorithm {
        fn name(&self) -> &'static str {
            self.name
        }

        fn train(
            &mut self,
            env: &mut VecEnv,
            budget: TrainingBudget,
            _seed: u64,
            on_epoch: &dyn Fn(&EpochMetrics),
        ) -> Vec<EpochMetrics> {
            let n_epochs = match budget {
                TrainingBudget::Epochs(n) => n,
                TrainingBudget::Steps(_) => 1,
            };

            // Do one real step per epoch to prove the env works.
            let _ = env.reset_all();
            let act_dim = env.model().nu;
            let n_envs = env.n_envs();

            let mut metrics = Vec::with_capacity(n_epochs);
            for epoch in 0..n_epochs {
                let actions = crate::tensor::Tensor::zeros(&[n_envs, act_dim]);
                let _ = env.step(&actions);
                let mean_reward = -1.0;
                self.best
                    .maybe_update(epoch, mean_reward, self.policy.params());
                let em = EpochMetrics {
                    epoch,
                    mean_reward,
                    done_count: 0,
                    total_steps: n_envs,
                    wall_time_ms: 0,
                    extra: BTreeMap::new(),
                };
                on_epoch(&em);
                metrics.push(em);
            }
            metrics
        }

        fn policy_artifact(&self) -> PolicyArtifact {
            PolicyArtifact::from_policy(&*self.policy)
        }

        fn best_artifact(&self) -> PolicyArtifact {
            self.best.to_artifact(self.policy.descriptor())
        }

        fn checkpoint(&self) -> TrainingCheckpoint {
            let (best_params, best_reward, best_epoch) = self.best.to_checkpoint();
            TrainingCheckpoint {
                algorithm_name: self.name.into(),
                policy_artifact: self.policy_artifact(),
                critics: vec![],
                optimizer_states: vec![],
                algorithm_state: BTreeMap::new(),
                best_params: Some(best_params),
                best_reward,
                best_epoch,
            }
        }
    }

    // ── Competition basics ────────────────────────────────────────────

    #[test]
    fn competition_runs_all_pairs() {
        let comp = Competition::new(2, TrainingBudget::Epochs(3), 42);
        let tasks = [reaching_2dof()];

        let builder_a: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("AlgoA"));
        let builder_b: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("AlgoB"));

        let result = comp.run(&tasks, &[builder_a, builder_b]).unwrap();
        assert_eq!(result.runs.len(), 2); // 1 task × 2 algorithms

        let a = result.find("reaching-2dof", "AlgoA").unwrap();
        assert_eq!(a.metrics.len(), 3); // 3 epochs

        let b = result.find("reaching-2dof", "AlgoB").unwrap();
        assert_eq!(b.metrics.len(), 3);
    }

    #[test]
    fn competition_empty_tasks() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("A"));
        let result = comp.run(&[], &[builder]).unwrap();
        assert!(result.runs.is_empty());
    }

    #[test]
    fn competition_empty_builders() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let tasks = [reaching_2dof()];
        let builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>] = &[];
        let result = comp.run(&tasks, builders).unwrap();
        assert!(result.runs.is_empty());
    }

    #[test]
    fn competition_multiple_tasks() {
        let comp = Competition::new(2, TrainingBudget::Epochs(2), 42);
        let tasks = [reaching_2dof(), reaching_6dof()];

        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Mock"));

        let result = comp.run(&tasks, &[builder]).unwrap();
        assert_eq!(result.runs.len(), 2); // 2 tasks × 1 algorithm

        assert!(result.find("reaching-2dof", "Mock").is_some());
        assert!(result.find("reaching-6dof", "Mock").is_some());
    }

    // ── RunResult helpers ─────────────────────────────────────────────

    #[test]
    fn run_result_final_reward() {
        let run = RunResult {
            task_name: "t".into(),
            algorithm_name: "a".into(),
            replicate_index: 0,
            metrics: vec![
                EpochMetrics {
                    epoch: 0,
                    mean_reward: -5.0,
                    done_count: 0,
                    total_steps: 10,
                    wall_time_ms: 0,
                    extra: BTreeMap::new(),
                },
                EpochMetrics {
                    epoch: 1,
                    mean_reward: -2.0,
                    done_count: 3,
                    total_steps: 10,
                    wall_time_ms: 0,
                    extra: BTreeMap::new(),
                },
            ],
            artifact: dummy_artifact(),
            best_artifact: dummy_artifact(),
        };
        assert_eq!(run.final_reward(), Some(-2.0));
        assert_eq!(run.total_dones(), 3);
    }

    #[test]
    fn run_result_empty_metrics() {
        let run = RunResult {
            task_name: "t".into(),
            algorithm_name: "a".into(),
            replicate_index: 0,
            metrics: vec![],
            artifact: dummy_artifact(),
            best_artifact: dummy_artifact(),
        };
        assert_eq!(run.final_reward(), None);
        assert_eq!(run.total_dones(), 0);
    }

    #[test]
    fn assert_finite_passes() {
        let run = RunResult {
            task_name: "t".into(),
            algorithm_name: "a".into(),
            replicate_index: 0,
            metrics: vec![EpochMetrics {
                epoch: 0,
                mean_reward: -5.0,
                done_count: 0,
                total_steps: 10,
                wall_time_ms: 0,
                extra: BTreeMap::new(),
            }],
            artifact: dummy_artifact(),
            best_artifact: dummy_artifact(),
        };
        run.assert_finite(); // should not panic
    }

    #[test]
    #[should_panic(expected = "non-finite reward")]
    fn assert_finite_catches_nan() {
        let run = RunResult {
            task_name: "t".into(),
            algorithm_name: "a".into(),
            replicate_index: 0,
            metrics: vec![EpochMetrics {
                epoch: 0,
                mean_reward: f64::NAN,
                done_count: 0,
                total_steps: 10,
                wall_time_ms: 0,
                extra: BTreeMap::new(),
            }],
            artifact: dummy_artifact(),
            best_artifact: dummy_artifact(),
        };
        run.assert_finite();
    }

    #[test]
    #[should_panic(expected = "non-finite reward")]
    fn assert_finite_catches_inf() {
        let run = RunResult {
            task_name: "t".into(),
            algorithm_name: "a".into(),
            replicate_index: 0,
            metrics: vec![EpochMetrics {
                epoch: 0,
                mean_reward: f64::INFINITY,
                done_count: 0,
                total_steps: 10,
                wall_time_ms: 0,
                extra: BTreeMap::new(),
            }],
            artifact: dummy_artifact(),
            best_artifact: dummy_artifact(),
        };
        run.assert_finite();
    }

    // ── CompetitionResult helpers ─────────────────────────────────────

    #[test]
    fn competition_result_for_task() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let tasks = [reaching_2dof()];

        let b1: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> = &|_| Box::new(MockAlgorithm::new("A"));
        let b2: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> = &|_| Box::new(MockAlgorithm::new("B"));

        let result = comp.run(&tasks, &[b1, b2]).unwrap();
        let for_task = result.for_task("reaching-2dof");
        assert_eq!(for_task.len(), 2);
    }

    #[test]
    fn competition_result_for_algorithm() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let tasks = [reaching_2dof(), reaching_6dof()];

        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_| Box::new(MockAlgorithm::new("Mock"));

        let result = comp.run(&tasks, &[builder]).unwrap();
        let for_algo = result.for_algorithm("Mock");
        assert_eq!(for_algo.len(), 2);
    }

    // ── Phase 3: Artifact integration ────────────────────────────────

    #[test]
    fn competition_artifacts_have_provenance() {
        let seed = 99;
        let comp = Competition::new(2, TrainingBudget::Epochs(3), seed);
        let tasks = [reaching_2dof()];

        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("AlgoP"));

        let result = comp.run(&tasks, &[builder]).unwrap();
        let run = result.find("reaching-2dof", "AlgoP").unwrap();

        let prov = run
            .artifact
            .provenance
            .as_ref()
            .expect("artifact should have provenance");
        assert_eq!(prov.algorithm, "AlgoP");
        assert_eq!(prov.task, "reaching-2dof");
        assert_eq!(prov.seed, seed);
        assert_eq!(prov.epochs_trained, 3);
        assert_eq!(prov.metrics.len(), 3);
        assert!(prov.parent.is_none());
        // Timestamp should be a valid ISO 8601 string (basic sanity check).
        assert!(prov.timestamp.ends_with('Z'));
        assert!(prov.timestamp.contains('T'));
    }

    #[test]
    fn competition_save_artifacts() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let tasks = [reaching_2dof()];

        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Saver"));

        let result = comp.run(&tasks, &[builder]).unwrap();

        let dir = std::env::temp_dir().join("ml_bridge_save_test");
        let _ = std::fs::remove_dir_all(&dir); // clean slate
        result.save_artifacts(&dir).unwrap();

        let path = dir.join("reaching-2dof_Saver.artifact.json");
        assert!(path.exists(), "artifact file should exist");

        // Round-trip: load it back and verify.
        let loaded = PolicyArtifact::load(&path).unwrap();
        assert!(loaded.provenance.is_some());
        assert_eq!(loaded.provenance.as_ref().unwrap().algorithm, "Saver");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn competition_best_for_task() {
        let comp = Competition::new(2, TrainingBudget::Epochs(3), 42);
        let tasks = [reaching_2dof()];

        // Both use MockAlgorithm (same reward), but we can still verify
        // best_for_task returns Some with correct provenance.
        let builder_a: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("BestA"));
        let builder_b: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("BestB"));

        let result = comp.run(&tasks, &[builder_a, builder_b]).unwrap();

        let best = result.best_for_task("reaching-2dof");
        assert!(best.is_some());
        // Both mocks produce identical reward, so best is one of them.
        let prov = best.unwrap().provenance.as_ref().unwrap();
        assert_eq!(prov.task, "reaching-2dof");

        // Non-existent task returns None.
        assert!(result.best_for_task("no-such-task").is_none());
    }

    // ── Phase 2: Best-policy tracking ────────────────────────────────

    #[test]
    fn run_result_best_reward_matches_manual_scan() {
        let run = RunResult {
            task_name: "t".into(),
            algorithm_name: "a".into(),
            replicate_index: 0,
            metrics: vec![
                EpochMetrics {
                    epoch: 0,
                    mean_reward: -5.0,
                    done_count: 0,
                    total_steps: 10,
                    wall_time_ms: 0,
                    extra: BTreeMap::new(),
                },
                EpochMetrics {
                    epoch: 1,
                    mean_reward: -1.0,
                    done_count: 0,
                    total_steps: 10,
                    wall_time_ms: 0,
                    extra: BTreeMap::new(),
                },
                EpochMetrics {
                    epoch: 2,
                    mean_reward: -3.0,
                    done_count: 0,
                    total_steps: 10,
                    wall_time_ms: 0,
                    extra: BTreeMap::new(),
                },
            ],
            artifact: dummy_artifact(),
            best_artifact: dummy_artifact(),
        };
        // Best is epoch 1 (-1.0), not final epoch 2 (-3.0).
        assert_eq!(run.best_reward(), Some(-1.0));
        assert_eq!(run.final_reward(), Some(-3.0));
    }

    #[test]
    fn provenance_best_fields_serde_round_trip() {
        let comp = Competition::new(2, TrainingBudget::Epochs(5), 42);
        let tasks = [reaching_2dof()];

        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Serde"));

        let result = comp.run(&tasks, &[builder]).unwrap();
        let run = result.find("reaching-2dof", "Serde").unwrap();

        let prov = run.artifact.provenance.as_ref().unwrap();
        // best_reward should be Some (mock produces finite rewards).
        assert!(prov.best_reward.is_some());

        // Round-trip through JSON.
        let json = serde_json::to_string(&prov).unwrap();
        let loaded: TrainingProvenance = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded.best_reward, prov.best_reward);
        assert_eq!(loaded.best_epoch, prov.best_epoch);
    }

    #[test]
    fn save_artifacts_writes_best_files() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let tasks = [reaching_2dof()];

        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("BestSave"));

        let result = comp.run(&tasks, &[builder]).unwrap();

        let dir = std::env::temp_dir().join("ml_bridge_best_save_test");
        let _ = std::fs::remove_dir_all(&dir);
        result.save_artifacts(&dir).unwrap();

        let final_path = dir.join("reaching-2dof_BestSave.artifact.json");
        let best_path = dir.join("reaching-2dof_BestSave.best.artifact.json");
        assert!(final_path.exists(), "final artifact file should exist");
        assert!(best_path.exists(), "best artifact file should exist");

        // Both should load and have provenance.
        let loaded_best = PolicyArtifact::load(&best_path).unwrap();
        assert!(loaded_best.provenance.is_some());
        assert_eq!(
            loaded_best.provenance.as_ref().unwrap().algorithm,
            "BestSave"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    // ── PR 2a: run_replicates + replicate_index ──────────────────────

    #[test]
    fn run_replicates_flat_shape() {
        let comp = Competition::new(2, TrainingBudget::Epochs(2), 42);
        let tasks = [reaching_2dof(), reaching_6dof()];

        let b1: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("MockA"));
        let b2: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("MockB"));

        let result = comp
            .run_replicates(&tasks, &[b1, b2], &[10, 20, 30])
            .unwrap();
        // 2 tasks × 2 builders × 3 seeds
        assert_eq!(result.runs.len(), 12);
    }

    #[test]
    fn run_replicates_replicate_index_monotonic() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Mock"));

        let result = comp
            .run_replicates(&tasks, &[builder], &[100, 200, 300])
            .unwrap();
        let mock_runs: Vec<&RunResult> = result
            .runs
            .iter()
            .filter(|r| r.task_name == "reaching-2dof" && r.algorithm_name == "Mock")
            .collect();
        assert_eq!(mock_runs.len(), 3);
        // seeds-outermost loop ordering: indices appear as [0, 1, 2].
        let indices: Vec<usize> = mock_runs.iter().map(|r| r.replicate_index).collect();
        assert_eq!(indices, vec![0, 1, 2]);
    }

    #[test]
    fn run_replicates_preserves_strict_gt_tie_breaking() {
        // MockAlgorithm emits `mean_reward = -1.0` on every epoch, so with
        // >= 2 epochs every epoch is tied at the same value.  Under strict
        // `>` tie-breaking (the inline scan at competition.rs :363-379 in
        // run_replicates's body), the earliest epoch wins and
        // provenance.best_epoch == 0.  A refactor that replaced the inline
        // scan with `RunResult::best_reward()`'s `partial_cmp`→Equal
        // fallback would pick the last epoch (`n_epochs - 1`) and fail
        // this assertion.  This guards the invariant Ch 41 §4.1 named as
        // PR 2a's load-bearing review gate.
        let comp = Competition::new(1, TrainingBudget::Epochs(3), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("TieBreaker"));

        let result = comp.run_replicates(&tasks, &[builder], &[42]).unwrap();
        let run = result.find("reaching-2dof", "TieBreaker").unwrap();
        let prov = run
            .artifact
            .provenance
            .as_ref()
            .expect("run should carry provenance");

        // All three epochs have mean_reward == -1.0 (MockAlgorithm's
        // constant), so the inline scan's strict `>` picks epoch 0.
        assert_eq!(
            prov.best_epoch, 0,
            "tied epochs should resolve to the earliest under strict `>`; \
             got best_epoch = {}",
            prov.best_epoch
        );
        // And provenance.best_reward should match the tied value.
        assert_eq!(prov.best_reward, Some(-1.0));
    }

    #[test]
    fn run_replicates_fresh_env_per_pair() {
        // Three replicates on the same (task, builder) pair should each
        // see an independently-reset env — no cross-replicate state leakage
        // through a stale VecEnv would produce inconsistent metrics.len().
        let comp = Competition::new(2, TrainingBudget::Epochs(4), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("FreshEnv"));

        let result = comp.run_replicates(&tasks, &[builder], &[1, 2, 3]).unwrap();
        assert_eq!(result.runs.len(), 3);
        for r in &result.runs {
            assert_eq!(
                r.metrics.len(),
                4,
                "replicate {} has stale state (metrics.len() = {})",
                r.replicate_index,
                r.metrics.len()
            );
        }
    }

    #[test]
    fn run_still_works_single_seed() {
        // `run` is now a one-line wrapper over `run_replicates(..., &[self.seed])`;
        // verify every result carries replicate_index == 0 and that the
        // existing single-seed contract is preserved exactly.
        let comp = Competition::new(2, TrainingBudget::Epochs(2), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Wrap"));

        let result = comp.run(&tasks, &[builder]).unwrap();
        assert_eq!(result.runs.len(), 1);
        for r in &result.runs {
            assert_eq!(r.replicate_index, 0);
        }
    }

    #[test]
    fn find_replicate_returns_specific_seed() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Finder"));

        let result = comp
            .run_replicates(&tasks, &[builder], &[10, 20, 30])
            .unwrap();
        // Specific replicate lookup.
        let r1 = result.find_replicate("reaching-2dof", "Finder", 1);
        assert!(r1.is_some());
        assert_eq!(r1.unwrap().replicate_index, 1);
        // Missing replicate.
        assert!(
            result
                .find_replicate("reaching-2dof", "Finder", 99)
                .is_none()
        );
        // Missing (task, algo) pair.
        assert!(result.find_replicate("no-such-task", "Finder", 0).is_none());
    }

    // ── PR 2a: SeedSummary + aggregation helpers ─────────────────────

    #[test]
    fn replicate_best_rewards_flat_filter() {
        let comp = Competition::new(2, TrainingBudget::Epochs(2), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Rwd"));

        let result = comp.run_replicates(&tasks, &[builder], &[1, 2, 3]).unwrap();
        let rewards = result.replicate_best_rewards("reaching-2dof", "Rwd");
        // MockAlgorithm always produces finite rewards, so every
        // replicate survives the silent filter-out.
        assert_eq!(rewards.len(), 3);
        for r in &rewards {
            assert!(r.is_finite(), "reward should be finite: {r}");
        }
    }

    #[test]
    fn replicate_final_rewards_flat_filter() {
        // Parallel to `replicate_best_rewards_flat_filter`.  Ch 51 §2.1
        // added `replicate_final_rewards` for the dual-metric rematch
        // amendment; the shape contract matches the best variant.
        let comp = Competition::new(2, TrainingBudget::Epochs(2), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Rwd"));

        let result = comp.run_replicates(&tasks, &[builder], &[1, 2, 3]).unwrap();
        let finals = result.replicate_final_rewards("reaching-2dof", "Rwd");
        let bests = result.replicate_best_rewards("reaching-2dof", "Rwd");
        assert_eq!(finals.len(), 3);
        assert_eq!(
            finals.len(),
            bests.len(),
            "best and final accessors must return the same length for a given (task, algo) pair",
        );
        for r in &finals {
            assert!(r.is_finite(), "final reward should be finite: {r}");
        }
    }

    #[test]
    fn describe_returns_seed_summary() {
        let comp = Competition::new(2, TrainingBudget::Epochs(2), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Desc"));

        let result = comp.run_replicates(&tasks, &[builder], &[1, 2, 3]).unwrap();
        let summary = result
            .describe("reaching-2dof", "Desc")
            .expect("describe should return Some for a 3-replicate set");
        assert_eq!(summary.n, 3);
        assert!(summary.mean.is_finite());
        assert!(summary.std_dev >= 0.0);
    }

    #[test]
    fn describe_none_for_missing_pair() {
        let comp = Competition::new(2, TrainingBudget::Epochs(1), 42);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_task| Box::new(MockAlgorithm::new("Present"));

        let result = comp.run_replicates(&tasks, &[builder], &[0]).unwrap();
        assert!(result.describe("no-such-task", "Present").is_none());
        assert!(result.describe("reaching-2dof", "no-such-algo").is_none());
    }

    #[test]
    fn seed_summary_from_rewards_n1_returns_zero_stddev() {
        let summary =
            SeedSummary::from_rewards(&[1.0]).expect("single-element slice should return Some");
        assert_eq!(summary.n, 1);
        assert!((summary.mean - 1.0).abs() < 1e-12);
        // `from_rewards` explicitly returns `0.0` for the `n == 1`
        // case — a hard-coded literal, not a computed value — so a
        // bit-equality check against `0.0_f64` is the right shape.
        assert_eq!(summary.std_dev.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn seed_summary_from_rewards_empty_returns_none() {
        assert!(SeedSummary::from_rewards(&[]).is_none());
    }

    #[test]
    fn seed_summary_from_rewards_bessel_correction() {
        // [1, 2, 3]: mean = 2.0, variance = ((-1)^2 + 0 + 1^2) / (3-1)
        //          = 2 / 2 = 1.0, std_dev = 1.0.  Sample std (Bessel's
        // correction, n-1 denominator), not population std (which would
        // give sqrt(2/3) ≈ 0.8165).
        let summary = SeedSummary::from_rewards(&[1.0, 2.0, 3.0]).unwrap();
        assert_eq!(summary.n, 3);
        assert!((summary.mean - 2.0).abs() < 1e-12);
        assert!(
            (summary.std_dev - 1.0).abs() < 1e-12,
            "expected sample std = 1.0 (Bessel), got {}",
            summary.std_dev
        );
    }

    #[test]
    fn replicate_best_rewards_silent_filter_out() {
        // ZeroEpochAlgorithm produces an empty metrics Vec, which makes
        // RunResult::best_reward() return None — the replicate is
        // silently filtered out by replicate_best_rewards per Ch 24
        // §4.7.  FiniteAlgorithm produces MockAlgorithm's usual finite
        // rewards.  Alternating the two across three seeds gives two
        // surviving rewards out of three seeds.
        //
        // Scoped allows are required because MockAlgorithm's test-only
        // `unwrap_used`/`expect_used` allows extend to our variant.
        struct ZeroEpochAlgorithm {
            policy: Box<dyn Policy>,
            best: crate::best_tracker::BestTracker,
        }

        impl Algorithm for ZeroEpochAlgorithm {
            fn name(&self) -> &'static str {
                "Mixed"
            }

            fn train(
                &mut self,
                _env: &mut VecEnv,
                _budget: TrainingBudget,
                _seed: u64,
                _on_epoch: &dyn Fn(&EpochMetrics),
            ) -> Vec<EpochMetrics> {
                // Empty metrics → RunResult::best_reward() is None.
                Vec::new()
            }

            fn policy_artifact(&self) -> PolicyArtifact {
                PolicyArtifact::from_policy(&*self.policy)
            }

            fn best_artifact(&self) -> PolicyArtifact {
                self.best.to_artifact(self.policy.descriptor())
            }

            fn checkpoint(&self) -> TrainingCheckpoint {
                let (best_params, best_reward, best_epoch) = self.best.to_checkpoint();
                TrainingCheckpoint {
                    algorithm_name: self.name().into(),
                    policy_artifact: self.policy_artifact(),
                    critics: vec![],
                    optimizer_states: vec![],
                    algorithm_state: BTreeMap::new(),
                    best_params: Some(best_params),
                    best_reward,
                    best_epoch,
                }
            }
        }

        // Seed 0 and seed 2 produce the finite-reward variant; seed 1
        // produces the zero-epoch variant.  The factory dispatches on
        // `self.seed` — but Competition's builder closure only sees
        // the `TaskConfig`, not the seed, so we dispatch via a
        // RefCell-backed counter instead.
        let call_count = std::cell::RefCell::new(0usize);
        let comp = Competition::new(1, TrainingBudget::Epochs(2), 0);
        let tasks = [reaching_2dof()];
        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> = &|_task| {
            let i = {
                let mut c = call_count.borrow_mut();
                let v = *c;
                *c += 1;
                v
            };
            if i == 1 {
                // Middle replicate: zero-epoch, gets filtered out.
                let policy: Box<dyn Policy> = Box::new(LinearPolicy::new(1, 1, &[1.0]));
                let best = crate::best_tracker::BestTracker::new(policy.params());
                Box::new(ZeroEpochAlgorithm { policy, best })
            } else {
                Box::new(MockAlgorithm::new("Mixed"))
            }
        };

        let result = comp
            .run_replicates(&tasks, &[builder], &[10, 20, 30])
            .unwrap();
        let rewards = result.replicate_best_rewards("reaching-2dof", "Mixed");
        // 3 seeds × 1 empty = 2 surviving rewards.
        assert_eq!(
            rewards.len(),
            2,
            "silent filter-out should drop the zero-epoch replicate; got {} rewards",
            rewards.len()
        );
        // And describe should report `n == 2`, not `n == 3`.
        let summary = result.describe("reaching-2dof", "Mixed").unwrap();
        assert_eq!(summary.n, 2);
    }

    #[test]
    fn validate_rejects_non_finite_best_reward() {
        let mut a = dummy_artifact();
        a.provenance = Some(TrainingProvenance {
            algorithm: "test".into(),
            task: "test".into(),
            seed: 0,
            epochs_trained: 1,
            final_reward: -1.0,
            total_steps: 100,
            wall_time_ms: 50,
            timestamp: "2026-01-01T00:00:00Z".into(),
            hyperparams: BTreeMap::new(),
            metrics: vec![],
            best_reward: Some(f64::INFINITY),
            best_epoch: 0,
            parent: None,
        });
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::NonFiniteValue { .. })
        ));
    }
}
