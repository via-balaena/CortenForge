//! Competition runner — runs algorithms on tasks, collects standardized results.
//!
//! The [`Competition`] runner is the dynamometer for the ML layer.  It takes a
//! set of [`TaskConfig`]s and a set of algorithm builders, runs every
//! (task, algorithm) combination, and returns structured [`RunResult`]s that
//! competition tests can assert against.
//!
//! Each (task, algorithm) pair gets a fresh [`VecEnv`](crate::VecEnv) — no
//! cross-contamination between runs.

use crate::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
use crate::error::EnvError;
use crate::task::TaskConfig;

// ── RunResult ───────────────────────────────────────────────────────────────

/// Result from running one algorithm on one task.
#[derive(Debug, Clone)]
pub struct RunResult {
    /// Task name (from [`TaskConfig::name()`]).
    pub task_name: String,
    /// Algorithm name (from [`Algorithm::name()`]).
    pub algorithm_name: String,
    /// Per-epoch metrics from [`Algorithm::train()`].
    pub metrics: Vec<EpochMetrics>,
}

impl RunResult {
    /// Final epoch's mean reward, or `None` if no epochs ran.
    #[must_use]
    pub fn final_reward(&self) -> Option<f64> {
        self.metrics.last().map(|m| m.mean_reward)
    }

    /// Total done triggers across all epochs.
    #[must_use]
    pub fn total_dones(&self) -> usize {
        self.metrics.iter().map(|m| m.done_count).sum()
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
}

impl Competition {
    /// Create a competition runner.
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
        }
    }

    /// Run all algorithm builders on all tasks.
    ///
    /// Returns one [`RunResult`] per (task, builder) pair, in task-major order.
    /// Each pair gets a fresh [`VecEnv`](crate::VecEnv).
    ///
    /// # Errors
    ///
    /// Returns [`EnvError`] if any [`TaskConfig::build_vec_env()`] fails.
    pub fn run(
        &self,
        tasks: &[TaskConfig],
        builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
    ) -> Result<CompetitionResult, EnvError> {
        let mut runs = Vec::with_capacity(tasks.len() * builders.len());

        for task in tasks {
            for builder in builders {
                let mut env = task.build_vec_env(self.n_envs)?;
                let mut algorithm = builder(task);
                let metrics = algorithm.train(&mut env, self.budget, self.seed);

                runs.push(RunResult {
                    task_name: task.name().to_string(),
                    algorithm_name: algorithm.name().to_string(),
                    metrics,
                });
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
    use crate::task::reaching_2dof;
    use crate::vec_env::VecEnv;

    /// Minimal Algorithm implementation for testing.
    struct MockAlgorithm {
        name: String,
    }

    impl MockAlgorithm {
        fn new(name: &str) -> Self {
            Self {
                name: name.to_string(),
            }
        }
    }

    impl Algorithm for MockAlgorithm {
        fn name(&self) -> &str {
            &self.name
        }

        fn train(
            &mut self,
            env: &mut VecEnv,
            budget: TrainingBudget,
            _seed: u64,
        ) -> Vec<EpochMetrics> {
            let n_epochs = match budget {
                TrainingBudget::Epochs(n) => n,
                TrainingBudget::Steps(_) => 1,
            };

            // Do one real step per epoch to prove the env works.
            let _ = env.reset_all();
            let act_dim = env.model().nu;
            let n_envs = env.n_envs();

            (0..n_epochs)
                .map(|epoch| {
                    let actions = crate::tensor::Tensor::zeros(&[n_envs, act_dim]);
                    let _ = env.step(&actions);
                    EpochMetrics {
                        epoch,
                        mean_reward: -1.0,
                        done_count: 0,
                        total_steps: n_envs,
                        wall_time_ms: 0,
                        extra: BTreeMap::new(),
                    }
                })
                .collect()
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
        let tasks = [reaching_2dof(), crate::task::reaching_6dof()];

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
        };
        assert_eq!(run.final_reward(), Some(-2.0));
        assert_eq!(run.total_dones(), 3);
    }

    #[test]
    fn run_result_empty_metrics() {
        let run = RunResult {
            task_name: "t".into(),
            algorithm_name: "a".into(),
            metrics: vec![],
        };
        assert_eq!(run.final_reward(), None);
        assert_eq!(run.total_dones(), 0);
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
        let tasks = [reaching_2dof(), crate::task::reaching_6dof()];

        let builder: &dyn Fn(&TaskConfig) -> Box<dyn Algorithm> =
            &|_| Box::new(MockAlgorithm::new("Mock"));

        let result = comp.run(&tasks, &[builder]).unwrap();
        let for_algo = result.for_algorithm("Mock");
        assert_eq!(for_algo.len(), 2);
    }
}
