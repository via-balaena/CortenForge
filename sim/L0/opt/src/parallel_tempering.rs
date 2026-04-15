//! Parallel Tempering — K Simulated Annealing chains at
//! geometric temperature ladder with Metropolis-criterion adjacent
//! swap moves.
//!
//! This module implements Ch 53 §2.3's Parallel Tempering class
//! commitment.  K chains run per-chain Metropolis updates on the
//! same `VecEnv` (evaluated sequentially — `VecEnv`'s
//! `collect_episodic_rollout` is stateless between calls), and
//! after each epoch's round of per-chain proposals an adjacent
//! swap pass attempts K-1 swaps between chains (i, i+1) under the
//! Boltzmann-weighted replica-exchange criterion.  The
//! lowest-temperature chain (chain index 0) is the returned
//! artifact — the exploit-end of the ladder — and is the only
//! chain whose fitness feeds `EpochMetrics::mean_reward`.
//!
//! # Class commitment qualification (Ch 53 §2.3)
//!
//! Ch 53 §2.3 names K, the temperature ladder, and the swap
//! interval as PT-code-commit sub-decisions.  This module picks:
//!
//! - **K = 4 chains** (default).  Small enough that total-compute
//!   parity at `16_000_000 / K = 4_000_000` env steps per chain
//!   still leaves each chain with `4_000_000 / (32 * 5000) = 25`
//!   Metropolis proposals, which is the minimum count above which
//!   the swap dynamics can exchange information more than once
//!   per chain.  Larger K tightens the budget per chain below
//!   that floor.
//! - **Geometric temperature ladder** `T_k = T_min *
//!   (T_max / T_min)^(k/(K-1))` for `k` in `0..K`, with chain 0
//!   at `T_min` and chain K-1 at `T_max`.  Geometric is the
//!   textbook PT ladder spacing because constant-ratio
//!   temperature gaps give approximately uniform swap acceptance
//!   across the ladder when the Boltzmann-weight overlap is the
//!   dominant term.
//! - **`T_min = 0.5` and `T_max = 50.0`** — the cold endpoint of
//!   basic SA's Ch 42 §4.5 schedule (`T_min = T_0 * alpha^100` on
//!   the geometric cooling schedule) and basic SA's starting
//!   temperature respectively.  The PT ladder therefore
//!   straddles basic SA's full dynamic range in a fixed-
//!   temperature form (PT conventionally does *not* cool each
//!   chain during training — that is what distinguishes PT from
//!   K parallel single-chain SAs).
//! - **Swap every epoch.** The tightest swap interval available
//!   under the per-epoch Metropolis grain.  Ch 53 §2.3 names
//!   "regular intervals" and "swap every epoch" was the explicit
//!   instruction handed to this commit.
//!
//! # Why `name()` returns `"SA"`
//!
//! Same rationale as `richer_sa.rs`: `sim_opt::analysis::
//! run_rematch` extracts replicates by the hardcoded algorithm
//! labels `"CEM"` and `"SA"`, and Ch 53 §2.4's measurement
//! commitment binds the follow-ups to the unchanged driver.  The
//! cleanest honoring is `Pt::name() == "SA"` with the fixture
//! file name and header identifying the variant.  See
//! `sim/L0/opt/tests/d2c_sr_rematch_pt.rs`.

use std::collections::BTreeMap;
use std::time::Instant;

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use sim_ml_chassis::{
    Algorithm, CURRENT_VERSION, EpochMetrics, Policy, PolicyArtifact, TrainingBudget,
    TrainingCheckpoint, VecEnv, collect_episodic_rollout,
};

// ── Hyperparameters ──────────────────────────────────────────────────────

/// Parallel Tempering hyperparameters.
#[derive(Debug, Clone, Copy)]
pub struct PtHyperparams {
    /// Number of chains in the ladder.  Must be at least 1.
    /// `K = 1` degenerates to basic SA with no swap step and is
    /// permitted only as a sanity check.
    pub n_chains: usize,
    /// Lowest temperature in the ladder, assigned to chain 0.
    /// Chain 0 is the returned artifact, so this is the
    /// "exploit" end of the ladder.
    pub t_min: f64,
    /// Highest temperature in the ladder, assigned to chain
    /// `n_chains - 1`.  This is the "explore" end.
    pub t_max: f64,
    /// Gaussian proposal standard deviation used by every chain.
    /// PT does not adapt the proposal width at this commit — the
    /// richer-proposal adaptation is a different Ch 53 §2.2
    /// variant.
    pub proposal_std: f64,
    /// Maximum environment steps per episode.  Must match CEM's
    /// and basic SA's `max_episode_steps` for compute parity.
    pub max_episode_steps: usize,
    /// Epochs between swap-pass attempts.  `1` runs a swap pass
    /// every epoch.  Larger values reduce swap frequency.
    pub swap_interval_epochs: usize,
}

impl PtHyperparams {
    /// Ch 53 §2.3 defaults: K = 4, geometric ladder 0.5 → 50.0,
    /// `proposal_std = 0.5`, swap every epoch.
    #[must_use]
    pub const fn ch53_defaults(max_episode_steps: usize) -> Self {
        Self {
            n_chains: 4,
            t_min: 0.5,
            t_max: 50.0,
            proposal_std: 0.5,
            max_episode_steps,
            swap_interval_epochs: 1,
        }
    }
}

// ── Pt ───────────────────────────────────────────────────────────────────

/// Parallel Tempering algorithm.
///
/// # Parts
///
/// - [`Policy`] — base trait, used as a scratch slot.  Every
///   chain evaluates proposals on the *same* policy instance by
///   calling `set_params` immediately before `forward`, which
///   matches the pattern `evaluate_fitness` uses in basic SA
///   and CEM.  At the end of each epoch, `self.policy` is
///   synchronized to chain 0's current params so
///   `policy_artifact()` reflects the coldest-chain state.
pub struct Pt {
    policy: Box<dyn Policy>,
    hyperparams: PtHyperparams,
    /// Per-chain accepted params.  `chain_params[0]` is the
    /// coldest chain (chain 0).
    chain_params: Vec<Vec<f64>>,
    /// Per-chain accepted fitness.  Starts at `NEG_INFINITY`
    /// for every chain so the first per-chain proposal is
    /// force-accepted — same convention as basic SA.
    chain_fitness: Vec<f64>,
    /// Per-chain (fixed) temperature.  Set once at construction
    /// time from the geometric-ladder formula.
    temperatures: Vec<f64>,
    /// Best-seen params across all chains (monotone; never
    /// regresses).
    best_params: Vec<f64>,
    /// Best-seen fitness across all chains.
    best_fitness: f64,
    /// Epoch at which `best_fitness` was first observed.
    best_epoch: usize,
}

impl Pt {
    /// Construct a new `Pt` with K chains at a geometric
    /// temperature ladder.  Every chain starts at the policy's
    /// initial params.
    ///
    /// # Panics
    ///
    /// Panics if `hyperparams.n_chains == 0` or if `t_min <= 0`
    /// or `t_max < t_min`.  These are configuration bugs — the
    /// trait surface of PT cannot meaningfully run with them.
    #[must_use]
    pub fn new(policy: Box<dyn Policy>, hyperparams: PtHyperparams) -> Self {
        assert!(hyperparams.n_chains >= 1, "PT: n_chains must be >= 1");
        assert!(
            hyperparams.t_min > 0.0,
            "PT: t_min must be strictly positive (found {})",
            hyperparams.t_min
        );
        assert!(
            hyperparams.t_max >= hyperparams.t_min,
            "PT: t_max ({}) must be >= t_min ({})",
            hyperparams.t_max,
            hyperparams.t_min
        );

        let initial_params = policy.params().to_vec();
        let k = hyperparams.n_chains;
        let temperatures = geometric_ladder(hyperparams.t_min, hyperparams.t_max, k);
        let chain_params = vec![initial_params.clone(); k];
        let chain_fitness = vec![f64::NEG_INFINITY; k];

        Self {
            policy,
            hyperparams,
            chain_params,
            chain_fitness,
            temperatures,
            best_params: initial_params,
            best_fitness: f64::NEG_INFINITY,
            best_epoch: 0,
        }
    }

    /// The temperature ladder, exposed for test inspection.
    #[must_use]
    pub fn temperatures(&self) -> &[f64] {
        &self.temperatures
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────

/// Geometric temperature ladder.  `temperatures[0] = t_min`,
/// `temperatures[k-1] = t_max`, with constant ratio between
/// adjacent entries.  If `k == 1`, returns `[t_min]`.  If
/// `t_max == t_min`, returns K copies of `t_min`.
// cast_precision_loss: `k` is the chain count (K=4 in the rematch);
// `k as f64` is exact at these magnitudes.
#[allow(clippy::cast_precision_loss)]
fn geometric_ladder(t_min: f64, t_max: f64, k: usize) -> Vec<f64> {
    if k == 1 {
        return vec![t_min];
    }
    if (t_max - t_min).abs() < f64::EPSILON {
        return vec![t_min; k];
    }
    let ratio = (t_max / t_min).powf(1.0 / (k as f64 - 1.0));
    (0..k).map(|i| t_min * ratio.powf(i as f64)).collect()
}

fn randn(rng: &mut impl rand::Rng) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

// cast_precision_loss: rewards are averaged as `usize → f64`; the
// episode counts come from hyperparameters in the single-digit to
// low-thousands range, well within f64's exact-integer mantissa.
#[allow(clippy::cast_precision_loss)]
fn evaluate_fitness(
    env: &mut VecEnv,
    policy: &mut dyn Policy,
    params: &[f64],
    max_episode_steps: usize,
) -> f64 {
    let n_envs = env.n_envs();
    let rollout = collect_episodic_rollout(
        env,
        &mut |_env_idx, obs| {
            policy.set_params(params);
            policy.forward(obs)
        },
        max_episode_steps,
    );
    let total_reward: f64 = rollout
        .trajectories
        .iter()
        .map(|traj| traj.rewards.iter().sum::<f64>())
        .sum();
    total_reward / n_envs as f64
}

// ── Algorithm trait impl ─────────────────────────────────────────────────

impl Algorithm for Pt {
    fn name(&self) -> &'static str {
        // Intentional "SA" — see module doc comment.
        "SA"
    }

    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::needless_range_loop
    )]
    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics> {
        let mut rng = StdRng::seed_from_u64(seed);
        let n_envs = env.n_envs();
        let hp = self.hyperparams;
        let k = hp.n_chains;

        // Per-epoch work: K chain proposals at `n_envs *
        // max_episode_steps` env steps each.  Total-compute
        // parity (Ch 53 §2.3) therefore divides the budget by
        // `K * n_envs * max_episode_steps`, not just `n_envs *
        // max_episode_steps` as basic SA does.  This is the
        // headline cost of PT: at fixed budget, each chain gets
        // `1/K` as many Metropolis proposals as basic SA gets.
        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(s) => s / (k * n_envs * hp.max_episode_steps).max(1),
        };

        let mut metrics = Vec::with_capacity(n_epochs);

        for epoch in 0..n_epochs {
            let t0 = Instant::now();

            // 1. Per-chain proposal + Metropolis accept/reject.
            //    Chains are evaluated sequentially on the same
            //    VecEnv — `collect_episodic_rollout` resets
            //    internal trajectory state each call, so
            //    sequential evaluation does not cross-contaminate.
            let mut chain_accepts = vec![0u8; k];
            for chain_idx in 0..k {
                let sigma = hp.proposal_std;
                let proposed_params: Vec<f64> = self.chain_params[chain_idx]
                    .iter()
                    .map(|&p| sigma.mul_add(randn(&mut rng), p))
                    .collect();

                let proposed_fitness = evaluate_fitness(
                    env,
                    &mut *self.policy,
                    &proposed_params,
                    hp.max_episode_steps,
                );

                let t_k = self.temperatures[chain_idx];
                let delta = proposed_fitness - self.chain_fitness[chain_idx];
                let accept = if delta > 0.0 {
                    true
                } else if t_k > 0.0 {
                    let accept_prob = (delta / t_k).exp();
                    rng.random::<f64>() < accept_prob
                } else {
                    false
                };

                if accept {
                    self.chain_params[chain_idx] = proposed_params;
                    self.chain_fitness[chain_idx] = proposed_fitness;
                    chain_accepts[chain_idx] = 1;
                }

                // Best-across-all-chains tracker.  Strict `>`
                // keeps the earlier epoch on ties.
                if self.chain_fitness[chain_idx] > self.best_fitness {
                    self.best_fitness = self.chain_fitness[chain_idx];
                    self.best_params.clone_from(&self.chain_params[chain_idx]);
                    self.best_epoch = epoch;
                }
            }

            // 2. Swap pass.  K-1 adjacent swap attempts between
            //    chains (i, i+1), each under the replica-exchange
            //    Metropolis criterion:
            //
            //    α = min(1, exp((f_{i+1} - f_i) * (1/T_i - 1/T_{i+1})))
            //
            //    (maximization form).  Since T_i < T_{i+1},
            //    (1/T_i - 1/T_{i+1}) > 0, so the colder chain
            //    gladly takes a better state from the hotter
            //    chain.  Swap both `chain_params` and
            //    `chain_fitness` to keep the (state, fitness)
            //    invariant across the pair.
            let mut swap_attempts = 0u32;
            let mut swap_accepts = 0u32;
            if k >= 2 && hp.swap_interval_epochs > 0 && (epoch + 1) % hp.swap_interval_epochs == 0 {
                for i in 0..(k - 1) {
                    swap_attempts += 1;
                    let f_i = self.chain_fitness[i];
                    let f_ip1 = self.chain_fitness[i + 1];
                    // If either chain's fitness is still
                    // NEG_INFINITY (both forced-accept on epoch 0
                    // already ran, so this is rare), the delta is
                    // NaN/inf and the swap is rejected safely via
                    // the `is_finite` guard.
                    if !f_i.is_finite() || !f_ip1.is_finite() {
                        continue;
                    }
                    let t_i = self.temperatures[i];
                    let t_ip1 = self.temperatures[i + 1];
                    let inv_diff = (1.0 / t_i) - (1.0 / t_ip1);
                    let delta = (f_ip1 - f_i) * inv_diff;
                    let accept = if delta >= 0.0 {
                        true
                    } else {
                        rng.random::<f64>() < delta.exp()
                    };
                    if accept {
                        self.chain_params.swap(i, i + 1);
                        self.chain_fitness.swap(i, i + 1);
                        swap_accepts += 1;
                    }
                }
            }

            // 3. Emit per-epoch metrics.  `mean_reward` is chain
            //    0's current accepted fitness — chain 0 is the
            //    coldest chain and the returned artifact, so
            //    chain 0's trajectory is the PT-as-optimizer
            //    trace.  Additional diagnostics in `extra`:
            //    - `accepted` — chain-0 accept this epoch (0 or 1).
            //      Matches basic SA's `accepted` semantics so
            //      downstream readers do not see a shape drift.
            //    - `pt_chain_fitness_k` — every chain's current
            //      fitness (k = 0..K-1).
            //    - `pt_temperature_k` — every chain's fixed
            //      temperature (k = 0..K-1).
            //    - `pt_swap_attempts` / `pt_swap_accepts` — swap
            //      pass statistics for this epoch.
            let mut extra = BTreeMap::new();
            extra.insert("accepted".into(), f64::from(chain_accepts[0]));
            extra.insert("pt_swap_attempts".into(), f64::from(swap_attempts));
            extra.insert("pt_swap_accepts".into(), f64::from(swap_accepts));
            for (i, f) in self.chain_fitness.iter().enumerate() {
                extra.insert(format!("pt_chain_fitness_{i}"), *f);
            }
            for (i, t) in self.temperatures.iter().enumerate() {
                extra.insert(format!("pt_temperature_{i}"), *t);
            }

            let em = EpochMetrics {
                epoch,
                mean_reward: self.chain_fitness[0],
                done_count: 0,
                total_steps: k * n_envs * hp.max_episode_steps,
                wall_time_ms: t0.elapsed().as_millis() as u64,
                extra,
            };
            on_epoch(&em);
            metrics.push(em);

            // 4. Synchronize the scratch policy slot to chain 0's
            //    current params so `policy_artifact()` reflects
            //    the coldest chain.
            self.policy.set_params(&self.chain_params[0]);
        }

        metrics
    }

    fn policy_artifact(&self) -> PolicyArtifact {
        PolicyArtifact::from_policy(&*self.policy)
    }

    fn best_artifact(&self) -> PolicyArtifact {
        PolicyArtifact {
            version: CURRENT_VERSION,
            descriptor: self.policy.descriptor(),
            params: self.best_params.clone(),
            provenance: None,
        }
    }

    fn checkpoint(&self) -> TrainingCheckpoint {
        // PT's checkpoint is lossy: `TrainingCheckpoint` carries
        // one `policy_artifact` and one `algorithm_state`
        // `BTreeMap<String, f64>`, which cannot round-trip K
        // chains' worth of param vectors.  What the checkpoint
        // does record is the best-across-all-chains tracker,
        // chain 0's fitness (since chain 0 is the artifact), and
        // the ladder temperatures — enough for a reader who
        // knows PT's structure to pick up the end-state of the
        // winning chain.  Resumption of the full PT ensemble
        // from a checkpoint is explicitly out of scope for
        // Ch 53 §2.3 and would need a PT-aware checkpoint
        // shape that the `Algorithm` trait does not currently
        // offer.
        let mut state = BTreeMap::new();
        state.insert("current_fitness".into(), self.chain_fitness[0]);
        for (i, t) in self.temperatures.iter().enumerate() {
            state.insert(format!("pt_temperature_{i}"), *t);
        }
        for (i, f) in self.chain_fitness.iter().enumerate() {
            state.insert(format!("pt_chain_fitness_{i}"), *f);
        }

        TrainingCheckpoint {
            algorithm_name: "SA".into(),
            policy_artifact: self.policy_artifact(),
            critics: vec![],
            optimizer_states: vec![],
            algorithm_state: state,
            best_params: Some(self.best_params.clone()),
            best_reward: if self.best_fitness.is_finite() {
                Some(self.best_fitness)
            } else {
                None
            },
            best_epoch: self.best_epoch,
        }
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use sim_ml_chassis::{LinearPolicy, reaching_2dof};

    const TEST_N_ENVS: usize = 4;

    fn make_hp() -> PtHyperparams {
        PtHyperparams {
            n_chains: 4,
            t_min: 0.5,
            t_max: 50.0,
            proposal_std: 0.3,
            max_episode_steps: 50,
            swap_interval_epochs: 1,
        }
    }

    fn make_pt() -> (Pt, sim_ml_chassis::TaskConfig) {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        (Pt::new(policy, make_hp()), task)
    }

    #[test]
    fn pt_name_is_sa() {
        let (pt, _) = make_pt();
        // Intentional: see module doc comment.
        assert_eq!(pt.name(), "SA");
    }

    #[test]
    fn pt_geometric_ladder_shape() {
        let ladder = geometric_ladder(0.5, 50.0, 4);
        assert_eq!(ladder.len(), 4);
        assert!((ladder[0] - 0.5).abs() < 1e-12);
        assert!((ladder[3] - 50.0).abs() < 1e-9);
        // Constant ratio — ladder[1]/ladder[0] ==
        // ladder[2]/ladder[1] == ladder[3]/ladder[2].
        let r01 = ladder[1] / ladder[0];
        let r12 = ladder[2] / ladder[1];
        let r23 = ladder[3] / ladder[2];
        assert!((r01 - r12).abs() < 1e-12);
        assert!((r12 - r23).abs() < 1e-12);
        // Monotone increase.
        for i in 1..ladder.len() {
            assert!(ladder[i] > ladder[i - 1]);
        }
    }

    #[test]
    fn pt_geometric_ladder_k1() {
        let ladder = geometric_ladder(0.5, 50.0, 1);
        assert_eq!(ladder, vec![0.5]);
    }

    #[test]
    fn pt_smoke_2dof() {
        let (mut pt, task) = make_pt();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();

        let metrics = pt.train(&mut env, TrainingBudget::Epochs(5), 42, &|_| {});

        assert_eq!(metrics.len(), 5);
        for (i, m) in metrics.iter().enumerate() {
            assert_eq!(m.epoch, i);
            assert!(m.total_steps > 0);
            assert!(m.extra.contains_key("accepted"));
            assert!(m.extra.contains_key("pt_swap_attempts"));
            assert!(m.extra.contains_key("pt_swap_accepts"));
            // All K chain-fitness and temperature entries present.
            for c in 0..4 {
                assert!(
                    m.extra.contains_key(&format!("pt_chain_fitness_{c}")),
                    "missing pt_chain_fitness_{c} at epoch {i}"
                );
                assert!(
                    m.extra.contains_key(&format!("pt_temperature_{c}")),
                    "missing pt_temperature_{c} at epoch {i}"
                );
            }
        }

        // Epoch 0 runs K-1 = 3 swap attempts.
        assert_eq!(metrics[0].extra["pt_swap_attempts"], 3.0);

        // Total steps accounts for all K chains.
        let hp = make_hp();
        let expected_steps = hp.n_chains * TEST_N_ENVS * hp.max_episode_steps;
        assert_eq!(metrics[0].total_steps, expected_steps);
    }

    #[test]
    fn pt_best_tracker_covers_all_chains() {
        let (mut pt, task) = make_pt();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();

        pt.train(&mut env, TrainingBudget::Epochs(10), 7, &|_| {});
        let cp = pt.checkpoint();
        let best = cp.best_reward.expect("best_reward must be Some");
        assert!(best.is_finite());

        // The best tracker must be at least as good as the best
        // *chain-0 current fitness* ever observed — because the
        // tracker scans *all* chains, not just chain 0.  The
        // minimum we can assert without running a golden trace
        // is that best >= chain 0's final fitness.
        let chain0_final = pt.chain_fitness[0];
        assert!(
            best >= chain0_final || (best - chain0_final).abs() < 1e-12,
            "best ({best}) should be >= chain 0 final ({chain0_final})"
        );
    }

    #[test]
    fn pt_checkpoint_records_temperature_ladder() {
        let (pt, _task) = make_pt();
        let cp = pt.checkpoint();
        assert_eq!(cp.algorithm_name, "SA");
        for c in 0..4 {
            assert!(
                cp.algorithm_state
                    .contains_key(&format!("pt_temperature_{c}"))
            );
            assert!(
                cp.algorithm_state
                    .contains_key(&format!("pt_chain_fitness_{c}"))
            );
        }
        // Chain 0 is coldest.
        let t0 = cp.algorithm_state["pt_temperature_0"];
        let t3 = cp.algorithm_state["pt_temperature_3"];
        assert!(t0 < t3);
    }

    #[test]
    #[should_panic(expected = "n_chains must be >= 1")]
    fn pt_rejects_zero_chains() {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let mut hp = make_hp();
        hp.n_chains = 0;
        drop(Pt::new(policy, hp));
    }
}
