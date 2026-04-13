# Appendix A — API inventory

This appendix is a reference index for the public (and a
handful of private-but-specified) items the study's Part 4 PR
plans create or modify. Each row names a symbol, its kind, the
chapter section that specifies it, and a one-line description.
A reader looking for "where is `run_replicates` specified" can
scan the sim-ml-bridge table and land on the owning chapter
section directly.

## Scope

The inventory is **narrow**: it enumerates only the items
Ch 40, Ch 41, and Ch 42 create or modify. Items that the
plans inherit unchanged from pre-study code — `Cem`,
`Reinforce`, `Td3`, `Sac`, `Ppo`, `LinearPolicy`,
`collect_episodic_rollout`, `VecEnv::builder`, the pre-PR-1
`BatchSim::new`, `BrownianThermostat`, `PassiveStack`, and
the rest of the existing ml-bridge / thermostat / core
surface — are **not** listed here, even when the Part 4
chapters reference them at the call-site level.

The rationale is the user preference "code speaks for
itself" (recorded as `feedback_code_speaks.md` in the
project's durable memory): inherited items are already
documented by the source tree itself, and duplicating them
into the appendix would create a maintenance burden that
drifts every time someone adds a new algorithm or crate. The
appendix's value is "what did the *study* decide," not "what
does sim-ml-bridge's API look like." The latter is
rustdoc territory.

A corollary: this appendix is **not a substitute for reading
the chapters**. The "Specified in" column points at the
authoritative rendering; the one-line descriptions are
navigation aids, not specifications. When in doubt about
behavior or rationale, read the chapter.

## sim-thermostat (post-PR-1)

All new or modified items live at `sim/L0/thermostat/src/`.

**`prf` module (new).** File: `prf.rs`. Top-level module
introduced by PR 1a; module-level doc at [Ch 40
§2.4](40-pr-1-chassis-reproducibility.md) names the two
categories of primitives (per-step noise generation and
seed derivation).

| Symbol | Kind | Specified in | Description |
|---|---|---|---|
| `prf::chacha8_block(key: &[u8; 32], block_counter: u64) -> [u8; 64]` | fn (`pub(crate)`) | [Ch 40 §2.2](40-pr-1-chassis-reproducibility.md) | One ChaCha8 block keyed by `key` at the given counter. Pure function. |
| `prf::expand_master_seed(master_seed: u64) -> [u8; 32]` | fn (`pub(crate)`) | [Ch 40 §2.2](40-pr-1-chassis-reproducibility.md) | Expand a `u64` seed into a 32-byte ChaCha8 key using the same rule `SeedableRng::seed_from_u64` uses. |
| `prf::encode_block_counter(traj_id: u64, step_index: u64) -> u64` | fn (`pub(crate)`) | [Ch 40 §2.2](40-pr-1-chassis-reproducibility.md) | Encode `(traj_id, step_index)` into a 64-bit block counter via the D4 32/32 bit layout. |
| `prf::box_muller_from_block(block: &[u8; 64]) -> [f64; 8]` | fn (`pub(crate)`) | [Ch 40 §2.2](40-pr-1-chassis-reproducibility.md) | Apply Box–Muller to a 64-byte block, producing 8 f64 Gaussians. Paired-sample structure. |
| `prf::splitmix64(seed: u64) -> u64` | fn (`pub`) | [Ch 40 §2.2](40-pr-1-chassis-reproducibility.md), [§2.4](40-pr-1-chassis-reproducibility.md) | Canonical counter-to-stream hash for deriving uncorrelated `u64` seeds from one master. `pub` exception to the crate-private rule; Ch 42 §5.5 consumes it across crates. |

Visibility pick from [Ch 40 §2.5](
40-pr-1-chassis-reproducibility.md): `pub(crate)` for the
module and the four noise-path functions; `pub` for
`splitmix64` because Ch 42's rematch analysis calls it from a
dependent crate.

**`LangevinThermostat` (modified).** File: `langevin.rs`. The
mutex-bearing shape is replaced with the counter-bearing
shape across PR 1b. Struct fields listed below are private
(no `pub` annotation on the fields in the rewrite at [Ch 40
§3.2](40-pr-1-chassis-reproducibility.md)); they appear here
because the study explicitly specified their shape and a
reviewer of PR 1b's diff should see the specification.

| Symbol | Kind | Specified in | Description |
|---|---|---|---|
| `LangevinThermostat::new(gamma: DVector<f64>, k_b_t: f64, master_seed: u64, traj_id: u64) -> Self` | fn (`pub`, 4-arg) | [Ch 15 D2](15-design-decisions.md), [Ch 40 §3.2](40-pr-1-chassis-reproducibility.md) | Construct a thermostat with per-DOF damping, bath temperature, master seed, and trajectory id. Replaces the current 3-arg signature at 56 call sites across 18 files. |
| `LangevinThermostat.master_seed: u64` | field (private) | [Ch 15 D7](15-design-decisions.md), [Ch 40 §3.2](40-pr-1-chassis-reproducibility.md) | Retained user-facing seed for `diagnostic_summary`; held alongside `master_key` because the key space is larger than the seed space. |
| `LangevinThermostat.master_key: [u8; 32]` | field (private) | [Ch 15 D7](15-design-decisions.md), [Ch 40 §3.2](40-pr-1-chassis-reproducibility.md) | 32-byte ChaCha8 key expanded once at construction via `prf::expand_master_seed`. |
| `LangevinThermostat.traj_id: u64` | field (private) | [Ch 15 D4/D7](15-design-decisions.md), [Ch 40 §3.2](40-pr-1-chassis-reproducibility.md) | Trajectory index; typically the env index under an `install_per_env` factory. |
| `LangevinThermostat.counter: AtomicU64` | field (private) | [Ch 15 D5](15-design-decisions.md), [Ch 40 §3.2](40-pr-1-chassis-reproducibility.md) | Per-apply step counter; advances exactly once per `apply` call, gated by `stochastic_active`. |
| `LangevinThermostat::with_ctrl_temperature(self, ctrl_idx: usize) -> Self` | fn (`pub`, unchanged) | [Ch 40 §3.2](40-pr-1-chassis-reproducibility.md) | Moved into the rewrite for exposition; behavior unchanged from current code. Listed because the rewrite's struct layout depends on it. |
| `<LangevinThermostat as Diagnose>::diagnostic_summary() -> String` | method (`pub`, contract) | [Ch 15 D8](15-design-decisions.md), [Ch 40 §3.2](40-pr-1-chassis-reproducibility.md) | Static-only tuple: reports `(k_b_t, n_dofs, master_seed, traj_id)` plus `ctrl_temp` when set. Runtime counter deliberately excluded. |

The `apply` method's signature is unchanged; its body is
rewritten to the counter-based PRF chain at [Ch 40
§3.2](40-pr-1-chassis-reproducibility.md). The
`Stochastic` impl is unchanged.

## sim-core (post-PR-1)

All new items live at `sim/L0/core/src/batch.rs`. The
existing `BatchSim::new(model: Arc<Model>, n: usize)`
constructor is **unchanged** — PR 1b picks the additive
second-constructor reading of D1 at [Ch 40 §3.3](
40-pr-1-chassis-reproducibility.md), preserving the
shared-model path for the ~30 deterministic callers.

| Symbol | Kind | Specified in | Description |
|---|---|---|---|
| `BatchSim::new_per_env<S, F>(prototype: &Arc<S>, n: usize, factory: F) -> Self` where `S: PerEnvStack, F: FnMut(usize) -> (Model, Arc<S>)` | fn (`pub`) | [Ch 40 §3.3](40-pr-1-chassis-reproducibility.md) | Construct a per-env batch with distinct models and stacks per environment. Used by stochastic-aware callers; generic over the stack type to keep sim-core free of a direct sim-thermostat dependency. |
| `batch::PerEnvStack` | trait (`pub`) | [Ch 40 §3.3](40-pr-1-chassis-reproducibility.md) | Trait exposing the `install_per_env` surface that sim-core's generic constructor needs. Defined in sim-core, implemented in sim-thermostat on `PassiveStack`. |
| `batch::EnvBatch<S> { models: Vec<Model>, stacks: Vec<Arc<S>> }` | struct (`pub`) | [Ch 40 §3.3](40-pr-1-chassis-reproducibility.md) | Return type of `PerEnvStack::install_per_env`. Two-field struct holding the per-env model and stack vectors. |

**Rendering note.** The `PerEnvStack` trait and `EnvBatch<S>`
generic are not sub-decisions — [Ch 40 §3.3](
40-pr-1-chassis-reproducibility.md) explicitly frames them as
"the minimum edit that makes D1's locked factory shape
implementable without inverting the sim-core / sim-thermostat
dependency direction." A reviewer of PR 1b should expect to
see both, regardless of whether they appear in the D1–D15
list (they do not).

## sim-ml-bridge (post-PR-2 and bundled Ch 41 amendment)

All items live under `sim/L0/ml-bridge/src/`. The study's
touch on this crate spans PR 2 (Ch 41) and the bundled
amendment in Ch 42 §2 that extends `TaskConfig::build_fn` to
take a per-replicate seed.

**`competition.rs` additions.**

| Symbol | Kind | Specified in | Description |
|---|---|---|---|
| `Competition::run_replicates(&self, tasks, builders, seeds: &[u64]) -> Result<CompetitionResult, EnvError>` | method (`pub`) | [Ch 23 §1](23-competition-api-v2.md), [Ch 41 §2.1](41-pr-2-competition-replicates.md) | New entry point for replicate-aware runs. Holds the task × builder × seed cross-product body. Seeds-outermost loop ordering per [Ch 23 §1.4](23-competition-api-v2.md). |
| `Competition::run(&self, tasks, builders) -> Result<CompetitionResult, EnvError>` | method (`pub`, rewrapped) | [Ch 23 §1.1](23-competition-api-v2.md), [Ch 41 §2.1](41-pr-2-competition-replicates.md) | Becomes a one-line wrapper: `self.run_replicates(tasks, builders, &[self.seed])`. Existing callers see zero behavior change for single-seed use. |
| `RunResult.replicate_index: usize` | field (`pub`, new) | [Ch 23 §1.3](23-competition-api-v2.md), [Ch 41 §2.1](41-pr-2-competition-replicates.md) | 0-based index within the replicate set. Stamped by `run_replicates` during the seeds-outermost loop. |
| `CompetitionResult::find_replicate(&self, task: &str, algorithm: &str, replicate_index: usize) -> Option<&RunResult>` | method (`pub`) | [Ch 23 §1.3](23-competition-api-v2.md), [Ch 41 §2.1](41-pr-2-competition-replicates.md) | Returns the specific replicate for a `(task, algo, idx)` triple. Three-line impl. |
| `CompetitionResult::replicate_best_rewards(&self, task: &str, algorithm: &str) -> Vec<f64>` | method (`pub`) | [Ch 24 D2](24-result-semantics.md), [Ch 41 §2.1](41-pr-2-competition-replicates.md) | Returns the `Vec<f64>` of per-replicate `best_reward()` values for a task × algorithm pair. Silently filters out replicates whose `best_reward()` is `None`. |
| `CompetitionResult::describe(&self, task: &str, algorithm: &str) -> Option<SeedSummary>` | method (`pub`) | [Ch 24 D2](24-result-semantics.md), [Ch 41 §2.1](41-pr-2-competition-replicates.md) | Returns a `SeedSummary` for a task × algorithm pair. `None` if no replicates match. |
| `SeedSummary { n: usize, mean: f64, std_dev: f64 }` | struct (`pub`, pub fields) | [Ch 24 §4.3](24-result-semantics.md), [Ch 41 §2.1](41-pr-2-competition-replicates.md) | Inline in `competition.rs`. Three-field summary used by `describe`. |
| `SeedSummary::from_rewards(rewards: &[f64]) -> Option<Self>` | fn (`pub`) | [Ch 24 §4.3](24-result-semantics.md), [Ch 41 §2.1](41-pr-2-competition-replicates.md) | Constructor. Returns `None` on empty slice. Uses Bessel's correction (sample std with `n−1` denominator). |

**`task.rs` signature extension (Ch 42 bundled amendment).**
The signature extension is the load-bearing sub-decision of
Ch 42 — [§2.1–§2.4](42-pr-3-sim-opt-rematch.md) walks four
renderings and picks R2 (extend the signature) on four
structural grounds.

| Symbol | Kind | Specified in | Description |
|---|---|---|---|
| `TaskConfig.build_fn: Arc<dyn Fn(usize, u64) -> Result<VecEnv, EnvError> + Send + Sync>` | field (`pub`, extended) | [Ch 41 §2.1](41-pr-2-competition-replicates.md), [Ch 42 §2.1](42-pr-3-sim-opt-rematch.md) | Signature extended from `Fn(usize) -> ...` to `Fn(usize, u64) -> ...` so stochastic-physics tasks can vary their `LangevinThermostat::master_seed` per replicate. Threaded by `Competition::run_replicates` at `competition.rs:330`. |
| `TaskConfig::build_vec_env(&self, n_envs: usize, seed: u64) -> Result<VecEnv, EnvError>` | method (`pub`, 2-arg) | [Ch 41 §2.1](41-pr-2-competition-replicates.md), [Ch 42 §2.1](42-pr-3-sim-opt-rematch.md) | Forwards the second parameter to `build_fn`. Call site at `task.rs:108` becomes 2-arg; existing call site at `vec_env.rs:391` is unchanged (the seed flows through `TaskConfig`, not through `VecEnv::builder`). |

The stock deterministic-physics tasks (`reaching_2dof`,
`reaching_6dof`, `obstacle_reaching_6dof`) gain an ignored
`_seed: u64` parameter in their closures, documented at
[Ch 42 §2.5](42-pr-3-sim-opt-rematch.md). The convention
matches the existing Rust underscore-prefix idiom for unused
parameters.

**Contract changes (PR 2b semantic fixes).** These are
not new symbols — they are semantic tightenings of existing
items — but the study's specification of their post-PR-2
contracts is load-bearing for the rematch and belongs in an
API reference.

| Symbol | Kind | Specified in | Description |
|---|---|---|---|
| `EpochMetrics::mean_reward` (unit contract) | field (contract) | [Ch 24 D1](24-result-semantics.md), [Ch 41 §2.2](41-pr-2-competition-replicates.md) | Unit-uniform across all five algorithms: "mean per-episode total reward across `n_envs` trajectories." CEM, TD3, and SAC migrate into this shape; REINFORCE and PPO already satisfy it. |
| REINFORCE/PPO zero-fallback cleanup at `reinforce.rs:213-225` and `ppo.rs:313-325` | branch (contract) | [Ch 41 §2.4](41-pr-2-competition-replicates.md) | Shape (i) skip-the-push: drops the `EpochMetrics` construction + `on_epoch` + `metrics.push` on the `n_samples == 0` branch, leaving a bare `continue`. CEM, TD3, and SAC do not share this cleanup — the zero-fallback branch is REINFORCE/PPO-specific. |
| CEM `EpochMetrics.extra["elite_mean_reward_per_step"]` | map key (renamed) | [Ch 41 §2.3](41-pr-2-competition-replicates.md) | Renamed from `extra["elite_mean_reward"]` to `extra["elite_mean_reward_per_step"]`. Surfaces the per-step concept explicitly so it is not confused with the per-episode-total `mean_reward`. |

## sim-opt (post-PR-3, new crate)

The crate is new. Placement: `sim/L0/opt/` as an L0 peer of
ml-bridge and thermostat. Runtime deps: `sim-ml-bridge`,
`sim-thermostat`, `rand`, `serde`, `thiserror`. Dev-deps:
`sim-core`, `sim-mjcf`, `approx`. See [Ch 42 §3.1](
42-pr-3-sim-opt-rematch.md) for the `Cargo.toml` and
[§3.3](42-pr-3-sim-opt-rematch.md) for the
dependency-direction argument.

**`algorithm.rs` — the Simulated Annealing algorithm.**

| Symbol | Kind | Specified in | Description |
|---|---|---|---|
| `Sa` | struct (`pub`) | [Ch 42 §4.1](42-pr-3-sim-opt-rematch.md) | The SA algorithm. Eight fields: `policy`, `hyperparams`, `current_params`, `current_fitness`, `temperature`, `best_params`, `best_fitness`, `best_epoch`. Inline best-tracking (not a shared `BestTracker`, which is `pub(crate)` in ml-bridge). |
| `SaHyperparams { initial_temperature, proposal_std, cooling_decay, temperature_min, max_episode_steps }` | struct (`pub`, pub fields, `Copy`) | [Ch 42 §4.1](42-pr-3-sim-opt-rematch.md) | Five-field hyperparameter struct. Defaults defended at [§4.4–§4.5](42-pr-3-sim-opt-rematch.md): `cooling_decay = 0.955` geometric, `initial_temperature = 50.0`, `proposal_std = 0.5`, `temperature_min = 0.005`. |
| `Sa::new(policy: Box<dyn Policy>, hyperparams: SaHyperparams) -> Self` | fn (`pub`) | [Ch 42 §4.1](42-pr-3-sim-opt-rematch.md) | Construct a fresh SA. Starts with `current_fitness = f64::NEG_INFINITY` so the first Metropolis comparison always accepts (no baseline eval; see [§4.2](42-pr-3-sim-opt-rematch.md) for why). |
| `Sa::from_checkpoint(checkpoint: &TrainingCheckpoint, hyperparams: SaHyperparams) -> Result<Self, ArtifactError>` | fn (`pub`) | [Ch 42 §4.1](42-pr-3-sim-opt-rematch.md) | Reconstruct an SA from a checkpoint. Matches CEM's pattern at `cem.rs:89-115`: restore policy, read algorithm_state (temperature, current_fitness), restore best tracker. |
| `impl Algorithm for Sa` | trait impl | [Ch 42 §4.2](42-pr-3-sim-opt-rematch.md) | `name()`, `train()`, `best_artifact()`, `checkpoint()`, `set_best()`. `train()` implements Metropolis accept/reject with multi-env fitness averaging and geometric cooling. |

**`analysis.rs` — statistical analysis for the rematch.**

| Symbol | Kind | Specified in | Description |
|---|---|---|---|
| `BootstrapCi { point_estimate, lower, upper, n_resamples }` | struct (`pub`, pub fields, `Serialize + Deserialize`) | [Ch 42 §5.2](42-pr-3-sim-opt-rematch.md) | Bootstrap CI on a difference statistic with the observed point estimate and both percentile bounds. `n_resamples` records `B` in the serialized form. |
| `BootstrapCi::classify(&self) -> RematchOutcome` | method (`pub`) | [Ch 42 §5.2](42-pr-3-sim-opt-rematch.md) | Classifies the CI into one of Ch 30's three outcomes per Ch 32 §3.3's table. Implementation uses a three-tuple `match` on `(lower > 0, upper > 0, point > 0)`. |
| `RematchOutcome { Positive, Null, Ambiguous }` | enum (`pub`, `Serialize + Deserialize`) | [Ch 42 §5.4](42-pr-3-sim-opt-rematch.md) | Ch 30's three meaningful outcomes. No fourth hedge per [Ch 31 §3.2](31-failure-modes.md): "a 'weak positive' or 'near-null' hedge is not on the menu." |
| `bootstrap_diff_means(r_a: &[f64], r_b: &[f64], rng: &mut impl Rng) -> BootstrapCi` | fn (`pub`) | [Ch 42 §5.2](42-pr-3-sim-opt-rematch.md) | Percentile bootstrap CI on the difference of means of two samples. `B = 10_000` matches [Ch 32 §3.2](32-hyperparameter-sensitivity.md). Paired resampling. Panics on empty input. |
| `bootstrap_diff_medians(r_a: &[f64], r_b: &[f64], rng: &mut impl Rng) -> BootstrapCi` | fn (`pub`) | [Ch 42 §5.3](42-pr-3-sim-opt-rematch.md) | Identical shape to `bootstrap_diff_means` with `median` substituted for `mean`. Used when either algorithm's bimodality coefficient exceeds 5/9. |
| `bimodality_coefficient(values: &[f64]) -> f64` | fn (`pub`) | [Ch 42 §5.4](42-pr-3-sim-opt-rematch.md) | Pearson's bimodality coefficient (SAS small-sample corrected form). Requires `n >= 4`; panics otherwise. `BC > 5/9` indicates bimodality or strong skew. |
| `classify_outcome(lower: f64, upper: f64, point_estimate: f64) -> RematchOutcome` | fn (`pub`) | [Ch 42 §5.4](42-pr-3-sim-opt-rematch.md) | Free-function form of `BootstrapCi::classify`. Useful for testing edge cases without constructing a `BootstrapCi`. |
| `run_rematch(competition: &Competition, task: &TaskConfig, algorithm_builders: &[...], bootstrap_rng: &mut impl Rng) -> Result<RematchOutcome, EnvError>` | fn (`pub`) | [Ch 42 §5.5](42-pr-3-sim-opt-rematch.md) | Executes Ch 32's folded-pilot protocol end-to-end: initial batch at `N = 10`, classify, expand to `N = 20` on `Ambiguous`, return final classification. Single `Competition::run_replicates` call per batch per the bundled Ch 41 amendment. |
| `REMATCH_MASTER_SEED: u64 = 20_260_412` | const (`pub`) | [Ch 42 §5.5](42-pr-3-sim-opt-rematch.md) | Pre-registered master seed from [Ch 32 Decision 4](32-hyperparameter-sensitivity.md). Matches `d2c_cem_training.rs:62`'s `SEED_BASE` literal. |
| `N_INITIAL: usize = 10` | const (`pub`) | [Ch 42 §5.5](42-pr-3-sim-opt-rematch.md) | Initial batch size from [Ch 32 Decision 3](32-hyperparameter-sensitivity.md). |
| `N_EXPANDED: usize = 20` | const (`pub`) | [Ch 42 §5.5](42-pr-3-sim-opt-rematch.md) | Expanded batch size from [Ch 32 Decision 3](32-hyperparameter-sensitivity.md), triggered on the `Ambiguous` initial outcome. |
| `REMATCH_TASK_NAME: &str = "d2c-sr-rematch"` | const (`pub`) | [Ch 42 §5.5](42-pr-3-sim-opt-rematch.md) | Task name the rematch uses inside `Competition`'s task registry. Fixed so `replicate_best_rewards` calls find the task consistently across the initial and expanded batches. |

## What this appendix does not cover

- **Inherited items.** Unchanged public items the study
  references — `Cem`, `Reinforce`, `Td3`, `Sac`, `Ppo`,
  `Policy`, `LinearPolicy`, `Artifact`, `TrainingBudget`,
  `TrainingCheckpoint`, `EpochMetrics` (field set unchanged),
  `VecEnv`, `VecEnv::builder`, `TaskConfig` (pre-extension
  shape), `CompetitionResult` pre-extension methods (`find`,
  `for_task`, `for_algorithm`, `best_for_task`,
  `save_artifacts`, `print_ranked`, `print_summary`),
  `BatchSim::new`, `PassiveStack`, `PassiveStack::builder`,
  `BrownianThermostat`, and the rest of the pre-study
  surface.
- **Internal helpers.** Module-private functions that are
  not named in the chapters as specification-worth items:
  the inlined `randn` Box–Muller helper inside `sim-opt`'s
  SA loop at [Ch 42 §4.2](42-pr-3-sim-opt-rematch.md), the
  `mean` / `median` helpers in `analysis.rs`, the
  `test_and_classify` private helper inside `run_rematch`,
  and similar rendering-level internals.
- **Test-only items.** Mock types and hand-crafted inputs
  used in the unit tests — `MockAlgorithm`, the rematch
  fixture's SR task infrastructure duplicated from
  `d2c_cem_training.rs`. Test-surface items belong in
  Appendix B (test inventory).
- **Sub-decisions and rationale.** This appendix indexes
  *what* the plans specify, not *why*. Readers who want
  the defense of a pick (why `cooling_decay = 0.955`, why
  inline best-tracking over a shared `BestTracker`, why
  `BatchSim::new_per_env` instead of a destructive rewrite)
  should read the owning chapter section.
