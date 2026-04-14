# Appendix B — Test inventory

This appendix enumerates the tests the study's Part 4 PR
plans add, modify, or explicitly drop. It is organized by PR
(1a, 1b, 2a, 2b, 3a, 3b) so a PR implementor has a checklist:
every entry corresponds to either a new `#[test]` function
that must land with the PR or an existing test that must be
updated. A PR 2a reviewer can scan the PR 2a subsection and
know the exact test count that "PR 2a is ready for review"
means.

## Scope

Same narrowness as Appendix A: this inventory covers only
the tests the study's plans specify. Tests that already exist
in the ml-bridge / thermostat / core / mjcf crates and that
the plans leave unchanged are **not** listed here. A reader
who wants the full test surface of a crate should run
`cargo test --list` against that crate; the study's plans
add to that surface but do not describe the pre-existing
body.

## Headline count

- **PR 1a:** ≈10 new inline `#[test]` functions in a new
  `prf.rs` module. Specific names rendered at implementation
  time; the chapter locks the semantic coverage, not the
  function names.
- **PR 1b:** 1 new integration test
  (`parallel_matches_sequential_with_langevin`) + 5
  existing-test updates for the D2 4-arg constructor ripple.
  (The initial draft called for a second feature-gated
  comparator reading disk-trace files; session-13
  implementation replaced the two-function disk-trace
  design with a single-function in-process determinism
  check — see [Ch 40 §3.4 (a)](
  40-pr-1-chassis-reproducibility.md).)
- **PR 2a:** 13 new `#[test]` functions in
  `competition.rs`'s existing test module.
- **PR 2b:** 6 new `#[test]` functions across
  `cem.rs`/`td3.rs`/`sac.rs` + 1 in-place test rename.
- **PR 3a:** 4 new SA tests in `sim-opt/src/algorithm.rs` +
  10 new analysis tests in `sim-opt/src/analysis.rs` = 14
  new unit tests.
- **PR 3b commit 1:** 3 new inline `#[test]` functions in
  `sim-ml-bridge/src/task.rs` for the `TaskConfig::from_build_fn`
  constructor (the custom-stochastic-task path deferred from
  PR 2a, closed in commit `2adaa372`).
- **PR 3b commit 2:** 1 new `#[test] #[ignore]` integration
  test at `sim-opt/tests/d2c_sr_rematch.rs`.

**Total: ≈48 new runtime tests** (38 concretely named + ~10
inline prf tests whose names are not chapter-locked), **1
in-place rename, 5 existing-test updates**, plus two tests
the plans **explicitly dropped** (see the closing "Tests the
plans dropped" subsection).

Concretely named breakdown: 1 (PR 1b) + 13 (PR 2a) + 6 (PR 2b)
+ 14 (PR 3a) + 3 (PR 3b commit 1) + 1 (PR 3b commit 2) = 38.
(The stale `2 (PR 1b)` in earlier drafts of this appendix was
a session-13 drift: the initial two-function disk-trace test
design was replaced with a single-function in-process
determinism check — see [Ch 40 §3.4 (a)](40-pr-1-chassis-reproducibility.md).)

## PR 1a — `prf.rs` inline unit tests

**Location:** inline `#[cfg(test)] mod tests` at the bottom
of `sim/L0/thermostat/src/prf.rs`. Chapter convention for
the test surface: [Ch 40 §2.3](40-pr-1-chassis-reproducibility.md).

The chapter specifies test coverage semantically (what
property to verify) rather than by function name. The
implementation is free to pick idiomatic function names;
the chapter's gate is "the listed coverage is exercised."
Coverage items from [Ch 40 §2.3](
40-pr-1-chassis-reproducibility.md):

1. **ChaCha8 cross-check against `rand_chacha 0.9.0`:**
   `chacha8_block(expand_master_seed(0), 0)` equals
   `ChaCha8Rng::seed_from_u64(0)`'s block output at the
   same coordinate.
2. **Block-boundary coordinate:** counter pinned at a
   specific value near a word-position wraparound, chosen
   during implementation.
3. **High-end edge case:**
   `chacha8_block(expand_master_seed(u64::MAX), u64::MAX - 1)`
   equals the reference.
4. **Three pre-chosen `(seed, counter)` pairs** with
   hard-coded expected 64-byte outputs. Per [Ch 15 D6](
   15-design-decisions.md): "random" here means
   random-at-test-authoring-time, not random-at-runtime.
5. **`encode_block_counter(traj_id, step_index)` round-trip
   recovery** — obvious inverse on a handful of pairs.
6. **`splitmix64(0)`, `splitmix64(1)`, and two other values**
   matching the canonical SplitMix64 reference output.
7. **`box_muller_from_block(zeros)` smoke test** — pins the
   implementation's orientation on a well-defined input.

**Count:** ~10 `#[test]` functions per the chapter's
estimate. Each is small, independent, and requires no
fixture loading or `PassiveStack` construction. The inline
placement is sub-decision (a) at [Ch 40 §2.3](
40-pr-1-chassis-reproducibility.md), defended against the
alternative of a separate integration test file.

## PR 1b — `LangevinThermostat` rewrite regression tests

**Location:** the regression test lives at
`sim/L0/tests/integration/batch_sim.rs`, alongside the
existing `batch_matches_sequential_with_contacts` at `:55`.
Chapter source: [Ch 40 §3.4](40-pr-1-chassis-reproducibility.md).

### New tests

- **`parallel_matches_sequential_with_langevin`** — new
  `#[test]` at `sim/L0/tests/integration/batch_sim.rs` (owning
  crate is `sim-conformance-tests`, not the informal
  "sim-integration" name the initial draft used; corrected in
  session-13 drift-fix `ae1ff5ef`). Runs under both default
  features and `--features parallel`. Builds two
  `BatchSim::new_per_env` batches at the same `master_seed`
  with identical factories (32 envs × 1000 steps), then
  asserts bit-for-bit `(qpos, qvel)` agreement across the
  two runs. Three seeds: `0`, `0xD_06F0_0D42`, `u64::MAX`
  (per [Ch 15 D10](15-design-decisions.md)). Uses the 1-DOF
  SHO fixture inlined in the test file (shape identical to
  the existing Phase 1 integration tests, per [Ch 15 D11](
  15-design-decisions.md)). Under the parallel feature
  rayon's `par_iter_mut` scheduler is non-deterministic
  across runs; two successive same-seed runs agreeing proves
  the noise stream is scheduler-independent — the chassis
  property C-3 was architected to guarantee. This is the
  single-function in-process replacement for the initial
  draft's two-function disk-trace design (see [Ch 40 §3.4 (a)](
  40-pr-1-chassis-reproducibility.md) and its session-13
  post-implementation audit note).

### Existing-test updates

Five existing tests in `sim/L0/thermostat/src/langevin.rs`
update mechanically for the D2 4-arg constructor, per
[Ch 40 §3.4 (c)](40-pr-1-chassis-reproducibility.md):

- **`apply_does_not_advance_rng_when_stochastic_inactive`**
  at `langevin.rs:311-363` — semantic contract unchanged
  (the FD-invariant: first post-re-enable sample equals a
  fresh thermostat's first sample), but the operational
  construction switches to `LangevinThermostat::new` with
  the same `(master_seed, traj_id)` pair, and the equality
  check becomes a direct `assert_eq!` on two `f64` outputs.
  ~10 lines of change.
- **`new_initializes_stochastic_active_to_true`** at
  `langevin.rs:241-266` — adds `traj_id = 0` to the
  constructor call; semantic content unchanged.
- **`set_stochastic_active_roundtrips`** at `langevin.rs:241-266`
  — same 4-arg update.
- **`as_stochastic_returns_some_self`** at `langevin.rs:241-266`
  — same 4-arg update.
- **`diagnostic_summary_format_matches_spec`** at
  `langevin.rs:269-276` — updates to the D8 tuple format
  (static-only: `(kT, n_dofs, master_seed, traj_id[, ctrl_temp])`).

### Mechanical ripple: the D2 4-arg call-site migration

56 `LangevinThermostat::new` call sites across 18 files
update from 3-arg to 4-arg, per [Ch 40 §3.4 (b)](
40-pr-1-chassis-reproducibility.md). This is not a test
addition — it is a mechanical test-file update — but any
site that is itself inside a `#[test]` function is part of
the PR 1b diff and a reviewer should see the pattern. The
non-mechanical sites (per Ch 40 §3.4 (b), session-13
revision): `tests/multi_dof_equipartition.rs:188, :263`
(per-trajectory loops — `seed_base + i` splits into
`(master_seed, traj_id = i)`); `tests/multi_dof_equipartition
.rs:346, :357` and `tests/langevin_thermostat.rs:323, :334`
(reproducibility helpers — two parallel builds asserting
bit-identity, both get identical `traj_id = 0`); and
`src/ising_learner.rs:198` (production caller, `traj_id`
threaded from outer context). `tests/langevin_thermostat.rs
:282` (the `CountingWrapper`) is mechanical `traj_id = 0`
despite the initial-draft flag.

## PR 2a — Competition replicates (additive API)

**Location:** all 13 new tests live inside
`sim/L0/ml-bridge/src/competition.rs`'s existing
`#[cfg(test)] mod tests` block at `:416-900`, joining the
existing `competition_runs_all_pairs` and its siblings.
Chapter source: [Ch 41 §2.6](41-pr-2-competition-replicates.md).

1. **`run_replicates_flat_shape`** — asserts
   `result.runs.len() == tasks.len() * builders.len() * seeds.len()`
   for a two-task, two-builder, three-seed call. Expected
   length: 12.
2. **`run_replicates_replicate_index_monotonic`** — for each
   `(task, algo)` pair in the 12-element result, filters
   via `for_task` + `for_algorithm` and asserts
   `replicate_index` values `[0, 1, 2]` in order. Verifies
   the seeds-outermost loop and the index stamping.
3. **`run_replicates_preserves_strict_gt_tie_breaking`** —
   load-bearing regression guard for the inline best-epoch
   scan's strict-`>` tie-breaking invariant at
   `competition.rs:363-379`. Uses a `MockAlgorithm` variant
   that emits two `EpochMetrics` entries with identical
   `mean_reward` values and asserts
   `provenance.best_epoch == 0` (not `1`). A refactor that
   replaced the inline scan with `RunResult::best_reward()`
   would fail this test because `max_by` under
   `Ordering::Equal` returns the last iterated element.
4. **`run_replicates_fresh_env_per_pair`** — pushes the same
   `MockAlgorithm` builder through two tasks with three
   seeds and verifies per-run `metrics.len()` consistency
   (no cross-run state leakage from a stale `VecEnv`).
5. **`run_still_works_single_seed`** — asserts
   `comp.run(...)`'s forwarder path produces a `RunResult`
   with `replicate_index == 0` on every element.
6. **`find_replicate_returns_specific_seed`** — three-seed
   call, then `find_replicate("reaching-2dof", "Mock", 1)`
   returns the second replicate; `find_replicate(..., 99)`
   returns `None`.
7. **`replicate_best_rewards_flat_filter`** — three-seed
   call on `MockAlgorithm`, asserts
   `replicate_best_rewards("reaching-2dof", "Mock").len() == 3`.
8. **`describe_returns_seed_summary`** — asserts
   `describe("reaching-2dof", "Mock").is_some()` and that
   the returned `SeedSummary` has `n == 3`, finite `mean`,
   and `std_dev >= 0.0`.
9. **`describe_none_for_missing_pair`** — asserts
   `describe("no-such-task", "Mock").is_none()`.
10. **`seed_summary_from_rewards_n1_returns_zero_stddev`** —
    unit test on `SeedSummary::from_rewards(&[1.0])`;
    asserts `n == 1`, `mean == 1.0`, `std_dev == 0.0`.
    Covers the degenerate `n == 1` case [Ch 24 §4.6](
    24-result-semantics.md) named.
11. **`seed_summary_from_rewards_empty_returns_none`** —
    one-line test:
    `assert!(SeedSummary::from_rewards(&[]).is_none())`.
    Covers the empty-slice contract [Ch 24 §4.3](
    24-result-semantics.md) locked. Without this test, a
    refactor returning `Some(SeedSummary { n: 0, mean:
    NAN, std_dev: NAN })` would ship unnoticed.
12. **`seed_summary_from_rewards_bessel_correction`** —
    unit test on `SeedSummary::from_rewards(&[1.0, 2.0, 3.0])`;
    asserts `mean == 2.0` and `std_dev == 1.0` exactly
    (sample std with `n-1 = 2` denominator). Catches a
    population-vs-sample std regression in one line.
13. **`replicate_best_rewards_silent_filter_out`** —
    deliberately produces a `None` from
    `RunResult::best_reward()` on at least one replicate
    (via a `MockAlgorithm` variant whose `train` returns
    empty `Vec<EpochMetrics>` for one seed and finite
    metrics for the others); asserts
    `replicate_best_rewards(...).len() < seed_count` and
    that `SeedSummary.n` reflects the filtered count.

## PR 2b — Per-algorithm train-loop fixes (semantic)

**Location:** six new tests distributed across the three
algorithm files' existing test modules; one in-place rename.
Chapter source: [Ch 41 §2.6](41-pr-2-competition-replicates.md).

### New tests

14. **`cem_mean_reward_is_per_episode_total`** — in
    `cem.rs`'s test module near `cem_smoke_2dof` at
    `cem.rs:292`. Trains a CEM instance on a reaching task
    and asserts the final epoch's `mean_reward` is in the
    per-episode-total magnitude range (e.g., near −300 on
    `max_episode_steps = 300` with an approximately −1
    per-step reward), not per-step (which would be near
    −1).
15. **`cem_dual_reward_concept_split`** — load-bearing
    regression guard for [Ch 24 §3.5](24-result-semantics.md)'s
    scope-discipline argument that CEM carries two reward
    concepts internally under PR 2b (per-step elite
    fitness at `cem.rs:182-183`, per-episode-total report
    at `cem.rs:209`). Asserts both concepts simultaneously
    on a single run:
    `last.mean_reward.abs() > last.extra["elite_mean_reward_per_step"].abs() * 10.0`.
    Catches a future refactor that accidentally folds the
    reporting site back into the fitness loop.
16. **`td3_mean_reward_is_per_episode_total`** — same shape
    as test 14, in `td3.rs`'s test module near
    `td3_smoke_2dof` at `td3.rs:617`.
17. **`sac_mean_reward_is_per_episode_total`** — same shape
    as test 14, in `sac.rs`'s test module near
    `sac_smoke_2dof` at `sac.rs:672`.
18. **`td3_epoch_rewards_invariant_holds`** — debug-only
    test (`#[cfg(debug_assertions)]`) that exercises the
    pre-loop + `debug_assert_eq!(epoch_rewards.len(), n_envs)`
    invariant from [Ch 41 §2.2](41-pr-2-competition-replicates.md).
    Runs TD3 with a deliberately short `max_episode_steps`
    and asserts the debug-assert does not fire.
19. **`sac_epoch_rewards_invariant_holds`** — same shape as
    test 18, in `sac.rs`.

### In-place rename

20. **`cem_elite_mean_reward_per_step_key_present`** —
    existing unit test at `cem.rs:319` (and the sibling at
    `:354` that also reads the key). The two
    `contains_key` / indexing assertions update from
    `"elite_mean_reward"` to `"elite_mean_reward_per_step"`
    per [Ch 41 §2.3](41-pr-2-competition-replicates.md)'s
    rename. This is **not** a new test — it is an in-place
    update — but it is listed here because a PR 2b
    reviewer should see the rename explicitly in the diff.

## PR 3a — `sim-opt` crate unit tests

**Location:** two new inline test modules, one per
sim-opt library file. Both at
`sim/L0/opt/src/algorithm.rs` and
`sim/L0/opt/src/analysis.rs`. Chapter source: [Ch 42 §4.8](
42-pr-3-sim-opt-rematch.md) for SA, [Ch 42 §5.6](
42-pr-3-sim-opt-rematch.md) for analysis.

### SA unit tests (4 tests, `sim/L0/opt/src/algorithm.rs`)

21. **`sa_name`** — asserts `sa.name() == "SA"`. Three lines.
22. **`sa_smoke_2dof`** — constructs an `Sa` with a
    `LinearPolicy` on the `reaching_2dof()` task, trains
    for 5 epochs with `TrainingBudget::Epochs(5)`, asserts
    `metrics.len() == 5` and each metric has non-zero
    `total_steps`. Matches CEM's `cem_smoke_2dof` pattern.
    ~40 lines.
23. **`sa_best_tracker_monotone`** — constructs an SA,
    trains for 20 epochs, asserts that `sa.best_artifact()`'s
    underlying `best_reward` never decreases across
    checkpoint reads during training. Tests the inline
    best-tracking logic. ~25 lines.
24. **`sa_checkpoint_roundtrip`** — constructs an SA, trains
    for 10 epochs, calls `checkpoint()`, then
    `Sa::from_checkpoint(&checkpoint, hyperparams)`, and
    asserts the restored SA's `current_params` and
    `best_params` match. ~30 lines.

SA unit tests deliberately do **not** exercise the
rematch's SR task (that is PR 3b's scope) and do **not**
exercise multi-env averaging with stochastic physics (the
`reaching_2dof()` task is deterministic). They exercise the
`Algorithm` trait implementation surface and the
hyperparameter defaults.

### Analysis unit tests (10 tests, `sim/L0/opt/src/analysis.rs`)

25. **`bootstrap_diff_means_positive_ci`** — hand-crafted
    `r_a = [1.0; 5]`, `r_b = [0.0; 5]`; asserts the CI has
    `point_estimate ≈ 1.0`, `lower > 0.0`, `upper > 0.0`;
    classification is `Positive`. ~25 lines.
26. **`bootstrap_diff_means_null_ci`** — `r_a = r_b = [1.0,
    2.0, 3.0, 4.0, 5.0]`; asserts the CI straddles zero
    with `point_estimate ≈ 0`. ~20 lines.
27. **`bootstrap_diff_means_ambiguous_ci`** — values where
    sample means favor `r_a` but variance is large enough
    that the CI straddles zero. Specific values:
    `r_a = [1.0, -0.5, 2.0, 0.0, 1.0]`,
    `r_b = [0.0, 0.5, -0.5, 1.0, -1.0]`. Asserts
    classification is `Ambiguous`. Deterministic bootstrap
    RNG seed. ~25 lines.
28. **`bootstrap_diff_medians_robust_to_outlier`** —
    `r_a = [1.0, 1.0, 1.0, 1.0, 100.0]`,
    `r_b = [0.0, 0.0, 0.0, 0.0, 0.0]`. Asserts the
    median-based CI has `point_estimate = 1.0`, robust to
    the outlier. ~25 lines.
29. **`bimodality_coefficient_unimodal`** — input
    `[1.0, 1.5, 2.0, ..., 5.5]` (symmetric linear), asserts
    `BC < 5/9`. ~15 lines.
30. **`bimodality_coefficient_bimodal`** — input
    `[1.0; 5] ++ [5.0; 5]` (two clusters), asserts
    `BC > 5/9`. ~15 lines.
31. **`bimodality_coefficient_requires_n_geq_4`** — input
    of length 3, asserts the call panics with the
    documented error message. Uses `#[should_panic]`. ~10
    lines.
32. **`classify_outcome_boundary_cases`** — tests edge
    cases at `(lower=0, upper=0, point=0)`,
    `(lower=0, upper=1, point=0)`, etc. Ensures the strict
    `>` vs `>=` boundaries match [Ch 32 §3.3](
    32-hyperparameter-sensitivity.md)'s table. ~30 lines.
33. **`run_rematch_ambiguous_triggers_expansion`** — uses
    a mock `Competition` with a `MockAlgorithm` that
    produces deterministic replicate values engineered to
    classify as `Ambiguous` at `N = 10`. Asserts
    `run_rematch` runs the expansion phase (detectable via
    a call counter on the mock). The load-bearing
    driver-level test for the folded-pilot protocol's
    control flow. ~50 lines.
34. **`run_rematch_positive_short_circuits`** — similar to
    test 33 but with replicate values engineered to
    classify as `Positive` at `N = 10`. Asserts
    `run_rematch` does **not** run the expansion phase.
    ~40 lines.

## PR 3b commit 1 — `TaskConfig::from_build_fn` unit tests

**Location:** three new `#[test]` functions in
`sim/L0/ml-bridge/src/task.rs`'s existing `#[cfg(test)] mod
tests` block, appended after the stock-task metadata tests
at a `// ── from_build_fn (custom seeded constructor) ──`
section divider. Chapter source: [Ch 42 §2 sub-decision
(a)](42-pr-3-sim-opt-rematch.md) named the custom-builder
surface as the deferred post-PR-2a follow-up; the tests were
added alongside the constructor in commit `2adaa372`.

The three tests share a private `build_trivial_2dof_vec_env`
helper at `task.rs:1085` that constructs a minimal 2-DOF
`VecEnv` with a zero reward function, `always-false` done
predicate, and a `time > 1.0` truncation gate. The helper is
test-local and is not itself `#[test]`-annotated.

35. **`from_build_fn_metadata_roundtrip`** — calls
    `TaskConfig::from_build_fn("custom-seeded", 4, 2,
    vec![0.5, 0.5, 0.1, 0.1], |n_envs, _seed|
    build_trivial_2dof_vec_env(n_envs))` and asserts the
    four metadata accessors (`name`, `obs_dim`, `act_dim`,
    `obs_scale`) round-trip the constructor arguments
    verbatim. Then calls `build_vec_env(2, 0)` and
    `reset_all()` and asserts the observation tensor has
    shape `[2, 4]`. Covers the constructor's happy path.
    ~25 lines.
36. **`from_build_fn_threads_seed_into_closure`** — the
    load-bearing test for the entire from_build_fn surface.
    Constructs an `Arc<Mutex<Vec<u64>>>` that the closure
    pushes each observed seed into, then calls
    `build_vec_env` three times with distinct seeds
    (`12_345`, `67_890`, `0`) and asserts the recorded
    sequence matches. Proves that the per-replicate seed
    actually reaches the closure body — the invariant the
    synthesized `TaskConfigBuilder::build()` path
    deliberately does not satisfy. A refactor that silently
    dropped the seed parameter on the custom-builder path
    would fail this test. ~30 lines.
37. **`from_build_fn_propagates_env_error`** — constructs a
    task whose closure always returns
    `Err(EnvError::ZeroSubSteps)`; calls `build_vec_env(2, 0)`
    and asserts the error surfaces as `Err(ZeroSubSteps)`.
    Matches the error-propagation contract of the
    synthesized `TaskConfigBuilder::build()` closure. ~10
    lines.

## PR 3b commit 2 — Rematch integration test fixture

**Location:** one new integration test file at
`sim/L0/opt/tests/d2c_sr_rematch.rs`. Single `#[test]`
function. Chapter source: [Ch 42 §6.6](42-pr-3-sim-opt-rematch.md).

38. **`d2c_sr_rematch`** — single `#[test]` annotated with
    `#[ignore = "requires --release (~30-60 min)"]` per
    the `d2c_cem_training.rs` precedent. The function is
    ~90 lines: (1) matched-complexity gate assertion
    (`assert_eq!(cem_policy.n_params(), sa_policy.n_params())`
    plus both equal 3), (2) `Competition::new_verbose` with
    `TrainingBudget::Steps(REMATCH_BUDGET_STEPS)` and
    `N_ENVS = 32` (the `_verbose` constructor is a post-merge
    change at commit `a28dca05`, documented in Ch 50; the
    original draft used `Competition::new` and produced a
    silent run), (3) CEM and SA algorithm builders with
    hyperparameters inherited from `d2c_cem_training.rs:283`
    (CEM: `elite_fraction = 0.2`, `noise_std = 2.5`,
    `noise_decay = 0.98`, `noise_min = 0.1`) and [Ch 42
    §4.5](42-pr-3-sim-opt-rematch.md) (SA: `initial_temperature
    = 50.0`, `proposal_std = 0.5`, `cooling_decay = 0.955`,
    `temperature_min = 0.005`), both
    `LinearPolicy::set_params(&[0.0, 0.0, 2.0])` for
    matched-initial-conditions, (4) bootstrap RNG
    construction with `BOOTSTRAP_RNG_SEED = 0xB007_0057_00AA_0055`,
    (5) `run_rematch(&competition, &task, &algorithm_builders,
    &mut bootstrap_rng)`, and (6) verdict-print-and-pass via
    `match` on `RematchOutcome` with `eprintln!` for each
    variant.

The test's gate shape is **(α) protocol-completes-cleanly**
per sub-decision (g) at [Ch 42 §6.7](
42-pr-3-sim-opt-rematch.md). It does **not** assert a
specific `RematchOutcome` variant because [Ch 30](
30-the-scientific-question.md) names all three outcomes as
informative; asserting one would bake in a prior about
which outcome is expected and would contradict the
scientific framing.

The integration-test placement (rather than an inline
`#[cfg(test)]` module in `analysis.rs`) is defended at
[Ch 42 §6.8](42-pr-3-sim-opt-rematch.md): the fixture needs
`sim-mjcf`, `sim-core`, and `sim-thermostat` as
dev-dependencies that the sim-opt library itself does not
take. Moving it inline would pollute the crate's runtime
dependency graph.

## Tests the plans explicitly dropped

Two tests were named in Ch 41's original recon-to-leans
plan and then **dropped during Round 2 cold-read** in
favor of static analysis. They are listed here so a
reader sees the drop and knows why.

- **`reinforce_zero_fallback_skips_epoch`** — proposed
  runtime regression test for REINFORCE's `:213-225`
  zero-fallback skip-the-push. Dropped at [Ch 41 §2.6
  closing note](41-pr-2-competition-replicates.md).
  **Reason:** the zero-fallback branch is structurally
  unreachable from every currently-shipping task because
  `collect_episodic_rollout` runs every env through at
  least one step before evaluating trajectory
  completeness, and the reaching-task environments that
  every existing test trains on produce `n_samples > 0`
  by construction. A runtime test would require either
  instrumenting `collect_episodic_rollout` itself (out of
  PR 2's scope) or introducing a pathological zero-length-
  trajectory test fixture. Load-bearing safety argument
  falls back to static analysis: the `metrics.len() == N`
  census across six existing assertions, plus the
  `on_epoch` workspace grep confirming no in-repo caller
  counts callback invocations to infer `n_epochs`.
- **`ppo_zero_fallback_skips_epoch`** — proposed runtime
  regression test for PPO's `:313-325` zero-fallback
  skip-the-push. Dropped for the same reason.

A future PR that introduces an env or task whose rollout
can produce zero-length trajectories (e.g., a custom
benchmark with aggressive early termination) would be the
right time to revisit and add the runtime test. PR 2b is
not that PR, and Ch 41 names the deferral explicitly.

## What this appendix does not cover

- **Existing tests that the plans leave unchanged.** The
  ml-bridge crate has dozens of pre-existing tests in
  `competition.rs`, `cem.rs`, `td3.rs`, `sac.rs`,
  `reinforce.rs`, `ppo.rs`, `task.rs`, `vec_env.rs`, and
  others; the thermostat crate has tests for
  `BrownianThermostat`, `IsingLearner`, `Ratchet`,
  `DoubleWell`, and others. None of them are listed here
  unless the plans explicitly modify them.
- **`sim/L0/thermostat/tests/d2c_cem_training.rs`**
  specifically — the pre-existing D2c CEM training fixture
  that the study references throughout Part 3 and Part 4 as
  the scientific precedent for the rematch. It is the
  structural pattern the new `d2c_sr_rematch.rs` fixture was
  modeled on (same task name, same `SEED_BASE`, same
  matched-complexity anchor, same `eprintln!`-verdict-block
  gate shape). References live at Ch 32 §3.4, Ch 42 §4.5,
  Ch 42 §6.6, and test entry 38's hyperparameter citations.
  The test itself predates the study and is out of scope for
  this inventory, which covers only plans-added tests.
- **Test fixtures and helpers.** The `MockAlgorithm`
  variants used by PR 2a and PR 3a's analysis tests, the
  `rematch_task()` helper function at PR 3b's fixture,
  and the constants like `OBS_DIM`, `ACT_DIM`,
  `N_ENVS`, `EPISODE_STEPS`, `REMATCH_BUDGET_STEPS`
  inside the rematch fixture are test-local items;
  they are not separate `#[test]` functions and are
  not inventoried here.
- **Benchmark tests.** Criterion benchmarks in
  `sim/L0/core/benches/` and `sim/L0/ml-bridge/benches/`
  are not `#[test]` functions and are out of scope for
  the study's PR plans.
- **Golden-file tests.** None of the PR plans add
  golden-file comparisons.
