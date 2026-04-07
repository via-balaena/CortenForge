# Competition Tests Spec (Phase 3)

> **Status**: Draft
> **Crate**: sim-ml-bridge
> **Parent spec**: ML_COMPETITION_SPEC.md, Phase 3

## Context

Phase 2 is complete — all 5 algorithms (CEM, REINFORCE, PPO, TD3, SAC)
are implemented with 239 passing tests. Phase 3 adds integration tests
that use the `Competition` runner to validate the spec's hypotheses about
algorithm ordering. These are the dyno tests — the proof that the
algorithms behave as theory predicts.

## Files to modify

| File | Change |
|------|--------|
| `sim/L0/ml-bridge/src/competition.rs` | Add `print_summary()` to `CompetitionResult` |
| `sim/L0/ml-bridge/tests/competition.rs` | **New file** — all competition integration tests |

## Design decisions

### Integration test, not unit test

Tests go in `tests/competition.rs` (integration test), not inside
`src/competition.rs`. They exercise the full public API across algorithms,
policies, tasks, and the competition runner. The existing unit tests in
`src/competition.rs` cover runner mechanics with mocks — these tests cover
algorithm behavior.

### Per-hypothesis test functions

Each spec hypothesis gets its own `#[test] #[ignore]` function. This
allows running individual hypotheses
(`cargo test -p sim-ml-bridge --test competition hypothesis_cem -- --ignored`)
and gives better failure isolation than one monolithic test.

### SAC limitation

No `MlpStochasticPolicy` exists. SAC is limited to `LinearStochasticPolicy`
for the actor, even when other algorithms use MLP. This handicap is
documented in each affected test. Hypotheses 3 and 5 are partially
compromised and should be revisited when `MlpStochasticPolicy` is added.

### Conservative assertions

Single seed (42), no averaging. Assertions are relaxed compared to the
spec's multi-seed statistical expectations. If an assertion is flaky,
loosen it rather than add seed averaging. Failed hypotheses are findings,
not bugs.

## Step 1: Add `print_summary()` to `CompetitionResult`

In `src/competition.rs`, add to `impl CompetitionResult`:

```rust
pub fn print_summary(&self) {
    eprintln!("\n{:<20} {:<15} {:>14} {:>12} {:>10}",
        "Task", "Algorithm", "Final Reward", "Total Dones", "Wall (ms)");
    eprintln!("{}", "-".repeat(75));
    for run in &self.runs {
        let reward = run.final_reward()
            .map_or("N/A".to_string(), |r| format!("{r:.2}"));
        let wall: u64 = run.metrics.iter().map(|m| m.wall_time_ms).sum();
        eprintln!("{:<20} {:<15} {:>14} {:>12} {:>10}",
            run.task_name, run.algorithm_name, reward, run.total_dones(), wall);
    }
    eprintln!();
}
```

Uses `eprintln!` so output shows even under test capture (visible on
failure or with `--nocapture`). The spec explicitly envisions this method
(line 628 of ML_COMPETITION_SPEC.md).

## Step 2: Helpers and builder functions

### `improvement_pct` helper

```rust
fn improvement_pct(run: &RunResult) -> f64 {
    let first = run.metrics[0].mean_reward;
    let last = run.final_reward().unwrap();
    (last - first) / first.abs() * 100.0
}
```

Matches the spec's language ("91% reward improvement", "20-40%
improvement") and avoids repeating the math across assertions.

### Builder functions

10 builders in `tests/competition.rs`, one per (algorithm, policy_level):

```
build_cem_linear(task)       build_cem_mlp(task)
build_reinforce_linear(task) build_reinforce_mlp(task)
build_ppo_linear(task)       build_ppo_mlp(task)
build_td3_linear(task)       build_td3_mlp(task)
build_sac_linear(task)       build_sac_mlp(task)
```

Each returns `Box<dyn Algorithm>`.
Signature: `fn build_X(task: &TaskConfig) -> Box<dyn Algorithm>`.

### Hyperparams

Borrowed from existing smoke tests with adjusted budgets:

| Algorithm | Key hyperparams |
|-----------|----------------|
| CEM | `elite_fraction: 0.2, noise_std: 0.3, noise_decay: 0.95, noise_min: 0.01` |
| REINFORCE | `gamma: 0.99, sigma_init: 0.5, sigma_decay: 0.95, sigma_min: 0.05, lr: 0.05` |
| PPO | `clip_eps: 0.2, k_passes: 2, gamma: 0.99, gae_lambda: 0.95, sigma_init: 0.5, sigma_decay: 0.90, sigma_min: 0.05, lr: 0.025` |
| TD3 | `gamma: 0.99, tau: 0.005, policy_noise: 0.2, noise_clip: 0.5, exploration_noise: 0.1, policy_delay: 2, batch_size: 64, buffer_capacity: 50_000, warmup_steps: 200, lr: 3e-4` |
| SAC | `gamma: 0.99, tau: 0.005, alpha_init: 0.2, auto_alpha: true, target_entropy: -act_dim, alpha_lr: 3e-4, batch_size: 64, buffer_capacity: 50_000, warmup_steps: 200, lr: 3e-4` |

**Low-budget variant** (Test 4 only): TD3 builder uses
`warmup_steps: 100` instead of 200. At 20 envs, 200 warmup = 10 epochs
of random noise — half a 20-epoch budget wasted. 100 warmup = 5 epochs,
leaving 15 for real learning. (SAC is excluded from Test 4 due to
LinearStochasticPolicy handicap, so no low-warmup SAC builder needed.)

Builders branch on `task.act_dim()` for `max_episode_steps`:
- 2-DOF (act_dim=2): 300 steps
- 6-DOF (act_dim=6): 500 steps

MLP hidden dim: 32 (spec's 614-param actor on 6-DOF).

### SAC MLP note

`build_sac_mlp` uses `LinearStochasticPolicy` for the actor (no
`MlpStochasticPolicy` exists) but `MlpQ` for the critics. The actor
is the bottleneck — documented in each test that uses it.

## Step 3: Test functions

All tests use `#[test] #[ignore]`. Run via:
```
cargo test -p sim-ml-bridge --test competition -- --ignored --nocapture
```

### Test 1: `competition_2dof_all_linear`

All 5 algorithms, linear policies, 2-DOF. Regression baseline.

- **Budget**: 30 epochs, 20 envs
- **Asserts**:
  - All 5 produce metrics (no panics, no NaN)
  - All metrics values are finite
  - All 5 show reward improvement: `final_reward > first_reward`

### Test 2: `hypothesis_cem_scales_poorly`

Spec hypothesis 1: CEM scales poorly with param count.

- **Budget**: 30 epochs, 50 envs
- **Setup**: CEM linear on 2-DOF, CEM MLP on 6-DOF
- **Asserts**:
  - CEM 2-DOF: `total_dones >= 3` (10-param search with 50 candidates works)
  - CEM 6-DOF MLP: `total_dones <= 2` (614-param search, sample-starved —
    allow 1-2 lucky reaches rather than hard zero, but prove it's
    fundamentally inadequate)
  - CEM 2-DOF `final_reward` > CEM 6-DOF `final_reward`

### Test 3: `hypothesis_value_fn_matters_at_scale`

Spec hypothesis 2: PPO's value function matters at scale.

- **Budget**: 40 epochs, 50 envs
- **Setup**: PPO MLP and REINFORCE MLP on 6-DOF
- **Asserts**:
  - `ppo.final_reward() > reinforce.final_reward()` (learned baseline helps)
  - `ppo.total_dones() > reinforce.total_dones()` (PPO reaches targets)

### Test 4: `hypothesis_off_policy_efficiency`

Spec hypothesis 3: Off-policy methods are more sample-efficient.

- **Budget**: 20 epochs, 20 envs (deliberately low — forces efficiency to matter)
- **Setup**: TD3 MLP, PPO MLP, CEM MLP on 6-DOF
- **Off-policy builders use `warmup_steps: 100`** (not 200) — at 20 envs,
  200 warmup = 10 epochs of noise, half the budget. 100 warmup = 5 epochs,
  leaving 15 for real learning.
- **Asserts**:
  - `td3.final_reward() > cem.final_reward()` (off-policy reuse > evolutionary)
  - `ppo.final_reward() > cem.final_reward()` (gradient + baseline > evolutionary)
  - Document: SAC excluded — LinearStochasticPolicy handicap makes comparison unfair

### Test 5: `hypothesis_mlp_beats_linear`

Spec hypothesis 4: MLP >> linear for complex tasks.

- **Budget**: 40 epochs, 30 envs
- **Setup**: PPO with MLP vs PPO with linear, both on 6-DOF
- **Asserts**:
  - `ppo_mlp.final_reward() > ppo_linear.final_reward()`
  - `ppo_mlp.total_dones() > ppo_linear.total_dones()`

### Test 6: `hypothesis_entropy_helps`

Spec hypothesis 5: Entropy-driven exploration (SAC) > deterministic (TD3).

- **Budget**: 40 epochs, 30 envs
- **Setup**: SAC linear and TD3 linear on 6-DOF (fair — both linear)
- **Asserts**:
  - **Precondition**: both algorithms must show >10% reward improvement
    via `improvement_pct()`. If neither learns, the test is a finding
    (linear policies too weak for 6-DOF off-policy), not a pass. Skip
    the comparison and print a diagnostic instead.
  - If precondition holds: `sac.final_reward() >= td3.final_reward()`
    (entropy aids exploration). Compare on `final_reward` rather than
    `total_dones` — linear policies on 6-DOF may never produce dones,
    making `0 >= 0` a trivial pass.
  - Document: both linear policies, revisit with MLP when available

### Test 7: `competition_6dof_all_mlp`

Full sweep — all 5 algorithms, MLP policies, 6-DOF. The headline test.
Run this after Tests 2-6 pass to get the complete picture at maximum
budget. This test overlaps with Tests 3 and 5 but at higher budget
(50 epochs / 50 envs vs 20-40 / 20-30), which may reveal behavior
the focused tests miss.

- **Budget**: 50 epochs, 50 envs
- **Setup**: All 5 algorithms with MLP (SAC with LinearStochasticPolicy)
- **Asserts**:
  - `reinforce.final_reward() > cem.final_reward()` (gradient > evolutionary)
  - `ppo.final_reward() > reinforce.final_reward()` (baseline matters at scale)
  - Prints full summary table for diagnosis

## Step 4: Run and tune

Run each test individually. Adjust assertion thresholds based on actual
single-seed results. The spec's expected ranges are guides — the actual
numbers may differ. Document actual observed values in test comments.

## Verification

```bash
# Quick: existing tests still pass
cargo test -p sim-ml-bridge --lib

# Run one hypothesis (fast feedback)
cargo test -p sim-ml-bridge --test competition hypothesis_cem -- --ignored --nocapture

# Run all competition tests (full sweep — expect 10-20 min)
cargo test -p sim-ml-bridge --test competition -- --ignored --nocapture
```

## Risks

- **Wall time**: The full sweep may take 20+ min. Individual hypotheses
  should be 2-5 min each.
- **Off-policy warmup**: TD3/SAC burn 200 steps on random exploration.
  With 50 envs, that's 4 epochs of noise. May need to reduce warmup
  or increase budget.
- **Seed sensitivity**: If seed 42 gives atypical results, try 0, 123
  before changing assertions. Multi-seed is future work.
- **SAC handicap**: Hypotheses involving SAC are partially compromised
  until `MlpStochasticPolicy` exists. Document, don't suppress.
