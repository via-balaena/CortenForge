# Recon session C — Algorithm TrainingBudget::Steps audit

> **Status:** Raw recon, unverified. Do **not** lift text from this file
> directly into book chapters without first running the review protocol
> described in chapter 01 (factual pass against current source). Some
> file:line citations may have shifted since this was recorded.
>
> **When:** 2026-04-12, during the same session that committed chapters
> 00 and 01.
> **Method:** `Agent` tool with the `Explore` subagent, scoped prompt.
> **Used by:** Chapter 21 (TrainingBudget::Steps audit), chapter 22
> (Fair compute parity), chapter 24 (Result semantics), chapter 31
> (Failure modes).

## What the session asked

A systematic audit of every `Algorithm` implementation in
`sim/L0/ml-bridge/src/` to report exactly how each one handles
`TrainingBudget::Steps(N)`, where it converts Steps to an internal
epoch count, whether the conversion is sound, and what
`EpochMetrics::total_steps` semantically means for each algorithm.
The purpose was to verify that a rematch using `Steps(16M)` as the
budget unit would be honored identically by every algorithm.

## Key findings

### 1. All five algorithms use the SAME Steps formula

Reported across `cem.rs`, `reinforce.rs`, `ppo.rs`, `td3.rs`,
`sac.rs` (unverified):

```rust
let n_epochs = match budget {
    TrainingBudget::Epochs(n) => n,
    TrainingBudget::Steps(s) => s / (n_envs * max_episode_steps).max(1),
};
```

At `Steps(16_000_000), n_envs=32, max_episode_steps=5000`:
```
16_000_000 / (32 * 5000) = 100
```

**Every algorithm computes 100 epochs** from the same input. The
formula is consistent. No `unimplemented!()`, no `todo!()`, no
broken divisors, no hardcoded `n_envs`, no edge case producing
0 epochs (the `.max(1)` guard is present in all five).

Reported file:line citations:

- `cem.rs:141-144` — the match block
- `reinforce.rs:168-171`
- `ppo.rs:225-228`
- `td3.rs:239-242`
- `sac.rs:266-269`

**Rematch implication:** `TrainingBudget::Steps(16_000_000)` is a
well-defined, consistent budget unit across all five RL baselines.
Safe to use as the compute-parity mechanism for the SR rematch.

### 2. `total_steps` semantics split cleanly on on-policy / off-policy

This is the subtle finding and the one that affects the rematch's
reporting.

**On-policy algorithms (CEM, REINFORCE, PPO):**

`total_steps = sum(trajectory.len() for trajectory in rollout.trajectories)`

This is the **actual number of environment steps** consumed in the
epoch. Episodes may terminate early (via `done` or `truncated`),
so the sum is <= `n_envs * max_episode_steps`.

Reported at:
- `cem.rs:208` — `let epoch_steps: usize = rollout.trajectories.iter().map(Trajectory::len).sum();`
- `reinforce.rs:264-268` — same pattern
- `ppo.rs:418-422` — same pattern

**Off-policy algorithms (TD3, SAC):**

`epoch_steps += n_envs` per inner-loop iteration, regardless of
whether episodes have completed early.

Reported at:
- `td3.rs:309` — `epoch_steps += n_envs;` inside the `for _step in 0..max_episode_steps` loop
- `sac.rs:336` — same pattern

Result: off-policy `total_steps` is always `n_envs * (inner_loop_iterations)`,
which is a **worst-case ceiling**, not actual step count. If some envs
terminated early (done) and got auto-reset mid-loop, their post-reset
steps still count.

**Mixed comparison:** if a rematch reports "algorithm X consumed Y total
steps," the numbers mean different things across algorithms. CEM at
100 epochs might report fewer total steps than TD3 at 100 epochs even
if they did the same amount of actual work, because CEM counts honestly
and TD3 counts the ceiling.

This is **not** a budget-enforcement bug — the budget formula uses
`max_episode_steps`, which matches the off-policy ceiling counting. It's
a **reporting-honesty** bug: two different algorithms report incomparable
numbers under the same field name.

### 3. Warmup overhead in TD3 and SAC

Reported at:
- `td3.rs:282-286` — warmup phase (random exploration before gradient updates)
- `sac.rs:309-313` — same pattern

The warmup counts toward `epoch_steps` (because it's incremented
in the stepping loop). It does **not** get deducted from the
`TrainingBudget::Steps` budget before the formula runs — the
formula assumes zero warmup.

**For the rematch:** at `Steps(16_000_000)` with `warmup_steps=1000`
(the D2c TD3 config), TD3 burns `32 * 1000 = 32,000` env steps on
random exploration before the first gradient update. That's 0.2% of
the 16M budget. Small in absolute terms but:

1. It's unbudgeted overhead — off-policy gets less *effective*
   training than on-policy at the same declared budget
2. It's unreported — the `total_steps` at the end doesn't distinguish
   warmup steps from training steps

### 4. Per-epoch rollout structure differs by algorithm family

- **CEM**: 1 rollout per epoch. The rollout evaluates `n_envs`
  different candidates (one per env). Population-based search.
- **REINFORCE**: 1 rollout per epoch. On-policy. All `n_envs`
  trajectories discarded after the epoch's gradient update.
- **PPO**: 1 rollout per epoch. Then `k_passes` SGD passes on the
  frozen rollout data. `k_passes` costs CPU/wall-time but does **not**
  consume env steps. Reported at `ppo.rs:244-260` (rollout), then
  `348-411` (k_passes SGD).
- **TD3**: incremental stepping inside `for _step in 0..max_episode_steps`.
  Transitions go to replay buffer; gradient updates happen incrementally
  when buffer is full enough. An "epoch" ends when all envs have
  completed at least one episode (meaning auto-reset has fired for
  all of them).
- **SAC**: same structure as TD3, plus auto-tuned entropy temperature
  α (additional gradient update per step, no extra env cost).

**Wall-clock fairness vs step-budget fairness:**

At the same `Steps(16M)` budget:

- CEM runs 100 epochs at ~modest cost per epoch (one rollout each)
- PPO runs 100 epochs and does `k_passes * 100` gradient updates
- TD3/SAC run 100 epochs with many more gradient updates each
  (one per step, once buffer is warm)

CEM is the cheapest wall-clock, PPO middle, TD3/SAC most expensive.
But all four consume the *same* environment steps. If the budget is
"env steps" (hardware-rented time for physics), the comparison is
fair. If the budget is "wall clock" or "gradient updates," the
algorithms are not being compared on the same basis.

**Chapter 22 design call:** what's the benchmark unit? Env steps is
the standard RL answer, and it matches what `TrainingBudget::Steps`
already does. Wall clock would be different and probably not what
we want.

### 5. `best_reward` semantics across the algorithms

All five algorithms call `self.best.maybe_update(epoch, reward,
self.policy.params())` where `reward` is the per-epoch mean:

- **CEM**: `mean_reward = fitness.iter().map(|(_, f)| f).sum() / n_envs`
  where `fitness[i]` is the per-episode total reward / episode length
  (i.e., per-step mean reward). Reported at `cem.rs:209,213`.
- **REINFORCE, PPO**: `mean_reward = total_episodic_return / n_envs`
  (per-episode total, not per-step). Reported at `reinforce.rs:269-274`
  and `ppo.rs:423-428`.
- **TD3, SAC**: `mean_reward = mean of all episodes completed
  during the epoch` — may be 1 or many episodes, depending on how
  many auto-resets fired. Reported at `td3.rs:487-491` and
  `sac.rs:540-544`.

**Cross-algorithm comparison concern:** on-policy algorithms report
per-episode return; CEM reports per-step mean; off-policy reports
mean-over-completed-episodes. These are not the same quantity.

**Chapter 24 design call:** for the rematch, should the gate be
based on `best_reward` (as currently defined per-algorithm) or on
a unified metric computed from the rollout data? If unified, what's
the metric?

## My synthesis notes

1. **The Steps formula being uniform is the single biggest relief
   from this audit.** It means the rematch's compute-parity mechanism
   works. No per-algorithm special-casing needed at the budget level.

2. **The total_steps semantic split is a real issue for honest
   reporting.** Chapter 21 should flag it and chapter 22 should
   propose either (a) fix the off-policy algorithms to count
   actual steps, or (b) document the split and live with it, or
   (c) introduce a new metric (e.g., `effective_env_steps`) that
   everyone reports consistently.

3. **The warmup overhead is small but symbolically important.**
   Even 0.2% is measurable if the effect size is close. And "TD3
   runs less training" is exactly the kind of thing that can flip
   a rematch result.

4. **The best_reward semantic mismatch is a new finding not
   covered by the construction spec.** The construction spec's §7
   D2c rematch test asserts `sa_best >= max(cem_best, ppo_best,
   td3_best)` where `*_best` is the `RunResult::best_reward`
   field — which means the assertion is comparing per-step mean
   (CEM), per-episode total (PPO), and mean-over-episodes (TD3).
   Apples to oranges. Chapter 24 has to resolve this before the
   rematch can be trusted.

5. **PPO's k_passes is a red herring for compute parity but a
   real factor for wall-clock comparison.** If someone asks "why
   does PPO take longer than CEM at the same budget," the answer
   is k_passes * epoch_cost of SGD, not env steps. Worth a sentence
   in chapter 21.

## Caveats and verification status

- **Every file:line citation in this document is agent-reported,
  not verified.** Factual pass is mandatory before chapter use.
- **The "0.2% of 16M" warmup overhead calculation is a simple
  arithmetic check but depends on the `warmup_steps=1000` value
  from the D2c config.** Should be grep-verified against
  `d2c_cem_training.rs`'s Td3Hyperparams block.
- **The best_reward semantic mismatch is my synthesis, not agent
  C's direct finding.** Agent C observed that each algorithm
  computes `mean_reward` differently; I connected that to the
  construction spec's rematch gate. Worth double-checking the
  construction spec's exact gate language before lifting this to
  chapter 24.

## Open questions surfaced (deferred to Phase 3 / chapter design)

1. **Fix total_steps in off-policy?** Pro: honest reporting. Con:
   breaks backward compatibility with any existing analysis that
   reads those numbers.
2. **Unify best_reward metric?** Same trade-off. Changing the
   best_reward definition in any of the 5 algorithms is a
   cross-cutting change.
3. **Deduct warmup from budget?** If off-policy's `Steps(N)` should
   give N *training* steps, then warmup should be added on top.
   Simpler: document that off-policy's budget includes warmup.
4. **Report `total_env_steps` vs `total_training_steps`?** Could
   add a second field that's computed the same way for all
   algorithms and deprecate the current `total_steps`.

## What Phase 2 chapter drafts need from this file

- **Chapter 21 (TrainingBudget::Steps audit):** the uniform formula
  finding, the on-policy vs off-policy total_steps split, the PPO
  k_passes note, the warmup overhead in off-policy.
- **Chapter 22 (Fair compute parity):** the env-steps-vs-wall-clock
  framing, the decision for env steps as the rematch budget unit,
  the warmup overhead decision.
- **Chapter 24 (Result semantics):** the best_reward semantic
  mismatch across algorithms, the proposal for a unified metric.
- **Chapter 31 (Failure modes):** how any of these semantic issues
  could produce a misleading rematch result and how to detect that.
