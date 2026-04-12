# `TrainingBudget::Steps` audit

The Part 2 chapters are about the algorithm surface of the study,
not the chassis. Part 1 was about where stochastic state lives and
how parallel stepping interacts with it. Part 2 is about what the
RL algorithms on top of that chassis do with the budget they are
given, whether that budget means the same thing across algorithms,
and whether a rematch that uses the budget as its fairness lever
can trust it to behave consistently. This chapter does the first
piece of that work: an audit of how every `Algorithm` implementor
in `sim/L0/ml-bridge/src/` handles `TrainingBudget::Steps(N)`.

The question the audit is trying to answer is narrow and concrete.
The `D2c` rematch the study is ultimately working toward needs a
compute-parity mechanism: a way to say "every algorithm gets the
same amount of training work" that is defensible under scrutiny
and that produces results the rematch can be judged against. The
natural candidate is `TrainingBudget::Steps(16_000_000)`, because
"total env steps" is the standard way RL compute parity is reported
in the literature, and because it is the unit the existing
`TrainingBudget` enum already supports. The audit's job is to
verify that this candidate actually works the way the rematch will
assume it does — that every algorithm converts the budget into
the same amount of env work, that no algorithm silently broadens
or narrows the interpretation, and that the reporting each
algorithm does at the end of a run is comparable across algorithms.

As with the other Phase 2 chapters, every file:line citation in
this chapter is recon-reported and verified against current source
in the factual pass before commit.

## The uniform Steps-to-epochs formula

The audit's first and most significant finding is that every
algorithm implementation uses the same formula to convert
`TrainingBudget::Steps` into an internal epoch count:

```rust
let n_epochs = match budget {
    TrainingBudget::Epochs(n) => n,
    TrainingBudget::Steps(s) => s / (n_envs * max_episode_steps).max(1),
};
```

This pattern appears in `cem.rs:141–144`, `reinforce.rs:168–171`,
`ppo.rs:225–228`, `td3.rs:239–242`, and `sac.rs:266–269`. Five
implementations, five identical match blocks, no divergence. The
divisor `n_envs * max_episode_steps` is the worst-case per-epoch
env-step cost (every env runs to the full episode horizon), and
the `.max(1)` guard protects against degenerate configurations
that would otherwise produce zero epochs. No implementation uses
a hardcoded `n_envs` value; no implementation substitutes
`unimplemented!()` or `todo!()`; no implementation has a divisor
that differs from the others. The formula is arithmetically
consistent across the algorithm surface.

Concretely: under the D2c rematch's intended configuration of
`Steps(16_000_000)`, `n_envs = 32`, `max_episode_steps = 5000`,
every algorithm computes
`16_000_000 / (32 * 5000) = 100` epochs. Same input, same output,
five times over.

The computed `n_epochs` is also the literal iteration count of
each algorithm's outer training loop: `for epoch in 0..n_epochs`
appears at `cem.rs:153`, `reinforce.rs:175`, `ppo.rs:232`,
`td3.rs:259`, and `sac.rs:286`. None of the five has an outer-loop
early exit, none loops past the declared count, none substitutes
a different stopping condition. Budget-to-epochs conversion is
uniform and epoch count is honored as a hard iteration limit.

The implication for the rematch is that `TrainingBudget::Steps` is
a well-defined compute-parity unit at the budget-enforcement
level. Every algorithm receives a budget of 16M steps and runs
exactly 100 epochs of roll-and-train, and the budget formula does
not silently give any algorithm a head start or a handicap. The
fairness question does not end there — the rest of the chapter
and chapter 22 argue that — but the single biggest risk a
compute-parity mechanism faces (that different implementations
interpret the budget unit differently) is absent. The formula is
consistent across the algorithm surface.

## `total_steps` semantics split on-policy vs off-policy

The second finding is less relieving and is the reason this
audit is longer than one section. Each `Algorithm::train`
implementation returns, among other things, a `total_steps`
field that is nominally "the number of env steps consumed during
the training run." The field has the same name and the same type
across algorithms. But the way each algorithm computes the value
is different in a way that matters for cross-algorithm
comparison.

**On-policy algorithms (CEM, REINFORCE, PPO).** These compute
`total_steps` as the literal sum of trajectory lengths in each
epoch's rollout. The code pattern is

```rust
let epoch_steps: usize =
    rollout.trajectories.iter().map(Trajectory::len).sum();
```

at `cem.rs:208`, `reinforce.rs:264–268`, and `ppo.rs:418–422`.
Because trajectories may terminate early — via `done` when the
agent reaches a terminal state, or via `truncated` when the
episode horizon is hit — the sum is bounded above by
`n_envs * max_episode_steps` and in practice is often less.
On-policy `total_steps` is the *actual number of env steps
consumed*. Each entry in the sum is a single env's actual
episode length for that epoch.

**Off-policy algorithms (TD3, SAC).** These compute
`total_steps` by incrementing a counter inside the inner stepping
loop:

```rust
epoch_steps += n_envs;
```

at `td3.rs:309` and `sac.rs:336`. The counter advances by `n_envs`
on every iteration of the `for _step in 0..max_episode_steps`
inner loop. If individual envs have completed an episode
(done-or-truncated) and been auto-reset to a new starting state,
their post-reset iterations still count toward `epoch_steps`. The
inner loop is not bounded above at its stated maximum — TD3 and
SAC both contain an early-break at `td3.rs:273–275` and
`sac.rs:300–302`:

```rust
if env_complete.iter().all(|&c| c) {
    break;
}
```

The break fires when *all* envs have completed at least one
episode, at which point the epoch ends early. So off-policy
`epoch_steps` is not a fixed ceiling — it is
(inner-loop iterations until all-complete) × `n_envs`, where
individual-env resets during the inner loop inflate the count
relative to an on-policy sum of actual trajectory lengths.

The two reporting definitions are not subtly different. On-policy
reports the literal sum of this epoch's trajectory lengths;
off-policy reports inner-loop work expressed as
(iterations × `n_envs`), which systematically over-counts
individual envs that completed early and were reset mid-inner-loop.
At identical `Steps(N)` budgets an on-policy algorithm whose
episodes terminate early will report a smaller `total_steps` than
an off-policy algorithm doing the same amount of inner-loop work,
and a comparison that reads those numbers side-by-side without
knowing the counting conventions will mis-rank them.

The important distinction here is that this is **not a
budget-enforcement bug**. The budget formula uses
`n_envs * max_episode_steps` as its divisor — the worst-case
ceiling — and all five algorithms run the computed number of
epochs. What each algorithm reports about the *work inside an
epoch* is what differs. Chapter 22 is where the design call for
how to reconcile the two reporting conventions gets made. The
audit's job is to establish that the divergence exists, name
where it comes from, and leave the reconciliation to the chapter
that has the standing to decide.

## PPO's `k_passes` is a wall-clock factor, not a budget factor

One short note for completeness. After each rollout, PPO runs
`k_passes` SGD passes over the frozen rollout data (rollout
stepping at `ppo.rs:244–260`, the SGD passes at `ppo.rs:348–411`).
Each pass is a gradient update, not an env step: the passes cost
wall-clock time but do not advance `total_steps` and do not affect
the budget formula. A reader who sees PPO take longer than CEM at
the same `Steps(16M)` budget is seeing the k_passes cost, not a
budget difference. The env-steps-versus-wall-clock framing is
chapter 22's call.

## Warmup overhead in TD3 and SAC

The final audit finding is smaller in absolute terms than the
semantics split but interacts with it and deserves recording.
TD3 and SAC both have a warmup phase at the start of training —
a period of random exploration that fills the replay buffer
before any gradient updates happen. The warmup is at
`td3.rs:282–286` and `sac.rs:309–313` in the current tree. It
is controlled by a `warmup_steps` hyperparameter, which the D2c
TD3 configuration in `sim/L0/thermostat/tests/d2c_cem_training.rs`
sets to `1000`.

The warmup is not deducted from the `TrainingBudget::Steps`
budget before the formula runs. A rematch configured with
`Steps(16_000_000)`, `n_envs = 32`, `warmup_steps = 1000` has
TD3 spending roughly `32 * 1000 = 32,000` env steps on random
exploration before any gradient update, which is 0.2 percent of
the nominal 16M budget. In absolute terms the overhead is
small. The point of naming it is not that 0.2 percent is
individually significant but that it is invisible in the
reporting: because off-policy's `total_steps` lumps warmup
iterations together with training iterations, a reader cannot
separate the two from the reported value alone, and any
statement of the form "TD3 reached reward R after 16M training
steps" silently elides that the first 32,000 of those were
random. Chapter 22 will decide whether to deduct warmup from
the budget, tag it in the reporting, or document the overhead
and live with it.

## What this audit has not covered

Three adjacent concerns belong elsewhere.

**`best_reward` semantics.** A parallel divergence exists in
`best_reward` — the five algorithms compute per-epoch mean
reward in three different ways, and the construction spec's
existing D2c rematch test assertion compares these directly.
Chapter 24 audits this specifically and proposes a resolution.

**`TrainingBudget::Epochs`.** The budget enum has two arms and
this chapter audits only one. The `Epochs(n)` arm passes `n`
through as the iteration count, which means "one epoch" is
whatever each algorithm's epoch structure says it is — and per
the recon, an epoch means quite different things across
algorithm families (CEM: one population-scale rollout;
REINFORCE/PPO: one rollout then gradient update; TD3/SAC:
inner stepping until all envs complete one episode). The
rematch uses `Steps`, so `Epochs` is out of scope for this
audit, but a reader should know that if they ever switch the
budget unit, the uniformity finding of the first section does
not automatically carry over.

**Rematch failure modes.** The budget-reporting divergence
this chapter establishes is one of the mechanisms by which a
rematch result can mislead an honest reader. Chapter 31 will
enumerate rematch failure modes and spec the guards; Ch 21's
contribution is only to identify the specific reporting
mechanism that enables one of them.

## What the audit establishes

The primary finding is the uniform Steps-to-epochs formula:
every algorithm converts `TrainingBudget::Steps(N)` into
`N / (n_envs * max_episode_steps).max(1)` epochs, runs exactly
that many outer-loop iterations, and the rematch's
compute-parity mechanism is therefore sound at the
budget-enforcement level. The primary complication is the
on-policy versus off-policy `total_steps` divergence: on-policy
reports an exact sum of this epoch's trajectory lengths, and
off-policy reports inner-loop iterations-until-all-complete
times `n_envs`, which systematically over-counts individual
envs that reset mid-inner-loop. A writeup that reads the
numbers side-by-side without knowing the counting conventions
will draw conclusions that neither convention licenses. The
PPO `k_passes` observation and the TD3/SAC warmup overhead are
smaller findings that interact with the `total_steps` problem.
All three are chapter 22 design calls. None of them invalidates
the uniform formula, and none of them is an implementation bug
in the strict sense — each is something the current code does
on purpose, and the question for chapter 22 is whether the
purposes are still the right ones under a rematch's demands.
