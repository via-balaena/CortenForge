# Single-seed is broken

The second finding from Part 0: the experiment runner that is
supposed to orchestrate the D2c rematch — `Competition` — takes a
single integer seed and passes it to every algorithm in a run. The
rematch, as specified in the construction document, runs SA, CEM,
PPO, and TD3 at `SEED = 20_260_412` and reports the winner. This
chapter is the argument that a single-seed comparison is not a
sound experimental protocol for reinforcement learning, that the
chassis bakes the flaw in at the API layer, and that the rematch
test as drafted inherits the flaw directly.

There is a published, well-known version of this argument. It is
not a subtle or contested point. What the chapter adds is the
specific mapping onto our codebase, and the specific observation
that the `Competition` API is *structurally mismatched* with a
multi-seed experiment — not literally unable to host one (a
determined test author can loop in client code), but mismatched
in a way that makes every cross-algorithm benchmark reinvent the
same wheel at its own risk. The chassis should carry the
multi-seed contract once, so that every subsequent experiment
inherits it rather than re-deriving it.

## What the API currently looks like

The runner's constructor is

```rust
pub const fn new(n_envs: usize, budget: TrainingBudget, seed: u64) -> Self
```

in `sim/L0/ml-bridge/src/competition.rs`. The `seed: u64` is stored
on the struct and handed to every algorithm's `train` call inside
`Competition::run`:

```rust
let metrics = algorithm.train(&mut env, self.budget, self.seed, &|m| { ... });
```

One seed, one train call per algorithm, one reported result per
algorithm per task. There is no vector type anywhere on the hot
path — the shape of the API says, at the type level, "a competition
is one seed's worth of experiment."

This is the core problem. Not the scalar-vs-vector detail, but the
fact that the *type* of the experiment is "single replicate." Every
downstream operation — the result table, the ranking, the gate
condition in the rematch test — is defined in terms of one number
per algorithm per task. Introducing replicates as a first-class
chassis capability requires re-typing the result surface to nest
per-seed runs, re-typing the gate to operate on distributions, and
re-typing the per-algorithm `best_reward` reporting to mean "best
across replicates" rather than "best within the single run." That
is a Competition v2, not a patch — and the later section "Why the
fix belongs in the chassis" argues why it needs to be done as a
v2 rather than papered over in client code.

## What Henderson et al. 2018 demonstrated

The standard reference for why single-seed RL comparisons are
unreliable is Henderson, Islam, Bachman, Pineau, Precup, and Meger,
*Deep Reinforcement Learning that Matters* (AAAI 2018; arXiv
1709.06560). The paper's core empirical finding, for our purposes,
is that seed-to-seed variance *within* a single algorithm — same
hyperparameters, same environment, same code, different RNG seeds —
is routinely large enough that you can partition ten runs of the
same algorithm into two groups of five and produce learning curves
that look indistinguishable from the between-algorithm gaps people
report in papers. In the paper's own framing, "the variance between
runs is enough to create statistically different distributions just
from varying random seeds." In other words: if you run algorithm A
with seeds 1–5 and algorithm B with seeds 6–10 and find that A
wins, a non-trivial fraction of the time you would have found B
winning if you had run A with seeds 6–10 and B with seeds 1–5. The
apparent ranking is not a property of the algorithms; it is a
property of the seed partition.

The paper's recommended fix is straightforward and now widely
adopted: run each algorithm with multiple seeds, report mean
performance with a confidence interval or standard error, and use
a proper significance test (the paper discusses bootstrap
confidence intervals, two-sample t-tests, and Kolmogorov–Smirnov)
rather than a raw ranking to claim "algorithm A beats algorithm B."
Henderson et al. explicitly decline to name a fixed number of seeds
and point at power analysis instead; the follow-up work by Colas,
Sigaud, and Oudeyer, *How Many Random Seeds?* (arXiv 1806.08295),
is the one that actually computes concrete seed counts for common
effect sizes. The exact threshold and statistical test remain
debated; the requirement to run multiple seeds is not.

A single-seed comparison can, in principle, still be informative —
if the effect size you are trying to detect is much larger than the
seed-to-seed variance of both algorithms, a single seed is enough.
**But establishing that precondition requires knowing the variance,
which requires running multiple seeds.** The asymmetry is the whole
point: multi-seed is what *licenses* single-seed, not the other way
around. You cannot skip multi-seed on the argument that "our effect
is probably big," because "probably big" is exactly the claim multi-
seed would justify or disprove. The only situations in which single-
seed is genuinely safe are situations in which you have already run
multi-seed and decided the variance is negligible. For any
experiment that has not been through that calibration, single-seed
is a way of not-knowing dressed up as a way of knowing.

## Our specific case: the rematch

The construction spec defines the D2c rematch test as a single call
to `Competition::new_verbose(N_ENVS, TrainingBudget::Epochs(N_EPOCHS), SEED)`
with `const SEED: u64 = 20_260_412`, followed by a call to
`comp.run(...)` and a gate `sa_best >= best_rl` that compares
scalar `best_reward` values. Every element of that protocol is
single-seed: the competition object is single-seed by type, the
returned `sa_best` is single-seed by type, the gate is a scalar
comparison with no notion of variance or confidence.

The relevant effect size is whatever margin by which we expect
physics-aware SA to beat matched-complexity RL on a stochastic-
resonance task. The D2 SR findings memo records the prior result:
the original D2c run had CEM finding "a flat, broad kT band where
reward sits near the maximum" but unable to resolve the peak; SA
should, in principle, do better because Metropolis accept/reject
with a small Gaussian proposal is geometry-appropriate for that
landscape. How much better? The honest answer is that we do not
know. The rematch exists to find out. And until we know, we cannot
know whether a single-seed comparison is large enough to dominate
seed variance or not — which is exactly the scenario Henderson
tells us to treat as "run more seeds."

There is an additional wrinkle specific to our case. SR is a flat,
broad landscape: multiple points on the optimization surface have
near-identical reward. Which point any particular run lands on is
precisely the kind of thing seed-sensitive stochastic optimizers
are expected to disagree about. The variance we expect to measure
is not small. The single-seed gate is not just statistically
questionable in general; it is questionable in the specific
experiment the rematch is running.

## Why the fix belongs in the chassis

The fix is not *literally impossible* in client code. A determined
test author could write a thirty-line helper that calls
`comp.run()` five times, pulls `sa.best_reward()` and
`cem.best_reward()` (etc.) out of each `CompetitionResult`,
collects them into `Vec<f64>`, and runs a scipy-equivalent
significance test in the test file. For the one-shot rematch
test, that would work. Calling the current API "structurally
unable" to run the experiment would overclaim — what is true is
that the current API is *structurally mismatched* with the
experiment, and the mismatch has to be papered over somewhere.

The question is *where*. If the mismatch lives in the test file,
every subsequent cross-algorithm benchmark — D3, D4, any future
rematch against any future landscape — re-implements the same
thirty lines, and every implementation has its own chance to get
the significance test wrong. The chassis is the place where
cross-algorithm benchmarks live; a benchmark protocol is a
chassis feature, not a per-test feature. Making multi-seed
comparison a first-class chassis capability is what keeps the
gate honest across the next several experiments without each one
paying a tax for it. That is the real reason the fix belongs at
the type signature and not in the test file — not that the loop
is impossible, but that having every test re-implement the loop
is worse than having none of them do it.

Concretely, the right shape is a `Competition` that takes a list
of seeds up front, loops internally, and returns a result object
whose leaves are `Vec<f64>` rather than `f64`. The gate then
operates on those vectors with an explicit statistical test. This
is a small amount of code, but it cannot happen without changing
the type of `CompetitionResult`, and changing that type is a
breaking change to every caller. `CompetitionResult` is currently
a flat list of `(task, algorithm, best_reward)` triples with no
slot for replicate index, and `result.print_ranked(...)` operates
on that flat list. `best_reward` itself is a per-algorithm
scalar, and chapter 24 will show that its meaning already differs
across algorithms in subtle ways that multi-seed aggregation will
make worse, not better.

## What this chapter does not decide

It does not specify the exact API shape. Whether the new
constructor takes `seeds: &[u64]` or `seeds: impl IntoIterator<Item
= u64>`, whether the result type carries raw `Vec<f64>` or a
lightweight `SeedSummary { mean, stderr, n }`, what statistical
test the gate uses, what counts as "enough" replicates for the D2c
rematch specifically — all of those are chapter 23 (the Competition
v2 API shape) and chapter 24 (result semantics) questions. Both
chapters are flagged for human review because the API becomes
load-bearing the moment the PR lands and any subsequent shape
change breaks every caller.

The point of this chapter is only to establish that the current
`Competition` shape is structurally mismatched with the experiment
the rematch needs to run — not *impossible*, but mismatched in a
way that forces every future cross-algorithm benchmark to pay a
tax the chassis should be paying once — and that the fix lives at
the type signature, not inside the implementation of `run`.
