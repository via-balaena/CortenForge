# Result semantics

Chapter 20 established that `Competition`'s single-seed shape is
broken and that `best_reward` itself is a per-algorithm scalar
whose meaning already differs across the five algorithms in
subtle ways that multi-seed aggregation will make worse, not
better. Chapter 20 deferred the audit of those differences to
this chapter. Chapter 23 locked the replicates API shape —
`run_replicates(seeds)`, flat `Vec<RunResult>` with a
`replicate_index` field, filter-by-`(task, algorithm)` as the
aggregation idiom — and left the Ch 24 hook open at Section 1.5:
"`RunResult::best_reward()` helper can be rewritten at the
single-replicate level if Ch 24 decides the per-algorithm
`mean_reward` definition is itself the problem." This chapter
runs the audit, finds that `mean_reward` definition is in fact
the problem, and picks the two remaining shapes the chassis
needs: what each algorithm's per-replicate scalar means
(Decision 1) and what helpers `CompetitionResult` exposes over
the filtered slice (Decision 2).

The picks are:

1. **Per-replicate reduction.** Fix the algorithms. Every
   `EpochMetrics::mean_reward` becomes "mean per-episode total
   reward across `n_envs` trajectories", matching the REINFORCE
   and PPO convention that four of the five algorithms already
   approximately use. CEM drops its length normalization at the
   reporting site but keeps it at the elite-selection site.
   TD3 and SAC switch the denominator from the local
   `epoch_rewards.len()` to the constant `n_envs` and pick up
   partial-episode contributions for envs that did not complete
   within `max_episode_steps`. `RunResult::best_reward()` is
   unchanged — once the inputs are uniform, max over
   `metrics[].mean_reward` is semantically fine. Five train
   loops are touched, `EpochMetrics` gains no new fields, and
   the chassis surface is identical.
2. **Across-replicate aggregation surface.** `CompetitionResult`
   gets two new methods: `replicate_best_rewards(task, algo) ->
   Vec<f64>` as the raw primitive, and `describe(task, algo) ->
   Option<SeedSummary>` as a minimal convenience. `SeedSummary`
   has three fields — `n`, `mean`, `std_dev` — and nothing else.
   No stderr, no min/max, no percentiles. Sample std (Bessel's
   `n-1` denominator), not population std. The raw primitive
   silently filters out `None` replicates from
   `RunResult::best_reward()`; callers that want to detect
   missing replicates compare `.len()` to their seed count at the
   call site.

The chapter ends with a scope-discipline section naming what it
does not decide — most notably, the statistical test, the
replicate count `N`, the rematch pilot design, bootstrap and
percentile helpers, and the rewrite of the D2c test gate. Chapter
32 owns the statistical test and the replicate count; the
execution PR plans in Part 4 own the ml-bridge-side cleanups that
fall out of Decision 1. Chapter 24 picks the chassis shape and
flags the downstream work.

## What Ch 24 inherits from earlier chapters

Ch 20 named the audit: "`best_reward` itself is a per-algorithm
scalar, and chapter 24 will show that its meaning already differs
across algorithms in subtle ways that multi-seed aggregation will
make worse, not better." Ch 20 did not run the audit; it
established the problem and pointed at five train loops as the
place the differences live.

Ch 20 also explicitly deferred the API-shape choice for the
aggregation surface, listing "raw `Vec<f64>` or a lightweight
`SeedSummary { mean, stderr, n }`" as two of the possibilities.
Ch 24's Decision 2 resolves that fork and departs from Ch 20's
placeholder in two ways: it ships *both* the raw `Vec` primitive
and a convenience struct rather than picking one of them, and the
struct's spread field is `std_dev` rather than `stderr`. Section
4.6 argues both departures explicitly.

Ch 23 locked the return shape. `CompetitionResult` stays as
`{ runs: Vec<RunResult> }`; `RunResult` gains `replicate_index:
usize`; per-replicate raw state — full `metrics: Vec<EpochMetrics>`,
full `artifact`, full `best_artifact` — is preserved for every
replicate. Cross-replicate aggregation is a function over
`result.runs.iter().filter(|r| r.task_name == task && r.algorithm_name
== algo)`. Ch 24 is free to define that filter's body in whatever
shape the analysis calls for.

Ch 23 Section 1.5 also distinguished "the helper stays, the
per-replicate input gets fixed" from "the per-replicate input is
fine, the helper gets rewritten to aggregate differently". Ch 24's
Decision 1 picks the first path: the helper at
`sim/L0/ml-bridge/src/competition.rs:45-50` (recon-reported) is
untouched; what changes is the five upstream computations that
feed it.

## Section 1 — The audit: how `mean_reward` is computed, algorithm by algorithm

### 1.1 What the chassis surface looks like today

The single scalar the algorithms report is
`EpochMetrics::mean_reward: f64`, defined at
`sim/L0/ml-bridge/src/algorithm.rs:32-43` (recon-reported). The
struct has five named fields — `epoch`, `mean_reward`,
`done_count`, `total_steps`, `wall_time_ms` — and one
`extra: BTreeMap<String, f64>` for algorithm-specific
diagnostics. There is no per-episode-total variant, no
per-step-mean variant, no unit tag. Whatever the algorithm
decides goes into the single `f64` field, and that is the number
`RunResult::best_reward()` takes a max over.

`RunResult::best_reward()` at
`sim/L0/ml-bridge/src/competition.rs:45-50` (recon-reported) is
twelve lines of code:

```rust
pub fn best_reward(&self) -> Option<f64> {
    self.metrics
        .iter()
        .map(|m| m.mean_reward)
        .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
}
```

The helper is deliberately simple. It assumes whatever each
algorithm put in `mean_reward` is comparable across algorithms.
That assumption is where the audit lives.

### 1.2 The rollout helper — not the culprit

Three of the five algorithms (CEM, REINFORCE, PPO) share a
rollout helper at `sim/L0/ml-bridge/src/rollout.rs:82`
(recon-reported), `collect_episodic_rollout`, which returns an
`EpisodicRollout { trajectories: Vec<Trajectory> }`. Each
`Trajectory` is one env's episode: a `rewards: Vec<f64>` of
per-step rewards, a `done: bool` distinguishing true terminal
states from truncations, and the obs/action streams used for
policy updates. The helper does no reward aggregation. It
returns raw per-step rewards and the caller produces the scalar.

TD3 and SAC inline their own rollout loop instead of using
`collect_episodic_rollout` — they need fine-grained control over
the replay buffer pushes and cannot wait for whole episodes to
complete before storing transitions. But the aggregation story
is the same shape: the inline loop produces per-step data, the
algorithm-side code turns that into a scalar.

This matters for the chapter's argument. The apples-to-oranges
story is not in a shared rollout library that one fix could
correct. It is in five independent choices made in five separate
train loops. Whatever Decision 1 picks, the implementation is
a five-file change, not a one-file change.

### 1.3 CEM — per-step mean, length-normalized

CEM's train loop at
`sim/L0/ml-bridge/src/cem.rs:167-229` (recon-reported) does the
rollout through `collect_episodic_rollout`, then computes
`fitness` at `cem.rs:177-185` (recon-reported):

```rust
let mut fitness: Vec<(usize, f64)> = rollout
    .trajectories
    .iter()
    .enumerate()
    .map(|(i, traj)| {
        let total: f64 = traj.rewards.iter().sum();
        (i, total / traj.len().max(1) as f64)
    })
    .collect();
```

Fitness per trajectory is `total / traj.len().max(1)` — the
per-step mean reward for that episode. CEM then sorts fitness
descending, picks the top `n_elites`, and averages the elite
parameters to form the next generation's mean. The `max(1)`
guard is defensive; in practice `traj.len()` is always at least
one because the rollout helper runs until every env has taken
at least one step.

The reported `mean_reward` at `cem.rs:209` (recon-reported) is:

```rust
let mean_reward: f64 = fitness.iter().map(|(_, f)| f).sum::<f64>() / n_envs as f64;
```

Mean across envs of per-step mean per trajectory. **CEM reports
reward-per-step, length-normalized.** An episode with reward 1
per step and length 500 contributes `fitness = 1.0`, not `500.0`.
A policy that survives twice as long at the same per-step reward
reports the same `mean_reward` as a policy that survives half
as long.

### 1.4 REINFORCE — per-episode total, constant denominator

REINFORCE's train loop at
`sim/L0/ml-bridge/src/reinforce.rs:175-292` (recon-reported) also
goes through `collect_episodic_rollout`. The `mean_reward`
computation is at `reinforce.rs:269-274` (recon-reported):

```rust
let total_reward: f64 = rollout
    .trajectories
    .iter()
    .map(|t| t.rewards.iter().sum::<f64>())
    .sum();
let mean_reward = total_reward / n_envs as f64;
```

`total_reward` is the sum over trajectories of each trajectory's
reward sum. Divided by `n_envs`. **REINFORCE reports
reward-per-episode, with a constant denominator.** No length
normalization. Same 500-step episode with reward 1 per step
contributes `500.0`, not `1.0`.

Two sub-observations worth naming:

A trajectory that is truncated at the rollout helper's
`max_steps` limit contributes its partial sum at full weight to
`total_reward` and still divides by `n_envs`. So a partial
episode counts exactly as much as a complete one in the average.
REINFORCE has no notion of "this trajectory didn't finish,
exclude it".

At `reinforce.rs:214-222` (recon-reported) there is a zero-fallback
short-circuit that emits `EpochMetrics { mean_reward: 0.0, ... }`
when `n_samples == 0`. The short-circuit exists to handle the
degenerate case where no usable samples were collected from the
rollout (e.g., every trajectory has zero steps, which shouldn't
happen in practice but the guard is there). The reported value
is `0.0` — not `None`, not `NaN`. A `max`-based `best_reward()`
on a run where every epoch hit this short-circuit would return
`Some(0.0)` as if zero were a real best. This is a latent bug,
not a Decision 1 concern, but Section 3.7 flags it for the
execution PR plans.

### 1.5 PPO — identical to REINFORCE

PPO's train loop at
`sim/L0/ml-bridge/src/ppo.rs:232-458` (recon-reported) has
essentially the same aggregation. The `mean_reward` computation
at `ppo.rs:423-428` (recon-reported) is:

```rust
let total_reward: f64 = rollout
    .trajectories
    .iter()
    .map(|t| t.rewards.iter().sum::<f64>())
    .sum();
let mean_reward = total_reward / n_envs as f64;
```

Line-for-line the same five lines of arithmetic as REINFORCE.
Same unit, same constant denominator, same partial-episode
behavior. Same zero-fallback short-circuit at
`ppo.rs:314-322` (recon-reported).

The literal duplication between REINFORCE and PPO is worth
naming as its own finding. It is evidence that the chassis has
a shared-rollout-aggregation gap: the rollout helper computes
raw per-step rewards and every on-policy caller has to
independently write the same five lines to turn them into a
scalar. If Decision 1 were "add a shared aggregation helper and
have every algorithm call it", PPO and REINFORCE would share
the helper trivially. That is not the shape Decision 1 picks
(see Section 3.2), but the duplication is a hint that the
chassis could grow one in a future iteration.

### 1.6 TD3 — mean over first-completed episodes, variable denominator

TD3's train loop inlines its own rollout because it needs to
push per-step transitions to the replay buffer as they happen
rather than waiting for episode boundaries. The outer structure
at `sim/L0/ml-bridge/src/td3.rs:259-284` (recon-reported) is:

```rust
for epoch in 0..n_epochs {
    let t0 = Instant::now();
    let mut epoch_rewards: Vec<f64> = Vec::new();
    let mut epoch_done_count = 0_usize;
    // ...
    let mut env_episode_reward = vec![0.0_f64; n_envs];
    let mut env_complete = vec![false; n_envs];

    for _step in 0..hp.max_episode_steps {
        if env_complete.iter().all(|&c| c) {
            break;
        }
        // ... step all envs, push transitions, run gradient updates
    }
```

The inner loop runs `hp.max_episode_steps` iterations or
breaks early when every env has completed at least one episode,
whichever comes first. At each inner step, every env takes an
action regardless of whether it has already completed an
episode this epoch — that is the Ch 22 "off-policy steps all
`n_envs` envs unconditionally" finding, and for TD3 specifically
it matters here because completed envs continue to contribute
transitions to the replay buffer but not to `epoch_rewards`.

Reward accumulation and episode-boundary detection happen inside
the inner loop at `td3.rs:335-351` (recon-reported):

```rust
env_episode_reward[i] += reward;

if (done || truncated) && !env_complete[i] {
    env_complete[i] = true;
    epoch_rewards.push(env_episode_reward[i]);
    if done {
        epoch_done_count += 1;
    }
}
// ... auto-reset on done/truncated, zero env_episode_reward[i]
```

Each env pushes **exactly one** episode total to `epoch_rewards`
— the reward it accumulated up to the first `done` or `truncated`
signal of the epoch. Envs that complete a second, third, or
fourth episode before the inner loop exits contribute those
episodes' transitions to the replay buffer (so the gradient
updates see them) but their rewards never reach `epoch_rewards`.

The `mean_reward` computation at `td3.rs:487-491` (recon-reported)
is:

```rust
let mean_reward = if epoch_rewards.is_empty() {
    0.0
} else {
    epoch_rewards.iter().sum::<f64>() / epoch_rewards.len() as f64
};
```

**TD3 reports reward-per-episode, with a variable denominator.**
In the common case where every env completes its first episode
within `max_episode_steps`, `epoch_rewards.len() == n_envs` and
the scalar matches REINFORCE's shape exactly. In the edge case
where some env never completes its first episode within
`max_episode_steps`, that env contributes nothing to
`epoch_rewards`, and the denominator shrinks. REINFORCE would
divide by `n_envs` and include that env's partial trajectory at
full weight; TD3 divides by a smaller number and omits the
incomplete env entirely.

There is also a mild asymmetry between what the metric reports
and what training actually uses. The replay buffer sees every
transition from every env (including the second, third, fourth
episodes after first-completion); the `mean_reward` scalar sees
only first-episode totals. A reader looking at `mean_reward`
would conclude the policy's performance is based on the same
work TD3's gradient updates trained on, but it is actually a
proper subset of that work. This asymmetry is out of scope for
Decision 1, which is strictly about unit comparability, but it
is worth naming as a separate issue the execution PR plans may
want to revisit.

### 1.7 SAC — identical pattern to TD3

SAC's train loop at
`sim/L0/ml-bridge/src/sac.rs:286-572` (recon-reported) has the
same inline rollout structure as TD3. `env_complete` gating at
`sac.rs:355-363` (recon-reported), same first-completed-episode
push semantics. `mean_reward` computation at
`sac.rs:540-544` (recon-reported):

```rust
let mean_reward = if epoch_rewards.is_empty() {
    0.0
} else {
    epoch_rewards.iter().sum::<f64>() / epoch_rewards.len() as f64
};
```

Identical to TD3 line-for-line. Same unit, same variable
denominator, same first-completed-episode-per-env semantics,
same asymmetry between reporting and training work.

### 1.8 Refining the characterization

Previous recon framed the audit as a three-way split: per-step
mean (CEM) versus per-episode total (REINFORCE, PPO) versus
mean-over-completed-episodes (TD3, SAC). Line-level reading
shows that framing is close but imprecise. The source is better
described as a **two-way split with a denominator wrinkle**:

1. **CEM reports reward-per-step.** One algorithm, length-normalized
   via the `/ traj.len().max(1)` at `cem.rs:183` (recon-reported).
2. **REINFORCE, PPO, TD3, SAC report reward-per-episode.** Four
   algorithms, length-unnormalized. They differ only in the
   denominator: REINFORCE and PPO divide by the constant
   `n_envs`; TD3 and SAC divide by the variable
   `epoch_rewards.len()` which equals `n_envs` in the common
   case and shrinks in an edge case.

The three-way framing is not wrong in spirit — the denominator
wrinkle is a real second-order difference — but the correct
framing matters for what Decision 1 has to fix. Fixing "CEM's
length normalization and the TD3/SAC denominator" is two small
targeted changes; fixing "four algorithms that fundamentally
disagree about how to count" would be a much larger refactor.
Four of the five algorithms already agree on the primary unit,
and the fix is smaller than the three-way framing made it sound.

In the complete-episode case — every env finishes within
`max_episode_steps` — REINFORCE's
`trajectory.rewards.iter().sum()` and TD3's `env_episode_reward[i]`
accumulator produce identical per-trajectory numbers. The two
loops disagree on what to do with incomplete envs and on how
to drive the denominator, not on what a complete episode's
total reward is. The audit confirmed this by reading both
accumulators; the "two-way split" characterization depends on
the agreement and the chapter names it here so a reader does
not have to reconstruct the check from the prose alone.

### 1.9 What this means in practice

Pick a task with per-step reward on the order of 1 and typical
episode length on the order of 500. A well-trained policy that
holds reward near 1 per step for the full episode reports:

| Algorithm | `mean_reward` value | Unit |
|---|---|---|
| CEM | ≈ 1.0 | reward-per-step |
| REINFORCE | ≈ 500.0 | reward-per-episode |
| PPO | ≈ 500.0 | reward-per-episode |
| TD3 | ≈ 500.0 | reward-per-episode |
| SAC | ≈ 500.0 | reward-per-episode |

The CEM-versus-rest gap is ~2 to 3 orders of magnitude by
design. The D2c rematch test at
`sim/L0/thermostat/tests/d2c_cem_training.rs` (recon-reported,
exact line range deferred to factual pass) compares
`best_reward()` values across algorithms and passes or fails
the test based on those comparisons. Whatever that test was
measuring, it was not what the name suggested it was measuring.

The gap is not a fixed scaling factor. It depends on episode
length. Episode length varies across tasks (different
`max_episode_steps` budgets), across seeds (stochastic envs
truncate at different points), and across training progress
(a policy that learns to survive longer produces longer
trajectories). There is no post-hoc conversion that can be
applied at the reporting layer to fix what the algorithm-side
computation already did. The fix has to live inside the train
loops.

## Section 2 — How this touches Ch 23 and the D2c story

### 2.1 Not a Ch 23 retroactive patch

The Ch 21 post-commit patch from Ch 22 findings (commit
`3e1ec0ff`) established the pattern for how downstream findings
amend upstream chapters: when a downstream chapter discovers
that an upstream claim is factually wrong, the upstream chapter
gets a narrow amendment and a forward-reference paragraph
rather than a full re-review. That precedent applies when the
downstream finding makes the upstream claim factually wrong.

None of Ch 23's claims become factually wrong in light of this
audit. Ch 23 locked three picks:

- **`run_replicates(seeds)` + flat `Vec<RunResult>`**. This is a
  mechanical-shape pick about how multiple replicates are
  sequenced and stored. Nothing about it depends on what
  `mean_reward` means. The flat vector stores whatever the
  algorithm produced; the per-replicate semantics Decision 1
  changes are opaque to the return shape.
- **PPO exclusion from the rematch pool.** Ch 23 excluded PPO
  on D1d-style exploration-noise-inflation grounds — a specific
  failure mode in which a policy class large enough to
  represent good solutions combines with action noise large
  enough to make training unstable, producing a "sometimes
  wanders into the solution and the best-epoch metric caches
  it" trajectory that Ch 30 identified as D1d's mode. Ch 24's
  unit-mismatch finding is a *separate* observation about PPO;
  it does not invalidate or supersede the D1d argument.
- **Matched complexity = `LinearPolicy(2, 1)`, `n_params = 3`.**
  A parameterization-count pick. Independent of reward
  semantics.

Ch 23 stands. This chapter adds context.

### 2.2 The supporting observation for PPO (and TD3 and SAC)

Ch 24's finding adds a corroborating observation on a separate
mechanism axis. Ch 23's D1d argument concerned what PPO *learned*
— exploration noise dominating the fitness gradient. Ch 24's
observation concerns what the metric *measured* — the unit the
number was stored under. Both conclusions point in the same
direction and neither changes Ch 23's pool-composition call; the
point is that there are two independent mechanisms by which the
D2c PPO number fails to mean what its name suggests, not that
Ch 24 has found a new reason to exclude PPO that Ch 23 missed.
PPO's D2c `best_reward` was in reward-per-episode units. CEM's
D2c `best_reward` was in reward-per-step units. The D2c rematch
test compared them
directly. The number PPO was scoring "against" was off by
roughly two orders of magnitude in units alone, before any
question of whether PPO had actually learned anything.

The same observation applies to TD3 and SAC — they were also
comparing reward-per-episode numbers to CEM's reward-per-step
number in D2c. Ch 30 excluded TD3 and SAC on linear-$Q$
expressiveness grounds; Ch 24 adds the unit-mismatch
observation as additional corroboration. The D2c test was
apples-to-oranges against all three off-policy algorithms at
once.

This does not change Ch 23's pool composition. The rematch pool
is still `{CEM, SA}`, PPO is still out, TD3 and SAC are still
out. What changes is readers' confidence in the D2c test's
original verdict: the test was measuring something, but it was
not cleanly measuring what its name claimed it was measuring,
and the unit mismatch is a separate and independent reason to
rewrite it.

### 2.3 Scope discipline for Section 2

Chapter 24 does not rewrite Chapter 23. Chapter 24 does not
relitigate the D2c result. The D2c test gate at
`sim/L0/thermostat/tests/d2c_cem_training.rs` is broken on unit
grounds, but rewriting the test is an execution-layer concern —
it lives in a test file that belongs to ml-bridge's test suite,
not in the chassis, and the rewrite is a Chapter 41 responsibility
once the chassis-side Decision 1 fix lands. Whether the rewritten
gate clears or rejects CEM in the rematch is a Chapter 32
question (statistical test, replicate count) that depends on a
pilot run that does not yet exist.

## Section 3 — Decision 1: per-replicate reduction

### 3.1 The three options

Three reconciliation paths are available to Decision 1:

- **(a) Fix the algorithms.** Breaking change at the computation
  sites. Each of the five train loops has its `mean_reward`
  computation rewritten to produce a consistent unit.
  `EpochMetrics::mean_reward` keeps its single-`f64` shape but
  its meaning becomes uniform across algorithms.
- **(b) Add a parallel unified metric.** Add a new field to
  `EpochMetrics`, say `mean_reward_per_episode: f64`, that every
  algorithm populates in a consistent way. The existing
  `mean_reward` keeps its per-algorithm meaning. New code that
  cares about cross-algorithm comparability uses the new field;
  old code that is fine with per-algorithm semantics keeps using
  `mean_reward`.
- **(c) Accept the split and document it.** No code change. Add
  a doc comment on `EpochMetrics::mean_reward` explaining that
  the unit is algorithm-specific and callers must reconcile
  before comparing. `RunResult::best_reward()` gets a doc
  warning that it is not comparable across algorithms. The
  rematch gate reconciles at its call site; every other caller
  reconciles at theirs.

### 3.2 The pick: (a) fix the algorithms

Standardize on "mean per-episode total reward across `n_envs`
trajectories", the REINFORCE/PPO convention. Concretely:

- **CEM.** At `cem.rs:209` (recon-reported), the reported
  `mean_reward` becomes the sum of per-trajectory totals divided
  by `n_envs`, matching REINFORCE's expression. The `fitness`
  computation at `cem.rs:183` (recon-reported), used for elite
  selection, stays length-normalized — see Section 3.5 for the
  internal-split rationale. The reporting site drops the
  length normalization; the selection site keeps it.
- **REINFORCE.** Already correct. No change at
  `reinforce.rs:274` (recon-reported).
- **PPO.** Already correct. No change at `ppo.rs:428`
  (recon-reported).
- **TD3.** At `td3.rs:487-491` (recon-reported), the denominator
  switches from `epoch_rewards.len()` to `n_envs`, and a small
  loop before the computation pushes any remaining
  `env_episode_reward[i]` for incomplete envs into
  `epoch_rewards` so the partial-trajectory reward gets counted.
  See Section 3.6 for the incomplete-env handling.
- **SAC.** Same shape as TD3 at `sac.rs:540-544` (recon-reported).

`RunResult::best_reward()` at `competition.rs:45-50`
(recon-reported) stays as-is. Once the five inputs are
consistent, taking a max over `metrics[].mean_reward` is
semantically fine — all five values are in the same units, and
the max picks the highest-scoring epoch.

Three train loops are touched — CEM, TD3, and SAC. REINFORCE and
PPO are already correct and need no edits. `EpochMetrics` gains
no new fields. No new types on `RunResult` or `CompetitionResult`.
The chassis surface is unchanged; what changes is the
implementation behind the existing surface. This is a "fix the
bug" change, not a "grow the API" change.

### 3.3 Counterfactual: why not (b) — parallel field

Option (b) is attractive because it sounds non-breaking. Nothing
already-written stops working; new code opts into the new field.
The D2c test at `d2c_cem_training.rs` could be migrated to the
new field on its own schedule; everything else continues to
compile and run as before.

The argument against (b) is that it trades a clean break for a
lingering one:

- Every algorithm still has to be modified to populate the new
  field. The five-file change happens either way.
- The old `mean_reward` field still exists, still reports its
  per-algorithm value, and every caller has to decide which
  field to trust. "Both exist, use the new one" is a style
  convention, not a type-enforced invariant. The bug recurs the
  first time a new competitor test or a well-meaning cleanup
  reaches for the old field.
- `RunResult::best_reward()` becomes a "which field does it use?"
  trap. Changing it to use the new field is also a breaking
  change in disguise — every test that asserted on
  `best_reward()` values before the change is now asserting on
  different numbers. The "non-breaking" claim of (b) turns out
  to be "non-breaking at the compile layer, silently breaking at
  the test-assertion layer".
Option (b) tries to be both things at once and ends up being
neither. Rejected.

### 3.4 Counterfactual: why not (c) — document and punt

Option (c) is Chapter 20's failure mode restated as a feature.
Chapter 20's whole argument was that single-seed results are
broken and that the chassis needs to take the scalar-shape
problem seriously at the type level, not at the documentation
level. If Chapter 24's answer is "yes but only at the
documentation level", Chapter 20's finding is not solved, it is
paraphrased into a doc comment.

The execution cost of (a) is small: roughly ten line-level
changes across three files (CEM, TD3, SAC), plus a three-line
pre-loop in each of TD3 and SAC, plus updated doc comments. Call
it twenty lines total. The execution cost of (c) is one well-written doc
block and a permanent interpretation tax on every caller of the
chassis — every future test gate has to decide what the reward
it is comparing actually means, every future reader of the D2c
test has to learn the unit rules before trusting the number,
every future refactor has to re-learn that `mean_reward` is a
misleading name and be careful not to use it cross-algorithm.

For a permanent chassis-level API, (c) is the worst of the three.
The documentation obligation is real forever; the code obligation
is one PR. Rejected.

### 3.5 Sub-decision: CEM's internal elite-selection fitness

CEM uses `fitness` for elite selection at `cem.rs:177-189`
(recon-reported), and the length-normalization is baked into
the fitness expression. Decision 1 picks "leave elite-selection
fitness length-normalized; change only the `mean_reward`
reporting". Two paths of reasoning converge on this call.

The first path: for survival-oriented tasks where the agent's
per-step reward is approximately constant as long as it stays
alive, a longer trajectory at the same per-step reward is the
same thing as a higher total reward. In that regime, per-step
mean and per-episode total agree up to a monotone rescaling of
the selection ordering; it does not matter which CEM uses, and
both produce the same elite set.

The second path: Chapter 24's scope is reporting comparability
across algorithms, not the internal workings of any single
algorithm's search. The reporting standardization forces CEM to
report per-episode total so cross-algorithm comparisons are
meaningful. The selection criterion is a separate CEM-internal
design question — per-step mean and per-episode total apply
different selection pressures to elite candidates, and which
pressure CEM should use is a call about what CEM-in-D2c-era was
tuned to do, not a call that a reporting-bug fix has any
principled basis for overriding. Chapter 24 declines to override
CEM's selection on scope grounds, not on "we thought about it and
the current choice is better" grounds. The two are different
defenses and the chapter owes the reader the honest one.

The honest cost: CEM will carry two reward concepts internally —
`fitness` (per-step mean, used for selection) and `mean_reward`
(per-episode total, used for reporting). This is a real cost,
not a free choice. A reader of the CEM train loop after Decision
1 lands will see two arithmetic expressions computing reward
summaries from the same `rollout.trajectories` via different
normalizations and will reasonably ask why. The answer the
chapter commits to is: "Ch 24 standardized reporting without
touching selection because selection is a separate design
concern the reporting fix has no mandate to override." The
execution PR for Decision 1 should add a doc comment on the
CEM train loop naming the internal split and its rationale
explicitly, so that readers do not have to reconstruct the
reasoning from the code alone. The `extra` BTreeMap on
`EpochMetrics` already carries an `"elite_mean_reward"` key at
`cem.rs:219` (recon-reported) that reports the per-step-mean
elite number; that key's semantics stay unchanged and provide
the per-step signal for callers who want it.

An alternative Decision 1.5 fork — making CEM's elite selection
also switch to per-episode total — would eliminate the internal
split at the cost of altering CEM's D2c-era search behavior in
a way Chapter 24 cannot predict without re-running D2c CEM.
Chapter 24's scope is "fix the reporting apples-to-oranges".
Whether CEM's selection pressure should be reconciled with the
reporting change is a separate design question, and one Chapter
24 has no mandate to answer. Out of scope.

### 3.6 Sub-decision: the TD3/SAC denominator edge case

Switching `td3.rs:490` and `sac.rs:543` (recon-reported) from
`epoch_rewards.len()` to `n_envs` is the mechanical part of the
TD3/SAC fix. The semantic question is what an env that did not
complete its first episode within `max_episode_steps` contributes
to the numerator.

Two options:

- **Contribute 0.** An incomplete env contributes nothing to the
  numerator. The sum is then divided by `n_envs`. The incomplete
  env's partial progress vanishes. Pessimistic bias: a policy
  that got halfway to a good outcome and was cut short looks the
  same as a policy that did nothing.
- **Contribute `env_episode_reward[i]` (the partial).** An
  incomplete env contributes whatever reward it accumulated before
  the cutoff. Then the sum is divided by `n_envs`. Consistent
  with REINFORCE's treatment of truncated trajectories, which
  also contribute their partial sums at full weight to
  `total_reward` and still divide by `n_envs`.

The pick: **contribute the partial.** The reasoning is that
Decision 1 standardizes on REINFORCE's convention, and
REINFORCE includes partial trajectories at full weight in its
numerator. Standardizing on REINFORCE's convention means
standardizing on how REINFORCE handles truncated trajectories
too, not just how it handles complete ones. Contributing 0
would create a third convention — neither REINFORCE's
full-weight inclusion nor TD3's current len-of-completed
exclusion — and the whole point of Decision 1 is to remove
conventions, not add new ones. Consistency with REINFORCE's
numerator is the reason, not consistency for its own sake.

A third option worth naming and rejecting: *extend the inner
loop until every env completes its first episode, ignoring
`max_episode_steps`*. This would remove the incomplete-env edge
case entirely — no env would ever contribute a partial, because
every env would always complete — at the cost of breaking the
fixed compute budget Chapter 22's fair-compute-parity argument
depends on. An epoch whose slowest env stalls near
`max_episode_steps - 1` before completing would consume
substantially more env steps than `n_envs * max_episode_steps`,
and the budget-enforcement mechanism from Chapter 21 would lose
its guarantee. Rejected on Chapter 22 grounds.

The TD3/SAC fix gets a small loop before the `mean_reward`
computation:

```rust
// Include any envs that did not complete within max_episode_steps.
for i in 0..n_envs {
    if !env_complete[i] {
        epoch_rewards.push(env_episode_reward[i]);
    }
}
let mean_reward = epoch_rewards.iter().sum::<f64>() / n_envs as f64;
```

Three lines before the existing computation, plus the
denominator swap. `epoch_rewards.len()` is no longer meaningful;
the computation always divides by `n_envs`. The old `is_empty()`
guard at `td3.rs:487` (recon-reported) becomes unnecessary —
with incomplete envs pushed before the division, `epoch_rewards`
has exactly `n_envs` entries after the loop, and `n_envs > 0`
is an invariant of `VecEnv`.

A second-order note: the `done_count` metric at `td3.rs:489`
(recon-reported) counts only first-completed episodes that
ended in a true terminal state. This chapter does not change
`done_count` — it stays as-is under the "first-completed
episode per env only" semantics because `done_count` is a
different question than `mean_reward` (how many episodes ended
naturally, rather than what the average reward was). The
asymmetry is worth naming in the execution PR plan's
documentation commits.

### 3.7 Minor flag: the REINFORCE/PPO zero-fallback

At `reinforce.rs:214-222` and `ppo.rs:314-322` (recon-reported),
both algorithms emit `EpochMetrics { mean_reward: 0.0, ... }`
when `n_samples == 0`. This is a "no samples collected"
short-circuit, not "reward is actually zero", but it reports a
value that a max-based `best_reward()` would pick up as a real
best.

This is a latent defect in the ml-bridge surface, not a
Decision 1 question. Decision 1 is about the *meaning* of
`mean_reward` when there are samples; the zero-fallback is
about what to emit when there are not. The cleanest fix is to
skip the metric entirely (don't push an
`EpochMetrics` at all for short-circuited epochs) or to emit
`Option<f64>`, but both touch the `EpochMetrics` type or the
train-loop control flow in ways that go beyond Chapter 24's
scope.

Flagged for Chapter 41 (PR 2 execution plan). Chapter 41 decides
whether to skip, emit `None`, emit `NaN`, or leave as-is; the
decision does not affect Chapter 24's reduction semantics.

## Section 4 — Decision 2: across-replicate aggregation surface

### 4.1 What Ch 23 inherited Ch 24 with

Chapter 23 committed to `CompetitionResult { runs: Vec<RunResult> }`
with a `replicate_index: usize` field on each `RunResult`.
Aggregation is a function over
`result.runs.iter().filter(|r| r.task_name == task && r.algorithm_name == algo)`.
Chapter 24's job is to pick the shape of helpers that consume
this filter — what methods `CompetitionResult` exposes, what
types they return, and what commitments those types make about
how the rematch gate will be expressed.

Chapter 23 Section 1.5 specifically left the Chapter 24 hook
as: "`RunResult::best_reward()` helper can be rewritten at the
single-replicate level if Ch 24 decides the per-algorithm
`mean_reward` definition is itself the problem, in which case
the across-replicate layer inherits the fix for free."

Decision 1 answered the first half of that hook: yes,
`mean_reward` is the problem, and the fix lives inside the
train loops, not inside `RunResult::best_reward()`. The
`best_reward()` helper stays untouched. The single-replicate
primitive Chapter 24 builds on is therefore the existing
`RunResult::best_reward() -> Option<f64>`, with its inputs now
uniform across algorithms thanks to Decision 1. Decision 2
picks what `CompetitionResult` wraps around that primitive for
the multi-replicate case.

### 4.2 The scope spectrum

Three honest options at different levels of commitment:

- **S (smaller): raw `Vec` only.** One method on
  `CompetitionResult`:
  `replicate_best_rewards(task, algo) -> Vec<f64>`. No
  convenience struct, no mean/std accessors. Callers compute
  whatever summary they want. Chapter 32 picks the statistical
  test and adds whatever helpers the test needs.
- **M (medium): primitive + minimal summary struct.** Same
  primitive as S, plus
  `describe(task, algo) -> Option<SeedSummary>` with
  `SeedSummary { n, mean, std_dev }`. Three fields, no stderr,
  no min/max, no percentiles.
- **L (larger): M plus percentiles.** Adds median/IQR fields or
  a percentile accessor to support non-Gaussian seed
  distributions.

### 4.3 The pick: M

Concrete API:

```rust
impl CompetitionResult {
    /// Per-replicate `best_reward` values for a (task, algorithm) pair,
    /// after filtering out `None` replicates (see note below).
    ///
    /// Callers that want to detect missing replicates should compare
    /// the returned vector's length to their seed count at the call site.
    pub fn replicate_best_rewards(&self, task: &str, algo: &str) -> Vec<f64>;

    /// Lightweight summary of the per-replicate best-reward distribution.
    ///
    /// Returns `None` if no replicate produced a finite best reward for
    /// this (task, algorithm) pair. The summary's `n` field is the count
    /// of non-`None` replicates, not the count of seeds originally passed
    /// to `run_replicates`.
    pub fn describe(&self, task: &str, algo: &str) -> Option<SeedSummary>;
}

/// Lightweight summary of a seed-replicate distribution.
pub struct SeedSummary {
    /// Count of non-`None` replicates. May be less than the seed count
    /// passed to `run_replicates` if any replicate had zero usable epochs.
    pub n: usize,
    /// Sample mean.
    pub mean: f64,
    /// Sample standard deviation with Bessel's correction (n-1 denominator).
    /// Zero when `n == 1`; callers that need to guard on `n >= 2` should
    /// check `self.n` directly.
    pub std_dev: f64,
}

impl SeedSummary {
    /// Build a summary from a slice of rewards. Returns `None` if the
    /// slice is empty.
    pub fn from_rewards(rewards: &[f64]) -> Option<SeedSummary>;
}
```

Three fields on the struct. Sample std with Bessel's
correction. Silent filter-out of `None` replicates at the raw
primitive. No stderr, no min/max, no percentiles.

### 4.4 Counterfactual: why not S (raw `Vec` only)

Option S is cleaner in principle. It commits nothing, lets
Chapter 32 build exactly the helpers its chosen statistical
test needs, and avoids endorsing any particular reporting
convention. It also matches the "less chassis, cleaner
semantics" flavor Decision 1 is already picking — five train
loops get smaller, and the aggregation layer stays minimal.

The argument against S is that the chassis is not consumed only
by the rematch. Chapters 40 through 42 are the execution PR
plans, and two of them — PR 1 (chassis reproducibility) and PR 2
(Competition replicates + algorithm surface fixes) — will land
*before* Chapter 32 finalizes its statistical test. If Chapter 24
ships only the raw primitive, the execution PR authors will
invent their own conventions at the first call site that wants
to print "mean ± something" to the console. They will
re-implement mean/std computation at every call site. They will
name the summary structs differently each time. They may use
population std in one place and sample std in another without
realizing the convention matters. That is the same
punt-to-callers failure mode Decision 1 rejected, applied to
summary statistics instead of reward units.

A minimal struct with three fields gives the execution PR
authors a shared vocabulary without forcing them into a
specific statistical test. The struct is a convenience, not a
mandate: callers who want bootstrap confidence intervals or
median/IQR go straight to `replicate_best_rewards` and compute
whatever they need. Callers who want "the normal print-this-to-
console summary" call `describe` and get it. The struct is
doing the work of a vocabulary, not the work of an answer.

Rejected. Option M is the right balance.

### 4.5 Counterfactual: why not L (add percentiles)

Option L addresses a real concern. Seed distributions across
replicates are sometimes non-Gaussian — in particular, when a
fraction of seeds collapse to a local optimum and the rest find
a better one, the distribution becomes bimodal and mean/std
summarizes it badly. The mean lands between the two modes, the
std looks enormous, and both summary numbers misrepresent the
actual population. Median and IQR are robust to this kind of
structure; mean/std is not.

But L speculates about a problem we do not have evidence we
have yet. The rematch pilot does not exist, the seed
distribution has not been characterized, and shipping
percentile accessors "just in case" is the shape of chassis
overbuild Chapter 22 argued against. If the pilot shows the
distribution is bimodal, percentile accessors can be added in a
one-PR change — the underlying data is already in the raw `Vec`
primitive, so no new storage or collection logic is needed,
only new read helpers.

The better move: ship M now, let the rematch pilot (Chapter 32)
measure whether the distribution is bimodal, and add percentiles
if and only if the data demands it. Rejected.

### 4.6 Sub-decision: sample std vs population std

`SeedSummary::std_dev` uses Bessel's correction (`n-1`
denominator), matching the sample-std convention almost every
RL paper uses when reporting "mean ± std across seeds". The
population std (`n` denominator) has a cleaner
maximum-likelihood interpretation for distributions whose
parameters you already know, but that is not the situation the
rematch is in: the rematch is estimating the spread of a
seed-level population from a small sample, and Bessel's
correction is the standard conservative estimator for that.

With `n` small (the rematch will likely use 3 to 10 seeds), the
absolute gap between sample and population std is meaningful in
absolute terms. The argument for Bessel's correction is
convention-matching, not test-specific: picking the population
std would surprise every reader who knows the RL literature,
and since Chapter 24 is deliberately not picking the statistical
test (Chapter 32 owns that), the chassis should not argue for a
denominator on grounds that implicitly assume one test over
another. Convention-matching is a cleaner reason and does not
depend on a test Chapter 32 has not picked yet.

`n == 1` is a degenerate case for sample std — the `n-1`
division is division by zero. `from_rewards` handles this by
returning `std_dev = 0.0` with a doc note that `n == 1` carries
no spread information. Callers that want to guard on `n >= 2`
check `.n` directly. The alternative — returning `None` from
`from_rewards` on `n == 1` — would make "I have one replicate
and I want its mean" unnecessarily awkward, and the rematch
will never run with a single seed anyway, so this is a
boundary-condition choice, not a load-bearing one.

### 4.7 Sub-decision: `Vec<f64>` vs `Vec<Option<f64>>` for the primitive

`RunResult::best_reward()` returns `Option<f64>` — `None` if the
run had zero epochs, which can happen under
`TrainingBudget::Steps(0)` or under a hypothetical future
algorithm whose train loop exits before producing any metrics
or under the REINFORCE/PPO zero-fallback case Section 3.7
flagged. The primitive `replicate_best_rewards` has two honest
choices:

- **`Vec<f64>` with silent filter-out.** Every caller gets a
  clean vector of values. The `None` case is erased from the
  primitive's output.
- **`Vec<Option<f64>>`.** Every caller sees the `None`s and
  decides. More honest, but every call site has to
  `.filter_map(|x| x)` or `.flatten()` before doing anything
  real.

The pick: `Vec<f64>`. The `None` case is degenerate — in
practice, zero-epoch runs happen only because of bugs or
misconfiguration, not because a run "just didn't learn".
Forcing every caller to handle it as a routine branch is noise
that obscures the common-case code. Callers that need to detect
missing replicates compare `replicate_best_rewards(task,
algo).len()` to their seed count at the call site — a one-line
check that is available when needed and absent when not.

The `n` field on `SeedSummary` is documented as "count of
non-`None` replicates, not count of seeds originally passed to
`run_replicates`". A reader who misses the distinction could
draw the wrong conclusion about how many seeds the summary is
based on, so the doc comment names the semantics explicitly.

This choice is intentionally asymmetric with the
parts-per-million precision Decision 1 is demanding of
per-algorithm reward semantics. The reason is that Decision 1's
apples-to-oranges problem is a routine path — every comparison
between CEM and any other algorithm hits it, every time — and
the fix has to be at the type-enforced level because the routine
path cannot be documented around. The `None` case here is a
boundary-condition bug that should be fixed at its source
(Section 3.7 flags the REINFORCE/PPO zero-fallback as one such
source), not a routine path that every caller has to reason
about.

One contingency worth naming explicitly: Decision 2's silent
filter-out is contingent on Chapter 41 fixing the REINFORCE/PPO
zero-fallback source that Section 3.7 flagged. If Chapter 41
leaves the zero-fallback emitting `mean_reward: 0.0` on
short-circuited epochs, `replicate_best_rewards` will silently
pass those zeros through as real data points *and* silently
filter out true zero-epoch runs that returned `None` — the
worst of both worlds, a Vec that conflates "real zero" with
"degenerate short-circuit" and drops neither. The Decision 2
pick rests on the assumption that Chapter 41 will emit `None`
or skip the metric entirely on short-circuited epochs, making
the filter-out well-defined. If that assumption breaks in
execution, Decision 2's return type needs to be revisited.

### 4.8 What the execution layer gets

The PR 2 plan (Chapter 41, Competition replicates and algorithm
surface fixes) inherits from this chapter:

- `run_replicates(seeds)` API from Chapter 23
- `replicate_best_rewards`, `describe`, and `SeedSummary` from
  this chapter
- Rewritten per-algorithm `mean_reward` computation from
  Decision 1, touching `cem.rs`, `td3.rs`, and `sac.rs`
  (REINFORCE and PPO are no-ops)
- The REINFORCE/PPO zero-fallback cleanup flag from Section 3.7
  as a follow-up in the same PR or a sibling

The PR 3 plan (Chapter 42, sim-opt split and rematch) uses
these to express the rematch's statistical gate in whatever
shape Chapter 32 decides. For a mean-based gate,
`describe(task, algo).map(|s| s.mean)` plus a `std_dev`-based
width gives the upper and lower bounds. For a bootstrap
confidence interval, pull the raw vector from
`replicate_best_rewards` and resample. For median and IQR, pull
the raw vector and compute percentiles directly. All three
test shapes are one or two lines off the Chapter 24 API; none
require extending `CompetitionResult` further.

## Section 5 — What Chapter 24 does not decide

Chapter 24 picks the shape of the per-replicate reduction and
the across-replicate aggregation surface. It deliberately
declines the following:

- **The statistical test.** Whether the rematch gate is
  mean-plus-one-std, Welch's t, a bootstrap confidence
  interval, Mann-Whitney U, or something else is a Chapter 32
  decision. Chapter 24 gives Chapter 32 a shape that supports
  all of the above.
- **The replicate count `N`.** Chapter 32 owns this. It depends
  on measured variance from a pilot that does not yet exist.
- **The pilot design itself.** How many seeds the pilot uses,
  which task it pilots on, whether the pilot uses CEM or SA or
  both — all Chapter 32 calls.
- **Percentile and bootstrap helpers on `CompetitionResult`.**
  Section 4.5 rejected these as speculative for this chapter.
  They can be added later in Chapter 32 if the data demands
  them.
- **Whether the D2c test gate at
  `sim/L0/thermostat/tests/d2c_cem_training.rs` needs to be
  rewritten.** It does — the test is unit-broken against CEM
  versus any of the other four algorithms — but rewriting the
  test is a Chapter 41 execution-layer concern. The test lives
  in ml-bridge's test suite, not in the chassis, and Chapter 41
  owns ml-bridge's surface fixes.
- **Whether CEM's internal elite-selection fitness should
  eventually be reconciled with the reporting change.**
  Section 3.5 argued to leave it alone for now. A future
  chapter or a future iteration of CEM itself may revisit the
  call if the internal split starts confusing readers of the
  CEM train loop.
- **Whether CEM's `extra.elite_mean_reward` diagnostics key
  needs renaming or a doc update.** With Decision 1 in effect,
  CEM's `mean_reward` is in per-episode-total units and the
  `elite_mean_reward` extras key is in per-step units — the
  naming collision is mild but real, and a reader of the
  `EpochMetrics.extra` map will reasonably ask what unit the
  elite key is in. The execution PR for Decision 1 should at
  least update the doc comment explaining both keys side by
  side; whether to rename the key outright is a Chapter 41
  cleanup call.
- **The REINFORCE/PPO zero-fallback cleanup.** Section 3.7
  flagged this as a latent defect in ml-bridge. It is handled
  in Chapter 41's execution plan, not in Chapter 24's scope.
- **The concrete rustdoc text for the rewritten
  `EpochMetrics::mean_reward`.** Chapter 41 writes the rustdoc
  comment alongside the train-loop rewrites as part of the
  execution PR. Chapter 24 names the new semantic in prose; the
  code-level documentation is Chapter 41's job.
- **Whether `RunResult::final_reward()` at
  `competition.rs:36-41` has the same semantic issue.** It does
  — `final_reward()` also pulls from
  `metrics.last().mean_reward`, so once Decision 1 lands the
  helper is fixed for free along with `best_reward()`. No
  separate Chapter 24 pick is needed.
- **The reporting-vs-training asymmetry in TD3/SAC** (Section
  1.6 named this as a separate issue: the replay buffer sees
  every episode including post-completion ones, the
  `mean_reward` scalar sees only first-completed episodes).
  This is a documentation concern for Chapter 41, not a
  Decision 1 question.

Chapter 24 owns two decisions and one audit. Everything else
is deferred to downstream chapters with explicit pointers.
