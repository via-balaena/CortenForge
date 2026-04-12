# Competition API v2

Chapter 20 established that `Competition`'s single-seed shape is
structurally mismatched with a multi-seed experimental protocol and
deferred the specific API shape to this chapter. Chapter 30 made
the pool call for TD3 and SAC (both out on linear-$Q$ grounds) and
deferred the PPO pool-membership call to this chapter. Chapter 30
also flagged that the "matched complexity" phrase load-bearing its
geometry-vs-expressiveness argument needs an operational definition
before any rematch result can be interpreted. This chapter picks
all three.

The picks are:

1. **Replicates API.** Add `Competition::run_replicates(tasks,
   builders, seeds: &[u64])` as the general-case entry point
   and make the existing `run(tasks, builders)` a thin wrapper
   that calls `self.run_replicates(tasks, builders, &[self.seed])`.
   The shared loop body lives in `run_replicates`; `run` is
   one line of forwarding. The return type remains
   `CompetitionResult { runs: Vec<RunResult> }` with `RunResult`
   gaining a `replicate_index: usize` field. Replicate execution
   is sequential. Error handling preserves the current
   first-failure-aborts `Result<_, EnvError>` shape.
2. **PPO pool-membership.** PPO is excluded from the rematch pool.
   Rematch pool collapses to `{CEM, SA}`. The argument is that
   the symmetric-exclusion rule Ch 23 distills from Ch 30's
   TD3/SAC reasoning — known characterized failure mode,
   orthogonal to the rematch's geometry question, expected
   contamination of the rematch result — applies to PPO's
   D1d-style exploration-noise inflation on a risk-management
   calculus: the probability-weighted expected value of PPO
   inclusion is negative because the false-positive-persists
   branch contaminates the rematch while the clean-pass branch
   is only modestly informative. The chapter also clarifies why
   REINFORCE is not a candidate for the pool at all, a point
   that came up during Ch 23's recon and is worth naming
   explicitly.
3. **Matched complexity operational definition.** "Same concrete
   `Policy` implementation, same `n_params()`, equal to what the
   D2c CEM baseline used." The D2c CEM baseline uses `LinearPolicy`
   with `obs_dim = 2` and `act_dim = 1`, giving `n_params = 3`
   (recon-reported at `sim/L0/thermostat/tests/d2c_cem_training.rs:277`
   and `sim/L0/ml-bridge/src/linear.rs:61`). SA must use the same
   policy class and the same parameter count, verified by a
   one-line equality assertion in the rematch test fixture.

The chapter ends with a scope-discipline section naming what it
does not decide — most notably, it does not pick the specific
number of replicates for the rematch, the statistical test the
gate uses, the aggregation rule for `best_reward` across
replicates, or the concrete signature of `run_replicates` down to
every parameter. Replicate count and the gate's statistical test
are a Chapter 32 call that depends on a pilot run that does not
yet exist. The `best_reward` aggregation rule is Chapter 24's
responsibility and is designed to fit cleanly inside the return
shape this chapter picks. The exact `run_replicates` signature is
a Part 4 PR concern that inherits this chapter's API shape.

## What Ch 23 inherits from earlier chapters

Ch 20 established the core mismatch. The current
`Competition::new(n_envs, budget, seed)` at
`sim/L0/ml-bridge/src/competition.rs:290` (recon-reported) takes a
single `u64` seed, stores it on the struct, and passes it directly
to `Algorithm::train(seed)` at `competition.rs:341` (recon-reported).
Every `CompetitionResult { runs: Vec<RunResult> }` is a flat list
of single-replicate results keyed by `(task_name, algorithm_name)`
at `competition.rs:83-86` (recon-reported). The chapter argued
this is structurally mismatched with a multi-seed protocol and
that the fix has to live at the type signature, not in client
code, because otherwise every downstream cross-algorithm benchmark
re-implements the same thirty-line loop at its own risk.

Ch 20 explicitly deferred the API shape:

> It does not specify the exact API shape. Whether the new
> constructor takes `seeds: &[u64]` or `seeds: impl IntoIterator<Item
> = u64>`, whether the result type carries raw `Vec<f64>` or a
> lightweight `SeedSummary { mean, stderr, n }`, what statistical
> test the gate uses, what counts as "enough" replicates for the
> D2c rematch specifically — all of those are chapter 23 (the
> Competition v2 API shape) and chapter 24 (result semantics)
> questions.

Ch 30 made the linear-$Q$ pool calls. TD3 and SAC are excluded
because their D2c failure mode ("linear-$Q$ is too weak to
represent the value landscape," per `project_d2_sr_findings.md:16,18`
and Ch 30 Section "What the D2c result established") is known and
characterized, and including them would "drag the 'best
matched-complexity RL baseline' down and makes any SA victory
larger in apparent effect size than the geometry finding actually
supports" (`30-the-scientific-question.md:125-127`, recon-reported).
PPO was explicitly flagged as a different mechanism and punted to
this chapter:

> PPO's D2c failure was a different mechanism (D1d-style
> exploration-noise inflation, not linear-$Q$), so whether PPO
> stays in the pool is a genuinely harder call that belongs to
> chapter 23 — the chapter's reasoning here does not cover it.
> (`30-the-scientific-question.md:212-216`, recon-reported)

Ch 30 also flagged the matched-complexity definition:

> The operational definition of matched complexity (parameter
> count? parameterization family? representation width?) is a
> chapter 23 question and has to be nailed down before any
> rematch result can be interpreted through the split this
> chapter is building on.
> (`30-the-scientific-question.md:88-92`, recon-reported)

Ch 22 is adjacent but not load-bearing for this chapter. Ch 22's
fair-compute-parity work settled the budget unit, the
configuration uniformity, and the warmup and k-passes framing for
the rematch — all of which are orthogonal to the three decisions
this chapter makes. The one way Ch 22 touches Ch 23 is that the
rematch configuration it committed to (`n_envs = 32`,
`max_episode_steps = 5000`, `TrainingBudget::Steps(16M)`) is the
fixed context in which "matched complexity" gets measured, and
the rematch is exactly what the replicates API has to host. Ch
23's picks are consistent with Ch 22's commitments and do not
revisit them.

## Section 1 — Replicates API: `run_replicates(seeds: &[u64])`

The first decision is the shape of the replicates entry point.
Four sub-questions compose the answer: whether to add a new method
or mutate the existing one, how to surface the seed list, what
the return shape looks like, and how execution order and errors
are handled. Each sub-section addresses one.

### 1.1 A new method, not a replacement for `run`

The natural impulse is to change `run` to take a seed list directly
and retire the single-seed shape. This chapter picks the weaker
option: leave `run` as it is, add `run_replicates` as a sibling,
and let callers opt in.

The reason is callsite cost. `Competition::run` has a test-suite
presence that is wider than the rematch. The competition tests at
`competition.rs:520-538` (recon-reported) use the single-seed
shape as a sanity check for the harness itself —
`competition_runs_all_pairs`, `competition_empty_tasks`,
`competition_empty_builders`, `competition_multiple_tasks`,
`run_result_final_reward`, and several artifact tests — none of
which care about replicates and several of which exist specifically
to verify that the harness handles pathological cases without the
replicate complication. Making `run_replicates` a sibling keeps
those tests unchanged and lets the replicates-aware surface be
added without rewriting the harness's existing test-suite
investment.

The cost of the sibling approach is that `Competition` now has
two entry points where one would suffice, and a careless caller
can invoke `run` in a context where `run_replicates` is the
intended API. This is a real cost and the rematch test has to be
the first one that gets it right — if the D2c rematch test is
written against `run`, the whole Ch 20 argument is defeated at
its one most visible moment. Part 4's execution PR for this change
owns the responsibility for converting the rematch test, which
means the rematch test is what forces `run_replicates` into
existence rather than just being offered it.

**The pick is: add `run_replicates` and make `run` a thin wrapper
over it.** `run_replicates` at `competition.rs` (new) carries the
shared internal loop: outer over seeds, middle over tasks, inner
over builders, fresh `VecEnv` per `(task, builder)` pair. `run`
becomes a one-line forwarder — `self.run_replicates(tasks, builders,
&[self.seed])` — and returns `Result<CompetitionResult, EnvError>`
unchanged. Existing tests that call `comp.run(...)` stay unchanged
at the callsite and continue to hit the same machinery, now
routed through the replicates-aware implementation. The
single-source-of-truth property is preserved: there is one
internal loop body, one error-handling path, one provenance-
population block, and the two public entry points are a general
case and a cheap specialization of it.

This is stronger than "two independent sibling methods." The
sibling framing was tempting because it minimizes the diff in the
mechanical sense — nothing about `run`'s body has to change — but
it leaves `Competition` with two parallel implementations that
can drift. The wrapper framing is slightly more work in the
initial PR (the existing `run` body becomes the new
`run_replicates` body, with a seed-loop outer) but costs nothing
in ongoing maintenance and lets the chassis honestly carry the
Ch 20 contract in one place. Chapter 20's argument — that the
chassis must carry the multi-seed contract once so every
experiment inherits it — is best served by making the contract
live in one internal method and exposing it under both a
single-seed and a multi-seed name.

A narrower alternative worth naming: mutate `run`'s signature
directly to take `seeds: &[u64]` and retire the single-seed
shape. This is the cleanest type-level fix — one method, one
shape — but it breaks every existing test callsite (the harness
tests at `competition.rs:520-538`, recon-reported — plus every
downstream callsite in the workspace that consumes the
`Competition::run(n, budget, seed)` → `.run(tasks, builders)`
pattern). The wrapper-over-`run_replicates` form is a better
trade because it gives the same single-source-of-truth property
without the callsite churn. Retiring `run` entirely is a valid
follow-up for a later PR once the replicates-aware path has
stabilized.

### 1.2 Seed surface: `&[u64]`, caller owns derivation

The second question is what `run_replicates` takes in place of
`run`'s `self.seed`. Three candidates were plausible:

- **`seeds: &[u64]`**: the caller passes a slice of seeds directly.
- **`(master_seed: u64, n_replicates: usize)`**: the caller passes
  a master seed and a count, and `Competition` derives the
  per-replicate seeds internally via some chosen scheme.
- **`seed_fn: impl Fn(usize) -> u64`**: the caller passes a seed
  factory closure and a count.

Ch 23 picks **`&[u64]`**. The load-bearing argument is
physics-seed-domain separation; two secondary considerations
support it but would not on their own determine the pick.

**Part 1 (load-bearing): separation of concerns with the physics
seed domain.**
Chapters 14 and 15 decided the physics-noise side of the chassis
uses shape 3 (counter-based PRF) with `(master_seed, traj_id)`
hosted on per-env component instances via `install_per_env`. The
master seed the physics side takes is completely separate from
the `u64` the algorithm side takes — the physics seed keys a
ChaCha8 PRF over the Langevin noise sequence, and the algorithm
seed keys the RL algorithm's internal RNG (weight initialization,
batch shuffling, exploration noise sampling). The two seed
domains do not share any meaningful structure, and a rematch run
needs both to be controlled independently. Baking a
`(master_seed, n_replicates)` derivation scheme into
`Competition` would create an implicit expectation that the
algorithm seed and the physics seed should be derived from the
same source, which is the opposite of what the separation demands.
The `&[u64]` shape stays out of that entanglement — the caller
picks whatever algorithm seeds they want, the factory closure in
`BatchSim::new` picks whatever physics seeds it wants, and the
two are syntactically and semantically decoupled.

**Part 2 (secondary): the factory closure adds signature
complexity without benefit.** Option 3 (`seed_fn: impl Fn(usize)
-> u64`) is the most flexible of the three candidates — the
caller can compute each seed from an index — but the flexibility
has no use case the simpler `&[u64]` shape does not already
cover, because any caller with a factory closure can just call
it over `0..n` and `collect::<Vec<_>>()` at the callsite. The
closure signature adds a generic parameter to `run_replicates`,
which makes the method harder to invoke with a `&dyn` trait
object and harder for a new reader to parse at the signature
level. The benefit over `&[u64]` is zero and the cost is
nonzero; the slice wins.

**Part 3 (secondary): consistency with the existing single-seed
pattern.** `Competition::new` already takes `seed: u64` as a raw
integer at `competition.rs:290` (recon-reported) and passes it
through to `Algorithm::train` unchanged at `competition.rs:341`
(recon-reported). A `(master_seed, n_replicates)` signature
would introduce a derivation scheme (splitmix64, XOR-with-index,
or whatever) baked into `Competition` at the type level where
the current single-seed path has none. This is a consistency
argument and is genuinely weaker than Part 1 — "the existing
pattern is raw `u64`" is a reason to continue raw `u64` only if
the raw `u64` pattern is itself correct, which circles back to
the physics-seed-domain argument. It is worth naming as a
secondary consideration because a reader evaluating the
`&[u64]` pick should not have to relitigate the raw-seed choice
from first principles, but it is not load-bearing on its own.

The callsite convention Ch 23 recommends, but does not enforce,
is splitmix64 applied to a master seed:

```rust
let master: u64 = 20_260_412;
let seeds: Vec<u64> = (0..n_replicates)
    .map(|i| splitmix64(master.wrapping_add(i as u64)))
    .collect();
let result = comp.run_replicates(&tasks, &builders, &seeds)?;
```

The splitmix64 convention is cheap, well-known, has no collision
mode at master seed 0, and produces streams that are independent
enough for practical replicate comparisons. A simpler convention —
`(master..master + n)` — also works for most practical purposes
but has a mild correlation structure that can surprise readers;
splitmix64 is the conservative choice. The study's recommendation
is splitmix64 in prose and the API stays silent.

### 1.3 Return shape: flat `Vec<RunResult>` plus `replicate_index`

The third sub-question is what `run_replicates` returns. The
current `CompetitionResult` has one field — `runs: Vec<RunResult>`
at `competition.rs:85` — and each `RunResult` carries `task_name`,
`algorithm_name`, `metrics`, `artifact`, and `best_artifact`
fields at `competition.rs:23-34` (recon-reported). The task-major
ordering of `runs` is a flat cross-product of tasks and builders.
Two natural ways to extend this to replicates:

- **Nested.** Return `Vec<ReplicateGroup>` where each group holds
  `task_name`, `algorithm_name`, and a `replicates: Vec<RunResult>`
  sub-field. Chapter 24's `best_reward` aggregation operates on
  the sub-field directly.
- **Flat.** Return `Vec<RunResult>` (unchanged field name and type)
  where each `RunResult` has a new `replicate_index: usize` field,
  and the length of `runs` is `tasks.len() * builders.len() * seeds.len()`.

Ch 23 picks **flat**. The pick is on ergonomics and refactor
cost; neither shape is abstractly wrong, and a reader who
preferred nested on "clearer containment of the replicate set"
grounds would have a defensible case. The argument for flat has
three parts: the existing helper API generalizes cleanly under
it, the `replicate_index` field is the minimum structural
addition, and Ch 24's likely aggregation shape fits naturally
over a filter-by-`(task, algo)` slice of the flat list.

**Part 1: the existing helper API generalizes cleanly under flat.**
`CompetitionResult` already has three lookup methods for the
flat-list case: `find(task, algorithm)` at
`competition.rs:91-95`, `for_task(task)` at
`competition.rs:99-101`, and `for_algorithm(algorithm)` at
`competition.rs:105-110` (all recon-reported). Under the flat
shape, these methods keep working — `find` returns an
`Option<&RunResult>` for the first match (which is now the first
replicate, and a new `find_replicate(task, algo, replicate_idx)`
sits alongside it for the specific case), `for_task` returns all
replicates across all algorithms for a task, and `for_algorithm`
returns all replicates across all tasks for an algorithm. No
existing caller that uses these methods on a single-replicate
result breaks; every existing caller just gets more elements in
the returned vector.

Under the nested shape, every one of these methods has to be
rewritten. `find` becomes `find(task, algo)` returning a
`&ReplicateGroup`; if the caller wanted a specific replicate,
they then index into `.replicates`. `for_task` returns
`Vec<&ReplicateGroup>`. `for_algorithm` does the same. The
methods are still sensible but the callsites all break, and every
test that uses them has to be rewritten. The flat shape is the
cheaper refactor by a wide margin.

**Part 2: the `replicate_index` field is the minimum addition
needed.** Under the flat shape, the only new field on `RunResult`
is `replicate_index: usize`. The existing fields (`task_name`,
`algorithm_name`, `metrics`, `artifact`, `best_artifact`) are
per-replicate already — they were always per-run, just with the
run count being 1. The single added field makes each `RunResult`
self-identifying within the full replicate set.

Ch 24 will define how `best_reward` aggregates across replicates.
Under the flat shape, the aggregation is naturally expressed as a
function over a slice filtered by `(task_name, algorithm_name)`:

```rust
fn best_reward_aggregate(result: &CompetitionResult, task: &str, algo: &str) -> Option<f64> {
    let replicates = result.runs.iter()
        .filter(|r| r.task_name == task && r.algorithm_name == algo);
    // Ch 24 specifies the aggregation body.
}
```

Chapter 24 is free to define any aggregation it wants over this
slice — mean with standard error, median with IQR, best-of-N,
bootstrap confidence interval, whatever the analysis calls for —
without Ch 23 having committed the return type to a specific
shape. Under a nested shape, the same aggregation would operate
on `group.replicates` rather than a filter on `result.runs`. The
call sites are comparable in length; the difference is that the
filter idiom composes with the existing flat-list helpers
(`for_task`, `for_algorithm`) while the nested `group.replicates`
idiom lives inside a new `ReplicateGroup` type that has to be
introduced. Both work. Flat is picked because it costs fewer new
types and fewer rewritten helpers; a reader who is unmoved by
that trade and prefers the containment discipline of a nested
type is not wrong, and Ch 23 is being honest that this is an
ergonomics-and-refactor call rather than a principled one.

**Part 3: per-replicate full state is worth preserving.** An
alternative to the flat `RunResult`-per-replicate shape would be
to store only an aggregated summary per `(task, algorithm)` pair
(e.g., `mean_best_reward: f64`, `std_best_reward: f64`, `n_seeds:
usize`) and drop the per-replicate `metrics`, `artifact`, and
`best_artifact` from the result. This is smaller but lossy: a
future analysis of variance within the replicate set, or a
diagnostic plot of per-replicate learning curves, or an artifact
save for each replicate's best-epoch snapshot, all require the
per-replicate data to be preserved. Dropping it forces a full
re-run to recover. The flat-per-replicate shape keeps everything
the existing per-run machinery already produces and adds one
field; it is the minimum-diff answer that does not close the
door on any future analysis.

The concrete change to `RunResult` at `competition.rs:23-34` is:

```rust
pub struct RunResult {
    pub task_name: String,
    pub algorithm_name: String,
    pub replicate_index: usize,  // NEW: 0-based index within replicate set
    pub metrics: Vec<EpochMetrics>,
    pub artifact: PolicyArtifact,
    pub best_artifact: PolicyArtifact,
}
```

The `run` method (the single-seed sibling) sets `replicate_index
= 0` on every `RunResult` it produces, which keeps it
type-compatible with the new shape and means existing tests that
check `.metrics.len()` or `.best_reward()` continue to pass
unchanged.

The `CompetitionResult::find` helper's existing signature
`find(task, algorithm) -> Option<&RunResult>` becomes ambiguous
under replicates (which replicate is "the" result for `(task,
algo)`?). Ch 23 recommends extending the API with a new
`find_replicate(task, algorithm, replicate_index) -> Option<&RunResult>`
method that addresses the specific replicate, and leaving `find`
with its current behavior for the single-seed case. A caller that
wants all replicates for a `(task, algo)` uses a `.filter()`
directly on `result.runs` — this is a three-line pattern that the
flat-slice approach makes simple, and Part 4 can decide whether
to also add a `for_task_algorithm(task, algo) -> Vec<&RunResult>`
helper. The incremental additions are small and backward
compatible with `find`'s existing behavior on single-replicate
results.

### 1.4 Execution order and error handling

The fourth sub-question is what happens inside `run_replicates`.
The existing `run` loops over tasks then over builders at
`competition.rs:328-329` (recon-reported), creating a fresh
`VecEnv` per pair at `competition.rs:330`. The natural extension
is a third loop, and the question is where the seed loop goes
and what happens when something fails.

**Loop order: `seeds → tasks → builders`, outermost to innermost.**
The rematch interpretation of replicates is "N independent runs
of the full `(task, algorithm)` cross-product under different
seeds," which maps onto seeds-outermost. Each replicate is a
conceptually complete single-seed competition; the outer loop
treats the existing `run` body as a unit and repeats it N times.
This matches the Henderson-style reading of replicates Ch 20
anchored on: "if you run algorithm A with seeds 1–5 and algorithm
B with seeds 6–10 ..." is the template, and the template means
each replicate is a full row of the task × algorithm matrix with
its own seed.

A narrower alternative is to put seeds innermost (fix a task and
builder, run N seeds, then move on). This has a compute-locality
advantage — the `VecEnv` for a `(task, algorithm)` pair could
theoretically be reused across seeds — but the `VecEnv` is
freshly rebuilt per pair in the existing loop at
`competition.rs:330` precisely to prevent cross-run contamination,
and reusing it across seeds would introduce the same contamination
risk across replicates. Keeping the existing "fresh env per
individual run" discipline means there is no compute-locality
benefit to gain from reordering. Seeds-outermost is the cleaner
mapping.

**Execution is sequential.** The replicates within `run_replicates`
run one after the other, same as the task × builder pairs within
`run`. Ml-bridge is on L0 (Bevy-free) and has no `rayon` or
`tokio` dependency; parallelism would be a net-new dependency for
this feature. Ch 22's compute-parity argument is about honest
env-step counts, not wall-clock, and the rematch reports per-family
step totals separately — parallelizing replicates would not
change the reported numbers and would only affect wall-clock. A
caller who wants throughput can wrap individual `run` calls in
rayon at the callsite, which is a pattern that already works with
the existing single-seed API. The chassis does not need to offer
parallel replicates as a built-in.

**Wall-clock is a real concern that Ch 23 is not resolving.** At
the rematch's `TrainingBudget::Steps(16M)` with `n_envs = 32` and
`max_episode_steps = 5000` (from Ch 22), a single replicate
already runs for tens of minutes to hours depending on physics
cost. Sequential replicates at `n = 5` or `n = 10` push the full
rematch into multi-hour-to-overnight territory, which is a cost
the rematch has to absorb whether it likes it or not. Ch 23 is
noting this rather than addressing it — the sequential-execution
pick stands because parallelism is out of scope for the
replicates API itself, and the choice of whether to wrap
replicate calls in externally-managed concurrency (a background
`cargo xtask` job, a GitHub Actions matrix, a local `parallel`
shell utility) belongs to whoever runs the rematch, not to
`Competition`.

**Error handling preserves the current `Result<_, EnvError>`
shape.** `run` returns `Result<CompetitionResult, EnvError>` at
`competition.rs:325`, and the only error channel is
`task.build_vec_env()` at `competition.rs:330`. Algorithm failures
today are caught post-hoc via `RunResult::assert_finite()` at
`competition.rs:66-76` (recon-reported), which panics on
`NaN`/`Inf` rewards. `run_replicates` inherits this exact error
model: the first `VecEnv` construction failure in any replicate
aborts the whole call and returns `Err(EnvError)`, and per-epoch
NaN/Inf detection still happens at the call site via
`assert_finite()` on each `RunResult` after the call returns.

A wider alternative — per-replicate `Result` collection, where
`run_replicates` returns `Vec<Result<CompetitionResult, EnvError>>`
or some structured error-and-success mix — was considered and
rejected. The cost is that every caller now has to iterate the
outer `Vec` and decide what to do with partial failures. The
benefit is that a single failed replicate does not abort the
whole run. But the rematch and every other cross-algorithm
benchmark treats replicate failure as a hard stop: if `n = 10`
and seed 3 hits an `EnvError`, the replicate set is incomplete
and the statistical gate cannot be run regardless, so the caller
just has to propagate the error anyway. Per-replicate `Result`
collection buys nothing for the rematch and makes the API harder
to use for simpler cases. The first-failure-aborts behavior is
the right default.

### 1.5 The Chapter 24 hook

Ch 23's replicates return type has to leave Ch 24 room to do its
job without contortions. Ch 24's responsibility is the
`best_reward` semantic audit across the five algorithms —
Chapter 20 flagged that CEM uses per-step mean, REINFORCE/PPO use
per-episode total, TD3/SAC use mean over completed episodes, and
the existing `RunResult::best_reward()` helper at
`competition.rs:43-50` computes the maximum over those
differently-defined per-epoch means. Under replicates, the
aggregation question becomes "best across which replicates, and
how defined."

The flat `Vec<RunResult>` + `replicate_index` shape gives Ch 24
the following hooks:

1. Per-replicate raw state (`metrics`, `artifact`, `best_artifact`)
   is preserved, so Ch 24 can define aggregation at any level of
   granularity — per-epoch, per-run, per-replicate-set.
2. The `(task_name, algorithm_name)` keying of the flat list
   matches how Ch 24 will want to slice. A filter over
   `result.runs` by `(task, algo)` returns exactly the replicate
   set that Ch 24's aggregation operates on.
3. The `RunResult::best_reward()` method (the existing helper)
   continues to work on a single `RunResult` as it does today —
   returning the max-per-epoch-mean within one replicate. Ch 24's
   aggregation across replicates layers on top of this existing
   helper rather than replacing it. If Ch 24 decides the
   per-algorithm `best_reward` definition is itself the problem
   and the helper needs to be rewritten, the rewrite happens on a
   single-replicate `RunResult` and the across-replicate layer
   inherits the fix.

Ch 23 does not commit Ch 24 to any specific aggregation rule. The
hook is structural — "here is the slice, here is the
single-replicate helper, do whatever you need to do." Ch 24 picks
the aggregation and may introduce new helpers on
`CompetitionResult` (e.g., `mean_best_reward(task, algo)`,
`bootstrap_ci(task, algo, n_bootstrap)`); Ch 23 is silent on
which and leaves the surface extensible.

## Section 2 — PPO pool-membership: excluded

The second decision is whether PPO stays in the rematch pool. Ch
30 excluded TD3 and SAC on linear-$Q$ grounds and explicitly
deferred PPO to this chapter. Ch 23 excludes PPO. The pool
collapses to `{CEM, SA}`. The argument has three parts: the
symmetric-exclusion rule Ch 30 applied, the specific mapping onto
PPO's D1d-style failure mode, and a clarification about REINFORCE
that came up during recon and is worth naming.

### 2.1 The symmetric-exclusion rule Ch 23 distills from Ch 30

Chapter 30's reasoning for excluding TD3 and SAC was not "these
algorithms are bad." It was closer to "these algorithms have a
known characterized failure mode on this task and including them
would drag the RL baseline down." The chapter's framing was:

> including an algorithm whose D2c failure we already know the
> mechanism of means one of the "algorithms in the comparison" is
> guaranteed to under-perform for a reason unrelated to geometry,
> which drags the "best matched-complexity RL baseline" down and
> makes any SA victory larger in apparent effect size than the
> geometry finding actually supports.
> (`30-the-scientific-question.md:122-127`, recon-reported)

Ch 30's passage is a two-clause argument: (i) the failure mode is
known, (ii) including the algorithm inflates the apparent SA
margin. Ch 23 distills it into a three-part rule for its own use,
because PPO's case is different enough from TD3/SAC's that the
informal two-clause argument has to be explicit about what
"unrelated to geometry" means before it can be mechanically
applied. The three-part rule Ch 23 uses:

(a) The failure mode is known and characterized at the
    mechanism level, not just observed at the outcome level.
(b) The failure mode is orthogonal to the rematch's scientific
    question — the algorithm's "under-performance for a reason
    unrelated to geometry" phrasing in Ch 30 has to actually
    point at a non-geometry reason.
(c) Including the algorithm would contaminate the rematch's
    result in a way that a post-hoc reader cannot disentangle
    from the geometry comparison.

This three-part formalization is Ch 23's own work, not a direct
quote from Ch 30. It is internally consistent with Ch 30's
argument and (Ch 23 will argue) produces the same answer on
TD3/SAC that Ch 30 produced directly. It is worth naming as a
reconstruction so a reader can evaluate Ch 23 on the strength of
the rule itself rather than treating the rule as inherited
infrastructure.

**A subtlety that matters for Section 3.** Chapter 30 names
TD3/SAC's failure as linear-$Q$ expressiveness — specifically,
"their internal representation of the value function did not
have enough parameters to track the problem"
(`30:74-76`, recon-reported). Section 3 of this chapter defines
matched complexity as a rule on the *policy* (`LinearPolicy` with
`n_params = 3`, anchored to D2c CEM's baseline). A careless read
would conclude that "matched complexity" already controls for
the TD3/SAC failure and therefore the TD3/SAC exclusion is
redundant. It is not, for a structural reason worth stating
explicitly: TD3/SAC's linear-$Q$ failure lives in the
*value-function approximator*, a component CEM and SA do not
have at all. At D2c,
`sim/L0/thermostat/tests/d2c_cem_training.rs:299-302`
(recon-reported) shows TD3 constructing `LinearQ::new(OBS_DIM,
ACT_DIM, &scale)` twin-critic instances that are separate from
its `LinearPolicy` actor. "Matching complexity on the policy"
leaves the Q approximator unchanged, and TD3/SAC still operate
under a linear Q regardless of what the policy looks like.
Matching complexity on the Q instead of the policy is not a
sensible move either, because CEM and SA have no Q function to
match against. The TD3/SAC exclusion stands because the failure
lives in a component that the matched-complexity rule
structurally cannot reach.

For TD3 and SAC, Ch 30 establishes (a) — the linear-$Q$
mechanism is characterized in `project_d2_sr_findings.md:16,18`
(recon-reported — TD3 and SAC converged to eval `kt_mult = -0.31`
and `-0.78` respectively, which are physically nonsensical for a
temperature multiplier and reflect value-estimate sign confusion
rather than a policy-exploration issue), (b) — the linear-$Q$
failure is structurally separate from the geometry question (see
the previous paragraph), and (c) — including TD3/SAC would drag
the RL baseline down. The rule excludes both.

The test for PPO is whether the same three parts hold for a
different mechanism.

### 2.2 PPO's D1d-style exploration-noise inflation fits the rule

**(a) Is the failure mode known and characterized?** Mostly yes,
with one honest caveat about how the characterization was built.
`project_d2_sr_findings.md:17,20,22` (recon-reported) records
PPO's D2c result as "PASS* — false positive, synchrony from
transient not SR" with the root cause described as "D1d-style
exploration-noise inflation — per-episode noise (~0.02) exceeds
fitness gradient within SR band." The architectural mechanism is
external: PPO's exploration sigma is a scalar hyperparameter
(`sim/L0/ml-bridge/src/ppo.rs:41,43,45` carrying `sigma_init`,
`sigma_decay`, `sigma_min`, recon-reported) decayed on a fixed
schedule at `ppo.rs:414` (recon-reported) regardless of the
landscape's local gradient. On the SR landscape's broad
elevated-but-flat region, this schedule produces a per-step
action perturbation large enough to dominate the policy's
learning signal, on the D2 SR findings memo's reading.

The caveat: the "D1d-style" label is an analogy that points back
to a different task. The underlying mechanism was first named at
`sim/L0/thermostat/tests/d1d_reinforce_comparison.rs:256-260`
(recon-reported), where it was documented at the source level:
"REINFORCE's training rewards are inflated by exploration noise
... the deterministic policy (`forward()` without noise) never
learns the switching behavior — the noise was doing the work,
not the policy." That finding was about REINFORCE on a Brownian
ratchet task — discrete switching dynamics, a completely
different task family from SR's broad-and-flat continuous
landscape. The D2c findings memo applied the same label to PPO's
D2c result by analogy: PPO and REINFORCE share an identical
external-sigma exploration architecture (PPO at
`ppo.rs:41-45,81,414` and REINFORCE at
`sim/L0/ml-bridge/src/reinforce.rs:34-38,69,261`, both
recon-reported — same field names, same sample-then-decay
pattern, same per-step action perturbation), so the pattern
*should* reproduce on PPO + SR for the same architectural
reason it reproduced on REINFORCE + ratchet. The D2 SR findings
memo's "per-episode noise exceeds fitness gradient" phrasing is
an inference from architectural similarity and D2c output
behavior, not an independent empirical replication of the D1d
diagnostic (running the trained PPO policy deterministically and
observing chance-level synchrony).

This makes the characterization strong enough to ground (a)
under Ch 23's rule but not airtight. If the D1d diagnostic were
run against D2c's trained PPO policy and returned a
deterministic synchrony at the elevated-region floor, (a) would
be closed. As it stands, (a) holds on the strength of the
architectural analogy plus the D2 SR memo's outcome-level
characterization. Ch 23 is committing to the exclusion under
this evidential state rather than waiting for the stronger
diagnostic.

**(b) Is the failure mode orthogonal to the rematch's scientific
question?** Yes. Chapter 30 defines the rematch's question as
whether "physics-aware SA, with geometry-appropriate updates and
the same matched-complexity representation the D2c RL baselines
got, resolves the SR peak more reliably than those baselines
resolved it" (`30:130-135`, recon-reported). The question's
operative axis is the *update rule's geometric shape* — SA's
Metropolis accept/reject with a small Gaussian proposal is a
local random walk, and a local random walk is the shape the SR
landscape needs for peak resolution after the broad region is
found. The question is not about exploration noise calibration,
or about the fixed-vs-learned-sigma distinction, or about how an
algorithm's external noise schedule interacts with landscape
flatness. Those are separate questions that have their own
answers and their own experiments.

PPO's failure mode lives on the exploration-noise-calibration
axis, not the geometric-update-shape axis. Including PPO in the
rematch means the comparison would contain one data point where
"best matched-complexity RL" means "CEM with elite selection" and
one where it means "PPO with a sigma schedule that inflates noise
relative to the gradient." The two data points do not answer the
same question. For the rematch to cleanly test geometry-vs-
expressiveness, the RL side of the comparison has to be an
algorithm whose failure mode (if any) is the geometric one — not
the noise-calibration one. Part (b) holds.

**(c) Would including PPO contaminate the rematch's result in a
way a reader cannot disentangle from the geometry comparison?**
Yes, on the balance of risks. The honest calculus has three
possible outcomes under which PPO might be included at the
rematch's 16M-step budget, and they are not all equally bad:

- **False-positive persists.** PPO at 16M shows a nominal pass
  with the D1d-pattern signature — the trained deterministic
  policy does not resolve the peak, but the training-time
  synchrony looks high because the action noise is doing the
  work. This is strictly worse than the D2c result because it
  is the same false positive re-observed at higher compute,
  and the rematch would report a "PPO result" that a reader who
  did not read the D2 SR findings memo would mistake for
  evidence of RL at matched complexity. This outcome actively
  contaminates the rematch.
- **False-positive washes out into a clean pass.** PPO at 16M
  actually resolves the peak — the noise schedule becomes
  negligible relative to the learning signal at enough compute,
  and the trained deterministic policy cleanly converges near
  `kt_mult ≈ 2.55`. **This would be genuinely informative.** A
  clean PPO pass at matched complexity would mean "at 16M
  steps, policy-gradient RL at `n_params = 3` can resolve the
  SR peak," which is a direct answer to the kind of question
  the rematch cares about — not the same question Ch 30 is
  asking, but an adjacent and interpretable one. The chapter
  should not pretend this case is uninformative.
- **False-positive washes out into a clean fail.** PPO at 16M
  does not resolve the peak even with the noise reduced. This
  is also informative in principle, but only if the reader
  knows which sigma schedule PPO used — the "clean fail"
  result is a property of the algorithm *at that specific
  exploration schedule*, and a different schedule could give
  a different answer. Interpretability is partial.

The exclusion argument rests on the probability weighting across
these three outcomes, not on a symmetric "all branches
contaminating" claim. The false-positive-persists outcome is
strictly bad for the rematch. The clean-pass outcome is good but
modestly informative: it would add one data point to an existing
RL-at-matched-complexity question that Ch 30 did not assign the
rematch to answer. The clean-fail outcome is ambiguous because
it depends on hyperparameter choices that Ch 23 is not
committing to tune. The expected value of including PPO — the
probability-weighted sum of "contamination" vs "modest
information gain" vs "ambiguous finding" — is negative if the
probability of the persistence case is non-trivial, which the D2
SR findings memo's D1d-analogy reading suggests it is.

Ch 23 is excluding PPO on this expected-value calculation, not
on a claim that every possible PPO result would be confounding.
Part (c) holds under the honest reading: *the risk of
contamination exceeds the value of the information PPO might
provide*, given the D2 SR memo's characterization and the
rematch's scope discipline around its central question.

All three parts of the rule apply to PPO, under the honest
evidential states Ch 23 just named. **PPO is excluded.** The
exclusion is a defensible risk-management call, not a proof that
PPO's result would be strictly uninformative.

### 2.3 REINFORCE was never in scope

A clarification that came up during Ch 23's recon: REINFORCE is
not in the rematch pool, and it is not being excluded from it
either — it was never a candidate. The D2c study ran exactly
four algorithms (CEM, TD3, PPO, SAC) at
`sim/L0/thermostat/tests/d2c_cem_training.rs:274-387`
(recon-reported — four `#[test]` functions, `d2c_cem`, `d2c_td3`,
`d2c_ppo`, `d2c_sac`, one per algorithm). The `project_d2_sr_findings.md:11`
summary (recon-reported) states: "D2c: 4-algorithm comparison
(CEM, TD3, PPO, SAC)." REINFORCE was not in D2c. Ch 30 does not
mention REINFORCE by name in its pool-membership discussion
because the rematch pool is scoped to D2c's participants minus
Ch 30's exclusions — REINFORCE was never in that set.

This matters because the Ch 23 recon initially surfaced a concern
that REINFORCE and PPO share an identical external-sigma
architecture (at `reinforce.rs:34-38,69,261` and
`ppo.rs:41-45,81,414`, both recon-reported), and any argument
excluding PPO on exploration-noise grounds would by symmetry
apply to REINFORCE too. If the rematch pool were "all on-policy
algorithms in ml-bridge," this symmetry argument would mean
excluding PPO forces excluding REINFORCE. But the rematch pool
is not "all on-policy algorithms in ml-bridge." It is "the subset
of D2c's pool that Chapter 30's exclusion rule does not remove."
D2c's pool did not include REINFORCE. The symmetry concern
dissolves: REINFORCE is out-of-scope for the rematch not because
it failed a test, but because it was never up for inclusion.

The observation that REINFORCE's architecture is vulnerable to
the same D1d-style pattern is still true — and in fact it is
stronger than the PPO observation because D1d itself is where the
pattern was first named on REINFORCE (at
`d1d_reinforce_comparison.rs:256-260`, recon-reported). But
"REINFORCE exhibits exploration-noise inflation" is a finding
about REINFORCE in its own right, not a fact about the rematch
pool. The rematch pool question is a narrower one: of the D2c
participants, which stay in after Chapter 30's rule is applied?
The answer for PPO is "out." The answer for REINFORCE is "not
applicable — not a participant."

A future study that surveys on-policy RL on SR-like landscapes at
scale would absolutely want to ask whether REINFORCE exhibits the
same D1d-style inflation on the SR task that PPO did, and whether
the pattern persists at higher compute budgets. That study would
be a separate investigation with its own scope; it is not what
the rematch is.

### 2.4 What the exclusion costs and what it does not

Excluding PPO collapses the rematch pool to two algorithms: CEM
and SA. A two-algorithm pool is thin, and it is worth being
honest about what that thinness means.

**What it costs in the positive case.** If SA beats CEM, the
headline result is "geometry-appropriate updates beat
elite-selection-based sampling at matched complexity on this
task." The two-algorithm pool means the rematch has no third
algorithm that would falsify the more specific alternative
hypothesis "*any* algorithm that takes small local steps beats
CEM on SR" — a rematch with CEM, SA, and a naive Metropolis-
free random walker could distinguish "SA's acceptance rule
matters" from "any local-step method works," and a two-algorithm
pool cannot. A positive SA-vs-CEM result is therefore a
joint bound on SA's specific mechanism *and* on the broader
local-walk category; the rematch cannot disentangle them. This
is a genuine limitation on interpretive scope, not just a loss
of cross-check count. The chapter is stating it and accepting
it, not handwaving it.

**What it costs in the null case.** If SA is statistically
indistinguishable from CEM, the reader cannot rule out "the
RL side of the comparison is the worst member of an otherwise
reasonable pool." A three-algorithm pool with PPO in it would
give the reader a second RL baseline to sanity-check CEM
against: if both CEM and PPO fail at the peak and SA is no
better, the null becomes "neither elite-selection nor
policy-gradient resolves this landscape at matched complexity,"
which is a stronger claim than "CEM specifically doesn't." The
two-algorithm pool collapses the interpretation of the null to
"SA vs CEM only," and any reader who wonders whether the whole
RL family could resolve the peak has to wait for a follow-up
study. This is a real cost that is actually largest in the
null case, not the positive case, and the chapter did not name
this asymmetry in its initial pass.

**What it does not cost.** The rematch is not trying to be a
survey of derivative-free RL on SR-like landscapes. It is trying
to answer one scientific question cleanly — whether SA's
geometry-appropriate update beats CEM's geometry-inappropriate
one on a specific task at matched complexity. A three-algorithm
pool that includes a known-failure-risk third algorithm is
worse in the positive case than a two-algorithm pool because
the known-risk algorithm's expected contribution is more
contamination than information (Section 2.2(c)'s expected-value
calculus). The exclusion is defensible on positive-case grounds
even as it loses interpretive power in the null case; that is
the trade, and it is worth being honest about.

**The follow-up studies.** The broader questions — "does PPO's
D1d-style failure persist at higher compute budgets," "does
REINFORCE show the same pattern on SR specifically," "is SA's
geometry advantage task-specific or general" — are follow-up
studies with their own scopes. Excluding PPO from the rematch
does not foreclose any of them. It only says "the rematch is not
the experiment that answers these." Chapter 30's "what each
outcome would tell us" section already names the follow-up path
(richer SA proposals, Parallel Tempering, the same SR task) as
the natural sequel. A PPO-at-higher-budget investigation would
sit next to those, not inside the rematch.

**Hyperparameter sensitivity is Chapter 32's concern, not this
one's.** Chapter 32 is the study's hyperparameter-sensitivity
closer. It inherits a question from this chapter's exclusion
argument: at what exploration-noise schedule would PPO
(and equivalently REINFORCE) stop exhibiting D1d-style inflation
on the SR task, and is there any schedule that makes the
algorithm's pool-membership defensible? Ch 32 may choose to
answer that question, or may choose to treat it as out-of-scope
and defer to the follow-up study. Ch 23's argument is that PPO
cannot be in the rematch pool as it stands today; Ch 32's answer
to the schedule question does not change the rematch decision
unless it identifies a schedule specifically suited for SR, which
would be an effectively different algorithm.

## Section 3 — Matched complexity: the D2c CEM baseline

The third decision is what "matched complexity" means
operationally for the SA-vs-CEM rematch. Chapter 30 flagged the
question explicitly:

> The operational definition of matched complexity (parameter
> count? parameterization family? representation width?) is a
> chapter 23 question and has to be nailed down before any
> rematch result can be interpreted through the split this
> chapter is building on.
> (`30:88-92`, recon-reported)

Ch 30 named three candidates — parameter count, parameterization
family, representation width — without committing. This section
picks.

### 3.1 The D2c CEM baseline anchor

The load-bearing observation is that the rematch is explicitly a
rematch of D2c. Its scientific question (Ch 30) is "does SA, at
the same matched-complexity representation the D2c RL baselines
got, resolve the SR peak more reliably than those baselines
resolved it." The phrase "the same matched-complexity
representation the D2c RL baselines got" points directly at D2c
as the reference: whatever policy class and capacity D2c used for
CEM is what SA has to match. Anchoring to D2c eliminates the
cherry-picking concern — the rematch cannot silently pick a
richer policy class for SA that D2c's CEM did not have, because
the anchor is fixed.

The D2c CEM baseline uses `LinearPolicy::new(OBS_DIM, ACT_DIM,
&obs_scale())` at `sim/L0/thermostat/tests/d2c_cem_training.rs:277`
(recon-reported) with `OBS_DIM = 2` and `ACT_DIM = 1` at
`d2c_cem_training.rs:81-82` (recon-reported). `LinearPolicy` is
defined at `sim/L0/ml-bridge/src/linear.rs` and its constructor
at `linear.rs:55` (recon-reported) computes
`n_params = act_dim * (obs_dim + 1) = 1 * (2 + 1) = 3` at
`linear.rs:61` (recon-reported). The D2c CEM baseline's parameter
count is three.

The test file makes this visible at the callsite: the line
immediately after policy construction at
`d2c_cem_training.rs:278` is `policy.set_params(&[0.0, 0.0, 2.0])`
(recon-reported), which sets exactly three parameters. The "3" is
not a study-level inference — it is on the page, three
initialization values for three weight slots.

### 3.2 The operational rule

Ch 23 defines "matched complexity" as:

**Same concrete `Policy` implementation, same `n_params()`, equal
to what the D2c CEM baseline used. Verifiable as a one-line
equality assertion in the rematch test fixture.**

Concretely, for the D2c rematch specifically: SA operates on a
`LinearPolicy` with `obs_dim = 2`, `act_dim = 1`, and `n_params =
3`. The rematch test fixture constructs both CEM's and SA's
policies, asserts `cem_policy.n_params() == sa_policy.n_params()`,
and asserts both are of type `LinearPolicy` (or, if SA takes a
`Box<dyn Policy>` like CEM does, that the concrete type behind
the trait object is `LinearPolicy`). This is one assertion, takes
one line of test code, and makes the match explicit at runtime
rather than at the level of the study's narrative.

The rule has three effects:

1. **Same parameterization family.** Both algorithms operate on
   a `LinearPolicy`, which eliminates the expressiveness channel
   Chapter 30 worried about. A linear policy has one functional
   form — the action is a scaled inner product plus a bias. SA
   and CEM cannot differ on expressiveness because they are
   sampling over the same parameter space.

2. **Same representation width.** `n_params() == n_params()` is
   the testable version of "same representation capacity." If a
   future rematch wants to use MLP policies, the same rule applies
   — both algorithms use the same MLP architecture with the same
   width and depth — but the D2c rematch specifically inherits
   `LinearPolicy(2, 1)` because that is what D2c's CEM baseline
   used.

3. **No hidden asymmetry from picking.** The anchor is "whatever
   D2c used," not "whatever we think SA needs." A rematch that
   picked a policy class SA happens to work well with would be
   the cherry-picking failure mode Ch 30 named. The D2c anchor
   rules that out by construction.

**Ch 30's three-way framing is D2c-specific, not general.**
Chapter 30 named three candidates for operationalizing matched
complexity — parameter count, parameterization family,
representation width — and Ch 23 is picking all three at once
only because the D2c CEM anchor makes them coextensive. A linear
policy at `obs_dim = 2, act_dim = 1` has one family, one width,
and one parameter count: the three axes collapse onto a single
structural choice, and matching one matches all three. A future
non-D2c rematch with a richer policy class (an MLP with variable
width and depth, for instance) would have to pick which of the
three axes is primary when they pull apart — e.g., a 3-param
linear policy vs a 3-param 1-layer MLP with tied weights have
the same parameter count but different parameterization families
and different representation widths, and Ch 23's rule gives no
guidance on which to match. This is an intentional gap: the
rematch Ch 23 is scoping is the D2c rematch, and the D2c anchor
resolves the three-way question by forcing a specific
`LinearPolicy` instance. Future rematches inherit an open
question that Ch 23 is not answering, and whichever chapter sets
up a non-D2c rematch will have to make the matched-complexity
call for its own scope.

### 3.3 What the rule does not require

The rule does not require that SA and CEM use the same *learning
hyperparameters*. CEM has `elite_fraction`, `noise_std`,
`noise_decay`, `noise_min` (from `CemHyperparams` at
`sim/L0/ml-bridge/src/cem.rs:28-39`, recon-reported). SA has
whatever hyperparameters SA has (initial temperature, cooling
schedule, proposal standard deviation, etc.). These are
algorithm-specific and do not affect the parameterization of the
policy they operate on. "Matched complexity" is about the
representation, not about the update rule — the whole point of
the rematch is to hold the representation fixed and vary the
update rule.

It also does not require the rematch test to train SA with the
same `TrainingBudget` hyperparameters as CEM, beyond what Ch 22
already committed (`TrainingBudget::Steps(16M)` at
`n_envs = 32, max_episode_steps = 5000`). Budget is compute
parity, which is a separate axis Ch 22 handled. Ch 23's rule is
silent on budget because Ch 22 already settled it.

### 3.4 The edge case: SA's representation needs

A thoughtful reader asks whether `LinearPolicy(2, 1)` is actually
rich enough for SA to do its job on the SR task. The D2c rematch
anchor fixes the answer: the representation is whatever D2c's
CEM had, regardless of whether it is the best representation for
SA. If it turns out that SA needs more than three parameters to
resolve the SR peak, one of three things has to happen:

1. **SA fails the rematch with the linear policy.** This is a
   valid outcome and Chapter 30 names it as the "null" case:
   "SA finds the elevated region but, like CEM, cannot resolve
   the peak; its best reward is statistically indistinguishable
   from the best matched-complexity RL baseline"
   (`30:164-167`, recon-reported). The interpretation of the
   null is that the landscape's geometry defeats both SA's
   Gaussian-proposal Metropolis step and CEM's elite-selection
   sampling at this representation, and the next experiment
   either changes the proposal structure (Ch 30's (a) follow-up)
   or switches to Parallel Tempering (Ch 30's (b) follow-up).
2. **SA passes the rematch with the linear policy.** This is the
   positive-case outcome and requires no representation change.
3. **Someone argues that the linear policy is not enough.** This
   argument has to precede the rematch, not follow it. A
   post-hoc "oh, it turns out we needed MLP" move would be the
   classic move-the-goalposts failure that Ch 30's scope
   discipline section explicitly rules out:

   > Picking a new benchmark post hoc is explicitly not on this
   > list: if SR turns out to be a bad discriminator for
   > physics-aware vs generic, that is information about *this
   > experiment*, not a license to redefine the question after
   > seeing the answer.
   > (`30:183-187`, recon-reported)

   The same principle applies to representation: picking a richer
   policy class after seeing SA fail with the linear one would be
   information about the experiment, not a license to redefine
   the matched-complexity anchor.

The honest answer is that the D2c CEM baseline is what the
rematch has to match, whether or not that matching is favorable
to SA. That is the price of anchoring the rematch to a concrete
prior experiment rather than a hypothetical one.

A valid pre-rematch move is to run a small SA pilot on the D2c
linear policy and check for *structural* impossibility — not
"SA's mean performance looks bad on the pilot," which would be
cherry-picking via hyperparameter search, but specifically
objective criteria like acceptance-rate collapse (the Metropolis
acceptance rate is 0 or 1 for the entire pilot run, which means
the proposal step size is miscalibrated at the edge of the
distribution rather than somewhere inside it). A collapsed
acceptance rate is a proposal-calibration issue that can be
fixed up-front by adjusting the proposal standard deviation
before the rematch, and this fix is not cherry-picking because
it is responding to a signal that exists independently of
SA's reward performance. Any other pilot signal — "SA's
mean reward looks low," "SA isn't converging fast enough,"
"the proposal schedule seems off" — is either a reward
signal in disguise or a hyperparameter-search excuse, and the
move-the-goalposts discipline rules both out. The pilot's
scope is narrow by design: it only detects the impossibility
case, not the "might be better with tuning" case.

## What Chapter 23 does not decide

- **The specific number of replicates for the D2c rematch.** Ch
  20 established the need for replicates and Ch 32 owns the
  replicate-count question because it depends on measured
  variance from a pilot run that does not yet exist. The Colas
  et al. "How Many Random Seeds?" paper Ch 20 cited
  (`20-single-seed-is-broken.md:85-89`, recon-reported) is the
  methodology for picking the number once variance is known.
  Ch 23's API shape is agnostic to the count — `&[u64]` accepts
  any length.
- **The statistical test the gate uses.** Ch 24's `best_reward`
  semantic audit decides the per-replicate reduction and Ch 32
  decides the across-replicate statistical test (t-test,
  Mann-Whitney, bootstrap CI, whatever the variance measurement
  justifies). Ch 23's return shape is compatible with any of
  these.
- **The exact `best_reward` aggregation across replicates.** Ch
  24 owns this. Ch 23's flat `Vec<RunResult>` + `replicate_index`
  shape leaves Ch 24 free to define aggregation over any filter
  of the result list without Ch 23 having committed to one.
- **The exact `run_replicates` signature down to parameter names
  and generic bounds.** A Part 4 execution PR owns the concrete
  function signature and the implementation. Ch 23's
  `&[u64]` commitment is the load-bearing type-level call;
  whether the method takes `&[u64]` directly or
  `impl AsRef<[u64]>` or a slice reference with a particular
  lifetime bound is a mechanical choice that Part 4 can resolve
  however it prefers as long as the caller surface is a seed
  slice.
- **The internal implementation layout beyond "one shared loop
  body."** Section 1.1 commits to making `run` a thin wrapper
  over `run_replicates(tasks, builders, &[self.seed])`. How the
  shared loop body is factored — free function, private
  method, inlined — is a Part 4 implementation detail.
- **Follow-up studies for PPO and REINFORCE.** Ch 23 excludes
  PPO from the rematch pool and clarifies REINFORCE's
  out-of-scope status. Whether a future study investigates
  D1d-style exploration-noise inflation at higher compute budgets
  on the SR task, or surveys on-policy RL broadly on SR-like
  landscapes, is a separate scope call that does not belong in
  this chapter.
- **The pilot run's specific design.** Section 3.4 notes that a
  pre-rematch SA pilot on the D2c linear policy is a valid move
  to check whether `n_params = 3` is plausibly navigable under
  objective structural criteria (acceptance-rate collapse). The
  pilot's design (seed count, episode count, evaluation
  protocol) is a pre-rematch concern that depends on whoever
  runs it and is not specified here.

A note on the `(recon-reported)` citations throughout this
chapter. Every file:line reference tagged `(recon-reported)` was
collected during Ch 23's recon pass and verified through Ch 23's
factual pass against the current source tree. The tag is kept
in the text as a trail-of-verification marker. A small number of
these citations are load-bearing — the Section 3 anchor numbers
(`d2c_cem_training.rs:277-278` for `LinearPolicy::new(OBS_DIM,
ACT_DIM, &obs_scale())` and `policy.set_params(&[0.0, 0.0, 2.0])`,
`linear.rs:61` for `n_params = act_dim * (obs_dim + 1)`, and
`d2c_cem_training.rs:81-82` for `OBS_DIM = 2` and `ACT_DIM = 1`)
are the ones the matched-complexity rule is built on, and Part
4's execution PR for the rematch test fixture should re-verify
them against the tree at PR-write time with a hard-fail if the
numbers have drifted. Line-number drift is expected over time;
the structural claim (CEM uses a `LinearPolicy(2, 1)` with
`n_params = 3` at D2c) is what needs to hold.

The three calls Ch 23 does make — `run_replicates` with
`&[u64]` plus flat return, PPO exclusion, matched complexity
anchored to D2c's `LinearPolicy(2, 1)` — are the minimum set
needed for the rematch protocol to have a well-defined API
surface, a well-defined pool, and a well-defined fairness
contract. Chapter 24, Chapter 31, Chapter 32, and the Part 4
execution PRs inherit them as fixed inputs.
