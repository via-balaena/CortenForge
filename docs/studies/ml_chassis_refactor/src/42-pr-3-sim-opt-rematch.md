# PR 3: sim-opt split and rematch

Part 4's third and final PR-plan chapter picks up where Ch 41
left off. Ch 40 turned Ch 15's C-3 pick and the session-4 D1–D15
table into PR 1 — the chassis-reproducibility execution plan.
Ch 41 did the equivalent work for PR 2, the Competition
replicates API + algorithm surface fixes PR. Ch 42 does the
equivalent work for PR 3, the execution layer for the rematch
itself — the PR that creates a new `sim-opt` crate from scratch,
lands Simulated Annealing as a new `Algorithm` implementation
under the matched-complexity anchor Ch 23 §3 specified, ships
the bootstrap / bimodality / classify analysis machinery Ch 32
§4.8 sketched, and builds the rematch test fixture that runs
Ch 32's folded-pilot protocol end-to-end.

PR 3 is a different animal from PR 1 and PR 2. PR 1 was a
chassis rewire on existing code — one new module alongside a
constructor rewrite and a ripple across 18 files. PR 2 was an
additive API extension on a single file plus a targeted
three-file surgical fix to per-algorithm metric computation.
PR 3 creates an entirely new crate, implements an entirely new
algorithm, and builds an entirely new test fixture whose purpose
is to *run* the scientific experiment the rest of the study has
been preparing. The diff is larger than PR 2 and comparable to
PR 1 in total line count, but its shape is different from both:
PR 3 is greenfield code in a new location, with very few edits
to committed code paths and almost no rippling migration.

Ch 42 plans a **2-PR split**. PR 3a ships the `sim-opt` crate
itself — `Cargo.toml`, `lib.rs`, the `Sa` / `SaHyperparams`
structs with their `Algorithm` trait implementation under
`sim-opt/src/algorithm.rs`, and the analysis module at
`sim-opt/src/analysis.rs` containing `bootstrap_diff_means`,
`bootstrap_diff_medians`, `bimodality_coefficient`, and
`classify_outcome`. PR 3a is a purely additive new-crate
landing with no touched files in ml-bridge, thermostat, or
sim-core beyond the workspace `Cargo.toml` `members` list
addition and a one-line `sim-opt = { path = "sim/L0/opt" }`
entry in `[workspace.dependencies]`. It has zero external
consumers at merge time and can land independently of PR 1b,
PR 2b, or any other in-flight Part 4 work. PR 3b ships the
rematch test fixture at `sim/L0/opt/tests/d2c_sr_rematch.rs`,
which is the runnable form of Ch 32 §4.8's skeleton. PR 3b
depends on both PR 1b and PR 2b being merged first, because
the fixture calls `BatchSim::new_per_env` with per-env
`LangevinThermostat` factories (PR 1b) and consumes the
unit-uniform `EpochMetrics::mean_reward` via
`Competition::run_replicates` with per-replicate seeds flowing
through to `TaskConfig::build_vec_env` (PR 2b + this chapter's
§2 bundled amendment to Ch 41). §8 argues the split against
single-PR and three-PR alternatives.

Ch 42's in-chapter sub-decisions are ten in number — one more
than Ch 40's four and Ch 41's six, reflecting the larger design
surface PR 3 renders. The ten sub-decisions fall naturally into
the sections that argue them: §2 argues (a) the `TaskConfig`
seed-extension patch, §3 argues (b) the `sim-opt` crate
placement, §4 argues (c)–(e) the SA fitness shape + cooling
schedule + default hyperparameters, §5 argues (f) the analysis
module's library-vs-test-file placement, §6 argues (g) the SR
task infrastructure duplication and (h) the rematch test gate
assertion shape, §8 argues (i) the PR split, and §9 names
(j) the writeup artifact handling as a narrow out-of-scope
call.

One of those ten calls — sub-decision (a), the
`TaskConfig::build_fn` seed extension — is not a Ch 42-internal
call in the usual sense. It is a **bundled narrow amendment to
Ch 41** that Ch 42's recon surfaced as a gap in Ch 41's
committed `run_replicates` API surface. The gap: Ch 41's
`run_replicates(tasks, builders, seeds)` threads the per-
replicate seed to `algorithm.train(..., seed, ...)` but calls
`task.build_vec_env(self.n_envs)` with no seed channel,
meaning the `LangevinThermostat`'s `master_seed` stays frozen
at `TaskConfig` construction time and cannot vary per replicate.
For the rematch specifically — and for any future multi-
replicate experiment with stochastic physics — this collapses
half the seed-population variance Ch 32's bootstrap CI is
supposed to measure. §2 renders the patch: extending
`TaskConfig::build_fn` from `Fn(usize) -> Result<VecEnv,
EnvError>` to `Fn(usize, u64) -> Result<VecEnv, EnvError>`,
threading the seed through `run_replicates`'s inner body at
the `build_vec_env` call site, and updating three stock tasks
plus internal test helpers to accept (and, in the current
stock-task case, ignore) the new parameter. The amendment is
bundled into the Ch 42 commit following the `843dc21c`
precedent (Ch 24 §1.9/§2.2/§5 patch bundled into the Ch 41
commit), because PR 2a's documented scope in Ch 41 §1.1 and
§2.1 needs to reflect the extended signature before PR 2a's
code actually ships — otherwise a reader implementing PR 2a
from Ch 41's plan would ship an API that Ch 42's rematch
cannot use, and the error would be caught at PR 3b's review
rather than at the planning layer where Ch 42 is surfacing it.

The rest of this chapter's sub-decisions are Ch 42-internal in
the conventional sense: calls that Ch 32 and the upstream Part
4 plans left as Part-4-execution concerns. Ch 42 renders each,
defends each against the alternatives that could have gone
differently, and closes with the scope-discipline section
naming what it does not decide.

## Section 1 — Scope and inheritance

### 1.1 What Ch 42 inherits from Ch 23

Ch 23 (`57bf1c25`) locked three calls that Ch 42 renders
without re-litigation.

**The rematch pool.** Ch 23 §2 picked `{CEM, SA}` as the
post-D2c rematch pool, with PPO, TD3, and SAC all excluded on
the shared "known characterized failure mode" logic Ch 30
established for TD3/SAC and Ch 23 extended to PPO. The pool
Ch 42's rematch fixture runs is exactly `{CEM, SA}` — two
algorithms, no more, no hedge. TD3 and SAC are not re-tested
in the rematch (they would fail for the same linear-Q reasons
they failed in D2c); PPO is not re-tested either (its D1d-style
exploration-noise inflation would be a re-run of the same
D2c false positive under the same mechanism).

**The matched-complexity anchor.** Ch 23 §3.1 defined matched
complexity as "same concrete `Policy` implementation, same
`n_params()`, equal to what the D2c CEM baseline used." For the
D2c rematch specifically, this resolves to
`LinearPolicy::new(obs_dim = 2, act_dim = 1, &obs_scale)` with
`n_params = act_dim * (obs_dim + 1) = 3` — the formula is at
`sim/L0/ml-bridge/src/linear.rs:61` (recon-reported), and the
D2c CEM baseline's matching callsite is at
`sim/L0/thermostat/tests/d2c_cem_training.rs:277` (recon-
reported) with the follow-up `policy.set_params(&[0.0, 0.0,
2.0])` at `d2c_cem_training.rs:278` (recon-reported) making
the three-parameter count visible at the callsite.

Ch 23 §3.2 committed to "verifiable as a one-line equality
assertion in the rematch test fixture." Ch 42's rematch fixture
at §6 renders this as a pre-run assertion before the
`run_replicates` call: `assert_eq!(cem_policy.n_params(),
sa_policy.n_params())`, with a surrounding sanity-check that
both equal `3`. Ch 31 §4.3 also named the assertion as the
protective mechanism against "SA parameterization drift" —
the failure mode where a future edit to SA's policy class
silently breaks the matched-complexity condition and a
post-hoc reader cannot distinguish a geometry result from an
expressiveness result. Ch 42 implements that gate; Ch 31 is
the index pointing at it.

**The `run_replicates` API shape.** Ch 23 §1.3 picked the flat
`Vec<RunResult>` return shape with a `replicate_index: usize`
field on each run, plus the filter-by-`(task, algorithm)`
slice idiom for building per-algorithm replicate vectors. Ch
41 PR 2a is the execution PR that lands this shape; Ch 42's
rematch fixture consumes it via
`result.replicate_best_rewards("d2c-sr-rematch", "SA")` and
`result.replicate_best_rewards("d2c-sr-rematch", "CEM")`
calls, which return `Vec<f64>` of length `N = 10` (initial
batch) or `N = 20` (post-expansion) per algorithm. The raw
vectors feed into the bootstrap test directly; no
`SeedSummary` transformation happens on the test path.

### 1.2 What Ch 42 inherits from Ch 24

Ch 24 (`b9f85e19`) locked two decisions that Ch 42 consumes.

**Decision 1 — the per-replicate metric.** Every algorithm's
`EpochMetrics::mean_reward` is the "mean per-episode total
reward across `n_envs` trajectories." Ch 41 PR 2b is the
execution PR that fixes CEM, TD3, and SAC to emit values in
this unit (with REINFORCE and PPO as no-ops that already emit
the right shape). Ch 42's SA implementation is greenfield and
adopts the unit by construction — the first line of SA's
`EpochMetrics::mean_reward` computation at §4.2 follows the
exact idiom Ch 41 §2.2's Form (i) applies to CEM:
`epoch_rewards.iter().sum::<f64>() / n_envs as f64`, where
`epoch_rewards` is the per-env per-episode total reward
collected during the multi-env evaluation pass.

The practical consequence: SA emits values in the same unit
as the five existing ml-bridge algorithms (post-Ch 41 PR 2b),
and `RunResult::best_reward()` comparisons between CEM and SA
via the rematch's bootstrap test are unit-correct by
construction. There is no per-replicate reduction asymmetry
Ch 42 has to paper over.

**Decision 2 — the across-replicate aggregation surface.**
`CompetitionResult::replicate_best_rewards(task: &str, algo:
&str) -> Vec<f64>` returns the raw vector of per-replicate
`RunResult::best_reward()` scalars (with `None` entries
silently filtered out, per Ch 24 §4.7's contingency that PR
2b's REINFORCE/PPO zero-fallback cleanup makes the filter
moot in the rematch pool). Ch 24 §4.4 named the raw vector as
the test-agnostic primitive designed to support "any
reasonable statistical test that could be built on top of it,"
and Ch 32 §2.1 confirmed the shape handles bootstrap CI on
the difference of means directly without needing any new
`CompetitionResult` methods. Ch 42's analysis module at §5
calls `replicate_best_rewards` twice — once per algorithm —
and hands the two vectors to `bootstrap_diff_means` as
`&[f64]` slices.

The `describe(task, algo) -> Option<SeedSummary>` convenience
helper and the `SeedSummary { n, mean, std_dev }` struct are
available but not on the test path. Ch 42's rematch writeup
recommendation at §9 suggests that a human-facing writeup
call `describe` for the table caption alongside the bootstrap
CI, but the test fixture itself does not depend on
`SeedSummary` and Ch 42's analysis module operates on the
raw primitive only.

### 1.3 What Ch 42 inherits from Ch 30

Ch 30 (committed before Part 2 branched) named the scientific
question in its operational form:

> SA, with geometry-appropriate updates and the same
> matched-complexity representation the D2c RL baselines got,
> resolves the SR peak more reliably than those baselines
> resolved it.

Ch 30 also named three meaningful outcomes — positive, null,
ambiguous — and pre-committed the allowed follow-up
experiments under the null outcome: (a) a richer proposal
structure for SA (non-Gaussian or CMA-style adaptive
covariance) on the same SR task, or (b) Parallel Tempering on
the same SR task. Picking a different benchmark post-hoc is
explicitly not on the list.

Ch 42 inherits three things from Ch 30 at the rendering layer:

**The operational phrasing of the SA update rule.** Ch 30
§"What 'physics-aware beats generic RL' is supposed to mean"
specifies that SA's update is "Metropolis accept/reject with
a small Gaussian proposal" — "literally a local random walk
with a temperature schedule." The Gaussian proposal family is
locked by this phrasing. Ch 42's SA implementation at §4.1
uses a Gaussian proposal generated via an inlined Box-Muller
helper (matching CEM's `randn` at `ml-bridge/src/cem.rs:115`,
recon-reported) and defends the Gaussian choice against
non-Gaussian alternatives in one paragraph under the "what
Ch 30 locked" framing rather than arguing it as a Ch 42
sub-decision. The *parameters* of
the Gaussian (proposal standard deviation, cooling schedule,
initial temperature) are genuine Ch 42 calls and are argued in
§4.

**The three-outcome classifier as the rematch's verdict
layer.** Ch 32 §3.3 locked the exact CI-to-outcome mapping as
a five-row table; Ch 30 is the chapter that defined the three
outcomes those rows collapse into. Ch 42's `classify_outcome`
function at §5.4 implements Ch 32 §3.3's table as a
deterministic rule, and the `RematchOutcome` enum at
`sim-opt/src/analysis.rs` has exactly three variants
(`Positive`, `Null`, `Ambiguous`) matching Ch 30's framing —
no fourth "weak positive" hedge, per Ch 31 §3.2's writeup
discipline.

**The pre-committed null follow-ups as design guidance for
the analysis module's reusability.** Ch 30's two allowed null
follow-ups — richer proposal structure, Parallel Tempering —
are *on the same SR task*. Both would reuse Ch 42's analysis
module verbatim: the bootstrap CI, the bimodality check, the
classifier, the folded-pilot driver. Ch 42's §5.1 places the
analysis module as a library (`pub mod analysis`) rather than
inlining it in the test fixture precisely so the follow-up
experiments get the machinery for free without having to
copy-paste from the rematch fixture. The library placement is
a Ch 42 sub-decision (see §7 row (h)) and the forward-
compatibility with Ch 30's null follow-ups is its load-bearing
justification.

### 1.4 What Ch 42 inherits from Ch 31

Ch 31 (`6df97c8a` + `6b876bc5` §4.4 patch) enumerated four
buckets of rematch failure modes and named Ch 42 as the owner
of two protective mechanisms and the implementation layer of
two others.

**§3.2 seed-variance envelope (bucket 3, effect-size-vs-noise).**
Ch 31 §3.2 named the ambiguous-outcome failure shape — SA
beats CEM in expectation but the margin is within the seed-
variance envelope — and pointed at Ch 32's test-family
decision as the protective mechanism. Ch 42 implements Ch 32's
bootstrap CI and the Ch 30 three-bucket classification, which
is where the §3.2 protective mechanism physically lands. The
test fixture's verdict classification at §6.7 is the
runtime-enforced version of Ch 31 §3.2's writeup discipline
("the rematch's headline conclusion is one of the three Ch 30
outcomes, and hedged language is dispatched explicitly to the
ambiguous bucket rather than offered as a fourth option").

**§4.1 chassis reproducibility (bucket 4 prerequisite).** Ch
31 §4.1 named Ch 40's PR 1 (`prf.rs` + `LangevinThermostat`
rewrite + `BatchSim::new_per_env`) as the protective mechanism
against latent flakiness under `--features parallel`. Ch 42's
rematch fixture uses `BatchSim::new_per_env` with per-env
`LangevinThermostat` factories (a direct consumer of PR 1b's
chassis). A rematch run *before* PR 1b merges is either
unreproducible (under `--features parallel`) or reproducible
only via the "do not enable parallel features" workaround Ch
31 §4.1 named. Ch 42's PR 3b cannot merge before PR 1b is in
`main`; §8 names this as a hard merge-order constraint.

**§4.2 algorithm-surface metric fix (bucket 4 prerequisite).**
Ch 31 §4.2 named Ch 41's PR 2 as the protective mechanism
against apples-to-oranges `best_reward()` comparisons. Ch 42's
bootstrap test compares CEM's and SA's per-replicate
`best_reward()` values directly, and that comparison is
unit-correct only if PR 2b has standardized CEM's
`mean_reward` (at `cem.rs:209`, recon-reported) onto the
per-episode-total unit. Ch 42's PR 3b cannot merge before PR
2b is in `main`; §8 names this as the second hard merge-order
constraint.

**§4.3 SA parameterization drift (matched-complexity shape).**
Ch 31 §4.3 explicitly names Ch 42's SA implementation as the
protective mechanism's site: "SA must instantiate its policy
as `LinearPolicy(2, 1)` with `n_params = 3`, the rematch test
gate enforces the equality with a one-line assertion (Ch 23
§3's recommended form), and a Ch 42 PR that lands SA without
the assertion fails its own gate." Ch 42 implements the
assertion at §6.5 as part of the rematch test fixture's
pre-run checks. The assertion form is `assert_eq!(
cem_policy.n_params(), sa_policy.n_params())` plus a
surrounding sanity-check that both equal `3`. The assertion
lives in the rematch test fixture rather than inside SA's
internal unit tests, because the matched-complexity anchor is
a property of the *rematch* — it is what Ch 42's rematch
specifically compares — and a future non-D2c rematch with a
different matched-complexity target would use a different
assertion.

**§4.4 pilot existence (statistical-test shape).** Ch 31 §4.4
(post-`6b876bc5` patch) named Ch 32's folded-pilot protocol as
the protective mechanism's spec-layer and flagged a specific
failure mode: "a rematch implementation that ignores the
expansion rule and runs at a fixed `N` would satisfy 'Ch 32
has landed' but not the protective mechanism this entry
requires." Ch 42 implements the folded pilot's expansion rule
at §5.5 (the `run_rematch` driver) and at §6.6 (the driver's
call from the test fixture). The rematch test fixture is the
place where "Ch 32 has landed" becomes "the rematch
implementation honors the protocol."

**§3.1 warmup-overhead gate (currently moot).** Ch 31 §3.1
named the gate's shape (effect size exceeds unbudgeted-
overhead floor by a predetermined factor) and Ch 32 Decision 5
picked the factor (`5×`). Both chapters noted that the gate
is currently moot for the `{CEM, SA}` pool because neither
algorithm has a warmup phase. Ch 42 inherits the moot-ness:
the rematch fixture does not implement a runtime warmup-gate
check, because there is nothing to gate against. §9 names the
gate as deferred-but-recognized, flagged for re-activation if
a future pool extension introduces a warmup phase.

### 1.5 What Ch 42 inherits from Ch 32

Ch 32 (`dee0b254`) is the bible for the rematch's statistical
protocol. Five decisions plus one contingency, all of which
Ch 42 implements verbatim. The implementation layer is the
`sim-opt/src/analysis.rs` module at §5 and the rematch test
fixture at §6. The correspondence is one-to-one; Ch 42 adds
nothing to the protocol and subtracts nothing.

**Decision 1 — test family.** Bootstrap CI on the difference
of means, `B = 10_000` resamples, two-sided 95 percent
interval, percentile bootstrap (not BCa). Ch 42 implements
this as `bootstrap_diff_means(r_a: &[f64], r_b: &[f64], rng:
&mut impl Rng) -> BootstrapCi` in `sim-opt/src/analysis.rs`,
matching the Ch 32 §3.2 pseudocode shape. The `BootstrapCi`
struct carries `point_estimate: f64`, `lower: f64`,
`upper: f64` fields plus a method `classify() -> RematchOutcome`
that applies Ch 32 §3.3's table.

**Decision 2 — significance threshold and Ch 30 mapping.** Ch
32 §3.3's five-row table:

| CI lower | CI upper | Point est | Ch 30 outcome |
|---|---|---|---|
| `> 0` | `> 0` | `> 0` | **Positive** |
| `< 0` | `< 0` | `< 0` | **Null** |
| `≤ 0` | `> 0` | `> 0` | **Ambiguous** |
| `≤ 0` | `> 0` | `≤ 0` | **Null** |
| `≤ 0` | `≤ 0` | `≤ 0` | **Null** |

Ch 42's `classify_outcome` function implements the table as
a `match` expression on `(lower > 0.0, upper > 0.0,
point_estimate > 0.0)`. The strict-inequality-at-zero boundary
is handled explicitly so a reader who recomputes the CI from
the published replicate values gets the same verdict.

**Decision 3 — replicate count protocol.** `N_initial = 10`
with a pre-registered single expansion to `N_final = 20` iff
the initial-batch CI is ambiguous. Hard cap at 20, no
intermediate stops. The initial batch is the pilot (folded
pilot). Ch 42's `run_rematch` driver at §5.5 implements the
protocol as a straight-line function: run 10 replicates,
classify, if ambiguous run 10 more and re-classify, return the
final outcome. No sequential-peeking, no intermediate reads,
no human judgment at the expansion point.

**Decision 4 — initial-batch composition.** D2c SR task,
`{CEM, SA}` at matched complexity, full `Steps(16M)` per
replicate, seeds derived as `splitmix64(MASTER.wrapping_add(
i))` with `MASTER = 20_260_412` matching
`d2c_cem_training.rs:62`'s `SEED_BASE` literal and Ch 23
§1.2's recommended splitmix64 convention. Ch 42's rematch
fixture hard-codes the `MASTER` literal (at `§6.3`) and uses
`sim_thermostat::prf::splitmix64` (shipped by PR 1a) to derive
the per-replicate seeds. Budget enforcement happens via
`TrainingBudget::Steps(16_000_000)` passed to `Competition::new`.

**Decision 5 — warmup-overhead factor.** `5×` for any future
pool extension. Moot for `{CEM, SA}` as noted in §1.4. Ch 42
implements no runtime gate for it.

**Contingency — bimodality escalation.** If either algorithm's
Pearson-based bimodality coefficient exceeds `5/9` at the
classification step, the test substitutes bootstrap on the
difference of medians for bootstrap on the difference of
means, keeping `B`, CI level, and the classification rule the
same. Ch 42 implements `bimodality_coefficient(values: &[f64])
-> f64` at §5.3 using the SAS small-sample-corrected formula
Ch 32 §6.2 specified, and the analysis module's `test_and_
classify` function at §5.5 checks the threshold and dispatches
to `bootstrap_diff_means` or `bootstrap_diff_medians`
accordingly. The threshold check is strict inequality (`bc >
5.0 / 9.0`), matching Ch 32 §6.2's explicit rule for the
boundary case.

One piece of Ch 32's specification that lands in the analysis
module's structure rather than in a single function: the
`run_rematch` driver at §5.5 executes the entire folded-pilot
protocol as a straight-line function, taking `(competition:
&Competition, task_builder: impl Fn(u64) -> TaskConfig,
builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
bootstrap_rng: &mut impl Rng) -> Result<RematchOutcome,
EnvError>` and returning the final classified outcome. The
`task_builder` closure parameter is the Ch 42-novel piece
that allows the driver to construct a fresh seeded `TaskConfig`
per replicate — it is the rendering form the §2 Ch 41 patch
makes possible, and §5.5 uses the exact closure shape §2's
patched `run_replicates` body expects.

### 1.6 What Ch 42 inherits from Ch 40

Ch 40 (`930d3890` + `b5cb3f6c` scope correction) is PR 1.
Three pieces of Ch 40's output flow through to Ch 42:

**`sim_thermostat::prf::splitmix64`.** PR 1a ships
`splitmix64` as a `pub` function at `sim/L0/thermostat/src/
prf.rs` (planned location; recon confirms the file does not
exist today). Ch 42's rematch fixture imports it as
`use sim_thermostat::prf::splitmix64;` and calls it at the
seed-derivation site. The cross-crate dependency is already
in place: `sim-opt/Cargo.toml`'s `[dependencies]` section
under §3.1 lists `sim-thermostat = { workspace = true }`, and
PR 1a's addition of `prf.rs` is picked up automatically.

**`BatchSim::new_per_env(prototype, n, factory)`.** PR 1b
ships the per-env constructor on `BatchSim` with the
`PerEnvStack` trait and `EnvBatch<S>` generic rendering Ch 40
§3.3 spec'd. Ch 42's rematch fixture does not call
`BatchSim::new_per_env` directly — it goes through
`VecEnv::builder`, which under PR 1b gains an analogous per-
env path internally (the vec_env.rs:391 call site at
recon-reported updates from `BatchSim::new(model, n_envs)` to
the per-env factory form when the `VecEnv` is built from a
`PassiveStack` that contains a `LangevinThermostat`). The
rematch fixture's `make_training_vecenv(seed: u64) -> VecEnv`
helper at §6.2 constructs a `VecEnv` via the per-env path,
and each env's `LangevinThermostat` receives `(gamma, k_b_t,
master_seed = seed, traj_id = env_index)` at construction.

**`LangevinThermostat::new(gamma, k_b_t, master_seed,
traj_id)`.** PR 1b ships the 4-argument direct constructor
matching D2's decision. Ch 42's rematch fixture uses the
4-argument form directly; the `master_seed` varies per
replicate via the Ch 41 §2 patch (this chapter's §2), and the
`traj_id` is the env index within the replicate's batch.

### 1.7 What Ch 42 inherits from Ch 41

Ch 41 (`843dc21c`) is PR 2. Four pieces of Ch 41's output flow
through to Ch 42:

**`Competition::run_replicates(tasks, builders, seeds)`.** PR
2a ships the general-case entry point. Ch 42's `run_rematch`
driver calls `competition.run_replicates(&[task],
&builders, &[seed])` per replicate under the patched seed-
threading semantics introduced in this chapter's §2.

**`RunResult::replicate_index: usize`.** PR 2a ships the field
on `RunResult`. Ch 42's rematch fixture does not read the field
directly — it consumes `replicate_best_rewards(task, algo)`
instead, which flattens the ordering internally — but the
field is what lets `replicate_best_rewards` return values in
a stable order across the 10-element initial batch and the
10-element expansion batch. Under §5.5's `run_rematch` driver,
the two `run_replicates` calls produce a `CompetitionResult`
with `replicate_index` values `0..10` and then `0..10` again
for the expansion (each call is its own seeds slice, so the
indices restart at 0 per call). The rematch stitches the two
batches together via `replicate_best_rewards` calls on each
result and concatenation at the analysis layer; it does not
need to read `replicate_index` directly or preserve cross-
batch continuity in the index values, because the analysis
operates on the raw `Vec<f64>` of rewards without caring
which replicate produced each scalar.

**`CompetitionResult::replicate_best_rewards(task, algo)`.**
PR 2a ships the raw vector accessor. Ch 42's analysis module
calls it twice per `run_rematch` invocation — once per
algorithm — to build the `r_sa` and `r_cem` inputs to
`bootstrap_diff_means`. When `run_replicates` is called inside
an outer replicate loop (as in the rematch's case), the
rematch driver concatenates the per-call `Vec<f64>` results
manually: `r_sa.extend_from_slice(&per_call_result.
replicate_best_rewards("d2c-sr-rematch", "SA"));`. Each
per-call result has a one-element vector per algorithm
(because the call has one seed); the concatenation produces
the full 10-element vector for the initial batch, or the full
20-element vector for the expanded batch.

**Unit-uniform `EpochMetrics::mean_reward`.** PR 2b ships the
Decision 1 train-loop rewrites for CEM, TD3, and SAC. Ch 42's
SA implementation at §4.2 adopts the same unit by
construction: SA's per-epoch `mean_reward` is
`epoch_rewards.iter().sum::<f64>() / n_envs as f64`, where
`epoch_rewards` is the per-env per-episode total reward vector
collected during the multi-env evaluation pass. This is the
same formula CEM uses post-PR-2b (at `cem.rs:209`, the
post-Ch-41-patch line-number), line-for-line. The cross-
algorithm comparison Ch 42's bootstrap test performs is unit-
correct by construction.

### 1.8 Genre note: Ch 42 is heaviest by design surface

Ch 40 and Ch 41 were rendering-heavy in the descriptive-but-
precise voice Part 4 committed to: most of the chapter pages
describe file-level diffs, section-by-section, with a minority
fraction devoted to the in-chapter sub-decisions that actually
needed defending. Ch 40's sub-decision count was four; Ch 41's
was six. Ch 42's is ten, and the chapter's section structure
reflects the larger design surface — a new crate, a new
algorithm, a new test fixture, and a bundled upstream patch
all render at the file:line level.

The genre is still rendering-over-arguing. Every sub-decision
Ch 42 makes is defended in the section that introduces it —
§2 for the Ch 41 patch, §3 for the crate layout, §4 for SA's
hyperparameters, §5 for the analysis module, §6 for the
fixture — and §7 tables them for reader reference. The
decisions Ch 42 *inherits* from Ch 23/24/30/31/32/40/41 are
rendered without re-defense; a reviewer who challenges an
inheritance should go to the owning chapter, not Ch 42.

One piece of Ch 42 is genuinely new argument rather than
rendering: §4.2's counterfactual walk through four shapes for
SA's fitness evaluation (single-env-per-iteration vs multi-env
averaging vs replica SA vs inner Metropolis loop). Ch 30 and
Ch 23 lock the Gaussian proposal and the matched-complexity
anchor, but neither chapter addresses how SA's inner loop
composes with BatchSim's parallel batch shape. This is
genuine new ground, and §4.2 argues it explicitly rather than
rendering it.

### 1.9 The relationship to PR 1 and PR 2

The four Part 4 PRs (PR 1a, PR 1b, PR 2a, PR 2b) and Ch 42's
PR 3a and PR 3b form a partial order rather than a total
order:

- **PR 3a (sim-opt crate + SA + analysis module)** is a
  purely additive new-crate landing. It depends on PR 2a being
  merged (because it needs the `Competition::run_replicates`
  surface and the `replicate_best_rewards` helper in its type
  signatures, even though its actual tests can run without any
  replicate calls). It does *not* depend on PR 1a, PR 1b, or
  PR 2b at the code level. PR 3a's unit tests exercise SA on
  a tiny toy task (reaching_2dof, inherited from ml-bridge) to
  verify the `Algorithm` trait implementation; they do not
  exercise the SR task or any stochastic physics.
- **PR 3b (rematch test fixture)** depends on PR 1b and PR 2b
  both merged. PR 1b is needed for `LangevinThermostat`'s
  4-argument constructor and the per-env `BatchSim` path. PR
  2b is needed for the unit-uniform `mean_reward` contract on
  CEM. PR 3b also depends on PR 3a (the `sim-opt` crate must
  exist and the SA implementation must be callable).

The merge-order constraint for PR 3 is therefore:
`PR 1a + PR 1b + PR 2a + PR 2b` all in `main` → `PR 3a` in
`main` → `PR 3b` in `main`. Within the PR 1 and PR 2 bundles,
the additive halves (1a, 2a) can land in any order and the
semantic halves (1b, 2b) depend only on their own additive
halves. PR 3a can land as soon as PR 2a is in `main` (needing
the `run_replicates` surface but not the `mean_reward` fix);
PR 3b is the long pole and merges last, after the other five
PRs are all in place.

§8 argues the merge-order against alternatives and names the
rollback path if any PR has to back out mid-land.

The Ch 41 bundled amendment this chapter's §2 introduces
changes one detail of the above. The amendment extends
`TaskConfig::build_fn`'s signature to take a `u64` seed
argument and threads that seed through `run_replicates`'s
inner `build_vec_env` call. The amendment is entirely within
PR 2a's scope (PR 2a is the additive-API half of PR 2, and
`TaskConfig::build_fn` is part of that API surface). The
amendment does *not* grow PR 2a's merge dependency on PR 1
or on Ch 42's own PR 3 — PR 2a with the amendment still has
no prerequisites beyond what Ch 41 already specified. The
amendment's effect on the merge-order partial order is zero.

## Section 2 — The `TaskConfig::build_fn` seed extension (sub-decision (a))

### 2.1 The gap

Ch 41 §2.1 renders `Competition::run_replicates` as a seeds-
outermost loop over an inner body that calls
`task.build_vec_env(self.n_envs)` once per `(task, builder,
seed)` triple. The seed is threaded through to
`algorithm.train(&mut env, self.budget, seed, &on_epoch)` at
the inner body's training call site, but the `VecEnv`
construction is seed-blind — `build_vec_env` takes `n_envs`
as its only parameter, and the closure captured by
`TaskConfig::build_fn` at `sim/L0/ml-bridge/src/task.rs:43`
(recon-reported) has no seed channel.

The signature at `task.rs:43`:

```rust
pub struct TaskConfig {
    name: String,
    obs_dim: usize,
    act_dim: usize,
    obs_scale: Vec<f64>,
    build_fn: Arc<dyn Fn(usize) -> Result<VecEnv, EnvError>
                     + Send + Sync>,
}
```

The `build_fn` closure takes `usize` (the env count) and
returns `Result<VecEnv, EnvError>`. The seed cannot reach it
from `run_replicates`. Whatever seed the `LangevinThermostat`
uses inside the built `VecEnv` is captured at `TaskConfig`
construction time, frozen for the lifetime of the
`TaskConfig`, and identical across all replicates that run
through the same `TaskConfig`.

For deterministic-physics tasks, this is fine. `reaching_2dof`
at `task.rs:362` (recon-reported) builds a contact-and-spring
model with no stochastic components, and running the task
across 10 replicates with 10 different algorithm seeds
produces 10 different algorithm exploration trajectories over
the same deterministic environment. The seed-population
variance the rematch's bootstrap CI is supposed to measure
comes entirely from the algorithm's exploration RNG; there is
no physics contribution to vary.

For the rematch's D2c SR task, this is not fine. The task's
`LangevinThermostat` contributes a random noise sequence at
every physics step (the Gaussian noise Ch 40's `prf.rs`
`box_muller_from_block` generates), and that noise sequence
is part of what the algorithm is optimizing against. Two
replicates with the same `LangevinThermostat` master seed see
*the same* noise sequence and differ only in the algorithm's
exploration — and the algorithm's exploration is operating on
a fixed environment, not a distribution over environments.
The rematch's scientific question is "does SA resolve the SR
peak more reliably than CEM across the distribution of
physics noise sequences and algorithm exploration
trajectories," and a fixed physics noise sequence collapses
half of that distribution.

The collapse is not subtle. Ch 32's bootstrap CI on the
difference of means measures the variance of the
`(replicate -> best_reward)` mapping, and that variance has
two sources: the algorithm's exploration RNG and the physics
noise sequence. If the physics is frozen, the measured
variance is only the algorithm's contribution, which is
smaller than the joint variance, and the bootstrap CI is
narrower than it should be, and the test over-rejects (too
many positive or null classifications, too few ambiguous).
Ch 32 §4.3's FPR analysis assumes the joint variance is what
the bootstrap sees; under frozen physics, the analysis does
not hold.

This is the gap Ch 42's recon surfaced. The gap does not
exist at the chassis level (PR 1b ships the per-env
`LangevinThermostat` with its own master_seed channel
directly) and it does not exist at the algorithm level (PR
2a threads the seed through to `algorithm.train`). It exists
specifically at the `TaskConfig::build_fn` boundary, where
the seed cannot flow from `run_replicates`'s loop into the
`VecEnv` construction closure.

### 2.2 Why this did not surface in Ch 41

Ch 41 §2.1 wrote the `run_replicates` body with the seed
flowing to `algorithm.train` only. The reason is structural:
Ch 41's scope was the algorithm-surface metric fix and the
replicate API shape, both of which are "same environment,
different algorithm seed" concerns. The rematch's "different
environment per replicate" need did not come up because Ch
41 did not render the rematch — Ch 42 does.

Ch 41 inherited the `TaskConfig::build_fn` signature from the
pre-Part-4 `task.rs` file at `:43` (recon-reported) and did
not touch it. The signature predates the whole study; it was
designed for the reaching tasks, which are deterministic, and
the seedless closure shape was sufficient for those tasks'
needs. When Ch 41 rendered `run_replicates`, the natural move
was to thread the seed to `algorithm.train` (the one site
that obviously needed it) and leave `task.build_vec_env`
alone.

A reader of Ch 41 as drafted might reasonably ask: "Why
doesn't `build_vec_env` take a seed? What if the task is
stochastic?" Ch 41 has no answer because Ch 41's scope did
not cover stochastic tasks. The question surfaces when Ch 42
tries to render the rematch and discovers that Ch 32 §4.8's
skeleton cannot be rendered literally without extending the
`build_vec_env` signature.

This is exactly the "example reveals engine gap" pattern the
user's preference for "fix gaps before continuing" addresses.
Ch 42 cannot render Ch 32's protocol faithfully without
either (a) working around the gap in sim-opt or (b) fixing
the gap at the chassis level. §2.3 walks the alternatives;
§2.4 picks the fix.

### 2.3 Four renderings of the gap

Four options were considered for resolving the gap. Ch 42
walks each and explains why three are rejected.

**Rendering (i): sim-opt owns the replicate loop.** The
rematch fixture constructs a fresh `TaskConfig` per replicate
with the seed baked into the closure via capture:

```rust
for replicate_index in 0..N_INITIAL {
    let seed = splitmix64(MASTER.wrapping_add(replicate_index));
    let task = TaskConfig::builder()
        .name("d2c-sr-rematch")
        .build_fn(move |n_envs| make_training_vecenv(seed, n_envs))
        .build();
    let per_call_result = comp.run_replicates(
        &[task],
        &builders,
        &[seed],
    )?;
    r_cem.extend_from_slice(
        &per_call_result.replicate_best_rewards("d2c-sr-rematch", "CEM")
    );
    r_sa.extend_from_slice(
        &per_call_result.replicate_best_rewards("d2c-sr-rematch", "SA")
    );
}
```

Ten calls to `run_replicates` (one per replicate), each with
a fresh `TaskConfig` whose closure captures the current
replicate's seed, each with a one-element seeds slice. The
outer loop is in sim-opt; `run_replicates` is called as a
single-seed helper.

Advantages: zero upstream change. Ch 41 and the underlying
chassis stay unmodified. The gap is papered over entirely
within Ch 42's scope.

Disadvantages: four, and they compound.

First, the rematch fixture's code is harder to read than Ch
32 §4.8's skeleton. The skeleton passes a 10-element seeds
slice to `run_replicates` and expects the call to return all
20 `RunResult` entries in one shot. Rendering (i) requires a
reader to understand *why* the outer loop exists, which the
fixture's code cannot explain without a comment pointing at
Ch 41's `build_fn` limitation. The reader follows an
archaeological trail from the outer loop back to the Ch 41
API shape and discovers the gap the fixture is working
around. This is the opposite of the R34 chassis-overbuild
philosophy's "the chassis serves the caller" intent.

Second, the `replicate_index` field Ch 41 PR 2a adds to
`RunResult` becomes useless for the rematch. Under rendering
(i), each `run_replicates` call has one seed, so
`replicate_index` is always `0` within a single call. The
rematch fixture has to track its own logical replicate index
via the outer loop variable, which duplicates Ch 41's
responsibility at the call-site level.

Third, future multi-replicate experiments on stochastic
tasks would each have to invent the same workaround. Ch 30's
two pre-committed null follow-ups — richer SA proposal and
Parallel Tempering — are on the same SR task and would each
need a variant of rendering (i)'s outer loop. Ch 42's §5
analysis module is designed to be reusable for those
follow-ups; rendering (i) would force each follow-up to
re-implement the outer loop locally, which is non-reusable
plumbing.

Fourth, the `for_task` / `for_algorithm` / `find_replicate`
helpers on `CompetitionResult` that Ch 41 §2.1 landed cannot
be used on a cross-replicate result under rendering (i),
because the rematch fixture never builds a single
`CompetitionResult` holding all 20 runs — it holds 20
separate `CompetitionResult`s, one per call, each with 2
runs. A helper that takes a cross-replicate view has to be
built at the rematch layer. This is chassis work that Ch 41
already did and Ch 42 would have to redo.

Rendering (i) is rejected on the cumulative weight of these
four disadvantages.

**Rendering (ii): extend `TaskConfig::build_fn` to take a
seed, bundled as a Ch 41 amendment.** The `build_fn` closure
signature changes from `Fn(usize) -> Result<VecEnv, EnvError>`
to `Fn(usize, u64) -> Result<VecEnv, EnvError>`. The seed
flows from `run_replicates`'s loop into `task.build_vec_env(
self.n_envs, seed)` at the inner body, and from there into
the closure. Every existing `TaskConfig` builder updates its
closure to accept the new parameter; stock tasks that do not
use the seed add `_seed: u64` as an ignored parameter.

Advantages: the chassis API matches what a stochastic-physics
rematch needs. Ch 32 §4.8's skeleton renders literally — the
rematch fixture calls `run_replicates(&[task], &builders,
&seeds)` with the 10-element seeds slice, the `TaskConfig`
closure receives each replicate's seed at `build_vec_env` call
time, and the `LangevinThermostat` inside the built `VecEnv`
uses that seed as its `master_seed`. No outer loop, no
per-call result concatenation, no duplicate index tracking.
The `replicate_index` field on `RunResult` is used as
designed — 0 through 9 for the initial batch, 10 through 19
for the expanded batch — and the rematch fixture's code reads
like the Ch 32 skeleton.

The three stock tasks (`reaching_2dof` at `task.rs:362`,
`reaching_6dof` at `task.rs:445`, `obstacle_reaching_6dof`
at `task.rs:619`, all recon-reported) accept-and-ignore the
seed. Each gains one ignored parameter in its builder
closure at `task.rs:396`, `:500`, and `:687` respectively
(recon-reported). `competition.rs`'s test module at
`:422-900` (recon-reported) uses `reaching_2dof()` and
`reaching_6dof()` directly — no mock task type to update —
and `run_replicates`'s existing inner body at
`competition.rs:330` (recon-reported) is the one call site
in the test module that threads the seed through
`build_vec_env`. The stock-task ripple is small — six
source-file sites in `task.rs` (three closures plus three
function-level plumbing updates), one call-site update in
`competition.rs`, and the `TaskConfig`/`TaskConfigBuilder`
type signatures at `task.rs:43`, `:108`, and `:241` — all
mechanical.

Disadvantages: the Ch 41 plan chapter needs an amendment, and
the amendment is larger than the factual-correction patches
that `843dc21c`, `b5cb3f6c`, and `6b876bc5` bundled. Ch 41
§1.1 gains a sentence naming the extension; Ch 41 §2.1 gains
a paragraph rendering the new `build_vec_env` call and the
stock-task ripple; Ch 41's sub-decision table in §5 gains a
new row; Ch 41's review log gets a Round 2 section. The
amendment is still a plan-chapter edit, not a code-level
change, and the actual `task.rs` / `competition.rs` diff
lands when PR 2a ships — but the plan chapter's diff is
larger than prior bundled patches.

Rendering (ii) is the pick. §2.4 defends the call.

**Rendering (iii): sim-opt ships a bespoke `Rematch::run`
function bypassing `Competition::run_replicates` entirely.**
The rematch fixture does not call `run_replicates` at all.
Instead, sim-opt defines a new function
`Rematch::run(task_builder, algorithm_builders, seeds) ->
RematchOutcome` that implements the folded-pilot protocol
from scratch: it loops over replicates, constructs a fresh
`VecEnv` per replicate via `task_builder(seed)`, runs each
algorithm's `train` method directly, builds `RunResult`
values manually, and computes the bootstrap CI and
classification inline.

Advantages: zero upstream change. sim-opt owns the entire
rematch pipeline as a self-contained function.

Disadvantages: this abandons the "use the chassis API that
was purpose-built" discipline Ch 23 and Ch 24 committed to.
The whole point of `Competition::run_replicates` is to be the
general-case replicate entry point; bypassing it for the
rematch means the rematch is a special case, which is exactly
what Ch 23 §1.1 argued against. It also duplicates
`run_replicates`'s body inside sim-opt (the fresh-per-pair
`VecEnv` discipline at `competition.rs:330`, recon-reported;
the inline best-epoch scan at `:363-379`; the provenance
construction at `:381-395`), which is either a copy-paste
from ml-bridge or a second implementation of the same logic.
Either way, the chassis API that Ch 23 and Ch 24 worked out
gets bypassed, and the bypass is purely because of a missing
seed channel on one closure. Rejected.

**Rendering (iv): `AtomicU64` inside the `TaskConfig` closure.**
The closure captures an `Arc<AtomicU64>` counter, reads it on
each `build_vec_env` call, increments it, and uses the read
value as the replicate index for splitmix64 derivation.
Rendering (i)'s outer loop is avoided; the chassis stays
unchanged.

Advantages: zero upstream change, single `run_replicates`
call with 10-element seeds slice.

Disadvantages: the closure now carries hidden mutable state,
and the seed-per-replicate contract is implicit rather than
typed. A reader of the closure has no way to see that the
`AtomicU64` read-increment is happening — they would have to
trace the closure's captures. The rendering hides the very
thing the rematch is trying to make pre-registered and
reproducible (the per-replicate seed). And if the closure is
accidentally called more than once per replicate (for
instance, by a future Ch 41 refactor that retries a failed
`VecEnv` construction), the counter advances in ways the
rematch's analysis cannot account for. Rejected as a hack
that trades compile-time guarantees for runtime correctness.

### 2.4 The pick: rendering (ii)

Rendering (ii) is the pick. The argument has two parts: the
structural case (reasons 1–4, which defend the amendment's
*necessity* as a chassis extension) and the polish case
(reasons 5–6, which defend the pick's *elegance* given the
project's design-discipline preferences).

**Structural case.**

*(1) "Fix gaps before continuing" is a standing user
preference.* The memory entry at `feedback_fix_gaps_before_
continuing.md` says explicitly: "when examples reveal engine
gaps, stop and fix the engine first." Ch 42's drafting is the
first place the rematch's seed-threading need collides with
the `TaskConfig::build_fn` signature, and the collision
reveals a real engine gap — a gap at the boundary between
the chassis and a stochastic-physics use case that the
chassis is supposed to serve. The preference points
unambiguously at the fix-the-engine side. Rendering (i) is
the route-around-the-gap option; the preference rules it out
as a first-order matter.

*(2) "Breaking changes that fix architecture over non-
breaking hacks" is another standing user preference.*
Rendering (i) is a non-breaking hack — sim-opt owns the
replicate loop and Ch 41 stays untouched at the cost of a
workaround outer loop in the rematch fixture. Rendering
(ii) is a breaking change — the `TaskConfig::build_fn`
signature changes, and every caller updates to match. The
preference points at rendering (ii). The breaking change is
narrow (~seven sites, all mechanical) but it is a real API
change, not a papered-over workaround. Combined with (1),
this is a structural argument about the project's
engine-vs-caller relationship, not a stylistic choice.

*(3) Chassis-overbuild philosophy (the R34 argument).* The
memory entry at `feedback_r34_architecture.md` frames the
chassis as the thing that should be overbuilt so the
aftermarket (tests, examples, rematches) can bolt on without
friction. Rendering (ii) extends the chassis to support
stochastic-physics replicate experiments generally; rendering
(i) leaves the chassis unchanged and forces every stochastic-
physics rematch to own its own plumbing. The former is the
R34 architecture; the latter is the opposite. Any future
rematch on a stochastic-physics task — and the thermo-RL
loop vision named in project memory as the north star has
many such future experiments — inherits rendering (ii)'s
chassis feature for free and inherits rendering (i)'s
workaround as copy-paste-able technical debt. The structural
argument favors rendering (ii).

*(4) Future reuse for Ch 30's pre-committed null follow-ups.*
Ch 30's two null follow-ups (richer SA proposal, Parallel
Tempering) are *on the same SR task* and will each want
per-replicate physics seeds. Rendering (ii) gives them the
chassis API directly; rendering (i) forces each to
re-implement the outer loop workaround. The analysis module
Ch 42 §5 ships is designed to be reusable for the follow-
ups, and rendering (ii) makes the reuse actually work
without each follow-up re-inventing the seed-threading
plumbing. This is a structural argument about the rematch's
future expansion path, not a style argument about any
particular fixture's readability.

**Polish case.**

*(5) Readability.* The user's top-priority preference is
"readability and organization." Rendering (ii) makes the
rematch fixture read like Ch 32 §4.8's skeleton — the top-
level call is `run_replicates(tasks, builders, &seeds)`, the
seeds slice carries the per-replicate variation, and the
`LangevinThermostat` picks up the seed from the closure at
build time. A reader coming to the fixture cold sees the
protocol literally. Rendering (i) requires the reader to
understand why the outer loop exists, which needs a comment
pointing back at a missing chassis feature. Readability is
not merely cosmetic in this project's value system (the
memory's first-listed user preference is "readability and
organization are the highest priority"), so this reason
carries more weight than it would in a project that
subordinates readability to other concerns. Still, it is a
style argument rather than a structural one, and the pick
would hold even without it.

*(6) A-grade discipline.* The project's core quality
preference ("A-grade or it doesn't ship") applies to every
Ch 42 artifact, including the rematch fixture. A fixture
whose shape is dictated by a chassis gap is not A-grade; it
is the minimum-diff workaround, and the memory entry at
`feedback_no_allergy_to_bold_choices.md` explicitly warns
against pre-weighting toward minimum-diff options. Rendering
(ii) is the choice an author picks when they are not rushed
and are willing to accept a slightly bigger commit in
exchange for the right shape. Again, this is a discipline
argument rather than a structural one — it names which
rendering is the more disciplined pick, not which one is
necessary.

The structural case alone (reasons 1–4) is sufficient to
reject rendering (i) and pick rendering (ii). The polish
case (reasons 5–6) confirms the pick under the project's
specific readability and A-grade preferences but does not
do load-bearing work on its own. A reader who is unmoved by
the polish case should still be moved by the structural
case.

The counter-argument is that rendering (ii) bundles a larger
Ch 41 amendment than prior post-commit patches. §2.5 names
the exact scope of the amendment and compares it to
`843dc21c`'s footprint.

### 2.5 The amendment's scope

The Ch 41 amendment touches four artifacts: the Ch 41 source
file, the Ch 41 review log, the ml-bridge `task.rs` plan
renderings inside Ch 41, and Ch 41's sub-decision table in §5.
The amendment does *not* touch any Ch 23 or Ch 24 file
(Ch 23's `run_replicates` API shape is unchanged; Ch 24's
aggregation surface is unchanged; only the `TaskConfig::build_
fn` signature extends), and does not touch the `843dc21c`
bundled Ch 24 patch (which is about the `d2c_cem_training.rs`
overclaim, a separate concern).

**Ch 41 §1.1 amendment.** The "replicates API shape" bullet
paragraph beginning "Ch 23 picked `Competition::run_replicates(
tasks, builders, seeds: &[u64])` as the general-case entry
point, returning `Result<CompetitionResult, EnvError>`" at
`41-pr-2-competition-replicates.md:77-82` (recon-reported)
gains a continuation sentence naming the `build_vec_env`
seed-threading need. The sentence: "Ch 42
surfaced a gap in the original `run_replicates` rendering —
`task.build_vec_env(self.n_envs)` was seed-blind, which meant
stochastic-physics tasks could not vary their physics noise
sequence per replicate — and this chapter's updated rendering
of PR 2a extends `TaskConfig::build_fn` to take a second `u64`
seed parameter and threads it through at the
`build_vec_env` call site." Roughly two sentences added,
scoped to name the extension as part of PR 2a's revised plan.

**Ch 41 §2.1 amendment.** The code block at
`41-pr-2-competition-replicates.md:458-489` (recon-reported,
the `run_replicates` body skeleton) updates its inner body
from `let mut env = task.build_vec_env(self.n_envs)?;` to
`let mut env = task.build_vec_env(self.n_envs, seed)?;`. The
paragraph preceding the code block gains a sentence naming
the extension. A new paragraph after the code block renders
the `TaskConfig::build_fn` signature change and the stock-
task ripple:

- `task.rs:43` (recon-reported) — `build_fn: Arc<dyn Fn(usize)
  -> Result<VecEnv, EnvError> + Send + Sync>` becomes
  `build_fn: Arc<dyn Fn(usize, u64) -> Result<VecEnv, EnvError>
  + Send + Sync>`.
- `task.rs:108` (recon-reported) — `TaskConfig::build_vec_env(
  &self, n_envs: usize)` becomes `TaskConfig::build_vec_env(
  &self, n_envs: usize, seed: u64)`, and its body
  `(self.build_fn)(n_envs)` at `task.rs:109` becomes
  `(self.build_fn)(n_envs, seed)`.
- `task.rs:241` (recon-reported) — the `TaskConfigBuilder`'s
  own internal `build_fn` plumbing closure's type at the
  `let build_fn = Arc::new(move |n_envs: usize| ...)`
  declaration is updated to the two-argument form.
- `task.rs:362-444` (recon-reported, `pub fn reaching_2dof() ->
  TaskConfig` fn def at `:362`, closure declared at `:396`) —
  the `build_fn` closure gains an ignored `_seed: u64`
  parameter.
- `task.rs:445-617` (recon-reported, `pub fn reaching_6dof() ->
  TaskConfig` fn def at `:445`, closure declared at `:500`) —
  same closure signature update.
- `task.rs:619-~725` (recon-reported, `pub fn
  obstacle_reaching_6dof() -> TaskConfig` fn def at `:619`,
  closure declared at `:687`) — same closure signature update.
- `competition.rs:330` (recon-reported) — `let mut env =
  task.build_vec_env(self.n_envs)?;` becomes `let mut env =
  task.build_vec_env(self.n_envs, seed)?;` inside the
  `run_replicates` body.
- `sim/L0/ml-bridge/src/vec_env.rs:391` (recon-reported, the
  `BatchSim::new(Arc::clone(&self.model), self.n_envs)` call
  inside `VecEnvBuilder::build()`) is unchanged. The seed
  does not flow through `VecEnv::builder` at all — the closure
  inside each `TaskConfig`'s `build_fn` captures the seed and
  uses it in the `LangevinThermostat` construction *before*
  calling `VecEnv::builder.build()`. The rematch fixture's
  `make_training_vecenv(master_seed, n_envs)` helper at §6.2
  is the closure site that carries the seed.

Approximately seven source-file sites in ml-bridge (three
stock-task closures, the `TaskConfigBuilder` internal
plumbing, `build_fn` field type, `build_vec_env` method, and
`competition.rs`'s inner-loop `build_vec_env` call), plus the
plan-chapter rendering of each. The total added text in
Ch 41 §2.1 is roughly 40 to 60 lines — a bulleted list of the
ripple sites plus the paragraph naming the rationale.

**Ch 41 §5 sub-decision table amendment.** Ch 41 §5's table
gains a new row (g) naming the extension as a sub-decision:

| (g) | `TaskConfig::build_fn` seed extension | extend signature with a `u64` seed parameter; thread through `run_replicates`'s `build_vec_env` call; stock tasks accept-and-ignore | Added during Ch 42 drafting after recon surfaced the gap at the rematch layer; amendment bundled into Ch 42 commit per the `843dc21c` precedent |

One row added; the existing six rows unchanged. Ch 41's
sub-decision count goes from six to seven.

**Ch 41 review log amendment.** Ch 41's review log at
`docs/studies/ml_chassis_refactor/review_logs/41-pr-2-
competition-replicates.review.md` gains a Round 2 section
similar in shape to Ch 24's Round 2 from `843dc21c`. The
section names:

- The source of the finding (Ch 42 §2 recon).
- The evidence (the gap between Ch 32 §4.8's skeleton and
  Ch 41's rendered `run_replicates` body for stochastic
  tasks).
- The amendment (the four source-file updates above plus
  the chapter edits).
- The load-bearing verdict (the amendment is a scope
  extension, not a decision revision; Ch 41's existing
  decisions are unchanged, the additive-vs-semantic split,
  the CEM rewrite form, the TD3/SAC pre-loop shape, the
  REINFORCE/PPO zero-fallback cleanup, the CEM rename, and
  the d2c_cem_training.rs doc-only scope are all preserved).

Approximately 15 to 20 lines added to the review log.

**Total amendment footprint.** Ch 41 source file: ~60 lines
added across §1.1, §2.1, and §5. Ch 41 review log: ~20 lines
added. Total: ~80 lines. This is *smaller* than `843dc21c`'s
Ch 24 patch footprint (which amended Ch 24 §1.9, §2.2, and
§5 plus the review log for a total of ~120 lines across the
four artifacts). The amendment is a narrow patch in the same
precedent family as the prior bundled amendments, not a
larger-scale revision.

### 2.6 The bundled-commit case

The amendment could in principle land as a separate commit
after Ch 42's own commit — a three-commit sequence of
"Ch 42 drafts", then "Ch 41 amendment", then "amendment
reflected in Ch 42 §2's cross-references". This is what the
first drafts of `3e1ec0ff` and `6b876bc5` used.

Ch 42 bundles the amendment with its own commit instead,
following the `b5cb3f6c` and `843dc21c` precedents that
established the bundled pattern for cases where the
amendment is load-bearing for the downstream chapter's
argument. Three reasons:

**First, §2 of this chapter depends on the amendment for
the §2.3 rendering (ii) argument to make sense.** A reader
who sees Ch 42's §2 without the amendment sees the
rendering (ii) pick as aspirational (the Ch 41 chapter has
not been updated to match) and has to chase a later commit
to verify the actual plan. Bundling puts the amendment and
the downstream rendering in the same reviewable unit.

**Second, PR 2a's implementation depends on the amendment
for the signature.** A reader implementing PR 2a from Ch
41's plan *without* the amendment would ship `build_fn:
Fn(usize)`, which Ch 42's PR 3b cannot consume. The error
would be caught at PR 3b's review time, which is too late —
PR 2a would already have landed with the wrong signature
and a separate follow-up PR would be needed to extend it.
Bundling the amendment into the Ch 42 commit means PR 2a's
plan is correct as soon as Ch 42 commits, and PR 2a can
ship the right signature on its first landing.

**Third, the `843dc21c` precedent is the most recent bundled
amendment and is the pattern Ch 42 explicitly follows.** The
session-9 commit message names the bundled amendment as a
three-site patch; Ch 42's commit message will name this
amendment similarly. The review log for Ch 42 will cite the
amendment explicitly so a reader of the Ch 42 commit sees
the scope clearly.

The alternative — a separate commit — would be acceptable
if the amendment were purely factual (the `3e1ec0ff`
pattern), but rendering (ii) is a scope extension the
downstream chapter *argues* and the upstream chapter needs to
reflect. The bundling matches the `843dc21c` / `b5cb3f6c`
pattern for argument-bearing amendments.

### 2.7 The amendment's interaction with `843dc21c`'s Ch 24 patch

`843dc21c` bundled a Ch 24 §1.9 / §2.2 / §5 patch correcting
the factually-wrong "the D2c test compares `best_reward()`
values across algorithms" overclaim. The patch narrowed Ch
41's `d2c_cem_training.rs` scope to doc-only updates and
deferred the structural rewrite to Ch 42's PR 3's "new test
fixture under `sim-opt/tests/`."

Ch 42's §6 is that new test fixture, and it sits in
`sim-opt/tests/d2c_sr_rematch.rs`. The `843dc21c` patch's
deferral pointer lands exactly where Ch 42 plans it — the
rematch is implemented in a greenfield test file under the
new `sim-opt` crate, not as a rewrite of the legacy
`d2c_cem_training.rs`. The Ch 42 rematch fixture and the
`d2c_cem_training.rs` legacy file coexist: the legacy file
continues to document the D2c experiment's single-seed
training runs; the rematch fixture documents the multi-seed
statistical comparison.

The two amendments (the `843dc21c` Ch 24 patch and this
chapter's §2 Ch 41 patch) do not interact at the source-file
level — `843dc21c`'s patch was entirely in Ch 24 and did not
touch Ch 41, and this chapter's patch is entirely in Ch 41
and does not touch Ch 24. They are sequential precedents for
the bundled-amendment pattern, not overlapping edits.

## Section 3 — The `sim-opt` crate (sub-decision (b))

### 3.1 The crate's `Cargo.toml`

The `sim-opt` crate lives at `sim/L0/opt/` as a peer of
`sim/L0/ml-bridge/` and `sim/L0/thermostat/` in the L0
Bevy-free layer. The workspace `Cargo.toml` at
`Cargo.toml:296-306` (recon-reported, the `# Simulation
domain - Layer 0 (Bevy-free)` section spanning the header
comment through the closing blank line before the L1 section)
gains one new member line, `"sim/L0/opt"`, and one new
workspace dependency entry, `sim-opt = { path = "sim/L0/opt"
}`, placed in declaration order after `sim/L0/thermostat` and
`sim-thermostat` respectively. The L0 members are ordered
topologically (foundational crates first, then domain
crates), not alphabetically; `sim-opt` joins at the end as
the newest L0 crate.

The new crate's `Cargo.toml` at `sim/L0/opt/Cargo.toml`:

```toml
[package]
name = "sim-opt"
description = "Gradient-free optimization algorithms and rematch analysis machinery"
version.workspace = true
edition.workspace = true
license.workspace = true
repository.workspace = true
authors.workspace = true
rust-version.workspace = true

[dependencies]
rand = { workspace = true }
serde = { workspace = true }
sim-ml-bridge = { workspace = true }
sim-thermostat = { workspace = true }
thiserror = { workspace = true }

[dev-dependencies]
approx = { workspace = true }
sim-core = { workspace = true }
sim-mjcf = { workspace = true }

[lints]
workspace = true
```

Seven direct runtime dependencies, three dev-dependencies,
workspace-inherited package metadata and lints. Each
dependency is justified at §3.3 below.

### 3.2 Module structure

The crate's `src/` tree:

```
sim/L0/opt/
├── Cargo.toml
├── src/
│   ├── lib.rs           // crate root, pub re-exports
│   ├── algorithm.rs     // Sa, SaHyperparams, Algorithm impl
│   └── analysis.rs      // bootstrap, bimodality, classify, run_rematch
└── tests/
    └── d2c_sr_rematch.rs  // PR 3b's rematch test fixture
```

Four files total at the source layer, two pub modules from
the crate root. `lib.rs` is the crate root and contains
only the `pub mod algorithm;` / `pub mod analysis;`
declarations plus `pub use` re-exports for the types the
rematch fixture (and future consumers) call directly.

**`lib.rs` skeleton.**

```rust
//! # sim-opt
//!
//! Gradient-free optimization algorithms and rematch analysis
//! machinery.
//!
//! This crate is **Layer 0** — zero Bevy, zero ML framework
//! dependencies. It extends `sim-ml-bridge`'s `Algorithm` trait
//! with Simulated Annealing and ships the statistical-analysis
//! machinery the ml-chassis-refactor study's rematch consumes.
//!
//! ## Scope
//!
//! - [`Sa`] / [`SaHyperparams`] — Simulated Annealing implemented
//!   as an `Algorithm` trait impl. Consumes `Policy` and
//!   `VecEnv` directly, like CEM, and emits per-epoch
//!   `EpochMetrics` in the per-episode-total unit the ml-bridge
//!   algorithms standardized on.
//! - [`analysis`] — bootstrap CI on the difference of means
//!   and medians, bimodality coefficient, Ch 30 three-outcome
//!   classifier, and the folded-pilot driver that executes
//!   Chapter 32's rematch protocol end-to-end.
//!
//! ## What this crate does NOT do
//!
//! - **No gradient-based algorithms.** Those live in
//!   `sim-ml-bridge` alongside CEM, REINFORCE, PPO, TD3, and
//!   SAC. `sim-opt` is specifically the gradient-free branch.
//! - **No policy or network implementations.** Policies come
//!   from `sim-ml-bridge::LinearPolicy` (or `MlpPolicy`, etc.)
//!   and are passed into `Sa::new` at construction time.
//! - **No environment construction.** `VecEnv` instances come
//!   from `sim-ml-bridge::TaskConfig::build_vec_env(n_envs,
//!   seed)`, which sim-opt's analysis module calls via the
//!   rematch driver.
//! - **No Bevy dependency.** This is Layer 0.

#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod algorithm;
pub mod analysis;

pub use algorithm::{Sa, SaHyperparams};
pub use analysis::{
    BootstrapCi, RematchOutcome, bimodality_coefficient,
    bootstrap_diff_means, bootstrap_diff_medians,
    classify_outcome, run_rematch,
};
```

The `deny(unwrap_used, expect_used)` lint matches ml-bridge's
pattern at `sim/L0/ml-bridge/src/lib.rs:70-71` (recon-
reported) — library code denies, test code is allowed to
unwrap.

### 3.3 Dependency direction and justification

Each runtime and dev dependency has a load-bearing reason.
Ch 42 names each here so PR 3a's reviewer has a checklist.

**`rand = { workspace = true }`.** Runtime dependency. SA's
`train` method uses `StdRng::seed_from_u64(seed)` for
proposal generation, matching the pattern Cem's `train` uses
at `sim/L0/ml-bridge/src/cem.rs:139` (recon-reported). SA's
Gaussian proposal uses an inlined Box-Muller helper (local
`randn` function in `algorithm.rs`), matching CEM's
`randn` at `cem.rs:115` (recon-reported), which avoids a
`rand_distr` dependency entirely. The analysis module's
bootstrap functions also use a `&mut impl Rng` parameter for
resample draws. `rand` is at workspace dependencies at
`Cargo.toml:381` (recon-reported).

**`serde = { workspace = true }`.** Runtime dependency. SA's
`EpochMetrics::extra` uses `serde`-serializable floats, and
SA's checkpoint structure (implementing the `Algorithm`
trait's `checkpoint` method) serializes via `serde`. `serde`
is at `Cargo.toml:367` (recon-reported).

**`sim-ml-bridge = { workspace = true }`.** Runtime
dependency — the load-bearing one. sim-opt consumes from
ml-bridge:

- `Algorithm`, `EpochMetrics`, `TrainingBudget` — the trait
  SA implements and the types it produces, at
  `ml-bridge/src/algorithm.rs:77-120` (recon-reported).
- `Policy` — the trait SA's policy argument type references,
  at `ml-bridge/src/policy.rs:30` (recon-reported, via the
  `pub use` at `ml-bridge/src/lib.rs:120`).
- `LinearPolicy` — the concrete policy the rematch fixture
  constructs for both CEM and SA, at
  `ml-bridge/src/linear.rs:38-68` (recon-reported, struct
  def at `:38` through `::new` body closure at `:68`).
- `VecEnv` — SA's `train` method parameter, at
  `ml-bridge/src/vec_env.rs:76` (recon-reported, via the
  `pub use` at `lib.rs:135`).
- `collect_episodic_rollout`, `EpisodicRollout`, `Trajectory`
  — the rollout helper SA uses for multi-env fitness
  evaluation, at `ml-bridge/src/rollout.rs:82-...` (recon-
  reported).
- `PolicyArtifact`, `ArtifactError`, `TrainingCheckpoint` —
  the types SA's `policy_artifact` / `best_artifact` /
  `checkpoint` methods return, at
  `ml-bridge/src/artifact.rs` (recon-reported, via the
  `pub use` at `lib.rs:102-105`).
- `Competition`, `CompetitionResult`, `RunResult`,
  `TaskConfig` — the types the analysis module's
  `run_rematch` driver consumes, at
  `ml-bridge/src/competition.rs` and `ml-bridge/src/task.rs`
  (recon-reported).
- `SeedSummary` — the aggregation struct Ch 41 PR 2a ships,
  consumed optionally by `run_rematch` for human-facing
  summary formatting. (Not strictly required, but available
  for rematch-writeup-authors who want the three-field
  summary alongside the CI.)

The dependency direction is clean: sim-opt → ml-bridge. The
reverse direction does not exist and would not make sense —
ml-bridge has no reason to know about SA or about the
rematch's analysis machinery.

**`sim-thermostat = { workspace = true }`.** Runtime
dependency. sim-opt's analysis module uses `sim_thermostat::
prf::splitmix64` (shipped by PR 1a) to derive per-replicate
seeds from the pre-registered `MASTER` literal. The
`splitmix64` function is the only thermostat-crate item
sim-opt consumes at the runtime layer; the dependency is
thin but genuine.

**`thiserror = { workspace = true }`.** Runtime dependency.
sim-opt's error types (for SA's construction errors and
for rematch driver errors that wrap `EnvError` from
ml-bridge) derive `thiserror::Error`, matching the workspace
pattern.

**`approx = { workspace = true }` (dev-dependency).** Used
in the analysis module's unit tests for near-equal float
assertions on bootstrap CI bounds and bimodality coefficient
values, matching the pattern ml-bridge's tests use at
`ml-bridge/Cargo.toml:25` (recon-reported).

**`sim-core = { workspace = true }` (dev-dependency).** The
rematch test fixture at `sim-opt/tests/d2c_sr_rematch.rs`
uses `sim_core::DVector` for the `LangevinThermostat`'s
gamma vector argument (matching the pattern at
`sim/L0/thermostat/tests/d2c_cem_training.rs:91`, recon-
reported: `DVector::from_element(model.nv, GAMMA)`). `sim-core`
is not a runtime dependency because sim-opt's library code
does not touch `Model` / `Data` / `DVector` directly — only
the test fixture does.

**`sim-mjcf = { workspace = true }` (dev-dependency).** The
rematch test fixture uses `sim_mjcf::load_model(SR_XML)` to
parse the D2c SR task's MuJoCo XML, matching the pattern at
`d2c_cem_training.rs:87` (recon-reported). Same rationale as
`sim-core`: runtime sim-opt code does not touch MJCF, only
the test fixture does.

### 3.4 Why `sim/L0/opt/` and not a sub-module of ml-bridge

A reader could reasonably ask why SA is not a new module
inside `sim-ml-bridge` alongside `cem.rs`, `reinforce.rs`,
`ppo.rs`, `td3.rs`, and `sac.rs`. The five existing ml-bridge
algorithms live together; a sixth would match the pattern.

The argument for a new crate rather than a sub-module has
three parts.

**First, Ch 23 and Ch 32 explicitly name `sim-opt`.** Ch 32
§7's scope-discipline section says "Ch 42 (Part 4 PR 3,
sim-opt split and rematch) owns the implementation." Ch 31
§4.3 says "Ch 42 (Part 4 PR 3) is the execution PR that
lands SA in `sim-opt` and wires it into `Competition`." The
crate name is already locked by the upstream chapters. This
alone would be sufficient.

**Second, the physics-aware ML pivot spec's three-crate
split.** Project memory's `project_sim_ml_pivot.md` describes
a target architecture of `sim-ml-chassis + sim-rl + sim-opt`,
separating the chassis from the RL algorithms and from the
gradient-free optimization side. The pivot spec has not been
enacted in Parts 1–3 (ml-bridge has remained monolithic), and
Ch 42 does not execute the full pivot. But introducing
`sim-opt` as the first of the three pivot crates is the
minimum step that begins to materialize the pivot's intended
architecture without requiring a full ml-bridge reorganization
in PR 3. The pivot's `sim-opt` is the gradient-free branch;
SA is the first inhabitant; the analysis module is the
rematch infrastructure. A future PR could split ml-bridge
further, but PR 3 only opens the `sim-opt` crate.

**Third, the analysis module's natural home is with SA, not
with ml-bridge.** The rematch's bootstrap / bimodality /
classify functions are *about the rematch*, not about the
general ml-bridge algorithm surface. Putting them inside
ml-bridge (e.g., at `ml-bridge/src/rematch.rs`) would conflate
"chassis code ml-bridge ships" with "rematch-specific
analysis Ch 42 ships." The code review surface for ml-bridge
should be about the ML chassis; the rematch analysis is
downstream of that chassis and benefits from living at its
own level.

The counter-argument — "a new crate for essentially one
struct is overbuild" — is answered by two observations.
First, `sim-opt` ships two substantial modules (SA plus the
analysis machinery), not one. Second, the crate has a clear
future expansion path under Ch 30's null follow-ups: a
richer-proposal SA variant (Ch 30's (a) follow-up) and a
Parallel Tempering algorithm (Ch 30's (b) follow-up) would
both live in `sim-opt` alongside the initial SA
implementation, and `sim-opt` would have three to four
algorithms over the next few study revisions. The crate is
greenfield chassis for the gradient-free branch, not a
placeholder for one struct.

### 3.5 L0 placement and the layering discipline

The `sim/L0/` tree holds Bevy-free crates. `sim-opt` has no
Bevy dependency, no GPU dependency, and no rendering
dependency — it is pure computation over the ml-bridge
chassis. L0 is the right placement. The L0/L1 layering
matches the project's "Layer 0 (Bevy-free) simulation crates"
framing in the workspace `Cargo.toml` at line 296 (recon-
reported) and in the memory's codebase structure entry.

A future visualization of the rematch's results — e.g., a
Bevy-based plot of the bootstrap distribution or the
per-replicate policy trajectories — would live in a separate
L1 crate (perhaps `sim-bevy-rematch-viz`) that depends on
`sim-opt` as its data source. Ch 42 does not plan such a
crate; it is purely a forward compatibility note.

## Section 4 — The SA implementation (sub-decisions (c)–(e))

### 4.1 `Sa` struct and `SaHyperparams`

SA implements the `Algorithm` trait from `ml-bridge/src/
algorithm.rs:77-120` (recon-reported), matching the shape the
five existing ml-bridge algorithms use. The struct at
`sim/L0/opt/src/algorithm.rs`:

```rust
use std::collections::BTreeMap;
use std::time::Instant;

use rand::SeedableRng;
use rand::rngs::StdRng;

use sim_ml_bridge::{
    Algorithm, ArtifactError, EpochMetrics, PolicyArtifact,
    Policy, TrainingBudget, TrainingCheckpoint, VecEnv,
    collect_episodic_rollout,
};

/// Simulated Annealing hyperparameters.
///
/// SA evaluates one candidate policy per epoch (the current
/// best policy perturbed by a Gaussian proposal), averages its
/// reward across `n_envs` parallel episodes, and accepts or
/// rejects the proposal under a Metropolis criterion with a
/// geometric cooling temperature.
#[derive(Debug, Clone, Copy)]
pub struct SaHyperparams {
    /// Initial temperature for the Metropolis accept/reject.
    /// The reward scale of the task sets the right order of
    /// magnitude; see §4.5 for the default's defense.
    pub initial_temperature: f64,
    /// Standard deviation of the Gaussian proposal, applied
    /// element-wise to the current policy's parameter vector.
    pub proposal_std: f64,
    /// Multiplicative cooling decay applied to the temperature
    /// each epoch: `T_{k+1} = T_k * cooling_decay`. Geometric
    /// schedule. See §4.4 for the default's defense.
    pub cooling_decay: f64,
    /// Floor on the temperature (prevents degenerate
    /// accept-nothing behavior at very late epochs).
    pub temperature_min: f64,
    /// Maximum environment steps per episode. Matches CEM's
    /// `CemHyperparams::max_episode_steps` at
    /// `ml-bridge/src/cem.rs:38` (recon-reported) — the rollout
    /// helper is the same, and the budget-to-epochs formula is
    /// the same, so the value has to match for compute parity.
    pub max_episode_steps: usize,
}

/// Simulated Annealing algorithm.
///
/// # Parts
///
/// - [`Policy`] — base trait only. SA perturbs `params()`
///   directly and applies Metropolis accept/reject; no gradients
///   needed. This is the same trait-bound shape CEM uses.
///
/// # Constructor
///
/// ```ignore
/// let sa = Sa::new(
///     Box::new(LinearPolicy::new(obs_dim, act_dim, &obs_scale)),
///     SaHyperparams {
///         initial_temperature: 0.5,
///         proposal_std: 0.5,
///         cooling_decay: 0.955,
///         temperature_min: 0.005,
///         max_episode_steps: 5000,
///     },
/// );
/// ```
pub struct Sa {
    policy: Box<dyn Policy>,
    hyperparams: SaHyperparams,
    /// Current accepted params — the state of the Metropolis
    /// chain. Starts at the policy's initial params and moves
    /// with each accepted proposal.
    current_params: Vec<f64>,
    /// Current accepted fitness — the mean per-episode total
    /// reward across `n_envs` trajectories at `current_params`,
    /// measured in the Ch 24 Decision 1 unit.
    current_fitness: f64,
    /// Current temperature (decayed each epoch).
    temperature: f64,
    /// Best-seen params and fitness, separate from the current
    /// Metropolis state. SA's best is monotone: once seen, it
    /// cannot regress.
    best_params: Vec<f64>,
    best_fitness: f64,
    best_epoch: usize,
}
```

Five fields on `SaHyperparams`, eight fields on `Sa`. The
struct shape matches CEM's at `ml-bridge/src/cem.rs:58-65`
(recon-reported) in spirit: a boxed policy, a hyperparams
copy, a running state variable (temperature instead of
noise_std), and a best-tracker (inline instead of
`BestTracker` because `BestTracker` is `pub(crate)` at
`ml-bridge/src/lib.rs:79`, recon-reported, and not available
across crate boundaries).

The inline best-tracking is a genuine duplication — the
CEM struct holds a `best: BestTracker` and SA holds
`best_params: Vec<f64> + best_fitness: f64 + best_epoch:
usize` directly. Ch 42 does not introduce a `pub` version of
`BestTracker` or move it to a shared module; the duplication
is small (three fields plus inline update logic, ~15 lines
in `Sa::new` and the train loop) and the alternative (a
shared `pub mod best_tracker` in ml-bridge) would be
speculative chassis growth. §9 names the shared-best-tracker
question as deferred-but-recognized.

**`Sa::new` and `Sa::from_checkpoint`.**

```rust
impl Sa {
    /// Create a new SA instance with the policy's initial
    /// params as the Metropolis chain's starting point.
    #[must_use]
    pub fn new(
        policy: Box<dyn Policy>,
        hyperparams: SaHyperparams,
    ) -> Self {
        let current_params = policy.params().to_vec();
        let best_params = current_params.clone();
        Self {
            policy,
            hyperparams,
            current_params,
            current_fitness: f64::NEG_INFINITY,
            temperature: hyperparams.initial_temperature,
            best_params,
            best_fitness: f64::NEG_INFINITY,
            best_epoch: 0,
        }
    }

    /// Reconstruct an SA instance from a checkpoint.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the policy cannot be
    /// reconstructed from the checkpoint's policy_artifact.
    pub fn from_checkpoint(
        checkpoint: &TrainingCheckpoint,
        hyperparams: SaHyperparams,
    ) -> Result<Self, ArtifactError> {
        let policy = checkpoint.policy_artifact.to_policy()?;
        let temperature = checkpoint
            .algorithm_state
            .get("temperature")
            .copied()
            .unwrap_or(hyperparams.initial_temperature);
        let current_fitness = checkpoint
            .algorithm_state
            .get("current_fitness")
            .copied()
            .unwrap_or(f64::NEG_INFINITY);
        let best_params = checkpoint
            .best_params
            .clone()
            .unwrap_or_else(|| policy.params().to_vec());
        let best_fitness = checkpoint.best_reward.unwrap_or(f64::NEG_INFINITY);
        let best_epoch = checkpoint.best_epoch;
        let current_params = policy.params().to_vec();
        Ok(Self {
            policy,
            hyperparams,
            current_params,
            current_fitness,
            temperature,
            best_params,
            best_fitness,
            best_epoch,
        })
    }
}
```

The checkpoint round-trip matches CEM's pattern at
`ml-bridge/src/cem.rs:89-115` (recon-reported): restore the
policy, read the algorithm-specific state (temperature and
current_fitness for SA, noise_std for CEM), restore the
best tracker, and return the reconstructed instance.

### 4.2 The `train` loop

SA's `train` method is the load-bearing piece of the
implementation. It follows the Algorithm trait's contract
(take `&mut VecEnv`, `TrainingBudget`, `seed`, `on_epoch`
callback; return `Vec<EpochMetrics>`) and implements the
Metropolis accept/reject rule with multi-env fitness
averaging.

```rust
impl Algorithm for Sa {
    fn name(&self) -> &'static str {
        "SA"
    }

    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
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
        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(s) => {
                s / (n_envs * self.hyperparams.max_episode_steps).max(1)
            }
        };
        let hp = self.hyperparams;

        let mut metrics = Vec::with_capacity(n_epochs);

        // Note: no baseline evaluation outside the loop. SA's
        // first iteration's Metropolis comparison is against
        // self.current_fitness = f64::NEG_INFINITY, which forces
        // accept on the first proposal (delta = proposed -
        // (-inf) = +inf, always accepted). This keeps SA's
        // total env-step count at exactly n_envs *
        // max_episode_steps * n_epochs (= 16M for the rematch
        // config), matching CEM's budget formula exactly. The
        // initial policy's fitness is never evaluated directly;
        // the first *accepted* state is the perturbed proposal,
        // which becomes the chain's baseline.

        for epoch in 0..n_epochs {
            let t0 = Instant::now();

            // 1. Propose: perturb current_params with a Gaussian
            //    of standard deviation proposal_std, element-wise.
            //    Uses the same inlined Box-Muller helper as CEM
            //    at ml-bridge/src/cem.rs:115 (recon-reported) to
            //    avoid a rand_distr dependency on sim-opt.
            let proposed_params: Vec<f64> = self
                .current_params
                .iter()
                .map(|&p| hp.proposal_std.mul_add(randn(&mut rng), p))
                .collect();

            // 2. Evaluate: run n_envs episodes with the proposed
            //    params and average the per-episode total reward.
            let proposed_fitness = evaluate_fitness(
                env,
                &mut *self.policy,
                &proposed_params,
                hp.max_episode_steps,
            );

            // 3. Metropolis accept/reject. Accept if the
            //    proposal is strictly better, or with probability
            //    exp((proposed - current) / T) if worse.
            let delta = proposed_fitness - self.current_fitness;
            let accept = if delta > 0.0 {
                true
            } else if self.temperature > 0.0 {
                let accept_prob = (delta / self.temperature).exp();
                rng.gen::<f64>() < accept_prob
            } else {
                false
            };

            let accepted_count = if accept { 1 } else { 0 };

            if accept {
                self.current_params = proposed_params;
                self.current_fitness = proposed_fitness;
            }

            // 4. Best-tracking. Strict `>` matches
            //    BestTracker's convention at
            //    `ml-bridge/src/best_tracker.rs` (recon-
            //    reported): ties keep the earlier epoch.
            if self.current_fitness > self.best_fitness {
                self.best_fitness = self.current_fitness;
                self.best_params.copy_from_slice(&self.current_params);
                self.best_epoch = epoch;
            }

            // 5. Cool the temperature (geometric schedule).
            self.temperature =
                (self.temperature * hp.cooling_decay).max(hp.temperature_min);

            // 6. Emit per-epoch metrics in the Ch 24 Decision 1
            //    unit (mean per-episode total across n_envs).
            //    SA's mean_reward is the *current* fitness at
            //    this epoch (the accepted chain state), not the
            //    proposed fitness — the chain's state is what
            //    the algorithm is committed to.
            let mut extra = BTreeMap::new();
            extra.insert("temperature".into(), self.temperature);
            extra.insert("accepted".into(), accepted_count as f64);
            extra.insert("proposed_fitness".into(), proposed_fitness);

            let em = EpochMetrics {
                epoch,
                mean_reward: self.current_fitness,
                done_count: 0, // filled by evaluate_fitness in a future revision
                total_steps: n_envs * hp.max_episode_steps,
                wall_time_ms: t0.elapsed().as_millis() as u64,
                extra,
            };
            on_epoch(&em);
            metrics.push(em);

            // 7. Sync the policy to the accepted chain state so
            //    the next iteration's forward passes reflect it.
            //    (The forward pass inside evaluate_fitness uses
            //    set_params from proposed_params, which overwrites
            //    whatever was there, so this sync is mostly for
            //    post-train policy_artifact() calls.)
            self.policy.set_params(&self.current_params);
        }

        metrics
    }

    fn policy_artifact(&self) -> PolicyArtifact {
        let mut policy = self.policy_clone_with_params(&self.current_params);
        PolicyArtifact::from_policy(&*policy)
    }

    fn best_artifact(&self) -> PolicyArtifact {
        let policy = self.policy_clone_with_params(&self.best_params);
        PolicyArtifact::from_policy(&*policy)
    }

    fn checkpoint(&self) -> TrainingCheckpoint {
        TrainingCheckpoint {
            algorithm_name: "SA".into(),
            policy_artifact: self.policy_artifact(),
            critics: vec![],
            optimizer_states: vec![],
            algorithm_state: BTreeMap::from([
                ("temperature".into(), self.temperature),
                ("current_fitness".into(), self.current_fitness),
            ]),
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

/// Box-Muller normal sample. Inlined to avoid a rand_distr
/// dependency and match the CEM pattern at
/// `ml-bridge/src/cem.rs:115` (recon-reported).
fn randn(rng: &mut impl rand::Rng) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

/// Evaluate a candidate parameter vector by running `n_envs`
/// parallel episodes and averaging the per-episode total
/// reward. The unit matches the Ch 24 Decision 1
/// standardization — see `ml-bridge/src/cem.rs:209` for the
/// CEM analogue post-PR-2b.
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
    let epoch_rewards: Vec<f64> = rollout
        .trajectories
        .iter()
        .map(|traj| traj.rewards.iter().sum::<f64>())
        .collect();
    epoch_rewards.iter().sum::<f64>() / n_envs as f64
}
```

The `train` method is 80 lines of body plus the
`evaluate_fitness` helper at ~25 lines. Comparable in size to
CEM's `train` at `ml-bridge/src/cem.rs:132-234` (recon-
reported), which is ~100 lines of body.

One implementation note worth flagging at the rendering
level: the `policy_clone_with_params` helper is a small
utility that clones the policy's current shape and overrides
its params to the given vector, without mutating `self.policy`.
It is a ~5-line helper defined in `algorithm.rs` below the
`impl Algorithm for Sa` block:

```rust
impl Sa {
    fn policy_clone_with_params(&self, params: &[f64]) -> Box<dyn Policy> {
        // The existing Policy trait does not support a clone
        // method; we reconstruct the policy from its descriptor
        // + params via the same from_descriptor path
        // PolicyArtifact::to_policy uses. This is a modest
        // workaround for the trait shape and is flagged in §9
        // as a future-cleanup item.
        let descriptor = self.policy.descriptor();
        let mut rebuilt = PolicyArtifact::bare(descriptor, params.to_vec())
            .to_policy()
            .expect("policy rebuild must succeed for same-shape params");
        rebuilt
    }
}
```

This works because `PolicyArtifact::to_policy` at
`ml-bridge/src/artifact.rs:400` (recon-reported via the
`pub use` at `lib.rs:102-105`) rebuilds a concrete policy
from a descriptor and a parameter vector, and the descriptor
is available via `self.policy.descriptor()`. The round-trip
is mildly wasteful (allocating a new policy each time
`policy_artifact` or `best_artifact` is called), but the
calls happen at most twice per `train` invocation, so the
overhead is negligible. A cleaner shape would be a `Policy::
clone_with_params` method on the trait, which is a future-
cleanup item §9 names.

The expect call is allowed here because `expect_used` is
denied at the crate root — the code that calls expect is
inside a helper where the invariant (same-shape params
always rebuild successfully) is load-bearing. The workspace
lint at `Cargo.toml:336` sets `expect_used = "warn"` and the
crate-level `#![deny(clippy::expect_used)]` would fail this.
PR 3a's implementation either (a) uses an explicit panic
with a clear message, or (b) propagates the error via
`Result<Box<dyn Policy>, ArtifactError>` and the callers
unwrap at the trait-method boundary where `Algorithm::
policy_artifact` is `-> PolicyArtifact` (infallible). The
cleanest rendering is (a): replace `.expect(...)` with an
explicit `unreachable!()` on the impossible error branch
after asserting the descriptor matches. PR 3a's reviewer
should prefer (a) — the descriptor equality is a real
invariant, not a "probably won't fail" hope.

### 4.3 `mean_reward` in the Ch 24 Decision 1 unit

SA's `mean_reward` emitted per epoch is the `current_fitness`
value — the mean per-episode total reward across `n_envs`
trajectories at the chain's currently accepted parameters,
in the Ch 24 Decision 1 unit. The formula:

```rust
mean_reward = current_fitness
            = epoch_rewards.iter().sum::<f64>() / n_envs as f64
```

where `epoch_rewards` is the per-env per-episode total reward
vector collected during `evaluate_fitness`. This is the same
formula CEM uses post-PR-2b (at `cem.rs:209`, the post-Ch-41-
patch line), and it matches REINFORCE and PPO's idiom
(per Ch 24 Decision 1's baseline).

Two subtleties worth naming in-line:

**First, the reported mean_reward is the accepted chain
state's fitness, not the proposed fitness.** At any epoch
`k`, SA has just evaluated a proposed parameter vector and
either accepted or rejected it. If accepted, `current_
fitness = proposed_fitness` and `mean_reward = proposed_
fitness`. If rejected, `current_fitness` stays at its
previous value, and `mean_reward` is the *old* fitness —
the chain did not move. This is the correct reporting
convention for a Metropolis chain: the chain's trajectory
is the sequence of accepted states, not the sequence of
proposed states. A reader of SA's epoch-by-epoch
`mean_reward` sees the chain's trajectory directly.

**Second, the `proposed_fitness` is recorded separately in
`extra` for diagnostics.** A reader who wants to see the
proposal history (for debugging SA's acceptance rate or for
computing the acceptance-rejection diagnostic) can read
`extra["proposed_fitness"]` at each epoch. The `accepted`
field in `extra` (0.0 or 1.0 per epoch) records whether the
proposal was accepted; summing it over epochs gives the total
acceptance count. Ch 42 does not specify a runtime
acceptance-rate gate (see §9), but the diagnostic is
available for post-hoc analysis.

### 4.4 Sub-decision (c): the cooling schedule — geometric

Four cooling schedules are plausible candidates for SA's
temperature decay over a budgeted number of epochs:

1. **Geometric.** `T_{k+1} = T_k * alpha` for some `alpha ∈
   (0, 1)`. Single-parameter schedule. Standard in SA
   literature (Kirkpatrick 1983 and subsequent).
2. **Linear.** `T_{k+1} = T_0 - k * (T_0 - T_min) / n_epochs`.
   Single-parameter schedule (the target end temperature);
   less common in practice.
3. **Logarithmic.** `T_k = T_0 / log(k + 2)`. Provably
   converges to the global optimum given infinite time but
   excruciatingly slow in finite budgets. Not commonly used
   in practical SA.
4. **Adaptive** (e.g., based on acceptance rate). Adjust
   temperature each epoch based on observed acceptance ratio.
   Adds complexity and a second tuning knob.

**The pick: geometric.** Four reasons.

**First, geometric is the conventional SA default.** The
textbook SA schedule from Kirkpatrick et al. 1983 is
geometric, and the canonical implementations in the
operations research literature use geometric cooling with
`alpha ∈ [0.9, 0.99]`. A reader who knows SA from a standard
reference expects geometric; rendering a different schedule
would require an argument Ch 30 did not make and Ch 42
does not want to own.

**Second, geometric composes cleanly with the
budget-to-epochs formula.** The budget is `Steps(16M)`; the
per-epoch work is `n_envs * max_episode_steps = 32 * 5000 =
160_000` steps; the epoch count is `16M / 160_000 = 100`.
With 100 epochs and `T_0 = 0.5`, a geometric schedule with
`alpha = 0.955` gives `T_100 ≈ 0.5 * 0.955^100 ≈ 0.005`,
which is a 100× total cooling ratio. The math is explainable
in one line and tunable at a single knob.

**Third, linear and logarithmic both have structural
problems for the rematch's budget.** Linear's temperature
falls off at a constant rate regardless of the epoch index,
which over-explores early and under-exploits late.
Logarithmic's cooling ratio over 100 epochs is roughly
`log(102) ≈ 4.6`, which is an order of magnitude less
cooling than geometric — the chain barely cools at all over
the rematch's budget. Both schedules are rejected on the
grounds that they do not match SA literature's default
behavior and would require defense Ch 42 does not want to
own.

**Fourth, adaptive cooling is overkill for the rematch's
scope.** An adaptive schedule would add a tuning knob and a
feedback loop, and would require Ch 42 to argue why the
feedback is worth the complexity. The rematch's scope is
"does geometry-appropriate SA beat generic CEM at matched
complexity" — *specifically* the plain Metropolis form Ch 30
described. An adaptive variant is exactly the "richer
proposal structure" Ch 30 §"What each outcome would tell us"
names as the (a) null follow-up, to be run *after* the
basic rematch. Running an adaptive SA in the rematch would
contaminate the basic-SA result with a hyperparameter-tuning
signal and would deny the null outcome its informational
content. Rejected.

### 4.5 Sub-decision (d): `initial_temperature` and
`proposal_std` defaults

Two numerical defaults need picking: `initial_temperature`
and `proposal_std`. Both live on `SaHyperparams` and both
are tunable by the caller. Ch 42 picks defaults that are
defensible at the D2c SR task's reward scale.

**`initial_temperature = 0.5`.** The D2c SR task's reward
is `data.qpos[0].signum() * (omega * data.time).cos()`
(at `d2c_cem_training.rs:114`, recon-reported), which is
bounded in `[-1, +1]` per step. Per-episode total reward
over `EPISODE_STEPS = 5000` (at `d2c_cem_training.rs:73`,
recon-reported) is bounded in roughly `[-5000, +5000]`. But
the *mean* per-episode total across `n_envs = 32` trajectories
in post-Ch-24 units is divided by `n_envs` *inside* the mean,
so the effective reward scale is `[-(5000/32), +(5000/32)]
= [-156.25, +156.25]` — wait, that is wrong. Let me rewalk.

The per-episode total reward for one trajectory is bounded
in `[-5000, +5000]` (5000 steps times max |reward| = 1). The
mean across `n_envs = 32` trajectories is bounded in the
same range `[-5000, +5000]` — averaging 32 values each in
`[-5000, +5000]` gives a mean in `[-5000, +5000]`. The D2c
SR task's actual rewards are much smaller in magnitude
because the reward is `signum * cos`, which oscillates
around zero, and the per-step values sum to something much
smaller than the per-step bound times episode length.

The D2 SR findings memo records peak synchrony at `~0.098 ±
0.022` near `kt_mult ≈ 2.55` (per project memory's reference
to the memo). If "synchrony" is the per-step mean reward,
then the per-episode total is roughly `0.098 * 5000 ≈ 490`
at the peak, and the mean across `n_envs` is around the
same (all 32 envs are running the same policy against
different Langevin realizations, so the mean is roughly the
single-episode expectation). So the rematch's effective
reward scale is roughly `[-490, +490]` at peak and `[0, 490]`
at the optimistic end.

`initial_temperature = 0.5` is then 1/980 of the peak reward
scale. A Metropolis accept/reject with `T = 0.5` and a
delta of (say) `-10` (a reward drop of 10 units, which is
2% of the peak) gives an accept probability of `exp(-10 /
0.5) = exp(-20) ≈ 2e-9`, essentially zero. A delta of `-1`
(0.2% of peak) gives `exp(-1/0.5) = exp(-2) ≈ 0.135`, a
plausible accept rate for a Metropolis chain's early
exploration.

This is too cold. At `T = 0.5`, SA behaves like near-greedy
hill climbing and misses the ambient-exploration phase that
gives SA its geometry-appropriate advantage over CEM. A
warmer initial temperature is the right call.

**Revised initial_temperature pick: `T_0 = 50`.** At `T_0 =
50`, a delta of `-10` gives `exp(-10/50) = exp(-0.2) ≈
0.82` — most downhill proposals are accepted, producing the
ambient exploration SA needs. A delta of `-50` (10% of peak)
gives `exp(-1) ≈ 0.37`, still frequently accepted. A delta
of `-200` (40% of peak) gives `exp(-4) ≈ 0.018`, rarely
accepted. This covers the whole dynamic range SA needs for
the SR landscape.

The cooling schedule's endpoint: `T_100 ≈ T_0 * alpha^100`.
With `T_0 = 50` and `alpha = 0.955`, `T_100 ≈ 0.5`, which
is the cold-exploit regime. The 100× ratio over 100 epochs
is the standard SA cooling shape.

**`proposal_std = 0.5`.** The parameter vector has three
elements (the `LinearPolicy(2, 1)` weights). CEM's
`noise_std = 2.5` at `d2c_cem_training.rs:283` (recon-
reported) is the baseline CEM exploration scale for the
same parameter space. SA's `proposal_std` does not need to
match CEM's `noise_std` exactly — SA and CEM explore
differently (SA is a Metropolis chain around one point; CEM
samples a population around a mean) — but a value in the
same order of magnitude is a reasonable starting point.

`proposal_std = 0.5` is 1/5 of CEM's `noise_std`. The
rationale: SA's chain moves in increments of `proposal_std`
at each epoch, and over 100 epochs the expected drift is
`0.5 * sqrt(100) = 5` in each parameter dimension (Gaussian
random-walk scaling). This is enough to traverse the
`LinearPolicy` parameter space (whose elements are typically
`O(1)` after CEM's exploration finishes) without being so
coarse that the chain takes large jumps that miss local
structure.

A caller who wants a different proposal std passes a custom
`SaHyperparams`. The rematch fixture at §6 uses the default
`0.5`. Tuning is flagged in §9 as "out of scope for PR 3 —
if the first rematch run shows SA failing for
proposal-std reasons, the Ch 30 null follow-up (a) is the
pre-registered response, not a post-hoc tuning sweep."

**`cooling_decay = 0.955` and `temperature_min = 0.005`.**
These follow directly from the initial_temperature pick and
the geometric-schedule reasoning in §4.4. `cooling_decay =
0.955` gives `T_100 ≈ T_0 * 0.955^100 ≈ 0.5` at `T_0 = 50`,
which is the transition from warm-explore to cold-exploit.
`temperature_min = 0.005` is a floor against divide-by-zero
in the Metropolis formula's denominator; at `T = 0`, the
formula degenerates to pure greedy hill-climbing, and the
floor prevents that collapse. The floor's value is
effectively unreached (geometric decay from 50 to 0.005 over
100 epochs keeps the temperature above 0.005 at every epoch)
but is kept as a defensive guard.

**The honest framing: these defaults are a calibration
guess.** The reward-scale arithmetic in this section walks
through peak-reward-to-temperature ratios and argues that
`T_0 = 50` produces "plausible" Metropolis accept rates for
deltas of ~10, ~50, and ~200 reward units. But the argument
does *not* determine the typical delta magnitude between
nearby parameter perturbations under a `proposal_std = 0.5`
Gaussian step on the SR landscape — that would require
empirical data from a pilot run or from D2c's existing CEM
exploration traces, neither of which are available at the
rendering time of this chapter. The calibration is therefore
a guess defended by a plausibility argument, not a fitted
value from observed SR dynamics. If the first rematch run
shows SA accepting nearly every proposal (indicating `T_0`
is too warm for the observed delta distribution) or
rejecting nearly every proposal (too cold), the sensible
response is Ch 30's (a) null follow-up with revised
hyperparameters, not a post-hoc tuning sweep on the basic
rematch. The acceptance rate per epoch is tracked in
`EpochMetrics::extra["accepted"]` (see §4.2) for post-hoc
inspection, and the sum across epochs gives the observed
acceptance count — a reader of the rematch's output can
verify whether the T_0 pick was in the right range. The
`SaHyperparams` struct exposes all four values as `pub`
fields so a follow-up experiment can override them without
touching sim-opt's source.

### 4.6 Sub-decision (e): the fitness evaluation shape —
multi-env averaging

This is the substantive argument in §4 and the one Ch 30 and
Ch 23 did not address. SA evaluates a proposed policy by
running episodes in `VecEnv` and reducing the per-env
results to a scalar fitness. Four shapes are plausible.

**Shape (i): single-env-per-iteration.** Each Metropolis
iteration runs one episode on one env (say env[0]) and uses
that single episode's total reward as the fitness. Over 100
epochs, SA uses 100 episodes of the 32-env pool and leaves
the other 31 envs idle per iteration. The budget-to-epochs
formula `n_epochs = budget / (n_envs * max_episode_steps)` is
unchanged, so SA still runs 100 Metropolis iterations at
`Steps(16M)`. The work per iteration is 1/32 of the
`n_envs` pool.

Problem: 31/32 of `BatchSim`'s parallel envs are wasted.
Wall-clock time per iteration is roughly `1/32` of what
multi-env averaging would cost (at full parallelism), but
the budget is counted in env steps, not wall-clock — so
the budget formula would have to *not* multiply by `n_envs`
to be consistent. If the budget formula does multiply (as
Ch 22 committed for the uniform compute-parity mechanism),
SA at shape (i) runs 100 iterations but each iteration only
actually uses 5000 env steps (one episode on one env), so
the total env work is 500_000, not 16M. The compute-parity
argument Ch 22 made is broken.

Alternative: change the budget formula for SA. This violates
Ch 22's uniform `n_epochs = N / (n_envs * max_episode_
steps).max(1)` commitment. Rejected.

**Shape (ii): multi-env averaging.** Each Metropolis
iteration runs `n_envs` episodes in parallel via
`collect_episodic_rollout` and averages the per-env
per-episode total rewards into a single fitness scalar.
The fitness is the Ch 24 Decision 1 `mean_reward` — mean
per-episode total across `n_envs` trajectories — and the
budget formula gives `n_epochs = 16M / (32 * 5000) = 100`
Metropolis iterations. The full parallel pool is used each
iteration.

Consequences: the fitness has 1/32 the variance of a
single-env estimate (under independent Langevin realizations
across envs), which means the Metropolis chain has a lower
effective temperature for the same nominal temperature — the
chain discriminates proposals more sharply than it would
at single-env. This is a real effect and the defense:
Metropolis is *robust* to fitness noise, meaning the chain's
stationary distribution converges to the correct Boltzmann
distribution even under noisy fitness estimates, as long as
the noise is unbiased. Multi-env averaging is unbiased
(each env's draw is from the same distribution, under
different Langevin realizations), so the averaging just
tightens the noise without changing the target distribution.

**Shape (iii): replica SA.** Run 32 independent SA chains in
parallel, one per env. Each chain has its own temperature,
its own current state, its own best tracker. At the end,
pick the best chain's best policy. This uses the full
parallel pool *and* does 32× more Metropolis work than
shape (ii) at the same budget.

Problem: this is structurally Parallel Tempering without
the tempering step, and Ch 30 explicitly reserves Parallel
Tempering as the (b) null follow-up. Running 32 parallel SA
chains in the initial rematch would contaminate the rematch's
result with a "multiple-chains-vs-single-chain" effect that
Ch 30's framing specifically calls out as a distinct
experiment. The rematch is *the single-chain SA case*, and
shape (iii) would turn it into a different experiment. Ch
30's scope discipline rules this out:

> Picking a new benchmark post hoc is explicitly not on this
> list: if SR turns out to be a bad discriminator for
> physics-aware vs generic, that is information about *this
> experiment*, not a license to redefine the question after
> seeing the answer.

Rejected on Ch 30 scope grounds. Shape (iii) is a future
experiment, not this rematch.

**Shape (iv): inner Metropolis loop per epoch.** Each epoch
runs `n_envs` Metropolis steps, each evaluated on one env.
The intent is "3200 Metropolis steps instead of 100, which
is closer to the textbook SA chain length." The
implementation question is how the inner steps interact with
`BatchSim`'s parallel-step-is-the-unit-of-work model.

Problem: `BatchSim`'s step API is `step_all()` — it steps
every env in parallel, there is no single-env step path.
Shape (iv) has two ways to implement its "one Metropolis
step per env" structure, and both fail:

*(iv-a)* Each inner Metropolis step calls
`collect_episodic_rollout` (which internally runs
`BatchSim.step_all()` for 5000 episode-steps), reads env[i]'s
trajectory, and discards the other 31 env trajectories. Over
3200 inner steps, this pays for `3200 × 5000 = 16M`
`step_all()` calls, each of which steps 32 envs — so the
total env-step count is `3200 × 32 × 5000 = 512M`. This is
32× over the 16M budget Ch 22 committed to. Shape (iv-a)
breaks Ch 22's budget formula by exactly the factor Ch 22's
uniform `n_epochs = budget / (n_envs × max_episode_steps)`
formula protects against.

*(iv-b)* Each epoch runs *one* `BatchSim.step_all()` loop of
5000 steps, collecting 32 parallel env trajectories, and
then applies 32 Metropolis accept/reject decisions based on
the 32 trajectories — one decision per env. But this is
structurally replica SA: 32 parallel chains each running
their own Metropolis trajectory, with the only shared state
being the physics solver infrastructure. Shape (iv-b) is
shape (iii) in disguise, and Ch 30 reserves parallel-chain
SA as the (b) null follow-up. Rejected on the same Ch 30
scope-discipline grounds as shape (iii).

The honest framing: shape (iv) is either a 32×-over-budget
single-chain SA or a disguised replica SA. Neither survives
the rematch's scoping. Rejected.

One might argue for shape (iv-a) on the theoretical grounds
that 3200 Metropolis steps is closer to the "nominal" SA
chain length that textbook SA examples use. This is true but
does not change the rejection: Ch 22's compute-parity
commitment is counted in env steps, and shape (iv-a) consumes
32× the budget to achieve its chain length. The rematch runs
under matched compute with CEM, not under nominal SA chain
length, and shape (iv-a) breaks the matching by 32×.

**The pick: shape (ii), multi-env averaging.** Pressure-
tested against three alternatives, each rejected on
structural grounds. Shape (i) breaks Ch 22's budget formula
or wastes parallelism. Shape (iii) runs a different
experiment than the one Ch 30 scoped. Shape (iv) either
breaks Ch 22's budget formula by 32× (variant iv-a) or
collapses into shape (iii) (variant iv-b).

The cost of shape (ii) — 100 Metropolis iterations total,
which is fewer than a textbook SA chain — is acknowledged in
§4.7 and named as a honest trade for wall-clock parity with
CEM. It is *not* a defect; it is the deliberate shape of a
matched-complexity, matched-compute comparison. A reader who
expects "textbook SA runs 10000+ Metropolis steps" should
understand that the rematch is running SA under the same
compute budget as CEM's 100-epoch elite-selection loop, not
under a textbook SA-literature budget.

### 4.7 The 100-Metropolis-step cost, named honestly

Under shape (ii), SA runs exactly 100 Metropolis iterations
for the rematch's 16M env step budget. This is fewer than a
typical SA literature example, which might run 10⁴ or more
steps. The gap is a consequence of Ch 22's compute-parity
commitment: SA's compute budget is matched to CEM's compute
budget, and CEM's 100-epoch loop at the D2c config uses
`100 * 32 * 5000 = 16M` env steps. SA at shape (ii)
consumes the same 16M env steps across 100 Metropolis steps.
SA at shape (iv) would spread 16M env steps across 3200
Metropolis steps but lose wall-clock parity.

The honest framing: the rematch is testing *SA at matched
compute* against *CEM at matched compute*, not *SA at its
natural chain length* against *CEM at its natural chain
length*. If SA needs more Metropolis steps to resolve the SR
peak than 100 steps can give it, the rematch's null outcome
is the right answer — it says "100-step SA under shape (ii)
does not resolve the SR peak any better than CEM does," and
Ch 30's (a) follow-up (richer proposal structure) or (b)
follow-up (Parallel Tempering) is the pre-registered next
experiment. An ambiguous outcome at 100 steps would be
particularly informative: it says "SA nearly works at this
compute budget, and a larger budget or a different schedule
might resolve it." A null says "SA does not work at this
compute budget, full stop."

A reader of the rematch writeup who sees a null and thinks
"but SA would have worked with more iterations" is correct
that SA might work with more iterations — and the allowed
follow-ups (richer proposal, Parallel Tempering) are
specifically the experiments that test that hypothesis under
different compute regimes. The basic rematch is the
single-chain, matched-compute baseline.

### 4.8 Unit tests for SA

PR 3a ships four unit tests for SA at `sim/L0/opt/src/
algorithm.rs`'s test module (inline `#[cfg(test)] mod tests`,
matching CEM's pattern at `ml-bridge/src/cem.rs:265-...`,
recon-reported):

1. **`sa_name`.** Asserts `sa.name() == "SA"`. Three lines.
2. **`sa_smoke_2dof`.** Constructs an `Sa` with a
   `LinearPolicy` on the `reaching_2dof()` task (from
   `ml-bridge/src/task.rs`), trains for 5 epochs with
   `TrainingBudget::Epochs(5)`, asserts the metrics have
   length 5 and each has a non-zero `total_steps`. Matches
   CEM's `cem_smoke_2dof` pattern at `cem.rs:291-329`
   (recon-reported). ~40 lines.
3. **`sa_best_tracker_monotone`.** Constructs an SA, trains
   for 20 epochs, asserts that `sa.best_artifact()`'s
   underlying best_reward never decreases across
   checkpoint reads during training. Tests the inline
   best-tracking logic. ~25 lines.
4. **`sa_checkpoint_roundtrip`.** Constructs an SA, trains
   for 10 epochs, calls `checkpoint()`, then
   `Sa::from_checkpoint(&checkpoint, hyperparams)`, and
   asserts the restored SA's current_params and best_params
   match. ~30 lines.

The tests do *not* exercise the rematch's SR task (that is
PR 3b's scope, inside the `sim-opt/tests/` integration test
directory), and they do *not* exercise multi-env averaging
with stochastic physics (the `reaching_2dof()` task is
deterministic). They exercise the `Algorithm` trait
implementation surface and the hyperparameter defaults;
that is the scope PR 3a is reviewable for.

## Section 5 — The analysis module (sub-decision (f))

### 5.1 Placement: library module, not inline in the test fixture

Ch 42 places the rematch's statistical-analysis machinery in
a library module at `sim/L0/opt/src/analysis.rs`, not inline
in the rematch test fixture at `sim-opt/tests/d2c_sr_
rematch.rs`. The library module is the sub-decision (f) pick.

Two placements were considered.

**Placement (1): library module (`pub mod analysis` in
`lib.rs`).** The bootstrap, bimodality, and classifier
functions live in `sim-opt/src/analysis.rs` with public
symbols re-exported from `lib.rs`. The rematch test fixture
imports them via `use sim_opt::{bootstrap_diff_means,
bimodality_coefficient, classify_outcome, run_rematch};` and
calls them at the fixture's top-level test function.

**Placement (2): inline in the test fixture.** The bootstrap,
bimodality, and classifier are private helper functions
inside `sim-opt/tests/d2c_sr_rematch.rs`, not exported from
the crate. Only the rematch fixture can call them. If a
future rematch wants to reuse the machinery, it either
copy-pastes from the fixture or refactors the functions up
to the library later.

**The pick: placement (1), library module.** Four reasons.

**First, Ch 30's pre-committed null follow-ups.** Ch 30
§"What each outcome would tell us" under the null bucket
names two allowed follow-ups: (a) a richer proposal
structure for SA and (b) Parallel Tempering, both on the
same SR task. Both follow-ups would run their own instance
of the rematch protocol — with a different SA implementation
or a different algorithm — and need the same bootstrap /
bimodality / classifier machinery the initial rematch uses.
Under placement (1), each follow-up imports from
`sim_opt::analysis` and gets the machinery directly. Under
placement (2), each follow-up copy-pastes from the initial
rematch's test fixture (which is at `sim-opt/tests/d2c_sr_
rematch.rs` — a file that would grow or split for each
follow-up's new test). Placement (1) is the R34 chassis-
overbuild form; placement (2) is the workaround form.

**Second, unit-test scope.** A library module can have its
own test submodule at `sim-opt/src/analysis.rs`'s
`#[cfg(test)] mod tests`, which tests the bootstrap /
bimodality / classifier functions in isolation with
hand-crafted inputs. An inline-in-test-fixture placement
cannot — the functions are private to the test file and can
only be exercised indirectly through the fixture's full
rematch run. Unit-level testing of the analysis machinery is
valuable (e.g., testing that `classify_outcome` produces
the right bucket for edge cases like `(lower=0, upper=0,
point=0)`, which the full rematch would never hit), and
placement (1) makes it cheap.

**Third, reader navigation.** A reader of Ch 42 or of the
rematch writeup who wants to understand *how* the bootstrap
is implemented — what `B` is, what the resampling strategy
is, how the CI is constructed — should be able to read a
self-contained file. A library module at `sim-opt/src/
analysis.rs` is that file. An inline-in-test-fixture
placement hides the analysis code behind the test fixture's
setup boilerplate, which is readable but not as self-
contained.

**Fourth, the "code speaks for itself" preference.** The
memory entry at `feedback_code_speaks.md` frames code as the
authoritative documentation for the design. The analysis
module is Ch 32's protocol made code; a reader who wants
to verify Ch 32's claims should be able to look at
`analysis.rs` and see the protocol rendered directly. A
library module with named public functions is easier to
read as documentation than a test-file-private helper.

The counter-argument — "the analysis machinery has only one
current consumer, so a library module is premature" — is
answered by the Ch 30 null-follow-up argument. The initial
rematch has one current consumer, but Ch 30 already names
the next two consumers as pre-committed follow-ups on the
same task with the same protocol. The library module is not
premature; it is where the future consumers are supposed to
come from.

### 5.2 `bootstrap_diff_means`

The canonical percentile-bootstrap-on-means function,
implementing Ch 32 §3.2's pseudocode at the signature level.

```rust
use rand::Rng;
use serde::{Deserialize, Serialize};

/// A bootstrap confidence interval on a difference statistic,
/// with the point estimate and both bounds.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BootstrapCi {
    /// The observed difference of means (or medians) from the
    /// original samples, not from the bootstrap distribution.
    pub point_estimate: f64,
    /// Lower bound (2.5th percentile of the bootstrap
    /// distribution for a two-sided 95% CI).
    pub lower: f64,
    /// Upper bound (97.5th percentile).
    pub upper: f64,
    /// Number of bootstrap resamples drawn.
    pub n_resamples: usize,
}

impl BootstrapCi {
    /// Classify the CI into one of Ch 30's three outcomes per
    /// Ch 32 §3.3's table.
    #[must_use]
    pub const fn classify(&self) -> RematchOutcome {
        if self.lower > 0.0 && self.upper > 0.0 {
            RematchOutcome::Positive
        } else if self.lower >= 0.0 || self.upper <= 0.0 {
            // CI excludes zero on the low side, or strictly on
            // the high side — both are null-strict cases.
            // Actually, the Ch 32 §3.3 table distinguishes
            // "positive" (strict positive CI) from "null"
            // (strict negative CI, or straddle-with-point≤0).
            // The match below is the literal table.
            if self.upper <= 0.0 {
                RematchOutcome::Null
            } else if self.point_estimate > 0.0 {
                // straddle with point > 0 — ambiguous
                RematchOutcome::Ambiguous
            } else {
                RematchOutcome::Null
            }
        } else if self.point_estimate > 0.0 {
            RematchOutcome::Ambiguous
        } else {
            RematchOutcome::Null
        }
    }
}
```

The `classify` method's body is awkward because it is
rendering a table into a sequence of `if` branches that
preserves the table's semantics exactly. A cleaner rendering
uses a single `match` on a three-tuple of booleans:

```rust
impl BootstrapCi {
    #[must_use]
    pub fn classify(&self) -> RematchOutcome {
        let lower_pos = self.lower > 0.0;
        let upper_pos = self.upper > 0.0;
        let point_pos = self.point_estimate > 0.0;
        match (lower_pos, upper_pos, point_pos) {
            (true,  true,  _    ) => RematchOutcome::Positive,
            (false, false, _    ) => RematchOutcome::Null,
            (false, true,  true ) => RematchOutcome::Ambiguous,
            (false, true,  false) => RematchOutcome::Null,
            (true,  false, _    ) => {
                // Impossible under a well-formed CI (lower ≤
                // upper by construction). Treat as null
                // defensively.
                RematchOutcome::Null
            }
        }
    }
}
```

The `match` form is the PR 3a implementation form. The `if`
form above is a walk-through of the same logic for reader
clarity in the plan chapter; PR 3a ships the `match` form.

**`bootstrap_diff_means` itself.**

```rust
/// Percentile bootstrap CI on the difference of means of two
/// samples. Ch 32 §3.2 pseudocode implementation.
///
/// # Panics
///
/// Panics if either input slice is empty. The rematch protocol
/// at §5.5 guarantees non-empty inputs at every call site, so
/// the panic is a defensive guard rather than a runtime
/// condition.
pub fn bootstrap_diff_means(
    r_a: &[f64],
    r_b: &[f64],
    rng: &mut impl Rng,
) -> BootstrapCi {
    const B: usize = 10_000;
    assert!(!r_a.is_empty(), "bootstrap_diff_means: r_a is empty");
    assert!(!r_b.is_empty(), "bootstrap_diff_means: r_b is empty");

    let n_a = r_a.len();
    let n_b = r_b.len();

    let mean_a = mean(r_a);
    let mean_b = mean(r_b);
    let point_estimate = mean_a - mean_b;

    let mut diffs = Vec::with_capacity(B);
    for _ in 0..B {
        let resample_mean_a: f64 = (0..n_a)
            .map(|_| r_a[rng.gen_range(0..n_a)])
            .sum::<f64>()
            / (n_a as f64);
        let resample_mean_b: f64 = (0..n_b)
            .map(|_| r_b[rng.gen_range(0..n_b)])
            .sum::<f64>()
            / (n_b as f64);
        diffs.push(resample_mean_a - resample_mean_b);
    }
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let lower_idx = (0.025 * B as f64) as usize;
    let upper_idx = (0.975 * B as f64) as usize;

    BootstrapCi {
        point_estimate,
        lower: diffs[lower_idx],
        upper: diffs[upper_idx],
        n_resamples: B,
    }
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}
```

`B = 10_000` matches Ch 32 §3.2's pick. Paired resampling
(each iteration draws both `n_a` and `n_b` samples with
replacement, independently) matches the Ch 32 §3.2 shape.
Percentile indexing at `(0.025 * B) as usize` and
`(0.975 * B) as usize` gives indices 250 and 9750, which
are the 2.5th and 97.5th percentiles of the sorted bootstrap
distribution.

The `n_resamples: usize` field on `BootstrapCi` records `B`
in the result struct so a caller who wants to verify the
bootstrap's resample count can read it directly — a reader
of a serialized `BootstrapCi` (via `serde`) has the count
in the serialized representation.

### 5.3 `bootstrap_diff_medians`

Identical shape to `bootstrap_diff_means` with `median`
substituted for `mean`. The substitution is at three sites:
the initial `mean_a` / `mean_b` computation for the point
estimate, and the per-resample `resample_mean_a` /
`resample_mean_b` computation inside the loop.

```rust
pub fn bootstrap_diff_medians(
    r_a: &[f64],
    r_b: &[f64],
    rng: &mut impl Rng,
) -> BootstrapCi {
    const B: usize = 10_000;
    assert!(!r_a.is_empty(), "bootstrap_diff_medians: r_a is empty");
    assert!(!r_b.is_empty(), "bootstrap_diff_medians: r_b is empty");

    let n_a = r_a.len();
    let n_b = r_b.len();

    let point_estimate = median(r_a) - median(r_b);

    let mut diffs = Vec::with_capacity(B);
    let mut resample_a = vec![0.0; n_a];
    let mut resample_b = vec![0.0; n_b];
    for _ in 0..B {
        for slot in resample_a.iter_mut() {
            *slot = r_a[rng.gen_range(0..n_a)];
        }
        for slot in resample_b.iter_mut() {
            *slot = r_b[rng.gen_range(0..n_b)];
        }
        diffs.push(median(&resample_a) - median(&resample_b));
    }
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let lower_idx = (0.025 * B as f64) as usize;
    let upper_idx = (0.975 * B as f64) as usize;

    BootstrapCi {
        point_estimate,
        lower: diffs[lower_idx],
        upper: diffs[upper_idx],
        n_resamples: B,
    }
}

fn median(values: &[f64]) -> f64 {
    let mut sorted: Vec<f64> = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = sorted.len();
    if n % 2 == 0 {
        0.5 * (sorted[n / 2 - 1] + sorted[n / 2])
    } else {
        sorted[n / 2]
    }
}
```

The median function sorts a copy of the input — cheap at
`n = 10` or `n = 20` — and takes the middle element (or the
average of the two middle elements for even `n`). This is
the textbook sample median definition.

The resample vectors `resample_a` and `resample_b` are
allocated once outside the bootstrap loop and reused across
iterations. This is a small performance optimization (avoids
`Vec::with_capacity` in the hot loop) that matters when `B
= 10_000` and the loop runs twice per rematch classification
step. The mean-based bootstrap in §5.2 does not need the
optimization because it computes the mean directly from the
draws without materializing the resample vector.

### 5.4 `bimodality_coefficient` and `classify_outcome`

The Pearson/SAS bimodality coefficient per Ch 32 §6.2's
formula:

```rust
/// Pearson's bimodality coefficient (SAS small-sample
/// corrected form). `BC > 5/9` indicates bimodality (or
/// strong skew); values below the threshold indicate
/// unimodality.
///
/// Requires `n >= 4` because the denominator has a
/// `(n - 2) * (n - 3)` factor.
///
/// # Panics
///
/// Panics if `values.len() < 4`. The rematch protocol
/// guarantees `n = 10` or `n = 20` at every call site.
#[allow(clippy::cast_precision_loss)]
pub fn bimodality_coefficient(values: &[f64]) -> f64 {
    assert!(
        values.len() >= 4,
        "bimodality_coefficient requires n >= 4, got {}",
        values.len(),
    );
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let m2: f64 = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
    let m3: f64 = values.iter().map(|v| (v - mean).powi(3)).sum::<f64>() / n;
    let m4: f64 = values.iter().map(|v| (v - mean).powi(4)).sum::<f64>() / n;

    // Sample skewness g = m3 / m2^(3/2). Sample excess
    // kurtosis kappa = m4 / m2^2 - 3.
    let g = if m2 > 0.0 { m3 / m2.powf(1.5) } else { 0.0 };
    let kappa = if m2 > 0.0 { m4 / (m2 * m2) - 3.0 } else { 0.0 };

    let numerator = g * g + 1.0;
    let correction = (3.0 * (n - 1.0).powi(2)) / ((n - 2.0) * (n - 3.0));
    let denominator = kappa + correction;

    if denominator > 0.0 {
        numerator / denominator
    } else {
        // Degenerate case: constant input or near-constant.
        // Return 0.0 (unimodal) defensively.
        0.0
    }
}
```

The skewness and kurtosis are computed from the biased
(moment-based) definitions, not the bias-corrected sample
forms. The Ch 32 §6.2 formula uses the biased moments with
the `(n - 1)^2 / ((n - 2)(n - 3))` correction term applied
at the denominator level. This is the SAS convention and
matches the Ch 32 pick.

The `m2 > 0.0` and `denominator > 0.0` guards handle
near-constant inputs (where all values are nearly identical
and `m2 ≈ 0`). The guards return `0.0` (unimodal) as the
defensive default, which is the correct behavior for a
degenerate rematch input where all 10 replicates produced
the same scalar. This is vanishingly unlikely in practice —
seed-varied training runs never converge to exact-equal
scalars at the f64 precision level — but the guards prevent
a division-by-zero if it happens.

**`classify_outcome` as a free function.** The classification
rule is available both as a method on `BootstrapCi` (§5.2's
`classify`) and as a free function taking the three scalars
directly:

```rust
/// Ch 30's three-outcome classifier per Ch 32 §3.3's table.
///
/// Takes the CI bounds and point estimate directly — the free
/// function is useful for callers that want to classify
/// without constructing a `BootstrapCi` (e.g., for testing
/// edge cases).
#[must_use]
pub fn classify_outcome(
    lower: f64,
    upper: f64,
    point_estimate: f64,
) -> RematchOutcome {
    let lower_pos = lower > 0.0;
    let upper_pos = upper > 0.0;
    let point_pos = point_estimate > 0.0;
    match (lower_pos, upper_pos, point_pos) {
        (true,  true,  _    ) => RematchOutcome::Positive,
        (false, false, _    ) => RematchOutcome::Null,
        (false, true,  true ) => RematchOutcome::Ambiguous,
        (false, true,  false) => RematchOutcome::Null,
        (true,  false, _    ) => RematchOutcome::Null,
    }
}

/// One of Ch 30's three meaningful outcomes for the rematch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RematchOutcome {
    /// CI lower > 0: SA reliably outperforms CEM.
    Positive,
    /// CI upper <= 0: SA does not outperform CEM, or
    /// CI straddles zero with point estimate <= 0.
    Null,
    /// CI straddles zero with point estimate > 0: the margin
    /// is within the seed-variance envelope. Triggers the
    /// folded-pilot expansion from N=10 to N=20.
    Ambiguous,
}
```

Three variants, no fourth hedge. Per Ch 31 §3.2: "a 'weak
positive' or 'near-null' hedge is not on the menu — Ch 30
deliberately specified three outcomes."

### 5.5 `run_rematch`: the folded-pilot driver

The driver is the function that executes Ch 32's folded-pilot
protocol end-to-end. It takes the rematch's inputs (a task
builder, an algorithm builder list, a bootstrap RNG), runs
the initial batch via `Competition::run_replicates`,
classifies the result, expands to `N = 20` if ambiguous,
and returns the final classified outcome.

```rust
use sim_ml_bridge::{
    Algorithm, Competition, CompetitionResult, EnvError,
    TaskConfig,
};
use sim_thermostat::prf::splitmix64;

/// Pre-registered master seed from Ch 32 Decision 4. Matches
/// `d2c_cem_training.rs:62`'s `SEED_BASE` literal and Ch 23
/// §1.2's example master value.
pub const REMATCH_MASTER_SEED: u64 = 20_260_412;

/// The initial batch size Ch 32 Decision 3 picks.
pub const N_INITIAL: usize = 10;

/// The expanded batch size Ch 32 Decision 3 picks.
pub const N_EXPANDED: usize = 20;

/// The task name the rematch uses inside `Competition`'s
/// task registry. Fixed so `replicate_best_rewards` calls
/// can find the task consistently across the initial and
/// expanded batches.
pub const REMATCH_TASK_NAME: &str = "d2c-sr-rematch";

/// Run the folded-pilot rematch protocol end-to-end.
///
/// Parameters:
/// - `competition`: the `Competition` runner pre-configured
///   with `Steps(16M)` budget and `n_envs = 32`.
/// - `task`: the single `TaskConfig` whose `build_fn` closure
///   uses its `seed: u64` parameter to construct a fresh
///   `VecEnv` with a freshly-seeded `LangevinThermostat` on
///   each call. Under the Ch 42 §2 patched `TaskConfig::
///   build_fn` signature, each of `Competition::run_replicates`'s
///   10 inner `build_vec_env(n_envs, seed)` calls produces a
///   `VecEnv` with that replicate's master seed.
/// - `algorithm_builders`: slice of builder closures producing
///   `Box<dyn Algorithm>`. Rematch requires exactly two:
///   `{CEM, SA}` at matched complexity, in that order.
/// - `bootstrap_rng`: the RNG used for bootstrap resampling.
///   Independent of the per-replicate training RNGs.
///
/// Returns `Ok(RematchOutcome)` with the final classified
/// outcome (after expansion if the initial batch was
/// ambiguous), or `Err(EnvError)` if any of the
/// `run_replicates` calls failed.
///
/// # Errors
///
/// Propagates any `EnvError` from `Competition::run_replicates`.
pub fn run_rematch(
    competition: &Competition,
    task: &TaskConfig,
    algorithm_builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
    bootstrap_rng: &mut impl rand::Rng,
) -> Result<RematchOutcome, EnvError> {
    // Derive the initial batch's 10 seeds.
    let initial_seeds: Vec<u64> = (0..N_INITIAL)
        .map(|i| splitmix64(REMATCH_MASTER_SEED.wrapping_add(i as u64)))
        .collect();

    // Run the initial batch in a single run_replicates call.
    // Under the Ch 42 §2 patch, Competition::run_replicates
    // calls task.build_vec_env(n_envs, seed) with each
    // replicate's seed, and the TaskConfig's build_fn closure
    // uses that seed to construct the LangevinThermostat with
    // the correct per-replicate master_seed. The same seed is
    // also threaded to algorithm.train(..., seed, ...) for the
    // algorithm's exploration RNG, so one seed per replicate
    // controls both the physics noise sequence and the
    // algorithm's exploration.
    let initial_result = competition.run_replicates(
        std::slice::from_ref(task),
        algorithm_builders,
        &initial_seeds,
    )?;

    let mut r_cem = initial_result
        .replicate_best_rewards(REMATCH_TASK_NAME, "CEM");
    let mut r_sa = initial_result
        .replicate_best_rewards(REMATCH_TASK_NAME, "SA");

    // Classify the initial batch.
    let initial_outcome = test_and_classify(&r_cem, &r_sa, bootstrap_rng);

    // If unambiguous, stop here.
    if initial_outcome != RematchOutcome::Ambiguous {
        return Ok(initial_outcome);
    }

    // Ambiguous: expand to N = 20 with the next 10 seeds in a
    // second run_replicates call.
    let expansion_seeds: Vec<u64> = (N_INITIAL..N_EXPANDED)
        .map(|i| splitmix64(REMATCH_MASTER_SEED.wrapping_add(i as u64)))
        .collect();
    let expansion_result = competition.run_replicates(
        std::slice::from_ref(task),
        algorithm_builders,
        &expansion_seeds,
    )?;

    r_cem.extend_from_slice(
        &expansion_result.replicate_best_rewards(REMATCH_TASK_NAME, "CEM"),
    );
    r_sa.extend_from_slice(
        &expansion_result.replicate_best_rewards(REMATCH_TASK_NAME, "SA"),
    );

    // Re-classify at N = 20. The expanded batch's verdict is
    // the headline, regardless of bucket.
    Ok(test_and_classify(&r_cem, &r_sa, bootstrap_rng))
}

/// Apply Ch 32's test family and classification rule to a
/// pair of replicate vectors. Checks bimodality first and
/// substitutes medians for means if either algorithm's BC
/// exceeds 5/9.
fn test_and_classify(
    r_cem: &[f64],
    r_sa: &[f64],
    rng: &mut impl rand::Rng,
) -> RematchOutcome {
    let bc_cem = bimodality_coefficient(r_cem);
    let bc_sa = bimodality_coefficient(r_sa);
    let ci = if bc_cem > 5.0 / 9.0 || bc_sa > 5.0 / 9.0 {
        // Bimodality contingency: substitute medians.
        bootstrap_diff_medians(r_sa, r_cem, rng)
    } else {
        bootstrap_diff_means(r_sa, r_cem, rng)
    };
    ci.classify()
}
```

Two implementation notes worth naming at the rendering layer.

**First, the single `run_replicates` call matches Ch 32 §4.8's
skeleton literally.** The driver passes the full 10-element
seeds slice (or 20-element post-expansion) to `run_replicates`
in one call, and the `TaskConfig`'s `build_fn` closure — under
the §2 patched `Fn(usize, u64)` signature — uses each seed as
the `LangevinThermostat`'s `master_seed` at `VecEnv`
construction time. The rematch has one logical task (D2c SR)
with one `TaskConfig` whose `build_fn` takes the seed as a
parameter, and `run_replicates` handles the cross-replicate
loop internally. This is the shape §2.3's rendering (ii) was
picked for, and its value is visible here: without the §2
patch, the closure would have to capture the seed at
construction time, forcing a fresh `TaskConfig` per replicate
and a Ch 42-owned outer loop (rendering (i)); with the §2
patch, `run_replicates` owns the loop and Ch 32 §4.8's
skeleton renders directly. The code is shorter, the chassis
API is properly used, and future stochastic-physics
experiments get the same pattern for free.

**Second, the `test_and_classify` function argument order
is `(r_cem, r_sa)` but the bootstrap call is
`bootstrap_diff_means(r_sa, r_cem, rng)`, producing
`mean(r_sa) - mean(r_cem)`.** The difference is "SA minus
CEM," which is the natural orientation for the rematch's
hypothesis (positive means SA wins). A reader who sees
`test_and_classify(r_cem, r_sa, rng)` at the call site and
then reads the body to find `bootstrap_diff_means(r_sa,
r_cem, ...)` needs a moment to reconcile the argument
ordering. The signature's `(r_cem, r_sa)` order is
alphabetical (CEM < SA) and the bootstrap's `(r_sa, r_cem)`
order is "first minus second"; both conventions are
defensible, and the inline documentation comment on
`test_and_classify` names the orientation explicitly: "the
returned outcome is in SA-vs-CEM orientation: positive means
SA's mean exceeds CEM's mean."

### 5.6 Unit tests for the analysis module

PR 3a ships unit tests for each analysis function in
`sim-opt/src/analysis.rs`'s `#[cfg(test)] mod tests`:

1. **`bootstrap_diff_means_positive_ci`.** Hand-crafted
   inputs `r_a = [1.0, 1.0, 1.0, 1.0, 1.0]` and `r_b = [0.0,
   0.0, 0.0, 0.0, 0.0]`, asserts the resulting CI has
   `point_estimate ≈ 1.0`, `lower > 0.0`, `upper > 0.0`.
   Classification should be `Positive`. ~25 lines.
2. **`bootstrap_diff_means_null_ci`.** Inputs `r_a = r_b =
   [1.0, 2.0, 3.0, 4.0, 5.0]` (identical samples), asserts
   the CI straddles zero with point_estimate ≈ 0. ~20 lines.
3. **`bootstrap_diff_means_ambiguous_ci`.** Inputs where the
   sample means favor `r_a` but the within-sample variance
   is large enough that the CI straddles zero. Specific
   values: `r_a = [1.0, -0.5, 2.0, 0.0, 1.0]`, `r_b = [0.0,
   0.5, -0.5, 1.0, -1.0]`. Asserts classification is
   `Ambiguous`. ~25 lines. Deterministic bootstrap RNG seed
   so the test is reproducible.
4. **`bootstrap_diff_medians_robust_to_outlier`.** Inputs
   `r_a = [1.0, 1.0, 1.0, 1.0, 100.0]` (one outlier) and
   `r_b = [0.0, 0.0, 0.0, 0.0, 0.0]`. Asserts the
   median-based CI has `point_estimate = 1.0` (median of
   r_a minus median of r_b), robust to the outlier that
   would inflate the mean-based CI. ~25 lines.
5. **`bimodality_coefficient_unimodal`.** Input
   `[1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5]`
   (symmetric linear), asserts `BC < 5/9`. ~15 lines.
6. **`bimodality_coefficient_bimodal`.** Input
   `[1.0, 1.0, 1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0]`
   (two clusters at 1 and 5), asserts `BC > 5/9`. ~15 lines.
7. **`bimodality_coefficient_requires_n_geq_4`.** Input of
   length 3, asserts the call panics with the documented
   error message. Uses `#[should_panic]`. ~10 lines.
8. **`classify_outcome_boundary_cases`.** Tests the edge
   cases at `(lower=0, upper=0, point=0)`,
   `(lower=0, upper=1, point=0)`, and similar boundary
   conditions. Ensures the strict `>` vs `>=` boundaries
   match Ch 32 §3.3's table. ~30 lines with multiple
   assertions.
9. **`run_rematch_ambiguous_triggers_expansion`.** Uses a
   mock `Competition` with a `MockAlgorithm` that produces
   deterministic replicate values engineered to classify as
   `Ambiguous` at N=10. Asserts that `run_rematch` runs the
   expansion phase (detectable via a call counter on the
   mock). ~50 lines. This test is the one that exercises
   the folded-pilot driver's control flow directly.
10. **`run_rematch_positive_short_circuits`.** Similar to
    (9) but with replicate values engineered to classify as
    `Positive` at N=10. Asserts `run_rematch` does *not*
    run the expansion phase. ~40 lines.

Ten tests total. Test (9) and (10) are the load-bearing
driver-level tests; tests (1) through (8) are unit-level
coverage for the analysis functions in isolation. All tests
use seeded RNGs so the bootstrap results are reproducible.

## Section 6 — The rematch test fixture (sub-decisions (g)–(h))

### 6.1 File layout

PR 3b ships one new file at `sim/L0/opt/tests/d2c_sr_
rematch.rs`. The file lives in `sim-opt`'s integration test
directory (alongside `tests/` at the crate root), not inside
`src/`. The placement matches the ml-bridge + thermostat
pattern: integration tests that need external crates (here,
`sim-mjcf` and `sim-core`) live in `tests/`, while unit
tests for the library live inline in `src/`.

The fixture is a single file of ~350 lines: constants and
MJCF XML block duplicated from `d2c_cem_training.rs`, a
`make_training_vecenv(seed)` helper constructing the SR task
with the given `master_seed`, the two algorithm builder
closures for CEM and SA, the top-level `#[test]` function
implementing the rematch, and the matched-complexity gate
assertion.

The `#[test]` function is marked `#[ignore]` with a message
matching the `d2c_cem_training.rs` convention:
`#[ignore = "requires --release (~30-60 min)"]`. The test is
gated on explicit `--ignored` flag plus `--release` mode
because the rematch's compute cost is 320M env steps in the
unambiguous path or 640M env steps in the expanded path, and
debug-mode execution would take hours where release-mode
takes tens of minutes.

### 6.2 The SR task infrastructure — sub-decision (f): duplicate

The D2c SR task's fixture infrastructure lives at
`sim/L0/thermostat/tests/d2c_cem_training.rs`: the `SR_XML`
MJCF model constant at `d2c_cem_training.rs:37-53` (recon-
reported, the `const SR_XML: &str = r#"..."#` block), the
central parameter constants at `d2c_cem_training.rs:57-62`
(recon-reported, `DELTA_V`, `X_0`, `GAMMA`, `K_B_T_BASE`,
`A_0`, `SEED_BASE`), the episode parameters at
`d2c_cem_training.rs:72-73` (recon-reported, `SUB_STEPS` and
`EPISODE_STEPS`), the signal frequency helper at
`d2c_cem_training.rs:66-68` (recon-reported, `signal_omega`),
and the `make_training_vecenv(seed)` helper at
`d2c_cem_training.rs:86-120` (recon-reported).

Ch 42's rematch fixture needs the same fixture infrastructure
— the same MJCF model, the same constants, the same
`make_training_vecenv` helper — because the rematch runs the
*same task* as D2c, just with different algorithms and
multiple replicate seeds.

Two placements were considered for sharing the infrastructure
across `d2c_cem_training.rs` and `d2c_sr_rematch.rs`.

**Placement (1): duplicate the infrastructure in the rematch
fixture.** Copy the `SR_XML` block, the constants, and the
helpers from `d2c_cem_training.rs` into
`sim-opt/tests/d2c_sr_rematch.rs`. The two files hold
independent copies. If the SR task's MJCF model or constants
ever change, both files need updating.

**Placement (2): extract the fixture infrastructure into a
shared test-support module.** Create a new module at (for
example) `sim-thermostat/tests/sr_fixture.rs` or
`sim-thermostat/src/test_support.rs` that exports `SR_XML`,
`make_training_vecenv`, and the constants as public items.
Both `d2c_cem_training.rs` and the rematch fixture import
from it. Single source of truth; one-site update for any
future changes.

**The pick: placement (1), duplicate.** Four reasons.

**First, integration tests are siloed per-crate.**
Rust's integration test layout requires integration tests at
`crate/tests/*.rs` to be self-contained per crate. A shared
test-support module in `sim-thermostat/src/test_support.rs`
would need `pub` visibility (accepting the "public API for
test use" compromise) and a `#[cfg(test)]` or feature-gate
to avoid exposing it to production builds. The
`sim-opt/tests/d2c_sr_rematch.rs` file would then depend on
`sim-thermostat`'s test support via a dev-dependency that
activates the feature. This is additional workspace
machinery for a single sharing case.

**Second, the SR task infrastructure is frozen.** D2c is
complete. The `SR_XML` model, the constants, and the reward
function are the D2 spec's output, and any future change to
them would be a different experiment, not a revision of D2c.
The duplication's maintenance cost (keeping two files in
sync) is low because the files should not change in sync —
if one changes, it is because the experiment is being
redefined, which is exactly the scope gate that should
trigger an explicit review of both files.

**Third, the user's "readability is the highest priority"
preference argues for self-contained files.** A reader of
`d2c_sr_rematch.rs` should be able to understand the fixture
without cross-referencing another file. The duplication
makes `d2c_sr_rematch.rs` read as a complete rematch
specification from top to bottom. A shared module forces the
reader to follow an import chain across crate boundaries to
find the reward function and the MJCF model.

**Fourth, scope discipline.** The scope-discipline preference
("fix the engine first, but don't grow the engine for one use
case") argues against extracting a shared module in PR 3 when
the sharing has one current consumer. If a third consumer
appears (for example, Ch 30's (a) or (b) null follow-up
rematch), the extraction becomes worth the workspace
machinery. In the meantime, ~50 lines of duplication is
cheaper than a cross-crate test-support module.

The counter-argument — "duplication is always worse than
sharing" — is a general principle that does not apply at the
scale of 50 lines of frozen-fixture code. The memory's
"don't design for hypothetical future requirements" preference
(implicit in the A-grade discipline) points at placement (1)
as the scope-disciplined pick.

### 6.3 The duplicated constants and MJCF block

The rematch fixture's top section mirrors
`d2c_cem_training.rs`'s structure:

```rust
//! D2c rematch — CEM vs SA at matched complexity on the SR task.
//!
//! Implements Ch 32's folded-pilot protocol via sim-opt's
//! `run_rematch` driver. See the ml-chassis-refactor study Ch 42
//! for the rendering argument.
//!
//! **Requires --release (~30-60 min)** because each replicate
//! runs at `Steps(16M)` budget. The initial batch is 10
//! replicates per algorithm; the expanded batch (iff ambiguous)
//! is 20 per algorithm.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::suboptimal_flops,
    clippy::doc_markdown,
    clippy::too_many_lines
)]

use std::sync::Arc;

use rand::SeedableRng;
use rand::rngs::StdRng;
use sim_core::DVector;
use sim_ml_bridge::{
    ActionSpace, Algorithm, Cem, CemHyperparams, Competition,
    LinearPolicy, ObservationSpace, Policy, TaskConfig,
    TrainingBudget, VecEnv,
};
use sim_opt::{Sa, SaHyperparams, RematchOutcome, run_rematch,
    REMATCH_MASTER_SEED, REMATCH_TASK_NAME};
use sim_thermostat::{
    DoubleWellPotential, LangevinThermostat, OscillatingField,
    PassiveStack,
};

// ─── MJCF model (duplicated from d2c_cem_training.rs:37-53) ──
const SR_XML: &str = r#"
<mujoco model="stochastic-resonance">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="particle">
      <joint name="x" type="slide" axis="1 0 0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <general name="temp_ctrl" joint="x" gainprm="0" biasprm="0 0 0"
             ctrllimited="true" ctrlrange="0 10"/>
  </actuator>
</mujoco>
"#;

// ─── Central parameters (duplicated from d2c_cem_training.rs:57-62) ──
const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T_BASE: f64 = 1.0;
const A_0: f64 = 0.3;
// Note: SEED_BASE / REMATCH_MASTER_SEED are the same literal
// (20_260_412) by design — Ch 32 Decision 4 matched the rematch's
// master to the D2c legacy test's SEED_BASE for three-way
// consistency (Ch 23 §1.2, Ch 32 §4.6, d2c_cem_training.rs:62).
// The rematch fixture imports REMATCH_MASTER_SEED from sim_opt.

const KRAMERS_RATE: f64 = 0.01214;

fn signal_omega() -> f64 {
    2.0 * std::f64::consts::PI * KRAMERS_RATE
}

// ─── Episode parameters (duplicated from d2c_cem_training.rs:72-73) ──
const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 5_000;

// ─── Training constants ──────────────────────────────────────
const N_ENVS: usize = 32;
const OBS_DIM: usize = 2;
const ACT_DIM: usize = 1;
const REMATCH_BUDGET_STEPS: usize =
    N_ENVS * EPISODE_STEPS * 100; // 16M = 32 * 5000 * 100

fn obs_scale() -> Vec<f64> {
    vec![1.0 / X_0, 1.0]
}
```

The imports, constants, and helpers mirror
`d2c_cem_training.rs:1-82` (recon-reported) with two
differences: the `SEED_BASE` constant is replaced by the
imported `REMATCH_MASTER_SEED` from `sim_opt`, and the
rematch-specific `REMATCH_BUDGET_STEPS` constant is defined
explicitly as `N_ENVS * EPISODE_STEPS * 100 = 16_000_000`.
Both differences are rematch-specific; the SR task
infrastructure itself (MJCF, physical constants, episode
parameters) is a verbatim duplication.

### 6.4 `make_training_vecenv` with per-replicate seed

The helper function constructs a fresh `VecEnv` for the
rematch's per-replicate runs, taking the `master_seed` as
a parameter. Under Ch 40 PR 1b, the `LangevinThermostat`
is per-env (via `BatchSim::new_per_env` and the `PerEnvStack`
trait), and each env's thermostat uses `(gamma, k_b_t,
master_seed, traj_id = env_index)`:

```rust
fn make_training_vecenv(master_seed: u64, n_envs: usize) -> VecEnv {
    // Under Ch 40 PR 1b's per-env hosting path, BatchSim::new_per_env
    // takes a factory closure |i| -> (Model, Arc<PassiveStack>)
    // that constructs each env's physics stack independently.
    // All 32 envs share the same `master_seed` (the per-replicate
    // physics seed this closure receives from the patched Ch 41
    // `TaskConfig::build_fn` signature), but each env gets a
    // distinct `traj_id = i` so the per-env LangevinThermostat
    // threads produce independent noise sequences from the same
    // master key. This is the per-env hosting pattern Ch 40
    // §3.3's `PerEnvStack` trait and `EnvBatch<S>` generic render.
    //
    // The concrete shape is a VecEnv::builder_per_env(...)
    // helper that Ch 40 PR 1b adds alongside VecEnv::builder, or
    // a new VecEnv construction path that uses BatchSim::new_per_env
    // internally. Ch 40 §3.3's rendering of the specific helper
    // shape is the authoritative source for the closure signature;
    // Ch 42 mirrors it here without re-rendering.
    //
    // This sketch uses the shared-model VecEnv::builder with a
    // single-thermostat PassiveStack as a rendering placeholder.
    // Post-PR-1b, it should read as a per-env factory closure
    // where each call to the closure constructs a fresh
    // (Model, Arc<PassiveStack>) pair with LangevinThermostat::new(
    // gamma, K_B_T_BASE, master_seed, i as u64) — master_seed
    // constant, traj_id = env index.

    let mut model = sim_mjcf::load_model(SR_XML).unwrap();
    let omega = signal_omega();

    // Placeholder rendering: single LangevinThermostat with
    // traj_id=0, matching the current d2c_cem_training.rs:91
    // pattern. Post-PR-1b, this becomes a per-env factory
    // closure that sets traj_id = env_index per env while
    // keeping master_seed constant across the replicate's
    // 32-env batch.
    let thermostat = LangevinThermostat::new(
        DVector::from_element(model.nv, GAMMA),
        K_B_T_BASE,
        master_seed,
        0,
    )
    .with_ctrl_temperature(0);
    let double_well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let signal = OscillatingField::new(A_0, omega, 0.0, 0);

    PassiveStack::builder()
        .with(thermostat)
        .with(double_well)
        .with(signal)
        .build()
        .install(&mut model);

    let model = Arc::new(model);
    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .unwrap();
    let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

    VecEnv::builder(model, n_envs)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(move |_m, data| data.qpos[0].signum() * (omega * data.time).cos())
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > (EPISODE_STEPS as f64) * (SUB_STEPS as f64) * 0.001)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap()
}
```

The helper is ~35 lines, matching `d2c_cem_training.rs:86-120`
almost line-for-line with two differences: the `seed`
parameter is renamed to `master_seed` for clarity (Ch 40's
4-arg constructor uses the same name), and the `n_envs`
parameter is added so the helper is reusable at both N_ENVS
= 32 for the rematch and smaller N for unit tests.

### 6.5 The `TaskConfig` for the rematch

The rematch constructs a single `TaskConfig` whose `build_fn`
closure uses its `seed: u64` parameter directly. Under the §2
patched `Fn(usize, u64)` signature, each call from
`Competition::run_replicates` at `build_vec_env(n_envs, seed)`
constructs a fresh `VecEnv` with the `LangevinThermostat`
seeded at `seed`'s value — meaning all 10 (or 20) replicates
share the same `TaskConfig`, and the per-replicate physics
variation comes from the closure's seed parameter, not from
a per-replicate `TaskConfig` rebuild.

```rust
fn rematch_task() -> TaskConfig {
    TaskConfig::builder()
        .name(REMATCH_TASK_NAME)
        .obs_dim(OBS_DIM)
        .act_dim(ACT_DIM)
        .obs_scale(obs_scale())
        .build_fn(|n_envs, seed| {
            Ok(make_training_vecenv(seed, n_envs))
        })
        .build()
}
```

The closure is ~3 lines: it takes `(n_envs, seed)` and calls
`make_training_vecenv(seed, n_envs)` to produce the `VecEnv`.
The `seed` parameter is the per-replicate master seed that
`Competition::run_replicates` threads through from its seeds
slice under the §2 patch; the `make_training_vecenv` helper
uses it to construct the `LangevinThermostat` with the
correct `master_seed` (matching the `d2c_cem_training.rs:91`
pattern but with the per-replicate value instead of the
fixture-global `SEED_BASE`).

A subtlety worth naming: under §2's patched
`TaskConfig::build_fn`, each call to `build_vec_env` inside
`run_replicates`'s inner loop constructs a fresh `VecEnv`
from scratch — the MJCF model is re-parsed, the
`PassiveStack` is rebuilt, the `LangevinThermostat` is
re-instantiated with the current replicate's seed, and the
`VecEnv::builder` returns a new `VecEnv`. This is not a
performance optimization; it is the freshness discipline
Ch 41 §2.1 already committed to ("fresh per-pair `VecEnv`
construction at `competition.rs:330`, recon-reported"),
extended to per-replicate freshness under the §2 patch.
The MJCF parsing overhead is small (~1 ms per parse per
the existing `make_training_vecenv` pattern) and is
dwarfed by the 16M env steps the replicate actually runs.

Under this shape, there is no Ch 42-owned outer loop in
`run_rematch` at §5.5 — the single `run_replicates` call
with a 10-element seeds slice handles the cross-replicate
iteration internally, and the rematch fixture's test
function at §6.6 constructs exactly one `TaskConfig` and
passes it to `run_rematch` as a single reference. The §2
patch is what makes this shape possible; without it, the
`build_fn` closure would have no seed channel and the
rematch would have to capture a per-replicate value,
forcing the outer-loop workaround that §2.3's rendering (i)
describes and §2.4 rejects.

### 6.6 The rematch `#[test]` function

The top-level test function at `sim-opt/tests/d2c_sr_
rematch.rs`:

```rust
#[test]
#[ignore = "requires --release (~30-60 min)"]
fn d2c_sr_rematch() {
    // ── Matched-complexity gate (Ch 23 §3, Ch 31 §4.3) ──
    let cem_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let sa_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    assert_eq!(
        cem_policy.n_params(),
        sa_policy.n_params(),
        "matched-complexity gate: CEM and SA must use same n_params"
    );
    assert_eq!(
        cem_policy.n_params(),
        3,
        "D2c rematch uses LinearPolicy(2, 1) with n_params = 3 (Ch 23 §3.1)"
    );

    // ── Build the Competition runner ──
    // seed=0 is unused under run_rematch because the per-replicate
    // seeds come from splitmix64(MASTER + i), threaded through
    // run_replicates's per-call seeds slice. The Competition's
    // stored seed only matters for the single-seed run() method,
    // which the rematch does not call.
    let competition = Competition::new(
        N_ENVS,
        TrainingBudget::Steps(REMATCH_BUDGET_STEPS),
        0,
    );

    // ── Algorithm builders ──
    let cem_builder = |_task: &TaskConfig| -> Box<dyn Algorithm> {
        let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
        // Match d2c_cem_training.rs:278's non-zero init so CEM
        // starts at the same bias CEM's D2c training used.
        policy.set_params(&[0.0, 0.0, 2.0]);
        Box::new(Cem::new(
            Box::new(policy),
            CemHyperparams {
                elite_fraction: 0.2,
                noise_std: 2.5,
                noise_decay: 0.98,
                noise_min: 0.1,
                max_episode_steps: EPISODE_STEPS,
            },
        ))
    };

    let sa_builder = |_task: &TaskConfig| -> Box<dyn Algorithm> {
        let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
        // Match CEM's non-zero init so SA starts at the same point
        // as CEM's D2c run. This is not matched-complexity — that
        // is the `n_params` assertion above — but it is
        // matched-initial-conditions, which removes initialization
        // asymmetry from the comparison.
        policy.set_params(&[0.0, 0.0, 2.0]);
        Box::new(Sa::new(
            Box::new(policy),
            SaHyperparams {
                initial_temperature: 50.0,
                proposal_std: 0.5,
                cooling_decay: 0.955,
                temperature_min: 0.005,
                max_episode_steps: EPISODE_STEPS,
            },
        ))
    };

    let algorithm_builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> =
        vec![&cem_builder, &sa_builder];

    // ── Bootstrap RNG ──
    // Pre-registered seed for the bootstrap resampling RNG.
    // Ch 32 §7 explicitly defers the bootstrap RNG seed to Ch 42
    // with the note that "the rematch writeup should name the
    // bootstrap RNG seed for full reproducibility." Ch 42 picks
    // this value as the literal 0xB007_57AP_5EEDu64-style
    // rendering with a fixed numeric value.
    const BOOTSTRAP_RNG_SEED: u64 = 0xB007_0057_00AA_0055; // "BOOTSTRAP AA55"
    let mut bootstrap_rng = StdRng::seed_from_u64(BOOTSTRAP_RNG_SEED);

    // ── Build the single TaskConfig for the rematch ──
    let task = rematch_task();

    // ── Run the rematch ──
    eprintln!("\n--- D2c SR rematch: folded-pilot protocol ---");
    eprintln!("Master seed: {REMATCH_MASTER_SEED} (matches d2c_cem_training.rs:62)");
    eprintln!("Initial N: 10, expanded N (iff ambiguous): 20");
    eprintln!("Budget per replicate: Steps({REMATCH_BUDGET_STEPS})");

    let outcome = run_rematch(
        &competition,
        &task,
        &algorithm_builders,
        &mut bootstrap_rng,
    )
    .expect("rematch protocol should complete without env errors");

    eprintln!("\n--- Rematch outcome: {outcome:?} ---");

    // ── Test gate: protocol completes cleanly (sub-decision g) ──
    // The test does NOT assert a specific outcome (positive /
    // null / ambiguous). Ch 30 names all three as informative,
    // and asserting one would beg the question. Instead, the
    // test passes if the protocol ran to completion without
    // panicking and produced some `RematchOutcome`. The verdict
    // is emitted via eprintln so a reader of the test log sees
    // it — matching d2c_cem_training.rs's per-algorithm eprintln
    // gates pattern.
    match outcome {
        RematchOutcome::Positive => eprintln!("CLASSIFICATION: positive"),
        RematchOutcome::Null => eprintln!("CLASSIFICATION: null"),
        RematchOutcome::Ambiguous => eprintln!("CLASSIFICATION: ambiguous (N=20 expanded)"),
    }
}
```

The function is ~90 lines. Its structure: (1) matched-
complexity gate assertion, (2) `Competition` construction
with the 16M-step budget, (3) algorithm builders for CEM
and SA with hyperparameters inherited from their respective
chapters, (4) bootstrap RNG construction with a
pre-registered seed, (5) the `run_rematch` call, and (6)
the verdict-print-and-pass logic implementing sub-decision
(g).

### 6.7 Sub-decision (g): the test gate assertion shape

Ch 42 picks **(α) protocol-completes-cleanly** as the
rematch test gate's assertion shape. Three alternatives were
considered.

**Shape (α): protocol-completes-cleanly.** The test asserts
(1) the matched-complexity gate holds, (2)
`run_rematch` returns `Ok(outcome)` without panicking or
producing an `EnvError`, and (3) some `RematchOutcome`
variant is produced. The specific outcome is printed via
eprintln and is the scientific result; the test's pass/fail
is orthogonal to which outcome was produced.

**Shape (β): data-generator.** The test serializes the
raw replicate vectors and the bootstrap CI to a JSON file
(e.g., `target/test-artifacts/d2c_sr_rematch.json`) and
always passes. The test is a data producer rather than a
gate. A reader of the rematch's results reads the JSON file
after the test runs; a writeup author reads the same JSON.

**Shape (γ): smoke-gate.** The test asserts protocol
completion plus a loose sanity check that both algorithms'
replicate vectors contain finite values within a reasonable
range (e.g., all per-episode totals in `[-5000, +5000]`).
Weaker than asserting a specific outcome; stronger than
pure completion.

**The pick: shape (α).** Four reasons.

**First, Ch 30 explicitly names all three outcomes as
informative.** Ch 30 §"What each outcome would tell us"
treats positive, null, and ambiguous as three distinct
informative outcomes, each prescribing a different next
experiment. A test that asserts a specific outcome would
bake in a prior about which outcome is expected, which
contradicts Ch 30's scope discipline. A test that fails on
an "unexpected" outcome would mean the rematch's scientific
result depends on whether the test passes or fails, which
inverts the purpose of the experiment — the rematch *is* the
test, and its outcome *is* the scientific result.

**Second, the `d2c_cem_training.rs` legacy file's gate shape
is the precedent.** That file's tests assert per-algorithm
Gate A (synchrony t-statistic significance) and Gate B
(monotone learning), both within-algorithm gates. The
rematch's cross-algorithm comparison verdict is printed to
eprintln in `d2c_cem_training.rs` as a human-readable
summary, not asserted as a test gate. Ch 42's rematch
fixture follows the same convention: within-algorithm
correctness is asserted (via `run_rematch`'s own error
handling), and the cross-algorithm verdict is printed for
human reading.

**Third, shape (β)'s data-generator form is a scope
widening.** Shape (β) would add JSON serialization
boilerplate to the rematch fixture, plus a
`target/test-artifacts/` directory convention, plus a
writeup-generation step that reads the JSON. None of these
are currently in scope for PR 3. If a future writeup
pipeline wants serialized replicate data, it can be added
in a post-PR-3 commit that reads the rematch's eprintln
output or adds the serialization at that time. Shape (α)
does not foreclose shape (β); it is the minimal-scope form.

**Fourth, shape (γ)'s sanity check is speculative.** The
sanity check would assert bounds on the rematch's replicate
values that Ch 42 has no way to verify are correct without
running the rematch and seeing what values it produces. A
bound of `[-5000, +5000]` on per-episode totals would pass
for any rematch run, positive, null, or ambiguous, so it
adds no gate strength beyond shape (α). A tighter bound
would be a specific prediction about the rematch's outcome,
which Ch 42 explicitly declines to make.

Shape (α) is the pick. §9 names the scope-widening moves
(serialization to JSON, test-fixture-as-data-generator) as
deferred future-PR concerns.

### 6.8 Integration test versus inline unit test

One placement question worth naming even though it is not a
sub-decision: why does the rematch fixture live at
`sim-opt/tests/d2c_sr_rematch.rs` (an integration test) and
not at `sim-opt/src/analysis.rs`'s `#[cfg(test)] mod tests`
(inline unit test)?

The answer is that the rematch fixture needs external
crates (`sim-mjcf` for loading the MJCF model, `sim-core`
for `DVector` and `Model`, `sim-thermostat` for
`LangevinThermostat` and `PassiveStack`) as dev-dependencies
that the crate's library code does not take. Rust's
integration test directory at `sim-opt/tests/` is the right
home for a test file that needs dev-dependencies beyond what
the library uses. Moving it to `src/`'s inline test module
would force those crates to be regular dependencies instead
of dev-dependencies, which would pollute the crate's runtime
dependency graph for no runtime benefit.

The ten unit tests for the analysis module at §5.6 are
inline in `src/analysis.rs` because they only need
hand-crafted `Vec<f64>` inputs — no external chassis — so
they can be unit tests. The rematch fixture needs the full
chassis stack, so it is an integration test. The split is
Rust-convention-aligned and matches the `sim-thermostat`
crate's own `tests/d2c_cem_training.rs` at the integration
level.

## Section 7 — In-chapter sub-decisions summary

Ch 42 made ten in-chapter sub-decisions — the calls Ch 23,
Ch 24, Ch 30, Ch 31, Ch 32, Ch 40, and Ch 41 did not lock and
Ch 42 had to pick. Table 1 names them, records the pick, and
summarizes the reasoning in one line. A reader can use this
table as a quick index into the chapter's sections.

| # | Sub-decision | Pick | One-line rationale |
|---|---|---|---|
| (a) | `TaskConfig::build_fn` seed extension + Ch 41 bundled patch | Extend signature to `Fn(usize, u64) -> Result<VecEnv, EnvError>`; thread seed through `run_replicates`; amend Ch 41 §1.1/§2.1/§5 + review log in the Ch 42 commit per the `843dc21c` precedent | Ch 32 §4.8's skeleton is unrenderable under Ch 41's seed-blind `build_fn`; fix-the-engine-first preference; breaking-change-over-hack preference; rematch fixture reads like the Ch 32 skeleton |
| (b) | sim-opt crate placement | `sim/L0/opt/` as L0 peer of ml-bridge and thermostat; `sim-ml-bridge` + `sim-thermostat` as runtime deps; `sim-mjcf` + `sim-core` as dev-deps | Ch 23/31/32 already name `sim-opt`; pivot spec's 3-crate split begins here; analysis module belongs with SA, not with ml-bridge's RL surface |
| (c) | SA cooling schedule | Geometric: `T_{k+1} = T_k * 0.955` with `T_min = 0.005` | Conventional SA default; composes cleanly with `Steps(16M)` → 100 epochs; linear and logarithmic don't match budget shape; adaptive is overkill and contaminates the rematch's basic-SA framing |
| (d) | SA `initial_temperature` + `proposal_std` defaults | `initial_temperature = 50.0`, `proposal_std = 0.5`, tunable via `SaHyperparams` | `T_0 = 50` calibrated to D2c reward scale (~490 peak per-episode total) so early epochs admit meaningful downhill proposals; `proposal_std = 0.5` matches 1/5 of CEM's `noise_std = 2.5` for the 3-param space |
| (e) | SA fitness evaluation shape | Multi-env averaging (shape ii): run `n_envs` episodes per epoch, average per-episode totals into one fitness scalar, 100 Metropolis steps per rematch | Shape (i) breaks Ch 22 budget formula or wastes parallelism; shape (iii) is Parallel Tempering (Ch 30 null follow-up, contaminates); shape (iv) breaks wall-clock parity with CEM |
| (f) | SR task infrastructure for the rematch fixture | Duplicate `SR_XML` + constants + `make_training_vecenv` from `d2c_cem_training.rs` into `sim-opt/tests/d2c_sr_rematch.rs` (~50 lines) | Integration tests are siloed per-crate; SR task is frozen post-D2c; self-contained files serve the readability preference; scope discipline against premature extraction |
| (g) | Rematch test gate assertion shape | Protocol-completes-cleanly (α): assert matched-complexity gate + `run_rematch` returns `Ok(outcome)`; verdict via eprintln | Ch 30 names all three outcomes as informative; asserting a specific outcome begs the question; matches `d2c_cem_training.rs`'s within-algorithm-gates + cross-algorithm-eprintln pattern |
| (h) | Analysis module placement | Library module at `sim-opt/src/analysis.rs` with `pub mod analysis` re-exported from `lib.rs` | Ch 30's null follow-ups (richer SA proposal, Parallel Tempering) reuse the machinery; unit-level tests possible; reader-navigation clarity; R34 chassis-overbuild form |
| (i) | PR 3 split structure | 2-PR split: PR 3a = sim-opt crate + SA + analysis module (additive, merges after PR 2a). PR 3b = rematch test fixture (depends on PR 1b + PR 2b + PR 3a all merged) | Mirrors Ch 40 and Ch 41's 2-PR pattern; PR 3a unblocks future sim-opt consumers without waiting for the full Part 4 stack; PR 3b is the narrow risk-heavy half |
| (j) | Writeup artifact handling | Narrow scope: PR 3 ships no writeup; the rematch writeup is deferred to a future post-execution commit after someone runs `cargo test --release --ignored d2c_sr_rematch` | Ch 32 §7 explicitly defers writeup format to writeup author; writing a writeup for an unexecuted experiment violates scope discipline; matches Ch 40/41 "PR ships code, not results" pattern |

Sub-decision (a) is the one worth the most reader attention.
It is a bundled amendment to a committed upstream chapter
(Ch 41), which makes it a structurally different kind of
sub-decision from the other nine — those are Ch 42-internal
calls about sim-opt's own layout or SA's own shape, while
(a) is a scope-extension Ch 42 is importing into Ch 41's
committed PR 2a plan. §2 defends the call in full, and §2.5
bounds its footprint against the `843dc21c` precedent's
footprint.

**What Ch 42 does *not* treat as sub-decisions.** The
following are mentioned in Ch 42 but inherited wholesale
from Ch 23, Ch 24, Ch 30, Ch 31, Ch 32, Ch 40, or Ch 41, and
a reader should *not* read them as in-chapter calls:

- The rematch pool being `{CEM, SA}` with PPO/TD3/SAC
  excluded — Ch 23 §2.
- The matched-complexity anchor as `LinearPolicy(2, 1)` with
  `n_params = 3` — Ch 23 §3.1.
- The `run_replicates(tasks, builders, seeds)` API shape —
  Ch 23 §1.3.
- The Ch 24 Decision 1 unit for `mean_reward` ("mean
  per-episode total across `n_envs`") — Ch 24 §3.
- The `replicate_best_rewards` raw-vector primitive and
  `SeedSummary` convenience struct — Ch 24 Decision 2.
- The three-outcome framing (positive / null / ambiguous) and
  the Ch 30 null follow-up list (richer SA proposal,
  Parallel Tempering) — Ch 30.
- The pre-committed "allowed null follow-ups on the same
  task" discipline — Ch 30.
- The bootstrap-CI-on-means test family, `B = 10_000`,
  two-sided 95% percentile bootstrap — Ch 32 Decision 1.
- The CI-to-outcome classification table — Ch 32 §3.3.
- The folded-pilot protocol (N_initial = 10, N_expanded = 20
  iff ambiguous, no intermediate stops) — Ch 32 Decision 3.
- The master seed `20_260_412` and splitmix64 derivation —
  Ch 32 Decision 4, matching `d2c_cem_training.rs:62` and
  Ch 23 §1.2.
- The Pearson-based bimodality coefficient with `BC > 5/9`
  threshold and median-based escalation — Ch 32 §6.
- The per-replicate warmup-gate `5×` factor (currently moot
  for `{CEM, SA}`) — Ch 32 Decision 5.
- The matched-complexity one-line equality assertion as the
  rematch fixture's pre-run gate — Ch 23 §3.2 + Ch 31 §4.3.
- The Gaussian proposal family for SA — Ch 30 §"What
  'physics-aware beats generic RL' is supposed to mean."
- The non-zero policy initialization `[0.0, 0.0, 2.0]` —
  inherited from `d2c_cem_training.rs:278` (recon-reported)
  as the D2c CEM baseline's starting point; the rematch
  uses the same initialization for both CEM and SA so the
  comparison is matched-initial-conditions as well as
  matched-complexity.
- The `prf::splitmix64` primitive's presence in
  `sim-thermostat` — Ch 40 PR 1a.
- The `BatchSim::new_per_env` + `PerEnvStack` trait for
  per-env `LangevinThermostat` hosting — Ch 40 PR 1b +
  §3.3.
- The 4-argument `LangevinThermostat::new(gamma, k_b_t,
  master_seed, traj_id)` constructor — Ch 40 PR 1b via D2.
- The unit-uniform `mean_reward` contract across CEM, TD3,
  SAC after PR 2b — Ch 41 PR 2b / Ch 24 Decision 1.

When a PR 3a or PR 3b reviewer challenges any of these, the
correct response is to point at the owning argument chapter
and the relevant section number, not to re-argue the call
in the PR's review comments.

## Section 8 — PR split and landing sequence (sub-decision (i))

### 8.1 The 2-PR split: PR 3a additive, PR 3b the fixture

Ch 42's sub-decision (i) is whether PR 3 ships as one PR,
two PRs, or three PRs. Three shapes are worth naming:

- **Shape (i): one PR.** The entire sim-opt crate, SA
  implementation, analysis module, and rematch test fixture
  land in a single review cycle. Smallest PR count, single
  reviewer pass over the full diff.
- **Shape (ii): two PRs — PR 3a additive crate + algorithm +
  analysis, PR 3b the rematch test fixture.** PR 3a ships
  the `sim-opt` crate creation, the `Sa` / `SaHyperparams`
  types with the `Algorithm` trait impl, the analysis module
  with bootstrap / bimodality / classify / run_rematch, and
  the unit tests for both (at `src/algorithm.rs`'s test
  module and `src/analysis.rs`'s test module). PR 3a is
  purely additive — it creates a new crate and has no
  consumers in ml-bridge, thermostat, sim-core, or any
  existing test fixture at merge time. PR 3b ships
  `sim-opt/tests/d2c_sr_rematch.rs` (the rematch fixture)
  and depends on PR 1b, PR 2b, and PR 3a all merged.
- **Shape (iii): three PRs.** PR 3a = sim-opt crate + SA only.
  PR 3b = analysis module. PR 3c = rematch fixture. Three
  review cycles, finest-grained rollback capability, maximum
  review surface fragmentation.

**The pick: shape (ii), 2-PR split.** Four reasons.

**First, shape (ii) mirrors Ch 40 and Ch 41's pattern.** Both
PR 1 and PR 2 split additive-vs-semantic halves: PR 1a
additive `prf.rs`, PR 1b semantic `LangevinThermostat`
rewrite; PR 2a additive `run_replicates`, PR 2b semantic
train-loop fix. PR 3's analog: PR 3a additive new crate,
PR 3b the rematch fixture (which is the "uses the new crate
plus the upstream chassis changes" semantic half). The
pattern is internally consistent across Part 4.

**Second, PR 3a can merge independently of PR 1b and PR 2b.**
PR 3a's code — the sim-opt crate, SA, the analysis module —
has no runtime dependency on Ch 40's `LangevinThermostat`
rewrite or Ch 41's train-loop fix. SA exercises the
`Algorithm` trait with `reaching_2dof()` in its unit tests
(deterministic physics, no Langevin); the analysis module
exercises bootstrap + bimodality + classify with hand-crafted
`Vec<f64>` inputs (no physics at all). PR 3a's test suite
passes under any combination of PR 1a / PR 1b / PR 2a /
PR 2b being merged or not. This makes PR 3a genuinely
independent of the rest of the Part 4 stack and merges as
soon as PR 2a is in `main` (needing only `run_replicates`
and `replicate_best_rewards` to appear in the type
signatures of `run_rematch` and the analysis module).

**Third, PR 3b's scope is the rematch fixture only, which
is small and focused.** PR 3b is ~350 lines of test fixture
code (the duplicated SR task infrastructure, the
`rematch_task_builder` function, the `d2c_sr_rematch` test
function, and the test-level imports). Its review surface
is the fixture's correctness and its agreement with Ch 32's
protocol. Reviewers can read it end-to-end without also
having to understand sim-opt's internal module structure,
which was reviewed in PR 3a.

**Fourth, shape (iii) is over-fragmentation.** Splitting SA
from the analysis module would produce two PRs that each
review less than ~200 lines of greenfield code, plus a
third PR for the fixture. The review surface fragmentation
is not matched by a review-complexity reduction — SA and
the analysis module are independent enough to review
together (no cross-imports between `algorithm.rs` and
`analysis.rs` at the source level), but separating them
into two PRs forces reviewers to load crate context twice.
Shape (iii)'s finest-grained rollback capability is
theoretical — no one rolls back a module within a new
crate when the whole crate can be backed out — and the cost
(three review cycles instead of two) is real. Rejected.

**Shape (i)'s single-PR form** is the alternative that has
a genuine case: one review cycle means one reviewer pass
over the whole Ch 42 output, and the reviewer sees SA and
the analysis module and the fixture together as one
coherent artifact. The case against it is that shape (i)'s
PR is larger (~600 lines of greenfield code in src/ plus
~350 lines of test fixture), and the fixture's review
surface is materially different from the library code's
review surface. A reviewer who wants to focus on the
rematch's protocol correctness has to also page through
the crate-creation boilerplate and the SA hyperparameter
argument; a reviewer focused on SA's correctness has to
also page through the rematch's MJCF block and the Ch 30
three-outcome-eprintln boilerplate. Shape (ii) lets each
review cycle focus on its proper subject.

### 8.2 PR 3a contents and risk

PR 3a ships:

- `sim/L0/opt/Cargo.toml` — new crate manifest per §3.1.
- `sim/L0/opt/src/lib.rs` — crate root with `pub mod`
  declarations and re-exports per §3.2.
- `sim/L0/opt/src/algorithm.rs` — `Sa`, `SaHyperparams`,
  `Algorithm` trait impl, the `evaluate_fitness` helper,
  the `policy_clone_with_params` helper, and ~120 lines of
  `#[cfg(test)] mod tests` with four SA unit tests per §4.8.
- `sim/L0/opt/src/analysis.rs` — `BootstrapCi`,
  `RematchOutcome`, `bootstrap_diff_means`,
  `bootstrap_diff_medians`, `bimodality_coefficient`,
  `classify_outcome`, `run_rematch`, `test_and_classify`,
  `REMATCH_MASTER_SEED`, `N_INITIAL`, `N_EXPANDED`,
  `REMATCH_TASK_NAME`, plus internal `mean` / `median`
  helpers, plus ~300 lines of `#[cfg(test)] mod tests` with
  ten analysis unit tests per §5.6.
- Workspace `Cargo.toml` amendment — one `"sim/L0/opt"`
  line added to the L0 members section at `Cargo.toml:306`
  (recon-reported, after `"sim/L0/thermostat"`), and one
  `sim-opt = { path = "sim/L0/opt" }` line added to the
  L0 dependencies section at `Cargo.toml:481` (recon-
  reported, after `sim-thermostat`).

**Total diff size estimate:** ~800 lines added, 2 lines
modified (the workspace Cargo.toml amendments). Source
files: ~500 lines of library code, ~420 lines of unit tests.
All added, no deletions.

**Risk profile:** low. PR 3a has zero consumers at merge
time — no existing file imports from `sim-opt` — and its own
tests are self-contained. The only way PR 3a can fail
review is if the SA implementation has a bug (caught by the
four unit tests) or the analysis functions have a bug
(caught by the ten unit tests) or the Cargo.toml has a
syntax error (caught by `cargo check`). A reviewer can
evaluate PR 3a by reading the new files plus running
`cargo test -p sim-opt` to confirm the tests pass.

**Rollback path:** if PR 3a is merged and a bug is
discovered, the rollback is a `git revert` of the merge
commit, which removes the sim-opt crate and the workspace
amendments cleanly. No other crate is affected because no
other crate imports from `sim-opt`.

### 8.3 PR 3b contents and risk

PR 3b ships:

- `sim/L0/opt/tests/d2c_sr_rematch.rs` — the rematch test
  fixture per §6, ~350 lines.
- (No source-file changes.) PR 3b is tests-only.

**Total diff size estimate:** ~350 lines added. One file.

**Risk profile:** medium, driven by two concerns.

**First, the rematch fixture depends on the full chassis
stack.** PR 1a, PR 1b, PR 2a, PR 2b, and PR 3a must all be
in `main` before PR 3b can be merged. If any of the
upstream merges has lingered or been rolled back, PR 3b's
review cycle has to wait. The dependency chain is long.

**Second, the rematch fixture's `#[ignore]` + `--release`
requirement means CI does not exercise it automatically.**
The fixture runs for 30-60 minutes per full rematch,
which exceeds any reasonable CI budget. A reviewer cannot
verify "the rematch produces any specific outcome" without
running the ignored test manually on appropriate hardware.
The reviewer can verify that (a) the fixture compiles under
`cargo check --tests -p sim-opt`, (b) the matched-complexity
gate assertion is present and correct, (c) the
`run_rematch` call site uses the correct builders and
seed-threading shape, and (d) the fixture's constants match
Ch 32's specification (`MASTER = 20_260_412`,
`N_INITIAL = 10`, etc.). These are necessary but not
sufficient for rematch correctness; actual rematch
correctness is exercised only when someone runs the
ignored test on hardware that can afford the compute.

**Rollback path:** PR 3b is tests-only, so a rollback is a
`git revert` of the test file only, with no effect on any
production code.

### 8.4 Merge-order alternatives

The formal partial order is:

```
PR 1a ──┐
        ├──> PR 1b
        │
PR 2a ──┼──> PR 2b
        │
        └──> PR 3a ──> PR 3b
                       ^
                       |
        (also needs PR 1b + PR 2b)
```

Within the partial order, four concrete orderings are
reasonable:

1. **Conservative sequential.** PR 1a → PR 1b → PR 2a → PR 2b
   → PR 3a → PR 3b. Six merges in strict dependency order.
   Matches Ch 41 §4.4's conservative-ordering option.
2. **Interleaved additive-first.** PR 1a → PR 2a → PR 3a
   (all additive halves) → PR 1b → PR 2b → PR 3b. Additive
   halves all land first, semantic halves all land second.
   Maximizes parallel review of the additive halves once PR
   1a and PR 2a have cleared.
3. **PR 3a as soon as PR 2a merges.** PR 1a → PR 2a → PR 3a
   (PR 3a merges here, without waiting for PR 1b / PR 2b)
   → PR 1b → PR 2b → PR 3b. The ordering exploits PR 3a's
   independence from PR 1b and PR 2b. PR 3a can land any
   time after PR 2a is in `main`.
4. **Fully parallel.** All six PRs in review concurrently,
   merging as each clears review subject to the partial
   order. Ch 41 §4.4 allowed this for the first four PRs;
   Ch 42 allows it for all six with the PR 3b constraint
   that PR 1b + PR 2b + PR 3a all merge before PR 3b.

**The lean is ordering (3).** PR 3a lands as soon as PR 2a
is in `main`, which unblocks Ch 30's pre-committed null
follow-ups (richer SA proposal, Parallel Tempering) from
the chassis layer while PR 1b and PR 2b go through their
own review cycles. PR 3b waits for the full stack as it
must.

The other orderings are also acceptable. Ch 42 does not
impose a strict order beyond the partial-order constraints;
ordering (3) is the lean on efficiency grounds.

### 8.5 The Ch 41 amendment's merge interaction

The Ch 41 amendment bundled in this chapter's §2 is a plan-
chapter edit, not a source-code edit. It lands in the Ch 42
commit (alongside the chapter draft and review log) and
takes effect the moment the Ch 42 commit is reviewed and
approved by the user. The actual source-code ripple from
the amendment — the `TaskConfig::build_fn` signature
extension, the `run_replicates` body update, the stock-task
ignored-seed additions — happens when PR 2a ships.

Two scenarios worth naming:

**Scenario A: PR 2a has not yet been opened as a PR.** This
is the current state as of Ch 42's drafting. The Ch 41
amendment updates the plan chapter; when someone later
opens PR 2a, they read the amended Ch 41 and ship the
extended signature from the first commit. No rework.

**Scenario B: PR 2a has been opened and is in review.** A
PR 2a reviewer sees the original Ch 41 plan (without the
amendment) and the pre-amendment signature. The Ch 42
commit adds the amendment to Ch 41, which invalidates the
PR 2a code-level diff (because the signature now needs to
include the seed parameter). PR 2a's author would need to
update the PR with the extended signature, and the reviewer
would need to re-check the updated diff against the amended
plan. This is the rework case.

Scenario B is avoided by the ordering choice: if PR 2a has
not yet been opened, Ch 42's commit amends the plan first,
and PR 2a ships the extended signature on its first
iteration. This is ordering (3)'s practical argument — PR
3a can merge early, but the Ch 42 commit should land
*before* PR 2a's actual source-code review begins, so PR
2a's author reads the amended plan.

The concrete sequencing: Ch 42 commits first (drafting +
review log + Ch 41 amendment). Then PR 1a, PR 2a, PR 1b,
PR 2b, PR 3a, PR 3b open and merge per the partial order.
The Ch 42 commit is the "amended plan is now the plan"
checkpoint; all PRs open against the amended plan.

### 8.6 Rollback paths summarized

Each PR has its own rollback path:

- **PR 3a rollback.** `git revert` the merge commit. Removes
  the sim-opt crate and the workspace Cargo.toml lines. No
  effect on ml-bridge, thermostat, sim-core, or any existing
  test fixture. Clean.
- **PR 3b rollback.** `git revert` the merge commit. Removes
  the `d2c_sr_rematch.rs` file. No effect on any other file.
  Clean.
- **Ch 42 commit rollback (Ch 41 amendment).** The Ch 42
  commit is a documentation commit affecting only the
  ml-chassis-refactor study book's source and review-log
  files. A rollback via `git revert` removes Ch 42, removes
  the Ch 42 review log, and removes the Ch 41 amendment +
  Ch 41 review log Round 2 section. It does *not* affect any
  source code. After the rollback, PR 2a's plan reverts to
  its pre-amendment state; if PR 2a has not yet opened,
  rollback has no cascading effect. If PR 2a has opened
  with the extended signature, rollback means the PR 2a
  code-level diff no longer matches Ch 41's plan, and the
  PR 2a author would need to either (a) revert the signature
  extension (losing the rematch's physics-seed channel, which
  is load-bearing) or (b) re-open a new version of the
  Ch 42 commit to restore the amendment. This is the
  awkward-rollback case, and it is why Ch 42's commit should
  only happen when the user is confident the rematch's
  scope is right.

§9 flags this as the load-bearing scope gate: Ch 42's
bundled amendment is structurally different from PR code
rollback paths, and the user's explicit approval of the
bundled amendment is the gate that prevents the awkward
rollback scenario.

## Section 9 — What Chapter 42 does not decide

Ch 42 renders PR 3, plans the 2-PR split, and bundles a
narrow amendment to Ch 41's PR 2a plan. It also defers a
handful of calls that surfaced during drafting but are out
of scope.

**The rematch writeup.** Ch 42 ships no writeup artifact.
The rematch's published writeup — the markdown document
that renders the bootstrap CI, the classification verdict,
the per-replicate table, the plots, and the prose discussion
— is a post-execution artifact authored after someone runs
`cargo test --release --ignored d2c_sr_rematch` on hardware
that can afford the compute. Ch 32 §7 explicitly deferred
the writeup's I/O format, plot shape, and BCa-vs-percentile
CI method to "a future writeup author." Ch 42 honors the
deferral: the PR 3 artifacts are the runnable rematch and
the chassis to run it, not a writeup for an unexecuted
experiment. A future commit (possibly in a new Part 5 of
this study book, or in a project-level memo outside the
study) will author the writeup once the rematch has been
run and its output is available.

**The bootstrap RNG implementation choice.** Ch 42 §6.6 picks
`StdRng` as the bootstrap resampling RNG implementation and
the literal value `0xB007_0057_00AA_0055` as its seed. Ch
32 §7 deferred this pick to Ch 42 with the note that "the
rematch writeup should name the bootstrap RNG seed for full
reproducibility." The pick is rendered in the fixture's
source code and is visible at `sim-opt/tests/d2c_sr_
rematch.rs`'s constant block. A future writeup author who
wants to cite the RNG pick in the published writeup reads
the fixture's constant directly. Ch 42 does not require the
writeup to mention the RNG; the requirement is that the
fixture makes the pick recoverable.

**SA hyperparameter tuning beyond the §4.5 defaults.** The
`SaHyperparams` defaults at §4.5 are calibrated to the D2c
SR reward scale and are defensible on the per-step reward
bound of `[-1, +1]` and the peak synchrony reference value
from the D2 SR findings memo. A reader who thinks the
defaults are wrong has three options: (a) change the
defaults via a future PR to sim-opt's `algorithm.rs`,
(b) override the defaults at the rematch fixture's call
site (the fixture constructs `SaHyperparams { ... }`
explicitly), or (c) run the rematch with the defaults, see
the result, and adjust in a follow-up experiment. Ch 42
does *not* run a hyperparameter sensitivity sweep for SA
before the rematch — that would violate Ch 30's scope
discipline (the rematch is the basic-SA case, and a
post-sensitivity-sweep rematch is a different experiment).
The defaults are a guess, defended by a calibration
argument, and will be tested by the rematch's actual run.

**SA's acceptance-rate collapse diagnostic.** Ch 23 §3.4's
edge-case discussion names the "representation too narrow"
failure mode for SA but does not specify a runtime gate
for acceptance-rate collapse — the scenario where SA's
Metropolis chain stops accepting proposals (because the
temperature has dropped below a threshold and all
downhill proposals are rejected). Ch 42's SA implementation
emits `extra["accepted"]` per epoch (0.0 or 1.0) for
post-hoc analysis, but does not gate on acceptance rate
at runtime. A reader who wants to check acceptance rate
reads the eprintln output or the serialized metrics
post-run. A future revision of sim-opt might add a
runtime gate (e.g., a warning if cumulative acceptance
< 10% after 20 epochs); Ch 42 declines to spec it.

**MLP or non-linear policy extensions for a future
rematch.** Ch 23 §3.1's matched-complexity anchor locks
`LinearPolicy(2, 1)` with `n_params = 3` for *this*
rematch, but a future experiment that wants to vary the
policy class (e.g., MLP(2, 8, 1) with `n_params = 33`)
would need its own matched-complexity argument and its
own anchor. Ch 42 does not plan such an extension. The
sim-opt crate's SA implementation is policy-agnostic at
the trait level (it takes `Box<dyn Policy>`), so an MLP-
based SA rematch would reuse SA unchanged; only the
rematch fixture and its policy construction would
change.

**The `Policy::clone_with_params` trait method.** §4.2
named the current policy cloning workaround (going through
`PolicyArtifact::bare` + `PolicyArtifact::to_policy`) as a
modest inefficiency. A cleaner shape would be a
`Policy::clone_with_params(&self, params: &[f64]) -> Box<dyn
Policy>` method on the `Policy` trait in ml-bridge. Adding
this method is a future ml-bridge PR, not PR 3a's scope.
PR 3a uses the workaround; a future refactor can adopt the
trait method.

**The `BestTracker::pub` visibility.** §4.1 noted that
`ml-bridge/src/best_tracker.rs`'s `BestTracker` struct is
`pub(crate)` and therefore unavailable to sim-opt, forcing
the `Sa` struct to duplicate the best-tracking logic inline.
A cleaner shape would be making `BestTracker` `pub` and
sharing it across ml-bridge and sim-opt. Ch 42 does not
propose this change because the duplication is small
(~15 lines across `Sa::new` and the train loop) and the
`BestTracker` API is not obviously the right shape for a
shared public surface — sim-opt's SA tracks best params
with scalar fitness, but a future multi-objective
optimization algorithm in sim-opt might track a Pareto
front, which `BestTracker` does not support. The
visibility question is a future call when a second
consumer surfaces.

**Ch 24 §1.5's shared rollout-aggregation helper.** Ch 24
§1.5 observed that REINFORCE, PPO, and (post-Ch-41-PR-2b)
CEM all have identical five-line `mean_reward`
computations at the "sum epoch_rewards, divide by `n_envs`"
level. Ch 42's SA adds a fourth copy of the same idiom
inside `evaluate_fitness`. The shared helper
(`EpisodicRollout::mean_reward_per_episode(n_envs)` or
similar) is a future-PR cleanup that Ch 24 §1.5 flagged
and Ch 41 §6 deferred. Ch 42 does not promote the helper
because the scope discipline is "ship SA, not refactor
ml-bridge." A future PR that extracts the helper would
update CEM, REINFORCE, PPO, and SA simultaneously; at that
point the three ml-bridge call sites and sim-opt's one
call site would all adopt the helper.

**`d2c_cem_training.rs` retirement.** The legacy D2c
training test file at `sim/L0/thermostat/tests/d2c_cem_
training.rs` is a single-seed training-run documentation
that the rematch fixture does *not* replace. After Ch 42's
PR 3b lands, both files coexist: `d2c_cem_training.rs`
documents the D2c experiment's per-algorithm single-seed
training runs, and `d2c_sr_rematch.rs` documents the
multi-seed statistical rematch. A future decision to
retire `d2c_cem_training.rs` entirely — on the grounds
that the rematch supersedes it — is possible but is
explicitly out of scope for PR 3. Ch 41 §6 already flagged
the retirement question as "a Ch 42 call"; Ch 42 defers it
further to "a post-rematch-execution call" on the grounds
that retiring the legacy file before the rematch has
actually run would be premature.

**Re-running the rematch under future chassis changes.**
The rematch is a one-time scientific question about the
post-Ch-40/41/42 chassis state. Ch 32 §7 explicitly deferred
the "should the rematch be re-run periodically" question,
and Ch 42 inherits the deferral. A future re-run would
re-apply Ch 32's protocol with possibly different seed
values (the pre-registered `MASTER = 20_260_412` protects
*this* rematch's reproducibility, not all future rematches').
Ch 42 does not plan re-run machinery.

**The warmup-overhead gate for pool extensions.** Ch 32
Decision 5 and Ch 31 §3.1 both name the 5× warmup-overhead
factor as forward-compatible with future pool extensions
that introduce algorithms with a warmup phase (e.g., TD3
or SAC re-added to the pool under a future richer-
expressiveness experiment). Ch 42 does not implement the
runtime gate because the current `{CEM, SA}` pool has no
warmup. A future pool extension PR would implement the
gate as part of its own scope.

**The `PerEnvStack` trait's public surface growth.** Ch 40
§3.3's `PerEnvStack` trait and `EnvBatch<S>` generic live
in `sim-core::batch` and are implemented by `PassiveStack`
in `sim-thermostat`. The trait is `pub` at the sim-core
level. A future stochastic component (e.g., Ornstein-
Uhlenbeck colored noise) would implement `PerEnvStack` for
its own stack type, but that implementation lives in the
future component's crate, not in sim-opt. Ch 42 does not
interact with the `PerEnvStack` trait beyond consuming
`PassiveStack`'s implementation via `VecEnv::builder` in
the rematch fixture.

**Data serialization for the rematch's output.** The
rematch fixture at §6.6 prints the classification outcome
via eprintln and the test passes on completion; it does
not serialize the replicate vectors, the bootstrap CI
bounds, the bimodality coefficients, or any other
intermediate state. A future writeup pipeline that wants
serialized data can either (a) read the test's eprintln
output and parse it, (b) extend the rematch fixture to
write a JSON file to `target/test-artifacts/`, or (c)
import the analysis module directly into a separate
binary that runs the rematch and serializes the output.
Ch 42 leaves all three options open and picks none.

**The rematch's effect size threshold for "convincingly
positive."** Ch 30's §"What the chapter does not decide"
explicitly deferred the "effect size we would call
'convincingly positive'" question to Ch 23 / Ch 24, and
Ch 32 Decision 1's bootstrap CI is the effect-size
classifier — if the CI lower bound exceeds zero, the
result is classified as positive. Ch 42 honors this chain:
the effect-size threshold is the CI lower bound, not a
separate scalar. A future commentary that frames a
specific positive result as "x% improvement over CEM"
is a writeup-author call and does not change the
classifier.

**Future Parts 5+ of the study book.** This chapter is the
last of Part 4. After Ch 42's commit, the study's remaining
work is the appendices (API inventory, test inventory) and
any post-rematch-execution writeup that a future Part 5
might host. Ch 42 does not plan the appendices or Part 5;
their scope is for future sessions.

**The `BOOTSTRAP_RNG_SEED` literal's specific value.**
§6.6's constant `0xB007_0057_00AA_0055` is a cute literal
whose hex representation spells a pattern. The exact
bytes are not load-bearing — any fixed `u64` would serve
the reproducibility role equally well. Ch 42 picks the
literal for pre-registration visibility (it is visible in
the fixture and easy to cross-reference) and for reader
amusement. A future writeup that finds the literal
distracting can substitute any other fixed `u64` in a
narrow post-Ch-42 patch with no downstream effect.

**Session 12 post-commit patch to §4.1.** During session
11's Appendix A factual pass a prose drift surfaced in
Ch 42 §4.1: line 1634 originally said "seven fields on
`Sa`" while the struct block at lines 1612–1631 enumerates
eight (`policy`, `hyperparams`, `current_params`,
`current_fitness`, `temperature`, `best_params`,
`best_fitness`, `best_epoch`). The appendix's inventory
row had the correct count. Session 11 deferred the
in-chapter fix; session 12 lands it as a one-word prose
patch (`seven` → `eight`) following the `b5cb3f6c` /
`3e1ec0ff` narrow-post-commit-patch pattern. No argument
in the chapter changes — the drift is a simple off-by-one
in the count, not a structural issue.

---

Chapter 42 plans PR 3 as the execution layer for the
rematch, creates the `sim-opt` crate, lands Simulated
Annealing, ships the bootstrap / bimodality / classify /
folded-pilot analysis machinery Ch 32 specified, and builds
the rematch test fixture that runs Ch 32's protocol
end-to-end. It bundles a narrow amendment to Ch 41 PR 2a's
plan extending `TaskConfig::build_fn` to thread a per-
replicate seed through to `build_vec_env`, closing the
engine gap Ch 42's recon surfaced. It does not run the
rematch, does not author the rematch's writeup, and does
not decide the scientific question — the rematch is the
test, and its outcome is the test's answer, which will be
produced when someone with appropriate hardware runs
`cargo test --release --ignored d2c_sr_rematch` against a
tree with PR 1a, PR 1b, PR 2a, PR 2b, PR 3a, and PR 3b all
merged.

Part 4's execution layer ends here.

