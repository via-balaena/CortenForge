# PR 2: Competition replicates and algorithm surface fixes

Part 4's second PR-plan chapter picks up where Ch 40 left off.
Ch 40 turned Ch 15's C-3 pick and the session-4 D1–D15 table
into PR 1 — the chassis-reproducibility execution plan. Ch 41
does the equivalent work for PR 2, the Competition replicates
API + algorithm surface fixes PR. Its inputs are three
committed chapters: Ch 23 (Competition API v2), which locked
the `run_replicates` return shape and the pool-membership call;
Ch 24 (Result semantics), which picked the per-replicate
reduction (Decision 1) and the across-replicate aggregation
surface (Decision 2); and Ch 31 §4.2, which demoted the CEM
`elite_mean_reward` labeling to a Ch 41 cleanup item. Ch 41's
job is to turn those picks into an executable plan — what the
file-level diff is, what the test surface becomes, how the PR
is split for review, what the landing sequence looks like
against PR 1, and what the rollback path is if something
breaks mid-land.

PR 2 is a different shape from PR 1 even though both are
Part 4 execution PRs. PR 1 was primarily a chassis rewire —
one new module (`prf.rs`), one constructor rewrite
(`LangevinThermostat::new`), one new per-env hosting path on
`BatchSim`, and a 56-call-site ripple across the thermostat
crate. PR 2 is a different animal. It has no new module, no
new constructor, and no cross-crate dependency ripples. What
it does have is an additive extension of an existing public
API (`Competition::run_replicates` alongside `Competition::run`
and a handful of new read helpers on `CompetitionResult`), a
targeted three-file surgical fix to the per-algorithm
`mean_reward` computation, and one small cosmetic cleanup
inside `cem.rs`. The diff is smaller than PR 1 by a wide
margin — on the order of a few hundred lines against PR 1's
two-thousand-plus-line touched surface — but the PR is no
less load-bearing for that: every post-PR-2 rematch result,
every post-PR-2 cross-algorithm comparison, every downstream
`SeedSummary` computation depends on the unit-uniformity that
PR 2 enforces at the algorithm surface.

Ch 41 plans a **2-PR split**, mirroring Ch 40's structure but
for different reasons. PR 2a ships the additive API surface
(the `run_replicates` entry point, the `replicate_index` field
on `RunResult`, and the new `replicate_best_rewards` /
`describe` / `SeedSummary` types for across-replicate
aggregation). PR 2b ships the semantic fix (Decision 1's
train-loop rewrites for CEM, TD3, SAC; the REINFORCE/PPO
zero-fallback cleanup; the CEM `elite_mean_reward` rename)
plus the narrow doc-only update to `d2c_cem_training.rs` that
surfaces during drafting. PR 2a is purely additive and
independently reviewable; PR 2b is the behavior change that
matters for every downstream unit assertion. §4 argues the
split against the single-PR alternative.

Ch 41's in-chapter sub-decisions are six in number, and as
with Ch 40 none of them re-litigate the Ch 23 / Ch 24 / Ch 31
picks. Section 2 covers four of them — the CEM `mean_reward`
rewrite form, the TD3/SAC pre-loop form and its invariant
guard, the REINFORCE/PPO zero-fallback cleanup shape, and the
CEM `elite_mean_reward` rename-or-doc call. Section 3 covers
the fifth one (the scope of `d2c_cem_training.rs` updates in
PR 2, which narrows substantially once a recon-surfaced
miscite in Ch 24 §2.2/§5 is corrected). Section 4 covers the
sixth one (the PR split itself). Section 4 also tables them
together so a reader can use it as a quick index into the
chapter. Section 5 draws the scope line around what Ch 41
does not decide, including the narrow post-commit patch to
Ch 24 §2.2 and §5 that is bundled into the Ch 41 commit
following the `b5cb3f6c` precedent.

## Section 1 — Scope and inheritance

### 1.1 What Ch 41 inherits from Ch 23

Ch 23 (`57bf1c25`) locked three calls that Ch 41 renders
without re-litigation.

**The replicates API shape.** Ch 23 picked
`Competition::run_replicates(tasks, builders, seeds: &[u64])`
as the general-case entry point, returning
`Result<CompetitionResult, EnvError>`. `run` becomes a one-line
wrapper — `self.run_replicates(tasks, builders, &[self.seed])`
— and returns the same type unchanged. The return shape is
flat `Vec<RunResult>` (same field name and type on
`CompetitionResult` as today) with each `RunResult` gaining a
new `replicate_index: usize` field. The length of
`result.runs` becomes `tasks.len() * builders.len() *
seeds.len()`, and the natural idiom for cross-replicate
aggregation is a filter over `result.runs.iter().filter(|r|
r.task_name == task && r.algorithm_name == algo)`. Execution
is sequential, same as the existing `run` — replicates run one
after the other, the existing task × builder inner loops stay
task-major-then-builder-major, and error handling preserves
the first-failure-aborts `Result<_, EnvError>` contract. Ch 23
§1.4 defended sequential-over-parallel on L0 (Bevy-free)
grounds: ml-bridge has no `rayon` dependency, parallelism
would be a net-new dependency for a feature whose users can
wrap calls externally at the callsite, and Ch 22's
compute-parity argument cares about honest env-step counts
rather than wall-clock.

**Ch 42 amendment (bundled into the Ch 42 commit per the
`843dc21c` precedent).** Ch 42's recon-to-leans surfaced
that the original Ch 41 rendering of `run_replicates` was
seed-blind at the `TaskConfig::build_vec_env` call site —
the seed threaded through to `algorithm.train` but not to
the `VecEnv` construction, meaning stochastic-physics tasks
(specifically the rematch's D2c SR task with its
`LangevinThermostat`) could not vary their physics noise
sequence per replicate. Ch 42 §2 walks four rendering
alternatives and picks extending `TaskConfig::build_fn`
from `Fn(usize) -> Result<VecEnv, EnvError>` to
`Fn(usize, u64) -> Result<VecEnv, EnvError>`. The
amendment adds a `u64` seed parameter to `build_fn` and
threads it through `run_replicates`'s inner body; §2.1
below renders the updated `build_vec_env` call and the
stock-task ripple. See Ch 42 §2 for the full argument and
the counterfactual walk.

Ch 23 §1.3 also recommended but did not specify a new helper
on `CompetitionResult`: `find_replicate(task, algorithm,
replicate_index) -> Option<&RunResult>` addresses the
ambiguity introduced by `find`'s current behavior ("which
replicate is 'the' result for `(task, algo)`?" under the new
multi-replicate return shape). The existing `find`,
`for_task`, and `for_algorithm` helpers at
`sim/L0/ml-bridge/src/competition.rs:91-110` (recon-reported)
continue to work: `find` returns the first replicate for the
pair (equivalent to the single-seed case), `for_task` and
`for_algorithm` return all replicates for the matching task
or algorithm across the full cross-product.

**PPO pool-membership.** Ch 23 §2 excluded PPO from the
rematch pool. The rematch pool collapses to `{CEM, SA}`. This
decision is inherited by Ch 41 as a fact about which test
gate Ch 42's PR 3 will eventually ship, not as a fact about
which algorithms PR 2 touches. **PR 2 touches all five
algorithms that ship today** (CEM, REINFORCE, PPO, TD3, SAC),
splitting the touched surface across two axes: Decision 1's
per-replicate reduction rewrite affects three algorithms (CEM
at `cem.rs:209`, TD3 at `td3.rs:487-491`, SAC at
`sac.rs:540-544`, per Ch 24 §3.2 — REINFORCE and PPO are
Decision 1 no-ops because their `mean_reward` computations
already match the picked standard); the Ch 24 §4.7 zero-fallback
cleanup affects the other two (REINFORCE at `reinforce.rs:213-225`
and PPO at `ppo.rs:313-325`). Every algorithm's train loop is
edited under at least one of the two axes, and PR 2 honors
the unit-uniformity guarantee across the full ml-bridge
surface rather than only the rematch pool. A reader who
assumes "PPO is out of the rematch pool" implies "PPO's train
loop is not touched by PR 2" is misreading the
pool-membership scope: the pool call is about what the D2c
rematch runs, and PR 2's train-loop edits are about what
every algorithm reports and how it handles degenerate epochs.
They are different questions and PR 2 answers the second
without depending on the first.

**Matched complexity anchor.** Ch 23 §3 picked
`LinearPolicy(2, 1)` with `n_params = 3` as the matched
complexity anchor for the rematch, enforceable as a one-line
equality assertion in the rematch test fixture. This is a
Ch 42 concern (the rematch test fixture lives in PR 3's
diff), and PR 2 does not need to assert it anywhere. Ch 41
mentions it only to confirm that PR 2's algorithm-surface
changes are agnostic to policy parameterization — the
`mean_reward` rewrites touch how the scalar is computed, not
what policy class the algorithm operates on.

### 1.2 What Ch 41 inherits from Ch 24 Decision 1

Ch 24 (`b9f85e19`) Decision 1 picked **"fix the algorithms"**
— standardize every `EpochMetrics::mean_reward` on "mean
per-episode total reward across `n_envs` trajectories", the
REINFORCE/PPO convention four of the five algorithms already
approximately use. Three train loops are touched; two are
no-ops. The concrete line-level changes Ch 24 §3.2 specified,
with the recon-reported file:line coordinates that PR 2 lands:

- **CEM.** At `cem.rs:209` (recon-reported), the reported
  `mean_reward` changes from the current
  `fitness.iter().map(|(_, f)| f).sum::<f64>() / n_envs as f64`
  — which divides by `n_envs` after summing the per-episode
  *length-normalized* per-step means computed at
  `cem.rs:182-183` (recon-reported) — to the REINFORCE-shape
  `total_reward / n_envs`, where `total_reward` is the sum of
  per-trajectory raw totals. The `fitness` computation at
  `cem.rs:177-189` (recon-reported), used for elite selection,
  **stays length-normalized**. Ch 24 §3.5 defended this
  internal split on scope grounds: Ch 24's scope is reporting
  comparability across algorithms, not CEM's internal
  selection pressure. Touching `fitness` would alter CEM's
  D2c-era search behavior in a way Ch 24 has no mandate to
  override, and the resulting two-reward-concept split inside
  CEM is named as a real cost paid for the scope discipline.
- **TD3.** At `td3.rs:487-491` (recon-reported), the
  denominator switches from `epoch_rewards.len()` to
  `n_envs`, and a small pre-loop before the computation
  pushes any remaining `env_episode_reward[i]` for incomplete
  envs into `epoch_rewards` so the partial-trajectory reward
  gets counted. Ch 24 §3.6 defended the
  contribute-the-partial choice: Decision 1 standardizes on
  REINFORCE's convention, REINFORCE includes partial
  trajectories at full weight in its numerator, so TD3 does
  the same.
- **SAC.** At `sac.rs:540-544` (recon-reported), the same
  edit as TD3. SAC and TD3's inline rollout structures are
  identical in the relevant section — `env_episode_reward`
  accumulation at `sac.rs:355-363` (recon-reported) and the
  denominator at `sac.rs:543` — and the fix is line-for-line
  the same shape.
- **REINFORCE.** No-op. The computation at
  `reinforce.rs:269-274` (recon-reported) already matches the
  picked standard: `total_reward / n_envs`.
- **PPO.** No-op. The computation at `ppo.rs:423-428`
  (recon-reported) is literally identical to REINFORCE's five
  arithmetic lines — same shape, same constant denominator,
  same partial-episode behavior.

Ch 24 §4.7 also flagged a contingency: the REINFORCE/PPO
zero-fallback at `reinforce.rs:213-225` and `ppo.rs:313-325`
(recon-reported) emits `EpochMetrics { mean_reward: 0.0, .. }`
when `n_samples == 0`. This is "no samples collected" not
"reward is actually zero", but a max-based `best_reward()`
would pick up the `0.0` as a real best. Ch 24 §4.7's Decision 2
silent-filter-out of `None` replicates is contingent on Ch 41
fixing this so zero-epoch runs map to `None` from
`RunResult::best_reward()`, not to a false zero. §2.4 picks
the fix shape.

`RunResult::best_reward()` at `competition.rs:45-50`
(recon-reported) stays untouched. Ch 24 §4.1 was explicit:
"Decision 1 answered the first half of that hook: yes,
`mean_reward` is the problem, and the fix lives inside the
train loops, not inside `RunResult::best_reward()`." Once the
per-algorithm `mean_reward` values are in a single unit, the
existing max-over-`metrics[].mean_reward` helper is
semantically fine.

There is one subtlety worth naming at the inheritance stage
because it interacts with PR 2's diff shape. `Competition::run`
today does not compute its provenance's `best_reward` /
`best_epoch` via `RunResult::best_reward()`. It has its own
inline scan at `competition.rs:363-379` (recon-reported), with
a "strict `>`" comment at `:362` documenting the tie-breaking
convention (ties keep the earlier epoch, matching
`BestTracker` §3.3). The scan uses `f64::NEG_INFINITY` as the
accumulator seed and returns `None` when `metrics.is_empty()`
or when the best reward is non-finite. This inline block is
separate from the `RunResult::best_reward()` helper and
happens to produce the same result in the common case (once
Decision 1 lands) but not identical code. PR 2's `run` →
`run_replicates` rewire has to preserve this inline scan
inside the shared internal loop body, not fold it away into
`RunResult::best_reward()`. §2.1 renders the detail; it is
flagged here so a reader who assumes "PR 2 replaces the scan
with a helper call" can correct the assumption up front.

### 1.3 What Ch 41 inherits from Ch 24 Decision 2

Ch 24 Decision 2 picked **Option M** (medium): the raw
`replicate_best_rewards(task, algo) -> Vec<f64>` primitive plus
a minimal `describe(task, algo) -> Option<SeedSummary>`
convenience struct. `SeedSummary` carries three fields — `n`,
`mean`, `std_dev` — and nothing else. Sample standard deviation
(Bessel's correction, `n-1` denominator), not population std,
on convention-matching-with-RL-literature grounds. Silent
filter-out of `None` replicates at the primitive; callers that
want to detect missing replicates compare
`replicate_best_rewards(task, algo).len()` to their seed count
at the call site. `n == 1` returns `std_dev = 0.0` with a doc
note that sample std is undefined for a single observation;
callers that need to guard on `n >= 2` check `.n` directly.

The concrete Ch 24 §4.3 API sketch PR 2 implements:

```rust
impl CompetitionResult {
    pub fn replicate_best_rewards(&self, task: &str, algo: &str) -> Vec<f64>;
    pub fn describe(&self, task: &str, algo: &str) -> Option<SeedSummary>;
}

pub struct SeedSummary {
    pub n: usize,
    pub mean: f64,
    pub std_dev: f64,
}

impl SeedSummary {
    pub fn from_rewards(rewards: &[f64]) -> Option<SeedSummary>;
}
```

Three public methods on `CompetitionResult`, one new struct,
one constructor. §2.1 describes the file placement and
visibility questions in more detail; the shape is fixed by
Ch 24 and Ch 41 is not reopening it.

One contingency from Ch 24 §4.7 ships with Decision 2: the
`Vec<f64>` (rather than `Vec<Option<f64>>`) silent-filter-out
is contingent on PR 2 also fixing the REINFORCE/PPO
zero-fallback that Ch 24 §3.7 flagged. If PR 2 shipped
Decision 2 without the zero-fallback cleanup, the primitive
would silently pass emitted-zeros-from-the-short-circuit
through as real data points *and* silently filter out true
zero-epoch runs that returned `None`, which Ch 24 §4.7 named
as "the worst of both worlds." §2.4 ships the cleanup as part
of the same PR to close the contingency.

### 1.4 What Ch 41 inherits from Ch 31 §4.2

Ch 31 (`6df97c8a`) §4.2 is the Part 3 closer's bucket-4 entry
for "algorithm-surface metric fix (apples-to-oranges shape)."
The section points at Ch 41 as the PR that lands the metric
fix — everything in §1.2 and §1.3 above falls under this
pointer — and carries one additional item demoted during
Ch 31's recon pass: the CEM `extra.elite_mean_reward`
diagnostics key at `cem.rs:219` (recon-reported) is in
per-step units, while Ch 24 Decision 1's `mean_reward` becomes
per-episode-total. Ch 31 §4.2 recon-verified that
`elite_mean_reward` has no rematch reporting consumers — a
grep across the codebase for the string returns exactly four
hits, all inside `cem.rs`: the write site at `cem.rs:191`, the
`extra.insert` at `cem.rs:219`, and two unit tests at
`cem.rs:319` and `cem.rs:354` that assert presence and
finiteness of the key (all recon-reported). The unit mismatch
is therefore a CEM-internal cleanup item that Ch 41 owns as a
sub-decision, not a rematch failure mode the writeup has to
guard against.

Ch 24 §5 framed the same item as a Ch 41 call: "Whether CEM's
`extra.elite_mean_reward` diagnostics key needs renaming or a
doc update. With Decision 1 in effect, CEM's `mean_reward` is
in per-episode-total units and the `elite_mean_reward` extras
key is in per-step units — the naming collision is mild but
real, and a reader of the `EpochMetrics.extra` map will
reasonably ask what unit the elite key is in. The execution
PR for Decision 1 should at least update the doc comment
explaining both keys side by side; whether to rename the key
outright is a Chapter 41 cleanup call." §2.3 picks — the
short answer is rename with a `_per_step` suffix.

### 1.5 Genre note: Ch 41 renders more than it argues

Ch 40's genre note applies to Ch 41 without modification.
Ch 41 is the second PR-plan chapter in Part 4 and inherits
the rendering-over-arguing voice Ch 40 established. A reader
expecting Ch 14/15/23/24/32-style counterfactual walks
through design alternatives will see less of that and more
of the other shape: diff descriptions, call-site tallies, PR
description drafts, landing-order justifications, rollback
walkthroughs. The in-chapter sub-decisions — the calls Ch 23,
Ch 24, and Ch 31 §4.2 did *not* lock and Ch 41 has to make —
get the full counterfactual treatment (§2.2 through §3 for
the sub-decisions, §4 for the PR split), but the rest of the
chapter is rendering. A reader who wants to understand why
Decision 1 is "fix the algorithms" rather than "parallel
field" should read Ch 24 §3.3; a reader who wants to know
what the diff looks like at `cem.rs:209` should read this
chapter.

### 1.6 The relationship to PR 1

PR 2 is decoupled from PR 1 at the code level. Ch 40's diff
touches `sim/L0/thermostat/src/{prf.rs, langevin.rs,
stack.rs, component.rs}` plus a 56-call-site ripple across 18
files in the thermostat and integration-test trees. Ch 41's
diff touches `sim/L0/ml-bridge/src/{competition.rs, cem.rs,
reinforce.rs, ppo.rs, td3.rs, sac.rs}` and possibly one new
file for `SeedSummary`'s placement (see §2.1). The two diffs
share no files and no call-site ripples. PR 1 can land before,
after, or concurrently with PR 2 without any merge-order
dependency surfacing, and neither PR can rollback the other.

This decoupling is worth naming because it affects the
rematch's readiness gate. Ch 31 §4.1 names PR 1 (Ch 40) as a
bucket-4 prerequisite for the rematch under a latent-flakiness
shape; §4.2 names PR 2 (Ch 41) as a bucket-4 prerequisite
under an apples-to-oranges shape. Both have to land before the
rematch can run, but they are independent bucket-4 items — PR
1 does not unblock PR 2 or vice versa — and Ch 42's PR 3
consumes both. The merge-order options for PR 1 and PR 2 are
therefore free, and the rematch readiness gate is a
conjunction: both have to be merged before PR 3 can be
written.

One minor touchpoint: Ch 40 §2.4 committed to shipping
`splitmix64` in `prf.rs` so Ch 42's rematch can call it for
seed derivation. PR 2 does not use `splitmix64` — Ch 41's
call-site to `run_replicates` takes `&[u64]` directly, and
Ch 23 §1.2's recommended idiom is documented in prose rather
than enforced in the signature. The rematch test fixture
(Ch 42) is where the `splitmix64`-derived seed slice gets
built and passed into `run_replicates`. PR 2's `run_replicates`
is agnostic to how the caller produced the slice; the helper
from Ch 40's PR 1a is a convenience that Ch 42's PR 3 picks
up independently.

## Section 2 — PR 2a and the semantic surface fixes

PR 2 is split into two PRs. PR 2a is the additive half — the
new `run_replicates` entry point, the new `replicate_index`
field on `RunResult`, and the new `replicate_best_rewards` /
`describe` / `SeedSummary` surface. PR 2a has zero semantic
effect on existing callers: every current `comp.run(...)`
callsite continues to compile and produce identical results,
every existing test at `competition.rs:520-875` and
`ml-bridge/tests/competition.rs` passes unchanged, and the
replicate-index value on results produced by `run` is always
zero (the single-replicate case). PR 2b is the semantic half
— the train-loop rewrites for CEM, TD3, and SAC, the
REINFORCE/PPO zero-fallback cleanup, and the CEM
`elite_mean_reward` rename. PR 2b changes the numeric values
every existing test would observe if it asserted on absolute
reward magnitudes, which is the load-bearing reason for the
split. §4 argues the 2-PR shape against the single-PR
alternative.

Section 2 describes the diff for both PRs together, with the
PR 2a / PR 2b split marked inline per subsection. The sections
2.1–2.3 are mostly PR 2a; sections 2.2–2.5 are mostly PR 2b.

### 2.1 The additive API surface (PR 2a)

PR 2a's diff is contained within the ml-bridge crate and
touches primarily one file, `competition.rs`, with one
supporting file placement choice for `SeedSummary`. The file
placement is one of the sub-decisions (§2.1's end names it
and picks).

**`competition.rs` changes.**

The `RunResult` struct at `competition.rs:23-34`
(recon-reported) gains one new field:

```rust
pub struct RunResult {
    pub task_name: String,
    pub algorithm_name: String,
    pub replicate_index: usize,  // NEW: 0-based index within the replicate set
    pub metrics: Vec<EpochMetrics>,
    pub artifact: PolicyArtifact,
    pub best_artifact: PolicyArtifact,
}
```

The `Competition` struct at `competition.rs:276-281`
(recon-reported) is unchanged. The existing constructors
`Competition::new` at `competition.rs:290` and
`Competition::new_verbose` at `competition.rs:304` stay with
their single-seed `seed: u64` parameter. The stored `seed`
field continues to be the default for the single-seed `run`
path.

The `run` method at `competition.rs:321-411` (recon-reported)
becomes a one-line wrapper:

```rust
pub fn run(
    &self,
    tasks: &[TaskConfig],
    builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
) -> Result<CompetitionResult, EnvError> {
    self.run_replicates(tasks, builders, &[self.seed])
}
```

The body of the current `run` — including the task × builder
nested loops, the per-pair fresh `VecEnv` construction at
`competition.rs:330` (recon-reported), the `algorithm.train`
call at `competition.rs:341`, the inline best-epoch scan at
`competition.rs:363-379` (recon-reported) with its strict-`>`
tie-breaking convention, the provenance construction at
`competition.rs:381-395`, and the `RunResult` push at
`competition.rs:400-406` — moves into a new method
`run_replicates` wrapped in an outer loop over seeds:

```rust
pub fn run_replicates(
    &self,
    tasks: &[TaskConfig],
    builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
    seeds: &[u64],
) -> Result<CompetitionResult, EnvError> {
    let mut runs = Vec::with_capacity(
        tasks.len() * builders.len() * seeds.len()
    );

    for (replicate_index, &seed) in seeds.iter().enumerate() {
        for task in tasks {
            for builder in builders {
                // Ch 42 amendment: build_vec_env takes the
                // per-replicate seed so stochastic-physics
                // tasks can vary their LangevinThermostat
                // master_seed per replicate.
                let mut env = task.build_vec_env(self.n_envs, seed)?;

                // ... existing per-pair body, with `seed` replacing `self.seed`
                // ... and `replicate_index` recorded on the RunResult

                runs.push(RunResult {
                    task_name: task.name().to_string(),
                    algorithm_name: name.to_string(),
                    replicate_index,
                    metrics,
                    artifact,
                    best_artifact,
                });
            }
        }
    }

    Ok(CompetitionResult { runs })
}
```

**Ch 42 amendment: `TaskConfig::build_fn` signature
extension.** The `build_vec_env(self.n_envs, seed)` call in
the skeleton above is the amended form — under Ch 42 §2's
bundled amendment, `TaskConfig::build_fn` takes a `u64`
seed as its second parameter, and `run_replicates` threads
the per-replicate seed through at the `build_vec_env` call
site. The ripple across `task.rs` touches seven sites, all
recon-reported against the current source tree:

- `task.rs:43` — `build_fn: Arc<dyn Fn(usize) -> Result<
  VecEnv, EnvError> + Send + Sync>` becomes `build_fn:
  Arc<dyn Fn(usize, u64) -> Result<VecEnv, EnvError> + Send
  + Sync>`.
- `task.rs:108` — `pub fn build_vec_env(&self, n_envs:
  usize)` becomes `pub fn build_vec_env(&self, n_envs:
  usize, seed: u64)`, with its body `(self.build_fn)(n_envs)`
  at `:109` becoming `(self.build_fn)(n_envs, seed)`.
- `task.rs:241` — the `TaskConfigBuilder`'s internal
  `let build_fn = Arc::new(move |n_envs: usize| ...)`
  declaration updates to the two-argument closure form.
- `task.rs:362, :396` — `pub fn reaching_2dof` and its
  internal `build_fn` closure: closure gains an ignored
  `_seed: u64` parameter.
- `task.rs:445, :500` — `pub fn reaching_6dof` and its
  closure: same ignored-parameter addition.
- `task.rs:619, :687` — `pub fn obstacle_reaching_6dof` and
  its closure: same.

The `_seed: u64` ignored parameter on each stock task
reflects that deterministic-physics tasks have no reason
to consume a per-replicate seed — only stochastic-physics
tasks (the rematch's D2c SR task at Ch 42 §6.5) use the
parameter. The convention matches the existing Rust idiom
of underscore-prefix for unused parameters.

`sim/L0/ml-bridge/src/vec_env.rs:391` (the `BatchSim::new`
call inside `VecEnvBuilder::build()`) is unchanged because
the seed does not flow through `VecEnv::builder` at all —
the `TaskConfig` closure captures the seed (or receives it
as a parameter and passes it down into the
`LangevinThermostat` construction) *before* calling
`VecEnv::builder.build()`. The rematch fixture at Ch 42 §6
is where the seed consumption happens.

See Ch 42 §2.5 for the full amendment scope and Ch 42 §2.4
for the pick-defense against R1 (sim-opt owns the outer
loop), R3 (bypass `run_replicates`), and R4 (`AtomicU64`
in the closure).

The ordering is `seeds → tasks → builders`, outermost to
innermost, matching Ch 23 §1.4's pick: "seeds-outermost
treats the existing `run` body as a unit and repeats it N
times." The `VecEnv` stays fresh per `(task, builder)` pair
inside the innermost loop, which preserves the existing
no-cross-contamination discipline at
`competition.rs:330` (recon-reported) and keeps replicate
contamination risk at zero by construction.

The provenance construction at `competition.rs:381-395`
(recon-reported) takes `self.seed` as the recorded seed
today; under `run_replicates`, it takes the current loop's
`seed` instead. The `seed` field on `TrainingProvenance` at
`competition.rs:384` (recon-reported) therefore reflects the
actual per-replicate seed, not the `Competition`-level default.
This is a semantic change: a reader of a `run`-produced
artifact file still sees `seed == comp.seed` (because `run`
calls `run_replicates` with the default), but a reader of a
`run_replicates`-produced artifact file sees the
per-replicate seed, which is the correct value to record for
reproducibility purposes.

The inline best-epoch scan at `competition.rs:363-379` stays
inside the per-pair body, unchanged in shape. Ch 41 explicitly
does **not** rewrite this scan to call
`RunResult::best_reward()` instead — the two do equivalent
work in the common case (post-Decision-1), but the inline
scan has a strict-`>` tie-breaking convention documented at
`competition.rs:362` and matching `BestTracker` §3.3, while
`RunResult::best_reward()` uses `max_by` with
`partial_cmp`-`Ordering::Equal` fallback at `competition.rs:49`
(recon-reported). The two produce the same scalar but the
inline scan carries the "ties keep the earlier epoch"
invariant that matters for `best_epoch` indexing, which
`RunResult::best_reward()` does not. PR 2 preserves the
inline scan.

The three helper methods on `CompetitionResult` at
`competition.rs:91-110` (recon-reported) — `find`, `for_task`,
`for_algorithm` — keep working under the flat shape. `find`
now returns the first matching replicate (which in the
single-seed case is still "the" result and matches current
behavior exactly); `for_task` and `for_algorithm` return all
replicates for the matching key across the full cross-product.
A new helper `find_replicate` is added alongside:

```rust
impl CompetitionResult {
    pub fn find_replicate(
        &self,
        task: &str,
        algorithm: &str,
        replicate_index: usize,
    ) -> Option<&RunResult> {
        self.runs.iter().find(|r| {
            r.task_name == task
                && r.algorithm_name == algorithm
                && r.replicate_index == replicate_index
        })
    }
}
```

Three lines. Returns the specific replicate for the
`(task, algo, idx)` triple, `None` if none. Ch 23 §1.3
recommended this helper in prose; PR 2a lands it.

`CompetitionResult::best_for_task` at `competition.rs:137-146`
(recon-reported), `save_artifacts` at
`competition.rs:120-133`, `print_ranked` at
`competition.rs:153-192`, and `print_summary` at
`competition.rs:198-219` all continue to work unchanged. Under
replicates, `best_for_task` returns the artifact with the
highest `best_reward()` across all `(algorithm, replicate)`
pairs for the task — still a sensible answer — and
`save_artifacts` writes every per-replicate artifact to disk
with the existing `{task}_{algorithm}.artifact.json` naming
convention at `competition.rs:124`. The naming is not
replicate-aware, which means running a `run_replicates` with
multiple replicates and calling `save_artifacts` would
overwrite files within a task × algorithm pair. This is a
latent collision that PR 2a flags but does not fix — the
existing callers either use single-replicate runs (where the
collision does not manifest) or do not call `save_artifacts`
at all — and Ch 42 owns the fix if the rematch wants to
persist per-replicate artifacts to disk.

**New `SeedSummary` module.**

The new `SeedSummary` struct and its `from_rewards`
constructor need a home. Two options:

- **Option (i): inline in `competition.rs`.** Drop the struct
  definition and its `impl` block at the bottom of
  `competition.rs`, below `CompetitionResult`'s helper methods.
  The file grows by ~40 lines.
- **Option (ii): new file `seed_summary.rs`.** Ship
  `SeedSummary` in its own module, with `pub mod
  seed_summary;` in `lib.rs` and a re-export via `pub use
  seed_summary::SeedSummary;`. The file is small (~40 lines
  including docs and inline unit tests) but self-contained.

The lean is **Option (i) — inline in `competition.rs`**. The
struct's only purpose is to summarize `CompetitionResult`'s
filtered slice; it has no reuse outside the competition
surface, and its tests belong in the same file as the
`replicate_best_rewards` and `describe` tests that exercise
it. Option (ii) would be justified if `SeedSummary` had
callers outside `competition.rs` or if its implementation
grew to a size where file length was a readability concern,
but neither holds: the implementation is ~40 lines total and
`competition.rs` is already ~900 lines — adding 40 keeps it
under 1000, well below the "needs splitting" threshold.
Option (i) also keeps the test surface for Decision 2 in one
file, which makes the PR 2a review surface contiguous.

This is not one of the six in-chapter sub-decisions; it is a
rendering detail that follows from Ch 24 Decision 2's locked
shape and the ml-bridge crate's existing module layout. §4's
sub-decision table does not include it.

**Visibility.**

All three new methods on `CompetitionResult`
(`replicate_best_rewards`, `describe`, `find_replicate`) are
`pub`. `SeedSummary` is `pub` with `pub` fields (`n`, `mean`,
`std_dev`) and a `pub` `from_rewards` constructor. `run_replicates`
is `pub`. The `replicate_index` field on `RunResult` is `pub`.
There is no reason to narrow visibility below `pub` on any of
these — they are the documented callable surface the rematch
test fixture in Ch 42 will use, and narrowing visibility
would force every caller to go through an abstraction
boundary that has no abstraction value.

### 2.2 The per-algorithm train-loop fixes (PR 2b)

PR 2b's Decision 1 diff touches three files — `cem.rs`,
`td3.rs`, and `sac.rs` — with REINFORCE and PPO getting no
semantic change (the zero-fallback cleanup in §2.4 is a
separate concern that touches `reinforce.rs` and `ppo.rs` but
does not modify their `mean_reward` computation). The three
targeted fixes are surgical — roughly ten line-level changes
across the three files, plus a three-line pre-loop in each of
TD3 and SAC, plus the CEM `elite_mean_reward` rename per §2.3.
Fewer than twenty lines of ml-bridge source change total.

**CEM rewrite at `cem.rs:209`.**

The current line at `cem.rs:209` (recon-reported) is:

```rust
let mean_reward: f64 = fitness.iter().map(|(_, f)| f).sum::<f64>() / n_envs as f64;
```

`fitness` at `cem.rs:177-185` (recon-reported) is a
`Vec<(usize, f64)>` where each `f64` is
`total / traj.len().max(1)` — the per-step mean for that
trajectory. Ch 24 §3.2's replacement is `total_reward /
n_envs`, matching REINFORCE's shape at `reinforce.rs:274`
(recon-reported). The question is how to compute
`total_reward` given that CEM's fitness loop already traverses
`rollout.trajectories` once. Two forms are plausible:

- **Form (i): a second traversal.** Add a one-line
  computation immediately before `cem.rs:209` that traverses
  `rollout.trajectories` independently and sums the raw totals:
  `let total_reward: f64 = rollout.trajectories.iter().map(|t|
  t.rewards.iter().sum::<f64>()).sum();`. The form is
  line-for-line identical to REINFORCE at
  `reinforce.rs:269-273` (recon-reported) and PPO at
  `ppo.rs:423-427` (recon-reported).
- **Form (ii): extend the fitness loop.** Modify the fitness
  computation at `cem.rs:177-185` to also accumulate
  `total_reward` inside the same `map` chain, emitting a
  tuple of `(fitness_length_normalized, raw_total)` and
  summing the raw totals afterward. This saves one `Vec`
  traversal but couples the elite-selection and reporting
  concerns in the same loop.

The lean is **Form (i)**. The argument is that Ch 24 §1.5
named the REINFORCE/PPO five-line duplication as "evidence
that the chassis has a shared-rollout-aggregation gap" — an
observation the chapter explicitly flagged as a hint the
chassis could grow a shared aggregation helper in a future
iteration. Form (i) makes CEM a third-copy of the same
five-line idiom, which strengthens Ch 24 §1.5's observation
by making the duplication visible at three sites instead of
two. If a future PR introduces a shared `mean_episode_reward`
helper on `EpisodicRollout`, it replaces three identical
expressions with three identical calls — Form (i) keeps the
code at the "three visible copies" state that makes the
refactor obvious when it comes. Form (ii) couples concerns
that should stay separate (CEM's elite-selection fitness is a
per-step metric; CEM's reporting `mean_reward` is a
per-episode-total metric under Decision 1; merging them into
one loop obscures the split that Ch 24 §3.5 deliberately
documented as "a real cost" of scope discipline). The
one-loop save is microscopic compared to the clarity loss.

**Form (i) is the pick.** Concretely, the CEM patch drops the
existing line at `cem.rs:209` and replaces it with:

```rust
let total_reward: f64 = rollout
    .trajectories
    .iter()
    .map(|t| t.rewards.iter().sum::<f64>())
    .sum();
let mean_reward = total_reward / n_envs as f64;
```

Five lines (same as REINFORCE and PPO). The `fitness`
computation at `cem.rs:177-189` stays completely unchanged;
its `elite_mean_reward` downstream at `cem.rs:191` and its
extras insert at `cem.rs:219` stay unchanged in value (per
§2.3, only the extras *key name* changes). The `best`
tracking call at `cem.rs:212-213` now gets the per-episode
total as its `mean_reward` input, which changes the numeric
value `BestTracker` sees but does not change the strict-`>`
ordering it uses for epoch selection — the epoch that would
have been picked under the per-step metric is still picked
under the per-episode metric, assuming approximately constant
episode length across epochs.

**TD3 fix at `td3.rs:487-491`.**

The current block at `td3.rs:487-491` (recon-reported) is:

```rust
let mean_reward = if epoch_rewards.is_empty() {
    0.0
} else {
    epoch_rewards.iter().sum::<f64>() / epoch_rewards.len() as f64
};
```

Ch 24 §3.6's replacement sketches a pre-loop that pushes
incomplete envs' `env_episode_reward[i]` into `epoch_rewards`
before the computation, then divides by `n_envs` unconditionally.
The concrete patch:

```rust
// Include any envs that did not complete an episode within
// max_episode_steps: their partial episode reward counts at
// full weight, matching REINFORCE's treatment of truncated
// trajectories.
for i in 0..n_envs {
    if !env_complete[i] {
        epoch_rewards.push(env_episode_reward[i]);
    }
}
debug_assert_eq!(
    epoch_rewards.len(),
    n_envs,
    "epoch_rewards / n_envs invariant violated"
);
let mean_reward = epoch_rewards.iter().sum::<f64>() / n_envs as f64;
```

Three lines of pre-loop (matching Ch 24 §3.6's sketch), one
line of `debug_assert_eq!`, and one line for the `mean_reward`
computation — total five lines replacing the five lines of
the current `if/else`. The `debug_assert_eq!` is a
sub-decision (see §2.2's end).

The `epoch_rewards` `Vec` is still used downstream for the
`EpochMetrics.total_steps` bookkeeping at `td3.rs:504-510`
(recon-reported) — actually no, `total_steps` is
`epoch_steps`, not `epoch_rewards.len()`, so `epoch_rewards`
has no other consumer after the fix. Ch 41 could drop the Vec
entirely and compute the sum in a single pass, but the
minimum-diff pick keeps it as the explicit intermediate
because the `debug_assert_eq!` invariant is expressible
against it in a way it is not expressible against a
direct-sum form. The debug-assert catches invariant drift if
a future edit to the inner loop changes the relationship
between `env_complete` and the `epoch_rewards.push` call.

**SAC fix at `sac.rs:540-544`.**

Same patch shape as TD3. The current block at
`sac.rs:540-544` (recon-reported) is identical to TD3's block
at `td3.rs:487-491` line-for-line. The pre-loop and the
`debug_assert_eq!` and the denominator swap all apply
identically. The `env_complete` state at `sac.rs:355-363`
(recon-reported) has the same structure as TD3's at
`td3.rs:335-351` (recon-reported), so the semantic guarantee
is the same: every env contributes exactly one value to
`epoch_rewards` after the pre-loop, the value is either the
first-completed episode's reward (if the env completed) or
the partial episode's accumulated reward at cutoff (if it did
not).

**Sub-decision: the `debug_assert_eq!` invariant guard.**

Adding the debug-assert is a minor but genuine call. Two
shapes are worth naming:

- **Shape (a): add `debug_assert_eq!(epoch_rewards.len(),
  n_envs)` immediately after the pre-loop.** Cheap in the
  debug build, zero-cost in release, catches the invariant
  drift case where a future edit breaks the relationship
  between `env_complete` and the push-to-`epoch_rewards` call.
- **Shape (b): omit the assert.** The invariant is "obvious"
  from reading the pre-loop alongside the inner-loop push-site
  at `td3.rs:337-343` / `sac.rs:357-363`, and adding an
  assert for every structurally guaranteed invariant would
  flood the code with noise.

The lean is **shape (a)**. The invariant is *currently*
obvious, but it is obvious specifically because the
inner-loop push and the pre-loop are both treating
`env_complete` as the source of truth for "did this env
contribute yet?" If a future edit adds a third site that
pushes to `epoch_rewards` without checking `env_complete` —
which would be a small, local-looking edit inside the
existing inner loop — the debug-assert catches the drift at
the next test run, before it propagates into a silently
wrong `mean_reward` calculation. The cost of shape (a) is one
line per file (TD3 and SAC), the benefit is a regression
guard on exactly the invariant Ch 24 Decision 1 depends on.
Shape (b)'s "obvious invariant" framing is exactly the kind
of local reasoning that decays as the surrounding code
evolves. One debug-assert is cheap insurance.

A narrower alternative — shape (a') — would be to assert the
stronger invariant `epoch_rewards` equals the set of
per-env accumulated rewards, which is a `Vec`-level equality
check. This is more rigorous and also more expensive (it
requires a reference vector to compare against) and does not
add useful coverage over shape (a): if `epoch_rewards.len() ==
n_envs` holds, the only way the per-env values could drift is
if a push went to the wrong index or a value was corrupted
mid-loop — scenarios that are much less likely than the
length drift shape (a) catches. Shape (a) is the right cost
tier.

The `debug_assert_eq!` is shape (a) in both TD3 and SAC.

### 2.3 The CEM `elite_mean_reward` rename (PR 2b)

Ch 31 §4.2's demoted cleanup item is the naming collision
between CEM's `extra.elite_mean_reward` (per-step units,
unchanged by Decision 1) and CEM's `mean_reward`
(per-episode-total units, post-Decision-1). Ch 24 §5 framed
this as a Ch 41 choice between rename and doc-update.

**Option (a): rename the key to `elite_mean_reward_per_step`.**
Three source edits:

- `cem.rs:219` — change the `.insert("elite_mean_reward"...)`
  call to `.insert("elite_mean_reward_per_step"...)`.
- `cem.rs:319` — update the `m.extra.contains_key(...)`
  assertion to the new key name.
- `cem.rs:354` — update the `m.extra[...]` index to the new
  key name.

No other files reference the key — the grep across the
workspace for `elite_mean_reward` returns exactly those four
hits, all in `cem.rs` (the write at `:191`, the insert at
`:219`, and the two unit tests at `:319` and `:354`, all
recon-reported). The rename is mechanical and contained.

**Option (b): leave the key name, add a doc comment.** One
source edit: a three-line doc comment above
`cem.rs:217-219` (the extras-insert block) explaining that
`elite_mean_reward` is in per-step units while
`mean_reward` (the EpochMetrics field) is in per-episode-total
units post-Decision-1. The two unit tests stay unchanged.

**The lean is Option (a).** The argument is that the unit
belongs in the key name, not in a doc comment. A reader
inspecting `EpochMetrics.extra` in a debugger, a log-parser
reading JSON dumps of the provenance metadata, or a test
author writing a new assertion on the extras map does not
see the doc comment — they see the key name. If the key name
is `elite_mean_reward`, a reader reasonably assumes it is in
the same units as `mean_reward`, because the naming convention
across the rest of the ml-bridge surface is unit-consistent.
The `_per_step` suffix makes the unit visible at every
reading site, and the cost is two test line updates inside
`cem.rs`. The cost is small, the benefit is persistent.

Option (b) is defensible — the unit mismatch is mild (the two
keys are in the same metrics struct but the doc would
disambiguate) and the rename touches a user-visible key that
some downstream tool might be consuming. But the
"user-visible key consumption" concern dissolves on the
workspace grep: there is no such downstream tool. The only
callers of the key are CEM's own unit tests, which change
in lockstep with the rename. A future downstream consumer
(a custom analysis script, a Ch 42 rematch metric) would
write against whichever name ships. Option (a) ships.

### 2.4 The REINFORCE/PPO zero-fallback cleanup (PR 2b)

Ch 24 §4.7 flagged the zero-fallback at
`reinforce.rs:213-225` and `ppo.rs:313-325` (recon-reported)
as a Ch 41 concern. The current behavior: when `n_samples ==
0` in REINFORCE's training loop, the algorithm emits an
`EpochMetrics { mean_reward: 0.0, .. }` via
`metrics.push(em)` and `continue`s to the next epoch. PPO has
identical structure at the equivalent line range. The "no
samples collected" case is meant to guard against degenerate
rollout outcomes (every trajectory has zero steps, which is
structurally hard to produce but the guard is defensive), but
the emitted `0.0` gets picked up by `RunResult::best_reward()`
at `competition.rs:45-50` as a real reward value. A run where
every epoch hit this short-circuit would return `Some(0.0)`
from `best_reward()`, which is then treated as a real number
by Decision 2's `replicate_best_rewards` helper.

Ch 24 §4.7's silent-filter-out of `None` replicates is
contingent on PR 2 making the fallback emit something
`best_reward()` can recognize as "no valid data" — either
`None` from a missing epoch or an explicit non-value that
`best_reward()` filters out. Four shapes are plausible:

- **Shape (i): skip the `metrics.push` entirely.** On the
  `n_samples == 0` branch, replace the current
  `metrics.push(em); continue;` with just `continue;`. The
  short-circuit epoch never appears in `metrics`, which
  means `metrics.len()` reflects the count of non-degenerate
  epochs rather than the nominal `n_epochs`. `best_reward()`
  returns `None` if every epoch short-circuits; otherwise it
  returns the max over the non-degenerate epochs. The
  per-replicate raw primitive in Decision 2 maps the
  `None` case to a filtered-out entry (the silent-filter-out
  path Ch 24 §4.7 committed to).
- **Shape (ii): emit `EpochMetrics { mean_reward:
  f64::NAN, .. }`.** Keep the `metrics.push` but emit NaN
  instead of zero. `best_reward()`'s `partial_cmp` fallback at
  `competition.rs:49` (recon-reported) treats NaN as `Equal`,
  which means NaN values compare equal to every other value
  — `max_by` does not "beat" NaN with a finite number, and
  the result depends on iteration order. This is ambiguous
  semantics and would also trip `RunResult::assert_finite()`
  at `competition.rs:66-76` downstream, which panics on NaN.
- **Shape (iii): change `EpochMetrics::mean_reward` from
  `f64` to `Option<f64>`.** The cleanest type-level fix, but
  requires changing the public `EpochMetrics` struct at
  `algorithm.rs:32-45` (recon-reported), which is a wider
  surface change than Ch 24 §3.7 flagged. Every caller that
  reads `m.mean_reward` would have to handle the `Option`,
  and the recon-reported grep across the ml-bridge tree
  returned direct `m.mean_reward` or `metrics[...].mean_reward`
  reads at `competition.rs:48,:166,:167,:206,:345,:351,:386`
  (ranked and summary printing, provenance construction, the
  verbose-mode epoch callback, and `find_replicate`'s eventual
  consumers); additional reads live in the integration-test
  tree at `sim/L0/thermostat/tests/d1c_cem_training.rs`,
  `d1d_reinforce_comparison.rs`, and `d2c_cem_training.rs`
  (recon-reported as file-level hits; the exact per-line
  counts are a PR 2b author's grep concern, not a Ch 41
  citation). Every one would have to handle the `Option`.
- **Shape (iv): emit `EpochMetrics { mean_reward: 0.0, .. }`
  with a marker in `extra`.** Keep the current emit, add an
  `extra.insert("zero_samples".into(), 1.0)` so downstream
  callers can check. Decision 2's primitive would have to
  recognize the marker and filter. Adds coupling between the
  REINFORCE/PPO cleanup and Decision 2's helper
  implementation.

**The lean is Shape (i) — skip the `metrics.push` entirely.**
The argument is a combination of simplicity and scope. Shape
(i) is the minimum-diff form that makes Ch 24 §4.7's
silent-filter-out well-defined: zero-epoch runs map to `None`
from `RunResult::best_reward()`, zero-replicate-set runs map
to an empty `Vec<f64>` from `replicate_best_rewards`, and
`describe` returns `None` for completely degenerate cases.
Shape (ii) has the NaN ambiguity Ch 24 §4.7 described as "the
worst of both worlds" recast in type-level form. Shape (iii)
is the right answer in the abstract but the struct rewrite
has a blast radius that goes well outside PR 2's scope —
every caller of `EpochMetrics` across `competition.rs`,
`d1c_cem_training.rs`, `d1d_reinforce_comparison.rs`,
`d2c_cem_training.rs`, and the six integration test files
that read `mean_reward` directly would have to handle the
`Option`, which would bloat PR 2b from a surgical fix into a
multi-crate refactor. Shape (iv) couples the REINFORCE/PPO
cleanup to Decision 2's helper logic in a way that makes
both harder to audit.

Shape (iii) would also be the structurally cleanest answer if
PR 2 were scoped to chassis surface redesign. It is not — Ch
24 §3.7 explicitly declined to widen Decision 1's scope
beyond the train-loop rewrites, and PR 2 inherits that
narrower scope. A future chassis-API PR that does redesign
`EpochMetrics` (to handle missing data via `Option`, to add a
per-episode-complete flag, to surface the unit in the type
system) would land shape (iii) naturally; PR 2's scope is
the surgical semantic fix, not the structural redesign.

Shape (i) has one risk worth naming: it changes `metrics.len()`
from being equal to the nominal epoch count to being less
than or equal. A test that asserts
`metrics.len() == expected_n_epochs` and runs against an
algorithm whose training actually triggers the short-circuit
would fail. The recon-surfaced census of such assertions
returned six hits across ml-bridge — `cem.rs:313`,
`reinforce.rs:364`, `ppo.rs:539`, `sac.rs:678`, `td3.rs:623`,
and `ml-bridge/tests/custom_task.rs:99` — each asserting a
specific epoch count on a small smoke-test training run on a
reaching task. In all six cases, the training runs on well-
specified reaching-task environments where `collect_episodic_rollout`
produces at least one step per trajectory per env, which
means `n_samples > 0` is structurally guaranteed and the
zero-fallback short-circuit never fires. None of the existing
assertions break under shape (i). The census is
recon-reported and worth naming here so a PR 2b reviewer does
not have to re-derive it.

A future test that deliberately triggers the zero-fallback
(e.g., to verify the skip-the-push behavior) would need to
assert `metrics.len() < n_epochs` rather than `==`. Such a
test is added in §3 as part of PR 2b's new-regression-test
additions.

**Concrete patch.** At `reinforce.rs:213-225` (recon-reported)
— the current if-block is:

```rust
if n_samples == 0 {
    let em = EpochMetrics {
        epoch,
        mean_reward: 0.0,
        done_count: 0,
        total_steps: 0,
        wall_time_ms: t0.elapsed().as_millis() as u64,
        extra: BTreeMap::new(),
    };
    on_epoch(&em);
    metrics.push(em);
    continue;
}
```

The patched if-block drops the `metrics.push(em)` call, the
preceding `on_epoch(&em)` call, and the preceding
`EpochMetrics` construction, replacing the whole short-circuit
with a bare `continue`:

```rust
if n_samples == 0 {
    continue;
}
```

Three lines replace thirteen lines (net `−10`). No `EpochMetrics` is
constructed, no `on_epoch` callback fires (the callback
contract now says "called once per epoch that produces
usable data," which is a narrower guarantee than the current
behavior's "called once per epoch iteration"). The `continue`
still advances the epoch loop counter, so `epoch` values
remain contiguous from the algorithm's perspective — they
just do not all appear in the `metrics` vector.

The PPO patch at `ppo.rs:313-325` (recon-reported) is
line-for-line the same shape. The `n_samples == 0` block
drops its `EpochMetrics` construction and its `metrics.push`,
leaving a bare `continue`.

**A note on the `on_epoch` callback contract.** The callback
at `competition.rs:341` (recon-reported) is fired inside
`Algorithm::train` once per epoch that the algorithm decides
to produce. Today's behavior is "once per epoch iteration
including degenerate ones"; post-shape-(i), the contract
becomes "once per epoch iteration that produces a usable
metrics entry." This is a subtle contract change in the
`Algorithm` trait but affects no current caller because the
callback's only current users are `Competition`'s
verbose-mode logging at `competition.rs:341-348` and any test
callback captures. The verbose logging prints per-epoch
progress to stderr, and a skipped epoch means one fewer
print-line — a benign change. Test callbacks that count
callback invocations would see fewer invocations, but the
existing tests do not do this (the six `metrics.len() == N`
assertions listed above read from the returned `metrics`
vector, not from callback count).

### 2.5 The `d2c_cem_training.rs` doc-only update (PR 2b)

This is one of the sub-decisions — §3 argues the scope —
and the patch itself is minimal. `d2c_cem_training.rs`
at `sim/L0/thermostat/tests/d2c_cem_training.rs` does not
import `Competition`, does not call `best_reward()` or
`final_reward()`, and consists of four independent
`#[test]` functions (`d2c_cem` at `:276`, `d2c_td3` at
`:295`, `d2c_ppo` at `:331`, `d2c_sac` at `:356`, all
recon-reported) that each train one algorithm and assert
per-algorithm gates on synchrony and reward improvement.
None of the test gates are broken by Decision 1's
unit-standardization — Gate B's monotonicity check
(`best_last_10 > first_5_mean` at `d2c_cem_training.rs:231,
:235`, recon-reported) depends on the *ordering* of
`mean_reward` values within one algorithm's training
trajectory, not on their absolute scale, and a within-
algorithm positive rescaling preserves the `>` comparison.

What PR 2b does add to the file is doc-only: a file-level
doc comment extending the existing module-level doc block at
`d2c_cem_training.rs:1-9` (recon-reported) to name the
post-Decision-1 `mean_reward` unit so a reader of the eprintln
output at `d2c_cem_training.rs:218-221` (recon-reported — the
eprintln macro call inside the training callback block at
`:216-223`) understands what the number means, and an updated
eprintln format string at `:219` that includes the unit inline
(e.g., `"{name} epoch {:3}: mean_reward = {:+.6} (per-episode
total across n_envs trajectories)"`). Both changes are narrative: the test
assertions do not change, the gate logic does not change,
and the file's behavior under `cargo test --release
--ignored` is identical. The point of the doc is not to make
the test correct — it is already correct — but to make the
eprintln output unambiguous for a human reading it.

The narrower reason for the doc-only scope is structural.
The D2c rematch — the experiment that genuinely needs the
unit-uniform surface Decision 1 provides — is *not*
implemented in `d2c_cem_training.rs`. It is implemented
(when Ch 42 ships) in a new test fixture under
`sim-opt/tests/` that will use `Competition::run_replicates`
with the matched-complexity anchor from Ch 23 §3 and the
folded-pilot protocol from Ch 32 §4.6. The current
`d2c_cem_training.rs` is a legacy file that documents the
D2c experiment's *training runs*, not its rematch. Rewriting
it to use `Competition::run_replicates` would be a PR 3
concern — and PR 3 will build a different test fixture
anyway, leaving `d2c_cem_training.rs` either deprecated or
retired.

PR 2b's scope is therefore doc-only on `d2c_cem_training.rs`:
the module-level doc comment gets a unit note, the eprintln
format string gets a unit suffix. Nothing else.

### 2.6 Test surface additions

PR 2a and PR 2b each ship new regression tests alongside
their source changes. The additions are enumerated here so a
PR 2 reviewer has a checklist — every entry either lands as a
new `#[test]` function in the relevant file or extends an
existing test module, and the table doubles as a test-scope
gate on what "PR 2 is ready for review" means.

PR 2a (additive API) adds these tests in the ml-bridge crate's
`competition.rs` test module at `sim/L0/ml-bridge/src/competition.rs`
(inside the existing `#[cfg(test)] mod tests` block at
`competition.rs:416-900` recon-reported as the module span,
joining the existing tests `competition_runs_all_pairs` at
`:520` and its siblings through `:875`):

1. **`run_replicates_flat_shape`.** Asserts `result.runs.len()
   == tasks.len() * builders.len() * seeds.len()` for a
   two-task, two-builder, three-seed call. The lean shape of
   the test: construct two tasks via `reaching_2dof()` and
   `reaching_6dof()`, two `MockAlgorithm` builders, a seed
   slice of `[42, 43, 44]`, and call
   `comp.run_replicates(&tasks, &builders, &[42, 43, 44])`.
   Assert `result.runs.len() == 12`.
2. **`run_replicates_replicate_index_monotonic`.** For each
   `(task, algo)` pair in the 12-element result, filter via
   `for_task` + `for_algorithm` and assert the filtered slice
   has `replicate_index` values `[0, 1, 2]` in the order they
   appear. Verifies the seeds-outermost loop ordering and the
   index stamping.
2a. **`run_replicates_preserves_strict_gt_tie_breaking`.** A
    regression test for the inline best-epoch scan's
    strict-`>` tie-breaking invariant at `competition.rs:363-379`
    (recon-reported). §4.1 flags this as the load-bearing
    review-time attention point for PR 2a, because the inline
    scan and `RunResult::best_reward()` at `:45-50`
    (recon-reported) produce the same scalar but differ on
    tie-breaking — the scan uses strict `>` with
    `f64::NEG_INFINITY` seed (earlier epoch wins under a tie),
    while `best_reward()` uses `max_by` with
    `partial_cmp`→`Ordering::Equal` fallback (iterator order
    wins under a tie, which in practice is the *later* epoch).
    The test's shape: a `MockAlgorithm` variant that emits
    exactly two `EpochMetrics` entries with *identical*
    `mean_reward` values (e.g., both `-1.0`); call
    `comp.run_replicates(...)` with a single task, single
    builder, single seed; retrieve the resulting `RunResult`;
    inspect its `provenance.as_ref().unwrap().best_epoch` and
    assert it equals `0`, not `1`. A refactor that replaced
    the inline scan with `RunResult::best_reward()` would
    fail this test at the `best_epoch` comparison because
    `max_by` under `Ordering::Equal` returns the last
    iterated element (epoch 1), not the first (epoch 0).
    This test is the runtime gate that the §4.1 review-time
    comment relies on — without it, the tie-breaking
    invariant is a code-review convention, not an enforced
    contract.
3. **`run_replicates_fresh_env_per_pair`.** Pushes through the
   same `MockAlgorithm` builder for two different tasks with
   three seeds, and verifies that the per-run `metrics.len()`
   is consistent (no cross-run state leakage from a stale
   `VecEnv`). This is a regression guard against a refactor
   that drops the fresh-per-pair `VecEnv` construction at the
   current `competition.rs:330` (recon-reported) inside the
   new shared loop body.
4. **`run_still_works_single_seed`.** The existing
   `competition_runs_all_pairs` test at `competition.rs:520-538`
   (recon-reported) already exercises the `run` → `run_replicates`
   wrapper path by virtue of `run` being the thin forwarder,
   but PR 2a adds an explicit assertion that a fresh
   `RunResult` produced via `comp.run(...)` has
   `replicate_index == 0` on every element. Three lines of
   assertion inside a new `#[test]` function.
5. **`find_replicate_returns_specific_seed`.** A three-seed
   call, then `find_replicate("reaching-2dof", "Mock", 1)`
   returns the second replicate; `find_replicate(..., 99)`
   returns `None`.
6. **`replicate_best_rewards_flat_filter`.** A three-seed call
   on `MockAlgorithm`, then assert
   `replicate_best_rewards("reaching-2dof", "Mock").len() == 3`
   (the Mock always produces finite rewards; silent-filter-out
   behavior is tested separately below).
7. **`describe_returns_seed_summary`.** Same three-seed setup;
   assert `describe("reaching-2dof", "Mock").is_some()` and
   that the returned `SeedSummary` has `n == 3`, `mean` is
   finite, `std_dev >= 0.0`.
8. **`describe_none_for_missing_pair`.** Assert
   `describe("no-such-task", "Mock").is_none()`.
9. **`seed_summary_from_rewards_n1_returns_zero_stddev`.**
   Unit test on `SeedSummary::from_rewards(&[1.0])` — assert
   `n == 1`, `mean == 1.0`, `std_dev == 0.0`. Covers the
   degenerate `n == 1` case Ch 24 §4.6 named.
9a. **`seed_summary_from_rewards_empty_returns_none`.** One-line
    unit test: `assert!(SeedSummary::from_rewards(&[]).is_none())`.
    Covers the empty-slice contract Ch 24 §4.3 locked ("Build
    a summary from a slice of rewards. Returns `None` if the
    slice is empty."). Without this test, a future refactor
    that accidentally returns `Some(SeedSummary { n: 0, mean:
    f64::NAN, std_dev: f64::NAN })` on empty input would ship
    unnoticed.
10. **`seed_summary_from_rewards_bessel_correction`.** Unit
    test on `SeedSummary::from_rewards(&[1.0, 2.0, 3.0])` —
    assert `mean == 2.0` and `std_dev == 1.0` exactly (sample
    std with `n-1 = 2` denominator on the variance-summed
    `[1.0, 0.0, 1.0]` = `2.0 / 2.0` = `1.0`, so `std_dev =
    sqrt(1.0) = 1.0`). Catches a population-vs-sample std
    regression in one line.
11. **`replicate_best_rewards_silent_filter_out`.** A test
    that deliberately produces a `None` from
    `RunResult::best_reward()` on at least one replicate (via
    a `MockAlgorithm` variant whose `train` returns an empty
    `Vec<EpochMetrics>` for one seed and finite metrics for
    the others) and asserts the silent filter-out:
    `replicate_best_rewards(...).len() < seed_count` on the
    pair where the `None` replicate lives. Also asserts that
    `SeedSummary.n` reflects the filtered count rather than
    the seed count.

Thirteen new tests in `competition.rs`'s test module for PR 2a.

PR 2b (semantic fix) adds these tests in the algorithm files
themselves and in one new location:

12. **`cem_mean_reward_is_per_episode_total`.** A unit test in
    `cem.rs`'s test module (at or near the existing
    `cem_smoke_2dof` at `cem.rs:292`, recon-reported, inside
    the `#[cfg(test)] mod tests` block at `cem.rs:264-414`) that
    trains a CEM instance on a reaching task with a known
    per-step reward scale and asserts that the final epoch's
    `mean_reward` is consistent with "per-episode total"
    semantics rather than "per-step mean" — specifically,
    assert `final_reward > 10.0` or similar magnitude gate,
    because the reaching task at `max_episode_steps = 300`
    with a policy that earns approximately −1 per step
    produces a final reward near −300, not near −1. The exact
    threshold is a drafting detail of the PR; the gate's
    shape is "the reported value is in the right order of
    magnitude for a per-episode total."
12a. **`cem_dual_reward_concept_split`.** The regression guard
    for Ch 24 §3.5's scope-discipline argument — CEM carries
    two reward concepts internally under PR 2b: the
    length-normalized fitness at `cem.rs:182-183`
    (recon-reported), used for elite selection, stays in
    per-step units; the reported `mean_reward` at `cem.rs:209`
    moves to per-episode-total units. §2.3's CEM
    `elite_mean_reward_per_step` key rename surfaces the
    per-step concept in the extras map. This test asserts
    both concepts simultaneously on a single run: train CEM
    on `reaching_2dof()` for a few epochs, inspect the last
    epoch's `EpochMetrics`, and assert
    `last.mean_reward.abs() > last.extra["elite_mean_reward_per_step"].abs() * 10.0`
    — the per-episode total's absolute magnitude must be at
    least an order of magnitude larger than the per-step
    mean's absolute magnitude, for any reasonable episode
    length at the reaching task's `max_episode_steps`. This
    catches any future refactor that accidentally crosses the
    two computations (e.g., a well-meaning cleanup that folds
    the reporting site at `:209` back into the fitness loop,
    losing the per-episode semantics). Test (12) only asserts
    the reporting side; test (12a) asserts the invariant that
    makes Ch 24 §3.5's "real cost" framing load-bearing.
13. **`td3_mean_reward_is_per_episode_total`.** Same shape as
    (12), in `td3.rs`'s test module at or near the existing
    `td3_smoke_2dof` at `td3.rs:617`, recon-reported, inside
    the `#[cfg(test)] mod tests` block beginning at
    `td3.rs:574`. The gate asserts magnitude consistency with
    per-episode total units, not per-step or
    mean-over-completed-episodes.
14. **`sac_mean_reward_is_per_episode_total`.** Same shape as
    (12), in `sac.rs`'s test module at or near the existing
    `sac_smoke_2dof` at `sac.rs:672`, recon-reported, inside
    the `#[cfg(test)] mod tests` block beginning at
    `sac.rs:630`.
15. **`td3_epoch_rewards_invariant_holds`.** Debug-only test
    (`#[cfg(debug_assertions)]`) that exercises the pre-loop
    + `debug_assert_eq!` invariant from §2.2. Runs TD3 with a
    deliberately short `max_episode_steps` that could allow
    some envs to remain `env_complete = false` at the cutoff,
    and asserts the debug-assert does not fire. The test's
    existence documents the invariant; if the assertion ever
    starts firing in CI, the test's failure surfaces the
    invariant break at the location Ch 41 guards.
16. **`sac_epoch_rewards_invariant_holds`.** Same shape as
    (15), in `sac.rs`.
17. **`cem_elite_mean_reward_per_step_key_present`.** Rename
    of the existing unit test at `cem.rs:319` (and the sibling
    at `:354` which also reads the key, recon-reported).
    Existing name `cem_smoke_2dof` at `:292` (recon-reported)
    stays; the two specific `contains_key` and indexing
    assertions update to the new key name
    `elite_mean_reward_per_step`. These are *not* new tests —
    they are in-place updates to existing tests — but they
    are listed here because a reviewer looking at the PR 2b
    diff should see the rename explicitly.

Six new tests plus one in-place rename for PR 2b. Combined
with PR 2a's thirteen, PR 2 ships nineteen new `#[test]`
functions across the ml-bridge crate plus one in-place
test rename. The original recon-to-leans plan also named two
zero-fallback runtime tests (`reinforce_zero_fallback_skips_epoch`
and `ppo_zero_fallback_skips_epoch`) as potential PR 2b
additions; §2.6's closing note below explains why they are
dropped from the runtime enumeration and what guards the
zero-fallback skip-the-push semantic instead.

**A closing note on the zero-fallback's test coverage.** The
initial recon-to-leans note listed two runtime regression
tests for §2.4's shape (i) — one for REINFORCE's
`:213-225` skip-the-push and one for PPO's `:313-325`
equivalent — marked conditional on whether the zero-fallback
branch is reachable from the existing `collect_episodic_rollout`
helper's env/task surface. §2.4's thinking-pass review
established that the zero-fallback branch is structurally
unreachable from every currently-shipping task in the
workspace: `collect_episodic_rollout` runs every env through
at least one step before evaluating trajectory completeness,
and the reaching-task environments that every existing test
trains on produce `n_samples > 0` by construction. A runtime
test exercising the skip-the-push would need to either
instrument `collect_episodic_rollout` itself (out of PR 2's
scope) or introduce a task whose rollout returns zero-length
trajectories on purpose (requires building a pathological
test fixture whose value is limited to exercising exactly
this one branch).

Ch 41 therefore **drops tests (17) and (18) from the runtime
enumeration entirely** and rests §2.4's safety argument on
two load-bearing static analyses instead: (a) the
`metrics.len() == N` census across the six existing assertions
listed in §2.4, each of which is verified to train on an env
where `n_samples > 0` is structurally guaranteed (so the
skip-the-push never fires and the assertions never see a
shortened `metrics` vector), and (b) the `on_epoch` workspace
grep in §2.4 confirming that no in-repo caller counts
callback invocations to infer `n_epochs`. These two analyses
are the load-bearing safety argument for shape (i); a runtime
regression test would be a belt-and-suspenders guard on top
of them but cannot be built against the current env/task
surface without introducing a pathological fixture whose
cost exceeds its benefit. A future PR that introduces an env
or task whose rollout can produce zero-length trajectories
(e.g., a custom benchmark task with an aggressive early-
termination condition) would be the right time to revisit
and add the runtime test — PR 2b is not that PR, and the
chapter names the deferral explicitly rather than leaving
it as a "PR 2b author drafting concern."

A note on test placement. The PR 2a tests live inside
`competition.rs`'s existing `#[cfg(test)] mod tests` block
beginning at `competition.rs:416` and running to `:900`
(recon-reported, module-EOF), because that is where all
`CompetitionResult` tests live today and the module layout
does not need a new test file. The PR 2b tests live inside
each algorithm's existing `mod tests` block: `cem.rs:264-414`,
`td3.rs:574-EOF`, and `sac.rs:630-EOF` (recon-reported as
module-start lines; the end-of-file lines are implicit
because each `mod tests` runs to the file's EOF). There is
no new test file for `SeedSummary` specifically because
§2.1 places `SeedSummary` inline in `competition.rs` and
its tests belong next to it.

## Section 3 — Scope of `d2c_cem_training.rs` updates (sub-decision)

This sub-decision is important enough to get its own section
because the initial recon pass surfaced a factual
discrepancy with Ch 24 that narrows Ch 41's scope
substantially and also implies a narrow post-commit patch
to Ch 24 itself. §5's scope-discipline section handles the
patch; this section defends the narrowed scope for PR 2b.

### 3.1 Ch 24 §1.9, §2.2, and §5 overclaim the test's brokenness

The overclaim appears in three Ch 24 sites, worth naming
individually because the bundled patch amends each.

**Ch 24 §1.9** (at `24-result-semantics.md:428-433`,
recon-reported) reads: "The D2c rematch test at
`sim/L0/thermostat/tests/d2c_cem_training.rs` (recon-reported,
exact line range deferred to factual pass) compares
`best_reward()` values across algorithms and passes or fails
the test based on those comparisons. Whatever that test was
measuring, it was not what the name suggested it was
measuring." This is the strongest form of the overclaim and
the one with the load-bearing effect on Ch 24's "what this
means in practice" framing.

**Ch 24 §2.2** (at `24-result-semantics.md:492-505`,
recon-reported) reads: "PPO's D2c `best_reward` was in
reward-per-episode units. CEM's D2c `best_reward` was in
reward-per-step units. **The D2c rematch test compared them
directly.** … The D2c test was apples-to-oranges against all
three off-policy algorithms at once." This echoes the
overclaim in a corroborating-observation framing: the claim
is that the test compared `best_reward` values directly, which
implies the test reads `best_reward()` and uses it in a
cross-algorithm assertion.

**Ch 24 §5** (at `24-result-semantics.md:1094-1100`,
recon-reported) reads: "Whether the D2c test gate at
`sim/L0/thermostat/tests/d2c_cem_training.rs` needs to be
rewritten. It does — the test is unit-broken against CEM
versus any of the other four algorithms — but rewriting the
test is a Chapter 41 execution-layer concern."

All three claims are factually incorrect at the source level.
`d2c_cem_training.rs` does not import `Competition` — the
use statement at `:27-33` (recon-reported) pulls
`Algorithm`, `Cem`, `Ppo`, `Sac`, `Td3`, `LinearPolicy`,
`LinearStochasticPolicy`, `LinearQ`, `LinearValue`, and the
environment and tensor types, but `Competition` is not on
the list. The file does not call `best_reward()` or
`final_reward()` anywhere — grep across the file returns
zero hits for either helper. Its four `#[test]` functions
(`d2c_cem` at `:276`, `d2c_td3` at `:295`, `d2c_ppo` at
`:331`, `d2c_sac` at `:356`, all recon-reported) each train
a single algorithm via direct `algo.train(...)` calls and
evaluate deterministically via a per-algorithm
`evaluate_policy` helper at `d2c_cem_training.rs:159-191`
(recon-reported). Each test's pass/fail is determined by
per-algorithm Gate A (`|t| > 2.861` on the synchrony
t-stat) and Gate B (`best_last_10 > first_5_mean` on the
`mean_reward` fold) — no cross-algorithm comparison lives
in any assertion.

The cross-algorithm comparison in the D2c "experiment"
lives in two places, neither of them inside the test file:

1. **The D2 SR findings memo** (referenced as
   `project_d2_sr_findings.md` in project memory, not a
   file in the repo). The memo records the four-algorithm
   comparison's headline numbers and classifies PPO as a
   "PASS*, false positive, synchrony from transient not SR"
   (recon-reported from the project memory summary in
   previous study sessions). The comparison is narrative,
   not programmatic.
2. **Human reading of the eprintln output.** The test file
   at `d2c_cem_training.rs:216-261` (recon-reported) prints
   per-epoch `mean_reward` values and per-algorithm gate
   results to stderr. A human running `cargo test --release
   --ignored` on all four tests sees four eprintln blocks
   side by side and draws conclusions from them. The
   conclusions are human-constructed, not test-enforced.

The "unit-broken" framing applies to the D2 SR findings
memo's cross-algorithm numbers and to the human-read
eprintln output. It does not apply to any assertion inside
`d2c_cem_training.rs`. Gate B's `best_last_10 > first_5_mean`
check is a within-algorithm monotonicity gate that works
identically before and after Decision 1, because a positive
scalar rescaling of `mean_reward` values preserves the `>`
ordering used in the fold. A policy that improves under the
old metric improves under the new metric, and the gate
fires the same way.

### 3.2 What this means for PR 2b's scope

If the overclaim held — if any of Ch 24's three claimed
occurrences actually matched a cross-algorithm `best_reward()`
comparison in the test file — PR 2b would need to rewrite
the test gates
to use unit-correct aggregation, which would mean wiring
`Competition::run_replicates` into a new gate structure,
adding a cross-algorithm comparison built on `SeedSummary`,
and handling the per-algorithm hyperparameter matching.
That is a substantial rewrite, and it is what Ch 24 §5
appeared to be committing Ch 41 to.

Since the overclaim does not hold, PR 2b's scope for
`d2c_cem_training.rs` narrows to what §2.5 describes:
doc-only updates to the module comment and the eprintln
format strings to name the post-Decision-1 unit. No
assertion changes, no `Competition` wire-up, no
cross-algorithm gate rewrite. The substantial rewrite (if
one is ever written) lives in Ch 42's PR 3, which ships a
new rematch test fixture under `sim-opt/tests/` using
`Competition::run_replicates` against Ch 32's folded-pilot
protocol. That fixture will be a different file, not a
rewrite of `d2c_cem_training.rs`, because the rematch runs
`{CEM, SA}` at matched complexity with the Ch 32 seed
derivation rule, not the original four-algorithm D2c pool.
`d2c_cem_training.rs` is the D2c experiment's legacy
training-run documentation; the rematch is a different
experiment.

### 3.3 The Ch 24 patch bundled into the Ch 41 commit

Following the `b5cb3f6c` precedent (which bundled a Ch 15
§5.6 citation patch with a Ch 40 §5 scope correction in the
same commit), Ch 41's commit bundles a narrow post-commit
patch to Ch 24 §1.9, §2.2, and §5 that corrects the three
overclaim occurrences. The patch amends each site in place:

- **§1.9** (`24-result-semantics.md:428-433`) — the sentence
  "The D2c rematch test at … compares `best_reward()` values
  across algorithms and passes or fails the test based on
  those comparisons" is replaced with a corrected framing
  naming the actual locus (the D2 SR findings memo + human-
  read eprintln output) rather than a nonexistent test gate.
  Gate A and Gate B's per-algorithm nature is named
  explicitly so a reader of the patched §1.9 understands why
  the within-algorithm gates are robust to Decision 1's
  rescaling.
- **§2.2** (`24-result-semantics.md:492-505`) — the sentence
  "The D2c rematch test compared them directly" is replaced
  with a corrected framing naming the narrative comparison
  in the D2 SR findings memo, not a `best_reward()` read in
  the test file. The surrounding paragraph's "The D2c test
  was apples-to-oranges against all three off-policy
  algorithms at once" becomes "The D2c comparison (in the
  D2 SR findings memo) was apples-to-oranges." The supporting-
  observation framing — "two independent mechanisms by which
  the D2c PPO number fails to mean what its name suggests"
  — survives unchanged; only the test-file attribution moves
  to the memo attribution.
- **§5** (`24-result-semantics.md:1094-1100`) — the bullet
  "Whether the D2c test gate at … needs to be rewritten. It
  does — the test is unit-broken against CEM versus any of
  the other four algorithms — but rewriting the test is a
  Chapter 41 execution-layer concern" is replaced with a
  corrected framing: the test file's per-algorithm gates are
  not unit-broken; the D2 SR findings memo's cross-algorithm
  comparison is unit-broken; Ch 41's doc-only scope addresses
  the eprintln output; Ch 42's PR 3 ships a new rematch test
  fixture under `sim-opt/tests/` rather than rewriting
  `d2c_cem_training.rs`.

The bundled patch does not change Ch 24's Decision 1 or
Decision 2, does not change Ch 24's scope discipline, and
does not alter any of Ch 24's file:line citations — the
patch is narrow and factual-accuracy-preserving. The
supporting-observation axis Ch 24 §2.2 makes ("two
independent mechanisms by which the D2c PPO number fails to
mean what its name suggests") survives intact; only the
test-file attribution is corrected to name the memo as the
actual locus.

The patch is bundled into the Ch 41 commit rather than
landed as a separate follow-up commit because §2.5 and §3.2
of this chapter depend on the corrected Ch 24 framing for
the doc-only narrowing to make sense. A reader who sees the
Ch 41 commit first and then has to chase a separate Ch 24
patch commit to understand why PR 2b's scope is narrow is
reading in a harder order than the one-commit bundle
allows. The precedent is `b5cb3f6c`, which made exactly this
call for the Ch 15 §5.6 / Ch 40 §5 cross-reference.

## Section 4 — PR split and landing sequence

### 4.1 The 2-PR split: PR 2a additive, PR 2b semantic

Ch 41's second in-chapter sub-decision (third, if the
§2.2 debug-assert count separately) is whether PR 2 ships
as one PR or two. Three shapes are worth naming:

- **Shape (i): one PR.** The full diff lands in a single
  review cycle. Additive API, algorithm fixes, zero-fallback
  cleanup, CEM rename, `d2c_cem_training.rs` doc update —
  all in one commit-set. Smaller PR count, single review.
- **Shape (ii): two PRs — PR 2a additive, PR 2b semantic.**
  PR 2a ships the `run_replicates` + `replicate_index` +
  aggregation helpers (purely additive, zero semantic effect
  on existing callers). PR 2b ships the train-loop rewrites,
  the zero-fallback cleanup, the CEM rename, and the
  `d2c_cem_training.rs` doc update. Two review cycles, PR
  2a can land independently and any behavior regression
  surfaces only under PR 2b.
- **Shape (iii): three PRs — split semantic further.** PR 2a
  additive; PR 2b' Decision 1 train-loop rewrites; PR 2b''
  everything else (zero-fallback cleanup, CEM rename, doc
  update). Three review cycles, finest-grained rollback
  control.

**The lean is shape (ii).** The argument has four parts.

**Part 1: additive vs semantic is the natural fracture line.**
PR 2a's diff has zero observable effect on any existing
caller. Every `Competition::new(...)` callsite at
`competition.rs:520-875` (recon-reported) and in
`ml-bridge/tests/competition.rs` continues to produce
identical results — the single-seed path in `run_replicates`
is the same code as the current `run` body with a
`replicate_index = 0` stamped on the output, and the
`RunResult::replicate_index` field's presence is not
asserted against in any existing test. PR 2a is therefore
review-equivalent to "add a new public API surface without
touching any existing behavior." A reviewer's job on PR 2a
is to read the new `run_replicates` body, verify it
preserves the existing per-pair loop shape including the
inline best-epoch scan, and check the new helpers.

PR 2b is the opposite. Every existing test that asserts on
absolute reward magnitudes — and the census of such tests
in the ml-bridge tree finds approximately 47 `final_reward()`
call sites at `sim/L0/ml-bridge/tests/competition.rs`
(recon-reported, across a line range spanning `:891` through
`:1458`) where values are compared against specific thresholds
— could in principle be affected by Decision 1's
CEM rewrite if the test runs CEM. The census is
recon-reported and worth running during PR 2b's author
phase: any test asserting `cem.final_reward() > some_number`
against a specific threshold will see a different number
post-Decision-1. Whether those tests still pass depends on
whether the threshold was set conservatively enough to
tolerate the ~500× difference (roughly, since CEM's
per-step values for a well-trained reaching policy are
~1.0 and the per-episode totals are ~500.0 depending on the
episode length) or whether the thresholds were tight. PR 2b
is the PR where that tightness shows up; splitting it from
PR 2a means PR 2a merges first without risk and PR 2b's
review cycle can focus on exactly the compatibility
question without the API-shape review in its way.

**Part 2: PR 2a unblocks Ch 42 earlier.** Ch 42's PR 3 — the
rematch PR — consumes `Competition::run_replicates` and the
`SeedSummary` aggregation surface directly. If PR 2 ships as
one bundle, Ch 42 cannot start writing the rematch test
fixture against the new API until the whole PR 2 bundle is
merged. If PR 2a ships independently, Ch 42 can start
writing the rematch fixture against PR 2a's surface as soon
as PR 2a merges, and PR 2b's merge gates only the
per-algorithm unit uniformity (which matters for the
rematch's numerical results but not for the fixture's
structure). The effective parallelism between PR 2a/PR 2b
review and Ch 42 drafting is worth the two-PR overhead.

**Part 3: PR 2b's risk profile is different from PR 2a's.**
If PR 2a is reverted, the revert is cosmetic: no behavior
change propagates because there is no behavior change to
revert. If PR 2b is reverted, the revert restores the
pre-Decision-1 apples-to-oranges shape across the three
affected algorithms. The rollback decision-making is
genuinely different for the two halves, and bundling them
means a rollback of either becomes a rollback of both.

**Part 4: shape (iii) is overbuild.** The three-PR split is
worth considering because each of the three sub-pieces
(Decision 1 rewrites, zero-fallback cleanup, CEM rename) is
independently testable. But the three are semantically
coupled: the zero-fallback cleanup is required for Ch 24
§4.7's silent-filter-out contingency, the CEM rename is
scoped to a single insert-line and two test lines, and the
Decision 1 rewrites are the load-bearing half. Splitting
them into three PRs adds two extra review cycles for
negligible rollback-granularity benefit. Shape (iii)'s
"fine-grained rollback" framing is worth the overhead only
if each piece has independent risk, and they do not —
rollback of the Decision 1 rewrites would force rollback of
the zero-fallback cleanup because the latter was written
against the former's assumptions about when `mean_reward`
is meaningful, and rollback of the CEM rename alone is not
a meaningful rollback (it is a cosmetic revert). Shape (ii)
is the right cost tier.

Shape (ii) ships.

### 4.2 PR 2a contents and risk

PR 2a's diff, in summary:

- `sim/L0/ml-bridge/src/competition.rs` — one new field on
  `RunResult` (`replicate_index: usize`); one new method
  `run_replicates` replacing the body of the existing `run`;
  `run` becomes a one-line wrapper; three new methods on
  `CompetitionResult` (`find_replicate`,
  `replicate_best_rewards`, `describe`); one new struct
  `SeedSummary` with its `from_rewards` constructor, inline
  below `CompetitionResult`; thirteen new inline unit tests
  covering the additive contract (enumerated at §2.6; the
  count below the `~4` estimate in prior drafts, which was
  stale against the authoritative §2.6 enumeration). Total
  diff: approximately +310 / −30 lines in `competition.rs`
  across two commits (commit 2 adds `run_replicates` +
  `replicate_index` + six tests, commit 3 adds
  `SeedSummary` + aggregation helpers + seven tests).
- `sim/L0/ml-bridge/src/lib.rs` — one `pub use` line for
  `SeedSummary` if the new struct is not re-exported through
  the module root already (recon during drafting confirms
  the existing re-export pattern; a single line may or may
  not be needed depending on how `SeedSummary` is imported).

Two files touched, ~200 lines of code, no other crates
involved.

**Risk profile.** PR 2a is the lowest-risk half of PR 2.
Every existing test passes unchanged by construction: the
`run_replicates` body is the existing `run` body with an
outer seeds loop of length one, the `replicate_index` field
is populated as `0` for every single-seed call, the new
helpers are read-only accessors that never fire for any
existing test. The only way PR 2a breaks something is if a
transcription bug in the `run_replicates` body alters the
existing per-pair behavior — the mitigation is that the
full existing competition test suite at
`competition.rs:520-875` runs as a regression gate, and any
transcription bug in the inner loop would trip it.

The inline best-epoch scan preservation at
`competition.rs:363-379` (recon-reported) is the specific
attention point. A code review of PR 2a should verify that
the scan is moved inside the new `run_replicates` body
verbatim, including the strict-`>` tie-breaking, the
`f64::NEG_INFINITY` seed, and the `metrics.is_empty()` /
non-finite check. A rewrite of the scan to use
`RunResult::best_reward()` would be subtly wrong (different
tie-breaking) and should not be accepted without a separate
discussion about the `best_epoch` semantic.

### 4.3 PR 2b contents and risk

PR 2b's diff, in summary:

- `sim/L0/ml-bridge/src/cem.rs` — `mean_reward` rewrite at
  `:209` (5 lines for the new `total_reward` computation
  and `mean_reward` assignment); `elite_mean_reward` key
  rename at `:219` (1 line); unit test updates at `:319`
  and `:354` for the renamed key (2 lines). Total: ~8 lines.
- `sim/L0/ml-bridge/src/td3.rs` — pre-loop + denominator
  swap at `:487-491`, with the `debug_assert_eq!` invariant
  (5 lines replacing 5 lines, net zero). Total: ~5 lines.
- `sim/L0/ml-bridge/src/sac.rs` — same shape as TD3 at
  `:540-544`. Total: ~5 lines.
- `sim/L0/ml-bridge/src/reinforce.rs` — zero-fallback
  cleanup at `:213-225` (drops 10 lines of `EpochMetrics`
  construction + `on_epoch` + push; keeps the `continue`).
  Total: −10 lines.
- `sim/L0/ml-bridge/src/ppo.rs` — zero-fallback cleanup at
  `:313-325`, same shape as REINFORCE. Total: −10 lines.
- `sim/L0/thermostat/tests/d2c_cem_training.rs` — module-
  level doc comment addition, eprintln format string update
  at `:216-222`. Total: ~6 lines.

Six files touched, approximately 30 lines of net diff (the
reinforce/ppo cleanup removes ~20 lines, the other five files
add or replace on the order of 25 lines total). Much smaller than PR 2a's surface but the
semantic load is concentrated here.

**Risk profile.** PR 2b is the higher-risk half.

The specific risks:

- **Reward-threshold tests.** The ml-bridge integration tests
  at `sim/L0/ml-bridge/tests/competition.rs` that assert on
  absolute `final_reward()` values (census count: approximately
  47 call sites spanning `:891` through `:1458`, recon-reported
  as a range rather than a per-line list) may or may not be
  robust to the ~500× scale shift for CEM. PR 2b's author
  must run the existing test suite end-to-end after the
  Decision 1 rewrites land and patch any tight-threshold
  assertions that break. The patches are expected to be
  conservative — raising thresholds from the old per-step
  scale to the new per-episode scale — and should not
  change the semantics of any individual test's pass/fail
  relative to its training outcome. The PR 2b description
  should list the tests patched as part of its landing
  notes, so a reviewer can verify the patches are
  semantic-preserving.
- **d2c_cem_training.rs tests.** The four `#[test]` functions
  are `#[ignore]`d today (per the `#[ignore = "requires
  --release (~10 min)"]` annotation at `d2c_cem_training.rs:275`
  and equivalents for the other three tests, recon-reported),
  which means they do not run under a normal `cargo test`
  and do not run in CI. Their Gate B's `best_last_10 >
  first_5_mean` monotonicity check is robust to Decision 1's
  rescaling by construction (as §3.2 established), but the
  absolute numeric values printed in the eprintln output
  shift by orders of magnitude. The doc-only update in §2.5
  handles the narrative side; the Gate B assertion is
  unaffected.
- **The TD3/SAC invariant assertion.** The
  `debug_assert_eq!` from §2.2 fires only in debug builds.
  A release build does not catch the invariant drift, which
  means any CI build matrix that runs only `--release` would
  not catch an invariant break. Ch 41 flags this as a known
  limitation: the assertion is a regression guard against
  future edits in debug mode, not a runtime invariant
  enforcer in release mode. A stronger guard would be
  `assert_eq!` (always-on), but always-on equality checks
  in hot training loops are a nontrivial cost the CEM/TD3/SAC
  loops do not need. Debug-mode coverage is the right cost
  tier.
- **The zero-fallback contract change.** The `on_epoch`
  callback contract narrows from "once per epoch iteration"
  to "once per epoch iteration that produces usable data."
  No current caller depends on the wider contract (see §2.4),
  but a future caller that counts callback invocations to
  infer `n_epochs` would break. The PR 2b description should
  name the contract change explicitly in the PR body so the
  documentation-style change is visible.

### 4.4 Merge order and cross-PR dependencies with Ch 40

Ch 40's PR 1 and Ch 41's PR 2 are decoupled at the code
level (§1.6). The merge-order options for the four PRs
(PR 1a, PR 1b, PR 2a, PR 2b) are:

- **PR 1a → PR 1b → PR 2a → PR 2b.** Ch 40's bundle lands
  first, then Ch 41's. Conservative ordering; suitable if PR
  2 is the riskier bundle.
- **PR 2a → PR 2b → PR 1a → PR 1b.** Ch 41's bundle lands
  first, then Ch 40's. Plausible if PR 2's review is faster;
  does not affect Ch 42's readiness because both bundles
  have to land before PR 3 anyway.
- **Interleaved: PR 1a → PR 2a → PR 1b → PR 2b.** Additive
  halves land first in either order, semantic halves land
  second. Maximizes parallel review of PR 1b and PR 2b once
  PR 1a and PR 2a are merged.
- **Fully parallel: all four PRs in review concurrently,
  merging as each clears review.** Allowable because the
  diffs share no files and no cross-PR dependencies at the
  code level.

The lean is **any of the above**. Ch 41 does not pick a
specific merge order because the choice is an operational
concern (reviewer availability, CI queue, individual
preference) rather than a technical one. The only hard
constraint is that PR 3 — the Ch 42 rematch PR — depends on
both PR 1b and PR 2b being merged, because PR 3's test
fixture uses both `BatchSim::new_per_env` with per-env
`LangevinThermostat` factories (PR 1b) and
`Competition::run_replicates` with unit-uniform `mean_reward`
(PR 2b). The readiness gate for Ch 42's drafting is the
conjunction; the readiness gate for Ch 42's PR 3 merge is
the same conjunction. Until both are merged, PR 3 can be
drafted against the chapter specs but cannot merge itself.

## Section 5 — In-chapter sub-decisions summary

Ch 41 made six in-chapter sub-decisions — the calls Ch 23,
Ch 24, and Ch 31 §4.2 did not lock and Ch 41 had to pick.
Table 1 names them, records the pick, and summarizes the
reasoning in one line. A reader can use this table as a
quick index into the chapter's sections. Row (g) was added
during Ch 42 drafting as a bundled amendment per the
`843dc21c` precedent — see the note after the table.

| # | Sub-decision | Pick | One-line rationale |
|---|---|---|---|
| (a) | CEM `mean_reward` rewrite form | Form (i): add a second traversal of `rollout.trajectories` at `cem.rs:209`, line-for-line matching REINFORCE/PPO's idiom | Strengthens Ch 24 §1.5's shared-rollout-aggregation-helper observation by surfacing the three-copy duplication; keeps CEM's fitness loop concern-pure |
| (b) | TD3/SAC `debug_assert_eq!` invariant guard | Yes — shape (a), `debug_assert_eq!(epoch_rewards.len(), n_envs)` after the pre-loop in both files | Cheap regression guard against future inner-loop edits that would silently break the `env_complete`/`epoch_rewards.push` relationship |
| (c) | CEM `elite_mean_reward` key | Rename to `elite_mean_reward_per_step`; update the two unit tests at `cem.rs:319,:354` | Unit belongs in the key name, not in a doc comment; zero downstream callers outside the two unit tests |
| (d) | REINFORCE/PPO zero-fallback cleanup | Shape (i): drop the `EpochMetrics` construction, `on_epoch` call, and `metrics.push` from the `n_samples == 0` branch; leave bare `continue` | Minimum-diff form; closes Ch 24 §4.7's silent-filter-out contingency; safe against all existing `metrics.len() == N` assertions per census |
| (e) | `d2c_cem_training.rs` scope in PR 2b | Doc-only — module-level comment + eprintln format string update at `:216-222` | The test file does not use `Competition` or `best_reward()`; its gates are within-algorithm and robust to Decision 1's rescaling. Full rewrite is Ch 42 territory |
| (f) | PR structure | Shape (ii): 2-PR split — PR 2a additive API, PR 2b semantic fix + cleanup + doc update | Additive/semantic is the natural fracture line; PR 2a unblocks Ch 42 drafting; PR 2b's risk profile is different; 3-PR split (shape iii) is overbuild |
| (g) | `TaskConfig::build_fn` seed extension | Extend signature from `Fn(usize) -> Result<VecEnv, EnvError>` to `Fn(usize, u64) -> Result<VecEnv, EnvError>`; thread seed through `run_replicates`'s inner `build_vec_env` call; stock tasks accept-and-ignore the seed | Added during Ch 42 drafting after recon surfaced the gap at the rematch layer: `run_replicates` could not thread a per-replicate seed to stochastic-physics tasks, collapsing half the rematch's seed-population variance. See Ch 42 §2 for the full argument; bundled per the `843dc21c` precedent |

Sub-decision (e) is the one worth the most reader
attention. Ch 24 §1.9, §2.2, and §5 overclaim the structural
scope of "unit brokenness" in `d2c_cem_training.rs`; §3
walks the source evidence for the narrower claim. The
narrower scope changes PR 2b's diff from "substantial
rewrite" to "doc-only update" on the affected file, which
is a meaningful reduction in PR 2b's surface and risk.

**What Ch 41 does *not* treat as sub-decisions.** The
following are mentioned in Ch 41 but inherited wholesale
from Ch 23, Ch 24, Ch 31 §4.2, or the §1 inheritance
sections, and a reader should *not* read them as in-chapter
calls:

- The `run_replicates` return shape (flat `Vec<RunResult>` +
  `replicate_index`) — Ch 23 §1.3.
- The `&[u64]` seed surface — Ch 23 §1.2.
- The sequential execution and first-failure-aborts error
  model — Ch 23 §1.4.
- PPO's pool-membership exclusion — Ch 23 §2.
- The matched-complexity anchor — Ch 23 §3.
- "Fix the algorithms" (Decision 1) over "parallel field"
  or "accept and document" — Ch 24 §3.2–§3.4.
- CEM keeping `fitness` length-normalized at `cem.rs:183` —
  Ch 24 §3.5.
- "Contribute the partial" for TD3/SAC incomplete envs —
  Ch 24 §3.6.
- `SeedSummary`'s three fields (`n`, `mean`, `std_dev`),
  Bessel's correction for sample std, silent-filter-out on
  the raw primitive — Ch 24 §4.3, §4.6, §4.7.
- The Option M (medium) aggregation surface over Option S
  (raw only) or Option L (with percentiles) — Ch 24 §4.4–§4.5.
- The `SeedSummary` placement as inline-in-`competition.rs`
  vs new file — a rendering detail, not a sub-decision
  (§2.1's closing paragraph).

When a PR 2a or PR 2b reviewer challenges any of these, the
correct response is to point at the owning argument chapter
(Ch 23 or Ch 24) and the relevant section number, not to
re-argue the call in the PR's review comments.

## Section 6 — What Chapter 41 does not decide

Ch 41 renders PR 2, not PR 1 or PR 3. It also defers a
handful of calls that surfaced during drafting but are out
of scope.

**Ch 40 (PR 1) scope.** The chassis-reproducibility work is
owned by Ch 40. Ch 41 makes no claim about `prf.rs`'s API
shape, no claim about `BatchSim::new_per_env`'s signature,
no claim about the `LangevinThermostat` rewrite's internals,
and no claim about the 56-call-site D2 ripple. Ch 41 inherits
Ch 40's bundled completion as a prerequisite for Ch 42's PR
3 (§1.6) but does not depend on any specific Ch 40 detail
for PR 2's own execution.

**Ch 42 (PR 3) scope.** The sim-opt split + rematch is owned
by Ch 42. Ch 42 will call `Competition::run_replicates` with
the matched-complexity anchor from Ch 23 §3, the folded-pilot
protocol from Ch 32 §4.6, the splitmix64 seed derivation
shipped in Ch 40's `prf.rs`, and the unit-uniform
`mean_reward` shipped by Ch 41's PR 2b. But the shape of the
rematch's analysis pipeline — how many seeds the pilot uses,
what the SA hyperparameters are, how the bootstrap CI is
computed, what the writeup table looks like, what the
follow-up experiments are — is rendered in Ch 42, not here.
A reader wanting to understand how SA gets into ml-bridge
should read Ch 42 when it ships.

**The new shared rollout-aggregation helper Ch 24 §1.5
hinted at.** Ch 24 §1.5 observed that REINFORCE and PPO have
literally identical five-line `mean_reward` computations and
named this as "evidence that the chassis has a
shared-rollout-aggregation gap." Post-PR 2b, CEM also has an
identical five-line form (§2.2 picked Form (i) on the
explicit grounds that it creates the three-copy duplication
which makes the refactor obvious). A shared helper — e.g., a
method on `EpisodicRollout` called `mean_reward_per_episode(n_envs)`
that encapsulates the five-line idiom — is a future-PR
cleanup that is not scoped to PR 2. A reader considering
whether to add it to PR 2 should reject the idea: the scope
discipline Ch 24 §3 committed to is "fix the reporting
apples-to-oranges at the train loop level," not "grow the
chassis with a new abstraction." The shared helper is a
separate, later PR whose motivation is reuse across
stochastic and off-policy algorithms, and its right time is
after PR 2 has landed and the three-copy duplication is
visible.

**Re-exporting `replay_buffer.rs` or other ml-bridge
internals.** PR 2 touches `reinforce.rs`, `ppo.rs`,
`td3.rs`, `sac.rs`, and `cem.rs` but does not change the
public surface of any of them beyond the `EpochMetrics`-
emission behavior. A reviewer who expects PR 2 to rearrange
ml-bridge's module structure or to grow the crate's public
API beyond `Competition`, `RunResult`, `CompetitionResult`,
and `SeedSummary` is reading the wrong chapter. ml-bridge's
module layout stays put.

**Deprecation of `Competition::run` in favor of
`run_replicates`.** The §2.1 wrapper approach keeps `run`
as a thin forwarder over `run_replicates(..., &[self.seed])`,
which preserves the existing callers' ergonomics. A future
PR could `#[deprecate]` `run` and push all callers toward
`run_replicates` explicitly, but that is a
scope-widening move: `run` has a specific ergonomic
advantage (single-seed callers do not have to construct a
one-element slice) and no downside other than the two-method
surface. Ch 41 does not deprecate `run`; a later PR can if
the two-method surface becomes a burden.

**Deprecation or retirement of `d2c_cem_training.rs`.**
§2.5 and §3.2 establish that the test file is a legacy
record of the D2c experiment's training runs, distinct
from the rematch. Whether to retire the file entirely once
Ch 42's PR 3 ships the rematch — or to keep it as
historical documentation — is a Ch 42 call. PR 2b updates
the file's doc and eprintln but does not retire it.

**The concrete rustdoc text for the rewritten
`EpochMetrics::mean_reward`.** Ch 24 §5's deferred item
"the concrete rustdoc text for the rewritten
`EpochMetrics::mean_reward`" lands as part of PR 2b. The
exact wording is a drafting detail of the PR — something
like "Mean per-episode total reward across `n_envs`
trajectories. All five ml-bridge algorithms (CEM, REINFORCE,
PPO, TD3, SAC) emit values in this unit post-Ch-24
Decision 1." The rustdoc is a documentation detail that PR
2b's author writes alongside the code changes and that the
PR review pressure-tests for clarity.

**The reporting-vs-training asymmetry in TD3/SAC** (Ch 24
§1.6 and §5 flagged this). TD3 and SAC's replay buffers see
every episode's transitions including post-first-completion
episodes, but `mean_reward` sees only first-completed
episodes. PR 2b's pre-loop + denominator fix adds
incomplete-env partials to `epoch_rewards`, which aligns
the denominator with `n_envs` but does not align the
numerator with the full set of episodes the replay buffer
trained on. The asymmetry persists and is not a PR 2
concern. A documentation comment on the rewritten
`mean_reward` block in `td3.rs` and `sac.rs` (six lines
total across the two files) names the asymmetry explicitly
so a reader of the post-PR-2b train loop sees the
disclosure. Writing that doc comment is a PR 2b drafting
detail.

**A Ch 31 §4.2 phrasing slip.** During Ch 41's recon pass,
the grep for `elite_mean_reward` returned exactly four hits,
all inside `cem.rs`: the write at `:191`, the insert at
`:219`, and the two unit tests at `:319` and `:354`. Ch 31
§4.2's prose at the relevant line range says "the only
readers outside `cem.rs` are CEM's own unit tests at
`cem.rs:319` and `cem.rs:354`" — which is a phrasing slip,
since both tests are inside `cem.rs` (in the `#[cfg(test)]
mod tests` block). The load-bearing claim Ch 31 §4.2 was
making — "no rematch reporting consumers exist" — is still
correct, so the phrasing slip does not change any of Ch 31's
conclusions. Ch 41 does **not** patch this slip. Unlike the
Ch 24 §2.2/§5 overclaim (which affects Ch 41's scope and is
bundled into the Ch 41 commit per the `b5cb3f6c` precedent),
the Ch 31 slip is phrasing-only and has no downstream
consequence that Ch 41 needs to correct for. Flagging it
here so a reader who notices the discrepancy in Ch 31 after
Ch 41 lands has the explanation.

**Post-commit patch to Ch 24 §1.9, §2.2, and §5.** The patch
is bundled into the Ch 41 commit per §3.3 and the `b5cb3f6c`
precedent. The bundle form is the Ch 24 source file
(`docs/studies/ml_chassis_refactor/src/24-result-semantics.md`)
and its review log
(`docs/studies/ml_chassis_refactor/review_logs/24-result-semantics.review.md`)
get narrow amendments in the same commit as the Ch 41
chapter and review log. The amendments correct §1.9's
"compares `best_reward()` values across algorithms" sentence,
§2.2's "The D2c rematch test compared them directly" sentence,
and §5's "the test is unit-broken against CEM versus any of
the other four algorithms" bullet — all three are replaced
with framings that name the actual locus of the unit
mismatch (the D2 SR findings memo and the human-read
eprintln output, not any assertion inside the test file).
No other Ch 24 claim changes; no Ch 24 file:line citation is
altered; neither Decision 1 nor Decision 2 is touched. The
amendment is narrower than the typical `3e1ec0ff` pattern
(which replaced two factually-wrong phrasings in Ch 21)
because the factual reading of Ch 24's load-bearing claims
stays intact — it is a scope correction, not a decision
revision.

**What Ch 41 does decide,** to close the chapter: the shape
of PR 2a (additive `run_replicates` + flat `Vec<RunResult>`
+ `replicate_index` + new aggregation helpers + new
`SeedSummary`, preserving the inline best-epoch scan at
`competition.rs:363-379`), the shape of PR 2b (CEM
`mean_reward` rewrite + TD3/SAC pre-loop + denominator
swap with `debug_assert_eq!` invariant + REINFORCE/PPO
zero-fallback skip-the-push + CEM `elite_mean_reward`
rename to `elite_mean_reward_per_step` + `d2c_cem_training.rs`
doc-only update), the 2-PR split structure, and the
free merge-order relationship with Ch 40's PR 1a/1b.
Part 4's execution layer continues here.
