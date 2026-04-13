# Remaining Work — Post-PR-3b

This appendix is the canonical "what's left" list after PR 3b
landed in session 17 (commits `2adaa372`, `3ad13c0e`,
`e33d4d57` on `feature/ml-chassis-study`, 49 commits ahead of
main as of 2026-04-13). PR 3b is the **final implementation
PR** in the study plan; everything below is post-implementation
work — running the test, writing it up, conditional follow-ups,
appendix cleanup, and future-PR-if-motivated items.

The list is meant to be read cold by a future contributor (or
the same contributor on a different day) without having to
re-read all of Part 4. Each workstream names its blocking
dependencies, its priority, the concrete commands or files
involved, and the cross-references back into the chapters that
own the deferral.

Five workstreams. Read the priority order first.

## Priority order at a glance

1. **[ ] Run the rematch.** ([§1](#1-run-the-rematch)) The
   linchpin — its outcome determines whether §3 fires at all,
   and §2 is blocked on its data. ~30-60 min unattended wall
   clock. Fire-and-forget; do this first.
2. **[ ] Author the rematch writeup.** ([§2](#2-author-the-rematch-writeup))
   Blocked on §1. Genre: a markdown doc rendering the bootstrap
   CI bounds, per-replicate table, bimodality coefficients,
   and classification verdict.
3. **[ ] Conditional Ch 30 null follow-ups.** ([§3](#3-conditional-ch-30-null-follow-ups))
   Only fires if §1's outcome is `Null` or `Ambiguous`. Skip
   entirely if `Positive`.
4. **[ ] Study book appendices.** ([§4](#4-study-book-appendices))
   Two scaffold files (`90-api-inventory.md`,
   `91-test-inventory.md`) need filling in. Doc-only,
   independent of source code, can land any session.
5. **[ ] Revisitable Ch 42 §9 deferrals.** ([§5](#5-revisitable-ch-42-9-deferrals))
   Future-PR-if-motivated items. Nothing to schedule now;
   these fire only when a downstream PR creates the
   motivation.

A `Positive` rematch outcome closes the study question and
collapses the post-PR-3b path to {§1, §2, §4, §5-as-motivated}.
A `Null` or `Ambiguous` outcome opens §3.

---

## §1 — Run the rematch

**Status:** not yet run as of session 17 close. Deferred from
session 17 by user choice (the test's correctness is reviewable
from compile-time + sub-decision (g)'s
"protocol-completes-cleanly" gate without needing to see a
successful outcome, and the wall clock is not session-budget
friendly during active iteration).

**Blocking dependencies:** none. Branch
`feature/ml-chassis-study` already carries PR 1a, PR 1b, PR 2a,
PR 2b, PR 3a, and both PR 3b commits. The test compiles cleanly
and registers as `ignored, requires --release (~30-60 min)`
under `cargo test -p sim-opt`.

**The invocation:**

```text
cargo test -p sim-opt --release --ignored d2c_sr_rematch -- --nocapture
```

The `--nocapture` flag is essential — without it Cargo's test
runner will swallow the eprintln verdict block and the rematch's
output will be lost. The test prints the bootstrap CI bounds,
the bimodality coefficients, the per-replicate reward vectors
for both CEM and SA, and the final `RematchOutcome`
discriminant via eprintln; capture *all* of stderr.

**Wall clock:** ~30-60 minutes for the unambiguous path (10
replicates per algorithm = 20 total), or ~60-120 minutes for
the ambiguous path (folded expansion to 20 replicates per
algorithm = 40 total). The test is single-threaded across
replicates because `Competition::run_replicates` runs them
sequentially; `n_envs = 32` parallelizes within each replicate
via `BatchSim`.

**Compute requirements:** any machine that can build sim-opt
in `--release` mode. No GPU. No special hardware. The 16M env
steps per replicate are CPU-bound on the SR task's tiny
1-DOF physics.

**Output capture:** redirect stderr to a file you can hand to
the §2 writeup author:

```text
cargo test -p sim-opt --release --ignored d2c_sr_rematch -- --nocapture 2> rematch-output.txt
```

Save `rematch-output.txt` somewhere durable (not just in
`target/`). The session-17 follow-up scope lists it as the
data source for §2.

**Reproducibility anchors** (already pre-registered in the
fixture source — see `sim/L0/opt/tests/d2c_sr_rematch.rs`):

- `REMATCH_MASTER_SEED = 20_260_412` (matches
  `d2c_cem_training.rs:69`'s `SEED_BASE`)
- `BOOTSTRAP_RNG_SEED = 0xB007_0057_00AA_0055` ("BOOTSTRAP
  AA55")
- `N_INITIAL = 10`, `N_EXPANDED = 20`
- Per-replicate seeds derived via
  `splitmix64(REMATCH_MASTER_SEED.wrapping_add(i))`

A future re-run of the same test on the same code at the same
tree state should produce byte-identical output (modulo
floating-point determinism caveats from BatchSim).

**Outcome semantics** (Ch 30 §"What each outcome would tell
us"):

- **Positive** — CI lower bound > 0. SA reliably outperforms
  CEM at matched complexity. The study question is answered;
  the rematch closes successfully. Skip §3 entirely.
- **Null** — CI upper bound ≤ 0, or CI straddles zero with
  point estimate ≤ 0. SA does not outperform CEM. Open §3
  for the pre-committed null follow-ups (richer SA proposal,
  Parallel Tempering).
- **Ambiguous** — CI straddles zero with point estimate > 0.
  Margin within seed-variance envelope; the folded-pilot
  protocol already auto-expands from N=10 to N=20 once
  inside `run_rematch`. If the post-expansion result is
  *still* Ambiguous, treat as Null for §3 follow-up
  purposes.

**Cross-references:**

- Ch 30 §"What each outcome would tell us" — outcome semantics
- Ch 32 §3.2 — bootstrap CI specification
- Ch 32 §6.3 — bimodality contingency
- Ch 42 §6.6 — fixture rendering
- Ch 42 §6.7 — sub-decision (g) "protocol-completes-cleanly"
  gate
- Ch 42 §9 — "The rematch has not yet been run." entry

---

## §2 — Author the rematch writeup

**Status:** deferred. Ch 32 §7 explicitly deferred the writeup
format to "a future writeup author." Ch 42 §6 sub-decision (j)
deferred it further to "a post-execution commit after someone
runs `cargo test --release --ignored d2c_sr_rematch`." Ch 42
§9 names it as a remaining artifact under "The rematch
writeup."

**Blocking dependencies:** §1 must complete and produce a
captured eprintln output.

**The artifact:** a markdown document rendering the rematch's
results. Genre: results writeup, not specification. The
writeup answers the question "what did the rematch find?"
with prose discussion grounded in the captured data.

**Suggested structure** (not prescriptive — the writeup
author has latitude here):

1. **Question and answer.** One paragraph naming the Ch 30
   question ("does SA convincingly outperform CEM at matched
   complexity on the D2c SR task?") and the rematch's
   one-sentence answer.
2. **Method recap.** One paragraph naming the protocol
   (folded-pilot, N_INITIAL=10, expansion to 20 if
   ambiguous), the matched-complexity anchor (`LinearPolicy(2,
   1)`, n_params=3), the bootstrap CI on difference of means
   (or medians via the bimodality contingency), and the
   reproducibility anchors. Cross-reference Ch 32 for the
   protocol detail rather than re-rendering it.
3. **Per-replicate data.** A table or two: one row per
   replicate × algorithm, columns `replicate_index | seed |
   best_per_episode_reward`. The data comes from the eprintln
   output. ~20 rows for the unambiguous path, ~40 for the
   ambiguous path.
4. **Bootstrap CI summary.** The point estimate, lower bound,
   upper bound, and `n_resamples` (10_000) for the
   `bootstrap_diff_means` (or `bootstrap_diff_medians` if the
   bimodality threshold tripped) result. The CI's
   classification (Positive / Null / Ambiguous).
5. **Bimodality coefficients.** The per-algorithm BC values
   and whether either crossed the 5/9 threshold. If yes, the
   CI shifts to medians per Ch 32 §6.3 — name the shift
   explicitly.
6. **Verdict.** The `RematchOutcome` enum value and a
   one-paragraph interpretation grounded in the data. If
   Positive, name the effect size in human terms ("SA's mean
   exceeds CEM's by X reward units, with the CI lower bound
   at Y, well above zero"). If Null, name the bound on the
   gap ("CI upper bound at Y, so any SA advantage is bounded
   above by Y"). If Ambiguous-after-expansion, name what
   would resolve it (more replicates, longer training, or one
   of the §3 follow-ups).
7. **Next steps.** Conditional on the verdict: link to §3 if
   Null/Ambiguous, link back to Ch 30 if Positive (the study
   question is closed), or both.

**Plot, optional.** A box-and-whisker plot or violin plot of
the per-replicate rewards for both algorithms is the obvious
visualization. Generate it separately (matplotlib, plotly,
gnuplot, whatever) and embed as an image. The plot is *not*
required — the table + verdict carries the load — but it
helps a future reader skim.

**Where the writeup lives:** the natural home is a new
`docs/studies/ml_chassis_refactor/results/` directory or a
new Part 5 in the study book (`SUMMARY.md` extension).
Either works. The writeup author picks. If Part 5, add a
`SUMMARY.md` entry; if `results/`, add a cross-reference from
this appendix's status checkbox.

**Cross-references:**

- Ch 30 §"What each outcome would tell us" — outcome semantics
- Ch 32 §3.2 — bootstrap CI shape
- Ch 32 §6.3 — bimodality contingency
- Ch 32 §7 — original writeup deferral
- Ch 42 §6 sub-decision (j) — Ch 42's writeup deferral
- Ch 42 §9 — "The rematch writeup" entry
- `sim/L0/opt/tests/d2c_sr_rematch.rs` — the fixture, includes
  the eprintln verdict block as the "data format" the writeup
  parses

---

## §3 — Conditional Ch 30 null follow-ups

**Status:** conditional. Fires only if §1's rematch produces a
`Null` outcome (or an `Ambiguous`-after-expansion outcome
that the writeup author treats as Null). Skip entirely if
the rematch is `Positive`.

**Blocking dependencies:** §1's outcome must be Null or
Ambiguous.

**Why these and not others:** Ch 30 explicitly pre-committed
to two follow-ups. The pre-commitment matters — Ch 30's
"failure modes that would need a different experiment" list
named these *before* the rematch ran, so a Null outcome
triggers them by design rather than by post-hoc rationalization.
A reader who is tempted to add a third follow-up after seeing
a Null result should re-read Ch 30 §3 first; the menu is
fixed for this study.

### §3.a — Richer-proposal SA variant

The current `Sa` implementation in
`sim/L0/opt/src/algorithm.rs` uses an isotropic Gaussian
proposal (`proposal_std` is a single scalar applied to every
parameter dimension) with no adaptation. A null result from
the basic SA could mean either (a) SA fundamentally cannot
outperform CEM at this task, or (b) the *basic* SA proposal
shape is too constrained and a richer proposal would do
better. Ch 30 names two rich-proposal candidates:

- **Adaptive `proposal_std`.** The proposal scale adapts to
  recent acceptance rate — if accept rate falls below some
  threshold, shrink `proposal_std`; if it rises above, grow.
  Common in Metropolis-Hastings literature as the
  Robbins-Monro adaptation. Adds ~30 lines to `Sa::train`.
- **Anisotropic proposal covariance.** The proposal becomes
  a multivariate Gaussian with a learned covariance matrix
  rather than `proposal_std * I`. The covariance can be
  estimated from the chain's history (empirical covariance
  of accepted states) or from the gradient of the fitness
  surface (which SA doesn't have, so the empirical-history
  approach is the right shape for a gradient-free algorithm).
  Adds ~80 lines plus a new `proposal_cov: DMatrix<f64>`
  field on `Sa`.

**Implementation shape:** add a new struct (e.g.,
`SaAdaptive` or `SaRich`) in
`sim/L0/opt/src/algorithm.rs` that wraps the existing `Sa`
shape with the adaptive logic, OR add the adaptation as a
configurable mode on the existing `Sa` struct (e.g., a
`SaProposalMode` enum field on `SaHyperparams`). The wrapper
shape is cleaner for keeping the basic-SA test fixture
untouched; the configurable-mode shape is cheaper at the
type-system level.

**Test fixture:** parallel to `d2c_sr_rematch.rs`, named
`d2c_sr_rematch_richer.rs`, identical structure but with the
SA builder constructing the rich-proposal variant instead of
basic Sa. Same matched-complexity gate, same
protocol-completes-cleanly assertion shape, same eprintln
verdict block. Reuses `make_training_vecenv`,
`rematch_task()`, the CEM builder, and all the constants
verbatim — duplicating the file is consistent with Ch 42 §6
sub-decision (f)'s scope-discipline duplication argument.

**Estimated scope:** 1-2 sessions. ~150 lines of richer-Sa
implementation + ~50 lines of unit tests + ~350 lines of test
fixture (mostly duplicated from `d2c_sr_rematch.rs`). One
PR, possibly two if the richer-Sa unit tests need their own
review cycle.

### §3.b — Parallel Tempering

Parallel Tempering (PT) runs multiple SA chains at different
temperatures simultaneously and periodically swaps states
between adjacent-temperature chains under a Metropolis
criterion. The high-temperature chains explore broadly; the
low-temperature chains exploit; the swaps let the
low-temperature chains "discover" basins the
high-temperature chains found. PT is a different algorithm,
not a richer proposal — its compute footprint is roughly K×
basic SA where K is the number of chains.

**Implementation shape:** new module
`sim/L0/opt/src/parallel_tempering.rs` with a
`ParallelTempering` struct implementing the `Algorithm`
trait. K parallel `Sa`-shaped chains stored as a `Vec<SaChain>`
internal type, each with its own temperature schedule. Per
epoch: each chain perturbs and accepts/rejects independently,
then a chain-pair swap pass attempts a Metropolis-criterion
exchange between adjacent chains. `policy_artifact` returns
the lowest-temperature chain's best params. Adds ~250 lines
plus its own unit tests.

**Compute parity caveat:** PT uses K× the per-epoch compute
of basic SA. To preserve compute parity with CEM (which is
what the matched-complexity rematch's framing requires), the
PT rematch would need to either (a) reduce the per-chain
budget by 1/K so the total compute matches CEM's, or (b)
budget K× the wall clock and acknowledge the comparison is
"PT at K× compute vs CEM at 1× compute." Ch 30's
pre-commitment doesn't specify which; the PT follow-up
author picks and defends it in the §2-style writeup.

**Test fixture:** parallel to `d2c_sr_rematch.rs`, named
`d2c_sr_rematch_pt.rs`, same shape with the SA builder
replaced by a `ParallelTempering` builder. Same
matched-complexity gate (PT's policy is still
`LinearPolicy(2, 1)` with n_params=3 — the K chains all
share the same parameter space).

**Estimated scope:** 2-3 sessions. PT is structurally more
complex than richer-proposal SA. ~250 lines of PT
implementation + ~80 lines of unit tests + ~350 lines of
test fixture. Probably its own PR.

### Bundling

Both follow-ups could ship as one PR or as two PRs. The
arguments mirror Ch 42's PR 3a / PR 3b split:

- **One PR:** atomic — both follow-ups are tested against the
  same Null result, both reuse the same analysis module,
  both report verdicts in the same writeup. The PR is
  larger (~900-1200 lines including both fixtures) but
  reviewable as one coherent "Ch 30 null follow-up" unit.
- **Two PRs:** richer-Sa first (smaller, lower-risk), PT
  second (larger, requires the PT-vs-CEM compute parity
  argument). Two review cycles, finer-grained rollback.

The lean is one PR if both follow-ups produce results
quickly; two PRs if the PT compute-parity argument needs
its own review cycle. The follow-up author picks based on
how Null the original rematch is (a "barely Null" result
might motivate richer-Sa first to see if a small adaptation
flips the verdict, deferring PT; a "decisively Null" result
might justify going straight to PT as the more aggressive
algorithm change).

### Skipping §3 entirely

If §1 produces `Positive`, the study question ("does SA
convincingly outperform CEM at matched complexity on D2c SR?")
is answered Yes, and §3 is unnecessary. Ch 30's null
follow-ups exist *to handle the No case*; a Yes case closes
the menu. The §2 writeup author renders the Positive verdict
and the appendix checkbox below stays unchecked permanently.

**Cross-references:**

- Ch 30 §3 — the pre-committed follow-up menu
- Ch 30 §"What each outcome would tell us" — outcome
  semantics
- Ch 31 §3.2 — three-outcome-all-informative framing
- Ch 42 §9 — "MLP or non-linear policy extensions" entry
  (related but different — that one is about policy
  complexity, not algorithm)
- `sim/L0/opt/src/algorithm.rs` — current `Sa` implementation
  the richer variant extends
- `sim/L0/opt/src/analysis.rs` — `run_rematch` driver both
  follow-ups reuse verbatim

---

## §4 — Study book appendices

**Status:** scaffolded but not filled in. Both files exist
in `docs/studies/ml_chassis_refactor/src/` and are listed in
`SUMMARY.md`'s Appendices section, but their content is
deferred from session 10's drafting backlog. After PR 3b, the
inventories need to cover the post-Ch-42 surface including
the new sim-opt crate and the new `TaskConfig::from_build_fn`
constructor.

**Blocking dependencies:** none. Doc-only, independent of
any source code execution. Can land any session, even before
§1.

### §4.a — `90-api-inventory.md`

**File:** `docs/studies/ml_chassis_refactor/src/90-api-inventory.md`

**Scope:** enumerate every `pub` type, function, method, and
constant the study touched across the three relevant L0
crates. Post-PR-3b that's:

- **`sim-ml-bridge`** — `Algorithm` trait + impls (Cem,
  Reinforce, Ppo, Sac, Td3), `Policy` / `StochasticPolicy`
  / `DifferentiablePolicy` traits + impls (LinearPolicy,
  LinearStochasticPolicy, MlpPolicy, AutogradPolicy, etc.),
  `ValueFn` / `QFunction` traits + impls, `OptimizerConfig`
  + `Optimizer` trait + impls (Adam), `TaskConfig` +
  `TaskConfigBuilder` + the new `TaskConfig::from_build_fn`
  constructor, `Competition` + `CompetitionResult` +
  `RunResult`, `VecEnv` + `VecEnvBuilder`, `SimEnv` +
  `SimEnvBuilder`, `ObservationSpace` + `ActionSpace`,
  `EpochMetrics` + `TrainingBudget` + `TrainingCheckpoint`,
  `PolicyArtifact` + `PolicyDescriptor`, `EnvError` +
  `ArtifactError`, the stock-task factories
  (`reaching_2dof`, `reaching_6dof`,
  `obstacle_reaching_6dof`), `BestTracker` (note:
  `pub(crate)`, see §5), helpers like `collect_episodic_
  rollout`.
- **`sim-opt`** — `Sa` + `SaHyperparams`, the analysis
  module's `BootstrapCi` + `RematchOutcome` +
  `bootstrap_diff_means` + `bootstrap_diff_medians` +
  `bimodality_coefficient` + `classify_outcome` +
  `run_rematch`, the four rematch constants
  (`REMATCH_MASTER_SEED`, `N_INITIAL`, `N_EXPANDED`,
  `REMATCH_TASK_NAME`).
- **`sim-thermostat`** — `LangevinThermostat` (4-arg
  post-PR-1b constructor), `DoubleWellPotential`,
  `OscillatingField`, `PassiveStack` +
  `PassiveStackBuilder`, the `prf::splitmix64` helper from
  PR 1a, the `PerEnvStack` trait + `EnvBatch<S>` generic
  from Ch 40.

**Format suggestion:** one section per crate, then a table
per section with columns `Item | Kind | Signature | Where
introduced`. The "Where introduced" column cross-references
the chapter that owns the item (e.g.,
`prf::splitmix64` → "Ch 40 PR 1a",
`TaskConfig::from_build_fn` → "Ch 42 §6.5 amendment, PR 3b
commit 1"). ~400-600 lines total, depending on how
comprehensive the enumeration is.

**Tooling shortcut:** `cargo doc -p sim-ml-bridge -p sim-opt
-p sim-thermostat --no-deps` generates HTML rustdoc that
enumerates every public item. The appendix can hand-curate
the list from the rustdoc output rather than re-discovering
it from grep.

### §4.b — `91-test-inventory.md`

**File:** `docs/studies/ml_chassis_refactor/src/91-test-inventory.md`

**Scope:** enumerate every test fixture across the three L0
crates the study touched. Post-PR-3b that's:

- **`sim-ml-bridge` inline unit tests** in `src/*.rs` —
  algorithm tests (cem, reinforce, ppo, sac, td3), policy
  tests (linear, mlp, autograd_policy, autograd_value),
  value/q-function tests, optimizer tests, task tests
  including the new `from_build_fn_*` tests, competition
  tests including `run_replicates_*`, vec_env tests,
  artifact tests, best_tracker tests.
- **`sim-ml-bridge` integration tests** in `tests/*.rs` —
  any `tests/competition.rs` or similar.
- **`sim-opt` inline unit tests** in `src/algorithm.rs`
  (4 SA tests) and `src/analysis.rs` (10 analysis tests).
- **`sim-opt` integration tests** in `tests/*.rs` —
  `d2c_sr_rematch.rs` (the rematch fixture, `#[ignore]`d).
- **`sim-thermostat` integration tests** in `tests/*.rs` —
  `d2a_stochastic_resonance_components.rs`,
  `d2b_stochastic_resonance_baselines.rs`,
  `d2c_cem_training.rs` (the legacy single-seed D2c CEM
  trainer the rematch parallels).

**Format suggestion:** one section per crate, then a table
per section with columns `Test name | Module/file |
Marker (#[ignore]?) | Purpose | ~Wall clock`. The wall-clock
column matters for `#[ignore]`d tests so a reader knows what
they're committing to before invoking. ~200-400 lines total.

### Bundling

Both appendices can land as one commit
(`docs(ml-chassis-study): fill in API and test inventory
appendices`) or as two commits (one per file). One commit is
slightly cleaner since they're both
"session-10-deferred-backlog cleanup" with the same
motivation. ~600-1000 lines combined. Doc-only, no source
code touched.

**Cross-references:**

- `SUMMARY.md` — both files already listed in the Appendices
  section
- `docs/studies/ml_chassis_refactor/src/01-how-this-study-is-produced.md`
  — names the session-10 backlog
- Ch 42 §9 — "Future Parts 5+ of the study book" entry
  mentions the appendices

---

## §5 — Revisitable Ch 42 §9 deferrals

**Status:** "future-PR if motivated." Each item in this
section is something Ch 42 §9 explicitly declined to address
because the motivation hadn't arrived yet. They sit in this
appendix as a reminder list, not a TODO list — none of them
should fire until a *separate* PR creates a concrete reason.
A future contributor reading this appendix should NOT
preemptively schedule any item in §5 just because it's
listed here.

The full list is at Ch 42 §9. The four items most likely to
become motivated by post-PR-3b work:

### §5.a — `Policy::clone_with_params` trait method

**Originating spec:** Ch 42 §4.2, Ch 42 §9 "The
`Policy::clone_with_params` trait method"

**What:** add a `clone_with_params(&self, params: &[f64]) ->
Box<dyn Policy>` method to the `Policy` trait in
`sim/L0/ml-bridge/src/policy.rs`. Returns a fully-owned
`Box<dyn Policy>` with the given parameter vector, leaving
`self` untouched.

**Why it's deferred:** SA as shipped doesn't need it. SA's
`policy_artifact` uses `from_policy(&*self.policy)` on the
always-synced policy, and `best_artifact` is constructed via
direct struct literal with `params = self.best_params.clone()`
— neither path needs to clone the policy itself.

**When it becomes motivated:** if/when a batch SA variant is
added that evaluates multiple candidate policies in parallel
without destructively mutating `self.policy`. A batch
evaluator needs to construct N policy clones each with
different params to run in parallel; without
`clone_with_params`, the only path is to round-trip through
`PolicyArtifact { params, descriptor, ... }` and
`artifact.to_policy()`, which works but is heavier than a
direct trait method.

**Estimated scope:** ~40 lines to add the trait method, ~200
lines across the impls (`LinearPolicy`,
`LinearStochasticPolicy`, `MlpPolicy`, `AutogradPolicy`,
etc.) plus their unit tests. One sim-ml-bridge PR.

### §5.b — `BestTracker::pub` visibility

**Originating spec:** Ch 42 §4.1, Ch 42 §9 "The
`BestTracker::pub` visibility"

**What:** change `sim/L0/ml-bridge/src/best_tracker.rs`'s
`BestTracker` struct from `pub(crate)` to `pub`, allowing
sim-opt and other downstream crates to import it directly
rather than duplicating the best-tracking logic.

**Why it's deferred:** `Sa` duplicates the best-tracking
logic inline (~15 lines across `Sa::new` and the train loop).
The duplication is small and `BestTracker`'s API may not be
the right shape for a shared public surface — sim-opt's SA
tracks scalar fitness, but a future multi-objective
optimization algorithm might track a Pareto front, which
`BestTracker` doesn't support.

**When it becomes motivated:** if/when a second consumer of
`BestTracker` surfaces in sim-opt or a third L0 crate, AND
that consumer's fitness shape is compatible with the existing
scalar `BestTracker` API. If the second consumer needs
multi-objective tracking, the right move is to design a
`BestTracker` v2 with a generic fitness type rather than
publish the v1 surface.

**Estimated scope:** ~5 lines to flip visibility, ~30 lines
to update sim-opt's `Sa` to consume the shared tracker, ~50
lines of integration test confirming the round-trip. One
sim-ml-bridge + sim-opt cross-crate PR.

### §5.c — Shared `EpisodicRollout::mean_reward_per_episode` helper

**Originating spec:** Ch 24 §1.5, Ch 41 §6, Ch 42 §9 "Ch 24
§1.5's shared rollout-aggregation helper"

**What:** extract the "sum epoch rewards across episodes,
divide by `n_envs`" idiom into a shared helper method on
`EpisodicRollout` (or a free function in `sim-ml-bridge::
rollout`). Currently five identical 5-line copies of the
same idiom across CEM, REINFORCE, PPO, SAC, and (as of
PR 3a) sim-opt's SA `evaluate_fitness`.

**Why it's deferred:** the duplication is small (5 lines × 5
sites = 25 lines) and the abstraction's correct shape isn't
obviously a method on `EpisodicRollout` vs a free function
vs an associated function on `Algorithm`. A premature
extraction that picks the wrong shape would force a second
refactor later.

**When it becomes motivated:** if/when a sixth consumer of
the same idiom appears, OR if/when a unit-related bug
surfaces in one of the five existing copies and the fix
needs to apply to all five simultaneously. Either trigger
makes the extraction's value obvious.

**Estimated scope:** ~30 lines for the helper + ~25 lines
of refactoring across the five call sites + adjusted unit
tests. One sim-ml-bridge + sim-opt cross-crate PR (sim-opt
must update simultaneously since SA is one of the five
consumers).

### §5.d — SA acceptance-rate runtime gate

**Originating spec:** Ch 23 §3.4, Ch 42 §9 "SA's
acceptance-rate collapse diagnostic"

**What:** add a runtime warning (or hard gate) to `Sa::train`
if cumulative acceptance rate drops below some threshold
(e.g., 10% over 20 epochs). The data is already there —
each epoch's `EpochMetrics::extra["accepted"]` carries the
0.0/1.0 acceptance flag — but `Sa::train` doesn't gate on
it.

**Why it's deferred:** the post-hoc analysis path already
exposes the data via the eprintln verdict block and the
serialized metrics; a runtime gate is a different kind of
guard (interrupting the run vs reporting after the run) and
its threshold value is arbitrary without empirical data on
what "collapsed" looks like in practice.

**When it becomes motivated:** if/when the rematch produces
a Null result that on inspection turns out to be caused by
acceptance-rate collapse rather than fundamental
SA-vs-CEM dynamics. The §2 writeup would name the collapse
as a finding; a follow-up PR adds the runtime gate.

**Estimated scope:** ~40 lines in `Sa::train` plus ~30 lines
of unit tests (constructing a fitness landscape that triggers
collapse and asserting the gate fires). Sim-opt-only PR.

### Other §9 items not enumerated above

Ch 42 §9 also lists: `d2c_cem_training.rs` retirement,
re-running the rematch under future chassis changes, the
warmup-overhead gate, `PerEnvStack` trait public surface
growth, data serialization for the rematch's output, the
rematch's effect-size threshold, MLP/non-linear policy
extensions, and the `BOOTSTRAP_RNG_SEED` literal's specific
value. None of these are likely to become motivated by
near-term post-PR-3b work; they're catalog entries for
future contributors who might find one of them relevant.

**Cross-references:** Ch 42 §9 (full list of deferrals).

---

## Status checkboxes (update as items complete)

- [ ] **§1** — Run the rematch (`cargo test -p sim-opt
  --release --ignored d2c_sr_rematch -- --nocapture`); save
  stderr to a durable file
- [ ] **§2** — Author the rematch writeup (blocked on §1)
- [ ] **§3** — Conditional null follow-ups (only if §1
  produces Null/Ambiguous):
  - [ ] **§3.a** — Richer-proposal SA variant
  - [ ] **§3.b** — Parallel Tempering
- [ ] **§4** — Fill in study book appendices:
  - [ ] **§4.a** — `90-api-inventory.md`
  - [ ] **§4.b** — `91-test-inventory.md`
- [ ] **§5** — Revisitable Ch 42 §9 deferrals (no scheduling;
  fires only when motivated by separate PRs):
  - [ ] **§5.a** — `Policy::clone_with_params`
  - [ ] **§5.b** — `BestTracker::pub` visibility
  - [ ] **§5.c** — Shared rollout-aggregation helper
  - [ ] **§5.d** — SA acceptance-rate runtime gate

When an item completes, replace the `[ ]` with `[x]` and add
a one-line note pointing at the commit or PR that closed it.
The appendix is meant to be edited in place over the
post-PR-3b lifetime of the study, ending the study book's
narrative arc when all checkboxes are either checked or
explicitly retired.

---

## Branch state at session 17 close

- **Branch:** `feature/ml-chassis-study`
- **Position:** 49 commits ahead of main
- **Working tree:** clean
- **Latest commits:**
  - `e33d4d57` — `docs(ml-chassis-study): Ch 42 PR 3b
    post-implementation audit`
  - `3ad13c0e` — `test(sim-opt): add D2c SR rematch fixture`
  - `2adaa372` — `feat(sim-ml-bridge): add
    TaskConfig::from_build_fn for custom seeded tasks`
  - `6a1f6764` — `docs(ml-chassis-study): Ch 42 PR 3a
    post-implementation audit`
  - `13a70c4c` — `feat(sim-opt): add pub-use re-exports at
    crate root`
  - (40 earlier commits across sessions 11-16)

The branch is in a clean, mergeable state. The user's session
17 plan is to push and PR what's done, then return to the
remaining work (this appendix) on a future day.
