# Failure modes

Chapter 30 set up the scientific question and named three meaningful
outcomes (positive, null, ambiguous). Chapters 22, 23, and 24
specified the rematch's compute-parity unit, pool, matched-complexity
anchor, per-replicate metric, and across-replicate aggregation
surface. Each of those chapters made decisions in service of an
honest answer to Ch 30's question, and each named gates,
pre-emptions, or downstream cleanups that have to hold for the
answer to land.

This chapter is the index. It enumerates the ways the rematch can
fail to deliver a clean answer to Ch 30's question, points each
failure mode at the chapter that specifies its protective mechanism,
and gives a reader who is about to act on the rematch's result a
single place to scan before forming a conclusion. Chapter 31 makes
no new design calls. Every failure mode it names inherits from a
committed chapter, and every protective mechanism it points to
already exists somewhere in the book.

A note on what "failure" means here. The rematch can produce a
correct number that nevertheless does not deliver the answer Ch 30
asked for. The number can be read incorrectly by the audience. The
writeup that carries it can omit context the reader needs. The
number's magnitude can land within an irreducible noise floor and
mean less than it looks like. The implementation that produced the
number can have deviated from the protocol the design chapters
specified. Each of these is a distinct failure shape with a distinct
protective mechanism, and they are easier to enumerate when they
are kept apart than when they are folded into a single "things
that could go wrong" list. Ch 31 organizes them into four buckets
along the rematch's pipeline, traced backwards from the reader to
the input.

The four buckets are reader interpretation failures (correct
numbers, wrong conclusion), reporting discipline failures (correct
numbers, missing context), effect-size-vs-noise failures (the
headline number lands within an irreducible noise floor), and
protocol or prerequisite failures (the rematch as actually executed
deviates from the rematch as specified, or runs before its
prerequisites land). The order reflects how close the failure is
to the published result: bucket 1 is the latest stage, bucket 4 is
the earliest. The protective mechanisms run in the opposite
direction — fixing a bucket-4 vulnerability is cheaper before it
becomes a bucket-1 misreading.

## Section 1 — Reader interpretation failures

The rematch can produce a number that is correct in every internal
sense — the chassis is reproducible, the pool is honest, the metric
is uniform across algorithms, the writeup discloses everything Ch
22 and Ch 24 require — and a reader can still read the number into
a conclusion the rematch does not support. The protective mechanism
for this class is pre-emptive language: explicit text in the
rematch writeup that names what the result is and is not evidence
for. Ch 30 supplies the language for two of the three entries
below; Ch 22 supplies the third.

### 1.1 Single-task generalization

The rematch is a single experiment on the stochastic-resonance
task at one matched-complexity setting. A positive result —
SA resolves the SR peak more reliably than the matched-complexity
RL baseline — is evidence that geometry-appropriate updates beat
generic RL *on this particular task, at this particular matched
complexity, with this particular physics-aware method*. It is not
evidence that custom methods are better than RL in general, or
that SA is better than RL in general, or that the physics-aware
ML pivot's strategic claim is vindicated. A reader who reads a
single positive data point as a proof of the general principle is
making the seed-partition inflation error Ch 20 established, in
the form Ch 30 labels as SOTA-by-partition inflation.

The protective mechanism is in Ch 30, not Ch 31. Ch 30's "What
each outcome would tell us" section already has the pre-emptive
language for the positive case: a single positive result "is
evidence for the pivot's strategic claim, not vindication of it
— the pivot's strategic claim is a generalization across many
tasks and methods, and one task with one method can at most be a
compatible data point. Read larger than that, a single positive
result would be exactly the kind of SOTA-by-partition inflation
chapter 20 warns about." The rematch writeup carries this language
forward verbatim or in a tightly paraphrased form. Ch 31's
contribution is to name the misreading as a failure mode the
writeup must pre-empt, not to re-derive Ch 30's language.

The failure mode is realizable because one-data-point
generalization is a *common* misreading of any benchmark result.
A reader who skims the rematch writeup and extracts the headline
"SA beats CEM at SR" is a plausible reader, not an exotic one,
and pre-empting a plausible misreading is what the writeup is
for.

### 1.2 The rematch as an algorithm-selection guide

The rematch's compute-parity unit is env steps, picked in Ch 22
on the grounds that wall-clock is machine-dependent and
gradient-update count is non-uniform across algorithm families.
The decision is correct for what the rematch is asking — "which
algorithm learns best per env step" — and wrong for a question
the rematch is not asking but a reader might mistake it for
asking — "which algorithm should a practitioner choose for a real
workload."

The two questions diverge most visibly on PPO's `k_passes` (PPO
performs `k_passes` SGD updates per epoch over a frozen rollout,
which costs wall-clock time but does not advance the env-step
counter), but the divergence is more general than PPO. SA's
inner-loop cost per accepted move, CEM's elite-selection sort
cost, and TD3/SAC's per-step actor-critic gradient updates are
all wall-clock costs the rematch does not measure. A reader who
runs the rematch's headline curves and concludes "SA is faster
than CEM in practice" is reading a question into the curves that
the curves are not constructed to answer.

The protective mechanism is in Ch 22 §`PPO's k_passes`. Ch 22
already specifies that the rematch writeup mentions `k_passes`
exactly once in a short note that says "PPO runs `k_passes` SGD
passes per epoch, which costs wall-clock time but does not
advance the env-step counter; the rematch does not adjust for
this," and that the rematch's silence on wall-clock generally is
"a property of the design, not an oversight." Ch 31 generalizes
the failure mode beyond PPO: any reader who interprets the
rematch's headline ranking as a practitioner's algorithm-choice
guide is doing what Ch 22 named pre-emptively. The writeup's
pre-emption text is the generalized version of Ch 22's PPO note,
not the PPO-specific version: "the rematch answers 'which
algorithm learns best per env step,' not 'which algorithm a
practitioner should choose for a real workload'."

Ch 22 flagged this failure mode for Ch 31 in its closing
scope-discipline section: "the rematch's silence on wall-clock
is a property of the design, and chapter 31 may need to name it
in the failure-mode enumeration so a reader who misuses the
rematch as an algorithm-selection guide has a pre-emptive
counter." The hand-off is soft on Ch 22's side ("may need to
name") and Ch 31 takes the option, broadening the pre-emption
beyond Ch 22's PPO-specific `k_passes` framing. Section 5 flags
the broadening as the only piece of §1.2 that is Ch 31's
framing rather than direct inheritance.

### 1.3 Null as physics-aware-ML class rejection

A null outcome — SA finds the elevated region but cannot resolve
the peak any more reliably than the matched-complexity RL
baseline — is evidence that *this specific Gaussian-proposal
random-walk SA, at this matched complexity, on this landscape* is
not geometry-appropriate enough to beat generic RL. It is not
evidence that physics-aware ML as a class is wrong, that
geometry-appropriate updates as a class do not beat generic RL,
or that the pivot's strategic claim has been refuted. A reader
who reads a null result as a class rejection is making the
inverse of the single-task generalization error in §1.1: treating
one negative data point as a proof against the general principle.

The protective mechanism is, again, in Ch 30. Ch 30's null
outcome pre-emption explicitly bounds the inference: "A null
updates against *that specific hypothesis*, not against
physics-aware ML as a class — but it does put the next experiment
explicitly in charge of carrying the pivot's strategic claim,
which means the claim is on a clock rather than on a concept."
Ch 30 also names the allowed follow-ups (richer SA proposal
structure or Parallel Tempering on the same SR task) and
explicitly excludes "pick a different benchmark until one works"
as a follow-up move. The rematch writeup carries this language
forward.

The reason this gets its own subsection rather than being folded
into §1.1 is that the protective mechanisms for the two
misreadings are different. The single-task-generalization
pre-emption is a one-line bound on what the result *means*. The
null-as-class-rejection pre-emption is a list of what counts as a
legitimate next experiment, written *before* the result is in,
specifically so that a "we'll pivot the experiment" response after
a null cannot look like motivated reasoning. The two misreadings
are symmetric in shape but require asymmetric writeup discipline,
and Ch 31 keeps them separate to make that discipline visible.

## Section 2 — Reporting discipline failures

The rematch can produce correct numbers and a careful reader can
still draw a wrong conclusion if the writeup omits context the
numbers depend on. The protective mechanism for this class is
writeup discipline: explicit fields, explicit labels, explicit
units, in the rematch's published tables and figures. The two
failure modes below are the load-bearing ones; both inherit from
chapters that named the requirement but delegated the writeup-level
guard to Ch 31.

### 2.1 Per-family actual env step counts

The rematch's compute-parity decision in Ch 22 picks env steps as
the budget unit and `Steps(16M)` as the value, with all five
algorithms running 100 epochs. The recon Ch 22 carried out also
established that *at the same `Steps(N)` budget, on-policy and
off-policy do genuinely different amounts of actual env work per
epoch*. On-policy stops a rollout at `done`; off-policy steps all
`n_envs` envs unconditionally every inner-loop iteration,
including completed envs that have been auto-reset, until the
slowest env has finished one episode. At `n_envs = 32` and
typical episode lengths around `2000–3000` steps, the per-epoch
env-call counts diverge by tens of thousands of calls between the
families, and at 100 epochs the divergence is in the millions.
Both counters report honestly. The work itself differs.

Ch 22 picked option (b) — document the disparity, do not fix it
in the algorithm surface — for the rematch, on the grounds that
option (a′) (gating off-policy's inner loop on `!env_complete[i]`)
is an algorithm-design change out of scope for the study. Ch 22
explicitly named the risk: "if the writeup reports only the
nominal Steps budget and omits the per-family actual step counts,
the comparison silently mis-represents the compute each algorithm
used." That risk is the failure mode Ch 31 enumerates here.

The protective mechanism is a writeup-level guard: every
rematch result table includes a per-family
`total_env_step_calls` column alongside the `Steps(16M)` budget
column. Neither family's actual `env.step()` count equals the
nominal budget — the nominal value is a budget-formula input
that determines `n_epochs`, not a predicted call count. The
formula `n_epochs = N / (n_envs * max_episode_steps).max(1)`
uses `max_episode_steps = 5000` as the per-trajectory maximum,
but on-policy rollouts stop at `done` when typical episodes
terminate well below the maximum, so on-policy actuals come in
substantially below nominal. Off-policy actuals also come in
below nominal but by a smaller factor, because post-completion
stepping pads the count past on-policy's `done`-stop point. The
disparity the column guards against is the gap between the two
families' actual counts (Ch 22's example figures: on-policy
~6.4M, off-policy ~9.6M, against a nominal `Steps(16M)`), not a
gap between either family and the nominal. A reader who sees
both actuals can form their own judgment about whether the
comparison is fair under their preferred interpretation of
"fair." A reader who sees only `Steps(16M)` cannot.

The writeup-level guard is the rematch's bridge between Ch 22's
"document and live with it" decision and Ch 30's standard for an
honest answer. Without the guard, the decision quietly slides
from "live with a visible disparity" to "live with a hidden
disparity," and the rematch loses the property that makes its
under-fair-compute claim defensible.

The guard is nontrivial because two pieces of the implementation
have to cooperate. First, the rematch's reporting code has to
expose the per-family `total_steps` value in its output structure,
not roll it up into a single uniform field. Ch 24's
`replicate_best_rewards(task, algo) -> Vec<f64>` primitive
pulls best-reward per replicate; an analogous primitive
(`replicate_total_steps(task, algo) -> Vec<u64>`, or simply
reading `result.runs[i].metrics.last().total_steps` directly) is
needed for the per-family step counts to land in the writeup's
tables. Second, the writeup has to actually display them — a
table that *could* show the numbers but in practice gets
formatted into a single budget column is the same failure as
omitting the field entirely. The guard is "the writeup table
displays per-family actual step counts as a separate column from
the nominal budget," not "the data structure can produce them on
demand."

Ch 31's contribution to this failure mode is naming the guard
explicitly so the rematch writeup has a checkable acceptance
criterion. Ch 22 named the requirement; Ch 31 names what the
requirement looks like in the published artifact.

### 2.2 Spread metric labeling

Ch 24 §4.6 picked sample standard deviation (Bessel-corrected, n−1
denominator) as the spread metric for `SeedSummary`. The choice
is convention-matching: sample std is what almost every empirical
RL paper reports as "± X" alongside a mean, and matching the
convention reduces the friction of comparing the rematch's
numbers to other published baselines. Ch 24 considered and rejected
standard error of the mean (`std / sqrt(n)`) by omission from
`SeedSummary` at §4.2 and §4.3 (Option M deliberately ships
with `n`, `mean`, and `std_dev` only — no stderr field), and
deferred bootstrap confidence intervals to a future chapter at
§5, both on scope grounds.

The decision is sound. The failure mode is the writeup's
*labeling* of the spread, not the spread itself. A rematch table
that reports "SA: 0.087 ± 0.04" without specifying what `± 0.04`
is leaves the reader to guess between sample std, SEM, and a
confidence interval. The three quantities differ by a factor of
roughly `√n` between sample std and SEM at small `n`, and by
larger factors for confidence intervals depending on the chosen
coverage. At the replicate counts Ch 24 §4.6 projects (`n = 3` to `n = 10`),
`√n` is 1.7 to 3.2, which is the difference between "the means
overlap substantially" and "the means are well-separated" for
any plausible rematch effect size. A reader who reads the spread as SEM when it
is sample std concludes the rematch's mean is √n times more
reliably resolved than it actually is — a substantive misreading,
not a labeling pedantry.

The protective mechanism is a writeup discipline: every place the
rematch reports a `mean ± spread` value, the spread's identity is
named in the same line, the table caption, or a footnote. The
text the rematch writeup uses is something like "values are
mean over `n` replicates ± sample standard deviation
(Bessel-corrected); sample std is reported rather than standard
error of the mean for convention-matching reasons (Ch 24 §4.6)."
The exact wording is a writeup choice; the requirement is that the
disambiguation is present and visible at every reporting site.

§2.1 and §2.2 have different realization shapes. §2.1 is a sin
of omission whose absence is checkable structurally — a
per-family-step-count column is present in the result tables or
it is not, and a single review of the published artifact
catches the omission. §2.2 is a sin of brevity whose detection
requires checking every reporting site for an identifier on the
spread value: every place the rematch reports a `mean ±
spread`, the discipline has to hold separately, because each
reporting site is a fresh opportunity to omit the label in the
name of compactness. Both earn their own guards.

The two-entry bucket. §2.1 and §2.2 are the only entries in
bucket 2 that earn their own subsections. Two adjacent items
that initially looked like bucket 2 candidates are actually
bucket 4 items, and they are enumerated there: the latent
REINFORCE/PPO zero-fallback that could cause `SeedSummary.n` to
shrink silently (Ch 24 §4.7 contingency on Ch 41) is a
prerequisite-failure shape because the protective mechanism is
"Ch 41 lands the fix," not "the writeup adds a column"; and the
CEM `extra.elite_mean_reward` unit-mismatch flag from Ch 24 §5
is a CEM-internal scaffolding item (recon-verified: only readers
outside `cem.rs` are CEM's own unit tests, no rematch reporting
code reads it) that does not surface to a rematch reader and
does not earn its own Ch 31 entry. Both belong to Ch 41's
execution-PR cleanup scope; see §4.2 for the inheritance
bookkeeping line item.

## Section 3 — Effect-size-vs-noise failures

The rematch can run cleanly under a chassis that is reproducible,
a pool that is honest, a metric that is uniform, and a writeup
that discloses everything Sections 1 and 2 require, and the
headline number can still mean less than it looks like if its
magnitude is within an irreducible noise floor. There are two
such floors. Ch 22 establishes the first; Ch 30 establishes the
second.

### 3.1 The warmup-overhead gate

Ch 22 picked "document and live with" the TD3/SAC warmup
overhead, which is `n_envs * warmup_steps = 32 * 1000 = 32{,}000`
env steps of random exploration before any gradient update — 0.2
percent of the 16M rematch budget at the D2c configuration. The
decision is conditional, and Ch 22 was explicit about the
condition: "the case against deducting" holds *only if* the
rematch's effect size is large relative to 0.2 percent. If the
rematch's reward curves are separated by 3 percent, the 0.2
percent handicap is one-fifteenth of the effect and the "live
with it" call is defensible. If the curves are separated by 0.5
percent, the 0.2 percent overhead is 40 percent of the effect
and the call is not defensible — at that point the rematch's
headline claim is within a warmup's distance of zero, and the
warmup needs to be either deducted from the budget or reported
as a separate field before the rematch can be trusted.

This is the failure mode Ch 22 explicitly delegated to Ch 31:
"chapter 31's gate is what makes this conditional explicit: the
gate has to bound the rematch's effect size away from the warmup
overhead, and if the gate cannot be met the decision returns
here and the fix stops being out of scope."

The gate has the following shape. After the rematch produces its
headline result — the difference in mean best reward between SA
and the matched-complexity RL baseline (CEM, after Ch 23's
pool-membership decision excludes TD3/SAC/PPO) — the writeup
checks whether the magnitude of that difference exceeds 0.2
percent of the budget-equivalent reward scale by some
predetermined factor. Ch 31 deliberately does not specify the
factor (Ch 32 owns that, alongside the statistical test choice
and the replicate count). What Ch 31 specifies is that the gate
*exists*, that the rematch writeup applies it before publishing
the headline conclusion, and that a gate failure triggers a
return to Ch 22's option set rather than a quiet acknowledgment
in a footnote.

There is a subtlety worth naming here: the warmup overhead
applies to TD3 and SAC, both of which are excluded from the
rematch pool by Ch 23. After Ch 23, the rematch pool is `{CEM,
SA}` and neither has a warmup phase. So the warmup-overhead gate
is, strictly, *retroactive risk pre-emption* — it asks "what if
the pool had included TD3 or SAC?" and the answer is "the
rematch's headline would have been within the warmup floor." The
gate as a live constraint on the post-Ch-23 rematch is moot.

But the failure mode does not vanish, because the gate's purpose
is broader than its TD3/SAC instantiation. The pattern Ch 22
named — "the rematch's headline lands within a small unbudgeted
overhead the design chose to live with" — generalizes to any
future pool addition that introduces a similar overhead, and to
any future budget choice that makes 32,000 steps a larger
fraction of the total. The gate's *number* (0.2 percent at the
16M D2c budget) is specific to the current rematch
configuration; the gate's *shape* (effect size must exceed
unbudgeted-overhead floor by a predetermined factor before the
"live with it" call survives) is the protective mechanism Ch 31
inherits from Ch 22 and passes forward to Ch 32. Ch 31 names the
gate as a failure mode of the rematch's design pattern, not of
its current instantiation.

A reader who is only running the rematch in its current `{CEM,
SA}` form can skip §3.1's check in practice. A reader who is
extending the rematch — adding back PPO, adding a third
physics-aware method, changing the budget — has to apply the
gate before the extended rematch can be read as cleanly as the
current one.

### 3.2 The seed-variance envelope

Ch 30's three outcomes include an "ambiguous" outcome: "SA beats
RL in expectation, but the margin is within the seed-variance
envelope — a plausible false positive from a favorable seed
partition, exactly the kind of result Henderson warns about." The
ambiguous outcome is the second irreducible noise floor: even at
a perfectly executed rematch with all chassis fixes in place and
all metrics uniform, the difference between SA's mean replicate
reward and CEM's mean replicate reward can be within the spread
of either distribution, and a reader who treats a within-envelope
difference as a positive outcome is making the same partition-
inflation error Ch 20 warned about at a smaller scale.

The protective mechanism is the across-replicate aggregation
surface from Ch 24 (`SeedSummary { n, mean, std_dev }` and
`replicate_best_rewards(task, algo) -> Vec<f64>` for downstream
analysis) combined with the statistical-test decision Ch 32 will
make. Ch 31 does not specify the test; Ch 32 owns that. What Ch
31 names is that the failure mode is realizable, that the
rematch writeup has to apply the test before claiming a positive
or null outcome, and that an ambiguous result is not a failure
of the experiment but a signal to run the next one differently
— Ch 30's framing carried forward.

The gate's shape: after the test produces a p-value, confidence
interval, or whatever Ch 32 chooses, the writeup classifies the
result into one of Ch 30's three buckets (positive, null,
ambiguous) explicitly. A "weak positive" or "near-null" hedge is
not on the menu — Ch 30 deliberately specified three outcomes
because each one prescribes a different next experiment, and a
hedged result short-circuits that prescription. Ch 31 names the
gate as a writeup discipline: the rematch's headline conclusion
is one of the three Ch 30 outcomes, and hedged language is
dispatched explicitly to the ambiguous bucket rather than
offered as a fourth option.

The relationship between §3.1 and §3.2. The warmup-overhead
floor is a constant (currently moot, by §3.1's note); the
seed-variance envelope is a function of the chosen replicate
count `n` and the per-replicate variance the pilot will measure.
The two floors interact: a rematch effect size that exceeds the
warmup floor but not the seed-variance envelope is in the
ambiguous bucket, and a rematch effect size that exceeds both is
in the positive bucket. Ch 31 enumerates them as separate
failure modes because their protective mechanisms are different
chapters' work — Ch 22 owns the first, Ch 32 owns the second —
and folding them would obscure the inheritance.

## Section 4 — Protocol and prerequisite failures

The rematch can be specified perfectly and read perfectly and
still fail to deliver the answer Ch 30 asked for if the
implementation that produced its numbers deviated from the
protocol the design chapters specified, or if a prerequisite
chapter or PR has not landed before the rematch runs. The
protective mechanism for this class is upstream PR ordering and
spec fidelity: the chapters and PRs in this section must land
in the order they imply, and the SA implementation in Ch 42 must
match the matched-complexity anchor Ch 23 specified.

Bucket 4 entries are paired with the failure shape they cause if
the prerequisite is missed. The shape is named in parentheses
after each subsection title, so the bucket-4 → bucket-1/2/3
shadow relationship is visible at the index level.

### 4.1 Chassis reproducibility (latent flakiness shape)

Ch 13 named the latent flakiness in `LangevinThermostat`'s
shared-mutex hosting. Ch 14 laid out the design grid; Ch 15
picked C-3 (per-env hosting via `install_per_env`, counter-based
`prf.rs` mechanism). Ch 40 (Part 4 PR 1) is the execution PR
that lands the chassis fix. The 14 D1–D14 implementation
decisions locked in Ch 15's post-commit conversation are the
concrete calls, and the 2-PR split (D14: PR 1a additive
`prf.rs` module, PR 1b LangevinThermostat rewrite +
`BatchSim::new` rewire) is the landing strategy.

The failure mode if Ch 40 has not landed before the rematch
runs: the rematch's "same seed produces the same result" property
silently does not hold under the `parallel` feature, and the
rematch's replicates produce values that differ across runs of
the same configuration for reasons unrelated to the algorithm
under test. A reader who runs the rematch twice and sees
different headline numbers cannot tell whether the difference
is a real signal (different seeds, different replicate samples)
or a chassis bug (parallel-feature flakiness). The failure shape
shows up as bucket 3 (effect-size-vs-noise) — the noise floor is
inflated by chassis nondeterminism — but the *cause* is
upstream, and the protective mechanism is "Ch 40 lands first,"
not "the rematch tolerates a higher noise floor."

There is a subtlety that softens this failure mode in practice
under current configurations. Key finding 1 in the Ch 13 phase-2
recon establishes that `sim/L0/core/Cargo.toml:37` declares
`default = []` for the `parallel` feature and that no `Cargo.toml`
under `sim/` opts in via `features = ["parallel"]`. Under a plain
`cargo test` or `cargo run`, the `par_iter_mut` branch is dead
code and the latent flakiness is unreachable. So a rematch run
under default features is *currently* reproducible without Ch 40
having landed, and the protective mechanism is, in practice,
"do not enable `--features parallel` for the rematch run until
Ch 40 has landed." This is a thinner guard than "Ch 40 has
landed," but it is a real guard, and a rematch that uses default
features can in principle run before Ch 40 if the writeup is
explicit about the feature flag.

The honest framing is: Ch 40 is the *correct* protective
mechanism (it eliminates the latent defect by construction). The
"do not enable `--features parallel`" workaround is a *temporary*
protective mechanism that holds only as long as no one in the
rematch's call path opts in to the feature. A rematch writeup
that uses the workaround discloses it; a rematch run after Ch 40
does not need to.

### 4.2 Algorithm-surface metric fix (apples-to-oranges shape)

Ch 24 picked Decision 1: standardize every algorithm's
`EpochMetrics::mean_reward` on "mean per-episode total reward
across `n_envs` trajectories." The fix touches three algorithms
(CEM at `cem.rs:209`, TD3 at `td3.rs:490` plus a three-line
pre-loop, SAC at `sac.rs:543` plus a three-line pre-loop;
REINFORCE and PPO are no-ops). CEM keeps its length-normalized
fitness at `cem.rs:183` for elite selection on scope grounds —
an internal split that Ch 24 §3.5 documented as a real cost. Ch
41 (Part 4 PR 2) is the execution PR that lands the metric fix
along with the across-replicate aggregation surface (Ch 24
Decision 2: `replicate_best_rewards`, `describe`, `SeedSummary`).

The failure mode if Ch 41 has not landed before the rematch
runs: every algorithm in the rematch reports `mean_reward` in
its current pre-fix units, and `RunResult::best_reward()`
returns a max over apples-to-oranges values across CEM and SA.
The current pre-fix units are spelled out in Ch 24 §1: CEM's
`mean_reward` is per-step-normalized (each trajectory's total
reward divided by trajectory length, then averaged across
`n_envs` trajectories), and the four other algorithms agree on
per-episode total at the numerator with denominator differences. SA does not currently exist as an
algorithm in `sim/L0/ml-bridge/src/`, so the pre-fix unit
question for SA is conditional on what Ch 42 decides for its
metric reporting — but the safe assumption is that SA matches
whichever convention Ch 41 establishes as the standard, which
means a pre-Ch-41 rematch would compare CEM-per-step against
SA-something-else, and the comparison is structurally broken
before any seed runs.

The failure shape shows up as bucket 1 (reader interpretation)
— a reader sees a `best_reward` table and interprets it as
algorithm A vs algorithm B comparison when it is actually unit A
vs unit B comparison — but the cause is upstream, and the
protective mechanism is "Ch 41 lands first." The rematch cannot
run honestly until the algorithm surface speaks one language.

This subsection also carries the forward-pointer that Ch 31's
recon collapsed §2.2's old elite_mean_reward candidate into. Ch
24 §5 flagged that CEM's `extra.elite_mean_reward` diagnostics
key sits at `cem.rs:219` in per-step units while CEM's
`mean_reward` post-Decision-1 is in per-episode-total units — a
naming collision a reader who pokes into `EpochMetrics.extra`
will reasonably ask about. Recon for Ch 31 (grep across the
codebase for `elite_mean_reward`) confirmed that the only
readers outside `cem.rs` are CEM's own unit tests at `cem.rs:319`
and `cem.rs:354`, both of which assert presence and finiteness
without comparing the value to anything. No rematch reporting
code reads the key. The unit mismatch is therefore a
CEM-internal cleanup item that Ch 41 picks up alongside the
Decision 1 train-loop rewrite, not a rematch failure mode the
writeup needs to guard against. Ch 31 names it here for
inheritance bookkeeping; the protective mechanism is the same
"Ch 41 rename-or-doc" follow-up Ch 24 §5 already specified
(Ch 24 leaves the choice between renaming the key outright and
updating the doc comment side-by-side as a Ch 41 cleanup call).

### 4.3 SA parameterization drift (matched-complexity shape)

Ch 23 §3 picked the operational definition of "matched
complexity" as the D2c CEM baseline anchor: SA's policy class is
`LinearPolicy(2, 1)` with `n_params = 3`, the same parameterization
the D2c CEM run used. The choice is enforceable as a one-line
equality assertion in the rematch test gate. Ch 42 (Part 4 PR 3)
is the execution PR that lands SA in `sim-opt` and wires it into
`Competition`.

The failure mode if Ch 42 lands SA with a parameterization that
deviates from the matched-complexity anchor: a positive rematch
result is uninterpretable. Ch 30's geometry-vs-expressiveness
distinction — the load-bearing distinction the whole rematch
rests on — collapses. A SA that uses a richer policy class than
CEM did in D2c could be winning because its representation has
more expressiveness, not because its updates are
geometry-appropriate. A reader who sees "SA: 0.087, CEM: 0.072"
and interprets it as evidence for the pivot's strategic claim
is being misled by an expressiveness gap masquerading as a
geometry gap, and the misleading is structural — no amount of
seed averaging or statistical-test rigor can recover the
distinction once the parameterization has drifted.

The failure shape shows up as bucket 1 (reader interpretation)
again, but the protective mechanism is neither writeup discipline
nor pre-emptive language. The protective mechanism is *Ch 42's
acceptance criterion for the SA implementation*: SA must
instantiate its policy as `LinearPolicy(2, 1)` with `n_params =
3`, the rematch test gate enforces the equality with a one-line
assertion (Ch 23 §3's recommended form), and a Ch 42 PR that
lands SA without the assertion fails its own gate. The reason
this failure mode lives in Ch 31 rather than only in Ch 42 is
that Ch 31 is the index a reader scans before acting on the
rematch's result. A reader who has not read Ch 42 — and most
readers have not read Ch 42 — needs to know that the
matched-complexity anchor exists, that it is enforceable, and
that a rematch result without the enforcement is uninterpretable
in a specific direction. Ch 31 names the vulnerability so the
reader can check whether it has been addressed; Ch 42 does the
addressing.

This is the failure mode the recon-to-leans conversation
specifically pressure-tested for "is it duplicative with Ch 42?"
The settling argument is internal consistency with §4.1 and
§4.2: those entries also have their fix in a downstream PR
chapter (Ch 40 and Ch 41 respectively), and Ch 31 enumerates
them anyway because Ch 31 is the single index, not because Ch
40 and Ch 41 are insufficient. Section 4.3 is the same shape and
earns the same enumeration.

### 4.4 Pilot existence (statistical-test shape)

Ch 32 owns three things Ch 24 explicitly deferred: the
statistical test the rematch uses to classify its outcome into
one of Ch 30's three buckets, the replicate count `N`, and the
pilot design itself. The replicate count depends on measured
per-replicate variance, which depends on running a pilot. The
statistical test depends on the distributional shape of the
replicate values, which also depends on pilot data.

The failure mode if Ch 32 has not landed before the rematch
runs: the rematch picks an `N` and a test ad hoc, the picked `N`
turns out to be too small to detect the rematch's actual effect
size at the chosen significance level, and the rematch reports
an ambiguous outcome when a larger `N` would have produced a
positive or null. The ambiguous outcome is a real Ch 30 outcome,
not a failure — Ch 30 explicitly names it as informative — but
*ambiguous because of insufficient power* is different from
*ambiguous because the effect is genuinely within the noise
envelope*, and the rematch writeup loses the ability to
distinguish them if Ch 32's pilot work has not been done.

The failure shape shows up as bucket 3 (effect-size-vs-noise),
specifically §3.2's seed-variance envelope. The protective
mechanism is a two-part conjunction: "Ch 32 has landed" AND
"the rematch protocol implements the initial-batch read with
its pre-registered escalation rule." Ch 32's landing means the
test has been picked with eyes open (bootstrap CI on the
difference of means, per Ch 32's Decision 1), the significance
threshold has been chosen (95 percent CI with the three-bucket
Ch 30 mapping, per Ch 32's Decision 2), the replicate count
protocol has been specified (`N = 10` initial with a
pre-registered single expansion to `N = 20` iff the initial-
batch CI is ambiguous, per Ch 32's Decision 3), the initial-
batch composition has been fixed (D2c task, both `{CEM, SA}` at
matched complexity, full `Steps(16M)` per replicate, seeds
derived via `splitmix64(MASTER.wrapping_add(i))` with
pre-registered `MASTER = 20_260_412`, per Ch 32's Decision 4),
and the bimodality contingency has been pre-registered (median
substitution if the Pearson-based bimodality coefficient
strictly exceeds `5/9`, per Ch 32's Section 6).

A subtlety worth naming: Ch 32's prerequisite is satisfied by
the chapter being drafted and its decisions carried into the
rematch protocol, not by a separate execution PR merging into
`main`. Ch 32 is a study chapter, not an execution PR chapter.
Ch 32 went further than this §4.4 entry originally anticipated
and specified a *folded* pilot — the rematch's first ten
replicates serve as both the pilot (measuring variance and
distributional shape via the bimodality coefficient) and the
first half of the rematch's data, with a pre-registered single
expansion to twenty replicates iff the initial-batch CI is
ambiguous. There is no separate pilot event. What Ch 32's
landing does *not* do on its own is run the rematch — it
specifies the protocol, and the protocol is executed at
rematch time by Ch 42's implementation. The prerequisite is
therefore two-part: Ch 32's decisions exist (satisfied by the
chapter's commit), and the rematch implementation honors the
initial-batch read with its escalation rule (enforced at
rematch execution time by Ch 42). A rematch implementation
that ignores the expansion rule and runs at a fixed `N` would
satisfy "Ch 32 has landed" but not the protective mechanism
this entry requires.

This is the only bucket-4 entry whose prerequisite is a study
chapter rather than an execution PR. Ch 31 includes it because
Ch 32's absence is a real failure mode of the rematch as a
scientific instrument, and Ch 31's bucket-4 shape — "the rematch
ran before its prerequisites landed" — covers conceptual
prerequisites as well as code prerequisites.

## Section 5 — What Chapter 31 does not decide

Chapter 31 is the failure-modes index. It deliberately declines
the following:

- **The numerical thresholds in §3.1 and §3.2.** The
  warmup-overhead gate's "predetermined factor" by which the
  effect size must exceed 0.2 percent is a Ch 32 call, not a Ch
  31 call. The seed-variance envelope's specific test choice and
  significance threshold are also Ch 32 calls. Ch 31 names the
  gates' shapes; Ch 32 picks the numbers.
- **The replicate count `N`.** Same as above. Ch 32 owns it,
  contingent on the pilot.
- **The specific pre-emptive language the rematch writeup uses
  in §1.1, §1.2, §1.3.** Ch 30 supplies the language for §1.1
  and §1.3; Ch 22's `k_passes` paragraph supplies the seed for
  §1.2's broader version. The rematch writeup carries the
  language forward, possibly with minor tightening for length.
  Ch 31 does not write the writeup's prose; it names which
  sentences from upstream chapters the writeup carries.
- **The specific column or footnote layout the rematch writeup
  uses for §2.1 and §2.2.** Ch 31 specifies that per-family
  actual env step counts appear as a separate column from the
  nominal budget, and that spread-metric labels disambiguate
  sample std from SEM at every reporting site. The exact
  column header text, footnote phrasing, and table caption
  style are writeup-author choices.
- **Whether `Ch 41` should fix the REINFORCE/PPO zero-fallback
  by emitting `None` or by skipping the metric.** Ch 24 §3.7
  flagged this latent defect and noted that "the cleanest fix
  is to skip the metric entirely"; Ch 24 §4.7 carries the
  contingency that the silent None-filter in Decision 2 depends
  on Ch 41 acting on §3.7's recommendation. Ch 31 inherits the
  flag as a bucket-4 prerequisite (not as a separate enumerated
  entry, because the failure shape it causes — silent `n`
  shrinkage in `SeedSummary` — is rolled into §4.2's "Ch 41
  must land first" framing). The choice between `None` and skip
  is Ch 41's decision.
- **The generalization of Ch 22's PPO-specific `k_passes` note
  to a class-wide algorithm-selection-guide failure mode in
  §1.2.** Ch 22's note addresses PPO's `k_passes` specifically;
  §1.2 generalizes it to "any reader who interprets the
  rematch's headline ranking as a practitioner's algorithm-choice
  guide." The generalization is Ch 31's framing, not Ch 22's,
  though it inherits Ch 22's pre-emption pattern. The
  generalization is the only piece of §1.2 that is not direct
  inheritance.
- **Whether a currently-moot gate belongs in a live failure-modes
  index.** §3.1's warmup-overhead gate is moot for the post-Ch-23
  `{CEM, SA}` rematch pool because neither algorithm has a
  warmup phase. Ch 31 keeps the entry because the gate's *shape*
  (effect size exceeds unbudgeted-overhead floor by a
  predetermined factor) generalizes past its current
  instantiation to any future pool addition that introduces a
  similar overhead. Whether that forward-compatibility
  generalization is worth an index slot is a Ch 31 editorial
  call — the only such call the chapter makes that is not pure
  inheritance.
- **Whether the "do not enable `--features parallel`"
  workaround in §4.1 is a legitimate pre-Ch-40 rematch-enabling
  path.** §4.1 describes the workaround's existence and the
  scope under which it currently holds (key finding 1 in the
  Ch 13 phase-2 recon: `sim/L0/core/Cargo.toml:37` declares
  `default = []` and no `Cargo.toml` under `sim/` opts in via
  `features = ["parallel"]`). It does not recommend running the
  rematch under the workaround. Whether the rematch should
  actually run before Ch 40 lands is an execution-sequencing
  question Ch 31 does not decide.
- **The execution-PR sequencing among Ch 40, Ch 41, Ch 42.** Ch
  31 names each as a prerequisite for the rematch but does not
  pick an order. The sequencing is implicit in the dependencies
  (Ch 40's chassis fix is independent; Ch 41's metric fix is
  independent; Ch 42's SA implementation depends on neither
  chassis nor metric being fixed first, but its rematch use
  depends on both), and the M4 decision in the study's
  Part-4-structure conversation ("one chapter per PR") does the
  rest. Ch 31 does not duplicate the sequencing argument.
- **Whether SA should also be added to the chassis tests.** Ch
  42's spec for SA includes its own test surface; Ch 31 does
  not enumerate SA's testing requirements as a failure mode of
  the rematch because SA's tests are inside Ch 42's scope, not
  inside the rematch's scope.
- **Any failure mode whose protective mechanism is "do better
  next time."** Ch 31 only enumerates failure modes that have a
  *concrete* protective mechanism — a writeup column, a
  pre-emptive sentence, a gate, a PR. Failure modes whose only
  defense is "the next experiment will be more careful" are
  outside Ch 31's scope, because they cannot be checked against
  the rematch's published artifact.

Chapter 31 enumerates the rematch's failure modes in four
buckets and points each one at the chapter that owns its
protective mechanism. It owns no decisions of its own. The
chapters it indexes — Ch 22, Ch 23, Ch 24, Ch 30, Ch 40, Ch 41,
Ch 42, Ch 32 — already made every load-bearing call. Ch 31's
contribution is the index itself: a single place a reader can
scan before forming a conclusion about what the rematch's
result means, and a single place a writer can scan before
publishing the rematch's writeup to check that every protective
mechanism is in place.
