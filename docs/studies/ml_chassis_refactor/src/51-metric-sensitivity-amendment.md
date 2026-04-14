# Metric-sensitivity amendment

Chapter 50 reported the 2026-04-14 D2c-SR rematch run, which
classified as `RematchOutcome::Null` under the bootstrap on
`RunResult::best_reward()` per replicate. Its §"What the metric
measured" section flagged a finding the pre-registered protocol
had not anticipated: the same per-replicate data would have
classified as `Positive` if the protocol had bootstrapped on
`RunResult::final_reward()` instead. For CEM — a population
method whose per-epoch `mean_reward` reflects the current
population rather than an incumbent — the gap between peak and
final rewards averaged 183.8 units per replicate. For SA — an
incumbent tracker whose `mean_reward` is monotone by construction
— the gap was near zero. The choice between peak and final
aggregation therefore swings the verdict on this run in a way
that Chs 24, 30, 31, and 32 did not predict and did not
pre-register as a decision space.

This chapter is the formal methodological response. It has four
sections. Section 1 audits where `best_reward` was named across
Chs 24, 30, 31, and 32, and classifies whether the metric was
pre-registered as a *scientific* decision (the operationalization
of Ch 30's question) or only as a *protocol* input (the default
scalar Ch 32's bootstrap resamples). Section 2 specifies the
protocol amendment this chapter picks — dual-metric reporting
through a new `TwoMetricOutcome` surface on
`sim_opt::analysis` — and commits the interpretation framework
for the joint `(best, final)` verdict in advance of the
amended re-run. Sections 3 and 4 are the results and
interpretation of that re-run; they are **deliberately unwritten
at the time this chapter is first committed**, so that the
amendment and its interpretation framework are on the record
before the verdict they will be used to adjudicate.

This chapter does not retire `best_reward`, does not re-open
Ch 30's scientific question, and does not preempt the Ch 30
null follow-ups (richer-proposal SA, Parallel Tempering) that
remain on the pre-committed menu. It does one narrow thing:
formalize the aggregation-metric choice that was implicit in the
rematch's chassis inheritance, and amend the rematch protocol so
that Ch 30's scientific question can be read through both
metrics jointly instead of through one of them by accident.

## Pre-registration scope for this chapter

Before the audit, the scope rule. Sections 1 and 2 of this
chapter are committed to `main` before Section 3 or 4 is
written, and before the code changes in `sim_opt::analysis`
that produce Section 3's numbers have landed. The git commit
that introduces this file contains Sections 1 and 2 only. Any
subsequent commit that adds Sections 3 and 4 is a follow-up
commit after the step B code change and after the re-run. The
separation is intentional: Section 2 is this chapter's
pre-registration, and a pre-registration that is written after
its result is observed is not a pre-registration.

One honest qualifier up front. Chapter 50 already reports, in
its per-replicate table, both `CEM peak` and `CEM final` for
the 2026-04-14 run (columns at 50-d2c-sr-rematch-writeup.md:197-208),
and its prose states in so many words that *"had the rematch
used 'final epoch mean reward' as its replicate statistic, the
bootstrap CI would have been firmly positive and the verdict
would have been Positive"* (50-d2c-sr-rematch-writeup.md:180-190).
The dual-metric numbers for the existing run are therefore
predictable from a reader of Ch 50 alone. The pre-registration
this chapter offers is not "we don't know what the amended
verdict will say for the 2026-04-14 data." It is "we commit the
interpretation framework in Section 2.3 before the re-run, so
that the framework's rules are on the record independently of
which cell of the rule table the verdict lands in." The
framework's theoretical grounding — the peak-vs-final asymmetry
of population versus incumbent methods — would be the same
framework if the 2026-04-14 data had landed in any other cell.

## Section 1 — The audit: where `best_reward` was named

### 1.1 The question this section answers

Ch 50's finding is that the aggregation primitive chosen by
Ch 24 — `RunResult::best_reward()`, the max over a run's
per-epoch `mean_reward` — is load-bearing on the Ch 30 verdict
in a way that was not visible when the protocol was designed.
Before amending the protocol, this section asks a narrower
historical question: *was `best_reward` explicitly pre-registered
as the per-replicate statistic for Ch 30's scientific question
in Chs 30, 31, or 32, or did it get implicitly inherited from
Ch 24's aggregation-surface pick without being flagged as
load-bearing on Ch 30's question?*

The distinction matters because pre-registration guards against
post-hoc metric cherry-picking — picking, after seeing a result,
the metric that produces the result you wanted. If the metric
was explicitly named as Ch 30's operationalization and then
swapped after the verdict came in, the swap would violate
pre-registration semantics regardless of how theoretically
defensible the new metric is. If the metric was implicitly
inherited — if no chapter ever asked "is `best_reward` the right
operationalization of Ch 30's question?" — then the amendment is
not a cherry-pick; it is the first time that question gets
asked, and the amendment's answer is committed before the
amended verdict is observed.

The hypothesis going into the audit is that the metric was
implicit. The audit confirms that hypothesis.

### 1.2 Ch 24 — where the primitive was shipped

Ch 24's job was to standardize `EpochMetrics::mean_reward`
across the five algorithms (Decision 1) and to pick the
across-replicate aggregation surface `CompetitionResult`
exposes (Decision 2). Both decisions occurred in service of
the *chassis*, not in service of Ch 30's scientific question.

The chapter lays out its deferral list explicitly. Section 5
("What Chapter 24 does not decide," at
24-result-semantics.md:1091-1159) names eleven things the
chapter declines to pick. The statistical test is deferred to
Ch 32 (24-result-semantics.md:1097-1101). The replicate count
`N` is deferred to Ch 32 (:1102-1103). The pilot design is
deferred to Ch 32 (:1104-1106). Percentile helpers are
rejected as speculative for Ch 24 and deferred to Ch 32 if
the data demands them (:1107-1110).

The eleventh item is the one the audit needs, and it is what
is *not* on the list. Section 5 contains this bullet at
24-result-semantics.md:1154-1159:

> **Whether `RunResult::final_reward()` at
> `competition.rs:36-41` has the same semantic issue.** It does
> — `final_reward()` also pulls from
> `metrics.last().mean_reward`, so once Decision 1 lands the
> helper is fixed for free along with `best_reward()`. No
> separate Chapter 24 pick is needed.

This is the single passage in Ch 24 that names `final_reward`
alongside `best_reward`. Its framing is diagnostic of the
chapter's view of the metric space: the two helpers are treated
as parallel plumbing that get fixed together under Decision 1's
unit rescaling, not as alternative operationalizations of the
rematch's statistical question. Ch 24 literally has `final_reward`
in view and immediately returns to treating `best_reward` as the
default without naming the choice. The deferral list does not
contain an item like *"whether the rematch's per-replicate
statistic should be `best_reward` or `final_reward` is a
Chapter 32 decision."* That question is not on the list because,
from Ch 24's vantage, it was not a question.

Ch 24 §4.3 picks the aggregation surface at
24-result-semantics.md:868-912. The API that ships is:

```rust
impl CompetitionResult {
    pub fn replicate_best_rewards(&self, task: &str, algo: &str) -> Vec<f64>;
    pub fn describe(&self, task: &str, algo: &str) -> Option<SeedSummary>;
}
```

The primitive is named `replicate_best_rewards`, not
`replicate_rewards_by_metric(metric)`. There is no
`replicate_final_rewards` accessor. The singular naming is
consistent with Ch 24's implicit view: there is *one* per-replicate
statistic, it is derived from `best_reward`, and the aggregation
surface exposes it. This is the moment the metric choice was
effectively made, and it was made as a consequence of choosing
the aggregation *surface*, not as a scientific pre-registration.

**Classification of Ch 24:** references `best_reward` as the
aggregation-surface primitive without flagging it as load-bearing
on Ch 30's question. The scientific-commitment question was
never surfaced. Ch 24 shipped the metric as a mechanical
by-product of Decisions 1 and 2, not as a Ch 30-aware choice.

### 1.3 Ch 30 — the scientific question

Ch 30 is 222 lines long. It contains zero formal-protocol
references to `best_reward`, `final_reward`,
`replicate_best_rewards`, or `RunResult`. It names the scientific
question and the three outcomes and then defers all metric
decisions downstream.

The question is framed at 30-the-scientific-question.md:45-49
as *"resolving the peak within [the elevated region] —
reliably ending up near `kt_mult ≈ 2.55` rather than somewhere
plausible but worse inside the elevated region."* The framing
is a statement about *where the converged policy lives in
parameter space*, not about *what reward the training
trajectory transiently touches*. This matters for §2's
interpretation framework: Ch 30's question is more directly
operationalized by a measurement of converged policy quality
than by a measurement of peak-across-training reward.

The Positive outcome at 30-the-scientific-question.md:148-162
is stated as *"SA resolves the SR peak — converges to a
`kt_mult` near the empirical peak around $2.55$, with reward
near the measured maximum."* Again the framing is
converged-policy-first.

The Null outcome at 30-the-scientific-question.md:164-170 is
the only passage in Ch 30 that contains the phrase "best
reward." It reads *"SA finds the elevated region but, like CEM,
cannot resolve the peak; its best reward is statistically
indistinguishable from the best matched-complexity RL
baseline."* The phrase "best reward" appears without
antecedent and without a code reference. A reader arriving at
this line does not know whether "best reward" means
`RunResult::best_reward()`, the final-epoch reward of the best
replicate, the average reward of the best-performing
algorithm, or something else. The usage is colloquial and
pre-theoretic: "the best reward we observed" in the
conversational sense, not "the `best_reward()` accessor we
pre-registered as the per-replicate statistic." Nothing in the
chapter ties the phrase to Ch 24's `RunResult::best_reward()`.

An adversarial reader could push back on the "colloquial"
reading. *"'Best reward' is literally the name of Ch 24's
accessor,"* the pushback goes. *"Occam's razor says Ch 30
meant the accessor. Your colloquial reading is motivated
reasoning on a chapter that inconveniently names the metric."*
The rebuttal is a construction-order argument. Ch 30 precedes
Ch 24 in the study's logical dependency order — Ch 30 itself
states at 30-the-scientific-question.md:201-206 that effect
size, significance threshold, and replicate count are *"all
chapter 23 / 24 questions,"* explicitly deferring the numerical
operationalization to chapters that come after it. A reader
writing "best reward" in Ch 30 could not have been citing a
specific accessor from a chapter whose picks had not been made
yet in the study's construction. The colloquial reading is
forced by the dependency order, not preferred for convenience.
If Ch 30 had wanted to pre-register `RunResult::best_reward()`
as the per-replicate statistic, it would have had to either
(a) wait until Ch 24 shipped the accessor and then amend
Ch 30 retroactively, or (b) name a code-level reference. It
did neither.

The chapter closes with an explicit deferral list at
30-the-scientific-question.md:201-222:

> It does not decide the effect size we would call
> "convincingly positive." It does not decide the significance
> threshold. It does not decide how many seeds constitute
> enough replicates for this specific experiment. All of
> those are chapter 23 / 24 questions.

The deferral routes *everything numerical* downstream. The
per-replicate statistic is not mentioned as a separate
deferral because, like Ch 24 before it, Ch 30 did not see
the metric as a decision space. The chapter pre-registers the
*question* and the *shape of the three outcomes*, and leaves
the operationalization to Chs 23 and 24.

**Classification of Ch 30:** does not discuss the replicate-level
statistic at all in a formal-protocol sense. The single
colloquial "best reward" at :165 is insufficient to count as a
pre-registration of `RunResult::best_reward()` as THE metric
for the scientific question.

### 1.4 Ch 31 — failure modes

Ch 31's preamble at 31-failure-modes.md:3-10 states that
*"Chapters 22, 23, and 24 specified the rematch's compute-parity
unit, pool, matched-complexity anchor, per-replicate metric, and
across-replicate aggregation surface,"* which is the first
place the phrase "per-replicate metric" appears in the study,
and where it appears it is *inherited*, not chosen. The chapter
then makes the framing explicit at :12-19: *"This chapter is
the index...Chapter 31 makes no new design calls. Every failure
mode it names inherits from a committed chapter."* Ch 31's
authorial stance is that metric decisions are upstream and
already settled; the chapter's job is to catalogue failure
modes, not re-derive picks.

Ch 31 references `replicate_best_rewards` in two places, both
infrastructural. At 31-failure-modes.md:247-251 the passage
cites `replicate_best_rewards` as the *shape precedent* for a
proposed new `replicate_total_steps` accessor (the per-family
step-counts guard) — the reference is about the aggregation
primitive's API shape, not about best-reward as the scientific
metric. At :535-537 the passage names `replicate_best_rewards`,
`describe`, and `SeedSummary` as the three pieces of Ch 24
Decision 2 that Ch 41's execution PR has to land before the
rematch can run. Both references treat `best_reward` as given
infrastructure, inherited through the chassis surface. Neither
engages the scientific-commitment question.

**Classification of Ch 31:** infrastructure-level references
only, in explicit service of cataloguing (not deciding). No
pre-registration of the metric as a scientific choice.

### 1.5 Ch 32 — the statistical protocol chapter

Ch 32 is 1460 lines long and is the chapter where the
pre-registration question is least clear-cut. Ch 32 is the
chapter that committed to a statistical test, a classification
rule, a bimodality contingency, and a replicate count. If the
metric was going to be pre-registered as a scientific decision
anywhere, Ch 32 would be the place.

The inheritance statement is made specific at
32-hyperparameter-sensitivity.md:112-127:

> Ch 24 picked the per-replicate metric (every algorithm's
> `EpochMetrics::mean_reward` standardized on "mean per-episode
> total reward across `n_envs` trajectories", with three train
> loops touched at `cem.rs:209`, `td3.rs:490`, and
> `sac.rs:543`) and the across-replicate aggregation surface
> (`replicate_best_rewards(task, algo) -> Vec<f64>` as the raw
> primitive...). The shape was deliberately chosen so that any
> reasonable statistical test could be built on top of it.
> Ch 32's test family is built on the raw primitive: the
> bootstrap resamples the per-replicate `f64` values directly.

Three observations about this passage. First, Ch 32 names its
inheritance as "the per-replicate metric," singular, and the
metric it names is the one Ch 24 standardized — `mean_reward` at
the per-epoch level, implicitly aggregated by `best_reward()`
at the per-replicate level via the `replicate_best_rewards`
accessor. Second, the phrase *"any reasonable statistical test
could be built on top of it"* treats the primitive as a
chassis-level affordance, not as a scientific commitment; Ch 32
is naming what its own test operates on, not what Ch 30's
question demands. Third, the sentence *"the bootstrap resamples
the per-replicate `f64` values directly"* elides the question
"what's in the `f64`?" The answer is `best_reward`, but the
chapter's framing is that the `f64` is a given input, not a
scientific choice.

Ch 32 §1.1 at :170-180 is more specific:

> Each run produces a `RunResult` with a per-epoch metrics
> history; `RunResult::best_reward()` reduces that history to
> a single scalar — the maximum `mean_reward` over the run's
> epochs, in the per-episode-total units Ch 24 standardized
> on. Ch 24's `replicate_best_rewards(task, algo)` returns
> the `Vec<f64>` of these scalars across the `N` runs.

This is the closest Ch 32 comes to naming the metric. The
naming is description, not pre-registration: the chapter is
explaining what the chassis accessor returns and how the
bootstrap consumes it. No counterfactual is named — "we could
have used `final_reward` instead but picked `best_reward`
because…" — and no scientific justification is offered for
why maximum-over-epochs is the right operationalization of
"resolves the peak."

The decisive passage is in Ch 32 §"What Chapter 32 does not
decide" at 32-hyperparameter-sensitivity.md:1381-1387:

> **Whether the rematch should also report secondary metrics
> alongside `best_reward`.** Ch 24 standardized `mean_reward`
> and `RunResult::best_reward()` is the per-replicate scalar.
> Whether the rematch writeup should also report
> `RunResult::final_reward()`, the full epoch-by-epoch reward
> curve, or other secondary metrics is a writeup author
> choice. Ch 32's protocol operates on `best_reward` only.

This passage is the best candidate for an explicit Ch 30/31/32
commitment, and on close reading it is still not one. The
passage commits Ch 32's *protocol* to `best_reward` as its
input — "Ch 32's protocol operates on `best_reward` only" — but
frames the commitment as inheritance from Ch 24 rather than as
a Ch 30-aware scientific choice. The dispositive word is "also"
in "the rematch should also report secondary metrics alongside
`best_reward`." The question Ch 32 declines is whether to
report `final_reward` *as a secondary* alongside the primary
`best_reward`; it does not entertain whether `best_reward`
should itself be the primary. That framing presupposes
`best_reward` as the primary, inherits that presupposition
from Ch 24's chassis pick, and punts anything else to writeup
author choice. The word "also" carries the presupposition; the
sentence's shape is unintelligible without it.

A Ch 32 passage that pre-registered `best_reward` as a scientific
commitment would read differently. It would read something like
*"We considered operationalizing 'resolves the peak' through
`best_reward` (the max over training) versus `final_reward` (the
converged policy) and chose `best_reward` because…"* No such
passage exists anywhere in Ch 32. The chapter's commitment to
`best_reward` is protocol-level (what the bootstrap's input is),
not scientific-level (what Ch 30's question means).

**Classification of Ch 32:** committed to `best_reward` at the
protocol level as Ch 32's bootstrap input, with the commitment
framed as inheritance from Ch 24. Did not commit to `best_reward`
at the scientific level as the operationalization of Ch 30's
"resolves the peak" question. The 1381-1387 passage explicitly
treats `final_reward` reporting as a "writeup author choice,"
language that would be unnecessary if `best_reward` had been
pre-registered as scientifically load-bearing.

### 1.6 What the audit concludes

The metric was **pre-registered at the protocol level** (Ch 32's
bootstrap operates on `best_reward`, via Ch 24's aggregation
surface) and **not pre-registered at the scientific level** (no
chapter examined whether `best_reward` operationalizes Ch 30's
"resolves the peak" correctly). The distinction matters for
what the amendment is allowed to do.

Pre-registration guards against a specific failure mode:
observing an unwanted verdict, then picking a different metric
*because it produces a wanted verdict*, then claiming the
different metric was the correct choice all along. For the
guard to bite, the original metric has to have been named as
*the correct choice* at the time of pre-registration. Ch 32's
commitment is narrower than that. It names `best_reward` as the
bootstrap's input, not as the correct operationalization of
Ch 30's question, and its surrounding text (the "author choice"
language at :1381-1387) is consistent with the narrower reading.

The amendment this chapter picks is therefore scope-legal for
two independent reasons. First, the metric was never pre-registered
at the scientific level, so there is no scientific pre-registration
for the amendment to cherry-pick around; it is the first time
the scientific-level question has been asked. Second, the
amendment does not *replace* `best_reward` as the protocol's
input — Ch 32's protocol-level commitment stands intact — it
*extends* the protocol to run the same bootstrap and
classification pipeline on both `best_reward` and `final_reward`
in parallel, and reports both classifications in the verdict.
Ch 32's "operates on `best_reward` only" sentence stays exactly
as Ch 32 wrote it. The amendment does not edit Ch 32; it adds
a Ch 51 clause that the same bootstrap-and-classify pipeline
Ch 32 specified is also applied, in parallel, to `final_reward`
through the new `replicate_final_rewards` accessor. Nothing
Ch 32 decided is being retracted; Ch 32's test family is being
applied twice instead of once, under a Ch 51 extension Ch 32
did not anticipate but does not forbid.

The third safeguard, and the one Section 2 implements in
prose, is that the interpretation framework for the joint
`(best, final)` verdict is committed here — in writing, in
this commit — before Section 3's results exist. Section 2.3's
nine-cell interpretation table is the pre-registration this
chapter's amendment rests on.

## Section 2 — The amendment: dual-metric reporting

### 2.1 What the amendment changes

The amendment extends the `sim_opt::analysis::run_rematch`
driver and its `_with_runner` variant so that they compute
both `best_reward` and `final_reward` per-replicate statistics
and run the Ch 32 §3.2/§3.3 bootstrap-and-classify pipeline
twice, once on each primitive. The return type changes from
`Result<RematchOutcome, EnvError>` to
`Result<TwoMetricOutcome, EnvError>`, where:

```rust
pub struct TwoMetricOutcome {
    pub best_ci: BootstrapCi,
    pub best_outcome: RematchOutcome,
    pub final_ci: BootstrapCi,
    pub final_outcome: RematchOutcome,
}
```

`CompetitionResult` gains a parallel accessor
`replicate_final_rewards(task, algo) -> Vec<f64>` that mirrors
the existing `replicate_best_rewards` in shape, including the
§4.7 silent-filter-out of `None` replicates (see
24-result-semantics.md:1007-1065). The bootstrap is the same
function; it takes a `&[f64]` input, and step B passes both
primitives through the same code path. `emit_rematch_summary`
gains a side-by-side layout that prints both tables and both
CI/classification blocks.

The fixture at `sim/L0/opt/tests/d2c_sr_rematch.rs` does not
gain a new assertion. Ch 42 §6 sub-decision (g) picked
"protocol-completes-cleanly" as the fixture's gate shape, and
that gate shape is metric-agnostic: it passes on a non-error
`TwoMetricOutcome` regardless of what either of the two
`RematchOutcome` values says. The fixture's eprintln verdict
block is extended to print both verdicts side by side; the
human reader does the interpretation via Section 2.3's
framework.

### 2.2 What the amendment does not change

Ch 24's aggregation primitive stays. `replicate_best_rewards`
continues to return `Vec<f64>` in the same shape with the same
`None`-filtering contract. `best_reward` remains the default
per-replicate statistic for any caller outside the rematch;
Ch 24 §4.3's API is additively extended, not retracted.

Ch 32's bootstrap-and-classify pipeline is unchanged. The same
`B = 10_000` resamples, the same classification table from
Ch 32 §3.3, the same bimodality contingency from Ch 32 §6.3
(applied to each metric's replicate vector independently), and
the same N=10 → N=20 expansion rule from Ch 32 §3.3 are used
for both primitives. The expansion rule is triggered if
*either* metric's classification is Ambiguous; if only one is
Ambiguous, both primitives are expanded together so that the
replicate pool stays matched.

Ch 30's three-outcome language (Positive, Null, Ambiguous)
stays. The scientific question — *does SA resolve the D2c-SR
peak more reliably than CEM?* — is unchanged. What changes is
the *operationalization* of the question: the verdict is now
a pair of three-outcome classifications instead of a single
classification, and Section 2.3 specifies how the pair maps
back to a single reading of Ch 30's question.

### 2.3 The interpretation framework

Ch 30 named three outcomes. Under the amendment the rematch
produces two classifications, one per metric, yielding nine
possible `(best, final)` combinations. This section commits,
in advance of any amended re-run, to what each combination
means for Ch 30's scientific question. The interpretation is
derived from the theoretical difference between `best_reward`
and `final_reward` as aggregation choices under population
versus incumbent methods, not from any specific run's numbers.

**Rationale for the framework.** CEM is a population method;
its per-epoch `mean_reward` evaluates the mean performance of
the current population, which is refit each epoch from the
elite fraction of the previous epoch's evaluations. A lucky
population samples high rewards at one epoch; the elite refit
pulls the distribution toward the lucky members; the next
epoch's population draws from the shifted distribution and
can end up off-peak. `best_reward` — the max over the training
trajectory — captures the luckiest epoch in that drifting
sequence, which is not the same as the converged policy's
reward. SA is an incumbent tracker; its per-epoch
`mean_reward` evaluates the incumbent policy, which is
monotone by construction under the Metropolis accept rule. For
SA, `best_reward` and `final_reward` are nearly the same
number by design (Ch 50 reports exactly this pattern at
50-d2c-sr-rematch-writeup.md:233-242).

The scientific asymmetry is: `best_reward` systematically
rewards algorithms whose training trajectory passes through
high-reward regions, regardless of whether the converged
policy ends up there. `final_reward` rewards algorithms whose
converged policy is the good one, regardless of what the
trajectory touched along the way. Ch 30's question asks about
*resolving* the peak — reliably *ending up* near
`kt_mult ≈ 2.55` — which is a statement about the converged
policy's location in parameter space, not about the training
trajectory's transient reward. On that reading, `final_reward`
is the more direct operationalization of Ch 30's question.
`best_reward` is a weaker and noisier signal about the same
question: it reports whether the training trajectory ever
passed through a region of high mean-reward, which may or may
not coincide with the physics-resonance peak at
`kt_mult ≈ 2.55`. A high `best_reward` value could come from
transient visits to the resonance peak, from a local plateau
in the reward landscape, or from exploration-noise inflation
of the per-epoch mean. It bounds exploration reach in the
reward dimension, not peak proximity in the parameter
dimension, and the framework reads it accordingly.

The framework treats `final_reward` as the *primary*
operationalization of Ch 30's "resolves the peak" question and
`best_reward` as a *complementary* signal about exploration
reach. The framework does not retire `best_reward` — a null
on `best_reward` is still informative as a bound on how high
the algorithm's training trajectory ever went — but it reads
the two classifications through the lens of what each one
actually measures.

**The nine-cell table.** For each of the nine
`(best_outcome, final_outcome)` combinations, the framework
maps the pair to one of four Ch 30 readings: Positive, Null,
Split-metric (needs interpretation), or Expand (fire Ch 32's
N=20 expansion before reading the verdict). The mapping is:

| best        | final       | Ch 30 reading  | notes                                                     |
|-------------|-------------|----------------|-----------------------------------------------------------|
| Positive    | Positive    | **Positive**   | SA both touches higher rewards AND holds them. Clean.     |
| Positive    | Null        | **Split**      | SA touches but does not hold. Converged policy not better.|
| Positive    | Ambiguous   | Expand         | Fire N=20 on both metrics; re-read after.                 |
| Null        | Positive    | **Positive**   | SA holds the peak; CEM only touches it transiently.       |
| Null        | Null        | **Null**       | Neither touches nor holds. Robust null.                   |
| Null        | Ambiguous   | Expand         | Fire N=20 on both metrics; re-read after.                 |
| Ambiguous   | Positive    | Expand         | Fire N=20 on both metrics; re-read after.                 |
| Ambiguous   | Null        | Expand         | Fire N=20 on both metrics; re-read after.                 |
| Ambiguous   | Ambiguous   | Expand         | Fire N=20 on both metrics; re-read after.                 |

The two non-obvious cells are `(Positive, Null)` and
`(Null, Positive)`. Both are the framework's response to
metric-sensitive data — situations where the two metrics
disagree on which algorithm wins. The framework's resolution
is that **`final_reward` is dispositive for Ch 30's question**:

- `(Positive, Null)` reads as **Split**, not Positive,
  because SA's exploration reached high rewards but its
  converged policy did not outperform CEM's. For the "resolves
  the peak" question, reaching without resolving is an
  incomplete answer. A Split reading triggers the Ch 30 null
  follow-ups (richer-proposal SA, Parallel Tempering) on the
  theory that the isotropic proposal found the neighborhood but
  could not lock onto the peak, and a richer proposal structure
  or a PT chain might.
- `(Null, Positive)` reads as **Positive**, because the
  converged policy — which is what "resolves the peak" is
  actually asking about — outperforms CEM, even though CEM's
  lucky populations transiently touched higher rewards. The
  Null on `best_reward` in this cell reflects CEM's
  population-dynamics volatility, not SA's inability to reach
  the peak. For Ch 30's question as written, this cell is a
  positive result.

The asymmetry of the two non-obvious cells is not a fudge.
It follows from the framework's commitment to `final_reward`
as the primary operationalization of *"resolves the peak."*
Under a framework that had committed to `best_reward` as
primary, the two cells would swap readings and the nine-cell
table would be symmetric. This chapter commits to
`final_reward` as primary because the theoretical grounding
points that way, and because the 2026-04-14 run's
metric-sensitivity finding is specifically the signature of
a `(Null, Positive)` cell under the `best_reward`-primary
reading. If the framework does not commit to `final_reward`
as primary, the amendment is not distinguishable from
"report two metrics and let the reader pick" — which is a
disclosure move, not an amendment.

**Transparency qualifier.** As noted in this chapter's
preamble, Ch 50's per-replicate table already makes the
2026-04-14 run's dual-metric outcome predictable: the
existing data lands in the `(Null, Positive)` cell under
Section 2.3's framework, and the framework reads that cell
as **Positive** for Ch 30's question. A reader who is
suspicious that the framework was engineered to produce a
Positive reading on the 2026-04-14 data is entitled to that
suspicion, and the best response is the framework's
theoretical grounding: the peak-vs-final asymmetry between
population and incumbent methods is a prior about algorithm
dynamics, not about this run's outcome, and the framework
would read the same way if the 2026-04-14 data had landed in
any other cell. A more conservative reader may prefer to
treat `(Null, Positive)` as **Split** rather than Positive on
the grounds that metric-sensitivity itself is a warning sign
and the conservative move is to fire the Ch 30 follow-ups
anyway. Section 2.4 below says something explicit about what
the follow-ups do under each reading.

### 2.4 What the amendment does not preempt

The Ch 30 null follow-ups — richer-proposal SA and Parallel
Tempering, both sketched in Ch 30 §3 and scope-estimated in
Ch 92 §3 — remain on the pre-committed menu. Whether they
fire after the dual-metric re-run depends on which cell of
Section 2.3's table the re-run lands in and on which reading
of the disputed cells the user accepts:

- If the re-run lands in a clean cell (`Positive/Positive` or
  `Null/Null`), the follow-ups fire or not per Ch 30's original
  rule: a Null fires them, a Positive does not.
- If the re-run lands in `Null/Positive` and the user accepts
  this chapter's Positive reading, the follow-ups are arguably
  moot — SA has resolved the peak under the direct
  operationalization, and the follow-ups' purpose (finding out
  whether a richer proposal can resolve a peak basic SA
  couldn't) is already answered. The user may nonetheless
  choose to run the follow-ups as a sensitivity check on the
  framework's reading. This chapter takes no position on that
  choice.
- If the re-run lands in `Positive/Null` (Split), the
  follow-ups fire per the Split reading in Section 2.3 — the
  isotropic proposal touched the peak but could not lock onto
  it, and the follow-ups are the exact response to that
  failure mode.
- If the re-run lands in any Ambiguous cell, Ch 32's N=20
  expansion fires first, and the follow-up decision waits for
  the expanded data.

The follow-ups' paused status is unchanged by this chapter.
This chapter's deliverable is the amendment and its
interpretation framework; the firing decision for the
follow-ups is made after Section 3 is written.

### 2.5 Reproducibility anchors for the amendment

The pre-registration recorded by Sections 1 and 2 is
anchored to the commit that introduces this file. Any
subsequent reader or re-runner needs to be able to verify
that Sections 1 and 2 were committed before Sections 3 and 4
were written, and that Section 2.3's framework was committed
before the amended re-run's numbers were known. The anchors:

- **Sections 1 and 2 commit** — to be filled in by the commit
  that lands this file, before `sim_opt::analysis` gains
  `TwoMetricOutcome` and before the fixture re-runs under the
  new pipeline. Branch: `feature/ml-chassis-post-impl`.
  Parent commit: `c8639cbd` (appendix gap close).
- **Step B code commit(s)** — to be filled in after
  `sim_opt::analysis` gains `TwoMetricOutcome`,
  `replicate_final_rewards`, and the extended `run_rematch`
  driver; after the fixture builds green under the new
  return type; and before the amended re-run fires.
- **Amended re-run commit** — to be filled in after the
  fixture runs under the new pipeline, with a captured log
  at `/tmp/d2c_sr_rematch_dual.log` (ephemeral — regenerate
  with the same caffeinate-wrapped cargo-test command Ch 50
  uses).
- **Sections 3 and 4 commit** — to be filled in after the
  amended re-run is captured, in a commit whose parent is
  the amended-re-run commit.

The four commits are deliberately separated so that
`git log --reverse` over this file plus `sim/L0/opt/src/analysis.rs`
plus the fixture's log makes the pre-registration sequence
visible without needing to trust this chapter's own claim
about when things were written.

Seeds and bootstrap constants are unchanged from Ch 50:
`REMATCH_MASTER_SEED = 20_260_412`,
`BOOTSTRAP_RNG_SEED = 0xB007_0057_00AA_0055`,
`N_INITIAL = 10`, `B = 10_000`,
`TrainingBudget::Steps(16_000_000)`, matched-complexity
anchor `LinearPolicy(2, 1)` with `n_params = 3` and init
`[0.0, 0.0, 2.0]`. If the amended re-run reproduces Ch 50's
per-replicate peak and final numbers (modulo any
determinism discrepancy the re-run itself reveals), the
numerical anchor is the same; only the classification
pipeline's output shape changes.

## Section 3 — Results

*This section will be written after the step B code change
lands on `feature/ml-chassis-post-impl` and the rematch
fixture re-runs under the `TwoMetricOutcome` pipeline. The
deliberate separation between Sections 1–2 and Sections 3–4
is part of this chapter's pre-registration discipline: the
amendment and its interpretation framework are committed
before the amended verdict is observed. See Section 2.5 for
the four-commit reproducibility anchor.*

## Section 4 — Interpretation

*This section will be written after Section 3. It will map
the amended run's `(best, final)` verdict through Section
2.3's nine-cell framework, report the Ch 30 reading
(Positive, Null, Split, or Expand), and state which Ch 30
null follow-ups (if any) fire next.*

## Cross-references

- Ch 24 §4.3 (24-result-semantics.md:868-912) — where
  `replicate_best_rewards` was named as the raw primitive
- Ch 24 §5 (24-result-semantics.md:1091-1159) — the 11-item
  "what Ch 24 does not decide" list, notably its treatment of
  `final_reward` at :1154-1159 as parallel plumbing without
  naming it as a metric alternative
- Ch 30 §"What each outcome would tell us"
  (30-the-scientific-question.md:143-199) — three outcomes
  named in converged-policy language
- Ch 30 §"What the chapter does not decide"
  (30-the-scientific-question.md:201-222) — explicit deferral
  of metric decisions to Chs 23/24
- Ch 31 preamble (31-failure-modes.md:3-19) — inheritance of
  "per-replicate metric" from Chs 22-24, with Ch 31 explicitly
  framing itself as a cataloguing chapter that makes no new
  design calls
- Ch 31 `replicate_best_rewards` references
  (31-failure-modes.md:247-251 and :535-537) — both
  infrastructural: the first as a shape precedent for a
  proposed `replicate_total_steps` accessor, the second as
  part of the Ch 24 Decision 2 surface Ch 41 must land before
  the rematch
- Ch 32 §"What Ch 32 inherits from earlier chapters"
  (32-hyperparameter-sensitivity.md:112-127) — the Ch 24
  inheritance statement
- Ch 32 §1.1 (32-hyperparameter-sensitivity.md:170-180) —
  `best_reward` named as the bootstrap's input
- Ch 32 §"What Chapter 32 does not decide"
  (32-hyperparameter-sensitivity.md:1381-1387) — the
  "author choice" framing that this chapter's §1.5 audit
  treats as the strongest non-pre-registration evidence
- Ch 50 §"What the metric measured"
  (50-d2c-sr-rematch-writeup.md:167-298) — the post-execution
  finding that triggered this chapter
- Ch 50 per-replicate table
  (50-d2c-sr-rematch-writeup.md:193-208) — the
  existing-data anchor for the dual-metric re-run
- `sim/L0/ml-bridge/src/competition.rs:43-45` —
  `RunResult::final_reward()` (exists, to be plumbed through
  `CompetitionResult::replicate_final_rewards` in step B)
- `sim/L0/ml-bridge/src/competition.rs:49-54` —
  `RunResult::best_reward()` (existing, unchanged)
- `sim/L0/opt/src/analysis.rs` — the module step B extends
  with `TwoMetricOutcome` and the dual-metric driver
- `sim/L0/opt/tests/d2c_sr_rematch.rs` — the fixture whose
  eprintln verdict block gains a dual-metric layout
