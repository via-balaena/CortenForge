# Hyperparameter sensitivity

Chapter 30 named three meaningful outcomes for the rematch
(positive, null, ambiguous) and explicitly deferred three things
to later chapters: the effect size that would count as
"convincingly positive", the significance threshold, and the
replicate count. Chapter 24 picked the across-replicate
aggregation surface (`SeedSummary { n, mean, std_dev }` plus the
raw `replicate_best_rewards(task, algo) -> Vec<f64>` primitive)
in a deliberately test-agnostic shape — Section 5 of Ch 24
listed "the statistical test", "the replicate count `N`", and
"the pilot design itself" as the three things Ch 24 would not
decide and pointed at this chapter as the owner of all three.
Chapter 31 then named two further owed values: the
"predetermined factor" by which the rematch's effect size must
exceed the 0.2 percent warmup-overhead floor (§3.1) and the
test-and-significance choice that bounds the seed-variance
envelope (§3.2). Chapter 31 §4.4 also named "pilot existence"
as a bucket-4 prerequisite for the rematch and identified this
chapter as the protective mechanism.

This chapter picks all of those values. It is an argument
chapter in the Ch 23 / Ch 24 mold: it audits the statistical
landscape the upstream chapters have set up, picks a small
number of decisions, defends each pick against the alternatives
that could have gone differently, and ends with a
scope-discipline section naming what it does not decide.

The picks are:

1. **Test family.** Bootstrap CI on the difference of means;
   `B = 10000` resamples; two-sided 95 percent interval. The
   modern empirical-RL standard (Henderson 2018, Agarwal 2021),
   distribution-free, and the only test in the candidate pool
   that does meaningful work in the small-`N` regime without
   resting on a normality assumption.
2. **Significance threshold and Ch 30 mapping.** CI lower bound
   above zero ⇒ **positive**; CI upper bound at or below zero ⇒
   **null**; CI straddles zero with point estimate above zero ⇒
   **ambiguous**; CI straddles zero with point estimate at or
   below zero ⇒ **null**. Three Ch 30 buckets, no fourth-option
   hedge.
3. **Replicate count protocol.** `N = 10` initial replicates per
   algorithm, with a pre-registered single expansion to `N = 20`
   if and only if the initial-batch CI is ambiguous. Hard cap at
   `N = 20`. The initial batch *is* the pilot — there is no
   separate pre-rematch run.
4. **Initial-batch protocol.** D2c task; both `{CEM, SA}`; ten
   replicate seeds derived as
   `splitmix64(MASTER.wrapping_add(i))` for `i` in `0..10` with
   pre-registered `MASTER = 20_260_412` (Ch 23 §1.2's prose
   recommendation, not raw sequential integers); full
   `Steps(16M)` per-replicate budget. After-batch reads:
   bootstrap 95 percent CI on the difference of means and the
   Pearson-based bimodality coefficient per algorithm.
5. **Warmup-overhead factor (Ch 31 §3.1).** `5×` — the rematch's
   effect size must exceed 1 percent of the budget-equivalent
   reward scale to clear the warmup gate. Currently moot for the
   `{CEM, SA}` pool (neither algorithm has a warmup phase) but
   fixed for forward use under any future pool extension.

Plus one named contingency, pre-registered as a deterministic
rule rather than a post-hoc judgment call: if either algorithm's
initial-batch bimodality coefficient exceeds `5/9`, the test
escalates to bootstrap CI on the difference of *medians* with
the same `B`, the same 95 percent interval, and the same Ch 30
mapping. One-line site change. No further human adjustment at
the escalation point.

The chapter ends by naming what it does not decide: the precise
RNG sequence used to draw bootstrap resamples, the I/O format of
the rematch writeup's CI plot, the wall-clock budget the
rematch's compute will consume, and several smaller choices that
belong to the execution PR plans in Part 4. It also notes one
narrow Ch 31 patch warranted by Decision 3's folded-pilot
shape — the same 3e1ec0ff post-commit-patch pattern Ch 22
applied to Ch 21.

## What Ch 32 inherits from earlier chapters

Ch 30 supplies the three outcomes. Positive: SA reliably
outperforms the matched-complexity RL baseline on the rematch's
task. Null: SA does not outperform the baseline (or
underperforms it). Ambiguous: SA appears to outperform the
baseline but the margin is within the seed-variance envelope —
"a plausible false positive from a favorable seed partition,
exactly the kind of result Henderson warns about." Ch 30 §"What
the chapter does not decide" was explicit that the effect size,
the significance threshold, and the replicate count are deferred
"to chapter 23 / 24 questions, and the answers depend on
measured variance from a small pilot run that does not yet
exist." That deferral is what this chapter resolves.

Ch 23 picked the rematch pool (`{CEM, SA}`, with PPO excluded
under the same logic that excluded TD3 and SAC, and matched
complexity defined as `LinearPolicy(2, 1)` with `n_params = 3`,
the D2c CEM baseline anchor enforceable as a one-line equality
assertion in the rematch test gate). Ch 23 also picked the
replicates API: `run_replicates(tasks, builders, seeds)` as the
general entry point with `run` as a one-line wrapper, returning
a `Result<CompetitionResult, EnvError>` whose `runs` field is a
flat `Vec<RunResult>` with a `replicate_index` field, sequential
across replicates and tasks, first-failure-aborts. Ch 23 also
recommended (in prose, API stays silent) splitmix64 applied to a
master seed as the per-replicate seed derivation convention.
Ch 32 inherits all three: the rematch runs CEM and SA at matched
complexity, the per-batch loop is
`competition.run_replicates(&tasks, &builders, &seeds)?`, and
the `seeds` slice is derived from a pre-registered master value
via splitmix64 in the form Section 4.6 specifies.

Ch 24 picked the per-replicate metric (every algorithm's
`EpochMetrics::mean_reward` standardized on "mean per-episode
total reward across `n_envs` trajectories", with three train
loops touched at `cem.rs:209`, `td3.rs:490`, and `sac.rs:543`)
and the across-replicate aggregation surface
(`replicate_best_rewards(task, algo) -> Vec<f64>` as the raw
primitive, `describe(task, algo) -> Option<SeedSummary>` as the
minimal convenience, `SeedSummary { n, mean, std_dev }` with
sample std via Bessel's correction). The shape was deliberately
chosen so that any reasonable statistical test could be built on
top of it. Ch 32's test family is built on the raw primitive:
the bootstrap resamples the per-replicate `f64` values directly,
and `SeedSummary` is the print-to-console summary the writeup
uses alongside the CI rather than the data structure the test
operates on.

Ch 24 §4.5 was specifically prescient about one thing this
chapter has to address: "Seed distributions across replicates
are sometimes non-Gaussian — in particular, when a fraction of
seeds collapse to a local optimum and the rest find a better
one, the distribution becomes bimodal and mean/std summarizes it
badly." Ch 24 rejected adding percentile helpers as speculative
chassis overbuild and named Ch 32 as the chapter that would
"measure whether the distribution is bimodal, and add
percentiles if and only if the data demands it." Ch 32's
bimodality contingency answers that hand-off without adding
chassis: the median-based escalation operates on the raw
`Vec<f64>` primitive Ch 24 already shipped, so no new
`CompetitionResult` methods are needed.

Ch 31 enumerated four buckets of rematch failure modes and named
this chapter as the owner of three values: the warmup-overhead
factor in §3.1, the test choice and significance threshold in
§3.2, and the pilot existence in §4.4. The first two are
straightforward picks Ch 32 fills. The third is the load-bearing
prerequisite Ch 31 §4.4 named: "Ch 32's prerequisite is
satisfied by the chapter being drafted and its decisions carried
into the rematch protocol, not by a separate execution PR
merging into `main`. Ch 32 is a study chapter, not an execution
PR chapter, and the pilot it specifies will be run as part of
the rematch's preparation rather than as a separate landing
event." Ch 32's Decision 3 (the folded-pilot scheme) takes
this further than Ch 31 anticipated — there is no separate pilot
at all. Section 7 names the consequent narrow Ch 31 §4.4 patch.

Ch 22 supplied the warmup overhead's numerical anchor: at the
D2c configuration, TD3 and SAC each spend `n_envs * warmup_steps
= 32 * 1000 = 32{,}000` env steps on random exploration before
any gradient update — 0.2 percent of the 16M rematch budget. Ch
22 picked "document and live with it" but flagged the call as
*conditional on the rematch's effect size being large relative
to 0.2 percent* — Ch 22's italicized framing. Ch 31 §3.1 then
handed Ch 32 the job of picking the multiplier that makes
"large relative to" operational. Decision 5 picks `5×`.

## Section 1 — The audit: what's known and unknown about the replicate distribution

### 1.1 What the rematch is testing for

The rematch produces, per algorithm in the pool `{CEM, SA}`,
some number `N` of independent training runs at the matched
complexity Ch 23 specified, each at the same `Steps(16M)` budget
Ch 22 picked. Each run produces a `RunResult` with a per-epoch
metrics history; `RunResult::best_reward()` reduces that history
to a single scalar — the maximum `mean_reward` over the run's
epochs, in the per-episode-total units Ch 24 standardized on. Ch
24's `replicate_best_rewards(task, algo)` returns the
`Vec<f64>` of these scalars across the `N` runs.

The thing the rematch is asking a statistical test to decide is
this: given the two vectors `r_SA: Vec<f64>` and `r_CEM:
Vec<f64>`, each of length `N`, is the difference in their
distributions large enough and consistent enough to support one
of Ch 30's three claims about the underlying populations from
which the seeds were drawn? "Underlying population" is the right
framing because Ch 20 established — and Ch 30 reaffirmed — that
each random seed picks a different starting point in the
algorithm's optimization landscape, and a single seed's outcome
is one sample from the distribution over all possible starting
points. The rematch is estimating the means (or some other
location parameter) of those two seed-population distributions
and asking whether they are reliably different.

This is a two-sample location test in the classical statistical
framing. The candidate test families are well-known. Picking
among them is the work of Section 3.

### 1.2 What is unknown

Three things about the seed-population distributions are
genuinely unknown at the time this chapter is being written, and
will remain unknown until the rematch's first batch of
replicates runs:

1. **Per-replicate variance.** Neither CEM nor SA has been run
   at D2c with multiple seeds in the post-Ch-24 metric. The D2c
   artifact (`sim/L0/thermostat/tests/d2c_cem_training.rs`) is a
   single-seed test, and the metric Ch 24 standardized on did
   not exist when the test was written. Variance estimates from
   the legacy single-seed runs are not transferable.
2. **Distributional shape.** Whether the per-replicate
   `best_reward` distributions are unimodal and roughly
   symmetric (in which case mean-based tests are well-behaved),
   bimodal with a fraction of seeds collapsing to a local
   optimum (in which case mean-based tests summarize badly and
   median-based tests are more robust), or heavy-tailed (in
   which case the standard error is dominated by a few outlier
   seeds and bootstrap is preferable to parametric tests) is
   unknown. Ch 24 §4.5 explicitly named bimodality as a
   plausible shape and deferred the empirical check to this
   chapter.
3. **Effect size.** Whether SA reliably beats CEM at matched
   complexity, reliably loses, or ties is the rematch's whole
   question. Ch 30 framed three plausible outcomes; the actual
   outcome is unknown, and Ch 32 is not allowed to assume any
   of them when picking the test.

### 1.3 What is known

Two things are known well enough to be load-bearing:

1. **Henderson 2018's diagnosis is the right starting point.**
   The Henderson et al. 2018 "Deep Reinforcement Learning that
   Matters" paper established that small-`N` seed comparisons in
   RL are routinely overconfident, that mean-plus-stderr summary
   statistics conflate variance from the algorithm with variance
   from the seeds, and that distribution-free tests are the
   honest move for the regime the rematch sits in. Ch 30 cited
   Henderson directly when introducing the ambiguous outcome
   ("a plausible false positive from a favorable seed partition,
   exactly the kind of result Henderson warns about"). Ch 32
   inherits the diagnosis: the test pick has to be defensible
   under Henderson's critique, not just defensible under
   classical small-sample inference.
2. **Agarwal 2021's recommendation refines it.** The Agarwal et
   al. 2021 NeurIPS paper "Deep Reinforcement Learning at the
   Edge of the Statistical Precipice" went further than Henderson
   and explicitly recommended bootstrap confidence intervals on
   point estimates as the modern empirical-RL evaluation
   standard. The argument is that bootstrap CIs are
   distribution-free, scale to any `N`, give a CI directly
   (which maps cleanly onto reporting conventions), and avoid
   the parametric assumptions that small-`N` t-tests rely on.
   Ch 32's test pick takes Agarwal's recommendation as the
   default and asks whether anything in the rematch's specific
   shape argues against it.

### 1.4 The constraint that does the most work

Across the three unknowns and the two knowns, one observation
constrains the design more than any other: **the test choice is
constrained more by `N` than by distributional shape.** At the
small-`N` regime the rematch sits in (3 to 20 seeds, by Ch 24's
projection and this chapter's picks), no test in the candidate
pool gives tight power, and the gaps between tests are smaller
than the gaps between `N` values. Welch's `t` at `N = 10` with
`df ≈ 18` is barely above qualitative; Mann-Whitney U at `N = 3`
literally cannot reject at `α = 0.05` (the minimum achievable
two-sided p-value with two groups of 3 is `2 / C(6,3) = 0.1`);
bootstrap CI at `N = 10` has slightly anti-conservative
coverage (empirical estimates from the small-sample bootstrap
literature place it near 92–94 percent against nominal 95
percent for moderate distributional shapes) but remains the
only test in the pool that scales monotonically with `N`
without distributional assumptions.

The practical consequence is that picking the test ahead of the
pilot is *less* speculative than picking `N` ahead of the pilot.
Test family is a structural choice that is defensible under any
plausible distributional shape; `N` is a power choice that
genuinely depends on measured variance. Section 3 picks the test
on structural grounds; Section 4 picks an initial `N` and a
deterministic expansion rule that lets the rematch's data drive
the final count.

## Section 2 — How this touches Ch 24 and the inheritance shape

### 2.1 Not a Ch 24 retroactive patch

Ch 24 picked an aggregation surface in deliberate
test-agnosticism: "Chapter 24 gives Chapter 32 a shape that
supports all of the above" (Ch 24 §5, where "the above" listed
mean-plus-one-std, Welch's t, bootstrap confidence interval, and
Mann-Whitney U). The pick this chapter makes — bootstrap CI on
the difference of means, with median-based escalation under
bimodality — is one of the four shapes Ch 24 explicitly named as
supportable, and it builds on the raw `Vec<f64>` primitive
without needing any new `CompetitionResult` methods.

Specifically: bootstrap on means uses
`replicate_best_rewards(task, "SA")` and
`replicate_best_rewards(task, "CEM")` as the input vectors,
draws `B = 10000` paired resamples from each, computes the
difference of means for each resample, and returns the 2.5th and
97.5th percentiles of the resulting bootstrap distribution as
the 95 percent CI. The escalation to medians uses the same two
input vectors and the same resampling procedure with
`np.median` (or the Rust equivalent) substituted for `np.mean`.
Both operate at the consumer level — the test is implementation
that lives in the rematch's analysis code, not in
`CompetitionResult` itself.

The `SeedSummary` convenience struct is *not* the data structure
the test operates on. `SeedSummary` is the print-to-console
summary the rematch writeup uses to report mean and standard
deviation alongside the CI — three numbers per algorithm in the
table caption, distinct from the CI bounds. The bootstrap test
ignores `SeedSummary` and reads the raw vectors directly. Ch 24's
shape supports this cleanly: the `describe` helper exists for
the writeup, the raw primitive exists for the test, and neither
forces a particular statistical method.

### 2.2 The contingency uses primitives Ch 24 already shipped

Ch 24 §4.5 rejected adding percentile or median helpers to
`CompetitionResult` on the grounds that it would be speculative
chassis overbuild ("Option L addresses a real concern... But L
speculates about a problem we do not have evidence we have
yet"). Ch 24 named this chapter as the place the bimodality
question would be answered empirically, and named the median-
based escalation as a one-PR change against the existing
primitive: "the underlying data is already in the raw `Vec`
primitive, so no new storage or collection logic is needed,
only new read helpers."

Ch 32's contingency takes Ch 24 at its word. The escalation does
not add helpers to `CompetitionResult`. It substitutes `median`
for `mean` inside the rematch's bootstrap loop — a change at the
analysis-code level, not the chassis level. The chassis surface
Ch 24 picked is unchanged. Ch 24's prediction that the median
escalation would be "a one-PR change... only new read helpers"
is borne out: the read helper is `slice::sort_by` plus index
arithmetic, ten lines of analysis code, no chassis impact.

### 2.3 Scope discipline for Section 2

This chapter does not retroactively patch Ch 24. The aggregation
surface is exactly the right shape for the test family Ch 32
picks. The two chapters are corroborating from different angles:
Ch 24 picked a shape on chassis-minimalism grounds without
knowing which test would consume it; Ch 32 picked a test on
small-`N` defensibility grounds and finds that Ch 24's shape is
exactly what the test needs. The agreement is structural, not
coincidental — Ch 24 §4.6 picked sample std with Bessel's
correction on convention-matching grounds rather than
test-specific grounds, which is the right move for any
chapter that hands off to an unknown downstream test, and Ch 32
inherits a `std_dev` field that matches the convention bootstrap
analyses report alongside CIs.

## Section 3 — Decision 1: test family and significance threshold

### 3.1 The candidate pool

Five test families are plausible candidates for the rematch's
two-sample comparison:

1. **Mean ± 1σ overlap (Henderson-style legacy).** Compute
   `mean(r_SA) ± std_dev(r_SA)` and `mean(r_CEM) ± std_dev(r_CEM)`;
   declare a positive result if the intervals do not overlap.
   Common in legacy RL papers. Maps directly onto `SeedSummary`'s
   three fields.
2. **Welch's t-test.** Two-sample t-test with unequal variances.
   Parametric, assumes per-replicate values are roughly
   normally distributed. Returns a p-value.
3. **Mann-Whitney U.** Rank-based two-sample test, distribution-
   free. Tests for stochastic dominance: `P(X > Y) > 0.5`.
4. **Bootstrap CI on the difference of means.** Distribution-
   free. Resample with replacement from each input vector,
   compute the difference of means for each pair of resamples,
   take the 2.5th and 97.5th percentiles of the resulting
   bootstrap distribution.
5. **Permutation test (exact two-sample).** Enumerate (or
   randomly sample) all possible label permutations of the
   pooled data, compute the difference of means for each, and
   compute the proportion of permutations with a difference at
   least as extreme as the observed one.

### 3.2 The pick: bootstrap CI on the difference of means

Concrete specification:

```rust
fn rematch_test(
    r_sa: &[f64],     // replicate_best_rewards(task, "SA")
    r_cem: &[f64],    // replicate_best_rewards(task, "CEM")
    rng: &mut impl Rng,
) -> RematchResult {
    const B: usize = 10_000;
    let n_sa = r_sa.len();
    let n_cem = r_cem.len();

    let mut diffs = Vec::with_capacity(B);
    for _ in 0..B {
        let mean_sa: f64 = (0..n_sa)
            .map(|_| r_sa[rng.gen_range(0..n_sa)])
            .sum::<f64>() / (n_sa as f64);
        let mean_cem: f64 = (0..n_cem)
            .map(|_| r_cem[rng.gen_range(0..n_cem)])
            .sum::<f64>() / (n_cem as f64);
        diffs.push(mean_sa - mean_cem);
    }
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let lower = diffs[(0.025 * B as f64) as usize];
    let upper = diffs[(0.975 * B as f64) as usize];
    let point_estimate = mean(r_sa) - mean(r_cem);

    classify(point_estimate, lower, upper)
}
```

`B = 10000` resamples. This is the standard value in the
modern empirical-RL literature and is sufficient for the Monte
Carlo error in the reported CI bounds to be well below their
rounding precision — at `B = 10000` the standard error on the
2.5th or 97.5th percentile estimate is roughly
`sqrt(0.025 * 0.975 / 10000) ≈ 0.0016`, and the CI bounds are
reported to at most three decimal places, so increasing `B`
further does not change the reported numbers. `B = 1000` would
be marginal and risks visible Monte Carlo noise in the reported
bounds; `B = 100000` is strictly more compute for no gain.

Two-sided 95 percent interval. The 95 percent confidence level
is the field-standard convention in empirical RL; 90 percent
would be more liberal and risk a reader interpretation of
"looks positive" as stronger evidence than it is, while 99
percent would be more conservative and would widen the CI
enough to push realistic rematch effect sizes into the
ambiguous bucket. 95 percent matches Henderson 2018 and
Agarwal 2021 and is what the rematch's audience expects.

Paired resampling — each bootstrap iteration draws `n_sa`
samples with replacement from `r_sa` and `n_cem` samples with
replacement from `r_cem` independently, computes the difference
of means, and contributes one point to the bootstrap
distribution. The final CI is the 2.5th and 97.5th percentiles
of the sorted bootstrap distribution. The point estimate is the
difference of the original sample means (not the bootstrap
distribution's mean, which would be a slightly biased
estimator).

The argument for bootstrap CI on the difference of means rests
on three claims:

1. **It is the modern empirical-RL standard.** Agarwal 2021
   explicitly recommends bootstrap CIs over point estimates and
   over parametric tests for small-`N` RL evaluation. The
   recommendation is widely adopted in current empirical RL
   papers and is the test a contemporary skeptical reader would
   expect to see in the rematch's writeup.
2. **It is distribution-free.** Bootstrap makes no assumption
   about the per-replicate distribution's shape. The pilot does
   not have to vouch for normality before the test can be
   trusted; the test works under any distributional shape that
   the resample-with-replacement procedure can capture, which
   includes the bimodal and heavy-tailed shapes Ch 24 §4.5
   flagged as plausible.
3. **It scales monotonically with `N`.** Adding more replicates
   tightens the bootstrap CI in the expected way — the CI width
   shrinks roughly as `1 / sqrt(N)`. Within the rematch's `N`
   range (`N = 10` and `N = 20`), there is no value below which
   the test silently breaks (unlike Mann-Whitney U at `N = 3`),
   and there is no value above which the test starts making
   unjustified parametric claims (unlike Welch's t at any `N`).
   Outside the rematch's range, bootstrap is genuinely
   degenerate at `N ≤ 2` — there are only four distinct
   resamples at `N = 2`, and a single degenerate point at
   `N = 1` — but the rematch never operates in that regime. The relationship between `N` and CI width is the same
   relationship the rematch's expansion rule (Decision 3) reads,
   so the test family and the protocol are mutually compatible
   by construction.

The third claim is the load-bearing one for this chapter. The
pre-registered expansion from `N = 10` to `N = 20` only makes
sense if the test continues to behave the same way at the
expanded `N`. Bootstrap CI does. A test that changed its
distributional assumptions or its rejection region between
`N = 10` and `N = 20` would create a multiple-testing concern at
the expansion boundary; bootstrap does not.

### 3.3 Significance threshold and Ch 30 mapping

The 95 percent CI gets mapped onto Ch 30's three outcomes by the
following deterministic rule:

| CI lower | CI upper | Point est | Ch 30 outcome |
|---|---|---|---|
| `> 0` | `> 0` | `> 0` | **Positive** |
| `< 0` | `< 0` | `< 0` | **Null** |
| `≤ 0` | `> 0` | `> 0` | **Ambiguous** |
| `≤ 0` | `> 0` | `≤ 0` | **Null** |
| `≤ 0` | `≤ 0` | `≤ 0` | **Null** |

The "positive" bucket requires the CI to be strictly above zero.
The "null" bucket covers two distinct shapes: SA reliably loses
(CI strictly below zero) and SA does not appear to win (CI
straddles zero with point estimate at or below zero). The
"ambiguous" bucket is the narrow region where the point estimate
favors SA but the CI cannot rule out the null — this is the
rematch's "interesting but underpowered" region and the only one
that triggers the Decision 3 expansion to `N = 20`.

The rule deliberately collapses the SA-loses case into "null"
rather than splitting it into "null-loss" and "null-tie". Ch 30's
framing names three outcomes, and a four-way classification
would invite a hedged fourth-option reading that Ch 31 §3.2
explicitly named as a failure mode ("a 'weak positive' or
'near-null' hedge is not on the menu"). The mapping has three
buckets and no fourth. A reader who wants the loss-vs-tie
distinction can read the point estimate directly — it is in the
writeup's table — but the rematch's headline claim is one of the
three Ch 30 outcomes.

### 3.4 Counterfactual: why not mean ± 1σ overlap

Mean ± 1σ overlap is the simplest test in the candidate pool and
the one that legacy RL papers most often report. Its appeal is
that it maps directly onto `SeedSummary`'s three fields without
any additional computation, and that a reader without
statistical background can interpret it visually as "the error
bars overlap, so the difference is not real."

The case against it is that the overlap rule has no significance
interpretation and is *less* powerful than the parametric
alternatives it superficially resembles. Two intervals that
overlap by a small margin can correspond to a difference of
means whose 95 percent CI excludes zero — the overlap rule
rejects in this case, but Welch's t and bootstrap CI both
correctly identify the difference as significant. The asymmetry
goes the other way too: two intervals that just barely fail to
overlap can correspond to a difference whose CI straddles zero.
Henderson 2018 named this rule specifically as one of the
practices the field needed to move past, and Agarwal 2021 went
further and recommended replacing it outright with bootstrap
CIs. Adopting it for the rematch would be choosing a 2015-era
convention over a 2021-era convention with no strong argument
for the older one. Rejected.

### 3.5 Counterfactual: why not Welch's t

Welch's t is the parametric workhorse for small-sample two-group
comparisons. It is more powerful than bootstrap when the
underlying data is normally distributed, by a margin of perhaps
5 to 15 percent at `N = 10` per group. The case for it is that
"more powerful" is real and that at the small `N` the rematch
will use, every percentage point of power matters.

The case against it has two parts. First, the normality
assumption is invisible to a reader who has not checked it. Ch
24 §4.5 explicitly named bimodality as a plausible distributional
shape for RL seed populations, and a Welch's t result on a
bimodal distribution is not just "less powerful than it should
be" — it can be actively misleading, because the t-statistic's
denominator is the pooled sample standard deviation, which
overstates the true spread of a bimodal distribution and
under-rejects compared to the bootstrap that operates on the raw
samples. A skeptical reader of the rematch writeup would have to
re-check the distributional assumption manually and the rematch
provides no machinery for them to do so cheaply. Bootstrap CI
ducks this concern entirely.

Second, the 5 to 15 percent power gap that favors Welch's t
under normality is much smaller than the gap between `N = 10`
and `N = 20` for either test. The expansion rule in Decision 3
buys back more power than the test choice loses by switching
from t to bootstrap, and it does so without committing to a
distributional assumption. The trade is structurally favorable
for bootstrap. Rejected.

### 3.6 Counterfactual: why not Mann-Whitney U

Mann-Whitney U is the rank-based nonparametric workhorse and is
robust to outliers and distributional shape in a way that mirrors
bootstrap's robustness. The case for it is that it is even more
distribution-free than bootstrap (it does not even need the
sample to be representative of the population in the
moment-matching sense bootstrap implicitly assumes) and that it
has well-known power characteristics for small `N`.

Three arguments against. First, Mann-Whitney U tests stochastic
dominance — `P(X > Y) > 0.5` — rather than difference of
location parameters. This is a different question from "does SA
have a higher mean reward than CEM?", which is the question Ch
30 framed and the question the rematch's writeup will report.
The two questions agree under unimodal symmetric distributions
but can diverge under heavy tails or skewness, and the
divergence is hard to communicate to a reader who is reading the
writeup as "is one method better than the other in expectation?"
Bootstrap on the difference of means answers exactly the
expectation question.

Second, Mann-Whitney U at `N = 3 vs N = 3` literally cannot
reject at `α = 0.05` (the minimum achievable two-sided p-value
is `2 / C(6,3) = 0.1`). At `N = 10 vs N = 10` the floor lifts,
but at `N = 5 vs N = 5` the minimum achievable p-value is
`2 / C(10,5) = 0.0079`, which is fine for rejection but means
the test has a discrete granularity that interacts oddly with
expansion. A test whose p-value space is discrete creates an
expansion boundary that bootstrap's continuous CI does not.

Third, Mann-Whitney U does not produce a confidence interval
naturally — it produces a p-value or an effect size estimate
(the "common language effect size"). Reporting "p = 0.04, U =
26" in the rematch writeup is less informative than reporting "SA
- CEM = 0.087, 95% CI [0.012, 0.158]", and the former is the
reporting format Ch 30's three outcomes are framed against.
Rejected.

### 3.7 Counterfactual: why not permutation test

Permutation test is the same nonparametric family as bootstrap
and is, in some senses, more rigorous: it produces an *exact*
p-value (under the sharp null hypothesis of identical
distributions) rather than the asymptotic CI bootstrap returns.
At `N = 10 vs N = 10`, the exact permutation count is
`C(20, 10) = 184{,}756`, fully tractable. At `N = 20 vs N = 20`
the count is `C(40, 20) ≈ 1.4 * 10^11`, which requires Monte
Carlo permutation sampling rather than exact enumeration but is
still cheap.

The case against permutation is narrower than the cases against
mean-overlap, Welch's t, and Mann-Whitney U: it is genuinely a
defensible test family for the rematch. The argument for picking
bootstrap over permutation is that the rematch's reporting
format is "point estimate ± CI", not "p-value", and bootstrap
produces a CI directly while permutation produces a p-value
directly. The two can be made to agree (a permutation test can
be inverted to produce a CI; a bootstrap CI can be tested at the
α boundary), but the natural output of each is different and the
rematch's reporting convention favors bootstrap's natural output.

A secondary argument: permutation tests the sharp null
hypothesis (the two distributions are identical), which is a
stronger null than the difference-of-means null bootstrap tests.
Under heavy tails, a permutation test can reject for reasons
that have nothing to do with the difference of means (it can
reject because the variances differ even if the means are
identical). Bootstrap on the difference of means tests exactly
the difference-of-means null and does not pick up
variance-difference rejections as false positives for the
location question.

This is a closer call than the other counterfactuals. Permutation
is a defensible alternative; bootstrap wins on output-format
match and on null-hypothesis specificity. Rejected, but not as
firmly as the other three.

### 3.8 Sub-decision: percentile bootstrap vs BCa bootstrap

The bootstrap procedure in Section 3.2 is the *percentile*
bootstrap — take the 2.5th and 97.5th percentiles of the
resampled distribution directly as the CI bounds. The modern
statistical alternative is the *BCa* (bias-corrected and
accelerated) bootstrap, which adjusts the percentile cutoffs by
estimating the bootstrap distribution's bias and skew and
correcting for both. BCa is the canonical modern recommendation
in the statistical literature (Efron & Tibshirani 1993, and
subsequent small-sample studies) because percentile bootstrap
can be anti-conservative at small `N` when the bootstrap
distribution is skewed, and the bias-correction step cleans up
the coverage toward the nominal rate.

The case for BCa is real: at `N = 10` per group, the percentile
bootstrap CI is anti-conservative by a few percentage points
(the 92-94 percent empirical coverage claim in Section 1.4),
and BCa would bring the coverage closer to the nominal 95
percent. For a rematch whose headline is a small effect size,
those few percentage points could be the difference between
"ambiguous" and "positive."

The case for percentile bootstrap is reader familiarity and
implementation simplicity. The percentile bootstrap is what
Agarwal 2021 uses in the `rliable` library that is the current
de facto empirical-RL evaluation standard, and it is what a
contemporary RL reader will recognize from recent papers. BCa
is more rigorous but is also one level of statistical
machinery removed from the audience's default mental model.
The implementation is also simpler — percentile bootstrap is
twenty lines, BCa adds roughly forty more lines for the bias
estimate (via the proportion of resamples below the observed
mean) and the acceleration estimate (via jackknife
resampling).

The pick is percentile bootstrap, with two-fold reasoning.
First, the chapter's anti-conservatism concern at `N = 10` is
addressed by the expansion rule: ambiguous results trigger the
N=20 expansion, and at N=20 the percentile bootstrap's coverage
tightens toward nominal. The structural insurance against
anti-conservatism is the expansion, not the CI-computation
method. Second, matching Agarwal 2021's `rliable` convention
gives the rematch writeup a direct comparison to recent
empirical-RL papers, which is exactly the audience the rematch
is addressing. A reader who wants BCa can compute it from the
published replicate values with the same `replicate_best_rewards`
primitive — the raw data is available and the CI machinery is
swappable at the writeup level.

The rejection is on balance-of-arguments grounds: BCa is
marginally better statistically but less transparent to the
audience, and the rematch's primary anti-conservatism insurance
comes from the expansion rule rather than from the CI method.
The chapter names BCa as a defensible alternative that a future
writeup author could substitute at the Ch 42 execution level
without re-opening Ch 32's decisions.

## Section 4 — Decision 2: replicate count protocol and initial batch

### 4.1 The problem

Ch 24 §4.6 projected `N` in the range 3 to 10 and explicitly
deferred the choice to this chapter pending pilot variance data.
The pilot does not exist. Three structural facts make the
choice harder than it would be under a conventional power
analysis:

1. **Variance is unknown**, so a power-based `N` cannot be
   computed up front. Conventional power analysis would say
   "pick `N` such that the test has 80 percent power against the
   smallest effect size of interest"; without variance the
   effect-size-to-`N` mapping is undefined.
2. **Compute is non-trivial.** At the D2c rematch budget of 16M
   env steps per replicate, every additional seed costs 16M env
   steps for SA and 16M env steps for CEM. The total cost grows
   linearly: `N = 5` is 160M env steps total, `N = 10` is 320M,
   `N = 20` is 640M. None of these are blocking, but none are
   free either.
3. **A separate pilot is wasteful.** Running a pilot at full
   per-replicate budget burns compute that could roll into the
   rematch directly. A pilot that uses a *shorter* per-replicate
   budget gives a variance estimate that is biased low (variance
   typically grows over a training run as some seeds find good
   policies and some do not), so the pilot has to use the full
   16M budget to be honest. At the full budget, the pilot's
   replicates are indistinguishable from rematch replicates
   except by labelling.

The third fact suggests a refactoring of the conventional
"pilot then rematch" structure. If pilot replicates and rematch
replicates are computationally identical, they should be the
same replicates.

### 4.2 The pick: folded pilot with pre-registered expansion

Concrete protocol:

1. Pre-register a master seed `MASTER = 20_260_412` and derive
   the initial batch's ten replicate seeds as
   `splitmix64(MASTER.wrapping_add(i))` for `i` in `0..10`, per
   the Ch 23 §1.2 prose recommendation. The master value is
   fixed in this chapter before any replicate runs; the
   splitmix64 derivation is the Ch 23-recommended form that
   avoids the mild correlation structure of raw sequential
   master values.
2. Run `competition.run_replicates(&tasks, &builders, &seeds)?`
   with both `CEM` and `SA` registered in the pool, both at
   matched complexity (`LinearPolicy(2, 1)`, `n_params = 3`), at
   the D2c task, at `Steps(16M)` per replicate. This produces 20
   `RunResult` entries — 10 per algorithm.
3. After the initial batch lands, compute three things from the
   raw `Vec<f64>` values returned by `replicate_best_rewards`:
   - The bootstrap 95 percent CI on the difference of means
     using the Section 3.2 procedure.
   - The bimodality coefficient per algorithm using Section 5's
     formula.
   - The Ch 30 outcome classification (positive / null /
     ambiguous) using Section 3.3's table.
4. Apply the decision rule:
   - If the outcome is **positive** or **null** (CI excludes
     zero in either direction, or CI straddles zero with point
     estimate at or below zero), stop. The rematch's headline is
     the result at `N = 10`.
   - If the outcome is **ambiguous** (CI straddles zero with
     point estimate above zero), expand to `N_final = 20` by
     running an additional ten replicates per algorithm with
     seeds derived as `splitmix64(MASTER.wrapping_add(i))` for
     `i` in `10..20`. After the expanded batch lands, recompute
     the CI and reclassify. The rematch's headline is the result
     at `N = 20` regardless of which bucket it lands in.
5. The bimodality coefficient check (Section 5) applies at
   *every* classification step — both at the initial `N = 10`
   read and at the expanded `N = 20` read. If either algorithm's
   bimodality coefficient exceeds `5/9` at the read, the
   bootstrap test substitutes medians for means before
   classifying. The substitution is mechanical and pre-
   registered.

`N_initial = 10` and `N_final = 20` are the only two replicate
counts the rematch is allowed to use. There is no `N = 15`
intermediate stop, no `N = 30` further expansion. The hard cap
at 20 is what makes the protocol pre-registered rather than
sequential.

### 4.3 How the expansion rule interacts with Type I error

The classical "peeking" concern with sequential analysis is that
checking for significance multiple times and stopping early on
the first significant result inflates the Type I error rate
above the nominal `α` — a 95 percent CI checked at `N = 10` and
again at `N = 20` with "stop and reject if either CI excludes
zero" gives an effective false-positive rate that can approach
`2α` via the union bound, substantially above the nominal rate.

The rematch's rule has a less severe version of this property
than the worst-case peeking rule, but it does *not* have exactly
nominal FPR — that claim would overstate the protocol's
statistical properties. The honest framing is a two-part upper
bound.

First, the protocol's "report positive" one-sided FPR under the
null hypothesis is bounded above by `2α_1 = 5 percent` via the
union bound, where `α_1 = 2.5 percent` is the one-sided nominal
rate. This is the worst case: if the N=10 and N=20 tests were
statistically independent, the "either test rejects" event would
have exactly `2α_1` probability under the null. The bound is
shared with any two-look sequential design that does not apply a
multiple-testing correction.

Second, the protocol's practical FPR is substantially below the
`2α_1` worst case because the N=10 and N=20 tests share data.
The N=20 test uses the original ten replicates plus ten new
ones, so its result is positively correlated with the N=10
result. Under the null, the realizations where the N=10 test
would narrowly reject are the same realizations where the N=20
test is most likely to reject, which means the two rejection
events are not independent — they overlap. The overlap reduces
the combined FPR below `2α_1` without further correction.

A quantitative analysis of exactly how much correlation damps
the bound is beyond this chapter's scope — it would require
simulation under a specific distributional model, and Ch 32
deliberately declines to commit to such a model without pilot
data. What Ch 32 claims is the qualitative behavior: the
practical FPR sits somewhere between `α_1` and `2α_1`, almost
certainly closer to `α_1` than to `2α_1`, and it is *not*
inflated to the degree a naive "stop on significance" peeking
rule would inflate it. A Bonferroni-corrected alternative (use
`α_1 / 2 = 1.25 percent` at each look, equivalently 97.5 percent
CIs) would give a strict `2 * α_1 / 2 = α_1 = 2.5 percent`
combined FPR but at the cost of meaningfully wider CIs at both
stopping points, which is its own form of conservatism. The
uncorrected rule trades a bounded FPR inflation for compute
savings and for CI widths that match reader expectations; the
trade is deliberate and named here rather than obscured.

The two structural reasons the inflation is modest rather than
catastrophic are:

1. **The rule stops on certainty, not on significance.** The
   rematch does not "keep checking until the CI excludes zero"
   — it stops as soon as the CI is unambiguous in *either*
   direction. Under the null, roughly half of the ambiguous-at-
   N=10 realizations have CIs that would tighten around zero at
   N=20 rather than exclude it, so the expansion is as likely
   to confirm the null as it is to find a spurious positive.
2. **The N=10 and N=20 tests share data.** The conditional
   distribution of the N=20 test given the N=10 observations is
   not the unconditional null distribution — it is shifted by
   the N=10 data in a way that positively correlates the two
   tests.

Together these make the rule roughly equivalent to "run one
test at the final N, report it honestly" for most realistic
effect sizes, even though a strict worst-case FPR analysis must
allow for the `2α_1` upper bound.

There is also an intra-batch composition concern worth naming
explicitly. The early-stopping rule applies to whichever test
(mean-based or median-based) the bimodality check selects at
the initial `N = 10` read. If bimodality triggers the median
substitution at `N = 10`, the early-stopping rule still applies
to the median-based test result, and expansion to `N = 20` is
triggered (or not) based on whether the *median-based* CI is
ambiguous. The substitution and the stopping rule are
independent at each classification step — the substitution
decides *which* test runs, and the stopping rule decides
*whether* to expand based on the substituted test's result.

The inter-batch composition concern is subtler. If N=10 is
bimodal (median-based test runs, result is ambiguous, expand)
but N=20 is not bimodal (mean-based test runs on the expanded
data), the rematch's headline switches from median-based to
mean-based between the two reads. The rule here is that the
headline reported is always the test appropriate to the *final*
data's distributional shape: the bimodality check is
re-evaluated at each classification step, and the headline
result is the test verdict on the data at the last step the
protocol ran. A reader who follows the protocol-trace
would see the median-based test at N=10 and the mean-based test
at N=20 as two separate classification steps, and should
understand this as "the distribution at N=20 is not bimodal, so
means are the right location parameter for the headline, while
the N=10 intermediate was bimodal and was appropriately handled
with medians at that point." The headline is the N=20 test's
verdict; how much of the N=10 intermediate gets surfaced in the
writeup alongside the headline is a writeup-format choice
Section 7 declares out of scope. The pre-registration makes
both rules deterministic and recoverable by a skeptical reader
from the published replicate values.

### 4.4 Why `N_initial = 10` and not 5 or 15

Three values were considered for the initial batch: `N = 5`,
`N = 10`, and `N = 15`.

`N = 5` is the smallest value where bootstrap CI is meaningful at
all. The bootstrap distribution at `N = 5` per algorithm has only
`5^5 = 3125` distinct resamples per algorithm and the CI has very
high variance from one sample to another. The CI's empirical
coverage at `N = 5` is reported in the small-sample bootstrap
literature as roughly 88 to 92 percent against nominal 95
percent, depending on distributional shape, which is meaningfully
anti-conservative. Rejected: too small to give the initial
batch a defensible read.

`N = 15` is a value that achieves close-to-nominal coverage
(roughly 93 to 95 percent against nominal 95 percent, per the
same literature) with a single batch and no expansion. The case
for it is that it is "just enough" for bootstrap to behave well
without the bounded FPR inflation an expansion rule introduces.
Rejected on two grounds: the bounded inflation the expansion
rule produces (bounded above by `2α_1` via the union bound, per
Section 4.3, and in practice much smaller due to shared-data
correlation) is a price the chapter accepts in exchange for the
cheaper-when-unambiguous path that `N = 10` with expansion
preserves, and `N = 15` forecloses that path. The expected compute cost under `N = 10`
with expansion is *less* than `N = 15` whenever the rematch's
true effect size lands in either Ch 30 outcome that is not
ambiguous, and the rematch's prior on which outcome is most
likely is uninformative — Ch 30's three outcomes are framed
without prior weighting. Picking `N = 15` would be paying the
full `N = 15` compute cost up front to eliminate a concern that
the expansion rule already eliminates.

`N = 10` is the value that survives both arguments. At `N = 10`,
bootstrap CI has roughly 92 to 94 percent coverage against
nominal 95 percent — slightly anti-conservative but defensible
under the Henderson/Agarwal recommendation, and the expansion to
`N = 20` brings coverage back close to nominal in the
ambiguous-result case where the additional precision matters
most. The compute cost in the unambiguous case is 320M env
steps; in the ambiguous case it is 640M env steps. The
expectation across both cases is between the two.

### 4.5 Why `N_final = 20` and not 30 or unbounded

`N_final = 20` is the hard cap on expansion. Three reasons.

First, the marginal coverage improvement from `N = 20` to
`N = 30` is small. Empirical estimates for the percentile
bootstrap at `N = 20` place the coverage in the 94 to 95
percent range against nominal 95 percent — within rounding
distance of the nominal rate. Expanding past 20 buys diminishing
returns relative to the linear compute cost.

Second, at `N = 20` the expanded compute cost is already 640M
env steps for the rematch alone. Expanding past 20 starts
making the rematch a non-trivial fraction of the project's
compute budget for reasons that no longer pay back in
statistical precision.

Third, an unbounded expansion ("keep adding seeds until the CI
excludes zero") *is* the classical peeking failure mode. Section
4.3's argument that the bounded expansion rule does not inflate
α relies on the existence of the hard cap. Without the cap, the
rule degenerates into "stop when significant", which inflates α
exactly as the peeking literature warns.

The hard cap is what makes the pre-registration honest. If the
result at `N = 20` is still ambiguous, the rematch reports
"ambiguous" — Ch 30 explicitly named this as one of the three
informative outcomes, and an ambiguous result at `N = 20` is
itself a finding (specifically: the effect is small enough
relative to the seed-population variance that the rematch as
specified cannot resolve it, which is a real signal about the
rematch's design and a prompt to run a different rematch
differently rather than a failure of this one).

### 4.6 Sub-decision: master seed and per-replicate derivation

The pre-registered master value is the literal
`MASTER = 20_260_412`, and the per-replicate seeds are derived
as `splitmix64(MASTER.wrapping_add(i))` for `i` in `0..10` for
the initial batch and `i` in `10..20` for the expansion. Three
reasons.

First, the `MASTER` literal is visible in this chapter and in
the rematch's writeup, so a future reader can reproduce the
exact replicates the rematch ran by applying the Ch 23-documented
splitmix64 function to the same master value and index range.
Reproducibility is preserved without losing the stream-
independence properties of a proper PRF-based seed derivation.

Second, the splitmix64 derivation is the form Ch 23 §1.2
explicitly recommended in prose: "A simpler convention —
`(master..master + n)` — also works for most practical purposes
but has a mild correlation structure that can surprise readers;
splitmix64 is the conservative choice. The study's recommendation
is splitmix64 in prose and the API stays silent." Ch 32 follows
Ch 23's prose recommendation rather than contradicting it; there
is no chapter-level reason the rematch should prefer the form
Ch 23 cautioned against.

Third, the derivation is pre-registered — both the `MASTER`
literal and the splitmix64 derivation rule are fixed in this
chapter before any replicate runs. A reader who is suspicious
that the rematch's seeds were chosen post-hoc to favor a
particular outcome can verify that the seeds the writeup reports
match `splitmix64(20_260_412 + i)` for the claimed `i` range.
This is the cheap version of pre-registration and it costs
nothing.

Two alternatives were considered and rejected.

The raw sequential form `[0, 1, 2, ..., 9]` is the simplest
possible pre-registration and maximally visible in the writeup
(a reader does not even have to run splitmix64 to reproduce the
seeds). The rejection is direct: Ch 23 §1.2 specifically flagged
`(master..master+n)` as having "a mild correlation structure
that can surprise readers" and recommended splitmix64 as the
conservative alternative. Picking raw sequential integers would
be Ch 32 contradicting Ch 23's explicit prose recommendation
without a chapter-level reason to override it. Rejected.

Deriving seeds from a hash of some semantically meaningful
string (`hash("ml_chassis_refactor_rematch_2026")` or similar)
was the second rejected alternative. A cryptographic hash of an
arbitrary string produces a uniformly-distributed `u64` that is
essentially equivalent to a raw master seed for the purpose of
downstream splitmix64 derivation, but it is one additional step
removed from the reader's ability to reproduce the value in
their head. Splitmix64 applied to a fixed integer master is
cleaner and is what Ch 23 recommended. Rejected.

The `MASTER = 20_260_412` value itself has no load-bearing
properties. Any fixed integer value would serve the
pre-registration role equally well. `20_260_412` is chosen to
match Ch 23's own example callsite literal (Ch 23 §1.2 uses
`let master: u64 = 20_260_412;` in its recommended-convention
code block), on the consistency grounds that a reader who
learned the splitmix64 convention in Ch 23 should see the same
master literal in Ch 32 rather than having to memorize two.

### 4.7 Sub-decision: paired vs unpaired bootstrap

The bootstrap procedure in Section 3.2 draws resamples from
`r_sa` and `r_cem` independently. This is unpaired (or
"two-sample") bootstrap, appropriate when the two algorithms'
replicates are not naturally paired by some shared structure.

The alternative is paired bootstrap, where each bootstrap
iteration draws a single index `i` from `0..N` and uses
`(r_sa[i], r_cem[i])` as a paired observation. Paired bootstrap
is more powerful when the two algorithms' results are correlated
across seeds — that is, when seeds that are favorable for one
algorithm are also favorable for the other.

The rematch does not naturally pair CEM and SA replicates. The
two algorithms have different per-seed dynamics (CEM samples a
population of policies and updates a Gaussian; SA perturbs a
single policy and accepts or rejects), and the master seed
controls the RNG for each algorithm independently. There is no
reason to expect that "the seed that gave CEM a high-reward
trajectory" would also give SA a high-reward trajectory, and a
paired test would impose a structural assumption the data does
not support.

Unpaired bootstrap is the right pick. Locked.

### 4.8 What the rematch protocol gets

The rematch's test code looks roughly like:

```rust
const MASTER: u64 = 20_260_412;

fn derive_seeds(range: Range<u64>) -> Vec<u64> {
    range.map(|i| splitmix64(MASTER.wrapping_add(i))).collect()
}

fn run_rematch(
    competition: &Competition,
    tasks: &[Task],
    builders: &[AlgorithmBuilder],
) -> Result<RematchOutcome, EnvError> {
    let initial_seeds = derive_seeds(0..10);
    let result = competition.run_replicates(tasks, builders, &initial_seeds)?;

    let r_sa = result.replicate_best_rewards("d2c", "SA");
    let r_cem = result.replicate_best_rewards("d2c", "CEM");

    let outcome = test_and_classify(&r_sa, &r_cem);
    if matches!(outcome, RematchOutcome::Ambiguous) {
        let expansion_seeds = derive_seeds(10..20);
        let expanded = competition.run_replicates(tasks, builders, &expansion_seeds)?;

        let r_sa_full = [r_sa, expanded.replicate_best_rewards("d2c", "SA")].concat();
        let r_cem_full = [r_cem, expanded.replicate_best_rewards("d2c", "CEM")].concat();

        Ok(test_and_classify(&r_sa_full, &r_cem_full))
    } else {
        Ok(outcome)
    }
}

fn test_and_classify(r_sa: &[f64], r_cem: &[f64]) -> RematchOutcome {
    let bc_sa = bimodality_coefficient(r_sa);
    let bc_cem = bimodality_coefficient(r_cem);

    if bc_sa > 5.0 / 9.0 || bc_cem > 5.0 / 9.0 {
        bootstrap_diff_medians(r_sa, r_cem).classify()
    } else {
        bootstrap_diff_means(r_sa, r_cem).classify()
    }
}
```

The skeleton is roughly fifty lines of analysis code. The
chassis surface (`competition.run_replicates`,
`replicate_best_rewards`) is exactly what Ch 23 and Ch 24
shipped. The bimodality contingency, the bootstrap procedure,
and the classification rule all live in the rematch's analysis
module rather than in `CompetitionResult`. Ch 32 owns the
specification; Ch 42 owns the implementation as part of the
rematch execution PR.

## Section 5 — Decision 3: warmup-overhead factor

### 5.1 What Ch 31 §3.1 owed Ch 32

Ch 22 picked "document and live with it" for the TD3/SAC warmup
overhead — `n_envs * warmup_steps = 32 * 1000 = 32{,}000` env
steps of random exploration before any gradient update, 0.2
percent of the 16M rematch budget at the D2c configuration. Ch
22 named the pick *conditional on the rematch's effect size
being large relative to 0.2 percent* — the italicized qualifier
is Ch 22's own language — and stated that the condition is
"load-bearing", with the sense that "if the rematch's curves
are separated by (say) 0.5 percent, the 0.2 percent overhead is
40 percent of the effect and the decision is not defensible."
Ch 31 §3.1 turned the conditional into a gate: "the gate has to
bound the rematch's effect size away from the warmup overhead,
and if the gate cannot be met the decision returns here and the
fix stops being out of scope." Ch 31 §3.1 then deferred the
specific multiplier ("the predetermined factor") to this chapter.

### 5.2 The pick: 5×

The rematch's effect size must exceed `5 × 0.2% = 1.0%` of the
budget-equivalent reward scale for the warmup gate to clear. If
the rematch's headline result is a difference of means smaller
than 1 percent of the budget-equivalent reward scale, the
warmup overhead is large enough relative to the effect that the
"document and live with" call fails and the rematch's headline
returns to Ch 22's option set (deduct the warmup from the
budget, or report the warmup as a separate field, or extend the
budget to compensate).

`5×` is the conventional "rules of thumb" multiplier in
empirical work for "this is real, not noise floor." The
Henderson/Agarwal line of empirical-RL critique uses similar
order-of-magnitude separations when discussing noise floors.
The number is defensible without being arbitrary: it is large
enough that a rematch effect comfortably above the warmup floor
is unambiguously above it, and small enough that the gate does
not foreclose effects that are real but modest.

### 5.3 Why the gate is currently moot

Ch 31 §3.1 already noted that the gate is moot for the
post-Ch-23 `{CEM, SA}` rematch pool — neither CEM nor SA has a
warmup phase, so the 0.2 percent floor does not apply to either
algorithm in the current pool. Ch 31 kept the entry in its index
on forward-compatibility grounds: "The gate's *number* (0.2
percent at the 16M D2c budget) is specific to the current
rematch configuration; the gate's *shape* (effect size must
exceed unbudgeted-overhead floor by a predetermined factor
before the 'live with it' call survives) is the protective
mechanism Ch 31 inherits from Ch 22 and passes forward to Ch 32."

Ch 32's pick is the multiplier the gate's shape uses. The pick
is fixed at `5×` for all forward extensions of the rematch
that introduce a similar overhead. A future rematch that adds
TD3 or SAC back into the pool, or that changes the per-algorithm
budget in a way that makes the warmup a larger fraction of the
total, applies the `5×` multiplier to the new floor and gates the
result accordingly. The multiplier is a property of the
rematch's design discipline, not of the current pool's
instantiation.

### 5.4 Counterfactual: why not 3× or 10×

`3×` is the smallest multiplier that gives a "comfortably above"
read. The case for it is that it is the least restrictive option
that still rules out near-noise effects. The case against it is
that at `3×` the gate clears for effects that are only
marginally above the floor — a rematch effect of 0.6 percent
clears the gate but is also genuinely close to the noise level,
and a reader who sees the cleared gate may overweight the result.
Rejected as too permissive.

`10×` is the most restrictive multiplier in the conventional
range. The case for it is that it forecloses any reading where
the effect is within an order of magnitude of the floor. The
case against it is that 10× is overcautious for a gate that is
already moot for the current pool. Picking 10× would foreclose
effects of 1 to 2 percent that are perfectly real and that the
rematch should be able to report as positive findings. Rejected
as too restrictive.

`5×` is the value that survives both arguments. It is the
smallest multiplier that gives a "comfortably above" read
without being either marginally permissive or unnecessarily
restrictive.

## Section 6 — The bimodality contingency

### 6.1 The shape of the failure mode

Ch 24 §4.5 named bimodality as a plausible distributional shape
for the rematch's per-replicate distributions: "in particular,
when a fraction of seeds collapse to a local optimum and the
rest find a better one, the distribution becomes bimodal and
mean/std summarizes it badly." A bimodal distribution has two
peaks; the mean lands between the peaks, and the standard
deviation looks enormous because it captures the gap between
the modes rather than the spread within either mode.

If the rematch's per-replicate distributions are bimodal, the
mean-based bootstrap test in Section 3.2 is operating on the
wrong location parameter. The mean is summarizing the
distribution badly, and a CI on the difference of means is
estimating the difference between two badly-summarized
quantities. The right move when bimodality is detected is to
switch to a location parameter that handles bimodality better —
the median — and re-run the same bootstrap procedure with
medians substituted for means.

### 6.2 The detection criterion: the bimodality coefficient

The bimodality coefficient is a small-sample formal statistic
for detecting bimodality, rooted in Pearson's 1916 mode relation
between skewness and kurtosis and given in its small-sample-
corrected form by SAS as:

```
BC = (g² + 1) / (κ + (3 (n-1)²) / ((n-2)(n-3)))
```

where `g` is the sample skewness, `κ` is the sample excess
kurtosis, and `n` is the sample size. The textbook threshold for
"bimodal or strongly skewed" is `BC > 5/9 ≈ 0.556`, which is the
BC value of the uniform distribution — values above this cutoff
indicate that the distribution is more bimodal than a uniform
distribution, which is the conventional cutoff for "the mean is
no longer a good location summary." (Note: this is distinct
from Hartigan's dip test, which is a different small-sample
unimodality statistic; Ch 32 uses the Pearson/SAS bimodality
coefficient, not the dip test.)

`BC` is computable from the raw `Vec<f64>` in roughly twenty
lines of code. It needs `n ≥ 4` to be defined (the formula
divides by `(n-2)(n-3)`), which `N = 10` and `N = 20` both
satisfy. The threshold is mechanical: the rematch's analysis
code computes `BC` for both algorithms' replicate vectors, and
if either strictly exceeds `5/9`, the bootstrap test
substitutes medians for means. The strict inequality handles
the boundary case `BC = 5/9` exactly (which is vanishingly
unlikely under any realistic distribution but deserves an
explicit rule rather than a handwave): exactly-uniform BC
keeps the mean-based test, since the threshold is "more bimodal
than uniform", not "at least as bimodal as uniform".

### 6.3 Why "either algorithm" rather than "both algorithms"

The escalation triggers if *either* algorithm's bimodality
coefficient exceeds the threshold, not only if both do. The
reasoning is that the difference of means is wrong as a location
parameter whenever *one* of its components is summarizing
badly — there is no reading where "one algorithm is bimodal but
the other is not, so the difference of means is fine for the
other one" makes sense, because the difference is computed by
subtracting one (badly-summarized) mean from another (well-
summarized) mean, and the result is dominated by the badly-
summarized term.

The asymmetric trigger is the safer pick. It escalates more
often than a "both must be bimodal" rule, which means the
median-based test runs in some cases where the mean-based test
would also have been fine. The cost is small: the median-based
test has slightly less power than the mean-based test under
truly unimodal distributions, by maybe 5 to 10 percent for `N`
in the rematch's range. The benefit is that the rematch never
reports a result based on a wrong location parameter. The trade
favors the asymmetric trigger.

### 6.4 Why the escalation is pre-registered as a deterministic rule

The bimodality check produces a number; the threshold is fixed;
the substitution is mechanical. There is no human judgment at
the escalation point. A reader of the rematch's writeup who
wants to verify the escalation can recompute `BC` from the
published replicate values, compare to `5/9`, and check that the
test the writeup reports is the test the rule prescribes.

The alternative — "compute `BC`, then make a judgment call about
whether the bimodality is severe enough to escalate" — is
exactly the kind of post-hoc analyst judgment that the
pre-registration of the rest of the protocol is designed to
foreclose. Pre-registering the rule before the data is in hand
is what makes the contingency honest. Locked.

### 6.5 What the median-based test looks like

The substitution is one line at the bootstrap test site. The
mean version computes `mean(resample_sa) - mean(resample_cem)`
for each bootstrap iteration; the median version computes
`median(resample_sa) - median(resample_cem)`. Everything else
about the procedure — `B = 10000`, paired resampling, two-sided
95 percent CI, the Section 3.3 classification rule — is
identical. The Ch 30 mapping is unchanged.

The median-based CI is interpreted in the writeup as "the 95
percent CI on the difference of medians" rather than "the 95
percent CI on the difference of means", and the writeup names
the bimodality detection that triggered the substitution. A
reader who sees the median-based reporting can verify that the
escalation was rule-driven by recomputing `BC` from the
published replicate values.

## Section 7 — What Chapter 32 does not decide

Chapter 32 picks the test family, the significance threshold, the
replicate count protocol, the initial-batch composition, the
warmup-overhead factor, and the bimodality contingency
mechanism. It deliberately declines the following:

- **The precise RNG sequence used to draw bootstrap resamples.**
  The rematch's analysis code uses some `rand::Rng`
  implementation seeded with some master value. Ch 32 does not
  pick the implementation (`StdRng`, `ChaCha8Rng`, etc.) or the
  seed for the bootstrap RNG. The pick is reproducibility-
  irrelevant for `B = 10000` resamples — Monte Carlo error in
  the CI is well below the rounding precision of the published
  result — but the rematch writeup should name the bootstrap RNG
  seed for full reproducibility. That is a Ch 42 execution
  detail, not a Ch 32 design pick.
- **The I/O format of the rematch writeup's CI plot.** Whether
  the rematch reports the CI as a plot, a table row, or both;
  whether the plot uses a violin shape, a box-and-whisker, or
  a forest plot; whether the writeup includes the bootstrap
  distribution itself or only the CI bounds — all writeup
  presentation choices that belong to the writeup author, not
  to this chapter.
- **The wall-clock budget the rematch's compute will consume.**
  Ch 32 specifies env-step counts (320M unambiguous, 640M
  expanded), not wall-clock time. The wall-clock cost depends on
  the chassis's per-step cost, the parallelism configuration,
  and the hardware the rematch runs on, none of which are
  Ch 32's concern. Ch 22 was deliberate about specifying budget
  in env steps rather than wall-clock for exactly this reason.
- **Whether the rematch should be re-run periodically as the
  chassis evolves.** The rematch is a one-time scientific
  question about the post-Ch-40/41/42 chassis state; Ch 32 does
  not decide whether to repeat the rematch under future chassis
  changes. A future re-run would re-apply this chapter's
  protocol with possibly different seed values (the
  pre-registered values protect *this* rematch's
  reproducibility, not all future rematches').
- **Whether the rematch should also report secondary metrics
  alongside `best_reward`.** Ch 24 standardized `mean_reward`
  and `RunResult::best_reward()` is the per-replicate scalar.
  Whether the rematch writeup should also report
  `RunResult::final_reward()`, the full epoch-by-epoch reward
  curve, or other secondary metrics is a writeup author choice.
  Ch 32's protocol operates on `best_reward` only.
- **Whether the bimodality coefficient should also be reported
  in the rematch writeup independently of whether it triggered
  the escalation.** A writeup that reports `BC = 0.31` for both
  algorithms gives a reader confidence that the unimodal
  assumption held; a writeup that omits the `BC` value reports
  the test result without the escalation context. Ch 32's
  recommendation (in prose, not as a binding rule) is that the
  writeup report `BC` for both algorithms in the table caption
  alongside `n`, `mean`, `std_dev`. The choice is a writeup
  author call.
- **Whether the test should be re-run with different `B`
  values as a sensitivity check.** `B = 10000` is sufficient for
  Monte Carlo error to be below the rounding precision of the
  published CI. A sensitivity check at `B = 100000` would
  produce essentially the same CI bounds and would not be
  informative. Ch 32 does not require it; a writeup author who
  wants the sanity check is free to add it but it is not part
  of the protocol.
- **The execution layer code that implements this protocol.**
  Ch 42 (Part 4 PR 3, sim-opt split and rematch) owns the
  implementation. Ch 32 specifies the protocol in prose and
  pseudocode; Ch 42 writes the actual analysis code, picks the
  module structure, and lands the rematch as a runnable PR.
  The skeleton in Section 4.8 is illustrative, not binding —
  Ch 42 may choose a different code organization as long as the
  protocol's behavior matches.
- **The `None`-replicate degenerate case.** Ch 24 §4.7
  documented that `replicate_best_rewards` silently filters out
  `None` values — replicates whose `RunResult::best_reward()`
  returned `None` because the run had zero epochs. For the
  rematch pool `{CEM, SA}`, this case is not a practical
  concern: CEM always produces at least one finite epoch under
  `Steps(16M)`, and SA is a to-be-implemented algorithm whose
  metric-reporting contract Ch 42 specifies alongside the
  implementation. Ch 32 assumes all 10 (or 20) replicates
  return a finite `best_reward`. If the assumption breaks at
  rematch time — for example, a transient infrastructure
  failure causes a replicate to produce no metrics — the
  `SeedSummary.n` field and the bootstrap sample size will be
  smaller than the seed count, and the protocol's statistical
  guarantees degrade proportionally. Ch 42's rematch
  implementation is responsible for detecting the case and
  either retrying or reporting the degradation; Ch 32's
  protocol is silent on the recovery mechanism.
- **Whether BCa or another bias-corrected bootstrap should
  replace percentile bootstrap at the writeup level.** Section
  3.8 named BCa as a defensible alternative and picked
  percentile bootstrap on balance. A writeup author who wants
  to substitute BCa at publication time can do so using the
  raw `replicate_best_rewards` primitive; Ch 32 does not
  require it but does not forbid it, provided the substitution
  is named in the writeup's methods section.
- **The narrow Ch 31 §4.4 patch warranted by Decision 2's
  folded-pilot shape.** Ch 31 §4.4 currently describes the
  pilot as a separate pre-rematch event ("the pilot has been
  run, the variance has been measured"). Under the folded-pilot
  scheme this chapter picks, the pilot is the first batch *of*
  the rematch, and the prerequisite Ch 31 §4.4 names is
  satisfied by the protocol implementing the initial-batch read
  with its pre-registered escalation rule rather than by a
  separate pilot run. Ch 31 §4.4 is narrow enough that a
  3e1ec0ff-style post-commit patch is the right fix — the same
  pattern Ch 22's findings applied to Ch 21. Ch 32 names the
  patch as warranted but does not write the patch itself; the
  patch lands in a separate post-Ch-32 commit.

Chapter 32 owns five decisions and one contingency. Everything
else is deferred to downstream chapters or to writeup author
choices, with explicit pointers. The chapters Ch 32 indexes —
Ch 22, Ch 23, Ch 24, Ch 30, Ch 31, Ch 42 — already made every
decision Ch 32 inherits, and Ch 32's contribution is the picks
that make Ch 30's three outcomes operationally testable from the
chassis surface Ch 24 shipped.
