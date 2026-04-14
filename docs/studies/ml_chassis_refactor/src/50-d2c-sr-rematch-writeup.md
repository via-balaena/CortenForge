# The D2c SR rematch: results

This chapter reports what the rematch found. Chapters 30 through 32
argued what the rematch was asking and how it would be decided;
chapters 40 through 42 built the chassis and the fixture that would
run it; this chapter renders the output of the one run that actually
happened — `cargo test -p sim-opt --release d2c_sr_rematch --
--ignored --nocapture`, executed on 2026-04-14 against commit
`a28dca05`, with a total wall time of three hours and forty-nine
minutes. The rematch classified as **Null**: the bootstrap 95 percent
confidence interval on the difference of means between SA and CEM,
computed from ten replicates per algorithm, straddled zero with a
negative point estimate.

The one-sentence answer to Ch 30's question — *does Simulated
Annealing, matched to CEM's parameter count and given the same
compute budget, resolve the peak of the D2c stochastic-resonance
landscape more reliably than CEM?* — is: **no, not under the
`best_reward()` aggregation that the protocol measures**. The
qualifier matters. The second half of this chapter is about why.

## Method recap

The rematch ran the folded-pilot protocol specified in Chapter 32
§3.2. Ten replicate seeds were drawn from
`splitmix64(REMATCH_MASTER_SEED.wrapping_add(i))` for `i` in
`0..10`, with `REMATCH_MASTER_SEED = 20_260_412` matching the
`d2c_cem_training.rs` `SEED_BASE` literal. Each `(replicate, seed)`
triple threaded through a fresh `VecEnv` of 32 parallel environments
via `TaskConfig::build_vec_env`, so the physics noise sequence from
the `LangevinThermostat` varied per replicate while the task
definition stayed fixed. Both CEM and SA ran against each seed at
the matched-complexity anchor `LinearPolicy(2, 1)` with
`n_params = 3` and shared initialization `[0.0, 0.0, 2.0]`, so the
only thing separating the two algorithms was their parameter-update
rule. Each training consumed a 16 million environment-step budget
(`TrainingBudget::Steps(32 * 5_000 * 100)`), corresponding to 100
epochs of 160,000 steps each. Twenty trainings in total for the
N=10 batch. The expansion path to N=20 was pre-registered but did
not fire: the N=10 classification was not `Ambiguous`.

Per-replicate summaries were built from `RunResult::best_reward()`,
which returns the maximum `mean_reward` over the 100 epochs of a
training — not the final epoch, and not an average over late
epochs. The bootstrap pipeline then computed a confidence interval
on `mean(SA) - mean(CEM)` with `B = 10_000` resamples, seeded from
`BOOTSTRAP_RNG_SEED = 0xB007_0057_00AA_0055` independently of the
per-replicate physics seeds. The classification rule from Ch 32
§3.3 mapped the CI's lower bound, upper bound, and point estimate
onto one of the three Ch 30 outcomes.

The full protocol detail lives in Chapter 32; this section is a
recap, not a specification.

## Per-replicate data

The ten replicates produced twenty trainings. For each replicate,
the table below lists the seed, CEM's peak reward, SA's peak
reward, and the per-replicate difference in SA's favor (negative
values indicate CEM won that replicate).

| Rep | Seed                    | CEM peak | SA peak  | SA − CEM |
|----:|------------------------:|---------:|---------:|---------:|
|   0 | `6804238419877346958`   | 210.35   | 55.77    | −154.58  |
|   1 | `15460780097817330730`  | 212.64   | 212.32   |   −0.32  |
|   2 | `2279088214681873082`   | 292.98   | 292.12   |   −0.86  |
|   3 | `1553230638221815257`   | 206.16   | 198.09   |   −8.07  |
|   4 | `18395626707652823870`  | 202.60   | 246.66   |  +44.06  |
|   5 | `12811740248017595566`  | 222.49   | 60.41    | −162.08  |
|   6 | `15928824696626942569`  | 164.56   | 158.26   |   −6.30  |
|   7 | `9819515905777901491`   | 220.76   | 226.69   |   +5.93  |
|   8 | `13225215851116322903`  | 220.89   | 244.62   |  +23.73  |
|   9 | `11851954307091673240`  | 181.08   | 203.84   |  +22.76  |

Across the ten replicates CEM won six and SA won four. Two
replicates (reps 1 and 2) were effectively ties — CEM and SA peaked
within less than one reward unit of each other. The remaining
eight replicates split four-four but were asymmetric in magnitude:
CEM's wins on replicates 0 and 5 were dramatic (−154 and −162 in
the difference column), while SA's four wins clustered in the +5
to +44 range. The raw aggregates derived from this table are
`mean(CEM peak) = 213.45`, `mean(SA peak) = 189.88`, and the
point-estimate difference `mean(SA) − mean(CEM) = −23.57`.

## Bootstrap CI summary

The classification pipeline first checks bimodality — see the
next section — and then, conditional on neither algorithm's
reward distribution exceeding the `5/9` threshold, computes a
bootstrap CI on the difference of means. For this data, the
bimodality check did not trip, so the bootstrap ran on means and
produced:

| quantity        | value         |
|-----------------|--------------:|
| point estimate  | −23.57        |
| CI lower (2.5%) | −76.47        |
| CI upper (97.5%)| +24.17        |
| `n_resamples`   | 10,000        |

The CI straddles zero. Both the point estimate and the lower bound
are negative; the upper bound is marginally positive. Under Ch 32
§3.3's classification table, the combination `(lower_pos=false,
upper_pos=true, point_pos=false)` maps to `RematchOutcome::Null`.

Note the CI width. The interval spans roughly 101 reward units —
almost half the magnitude of either algorithm's mean peak. This is
a high-variance, low-signal regime; the data does not cleanly
separate the two algorithms in either direction. The Null here is
not a confident "SA loses"; it is "SA does not reliably win, and
ten replicates are not enough to distinguish SA losing from SA
merely being no better than CEM." The upper bound at +24.17
bounds any SA advantage from above: whatever edge SA has, the
bootstrap cannot rule out that it is as small as zero.

## Bimodality coefficients

Per-algorithm Pearson bimodality coefficients, computed over the
ten peak-reward values:

| algorithm | BC    | threshold | triggered? |
|-----------|------:|----------:|-----------:|
| CEM       | 0.371 | 5/9 ≈ 0.556 | no        |
| SA        | 0.421 | 5/9 ≈ 0.556 | no        |

Neither distribution is bimodal enough to trip Ch 32 §6.3's median
substitution. The bootstrap CI above ran on means, not medians.
This is worth flagging explicitly because the median result would
differ — SA's replicate distribution has two low-tail outliers
(replicates 0 and 5, where SA peaked near 60 while CEM peaked
near 220) that pull SA's mean down. A median-based statistic is
less sensitive to those two points. Under `bootstrap_diff_medians`
the point estimate would shift toward SA (CEM's median is 211.50,
SA's median is 208.08, difference −3.42 instead of −23.57) and
the CI would narrow. This does not change the verdict — the shift
is not large enough to cross zero — but the direction is worth
naming: the two low-SA replicates are a meaningful fraction of
what makes the mean-based difference look negative, and a
median-based protocol (had the bimodality gate tripped) would
have produced a Null that looked much closer to a tie. The
bimodality coefficients themselves tell the same story more
quantitatively: SA's BC (0.421) is higher than CEM's (0.371),
consistent with SA's distribution being the one with the
noticeable tails. Neither algorithm's distribution is *cleanly*
bimodal, so the protocol stayed on means.

## Verdict

The rematch classification is `RematchOutcome::Null`. SA does not
reliably outperform CEM on the D2c stochastic-resonance task at
matched complexity under the `best_reward()` aggregation, with
the upper bound of the bootstrap CI at +24.17 reward units. Any
SA advantage over CEM on this task, if present, is bounded above
by that value — small relative to the ~200 reward unit magnitude
of either algorithm's per-replicate peaks.

This is one of Ch 30's three pre-registered meaningful outcomes.
It is informative in the sense Ch 30 §3 named: a Null tells us
that the basic isotropic-proposal SA, as implemented in
`sim/L0/opt/src/algorithm.rs`, does not resolve the D2c-SR peak
more reliably than CEM at the same parameter count and compute
budget. It does not tell us that SA *cannot* outperform CEM on
this task — that is what the Ch 30 follow-ups in the "Next steps"
section below are for.

## What the metric measured

The writeup could end at the previous section and still honor
the Ch 92 §2 template. The rest of this chapter is the part
the template did not anticipate: a methodological finding
about what `best_reward()` actually measures, which the author
did not see coming and which the reader needs to see the
verdict in its full context.

During the run, an intermediate readout of the live log at
replicate 5 suggested the opposite verdict. Every `[ALG] done`
line in the test output prints the *final* epoch's reward, and
read naively those lines told a clean Positive story: CEM
replicate 0 finished at 14.16, SA replicate 0 finished at
55.77; CEM replicate 1 at 26.59, SA replicate 1 at 212.32; CEM
replicate 2 at 21.97, SA replicate 2 at 292.12; and so on. On
final-epoch reward, SA won every single replicate in the first
six reported, by margins averaging +147. Had the rematch used
"final epoch mean reward" as its replicate statistic, the
bootstrap CI would have been firmly positive and the verdict
would have been Positive. But the rematch does not use the
final-epoch reward. It uses `RunResult::best_reward()`, which
is the *maximum* `mean_reward` across the full 100-epoch
training. The same data under the two metrics produces opposite
verdicts. The Null surfaced entirely because of the gap between
CEM's peak and CEM's final, which is enormous and asymmetric.

Concretely, the per-replicate gap between `best_reward()` (peak
across epochs) and `final_reward()` (last epoch) for CEM looks
like this:

| Rep | CEM peak | CEM final | peak − final |
|----:|---------:|----------:|-------------:|
|   0 | 210.35   |   14.16   | 196.19       |
|   1 | 212.64   |   26.59   | 186.05       |
|   2 | 292.98   |   21.97   | 271.01       |
|   3 | 206.16   |  −31.22   | 237.38       |
|   4 | 202.60   |   86.13   | 116.47       |
|   5 | 222.49   |   45.40   | 177.09       |
|   6 | 164.56   |   16.58   | 147.98       |
|   7 | 220.76   |   95.34   | 125.42       |
|   8 | 220.89   |   39.33   | 181.56       |
|   9 | 181.08   |  −17.54   | 198.62       |

Every CEM replicate showed a peak-to-final drop of at least 116
reward units. Three replicates (2, 3, 9) dropped by more than
190 units. Averaged across the ten replicates, CEM's peak
exceeded its final by 183.8 reward units — roughly the magnitude
of either algorithm's per-replicate peak. CEM discovered good
policies during training and then lost them. This is not a bug;
it is the defining property of CEM. CEM is a population method:
each epoch samples a fresh population from the elite-refit
distribution, which means the reported `mean_reward` at any
given epoch is the mean performance of the current population,
not of the incumbent best. A lucky population produces a high
`mean_reward` at one epoch; the elite-refit then pulls the
distribution toward the lucky members' parameters, and the next
epoch's population draws from the shifted distribution. When
that shift overshoots or lands off-peak, the population-mean
reward can drop sharply. Across 100 epochs, CEM's population
drifts through regions of the parameter space whose reward
varies widely, and the single highest-reward epoch in that
trajectory is what `best_reward()` captures.

SA, by contrast, tracks an incumbent and accepts a proposal
only if it improves on the incumbent under a Metropolis
criterion. The incumbent's reward is monotonically
non-decreasing by construction, so the epoch-level
`mean_reward` — which evaluates the incumbent in each of the 32
parallel environments — also trends monotonically. The gap
between SA's peak and SA's final is small across the board
(the peak-minus-final values for SA top out at 15.73, with
seven of ten replicates showing exactly zero) because SA's
"final" *is* the best policy it has ever held. For SA, `best_reward()` and
`final_reward()` are nearly the same number. For CEM, they are
radically different numbers, and `best_reward()` is the
larger of the two.

The consequence is structural: `best_reward()` asks "what is
the best mean-reward epoch you saw during training?" and for a
population-based explorer that question is answered by the
luckiest epoch the population drifted through. It is not the
same question as "how well does your final policy perform?"
and on a task where the two algorithms' population dynamics
differ as sharply as CEM and SA do, the two questions have
different answers. Under the first question, the two
algorithms are competitive on D2c-SR and the rematch is Null.
Under the second, SA would appear to dominate by a large
margin.

Neither answer is wrong. They are different measurements. The
`best_reward()` aggregation, as chosen by Ch 24, was picked for
a reasonable reason: it is robust to late-training collapse
(e.g., a late-epoch numerical instability or a policy that
overfits to one reward component), and it answers the question
"given this algorithm's training trajectory, what is the best
policy you could extract from it?" That question is the right
one for a practitioner who intends to use the `best_artifact`
policy at inference time, not the final one. The Ch 42 fixture
and the Ch 92 tracker both route the per-replicate statistic
through `best_artifact`'s provenance, not `artifact`'s, which
is consistent with that intent. The rematch is measuring what
it says it is measuring.

But the interpretation of the Null is therefore narrower than
the headline suggests. The rematch does not say "SA is not
better than CEM at D2c-SR." It says "under the
best-epoch-across-training aggregation, where CEM's
population-exploration volatility is counted as a positive
signal, SA does not reliably beat CEM at D2c-SR." A
final-reward aggregation would have produced the opposite
verdict. Neither aggregation is universally correct; the
choice depends on whether the downstream consumer of the
policy wants "the policy I ended with" or "the best policy
this training trajectory ever produced." The rematch
pre-registered the second, and the reader should evaluate the
Null with that in mind.

This is not a protocol weakness. Ch 24, Ch 30, and Ch 32 all
chose the peak-aggregation primitive deliberately. What this
chapter adds is the observation that the primitive's choice
is load-bearing in a way that was not visible in advance — the
D2 single-seed runs that motivated the rematch did not
surface the peak-vs-final gap as a comparison axis, because a
single seed does not produce a replicate distribution. The
observation belongs here, in the post-execution writeup,
because the run is what made it visible. A future
Ch 24-review-round that wanted to pre-register a richer
aggregation surface — reporting *both* peak-reward and
final-reward replicate means, for example, and classifying
against each — would have to cite this chapter as the
evidence that the distinction matters in practice.

## Next steps

Per Ch 30 §3 and Ch 92 §3, a Null verdict fires two
pre-registered follow-ups:

1. **Richer-proposal SA.** The current `Sa` implementation uses
   an isotropic scalar `proposal_std` with no adaptation. The
   null leaves open the possibility that a smarter proposal
   distribution — adaptive `proposal_std` tied to recent accept
   rate, or an anisotropic covariance estimated from the
   chain's acceptance history — would resolve the D2c-SR peak
   where the basic SA does not. Implementation sketch and
   scope estimate live in Ch 92 §3.a.
2. **Parallel Tempering.** K parallel SA chains at different
   temperatures with periodic Metropolis-criterion state swaps
   between adjacent chains. Structurally different from a
   richer proposal: PT uses K× the per-chain compute of basic
   SA, so the rematch framing would have to either reduce
   per-chain budget by 1/K to preserve compute parity with CEM
   or acknowledge the comparison is at K× compute. Ch 92 §3.b
   has the scope detail.

Both follow-ups reuse the `sim_opt::analysis` module and the
rematch driver verbatim — they differ only in the SA builder
passed into `Competition::run_replicates`. A parallel test
fixture duplicates `sim/L0/opt/tests/d2c_sr_rematch.rs` with the
SA builder replaced.

The metric-sensitivity observation from the previous section is
orthogonal to both follow-ups and does not itself trigger a
Ch 30 pre-committed action — Ch 30's failure-modes menu does
not include "the aggregation primitive is load-bearing in a
way we did not anticipate." A future reader who wants to
re-run the rematch under a final-reward aggregation for
comparison should do so as a new study question, not as a
continuation of Ch 30's original menu.

## Reproducibility anchors

Any future re-run of this writeup should be able to verify the
numbers above from these anchors:

- Commit: `a28dca05` (`feature/ml-chassis-post-impl`)
- Fixture: `sim/L0/opt/tests/d2c_sr_rematch.rs`
- Command: `cargo test -p sim-opt --release d2c_sr_rematch -- --ignored --nocapture`
- `REMATCH_MASTER_SEED`: `20_260_412`
- `BOOTSTRAP_RNG_SEED`: `0xB007_0057_00AA_0055`
- Bootstrap resamples `B`: `10_000`
- Initial batch size `N_INITIAL`: `10`
- Budget: `TrainingBudget::Steps(16_000_000)` = `32 * 5_000 * 100`
- Matched-complexity anchor: `LinearPolicy(2, 1)`, `n_params = 3`, init `[0.0, 0.0, 2.0]`
- Captured run log: `/tmp/d2c_sr_rematch.log` (ephemeral — rerun the
  command above to regenerate)
- Wall time: 13,760.92 seconds (~3h49m) on a MacBook Pro under
  `caffeinate -i`

## Cross-references

- Ch 30 §"What each outcome would tell us" — Null semantics
- Ch 30 §3 — pre-committed null follow-ups menu
- Ch 32 §3.2 — bootstrap CI pseudocode
- Ch 32 §3.3 — three-outcome classification table
- Ch 32 §6.3 — bimodality contingency
- Ch 32 §7 — writeup-format deferral
- Ch 42 §6 sub-decision (j) — post-execution writeup deferral
- Ch 42 §9 — remaining-artifact entry
- Ch 92 §2 — suggested structure for this writeup
- Ch 92 §3 — Ch 30 null follow-ups, scope estimates
- `sim/L0/opt/src/analysis.rs` — bootstrap, classification, folded-pilot driver
- `sim/L0/ml-bridge/src/competition.rs` — `RunResult::best_reward()` definition
- `sim/L0/opt/tests/d2c_sr_rematch.rs` — the fixture
