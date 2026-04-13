# Review log — Chapter 32: Hyperparameter sensitivity

Chapter 32 is an argument chapter — Part 3's second ungated
closer. It picks five decisions (test family, significance
threshold, replicate count protocol + initial batch, warmup-
overhead factor) plus one named contingency (bimodality
substitution) and defends each pick against the alternatives
that could have gone differently. Factual and thinking passes
ran sequentially per the argument-chapter pattern established
by Ch 15 / Ch 23 / Ch 24.

## Recon

Ch 32's recon was focused on three upstream chapters and one
set of external statistical references.

- **Ch 24 §4 and §5** — re-read in full to confirm the
  aggregation surface Ch 24 shipped (`SeedSummary { n, mean,
  std_dev }`, `replicate_best_rewards(task, algo) -> Vec<f64>`)
  and the three explicit deferrals to Ch 32 (statistical test,
  replicate count `N`, pilot design). Ch 24 §4.5 also named
  bimodality as a plausible distributional shape and deferred
  the empirical check to Ch 32.
- **Ch 31 §3.1, §3.2, §4.4** — re-read to confirm the three
  values Ch 31 handed Ch 32: the warmup-overhead gate's
  "predetermined factor" (§3.1), the seed-variance envelope
  test and significance threshold (§3.2), and the pilot-
  existence bucket-4 prerequisite (§4.4).
- **Ch 30 §"What the chapter does not decide"** — re-read to
  confirm the three-outcome framing (positive, null, ambiguous)
  and the Henderson-warning attribution for the ambiguous
  bucket.
- **Ch 22 §"The warmup overhead"** — re-read to confirm the
  numerical anchor (`n_envs * warmup_steps = 32 * 1000 =
  32{,}000` env steps = 0.2% of 16M) and the exact conditional
  language Ch 22 uses ("*conditional on the rematch's effect
  size being large relative to 0.2 percent*").
- **Ch 23 §1.2** — re-read to confirm the seed-derivation prose
  recommendation (splitmix64 applied to a master seed) and the
  example master literal (`let master: u64 = 20_260_412;`).
  This recon found a cross-reference worth naming: Ch 23's
  example master literal coincides with `SEED_BASE = 20_260_412`
  at `d2c_cem_training.rs:62`, so reusing the value in Ch 32
  gives three-way consistency across the study, the D2c test
  fixture, and the rematch protocol.
- **Henderson 2018 and Agarwal 2021** — external references
  were cross-checked for paper titles and claims. Henderson et
  al. 2018, "Deep Reinforcement Learning that Matters" (AAAI
  2018) — established the small-`N` seed-overconfidence
  critique. Agarwal et al. 2021, "Deep Reinforcement Learning
  at the Edge of the Statistical Precipice" (NeurIPS 2021) —
  recommended bootstrap confidence intervals as the modern
  empirical-RL evaluation standard. Both papers are well-known
  in the field and the attributions are uncontroversial.

**One targeted recon question** surfaced during the
recon-to-leans conversation and was resolved before drafting:
how to handle the pilot-doesn't-exist structural difficulty.
Three plausible shapes were enumerated — (a) design the pilot
in detail and punt the test pick to a later chapter, (b) pick a
test conservative enough to work without pilot variance data,
and (c) enumerate candidate tests with acceptance criteria and
bind the binding to a pilot-read chapter. **Resolution:**
refined option (b). Bootstrap CI on the difference of means is
defensible without pilot data because it is distribution-free,
the choice of test family is constrained more by `N` than by
distributional shape at the small-`N` regime, and the pilot's
role can be reshaped from "decide the test" to "scout `N` and
check for bimodality." This reshape in turn opens the folded-
pilot scheme (the initial batch IS the pilot), which saves
compute relative to running a separate pilot + rematch.

## Factual pass

**Status:** 36 checkable claims across code citations,
cross-chapter quotes, external paper attributions, and
mathematical/statistical facts. The chapter has a moderate
falsifiable surface — most claims are verbatim quotes from
upstream chapters, file:line citations to ml-bridge train
loops, and standard small-sample statistics.

| # | Claim | Verdict |
|---|---|---|
| 1 | Ch 30 three outcomes + Henderson warning quote | VERIFIED |
| 2 | Ch 24 §5 three deferrals list (test, N, pilot) | VERIFIED |
| 3 | Ch 31 §3.1 "predetermined factor" language | VERIFIED |
| 4 | Ch 31 §3.2 test-and-significance deferral | VERIFIED |
| 5 | Ch 31 §4.4 "pilot existence" bucket-4 prerequisite | VERIFIED |
| 6 | Ch 23 pool `{CEM, SA}`, PPO excluded | VERIFIED |
| 7 | Ch 23 matched complexity `LinearPolicy(2,1)`, `n_params=3` | VERIFIED |
| 8 | Ch 23 `run_replicates` API — signature shape and return type | PARTIAL — my draft cited `run_replicates(seeds: &[u64])` as the signature; actual Ch 23 takes `(tasks, builders, seeds)` returning `Result<CompetitionResult, EnvError>`. Three sites affected (intro inheritance, §4.2, §4.8 code skeleton). |
| 9 | Ch 23 splitmix64 prose recommendation | PARTIAL — initial draft missed this upstream recommendation entirely and picked raw sequential integers `[0..10]`, which is exactly the `(master..master+n)` form Ch 23 §1.2 warned about as having "a mild correlation structure that can surprise readers." §4.6 rewritten to adopt splitmix64. |
| 10 | Ch 24 `replicate_best_rewards` signature | VERIFIED |
| 11 | Ch 24 `describe` signature | VERIFIED |
| 12 | Ch 24 `SeedSummary { n, mean, std_dev }` | VERIFIED |
| 13 | Ch 24 Decision 1 train loops: `cem.rs:209`, `td3.rs:490`, `sac.rs:543` | VERIFIED |
| 14 | Ch 24 §4.5 bimodality quote ("Seed distributions across replicates are sometimes non-Gaussian...") | VERIFIED |
| 15 | Ch 24 §4.5 "no new storage or collection logic is needed, only new read helpers" | VERIFIED |
| 16 | Ch 24 §4.6 sample std / Bessel's correction | VERIFIED |
| 17 | Ch 24 §4.6 `N` range projection "3 to 10" | VERIFIED |
| 18 | Ch 22 warmup arithmetic (`32 * 1000 = 32,000 = 0.2% of 16M`) | VERIFIED |
| 19 | Ch 22 "document and live with it" language | VERIFIED |
| 20 | Ch 22 "case against deducting" conditional language | PARTIAL — my draft put Ch 31's paraphrase ("holds *only if*...") in quotes attributed to Ch 22; actual Ch 22 framing is "*conditional on* the rematch's effect size being large relative to 0.2 percent." Two sites affected (§5.1, "What Ch 32 inherits"). |
| 21 | Ch 31 §3.1 "moot for {CEM, SA}" note | VERIFIED |
| 22 | Ch 31 §3.1 "the gate has to bound" quote | VERIFIED |
| 23 | Ch 31 §3.2 "weak positive / near-null hedge" quote | VERIFIED |
| 24 | Ch 31 §4.4 "Ch 32's prerequisite is satisfied by the chapter being drafted" quote | VERIFIED |
| 25 | Mann-Whitney U minimum p-value at `N = 3 vs N = 3`: `2/C(6,3) = 2/20 = 0.1` | VERIFIED |
| 26 | Mann-Whitney U minimum p-value at `N = 5 vs N = 5`: `2/C(10,5) = 2/252 ≈ 0.0079` | VERIFIED |
| 27 | `C(20, 10) = 184{,}756` (exact permutation count at `N = 10 vs N = 10`) | VERIFIED |
| 28 | `C(40, 20) ≈ 1.4 × 10^11` (approximate permutation count at `N = 20 vs N = 20`) | VERIFIED (exact value: 137,846,528,820) |
| 29 | Bimodality coefficient formula and small-sample correction | PARTIAL — my draft attributed the BC to "Hartigan's bimodality coefficient." Hartigan (1985) authored the dip test for unimodality, a distinct statistic. The BC with the 5/9 threshold is the Pearson 1916 / SAS small-sample formula. Two sites affected (intro bullet 4, §6.2 heading and body). |
| 30 | BC threshold `5/9 ≈ 0.556` as the uniform-distribution value | VERIFIED |
| 31 | BC requires `n ≥ 4` (formula divides by `(n-2)(n-3)`) | VERIFIED |
| 32 | `sim/L0/thermostat/tests/d2c_cem_training.rs` exists and is single-seed | VERIFIED |
| 33 | `SEED_BASE = 20_260_412` at `d2c_cem_training.rs:62` (cross-reference recon) | VERIFIED |
| 34 | `RunResult::best_reward() -> Option<f64>` at `competition.rs:45` | VERIFIED |
| 35 | Henderson 2018 "Deep Reinforcement Learning that Matters" AAAI | VERIFIED |
| 36 | Agarwal 2021 "Deep Reinforcement Learning at the Edge of the Statistical Precipice" NeurIPS | VERIFIED |

**Counts:** 32 VERIFIED, 4 PARTIAL, 0 WRONG. All four partials
were applied as fixes on-the-spot during the factual pass.

**Most consequential fix:** Claim #9 (splitmix64 prose
recommendation). The initial draft picked raw sequential
integers `[0..10]` as the master seed values and rejected only
hash-from-string as an alternative, missing splitmix64 entirely.
Ch 23 §1.2 specifically flagged raw sequential seeds as having
"a mild correlation structure that can surprise readers" and
recommended splitmix64 as the conservative alternative. §4.6
was rewritten to adopt the splitmix64 derivation, reuse Ch 23's
example master literal `20_260_412` (which coincides with the
existing `SEED_BASE = 20_260_412` at `d2c_cem_training.rs:62`,
giving three-way consistency across the study, the test
fixture, and the rematch protocol), and document the rejection
of both sequential integers and hash-from-string. §4.2
protocol step, §4.8 code skeleton, and the intro bullet 4 were
updated to match. This is the single most consequential
factual finding — picking the form Ch 23 explicitly cautioned
against would have been Ch 32 contradicting its own upstream
chapter without a chapter-level reason.

**Verdict:** PASS with attribution drift. No source code line
numbers had drifted from their canonical Ch 24 citations. The
one structural inheritance error (Ch 23 splitmix64
recommendation) was caught and corrected before the thinking
pass began.

## Thinking pass

**Status:** Argument-chapter 10-point brief, run sequentially
after the factual pass per the Ch 15 / Ch 23 / Ch 24 protocol.
The brief pushed on assumptions, unexamined alternatives,
soft-word calibration, scope discipline, whether the argument
earns its conclusion, internal consistency, counterfactual
depth, inheritance fidelity, edge cases in the protocol, and
skeptical-reader pushback points.

**Verdict:** Ship with edits. One MUST-FIX, five SHOULD-FIX,
three MINOR. All nine applied on a single round.

**MUST-FIX findings and fixes:**

1. **§4.3 FPR argument was too strong.** The initial draft
   claimed the expansion rule "does not inflate type I error"
   and that "the false-positive rate at `N = 20` is the
   nominal 5 percent, and the early stopping at `N = 10` does
   not change the rate." This is inaccurate. The protocol's
   one-sided FPR under the null is bounded above by `2α_1`
   via the union bound, not exactly nominal. The inflation is
   modest in practice because the `N = 10` and `N = 20` tests
   share data (positively correlated), but it is not zero. A
   skeptical reader with a statistics background would catch
   the overclaim and doubt the rest of the chapter. *Fix:*
   rewrote §4.3 as a two-part upper bound — worst case `2α_1`
   via union bound, practical case substantially below `2α_1`
   because of shared-data correlation. Named the Bonferroni-
   corrected alternative explicitly and documented the trade:
   the protocol accepts bounded FPR inflation in exchange for
   compute savings and for CI widths that match reader
   expectations. Also addressed an inter-batch composition
   concern the original draft ducked: if the bimodality check
   triggers at `N = 10` but not at `N = 20`, the test shifts
   from medians to means between the two reads, and the
   headline is the test appropriate to the *final* data's
   distributional shape.

**SHOULD-FIX findings and fixes:**

2. **§3.2 missing BCa bootstrap counterfactual.** The draft
   picked percentile bootstrap implicitly without mentioning
   BCa (bias-corrected and accelerated) bootstrap. BCa is the
   modern statistical recommendation over percentile at small
   `N` because it corrects for bias and skew in the bootstrap
   distribution. A skeptical reader with a statistics
   background would immediately ask why not BCa, and the
   chapter had no answer. *Fix:* added §3.8 as a new
   sub-decision. The argument for percentile: Agarwal 2021's
   `rliable` convention uses it, reader familiarity, simpler
   implementation, and the anti-conservatism concern at `N = 10`
   is addressed structurally by the expansion rule rather than
   by the CI-computation method. The argument for BCa: marginal
   coverage improvement at small `N`. Picked percentile on
   balance-of-arguments grounds and named BCa as a defensible
   alternative a Ch 42 writeup author could substitute at
   publication time.

3. **§4.3 inter-batch bimodality composition underspecified.**
   The original §4.3 named the intra-batch case (bimodality at
   `N = 10` triggers median substitution, early-stopping still
   applies to the median-based result) but did not address the
   inter-batch case (what happens if `N = 10` is bimodal but
   `N = 20` is not). *Fix:* folded into the §4.3 rewrite under
   (1). The rule is that the bimodality check re-evaluates at
   each classification step and the headline uses the test
   appropriate to the final data's distributional shape. A
   reader who follows the protocol-trace sees both
   intermediates and both choices, and the pre-registration
   makes the rule deterministic.

4. **§7 did not address the `None`-replicate case.** Ch 24
   §4.7 flagged the silent filter-out of `None` replicates as
   a contingency depending on Ch 41 fixing the REINFORCE/PPO
   zero-fallback. For the rematch pool `{CEM, SA}` the case is
   not a practical concern — CEM always produces at least one
   finite epoch under `Steps(16M)`, and SA's metric-reporting
   contract is Ch 42's spec — but Ch 32 had not named it
   explicitly. *Fix:* added a §7 bullet acknowledging the
   assumption that all replicates return finite `best_reward`,
   and naming Ch 42's rematch implementation as responsible
   for detecting the case at runtime and either retrying or
   reporting the degradation.

5. **`B = 10000` picked without justification.** The Section
   3.2 draft stated `B = 10000` as a constant without
   defending it against `B = 1000` or `B = 100000`. *Fix:*
   added a one-paragraph defense grounded in the Monte Carlo
   error on the percentile estimate: at `B = 10000`, the SE
   on the 2.5th or 97.5th percentile estimate is roughly
   `sqrt(0.025 * 0.975 / 10000) ≈ 0.0016`, which is well
   below the rounding precision of the reported CI bounds.
   Smaller `B` risks visible Monte Carlo noise; larger `B` is
   strictly more compute for no gain.

6. **95% CI level picked without justification.** Same shape
   as finding 5 — the draft stated "95 percent" without
   defending against 90% or 99%. *Fix:* added a one-paragraph
   defense: 95% is the field-standard convention, matches
   Henderson 2018 and Agarwal 2021, and is what the rematch's
   audience expects. 90% would be more liberal and risk
   misreading, 99% would widen the CI enough to push realistic
   effect sizes into the ambiguous bucket.

**MINOR findings and fixes:**

7. **§1.4 "true coverage near 92–94 percent" too strong.** The
   phrase "true coverage" implied a proof rather than a
   citation. *Fix:* softened to "empirical estimates from the
   small-sample bootstrap literature place it near 92–94
   percent against nominal 95 percent for moderate
   distributional shapes." Same softening applied in §4.4 and
   §4.5 where the same phrasing recurred.

8. **§3.2 claim 3 "no `N` value below which the test silently
   breaks" too strong.** At `N = 1, 2` the bootstrap-with-
   replacement procedure is degenerate (only 4 distinct
   resamples at `N = 2`, a single point at `N = 1`). The
   rematch never operates in that regime, but the claim was
   technically false. *Fix:* scoped the claim to "within the
   rematch's `N` range (`N = 10` and `N = 20`)" and added an
   explicit note that bootstrap is genuinely degenerate at
   `N ≤ 2` but the rematch is never in that regime.

9. **§6.2 boundary case `BC = 5/9` exactly.** The draft did
   not say whether the threshold check used strict or
   non-strict inequality. *Fix:* named the rule as strict
   inequality (`BC > 5/9`) and added a one-sentence note
   handling the exactly-uniform case. The boundary is
   vanishingly unlikely under any realistic distribution but
   deserves an explicit rule.

**Factual-pass partials applied alongside the thinking-pass
fixes:**

| # | Partial | Fix site |
|---|---|---|
| F8 | `run_replicates` signature drift | Intro inheritance paragraph, §4.2 step 2, §4.8 code skeleton |
| F9 | splitmix64 prose recommendation missed | §4.6 rewritten; intro bullet 4, §4.2 step 1, §4.8 code skeleton updated |
| F20 | Ch 22 "case against deducting" quote attribution | §5.1, "What Ch 32 inherits" paragraph |
| F29 | "Hartigan's bimodality coefficient" misattribution | Intro bullet 4, §6.2 heading and body |

## Self-review after fix application

After applying all factual and thinking-pass fixes, a final
read-through caught three editing slips, all neighbor-sentence
drift between edited regions and untouched regions:

1. **§4.4's reference to Section 4.3's FPR argument.** The
   original §4.4 said "the multiple-testing concern is not a
   real concern under the rematch's expansion rule (Section
   4.3)" — but the §4.3 rewrite now acknowledges the bound,
   so the §4.4 phrasing was out of sync. Updated §4.4 to
   reflect the bounded-but-not-nominal framing.

2. **§4.4 and §4.5 "true coverage" phrasing.** The §1.4 fix
   for Finding 7 did not propagate to §4.4 and §4.5, which
   used the same "true coverage" phrasing in the `N = 5`,
   `N = 15`, and `N = 20` discussions. Propagated the
   softening to "empirical coverage" / "empirical estimates"
   at both sites.

3. **§4.3 "reported in the writeup's protocol-trace section"
   read as a writeup mandate.** §7 declares writeup format
   choices out of scope; the §4.3 rewrite should have
   phrased the N=10 intermediate as recommendation-level, not
   mandate-level. Softened to "how much of the N=10
   intermediate gets surfaced in the writeup alongside the
   headline is a writeup-format choice Section 7 declares
   out of scope."

No factual claims change in the self-review patches.

## Decisions locked

| # | Decision | Value |
|---|---|---|
| D1 | Statistical test family | Bootstrap CI on the difference of means; `B = 10000` resamples; two-sided 95% interval; paired resampling |
| D2 | Significance threshold + Ch 30 mapping | 95% CI lower bound > 0 ⇒ positive; CI upper bound ≤ 0 ⇒ null; CI straddles 0 with point estimate > 0 ⇒ ambiguous; CI straddles 0 with point estimate ≤ 0 ⇒ null. Three Ch 30 buckets, no fourth-option hedge |
| D3 | Percentile vs BCa bootstrap | Percentile. BCa named as defensible alternative Ch 42 writeup author may substitute at publication time |
| D4 | Replicate count protocol | `N_initial = 10`, pre-registered single expansion to `N_final = 20` iff initial-batch CI is ambiguous, hard cap at 20 |
| D5 | Initial-batch composition | D2c task; both `{CEM, SA}` at matched complexity `LinearPolicy(2, 1)`, `n_params = 3`; full `Steps(16M)` per replicate |
| D6 | Seed derivation | `splitmix64(MASTER.wrapping_add(i))` for `i` in `0..10` (initial) and `i` in `10..20` (expansion), with pre-registered `MASTER = 20_260_412` matching Ch 23 §1.2's example and `d2c_cem_training.rs:62`'s existing `SEED_BASE` |
| D7 | Paired vs unpaired bootstrap | Unpaired — CEM and SA do not naturally pair by seed |
| D8 | Warmup-overhead factor (Ch 31 §3.1) | 5× — effect must exceed ~1% of budget-equivalent reward scale. Moot for current `{CEM, SA}` pool, fixed for forward use |
| D9 | Bimodality detection criterion | Pearson-based (SAS small-sample-corrected) bimodality coefficient with strict-inequality threshold `BC > 5/9` |
| D10 | Bimodality escalation rule | If either algorithm's BC exceeds 5/9 at any classification step, substitute bootstrap CI on the difference of *medians*; same B, same α, same Ch 30 mapping. Pre-registered, deterministic, one-line site change |

## Ch 31 patch warranted

Ch 31 §4.4 describes the pilot as a separate pre-rematch event
("the pilot has been run, the variance has been measured").
Under Ch 32's folded-pilot scheme, the pilot is the first batch
*of* the rematch, not a separate landing event. Ch 31 §4.4
needs a 3e1ec0ff-style post-commit patch to reflect this — the
same pattern Ch 22's findings applied to Ch 21. The patch is
narrow and lands in a separate commit after Ch 32 is
committed. §3.1 and §3.2 need no patches — they ungate cleanly
because Ch 32 fills the values they were waiting for.

## Process notes for future Part 4 chapters

- **Sequential factual → thinking passes matter for argument
  chapters.** Ch 32's factual pass surfaced the splitmix64
  recommendation gap (finding F9), which was a structural
  inheritance error that would have cascaded into every
  section that referred to seed derivation. Catching it at
  the factual-pass stage meant the thinking pass operated on
  a draft whose inheritance framings were already correct.
  Running the two passes in parallel would have risked the
  thinking pass pressure-testing a draft whose upstream
  fidelity was broken at the ground level.
- **The 10-point argument-chapter brief caught what an 8-point
  descriptive brief would have missed.** The BCa counterfactual
  (finding 2) and the FPR-inflation overclaim (finding 1) both
  came from the "unexamined alternatives" and "does the
  argument earn its conclusion" points, neither of which is a
  natural descriptive-chapter concern. For Part 4's execution
  PR plans, each of which is argument-chapter-shaped (they
  argue for a specific PR scope and landing strategy), the
  10-point brief is the right tool.
- **Self-review after fix application catches neighbor-sentence
  drift.** Three editing slips were caught in the final read,
  all at the seam between an edited region and an untouched
  region. Running the self-review pass before committing is
  worth the extra five minutes — it turns "found in review"
  into "fixed before commit" for minor consistency drift.
