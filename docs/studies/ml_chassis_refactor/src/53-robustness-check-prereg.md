# Robustness check: Ch 30 follow-ups under §2.3's framework

Chapter 51 §4 endorsed **Positive** as the Ch 30 reading for
the amended re-run's `(Null, Positive)` joint verdict. That
endorsement declared the Ch 30 null follow-ups (richer-proposal
SA, Parallel Tempering) **moot** for the D2c-SR question —
their menu condition (Null on the primary metric) did not
fire. This chapter does not retract that declaration.

This chapter exists because the author has access to idle
compute on a second machine and wants to squeeze a robustness
check out of it. The follow-up algorithms Ch 30 §3 specified
— richer-proposal SA and Parallel Tempering — happen to be
exactly the right variants to test a different question than
Ch 30's: *"does §2.3's `final_reward`-primary framework
generalize from basic isotropic SA to the broader SA family,
or is basic SA's `final_reward` advantage on D2c-SR partly
accidental?"* The algorithms are the same; the question
they're answering is different, and the pre-committed decision
rule in §3 below reflects the different question.

Ch 53 is deliberately separated from Ch 51 by a commit
boundary so that `git log --reverse` over both files makes
the ordering visible: Ch 51 §4 endorsed Positive *first*, on
the Ch 30 question; Ch 53 runs a robustness check *second*,
on §2.3's framework. The sequence matters for the same
reason Ch 51 §2.5's four-commit anchor mattered — if the
robustness check's pre-commitment were written after its own
results were observed, it would not be a pre-commitment.

## Section 1 — Why this chapter exists

### 1.1 What this chapter is NOT

Ch 53 is not a Split reading of the `(Null, Positive)` cell.
Section 2.3 of Ch 51 left the Split reading open as a
transparency qualifier, and Ch 51 §4.5 explicitly argued
against adopting it. Adopting the Split reading now because
idle compute has become available would be a retreat from
§4.5's argument that is not compute-dependent in its logic.
§4.5 stands; Ch 51 §4's Positive endorsement is binding; the
Ch 30 null follow-ups remain moot for Ch 30's question.

Ch 53 is not a reopening of Ch 30. Ch 30's question is
"resolves the peak at `kt_mult ≈ 2.55`?" and Ch 51 §4.3
answered it Positive on SA's mean `final_reward` of `187.16`
against the empirical peak range `~165-293`. That answer
stands unless Ch 53 §5 explicitly retracts it under the rule
in §3.3 below, and §3.3 commits that *Ch 53 cannot
automatically retract Ch 51 §4's endorsement* — the most
Ch 53 can do, even under its strongest-signal outcome, is
open an investigation that a future chapter resolves.

Ch 53 is not a replay of Ch 30 §3's null-follow-ups menu. The
menu was "if SA fails on the primary metric, try richer SA
variants to see whether a smarter proposal finds the peak."
Under Ch 51 §4's Positive reading, basic SA did not fail, so
that menu condition is not fired. Ch 53 uses the same
algorithms for a different purpose, and §3's decision rule is
structured around that different purpose.

### 1.2 What this chapter IS

Ch 53 is a pre-committed robustness check on Ch 51 §2.3's
interpretation framework. The framework commits to
`final_reward` as the primary operationalization of Ch 30's
"resolves the peak" question on theoretical grounds — the
population-vs-incumbent asymmetry of CEM against basic SA.
The framework's theoretical grounding is a claim about *the
SA family as a class*, not about basic isotropic SA
specifically. Ch 53 tests whether that class-level claim
holds up when the SA family is instantiated by variants
other than basic SA.

If richer-proposal SA and Parallel Tempering both retain a
substantive `final_reward` advantage over CEM on D2c-SR
matching basic SA's effect-size class, §2.3's framework is
corroborated across the SA family, and Ch 51 §4's Positive
reading is strengthened by triangulation. If they do not,
§2.3's framework has a more specific scope than Ch 51
claimed — it works for basic SA on D2c-SR but does not
generalize across the SA family — and Ch 53 §5 opens an
investigation into what that means for Ch 51 §4's reading.

The chapter is therefore an extension of Ch 51's
methodological discipline, not a redo of Ch 30. Its scope is
"how robust is the framework?" not "what is Ch 30's answer?"

### 1.3 Why pre-commitment matters here

The same pre-registration discipline Ch 51 §2 applied to the
dual-metric amendment applies to Ch 53. Idle compute is not
a license to run experiments whose interpretation gets
decided after seeing the numbers. If Ch 53 ran the follow-ups
first and then wrote an interpretation section whose rules
were chosen with the results in view, the chapter would be
providing the appearance of robustness-checking without the
substance. The substance requires committing the decision
rule before any follow-up fires, which is what §3 below does.

The pre-commitment binds the *author's* interpretation, not
the data. If a future reader disagrees with §3's decision
rule, they are welcome to apply a different one — but they
should do so explicitly, citing the disagreement and their
alternative rule, rather than silently adopting a different
interpretation because they like it better.

## Section 2 — The follow-ups

### 2.1 Scope of the pre-registration

Ch 53 pre-registers at the *class* level, not the
implementation level. The specific algorithmic details of
each variant — adaptation mechanism for richer-proposal SA,
number of chains and swap interval for PT — are deferred to
the code commits that implement each variant. This is the
same pattern Ch 23 used for deferring per-replicate seed
constants to Ch 32, and Ch 32 used for deferring the
bootstrap seed to Ch 42: the scientific commitment is to
*what class of variant runs*, not *what byte-level parameters
the variant's implementation chooses*.

The pre-registration binds four things per variant: (1) the
class of algorithm, (2) the compute-parity constraint,
(3) the seed pool, and (4) the measurement layer. The four
bindings jointly ensure that whatever the code commits pick
for the byte-level parameters, the resulting run is
comparable to basic SA on D2c-SR and interpretable under §3's
decision rule.

### 2.2 Richer-proposal SA

**Class commitment.** The richer-proposal SA variant is any
SA implementation whose proposal distribution is
*non-stationary during training*, in contrast to
`sim/L0/opt/src/algorithm.rs`'s `Sa::new` isotropic scalar
`proposal_std` with no adaptation. The non-stationarity may
be driven by recent accept rate (e.g., the 1/5 success rule),
by an accept-rate-tracked Gaussian covariance, by covariance
matrix adaptation (CMA), or by any other mechanism that
updates the proposal distribution based on the chain's
history during a single replicate.

**What disqualifies.** Variants that only change the
stationary proposal width (e.g., `proposal_std = 0.25`
instead of `0.5`) are not "richer"; they are hyperparameter
tweaks of basic SA and do not constitute a robustness check
of the framework. The class boundary is "does the proposal
distribution evolve during training?" If yes, it qualifies.
If no, it does not.

**Compute-parity commitment.** Matched-compute at
`TrainingBudget::Steps(16_000_000)` per replicate, identical
to basic SA. No adjustment for adaptation overhead — the
matched-complexity anchor is parameter count (`LinearPolicy(2,
1)`, `n_params = 3`), not evaluation count.

**Seed commitment.** Same `REMATCH_MASTER_SEED = 20_260_412`
as basic SA and CEM. The per-replicate seeds are
`splitmix64(REMATCH_MASTER_SEED + i)` for `i in 0..10`,
yielding the same physics noise sequences used in Ch 50's
and Ch 52's runs. This maximizes controlled comparison — any
difference in outcome between basic SA and richer SA on the
same replicate index is purely attributable to the algorithm's
proposal mechanism, with the physics noise held fixed.

**Measurement commitment.** Runs through `run_rematch` with
the dual-metric `TwoMetricOutcome` pipeline from `086c04c8`,
producing `(best_ci, best_outcome, final_ci, final_outcome)`
per the amended pipeline. `BOOTSTRAP_RNG_SEED` is
`0xB007_0057_00AA_0055` unchanged.

### 2.3 Parallel Tempering

**Class commitment.** PT runs `K` SA chains at temperatures
`T_1 < T_2 < ... < T_K`, with Metropolis-criterion swap moves
between adjacent chains at regular intervals. The chain
returned as the policy artifact for each replicate is the
lowest-temperature chain (chain 1). The values of `K`, the
temperature ladder, and the swap interval are Ch 53 §2
sub-decisions resolved in the PT code commit. Reasonable
implementations of the class qualify; exotic ones (e.g.,
adaptive temperature ladders, replica-exchange variants
beyond classical PT) should be flagged in the code commit's
message.

**Compute-parity commitment.** Total compute across all `K`
chains matches basic SA's `16_000_000` env steps. Each chain
therefore receives `16_000_000 / K` training steps, rounded
down. This is the more conservative of the two options Ch 92
§3.b named (total-compute parity vs per-chain parity) and
stays faithful to Ch 32's compute-parity commitment. If a
future chapter wants to run PT at per-chain parity (`K ×
16_000_000` total) that is a different experiment and should
be Ch 54 or later, not a continuation of this chapter.

**Seed commitment.** Same `REMATCH_MASTER_SEED = 20_260_412`
for the replicate pool. Within-replicate seed threading for
the `K` chains is the PT code commit's sub-decision; the
pre-registration only binds the replicate pool seed.

**Measurement commitment.** Same as §2.2 — dual-metric
`TwoMetricOutcome` pipeline, `BOOTSTRAP_RNG_SEED =
0xB007_0057_00AA_0055`.

### 2.4 What both variants must share

Whatever the code commits pick for the byte-level parameters,
the two follow-up runs must both:

1. Use the same `REMATCH_MASTER_SEED`, so the replicate pool
   is aligned with Ch 52's basic-SA run and per-replicate
   comparisons are noise-controlled.
2. Use the same `BOOTSTRAP_RNG_SEED`, so the bootstrap CIs
   are deterministic and reproducible.
3. Produce `TwoMetricOutcome` through the amended `run_rematch`
   driver, so `best_reward` and `final_reward` classifications
   are directly comparable to Ch 52's.
4. Run at matched total compute of `16_000_000` env steps.
5. Use `LinearPolicy(2, 1)` with `n_params = 3` and init
   `[0.0, 0.0, 2.0]`, matching Ch 42 §6's matched-complexity
   and matched-init-condition anchors.

Any follow-up run that violates any of the five bindings
above does not qualify as a Ch 53 follow-up and cannot be
interpreted under §3's decision rule.

## Section 3 — The pre-committed decision rule

This is the chapter's core commitment. The following three
cases are pre-registered in advance of any follow-up run.
Whichever case the joint outcome of the two variants lands
in, §4 (written after the runs) applies the case's rule
mechanically. No additional interpretive discretion is
permitted at §4's time.

### 3.1 The decision variables

For each variant (richer-SA and PT), the dual-metric
pipeline produces a `final_reward` classification in
`{Positive, Null, Ambiguous}` per Ch 32 §3.3's rule. The
decision rule operates on these two classifications jointly.
The `best_reward` classifications are reported in §4 as
context but do not enter the decision rule. This asymmetry
reflects Ch 51 §2.3's commitment that `final_reward` is the
primary operationalization of Ch 30's question; Ch 53's
robustness check inherits the same commitment.

Let `C_rich` = richer-SA's `final_reward` classification.
Let `C_pt` = PT's `final_reward` classification.

### 3.2 The three cases

**Case Corroborate.** `C_rich == Positive` AND `C_pt ==
Positive` AND both variants' `final_reward` CI lower bounds
are strictly positive AND both variants' `final_reward` point
estimates are at least `+50` reward units.

The `+50` floor is not arbitrary. Basic SA's `final_reward`
point estimate was `+157.49` with a CI lower bound of
`+104.53`. A corroborating variant should land in the same
effect-size class, not merely be statistically positive. The
`+50` threshold is "half of basic SA's CI lower bound,
rounded down," chosen as a conservative lower bound on
"same order of magnitude as basic SA's advantage." The floor
is deliberately below basic SA's actual CI lower bound so
that a variant with an effect size genuinely comparable to
basic SA's (even on the weaker side of that comparison)
still counts as corroboration.

**Case Contradict.** `C_rich == Null` OR `C_pt == Null`. If
either variant fails to reject the null on `final_reward`,
the framework's class-level claim has a counterexample and
the case is Contradict. Both variants failing is strictly
stronger evidence of contradict than one failing, but the
rule does not distinguish; either triggers a framework
investigation.

**Case Mixed.** Any joint outcome not covered by Corroborate
or Contradict. This includes: at least one variant is
Ambiguous and none is Null; both variants are Positive but
at least one has a point estimate below `+50`; both variants
are Positive with CIs spanning zero on their lower bound
(technically impossible under Ch 32 §3.3's Positive
classification, listed here for completeness). The Mixed
case covers the space where the signal exists but does not
cleanly match basic SA's effect-size class.

### 3.3 What each case triggers

**Under Corroborate.** Ch 51 §4's Positive endorsement is
**strengthened**. Ch 53 §5 reports the corroboration, and
Ch 53 §6 adds a paragraph to Ch 51 §4.6's methodological
takeaway noting that the framework's class-level claim was
independently verified by two SA variants at matched
compute. No retraction of any prior chapter. The Ch 30 null
follow-ups remain moot (they were moot under Ch 51 §4.4 and
Corroborate does not change that). Ch 51 §4 and Ch 53 §6
together constitute the chapter-level endpoint for Ch 30's
D2c-SR question for the ml-chassis-refactor study.

**Under Mixed.** Ch 51 §4's Positive endorsement **stands**
per §2.3's rule applied to basic SA's data, but Ch 53 §5
opens an investigation into why the class-level framework
produces a weaker signal under the variant whose
`final_reward` fell into the Mixed zone. The investigation
is an open research question, not a concrete follow-up menu:
Ch 53 §6 proposes specific hypotheses for what could drive
the Mixed signal (adaptation overfitting, chain correlation
in PT, matched-compute shortfall under `K`-way splitting,
etc.), and a future chapter (Ch 54 or later) picks one to
investigate. Ch 51 §4 is not retracted. Ch 30's D2c-SR
question remains answered Positive under §2.3's rule applied
to the basic-SA evidence; Ch 53 adds an asterisk about
framework-generalizability.

**Under Contradict.** Ch 51 §4's Positive endorsement is
**not automatically retracted** — §2.3's rule was applied
honestly to basic SA's data, and the rule's output is a
function of the data it was applied to, not of data from
other variants. However, Contradict is a stronger signal
than Mixed and triggers a stronger response: Ch 53 §5
documents the contradict outcome, and Ch 53 §6 explicitly
opens the question "does Ch 51 §4's Positive endorsement
still read as the study's answer to Ch 30, or should a
future chapter retract it in favor of reopening Ch 30's
null-follow-ups menu under a framework revision?" The
question is open; Ch 53 cannot answer it, because answering
it requires diagnosing *why* the Contradict happened, which
is itself a research task the chapter does not pre-commit to
a specific form for.

**Under any case, Ch 51 §4 is never retracted by Ch 53
alone.** A retraction decision requires a future chapter
that (a) diagnoses the mechanism behind the Contradict or
Mixed signal, (b) argues that the diagnosis invalidates
§2.3's `final_reward`-primary commitment, and (c) explicitly
rewrites or supersedes §2.3. Ch 53's role stops at opening
the question. This is deliberate: retracting a pre-registered
endorsement should require more evidence than any single
robustness check can provide, and the retraction process
should be as disciplined as the original endorsement was.

### 3.4 What the rule does not commit

Ch 53 does not pre-commit what Ch 54 or later does with a
Mixed or Contradict outcome. The possibilities are
open-ended: investigation, retraction, framework revision,
or leaving the study at "Ch 51 §4 answered Positive for
basic SA and Ch 53 §5 flagged generalizability as an open
question." The next chapter's author picks based on what
Ch 53 §5 actually finds and on the study's broader
priorities at that time. Ch 53 only binds what the robustness
check *itself* does — it commits the decision rule, not the
post-decision trajectory.

## Section 4 — Reproducibility anchors

Ch 53 uses a four-commit sequence mirroring Ch 51 §2.5:

- **§§1-3 pre-registration commit** — the commit that lands
  this file with §§1-3 complete and §§5-6 as placeholders.
  Parent commit: `867b0b93` (Ch 51 §§3-4 + §2.5 anchor
  fill-in). Branch: `feature/ml-chassis-post-impl`.
- **Richer-SA digest commit** — lands the captured log
  from the richer-SA run as a new appendix
  `docs/studies/ml_chassis_refactor/src/5x-richer-sa-digest.md`
  (exact slot to be picked by the commit). Contains the
  dual-metric summary block verbatim from the log plus a
  per-replicate comparison table against Ch 52's basic-SA
  values. The richer-SA code changes live in whichever
  commit(s) land the variant in `sim/L0/opt/src/algorithm.rs`
  or a neighboring module; those are bundled into the digest
  commit or split at the author's discretion.
- **PT digest commit** — same shape as the richer-SA digest
  commit, for the PT run.
- **§§5-6 interpretation commit** — replaces this chapter's
  §§5-6 placeholders with real content: §5 names the case
  (Corroborate / Mixed / Contradict) and applies §3.3's
  rule; §6 draws the methodological conclusion for the
  study.

The hash placeholders above are filled in by the §§5-6
interpretation commit, which is self-referential for its
own hash in the same way Ch 51 §§3-4 was. Pre-registration
discipline is preserved by the ordering: this chapter's
§§1-3 land *before* any follow-up variant's code or run, the
digest commits land *between* §§1-3 and §§5-6, and the §§5-6
interpretation commit lands *last* with parents including
both digest commits.

### 4.1 Ch 53 anchor fill-ins

- **§§1-3 pre-registration commit** — `cde92f8c`
  (`docs(ml-chassis-study): add Ch 53 §§1-4 — robustness-check
  pre-registration`). Branch: `feature/ml-chassis-post-impl`.
  Parent commit: `867b0b93` (Ch 51 §§3-4 + §2.5 anchor fill-in).
  Landed before any richer-SA or PT code or run existed, which
  is what makes §§1-3 of this chapter a pre-registration.
- **Richer-SA digest commit** — `002d056a`
  (`docs(ml-chassis-study): Ch 54 richer-SA digest`). Captures
  the Ch 53 §2.2 follow-up run's dual-metric summary in a new
  Chapter 54 and records the per-replicate comparison against
  Ch 52's basic-SA numbers on the shared replicate seed pool.
  The richer-SA code (Rechenberg 1/5 success rule) landed in
  `c7aabcc7` with a self-review fix at `abf0f3aa`, both prior
  to this digest and prior to the production run at
  `/tmp/ch53_richer_sa.log`. Wall time 13,598.48s (~3h46m).
- **PT digest commit** — `7745d25b`
  (`docs(ml-chassis-study): Ch 55 PT digest`). Captures the
  Ch 53 §2.3 Parallel Tempering follow-up run's dual-metric
  summary in a new Chapter 55 and records the per-replicate
  comparison against Ch 52's basic-SA numbers on the shared
  replicate seed pool. The PT code (`K = 4` geometric ladder
  `0.5 → 50.0`, swap every epoch, total-compute parity) landed
  in `8f8e006f`, prior to this digest and prior to the
  production run at `/tmp/ch53_pt.log`. Wall time 13,342.65s
  (~3h42m).
- **§§5-6 interpretation commit** — this chapter's §§5-6
  commit, with parent `7745d25b`. Self-referential: the
  commit's own hash cannot be written into its own content,
  but `git log --reverse` over this file identifies it as the
  commit immediately following `7745d25b` that replaces the
  §§5-6 placeholders below with prose. Committed after §§1-3's
  decision rule was anchored and after both digest commits'
  numbers were captured, so §5's case classification applies
  §3.2's pre-registered rule rather than deriving one after
  the fact, and §6's interpretation applies §3.3's
  pre-registered response in the direction §3.3 points.

### 4.2 Hardware note

Ch 53 follow-up runs are expected to execute on a machine
other than the one Ch 50 and Ch 52 used for basic SA's run.
Reproducibility is a function of seeds and fixture code, not
of hardware; the floating-point numerics of `f64` arithmetic
on x86_64 and aarch64 agree to bit-for-bit identity for the
operations `sim/L0/opt` and `sim/L0/ml-bridge` use, so a
machine change does not threaten the reproducibility
anchors. Wall-time is a function of hardware and is not
part of the reproducibility contract; it is reported in §5
for context but not asserted.

## Section 5 — Results

The richer-SA follow-up ran for 13,598.48s (~3h46m) and
landed its dual-metric digest at commit `002d056a` (Ch 54).
The PT follow-up ran for 13,342.65s (~3h42m) and landed its
dual-metric digest at commit `7745d25b` (Ch 55). Both ran on
the same MacBook Pro as Ch 52's basic-SA baseline —
§4.2's hardware note anticipated a second-machine execution,
but in practice both runs fired on the Ch 52 host, so the
§4.2 cross-machine reproducibility claim is not exercised
here and remains an untested inheritance from Ch 23 §3.

§3.1 defines `C_rich` as richer-SA's `final_reward`
classification and `C_pt` as PT's, with the `best_reward`
classifications reported as context. Extracting those values
from the Ch 54 and Ch 55 digest summary blocks:

### 5.1 Richer-SA decision variables

- `best_reward` bootstrap CI on `mean(SA) - mean(CEM)`:
  point = `−28.0207`, CI = `[−70.5583, +10.8916]`,
  `B = 10_000`
- `best_reward` classification: **Null**
- `final_reward` bootstrap CI on `mean(SA) - mean(CEM)`:
  point = `+155.2862`, CI = `[+110.3305, +197.1795]`,
  `B = 10_000`
- `final_reward` classification: **Positive**
- Joint `(best, final)` pair: `(Null, Positive)`
- `C_rich = Positive` (from the `final_reward` row per §3.1)

Folded-pilot expansion did not fire — the initial `N = 10`
bootstrap produced a decisive Null / Positive pair with
neither metric classifying Ambiguous, so `N` stayed at 10.

### 5.2 PT decision variables

- `best_reward` bootstrap CI on `mean(SA) - mean(CEM)`:
  point = `−3.2947`, CI = `[−35.1893, +26.1591]`,
  `B = 10_000`
- `best_reward` classification: **Null**
- `final_reward` bootstrap CI on `mean(SA) - mean(CEM)`:
  point = `+180.3329`, CI = `[+146.1572, +212.3541]`,
  `B = 10_000`
- `final_reward` classification: **Positive**
- Joint `(best, final)` pair: `(Null, Positive)`
- `C_pt = Positive` (from the `final_reward` row per §3.1)

Folded-pilot expansion did not fire for PT either — same
reason as richer-SA.

The `best_reward` Null classification on both variants
mirrors Ch 52 basic-SA's Null on `best_reward` (point =
`−23.5743`, CI = `[−76.4708, +24.1681]`). The robustness
check preserves basic SA's `best_reward` Null exactly as
§2.3's framework would predict: all three SA variants'
single-seed incumbents face the same population-vs-incumbent
asymmetry against CEM's `best_reward`, and all three land
in the same classification on the secondary metric. This
is not load-bearing for §3.1's decision (which operates on
`final_reward` only), but it is a consistency check that
the three SA variants are behaving as §2.3's framework
expects them to behave on the metric §2.3 declared
structurally disadvantageous to them.

### 5.3 Joint classification and case

`(C_rich, C_pt) = (Positive, Positive)`.

Applying §3.2's Case Corroborate preconditions
mechanically:

1. `C_rich == Positive` AND `C_pt == Positive` — yes, both
   classifications from §§5.1-5.2 are `Positive`.
2. Both `final_reward` CI lower bounds strictly positive —
   yes, `+110.3305` (richer-SA) and `+146.1572` (PT) are
   both strictly greater than zero.
3. Both `final_reward` point estimates ≥ `+50` — yes,
   `+155.2862` (richer-SA) and `+180.3329` (PT) are both
   well above the `+50` floor. PT's point estimate is in
   fact higher than Ch 52 basic-SA's `+157.4883` point
   estimate by `+22.84` units, and richer-SA's is within
   `−2.20` units of basic SA's.

All three preconditions are met. The joint outcome is
**Case Corroborate** per §3.2.

For symmetry-checking purposes, observe that neither
variant triggers the Case Contradict precondition
`C_rich == Null OR C_pt == Null` (both are Positive), and
neither lands in Case Mixed (no Ambiguous classifications;
no point estimate below `+50`; no Positive classification
with a CI spanning zero on the lower bound). The case
resolution is unambiguous under §3.2's rule.

## Section 6 — Interpretation

§3.3 commits that under Case Corroborate, Ch 51 §4's
Positive endorsement is strengthened, Ch 53 §5 reports the
corroboration, and Ch 53 §6 adds a paragraph to Ch 51 §4.6's
methodological takeaway noting that the framework's
class-level claim was independently verified by two SA
variants at matched compute, with no retraction of any prior
chapter and with the Ch 30 null follow-ups remaining moot as
they were under Ch 51 §4.4. This section does that
mechanically; no interpretive discretion beyond what §3.3
pre-committed is permitted.

### 6.1 What Corroborate means for Ch 51 §4

Ch 51 §4.3 read the amended re-run's `(Null, Positive)`
joint as Positive on Ch 30's "resolves the peak" question,
grounded in §2.3's commitment that `final_reward` is the
primary operationalization of Ch 30's converged-policy
language. Ch 51 §4.4 declared the Ch 30 null follow-ups
moot for Ch 30's question because the menu condition — a
Null classification on the primary metric — did not fire
for basic SA. Ch 51 §4.5 defended the Positive reading
over the Split alternative on three grounds (theoretical
asymmetry, §2.3's pre-registered rule, and §1.1's
consistency test that the same framework would have read
any other cell the same way). Ch 51 §4.6 stepped back and
named the broader procedural template Ch 51 as a whole
embodies.

Under Corroborate, none of those readings is retracted.
Ch 51 §4.3's Positive stands. Ch 51 §4.4's mootness
declaration stands — Ch 53 reused the Ch 30 follow-up
algorithms for a *different* question than Ch 30's, and
both variants' `final_reward` classifications coming back
Positive does not change the fact that basic SA's Ch 52
`final_reward` classification was also Positive and
therefore the Ch 30 menu condition never fired. Ch 51
§4.5's three-reason defense of Positive over Split is
unaffected — the defense was always independent of
compute availability and of other-variant results. Ch 51
§4.6's procedural template stands too, and §6.2 below
extends it.

The strengthening under Corroborate is evidential, not
procedural: §2.3's framework made a class-level claim
(the `final_reward`-primary reading holds for the SA
family against CEM on problems where the peak is above the
empirical peak range and CEM's population never resolves
to the incumbent), and Ch 53 provides independent
evidence for that class-level claim by running the framework's rule
on two additional SA variants under matched seeds,
matched compute, and matched measurement. Both variants
came back with `final_reward` CIs strictly above zero and
point estimates well above the `+50` floor. The
framework's class-level grounding is therefore not an
artifact of basic SA's specific per-replicate dynamics on
D2c-SR — it holds for richer-proposal SA under a
Rechenberg 1/5 adaptive controller *and* for PT chain 1
under a 4-way tempering split. Both variants' evidence
would have been enough to clear §3.2's Corroborate floor
on their own; having both clears it simultaneously is
the strongest evidence Ch 53's rule permits.

### 6.2 Strengthening Ch 51 §4.6's methodological takeaway

Ch 51 §4.6 named three conditions under which a
post-execution methodological finding can be honestly
amended into a pre-registered protocol: (1) the amendment
extends rather than retracts, (2) the interpretation
framework is pre-registered before the amended data, and
(3) the framework's rule is applied even when the rule
ratifies an inconvenient call. Ch 53 adds a fourth
condition, which the Corroborate outcome makes
demonstrable rather than aspirational:

4. **The framework's class-level grounding admits
   independent robustness-checking, and the
   robustness check is pre-committed to a decision rule
   before any follow-up run fires.** Ch 51 §2.3's
   framework grounds the `final_reward`-primary reading
   in a theoretical claim about the SA family as a
   *class* — population-vs-incumbent asymmetry under
   CEM's per-epoch `mean_reward` semantics — not in an
   empirical observation specific to basic isotropic SA.
   A class-level claim is a testable commitment: it
   predicts that other members of the class should behave
   the same way under the same framework. Ch 53 tested
   that commitment on two variants (richer-proposal SA
   under Rechenberg 1/5 adaptation, and Parallel
   Tempering chain 1 under a K=4 geometric ladder with
   Metropolis swap moves) chosen in advance for their
   mechanical distance from basic SA: richer-SA changes
   the proposal distribution's time-dependence, and PT
   changes the chain topology itself. Both variants
   returned `final_reward` classifications `Positive`
   with CI lower bounds and point estimates that cleared
   §3.2's pre-committed Corroborate preconditions. The
   framework's class-level claim is therefore not a rhetorical
   frame the framework wraps around a single-variant
   result; it is a predictive commitment that survived
   contact with two variants whose implementation details
   have nothing in common beyond belonging to the same
   class.

   This is the test Ch 51 §4.6 could not by itself
   perform: §4.6 could argue the framework's rule was
   applied honestly to basic SA's data, but could not
   demonstrate that the framework's *class-level*
   grounding was empirically sound. Ch 53 supplies that
   demonstration by pre-committing to a decision rule
   under which each of three outcomes (Corroborate,
   Mixed, Contradict) would have required a specific
   §3.3 response, then running the two variants, then
   applying the rule in the direction the rule points —
   which in this case happened to be Corroborate, but
   §3.2's rule would have been applied mechanically in
   any of the three directions. The pre-commitment is
   what distinguishes this robustness check from a
   post-hoc validation exercise, and is what makes the
   Corroborate outcome usable as evidence for §2.3's
   class-level grounding rather than as a self-fulfilling
   confirmation.

Future chapters that encounter their own load-bearing
post-execution methodological findings can cite Ch 51 §4.6
as the three-condition procedural template and Ch 53
§6.2 as the fourth-condition extension: when the framework
admits a class-level test, pre-commit to a robustness-check
decision rule before running the test, so that the test's
result is interpretable regardless of which direction it
lands in.

### 6.3 What Ch 53 does not do

Ch 53 does not retract anything. §3.3's "Ch 51 §4 is never
retracted by Ch 53 alone" clause is binding under Case
Corroborate for a stronger reason than under the other two
cases: there is nothing to retract because the evidence
strengthens rather than weakens the original reading. The
Ch 30 null follow-ups remain moot for Ch 30's question, as
they were under Ch 51 §4.4 — Ch 53 reused their algorithms
for a different question than Ch 30's and answered that
different question with a Corroborate verdict, which does
not reopen Ch 30's menu condition.

Ch 53 does not open a new research question. Under
Mixed or Contradict, §3.3 would have required §6 to propose
hypotheses or open the retraction question; under
Corroborate, §3.3 requires neither. The next chapter in
the study is not constrained to address anything Ch 53
raised, because Ch 53's outcome raises nothing that needs
addressing.

Ch 53 does not exhaust the robustness-check space. The
framework's class-level claim could be tested against
further variants (CMA-ES, adaptive-temperature PT,
population-annealing SA, etc.), against different
fixtures, or against different compute budgets. Ch 53
tested it against the two variants Ch 30 §3 had already
named, under the compute parity Ch 32 already committed
to, on the D2c-SR fixture already in the study. A future
author who wants a stronger corroborate-or-contradict
signal against a broader class definition is welcome to
pre-commit their own robustness check under Ch 53's
template.

### 6.4 Study-level endpoint on Ch 30's D2c-SR question

Per §3.3's "Ch 51 §4 and Ch 53 §6 together constitute the
chapter-level endpoint for Ch 30's D2c-SR question for
the ml-chassis-refactor study" clause, this section closes
the question. The study's answer to Ch 30's "does SA
resolve the peak at `kt_mult ≈ 2.55`?" is **Positive**,
grounded in Ch 51 §2.3's `final_reward`-primary framework,
applied through Ch 51 §4 to basic SA's `(Null, Positive)`
joint in Ch 52, and corroborated through Ch 53 by two
additional SA variants at matched compute whose
`final_reward` classifications independently meet §3.2's
Corroborate preconditions. Any future chapter that wants
to revisit the answer is welcome to do so, but it will
need to introduce a new mechanism (variant, fixture,
framework revision, or other) rather than re-derive a
different answer from the evidence Chs 51-55 already
established.

## Cross-references

- Ch 30 §3
  (`30-the-scientific-question.md:164-199`) — the original
  null-follow-ups menu, whose algorithms Ch 53 reuses for a
  different purpose
- Ch 50 §"What the metric measured"
  (`50-d2c-sr-rematch-writeup.md:167-298`) — the finding
  that motivated Ch 51 and by extension Ch 53
- Ch 51 §2.3 — the nine-cell framework whose class-level
  claim Ch 53 robustness-checks
- Ch 51 §4.5 — the three-reason defense of the Positive
  reading over the conservative Split, which Ch 53 §1.1
  reaffirms as compute-independent in its logic
- Ch 51 §4.6 — the broader methodological takeaway whose
  Ch 53 corollary under Corroborate is a triangulation
  statement
- Ch 52 (`52-amended-rerun-digest.md`) — the basic-SA run
  whose dual-metric data Ch 53's variants are compared
  against on the same replicate seed pool
- Ch 54 (`54-richer-sa-digest.md`) — the richer-SA anchor-2
  digest whose `final_reward` classification feeds §5.1's
  `C_rich` decision variable
- Ch 55 (`55-pt-digest.md`) — the PT anchor-3 digest whose
  `final_reward` classification feeds §5.2's `C_pt` decision
  variable
- Ch 92 §3 — the Ch 30 null follow-ups scope estimates,
  which Ch 53 §2 inherits byte-level parameters from in
  spirit (the code commits pick specifics)
- `sim/L0/opt/src/algorithm.rs` — where `Sa::new` lives and
  where richer-SA likely lands
- `sim/L0/opt/src/analysis.rs:466-557` — the `run_rematch`
  and `run_rematch_with_runner` driver Ch 53's follow-ups
  run through unchanged
- `sim/L0/opt/tests/d2c_sr_rematch.rs` — the fixture shape
  Ch 53's per-variant fixtures mirror
