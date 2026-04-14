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

### 4.1 Ch 53 anchor fill-ins (to be filled at §§5-6 commit time)

- **§§1-3 pre-registration commit** — (to be filled)
- **Richer-SA digest commit** — (to be filled)
- **PT digest commit** — (to be filled)
- **§§5-6 interpretation commit** — (self-reference)

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

*This section will be written after the richer-SA and PT
follow-up runs complete and their digests land as the second
and third anchor commits of §4's four-commit sequence. At
that time, §5 will report each variant's `TwoMetricOutcome`
summary (mirroring Ch 51 §3.1's shape), the joint
`(C_rich, C_pt)` classification pair, and the case named by
§3.2.*

## Section 6 — Interpretation

*This section will be written after §5. It will apply §3.3's
rule mechanically and state which of the three responses
(Corroborate / Mixed / Contradict) the case triggers. If
Corroborate: a paragraph strengthening Ch 51 §4.6's
methodological takeaway. If Mixed: specific hypotheses for
what could drive the weaker signal and a pointer to a future
chapter for investigation. If Contradict: the explicit
open question about whether Ch 51 §4's Positive endorsement
should be retracted by a future chapter.*

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
