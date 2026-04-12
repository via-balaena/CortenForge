# Review log — Chapter 31: Failure modes

Chapter 31 is a descriptive synthesis chapter — Part 3's
ungated closer. It enumerates failure modes of the rematch in
four buckets (reader interpretation, reporting discipline,
effect-size-vs-noise, protocol/prerequisite) and points each
one at the upstream chapter that owns its protective mechanism.
Ch 31 makes no new design calls. Every failure mode it names
inherits from a committed chapter. Factual and thinking passes
ran in parallel per the descriptive-chapter pattern.

## Recon

Ch 31's recon was comparatively light because the chapter is
synthesis over already-committed material. Re-read Ch 22 in
full (the load-bearing source for §1.2, §2.1, §3.1), Ch 30 in
full (for §1.1, §1.3, §3.2), Ch 24 §5 (scope-discipline
section, for the Ch 31 inheritance handoff), and Ch 23 §2
(PPO exclusion, to keep Ch 31 from re-litigating).

**One targeted recon question** surfaced during the
recon-to-leans phase and was resolved before drafting: whether
CEM's `extra.elite_mean_reward` diagnostics key earns its own
§2.2 entry as a bucket-2 reader-confusion vector, or whether
it is CEM-internal scaffolding that should be demoted to a Ch
41 follow-up. Grep across the codebase for `elite_mean_reward`
returned four occurrences: `cem.rs:191` (creation),
`cem.rs:219` (insertion into `extra` map), `cem.rs:319` and
`cem.rs:354` (CEM's own unit tests, asserting presence and
finiteness only). No consumers in `competition.rs`, the D2c
test gate, sim-thermostat, or any rematch reporting code.
**Resolution:** demote. The unit mismatch does not surface to
a rematch reader. The original §2.2 candidate was replaced
with spread-metric labeling (sample std vs SEM
disambiguation), which is a real reader-confusion vector
inheriting from Ch 24 §4.6, and the elite_mean_reward item
was relocated to §4.2 as a Ch 41 cleanup pointer.

## Factual pass

**Status:** Run by a general-purpose sub-agent over 27
checkable claims. The chapter has a moderate falsifiable
surface — most claims are verbatim quotes from upstream
chapters or file:line citations, both of which are easy to
verify mechanically.

| # | Claim | Verdict |
|---|---|---|
| 1 | Intro inheritance summary (Ch 22 = compute parity, Ch 23 = pool + matched complexity, Ch 24 = per-replicate metric + aggregation) | VERIFIED |
| 2 | Ch 30 single-positive quote at lines 155-161 | VERIFIED |
| 3 | "SOTA-by-partition inflation" attributed to Ch 20 | PARTIAL — the phrase is Ch 30's coinage, not Ch 20's. Ch 20 establishes the underlying seed-partition inflation phenomenon. |
| 4 | Ch 22 line 383 delegation quote (closing scope-discipline section) | VERIFIED |
| 5 | Ch 22 line 316-319 `k_passes` paragraph quote | VERIFIED |
| 6 | Ch 30 null-outcome pre-emption quote (lines 170-174) | VERIFIED |
| 7 | Ch 30 allowed-follow-ups list (richer SA, Parallel Tempering on same SR task) | VERIFIED |
| 8 | Ch 22 picked option (b), option (a′) out of scope | VERIFIED |
| 9 | Ch 22 lines 204-208 writeup-omission risk quote | VERIFIED |
| 10 | `td3.rs:272` inner loop steps all n_envs every iteration regardless of completion | VERIFIED |
| 11 | Ch 24 §4.3 `replicate_best_rewards(task, algo) -> Vec<f64>` signature | VERIFIED |
| 12 | Ch 24 §4.5 SEM/bootstrap rejection on scope grounds | PARTIAL — §4.5 is about percentiles. SEM is rejected via Option-M omission in §4.2/§4.3; bootstrap is deferred at §5. |
| 13 | Ch 24 §4.7 zero-fallback contingency + recon claim about elite_mean_reward consumers | VERIFIED |
| 14 | Ch 22 warmup conditional (3% vs 0.5% effect size framing) | VERIFIED |
| 15 | Ch 22 lines 253-257 explicit gate-delegation quote | VERIFIED |
| 16 | `warmup_steps = 1000` at `d2c_cem_training.rs:321` | VERIFIED |
| 17 | Ch 23 §2 pool collapse to {CEM, SA}, neither has warmup | VERIFIED |
| 18 | Ch 30 ambiguous-outcome quote (lines 189-191) | VERIFIED |
| 19 | Ch 13 latent flakiness, Ch 14 grid, Ch 15 C-3 pick + install_per_env + prf.rs | VERIFIED |
| 20 | `sim/L0/core/Cargo.toml:37` `default = []`, no sim/ opt-ins | VERIFIED |
| 21 | Decision 1 line locations: cem.rs:209, td3.rs:490, sac.rs:543, REINFORCE/PPO no-ops | VERIFIED |
| 22 | CEM length-normalized fitness at `cem.rs:183`, Ch 24 §3.5 internal-split discussion | VERIFIED |
| 23 | Ch 24 §1 CEM mean_reward characterization | PARTIAL — paraphrased CEM divisor as "total steps"; actual is `n_envs` (the per-step normalization happens inside each fitness value before averaging across envs). |
| 24 | `cem.rs:219` insertion + `cem.rs:319` / `cem.rs:354` unit-test consumers | VERIFIED |
| 25 | Ch 23 §3.1/§3.2 matched-complexity anchor: `LinearPolicy(2,1)`, `n_params=3` | VERIFIED |
| 26 | Ch 24 §5 defers stat test, N, pilot to Ch 32 | VERIFIED |
| 27 | "skip the metric entirely" attributed to Ch 24 §4.7 | PARTIAL — quote lives in Ch 24 §3.7. §4.7 carries the contingency that depends on §3.7's recommendation. |

**Counts:** 23 VERIFIED, 4 PARTIAL, 0 WRONG. All four partials
are localized attribution drift (a section number, a chapter
label, a paraphrased divisor name) — none misrepresent what an
upstream chapter actually decided. All four were applied as
fixes in the single-round revision below.

**Verdict:** PASS with localized attribution drift. No source
code line numbers had drifted from the values in Ch 24's
review log. All load-bearing structural claims (pool
membership, matched-complexity anchor, Decision 1/2, warmup
conditional, bucket-4 prerequisites, failure-mode delegations)
verified verbatim against upstream files.

## Thinking pass

**Status:** Cold-reader sub-agent, eight-point brief for
descriptive chapters. The brief pushed on the four-bucket
taxonomy, the §2.2 substitution, the inheritance accuracy
across all four buckets, and the soft-word calibration on
claims about reader behavior.

**Verdict:** Ship with edits. Two MUST-FIX findings, seven
SHOULD-FIX, four MINOR. All thirteen applied on a single
round, alongside the four factual-pass partials.

**MUST-FIX findings and fixes:**

1. **§2.1 numerical-agreement error.** The paragraph beginning
   "The protective mechanism is a writeup-level guard" claimed
   "the two columns agree numerically only for the on-policy
   family (within the small warmup discrepancy treated in
   §3.1); for the off-policy family, the actual count is
   larger than the nominal budget by a predictable factor."
   This is wrong in three distinct ways, all of which Ch 22's
   own numbers contradict: (a) on-policy ~6.4M vs nominal 16M
   is a ~60% gap, not a warmup-sized one; (b) off-policy ~9.6M
   is *smaller* than 16M, not larger; (c) the warmup applies
   to TD3/SAC (off-policy), not on-policy. The chapter's own
   example numbers two paragraphs later directly contradict
   the framing, and a careful reader catches the contradiction
   in ten seconds. *Fix:* rewrote the paragraph. New framing
   names the nominal value as a budget-formula input (not a
   predicted call count), explains why both families come in
   below nominal (on-policy stops at `done`, off-policy pads
   past `done` but not to the full `max_episode_steps`
   ceiling), and redefines the disparity the column guards
   against as the gap between the two families' actuals (~6.4M
   vs ~9.6M), not a gap to nominal.

2. **§1.2 inheritance-strengthening error.** The paragraph
   beginning "This is the failure mode Ch 22 explicitly
   delegated to Ch 31" overstated Ch 22's hand-off. Ch 22 line
   383 says "chapter 31 *may need* to name it" — soft. Ch 31's
   "explicitly delegated" is too strong. Contrast with §3.1's
   warmup gate, where Ch 22 line 253 actually says "chapter
   31's gate is what makes this conditional explicit" — that
   IS explicit delegation, and §3.1's "explicitly delegated"
   wording is accurate. *Fix:* replaced "explicitly delegated"
   with "Ch 22 flagged this failure mode for Ch 31 in its
   closing scope-discipline section: ... The hand-off is soft
   on Ch 22's side ('may need to name') and Ch 31 takes the
   option, broadening the pre-emption beyond Ch 22's
   PPO-specific `k_passes` framing." Also added a forward
   reference to the new Section 5 bullet that flags the
   broadening as Ch 31's framing rather than direct
   inheritance.

**SHOULD-FIX findings and fixes:**

3. **§2.2 invented Ch 32 n-range projection.** Ch 32 does not
   yet exist; the range Ch 24 §4.6 actually projects is 3 to
   10. *Fix:* changed "(`n = 5` to `n = 10` per Ch 32's later
   judgment), `√n` is 2.2 to 3.2" to "(`n = 3` to `n = 10`),
   `√n` is 1.7 to 3.2", and re-attributed the source to Ch 24
   §4.6.

4. **§2.2 sin-of-brevity comparative reviewer-behavior claim.**
   The original paragraph claimed §2.2 "is realistic in a way
   that §2.1 is not" and that "careful writeup review will
   catch" §2.1 but not §2.2 — a substantive new claim about
   reviewer behavior with no defense. *Fix:* rewrote to a
   neutral framing of the two failure shapes' realization
   patterns (§2.1 is structurally checkable in a single review
   pass; §2.2 requires per-site discipline because each
   reporting site is a fresh opportunity to omit the label).
   Dropped the comparative claim.

5. **§1.2 generalization not flagged in §5.** The §1.2
   subsection generalizes Ch 22's PPO-specific `k_passes`
   note to a class-wide algorithm-selection-guide pre-emption.
   That generalization is a new framing decision, not a direct
   inheritance, and §5 did not flag it. *Fix:* added a §5
   bullet naming the generalization explicitly as Ch 31's
   framing rather than Ch 22's, and added a forward reference
   from the §1.2 paragraph to the new §5 bullet.

6. **§3.1 moot-gate editorial call not flagged in §5.** §3.1
   notes the warmup-overhead gate is moot for the post-Ch-23
   `{CEM, SA}` pool, then argues the gate survives in Ch 31
   for forward-compatibility reasons. That's a real Ch 31
   editorial decision (keep a moot gate in a live index
   because its shape generalizes), and §5 did not flag it.
   *Fix:* added a §5 bullet explicitly naming this as the
   only Ch 31 editorial call that is not pure inheritance.

7. **§4.4 "Ch 32 is ungated" parenthetical was a study-meta
   claim without a citation.** The status is true (per the
   project memory's protocol notes) but the chapter did not
   source it. *Fix:* dropped the parenthetical claim and
   rewrote the sentence to avoid the meta-claim entirely. The
   prerequisite framing (conceptual rather than code-level)
   is preserved.

8. **§4.2 elite_mean_reward "doc-update" weakened Ch 24's
   open choice.** Ch 24 §5 leaves the choice between renaming
   the key outright and updating the doc comment side-by-side
   as a Ch 41 cleanup call. §4.2's "the same 'Ch 41
   doc-update' line item" weakened that to a specific
   doc-update commitment. *Fix:* changed to "Ch 41
   rename-or-doc' follow-up Ch 24 §5 already specified" and
   added a parenthetical noting that Ch 24 leaves the choice
   open.

9. **§2.2 hoist of elite_mean_reward demotion reasoning.**
   The §2.2 closing paragraph mentioned the demotion but did
   not point at §4.2 for the inheritance-bookkeeping line
   item. *Fix:* added "see §4.2 for the inheritance
   bookkeeping line item" to the §2.2 closing paragraph.

**MINOR findings and fixes:**

10. **§4.1 "session 4" is inside-baseball.** Replaced with
    "Ch 15's post-commit conversation" — a phrasing that
    survives outside the project memory's session numbering.

11. **§1.1 "median reader" too strong.** Replaced with
    "plausible reader, not an exotic one" and "plausible
    misreading."

12. **§3.2 "never a hybrid" too strong.** Replaced with "is
    labeled as one of the three Ch 30 outcomes; hedged
    language like 'weak positive' or 'near-null' is dispatched
    explicitly to the ambiguous bucket rather than offered as
    a fourth option."

13. **§4.1 workaround framing.** Added a §5 bullet naming the
    "do not enable `--features parallel`" workaround as not a
    Ch 31 recommendation, only an existence-flag. Whether the
    rematch should actually run before Ch 40 lands is an
    execution-sequencing question Ch 31 does not decide.

**Factual-pass partials applied alongside the thinking-pass
fixes:**

14. **F3 (SOTA-by-partition attribution).** Changed "the
    SOTA-by-partition inflation error Ch 20 warned about" to
    "the seed-partition inflation error Ch 20 established, in
    the form Ch 30 labels as SOTA-by-partition inflation."
    Splits the underlying phenomenon (Ch 20) from the label
    (Ch 30).

15. **F12 (§4.5 SEM/bootstrap attribution).** Rewrote the §2.2
    sentence to say "Ch 24 considered and rejected standard
    error of the mean (`std / sqrt(n)`) by omission from
    `SeedSummary` at §4.2 and §4.3 (Option M deliberately
    ships with `n`, `mean`, and `std_dev` only — no stderr
    field), and deferred bootstrap confidence intervals to a
    future chapter at §5, both on scope grounds."

16. **F23 (CEM mean_reward divisor).** Rewrote the §4.2
    parenthetical to "CEM's `mean_reward` is per-step-normalized
    (each trajectory's total reward divided by trajectory
    length, then averaged across `n_envs` trajectories)."
    Captures the per-step nature without misnaming the
    outermost divisor.

17. **F27 (§4.7 → §3.7 attribution).** Rewrote the §5 bullet on
    the REINFORCE/PPO zero-fallback to attribute the "skip the
    metric entirely" quote to Ch 24 §3.7 (where it lives) and
    name §4.7 as carrying the contingency that depends on
    §3.7's recommendation.

## Self-review patches

After applying the 17 fixes, a self-review skim caught two
small editing slips left over from the fix application — both
of the form "the fix expanded a sentence but didn't prune the
now-redundant phrase from a neighboring sentence":

- **§1.1 closing paragraph.** The T11 fix replaced "is the
  median reader, not the careful one" with "is a plausible
  reader, not an exotic one," but the prior sentence already
  said "not an exotic one" about the misreading itself. Two
  occurrences of "not an exotic one" in 25 words. *Patch:*
  dropped the first occurrence and merged the two sentences
  into one flow.
- **§3.2 gate's-shape paragraph.** The T12 fix replaced "never
  a hybrid" with "is labeled as one of the three Ch 30
  outcomes; hedged language like 'weak positive' or 'near-null'
  is dispatched explicitly to the ambiguous bucket," but the
  immediately preceding sentence already used "weak positive"
  / "near-null" as examples. Two occurrences of the same
  example list in adjacent sentences. *Patch:* dropped the
  example list from the second sentence and tightened the
  framing to "the rematch's headline conclusion is one of the
  three Ch 30 outcomes, and hedged language is dispatched
  explicitly to the ambiguous bucket rather than offered as a
  fourth option."

Both patches are cleanups of T11/T12's downstream effects, not
new findings. No factual claims change.

## Round count

Single round of formal fixes (the 17 above), plus the two
self-review patches. No second factual or thinking pass.
The 17 findings split into 4 must-fix-equivalent (the §2.1
numerical error and the §1.2 strengthening from the thinking
pass, plus the §4.5 attribution and §3.7 attribution from the
factual pass — both of which are "factually wrong as cited"
even though the factual pass labeled them PARTIAL because the
substance was right). All localized edits, no structural
rewrites. The four-bucket taxonomy held. The §2.2 substitution
held (the spread-metric labeling failure mode is real, the
section is tighter after the sin-of-brevity removal but not
thin enough to fold).

## What changed structurally

- §2.1's writeup-guard paragraph rewritten end to end.
- §1.2's "explicitly delegated" claim replaced with a softer
  hand-off framing plus a forward reference to a new §5 bullet.
- §2.2's sin-of-brevity comparative claim removed; section is
  ~60 lines shorter.
- §5 added three new bullets (§1.2 generalization, moot-gate
  editorial call, parallel-feature workaround) and rewrote the
  zero-fallback bullet to fix the §4.7 → §3.7 attribution.

## What did not change

- The four-bucket taxonomy.
- The §2.2 substitution (spread-metric labeling stayed; the
  recon-collapsed elite_mean_reward original candidate stayed
  demoted to §4.2's Ch 41 cleanup pointer).
- The §3.1 moot-gate decision to keep the entry for
  forward-compatibility reasons (now flagged in §5 as the only
  Ch 31 editorial call).
- The §4.3 SA-parameterization-drift entry (the recon-to-leans
  pressure-test about whether it duplicates Ch 42 was settled
  before drafting; the chapter keeps it on internal-consistency
  grounds with §4.1 and §4.2).
- All file:line citations and source-code claims (the factual
  pass found zero source-line drift).

## Verdict

Ship. Single round of edits applied all 17 findings. The
chapter is the failure-modes index it set out to be — no new
design calls, every entry inherits from a committed chapter,
every protective mechanism points at an upstream chapter's
gate or writeup discipline. Chapter 31 closes Part 3 alongside
Ch 32 (still pending) and is ready for commit pending user
approval.

Pending Ch 31's commit, the next pieces of work are Ch 32
(Hyperparameter sensitivity, the other Part 3 closer) and the
Part 4 execution PR plans (Ch 40, 41, 42, all gated). Ch 31
does not change the priority order among those — they remain
independent work streams.
