# Review log: Ch 24 Result semantics

Chapter type: **argument chapter**. Makes two high-stakes design
decisions — the per-replicate reduction shape (`EpochMetrics::mean_reward`
semantics across five algorithms) and the across-replicate
aggregation surface on `CompetitionResult`. Phase 3 **gated
chapter** — paused for user review before commit per the Ch 01
protocol. Last gated chapter in Part 2.

Passes run **sequentially** per the Phase 2 process note for
design chapters: factual pass first, discrepancy fix, then
thinking pass.

## Pre-draft recon

Ch 24 required direct source recon before drafting — the apples-
to-oranges characterization of `mean_reward` across the five
algorithms was from prior memory, not from a fresh read of the
source, and the chapter's argument had to rest on line-level
verification of where and how each algorithm populates
`mean_reward` on `EpochMetrics`. Recon was performed in-session
via direct Grep and Read against `sim/L0/ml-bridge/src/` and
returned the following load-bearing findings:

1. **`EpochMetrics` and the rollout helper.** `EpochMetrics` at
   `algorithm.rs:32-43` has five named fields (`epoch`,
   `mean_reward`, `done_count`, `total_steps`, `wall_time_ms`)
   plus an `extra: BTreeMap<String, f64>` for diagnostics.
   `RunResult::best_reward()` at `competition.rs:45-50` is a
   twelve-line `max`-over-`metrics[].mean_reward` helper.
   `collect_episodic_rollout` at `rollout.rs:82` returns raw
   per-step rewards per trajectory and does no aggregation — the
   apples-to-oranges lives in algorithm-side code, not in the
   rollout library.

2. **Per-algorithm `mean_reward` computation sites.**
   - CEM at `cem.rs:209`: `fitness.sum() / n_envs` where fitness
     at `cem.rs:183` is `total / traj.len().max(1)` — per-step
     mean, length-normalized. **Unit: reward-per-step.**
   - REINFORCE at `reinforce.rs:274`: `total_reward / n_envs`
     where `total_reward` sums per-trajectory totals. **Unit:
     reward-per-episode, constant denominator.**
   - PPO at `ppo.rs:428`: line-for-line identical to REINFORCE.
   - TD3 at `td3.rs:490`: `epoch_rewards.sum() / epoch_rewards.len()`
     where `epoch_rewards` is populated at `td3.rs:337-343` only
     from first-completed episodes per env. **Unit:
     reward-per-episode, variable denominator.**
   - SAC at `sac.rs:543`: line-for-line identical to TD3.

3. **Refined characterization.** Previous recon framed the audit
   as a three-way split (per-step mean / per-episode total /
   mean-over-completed-episodes). Line-level reading showed the
   correct framing is a **two-way split with a denominator
   wrinkle** — CEM is the only length-normalized algorithm; the
   other four agree on per-episode total and differ only in
   denominator choice (REINFORCE/PPO use constant `n_envs`;
   TD3/SAC use variable `epoch_rewards.len()`). This refinement
   mattered for Decision 1's scope: the fix is smaller than the
   three-way framing made it sound.

4. **Zero-fallback minor flag.** REINFORCE at `reinforce.rs:214-222`
   and PPO at `ppo.rs:314-322` both emit
   `EpochMetrics { mean_reward: 0.0, ... }` when `n_samples == 0`.
   This is a "no samples collected" short-circuit but reports a
   value a max-based `best_reward()` would treat as real. Flagged
   as a latent defect for Ch 41 execution-PR cleanup; out of
   Decision 1 scope.

The two-decision structure was briefed to the user after recon
for a leans-check, with both decisions surfaced as substantive
forks: Decision 1 as (a)/(b)/(c) reconciliation options with
a lean on (a) fix-the-algorithms, Decision 2 as an S/M/L scope
spectrum with an honest 60/40 lean on M over S. The user locked
Decision 1 as proposed (leave CEM elite-selection fitness
length-normalized; change only `mean_reward` reporting) and
delegated Decision 2 to the assistant. Decision 2's final pick
(minimal struct, silent filter-out of `None` replicates, sample
std via Bessel's correction) is an assistant call under the
standing delegation.

## Factual pass

**Status:** Run via Explore agent against the full list of
`(recon-reported)` citations. **30/30 source-level claims
verified** plus cross-chapter claims against Ch 20 and Ch 23.

| # | Claim | Result |
|---|-------|--------|
| 1 | `algorithm.rs:32-43` `EpochMetrics` struct definition | Verified |
| 2 | `competition.rs:45-50` `RunResult::best_reward()` function body | Verified |
| 3 | `rollout.rs:82` `collect_episodic_rollout` signature | Verified |
| 4 | `cem.rs:177-185` fitness block (per-step mean arithmetic) | Verified |
| 5 | `cem.rs:183` `total / traj.len().max(1)` expression | Verified |
| 6 | `cem.rs:209` `mean_reward` assignment | Verified |
| 7 | `cem.rs:219` `elite_mean_reward` extras key | Verified |
| 8 | `reinforce.rs:175-292` train loop outer range | Verified |
| 9 | `reinforce.rs:269-274` total_reward + mean_reward block | Verified |
| 10 | `reinforce.rs:274` `mean_reward` assignment | Verified |
| 11 | `reinforce.rs:214-222` zero-fallback short-circuit | Verified |
| 12 | `ppo.rs:232-458` train loop outer range | Verified |
| 13 | `ppo.rs:423-428` total_reward + mean_reward block | Verified |
| 14 | `ppo.rs:428` `mean_reward` assignment | Verified |
| 15 | `ppo.rs:314-322` zero-fallback short-circuit | Verified |
| 16 | REINFORCE/PPO line-for-line identical (`mean_reward` expressions) | Verified |
| 17 | `td3.rs:259` epoch loop start | Verified |
| 18 | `td3.rs:487-491` `mean_reward` computation | Verified |
| 19 | `sac.rs:286` epoch loop start | Verified |
| 20 | `sac.rs:540-544` `mean_reward` computation (identical to TD3) | Verified |
| 21 | CEM uses `collect_episodic_rollout` | Verified |
| 22 | TD3 inlines rollout (does not use helper) | Verified |
| 23 | SAC inlines rollout (does not use helper) | Verified |
| 24 | `RunResult::best_reward()` returns `Option<f64>` | Verified |
| 25 | All verbatim code blocks character-for-character | Verified |
| 26 | Ch 20:177-181 quote about per-algorithm scalar | Verified |
| 27 | Ch 23 inheritance (run_replicates, flat Vec, replicate_index) | Verified |
| 28 | Ch 23 Section 1.5 hook language | Verified |
| 29 | `competition.rs:36-41` `RunResult::final_reward()` cross-reference | Verified |
| 30 | Ch 22 fair-compute-parity cross-reference | Verified |

**Verdict:** Factual pass mostly clean. Three drifts and one
factual error fixed before the thinking pass.

**Fixes applied before thinking pass:**

1. **❌ Factual error — EpochMetrics field count.** Section 1.1
   said "four named fields" but listed five (`epoch`,
   `mean_reward`, `done_count`, `total_steps`, `wall_time_ms`).
   Fixed to "five named fields."

2. **⚠️ Drift — `td3.rs:259-275` → `td3.rs:259-284`.** The
   outer structure's code block extends further than the
   originally cited range. Widened to 259-284 to bracket the
   shown construct cleanly.

3. **⚠️ Drift — `td3.rs:312-351` → `td3.rs:335-351`.** The
   reward accumulation block starts at line 335
   (`env_episode_reward[i] += reward`), not line 312 (which is
   the start of the enclosing per-env `for` loop). Tightened
   to 335-351.

4. **Minor drift — `sac.rs:286-570` → `sac.rs:286-572`.** SAC
   train loop actually closes around line 572. Off-by-2 in the
   original citation, fixed.

**Non-verified items (by kind, intentional):**
- Argument-level claims about "matched REINFORCE's convention,"
  "Decision 1 is a bug fix, not an API growth," "the fix is
  smaller than the three-way framing" — these are Ch 24's own
  reasoning, not source-verifiable.
- Future-scope claims about what Ch 32 and Ch 41 will or will
  not do — deferred by design.
- The exact `d2c_cem_training.rs` line range the chapter
  mentions in Section 1.9; cited as "recon-reported, exact line
  range deferred to factual pass" in the draft and left that
  way because the exact range is not load-bearing and changes
  as the D2c test evolves.

## Thinking pass

**Status:** Run via general-purpose agent against the 10-point
brief for argument chapters. **10 findings surfaced.** Verdict:
*"ship with minor edits."* 0 must-fix, 5 should-fix, 5 minor.
All 10 applied; no second round triggered because none of the
findings revealed a structural issue that would require a
rewrite of the Section 1 audit or either decision.

### Findings applied

**Should-fix (5):**

1. **#1 — Line count framing in Section 3.2 / 3.4 is wrong.**
   The chapter summary said "Five train loops are touched" and
   "roughly five line-level changes across five files," but
   REINFORCE and PPO are explicit no-ops under Decision 1 and
   the actual edits are in three files (CEM, TD3, SAC).
   Counting honestly, Decision 1 is ~ten line-level changes
   across three files plus two three-line pre-loops. Fixed both
   the Section 3.2 "Three train loops are touched (CEM, TD3,
   SAC; REINFORCE and PPO are already correct)" reframing and
   the Section 3.4 "roughly ten line-level changes across three
   files" reframing. The "twenty lines total" conclusion still
   lands.

2. **#2 — PPO/D2c "second, independent reason" overclaim.**
   Section 2.2 framed Ch 24's unit-mismatch observation as a
   "second, independent reason to distrust D2c's read on PPO."
   Independence-of-mechanism is real (learning dynamics vs
   reporting metric), but independence-of-conclusion is not —
   both observations point at "D2c's PPO number does not mean
   what its name suggests." Reframed to "a corroborating
   observation on a separate mechanism axis" and added a
   sentence explicitly noting the two observations point the
   same direction. The section no longer claims the calculus
   changes; it claims there are two independent mechanisms
   that happen to converge on the same verdict.

3. **#3 — CEM internal-split rationalization was a fig leaf.**
   Section 3.5's "two paths of reasoning" for leaving
   elite-selection fitness length-normalized argued (path 2)
   that per-step mean is "the more *principled* selection
   criterion" — which undercut Decision 1's decision to
   standardize reporting on per-episode total. If per-step is
   more principled, why is the whole chassis moving to
   per-episode? The defense on "principle" grounds was the
   chapter having it both ways. Rewrote Section 3.5 to defend
   the pick on **scope grounds only**: Ch 24's scope is
   reporting comparability across algorithms, not the internal
   workings of any single algorithm's search, so Ch 24 declines
   to override CEM's selection criterion. The chapter now names
   the internal split as a **real cost**, not a "smaller
   confusion," and commits to having the execution PR document
   the split explicitly in a doc comment on the CEM train loop.

4. **#4 — Section 3.6 TD3/SAC partial-episode reasoning was
   thin plus missing a counterfactual.** "The reasoning is
   consistency" stopped short of saying *why* consistency is
   the right preference here. Upgraded to "Decision 1
   standardizes on REINFORCE's convention, and REINFORCE
   includes partial trajectories at full weight — standardizing
   on REINFORCE's convention means standardizing on truncated
   trajectories too, not just complete ones. Contributing 0
   would create a third convention." Also added a third-option
   counterfactual: "extend the inner loop until every env
   completes, ignoring `max_episode_steps`" — rejected on
   Ch 22 fair-compute-parity grounds. The section now engages
   the third option a reviewer would ask about and rejects it
   on the right axis.

5. **#5 — Section 4.7 contingency on Ch 41's zero-fallback
   handling.** Decision 2's silent filter-out of `None`
   replicates works cleanly *if* Ch 41's zero-fallback fix
   (Section 3.7) emits `None` or skips the metric entirely. If
   Ch 41 leaves the zero-fallback as `mean_reward: 0.0`, the
   filter-out becomes degenerate — real zeros pass through,
   fake zeros pass through, and true `None` runs get dropped.
   Added a one-paragraph contingency note at the end of 4.7
   naming the dependency explicitly and flagging that Decision
   2 needs to be revisited if Ch 41 breaks the assumption.

**Minor (5):**

6. **#6 — Soft-word audit.** Three flags: `*by design*, not by
   accident` → `by design` (the hedge was doing no work);
   `The cleanest fix is probably to skip the metric` →
   `The cleanest fix is to skip the metric` (the "probably" was
   ducking a sub-decision that was already clear); `so this is
   mainly a boundary-condition choice` → `so this is a
   boundary-condition choice, not a load-bearing one` (soft-
   selling a call that could be stated flat). All three flattened.

7. **#7 — Section 3.3 counterfactual (b) fourth bullet was a
   strawman.** The "cleanest form of (b) is effectively (a)
   with a Rust deprecation warning" bullet was doing
   argumentative work that the first three bullets already did.
   Dropped it. Counterfactual (b) now makes its case in three
   bullets without the weakest one dragging.

8. **#8 — Scope-discipline list was missing the
   `elite_mean_reward` naming item.** Section 5 flagged seven
   related non-decisions but did not mention that CEM's
   `extra.elite_mean_reward` diagnostics key — which continues
   to report per-step-mean under Decision 1 — now coexists with
   a `mean_reward` field in per-episode-total units, creating
   a mild naming collision a future reader will notice. Added
   as a scope-discipline bullet naming the doc-update obligation
   (Ch 41) and flagging the rename call as separately Ch 41's
   to make.

9. **#9 — Section 1.8 needed explicit agreement verification.**
   The "two-way split with denominator wrinkle" refinement is
   true, but the chapter did not explicitly state that it had
   been verified that REINFORCE's
   `trajectory.rewards.iter().sum()` and TD3's
   `env_episode_reward[i]` accumulator produce identical
   per-trajectory totals in the complete-episode case. A
   skeptic could reasonably ask "are you sure they agree up
   to the denominator, or is there a second-order accumulator
   difference you missed?" Added one sentence naming the check
   explicitly.

10. **#10 — Section 4.6 sample-std defense was over-specific.**
    The chapter argued for Bessel's correction on
    "bias-toward-optimistic-significance" grounds, which is a
    test-specific argument (true for mean±std gate rules, less
    clearly true for other tests Ch 32 might pick). Since
    Ch 24 deliberately does not pick the test, it should not
    argue for a denominator on grounds that implicitly assume
    one test over another. Reframed to convention-matching:
    "picking the population std would surprise every reader
    who knows the RL literature." The argument no longer
    depends on a test Ch 32 has not picked.

### No second round triggered

All 10 findings applied cleanly. None surfaced a structural issue
(e.g., a decision that should have gone the other way, a Section 1
finding that was load-bearing in a way the decisions missed, a
retroactive challenge to Ch 23's picks). The Ch 22 precedent — a
single-round thinking pass producing edits that sharpened the
chapter without rewriting its structure — applies here. The
revised chapter has the same two decisions, the same audit, and
the same scope-discipline closing; what changed is the argument
prose for each decision, now honest about costs (CEM internal
split), contingencies (Ch 41's zero-fallback handling), and
scope (sample-std convention over test-specific arguments).

## Notes for commit and follow-ups

**No retroactive Ch 23 patch.** Section 2.1 explicitly argues
Ch 24 does not invalidate any of Ch 23's three locked picks —
the `run_replicates` API shape, the PPO exclusion, and the
matched-complexity anchor are all unaffected by the unit-mismatch
finding. The Ch 21-post-commit-patch precedent (commit
`3e1ec0ff`) applies when a downstream finding makes an upstream
claim factually wrong, and Ch 24's finding corroborates Ch 23
rather than contradicting it.

**Decision 1 execution dependency.** The three-file fix (CEM,
TD3, SAC) is load-bearing for every execution PR that follows.
Ch 41 (PR 2) owns the implementation; this chapter specifies
the mechanical shape at line-level precision for each algorithm.
The 3.6 sub-decision on incomplete-env partial contribution is
the non-obvious part; the rest is denominator arithmetic.

**Decision 2 execution dependency on Ch 41.** Section 4.7's
contingency note makes Decision 2's silent filter-out explicitly
contingent on Ch 41 fixing the REINFORCE/PPO zero-fallback at
the source (Section 3.7). If Ch 41 leaves the fallback emitting
`0.0`, Decision 2's return type needs to be revisited. The
dependency is named in the chapter and flagged here so Ch 41
inherits it as a requirement rather than a suggestion.

**Delegation standing for the Decision 2 call.** Decision 2's
scope-spectrum pick (M over S and L), the sample-std convention,
the three-field `SeedSummary` shape, and the `Vec<f64>` return
type are all assistant calls under standing user delegation.
Decision 1 was explicitly user-locked ("yes let's go with your
suggestion"). Both will be flagged in the commit message.

**Ch 31 and Ch 32 cross-dependency.** Ch 24 leaves Ch 32 free
to pick any statistical test over the `replicate_best_rewards`
primitive. Ch 31 inherits Ch 24's scope-discipline list on which
rematch-adjacent concerns are deferred (D2c test rewrite, pilot
design, replicate count, bootstrap helpers). No contortion
expected.

**What Ch 24 does not resolve and names as downstream follow-ups:**

- Specific statistical test the rematch gate uses (Ch 32).
- Replicate count `N` (Ch 32, depends on pilot variance).
- Pilot design itself (Ch 32).
- Percentile / bootstrap helpers on `CompetitionResult` (Ch 32).
- D2c test gate rewrite (Ch 41, execution).
- REINFORCE/PPO zero-fallback cleanup (Ch 41, execution).
- `EpochMetrics::mean_reward` rustdoc update (Ch 41, execution).
- CEM `extra.elite_mean_reward` doc update / rename (Ch 41, execution).
- CEM elite-selection fitness reconciliation with reporting (out
  of scope, flagged for future iteration).
- TD3/SAC reporting-vs-training asymmetry documentation (Ch 41,
  execution).
- `RunResult::final_reward()` inherits Decision 1's fix for free;
  no separate pick needed.

## Gated review status

**Paused for user review before commit.** Per the Ch 01 protocol's
list of gated chapters, Ch 24 (result semantics) is flagged for
human review because Decision 1's three-file algorithm-level fix
is load-bearing for every downstream execution PR, and Decision
2's API surface (raw Vec primitive + `SeedSummary` convenience)
becomes chassis-level the moment the PR lands. The chapter has
cleared both passes on a single round with all 10 thinking-pass
findings applied. The user has seen the recon-to-leans report,
locked Decision 1 as proposed, and delegated Decision 2 to the
assistant. What needs the user's eyes on the final draft is
whether the revised Section 3.5 (CEM internal split as real cost
on scope grounds, not principle grounds) and Section 3.6 (the
three-option counterfactual with Ch 22 rejection of the
extend-the-inner-loop alternative) read cleanly, and whether
Section 4.7's contingency note on Ch 41 is the right level of
commitment for a dependency that spans two chapters.
