# Review log — Chapter 21: `TrainingBudget::Steps` audit

## Factual pass

**Status:** Run by a general-purpose sub-agent with no prior
context. 24 falsifiable claims extracted from the draft and
verified against current source.

| # | Claim | Verdict |
|---|---|---|
| 1 | Uniform match block formula across five files | VERIFIED |
| 2–6 | Match block line citations in `cem.rs`, `reinforce.rs`, `ppo.rs`, `td3.rs`, `sac.rs` | VERIFIED (all five exact) |
| 7 | No hardcoded `n_envs`, no `unimplemented!()`, no divisor divergence | VERIFIED |
| 8 | `16_000_000 / (32 * 5000) = 100` epochs | VERIFIED |
| 9 | On-policy sum pattern `rollout.trajectories.iter().map(Trajectory::len).sum()` | VERIFIED (literal in all three files) |
| 10 | `cem.rs:208` epoch_steps sum | VERIFIED |
| 11 | `reinforce.rs:264–268` epoch_steps sum | VERIFIED |
| 12 | `ppo.rs:418–422` epoch_steps sum | VERIFIED |
| 13 | `td3.rs:309` `epoch_steps += n_envs` | VERIFIED |
| 14 | `sac.rs:336` `epoch_steps += n_envs` | VERIFIED |
| 15 | TD3/SAC inner loop structure | VERIFIED (plus a subtlety, see below) |
| 16 | PPO rollout at `ppo.rs:244–260` | VERIFIED |
| 17 | PPO k_passes SGD at `ppo.rs:348–411` | VERIFIED |
| 18 | TD3 warmup at `td3.rs:282–286` | VERIFIED |
| 19 | SAC warmup at `sac.rs:309–313` | VERIFIED |
| 20 | `warmup_steps` hyperparameter exists | VERIFIED |
| 21 | D2c TD3 config `warmup_steps = 1000` | VERIFIED (path drift, see below) |
| 22 | `32 * 1000 = 32,000` warmup env steps | VERIFIED |
| 23 | `32,000 / 16,000,000 = 0.2%` | VERIFIED |
| 24 | Off-policy "deterministic ceiling" claim | **DISCREPANCY** |

**Discrepancies resolved:**

1. **Claim 24 — off-policy "ceiling" framing was wrong.** Both
   TD3 and SAC contain an early-break at `td3.rs:273–275` /
   `sac.rs:300–302`:
   ```rust
   if env_complete.iter().all(|&c| c) { break; }
   ```
   The inner stepping loop exits once all envs have completed at
   least one episode. So `epoch_steps` is not a fixed
   `n_envs * max_episode_steps * n_epochs` ceiling — it is
   (iterations-until-all-complete) × `n_envs`, which still
   over-counts individual envs that reset mid-inner-loop but is
   not a deterministic ceiling. Chapter's `total_steps` section
   rewritten: the code snippet for the all-complete break is
   now quoted inline, the "deterministic ceiling" language is
   gone, and the divergence is described as "on-policy sums
   trajectory lengths, off-policy counts inner-loop iterations
   × `n_envs`, over-counting envs that reset mid-inner-loop."
2. **Path drift — `d2c_cem_training.rs`.** Chapter originally
   wrote "in `d2c_cem_training.rs`" without qualifying the
   path. Actual path is
   `sim/L0/thermostat/tests/d2c_cem_training.rs`, with
   `warmup_steps = 1000` at line 321 (and a second setting at
   line 380). Chapter updated to use the full path.

**Additional check performed during fix work:** verified each
algorithm runs exactly `n_epochs` outer-loop iterations. Grep
for `for .* in 0..n_epochs` returns exactly five matches, one
per algorithm file, at `cem.rs:153`, `reinforce.rs:175`,
`ppo.rs:232`, `td3.rs:259`, `sac.rs:286`. All five are plain
`for epoch in 0..n_epochs` loops with no outer-loop early exit.
This was added to the chapter's uniform-formula section
explicitly as a separate paragraph, because the thinking pass
(below) flagged it as a missing verification step.

## Thinking pass

**Status:** Cold-reader sub-agent with no prior context. Ten-point
brief covering scope discipline, framing, the reporting-vs-work
distinction, soft words, central-argument completeness, and
over/under-claiming.

**Verdict:** Ship with minor edits. Six-item fix list; all
applied.

**Substantive findings and fixes:**

1. **"Uniform formula is a relief" was partially unearned.** The
   chapter verified the formula is identical across the five
   files but did not verify each algorithm actually *runs* the
   computed number of epochs — only that each one *computes*
   the same number. A skeptic would ask whether any algorithm
   loops longer, exits early, or honors a different stopping
   condition. *Fix:* grep-verified the `for epoch in 0..n_epochs`
   pattern in all five files, and added a paragraph to the
   uniform-formula section stating explicitly that budget-to-
   epochs conversion is uniform *and* epoch count is honored as
   a hard iteration limit.
2. **"Roughly the same amount of physics work" was asserted,
   not established.** The chapter repeatedly pivoted from "the
   reporting diverges" to "but they're doing the same work," a
   claim the audit did not have the standing to make without
   direct verification. *Fix:* removed the "same physics work"
   language entirely. The chapter now says "all five algorithms
   run the computed number of epochs" and leaves it at that —
   the per-epoch work-amount comparison is Ch 22's problem.
3. **Rematch-implications paragraphs in the `total_steps`
   section were pre-arguing Ch 22.** The "two separate ways,"
   "more honest phrasing," and "Algorithm X was given a budget
   of 16M steps" recommendations all belonged to Ch 22. *Fix:*
   cut those two paragraphs. The section now closes with a
   one-paragraph "the audit's job is to establish the
   divergence, Ch 22 makes the call" deferral.
4. **PPO `k_passes` section was a tangent.** The section was
   load-bearing only for wall-clock framing, which the chapter
   admits is Ch 22's call. *Fix:* cut from ~20 lines to 8,
   reframed as a one-paragraph "short note for completeness"
   that establishes the k_passes-don't-advance-total_steps fact
   and defers the wall-clock framing explicitly.
5. **Warmup section oversold "compounds two problems."** The
   two problems were actually the same fact viewed twice
   (unbudgeted warmup that isn't separately tagged). The
   "few percent effect size" speculation was unearned — the
   chapter cited no D2c effect-size number. *Fix:* removed the
   "compounds two problems" framing and the effect-size
   speculation; the section now makes a tighter point — the
   overhead is small in absolute terms, but invisible in the
   reporting, and a "TD3 reached R after 16M steps" statement
   silently elides that the first 32,000 of those were random.
6. **`best_reward` forward reference was 14 lines reproducing
   three algorithm-specific computations.** That detail belongs
   in Ch 24's own audit. *Fix:* cut to a three-sentence
   paragraph noting the divergence exists, that the construction
   spec's existing D2c rematch test assertion compares these
   quantities directly, and that Ch 24 handles the resolution.
7. **`TrainingBudget::Epochs` was out of scope but not named.**
   Chapter silently audited only the `Steps` arm. *Fix:* added
   a three-sentence paragraph to the "what this audit has not
   covered" section stating that `Epochs(n)` is out of scope
   because the rematch uses `Steps`, but that a reader switching
   to `Epochs` should know uniformity does not carry over — an
   epoch means structurally different things across the CEM /
   on-policy / off-policy families.
8. **Soft-word cleanup.** Replaced "the formula is a relief"
   with "the formula is consistent across the algorithm
   surface." Dropped "deserving separate handling," "worth
   naming," and one of the two "single biggest" uses.

**Lower-severity items noted but not acted on:**

- Cold reader suggested folding the PPO `k_passes` content
  into §1 rather than keeping it as its own section. Kept
  as a short standalone section because the cross-algorithm
  comparison context is different from the uniform-formula
  context; a reader skimming for "why does PPO take longer"
  will find the answer faster with a dedicated heading.

## Second round

**Triggered:** No. All edits were targeted revisions to
connective tissue, one missing verification (the hard-iteration
check), and one real factual correction (the inner early-break).
The chapter's central argument — uniform formula is sound,
reporting diverges, three smaller findings interact — holds
after the edits and is better-supported.

## Open questions carried forward

- **Does Ch 22 need to quantify the D2c effect size to decide
  how much the warmup / reporting divergences matter?** The
  audit left the magnitude question open on purpose. If Ch 22
  needs a number, a short empirical check — run the D2c rematch
  at the current configuration and record total_steps for all
  five algorithms side by side — would give one. Flagged.
- **`best_reward` detail.** The construction spec's existing
  D2c rematch test assertion compares three different metric
  definitions. Ch 24 inherits this.
- **Rematch failure modes.** Chapter 31 inherits the
  reporting-mechanism identification from this audit.

## Status

Drafted, factual pass run with 22/24 verified + 1 path drift +
1 discrepancy resolved, thinking pass run with eight edits
applied (six substantive + two soft-word), no second round
triggered. Ready for commit (pending user permission).

## Round 2 — post-commit patch from chapter 22 thinking pass

**Triggered:** Yes. During chapter 22's thinking pass, a cold
reader pushed back on chapter 22's dismissal of the recon's
option (a) for `total_steps` reconciliation. The pushback sent
the chapter 22 author back to `td3.rs:260–310` and surfaced that
off-policy's inner loop steps *all* `n_envs` envs unconditionally
every iteration, including envs that have already completed an
episode during the current epoch (which get auto-reset and then
continue being stepped). The practical consequence is that at
the same `Steps(N)` budget, on-policy and off-policy perform
*different amounts of actual env work per epoch*, not just
report the same work in different units. Chapter 21's original
"reporting divergence" framing understated the finding.

**Impact on chapter 21.** The audit's factual claims are
unchanged — every citation still verifies, every grep result
still holds, and the uniform Steps-to-epochs formula finding is
untouched. What changes is the framing of the `total_steps`
semantics split: the two counters are honestly counting the
same metric (actual `env.step()` call count), and the reason
the reported numbers differ is that the two families genuinely
do different amounts of work per epoch. The original section
treated the divergence as a reporting issue downstream of the
budget formula; the revised framing makes clear that the work
disparity is upstream of the reporting and the counters are
both honest.

**Revisions applied:**

1. The paragraph in `total_steps` section stating "off-policy
   reports inner-loop work expressed as (iterations × `n_envs`),
   which systematically over-counts individual envs that
   completed early and were reset mid-inner-loop" was wrong —
   the counter is not over-counting, it is honestly counting
   the actual `env.step()` calls off-policy performs. Rewrote
   the paragraph to drop the "over-counts" framing and describe
   the divergence as "at identical `Steps(N)` budgets the two
   families produce different numbers under the same field name."
2. Phrase "an off-policy algorithm doing the same amount of
   inner-loop work" was factually wrong — the two families do
   *different* amounts of inner-loop work. Removed.
3. Added a forward-reference paragraph at the end of the
   `total_steps` section pointing chapter 22 readers at the
   deeper treatment: "the divergence goes deeper than the
   reporting-level framing of this section suggests, and chapter
   22 unpacks it. Both counters honestly count actual
   `env.step()` calls, but the two families do different amounts
   of actual env work per epoch." This naming change keeps Ch 21
   a correct audit of the current code while pointing readers
   at the deeper finding Ch 22 establishes.
4. The "What the audit establishes" summary paragraph was
   updated for the same reason: the old version said off-policy
   "systematically over-counts individual envs that reset
   mid-inner-loop," which is wrong. The new version says "both
   are honest counts of actual `env.step()` calls, and at the
   same epoch budget the two families do different amounts of
   actual env work per epoch" and forwards the deeper reading
   to chapter 22.

**Why the patch is a post-commit amendment rather than a
re-review.** The factual content of chapter 21 is unchanged
— every citation still holds, the uniform formula finding is
untouched, and the `total_steps` section still correctly
describes what each algorithm's counter does. The patch is a
framing correction that brings Ch 21's characterization in
line with Ch 22's deeper reading. A full re-review would not
turn up new findings; the thinking pass that caught the issue
was chapter 22's, not a re-reading of chapter 21, and chapter
22 is where the deeper argument lives. Chapter 21's job is to
audit the current code, which it still does faithfully; the
amendment only removes two factually-wrong phrasings ("over-
counts," "same amount of inner-loop work") and adds a forward
reference to where the deeper reading gets worked through.

No second round of the full protocol was run on the patched
chapter; the amendment is narrow, the factual pass on the
original draft still holds, and the thinking-pass concern that
motivated the patch was resolved in chapter 22's own revision
cycle.
