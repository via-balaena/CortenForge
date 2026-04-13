# Review log — Chapter 22: Fair compute parity

Chapter 22 is a design/decision chapter. It inherits its facts
from chapter 21 (committed) and makes five design calls for
the rematch protocol.

## Factual pass

**Status:** Run by a general-purpose sub-agent. Small falsifiable
surface — the chapter inherits from chapter 21 and makes few new
source-level claims.

| # | Claim | Verdict |
|---|---|---|
| 1 | `warmup_steps = 1000` at `sim/L0/thermostat/tests/d2c_cem_training.rs:321` | VERIFIED |
| 2 | Ch 21 established "Steps-to-epochs formula is uniform" | VERIFIED |
| 3 | Ch 21 established "all five algorithms run the computed number of epochs as a hard count" | VERIFIED |
| 4 | Ch 21 established "`total_steps` reporting diverges between on-policy and off-policy" | VERIFIED |
| 5 | Ch 21 established "PPO `k_passes` is a wall-clock factor, not a budget factor" | VERIFIED |
| 6 | Ch 21 established "TD3/SAC warmup overhead is ~0.2% and invisible in current reporting" | VERIFIED |
| 7 | `32 * 1000 = 32,000` warmup env steps | VERIFIED |
| 8 | `32,000 / 16,000,000 = 0.2%` | VERIFIED |
| 9 | Recon-named options (a)/(b)/(c) match the chapter's labels | VERIFIED |

**Verdict:** PASS. All feeder-chapter attributions faithful, all
arithmetic correct, no drift.

## Thinking pass

**Status:** Cold-reader sub-agent, ten-point brief focused on
the *decisions themselves* and the reasoning behind them (not
just prose). Seven substantive findings; all applied, some
triggering a deeper re-read that changed the chapter's framing.

**Verdict:** Ship with minor edits. One of the edits turned out
to require a structural rewrite of the `total_steps` section,
so "minor" understates the work — but the chapter's decisions
held after the rewrite.

**Substantive findings and fixes:**

1. **Missing sim-seconds alternative in the budget-unit
   dispatch.** A thermo-computing study whose substrate is
   physics simulation owes a reviewer an explicit dispatch of
   sim-seconds (MuJoCo rollout time) as a budget unit. *Fix:*
   added a paragraph noting that sim-seconds collapses to env
   steps at fixed `dt` and fixed `max_episode_steps` — it is
   the same quantity in different units, not a distinct
   alternative.

2. **Option (a) for `total_steps` was dismissed too quickly.**
   The cold reader pushed back on my framing that option (a)
   required "rewriting the off-policy inner loop's counting"
   and suggested it was plausibly a ~5-line change. To test
   the claim I re-read `td3.rs:260–310` and discovered the
   issue was deeper than a counter change: the off-policy
   inner loop steps *all* `n_envs` envs unconditionally every
   iteration, regardless of whether any env has completed an
   episode. At the same Steps(N) budget, off-policy performs
   more actual `env.step()` calls per epoch than on-policy —
   roughly `n_envs × (max-over-envs of first-episode length)`
   vs on-policy's `sum of episode lengths before done`. This
   is not a reporting artifact; it is a real work disparity
   that both current counters honestly report.

   **Consequence:** chapter 21's "reporting divergence"
   framing understated the finding. The chapter 22 section
   needed a restructure. *Fix applied:*
   - Renamed the section from "Handling the total_steps
     reporting divergence" to "The work inside an epoch is
     not uniform."
   - Rewrote the body to name the actual cause: off-policy's
     inner loop calls `env.step()` on all envs every
     iteration, including post-completion envs that have
     been auto-reset.
   - Broke the recon's option (a) into two sub-options:
     (a) "fix the counter to stop at first completion" — a
     reporting-only change that hides the disparity, which
     the chapter rejects explicitly; and (a′) "change
     off-policy's inner loop to actually stop stepping
     completed envs" — a behavior change that is ~10–20
     lines of code but alters what TD3 and SAC *are*
     (they rely on collecting many transitions per epoch).
   - Reframed option (c) as moot under the new understanding,
     because both current counters are already uniformly
     computing "actual env.step() call count" and there is no
     metric that would produce the same number for both
     families at the same epoch count — the families do
     different amounts of work per epoch, and any "uniform
     metric" would either be (a) in disguise or (a′) in
     disguise.
   - Picked option (b) — document and report per-family
     actual step counts in the rematch writeup — as the
     rematch answer, and named option (a′) as the proper
     long-term direction contingent on an algorithm-design
     discussion that is out of scope for the study.
   - Added the risk paragraph: option (b) depends on writeup
     discipline; chapter 31 inherits the guard.

   This is the largest single change in the Ch 22 revision
   cycle. The chapter's overall architecture (rematch-now,
   surface-later) survives, but the `total_steps` section is
   substantively different from the first draft.

3. **Warmup decision didn't acknowledge effect-size
   dependency.** The cold reader pointed out that "0.2 percent
   is small" is only defensible conditional on the rematch's
   effect size being large relative to 0.2 percent — and that
   the chapter had handed the dependency to Ch 31 without
   naming it in its own text. *Fix:* added a paragraph to the
   warmup section making the conditional explicit: if the
   rematch's effect is ~3 percent, 0.2 percent is a fifteenth
   of the effect and "live with it" is fine; if the effect is
   ~0.5 percent, 0.2 percent is 40 percent of the effect and
   the decision is not defensible. Chapter 31's gate is what
   makes this conditional enforceable.

4. **"Non-contentious" closer undercut the chapter's own
   work.** The cold reader flagged the closing sentence
   describing the decisions as "non-contentious" and the
   warmup call as "a 0.2 percent rounding" — language that
   retroactively minimized two decisions the chapter spent
   pages justifying. *Fix:* rewrote the closer to say
   "implementation-level rather than API-level and do not
   need the same deliberation layer" without minimizing the
   substance.

5. **Missing fifth design call: rematch configuration
   uniformity.** The cold reader noted that the chapter
   audited `Steps` uniformity at the formula level but did
   not address whether the rematch should require identical
   `n_envs` and `max_episode_steps` across algorithms. If
   different algorithms used different parallelism choices,
   the budget formula would give different epoch counts and
   the uniformity finding would not apply. *Fix:* added this
   as the second design call — "all five algorithms run with
   identical `n_envs = 32` and `max_episode_steps = 5000`" —
   both in the budget-unit section and in the summary of
   design calls at the end.

6. **PPO `k_passes` didn't name the learning-efficiency vs
   algorithm-selection distinction.** The cold reader flagged
   that the chapter said wall-clock was out of scope without
   naming *what question* the rematch does answer. The
   distinction is: "which algorithm learns best per env step"
   (the rematch's question) vs "which algorithm should a
   practitioner choose for a real workload" (not the
   rematch's question). *Fix:* added one paragraph to the
   `k_passes` section making this distinction explicit and
   naming the silence on wall-clock as a property of the
   design, not an oversight.

7. **Soft-word cleanup.** Dropped "straightforward" (replaced
   with "direct"), "All three are real" (removed in the
   section rewrite), and the self-congratulatory "three
   reporting design calls with explicit two-part answers"
   framing in the closer.

**Lower-severity items noted but not acted on:**

- The cold reader suggested changing chapter 31 forward
  references to use "inherits" language rather than "will
  spec" language. Kept "will spec" because the chapter 31
  sections are predictable from chapter 22's text and the
  distinction the cold reader drew felt finer than the
  chapter benefits from.

## Second round

**Triggered:** No. The biggest revision (the `total_steps`
section rewrite) was substantive enough that a round-2 cold
read would be defensible, but the revision was driven by the
first thinking pass and a direct re-read of the source code,
not by a different argument the first pass missed. The
rewritten section stands on its own and the decisions it
makes are consistent with the rest of the chapter. A round-2
read would be duplicative.

## Open questions carried forward

- **Can off-policy actually be changed to stop stepping
  completed envs without hurting its learning?** This is
  option (a′) in the chapter's `total_steps` section, and
  the study rejects it as out of scope. If a future PR wants
  to implement it, an algorithm-design discussion has to
  happen first: off-policy methods are designed to collect
  many transitions per epoch, and gating that on completion
  is not a free reporting fix.
- **What is the expected effect size of the D2c rematch?**
  The warmup decision's defensibility depends on this. The
  study does not currently have an empirical estimate. A
  short prep experiment before the rematch runs would give
  one. Flagged for chapter 31.
- **Does Ch 31 need to pre-empt the "algorithm-selection"
  misreading of the rematch?** The PPO `k_passes` framing
  added in the thinking pass raises the possibility. Chapter
  22 flags it; chapter 31 decides whether to include it in
  the failure-mode enumeration.

## Status

Drafted, factual pass run with 9/9 claims verified, thinking
pass run with seven findings applied (one triggering a
structural rewrite of the `total_steps` section, the other six
targeted edits). No second round triggered. Ready for commit
(pending user permission).
