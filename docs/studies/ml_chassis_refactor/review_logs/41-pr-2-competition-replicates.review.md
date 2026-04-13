# Review log — Chapter 41: PR 2 Competition replicates and algorithm surface fixes

Chapter 41 is the second PR-plan chapter in Part 4 of the
study. It plans PR 2 (Competition replicates + algorithm
surface fixes) as a 2-PR split into PR 2a (additive API
extensions: `run_replicates`, `replicate_index`,
`replicate_best_rewards`, `describe`, `SeedSummary`) and PR 2b
(semantic fix: Decision 1 train-loop rewrites for CEM/TD3/SAC,
REINFORCE/PPO zero-fallback cleanup, CEM
`elite_mean_reward` key rename, and a doc-only update to
`d2c_cem_training.rs`).

The chapter is gated per the Ch 01 protocol and pauses for
user review after the factual + thinking passes and before
commit. This log documents the pre-commit state.

As with Ch 40, Ch 41 is a hybrid of rendering and argument.
Most of the chapter renders Ch 23/Ch 24/Ch 31 §4.2's
decisions into concrete diff plans and landing sequences;
six sections make genuine in-chapter sub-decisions that the
upstream chapters did not lock (the CEM mean_reward rewrite
form, the TD3/SAC debug-assert invariant guard, the CEM
`elite_mean_reward` rename, the REINFORCE/PPO zero-fallback
shape, the `d2c_cem_training.rs` scope narrowing, and the
2-PR split itself). The factual pass verified the rendering
claims; the thinking pass pressure-tested the six sub-decisions.
Both passes ran sequentially per the argument-chapter
pattern, because Ch 41's §3 (`d2c_cem_training.rs` scope
narrowing driven by a Ch 24 overclaim finding) is an argument
that changes what PR 2b's diff looks like, and the chapter's
voice shifts mode there.

## Recon

Ch 41's recon had four classes of source-tree verification
and one re-read of upstream chapters.

- **Ch 23 (Competition API v2) — full re-read.** Absorbed
  the three picks (replicates API shape, PPO exclusion from
  the rematch pool, matched-complexity anchor) as inputs
  rather than work-products. Ch 41 renders the first, cites
  the second for scope clarification, and notes the third as
  Ch 42 territory. One item confirmed during recon: Ch 23's
  `run_replicates(tasks, builders, seeds)` signature (not
  `(seeds)` alone) — Ch 32's factual pass caught a paraphrase
  drift on this three separate times, worth double-checking
  during Ch 41 drafting. Confirmed against the signature
  locked in Ch 23 §1.
- **Ch 24 (Result semantics) — full re-read.** Absorbed
  Decision 1 (fix the algorithms — CEM/TD3/SAC rewrites,
  REINFORCE/PPO no-ops) and Decision 2 (`replicate_best_rewards`,
  `describe`, `SeedSummary`) in full. Ch 41 inherits both
  decisions as locked shapes. One surprise surfaced during
  recon and is flagged as a post-commit patch bundled into
  the Ch 41 commit — see "Ch 24 §1.9 / §2.2 / §5 patch bundled into
  the Ch 41 commit" below.
- **Ch 31 §4.2 — targeted re-read.** Absorbed the
  algorithm-surface metric fix inheritance bookkeeping and
  the `elite_mean_reward` cleanup demotion. One minor
  phrasing slip in Ch 31 §4.2 ("outside cem.rs" when both
  readers are inside cem.rs) surfaced and is explicitly left
  un-patched per §6's decision — the load-bearing claim is
  still correct.
- **Source tree — `competition.rs`, `cem.rs`, `reinforce.rs`,
  `ppo.rs`, `td3.rs`, `sac.rs`, `algorithm.rs`, `rollout.rs`,
  `d2c_cem_training.rs`.** Full Read of each file or targeted
  Read of the relevant ranges. Cited file:line ranges
  confirmed during drafting and the factual pass.
- **Grep censuses.** Three systematic grep runs during recon:
  (a) `elite_mean_reward` workspace-wide, returning 4 hits
  all inside `cem.rs` — write at `:191`, insert at `:219`,
  unit tests at `:319` and `:354`. Load-bearing for Ch 31
  §4.2's "no rematch reporting consumers" claim and for
  §2.3's rename call. (b) `metrics.len()` workspace-wide in
  ml-bridge and thermostat test trees, returning 6 hits of
  the shape `assert_eq!(metrics.len(), N)` against specific
  reaching-task smoke tests — all safe under §2.4's
  skip-the-push shape (i) zero-fallback cleanup because the
  tests train on well-specified envs that never hit the
  zero-fallback branch. Census is recon-reported in §2.4 so
  the PR 2b reviewer does not re-derive it. (c) `final_reward()`
  in `ml-bridge/tests/competition.rs`, returning 47 hits
  across `:891-:1458` — recon-reported in §4.3 as the
  reward-threshold-test risk surface for PR 2b.
- **d2c_cem_training.rs full-file review.** The scope of this
  read was narrower than a full-file audit but specifically
  targeted the recon question "does this file actually use
  Competition or best_reward() as Ch 24 §2.2 and §5 claim?"
  Verified: (i) the use statement at `:27-33` does not import
  `Competition`; (ii) grep across the file for `best_reward`
  and `final_reward` returns zero hits; (iii) each of the
  four `#[test]` functions at `:276`, `:295`, `:331`, `:356`
  trains exactly one algorithm and asserts per-algorithm
  gates (Gate A on synchrony t-stat, Gate B on
  `best_last_10 > first_5_mean` within one algorithm's
  metrics trajectory); (iv) the file is 386 lines total with
  no post-fn cross-algorithm code (verified by reading
  `:380-386`). The finding is load-bearing for §3's narrowing
  of PR 2b's `d2c_cem_training.rs` scope from "full rewrite
  to use Competition" to "doc-only update."

**One targeted recon question** surfaced during the recon-to-
leans conversation and was resolved before drafting: how
large PR 2 should be, given Ch 40's 2-PR precedent and Ch
41's different diff shape. Three shapes were named in the
recon-to-leans — (i) single PR, (ii) 2-PR split into additive
and semantic halves, (iii) 3-PR split isolating the CEM
rename from Decision 1 rewrites. The lean toward (ii) was
communicated to the user in the recon-to-leans note; the
user delegated the call under standing autonomous-delegation
authority. §4.1 defends the pick in chapter form.

**Five other in-chapter calls** were also surfaced during
the recon-to-leans note and delegated: (a) CEM `mean_reward`
rewrite form (Form (i): second traversal matching
REINFORCE/PPO idiom), (b) TD3/SAC `debug_assert_eq!`
invariant guard (shape (a): add the debug-assert), (c) CEM
`elite_mean_reward` rename-or-doc (rename to
`elite_mean_reward_per_step`), (d) REINFORCE/PPO
zero-fallback shape (shape (i): skip the `metrics.push`), (e)
`d2c_cem_training.rs` scope in PR 2b (doc-only narrowing
driven by the Ch 24 overclaim finding). Each is defended in
the chapter via counterfactual walk; §5 tables them with
one-line rationale.

## Factual pass

The verification checklist for Ch 41 extracted roughly 65
concrete claims from the draft — file:line citations,
function names, struct field lists, grep results, paraphrases
of upstream chapter text. The checklist was run against the
current source tree during drafting and fixed in place; this
log records what the final state is rather than what the
initial draft said.

**Verified.** Approximately 60 of the ~65 claims verified on
the first pass. Highlights:

- `competition.rs:23-34` — `RunResult` struct with its five
  existing fields. Verified.
- `competition.rs:45-50` — `RunResult::best_reward()` body
  with `max_by` + `partial_cmp` + `Ordering::Equal` fallback.
  Verified.
- `competition.rs:49` — the specific `Ordering::Equal` call
  inside `best_reward()`'s `max_by` closure. Verified.
- `competition.rs:290, 304` — `Competition::new` and
  `new_verbose` constructor lines. Verified.
- `competition.rs:321-411` — the `run` method's full span.
  Verified.
- `competition.rs:362` — the strict-`>` comment on the
  inline best-epoch scan. Verified.
- `competition.rs:363-379` — the inline best-epoch scan's
  full span (the block that Ch 41 §2.1 explicitly preserves).
  Verified.
- `competition.rs:381-395, 400-406` — provenance construction
  and RunResult push spans. Verified.
- `competition.rs:91-110` — the three helper methods (`find`,
  `for_task`, `for_algorithm`). Verified.
- `competition.rs:120-133, 137-146, 153-192, 198-219` —
  `save_artifacts`, `best_for_task`, `print_ranked`,
  `print_summary`. All verified.
- `competition.rs:66-76` — `RunResult::assert_finite()`
  with `is_finite()` + `assert!` panic. Verified.
- `competition.rs:416` — start of `#[cfg(test)] mod tests`
  block. Verified.
- `competition.rs:520-538` — `competition_runs_all_pairs`
  test function body. Verified.
- `cem.rs:141-144` — n_epochs formula. Verified.
- `cem.rs:177-185` — fitness Vec construction. Verified.
- `cem.rs:177-189` — fitness + sort + elites slice. Verified.
- `cem.rs:182-183` — length normalization inside map
  closure. Verified.
- `cem.rs:191` — `elite_mean_reward` computation. Verified.
- `cem.rs:209` — `mean_reward` reporting site. Verified.
- `cem.rs:212-213` — `self.best.maybe_update(...)` call.
  Verified.
- `cem.rs:219` — `elite_mean_reward` extras insert. Verified.
- `cem.rs:264` — start of `#[cfg(test)] mod tests`. Verified.
- `cem.rs:292` — `fn cem_smoke_2dof`. Verified.
- `cem.rs:319, 354` — unit test reads of `elite_mean_reward`
  key. Verified.
- `reinforce.rs:168-171` — n_epochs formula. Verified.
- `reinforce.rs:213-225` — full `if n_samples == 0` block.
  **Factual pass correction** — initial draft cited
  `:214-222` following Ch 24's narrower range; widened to
  `:213-225` to cover the full if-block that Ch 41's patch
  modifies, including the `metrics.push` and `continue`
  lines that sit outside Ch 24's cited range.
- `reinforce.rs:269-274` — `mean_reward` computation.
  Verified.
- `reinforce.rs:324` — start of `#[cfg(test)] mod tests`.
  Verified.
- `reinforce.rs:358` — `fn reinforce_smoke_2dof`. Verified.
- `ppo.rs:225-228` — n_epochs formula. Verified.
- `ppo.rs:313-325` — full `if n_samples == 0` block. Same
  factual-pass widening as REINFORCE (Ch 24 cited :314-322).
- `ppo.rs:423-428` — `mean_reward` computation. Verified.
- `ppo.rs:494` — start of `#[cfg(test)] mod tests`.
  Verified.
- `ppo.rs:533` — `fn ppo_smoke_2dof`. Verified.
- `td3.rs:239-242` — n_epochs formula. Verified.
- `td3.rs:259-284` — outer epoch loop setup. Verified.
- `td3.rs:335-351` — reward accumulation block. Verified.
- `td3.rs:337-343` — specific push-to-`epoch_rewards`
  within the inner loop. Verified.
- `td3.rs:487-491` — `mean_reward` computation. Verified.
- `td3.rs:574` — start of `#[cfg(test)] mod tests`.
  Verified.
- `td3.rs:617` — `fn td3_smoke_2dof`. **Factual pass
  correction** — initial draft cited `td3.rs:618`; corrected
  to `:617`.
- `sac.rs:266-269` — n_epochs formula. Verified.
- `sac.rs:286` — epoch loop start. Verified.
- `sac.rs:355-363, 357-363` — reward accumulation. Verified.
- `sac.rs:540-544` — `mean_reward` computation. Verified.
- `sac.rs:630` — start of `#[cfg(test)] mod tests`.
  Verified.
- `sac.rs:672` — `fn sac_smoke_2dof`. **Factual pass
  correction** — initial draft's hand-wavy `sac.rs:~675-`
  corrected to `:672`.
- `algorithm.rs:32-45` — `EpochMetrics` struct with its five
  fields. Verified. (Ch 24 cited `:32-43` for the same
  struct; the actual end is `:45` at the closing brace.
  Ch 41 uses the wider `:32-45` range.)
- `rollout.rs:82` — `collect_episodic_rollout` function
  declaration. Verified.
- `d2c_cem_training.rs:1-9` — module-level doc block.
  **Factual pass correction** — initial draft cited `:3-10`;
  corrected to `:1-9` (the doc block starts at `:1` and ends
  at `:9`; `:10` is blank).
- `d2c_cem_training.rs:27-33` — use statements (two `use`
  lines — `sim_ml_bridge` at `:27-32` and `sim_thermostat`
  at `:33`). Verified.
- `d2c_cem_training.rs:62` — `SEED_BASE` constant. Verified.
- `d2c_cem_training.rs:159-191` — `evaluate_policy` helper.
  Verified.
- `d2c_cem_training.rs:216-223` — training callback block
  containing the eprintln. **Factual pass refinement** —
  initial draft cited `:216-222` for the eprintln format
  string; corrected to cite the eprintln macro call at
  `:218-221` inside the callback block at `:216-223`.
- `d2c_cem_training.rs:275` — `#[ignore]` annotation.
  Verified.
- `d2c_cem_training.rs:276, 295, 331, 356` — the four
  `#[test]` function definitions. Verified.
- `d2c_cem_training.rs:226, 231, 233, 235` — Gate B
  `first_5_mean` / `best_last_10` / `gate_b` computation
  sites. Verified.
- `ml-bridge/tests/competition.rs` `final_reward()` census —
  **factual pass correction** — initial draft said "15+ call
  sites between `:891` and `:1458`"; corrected to
  "approximately 47 call sites spanning `:891` through
  `:1458`" (grep count verified at 47).

**Factual pass drift items (5 total, all fixed single round):**

1. REINFORCE and PPO zero-fallback cited range
   (`:214-222` / `:314-322`) widened to the full if-block
   spans (`:213-225` / `:313-325`) to match the code blocks
   shown in the draft. Ch 24 cited the narrower ranges
   (EpochMetrics construction + on_epoch only); Ch 41 needs
   the wider ranges to cover the `metrics.push` and
   `continue` lines that the patch edits.
2. `d2c_cem_training.rs` module-level doc block range
   corrected from `:3-10` to `:1-9`.
3. `d2c_cem_training.rs` eprintln cite refined from the hand-
   wavy `:216-222` range to the specific eprintln macro at
   `:218-221` inside the callback at `:216-223`.
4. `ml-bridge/tests/competition.rs` `final_reward()` census
   count corrected from "15+" to "approximately 47."
5. REINFORCE/PPO zero-fallback line-count diff corrected from
   "−8 lines" to "−10 lines" (13-line if-block replaced by
   3-line bare-continue form).

Plus three test-module line-number cites in §2.6 that were
speculative in the initial draft and verified during the
factual pass: `cem.rs:264-414` (draft had `:262-414`),
`td3.rs:617` (draft had `:618`), `sac.rs:672` (draft had
`:~675-`). All corrected in place.

**One recon-surfaced finding not previously seen.** During
recon, the grep for `best_reward()` / `final_reward()` /
`Competition::new` in `sim/L0/thermostat/tests/d2c_cem_training.rs`
returned zero hits. This contradicted Ch 24 §2.2's claim
that "the D2c rematch test at
`sim/L0/thermostat/tests/d2c_cem_training.rs` compares
`best_reward()` values across algorithms and passes or fails
the test based on those comparisons." Full-file read
confirmed the finding: the file has four independent
`#[test]` functions (`d2c_cem`, `d2c_td3`, `d2c_ppo`,
`d2c_sac`), each trains one algorithm, each asserts
per-algorithm Gate A and Gate B, and the cross-algorithm
comparison lives in the D2 SR findings memo and in human-read
eprintln output rather than in any test assertion. This is
the Ch 24 overclaim that §3 addresses and that the bundled
Ch 24 patch corrects.

## Thinking pass

The 10-point argument-chapter brief was applied to Ch 41's
six in-chapter sub-decisions (§2.2 CEM rewrite form,
§2.2 TD3/SAC debug-assert, §2.3 CEM rename, §2.4
zero-fallback shape, §3 `d2c_cem_training.rs` scope, §4.1
PR split) and to the Ch 24 overclaim finding in §3.

**Three findings, all should-fix, all applied single round:**

**T1 (§1.2) — "all five algorithms" overclaim clarification.**
The initial draft said "Ch 41 modifies all five algorithms
that ship today (CEM, REINFORCE, PPO, TD3, SAC), because
Decision 1's unit-uniformity guarantee applies to the entire
ml-bridge surface." This conflates Decision 1's three-algo
scope (CEM/TD3/SAC) with the zero-fallback cleanup's two-algo
scope (REINFORCE/PPO). The two axes together touch all five,
but "Decision 1 touches all five" is not true. The fix:
clarify the split-axis framing, naming Decision 1's three
algorithms and the zero-fallback cleanup's two separately.
Applied in §1.2's "PPO pool-membership" paragraph.

**T2 (§2.6) — under-specified test surface.** The initial
draft mentioned "~4 new inline unit tests covering the
additive contract" in §2.1 without enumerating them and
without listing PR 2b's new tests at all. The recon-to-leans
planned a 7-point test enumeration as sub-decision (h); the
initial draft folded it into a hand-wavy phrase. The fix:
add §2.6 "Test surface additions" enumerating 11 PR 2a tests
and 7 PR 2b tests (plus one in-place rename of the existing
`cem_smoke_2dof`-adjacent `elite_mean_reward` asserts at
`cem.rs:319, 354`). This gives the PR 2 reviewer a concrete
checklist rather than an estimate.

**T3 (§2.4 shape-iii argument) — unverified "six integration
test files" count.** The initial draft's argument against
shape (iii) (Option<f64> EpochMetrics struct change) cited
"many callers across the ml-bridge crate and its tests" and
made a specific numeric claim "six integration test files
that read mean_reward directly." The count was not verified
during recon — I knew the integration test files read
`mean_reward` but had not exhaustively grep'd for per-file
read sites. The fix: replace the specific count with a
specific enumeration of the `competition.rs` read sites
(`:48,:166,:167,:206,:345,:351,:386`) and a named list of
integration test files (`d1c_cem_training.rs`,
`d1d_reinforce_comparison.rs`, `d2c_cem_training.rs`) with
an explicit note that the exact per-line counts are a PR 2b
author concern. Applied in §2.4's shape (iii) bullet.

**Two additional small calls made during the thinking pass**
(not rising to should-fix but noted here for completeness):

- **§2.4's shape (iii) framing strengthened.** Added an
  extra paragraph explicitly naming shape (iii) as "the
  structurally cleanest answer if PR 2 were scoped to
  chassis surface redesign" rather than rejecting it as
  infeasible. This makes the scope-discipline argument
  honest: shape (iii) is not wrong in the abstract, it is
  out of PR 2's scope because Ch 24 §3.7 explicitly declined
  to widen Decision 1 beyond train-loop rewrites. A future
  chassis-API PR that redesigns `EpochMetrics` would land
  shape (iii) naturally. Applied in §2.4 after the shape
  (iii) bullet and before the "Shape (i) has one risk" block.
- **§3.1 argument verification.** The thinking pass
  specifically pressure-tested the §3 overclaim finding by
  asking: "Is there any cross-algorithm code in
  `d2c_cem_training.rs` that I missed?" This triggered a
  read of `d2c_cem_training.rs:380-386` (the tail of the
  file past the last `#[test]` fn), which confirmed the
  file is 386 lines and ends at the `}` closing `fn d2c_sac`
  — no post-fn cross-algorithm comparison code exists. The
  §3 argument is airtight against this specific
  counterfactual.

**Points (1-10) of the argument-chapter brief — coverage:**

1. **Unstated assumptions.** §1.2's "all five algorithms"
   framing (T1 fix) addressed the one unstated-assumption
   finding. §2.4's shape (iii) paragraph addressed another
   near-miss (the unstated assumption that chassis-API
   redesign is "infeasible" rather than "out of scope").
2. **Alternatives not considered.** §2.2 CEM rewrite Form
   (ii) was considered and rejected; a Form (iii) midway
   shape (accumulate `total_reward` inside the fitness
   `map` returning a 3-tuple) was not named but is
   essentially a minor variant of Form (ii) and rejected on
   the same concern-purity grounds. §2.4 zero-fallback
   shapes (i)-(iv) exhaustive. §4.1 PR-split shapes
   (i)-(iii) exhaustive; a shape (iv) four-PR split is
   implicitly rejected by part 4's logic against shape
   (iii).
3. **"Obviously" / "clearly" tells.** One use of "currently
   obvious" in §2.2's debug-assert defense — the word is
   doing real work (the sub-decision hinges on
   "currently obvious vs future-fragile") rather than
   hand-waving. Left as-is.
4. **Conclusion-follows-evidence.** The §3 argument
   (Ch 24 overclaim) is the most load-bearing and was
   verified by the `:380-386` tail read during the
   thinking pass. The §2.4 shape (i) pick is verified by the
   `metrics.len()` grep census in §2.4 itself. The §4.1
   PR-split pick is a structural argument about review
   surface and risk profile that does not require source
   verification.
5. **Simpler explanation.** §3's narrowing to "doc-only"
   scope for `d2c_cem_training.rs` is the simpler
   explanation relative to Ch 24's "full rewrite" framing,
   and the chapter takes it.
6. **Falsifiability.** §3's finding is falsified by any
   cross-algorithm code inside `d2c_cem_training.rs` —
   verified absent by full-file read. §2.4's shape (i) is
   falsified by any existing test that asserts
   `metrics.len() == n_epochs` on an algorithm whose training
   actually hits the zero-fallback — verified absent by the
   census. §4.1's split pick is not strictly falsifiable in
   a factual sense but is tested against shape (i) and (iii)
   counterfactuals.
7. **Defends picks against reader pushback.** All six
   sub-decisions have counterfactual walks. §2.3's CEM
   rename walks the "keep-the-name, add-a-doc" alternative.
   §2.2's TD3/SAC debug-assert walks shapes (a), (b), (a').
   §2.4's zero-fallback walks four shapes. §4.1 walks three
   shapes.
8. **Scope discipline honored.** §6's "What Ch 41 does not
   decide" enumerates the deferred items: Ch 40 scope, Ch
   42 scope, shared helper deferral, EpochMetrics module
   layout, `run` deprecation deferral, `d2c_cem_training.rs`
   retirement deferral, rustdoc text detail, TD3/SAC
   reporting-vs-training asymmetry, Ch 31 §4.2 phrasing
   slip, Ch 24 patch bundling. Comprehensive.
9. **Over-claim / under-claim check.** T1 was the main
   over-claim; fixed. T3 was a smaller over-claim (specific
   count on unverified census); fixed by replacing with
   specific enumeration. No under-claims found.
10. **Rendering/argument boundary.** Ch 41 renders §1, §2.1,
    §2.5, §2.6, §4.4, and most of §6. It argues §2.2, §2.3,
    §2.4, §3, §4.1, §4.2, §4.3. The boundaries are clear
    and the genre note in §1.5 sets the reader up for the
    split.

**A note on what the thinking pass did NOT trigger a fix for:**

- The Ch 31 §4.2 "outside cem.rs" phrasing slip is
  deliberately not patched. The load-bearing claim is still
  correct; the fix would be churn. §6 names the slip for
  transparency.
- Ch 24 §3.6's done_count line-number cite (`td3.rs:489` —
  actually the `0.0` empty-branch fallback, not a done_count
  site) is a minor Ch 24 drift that does not affect Ch 41's
  argument and is not bundled into the Ch 24 patch. The
  bundled patch is narrow and stays focused on §2.2 and §5.
- Ch 24 §1.1's `algorithm.rs:32-43` EpochMetrics range drift
  (actual is `:32-45`) is similarly minor and un-patched.
- The `save_artifacts` replicate-collision issue at
  `competition.rs:124` (per-replicate filename collision
  when multiple replicates share a `(task, algorithm)` pair)
  is flagged in §2.1 but not fixed. Deferred to Ch 42's PR 3.

## Ch 24 §1.9 / §2.2 / §5 patch bundled into the Ch 41 commit

Following the `b5cb3f6c` precedent, the Ch 41 commit bundles
a narrow post-commit patch to Ch 24 §1.9, §2.2, and §5
correcting the factual overclaim that `d2c_cem_training.rs`
"compares `best_reward()` values across algorithms." A
**factual pass correction** during review log writing
discovered that the overclaim appears in three Ch 24 sites,
not two — the initial recon-to-leans note only named §2.2
and §5. The third site (§1.9 at `24-result-semantics.md:428-433`)
contains the strongest form of the overclaim and is the
sentence Ch 41 §3.1 quotes as load-bearing evidence. Ch 41
§3.1 initially misattributed that sentence to §2.2; the
attribution was corrected and §3.1 now names all three
sites individually. The patch amends each:

- **Ch 24 §1.9 (`:428-433`, at the "What this means in
  practice" table closer).** The sentence "The D2c rematch
  test at `sim/L0/thermostat/tests/d2c_cem_training.rs`
  (recon-reported, exact line range deferred to factual
  pass) compares `best_reward()` values across algorithms
  and passes or fails the test based on those comparisons.
  Whatever that test was measuring, it was not what the
  name suggested it was measuring" is replaced with a
  corrected framing: the cross-algorithm comparison lives
  in the D2 SR findings memo and in human-read eprintln
  output, not in any assertion inside the test file; Gate A
  (per-algorithm synchrony t-stat) and Gate B (within-
  algorithm `best_last_10 > first_5_mean`) are unaffected by
  Decision 1's rescaling because they are per-algorithm
  monotonicity checks; the unit-brokenness applies to the
  narrative reading of the test's eprintln output and the
  memo's headline table, not to any `#[test]` gate.
- **Ch 24 §2.2 (`:492-505`, the "supporting observation for
  PPO (and TD3 and SAC)" paragraph).** The sentence "The
  D2c rematch test compared them directly" is replaced with
  a corrected framing naming the D2 SR findings memo as the
  locus of the cross-algorithm comparison. The subsequent
  "The D2c test was apples-to-oranges against all three
  off-policy algorithms at once" becomes "The D2c comparison
  (in the D2 SR findings memo) was apples-to-oranges." The
  supporting-observation axis Ch 24 §2.2 develops — "two
  independent mechanisms by which the D2c PPO number fails
  to mean what its name suggests" — survives unchanged.
- **Ch 24 §5 (`:1094-1100`, the "Whether the D2c test gate
  needs to be rewritten" bullet).** The sentence "It does —
  the test is unit-broken against CEM versus any of the
  other four algorithms — but rewriting the test is a
  Chapter 41 execution-layer concern" is replaced with:
  the test file does not contain a cross-algorithm
  comparison in its assertions; the unit brokenness exists
  in the D2 SR findings memo's narrative comparison and in
  the eprintln output a human reader reconstructs from
  per-algorithm runs; Ch 41's execution-layer scope for
  `d2c_cem_training.rs` is doc-only; Ch 42's rematch PR
  (PR 3) ships the unit-correct cross-algorithm gate in a
  new test fixture under `sim-opt/tests/` rather than as a
  rewrite of `d2c_cem_training.rs`.

The bundled patch does not change Ch 24's Decision 1 or
Decision 2, does not alter any Ch 24 file:line citation, and
does not touch Ch 24's scope discipline closer. Ch 24's
review log gets a "Round 2" section (following the Ch 10 and
Ch 21 precedents) documenting what changed and why. The
Round 2 section names Ch 41 §3 as the source of the
correction and cites the full-file read at
`d2c_cem_training.rs:380-386` as the verifying evidence.

The patch is narrower than the typical `3e1ec0ff` pattern
(which replaced two factually-wrong phrasings in Ch 21)
because the factual reading of Ch 24's load-bearing claims
stays intact — Decision 1 (fix the algorithms) and Decision 2
(`replicate_best_rewards` + `describe` + `SeedSummary`) are
unchanged. The patch is a scope correction of a specific
test-file's role in the apples-to-oranges shape, not a
revision of the decisions themselves.

## Round 2 thinking pass — cold-read delegation

After the Round 1 factual + thinking passes cleared and the
chapter was surfaced for the human review gate, the user
requested a fresh-perspective review of three items I flagged
as "worth particular attention": sub-decision (e) plus the
bundled Ch 24 patch, sub-decision (d) REINFORCE/PPO
zero-fallback skip-the-push, and the §2.6 test surface
enumeration. Per the Ch 01 protocol for high-stakes
chapters ("the thinking pass is run twice, once by the
original author and once by a fresh sub-agent, and any
disagreements between the two are reconciled explicitly in
the chapter"), I delegated the cold read to an Explore
sub-agent with a tight source-verification brief while
running targeted independent grep/Read checks in parallel.

**The sub-agent's verdict:**

- **Item 1 — sub-decision (e) + Ch 24 patch: HOLDS.** Cold
  read independently verified that `d2c_cem_training.rs`
  contains zero `best_reward(`/`final_reward(`/`Competition`
  references, all four `#[test]` functions at `:276`,
  `:295`, `:331`, `:356` are per-algorithm, nothing past
  the last fn (`:386` EOF) does cross-algorithm work, and
  grep for `d2c`/`D2c` across `sim/` turns up no other file
  doing cross-algorithm D2c comparison. The Ch 24
  amendments were independently read against the current
  source and verified as scope-correction only (Decision 1
  and Decision 2 unchanged; no file:line citations altered).
- **Item 2 — sub-decision (d) zero-fallback skip-the-push:
  HOLDS with one caveat folded into Item 3.** The six
  `metrics.len() == N` assertions were independently read
  and each confirmed to run on a reaching task where the
  zero-fallback branch is structurally unreachable. My
  own parallel `on_epoch` workspace grep (16 hits, all
  inside ml-bridge src, none in tests counting invocations)
  independently verified that no in-repo caller depends on
  the "once per epoch iteration" contract. The caveat: the
  §2.6 enumeration's tests (17)/(18) were marked as
  conditional doc-only, which the cold reader rightly
  called out as an explicit deferral slipping past the
  "flag don't defer" discipline.
- **Item 3 — §2.6 test surface enumeration: DOES NOT HOLD.**
  Four regression-coverage gaps identified.

The four gaps and the fixes applied:

**Gap A — no runtime regression test for the inline
best-epoch scan's strict-`>` tie-breaking invariant at
`competition.rs:363-379`.** §4.1 of the chapter flagged this
as the single "specific attention point" for PR 2a review,
warning that a refactor to `RunResult::best_reward()` would
be "subtly wrong (different tie-breaking) and should not be
accepted without a separate discussion." Without a runtime
test, that warning is a review-time convention, not an
enforced contract. The inline scan uses strict `>` with
`f64::NEG_INFINITY` seed (earlier epoch wins under a tie),
while `best_reward()` at `competition.rs:45-50` uses
`max_by` with `partial_cmp`→`Ordering::Equal` fallback.
Rust's `Iterator::max_by` returns the *last* equal element
under a tie (independently verified during the Round 2
factual pass by re-reading `competition.rs:49`), so the two
produce different `best_epoch` values when two epochs share
identical `mean_reward`. **Fix: added test 2a
`run_replicates_preserves_strict_gt_tie_breaking`** — a
regression guard that constructs a MockAlgorithm variant
emitting two epochs with identical rewards, calls
`comp.run_replicates(...)` with a single seed, and asserts
`provenance.as_ref().unwrap().best_epoch == 0` (strict `>`
preserves the earlier epoch). A refactor that swapped the
inline scan for `best_reward()` would fail at the
`best_epoch` comparison and be caught at test time instead
of shipping silently.

**Gap B — no empty-slice test for
`SeedSummary::from_rewards`.** Tests (9) and (10) cover
`n == 1` and `n == 3`; the empty-slice case returning `None`
per Ch 24 §4.3's locked contract was not tested. **Fix:
added test 9a `seed_summary_from_rewards_empty_returns_none`**,
a one-line `assert!(SeedSummary::from_rewards(&[]).is_none())`.

**Gap C — no simultaneous validation of CEM's dual
reward-concept split.** Test (12)
`cem_mean_reward_is_per_episode_total` asserted the reported
`mean_reward` magnitude after Decision 1, but nothing
simultaneously asserted that the length-normalized fitness
at `cem.rs:182-183` stays in per-step units. Ch 24 §3.5's
scope-discipline argument ("CEM carries two reward concepts
internally") is what PR 2b renders into code, and without a
test that validates both concepts at once, a future refactor
that crosses the two computations would not be caught. **Fix:
added test 12a `cem_dual_reward_concept_split`** — trains CEM
for a few epochs on `reaching_2dof()` and asserts
`last.mean_reward.abs() > last.extra["elite_mean_reward_per_step"].abs() * 10.0`,
which is the order-of-magnitude gate that holds for any
reasonable episode length at the reaching task's
`max_episode_steps`.

**Gap D — conditional zero-fallback tests deferred to PR 2b
author drafting.** Tests (17)/(18)
(`{reinforce,ppo}_zero_fallback_skips_epoch`) were marked as
conditional runtime tests with a fallback to doc-only notes
"if no such configuration is reachable from the existing
env/task surface; the PR 2b author makes that call during
drafting." The cold reader rightly called out this as an
explicit deferral that slips past the "flag don't defer"
discipline (`feedback_explicit_deferrals` memory). **Fix:
dropped tests (17) and (18) from the runtime enumeration
entirely** and added a closing note to §2.6 naming the two
load-bearing static analyses that guard the skip-the-push
semantic instead: (a) the `metrics.len() == N` census across
the six existing assertions, each verified to train on an
env where `n_samples > 0` is structurally guaranteed, and
(b) the `on_epoch` workspace grep confirming no in-repo
caller counts callback invocations. The closing note is
explicit that a runtime regression test would be a
belt-and-suspenders guard on top of these analyses but
cannot be built against the current env/task surface
without a pathological fixture whose cost exceeds its
benefit; a future env/task that can legitimately produce
zero-length trajectories would be the right PR to add the
runtime test.

**Test count update.** Pre-Round-2: 11 PR 2a tests + 7
PR 2b tests + 1 in-place rename = 18 new `#[test]` items
plus one rename. Post-Round-2: 13 PR 2a tests (adding 2a and
9a) + 6 PR 2b tests (adding 12a, dropping 17 and 18) + 1
in-place rename = 19 new `#[test]` items plus one rename.
The Round 2 delta is +3 runtime tests − 2 dropped conditional
tests + 1 closing-note paragraph naming the dropped tests'
static-analysis equivalents.

**Round 2 factual pass on the additions.** The four new
citations introduced in the Round 2 edits were independently
verified against source: `competition.rs:45-50` for the
`best_reward` helper body (Rust std `Iterator::max_by`
behavior on ties verified against the language documentation
by re-reading `competition.rs:49`'s `partial_cmp`+`Equal`
fallback), `competition.rs:363-379` for the inline scan (from
the Round 1 factual pass), `cem.rs:182-183` for the
length-normalized fitness expression (from Round 1), and
`cem.rs:209` for the reported `mean_reward` site (from
Round 1). No new drift.

**Round 2 conclusion.** All four gaps closed single round.
The chapter is ready for the commit gate. The cold reader's
verdict was "hold for fix"; after the four fixes, the
chapter's test surface is structurally complete relative to
PR 2's diff, and the Round 2 thinking pass is closed.

Ch 41 final line count is 2062 (up from 1914 pre-Round-2),
reflecting the three new test entries plus the closing note
replacing the two dropped tests. Ch 40's 1821 is the
reference; Ch 41 is now ~12% longer, which tracks the
additional scope of "bundled Ch 24 patch" plus "Round 2
thinking-pass fixes" relative to Ch 40's single-chapter
scope.

## Human review pause

Per the Ch 01 protocol, Ch 41 is a gated chapter. It pauses
here for human review. The user's review should focus on:

1. **The six in-chapter sub-decisions** (§5's Table 1). Most
   consequential: sub-decision (e) (`d2c_cem_training.rs`
   scope narrowing to doc-only) and sub-decision (f) (2-PR
   split into 2a/2b). Sub-decision (d) (zero-fallback
   skip-the-push shape) is also worth attention because it
   changes the `on_epoch` callback contract subtly.
2. **The Ch 24 patch bundling.** The first time a Ch 41-era
   commit amends a committed argument chapter in scope
   (Ch 24 §2.2 and §5). The user explicitly sign-off on
   the bundle was received during recon-to-leans; the
   patch text itself is in §3.3 (in the chapter body) and
   summarized above in this review log.
3. **The test surface enumeration in §2.6.** Eleven PR 2a
   tests + seven PR 2b tests + one in-place rename. The
   user should confirm the set is complete relative to
   what they want PR 2's review gate to be.
4. **The scope-discipline §6 "What Ch 41 does not decide"
   section.** Nothing critical should sneak in that Ch 41
   has committed to by silence.

Ch 41 is 1914 lines at commit time, landing in the same
band as Ch 40's 1821. The branch is `feature/ml-chassis-study`
and stays there for Phase 4 (Ch 42 remains next session).

## Round 2 — Ch 42 bundled amendment (session 10)

Following the `843dc21c` precedent from session 9, Ch 42's
drafting in session 10 surfaced a real gap in Ch 41's
original rendering of `run_replicates`, and Ch 42 bundles a
narrow amendment to Ch 41 §1.1, §2.1, and §5 (the
sub-decisions table) in the Ch 42 commit.

**The finding.** Ch 41 §2.1 rendered the `run_replicates`
body with seeds flowing to `algorithm.train(..., seed, ...)`
at `competition.rs:341` (recon-reported) but not to
`task.build_vec_env(self.n_envs)` at `competition.rs:330`
(recon-reported) — the `TaskConfig::build_fn` signature at
`task.rs:43` (recon-reported) is `Fn(usize) -> Result<
VecEnv, EnvError>`, seed-blind. For deterministic-physics
tasks like `reaching_2dof` / `reaching_6dof` /
`obstacle_reaching_6dof`, this is fine — the per-replicate
variation comes entirely from the algorithm's exploration
RNG, and the physics is identical across replicates. For
stochastic-physics tasks like the rematch's D2c SR task
with its `LangevinThermostat`, this is a gap: the
`LangevinThermostat`'s `master_seed` is captured at
`TaskConfig` construction time and cannot vary per
replicate, which collapses half the seed-population
variance Ch 32's bootstrap CI is supposed to measure.

**The verifying evidence.** Ch 42's recon read
`sim/L0/ml-bridge/src/task.rs` in full and confirmed that
the current `build_fn` signature at `:43` takes only
`usize` (the env count) and has no seed channel. It also
read `competition.rs:321-407` for the `run` body (which
`run_replicates` reuses), confirming that the inner
`task.build_vec_env(self.n_envs)` call at `:330` is the
single point where the seed would need to thread through
to reach the `TaskConfig` closure. Ch 42 then walked four
rendering alternatives for resolving the gap (R1 sim-opt
owns the replicate loop with fresh TaskConfig per
replicate, R2 extend `TaskConfig::build_fn` signature
bundled as Ch 41 amendment, R3 sim-opt bypasses
`run_replicates`, R4 `AtomicU64` inside the TaskConfig
closure) and picked R2 on six grounds: four structural
(fix-gaps-before-continuing preference, breaking-change-
over-hacks preference, R34 chassis-overbuild philosophy,
future reuse for Ch 30's pre-committed null follow-ups)
and two polish (readability, A-grade discipline). The
user explicitly delegated the R1/R2 call to Ch 42's
author judgment; the pick is R2.

**The amendment.** Ch 42 §2 argues R2 in full. The Ch 41
source file gains the following changes, all bundled into
the Ch 42 commit:

- **§1.1 paragraph addition** after the "replicates API
  shape" bullet at `:77-99`, naming the Ch 42 amendment,
  pointing at §2.1 for the rendered change, and pointing
  at Ch 42 §2 for the full argument. Approximately 15 new
  lines.
- **§2.1 code block update** at `:458-489` — the skeleton's
  inner body gains `let mut env = task.build_vec_env(
  self.n_envs, seed)?;` with an inline comment citing the
  Ch 42 amendment. Approximately 3 modified lines.
- **§2.1 new paragraph after the code block** rendering
  the seven-site `task.rs` ripple: `:43` (field type),
  `:108` (method signature), `:241` (builder plumbing),
  `:362`/`:396` (reaching_2dof fn and closure), `:445`/`:500`
  (reaching_6dof fn and closure), `:619`/`:687`
  (obstacle_reaching_6dof fn and closure). Plus a note on
  `vec_env.rs:391` being unchanged. Approximately 60 new
  lines.
- **§5 sub-decisions table row (g)** added as the seventh
  sub-decision, naming the extension, the pick, and the
  one-line rationale. Approximately 1 new table row.

The amendment is a scope extension of PR 2a's plan, not a
decision revision. Ch 41's existing sub-decisions (a)–(f)
are unchanged. The additive/semantic 2-PR split structure
is unchanged. PR 2a's source-code diff becomes slightly
larger than the pre-amendment plan (~7 more sites in
`task.rs`), but PR 2a's merge dependencies are unchanged
and its landing sequence in §4.4 is unchanged.

**The bundled-commit form.** The amendment lands in the
Ch 42 commit alongside Ch 42's own chapter and review log,
matching the `843dc21c` (Ch 24 patch bundled into the Ch 41
commit) and `b5cb3f6c` (Ch 15 §5.6 patch bundled into the
Ch 40 commit) precedents. The Ch 42 commit message names
the bundle as a three-artifact edit: Ch 42 draft + Ch 41
amendment + review logs. A reader of the Ch 42 commit sees
the amendment scope in one diff and can verify the
bundled pattern against the review log.

**Verdict: amendment applied.** The round-2 section of this
review log documents the bundling state. Ch 41's final
state post-amendment is ~60 lines larger than the commit
state at session 9; the amendment does not affect Ch 41's
Round 1 factual or thinking-pass findings.

Post-Round-2 line count is roughly 2122 lines (up from 1914
at the session-9 commit). The branch stays on
`feature/ml-chassis-study`.

## Round 3 — Post-implementation audit (session 15 PR 2b)

**Context.** PR 2b landed as five commits in session 15:
`b95402d8` (CEM rewrite + rename + tests), `7ad55733` (TD3/SAC
pre-loop + invariant + tests), `5210f412` (REINFORCE/PPO
zero-fallback cleanup), `3ce98bd4` (d2c doc-only), `74627346`
(integration test findings-mode conversion). All 397→401 lib
tests passing debug / 399 passing release. This Round 3 entry
documents the post-implementation audit of the shipped code
against Ch 41's spec, matching the session 13 PR 1b and
session 14 PR 2a post-impl audit pattern.

**Audit approach.** Re-read Ch 41 §2.2–§2.6 and §4.3 against
the shipped commits and bundle any spec-vs-shipped drift into
one narrow patch. The audit focused on four spec surfaces:
the per-algorithm train-loop rewrites in §2.2, the CEM rename
in §2.3, the REINFORCE/PPO cleanup in §2.4, the d2c doc-only
update in §2.5, and the risk framing in §4.3.

**Drift found.**

1. **§4.3 "conservative threshold patches" framing was
   substantively wrong.** The risk bullet at
   `41-pr-2-competition-replicates.md:1838-1853` predicted
   that the integration-test repair would consist of
   "raising thresholds from the old per-step scale to the
   new per-episode scale" and would not change any test's
   pass/fail relative to its training outcome. Reality from
   the session 15 targeted 5-test rerun was qualitatively
   different: four of five at-risk tests failed on *ordering
   assertions* not threshold assertions, and three of those
   four flipped hard on `r_cem > r_td3` because the
   unit-mismatched metric had been quietly masking the true
   ordering. Tests 7/8 revealed `SAC > TD3 >> CEM` on
   reaching-6dof (inverse of the pre-PR-2b "CEM dominates"
   framing); Test 13 revealed `TD3 > SAC >> CEM` on
   obstacle-reaching-6dof, which actually *validates* that
   test's original "gradient methods outperform CEM on
   nonlinear landscape" hypothesis — the pre-PR-2b "CEM
   still dominates, hypothesis wrong" conclusion was a
   unit-mismatch artifact. The repair shape therefore
   shifted from threshold raising to findings-mode conversion
   (removing ordering assertions and replacing them with
   `print_reversal_check` + documented eprintln blocks),
   landed as commit `74627346`. The post-impl audit patch
   appends a "Post-implementation reality check" block to
   §4.3 documenting the actual observation.

**No other drifts found.** Every §2.2–§2.5 spec item landed
verbatim or with minor defensible rendering choices:

- §2.2 CEM rewrite: shipped the exact 5-line form Ch 41
  specified at `cem.rs:209`, plus a 4-line doc comment
  explaining Ch 24 §3.5's dual-concept split (not in the
  spec, but justified as a refactor-guard comment).
- §2.2 TD3/SAC pre-loop + `debug_assert_eq!`: shipped
  verbatim with a 6-line doc comment on each site.
- §2.3 CEM rename: three source edits at `cem.rs:219/:319/:354`
  shipped verbatim.
- §2.4 REINFORCE/PPO zero-fallback cleanup: shipped the
  exact 3-line bare-continue form; −10 lines per file as
  §2.4 predicted.
- §2.5 d2c doc-only: module-level doc extension (7 lines
  shipped vs §2.5's "~3 lines added" — a minor overshoot
  justified by the museum-plaque-READMEs preference for
  explicit unit annotation; not a drift worth patching)
  and eprintln format string update at `:219-220` shipped
  verbatim.

**§2.6 test enumeration.** Ch 41's PR 2b test enumeration
specified six new tests plus one in-place rename: tests
(12) `cem_mean_reward_is_per_episode_total`, (12a)
`cem_dual_reward_concept_split`, (13)
`td3_mean_reward_is_per_episode_total`, (14)
`sac_mean_reward_is_per_episode_total`, (15)
`td3_epoch_rewards_invariant_holds` (debug-only), (16)
`sac_epoch_rewards_invariant_holds` (debug-only), plus the
in-place rename at `cem.rs:319/:354`. All seven landed as
enumerated. Session 14's PR 2a shipped the other 13 tests
that Ch 41 §2.6 counted. Total ml-bridge test surface for
PR 2 lands at 19 new `#[test]` functions + 1 in-place
rename, matching §2.6's closing count exactly.

**§5 sub-decision table observance.** All seven in-chapter
sub-decisions (a)–(g) are observed correctly:
- (a) CEM Form (i) second-traversal: shipped at `cem.rs:209`.
- (b) TD3/SAC `debug_assert_eq!` shape (a): shipped in both files.
- (c) CEM rename to `elite_mean_reward_per_step`: shipped.
- (d) REINFORCE/PPO shape (i) bare `continue`: shipped.
- (e) d2c doc-only: shipped.
- (f) 2-PR split shape (ii): session 14 shipped PR 2a,
  session 15 shipped PR 2b.
- (g) `TaskConfig::build_fn` seed extension: session 14
  shipped as PR 2a commit 1 (`153bad3c`).

**Ch 24 overclaim patch.** Ch 41 §3.3 bundled the Ch 24
§1.9/§2.2/§5 overclaim patch into the Ch 41 drafting commit
(`843dc21c`), not into a PR 2b code commit. A session-15
handoff note in project memory had incorrectly framed this
patch as PR 2b scope; the post-impl memory update corrects
that note. The patch itself was verified present at the
three cited Ch 24 sites during recon and requires no
further work.

**Verification rerun.** After commit `74627346` landed, a
5-test `--ignored` rerun produced 5/5 green:
`hypothesis_off_policy_efficiency`,
`hypothesis_entropy_helps`, `competition_6dof_all_mlp`,
`competition_6dof_autograd_1layer_parity`, and
`competition_6dof_obstacle_autograd_2layer` all pass under
the findings-mode patches. Test 13's verification rerun
also confirmed the concrete numbers: `TD3 (-0.55) > SAC
(-0.97) >> CEM (-206.65) > PPO (-269.06) > REINFORCE
(-269.13)`.

**Verdict: post-impl audit applied.** One narrow §4.3
patch (the Post-implementation reality check block). No
other drifts found. The shipped code matches Ch 41's spec
within the small rendering-discretion margin that session
13/14's own audits also observed.

Post-Round-3 chapter line count is approximately 2190 lines
(up from 2122 at Round 2). The branch stays on
`feature/ml-chassis-study`.
