# Review log — Appendix B: Test inventory

Appendix B is a reference index enumerating the tests Part 4's
PR plans add, modify, or drop, organized by PR (1a, 1b, 2a, 2b,
3a, 3b). Same genre as Appendix A: reference material whose
value is citation accuracy. Ungated per the Ch 01 protocol.

## Genre note

Appendix B is a PR-implementor checklist. A PR 2a reviewer can
scan the PR 2a subsection and know the exact test count that
"PR 2a is ready for review" means. The factual pass therefore
verifies every test function name against the committed
chapter source and every file path / line citation against
the current source tree.

## Factual pass

**Status:** Done in-chapter during drafting, via targeted
Read against Ch 40 §2.3 + §3.4, Ch 41 §2.6, Ch 42 §4.8 +
§5.6 + §6.6 + §6.7 + §6.8, and cross-referenced decision
sources in Ch 15 (D6, D9, D10, D11), Ch 24 (§3.5, §4.3,
§4.6), and Ch 32 (§3.3).

| # | Claim | Verdict |
|---|---|---|
| 1 | PR 1a has ~10 inline `#[test]` functions in `prf.rs` | VERIFIED against Ch 40 §2.3 closing "The test count is ~10 `#[test]` functions" |
| 2 | PR 1a test names are not chapter-locked | VERIFIED — Ch 40 §2.3 lists coverage items by property, not by function name |
| 3 | Seven PR 1a coverage items enumerated | VERIFIED against Ch 40 §2.3 bullet list |
| 4 | `parallel_matches_sequential_with_langevin` at `sim/L0/tests/integration/batch_sim.rs` | VERIFIED against Ch 40 §3.4 (a) |
| 5 | `parallel_and_sequential_traces_match_langevin` gated on `#[cfg(feature = "parallel")]` | VERIFIED against Ch 40 §3.4 (a) and Ch 15 D9 strategy A |
| 6 | Three master seeds: `0`, `0xD06F00D_42`, `u64::MAX` | VERIFIED against Ch 15 D10 |
| 7 | PR 1b updates five existing langevin.rs tests at `:241-266`, `:269-276`, `:311-363` | VERIFIED against Ch 40 §3.4 (c) |
| 8 | PR 1b's D2 4-arg ripple is 56 call sites across 18 files | VERIFIED against Ch 40 §3.4 (b) |
| 9 | PR 2a ships 13 new tests in `competition.rs`'s test module | VERIFIED against Ch 41 §2.6 closing count ("Thirteen new tests in `competition.rs`'s test module for PR 2a") |
| 10 | All 13 PR 2a test names | VERIFIED against Ch 41 §2.6 enumerated list; my appendix renumbered from the chapter's `1, 2, 2a, 3, ..., 11` form to flat `1-13` |
| 11 | `run_replicates_preserves_strict_gt_tie_breaking` is the load-bearing regression guard for `competition.rs:363-379` | VERIFIED against Ch 41 §2.6 test 2a |
| 12 | PR 2b ships 6 new tests + 1 in-place rename | VERIFIED against Ch 41 §2.6 closing count |
| 13 | `cem_dual_reward_concept_split` regression guard for Ch 24 §3.5 | VERIFIED against Ch 41 §2.6 test 12a |
| 14 | Five debug-assert test names (`td3_epoch_rewards_invariant_holds`, `sac_epoch_rewards_invariant_holds`) | VERIFIED against Ch 41 §2.6 tests 15, 16 |
| 15 | `cem_elite_mean_reward_per_step_key_present` is an in-place rename at `cem.rs:319` and `:354` | VERIFIED against Ch 41 §2.6 test 17 |
| 16 | PR 3a SA tests: `sa_name`, `sa_smoke_2dof`, `sa_best_tracker_monotone`, `sa_checkpoint_roundtrip` | VERIFIED against Ch 42 §4.8 |
| 17 | PR 3a analysis tests: 10 tests from `bootstrap_diff_means_positive_ci` through `run_rematch_positive_short_circuits` | VERIFIED against Ch 42 §5.6 |
| 18 | PR 3b single test `d2c_sr_rematch` with `#[ignore = "requires --release (~30-60 min)"]` | VERIFIED against Ch 42 §6.6 |
| 19 | `d2c_sr_rematch` test gate shape is (α) protocol-completes-cleanly | VERIFIED against Ch 42 §6.7 |
| 20 | `BOOTSTRAP_RNG_SEED = 0xB007_0057_00AA_0055` | VERIFIED against Ch 42 §6.6 code block at `:3599` |
| 21 | Integration-test placement vs inline is defended at Ch 42 §6.8 | VERIFIED |
| 22 | Dropped tests `reinforce_zero_fallback_skips_epoch` and `ppo_zero_fallback_skips_epoch` | VERIFIED against Ch 41 §2.6 closing note |
| 23 | Drop reason: zero-fallback branch is structurally unreachable | VERIFIED against Ch 41 §2.6 closing note |
| 24 | Headline count = 36 concretely named new tests + ~10 unnamed PR 1a = ≈46 total | VERIFIED by direct sum |

**Drift items fixed during drafting:**

1. **`Ch 22 §3.1` citation for CEM hyperparameters.** First
   draft cited "[Ch 22 §3.1]" as the source of the rematch
   fixture's CEM hyperparameters (`elite_fraction`,
   `noise_std`, `noise_decay`, `noise_min`). Ch 22 has no
   H3 subsections at all — only H2 sections — so the
   citation is factually wrong. The hyperparameters come
   directly from `d2c_cem_training.rs:283`, per Ch 42's own
   citation at line 2176. *Fix:* replaced with the
   `d2c_cem_training.rs:283` citation and the inlined
   parameter values; SA's hyperparameter citation
   ([Ch 42 §4.5]) retained.

2. **Headline count off by one.** First draft said "35
   concretely named" but the actual sum is 2+13+6+14+1 = 36.
   *Fix:* corrected to 36 and added the per-PR breakdown
   `2 + 13 + 6 + 14 + 1 = 36` for readers who want to
   verify.

**Verdict:** PASS. 24 claims verified, 2 drift items fixed
single round.

## Thinking pass

**Status:** Author cold-read in one pass. Light for the same
reasons as Appendix A — the test inventory does not decide
anything; it indexes what other chapters already decided.

**Verdict:** Ship. No must-fix findings.

**Three should-fix items considered and resolved:**

1. **Numbering convention: reset-per-PR vs continuous.**
   The appendix uses continuous numbering for the 36
   concretely named tests (PR 1b unnumbered + PR 2a 1-13 +
   PR 2b 14-20 + PR 3a SA 21-24 + PR 3a analysis 25-34 +
   PR 3b 35). PR 1a uses its own 1-7 numbering for
   *coverage items* (not test function names) because its
   names are not chapter-locked. Considered: resetting per
   PR so PR 2a → 1-13, PR 2b → 1-7, etc. *Decision:*
   continuous numbering wins for unambiguous cross-reference
   ("test 34" means the `run_rematch_positive_short_circuits`
   test, anywhere). The PR 1a collision with PR 2a's
   numbers is visually disambiguated by section headers and
   by PR 1a's items being framed as coverage themes
   rather than test functions.

2. **Whether to list test-local helpers (`MockAlgorithm`,
   `rematch_task()`, constants like `OBS_DIM`, `N_ENVS`,
   `REMATCH_BUDGET_STEPS`) in the inventory.** *Decision:*
   No. These are test-local rendering details, not
   separate `#[test]` functions, and the "What this
   appendix does not cover" closer names them explicitly as
   out of scope. A reader who wants helper details reads
   the relevant chapter section.

3. **Whether to include benchmark tests (`sim/L0/core/benches/`,
   `sim/L0/ml-bridge/benches/`).** *Decision:* No. The study's
   PR plans do not touch the benches. The closer names them
   as explicitly out of scope.

## Cross-appendix consistency

Appendix B's entries align with Appendix A's in two places
worth checking:

- **`SeedSummary::from_rewards` empty-slice contract.**
  Appendix A row (sim-ml-bridge) cites Ch 24 §4.3 and
  Ch 41 §2.1; Appendix B test 11 (`seed_summary_from_rewards_empty_returns_none`)
  cites Ch 24 §4.3 as well. Consistent.
- **`EpochMetrics::mean_reward` unit contract.** Appendix A
  row (sim-ml-bridge) cites Ch 24 D1 and Ch 41 §2.2;
  Appendix B tests 14, 16, 17
  (`cem_mean_reward_is_per_episode_total`,
  `td3_mean_reward_is_per_episode_total`,
  `sac_mean_reward_is_per_episode_total`) are the runtime
  verification of that contract. Consistent.

## Scope-closing note

With Appendices A and B committed, the study book is
structurally complete: Part 0–4 are all linked in SUMMARY.md,
and both appendices provide reference material for the PR
implementor. The remaining work — PR 1a/1b/2a/2b/3a/3b
actual code landings — is a separate project phase and
lives outside the study book's scope. The appendices close
Phase 4.
