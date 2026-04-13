# Review log — Appendix A: API inventory

Appendix A is a reference index, not an argument chapter. It
enumerates the public (and a handful of private-but-specified)
items the Part 4 PR plans create or modify, organized by crate,
with citations to the owning chapter sections. Ungated per the
Ch 01 protocol — reference material does not need the gated
author-pause-for-approval flow that the Part 4 argument
chapters used.

## Genre note

This appendix is reference material whose whole value is
citation accuracy. The factual pass is therefore the most
important pass: every symbol name, file path, kind
annotation, and chapter cross-reference must be verified
against the committed Part 4 chapters and the current
source tree. The thinking pass is light because there are
no decisions to pressure-test — the appendix does not
decide anything; it indexes what other chapters already
decided.

## Factual pass

**Status:** Done in-chapter during drafting, via targeted
Read and Grep against the committed Ch 15/23/24/31/32/40/41/42
source and the ml-bridge / thermostat / core crate trees.

| # | Claim | Verdict |
|---|---|---|
| 1 | `prf::chacha8_block(key: &[u8; 32], block_counter: u64) -> [u8; 64]` signature at Ch 40 §2.2 | VERIFIED |
| 2 | `prf::expand_master_seed`, `encode_block_counter`, `box_muller_from_block` signatures at Ch 40 §2.2 | VERIFIED |
| 3 | `prf::splitmix64` is `pub`, the other four prf items are `pub(crate)` | VERIFIED against Ch 40 §2.5's visibility pick |
| 4 | `LangevinThermostat::new` 4-arg signature `(gamma, k_b_t, master_seed, traj_id)` | VERIFIED against Ch 40 §3.2 struct block and Ch 15 D2 |
| 5 | `master_seed`, `master_key`, `traj_id`, `counter` as private fields on the rewritten struct | VERIFIED against Ch 40 §3.2 struct block |
| 6 | `diagnostic_summary` static tuple specified by Ch 15 D8 | VERIFIED — Ch 40 §3.2 explicitly says "The `Diagnose` impl changes per D8" and "D8 specifies a static-only tuple" |
| 7 | `BatchSim::new_per_env<S, F>(prototype: &Arc<S>, n: usize, factory: F)` generic over `S: PerEnvStack` | VERIFIED against Ch 40 §3.3's generic rewrite |
| 8 | `PerEnvStack` trait and `EnvBatch<S>` live in `sim-core::batch` (not sim-thermostat) | VERIFIED against Ch 40 §3.3 |
| 9 | `Competition::run_replicates(&self, tasks, builders, seeds: &[u64])` signature at Ch 41 §2.1 | VERIFIED against Ch 41 §2.1 code block |
| 10 | `Competition::run` becomes one-line wrapper: `self.run_replicates(tasks, builders, &[self.seed])` | VERIFIED against Ch 41 §2.1 |
| 11 | `RunResult.replicate_index: usize` as new field | VERIFIED against Ch 41 §2.1 struct block |
| 12 | `CompetitionResult::find_replicate`, `replicate_best_rewards`, `describe` methods | VERIFIED against Ch 41 §2.1 |
| 13 | `SeedSummary { n, mean, std_dev }` with Bessel's correction | VERIFIED against Ch 24 §4.6 and Ch 41 §2.1 |
| 14 | `SeedSummary::from_rewards` returns `None` on empty slice | VERIFIED against Ch 24 §4.3 "Returns `None` if the slice is empty" and the `seed_summary_from_rewards_empty_returns_none` test at Ch 41 §2.6 |
| 15 | `TaskConfig::build_fn` extended from `Fn(usize) -> ...` to `Fn(usize, u64) -> ...` | VERIFIED against Ch 42 §2.1 |
| 16 | `TaskConfig::build_vec_env(&self, n_envs, seed)` 2-arg form | VERIFIED against Ch 42 §2.1 |
| 17 | Deterministic stock tasks gain `_seed: u64` ignored parameter | VERIFIED against Ch 42 §2.1 prose |
| 18 | `EpochMetrics::mean_reward` unit-uniform contract "mean per-episode total across n_envs trajectories" | VERIFIED against Ch 24 §3.2's pick and Ch 41 §2.2 |
| 19 | REINFORCE/PPO zero-fallback cleanup at `reinforce.rs:213-225` and `ppo.rs:313-325` | VERIFIED against Ch 41 §2.4 "The REINFORCE/PPO zero-fallback cleanup" |
| 20 | CEM `extra["elite_mean_reward_per_step"]` renamed from `extra["elite_mean_reward"]` | VERIFIED against Ch 41 §2.3 |
| 21 | `sim-opt` crate placement at `sim/L0/opt/` with dep list | VERIFIED against Ch 42 §3.1 |
| 22 | `Sa`, `SaHyperparams` struct shapes | VERIFIED against Ch 42 §4.1 code blocks |
| 23 | `Sa::new`, `Sa::from_checkpoint` signatures | VERIFIED against Ch 42 §4.1 code blocks |
| 24 | `impl Algorithm for Sa` at Ch 42 §4.2 | VERIFIED |
| 25 | `BootstrapCi`, `RematchOutcome` types at Ch 42 §5.2 and §5.4 | VERIFIED |
| 26 | `bootstrap_diff_means`, `bootstrap_diff_medians`, `bimodality_coefficient`, `classify_outcome`, `run_rematch` free-function signatures at Ch 42 §5.2–§5.5 | VERIFIED |
| 27 | Constants `REMATCH_MASTER_SEED = 20_260_412`, `N_INITIAL = 10`, `N_EXPANDED = 20`, `REMATCH_TASK_NAME = "d2c-sr-rematch"` | VERIFIED against Ch 42 §5.5 |

**Drift items fixed during drafting:**

1. **Row: zero-fallback cleanup attributed to CEM.** First draft
   listed the row as "`Cem::train` zero-fallback at
   `cem.rs:213-225`." This was wrong on two counts: (a) the
   zero-fallback cleanup in Ch 41 §2.4 is the
   REINFORCE/PPO-specific shape, not CEM, and (b) the line
   numbers `:213-225` and `:313-325` belong to `reinforce.rs`
   and `ppo.rs`, not `cem.rs`. *Fix:* row renamed to
   "REINFORCE/PPO zero-fallback cleanup at `reinforce.rs:213-225`
   and `ppo.rs:313-325`," description rewritten to explicitly
   exclude CEM/TD3/SAC.

2. **`Sa` field count.** First draft said "Seven fields" while
   enumerating eight names. *Fix:* corrected to "Eight fields."
   See the Ch 42 prose drift flag below — Ch 42 §4.1's prose
   says "seven fields on `Sa`" while its own struct block
   shows eight fields, which is a separate Ch 42 drift worth
   a post-commit patch.

3. **`feedback_code_speaks` citation form.** First draft tried
   to link to the memory file via a relative path that would
   not resolve inside the mdbook. *Fix:* replaced the link
   with a plain-text citation naming the preference and the
   memory-file name.

**Verdict:** PASS. 27 claims verified, 3 drift items fixed
single round.

## Ch 42 §4.1 prose drift — post-commit patch candidate

The factual pass surfaced a drift in Ch 42 §4.1 itself (not
in Appendix A). The prose at `42-pr-3-sim-opt-rematch.md:1634`
reads:

> Five fields on `SaHyperparams`, seven fields on `Sa`.

The struct definition in the same section at `:1612-1632`
shows eight fields on `Sa`: `policy`, `hyperparams`,
`current_params`, `current_fitness`, `temperature`,
`best_params`, `best_fitness`, `best_epoch`.

The discrepancy is a prose-level count error. It does not
affect the struct spec or any other claim. A narrow
post-commit patch to Ch 42 (along the `b5cb3f6c` /
`3e1ec0ff` pattern) could correct the prose to "eight fields
on `Sa`" with no other changes. This patch is optional —
the appendix already reports the correct count — and is
flagged here rather than bundled into the appendix commit
because it is a separate chapter's fix.

## Thinking pass

**Status:** Author cold-read in one pass. Light because the
appendix does not decide anything — it indexes what other
chapters already decided — so there is no argument to
pressure-test.

**Verdict:** Ship. No must-fix findings. Three should-fix
items considered and dismissed or applied during drafting:

1. **Whether to include `impl Algorithm for Sa` as a single
   row or split into five rows for `name`, `train`,
   `best_artifact`, `checkpoint`, `set_best`.** Kept as a
   single row — the trait impl is the conceptual unit, and
   splitting into five rows would overspecify without adding
   reader value. The description already names the key
   behaviors ("Metropolis accept/reject with multi-env
   fitness averaging and geometric cooling").

2. **Whether to list `Sa.train` loop internals like the
   inlined `randn` Box–Muller helper.** Not listed — it is
   an internal rendering detail, not a specified public
   item. The "What this appendix does not cover" closer
   names internal helpers as explicitly out of scope.

3. **Whether to list private fields on `LangevinThermostat`
   at all.** Kept. The private fields `master_seed`,
   `master_key`, `traj_id`, `counter` are D7/D4/D5
   specifications — the study explicitly decided their
   shape, and a reviewer of PR 1b's diff should see the
   specification somewhere authoritative. Listing them with
   `Kind = "field (private)"` makes the visibility honest
   without hiding the specification. Pre-rewrite private
   fields like `gamma`, `k_b_t`, and `stochastic_active`
   are *not* listed because they are inherited unchanged.
   The consistency rule is "list fields the study touched."

## Scope-closing note

Appendix A covers only API items. Tests (the other surface
the study's plans enumerate) are Appendix B's scope.
Cross-references between the two appendices: Appendix B's
test descriptions name the API items under test; Appendix A
does not enumerate the tests themselves beyond the brief
"specified in Ch 41 §2.6" cross-reference on the
`seed_summary_from_rewards_empty_returns_none` test in
factual-pass row 14.
