# CI Optimization Spec

> Make the Quality Gate **faster** without ever reducing the coverage that
> gates a merge.
>
> **Hard invariant:** nothing reaches `main` without a full run of the suite.

## Goal

Cut PR feedback from ~30–50 min to single-digit minutes. The lever is
**parallelism and scoping**, never dropped coverage — every speedup here either
splits the same work across more runners or scopes *pre-merge* feedback while a
*full* run still gates the merge and backstops `main`.

## Current state

`.github/workflows/quality-gate.yml` is one workflow on **two triggers**:

- `pull_request: [main, develop]` — the pre-merge gate (gates the merge button).
- `push: [main, develop]` — the post-merge backstop (catches squash skew).

Today both run the *identical* full suite, so the heavy gate is paid twice per
change. Caching is `Swatinem/rust-cache@v2` (good; deps-only, save-if main/develop).

The long pole was `tests-release`: a ~29-min sequential monolith. It is
**test-execution-bound, not compile-bound** — stochastic-physics validators
(90M–600M-step Boltzmann / CEM / REINFORCE / SA / MuJoCo-conformance) run
minutes each; compile is only ~5 of the 29 min. The repo cache sits near the
10 GB GitHub ceiling, so adding cache keys risks LRU eviction → cold builds.

## Design — three execution contexts

| Context | Trigger | Scope | Role |
|---|---|---|---|
| PR | `pull_request` | affected subset (changed crates + **reverse-dep closure**) | fast feedback, informational |
| Merge | `merge_group` | **full** suite | the required gate |
| Backstop | `push: [main]` | **full** suite | confirm main green / squash skew |

### ★ The load-bearing correctness line

The pre-commit hook safely scopes clippy/fmt to *changed crates* — lint is not
cross-crate. **Tests are not** like that: an L0 change (`sim-core`) can break
everything above it. So "only test what changed" MUST mean **changed crates +
their full reverse-dependency closure**, plus a **full-fallback valve** (any
change to `Cargo.lock`, workspace `Cargo.toml`, `rust-toolchain*`,
`.github/workflows/**`, `xtask/**`, or shared fixtures → run everything). The
full suite always runs on `merge_group` and `push:[main]`. A naive
changed-crate-only scope would be the "passing CI ≠ tests ran" failure mode.

## Rollout (independent PRs — measure each)

### PR #1 — pure speedup, no logic change ✅ (this PR)

- **Shard `tests-release` into a 3-leg matrix**, mirroring the proven `grade`
  3-shard structure (job id unchanged, so `quality-gate`'s `needs:` waits for
  all legs → branch protection still gates every crate). Unlike `grade`, the
  assignment cannot delegate to a round-robin tool: because the job is
  test-execution-bound, round-robin over the sorted crate list *clusters* the
  heavy learners onto one shard (the same imbalance `grade` shows: shard 1
  ~18 min vs ~10), and a naive 2-way split (#351, reverted) put `sim-soft`
  alone vs the four learners and barely moved the critical path (29→27). The
  legs are therefore **hand-balanced to spread the five heavy learners**
  (`sim-conformance-tests`, `sim-thermostat`, `sim-rl`, `sim-opt`,
  `sim-ml-chassis`) one-or-two per shard. 3 legs (not matrix-per-crate) keeps
  the cache-key count bounded under the 10 GB ceiling. **Re-tune from measured
  per-shard wall-clock if one leg dominates.**
- **`timeout-minutes` on every job** (the missing hang guard — jobs previously
  inherited GitHub's 360-min default ceiling). Heavy jobs 45 min, lighter jobs
  10–30.

  Expected: critical path ~29 → ~13–15 min. PR #1 *is* the measurement — record
  actual before/after below.

### PR #1b — Tier-A speedups (zero correctness risk)

cargo-nextest; a faster linker (mold/lld) ± sccache. Watch cache-key count vs
the 10 GB ceiling.

### PR #2 — affected-subset scoping (`xtask affected`)

`git diff --name-only base...head` → owning crate → **reverse-dep closure** via
`cargo metadata`, plus the full-fallback valve above. Emits `affected_crates` +
`needs_full` as job outputs; PR-context `if:` gating consumes them.
`push:[main]` stays full.

### PR #3 (optional, "complete") — merge queue

Enable `merge_group`; the full required gate moves there; PR jobs become
informational. Required-check wiring: keep the always-run `quality-gate`
summary job as the **single required check** (`needs:` all; passes on success
*or* skipped) so a conditionally-skipped heavy job can't block the merge button.

## Risks → mitigations

- Affected-set misses a dependent → reverse-dep **closure** + full-fallback
  valve + full run on `merge_group` / `push:[main]`.
- Two PRs each pass lean subsets but interact → merge queue (PR #3); until then
  the post-merge backstop catches it (non-blocking).
- A skipped *required* check blocks the merge button → single always-run
  summary gate (PR #3).
- More shards → more cache keys → eviction under the 10 GB ceiling → cold
  builds. Keep shard counts bounded; measure.

## Measurements

| Change | Critical path before | after | Notes |
|---|---|---|---|
| PR #1 (shard tests-release) | ~29 min | _TBD from CI run_ | per-shard wall-clock to be recorded; rebalance if one leg dominates |
</content>
</invoke>
