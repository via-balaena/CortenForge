# Phase 1 — BF-7 + BF-8 Preemptive Reconciliation Scope

**Status:** Scope memo — second PR of the six-phase foundation-up sequence. Pre-code recon complete; ready to edit.
**Date:** 2026-04-26, post PR #216 (`a0e18beb`) — L0 foundational refactor MERGED.
**Branch:** `feature/phase-1-bf7-bf8-preemptive`, off main `a0e18beb`.
**Follows:** `project_gameplan.md` (memory) — Phase 1 of the six-phase foundation-up sequence. Phase 0 (Track A bundle) absorbed by PRs #213 (CI xtask unification) + #216 (L0 refactor); recon-confirmed 2026-04-26 via `quality-gate.yml` + `sim/L0/mjcf/src/builder/mod.rs:234`.
**Target:** ~30–60 changed lines across two book files: `docs/studies/soft_body_architecture/src/100-optimization/00-forward.md` (BF-7) + `docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md` (BF-8). One PR. Doc-only — zero production-code change.

Phase 1 is **book-iteration** — reconcile drift between the shipped `sim-soft` API and the book that documents it, before Phase 2 starts extending either. The book is the spec; when shipped code and book disagree, future readers can no longer treat the book as the spec. BF-7 and BF-8 sit on the exact surfaces Phase 2 (multi-element FEM) and downstream phases will extend, so fixing preemptively is foundation-up.

**Why this matters under the sharpened scope (2026-04-24).** `RewardBreakdown` (BF-7) is the structured reward carrier every downstream optimizer in Part 10 reads from; its degenerate-scene contract has to be authoritative before Phase 2's multi-element scenes start populating fields the 1-tet skeleton left as `NaN`. `Solver::step` (BF-8) is the trait-method signature consumers wire VJPs through; its `Var`-handle parameter is what lets `tape.push_custom` bind `theta_var` as a parent — the chassis API model PR #213 ships. Phase 2 will write a multi-element solver against the same trait; if the book still pins `theta: &Tensor<f64>`, the new solver will face the same fork the skeleton did.

## 0. Baseline — current state of book vs shipped code

**BF-7 — `RewardBreakdown` NaN-sentinel contract not stated in book.**

Shipped code (verified 2026-04-26):
- `sim/L0/soft/src/observable/basic.rs:78-82` populates `pressure_uniformity: f64::NAN`, `coverage: f64::NAN`, `peak_bound: x_peak`, `stiffness_bound: theta_val / x_peak` on the 1-tet skeleton scene.
- `sim/L0/soft/src/readout/reward_breakdown.rs:53-67` `score_with` explicitly NaN-skips per-field via `if !field.is_nan() { acc += weight * field }`. Composite scalar is finite even when individual fields are sentinels.
- The struct docstring (lines 44-52) names the NaN-sentinel pattern.

Book (`docs/studies/soft_body_architecture/src/100-optimization/00-forward.md`):
- Line 47 (item 4 of the stage-by-stage forward map) names `RewardBreakdown` as "the structured carrier" with four terms — `pressure uniformity, coverage, peak-pressure barrier, and effective-stiffness-bound`. **No mention of the NaN-sentinel contract for degenerate scenes.**
- Line 53 import + lines 63-67 trait signature — both reference `RewardBreakdown` without flagging that some scenes structurally cannot populate every field.
- `RewardBreakdown` is cited from many other chapters (regression tests, sim-to-real correction, gradcheck, BayesOpt) — but every cite assumes "all four fields are populated." A 1-tet test fixture using a half-populated breakdown would silently pass through to those chapters' downstream consumers without warning.

The drift: shipped code defines the NaN-sentinel as canonical contract; the book never names the contract; future authors writing against the book would assume all four fields are always populated.

**BF-8 — `Solver::step` signature in book pre-dates PR #213's `push_custom` chassis model.**

Shipped code (verified 2026-04-26):
- `sim/L0/soft/src/solver/mod.rs:77-93` defines `pub trait Solver` with:
  ```rust
  fn step(
      &mut self,
      tape: &mut Self::Tape,
      x_prev: &Tensor<f64>,
      v_prev: &Tensor<f64>,
      theta_var: Var,
      dt: f64,
  ) -> NewtonStep<Self::Tape>;
  ```
- `sim/L0/soft/src/solver/backward_euler.rs:414-446` concrete impl: reads θ value via `tape.value_tensor(theta_var)` (line 425), assembles + Newton-iterates, then pushes `NewtonStepVjp` onto the tape via `tape.push_custom(&[theta_var], x_final_tensor, Box::new(vjp))` (line 445) so the VJP fires automatically during `tape.backward`.
- `replay_step` keeps `theta: &Tensor<f64>` (line 104) — replay does not compose into the tape, so no `Var` handle is needed.

Book (`docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md`):
- Line 92: `Solver::step` parameter is `theta: &Tensor<f64>` — the tensor-by-reference shape. Pre-dates PR #213's `push_custom` model where VJPs are bundled with nodes at forward-pass time, parented by `Var` handles.
- Line 100: `Solver::replay_step` parameter is `theta: &Tensor<f64>` — **correct as-is**, matches shipped code.
- Line 111: determinism-tuple text reads `(x_prev, v_prev, theta, dt)` as the inputs that `step`'s output is determined by. The tuple-as-stated is loose enough to remain correct (θ-the-value is what determinism is over, and `theta_var` is a handle to that value).
- Line 13 (trait surface table): minimal-surface lists `step(tape, ...) -> NewtonStep` — loose, doesn't pin any θ shape.

The drift: book's pinned signature differs from shipped trait; consumers reading the book to wire a new `Solver` impl (Phase 2 multi-element solver, Phase 5 contact solver, Phase E GPU solver) would write against the wrong API.

**Cross-reference impact (verified 2026-04-26 via `grep -rn "theta: &Tensor<f64>" docs/studies/soft_body_architecture/src/`):** the only other Part 11 site that pins a `Solver::step`-shaped signature is the same chapter (line 92 + 100). The Part 5 backward-Euler chapters carry a separate, structurally-deeper drift discussed in §7.

## 1. The two load-bearing invariants

| # | Invariant | What green looks like |
|---|---|---|
| I-A | Book documents the shipped `RewardBreakdown` contract end-to-end | Part 10 Ch 00 names the NaN-sentinel pattern as expected for scenes that do not structurally support every reward term, references the in-source docstring at `sim/L0/soft/src/readout/reward_breakdown.rs`, and the contract is consistent with `score_with`'s NaN-skip semantics. |
| I-B | Book pins the shipped `Solver::step` signature exactly | Part 11 Ch 01 §"`Solver` — the only `dyn`-safe public trait" carries `theta_var: Var` for `step`, retains `theta: &Tensor<f64>` for `replay_step`, and adds one paragraph naming the rationale (push_custom + Var-parent model). The minimal-surface table row stays loose. |

I-A is doc-content additive (a paragraph, not a rewrite); I-B is a one-line signature swap plus an explanatory paragraph. Neither touches code.

## 2. The two changes — concrete edits

### Change 1: BF-7 — Part 10 Ch 00 RewardBreakdown sentinel paragraph

`docs/studies/soft_body_architecture/src/100-optimization/00-forward.md`, **insert one new paragraph** between current item 4 of the stage-by-stage forward map (line 47) and the trait-signature code block (line 49). Approximate text:

> **NaN-sentinel contract for structurally-degenerate scenes.** Some scenes do not structurally support every reward term — e.g., a single-tet probe scene with no contact has exactly one Gauss point and no surface for `pressure_uniformity` or `coverage` to evaluate against. `Observable::reward_breakdown` reports such fields as `f64::NAN`; `RewardBreakdown::score_with` explicitly skips NaN fields rather than letting them propagate through the scalar composition (IEEE 754's `NaN × 0.0 = NaN` would otherwise contaminate the result). Downstream consumers reading the per-term breakdown — [Ch 03 preference learning](03-preference.md), [Ch 05 sim-to-real correction](05-sim-to-real.md), [Part 11 Ch 04 §01 regression tests](../110-crate/04-testing/01-regression.md) — must handle NaN per-term. The contract is: **a NaN field means structurally undefined for this scene, not "computation failed."** The struct's source-level docstring at [`sim-soft::readout::RewardBreakdown`](../110-crate/00-module-layout/09-readout.md) carries the canonical statement.

That's the entire BF-7 edit — one paragraph, ~8 lines including the bold header. No other text on the page changes.

### Change 2: BF-8 — Part 11 Ch 01 Solver::step signature + rationale

`docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md`, **two edits**:

**(a)** Line 92 — replace `theta: &Tensor<f64>,` with `theta_var: Var,` inside the `Solver::step` signature. The block becomes:

```rust
fn step(
    &mut self,
    tape: &mut Self::Tape,
    x_prev: &Tensor<f64>,
    v_prev: &Tensor<f64>,
    theta_var: Var,
    dt: f64,
) -> NewtonStep<Self::Tape>;
```

`Solver::replay_step` (line 100) stays `theta: &Tensor<f64>` — matches shipped code.

**(b)** Insert a new paragraph between the existing "Two concrete impls" paragraph (current line 109, ending `... and the `Solver` impl picks one.`) and the `&mut self` mutability discipline paragraph (current line 111, starting `\`Solver::step\`'s \`&mut self\` admits *intra-call* mutability...`). The flow becomes: trait → impls → why-this-asymmetry → mutability discipline. Approximate text:

> **Why `theta_var: Var` on `step`, `theta: &Tensor<f64>` on `replay_step`.** `step` composes into the autograd tape via [chassis `Tape::push_custom`](../../80-gpu/04-chassis-extension/02-vjp-api.md), which expects a `Var` handle for each parent node — `theta_var` is what lets the `NewtonStepVjp` bind θ as a parent and fire automatically during `tape.backward(reward_var)`. The shipped `CpuNewtonSolver::step` reads θ's primal value via `tape.value_tensor(theta_var)` then assembles the residual + tangent against that snapshot, then pushes `NewtonStepVjp` with `tape.push_custom(&[theta_var], x_final_tensor, ...)`. `replay_step` is the [checkpointed-step VJP](../../60-differentiability/04-checkpointing/00-uniform.md) entry point; it does not compose into a tape (no `&mut Tape` parameter) and rebuilds its own factor on a scratch context, so a bare tensor is the right shape. The asymmetry is intentional: tape-composing entry points take `Var`, pure-function entry points take `Tensor`.

Determinism-tuple text on line 111 (`(x_prev, v_prev, theta, dt)`) stays as-is — θ-the-value is the determinism quantity, and "theta" reads as the value-level concept regardless of whether the live API parameter is the value or a `Var` handle to it.

The chapter's `use` block at the top of the file does not import `Var`; this is text-level prose so explicit imports aren't required (no Rust block in the chapter currently imports beyond the implicit context). The `Solver` code block is illustrative-trait-surface, not a compilable example.

### Change 3 — minimal-surface table row tightening: SKIP

Decided per Decision F (§3): leave the table row at line 13 as-is. The loose form `step(tape, ...) -> NewtonStep` is forward-compatibility robust; the full code block on lines 87-95 is the authoritative pin. Tightening would add a redundant signature anchor with no readability gain.

## 3. Locked decisions

**Decision A: BF-1..BF-6 stay drip-cadence — none folded into Phase 1.**
Per gameplan §"Phase 1": "Other BF-1..BF-6 stay drip-cadence; pull into this PR only if cheap and on-path." Recon-verified per BF:
- **BF-1, BF-2, BF-3** sit in Part 6 Ch 02 (faer Cholesky pseudocode); not on the Phase 1 file path. **Excluded.**
- **BF-4** (`Differentiable::register_vjp`) sits in 00-core.md lines 117-140 — same chapter as BF-8. But the fix per §13 is a conceptual rewrite (drop register_vjp from the trait OR convert to factory signature) — book's registry-key autograd model doesn't fit chassis's `push_custom` model at all. Not cheap, not signature-drift. **Excluded.**
- **BF-5** (9×9 tangent flattening) sits in Part 2 Ch 04; not on path. **Excluded.**
- **BF-6** (`Differentiable::ift_adjoint`) sits in 00-core.md lines 123-128 — same chapter. Same root cause as BF-4: book API pre-dates push_custom. Same exclusion rationale. **Excluded.**

Cutting hard per `feedback_scope_creep_correction`. Phase 1 is BF-7 + BF-8 only.

**Decision B: Part 5 Ch 00 + 50-time-integration/00-backward-euler/01-newton.md drift stays out of scope.**
Recon (§0 cross-reference impact, 2026-04-26) surfaced two free-standing `pub fn step(...)` pseudocode blocks at `50-time-integration/00-backward-euler.md:68-91` and `50-time-integration/00-backward-euler/01-newton.md:61-84` that share BF-8's signature drift (`theta: &Tensor<f64>`) AND additionally use `tape.record_ift_step(&step)` — a BF-4/BF-6-class autograd-API mismatch (book's pre-PR-#213 registry model). Fixing properly means rewriting the pseudocode body to use `tape.push_custom`, not just swapping a parameter. Logged as new finding **BF-9** in §7; deferred to drip-cadence.

**Decision C: Pre-squash tag — skip.**
Per `feedback_pre_squash_tag.md`: tag `<branch>-pre-squash` only when the PR touches chapters with `git log --reverse` claims or hash anchors. Phase 1 edits two prose chapters — no chapter hash anchors, expected ≤2 commits. Skip the pre-squash tag unless commit count grows beyond 5 (mirrors Phase 0's policy).

**Decision D: ForwardMap signature in 00-forward.md stays `theta: &Tensor<f64>`.**
`ForwardMap::evaluate` (line 65) and `ForwardMap::gradient` (line 75) take `theta: &Tensor<f64>`. ForwardMap is a higher-level wrapper that does NOT directly compose into the autograd tape — it owns the tape internally and constructs `theta_var` from `theta` via `tape.param_tensor(theta.clone())` (verified at `sim/L0/soft/src/readout/skeleton_forward_map.rs:524`). The bare-tensor parameter is correct for ForwardMap and matches shipped code. **Not part of BF-8.**

**Decision E: BF-9 backlog entry logged in `soft_body_walking_skeleton_scope.md §13` as commit 3 of this PR.**
BF-9 was discovered during Phase 1 R-2 cross-chapter audit (Part 5 Ch 00 + 01-newton.md `pub fn step` pseudocode using `theta: &Tensor<f64>` + `tape.record_ift_step` — same drift class as BF-4/BF-6 but pseudocode-deeper). Per `feedback_explicit_deferrals`: never silently defer; the §13 backlog is where future drip-cadence work picks up findings. ~5 added lines as a new table row, on-theme, low-risk. In-PR rather than separate followup because the discovery is causally tied to Phase 1 recon.

**Decision F: minimal-surface table row at 00-core.md:13 stays loose — no tightening.**
The current row text `step(tape, ...) -> NewtonStep` does not pin θ shape; the authoritative pin is the full code block on lines 87-95. Tightening to `step(tape, theta_var, ...) -> NewtonStep` adds a redundant anchor that would itself need maintenance on future signature evolution. Forward-compat trumps signature redundancy.

**Decision G: BF-7 paragraph cross-references stay at three (Ch 03 / Ch 05 / Part 11 Ch 04 §01) — no §00 addition.**
The three cited consumers are the ones that actively read per-term breakdown values where NaN handling is load-bearing. Part 11 Ch 04 §00 (unit tests) is upstream of the contract rather than a consumer of it; adding it would dilute the paragraph's "downstream consumers" framing and push past the ~8-line density target.

## 4. Test harness

Phase 1 is doc-only; the verification surface is mdbook + grep, not `cargo test`.

- **mdbook build clean.** From `docs/studies/soft_body_architecture/`, `mdbook build` succeeds with zero errors / warnings on the edited chapters. Internal links resolve (the new paragraphs reference `../110-crate/00-module-layout/09-readout.md`, `03-preference.md`, `05-sim-to-real.md`, `../110-crate/04-testing/01-regression.md`, `../../80-gpu/04-chassis-extension/02-vjp-api.md`, `../../60-differentiability/04-checkpointing/00-uniform.md` — all need to resolve).
- **Grep audit post-edit.** `grep -rn "theta: &Tensor<f64>" docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md` returns exactly `replay_step` + `fd_wrapper` + `reward_breakdown` (three remaining legitimate uses) — no `step(...)` signature line.
- **Grep audit cross-chapter.** `grep -rn "RewardBreakdown" docs/studies/soft_body_architecture/src/100-optimization/00-forward.md` count ≥ existing baseline (we are adding references, not removing).
- **Workspace `cargo xtask grade sim-soft`** holds A under integration-only (sanity check that no production code change leaked into the PR).

## 5. Green checklist

- [ ] `mdbook build` clean from `docs/studies/soft_body_architecture/` with zero errors / warnings on edited chapters.
- [ ] All six inline links in the new paragraphs resolve (mdbook fail-fasts on broken links; recon-confirmed all six target paths exist 2026-04-26).
- [ ] `cargo xtask grade sim-soft` A under integration-only profile (sanity — should be unaffected since no Rust change).
- [ ] Recon-confirmed cross-chapter grep audit per §4.
- [ ] BF-9 logged in §13 of `soft_body_walking_skeleton_scope.md` as commit 3 (per Decision E).
- [ ] CI Quality Gate green — `tests-debug` + `grade` jobs pass; no regression from doc-only PR.
- [ ] Pre-squash tag — **skip** unless commit count grows beyond 5 (Decision C).
- [ ] Platform-agility invariant: Phase 1 reconciles documentation against shipped API, no platform extension — clear.

## 6. Stress-test rounds (most executed pre-code via recon)

| # | Round | Status | Result / Hook |
|---|---|---|---|
| R-1 | Confirm shipped sim-soft API matches the proposed book signatures | **Done pre-code** | `sim/L0/soft/src/solver/mod.rs:86-91` confirms `Solver::step(... theta_var: Var ...)`. `sim/L0/soft/src/observable/basic.rs:78-82` confirms `pressure_uniformity: f64::NAN, coverage: f64::NAN`. `sim/L0/soft/src/readout/reward_breakdown.rs:53-67` confirms `score_with` NaN-skip semantics. |
| R-2 | Cross-chapter scope audit — what other book sites pin `Solver::step` shape? | **Done pre-code** | grep across `docs/studies/soft_body_architecture/src/` for `theta: &Tensor<f64>`: only `00-core.md:92` is a real `Solver::step` site. Lines `100, 132, 153` are `replay_step / fd_wrapper / reward_breakdown` (other APIs). Part 5 Ch 00 sites at `50-time-integration/00-backward-euler.md:68-91` + `01-newton.md:61-84` carry deeper drift (BF-9, deferred). |
| R-3 | BF-1..BF-6 cheap-and-on-path audit | **Done pre-code** | Per Decision A. None on file path or none cheap. All deferred. |
| R-4 | New paragraph link integrity | Live in mdbook build | Six new internal links across the two new paragraphs; mdbook will fail-fast on any unresolved one. |
| R-5 | Determinism-tuple text robustness on line 111 of 00-core.md | **Done pre-code** | The phrase `(x_prev, v_prev, theta, dt)` reads as value-level inputs. Determinism is over θ-the-value regardless of whether the API parameter is `theta` or `theta_var`. No edit needed. |
| R-6 | Risk-mitigation review per `feedback_risk_mitigation_review` | Live pre-commit | Specific lens to apply: (i) does the new BF-7 paragraph create a reading where downstream consumers think NaN is acceptable in *non-degenerate* scenes? Mitigation: word "structurally" prominently. (ii) does BF-8's rationale paragraph create confusion about `replay_step`'s asymmetry being a bug rather than intentional? Mitigation: name "intentional" explicitly. |

## 7. What Phase 1 does NOT do

- Does NOT touch any Rust source. Doc-only PR.
- Does NOT fix BF-1, BF-2, BF-3, BF-4, BF-5, BF-6 (per Decision A). All stay drip-cadence.
- Does NOT touch Part 5 Ch 00 (`50-time-integration/00-backward-euler.md`) or its sub-leaf 01-newton.md, despite both carrying BF-8-shaped drift plus deeper BF-4/BF-6-class drift (`tape.record_ift_step` vs `tape.push_custom`). Logged as **BF-9** for drip-cadence: "Part 5 Ch 00 + 50-time-integration/00-backward-euler/01-newton.md `pub fn step` pseudocode pre-dates PR #213 chassis API; uses `theta: &Tensor<f64>` parameter and `tape.record_ift_step(&step)` registration model. Fix is a pseudocode-body rewrite, not a parameter swap. Class: pseudocode / platform-API mismatch."
- Does NOT add new validators to `RewardBreakdown` — the four γ-locked fields are unchanged.
- Does NOT change ForwardMap's `theta: &Tensor<f64>` parameter (per Decision D).
- Does NOT touch `replay_step` — its `theta: &Tensor<f64>` matches shipped code.
- Does NOT touch any soft-body source code. Phase 2 starts that.

## 8. Sequencing within the PR

1. Branch off main `a0e18beb` as `feature/phase-1-bf7-bf8-preemptive`.
2. **Commit 1: BF-7 paragraph in `docs/studies/soft_body_architecture/src/100-optimization/00-forward.md`.** Push, run `mdbook build` to verify link resolution + no warnings.
3. **Commit 2: BF-8 signature swap + rationale paragraph in `docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md`.** Push, re-run `mdbook build`.
4. **Commit 3: log BF-9 entry in `sim/docs/todo/soft_body_walking_skeleton_scope.md §13`** (per Decision E) — one new table row.
5. CI runs grade + tests-debug; doc-only PR should pass cleanly. No `cargo test` change since no Rust change.
6. Pre-commit risk-mitigation re-read per R-6.
7. Squash to one commit at merge (skip pre-squash tag per Decision C).

## 9. Open decisions

All Phase 1 decisions are locked in §3 (Decisions A–G). No open items at the scope-memo level. Live items below the scope-memo bar (e.g., exact prose phrasing of inserted paragraphs) are resolved at commit time via R-6 risk-mitigation re-read.
