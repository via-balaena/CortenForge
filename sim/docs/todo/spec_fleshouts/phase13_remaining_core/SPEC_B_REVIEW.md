# Spec B — PGS Solver Conformance — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_B.md`
**Implementation session(s):** Session 6
**Reviewer:** AI agent
**Date:** 2026-03-11

---

## Purpose

This review verifies that the implementation matches the approved spec,
surfaces weakly implemented items that should be fixed now, and ensures
deferred work is properly tracked so nothing falls through the cracks.

**Five questions this review answers:**

1. **Closed?** Are the conformance gaps from the spec's Key Behaviors
   table actually closed?
2. **Faithful?** Does the implementation match the spec — every section,
   every AC, every convention note, every planned test?
3. **Predicted?** Did the blast radius match the spec's predictions, or
   were there surprises?
4. **Solid?** Are there any items that technically "work" but are weakly
   implemented (hacks, TODOs, incomplete edge cases, loose tolerances)?
5. **Tracked?** Is every piece of deferred or out-of-scope work tracked
   in `sim/docs/todo/` or `sim/docs/ROADMAP_V1.md`?

---

## 1. Key Behaviors Gap Closure

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| PGS convergence check | `improvement * scale < tolerance` → break | None — always runs `max_iters` | | |
| `solver_niter` | Actual iteration count (may be < max_iters) | Always `max_iters` | | |
| `solver_stat` for PGS | Populated per iteration (improvement, nactive, nchange) | Empty (`solver_stat.clear()`) | | |
| PGS improvement tracking | `improvement = -Σ costChange(...)` accumulated per sweep | Cost guard exists but improvement not tracked | | |
| PGS scale factor | `1 / (meaninertia * max(1, nv))` | Not computed | | |
| `costChange` return value | Returns cost change; caller accumulates | Cost change computed but not returned (inline) | | |
| QCQP cone projection | `mju_QCQP2/3/N` in `engine_util_solve.c` | `qcqp2/3/qcqp_nd` in `qcqp.rs` — **matches exactly** | | |
| PGS warmstart gate | Zero forces when cost > 0 | Zero forces when dual_cost ≥ 0 — **equivalent** | | |

**Unclosed gaps:**

---

## 2. Spec Section Compliance

### S1. DT-19 QCQP verification (no code changes)

**Grade:**

**Spec says:**
Verification only. Line-by-line comparison of CortenForge's `qcqp2/3/qcqp_nd`
against MuJoCo's `mju_QCQP2/3/N()` in `engine_util_solve.c`. No code changes
needed. Mark DT-19 as verified. Run existing 14 QCQP tests to confirm.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. PGS early termination (DT-128)

**Grade:**

**Spec says:**
Refactor the `for _iter in 0..max_iters` loop into a `while iter < max_iters`
loop with improvement accumulation and convergence break. Capture cost_change
from both elliptic and scalar branches and accumulate
`improvement -= cost_change`. Compute `scale = 1.0 / (data.stat_meaninertia *
max(1.0, nv as f64))`. Break when `improvement < tolerance` after incrementing
iter. Report `data.solver_niter = iter` (actual count). Populate
`data.solver_stat` per iteration.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. PGS warmstart verification (DT-129)

**Grade:**

**Spec says:**
Verify existing warmstart logic matches MuJoCo's PGS path.
`classify_constraint_states()` maps qacc_warmstart → efc_force (equivalent to
MuJoCo's `mj_constraintUpdate()`). Dual cost `< 0.0` → use warmstart forces
(equivalent to MuJoCo's `cost > 0 → zero forces`). No code changes expected.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Solver statistics population

**Grade:**

**Spec says:**
Populate `data.solver_stat` with per-iteration `SolverStat` entries. PGS uses
`gradient = 0`, `lineslope = 0`, `nline = 0`. `improvement` populated from S2's
accumulation. `nactive` and `nchange` are diagnostic-only placeholders (0, 0).
Integrated into S2's implementation.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | PGS early termination triggers — `solver_niter` matches MuJoCo, < max_iters | T1 | | |
| AC2 | PGS produces correct forces — efc_force within solver_tolerance with/without early termination | T2 | | |
| AC3 | solver_stat populated — len == solver_niter, improvement >= 0, gradient/lineslope/nline == 0 | T3 | | |
| AC4 | PGS max_iters honored — solver_tolerance = 0 → runs exactly max_iters | T4 | | |
| AC5 | QCQP unchanged — qcqp.rs not modified, 14 existing tests pass | — (code review + existing tests) | | |
| AC6 | Newton solver unaffected — qacc identical within 1e-15 before/after Spec B | T5 | | |
| AC7 | Warmstart conformance — classify at qacc_warmstart, cost comparison, zero when not beneficial | — (code review) | | |
| AC8 | Golden flag checkpoint — record pass/fail for all 26 golden flag tests | T6 | | |
| AC9 | PGS MuJoCo conformance — efc_force within 1e-10, solver_niter exact, qacc within 1e-10 | T7 | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

> **Test numbering note:** Spec uses T1–T7. Implementation test function
> names will be mapped here during review execution.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | PGS early termination — solver_niter < max_iters for simple equality constraint | | | |
| T2 | PGS force correctness — efc_force matches between early-termination and max_iters runs | | | |
| T3 | PGS solver_stat — len == solver_niter, improvement >= 0, gradient/lineslope == 0, last entry improvement < solver_tolerance | | | |
| T4 | PGS max_iters edge case — tolerance = 0, iterations = 5 → solver_niter = 5 | | | |
| T5 | Newton regression — flag_golden_test.xml qacc unchanged within 1e-15 | | | |
| T6 | Golden flag checkpoint — run all 26 golden tests with --ignored | | | |
| T7 | PGS MuJoCo conformance — efc_force, solver_niter, qacc match MuJoCo reference within 1e-10 | | | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list, plus pre-existing
tests the spec identifies as regression guards.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| Existing 14 QCQP unit tests | QCQP conformance — verify no regression from Spec B changes | | AC5 |
| Existing 79 conformance tests | Full pipeline conformance (Newton) — verify no regressions | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| `nefc = 0` | PGS with no constraints should return immediately | | | Existing guard at pgs.rs:73-75 |
| `solver_tolerance = 0.0` | Disables early termination | | | T4 |
| `solver_iterations = 0` | Zero iterations — solver_niter should be 0 | | | Edge case variant of T4 |
| `solver_iterations = 1` | Single sweep — improvement computed once | | | Variant of T1 |
| Warmstart disabled | Cold start path; improvement from zero forces | | | Variant of T1 with disable_warmstart |
| All updates reverted | Every costChange returns 0 → improvement = 0 → converged | | | Implicitly tested by convergence |

**Missing tests:**

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| PGS iteration count: always `max_iters` → may terminate early when converged | | |
| `solver_niter` value: always `max_iters` → actual iteration count | | |
| `solver_stat` for PGS: empty → populated with per-iteration stats | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/constraint/solver/pgs.rs` (~40 modified, ~15 added) | | |
| `sim/L0/tests/integration/` (tests T1–T7, +120–180 lines) | | |
| `sim/L0/tests/assets/golden/pgs_conformance/` (MuJoCo reference .npy files) | | |
| `sim/L0/tests/scripts/` (Python script to dump PGS reference data) | | |

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Non-Modification Sites

Spec identifies 6 files/lines that must NOT be modified. Verify untouched.

| File:line | What it does | Verified Untouched? | Notes |
|-----------|-------------|---------------------|-------|
| `newton.rs:42-338` | Newton solver | | Independent solver |
| `cg.rs` | CG solver | | Independent solver |
| `qcqp.rs:25-256` | QCQP solvers | | Verified correct — DT-19 closed |
| `assembly.rs` | Constraint assembly | | Assembly-side — Spec A scope |
| `constraint/mod.rs:277-288` | Warmstart entry point | | Calls PGS warmstart correctly |
| `constraint/mod.rs:408-443` | Solver dispatch | | Dispatches to PGS correctly |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| 14 QCQP unit tests (`qcqp.rs`) | Pass (unchanged) | | |
| 79 conformance tests (`mujoco_conformance/*.rs`) | Pass (unchanged) | | |
| Golden flag tests (`golden_flags.rs`) | Unchanged (still `#[ignore]`) | | |
| Phase 13 diagnostic (`phase13_diagnostic.rs`) | Pass (unchanged) | | |
| Existing PGS-specific tests (if any) | May see different `solver_niter` | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `meaninertia` | Use `data.stat_meaninertia` (per-step), NOT `model.stat_meaninertia` (build-time) | | |
| `opt.tolerance` | Direct port — `model.solver_tolerance` (scalar, default 1e-8) | | |
| `opt.iterations` | Direct port — `model.solver_iterations` (usize, default 100) | | |
| `solver_niter` | Use `data.solver_niter = iter` (no island indexing) | | |
| `saveStats` | Map fields 1:1 to `SolverStat`; PGS uses 0 for gradient, lineslope, nline | | |
| `costChange` | Refactor to return value for accumulation | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| DT-39 (tendon_invweight0) — Spec A | Out of Scope, bullet 1 | | | |
| DT-23 (per-DOF friction params) — Spec A | Out of Scope, bullet 2 | | | |
| Newton solver convergence behavior (24 golden flags at ~0.002 qacc) | Out of Scope, bullet 3 | | | |
| PGS `nactive`/`nchange` counting (diagnostic only) | Out of Scope, bullet 4 | | | |
| §46 composite bodies — Spec C | Out of Scope, bullet 5 | | | |
| §66 plugin system — Spec D | Out of Scope, bullet 6 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| PGS `solver_stat` `nactive`/`nchange` per-iteration counting — placeholder 0 used | Session 6 — S2 implementation | | DT-162 | |
| PGS warmstart primal cost gate — CF uses dual cost, MuJoCo uses primal cost | Session 6 — S3 warmstart verification | | DT-163 | |

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

---

## 9. Test Coverage Summary

**Domain test results:**
```
sim-core:              — passed, — failed, — ignored
sim-mjcf:              — passed, — failed, — ignored
sim-conformance-tests: — passed, — failed, — ignored
Total:                 — passed, — failed, — ignored
```

**New tests added:**
**Tests modified:**
**Pre-existing test regressions:**

**Clippy:**
**Fmt:**

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | |
| Spec section compliance | 2 | |
| Acceptance criteria | 3 | |
| Test plan completeness | 4 | |
| Blast radius accuracy | 5 | |
| Convention fidelity | 6 | |
| Weak items | 7 | |
| Deferred work tracking | 8 | |
| Test health | 9 | |

**Overall:**

**Items fixed during review:**

**Items to fix before shipping:**

**Items tracked for future work:**
