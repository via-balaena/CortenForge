# Spec A — Constraint Assembly Conformance — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_A.md`
**Implementation session(s):** Session 2
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

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| tendon_invweight0 computation | `J · M⁻¹ · J^T` via `setInertia()` in `engine_setconst.c` | Diagonal-only `Σ(coef² · M⁻¹[i,i])` for fixed; body_invweight0 average for spatial | | |
| diagApprox for tendon constraints | Returns precomputed `tendon_invweight0[t]` | Same — reads stored value (correct dispatch, wrong stored value) | | |
| DT-23: per-DOF friction solref/solimp | `dof_solref_friction[dof]` per-DOF | `model.dof_solref[dof_idx]` per-DOF (correct) | | |
| DT-23: per-tendon friction solref/solimp | `tendon_solref_friction[t]` per-tendon | `model.tendon_solref_fri[t]` per-tendon (correct) | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

### S1. Fix `tendon_invweight0` — full M⁻¹ solve

**Grade:**

**Spec says:**
Replace the diagonal-only / heuristic tendon_invweight0 computation in
`model_init.rs:1056–1115` with the full `J · M⁻¹ · J^T` solve using
`data.ten_J[t]` and `mj_solve_sparse_batch`. Handles both fixed and spatial
tendons uniformly (no `match tendon_type`). Remove unused `TendonType`/`WrapType`
import at line 903. Clamp result to `MIN_VAL` (1e-15).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Update docstring

**Grade:**

**Spec says:**
Update `compute_invweight0()` docstring at `model_init.rs:898` to describe
the full `J · M⁻¹ · J^T` formula. Also update `diagapprox_bodyweight.rs:6`
module docstring and line 233 test comment to reflect the full formula.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. DT-23 verification (no code changes)

**Grade:**

**Spec says:**
Code review only. Verify assembly code at `assembly.rs:369-371` reads per-DOF
params and `assembly.rs:396-398` reads per-tendon params. Verify builder
fan-out at `builder/joint.rs:160-163` and `builder/tendon.rs:43-48`. Verify
defaults cascade at `defaults.rs:209-213` and `defaults.rs:641-645`. Confirm
existing tests T7–T11 cover routing. No code changes expected.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Flag model tendon_invweight0 ≈ 5388.92 ± 1e-2 | T1 | | |
| AC2 | Single-joint tendon: invweight0 = dof_invweight0 ± 1e-12 | T2 | | |
| AC3 | Multi-joint tendon: invweight0 > Σ(coef² · dof_invweight0) | T3 | | |
| AC4 | All 26 golden flag tests pass at TOLERANCE = 1e-8 | T4 | | |
| AC5 | Docstring describes full J·M⁻¹·J^T formula | — (code review) | | |
| AC6 | DT-23 routing: per-DOF and per-tendon params correctly routed | — (code review) + existing T7–T11 | | |
| AC7 | Spatial tendon: invweight0 > MIN_VAL, no panics/regressions | T5 | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Flag model tendon_invweight0 conformance (≈ 5388.92) | | | |
| T2 | Single-joint tendon — diagonal equivalence | | | |
| T3 | Two-joint tendon — off-diagonal coupling (strictly greater than diagonal) | | | |
| T4 | Golden flag tests — all 26 pass at TOLERANCE = 1e-8 (`#[ignore]` removed) | | | |
| T5 | Spatial tendon invweight0 — regression guard (> MIN_VAL, no panics) | | | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| Existing T7–T11 | DT-23 routing (multi-DOF, defaults, end-to-end) — AC6 verification | `assembly.rs:1064-1496` | Pre-existing, not new |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Single-joint tendon | Full form must collapse to diagonal for 1 DOF | | | T2 |
| Multi-joint serial chain | Off-diagonal M⁻¹ coupling is the bug | | | T3 |
| Multi-joint independent bodies | Full form must equal diagonal when M is block-diagonal (no coupling) | | | Existing `tendon_invweight0_fixed_tendon` in `diagapprox_bodyweight.rs` |
| Zero-coef tendon wraps | J_tendon all zeros → MIN_VAL result | | | T2 variant (if all coefs zero) |
| Ball/free joint wraps | ten_J[dof_adr] = coef at first DOF component; multi-DOF subblock coupling captured by M⁻¹ solve | | | No dedicated test (covered implicitly) |
| nv=0 model | Early return in `compute_invweight0` — tendon_invweight0 stays zero | | | Existing guard at line 918-922 |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `tendon_invweight0` for multi-joint fixed tendons: diagonal-only → full `J · M⁻¹ · J^T` (toward MuJoCo) | | |
| `tendon_invweight0` for spatial tendons: avg `body_invweight0` → full `J · M⁻¹ · J^T` (toward MuJoCo) | | |
| Single-joint tendon invweight0: unchanged (mathematically identical) | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/types/model_init.rs` (replace tendon_invweight0 computation + docstring + remove unused import) | | |
| `sim/L0/tests/integration/phase13_diagnostic.rs` (add/modify invweight0 assertion) | | |
| `sim/L0/tests/integration/golden_flags.rs` (remove `#[ignore]` from all 26 tests) | | |
| `sim/L0/tests/integration/diagapprox_bodyweight.rs` (update docstring + comment) | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `phase13_diagnostic` | Values change (become correct) | | |
| `tendon_invweight0_fixed_tendon` in `diagapprox_bodyweight.rs:209` | Pass (unchanged — block-diagonal M → full = diagonal) | | |
| `golden_baseline` | Pass (was failing at 0.0021) | | |
| `golden_disable_equality` | Pass (was failing at 0.0069) | | |
| `golden_disable_filterparent` | Pass (was failing at 0.020) | | |
| `golden_disable_constraint` | Pass (already passing, `#[ignore]` removed) | | |
| `golden_disable_frictionloss` | Pass (already passing, `#[ignore]` removed) | | |
| 21 other golden flag tests | Pass (were failing at ~0.002) | | |
| All 79 conformance tests | Pass (unchanged — no tendon friction rows) | | |
| T7–T11 (DT-23 routing) | Pass (unchanged — routing code not modified) | | |

**Unexpected regressions:**
{To be filled during review execution.}

### Non-Modification Sites

The spec identifies 4 consumer sites that should NOT have been modified.
Verify each was untouched.

| File:line | What it does | Why NOT modified | Untouched? |
|-----------|-------------|-----------------|------------|
| `assembly.rs:395` | Reads `model.tendon_invweight0.get(t)` | Consumer — reads corrected value, no logic change | |
| `assembly.rs:535` | Reads `model.tendon_invweight0.get(t)` | Consumer — reads corrected value, no logic change | |
| `impedance.rs:366-368` | Returns `model.tendon_invweight0[id]` | Consumer — dispatch already correct | |
| `builder/build.rs:514` | Calls `model.compute_invweight0()` | Caller — already calls the right function | |

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `ten_J` layout: MuJoCo `d->ten_J + i*nv` (contiguous C array) | Access `data.ten_J[t]` directly — no index arithmetic needed | | |
| `mj_solveM`: MuJoCo `mj_solveM(m, d, res, res, 1)` (in-place, single RHS) | Wrap single DVector in 1-column DMatrix for batch solver | | |
| `mjMINVAL` = 1e-15 | Direct port — same value as `MIN_VAL` | | |
| `mju_dot`: MuJoCo `mju_dot(J, res, nv)` | Use DVector::dot or manual loop over nv elements | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution. Check for: TODO/FIXME/HACK comments,
hardcoded values, loose tolerances, missing edge-case guards, placeholder
error handling, algorithm deviations from spec, dead code.}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| DT-19: QCQP cone projection verification | Out of Scope, bullet 1 | SESSION_PLAN.md Spec B (Sessions 5–8) | DT-19 | |
| DT-128: PGS early termination | Out of Scope, bullet 2 | SESSION_PLAN.md Spec B (Sessions 5–8) | DT-128 | |
| DT-129: PGS warmstart projection | Out of Scope, bullet 3 | SESSION_PLAN.md Spec B (Sessions 5–8) | DT-129 | |
| Pyramidal diagApprox factor-of-2 (CF=4.0, MJ=8.0 for contact rows) | Out of Scope, bullet 4 | Informational note only — no conformance bug | — | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

{To be filled during review execution.}

---

## 9. Test Coverage Summary

**Domain test results:**
```
{To be filled during review execution.}
sim-core:              N passed, 0 failed, M ignored
sim-mjcf:              N passed, 0 failed, M ignored
sim-conformance-tests: N passed, 0 failed, M ignored
Total:                 N passed, 0 failed, M ignored
```

**New tests added:** {count}
**Tests modified:** {count}
**Pre-existing test regressions:** {count — should be 0}

**Clippy:** {To be filled during review execution}
**Fmt:** {To be filled during review execution}

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
{To be filled during review execution.}

**Items to fix before shipping:**
{To be filled during review execution.}

**Items tracked for future work:**
{To be filled during review execution.}
