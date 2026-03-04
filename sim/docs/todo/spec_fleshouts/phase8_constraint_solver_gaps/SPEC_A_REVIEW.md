# Spec A — Solver Param & Margin Completeness — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_A.md`
**Implementation session(s):** Session 5
**Reviewer:** AI agent
**Date:** 2026-03-04

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
| Tendon limit activation check | `dist < margin` where `margin = m->tendon_margin[i]` (`mj_instantiateLimit()`) | Hardcoded `dist < 0.0` at 4 sites in assembly.rs:150,154,562,586 | | |
| Tendon limit `finalize_row!` margin arg | `margin` passed to `mj_addConstraint()` → `efc_margin[row]` | Hardcoded `0.0` at assembly.rs:573,597 | | |
| `tendon_margin` model field | `tendon_margin[ntendon]` in `mjModel` (`mjmodel.h`), default `0.0` | **Missing** — field does not exist in Model | | |
| Counting / instantiation agreement | Identical `dist < margin` in both `mj_nl()` and `mj_instantiateLimit()` | Both use `< 0.0` (consistent but wrong) | | |
| DOF friction solref direct read | `m->dof_solref[mjNREF*id]` in `getsolparam()` | `model.dof_solref[dof_idx]` at assembly.rs:391 — **conformant** | | |
| Multi-DOF solreffriction fan-out | CopyTree loops `for j1 in 0..nv`, copies identical values | Builder loops `for i in 0..nv`, pushes identical values — **conformant** (untested) | | |
| Tendon friction solref read | `m->tendon_solref_friction[mjNREF*id]` in `getsolparam()` | `model.tendon_solref_fri[t]` at assembly.rs:418 — **conformant** (untested) | | |
| Default solref/solimp values | `[0.02, 1.0]` / `[0.9, 0.95, 0.001, 0.5, 2.0]` via `mj_defaultSolRefImp()` | `DEFAULT_SOLREF` / `DEFAULT_SOLIMP` constants — **conformant** | | |

**Unclosed gaps:**

---

## 2. Spec Section Compliance

### S1. Add `tendon_margin` model field (DT-33)

**Grade:**

**Spec says:**
Add `tendon_margin: Vec<f64>` to `Model` struct in `model.rs` (after
`tendon_solimp_lim`), with doc comment citing MuJoCo ref. Add initialization
`tendon_margin: vec![]` in `model_init.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Wire `tendon_margin` through builder pipeline (DT-33)

**Grade:**

**Spec says:**
Add `tendon_margin: Vec<f64>` field to `ModelBuilder` in `builder/mod.rs`,
initialize in `builder/init.rs`, push `tendon.margin.unwrap_or(0.0)` in
`builder/tendon.rs`, transfer `tendon_margin: self.tendon_margin` in
`builder/build.rs`. 4-file pattern matching `jnt_margin`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Fix assembly.rs tendon limit activation and margin wiring (DT-33)

**Grade:**

**Spec says:**
Read `model.tendon_margin[t]` once per tendon. Replace 4 hardcoded `< 0.0`
with `< margin` (counting lower/upper at lines 150/154, instantiation
lower/upper at lines 562/586). Replace 2 hardcoded `0.0` margin args with
`margin` in `finalize_row!` calls (lines 573/597). Total: 6 modification sites.
Counting and instantiation must use identical conditions.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. DT-23 verification and test coverage

**Grade:**

**Spec says:**
Verification-only — no production code changes. Add tests exercising: multi-DOF
fan-out for ball (3 DOFs, T7) and free (6 DOFs, T8), defaults cascade (T9),
non-default solreffriction in DOF friction constraint rows (T10), and tendon
friction solref end-to-end (T11). If verification reveals a bug, update spec
before fixing.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Tendon margin=0 regression — `nefc >= 1`, `efc_margin[0] == 0.0`, `efc_pos[0] == -0.5` | T1 | | |
| AC2 | Tendon margin>0 pre-activation — `nefc >= 1`, `efc_margin[0] == 0.1`, `efc_pos[0] ≈ 0.05` | T2 | | |
| AC3 | Tendon margin in `efc_margin` field — `efc_margin[0] == 0.2` (not `0.0`) | T2, T3 | | |
| AC4 | Counting and instantiation phases agree with margin>0 — no panic, correct row count | T2, T5 | | |
| AC5 | DISABLE_LIMIT ignores tendon margin — `nefc == 0` | T4 | | |
| AC6 | Negative margin shrinks activation zone — Case A: `nefc == 0`, Case B: `nefc >= 1` | T5 | | |
| AC7 | Large margin overlap — both limits active — `nefc >= 2` | T6 | | |
| AC8 | Ball joint 3 DOFs identical dof_solref — all 3 equal `[0.05, 0.8]` | T7 | | |
| AC9 | Free joint 6 DOFs identical dof_solref — all 6 equal `[0.05, 0.8]` | T8 | | |
| AC10 | Defaults cascade for solreffriction — `dof_solref[0] == [0.03, 0.7]` | T9 | | |
| AC11 | Non-default solreffriction in constraint rows — `efc_solref[friction_row] == [0.05, 0.8]` | T10 | | |
| AC12 | Tendon friction solref end-to-end — `efc_solref[tendon_friction_row] == [0.04, 0.9]` | T11 | | |
| AC13 | Pipeline completeness (code review) — all 6 locations wired | — (code review) | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

> **Test numbering note:** If the spec's test labels (T1, T2, ...) differ
> from the implementation's test function names (e.g., `t01_*`, `t02_*`),
> note the mapping convention here so readers can cross-reference.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Tendon margin=0 regression (AC1) — model with limited tendon, margin=0, violated lower limit → `nefc >= 1`, `efc_margin == 0`, `efc_pos == -0.5` | | | |
| T2 | Tendon margin>0 pre-activation, lower limit (AC2, AC3, AC4) — margin=0.1, `ten_length=-0.95`, lower dist=0.05 < 0.1 → pre-activated | | | |
| T3 | Tendon margin>0 pre-activation, upper limit (AC3) — margin=0.1, `ten_length=0.95`, upper dist=0.05 < 0.1 → pre-activated, Jacobian is `-ten_J` | | | |
| T4 | DISABLE_LIMIT ignores tendon margin (AC5) — margin=0.1, within margin zone, DISABLE_LIMIT set → `nefc == 0` | | | |
| T5 | Negative margin shrinks activation zone (AC6) — margin=-0.1, Case A: dist=-0.05 (no row), Case B: dist=-0.15 (row) | | | |
| T6 | Large margin overlap — both limits active (AC7) — range=(-0.5, 0.5), margin=0.6, length=0.0 → `nefc >= 2` | | | |
| T7 | Ball joint 3 DOFs identical dof_solref (AC8) — ball joint, solreffriction="0.05 0.8", all 3 DOFs equal | | | |
| T8 | Free joint 6 DOFs identical dof_solref (AC9) — free joint, solreffriction="0.05 0.8", all 6 DOFs equal | | | |
| T9 | Defaults cascade for solreffriction (AC10) — default class overrides, joint inherits `[0.03, 0.7]` | | | |
| T10 | Non-default solreffriction in constraint rows (AC11) — hinge joint, frictionloss=1.0, solreffriction="0.05 0.8" → `efc_solref == [0.05, 0.8]` | | | |
| T11 | Tendon friction solref end-to-end (AC12) — tendon frictionloss=1.0, solreffriction="0.04 0.9" → `efc_solref == [0.04, 0.9]` | | | |

### Supplementary Tests

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T12 | Multi-tendon indexing — 3 tendons with different margins, verify correct margin applied to each. Catches off-by-one indexing bugs in `model.tendon_margin[t]`. | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| `margin = 0.0` (default) | Regression: must produce identical behavior to pre-change hardcoded `< 0.0` | | | |
| `margin > 0` (pre-activation) | Core feature: constraint fires before limit violation | | | |
| `margin < 0` (dead zone) | Valid in MuJoCo: shrinks activation zone, ignores small violations | | | |
| Large margin overlap | Both limits fire simultaneously when margin > half range width | | | |
| `DISABLE_LIMIT` with margin | Limit flag overrides margin — no rows regardless of margin value | | | |
| `tendon_limited = false` with margin | Inactive tendon ignores margin | | | |
| Ball joint (3 DOFs) solreffriction | Builder fan-out: all 3 DOFs must get identical values | | | |
| Free joint (6 DOFs) solreffriction | Builder fan-out: all 6 DOFs must get identical values | | | |
| Defaults cascade override | Class-level `solreffriction` overrides global default | | | |
| `frictionloss = 0.0` | No friction row emitted regardless of `solreffriction` value | | | |
| World body tendon attachment | Tendon Jacobian has zeros for world DOFs — margin activation still correct with zero Jacobian rows | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Tendon limit activation with margin>0: constraint activates when `dist < margin` instead of `dist < 0.0` | | |
| Tendon limit `efc_margin` values: actual `tendon_margin[t]` instead of always `0.0` | | |
| DT-23 (no behavioral change) — already conformant | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `core/src/types/model.rs` — Add `tendon_margin: Vec<f64>` (+4 lines) | | |
| `core/src/types/model_init.rs` — Add initialization (+1 line) | | |
| `mjcf/src/builder/mod.rs` — Add field to ModelBuilder (+1 line) | | |
| `mjcf/src/builder/init.rs` — Add initialization (+1 line) | | |
| `mjcf/src/builder/tendon.rs` — Add push (+2 lines) | | |
| `mjcf/src/builder/build.rs` — Add transfer (+1 line) | | |
| `core/src/constraint/assembly.rs` — Replace 6 hardcoded sites (~10 modified) | | |
| `core/src/constraint/assembly.rs` (tests) — New tests T1–T12 (+200–300 lines) | | |

{Unexpected files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Non-Modification Sites

The spec explicitly listed files that should NOT be modified. Verify none were
touched during implementation.

| File:line | What it does | Why NOT modified | Actually untouched? | Notes |
|-----------|-------------|-----------------|---------------------|-------|
| assembly.rs:109 | `let margin = model.jnt_margin[jnt_id]` (joint limit counting) | Already uses `jnt_margin` — correct, different subsystem | | |
| assembly.rs:130 | `let margin = model.jnt_margin[jnt_id]` (ball joint counting) | Already uses `jnt_margin` — correct | | |
| assembly.rs:391–392 | `model.dof_solref[dof_idx]`, `model.dof_solimp[dof_idx]` (DOF friction) | Already correct — DT-23 is verification-only | | |
| assembly.rs:418–419 | `model.tendon_solref_fri[t]`, `model.tendon_solimp_fri[t]` (tendon friction) | Already correct — DT-23 is verification-only | | |
| impedance.rs:48–107 | `compute_impedance()` — uses `(pos - margin).abs()` | Already correct — margin flows through `finalize_row!` automatically | | |
| impedance.rs:196–202 | `compute_aref()` — uses `K * imp * (pos - margin)` | Already correct — no changes needed | | |
| impedance.rs:144–192 | `compute_kbip()` — independent of margin | Not affected | | |

### Data Staleness Guards

Spec notes: No `EXPECTED_SIZE` constants or static assertions in affected code
paths. Adding `tendon_margin` to Model has zero serialization impact (no serde
derives). Only compile-time check: Rust's exhaustive struct initialization
catches missing fields in `build.rs` or `model_init.rs`.

**Verified?**

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `t8_margin_zero_regression` (assembly.rs:904) | Pass (unchanged) — tests joint margin, not tendon | | |
| `t9_margin_pre_activation` (assembly.rs:931) | Pass (unchanged) — tests joint margin | | |
| `t10_ball_margin` (assembly.rs:955) | Pass (unchanged) — tests ball joint limit margin | | |
| `t11_disable_limit_ignores_margin` (assembly.rs:1045) | Pass (unchanged) — tests joint DISABLE_LIMIT | | |
| Friction loss integration tests (unified_solvers.rs:80,83,95,108,147) | Pass (unchanged) — default solref values, no tendon margin | | |
| Tendon length/velocity tests (sim-tendon) | Pass (unchanged) — test tendon computation, not constraint assembly | | |
| Phase 7 parsing/defaults tests (sim-mjcf) | Pass (unchanged) — Phase 8 does not modify parsing or defaults | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `tendon_margin` — `Vec<f64>` indexed by tendon id | Direct port — identical semantics to MuJoCo `tendon_margin[ntendon]` flat array | | |
| `tendon_range` — `(f64, f64)` tuple, `.0` = min, `.1` = max | Use `model.tendon_range[t].0` for `tendon_range[2*i]` (lower), `.1` for upper | | |
| `dof_solref` — `[f64; 2]` indexed by DOF id | Direct port — `model.dof_solref[dof_idx]` for `m->dof_solref + mjNREF*id` | | |
| CopyTree fan-out — `for i in 0..nv` pushes identical solref to each DOF | Direct port — no translation needed | | |
| Side convention — lower: `dist = length - limit_min`; upper: `dist = limit_max - length` | Equivalent to MuJoCo's `side * (range - value)` — both produce identical dist values | | |
| Jacobian sign — lower: `+ten_J`; upper: `-ten_J` | Direct port — matches MuJoCo's `-side` formula | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| `solreffriction` on contacts (per-direction friction solver params on `<geom>` or `<pair>`) | Out of Scope, bullet 1 | | | |
| Mixed-sign solref validation (`(solref[0] > 0) ^ (solref[1] > 0)` → MuJoCo warns and replaces with default) | Out of Scope, bullet 2 (rubric gap log R11) | | | |
| Tendon limit `solref_limit`/`solimp_limit` naming — already done in DT-32 | Out of Scope, bullet 3 | | | |
| `efc_impP` impedance derivative field (DT-22) — API introspection only | Out of Scope, bullet 4 | | | |
| Condim=4/6 for deformable contacts — DT-25 scope | Out of Scope, bullet 5 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|

---

## 9. Test Coverage Summary

**Domain test results:**
```
{to be filled during review execution}
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
