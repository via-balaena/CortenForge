# Spec A — Defaults Completeness: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_A.md`
**Implementation session(s):** Session 5
**Reviewer:** AI agent
**Date:** 2026-03-02

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
| Equality defaults cascade | `<default><equality solref="...">` applies to all 5 types via `OneEquality()` with `readingdefaults=true` | Not implemented — 5 hardcoded fallback sites, no defaults struct, class field ignored | | |
| `active` in equality defaults | Parsed as `bool` via `MapValue()+bool_map`, inheritable via class | `active: bool` (not `Option<bool>`) — cannot distinguish "not set" from "default true" | | |
| `qpos_spring` for hinge/slide | `m->qpos_spring[padr] = springref` | Uses `jnt_springref[jnt_id]` — numerically equivalent for hinge/slide but different index space | | |
| `qpos_spring` for ball | `[1,0,0,0]` copied from `qpos0` | Not implemented — no `qpos_spring` array exists | | |
| `qpos_spring` for free | `pos[3]+quat[4]` copied from `qpos0` | Not implemented | | |
| Tendon sentinel resolution | Evaluates at `qpos_spring` configuration | Evaluates at `qpos0` — divergent when `springref != 0` | | |
| `implicit_springref` init | Reads from `qpos_spring` per-DOF | Reads from `jnt_springref` per-joint | | |
| Actuator shortcut defaults | `cylinder|muscle|adhesion|damper|intvelocity` dispatched to `OneActuator()` | Missing from `parse_default()` match — silently dropped | | |
| Type-specific default fields | Stored on single `mjsActuator` per class | `MjcfActuatorDefaults` lacks `area`, `diameter`, `bias`, `timeconst`, muscle params, `gain` | | |

**Unclosed gaps:**

---

## 2. Spec Section Compliance

### S1. DT-2 — Equality Defaults: Struct and Parse

**Grade:**

**Spec says:**
Create `MjcfEqualityDefaults` struct with `active: Option<bool>`, `solref: Option<[f64; 2]>`,
`solimp: Option<[f64; 5]>`. Add `equality: Option<MjcfEqualityDefaults>` to `MjcfDefault`.
Change `active` from `bool` to `Option<bool>` on all 5 equality structs. Parse `<equality>`
in `parse_default()`. Create `parse_equality_defaults()`. Update 5 equality `active` parsers
from `!= "false"` to `== "true"`. Export `MjcfEqualityDefaults` in `lib.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. DT-2 — Equality Defaults: Merge and Cascade

**Grade:**

**Spec says:**
Add `merge_equality_defaults()` (parent/child merge with `c.field.or(p.field)`). Add `equality`
to `merge_defaults()`. Add `equality_defaults()` accessor. Add `apply_equality_defaults()` taking
`(&mut Option<bool>, &mut Option<[f64;2]>, &mut Option<[f64;5]>)` — generic across all 5 types.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. DT-2 — Equality Defaults: Builder Integration

**Grade:**

**Spec says:**
Pass resolver into `process_equality_constraints()`. At each of 5 match arms, extract
`active/solref/solimp` into mutable locals, call `apply_equality_defaults()`, then use
cascaded values. `unwrap_or(DEFAULT_SOLREF/SOLIMP)` fallbacks remain for no-class+no-attr case.
Update `active` from `bool` push to `.unwrap_or(true)` push.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. DT-13 — `qpos_spring`: Model Array

**Grade:**

**Spec says:**
Add `qpos_spring: Vec<f64>` to Model (sized `nq`, same as `qpos0`). Init as empty vec in
`model_init.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. DT-13 — `qpos_spring`: Builder Population

**Grade:**

**Spec says:**
Add `qpos_spring_values: Vec<f64>` accumulator to `ModelBuilder`. Populate during joint building:
hinge/slide push `springref` scalar, ball pushes `[1,0,0,0]` from qpos0, free pushes 7D from qpos0.
Populate for flex vertex slide joints (`push(0.0)`). Convert to Model in `build.rs`. Update tendon
sentinel to read `qpos_spring` instead of `qpos0`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. DT-13 — `qpos_spring`: Consumer Migration

**Grade:**

**Spec says:**
Update `passive.rs` spring force from `jnt_springref[jnt_id]` to `qpos_spring[qpos_adr]`.
Update `energy.rs` spring energy likewise. Update `model_init.rs` `implicit_springref` to read
`qpos_spring[jnt_qpos_adr[jnt_id]]`. Ball/free spring force/energy remain as stubs (Spec B scope).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. DT-13 — `qpos_spring`: Test Model Factory Updates

**Grade:**

**Spec says:**
~18 sites that push `jnt_springref` must also push `qpos_spring`. Hinge/slide: same scalar.
Free: 7D `[pos+identity_quat]` or matching `qpos0`. Affects `model_factories.rs` (3 sites),
`sensor/mod.rs` (3 sites), `jacobian.rs` (7 sites), `constraint/jacobian.rs` (2 sites),
`forward/muscle.rs` (6 sites).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. DT-14 — Actuator Defaults: Parser Extension

**Grade:**

**Spec says:**
Extend `parse_default()` match arms to include `b"cylinder"|b"muscle"|b"adhesion"|b"damper"|b"intvelocity"`.
Add 14 type-specific fields to `MjcfActuatorDefaults` (area, diameter, timeconst, bias,
muscle_timeconst, range, force, scale, lmin, lmax, vmax, fpmax, fvmax, gain). Parse them in
`parse_actuator_defaults()`. Handle `timeconst` ambiguity (1 value → cylinder, 2 → muscle).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S9. DT-14 — Actuator Defaults: Merge and Cascade

**Grade:**

**Spec says:**
Extend `merge_actuator_defaults()` with 14 `c.field.or(p.field)` lines. Extend
`apply_to_actuator()` with sentinel-based detection for non-Option fields (area default 1.0,
bias default [0,0,0], etc.) and Option-based cascade for Option fields. Known limitation:
sentinel detection is imperfect when user explicitly sets field to its default value.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Equality defaults — solref cascade | | | |
| AC2 | Equality defaults — solimp cascade | | | |
| AC3 | Equality defaults — active cascade | | | |
| AC4 | Equality defaults — no class → hardcoded default | | | |
| AC5 | `qpos_spring` — hinge joint with springref | | | |
| AC6 | `qpos_spring` — ball joint | | | |
| AC7 | `qpos_spring` — free joint | | | |
| AC8 | `qpos_spring` — hinge/slide spring force regression | | | |
| AC9 | Actuator defaults — cylinder area cascade | | | |
| AC10 | Actuator defaults — muscle params cascade | | | |
| AC11 | Tendon sentinel resolution uses `qpos_spring` | | | |
| AC12 | No regression (full domain suite) | | | |
| AC13 | Adhesion defaults — gain cascade | | | |
| AC14 | Damper defaults — kv cascade | | | |
| AC15 | Ball `qpos_spring` copies from `qpos0`, not hardcoded (code review) | — | | |
| AC16 | `MjcfEqualityDefaults` exported (code review) | — | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

> **Test numbering note:** Spec uses T1–T15. Implementation test function
> names will be mapped during review execution (Session 7).

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Equality defaults — solref/solimp cascade (AC1, AC2) | | | |
| T2 | Equality defaults — active=false cascade (AC3) | | | |
| T3 | Equality defaults — no class → MuJoCo built-in (AC4) | | | |
| T4 | `qpos_spring` — hinge with explicit springref (AC5) | | | |
| T5 | `qpos_spring` — ball joint identity quaternion (AC6) | | | |
| T6 | `qpos_spring` — free joint body pose (AC7) | | | |
| T7 | Spring force regression — hinge/slide unchanged (AC8) | | | |
| T8 | Cylinder area defaults cascade (AC9) | | | |
| T9 | Muscle range defaults cascade (AC10) | | | |
| T10 | Tendon sentinel uses `qpos_spring` (AC11) | | | |
| T11 | Equality defaults — nested class inheritance (supplementary) | | | |
| T12 | Multiple actuator shortcuts in one default (supplementary) | | | |
| T13 | Mixed joint types — `qpos_spring` alignment (supplementary) | | | |
| T14 | Adhesion gain defaults cascade (AC13) | | | |
| T15 | Damper kv defaults cascade (AC14) | | | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| | | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Equality constraint with NO class, no explicit solref | Must get hardcoded default `[0.02, 1.0]` | | | |
| Nested class inheritance for equality | Child overrides solimp, inherits parent solref | | | |
| `qpos_spring` for ball (identity quaternion from qpos0) | Ball ignores `springref`; must copy qpos0 | | | |
| `qpos_spring` for free with non-identity quat | Free copies full 7D pose from qpos0 (non-identity quat proves actual copy) | | | |
| Hinge springref=0 → `qpos_spring[adr]==0` same as `qpos0[adr]` | Regression: values numerically identical to before | | | |
| Multiple actuator shortcuts in one `<default>` | Last wins (struct replacement at parse level) | | | |
| Tendon sentinel with springref!=0 | Only test that proves qpos0→qpos_spring matters | | | |
| `<default><equality>` does NOT cascade data/anchor/polycoef | These are type-specific, not in defaults context | | | |
| `active` defaults cascade (Option<bool> migration) | Previous `bool` type prevented cascade | | | |
| Mixed joint types qpos_spring alignment | Different nq per joint type must align | | | |
| Element-level override beats class default | `<connect solref="0.1 0.5" class="X">` should use 0.1/0.5 not X's default | | | |
| `<default><cylinder area="0.01"/>` then `<cylinder class="...">` | Cylinder-specific defaults cascade | | | |
| `<default><adhesion gain="0.5"/>` then `<adhesion class="...">` | Adhesion-specific defaults cascade | | | |
| `<default><damper kv="5.0"/>` then `<damper class="...">` | Damper-specific defaults cascade; expansion maps kv→gainprm[2]=-kv | | | |
| Ball `qpos_spring` copies from `qpos0` structurally | Prevents hardcoded identity quat diverging from actual qpos0 in future | | | |
| Muscle sentinel detection (`gainprm[0]==1` → 0.75 overwrite) | MuJoCo quirk only manifests for `<general dyntype="muscle">` path; documented as known divergence — see Out of Scope | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Equality constraints inherit from default classes (hardcoded fallback → class cascade then fallback) | | |
| `active` type on equality structs changes from `bool` to `Option<bool>` | | |
| Spring force reads `qpos_spring` instead of `jnt_springref` | | |
| Tendon sentinel resolution uses `qpos_spring` instead of `qpos0` | | |
| `implicit_springref` source changes from `jnt_springref[jnt_id]` to `qpos_spring[qpos_adr]` | | |
| Actuator defaults parse `cylinder|muscle|adhesion|damper|intvelocity` (previously silently dropped) | | |
| Actuator type-specific defaults cascade via `apply_to_actuator()` (new code path) | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/parser.rs` | | |
| `sim/L0/mjcf/src/defaults.rs` | | |
| `sim/L0/mjcf/src/builder/equality.rs` | | |
| `sim/L0/mjcf/src/builder/mod.rs` | | |
| `sim/L0/mjcf/src/builder/joint.rs` | | |
| `sim/L0/mjcf/src/builder/flex.rs` | | |
| `sim/L0/mjcf/src/builder/build.rs` | | |
| `sim/L0/mjcf/src/builder/init.rs` | | |
| `sim/L0/mjcf/src/lib.rs` | | |
| `sim/L0/core/src/types/model.rs` | | |
| `sim/L0/core/src/types/model_init.rs` | | |
| `sim/L0/core/src/forward/passive.rs` | | |
| `sim/L0/core/src/energy.rs` | | |
| `sim/L0/core/src/types/model_factories.rs` | | |
| Various test files (~15 sites) | | |
| Test file(s) for Spec A (~200 lines) | | |

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_equality_connect` | Pass — values unchanged (no class defaults in test models) | | |
| `test_equality_weld` | Pass — values unchanged | | |
| Spring force tests | Pass — values unchanged (hinge/slide: `qpos_spring == jnt_springref` when both 0.0) | | |
| Energy tests | Pass — values unchanged | | |
| Tendon tests | Pass — values unchanged (test models use springref=0) | | |
| Model factory tests | Compile error → fix (must add `qpos_spring` field) | | |
| Jacobian tests | Compile error → fix (must add `qpos_spring` to test models) | | |
| Muscle tests | Compile error → fix (must add `qpos_spring` to test models) | | |
| Sensor tests | Compile error → fix (must add `qpos_spring` to test models) | | |
| Constraint tests | Compile error → fix (must add `qpos_spring` to test models) | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `qpos_spring` index space: `model.qpos_spring[qpos_adr]` where `qpos_adr = jnt_qpos_adr[jnt_id]` | Direct port — same index space. Use `jnt_qpos_adr` (not `jnt_dof_adr`) | | |
| `jnt_springref` vs `qpos_spring`: keep `jnt_springref` as builder intermediate; runtime reads `qpos_spring` | Runtime code reads `qpos_spring`; `jnt_springref` stays for builder use only | | |
| `solref` size: `[f64; 2]` | Direct port | | |
| `solimp` size: `[f64; 5]` | Direct port | | |
| Equality default struct: single `MjcfEqualityDefaults` per class, applies to all 5 types | Direct port | | |
| Actuator default struct: single `MjcfActuatorDefaults` per class, all shortcuts write to same struct | Direct port | | |
| Shortcut expansion timing: deferred to build time | Deferred expansion produces identical results because same mapping logic | | |
| `springref` default: `0.0` (from memset zero; NOT NaN or sentinel) | Direct port — `0.0` is the default when no `springref` attribute specified | | |
| `bool_map`: only `"true"`/`"false"` — standardize to `== "true"` when touching equality active sites | Standardize to `== "true"` | | |

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
| DT-11 (Joint `range` default) — already implemented | Out of Scope, bullet 1 | | | |
| Ball/free spring force computation (`mji_subQuat` path) | Out of Scope, bullet 2 | | | |
| Ball/free spring energy computation | Out of Scope, bullet 3 | | | |
| `IntVelocity` enum variant (actuator type completeness) | Out of Scope, bullet 4 | | | |
| Muscle sentinel detection for `<general>` path (`gainprm[0]==1`) | Out of Scope, bullet 5 | | | |
| Migration of `MjcfActuator` type-specific fields from non-Option to Option | Out of Scope, bullet 6 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

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
