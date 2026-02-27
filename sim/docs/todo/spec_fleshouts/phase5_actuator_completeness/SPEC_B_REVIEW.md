# Spec B — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_B.md`
**Implementation session(s):** Session 7
**Reviewer:** AI agent
**Date:** 2026-02-27

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
| `mjTRN_SLIDERCRANK` transmission | Full implementation: length, derivatives, Jacobian composition, moment via chain rule | **Not implemented** — no enum variant, no parsing, no runtime | | |
| `mjTRN_JOINTINPARENT` for hinge/slide | Identical to `mjTRN_JOINT` — scalar `gear[0]` | **Not implemented** — no enum variant, no parsing | | |
| `mjTRN_JOINTINPARENT` for ball/free | Gear rotation via inverse quaternion | **Not implemented** — also pre-existing gap in `mjTRN_JOINT` (`nv == 1` guard skips ball/free) | | |
| `mj_jacPointAxis` helper | `engine_core_util.c` — `cross(jacr_col, axis)` per DOF | **Not implemented** — `jacobian.rs` has `mj_jac` and `mj_jac_site` only | | |
| Slider-crank `det <= 0` singularity | Silent degradation: `length = av`, `dlda = vec`, `dldv = axis` | N/A | | |
| Slider-crank gear scaling | `length *= gear[0]` after derivatives; `moment *= gear[0]` during storage | N/A | | |
| `cranklength > 0` validation | Compiler error (`mjCError`) | N/A | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it. This is the core of the review.

**How to grade each section:**

- Read the spec section (algorithm, file path, MuJoCo equivalent, design
  decision, before/after code).
- Read the actual implementation.
- Compare. Grade honestly.

| Grade | Meaning |
|-------|---------|
| **Pass** | Implementation matches spec. Algorithm, file location, edge cases all correct. |
| **Weak** | Implementation works but deviates from spec, uses shortcuts, has loose tolerances, missing edge-case guards, or TODOs. **Fix before shipping.** |
| **Deviated** | Implementation intentionally diverged from spec (spec gap discovered during implementation). Deviation is documented and justified. Acceptable if the spec was updated. |
| **Missing** | Section not implemented. Must be either fixed or explicitly deferred with tracking. |

### S1. Enum variants — `enums.rs`

**Grade:**

**Spec says:**
Add two new variants `SliderCrank` and `JointInParent` to `ActuatorTransmission`
enum, appended after `Body`. Matching MuJoCo's `mjTRN_SLIDERCRANK` and
`mjTRN_JOINTINPARENT`. `#[derive(Default)]` remains on `Joint`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Model field + init — `model.rs`, `model_init.rs`

**Grade:**

**Spec says:**
Add `actuator_cranklength: Vec<f64>` field to `Model`. Initialize to `vec![]`
in `Model::empty()`. SliderCrank actuators store the rod length; all others
store `0.0`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. `mj_jac_point_axis` helper — `jacobian.rs`

**Grade:**

**Spec says:**
New function `mj_jac_point_axis(model, data, body_id, point, axis)` returning
`(DMatrix<f64>, DMatrix<f64>)` — both 3×nv. Calls `mj_jac()` internally, then
computes axis Jacobian column-by-column via `cross(jacr_col, axis)`. Cross
product order: `cross(jacr_col, axis)`, NOT `cross(axis, jacr_col)`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. MJCF parsing — `types.rs`, `parser.rs`, `builder/actuator.rs`

**Grade:**

**Spec says:**
Four new attributes: `jointinparent` (site name), `cranksite` (site name),
`slidersite` (site name), `cranklength` (f64). `jointinparent` maps to
`JointInParent` with same `trnid` semantics as `joint`. `cranksite` +
`slidersite` map to `SliderCrank` with `trnid[0]` = cranksite, `trnid[1]` =
slidersite. Builder validates `cranklength > 0` and `slidersite` present when
`cranksite` specified.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. SliderCrank transmission dispatch — `actuation.rs`

**Grade:**

**Spec says:**
New standalone function `mj_transmission_slidercrank()` following the existing
`mj_transmission_site()` / `mj_transmission_body()` pattern. Called from
`forward_core()`. Computes: slider axis (z-column of slider site xmat), vec
(crank - slider), length = `av - sqrt(det)` with degenerate fallback,
derivatives `dlda`/`dldv`, Jacobians via `mj_jac_point_axis` (slider) and
`mj_jac_site` (crank), moment via chain rule, gear scaling on both length
and moment.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. JointInParent + SliderCrank pipeline arms — `actuation.rs`

**Grade:**

**Spec says:**
Widen existing `Joint` match arms to `Joint | JointInParent` for both
`mj_actuator_length` (velocity stage) and `mj_fwd_actuation` Phase 3 (force
stage). Add `SliderCrank` alongside `Site` arms — velocity from cached moment,
force via cached moment vector.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Match arm updates — 8 exhaustive match sites

**Grade:**

**Spec says:**
8 production match sites across 7 files need explicit arms for `SliderCrank`
and `JointInParent`:
- S7a: `muscle.rs` ~160 — `compute_actuator_params` Phase 1 lengthrange
- S7b: `muscle.rs` ~344 — `build_actuator_moment`
- S7c: `muscle.rs` ~509 — `mj_set_length_range` Step 3 uselimit (**CRITICAL:** replace `_ => {}` catch-all)
- S7d: `derivatives.rs` ~563 — `mjd_actuator_vel`
- S7e: `sensor/position.rs` ~282 — `mj_sensor_pos` ActuatorPos
- S7f: `builder/build.rs` ~565 — sleep policy resolution
- Plus 2 arms in `actuation.rs` (covered by S6)

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | SliderCrank length — colinear geometry | T1 | | |
| AC2 | SliderCrank moment — offset geometry | T1, T2 | | |
| AC3 | SliderCrank velocity from cached moment | T2 | | |
| AC4 | SliderCrank force application | T3 | | |
| AC5 | SliderCrank degenerate case (det ≤ 0) | T4 | | |
| AC6 | JointInParent hinge == Joint identity | T5 | | |
| AC7 | JointInParent slide == Joint identity | T6 | | |
| AC8 | MJCF parsing — `jointinparent` round-trip | T7 | | |
| AC9 | MJCF parsing — SliderCrank round-trip | T8 | | |
| AC10 | MJCF error — cranksite without slidersite | T9 | | |
| AC11 | MJCF error — non-positive cranklength | T10 | | |
| AC12 | Exhaustive match completeness | — (code review) | | |
| AC13 | `mj_jac_point_axis` correctness | T11 | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | SliderCrank length + moment — colinear geometry (AC1, AC2) | | | |
| T2 | SliderCrank moment + velocity — offset geometry (AC2, AC3) | | | |
| T3 | SliderCrank end-to-end force application (AC4) | | | |
| T4 | SliderCrank degenerate (det ≤ 0) — no panic (AC5) | | | |
| T5 | JointInParent hinge identity (AC6) | | | |
| T6 | JointInParent slide identity (AC7) | | | |
| T7 | MJCF parsing — jointinparent (AC8) | | | |
| T8 | MJCF parsing — SliderCrank (AC9) | | | |
| T9 | MJCF error — cranksite without slidersite (AC10) | | | |
| T10 | MJCF error — non-positive cranklength (AC11) | | | |
| T11 | `mj_jac_point_axis` correctness (AC13) | | | |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| SliderCrank `det ≤ 0` (crank orthogonal to slider axis) | Singularity: sqrt of negative number would panic without guard | | | |
| JointInParent hinge == Joint identity | Behavioral equivalence must hold exactly | | | |
| JointInParent slide == Joint identity | Same for slide joints | | | |
| `cranklength = 0` (non-positive) | MuJoCo compiler rejects this; we must too | | | |
| Missing `slidersite` with `cranksite` present | MuJoCo requires both; partial spec is invalid | | | |
| SliderCrank with gear != 1 | Gear scaling affects length and moment differently | | | |

### Supplementary Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T2 (velocity) | Velocity = moment.dot(qvel) with no gear factor — verifies gear-baked-into-moment convention | | | |
| T11 (jac_point_axis) | Standalone helper correctness — new function not covered by any AC except AC13 | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| 2 new `ActuatorTransmission` variants (Joint, Tendon, Site, Body → + SliderCrank, JointInParent) | | |
| New `mj_jac_point_axis` function in `jacobian.rs` | | |
| New `actuator_cranklength` field in Model | | |
| `_ => {}` catch-all at muscle.rs:509 replaced with explicit arms | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `core/src/types/enums.rs` | | |
| `core/src/types/model.rs` | | |
| `core/src/types/model_init.rs` | | |
| `core/src/jacobian.rs` | | |
| `core/src/forward/actuation.rs` | | |
| `core/src/forward/muscle.rs` | | |
| `core/src/derivatives.rs` | | |
| `core/src/sensor/position.rs` | | |
| `mjcf/src/types.rs` | | |
| `mjcf/src/parser.rs` | | |
| `mjcf/src/builder/actuator.rs` | | |
| `mjcf/src/builder/build.rs` | | |
| `tests/integration/transmission_specb.rs` | | |

{Unexpected files (to be filled during review execution):}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| Phase 4 regression suite (39 tests) | Pass (unchanged) — extension only | | |
| Spec A tests (T7-T12) | Pass (unchanged) — new variants added but existing behavior untouched | | |
| `test_site_actuator` | Pass (unchanged) — site parsing path not modified | | |
| `test_dampratio_mjcf_roundtrip` | Pass (unchanged) — dampratio for Joint unaffected | | |
| `test_acc0_motor_via_mjcf` | Pass (unchanged) — motor acc0 unaffected | | |
| Full sim domain baseline (~2,148+ tests) | All pass — extension-only change | | |

**Unexpected regressions:**
{To be filled during review execution.}

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `actuator_trnid` indexing: MuJoCo flat `[2*i]`/`[2*i+1]` → CortenForge `Vec<[usize; 2]>` `[i][0]`/`[i][1]` | Use `model.actuator_trnid[i][0]` wherever MuJoCo uses `m->actuator_trnid[2*i]` | | |
| `actuator_trnid` slot semantics (SliderCrank): `trnid[0]` = cranksite, `trnid[1]` = slidersite | Direct port — same semantic assignment | | |
| `actuator_gear`: MuJoCo flat `gear + 6*i` → CortenForge `Vec<[f64; 6]>` `[i][0]` | Use `model.actuator_gear[i][0]` wherever MuJoCo uses `m->actuator_gear[6*i]` | | |
| `site_xmat` z-column: MuJoCo row-major indices `[2]`, `[5]`, `[8]` → CortenForge `nalgebra::Matrix3` column access `.column(2)` | Use `data.site_xmat[id].column(2)` to extract z-axis | | |
| `actuator_moment`: MuJoCo sparse CSR → CortenForge dense `Vec<DVector<f64>>` | Store directly into dense vector; no sparse compression needed | | |
| Jacobian layout: MuJoCo row-major `3*nv` → CortenForge `nalgebra::DMatrix` (column-major) `matrix[(row, col)]` | Use `matrix[(k, j)]` wherever MuJoCo uses `jac[k*nv + j]` | | |
| Quaternion convention: `(w, x, y, z)` in both | Direct port — no translation needed | | |
| Function naming: C camelCase → Rust snake_case | Spec uses C names for MuJoCo reference, Rust names for implementation | | |

---

## 7. Weak Implementation Inventory

Items that technically work but aren't solid. These should be fixed now —
"weak" items left unfixed tend to become permanent technical debt.

**What counts as weak:**

- `TODO` / `FIXME` / `HACK` comments in new code
- Hardcoded values that should come from the spec or MuJoCo
- Loose tolerances (e.g., 1e-3 where MuJoCo conformance demands 1e-10)
- Missing edge-case guards the spec calls for
- Placeholder error handling (e.g., `unwrap()` in non-test code, empty
  `Err(_) => {}` that swallows information)
- Functionality that "passes tests" but uses a different algorithm than
  the spec prescribes
- Dead code or commented-out code from debugging

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution.}

**Severity guide:**
- **High** — Conformance risk. MuJoCo would produce different results.
  Fix before shipping.
- **Medium** — Code quality issue. Correct behavior but fragile, unclear,
  or not matching spec's prescribed approach. Fix if time permits, else
  track.
- **Low** — Style or minor robustness. No conformance risk. Track if not
  fixing now.

---

## 8. Deferred Work Tracker

Every item that was in the spec's scope but not fully implemented, plus
anything discovered during implementation that's out of scope. **The goal:
nothing deferred is untracked.**

For each item, verify it appears in at least one of:
- `sim/docs/todo/` (future_work file or spec_fleshout)
- `sim/docs/ROADMAP_V1.md`
- The umbrella spec's Out of Scope section (if applicable)

If it's not tracked anywhere, it's a review finding — add tracking now.

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Ball/free joint transmission (both Joint and JointInParent) — `nv == 3` and `nv == 6` sub-paths in `mj_transmission()` | Out of Scope, bullet 1 | | | |
| `mjTRN_TENDON` and `mjTRN_SITE` improvements | Out of Scope, bullet 2 | | | |
| Sparse moment compression (CSR) | Out of Scope, bullet 3 | | | |

### Discovered During Implementation

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

Quick sanity check on test health after the implementation.

**Domain test results:**
```
{To be filled during review execution.}
```

**New tests added:** {count}
**Tests modified:** {count}
**Pre-existing test regressions:** {count — should be 0}

**Clippy:** {clean / N warnings}
**Fmt:** {clean / issues}

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

**Items to fix before shipping:**
1. {To be filled during review execution.}

**Items tracked for future work:**
1. {To be filled during review execution.}
