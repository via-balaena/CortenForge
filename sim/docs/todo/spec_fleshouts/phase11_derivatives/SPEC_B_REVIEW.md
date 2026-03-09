# Sensor Derivatives (C, D Matrices) — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_B.md`
**Implementation session(s):** Session 11
**Reviewer:** AI agent
**Date:** 2026-03-09

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
| Sensor derivative computation | `mjd_transitionFD()` computes C/D via `mjd_stepFD()` when C/D pointers non-NULL | C/D always `None` — not computed | | |
| Sensor opt-in mechanism | Nullable C/D pointer -> `skipsensor` flag in `mj_stepSkip()` | No opt-in mechanism. `step()` always evaluates sensors. | | |
| FD sensor recording | `getState()` copies `d->sensordata` after each perturbed step | Not implemented — `extract_state()` captures only state, not sensors | | |
| Sensor skip during FD (no C/D) | `skipsensor=1` -> sensors skipped -> faster FD | Not implemented — `step()` always evaluates sensors regardless of whether C/D are needed | | |
| Control-clamped D columns | `clampedDiff` for sensor columns (same clamping as state columns) | State columns fixed (Session 9). Sensor columns not implemented. | | |
| `nsensordata == 0` | Allocates 0-length buffers, no-op sensor differencing. C/D are 0xndx and 0xnu. | C/D = None. | | |
| `nhistory != 0` | `mjd_stepFD` errors: "delays are not supported" | Guard already added (Session 9). | | |
| Sensor noise | Not applied during pipeline. FD derivatives are noiseless. | Same — no noise in pipeline. | | |
| Sensor cutoff | Applied via `mj_sensor_postprocess`. Creates zero-derivative regions. | Same — cutoff applied in pipeline. | | |

**Unclosed gaps:**

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it.

| Grade | Meaning |
|-------|---------|
| **Pass** | Implementation matches spec. Algorithm, file location, edge cases all correct. |
| **Weak** | Implementation works but deviates from spec, uses shortcuts, has loose tolerances, missing edge-case guards, or TODOs. **Fix before shipping.** |
| **Deviated** | Implementation intentionally diverged from spec (spec gap discovered during implementation). Deviation is documented and justified. Acceptable if the spec was updated. |
| **Missing** | Section not implemented. Must be either fixed or explicitly deferred with tracking. |

### S1. DerivativeConfig extension

**Grade:**

**Spec says:**
Add `compute_sensor_derivatives: bool` field (default `false`) to
`DerivativeConfig`. Update all 13 struct-literal callers to add
`compute_sensor_derivatives: false`. Sites using `..Default::default()` are
unaffected.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. FD sensor derivative loop in `mjd_transition_fd`

**Grade:**

**Spec says:**
Extend the existing `mjd_transition_fd` perturbation loop to capture
`sensordata` deltas alongside state deltas. Piggyback sensor recording on
every existing `step()` call — capture `scratch.sensordata.clone()` after
each perturbed step. Clamped control differencing for D columns follows the
same `in_ctrl_range` pattern as B columns. No new `step()` calls added.
When `compute_sensors` is false, zero overhead.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. FD sensor derivative loop in `mjd_transition_hybrid`

**Grade:**

**Spec says:**
Extend the hybrid path to compute FD sensor C/D columns. Three categories:
(1) FD columns piggyback sensor recording, (2) analytical velocity columns
need sensor-only FD with `skipstage=MjStage::Pos`, (3) analytical B columns
need sensor-only FD with `skipstage=MjStage::Vel`. Sensor-only FD passes use
`forward_skip(skipstage, false) + integrate()`. No A/B numerical coupling —
enabling sensor derivatives does NOT change A/B values.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. TransitionMatrices wiring

**Grade:**

**Spec says:**
Update `TransitionMatrices` docstrings to reflect that C/D are now populated
when `compute_sensor_derivatives` is true. `None` means "not requested,"
`Some` means "computed."

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. `mjd_transition` dispatch and `nsensordata == 0` handling

**Grade:**

**Spec says:**
When `compute_sensor_derivatives` is true but `nsensordata == 0`, return
`C: Some(DMatrix::zeros(0, nx))`, `D: Some(DMatrix::zeros(0, nu))`.
Preserves Some/None API semantics: `Some` = computation requested and
performed, `None` = not requested.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | C matrix dimensions (nsensordata x (2*nv + na)) | T1 | | |
| AC2 | D matrix dimensions (nsensordata x nu) | T1 | | |
| AC3 | Opt-in negative case (C/D = None with default config) | T2 | | |
| AC4 | FD C matrix accuracy vs independent FD (< 1e-6 rel) | T3 | | |
| AC5 | FD D matrix accuracy vs independent FD (< 1e-6 rel) | T3 | | |
| AC6 | Hybrid C/D matches pure FD C/D (< 1e-6 rel) | T4 | | |
| AC7 | A/B unchanged when C/D enabled (bitwise equal) | T5 | | |
| AC8 | `nsensordata == 0` returns empty Some matrices | T6 | | |
| AC9 | Multi-sensor-type model (jointpos, jointvel, actuatorfrc) | T7 | | |
| AC10 | Control-limited actuator D column (backward-only FD) | T8 | | |
| AC11 | Backward compatibility — existing callers compile | — (code review) | | |
| AC12 | No regression in existing tests | T9 | | |
| AC13 | Structural C/D-to-A/B relationship for jointpos sensors | T11 | | |
| AC14 | Cutoff-clamped sensor produces zero derivatives | T12 | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Sensor derivative dimensions (C shape = nsensordata x nx, D shape = nsensordata x nu) | | | |
| T2 | Opt-in negative case (C/D = None with default config) | | | |
| T3 | C/D accuracy via independent FD validation (< 1e-6 rel tolerance) | | | |
| T4 | Hybrid C/D matches FD C/D (< 1e-6 rel tolerance) | | | |
| T5 | A/B unchanged when sensors enabled (bitwise equality) | | | |
| T6 | `nsensordata == 0` edge case (0-row matrices) | | | |
| T7 | Multi-sensor-type model (jointpos + jointvel + actuatorfrc) | | | |
| T8 | Control-limited actuator D column (backward-only differencing at range boundary) | | | |
| T9 | Regression — existing derivative tests pass | | | |
| T11 | Structural C/D-to-A/B cross-check for jointpos sensors | | | |
| T12 | Cutoff-clamped sensor zero derivatives | | | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T10 | Forward-difference sensor derivatives (centered: false code path) | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| `nsensordata == 0` | No sensors -> C/D should be empty `Some`, not `None`. MuJoCo returns non-null 0-row matrices. | | | |
| `nu == 0` (no controls) | D matrix should have 0 columns. No control loop runs. | | | |
| `na == 0` (no activations) | Activation block of C is absent (nx = 2*nv, not 2*nv+na). | | | |
| Control-limited actuator at range boundary | Forward-only or backward-only FD for D column. `clampedDiff` behavior. | | | |
| Cutoff-clamped sensor | FD captures zero derivative in clamped region. MuJoCo behavior. | | | |
| Centered vs forward differences | Both differencing modes must produce correct C/D. | | | |
| Analytical A columns + sensor FD | Hybrid path: analytical position columns need separate sensor FD passes. | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `DerivativeConfig` struct layout: 3 fields -> 4 fields (+ compute_sensor_derivatives) | | |
| `TransitionMatrices.C/D`: always `None` -> `Some(matrix)` when compute_sensor_derivatives true | | |
| `mjd_transition_fd` sensor evaluation: step() always evaluates sensors -> now also captures sensordata when requested | | |
| `mjd_transition_hybrid` FD cost: additional FD steps for sensor-only columns when compute_sensor_derivatives true | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/derivatives.rs` (~200 new + ~30 modified) | | |
| `sim/L0/tests/integration/derivatives.rs` (~250 new + ~7 modified) | | |
| `sim/L0/tests/integration/implicit_integration.rs` (~4 modified) | | |

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Non-Modification Sites: Predicted vs Actual

The spec predicted these files would NOT be modified. Verify they were untouched.

| File:line | Spec Prediction (why NOT modified) | Actually Unchanged? | Notes |
|-----------|-----------------------------------|---------------------|-------|
| `forward/mod.rs:309` | `forward_skip(skipstage, skipsensor)` already supports `skipsensor` parameter — no changes needed | | |
| `lib.rs:265-270` | Exports `DerivativeConfig`, `TransitionMatrices` — already exported, no new exports needed | | |
| `derivatives.rs:451` | `extract_state()` — extracts state only, sensor extraction is independent | | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_fd_produces_correct_dimensions` | Pass (unchanged) — uses `Default::default()` | | |
| `test_transition_matrices_c_d_none` | Pass (unchanged) — asserts `C.is_none()` and `D.is_none()` with default config | | |
| `test_centered_vs_forward` | Compile fix needed — struct literal without `compute_sensor_derivatives` | | |
| `test_identity_at_origin` | Compile fix needed — struct literal without `compute_sensor_derivatives` | | |
| `test_b_matrix_nonzero` | Compile fix needed — struct literal without `compute_sensor_derivatives` | | |
| `test_contact_derivative_nonzero` | Compile fix needed — struct literal without `compute_sensor_derivatives` | | |
| `test_eps_convergence` | Compile fix needed — 2 struct literals without `compute_sensor_derivatives` | | |
| `fd_convergence_check` | Compile fix needed — 2 struct literals without `compute_sensor_derivatives` | | |
| `test_implicit_*` (4 tests) | Compile fix needed — 4 struct literals without `compute_sensor_derivatives` | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| C/D opt-in: nullable output pointers -> `DerivativeConfig.compute_sensor_derivatives: bool` | Use `config.compute_sensor_derivatives` wherever MuJoCo checks pointer nullability | | |
| `mj_stepSkip()` -> `forward_skip + integrate` for sensor-only FD passes; `step()` for piggybacked FD columns | For sensor-only FD passes (S3): use `scratch.forward_skip(model, skipstage, false)?; scratch.integrate(model)`. For piggybacked FD columns (S2, S3): keep `scratch.step(model)?`. | | |
| Matrix layout: nalgebra column-major, `column_mut(i).copy_from(&col)` writes columns directly | Direct port — nalgebra column writes match `mjd_stepFD`'s column layout. No transpose step needed. | | |
| `getState()` sensor capture: `mju_copy(sensor, d->sensordata, m->nsensordata)` -> `scratch.sensordata.clone()` | Use `scratch.sensordata.clone()` wherever MuJoCo uses `getState(,,,sensor)` | | |
| `extract_state()`: returns tangent-space state directly. Sensor extraction is independent (flat f64 copy). | Direct port — sensor extraction is independent of state extraction. | | |
| Sensor dimensions: `ns = m->nsensordata` -> `model.nsensordata` | Direct port — no translation needed | | |
| `skipsensor` polarity: `true` means SKIP sensors | Direct port — same polarity. Pass `!config.compute_sensor_derivatives` for sensor control. | | |

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
| Analytical sensor derivatives (per-sensor-type analytical Jacobians) | Out of Scope, bullet 1 | | | |
| Inverse dynamics sensor derivatives (`DsDq`, `DsDv`, `DsDa` in `mjd_inverseFD`) | Out of Scope, bullet 2 | | | |
| Parallel FD computation (DT-49) | Out of Scope, bullet 3 | | | |
| `step()` -> `forward_skip + integrate` migration for existing FD loops (rubric EGT-8) | Out of Scope, bullet 4 | | | |

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
sim-core:              N passed, 0 failed, M ignored
sim-mjcf:              N passed, 0 failed, M ignored
sim-conformance-tests: N passed, 0 failed, M ignored
Total:                 N passed, 0 failed, M ignored
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
