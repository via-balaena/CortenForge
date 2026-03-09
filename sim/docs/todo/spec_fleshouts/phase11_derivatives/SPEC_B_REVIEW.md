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
| Sensor derivative computation | `mjd_transitionFD()` computes C/D via `mjd_stepFD()` when C/D pointers non-NULL | C/D always `None` — not computed | `mjd_transition_fd` and `mjd_transition_hybrid` compute C/D when `compute_sensor_derivatives: true`. Sensor recording piggybacks on existing FD steps. | **Yes** |
| Sensor opt-in mechanism | Nullable C/D pointer -> `skipsensor` flag in `mj_stepSkip()` | No opt-in mechanism. `step()` always evaluates sensors. | `DerivativeConfig.compute_sensor_derivatives: bool` (default `false`). When false, C/D remain `None`, zero overhead. | **Yes** |
| FD sensor recording | `getState()` copies `d->sensordata` after each perturbed step | Not implemented — `extract_state()` captures only state, not sensors | `scratch.sensordata.clone()` captures sensor outputs after each perturbed step when `compute_sensors` is true. | **Yes** |
| Sensor skip during FD (no C/D) | `skipsensor=1` -> sensors skipped -> faster FD | Not implemented — `step()` always evaluates sensors regardless of whether C/D are needed | **Partial.** Hybrid path sensor-only FD passes use `forward_skip(skipstage, false)` (sensors evaluated). When `compute_sensors=false`, no sensordata clones — but `step()` still evaluates sensors. Full `skipsensor` optimization (skipping sensor eval entirely) deferred to Out of Scope. | **Partial** (conformance-correct; performance gap deferred) |
| Control-clamped D columns | `clampedDiff` for sensor columns (same clamping as state columns) | State columns fixed (Session 9). Sensor columns not implemented. | D columns follow identical `in_ctrl_range` / forward-backward-centered clamped differencing pattern as B columns. Four-way match: centered, forward-only, backward-only, both-blocked. | **Yes** |
| `nsensordata == 0` | Allocates 0-length buffers, no-op sensor differencing. C/D are 0×ndx and 0×nu. | C/D = None. | Returns `C: Some(DMatrix::zeros(0, nx))`, `D: Some(DMatrix::zeros(0, nu))` when `compute_sensor_derivatives: true` and `nsensordata == 0`. Matches MuJoCo's non-null 0-row behavior. | **Yes** |
| `nhistory != 0` | `mjd_stepFD` errors: "delays are not supported" | Guard already added (Session 9). | Unchanged — guard still present. | N/A (pre-existing) |
| Sensor noise | Not applied during pipeline. FD derivatives are noiseless. | Same — no noise in pipeline. | Unchanged — same behavior. | N/A (already conformant) |
| Sensor cutoff | Applied via `mj_sensor_postprocess`. Creates zero-derivative regions. | Same — cutoff applied in pipeline. | Unchanged — cutoff applied during `step()`. FD naturally captures zero derivatives in clamped region. Verified by T12. | N/A (already conformant) |

**Unclosed gaps:** The `skipsensor` performance optimization for the pure-FD
path (`mjd_transition_fd`) is deferred — `step()` always evaluates sensors
even when `compute_sensors=false`. This is a performance gap, not a
conformance gap. The numerical results are identical.

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

**Grade:** Pass

**Spec says:**
Add `compute_sensor_derivatives: bool` field (default `false`) to
`DerivativeConfig`. Update all 13 struct-literal callers to add
`compute_sensor_derivatives: false`. Sites using `..Default::default()` are
unaffected.

**Implementation does:**
Field added at `derivatives.rs:180` with full docstring matching spec
verbatim (MuJoCo equivalent, behavior when true/false). `Default` impl at
line 189 sets `false`. Struct-literal callers updated: 2 in
`fd_convergence_check` (lines 3491, 3497), 7 in `integration/derivatives.rs`
(lines 222, 228, 262, 353, 401, 500, 506), 4 in
`integration/implicit_integration.rs` (lines 1076, 1087, 1149, 1160). Total:
13 sites, all with `compute_sensor_derivatives: false`.

**Gaps (if any):** None.

**Action:** None.

### S2. FD sensor derivative loop in `mjd_transition_fd`

**Grade:** Pass

**Spec says:**
Extend the existing `mjd_transition_fd` perturbation loop to capture
`sensordata` deltas alongside state deltas. Piggyback sensor recording on
every existing `step()` call — capture `scratch.sensordata.clone()` after
each perturbed step. Clamped control differencing for D columns follows the
same `in_ctrl_range` pattern as B columns. No new `step()` calls added.
When `compute_sensors` is false, zero overhead.

**Implementation does:**
- `compute_sensors` gate at line 269: `config.compute_sensor_derivatives && ns > 0`
- Baseline sensor_0 captured at line 287 after nominal `step()`
- State perturbation loop: `s_plus` captured at line 332, `s_minus` at 356
  after existing `step()` calls. C columns filled via centered (lines 366-369)
  or forward (lines 375-378) differencing.
- Control perturbation loop: `s_plus`/`s_minus` captured after existing
  `step()` calls. D columns use four-way clamped differencing at lines 442-451
  matching B column pattern exactly.
- No new `step()` calls added — sensor recording piggybacks on existing FD steps.
- When `compute_sensors` is false: no `sensordata.clone()`, no C/D allocation.

**Gaps (if any):** None.

**Action:** None.

### S3. FD sensor derivative loop in `mjd_transition_hybrid`

**Grade:** Pass

**Spec says:**
Extend the hybrid path to compute FD sensor C/D columns. Three categories:
(1) FD columns piggyback sensor recording, (2) analytical velocity columns
need sensor-only FD with `skipstage=MjStage::Pos`, (3) analytical B columns
need sensor-only FD with `skipstage=MjStage::Vel`. Sensor-only FD passes use
`forward_skip(skipstage, false) + integrate()`. No A/B numerical coupling —
enabling sensor derivatives does NOT change A/B values.

**Implementation does:**
- `compute_sensors` gate at line 2733.
- Baseline sensor_0 captured at line 2744 after nominal `step()`.
- **Position columns (analytical):** Sensor-only FD at lines 2838-2884
  with `forward_skip(MjStage::None, false) + integrate()`.
- **Position columns (FD fallback):** Piggybacked sensor recording at
  lines 2886-2948 after existing `step()`.
- **Velocity columns:** Sensor-only FD at lines 2951-3008 with
  `forward_skip(MjStage::Pos, false) + integrate()`.
- **Analytical activation columns:** Sensor-only FD at lines 3010-3080
  with `forward_skip(MjStage::Vel, false) + integrate()`.
- **FD activation columns (muscle):** Piggybacked at lines 3082-3144.
- **Analytical B columns:** Sensor-only FD at lines 3214-3271 with
  `forward_skip(MjStage::Vel, false) + integrate()`.
- **FD B columns:** Piggybacked at lines 3273-3340.
- A/B values unchanged when sensors enabled (verified by T5 bitwise test).

**Gaps (if any):** None.

**Action:** None.

### S4. TransitionMatrices wiring

**Grade:** Pass

**Spec says:**
Update `TransitionMatrices` docstrings to reflect that C/D are now populated
when `compute_sensor_derivatives` is true. `None` means "not requested,"
`Some` means "computed."

**Implementation does:**
Docstrings updated at lines 117-128. C docstring states dimensions,
when `None` vs `Some`, and notes that `Some` applies even when
`nsensordata == 0`. D docstring follows same pattern. Matches spec verbatim.

**Gaps (if any):** None.

**Action:** None.

### S5. `mjd_transition` dispatch and `nsensordata == 0` handling

**Grade:** Pass

**Spec says:**
When `compute_sensor_derivatives` is true but `nsensordata == 0`, return
`C: Some(DMatrix::zeros(0, nx))`, `D: Some(DMatrix::zeros(0, nu))`.
Preserves Some/None API semantics: `Some` = computation requested and
performed, `None` = not requested.

**Implementation does:**
Both `mjd_transition_fd` (lines 454-464) and `mjd_transition_hybrid`
(lines 3342-3352) use identical pattern:
```rust
let (c_result, d_result) = if config.compute_sensor_derivatives {
    if ns > 0 {
        (C, D)  // populated matrices
    } else {
        (Some(DMatrix::zeros(0, nx)), Some(DMatrix::zeros(0, nu)))  // AD-2
    }
} else {
    (None, None)
};
```
Matches spec exactly. Verified by T6 test.

**Gaps (if any):** None.

**Action:** None.

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | C matrix dimensions (nsensordata x (2*nv + na)) | T1 | **Pass** | `t1_sensor_derivative_dimensions` at line 2063 asserts C shape (2, 4) |
| AC2 | D matrix dimensions (nsensordata x nu) | T1 | **Pass** | Same test asserts D shape (2, 1) |
| AC3 | Opt-in negative case (C/D = None with default config) | T2 | **Pass** | `t2_sensor_derivative_opt_in_negative` at line 2089. Also verified by pre-existing `test_a_matrix_dimensions` at lines 206-207 |
| AC4 | FD C matrix accuracy vs independent FD (< 1e-6 rel) | T3 | **Pass** | `t3_sensor_derivative_fd_accuracy` at line 2101 |
| AC5 | FD D matrix accuracy vs independent FD (< 1e-6 rel) | T3 | **Pass** | Same test validates D matrix |
| AC6 | Hybrid C/D matches pure FD C/D (< 1e-6 rel) | T4 | **Pass** | `t4_hybrid_matches_fd_sensor_derivatives` at line 2188 |
| AC7 | A/B unchanged when C/D enabled (bitwise equal) | T5 | **Pass** | `t5_ab_unchanged_with_sensors_enabled` at line 2220 |
| AC8 | `nsensordata == 0` returns empty Some matrices | T6 | **Pass** | `t6_nsensordata_zero_empty_matrices` at line 2252 |
| AC9 | Multi-sensor-type model (jointpos, jointvel, actuatorfrc) | T7 | **Pass** | `t7_multi_sensor_type_model` at line 2282 |
| AC10 | Control-limited actuator D column (backward-only FD) | T8 | **Pass** | `t8_control_limited_actuator_d_column` at line 2345 |
| AC11 | Backward compatibility — existing callers compile | — (code review) | **Pass** | All 13 struct-literal sites updated. All tests compile and pass. |
| AC12 | No regression in existing tests | T9 | **Pass** | Full domain suite: 2,674 passed, 0 failed. |
| AC13 | Structural C/D-to-A/B relationship for jointpos sensors | T11 | **Pass** | `t11_structural_cd_to_ab_crosscheck` at line 2424 |
| AC14 | Cutoff-clamped sensor produces zero derivatives | T12 | **Pass** | `t12_cutoff_clamped_sensor_zero_derivatives` at line 2474 |

**Missing or failing ACs:** None. All 14 ACs pass.

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Sensor derivative dimensions (C shape = nsensordata x nx, D shape = nsensordata x nu) | **Yes** | `t1_sensor_derivative_dimensions` (line 2063) | Asserts C=(2,4), D=(2,1) |
| T2 | Opt-in negative case (C/D = None with default config) | **Yes** | `t2_sensor_derivative_opt_in_negative` (line 2089) | Asserts C.is_none(), D.is_none() |
| T3 | C/D accuracy via independent FD validation (< 1e-6 rel tolerance) | **Yes** | `t3_sensor_derivative_fd_accuracy` (line 2101) | Independent per-column FD comparison |
| T4 | Hybrid C/D matches FD C/D (< 1e-6 rel tolerance) | **Yes** | `t4_hybrid_matches_fd_sensor_derivatives` (line 2188) | Compares fd vs hybrid C/D |
| T5 | A/B unchanged when sensors enabled (bitwise equality) | **Yes** | `t5_ab_unchanged_with_sensors_enabled` (line 2220) | Bitwise A/B equality |
| T6 | `nsensordata == 0` edge case (0-row matrices) | **Yes** | `t6_nsensordata_zero_empty_matrices` (line 2252) | Verifies Some(0×4), Some(0×0) |
| T7 | Multi-sensor-type model (jointpos + jointvel + actuatorfrc) | **Yes** | `t7_multi_sensor_type_model` (line 2282) | 3 sensor types, non-zero C/D |
| T8 | Control-limited actuator D column (backward-only differencing at range boundary) | **Yes** | `t8_control_limited_actuator_d_column` (line 2345) | ctrl at upper bound |
| T9 | Regression — existing derivative tests pass | **Yes** | Full test suite run | 2,674 passed, 0 failed |
| T11 | Structural C/D-to-A/B cross-check for jointpos sensors | **Yes** | `t11_structural_cd_to_ab_crosscheck` (line 2424) | C position block ≈ identity for jointpos |
| T12 | Cutoff-clamped sensor zero derivatives | **Yes** | `t12_cutoff_clamped_sensor_zero_derivatives` (line 2474) | Clamped sensor → zero C/D rows |

### Supplementary Tests

Tests that were added beyond the spec's planned test list.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T10 | Forward-difference sensor derivatives (centered: false code path) | `t10_forward_difference_sensor_derivatives` (line 2384) | Validates O(eps) accuracy non-centered path |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| `nsensordata == 0` | No sensors -> C/D should be empty `Some`, not `None`. MuJoCo returns non-null 0-row matrices. | **Yes** | `t6_nsensordata_zero_empty_matrices` | C=(0,4), D=(0,0) |
| `nu == 0` (no controls) | D matrix should have 0 columns. No control loop runs. | **Yes** | `t6_nsensordata_zero_empty_matrices` | D has 0 columns (nu=0 for pendulum) |
| `na == 0` (no activations) | Activation block of C is absent (nx = 2*nv, not 2*nv+na). | **Yes** | T1, T3 | All test models have na=0 |
| Control-limited actuator at range boundary | Forward-only or backward-only FD for D column. `clampedDiff` behavior. | **Yes** | `t8_control_limited_actuator_d_column` | ctrl at upper bound |
| Cutoff-clamped sensor | FD captures zero derivative in clamped region. MuJoCo behavior. | **Yes** | `t12_cutoff_clamped_sensor_zero_derivatives` | Zero C row for clamped sensor |
| Centered vs forward differences | Both differencing modes must produce correct C/D. | **Yes** | T3 (centered), T10 (forward) | Both code paths exercised |
| Analytical A columns + sensor FD | Hybrid path: analytical position columns need separate sensor FD passes. | **Yes** | `t4_hybrid_matches_fd_sensor_derivatives` | Hybrid ≡ FD within tolerance |

**Missing tests:** None. All planned tests and edge cases are covered.

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `DerivativeConfig` struct layout: 3 fields -> 4 fields (+ compute_sensor_derivatives) | **Yes** | Field added at line 180 with docstring |
| `TransitionMatrices.C/D`: always `None` -> `Some(matrix)` when compute_sensor_derivatives true | **Yes** | Both `mjd_transition_fd` and `mjd_transition_hybrid` populate C/D |
| `mjd_transition_fd` sensor evaluation: step() always evaluates sensors -> now also captures sensordata when requested | **Yes** | `sensordata.clone()` piggybacks on existing `step()` calls |
| `mjd_transition_hybrid` FD cost: additional FD steps for sensor-only columns when compute_sensor_derivatives true | **Yes** | Sensor-only FD passes added for analytical columns using `forward_skip + integrate` |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/derivatives.rs` (~200 new + ~30 modified) | **Yes** (482 insertions, 27 deletions) | None |
| `sim/L0/tests/integration/derivatives.rs` (~250 new + ~7 modified) | **Yes** (501 insertions) | None |
| `sim/L0/tests/integration/implicit_integration.rs` (~4 modified) | **Yes** (4 insertions) | None |

Line counts exceeded estimates (482 vs ~230, 501 vs ~257) — implementation
was more thorough than estimated. No unexpected files changed.

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| (none) | |

### Non-Modification Sites: Predicted vs Actual

The spec predicted these files would NOT be modified. Verify they were untouched.

| File:line | Spec Prediction (why NOT modified) | Actually Unchanged? | Notes |
|-----------|-----------------------------------|---------------------|-------|
| `forward/mod.rs:309` | `forward_skip(skipstage, skipsensor)` already supports `skipsensor` parameter — no changes needed | **Yes** | Last modified by Spec A (276bed5), not Spec B |
| `lib.rs:265-270` | Exports `DerivativeConfig`, `TransitionMatrices` — already exported, no new exports needed | **Yes** | Last modified by Spec A (276bed5), not Spec B |
| `derivatives.rs:451` | `extract_state()` — extracts state only, sensor extraction is independent | **Yes** | Function unchanged — verified by code review (lines 545-564) |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_a_matrix_dimensions` | Pass (unchanged) — uses `Default::default()` | Pass | No |
| `test_a_matrix_dimensions` (C/D none check) | Pass (unchanged) — asserts `C.is_none()` and `D.is_none()` with default config | Pass — lines 206-207 still assert None | No (spec referenced as `test_transition_matrices_c_d_none` but the check lives in `test_a_matrix_dimensions`) |
| `test_centered_vs_forward_convergence` | Compile fix needed — struct literal without `compute_sensor_derivatives` | Compile fix applied (line 222, 228) | No |
| `test_b_matrix_structure` | Compile fix needed — struct literal without `compute_sensor_derivatives` | Compile fix applied (line 262) | No (spec called it `test_b_matrix_nonzero`) |
| `test_activation_derivatives` | Compile fix needed — struct literal without `compute_sensor_derivatives` | Compile fix applied (line 353) | No (spec called it `test_identity_at_origin`) |
| `test_contact_sensitivity` | Compile fix needed — struct literal without `compute_sensor_derivatives` | Compile fix applied (line 401) | No (spec called it `test_contact_derivative_nonzero`) |
| `test_fd_convergence` | Compile fix needed — 2 struct literals without `compute_sensor_derivatives` | Compile fix applied (lines 500, 506) | No (spec called it `test_eps_convergence`) |
| `fd_convergence_check` | Compile fix needed — 2 struct literals without `compute_sensor_derivatives` | Compile fix applied (lines 3491, 3497) | No |
| `test_implicitfast_derivative_consistency` / `test_implicit_derivative_consistency` | Compile fix needed — 4 struct literals without `compute_sensor_derivatives` | Compile fix applied (lines 1076, 1087, 1149, 1160) | No |

**Unexpected regressions:** None. Test names in the spec's blast radius
table were slightly different from actual names (e.g., spec said
`test_b_matrix_nonzero` → actual is `test_b_matrix_structure`), but all
identified sites were correctly updated.

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| C/D opt-in: nullable output pointers -> `DerivativeConfig.compute_sensor_derivatives: bool` | Use `config.compute_sensor_derivatives` wherever MuJoCo checks pointer nullability | **Yes** | `compute_sensors` gate at lines 269 (fd) and 2733 (hybrid) |
| `mj_stepSkip()` -> `forward_skip + integrate` for sensor-only FD passes; `step()` for piggybacked FD columns | For sensor-only FD passes (S3): use `scratch.forward_skip(model, skipstage, false)?; scratch.integrate(model)`. For piggybacked FD columns (S2, S3): keep `scratch.step(model)?`. | **Yes** | Sensor-only passes use `forward_skip + integrate`. Piggybacked passes use `step()`. Verified in hybrid path at lines 2855/2971/3042/3236 (sensor-only) and 2902/3098/3289 (piggybacked). |
| Matrix layout: nalgebra column-major, `column_mut(i).copy_from(&col)` writes columns directly | Direct port — nalgebra column writes match `mjd_stepFD`'s column layout. No transpose step needed. | **Yes** | All C/D columns written via `column_mut(i).copy_from(&scol)` |
| `getState()` sensor capture: `mju_copy(sensor, d->sensordata, m->nsensordata)` -> `scratch.sensordata.clone()` | Use `scratch.sensordata.clone()` wherever MuJoCo uses `getState(,,,sensor)` | **Yes** | All sensor captures use `scratch.sensordata.clone()` |
| `extract_state()`: returns tangent-space state directly. Sensor extraction is independent (flat f64 copy). | Direct port — sensor extraction is independent of state extraction. | **Yes** | `extract_state()` unchanged. Sensor data captured independently via `.sensordata.clone()`. |
| Sensor dimensions: `ns = m->nsensordata` -> `model.nsensordata` | Direct port — no translation needed | **Yes** | `let ns = model.nsensordata;` at lines 269 and 2733 |
| `skipsensor` polarity: `true` means SKIP sensors | Direct port — same polarity. Pass `!config.compute_sensor_derivatives` for sensor control. | **Yes** | Sensor-only FD passes use `forward_skip(skipstage, false)` where `false` = don't skip sensors |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| (none) | | | | |

No weak implementations found. All code paths are clean, no TODOs, no loose
tolerances, no shortcuts. The implementation is thorough with proper
`compute_sensors` gating throughout.

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Analytical sensor derivatives (per-sensor-type analytical Jacobians) | Out of Scope, bullet 1 | `future_work_10f.md` | DT-157 | **Yes** — added during review |
| Inverse dynamics sensor derivatives (`DsDq`, `DsDv`, `DsDa` in `mjd_inverseFD`) | Out of Scope, bullet 2 | `future_work_10f.md` | DT-158 | **Yes** — added during review |
| Parallel FD computation (DT-49) | Out of Scope, bullet 3 | `future_work_10f.md` | DT-49 | **Yes** — pre-existing, still pending |
| `step()` -> `forward_skip + integrate` migration for existing FD loops (rubric EGT-8) | Out of Scope, bullet 4 | `future_work_10f.md` | DT-159 | **Yes** — added during review |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| (none) | | | | |

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| DT-47 needs "DONE" status in `future_work_10f.md` | Review section 8 audit | `future_work_10f.md` | DT-47 | **Done** — marked DONE during review |
| 3 deferred items lacked DT entries | Review section 8 audit | `future_work_10f.md` | DT-157, DT-158, DT-159 | **Done** — entries added during review |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| (none) | | | |

---

## 9. Test Coverage Summary

**Domain test results:**
```
sim-core:              1247 passed, 0 failed, 2 ignored
sim-conformance-tests: 603 passed, 0 failed, 0 ignored
sim-mjcf:              187 passed, 0 failed, 0 ignored
sim-physics:           331 passed, 0 failed, 0 ignored
sim-constraint:        39 passed, 0 failed, 0 ignored
sim-muscle:            6 passed, 0 failed, 0 ignored
sim-tendon:            63 passed, 0 failed, 0 ignored
sim-sensor:            22 passed, 0 failed, 0 ignored
sim-types:             53 passed, 0 failed, 0 ignored
sim-urdf:              52 passed, 0 failed, 0 ignored
sim-simd:              32 passed, 0 failed, 0 ignored
Total:                 2,674 passed, 0 failed
```

**New tests added:** 11 sensor derivative tests (T1-T8, T10-T12) + helper
function `sensor_pendulum_2link`. Total Phase 11: +33 tests (30 → 63 in
derivatives.rs).

**Tests modified:** 13 DerivativeConfig struct-literal sites across
`derivatives.rs` (7) and `implicit_integration.rs` (4) and
`fd_convergence_check` (2) — added `compute_sensor_derivatives: false`.

**Pre-existing test regressions:** None.

**Clippy:** Clean (0 warnings).
**Fmt:** Clean.

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | **Pass** — all conformance gaps closed. `skipsensor` perf optimization deferred (not a conformance gap). |
| Spec section compliance | 2 | **Pass** — all 5 sections (S1-S5) grade Pass. No deviations. |
| Acceptance criteria | 3 | **Pass** — all 14 ACs verified with passing tests. |
| Test plan completeness | 4 | **Pass** — all 11 planned tests + 1 supplementary test implemented. All edge cases covered. |
| Blast radius accuracy | 5 | **Pass** — all predicted changes occurred. No unexpected files. Non-modification sites untouched. Test name discrepancies cosmetic only. |
| Convention fidelity | 6 | **Pass** — all 7 convention notes followed exactly. |
| Weak items | 7 | **Pass** — no weak implementations found. |
| Deferred work tracking | 8 | **Pass** — all 4 Out of Scope items tracked. DT-47 needs DONE status update (action item below). |
| Test health | 9 | **Pass** — 2,674 tests pass, 0 fail. Clippy/fmt clean. |

**Overall:** **Ship.** Implementation is faithful to the spec across all
review dimensions. No fixes needed, no weak items, no regressions.

**Items fixed during review:**
- Marked DT-47 as DONE in `future_work_10f.md`
- Added DT-157 (analytical sensor derivatives) to `future_work_10f.md`
- Added DT-158 (inverse dynamics sensor derivatives) to `future_work_10f.md`
- Added DT-159 (`step()` → `forward_skip + integrate` FD migration) to `future_work_10f.md`
- Updated index (`future_work_10b.md`) totals: 115 → 118

**Items to fix before shipping:** None — all action items resolved during review.

**Items tracked for future work:**
- DT-157: Analytical sensor derivatives (per-sensor-type Jacobians)
- DT-158: Inverse dynamics sensor derivatives (DsDq/DsDv/DsDa)
- DT-159: `step()` → `forward_skip + integrate` migration for FD `skipsensor` optimization
- DT-49: Parallel FD computation (pre-existing)
