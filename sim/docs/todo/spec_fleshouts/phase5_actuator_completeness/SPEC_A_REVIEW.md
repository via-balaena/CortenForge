# Spec A — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_A.md`
**Implementation session(s):** Session 3
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
| acc0 for non-muscle actuators | Computed for all `i < m->nu` in `set0()` | Only for `ActuatorDynamics::Muscle`; all others = 0.0 | | |
| dampratio → damping conversion | `set0()`: positive `biasprm[2]` → `-2*dampratio*sqrt(kp*mass)` for position-like actuators | Not implemented; positive `biasprm[2]` remains positive (wrong sign, wrong magnitude) | | |
| `dof_M0` diagonal mass | Computed by `mj_setM0()` via CRB at qpos0 | Does not exist as field; equivalent `qM[(i,i)]` available after CRBA | | |
| lengthrange for unlimited joints | Simulation-based estimation via `evalAct()` | Not implemented; unlimited joints → `lengthrange = (0, 0)` | | |
| lengthrange for site transmissions | Simulation-based estimation via `evalAct()` | Not implemented; sites → `lengthrange = (0, 0)` | | |
| `mjLROpt` / `<lengthrange>` XML element | Configurable options for LR estimation | Not parsed | | |

**Unclosed gaps:**

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

### S1. Extend acc0 computation to all actuators (DT-57)

**Grade:**

**Spec says:**
Remove the `ActuatorDynamics::Muscle` guard from the acc0 loop. The existing
acc0 infrastructure (FK → CRBA → sparse solve → norm) is correct for all
transmission types. Rename function to `compute_actuator_params()`. acc0
computed for every actuator, not just muscles. Includes `build_actuator_moment`
helper extraction for moment vector construction.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Rename callers: `compute_muscle_params` → `compute_actuator_params`

**Grade:**

**Spec says:**
Mechanical rename of `compute_muscle_params()` to `compute_actuator_params()`
across all call sites: `sim/L0/mjcf/src/builder/mod.rs`, test helpers in
`muscle.rs`, and the function definition itself.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. dampratio-to-damping conversion (DT-56)

**Grade:**

**Spec says:**
Insert dampratio conversion loop between acc0 and F0 resolution in
`compute_actuator_params`. Algorithm: (1) position-actuator fingerprint check
`gainprm[0] != -biasprm[1]` using exact float comparison, (2) skip if
`biasprm[2] <= 0`, (3) compute reflected inertia from moment vectors and qM
diagonal (`dof_M0` equivalent), (4) `damping = dampratio * 2 * sqrt(kp * mass)`,
(5) store as `-damping` in `biasprm[2]`. Uses stored moment vectors from acc0
loop (option a). Constant `MJ_MINVAL = 1e-15` for `trn2` guard.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3b. Parse `dampratio` MJCF attribute for position actuators

**Grade:**

**Spec says:**
Add `dampratio: Option<f64>` to `MjcfActuator` struct in `sim/L0/mjcf/src/types.rs`.
Parse from `<position>` element. In builder's position actuator arm: if
`dampratio` is specified, store as positive `biasprm[2]` (dampratio semantics);
if `kv` is specified, store as `-kv` (explicit damping); if neither, `biasprm[2] = 0.0`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Simulation-based length-range estimation (DT-59, DT-77)

**Grade:**

**Spec says:**
Implement `mj_set_length_range()` + `eval_length_range()` + `build_actuator_moment()`
in `muscle.rs`. Sub-sections:

- **S4a:** `LengthRangeOpt` struct with MuJoCo defaults + `LengthRangeMode` enum
  in `sim/L0/core/src/types/model.rs`.
- **S4b:** `mj_set_length_range()` — mode filtering, useexisting check, uselimit
  copy from limits, fallthrough to simulation.
- **S4c:** `eval_length_range()` — two-sided simulation with velocity damping,
  full step1/step2 pipeline (gravity+contacts active), force = `sign * accel *
  moment / ||M^{-1} moment||`, force capping, convergence check.
- **S4d:** Actuator length read from `data.actuator_length[actuator_idx]`
  (populated by step1's transmission stage).
- **S4e:** `LengthRangeError` enum (InvalidRange, ConvergenceFailed).
- **S4f:** Integration into `compute_actuator_params` as Phase 1b (after
  limit-based Phase 1a, before Phase 2 acc0). Option (b) chosen: limit path
  runs unconditionally for all actuators, simulation path runs per LR mode.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | acc0 for motor actuator on single hinge: `acc0 = 20.0 ± 1e-6` (gear=2.0, I_yy=0.1) | T1 | | |
| AC2 | acc0 for position actuator: `acc0 = 10.0 ± 1e-6` (kp=100, kv=10, gear=1.0, I_yy=0.1) | T2 | | |
| AC3 | acc0 unchanged for muscle actuator (regression): `acc0 = 20.0 ± 0.2` | T3 | | |
| AC4 | dampratio conversion: `biasprm[2] = -2*sqrt(100*0.1) ≈ -6.3246 ± 1e-4` (kp=100, dampratio=1.0, gear=1.0, I_yy=0.1) | T4 | | |
| AC5 | dampratio skipped for motor actuator: `biasprm[2] = 0.0` (gainprm[0]=1.0, biasprm[1]=0.0) | T5 | | |
| AC6 | dampratio skipped for explicit kv: `biasprm[2] = -10.0` (kp=100, kv=10) | T6 | | |
| AC7 | lengthrange from limits unchanged: `(-2.0, 2.0)` (range=[-1,1], gear=2.0) | T7 | | |
| AC8 | lengthrange via simulation for unlimited slide: nonzero range found | T8 | | |
| AC9 | lengthrange via simulation for site transmission: valid range (lo < hi) | T9 | | |
| AC10 | function renamed: `compute_muscle_params` does not exist in codebase | — (code review) | | |
| AC11 | existing tests pass: all 2,148+ domain tests pass | T10 | | |
| AC12 | `dampratio` MJCF attribute parsed: positive before compute, negative after | T11 | | |
| AC13 | lengthrange mode filtering: motor on unlimited slide → (0.0, 0.0) with mode=Muscle | T12 | | |
| AC14 | multi-body acc0 + dampratio: 3-body chain values match MuJoCo ±1e-6 | T13 | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | acc0 for motor actuator: single hinge, motor, gear=2.0, I_yy=0.1. Expected acc0=20.0±1e-6 | | | |
| T2 | acc0 for position actuator: single hinge, position (kp=100, kv=10), gear=1.0, I_yy=0.1. Expected acc0=10.0±1e-6 | | | |
| T3 | acc0 muscle regression: existing `build_muscle_model_joint(2.0)`. Expected acc0=20.0±0.2 | | | |
| T4 | dampratio conversion: single hinge, position, kp=100, dampratio=1.0, gear=1.0, I_yy=0.1. Expected biasprm[2]=-6.3246±1e-4 | | | |
| T5 | dampratio skip for motor: gainprm[0]=1.0, biasprm[1]=0.0. Expected biasprm[2]=0.0 | | | |
| T6 | dampratio skip for explicit kv: position with kv=10.0. Expected biasprm[2]=-10.0 | | | |
| T7 | lengthrange limits regression: muscle on limited hinge [-1,1], gear=2.0. Expected (-2.0, 2.0) | | | |
| T8 | lengthrange unlimited slide simulation: muscle on unlimited slide. Expected nonzero range | | | |
| T9 | lengthrange site transmission simulation: muscle with site transmission on hinge. Expected valid range (lo < hi) | | | |
| T10 | existing tests pass: `cargo test -p sim-core -p sim-mjcf -p sim-muscle -p sim-sensor`. All pass | | | |
| T11 | dampratio MJCF round-trip: MJCF with `<position dampratio="1.0" kp="100"/>`. biasprm[2] positive before, negative after | | | |
| T12 | lengthrange mode filtering: motor (non-muscle) on unlimited slide, mode=Muscle. Expected (0.0, 0.0) | | | |
| T13 | multi-body acc0 + dampratio conformance: 3-body chain, 2 hinges, 2 actuators. Values match MuJoCo ±1e-6 | | | |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| `nv == 0` (no DOFs) | acc0 loop should produce 0 for all actuators | | | Supplementary S1 |
| Zero transmission (body at qpos0) | Moment is all zeros → acc0 = 0, dampratio mass = 0 | | | T5 (motor with biastype::None) |
| `gainprm[0] != -biasprm[1]` (not position-like) | Dampratio must NOT fire for motor/velocity/damper | | | T5 |
| `biasprm[2] <= 0` (explicit kv) | Dampratio must NOT convert already-negative kv | | | T6 |
| `biasprm[2] == 0` (zero damping) | Not positive → skip (no conversion to −0) | | | T6 variant |
| Multi-DOF transmission (multi-body chain) | Reflected inertia and acc0 with mass-matrix coupling | | | T13 |
| Unlimited hinge joint | Falls through limit-based path to simulation | | | T8 |
| Site transmission | No limit path at all → always simulation | | | T9 |
| Convergence failure | Simulation doesn't converge → warning, keep (0,0) | | | Supplementary S2 |
| LR mode=Muscle filtering | Non-muscle actuators must NOT get simulation-based LR | | | T12 |
| Multi-DOF tendon + dampratio | Reflected inertia sums across DOFs (exotic but correct) | | | Future (not in v1.0 test set) |

### Supplementary Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| S1 | acc0 with nv=0: zero-DOF model produces acc0=0. Guards against panic on empty qM | | | |
| S2 | lengthrange convergence failure: body transmission → simulation produces (0,0) range. Verifies graceful error handling | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| acc0 for non-muscle actuators: from 0.0 to computed `\|\|M^{-1}J\|\|` | | |
| `compute_muscle_params` → `compute_actuator_params` rename | | |
| dampratio conversion: positive `biasprm[2]` → negative damping | | |
| lengthrange for unlimited joints: from (0,0) to simulation-estimated | | |
| lengthrange for site transmissions: from (0,0) to simulation-estimated | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/forward/muscle.rs` | | |
| `sim/L0/core/src/types/model.rs` | | |
| `sim/L0/core/src/types/model_init.rs` (no change expected) | | |
| `sim/L0/mjcf/src/builder/actuator.rs` | | |
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/builder/mod.rs` | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_fl_curve_shape` | Pass (unchanged) | | |
| `test_fv_curve_shape` | Pass (unchanged) | | |
| `test_fp_curve_shape` | Pass (unchanged) | | |
| `test_activation_dynamics_ramp_up` | Pass (unchanged) | | |
| `test_activation_dynamics_ramp_down` | Pass (unchanged) | | |
| `test_activation_asymmetry` | Pass (unchanged) | | |
| `test_muscle_force_at_optimal_length` | Pass (unchanged) | | |
| `test_muscle_act_num_is_one` | Pass (unchanged) | | |
| `test_motor_actuator_unchanged` | Pass (unchanged) | | |
| `test_control_clamping` | Pass (unchanged) | | |
| `test_force_clamping` | Pass (unchanged) | | |
| `test_muscle_params_transferred` | Pass (unchanged) | | |
| `test_acc0_single_hinge` | Pass (unchanged) | | |
| `test_f0_auto_computation` | Pass (unchanged) | | |
| `test_f0_explicit_not_overridden` | Call site rename needed | | |
| `test_rk4_activation_single_step` | Pass (unchanged) | | |
| `test_sigmoid_boundaries` | Pass (unchanged) | | |
| Phase 4 regression suite (39 tests) | Pass (unchanged) | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `actuator_moment` storage: MuJoCo sparse CSR → CortenForge dense `Vec<DVector<f64>>` | Construct moment vectors manually from transmission type via `build_actuator_moment` helper (same approach as existing `compute_muscle_params`). `data.actuator_moment` is populated by `mj_fwd_actuation` (acceleration stage), NOT by `step1` alone. | | |
| `dof_M0`: MuJoCo separate field (10-element CRB) → CortenForge `data.qM[(i,i)]` diagonal | Read `qM` diagonal instead of separate CRB pass. Numerically equivalent at qpos0. | | |
| `biasprm`/`gainprm` indexing: MuJoCo flat array `m->actuator_biasprm + i*mjNBIAS` → CortenForge `model.actuator_biasprm[i]` Vec of `[f64; 9]` | Direct index: `model.actuator_biasprm[i][k]` replaces `biasprm[k]` | | |
| `mjMINVAL` = `1e-15` | Use `1e-15` for `trn2 > mjMINVAL` guard to match MuJoCo | | |
| `actuator_lengthrange` storage: MuJoCo flat `[2*i]/[2*i+1]` → CortenForge `Vec<(f64, f64)>` | `.0` = min, `.1` = max | | |
| `mj_solveM`: MuJoCo sparse solve → CortenForge `mj_solve_sparse(rowadr, rownnz, colind, qLD_data, qLD_diag_inv, &mut x)` | Copy `rhs` into `x`, then call `mj_solve_sparse` in-place | | |
| `step1`/`step2` pipeline: MuJoCo `mj_step1`+`mj_step2` → CortenForge `data.step1(model)`+`data.step2(model)` | Direct port. For evalAct, run with full gravity/contacts/passive — MuJoCo does NOT disable these. | | |
| LR mode enum: MuJoCo `mjLRMODE_NONE=0..ALL=3` → CortenForge `LengthRangeMode` enum | Direct mapping | | |

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
| `<lengthrange>` XML element parsing | Out of Scope, bullet 1 | | | |
| `dof_M0` as a separate model field | Out of Scope, bullet 2 | | | |
| Spec B transmission types (SliderCrank, JointInParent) | Out of Scope, bullet 3 | | | |

### Discovered During Implementation

Items that came up during implementation that weren't anticipated by the
spec. These are the most likely to be untracked.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Spec Gaps Found During Implementation

Items where the spec was wrong or incomplete and was (or should have been)
updated. Verify the spec was actually updated.

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
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

**Items to fix before shipping:**

**Items tracked for future work:**
