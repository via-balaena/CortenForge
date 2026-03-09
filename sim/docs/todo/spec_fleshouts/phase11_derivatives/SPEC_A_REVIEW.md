# Analytical Position Derivatives (`mjd_smooth_pos`) — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase11_derivatives/SPEC_A.md`
**Implementation session(s):** Session 6
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

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Position columns of A | FD via `mjd_stepFD` with `mj_stepSkip(mjSTAGE_NONE)` | FD via `step()` in `mjd_transition_hybrid` lines 1448–1511 | | |
| `∂qfrc_smooth/∂qpos` | Not computed (captured implicitly by FD) | Not computed | | |
| `∂qfrc_constraint/∂qpos` | Captured by FD (re-runs collision detection) | Captured by FD | | |
| Transition A matrix values | Produced by pure FD | Hybrid: analytical velocity cols + FD position cols | | |
| Sleeping bodies | FD captures sleeping state implicitly | Forward pass already handles sleeping; derivative functions receive post-forward-pass data | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

### S1. `mjd_passive_pos` — Passive force position derivatives

**Grade:**

**Spec says:**
Mirrors `mjd_passive_vel` structure. Computes analytically tractable components:
joint springs (hinge/slide/ball/free diagonal/block stiffness, using `mjd_sub_quat`
for ball/free rotational) and tendon springs (`J^T · (-k) · J` outer product).
Gated by `DISABLE_SPRING`. Fluid, gravcomp, flex deferred (AD-1).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. `mjd_actuator_pos` — Actuator force position derivatives

**Grade:**

**Spec says:**
Position derivative of actuator force through actuator length `L`:
`∂force/∂qpos = (∂gain/∂L · input + ∂bias/∂L) · ∂L/∂qpos` where `∂L/∂qpos = moment`.
Per-gain-type `dgain_dl` computation (Fixed=0, Affine=prm[1], Muscle/HillMuscle via
active FL curve derivative). Per-bias-type `dbias_dl` (None=0, Affine=prm[1],
Muscle/HillMuscle=0.0 deferred). Transmission dispatch same as `mjd_actuator_vel`.
Helper functions: `muscle_active_fl_deriv`, `hill_active_fl_deriv`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. `mjd_rne_pos` — RNE position derivatives

**Grade:**

**Spec says:**
Three-phase structure (forward → backward → projection) mirroring `mjd_rne_vel`.
Differentiates `RNEA(q, v, qacc)` at actual acceleration (AD-2), capturing
`(∂M/∂q)·qacc`. Uses new scratch Jacobians `deriv_Dcvel_pos`, `deriv_Dcacc_pos`,
`deriv_Dcfrc_pos`. Forward pass: propagate velocity/acceleration Jacobians
root→leaves with per-joint-type transform derivatives via uniform
`spatial_cross_motion(S_col, X·v)` formula. Backward pass: body force derivatives
via `I·Dcacc + crossForce_vel(I·v)·Dcvel + crossForce_frc(v)·I·Dcvel`. Accumulate
leaves→root. Projection: `qDeriv_pos[dof, :] -= S^T · Dcfrc_pos[body]` plus
`(∂S^T/∂q)·cfrc` cross-product correction term.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. `mjd_smooth_pos` — Assembly function

**Grade:**

**Spec says:**
Simple dispatcher (~10 lines) mirroring `mjd_smooth_vel`. Zeroes `qDeriv_pos`,
then calls `mjd_passive_pos`, `mjd_actuator_pos`, `mjd_rne_pos`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. `Data.qDeriv_pos` field and scratch Jacobians

**Grade:**

**Spec says:**
Add four new fields to Data: `qDeriv_pos: DMatrix<f64>` (nv × nv),
`deriv_Dcvel_pos: Vec<DMatrix<f64>>` (nbody × 6×nv),
`deriv_Dcacc_pos: Vec<DMatrix<f64>>` (nbody × 6×nv),
`deriv_Dcfrc_pos: Vec<DMatrix<f64>>` (nbody × 6×nv).
Initialize in `make_data()` to zeros. No `reset()` changes — fields are
transient, zeroed at start of `mjd_smooth_pos()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Hybrid integration — analytical position columns

**Grade:**

**Spec says:**
Replace FD position column loop (lines 1448–1511) in `mjd_transition_hybrid()`
with analytical position columns for eligible models. Eligibility check:
`fluid_density == 0 && fluid_viscosity == 0 && ngravcomp_body == 0 && nflex == 0
&& !has_site_body_slidercrank_actuators && !has_muscle_bias_actuators`.
Per-integrator `dvdq` computation (Euler: `h·M⁻¹·qDeriv_pos`, ISD:
`(M+hD+h²K)⁻¹·h·qDeriv_pos` with NO correction term, ImplicitFast/Implicit:
similar). Chain rule: `∂q⁺/∂q = dqpos_dqpos + dqpos_dqvel · dvdq`.
Activate `IntegrationDerivatives.dqpos_dqpos` (remove `#[allow(dead_code)]`).
Ineligible models fall back to existing FD loop unchanged.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Export and model eligibility field

**Grade:**

**Spec says:**
Export `mjd_smooth_pos` from `lib.rs` in the `pub use derivatives::` block.
Add `ngravcomp_body: usize` to Model if not already trackable via existing fields.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Hinge joint spring position derivative matches FD (exact: −k) | | | |
| AC2 | Ball joint spring position derivative matches FD (3×3 block) | | | |
| AC3 | Free joint spring position derivative matches FD (6×6 block) | | | |
| AC4 | Tendon spring position derivative matches FD | | | |
| AC5 | Affine actuator position derivative matches FD | | | |
| AC6 | RNE position derivatives match FD for 3-link pendulum | | | |
| AC7 | Transition A position columns match FD — Euler integrator | | | |
| AC8 | Transition A position columns match FD — ISD integrator | | | |
| AC9 | Transition A position columns match FD — ImplicitFast integrator | | | |
| AC10 | Ball/Free joint transition position columns match FD | | | |
| AC11 | FD fallback for ineligible models | | | |
| AC12 | Performance: ≥1.5× speedup for analytical vs FD position columns | | | |
| AC13 | No regression in existing tests | | | |
| AC14 | `dqpos_dqpos` no longer dead code (code review) | — (code review) | | |
| AC15 | `qDeriv_pos` field exists on Data (code review) | — (code review) | | |
| AC16 | Velocity columns unchanged | | | |
| AC17 | Contact isolation — analytical matches FD on contact-free models | | | |
| AC18 | Slide joint spring position derivative matches FD | | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Hinge spring position derivative (analytical, exact −k) | | | |
| T2 | Ball spring position derivative (FD-validated, 3×3 Jacobian) | | | |
| T3 | Free joint spring position derivative (FD-validated, 6×6 block) | | | |
| T4 | Tendon spring position derivative (FD-validated, J^T·k·J pattern) | | | |
| T5 | Affine actuator position derivative (FD-validated, gainprm[1]·ctrl·gear²) | | | |
| T6 | RNE position derivatives — 3-link pendulum (FD-validated, full 3×3) | | | |
| T7 | Transition A matrix — Euler — position columns match FD (also verifies velocity columns unchanged) | | | |
| T8 | Transition A matrix — ISD — position columns match FD (validates (M+hD+h²K)⁻¹ formula) | | | |
| T9 | Transition A matrix — ImplicitFast — position columns match FD | | | |
| T10 | Transition A matrix — Ball/Free joints (9 position columns) | | | |
| T11 | FD fallback for ineligible model (fluid density > 0) | | | |
| T12 | Performance benchmark (100 iterations, ≥1.5× speedup) | | | |
| T13 | Regression — all existing tests pass | | | |
| T14 | Contact isolation — analytical matches FD exactly on contact-free model | | | |

### Supplementary Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T_supp1 | Zero-mass body — no NaN or division by zero in RNE pos | | | |
| T_supp2 | Disabled gravity — gravity contribution is zero | | | |
| T_supp3 | Disabled springs — passive contribution is zero | | | |
| T_supp4 | nv=0 model — all functions return without error | | | |
| T_supp5 | Multi-joint body (FD-validated) — validates all-joints extraction | | | |
| T_supp6 | Slide joint spring position derivative (exact −k, matches FD) → AC18 | | | |
| T_supp7 | FD convergence — error decreases with shrinking ε | | | |
| T_supp8 | Sleeping bodies — derivatives correct after forward pass | | | |
| T_supp9 | Zero-derivative model (negative test) — qDeriv_pos ≈ zero matrix | | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| World body (body_id == 0) | No parent, no joints. RNE forward pass must skip. | | | |
| Zero-mass body | Zero gravity contribution, zero inertia. | | | |
| Disabled gravity (`DISABLE_GRAVITY`) | Gravity torque position derivatives should be zero. | | | |
| Disabled springs (`DISABLE_SPRING`) | `mjd_passive_pos` should skip spring computation. | | | |
| `nv = 0` (no joints) | All derivative functions should return immediately. | | | |
| Single-joint body (hinge) | Simplest non-trivial case. | | | |
| Multi-joint body | Multiple joints on one body (rare but valid). | | | |
| Slide joint | Different from hinge: translation derivative. | | | |
| FD convergence | Shrinking ε confirms analytical matches FD limit. | | | |
| ISD no correction term | Validate that NO +h²·K correction is needed. | | | |
| `dqpos_dqpos` values | Hinge/Slide = 1.0, Ball/Free = quaternion derivative. | | | |
| Sleeping bodies | Position derivatives should still be correct. | | | |
| No position-dependent forces | `qDeriv_pos` should be approximately zero. | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Hybrid position columns (eligible models): FD → analytical via `mjd_smooth_pos()` + chain rule | | |
| Hybrid position columns (ineligible models): unchanged FD | | |
| `IntegrationDerivatives.dqpos_dqpos`: computed but unused → computed and consumed | | |
| `Data` struct size: N fields → N+4 fields | | |

### Files Affected: Predicted vs Actual

| Predicted File | Est. Lines | Actually Changed? | Unexpected Files Changed |
|---------------|-----------|-------------------|------------------------|
| `sim/L0/core/src/derivatives.rs` | +500–700 new, ~50 modified | | |
| `sim/L0/core/src/types/data.rs` | +10 | | |
| `sim/L0/core/src/types/model_init.rs` | +5 | | |
| `sim/L0/core/src/lib.rs` | +1 | | |
| `sim/L0/tests/integration/derivatives.rs` | +300–400 | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_transition_fd_*` | Pass (unchanged) | | |
| `test_hybrid_*` | Pass (identical values) | | |
| `test_smooth_vel_*` | Pass (unchanged) | | |
| `test_sub_quat_*` | Pass (unchanged) | | |
| `test_inverse_fd_*` | Pass (unchanged) | | |
| `test_forward_skip_*` | Pass (unchanged) | | |
| `test_muscle_vel_deriv_*` | Pass (unchanged) | | |
| `test_fluid_deriv_*` | Pass (unchanged) | | |

**Unexpected regressions:**
{To be filled during review execution.}

### Non-Modification Sites: Predicted vs Actual

The spec lists code regions that must NOT be modified. Verify each is untouched.

| Site | What It Does | Modified? | Notes |
|------|-------------|-----------|-------|
| `derivatives.rs:226–353` (`mjd_transition_fd`) | Pure FD path — validation reference | | |
| `derivatives.rs:532–651` (`mjd_actuator_vel`) | Velocity derivatives — independent | | |
| `derivatives.rs:780–1000` (`mjd_rne_vel`) | Velocity RNE — independent | | |
| `derivatives.rs:1513–1553` (Muscle activation FD fallback) | Activation columns — not affected by position columns | | |
| `derivatives.rs:1555–1649` (B matrix computation) | Control influence — not affected by position columns | | |
| `forward/mod.rs:309+` (`forward_skip`) | Pipeline function — not modified by position derivatives | | |

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Position derivative storage | CortenForge extension — `Data.qDeriv_pos` (nv × nv dense) | | |
| Tangent space | `mj_integrate_pos_explicit` for perturbation — direct port | | |
| `qDeriv_pos` indexing | Row = force DOF, col = position tangent DOF (same as `qDeriv`) | | |
| SpatialVector layout | `[angular; linear]` — direct port | | |
| Quaternion convention | `(w, x, y, z)` — direct port | | |
| `subquat` output | Body-frame 3-vector via `mjd_sub_quat` — direct port | | |
| Constraint exclusion | Analytical computes `∂qfrc_smooth/∂qpos` only — same approximation as velocity columns | | |

---

## 7. Weak Implementation Inventory

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution.}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Full FK Jacobians `∂xpos/∂qpos`, `∂xmat/∂qpos` | Out of Scope, bullet 1 | | DT-45 | |
| Contact force position derivatives `∂qfrc_constraint/∂qpos` | Out of Scope, bullet 2 | | DT-46 | |
| Fluid force position derivatives | Out of Scope, bullet 3 | | | |
| Gravity compensation position derivatives | Out of Scope, bullet 4 | | | |
| Flex bending/edge position derivatives | Out of Scope, bullet 5 | | | |
| Actuator moment-arm cross-term for Site/Body/SliderCrank | Out of Scope, bullet 6 | | | |
| Muscle/HillMuscle bias passive FL position derivative | Out of Scope, bullet 7 | | | |
| Automatic differentiation | Out of Scope, bullet 8 | | DT-50 | |
| Sparse derivative storage | Out of Scope, bullet 9 | | DT-48 | |
| Parallel FD | Out of Scope, bullet 10 | | DT-49 | |

### From Spec's "Scope Adjustment" Deferred Components

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Fluid forces (`qfrc_fluid`) analytical position derivatives | Scope Adjustment, deferred table | | | |
| Gravity compensation (`qfrc_gravcomp`) analytical position derivatives | Scope Adjustment, deferred table | | | |
| Flex bending forces (cotangent/Bridson) analytical position derivatives | Scope Adjustment, deferred table | | | |
| Actuator moment-arm cross-term `(∂moment/∂qpos)·force` for Site/Body/SliderCrank | Scope Adjustment, deferred table | | | |
| Muscle/HillMuscle bias passive FL position derivative (`∂bias/∂L`) | Scope Adjustment, deferred table | | | |

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
