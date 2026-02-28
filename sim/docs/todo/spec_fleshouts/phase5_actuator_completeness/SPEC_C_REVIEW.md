# Spec C — Hill-Type Muscle Dynamics: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_C.md`
**Implementation session(s):** Session 17
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
| Muscle activation dynamics | `mju_muscleDynamics()` in `engine_util_misc.c:1482` | `muscle_activation_dynamics()` in `muscle.rs:103` — exact match | | |
| Muscle force: active | `mju_muscleGain()` piecewise-quadratic FL×FV | `GainType::Muscle` in `actuation.rs:513` — exact match | | |
| Muscle force: passive | `mju_muscleBias()` half-quadratic FP | `BiasType::Muscle` in `actuation.rs:542` — exact match | | |
| Hill-type dynamics | Not implemented (only `mjDYN_MUSCLE`) | Not implemented | | |
| F0 resolution | `set0()` in `engine_setconst.c` | `compute_actuator_params()` Phase 4, `muscle.rs:334` — `Muscle` only | | |

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

### S1. Add `ActuatorDynamics::HillMuscle` enum variant

**Grade:**

**Spec says:**
Add `ActuatorDynamics::HillMuscle` variant to `core/src/types/enums.rs`,
placed before `User`. No payload — all parameters live in `gainprm`/`dynprm`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Add `GainType::HillMuscle` and `BiasType::HillMuscle` enum variants

**Grade:**

**Spec says:**
Add `GainType::HillMuscle` and `BiasType::HillMuscle` variants to
`core/src/types/enums.rs`, placed before `User` in each enum.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Add HillMuscle force computation in `mj_fwd_actuation()`

**Grade:**

**Spec says:**
Add `GainType::HillMuscle` gain arm computing `−F0 × FL(L_norm) × FV(V_norm) ×
cos(α)` and `BiasType::HillMuscle` bias arm computing `−F0 × FP(L_norm) ×
cos(α)` in `core/src/forward/actuation.rs`. Uses inline Hill curve functions
(Gaussian FL, Hill FV, exponential FP). Parameters extracted from `gainprm`
indices 2, 4–7. Rigid tendon geometry:
`fiber_length = (mt_length − tendon_slack_length) / cos(α)`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Add `HillMuscle` dynamics dispatch in `mj_fwd_actuation()`

**Grade:**

**Spec says:**
Add `ActuatorDynamics::HillMuscle` arm identical to the existing `Muscle` arm
— calls `muscle_activation_dynamics()` and supports `actearly` via
`mj_next_activation()`. File: `core/src/forward/actuation.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Widen F0 resolution in `compute_actuator_params()`

**Grade:**

**Spec says:**
Widen the Phase 4 `dyntype == Muscle` guard in `core/src/forward/muscle.rs`
(line 336) to also include `HillMuscle`. Same formula: `F0 = scale / acc0`.
Also sync `biasprm[2] = gainprm[2]`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Widen `is_muscle` in `mj_set_length_range()`

**Grade:**

**Spec says:**
Widen the `is_muscle` check in `core/src/forward/muscle.rs` (line 533) from
`GainType::Muscle || BiasType::Muscle` to include `GainType::HillMuscle` and
`BiasType::HillMuscle` via `matches!()` macro.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Update derivatives pipeline

**Grade:**

**Spec says:**
Widen 8+ match sites in `core/src/derivatives.rs` to include HillMuscle
variants alongside Muscle (skip arms, FD fallback arms, integration derivative
arm). All changes mechanical — same pattern as existing Muscle arms. No
analytical derivatives needed (FD fallback).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. Update MJCF builder

**Grade:**

**Spec says:**
In `mjcf/src/builder/actuator.rs`: add `"hillmuscle"` parsing in
`parse_dyntype()`, `parse_gaintype()`, `parse_biastype()`. Set `act_num = 1`.
Auto-set `gaintype`/`biastype` to `HillMuscle` when `dyntype = HillMuscle`
and not explicitly overridden. Default `gainprm` for HillMuscle:
`[0.75, 1.05, -1.0, 200.0, 0.10, 0.20, 10.0, 0.0, 35.0, 0.0]`. Enforce
`actlimited = true`, `actrange = [0, 1]`. Validate HillMuscle parameters
(optimal_fiber_length > 0, tendon_slack_length >= 0, etc.).

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | HillMuscle activation dynamics match MuJoCo — hard switch | T1 | | |
| AC2 | HillMuscle activation dynamics — smooth blend | T2 | | |
| AC3 | HillMuscle force at isometric optimal length (F0=1000, gain=−1000, bias=0) | T3 | | |
| AC4 | HillMuscle force with pennation angle (20°, gain≈−939.69) | T4 | | |
| AC5 | HillMuscle F0 auto-computation (gainprm[2] < 0 → F0 = scale/acc0) | T5 | | |
| AC6 | HillMuscle dynamics dispatch reuses `muscle_activation_dynamics()` (act_dot identical to Muscle) | T6 | | |
| AC7 | HillMuscle with `actearly = true` (force != 0 when act=0, ctrl=1) | T7 | | |
| AC8 | Existing Muscle actuators unaffected (regression) | T8 | | |
| AC9 | Mixed Muscle + HillMuscle model (Muscle behavior unchanged) | T9 | | |
| AC10 | MJCF parsing round-trip (`dyntype="hillmuscle"` → correct types, gainprm, act_num, actlimited, actrange) | T10 | | |
| AC11 | Builder validation rejects invalid parameters (optimal_fiber_length=0 → error) | T11 | | |
| AC12 | No `#[allow(unreachable_patterns)]` in modified files | — (code review) | | |
| AC13 | HillMuscle arm present in all exhaustive match sites | — (code review) | | |
| AC14 | `lengthrange` mode filtering includes HillMuscle (`is_muscle` widened) | T12 | | |
| AC15 | HillMuscle force with zero lengthrange (no NaN, matches AC3) | T13 | | |
| AC16 | Mixed gain/bias type override (`dyntype="hillmuscle" gaintype="affine"` honored) | T14 | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Activation dynamics — hard switch path (ctrl=0.8, act=0.3, tausmooth=0 → act_dot=52.631578...) | | | |
| T2 | Activation dynamics — smooth blend path (ctrl=0.55, act=0.5, tausmooth=0.2 → act_dot=2.798737...) | | | |
| T3 | HillMuscle force — isometric at optimal length (F0=1000, L=L_slack+L_opt, V=0, α=0 → force=−1000) | | | |
| T4 | HillMuscle force — pennation angle effect (α=20° → gain≈−939.69) | | | |
| T5 | F0 auto-computation for HillMuscle (gainprm[2]<0 → F0=scale/acc0) | | | |
| T6 | Dynamics dispatch equivalence — Muscle vs HillMuscle (act_dot exact match) | | | |
| T7 | Actearly interaction (actearly=true, act=0, ctrl=1 → force≠0) | | | |
| T8 | Muscle regression — no behavioral change (all pre-Spec-C expected values match) | | | |
| T9 | Mixed Muscle + HillMuscle model (Muscle act_dot and force unchanged) | | | |
| T10 | MJCF parsing — `dyntype="hillmuscle"` round-trip (all enum fields + gainprm values) | | | |
| T11 | Builder validation — invalid parameters (optimal_fiber_length=0, pennation>π/2, vmax<0 → errors) | | | |
| T12 | Length-range mode filtering (HillMuscle passes `is_muscle` check with LengthRangeMode::Muscle) | | | |
| T13 | HillMuscle with zero lengthrange (force is finite, matches AC3) | | | |
| T14 | Mixed gain/bias type override (dyntype=hillmuscle, gaintype=affine → Affine gain used) | | | |

### Supplementary Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 additional cases (V2–V6) | Additional activation dynamics verification points across boundary conditions | | | |
| Hill FL curve unit test | `hill_active_fl()` at L=0.5, 1.0, 1.6, 0.49, 1.61 | | | |
| Hill FV curve unit test | `hill_force_velocity()` at V=−1, 0, 0.5, −0.5 | | | |
| Hill FP curve unit test | `hill_passive_fl()` at L=0.9, 1.0, 1.5 | | | |

### Edge Case Inventory

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Zero pennation angle (α=0) | Degenerates: `cos(0)=1`, fiber force = MTU force. Most common case. | | | |
| Nonzero pennation (α=20°) | cos(α) < 1 reduces force. Validates projection. | | | |
| Zero lengthrange (lr[0] == lr[1]) | HillMuscle uses `fiber_length / optimal_fiber_length` — not affected by L0=0 division-by-zero. But `actuator_length` may make fiber_length ≤ 0. | | | |
| `ctrl` outside [0,1] | Clamped by `muscle_activation_dynamics()`. Same as Muscle. | | | |
| `act` at boundaries 0.0, 1.0 | Boundary time constants (V3, V4 from rubric). | | | |
| `tausmooth > 0` | Quintic sigmoid blend instead of hard switch. Different codepath. | | | |
| `actearly = true` | Next-step activation used for force. | | | |
| F0 auto-computation (`gainprm[2] < 0`) | `F0 = scale / acc0`. Regression against Spec A. | | | |
| MJCF round-trip | `"hillmuscle"` string → enum → correct types. | | | |
| Invalid parameters | Validation catches bad inputs. | | | |
| Mixed Muscle + HillMuscle | Regression: existing Muscle behavior unchanged. | | | |
| `LengthRangeMode::Muscle` with HillMuscle | Mode filter must include HillMuscle. | | | |
| Rigid tendon only (compliant tendon N/A) | Compliant tendon is out of scope. Only rigid tendon supported. N/A — no test needed; documented as out-of-scope deferred work. | | | |
| Mixed gain/bias type override | `dyntype="hillmuscle" gaintype="affine"` — user overrides auto-set. Builder should honor explicit override. | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| New `ActuatorDynamics::HillMuscle` variant (additive, no existing code affected) | | |
| New `GainType::HillMuscle` variant (additive, no existing code affected) | | |
| New `BiasType::HillMuscle` variant (additive, no existing code affected) | | |
| `compute_actuator_params()` Phase 4 — F0 for `Muscle | HillMuscle` (widened guard, no existing behavior changes) | | |
| `mj_set_length_range()` `is_muscle` — includes HillMuscle variants (widened check, no existing behavior changes) | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `core/src/types/enums.rs` (+3 enum variants) | | |
| `core/src/forward/actuation.rs` (+1 dynamics, +1 gain, +1 bias arm, +Hill curve fns) | | |
| `core/src/forward/muscle.rs` (widen F0 guard, widen `is_muscle`) | | |
| `core/src/derivatives.rs` (widen 8 match arms) | | |
| `core/src/integrate/rk4.rs` (no changes — wildcard handles HillMuscle) | | |
| `mjcf/src/builder/actuator.rs` (+3 parse cases, +act_num, auto-set, validation) | | |
| Test files (T1–T14 + supplementary, +200 est. lines) | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `muscle_tests::*` (17 tests) | Pass (unchanged) — tests use `ActuatorDynamics::Muscle`, not `HillMuscle` | | |
| `actuation` integration tests | Pass (unchanged) — tests use existing actuator types | | |
| Phase 4 regression suite (39 tests) | Pass (unchanged) — Phase 5 does not modify Phase 4 code paths | | |
| Builder/parser tests | Pass (unchanged) — existing parse cases unmodified | | |
| Derivatives tests | Pass (unchanged) — existing match arms unmodified (only widened) | | |
| Full sim domain baseline (2,148+ tests) | Pass (unchanged) — extension only, no existing behavior modified | | |

**Unexpected regressions:**

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Activation dynamics | Direct port — HillMuscle reuses `muscle_activation_dynamics()`. Do NOT use sim-muscle's `ActivationDynamics::derivative()` (simple `(u−a)/τ`, no activation-dependent scaling). | | |
| `gainprm` layout | HillMuscle reuses indices 0–3 (range_lo, range_hi, F0, scale). Indices 4–9 are repurposed for Hill-specific parameters. | | |
| `dynprm` layout | Direct port — HillMuscle uses same `dynprm` layout as `Muscle` (indices 0–2: τ_act, τ_deact, τ_smooth). | | |
| Length normalization | HillMuscle does NOT use MuJoCo's normalized-length formula. Instead: `fiber_length = (actuator_length − tendon_slack_length) / cos(α)`, then `norm_len = fiber_length / optimal_fiber_length`. Use `gainprm[4]` for `optimal_fiber_length` and `gainprm[5]` for `tendon_slack_length`. | | |
| Velocity normalization | HillMuscle does NOT use MuJoCo's velocity normalization. Instead: `fiber_vel = actuator_velocity / cos(α)`, then `norm_vel = fiber_vel / (optimal_fiber_length × max_contraction_velocity)`. Use `gainprm[4]` for `optimal_fiber_length` and `gainprm[6]` for `max_contraction_velocity`. | | |
| Force sign | HillMuscle force computation must return **negative** gain (opposing motion convention). sim-muscle's force functions return positive magnitude — negate when assigning to `gain`. | | |
| Activation state ownership | sim-core owns activation state via `data.act[act_adr]`. sim-muscle's internal `ActivationState` struct is NOT used. Do not construct sim-muscle's `ActivationState` or call `ActivationState::update()`. | | |
| Pennation angle units | Store as radians in `gainprm[7]`. No degree→radian conversion during parsing. | | |
| Moment arm | Set `moment_arm = 1.0` when constructing `HillMuscleConfig` from `gainprm`. All moment arm computation handled by sim-core's transmission system. Prevents double-application. | | |

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

**Severity guide:**
- **High** — Conformance risk. MuJoCo would produce different results. Fix before shipping.
- **Medium** — Code quality issue. Correct behavior but fragile, unclear, or not matching spec's prescribed approach. Fix if time permits, else track.
- **Low** — Style or minor robustness. No conformance risk. Track if not fixing now.

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Compliant tendon mode — requires persistent fiber state (`act_num ≥ 2` or separate state array) and custom integration | Out of Scope, bullet 1 | | | |
| Named MJCF attributes (`optlen`, `slacklen`, `pennation`) — convenience UX improvement | Out of Scope, bullet 2 | | | |
| `<hillmuscle>` shortcut element — analogous to `<muscle>`, no MuJoCo precedent | Out of Scope, bullet 3 | | | |
| Variable pennation angle — `α = asin(w / L_fiber)` as function of fiber length | Out of Scope, bullet 4 | | | |
| Configurable curve parameters — Gaussian widths, FV curvature, etc. via extended `gainprm`/`biasprm` | Out of Scope, bullet 5 | | | |
| New `GainType::User` / `BiasType::User` callback infrastructure — per-actuator callbacks | Out of Scope, bullet 6 | | | |

### Discovered During Implementation

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
