# Spec B — Joint Physics: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_B.md`
**Implementation session(s):** Session 10
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

## Scope Adjustment (from spec)

The spec's Scope Adjustment section (SPEC_B.md lines 33-52) documents two
changes from the umbrella that the reviewer must be aware of:

1. **§60 (`springinertia`) — DROPPED.** Does not exist in MuJoCo. No
   `springinertia` field in `mjmodel.h`, no XML attribute, zero GitHub search
   results across `google-deepmind/mujoco`. The umbrella conflated
   `dof_armature` with a nonexistent spring-inertia coupling. AC16 verifies
   this was correctly NOT implemented.

2. **§64 — EXPANDED from energy-only to force + energy.** The umbrella scoped
   §64 as "ball/free spring energy only (energy.rs stub)." The rubric's
   empirical verification discovered that spring force (`passive.rs`) is also
   missing — energy without force is physically inconsistent. The spec expanded
   §64 to include both S1 (spring force in `passive.rs`) and S2 (spring energy
   + pre-existing fixes in `energy.rs`). This expansion was approved during
   spec review.

**Impact on review:** S1 (spring force) is in-scope despite the umbrella
only mentioning energy. Do NOT grade S1 as scope creep.

---

## 1. Key Behaviors Gap Closure

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Ball spring force | `-k * subQuat(q, q_spring)` in `mj_passive()` | **Missing** — `visit_multi_dof_joint` does damping only (`passive.rs:818-835`) | | |
| Free spring force (trans) | `-k * (pos - pos_spring)` for 3 translational DOFs | **Missing** — same function | | |
| Free spring force (rot) | Falls through to ball case after `padr += 3; dadr += 3` | **Missing** — same function | | |
| Ball spring energy | `0.5 * k * ||subQuat(q, q_spring)||²` in `mj_energyPos()` | **Stub** — returns 0 (`energy.rs:50-53`) | | |
| Free spring energy (trans) | `0.5 * k * ||pos - pos_spring||²` | **Stub** — returns 0 | | |
| Free spring energy (rot) | Falls through to ball, same stiffness | **Stub** — returns 0 | | |
| `DISABLE_SPRING` gate on energy | Gates entire spring section | **Missing** — spring section always runs (`energy.rs:38`) | | |
| Sleep filter on spring energy | Applied to spring/tendon section only (gravity unconditional) | **Missing** — no sleep check in energy (`energy.rs:38`) | | |
| Stiffness guard (energy) | `== 0` (exact zero; negative stiffness processed) | **Wrong** — `> 0.0` skips negative stiffness (`energy.rs:40`) | | |
| Joint limit activation | `dist < m->jnt_margin[i]` | **Hardcoded** — `dist < 0.0` at 6 sites (`assembly.rs:109,113,128,439,460,492`) | | |
| Margin on constraint row | `efc_margin = margin` via `mj_addConstraint` | **Hardcoded** — `0.0` at 3 `finalize_row!` calls (`assembly.rs:448,469,508`) | | |
| `implicit_mode` gate (spring) | Spring force suppressed in implicit mode | **Correct for hinge/slide** — needs matching for ball/free | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

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

### S1. Ball/free spring force in `passive.rs`

**Grade:**

**Spec says:**
Add spring force computation to `visit_multi_dof_joint()` in `passive.rs`.
Match on `ctx.jnt_type`: Free → 3 translational Euclidean DOFs
(`-k * (pos - pos_spring)`) then fall through to ball case for rotational;
Ball → 3 rotational DOFs via `subquat()`; `_ => {}` for Hinge/Slide (handled
by `visit_1dof_joint`). Gate on `!implicit_mode && has_spring` and
`stiffness != 0.0`. Use `UnitQuaternion::new_normalize()` for raw qpos values.
Required imports: `use crate::tendon::subquat;` and
`use nalgebra::{Quaternion, UnitQuaternion};`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Ball/free spring energy + pre-existing fixes in `energy.rs`

**Grade:**

**Spec says:**
Fix 3 pre-existing gaps affecting ALL joint types: (1) add `DISABLE_SPRING`
gate (`model.disableflags & DISABLE_SPRING == 0`) around entire spring section,
(2) add sleep filter on spring section only — conditional on `ENABLE_SLEEP`
flag (`model.enableflags & ENABLE_SLEEP != 0`), then check
`data.body_sleep_state[body_id] == SleepState::Asleep` via
`model.jnt_body[jnt_id]` (NOT on gravity section), (3) change stiffness guard
from `> 0.0` to `== 0.0` continue. Add ball/free energy: free translational
`0.5 * k * ||pos - pos_spring||²` then fall through to ball rotational
`0.5 * k * ||subquat(q, q_spring)||²`. Required imports: `use
crate::tendon::subquat;`, `use crate::types::{DISABLE_SPRING, ENABLE_SLEEP,
SleepState, ...};`, `use nalgebra::{Quaternion, UnitQuaternion};`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Parse `margin` attribute for `<joint>` and `<default><joint>`

**Grade:**

**Spec says:**
Add `margin = parse_float_attr(e, "margin")` to both `parse_joint_attrs()`
and `parse_joint_defaults()` in `parser.rs`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Add `margin` field to MJCF types

**Grade:**

**Spec says:**
Add `margin: Option<f64>` to both `MjcfJoint` and `MjcfJointDefaults` structs
in `types.rs`. Update `MjcfJoint::default()` to include `margin: None`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Wire `margin` through defaults cascade

**Grade:**

**Spec says:**
Add `if result.margin.is_none() { result.margin = defaults.margin; }` to
`apply_to_joint()` in `defaults.rs`, following existing field cascade pattern.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Add `jnt_margin` to Model and wire through builder

**Grade:**

**Spec says:**
Add `jnt_margin: Vec<f64>` to `Model` (model.rs), init to `vec![]`
(model_init.rs), add `jnt_margin: Vec<f64>` to `ModelBuilder` (builder/mod.rs),
init in `ModelBuilder::new()` (builder/init.rs), push
`joint.margin.unwrap_or(0.0)` (builder/joint.rs), and finalize with
`model.jnt_margin = self.jnt_margin`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Replace hardcoded activation checks + `finalize_row!` margin args

**Grade:**

**Spec says:**
Replace 9 sites in `assembly.rs`. Read `model.jnt_margin[jnt_id]` once per
joint. Counting and assembly phases must use identical activation conditions.
Tendon limits (4 sites at :148, :152, :540, :552) are NOT modified.

Counting phase: C1 (:109) hinge/slide lower `< 0.0` → `< margin`,
C2 (:113) hinge/slide upper `< 0.0` → `< margin`, C3 (:128) ball
`< 0.0` → `< margin`.

Assembly phase: A1 (:439) hinge/slide lower `< 0.0` → `< margin`,
A2 (:460) hinge/slide upper `< 0.0` → `< margin`, A3 (:492) ball
`< 0.0` → `< margin`.

`finalize_row!` margin arg: F1 (:448) `0.0` → `margin`, F2 (:469)
`0.0` → `margin`, F3 (:508) `0.0` → `margin`.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Coverage Type | Status | Notes |
|----|-------------|---------|---------------|--------|-------|
| AC1 | Ball joint spring force — 90° rotation | T1 | Direct | | |
| AC2 | Free joint spring force — translational + rotational | T2 | Direct | | |
| AC3 | Ball joint spring energy — 90° rotation | T3 | Direct | | |
| AC4 | Free joint spring energy — translational + rotational | T4 | Direct | | |
| AC5 | Zero stiffness — no spring force or energy | T5 | Edge case | | |
| AC6 | `DISABLE_SPRING` suppresses spring force AND energy for ALL joint types | T6 | Direct | | |
| AC7 | `implicit_mode` suppresses ball/free spring force | T7 | Edge case | | |
| AC8 | `margin=0` regression — identical behavior to current code | T8 | Regression | | |
| AC9 | `margin > 0` pre-activation | T9 | Direct | | |
| AC10 | Ball joint with margin | T10 | Direct | | |
| AC11 | `DISABLE_LIMIT` ignores margin | T11 | Edge case | | |
| AC12 | Sleep filter on spring energy | T12 | Direct | | |
| AC13 | Negative stiffness produces force/energy | T13 | Edge case | | |
| AC14 | `margin` parses from MJCF and defaults | T14 | Direct | | |
| AC15 | No new `unsafe` blocks | — (code review) | Code review | | |
| AC16 | §60 not implemented | — (code review) | Code review | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

> **Test numbering note:** The spec uses T1–T17. Implementation test
> functions may use `t01_*`, `t02_*` etc. The mapping convention will be
> noted during review execution.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Ball joint spring force — 90° rotation about X | | | |
| T2 | Free joint spring force — translational + rotational | | | |
| T3 | Ball joint spring energy — 90° rotation | | | |
| T4 | Free joint spring energy — translational + rotational | | | |
| T5 | Zero stiffness — no force or energy | | | |
| T6 | `DISABLE_SPRING` suppresses all 4 joint types | | | |
| T7 | `implicit_mode` suppresses ball/free spring force | | | |
| T8 | `margin=0` regression | | | |
| T9 | `margin > 0` pre-activation — hinge | | | |
| T10 | Ball joint with margin | | | |
| T11 | `DISABLE_LIMIT` ignores margin | | | |
| T12 | Sleep filter on spring energy | | | |
| T13 | Negative stiffness — force and energy computed | | | |
| T14 | Margin parsed from MJCF defaults | | | |
| T15 | Near-180° rotation edge case (supplementary) | | | |
| T16 | Identity quaternion — zero force/energy (supplementary) | | | |
| T17 | Multi-joint model — free + hinge chain (supplementary) | | | |

### Supplementary Tests (from spec)

The spec defines 3 supplementary tests beyond the core T1–T14 plan. These
are pre-populated from the spec — verify they were implemented.

| Test | What it covers | Rationale (from spec) | Test Function | Implemented? |
|------|---------------|----------------------|---------------|-------------|
| T15 | Wrapping boundary in `subquat` (near-180°) | Catches sign-flip bugs at the quaternion double-cover boundary | | |
| T16 | Zero-rotation edge case (identity quat) | Catches division-by-zero in `subquat` small-angle path | | |
| T17 | Cross-joint indexing (free + hinge chain) | Catches `qpos_adr`/`dof_adr` indexing bugs that only appear with multiple joints | | |

### Supplementary Tests (from implementation/review)

Tests added beyond the spec's planned test list — additional coverage
discovered during implementation or review.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| | | | |

### Edge Case Inventory

| Edge Case | Spec Says | Spec Test(s) | Spec AC(s) | Tested? | Test Function | Notes |
|-----------|----------|-------------|-----------|---------|---------------|-------|
| Zero stiffness | Must produce zero force/energy (exact `== 0` guard) | T5 | AC5 | | | |
| Identity quaternion reference | Zero rotation → zero force/energy | T16 | AC5 | | | |
| 90° rotation | Standard test case for quaternion math correctness | T1, T3 | AC1, AC3 | | | |
| Near-180° rotation | Wrapping boundary in `subquat` — tests shortest-path | T15 | AC1 | | | |
| Free joint: trans + rot combined | Verifies translational uses Euclidean, rotational uses quaternion; correct DOF/qpos indexing | T2, T4 | AC2, AC4 | | | |
| `DISABLE_SPRING` flag | Spring force AND energy suppressed for ALL joint types | T6 | AC6 | | | |
| `margin=0` regression | Must produce identical output to current code | T8 | AC8 | | | |
| `margin > 0` pre-activation | Constraint created before limit violation | T9 | AC9 | | | |
| Ball joint with margin | Ball limit uses margin correctly | T10 | AC10 | | | |
| `DISABLE_LIMIT` ignores margin | No constraints regardless of margin value | T11 | AC11 | | | |
| Sleep filter on spring energy | Sleeping bodies excluded from spring energy but NOT gravitational energy | T12 | AC12 | | | |
| `implicit_mode` suppresses ball/free spring | Total suppression — neither explicit nor implicit | T7 | AC7 | | | |
| Free joint qpos_spring layout | Trans reference at `qpos_adr..+3`, quat at `qpos_adr+3..+7` | T2, T17 | AC2 | | | |
| Negative stiffness | Force/energy IS computed (matches MuJoCo `== 0` guard) | T13 | AC13 | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Ball/free spring force: `qfrc_spring` = 0 → `-k * subquat(q, q_spring)` for ball, `-k * (pos - pos_spring)` + rotational for free | | |
| Ball/free spring energy: missing → includes `0.5 * k * θ²` (ball) and `0.5 * k * (||d||² + θ²)` (free) | | |
| `DISABLE_SPRING` gate on energy (ALL types): spring energy always computed → zero when flag set | | |
| Sleep filter on spring energy (ALL types): sleeping bodies' springs counted → excluded | | |
| Stiffness guard (ALL types): `> 0.0` → `== 0.0` (processes negative) | | |
| Joint limit activation (margin=0): `dist < 0.0` → `dist < 0.0` (identical) | | |
| Joint limit activation (margin>0): not possible → `dist < margin` pre-activation | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/forward/passive.rs` | | |
| `sim/L0/core/src/energy.rs` | | |
| `sim/L0/mjcf/src/parser.rs` | | |
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/defaults.rs` | | |
| `sim/L0/core/src/types/model.rs` | | |
| `sim/L0/core/src/types/model_init.rs` | | |
| `sim/L0/mjcf/src/builder/joint.rs` | | |
| `sim/L0/mjcf/src/builder/mod.rs` | | |
| `sim/L0/mjcf/src/builder/init.rs` | | |
| `sim/L0/core/src/constraint/assembly.rs` | | |
| `sim/L0/core/tests/` (new tests) | | |

{Any unexpected files changed:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_model_data_energy` (`lib.rs:316`) | Pass (unchanged) — uses zero gravity + kinetic energy; no spring stiffness | | |
| `test_ball_limit_axis_angle_*` (`impedance.rs:511-580`) | Pass (unchanged) — tests utility, not assembly | | |
| Constraint assembly tests (`assembly.rs` tests) | Pass (unchanged) — default `jnt_margin` = 0.0 | | |
| Passive force tests (`passive.rs` tests) | Pass (unchanged) — use hinge/slide joints | | |
| `test_hill_passive_fl_curve` (`muscle.rs:2228`) | Pass (unchanged) — muscle-specific, not joint springs | | |

**Unexpected regressions:**
{To be filled during review execution.}

### Non-Modification Sites Verification

The spec explicitly lists 6 sites that must NOT be modified. Verify none were
accidentally changed during implementation. Line numbers are from the spec
(pre-implementation) and may have shifted by a few lines due to insertions
elsewhere in the file — use the "What it does" description to locate each site.

| Site | File:Line | What it does | Why NOT modified | Accidentally Changed? |
|------|-----------|-------------|------------------|-----------------------|
| NM1 | `assembly.rs:148` | Tendon lower limit: `length - limit_min < 0.0` | Tendon margins are a separate feature — not part of §64a | |
| NM2 | `assembly.rs:152` | Tendon upper limit: `limit_max - length < 0.0` | Same — tendon limits, not joint limits | |
| NM3 | `assembly.rs:540` | Tendon limit assembly lower | Same | |
| NM4 | `assembly.rs:552` | Tendon limit assembly upper | Same | |
| NM5 | `passive.rs:800-814` | `visit_1dof_joint` spring/damper | Already correct for hinge/slide — not modified by this spec | |
| NM6 | `energy.rs` gravity loop | `for body_id in 1..model.nbody` | MuJoCo gravity loop is unconditional — must NOT add sleep filter here | |

### Data Staleness

The spec notes: "No `EXPECTED_SIZE` constants are affected. `jnt_margin` is a
new `Vec<f64>` on `Model`, and all other changes modify runtime behavior
(energy computation, constraint activation), not model structure counts."

**Verified?**

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Quaternion type: `mjtNum[4]` → `UnitQuaternion<f64>` | Use `UnitQuaternion::new_normalize()` when constructing from raw `qpos` values | | |
| `subquat` args: `mju_subQuat(res, current, reference)` → `subquat(current, reference)` | Direct port — argument semantics match | | |
| qpos indexing: `m->jnt_qposadr[j]` → `model.jnt_qpos_adr[jnt_id]` | Direct port | | |
| DOF indexing: `m->jnt_dofadr[j]` → `model.jnt_dof_adr[jnt_id]` | Direct port | | |
| `jnt_range`: flat `jnt_range[2*i+0/1]` → `(f64, f64)` tuple `.0`/`.1` | Use `.0` for lower, `.1` for upper | | |
| `qfrc_spring`: `d->qfrc_spring` flat → `DVector<f64>` indexed by DOF | `data.qfrc_spring[dof_adr + i]` | | |
| Sleep check: `body_awake` → `body_sleep_state[body_id] == SleepState::Asleep` | `model.jnt_body[jnt_id]` → `data.body_sleep_state[body_id]` | | |
| `finalize_row!` margin: `mj_addConstraint(..., &margin, ...)` → 4th arg | Direct port — pass `margin` as 4th arg | | |
| `qpos_spring` (ball/free): per-joint reference config, indexed by `qpos_adr` | `model.qpos_spring[qpos_adr..qpos_adr+nq]` | | |
| `JointContext` fields: `ctx.jnt_id`, `ctx.jnt_type`, `ctx.dof_adr`, `ctx.qpos_adr`, `ctx.nv`, `ctx.nq` | Branch on `ctx.jnt_type` within `visit_multi_dof_joint` | | |
| Energy accumulation: `d->energy[0]` → `potential` local → `data.energy_potential` | Direct port | | |

---

## 7. Weak Implementation Inventory

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

**Severity guide:**

| Severity | Meaning |
|----------|---------|
| **High** | Conformance risk. MuJoCo would produce different results. Fix before shipping. |
| **Medium** | Code quality issue. Correct behavior but fragile, unclear, or not matching spec's prescribed approach. Fix if time permits, else track. |
| **Low** | Style or minor robustness. No conformance risk. Track if not fixing now. |

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

**If no weak items are found,** document the verification methodology:

> No weak items found. Verified by grepping for `TODO`, `FIXME`, `HACK`,
> `todo!`, `unimplemented!` in all new/modified files (zero matches).
> Reviewed all new production code for `unwrap()`, loose tolerances, and
> algorithm deviations from spec. {N files, ~M lines reviewed.}

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| §60 (`springinertia`) — does not exist in MuJoCo (see Scope Adjustment above; verified EGT-1 in rubric) | Out of Scope, bullet 1 + Scope Adjustment | | | |
| Tendon limit margins — separate feature | Out of Scope, bullet 2 | | | |
| `implicit_stiffness` for ball/free joints — already conformant | Out of Scope, bullet 3 | | | |
| `qpos_spring` runtime updates (`mj_setConst()`) | Out of Scope, bullet 4 | | | |
| DT-17 (global `<option o_margin>`) — per-geom margin, unrelated | Out of Scope, bullet 5 | | | |

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

**Overall:** {Ship / Ship after fixes / Needs rework}

**Items fixed during review:**
1. ...

**Items to fix before shipping:**
1. ...

**Items tracked for future work:**
1. ...
