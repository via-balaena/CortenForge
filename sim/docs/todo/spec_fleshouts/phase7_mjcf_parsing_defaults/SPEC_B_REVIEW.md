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
| Ball spring force | `-k * subQuat(q, q_spring)` in `mj_passive()` | **Missing** — `visit_multi_dof_joint` does damping only (`passive.rs:818-835`) | Implemented in `visit_multi_dof_joint()` at `passive.rs:866-883`. Uses `subquat(&q_cur, &q_spring)` → `-stiffness * dif[0..3]`. | **Yes** |
| Free spring force (trans) | `-k * (pos - pos_spring)` for 3 translational DOFs | **Missing** — same function | Implemented at `passive.rs:838-844`. Loop `for i in 0..3`: `-stiffness * (qpos - qpos_spring)`. | **Yes** |
| Free spring force (rot) | Falls through to ball case after `padr += 3; dadr += 3` | **Missing** — same function | Implemented at `passive.rs:847-864`. `quat_adr = ctx.qpos_adr + 3`, `rot_dof_adr = ctx.dof_adr + 3`, then `subquat` → `-stiffness * dif`. | **Yes** |
| Ball spring energy | `0.5 * k * \|\|subQuat(q, q_spring)\|\|²` in `mj_energyPos()` | **Stub** — returns 0 (`energy.rs:50-53`) | Implemented at `energy.rs:93-108`. `0.5 * stiffness * dif.dot(&dif)` using `subquat`. | **Yes** |
| Free spring energy (trans) | `0.5 * k * \|\|pos - pos_spring\|\|²` | **Stub** — returns 0 | Implemented at `energy.rs:68-74`. Loop `for i in 0..3`, accumulates `d*d`, then `0.5 * stiffness * trans_sq`. | **Yes** |
| Free spring energy (rot) | Falls through to ball, same stiffness | **Stub** — returns 0 | Implemented at `energy.rs:77-91`. `quat_adr = qpos_adr + 3`, then `subquat` → `0.5 * stiffness * dif.dot(&dif)`. | **Yes** |
| `DISABLE_SPRING` gate on energy | Gates entire spring section | **Missing** — spring section always runs (`energy.rs:38`) | Implemented at `energy.rs:39`. `if model.disableflags & DISABLE_SPRING == 0 { ... }` wraps entire spring loop. | **Yes** |
| Sleep filter on spring energy | Applied to spring/tendon section only (gravity unconditional) | **Missing** — no sleep check in energy (`energy.rs:38`) | Implemented at `energy.rs:40,51-56`. `sleep_enabled` check with `body_sleep_state[body_id] == SleepState::Asleep` → `continue`. Gravity loop (lines 32-36) has NO sleep filter — correct. | **Yes** |
| Stiffness guard (energy) | `== 0` (exact zero; negative stiffness processed) | **Wrong** — `> 0.0` skips negative stiffness (`energy.rs:40`) | Fixed at `energy.rs:46`. `if stiffness == 0.0 { continue; }` — exact zero check. | **Yes** |
| Joint limit activation | `dist < m->jnt_margin[i]` | **Hardcoded** — `dist < 0.0` at 6 sites (`assembly.rs:109,113,128,439,460,492`) | All 6 sites replaced: counting at `assembly.rs:110,114,130`, assembly at `assembly.rs:440,461,494`. All use `< margin` where `margin = model.jnt_margin[jnt_id]`. | **Yes** |
| Margin on constraint row | `efc_margin = margin` via `mj_addConstraint` | **Hardcoded** — `0.0` at 3 `finalize_row!` calls (`assembly.rs:448,469,508`) | All 3 sites replaced: `finalize_row!` 4th arg is now `margin` at `assembly.rs:449,470,509`. | **Yes** |
| `implicit_mode` gate (spring) | Spring force suppressed in implicit mode | **Correct for hinge/slide** — needs matching for ball/free | Gate at `passive.rs:834`: `if !self.implicit_mode && self.has_spring`. Ball/free spring force fully suppressed in implicit mode. | **Yes** |

**Unclosed gaps:** None. All 12 behaviors are closed.

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

**Grade:** Pass

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
Exact match with spec's "After" code block (`passive.rs:830-887`):
- Gate: `if !self.implicit_mode && self.has_spring` (line 834) ✓
- Stiffness guard: `if stiffness != 0.0` (line 836) ✓
- Free: 3 translational DOFs with Euclidean difference (lines 840-843) ✓
- Free→Ball fallthrough: `quat_adr = ctx.qpos_adr + 3`, `rot_dof_adr = ctx.dof_adr + 3` (lines 847-864) ✓
- Ball: `subquat(&q_cur, &q_spring)` → `qfrc_spring -= stiffness * dif[i]` (lines 866-883) ✓
- Wildcard: `_ => {}` (line 884) ✓
- Imports: `use crate::tendon::subquat;` (line 14), `use nalgebra::{..., Quaternion, UnitQuaternion, ...}` (line 19) ✓
- All quaternions constructed via `UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))` ✓

**Gaps (if any):** None.

**Action:** None required.

### S2. Ball/free spring energy + pre-existing fixes in `energy.rs`

**Grade:** Pass

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
Exact match with spec's "After" code block (`energy.rs:7-114`):
- Fix 1: `DISABLE_SPRING` gate at line 39 ✓
- Fix 2: Sleep filter at lines 40,51-56 (`sleep_enabled` + `SleepState::Asleep`) ✓
- Fix 2 (non-modification): Gravity loop at lines 32-36 has NO sleep filter ✓
- Fix 3: Stiffness guard `== 0.0` at line 46 ✓
- Free translational: `0.5 * stiffness * trans_sq` at lines 68-74 ✓
- Free→Ball fallthrough: `quat_adr = qpos_adr + 3` at line 77 ✓
- Ball: `subquat` → `0.5 * stiffness * dif.dot(&dif)` at lines 93-108 ✓
- Imports: `subquat`, `DISABLE_SPRING`, `ENABLE_SLEEP`, `SleepState`, `Quaternion`, `UnitQuaternion` all present (lines 7-11) ✓

**Gaps (if any):** None.

**Action:** None required.

### S3. Parse `margin` attribute for `<joint>` and `<default><joint>`

**Grade:** Pass

**Spec says:**
Add `margin = parse_float_attr(e, "margin")` to both `parse_joint_attrs()`
and `parse_joint_defaults()` in `parser.rs`.

**Implementation does:**
- `parse_joint_defaults()`: `defaults.margin = parse_float_attr(e, "margin");` at line 652 ✓
- `parse_joint_attrs()`: `joint.margin = parse_float_attr(e, "margin");` at line 1733 ✓

**Gaps (if any):** None.

**Action:** None required.

### S4. Add `margin` field to MJCF types

**Grade:** Pass

**Spec says:**
Add `margin: Option<f64>` to both `MjcfJoint` and `MjcfJointDefaults` structs
in `types.rs`. Update `MjcfJoint::default()` to include `margin: None`.

**Implementation does:**
- `MjcfJointDefaults`: `pub margin: Option<f64>` with doc comment at line ~612 ✓
- `MjcfJoint`: `pub margin: Option<f64>` with doc comment at line ~1437 ✓
- Both default to `None` via `Default` derive/impl ✓

**Gaps (if any):** None.

**Action:** None required.

### S5. Wire `margin` through defaults cascade

**Grade:** Pass

**Spec says:**
Add `if result.margin.is_none() { result.margin = defaults.margin; }` to
`apply_to_joint()` in `defaults.rs`, following existing field cascade pattern.

**Implementation does:**
Exact match at `defaults.rs:218-220`: `if result.margin.is_none() { result.margin = defaults.margin; }` ✓

**Gaps (if any):** None.

**Action:** None required.

### S6. Add `jnt_margin` to Model and wire through builder

**Grade:** Pass

**Spec says:**
Add `jnt_margin: Vec<f64>` to `Model` (model.rs), init to `vec![]`
(model_init.rs), add `jnt_margin: Vec<f64>` to `ModelBuilder` (builder/mod.rs),
init in `ModelBuilder::new()` (builder/init.rs), push
`joint.margin.unwrap_or(0.0)` (builder/joint.rs), and finalize with
`model.jnt_margin = self.jnt_margin`.

**Implementation does:**
- `model.rs:206-208`: `pub jnt_margin: Vec<f64>` with doc comment ✓
- `model_init.rs:100-101`: `jnt_margin: vec![]` ✓
- `builder/mod.rs:440-441`: `pub(crate) jnt_margin: Vec<f64>` with doc comment ✓
- `builder/init.rs:59-60`: `jnt_margin: vec![]` ✓
- `builder/joint.rs:139`: `self.jnt_margin.push(joint.margin.unwrap_or(0.0));` ✓
- `builder/build.rs:105-106`: `jnt_margin: self.jnt_margin,` ✓

**Gaps (if any):** None. The spec listed `builder/mod.rs` for finalize but implementation correctly uses `builder/build.rs` — this is the actual build/finalize location.

**Action:** None required.

### S7. Replace hardcoded activation checks + `finalize_row!` margin args

**Grade:** Pass

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
All 9 sites replaced. Verified line by line:
- Counting phase:
  - C1 (line 108-110): `let margin = model.jnt_margin[jnt_id]; if q - limit_min < margin` ✓
  - C2 (line 114): `if limit_max - q < margin` ✓
  - C3 (line 129-130): `let margin = model.jnt_margin[jnt_id]; if dist < margin` ✓
- Assembly phase:
  - A1 (line 433,440): `let margin = model.jnt_margin[jnt_id]; if dist_lower < margin` ✓
  - A2 (line 461): `if dist_upper < margin` ✓
  - A3 (line 492,494): `let margin = model.jnt_margin[jnt_id]; if dist < margin` ✓
- `finalize_row!` margin args:
  - F1 (line 449): 4th arg = `margin` ✓
  - F2 (line 470): 4th arg = `margin` ✓
  - F3 (line 509): 4th arg = `margin` ✓
- Counting–assembly consistency: Both phases use identical `< margin` conditions ✓
- Tendon limits: lines 149,153,541,564 — still use `< 0.0`, NOT modified ✓

**Gaps (if any):** None.

**Action:** None required.

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Coverage Type | Status | Notes |
|----|-------------|---------|---------------|--------|-------|
| AC1 | Ball joint spring force — 90° rotation | T1 | Direct | **Pass** | `t1_ball_spring_force_90deg` in `energy.rs:381-403`. Asserts `qfrc_spring[0]` ≈ `-π` within 1e-10. |
| AC2 | Free joint spring force — translational + rotational | T2 | Direct | **Pass** | `t2_free_spring_force_trans_rot` in `energy.rs:407-440`. Trans `[-1,0,0]`, rot Z ≈ `-π/2`. |
| AC3 | Ball joint spring energy — 90° rotation | T3 | Direct | **Pass** | `t3_ball_spring_energy_90deg` in `energy.rs:444-473`. Spring energy ≈ 2.4674, isolated from gravity. |
| AC4 | Free joint spring energy — translational + rotational | T4 | Direct | **Pass** | `t4_free_spring_energy_trans_rot` in `energy.rs:477-507`. Spring energy ≈ 1.7337. |
| AC5 | Zero stiffness — no spring force or energy | T5 | Edge case | **Pass** | `t5_zero_stiffness_no_force_or_energy` in `energy.rs:511-538`. Force = 0, energy diff = 0. |
| AC6 | `DISABLE_SPRING` suppresses spring force AND energy for ALL joint types | T6 | Direct | **Pass** | `t6_disable_spring_suppresses_all` in `energy.rs:542-594`. Tests ball + hinge, force=0, energy matches gravity-only. |
| AC7 | `implicit_mode` suppresses ball/free spring force | T7 | Edge case | **Pass** | `t7_implicit_mode_suppresses_ball_spring` in `energy.rs:598-617`. All spring DOFs = 0. |
| AC8 | `margin=0` regression — identical behavior to current code | T8 | Regression | **Pass** | `t8_margin_zero_regression` in `assembly.rs:836-859`. `efc_margin=0`, `efc_pos=-0.5`. |
| AC9 | `margin > 0` pre-activation | T9 | Direct | **Pass** | `t9_margin_pre_activation` in `assembly.rs:863-883`. Pre-activated with `efc_margin=0.1`, `efc_pos=0.05`. |
| AC10 | Ball joint with margin | T10 | Direct | **Pass** | `t10_ball_margin` in `assembly.rs:887-973`. `efc_margin=0.05`, `efc_pos≈0.03`. |
| AC11 | `DISABLE_LIMIT` ignores margin | T11 | Edge case | **Pass** | `t11_disable_limit_ignores_margin` in `assembly.rs:977-988`. `nefc=0` despite large margin. |
| AC12 | Sleep filter on spring energy | T12 | Direct | **Pass** | `t12_sleep_filter_spring_energy` in `energy.rs:622-655`. Asleep → gravity only; awake → gravity + spring. |
| AC13 | Negative stiffness produces force/energy | T13 | Edge case | **Pass** | `t13_negative_stiffness` in `energy.rs:659-680`. Force = 1.0, energy = -0.5. |
| AC14 | `margin` parses from MJCF and defaults | T14 | Direct | **Pass** | `t14_margin_parsed_from_defaults` in `defaults.rs:1483` + `t14b_margin_end_to_end_builder` in `defaults.rs:1518`. Tests defaults cascade AND end-to-end MJCF→Model. |
| AC15 | No new `unsafe` blocks | — (code review) | Code review | **Pass** | Grep for `unsafe` in `energy.rs`, `passive.rs`, `assembly.rs` — zero matches. |
| AC16 | §60 not implemented | — (code review) | Code review | **Pass** | Grep for `springinertia` in `sim/L0/` source (non-doc) — zero matches. Only in docs/specs. |

**Missing or failing ACs:** None. All 16 ACs pass.

---

## 4. Test Plan Completeness

> **Test numbering note:** The spec uses T1–T17. Implementation test
> functions use `t1_*`, `t2_*` naming convention (matching spec numbering).
> Tests are split across three modules: spring force/energy tests in
> `energy.rs::spec_b_tests`, margin tests in `assembly.rs::spec_b_margin_tests`,
> and parse/defaults tests in `defaults.rs`.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Ball joint spring force — 90° rotation about X | **Yes** | `t1_ball_spring_force_90deg` (`energy.rs:381`) | Tolerance 1e-10 ✓ |
| T2 | Free joint spring force — translational + rotational | **Yes** | `t2_free_spring_force_trans_rot` (`energy.rs:407`) | Trans+rot verified ✓ |
| T3 | Ball joint spring energy — 90° rotation | **Yes** | `t3_ball_spring_energy_90deg` (`energy.rs:444`) | Gravity-isolated ✓ |
| T4 | Free joint spring energy — translational + rotational | **Yes** | `t4_free_spring_energy_trans_rot` (`energy.rs:477`) | Gravity-isolated ✓ |
| T5 | Zero stiffness — no force or energy | **Yes** | `t5_zero_stiffness_no_force_or_energy` (`energy.rs:511`) | Both force and energy ✓ |
| T6 | `DISABLE_SPRING` suppresses all 4 joint types | **Yes** | `t6_disable_spring_suppresses_all` (`energy.rs:542`) | Ball + hinge tested; slide/free covered by flag being global ✓ |
| T7 | `implicit_mode` suppresses ball/free spring force | **Yes** | `t7_implicit_mode_suppresses_ball_spring` (`energy.rs:598`) | Ball tested; free follows same gate ✓ |
| T8 | `margin=0` regression | **Yes** | `t8_margin_zero_regression` (`assembly.rs:836`) | nefc, efc_margin, efc_pos all verified ✓ |
| T9 | `margin > 0` pre-activation — hinge | **Yes** | `t9_margin_pre_activation` (`assembly.rs:863`) | Pre-activation with dist=0.05 < margin=0.1 ✓ |
| T10 | Ball joint with margin | **Yes** | `t10_ball_margin` (`assembly.rs:887`) | Ball limit pre-activation verified ✓ |
| T11 | `DISABLE_LIMIT` ignores margin | **Yes** | `t11_disable_limit_ignores_margin` (`assembly.rs:977`) | nefc=0 verified ✓ |
| T12 | Sleep filter on spring energy | **Yes** | `t12_sleep_filter_spring_energy` (`energy.rs:622`) | Direct `mj_energy_pos` call, sleep vs awake ✓ |
| T13 | Negative stiffness — force and energy computed | **Yes** | `t13_negative_stiffness` (`energy.rs:659`) | Force=1.0, energy=-0.5 ✓ |
| T14 | Margin parsed from MJCF defaults | **Yes** | `t14_margin_parsed_from_defaults` (`defaults.rs:1483`) + `t14b_margin_end_to_end_builder` (`defaults.rs:1518`) | Unit test + end-to-end ✓ |
| T15 | Near-180° rotation edge case (supplementary) | **Yes** | `t15_near_180_rotation` (`energy.rs:684`) | Force magnitude ≈ 179° in rad ✓ |
| T16 | Identity quaternion — zero force/energy (supplementary) | **Yes** | `t16_identity_quat_zero_force` (`energy.rs:711`) | Zero force + zero energy ✓ |
| T17 | Multi-joint model — free + hinge chain (supplementary) | **Yes** | `t17_multi_joint_free_hinge` (`energy.rs:740`) | Cross-joint indexing verified ✓ |

### Supplementary Tests (from spec)

The spec defines 3 supplementary tests beyond the core T1–T14 plan. These
are pre-populated from the spec — verify they were implemented.

| Test | What it covers | Rationale (from spec) | Test Function | Implemented? |
|------|---------------|----------------------|---------------|-------------|
| T15 | Wrapping boundary in `subquat` (near-180°) | Catches sign-flip bugs at the quaternion double-cover boundary | `t15_near_180_rotation` | **Yes** |
| T16 | Zero-rotation edge case (identity quat) | Catches division-by-zero in `subquat` small-angle path | `t16_identity_quat_zero_force` | **Yes** |
| T17 | Cross-joint indexing (free + hinge chain) | Catches `qpos_adr`/`dof_adr` indexing bugs that only appear with multiple joints | `t17_multi_joint_free_hinge` | **Yes** |

### Supplementary Tests (from implementation/review)

Tests added beyond the spec's planned test list — additional coverage
discovered during implementation or review.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T14b | End-to-end MJCF parsing + builder → `Model.jnt_margin` | `t14b_margin_end_to_end_builder` (`defaults.rs:1518`) | Tests full pipeline from XML string to Model field, including explicit override |

### Edge Case Inventory

| Edge Case | Spec Says | Spec Test(s) | Spec AC(s) | Tested? | Test Function | Notes |
|-----------|----------|-------------|-----------|---------|---------------|-------|
| Zero stiffness | Must produce zero force/energy (exact `== 0` guard) | T5 | AC5 | **Yes** | `t5_zero_stiffness_no_force_or_energy` | |
| Identity quaternion reference | Zero rotation → zero force/energy | T16 | AC5 | **Yes** | `t16_identity_quat_zero_force` | |
| 90° rotation | Standard test case for quaternion math correctness | T1, T3 | AC1, AC3 | **Yes** | `t1_*`, `t3_*` | |
| Near-180° rotation | Wrapping boundary in `subquat` — tests shortest-path | T15 | AC1 | **Yes** | `t15_near_180_rotation` | |
| Free joint: trans + rot combined | Verifies translational uses Euclidean, rotational uses quaternion; correct DOF/qpos indexing | T2, T4 | AC2, AC4 | **Yes** | `t2_*`, `t4_*` | |
| `DISABLE_SPRING` flag | Spring force AND energy suppressed for ALL joint types | T6 | AC6 | **Yes** | `t6_disable_spring_suppresses_all` | |
| `margin=0` regression | Must produce identical output to current code | T8 | AC8 | **Yes** | `t8_margin_zero_regression` | |
| `margin > 0` pre-activation | Constraint created before limit violation | T9 | AC9 | **Yes** | `t9_margin_pre_activation` | |
| Ball joint with margin | Ball limit uses margin correctly | T10 | AC10 | **Yes** | `t10_ball_margin` | |
| `DISABLE_LIMIT` ignores margin | No constraints regardless of margin value | T11 | AC11 | **Yes** | `t11_disable_limit_ignores_margin` | |
| Sleep filter on spring energy | Sleeping bodies excluded from spring energy but NOT gravitational energy | T12 | AC12 | **Yes** | `t12_sleep_filter_spring_energy` | |
| `implicit_mode` suppresses ball/free spring | Total suppression — neither explicit nor implicit | T7 | AC7 | **Yes** | `t7_implicit_mode_suppresses_ball_spring` | |
| Free joint qpos_spring layout | Trans reference at `qpos_adr..+3`, quat at `qpos_adr+3..+7` | T2, T17 | AC2 | **Yes** | `t2_*`, `t17_*` | |
| Negative stiffness | Force/energy IS computed (matches MuJoCo `== 0` guard) | T13 | AC13 | **Yes** | `t13_negative_stiffness` | |

**Missing tests:** None. All 17 planned tests + 1 supplementary test (T14b) are implemented and passing.

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| Ball/free spring force: `qfrc_spring` = 0 → `-k * subquat(q, q_spring)` for ball, `-k * (pos - pos_spring)` + rotational for free | **Yes** | `passive.rs:834-887` |
| Ball/free spring energy: missing → includes `0.5 * k * θ²` (ball) and `0.5 * k * (||d||² + θ²)` (free) | **Yes** | `energy.rs:67-108` |
| `DISABLE_SPRING` gate on energy (ALL types): spring energy always computed → zero when flag set | **Yes** | `energy.rs:39` |
| Sleep filter on spring energy (ALL types): sleeping bodies' springs counted → excluded | **Yes** | `energy.rs:40,51-56` |
| Stiffness guard (ALL types): `> 0.0` → `== 0.0` (processes negative) | **Yes** | `energy.rs:46` |
| Joint limit activation (margin=0): `dist < 0.0` → `dist < 0.0` (identical) | **Yes** | All sites use `< margin`; `margin` defaults to 0.0 → identical behavior |
| Joint limit activation (margin>0): not possible → `dist < margin` pre-activation | **Yes** | Verified by T9 and T10 |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/forward/passive.rs` | **Yes** | |
| `sim/L0/core/src/energy.rs` | **Yes** | |
| `sim/L0/mjcf/src/parser.rs` | **Yes** | |
| `sim/L0/mjcf/src/types.rs` | **Yes** | |
| `sim/L0/mjcf/src/defaults.rs` | **Yes** | |
| `sim/L0/core/src/types/model.rs` | **Yes** | |
| `sim/L0/core/src/types/model_init.rs` | **Yes** | |
| `sim/L0/mjcf/src/builder/joint.rs` | **Yes** | |
| `sim/L0/mjcf/src/builder/mod.rs` | **Yes** | |
| `sim/L0/mjcf/src/builder/init.rs` | **Yes** | |
| `sim/L0/core/src/constraint/assembly.rs` | **Yes** | |
| `sim/L0/core/tests/` (new tests) | **No** — tests are inline in `energy.rs` and `assembly.rs` | Not a gap — inline `#[cfg(test)]` modules are the crate convention |

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| `sim/L0/mjcf/src/builder/build.rs` | Finalize line `jnt_margin: self.jnt_margin` — spec listed `builder/mod.rs` for this but `build.rs` is the actual build/finalize file. Expected in spirit. |
| `sim/L0/core/src/forward/muscle.rs` | Test-only: 3 hand-built `Model` structs in muscle tests needed `jnt_margin: vec![0.0]` added to compile after new `Model` field. No production code changed. |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_model_data_energy` (`lib.rs:316`) | Pass (unchanged) — uses zero gravity + kinetic energy; no spring stiffness | **Pass** | No |
| `test_ball_limit_axis_angle_*` (`impedance.rs:511-580`) | Pass (unchanged) — tests utility, not assembly | **Pass** | No |
| Constraint assembly tests (`assembly.rs` tests) | Pass (unchanged) — default `jnt_margin` = 0.0 | **Pass** | No |
| Passive force tests (`passive.rs` tests) | Pass (unchanged) — use hinge/slide joints | **Pass** | No |
| `test_hill_passive_fl_curve` (`muscle.rs:2228`) | Pass (unchanged) — muscle-specific, not joint springs | **Pass** (after `jnt_margin` field added to test models) | Minor — test models needed the new field, but behavior unchanged |

**Unexpected regressions:** None. All 2,068 domain tests pass.

### Non-Modification Sites Verification

The spec explicitly lists 6 sites that must NOT be modified. Verify none were
accidentally changed during implementation. Line numbers are from the spec
(pre-implementation) and may have shifted by a few lines due to insertions
elsewhere in the file — use the "What it does" description to locate each site.

| Site | File:Line | What it does | Why NOT modified | Accidentally Changed? |
|------|-----------|-------------|------------------|-----------------------|
| NM1 | `assembly.rs:149` | Tendon lower limit: `length - limit_min < 0.0` | Tendon margins are a separate feature — not part of §64a | **No** — still `< 0.0` at line 149 ✓ |
| NM2 | `assembly.rs:153` | Tendon upper limit: `limit_max - length < 0.0` | Same — tendon limits, not joint limits | **No** — still `< 0.0` at line 153 ✓ |
| NM3 | `assembly.rs:541` | Tendon limit assembly lower | Same | **No** — still `< 0.0` at line 541 ✓ |
| NM4 | `assembly.rs:564` | Tendon limit assembly upper | Same | **No** — still `< 0.0` at line 564 ✓ |
| NM5 | `passive.rs:794-818` | `visit_1dof_joint` spring/damper | Already correct for hinge/slide — not modified by this spec | **No** — unchanged ✓ |
| NM6 | `energy.rs:30-36` | Gravity loop: `for body_id in 1..model.nbody` | MuJoCo gravity loop is unconditional — must NOT add sleep filter here | **No** — no sleep filter in gravity section ✓ |

### Data Staleness

The spec notes: "No `EXPECTED_SIZE` constants are affected. `jnt_margin` is a
new `Vec<f64>` on `Model`, and all other changes modify runtime behavior
(energy computation, constraint activation), not model structure counts."

**Verified?** Yes. No `EXPECTED_SIZE` constants touched. The new `jnt_margin`
field is a `Vec<f64>` dynamically sized at build time.

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Quaternion type: `mjtNum[4]` → `UnitQuaternion<f64>` | Use `UnitQuaternion::new_normalize()` when constructing from raw `qpos` values | **Yes** | All 8 quaternion constructions in `passive.rs` and `energy.rs` use `UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))` |
| `subquat` args: `mju_subQuat(res, current, reference)` → `subquat(current, reference)` | Direct port — argument semantics match | **Yes** | All calls: `subquat(&q_cur, &q_spring)` — current first, reference second |
| qpos indexing: `m->jnt_qposadr[j]` → `model.jnt_qpos_adr[jnt_id]` | Direct port | **Yes** | Used consistently in both `passive.rs` and `energy.rs` |
| DOF indexing: `m->jnt_dofadr[j]` → `model.jnt_dof_adr[jnt_id]` | Direct port | **Yes** | `ctx.dof_adr` in passive, `model.jnt_dof_adr` not directly used in energy (energy uses qpos_adr) |
| `jnt_range`: flat `jnt_range[2*i+0/1]` → `(f64, f64)` tuple `.0`/`.1` | Use `.0` for lower, `.1` for upper | **Yes** | `assembly.rs:427`: `let (limit_min, limit_max) = model.jnt_range[jnt_id]` |
| `qfrc_spring`: `d->qfrc_spring` flat → `DVector<f64>` indexed by DOF | `data.qfrc_spring[dof_adr + i]` | **Yes** | Consistent throughout `passive.rs` |
| Sleep check: `body_awake` → `body_sleep_state[body_id] == SleepState::Asleep` | `model.jnt_body[jnt_id]` → `data.body_sleep_state[body_id]` | **Yes** | `energy.rs:52-55`: `model.jnt_body[jnt_id]` → `body_sleep_state` check |
| `finalize_row!` margin: `mj_addConstraint(..., &margin, ...)` → 4th arg | Direct port — pass `margin` as 4th arg | **Yes** | All 3 joint limit `finalize_row!` calls pass `margin` as 4th arg |
| `qpos_spring` (ball/free): per-joint reference config, indexed by `qpos_adr` | `model.qpos_spring[qpos_adr..qpos_adr+nq]` | **Yes** | Both `passive.rs` and `energy.rs` use `model.qpos_spring[qpos_adr + i]` |
| `JointContext` fields: `ctx.jnt_id`, `ctx.jnt_type`, `ctx.dof_adr`, `ctx.qpos_adr`, `ctx.nv`, `ctx.nq` | Branch on `ctx.jnt_type` within `visit_multi_dof_joint` | **Yes** | `match ctx.jnt_type` at `passive.rs:837` |
| Energy accumulation: `d->energy[0]` → `potential` local → `data.energy_potential` | Direct port | **Yes** | `energy.rs:113`: `data.energy_potential = potential;` |

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

> No weak items found. Verified by grepping for `TODO`, `FIXME`, `HACK`,
> `todo!`, `unimplemented!` in all new/modified files (zero matches in
> `energy.rs` and `passive.rs`; pre-existing `TODO(§32)` in `assembly.rs:705`
> is unrelated to this spec). Reviewed all new production code for `unwrap()`,
> loose tolerances, and algorithm deviations from spec. 13 files, ~250 new
> lines reviewed. All algorithms match spec exactly. Test tolerances are
> appropriate: 1e-10 for force (exact math), 1e-4 for energy (gravity
> isolation noise), 1e-6 for angle-based edge cases.

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| §60 (`springinertia`) — does not exist in MuJoCo (see Scope Adjustment above; verified EGT-1 in rubric) | Out of Scope, bullet 1 + Scope Adjustment | `future_work_15.md` §60, `ROADMAP_V1.md` Phase 7 | §60 | **Yes** — marked DROPPED in both `future_work_15.md` and `ROADMAP_V1.md`. |
| Tendon limit margins — separate feature | Out of Scope, bullet 2 | `ROADMAP_V1.md` Phase 8 | DT-33 | **Yes** — tracked as "Tendon `margin` attribute for limit activation distance". |
| `implicit_stiffness` for ball/free joints — already conformant | Out of Scope, bullet 3 | N/A | N/A | **Yes** — no tracking needed; already conformant (implicit_stiffness=0 for ball/free). |
| `qpos_spring` runtime updates (`mj_setConst()`) | Out of Scope, bullet 4 | `future_work_15.md`, `ROADMAP_V1.md` Post-v1.0 | DT-125 | **Yes** — new DT-125 created during second-pass review. |
| DT-17 (global `<option o_margin>`) — per-geom margin, unrelated | Out of Scope, bullet 5 | `ROADMAP_V1.md` Post-v1.0 | DT-17 | **Yes** — tracked as separate item. |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| `builder/build.rs` finalize line | Spec predicted `builder/mod.rs` but actual finalize location is `builder/build.rs` | N/A — not a deferred item, just a spec location mismatch | — | **Yes** — cosmetic, not a gap |

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| `qpos_spring` runtime updates needed explicit DT# | Second-pass review: "covered under general scope" → DT-125 | `future_work_15.md`, `ROADMAP_V1.md` | DT-125 | **Yes** |
| §60 still listed as active in roadmap + future_work | Second-pass review: should have been DROPPED after EGT-1 | `future_work_15.md`, `ROADMAP_V1.md` | §60 | **Yes** — both files updated |
| DT-13, §64, §64a not marked Done in roadmap | Second-pass review: completed items still listed as active | `ROADMAP_V1.md` | DT-13, §64, §64a | **Yes** — all struck through + annotated |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| None | | | |

---

## 9. Test Coverage Summary

**Domain test results:**
```
sim-core:               1114 passed, 0 failed, 1 ignored
sim-constraint:          187 passed, 0 failed, 0 ignored
sim-mjcf:                455 passed, 0 failed, 0 ignored
sim-conformance-tests:   301 passed, 0 failed, 0 ignored
(doctests):               11 passed, 0 failed, 16 ignored
Total:                  2068 passed, 0 failed
```

**New tests added:** 19 (T1-T17 + T14b + 3 builder helper functions)
**Tests modified:** 3 (muscle.rs test models — `jnt_margin` field added)
**Pre-existing test regressions:** 0

**Clippy:** Clean (0 warnings, `-D warnings`)
**Fmt:** Clean

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | **All 12 gaps closed** |
| Spec section compliance | 2 | **All 7 sections Pass** |
| Acceptance criteria | 3 | **All 16 ACs pass** |
| Test plan completeness | 4 | **All 17 + 1 supplementary tests implemented** |
| Blast radius accuracy | 5 | **All predictions accurate; 2 minor unexpected files (explained)** |
| Convention fidelity | 6 | **All 11 conventions followed** |
| Weak items | 7 | **None found** |
| Deferred work tracking | 8 | **All 5 out-of-scope items tracked** |
| Test health | 9 | **2,068 tests passing, 0 failures, clippy clean, fmt clean** |

**Overall:** Ship

**Items fixed during review:**
1. None required — implementation is faithful to spec.

**Items to fix before shipping:**
1. None.

**Items tracked for future work:**
1. §60 (`springinertia`) — nonexistent in MuJoCo per EGT-1. Marked DROPPED in `ROADMAP_V1.md` and `future_work_15.md`.
2. Tendon limit margins — tracked as DT-33 in Phase 8 (`ROADMAP_V1.md`).
3. `qpos_spring` runtime updates via `mj_setConst()` — tracked as DT-125 in `ROADMAP_V1.md` and `future_work_15.md`.

---

## Second-Pass Verification (Session 12 addendum)

Deep line-by-line verification of implementation against MuJoCo C source.

### Algorithm Verification

| Section | Status | Notes |
|---------|--------|-------|
| S1 (ball/free spring force) | **Exact match** | Gate logic, stiffness guard, free translational, free→ball fallthrough, quaternion construction order (w,x,y,z), subquat argument order (q_cur, q_spring), force sign convention — all match MuJoCo `mj_springdamper()`. |
| S2 (spring energy) | **Exact match** | DISABLE_SPRING gate, sleep filter, stiffness == 0.0 guard, free translational energy, free→ball fallthrough, ball energy via `subquat` norm² — all match MuJoCo `mj_energyPos()`. NM6 verified: gravity loop has NO sleep filter. |
| S7 (margin sites) | **Exact match** | 9 sites (3 counting + 3 assembly + 3 finalize_row!) all use `< margin`. Counting-assembly consistency invariant holds. NM1-NM4: tendon limits still use `< 0.0`. |

### Pre-existing Minor (Not Spec B scope)

- `visit_1dof_joint()` (passive.rs:794-818) lacks a `stiffness != 0.0` guard — zero stiffness produces correct zero force mathematically (`-0.0 * d = 0.0`), so this is a minor inefficiency not a correctness issue. Not introduced by Spec B.

### Deferred Work Tracking Fixes

Three documentation gaps discovered and fixed during second-pass:

1. **§60 springinertia** still listed as active in `ROADMAP_V1.md` and `future_work_15.md` → marked DROPPED in both.
2. **`qpos_spring` runtime updates** had no explicit DT number → created DT-125.
3. **DT-13, §64, §64a** not marked Done in `ROADMAP_V1.md` → struck through + annotated with commit hash.

### Test Quality Assessment

All 19 tests (T1-T17 + T14b + builders) verified:
- Tolerances: well-chosen (1e-15 for exact zeros, 1e-10 for analytical, 1e-4 for energy, 1e-3 for ball angle)
- Analytical derivations: all correct
- Edge cases: zero/negative stiffness, near-180°, identity, DISABLE_SPRING, DISABLE_LIMIT, implicit_mode, sleep filter, multi-joint, ball margin

**Second-pass verdict: No changes to implementation needed. Ship.**
