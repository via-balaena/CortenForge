# Phase 6 Spec B — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_B.md`
**Implementation session(s):** Session 8
**Reviewer:** AI agent
**Date:** 2026-02-28

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
| FramePos with reftype | `R_ref^T * (p_obj - p_ref)` via `get_xpos_xmat()` in `mj_sensorPos()` | World-frame position only; `sensor_reftype = None`, `sensor_refid = 0` | | |
| FrameQuat with reftype | `q_ref^{-1} * q_obj` via `get_xquat()` in `mj_sensorPos()` | World-frame quaternion only | | |
| FrameXAxis/YAxis/ZAxis with reftype | `R_ref^T * R_obj[:, col]` in `mj_sensorPos()` | World-frame axis only | | |
| FrameLinVel with reftype | `R_ref^T * (v_obj - v_ref - w_ref × dif)` with Coriolis in `mj_sensorVel()` | World-frame velocity only | | |
| FrameAngVel with reftype | `R_ref^T * (w_obj - w_ref)` in `mj_sensorVel()` | World-frame angular velocity only | | |
| FrameLinAcc/FrameAngAcc with reftype | Ignores `sensor_refid` — world frame only | World-frame acceleration (correct) | | |
| Builder reftype/refid resolution | Resolves reftype string → `mjtObj` + name → ID | Hardcoded `None`/`0` | | |
| No reftype/refname | World-frame output (no transform) | World-frame output (correct) | | |

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

### S1. Builder reference-object resolution

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/builder/sensor.rs`. **MuJoCo equivalent:** MJCF
compiler sensor resolution — resolves `reftype`/`refname` to
`sensor_reftype`/`sensor_refid` model arrays.
Replace hardcoded `MjObjectType::None`/`0` in `builder/sensor.rs:47–48` with
a call to a new `resolve_reference_object()` function that dispatches on the
`reftype` string (`site`/`body`/`xbody`/`geom`/`camera`) and resolves
`refname` to an object ID. Resolution rules: both absent → `(None, 0)`;
reftype present but refname absent → `(None, 0)`; refname present but reftype
absent → infer from name lookup (site → body → geom priority); both present →
dispatch on reftype string. `camera` → warn + `(None, 0)` (DT-117). Invalid
string → `ModelConversionError`. Resolves for ALL sensor types (not just
frame sensors), matching MuJoCo's build-time behavior.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Position-stage reference-frame transforms

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/sensor/position.rs`. **MuJoCo equivalent:**
`mj_sensorPos()` in `engine_sensor.c` — FramePos, FrameQuat,
FrameXAxis/YAxis/ZAxis cases with `sensor_refid[i] >= 0`.
Add `get_ref_pos_mat()` and `get_ref_quat()` helpers mirroring MuJoCo's
`get_xpos_xmat()` and `get_xquat()` dispatch tables. Inline reference-frame
transforms within each match arm after the world-frame computation (option (c)
from EGT-10 DECISION 2). Guard: `sensor_reftype[i] != MjObjectType::None`.
FramePos: `R_ref^T * (p_obj - p_ref)`. FrameQuat: `q_ref^{-1} * q_obj` using
`UnitQuaternion::inverse()` (= `mju_negQuat` for unit quaternions).
FrameXAxis/YAxis/ZAxis: `R_ref^T * axis_world`. `get_ref_quat` dispatch:
XBody → `xquat[refid]` (direct copy), Body → `xquat[refid] * body_iquat[refid]`
(`mulQuat`), Site/Geom → `mat2Quat(xmat)` via `from_matrix_unchecked`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Velocity-stage reference-frame transforms

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/sensor/velocity.rs`. **MuJoCo equivalent:**
`mj_sensorVel()` in `engine_sensor.c` — FrameLinVel, FrameAngVel cases with
`sensor_refid[i] >= 0`.
Duplicate `get_ref_pos_mat()` as file-local in `velocity.rs` (matches existing
pattern: `position.rs` and `velocity.rs` have independent dispatch). Inline
reference-frame transforms for FrameLinVel and FrameAngVel. FrameLinVel:
compute world-frame velocities for both primary and reference objects via
`object_velocity()` with `local_rot = None` (matching MuJoCo's
`mj_objectVelocity()` with `flg_local = 0`), then Coriolis correction
`v_rel = v_obj - v_ref - w_ref × (p_obj - p_ref)`, then rotate into reference
frame `R_ref^T * v_rel`. FrameAngVel: `R_ref^T * (w_obj - w_ref)` with NO
Coriolis term (angular velocity is reference-point-independent). Angular
velocity read from `cvel[body_id][0..3]` directly (equivalent to
`mj_objectVelocity()` angular output, avoids wasted linear computation).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Acceleration-stage: no change (document only)

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/sensor/acceleration.rs`. **MuJoCo equivalent:**
`mj_sensorAcc()` in `engine_sensor.c` — FrameLinAcc, FrameAngAcc.
No code change. MuJoCo ignores `sensor_refid` for FrameLinAcc/FrameAngAcc
(confirmed by inspection: no `get_xpos_xmat` call for reftype in acc arms).
CortenForge already outputs in world frame. Setting `reftype`/`refname` on
acc sensors has no effect on output — the builder stores reftype/refid, but
evaluation never reads it for these sensor types. This matches MuJoCo.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Builder resolves reftype/refname → model arrays | T1 (`t21_*`) | | |
| AC2 | Builder defaults — no reftype/refname → None/0 | T2 (`t22_*`) | | |
| AC3 | FramePos relative to rotated reference body (`[2.0, 1.0, 0.0] ± 1e-10`) | T3 (`t23_*`) | | |
| AC4 | FrameQuat relative to rotated reference site (`[0.7071, 0, 0, -0.7071] ± 1e-4`) | T4 (`t24_*`) | | |
| AC5 | FrameXAxis in reference frame (`[0, -1, 0] ± 1e-10`) | T5 (`t25_*`) | | |
| AC6 | FrameLinVel with Coriolis correction (`[0, -1, 0] ± 1e-6`) | T6 (`t26_*`) | | |
| AC7 | FrameAngVel in reference frame (`[0, 0, 3] ± 1e-10`) | T7 (`t27_*`) | | |
| AC8 | No reftype → world-frame output unchanged (regression) | T8 (`t28_*`) | | |
| AC9 | FrameLinAcc/FrameAngAcc unaffected by reftype | T9 (`t29_*`) | | |
| AC10 | `sensor_dim`/`sensor_adr` invariance | T10 (`t30_*`) | | |
| AC11 | Invalid reftype string → `ModelConversionError` | T11 (`t31_*`) | | |
| AC12 | Unknown refname → `ModelConversionError` | T12 (`t32_*`) | | |
| AC13 | reftype without refname → no transform | T13 (`t33_*`) | | |
| AC14 | Builder uses correct MjObjectType for each reftype string | T14 (`t34_*`) | | |
| AC15 | FramePos body vs xbody numerical distinction | T18 (`t38_*`) | | |
| AC16 | No code change in acceleration.rs | — (code review) | | |
| AC17 | FrameQuat with reftype="body" exercises `xquat * body_iquat` path | T19 (`t39_*`) | | |

**Missing or failing ACs:**

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

> **Test numbering:** T1–T19 are Spec B's local labels. In the test file,
> these map to functions `t21_*` through `t39_*` (continuing from Spec A's
> `t01_*`–`t20_*`).

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Builder resolves reftype="site" + refname="s_ref" → AC1. Model: body with two sites `s1` and `s_ref`. Sensor `<framepos site="s1" reftype="site" refname="s_ref"/>`. Assert: `sensor_reftype[0] == Site`, `sensor_refid[0] == site_id`. | | | |
| T2 | Builder default — no reftype/refname → AC2. Model: body with site. Sensor `<framepos site="s1"/>`. Assert: `sensor_reftype[0] == None`, `sensor_refid[0] == 0`. | | | |
| T3 | FramePos relative to rotated body → AC3. Body A at `[1,0,0]` with hinge at 90° about Z. Site B at `[0,2,0]`. `<framepos site="sB" reftype="xbody" refname="A"/>`. Expected: `[2.0, 1.0, 0.0] ± 1e-6`. Derivation: `Rz(-90°) * ([-1,2,0]) = [2,1,0]`. | | | |
| T4 | FrameQuat relative to rotated reference site → AC4. Body A with hinge at 90° about Z, site `s_ref`. Body B at identity. `<framequat objtype="xbody" objname="B" reftype="site" refname="s_ref"/>`. Expected: `[0.7071, 0, 0, -0.7071] ± 1e-4`. Exercises `mat2Quat(site_xmat)` path. | | | |
| T5 | FrameXAxis in reference frame → AC5. Body A with hinge at 90° about Z. Body B at identity. `<framexaxis objtype="xbody" objname="B" reftype="xbody" refname="A"/>`. Expected: `[0, -1, 0] ± 1e-10`. Derivation: `Rz(-90°) * [1,0,0] = [0,-1,0]`. | | | |
| T6 | FrameLinVel with Coriolis correction → AC6. Body A (ref) at origin with hinge about Z, `qvel = 1.0`. Body B (obj) at `[1,0,0]`, zero linear velocity. `<framelinvel site="sB" reftype="xbody" refname="A"/>`. Expected: `sensordata[1] ≈ -1.0 ± 1e-3`. Coriolis: `w_ref × r = [0,0,1] × [1,0,0] = [0,1,0]`. Tol 1e-3 (floating-point in spatial transport). | | | |
| T7 | FrameAngVel in reference frame → AC7. Body A (ref) with hinge about Z, `qvel_A = 2.0`. Body B (obj) with hinge about Z, `qvel_B = 5.0`. `<frameangvel objtype="xbody" objname="B" reftype="xbody" refname="A"/>`. Expected: `sensordata[2] ≈ 3.0 ± 1e-10`. | | | |
| T8 | No reftype regression guard → AC8. Body with site. Sensor `<framepos site="s1"/>` (no reftype). After forward: `sensordata` matches `data.site_xpos[site_id]` exactly. Verifies Spec A behavior unchanged. | | | |
| T9 | FrameLinAcc/FrameAngAcc unaffected by reftype → AC9. Two bodies, hinge joints. Two FrameLinAcc sensors: one with reftype, one without. Two FrameAngAcc sensors: same. Set `qvel != 0`. After forward: both pairs produce identical sensordata (reftype ignored). | | | |
| T10 | sensor_dim/sensor_adr invariance → AC10. `<framepos site="s1" reftype="xbody" refname="b1"/>` + `<framepos site="s2"/>`. Assert: both `sensor_dim == 3`, `sensor_adr[0] == 0`, `sensor_adr[1] == 3`. | | | |
| T11 | Invalid reftype → error → AC11. `<framepos site="s1" reftype="invalid" refname="s1"/>`. Assert: `load_model()` returns `Err` with message containing `"invalid reftype"`. | | | |
| T12 | Unknown refname → error → AC12. `<framepos site="s1" reftype="body" refname="nonexistent"/>`. Assert: `load_model()` returns `Err` with message containing `"unknown body"`. | | | |
| T13 | reftype without refname → no transform → AC13. `<framepos site="s1" reftype="body"/>`. Assert: `load_model()` succeeds, `sensor_reftype[0] == None`. After forward: output equals world-frame site position. | | | |
| T14 | All reftype strings resolve correctly → AC14. Model with 4 sensors, each using a different reftype: `"site"`, `"body"`, `"xbody"`, `"geom"`. Assert: `model.sensor_reftype` = `[Site, Body, XBody, Geom]`. | | | |
| T18 | FramePos body vs xbody numerical distinction → AC15. Body `bRef` with offset geom (`pos="0.3 0 0"`) so COM differs from joint origin. Object site on second body at `[0,1,0]`. Two sensors: `reftype="body"` (uses `xipos`/`ximat`) and `reftype="xbody"` (uses `xpos`/`xmat`). Assert: `(sensordata_body - sensordata_xbody).norm() > 0.01`. | | | |
| T19 | FrameQuat body vs xbody via `get_ref_quat` → AC17. Body `bRef` with offset geom so `body_iquat ≠ identity`, hinge at 90° about Z. Object body B at identity. Two sensors: `reftype="body"` (computes `xquat * body_iquat`) and `reftype="xbody"` (returns `xquat`). Assert: outputs differ `> 0.01`. | | | |

### Supplementary Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T15 | World body as reference — `reftype="xbody"`, `refname="world"` (body ID 0) → identity transform. Output should equal world-frame. Validates guard-free code path produces correct identity result. | | | |
| T16 | Same-body reference — object and reference are sites on the same body. Tests that relative position/orientation within a single rigid body is correctly computed. | | | |
| T17 | Sleeping body + awake reference — sleep filtering uses primary object's body, not reference. Validates sensor is skipped when primary is sleeping, regardless of reference state. | | | |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Spec Test(s) / AC(s) | Tested? | Test Function | Notes |
|-----------|----------|---------------------|---------|---------------|-------|
| No reftype/refname (world frame) | Regression — must not change existing behavior | T2, T8 / AC2, AC8 | | | |
| reftype=xbody with world body (`refname="world"`) | Identity transform — output == world frame. Not a special case in MuJoCo. | T15 / supplementary | | | |
| reftype=body vs reftype=xbody | Measurable difference on body with offset COM (`xipos`/`ximat` vs `xpos`/`xmat`) | T3, T18, T19 / AC3, AC15, AC17 | | | |
| reftype=geom | Uses `geom_xpos`/`geom_xmat` for position + rotation | T14 / AC14 | | | |
| reftype=site | Uses `site_xpos`/`site_xmat` for position + rotation | T4, T14 / AC4, AC14 | | | |
| FrameLinVel Coriolis term | Object and reference at different positions on rotating body; Coriolis correction `−w_ref × dif` required | T6 / AC6 | | | |
| FrameLinAcc/FrameAngAcc unaffected by refid | MuJoCo intentionally ignores `sensor_refid` for acc sensors — behavioral no-op | T9 / AC9 | | | |
| Cross-body reference | Reference and object on different bodies — distinct positions and velocities | T3, T6, T7 / AC3, AC6, AC7 | | | |
| Reference and object on same body | Relative offset within body — tests within-body relative computation | T16 / supplementary | | | |
| Sleeping primary body with awake reference | Sensor IS skipped (sleep check uses primary body, not reference) | T17 / supplementary | | | |
| sensor_dim/sensor_adr invariance | reftype must not change output dimensionality — still 3 or 4 components | T10 / AC10 | | | |
| reftype without refname | MuJoCo silently ignores (sets `refid = -1`) → no transform applied | T13 / AC13 | | | |
| Invalid reftype string | Error at build time — rejects typos in reftype attribute | T11 / AC11 | | | |
| Unknown refname | Error at build time — rejects references to nonexistent objects | T12 / AC12 | | | |

**Missing tests:**

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Old Behavior | New Behavior | Actually Happened? | Notes |
|-----------------|-------------|-------------|-------------------|-------|
| Frame sensors with `reftype`/`refname` MJCF attributes | Attributes silently ignored; output in world frame | Output in reference frame (relative measurement) — toward MuJoCo conformance | | |
| Builder reftype/refname resolution | Hardcoded `MjObjectType::None`/`0` for all sensors | Resolved from MJCF attributes via `resolve_reference_object()` | | |
| Invalid reftype string | Silently ignored (pushed None/0) | `ModelConversionError` with descriptive message | | |

### Files Affected: Predicted vs Actual

| Predicted File | Predicted Change | Est. Lines | Actually Changed? | Unexpected Files Changed |
|---------------|-----------------|-----------|-------------------|------------------------|
| `sim/L0/mjcf/src/builder/sensor.rs` | Replace hardcoded None/0 with `resolve_reference_object()` call; add new function | ~80 new, ~2 modified | | |
| `sim/L0/core/src/sensor/position.rs` | Add `get_ref_pos_mat()`, `get_ref_quat()` helpers; add ref-frame transform to FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis arms | ~60 new, ~15 modified | | |
| `sim/L0/core/src/sensor/velocity.rs` | Add `get_ref_pos_mat()` helper; add ref-frame transform to FrameLinVel, FrameAngVel arms | ~80 new, ~15 modified | | |
| `sim/L0/core/src/sensor/acceleration.rs` | **No change** — MuJoCo ignores refid for acc sensors | 0 | | |
| `sim/L0/tests/integration/sensor_phase6.rs` | New tests T21–T39 (continuing from Spec A's T1–T20) | ~500 new | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|

### Existing Test Impact: Predicted vs Actual

| Test | File / Line | Predicted Impact | Reason | Actual Impact | Surprise? |
|------|------------|-----------------|--------|---------------|-----------|
| `t01_parser_objtype_attribute` | `sensor_phase6.rs:16` | Pass (unchanged) | Tests parser, not builder/eval | | |
| `t02_parser_reftype_refname_separated` | `sensor_phase6.rs:47` | Pass (unchanged) | Tests parser, not builder/eval | | |
| `t03_parser_geom_attribute` | `sensor_phase6.rs:87` | Pass (unchanged) | Tests parser | | |
| `t04_builder_explicit_objtype_geom` | `sensor_phase6.rs:111` | Pass (unchanged) | No reftype in model | | |
| `t05_builder_body_xbody_distinction` | `sensor_phase6.rs:135` | Pass (unchanged) | No reftype in model | | |
| `t06_body_vs_xbody_position` | `sensor_phase6.rs:161` | Pass (unchanged) | No reftype → guard skips transform | | |
| `t07_body_framequat_mulquat` | `sensor_phase6.rs:216` | Pass (unchanged) | No reftype | | |
| `t08_default_inference_body_is_xbody` | `sensor_phase6.rs:280` | Pass (unchanged) | No reftype | | |
| `t09_touch_multi_geom_aggregation` | `sensor_phase6.rs:305` | Pass (unchanged) | Touch sensor, not frame sensor | | |
| `t10_touch_wrong_body_filtered` | `sensor_phase6.rs:381` | Pass (unchanged) | Touch sensor | | |
| `t11_touch_body_zero_geoms` | `sensor_phase6.rs:475` | Pass (unchanged) | Touch sensor | | |
| `t12_geom_framelinacc_matches_site` | `sensor_phase6.rs:506` | Pass (unchanged) | No reftype; acc stage unmodified | | |
| `t13_geom_frameangacc_static` | `sensor_phase6.rs:561` | Pass (unchanged) | No reftype; acc stage unmodified | | |
| `t14_geom_framelinacc_centripetal` | `sensor_phase6.rs:594` | Pass (unchanged) | No reftype | | |
| `t16_builder_regression_touch_as_site` | `sensor_phase6.rs:629` | Pass (unchanged) | Touch sensor | | |
| `t17_objtype_ignored_for_touch` | `sensor_phase6.rs:657` | Pass (unchanged) | Touch sensor | | |
| `t18_body_framepos_zero_mass` | `sensor_phase6.rs:683` | Pass (unchanged) | No reftype | | |
| `t19_geom_framelinvel_matches_site` | `sensor_phase6.rs:739` | Pass (unchanged) | No reftype | | |
| `t20_geom_frameangvel_matches_site` | `sensor_phase6.rs:789` | Pass (unchanged) | No reftype | | |
| Phase 5 test suite (2,238+ tests) | Various | Pass (unchanged) | No sensor evaluation paths modified for actuators | | |

> **Regression guarantee:** All existing tests (T1–T20) use no
> `reftype`/`refname` attributes → `sensor_reftype` remains
> `MjObjectType::None` → guard condition skips transform → existing
> behavior unchanged.

**Unexpected regressions:**

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `sensor_refid` disabled marker: `Vec<usize>`, guard via `sensor_reftype == MjObjectType::None` | Use `sensor_reftype[i] == MjObjectType::None` instead of `refid == -1` | | |
| `mjtObj` enum values: `MjObjectType` variants Body/XBody/Geom/Site; Camera → warn + skip (DT-117) | Map string → `MjObjectType` variant; camera → warn + skip (deferred) | | |
| `mju_mulMatTVec3(dst, mat, vec)`: row-major `mat^T * vec` | nalgebra column-major `mat.transpose() * vec` — same mathematical result | | |
| `mju_negQuat(dst, q)`: quaternion conjugate `[w, -x, -y, -z]` | `UnitQuaternion::inverse()` returns conjugate for unit quaternions | | |
| `cvel` spatial velocity layout: `[angular(3); linear(3)]` | `Vector3::new(cvel[0], cvel[1], cvel[2])` for angular | | |
| `get_xpos_xmat()` dispatch: per-type position + rotation source | Reference dispatch uses same arrays indexed by `sensor_reftype[i]`/`sensor_refid[i]` | | |
| `mj_objectVelocity()` `flg_local=0`: world-frame velocity | `object_velocity(data, body_id, &target_pos, None)` — `local_rot = None` for BOTH primary and reference | | |

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
| `Camera` variant in `MjObjectType` — `reftype="camera"` logs warning, falls through to `None` | Out of Scope, bullet 1 | | DT-117 | |
| Geom-attached velocity dispatch for FrameLinVel/FrameAngVel | Out of Scope, bullet 2 | | DT-118 | |
| FrameLinAcc/FrameAngAcc relative-frame — MuJoCo intentionally does not support | Out of Scope, bullet 3 | | — | |
| Sensor noise application — parsed/stored but not applied at runtime | Out of Scope, bullet 4 | | — | |
| Runtime sensor interpolation | Out of Scope, bullet 5 | | DT-107/DT-108 | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|

### Spec Gaps Found During Implementation

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
