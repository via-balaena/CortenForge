# Spec A — objtype + Touch Multi-Geom + Geom Acc: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_A.md`
**Implementation session(s):** Session 3
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
| Frame sensor objtype dispatch | Explicit `sensor_objtype[i]` set from MJCF `objtype` attribute, with `get_xpos_xmat()` / `get_xquat()` dispatch. 5 object types: `mjOBJ_XBODY`, `mjOBJ_BODY`, `mjOBJ_GEOM`, `mjOBJ_SITE`, `mjOBJ_CAMERA`. | No `objtype` attribute parsed. Builder guesses from name: site→body→geom heuristic. Only 3 types matched in evaluation. | | |
| `body` vs `xbody` distinction | `mjOBJ_BODY` reads `xipos`/`ximat` (COM/inertial frame); `mjOBJ_XBODY` reads `xpos`/`xmat` (joint frame). 0.15m difference for offset COM. | Only `MjObjectType::Body`, which reads `xpos`/`xmat` = MuJoCo's `mjOBJ_XBODY`. No inertial/COM frame option. | | |
| FrameQuat per objtype | `get_xquat()` computes per-type quaternion: `BODY` = `mulQuat(xquat, body_iquat)`, `GEOM` = `mulQuat(xquat[geom_bodyid], geom_quat)`, `SITE` = `mulQuat(xquat[site_bodyid], site_quat)`. | Site and Geom compute from rotation matrix via `from_matrix_unchecked`. Body reads `xquat` directly (= MuJoCo's XBODY behavior). | | |
| Touch sensor scope | Body-level: `site_bodyid[objid]` → iterate all contacts → `geom_bodyid[con->geom[k]] == bodyid`. ALL geoms contribute. | Single-geom: stores `(Geom, first_geom_id)`. Only contacts on first geom counted. | | |
| Touch force computation | `mj_contactForce()` reconstructs physical normal force from constraint basis. 33% higher than raw `efc_force` sum for pyramidal. | Reads `efc_force` directly. Sums all facet forces for pyramidal. Numerically wrong for pyramidal contacts. | | |
| Touch ray-geom filter | `mju_rayGeom()` filters contacts outside sensor site's volume. | No filter — all contacts on the geom summed. | | |
| Geom-attached FrameLinAcc | `mj_objectAcceleration(m, d, mjOBJ_GEOM, objid, tmp, 0)` → full spatial transport + Coriolis at geom position. | `_ => zeros`. Geom not matched. | | |
| Geom-attached FrameAngAcc | `mj_objectAcceleration(m, d, mjOBJ_GEOM, objid, tmp, 0)` → angular from body's `cacc[0..3]`. | `_ => zeros`. Geom not matched. | | |

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

### S1. Parser + types: `objtype` attribute, `geom=` attribute, `reftype`/`refname` separation (DT-62)

**Grade:**

**Spec says:**
Add `objtype`, `reftype`, `refname` fields to `MjcfSensor` struct. Parse `objtype` attribute, add `geom=` to the `objname` fallback chain, and separate `reftype`/`refname` into distinct fields (were conflated into one field). Preserve attribute provenance via builder-side inference when `objtype` is omitted.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Touch sensor keying change (DT-64)

**Grade:**

**Spec says:**
Change touch sensor builder to store `(Site, site_id)` instead of `(Geom, first_geom_id)`. Rewrite touch evaluation to resolve `site→body` at runtime via `model.site_body[objid]`, then iterate all contacts checking body-level membership via `model.geom_body[c.geom{1,2}] == body_id`. Continue reading `efc_force` directly (DT-118 deferred).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Builder: explicit objtype dispatch for frame sensors + `XBody` enum (DT-62)

**Grade:**

**Spec says:**
Add `XBody` variant to `MjObjectType` enum. Update `sensor_body_id()` to handle `XBody`. Rewrite frame sensor resolution in the builder: when `objtype` is present, dispatch explicitly (site/body/xbody/geom/camera); when absent, fall back to site→body(XBody)→geom name heuristic. Thread `objtype` through `process_sensors()` → `resolve_sensor_object()`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Evaluation: `XBody` + `Body` arms in frame sensors

**Grade:**

**Spec says:**
Add `XBody` and `Body` dispatch arms to every frame sensor evaluation site: FramePos (`xpos` for XBody, `xipos` for Body), FrameQuat (`xquat` direct for XBody, `xquat * body_iquat` for Body), FrameAxis (`xmat` for XBody, `ximat` for Body), FrameLinVel (`xpos` for XBody, `xipos` for Body), FrameAngVel (both read same `cvel`), Velocimeter (`xpos`/`xmat` for XBody, `xipos`/`ximat` for Body).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Evaluation: geom-attached FrameLinAcc/FrameAngAcc (DT-102)

**Grade:**

**Spec says:**
Add `Geom` arm to FrameLinAcc: resolve `body_id = model.geom_body[objid]`, `obj_pos = data.geom_xpos[objid]`, pass to `object_acceleration()`. Add `Geom` arm to FrameAngAcc: resolve `body_id = model.geom_body[objid]`, read `cacc[body_id]` angular components directly. Also add `XBody` and `Body` arms (XBody reads `xpos`, Body reads `xipos` for FrameLinAcc; both resolve to `objid` for FrameAngAcc).

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Parser stores `objtype` attribute | T1 | | |
| AC2 | Parser separates `reftype` and `refname` | T2 | | |
| AC3 | Parser handles `geom=` attribute | T3 | | |
| AC4 | Builder resolves explicit `objtype="geom"` to `MjObjectType::Geom` | T4 | | |
| AC5 | Builder resolves `objtype="body"` → Body and `objtype="xbody"` → XBody | T5 | | |
| AC6 | Body vs XBody position distinction (MuJoCo-verified, 0.15m COM offset) | T6 | | |
| AC7 | Body FrameQuat uses `mulQuat(xquat, body_iquat)` | T7 | | |
| AC8 | Default inference: `body="name"` without `objtype` → XBody | T8 | | |
| AC9 | Touch sensor multi-geom aggregation (MuJoCo-verified) | T9 | | |
| AC10 | Touch sensor ignores contacts on different bodies | T10 | | |
| AC11 | Touch sensor handles body with zero geoms | T11 | | |
| AC12 | Geom-attached FrameLinAcc (MuJoCo-verified, gravity-compensated) | T12 | | |
| AC13 | Geom-attached FrameAngAcc (static body → zeros) | T13 | | |
| AC14 | Geom-attached FrameLinAcc with offset geom (centripetal component) | T14 | | |
| AC15 | `MjObjectType::XBody` in enum (code review) | — (code review) | | |
| AC16 | Existing sensor tests pass (regression) | T15 | | |
| AC17 | Builder compiles and passes for existing MJCF models | T16 | | |
| AC18 | `objtype` ignored for non-frame sensors | T17 | | |
| AC19 | Body FramePos on zero-mass body | T18 | | |

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
| T1 | Parser `objtype` attribute → AC1. Parse `<framepos objtype="geom" objname="g1"/>`, assert `sensor.objtype == Some("geom")`. Parse without `objtype` → `None`. | | | |
| T2 | Parser `reftype`/`refname` separation → AC2. Parse with `reftype="body" refname="b1"`, assert both fields. Test partial: only `refname`, only `reftype`. | | | |
| T3 | Parser `geom=` attribute → AC3. Parse `<framepos geom="g1"/>`, assert `sensor.objname == Some("g1")`. | | | |
| T4 | Builder explicit `objtype="geom"` → AC4. Build with `<framepos objtype="geom" objname="g1"/>` and a geom `g1`. Assert `sensor_objtype[0] == Geom`. | | | |
| T5 | Builder `body`/`xbody` distinction → AC5. Both `objtype="body"` and `objtype="xbody"` on same body. Assert `Body` and `XBody` respectively. | | | |
| T6 | MuJoCo conformance — body vs xbody position → AC6. Body with capsule geom (COM offset 0.15m). Assert `xipos` vs `xpos` difference. Tol: 1e-10. | | | |
| T7 | Body FrameQuat uses `mulQuat` → AC7. Body with asymmetric geom, assert `sensordata = xquat * body_iquat`. Tol: 1e-10. | | | |
| T8 | Default inference `body=` → XBody → AC8. Build with `<framepos body="b1"/>` (no `objtype`). Assert `sensor_objtype[0] == XBody`. | | | |
| T9 | MuJoCo conformance — touch multi-geom → AC9. Body with 3 sphere geoms, 3 elliptic contacts. Expected: `≈ 29.43`. Tol: 1e-2. | | | |
| T10 | Touch — wrong body filtered → AC10. Two bodies, contacts on b2 only. Touch sensor on b1 = 0. | | | |
| T11 | Touch — body with zero geoms → AC11. Body with no geoms. Touch = 0.0. | | | |
| T12 | MuJoCo conformance — geom-attached FrameLinAcc → AC12. Static body, gravity. Expected: `[0, 0, 9.81]`. Tol: 1e-6. | | | |
| T13 | Geom-attached FrameAngAcc → AC13. Static body. Expected: `[0, 0, 0]`. Tol: 1e-10. | | | |
| T14 | Geom-attached FrameLinAcc with centripetal → AC14. Rotating body, geom at offset. Expected: `a_x ≈ -50.0`. Tol: 1e-6. | | | |
| T15 | Regression — existing sensor tests pass → AC16. Run full domain test suite. 0 failures. | | | |
| T16 | Builder regression — touch sensor builds as Site → AC17. Existing MJCF fixture builds with `sensor_objtype == Site`. | | | |
| T17 | Negative case — `objtype` ignored for Touch → AC18. `<touch site="s1" objtype="geom"/>` → `sensor_objtype == Site`. | | | |
| T18 | Edge case — Body FramePos on zero-mass body → AC19. Body with no geoms. `sensordata = xipos[b1]`. Tol: 1e-10. | | | |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| World body (`body_id == 0`) | `cacc[0]` is zero; sensors on world body sites produce correct values. | | | |
| Zero-mass body | `xipos`/`ximat` set to joint frame for massless bodies. Body-attached frame sensor reads inertial frame. | | | T18 covers this |
| Body with no geoms | Touch sensor on body with only sites → 0.0 (no contacts possible). | | | T11 covers this |
| Sleeping body | Touch sensor skipped (existing sleep check at `acceleration.rs:52–58`). | | | Covered by existing tests |
| `mjDSBL_SENSOR` flag | Early return, stale sensordata preserved (at `acceleration.rs:31–33`). | | | Covered by existing tests |
| Multi-geom body, contacts on all geoms | Touch sensor aggregates forces from all geoms. | | | T9 covers this |
| Contact on different body | Touch sensor excludes contacts from other bodies. | | | T10 covers this |
| Geom at offset position from body | FrameLinAcc includes spatial transport + Coriolis. | | | T14 covers this |
| `objtype` omitted | Builder falls back to name heuristic, `body=` → XBody. | | | T8 covers this |
| `objtype` on non-frame sensor | `objtype` silently ignored for Touch, Accelerometer, etc. | | | T17 covers this |
| `objtype="camera"` (unsupported) | Warning emitted, falls back to heuristic. | | | Deferred: DT-117 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T16 (builder regression) | Existing MJCF fixtures build correctly | Guards against builder changes breaking existing models |
| T17 (objtype ignored for Touch) | Non-frame sensor objtype silencing | Verifies pipeline dispatch table correctness — objtype only honored for frame sensors |

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
| Touch sensor objtype: `(Geom, first_geom_id)` → `(Site, site_id)` | | |
| Touch sensor scope: single-geom → all geoms on body contribute | | |
| Frame sensor `body=` heuristic: `MjObjectType::Body` → `MjObjectType::XBody` | | |
| Frame sensor `objtype="body"` explicit: not possible → `MjObjectType::Body` → reads `xipos`/`ximat` | | |
| Geom acc sensors: return zeros → return computed acceleration | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/parser.rs` | | |
| `sim/L0/mjcf/src/builder/sensor.rs` | | |
| `sim/L0/core/src/types/enums.rs` | | |
| `sim/L0/core/src/sensor/mod.rs` | | |
| `sim/L0/core/src/sensor/position.rs` | | |
| `sim/L0/core/src/sensor/velocity.rs` | | |
| `sim/L0/core/src/sensor/acceleration.rs` | | |
| `sim/L0/tests/integration/mjcf_sensors.rs` | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_touch_sensor_no_contact` | Needs update: `MjObjectType::Geom` → `MjObjectType::Site` | | |
| `test_touch_sensor_with_contact` | Needs update: geom ID → site ID, contact matching uses body-level check | | |
| `test_touch_sensor_*` (sim-sensor crate) | Pass (unchanged) — standalone crate tests don't use the pipeline | | |
| `t07_object_acceleration_static_gravity` | Pass (unchanged) — tests `object_acceleration()` directly | | |
| `t08_object_acceleration_centripetal` | Pass (unchanged) — tests `object_acceleration()` directly | | |
| Existing `sim-conformance-tests` | Pass — no frame sensor uses `objtype` attribute in existing test models | | |
| `mjcf_sensors.rs:71` | Needs update — assertion `sensor_objtype[0] == Body` → `XBody` | | |

**Unexpected regressions:**
{To be filled during review execution.}

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `mjOBJ_BODY` (1) — inertial/COM frame: reads `xipos`, `ximat` | Add `MjObjectType::XBody` for joint frame. Rename: current `Body` match arms that read `xpos`/`xmat` are semantically `XBody`. New `Body` arms read `xipos`/`ximat`. | | |
| `mjOBJ_XBODY` (2) — joint frame: reads `xpos`, `xmat` | `MjObjectType::XBody` = current `Body` data paths. MJCF `body="name"` (without `objtype`) defaults to `XBody`. | | |
| `sensor_objtype` array | `Vec<MjObjectType>` in `Model`. Direct port — same semantics. | | |
| `sensor_objid` array | `Vec<usize>` in `Model`. Direct port — same semantics. | | |
| `SpatialVector` layout | `cacc[body_id]` is `[f64; 6]`, `[0..3]` = angular, `[3..6]` = linear. Direct port. | | |
| `mj_contactForce()` | Deferred (DT-118). For frictionless: `efc_force[0]` IS normal force. For elliptic: first row IS normal force. For pyramidal: reconstruction needed (33% error). | | |
| Contact `con->frame[0..3]` (normal direction) | Use `contact.normal` wherever MuJoCo reads `con->frame[0..3]` for the normal direction. | | |
| `con->geom[0]`/`con->geom[1]` | `c.geom1` = `con->geom[0]`, `c.geom2` = `con->geom[1]`. Direct port. | | |
| `model.geom_bodyid` → `model.geom_body` | Use `model.geom_body` wherever MuJoCo uses `m->geom_bodyid`. | | |
| `model.site_bodyid` → `model.site_body` | Use `model.site_body` wherever MuJoCo uses `m->site_bodyid`. | | |
| Contact normal direction | MuJoCo `con->frame[0..3]` points geom2→geom1. Verify `Contact.normal` points same direction. Sign-flip depends on this. | | |
| Contact iteration pattern | CortenForge iterates `data.efc_type[ei]` forward, uses `data.efc_id[ei]` → `data.contacts[ci]`. Structurally inverted from MuJoCo but functionally equivalent. | | |

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
| `MjObjectType::Camera` — no `cam_xpos`/`cam_xmat`/`cam_quat` in Data yet | Out of Scope, bullet 1 | | DT-117 | |
| `mj_contactForce()` equivalent — touch reads `efc_force` directly, 33% error for pyramidal | Out of Scope, bullet 2 | | DT-118 | |
| Ray-geom intersection filter for touch — sums all contacts on body without spatial filtering | Out of Scope, bullet 3 | | DT-119 | |
| Frame sensor `reftype`/`refid` — relative-frame measurements (Spec B scope) | Out of Scope, bullet 4 | | DT-63 | |
| Sensor noise application — runtime noise not applied (RL training parity) | Out of Scope, bullet 5 | | — | |

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

**Overall:** {Ship / Fix then ship / Needs rework}

**Items to fix before shipping:**
1. {To be filled during review execution.}

**Items tracked for future work:**
1. DT-117 — `MjObjectType::Camera`
2. DT-118 — `mj_contactForce()` equivalent
3. DT-119 — Ray-geom intersection filter for touch
4. DT-63 — Frame sensor `reftype`/`refid` (Spec B)
