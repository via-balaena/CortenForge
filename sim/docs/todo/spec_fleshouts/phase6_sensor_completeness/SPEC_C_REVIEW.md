# Phase 6 Spec C — Missing Sensor Types: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_C.md`
**Implementation session(s):** Session 13
**Reviewer:** AI agent
**Date:** 2026-03-01

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
| Clock reads simulation time | `sensordata[0] = d->time` in `mj_computeSensorPos()`, case `mjSENS_CLOCK` | Not implemented — no `MjSensorType::Clock` variant | | |
| JointActuatorFrc reads net actuator force at DOF | `d->qfrc_actuator[m->jnt_dofadr[objid]]` in `mj_computeSensorAcc()`, case `mjSENS_JOINTACTFRC` | Not implemented — no `MjSensorType::JointActuatorFrc` variant | | |
| JointActuatorFrc restricted to hinge/slide | MuJoCo compiler rejects ball/free joints: `"joint must be slide or hinge in sensor"` | Not implemented — no joint type validation | | |
| GeomDist/Normal/FromTo shared case block | Body-vs-geom dispatch via `body_geomnum`/`body_geomadr`, all-pairs `mj_geomDistance()`, per-type output dispatch | Not implemented — no variants, no `geom_distance()`, no body-geom dispatch | | |
| Dual-object resolution (geom1/body1 + geom2/body2) | Strict XOR per side, 4 combinations (geom×geom, geom×body, body×geom, body×body), `mjOBJ_BODY` (not XBODY) | Not implemented — no dual-object pattern in parser or builder | | |
| `mj_geomDistance()` signed distance with cutoff | Returns `(signed_distance, fromto[6])`. Positive for separation, negative for penetration. Cutoff caps search. | Not implemented — no `geom_distance()` function in CortenForge | | |
| Cutoff=0 suppresses positive distances but NOT penetration | `dist = cutoff = 0` init; only `dist_new < 0` (penetration) updates. Non-penetrating returns 0.0. | Not implemented | | |
| GeomFromTo cutoff exemption in postprocess | `apply_cutoff()` explicit early return: `type == mjSENS_GEOMFROMTO` → skip clamping. Cutoff used only as `mj_geomDistance()` search radius. | Not implemented — `apply_cutoff()` logic does not exist for this type | | |
| GeomNormal cutoff exemption via AXIS datatype | `apply_cutoff()` implicit skip: `mjDATATYPE_AXIS` has no matching branch in `if/else if` chain → elements not clamped | Not implemented — type does not exist | | |
| MJCF element names `distance`/`normal`/`fromto` | Parser recognizes `<distance>`, `<normal>`, `<fromto>` as sensor element names | Parser does not recognize these element names — unrecognized elements silently dropped | | |

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

### S1. Enum variants and dim()

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/types/enums.rs`. **MuJoCo equivalent:**
`mjSENS_CLOCK` (45), `mjSENS_JOINTACTFRC` (16), `mjSENS_GEOMDIST` (39),
`mjSENS_GEOMNORMAL` (40), `mjSENS_GEOMFROMTO` (41).
Add 5 new variants (`Clock`, `JointActuatorFrc`, `GeomDist`, `GeomNormal`,
`GeomFromTo`) to `MjSensorType` before `User` (which stays last by convention,
matching MuJoCo's `mjSENS_USER` pattern). Ordering follows umbrella spec's
Shared Convention Registry §2. `dim()` arms: Clock/JointActuatorFrc/GeomDist
→ 1, GeomNormal → 3, GeomFromTo → 6.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. `sensor_write6()` helper

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/sensor/postprocess.rs`. **MuJoCo equivalent:**
`mju_copy(sensordata, fromto, 6)` in `mj_computeSensorPos()`.
Add `sensor_write6()` after `sensor_write4` (line 34), following the existing
`sensor_write3()`/`sensor_write4()` pattern. Signature:
`pub fn sensor_write6(sensordata: &mut DVector<f64>, adr: usize, v: &[f64; 6])`.
Takes raw `&[f64; 6]` (not nalgebra type — no standard 6D vector type, and
introducing one for a single call site is over-engineering). Writes 6 elements
via individual `sensor_write()` calls.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. `geom_distance()` helper function

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/sensor/position.rs` or new module
`sim/L0/core/src/sensor/geom_distance.rs`. **MuJoCo equivalent:**
`mj_geomDistance()` in `engine_support.c`.
AD-1 option (c) from rubric — thin wrapper over existing collision
infrastructure. Algorithm:
1. Read world-frame poses from `data.geom_xpos`/`data.geom_xmat`.
2. Bounding sphere early-out via `model.geom_rbound` — skip if
   `center_dist - rbound1 - rbound2 > cutoff` (when cutoff > 0).
3. Analytic sphere-sphere fast path (exact, no GJK needed).
4. Convert to `CollisionShape` via `geom_to_collision_shape()` — returns
   `None` for Plane/Mesh/Hfield/SDF (deferred DT-122).
5. Build `Isometry3` from pose arrays.
6. GJK/EPA query: overlapping → negative penetration + fromto from contact
   normal; non-overlapping → `gjk_closest_points()` (new GJK distance
   extension).
Returns `(f64, [f64; 6])` — signed distance and fromto surface points.
Cutoff cap matches `mj_geomDistance()` return-value semantics.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. MjcfSensorType variants and MjcfSensor struct

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/types.rs`. **MuJoCo equivalent:** MJCF element
name dispatch in parser.
Add 5 variants (`Clock`, `Jointactuatorfrc`, `Distance`, `Normal`, `Fromto`)
to `MjcfSensorType` before `User` (line 2937). Add `from_str()` arms
(`"clock"`, `"jointactuatorfrc"`, `"distance"`, `"normal"`, `"fromto"`),
`as_str()` arms, and `dim()` arms (1D/1D/1D/3D/6D). Add 4 new fields to
`MjcfSensor` struct (after `user` field at line 3090): `geom1`, `geom2`,
`body1`, `body2` — all `Option<String>`, default to `None`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Parser — new element names and dual-object attributes

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/parser.rs`. **MuJoCo equivalent:** MJCF parser
element dispatch.
No changes to `parse_sensors()` itself — `MjcfSensorType::from_str()`
(updated in S4) handles new element name recognition. `parse_sensor_attrs()`
needs conditional parsing of `geom1`/`geom2`/`body1`/`body2` attributes for
distance/normal/fromto sensors (after `objname` chain at line 3469, before
`objtype` parsing at line 3472). Strict XOR validation per side: exactly one
of `{geom1, body1}` and exactly one of `{geom2, body2}` must be specified.
Both or neither on one side logs a warning: `"exactly one of (geom1, body1)
must be specified"`. Note: existing `objname` chain tries `body`/`geom`
attribute names, but `body1`/`geom1` are different attribute names — no
collision.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Builder — convert_sensor_type(), sensor_datatype(), resolve_sensor_object()

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/builder/sensor.rs`. **MuJoCo equivalent:** Sensor
compilation in `user_objects.cc`.
1. `convert_sensor_type()` (line 400): 5 new mappings (Clock→Clock,
   Jointactuatorfrc→JointActuatorFrc, Distance→GeomDist, Normal→GeomNormal,
   Fromto→GeomFromTo).
2. `sensor_datatype()`: Clock/GeomDist/GeomNormal/GeomFromTo → Position;
   JointActuatorFrc → Acceleration.
3. `resolve_sensor_object()` — early return at line 85: Clock (like User)
   returns `(MjObjectType::None, 0)` BEFORE the `objname` check at line 89
   (critical — Clock has no objname). JointActuatorFrc arm: look up joint
   name, validate hinge/slide only (reject ball/free with `"must be slide
   or hinge"`), return `(MjObjectType::Joint, id)`.
4. `process_sensors()` modification: branch BEFORE calling
   `resolve_sensor_object()` for GeomDist/Normal/FromTo — use new
   `resolve_dual_object_side()` helper to resolve geom1/body1 → obj and
   geom2/body2 → ref into `sensor_objtype`/`sensor_reftype`/`sensor_objid`/
   `sensor_refid` arrays. These types never reach `resolve_sensor_object()`.
5. New `resolve_dual_object_side()`: match on `(geom_name, body_name)` —
   `(Some, None)` → `(Geom, id)`, `(None, Some)` → `(Body, id)`, else error.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Position-stage evaluation — Clock + GeomDist/Normal/FromTo

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/sensor/position.rs`. **MuJoCo equivalent:**
`mj_computeSensorPos()` in `engine_sensor.c` — cases `mjSENS_CLOCK` and
`mjSENS_GEOMDIST`/`mjSENS_GEOMNORMAL`/`mjSENS_GEOMFROMTO`.
Add 2 match arms in `mj_sensor_pos()` before `_ => {}` wildcard:
1. **Clock:** `sensor_write(sensordata, adr, 0, data.time)`.
2. **GeomDist|GeomNormal|GeomFromTo** (combined arm, mirroring MuJoCo's
   fall-through):
   - Read `objtype`/`objid`/`reftype`/`refid`/`cutoff` from model arrays.
   - Body-vs-geom dispatch: `if objtype == Body → (body_geom_num, body_geom_adr)
     else → (1, objid)`. Same for ref side.
   - All-pairs minimum distance: nested `for geom1..for geom2`, call
     `geom_distance()`, update on strict `<` (first-found wins on ties,
     matching MuJoCo's `if (dist_new < dist)` guard).
   - Per-type output: GeomDist → `sensor_write(dist)`, GeomNormal → compute
     `normal = fromto[3..6] - fromto[0..3]`, zero-vector guard
     `if norm_squared > 0.0` then normalize, `sensor_write3`, GeomFromTo →
     `sensor_write6`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. Acceleration-stage evaluation — JointActuatorFrc

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/sensor/acceleration.rs`. **MuJoCo equivalent:**
`mj_computeSensorAcc()` case `mjSENS_JOINTACTFRC` in `engine_sensor.c`.
Add one match arm in `mj_sensor_acc()` before `_ => {}` wildcard:
`sensor_write(sensordata, adr, 0, data.qfrc_actuator[model.jnt_dof_adr[objid]])`.
Does NOT need body accumulators (`cacc`/`cfrc_int`) — reads directly from
`data.qfrc_actuator` which is populated during `mj_fwdActuation`. Does NOT
trigger lazy gate at line 63.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S9. Postprocess cutoff exemptions

**Grade:**

**Spec says:**
**File:** `sim/L0/core/src/sensor/postprocess.rs`. **MuJoCo equivalent:**
`apply_cutoff()` in `engine_sensor.c:64–89`.
AD-3 option (a) from rubric — explicit type match. Add skip guard between
lines 55–57 (after `cutoff > 0.0` check, before per-element loop): `if
matches!(sensor_type, GeomFromTo | GeomNormal) { continue; }`. This skips
the entire per-element loop for these types, matching MuJoCo's behavior:
- GeomFromTo: MuJoCo explicitly returns before clamping (`type ==
  mjSENS_GEOMFROMTO` → early return). Cutoff used only as search radius.
- GeomNormal: MuJoCo implicitly skips — `mjDATATYPE_AXIS` has no matching
  branch in `if/else if` chain (only REAL/POSITIVE get clamped).
CortenForge uses explicit type match for both since `MjSensorDataType`
stores pipeline stage, not data kind.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S10. Fusestatic body protection

**Grade:**

**Spec says:**
**File:** `sim/L0/mjcf/src/builder/compiler.rs`. **MuJoCo equivalent:** Body
fusion exclusion for sensor-referenced bodies.
Add `Distance | Normal | Fromto` arm in `apply_fusestatic()` before `_ => {}`
at line 102. Read `sensor.body1` and `sensor.body2` (new `MjcfSensor` fields
from S4), NOT `sensor.objname` — only body references need protection (geom
references don't need protection because geoms are never fused). Insert body
names into protected set.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | Clock reads `data.time` at t=0 → `sensordata[clock_adr] == 0.0` (exact) | T1 | | |
| AC2 | Clock multi-step: 3× `mj_step()` with `timestep=0.01` → `sensordata[clock_adr] == 0.02` (formula: `(3-1)*0.01`) | T2 | | |
| AC3 | JointActuatorFrc two-actuator net force: `ctrl=[3,4]`, `gear=[1,2]` → `sensordata[jaf_adr] == 11.0 ± 1e-12` (3×1 + 4×2) | T3 | | |
| AC4 | JointActuatorFrc zero-actuator case → `sensordata[jaf_adr] == 0.0` (exact) | T4 | | |
| AC5 | JointActuatorFrc rejects ball joint: builder returns error containing `"must be slide or hinge"` | T5 | | |
| AC6 | GeomDist sphere-sphere: g1(0,0,0 r=0.1), g2(1,0,0 r=0.2), cutoff=10 → `0.7 ± 1e-10` | T6 | | |
| AC7 | GeomNormal sphere-sphere: same geoms → `[1.0, 0.0, 0.0] ± 1e-10` | T7 | | |
| AC8 | GeomFromTo sphere-sphere: same geoms → `[0.1, 0, 0, 0.8, 0, 0] ± 1e-10` | T8 | | |
| AC9 | Cutoff=0 suppresses positive distance: non-overlapping spheres, no cutoff attr → `0.0` (exact) | T9 | | |
| AC10 | Cutoff=0 does NOT suppress penetration: g1(0,0,0 r=0.5), g2(0.3,0,0 r=0.5) → `-0.7 ± 1e-10` | T10 | | |
| AC11 | Multi-geom body: b1 has g1a(0,0,0 r=0.1)+g1b(0.3,0,0 r=0.1), b2 has g2(1,0,0 r=0.1) → `0.5 ± 1e-10` (min of g1b↔g2) | T11 | | |
| AC12 | GeomFromTo cutoff exemption: penetrating spheres (dist=-0.7) + cutoff=0.5 → fromto NOT clamped to [-0.5, 0.5], output = `[0.5, 0, 0, -0.2, 0, 0] ± 1e-10` | T12 | | |
| AC13 | GeomDist postprocess clamp: actual dist=-0.7, cutoff=0.5 → `-0.5 ± 1e-10` (clamped) | T13 | | |
| AC14 | GeomNormal cutoff exemption: penetrating (normal=[-1,0,0]), cutoff=0.5 → `[-1.0, 0.0, 0.0] ± 1e-10` (NOT clamped) | T14 | | |
| AC15 | Parser recognizes all 5 element names: `<clock>`, `<jointactuatorfrc>`, `<distance>`, `<normal>`, `<fromto>` → correct `MjcfSensorType` variants + `geom1`/`geom2` fields populated | T15 | | |
| AC16 | Strict XOR validation: both `geom1` and `body1` on side 1 → warning logged about `"exactly one of (geom1, body1)"` | T16 | | |
| AC17 | Code review: no `unsafe` blocks in new code | — (code review) | | |
| AC18 | Code review: wildcard audit — all 7 wildcard match sites from EGT-8 Table 3/4/5 updated: `from_str()`, `mj_sensor_pos()`, `mj_sensor_acc()`, `postprocess.rs` cutoff, `apply_fusestatic()` | — (code review) | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

> **Test numbering:** T1–T29 are Spec C's local labels. Implementation
> uses a separate test file `sensor_phase6_spec_c.rs` with its own
> numbering `t01_*` through `t26_*` (some spec tests may be combined
> or renumbered). Verify actual test function names during review
> execution.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | Clock at t=0 → AC1. Model: `<clock name="clk"/>`, `timestep=0.01`. Call `mj_forward()`. Assert `sensordata[0] == 0.0`. Expected value analytically derived (V1 from rubric EGT-6). | | | |
| T2 | Clock multi-step → AC2. Same model. Call `mj_step()` 3×. Assert `sensordata[0] == 0.02` (formula: `(N-1)*timestep = 2*0.01`). Analytically derived (V1). | | | |
| T3 | JointActuatorFrc two-actuator → AC3. Model: one hinge joint, two motors `gear=[1,2]`, `ctrl=[3,4]`. `<jointactuatorfrc name="jaf" joint="j1"/>`. Call `mj_step()`. Assert `sensordata[jaf_adr] == 11.0 ± 1e-12`. MuJoCo-verified (EGT-2), analytical backing (V2). | | | |
| T4 | JointActuatorFrc zero-actuator → AC4. Model: one hinge joint, no actuators. Call `mj_step()`. Assert `sensordata[jaf_adr] == 0.0`. Analytically derived (qfrc_actuator zeroed at reset). | | | |
| T5 | JointActuatorFrc ball joint rejection → AC5. Model: ball joint, `<jointactuatorfrc joint="ball_j"/>`. Assert builder returns `Err` containing `"must be slide or hinge"`. | | | |
| T6 | GeomDist sphere-sphere → AC6. Model: g1 at (0,0,0) r=0.1, g2 at (1,0,0) r=0.2. `<distance geom1="g1" geom2="g2" cutoff="10"/>`. Call `mj_forward()`. Assert `sensordata[d_adr] == 0.7 ± 1e-10`. MuJoCo-verified (EGT-3), analytical backing (V3). | | | |
| T7 | GeomNormal sphere-sphere → AC7. Same geoms. `<normal geom1="g1" geom2="g2" cutoff="10"/>`. Assert `sensordata[n_adr..+3] == [1.0, 0.0, 0.0] ± 1e-10`. MuJoCo-verified (EGT-3), analytical backing (V4). | | | |
| T8 | GeomFromTo sphere-sphere → AC8. Same geoms. `<fromto geom1="g1" geom2="g2" cutoff="10"/>`. Assert `sensordata[ft_adr..+6] == [0.1, 0, 0, 0.8, 0, 0] ± 1e-10`. MuJoCo-verified (EGT-3), analytical backing (V5). | | | |
| T9 | Cutoff=0 non-penetrating → AC9. Non-overlapping spheres. `<distance geom1="g1" geom2="g2"/>` (no cutoff). Assert `sensordata[d_adr] == 0.0`. MuJoCo-verified (EGT-3), analytical backing (V7). | | | |
| T10 | Cutoff=0 penetrating → AC10. Overlapping spheres (g1 r=0.5, g2 at 0.3 r=0.5). `<distance geom1="g1" geom2="g2"/>` (cutoff=0). Assert `sensordata[d_adr] == -0.7 ± 1e-10`. MuJoCo-verified (EGT-3), analytical backing (V6). | | | |
| T11 | Multi-geom body distance → AC11. b1 with g1a(0,0,0 r=0.1)+g1b(0.3,0,0 r=0.1), b2 with g2(1,0,0 r=0.1). `<distance body1="b1" body2="b2" cutoff="10"/>`. Assert `sensordata[d_adr] == 0.5 ± 1e-10`. MuJoCo-verified (EGT-3), analytical backing (V8). | | | |
| T12 | GeomFromTo cutoff exemption → AC12. Overlapping spheres (dist=-0.7), cutoff=0.5. `<fromto geom1="g1" geom2="g2" cutoff="0.5"/>`. Assert fromto = `[0.5, 0, 0, -0.2, 0, 0] ± 1e-10`. Postprocess does NOT clamp. MuJoCo-verified (EGT-3), analytical backing (V9). | | | |
| T13 | GeomDist postprocess clamp → AC13. Overlapping spheres (actual dist=-0.7), cutoff=0.5. `<distance geom1="g1" geom2="g2" cutoff="0.5"/>`. Assert `sensordata[d_adr] == -0.5 ± 1e-10`. MuJoCo-verified (EGT-3), analytical backing (V10). | | | |
| T14 | GeomNormal cutoff exemption → AC14. Overlapping spheres (normal=[-1,0,0]), cutoff=0.5. `<normal geom1="g1" geom2="g2" cutoff="0.5"/>`. Assert `sensordata[n_adr..+3] == [-1.0, 0.0, 0.0] ± 1e-10`. NOT clamped. MuJoCo-verified (EGT-3), analytical backing (V9). | | | |
| T15 | Parser recognizes all 5 element names → AC15. MJCF string with all 5 sensor elements. Parse and assert correct `MjcfSensorType` variants. Assert `geom1`/`geom2` fields populated for distance/normal/fromto. | | | |
| T16 | Strict XOR validation → AC16. Two cases: (a) MJCF with both `geom1` and `body1` on same side → warning logged. (b) MJCF with neither `geom1` nor `body1` → warning logged. | | | |

### Supplementary Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T17 | Beyond-cutoff asymmetry. g1 at (0,0,0) r=0.1, g2 at (10,0,0) r=0.1, cutoff=1. `<distance>` → 1.0, `<normal>` → [0,0,0], `<fromto>` → [0,0,0,0,0,0]. MuJoCo-verified (EGT-3). | | | |
| T18 | FromTo ordering (swap geom order). Same geoms as T8 but `geom1="g2" geom2="g1"`. Assert fromto = `[0.8, 0, 0, 0.1, 0, 0]` (swapped). MuJoCo-verified. | | | |
| T19 | Mixed objtype/reftype. `<distance geom1="g1" body2="b2" cutoff="10"/>`. Assert distance computed correctly using geom1 (single) × body2 (all geoms). | | | |
| T20 | Clock with cutoff. `<clock name="clk" cutoff="5"/>`, timestep=1. Step 10×. Assert sensordata clamped at 5.0 (postprocess clamps). MuJoCo-verified. | | | |
| T21 | JointActuatorFrc with cutoff. `<jointactuatorfrc joint="j1" cutoff="5"/>`, ctrl=10. Assert sensor = 5.0 (clamped from qfrc_actuator=10). MuJoCo-verified. | | | |
| T22 | Coincident geoms. g1 at (0,0,0) r=0.5, g2 at (0,0,0) r=0.5 (same pos, different IDs). `<distance geom1="g1" geom2="g2" cutoff="10"/>`. Assert `sensordata[d_adr] == -1.0 ± 1e-10` (full penetration: -2r). MuJoCo-verified. | | | |
| T23 | Body with zero geoms. Body b1 with no geoms (only child body), body b2 with geom g2. `<distance body1="b1" body2="b2" cutoff="10"/>`. Assert `sensordata[d_adr] == 10.0` (cutoff init — loop never executes). MuJoCo-verified. | | | |
| T24 | Self-distance rejection. `<distance geom1="g1" geom2="g1" cutoff="10"/>` (same geom both sides). Assert builder/compiler rejects with error containing `"1st body/geom must be different from 2nd"` (or equivalent). MuJoCo-verified. | | | |
| T25 | Body-pair uses direct geoms only (not subtree). b1 at (0,0,0) with direct geom g1_parent (r=0.1) + child body b1_child at (1.5,0,0) with g1_child (r=0.1). b2 at (2,0,0) with g2 (r=0.1). `<distance body1="b1" body2="b2" cutoff="10"/>`. Assert `sensordata[d_adr] == 1.8 ± 1e-10` (g1_parent↔g2, NOT g1_child↔g2 = 0.3). Validates body_geom_num/body_geom_adr dispatch. MuJoCo-verified. | | | |
| T26 | Same-body geoms. Body b1 with g1 at (0,0,0 r=0.1) and g2 at (0.5,0,0 r=0.1). `<distance geom1="g1" geom2="g2" cutoff="10"/>`. Assert `sensordata[d_adr] == 0.3 ± 1e-10`. MuJoCo-verified. | | | |
| T27 | Noise determinism. `<distance geom1="g1" geom2="g2" cutoff="10" noise="0.1"/>`. Call `mj_forward()` 2× with identical state. Assert sensordata bitwise identical. MuJoCo-verified. | | | |
| T28 | Non-sphere geom type (box-sphere). g1=box (size=0.5,0.5,0.5) at (0,0,0), g2=sphere (r=0.3) at (2,0,0). `<distance geom1="g1" geom2="g2" cutoff="10"/>`. Assert `sensordata[d_adr] == 1.2 ± 1e-6` (GJK numerical precision for non-sphere). MuJoCo-verified. | | | |
| T29 | Negative cutoff rejection. `<distance geom1="g1" geom2="g2" cutoff="-1"/>`. Assert builder/compiler rejects or warns. MuJoCo: `"negative cutoff in sensor"`. Note: may be deferred (see Out of Scope). | | | |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Test(s) / AC(s) | Tested? | Test Function | Notes |
|-----------|----------|-----------------|---------|---------------|-------|
| Cutoff=0 non-penetrating | Positive distances suppressed (non-obvious) | T9 / AC9 | | | |
| Cutoff=0 penetrating | Penetration NOT suppressed (asymmetry) | T10 / AC10 | | | |
| Multi-geom body | N×M all-pairs minimum | T11 / AC11 | | | |
| GeomFromTo cutoff exemption | Postprocess must skip this type | T12 / AC12 | | | |
| GeomNormal cutoff exemption | Postprocess must skip this type | T14 / AC14 | | | |
| GeomDist postprocess clamp | Penetration clamped to [-cutoff, cutoff] | T13 / AC13 | | | |
| Beyond-cutoff asymmetry | Dist→cutoff, normal→[0,0,0], fromto→zeros | T17 / supplementary | | | |
| FromTo swap ordering | From=geom1 surface, To=geom2 surface | T18 / supplementary | | | |
| Mixed geom/body | geom1+body2 and body1+geom2 valid | T19 / supplementary | | | |
| Clock cutoff clamping | Universal cutoff applies to Clock | T20 / supplementary | | | |
| JointActuatorFrc cutoff | Universal cutoff applies to JointActuatorFrc | T21 / supplementary | | | |
| Ball joint rejection | Compiler rejects non-hinge/slide joints | T5 / AC5 | | | |
| Zero-actuator joint | qfrc_actuator reads 0 | T4 / AC4 | | | |
| Strict XOR validation | Both or neither geom/body is parse error | T16 / AC16 | | | |
| Clock time lag | After N steps, sensor reads (N-1)*timestep | T2 / AC2 | | | |
| Coincident geoms | Full penetration = -2r, arbitrary normal | T22 / supplementary | | | |
| Body with zero geoms | Loop never executes, returns cutoff init | T23 / supplementary | | | |
| Self-distance rejection | Compiler rejects geom1==geom2 | T24 / supplementary | | | |
| Body-pair direct geoms only | Uses body_geom_num, NOT subtree | T25 / supplementary | | | |
| Same-body geoms | Two geoms on same body is valid | T26 / supplementary | | | |
| Noise determinism | Noise is metadata-only, not applied at runtime | T27 / supplementary | | | |
| Non-sphere geom types | geom_distance works with box, capsule, etc. | T28 / supplementary | | | |
| Negative cutoff rejection | Compiler rejects negative cutoff values | T29 / supplementary | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Old Behavior | New Behavior | Actually Happened? | Notes |
|-----------------|-------------|-------------|-------------------|-------|
| 5 new `MjSensorType` variants (32 → 37) | Enum has 32 variants | Enum has 37 variants (Clock, JointActuatorFrc, GeomDist, GeomNormal, GeomFromTo) | | |
| 5 new `MjcfSensorType` variants (31 → 36) | Enum has 31 variants | Enum has 36 variants (Clock, Jointactuatorfrc, Distance, Normal, Fromto) | | |
| 4 new `MjcfSensor` fields (9 → 13) | Struct has 9 fields | Struct has 13 fields (+ geom1, geom2, body1, body2) | | |
| `sensor_write6()` added | No 6D write helper | `sensor_write6()` available in `postprocess.rs` | | |
| `geom_distance()` added | No geom distance function | `geom_distance()` available with sphere-sphere fast path + GJK fallback | | |
| GeomFromTo/GeomNormal cutoff exemption | Cutoff applies to all non-Touch/Rangefinder sensors | Cutoff skips GeomFromTo and GeomNormal via explicit type match | | |
| GJK distance extension | GJK returns bool only (intersection) | GJK returns distance + closest points via `gjk_closest_points()` | | |

### Files Affected: Predicted vs Actual

| Predicted File | Predicted Change | Est. Lines | Actually Changed? | Unexpected Files Changed |
|---------------|-----------------|-----------|-------------------|------------------------|
| `core/src/types/enums.rs` | 5 new `MjSensorType` variants + `dim()` arms | +15 | | |
| `core/src/sensor/postprocess.rs` | `sensor_write6()` + cutoff exemption guard | +20 | | |
| `core/src/sensor/position.rs` | Clock arm + GeomDist/Normal/FromTo combined arm | +60 | | |
| `core/src/sensor/acceleration.rs` | JointActuatorFrc arm | +6 | | |
| `core/src/gjk_epa.rs` | `gjk_closest_points()` — GJK distance extension | +80 | | |
| `core/src/sensor/geom_distance.rs` (new) | `geom_distance()` helper | +120 | | |
| `mjcf/src/types.rs` | 5 `MjcfSensorType` variants + methods + 4 `MjcfSensor` fields | +30 | | |
| `mjcf/src/parser.rs` | Dual-object attribute parsing + XOR validation | +25 | | |
| `mjcf/src/builder/sensor.rs` | `convert_sensor_type()`, `sensor_datatype()`, `resolve_sensor_object()`, `resolve_dual_object_side()`, `process_sensors()` mod | +60 | | |
| `mjcf/src/builder/compiler.rs` | `apply_fusestatic()` body1/body2 protection | +8 | | |
| Test files (new) | T1–T29 | +400 | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test Group | File / Count | Predicted Impact | Reason | Actual Impact | Surprise? |
|-----------|-------------|-----------------|--------|---------------|-----------|
| sim-core sensor unit tests | `core/src/sensor/mod.rs` (32 tests) | Pass — no overlap with new types | Touch/Magnetometer/Actuator/Rangefinder/Force/Accelerometer arms unchanged; postprocess cutoff `continue` only fires for new GeomFromTo/GeomNormal types | | |
| Phase 4 integration tests | `sensors_phase4.rs` (44 tests) | Pass — no overlap with new types | Accelerometer/FrameLinAcc/Force/Velocimeter/SubtreeLinVel arms all unchanged | | |
| Phase 6 Spec A+B tests | `sensor_phase6.rs` (39 tests) | Pass — additive changes only | Parser changes add new element names (don't modify existing). Builder changes add new type mappings (compiler enforces exhaustive). Evaluation adds new match arms before wildcard. | | |
| MJCF sensor tests | `mjcf_sensors.rs` (8 tests) | Pass — additive parsing | New element names don't affect existing JointPos/Gyro/Touch/etc. parsing | | |
| sim-sensor crate tests | `sensors.rs` (11 tests) | Pass — independent crate | sim-sensor tests don't use sim-core sensor evaluation pipeline | | |

**Compile-time impact note:** Adding 5 new `MjSensorType` variants causes
compile errors in all exhaustive matches (`dim()`, `convert_sensor_type()`,
`sensor_datatype()`, `resolve_sensor_object()`). Expected and desirable.
Non-exhaustive matches (wildcards in `from_str()`, `mj_sensor_pos()`,
`mj_sensor_acc()`, `postprocess.rs`, `apply_fusestatic()`) do NOT error —
these are the danger zones requiring manual audit (AC18).

**Unexpected regressions:**
{To be filled during review execution.}

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| MJCF `<distance>` → `MjcfSensorType::Distance` → `MjSensorType::GeomDist` | MJCF element `"distance"` maps to `Distance` in parser, then to `GeomDist` in builder | | |
| MJCF `<normal>` → `MjcfSensorType::Normal` → `MjSensorType::GeomNormal` | MJCF element `"normal"` maps to `Normal`, then to `GeomNormal` in builder | | |
| MJCF `<fromto>` → `MjcfSensorType::Fromto` → `MjSensorType::GeomFromTo` | MJCF element `"fromto"` maps to `Fromto`, then to `GeomFromTo` in builder | | |
| MJCF `<clock>` → `MjcfSensorType::Clock` → `MjSensorType::Clock` | Direct port — `"clock"` maps straight through | | |
| MJCF `<jointactuatorfrc>` → `MjcfSensorType::Jointactuatorfrc` → `MjSensorType::JointActuatorFrc` | Direct port — `"jointactuatorfrc"` maps through | | |
| `MjObjectType::None` for `mjOBJ_UNKNOWN` | Use `MjObjectType::None` wherever MuJoCo uses `mjOBJ_UNKNOWN` (Clock objtype/reftype, JointActuatorFrc reftype) | | |
| `body_geomnum` → `model.body_geom_num[bodyid]` | CortenForge uses snake_case: `body_geom_num` not `body_geomnum` (`model.rs:125`) | | |
| `body_geomadr` → `model.body_geom_adr[bodyid]` | CortenForge uses snake_case: `body_geom_adr` not `body_geomadr` (`model.rs:123`) | | |
| `jnt_dofadr` → `model.jnt_dof_adr[objid]` | CortenForge uses snake_case: `jnt_dof_adr` not `jnt_dofadr` (`model.rs:171`) | | |
| `qfrc_actuator` → `data.qfrc_actuator[...]` | Direct port — same semantics, nalgebra `DVector` (`data.rs:51`) | | |
| `d->time` → `data.time` | Direct port (`data.rs:499`, `f64`) | | |
| `d->geom_xpos` → `data.geom_xpos[geom_id]` | Direct port — `Vec<Vector3<f64>>` (`data.rs:107`) | | |
| `d->geom_xmat` → `data.geom_xmat[geom_id]` | MuJoCo flat 9-element → nalgebra `Matrix3` (`data.rs:109`) | | |
| `m->geom_rbound` → `model.geom_rbound[geom_id]` | Direct port — `Vec<f64>` (`model.rs:291`) | | |
| `sensor_write` helpers for 1D/3D output | Use `sensor_write` for 1D, `sensor_write3` for 3D (`postprocess.rs:12/21`) | | |
| 6D output (fromto) → `sensor_write6()` | New helper following `sensor_write3`/`sensor_write4` pattern (`postprocess.rs`) | | |
| `MjSensorDataType` is pipeline stage, NOT MuJoCo's data kind | Postprocess cutoff exemptions must use explicit sensor type matching, not datatype checks. CortenForge's `MjSensorDataType` = `Position`/`Velocity`/`Acceleration`, NOT `REAL`/`POSITIVE`/`AXIS`/`QUATERNION`. | | |
| `geom_distance()` return signature | Rust: `fn geom_distance(...) -> (f64, [f64; 6])`. MuJoCo: returns signed distance, populates `fromto` array. | | |
| nalgebra types for output | Normal output uses `Vector3` → `sensor_write3`. Fromto uses `[f64; 6]` → `sensor_write6` (no standard 6D nalgebra type). | | |

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
| `CamProjection` sensor — requires camera infrastructure (`cam_xpos`, `cam_xmat`, `cam_resolution`, `cam_fovy`, `cam_intrinsic`, `cam_sensorsize`, `MjObjectType::Camera`) | Out of Scope, bullet 1 | | DT-120 | |
| `InsideSite` sensor (`mjSENS_INSIDESITE`) — exists in MuJoCo 3.5.0 but not listed in umbrella spec, requires geometric containment testing | Out of Scope, bullet 2 | | DT-121 | |
| Mesh/Hfield/SDF geom distance — `geom_distance()` supports convex primitives only; non-convex pairs return `(cutoff, [0; 6])` with warning | Out of Scope, bullet 3 | | DT-122 | |
| Sensor noise application — `noise` parsed and stored but not applied at runtime (intentional for RL training parity) | Out of Scope, bullet 4 | | — | |
| Runtime sensor interpolation — `nsample`/`interp`/`delay` runtime behavior | Out of Scope, bullet 5 | | DT-107/DT-108 | |
| Performance optimization — Phase 6 is correctness/completeness, not performance | Out of Scope, bullet 6 | | — | |
| `GeomPoint` sensor — umbrella listed but MuJoCo 3.5.0 does not have `<geompoint>` element | Out of Scope, bullet 7 | | — (does not exist) | |
| Negative cutoff validation — MuJoCo compiler rejects `"negative cutoff in sensor"`, CortenForge does not validate at parse/compile time | Out of Scope, bullet 8 | | — | |

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

**Clippy:** {clean / N warnings — list them}
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
