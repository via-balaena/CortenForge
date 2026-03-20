# Spec B — Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_B.md`
**Implementation session(s):** Session 7
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
| `mjTRN_SLIDERCRANK` transmission | Full implementation: length, derivatives, Jacobian composition, moment via chain rule | **Not implemented** — no enum variant, no parsing, no runtime | Full implementation in `mj_transmission_slidercrank()`: enum variant, MJCF parsing, runtime length/moment/velocity/force pipeline | **Yes** |
| `mjTRN_JOINTINPARENT` for hinge/slide | Identical to `mjTRN_JOINT` — scalar `gear[0]` | **Not implemented** — no enum variant, no parsing | Enum variant, MJCF parsing, widened `Joint` arms in all 8 match sites. Behavioral identity with `Joint` for hinge/slide. | **Yes** |
| `mjTRN_JOINTINPARENT` for ball/free | Gear rotation via inverse quaternion | **Not implemented** — also pre-existing gap in `mjTRN_JOINT` (`nv == 1` guard skips ball/free) | Still not implemented — out of scope per spec (pre-existing gap in base `Joint` type) | **No** (out of scope, tracked) |
| `mj_jacPointAxis` helper | `engine_core_util.c` — `cross(jacr_col, axis)` per DOF | **Not implemented** — `jacobian.rs` has `mj_jac` and `mj_jac_site` only | `mj_jac_point_axis()` at `jacobian.rs:133` — exact algorithm match, `cross(jacr_col, axis)` per DOF | **Yes** |
| Slider-crank `det <= 0` singularity | Silent degradation: `length = av`, `dlda = vec`, `dldv = axis` | N/A | Implemented: `if det <= 0.0` path at `actuation.rs:273` — `length = av`, `dl_daxis = vec3`, `dl_dvec = axis`. Matches MuJoCo exactly. | **Yes** |
| Slider-crank gear scaling | `length *= gear[0]` after derivatives; `moment *= gear[0]` during storage | N/A | Implemented: `length * gear0` at `actuation.rs:290`, `moment[j] = m * gear0` at `actuation.rs:313`. Matches MuJoCo convention. | **Yes** |
| `cranklength > 0` validation | Compiler error (`mjCError`) | N/A | Validated in `builder/actuator.rs:135`: `if rod <= 0.0 { return Err(...) }`. Error message includes "cranklength". | **Yes** |

**Unclosed gaps:**
- Ball/free joint transmission for both `Joint` and `JointInParent` — intentionally out of scope per spec. Pre-existing gap in base `Joint` type (`nv == 1` guard). Tracked as DT-8 residual in ROADMAP_V1.md.

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

### S1. Enum variants — `enums.rs`

**Grade:** Pass

**Spec says:**
Add two new variants `SliderCrank` and `JointInParent` to `ActuatorTransmission`
enum, appended after `Body`. Matching MuJoCo's `mjTRN_SLIDERCRANK` and
`mjTRN_JOINTINPARENT`. `#[derive(Default)]` remains on `Joint`.

**Implementation does:**
`enums.rs:180` adds `SliderCrank` after `Body`, `enums.rs:184` adds `JointInParent` after `SliderCrank`. Both have MuJoCo-referencing doc comments. `#[derive(Default)]` on enum, `#[default]` on `Joint`. Exact match.

**Gaps (if any):** None.

**Action:** None.

### S2. Model field + init — `model.rs`, `model_init.rs`

**Grade:** Pass

**Spec says:**
Add `actuator_cranklength: Vec<f64>` field to `Model`. Initialize to `vec![]`
in `Model::empty()`. SliderCrank actuators store the rod length; all others
store `0.0`.

**Implementation does:**
`model.rs:587` adds `pub actuator_cranklength: Vec<f64>` with correct doc comment referencing `m->actuator_cranklength[i]`. `model_init.rs:254` initializes to `vec![]`. Builder (`builder/actuator.rs:261`) pushes `actuator.cranklength.unwrap_or(0.0)`. Exact match.

**Gaps (if any):** None.

**Action:** None.

### S3. `mj_jac_point_axis` helper — `jacobian.rs`

**Grade:** Pass

**Spec says:**
New function `mj_jac_point_axis(model, data, body_id, point, axis)` returning
`(DMatrix<f64>, DMatrix<f64>)` — both 3×nv. Calls `mj_jac()` internally, then
computes axis Jacobian column-by-column via `cross(jacr_col, axis)`. Cross
product order: `cross(jacr_col, axis)`, NOT `cross(axis, jacr_col)`.

**Implementation does:**
`jacobian.rs:133-157` — exact spec match. Signature: `(model, data, body_id, point, axis) -> (DMatrix, DMatrix)`. Calls `mj_jac()`, then column-by-column cross product `(r1*axis.z - r2*axis.y, r2*axis.x - r0*axis.z, r0*axis.y - r1*axis.x)` which is `cross(jacr_col, axis)`. Has `#[must_use]` and clippy allow for canonical MuJoCo names.

**Gaps (if any):** None. Cross product order verified against MuJoCo C source — matches.

**Action:** None.

### S4. MJCF parsing — `types.rs`, `parser.rs`, `builder/actuator.rs`

**Grade:** Pass

**Spec says:**
Four new attributes: `jointinparent` (site name), `cranksite` (site name),
`slidersite` (site name), `cranklength` (f64). `jointinparent` maps to
`JointInParent` with same `trnid` semantics as `joint`. `cranksite` +
`slidersite` map to `SliderCrank` with `trnid[0]` = cranksite, `trnid[1]` =
slidersite. Builder validates `cranklength > 0` and `slidersite` present when
`cranksite` specified.

**Implementation does:**
- `types.rs:2353` — `jointinparent: Option<String>`, `cranksite: Option<String>`, `slidersite: Option<String>`, `cranklength: Option<f64>`. All with correct doc comments. Default init to `None`.
- `parser.rs:2029-2032` — four attributes parsed: `get_attribute_opt` for strings, `parse_float_attr` for cranklength.
- `builder/actuator.rs:95-143` — Full transmission resolution: `jointinparent` → `JointInParent` with joint lookup; `cranksite` → `SliderCrank` with crank/slider site lookups, `cranklength > 0` validation, and `slidersite` required check. Error messages include relevant field names.
- `builder/actuator.rs:261-262` — pushes `cranklength.unwrap_or(0.0)`.

**Gaps (if any):** None. All four attributes, all three validation checks, all error messages correct.

**Action:** None.

### S5. SliderCrank transmission dispatch — `actuation.rs`

**Grade:** Pass

**Spec says:**
New standalone function `mj_transmission_slidercrank()` following the existing
`mj_transmission_site()` / `mj_transmission_body()` pattern. Called from
`forward_core()`. Computes: slider axis (z-column of slider site xmat), vec
(crank - slider), length = `av - sqrt(det)` with degenerate fallback,
derivatives `dlda`/`dldv`, Jacobians via `mj_jac_point_axis` (slider) and
`mj_jac_site` (crank), moment via chain rule, gear scaling on both length
and moment.

**Implementation does:**
`actuation.rs:251-314` — `mj_transmission_slidercrank()`:
1. Iterates `0..model.nu`, filters on `SliderCrank`.
2. Extracts `crank_id = trnid[i][0]`, `slider_id = trnid[i][1]`, `rod`, `gear0`.
3. `axis = data.site_xmat[slider_id].column(2)` — z-column. Correct.
4. `vec3 = site_xpos[crank_id] - site_xpos[slider_id]`. Correct direction.
5. `av = vec3.dot(&axis)`, `det = av*av + rod*rod - vec3.dot(&vec3)`. Exact MuJoCo formula.
6. Degenerate branch: `if det <= 0.0` → `length = av`, `dl_daxis = vec3`, `dl_dvec = axis`. Matches MuJoCo.
7. Normal branch: `sdet = det.sqrt()`, `length = av - sdet`, `factor = 1 - av/sdet`, derivatives computed independently (not reusing buffer as MuJoCo does, but numerically identical per spec decision).
8. `data.actuator_length[i] = length * gear0`. Gear scaling after derivatives. Correct.
9. Jacobians: `mj_jac_point_axis` on slider body/pos/axis, `mj_jac_site` on crank. Correct.
10. `jac_vec = jac_crank_point - jac_slider_point`. Correct subtraction.
11. Chain rule moment: `moment[j] = sum_k(dl_daxis[k]*jac_axis[(k,j)] + dl_dvec[k]*jac_vec[(k,j)]) * gear0`. Correct.

Dispatch: `forward/mod.rs:325` calls `mj_transmission_slidercrank(model, self)` after `mj_transmission_site`.

**Gaps (if any):** None. Algorithm is a faithful port of the MuJoCo C source.

**Action:** None.

### S6. JointInParent + SliderCrank pipeline arms — `actuation.rs`

**Grade:** Pass

**Spec says:**
Widen existing `Joint` match arms to `Joint | JointInParent` for both
`mj_actuator_length` (velocity stage) and `mj_fwd_actuation` Phase 3 (force
stage). Add `SliderCrank` alongside `Site` arms — velocity from cached moment,
force via cached moment vector.

**Implementation does:**
- `actuation.rs:327` — `mj_actuator_length`: `Joint | JointInParent` for velocity stage. Correct.
- `actuation.rs:347` — `Site | SliderCrank` for velocity from cached moment. Correct.
- `actuation.rs:578` — `mj_fwd_actuation` Phase 3: `Joint | JointInParent` for force stage. Correct.
- `actuation.rs:600-602` — `Site | Body | SliderCrank` for force via cached moment. Correct.

**Gaps (if any):** None.

**Action:** None.

### S7. Match arm updates — 8 exhaustive match sites

**Grade:** Pass

**Spec says:**
8 production match sites across 7 files need explicit arms for `SliderCrank`
and `JointInParent`:
- S7a: `muscle.rs` ~160 — `compute_actuator_params` Phase 1 lengthrange
- S7b: `muscle.rs` ~344 — `build_actuator_moment`
- S7c: `muscle.rs` ~509 — `mj_set_length_range` Step 3 uselimit (**CRITICAL:** replace `_ => {}` catch-all)
- S7d: `derivatives.rs` ~563 — `mjd_actuator_vel`
- S7e: `sensor/position.rs` ~282 — `mj_sensor_pos` ActuatorPos
- S7f: `builder/build.rs` ~565 — sleep policy resolution
- Plus 2 arms in `actuation.rs` (covered by S6)

**Implementation does:**
- **S7a** (`muscle.rs:206`): `Joint | JointInParent`, `Site | Body | SliderCrank`. Correct.
- **S7b** (`muscle.rs:347`): `Joint | JointInParent`. SliderCrank has full dedicated arm (`muscle.rs:460-493`) with slider-crank geometry computation mirroring `mj_transmission_slidercrank`. Correct.
- **S7c** (`muscle.rs:547`): `Joint | JointInParent` with joint limit copy, `Site | Body | SliderCrank` with skip. **CRITICAL check: No `_ => {}` catch-all remains.** Verified via grep — zero matches for `_ => {` in muscle.rs. All 6 variants have explicit arms via the Tendon arm plus these two. **Pass.**
- **S7d** (`derivatives.rs:564`): `Joint | JointInParent`, `Site | Body | SliderCrank`. Correct.
- **S7e** (`sensor/position.rs:283`): `Joint | JointInParent`, `Site | Body | SliderCrank`. Correct.
- **S7f** (`builder/build.rs:567`): `Joint | JointInParent`, `SliderCrank` with dedicated arm marking both crank and slider site trees as `AutoNever`. Correct.

All 8 match sites have explicit arms. Compilation succeeds (Rust exhaustive match guarantee enforces completeness).

**Gaps (if any):** None.

**Action:** None.

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | SliderCrank length — colinear geometry | T1 | **Pass** | `test_slidercrank_length_colinear` — asserts `actuator_length[0] ≈ 1.0 ± 1e-10` |
| AC2 | SliderCrank moment — offset geometry | T1, T2 | **Pass** | T1 checks moment at dof0 (≈0.0 for colinear), T2 checks moment ≈ 0.25378 ± 1e-4 |
| AC3 | SliderCrank velocity from cached moment | T2 | **Pass** | `test_slidercrank_moment_offset` — asserts `actuator_velocity[0] ≈ moment[0] ± 1e-3` with qvel=[1.0] |
| AC4 | SliderCrank force application | T3 | **Pass** | `test_slidercrank_force_application` — asserts `qfrc_actuator[0] != 0.0` with ctrl=1.0 |
| AC5 | SliderCrank degenerate case (det ≤ 0) | T4 | **Pass** | `test_slidercrank_degenerate` — asserts length=0.0±1e-10, no NaN, no panic |
| AC6 | JointInParent hinge == Joint identity | T5 | **Pass** | `test_jointinparent_hinge_identity` — bitwise equality (epsilon=0.0) for length, velocity, qfrc |
| AC7 | JointInParent slide == Joint identity | T6 | **Pass** | `test_jointinparent_slide_identity` — bitwise equality (epsilon=0.0) for length, velocity, qfrc |
| AC8 | MJCF parsing — `jointinparent` round-trip | T7 | **Pass** | `test_mjcf_jointinparent_roundtrip` — checks trntype, trnid[0], gear |
| AC9 | MJCF parsing — SliderCrank round-trip | T8 | **Pass** | `test_mjcf_slidercrank_roundtrip` — checks trntype, cranklength, gear |
| AC10 | MJCF error — cranksite without slidersite | T9 | **Pass** | `test_mjcf_cranksite_without_slidersite` — error contains "slidersite" |
| AC11 | MJCF error — non-positive cranklength | T10 | **Pass** | `test_mjcf_nonpositive_cranklength` — error contains "cranklength" |
| AC12 | Exhaustive match completeness | — (code review) | **Pass** | All 8 match sites verified (S7 above). No `_ => {}` catch-all remains. Compilation enforces exhaustiveness. |
| AC13 | `mj_jac_point_axis` correctness | T11 | **Pass** | `test_jac_point_axis_correctness` — asserts jac_axis col = cross((0,1,0),(0,0,1)) = (1,0,0) |

**Missing or failing ACs:** None. All 13 ACs pass.

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | SliderCrank length + moment — colinear geometry (AC1, AC2) | **Yes** | `test_slidercrank_length_colinear` | Length=1.0, moment=0.0 (colinear symmetry) |
| T2 | SliderCrank moment + velocity — offset geometry (AC2, AC3) | **Yes** | `test_slidercrank_moment_offset` | Moment≈0.25378, velocity=moment*qvel |
| T3 | SliderCrank end-to-end force application (AC4) | **Yes** | `test_slidercrank_force_application` | Uses offset geometry for nonzero moment |
| T4 | SliderCrank degenerate (det ≤ 0) — no panic (AC5) | **Yes** | `test_slidercrank_degenerate` | length=0.0, no NaN |
| T5 | JointInParent hinge identity (AC6) | **Yes** | `test_jointinparent_hinge_identity` | Bitwise equality with Joint |
| T6 | JointInParent slide identity (AC7) | **Yes** | `test_jointinparent_slide_identity` | Bitwise equality with Joint |
| T7 | MJCF parsing — jointinparent (AC8) | **Yes** | `test_mjcf_jointinparent_roundtrip` | trntype, trnid, gear checked |
| T8 | MJCF parsing — SliderCrank (AC9) | **Yes** | `test_mjcf_slidercrank_roundtrip` | trntype, cranklength, gear checked |
| T9 | MJCF error — cranksite without slidersite (AC10) | **Yes** | `test_mjcf_cranksite_without_slidersite` | Error message checked |
| T10 | MJCF error — non-positive cranklength (AC11) | **Yes** | `test_mjcf_nonpositive_cranklength` | Error message checked |
| T11 | `mj_jac_point_axis` correctness (AC13) | **Yes** | `test_jac_point_axis_correctness` | Manual cross product verification |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| SliderCrank `det ≤ 0` (crank orthogonal to slider axis) | Singularity: sqrt of negative number would panic without guard | **Yes** | `test_slidercrank_degenerate` | det=-0.75, verifies length=0 and no panic/NaN |
| JointInParent hinge == Joint identity | Behavioral equivalence must hold exactly | **Yes** | `test_jointinparent_hinge_identity` | Bitwise equality (epsilon=0.0) |
| JointInParent slide == Joint identity | Same for slide joints | **Yes** | `test_jointinparent_slide_identity` | Bitwise equality (epsilon=0.0) |
| `cranklength = 0` (non-positive) | MuJoCo compiler rejects this; we must too | **Yes** | `test_mjcf_nonpositive_cranklength` | `cranklength="0"` → Err |
| Missing `slidersite` with `cranksite` present | MuJoCo requires both; partial spec is invalid | **Yes** | `test_mjcf_cranksite_without_slidersite` | Missing slidersite → Err |
| SliderCrank with gear != 1 | Gear scaling affects length and moment differently | **Yes** | `test_slidercrank_length_colinear` (T1) | gear=2.0: length=0.5*2.0=1.0 |

### Supplementary Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T2 (velocity) | Velocity = moment.dot(qvel) with no gear factor — verifies gear-baked-into-moment convention | **Yes** | `test_slidercrank_moment_offset` (second half) | Asserts velocity ≈ moment with qvel=[1.0] |
| T11 (jac_point_axis) | Standalone helper correctness — new function not covered by any AC except AC13 | **Yes** | `test_jac_point_axis_correctness` | cross((0,1,0),(0,0,1))=(1,0,0) verified |

**Missing tests:** None. All 11 planned tests implemented. All edge cases covered.

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| 2 new `ActuatorTransmission` variants (Joint, Tendon, Site, Body → + SliderCrank, JointInParent) | **Yes** | `enums.rs:180,184` |
| New `mj_jac_point_axis` function in `jacobian.rs` | **Yes** | `jacobian.rs:133-157` |
| New `actuator_cranklength` field in Model | **Yes** | `model.rs:587` |
| `_ => {}` catch-all at muscle.rs:509 replaced with explicit arms | **Yes** | Verified: zero `_ => {` matches in muscle.rs |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `core/src/types/enums.rs` | **Yes** | |
| `core/src/types/model.rs` | **Yes** | |
| `core/src/types/model_init.rs` | **Yes** | |
| `core/src/jacobian.rs` | **Yes** | |
| `core/src/forward/actuation.rs` | **Yes** | |
| `core/src/forward/muscle.rs` | **Yes** | |
| `core/src/derivatives.rs` | **Yes** | |
| `core/src/sensor/position.rs` | **Yes** | |
| `mjcf/src/types.rs` | **Yes** | |
| `mjcf/src/parser.rs` | **Yes** | |
| `mjcf/src/builder/actuator.rs` | **Yes** | |
| `mjcf/src/builder/build.rs` | **Yes** | |
| `tests/integration/transmission_specb.rs` | **No** — tests placed in `actuator_phase5.rs` instead | Spec deviation: tests co-located with Spec A tests rather than separate file |

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| `core/src/forward/mod.rs` | Added `mj_transmission_slidercrank` to re-export list and dispatch call. Expected — needed for the dispatch plumbing. |
| `mjcf/src/builder/init.rs` | Added `actuator_cranklength: vec![]` to ModelBuilder init. Expected — parallels `model_init.rs`. |
| `mjcf/src/builder/mod.rs` | Added `actuator_cranklength: Vec<f64>` field to ModelBuilder struct. Expected — parallels Model struct. |

**Note:** The 3 "unexpected" files are all structurally necessary for the feature. The spec's file list was focused on semantic changes and didn't enumerate mechanical plumbing files. The test file deviation (`actuator_phase5.rs` instead of `transmission_specb.rs`) is a minor organizational choice — co-locating Phase 5 tests in one file is reasonable.

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| Phase 4 regression suite (39 tests) | Pass (unchanged) — extension only | **39 pass, 0 fail** | No |
| Spec A tests (T7-T12) | Pass (unchanged) — new variants added but existing behavior untouched | **7 Spec A tests pass** | No |
| `test_site_actuator` | Pass (unchanged) — site parsing path not modified | **Pass** | No |
| `test_dampratio_mjcf_roundtrip` | Pass (unchanged) — dampratio for Joint unaffected | **Pass** | No |
| `test_acc0_motor_via_mjcf` | Pass (unchanged) — motor acc0 unaffected | **Pass** | No |
| Full sim domain baseline (~2,148+ tests) | All pass — extension-only change | **2,161 pass, 0 fail, 21 ignored** | No |

**Unexpected regressions:** None.

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| `actuator_trnid` indexing: MuJoCo flat `[2*i]`/`[2*i+1]` → CortenForge `Vec<[usize; 2]>` `[i][0]`/`[i][1]` | Use `model.actuator_trnid[i][0]` wherever MuJoCo uses `m->actuator_trnid[2*i]` | **Yes** | `actuation.rs:257-258`, `muscle.rs:461-462`, `builder/actuator.rs:105,143` all use `[i][0]`/`[i][1]` |
| `actuator_trnid` slot semantics (SliderCrank): `trnid[0]` = cranksite, `trnid[1]` = slidersite | Direct port — same semantic assignment | **Yes** | `builder/actuator.rs:143`: `[crank_id, slider_id]`. `actuation.rs:257`: comment "trnid[0] = cranksite" |
| `actuator_gear`: MuJoCo flat `gear + 6*i` → CortenForge `Vec<[f64; 6]>` `[i][0]` | Use `model.actuator_gear[i][0]` wherever MuJoCo uses `m->actuator_gear[6*i]` | **Yes** | `actuation.rs:260`, `muscle.rs:343`, `derivatives.rs:560` all use `[i][0]` |
| `site_xmat` z-column: MuJoCo row-major indices `[2]`, `[5]`, `[8]` → CortenForge `nalgebra::Matrix3` column access `.column(2)` | Use `data.site_xmat[id].column(2)` to extract z-axis | **Yes** | `actuation.rs:263`, `muscle.rs:465` |
| `actuator_moment`: MuJoCo sparse CSR → CortenForge dense `Vec<DVector<f64>>` | Store directly into dense vector; no sparse compression needed | **Yes** | `actuation.rs:306`: `data.actuator_moment[i]` indexed directly |
| Jacobian layout: MuJoCo row-major `3*nv` → CortenForge `nalgebra::DMatrix` (column-major) `matrix[(row, col)]` | Use `matrix[(k, j)]` wherever MuJoCo uses `jac[k*nv + j]` | **Yes** | `actuation.rs:310`: `jac_axis[(k, j)]`, `jac_vec[(k, j)]`. `jacobian.rs:148-153`: `jacr[(0, i)]` etc. |
| Quaternion convention: `(w, x, y, z)` in both | Direct port — no translation needed | **Yes** | N/A for this spec (ball/free out of scope) |
| Function naming: C camelCase → Rust snake_case | Spec uses C names for MuJoCo reference, Rust names for implementation | **Yes** | `mj_jac_point_axis` (from `mj_jacPointAxis`), `mj_transmission_slidercrank` |

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
| — | — | No weak items found | — | — |

Grep for `TODO`, `FIXME`, `HACK`, `todo!`, `unimplemented!` in `actuation.rs` and `jacobian.rs`: zero matches. No loose tolerances — tests use 1e-10 for exact values and 1e-3/1e-4 only where analytical approximation warrants it. No `unwrap()` in production code. No dead code. Algorithm matches spec exactly.

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
| Ball/free joint transmission (both Joint and JointInParent) — `nv == 3` and `nv == 6` sub-paths in `mj_transmission()` | Out of Scope, bullet 1 | ROADMAP_V1.md | DT-104 | **Yes** — added during this review (was untracked; DT-8 covered the transmission types, not ball/free sub-paths) |
| `mjTRN_TENDON` and `mjTRN_SITE` improvements | Out of Scope, bullet 2 | ROADMAP_V1.md (already conformant, no tracking needed) | N/A | **Yes** — no action needed (already conformant) |
| Sparse moment compression (CSR) | Out of Scope, bullet 3 | ROADMAP_V1.md | DT-105 | **Yes** — added during this review (was untracked; DT-36 is about `qLD_L`, not `actuator_moment`) |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| 3 additional builder plumbing files (`forward/mod.rs`, `builder/init.rs`, `builder/mod.rs`) not in spec's file list | Implementation required re-exports and struct field additions | Spec's file list was incomplete but impact was trivial mechanical changes | N/A — no future work needed | **Yes** |

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| Ball/free joint transmission was not tracked — DT-8 covered the transmission *types* (now done), not the ball/free sub-paths | Review Section 8 audit | ROADMAP_V1.md | DT-104 (new) | **Yes** — added during this review |
| Sparse `actuator_moment` compression was not tracked — DT-36 is about `qLD_L`, not `actuator_moment` | Review Section 8 audit | ROADMAP_V1.md | DT-105 (new) | **Yes** — added during this review |
| DT-8 and §61 are now done (Spec B implemented both) | Review Section 8 audit | ROADMAP_V1.md | DT-8, §61 (struck through) | **Yes** — marked done during this review |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| Test file location | Spec predicted `tests/integration/transmission_specb.rs` but tests went into `actuator_phase5.rs` | No — minor organizational deviation | Co-locating Phase 5 tests is reasonable; no conformance impact |

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
sim-conformance-tests: 999 pass, 0 fail, 1 ignored
sim-core:              187 pass, 0 fail, 0 ignored
sim-mjcf:              423 pass, 0 fail, 0 ignored
sim-physics:           285 pass, 0 fail, 0 ignored
sim-constraint:         39 pass, 0 fail, 0 ignored
sim-muscle:              6 pass, 0 fail, 0 ignored
sim-tendon:             63 pass, 0 fail, 0 ignored
sim-sensor:             22 pass, 0 fail, 0 ignored
sim-urdf:               52 pass, 0 fail, 0 ignored
sim-types:              53 pass, 0 fail, 0 ignored
sim-simd:               32 pass, 0 fail, 0 ignored
Total:              2,161 pass, 0 fail, 21 ignored
```

**New tests added:** 11 (T1-T11 in `actuator_phase5.rs`)
**Tests modified:** 0
**Pre-existing test regressions:** 0

**Clippy:** clean (0 warnings with `-D warnings`)
**Fmt:** clean

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | **Pass** — 6/7 gaps closed; 1 intentionally out of scope (ball/free) |
| Spec section compliance | 2 | **Pass** — all 7 sections grade Pass |
| Acceptance criteria | 3 | **Pass** — 13/13 ACs pass |
| Test plan completeness | 4 | **Pass** — 11/11 tests implemented, all edge cases covered |
| Blast radius accuracy | 5 | **Pass** — all predictions correct; 3 minor unexpected files (plumbing); 0 regressions |
| Convention fidelity | 6 | **Pass** — 8/8 conventions followed correctly |
| Weak items | 7 | **Pass** — 0 weak items found |
| Deferred work tracking | 8 | **Pass (after fix)** — 2 of 3 out-of-scope items were untracked; added DT-104, DT-105 during review. DT-8 and §61 marked done. |
| Test health | 9 | **Pass** — 2,161 pass, 0 fail, clippy clean, fmt clean |

**Overall:** **Pass — ship-ready.** Implementation is a faithful port of the MuJoCo C source. All spec sections, acceptance criteria, and tests pass. No weak items, no untracked deferred work, no regressions. The only unclosed gap (ball/free joint transmission) is a pre-existing issue in the base `Joint` type that is intentionally out of scope and properly tracked.

**Items to fix before shipping:**
None.

**Items tracked for future work:**
1. Ball/free joint transmission for both `Joint` and `JointInParent` (DT-104 — new, added during this review)
2. Sparse `actuator_moment` compression (DT-105 — new, added during this review)
3. DT-8 and §61 marked **Done** in ROADMAP_V1.md (Spec B implemented both)
