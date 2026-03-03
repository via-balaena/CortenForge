# Spec C — Parsing Breadth: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_C.md`
**Implementation session(s):** Session 15
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

The spec's Scope Adjustment section documents changes from the umbrella:

1. **§55 — Camera user data out of scope.** MuJoCo also supports
   `cam_user`/`nuser_cam` but the umbrella only lists 7 element types.
   Camera is the 8th. Tracked as DT-123.

2. **§55 — Sensor user already partially implemented.** Sensor user
   parsing exists in `parse_sensor_attrs` and sensor defaults user exists
   in `MjcfSensorDefaults`. Only the builder wiring to Model is missing.

3. **DT-88 — `<flexcomp>` not in MuJoCo 3.5.0 binary.** Verified
   empirically — produces "Schema violation: unrecognized element".
   dm_control schema and MuJoCo documentation are the source of truth.
   Documentation-fidelity only, not strict MuJoCo conformance.

4. **DT-88 — `inertiabox` is scalar.** dm_control schema says `float`
   (scalar), MuJoCo docs page says `real(3)`. Spec follows dm_control
   schema — `f64` scalar.

**Impact on review:** DT-88 items are documentation-fidelity, not
strict-conformance. Do NOT grade them against MuJoCo binary behavior.

---

## 1. Key Behaviors Gap Closure

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Body user data parsing | `<body user="1 2 3">` → `body_user[id]` zero-padded to `nuser_body` | **Missing** — no `user` attribute parsed in `parse_body_attrs` | | |
| Geom user data parsing | `<geom user="...">` → `geom_user[id]` | **Missing** — `parse_geom_attrs` | | |
| Joint user data parsing | `<joint user="...">` → `jnt_user[id]` | **Missing** — `parse_joint_attrs` | | |
| Site user data parsing | `<site user="...">` → `site_user[id]` | **Missing** — `parse_site_attrs` | | |
| Tendon user data parsing | `<tendon user="...">` → `tendon_user[id]` | **Missing** — `parse_tendon_attrs` | | |
| Actuator user data parsing | `<actuator user="...">` → `actuator_user[id]` | **Missing** — `parse_actuator_attrs` | | |
| Sensor user data parsing | `<sensor_type user="...">` → `sensor_user[id]` | **Parsed** but **not wired** to Model | | |
| `<size>` element parsing | `<size nuser_body="5" .../>` → sets nuser_* fields | **Missing** — `<size>` falls to `skip_element` | | |
| nuser_* auto-sizing | Max resolved user length across type | **Missing** — no nuser_* fields on Model | | |
| Zero-padding | Elements with shorter user data padded to nuser_* length | **Missing** | | |
| Validation | Error if effective user length > explicit nuser_* | **Missing** | | |
| Default class inheritance for user | 5 types: joint, geom, site, tendon, actuator | **Missing** for 5 types. Sensor defaults user already exists | | |
| Model *_user arrays | `Vec<Vec<f64>>` per element type | **Missing** — no *_user fields on Model | | |
| Flexcomp inertiabox | Scalar on `<flexcomp>` | **Missing** — `parse_flex_attrs` | | |
| Flexcomp scale | Vec3 applied to generated vertices | **Missing** | | |
| Flexcomp quat | Rotation quaternion applied after scale | **Missing** | | |
| Flexcomp file | Path string for mesh/grid data | **Missing** | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

| Grade | Meaning |
|-------|---------|
| **Pass** | Implementation matches spec. Algorithm, file location, edge cases all correct. |
| **Weak** | Implementation works but deviates from spec, uses shortcuts, has loose tolerances, missing edge-case guards, or TODOs. **Fix before shipping.** |
| **Deviated** | Implementation intentionally diverged from spec (spec gap discovered during implementation). Deviation is documented and justified. Acceptable if the spec was updated. |
| **Missing** | Section not implemented. Must be either fixed or explicitly deferred with tracking. |

### S1. `<size>` Element Parsing + MjcfModel nuser_* Fields

**Grade:**

**Spec says:**
Add 7 `nuser_*` fields (i32, default -1) to `MjcfModel`. Handle `<size>` in
both `Event::Start` (with `skip_element`) and `Event::Empty` (self-closing).
New `parse_size_attrs()` function using `parse_int_attr` for each field.
Multiple `<size>` elements merge with last-writer-wins per attribute.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. MJCF Type Structs — Add `user` Field to 6 Element Types

**Grade:**

**Spec says:**
Add `user: Vec<f64>` to MjcfBody, MjcfGeom, MjcfJoint, MjcfSite, MjcfTendon,
MjcfActuator. Default to `Vec::new()`. Use `Vec<f64>` (not `Option<Vec<f64>>`)
matching existing `MjcfSensor.user` pattern. Empty vec = "no user data."

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Parser — Add `user` Attribute Parsing to 6 Element Types

**Grade:**

**Spec says:**
Add `user` attribute parsing to `parse_body_attrs`, `parse_geom_attrs`,
`parse_joint_attrs`, `parse_site_attrs`, `parse_tendon_attrs`,
`parse_actuator_attrs`. Same pattern as sensor: `get_attribute_opt(e, "user")`
→ `parse_float_array` → assign to struct. `user=""` → empty vec (not specified).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Defaults — Add `user` to 5 Defaults Structs + Merge + Apply

**Grade:**

**Spec says:**
Step 4a: Add `user: Option<Vec<f64>>` to 5 defaults structs (Joint, Geom,
Site, Tendon, Actuator). Default `None`.
Step 4b: Parse `user` in 5 `parse_*_defaults()` functions. Only set
`Some(...)` for non-empty arrays (user="" → `None`, not `Some(vec![])`).
Step 4c: Merge logic: `user: c.user.clone().or_else(|| p.user.clone())` —
child-wins-if-present, parent-as-fallback (transitive inheritance).
Step 4d: Apply logic: `if result.user.is_empty() { use default }` — matching
existing `apply_to_sensor` pattern.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Model Struct — Add `*_user` and `nuser_*` Fields

**Grade:**

**Spec says:**
Add 7 `*_user: Vec<Vec<f64>>` arrays and 7 `nuser_*: i32` fields to Model.
Initialize in `Model::empty()`: `*_user: vec![]`, `nuser_*: 0` (resolved
value, not -1 sentinel).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Builder Wiring — Auto-Size, Validate, Pad, Store

**Grade:**

**Spec says:**
Step 6a: Add 7 `*_user_raw: Vec<Vec<f64>>` to ModelBuilder. World body gets
empty vec pushed in `new()`.
Step 6b: Push user data in each `process_*` function.
Step 6c: `finalize_user_data()` post-processing — for each type: if explicit
nuser >= 0, validate all lengths ≤ nuser (error if not); if nuser == -1,
auto-size as max length. Pad all arrays to nuser with zeros.
Step 6d: Transfer finalized user arrays AND computed `nuser_*` values to
Model in `build()` (`model.nuser_body = computed_nuser as i32`, etc. for
all 7 types).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Flexcomp Attributes — Parse, Store, Transform

**Grade:**

**Spec says:**
Step 7a: Add 4 fields to MjcfFlex: `inertiabox: f64` (default 0.0),
`flexcomp_scale: Option<Vector3<f64>>`, `flexcomp_quat: Option<UnitQuaternion<f64>>`,
`flexcomp_file: Option<String>`.
Step 7b: Parse 4 attributes in `parse_flex_attrs`.
Step 7c: `apply_flexcomp_transforms()` after vertex generation in both
`parse_flexcomp` and `parse_flexcomp_empty`. Order: scale first
(component-wise multiply), then quaternion rotation.

**Implementation does:**

**Gaps (if any):**

**Action:**

### Non-Modification Sites (verify NOT touched)

These 6 sensor-related sites already implement user data correctly. The spec
explicitly requires they are NOT modified by S1–S7. Verify none were touched.

| File:line (from spec) | What it does | Why NOT modified | Verified? |
|----------------------|-------------|-----------------|-----------|
| `defaults.rs:641` (`apply_to_sensor`) | Applies sensor defaults including user | Already handles user correctly | |
| `defaults.rs:945` (`merge_sensor_defaults`) | Merges sensor defaults including user | Already handles user correctly | |
| `parser.rs:3630` (`parse_sensor_attrs` user) | Parses sensor user attribute | Already implemented | |
| `parser.rs:962` (`parse_sensor_defaults` user) | Parses sensor defaults user | Already implemented | |
| `types.rs:3178` (`MjcfSensor.user`) | Sensor user field | Already exists | |
| `types.rs:815` (`MjcfSensorDefaults.user`) | Sensor defaults user field | Already exists | |

---

## 3. Acceptance Criteria Verification

| AC | Description | Test(s) | Coverage Type | Status | Notes |
|----|-------------|---------|---------------|--------|-------|
| AC1 | `<size>` nuser_* parsing | T1 | Direct | | |
| AC2 | User data auto-sizing | T2 | Direct | | |
| AC3 | Explicit nuser_* with padding | T3 | Direct | | |
| AC4 | Too-long user data error | T4 | Direct | | |
| AC5 | Default class inheritance | T5 | Direct | | |
| AC6 | No-user-data model | T6 | Direct | | |
| AC7 | World body user data (always zeros) | T2 | Edge case | | |
| AC8 | Sensor user data wired to Model | T7 | Direct | | |
| AC9 | `user=""` inherits default | T8 | Edge case | | |
| AC10 | `user="0"` is data, not absence | T8 | Edge case | | |
| AC11 | Actuator subtype defaults last-write-wins | T9 | Edge case | | |
| AC12 | Transitive 3-level default inheritance | T10 | Direct | | |
| AC13 | All 7 element types store user data | T11 | Integration | | |
| AC14 | Too-long inherited default → error | T12 | Edge case | | |
| AC15 | Explicit override avoids too-long default | T12 | Edge case | | |
| AC16 | Flexcomp scale + quat transform | T13 | Direct | | |
| AC17 | Flexcomp inertiabox and file parsed | T14 | Direct | | |
| AC18 | Multiple `<size>` elements merge | T1 | Edge case | | |
| AC19 | User data physics inertness | T15 | Direct | | |
| AC20 | `nuser_*=-1` triggers auto-sizing | T2 | Edge case | | |
| AC21 | `childclass` propagation with nearest-ancestor rule | T16 | Direct | | |
| AC22 | `childclass` does not shadow root default | T17 | Direct | | |
| AC23 | `nuser_*` strict integer validation | T18 | Direct | | |
| AC24 | `<freejoint>` does not accept `user` | — | Code review | | |
| AC25 | Existing tests pass | — | Code review | | |
| AC26 | Hex notation in user data is a known deviation | — | Code review | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

> **Test numbering note:** The spec uses T1–T18. Implementation test
> function naming convention to be determined during review execution.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | `<size>` element parsing (AC1, AC18) — multiple `<size>` elements merge with last-writer-wins | | | |
| T2 | Auto-sizing with world body and padding (AC2, AC7, AC20) — world body zeros, max-length auto-size, nuser=-1 sentinel | | | |
| T3 | Explicit nuser_* with zero-padding (AC3) — geoms with explicit nuser_geom=5, partial user data | | | |
| T4 | Too-long user data validation (AC4) — error when user length > nuser_* | | | |
| T5 | Default class inheritance with full replacement (AC5) — inherited vs overridden, full replacement not merge | | | |
| T6 | No-user-data model (AC6) — all nuser_*=0, zero-width inner vecs | | | |
| T7 | Sensor user data wired to Model (AC8) — existing parse now stored on Model | | | |
| T8 | user="" and user="0" semantics (AC9, AC10) — empty string inherits, "0" is data | | | |
| T9 | Actuator subtype defaults last-write-wins (AC11) — general then motor in same class | | | |
| T10 | Transitive 3-level default inheritance (AC12) — A→B→C chain propagation | | | |
| T11 | All 7 element types comprehensive (AC13) — single model with all types | | | |
| T12 | Too-long inherited default vs explicit override (AC14, AC15) — error on inherit, success on override | | | |
| T13 | Flexcomp scale + quat transform (AC16) — grid vertices transformed | | | |
| T14 | Flexcomp inertiabox and file (AC17) — parsed and stored | | | |
| T15 | User data physics inertness (AC19) — bit-identical qpos/qvel with and without user data | | | |
| T16 | `childclass` propagation with nearest-ancestor rule (AC21) — inner childclass overrides outer | | | |
| T17 | `childclass` does not shadow root default (AC22) — class chain inherits through non-overriding class | | | |
| T18 | `nuser_*` validation edge cases (AC23) — nuser=-2 error, nuser=0 with data error, nuser=0 without data success | | | |

### Supplementary Tests (from spec)

The spec's Supplementary Tests section lists T8, T15, T18 as supplementary
rationale (they cover edge cases beyond core requirements). These are already
in the planned tests table above — verify they were implemented with the
supplementary rationale in mind (parser semantics, physics inertness,
boundary conditions).

| Test | What it covers | Rationale (from spec) | Test Function | Implemented? |
|------|---------------|----------------------|---------------|-------------|
| T8 | Parser semantics for empty vs zero | Critical semantic distinction — prevents defaults cascade bugs | | |
| T15 | Runtime behavior with user data | Guarantees user data is truly read-only from physics perspective | | |
| T18 | Boundary conditions for nuser_* values | Prevents invalid nuser values from corrupting Model state | | |

### Supplementary Tests (from implementation/review)

Tests added beyond the spec's planned test list — additional coverage
discovered during implementation or review. These don't map 1:1 to ACs
but strengthen the test suite.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| {To be filled during review execution} | | | |

### Edge Case Inventory

| Edge Case | Spec Says | Spec Test(s) | Spec AC(s) | Tested? | Test Function | Notes |
|-----------|----------|-------------|-----------|---------|---------------|-------|
| World body user data (always zeros) | Body 0 is implicit — can't have user attr. Must be zeros. | T2 | AC7 | | | |
| No-user-data model (nuser=0) | Zero-width arrays must still be correct shape | T6 | AC6 | | | |
| Too-long user array error | Validation prevents memory overrun | T4 | AC4 | | | |
| Default inheritance with no override | Element omits `user` — inherits default class user data unchanged | T5 (j1) | AC5 | | | |
| Default inheritance with shorter override | Full replacement, not partial merge | T5 | AC5 | | | |
| Single-value user (`user="42"`) | Must produce 1-element array, not error | T5 (j2) | AC5 | | | |
| `user=""` with defaults (inherits) | Empty string = not specified, defaults cascade applies | T8 | AC9 | | | |
| `user="0"` is data, not absence | Semantic distinction: 0.0 is data, nuser=1 not 0 | T8 | AC10 | | | |
| `<size nuser_*="0">` explicit zero | Zero-width arrays are legal when no user data exists | T18 (C) | AC23 | | | |
| `<size nuser_*="0">` with user data | Must error — length > 0 exceeds explicit nuser=0 | T18 (B) | AC23 | | | |
| `nuser_*=-2` rejected | Only -1 is valid negative value | T18 (A) | AC23 | | | |
| Actuator subtype last-write-wins | MuJoCo aliases motor/general etc. within default class | T9 | AC11 | | | |
| Transitive 3-level inheritance | Chain A→B→C must propagate through non-overriding intermediates | T10 | AC12 | | | |
| `childclass` propagation nearest-ancestor | Inner childclass overrides outer for user data | T16 | AC21 | | | |
| `childclass` does not shadow root default | Class selects, does not block parent inheritance | T17 | AC22 | | | |
| `class="main"` references root default | Escape from childclass scope | T17 (variant) | AC22 | | | |
| Too-long inherited default | Validation on effective (post-cascade) data | T12 | AC14 | | | |
| Explicit override avoids too-long default | Override shields from default length check | T12 | AC15 | | | |
| Multiple `<size>` elements merge | Last-writer-wins per attribute | T1 | AC18 | | | |
| nuser_*=-1 explicit auto-size | Sentinel triggers same behavior as omitting | T2 | AC20 | | | |
| `<freejoint>` does not accept `user` | Reduced attribute set — use `<joint type="free">` | — (code review) | AC24 | | | |
| User data physics inertness | Must not affect simulation output | T15 | AC19 | | | |
| Depth-first element ordering in `*_user` arrays | body_user[0] is world, then DFS pre-order | T2, T11 | AC2, AC13 | | | |
| Multiple elements with different user lengths | Auto-sizing picks max, shorter arrays padded | T2 | AC2 | | | |
| Auto-sizing from defaults only (no inline user) | Default class user length drives nuser_* when elements inherit | T5 | AC5 | | | |
| NaN/Inf in user data | MuJoCo accepts with warning — Rust f64 parsing handles NaN/Inf | — | — | | | |
| Hex notation (`0xFF`) in user data | MuJoCo accepts via strtod; Rust f64::parse does not | — (known deviation) | AC26 | | | |
| Whitespace variants (tabs, newlines) | MuJoCo accepts standard XML whitespace separators | — | — | | | |
| Commas/semicolons rejected as separators | Only whitespace separators valid | — | — | | | |
| `nuser_*` with float value rejected | `nuser_geom="2.5"` must not parse as integer | T18 | AC23 | | | |
| Override is full replacement not partial merge | Element `user="1"` replaces default `user="10 20 30"` entirely | T5 | AC5 | | | |
| Denormal float values | MuJoCo rejects some; Rust may accept | — (known deviation) | — | | | |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `<size>` element parsing: silently skipped → parsed for nuser_* attributes | | |
| Body/geom/joint/site/tendon/actuator `user` attribute: silently ignored → parsed and stored | | |
| Model `*_user` arrays: don't exist → 7 new `Vec<Vec<f64>>` arrays | | |
| Model `nuser_*` fields: don't exist → 7 new `i32` fields | | |
| Sensor user data: parsed but discarded → parsed AND wired to Model | | |
| 5 defaults structs: no `user` field → `user: Option<Vec<f64>>` added | | |
| Flexcomp vertices: no scale/quat transform → scale then rotate applied | | |
| Too-long user data: silently accepted → error on build | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/mjcf/src/types.rs` | | |
| `sim/L0/mjcf/src/parser.rs` | | |
| `sim/L0/mjcf/src/defaults.rs` | | |
| `sim/L0/core/src/types/model.rs` | | |
| `sim/L0/core/src/types/model_init.rs` | | |
| `sim/L0/mjcf/src/builder/init.rs` | | |
| `sim/L0/mjcf/src/builder/body.rs` | | |
| `sim/L0/mjcf/src/builder/geom.rs` | | |
| `sim/L0/mjcf/src/builder/joint.rs` | | |
| `sim/L0/mjcf/src/builder/tendon.rs` | | |
| `sim/L0/mjcf/src/builder/actuator.rs` | | |
| `sim/L0/mjcf/src/builder/sensor.rs` | | |
| `sim/L0/mjcf/src/builder/mod.rs` | | |
| `sim/L0/mjcf/src/builder/build.rs` | | |
| Test files (new) | | |

{Unexpected files changed — to be filled during review execution:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| {To be filled during review execution} | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| All flexcomp tests | Pass (unchanged) — no existing tests use scale/quat/inertiabox/file | | |
| All sensor defaults tests | Pass (unchanged) — sensor user defaults code path not modified | | |
| All joint/geom/site/tendon/actuator defaults tests | Pass (unchanged) — new `user` field defaults to `None` | | |
| All body/geom/joint parsing tests | Pass (unchanged) — new `user` field defaults to empty vec | | |
| Model serialization tests | May need update if Model serialization includes new fields | | |

**Unexpected regressions:**
{To be filled during review execution.}

### Non-Modification Sites Verification

Cross-reference the Non-Modification Sites table from Section 2. Verify
none of the 6 sensor-related sites were modified during implementation.
This is a consolidated gate — Section 2 checks individual sites, this
section confirms the aggregate result.

| Site | Modified? | Notes |
|------|-----------|-------|
| `defaults.rs` — `apply_to_sensor` | | |
| `defaults.rs` — `merge_sensor_defaults` | | |
| `parser.rs` — `parse_sensor_attrs` user | | |
| `parser.rs` — `parse_sensor_defaults` user | | |
| `types.rs` — `MjcfSensor.user` | | |
| `types.rs` — `MjcfSensorDefaults.user` | | |

---

## 6. Convention Notes Audit

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| User data storage: flat stride `[id * nuser + k]` → `Vec<Vec<f64>>` `[id][k]` | Stride access → indexed access. All inner vecs must be uniform length for conformance equivalence. | | |
| `nuser_*` type: C `int` → `i32` | Direct port — same type | | |
| `nuser_*` compiled default: `≥ 0` after compilation | Builder resolves `-1` → auto-size; positive → use as-is | | |
| User data field naming: `body_user`, `jnt_user`, `geom_user`, `site_user`, `tendon_user`, `actuator_user`, `sensor_user` | Direct port — no translation needed | | |
| `nuser_*` field naming: `nuser_body`, `nuser_jnt`, `nuser_geom`, `nuser_site`, `nuser_tendon`, `nuser_actuator`, `nuser_sensor` | Direct port — no translation needed | | |
| Quaternion (flexcomp): `[w, x, y, z]` | Construct via `UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))` | | |
| Element indexing (user arrays): `body_user[body_id]`, etc. | Direct port — array position = element ID | | |
| Element ordering: depth-first pre-order for bodies; encounter order for geoms/joints/sites; XML document order for sensors/tendons/actuators | Direct port — builder already processes in this order | | |

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

**Severity guide:**
- **High** — Conformance risk. MuJoCo would produce different results.
  Fix before shipping.
- **Medium** — Code quality issue. Correct behavior but fragile, unclear,
  or not matching spec's prescribed approach. Fix if time permits, else
  track.
- **Low** — Style or minor robustness. No conformance risk. Track if not
  fixing now.

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| {To be filled during review execution} | | | | |

---

## 8. Deferred Work Tracker

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| Camera user data (`cam_user`/`nuser_cam`) | Out of Scope, bullet 1 | | DT-123 | |
| Flexcomp pos/axisangle/xyaxis/zaxis/euler/origin | Out of Scope, bullet 2 | | | |
| Flexcomp file loading | Out of Scope, bullet 3 | | | |
| nuserdata (global runtime scratch array) | Out of Scope, bullet 4 | | | |
| Hex notation in user data (`0xFF` via strtod) | Out of Scope, bullet 5 | | | |
| Denormal/subnormal float rejection | Out of Scope, bullet 6 | | | |
| `nuser_*` range validation (exact error message matching) | Out of Scope, bullet 7 | | | |
| User data mutability at runtime | Out of Scope, bullet 8 | | | |

### Discovered During Implementation

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| {To be filled during review execution} | | | | |

### Discovered During Review

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| {To be filled during review execution} | | | | |

### Spec Gaps Found During Implementation

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| {To be filled during review execution} | | | |

---

## 9. Test Coverage Summary

**Domain test results:**
```
{To be filled during review execution}
```

**New tests added:** {To be filled}
**Tests modified:** {To be filled}
**Pre-existing test regressions:** {To be filled}

**Clippy:** {To be filled}
**Fmt:** {To be filled}

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

**Overall:** {To be filled during review execution}

**Items fixed during review:**
{To be filled during review execution.}

**Items to fix before shipping:**
{To be filled during review execution.}

**Items tracked for future work:**
{To be filled during review execution.}
