# Phase 6 — Sensor Completeness: Umbrella Spec

**Status:** Draft
**Phase:** Roadmap Phase 6
**Tasks:** 6 (§62, DT-62, DT-63, DT-64, DT-102, DT-109)
**Deliverables:** 4 sub-specs (Spec A, B, C, D)
**Test baseline:** 2,238+ domain tests (post-Phase 5)

---

## Scope

Phase 6 closes all sensor-related conformance gaps in the v1.0 roadmap.
**The goal is MuJoCo conformance** — after Phase 6, CortenForge produces
numerically identical results to MuJoCo for every supported sensor feature:
frame sensor object type resolution, relative-frame measurements, multi-geom
touch aggregation, geom-attached acceleration sensors, missing sensor types,
and sensor history attributes.

This umbrella spec coordinates the 4 sub-specs that comprise Phase 6. It
defines implementation order, file ownership, API contracts, and shared
conventions so the sub-specs can be written and implemented without
coordination conflicts.

> **Conformance mandate for sub-spec authors:** Each sub-spec must cite the
> exact MuJoCo C source (function, file, line range) for every behavior it
> implements. Acceptance criteria must include expected values derived from
> MuJoCo's output — not from hand-calculation or intuition. The MuJoCo C
> source code is the single source of truth. When the MuJoCo docs and the
> C source disagree, the C source wins.

---

## Task Assignment

Every Phase 6 task is assigned to exactly one deliverable. No orphans, no
overlaps. Each task closes a specific MuJoCo conformance gap.

| Task | Description | Deliverable | Rationale |
|------|-------------|-------------|-----------|
| DT-62 | Frame sensor `objtype` attribute not parsed — fallback heuristic used | **Spec A** | Enables explicit objtype dispatch; prerequisite for DT-102 |
| DT-64 | Multi-geom touch sensor aggregation (only first geom matched) | **Spec A** | Same parse→wire→test pattern as DT-62 |
| DT-102 | Geom-attached acc-stage sensors (FrameLinAcc/FrameAngAcc with `MjObjectType::Geom`) | **Spec A** | Depends on DT-62's objtype parsing; same evaluation files |
| DT-63 | Frame sensor `reftype`/`refid` — relative-frame measurements | **Spec B** | Architectural — requires reference frame transform in all 3 sensor stages |
| §62 | Missing sensor types: `clock`, `jointactuatorfrc`, `camprojection`, `geomdist`, `geompoint`, `geomnormal` | **Spec C** | Adds new enum variants + evaluation arms |
| DT-109 | Sensor history attributes: `nsample`/`interp`/`delay` on sensors | **Spec D** | Independent parsing — mirrors actuator history from Phase 5 Spec D |

### Sub-spec scope statements

Each sub-spec must identify the exact MuJoCo C functions it is porting, cite
them with source file and line ranges, and produce acceptance criteria with
MuJoCo-verified expected values.

**Spec A — objtype parsing, touch multi-geom, geom acc sensors** (DT-62, DT-64, DT-102):
Builder-level sensor object resolution and evaluation-level fixes. DT-62 adds
explicit `objtype` attribute parsing in the MJCF parser and builder, replacing
the current site→body→geom fallback heuristic for frame sensors. DT-64 changes
touch sensor resolution from single-geom keying to body-level aggregation
(all geoms on the sensor's body contribute). DT-102 wires geom-attached
FrameLinAcc/FrameAngAcc to compute object acceleration at the geom position
using `mj_objectAcceleration()` helpers (already landed in DT-103).
MuJoCo reference: `mj_sensorAcc()` in `engine_sensor.c`, `mj_sensorPos()` in
`engine_sensor.c`, `mj_transmission()` touch geom resolution in `engine_forward.c`.

**Spec B — relative-frame measurements** (DT-63):
Wires `reftype`/`refid` for frame sensors so measurements can be expressed
relative to a reference object (site, body, or geom) instead of always in
world frame. Requires fixing the parser (`reftype` attribute is currently
conflated with `refname`), wiring the builder to resolve both `reftype` and
`refname` into `sensor_reftype`/`sensor_refid`, and modifying all 9 frame
sensor evaluation arms (FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis,
FrameLinVel, FrameAngVel, FrameLinAcc, FrameAngAcc) across all 3 pipeline
stages to apply the reference-frame transform. MuJoCo reference:
`mj_sensorPos()`, `mj_sensorVel()`, `mj_sensorAcc()` in `engine_sensor.c`.

**Spec C — missing sensor types** (§62):
New `MjSensorType` variants and evaluation arms for `Clock`,
`JointActuatorFrc`, `CamProjection`, `GeomDist`, `GeomPoint`, `GeomNormal`.
Parsing in `parser.rs`/`types.rs` (new `MjcfSensorType` variants), builder
mapping in `builder/sensor.rs`, evaluation in the appropriate stage files.
MuJoCo reference: `mj_sensorPos()`/`mj_sensorVel()`/`mj_sensorAcc()` in
`engine_sensor.c`.

**Spec D — sensor history attributes** (DT-109):
MJCF parsing and model storage for `nsample`, `interp`, `delay` on sensor
elements. New model arrays, extends `nhistory` computation to include sensors
(currently actuator-only). No runtime behavior change (attributes stored for
future use). Mirrors Phase 5 Spec D's actuator history approach. MuJoCo
reference: `mjsSensor_` in `mjspec.h`, sensor model arrays in `mjmodel.h`.

---

## Dependency Graph & Implementation Order

```
          ┌───────────┐
          │  Spec A   │  Step 1
          │ DT-62/64  │
          │ DT-102    │
          │ objtype + │
          │ touch +   │
          │ geom acc  │
          └─────┬─────┘
                │ (Spec B depends on Spec A's objtype parsing)
          ┌─────▼─────┐   ┌───────────┐   ┌───────────┐
          │  Spec B   │   │  Spec C   │   │  Spec D   │  Step 2 (parallel)
          │  DT-63    │   │  §62      │   │  DT-109   │
          │ reftype/  │   │ missing   │   │ sensor    │
          │ refid     │   │ types     │   │ history   │
          └───────────┘   └───────────┘   └───────────┘
```

### Implementation order with rationale

| Step | Deliverable | Rationale |
|------|-------------|-----------|
| **1** | **Spec A** | DT-62 (objtype parsing) must land before DT-102 (geom acc sensors use explicit objtype) and before Spec B (reftype/refid builds on the same parser/builder section). DT-64 (touch multi-geom) is independent but groups naturally with DT-62 since both modify `resolve_sensor_object()`. |
| **2** | **Spec B, C, D** (parallel) | These three are independent — Spec B modifies frame sensor evaluation arms (adding ref-frame transforms to existing arms), Spec C adds new match arms (new sensor types), Spec D is parser/model-only (no evaluation changes). The one shared file (`builder/sensor.rs`) is safe because each adds code in different sections: Spec B adds reftype/refid resolution, Spec C adds new type mappings, Spec D adds history attribute parsing. |

### Dependency edges

| From → To | Specific dependency |
|-----------|-------------------|
| Spec A → Spec B | Spec B's reftype/refid resolution in the builder requires the objtype parsing infrastructure from DT-62. The parser fix (separating `reftype` from `refname`) is in Spec A's scope since it touches the same parser section. |
| Spec A → Spec C | Weak — Spec C adds new sensor types that may need objtype dispatch. Spec C should use the explicit objtype infrastructure from Spec A rather than the old heuristic. |
| DT-62 → DT-102 | DT-102 (geom acc sensors) requires `MjObjectType::Geom` to be explicitly settable via MJCF `objtype` attribute, which DT-62 provides. |

---

## File Ownership Matrix

Files touched by 2+ deliverables, with ownership sequence and handoff state.

### `sim/L0/mjcf/src/parser.rs` (sensor parsing section, ~line 3418–3490)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A | Parse `objtype` attribute on frame sensors. Fix `reftype`/`refname` conflation — separate into two distinct attributes. | `objtype` parsed for frame sensors; `reftype` and `refname` parsed as separate fields |
| 2 | Spec C | Add new `MjcfSensorType` variants to `from_str()`: `clock`, `jointactuatorfrc`, `camprojection`, `geomdist`, `geompoint`, `geomnormal` | 6 new sensor element names recognized |
| 3 | Spec D | Parse `nsample`, `interp`, `delay` attributes on sensor elements | 3 new sensor attributes parsed |

Low conflict: Spec A modifies existing attribute parsing; Spec C adds new element name dispatch; Spec D adds new attribute parsing in `parse_sensor_attrs()`.

### `sim/L0/mjcf/src/types.rs` (MjcfSensor struct + MjcfSensorType enum)

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A | Add `objtype: Option<String>` field to `MjcfSensor`. Separate `reftype` into its own field. | 2 new/modified struct fields |
| 2 | Spec C | Add 6 variants to `MjcfSensorType`: `Clock`, `Jointactuatorfrc`, `Camprojection`, `Geomdist`, `Geompoint`, `Geomnormal` | 39 enum variants total |
| 3 | Spec D | Add `nsample: Option<usize>`, `interp: Option<String>`, `delay: Option<f64>` fields to `MjcfSensor` | 3 new struct fields |

No conflict: different struct fields and different enum variants.

### `sim/L0/mjcf/src/builder/sensor.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A | Rewrite `resolve_sensor_object()` for frame sensors: use explicit `objtype` attribute instead of name-guessing heuristic. Rewrite touch sensor resolution: resolve to body_id, store all geom IDs (or keep body_id for runtime aggregation). | Explicit objtype dispatch; touch keyed by body |
| 2 | Spec B | Wire `reftype`/`refid` resolution: parse `reftype` attribute to `MjObjectType`, resolve `refname` to ID, push to `sensor_reftype`/`sensor_refid` (replacing hardcoded `None`/`0`). | reftype/refid fully wired |
| 3 | Spec C | Add new variants to `convert_sensor_type()`: 6 new mappings. Add new arms to `sensor_datatype()`: stage assignment for each new type. Extend `resolve_sensor_object()` for new sensor type object requirements. | 6 new sensor types fully buildable |
| 4 | Spec D | Add history attribute storage: push `nsample`, `interp`, `delay` to new model arrays. Update `nhistory` computation. | Sensor history attributes stored |

Higher risk file — 4 specs touch it. Spec A makes the largest structural change (rewriting `resolve_sensor_object()`). The others add incrementally.

### `sim/L0/core/src/types/enums.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec C | Add 6 new variants to `MjSensorType`: `Clock`, `JointActuatorFrc`, `CamProjection`, `GeomDist`, `GeomPoint`, `GeomNormal`. Update `dim()` method. | 38 enum variants |

Single-owner for Phase 6.

### `sim/L0/core/src/types/model.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A | Add `sensor_bodyid: Vec<Vec<usize>>` or equivalent for multi-geom touch (body→geom mapping), if needed at model level | Possible new field |
| 2 | Spec D | Add `sensor_nsample: Vec<usize>`, `sensor_interp: Vec<InterpType>`, `sensor_delay: Vec<f64>`. Update `nhistory` to include sensor contributions. | 3 new fields + nhistory change |

Low conflict: different fields.

### `sim/L0/core/src/types/model_init.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A | Init new touch-related fields if added | New field defaults |
| 2 | Spec D | Init `sensor_nsample`, `sensor_interp`, `sensor_delay` | 3 new field defaults |

No conflict: same pattern as model.rs.

### `sim/L0/core/src/sensor/position.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec B | Add reference-frame transform to FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis | Relative frame support in pos-stage frame sensors |
| 2 | Spec C | Add match arms for new position-stage sensors (`Clock` if pos-stage) | New sensor evaluation arms |

Low conflict: Spec B modifies existing arms; Spec C adds new arms.

### `sim/L0/core/src/sensor/velocity.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec B | Add reference-frame transform to FrameLinVel, FrameAngVel | Relative frame support in vel-stage frame sensors |

Single-owner for Phase 6.

### `sim/L0/core/src/sensor/acceleration.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec A | Wire FrameLinAcc/FrameAngAcc for `MjObjectType::Geom` (DT-102). Modify Touch evaluation to aggregate across all geoms on body (DT-64). | Geom acc sensors working; touch body-level aggregation |
| 2 | Spec B | Add reference-frame transform to FrameLinAcc, FrameAngAcc | Relative frame support in acc-stage frame sensors |
| 3 | Spec C | Add match arms for new acc-stage sensors (`JointActuatorFrc`) | New sensor evaluation arms |

Higher risk — 3 specs touch this file. Spec A modifies existing arms (geom dispatch, touch rewrite); Spec B adds ref-frame transforms to those same arms; Spec C adds new arms.

### `sim/L0/core/src/sensor/postprocess.rs`

| Order | Deliverable | Change | State after |
|-------|-------------|--------|-------------|
| 1 | Spec C | Add cutoff handling for new sensor types (if they need special treatment like Touch/Rangefinder) | Updated cutoff dispatch |

Single-owner.

---

## API Contracts

Cross-spec API dependencies. Sub-specs must be written against these contracts,
not against the current (pre-Phase 6) codebase.

### Contract 1: `resolve_sensor_object()` after Spec A (Spec A → Spec B, Spec C)

**Current signature:**
```rust
fn resolve_sensor_object(
    &self,
    sensor_type: MjSensorType,
    objname: Option<&str>,
) -> Result<(MjObjectType, usize), ModelConversionError>
```

**After Spec A — expected changes:**
1. Frame sensor resolution uses explicit `objtype` attribute from MJCF instead
   of site→body→geom name guessing. The function gains an `objtype` parameter
   or reads it from the `MjcfSensor` struct directly.
2. Touch sensor resolution returns `(MjObjectType::Body, body_id)` or
   `(MjObjectType::Site, site_id)` instead of `(MjObjectType::Geom, first_geom_id)`.
   The evaluation code handles body→geom iteration at runtime.

**What Spec B needs:** Spec B adds `reftype`/`refid` resolution — a parallel
resolution path for the reference object. Spec B should follow the same
resolution pattern that Spec A establishes for the primary object.

**What Spec C needs:** New sensor types in Spec C need object resolution
entries in `resolve_sensor_object()`. They should use the explicit dispatch
established by Spec A (e.g., `GeomDist` needs two geom IDs, `CamProjection`
needs a camera + target body).

### Contract 2: Touch sensor keying change (Spec A, affects evaluation)

**Current:** Touch sensor stores `(MjObjectType::Geom, first_geom_id)` in model.
Evaluation scans `efc_force` for contacts involving that single geom.

**After Spec A:** Touch sensor stores `(MjObjectType::Site, site_id)` or
equivalent. Evaluation resolves site→body at runtime, then scans contacts
for any geom attached to that body.

**MuJoCo behavior:** Touch sensor in MuJoCo is site-based. `mj_sensorAcc()`
in `engine_sensor.c` iterates contacts and checks if either contact geom
belongs to the site's body. All geoms on the body contribute.

### Contract 3: `MjcfSensor` struct after Spec A (Spec A → Spec B, Spec D)

**Current fields:** `name`, `class`, `sensor_type`, `objname`, `refname`,
`noise`, `cutoff`, `user`.

**After Spec A — expected new fields:**
- `objtype: Option<String>` — explicit object type string from MJCF attribute
- `reftype` separated from `refname` into its own field

**What Spec B needs:** The separated `reftype` field for reference object
type resolution.

**What Spec D needs:** Spec D adds `nsample`, `interp`, `delay` fields —
independent of Spec A's changes.

### Contract 4: Frame sensor ref-frame transform (Spec B, applies to Spec A's evaluation code)

**After Spec B:** Every frame sensor evaluation arm checks
`sensor_reftype[sensor_id]`. If `!= MjObjectType::None`, the sensor output is
transformed into the reference frame:
- Position: `R_ref^T * (pos_obj - pos_ref)`
- Orientation: `quat_ref^{-1} * quat_obj`
- Velocity: `R_ref^T * (vel_obj - vel_ref)`
- Acceleration: `R_ref^T * (acc_obj - acc_ref)`

This transform is applied after the existing evaluation logic, before
`sensor_write`. Spec B must not change the evaluation logic that Spec A
establishes — it wraps the result.

---

## Shared Convention Registry

Conventions decided once here. Sub-specs reference this section instead of
inventing their own.

### 1. New model array naming

All new sensor-related model arrays follow the existing `sensor_{name}` pattern:

| Field | Type | Default | Spec |
|-------|------|---------|------|
| `sensor_nsample` | `Vec<usize>` | `0` | D |
| `sensor_interp` | `Vec<InterpType>` | `InterpType::None` | D |
| `sensor_delay` | `Vec<f64>` | `0.0` | D |

### 2. New enum variant ordering

New `MjSensorType` variants are appended after `SubtreeAngMom`, before `User`
(which stays last by convention, matching MuJoCo's `mjSENS_USER` pattern):

```rust
pub enum MjSensorType {
    // ... existing 31 variants ...
    SubtreeAngMom,

    // Spec C — new types (appended before User)
    Clock,             // 1D — simulation time
    JointActuatorFrc,  // 1D — net actuator force on joint
    CamProjection,     // 2D — 3D point projected to camera image
    GeomDist,          // 1D — minimum distance between two geoms
    GeomPoint,         // 3D — nearest point on geom
    GeomNormal,        // 3D — surface normal at nearest point

    // User stays last
    User,
}
```

New `MjcfSensorType` variants follow the same ordering convention.

### 3. MJCF attribute naming

Match MuJoCo attribute names exactly:
- `objtype` (not `obj_type`)
- `reftype` (not `ref_type`)
- `refname` (not `ref_name`)
- `nsample`, `interp`, `delay` (match MuJoCo 3.x names exactly)
- New sensor element names: `clock`, `jointactuatorfrc`, `camprojection`,
  `geomdist`, `geompoint`, `geomnormal`

### 4. Sensor stage assignment for new types

| Sensor Type | Stage | Rationale |
|-------------|-------|-----------|
| `Clock` | Position | No velocity or force data needed — reads `data.time` |
| `JointActuatorFrc` | Acceleration | Reads `qfrc_actuator` which is computed post-actuation |
| `CamProjection` | Position | Reads camera pose + body position — position-stage data |
| `GeomDist` | Position | Distance query from geom poses — position-stage data |
| `GeomPoint` | Position | Nearest point from geom poses — position-stage data |
| `GeomNormal` | Position | Surface normal from geom poses — position-stage data |

### 5. Touch sensor convention change

**Before (current):** Touch sensor stores `(MjObjectType::Geom, first_geom_id)`.
**After Spec A:** Touch sensor stores `(MjObjectType::Site, site_id)`.

At evaluation time, the sensor resolves `site → body` via `model.site_body[site_id]`,
then iterates all contacts checking whether either contact geom belongs to that
body (via `model.geom_body[geom_id] == body_id`). This matches MuJoCo's behavior
in `mj_sensorAcc()`.

### 6. Default values for new fields

- Numeric fields default to `0.0` (not `f64::NAN` or sentinels).
- `Option<String>` fields default to `None`.
- Enum fields default to their `#[default]` variant.
- New arrays initialized in `model_init.rs` as empty `vec![]`.

---

## Cross-Spec Blast Radius

### Behavioral interactions between specs

| Interaction | Analysis |
|------------|----------|
| DT-62 objtype + DT-102 geom acc | **Dependency, not conflict.** DT-102 needs explicit `MjObjectType::Geom` from DT-62. Both are in Spec A — no cross-spec issue. |
| DT-62 objtype + DT-63 reftype | **Dependency.** Spec B's reftype parsing builds on the parser refactoring from Spec A (separating `reftype` from `refname`). Spec A lands first. |
| DT-64 touch rewrite + DT-63 reftype | **No interaction.** Touch sensors don't use reftype (they're not frame sensors). |
| Spec C new types + Spec B reftype | **Potential interaction.** New sensor types (e.g., `CamProjection`) might support reftype. MuJoCo reference determines this — likely no, since most new types are scalar or point-based, not frame-based. |
| Spec D history + Spec C new types | **No interaction.** History attributes apply to all sensor types uniformly. New types get the same default history attributes as existing types. |

### Existing test impact (cross-spec)

| Test area | Touched by | Conflict risk |
|-----------|-----------|--------------|
| `sensor/position.rs` tests | Spec A (frame sensor changes), Spec B (ref-frame transforms), Spec C (new arms) | **Medium.** Spec A changes frame sensor behavior (explicit objtype). Existing tests may need updated model setup if they relied on the heuristic. |
| `sensor/acceleration.rs` tests | Spec A (touch + geom acc), Spec B (ref-frame), Spec C (new arms) | **Medium.** Touch sensor behavior change may affect existing touch tests. |
| `sensor/velocity.rs` tests | Spec B only | **Low.** Single-owner for new ref-frame transforms. |
| Phase 5 regression suite (2,238+ tests) | None | **None.** Phase 6 does not modify any actuator code paths. |

### Test count changes

| Deliverable | Estimated new tests | Net change |
|-------------|-------------------|------------|
| Spec A | 10–15 (objtype explicit, touch multi-geom, geom acc sensors) | +10–15 |
| Spec B | 8–12 (ref-frame transforms across 3 stages, 9 frame sensor types) | +8–12 |
| Spec C | 8–12 (2 per new sensor type: happy path + edge case) | +8–12 |
| Spec D | 3–5 (parse nsample/interp/delay, defaults, nhistory) | +3–5 |
| **Total** | **29–44** | **+29–44** |

---

## Phase-Level Acceptance Criteria

These are the aggregate gates that determine "Phase 6 complete." Individual
sub-specs have their own ACs for technical correctness. **The overarching
criterion: CortenForge's sensor behavior is numerically identical to MuJoCo's
for every feature implemented in Phase 6.**

### PH6-AC1: All 6 tasks ship-complete
Every task in the assignment table has landed and its sub-spec ACs are met.
Every sub-spec AC that asserts a numerical value has that value verified
against MuJoCo's actual output.

### PH6-AC2: No regression in existing test suite
All 2,238+ domain tests from the post-Phase 5 baseline pass. Zero test
failures attributable to Phase 6 changes.

### PH6-AC3: Quality gate passes
`cargo xtask check` passes (formatting, clippy, all tests).

### PH6-AC4: Aggregate test growth
Domain test count increases by at least 29 (lower bound of per-spec estimates).
At least one MuJoCo conformance test (expected value from running MuJoCo) per
sub-spec.

### PH6-AC5: Sensor conformance coverage
After Phase 6, the following MuJoCo sensor features are covered:

| Feature | Pre-Phase 6 | Post-Phase 6 |
|---------|------------|-------------|
| Frame sensor objtype | Heuristic (site→body→geom guessing) | Explicit MJCF `objtype` attribute |
| Touch sensor scope | Single geom (first on body) | All geoms on sensor's body |
| Geom-attached acc sensors | FrameLinAcc/FrameAngAcc return zeros for Geom | Full `mj_objectAcceleration()` at geom position |
| Relative frame sensors | `reftype`/`refid` stubbed (`None`/`0`) | Fully wired — output in reference frame |
| Sensor types | 32 types | +6 types (38 total) |
| Sensor history attributes | Not parsed for sensors | Parsed + stored (runtime deferred) |

---

## Out of Scope

Explicitly excluded from Phase 6. Each exclusion states its conformance impact.

- **DT-65** (`User` sensor `dim` attribute) — tracked in Post-v1.0 Low-Priority
  MuJoCo Compat. *Conformance impact: minimal — user sensors are callback-based
  and the `dim` attribute only affects how many sensordata slots the sensor
  occupies. No standard model uses variable-dim user sensors.*

- **Sensor noise application** — `sensor_noise` is parsed and stored but not
  applied at runtime (intentional design — RL training parity). Not a
  conformance gap because MuJoCo's noise is stochastic and conformance tests
  cannot validate it deterministically.

- **Camera model parameters for `camprojection`** — if full camera intrinsics
  are not yet implemented, `CamProjection` sensor may be deferred or
  implemented with the existing camera model. Spec C will determine scope
  after reading the MuJoCo C source. *Conformance impact: minor — camera
  projection sensors are uncommon in RL/robotics models.*

- **Runtime sensor interpolation** — `nsample`/`interp`/`delay` runtime
  behavior (reading from history buffer during `mj_forward`) is deferred.
  Spec D stores the attributes; runtime is tracked as part of DT-107/DT-108.
  *Conformance impact: none for v1.0 — interpolation is a MuJoCo 3.x feature
  not exercised by standard conformance tests.*

- **Sensor performance optimization** — Phase 6 is correctness/completeness,
  not performance. *Conformance impact: none.*
