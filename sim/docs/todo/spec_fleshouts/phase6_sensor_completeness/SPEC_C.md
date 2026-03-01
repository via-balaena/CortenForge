# Phase 6 Spec C — Missing Sensor Types (Clock, JointActuatorFrc, GeomDist, GeomNormal, GeomFromTo)

**Status:** Draft
**Phase:** Roadmap Phase 6 — Sensor Completeness
**Effort:** L
**MuJoCo ref:** `mj_computeSensorPos()` and `mj_computeSensorAcc()` in `engine_sensor.c`;
`mj_geomDistance()` in `engine_support.c`; `apply_cutoff()` in `engine_sensor.c`
**MuJoCo version:** 3.5.0
**Prerequisites:**
- Spec A implementation landed (commit `28bc9f4`) — objtype parsing, builder
  infrastructure, `resolve_sensor_object()` explicit dispatch pattern
- Spec B implementation landed (commit `fb8ec66`) — `resolve_reference_object()`
  for reftype/refid resolution
- Test baseline: 2,238+ domain tests post-Phase 5

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment from Umbrella

The umbrella spec (§62) listed 6 sensor types. Empirical verification against
MuJoCo 3.5.0 produced scope corrections documented in SPEC_C_RUBRIC.md
(Scope Adjustment table):

| Umbrella claim | MuJoCo 3.5.0 reality | Action |
|----------------|----------------------|--------|
| `geompoint` | Does not exist. `<geompoint>` produces "unrecognized element" error. | Replace with `GeomFromTo` (MJCF element `fromto`). |
| `camprojection` | Exists but requires camera infrastructure (`cam_xpos`, `cam_xmat`, `cam_resolution`, `cam_fovy`, `cam_intrinsic`, `cam_sensorsize`) — none exists in CortenForge. | Defer to DT-120. |

**Spec C final scope: 5 sensor types.**

---

## Problem Statement

**Conformance gap — §62: Missing sensor types.** MuJoCo supports 5 sensor
types that CortenForge does not implement: `Clock`, `JointActuatorFrc`,
`GeomDist`, `GeomNormal`, and `GeomFromTo`. These types have evaluation
arms in `mj_computeSensorPos()` and `mj_computeSensorAcc()` in
`engine_sensor.c` that produce well-defined numerical output. CortenForge's
`MjSensorType` enum (`enums.rs:342–421`) and `MjcfSensorType` enum
(`types.rs:2863–2938`) lack these variants. The parser (`types.rs:2942–2977`)
cannot recognize the MJCF element names (`clock`, `jointactuatorfrc`,
`distance`, `normal`, `fromto`). The builder cannot map, resolve, or assign
pipeline stages for these types. The evaluation files (`position.rs`,
`acceleration.rs`) have no match arms — sensors of these types would hit
the `_ => {}` wildcard and silently produce zeros.

Additionally, the three geometry distance sensors (`GeomDist`, `GeomNormal`,
`GeomFromTo`) introduce a **dual-object resolution pattern** — each sensor
references two objects (obj + ref), each of which can be either a geom or a
body. This is a new pattern; existing sensors resolve at most one object.
The geom distance sensors also require a `geom_distance()` helper function
that does not exist in CortenForge.

**Conformance gap** — MJCF files containing these sensor types are either
silently dropped (unrecognized element) or, if enum variants were added
without evaluation, would produce zero output instead of the correct values.

---

## MuJoCo Reference

> **This is the most important section of the spec.** All algorithms, ACs,
> and tests are derived from what's documented here.

### Clock sensor

**Source:** `engine_sensor.c`, `mj_computeSensorPos()`, case `mjSENS_CLOCK`.

```c
case mjSENS_CLOCK:
  sensordata[0] = d->time;
  break;
```

- **Enum:** `mjSENS_CLOCK` (45)
- **Dim:** 1
- **Datatype:** `mjDATATYPE_REAL` (0)
- **Stage:** `mjSTAGE_POS` (position stage)
- **Objtype:** `mjOBJ_UNKNOWN` (0) — no object attachment
- **Objid:** -1
- **Reftype:** `mjOBJ_UNKNOWN` (0) — no reference
- **Refid:** -1
- **MJCF element:** `<clock name="..."/>`
- **Attributes:** `name`, `nsample`, `interp`, `delay`, `interval`, `cutoff`,
  `noise`, `user`. No object name attributes.

**Timing behavior:** Sensors are evaluated inside `mj_forward()`, before
time is incremented. After `mj_forward(t=0)`, sensor reads 0.0. After
`mj_step()` (which calls `mj_forward` then increments time), the sensor
reads the time at the *start* of the step — lagging by one timestep.
Formula: after N calls to `mj_step`, sensor reads `(N-1) * timestep`.

### JointActuatorFrc sensor

**Source:** `engine_sensor.c`, `mj_computeSensorAcc()`, case `mjSENS_JOINTACTFRC`.

```c
case mjSENS_JOINTACTFRC:
  sensordata[0] = d->qfrc_actuator[m->jnt_dofadr[objid]];
  break;
```

- **Enum:** `mjSENS_JOINTACTFRC` (16)
- **Dim:** 1
- **Datatype:** `mjDATATYPE_REAL` (0)
- **Stage:** `mjSTAGE_ACC` (acceleration stage)
- **Objtype:** `mjOBJ_JOINT` (3)
- **Reftype:** `mjOBJ_UNKNOWN` (0)
- **MJCF element:** `<jointactuatorfrc name="..." joint="..."/>`
- **Attributes:** `name`, `joint`, `nsample`, `interp`, `delay`, `interval`,
  `cutoff`, `noise`, `user`.

**Semantic difference from `ActuatorFrc`:** `ActuatorFrc` reads
`data.actuator_force[actuator_id]` — the force of a *single* actuator.
`JointActuatorFrc` reads `data.qfrc_actuator[dof_adr]` — the *net*
actuator force at a joint DOF (sum of all actuators acting on that joint,
mapped through their gear ratios).

**Joint type restriction:** MuJoCo's compiler enforces that the joint must
be `hinge` or `slide` — ball and free joints are rejected at compile time
with `"joint must be slide or hinge in sensor"`. This matches `JointPos`/
`JointVel` which also only support 1-DOF joints.

**Zero-actuator case:** When no actuators act on a joint,
`qfrc_actuator[dof_adr]` stays at its reset value of 0.0. The sensor
reads 0.0.

**Stage correctness:** `qfrc_actuator` is populated during the acceleration
pipeline (after `mj_fwdActuation`). If evaluated in the position stage,
the array would contain zero-initialized data — the sensor would read 0.0
regardless of actuator state. This makes stage assignment testable: a
non-trivial `ctrl` with a non-zero `gear` ratio produces a non-zero
sensor value only when evaluated in the correct (acceleration) stage.

### GeomDist / GeomNormal / GeomFromTo sensors

**Source:** `engine_sensor.c`, `mj_computeSensorPos()`, shared case block
for `mjSENS_GEOMDIST`, `mjSENS_GEOMNORMAL`, `mjSENS_GEOMFROMTO`.

```c
case mjSENS_GEOMDIST:
case mjSENS_GEOMNORMAL:
case mjSENS_GEOMFROMTO:
{
    mjtNum cutoff = m->sensor_cutoff[i];
    mjtNum dist = cutoff;          // initialize to cutoff
    mjtNum fromto[6] = {0};

    // Resolve geom lists — body ⇒ iterate body's direct geoms; geom ⇒ single
    int n1, id1;
    if (objtype == mjOBJ_BODY) {
        n1 = m->body_geomnum[objid];
        id1 = m->body_geomadr[objid];
    } else {
        n1 = 1; id1 = objid;
    }
    int n2, id2;
    if (reftype == mjOBJ_BODY) {
        n2 = m->body_geomnum[refid];
        id2 = m->body_geomadr[refid];
    } else {
        n2 = 1; id2 = refid;
    }

    // All-pairs minimum distance
    for (int geom1=id1; geom1 < id1+n1; geom1++) {
        for (int geom2=id2; geom2 < id2+n2; geom2++) {
            mjtNum fromto_new[6] = {0};
            mjtNum dist_new = mj_geomDistance(m, d, geom1, geom2,
                                              cutoff, fromto_new);
            if (dist_new < dist) {
                dist = dist_new;
                mju_copy(fromto, fromto_new, 6);
            }
        }
    }

    // Output
    if (type == mjSENS_GEOMDIST)   sensordata[0] = dist;
    else if (type == mjSENS_GEOMNORMAL) {
        mjtNum normal[3] = {fromto[3]-fromto[0],
                            fromto[4]-fromto[1],
                            fromto[5]-fromto[2]};
        if (normal[0] || normal[1] || normal[2])
            mju_normalize3(normal);
        mju_copy3(sensordata, normal);
    }
    else mju_copy(sensordata, fromto, 6);  // GEOMFROMTO
}
```

**Metadata:**

| Type | Enum | Dim | Datatype | Stage | Objtype | Reftype |
|------|------|-----|----------|-------|---------|---------|
| `GeomDist` | `mjSENS_GEOMDIST` (39) | 1 | `mjDATATYPE_REAL` (0) | POS | Geom or Body | Geom or Body |
| `GeomNormal` | `mjSENS_GEOMNORMAL` (40) | 3 | `mjDATATYPE_AXIS` (2) | POS | Geom or Body | Geom or Body |
| `GeomFromTo` | `mjSENS_GEOMFROMTO` (41) | 6 | `mjDATATYPE_REAL` (0) | POS | Geom or Body | Geom or Body |

**MJCF element names differ from enum names:**
- `distance` (not `geomdist`) — element `<distance geom1="..." geom2="..."/>`
- `normal` (not `geomnormal`) — element `<normal geom1="..." geom2="..."/>`
- `fromto` (not `geomfromto`) — element `<fromto geom1="..." geom2="..."/>`

**Dual-object attributes:** `geom1`, `geom2`, `body1`, `body2`, `cutoff`,
`name`, `nsample`, `interp`, `delay`, `interval`, `noise`, `user`.

**Object resolution rules:**
- **Strict XOR per side:** Exactly one of `{geom1, body1}` and exactly one
  of `{geom2, body2}` must be specified. Both or neither on one side is a
  parse error: `"exactly one of (geom1, body1) must be specified"` /
  `"exactly one of (geom2, body2) must be specified"`.
- `geom1`/`geom2` → `objtype`/`reftype` = `mjOBJ_GEOM`, single geom.
- `body1`/`body2` → `objtype`/`reftype` = `mjOBJ_BODY`, iterate all
  body's **direct** geoms only (uses `body_geomadr`/`body_geomnum`,
  NOT subtree/child bodies).
- **Mixed combinations supported:** `geom1` + `body2` and `body1` + `geom2`
  are valid. All 4 combinations work.
- **Same-body geoms:** Two geoms on the same body CAN be distance-measured.
- **Self-distance rejected:** `geom1 == geom2` rejected by MuJoCo compiler
  (`"1st body/geom must be different from 2nd body/geom"`).
- **Body/XBody distinction:** `body1`/`body2` resolve to `MjObjectType::Body`
  (NOT `MjObjectType::XBody`). MuJoCo uses `mjOBJ_BODY` for geom distance
  sensors. This matters because `Body` accesses `body_geom_adr`/`body_geom_num`
  for geom iteration, while `XBody` routes to frame-position logic.

**`mj_geomDistance()` behavior:**
- Returns `(signed_distance, fromto_points)`.
- `fromto = [from_x, from_y, from_z, to_x, to_y, to_z]` — "from" = nearest
  surface point on geom1, "to" = nearest surface point on geom2.
- For penetrating geoms (`dist < 0`): returns exact negative distance and
  populated fromto (surface points go inside opposite geom).
- For non-penetrating geoms where `dist < cutoff`: returns exact distance
  and populated fromto.
- For non-penetrating geoms where `dist >= cutoff`: returns `cutoff` and
  leaves fromto at initialization (zeroed).
- Supports all geom types (sphere, box, capsule, cylinder, ellipsoid, plane)
  via GJK/MPR collision system.

**Cutoff semantics (critical):**
- `cutoff=0` (default when omitted): dist initialized to 0.
  - Non-penetrating geoms: `dist_new >= 0`, so `dist_new < 0` is false →
    dist stays 0 → sensor returns 0.0. Positive distances suppressed.
  - Penetrating geoms: `dist_new < 0`, so `dist_new < 0` is true → dist
    updates → sensor returns negative penetration depth. Penetration NOT
    suppressed.
- `cutoff > 0`: dist initialized to cutoff. Distance capped at cutoff for
  distant geoms.
- Cutoff attribute defaults to 0.0 when omitted (despite schema marking
  `required="true"` for `<distance>`, `<normal>`, `<fromto>`, MuJoCo's
  parser accepts omission). CortenForge must NOT enforce required.

**Beyond-cutoff behavior (asymmetry):**
- `GeomDist`: returns cutoff value.
- `GeomNormal`: returns `[0, 0, 0]` (zero vector — fromto stays zeroed,
  difference is zero, normalization guard preserves zero).
- `GeomFromTo`: returns `[0, 0, 0, 0, 0, 0]` (fromto stays at init).

**FromTo output ordering:** `[from_x, from_y, from_z, to_x, to_y, to_z]`
— from = geom1 surface, to = geom2 surface. Swapping geom order swaps
from/to.

**Penetrating fromto:** Surface points go inside the opposite geom's volume.
E.g., g1 at origin r=0.5, g2 at (0.3,0,0) r=0.5: fromto = `[0.5, 0, 0,
-0.2, 0, 0]` — "from" is inside g2, "to" is inside g1.

**Normal zero-vector guard:** `if (normal[0] || normal[1] || normal[2])`
prevents normalization of zero vectors. When all components are zero
(beyond-cutoff or coincident geoms with no direction), the output stays
`[0, 0, 0]`.

**Body with zero geoms:** Loop body never executes, dist stays at cutoff
init. Returns cutoff (GeomDist) / `[0,0,0]` (GeomNormal) / `[0,0,0,0,0,0]`
(GeomFromTo).

**Coincident geoms:** Two geoms at the same position with the same radius r
have distance = -2r (full penetration). Normal direction is arbitrary (depends
on collision system's degenerate-case handling). Empirically verified:
coincident spheres r=0.5 → distance=-1.0, normal=[-1,0,0] (arbitrary),
fromto=[0.5,0,0,-0.5,0,0].

**Negative cutoff:** MuJoCo's compiler rejects negative cutoff values at
compile time: `"negative cutoff in sensor"`. This is a compile-time check,
not runtime.

**Noise is metadata-only:** The `noise` attribute is parsed and stored in
`sensor_noise` but **not applied** during `mj_forward()`. The forward pass
is fully deterministic. Noise is metadata for downstream/user-side injection.
Empirically verified: `noise=0.1` on distance/normal/fromto sensors produces
identical values across multiple runs.

**Postprocess cutoff clamping (`apply_cutoff()` in `engine_sensor.c:64–89`):**

```c
static void apply_cutoff(const mjModel* m, int i, mjtNum* data) {
  mjtNum cutoff = m->sensor_cutoff[i];
  if (cutoff <= 0) return;

  mjtSensor type = (mjtSensor)m->sensor_type[i];
  if (type == mjSENS_CONTACT || type == mjSENS_GEOMFROMTO) return;

  int dim = m->sensor_dim[i];
  for (int j = 0; j < dim; j++) {
    if (m->sensor_datatype[i] == mjDATATYPE_REAL)
      data[j] = mju_clip(data[j], -cutoff, cutoff);
    else if (m->sensor_datatype[i] == mjDATATYPE_POSITIVE)
      data[j] = mju_min(cutoff, data[j]);
    // AXIS, QUATERNION: no branch → implicitly skipped
  }
}
```

- **`GeomFromTo`:** Explicitly exempt via type check (`type == mjSENS_GEOMFROMTO`
  → early return). Cutoff is used only as `mj_geomDistance()` search radius,
  NOT for postprocess clamping.
- **`GeomNormal`:** Implicitly exempt via datatype. `mjDATATYPE_AXIS` (2)
  has no matching branch in the `if/else if` chain → elements are not clamped.
  MuJoCo's compiler normally rejects cutoff on AXIS sensors, but GeomNormal
  is **explicitly exempted** from this check (`type != mjSENS_GEOMNORMAL` in
  the compiler condition).
- **`GeomDist`:** `mjDATATYPE_REAL` → clamps to `[-cutoff, cutoff]`. This
  means penetration depth (-0.7) with cutoff=0.5 → sensor reads -0.5.
- **`Clock` and `JointActuatorFrc`:** `mjDATATYPE_REAL` → standard clamping.
  Clock with `cutoff=5` clamps at 5.0; JointActuatorFrc with `cutoff=5`
  clamps to `[-5, +5]`.

**Cutoff is universal:** The `apply_cutoff()` function runs on ALL sensors
with `cutoff > 0`. Per-element clamping: for multi-dimensional sensors, each
scalar element is clamped independently, NOT as a vector magnitude.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Clock reads simulation time | `sensordata[0] = d->time` in `mj_computeSensorPos()` | Not implemented — no `MjSensorType::Clock` variant |
| JointActuatorFrc reads net actuator force at DOF | `d->qfrc_actuator[m->jnt_dofadr[objid]]` in `mj_computeSensorAcc()` | Not implemented — no `MjSensorType::JointActuatorFrc` variant |
| JointActuatorFrc restricted to hinge/slide | Compiler rejects ball/free joints | Not implemented |
| GeomDist/Normal/FromTo shared case block | Body-vs-geom dispatch, all-pairs `mj_geomDistance()`, per-type output | Not implemented — no variants, no `geom_distance()` |
| Dual-object resolution (geom1/body1 + geom2/body2) | Strict XOR per side, 4 combinations, `mjOBJ_BODY` (not XBODY) | Not implemented — no dual-object pattern |
| `mj_geomDistance()` signed distance with cutoff | Returns signed distance, fromto surface points | Not implemented — no `geom_distance()` function |
| Cutoff=0 suppresses positive distances but NOT penetration | `dist = cutoff = 0`, only `dist_new < 0` updates | Not implemented |
| GeomFromTo cutoff exemption in postprocess | Explicit early return in `apply_cutoff()` | Not implemented |
| GeomNormal cutoff exemption via AXIS datatype | Implicit skip (no REAL/POSITIVE branch matches AXIS) | Not implemented |
| MJCF element names `distance`/`normal`/`fromto` | Parser recognizes these element names | Parser does not recognize them |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| MJCF `<distance>` → enum | `mjSENS_GEOMDIST` | `MjcfSensorType::Distance` → `MjSensorType::GeomDist` | MJCF element `"distance"` maps to `Distance` in parser, then to `GeomDist` in builder |
| MJCF `<normal>` → enum | `mjSENS_GEOMNORMAL` | `MjcfSensorType::Normal` → `MjSensorType::GeomNormal` | MJCF element `"normal"` maps to `Normal`, then to `GeomNormal` |
| MJCF `<fromto>` → enum | `mjSENS_GEOMFROMTO` | `MjcfSensorType::Fromto` → `MjSensorType::GeomFromTo` | MJCF element `"fromto"` maps to `Fromto`, then to `GeomFromTo` |
| MJCF `<clock>` → enum | `mjSENS_CLOCK` | `MjcfSensorType::Clock` → `MjSensorType::Clock` | Direct port — `"clock"` maps straight through |
| MJCF `<jointactuatorfrc>` → enum | `mjSENS_JOINTACTFRC` | `MjcfSensorType::Jointactuatorfrc` → `MjSensorType::JointActuatorFrc` | Direct port — `"jointactuatorfrc"` maps through |
| `MjObjectType::None` | `mjOBJ_UNKNOWN` (0) | `MjObjectType::None` | Use `MjObjectType::None` wherever MuJoCo uses `mjOBJ_UNKNOWN` |
| `body_geomnum` | `m->body_geomnum[bodyid]` | `model.body_geom_num[bodyid]` (`model.rs:125`) | CortenForge uses snake_case with underscores |
| `body_geomadr` | `m->body_geomadr[bodyid]` | `model.body_geom_adr[bodyid]` (`model.rs:123`) | CortenForge uses snake_case with underscores |
| `jnt_dofadr` | `m->jnt_dofadr[objid]` | `model.jnt_dof_adr[objid]` (`model.rs:171`) | CortenForge uses snake_case with underscores |
| `qfrc_actuator` | `d->qfrc_actuator[...]` | `data.qfrc_actuator[...]` (`data.rs:51`, `DVector<f64>`) | Direct port — same semantics, nalgebra `DVector` |
| `d->time` | `d->time` | `data.time` (`data.rs:499`, `f64`) | Direct port |
| `d->geom_xpos` | `d->geom_xpos[geom_id]` (world position) | `data.geom_xpos[geom_id]` (`data.rs:107`, `Vec<Vector3<f64>>`) | Direct port |
| `d->geom_xmat` | `d->geom_xmat[9*geom_id]` (3x3 rotation) | `data.geom_xmat[geom_id]` (`data.rs:109`, `Vec<Matrix3<f64>>`) | MuJoCo flat 9-element → nalgebra `Matrix3` |
| `m->geom_rbound` | `m->geom_rbound[geom_id]` (bounding radius) | `model.geom_rbound[geom_id]` (`model.rs:291`, `Vec<f64>`) | Direct port |
| `sensor_write` helpers | Direct array write `sensordata[i] = val` | `sensor_write(sensordata, adr, offset, val)` (`postprocess.rs:12`) | Use `sensor_write` for 1D, `sensor_write3` for 3D |
| 6D output (fromto) | `mju_copy(sensordata, fromto, 6)` | Need `sensor_write6()` — new helper (see S2) | Follow existing `sensor_write3`/`sensor_write4` pattern |
| `MjSensorDataType` | `mjDATATYPE_REAL`/`POSITIVE`/`AXIS`/`QUATERNION` (data kind) | `Position`/`Velocity`/`Acceleration` (pipeline stage) | CortenForge's `MjSensorDataType` is the pipeline stage, NOT MuJoCo's data kind. Postprocess cutoff exemptions must use explicit sensor type matching, not datatype checks |
| `geom_distance()` return | `mj_geomDistance()` returns signed distance, populates fromto array | Rust: `fn geom_distance(...) -> (f64, [f64; 6])` | New function (see S3) |
| nalgebra types | Raw `mjtNum` arrays | `sensor_write3` takes `&Vector3<f64>` — proposed `sensor_write6` takes `&[f64; 6]` (raw array, not nalgebra since no 6D type is standard) | Normal output uses `Vector3`, fromto uses `[f64; 6]` |

---

## Specification

> **Conformance rule:** Every algorithm in this section must produce
> numerically identical results to the MuJoCo C code cited in the MuJoCo
> Reference section. Deviations require explicit proof of numerical
> equivalence.

### S1. Enum variants and dim()

**File:** `core/src/types/enums.rs`
**MuJoCo equivalent:** `mjSENS_CLOCK` (45), `mjSENS_JOINTACTFRC` (16),
`mjSENS_GEOMDIST` (39), `mjSENS_GEOMNORMAL` (40), `mjSENS_GEOMFROMTO` (41)
**Design decision:** Add 5 new variants to `MjSensorType` before `User`
(which stays last by convention, matching MuJoCo's `mjSENS_USER` pattern).
Ordering follows the umbrella spec's Shared Convention Registry §2.

**After** (add before `User` at line 417):
```rust
// ========== New sensors (Phase 6 Spec C) ==========
/// Simulation clock (reads data.time, 1D). MuJoCo: mjSENS_CLOCK.
Clock,
/// Net actuator force at joint DOF (1D). MuJoCo: mjSENS_JOINTACTFRC.
/// Reads data.qfrc_actuator[model.jnt_dof_adr[objid]].
JointActuatorFrc,
/// Signed distance between two geoms or bodies (1D). MuJoCo: mjSENS_GEOMDIST.
GeomDist,
/// Surface normal at nearest point between geoms (3D). MuJoCo: mjSENS_GEOMNORMAL.
GeomNormal,
/// Nearest surface points between two geoms (6D). MuJoCo: mjSENS_GEOMFROMTO.
GeomFromTo,
```

**dim() additions** (in the `match self` block):
```rust
// 1D group:
Self::Clock | Self::JointActuatorFrc | Self::GeomDist => 1,

// 3D group:
Self::GeomNormal => 3,

// 6D — new group:
Self::GeomFromTo => 6,
```

### S2. `sensor_write6()` helper

**File:** `core/src/sensor/postprocess.rs`
**MuJoCo equivalent:** `mju_copy(sensordata, fromto, 6)` in
`mj_computeSensorPos()`.
**Design decision:** Add a `sensor_write6()` helper following the existing
convention of `sensor_write3()` (`postprocess.rs:21`) and `sensor_write4()`
(`postprocess.rs:29`). Takes a raw `&[f64; 6]` rather than a nalgebra type
because there is no standard 6D nalgebra vector type and introducing one
for a single call site is over-engineering.

**After** (add after `sensor_write4` at line 34):
```rust
/// Write a 6D vector to sensor data with bounds checking.
/// Used by GeomFromTo sensor (fromto points).
#[inline]
pub fn sensor_write6(sensordata: &mut DVector<f64>, adr: usize, v: &[f64; 6]) {
    sensor_write(sensordata, adr, 0, v[0]);
    sensor_write(sensordata, adr, 1, v[1]);
    sensor_write(sensordata, adr, 2, v[2]);
    sensor_write(sensordata, adr, 3, v[3]);
    sensor_write(sensordata, adr, 4, v[4]);
    sensor_write(sensordata, adr, 5, v[5]);
}
```

### S3. `geom_distance()` helper function

**File:** `core/src/sensor/position.rs` (or a new module
`core/src/sensor/geom_distance.rs` if the function exceeds ~80 lines)
**MuJoCo equivalent:** `mj_geomDistance()` in `engine_support.c`
**Design decision:** AD-1 option (c) from the rubric — thin wrapper over
existing collision infrastructure. CortenForge's `gjk_epa.rs` provides
`gjk_epa_contact()` which returns `GjkContact { point, normal, penetration }`
for overlapping shapes. For non-overlapping shapes, we need a GJK
closest-point query. The standard GJK algorithm naturally computes the
minimum distance between two convex shapes when they don't overlap — this
is the classic GJK distance query. We extend the existing GJK infrastructure
with a `gjk_distance()` function that returns `(distance, closest_point_a,
closest_point_b)` for non-overlapping convex shapes.

For sphere-sphere and sphere-capsule pairs, analytic formulas are faster and
more numerically stable — use those as fast paths before falling through to
GJK.

**Signature:**
```rust
/// Compute signed distance and nearest surface points between two geoms.
///
/// Returns `(signed_distance, fromto)` where:
/// - `signed_distance` > 0: separation gap (non-overlapping)
/// - `signed_distance` < 0: penetration depth (overlapping)
/// - `fromto = [from_x, from_y, from_z, to_x, to_y, to_z]`:
///   from = nearest surface point on geom1, to = nearest surface point on geom2
///
/// When `signed_distance >= cutoff`, returns `(cutoff, [0; 6])` — geoms
/// beyond cutoff are not computed.
///
/// MuJoCo ref: `mj_geomDistance()` in `engine_support.c`.
pub fn geom_distance(
    model: &Model,
    data: &Data,
    geom1: usize,
    geom2: usize,
    cutoff: f64,
) -> (f64, [f64; 6])
```

**Algorithm (Rust pseudocode):**

```rust
use crate::collision::narrow::geom_to_collision_shape;
use crate::collision_shape::CollisionShape;
use crate::gjk_epa::{gjk_epa_contact, gjk_closest_points};
use crate::types::{Data, GeomType, Model};
use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

pub fn geom_distance(
    model: &Model,
    data: &Data,
    geom1: usize,
    geom2: usize,
    cutoff: f64,
) -> (f64, [f64; 6]) {
    let zero_fromto = [0.0f64; 6];

    // 1. Read world-frame poses from Data
    //    data.geom_xpos: Vec<Vector3<f64>> (data.rs:107)
    //    data.geom_xmat: Vec<Matrix3<f64>> (data.rs:109)
    let pos1 = data.geom_xpos[geom1];
    let pos2 = data.geom_xpos[geom2];
    let mat1 = data.geom_xmat[geom1];
    let mat2 = data.geom_xmat[geom2];

    // 2. Bounding sphere early-out
    //    model.geom_rbound: Vec<f64> (model.rs:291)
    let center_dist = (pos2 - pos1).norm();
    let rbound1 = model.geom_rbound[geom1];
    let rbound2 = model.geom_rbound[geom2];
    let min_possible = center_dist - rbound1 - rbound2;
    if cutoff > 0.0 && min_possible > cutoff {
        return (cutoff, zero_fromto);
    }

    // 3. Read shape parameters
    //    model.geom_type: Vec<GeomType> (model.rs:251)
    //    model.geom_size: Vec<Vector3<f64>> (model.rs:259)
    let gtype1 = model.geom_type[geom1];
    let gtype2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // 4. Analytic sphere-sphere fast path
    if gtype1 == GeomType::Sphere && gtype2 == GeomType::Sphere {
        let r1 = size1.x;
        let r2 = size2.x;
        let dist = center_dist - r1 - r2;

        // Direction from geom1 center to geom2 center
        let dir = if center_dist > 1e-14 {
            (pos2 - pos1) / center_dist
        } else {
            Vector3::x() // arbitrary for coincident centers
        };

        // Surface points: from = g1 surface toward g2, to = g2 surface toward g1
        let from_pt = pos1 + r1 * dir;
        let to_pt = pos2 - r2 * dir;
        let fromto = [from_pt.x, from_pt.y, from_pt.z,
                      to_pt.x, to_pt.y, to_pt.z];

        // Cutoff cap (matches mj_geomDistance return-value semantics)
        if cutoff > 0.0 && dist >= cutoff {
            return (cutoff, zero_fromto);
        }
        return (dist, fromto);
    }

    // 5. Convert to CollisionShape (convex primitives only)
    //    geom_to_collision_shape() at collision/narrow.rs:208
    //    Returns None for Plane, Mesh, Hfield, SDF
    let shape1 = match geom_to_collision_shape(gtype1, size1) {
        Some(s) => s,
        None => {
            // Non-convex or plane — deferred (DT-122)
            warn!("geom_distance: unsupported geom type {:?}", gtype1);
            return (cutoff, zero_fromto);
        }
    };
    let shape2 = match geom_to_collision_shape(gtype2, size2) {
        Some(s) => s,
        None => {
            warn!("geom_distance: unsupported geom type {:?}", gtype2);
            return (cutoff, zero_fromto);
        }
    };

    // 6. Build isometries from Data pose arrays
    let rot1 = UnitQuaternion::from_rotation_matrix(
        &nalgebra::Rotation3::from_matrix_unchecked(mat1),
    );
    let rot2 = UnitQuaternion::from_rotation_matrix(
        &nalgebra::Rotation3::from_matrix_unchecked(mat2),
    );
    let iso1 = Isometry3::from_parts(
        nalgebra::Translation3::from(pos1), rot1,
    );
    let iso2 = Isometry3::from_parts(
        nalgebra::Translation3::from(pos2), rot2,
    );

    // 7. GJK/EPA query
    if let Some(contact) = gjk_epa_contact(&shape1, &iso1, &shape2, &iso2) {
        // Overlapping — signed distance is negative penetration
        let dist = -contact.penetration;
        // Contact point is on shape A surface; compute from/to points
        // from = point on geom1 surface (contact.point + normal * penetration/2)
        // to = point on geom2 surface (contact.point - normal * penetration/2)
        // Approximation using contact normal for surface point recovery:
        let from_pt = contact.point + contact.normal * (contact.penetration * 0.5);
        let to_pt = contact.point - contact.normal * (contact.penetration * 0.5);
        let fromto = [from_pt.x, from_pt.y, from_pt.z,
                      to_pt.x, to_pt.y, to_pt.z];
        if cutoff > 0.0 && dist >= cutoff {
            return (cutoff, zero_fromto);
        }
        (dist, fromto)
    } else {
        // Non-overlapping — use GJK closest-point query (new function)
        match gjk_closest_points(&shape1, &iso1, &shape2, &iso2) {
            Some((dist, pt_a, pt_b)) => {
                let fromto = [pt_a.x, pt_a.y, pt_a.z,
                              pt_b.x, pt_b.y, pt_b.z];
                if cutoff > 0.0 && dist >= cutoff {
                    return (cutoff, zero_fromto);
                }
                (dist, fromto)
            }
            None => {
                // Should not happen for valid convex shapes
                (cutoff, zero_fromto)
            }
        }
    }
}
```

**GJK closest-point extension (`gjk_epa.rs`):**

The existing `gjk_query()` returns `GjkResult { intersecting, simplex,
iterations }`. When shapes do NOT overlap, the GJK simplex encodes the
closest points on the Minkowski difference to the origin. The standard
GJK distance algorithm:

```rust
/// Compute unsigned distance and closest surface points for non-overlapping
/// convex shapes. Returns None if shapes overlap (use gjk_epa_contact instead).
///
/// Algorithm: standard GJK closest-point query.
/// 1. Run GJK. If simplex contains origin → None (overlapping).
/// 2. Find closest point on final simplex to origin → Minkowski difference point.
/// 3. Decompose into barycentric coordinates to recover individual closest points.
pub fn gjk_closest_points(
    shape_a: &CollisionShape,
    pose_a: &Isometry3<f64>,
    shape_b: &CollisionShape,
    pose_b: &Isometry3<f64>,
) -> Option<(f64, Point3<f64>, Point3<f64>)> {
    let result = gjk_query(shape_a, pose_a, shape_b, pose_b);
    if result.intersecting {
        return None;
    }

    // The GJK simplex stores Minkowski difference vertices (A - B).
    // Each vertex v_i = support_A(d_i) - support_B(-d_i).
    // To recover closest points on A and B:
    // 1. Find closest point on simplex to origin: p = Σ λ_i * v_i
    // 2. closest_A = Σ λ_i * support_A(d_i)
    // 3. closest_B = Σ λ_i * support_B(-d_i)
    // The distance is |p| = |closest_A - closest_B|.

    let (closest_a, closest_b) =
        decompose_simplex_closest_points(&result.simplex, shape_a, pose_a, shape_b, pose_b);
    let diff = closest_a - closest_b;
    let dist = diff.coords.norm();
    Some((dist, closest_a, closest_b))
}
```

The `decompose_simplex_closest_points()` helper finds the closest point on
the simplex to the origin using Voronoi region tests (same as the GJK
`do_simplex` subroutine but returning barycentric coordinates), then uses
stored per-vertex support points from each shape to reconstruct the
individual closest points.

**Plane handling:** `geom_to_collision_shape()` returns `None` for planes
(`GeomType::Plane`). Plane-geom distance is deferred to DT-122 alongside
mesh/hfield/SDF. For v1.0, plane geoms in distance sensors return
`(cutoff, [0; 6])` with a warning. This is acceptable because plane geoms
are uncommon in distance sensor configurations.

**Mesh/Hfield/SDF handling:** Also deferred to DT-122. Non-convex geoms
return `(cutoff, [0; 6])` with a warning. Tracked as a conformance gap
acceptable for v1.0 since common RL/robotics models use convex primitives.

**Surface point accuracy note:** The penetrating-case surface point
computation above is an approximation using the EPA contact normal. For
sphere-sphere this is exact (handled by the analytic fast path). For other
shape pairs, EPA provides the penetration depth and normal but the surface
point decomposition may have minor numerical differences from MuJoCo's
`mj_geomDistance()` which uses its own collision backend. Conformance tests
should use tolerances of 1e-6 for non-sphere pairs. Sphere-sphere (analytic)
should match to 1e-10+.

### S4. MjcfSensorType variants and MjcfSensor struct

**File:** `mjcf/src/types.rs`
**MuJoCo equivalent:** MJCF element name dispatch in parser
**Design decision:** Add 5 variants to `MjcfSensorType` before `User`.
Add 4 new fields to `MjcfSensor` for dual-object geom distance attributes
(`geom1`, `geom2`, `body1`, `body2`).

**MjcfSensorType enum** (add before `User` at line 2937):
```rust
/// Clock sensor (simulation time, 1D). MJCF element: `<clock>`.
Clock,
/// Joint actuator force sensor (1D). MJCF element: `<jointactuatorfrc>`.
Jointactuatorfrc,
/// Geom distance sensor (1D). MJCF element: `<distance>`.
Distance,
/// Surface normal between geoms (3D). MJCF element: `<normal>`.
Normal,
/// From-to points between geoms (6D). MJCF element: `<fromto>`.
Fromto,
```

**from_str() additions** (in the match block, before `_ => None`):
```rust
"clock" => Some(Self::Clock),
"jointactuatorfrc" => Some(Self::Jointactuatorfrc),
"distance" => Some(Self::Distance),
"normal" => Some(Self::Normal),
"fromto" => Some(Self::Fromto),
```

**as_str() additions:**
```rust
Self::Clock => "clock",
Self::Jointactuatorfrc => "jointactuatorfrc",
Self::Distance => "distance",
Self::Normal => "normal",
Self::Fromto => "fromto",
```

**dim() additions:**
```rust
// 1D group:
Self::Clock | Self::Jointactuatorfrc | Self::Distance => 1,

// 3D group:
Self::Normal => 3,

// 6D — new group:
Self::Fromto => 6,
```

**MjcfSensor struct** (add after `user` field at line 3090):
```rust
/// First geom name (for distance/normal/fromto sensors).
pub geom1: Option<String>,
/// Second geom name (for distance/normal/fromto sensors).
pub geom2: Option<String>,
/// First body name (for distance/normal/fromto sensors).
pub body1: Option<String>,
/// Second body name (for distance/normal/fromto sensors).
pub body2: Option<String>,
```

**Default impl** — all 4 new fields default to `None`.

### S5. Parser — new element names and dual-object attributes

**File:** `mjcf/src/parser.rs`
**MuJoCo equivalent:** MJCF parser element dispatch
**Design decision:** No changes needed to `parse_sensors()` itself —
`MjcfSensorType::from_str()` (updated in S4) handles new element name
recognition. `parse_sensor_attrs()` needs conditional parsing of
`geom1`/`geom2`/`body1`/`body2` attributes for distance/normal/fromto
sensors, plus strict XOR validation.

**parse_sensor_attrs() additions** (after the `objname` chain at line 3469,
before the `objtype` parsing at line 3472):

```rust
// Dual-object attributes for distance/normal/fromto sensors
if matches!(
    sensor_type,
    MjcfSensorType::Distance | MjcfSensorType::Normal | MjcfSensorType::Fromto
) {
    sensor.geom1 = get_attribute_opt(e, "geom1");
    sensor.geom2 = get_attribute_opt(e, "geom2");
    sensor.body1 = get_attribute_opt(e, "body1");
    sensor.body2 = get_attribute_opt(e, "body2");

    // Strict XOR validation: exactly one of {geom1, body1} required
    let has_obj1 = sensor.geom1.is_some() || sensor.body1.is_some();
    let has_both_obj1 = sensor.geom1.is_some() && sensor.body1.is_some();
    if !has_obj1 || has_both_obj1 {
        // MuJoCo: "exactly one of (geom1, body1) must be specified"
        // Log error and skip — matches MuJoCo compiler behavior
        warn!(
            "sensor '{}': exactly one of (geom1, body1) must be specified",
            sensor.name
        );
    }

    // Strict XOR validation: exactly one of {geom2, body2} required
    let has_obj2 = sensor.geom2.is_some() || sensor.body2.is_some();
    let has_both_obj2 = sensor.geom2.is_some() && sensor.body2.is_some();
    if !has_obj2 || has_both_obj2 {
        warn!(
            "sensor '{}': exactly one of (geom2, body2) must be specified",
            sensor.name
        );
    }
}
```

Note: the existing `objname` or-else chain at line 3463 tries `joint`,
`site`, `body`, `geom`, `tendon`, `actuator`, `objname` in order. For
distance/normal/fromto sensors, `objname` is NOT used — object references
come from `geom1`/`body1` and `geom2`/`body2`. The existing chain may
accidentally pick up `body1` via the `body` arm or `geom1` via the `geom`
arm — but these are different attribute names (`body` vs `body1`, `geom`
vs `geom1`), so no collision occurs.

### S6. Builder — convert_sensor_type(), sensor_datatype(), resolve_sensor_object()

**File:** `mjcf/src/builder/sensor.rs`
**MuJoCo equivalent:** Sensor compilation in `user_objects.cc`
**Design decision:** Add 5 type mappings, 5 stage assignments, and object
resolution arms for all 5 new types. The dual-object pattern for geom
distance sensors is handled in the builder: `geom1`/`body1` → obj,
`geom2`/`body2` → ref (using existing `resolve_reference_object()` at
line 277).

**convert_sensor_type() additions** (before the closing brace at line 400):
```rust
MjcfSensorType::Clock => Some(MjSensorType::Clock),
MjcfSensorType::Jointactuatorfrc => Some(MjSensorType::JointActuatorFrc),
MjcfSensorType::Distance => Some(MjSensorType::GeomDist),
MjcfSensorType::Normal => Some(MjSensorType::GeomNormal),
MjcfSensorType::Fromto => Some(MjSensorType::GeomFromTo),
```

**sensor_datatype() additions** (in the match block):
```rust
// Position stage
MjSensorType::Clock
| MjSensorType::GeomDist
| MjSensorType::GeomNormal
| MjSensorType::GeomFromTo => MjSensorDataType::Position,

// Acceleration stage
MjSensorType::JointActuatorFrc => MjSensorDataType::Acceleration,
```

**resolve_sensor_object() — early return addition** (at line 85, alongside
the existing `User` early return, BEFORE the `objname` required check at
line 89):
```rust
// Clock and User have no object reference — early return before objname check
if sensor_type == MjSensorType::User || sensor_type == MjSensorType::Clock {
    return Ok((MjObjectType::None, 0));
}
```

This is critical: Clock has no `objname` attribute. If the Clock arm were
placed inside the main `match` at line 93 (after the `objname.ok_or_else()`
check at line 89), it would fail with "sensor missing object name" before
ever reaching the match. The early return at line 85 bypasses this check.

Note: GeomDist/GeomNormal/GeomFromTo also lack a standard `objname` — they
use `geom1`/`body1`/`geom2`/`body2` instead. These are handled in
`process_sensors()` (see below), which branches BEFORE calling
`resolve_sensor_object()`, so they never reach the `objname` check.

**resolve_sensor_object() — main match additions** (in the match at line 93):
```rust
// JointActuatorFrc: objname is a joint name (hinge/slide only)
MjSensorType::JointActuatorFrc => {
    let id = *self
        .joint_name_to_id
        .get(name)
        .ok_or_else(|| ModelConversionError {
            message: format!(
                "jointactuatorfrc sensor references unknown joint '{name}'"
            ),
        })?;
    // Validate joint type: MuJoCo rejects ball/free joints
    let jnt_type = self.jnt_type[id];
    if jnt_type != JointType::Hinge && jnt_type != JointType::Slide {
        return Err(ModelConversionError {
            message: format!(
                "jointactuatorfrc sensor: joint '{}' must be slide or hinge",
                name
            ),
        });
    }
    Ok((MjObjectType::Joint, id))
}

```

Note: GeomDist/GeomNormal/GeomFromTo do NOT need an arm in
`resolve_sensor_object()` — they are handled entirely in the modified
`process_sensors()` below, which branches before calling
`resolve_sensor_object()`.

**process_sensors() modification** — for dual-object sensors, replace the
standard obj/ref resolution with dual-object resolution:

```rust
// In process_sensors(), after the standard resolve_sensor_object() call:
let (objtype, objid, reftype, refid) = if matches!(
    sensor_type,
    MjSensorType::GeomDist | MjSensorType::GeomNormal | MjSensorType::GeomFromTo
) {
    // Dual-object resolution from geom1/body1 and geom2/body2
    let (ot, oi) = self.resolve_dual_object_side(
        mjcf_sensor.geom1.as_deref(),
        mjcf_sensor.body1.as_deref(),
        "1",
    )?;
    let (rt, ri) = self.resolve_dual_object_side(
        mjcf_sensor.geom2.as_deref(),
        mjcf_sensor.body2.as_deref(),
        "2",
    )?;
    (ot, oi, rt, ri)
} else {
    let (ot, oi) = self.resolve_sensor_object(
        sensor_type,
        mjcf_sensor.objname.as_deref(),
        mjcf_sensor.objtype.as_deref(),
    )?;
    let (rt, ri) = self.resolve_reference_object(
        mjcf_sensor.reftype.as_deref(),
        mjcf_sensor.refname.as_deref(),
    )?;
    (ot, oi, rt, ri)
};
```

**New helper `resolve_dual_object_side()`:**
```rust
/// Resolve one side of a dual-object sensor (geom1/body1 or geom2/body2).
///
/// Exactly one of geom_name or body_name must be Some (validated by parser).
/// Returns (MjObjectType::Geom, id) or (MjObjectType::Body, id).
fn resolve_dual_object_side(
    &self,
    geom_name: Option<&str>,
    body_name: Option<&str>,
    side: &str,
) -> std::result::Result<(MjObjectType, usize), ModelConversionError> {
    match (geom_name, body_name) {
        (Some(name), None) => {
            let id = *self
                .geom_name_to_id
                .get(name)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "distance sensor references unknown geom{side} '{name}'"
                    ),
                })?;
            Ok((MjObjectType::Geom, id))
        }
        (None, Some(name)) => {
            let id = *self
                .body_name_to_id
                .get(name)
                .ok_or_else(|| ModelConversionError {
                    message: format!(
                        "distance sensor references unknown body{side} '{name}'"
                    ),
                })?;
            Ok((MjObjectType::Body, id))
        }
        _ => Err(ModelConversionError {
            message: format!(
                "distance sensor: exactly one of (geom{side}, body{side}) \
                 must be specified"
            ),
        }),
    }
}
```

### S7. Position-stage evaluation — Clock + GeomDist/Normal/FromTo

**File:** `core/src/sensor/position.rs`
**MuJoCo equivalent:** `mj_computeSensorPos()` in `engine_sensor.c`
**Design decision:** Add 2 match arms in `mj_sensor_pos()`:
(1) Clock — trivial read of `data.time`.
(2) `GeomDist | GeomNormal | GeomFromTo` — combined arm with internal
type dispatch, mirroring MuJoCo's fall-through case block and the existing
`FrameXAxis | FrameYAxis | FrameZAxis` pattern at line 213.

**Clock arm** (add before `_ => {}` wildcard):
```rust
MjSensorType::Clock => {
    let adr = model.sensor_adr[sensor_id];
    sensor_write(&mut data.sensordata, adr, 0, data.time);
}
```

**GeomDist/Normal/FromTo arm** (add before `_ => {}` wildcard):
```rust
MjSensorType::GeomDist
| MjSensorType::GeomNormal
| MjSensorType::GeomFromTo => {
    let adr = model.sensor_adr[sensor_id];
    let objtype = model.sensor_objtype[sensor_id];
    let objid = model.sensor_objid[sensor_id];
    let reftype = model.sensor_reftype[sensor_id];
    let refid = model.sensor_refid[sensor_id];
    let cutoff = model.sensor_cutoff[sensor_id];
    let sensor_type = model.sensor_type[sensor_id];

    // Resolve geom lists — body ⇒ iterate body's direct geoms; geom ⇒ single
    let (n1, id1) = if objtype == MjObjectType::Body {
        (model.body_geom_num[objid], model.body_geom_adr[objid])
    } else {
        (1, objid)
    };
    let (n2, id2) = if reftype == MjObjectType::Body {
        (model.body_geom_num[refid], model.body_geom_adr[refid])
    } else {
        (1, refid)
    };

    // All-pairs minimum distance
    let mut dist = cutoff;
    let mut fromto = [0.0f64; 6];
    for geom1 in id1..id1 + n1 {
        for geom2 in id2..id2 + n2 {
            let (dist_new, fromto_new) = geom_distance(
                model, data, geom1, geom2, cutoff,
            );
            if dist_new < dist {
                dist = dist_new;
                fromto = fromto_new;
            }
        }
    }

    // Per-type output
    match sensor_type {
        MjSensorType::GeomDist => {
            sensor_write(&mut data.sensordata, adr, 0, dist);
        }
        MjSensorType::GeomNormal => {
            let normal = Vector3::new(
                fromto[3] - fromto[0],
                fromto[4] - fromto[1],
                fromto[5] - fromto[2],
            );
            // Zero-vector guard: only normalize if non-zero
            let output = if normal.norm_squared() > 0.0 {
                normal.normalize()
            } else {
                normal // stays [0, 0, 0]
            };
            sensor_write3(&mut data.sensordata, adr, &output);
        }
        MjSensorType::GeomFromTo => {
            sensor_write6(&mut data.sensordata, adr, &fromto);
        }
        _ => unreachable!(),
    }
}
```

Note: `geom_distance()` takes `cutoff` and returns the capped distance
(matching `mj_geomDistance()` behavior). The `if dist_new < dist` guard
matches MuJoCo exactly — strict `<`, so first-found pair wins on ties.

### S8. Acceleration-stage evaluation — JointActuatorFrc

**File:** `core/src/sensor/acceleration.rs`
**MuJoCo equivalent:** `mj_computeSensorAcc()` case `mjSENS_JOINTACTFRC`
in `engine_sensor.c`
**Design decision:** Add one match arm in `mj_sensor_acc()`.
`JointActuatorFrc` reads `qfrc_actuator` directly — it does NOT need
body accumulators (`cacc`/`cfrc_int`), so it does NOT need to trigger
the lazy gate at line 63. It reads from `data.qfrc_actuator` which is
populated during `mj_fwdActuation`.

**After** (add before `_ => {}` wildcard in the main match at line 81):
```rust
MjSensorType::JointActuatorFrc => {
    let objid = model.sensor_objid[sensor_id];
    let adr = model.sensor_adr[sensor_id];
    let dof_adr = model.jnt_dof_adr[objid];
    sensor_write(&mut data.sensordata, adr, 0, data.qfrc_actuator[dof_adr]);
}
```

### S9. Postprocess cutoff exemptions

**File:** `core/src/sensor/postprocess.rs`
**MuJoCo equivalent:** `apply_cutoff()` in `engine_sensor.c:64–89`
**Design decision:** AD-3 option (a) from the rubric — explicit type match.
Add a skip guard for `GeomFromTo` and `GeomNormal` before the per-element
loop (between lines 55–57). MuJoCo exempts `GeomFromTo` explicitly via type
check and `GeomNormal` implicitly via AXIS datatype (only REAL/POSITIVE
get clamped). Since CortenForge's `MjSensorDataType` stores pipeline stage
(not data kind), we use explicit type matching for both.

**Before** (current code at lines 55–71):
```rust
if cutoff > 0.0 {
    let sensor_type = model.sensor_type[sensor_id];
    for i in 0..dim {
        let idx = adr + i;
        if idx < data.sensordata.len() {
            let clamped = match sensor_type {
                // Positive-type sensors: only clamp on positive side
                MjSensorType::Touch | MjSensorType::Rangefinder => {
                    data.sensordata[idx].min(cutoff)
                }
                // Real-type sensors: clamp both sides
                _ => data.sensordata[idx].clamp(-cutoff, cutoff),
            };
            sensor_write(&mut data.sensordata, adr, i, clamped);
        }
    }
}
```

**After:**
```rust
if cutoff > 0.0 {
    let sensor_type = model.sensor_type[sensor_id];

    // Skip cutoff clamping for exempt sensor types:
    // - GeomFromTo: MuJoCo explicitly returns before clamping
    //   (cutoff used only as mj_geomDistance search radius)
    // - GeomNormal: MuJoCo implicitly skips via AXIS datatype
    //   (only REAL/POSITIVE branches execute)
    // - Touch (mjSENS_CONTACT): MuJoCo skips in apply_cutoff()
    //   (CortenForge handles Touch with positive-only clamping below,
    //    which is correct per MuJoCo's separate sensor_cutoff mechanism
    //    for touch — see acceleration.rs touch evaluation)
    if matches!(
        sensor_type,
        MjSensorType::GeomFromTo | MjSensorType::GeomNormal
    ) {
        continue;
    }

    for i in 0..dim {
        let idx = adr + i;
        if idx < data.sensordata.len() {
            let clamped = match sensor_type {
                MjSensorType::Touch | MjSensorType::Rangefinder => {
                    data.sensordata[idx].min(cutoff)
                }
                _ => data.sensordata[idx].clamp(-cutoff, cutoff),
            };
            sensor_write(&mut data.sensordata, adr, i, clamped);
        }
    }
}
```

Note: The `continue` skips the entire per-element loop for these types,
matching MuJoCo's early return before the `for j` loop in `apply_cutoff()`.

### S10. Fusestatic body protection

**File:** `mjcf/src/builder/compiler.rs`
**MuJoCo equivalent:** Body fusion exclusion for sensor-referenced bodies
**Design decision:** Add `Distance | Normal | Fromto` to the sensor body
protection match in `apply_fusestatic()` at line 85. Only `body1`/`body2`
references need protection — geom references don't need protection because
geoms are never fused. The match arm must read `sensor.body1` and
`sensor.body2` (new `MjcfSensor` fields from S4), NOT `sensor.objname`.

**After** (add new arm before `_ => {}` at line 102):
```rust
MjcfSensorType::Distance
| MjcfSensorType::Normal
| MjcfSensorType::Fromto => {
    if let Some(ref name) = sensor.body1 {
        protected.insert(name.clone());
    }
    if let Some(ref name) = sensor.body2 {
        protected.insert(name.clone());
    }
}
```

---

## Acceptance Criteria

### AC1: Clock reads `data.time` *(runtime test — analytically derived)*
**Given:** Model with `<clock name="clk"/>`, `timestep=0.01`
**After:** `mj_forward()` at t=0
**Assert:** `sensordata[clock_adr] == 0.0` (exact)
**Field:** `Data.sensordata`

### AC2: Clock multi-step time progression *(runtime test — analytically derived)*
**Given:** Model with `<clock name="clk"/>`, `timestep=0.01`
**After:** 3 calls to `mj_step()`
**Assert:** `sensordata[clock_adr] == 0.02` (formula: `(3-1)*0.01`)
**Field:** `Data.sensordata`

### AC3: JointActuatorFrc two-actuator net force *(runtime test — MuJoCo-verified)*
**Given:** Model with one hinge joint, two motor actuators, `ctrl=[3.0, 4.0]`,
`gear=[1.0, 2.0]`, `<jointactuatorfrc name="jaf" joint="j1"/>`
**After:** `mj_step()`
**Assert:** `sensordata[jaf_adr] == 11.0 ± 1e-12` (3.0*1.0 + 4.0*2.0)
**Field:** `Data.sensordata`
**Stage-correctness validation:** This AC implicitly validates correct
acceleration-stage placement. `qfrc_actuator` is only populated during
`mj_fwdActuation` (acceleration pipeline). If the sensor were incorrectly
placed in the position stage, `qfrc_actuator[dof_adr]` would read 0.0
(zero-initialized data), and this AC would fail (0.0 ≠ 11.0). A non-zero
expected value with non-zero ctrl+gear is the minimal test for stage
correctness.

### AC4: JointActuatorFrc zero-actuator case *(runtime test — analytically derived)*
**Given:** Model with one hinge joint, no actuators,
`<jointactuatorfrc name="jaf" joint="j1"/>`
**After:** `mj_step()`
**Assert:** `sensordata[jaf_adr] == 0.0` (exact)
**Field:** `Data.sensordata`

### AC5: JointActuatorFrc rejects ball joint *(runtime test — code review)*
**Given:** Model with ball joint, `<jointactuatorfrc joint="ball_j"/>`
**After:** Model compilation
**Assert:** Builder returns error containing "must be slide or hinge"
**Field:** Model compilation error

### AC6: GeomDist sphere-sphere *(runtime test — MuJoCo-verified)*
**Given:** Two spheres: g1 at (0,0,0) r=0.1, g2 at (1,0,0) r=0.2,
`<distance name="d" geom1="g1" geom2="g2" cutoff="10"/>`
**After:** `mj_forward()`
**Assert:** `sensordata[d_adr] == 0.7 ± 1e-10`
**Field:** `Data.sensordata`

### AC7: GeomNormal sphere-sphere *(runtime test — MuJoCo-verified)*
**Given:** Same model as AC6,
`<normal name="n" geom1="g1" geom2="g2" cutoff="10"/>`
**After:** `mj_forward()`
**Assert:** `sensordata[n_adr..n_adr+3] == [1.0, 0.0, 0.0] ± 1e-10`
**Field:** `Data.sensordata`

### AC8: GeomFromTo sphere-sphere *(runtime test — MuJoCo-verified)*
**Given:** Same model as AC6,
`<fromto name="ft" geom1="g1" geom2="g2" cutoff="10"/>`
**After:** `mj_forward()`
**Assert:** `sensordata[ft_adr..ft_adr+6] == [0.1, 0.0, 0.0, 0.8, 0.0, 0.0] ± 1e-10`
**Field:** `Data.sensordata`

### AC9: Cutoff=0 suppresses positive distance *(runtime test — MuJoCo-verified)*
**Given:** Two non-overlapping spheres, `<distance geom1="g1" geom2="g2"/>`
(cutoff omitted → defaults to 0.0)
**After:** `mj_forward()`
**Assert:** `sensordata[d_adr] == 0.0` (exact)
**Field:** `Data.sensordata`

### AC10: Cutoff=0 does NOT suppress penetration *(runtime test — MuJoCo-verified)*
**Given:** Overlapping spheres: g1 at (0,0,0) r=0.5, g2 at (0.3,0,0) r=0.5,
`<distance geom1="g1" geom2="g2"/>` (cutoff=0)
**After:** `mj_forward()`
**Assert:** `sensordata[d_adr] == -0.7 ± 1e-10`
**Field:** `Data.sensordata`

### AC11: Multi-geom body distance *(runtime test — MuJoCo-verified)*
**Given:** b1 has g1a(0,0,0 r=0.1), g1b(0.3,0,0 r=0.1); b2 has g2(1,0,0 r=0.1).
`<distance body1="b1" body2="b2" cutoff="10"/>`
**After:** `mj_forward()`
**Assert:** `sensordata[d_adr] == 0.5 ± 1e-10` (min of g1b↔g2 = 0.5)
**Field:** `Data.sensordata`

### AC12: GeomFromTo cutoff exemption *(runtime test — MuJoCo-verified)*
**Given:** Penetrating spheres with cutoff=0.5 (actual dist=-0.7),
`<fromto geom1="g1" geom2="g2" cutoff="0.5"/>`
**After:** `mj_forward()`
**Assert:** `fromto` values are NOT clamped to [-0.5, 0.5] per element.
Output matches unclamped fromto points: `[0.5, 0, 0, -0.2, 0, 0] ± 1e-10`
**Field:** `Data.sensordata`

### AC13: GeomDist postprocess clamping *(runtime test — MuJoCo-verified)*
**Given:** Penetrating spheres (actual dist=-0.7), cutoff=0.5,
`<distance geom1="g1" geom2="g2" cutoff="0.5"/>`
**After:** `mj_forward()`
**Assert:** `sensordata[d_adr] == -0.5 ± 1e-10` (clamped from -0.7 to -0.5)
**Field:** `Data.sensordata`

### AC14: GeomNormal cutoff exemption *(runtime test — MuJoCo-verified)*
**Given:** Penetrating spheres (actual dist=-0.7, normal=[-1,0,0]), cutoff=0.5,
`<normal geom1="g1" geom2="g2" cutoff="0.5"/>`
**After:** `mj_forward()`
**Assert:** `sensordata[n_adr..n_adr+3] == [-1.0, 0.0, 0.0] ± 1e-10`
(NOT clamped — values outside [-0.5, 0.5] preserved)
**Field:** `Data.sensordata`

### AC15: Parser recognizes new element names *(runtime test)*
**Given:** MJCF with `<clock/>`, `<jointactuatorfrc joint="j"/>`,
`<distance geom1="g1" geom2="g2"/>`, `<normal geom1="g1" geom2="g2"/>`,
`<fromto geom1="g1" geom2="g2"/>`
**After:** Parse MJCF
**Assert:** 5 sensors parsed with correct `MjcfSensorType` variants
**Field:** `Vec<MjcfSensor>`

### AC16: Strict XOR validation *(runtime test)*
**Given:** MJCF with `<distance geom1="g1" body1="b1" geom2="g2"/>`
(both geom1 and body1 on side 1)
**After:** Parse MJCF
**Assert:** Warning logged about "exactly one of (geom1, body1)"
**Field:** Parser warning

### AC17: Code review — no `unsafe` blocks *(code review)*
No `unsafe` blocks in new code.

### AC18: Code review — wildcard audit *(code review)*
All 7 wildcard match sites from EGT-8 Table 3/4/5 have been updated:
`from_str()`, `mj_sensor_pos()`, `mj_sensor_acc()`, `postprocess.rs`
cutoff, `apply_fusestatic()`. (Builder matches are exhaustive — compiler
enforces.)

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (clock at t=0) | T1 | Direct |
| AC2 (clock multi-step) | T2 | Direct |
| AC3 (JointActuatorFrc net force) | T3 | Direct |
| AC4 (JointActuatorFrc zero-actuator) | T4 | Edge case |
| AC5 (JointActuatorFrc ball reject) | T5 | Negative |
| AC6 (GeomDist sphere-sphere) | T6 | Direct |
| AC7 (GeomNormal sphere-sphere) | T7 | Direct |
| AC8 (GeomFromTo sphere-sphere) | T8 | Direct |
| AC9 (cutoff=0 suppression) | T9 | Edge case |
| AC10 (cutoff=0 penetration) | T10 | Edge case |
| AC11 (multi-geom body) | T11 | Direct |
| AC12 (GeomFromTo cutoff exemption) | T12 | Edge case |
| AC13 (GeomDist postprocess clamp) | T13 | Edge case |
| AC14 (GeomNormal cutoff exemption) | T14 | Edge case |
| AC15 (parser element names) | T15 | Direct |
| AC16 (strict XOR validation) | T16 | Negative |
| AC17 (no unsafe) | — | Code review (manual) |
| AC18 (wildcard audit) | — | Code review (manual) |

---

## Test Plan

### T1: Clock at t=0 → AC1
Model: `<clock name="clk"/>`, `timestep=0.01`. Call `mj_forward()`.
Assert `sensordata[0] == 0.0`.
Expected value analytically derived (V1 from rubric EGT-6).

### T2: Clock multi-step → AC2
Model: same as T1. Call `mj_step()` 3 times.
Assert `sensordata[0] == 0.02` (formula: `(N-1)*timestep = 2*0.01`).
Expected value analytically derived (V1).

### T3: JointActuatorFrc two-actuator → AC3
Model: one hinge joint, two motors with `gear=[1,2]`, `ctrl=[3,4]`.
`<jointactuatorfrc name="jaf" joint="j1"/>`.
Call `mj_step()`. Assert `sensordata[jaf_adr] == 11.0 ± 1e-12`.
MuJoCo-verified (EGT-2). Analytical backing (V2).

### T4: JointActuatorFrc zero-actuator → AC4
Model: one hinge joint, no actuators.
`<jointactuatorfrc name="jaf" joint="j1"/>`.
Call `mj_step()`. Assert `sensordata[jaf_adr] == 0.0`.
Analytically derived (qfrc_actuator zeroed at reset).

### T5: JointActuatorFrc ball joint rejection → AC5
Model: ball joint, `<jointactuatorfrc joint="ball_j"/>`.
Assert builder returns `Err` containing "must be slide or hinge".

### T6: GeomDist sphere-sphere → AC6
Model: g1 at (0,0,0) r=0.1, g2 at (1,0,0) r=0.2.
`<distance name="d" geom1="g1" geom2="g2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[d_adr] == 0.7 ± 1e-10`.
MuJoCo-verified (EGT-3). Analytical backing (V3).

### T7: GeomNormal sphere-sphere → AC7
Model: same geoms. `<normal name="n" geom1="g1" geom2="g2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[n_adr..+3] == [1.0, 0.0, 0.0] ± 1e-10`.
MuJoCo-verified (EGT-3). Analytical backing (V4).

### T8: GeomFromTo sphere-sphere → AC8
Model: same geoms. `<fromto name="ft" geom1="g1" geom2="g2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[ft_adr..+6] == [0.1, 0, 0, 0.8, 0, 0] ± 1e-10`.
MuJoCo-verified (EGT-3). Analytical backing (V5).

### T9: Cutoff=0 non-penetrating → AC9
Model: non-overlapping spheres. `<distance geom1="g1" geom2="g2"/>` (no cutoff).
Call `mj_forward()`. Assert `sensordata[d_adr] == 0.0`.
MuJoCo-verified (EGT-3). Analytical backing (V7).

### T10: Cutoff=0 penetrating → AC10
Model: overlapping spheres (g1 r=0.5, g2 at 0.3 r=0.5).
`<distance geom1="g1" geom2="g2"/>` (cutoff=0).
Call `mj_forward()`. Assert `sensordata[d_adr] == -0.7 ± 1e-10`.
MuJoCo-verified (EGT-3). Analytical backing (V6).

### T11: Multi-geom body distance → AC11
Model: b1 with g1a(0,0,0 r=0.1) + g1b(0.3,0,0 r=0.1), b2 with g2(1,0,0 r=0.1).
`<distance body1="b1" body2="b2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[d_adr] == 0.5 ± 1e-10`.
MuJoCo-verified (EGT-3). Analytical backing (V8).

### T12: GeomFromTo cutoff exemption → AC12
Model: overlapping spheres (dist=-0.7), cutoff=0.5.
`<fromto geom1="g1" geom2="g2" cutoff="0.5"/>`.
Call `mj_forward()`. Assert fromto = `[0.5, 0, 0, -0.2, 0, 0] ± 1e-10`.
(Postprocess does NOT clamp these values.)
MuJoCo-verified (EGT-3). Analytical backing (V9).

### T13: GeomDist postprocess clamp → AC13
Model: overlapping spheres (actual dist=-0.7), cutoff=0.5.
`<distance geom1="g1" geom2="g2" cutoff="0.5"/>`.
Call `mj_forward()`. Assert `sensordata[d_adr] == -0.5 ± 1e-10`.
MuJoCo-verified (EGT-3). Analytical backing (V10).

### T14: GeomNormal cutoff exemption → AC14
Model: overlapping spheres (normal=[-1,0,0]), cutoff=0.5.
`<normal geom1="g1" geom2="g2" cutoff="0.5"/>`.
Call `mj_forward()`. Assert `sensordata[n_adr..+3] == [-1.0, 0.0, 0.0] ± 1e-10`.
(NOT clamped to [-0.5, 0.5].)
MuJoCo-verified (EGT-3). Analytical backing (V9).

### T15: Parser recognizes all 5 element names → AC15
MJCF string with all 5 sensor elements. Parse and assert correct
`MjcfSensorType` variants. Assert `geom1`/`geom2` fields populated for
distance/normal/fromto.

### T16: Strict XOR validation → AC16
MJCF with both `geom1` and `body1` on same side. Assert warning logged.
MJCF with neither `geom1` nor `body1`. Assert warning logged.

### T17: Beyond-cutoff asymmetry → supplementary
Model: g1 at (0,0,0) r=0.1, g2 at (10,0,0) r=0.1, cutoff=1.
`<distance>` → 1.0, `<normal>` → [0,0,0], `<fromto>` → [0,0,0,0,0,0].
MuJoCo-verified (EGT-3).

### T18: FromTo ordering (swap geom order) → supplementary
Model: same geoms as T8 but `geom1="g2" geom2="g1"`.
Assert fromto = `[0.8, 0, 0, 0.1, 0, 0]` (swapped).
MuJoCo-verified (EGT-3).

### T19: Mixed objtype/reftype → supplementary
Model: `<distance geom1="g1" body2="b2" cutoff="10"/>`.
Assert distance computed correctly using geom1 (single) × body2 (all geoms).

### T20: Clock with cutoff → supplementary
Model: `<clock name="clk" cutoff="5"/>`, timestep=1.
Step 10 times. Assert sensordata clamped at 5.0 (postprocess clamps).
MuJoCo-verified (EGT-3).

### T21: JointActuatorFrc with cutoff → supplementary
Model: `<jointactuatorfrc joint="j1" cutoff="5"/>`, ctrl=10.
Assert sensor = 5.0 (clamped from qfrc_actuator=10).
MuJoCo-verified (EGT-3).

### T22: Coincident geoms → supplementary
Model: g1 at (0,0,0) r=0.5, g2 at (0,0,0) r=0.5 (same position, different IDs).
`<distance geom1="g1" geom2="g2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[d_adr] == -1.0 ± 1e-10` (full
penetration: -2r = -2*0.5 = -1.0).
MuJoCo-verified (EGT-3: coincident geoms → -1.0).

### T23: Body with zero geoms → supplementary
Model: body b1 with no geoms (only a child body), body b2 with geom g2.
`<distance body1="b1" body2="b2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[d_adr] == 10.0` (cutoff init —
body_geom_num[b1]=0, loop never executes, dist stays at cutoff).
MuJoCo-verified (EGT-3).

### T24: Self-distance rejection → supplementary
Model: `<distance geom1="g1" geom2="g1" cutoff="10"/>` (same geom both sides).
Assert: builder/compiler rejects with error containing
"1st body/geom must be different from 2nd" (or equivalent validation).
MuJoCo-verified (EGT-3: compiler rejects self-distance).

### T25: Body-pair uses direct geoms only (not subtree) → supplementary
Model: b1 at (0,0,0) with direct geom g1_parent (r=0.1) + child body
b1_child at (1.5,0,0) with g1_child (r=0.1). b2 at (2,0,0) with g2 (r=0.1).
`<distance body1="b1" body2="b2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[d_adr] == 1.8 ± 1e-10`
(g1_parent↔g2 = |2.0-0.0|-0.1-0.1 = 1.8, NOT g1_child↔g2 = |2.0-1.5|-0.1-0.1 = 0.3).
Validates body_geom_num/body_geom_adr dispatch uses direct geoms only.
MuJoCo-verified (EGT-3: body-pair subtree test).

### T26: Same-body geoms → supplementary
Model: body b1 with g1 at (0,0,0 r=0.1) and g2 at (0.5,0,0 r=0.1).
`<distance geom1="g1" geom2="g2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[d_adr] == 0.3 ± 1e-10`
(|0.5-0.0| - 0.1 - 0.1 = 0.3). Two geoms on same body is valid.
MuJoCo-verified (EGT-3: same-body geoms accepted).

### T27: Noise determinism → supplementary
Model: `<distance geom1="g1" geom2="g2" cutoff="10" noise="0.1"/>`.
Call `mj_forward()` twice with identical state.
Assert sensordata values are bitwise identical across both calls.
Validates noise is metadata-only, not applied during mj_forward.
MuJoCo-verified (EGT-3).

### T28: Non-sphere geom type (box-sphere) → supplementary
Model: g1=box (size=0.5,0.5,0.5) at (0,0,0), g2=sphere (r=0.3) at (2,0,0).
`<distance geom1="g1" geom2="g2" cutoff="10"/>`.
Call `mj_forward()`. Assert `sensordata[d_adr] == 1.2 ± 1e-6`
(|2.0-0.0| - 0.5 - 0.3 = 1.2 — box half-extent along x = size[0] = 0.5).
Validates geom_distance works with non-sphere geom types via GJK.
Tolerance 1e-6 (GJK numerical precision for non-sphere pairs).
MuJoCo-verified (EGT-3: box-sphere = 1.2).

### T29: Negative cutoff rejection → supplementary
Model: `<distance geom1="g1" geom2="g2" cutoff="-1"/>`.
Assert: builder/compiler rejects or warns about negative cutoff.
MuJoCo-verified (EGT-3: "negative cutoff in sensor").
**Note:** CortenForge may not implement this validation in Spec C
(see Out of Scope). If deferred, this test is a placeholder for future
compiler validation work.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Cutoff=0 non-penetrating | Positive distances suppressed (non-obvious) | T9 | AC9 |
| Cutoff=0 penetrating | Penetration NOT suppressed (asymmetry) | T10 | AC10 |
| Multi-geom body | N×M all-pairs minimum | T11 | AC11 |
| GeomFromTo cutoff exemption | Postprocess must skip this type | T12 | AC12 |
| GeomNormal cutoff exemption | Postprocess must skip this type | T14 | AC14 |
| GeomDist postprocess clamp | Penetration clamped to [-cutoff, cutoff] | T13 | AC13 |
| Beyond-cutoff asymmetry | Dist→cutoff, normal→[0,0,0], fromto→zeros | T17 | — |
| FromTo swap ordering | From=geom1 surface, To=geom2 surface | T18 | — |
| Mixed geom/body | geom1+body2 and body1+geom2 valid | T19 | — |
| Clock cutoff clamping | Universal cutoff applies to Clock | T20 | — |
| JointActuatorFrc cutoff | Universal cutoff applies to JointActuatorFrc | T21 | — |
| Ball joint rejection | Compiler rejects non-hinge/slide joints | T5 | AC5 |
| Zero-actuator joint | qfrc_actuator reads 0 | T4 | AC4 |
| Strict XOR validation | Both or neither geom/body is parse error | T16 | AC16 |
| Clock time lag | After N steps, sensor reads (N-1)*timestep | T2 | AC2 |
| Coincident geoms | Full penetration = -2r, arbitrary normal | T22 | — |
| Body with zero geoms | Loop never executes, returns cutoff init | T23 | — |
| Self-distance rejection | Compiler rejects geom1==geom2 | T24 | — |
| Body-pair direct geoms only | Uses body_geom_num, NOT subtree | T25 | — |
| Same-body geoms | Two geoms on same body is valid | T26 | — |
| Noise determinism | Noise is metadata-only, not applied at runtime | T27 | — |
| Non-sphere geom types | geom_distance works with box, capsule, etc. | T28 | — |
| Negative cutoff rejection | Compiler rejects negative cutoff values | T29 | — |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T17 (beyond-cutoff) | All 3 geom sensors with beyond-cutoff input | Validates the critical asymmetry between dist/normal/fromto when geoms are too far apart |
| T18 (fromto swap) | GeomFromTo with swapped geom1/geom2 | Validates from/to ordering matches MuJoCo |
| T19 (mixed obj/ref) | Distance with geom1+body2 | Validates mixed objtype/reftype works |
| T20 (clock cutoff) | Clock with cutoff=5 | Validates universal cutoff applies to new types |
| T21 (jointactfrc cutoff) | JointActuatorFrc with cutoff=5 | Validates universal cutoff applies to new types |
| T22 (coincident geoms) | Distance/normal/fromto for coincident geoms | Validates full-penetration edge case (-2r) |
| T23 (zero-geom body) | Body with no geoms in distance sensor | Validates loop-skip returns cutoff init value |
| T24 (self-distance) | geom1==geom2 rejection | Validates compiler rejects self-referencing |
| T25 (subtree) | Body-pair uses direct geoms only | Validates body_geom_num dispatch, NOT child traversal |
| T26 (same-body) | Two geoms on same body | Validates no same-body restriction |
| T27 (noise determinism) | Noise attribute doesn't affect mj_forward | Validates deterministic output with noise metadata |
| T28 (non-sphere) | Box-sphere distance | Validates GJK-based distance for non-sphere pairs |
| T29 (negative cutoff) | Negative cutoff rejection | Validates compiler/builder error (placeholder if deferred) |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| 5 new `MjSensorType` variants | Enum has 32 variants | Enum has 37 variants | Toward MuJoCo | All exhaustive matches on `MjSensorType` — **compiler enforces** (`dim()` at `enums.rs:426`, `sensor_datatype()` at `builder/sensor.rs:404`, `resolve_sensor_object()` at `builder/sensor.rs:93`, `convert_sensor_type()` at `builder/sensor.rs:366`) | Add match arms — compiler error guides |
| 5 new `MjcfSensorType` variants | Enum has 31 variants | Enum has 36 variants | Toward MuJoCo | Exhaustive matches (`as_str()`, `dim()`) + wildcard (`from_str()`) | Add match arms — compiler error for exhaustive; manual for `from_str()` wildcard |
| 4 new `MjcfSensor` fields | Struct has 9 fields | Struct has 13 fields | Toward MuJoCo | All `MjcfSensor` construction sites, `Default` impl | Fields are `Option<String>` defaulting to `None` — transparent |
| `sensor_write6()` added | No 6D write helper | `sensor_write6()` available | N/A (new code) | None — additive | None |
| `geom_distance()` added | No geom distance function | `geom_distance()` available | Toward MuJoCo | None — new code | None |
| GeomFromTo/GeomNormal cutoff exemption | Cutoff applies to all non-Touch/Rangefinder sensors | Cutoff skips GeomFromTo and GeomNormal | Toward MuJoCo | None — new sensor types only | None |
| GJK distance extension | GJK returns bool only (intersection) | GJK returns distance + closest points | N/A (infrastructure) | None — additive | None |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `core/src/types/enums.rs` | 5 new `MjSensorType` variants + `dim()` arms | +15 |
| `core/src/sensor/postprocess.rs` | `sensor_write6()` + cutoff exemption guard | +20 |
| `core/src/sensor/position.rs` | Clock arm + GeomDist/Normal/FromTo combined arm | +60 |
| `core/src/sensor/acceleration.rs` | JointActuatorFrc arm | +6 |
| `core/src/gjk_epa.rs` | `gjk_closest_points()` — GJK distance extension | +80 |
| `core/src/sensor/geom_distance.rs` (new) | `geom_distance()` helper | +120 |
| `mjcf/src/types.rs` | 5 `MjcfSensorType` variants + methods + 4 `MjcfSensor` fields | +30 |
| `mjcf/src/parser.rs` | Dual-object attribute parsing + XOR validation | +25 |
| `mjcf/src/builder/sensor.rs` | `convert_sensor_type()`, `sensor_datatype()`, `resolve_sensor_object()`, `resolve_dual_object_side()`, `process_sensors()` modification | +60 |
| `mjcf/src/builder/compiler.rs` | `apply_fusestatic()` body1/body2 protection | +8 |
| Test files (new) | T1–T29 | +400 |

### Existing Test Impact

**144 existing sensor tests across 5 files. All expected to pass unchanged.**

Spec C adds new enum variants and new match arms but does NOT modify any
existing match arm logic. The only file-level change that affects existing
code is the postprocess cutoff guard (S9), which adds a `continue` before
the existing per-element loop — but only for the new `GeomFromTo` and
`GeomNormal` types, which don't exist in any current test model.

**sim/L0/core/src/sensor/mod.rs (32 tests):**

| Test | Line | Expected impact | Reason |
|------|------|----------------|--------|
| `test_touch_sensor_no_contact` | 188 | Pass | Touch arm unchanged |
| `test_touch_sensor_with_contact` | 207 | Pass | Touch arm unchanged |
| `test_magnetometer_identity_frame` | 254 | Pass | Magnetometer arm unchanged |
| `test_magnetometer_zero_field` | 284 | Pass | Magnetometer arm unchanged |
| `test_actuator_pos_sensor` | 309 | Pass | ActuatorPos arm unchanged |
| `test_actuator_vel_sensor` | 356 | Pass | ActuatorVel arm unchanged |
| `test_subtree_angmom_at_rest` | 446 | Pass | SubtreeAngMom arm unchanged |
| `test_subtree_angmom_spinning` | 466 | Pass | SubtreeAngMom arm unchanged |
| `test_rangefinder_no_geoms_to_hit` | 496 | Pass | Rangefinder arm unchanged |
| `test_rangefinder_hits_sphere` | 516 | Pass | Rangefinder arm unchanged |
| `test_rangefinder_cutoff_preserves_no_hit_sentinel` | 641 | Pass | Positive-type cutoff unchanged |
| `test_sensor_cutoff_clamps_value` | 730 | Pass | Default cutoff arm unchanged (new types skip via `continue` only for GeomFromTo/GeomNormal) |
| `test_sensor_cutoff_negative_clamps` | 752 | Pass | Same reason |
| `test_sensor_no_cutoff_when_zero` | 772 | Pass | `cutoff > 0.0` guard unchanged |
| `test_force_sensor_at_rest_in_gravity` | 669 | Pass | Force arm unchanged |
| `test_accelerometer_at_rest_reads_gravity` | 955 | Pass | Accelerometer arm unchanged |
| `test_frame_quat_sensor` | 931 | Pass | FrameQuat arm unchanged |
| `test_joint_pos_sensor_reads_correctly` | 895 | Pass | JointPos arm unchanged |
| Remaining 14 tests (actuator, tendon, ball, subtree, etc.) | Various | Pass | No overlap with new sensor types |

**sim/L0/tests/integration/sensors_phase4.rs (44 tests):**

| Test | Line | Expected impact | Reason |
|------|------|----------------|--------|
| `d01_accelerometer_static_gravity` | 48 | Pass | Accelerometer arm unchanged |
| `d04_framelinacc_includes_gravity` | 208 | Pass | FrameLinAcc arm unchanged |
| `d06_force_sensor_static` | 308 | Pass | Force arm unchanged |
| `d11_disable_sensor_acc_stage` | 582 | Pass | Disable flag path unchanged |
| `d12_acc_sensor_sleep_computes_all` | 629 | Pass | Sleep path unchanged |
| `a01_velocimeter_site_offset` | 721 | Pass | Velocimeter arm unchanged |
| `b01_subtree_linvel_free_body` | 1139 | Pass | SubtreeLinVel arm unchanged |
| `c01_no_acc_sensors_flag_false` | 1716 | Pass | Lazy gate unchanged |
| Remaining 36 tests | Various | Pass | No overlap with new sensor types |

**sim/L0/tests/integration/sensor_phase6.rs (39 tests — Spec A + Spec B):**

| Test | Line | Expected impact | Reason |
|------|------|----------------|--------|
| `t01_parser_objtype_attribute` | 16 | Pass | Parser changes are additive (new element names, not modifying existing parsing) |
| `t09_touch_multi_geom_aggregation` | 305 | Pass | Touch arm unchanged |
| `t12_geom_framelinacc_matches_site` | 506 | Pass | FrameLinAcc Geom arm unchanged |
| `t23_framepos_relative_to_rotated_body` | 1059 | Pass | FramePos ref-frame transform unchanged |
| `t26_framelinvel_with_coriolis` | 1189 | Pass | FrameLinVel ref-frame unchanged |
| `t29_acc_unaffected_by_reftype` | 1392 | Pass | Acc stage unchanged |
| Remaining 33 tests | Various | Pass | No overlap with new sensor types |

**sim/L0/tests/integration/mjcf_sensors.rs (8 tests):**

| Test | Line | Expected impact | Reason |
|------|------|----------------|--------|
| All 8 tests (`test_jointpos_sensor_roundtrip`, etc.) | 20–267 | Pass | MJCF parsing is additive — new element names don't affect existing ones |

**sim/L0/tests/integration/sensors.rs (11 tests):**

| Test | Line | Expected impact | Reason |
|------|------|----------------|--------|
| All 11 tests (`test_imu_reads_body_state`, etc.) | 18–215 | Pass | sim-sensor crate tests — independent of sim-core sensor evaluation |

**Compile-time impact:** Adding 5 new `MjSensorType` variants will cause
compile errors in all exhaustive matches (`dim()`, `convert_sensor_type()`,
`sensor_datatype()`, `resolve_sensor_object()`). This is expected and
desirable — the compiler guides the implementer to all required sites.
Non-exhaustive matches (`_ =>` wildcards in `from_str()`, `mj_sensor_pos()`,
`mj_sensor_acc()`, `postprocess.rs`, `apply_fusestatic()`) will NOT error —
these are the danger zones requiring manual audit (AC18).

---

## Execution Order

1. **S1** (enum variants + dim) → compile to verify exhaustive match errors
   propagate correctly to builder
2. **S4** (MjcfSensorType + MjcfSensor struct) → compile to verify parser
   impact
3. **S5** (parser dual-object attributes) → verify parser recognizes new
   elements: run T15, T16
4. **S6** (builder: convert, datatype, resolve) → verify builder compiles
   with new types: run T5 (ball joint rejection), T24 (self-distance
   rejection), T29 (negative cutoff rejection — if implemented)
5. **S2** (sensor_write6) → compile to verify helper availability
6. **S3** (geom_distance helper) → verify with unit tests on sphere-sphere
7. **S7** (position-stage evaluation) → run T1, T2, T6, T7, T8, T9, T10,
   T11, T17, T18, T19, T22, T23, T25, T26, T27, T28
8. **S8** (acceleration-stage evaluation) → run T3, T4
9. **S9** (postprocess cutoff exemptions) → run T12, T13, T14, T20, T21
10. **S10** (fusestatic body protection) → verify with existing fusestatic
    tests

After each section, run:
```
cargo test -p sim-core -p sim-mjcf -p sim-sensor -p sim-conformance-tests
```

Dependencies:
- S1 must land before S6, S7, S8 (new enum variants needed)
- S4 must land before S5, S6 (new MJCF types needed)
- S5 must land before S6 (parser feeds builder)
- S2 must land before S7 (sensor_write6 needed by GeomFromTo)
- S3 must land before S7 (geom_distance needed by geom sensor evaluation)
- S6 must land before S7, S8 (builder stage assignment needed)
- S7, S8 independent of each other (different pipeline stages)
- S9 after S7 (postprocess runs after evaluation — test needs sensor values)
- S10 independent of S7/S8/S9 (compiler-level, not evaluation-level)

Cross-spec dependencies:
- Spec A (commit `28bc9f4`): objtype parsing infrastructure, `resolve_sensor_object()` pattern
- Spec B (commit `fb8ec66`): `resolve_reference_object()` for reftype/refid resolution.
  **Reftype overlap note:** Spec B's `resolve_reference_object()` was designed
  for frame sensors where `reftype` selects a reference frame (body/site).
  GeomDist/Normal/FromTo reuse the `sensor_reftype`/`sensor_refid` model
  arrays for the *second* geom/body, but do NOT use `reftype` in the
  frame-reference sense. This is safe because `resolve_reference_object()`
  already handles `"body"` and `"geom"` string types. The builder maps
  `geom2`→`(Geom, id)` and `body2`→`(Body, id)` into the same arrays.
- No dependency on Spec D (sensor history attributes are independent)

---

## Out of Scope

- **`CamProjection` sensor** — requires camera infrastructure (`cam_xpos`,
  `cam_xmat`, `cam_resolution`, `cam_fovy`, `cam_intrinsic`, `cam_sensorsize`,
  `MjObjectType::Camera`). Deferred to DT-120. Conformance impact: minor —
  camera projection sensors are uncommon in RL/robotics models.

- **`InsideSite` sensor** (`mjSENS_INSIDESITE`) — exists in MuJoCo 3.5.0
  but not listed in umbrella spec. Requires geometric containment testing.
  Tracked as DT-121. Conformance impact: minor.

- **Mesh/Hfield/SDF geom distance** — `geom_distance()` in this spec
  supports convex primitives only (Sphere, Box, Capsule, Cylinder, Ellipsoid).
  Non-convex geom pairs return `(cutoff, [0; 6])` with a warning. Tracked
  as DT-122. Conformance impact: gap for non-convex geometry distance
  queries — acceptable for v1.0 since common RL models use convex primitives.

- **Sensor noise application** — `noise` is parsed and stored but not
  applied at runtime (intentional design for RL training parity).

- **Runtime sensor interpolation** — `nsample`/`interp`/`delay` runtime
  behavior deferred to DT-107/DT-108.

- **Performance optimization** — Phase 6 is correctness/completeness, not
  performance.

- **`GeomPoint` sensor** — umbrella listed this but MuJoCo 3.5.0 does not
  have `<geompoint>` element. Does not exist.

- **Negative cutoff validation** — MuJoCo's compiler rejects negative
  cutoff (`"negative cutoff in sensor"`). CortenForge does not currently
  validate this at parse/compile time. Adding compiler-level validation
  is a cross-cutting concern beyond Spec C scope. Tracked for future work.
