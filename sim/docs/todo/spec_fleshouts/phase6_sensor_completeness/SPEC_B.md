# Phase 6 Spec B — Frame Sensor `reftype`/`refid` (Relative-Frame Measurements)

**Status:** Draft
**Phase:** Roadmap Phase 6 — Sensor Completeness
**Effort:** M
**MuJoCo ref:** `mj_sensorPos()` (`engine_sensor.c:470–750`), `mj_sensorVel()` (`engine_sensor.c:750–950`), `mj_sensorAcc()` (`engine_sensor.c:950–1100`); `get_xpos_xmat()` (`engine_sensor.c:30–70`), `get_xquat()` (`engine_sensor.c:70–110`) helpers; `mj_objectVelocity()` (`engine_core_smooth.c:470–530`). Line ranges approximate for MuJoCo 3.5.0 — use function name + case label as stable anchors
**MuJoCo version:** 3.5.0
**Prerequisites:**
- Phase 6 Spec A (DT-62 objtype parsing, DT-64 touch multi-geom, DT-102 geom acc) — landed in `28bc9f4`
- Spec A established: explicit `objtype` attribute parsing, separated `reftype`/`refname` parser fields (`types.rs:3082–3084`, `parser.rs:3474–3476`), `resolve_sensor_object()` infrastructure (`builder/sensor.rs:74–248`), `MjObjectType::XBody` variant (`enums.rs:503`)

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Problem Statement

**Terminology:** "Relative-frame measurement" (title) refers to the sensor
output — a measurement expressed relative to a reference object's frame.
"Reference-frame transform" (body) refers to the coordinate transform that
converts a world-frame quantity into the reference object's frame. Both terms
describe the same feature; the title names the outcome, the body names the
operation.

**Conformance gap** — MuJoCo implements reference-frame transforms for all
frame sensors in `mj_sensorPos()` and `mj_sensorVel()` (`engine_sensor.c`);
CortenForge does not implement this yet. The MJCF attributes `reftype` and
`refname` are parsed (Spec A, `parser.rs:3474–3476`) and stored in the
`MjcfSensor` struct (`types.rs:3082–3084`), but the builder hardcodes
`sensor_reftype = MjObjectType::None` and `sensor_refid = 0`
(`builder/sensor.rs:47–48`). The evaluation code in `position.rs` and
`velocity.rs` never reads `sensor_reftype` or `sensor_refid`.

Consequence: any MJCF model using `reftype`/`refname` on frame sensors
produces world-frame output instead of relative-frame output. This is a
conformance gap for all position-stage frame sensors (FramePos, FrameQuat,
FrameXAxis, FrameYAxis, FrameZAxis) and all velocity-stage frame sensors
(FrameLinVel, FrameAngVel). MuJoCo's acceleration-stage frame sensors
(FrameLinAcc, FrameAngAcc) intentionally ignore `sensor_refid` — no change
needed there.

---

## MuJoCo Reference

### Position-stage reference-frame transforms (`mj_sensorPos`)

For each position-stage frame sensor, when `sensor_refid[i] >= 0`, MuJoCo
resolves the reference object's position and rotation via `get_xpos_xmat()`
and applies a relative transform.

**FramePos** (`mjSENS_FRAMEPOS` case in `mj_sensorPos`):
```c
get_xpos_xmat(m, d, reftype, refid, refpos, refmat);
get_xpos_xmat(m, d, objtype, objid, objpos, objmat);
mju_sub3(tmp, objpos, refpos);              // tmp = p_obj - p_ref
mju_mulMatTVec3(sensordata, refmat, tmp);   // output = R_ref^T * tmp
```

**FrameQuat** (`mjSENS_FRAMEQUAT` case):
```c
get_xquat(m, d, reftype, refid, refquat);
get_xquat(m, d, objtype, objid, objquat);
mju_negQuat(tmp, refquat);                  // tmp = conjugate(q_ref)
mju_mulQuat(sensordata, tmp, objquat);      // output = q_ref^{-1} * q_obj
```
Note: `mju_negQuat` computes the quaternion **conjugate** `[w, -x, -y, -z]`,
not a full 4-component negation. For unit quaternions, conjugate == inverse.
MuJoCo normalizes the result via `mju_normalize4`.

**FrameXAxis/YAxis/ZAxis** (`mjSENS_FRAMExAXIS` cases):
```c
get_xpos_xmat(m, d, reftype, refid, refpos, refmat);
get_xpos_xmat(m, d, objtype, objid, objpos, objmat);
mju_mulMatTVec3(sensordata, refmat, objmat + 3*col);  // col = 0,1,2 for X,Y,Z
```

### Velocity-stage reference-frame transforms (`mj_sensorVel`)

**FrameLinVel** (`mjSENS_FRAMELINVEL` case in `mj_sensorVel`):
```c
// 1. Object velocity at object position (world frame, flg_local=0)
mj_objectVelocity(m, d, objtype, objid, objvel6, 0);   // [w_obj(3); v_obj(3)]

// 2. Reference velocity at reference position (world frame, flg_local=0)
mj_objectVelocity(m, d, reftype, refid, refvel6, 0);   // [w_ref(3); v_ref(3)]

// 3. Relative velocity with Coriolis correction
get_xpos_xmat(m, d, reftype, refid, refpos, refmat);
get_xpos_xmat(m, d, objtype, objid, objpos, NULL);
mju_sub3(dif, objpos, refpos);            // dif = p_obj - p_ref
mju_cross(tmp, refvel6, dif);             // tmp = w_ref × dif  (angular = refvel6[0..3])
mju_sub3(tmp2, objvel6+3, refvel6+3);     // tmp2 = v_obj - v_ref
mju_subFrom3(tmp2, tmp);                  // tmp2 -= w_ref × dif

// 4. Rotate into reference frame
mju_mulMatTVec3(sensordata, refmat, tmp2); // output = R_ref^T * (v_obj - v_ref - w_ref × dif)
```

**FrameAngVel** (`mjSENS_FRAMEANGVEL` case):
```c
mj_objectVelocity(m, d, objtype, objid, objvel6, 0);
mj_objectVelocity(m, d, reftype, refid, refvel6, 0);
get_xpos_xmat(m, d, reftype, refid, NULL, refmat);

// w_rel = w_obj - w_ref  (NO Coriolis term for angular velocity)
mju_sub3(tmp, objvel6, refvel6);    // angular components are [0..3]

// Rotate into reference frame
mju_mulMatTVec3(sensordata, refmat, tmp);  // output = R_ref^T * (w_obj - w_ref)
```

### Acceleration-stage: no reference-frame transform

`mj_sensorAcc()` in `engine_sensor.c` does **not** read `sensor_refid` for
FrameLinAcc or FrameAngAcc. These sensors always output in world frame
regardless of `reftype`/`refname` MJCF attributes. This is confirmed by
inspection of the C source — there is no `get_xpos_xmat` call for reftype
in the acceleration sensor arms.

### `get_xpos_xmat()` dispatch table

| Object type | MuJoCo enum | Position source | Rotation source |
|-------------|-------------|-----------------|-----------------|
| `body` | `mjOBJ_BODY` (1) | `d->xipos + 3*id` | `d->ximat + 9*id` |
| `xbody` | `mjOBJ_XBODY` (2) | `d->xpos + 3*id` | `d->xmat + 9*id` |
| `geom` | `mjOBJ_GEOM` (5) | `d->geom_xpos + 3*id` | `d->geom_xmat + 9*id` |
| `site` | `mjOBJ_SITE` (6) | `d->site_xpos + 3*id` | `d->site_xmat + 9*id` |
| `camera` | `mjOBJ_CAMERA` (7) | `d->cam_xpos + 3*id` | `d->cam_xmat + 9*id` |

CortenForge field mapping:

| MuJoCo source | CortenForge field | Location |
|---------------|-------------------|----------|
| `d->xipos` | `data.xipos[body_id]` | `data.rs:101` |
| `d->ximat` | `data.ximat[body_id]` | `data.rs:103` |
| `d->xpos` | `data.xpos[body_id]` | `data.rs:95` |
| `d->xmat` | `data.xmat[body_id]` | `data.rs:99` |
| `d->xquat` | `data.xquat[body_id]` | `data.rs:97` |
| `m->body_iquat` | `model.body_iquat[body_id]` | `model.rs:135` |
| `d->geom_xpos` | `data.geom_xpos[geom_id]` | `data.rs:107` |
| `d->geom_xmat` | `data.geom_xmat[geom_id]` | `data.rs:109` |
| `d->site_xpos` | `data.site_xpos[site_id]` | `data.rs:113` |
| `d->site_xmat` | `data.site_xmat[site_id]` | `data.rs:115` |

### `get_xquat()` dispatch table

| Object type | Quaternion computation |
|-------------|----------------------|
| `mjOBJ_XBODY` | `d->xquat[id]` (direct copy) |
| `mjOBJ_BODY` | `mulQuat(d->xquat[body], m->body_iquat[body])` |
| `mjOBJ_GEOM` | `mat2Quat(d->geom_xmat[id])` |
| `mjOBJ_SITE` | `mat2Quat(d->site_xmat[id])` |
| `mjOBJ_CAMERA` | `mat2Quat(d->cam_xmat[id])` |

### Disabled reference frame (`refid == -1`)

When `sensor_refid[i] == -1` (no `reftype`/`refname` in MJCF), MuJoCo skips
the reference-frame transform entirely. Sensor outputs in world frame.

CortenForge convention: `sensor_refid` is `Vec<usize>` (no `-1`). The guard
condition uses `sensor_reftype[i] == MjObjectType::None` (already the default
at `builder/sensor.rs:47`).

### World body as reference

When `reftype="xbody"` and `refname="world"` (body ID 0), the transform
applies but has no effect: `R_world = I`, `p_world = [0,0,0]`. Output is
numerically identical to world-frame. Not a special case in MuJoCo.

### MJCF parsing for `reftype`/`refname`

MuJoCo MJCF parsing:
- `reftype` attribute: string `"body"`, `"site"`, `"geom"`, `"xbody"`, `"camera"`
- `refname` attribute: string naming the reference object
- Both absent: `sensor_reftype = mjOBJ_UNKNOWN`, `sensor_refid = -1` (world frame)
- `refname` given without `reftype`: MuJoCo infers type from name lookup order
- `reftype` given without `refname`: MuJoCo sets `sensor_refid = -1` (no transform)

CortenForge Spec A already parses `reftype` and `refname` as separate
`Option<String>` fields in `MjcfSensor` (`types.rs:3082–3084`). The builder
at `builder/sensor.rs:47–48` currently hardcodes `MjObjectType::None` and `0`.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| FramePos with reftype | `R_ref^T * (p_obj - p_ref)` via `get_xpos_xmat()` in `mj_sensorPos()` | World-frame position only; `sensor_reftype = None`, `sensor_refid = 0` |
| FrameQuat with reftype | `q_ref^{-1} * q_obj` via `get_xquat()` in `mj_sensorPos()` | World-frame quaternion only |
| FrameXAxis/YAxis/ZAxis with reftype | `R_ref^T * R_obj[:, col]` in `mj_sensorPos()` | World-frame axis only |
| FrameLinVel with reftype | `R_ref^T * (v_obj - v_ref - w_ref × dif)` with Coriolis in `mj_sensorVel()` | World-frame velocity only |
| FrameAngVel with reftype | `R_ref^T * (w_obj - w_ref)` in `mj_sensorVel()` | World-frame angular velocity only |
| FrameLinAcc/FrameAngAcc with reftype | Ignores `sensor_refid` — world frame only | World-frame acceleration (correct) |
| Builder reftype/refid resolution | Resolves reftype string → `mjtObj` + name → ID | Hardcoded `None`/`0` |
| No reftype/refname | World-frame output (no transform) | World-frame output (correct) |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `sensor_refid` disabled marker | `int`, `-1` = no ref | `Vec<usize>`, guard via `sensor_reftype == MjObjectType::None` | Use `sensor_reftype[i] == MjObjectType::None` instead of `refid == -1` |
| `mjtObj` enum values | `1,2,5,6,7` for body/xbody/geom/site/camera | `MjObjectType` variants Body/XBody/Geom/Site; Camera not yet implemented (DT-117) | Map string → `MjObjectType` variant; camera → warn + skip (deferred) |
| `mju_mulMatTVec3(dst, mat, vec)` | Row-major `mat^T * vec` | nalgebra column-major `mat.transpose() * vec` | Direct port — same mathematical result |
| `mju_negQuat(dst, q)` | Quaternion **conjugate**: `[w, -x, -y, -z]` (negates imaginary only) | `UnitQuaternion::inverse()` returns conjugate for unit quaternions | Use `q_ref.inverse()` — matches `mju_negQuat` semantics |
| `cvel` spatial velocity layout | `[angular(3); linear(3)]` | `data.cvel[body_id]` — `[0..3]` angular, `[3..6]` linear | `Vector3::new(cvel[0], cvel[1], cvel[2])` for angular |
| `get_xpos_xmat()` dispatch | Position + rotation source by object type | Per-type dispatch in position.rs (lines 108–113) and velocity.rs (lines 135–148) | Reference dispatch uses same arrays but indexed by `sensor_reftype[i]`/`sensor_refid[i]` |
| `mj_objectVelocity()` `flg_local=0` | World-frame velocity output | `object_velocity(data, body_id, &target_pos, None)` at `spatial.rs:264` | Use `local_rot = None` for BOTH primary and reference objects — velocities in world frame, rotated into reference frame only as final step. Using `local_rot = Some(...)` for either object would apply unwanted rotation before relative computation |

---

## Specification

**Conformance rule:** Every algorithm in this section must produce numerically
identical results to the MuJoCo C code cited in the MuJoCo Reference section.

### S1. Builder reference-object resolution

**File:** `sim/L0/mjcf/src/builder/sensor.rs`
**MuJoCo equivalent:** MJCF compiler sensor resolution — resolves `reftype`/`refname` to `sensor_reftype`/`sensor_refid` model arrays
**Design decision:** Mirror the existing `resolve_sensor_object()` pattern
from Spec A for the reference object. The builder resolves `reftype`/`refname`
for ALL sensor types (not just frame sensors), matching MuJoCo's behavior
(MuJoCo resolves at parse time for all sensors; evaluation only reads
`sensor_refid` for frame sensors). This catches typos in `refname` on non-frame
sensors at build time rather than silently ignoring them.

**Before** (`builder/sensor.rs:47–48`):
```rust
self.sensor_reftype.push(MjObjectType::None);
self.sensor_refid.push(0);
```

**After:**
```rust
let (reftype, refid) = self.resolve_reference_object(
    mjcf_sensor.reftype.as_deref(),
    mjcf_sensor.refname.as_deref(),
)?;
self.sensor_reftype.push(reftype);
self.sensor_refid.push(refid);
```

New function `resolve_reference_object`:

```rust
/// Resolve reftype/refname to (MjObjectType, refid).
///
/// Resolution rules:
/// - Both absent → (MjObjectType::None, 0) — no transform
/// - reftype present, refname absent → (MjObjectType::None, 0) — no transform
///   (matches MuJoCo: reftype without refname → sensor_refid = -1)
/// - refname present, reftype absent → infer type from name lookup
///   (site → body → geom priority, mirrors resolve_frame_sensor_by_name)
/// - Both present → dispatch on reftype string, look up refname
fn resolve_reference_object(
    &self,
    reftype_str: Option<&str>,
    refname: Option<&str>,
) -> Result<(MjObjectType, usize), ModelConversionError> {
    let name = match refname {
        Some(n) => n,
        None => return Ok((MjObjectType::None, 0)),
    };

    match reftype_str {
        Some("site") => {
            let id = *self.site_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError {
                    message: format!("sensor refname references unknown site '{name}'"),
                }
            })?;
            Ok((MjObjectType::Site, id))
        }
        Some("body") => {
            let id = *self.body_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError {
                    message: format!("sensor refname references unknown body '{name}'"),
                }
            })?;
            Ok((MjObjectType::Body, id))
        }
        Some("xbody") => {
            let id = *self.body_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError {
                    message: format!("sensor refname references unknown body '{name}'"),
                }
            })?;
            Ok((MjObjectType::XBody, id))
        }
        Some("geom") => {
            let id = *self.geom_name_to_id.get(name).ok_or_else(|| {
                ModelConversionError {
                    message: format!("sensor refname references unknown geom '{name}'"),
                }
            })?;
            Ok((MjObjectType::Geom, id))
        }
        Some("camera") => {
            // DT-117: Camera not yet implemented. Warn and ignore.
            warn!(
                "reftype='camera' not yet supported (DT-117); \
                 ignoring reftype/refname for sensor"
            );
            Ok((MjObjectType::None, 0))
        }
        Some(other) => Err(ModelConversionError {
            message: format!(
                "sensor has invalid reftype '{other}' \
                 (expected site/body/xbody/geom/camera)"
            ),
        }),
        None => {
            // refname without reftype — infer type from name lookup
            // Priority: site → body (as XBody) → geom
            if let Some(&id) = self.site_name_to_id.get(name) {
                Ok((MjObjectType::Site, id))
            } else if let Some(&id) = self.body_name_to_id.get(name) {
                Ok((MjObjectType::XBody, id))
            } else if let Some(&id) = self.geom_name_to_id.get(name) {
                Ok((MjObjectType::Geom, id))
            } else {
                Err(ModelConversionError {
                    message: format!(
                        "sensor refname references unknown object '{name}'"
                    ),
                })
            }
        }
    }
}
```

**Scope of reftype support by sensor type (MuJoCo behavior):**
MuJoCo resolves `reftype`/`refname` at parse time for all sensors. Only frame
sensors (FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis, FrameLinVel, FrameAngVel)
read `sensor_refid` at evaluation time. FrameLinAcc/FrameAngAcc ignore it.
All other sensor types (JointPos, Touch, Gyro, etc.) ignore `sensor_refid`
entirely. The builder resolves for all types, but only evaluation code for
frame sensors uses the result.

### S2. Position-stage reference-frame transforms

**File:** `sim/L0/core/src/sensor/position.rs`
**MuJoCo equivalent:** `mj_sensorPos()` in `engine_sensor.c` — FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis cases with `sensor_refid[i] >= 0`
**Design decision:** Inline the reference-frame transform within each match
arm after the world-frame computation (option (c) from EGT-10 DECISION 2).
This matches MuJoCo's inline approach and keeps each arm self-contained.
The guard condition is `sensor_reftype[sensor_id] != MjObjectType::None`.

Two helper functions will be added to avoid code duplication:

```rust
/// Resolve position and rotation matrix for a reference object.
/// Mirrors MuJoCo's get_xpos_xmat() dispatch.
fn get_ref_pos_mat(
    model: &Model,
    data: &Data,
    reftype: MjObjectType,
    refid: usize,
) -> (Vector3<f64>, Matrix3<f64>) {
    match reftype {
        MjObjectType::Site if refid < model.nsite => {
            (data.site_xpos[refid], data.site_xmat[refid])
        }
        MjObjectType::XBody if refid < model.nbody => {
            (data.xpos[refid], data.xmat[refid])
        }
        MjObjectType::Body if refid < model.nbody => {
            (data.xipos[refid], data.ximat[refid])
        }
        MjObjectType::Geom if refid < model.ngeom => {
            (data.geom_xpos[refid], data.geom_xmat[refid])
        }
        _ => (Vector3::zeros(), Matrix3::identity()),
    }
}

/// Resolve quaternion for a reference object.
/// Mirrors MuJoCo's get_xquat() dispatch.
fn get_ref_quat(
    model: &Model,
    data: &Data,
    reftype: MjObjectType,
    refid: usize,
) -> UnitQuaternion<f64> {
    match reftype {
        MjObjectType::XBody if refid < model.nbody => data.xquat[refid],
        MjObjectType::Body if refid < model.nbody => {
            data.xquat[refid] * model.body_iquat[refid]
        }
        MjObjectType::Site if refid < model.nsite => {
            UnitQuaternion::from_rotation_matrix(
                &nalgebra::Rotation3::from_matrix_unchecked(data.site_xmat[refid]),
            )
        }
        MjObjectType::Geom if refid < model.ngeom => {
            UnitQuaternion::from_rotation_matrix(
                &nalgebra::Rotation3::from_matrix_unchecked(data.geom_xmat[refid]),
            )
        }
        _ => UnitQuaternion::identity(),
    }
}
```

**FramePos** (lines 106–116 → modified):

After the existing world-frame position lookup:
```rust
MjSensorType::FramePos => {
    let pos = match model.sensor_objtype[sensor_id] {
        // ... existing dispatch (unchanged) ...
    };
    // Reference-frame transform (Spec B)
    if model.sensor_reftype[sensor_id] != MjObjectType::None {
        let (ref_pos, ref_mat) = get_ref_pos_mat(
            model, data,
            model.sensor_reftype[sensor_id],
            model.sensor_refid[sensor_id],
        );
        // p_relative = R_ref^T * (p_obj - p_ref)
        let relative = ref_mat.transpose() * (pos - ref_pos);
        sensor_write3(&mut data.sensordata, adr, &relative);
    } else {
        sensor_write3(&mut data.sensordata, adr, &pos);
    }
}
```

**FrameQuat** (lines 118–142 → modified):

After the existing world-frame quaternion lookup:
```rust
MjSensorType::FrameQuat => {
    let quat = match model.sensor_objtype[sensor_id] {
        // ... existing dispatch (unchanged) ...
    };
    if model.sensor_reftype[sensor_id] != MjObjectType::None {
        let ref_quat = get_ref_quat(
            model, data,
            model.sensor_reftype[sensor_id],
            model.sensor_refid[sensor_id],
        );
        // q_relative = q_ref^{-1} * q_obj
        // UnitQuaternion::inverse() returns conjugate (= mju_negQuat for unit quats)
        // MuJoCo normalizes result via mju_normalize4; nalgebra UnitQuaternion
        // maintains unit norm by construction, so no explicit normalization needed.
        let relative = ref_quat.inverse() * quat;
        sensor_write4(&mut data.sensordata, adr, relative.w, relative.i, relative.j, relative.k);
    } else {
        sensor_write4(&mut data.sensordata, adr, quat.w, quat.i, quat.j, quat.k);
    }
}
```

**FrameXAxis/YAxis/ZAxis** (lines 144–163 → modified):

After the existing world-frame axis extraction:
```rust
MjSensorType::FrameXAxis | MjSensorType::FrameYAxis | MjSensorType::FrameZAxis => {
    let mat = match model.sensor_objtype[sensor_id] {
        // ... existing dispatch (unchanged) ...
    };
    let col_idx = match model.sensor_type[sensor_id] {
        MjSensorType::FrameXAxis => 0,
        MjSensorType::FrameYAxis => 1,
        MjSensorType::FrameZAxis => 2,
        _ => 0,
    };
    let col = Vector3::new(mat[(0, col_idx)], mat[(1, col_idx)], mat[(2, col_idx)]);
    if model.sensor_reftype[sensor_id] != MjObjectType::None {
        let (_ref_pos, ref_mat) = get_ref_pos_mat(
            model, data,
            model.sensor_reftype[sensor_id],
            model.sensor_refid[sensor_id],
        );
        // axis_in_ref = R_ref^T * axis_world
        let relative = ref_mat.transpose() * col;
        sensor_write3(&mut data.sensordata, adr, &relative);
    } else {
        sensor_write3(&mut data.sensordata, adr, &col);
    }
}
```

### S3. Velocity-stage reference-frame transforms

**File:** `sim/L0/core/src/sensor/velocity.rs`
**MuJoCo equivalent:** `mj_sensorVel()` in `engine_sensor.c` — FrameLinVel, FrameAngVel cases with `sensor_refid[i] >= 0`
**Design decision:** Same inline approach as S2. The velocity transforms
require calling `object_velocity()` for the reference object. Both primary
and reference objects use `local_rot = None` (world-frame velocities), with
rotation into the reference frame applied as the final step. This matches
MuJoCo's `mj_objectVelocity()` with `flg_local = 0`.

The `get_ref_pos_mat` helper is also needed in this file. Rather than
cross-module sharing (which would require moving it to `sensor/mod.rs` or a
shared module), duplicate it as a file-local function. This matches the
existing pattern: `position.rs` and `velocity.rs` both have independent
object-type dispatch within their match arms.

**FrameLinVel** (lines 134–151 → modified):

```rust
MjSensorType::FrameLinVel => {
    let (body_id, obj_pos) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid])
        }
        MjObjectType::XBody if objid < model.nbody => (objid, data.xpos[objid]),
        MjObjectType::Body if objid < model.nbody => (objid, data.xipos[objid]),
        MjObjectType::Geom if objid < model.ngeom => {
            (model.geom_body[objid], data.geom_xpos[objid])
        }
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };

    if model.sensor_reftype[sensor_id] != MjObjectType::None {
        let reftype = model.sensor_reftype[sensor_id];
        let refid = model.sensor_refid[sensor_id];

        // Reference object body_id and position (bounds-guarded)
        let (ref_body_id, ref_pos) = match reftype {
            MjObjectType::Site if refid < model.nsite => {
                (model.site_body[refid], data.site_xpos[refid])
            }
            MjObjectType::XBody if refid < model.nbody => (refid, data.xpos[refid]),
            MjObjectType::Body if refid < model.nbody => (refid, data.xipos[refid]),
            MjObjectType::Geom if refid < model.ngeom => {
                (model.geom_body[refid], data.geom_xpos[refid])
            }
            _ => {
                sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                continue;
            }
        };
        let ref_mat = match reftype {
            MjObjectType::Site if refid < model.nsite => data.site_xmat[refid],
            MjObjectType::XBody if refid < model.nbody => data.xmat[refid],
            MjObjectType::Body if refid < model.nbody => data.ximat[refid],
            MjObjectType::Geom if refid < model.ngeom => data.geom_xmat[refid],
            _ => Matrix3::identity(),
        };

        // Object velocity in world frame (flg_local = 0 → local_rot = None)
        let (w_obj, v_obj) = object_velocity(data, body_id, &obj_pos, None);

        // Reference velocity in world frame (flg_local = 0 → local_rot = None)
        let (w_ref, v_ref) = object_velocity(data, ref_body_id, &ref_pos, None);

        // Coriolis correction: v_rel = v_obj - v_ref - w_ref × (p_obj - p_ref)
        let dif = obj_pos - ref_pos;
        let coriolis = w_ref.cross(&dif);
        let v_rel_world = v_obj - v_ref - coriolis;

        // Rotate into reference frame
        let v_rel = ref_mat.transpose() * v_rel_world;
        sensor_write3(&mut data.sensordata, adr, &v_rel);
    } else {
        let (_omega, v) = object_velocity(data, body_id, &obj_pos, None);
        sensor_write3(&mut data.sensordata, adr, &v);
    }
}
```

**FrameAngVel** (lines 153–182 → modified):

**Note:** MuJoCo calls `mj_objectVelocity()` for FrameAngVel (as shown in the
MuJoCo Reference section), which returns a 6-vector `[angular(3); linear(3)]`
and only the angular part `[0..3]` is used. Since angular velocity is
independent of spatial transport point (unlike linear velocity, angular velocity
is the same regardless of the reference point on the rigid body),
`mj_objectVelocity()` for the angular part is equivalent to reading
`cvel[body_id][0..3]` directly. The algorithm below reads `cvel` directly
because CortenForge's existing FrameAngVel code already uses this pattern
(see `velocity.rs:155–170`), and calling `object_velocity()` only to discard
the linear part would be wasteful. Both approaches are numerically identical.

```rust
MjSensorType::FrameAngVel => {
    // Object angular velocity in world frame
    let (body_id_obj, w_obj) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            let body_id = model.site_body[objid];
            (body_id, Vector3::new(
                data.cvel[body_id][0], data.cvel[body_id][1], data.cvel[body_id][2],
            ))
        }
        MjObjectType::XBody | MjObjectType::Body if objid < model.nbody => {
            (objid, Vector3::new(
                data.cvel[objid][0], data.cvel[objid][1], data.cvel[objid][2],
            ))
        }
        MjObjectType::Geom if objid < model.ngeom => {
            let body_id = model.geom_body[objid];
            (body_id, Vector3::new(
                data.cvel[body_id][0], data.cvel[body_id][1], data.cvel[body_id][2],
            ))
        }
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };

    if model.sensor_reftype[sensor_id] != MjObjectType::None {
        let reftype = model.sensor_reftype[sensor_id];
        let refid = model.sensor_refid[sensor_id];

        // Reference angular velocity in world frame (bounds-guarded)
        let ref_body_id = match reftype {
            MjObjectType::Site if refid < model.nsite => model.site_body[refid],
            MjObjectType::XBody | MjObjectType::Body if refid < model.nbody => refid,
            MjObjectType::Geom if refid < model.ngeom => model.geom_body[refid],
            _ => {
                sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
                continue;
            }
        };
        let w_ref = Vector3::new(
            data.cvel[ref_body_id][0],
            data.cvel[ref_body_id][1],
            data.cvel[ref_body_id][2],
        );
        let ref_mat = match reftype {
            MjObjectType::Site if refid < model.nsite => data.site_xmat[refid],
            MjObjectType::XBody if refid < model.nbody => data.xmat[refid],
            MjObjectType::Body if refid < model.nbody => data.ximat[refid],
            MjObjectType::Geom if refid < model.ngeom => data.geom_xmat[refid],
            _ => Matrix3::identity(),
        };

        // w_rel = w_obj - w_ref (NO Coriolis for angular velocity)
        let w_rel_world = w_obj - w_ref;

        // Rotate into reference frame
        let w_rel = ref_mat.transpose() * w_rel_world;
        sensor_write3(&mut data.sensordata, adr, &w_rel);
    } else {
        sensor_write3(&mut data.sensordata, adr, &w_obj);
    }
}
```

### S4. Acceleration-stage: no change (document only)

**File:** `sim/L0/core/src/sensor/acceleration.rs`
**MuJoCo equivalent:** `mj_sensorAcc()` in `engine_sensor.c` — FrameLinAcc, FrameAngAcc
**Design decision:** No code change. MuJoCo ignores `sensor_refid` for
acceleration-stage frame sensors. CortenForge already outputs in world frame.
Setting `reftype`/`refname` on FrameLinAcc/FrameAngAcc in MJCF has no effect
on output — the builder stores the reftype/refid, but evaluation never reads
it for these sensor types. This matches MuJoCo.

---

## Acceptance Criteria

### AC1: Builder resolves reftype/refname → model arrays *(runtime test)*
**Given:** MJCF model with `<framepos site="s1" reftype="site" refname="s_ref"/>`
**After:** `load_model()`
**Assert:** `model.sensor_reftype[0] == MjObjectType::Site`, `model.sensor_refid[0] == site_id_of_s_ref`
**Field:** `model.sensor_reftype`, `model.sensor_refid`

### AC2: Builder defaults — no reftype/refname → None/0 *(runtime test)*
**Given:** MJCF model with `<framepos site="s1"/>` (no reftype/refname)
**After:** `load_model()`
**Assert:** `model.sensor_reftype[0] == MjObjectType::None`, `model.sensor_refid[0] == 0`
**Field:** `model.sensor_reftype`, `model.sensor_refid`

### AC3: FramePos relative to rotated reference body *(runtime test — MuJoCo-verified)*
**Given:** Body A at `[1, 0, 0]` rotated 90° about Z. Site B at `[0, 2, 0]`. `<framepos site="sB" reftype="xbody" refname="A"/>`
**After:** `data.forward()`
**Assert:** `sensordata = [2.0, 1.0, 0.0] ± 1e-10`
**Derivation:** MuJoCo 3.5.0 produces `[2.0, 1.0, 0.0]` for this configuration; we assert `[2.0, 1.0, 0.0] ± 1e-10`. Verified by tracing `mj_sensorPos()`, `mjSENS_FRAMEPOS` case: `get_xpos_xmat()` resolves ref position `[1,0,0]` and rotation `Rz(90°)`. Transform: `Rz(-90°) * ([0,2,0] - [1,0,0]) = Rz(-90°) * [-1,2,0] = [2,1,0]`.
**Field:** `data.sensordata[adr..adr+3]`

### AC4: FrameQuat relative to rotated reference site *(runtime test — analytically derived)*
**Given:** Reference site `s_ref` on body A rotated 90° about Z: `q_A = [0.7071, 0, 0, 0.7071]`. Object B at identity: `q_B = [1, 0, 0, 0]`. `<framequat objtype="xbody" objname="B" reftype="site" refname="s_ref"/>` (exercises `mat2Quat(site_xmat)` path in `get_ref_quat`)
**After:** `data.forward()`
**Assert:** `sensordata = [0.7071, 0, 0, -0.7071] ± 1e-4` (`q_ref^{-1} * q_obj` where `q_ref` is derived from `site_xmat` via `mat2Quat`)
**Field:** `data.sensordata[adr..adr+4]`

### AC5: FrameXAxis in reference frame *(runtime test — analytically derived)*
**Given:** Object at identity. Reference rotated 90° about Z. `<framexaxis objtype="xbody" objname="B" reftype="xbody" refname="A"/>`
**After:** `data.forward()`
**Assert:** Object X-axis `[1,0,0]` in world → `R_ref^T * [1,0,0] = [0, -1, 0] ± 1e-10` (Rz(-90°) maps x→y direction, so x-axis in ref frame = `[0,-1,0]`)
**Field:** `data.sensordata[adr..adr+3]`

### AC6: FrameLinVel with Coriolis correction *(runtime test — MuJoCo-verified)*
**Given:** Reference body at origin, spinning at `ω = [0,0,1]` rad/s about Z (identity orientation at t=0). Object at `[1,0,0]`, both have zero linear velocity in world frame.
**After:** `data.forward()` with `qvel` set for angular velocity
**Assert:** `sensordata = [0.0, -1.0, 0.0] ± 1e-6` (Coriolis: `-w × r = -[0,1,0]`, rotated by `I`)
**Derivation:** MuJoCo 3.5.0 produces `[0.0, -1.0, 0.0]` for this configuration; we assert `[0.0, -1.0, 0.0] ± 1e-6`. Verified by tracing `mj_sensorVel()`, `mjSENS_FRAMELINVEL` case: `mj_objectVelocity()` with `flg_local=0` returns world-frame velocities. `get_xpos_xmat()` resolves ref position `[0,0,0]` and rotation `I`. Coriolis: `w_ref × (p_obj - p_ref) = [0,0,1] × [1,0,0] = [0,1,0]`. `v_rel = 0 - 0 - [0,1,0] = [0,-1,0]`. Final: `R_ref^T * [0,-1,0] = I * [0,-1,0] = [0,-1,0]`.
**Field:** `data.sensordata[adr..adr+3]`

### AC7: FrameAngVel in reference frame *(runtime test — analytically derived)*
**Given:** Object spinning at `ω_obj = [0,0,5]`, reference spinning at `ω_ref = [0,0,2]`. Reference at identity orientation.
**After:** `data.forward()`
**Assert:** `sensordata = [0.0, 0.0, 3.0] ± 1e-10` (`R_ref^T * (w_obj - w_ref) = I * [0,0,3]`)
**Field:** `data.sensordata[adr..adr+3]`

### AC8: No reftype → world-frame output unchanged (regression) *(runtime test)*
**Given:** MJCF model with `<framepos site="s1"/>` (no reftype/refname)
**After:** `data.forward()`
**Assert:** `sensordata` matches `data.site_xpos[site_id]` exactly (world frame)
**Field:** `data.sensordata[adr..adr+3]`

### AC9: FrameLinAcc/FrameAngAcc unaffected by reftype *(runtime test)*
**Given:** MJCF with `<framelinacc site="s1" reftype="xbody" refname="b_ref"/>` and identical `<framelinacc site="s1"/>` (no reftype)
**After:** `data.forward()` with non-zero velocity
**Assert:** Both sensors produce identical sensordata (reftype ignored for acc sensors)
**Field:** `data.sensordata`

### AC10: `sensor_dim`/`sensor_adr` invariance *(runtime test)*
**Given:** MJCF with two FramePos sensors: one with reftype/refname, one without
**After:** `load_model()`
**Assert:** Both have `sensor_dim == 3`, `sensor_adr` offsets correct (reftype does NOT change dimensionality)
**Field:** `model.sensor_dim`, `model.sensor_adr`

### AC11: Invalid reftype string → `ModelConversionError` *(runtime test)*
**Given:** MJCF with `<framepos site="s1" reftype="invalid" refname="s1"/>`
**After:** `load_model()`
**Assert:** Returns `Err(ModelConversionError)` with message containing "invalid reftype"
**Field:** Error variant

### AC12: Unknown refname → `ModelConversionError` *(runtime test)*
**Given:** MJCF with `<framepos site="s1" reftype="body" refname="nonexistent"/>`
**After:** `load_model()`
**Assert:** Returns `Err(ModelConversionError)` with message containing "unknown body"
**Field:** Error variant

### AC13: reftype without refname → no transform *(runtime test)*
**Given:** MJCF with `<framepos site="s1" reftype="body"/>` (reftype but no refname)
**After:** `load_model()` succeeds, `data.forward()`
**Assert:** `sensor_reftype[0] == MjObjectType::None` (ignored), output = world frame
**Field:** `model.sensor_reftype`, `data.sensordata`

### AC14: Builder uses correct MjObjectType for each reftype string *(runtime test)*
**Given:** Model with sensors using reftype `"site"`, `"body"`, `"xbody"`, `"geom"`
**After:** `load_model()`
**Assert:** `sensor_reftype` = `[Site, Body, XBody, Geom]` respectively
**Field:** `model.sensor_reftype`

### AC15: FramePos body vs xbody numerical distinction *(runtime test — analytically derived)*
**Given:** Body with offset geom (COM ≠ joint origin). Two sensors: `<framepos site="sObj" reftype="body" refname="bRef"/>` and `<framepos site="sObj" reftype="xbody" refname="bRef"/>`.
**After:** `data.forward()`
**Assert:** The two sensors produce different sensordata (because `reftype="body"` uses `xipos`/`ximat` while `reftype="xbody"` uses `xpos`/`xmat`, and `xipos ≠ xpos` for a body with offset COM)
**Field:** `data.sensordata`

### AC16: No code change in acceleration.rs *(code review)*
Verify that `acceleration.rs` FrameLinAcc/FrameAngAcc match arms are
unchanged by this spec. The `sensor_reftype`/`sensor_refid` arrays are not
read in this file.

### AC17: FrameQuat with reftype="body" exercises `xquat * body_iquat` path *(runtime test — analytically derived)*
**Given:** Body `bRef` with offset geom (`pos="0.3 0 0"`) so `body_iquat` ≠ identity, rotated 90° about Z via hinge. Object body B at identity. `<framequat objtype="xbody" objname="B" reftype="body" refname="bRef"/>`
**After:** `data.forward()`
**Assert:** `sensordata` differs from the result using `reftype="xbody"` (which returns `xquat` directly). The `reftype="body"` path computes `xquat[refid] * body_iquat[refid]`, giving the COM-frame orientation, which differs from the joint-frame orientation when `body_iquat ≠ identity`.
**Field:** `data.sensordata[adr..adr+4]`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (builder resolves reftype) | T1 | Direct |
| AC2 (builder defaults) | T2 | Direct |
| AC3 (FramePos relative) | T3 | Direct |
| AC4 (FrameQuat relative) | T4 | Direct |
| AC5 (FrameXAxis relative) | T5 | Direct |
| AC6 (FrameLinVel Coriolis) | T6 | Direct |
| AC7 (FrameAngVel relative) | T7 | Direct |
| AC8 (no reftype regression) | T8 | Regression |
| AC9 (acc unaffected) | T9 | Direct |
| AC10 (dim/adr invariance) | T10 | Direct |
| AC11 (invalid reftype error) | T11 | Direct |
| AC12 (unknown refname error) | T12 | Direct |
| AC13 (reftype without refname) | T13 | Direct |
| AC14 (reftype→MjObjectType mapping) | T14 | Direct |
| AC15 (body vs xbody numerical) | T18 | Direct |
| AC16 (no acc change) | — | Code review (manual) |
| AC17 (FrameQuat body arm) | T19 | Direct |

---

## Test Plan

All tests in `sim/L0/tests/integration/sensor_phase6.rs` (appended after
existing T1–T20 from Spec A). Test naming continues from T21.

> **Test numbering:** T1–T19 below are Spec B's local labels for the test plan.
> In the test file, these map to functions `t21_*` through `t39_*` (continuing
> from Spec A's `t01_*`–`t20_*`). References to "existing tests T1–T20" or
> "Spec A tests" always mean the file-level `t01_*`–`t20_*` functions.

### T1: Builder resolves reftype="site" + refname="s_ref" → AC1
Model: body with two sites `s1` and `s_ref`. Sensor `<framepos site="s1" reftype="site" refname="s_ref"/>`.
Assert: `model.sensor_reftype[0] == MjObjectType::Site`, `model.sensor_refid[0] == site_id_of_s_ref`.

### T2: Builder default — no reftype/refname → AC2
Model: body with site. Sensor `<framepos site="s1"/>` (no reftype/refname).
Assert: `model.sensor_reftype[0] == MjObjectType::None`, `model.sensor_refid[0] == 0`.

### T3: FramePos relative to rotated body → AC3
Model: body A at `[1,0,0]` with hinge joint preset to 90° (π/2 rad) about Z.
Body B at `[0,2,0]` with site `sB`. Sensor `<framepos site="sB" reftype="xbody" refname="A"/>`.
After forward: `sensordata = [2.0, 1.0, 0.0] ± 1e-6`.
Expected value: analytically derived from MuJoCo 3.5.0 C source (`mj_sensorPos`,
`mjSENS_FRAMEPOS` case with `get_xpos_xmat` dispatch).
Derivation: `Rz(-90°) * ([0,2,0] - [1,0,0]) = Rz(-90°) * [-1,2,0] = [2,1,0]`.

### T4: FrameQuat relative to rotated reference site → AC4
Model: body A with hinge preset to 90° about Z, with site `s_ref`. Body B at identity.
Sensor `<framequat objtype="xbody" objname="B" reftype="site" refname="s_ref"/>`.
After forward: `sensordata = [0.7071, 0, 0, -0.7071] ± 1e-4`.
Analytically: `q_ref = mat2Quat(site_xmat[s_ref])` where `site_xmat` = Rz(90°) → `q_ref = [cos45,0,0,sin45]`.
`q_ref^{-1} * q_B = conj([cos45,0,0,sin45]) * [1,0,0,0] = [cos45,0,0,-sin45]`.
Exercises the `mat2Quat(site_xmat)` path in `get_ref_quat()` (distinct from the `xquat` direct-copy path).

### T5: FrameXAxis in reference frame → AC5
Model: body A with hinge at 90° about Z. Body B at identity.
Sensor `<framexaxis objtype="xbody" objname="B" reftype="xbody" refname="A"/>`.
After forward: object X-axis is `[1,0,0]` in world. Reference rotation is Rz(90°).
`R_ref^T * [1,0,0] = Rz(-90°) * [1,0,0] = [0,-1,0]`.
Assert: `sensordata = [0.0, -1.0, 0.0] ± 1e-10`.

### T6: FrameLinVel with Coriolis correction → AC6
Model: body A (reference) at origin with hinge about Z, `qvel = 1.0` (1 rad/s).
Body B (object) at `[1,0,0]` with site, both at zero linear velocity.
Sensor `<framelinvel site="sB" reftype="xbody" refname="A"/>`.
After forward: Coriolis `w_ref × r = [0,0,1] × [1,0,0] = [0,1,0]`.
`v_rel = 0 - 0 - [0,1,0] = [0,-1,0]`. In ref frame (identity rot): `[0,-1,0]`.
Assert: `sensordata[1] ≈ -1.0 ± 1e-3`.
Expected value: MuJoCo 3.5.0 C source verified (`mj_sensorVel`,
`mjSENS_FRAMELINVEL` case with `mj_objectVelocity` and Coriolis correction).
Derivation: `mj_objectVelocity()` with `flg_local=0` → world-frame velocities.
`w_ref × (p_obj - p_ref) = [0,0,1] × [1,0,0] = [0,1,0]`.
`v_rel = 0 - 0 - [0,1,0] = [0,-1,0]`. `R_ref^T * [0,-1,0] = I * [0,-1,0] = [0,-1,0]`.
Tolerance: 1e-3 (accounts for floating-point in spatial transport).

### T7: FrameAngVel in reference frame → AC7
Model: body A (reference) with hinge about Z, `qvel_A = 2.0`.
Body B (object) with hinge about Z, `qvel_B = 5.0`.
Sensor `<frameangvel objtype="xbody" objname="B" reftype="xbody" refname="A"/>`.
After forward: `w_rel = [0,0,5] - [0,0,2] = [0,0,3]`. Ref at identity: `[0,0,3]`.
Assert: `sensordata[2] ≈ 3.0 ± 1e-10`.
Analytically derived.

### T8: No reftype regression guard → AC8
Model: body with site. Sensor `<framepos site="s1"/>` (no reftype).
After forward: `sensordata` matches `data.site_xpos[site_id]` exactly.
Verifies existing Spec A behavior unchanged.

### T9: FrameLinAcc/FrameAngAcc unaffected by reftype → AC9
Model: two bodies, hinge joints. Two FrameLinAcc sensors: one with reftype, one without.
Two FrameAngAcc sensors: same setup. Set `qvel != 0` for non-trivial output.
After forward: both pairs produce identical sensordata values (reftype ignored).

### T10: sensor_dim/sensor_adr invariance → AC10
Model: `<framepos site="s1" reftype="xbody" refname="b1"/>` followed by
`<framepos site="s2"/>` (no reftype).
Assert: both have `sensor_dim == 3`. `sensor_adr[0] == 0`, `sensor_adr[1] == 3`.

### T11: Invalid reftype → error → AC11
Model: `<framepos site="s1" reftype="invalid" refname="s1"/>`.
Assert: `load_model()` returns `Err` with message containing `"invalid reftype"`.

### T12: Unknown refname → error → AC12
Model: `<framepos site="s1" reftype="body" refname="nonexistent"/>`.
Assert: `load_model()` returns `Err` with message containing `"unknown body"`.

### T13: reftype without refname → no transform → AC13
Model: `<framepos site="s1" reftype="body"/>`.
Assert: `load_model()` succeeds, `sensor_reftype[0] == MjObjectType::None`.
After forward: output equals world-frame site position.

### T14: All reftype strings resolve correctly → AC14
Model with 4 sensors, each using a different reftype: `"site"`, `"body"`, `"xbody"`, `"geom"`.
Assert: `model.sensor_reftype` = `[Site, Body, XBody, Geom]`.

### T18: FramePos body vs xbody numerical distinction → AC15
Model: body `bRef` with offset geom (`pos="0.3 0 0"`) so COM differs from joint
origin. Object site `sObj` on a second body at `[0, 1, 0]`. Two sensors:
`<framepos site="sObj" reftype="body" refname="bRef"/>` and
`<framepos site="sObj" reftype="xbody" refname="bRef"/>`.
After forward: the two sensordata values differ because `reftype="body"` uses
`xipos`/`ximat` (COM frame) while `reftype="xbody"` uses `xpos`/`xmat` (joint
frame), and `xipos.x ≠ xpos.x` when the body has offset COM.
Assert: `(sensordata_body - sensordata_xbody).norm() > 0.01`.

### T19: FrameQuat body vs xbody via `get_ref_quat` → AC17
Model: body `bRef` with offset geom (`pos="0.3 0 0"`) so `body_iquat ≠ identity`,
with hinge joint preset to 90° about Z. Object body B at identity. Two sensors:
`<framequat objtype="xbody" objname="B" reftype="body" refname="bRef"/>` and
`<framequat objtype="xbody" objname="B" reftype="xbody" refname="bRef"/>`.
After forward: `reftype="body"` computes `get_ref_quat` via `xquat[refid] * body_iquat[refid]`
(COM-frame orientation), while `reftype="xbody"` returns `xquat[refid]` directly.
Since `body_iquat ≠ identity`, the two produce different reference quaternions
and therefore different relative quaternion outputs.
Assert: `(sensordata_body - sensordata_xbody).norm() > 0.01`.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| No reftype/refname (world frame) | Regression — must not change existing behavior | T2, T8 | AC2, AC8 |
| reftype=xbody with world body (`refname="world"`) | Identity transform — output == world frame | T15 | supplementary |
| reftype=body (COM frame `xipos`/`ximat`) vs reftype=xbody (joint frame `xpos`/`xmat`) | Measurable difference on body with offset COM | T3, T18, T19 | AC3, AC15, AC17 |
| reftype=geom | Uses `geom_xpos`/`geom_xmat` | T14 | AC14 |
| reftype=site | Uses `site_xpos`/`site_xmat` | T4, T14 | AC4, AC14 |
| FrameLinVel Coriolis term | Object and reference at different positions on rotating body | T6 | AC6 |
| FrameLinAcc/FrameAngAcc unaffected by refid | Behavioral no-op for acc sensors | T9 | AC9 |
| Cross-body reference | Reference and object on different bodies | T3, T6, T7 | AC3, AC6, AC7 |
| Reference and object on same body | Relative offset within body | T16 | supplementary |
| Sleeping primary body with awake reference | Sensor IS skipped (sleep uses primary body, not reference) | T17 | supplementary |
| sensor_dim/sensor_adr invariance | reftype must not change output dimensionality | T10 | AC10 |
| reftype without refname | MuJoCo silently ignores → no transform | T13 | AC13 |
| Invalid reftype string | Error handling | T11 | AC11 |
| Unknown refname | Error handling | T12 | AC12 |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T15 (world body as reference) | reftype=xbody, refname=world → identity transform | Boundary case: output should equal world-frame. Validates guard-free code path produces correct identity result |
| T16 (same-body reference) | Object and reference are sites on the same body | Tests that relative position/orientation within a single rigid body is correctly computed |
| T17 (sleeping body + awake reference) | Sleep filtering uses primary object's body, not reference | Validates that sensor is skipped when primary is sleeping, regardless of reference state |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Frame sensors with `reftype`/`refname` MJCF attributes | Attributes silently ignored; output in world frame | Output in reference frame (relative measurement) | Toward MuJoCo conformance | Any MJCF model using `reftype`/`refname` on frame sensors | None — transparent change; models without reftype are unaffected |
| Builder reftype/refname resolution | Hardcoded `MjObjectType::None`/`0` | Resolved from MJCF attributes | Toward MuJoCo conformance | Builder internals only | None |
| Invalid reftype string | Silently ignored (pushed None/0) | `ModelConversionError` | Toward MuJoCo conformance | Models with typos in reftype | Fix the typo |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/mjcf/src/builder/sensor.rs` | Replace hardcoded None/0 with `resolve_reference_object()` call; add new function | ~80 new, ~2 modified |
| `sim/L0/core/src/sensor/position.rs` | Add `get_ref_pos_mat()`, `get_ref_quat()` helpers; add ref-frame transform to FramePos, FrameQuat, FrameXAxis/YAxis/ZAxis arms | ~60 new, ~15 modified |
| `sim/L0/core/src/sensor/velocity.rs` | Add `get_ref_pos_mat()` helper; add ref-frame transform to FrameLinVel, FrameAngVel arms | ~80 new, ~15 modified |
| `sim/L0/core/src/sensor/acceleration.rs` | **No change** — MuJoCo ignores refid for acc sensors | 0 |
| `sim/L0/tests/integration/sensor_phase6.rs` | New tests T21–T39 | ~500 new |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `t01_parser_objtype_attribute` | `sensor_phase6.rs:16` | Pass (unchanged) | Tests parser, not builder/eval |
| `t02_parser_reftype_refname_separated` | `sensor_phase6.rs:47` | Pass (unchanged) | Tests parser, not builder/eval |
| `t03_parser_geom_attribute` | `sensor_phase6.rs:87` | Pass (unchanged) | Tests parser |
| `t04_builder_explicit_objtype_geom` | `sensor_phase6.rs:111` | Pass (unchanged) | No reftype in model |
| `t05_builder_body_xbody_distinction` | `sensor_phase6.rs:135` | Pass (unchanged) | No reftype in model |
| `t06_body_vs_xbody_position` | `sensor_phase6.rs:161` | Pass (unchanged) | No reftype → guard skips transform |
| `t07_body_framequat_mulquat` | `sensor_phase6.rs:216` | Pass (unchanged) | No reftype |
| `t08_default_inference_body_is_xbody` | `sensor_phase6.rs:280` | Pass (unchanged) | No reftype |
| `t09_touch_multi_geom_aggregation` | `sensor_phase6.rs:305` | Pass (unchanged) | Touch sensor, not frame sensor |
| `t10_touch_wrong_body_filtered` | `sensor_phase6.rs:381` | Pass (unchanged) | Touch sensor |
| `t11_touch_body_zero_geoms` | `sensor_phase6.rs:475` | Pass (unchanged) | Touch sensor |
| `t12_geom_framelinacc_matches_site` | `sensor_phase6.rs:506` | Pass (unchanged) | No reftype; acc stage unmodified |
| `t13_geom_frameangacc_static` | `sensor_phase6.rs:561` | Pass (unchanged) | No reftype; acc stage unmodified |
| `t14_geom_framelinacc_centripetal` | `sensor_phase6.rs:594` | Pass (unchanged) | No reftype |
| `t16_builder_regression_touch_as_site` | `sensor_phase6.rs:629` | Pass (unchanged) | Touch sensor |
| `t17_objtype_ignored_for_touch` | `sensor_phase6.rs:657` | Pass (unchanged) | Touch sensor |
| `t18_body_framepos_zero_mass` | `sensor_phase6.rs:683` | Pass (unchanged) | No reftype |
| `t19_geom_framelinvel_matches_site` | `sensor_phase6.rs:739` | Pass (unchanged) | No reftype |
| `t20_geom_frameangvel_matches_site` | `sensor_phase6.rs:789` | Pass (unchanged) | No reftype |
| Phase 5 test suite (2,238+ tests) | Various | Pass (unchanged) | No sensor evaluation paths modified for actuators |

> **Regression guarantee:** All existing tests (T1–T20) use no `reftype`/`refname`
> attributes → `sensor_reftype` remains `MjObjectType::None` → guard condition
> skips transform → existing behavior unchanged.

**Downstream consumers:** `sensor_reftype`/`sensor_refid` are read only in
evaluation files (`position.rs`, `velocity.rs`). The L1 crate (`sim-bevy`)
does not read these arrays — it consumes `sensordata` values, which are
transparently updated by the evaluation code. No downstream API change.

---

## Execution Order

1. **S1** (builder reference resolution) → verify: `cargo test -p sim-mjcf` passes, new builder tests pass (T1, T2, T10, T11, T12, T13, T14). S1 must land first because S2 and S3 read `sensor_reftype`/`sensor_refid` populated by the builder.

2. **S2** (position-stage transforms) → verify: `cargo test -p sim-core -p sim-mjcf -p sim-sensor -p sim-conformance-tests` passes, T3, T4, T5, T8 pass. Position stage is simpler (no Coriolis) and validates the `get_ref_pos_mat`/`get_ref_quat` infrastructure that S3 reuses.

3. **S3** (velocity-stage transforms) → verify: full domain test suite passes, T6, T7 pass.

4. **S4** (acceleration no-change) → verify by code review (AC16). Run full domain tests to confirm no regression.

**Cross-spec interactions:**
- **Spec C** (new sensor types) — adds non-frame sensors (e.g., magnetometer, rangefinder). None are frame sensors, so `reftype`/`refname` is irrelevant to Spec C's scope. No interaction.
- **Spec D** (history/interpolation attributes) — operates on different model arrays (`sensor_cutoff`, `sensor_noise`). Orthogonal to reference-frame transforms. No interaction.

After each section, run:
```
cargo test -p sim-core -p sim-mjcf -p sim-sensor -p sim-conformance-tests
```

---

## Out of Scope

- **DT-117** (`Camera` variant in `MjObjectType`) — `reftype="camera"` logs a warning and falls through to `MjObjectType::None` (no transform). Tracked as DT-117. *Conformance impact: minor — camera reference frames are uncommon.*

- **DT-118** (Geom-attached velocity dispatch for FrameLinVel/FrameAngVel — already handled in Spec A review). *Conformance impact: none — landed.*

- **FrameLinAcc/FrameAngAcc relative-frame** — MuJoCo intentionally does not support this. Not a conformance gap.

- **Sensor noise application** — parsed and stored but not applied at runtime (intentional). Not a conformance gap.

- **Runtime sensor interpolation** (DT-107/DT-108) — deferred to Spec D (attributes) and post-v1.0 (runtime). *Conformance impact: none for v1.0.*
