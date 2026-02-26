# DT-103 — Extract Spatial Transport Helpers

**Status:** Draft (Revision 3 — gaps D1–D8 closed)
**Scope:** Extract `transport_motion`, `transport_force`, `object_velocity`,
`object_acceleration`, and `object_force` helpers into `dynamics/spatial.rs`.
Rewrite 6 sensor arms and 1 existing helper to call them. Pure refactor — no
behavior change, no new `Data` fields.
**Origin:** Phase 4A.6 (`SENSOR_CACC_CFRC_REFACTOR.md`, DT-103 deferred item).
**Prerequisite:** Phase 4A.6 sensor refactor — already landed (commit 16cfcb3).
The `flg_rnepost` lazy gate (commit 8e8f5f7), cvel fixes (commit 444046e), and
§56 subtree fields (commit 503ac6d) are all prerequisites that have already
shipped. This spec builds on the code as it exists today.
**MuJoCo ref:** `engine_core_util.c` (`mj_objectAcceleration`, `mj_objectVelocity`),
`engine_util_spatial.c` (`mju_transformSpatial`), `engine_sensor.c` (Force/Torque).

---

## Problem Statement

The spatial transport pattern — "read 6D vector at body origin (`xpos[body_id]`),
shift to target point, optionally add Coriolis correction, optionally rotate to
local frame" — is inlined across **6 sensor arms** in two files and **1 existing
helper** in a third file (7 consumers total). The deficiencies:

1. **Convention hazard.** The `xpos[body_id]` reference point for `cacc`/`cvel`/
   `cfrc_int` (our convention, vs MuJoCo's `subtree_com[root]`) is written as a
   raw subtraction in 6 separate locations (5 sensor arms + 1 helper). Getting
   this wrong silently produces incorrect physics. A helper encodes this
   convention once.

2. **Duplication.** The acceleration transport + Coriolis pattern is literally
   copy-pasted between Accelerometer and FrameLinAcc (lines 107–123 vs 267–280
   — identical modulo variable names). The velocity transport pattern is
   copy-pasted between Velocimeter and FrameLinVel (lines 119–131 vs 157–168).

3. **Inconsistency risk.** `object_velocity_local` in `dynamics/spatial.rs` is a
   lower-level helper for cvel transport with rotation. It uses the same
   `xpos[body_id]` convention but lives in a different module from the sensor
   transport code. There is no acceleration counterpart.

4. **Future consumers.** DT-102 (geom-attached frame sensors) will need the same
   transport. Any future sensor reading `cacc`, `cvel`, or `cfrc_int` at a point
   other than the body origin will need it.

---

## MuJoCo Reference

### `mj_objectVelocity` (`engine_core_util.c`)

```c
void mj_objectVelocity(const mjModel* m, const mjData* d,
                        int objtype, int objid, mjtNum* res, int flg_local) {
    // 1. Resolve body_id and target position from (objtype, objid)
    // 2. Read cvel[body_id] at subtree_com[root]
    // 3. mju_transformSpatial(res, cvel, 0, target_pos, subtree_com[root], NULL)
    //    -> motion transport: angular unchanged, v_lin += omega x r
    // 4. If flg_local: rotate by object rotation matrix
}
```

### `mj_objectAcceleration` (`engine_core_util.c`)

```c
void mj_objectAcceleration(const mjModel* m, const mjData* d,
                            int objtype, int objid, mjtNum* res, int flg_local) {
    // 1. Resolve body_id and target position from (objtype, objid)
    // 2. Read cacc[body_id] at subtree_com[root]
    // 3. mju_transformSpatial(res, cacc, 0, target_pos, subtree_com[root], NULL)
    //    -> motion transport: angular unchanged, a_lin += alpha x r
    // 4. Read cvel[body_id], apply same transport to get v_at_point
    // 5. Coriolis: a_lin += omega x v_lin_at_point
    // 6. If flg_local: rotate by object rotation matrix
}
```

### `mju_transformSpatial` (`engine_util_spatial.c`)

```c
void mju_transformSpatial(mjtNum* res, const mjtNum* vec, int flg_force,
                          const mjtNum* newpos, const mjtNum* oldpos,
                          const mjtNum* rotnew2old) {
    // r = newpos - oldpos
    // if flg_force == 0 (motion):
    //   angular unchanged
    //   linear += angular x r
    // if flg_force == 1 (force):
    //   force unchanged
    //   torque -= r x force
    // if rotnew2old != NULL: rotate both components
}
```

### Force/Torque sensors (`engine_sensor.c`)

MuJoCo has no separate `mj_objectForce` function. The Force/Torque sensor arms
call `mju_transformSpatial` with `flg_force=1` directly:

```c
case mjSENS_FORCE:
  bodyid = m->site_bodyid[objid];
  rootid = m->body_rootid[bodyid];
  mju_transformSpatial(tmp, d->cfrc_int + 6*bodyid, 1,
                       d->site_xpos + 3*objid,
                       d->subtree_com + 3*rootid,
                       d->site_xmat + 9*objid);
  mju_copy3(sensordata, tmp+3);  // force part (indices 3-5)
```

### Convention Difference

| Field | Our Convention | MuJoCo Convention |
|-------|----------------|-------------------|
| `cacc[b]` | at `xpos[b]` (body origin) | at `subtree_com[body_rootid[b]]` |
| `cvel[b]` | at `xpos[b]` (body origin) | at `subtree_com[body_rootid[b]]` |
| `cfrc_int[b]` | at `xpos[b]` (body origin) | at `subtree_com[body_rootid[b]]` |

**Porting rule:** When porting MuJoCo code that reads `cacc`, `cfrc_int`, or
`cvel`, substitute `xpos[body_id]` wherever MuJoCo uses `subtree_com + 3*rootid`.
The transport formula and everything else stays the same.

---

## Design Decisions

### D1: Low-Level Transport Helpers (not MuJoCo-style API)

MuJoCo bundles objtype resolution with transport in `mj_objectAcceleration(m, d,
objtype, objid, res, flg_local)`. Our sensor arms already resolve `(body_id,
target_pos, rotation_mat)` via their own `match sensor_objtype` dispatch. A
MuJoCo-style API would force every caller to dispatch through objtype even when
`body_id` and `target_pos` are already known. Our helpers do only the physics:
transport + optional Coriolis + optional rotation. More composable, avoids
redundant dispatch.

### D2: Include Velocity and Force Transport (not just acceleration)

`object_velocity_local` already exists but only handles `flg_local=1` (forced
rotation). Need a version without forced rotation for FrameLinVel. Velocity
transport duplicated between Velocimeter and FrameLinVel. Force/Torque transport
duplicated inline. Unifying under consistent API is worth marginal effort.

### D3: All Helpers in `dynamics/spatial.rs`

Where `object_velocity_local` already lives. Canonical location for spatial
algebra utilities.

### D4: Return `(Vector3<f64>, Vector3<f64>)` Tuples

Every sensor arm needs either the angular OR the linear component, never both as
a packed 6-vector. Tuples avoid element extraction at every call site. Makes API
self-documenting: `(alpha, a_linear)` vs `result[0..3]`. The preserved
`object_velocity_local` wrapper retains `[f64; 6]` for backward compatibility.

### D5: Helpers Take `&Data`, not `(&Model, &Data)`

Helpers only need `data.cacc`, `data.cvel`, `data.cfrc_int`, and `data.xpos` —
all on `Data`. The `Model` parameter in `object_velocity_local` is already
unused (`_model`). New helpers drop it. Preserved wrapper retains it for backward
compatibility.

### D6: FrameAngAcc, FrameAngVel, and Gyro Are NOT Refactored

Angular components (velocity and acceleration) are reference-point-independent
on rigid bodies. No spatial transport is needed. Forcing them through the helper
would compute unnecessary terms. Intentional minimality.

---

## New Functions

All added to `sim/L0/core/src/dynamics/spatial.rs`.

### 1. `transport_motion` — Spatial Motion Transport (shared kernel)

```rust
/// Shift a spatial motion vector (velocity or acceleration) from `body_origin`
/// to `target_pos`.
///
/// Motion transport: angular unchanged, linear += angular × r.
/// Equivalent to `mju_transformSpatial` with `flg_force=0` and no rotation.
#[inline]
pub(crate) fn transport_motion(
    angular: &Vector3<f64>,
    linear: &Vector3<f64>,
    target_pos: &Vector3<f64>,
    body_origin: &Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let r = target_pos - body_origin;
    (*angular, linear + angular.cross(&r))
}
```

### 2. `transport_force` — Spatial Force Transport (shared kernel)

```rust
/// Shift a spatial force vector (wrench) from `body_origin` to `target_pos`.
///
/// Force transport: force unchanged, torque -= r × force.
/// Equivalent to `mju_transformSpatial` with `flg_force=1` and no rotation.
#[inline]
pub(crate) fn transport_force(
    torque: &Vector3<f64>,
    force: &Vector3<f64>,
    target_pos: &Vector3<f64>,
    body_origin: &Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let r = target_pos - body_origin;
    (torque - r.cross(force), *force)
}
```

### 3. `object_velocity` — Velocity at a Point

```rust
/// Compute 6D velocity at `target_pos` on body `body_id`, optionally rotated
/// into a local frame.
///
/// Equivalent to `mj_objectVelocity(m, d, ..., flg_local)`:
/// - `local_rot = None` → world frame (flg_local=0)
/// - `local_rot = Some(&mat)` → local frame (flg_local=1)
///
/// Convention: reads `cvel[body_id]` at `xpos[body_id]` (our convention).
pub(crate) fn object_velocity(
    data: &Data,
    body_id: usize,
    target_pos: &Vector3<f64>,
    local_rot: Option<&Matrix3<f64>>,
) -> (Vector3<f64>, Vector3<f64>) {
    let cvel = data.cvel[body_id];
    let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);
    let v_lin = Vector3::new(cvel[3], cvel[4], cvel[5]);

    let (omega_at_point, v_at_point) =
        transport_motion(&omega, &v_lin, target_pos, &data.xpos[body_id]);

    match local_rot {
        Some(rot) => (rot.transpose() * omega_at_point, rot.transpose() * v_at_point),
        None => (omega_at_point, v_at_point),
    }
}
```

### 4. `object_acceleration` — Acceleration at a Point with Coriolis

```rust
/// Compute 6D acceleration at `target_pos` on body `body_id` with Coriolis
/// correction, optionally rotated into a local frame.
///
/// Equivalent to `mj_objectAcceleration(m, d, ..., flg_local)`:
/// 1. Motion transport of `cacc` from body origin to target
/// 2. Motion transport of `cvel` from body origin to target (world frame)
/// 3. Coriolis: a_lin += omega × v_lin
/// 4. Optional rotation
///
/// Convention: reads `cacc[body_id]` and `cvel[body_id]` at `xpos[body_id]`.
pub(crate) fn object_acceleration(
    data: &Data,
    body_id: usize,
    target_pos: &Vector3<f64>,
    local_rot: Option<&Matrix3<f64>>,
) -> (Vector3<f64>, Vector3<f64>) {
    let cacc = data.cacc[body_id];
    let alpha = Vector3::new(cacc[0], cacc[1], cacc[2]);
    let a_lin = Vector3::new(cacc[3], cacc[4], cacc[5]);

    let (alpha_at_point, a_at_point) =
        transport_motion(&alpha, &a_lin, target_pos, &data.xpos[body_id]);

    // Velocity at target point (world frame) for Coriolis
    let (omega_at_point, v_at_point) =
        object_velocity(data, body_id, target_pos, None);

    // Coriolis correction: a += omega × v
    let a_corrected = a_at_point + omega_at_point.cross(&v_at_point);

    match local_rot {
        Some(rot) => (rot.transpose() * alpha_at_point, rot.transpose() * a_corrected),
        None => (alpha_at_point, a_corrected),
    }
}
```

### 5. `object_force` — Wrench at a Point

```rust
/// Compute constraint wrench at `target_pos` on body `body_id`, optionally
/// rotated into a local frame.
///
/// Force transport: force unchanged, torque shifted by lever arm.
/// No Coriolis — wrenches are not velocity-dependent.
///
/// Convention: reads `cfrc_int[body_id]` at `xpos[body_id]`.
pub(crate) fn object_force(
    data: &Data,
    body_id: usize,
    target_pos: &Vector3<f64>,
    local_rot: Option<&Matrix3<f64>>,
) -> (Vector3<f64>, Vector3<f64>) {
    let cfrc = data.cfrc_int[body_id];
    let torque = Vector3::new(cfrc[0], cfrc[1], cfrc[2]);
    let force = Vector3::new(cfrc[3], cfrc[4], cfrc[5]);

    let (torque_at_point, force_at_point) =
        transport_force(&torque, &force, target_pos, &data.xpos[body_id]);

    match local_rot {
        Some(rot) => (rot.transpose() * torque_at_point, rot.transpose() * force_at_point),
        None => (torque_at_point, force_at_point),
    }
}
```

### 6. `object_velocity_local` — Rewrite as Backward-Compatible Wrapper

```rust
/// Compute 6D velocity at an object center in its local frame.
///
/// Equivalent to MuJoCo's `mj_objectVelocity(m, d, objtype, id, res, flg_local=1)`.
/// Returns `[ω_local; v_local]`.
///
/// This is a backward-compatible wrapper around `object_velocity`. New code
/// should call `object_velocity` directly.
pub fn object_velocity_local(
    _model: &Model,
    data: &Data,
    body_id: usize,
    point: &Vector3<f64>,
    rot: &Matrix3<f64>,
) -> [f64; 6] {
    if body_id == 0 {
        return [0.0; 6];
    }
    let (omega, v) = object_velocity(data, body_id, point, Some(rot));
    [omega.x, omega.y, omega.z, v.x, v.y, v.z]
}
```

---

## Consumer Rewrites

### 7 consumers refactored (6 sensor arms + 1 helper; 3 arms unchanged)

| Consumer | File | Before | After | Transport Type |
|----------|------|--------|-------|----------------|
| Accelerometer | `sensor/acceleration.rs` | 20 lines inline | `object_acceleration(data, body_id, &site_pos, Some(&site_mat))` | Motion + Coriolis + rotation |
| FrameLinAcc | `sensor/acceleration.rs` | 16 lines inline | `object_acceleration(data, body_id, &obj_pos, None)` | Motion + Coriolis |
| Force | `sensor/acceleration.rs` | 5 lines inline | `object_force(data, body_id, &xpos, Some(&site_mat))` | Force (rotation only) |
| Torque | `sensor/acceleration.rs` | 13 lines inline | `object_force(data, body_id, &site_pos, Some(&site_mat))` | Force + shift + rotation |
| Velocimeter | `sensor/velocity.rs` | 14 lines inline | `object_velocity(data, body_id, &site_pos, Some(&site_mat))` | Motion + rotation |
| FrameLinVel | `sensor/velocity.rs` | 12 lines inline | `object_velocity(data, body_id, &site_pos, None)` | Motion |
| `object_velocity_local` | `dynamics/spatial.rs` | 20 lines standalone | 6-line wrapper calling `object_velocity` | Wrapper |

### 3 sensor arms unchanged (no spatial transport)

| Consumer | Reason |
|----------|--------|
| FrameAngAcc | Angular acceleration is reference-point-independent. No shift needed. |
| FrameAngVel | Angular velocity is reference-point-independent. No shift needed. |
| Gyro | Same as FrameAngVel — rotation only, no transport. |

### Sensor arm sketches (after refactor)

**Accelerometer:**
```rust
MjSensorType::Accelerometer => {
    let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => (
            model.site_body[objid], data.site_xpos[objid], data.site_xmat[objid],
        ),
        MjObjectType::Body if objid < model.nbody => {
            (objid, data.xpos[objid], data.xmat[objid])
        }
        _ => { sensor_write3(&mut data.sensordata, adr, &Vector3::zeros()); continue; }
    };
    let (_alpha, a_lin) = object_acceleration(data, body_id, &site_pos, Some(&site_mat));
    sensor_write3(&mut data.sensordata, adr, &a_lin);
}
```

**FrameLinAcc:**
```rust
MjSensorType::FrameLinAcc => {
    let (body_id, obj_pos) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid])
        }
        MjObjectType::Body if objid < model.nbody => (objid, data.xpos[objid]),
        _ => { sensor_write3(&mut data.sensordata, adr, &Vector3::zeros()); continue; }
    };
    let (_alpha, a_lin) = object_acceleration(data, body_id, &obj_pos, None);
    sensor_write3(&mut data.sensordata, adr, &a_lin);
}
```

**Force:**
```rust
MjSensorType::Force => {
    let (body_id, site_mat) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xmat[objid])
        }
        MjObjectType::Body if objid < model.nbody => (objid, data.xmat[objid]),
        _ => { sensor_write3(&mut data.sensordata, adr, &Vector3::zeros()); continue; }
    };
    // Force is translation-invariant. Shift is zero (xpos → xpos).
    // We call object_force for rotation only.
    let (_torque, force) = object_force(data, body_id, &data.xpos[body_id], Some(&site_mat));
    sensor_write3(&mut data.sensordata, adr, &force);
}
```

**Torque:**
```rust
MjSensorType::Torque => {
    let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => (
            model.site_body[objid], data.site_xpos[objid], data.site_xmat[objid],
        ),
        MjObjectType::Body if objid < model.nbody => {
            (objid, data.xpos[objid], data.xmat[objid])
        }
        _ => { sensor_write3(&mut data.sensordata, adr, &Vector3::zeros()); continue; }
    };
    let (torque, _force) = object_force(data, body_id, &site_pos, Some(&site_mat));
    sensor_write3(&mut data.sensordata, adr, &torque);
}
```

**Velocimeter:**
```rust
MjSensorType::Velocimeter => {
    let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => (
            model.site_body[objid], data.site_xpos[objid], data.site_xmat[objid],
        ),
        MjObjectType::Body if objid < model.nbody => {
            (objid, data.xpos[objid], data.xmat[objid])
        }
        _ => { sensor_write3(&mut data.sensordata, adr, &Vector3::zeros()); continue; }
    };
    let (_omega, v) = object_velocity(data, body_id, &site_pos, Some(&site_mat));
    sensor_write3(&mut data.sensordata, adr, &v);
}
```

**FrameLinVel:**
```rust
MjSensorType::FrameLinVel => {
    let (body_id, site_pos) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid])
        }
        MjObjectType::Body if objid < model.nbody => (objid, data.xpos[objid]),
        _ => { sensor_write3(&mut data.sensordata, adr, &Vector3::zeros()); continue; }
    };
    let (_omega, v) = object_velocity(data, body_id, &site_pos, None);
    sensor_write3(&mut data.sensordata, adr, &v);
}
```

---

## Acceptance Criteria

### AC1: `transport_motion` — Zero Offset

**Input:** `angular = [0, 0, 10]`, `linear = [1, 2, 3]`,
`target_pos = xpos[body_id]` (same point).
**Expected:** `([0, 0, 10], [1, 2, 3])` unchanged.
**Tolerance:** Exact (bit-identical).
**Verifies:** When `r = 0`, no shift applied.

### AC2: `transport_motion` — Nonzero Offset

**Input:** `angular = [0, 0, 10]`, `linear = [0, 0, 0]`, `r = [1, 0, 0]`.
**Expected:** `angular = [0, 0, 10]`, `linear = [0, 0, 10] × [1, 0, 0] = [0, 10, 0]`.
**Tolerance:** 1e-15.
**Verifies:** Correct cross product direction for motion transport.

### AC3: `transport_force` — Nonzero Offset

**Input:** `torque = [0, 0, 0]`, `force = [0, 0, 100]`, `r = [0.5, 0, 0]`.
**Expected:** `force = [0, 0, 100]` (unchanged),
`torque = [0, 0, 0] - [0.5, 0, 0] × [0, 0, 100] = [0, -50, 0]`.
**Tolerance:** 1e-15.
**Verifies:** Correct sign and direction for force transport.

### AC4: `object_velocity` Matches `object_velocity_local`

**Model:** Rotating body (omega = [0, 0, 5] rad/s), site at offset [0.3, 0, 0].
**Expected:** `object_velocity(data, body_id, &site_pos, Some(&site_mat))`
produces the same `(omega_local, v_local)` as the existing
`object_velocity_local(model, data, body_id, &site_pos, &site_mat)`.
**Tolerance:** 1e-15 (bit-identical).
**Verifies:** New helper is a correct replacement for existing helper.

### AC5: `object_acceleration` — Coriolis Correction

**Model:** Body rotating at omega = [0, 0, 10] rad/s, no angular acceleration,
no gravity. Site at offset [0.5, 0, 0].
**Expected:** `a_linear = omega × v_at_site = [0, 0, 10] × [0, 5, 0] = [-50, 0, 0]`
(centripetal acceleration, inward).
**Tolerance:** 1e-10.
**Verifies:** Coriolis correction path matches existing Accelerometer arm.

### AC6: Sensor Behavioral Regression — Phase 4 Tests

**Test suite:** All 39 tests in `sim/L0/tests/integration/sensors_phase4.rs`.
**Expected:** All pass with tolerances unchanged.
**Verifies:** The refactor is behavior-preserving.

### AC7: Sensor Behavioral Regression — Full Sim Domain

**Test suite:** Full sim domain test suite (2,141+ tests).
**Expected:** Zero failures.
**Verifies:** No unintended side effects across the entire sim domain.

### AC8: `object_velocity_local` Backward Compatibility

**Test:** Existing callers (`forward/passive.rs` × 2, `derivatives.rs` × 2)
produce identical results before and after the rewrite.
**Verifies:** Wrapper correctly delegates to `object_velocity`.

### AC9: Convention Encoded Once (Code Review)

**Verify:** `grep -n 'xpos\[body_id\]' sim/L0/core/src/sensor/` returns zero
hits in transport-related code. The `xpos[body_id]` reference point appears only
in objtype dispatch arms (where it IS the target position for body-attached
sensors), never as a raw reference-point subtraction. All transport goes through
`transport_motion` / `transport_force` / `object_velocity` / `object_acceleration`
/ `object_force`.

### AC10: `transport_force` — Force Invariance

**Input:** `force = [3, 7, 11]`, multiple different offsets `r = [0,0,0]`,
`r = [1,0,0]`, `r = [0.5, -0.3, 0.8]`.
**Expected:** Force component of output is `[3, 7, 11]` for every offset.
**Tolerance:** Exact (bit-identical).
**Verifies:** Force is translation-invariant — only torque shifts.

### AC11: World Body (body_id = 0) Produces Valid Output

**Input:** `object_velocity(data, 0, &[1,0,0], None)` and
`object_acceleration(data, 0, &[1,0,0], None)`.
**Expected:** Velocity returns `([0,0,0], [0,0,0])` — world body is static.
Acceleration returns `([0,0,0], [0,0,−g])` where g = gravity pseudo-acceleration.
Output is finite (not NaN/Inf).
**Tolerance:** 1e-15 for velocity, 1e-10 for acceleration.
**Verifies:** Helpers handle body_id = 0 without guards. `cvel[0]` and `cacc[0]`
are valid zero-initialized arrays; the math produces correct results.

### AC12: No New `Data` Fields

This is a pure refactor. No new allocations. `EXPECTED_SIZE` unchanged at 4104.

---

## Edge Cases & Interactions

### DISABLE_SENSOR

The `DISABLE_SENSOR` flag causes `mj_sensor_vel` and `mj_sensor_acc` to return
early before the sensor loop. This refactor does NOT touch the early-return
guard — it lives above the sensor dispatch loop in both files. Existing Phase 4
tests (T13 in `SENSOR_CACC_CFRC_REFACTOR.md`) already verify this behavior.
No new tests needed; AC6 regression covers it.

### Sleep

The sleep guard (`body_sleep_state[body_id] == Asleep → continue`) runs per-
sensor inside the dispatch loop, ABOVE the sensor type match arms. This refactor
only changes code INSIDE the match arms. Sleep behavior is structurally
unaffected. Existing Phase 4 tests (T14 in `SENSOR_CACC_CFRC_REFACTOR.md`)
already verify sleep interaction. No new tests needed; AC6 regression covers it.

### Zero-Mass Bodies

Zero-mass bodies have `cacc = [0; 0]` and `cvel = [0; 0]` (no dynamics). The
transport helpers produce correct zero output: `transport_motion([0,0,0],
[0,0,0], any_pos, any_origin) = ([0,0,0], [0,0,0])`. No special guard needed.
World body (body_id = 0) is the canonical zero-mass case and is tested in T13.

---

## Test Plan

### Unit Tests (T1–T4): Pure transport kernels

**T1:** `transport_motion` — zero offset.
Direct unit test with `target_pos == body_origin`. Verify output equals input.
Exact equality.

**T2:** `transport_motion` — cross product.
`angular = [0, 0, 1]`, `linear = [0, 0, 0]`, `r = [1, 0, 0]`.
Output linear = `[0, 1, 0]`. Tolerance: 1e-15.

**T3:** `transport_force` — moment arm.
`torque = [0, 0, 0]`, `force = [0, 0, 1]`, `r = [1, 0, 0]`.
Output torque = `[0, -1, 0]`. Tolerance: 1e-15.

**T4:** `transport_force` — force invariance.
Verify force component is unchanged regardless of offset. Test with several
different `r` values. Exact equality for force component.

### Integration Tests (T5–T10): Composed helpers against models

**T5:** `object_velocity` vs `object_velocity_local` consistency.
Build a model with a rotating body. Call both functions, verify results match
to 1e-15. Tests both `Some(rot)` and `None` paths.

**T6:** `object_velocity` — world frame (`None` rotation).
Verify world-frame output matches the existing inline FrameLinVel computation
for a site-attached sensor with nonzero offset.

**T7:** `object_acceleration` — static body.
Body at rest under gravity. `object_acceleration(data, body_id, &xpos, None)`
returns `[0, 0, 0, 0, 0, +9.81]`. Tolerance: 1e-10.

**T8:** `object_acceleration` — centripetal.
Body rotating at omega = [0, 0, 10], site at offset [0.5, 0, 0], no gravity.
Linear acceleration = [-50, 0, 0]. Tolerance: 1e-8.

**T9:** `object_force` — translation invariance.
Verify force component is identical at body origin and at offset point.
Exact equality.

**T10:** `object_force` — torque transport.
Body with known `cfrc_int`. Torque at site = torque_at_origin − r × force.
Verify against hand-computed value. Tolerance: 1e-12.

### Regression Tests (T11–T12): Full suite

**T11:** Phase 4 sensor regression.
Run all 39 `sensors_phase4` tests. All pass, tolerances unchanged.

**T12:** Full sim domain regression.
Run full sim domain tests. All 2,141+ pass.

### Edge Cases (T13–T14)

**T13:** World body (body_id = 0).
`object_velocity(data, 0, &point, None)` returns finite, valid values (not NaN).
World body has `xpos[0] = [0,0,0]` and `cvel[0] = [0;0;0;0;0;0]`, so velocity
is zero. `object_acceleration(data, 0, &point, None)` returns gravity
pseudo-acceleration.

**T14:** Backward compatibility of `object_velocity_local`.
Call `object_velocity_local` from a context identical to its use in
`forward/passive.rs`. Verify return value matches pre-refactor output exactly
(bit-identical).

---

## Blast Radius

### Files Modified

| File | Change | Lines |
|------|--------|-------|
| `sim/L0/core/src/dynamics/spatial.rs` | Add `transport_motion`, `transport_force`, `object_velocity`, `object_acceleration`, `object_force`. Rewrite `object_velocity_local` as 6-line wrapper. | +~65, −~15 |
| `sim/L0/core/src/dynamics/mod.rs` | Add re-exports for new `pub(crate)` functions. | +5 |
| `sim/L0/core/src/sensor/acceleration.rs` | Rewrite Accelerometer, FrameLinAcc, Force, Torque arms to call helpers. Remove inlined transport logic. Add import for helpers. | −~55, +~10 |
| `sim/L0/core/src/sensor/velocity.rs` | Rewrite Velocimeter, FrameLinVel arms to call `object_velocity`. Remove inlined transport logic. Add import for helpers. | −~30, +~10 |
| `sim/L0/tests/integration/` | New test file `spatial_transport.rs` with T1–T14. | +~200 |

### Files NOT Modified

| File | Reason |
|------|--------|
| `sensor/acceleration.rs` — FrameAngAcc, Touch, ActuatorFrc, JointLimitFrc, TendonLimitFrc arms | Not spatial transport consumers. |
| `sensor/velocity.rs` — FrameAngVel, Gyro, JointVel, BallAngVel, SubtreeLinVel, SubtreeAngMom arms | Not spatial transport consumers. |
| `forward/passive.rs` | Calls `object_velocity_local` which retains its signature. No change. |
| `derivatives.rs` | Calls `object_velocity_local` which retains its signature. No change. |
| `types/data.rs` | No new fields. `EXPECTED_SIZE` unchanged at 4104. |

### Existing Test Impact

- **39 Phase 4 sensor tests:** Must all pass unchanged (AC6).
- **Full sim domain (2,141+ tests):** Must all pass (AC7).
- **Derivatives tests:** `object_velocity_local` wrapper must be bit-identical (AC8).

---

## Implementation Steps

1. Add `transport_motion` and `transport_force` to `dynamics/spatial.rs`.
2. Add `object_velocity` to `dynamics/spatial.rs`.
3. Add `object_acceleration` to `dynamics/spatial.rs`.
4. Add `object_force` to `dynamics/spatial.rs`.
5. Rewrite `object_velocity_local` as thin wrapper calling `object_velocity`.
6. Update `dynamics/mod.rs` re-exports.
7. Rewrite sensor arms in `acceleration.rs`: Accelerometer, FrameLinAcc, Force, Torque.
8. Rewrite sensor arms in `velocity.rs`: Velocimeter, FrameLinVel.
9. Write tests T1–T14 in `sim/L0/tests/integration/spatial_transport.rs`.
10. Run full sim domain tests — verify zero failures.

### Out of Scope

- FrameAngAcc, FrameAngVel, Gyro (angular-only, no transport).
- Switching reference point convention from `xpos[b]` to `subtree_com[root]`.
  Both produce identical sensor output. Would require rewriting velocity FK,
  forward acceleration, and backward force accumulation (~500 lines). Zero payoff.
- Objtype dispatch refactoring. The `match sensor_objtype` blocks in each sensor
  arm are NOT touched by this refactor. Simplifying those is a separate concern
  (DT-62).
