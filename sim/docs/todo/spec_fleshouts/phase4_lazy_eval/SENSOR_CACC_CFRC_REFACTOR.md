# 4A.6 — Refactor Acc-Stage Sensors to Read `cacc`/`cfrc_int`

**Status:** Draft (Revision 2 — all gaps closed)
**Scope:** Accelerometer, FrameLinAcc, FrameAngAcc read from `cacc`; Force/Torque read from `cfrc_int`.
**Prerequisite:** `flg_rnepost` lazy gate (Phase 4A) — already landed (commit 8e8f5f7).
**MuJoCo ref:** `engine_sensor.c` (`mj_sensorAcc`), `engine_core_util.c` (`mj_objectAcceleration`), `engine_util_spatial.c` (`mju_transformSpatial`).

---

## Problem Statement

After the `flg_rnepost` lazy gate, `mj_body_accumulators()` runs on demand and
populates `cacc`, `cfrc_int`, and `cfrc_ext`. However, our acc-stage sensors
still ignore these fields and recompute acceleration from scratch:

- **Accelerometer / FrameLinAcc / FrameAngAcc** call `compute_body_acceleration()`
  and `compute_body_angular_acceleration()` from `sensor/derived.rs`, which
  walk the joint tree per-body to reconstruct acceleration from `qacc`. MuJoCo
  reads `cacc[body_id]` directly and applies a spatial transform.

- **Force / Torque** call `compute_site_force_torque()`, which performs an
  O(n²) subtree walk with per-body parent-chain membership tests, recomputing
  `m*a - f_gravity` and spin angular momentum from first principles. MuJoCo
  reads `cfrc_int[body_id]` (already backward-accumulated by `mj_rnePostConstraint`)
  and applies a single spatial force transform.

This is:
1. **Non-conformant** — MuJoCo reads persistent fields; we recompute.
2. **Redundant** — `mj_body_accumulators()` already did the work.
3. **Incorrect for multi-joint bodies** — `compute_body_acceleration()` only
   considers joints on the body itself, missing propagated acceleration from
   ancestors (parent Coriolis, centripetal). `cacc` includes the full
   recursive propagation: `cacc[b] = cacc[parent] + Σ_j(S[j] * qacc[j] + cvel[parent] × (S[j] * qvel[j]))`.

---

## MuJoCo Reference

### Accelerometer

From `engine_sensor.c`:
```c
case mjSENS_ACCELEROMETER:
  mj_objectAcceleration(m, d, mjOBJ_SITE, objid, tmp, 1);  // flg_local=1
  mju_copy3(sensordata, tmp+3);  // linear part (indices 3-5)
```

`mj_objectAcceleration` (from `engine_core_util.c`, line ~280):
1. Resolves `body_id` from `(objtype, objid)`. For `mjOBJ_SITE`:
   `body_id = m->site_bodyid[objid]`, position = `d->site_xpos`.
   For `mjOBJ_BODY`: `body_id = objid`, position = `d->xipos[objid]`
   (note: MuJoCo uses body COM `xipos`, not body origin `xpos`).
2. Reads `cacc[body_id]` — 6D spatial acceleration `[angular(0:3); linear(3:6)]`
   at `subtree_com[body_rootid[body_id]]` in world frame.
3. Calls `mju_transformSpatial` with `flg_force=0` (motion transform) to shift
   from `subtree_com[root]` to the target position:
   - `r = target_pos - subtree_com[root]`
   - angular: unchanged (`alpha_at_point = alpha`)
   - linear: `a_lin_at_point = a_lin + alpha × r`
4. Reads `cvel[body_id]` (also at `subtree_com[root]`), applies the same
   motion spatial transform to get velocity at the target point:
   - `omega_at_point = omega` (angular unchanged)
   - `v_lin_at_point = v_lin + omega × r`
5. Adds Coriolis correction: `a_lin += omega × v_lin_at_point`
   (this accounts for the `ω × (ω × r)` centripetal and `2ω × v_rel` terms
   that arise when differentiating velocity at a moving reference point).
6. If `flg_local=1`: rotates result into object frame via the object's
   rotation matrix (e.g., `site_xmat` for sites).

**Our convention difference:** Our `cacc[b]` is at `xpos[b]` (body origin), not
`subtree_com[root]`. The spatial transform shift uses `r = target_pos - xpos[body_id]`.
For body-attached sensors where the site is at the body origin, `r = 0` and
the Coriolis correction is also zero (since `v_at_point = v_at_origin`).

### FrameLinAcc / FrameAngAcc

From `engine_sensor.c`:
```c
case mjSENS_FRAMELINACC:
case mjSENS_FRAMEANGACC:
  mj_objectAcceleration(m, d, objtype, objid, tmp, 0);  // flg_local=0
  // FRAMELINACC: mju_copy3(sensordata, tmp+3)  — linear part
  // FRAMEANGACC: mju_copy3(sensordata, tmp)    — angular part
```

Same as accelerometer but `flg_local=0` (world frame output) and `objtype`
comes from the sensor definition (can be site, body, or geom). MuJoCo supports
`mjOBJ_XBODY` (body frame at `xpos`), `mjOBJ_GEOM`, and `mjOBJ_SITE` for
frame sensors.

### Force / Torque

From `engine_sensor.c`:
```c
case mjSENS_FORCE:
  bodyid = m->site_bodyid[objid];
  rootid = m->body_rootid[bodyid];
  mju_transformSpatial(tmp, d->cfrc_int + 6*bodyid, 1,   // flg_force=1
                       d->site_xpos + 3*objid,            // newpos = site
                       d->subtree_com + 3*rootid,          // oldpos = subtree COM
                       d->site_xmat + 9*objid);           // rotate to site frame
  mju_copy3(sensordata, tmp+3);  // force part (indices 3-5)

case mjSENS_TORQUE:
  // Identical transform, but:
  mju_copy3(sensordata, tmp);    // torque part (indices 0-2)
```

`mju_transformSpatial` with `flg_force=1` (from `engine_util_spatial.c`):
- `force_at_new = force_at_old` (force is translation-invariant)
- `torque_at_new = torque_at_old - (newpos - oldpos) × force`
- If rotation matrix provided: rotates both components into the local frame.

**Our convention difference:** Our `cfrc_int[b]` is at `xpos[b]`, so the shift
is from `xpos[b]` to `site_xpos[objid]`:
- `r = site_xpos[objid] - xpos[body_id]`
- `force_at_site = force_at_origin` (unchanged)
- `torque_at_site = torque_at_origin - r × force`

---

## Design

### Step 1: Accelerometer — read from `cacc`

Replace `compute_body_acceleration()` call with:

```rust
MjSensorType::Accelerometer => {
    let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid], data.site_xmat[objid])
        }
        MjObjectType::Body if objid < model.nbody => {
            (objid, data.xpos[objid], data.xmat[objid])
        }
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };

    // Read cacc (spatial acceleration at xpos[body_id])
    let alpha = Vector3::new(data.cacc[body_id][0], data.cacc[body_id][1], data.cacc[body_id][2]);
    let a_lin = Vector3::new(data.cacc[body_id][3], data.cacc[body_id][4], data.cacc[body_id][5]);

    // Shift from xpos[body_id] to site_pos (motion spatial transform)
    let r = site_pos - data.xpos[body_id];
    let a_at_site = a_lin + alpha.cross(&r);

    // Coriolis correction: a += omega × v_linear_at_site
    // This accounts for ω×(ω×r) centripetal and 2ω×v_rel terms.
    // When r=0 (site at body origin), omega×v_at_site = omega×v_at_origin,
    // which is nonzero if the body is rotating — but for a single rigid body
    // with no offset, the centripetal contribution is already in cacc
    // (propagated from parent). The Coriolis term here only matters for
    // the delta between xpos[body_id] and site_pos.
    let omega = Vector3::new(data.cvel[body_id][0], data.cvel[body_id][1], data.cvel[body_id][2]);
    let v_lin = Vector3::new(data.cvel[body_id][3], data.cvel[body_id][4], data.cvel[body_id][5]);
    let v_at_site = v_lin + omega.cross(&r);
    let a_corrected = a_at_site + omega.cross(&v_at_site);

    // Accelerometer output: proper acceleration in sensor frame.
    // cacc already includes gravity pseudo-acceleration:
    //   cacc[0] = [0,0,0, -gx,-gy,-gz]
    // This propagates through the tree, so for a body at rest:
    //   cacc[b] = [0,0,0, 0,0,+g]  (proper acceleration)
    // No need to subtract model.gravity separately.
    let a_sensor = site_mat.transpose() * a_corrected;
    sensor_write3(&mut data.sensordata, adr, &a_sensor);
}
```

**Gravity note:** `cacc[0] = [0,0,0, -gx,-gy,-gz]` (negative gravity). This
propagates through the tree, so for a body at rest with default gravity
`(0, 0, -9.81)`: `cacc[b] = [0,0,0, 0,0,+9.81]` — the "proper acceleration"
that a real accelerometer measures. Our current code computes
`a_world - model.gravity` separately. After the refactor, `cacc` already
includes this, so the `- model.gravity` subtraction is removed.

### Step 2: FrameLinAcc — read from `cacc`

```rust
MjSensorType::FrameLinAcc => {
    let (body_id, obj_pos) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid])
        }
        MjObjectType::Body if objid < model.nbody => {
            (objid, data.xpos[objid])
        }
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };

    let alpha = Vector3::new(data.cacc[body_id][0], data.cacc[body_id][1], data.cacc[body_id][2]);
    let a_lin = Vector3::new(data.cacc[body_id][3], data.cacc[body_id][4], data.cacc[body_id][5]);
    let r = obj_pos - data.xpos[body_id];
    let a_at_point = a_lin + alpha.cross(&r);

    // Coriolis correction (same as accelerometer)
    let omega = Vector3::new(data.cvel[body_id][0], data.cvel[body_id][1], data.cvel[body_id][2]);
    let v_lin = Vector3::new(data.cvel[body_id][3], data.cvel[body_id][4], data.cvel[body_id][5]);
    let v_at_point = v_lin + omega.cross(&r);
    let a_corrected = a_at_point + omega.cross(&v_at_point);

    // World frame output (no rotation to local frame)
    sensor_write3(&mut data.sensordata, adr, &a_corrected);
}
```

**Conformance fix:** `FrameLinAcc` now includes gravity pseudo-acceleration
(matches MuJoCo — `mj_objectAcceleration` reads `cacc` which includes it).
Our current implementation uses `compute_body_acceleration()` which does NOT
include gravity. For a static body with default gravity, FrameLinAcc changes
from `[0, 0, 0]` to `[0, 0, +9.81]`.

### Step 3: FrameAngAcc — read from `cacc`

```rust
MjSensorType::FrameAngAcc => {
    let body_id = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => model.site_body[objid],
        MjObjectType::Body if objid < model.nbody => objid,
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };

    // Angular acceleration is reference-point-independent (rigid body).
    // No Coriolis correction needed for angular part.
    let alpha = Vector3::new(data.cacc[body_id][0], data.cacc[body_id][1], data.cacc[body_id][2]);

    // World frame output
    sensor_write3(&mut data.sensordata, adr, &alpha);
}
```

Angular acceleration does not depend on reference point for a rigid body.
This follows from the motion spatial transform: the angular component is
invariant under point translation (`alpha_at_B = alpha_at_A` for all A, B
on the same rigid body). No Coriolis correction needed.

### Step 4: Force — read from `cfrc_int`

```rust
MjSensorType::Force => {
    let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid], data.site_xmat[objid])
        }
        MjObjectType::Body if objid < model.nbody => {
            (objid, data.xpos[objid], data.xmat[objid])
        }
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };

    // Read cfrc_int (spatial wrench [torque(0:3); force(3:6)] at xpos[body_id])
    let force = Vector3::new(
        data.cfrc_int[body_id][3],
        data.cfrc_int[body_id][4],
        data.cfrc_int[body_id][5],
    );

    // Force is translation-invariant (spatial force transform):
    // force_at_site = force_at_origin. No shift needed.
    // Rotate to site frame.
    let force_site = site_mat.transpose() * force;
    sensor_write3(&mut data.sensordata, adr, &force_site);
}
```

Force is the linear component of the spatial wrench and does not change
under reference-point translation. This follows from the force spatial
transform: `F_new = F_old`, `τ_new = τ_old - r × F_old`.

### Step 5: Torque — read from `cfrc_int`

```rust
MjSensorType::Torque => {
    let (body_id, site_pos, site_mat) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid], data.site_xmat[objid])
        }
        MjObjectType::Body if objid < model.nbody => {
            (objid, data.xpos[objid], data.xmat[objid])
        }
        _ => {
            sensor_write3(&mut data.sensordata, adr, &Vector3::zeros());
            continue;
        }
    };

    // Read cfrc_int (spatial wrench [torque(0:3); force(3:6)] at xpos[body_id])
    let torque_at_origin = Vector3::new(
        data.cfrc_int[body_id][0],
        data.cfrc_int[body_id][1],
        data.cfrc_int[body_id][2],
    );
    let force = Vector3::new(
        data.cfrc_int[body_id][3],
        data.cfrc_int[body_id][4],
        data.cfrc_int[body_id][5],
    );

    // Spatial force transform: shift torque from xpos[body_id] to site_pos
    // torque_at_site = torque_at_origin - (site_pos - xpos[body_id]) × force
    let r = site_pos - data.xpos[body_id];
    let torque_at_site = torque_at_origin - r.cross(&force);

    // Rotate to site frame
    let torque_site = site_mat.transpose() * torque_at_site;
    sensor_write3(&mut data.sensordata, adr, &torque_site);
}
```

Torque (moment) changes under reference-point translation by the standard
spatial force transport formula: `τ_new = τ_old - r × F`.

### Step 6: Delete dead helpers and update imports

After Steps 1-5, three functions in `sensor/derived.rs` become dead code:
- `compute_body_acceleration()` — only called from `mj_sensor_acc` (Accelerometer, FrameLinAcc arms) and from `compute_site_force_torque`
- `compute_body_angular_acceleration()` — only called from `mj_sensor_acc` (FrameAngAcc arm) and from `compute_site_force_torque`
- `compute_site_force_torque()` — only called from `mj_sensor_acc` (Force, Torque arms)

**Verify before deleting:** Search for callers in:
- `sensor/acceleration.rs` — will be rewritten (Steps 1-5)
- `sensor/derived.rs` — internal calls (deleted together)
- No other callers exist (confirmed by `grep -r "compute_body_acceleration\|compute_body_angular_acceleration\|compute_site_force_torque" sim/L0/`)

After deletion, `sensor/derived.rs` becomes empty. **Delete the entire module**
and remove:
- `mod derived;` from `sensor/mod.rs`
- `use super::derived::{...};` from `sensor/acceleration.rs`
- No other modules import from `sensor::derived`

Also remove the now-unused imports from `sensor/acceleration.rs`:
- `use nalgebra::Matrix3` — no longer needed (we read `data.site_xmat[objid]`
  which is already `Matrix3<f64>`, but the `_ => Matrix3::identity()` fallback
  is replaced with `continue`). **Check:** `Matrix3` may still be needed if any
  fallback arm uses it. Per the design above, all fallbacks use `continue` —
  `Matrix3` import can be removed.

---

## Convention Notes

### Reference Points

| Field | Our Convention | MuJoCo Convention |
|-------|---------------|-------------------|
| `cacc[b]` | at `xpos[b]` (body origin) | at `subtree_com[body_rootid[b]]` |
| `cfrc_int[b]` | at `xpos[b]` (body origin) | at `subtree_com[body_rootid[b]]` |
| `cvel[b]` | at `xpos[b]` (body origin) | at `subtree_com[body_rootid[b]]` |

The spatial transform formulas are the same; only the "old position" changes:
- MuJoCo: `r = target_pos - subtree_com[root]`
- Ours: `r = target_pos - xpos[body_id]`

For body-attached sensors where the site is at the body origin, `r = 0` and
both conventions yield the same result. When `r = 0`, the Coriolis correction
`omega × v_at_site` reduces to `omega × v_at_origin`, which is the same
value regardless of reference point convention (both `cvel[b]` store the
velocity at their respective reference points, but at `r = 0` we're reading
the velocity at the reference point itself — no lever-arm shift).

### SpatialVector Layout

Both MuJoCo and our codebase use `[angular(0:3); linear(3:6)]`.
- `cacc[b][0..3]` = angular acceleration (α)
- `cacc[b][3..6]` = linear acceleration (a)
- `cfrc_int[b][0..3]` = torque (τ)
- `cfrc_int[b][3..6]` = force (F)

### Gravity in `cacc`

`cacc[0] = [0,0,0, -gx,-gy,-gz]`. For default gravity `(0, 0, -9.81)`:
`cacc[0] = [0, 0, 0, 0, 0, +9.81]`.

This propagates through the tree via `cacc[b] = cacc[parent] + joint_terms`.
A stationary body (zero joint velocities and accelerations) has
`cacc[b] = cacc[parent] = ... = cacc[0] = [0,0,0, 0,0,+9.81]` — the proper
acceleration. Accelerometer reads this directly (in sensor frame).
FrameLinAcc also includes this gravity term (matches MuJoCo). Our current
`compute_body_acceleration()` does NOT include gravity — this is a
conformance gap that this refactor fixes.

### `cfrc_int` formula

From `mj_body_accumulators()` (`forward/acceleration.rs`, line ~583):
```
cfrc_int[b] = cinert[b] * cacc[b] + cvel[b] ×* (cinert[b] * cvel[b]) - cfrc_ext[b]
```
This is the Newton-Euler equation in spatial form: `I*a + v×*(I*v) - f_ext`.
The backward accumulation then propagates: `cfrc_int[parent] += cfrc_int[child]`.

---

## Acceptance Criteria

### AC1: Accelerometer reads `cacc` — static body
**Model:** Single body (mass=1.0 kg) with free joint, resting on a plane
(constrained at z=0). Gravity = `(0, 0, -9.81)`. Site "imu" at body origin
with identity orientation.
**Expected:** Accelerometer reads `[0.0, 0.0, +9.81]` (proper acceleration,
sensor frame = world frame). Tolerance: 1e-10.
**Verifies:** `cacc` includes gravity pseudo-acceleration; no explicit
`- model.gravity` needed.

### AC2: Accelerometer reads `cacc` — site offset with rotation
**Model:** Single body (mass=2.0 kg) with free joint. Body at origin,
site "imu" at offset `[0.5, 0.0, 0.0]` from body origin. Body rotating
at `ω = [0, 0, 10]` rad/s (set via `qvel[3..6] = [0, 0, 10]`). No gravity
(`<option gravity="0 0 0"/>`). Zero linear velocity.
**Expected:** Centripetal acceleration = `ω²r = 100 × 0.5 = 50 m/s²` directed
from site toward body origin (−x in world frame).
Sensor output (in site frame, assuming site orientation = body orientation = identity):
`[-50.0, 0.0, 0.0]`. Tolerance: 1e-8.
**Breakdown:** `r = [0.5, 0, 0]`, `alpha × r = 0` (no angular acceleration),
`omega × v_at_site = [0,0,10] × ([0,0,0] + [0,0,10]×[0.5,0,0]) = [0,0,10] × [0,5,0] = [-50,0,0]`.
**Verifies:** Coriolis correction path with nonzero `r`.

### AC3: FrameLinAcc includes gravity (conformance fix)
**Model:** Single body (mass=1.0 kg) with free joint, resting on plane.
Default gravity `(0, 0, -9.81)`. FrameLinAcc sensor on body.
**Expected:** `[0.0, 0.0, +9.81]` in world frame. Tolerance: 1e-10.
**This is a behavior change** — our current implementation returns
`[0.0, 0.0, 0.0]` because `compute_body_acceleration()` does not include
gravity. MuJoCo returns `[0.0, 0.0, +9.81]` because `mj_objectAcceleration`
reads `cacc` which includes the gravity pseudo-acceleration.

### AC4: FrameAngAcc reads `cacc` angular — hinge joint
**Model:** Single body (mass=1.0 kg) with y-axis hinge joint. Apply torque
producing `qacc[0] = 5.0` rad/s². FrameAngAcc sensor on body.
**Expected:** `[0.0, 5.0, 0.0]` in world frame (y-axis hinge, angular
acceleration about y). Tolerance: 1e-10.
**Verifies:** Angular acceleration read from `cacc[body_id][0..3]`.

### AC5: Force reads `cfrc_int` — single body under gravity
**Model:** Single body (mass=2.0 kg) with free joint, resting on plane.
Default gravity `(0, 0, -9.81)`. Force sensor at site on body origin
with identity orientation.
**Expected:** Force sensor reads `[0.0, 0.0, m*g]` = `[0.0, 0.0, +19.62]`
in site frame. (The internal constraint force supporting the body against
gravity is upward.)
**Note:** Sign convention: `cfrc_int` is the force needed to produce the
observed acceleration (Newton-Euler). For a static body on a plane,
`cacc = [0,0,0, 0,0,+9.81]`, `cfrc_int = m*cacc_lin = [0, 0, +19.62]`
(before subtracting `cfrc_ext`). The exact value depends on `cfrc_ext`
(external forces including gravity). Tolerance: 1e-6.

### AC6: Torque reads `cfrc_int` with moment arm
**Model:** Single body (mass=1.0 kg) with free joint. `xfrc_applied` =
`[0, 0, 0, 0, 0, 100.0]` (pure force `Fz = 100 N` at body origin, zero
torque). Torque sensor at site offset `[0.5, 0.0, 0.0]` from body origin.
No gravity (`<option gravity="0 0 0"/>`).
**Expected:** Spatial force transform: `τ_at_site = τ_at_origin - r × F`
= `[0,0,0] - [0.5, 0, 0] × [0, 0, 100]` = `[0, -50.0, 0]`.
In site frame (identity orientation): `[0.0, -50.0, 0.0]`. Tolerance: 1e-6.
**Verifies:** Spatial force transport from `xpos[body_id]` to `site_xpos`.

### AC7: Force/Torque on 3-link chain matches pre-refactor
**Model:** 3-link serial chain (hinge joints, equal masses = 1.0 kg each,
link length = 0.5 m). Default gravity. Force and torque sensors on link 1
(root of chain). Let simulation settle for several steps.
**Expected:** Force and torque values match pre-refactor
`compute_site_force_torque()` within 1e-6. (Both algorithms compute the
same Newton-Euler equation — `cfrc_int` backward-accumulation vs explicit
subtree sum — and should agree in exact arithmetic. The tolerance accounts
for different floating-point accumulation order.)

### AC8: Dead code deleted *(code review)*
`compute_body_acceleration()`, `compute_body_angular_acceleration()`, and
`compute_site_force_torque()` are deleted. The `sensor/derived.rs` module
is deleted entirely. No remaining callers in the codebase. The `mod derived;`
declaration and all `use super::derived::*` imports are removed from
`sensor/mod.rs` and `sensor/acceleration.rs`.

### AC9: No new `Data` fields
This refactor reads existing fields (`cacc`, `cfrc_int`, `cvel`, `xpos`,
`site_xpos`, `site_xmat`). No new allocations. `EXPECTED_SIZE` unchanged.

---

## Test Plan

### T1: Accelerometer — free fall
Free body (mass=1.0 kg), initial velocity `[1, 0, 0]`, default gravity.
After one `forward()` step, accelerometer reads `~[0, 0, 0]` (±1e-6).
Proper acceleration of free fall is zero (gravity and true acceleration
cancel in `cacc`). Verifies gravity cancellation via `cacc`.

### T2: Accelerometer — static under gravity
Body (mass=1.0 kg) resting on plane. Default gravity. After equilibrium,
accelerometer reads `[0, 0, +9.81]` (±1e-10) in world-aligned site frame.
Verifies `cacc` proper acceleration for static body.

### T3: Accelerometer — site offset + rotation (centripetal)
Body rotating at `ω = [0, 0, 10]` rad/s, no gravity. Site at `[0.5, 0, 0]`
offset from body origin. Accelerometer reads `[-50, 0, 0]` (±1e-8) in site
frame (centripetal = ω²r = 50 m/s² inward). Tests the Coriolis correction
path with nonzero `r`.

### T4: FrameLinAcc — gravity inclusion (conformance fix)
Static body on plane, default gravity. FrameLinAcc reads `[0, 0, +9.81]`
(±1e-10). **This is a NEW behavior** — currently returns `[0, 0, 0]`.

### T5: FrameAngAcc — hinge joint
Y-axis hinge with `qacc[0] = 5.0`. FrameAngAcc reads `[0, 5.0, 0]` (±1e-10).
Matches expected `alpha` from `cacc[body_id][0..3]`.

### T6: Force sensor — single body
Free body (mass=2.0 kg) on plane, default gravity. Force sensor at body
origin reads `[0, 0, +19.62]` (±1e-6) — upward support force.

### T7: Force sensor — 3-link chain regression
3-link chain (hinge joints, mass=1.0 kg each, length=0.5 m). Force sensor
on link 1. Values match pre-refactor `compute_site_force_torque()` within
1e-6. Record pre-refactor values before implementing.

### T8: Torque sensor — moment arm
Body with `xfrc_applied = [0,0,0, 0,0,100]`, no gravity. Torque sensor at
site offset `[0.5, 0, 0]`. Expects `[0, -50, 0]` (±1e-6) from spatial
force transport `τ_site = τ_origin - r × F`.

### T9: Torque sensor — 3-link chain regression
Same as T7 but for torque component. Match within 1e-6.

### T10: Existing sensor tests still pass
All existing accelerometer, force, torque, and frame acceleration tests in
`sensor/mod.rs` pass unchanged — except tests that check FrameLinAcc for
gravity-free values, which must be updated (see T4). Specific tests to check:
- `test_accelerometer_at_rest_reads_gravity()` — should still pass (reads +9.81)
- `test_accelerometer_in_free_fall_reads_zero()` — should still pass (reads ~0)
- `test_force_sensor_at_rest_in_gravity()` — verify value matches (may need tolerance update)
- `test_torque_sensor_at_rest_in_gravity()` — verify value matches

### T11: World body sensor (edge case)
Sensor attached to world body (body_id=0). `cacc[0] = [0,0,0, 0,0,+9.81]`
(gravity pseudo-acceleration). `cfrc_int[0]` = sum of all body forces
(root of backward accumulation). Verify sensor produces valid, finite values.
For accelerometer on world body: reads `[0, 0, +9.81]` in world frame.

### T12: Zero-mass body (edge case)
Body with mass ≈ 0 (`1e-20`). `cfrc_int` for this body is dominated by the
`cinert * cacc` term which may be near-zero. Force/torque sensor should
return near-zero without NaN or infinity.

### T13: DISABLE_SENSOR (edge case)
Model with accelerometer + force sensors, `DISABLE_SENSOR` flag set.
`mj_sensor_acc()` returns early. Sensor data remains at previous values
(NOT zeroed — matches MuJoCo S4.10 behavior). `mj_body_accumulators()` is
NOT triggered (lazy gate not reached). `flg_rnepost` stays false.

### T14: Sleep interaction (edge case)
Two bodies: body A sleeping, body B awake. Accelerometer on body B (awake).
After `forward()`: `flg_rnepost = true` (body B's sensor triggered the gate).
Body A's sensor (if any) is skipped by §16.5d sleep check. Verify
`mj_body_accumulators()` computed `cacc` for ALL bodies (including sleeping
body A) — the RNE pass is not sleep-gated.

---

## Files Affected

| File | Change |
|------|--------|
| `sim/L0/core/src/sensor/acceleration.rs` | Rewrite 5 sensor arms (Accelerometer, FrameLinAcc, FrameAngAcc, Force, Torque) to read `cacc`/`cfrc_int`. Remove `use super::derived::*` import. Remove `use nalgebra::Matrix3` (replaced by existing `data.site_xmat` type). |
| `sim/L0/core/src/sensor/derived.rs` | Delete entire module (all 3 remaining functions become dead code). |
| `sim/L0/core/src/sensor/mod.rs` | Remove `mod derived;` declaration. |
| `sim/L0/tests/integration/` | New/updated tests for the 5 sensor types (T1-T14). |

**Existing test impact:**
- `body_accumulators.rs` — unaffected (tests `cacc`/`cfrc_int` computation, not sensor reading)
- `sensor/mod.rs::test_accelerometer_at_rest_reads_gravity` — should still pass (proper acceleration is unchanged)
- `sensor/mod.rs::test_accelerometer_in_free_fall_reads_zero` — should still pass
- `sensor/mod.rs::test_force_sensor_at_rest_in_gravity` — values may change slightly due to different algorithm; verify and update tolerance if needed
- `sensor/mod.rs::test_torque_sensor_at_rest_in_gravity` — same as above
- No FrameLinAcc or FrameAngAcc tests exist currently — no breakage risk there

**Data staleness guard:** No new `Data` fields. `EXPECTED_SIZE` unchanged.

---

## Risks

### R1: FrameLinAcc behavior change (AC3)
FrameLinAcc will now include gravity. This matches MuJoCo but changes our
current output. Any downstream code that reads FrameLinAcc and assumes
gravity-free values will break. **Mitigation:** Search for FrameLinAcc
consumers before implementing. Currently no known consumers beyond tests.

### R2: Numerical differences in Force/Torque
The `cfrc_int` backward-accumulation path and the old
`compute_site_force_torque()` subtree-sum path should produce identical
results in exact arithmetic, but may differ by O(ε) in floating-point.
Use 1e-6 tolerance for regression tests. **Mitigation:** T7 and T9
explicitly test this.

### R3: `compute_body_acceleration()` bugs now exposed
Our current `compute_body_acceleration()` only considers joints on the target
body, missing ancestor contributions (parent angular acceleration, Coriolis
from grandparent, etc.). `cacc` includes the full recursive propagation via
`cacc[b] = cacc[parent] + Σ_j(S[j]*qacc[j] + cvel[parent]×(S[j]*qvel[j]))`.
For single-joint-per-body models (most tests), both agree. For multi-level
chains with hinge joints, `cacc` is more correct. This means some sensor
values may change slightly for complex models — this is a correctness
improvement, not a regression.

---

## Out of Scope

- **Geom-attached sensors** (`MjObjectType::Geom` for FrameLinAcc/FrameAngAcc).
  MuJoCo supports geom-attached frame sensors via `mj_objectAcceleration(m, d, mjOBJ_GEOM, ...)`.
  Currently handled by our `_ => { continue }` fallback (writes zeros).
  Adding geom support requires reading `geom_xpos`/`geom_xmat` and resolving
  `geom_body[objid]`. Tracked as DT-102 (depends on DT-62 for objtype parsing).
- **`mj_objectAcceleration()` as a standalone utility function.** For now, the
  spatial transform is inlined in each sensor arm (Steps 1-3 share the same
  pattern). If a fourth consumer appears, factor out into a shared helper in
  `sensor/` or `types/spatial.rs`.

---

## Relationship to Umbrella Spec

This spec implements **4A.6** from
[PHASE4_LAZY_EVAL_SPEC.md](./PHASE4_LAZY_EVAL_SPEC.md). The umbrella spec's
4A.6 section describes the motivation and lists the sensor types to refactor.
The umbrella spec's Out of Scope previously referenced "4A.6b" as a separate
sub-step for the Force/Torque `cfrc_int` refactor — this spec combines both
(acceleration sensors + force/torque sensors) into a single implementation
since the prerequisite (`flg_rnepost` lazy gate) is already landed and the
two refactors share the same test infrastructure.
