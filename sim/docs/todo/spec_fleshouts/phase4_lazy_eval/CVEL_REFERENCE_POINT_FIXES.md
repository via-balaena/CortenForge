# Phase 4 Prerequisite — `cvel` Reference Point Fixes

**Status:** Implemented & Audited (2026-02-26)
**Priority:** Must-fix before §56
**Scope:** 5 latent bugs where sensor/energy code reads `cvel[b][3:6]` without
shifting from `xpos[b]` to the physically correct reference point.

---

## Background

Our `cvel[b]` stores the 6D spatial velocity `[angular; linear]` at `xpos[b]`
(body frame origin). MuJoCo stores it at `subtree_com[body_rootid[b]]`.

The core dynamics pipeline (RNE, CRBA, body accumulators, derivatives,
constraints) is self-consistent — `cinert[b]` and `cvel[b]` are both at
`xpos[b]`, so all spatial algebra is correct.

The bugs are in **consumer code** that extracts the linear component of `cvel`
and treats it as a physical velocity at some other point (site position, body
COM) without doing the lever-arm shift:

```
v_point = v_origin + ω × (point - xpos[b])
```

The angular component `cvel[b][0:3]` is always correct — angular velocity is
reference-point-independent.

---

## Bug 1: Velocimeter Sensor — Missing Site Shift

**File:** `sensor/velocity.rs:111-136`

**Problem:** For site-attached velocimeters, reads `cvel[body][3:6]` (velocity
at body origin) and rotates to sensor frame. Does not shift to the site
position first. For bodies where the site is offset from the origin, the
reported linear velocity is wrong.

**MuJoCo behavior:** Calls `mj_objectVelocity(m, d, mjOBJ_SITE, id, res, 0)`
which shifts from `subtree_com[root]` to the site position, then
`mj_objectVelocity(... flg_local=1)` rotates to local frame.

**Fix:**

```rust
MjSensorType::Velocimeter => {
    let (v_world, site_mat) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            let body_id = model.site_body[objid];
            let omega = Vector3::new(
                data.cvel[body_id][0],
                data.cvel[body_id][1],
                data.cvel[body_id][2],
            );
            let v_origin = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );
            // Shift from body origin (xpos) to site position
            let dif = data.site_xpos[objid] - data.xpos[body_id];
            let v_site = v_origin + omega.cross(&dif);
            (v_site, data.site_xmat[objid])
        }
        MjObjectType::Body if objid < model.nbody => {
            // Body velocity at body origin — correct as-is
            let v = Vector3::new(
                data.cvel[objid][3],
                data.cvel[objid][4],
                data.cvel[objid][5],
            );
            (v, data.xmat[objid])
        }
        _ => (Vector3::zeros(), Matrix3::identity()),
    };
    let v_sensor = site_mat.transpose() * v_world;
    sensor_write3(&mut data.sensordata, adr, &v_sensor);
}
```

**MuJoCo calling convention:** MuJoCo's velocimeter calls
`mj_objectVelocity(m, d, mjOBJ_SITE, id, res, flg_local=0)` to shift from the
reference point to the site position in **world frame**, then rotates to
sensor frame via `site_xmat^T * v_world`. Our fix mirrors this: shift from
`xpos[body]` to `site_xpos[site]`, then rotate.

**Acceptance criteria:**
- B1-AC1: Hinge joint at world origin, z-axis, site at `[0, 0, 0.5]` from
  body origin, `qvel = 1.0` rad/s. Velocimeter reads `[-0.5, 0, 0]` in
  sensor frame (tangential velocity `v = ω × r`). Old code: `[0, 0, 0]`.
- B1-AC2: Same model, velocimeter on body (not site). Output unchanged from
  pre-fix baseline.
- B1-AC3: Site at `[0.3, 0.4, 0]` from body origin, `qvel = 2.0` rad/s
  about z-axis. Velocimeter reads `[-0.8, 0.6, 0]` in world frame before
  rotation (verify `|v| = ω * r = 2.0 * 0.5 = 1.0`).

---

## Bug 2: FrameLinVel Sensor — Missing Site Shift

**File:** `sensor/velocity.rs:138-157`

**Problem:** Identical to Bug 1. For site-attached FrameLinVel sensors, reads
velocity at body origin without shifting to site position.

**MuJoCo behavior:** Same as Bug 1 — `mj_objectVelocity` shifts to the
object's actual position.

**Fix:**

```rust
MjSensorType::FrameLinVel => {
    let v = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            let body_id = model.site_body[objid];
            let omega = Vector3::new(
                data.cvel[body_id][0],
                data.cvel[body_id][1],
                data.cvel[body_id][2],
            );
            let v_origin = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );
            // Shift from body origin to site position
            let dif = data.site_xpos[objid] - data.xpos[body_id];
            v_origin + omega.cross(&dif)
        }
        MjObjectType::Body if objid < model.nbody => Vector3::new(
            data.cvel[objid][3],
            data.cvel[objid][4],
            data.cvel[objid][5],
        ),
        _ => Vector3::zeros(),
    };
    sensor_write3(&mut data.sensordata, adr, &v);
}
```

**Note:** FrameLinVel reports in **world frame** (no rotation to sensor frame),
unlike Velocimeter which reports in sensor frame. The only change is the
lever-arm shift.

**Acceptance criteria:**
- B2-AC1: Same model as B1-AC1 (hinge z-axis, site at `[0, 0, 0.5]`,
  `qvel = 1.0`). FrameLinVel reads `[-0.5, 0, 0]` in **world frame** (no
  rotation to sensor frame). Old code: `[0, 0, 0]`.
- B2-AC2: FrameLinVel on a body (not site). Output unchanged.

---

## Bug 3: `compute_subtree_momentum()` — Velocity at Body Origin, Not COM

**File:** `sensor/derived.rs:42-68`

**Problem:** Linear momentum is `p = m * v_com` where `v_com` is the velocity
at the body's center of mass (`xipos[b]`). The code uses `cvel[b][3:6]`
which is velocity at `xpos[b]` (body origin). For bodies where `xipos ≠ xpos`
(COM offset from origin), the momentum is wrong.

**Error magnitude:** `m * ω × (xipos[b] - xpos[b])` per body.

**Fix:**

```rust
pub fn compute_subtree_momentum(model: &Model, data: &Data, root_body: usize) -> Vector3<f64> {
    let mut momentum = Vector3::zeros();
    for body_id in root_body..model.nbody {
        // ... descendant check ...
        if is_descendant {
            let mass = model.body_mass[body_id];
            let omega = Vector3::new(
                data.cvel[body_id][0],
                data.cvel[body_id][1],
                data.cvel[body_id][2],
            );
            let v_origin = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );
            // Shift from body origin (xpos) to body COM (xipos)
            let dif = data.xipos[body_id] - data.xpos[body_id];
            let v_com = v_origin + omega.cross(&dif);
            momentum += mass * v_com;
        }
    }
    momentum
}
```

**Note:** This function is replaced by §56's `mj_subtree_vel()` which already
handles the shift correctly. But the fix should land first because:
1. The function is used by existing sensor tests (regression baseline).
2. §56's O(n) algorithm will be validated against this corrected O(n²) helper.

**Acceptance criteria:**
- B3-AC1: Single body, mass=1.0, `<inertial pos="0 0 0.1"/>`, hinge z-axis,
  `qvel = 1.0`. COM offset = `[0, 0, 0.1]`, so shift = `ω × dif` =
  `[0,0,1] × [0,0,0.1] = [0, 0, 0]` — wait, that's zero for z-axis rotation
  with z-offset. Use x-axis offset: `<inertial pos="0.1 0 0"/>`, hinge z-axis,
  `qvel = 1.0`. Shift = `[0,0,1] × [0.1, 0, 0] = [0, 0.1, 0]`.
  `compute_subtree_momentum` returns `[0, 0.1, 0]`, not `[0, 0, 0]`.
- B3-AC2: Same model but `<inertial pos="0 0 0"/>` (COM at origin). Momentum
  unchanged from pre-fix baseline.
- B3-AC3: `SubtreeLinVel` sensor on the above model reads `[0, 0.1, 0]`
  (momentum / mass = `[0, 0.1, 0]`).

---

## Bug 4: `compute_subtree_angmom()` — Inconsistent Velocity/Lever-Arm

**File:** `sensor/derived.rs:305-346`

**Problem:** The orbital angular momentum term uses:
- Lever arm: `r = xipos[b] - subtree_com` (from subtree COM to body COM)
- Velocity: `cvel[b][3:6]` (at body origin, `xpos[b]`)

These are inconsistent. The formula `L = m * r × v` requires `v` at the same
point as the end of the lever arm (`xipos`), or equivalently, `v_com`.

**Fix:**

```rust
pub fn compute_subtree_angmom(model: &Model, data: &Data, root_body: usize) -> Vector3<f64> {
    let (com, _total_mass) = compute_subtree_com(model, data, root_body);
    let mut angmom = Vector3::zeros();

    for body_id in root_body..model.nbody {
        if !is_body_in_subtree(model, body_id, root_body) {
            continue;
        }

        let mass = model.body_mass[body_id];
        let omega = Vector3::new(
            data.cvel[body_id][0],
            data.cvel[body_id][1],
            data.cvel[body_id][2],
        );
        let v_origin = Vector3::new(
            data.cvel[body_id][3],
            data.cvel[body_id][4],
            data.cvel[body_id][5],
        );

        // Shift velocity from body origin (xpos) to body COM (xipos)
        let com_offset = data.xipos[body_id] - data.xpos[body_id];
        let v_com = v_origin + omega.cross(&com_offset);

        // Orbital: L = m * (xipos - subtree_com) × v_com
        let r = data.xipos[body_id] - com;
        angmom += mass * r.cross(&v_com);

        // Spin: L = R * diag(I) * R^T * omega (unchanged — angular is correct)
        let inertia = model.body_inertia[body_id];
        let xi_mat = data.ximat[body_id];
        let omega_local = xi_mat.transpose() * omega;
        let i_omega_local = Vector3::new(
            inertia.x * omega_local.x,
            inertia.y * omega_local.y,
            inertia.z * omega_local.z,
        );
        angmom += xi_mat * i_omega_local;
    }

    angmom
}
```

**Same note as Bug 3:** This function is replaced by §56's `mj_subtree_vel()`.
Fix it first to establish a correct baseline for validation.

**Acceptance criteria:**
- B4-AC1: Same model as B3-AC1 (`<inertial pos="0.1 0 0"/>`, hinge z-axis,
  `qvel = 1.0`). The orbital term `m * r × v_com` uses the corrected `v_com`
  (includes lever-arm shift `[0, 0.1, 0]`). Verify orbital contribution
  differs from pre-fix value by `m * r × (ω × com_offset)`.
- B4-AC2: Spin angular momentum `R * diag(I) * R^T * ω` is identical
  before/after fix (angular velocity is reference-point-independent).
- B4-AC3: `SubtreeAngMom` sensor on a model with COM offsets changes output.

---

## Bug 5: Kinetic Energy Fallback — Velocity at Body Origin, Not COM

**File:** `energy.rs:76-107`

**Problem:** The per-body kinetic energy fallback computes `0.5 * m * |v|²`
using `cvel[b][3:6]` (velocity at body origin). König's theorem requires
`v_com` (velocity at body COM):

```
KE = 0.5 * m * |v_com|² + 0.5 * ω^T * I_com * ω
```

**Practical impact:** Low — the primary path `0.5 * qvel^T * M * qvel` is
correct and is what runs when `qM` is available (always after CRBA).

**Fix:**

```rust
let omega = Vector3::new(data.cvel[body_id][0], data.cvel[body_id][1], data.cvel[body_id][2]);
let v_origin = Vector3::new(data.cvel[body_id][3], data.cvel[body_id][4], data.cvel[body_id][5]);

// Shift from body origin to body COM
let com_offset = data.xipos[body_id] - data.xpos[body_id];
let v_com = v_origin + omega.cross(&com_offset);

energy += 0.5 * mass * v_com.norm_squared();
```

**Acceptance criteria:**
- B5-AC1: Model with `<inertial pos="0.1 0 0"/>`, hinge z-axis, `qvel = 1.0`.
  Per-body KE fallback (`0.5 * m * |v_com|² + 0.5 * ω^T * I * ω`) matches
  `0.5 * qvel^T * M * qvel` within `1e-10`.
- B5-AC2: Same model with `<inertial pos="0 0 0"/>`. Energy unchanged from
  pre-fix baseline.

---

## When Are These Bugs Observable?

All 5 bugs produce zero error when `xipos[b] == xpos[b]` (body COM at body
origin) AND when sites are at body origins. This is the case for:

- Simple models with geometry centered on the body frame
- Free-floating bodies with symmetric geometry
- Most basic test models

The bugs become observable when:

- **Site offset from body:** Bugs 1, 2 — any model with sites not at body origin
- **COM offset from body origin:** Bugs 3, 4, 5 — bodies with asymmetric mass
  distribution, or bodies with `<inertial pos="..."/>` or `<geom pos="..."/>`
  that shifts the COM away from the body frame origin

In practice, most humanoid and robot models have COM offsets on many bodies
(limbs with offset geometry), so these bugs would be observable on real models.

**Existing test impact:** Most existing sensor tests use symmetric models with
COM at body origin and sites at body origins. These tests will NOT change
output after the fix (T6 regression guarantee). Tests that DO have COM offsets
or site offsets will see improved (corrected) values — this is intentional.

---

## Dependencies

This spec is **Phase 4 Step 0** — a prerequisite to both:
- [S56_SUBTREE_VEL_SPEC.md](./S56_SUBTREE_VEL_SPEC.md) — the corrected O(n²)
  helpers serve as the validation baseline for §56's O(n) equivalence test (T9).
- [PHASE4_LAZY_EVAL_SPEC.md](./PHASE4_LAZY_EVAL_SPEC.md) — lazy gates assume
  correct sensor baselines.

**Data staleness guard:** No new `Data` fields are added by this spec. The
`data_reset_field_inventory` compile-time size check is unaffected.

---

## Implementation Order

1. Fix Bug 1 (Velocimeter) + Bug 2 (FrameLinVel) — independent, same pattern
2. Fix Bug 3 (subtree momentum) + Bug 4 (subtree angmom) — same file, same pattern
3. Fix Bug 5 (energy fallback) — independent
4. Run full sensor test suite to verify regressions are improvements
5. Commit as "fix(sim): cvel reference point shifts for sensors and energy"

All fixes are ≤5 lines each. Total blast radius: ~25 lines of code changed.

---

## Test Plan

### T1: Velocimeter site offset
Hinge joint, site at `[0, 0, 0.5]` from body origin. Joint velocity = 1.0
rad/s about z-axis. Velocimeter should read tangential velocity
`v = ω × r = [−0.5, 0, 0]` (site-local), not zero (body-origin has no
tangential velocity when joint axis passes through origin).

### T2: FrameLinVel site offset
Same setup as T1 but using FrameLinVel sensor. Output in world frame should
show tangential velocity component.

### T3: Subtree momentum with COM offset
Body with `<inertial pos="0 0 0.1"/>`, mass=1, angular velocity ω_z=1.
Correct momentum includes `m * ω × (xipos - xpos) = [−0.1, 0, 0]`.
Old code would miss this term.

### T4: Subtree angular momentum with COM offset
Same model as T3. Orbital angular momentum about world COM should use
`v_com`, not `v_origin`. Compare against analytical formula.

### T5: Energy consistency
Model with COM offsets. Compare per-body KE fallback against
`0.5 * qvel^T * M * qvel`. Should match within `1e-10`.

### T6: Regression — no change when COM at origin
Simple model with COM at body origin and sites at body origins. All sensor
values and energy unchanged before/after fix.

### T7: World body edge case — no crash
Access `cvel[0]` (world body). Velocimeter attached to a site on the world
body. No panic, no NaN. World body has zero velocity, so output is zero.

### T8: Zero-mass body — no NaN
Body with `mass = 0`. `compute_subtree_momentum()` returns zero (mass * v_com
= 0 regardless of v_com). No NaN from division.

### T9: Sleeping body with offset site
Body is asleep (zero qvel), site offset from origin. Velocimeter reads zero
(cvel is zero when sleeping → shift produces zero). Verify no spurious
non-zero values.

### T10: DISABLE_SENSOR — sensors not evaluated
Model with `DISABLE_SENSOR` flag set. Sensors are not evaluated at all, so
no shift code runs. `sensordata` remains at its pre-step values. No crash.
