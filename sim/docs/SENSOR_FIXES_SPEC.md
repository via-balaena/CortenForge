# Sensor Correctness Fixes Specification

**Status:** Draft
**Scope:** `sim/L0/core/src/mujoco_pipeline.rs` sensor evaluation functions
**Baseline commit:** `035d08f` (feat: complete sensor implementation)

---

## Background

The initial sensor implementation (commit `035d08f`) brought 10+ sensor types from
stub to functional. A post-implementation review against MuJoCo's C source
(`engine_sensor.c`, `engine_core_smooth.c`) identified several issues: two
correctness bugs (rangefinder ray direction, cutoff clamping for positive-type
sensors), one latent type-system mismatch (JointPos/JointVel dimension overflow
for Ball/Free joints), one misleading comment (accelerometer sign), and one
code quality inconsistency (missing bounds checks on 1D sensor writes). This spec
describes the fixes with the physics reasoning behind each one.

---

## Fix Order and Dependencies

```
Fix 1: Accelerometer comment + test hardening   (no dependencies)  — Low severity
Fix 2: Rangefinder ray direction                 (no dependencies)  — HIGH: wrong output
Fix 3: JointPos/JointVel Ball/Free + BallQuat     (no dependencies)  — MEDIUM: latent bug
Fix 4: Bounds-check consistency for 1D writes    (no dependencies)  — Low: defensive
```

All four fixes are independent and can be done in any order or in parallel.

**Fix 2** is the only fix that changes numerical output for existing code paths
today (rays shoot backward). **Fix 3** is latent — it will break when MJCF
sensor wiring lands. **Fix 1** is a comment correction and test hardening (the
formula is already correct). **Fix 4** is a code quality refactor.

Recommended implementation order: **2 → 3 → 1 → 4**.

---

## Fix 1: Accelerometer Comment Correction and Test Hardening

### Problem

The accelerometer formula is:

```rust
// mujoco_pipeline.rs:5292
let a_proper = a_world - model.gravity;
```

Where `model.gravity = (0, 0, -9.81)` and `compute_body_acceleration` returns the
body's coordinate acceleration from `qacc`. At rest, `qacc ~ 0`, so:

```
a_proper = 0 - (0, 0, -9.81) = (0, 0, +9.81)
```

This produces `+g` upward — **correct** for a real accelerometer at rest (Einstein
equivalence principle). The formula matches MuJoCo's behavior.

However, the comments are contradictory:

- Line 5276: "accelerometer reads +g (opposing gravity)" — **correct**
- Line 5291: "At rest on ground, reads -g" — **wrong**

The initial review incorrectly flagged the formula as producing the wrong sign.
On closer analysis, `a_world - gravity` with `gravity = (0,0,-9.81)` produces
`(0, 0, +9.81)` which is the correct MuJoCo-matching result.

### MuJoCo Reference

MuJoCo achieves the same result differently. In `mj_rnePostConstraint`
(`engine_core_smooth.c`), the world body gets `cacc[world].linear = -gravity`:

```c
mju_scl3(d->cacc+3, m->opt.gravity, -1);  // (0, 0, +9.81) for default gravity
```

This propagates through the tree. For a body at rest (no joint acceleration),
`cacc.linear = (0, 0, +9.81)`. The accelerometer reads `cacc` directly.

Our approach is algebraically equivalent: we subtract the gravity vector from the
coordinate acceleration. Both produce `+g` at rest.

### Implementation Steps

1. **Fix the incorrect comment** at lines 5290-5291:
   ```rust
   // Before:
   // Accelerometer measures: a_proper = a - g (proper acceleration)
   // In free fall, accelerometer reads 0. At rest on ground, reads -g

   // After:
   // Proper acceleration = a_body - g_field
   // At rest: a_body=0, g_field=(0,0,-9.81), so a_proper=(0,0,+9.81) (upward)
   // In free fall: a_body~g_field, so a_proper~0. Matches real IMU behavior.
   ```

2. **Harden the existing test** `test_accelerometer_at_rest_reads_gravity` to
   verify the **sign** of the Z component, not just the magnitude. The test model
   uses default gravity `(0, 0, -9.81)` and an identity-rotation site, so the
   sensor Z output should be positive:
   ```rust
   // With default gravity (0, 0, -9.81) and axis-aligned site,
   // the Z component of proper acceleration should be positive (+9.81)
   assert!(az > 0.0, "Accelerometer Z should be positive at rest, got {az}");
   assert_relative_eq!(accel_mag, 9.81, epsilon = 2.0);
   ```

3. **Add a free-fall test**: Create a model with a **Free joint** (6 DOF), no
   ground plane, and no contacts. Run `forward()`. The accelerometer should read
   approximately zero because the body accelerates at `g` under gravity alone:
   `a_proper = a_body - g ≈ g - g = 0`.

   This requires a separate model setup from `make_sensor_test_model` (which uses
   a hinge joint). The free-joint model needs `nq=7`, `nv=6`, and the appropriate
   DOF/qpos initialization (identity quaternion for orientation):
   ```rust
   #[test]
   fn test_accelerometer_in_free_fall_reads_zero() {
       // Build a model with a single body on a Free joint (no contacts)
       let mut model = Model::empty();
       // ... add body with Free joint, site, accelerometer sensor ...
       // qpos = [0, 0, 0, 1, 0, 0, 0] (origin, identity quat)
       // No ground, no other geoms
       let mut data = model.make_data();
       data.forward(&model).unwrap();
       let accel_mag = (data.sensordata[0].powi(2)
           + data.sensordata[1].powi(2)
           + data.sensordata[2].powi(2)).sqrt();
       assert!(accel_mag < 1.0, "Free-fall accelerometer should read ~0, got {accel_mag}");
   }
   ```

### Validation

- Existing test upgraded to check sign (no code change needed if sign is already correct).
- New free-fall test verifies the complementary case.
- `cargo test --package sim-core` passes.

---

## Fix 2: Rangefinder Ray Direction

### Problem

Our rangefinder negates the site Z-axis:

```rust
// mujoco_pipeline.rs:4921
let ray_dir = -site_z;  // WRONG: shoots along -Z
```

MuJoCo shoots along **+Z** of the site frame. From the MuJoCo source
(engine_sensor.c):

```c
// MuJoCo source: extract +Z column of site rotation matrix
rvec[0] = d->site_xmat[9*objid+2];
rvec[1] = d->site_xmat[9*objid+5];
rvec[2] = d->site_xmat[9*objid+8];
```

The MuJoCo documentation confirms: "scalar distance to nearest geom or site along
z-axis" — the positive Z-axis. The site's `zaxis` attribute controls the look
direction, and the rangefinder shoots in that direction.

### Root Cause

The initial implementation assumed -Z based on a camera-like convention (cameras
often look along -Z). MuJoCo sites use +Z as the forward/look direction.

### Implementation Steps

1. **Remove the negation** on line 4921:
   ```rust
   // Before:
   let ray_dir = -site_z;

   // After:
   let ray_dir = site_z;
   ```

2. **Update both rangefinder comments** that reference the direction:
   - Line 4910: change "site's negative Z axis" to "site's positive Z axis"
   - Line 4915: change "shoots along -Z of site frame" to "shoots along +Z of site frame"

3. **Update the test** `test_rangefinder_hits_sphere` to place the target body
   along **+Z** of the site, not -Z. Currently `body_pos` is `(0, 0, -2.0)` to
   match the negated direction. Change it to `(0, 0, 1.0)`:
   ```rust
   model.body_pos.push(Vector3::new(0.0, 0.0, 1.0));
   ```
   Updated geometry: site is at world (0, 0, -0.5), ray shoots +Z. Sphere center
   at (0, 0, 1.0) with radius 0.5, bottom surface at z = 0.5. Distance from
   site to hit = 0.5 - (-0.5) = 1.0. This preserves the original ~1.0 expected
   distance. Update the test comment accordingly.

4. **Add a negative-direction test** to confirm that objects behind the sensor
   (along -Z) are NOT detected:
   ```rust
   #[test]
   fn test_rangefinder_ignores_objects_behind_sensor() {
       // Place sphere at (0, 0, -3), sensor looks along +Z
       // Should return -1.0 (no hit)
   }
   ```

5. **Fix cutoff interaction with rangefinder's -1 sentinel.** The current
   `mj_sensor_postprocess` clamps all sensor values to `[-cutoff, cutoff]`. But
   MuJoCo uses `mjDATATYPE_POSITIVE` for rangefinder sensors, which means cutoff
   only applies on the positive side: `min(cutoff, value)`. The -1 no-hit sentinel
   is preserved because `min(cutoff, -1) = -1`.

   Our code clamps to `[-cutoff, cutoff]`, which turns -1.0 into `-cutoff` when
   `cutoff < 1.0`. This corrupts the no-hit sentinel.

   The correct fix is to handle positive-type sensors in postprocess. Since we
   don't yet have a value-type enum (`MjDataType` with Real/Positive/Axis/Quaternion),
   the simplest correct approach is to check the sensor type:

   ```rust
   // In mj_sensor_postprocess:
   let cutoff = model.sensor_cutoff[sensor_id];
   if cutoff > 0.0 {
       let sensor_type = model.sensor_type[sensor_id];
       for i in 0..dim {
           let idx = adr + i;
           if idx < data.sensordata.len() {
               match sensor_type {
                   // Positive-type sensors: only clamp on positive side
                   MjSensorType::Touch | MjSensorType::Rangefinder => {
                       data.sensordata[idx] = data.sensordata[idx].min(cutoff);
                   }
                   // Real-type sensors: clamp both sides
                   _ => {
                       data.sensordata[idx] = data.sensordata[idx].clamp(-cutoff, cutoff);
                   }
               }
           }
       }
   }
   ```

   This matches MuJoCo's behavior: Touch and Rangefinder are `mjDATATYPE_POSITIVE`
   (0 or negative means inactive), so cutoff only limits the positive range.

6. **Update the `mj_sensor_postprocess` docstring** (lines 5467-5468). The current
   doc comment claims "For rangefinder, negative values (-1) indicate no hit and
   are preserved" — but the implementation does NOT preserve them (it clamps to
   `[-cutoff, cutoff]`). After applying step 5, replace lines 5467-5468 with:
   ```rust
   /// - Cutoff: For most sensors, clamps to [-cutoff, cutoff] (0 = no cutoff).
   ///   For positive-type sensors (Touch, Rangefinder), clamps positive side only:
   ///   min(value, cutoff). This preserves rangefinder's -1.0 no-hit sentinel.
   ```

7. **Add a test** for cutoff with rangefinder no-hit:
   ```rust
   #[test]
   fn test_rangefinder_cutoff_preserves_no_hit_sentinel() {
       // Set cutoff = 0.5, no geoms to hit
       // Rangefinder returns -1.0 (no hit)
       // After postprocess, value should still be -1.0, not -0.5
   }
   ```

### Validation

- `test_rangefinder_hits_sphere` passes with sphere at +Z.
- `test_rangefinder_ignores_objects_behind_sensor` confirms no false positives.
- `test_rangefinder_cutoff_preserves_no_hit_sentinel` confirms -1.0 is not clamped.
- `test_rangefinder_no_geoms_to_hit` unchanged (no targets = -1.0).
- `cargo test --package sim-core` passes.

---

## Fix 3: Remove Ball/Free Handling from JointPos/JointVel, Add BallQuat/BallAngVel Types

### Problem

**JointPos**: `dim() = 1` but the evaluation code at lines 4805-4820 writes 4
elements for Ball joints and 7 for Free joints. The bounds checks at lines
4808/4816 prevent a panic, but only 1 of the 4/7 values lands in the allocated
slot. The rest silently overwrite neighboring sensor data or are silently dropped.

**JointVel**: Same problem. `dim() = 1` but the evaluation code at lines
5064-5074 uses `jnt_type.nv()` to determine write count — 3 for Ball, 6 for Free.
The bounds check at line 5070 prevents a panic, but writes 3 or 6 elements into a
1-slot allocation, overwriting neighboring sensor data.

In MuJoCo, both `mjSENS_JOINTPOS` and `mjSENS_JOINTVEL` are defined as "scalar
(hinge and slide only)." Ball joints use separate sensor types:
- `mjSENS_BALLQUAT` (dim=4) for orientation
- `mjSENS_BALLANGVEL` (dim=3) for angular velocity
- Free joints use `mjSENS_FRAMEPOS` + `mjSENS_FRAMEQUAT` for position/orientation
  and `mjSENS_FRAMELINVEL` + `mjSENS_FRAMEANGVEL` for velocity.

Our MJCF parser (`sim/L0/mjcf/src/types.rs`) already defines `Ballquat` (dim=4)
and `Ballangvel` (dim=3) as `MjcfSensorType` variants, but the core pipeline
`MjSensorType` enum lacks them.

### Implementation Steps

1. **Remove the Ball and Free branches** from the `JointPos` match arm in
   `mj_sensor_pos()`. Replace with a comment explaining these joint types use
   different sensor types:

   ```rust
   MjSensorType::JointPos => {
       // Scalar joint position (hinge/slide only).
       // Ball joints use BallQuat, free joints use FramePos + FrameQuat.
       if objid < model.njnt {
           let qpos_adr = model.jnt_qpos_adr[objid];
           match model.jnt_type[objid] {
               MjJointType::Hinge | MjJointType::Slide => {
                   data.sensordata[adr] = data.qpos[qpos_adr];
               }
               _ => {} // Ball/Free not supported by JointPos; use BallQuat/FramePos
           }
       }
   }
   ```

2. **Restrict `JointVel` to hinge/slide only** in `mj_sensor_vel()`. Currently
   it uses `jnt_type.nv()` which returns 3 for Ball and 6 for Free. Replace the
   `nv`-based loop with an explicit hinge/slide check:

   ```rust
   MjSensorType::JointVel => {
       // Scalar joint velocity (hinge/slide only).
       // Ball joints use BallAngVel, free joints use FrameLinVel + FrameAngVel.
       if objid < model.njnt {
           let dof_adr = model.jnt_dof_adr[objid];
           match model.jnt_type[objid] {
               MjJointType::Hinge | MjJointType::Slide => {
                   data.sensordata[adr] = data.qvel[dof_adr];
               }
               _ => {} // Ball/Free not supported by JointVel; use BallAngVel/FrameLinVel
           }
       }
   }
   ```

3. **Add `BallQuat` and `BallAngVel` to `MjSensorType`** enum:

   ```rust
   /// Ball joint orientation quaternion (4D). MuJoCo: mjSENS_BALLQUAT.
   BallQuat,
   /// Ball joint angular velocity (3D). MuJoCo: mjSENS_BALLANGVEL.
   BallAngVel,
   ```

4. **Add dimensions** to `dim()`:
   ```rust
   Self::BallQuat => 4,
   Self::BallAngVel => 3,
   ```

   **Important — sensor datatype assignment:** The pipeline filters sensors by
   `model.sensor_datatype` with a hard `continue` at the top of each evaluation
   function (lines 4789, 5056, 5265). A sensor with the wrong datatype will be
   silently skipped and produce zeros. When adding BallQuat/BallAngVel sensors:
   - `BallQuat` must use `MjSensorDataType::Position` (evaluated in `mj_sensor_pos`)
   - `BallAngVel` must use `MjSensorDataType::Velocity` (evaluated in `mj_sensor_vel`)

   This applies to test code (via `add_sensor()`'s datatype parameter) and will
   apply to MJCF→core sensor wiring when that lands.

5. **Implement `BallQuat` in `mj_sensor_pos()`**:
   ```rust
   MjSensorType::BallQuat => {
       // Ball joint quaternion [w, x, y, z]
       if objid < model.njnt
           && model.jnt_type[objid] == MjJointType::Ball
       {
           let qpos_adr = model.jnt_qpos_adr[objid];
           if adr + 3 < data.sensordata.len() {
               for i in 0..4 {
                   data.sensordata[adr + i] = data.qpos[qpos_adr + i];
               }
               // Normalize quaternion (MuJoCo does mju_normalize4).
               // Use 1e-10 threshold and identity reset to match our
               // normalize_quaternion() convention (line 8980).
               let norm = (0..4)
                   .map(|i| data.sensordata[adr + i].powi(2))
                   .sum::<f64>()
                   .sqrt();
               if norm > 1e-10 {
                   for i in 0..4 {
                       data.sensordata[adr + i] /= norm;
                   }
               } else {
                   // Degenerate — reset to identity [w=1, x=0, y=0, z=0]
                   data.sensordata[adr] = 1.0;
                   data.sensordata[adr + 1] = 0.0;
                   data.sensordata[adr + 2] = 0.0;
                   data.sensordata[adr + 3] = 0.0;
               }
           }
       }
   }
   ```

6. **Implement `BallAngVel` in `mj_sensor_vel()`**:
   ```rust
   MjSensorType::BallAngVel => {
       // Ball joint angular velocity [wx, wy, wz] in local (child body) frame
       if objid < model.njnt
           && model.jnt_type[objid] == MjJointType::Ball
       {
           let dof_adr = model.jnt_dof_adr[objid];
           if adr + 2 < data.sensordata.len() {
               data.sensordata[adr] = data.qvel[dof_adr];
               data.sensordata[adr + 1] = data.qvel[dof_adr + 1];
               data.sensordata[adr + 2] = data.qvel[dof_adr + 2];
           }
       }
   }
   ```

7. **Add tests**:
   - `test_ball_quat_sensor_reads_quaternion`: Create a model with a Ball joint,
     set `qpos` to a known quaternion, verify 4 output elements match.
   - `test_ball_quat_sensor_normalizes`: Set a non-unit quaternion, verify output
     is normalized. Also test the degenerate case: set a near-zero quaternion
     `(0, 0, 0, 1e-15)` and verify output is identity `(1, 0, 0, 0)`.
   - `test_ball_angvel_sensor`: Set `qvel` for a Ball joint, verify 3D output.
   - `test_jointpos_ignores_ball_joint`: Attach JointPos to a Ball joint, verify
     it writes nothing (no corruption of adjacent data).
   - `test_jointvel_ignores_ball_joint`: Attach JointVel to a Ball joint, verify
     it writes nothing (no corruption of adjacent data).

### Validation

- All new tests pass.
- Existing JointPos tests for hinge joints are unaffected.
- `cargo test --package sim-core` passes.
- `cargo clippy --package sim-core --all-targets` clean.

---

## Fix 4: Bounds-Check Consistency for 1D Sensor Writes

### Problem

3D sensor writes consistently guard with `if adr + 2 < data.sensordata.len()`.
4D sensor writes guard with `if adr + 3 < data.sensordata.len()`. But 1D sensor
writes omit the check entirely:

| Location | Sensor | Write | Guard |
|----------|--------|-------|-------|
| Line 5379 | Touch | `sensordata[adr] = total_force` | None |
| Line 5017 | ActuatorPos | `sensordata[adr] = gear * qpos` | None |
| Line 5022 | ActuatorPos (tendon) | `sensordata[adr] = 0.0` | None |
| Line 5032 | TendonPos | `sensordata[adr] = 0.0` | None |
| Line 5407 | ActuatorFrc | `sensordata[adr] = qfrc_actuator` | None |
| Line 4926 | Rangefinder | `sensordata[adr] = -1.0` | None |
| Line 5000 | Rangefinder | `sensordata[adr] = closest_dist` | None |
| Line 5003 | Rangefinder | `sensordata[adr] = -1.0` | None |
| Line 4803 | JointPos (hinge) | `sensordata[adr] = qpos` | None |
| Line 5071 | JointVel | `sensordata[adr] = qvel` | None |
| Line 5228 | ActuatorVel | `sensordata[adr] = gear * qvel` | None |
| Line 5243 | TendonVel | `sensordata[adr] = 0.0` | None |

In normal operation, `sensordata` is allocated to exactly `nsensordata` slots in
`make_data()`, and `adr` is computed from the same allocation, so these never
overflow. But a malformed `Model` (wrong `nsensordata`, corrupted `sensor_adr`)
would cause a panic instead of silent corruption.

### Root Cause

The 1D sensors were written without the defensive guard because overflow seemed
impossible with correct allocation. The 3D sensors were written with guards because
out-of-bounds on a multi-element write is more obvious. The inconsistency is a
code quality issue, not a correctness bug in normal operation.

### Implementation Steps

Rather than adding `if adr < data.sensordata.len()` to every individual write
site (which bloats the code), introduce a **validated write helper** and use it
uniformly:

1. **Add helper functions** — one base writer and typed variants:
   ```rust
   /// Write a single sensor value with bounds checking.
   #[inline(always)]
   fn sensor_write(sensordata: &mut DVector<f64>, adr: usize, offset: usize, value: f64) {
       let idx = adr + offset;
       if idx < sensordata.len() {
           sensordata[idx] = value;
       }
   }

   /// Write a 3D vector to sensor data with bounds checking.
   #[inline(always)]
   fn sensor_write3(sensordata: &mut DVector<f64>, adr: usize, v: &Vector3<f64>) {
       sensor_write(sensordata, adr, 0, v.x);
       sensor_write(sensordata, adr, 1, v.y);
       sensor_write(sensordata, adr, 2, v.z);
   }

   /// Write a quaternion (w, x, y, z) to sensor data with bounds checking.
   #[inline(always)]
   fn sensor_write4(sensordata: &mut DVector<f64>, adr: usize, w: f64, x: f64, y: f64, z: f64) {
       sensor_write(sensordata, adr, 0, w);
       sensor_write(sensordata, adr, 1, x);
       sensor_write(sensordata, adr, 2, y);
       sensor_write(sensordata, adr, 3, z);
   }
   ```

2. **Migrate all sensor evaluation functions** (`mj_sensor_pos`, `mj_sensor_vel`,
   `mj_sensor_acc`, and the BallQuat/BallAngVel code from Fix 3) to use these
   helpers. This is a mechanical refactor — no behavioral change.

   Examples of the replacement pattern:
   ```rust
   // 1D: direct write → helper
   data.sensordata[adr] = total_force;
   // becomes:
   sensor_write(&mut data.sensordata, adr, 0, total_force);

   // 3D: guarded block → helper
   if adr + 2 < data.sensordata.len() {
       data.sensordata[adr] = v.x;
       data.sensordata[adr + 1] = v.y;
       data.sensordata[adr + 2] = v.z;
   }
   // becomes:
   sensor_write3(&mut data.sensordata, adr, &v);

   // 4D: guarded block → helper
   if adr + 3 < data.sensordata.len() {
       data.sensordata[adr] = quat.w;
       // ...
   }
   // becomes:
   sensor_write4(&mut data.sensordata, adr, quat.w, quat.i, quat.j, quat.k);
   ```

   Note on BallQuat normalization (Fix 3) and borrow checker: After
   writing the 4 quaternion elements with `sensor_write`, the read-back
   for norm computation is on a separate statement, so no borrow conflict.
   However, when writing the normalized values back, the read and write
   of `data.sensordata` must NOT occur in the same expression — extract
   the read into a local first:
   ```rust
   // WRONG: borrow conflict (mutable + immutable in same expression)
   sensor_write(&mut data.sensordata, adr, i, data.sensordata[adr + i] / norm);
   // RIGHT: read completes before mutable borrow begins
   let normalized = data.sensordata[adr + i] / norm;
   sensor_write(&mut data.sensordata, adr, i, normalized);
   ```

3. **Add one test** that verifies a malformed model with undersized `sensordata`
   does not panic:
   ```rust
   #[test]
   fn test_sensor_write_with_undersized_buffer_does_not_panic() {
       let mut model = make_sensor_test_model();
       add_sensor(&mut model, MjSensorType::Touch, ...);
       // Manually shrink sensordata
       let mut data = model.make_data();
       data.sensordata = DVector::zeros(0); // empty buffer
       // Should not panic
       data.forward(&model).unwrap();
   }
   ```

### Validation

- All existing tests pass (behavior unchanged for correctly-sized buffers).
- New robustness test passes.
- `cargo clippy` clean.

---

## Non-Functional Improvements (Deferred)

The following items were identified in the review but are **not correctness issues**.
They are documented here for future reference and should NOT be addressed in this
change.

### O(n^2) Subtree Iteration

`compute_site_force_torque()`, `compute_subtree_com()`, and
`compute_subtree_angmom()` each independently iterate `root_body..nbody` with
parent-chain walks. For large models (100+ bodies) with multiple Force/Torque
sensors, this becomes expensive.

**Future fix**: Extract a shared `collect_subtree_body_ids(model, root) -> Vec<usize>`
using BFS/DFS. Cache per-step if sensor topology is static.

### Redundant Force/Torque Computation

When both Force and Torque sensors reference the same site,
`compute_site_force_torque()` runs twice. The function returns `(force, torque)` —
each sensor discards the component it does not need.

**Future fix**: Cache `(force, torque)` results keyed by `(body_id, site_pos)`.
Only matters when both sensor types are attached to the same site.

### Magnetometer in Wrong Evaluation Stage

The magnetometer is evaluated in `mj_sensor_acc` (acceleration stage), but MuJoCo
evaluates it in `mj_sensorPos` (position stage) because it only depends on
`site_xmat` (computed during FK). Numerically this doesn't matter — `site_xmat`
is the same in all stages. But it means the magnetometer won't produce output if
a caller only runs the position stage.

**Future fix**: Move magnetometer from `mj_sensor_acc` to `mj_sensor_pos`. Add
the `MjSensorDataType::Position` tag for magnetometer sensors.

### Unused Loop Variable in Touch Sensor

Line 5375: `let _ = i;` suppresses unused variable warning for the contact loop
index.

**Future fix**: Change `for (i, contact) in ...` to `for contact in data.contacts.iter()`.

---

## Test Plan

After all four fixes are applied, run:

```bash
cargo test --package sim-core
cargo test --package sim-conformance-tests
cargo clippy --workspace --all-targets
```

All tests must pass with zero warnings (excluding pedantic-only lints in test
modules).

### New Tests Summary

| Fix | New Tests | Modified Tests |
|-----|-----------|----------------|
| 1 | Free-fall accelerometer (~0) | `test_accelerometer_at_rest_reads_gravity` (add sign check) |
| 2 | Rangefinder -Z no-hit, cutoff-preserves-sentinel | `test_rangefinder_hits_sphere` (move sphere from -Z to +Z) |
| 3 | BallQuat read, BallQuat normalize, BallAngVel read, JointPos-ignores-Ball, JointVel-ignores-Ball | None |
| 4 | Undersized buffer no-panic | None |

### Regression

All 23 existing sensor tests must continue to pass.

---

## Acceptance Criteria

1. Accelerometer comments accurately describe the formula. Tests verify both
   sign (+g at rest) and magnitude (~0 in free fall).
2. Rangefinder shoots along +Z of site frame (matching MuJoCo source).
   Cutoff postprocessing uses positive-only clamping for rangefinder and touch
   sensors, preserving the -1.0 no-hit sentinel.
3. `JointPos` and `JointVel` sensors are scalar-only (hinge/slide). `BallQuat`
   (4D) and `BallAngVel` (3D) handle ball joints as distinct sensor types.
4. All sensor writes are bounds-checked via uniform helper functions.
5. Full test suite passes. Clippy clean.
