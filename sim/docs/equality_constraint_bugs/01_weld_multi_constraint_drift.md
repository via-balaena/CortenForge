# Bug 1: Weld-to-World Drift in Multi-Constraint Scenes

**Status:** Root cause confirmed
**Severity:** High — body drifts 138mm instead of 0.37mm

## Symptom

A weld-to-world constraint holds a body perfectly in isolation (0.37mm
steady-state sag under gravity). But when the body is offset from the
world origin — or when a second weld constraint exists — the weld-to-
world body drifts 138mm+ over 5 seconds.

## Reproduction

```
Diagnostic test C2 in stress-test/src/main.rs

Scene: 1 free body at (0.5, 0, 1), nv=6
  weld body1=b (to world)

Result after 5s:
  displacement: 137.98mm (expected ~0.37mm)
  angular velocity: growing (body is rotating)

Also reproduced in D2 (multi-constraint scene):
  3 free bodies, nv=18
  weld body1=base body2=arm (pair) + weld body1=fixed (to world)
  fixed body displacement: 137.98mm (identical to C2)
```

## Root Cause: Two Independent Bugs

### Primary: Free Joint Angular Jacobian Uses Wrong Frame (shared with Bug 2)

**File:** `sim/L0/core/src/constraint/equality.rs`, lines 473-491

The `add_body_point_jacobian_row` function builds the angular DOF
columns of the Jacobian using **world-frame unit vectors** (`ex, ey, ez`):

```rust
// CURRENT (wrong):
let ex = Vector3::x();
let ey = Vector3::y();
let ez = Vector3::z();
j[(row, dof_adr + 3)] += sign * direction.dot(&ex.cross(&r));
j[(row, dof_adr + 4)] += sign * direction.dot(&ey.cross(&r));
j[(row, dof_adr + 5)] += sign * direction.dot(&ez.cross(&r));
```

But free joint angular velocity in `qvel` is in **body-LOCAL frame**
(confirmed by `forward/velocity.rs:112-121` which rotates `omega_local`
to world frame via `data.xquat[body_id] * omega_local`). The Jacobian
must use body rotation matrix columns (`R * e_i`), not world-frame
unit vectors.

**The contact Jacobian already has the correct pattern** (fixed in DT-75):

```rust
// Contact Jacobian (CORRECT) — jacobian.rs:121-128
let rot = data.xquat[jnt_body].to_rotation_matrix();
for i in 0..3 {
    let omega = rot * Vector3::ith(i, 1.0);
    j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega.cross(&r));
}
```

The same bug exists in `add_body_angular_jacobian_row` (lines 538-542):

```rust
// CURRENT (wrong):
j[(row, dof_adr + 3)] += sign * direction.x;
j[(row, dof_adr + 4)] += sign * direction.y;
j[(row, dof_adr + 5)] += sign * direction.z;
```

**Why this causes drift:** At identity rotation (R=I), `R*e_i = e_i`, so
the Jacobian is correct. When the body is offset from the constraint
point, gravity creates angular coupling that causes a tiny rotation. Once
the body rotates (R != I), the Jacobian angular terms become wrong. The
wrong Jacobian can't correctly damp the rotation, so it grows. This is
the positive feedback loop observed: sag -> angular coupling -> rotation
-> wrong Jacobian -> can't damp -> more rotation -> more sag.

A body on the z-axis (0, 0, 1) doesn't drift because the z-constraint
has zero angular coupling (`ez x (0,0,-1) = 0`), so gravity never
starts the rotation in the first place.

### Secondary: Weld eq_data Layout and Formulation Differ from MuJoCo

**File:** `sim/L0/core/src/constraint/equality.rs`, line 130

Our weld formulation combines `anchor + relpose_pos` into a single
offset on body1:

```rust
// CURRENT:
let offset = anchor + relpose_pos;          // combined on body1
let cpos = p1 + r1 * offset;               // constraint point on body1
let pos_error = cpos - p2;                  // error vs body2 origin
```

MuJoCo uses **separate anchor points for each body**:

```c
// MuJoCo engine_core_constraint.c:
// data[0..3] = anchor in body2's local frame (user-specified)
// data[3..6] = anchor in body1's local frame (auto-computed)
mjtNum* anchor = data + 3*(1-j);   // SWAPPED per body
pos[j] = xpos[body_j] + R_j * anchor;
cpos = pos[0] - pos[1];
```

Our `eq_data` layout:
- `[0..3]` = anchor, `[3..7]` = relpose_quat, `[7..10]` = relpose_pos

MuJoCo's `eq_data` layout:
- `[0..3]` = anchor in body2 frame, `[3..6]` = anchor in body1 frame (auto),
  `[6..10]` = relpose quat (`neg(q1)*q2`), `[10]` = torquescale

### Tertiary: Weld Rotational Jacobian Uses Simple 0.5 Scaling

**File:** `sim/L0/core/src/constraint/equality.rs`, lines 157-161

```rust
// CURRENT: simple scalar scaling
add_body_angular_jacobian_row(&mut j, 3 + row, &direction, body1, 0.5, ...);
add_body_angular_jacobian_row(&mut j, 3 + row, &direction, body2, -0.5, ...);
```

MuJoCo applies a **per-column quaternion correction**:

```c
// MuJoCo: correct rotation Jacobian
// 0.5 * neg(q1) * (jac0-jac1)_col * q0 * relpose
mju_mulQuatAxis(quat2, quat1, axis);
mju_mulQuat(quat3, quat2, quat);
jac[col] = 0.5 * quat3[1..3];
```

The simple 0.5 scaling is only correct when `neg(q1)*q0*relpose = identity`
(i.e., at the reference configuration). As bodies deviate from the reference
pose, MuJoCo's correction rotates the angular Jacobian columns to remain
consistent with the quaternion-based orientation error formula.

## Feedback Loop (Mechanism)

1. Gravity → translational sag → position error
2. Position error + angular coupling in Jacobian → rotational torque on body
3. Rotational torque → body rotates slightly (R deviates from identity)
4. **Bug trigger:** Jacobian angular terms are now wrong (using `e_i` instead of `R*e_i`)
5. Wrong Jacobian → constraint force projects incorrectly onto angular DOFs
6. Incorrect damping → rotation grows instead of being corrected
7. Rotation → constraint point `cpos` moves → larger position error
8. Positive feedback → progressive drift (20mm at 0.1s, 93mm at 1s, 138mm at 5s)

## Fix Spec

### Fix A (Critical): Free Joint Angular Jacobian — `equality.rs`

**`add_body_point_jacobian_row`**, lines 478-490. Replace:

```rust
let ex = Vector3::x();
let ey = Vector3::y();
let ez = Vector3::z();
j[(row, dof_adr + 3)] += sign * direction.dot(&ex.cross(&r));
j[(row, dof_adr + 4)] += sign * direction.dot(&ey.cross(&r));
j[(row, dof_adr + 5)] += sign * direction.dot(&ez.cross(&r));
```

With (matching the contact Jacobian pattern from `jacobian.rs:121-128`):

```rust
let rot = data.xquat[jnt_body].to_rotation_matrix();
for i in 0..3 {
    let body_axis = rot * Vector3::ith(i, 1.0);
    j[(row, dof_adr + 3 + i)] += sign * direction.dot(&body_axis.cross(&r));
}
```

**`add_body_angular_jacobian_row`**, lines 538-542. Replace:

```rust
j[(row, dof_adr + 3)] += sign * direction.x;
j[(row, dof_adr + 4)] += sign * direction.y;
j[(row, dof_adr + 5)] += sign * direction.z;
```

With:

```rust
let rot = data.xquat[jnt_body].to_rotation_matrix();
for i in 0..3 {
    let body_axis = rot * Vector3::ith(i, 1.0);
    j[(row, dof_adr + 3 + i)] += sign * direction.dot(&body_axis);
}
```

### Fix B (Important): Weld eq_data Layout and Formulation

Refactor `extract_weld_jacobian` and the MJCF builder to match MuJoCo's
separate-anchor formulation:

1. **Builder** (`sim/L0/mjcf/src/builder/equality.rs`):
   - `eq_data[0..3]` = anchor in body2's local frame (user-specified `anchor`
     attribute, per MuJoCo convention)
   - `eq_data[3..6]` = anchor in body1's local frame (auto-computed:
     convert body2 anchor to world, then to body1's frame)
   - `eq_data[6..10]` = relpose quaternion `neg(q1)*q2` (same as current
     `eq_data[3..7]`)
   - `eq_data[10]` = torquescale (default 1.0)

2. **Extraction** (`equality.rs:107`):
   ```rust
   let anchor_body2 = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);
   let anchor_body1 = Vector3::new(eq_data[3], eq_data[4], eq_data[5]);
   let relpose_quat = ... eq_data[6..10] ...;

   let cpos1 = p1 + r1 * anchor_body1;   // point on body1
   let cpos2 = p2 + r2 * anchor_body2;   // point on body2
   let pos_error = cpos1 - cpos2;

   // Jacobian body1 at cpos1, body2 at cpos2
   add_body_point_jacobian_row(..., body1, cpos1, 1.0, ...);
   add_body_point_jacobian_row(..., body2, cpos2, -1.0, ...);
   ```

### Fix C (Recommended): Quaternion-Corrected Rotational Jacobian

Replace the simple `0.5 * (J_body1 - J_body2)` angular Jacobian with
MuJoCo's per-column quaternion correction. This ensures accuracy when
bodies are far from their reference pose.

### Fix D (Recommended): Velocity from J*qvel

Replace cvel-derived velocity with `J * qvel` computation, matching
MuJoCo's `mj_referenceConstraint`. This guarantees consistency between
the Jacobian and the velocity used in Baumgarte stabilization.

## Verification

After Fix A alone: test C2 (offset weld) should show sub-1mm sag
(matching on-axis behavior).

After Fix A+B: all weld stress tests should pass with < 10mm error.

After Fix A+B+C: weld orientation tracking should be accurate even
for large deviations from reference pose.

## Files

- `sim/L0/core/src/constraint/equality.rs:473-491` — `add_body_point_jacobian_row` Free joint (Fix A)
- `sim/L0/core/src/constraint/equality.rs:538-542` — `add_body_angular_jacobian_row` Free joint (Fix A)
- `sim/L0/core/src/constraint/equality.rs:107-202` — `extract_weld_jacobian` (Fix B, C, D)
- `sim/L0/core/src/constraint/jacobian.rs:116-128` — Contact Jacobian (reference for correct pattern)
- `sim/L0/core/src/forward/velocity.rs:107-121` — Free joint cvel computation (confirms body-local angular velocity)
- `sim/L0/mjcf/src/builder/equality.rs:131-201` — Weld eq_data builder (Fix B)
