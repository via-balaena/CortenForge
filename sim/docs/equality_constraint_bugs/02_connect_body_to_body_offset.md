# Bug 2: Body-to-Body Connect Steady-State Offset

**Status:** FIXED (all fix items implemented and verified)
**Severity:** Was High — 55mm permanent offset in zero gravity

## Symptom

Two free bodies connected body-to-body via a connect constraint converge
to a position with a 55mm offset — even with zero gravity and no external
forces. The offset appears within the first 0.3s and stays permanently.
Angular velocity dampens to near zero.

## Reproduction

```
Diagnostic test D (and D1) in stress-test/src/main.rs

Scene: 2 free bodies, no gravity, nv=12
  body 1 (a): pos=(0, 0, 0), DOFs 0-5
  body 2 (b): pos=(0, 0, -0.5), DOFs 6-11

Constraint:
  connect body1=a body2=b anchor="0 0 -0.5"
  (anchor on body1 at (0,0,-0.5) should match body2 origin)

Initial: qvel[3] = 1.0 (angular kick on body1)

Result:
  t=0.1s: pivot_err=26.80mm, angvel=0.864
  t=0.3s: pivot_err=54.58mm, angvel=0.000
  t=1.3s: pivot_err=55.00mm, angvel=0.000
```

The anchor point on body1 settles 55mm away from body2's origin and stays
there permanently. The system loses all kinetic energy but converges to
the WRONG equilibrium.

## Root Cause: Free Joint Angular Jacobian Uses World-Frame Axes

**File:** `sim/L0/core/src/constraint/equality.rs`, lines 473-491

The function `add_body_point_jacobian_row` constructs the Jacobian for
free joint angular DOFs using **world-frame unit vectors**:

```rust
// CURRENT (wrong):
let ex = Vector3::x();   // world x-axis
let ey = Vector3::y();   // world y-axis
let ez = Vector3::z();   // world z-axis
j[(row, dof_adr + 3)] += sign * direction.dot(&ex.cross(&r));
j[(row, dof_adr + 4)] += sign * direction.dot(&ey.cross(&r));
j[(row, dof_adr + 5)] += sign * direction.dot(&ez.cross(&r));
```

But `qvel[dof_adr+3..dof_adr+6]` for a free joint stores angular
velocity in the **body-LOCAL frame**. This is confirmed by:

1. **`forward/velocity.rs:112-121`**: cvel computation rotates
   `omega_local` to world frame via `data.xquat[body_id] * omega_local`
2. **`mju_quatIntegrate`**: quaternion integration uses left-multiplication
   (`[0, omega_local] * q`), confirming body-frame angular velocity
3. **MuJoCo's `mj_jac`**: uses `d->xmat + 9*body + 3*j` (body rotation
   matrix columns) for free joint angular DOFs
4. **Our own contact Jacobian** (`jacobian.rs:121-128`): already fixed
   in DT-75 to use body rotation columns

The correct Jacobian for body-local angular velocity requires body
rotation matrix columns `R[:,i]`:

```
J[row, dof+i] = direction . (R[:,i] x r)
```

Our code gives `direction . (e_i x r)` which only equals the correct
value when R = I (identity rotation).

### Why This Creates the 55mm Offset

The test starts body1 with angular velocity `qvel[3] = 1.0` (rotation
around the local x-axis). The sequence of events:

1. **Step 0:** Body1 at identity rotation (R = I). Jacobian is correct
   (`R*e_i = e_i`). Constraint applies correct force.

2. **Steps 1-N:** Body1 rotates. R deviates from I. Jacobian angular
   terms become increasingly wrong. The constraint force projects
   incorrectly onto the angular DOFs:
   - The CORRECT force would apply a torque that opposes the rotation
     AND keeps the anchor on-target
   - The WRONG force applies torque in the wrong direction, allowing
     the anchor to drift

3. **Energy dissipation:** The constraint's damping term eventually
   removes all kinetic energy (angular velocity -> 0). But the path
   taken during damping was wrong due to incorrect forces.

4. **Final state:** Body1 and body2 are at rest, but body1 has rotated
   to a non-identity orientation. The anchor is 55mm from body2's origin.
   The constraint generates a force (pos_error != 0), but the WRONG
   Jacobian maps this force to DOFs in a way that doesn't reduce the error.
   Specifically, the force creates torques that rotate body1 AND translate
   it such that the net effect on the anchor position is near-zero.

### Proof: Ball Joint vs Free Joint

The Ball joint case in our code is CORRECT:

```rust
// Ball joint (CORRECT) — equality.rs:463-471
let rot = data.xquat[jnt_body].to_rotation_matrix();
for i in 0..3 {
    let omega_world = rot * Vector3::ith(i, 1.0);
    let j_col = omega_world.cross(&r);
    j[(row, dof_adr + i)] += sign * direction.dot(&j_col);
}
```

The Free joint case is a direct copy-paste error that forgot to apply
the rotation matrix:

```rust
// Free joint (WRONG) — equality.rs:485-490
let ex = Vector3::x();    // should be rot * Vector3::x()
let ey = Vector3::y();    // should be rot * Vector3::y()
let ez = Vector3::z();    // should be rot * Vector3::z()
```

### Why Single-Body Connect Works

Test A (connect body to world, zero anchor, no gravity) works because:
- Anchor = (0,0,0) = body center
- `r = anchor_world - jpos = body_center - body_center = 0`
- Angular Jacobian entries = `direction . (axis x 0) = 0`
- No angular coupling → body-frame vs world-frame distinction is irrelevant

Test B (connect to world, offset geom but zero anchor) works for the
same reason: anchor is at the body ORIGIN, so angular coupling is zero.

The bug only manifests when:
1. Anchor is offset from body center (creating angular coupling), AND
2. The body rotates away from identity orientation (making R != I)

## Fix Spec

### Fix (Critical): Body Rotation Columns in Free Joint Jacobian

**`add_body_point_jacobian_row`**, lines 478-490. Replace:

```rust
let jpos = Vector3::new(
    data.qpos[model.jnt_qpos_adr[jnt_id]],
    data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
    data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
);
let r = point - jpos;
let ex = Vector3::x();
let ey = Vector3::y();
let ez = Vector3::z();
j[(row, dof_adr + 3)] += sign * direction.dot(&ex.cross(&r));
j[(row, dof_adr + 4)] += sign * direction.dot(&ey.cross(&r));
j[(row, dof_adr + 5)] += sign * direction.dot(&ez.cross(&r));
```

With:

```rust
let jpos = Vector3::new(
    data.qpos[model.jnt_qpos_adr[jnt_id]],
    data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
    data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
);
let r = point - jpos;
let rot = data.xquat[jnt_body].to_rotation_matrix();
for i in 0..3 {
    let body_axis = rot * Vector3::ith(i, 1.0);
    j[(row, dof_adr + 3 + i)] += sign * direction.dot(&body_axis.cross(&r));
}
```

This matches the Ball joint pattern at lines 463-471 and the contact
Jacobian at `jacobian.rs:121-128`.

### Supplementary Fix: Missing Body2 Anchor

For general connect constraints, MuJoCo stores a second anchor in
body2's local frame (`eq_data[3..6]`), auto-computed so both anchors
map to the same world point at qpos0. Our code uses body2's origin
instead. For the specific test case this makes no difference (body2's
auto-computed anchor is (0,0,0) = origin), but for correctness the
builder should auto-compute and store body2's anchor:

```rust
// In process_connect():
// Auto-compute body2 anchor: convert body1 anchor to world, then to body2 frame
let (p1, q1) = self.body_world_pose(*body1_id);
let anchor_world = p1 + q1 * connect.anchor;
let (p2, q2) = self.body_world_pose(body2_id);
let anchor_body2 = q2.inverse() * (anchor_world - p2);
data[3] = anchor_body2.x;
data[4] = anchor_body2.y;
data[5] = anchor_body2.z;
```

Then update `extract_connect_jacobian` to use both anchors.

### Supplementary Fix: Velocity from J*qvel

Replace cvel-derived velocity computation with `J * qvel` for consistency
with MuJoCo. This guarantees the velocity used in Baumgarte stabilization
is consistent with the Jacobian.

## Verification

After the primary fix:
- Test D (body-to-body connect, angular kick) should converge to < 1mm error
- Test D1 (double pendulum, no gravity) should converge to < 1mm at both pivots
- Existing single-body connect tests should be unaffected (they already work)

## Files

- `sim/L0/core/src/constraint/equality.rs:473-491` — THE BUG (Free joint angular Jacobian)
- `sim/L0/core/src/constraint/equality.rs:463-471` — Ball joint (correct reference)
- `sim/L0/core/src/constraint/jacobian.rs:116-128` — Contact Jacobian (correct reference, fixed in DT-75)
- `sim/L0/core/src/forward/velocity.rs:107-121` — Free joint cvel (confirms body-local angular velocity)
- `sim/L0/mjcf/src/builder/equality.rs:69-121` — Connect eq_data builder (supplementary fix)
