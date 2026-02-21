# Future Work 10 — Phase 3A: Constraint/Joint Features + Physics + Pipeline + Trait Architecture (Items #38–42, §42A-i–v, §42B–F)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Items #38–42B are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers constraint/joint features (ball joint limits, tendon wrapping), physics
(fluid forces), pipeline flags, deformable body MJCF parsing, and the trait
architecture (§42C–F) described in [TRAIT_ARCHITECTURE.md](../TRAIT_ARCHITECTURE.md).

§42B–F form the trait architecture rollout: §42B (Phase A) proves the pattern with
`FlexBendingModel`, then §42C/D/E extend it to elasticity, actuators, and contact
solvers, and §42F assembles them into the composable `SimBuilder`. §42C–F are NOT
prerequisites to #45 — they add non-MuJoCo extensions and can proceed in parallel
with Batches 5–9 once §42B lands.

---

### 38. Ball Joint Limits (Rotation Cone via Quaternion Logarithm)
**Status:** ✅ Complete | **Effort:** M | **Prerequisites:** None

#### Current State

Limit enforcement in `assemble_unified_constraints()` silently skips ball joints
with `_ => {}` in both the counting loop and assembly loop. Hinge/slide limits work
correctly — single-DOF scalar constraints with `J = ±1.0`. Ball joints with
`limited="true"` get no limit enforcement, producing incorrect behavior for any
model that relies on ball joint rotation limits.

**Note on free joints:** MuJoCo does **not** support limits on free joints.
`mj_instantiateLimit()` in `engine_core_constraint.c` only handles `mjJNT_SLIDE`,
`mjJNT_HINGE`, and `mjJNT_BALL` — `mjJNT_FREE` is completely absent. This item
implements ball joint limits only, matching MuJoCo behavior. Any `limited="true"`
on a free joint should be silently ignored (no constraint rows), matching MuJoCo.

#### MuJoCo Reference

MuJoCo implements ball joint limits in `mj_instantiateLimit()` in
`engine_core_constraint.c`. **There is no swing-twist decomposition.** MuJoCo uses
a single rotation-angle cone limit via the quaternion logarithm map.

##### Step 1: Quaternion logarithm (`mju_quat2Vel`)

Source: `engine_util_spatial.c`. Called with `dt=1` to produce the axis-angle vector:

```c
void mju_quat2Vel(mjtNum res[3], const mjtNum quat[4], mjtNum dt) {
    mjtNum axis[3] = {quat[1], quat[2], quat[3]};
    mjtNum sin_a_2 = mju_normalize3(axis);     // returns |v|, normalizes in-place
    mjtNum speed = 2 * mju_atan2(sin_a_2, quat[0]);  // rotation angle
    if (speed > mjPI) {
        speed -= 2*mjPI;                       // wrap to (-π, π]
    }
    speed /= dt;
    mji_scl3(res, axis, speed);                // res = (speed/dt) * axis
}
```

When called with `dt=1`: `res = theta * axis` (the axis-angle vector), where `theta`
is the signed rotation angle in `(-π, π]` and `axis` is the unit rotation axis.

Key properties:
- `sin_a_2 = |(q.x, q.y, q.z)|` is always ≥ 0, so `atan2` returns `[0, π]` and
  `theta = 2 * atan2(...)` is in `[0, 2π]` before wrapping.
- After wrapping: `theta ∈ (-π, π]`. Negative theta occurs for rotations > 180°.
- The output `angleAxis = theta * axis` encodes both magnitude AND sign.

##### Step 2: Extract rotation magnitude

```c
value = mju_normalize3(angleAxis);   // returns |angleAxis| = |theta|, normalizes in-place
```

After this: `value = |theta| ≥ 0` (the unsigned rotation angle), and `angleAxis`
is now `sign(theta) * axis` — a unit vector that points along the rotation axis
when `theta > 0`, and opposite when `theta < 0`.

##### Step 3: Compute limit distance

```c
dist = mju_max(m->jnt_range[2*i], m->jnt_range[2*i+1]) - value;
```

`max(range[0], range[1])` is the cone half-angle. MuJoCo convention: `range[0] = 0`.
`dist > 0` = within limits, `dist < 0` = violated.

**Degenerate range:** When `max(range[0], range[1]) = 0` (e.g., `range="0 0"`),
`dist = 0 - angle = -angle ≤ 0` for any rotation. This effectively locks the ball
joint rotationally — every non-identity orientation violates the limit. This matches
MuJoCo behavior and is correct (the solver will push the joint back toward identity).

##### Step 4: Activation

Constraint activated when `dist < margin` (where `margin = jnt_margin[i]`).

##### Step 5: Jacobian

```c
mju_scl3(jac, angleAxis, -1);   // J = -angleAxis (after normalize3)
```

The Jacobian is `-sign(theta) * axis` — a 3-element vector placed at the joint's
3 angular DOF addresses. This is `∂dist/∂omega`:
- When `theta > 0`: `∂|theta|/∂omega = +axis`, so `J = -axis`.
- When `theta < 0`: `∂|theta|/∂omega = -axis`, so `J = +axis`.
- In both cases: `J = -sign(theta) * axis`, which MuJoCo computes naturally via
  `normalize3` on `theta * axis`.

##### Step 6: Single constraint row

One constraint row per ball joint (unlike hinge/slide which can have 2 rows for
lower + upper limits). The cone limit is inherently one-sided — `|theta| ≥ 0`
always, so only the upper bound matters.

| Aspect | Hinge/Slide | Ball |
|--------|-------------|------|
| Range meaning | `[lower, upper]` on scalar position | `max(r0, r1)` = cone half-angle |
| Constraint rows | Up to 2 (lower + upper) | 1 (cone) |
| Jacobian entries | 1 non-zero (`±1.0`) | 3 non-zero (`-sign(θ) * axis`) |
| State space | nq=1, nv=1 | nq=4 (quaternion), nv=3 |

#### Objective

Implement MuJoCo-conformant rotation cone limits for ball joints using the
quaternion logarithm map. Ensure free joints with `limited="true"` are silently
ignored (no constraint rows), matching MuJoCo.

#### Specification

##### S1. Helpers

###### `normalize_quat4` — quaternion normalization

Inline quaternion normalization matching the existing codebase pattern
(`QuaternionNormalizeVisitor::normalize_quaternion()`). Uses the same `1e-10`
guard threshold, defaulting to identity for degenerate quaternions.

**Threshold note:** MuJoCo's `mju_normalize4` uses `mjMINVAL` (`1e-15` for f64).
We use `1e-10` to match our existing `normalize_quaternion()` convention. The
difference is inconsequential — both thresholds are far below any physically
meaningful quaternion norm, and the fallback (identity) is identical.

**Why not reuse `QuaternionNormalizeVisitor::normalize_quaternion()`?** That
method operates on `&mut DVector<f64>` at an offset — it's tied to the visitor
pattern for in-place qpos normalization during integration. The constraint
assembly path needs a pure `[f64; 4] → [f64; 4]` function with no mutable
borrows on `data.qpos` (which is already borrowed immutably). A standalone
`normalize_quat4` is the correct factoring.

**Why `normalize_quat4` is required (not redundant):** The pipeline calls
`forward()` (which runs constraint assembly) BEFORE `integrate()` (which calls
`mj_normalize_quat()`). So on the first step — or whenever `qpos` is set
directly by user code — constraint assembly sees raw, potentially unnormalized
quaternions. The `normalize_quat4()` call in S2/S3 is the primary defense, not
a redundancy. MuJoCo does the same: `mj_instantiateLimit()` calls
`mju_normalize4(quat)` on the joint quaternion before computing the axis-angle.

```rust
/// Normalize a quaternion [w, x, y, z]. Returns identity if norm < 1e-10.
/// Matches MuJoCo's mju_normalize4 and our normalize_quaternion() convention.
#[inline]
fn normalize_quat4(q: [f64; 4]) -> [f64; 4] {
    let norm = (q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]).sqrt();
    if norm > 1e-10 {
        [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm]
    } else {
        [1.0, 0.0, 0.0, 0.0]  // identity
    }
}
```

###### `ball_limit_axis_angle` — axis-angle extraction

Mirrors MuJoCo's two-step process: compute the axis-angle vector via
`mju_quat2Vel`, then extract magnitude and unit direction via `mju_normalize3`.
Uses explicit norm guards (our `1e-10` convention, more conservative than
MuJoCo's `mjMINVAL = 1e-15`) instead of nalgebra's `normalize_mut()` to avoid
NaN propagation on zero/near-zero vectors.

```rust
/// Compute ball joint limit quantities from a unit quaternion [w, x, y, z].
/// Returns `(unit_dir, angle)` where:
///   - `angle = |theta| >= 0` (unsigned rotation magnitude)
///   - `unit_dir = sign(theta) * axis` (Jacobian direction)
///
/// When `angle < 1e-10` (near-identity), returns `(Vector3::z(), 0.0)`.
/// The unit_dir value is arbitrary in this case — the caller must not use it
/// because `dist = limit - 0 > 0` means no constraint is instantiated.
///
/// Matches MuJoCo's `mju_quat2Vel(angleAxis, q, 1)` followed by
/// `value = mju_normalize3(angleAxis)`.
fn ball_limit_axis_angle(q: [f64; 4]) -> (Vector3<f64>, f64) {
    // Step 1: mju_quat2Vel with dt=1
    let v = Vector3::new(q[1], q[2], q[3]);
    let sin_half = v.norm();

    // Guard: near-identity → angle ≈ 0, axis undefined.
    // MuJoCo's mju_normalize3 defaults to (1,0,0) for sub-mjMINVAL vectors.
    // We use (0,0,1) to match nalgebra's Vector3::z() convention — the choice
    // is arbitrary because the caller never uses unit_dir when angle ≈ 0
    // (dist = limit - 0 > 0 → no constraint instantiated).
    if sin_half < 1e-10 {
        return (Vector3::z(), 0.0);
    }

    let axis = v / sin_half;  // unit rotation axis
    let mut theta = 2.0 * sin_half.atan2(q[0]);
    if theta > std::f64::consts::PI {
        theta -= 2.0 * std::f64::consts::PI;
    }

    // Step 2: angleAxis = theta * axis, then normalize3 to get magnitude + direction.
    // |theta * axis| = |theta| (since |axis| = 1), direction = sign(theta) * axis.
    // This is equivalent to mju_normalize3(angleAxis).
    let angle = theta.abs();
    let unit_dir = if theta >= 0.0 { axis } else { -axis };

    (unit_dir, angle)
}
```

This avoids nalgebra's `normalize_mut()` (which produces NaN for zero vectors)
and instead uses explicit guards matching the codebase convention.

**Edge cases:**
- **Near-identity** (`sin_half < 1e-10`): Early return with `angle = 0.0`.
  Since `dist = limit - 0 > 0` for any positive limit, the constraint is never
  instantiated. The returned `unit_dir = z_axis` is never used.
- **Near-π** (`theta ≈ π`): The exponential map derivative has a singularity,
  making the Jacobian less reliable. MuJoCo handles this gracefully through soft
  constraints — the solver iterates and the small Jacobian error is absorbed by
  compliance. The wrapping logic is continuous through π: a rotation of `π + ε`
  maps to `angle = π - ε` about the opposite axis, matching the SO(3) topology.
- **Quaternion sign** (`q` vs `-q`): Both represent the same rotation.
  `sin_half = |(x,y,z)|` is always ≥ 0, so `atan2(sin_half, w)` and
  `atan2(sin_half, -w)` differ by π. The wrapping correction ensures both
  produce the same `angle` and `unit_dir`.
- **Exact boundary** (`angle == limit`): `dist = limit - angle = 0.0`. Since
  `dist < 0.0` is false, no constraint is created. The joint is exactly at the
  limit surface but not violating it.

##### S2. Constraint counting

Add `MjJointType::Ball` arm to the constraint counting loop. The `jnt_limited`
check is already at the top of the loop — this arm only executes for limited ball
joints.

```rust
MjJointType::Ball => {
    let adr = model.jnt_qpos_adr[jnt_id];
    let q = normalize_quat4([data.qpos[adr], data.qpos[adr+1],
                              data.qpos[adr+2], data.qpos[adr+3]]);
    let (_, angle) = ball_limit_axis_angle(q);
    let limit = model.jnt_range[jnt_id].0.max(model.jnt_range[jnt_id].1);
    let dist = limit - angle;
    if dist < 0.0 {  // margin = 0.0 (see S6)
        nefc += 1;
    }
}
MjJointType::Free => {
    // MuJoCo does not support free joint limits.
    // Silently ignore — no constraint rows.
}
```

##### S3. Constraint assembly

Add `MjJointType::Ball` arm to the assembly loop. Must use identical
activation logic as S2 to prevent counting/assembly mismatch.

```rust
MjJointType::Ball => {
    let qpos_adr = model.jnt_qpos_adr[jnt_id];
    let dof_adr = model.jnt_dof_adr[jnt_id];
    let q = normalize_quat4([data.qpos[qpos_adr], data.qpos[qpos_adr+1],
                              data.qpos[qpos_adr+2], data.qpos[qpos_adr+3]]);
    let (unit_dir, angle) = ball_limit_axis_angle(q);
    let limit = model.jnt_range[jnt_id].0.max(model.jnt_range[jnt_id].1);
    let dist = limit - angle;

    if dist < 0.0 {  // margin = 0.0 (see S6)
        // Jacobian: -unit_dir on 3 angular DOFs
        // unit_dir = sign(theta) * axis, so J = -sign(theta) * axis
        data.efc_J[(row, dof_adr + 0)] = -unit_dir.x;
        data.efc_J[(row, dof_adr + 1)] = -unit_dir.y;
        data.efc_J[(row, dof_adr + 2)] = -unit_dir.z;

        // Constraint-space velocity: J · qvel
        let vel = -(unit_dir.x * data.qvel[dof_adr]
                  + unit_dir.y * data.qvel[dof_adr + 1]
                  + unit_dir.z * data.qvel[dof_adr + 2]);

        // finalize_row! args: solref, solimp, pos, margin, vel, floss, type, dim, id, mu
        // - margin = 0.0: hardcoded (see S6 known limitation)
        // - floss = 0.0: limit constraints have no friction loss component
        // - dim = 1: single constraint row (cone limit, not per-axis)
        // - mu = [0.0; 5]: limit constraints have no friction — mu is only
        //   meaningful for contact constraints (normal + tangential friction)
        finalize_row!(
            model.jnt_solref[jnt_id],
            model.jnt_solimp[jnt_id],
            dist,
            0.0,
            vel,
            0.0,
            ConstraintType::LimitJoint,
            1,
            jnt_id,
            [0.0; 5]
        );
    }
}
MjJointType::Free => {
    // No limit support for free joints (matches MuJoCo).
}
```

##### S4. Tree affiliation

The existing `ConstraintType::LimitJoint` tree affiliation logic already handles
arbitrary `jnt_id` → `jnt_body` → `body_treeid` mapping. No change needed — ball
joint limits automatically affiliate to the correct tree.

##### S5. Force extraction (`jnt_limit_frc`)

The existing force extraction in `mj_fwd_constraint()` scans `efc_type` for
`ConstraintType::LimitJoint` and writes `data.jnt_limit_frc[data.efc_id[i]] =
data.efc_force[i]`. Since ball joint limits use the same `ConstraintType::LimitJoint`
and pass `jnt_id` as the constraint `id`, this logic works automatically. No code
change needed — but tests must verify the force is correctly propagated (see T13).

##### S5b. MJCF parsing prerequisite

Ball joint limits depend on the MJCF parser correctly populating `jnt_limited`,
`jnt_range`, `jnt_solref`, and `jnt_solimp` for ball joints. The existing parser
handles these attributes generically for all joint types — `limited="true"` sets
`jnt_limited[jnt_id] = true`, and `range="0 45"` populates `jnt_range[jnt_id]`
with degree-to-radian conversion when `<compiler angle="degree"/>` is active.
No parser changes are needed.

**Verification:** T2 explicitly asserts that `model.jnt_limited[0] == true` and
`model.jnt_range[0].1 == 45°→rad` after parsing a ball joint with
`limited="true" range="0 45"` under degree mode. T9 verifies radian mode
(no double-conversion). These tests confirm the parser path is correct for
ball joints before any constraint logic executes.

##### S6. Known limitation: `jnt_margin`

MuJoCo activates joint limit constraints when `dist < jnt_margin[i]`, allowing
soft activation before the limit is reached. Our code uses hardcoded `margin = 0.0`
for all joint limits. This is a pre-existing gap affecting hinge/slide limits too —
not introduced by this item. A future item should add `jnt_margin` parsing and wire
it into the activation condition for all joint types.

#### Acceptance Criteria

1. **Cone limit active**: A ball joint with `range="0 60"` (`<compiler angle="degree"/>`)
   constrains total rotation to ≤ 60°. Applying torque beyond the limit produces a
   constraint force that resists further rotation. Verify `theta` stays ≤ 60° + solver
   softness (determined by solref/solimp).

2. **Range interpretation**: `range="0 60"` and `range="60 0"` produce identical
   behavior — `max(0, 60) = 60°` is the cone half-angle.

3. **Jacobian direction (positive theta)**: Apply constant torque about X-axis on a
   ball joint with `range="0 30"`. When the joint is rotated about +X past the limit,
   the Jacobian should be `(-1, 0, 0)` — opposing the rotation.

4. **Jacobian direction (negative theta / wrap)**: A ball joint rotated 200° about
   Z-axis wraps to `theta = -160°`, `|theta| = 160°`. With `range="0 150"`, the
   limit is violated. The Jacobian should be `(0, 0, +1)` (not `(0, 0, -1)`),
   because `unit_dir = sign(-160°) * z_axis = -z_axis`, so `J = -(-z_axis) = +z_axis`.
   This correctly opposes further rotation in the direction that increases `|theta|`.

5. **Free joint limits ignored**: A free joint with `limited="true" range="0 45"`
   produces **zero** constraint rows. No limit enforcement on the rotational or
   translational components. This matches MuJoCo, which does not support free joint
   limits.

6. **Near-identity regression**: A ball joint at rest (identity quaternion, `theta ≈ 0`)
   with any `range > 0` produces no constraint row. Verify `nefc` is unchanged.

7. **Unlimited regression**: Ball joints with `limited="false"` produce no constraint
   rows. All existing hinge/slide limit tests pass unchanged.

8. **MuJoCo conformance (numerical)**: For a ball joint rotated to 50° about axis
   `(0.6, 0.8, 0)`, with `range="0 45"` (in degrees), verify:
   - `angle = 50° = 5π/18 ≈ 0.8727 rad`
   - `limit = 45° = π/4 ≈ 0.7854 rad`
   - `dist = π/4 - 5π/18 = -π/36 ≈ -0.0873 rad`
   - `unit_dir ≈ (0.6, 0.8, 0.0)` (theta > 0, so `unit_dir = axis`)
   - `J = (-0.6, -0.8, 0.0)` at the 3 angular DOFs
   - `efc_vel = J · qvel` for a known angular velocity (verifies constraint-space
     velocity computation — incorrect sign/indexing would produce wrong solver damping)
   - Exactly 1 constraint row of type `LimitJoint`.

9. **Quaternion wrap**: A ball joint rotated past 180° (e.g., `theta_initial = 200°`)
   correctly wraps via the `theta > π → theta -= 2π` branch, producing
   `theta = -160°` and `angle = |theta| = 160°`. The cone limit on `angle` activates
   correctly, and the Jacobian direction is correct (see criterion 4).

10. **Counting/assembly consistency**: The number of constraint rows counted in
    Phase 1 exactly equals the rows assembled in Phase 3. No panics from row
    overflow or underflow. Test with multiple ball joints, some limited, some not,
    some within limits, some violated.

11. **Limit force propagation**: After the constraint solver runs, `jnt_limit_frc[jnt_id]`
    is non-zero for a ball joint whose limit is actively violated, and matches
    `efc_force` for the corresponding constraint row.

12. **Mixed joint type consistency**: Hinge limits and ball limits coexist correctly
    in the same model. Counting and assembly produce the correct total `nefc` and
    each constraint row has the correct type, Jacobian dimensionality, and `efc_id`.

13. **Degenerate range locks joint**: `range="0 0"` (cone half-angle = 0) produces a
    limit constraint for any non-identity orientation. The solver pushes the joint
    back toward identity — effectively a rotational lock. After stepping, the
    rotation angle decreases toward zero. This matches MuJoCo behavior where
    `max(0, 0) = 0` makes `dist = 0 - angle = -angle ≤ 0` for all rotations.

14. **Near-π rotation**: A ball joint rotated 175° with `range="0 170"` correctly
    activates the limit (5° violation). The Jacobian direction is correct despite
    the near-singularity in the exponential map derivative. Multi-step simulation
    shows the solver converges — the joint angle decreases toward 170° after
    several steps, confirming the soft-constraint approach handles near-π gracefully.

15. **Small violation precision**: A ball joint rotated 45.5° with `range="0 45"`
    (0.5° violation = ~0.00873 rad) produces exactly 1 constraint row with
    `dist ≈ -0.00873 rad`. Verifies no precision loss for small violations.

16. **Negative-w quaternion in pipeline**: A ball joint whose quaternion has `w < 0`
    (valid — `q` and `-q` represent the same rotation) correctly activates limits
    and produces the same constraint distance and Jacobian direction as the
    equivalent positive-`w` quaternion. Verifies the double-cover invariance
    holds through the full constraint assembly pipeline, not just the helper.

17. **Multiple simultaneous violations**: Two or more ball joints with violated limits
    in the same model each produce their own constraint row. The counting phase and
    assembly phase agree on the total `nefc`, and each row has the correct `efc_id`,
    Jacobian entries, and `dist` value.

18. **`efc_vel` sign correctness**: The constraint-space velocity `efc_vel = J · qvel`
    has the correct sign for both cases: angular velocity pushing *into* the limit
    (negative `efc_vel` → solver applies stronger damping) and angular velocity
    pulling *out of* the limit (positive `efc_vel` → solver applies less damping).
    Incorrect sign would cause the solver to amplify instead of damp violations.

19. **Unnormalized quaternion robustness**: A ball joint whose quaternion has drifted
    from unit norm (e.g., `|q| = 1.05` after Euler integration, or user-set qpos)
    produces identical constraint distance and Jacobian as the equivalent unit
    quaternion, because `normalize_quat4()` in the constraint assembly code (S2/S3)
    normalizes before computing the axis-angle. (Note: `mj_normalize_quat()` runs
    in `integrate()`, which is AFTER `forward()` → constraint assembly. So the
    in-assembly normalization is the primary defense, not a redundancy.)

20. **Reversed range parsing**: `range="45 0"` is parsed and stored as `(45°→rad,
    0°→rad)` without swapping. The constraint assembly correctly uses
    `max(r0, r1) = 45°` at runtime. Parser-level and runtime-level assertions both
    pass.

21. **Margin = 0.0 regression anchor**: With the current hardcoded `margin = 0.0`,
    a ball joint rotated 0.5° *below* the limit produces no constraint row, while
    0.5° *above* the limit produces exactly 1 row. This documents the sharp
    activation boundary (no soft pre-activation zone) as a regression anchor for
    the future `jnt_margin` implementation.

#### Tests

All tests use `[w, x, y, z]` quaternion ordering (MuJoCo convention). Helper to
construct a quaternion from axis-angle:

```rust
/// Build quaternion [w, x, y, z] for rotation of `angle_deg` degrees about `axis`.
fn quat_from_axis_angle_deg(axis: [f64; 3], angle_deg: f64) -> [f64; 4] {
    let half = (angle_deg / 2.0_f64).to_radians();
    let norm = (axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]).sqrt();
    let s = half.sin() / norm;
    [half.cos(), axis[0]*s, axis[1]*s, axis[2]*s]
}
```

**Test placement:** T1 + T1b unit tests (8 tests) go in a `#[cfg(test)] mod
ball_limit_tests` module inside `mujoco_pipeline.rs` (they test private helper
functions — matching the established pattern of `impedance_tests`,
`subquat_tests`, etc.). T2–T22 integration tests (22 tests) go in
`sim/L0/tests/integration/ball_joint_limits.rs`. The `quat_from_axis_angle_deg`
helper is defined in both locations (it's a trivial 4-line function; duplicating
is simpler than creating a shared test utility crate).

**Integration test helpers:** T7, T14, and T15 extract the rotation angle from
the simulated quaternion to verify solver convergence. Since `normalize_quat4`
and `ball_limit_axis_angle` are private to `mujoco_pipeline.rs`, the integration
test file defines a local `extract_ball_angle(qpos: &[f64], offset: usize) -> f64`
helper that reimplements the same quaternion-to-angle computation:

```rust
/// Extract rotation angle (radians) from quaternion at qpos[offset..offset+4].
/// Reimplements normalize_quat4 + ball_limit_axis_angle for integration tests.
fn extract_ball_angle(qpos: &[f64], offset: usize) -> f64 {
    let (w, x, y, z) = (qpos[offset], qpos[offset+1], qpos[offset+2], qpos[offset+3]);
    let norm = (w*w + x*x + y*y + z*z).sqrt();
    let (w, x, y, z) = if norm > 1e-10 { (w/norm, x/norm, y/norm, z/norm) }
                        else { return 0.0; };
    let sin_half = (x*x + y*y + z*z).sqrt();
    if sin_half < 1e-10 { return 0.0; }
    let mut theta = 2.0 * sin_half.atan2(w);
    if theta > std::f64::consts::PI { theta -= 2.0 * std::f64::consts::PI; }
    theta.abs()
}
```

##### T1. Unit test: `ball_limit_axis_angle` (in `mujoco_pipeline.rs`)

```rust
#[test]
fn test_ball_limit_axis_angle_90deg_about_z() {
    // 90° rotation about Z: q = (cos(45°), 0, 0, sin(45°))
    let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 90.0);
    let (unit_dir, angle) = ball_limit_axis_angle(q);
    assert_relative_eq!(angle, PI / 2.0, epsilon = 1e-10);
    assert_relative_eq!(unit_dir.z, 1.0, epsilon = 1e-10);  // +Z (theta > 0)
    assert!(unit_dir.x.abs() < 1e-10);
    assert!(unit_dir.y.abs() < 1e-10);
}

#[test]
fn test_ball_limit_axis_angle_50deg_about_oblique() {
    // 50° about axis (0.6, 0.8, 0) — matches acceptance criterion 8
    let q = quat_from_axis_angle_deg([0.6, 0.8, 0.0], 50.0);
    let (unit_dir, angle) = ball_limit_axis_angle(q);
    assert_relative_eq!(angle, 50.0_f64.to_radians(), epsilon = 1e-10);
    assert_relative_eq!(unit_dir.x, 0.6, epsilon = 1e-10);
    assert_relative_eq!(unit_dir.y, 0.8, epsilon = 1e-10);
    assert_relative_eq!(unit_dir.z, 0.0, epsilon = 1e-10);
}

#[test]
fn test_ball_limit_axis_angle_200deg_wraps() {
    // 200° about Z → wraps to theta = -160°, angle = 160°
    let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 200.0);
    let (unit_dir, angle) = ball_limit_axis_angle(q);
    assert_relative_eq!(angle, 160.0_f64.to_radians(), epsilon = 1e-10);
    // theta < 0 after wrap, so unit_dir = sign(-) * z = -z
    assert_relative_eq!(unit_dir.z, -1.0, epsilon = 1e-10);
}

#[test]
fn test_ball_limit_axis_angle_identity() {
    let q = [1.0, 0.0, 0.0, 0.0];
    let (_, angle) = ball_limit_axis_angle(q);
    assert!(angle < 1e-10, "identity should have zero rotation angle");
    // unit_dir is arbitrary (z-axis default) — not asserted
}

#[test]
fn test_ball_limit_axis_angle_negative_quat() {
    // -q represents the same rotation as q. Verify identical output.
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    let neg_q = [-q[0], -q[1], -q[2], -q[3]];
    let (dir1, angle1) = ball_limit_axis_angle(q);
    let (dir2, angle2) = ball_limit_axis_angle(neg_q);
    assert_relative_eq!(angle1, angle2, epsilon = 1e-10);
    assert_relative_eq!(dir1, dir2, epsilon = 1e-10);
}

#[test]
fn test_ball_limit_axis_angle_exactly_180deg() {
    // Boundary case: exactly π rotation
    let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 180.0);
    let (unit_dir, angle) = ball_limit_axis_angle(q);
    assert_relative_eq!(angle, PI, epsilon = 1e-10);
    assert_relative_eq!(unit_dir.y, 1.0, epsilon = 1e-10);
}

#[test]
fn test_ball_limit_axis_angle_exact_boundary() {
    // Rotation angle exactly equals a typical limit — verify angle is precise
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 45.0);
    let (unit_dir, angle) = ball_limit_axis_angle(q);
    assert_relative_eq!(angle, 45.0_f64.to_radians(), epsilon = 1e-10);
    assert_relative_eq!(unit_dir.x, 1.0, epsilon = 1e-10);
}
```

##### T2. Integration test: constraint row counting (degree mode)

```rust
#[test]
fn test_ball_limit_creates_constraint_row() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Verify range parsed and converted to radians
    assert!(model.jnt_limited[0]);
    assert_relative_eq!(model.jnt_range[0].1, 45.0_f64.to_radians(), epsilon = 1e-6);

    // Rotate ball joint 60° about X (past the 45° limit)
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0];  // w
    data.qpos[1] = q[1];  // x
    data.qpos[2] = q[2];  // y
    data.qpos[3] = q[3];  // z

    data.step(&model).unwrap();

    // Expect exactly 1 LimitJoint constraint row
    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "should have exactly 1 ball limit row");
}
```

##### T3. Integration test: Jacobian, dist, and efc_vel (MuJoCo conformance)

```rust
#[test]
fn test_ball_limit_jacobian_dist_and_vel() {
    // 50° about axis (0.6, 0.8, 0), limit = 45° → dist = -π/36 ≈ -0.0873
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Set quaternion for 50° rotation about (0.6, 0.8, 0)
    let q = quat_from_axis_angle_deg([0.6, 0.8, 0.0], 50.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    // Set known angular velocity: omega = (1.0, 2.0, 0.5) rad/s
    let dof = model.jnt_dof_adr[0];
    data.qvel[dof + 0] = 1.0;
    data.qvel[dof + 1] = 2.0;
    data.qvel[dof + 2] = 0.5;

    data.step(&model).unwrap();

    let limit_row = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("should have a ball limit constraint row");

    // Jacobian: J = -unit_dir = -(0.6, 0.8, 0)
    assert_relative_eq!(data.efc_J[(limit_row, dof + 0)], -0.6, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 1)], -0.8, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 2)],  0.0, epsilon = 1e-4);

    // dist = π/4 - 5π/18 = -π/36 ≈ -0.0873
    let expected_dist = std::f64::consts::PI / 4.0 - 5.0 * std::f64::consts::PI / 18.0;
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-4);
    assert!(data.efc_pos[limit_row] < 0.0, "dist should be negative (violated)");

    // Constraint-space velocity: efc_vel = J · qvel
    // J = (-0.6, -0.8, 0), qvel = (1.0, 2.0, 0.5)
    // efc_vel = (-0.6)(1.0) + (-0.8)(2.0) + (0)(0.5) = -0.6 - 1.6 = -2.2
    assert_relative_eq!(data.efc_vel[limit_row], -2.2, epsilon = 1e-4,
        "efc_vel should equal J · qvel");

    // Exactly 1 constraint row
    assert_eq!(data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint)).count(), 1);
}
```

##### T4. Integration test: free joint limits ignored

```rust
#[test]
fn test_free_joint_limits_ignored() {
    let mjcf = r#"
        <mujoco>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="free" limited="true" range="0 0.5"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Free joint qpos: [x, y, z, qw, qx, qy, qz]
    // Rotate far past the "limit" — 120° about X
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 120.0);
    data.qpos[3] = q[0];  // qw
    data.qpos[4] = q[1];  // qx
    data.qpos[5] = q[2];  // qy
    data.qpos[6] = q[3];  // qz

    data.step(&model).unwrap();

    // No LimitJoint rows should exist — MuJoCo skips free joint limits entirely
    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 0, "free joints should have no limit constraints");
}
```

##### T5. Integration test: no constraint when within limits

```rust
#[test]
fn test_ball_limit_no_constraint_within_range() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate only 30° about Y — within the 45° limit
    let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 30.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 0, "within-limit ball joint should produce no constraint");
}
```

##### T5b. Integration test: no constraint at exact boundary

```rust
#[test]
fn test_ball_limit_no_constraint_at_exact_boundary() {
    // Rotation angle exactly equals the cone half-angle.
    // dist = limit - angle = 0.0, so dist < 0.0 is FALSE → no constraint.
    // Note: this relies on atan2(sin(x), cos(x)) == x in f64, which holds
    // for IEEE 754 but is not guaranteed by the Rust spec. If this test
    // flakes, the fix is to use 44.999° instead.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate exactly 45° about X — at the boundary
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 45.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 0,
        "ball joint at exact boundary (dist=0) should produce no constraint");
}
```

##### T6. Integration test: unlimited ball joint produces no rows

```rust
#[test]
fn test_unlimited_ball_joint_no_constraint() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="false" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 170° — far past any limit, but limited="false"
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 170.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 0, "unlimited ball joint should produce no constraint rows");
}
```

##### T7. Integration test: limit enforcement over time

Solver softness tolerance: with `DEFAULT_SOLREF = [0.02, 1.0]` (timeconst =
0.02s → 50 Hz natural frequency, dampratio = 1.0 → critical damping) and
`DEFAULT_SOLIMP = [0.9, 0.95, 0.001, 0.5, 2.0]`, the constraint allows ~1–2°
of penetration under moderate velocity. The 32° threshold (30° limit + 2°
tolerance) accounts for this softness. If default solver parameters change,
this threshold may need adjustment.

```rust
#[test]
fn test_ball_limit_enforced_over_simulation() {
    // Ball joint limited to 30°, initial rotation 35° about X (already past limit),
    // with angular velocity pushing further into the limit.
    // Default solref/solimp → stiff spring-damper, allows ~1-2° penetration.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 30"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Start at 35° about X
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 35.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];
    // Angular velocity pushing further into the limit (2 rad/s about X)
    data.qvel[0] = 2.0;

    for _ in 0..500 {
        data.step(&model).unwrap();
    }

    // Extract rotation angle from quaternion
    let angle = extract_ball_angle(data.qpos.as_slice(), 0);
    let angle_deg = angle.to_degrees();
    // The solver should push the angle back toward the 30° limit.
    // Allow ~2° solver softness (default solref/solimp spring-damper compliance).
    assert!(angle_deg < 32.0,
        "ball limit should hold near 30°: angle={angle_deg:.1}°, limit=30°");
}
```

##### T8. Integration test: wrapped rotation Jacobian

```rust
#[test]
fn test_ball_limit_wrap_jacobian_sign() {
    // Rotate 200° about Z → wraps to theta=-160°, angle=160°
    // Limit = 150° → violated by 10°
    // Jacobian should be (0, 0, +1) because unit_dir = sign(-160°) * z = -z,
    // so J = -(-z) = +z.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 150"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Set quaternion for 200° rotation about Z
    let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 200.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_row = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("wrapped rotation should trigger ball limit");
    let dof = model.jnt_dof_adr[0];

    // Jacobian should point along +Z (not -Z)
    assert_relative_eq!(data.efc_J[(limit_row, dof + 0)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 1)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 2)], 1.0, epsilon = 1e-4,
        "wrapped rotation Jacobian should point along +Z");

    // dist = 150° - 160° = -10° in radians
    let expected_dist = 150.0_f64.to_radians() - 160.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-4);
}
```

##### T9. Integration test: radian-mode range (no double-conversion)

```rust
#[test]
fn test_ball_limit_radian_mode() {
    // Default compiler uses radians — verify range is not double-converted
    let limit_rad = std::f64::consts::PI / 4.0;  // 45° in radians
    let mjcf = format!(r#"
        <mujoco>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 {limit_rad}"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#);
    let model = load_model(&mjcf).unwrap();

    // Range should be stored as-is (radians), not converted again
    assert_relative_eq!(model.jnt_range[0].1, limit_rad, epsilon = 1e-10);

    let mut data = model.make_data();

    // Rotate 60° (> 45° limit)
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0]; data.qpos[1] = q[1];
    data.qpos[2] = q[2]; data.qpos[3] = q[3];

    data.step(&model).unwrap();
    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint)).count();
    assert_eq!(limit_rows, 1, "radian-mode ball limit should activate");
}
```

##### T10. Integration test: range interpretation symmetry

```rust
#[test]
fn test_ball_limit_range_symmetry() {
    // range="0 45" and range="45 0" should produce identical behavior
    // (both use max(r0, r1) = 45° as the cone half-angle)
    for range in &["0 45", "45 0"] {
        let mjcf = format!(r#"
            <mujoco>
                <compiler angle="degree"/>
                <option gravity="0 0 0" timestep="0.001"/>
                <worldbody>
                    <body pos="0 0 0">
                        <joint type="ball" limited="true" range="{range}"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#);
        let model = load_model(&mjcf).unwrap();
        let mut data = model.make_data();

        // Rotate 60° about X (past limit)
        let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
        data.qpos[0] = q[0]; data.qpos[1] = q[1];
        data.qpos[2] = q[2]; data.qpos[3] = q[3];

        data.step(&model).unwrap();
        let limit_rows = data.efc_type.iter()
            .filter(|t| matches!(t, ConstraintType::LimitJoint)).count();
        assert_eq!(limit_rows, 1,
            "range=\"{range}\" should produce 1 ball limit constraint");
    }
}
```

##### T11. Integration test: multiple ball joints (counting/assembly consistency)

```rust
#[test]
fn test_multiple_ball_joints_counting_consistency() {
    // 3 ball joints: joint_a (limited, violated), joint_b (limited, within),
    // joint_c (unlimited). Verify exactly 1 constraint row total.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint name="a" type="ball" limited="true" range="0 30"/>
                    <geom type="sphere" size="0.1" mass="0.5"/>
                    <body pos="0.2 0 0">
                        <joint name="b" type="ball" limited="true" range="0 90"/>
                        <geom type="sphere" size="0.1" mass="0.5"/>
                        <body pos="0.2 0 0">
                            <joint name="c" type="ball"/>
                            <geom type="sphere" size="0.1" mass="0.5"/>
                        </body>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Joint a: rotated 50° about X → exceeds 30° limit (violated)
    let qa = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 50.0);
    let adr_a = model.jnt_qpos_adr[0];
    data.qpos[adr_a]   = qa[0]; data.qpos[adr_a+1] = qa[1];
    data.qpos[adr_a+2] = qa[2]; data.qpos[adr_a+3] = qa[3];

    // Joint b: rotated 45° about Y → within 90° limit (satisfied)
    let qb = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 45.0);
    let adr_b = model.jnt_qpos_adr[1];
    data.qpos[adr_b]   = qb[0]; data.qpos[adr_b+1] = qb[1];
    data.qpos[adr_b+2] = qb[2]; data.qpos[adr_b+3] = qb[3];

    // Joint c: rotated 170° about Z → unlimited, no constraint
    let qc = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 170.0);
    let adr_c = model.jnt_qpos_adr[2];
    data.qpos[adr_c]   = qc[0]; data.qpos[adr_c+1] = qc[1];
    data.qpos[adr_c+2] = qc[2]; data.qpos[adr_c+3] = qc[3];

    // This must not panic (counting == assembly)
    data.step(&model).unwrap();

    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "only joint_a should produce a limit constraint");

    // Verify efc_id points to joint 0 (joint_a)
    let limit_row = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint)).unwrap();
    assert_eq!(data.efc_id[limit_row], 0, "limit constraint should reference joint_a");
}
```

##### T12. Integration test: mixed hinge + ball limits (cross-type consistency)

```rust
#[test]
fn test_mixed_hinge_and_ball_limits() {
    // Hinge joint (violated upper limit) + ball joint (violated cone limit).
    // Verify both produce constraint rows with correct types and Jacobians.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint name="hinge_j" type="hinge" limited="true" range="-30 30"/>
                    <geom type="sphere" size="0.1" mass="0.5"/>
                    <body pos="0.2 0 0">
                        <joint name="ball_j" type="ball" limited="true" range="0 45"/>
                        <geom type="sphere" size="0.1" mass="0.5"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Hinge: set past upper limit (40° > 30°)
    data.qpos[model.jnt_qpos_adr[0]] = 40.0_f64.to_radians();

    // Ball: set past cone limit (60° > 45°)
    let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 60.0);
    let ball_adr = model.jnt_qpos_adr[1];
    data.qpos[ball_adr]     = q[0];
    data.qpos[ball_adr + 1] = q[1];
    data.qpos[ball_adr + 2] = q[2];
    data.qpos[ball_adr + 3] = q[3];

    // Must not panic (counting == assembly across mixed types)
    data.step(&model).unwrap();

    let limit_rows: Vec<_> = data.efc_type.iter().enumerate()
        .filter(|(_, t)| matches!(t, ConstraintType::LimitJoint))
        .collect();
    assert_eq!(limit_rows.len(), 2,
        "should have 2 limit rows: 1 hinge upper + 1 ball cone");

    // First limit row should be the hinge (joint 0, assembled first)
    let (hinge_row, _) = limit_rows[0];
    let hinge_dof = model.jnt_dof_adr[0];
    assert_eq!(data.efc_id[hinge_row], 0, "first limit row should be hinge joint");
    // Hinge upper limit Jacobian: -1.0 at single DOF
    assert_relative_eq!(data.efc_J[(hinge_row, hinge_dof)], -1.0, epsilon = 1e-10);

    // Second limit row should be the ball (joint 1)
    let (ball_row, _) = limit_rows[1];
    let ball_dof = model.jnt_dof_adr[1];
    assert_eq!(data.efc_id[ball_row], 1, "second limit row should be ball joint");
    // Ball Jacobian: 3 non-zero entries at angular DOFs
    // 60° about Y → unit_dir = (0, 1, 0), J = -(0, 1, 0) = (0, -1, 0)
    assert_relative_eq!(data.efc_J[(ball_row, ball_dof + 0)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(ball_row, ball_dof + 1)], -1.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(ball_row, ball_dof + 2)], 0.0, epsilon = 1e-4);
}
```

##### T13. Integration test: `jnt_limit_frc` propagation

```rust
#[test]
fn test_ball_limit_force_propagation() {
    // Verify that the constraint solver populates jnt_limit_frc for ball joints.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 30"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 60° about X — well past the 30° limit
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    // jnt_limit_frc[0] should be non-zero — the solver produced a constraint force
    assert!(data.jnt_limit_frc[0].abs() > 1e-10,
        "ball joint limit force should be non-zero: got {}",
        data.jnt_limit_frc[0]);

    // The force should also appear in efc_force for the LimitJoint row
    let limit_row = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("should have a ball limit constraint row");
    assert_relative_eq!(
        data.jnt_limit_frc[0], data.efc_force[limit_row],
        epsilon = 1e-15,
        "jnt_limit_frc should match efc_force for the limit row");
}
```

##### T14. Integration test: degenerate range "0 0" locks joint

```rust
#[test]
fn test_ball_limit_degenerate_range_locks_joint() {
    // range="0 0" → cone half-angle = max(0, 0) = 0.
    // Any non-identity rotation violates the limit (dist = 0 - angle = -angle ≤ 0).
    // This effectively locks the ball joint rotationally.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 20° about Y — any rotation should violate the zero-cone limit
    let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 20.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    // Must have exactly 1 constraint row
    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "degenerate range should activate limit for any rotation");

    // dist = 0 - 20° in radians ≈ -0.349
    let limit_row = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint)).unwrap();
    assert!(data.efc_pos[limit_row] < -0.3,
        "dist should be strongly negative: got {}", data.efc_pos[limit_row]);

    // After several steps, the solver should push rotation toward identity
    for _ in 0..200 {
        data.step(&model).unwrap();
    }
    let angle = extract_ball_angle(data.qpos.as_slice(), 0);
    assert!(angle.to_degrees() < 5.0,
        "degenerate range should push joint toward identity: angle={:.1}°",
        angle.to_degrees());
}
```

##### T15. Integration test: near-π rotation with active limit

```rust
#[test]
fn test_ball_limit_near_pi_rotation() {
    // 175° rotation with 170° limit — exercises the near-singularity region
    // of the exponential map derivative. The Jacobian is less precise here,
    // but the soft constraint solver should still converge.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 170"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 175° about X — 5° past the 170° limit, near π
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 175.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    // Constraint should activate
    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "near-π rotation should trigger ball limit");

    // dist ≈ 170° - 175° = -5° ≈ -0.0873 rad
    let limit_row = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint)).unwrap();
    let expected_dist = 170.0_f64.to_radians() - 175.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-4);

    // Jacobian should point along -X (theta > 0, unit_dir = +X, J = -X)
    let dof = model.jnt_dof_adr[0];
    assert_relative_eq!(data.efc_J[(limit_row, dof + 0)], -1.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 1)],  0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 2)],  0.0, epsilon = 1e-4);

    // Multi-step: solver should converge despite near-π singularity
    for _ in 0..200 {
        data.step(&model).unwrap();
    }
    let angle = extract_ball_angle(data.qpos.as_slice(), 0);
    // Allow 3° tolerance — near-π Jacobian is less precise, solver needs more steps
    assert!(angle.to_degrees() < 173.0,
        "near-π limit should hold: angle={:.1}°, limit=170°", angle.to_degrees());
}
```

##### T16. Integration test: small violation precision

```rust
#[test]
fn test_ball_limit_small_violation_precision() {
    // 45.5° rotation with 45° limit — 0.5° violation (~0.00873 rad).
    // Verifies no precision loss for small violations near the boundary.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 45.5° about Z — just barely past the 45° limit
    let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 45.5);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    // Must activate — even a tiny violation should produce a constraint row
    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "small violation should still activate limit");

    // dist = 45° - 45.5° = -0.5° ≈ -0.008727 rad
    let limit_row = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint)).unwrap();
    let expected_dist = 45.0_f64.to_radians() - 45.5_f64.to_radians();
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-6,
        "small violation dist should be precise");

    // Jacobian direction: along -Z (theta > 0 about Z)
    let dof = model.jnt_dof_adr[0];
    assert_relative_eq!(data.efc_J[(limit_row, dof + 0)], 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 1)], 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 2)], -1.0, epsilon = 1e-6);
}
```

##### T17. Integration test: negative-w quaternion in pipeline

```rust
#[test]
fn test_ball_limit_negative_w_quaternion() {
    // Verify the full constraint pipeline handles q with w < 0 correctly.
    // q and -q represent the same rotation — the constraint distance, Jacobian,
    // and force should be identical regardless of which hemisphere the
    // quaternion lies in.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Run with positive-w quaternion
    let q_pos = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    let model = load_model(mjcf).unwrap();
    let mut data_pos = model.make_data();
    data_pos.qpos[0] = q_pos[0];
    data_pos.qpos[1] = q_pos[1];
    data_pos.qpos[2] = q_pos[2];
    data_pos.qpos[3] = q_pos[3];
    data_pos.step(&model).unwrap();

    // Run with negative-w quaternion (same rotation, opposite hemisphere)
    let mut data_neg = model.make_data();
    data_neg.qpos[0] = -q_pos[0];
    data_neg.qpos[1] = -q_pos[1];
    data_neg.qpos[2] = -q_pos[2];
    data_neg.qpos[3] = -q_pos[3];
    data_neg.step(&model).unwrap();

    // Both should produce exactly 1 LimitJoint row
    let row_pos = data_pos.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("positive-w should have limit row");
    let row_neg = data_neg.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("negative-w should have limit row");

    // Constraint distance must match
    assert_relative_eq!(data_pos.efc_pos[row_pos], data_neg.efc_pos[row_neg],
        epsilon = 1e-10, "dist should be identical for q and -q");

    // Jacobian must match
    let dof = model.jnt_dof_adr[0];
    for d in 0..3 {
        assert_relative_eq!(
            data_pos.efc_J[(row_pos, dof + d)],
            data_neg.efc_J[(row_neg, dof + d)],
            epsilon = 1e-10,
            "Jacobian[{d}] should be identical for q and -q");
    }
}
```

##### T18. Integration test: multiple simultaneous violations

```rust
#[test]
fn test_multiple_ball_joints_simultaneously_violated() {
    // Two ball joints, both violated. Verify counting and assembly produce
    // exactly 2 constraint rows with correct per-joint efc_id, dist, and Jacobians.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint name="ball_a" type="ball" limited="true" range="0 30"/>
                    <geom type="sphere" size="0.1" mass="0.5"/>
                    <body pos="0.2 0 0">
                        <joint name="ball_b" type="ball" limited="true" range="0 45"/>
                        <geom type="sphere" size="0.1" mass="0.5"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Joint a: 50° about X → exceeds 30° limit (20° violation)
    let qa = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 50.0);
    let adr_a = model.jnt_qpos_adr[0];
    data.qpos[adr_a]   = qa[0]; data.qpos[adr_a+1] = qa[1];
    data.qpos[adr_a+2] = qa[2]; data.qpos[adr_a+3] = qa[3];

    // Joint b: 60° about Y → exceeds 45° limit (15° violation)
    let qb = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 60.0);
    let adr_b = model.jnt_qpos_adr[1];
    data.qpos[adr_b]   = qb[0]; data.qpos[adr_b+1] = qb[1];
    data.qpos[adr_b+2] = qb[2]; data.qpos[adr_b+3] = qb[3];

    // Must not panic (counting == assembly for multiple simultaneous violations)
    data.step(&model).unwrap();

    let limit_rows: Vec<_> = data.efc_type.iter().enumerate()
        .filter(|(_, t)| matches!(t, ConstraintType::LimitJoint))
        .collect();
    assert_eq!(limit_rows.len(), 2,
        "should have 2 limit rows for 2 simultaneously violated ball joints");

    // Row 0: joint a (assembled first)
    let (row_a, _) = limit_rows[0];
    assert_eq!(data.efc_id[row_a], 0, "first limit row should reference joint a");
    let expected_dist_a = 30.0_f64.to_radians() - 50.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[row_a], expected_dist_a, epsilon = 1e-4);
    // Jacobian: 50° about X, theta > 0 → J = (-1, 0, 0)
    let dof_a = model.jnt_dof_adr[0];
    assert_relative_eq!(data.efc_J[(row_a, dof_a + 0)], -1.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(row_a, dof_a + 1)],  0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(row_a, dof_a + 2)],  0.0, epsilon = 1e-4);

    // Row 1: joint b
    let (row_b, _) = limit_rows[1];
    assert_eq!(data.efc_id[row_b], 1, "second limit row should reference joint b");
    let expected_dist_b = 45.0_f64.to_radians() - 60.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[row_b], expected_dist_b, epsilon = 1e-4);
    // Jacobian: 60° about Y, theta > 0 → J = (0, -1, 0)
    let dof_b = model.jnt_dof_adr[1];
    assert_relative_eq!(data.efc_J[(row_b, dof_b + 0)],  0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(row_b, dof_b + 1)], -1.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(row_b, dof_b + 2)],  0.0, epsilon = 1e-4);
}
```

##### T19. Integration test: `efc_vel` sign with opposing angular velocity

```rust
#[test]
fn test_ball_limit_efc_vel_opposing_velocity() {
    // Ball joint rotated 60° about X past a 45° limit. Angular velocity is
    // in the -X direction (opposing the violation — moving back toward compliance).
    // efc_vel = J · qvel should be positive (indicating motion reducing violation),
    // producing correct solver damping behavior.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // 60° about X → theta > 0, unit_dir = (+1, 0, 0), J = (-1, 0, 0)
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0]; data.qpos[1] = q[1];
    data.qpos[2] = q[2]; data.qpos[3] = q[3];

    let dof = model.jnt_dof_adr[0];

    // Case 1: velocity pushing INTO the limit (+X direction, same as rotation)
    data.qvel[dof] = 2.0;     // omega_x = +2 rad/s
    data.qvel[dof + 1] = 0.0;
    data.qvel[dof + 2] = 0.0;
    data.step(&model).unwrap();

    let row_into = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("should have limit row");
    let vel_into = data.efc_vel[row_into];
    // J · qvel = (-1)(2) + (0)(0) + (0)(0) = -2.0 (negative = pushing into limit)
    assert!(vel_into < 0.0,
        "velocity into limit should produce negative efc_vel: got {vel_into}");
    assert_relative_eq!(vel_into, -2.0, epsilon = 1e-4);

    // Case 2: velocity pulling OUT of the limit (-X direction, opposing rotation)
    let mut data2 = model.make_data();
    data2.qpos[0] = q[0]; data2.qpos[1] = q[1];
    data2.qpos[2] = q[2]; data2.qpos[3] = q[3];
    data2.qvel[dof] = -2.0;   // omega_x = -2 rad/s (opposing)
    data2.qvel[dof + 1] = 0.0;
    data2.qvel[dof + 2] = 0.0;
    data2.step(&model).unwrap();

    let row_out = data2.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("should have limit row");
    let vel_out = data2.efc_vel[row_out];
    // J · qvel = (-1)(-2) + (0)(0) + (0)(0) = +2.0 (positive = pulling out of limit)
    assert!(vel_out > 0.0,
        "velocity out of limit should produce positive efc_vel: got {vel_out}");
    assert_relative_eq!(vel_out, 2.0, epsilon = 1e-4);
}
```

##### T20. Integration test: unnormalized quaternion in constraint assembly

```rust
#[test]
fn test_ball_limit_unnormalized_quaternion() {
    // Quaternions can drift from unit norm during Euler integration.
    // The normalize_quat4() call in constraint assembly must handle this
    // gracefully — producing the same constraint distance and Jacobian as
    // the equivalent unit quaternion.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();

    // Unit quaternion for 60° about X
    let q_unit = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);

    // Run with unit quaternion
    let mut data_unit = model.make_data();
    data_unit.qpos[0] = q_unit[0]; data_unit.qpos[1] = q_unit[1];
    data_unit.qpos[2] = q_unit[2]; data_unit.qpos[3] = q_unit[3];
    data_unit.step(&model).unwrap();

    // Run with scaled quaternion (norm = 1.05 — typical drift magnitude)
    // Note: step() calls forward() (constraint assembly) BEFORE integrate()
    // (which calls mj_normalize_quat). So on this first step, constraint
    // assembly sees the raw scaled quaternion. The normalize_quat4() call
    // INSIDE constraint assembly (S2/S3) is what provides correctness here.
    // This test verifies that defense works — without it, the 5% norm error
    // would produce a measurably different rotation angle.
    let scale = 1.05;
    let mut data_scaled = model.make_data();
    data_scaled.qpos[0] = q_unit[0] * scale;
    data_scaled.qpos[1] = q_unit[1] * scale;
    data_scaled.qpos[2] = q_unit[2] * scale;
    data_scaled.qpos[3] = q_unit[3] * scale;
    data_scaled.step(&model).unwrap();

    // Both should produce exactly 1 LimitJoint row
    let row_unit = data_unit.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("unit quat should have limit row");
    let row_scaled = data_scaled.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint))
        .expect("scaled quat should have limit row");

    // Constraint distance must match (normalization produces same rotation)
    assert_relative_eq!(data_unit.efc_pos[row_unit],
                        data_scaled.efc_pos[row_scaled],
                        epsilon = 1e-8,
                        "dist should be identical for unit and scaled quaternions");

    // Jacobian must match
    let dof = model.jnt_dof_adr[0];
    for d in 0..3 {
        assert_relative_eq!(
            data_unit.efc_J[(row_unit, dof + d)],
            data_scaled.efc_J[(row_scaled, dof + d)],
            epsilon = 1e-8,
            "Jacobian[{d}] should be identical for unit and scaled quaternions");
    }
}
```

##### T21. Integration test: `range="45 0"` parser-level verification

```rust
#[test]
fn test_ball_limit_reversed_range_parsing() {
    // Verify that range="45 0" (reversed order) is parsed correctly.
    // The parser should store raw values; the constraint assembly uses
    // max(r0, r1) at runtime to extract the cone half-angle.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="45 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();

    // Verify parser stored (45°→rad, 0°→rad) — not swapped
    assert_relative_eq!(model.jnt_range[0].0, 45.0_f64.to_radians(), epsilon = 1e-6,
        "range[0] should be 45° in radians");
    assert_relative_eq!(model.jnt_range[0].1, 0.0, epsilon = 1e-10,
        "range[1] should be 0");

    // Runtime: max(45°, 0°) = 45° is the effective cone half-angle
    let mut data = model.make_data();
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0]; data.qpos[1] = q[1];
    data.qpos[2] = q[2]; data.qpos[3] = q[3];
    data.step(&model).unwrap();

    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint)).count();
    assert_eq!(limit_rows, 1, "reversed range should still activate limit");

    // dist = max(45°, 0°) - 60° = 45° - 60° = -15° in radians
    let limit_row = data.efc_type.iter()
        .position(|t| matches!(t, ConstraintType::LimitJoint)).unwrap();
    let expected_dist = 45.0_f64.to_radians() - 60.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-4);
}
```

##### T22. Integration test: margin = 0.0 regression anchor

```rust
#[test]
fn test_ball_limit_margin_zero_regression() {
    // Regression anchor for the hardcoded margin = 0.0 behavior (S6).
    // When jnt_margin support is added in a future item, this test should
    // be updated to verify margin-aware activation.
    //
    // Current behavior: constraint activates only when dist < 0.0 (strict
    // violation). A rotation 0.5° short of the limit (dist = +0.5° > 0)
    // should NOT produce a constraint row.
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 44.5° about X — 0.5° short of the 45° limit
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 44.5);
    data.qpos[0] = q[0]; data.qpos[1] = q[1];
    data.qpos[2] = q[2]; data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_rows = data.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 0,
        "with margin=0, rotation 0.5° below limit should produce no constraint");

    // Contrast: 45.5° (0.5° past limit) should activate
    let mut data2 = model.make_data();
    let q2 = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 45.5);
    data2.qpos[0] = q2[0]; data2.qpos[1] = q2[1];
    data2.qpos[2] = q2[2]; data2.qpos[3] = q2[3];
    data2.step(&model).unwrap();

    let limit_rows2 = data2.efc_type.iter()
        .filter(|t| matches!(t, ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows2, 1,
        "with margin=0, rotation 0.5° past limit should produce 1 constraint");
}
```

##### T1b. Unit test: `ball_limit_axis_angle` with near-zero norm quaternion (in `mujoco_pipeline.rs`)

```rust
#[test]
fn test_ball_limit_axis_angle_near_zero_quaternion() {
    // A quaternion with near-zero norm (degenerate) should be caught by
    // normalize_quat4 (returns identity) before reaching ball_limit_axis_angle.
    // But ball_limit_axis_angle itself should also handle near-zero sin_half
    // gracefully — returning angle = 0 and an arbitrary direction.
    // This tests the helper directly with a quaternion that has tiny imaginary part.
    let q = [1.0, 1e-15, 1e-15, 1e-15]; // Nearly identity, sin_half ≈ 1.7e-15
    let (_, angle) = ball_limit_axis_angle(q);
    assert!(angle < 1e-10, "near-zero sin_half should produce angle ≈ 0: got {angle}");
    // unit_dir is arbitrary (z-axis default) — not asserted
}
```

#### MuJoCo Conformance Citation

**Reference implementation:** MuJoCo 3.x, `engine_core_constraint.c`,
`mj_instantiateLimit()`, `mjJNT_BALL` branch. The algorithm is:
1. `mju_normalize4(quat)` — normalize joint quaternion
2. `mju_quat2Vel(angleAxis, quat, 1)` — axis-angle vector via quaternion log
3. `value = mju_normalize3(angleAxis)` — extract rotation magnitude, normalize direction
4. `dist = mju_max(range[0], range[1]) - value` — distance to cone boundary
5. Activate when `dist < margin`
6. Jacobian: `mju_scl3(jac, angleAxis, -1)` — 3 entries at angular DOF addresses
7. Single constraint row via `mj_addConstraint(..., mjCNSTR_LIMIT_JOINT, ...)`

Our implementation matches this algorithm exactly, with the documented deviations:
- Normalization threshold: `1e-10` (ours) vs `mjMINVAL = 1e-15` (MuJoCo) — inconsequential
- Near-identity fallback direction: `(0,0,1)` (ours) vs `(1,0,0)` (MuJoCo) — never used
- Margin: hardcoded `0.0` (ours) vs `jnt_margin[i]` (MuJoCo) — pre-existing gap (S6)

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `assemble_unified_constraints()`: add
  `Ball` arm to constraint counting loop and assembly loop. Add `normalize_quat4()`
  and `ball_limit_axis_angle()` helper functions. Add `Free => {}` arm (no-op,
  matching MuJoCo). Add `#[cfg(test)] mod ball_limit_tests` with T1 + T1b unit
  tests (8 tests for the private `ball_limit_axis_angle` helper — follows the
  `impedance_tests`/`subquat_tests` precedent).
- `sim/L0/tests/integration/` — new test file `ball_joint_limits.rs` with tests
  T2–T22 (22 integration tests). Add `mod ball_joint_limits;` to `mod.rs`.

---

### 39. `wrap_inside` Algorithm (Inverse Tendon Wrapping)
**Status:** ✅ Complete | **Effort:** M | **Prerequisites:** None

#### Current State

Two issues exist:

1. **Build-time panic** (`mujoco_pipeline.rs:~3990`): `compute_spatial_tendon_length0()`
   panics if a sidesite is inside a wrapping geometry at model build time. MuJoCo
   treats this as valid — "side sites can be placed inside the sphere or cylinder,
   which causes an inverse wrap: the tendon path is constrained to pass through the
   object instead of going around it."

2. **Runtime silent degradation** (`mujoco_pipeline.rs:~10072`): `sphere_wrap()` returns
   `WrapResult::NoWrap` when an endpoint is inside the sphere. During simulation,
   if joint motion causes a sidesite to enter a wrapping geometry, the tendon
   silently loses its wrapping instead of switching to the inverse wrap algorithm.

#### MuJoCo Reference

MuJoCo's `mju_wrap()` in `engine_util_misc.c` handles 3D→2D projection, then
dispatches between two 2D algorithms based on the sidesite's **3D distance** from
the wrapping geom center (in geom-local frame):

```c
// s = sidesite position in geom-local 3D frame
if (side && mju_norm3(s) < radius) {
    wlen = wrap_inside(pnt, d, radius);   // inverse wrap → wlen = 0
} else {
    wlen = wrap_circle(pnt, d, (side ? sd : NULL), radius);  // normal wrap → wlen = arc_length
}
```

**Key architectural facts:**

1. The inside check uses `mju_norm3(s)` — the full 3D norm — for both spheres
   and cylinders. For cylinders, the Z-component of the sidesite position
   contributes to the inside check, even though wrapping is 2D in XY. This means
   a sidesite inside the cylinder cross-section but far along Z (`norm3 > radius`)
   will NOT trigger inside wrap.

2. The sidesite controls **dispatch only**. Once inside `wrap_inside`, the sidesite
   is completely unused — only the two endpoint positions and the radius matter.

3. **Inverse wrap produces a single tangent point, not two.** Both output slots
   are identical (`pnt[0..1] == pnt[2..3]`). The arc length is **zero** (`wlen = 0`).
   The total tendon contribution is `|p0 − tangent| + 0 + |tangent − p1|` — the
   sum of two straight segments meeting at one point on the circle surface. By
   triangle inequality, this is always ≥ straight-line distance `|p0 − p1|`.

4. The existing caller code (`mj_fwd_tendon_spatial`, line ~9539) handles this
   correctly without modification: `total_length += dist1 + arc_length + dist3`
   with `arc_length = 0` gives `|p0−t| + |t−p1|`. The sub-segment 2 (arc)
   Jacobian contributes zero because both endpoints are on `geom_body`. The
   Jacobian for segments 1 and 3 uses the point-Jacobian approach (treating
   wrap points as fixed on the geom body), which is valid for inside wrap
   because the tangent point is a stationary point of the path-length function
   constrained to the circle — the first-order variation of the tangent position
   does not contribute to length variation (same principle as exterior wrap).
   **No caller changes are required.**

5. **`wrap_xpos`/`wrap_obj` data storage is out of scope.** MuJoCo stores
   tangent point world positions in `d->wrap_xpos[]` and object markers in
   `d->wrap_obj[]` for visualization. Our implementation does not populate
   these fields (they are not in our `Data` struct). This is already documented
   as a separate follow-up in the spatial tendon spec (future_work_2.md §Known
   Limitations). The physics (length, Jacobian, forces) is correct without them.

**Normal wrap** (`wrap_circle`): Tendon wraps around the exterior, producing two
distinct tangent points and a circular arc. Returns arc length. Already implemented
as `sphere_wrap()` / `cylinder_wrap()`.

**Inverse wrap** (`wrap_inside`): Tendon path passes *through* the geometry,
touching the circle surface at a single tangent point. Returns 0 (zero arc).
Operates in 2D (the wrapping plane), making it geometry-agnostic — both sphere
and cylinder use the same 2D algorithm after 3D→2D projection.

#### Objective

Implement MuJoCo's `wrap_inside` Newton solver for inverse tendon wrapping.
Remove the build-time panic. Handle runtime sidesite-inside transitions.

#### Specification

##### S1. 2D Newton solver: `wrap_inside_2d()`

A standalone 2D function implementing MuJoCo's `wrap_inside()` from
`engine_util_misc.c`. Given two 2D endpoint positions and a circle radius,
find the single tangent point for an inverse (through-geometry) wrap path.

**Constants:**
```
ε = 1e-15    (MuJoCo's mjMINVAL — used for all near-zero checks)
```

**Signature:**
```rust
fn wrap_inside_2d(
    end0: Vector2<f64>,  // endpoint 0 in 2D wrapping plane
    end1: Vector2<f64>,  // endpoint 1 in 2D wrapping plane
    radius: f64,
) -> Option<Vector2<f64>>
// Returns None (no wrap, tendon goes straight) or Some(tangent_point).
```

`Option<Vector2<f64>>` maps directly to MuJoCo's return convention:
`None` ↔ return `-1`, `Some(point)` ↔ return `0` with populated `pnt`.

**Algorithm:**

*Step 1 — Validate inputs:*
```
len0 = |end0|
len1 = |end1|
If len0 ≤ radius OR len1 ≤ radius OR radius < ε OR len0 < ε OR len1 < ε:
    return None
```

Note: the endpoint check uses `≤` (less-than-or-equal), matching MuJoCo's
`wrap_inside`. This differs from the existing `sphere_wrap` early exit which
uses `<` (strict less-than, matching MuJoCo's `wrap_circle`). Points exactly
on the circle are rejected by `wrap_inside` because the tangent formula
degenerates when `len = radius` (division `A = radius/len` yields 1).

*Step 2 — Segment-circle intersection check:*
```
dif = end1 - end0
dd = |dif|²
If dd > ε:
    a = -(dif · end0) / dd          // nearest-point parameter on segment
    If 0 < a < 1:
        nearest = end0 + a · dif
        If |nearest| ≤ radius:
            return None              // straight path crosses circle
```

*Step 3 — Compute Newton equation parameters:*
```
A = radius / len0
B = radius / len1
cosG = (len0² + len1² - dd) / (2 · len0 · len1)

If cosG < -1 + ε:  return None            // near-antiparallel: no solution
```

*Step 4 — Prepare default fallback:*
```
fallback = normalize(0.5 · (end0 + end1)) · radius
```
This projects the midpoint of the two endpoints onto the circle surface.
It is used whenever the Newton solver encounters numerical issues — matching
MuJoCo's behavior of returning `0` (success) with a best-effort tangent point
rather than `-1` (no wrap) for solver failures.

**Safety invariant:** The fallback is computed *after* the antiparallel check
(`cosG < -1 + ε → None`). When `end0 ≈ -end1`, the midpoint is near-zero
and `normalize` would produce NaN — but the antiparallel check returns `None`
first. This reorder vs. MuJoCo's C code (which computes the default before
the cosG check) is a deliberate Rust safety improvement that does not affect
outputs: MuJoCo never *uses* the default when cosG ≈ -1.

```
If cosG >  1 - ε:  return Some(fallback)   // near-parallel / coincident: use fallback

G = acos(cosG)
```

The Newton equation: `f(z) = asin(A·z) + asin(B·z) - 2·asin(z) + G = 0`

where `z ∈ (0, 1)` is the sine of the tangent angle parameter. Since `A < 1`
and `B < 1` (endpoints are outside the circle), the function is monotonically
decreasing in the valid domain, and the Newton solver starts near the right
boundary (`z ≈ 1`) and moves leftward.

**Geometric derivation of the Newton equation:**

The stationarity condition for path length `L = |P0−T| + |T−P1|` constrained
to the circle is `(û₀ + û₁) · τ = 0`, where `û₀ = (P0−T)/|P0−T|`,
`û₁ = (P1−T)/|P1−T|`, and `τ` is the circle tangent at T. This means the two
line segments make equal angles with the circle normal at T.

Let `θ_T` be this common angle (at T, between the inward radial and each
segment). Define `z = sin(θ_T)`. Consider triangle `(O, T, P0)` with sides
`OT = r`, `OP0 = len0`, angle `φ₀` at `O`. By the sine rule:

```
sin(ξ₀) / r = sin(θ_T) / len0    →    ξ₀ = asin(r · z / len0) = asin(A · z)
```

where `ξ₀` is the angle at `P0`. Since angles sum to `π`:
`φ₀ = π − θ_T − asin(A·z)`. Similarly for triangle `(O, T, P1)`:
`φ₁ = π − θ_T − asin(B·z)`.

The tangent point lies between `P0` and `P1` angularly, so `φ₀ + φ₁ = G`:

```
(π − θ_T − asin(A·z)) + (π − θ_T − asin(B·z)) = G
```

In the interior wrap, the angle at T is **obtuse** (`θ_T > π/2`): the side
`OP0 = len0 > r = OT`, and the largest angle in a triangle is opposite the
longest side. Since `z = sin(θ_T)` and `θ_T ∈ (π/2, π)`, the correct
inversion is `θ_T = π − asin(z)`. Substituting:

```
φ₀ = π − (π − asin(z)) − asin(A·z) = asin(z) − asin(A·z)
φ₁ = asin(z) − asin(B·z)

φ₀ + φ₁ = G  →  asin(A·z) + asin(B·z) − 2·asin(z) + G = 0    ∎
```

The reconstruction angle `asin(z) − asin(A·z) = φ₀` gives the angular offset
from the `P0` direction to `T` on the circle.

*Step 5 — Newton iteration:*
```
z = 1 - 1e-7                        // initial guess (near domain boundary)
f = asin(A·z) + asin(B·z) - 2·asin(z) + G

If f > 0:  return Some(fallback)     // init on wrong side of root

For iter = 0..20:
    If |f| < 1e-6:  break            // converged

    df = A / max(ε, sqrt(1 - z²·A²))
       + B / max(ε, sqrt(1 - z²·B²))
       - 2 / max(ε, sqrt(1 - z²))

    If df > -ε:  return Some(fallback)       // derivative non-negative; abort

    z_new = z - f / df

    If z_new > z:  return Some(fallback)     // solver moving rightward; abort

    z = z_new
    f = asin(A·z) + asin(B·z) - 2·asin(z) + G

    If f > 1e-6:  return Some(fallback)      // overshot positive; abort

If not converged after 20 iterations:
    return Some(fallback)
```

All five Newton guards (init-side, df-sign, leftward-only, overshoot, maxiter)
return `Some(fallback)` — **not** `None`. MuJoCo returns `0` (success with
fallback point) for these cases, not `-1` (no wrap). This means the tendon
still wraps using the best-effort fallback point.

*Step 6 — Geometric reconstruction:*

Compute the tangent point by rotating from one endpoint direction. The choice
of reference endpoint depends on the orientation of the triangle (end0, O, end1):
```
cross = end0.x · end1.y - end0.y · end1.x

If cross > 0:
    vec = normalize(end0)
    ang = asin(z) - asin(A · z)
Else:
    vec = normalize(end1)
    ang = asin(z) - asin(B · z)

tangent = radius · (cos(ang) · vec.x - sin(ang) · vec.y,
                    sin(ang) · vec.x + cos(ang) · vec.y)

return Some(tangent)
```

The 2×2 matrix is a standard counterclockwise rotation by `ang`. The angle
`asin(z) - asin(A·z)` is the angular offset from the endpoint direction to the
tangent point on the circle.

**`cross = 0` case:** When `cross = 0`, both endpoints are collinear with the
origin, meaning `G = 0` (parallel) or `G = π` (antiparallel). Both cases are
caught by earlier guards: `cosG > 1 - ε → Some(fallback)` (Step 4) and
`cosG < -1 + ε → None` (Step 3). Therefore the `cross` branch is unreachable
when both endpoints are exactly collinear with O. For near-collinear cases
(`cross ≈ 0` but not zero), the `Else` branch selects `end1` as reference;
both choices produce the same tangent point by continuity.

##### S2. Extract wrapping-plane helper

The existing `sphere_wrap` constructs the 2D wrapping plane inline (steps 3–4,
~20 LOC). The inside-wrap branch needs the same construction. Extract into a
shared helper:

```rust
/// Construct the 2D wrapping plane for sphere wrapping.
///
/// Returns (axis0, axis1) where axis0 = normalize(p1) and axis1 is
/// perpendicular to axis0 in the plane containing (origin, p1, p2).
/// Handles the collinear degenerate case (p1, O, p2 collinear).
fn sphere_wrapping_plane(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>)
```

**Algorithm:**
```
1. normal = p1 × p2
2. If |normal| < 1e-10 · |p1| · |p2|:          // relative threshold
     u = normalize(p1)
     Pick cardinal axis e_k with smallest |u · e_k|
     normal = u × e_k
3. normal = normalize(normal)
4. axis0 = normalize(p1)
5. axis1 = normal × axis0
Return (axis0, axis1)
```

The relative threshold `1e-10 · |p1| · |p2|` matches the existing `sphere_wrap`
code (line ~10097). The degenerate fallback (step 2) constructs an arbitrary
perpendicular plane through p1 when p1, O, p2 are collinear. The caller can
recover `plane_normal = axis0 × axis1` if needed (e.g., for the dual-candidate
algorithm in the existing outside-wrap path).

Both the existing outside-wrap path and the new inside-wrap path call this
helper. No behavioral change to the outside-wrap path.

##### S3. Integration into `sphere_wrap()`

**Insertion point:** immediately after the endpoint-inside early exit (step 2,
line ~10072), before the coincident-sites check and straight-line clearance
(steps 3–4). This matches MuJoCo's ordering where the sidesite dispatch in
`mju_wrap()` occurs before `wrap_circle`'s straight-line check:

```rust
fn sphere_wrap(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    radius: f64,
    sidesite: Option<Vector3<f64>>,
) -> WrapResult {
    // Existing early exits (radius ≤ 0, endpoint inside sphere)...

    // NEW: Check for sidesite inside sphere (inverse wrap).
    // Uses 3D norm matching MuJoCo's mju_norm3(s) < radius.
    if let Some(ss) = sidesite {
        if ss.norm() < radius {
            let (axis0, axis1) = sphere_wrapping_plane(p1, p2);
            let end0 = Vector2::new(p1.dot(&axis0), p1.dot(&axis1));
            let end1 = Vector2::new(p2.dot(&axis0), p2.dot(&axis1));

            return match wrap_inside_2d(end0, end1, radius) {
                None => WrapResult::NoWrap,
                Some(t2d) => {
                    // Reconstruct 3D: same projection as mju_wrap
                    let t3d = axis0 * t2d.x + axis1 * t2d.y;
                    WrapResult::Wrapped {
                        tangent_point_1: t3d,
                        tangent_point_2: t3d,  // identical — single tangent point
                        arc_length: 0.0,
                    }
                }
            };
        }
    }

    // Existing outside-wrap logic (unchanged)...
}
```

If `wrap_inside_2d` returns `None`, we return `NoWrap` — we do NOT fall through
to the outside-wrap algorithm. This matches MuJoCo, where `wlen < 0` from
`wrap_inside` causes `mju_wrap` to return `-1` without trying `wrap_circle`.

##### S4. Integration into `cylinder_wrap()`

Add an inside-wrap branch after the endpoint-inside early exit. The inside check
uses **3D norm** (matching MuJoCo), while the 2D solver uses XY projection
(matching the existing cylinder wrapping plane):

```rust
fn cylinder_wrap(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    radius: f64,
    sidesite: Option<Vector3<f64>>,
) -> WrapResult {
    let p1_xy = Vector2::new(p1.x, p1.y);
    let p2_xy = Vector2::new(p2.x, p2.y);

    // Existing early exits (radius ≤ 0, endpoint inside cross-section)...

    // NEW: Check for sidesite inside (3D norm, matching MuJoCo).
    //
    // Behavioral change from existing panic check: the panic used 2D XY norm,
    // so a sidesite inside the cylinder cross-section but far along Z
    // (e.g., ss=(0,0,5) with r=0.1, norm3=5.0 > r) currently panics but
    // will now correctly use normal wrap instead of inside wrap. This matches
    // MuJoCo's mju_norm3(s) < radius dispatch.
    if let Some(ss) = sidesite {
        if ss.norm() < radius {
            return match wrap_inside_2d(p1_xy, p2_xy, radius) {
                None => WrapResult::NoWrap,
                Some(t2d) => {
                    // Z interpolation: same formula as normal cylinder wrap
                    // with arc_length = 0. Both points get identical Z.
                    let l0 = (p1_xy - t2d).norm();
                    let l1 = (p2_xy - t2d).norm();
                    let total = l0 + l1;
                    let tz = if total > 1e-10 {
                        p1.z + (p2.z - p1.z) * l0 / total
                    } else {
                        0.5 * (p1.z + p2.z)
                    };
                    let t3d = Vector3::new(t2d.x, t2d.y, tz);
                    WrapResult::Wrapped {
                        tangent_point_1: t3d,
                        tangent_point_2: t3d,  // identical
                        arc_length: 0.0,       // no helical component
                    }
                }
            };
        }
    }

    // Existing outside-wrap logic (unchanged)...
}
```

MuJoCo verification for cylinder Z with `wlen=0`: `L0/(L0+0+L1)` and
`(L0+0)/(L0+0+L1)` produce the same ratio, so both Z values are identical.
The helical correction `sqrt(wlen² + height²) = sqrt(0+0) = 0` confirms
`arc_length = 0`.

##### S5. Remove build-time panic and retire validation rule 9

Delete the sidesite-inside validation loop from `compute_spatial_tendon_length0()`
(lines 3979–4016). This entire loop — from the `// Validate rule 9` comment through
the `panic!()` — should be removed. Sidesite inside wrapping geometry is a valid
MuJoCo configuration now handled by the runtime wrapping algorithms.

The remaining code (FK, tendon_length0 copy, lengthspring sentinel) is unchanged
and will correctly compute tendon lengths using the inside-wrap algorithm via the
FK → `mj_fwd_tendon` → `sphere_wrap`/`cylinder_wrap` → `wrap_inside_2d` path.

**Validation rule 9 retirement:** The spatial tendon spec (future_work_2.md
§4.2, lines ~2641–2647) defines rule 9: "sidesite outside wrapping geometry."
This rule was a stopgap because `wrap_inside` was not implemented. With
`wrap_inside` now implemented, rule 9 is **retired** — sidesites inside
wrapping geometry are valid. The following updates are required:

1. `mujoco_pipeline.rs` — delete the rule 9 validation loop (lines 3979–4016)
   and update the `compute_spatial_tendon_length0()` doc comment (lines 3957–3968)
   to remove the panic mention and the `#[allow(clippy::panic)]` attribute.
2. `future_work_2.md` — update rule 9 text (lines ~2641–2647) to note it is
   retired: "Rule 9 (sidesite outside wrapping geometry) is retired — sidesites
   inside wrapping geometry are now handled by the `wrap_inside` algorithm (§39)."
3. `future_work_2.md` — update the "Known Limitations" gap note (lines ~3700–3712)
   to mark `wrap_inside` as implemented with a cross-reference to §39.

##### S6. Degenerate cases

| Case | Behavior | MuJoCo return |
|------|----------|---------------|
| Endpoint inside circle (`\|p\| ≤ radius`) | `None` — tangent formula degenerates | `-1` |
| Segment crosses circle (nearest point `≤ radius`) | `None` — straight path already penetrates | `-1` |
| Endpoints near-antiparallel (`cosG < -1+ε`) | `None` — no geometric solution | `-1` |
| Endpoints near-parallel or coincident (`cosG > 1-ε`) | `Some(fallback)` — midpoint projected onto circle | `0` with default |
| Coincident endpoints (`dd ≤ ε`) | Segment check skipped → `cosG → 1` → `Some(fallback)` | `0` with default |
| Newton diverges / non-convergence | `Some(fallback)` — best-effort fallback point | `0` with default |
| Sidesite at center (`\|ss\| ≈ 0`) | Inside check triggers (`0 < radius`). Solver well-defined since `A,B < 1` | ✓ |
| Sidesite exactly at radius (`\|ss\| = radius`) | Inside check is strict `<`, so normal wrap is used | ✓ |
| No sidesite | Inside wrap never triggered; normal wrap proceeds | ✓ |

#### Acceptance Criteria

**T1. No build-time panic (sphere):** A model with sidesite deliberately placed
inside a wrapping sphere loads without panic. Simulate one step without crash.

**T2. No build-time panic (cylinder):** Same for sidesite inside a wrapping cylinder.

**T3. Single tangent point on surface:** With sidesite inside a sphere,
`sphere_wrap` returns `WrapResult::Wrapped` with `tangent_point_1 == tangent_point_2`
and `arc_length == 0.0`. The tangent point lies on the sphere surface
(`|tangent| = radius ± 1e-10`).

**T4. Path length correctness:** The tendon length for inside wrap equals
`|p0 − tangent| + |tangent − p1|` (sum of two straight segments, no arc).
Verify this value matches MuJoCo 3.5.0 reference for the same model.

**T5. Newton convergence and tangency verification:** For a reference geometry
with `end0 = (2.0, 0.0)`, `end1 = (1.5, 1.5·√3)` (distance exactly 3.0 from
center, angle G = π/3 exactly), `radius = 1.0`. Implementation must use
`(1.5, 1.5 * 3.0_f64.sqrt())` — not a truncated decimal — to preserve the
exact invariants:
- The Newton solver converges within 20 iterations.
- The tangent point lies on the circle (`|t| = 1.0 ± 1e-10`).
- The tangent point satisfies the stationarity (tangency) condition:
  `|τ · ((t−end0)/|t−end0| + (t−end1)/|t−end1|)| < 1e-8`
  where `τ` is the circle tangent vector (perpendicular to `t`) at the
  tangent point. This verifies the Newton solver found the correct
  geometric critical point.

**T6. MuJoCo conformance (sphere):** Compare tendon length and wrap point
positions against MuJoCo 3.5.0. Tolerance: `1e-6` for tendon length, `1e-6`
for wrap point positions. Test model:

```xml
<!-- T6: Sphere inside-wrap conformance model.
     Geometric constraint: both endpoint sites must be on the SAME angular
     side of the sphere so the straight line between them clears the sphere
     (does not cross the circle in 2D). This ensures wrap_inside_2d produces
     a valid tangent point rather than returning None.

     Sphere-local positions at qpos=0:
       s1 = (0.3, 0, 0.15),  |s1| = 0.335 > 0.15 (outside)
       s2 = (0.3, 0, -0.15), |s2| = 0.335 > 0.15 (outside)
       side = (0.05, 0, 0),   |side| = 0.05 < 0.15 (inside)
     Line nearest to origin: (0.3, 0, 0), distance = 0.3 > 0.15 (clears) -->
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_sphere" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <site name="side" pos="0.05 0 0"/>  <!-- inside sphere: |ss|=0.05 < 0.15 -->
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.01" mass="0.1"/>  <!-- mass for MuJoCo validity -->
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_inside">
      <site site="s1"/>
      <geom geom="wrap_sphere" sidesite="side"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

Reference values obtained via the extraction script in §39-appendix below.

**MuJoCo 3.5.0 reference values:**
```
T6_ten_length  = 0.424264068711929   # = 0.3·√2 (two segments of length 0.15·√2)
T6_tangent_pos = (0.15, 0.0, 0.0)   # on sphere surface, |t| = 0.15 = radius
                                      # Z = 0 exactly by endpoint symmetry about Z=0
```

**T7. MuJoCo conformance (cylinder):** Same for sidesite inside a wrapping
cylinder. Tolerance: `1e-6` for tendon length, `1e-6` for wrap point positions.
Test model:

```xml
<!-- T7: Cylinder inside-wrap conformance model.
     No euler rotation — cylinder axis is Z. Sites offset in X with slight Y
     spread so XY projections are NOT antiparallel (both point roughly +X).

     Cylinder-local positions at qpos=0:
       s1_xy = (0.3, 0.1),  |s1_xy| = 0.316 > 0.15 (outside cross-section)
       s2_xy = (0.3, -0.1), |s2_xy| = 0.316 > 0.15 (outside cross-section)
       side_c = (0.05, 0, 0), |side_c|_3D = 0.05 < 0.15 (inside by 3D norm)
     Line in XY nearest to origin: (0.3, 0), distance = 0.3 > 0.15 (clears) -->
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl" type="cylinder" size="0.15 0.5"/>
      <site name="s1" pos="0.3 0.1 0.2"/>
      <site name="side_c" pos="0.05 0 0"/>  <!-- inside cylinder: |ss|_3D=0.05 < 0.15 -->
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>  <!-- mass for MuJoCo validity -->
        <site name="s2" pos="0.3 -0.1 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_inside_cyl">
      <site site="s1"/>
      <geom geom="wrap_cyl" sidesite="side_c"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

**MuJoCo 3.5.0 reference values:**
```
T7_ten_length  = 0.538516480713450
T7_tangent_pos = (0.15, 0.0, 0.0)    # on cylinder surface, XY norm = 0.15 = radius
                                       # Y = 0, Z = 0 exactly: XY endpoints (0.3, ±0.1)
                                       # are symmetric about X-axis → l0 = l1 → tz = 0
```

**T8. Transition robustness and dispatch correctness:** As a sidesite sweeps
from outside to inside a wrapping sphere (via slide joint), verify:
1. No crash, panic, or NaN at any position throughout the sweep.
2. Correct dispatch: when `|ss_local| ≥ radius`, the result depends on
   sidesite/line geometry (may be NoWrap or outside wrap); when
   `|ss_local| < radius`, inside wrap is used.
3. MuJoCo conformance at sampled points on both sides of the boundary.

**Note on inherent discontinuity:** The transition at `|ss| = radius` is
inherently discontinuous in MuJoCo's algorithm. When the sidesite is outside
and the straight line clears the sphere, MuJoCo may return NoWrap (straight
line); when the sidesite is inside, `wrap_inside` produces a tangent-point
detour that is strictly longer by the triangle inequality. This is a deliberate
algorithm-dispatch boundary, not a bug. Our implementation must reproduce
MuJoCo's behavior exactly — including the discontinuity.

**Critical model constraint:** The sidesite must be on a **different body**
from the wrapping geom, connected through a joint, so that joint motion changes
the sidesite's distance from the geom center. (If sidesite and geom are on the
same body, they move rigidly together and the distance never changes.)

**Co-movement note:** In this model, endpoint `s2` is on the same child body
`b1` as the sidesite `side_sweep`. As the slide joint sweeps, **both** the
sidesite and `s2` translate by the same delta along X. This means the endpoint
positions in geom-local frame also change during the sweep — the test is not
a pure sidesite sweep with fixed endpoints. This is intentional and realistic:
in biomechanical models, the sidesite is typically a bone landmark on the same
body as a muscle attachment point. The reference values below account for this.

Test model uses a slide joint along X to translate child body `b1` (carrying
both `side_sweep` and `s2`) toward/away from the sphere center.
At `qpos=0` the sidesite is just outside (`|ss|=0.16`);
at `qpos=-0.01` it crosses the boundary (`|ss|=0.15=radius`); at `qpos=-0.05`
it is well inside (`|ss|=0.11`).

```xml
<!-- T8: Transition robustness and dispatch model.
     Sphere on parent body b0, sidesite on child body b1 with slide joint.
     The slide joint translates b1 along X, changing the sidesite's distance
     from the sphere center.

     At qpos=0:    side_sweep sphere-local = (0.16, 0, 0), |ss|=0.16 > 0.15 (outside)
     At qpos=-0.01: side_sweep sphere-local = (0.15, 0, 0), |ss|=0.15 = radius (boundary)
     At qpos=-0.05: side_sweep sphere-local = (0.11, 0, 0), |ss|=0.11 < 0.15 (inside)

     Both endpoint sites remain outside and non-collinear throughout the sweep:
       s1 = (0.3, 0, 0.15), |s1| = 0.335 (fixed on b0)
       s2 at qpos=0: (0.3, 0, -0.15), at qpos=-0.05: (0.25, 0, -0.15)
       Line nearest to origin stays > 0.25 > 0.15 throughout. -->
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_s" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <body name="b1">
        <joint name="sweep" type="slide" axis="1 0 0" range="-0.05 0.05"/>
        <geom type="sphere" size="0.01" mass="0.1"/>  <!-- mass for MuJoCo validity -->
        <site name="side_sweep" pos="0.16 0 0"/>  <!-- just outside at qpos=0 -->
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_sweep">
      <site site="s1"/>
      <geom geom="wrap_s" sidesite="side_sweep"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

**Procedure:** Sweep `qpos[0]` from `-0.05` to `+0.05` in steps of `0.001`,
call `forward()` at each step. For each step verify:
- `ten_length[0]` is finite and positive (no NaN, no negative, no panic)

Additionally, check MuJoCo conformance at five specific positions spanning the
dispatch boundary:
- `qpos = -0.04`: sidesite at `(0.12, 0, 0)`, `|ss|=0.12 < 0.15` → inside wrap
- `qpos = -0.02`: sidesite at `(0.14, 0, 0)`, `|ss|=0.14 < 0.15` → inside wrap (first point past boundary)
- `qpos = -0.01`: sidesite at `(0.15, 0, 0)`, `|ss|=0.15 = radius` → outside dispatch (strict `<` fails), NoWrap (straight line clears sphere)
- `qpos =  0.00`: sidesite at `(0.16, 0, 0)`, `|ss|=0.16 > 0.15` → NoWrap (straight line clears sphere, sidesite same side as closest point)
- `qpos = +0.04`: sidesite at `(0.20, 0, 0)`, `|ss|=0.20 > 0.15` → NoWrap (same reason)

Tolerance: `1e-6` for length conformance at the five sampled points.

**MuJoCo 3.5.0 reference values:**
```
T8_qpos_neg004_length = 0.397761312300161   # qpos=-0.04, |ss|=0.12 < 0.15 → inside wrap
T8_qpos_neg002_length = 0.410539632911676   # qpos=-0.02, |ss|=0.14 < 0.15 → inside wrap
T8_qpos_neg001_length = 0.300166620396073   # qpos=-0.01, |ss|=0.15 = radius → NoWrap
T8_qpos_zero_length   = 0.300000000000000   # qpos= 0.00, |ss|=0.16 > 0.15 → NoWrap (straight)
T8_qpos_pos004_length = 0.302654919008431   # qpos=+0.04, |ss|=0.20 > 0.15 → NoWrap (straight)
```

**Discontinuity confirmed:** Between `qpos=-0.01` (exact boundary, `|ss|=radius`,
outside dispatch → NoWrap) and `qpos=-0.02` (first inside-wrap point), the
tendon length jumps discontinuously. This is the expected dispatch-boundary
discontinuity — when the sidesite crosses inside, `wrap_inside` produces a
tangent-point detour strictly longer than the straight line (triangle inequality).
The two new boundary reference values (`T8_qpos_neg001_length`,
`T8_qpos_neg002_length`) verify that the implementation reproduces MuJoCo's
behavior at the exact transition. The full 101-point sweep
(`qpos ∈ [-0.05, +0.05]` at 0.001 steps) produces finite positive values
throughout — no NaN, no crash.

**T9. Regression:** All existing spatial tendon tests (Tests 1–19b) pass
unchanged. These all have sidesites outside wrapping geometry.

**T10. Jacobian correctness (sphere):** The tendon Jacobian (`ten_J`) is
correct for the sphere inverse wrap case. Verify via central finite difference:
perturb `qpos` by `±1e-7`, recompute tendon length, compare
`J_fd = (L(q+ε) − L(q−ε)) / (2ε)` against `J_an = ten_J · e_i`. Tolerance:
`|J_fd − J_an| < 1e-4 · max(|J_fd|, 1.0)` (mixed relative/absolute —
relative for large entries, absolute for near-zero entries where relative
error is ill-defined). Looser than the `1e-5` used for exterior wrap because
the tangent-point stationarity is satisfied to solver tolerance `1e-6`,
introducing slightly more numerical noise in the implicit derivative.

**T11. Degenerate: segment crosses circle:** Construct a geometry where the
straight line between endpoints intersects the circle (endpoints on opposite
sides). `wrap_inside_2d` returns `None`. The tendon goes straight.

**T12. Degenerate: endpoint at circle surface:** If an endpoint is exactly at
`|p| = radius` during inside wrap, `wrap_inside_2d` returns `None` (the `≤`
check rejects it). Verify no crash.

**T13. Fallback behavior:** Construct an adversarial near-degenerate geometry
where `A ≈ 1` and `B ≈ 1` (endpoints barely outside the circle). The Newton
solver may not converge or may hit a guard. Verify that `wrap_inside_2d`
returns `Some(fallback_point)` (NOT `None`), and that the fallback point lies
on the circle surface (`|point| = radius ± 1e-10`).

**T14. Boundary: sidesite exactly at radius:** Set `|sidesite| = radius`
exactly. The inside check (`< radius`) does NOT trigger, so normal (outside)
wrap is used. Verify no crash and that the tendon length is finite and positive.

**T15. Tangency + Z-interpolation (cylinder):** Verify the inside-wrap tangent
point for a cylinder with asymmetric Z displacement:
1. XY tangency: the tangent point in XY satisfies the stationarity condition
   (same as T5): `|τ · ((t−end0)/|t−end0| + (t−end1)/|t−end1|)| < 1e-8`.
2. Z interpolation: `tz = p1.z + (p2.z − p1.z) · L0 / (L0 + L1)` where
   `L0 = |p1_xy − t_xy|`, `L1 = |p2_xy − t_xy|`.
3. The interpolated Z must be **nonzero** (asymmetric model design ensures
   `L0 ≠ L1`), actually exercising the interpolation formula.

Test model (asymmetric Y → asymmetric L0/L1 → nonzero tz):

```xml
<!-- T15: Cylinder inside-wrap with asymmetric Z interpolation.
     XY asymmetry: s1_xy = (0.3, 0.05), s2_xy = (0.3, -0.15)
       |s1_xy| = 0.304 > 0.15 (outside), |s2_xy| = 0.335 > 0.15 (outside)
     Z asymmetry: s1.z = 0.3, s2.z = -0.1 (different magnitudes)
     Sidesite inside: |side|_3D = 0.05 < 0.15
     Line in XY: vertical at x=0.3 from y=0.05 to y=-0.15,
       nearest to origin = (0.3, 0), distance = 0.3 > 0.15 (clears)
     Because |0.05| ≠ |−0.15|, the 2D tangent point is NOT on the X-axis,
     so L0 ≠ L1, giving tz ≠ 0.5·(0.3 + (−0.1)) = 0.1. -->
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl_z" type="cylinder" size="0.15 0.5"/>
      <site name="s1_z" pos="0.3 0.05 0.3"/>
      <site name="side_z" pos="0.05 0 0"/>  <!-- |ss|_3D=0.05 < 0.15 -->
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2_z" pos="0.3 -0.15 -0.1"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_z_interp">
      <site site="s1_z"/>
      <geom geom="wrap_cyl_z" sidesite="side_z"/>
      <site site="s2_z"/>
    </spatial>
  </tendon>
</mujoco>
```

**MuJoCo 3.5.0 reference values:**
```
T15_ten_length  = 0.542346519424661
T15_tangent_pos = (0.148607047563419, -0.020394739872909, 0.117656368532109)
                                       # t_z = 0.1177 ≠ 0 — confirms asymmetric Z interpolation
```

Tolerance: `1e-6` for tendon length, `1e-6` for wrap point positions.

**T16. Cylinder 3D-norm dispatch:** Sidesite inside the cylinder cross-section
(XY norm = 0.05 < 0.1) but outside by 3D norm (`√(0.0025 + 25) ≈ 5.0 > 0.1`).
Verify that **normal (outside) wrap** is used, not inside wrap — the tendon
length must match the no-sidesite case exactly. This validates the behavioral
change from the current code (which uses 2D XY norm and panics) to the correct
MuJoCo behavior (`mju_norm3(s) < radius`).

```xml
<!-- T16: Cylinder 3D-norm dispatch model.
     Cylinder radius=0.1. Sidesite at (0.05, 0, 5.0):
       |ss_xy| = 0.05 < 0.1 (inside cross-section)
       |ss_3D| = sqrt(0.0025 + 25) ≈ 5.0 > 0.1 (outside by 3D norm)
     MuJoCo uses mju_norm3(s) < radius → 5.0 < 0.1 is false → wrap_circle.
     Straight line clears cylinder → NoWrap. Length = |s1 − s2|. -->
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl_3d" type="cylinder" size="0.1 1.0"/>
      <site name="s1" pos="0.3 0.05 0.2"/>
      <site name="side_3d" pos="0.05 0 5.0"/>  <!-- |xy|=0.05 < 0.1, |3D|≈5.0 > 0.1 -->
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 -0.05 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_3d_dispatch">
      <site site="s1"/>
      <geom geom="wrap_cyl_3d" sidesite="side_3d"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

**MuJoCo 3.5.0 reference values:**
```
T16_ten_length = 0.412310562561766   # = sqrt(0.1² + 0.4²) = |s1−s2| (NoWrap, straight line)
```

Tolerance: `1e-6` for tendon length. Verify the result is identical to
`cylinder_wrap` without sidesite-inside dispatch (same model with sidesite
removed from the tendon spec produces the same length).

**T17. Jacobian correctness (cylinder):** Same method as T10 (central finite
difference, perturb `qpos` by `±1e-7`) but for a cylinder with sidesite inside.
The cylinder case has Z-interpolation (`tz = p1.z + (p2.z − p1.z) · l0 / total`)
which introduces a different Jacobian structure than sphere. Tolerance:
`|J_fd − J_an| < 1e-4 · max(|J_fd|, 1.0)` (same mixed tolerance as T10).

**T18. Sidesite at origin (sphere):** Sidesite at `(0, 0, 0)` — exactly at the
sphere center. `|ss| = 0 < radius` → inside wrap triggers. The solver is
well-defined since `A, B < 1` (both endpoints are outside the circle). Verify
no panic, solver converges, tangent point on surface, and MuJoCo conformance.

**Sphere-only rationale:** The origin edge case tests the Newton solver's
numerical behavior when `|ss| = 0`, not geometry-specific dispatch. Since
`wrap_inside_2d` is shared by sphere and cylinder, and the dispatch path
difference (3D norm) is already exercised by T16, testing sphere alone is
sufficient.

```xml
<!-- T18: Sidesite at sphere center (origin edge case).
     Same endpoints as T6, sidesite moved to exact center.
     Sidesite |ss| = 0 < 0.15 → inside wrap.
     Result should match T6 exactly — sidesite only controls dispatch. -->
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_sphere" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <site name="side_origin" pos="0 0 0"/>  <!-- at center: |ss|=0 < 0.15 -->
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_origin">
      <site site="s1"/>
      <geom geom="wrap_sphere" sidesite="side_origin"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
```

**MuJoCo 3.5.0 reference values:**
```
T18_ten_length  = 0.424264068711929   # identical to T6 — sidesite only controls dispatch
T18_tangent_pos = (0.15, 0.0, 0.0)    # identical to T6
```

Tolerance: `1e-6` for tendon length, `1e-6` for wrap point positions.
Reference values obtained via the extraction script in §39-appendix below.

**Rotated-geom coverage note:** All inside-wrap test models (T6–T8, T15, T18) use
wrapping geoms at default orientation. This is sufficient because the inside-wrap
dispatch (`|ss_local| < radius`) and the 2D Newton solver operate entirely in
geom-local coordinates — the body/geom rotation cancels in the
`geom_xmat.T · (site_xpos − geom_xpos)` transform. The geom-local→world
reconstruction for Jacobian computation shares the same code path as exterior
wrap, which is exercised by the existing tests 1–19b (T9 regression).

#### Test Matrix

| # | Location | Geometry | Config | Validates |
|---|----------|----------|--------|-----------|
| T1 | Integration | Sphere | Sidesite inside, build-time | No panic |
| T2 | Integration | Cylinder | Sidesite inside, build-time | No panic |
| T3 | Integration | Sphere | Sidesite inside, runtime | Single tangent point, on-surface, arc=0 |
| T4 | Integration | Sphere | Sidesite inside, runtime | Path length = \|p0−t\| + \|t−p1\|, matches MuJoCo |
| T5 | **Unit** | 2D circle | Known 2D geometry | Newton convergence + tangency condition |
| T6 | Integration | Sphere | MuJoCo 3.5.0 ref model | Conformance: length + wrap points |
| T7 | Integration | Cylinder | MuJoCo 3.5.0 ref model | Conformance: length + wrap points |
| T8 | Integration | Sphere | Slide joint sweep | Robustness (no NaN/crash) + dispatch correctness + MuJoCo conformance at sampled points |
| T9 | Integration | Both | Existing tests 1–19b | Regression |
| T10 | Integration | Sphere | Sidesite inside | Jacobian via central finite difference |
| T11 | **Unit** | 2D circle | Straddling endpoints | Segment-circle intersection → None |
| T12 | **Unit** | 2D circle | Endpoint at \|p\|=radius | No crash, returns None |
| T13 | **Unit** | 2D circle | Near-degenerate A≈1, B≈1 | Fallback point on circle |
| T14 | Integration | Sphere | Sidesite at \|ss\|=radius | Normal wrap, no crash |
| T15 | Integration | Cylinder | Asymmetric-Z endpoints (dedicated model) | XY tangency + nonzero Z interpolation |
| T16 | Integration | Cylinder | Sidesite inside XY, \|ss\|₃D > r | 3D norm dispatch matches MuJoCo |
| T17 | Integration | Cylinder | Sidesite inside, finite diff | Jacobian correctness (Z-interpolation) |
| T18 | Integration | Sphere | Sidesite at (0,0,0), MuJoCo 3.5.0 ref | Origin edge case, solver convergence, MuJoCo conformance |

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs`:
  - New `wrap_inside_2d()` function (2D Newton solver, ~80 LOC)
  - New `sphere_wrapping_plane()` helper (extracted from `sphere_wrap`, ~20 LOC)
  - Modify `sphere_wrap()`: refactor plane construction to use helper, add inside-wrap
    branch (~20 LOC net new)
  - Modify `cylinder_wrap()`: add inside-wrap branch (~25 LOC)
  - Delete sidesite-inside validation in `compute_spatial_tendon_length0()` (~30 LOC removed)
  - New `#[cfg(test)] mod wrap_inside_tests` — unit tests T5, T11, T12, T13 that
    call `wrap_inside_2d` directly with known 2D geometries. These must be in-file
    because `wrap_inside_2d` is a private function.
- `sim/L0/tests/integration/spatial_tendons.rs`:
  - New test models (sphere-inside, cylinder-inside, transition sweep,
    3D-norm dispatch, origin sidesite)
  - Integration tests T1–T4, T6–T10, T14–T18 (MJCF model → forward → check outputs)

#### Documentation Updates (post-implementation)

After implementation is complete and all tests pass, update the following:

1. **`sim/docs/todo/future_work_2.md`** — Spatial tendon spec:
   - Lines ~2641–2647: Annotate validation rule 9 as retired (§39).
   - Lines ~3700–3712: Mark `wrap_inside` gap as resolved with `~~strikethrough~~`
     and cross-reference to §39.
2. **`sim/docs/MUJOCO_GAP_ANALYSIS.md`** — Line ~28:
   - Strike through `wrap_inside` in the Phase 3A-v entry: `~~wrap_inside~~ ✅`.
3. **`sim/docs/todo/index.md`** — Line ~124:
   - Update §39 status column to `✅`.
4. **`sim/docs/todo/future_work_10.md`** — This section:
   - Update status from "Not started" to "✅ Complete".

#### §39-appendix: MuJoCo Reference Value Extraction Script

This script reproduces the reference values above. Run with `uv run --with mujoco --with numpy python3 script.py`.
The models include `<geom mass="0.1"/>` on child bodies (MuJoCo requires mass on moving bodies).

```python
#!/usr/bin/env python3
"""Extract MuJoCo reference values for §39 wrap_inside conformance tests.
   Verified against MuJoCo 3.5.0. Run: uv run --with mujoco --with numpy python3 <script>
"""
import mujoco
import numpy as np

def extract(name, xml, qpos_overrides=None):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if qpos_overrides:
        for idx, val in qpos_overrides.items():
            d.qpos[idx] = val
    mujoco.mj_forward(m, d)
    length = d.ten_length[0]
    nwrap = d.ten_wrapnum[0]
    wrapadr = d.ten_wrapadr[0]
    print("\n--- %s ---" % name)
    print("  ten_length  = %.15f" % length)
    print("  ten_wrapnum = %d" % nwrap)
    max_idx = min(nwrap, d.wrap_xpos.shape[0] - wrapadr)
    for i in range(max_idx):
        pos = d.wrap_xpos[wrapadr + i]
        print("  wrap_xpos[%d] = (%.15f, %.15f, %.15f)" % (i, pos[0], pos[1], pos[2]))
    return length

T6_XML = """
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_sphere" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <site name="side" pos="0.05 0 0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_inside">
      <site site="s1"/>
      <geom geom="wrap_sphere" sidesite="side"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"""
extract("T6 (sphere inside-wrap)", T6_XML)

T7_XML = """
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl" type="cylinder" size="0.15 0.5"/>
      <site name="s1" pos="0.3 0.1 0.2"/>
      <site name="side_c" pos="0.05 0 0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 -0.1 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_inside_cyl">
      <site site="s1"/>
      <geom geom="wrap_cyl" sidesite="side_c"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"""
extract("T7 (cylinder inside-wrap)", T7_XML)

T15_XML = """
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl_z" type="cylinder" size="0.15 0.5"/>
      <site name="s1_z" pos="0.3 0.05 0.3"/>
      <site name="side_z" pos="0.05 0 0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2_z" pos="0.3 -0.15 -0.1"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_z_interp">
      <site site="s1_z"/>
      <geom geom="wrap_cyl_z" sidesite="side_z"/>
      <site site="s2_z"/>
    </spatial>
  </tendon>
</mujoco>
"""
extract("T15 (cylinder Z-interpolation)", T15_XML)

T16_XML = """
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_cyl_3d" type="cylinder" size="0.1 1.0"/>
      <site name="s1" pos="0.3 0.05 0.2"/>
      <site name="side_3d" pos="0.05 0 5.0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 -0.05 -0.2"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_3d_dispatch">
      <site site="s1"/>
      <geom geom="wrap_cyl_3d" sidesite="side_3d"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"""
extract("T16 (cylinder 3D-norm dispatch)", T16_XML)

T18_XML = """
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_sphere" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <site name="side_origin" pos="0 0 0"/>
      <body name="b1">
        <joint name="j1" type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_origin">
      <site site="s1"/>
      <geom geom="wrap_sphere" sidesite="side_origin"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"""
extract("T18 (sidesite at origin)", T18_XML)

T8_XML = """
<mujoco>
  <worldbody>
    <body name="b0">
      <geom name="wrap_s" type="sphere" size="0.15"/>
      <site name="s1" pos="0.3 0 0.15"/>
      <body name="b1">
        <joint name="sweep" type="slide" axis="1 0 0" range="-0.05 0.05"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
        <site name="side_sweep" pos="0.16 0 0"/>
        <site name="s2" pos="0.3 0 -0.15"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t_sweep">
      <site site="s1"/>
      <geom geom="wrap_s" sidesite="side_sweep"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"""
for qval, label in [(-0.04, "inside"), (-0.02, "boundary inside"),
                     (-0.01, "boundary exact"), (0.0, "outside boundary"),
                     (0.04, "well outside")]:
    extract("T8 qpos=%.2f (%s)" % (qval, label), T8_XML, {0: qval})

print("\n--- T8 full sweep ---")
m = mujoco.MjModel.from_xml_string(T8_XML)
d = mujoco.MjData(m)
all_ok = True
for q_milli in range(-50, 51):
    q = q_milli * 0.001
    d.qpos[0] = q
    mujoco.mj_forward(m, d)
    L = d.ten_length[0]
    ok = np.isfinite(L) and L > 0
    if not ok:
        all_ok = False
        print("  FAIL qpos=%.3f  ten_length=%.10f" % (q, L))
    elif q_milli % 10 == 0:
        print("  OK   qpos=%.3f  ten_length=%.10f" % (q, L))
print("Sweep: %s" % ("ALL OK" if all_ok else "FAILURES DETECTED"))
```

Reference values above were extracted using this script on MuJoCo 3.5.0.

---

### 40. Fluid / Aerodynamic Forces (Two-Model System)
**Status:** ✅ Complete | **Effort:** L | **Prerequisites:** None

#### Current State

Both fluid models (inertia-box and ellipsoid) are fully implemented and verified
against MuJoCo 3.5.0. The MJCF parser handles `fluidshape` and `fluidcoef`
attributes on `<geom>` with default class inheritance. Build-time precomputation
populates `geom_fluid[12]` per geom (virtual mass/inertia via 15-point
Gauss-Kronrod quadrature). Runtime dispatch in `mj_fluid()` routes each body
to the correct model. 42 conformance tests pass (38 exact match at 1e-10,
T13 within 0.3%, T15/T26 internal-only, T20 trajectory within 1e-6).

#### MuJoCo Reference

MuJoCo implements fluid forces in `engine_passive.c`, gated by
`density > 0 || viscosity > 0`. Two models exist, dispatched **per-body**
(not per-geom):

| Model | Dispatch condition | Scope | Force components |
|-------|-------------------|-------|------------------|
| **Inertia-box** (legacy) | No child geom has `geom_fluid[0] > 0` | Per-body (uses body inertia box) | Quadratic drag + viscous resistance |
| **Ellipsoid** (advanced) | Any child geom has `geom_fluid[0] > 0` | Per-geom within body | Added mass (gyroscopic) + Magnus lift + Kutta lift + combined linear drag + combined angular drag |

**Key dispatch rule:** If **any** geom on a body has `geom_fluid[0] > 0`
(i.e., `fluidshape="ellipsoid"`), the entire body uses the ellipsoid model.
The inertia-box model is **not** applied for that body. Within the ellipsoid
model, individual geoms with `geom_fluid[0] == 0` are skipped.

Forces are stored in a separate `d->qfrc_fluid` buffer (nv-length), then added
to `d->qfrc_passive` after computation. This separation supports derivative
computation and sleep filtering.

##### Constants

```
mjMINVAL = 1e-15    // guard for near-zero denominators (used in drag, Kutta, etc.)
mjEPS    = 1e-14    // guard for virtual inertia degenerate case
```

##### Data layout: `geom_fluid` array

MuJoCo stores `mjNFLUID = 12` elements per geom in `m->geom_fluid`:

| Index | Field | Set by | Default |
|-------|-------|--------|---------|
| 0 | `geom_interaction_coef` (master switch) | compiler: 1.0 if `fluidshape="ellipsoid"`, 0.0 if `"none"` | 0.0 |
| 1 | `blunt_drag_coef` (C_blunt) | MJCF `fluidcoef[0]` | 0.5 |
| 2 | `slender_drag_coef` (C_slender) | MJCF `fluidcoef[1]` | 0.25 |
| 3 | `ang_drag_coef` (C_ang) | MJCF `fluidcoef[2]` | 1.5 |
| 4 | `kutta_lift_coef` (C_K) | MJCF `fluidcoef[3]` | 1.0 |
| 5 | `magnus_lift_coef` (C_M) | MJCF `fluidcoef[4]` | 1.0 |
| 6–8 | `virtual_mass[3]` | compiler: from ellipsoid potential flow | 0.0 |
| 9–11 | `virtual_inertia[3]` | compiler: from ellipsoid potential flow | 0.0 |

For geoms with `fluidshape="none"` (default): all 12 elements are 0.0.
For geoms with `fluidshape="ellipsoid"`: element 0 = 1.0, elements 1–5 from
`fluidcoef` (or defaults), elements 6–11 from build-time virtual mass/inertia
computation.

##### Object velocity helper (`mj_objectVelocity` equivalent)

Both fluid models require the 6D velocity at a point in a local frame. MuJoCo
provides `mj_objectVelocity(m, d, objtype, id, res, flg_local)`. Our codebase
stores `cvel[body]` as `[ω, v]` — a spatial motion vector where `v` is the
linear velocity at `xpos[body_id]` (the body origin in world frame).

**Convention note:** MuJoCo's `cvel` stores velocity at `subtree_com[rootid]`
via `cdof` vectors (pre-computed DOF Jacobians referenced to the subtree root
center of mass). Our `mj_fwd_velocity` instead propagates velocities using
xpos-to-xpos offsets, so `cvel[3:6]` gives velocity at `xpos[body_id]`.
Both conventions give identical results at any query point — the lever arm
formula `v_P = v_Q + ω × (P - Q)` is reference-point invariant — but the
lever arm in `object_velocity_local` must use `xpos[body_id]`, not
`subtree_com[rootid]`.

Our implementation simplifies the MuJoCo API by having callers pass the
target point and rotation matrix directly (avoiding ObjType dispatch):

```
object_velocity_local(model, data, body_id, point, rot) → [f64; 6]:
    // Static body early-out: world body (id=0) has mass=0 and is caught
    // by the dispatch loop's mass guard. For extra safety, return zeros
    // for body 0.
    if body_id == 0: return [0; 6]

    // Extract spatial velocity at body origin (world frame)
    ω = cvel[body_id][0..3]                         // angular velocity (world frame)
    v_origin = cvel[body_id][3..6]                   // linear velocity at xpos[body_id]

    // Shift linear velocity from body origin to requested point
    v_point = v_origin + ω × (point - data.xpos[body_id])

    // Rotate to local frame
    ω_local = rot^T * ω
    v_local = rot^T * v_point
    return [ω_local; v_local]
```

Callers pass the appropriate point and rotation:
- **Inertia-box model:** `object_velocity_local(m, d, body_id, xipos[body_id], ximat[body_id])`
- **Ellipsoid model:** `object_velocity_local(m, d, geom_body, geom_xpos[gid], geom_xmat[gid])`

**Frame note (inertia-box model):** The inertia-box model operates entirely
in the inertia frame (`ximat`/`xipos`). The velocity is computed at `xipos`
and rotated by `ximat`. Wind is also rotated by `ximat`, and forces are
rotated back to world using `ximat`. The application point is `xipos` (body
CoM). All three frame operations use the same matrix — there is no frame
inconsistency in MuJoCo's implementation.

##### Inertia-Box Model (`mj_inertiaBoxFluidModel`)

Called once per body. The world body (id=0) and bodies with
`body_mass < mjMINVAL` are already filtered by the dispatch loop (S5).

**1. Equivalent box from diagonal inertia** (full side lengths):
```
box[0] = sqrt(max(mjMINVAL, I_y + I_z - I_x) / mass * 6)
box[1] = sqrt(max(mjMINVAL, I_x + I_z - I_y) / mass * 6)
box[2] = sqrt(max(mjMINVAL, I_x + I_y - I_z) / mass * 6)
```
Note: `max` is applied to the inertia difference only, before dividing by mass.
Where `I_x, I_y, I_z` = `body_inertia[body_id]` (principal diagonal).

**2. Local 6D velocity** at body CoM in inertia frame:
```
lvel = object_velocity_local(model, data, Body, body_id)   // [ω₀,ω₁,ω₂, v₀,v₁,v₂]
```

**3. Wind subtraction** (translational only, rotate world wind to inertia frame):
```
v_wind_local = ximat[body_id]^T * model.wind
lvel[3..6] -= v_wind_local
```

**4. Quadratic drag** (density-dependent, per-axis in local frame):
```
lfrc[3+i] -= 0.5 * ρ * box[j] * box[k] * |lvel[3+i]| * lvel[3+i]
lfrc[i]   -= ρ * box[i] * (box[j]⁴ + box[k]⁴) / 64 * |lvel[i]| * lvel[i]
```
Where `(i,j,k)` cycles through `(0,1,2), (1,2,0), (2,0,1)`,
`ρ = model.density`.

**5. Viscous resistance** (viscosity-dependent):
```
diam = (box[0] + box[1] + box[2]) / 3          // equivalent sphere diameter
lfrc[3+i] -= 3 * π * diam * β * lvel[3+i]      // linear Stokes drag
lfrc[i]   -= π * diam³ * β * lvel[i]            // angular Stokes drag
```
Where `β = model.viscosity`.

**6. Force accumulation:**
```
bfrc = ximat[body_id] * lfrc                    // rotate to world frame (uses ximat)
mj_apply_ft(bfrc[3..6], bfrc[0..3], xipos[body_id], body_id, qfrc_fluid)
```

##### Ellipsoid Model (`mj_ellipsoidFluidModel`)

Called once per body. Iterates over the body's child geoms. Skips geoms with
`geom_fluid[0] == 0.0`.

For each qualifying geom:

**1. Unpack 12 coefficients** via `readFluidGeomInteraction`:
```
interaction_coef, C_blunt, C_slender, C_ang, C_K, C_M,
virtual_mass[3], virtual_inertia[3]
```

**2. Compute semi-axes** via `fluid_geom_semi_axes(geom_type, geom_size)`:

| Geom Type | Semi-axes `s = (s₀, s₁, s₂)` |
|-----------|-------------------------------|
| Sphere    | `(size[0], size[0], size[0])` |
| Capsule   | `(size[0], size[0], size[1] + size[0])` |
| Cylinder  | `(size[0], size[0], size[1])` |
| Default (ellipsoid, box, mesh) | `(size[0], size[1], size[2])` |

Note: MuJoCo's function is `mju_geomSemiAxes` (not `mju_geom2Ellipsoid`).

**3. Local 6D velocity** at geom center in geom frame:
```
lvel = object_velocity_local(model, data, Geom, geom_id)
v_wind_local = geom_xmat[gid]^T * model.wind
lvel[3..6] -= v_wind_local
```
Notation: `ω = lvel[0..3]` (angular), `v = lvel[3..6]` (linear).

**4. Initialize `lfrc = [0; 6]`, then accumulate 5 force components:**

**Component 1 — Added mass (gyroscopic only):**
MuJoCo calls `mj_addedMassForces` with `accels = NULL` (acceleration-dependent
terms `-m_A·v̇` and `-I_A·ω̇` are disabled due to circular dependency with
`qacc`). Only gyroscopic cross-product terms are active:
```
p_v[i] = ρ * virtual_mass[i] * v[i]         // virtual linear momentum
L_v[i] = ρ * virtual_inertia[i] * ω[i]      // virtual angular momentum
lfrc[3..6] += p_v × ω                        // added mass force
lfrc[0..3] += p_v × v                        // added mass torque 1
lfrc[0..3] += L_v × ω                        // added mass torque 2
```

**Component 2 — Magnus lift** (spin-induced):
```
volume = (4/3) * π * s₀ * s₁ * s₂
f_magnus[i] = C_M * ρ * volume * (ω × v)[i]
lfrc[3..6] += f_magnus
```

**Component 3 — Kutta lift** (circulation from asymmetric cross-section):
```
// Unnormalized ellipsoid gradient weighted by velocity
norm[i] = (s_j * s_k)² * v[i]        for (i,j,k) cyclic

// Projected ellipsoid cross-section area normal to velocity
proj_denom = (s₁s₂)⁴v₀² + (s₂s₀)⁴v₁² + (s₀s₁)⁴v₂²
proj_num   = (s₁s₂v₀)²  + (s₂s₀v₁)²  + (s₀s₁v₂)²
A_proj     = π * sqrt(proj_denom / max(mjMINVAL, proj_num))

// Angle between velocity and surface normal
cos_alpha  = proj_num / max(mjMINVAL, |v| * proj_denom)

// Double cross product: ((norm × v) × v), scaled
kutta_circ = C_K * ρ * cos_alpha * A_proj * (norm × v)
f_kutta    = kutta_circ × v
lfrc[3..6] += f_kutta
```
For spheres `(s₀=s₁=s₂)`, `norm ∝ v` so `norm × v = 0` and Kutta vanishes.
When `v = 0`, `norm = [0,0,0]` and `norm × v = 0`, so Kutta vanishes
naturally — no explicit speed guard is needed (the `mjMINVAL` denominators
prevent division by zero).

**Component 4 — Combined linear drag** (viscous + quadratic in one coefficient):
```
eq_D  = (2/3) * (s₀ + s₁ + s₂)               // equivalent sphere diameter
d_max = max(s₀, s₁, s₂)
d_min = min(s₀, s₁, s₂)
d_mid = s₀ + s₁ + s₂ - d_max - d_min
A_max = π * d_max * d_mid                      // maximum cross-section area

drag_lin_coef = β * 3π * eq_D
              + ρ * |v| * (A_proj * C_blunt + C_slender * (A_max - A_proj))
lfrc[3+i] -= drag_lin_coef * v[i]
```

**Component 5 — Combined angular drag** (viscous + quadratic in one coefficient):
```
// Per-axis moment: MuJoCo's mji_ellipsoid_max_moment(size, dir)
// Uses size[dir] (the axis itself) times max(other_two)^4
II[i] = (8/15) * π * s[i] * max(s[(i+1)%3], s[(i+2)%3])⁴

// Global maximum moment (uses global d_mid, d_max)
I_max = (8/15) * π * d_mid * d_max⁴

// Weighted angular momentum
mom_visc[i] = ω[i] * (C_ang * II[i] + C_slender * (I_max - II[i]))

// Scalar drag coefficient (norm collapses per-axis to single value)
drag_ang_coef = β * π * eq_D³ + ρ * |mom_visc|
lfrc[i] -= drag_ang_coef * ω[i]
```

**5. Scale and accumulate:**
```
lfrc *= interaction_coef                        // master coefficient (usually 1.0)
bfrc = geom_xmat[gid] * lfrc                   // rotate to world frame
mj_apply_ft(bfrc[3..6], bfrc[0..3], geom_xpos[gid], body_id, qfrc_fluid)
```
##### Virtual Mass/Inertia Precomputation

MuJoCo precomputes elements 6–11 of `geom_fluid` at model build time using
potential flow theory for ellipsoids (Lamb's hydrodynamics). The computation
uses **15-point Gauss-Kronrod quadrature** to evaluate the kappa integrals.

**Kappa integral** (one per axis, with cyclic permutation of `d_x, d_y, d_z`):
```
κ_x = d_x·d_y·d_z * ∫₀^∞ dl / [(d_x²+l)^(3/2) · (d_y²+l)^(1/2) · (d_z²+l)^(1/2)]
```

MuJoCo applies the change of variables `l = x³/(1-x)²` to map `[0,∞)` to
`[0,1]`, with Jacobian `dl/dx = x²(3-x)/(1-x)³`. Rather than evaluating the
integrand directly, MuJoCo precomputes the transformed node positions
(`kronrod_l`), the Jacobian values at those nodes (`kronrod_d`), and the
standard 15-point Gauss-Kronrod weights (`kronrod_w`) as compile-time constants.

A non-dimensionalization scale factor `scale = (d_x³·d_y·d_z)^(2/5)` is
applied for numerical stability across the wide range of integration points:

```
fn get_added_mass_kappa(dx: f64, dy: f64, dz: f64) -> f64 {
    let inv_dx2 = 1.0 / (dx * dx);
    let inv_dy2 = 1.0 / (dy * dy);
    let inv_dz2 = 1.0 / (dz * dz);
    let scale = (dx * dx * dx * dy * dz).powf(0.4);  // (dx³·dy·dz)^(2/5)

    let mut kappa = 0.0;
    for i in 0..15 {
        let lambda = scale * KRONROD_L[i];
        let denom = (1.0 + lambda * inv_dx2)
            * ((1.0 + lambda * inv_dx2) * (1.0 + lambda * inv_dy2)
               * (1.0 + lambda * inv_dz2)).sqrt();
        kappa += scale * KRONROD_D[i] / denom * KRONROD_W[i];
    }
    kappa * inv_dx2
}
```

Three kappa values via cyclic permutation:
- `κ_x = get_added_mass_kappa(d_x, d_y, d_z)`
- `κ_y = get_added_mass_kappa(d_y, d_z, d_x)`
- `κ_z = get_added_mass_kappa(d_z, d_x, d_y)`

**15-point Gauss-Kronrod constants** (from MuJoCo `user_objects.cc`):

```rust
/// Pre-transformed integration nodes: l_i = x_i³ / (1 - x_i)²
const KRONROD_L: [f64; 15] = [
    7.865151709349917e-08, 1.7347976913907274e-05, 3.548008144506193e-04,
    2.846636252924549e-03, 1.4094260903596077e-02, 5.3063261727396636e-02,
    1.7041978741317773e-01, 5.0e-01, 1.4036301548686991e+00, 3.9353484827022642e+00,
    1.1644841677041734e+01, 3.953187807410903e+01, 1.775711362220801e+02,
    1.4294772912937397e+03, 5.4087416549217705e+04,
];

/// Gauss-Kronrod integration weights
const KRONROD_W: [f64; 15] = [
    0.01146766, 0.03154605, 0.05239501, 0.07032663, 0.08450236,
    0.09517529, 0.10221647, 0.10474107, 0.10221647, 0.09517529,
    0.08450236, 0.07032663, 0.05239501, 0.03154605, 0.01146766,
];

/// Jacobian dl/dx evaluated at each node: x²(3-x)/(1-x)³
const KRONROD_D: [f64; 15] = [
    5.538677720489877e-05, 2.080868285293228e-03, 1.6514126520723166e-02,
    7.261900344370877e-02, 2.3985243401862602e-01, 6.868318249020725e-01,
    1.8551129519182894e+00, 5.0e+00, 1.4060031152313941e+01, 4.328941239611009e+01,
    1.5658546376397112e+02, 7.479826085305024e+02, 5.827404295002712e+03,
    1.167540197944512e+05, 2.5482945327264845e+07,
];
```

**Virtual mass:**
```
virtual_mass[i] = V * κ_i / max(mjEPS, 2 - κ_i)
where V = (4/3)π · d_x · d_y · d_z
```

**Virtual inertia:**
```
I_x_fac = (d_y² - d_z²)² · |κ_z - κ_y|
        / max(mjEPS, |2(d_y² - d_z²) + (d_y² + d_z²)(κ_y - κ_z)|)
virtual_inertia[i] = V · I_i_fac / 5
```
(with cyclic permutations for y, z axes). Note: uses `mjEPS` (1e-14), not
`mjMINVAL` (1e-15), matching MuJoCo's `GetAddedMassKappa`.

**Special case — sphere** (`d_x = d_y = d_z = r`): `κ_i = 2/3`,
`virtual_mass[i] = V/2`, `virtual_inertia[i] = 0` (degenerate 0/0 → 0 via
the `max(mjEPS, ...)` guard).

#### Objective

Implement both MuJoCo fluid models (inertia-box and ellipsoid) in the passive
force pipeline with bit-exact conformance against MuJoCo 3.5.0.

#### Specification

##### S1. MJCF parsing

Add to `MjcfGeom`:
```rust
pub fluidshape: Option<FluidShape>,      // "none" | "ellipsoid"
pub fluidcoef: Option<[f64; 5]>,         // [C_blunt, C_slender, C_ang, C_K, C_M]
```

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FluidShape {
    #[default]
    None,       // Inertia-box model (body-level)
    Ellipsoid,  // Ellipsoid model (per-geom)
}
```

Parse from `<geom>` element. Inherit from `<default>` class (both `fluidshape`
and `fluidcoef` appear in the MJCF schema under `<default><geom>`).

Validation:
- `fluidshape` must be `"none"` or `"ellipsoid"`. Reject any other value
  with a parse error (matches MuJoCo, which rejects invalid enum strings).
- If `fluidcoef` is provided, all 5 values must be present (MuJoCo rejects
  partial `fluidcoef`). Fewer than 5 values → parse error.
- MuJoCo does not enforce non-negativity on `fluidcoef` values at parse
  time — arbitrary `f64` values are accepted.

**Edge case:** If `fluidcoef` is provided but `fluidshape="none"` (or
defaulted to `"none"`), the coefficients are parsed but have no effect —
`geom_fluid` remains all zeros because `geom_fluid[0] = 0.0` (the master
switch is off). This matches MuJoCo: `SetFluidCoefs` only writes elements
1–5 when `fluidshape="ellipsoid"`.

##### S2. Model storage

Add to `Model`:
```rust
pub geom_fluid: Vec<[f64; 12]>,    // mjNFLUID=12 per geom (see data layout table)
```

The 12-element layout matches MuJoCo exactly (see `geom_fluid` array table in
MuJoCo Reference above). Elements 0–5 come from MJCF parsing; elements 6–11
from build-time precomputation. For geoms with `fluidshape="none"`: all 12
elements are 0.0. For geoms with `fluidshape="ellipsoid"`: element 0 = 1.0,
elements 1–5 from `fluidcoef` (or defaults `[0.5, 0.25, 1.5, 1.0, 1.0]`),
elements 6–11 from virtual mass/inertia computation.

##### S3. Build-time precomputation

In `model_builder.rs`, after geom sizes are resolved:

1. For each geom with `fluidshape == Ellipsoid`:
   a. Compute semi-axes via `fluid_geom_semi_axes(geom_type, geom_size)`.
   b. Compute three kappa values via `get_added_mass_kappa()` using 15-point
      Gauss-Kronrod quadrature (nodes, weights, and Jacobian derivatives are
      the `const` arrays above, matching MuJoCo's `GetAddedMassKappa`).
   c. Compute `virtual_mass[i] = V * κ_i / max(mjEPS, 2 - κ_i)`.
   d. Compute `virtual_inertia[i] = V * I_fac_i / 5` (see formula above).
   e. Pack all 12 elements into `geom_fluid[geom_id]`.

2. For each geom with `fluidshape == None`: set `geom_fluid[geom_id] = [0.0; 12]`.

##### S4. Data fields

Add to `Data`:
```rust
pub qfrc_fluid: DVector<f64>,    // nv-length, zeroed each step
```

In `mj_fwd_passive`, zero `qfrc_fluid` unconditionally (before calling
`mj_fluid`), then add it to `qfrc_passive` if `mj_fluid` returns true:
```rust
data.qfrc_fluid.fill(0.0);           // zero unconditionally (matches MuJoCo)
if mj_fluid(model, data) {
    data.qfrc_passive += &data.qfrc_fluid;
}
```

This separation is needed for derivative computation (future S9) and matches
MuJoCo's `d->qfrc_fluid`.
##### S5. Fluid force dispatch (`mj_fluid`)

```rust
fn mj_fluid(model: &Model, data: &mut Data) -> bool {
    if model.density == 0.0 && model.viscosity == 0.0 { return false; }

    // Note: qfrc_fluid is zeroed by the caller (mj_fwd_passive) before
    // calling mj_fluid, matching MuJoCo's mj_passive which zeros
    // qfrc_fluid at the top level. This ensures the buffer is clean even
    // if mj_fluid returns false (early exit on zero density+viscosity).

    // Iterate over all bodies. World body (id=0) has mass=0 and is skipped
    // by the mass < mjMINVAL check below. Sleep filtering is deferred to §41.
    for body_id in 0..model.nbody {
        // Mass guard — applies to BOTH models (matches MuJoCo)
        if model.body_mass[body_id] < MJ_MINVAL { continue; }

        // Dispatch: does any child geom have geom_fluid[0] > 0?
        // Note: bodies with zero geoms (geom_num == 0) always take the
        // inertia-box path — the empty iterator returns false. This
        // matches MuJoCo, where such bodies still get drag based on
        // their composite body inertia.
        let geom_adr = model.body_geom_adr[body_id];
        let geom_num = model.body_geom_num[body_id];
        let use_ellipsoid = (geom_adr..geom_adr + geom_num)
            .any(|gid| model.geom_fluid[gid][0] > 0.0);

        if use_ellipsoid {
            mj_ellipsoid_fluid(model, data, body_id);
        } else {
            mj_inertia_box_fluid(model, data, body_id);
        }
    }

    true
}
```

**Sleep filtering note:** MuJoCo gates the body loop on `mjENBL_SLEEP`:
```c
int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < m->nbody;
int nbody = sleep_filter ? d->nbody_awake : m->nbody;
for (int b=0; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;
```
If the codebase does not yet implement sleep filtering for passive forces,
this can be deferred — but the body iteration must be structurally compatible
so sleep filtering can be added later without refactoring the dispatch loop.

Called from `mj_fwd_passive()`. If it returns `true`, add `qfrc_fluid` to
`qfrc_passive`.

**Disable-flag interaction (§41):** In MuJoCo, `mj_passive` returns early
when both `mjDSBL_SPRING` and `mjDSBL_DAMPER` are set — this skips fluid
forces even when `density > 0 || viscosity > 0`. Until §41 wires disable
flags, this doesn't affect us (flags default to enabled). Once §41 lands,
the `mj_fwd_passive` call-site must gate fluid computation identically:
fluid forces are only computed when at least one of spring/damper is enabled.

##### S6. Inertia-box implementation (`mj_inertia_box_fluid`)

```rust
const MJ_MINVAL: f64 = 1e-15;

fn mj_inertia_box_fluid(model: &Model, data: &mut Data, body_id: usize) {
    let rho = model.density;
    let beta = model.viscosity;
    let mass = model.body_mass[body_id];
    let inertia = model.body_inertia[body_id]; // Vector3: diagonal

    // 1. Equivalent box dimensions (full side lengths)
    let b = Vector3::new(
        ((inertia.y + inertia.z - inertia.x).max(MJ_MINVAL) / mass * 6.0).sqrt(),
        ((inertia.x + inertia.z - inertia.y).max(MJ_MINVAL) / mass * 6.0).sqrt(),
        ((inertia.x + inertia.y - inertia.z).max(MJ_MINVAL) / mass * 6.0).sqrt(),
    );

    // 2. Local 6D velocity [ω₀,ω₁,ω₂, v₀,v₁,v₂] at body CoM in inertia frame
    let mut lvel = object_velocity_local(
        model, data, body_id, &data.xipos[body_id], &data.ximat[body_id],
    );

    // 3. Subtract wind (translational only, rotate world wind to inertia frame)
    //    MuJoCo uses ximat (inertia frame) consistently for the entire
    //    inertia-box model: velocity, wind, and force rotation.
    let ximat = data.ximat[body_id];
    let wind_local = ximat.transpose() * model.wind;
    lvel[3] -= wind_local.x;
    lvel[4] -= wind_local.y;
    lvel[5] -= wind_local.z;

    // 4. Compute local forces
    let mut lfrc = [0.0f64; 6];

    // Viscous resistance (viscosity) — computed first via mji_scl3 (assignment)
    if beta > 0.0 {
        let diam = (b.x + b.y + b.z) / 3.0;
        let d3 = diam * diam * diam;
        for i in 0..3 { lfrc[i]   = -PI * d3 * beta * lvel[i]; }
        for i in 0..3 { lfrc[3+i] = -3.0 * PI * diam * beta * lvel[3+i]; }
    }

    // Quadratic drag (density) — accumulated onto viscous result
    if rho > 0.0 {
        lfrc[3] -= 0.5 * rho * b.y * b.z * lvel[3].abs() * lvel[3];
        lfrc[4] -= 0.5 * rho * b.x * b.z * lvel[4].abs() * lvel[4];
        lfrc[5] -= 0.5 * rho * b.x * b.y * lvel[5].abs() * lvel[5];

        lfrc[0] -= rho*b.x*(b.y.powi(4)+b.z.powi(4))/64.0 * lvel[0].abs()*lvel[0];
        lfrc[1] -= rho*b.y*(b.x.powi(4)+b.z.powi(4))/64.0 * lvel[1].abs()*lvel[1];
        lfrc[2] -= rho*b.z*(b.x.powi(4)+b.y.powi(4))/64.0 * lvel[2].abs()*lvel[2];
    }

    // 5. Rotate to world frame (uses ximat) and apply at body CoM
    let bfrc = rotate_spatial_to_world(&ximat, &lfrc);
    mj_apply_ft(
        model, &data.xpos, &data.xquat,
        &Vector3::new(bfrc[3], bfrc[4], bfrc[5]),  // force
        &Vector3::new(bfrc[0], bfrc[1], bfrc[2]),  // torque
        &data.xipos[body_id],                        // application point
        body_id,
        &mut data.qfrc_fluid,
    );
}
```

**MuJoCo source conformance notes (S6):**
- Guards use `> 0.0` (not `!= 0.0`), matching MuJoCo's `> 0` checks.
- MuJoCo computes viscous first via `mji_scl3` (assignment, not `+=`), then
  density-dependent drag is subtracted from the result. When `lfrc` is
  pre-zeroed, accumulation order doesn't affect the result — but the
  assignment-then-subtract pattern matches MuJoCo's code structure.
- All frame operations (velocity, wind, force rotation) use `ximat`.

##### S7. Ellipsoid implementation (`mj_ellipsoid_fluid`)

```rust
fn mj_ellipsoid_fluid(model: &Model, data: &mut Data, body_id: usize) {
    let rho = model.density;
    let beta = model.viscosity;

    let geom_adr = model.body_geom_adr[body_id];
    let geom_num = model.body_geom_num[body_id];

    for gid in geom_adr..geom_adr + geom_num {
        let fluid = model.geom_fluid[gid];
        let interaction_coef = fluid[0];
        if interaction_coef == 0.0 { continue; }

        // Unpack coefficients
        let (c_blunt, c_slender, c_ang) = (fluid[1], fluid[2], fluid[3]);
        let (c_kutta, c_magnus)         = (fluid[4], fluid[5]);
        let vmass    = [fluid[6], fluid[7], fluid[8]];
        let vinertia = [fluid[9], fluid[10], fluid[11]];

        // Semi-axes
        let s = fluid_geom_semi_axes(model.geom_type[gid], &model.geom_size[gid]);

        // Local velocity at geom center in geom frame
        let geom_body = model.geom_body[gid];
        let mut lvel = object_velocity_local(
            model, data, geom_body, &data.geom_xpos[gid], &data.geom_xmat[gid],
        );
        let gxmat = data.geom_xmat[gid];
        let wind_local = gxmat.transpose() * model.wind;
        lvel[3] -= wind_local.x;
        lvel[4] -= wind_local.y;
        lvel[5] -= wind_local.z;

        let (w, v) = (&lvel[0..3], &lvel[3..6]);  // angular, linear
        let mut lfrc = [0.0f64; 6];
        let speed = norm3(v);

        // ── Component 1: Added mass (gyroscopic, accels=NULL) ──
        // MuJoCo calls mj_addedMassForces unconditionally (no density guard).
        // When rho=0, virtual momenta are zero and cross products vanish.
        let pv = [rho*vmass[0]*v[0], rho*vmass[1]*v[1], rho*vmass[2]*v[2]];
        let lv = [rho*vinertia[0]*w[0], rho*vinertia[1]*w[1],
                   rho*vinertia[2]*w[2]];
        add_cross3(&mut lfrc[3..6], &pv, w);  // force  += p_v × ω
        add_cross3(&mut lfrc[0..3], &pv, v);  // torque += p_v × v
        add_cross3(&mut lfrc[0..3], &lv, w);  // torque += L_v × ω

        // ── Component 2: Magnus lift ──
        // MuJoCo calls mj_viscousForces unconditionally — all five viscous
        // sub-components (Magnus, Kutta, linear drag, angular drag) are
        // computed without density or speed guards. When rho=0 or speed=0,
        // the ρ· or |v|· multipliers zero out the relevant terms naturally.
        let vol = (4.0/3.0) * PI * s[0] * s[1] * s[2];
        let mag = cross3(w, v);
        for i in 0..3 { lfrc[3+i] += c_magnus * rho * vol * mag[i]; }

        // ── Component 3: Kutta lift ──
        // Projected area and cos_alpha (reused by component 4).
        // No speed guard — MuJoCo computes unconditionally. When v=0:
        // norm=[0,0,0], proj_denom=0, proj_num=0, and mjMINVAL denominators
        // prevent division by zero. Kutta force vanishes via norm×v = 0.
        let norm_vec = [
            (s[1]*s[2]).powi(2) * v[0],
            (s[2]*s[0]).powi(2) * v[1],
            (s[0]*s[1]).powi(2) * v[2],
        ];
        let proj_denom = (s[1]*s[2]).powi(4)*v[0]*v[0]
                       + (s[2]*s[0]).powi(4)*v[1]*v[1]
                       + (s[0]*s[1]).powi(4)*v[2]*v[2];
        let proj_num = (s[1]*s[2]*v[0]).powi(2)
                     + (s[2]*s[0]*v[1]).powi(2)
                     + (s[0]*s[1]*v[2]).powi(2);
        let a_proj = PI * (proj_denom / proj_num.max(MJ_MINVAL)).sqrt();
        let cos_alpha = proj_num / (speed * proj_denom).max(MJ_MINVAL);

        let mut circ = cross3(&norm_vec, v);
        let kutta_scale = c_kutta * rho * cos_alpha * a_proj;
        for i in 0..3 { circ[i] *= kutta_scale; }
        let kf = cross3(&circ, v);
        for i in 0..3 { lfrc[3+i] += kf[i]; }

        // ── Component 4: Combined linear drag ──
        let eq_d = (2.0/3.0) * (s[0] + s[1] + s[2]);
        let d_max = s[0].max(s[1]).max(s[2]);
        let d_min = s[0].min(s[1]).min(s[2]);
        let d_mid = s[0] + s[1] + s[2] - d_max - d_min;
        let a_max = PI * d_max * d_mid;

        let drag_lin = beta * 3.0 * PI * eq_d
            + rho * speed * (a_proj * c_blunt + c_slender * (a_max - a_proj));
        for i in 0..3 { lfrc[3+i] -= drag_lin * v[i]; }

        // ── Component 5: Combined angular drag ──
        let i_max = (8.0/15.0) * PI * d_mid * d_max.powi(4);
        let ii = [
            ellipsoid_moment(s, 0),
            ellipsoid_moment(s, 1),
            ellipsoid_moment(s, 2),
        ];
        let mom_visc = [
            w[0] * (c_ang*ii[0] + c_slender*(i_max - ii[0])),
            w[1] * (c_ang*ii[1] + c_slender*(i_max - ii[1])),
            w[2] * (c_ang*ii[2] + c_slender*(i_max - ii[2])),
        ];
        let drag_ang = beta * PI * eq_d.powi(3) + rho * norm3(&mom_visc);
        for i in 0..3 { lfrc[i] -= drag_ang * w[i]; }

        // ── Scale by interaction coefficient and accumulate ──
        for i in 0..6 { lfrc[i] *= interaction_coef; }
        let bfrc = rotate_spatial_to_world(&gxmat, &lfrc);
        mj_apply_ft(
            model, &data.xpos, &data.xquat,
            &Vector3::new(bfrc[3], bfrc[4], bfrc[5]),
            &Vector3::new(bfrc[0], bfrc[1], bfrc[2]),
            &data.geom_xpos[gid],
            body_id,
            &mut data.qfrc_fluid,
        );
    }
}
```

**MuJoCo source conformance notes (S7):**
- MuJoCo calls `mj_addedMassForces` and `mj_viscousForces` unconditionally —
  no `if rho != 0.0` or `if speed > MJ_MINVAL` guards exist in the source.
  All zero-velocity and zero-density cases resolve naturally via multipliers.
- The `mjMINVAL` denominators in `A_proj` and `cos_alpha` prevent division by
  zero when `speed == 0` or `proj_num == 0`.
- For spheres `(s₀=s₁=s₂)`: `norm ∝ v`, so `norm × v = 0` and Kutta vanishes.

**Helper: `ellipsoid_moment(s, axis)`** — per-axis moment for angular drag.
MuJoCo's `mji_ellipsoid_max_moment(size, dir)` uses `s[dir]` (the semi-axis
itself) times `max(other_two_semi_axes)^4`:

```rust
fn ellipsoid_moment(s: &[f64; 3], axis: usize) -> f64 {
    let d1 = s[(axis + 1) % 3];
    let d2 = s[(axis + 2) % 3];
    (8.0 / 15.0) * PI * s[axis] * d1.max(d2).powi(4)
}
```

##### S8. Force accumulation (`mj_apply_ft`)

Already exists in the codebase (`mujoco_pipeline.rs:9767`). It projects a 6D
force/torque at a world-frame point through the body Jacobian into generalized
forces. No changes needed — reuse directly for fluid force accumulation.

**Math helpers** used in pseudocode above (standard operations, not separately
specified):
- `cross3(a, b) → [f64; 3]` — cross product
- `add_cross3(out, a, b)` — `out += a × b`
- `norm3(v) → f64` — Euclidean norm of 3-vector
- `rotate_spatial_to_world(R, lfrc) → [f64; 6]` — rotate 6D spatial vector:

```rust
fn rotate_spatial_to_world(xmat: &Matrix3<f64>, lfrc: &[f64; 6]) -> [f64; 6] {
    let torque = xmat * Vector3::new(lfrc[0], lfrc[1], lfrc[2]);
    let force  = xmat * Vector3::new(lfrc[3], lfrc[4], lfrc[5]);
    [torque.x, torque.y, torque.z, force.x, force.y, force.z]
}
```

##### S9. Implicit integration derivatives

MuJoCo provides full velocity-derivatives of fluid forces in `mjd_passive_vel`
for implicit integration stability. This contributes 6×6 body-level coupling
blocks to the D matrix via `addJTBJSparse`.

**✅ Implemented** in §40a. Analytical velocity derivatives for both fluid models
are now integrated into `mjd_passive_vel`. 31 tests pass including MuJoCo 3.5.0
conformance. The `qfrc_fluid` separation (S4) enabled derivatives to be cleanly
added without modifying the forward force path.
#### Test Plan

##### Test MJCF Models

**Model A — Inertia-box sphere (quadratic drag):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

**Model B — Inertia-box sphere (Stokes viscous):**
```xml
<mujoco>
  <option density="0" viscosity="1.0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.01" mass="0.001"/>
    </body>
  </worldbody>
</mujoco>
```

**Model C — Inertia-box wind on box:**
```xml
<mujoco>
  <option density="1.2" viscosity="0" wind="5 0 0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

**Model D — Ellipsoid sphere:**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model E — Ellipsoid non-spherical (Kutta + anisotropic drag):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model F — Custom fluidcoef (zero Magnus/Kutta):**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
            fluidshape="ellipsoid" fluidcoef="1.0 0.5 2.0 0.0 0.0"/>
    </body>
  </worldbody>
</mujoco>
```

**Model G — Zero fluid (regression):**
```xml
<mujoco>
  <option density="0" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

**Model H — Mixed geoms (body dispatch test):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.05" mass="0.5"/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="0.5"
            fluidshape="ellipsoid" pos="0.2 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

**Model I — Inertia-box non-spherical (multi-geom body):**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="capsule" size="0.05 0.15" mass="0.5" pos="-0.1 0 0"/>
      <geom type="capsule" size="0.05 0.15" mass="0.5" pos="0.1 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

**Model J — Ellipsoid with both density and viscosity:**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model K — Multi-body (independent per-body dispatch):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body name="box_body" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body>
    <body name="ellipsoid_body" pos="1 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model L — Default class inheritance:**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <default>
    <geom fluidshape="ellipsoid"/>
  </default>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

**Model M — Ellipsoid cylinder (semi-axes variant):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="cylinder" size="0.05 0.15" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model N — Inertia-box with non-trivial body_ipos (ximat != xmat):**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.05 0.025" mass="1" pos="0.1 0.05 0"/>
    </body>
  </worldbody>
</mujoco>
```

**Model O — Ellipsoid with zero-mass body (mass guard on ellipsoid path):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.001" mass="1e-20"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model P — Ellipsoid viscosity-only (density=0):**
```xml
<mujoco>
  <option density="0" viscosity="1.0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model Q — Inertia-box with non-axis-aligned wind:**
```xml
<mujoco>
  <option density="1.2" viscosity="0" wind="3 4 0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

**Model R — Ellipsoid capsule (semi-axes variant):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="capsule" size="0.05 0.15" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model S — Multi-ellipsoid body (per-geom accumulation):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="0.5"
            fluidshape="ellipsoid" pos="-0.15 0 0"/>
      <geom type="ellipsoid" size="0.08 0.04 0.03" mass="0.5"
            fluidshape="ellipsoid" pos="0.15 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

**Model T — Ellipsoid angular-only velocity (angular drag isolation):**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model U — Oblate spheroid (kappa validation variant):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.1 0.03" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model V — Ellipsoid with wind (wind + ellipsoid path):**
```xml
<mujoco>
  <option density="1.2" viscosity="0" wind="3 0 0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model W — Inertia-box angular velocity (angular drag path):**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.05 0.025" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

**Model X — fluidcoef without fluidshape=ellipsoid (edge case):**
```xml
<mujoco>
  <option density="1.2" viscosity="0"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"
            fluidcoef="1.0 0.5 2.0 0.5 0.5"/>
    </body>
  </worldbody>
</mujoco>
```

**Model Y — Rotated body (non-identity initial orientation):**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <worldbody>
    <body pos="0 0 1" quat="0.707 0 0.707 0">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**Model Z — Inertia-box with near-zero mass body (mass guard):**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.001" mass="1e-20"/>
    </body>
  </worldbody>
</mujoco>
```

**Model AA — Default class inheritance with fluidcoef:**
```xml
<mujoco>
  <option density="1.2" viscosity="0.001"/>
  <default>
    <geom fluidshape="ellipsoid" fluidcoef="1.0 0.5 2.0 0.5 0.5"/>
  </default>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"/>
    </body>
  </worldbody>
</mujoco>
```

##### Acceptance Tests

All conformance tests compare against MuJoCo 3.5.0 reference values extracted
via the script in the appendix. Tolerance: `1e-10` for force components
(matching MuJoCo's double-precision arithmetic).

| # | Type | Model | Description | Verification |
|---|------|-------|-------------|--------------|
| T1 | Integration | G | Zero fluid regression | `qfrc_fluid == 0`, existing passive force tests still pass (no regression from adding fluid pipeline) |
| T2 | Integration | A | Inertia-box quadratic drag, sphere `v=(0,0,-1)` | `qfrc_fluid` matches MuJoCo reference |
| T3 | Integration | B | Inertia-box Stokes drag, small sphere `v=(1,0,0)` | `qfrc_fluid` matches MuJoCo; verify `≈ -6πβrv` analytically |
| T4 | Integration | C | Inertia-box wind force, stationary box | `qfrc_fluid` matches MuJoCo reference |
| T5 | Integration | A | Inertia-box equivalent box dimensions from diagonal inertia | Verify `box[i]` computation against known sphere: `box = 2r` for uniform sphere |
| T6 | Integration | D | Ellipsoid drag on sphere, `v=(0,0,-1)` | `qfrc_fluid` matches MuJoCo reference |
| T7 | Integration | D | Ellipsoid Magnus lift, `ω=(0,0,10)`, `v=(1,0,0)` | Lateral force `f_y` matches MuJoCo; verify `∝ C_M·ρ·V·(ω×v)` |
| T8 | Integration | E | Ellipsoid Kutta lift on non-spherical body, `v=(1,0.5,0)` | `qfrc_fluid` matches MuJoCo reference; non-zero Kutta contribution |
| T9 | Integration | D | Ellipsoid added mass gyroscopic, `ω=(5,0,0)`, `v=(0,0,-1)` | Gyroscopic cross-product force matches MuJoCo |
| T10 | Integration | F | Custom fluidcoef, zeroed Magnus/Kutta | `qfrc_fluid` matches MuJoCo; verify no lift forces |
| T11 | Integration | H | Body dispatch: mixed geoms → entire body uses ellipsoid | Only ellipsoid geom contributes; inertia-box NOT applied |
| T12 | Integration | H | Body dispatch: none-geom skipped | Verified implicitly by T11: if `qfrc_fluid` matches MuJoCo, the none-geom contributed zero. Additionally assert `geom_fluid[0] == 0.0` for the sphere geom. |
| T13 | Integration | I | Inertia-box non-spherical body (anisotropic drag) | `qfrc_fluid` matches MuJoCo; verify direction-dependent drag |
| T14 | Integration | J | Ellipsoid combined viscous + quadratic drag | `qfrc_fluid` matches MuJoCo with both density and viscosity active |
| T15 | Integration | Z | Body mass guard (inertia-box path) | Body with `mass < mjMINVAL`: zero fluid force on inertia-box path |
| T16 | Unit | — | `fluid_geom_semi_axes` for all geom types | Sphere, capsule, cylinder, ellipsoid, box: verify against table |
| T17 | Unit | — | `get_added_mass_kappa` for sphere | `κ = 2/3`; `virtual_mass = V/2`; `virtual_inertia = 0` |
| T18 | Unit | — | `ellipsoid_moment` for known geometry | `(8/15)π·s[axis]·max(other_two)⁴` verified against manual computation |
| T19 | Unit | — | Kappa quadrature for prolate spheroid `(1, 1, 3)` | Compare against closed-form elliptic integral result |
| T20 | Integration | A, J | 50-step trajectory conformance | Step 50 frames, compare `qfrc_fluid` at every 10th step against MuJoCo |
| T21 | Integration | K | Multi-body: independent dispatch per body | Body 1 uses inertia-box, body 2 uses ellipsoid; both match MuJoCo |
| T22 | Integration | L | Default class inheritance of `fluidshape` | Geom inherits `fluidshape="ellipsoid"` from default; `geom_fluid[0] == 1.0` |
| T23 | Integration | M | Ellipsoid cylinder semi-axes `(r, r, h)` | `qfrc_fluid` matches MuJoCo; verify semi-axes = `(size[0], size[0], size[1])` |
| T24 | Unit | — | Gauss-Kronrod constants validation | Verify `KRONROD_L[7] == 0.5`, `KRONROD_D[7] == 5.0`, `KRONROD_W` sums to `≈1.0` |
| T25 | Integration | N | Inertia-box with ximat != xmat (geom offset) | `qfrc_fluid` matches MuJoCo; verifies ximat used for wind/force rotation |
| T26 | Integration | O | Mass guard on ellipsoid path | Body with `mass < mjMINVAL` and `fluidshape="ellipsoid"`: zero fluid force |
| T27 | Integration | P | Ellipsoid viscosity-only (density=0) | `qfrc_fluid` matches MuJoCo; added mass/Magnus/Kutta all zero, only viscous drag active |
| T28 | Integration | Q | Inertia-box with non-axis-aligned wind | `qfrc_fluid` matches MuJoCo; stresses wind rotation logic |
| T29 | Integration | R | Ellipsoid capsule semi-axes `(r, r, h+r)` | `qfrc_fluid` matches MuJoCo; verify semi-axes = `(size[0], size[0], size[1]+size[0])` |
| T30 | Integration | D | Ellipsoid with zero velocity (no speed guard) | `qvel=0`, `qfrc_fluid` matches MuJoCo; validates no spurious speed guard |
| T31 | Integration | S | Multi-ellipsoid body: per-geom force accumulation | Two ellipsoid geoms on one body; `qfrc_fluid` matches MuJoCo (sum of per-geom contributions) |
| T32 | Integration | T | Ellipsoid angular-only velocity `ω=(10,5,3)`, `v=0` | Angular drag active, Magnus/Kutta/linear drag vanish; `qfrc_fluid` matches MuJoCo |
| T33 | Integration | U | Oblate spheroid drag, `v=(1,0.5,0)` | `qfrc_fluid` matches MuJoCo; validates kappa quadrature for oblate geometry |
| T34 | Unit | — | Kappa quadrature for oblate spheroid `(3, 3, 1)` | Compare against closed-form oblate elliptic integral result |
| T35 | Integration | V | Ellipsoid with wind, stationary body | `qfrc_fluid` matches MuJoCo; exercises `geom_xmat^T * wind` path (distinct from inertia-box `ximat^T * wind`) |
| T36 | Integration | W | Inertia-box angular velocity `ω=(10,5,3)`, `v=(0,0,-1)` | `qfrc_fluid` matches MuJoCo; exercises angular quadratic drag `ρ·b[i]·(b[j]⁴+b[k]⁴)/64` and angular Stokes drag |
| T37 | Integration | X | fluidcoef without fluidshape=ellipsoid | `geom_fluid == [0.0; 12]`; `qfrc_fluid` matches inertia-box model (fluidcoef ignored) |
| T38 | Parse | — | Invalid `fluidshape` value rejected | `fluidshape="box"` → parse error |
| T39 | Parse | — | Partial `fluidcoef` rejected | `fluidcoef="0.5 0.25"` (2 of 5 values) → parse error |
| T40 | Integration | G | Zero fluid: `qfrc_fluid` buffer exists and is zeroed | `qfrc_fluid` all zeros; `qfrc_passive` unchanged from non-fluid baseline |
| T41 | Integration | Y | Rotated body (non-identity initial orientation) | `qfrc_fluid` matches MuJoCo; stresses velocity/wind rotation with non-identity `geom_xmat` and `ximat` at rest |
| T42 | Integration | AA | Default class inheritance of `fluidcoef` | Geom inherits both `fluidshape` and `fluidcoef` from default; `geom_fluid[1:6]` matches custom coefficients `[1.0, 0.5, 2.0, 0.5, 0.5]`; `qfrc_fluid` matches MuJoCo reference |

##### MuJoCo Reference Value Extraction Script

```python
"""Extract MuJoCo reference values for §40 fluid force conformance tests.

Run with: python extract_fluid_refs.py
Requires: mujoco >= 3.5.0
"""
import mujoco
import numpy as np

MODELS = {
    "A": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.1" mass="1"/>
            </body></worldbody></mujoco>""",
    "B": """<mujoco><option density="0" viscosity="1.0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.01" mass="0.001"/>
            </body></worldbody></mujoco>""",
    "C": """<mujoco><option density="1.2" viscosity="0" wind="5 0 0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="box" size="0.1 0.05 0.025" mass="1"/>
            </body></worldbody></mujoco>""",
    "D": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.1" mass="1" fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "E": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "F": """<mujoco><option density="1.2" viscosity="0.001"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
                  fluidshape="ellipsoid" fluidcoef="1.0 0.5 2.0 0.0 0.0"/>
            </body></worldbody></mujoco>""",
    "G": """<mujoco><option density="0" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.1" mass="1"/>
            </body></worldbody></mujoco>""",
    "H": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.05" mass="0.5"/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="0.5"
                  fluidshape="ellipsoid" pos="0.2 0 0"/>
            </body></worldbody></mujoco>""",
    "I": """<mujoco><option density="1.2" viscosity="0.001"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="capsule" size="0.05 0.15" mass="0.5" pos="-0.1 0 0"/>
            <geom type="capsule" size="0.05 0.15" mass="0.5" pos="0.1 0 0"/>
            </body></worldbody></mujoco>""",
    "J": """<mujoco><option density="1.2" viscosity="0.001"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "K": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody>
            <body name="box_body" pos="0 0 1"><freejoint/>
            <geom type="box" size="0.1 0.05 0.025" mass="1"/>
            </body>
            <body name="ellipsoid_body" pos="1 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "L": """<mujoco><option density="1.2" viscosity="0"/>
            <default><geom fluidshape="ellipsoid"/></default>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.1" mass="1"/>
            </body></worldbody></mujoco>""",
    "M": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="cylinder" size="0.05 0.15" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "N": """<mujoco><option density="1.2" viscosity="0.001"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="box" size="0.1 0.05 0.025" mass="1" pos="0.1 0.05 0"/>
            </body></worldbody></mujoco>""",
    "P": """<mujoco><option density="0" viscosity="1.0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "Q": """<mujoco><option density="1.2" viscosity="0" wind="3 4 0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="box" size="0.1 0.05 0.025" mass="1"/>
            </body></worldbody></mujoco>""",
    "R": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="capsule" size="0.05 0.15" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "S": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="0.5"
                  fluidshape="ellipsoid" pos="-0.15 0 0"/>
            <geom type="ellipsoid" size="0.08 0.04 0.03" mass="0.5"
                  fluidshape="ellipsoid" pos="0.15 0 0"/>
            </body></worldbody></mujoco>""",
    "T": """<mujoco><option density="1.2" viscosity="0.001"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "U": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.1 0.03" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "V": """<mujoco><option density="1.2" viscosity="0" wind="3 0 0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "W": """<mujoco><option density="1.2" viscosity="0.001"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="box" size="0.1 0.05 0.025" mass="1"/>
            </body></worldbody></mujoco>""",
    "X": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.1" mass="1"
                  fluidcoef="1.0 0.5 2.0 0.5 0.5"/>
            </body></worldbody></mujoco>""",
    "Y": """<mujoco><option density="1.2" viscosity="0.001"/>
            <worldbody><body pos="0 0 1" quat="0.707 0 0.707 0"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "Z": """<mujoco><option density="1.2" viscosity="0.001"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.001" mass="1e-20"/>
            </body></worldbody></mujoco>""",
    "O": """<mujoco><option density="1.2" viscosity="0"/>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="sphere" size="0.001" mass="1e-20"
                  fluidshape="ellipsoid"/>
            </body></worldbody></mujoco>""",
    "AA": """<mujoco><option density="1.2" viscosity="0.001"/>
            <default><geom fluidshape="ellipsoid"
                          fluidcoef="1.0 0.5 2.0 0.5 0.5"/></default>
            <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="ellipsoid" size="0.1 0.05 0.02" mass="1"/>
            </body></worldbody></mujoco>""",
}

TESTS = {
    "T2":  {"model": "A", "qvel": [0,0,0, 0,0,-1]},
    "T3":  {"model": "B", "qvel": [0,0,0, 1,0,0]},
    "T4":  {"model": "C", "qvel": [0,0,0, 0,0,0]},
    "T6":  {"model": "D", "qvel": [0,0,0, 0,0,-1]},
    "T7":  {"model": "D", "qvel": [0,0,10, 1,0,0]},
    "T8":  {"model": "E", "qvel": [0,0,0, 1,0.5,0]},
    "T9":  {"model": "D", "qvel": [5,0,0, 0,0,-1]},
    "T10": {"model": "F", "qvel": [5,0,0, 1,0.5,0]},
    "T11": {"model": "H", "qvel": [0,0,0, 0,0,-1]},
    "T13": {"model": "I", "qvel": [0,0,0, 0,0,-1]},
    "T14": {"model": "J", "qvel": [0,0,0, 1,0.5,0]},
    "T21": {"model": "K", "qvel": [0,0,0, 0,0,-1, 0,0,0, 0,0,-1]},
    "T23": {"model": "M", "qvel": [0,0,0, 0,0,-1]},
    "T25": {"model": "N", "qvel": [0,0,0, 1,0.5,0]},
    "T27": {"model": "P", "qvel": [0,0,0, 1,0.5,0]},
    "T28": {"model": "Q", "qvel": [0,0,0, 0,0,0]},
    "T29": {"model": "R", "qvel": [0,0,0, 0,0,-1]},
    "T30": {"model": "D", "qvel": [0,0,0, 0,0,0]},
    "T31": {"model": "S", "qvel": [0,0,0, 1,0.5,-1]},
    "T32": {"model": "T", "qvel": [10,5,3, 0,0,0]},
    "T33": {"model": "U", "qvel": [0,0,0, 1,0.5,0]},
    "T35": {"model": "V", "qvel": [0,0,0, 0,0,0]},
    "T36": {"model": "W", "qvel": [10,5,3, 0,0,-1]},
    "T37": {"model": "X", "qvel": [0,0,0, 0,0,-1]},
    "T41": {"model": "Y", "qvel": [0,0,0, 1,0.5,-1]},
    "T15": {"model": "Z", "qvel": [0,0,0, 1,0,0]},
    "T26": {"model": "O", "qvel": [0,0,0, 1,0,0]},
    "T42": {"model": "AA", "qvel": [0,0,0, 1,0.5,0]},
}

print("=== Single-step qfrc_fluid reference values ===")
for name, cfg in sorted(TESTS.items()):
    xml = MODELS[cfg["model"]]
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = cfg["qvel"]
    mujoco.mj_forward(m, d)
    print(f"\n{name} (Model {cfg['model']}, qvel={cfg['qvel']}):")
    print(f"  qfrc_passive = {np.array2string(d.qfrc_passive, precision=15)}")
    # Use qfrc_fluid if available (MuJoCo >= 3.2)
    if hasattr(d, 'qfrc_fluid'):
        print(f"  qfrc_fluid   = {np.array2string(d.qfrc_fluid, precision=15)}")
    # geom_fluid[0] for each geom (dispatch indicator)
    for g in range(m.ngeom):
        print(f"  geom {g} fluid[0:6] = {m.geom_fluid[g, :6].tolist()}")
        if m.geom_fluid[g, 0] > 0:
            print(f"  geom {g} fluid[6:12] = {m.geom_fluid[g, 6:12].tolist()}")

print("\n=== T5: Inertia-box equivalent box for sphere r=0.1 ===")
m = mujoco.MjModel.from_xml_string(MODELS["A"])
I = m.body_inertia[1]
mass = m.body_mass[1]
for i, (j, k) in enumerate([(1,2),(0,2),(0,1)]):
    box_i = np.sqrt(max(1e-15, (I[j]+I[k]-I[i]) / mass * 6))
    print(f"  box[{i}] = {box_i:.15f}  (expect sqrt(0.024) ≈ 0.1549 for sphere r=0.1)")

print("\n=== T15: Body mass guard — inertia-box (near-zero mass) ===")
m = mujoco.MjModel.from_xml_string(MODELS["Z"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 1,0,0]
mujoco.mj_forward(m, d)
print(f"  body_mass = {m.body_mass[1]}")
print(f"  qfrc_passive = {np.array2string(d.qfrc_passive, precision=15)}")
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid   = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T26: Body mass guard — ellipsoid (near-zero mass) ===")
m = mujoco.MjModel.from_xml_string(MODELS["O"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 1,0,0]
mujoco.mj_forward(m, d)
print(f"  body_mass = {m.body_mass[1]}")
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid   = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T17: Kappa and virtual mass for sphere r=0.1 ===")
m = mujoco.MjModel.from_xml_string(MODELS["D"])
print(f"  geom_fluid = {m.geom_fluid[0].tolist()}")
print(f"  virtual_mass    = {m.geom_fluid[0, 6:9].tolist()}")
print(f"  virtual_inertia = {m.geom_fluid[0, 9:12].tolist()}")

print("\n=== T22: Default class inheritance ===")
m = mujoco.MjModel.from_xml_string(MODELS["L"])
print(f"  geom_fluid[0] (interaction_coef) = {m.geom_fluid[0, 0]}")
print(f"  geom_fluid[0:6]  = {m.geom_fluid[0, :6].tolist()}")
print(f"  geom_fluid[6:12] = {m.geom_fluid[0, 6:12].tolist()}")

print("\n=== T23: Cylinder semi-axes ===")
m = mujoco.MjModel.from_xml_string(MODELS["M"])
print(f"  geom_fluid = {m.geom_fluid[0].tolist()}")
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 0,0,-1]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T20: 50-step trajectory ===")
for model_key in ["A", "J"]:
    xml = MODELS[model_key]
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qvel[:] = [0,0,0, 0,0,-1]
    print(f"\nModel {model_key}:")
    for step in range(50):
        mujoco.mj_step(m, d)
        if step % 10 == 9:
            fluid = d.qfrc_fluid if hasattr(d, 'qfrc_fluid') else d.qfrc_passive
            print(f"  step {step+1}: qfrc_fluid={np.array2string(fluid, precision=15)}")

print("\n=== T25: Inertia-box ximat != xmat (geom offset) ===")
m = mujoco.MjModel.from_xml_string(MODELS["N"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 1,0.5,0]
mujoco.mj_forward(m, d)
print(f"  body_inertia  = {m.body_inertia[1].tolist()}")
print(f"  ximat (3x3)   = {d.ximat[1].tolist()}")
print(f"  xmat  (3x3)   = {d.xmat[1].tolist()}")
print(f"  ximat == xmat? {np.allclose(d.ximat[1], d.xmat[1])}")
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid    = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T29: Capsule semi-axes ===")
m = mujoco.MjModel.from_xml_string(MODELS["R"])
print(f"  geom_fluid = {m.geom_fluid[0].tolist()}")
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 0,0,-1]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T31: Multi-ellipsoid body (per-geom accumulation) ===")
m = mujoco.MjModel.from_xml_string(MODELS["S"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 1,0.5,-1]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")
for g in range(m.ngeom):
    print(f"  geom {g} fluid = {m.geom_fluid[g].tolist()}")

print("\n=== T32: Angular-only velocity (angular drag isolation) ===")
m = mujoco.MjModel.from_xml_string(MODELS["T"])
d = mujoco.MjData(m)
d.qvel[:] = [10,5,3, 0,0,0]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T33: Oblate spheroid drag ===")
m = mujoco.MjModel.from_xml_string(MODELS["U"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 1,0.5,0]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")
print(f"  geom_fluid = {m.geom_fluid[0].tolist()}")

print("\n=== T34: Kappa for oblate spheroid (3, 3, 1) ===")
m = mujoco.MjModel.from_xml_string(MODELS["U"])
print(f"  virtual_mass    = {m.geom_fluid[0, 6:9].tolist()}")
print(f"  virtual_inertia = {m.geom_fluid[0, 9:12].tolist()}")

print("\n=== T35: Ellipsoid with wind (stationary body) ===")
m = mujoco.MjModel.from_xml_string(MODELS["V"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 0,0,0]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T36: Inertia-box angular velocity ===")
m = mujoco.MjModel.from_xml_string(MODELS["W"])
d = mujoco.MjData(m)
d.qvel[:] = [10,5,3, 0,0,-1]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T37: fluidcoef without fluidshape=ellipsoid ===")
m = mujoco.MjModel.from_xml_string(MODELS["X"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 0,0,-1]
mujoco.mj_forward(m, d)
print(f"  geom_fluid = {m.geom_fluid[0].tolist()}")
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T40: Zero fluid — qfrc_fluid buffer zeroed ===")
m = mujoco.MjModel.from_xml_string(MODELS["G"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 0,0,-1]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")
print(f"  qfrc_passive = {np.array2string(d.qfrc_passive, precision=15)}")

print("\n=== T41: Rotated body (non-identity initial orientation) ===")
m = mujoco.MjModel.from_xml_string(MODELS["Y"])
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 1,0.5,-1]
mujoco.mj_forward(m, d)
print(f"  ximat  = {d.ximat[1].tolist()}")
print(f"  geom_xmat = {d.geom_xmat[0].tolist()}")
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")

print("\n=== T42: Default class inheritance of fluidcoef ===")
m = mujoco.MjModel.from_xml_string(MODELS["AA"])
print(f"  geom_fluid[0] (interaction_coef) = {m.geom_fluid[0, 0]}")
print(f"  geom_fluid[0:6]  = {m.geom_fluid[0, :6].tolist()}")
print(f"  geom_fluid[6:12] = {m.geom_fluid[0, 6:12].tolist()}")
d = mujoco.MjData(m)
d.qvel[:] = [0,0,0, 1,0.5,0]
mujoco.mj_forward(m, d)
if hasattr(d, 'qfrc_fluid'):
    print(f"  qfrc_fluid = {np.array2string(d.qfrc_fluid, precision=15)}")
```

Reference values above should be extracted using this script on MuJoCo 3.5.0
and hardcoded into the integration tests.

#### Files

| File | Action | Changes |
|------|--------|---------|
| `sim/L0/mjcf/src/types.rs` | modify | `FluidShape` enum, add `fluidshape`/`fluidcoef` to `MjcfGeom` |
| `sim/L0/mjcf/src/parser.rs` | modify | Parse `fluidshape`, `fluidcoef` attributes on `<geom>` |
| `sim/L0/mjcf/src/defaults.rs` | modify | Add `fluidshape`/`fluidcoef` to `apply_to_geom()` default inheritance |
| `sim/L0/mjcf/src/model_builder.rs` | modify | Precompute `geom_fluid[12]` per geom (semi-axes → kappa → virtual mass/inertia), pack into Model |
| `sim/L0/core/src/mujoco_pipeline.rs` | modify | Add `Model::geom_fluid`, `Data::qfrc_fluid`. Add `mj_fluid()`, `mj_inertia_box_fluid()`, `mj_ellipsoid_fluid()`, `fluid_geom_semi_axes()`, `get_added_mass_kappa()`, `ellipsoid_moment()`, `rotate_spatial_to_world()`, `object_velocity_local()`, `cross3()`, `add_cross3()`, `norm3()`. Call from `mj_fwd_passive()`. Reuse existing `mj_apply_ft()`. |
| `sim/L0/tests/integration/fluid_forces.rs` | create | Tests T1–T42 (36 integration/unit + 2 parse error + 4 new coverage), MJCF model constants, MuJoCo 3.5.0 reference values |

#### Deferred Items

- **Sleep filtering + disable-flag interaction**: Moved to §41. When §41 wires
  `mj_fwd_passive()`, it should (a) replace `0..nbody` with sleep-filtered iteration
  in `mj_fluid()`, and (b) gate `mj_fluid()` on `!(DISABLE_SPRING && DISABLE_DAMPER)`.
  See §41 acceptance criteria #10 and #13.
- **Implicit derivatives (S9)**: Promoted to §40a. See below.
- **MuJoCo 3.5.0 reference verification** (40d): **Complete** (2026-02-20).
  Verified against MuJoCo 3.5.0 (Python 3.12.12, `uv` venv).

  **Results:**
  - 38/42 tests: exact match (tolerance 1e-10)
  - T13: 0.3% deviation — eigendecomposition ordering in composite body inertia
    (model_builder.rs). Tracked as future fix in capsule inertia eigenvalue ordering.
  - T15, T26: MuJoCo rejects `mass < mjMINVAL` at XML load time; these test our
    internal mass guard only — cannot verify against MuJoCo.
  - T20: 50-step trajectory match within 1e-6 accumulated tolerance.

  **Bugs found and fixed:**
  1. `mj_fwd_velocity`: free joint angular velocity was not rotated from body-local
     to world frame (Ball joint already did this correctly). Fixed by applying
     `xquat * omega_local` for free joints.
  2. `object_velocity_local`: reference point was `subtree_com[root_id]` but our
     `cvel` stores velocity at `xpos[body_id]`. Fixed by using `xpos[body_id]`.
  - T11 (mixed geoms with offset) and T41 (rotated body) values updated to match
    MuJoCo 3.5.0 after bug fixes. T20 upgraded from `is_finite()` to exact values.

---

### 40a. Fluid Force Velocity Derivatives (Implicit Integration)
**Status:** ✅ Complete | **Effort:** L | **Prerequisites:** §40

#### Current State

Fully implemented and verified. Both fluid models (inertia-box and ellipsoid)
have analytical `∂qfrc_fluid/∂qvel` derivatives integrated into `mjd_passive_vel`.
The implicit integrators (`Implicit` and `ImplicitFast`) now correctly account for
fluid damping in the D matrix (`M − h·D`).

**Implementation summary:**
- `mjd_fluid_vel` top-level dispatch in `derivatives.rs`, called from
  `mjd_passive_vel` before per-DOF damping (matching MuJoCo dispatch order)
- `mjd_inertia_box_fluid`: diagonal B (6 rank-1 updates via `add_rank1`)
- `mjd_ellipsoid_fluid`: full 6×6 B from 5 analytical component functions
  (`mjd_added_mass_forces`, `mjd_magnus_force`, `mjd_kutta_lift`,
  `mjd_viscous_drag`, `mjd_viscous_torque`), projected via `add_jtbj`
- `ImplicitFast` symmetrization: B ← (B + Bᵀ)/2 per-geom before projection
- Jacobian infrastructure: `mj_jac_point` (6×nv), `mj_jac_body_com`,
  `mj_jac_geom` in `mujoco_pipeline.rs`
- 31 integration tests in `fluid_derivatives.rs` (T1–T32, no T18/T33/T34):
  - T1–T3: inertia-box B scalar analytical validation (T1: viscous formula +
    velocity-independence, T2: density formula at known velocity, T3: viscous +
    density additivity proof via three-model decomposition)
  - T4–T5: inertia-box FD validation + MuJoCo conformance
  - T6: `mjd_cross` helper FD validation
  - T7–T11: per-component ellipsoid FD validation
  - T12–T14: full ellipsoid FD + MuJoCo conformance + multi-geom
  - T15–T16: symmetry verification (T15: ImplicitFast symmetric, T16: Implicit
    asymmetric — quantified max asymmetry > 1e-4, localized to angular-linear
    coupling block)
  - T17: ImplicitFast MuJoCo conformance
  - T19–T20: energy dissipation stability (Implicit + ImplicitFast)
  - T21–T24: guards and edge cases (T21: zero fluid, T22: zero velocity →
    analytical viscous-only B scalar validation + zero off-diagonal, T23:
    massless body, T24: interaction_coef=0 → inertia-box fallback equivalence
    proof via two-model comparison)
  - T25–T28: Jacobian geometry (geom offset, wind, mixed multi-body, single-axis)
  - T29–T31: joint-type coverage (hinge, ball, multi-body chain)
  - T32: `mj_jac_point` vs `mj_jac_site` exact match
- All MuJoCo conformance tests validated against MuJoCo 3.5.0 reference values
  extracted via `mj_forward` + sparse-to-dense `qDeriv` conversion
- DOF convention confirmed: both MuJoCo and our code use [v,ω] (linear first)
  for free joint DOFs — no permutation needed

**Files modified:**
| File | Changes |
|------|---------|
| `sim/L0/core/src/derivatives.rs` | Added `mjd_fluid_vel`, `mjd_inertia_box_fluid`, `mjd_ellipsoid_fluid`, 5 component Jacobian functions, `mjd_cross`, `add_jtbj`, `add_rank1`, `add_to_quadrant`, `rotate_jac_to_local`. Inserted `mjd_fluid_vel` call in `mjd_passive_vel`. Promoted `mjd_passive_vel` to `pub`. |
| `sim/L0/core/src/mujoco_pipeline.rs` | Added `mj_jac_point`, `mj_jac_body_com`, `mj_jac_geom`. Promoted `object_velocity_local`, `fluid_geom_semi_axes`, `ellipsoid_moment`, `norm3`, `MJ_MINVAL` to `pub(crate)`. |
| `sim/L0/core/src/lib.rs` | Added `mjd_passive_vel`, `mj_jac_point`, `mj_jac_site` to public re-exports. |
| `sim/L0/tests/integration/fluid_derivatives.rs` | **New file** — 31 tests (T1–T32, see test matrix below). |
| `sim/L0/tests/integration/mod.rs` | Registered `fluid_derivatives` module. |

**Deviations from spec:**
- T18 (Implicit conformance with humanoid) was not implemented as a separate test;
  T13 covers the same implicit path with an ellipsoid model.
- T33/T34 (regression) are covered implicitly — all existing §40 and derivative
  tests continue to pass.
- Test file is `fluid_derivatives.rs` (new file) rather than modifying
  `fluid_forces.rs`, keeping forward and derivative tests cleanly separated.

#### MuJoCo Reference

MuJoCo computes `∂qfrc_fluid/∂qvel` in `mjd_passive_vel()` (`engine_derivative.c`).
For each body, it constructs a 6×6 body-level drag-velocity Jacobian (B matrix)
and accumulates it into the D matrix via `addJTBJ(J^T · B · J)`:

- **Inertia-box model:** B is **diagonal** — 6 scalar entries, one per spatial
  axis. Viscous terms are velocity-independent constants; quadratic terms have
  `|v|`-dependent derivatives via `d(|v|·v)/dv = 2|v|`. Accumulated as 6
  independent rank-1 updates: `D += b_k · J_k^T · J_k` for each axis k.

- **Ellipsoid model:** B is a **full 6×6** matrix assembled from **5 analytical
  component derivative functions** — one per force component. MuJoCo does NOT use
  finite differences; each component has a hand-derived analytical Jacobian:
  - `mjd_addedMassForces()` → all four 3×3 quadrants (cross-product derivatives)
  - `mjd_magnus_force()` → quadrants (1,0) and (1,1)
  - `mjd_kutta_lift()` → quadrant (1,1)
  - `mjd_viscous_drag()` → quadrant (1,1)
  - `mjd_viscous_torque()` → quadrant (0,0)
  Accumulated as a single `D += J^T · B · J` with full 6×6 B.

The B matrix is assembled in the **local frame** (inertia frame for inertia-box,
geom frame for ellipsoid), then projected to generalized coordinates via the body
Jacobian rotated into that same local frame: `J_local = R^T · J_world`.

**Symmetrization:** For `ImplicitFast` only, the ellipsoid B matrix is
symmetrized: `B ← (B + B^T)/2` before projection. This is required because
`ImplicitFast` uses Cholesky factorization of `M − h·D` (Cholesky requires SPD).
The full `Implicit` integrator uses LU and can handle asymmetric D.
The inertia-box B is inherently diagonal (symmetric), so no symmetrization needed.

**Per-body vs per-geom:** Inertia-box derivatives use the body CoM Jacobian
(`mj_jacBodyCom`). Ellipsoid derivatives are **per-geom** — each geom gets its
own Jacobian (`mj_jacGeom` at `geom_xpos`), B matrix, and `J^T·B·J`
accumulation. Multiple geoms on the same body contribute additively.

**qDeriv storage:** MuJoCo stores `qDeriv` in compressed sparse row format
(`D_rowadr`, `D_colind`, `D_rownnz`). Our codebase uses a dense `DMatrix<f64>`
(`nv × nv`). This simplifies `J^T·B·J` accumulation to standard dense matrix
operations. Dense is acceptable for target model sizes (`nv < 200`).

**Dispatch order in MuJoCo's `mjd_passive_vel`:**
1. Fluid derivatives (density/viscosity check)
2. Per-DOF damping (damper disable check)
3. Flex edge damping
4. Tendon damping

Our `mjd_passive_vel` now matches this order: fluid derivatives first, then
per-DOF damping, then tendon damping. All contributions are additive so order
doesn't affect correctness, but matching MuJoCo simplifies auditing.

#### Objective

~~Implement `∂qfrc_fluid/∂qvel` for both fluid models so the implicit integrator
correctly accounts for fluid damping in the D matrix.~~ Done.

#### Specification

**Notation used throughout:**
- `ρ` = `model.density` (fluid density, kg/m³)
- `β` = `model.viscosity` (dynamic viscosity, Pa·s)

##### S1. Approach — Fully Analytical Derivatives for Both Models

Both models use analytical derivatives, matching MuJoCo exactly:

- **Inertia-box:** Closed-form diagonal B (6 scalars). Straightforward.
- **Ellipsoid:** 5 analytical component Jacobian functions, each filling specific
  3×3 quadrants of the 6×6 B matrix. More involved but well-defined — each
  component's derivative is a direct application of vector calculus identities
  (cross-product derivatives, `d(|v|·v)/dv`, projected-area derivatives).

No finite differences anywhere.

##### S2. Integration Point

`mjd_fluid_vel(model, data)` in `derivatives.rs` is called from
`mjd_passive_vel()` **before** per-DOF damping and tendon damping, matching
MuJoCo's dispatch order in `mjd_passive_vel()` where fluid derivatives are
computed first.

**Gate condition:** `model.density > 0.0 || model.viscosity > 0.0` (same as
`mj_fluid()`).

**Implemented `mjd_passive_vel` structure:**
```rust
pub fn mjd_passive_vel(model: &Model, data: &mut Data) {
    // 1. Fluid derivatives (before per-DOF damping)
    mjd_fluid_vel(model, data);

    // 2. Per-DOF damping: diagonal entries
    for i in 0..model.nv {
        data.qDeriv[(i, i)] += -model.implicit_damping[i];
    }

    // 3. Tendon damping
    // ...
}
```

**Call chain:**
```
mj_fwd_acceleration_implicitfast() / mj_fwd_acceleration_implicit_full()
  → data.qDeriv.fill(0.0)
  → mjd_passive_vel()
    → mjd_fluid_vel()     (first in mjd_passive_vel)
    → per-DOF damping
    → tendon damping
  → mjd_actuator_vel()
  → mjd_rne_vel()         (implicit_full only)
```

Note: `mj_fwd_acceleration_implicitfast` and `mj_fwd_acceleration_implicit_full`
call `mjd_passive_vel` directly (not through `mjd_smooth_vel`). They clear
`qDeriv` themselves before calling the derivative functions.

**Phase D benefit:** `mjd_passive_vel` is also called by `mjd_smooth_vel()`
(used by `mjd_transition_hybrid` for transition Jacobian computation and
exposed as a `pub` API). Fluid derivatives automatically improve transition
Jacobians for all integrator modes, not just the implicit D matrix — no
additional wiring needed.

##### S3. Body Jacobian Construction (6×nv)

A shared Jacobian kernel `mj_jac_point(model, data, body_id, point)` returns a
6×nv Jacobian mapping `qvel → [ω; v]` at an arbitrary world-frame point on the
body's kinematic chain. Wrapped as:

- `mj_jac_body_com(model, data, body_id)` → calls `mj_jac_point` at
  `xipos[body_id]` (body CoM) — used by inertia-box
- `mj_jac_geom(model, data, geom_id)` → calls `mj_jac_point` at
  `geom_xpos[geom_id]` with `geom_body[geom_id]` — used by ellipsoid

**Return format:** A single `DMatrix<f64>` of shape 6×nv:
- Rows 0–2: rotational Jacobian `J_rot` (angular velocity contribution)
- Rows 3–5: translational Jacobian `J_trans` (linear velocity contribution)

This differs from the existing `mj_jac_site()` which returns `(jac_trans, jac_rot)`
as two separate 3×nv matrices. The combined 6×nv format is required for the
`J^T·B·J` projection where B is 6×6.

**Shared kernel:**
```rust
/// Compute 6×nv world-frame Jacobian at `point` on the chain rooted at `body_id`.
/// Layout: rows 0–2 = angular, rows 3–5 = linear.
/// Chain-walks from body_id to root (same pattern as mj_jac_site / mj_apply_ft).
fn mj_jac_point(model: &Model, data: &Data, body_id: usize, point: &Vector3<f64>) -> DMatrix<f64> {
    let mut J = DMatrix::zeros(6, model.nv);
    let mut current = body_id;
    while current != 0 {
        // Walk joints of current body, fill J rows 0–2 (rot) and 3–5 (trans)
        // Same joint-type dispatch as mj_jac_site:
        //   Hinge:  J_rot[:,dof] += axis;  J_trans[:,dof] += axis × (point − anchor)
        //   Slide:  J_trans[:,dof] += axis
        //   Ball:   3 columns (local basis rotated to world)
        //   Free:   3 trans identity columns + 3 rot columns with cross product
        current = model.body_parent[current];
    }
    J
}
```

**Location:** `mujoco_pipeline.rs` alongside existing `mj_jac_site` and
`mj_apply_ft`. The derivative code in `derivatives.rs` calls these via
`pub(crate)` visibility.

**Local-frame rotation:** After computing the world-frame Jacobian, rotate into
the local frame before constructing B:
```rust
// For each 3-row block: J_local[block] = R^T · J_world[block]
// R = ximat[body_id]    for inertia-box
// R = geom_xmat[gid]    for ellipsoid
fn rotate_jac_to_local(J: &mut DMatrix<f64>, R: &Matrix3<f64>) {
    let Rt = R.transpose();
    // Rotate angular rows (0–2)
    let ang = Rt * J.rows(0, 3);
    J.rows_mut(0, 3).copy_from(&ang);
    // Rotate linear rows (3–5)
    let lin = Rt * J.rows(3, 3);
    J.rows_mut(3, 3).copy_from(&lin);
}
```

**Refactor deferral:** `mj_jac_site` currently implements its own chain-walk
returning `(3×nv, 3×nv)`. A future cleanup could refactor it to call
`mj_jac_point` and split the 6×nv result. Deferred to §40e to avoid widening
§40a's blast radius.

##### S4. Inertia-Box B Matrix (Diagonal, Per-Body)

The inertia-box forward force per axis `k` has the form `f_k = c_k · v_k` (viscous)
plus `f_k = -q_k · |v_k| · v_k` (quadratic). The derivative is:

```
∂f_k/∂v_k = c_k − q_k · d(|v|·v)/dv = c_k − 2·q_k·|v_k|
```

where `d(|v|·v)/dv = 2|v|` for scalar v (valid even at v=0 since 2·|0|=0).

**Angular axes (k = 0,1,2):**
```
b_k = −π·diam³·β − 2·ρ·box[k]·(box[j]⁴+box[l]⁴)/64 · |ω_k|
```
where `(j,l)` are the other two axes, diam = (box[0]+box[1]+box[2])/3.

**Linear axes (k = 3,4,5), mapping to box axes (0,1,2):**
```
b₃ = −3π·diam·β − 2·0.5·ρ·box[1]·box[2] · |v₀|
b₄ = −3π·diam·β − 2·0.5·ρ·box[0]·box[2] · |v₁|
b₅ = −3π·diam·β − 2·0.5·ρ·box[0]·box[1] · |v₂|
```

**MuJoCo equivalence:** MuJoCo applies viscous and quadratic terms as separate
rank-1 updates (up to 12 total: 6 viscous + 6 quadratic). Our formulation
combines them into 6 total rank-1 updates by summing `b_k = viscous_k + quad_k`.
This is mathematically identical: `(b₁+b₂)·JᵀJ = b₁·JᵀJ + b₂·JᵀJ`. When
only `viscosity > 0` the quadratic term is zero and vice versa, so the combined
formula naturally handles single-source cases.

**Accumulation:** Per-axis rank-1 update. For each axis k:
```
qDeriv += b_k · J_local[k,:]^T · J_local[k,:]
```
This is 6 rank-1 outer products per body. More efficient than a single 6×6 `J^T·B·J`
because B is diagonal.

##### S5. Ellipsoid B Matrix (Full 6×6, Per-Geom, Analytical Components)

The 6×6 B matrix uses spatial vector ordering `[angular(3), linear(3)]`, organized
in four 3×3 quadrants:

```
B = [ Q(0,0): ∂torque/∂ω    Q(0,1): ∂torque/∂v  ]
    [ Q(1,0): ∂force/∂ω     Q(1,1): ∂force/∂v   ]
```

**Quadrant placement convention:** Our spec uses `Q(row_block, col_block)` where
row_block 0 = torque (angular force), row_block 1 = linear force, col_block 0 =
angular velocity, col_block 1 = linear velocity. This is the natural Jacobian
layout `Q(output, input)`.

**Mapping to MuJoCo's `addToQuadrant(B, D, col_quad, row_quad)`:** MuJoCo's
parameter names are misleading — `col_quad` selects the **row** block and
`row_quad` selects the **column** block. Verified by tracing the implementation:
`B[6*(3*col_quad+i) + (3*row_quad+j)] += D[3*i+j]` with row-major B gives
`B_{3*col_quad+i, 3*row_quad+j} += D_{i,j}`. Our implementation MUST use the
intuitive `(row_block, col_block)` convention, not replicate MuJoCo's swapped
parameter order:

```rust
/// Add 3×3 matrix D (row-major) to B at block position (row_block, col_block).
fn add_to_quadrant(B: &mut [[f64; 6]; 6], D: &[f64; 9], row_block: usize, col_block: usize) {
    let r = 3 * row_block;
    let c = 3 * col_block;
    for i in 0..3 {
        for j in 0..3 {
            B[r + i][c + j] += D[3 * i + j];
        }
    }
}
```

Initialize `B = [[0.0; 6]; 6]`, then accumulate 5 component derivatives:

**Component 1 — Added mass (gyroscopic) derivatives: `mjd_addedMassForces`**

The forward computation (`mj_ellipsoid_fluid`, Component 1) has three cross-product
terms. Variable mapping: forward code `lv` = spec `p_ang`, forward code `pv` = spec
`p_lin`:
```
torque += p_ang × ω       where p_ang = ρ·[vi₀·ω₀, vi₁·ω₁, vi₂·ω₂]   (forward: lv)
torque += p_lin × v        where p_lin = ρ·[vm₀·v₀, vm₁·v₁, vm₂·v₂]   (forward: pv)
force  += p_lin × ω
```

**Helper — `mjd_cross(a, b) -> (Da, Db)`:**

```rust
/// Cross-product Jacobians: Da = ∂(a×b)/∂a, Db = ∂(a×b)/∂b.
/// Both are 3×3 row-major: M[i*3+j] = ∂(a×b)_i / ∂(var)_j.
/// Da = [b]×^T (skew-transpose of b), Db = −[a]×^T.
fn mjd_cross(a: &[f64; 3], b: &[f64; 3]) -> ([f64; 9], [f64; 9]) {
    let Da = [
         0.0,    b[2],  -b[1],   // row 0: ∂(a×b)₀/∂a_{0,1,2}
        -b[2],   0.0,    b[0],   // row 1: ∂(a×b)₁/∂a_{0,1,2}
         b[1],  -b[0],   0.0,    // row 2: ∂(a×b)₂/∂a_{0,1,2}
    ];
    let Db = [
         0.0,   -a[2],   a[1],   // row 0: ∂(a×b)₀/∂b_{0,1,2}
         a[2],   0.0,   -a[0],   // row 1: ∂(a×b)₁/∂b_{0,1,2}
        -a[1],   a[0],   0.0,    // row 2: ∂(a×b)₂/∂b_{0,1,2}
    ];
    (Da, Db)
}
```

**Derivative algorithm:** Each cross-product term where both arguments depend on
the same velocity variable contributes **two** 3×3 matrices to the same quadrant
(Da from the chain-ruled first argument + Db from the direct second argument).

**Term 1:** `torque += p_ang × ω` — both `p_ang` and `ω` depend on angular velocity
```
(Da, Db) = mjd_cross(p_ang, ω)
Q(0,0) += Db                          // ∂(p_ang × ω)/∂ω — direct
Da[k] *= ρ · vi[k % 3]               // chain rule: ∂p_ang/∂ω = diag(ρ·vi)
Q(0,0) += Da                          // ∂(p_ang × ω)/∂p_ang · ∂p_ang/∂ω
```

**Term 2:** `torque += p_lin × v` — both `p_lin` and `v` depend on linear velocity
```
(Da, Db) = mjd_cross(p_lin, v)
Q(0,1) += Db                          // ∂(p_lin × v)/∂v — direct
Da[k] *= ρ · vm[k % 3]               // chain rule: ∂p_lin/∂v = diag(ρ·vm)
Q(0,1) += Da                          // ∂(p_lin × v)/∂p_lin · ∂p_lin/∂v
```

**Term 3:** `force += p_lin × ω` — `p_lin` depends on v, `ω` is independent
```
(Da, Db) = mjd_cross(p_lin, ω)
Q(1,0) += Db                          // ∂(p_lin × ω)/∂ω → d(force)/d(ω)
Da[k] *= ρ · vm[k % 3]               // chain rule: ∂p_lin/∂v = diag(ρ·vm)
Q(1,1) += Da                          // ∂(p_lin × ω)/∂p_lin · ∂p_lin/∂v
```

The column scaling `Da[k] *= ρ·coef[k % 3]` applies the diagonal chain rule
`∂p/∂vel = diag(ρ·coef)` by scaling each column j of the row-major Da (flat
index k maps to column `k % 3`). This matches MuJoCo's loop:
`Da[i] *= fluid_density * virtual_mass[i % 3]`.

**Component 2 — Magnus lift: `mjd_magnus_force`**

Forward: `F = c_mag · ρ · V · (ω × v)` where V = ellipsoid volume.

```
∂F/∂ω = c_mag·ρ·V · ∂(ω×v)/∂ω = c_mag·ρ·V · [v]×^T  → Q(1,0)
∂F/∂v = c_mag·ρ·V · ∂(ω×v)/∂v = c_mag·ρ·V · (−[ω]×^T) → Q(1,1)
```

**Implementation via pre-scaling (matching MuJoCo):** Set `c = c_mag·ρ·V`, then
`scaled_ω = c·ω`, `scaled_v = c·v`, and call `mjd_cross(scaled_ω, scaled_v)`.
Because `∂(a×b)/∂a` depends only on `b` (and vice versa), evaluating at
`(c·ω, c·v)` directly yields `Da = [c·v]×^T = c·[v]×^T = ∂F/∂ω` and
`Db = −[c·ω]×^T = −c·[ω]×^T = ∂F/∂v`. Single `mjd_cross` call, no separate
scaling step needed. This is exactly what MuJoCo's `mjd_magnus_force` does.

**Component 3 — Kutta lift: `mjd_kutta_lift`**

Forward: `F = c_K·ρ·cos_α·A_proj · ((n×v) × v)` where
`n = [a·v₀, b·v₁, c·v₂]` with `a = (s₁s₂)², b = (s₂s₀)², c = (s₀s₁)²`.

Derivative is a 3×3 matrix filling Q(1,1). Depends only on linear velocity.
SymPy-derived analytical formula matching MuJoCo's `mjd_kutta_lift()`:

```
Shorthand:
  (x, y, z)  = (v₀, v₁, v₂)
  (xx, yy, zz, xy, yz, xz) = pairwise products
  (aa, bb, cc) = (a², b², c²)
  proj_denom = aa·xx + bb·yy + cc·zz
  proj_num   = a·xx + b·yy + c·zz
  norm2      = xx + yy + zz

Common factor (derivation from `mj_ellipsoid_fluid` Component 3, the two lines
  computing `a_proj` and `cos_alpha`):
  cos_α  = proj_num / max(MJ_MINVAL, speed · proj_denom)
  A_proj = π · √(proj_denom / max(MJ_MINVAL, proj_num))
  cos_α · A_proj = π · √proj_num / (speed · √proj_denom)
                 = π · √(proj_num / (proj_denom · norm2))
  The derivative extracts π·c_K·ρ as a common scalar and absorbs the
  velocity-dependent factors into Steps 1–4. The `proj_num` factors in
  Steps 1 and 4 account for chain-rule contributions from cos_α and A_proj.

  df_denom = π·c_K·ρ / max(MJ_MINVAL, √(proj_denom · proj_num · norm2))

Per-axis lift coefficients:
  dfx_coef = yy·(a−b) + zz·(a−c)
  dfy_coef = xx·(b−a) + zz·(b−c)
  dfz_coef = xx·(c−a) + yy·(c−b)

Auxiliary terms:
  proj_term = proj_num / max(MJ_MINVAL, proj_denom)
  cos_term  = proj_num / max(MJ_MINVAL, norm2)

Assembly (3×3 row-major, coef = [a,b,c], df_coef = [dfx,dfy,dfz], vel = [x,y,z]):
  Step 1: D[i][j] = 2·proj_num·(coef[j] − coef[i])
  Step 2: inner[k] = coef[k]²·proj_term − coef[k] + cos_term
          D[i][:] += df_coef[i] · inner[:]
  Step 3: D[i][j] *= vel[i]·vel[j]
  Step 4: D[i][i] -= df_coef[i]·proj_num
  Step 5: D *= df_denom
```

Guard: when `|v| < MJ_MINVAL`, `df_denom → 0` via the denominator guard,
producing a zero derivative (consistent with forward Kutta force being zero
at rest).

**Component 4 — Combined linear drag: `mjd_viscous_drag`**

Forward: `F_i = −drag_lin · v_i` where
`drag_lin = β·3π·d_eq + ρ·|v|·(A_proj·c_b + c_s·(A_max−A_proj))`.

Derivative is a 3×3 matrix filling Q(1,1), matching MuJoCo's `mjd_viscous_drag()`:

```
Geometric constants:
  eq_d  = (2/3)·(s₀+s₁+s₂)
  A_max = π·d_max·d_mid
  a = (s₁·s₂)², b = (s₂·s₀)², c = (s₀·s₁)²
  (aa, bb, cc) = (a², b², c²)

Velocity shorthand:
  (x, y, z) = (v₀, v₁, v₂)
  (xx, yy, zz, xy, yz, xz) = pairwise products

Projected area:
  proj_denom = aa·xx + bb·yy + cc·zz
  proj_num   = a·xx + b·yy + c·zz
  A_proj     = π·√(proj_denom / max(MJ_MINVAL, proj_num))

Speed:
  norm     = √(xx + yy + zz)
  inv_norm = 1 / max(MJ_MINVAL, norm)

Coefficient decomposition:
  lin_coef    = β·3π·eq_d                                  (Stokes drag)
  quad_coef   = ρ·(A_proj·c_b + c_s·(A_max − A_proj))     (quadratic drag)
  Aproj_coef  = ρ·norm·(c_b − c_s)                        (area gradient weight)

Area gradient (velocity-direction sensitivity of projected area × speed):
  dA_coef = π / max(MJ_MINVAL, √(proj_num³ · proj_denom))
  dAproj_dv[k] = Aproj_coef · dA_coef · coef[k] · vel[k]
                 · (coef[j]·vel[j]²·(coef[k]−coef[j])
                  + coef[l]·vel[l]²·(coef[k]−coef[l]))
  where (j,l) = other two axes, coef = [a,b,c], vel = [x,y,z]

Assembly (3×3 row-major):
  D[i][j]  = vel[i]·vel[j] + norm²·δ_{ij}     // d(|v|·v)/dv identity
  D[i][j] *= −quad_coef · inv_norm              // quadratic drag contribution
  D[i][j] += −vel[i] · dAproj_dv[j]            // area gradient outer product
  D[i][i] −= lin_coef                           // Stokes drag (diagonal)
```

Three physical contributions:
1. **Quadratic drag** `d(−quad_coef·|v|·v)/dv`: uses identity
   `d(|v|v)/dv = vvᵀ/|v| + |v|I = (vvᵀ + |v|²I)/|v|`
2. **Area variation** `−v · (∂drag_lin/∂v)ᵀ`: projected area depends on
   velocity direction, producing an asymmetric outer-product correction
3. **Stokes drag** `−lin_coef · I`: velocity-independent viscous contribution

**Component 5 — Combined angular drag: `mjd_viscous_torque`**

Forward: `τ_i = −drag_ang · ω_i` where `drag_ang = β·π·d_eq³ + ρ·|mom_visc|`.

Derivative is a 3×3 matrix filling Q(0,0), matching MuJoCo's `mjd_viscous_torque()`:

```
Geometric constants:
  eq_d  = (2/3)·(s₀+s₁+s₂)
  I_max = (8/15)·π·d_mid·d_max⁴
  II[k] = ellipsoid_moment(s, k)              (per-axis moment)

Angular velocity shorthand:
  (x, y, z) = (ω₀, ω₁, ω₂)

Per-axis moment coefficient:
  mom_coef[k] = c_ang·II[k] + c_s·(I_max − II[k])

Weighted angular momentum:
  mom_visc[k] = ω[k] · mom_coef[k]

Anisotropic density scaling:
  density = ρ / max(MJ_MINVAL, |mom_visc|)

Squared moment terms:
  mom_sq[k] = −density · ω[k] · mom_coef[k]²

Stokes torque coefficient:
  lin_coef = β · π · eq_d³

Assembly (3×3 row-major):
  diag_val = Σ_k (ω[k] · mom_sq[k]) − lin_coef
  D[i][j]  = diag_val · δ_{ij} + ω[i] · mom_sq[j]
```

This follows the same `d(−f(ω)·ω)/dω` pattern as viscous drag but with
anisotropic weighting through `mom_coef`. The derivative of `|mom_visc|` w.r.t.
`ω_j` is `mom_visc_j · mom_coef[j] / |mom_visc|`, which after multiplying
by `−ρ·ω_i` produces the `ω[i]·mom_sq[j]` outer-product term. The diagonal
captures both the linear drag and the isotropic part of the quadratic drag.

**Accumulation:** After assembling all 5 components into B:
1. If `integrator == ImplicitFast`: symmetrize `B ← (B + B^T) / 2`
   *(MuJoCo calls `mju_symmetrize(B, B, 6)` inside `mjd_ellipsoidFluid`.
   The symmetrization happens per-geom before projection, not on the final
   qDeriv. The qDeriv-level symmetrization in `mj_fwd_acceleration_implicitfast`
   is a separate step handled by existing infrastructure.)*
2. Scale by `interaction_coef`: `B *= interaction_coef`
   *(MuJoCo's `mjd_ellipsoidFluid` uses `interaction_coef` only as a 0/1 gate
   (`if (geom_interaction_coef == 0.0) continue`). Our forward code
   (`mujoco_pipeline.rs:12131`) scales forces by `interaction_coef`, so our
   derivatives must include this factor for mathematical correctness:
   `∂(c·f)/∂v = c · ∂f/∂v`. Since `interaction_coef ∈ {0.0, 1.0}` in all
   valid models, the numerical result is identical to MuJoCo.)*
3. Project: `qDeriv += J_local^T · B · J_local`

##### S6. Dense `J^T·B·J` Accumulation

**Full 6×6 projection (ellipsoid):**

```rust
/// Accumulate J^T · B · J into qDeriv.
/// J: 6×nv DMatrix, B: 6×6 row-major [[f64; 6]; 6].
fn add_jtbj(qDeriv: &mut DMatrix<f64>, J: &DMatrix<f64>, B: &[[f64; 6]; 6]) {
    let nv = J.ncols();
    // Convert B to nalgebra matrix for clean multiply
    let b_mat = Matrix6::from_fn(|i, j| B[i][j]);
    let bj = b_mat * J;             // 6×nv
    // qDeriv += J^T · BJ           // nv×nv
    for r in 0..nv {
        for k in 0..6 {
            let jtk = J[(k, r)];     // J^T[r, k]
            if jtk == 0.0 { continue; }
            for c in 0..nv {
                qDeriv[(r, c)] += jtk * bj[(k, c)];
            }
        }
    }
}
```

The inner loop skips zero entries for efficiency — body Jacobians are sparse
(only ancestor DOFs are nonzero). For a free-floating body with `nv_chain ≈ 6`,
this runs ~216 multiplies per geom instead of 6·nv² for the full dense product.

**Rank-1 update (inertia-box):**

```rust
/// Accumulate b · row^T · row into qDeriv (rank-1 update).
fn add_rank1(qDeriv: &mut DMatrix<f64>, b: f64, row: DMatrixSlice<f64>) {
    let nv = row.ncols();
    for r in 0..nv {
        let jr = row[(0, r)];
        if jr == 0.0 { continue; }
        let br = b * jr;
        for c in 0..nv {
            let jc = row[(0, c)];
            if jc == 0.0 { continue; }
            qDeriv[(r, c)] += br * jc;
        }
    }
}
```

**Sparse Jacobian deferral:** MuJoCo supports both dense and sparse Jacobian paths
(`addJTBJ` vs `addJTBJSparse`) selectable via `mj_isSparse(m)`. This
implementation uses dense Jacobians only. Sparse support is deferred to §40d — it
provides a constant-factor speedup for large `nv` but does not affect correctness.
Dense is acceptable for the target model sizes (`nv < 200`).

##### S7. Top-Level Dispatch: `mjd_fluid_vel`

```rust
pub(crate) fn mjd_fluid_vel(model: &Model, data: &mut Data) {
    if model.density == 0.0 && model.viscosity == 0.0 {
        return;
    }

    for body_id in 0..model.nbody {
        if model.body_mass[body_id] < MJ_MINVAL { continue; }

        let use_ellipsoid = (model.body_geom_adr[body_id]
            ..model.body_geom_adr[body_id] + model.body_geom_num[body_id])
            .any(|gid| model.geom_fluid[gid][0] > 0.0);

        if use_ellipsoid {
            mjd_ellipsoid_fluid(model, data, body_id);
        } else {
            mjd_inertia_box_fluid(model, data, body_id);
        }
    }
}
```

Called from `mjd_passive_vel()` **before** per-DOF damping and tendon damping
(matching MuJoCo's dispatch order — see S2).

**Sleep filtering deferral:** MuJoCo's `mjd_passive_vel` supports sleep filtering
(`d->body_awake_ind`, `d->nbody_awake`) to skip sleeping bodies. Our codebase
does not yet implement sleep. This is a performance optimization that does not
affect correctness — deferred to §40c.

##### S8. Velocity Extraction and Wind Subtraction

Both derivative functions need the same local-frame velocity used by the forward
computation. Reuse `object_velocity_local()` from `mujoco_pipeline.rs` and apply
the same wind subtraction (translational only, rotated to local frame):

```rust
// Inertia-box: velocity at body CoM in body inertia frame
let mut lvel = object_velocity_local(model, data, body_id, &data.xipos[body_id], &data.ximat[body_id]);
let wind_local = data.ximat[body_id].transpose() * model.wind;
lvel[3] -= wind_local.x;
lvel[4] -= wind_local.y;
lvel[5] -= wind_local.z;

// Ellipsoid: velocity at geom center in geom frame
let mut lvel = object_velocity_local(model, data, geom_body, &data.geom_xpos[gid], &data.geom_xmat[gid]);
let wind_local = data.geom_xmat[gid].transpose() * model.wind;
lvel[3] -= wind_local.x;
lvel[4] -= wind_local.y;
lvel[5] -= wind_local.z;
```

**Wind is constant** — it does not depend on `qvel`, so `∂wind/∂qvel = 0`. The
derivative formulas use the wind-subtracted velocity (relative velocity) as their
input, which is correct: `∂f(v_rel)/∂qvel = ∂f/∂v_rel · ∂v_rel/∂qvel` and
`∂v_rel/∂qvel = ∂v_body/∂qvel` since `∂wind/∂qvel = 0`.

Critical: the derivative functions must use **identical** velocity values as the
forward computation. The velocity extraction code (reference point, wind
subtraction, rotation matrix) must match `mj_inertia_box_fluid` and
`mj_ellipsoid_fluid` exactly.

##### S9. Zero-Velocity and Degenerate-Input Guards

Each component function requires specific guards to prevent numerical
amplification when velocity or angular velocity is near zero. The guards below
match MuJoCo's `engine_derivative.c` implementation exactly.

**Inertia-box (S4):** No special guards needed. The `d(|v|·v)/dv = 2|v|`
identity naturally evaluates to zero at `v = 0`, producing zero quadratic
diagonal entries. Viscous terms are velocity-independent constants and always
contribute.

**Added mass (Component 1):** No guards needed. Cross products of zero-valued
momenta produce zero derivatives naturally.

**Magnus lift (Component 2):** No guards needed. Pre-scaled vectors are zero
when velocity is zero, producing a zero 3×3 via `mjd_cross`.

**Kutta lift (Component 3):** Early return if `speed < MJ_MINVAL` — return
zero 3×3 matrix. Without this guard, `df_denom ≈ π·c_K·ρ / MJ_MINVAL ≈
O(1e22)` would amplify floating-point residuals in the intermediate terms
(Steps 1–4). MuJoCo's `mjd_kutta_lift` has the same early return. Consistent
with forward Kutta force being zero at rest.

**Viscous drag (Component 4):** Decompose into guarded and unguarded parts:
- **Stokes diagonal** (`−lin_coef · I`): always contributed
  (velocity-independent).
- **Quadratic drag + area gradient terms:** guarded by `speed < MJ_MINVAL` →
  skip. The quadratic identity `d(|v|v)/dv = (vvᵀ + |v|²I) / |v|` and area
  gradient `dA_coef = π / √(proj_num³ · proj_denom)` both have `|v|` or
  velocity products in their denominators. MuJoCo's `mjd_viscous_drag` guards
  the quadratic/area block with a speed check, contributing only the Stokes
  diagonal at zero velocity.

**Viscous torque (Component 5):** Decompose into guarded and unguarded parts:
- **Stokes torque diagonal** (`−lin_coef · I`): always contributed.
- **Anisotropic quadratic term** (`ω[i]·mom_sq[j]` outer product + isotropic
  part): guarded by `|mom_visc| < MJ_MINVAL` → skip. The
  `density = ρ / |mom_visc|` denominator would otherwise amplify residuals.
  MuJoCo's `mjd_viscous_torque` uses the same guard. At zero angular velocity,
  only the Stokes torque diagonal contributes.

#### Test Plan

##### Reference Value Generation

MuJoCo reference values for conformance tests (T5, T13, T17, T25–T31)
were generated using the following procedure:

1. Pin MuJoCo version: **3.5.0** (matching §40 forward conformance tests)
2. Python extraction script using `mujoco` package:
   ```python
   import mujoco
   import numpy as np
   m = mujoco.MjModel.from_xml_string(MODEL_XML)
   d = mujoco.MjData(m)
   # Model must use implicit integrator for qDeriv to be populated.
   # mj_forward → mj_fwd_acceleration → mjd_passive_vel populates qDeriv.
   assert m.opt.integrator in (mujoco.mjtIntegrator.mjINT_IMPLICIT,
                                mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
   # Set qpos, qvel to test state, then run full forward dynamics
   d.qpos[:] = TEST_QPOS
   d.qvel[:] = TEST_QVEL
   mujoco.mj_forward(m, d)
   # Extract qDeriv (sparse → dense conversion)
   qDeriv_dense = np.zeros((m.nv, m.nv))
   for i in range(m.nv):
       for s in range(m.D_rownnz[i]):
           j = m.D_colind[m.D_rowadr[i] + s]
           qDeriv_dense[i, j] = d.qDeriv[m.D_rowadr[i] + s]
   ```
3. Store as `const` arrays in test source files (same pattern as §40 tests)
4. For FD validation tests (T4, T6–T12), reference values are computed by the
   test itself — no external MuJoCo dependency needed

##### FD Validation Methodology

All FD validation tests (T4, T6–T12) use centered finite differences:

```
∂f_i/∂v_j ≈ (f_i(v + ε·e_j) − f_i(v − ε·e_j)) / (2ε)
```

- **ε = 1e-6** (balances truncation vs roundoff for f64)
- Perturbation applied to each velocity component independently
- Forward force computed by calling the component function directly (not through
  full pipeline), using the same geometric constants and coefficients
- Test velocities: **non-axis-aligned** (e.g., `v = [1.2, -0.7, 0.3]`,
  `ω = [0.5, -1.1, 0.8]`) to exercise all cross-terms
- Comparison: entry-wise absolute tolerance **1e-5**

##### Test Matrix

| # | Category | Description | Verification |
|---|----------|-------------|--------------|
| **Inertia-box derivatives** ||||
| T1 | Unit | Viscous B scalars (sphere, β>0, ρ=0) | Two-part: (1) velocity-independence — FD Jacobian identical at two velocities. (2) Analytical formula validation — compute `b_angular = −π·diam³·β`, `b_linear = −3π·diam·β` from sphere geometry, verify `mjd_passive_vel` diagonal matches at 1e-12. |
| T2 | Unit | Density B scalars (sphere, β=0, ρ>0) | Analytical formula: at qvel=[0,0,0,0,0,−1] (ω_z=−1), compute `b[2] = −2ρd(d⁴+d⁴)/64·|ω_z|` from sphere geometry, verify `qDeriv[(5,5)]` matches at 1e-12. Assert other 5 diagonal entries are zero (no viscosity, no velocity). |
| T3 | Unit | Combined B scalars (box, β>0, ρ>0) | Additivity proof: run `mjd_passive_vel` on three models (combined, viscous-only ρ=0, density-only β=0) with identical box geometry. Assert `qDeriv_combined = qDeriv_viscous + qDeriv_density` at 1e-10 (exact additivity through linear J^T·diag(b)·J projection). |
| T4 | FD validation | Inertia-box B matches FD | Perturb each of 6 velocity components, compare FD of forward inertia-box force against analytical B diagonal. Tol: 1e-5. |
| T5 | D matrix | Inertia-box qDeriv matches MuJoCo | Free-floating box, non-zero velocity. Extract `qDeriv` after `mjd_passive_vel`, compare against MuJoCo reference. Tol: 1e-10. |
| **Ellipsoid derivatives** ||||
| T6 | Unit | `mjd_cross` helper | Verify `∂(a×b)/∂a` and `∂(a×b)/∂b` against FD perturbation. Multiple vector pairs including zero components. |
| T7 | Unit | Added mass derivatives | Single ellipsoid, non-zero ω and v. Compare Q(0,0), Q(0,1), Q(1,0), Q(1,1) from `mjd_addedMassForces` against FD of forward added-mass forces. |
| T8 | Unit | Magnus derivatives | Verify `mjd_magnus_force` Q(1,0) and Q(1,1) against FD. |
| T9 | Unit | Kutta lift derivatives | Verify `mjd_kutta_lift` Q(1,1) against FD. Use non-axis-aligned velocity to exercise all cross-terms and avoid degenerate proj_denom. |
| T10 | Unit | Viscous drag derivatives | Verify `mjd_viscous_drag` Q(1,1) against FD. Both viscous-only (β>0,ρ=0) and combined (β>0,ρ>0) cases. |
| T11 | Unit | Viscous torque derivatives | Verify `mjd_viscous_torque` Q(0,0) against FD. |
| T12 | FD validation | Full ellipsoid 6×6 B matches FD | Assemble all 5 components into B. Perturb 6D velocity, compute full 6×6 FD Jacobian. Compare entry-wise. Tol: 1e-5. |
| T13 | D matrix | Ellipsoid qDeriv matches MuJoCo | Free-floating ellipsoid, non-zero velocity. Extract `qDeriv`, compare against MuJoCo. Tol: 1e-10. |
| T14 | D matrix | Multi-geom body | Body with 2 ellipsoid geoms. Verify additive accumulation via FD validation. |
| **Symmetrization** ||||
| T15 | Symmetry | ImplicitFast B is symmetric | Ellipsoid B after symmetrization: `B[i][j] == B[j][i]` for all entries. |
| T16 | Asymmetry | Full Implicit B is not symmetrized | Quantified: `max_{i<j} |qDeriv[i,j]−qDeriv[j,i]| > 1e-4` (added-mass gyroscopic cross-terms produce O(1) asymmetry). Localized: at least one asymmetric pair in the linear-angular coupling block (rows 0–2, cols 3–5). |
| **Integration / whole-pipeline** ||||
| T17 | Conformance | ImplicitFast qDeriv matches MuJoCo | Free-floating ellipsoid with `density=1.2`, `integrator="implicitfast"`. Compare fluid-only `qDeriv` against MuJoCo reference. Tol: 1e-8. |
| ~~T18~~ | ~~Conformance~~ | ~~Implicit qDeriv matches MuJoCo~~ | Not implemented separately — T13 covers implicit ellipsoid conformance. |
| T19 | Stability | Implicit + fluid energy dissipation | §40 `MODEL_A` (`fluid_forces.rs:36`: free-floating sphere, `density=1.2`), qvel = `[1.0, -0.5, 0.3, 0.5, -1.1, 0.8]`, `dt=0.002`, `integrator="implicit"`. Step 1000 frames. Acceptance: `KE[t+1] ≤ KE[t] + 1e-12` at every step (machine-epsilon drift allowed). Failure if any single-step KE increase exceeds 1e-12. |
| T20 | Stability | ImplicitFast + fluid energy dissipation | Same configuration as T19 with `integrator="implicitfast"`. Same acceptance criterion. |
| **Guards and edge cases** ||||
| T21 | Gate | Zero fluid → no D contribution | `density=0, viscosity=0` → `qDeriv` unchanged by `mjd_fluid_vel`. |
| T22 | Gate | Zero velocity → viscous-only B | Four checks: (1) all finite, (2) all diagonals negative, (3) diagonal values match analytical viscous B scalars (`−π·diam³·β` angular, `−3π·diam·β` linear) from box geometry at 1e-12, (4) all off-diagonal entries zero (inertia-box at identity qpos, free joint, zero velocity → diagonal-only qDeriv). |
| T23 | Guard | Massless body skipped | `body_mass < MJ_MINVAL` → no fluid derivative contribution. |
| T24 | Guard | `interaction_coef=0` → inertia-box fallback | Equivalence proof: run `mjd_passive_vel` on `fluidshape="none"` model and matching plain inertia-box model (same ellipsoid geometry, no fluidshape attr). Assert identical qDeriv at 1e-10 — proves ellipsoid path was skipped, inertia-box fallback ran identically. |
| **Jacobian reference point and geometry** ||||
| T25 | Conformance | Geom offset from body CoM | Ellipsoid geom with `pos="1 0.5 0"` (offset from body origin). Verify qDeriv matches MuJoCo — confirms Jacobian is computed at `geom_xpos`, not `xipos`. Tol: 1e-10. |
| T26 | Conformance | Wind + fluid derivative | Non-zero `wind="5 0 0"`, free-floating ellipsoid with velocity. Verify qDeriv matches MuJoCo — confirms velocity is correctly wind-subtracted before derivative computation. Tol: 1e-10. |
| T27 | Conformance | Mixed-type multi-body | Model with body A (inertia-box, no fluidshape) and body B (ellipsoid, fluidshape="ellipsoid"). Both moving. Verify both contribute to qDeriv independently and match MuJoCo. Tol: 1e-10. |
| T28 | Edge case | Single-axis velocity | Only `v_x ≠ 0`, all other components zero. Tests degenerate Kutta/viscous-drag proj_denom. Verify qDeriv matches MuJoCo (guards produce correct result, no NaN). Tol: 1e-10. |
| **Joint-type and multi-body coverage** ||||
| T29 | D matrix | Hinge joint Jacobian | Single body on hinge joint, inertia-box, density=1.2. Verify 1D Jacobian row produces correct rank-1 qDeriv update (single nonzero entry). Compare against MuJoCo. Tol: 1e-10. |
| T30 | D matrix | Ball joint Jacobian | Single body on ball joint, ellipsoid geom. Verify 3-DOF Jacobian structure in 3×3 qDeriv block. Compare against MuJoCo. Tol: 1e-10. |
| T31 | D matrix | Multi-body chain | 3-body chain (hinge joints), density=1000, non-zero velocity. Verify qDeriv has contributions from all 3 bodies at correct DOF intersections (lower-triangular ancestor pattern). Compare against MuJoCo. Tol: 1e-10. |
| **Jacobian infrastructure** ||||
| T32 | Unit | `mj_jac_point` matches `mj_jac_site` | Place a site at body CoM. Verify `mj_jac_point(body_id, xipos)` rows 0–2 match `jac_rot` and rows 3–5 match `jac_trans` from `mj_jac_site`. Test on a 3-body chain (hinge joints) to exercise ancestor chain-walk. Entry-wise exact match (0.0 tolerance). |
| **Regression** ||||
| ~~T33~~ | ~~Regression~~ | ~~§40 T1–T42 still pass~~ | Verified implicitly — all 773 conformance tests pass after implementation. Not a separate test. |
| ~~T34~~ | ~~Regression~~ | ~~Existing derivative tests pass~~ | Verified implicitly — full `cargo test -p sim-conformance-tests` green. Not a separate test. |

##### Tolerance Justification

- **1e-10** (MuJoCo direct comparison, T5/T13/T14/T25–T31): Both implementations
  use identical analytical formulas operating on identical floating-point state.
  Only source of error is accumulation order differences. This matches the
  tolerance used throughout the existing derivative conformance tests.
- **1e-8** (pipeline conformance, T17): Full pipeline runs forward kinematics
  → RNE → passive → derivatives, accumulating rounding across more stages.
  Slightly relaxed from 1e-10 to account for cross-stage accumulation.
- **1e-5** (FD validation, T4/T6–T12): Central finite differences with `ε=1e-6`
  have truncation error `O(ε²) ≈ 1e-12` but the perturbation interacts with
  `max(MJ_MINVAL, ·)` guards in nonlinear terms (Kutta, area gradient), producing
  `O(ε)` error at guard boundaries. 1e-5 allows for this while still catching
  formula errors (which typically produce `O(1)` discrepancies).

#### Cross-references

- §40 S9: original deferral note
- §13 D.5 (`future_work_4.md:3632`): implicit integration known gap #5
- §13 known limitation #3 (`future_work_4.md:3697`): listed as acceptable gap
- MuJoCo `engine_derivative.c`: `mjd_passive_vel`, `mjd_inertiaBoxFluid`, `mjd_ellipsoidFluid`
- MuJoCo `engine_derivative.c`: `mjd_cross`, `mjd_magnus_force`, `mjd_kutta_lift`, `mjd_viscous_drag`, `mjd_viscous_torque`, `mjd_addedMassForces`

**Deferred follow-ups (performance, not correctness):**
- §40c: Sleep filtering for fluid derivatives (S7 deferral)
- §40d: Sparse Jacobian support for fluid derivatives (S6 deferral)
- §40e: Refactor `mj_jac_site` to use `mj_jac_point` kernel (S3 deferral)

#### Files Modified

| File | Action | Changes |
|------|--------|---------|
| `sim/L0/core/src/derivatives.rs` | modified | Added `mjd_fluid_vel()` (top-level dispatch), `mjd_inertia_box_fluid()`, `mjd_ellipsoid_fluid()`, and 5 component Jacobian helpers (`mjd_cross`, `mjd_magnus_force`, `mjd_kutta_lift`, `mjd_viscous_drag`, `mjd_viscous_torque`, `mjd_added_mass_forces`). Added `add_jtbj()`, `add_rank1()`, `add_to_quadrant()`, `rotate_jac_to_local()` helpers. Inserted `mjd_fluid_vel` call at top of `mjd_passive_vel` (before per-DOF damping). Promoted `mjd_passive_vel` to `pub`. |
| `sim/L0/core/src/mujoco_pipeline.rs` | modified | Added `mj_jac_point()` shared kernel, `mj_jac_body_com()`, `mj_jac_geom()`. Made `object_velocity_local()`, `fluid_geom_semi_axes()`, `ellipsoid_moment()`, `norm3()`, `MJ_MINVAL` `pub(crate)` so `derivatives.rs` can reuse them. |
| `sim/L0/core/src/lib.rs` | modified | Added `mjd_passive_vel`, `mj_jac_point`, `mj_jac_site` to public re-exports. |
| `sim/L0/tests/integration/fluid_derivatives.rs` | **new** | 31 derivative-specific tests (T1–T32, excluding T18/T33/T34). |
| `sim/L0/tests/integration/mod.rs` | modified | Registered `fluid_derivatives` module. |

#### Implementation Phases

| Phase | Scope | Deliverable | Status |
|-------|-------|-------------|--------|
| P1 | Infrastructure | `mj_jac_point`, `mj_jac_body_com`, `mj_jac_geom`, `rotate_jac_to_local`, `add_jtbj`, `add_rank1`, `add_to_quadrant`, `mjd_cross`. Unit tests T6, T32. | ✅ |
| P2 | Inertia-box derivatives | `mjd_inertia_box_fluid`. Tests T1–T5. | ✅ |
| P3 | Ellipsoid component derivatives | 5 component Jacobian functions. Tests T7–T11. | ✅ |
| P4 | Ellipsoid assembly + integration | `mjd_ellipsoid_fluid`, symmetrization, `mjd_fluid_vel` dispatch. Tests T12–T16. | ✅ |
| P5 | Pipeline integration + conformance | Wire into `mjd_passive_vel`. Tests T17–T31. | ✅ |

---

### 40b. Tendon Visualization Data (`wrap_xpos`, `wrap_obj`)
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

The spatial tendon pipeline (`mj_fwd_tendon_spatial()`, line ~9396 of
`mujoco_pipeline.rs`) computes correct physics — length, Jacobian, forces —
but discards the intermediate geometric results needed for visualization.
Specifically, tangent points computed during `sphere_wrap()` / `cylinder_wrap()`
are transformed to world frame (lines 9521–9522) and used for Jacobian
accumulation, then thrown away.

MuJoCo stores these path points in four `Data` arrays:
- `d->wrap_xpos[]` — world-frame 3D positions of all tendon path points
- `d->wrap_obj[]` — object marker per path point (`-2` = pulley, `-1` = site,
  `≥ 0` = geom id)
- `d->ten_wrapadr[i]` — start index in `wrap_xpos`/`wrap_obj` for tendon `i`
- `d->ten_wrapnum[i]` — number of path points for tendon `i`

None of these fields exist in our `Data` struct. Without them, downstream
consumers (visualization, debugging, path inspection) cannot reconstruct the
tendon path through wrapping geometry.

**Provenance:** Identified as a known limitation during §4 (spatial tendon
pipeline, `future_work_2.md`) and deferred from §39 (`wrap_inside`).

#### Objective

Add `wrap_xpos`, `wrap_obj`, `ten_wrapadr`, and `ten_wrapnum` fields to `Data`.
Populate them during `mj_fwd_tendon_spatial()` by capturing the path points
that are already computed but currently discarded.

#### MuJoCo Reference — Data Layout

Source: `engine_core_smooth.c:mj_tendon()`, `mjdata.h`.

**Allocation (in `mjData`):**
```
wrap_xpos:  mjtNum*  (allocated: nwrap * 6 floats = nwrap * 2 points * 3 coords)
wrap_obj:   int*     (allocated: nwrap * 2 ints)
ten_wrapadr: int*    (length: ntendon)
ten_wrapnum: int*    (length: ntendon)
```

Where `nwrap` is the **model**'s total wrap path elements (sites + geoms +
pulleys across all tendons). The `2x` factor is an upper bound: a Site
produces at most 1 leading + 1 final = 2 data points; a Geom produces 0
(no-wrap) or 2 (tangent points) data points; a Pulley produces 1. No model
element exceeds 2 data points, so `nwrap * 2` is always sufficient.

**Indexing:** `wrap_xpos` is indexed by `wrapcount * 3` (3 floats per point).
`wrap_obj` is indexed by `wrapcount` (1 int per point). `ten_wrapadr[t]` gives
the starting `wrapcount` for tendon `t`; `ten_wrapnum[t]` gives the count.

**Per-segment path point rules (from `engine_core_smooth.c:mj_tendon()`):**

| Segment type | Points stored | `wrap_obj` values | Notes |
|--------------|---------------|-------------------|-------|
| **Pulley** (type0 == Pulley) | 1: zero position `[0,0,0]` | `[-2]` | Marks divisor change |
| **Site–Site** (straight, no geom) | 1: site0 world pos | `[-1]` | Only the leading site |
| **Site–Geom–Site** (wrapping, `wlen ≥ 0`) | 3: site0, tangent₁, tangent₂ | `[-1, geom_id, geom_id]` | Tangents in world frame |
| **Site–Geom–Site** (no wrap, `wlen < 0`) | 1: site0 world pos | `[-1]` | Falls back to straight |
| **Final site** (last in path, or before pulley) | 1: site1 world pos | `[-1]` | Closes the path |

**Example traces:**

*Model B (Site–Sphere–Site, wrapping occurs):*
```
Path elements: [Site(s0), Geom(sphere), Site(s1)]
Data points:   [s0_pos, tangent₁, tangent₂, s1_pos]
wrap_obj:      [-1,     geom_id,  geom_id,  -1]
ten_wrapnum:   4
```

*Model A (Site–Site, straight):*
```
Path elements: [Site(s0), Site(s1)]
Data points:   [s0_pos, s1_pos]
wrap_obj:      [-1,     -1]
ten_wrapnum:   2
```

*Model D (Site–Site–Pulley–Site–Site):*
```
Path elements: [Site(s1), Site(s2), Pulley(div=2), Site(s3), Site(s4)]
Data points:   [s1_pos, s2_pos, [0,0,0], s3_pos, s4_pos]
wrap_obj:      [-1,     -1,     -2,      -1,     -1]
ten_wrapnum:   5
```

*Model F (Site–Site–Site, multi-site):*
```
Path elements: [Site(s1), Site(s2), Site(s3)]
Data points:   [s1_pos, s2_pos, s3_pos]
wrap_obj:      [-1,     -1,     -1]
ten_wrapnum:   3
```

*Model B at qpos where no wrapping occurs:*
```
Path elements: [Site(s0), Geom(sphere), Site(s1)]
Data points:   [s0_pos, s1_pos]    (geom skipped — no wrap)
wrap_obj:      [-1,     -1]
ten_wrapnum:   2
```

#### Specification

##### S1. Data struct fields

Add to `Data` (in the "Tendon State" section, after `ten_J`):

```rust
/// Tendon wrap path point positions in world frame.
/// Flat storage: `wrap_xpos[ten_wrapadr[t] .. ten_wrapadr[t] + ten_wrapnum[t]]`
/// gives the ordered path points for tendon `t`.
/// Allocated with capacity `nwrap * 2` (upper bound).
pub wrap_xpos: Vec<Vector3<f64>>,

/// Object marker for each wrap path point.
/// -2 = pulley, -1 = site, ≥ 0 = geom id (tangent point on wrapping surface).
/// Parallel to `wrap_xpos`.
pub wrap_obj: Vec<i32>,

/// Start index in `wrap_xpos`/`wrap_obj` for each tendon (length `ntendon`).
pub ten_wrapadr: Vec<usize>,

/// Number of path points for each tendon (length `ntendon`).
pub ten_wrapnum: Vec<usize>,
```

**Design choice — `Vec<Vector3<f64>>` vs flat `Vec<f64>`:** Our codebase uses
`Vector3<f64>` for all spatial positions (`site_xpos`, `geom_xpos`, `xpos`).
Using `Vec<Vector3<f64>>` for `wrap_xpos` is consistent and type-safe. MuJoCo
uses flat `mjtNum*` with `*3` indexing — the semantic mapping is direct.

**Clone impl:** `Data` has a manual `Clone` impl (line ~2691). All four new
fields must be added there (after `ten_J`, line ~2756). This is a `.clone()`
call for each `Vec` field — follows the existing pattern.

**`reset()` not affected:** `wrap_xpos`/`wrap_obj` are computed state
recomputed on every `forward()` call (like `xpos`, `geom_xpos`). `reset()`
(line ~4369) does not need modification.

##### S2. Allocation in `make_data()`

In `make_data()`, allocate with the `nwrap * 2` upper bound:

```rust
// Tendon wrap visualization data
wrap_xpos: vec![Vector3::zeros(); self.nwrap * 2],
wrap_obj: vec![0i32; self.nwrap * 2],
ten_wrapadr: vec![0usize; self.ntendon],
ten_wrapnum: vec![0usize; self.ntendon],
```

The actual number of path points is computed at runtime and stored in
`ten_wrapnum`. The allocated capacity (`nwrap * 2`) is never exceeded because
each model wrap element produces at most 2 data points.

##### S3. Signature change for `mj_fwd_tendon_spatial()`

The current signature:
```rust
fn mj_fwd_tendon_spatial(model: &Model, data: &mut Data, t: usize)
```

Add a `wrapcount: &mut usize` parameter to track the global write cursor
across tendon calls:

```rust
fn mj_fwd_tendon_spatial(model: &Model, data: &mut Data, t: usize, wrapcount: &mut usize)
```

The caller (`mj_fwd_tendon` or the tendon loop) initializes `wrapcount = 0`
before the first tendon and passes it through each call.

##### S4. Population logic in `mj_fwd_tendon_spatial()`

At the top of the function, record the start address:

```rust
data.ten_wrapadr[t] = *wrapcount;
data.ten_wrapnum[t] = 0;
```

Then at each point in the existing control flow, store path points:

**Pulley case** (existing lines 9417–9423):
```rust
if type0 == WrapType::Pulley {
    divisor = model.wrap_prm[adr + j];
    // §40b: store pulley marker
    data.wrap_xpos[*wrapcount] = Vector3::zeros();
    data.wrap_obj[*wrapcount] = -2;
    data.ten_wrapnum[t] += 1;
    *wrapcount += 1;
}
```

**Site–Site case** (existing lines 9434–9469, store leading site):
```rust
// Before the existing distance/Jacobian computation:
data.wrap_xpos[*wrapcount] = p0;
data.wrap_obj[*wrapcount] = -1;
data.ten_wrapnum[t] += 1;
*wrapcount += 1;
```

**Site–Geom–Site, `WrapResult::Wrapped`** (existing lines 9515–9583):
```rust
// After computing t1, t2 in world frame (lines 9521–9522):
// Store 3 points: site0, tangent₁, tangent₂
data.wrap_xpos[*wrapcount] = p0;
data.wrap_obj[*wrapcount] = -1;
data.wrap_xpos[*wrapcount + 1] = t1;
data.wrap_obj[*wrapcount + 1] = geom_id as i32;
data.wrap_xpos[*wrapcount + 2] = t2;
data.wrap_obj[*wrapcount + 2] = geom_id as i32;
data.ten_wrapnum[t] += 3;
*wrapcount += 3;
```

**Site–Geom–Site, `WrapResult::NoWrap`** (existing lines 9584–9612):
```rust
// Store 1 point: site0 only (straight line, no tangent points)
data.wrap_xpos[*wrapcount] = p0;
data.wrap_obj[*wrapcount] = -1;
data.ten_wrapnum[t] += 1;
*wrapcount += 1;
```

**Final site** — after the `j` advance at the end of each iteration, check
if the current segment terminates the path or precedes a pulley:
```rust
// After j += 1 (Site-Site) or j += 2 (Site-Geom-Site):
let at_end = j == num - 1;
let before_pulley = j < num - 1 && model.wrap_type[adr + j + 1] == WrapType::Pulley;
if at_end || before_pulley {
    // id1 is the endpoint site from the segment just processed
    data.wrap_xpos[*wrapcount] = data.site_xpos[id1];
    data.wrap_obj[*wrapcount] = -1;
    data.ten_wrapnum[t] += 1;
    *wrapcount += 1;
}
```

**Note on `id1` scope:** The variable `id1` must be accessible at the final-site
check point. In the current code, `id1` is scoped inside the `if type1 == Site`
and `if type1 == Geom` branches. Two options:
1. Lift `id1` to the loop scope (set in both branches).
2. Re-derive: `id1 = model.wrap_objid[adr + j]` (since `j` was just advanced
   past the segment's endpoint site).

Option 1 is cleaner and matches MuJoCo's approach.

##### S5. Caller-side wiring

The tendon loop in the forward pass (wherever `mj_fwd_tendon_spatial` is called
for each tendon) must:

```rust
let mut wrapcount: usize = 0;
for t in 0..model.ntendon {
    match model.tendon_type[t] {
        TendonType::Spatial => {
            mj_fwd_tendon_spatial(model, data, t, &mut wrapcount);
        }
        TendonType::Fixed => {
            // Fixed tendons have no wrap path — zero out
            data.ten_wrapadr[t] = wrapcount;
            data.ten_wrapnum[t] = 0;
            // (existing fixed tendon logic unchanged)
        }
    }
}
```

##### S6. Degenerate tendon (num < 2)

For the early return at line 9403 (`if num < 2`), still set the wrap address:
```rust
data.ten_wrapadr[t] = *wrapcount;
data.ten_wrapnum[t] = 0;
```

##### S7. Inside-wrap handling (§39)

When `sphere_wrap` or `cylinder_wrap` returns `WrapResult::Wrapped` with
`tangent_point_1 == tangent_point_2` and `arc_length == 0.0` (the inside-wrap
case from §39), the path point storage is identical to the normal wrapped case:
3 points `[site0, tangent, tangent]` with `wrap_obj = [-1, geom_id, geom_id]`.
The two tangent points happen to be identical, but the storage format is
unchanged. No special-casing needed.

#### Acceptance Criteria

##### T1. Field existence and allocation
- `Data` has fields `wrap_xpos`, `wrap_obj`, `ten_wrapadr`, `ten_wrapnum`.
- After `make_data()`, `wrap_xpos.len() == nwrap * 2`,
  `wrap_obj.len() == nwrap * 2`, `ten_wrapadr.len() == ntendon`,
  `ten_wrapnum.len() == ntendon`.

##### T2. Straight tendon path (Model A: Site–Site)
```
After forward(): ten_wrapnum[0] == 2
  wrap_xpos[0] == site_xpos[s1_id]  (leading site)
  wrap_xpos[1] == site_xpos[s2_id]  (final site)
  wrap_obj[0] == -1, wrap_obj[1] == -1
```

##### T3. Multi-site straight tendon (Model F: Site–Site–Site)
```
After forward(): ten_wrapnum[0] == 3
  wrap_xpos[0..3] == [s1_pos, s2_pos, s3_pos]
  wrap_obj[0..3] == [-1, -1, -1]
```

##### T4. Sphere wrapping (Model B: Site–Sphere–Site)
```
After forward(): ten_wrapnum[0] == 4
  wrap_xpos[0] == origin site position
  wrap_xpos[1] == tangent₁ (on sphere surface, world frame)
  wrap_xpos[2] == tangent₂ (on sphere surface, world frame)
  wrap_xpos[3] == insertion site position
  wrap_obj == [-1, geom_id, geom_id, -1]
Verify: |tangent₁| ≈ sphere_radius (within geom frame)
Verify: tangent₁ and tangent₂ differ (not inside-wrap case at default qpos)
```

##### T5. Cylinder wrapping (Model C: Site–Cylinder–Site)
```
After forward(): ten_wrapnum[0] == 4
  wrap_xpos layout same as T4 but for cylinder
  wrap_obj == [-1, cyl_geom_id, cyl_geom_id, -1]
Verify: tangent points lie on cylinder surface (distance from axis ≈ radius)
```

##### T6. Pulley (Model D: Site–Site–Pulley–Site–Site)
```
After forward(): ten_wrapnum[0] == 5
  wrap_xpos[0] == s1_pos
  wrap_xpos[1] == s2_pos  (final site of first branch)
  wrap_xpos[2] == [0,0,0] (pulley marker)
  wrap_xpos[3] == s3_pos
  wrap_xpos[4] == s4_pos  (final site of second branch)
  wrap_obj == [-1, -1, -2, -1, -1]
```

##### T7. No-wrap fallback (Site–Geom–Site where straight path clears geom)
Construct or configure Model B so wrapping does not occur (e.g., sites far
from sphere). Verify `ten_wrapnum[0] == 2` (site0 + final site1, no tangent
points stored).

##### T8. Inside-wrap (§39 sidesite-inside models)
For the existing inside-wrap test models (sphere inside, cylinder inside):
```
ten_wrapnum[0] == 4
wrap_xpos[1] == wrap_xpos[2]  (identical tangent points)
wrap_obj[1] == wrap_obj[2] == geom_id
```

##### T9. Indexing consistency
For all test models: verify `ten_wrapadr[t] + ten_wrapnum[t] == ten_wrapadr[t+1]`
(or `== total_wrapcount` for the last tendon). This ensures contiguous,
non-overlapping storage.

##### T10. MuJoCo conformance — wrap_xpos positions
For Models B, C, E, H, I, J (sphere/cylinder/sidesite/collinear wrapping at
default qpos): compare `wrap_xpos` tangent point positions against MuJoCo 3.x
reference values with tolerance `1e-6` (positions, not lengths — tangent
computation involves transcendental functions so exact match is not expected).

Reference values must be extracted from MuJoCo and embedded as test constants,
following the same pattern as the existing `ten_length` conformance values.

##### T11. Stepped conformance
For Model B: step the simulation 5 times, verify `wrap_xpos` updates each step
(tangent points move as the tendon path changes). Compare against MuJoCo
reference trajectory positions.

##### T12. Fixed tendon passthrough
Fixed tendons produce `ten_wrapnum[t] == 0` and `ten_wrapadr[t]` is set
correctly (no gap in the address sequence).

##### T13. Multiple tendons
Construct a test model with 2 spatial tendons (e.g., one straight Site–Site
and one wrapped Site–Geom–Site). Verify `ten_wrapadr` and `ten_wrapnum`
correctly partition the `wrap_xpos`/`wrap_obj` arrays:
```
ten_wrapadr[0] == 0, ten_wrapnum[0] == 2  (straight: 2 points)
ten_wrapadr[1] == 2, ten_wrapnum[1] == 4  (wrapped: 4 points)
```
Each tendon's segment is independently correct and non-overlapping.

##### T14. Path reconstruction — straight segments
For Model A (straight): reconstruct the tendon path from `wrap_xpos` by
summing Euclidean distances between consecutive points. Verify the sum
equals `ten_length[0]` within `1e-10`. (For straight-only tendons, the
wrap_xpos points fully determine the path length.)

##### T15. Mixed segment model (Model L)
For Model L (mixed straight + wrapping segments): verify `ten_wrapnum`
reflects both the straight and wrapped sub-paths. The straight segment
stores 2 points; the wrapped segment stores 4 points (if wrapping occurs).
Verify `wrap_obj` markers correctly distinguish site points from tangent
points across the mixed path.

#### Implementation Phases

| Phase | Scope | Deliverable |
|-------|-------|-------------|
| P1 | Data fields + allocation | S1, S2 fields in `Data` + Clone impl, allocation in `make_data()`. T1. |
| P2 | Population logic | S3–S6 changes to `mj_fwd_tendon_spatial()` + caller. T2–T9, T12–T13, T15. |
| P3 | Inside-wrap integration | S7 verification. T8. |
| P4 | MuJoCo conformance | Extract reference values, T10–T11, T14. |

#### Files to Modify

| File | Action | Changes |
|------|--------|---------|
| `sim/L0/core/src/mujoco_pipeline.rs` | modify | Add `wrap_xpos`, `wrap_obj`, `ten_wrapadr`, `ten_wrapnum` to `Data` struct (S1). Update manual `Clone` impl (line ~2756). Allocate in `make_data()` (S2). Add `wrapcount` parameter to `mj_fwd_tendon_spatial()` and populate path points (S3–S6). Wire `wrapcount` through tendon loop (S5). |
| `sim/L0/tests/integration/spatial_tendons.rs` | modify | Add tests T2–T15 verifying wrap path points for all existing test models. Extract and embed MuJoCo reference values for tangent positions. |

#### Cross-references

- §4 spatial tendon pipeline (`future_work_2.md`): original deferral
- §39 `wrap_inside` (`future_work_10.md`): inside-wrap tangent points (S7)
- MuJoCo `engine_core_smooth.c:mj_tendon()`: reference implementation
- MuJoCo `mjdata.h`: `wrap_xpos` (`nwrap x 6`), `wrap_obj` (`nwrap x 2`)
- MuJoCo `mjmodel.h`: `nwrap`, `wrap_type`, `wrap_objid`, `wrap_prm`
- Downstream consumer: `sim/L1/bevy/src/resources.rs:441` (`TendonVisualData.path`)
  and `sim/L1/bevy/src/gizmos.rs:156` (`draw_tendons`) — primary visualization
  consumer of `wrap_xpos`. `TendonVisualData.path: Vec<Point3<f64>>` maps
  directly to `wrap_xpos[ten_wrapadr[t]..+ten_wrapnum[t]]`.

---

### 40c. Sleep Filtering for Fluid Derivatives
**Status:** Not started | **Effort:** S | **Prerequisites:** Sleep system (global)

#### Current State

MuJoCo's `mjd_passive_vel` supports sleep filtering via `d->body_awake_ind` and
`d->nbody_awake` to skip sleeping bodies during derivative computation. Our
codebase does not yet implement sleep state tracking, so `mjd_fluid_vel` iterates
all bodies unconditionally.

Deferred from §40a (S7) as a performance optimization that does not affect
correctness.

#### Objective

Once a global sleep system is implemented, gate the fluid derivative body loop
in `mjd_fluid_vel` on the body's awake state. Skip sleeping bodies whose
velocities are clamped to zero — their fluid derivatives are necessarily zero.

#### Scope

- Add `body_awake_ind` / `nbody_awake` to `Data` (or equivalent sleep state)
- Gate body loop in `mjd_fluid_vel` on awake state
- Verify no conformance regression (sleeping bodies should produce identical
  results since their velocities are zero)

#### Cross-references

- §40a S7 (`future_work_10.md:5173`): deferral note
- MuJoCo `engine_derivative.c`: `mjd_passive_vel` sleep filtering loop

---

### 40d. Sparse Jacobian Support for Fluid Derivatives
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

MuJoCo supports both dense and sparse Jacobian paths for fluid derivative
accumulation (`addJTBJ` vs `addJTBJSparse`), selectable via `mj_isSparse(m)`.
Our implementation uses dense Jacobians only (`DMatrix<f64>` for the 6×nv
Jacobian and `nv×nv` qDeriv). The current `add_jtbj` and `add_rank1` helpers
exploit Jacobian sparsity via zero-skipping but still operate on dense storage.

Deferred from §40a (S6) as a constant-factor speedup for large `nv`. Dense is
acceptable for target model sizes (`nv < 200`).

#### Objective

Implement sparse Jacobian storage and sparse `J^T·B·J` accumulation to improve
fluid derivative performance for models with `nv > 200`. This mirrors MuJoCo's
`addJTBJSparse` which operates on CSR-format qDeriv.

#### Scope

- Implement CSR or equivalent sparse storage for qDeriv
- Add `add_jtbj_sparse` and `add_rank1_sparse` variants
- Gate on model size or `mj_isSparse`-equivalent heuristic
- Benchmark: demonstrate speedup on a model with `nv ≈ 300–500`
- Verify exact numerical equivalence with dense path

#### Cross-references

- §40a S6 (`future_work_10.md:5140`): deferral note
- MuJoCo `engine_derivative.c`: `addJTBJSparse`, `mj_isSparse`

---

### 40e. Refactor `mj_jac_site` to Use `mj_jac_point` Kernel
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

`mj_jac_site` in `mujoco_pipeline.rs` implements its own chain-walk algorithm,
returning two separate 3×nv matrices `(jac_trans, jac_rot)`. The §40a
implementation added `mj_jac_point` as a shared 6×nv Jacobian kernel using the
same chain-walk pattern. Both functions are correct but the duplicated chain-walk
logic is a maintenance burden.

Deferred from §40a (S3) to avoid widening the blast radius of the fluid
derivative implementation.

#### Objective

Refactor `mj_jac_site` to call `mj_jac_point` internally and split the 6×nv
result into the existing `(jac_trans, jac_rot)` return format. This eliminates
the duplicated chain-walk code.

#### Scope

- Refactor `mj_jac_site` to delegate to `mj_jac_point`
- Split rows 0–2 (angular) → `jac_rot`, rows 3–5 (linear) → `jac_trans`
- Verify T32 (`mj_jac_point` vs `mj_jac_site` exact match) still passes
- Run full `sim-conformance-tests` to confirm no regression

#### Cross-references

- §40a S3 (`future_work_10.md:4770`): deferral note
- §40a T32: `mj_jac_point` vs `mj_jac_site` exact match test

---

### 41. `disableflags` / `enableflags` — Full Runtime Flag Wiring
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

The MJCF parser (`parser.rs:441-470`) parses 21 `<flag>` attributes into `MjcfFlag`
boolean fields. The model builder (`model_builder.rs:968-970`) wires **only**
`ENABLE_SLEEP`. `DISABLE_ISLAND` has a runtime constant but is hardcoded, not parsed
from `MjcfFlag.island`.

**Result:** All other parsed flags (constraint, equality, frictionloss, limit, contact,
passive, gravity, clampctrl, warmstart, filterparent, actuation, refsafe, sensor,
midphase, nativeccd, eulerdamp, override_contacts, energy, multiccd) are parsed into
`MjcfFlag` but **never converted to the `u32` bitfields** and never checked at runtime.

Additionally, the parser has a single `passive` field but MuJoCo 3.x uses separate
`spring` and `damper` flags. Three enable flags are not parsed: `fwdinv`, `invdiscrete`,
`autoreset`.

#### MuJoCo Reference

MuJoCo uses two `u32` bitfields (`mjtDisableBit` and `mjtEnableBit`) from
`mujoco.h`:

**Disable flags** (`mjNDISABLE = 19`):

| Constant | Bit | XML attr | Default | Pipeline effect |
|----------|-----|----------|---------|----------------|
| `mjDSBL_CONSTRAINT` | 1<<0 | `constraint` | enable | Skip **entire** constraint solver |
| `mjDSBL_EQUALITY` | 1<<1 | `equality` | enable | Skip equality constraint assembly |
| `mjDSBL_FRICTIONLOSS` | 1<<2 | `frictionloss` | enable | Skip friction loss constraint rows |
| `mjDSBL_LIMIT` | 1<<3 | `limit` | enable | Skip joint and tendon limit constraints |
| `mjDSBL_CONTACT` | 1<<4 | `contact` | enable | Skip collision detection + contact constraints |
| `mjDSBL_SPRING` | 1<<5 | `spring`† | enable | Skip passive spring forces (joint, tendon, flex) |
| `mjDSBL_DAMPER` | 1<<6 | `damper`† | enable | Skip passive damping forces (joint, tendon, flex) |
| `mjDSBL_GRAVITY` | 1<<7 | `gravity` | enable | Zero gravity vector in `mj_rne()` |
| `mjDSBL_CLAMPCTRL` | 1<<8 | `clampctrl` | enable | Skip clamping ctrl to actuator ctrlrange |
| `mjDSBL_WARMSTART` | 1<<9 | `warmstart` | enable | Skip warm-starting solver from prev solution |
| `mjDSBL_FILTERPARENT` | 1<<10 | `filterparent` | enable | Disable parent-child collision filtering |
| `mjDSBL_ACTUATION` | 1<<11 | `actuation` | enable | Skip actuator force computation |
| `mjDSBL_REFSAFE` | 1<<12 | `refsafe` | enable | Skip `solref[0] >= 2*timestep` enforcement |
| `mjDSBL_SENSOR` | 1<<13 | `sensor` | enable | Skip all sensor evaluation |
| `mjDSBL_MIDPHASE` | 1<<14 | `midphase` | enable | Skip BVH midphase (brute-force broadphase) |
| `mjDSBL_EULERDAMP` | 1<<15 | `eulerdamp` | enable | Skip implicit joint damping in Euler |
| `mjDSBL_AUTORESET` | 1<<16 | `autoreset` | enable | Skip auto-reset on NaN/divergence |
| `mjDSBL_NATIVECCD` | 1<<17 | `nativeccd` | enable | Fall back to libccd for convex collision |
| `mjDSBL_ISLAND` | 1<<18 | `island` | disable | Skip island discovery (global solve) |

†MuJoCo 3.x split the old `passive` flag into separate `spring` and `damper` flags.

**Enable flags** (`mjNENABLE = 6`):

| Constant | Bit | XML attr | Default | Pipeline effect |
|----------|-----|----------|---------|----------------|
| `mjENBL_OVERRIDE` | 1<<0 | `override` | disable | Enable contact parameter override |
| `mjENBL_ENERGY` | 1<<1 | `energy` | disable | Enable potential + kinetic energy computation |
| `mjENBL_FWDINV` | 1<<2 | `fwdinv` | disable | Enable forward/inverse comparison stats |
| `mjENBL_INVDISCRETE` | 1<<3 | `invdiscrete` | disable | Discrete-time inverse dynamics |
| `mjENBL_MULTICCD` | 1<<4 | `multiccd` | disable | Multi-point CCD for flat surfaces |
| `mjENBL_SLEEP` | 1<<5 | `sleep` | disable | Body sleeping/deactivation |

#### Objective

1. Define all disable/enable flag constants matching MuJoCo bit assignments
2. Wire parsed `MjcfFlag` fields to `Model.disableflags` / `Model.enableflags`
3. Add runtime flag checks at each pipeline stage
4. Update parser: split `passive` into `spring`/`damper`, add missing enable flags

#### Specification

##### S1. Flag constants

Define in `mujoco_pipeline.rs`:

```rust
// Disable flags (mjtDisableBit)
pub const DISABLE_CONSTRAINT:   u32 = 1 << 0;
pub const DISABLE_EQUALITY:     u32 = 1 << 1;
pub const DISABLE_FRICTIONLOSS: u32 = 1 << 2;
pub const DISABLE_LIMIT:        u32 = 1 << 3;
pub const DISABLE_CONTACT:      u32 = 1 << 4;
pub const DISABLE_SPRING:       u32 = 1 << 5;
pub const DISABLE_DAMPER:       u32 = 1 << 6;
pub const DISABLE_GRAVITY:      u32 = 1 << 7;
pub const DISABLE_CLAMPCTRL:    u32 = 1 << 8;
pub const DISABLE_WARMSTART:    u32 = 1 << 9;
pub const DISABLE_FILTERPARENT: u32 = 1 << 10;
pub const DISABLE_ACTUATION:    u32 = 1 << 11;
pub const DISABLE_REFSAFE:      u32 = 1 << 12;
pub const DISABLE_SENSOR:       u32 = 1 << 13;
pub const DISABLE_MIDPHASE:     u32 = 1 << 14;
pub const DISABLE_EULERDAMP:    u32 = 1 << 15;
pub const DISABLE_AUTORESET:    u32 = 1 << 16;
pub const DISABLE_NATIVECCD:    u32 = 1 << 17;
// DISABLE_ISLAND already exists at 1 << 18

// Enable flags (mjtEnableBit)
pub const ENABLE_OVERRIDE:    u32 = 1 << 0;
pub const ENABLE_ENERGY:      u32 = 1 << 1;
pub const ENABLE_FWDINV:      u32 = 1 << 2;
pub const ENABLE_INVDISCRETE: u32 = 1 << 3;
pub const ENABLE_MULTICCD:    u32 = 1 << 4;
// ENABLE_SLEEP already exists at 1 << 5
```

##### S2. Parser update

Update `MjcfFlag` to split `passive` into `spring` + `damper`:
```rust
pub spring: bool,   // was: passive (default true = enabled)
pub damper: bool,   // was: passive (default true = enabled)
```

For backward compatibility, parse both:
- `passive="disable"` → sets both `spring=false` and `damper=false`
- `spring="disable"` → sets only `spring=false`
- `damper="disable"` → sets only `damper=false`

Add missing enable flag fields:
```rust
pub fwdinv: bool,       // default false (disabled)
pub invdiscrete: bool,  // default false (disabled)
pub autoreset: bool,    // default true (enabled) — disable flag, not enable
```

##### S3. Model builder wiring

In `apply_options()`, convert all `MjcfFlag` booleans to bitfields:

```rust
fn flags_to_bits(flag: &MjcfFlag) -> (u32, u32) {
    let mut disable: u32 = 0;
    let mut enable: u32 = 0;

    if !flag.constraint   { disable |= DISABLE_CONSTRAINT; }
    if !flag.equality     { disable |= DISABLE_EQUALITY; }
    if !flag.frictionloss { disable |= DISABLE_FRICTIONLOSS; }
    if !flag.limit        { disable |= DISABLE_LIMIT; }
    if !flag.contact      { disable |= DISABLE_CONTACT; }
    if !flag.spring       { disable |= DISABLE_SPRING; }
    if !flag.damper       { disable |= DISABLE_DAMPER; }
    if !flag.gravity      { disable |= DISABLE_GRAVITY; }
    if !flag.clampctrl    { disable |= DISABLE_CLAMPCTRL; }
    if !flag.warmstart    { disable |= DISABLE_WARMSTART; }
    if !flag.filterparent { disable |= DISABLE_FILTERPARENT; }
    if !flag.actuation    { disable |= DISABLE_ACTUATION; }
    if !flag.refsafe      { disable |= DISABLE_REFSAFE; }
    if !flag.sensor       { disable |= DISABLE_SENSOR; }
    if !flag.midphase     { disable |= DISABLE_MIDPHASE; }
    if !flag.eulerdamp    { disable |= DISABLE_EULERDAMP; }
    if !flag.autoreset    { disable |= DISABLE_AUTORESET; }
    if !flag.nativeccd    { disable |= DISABLE_NATIVECCD; }
    if !flag.island       { disable |= DISABLE_ISLAND; }

    if flag.override_contacts { enable |= ENABLE_OVERRIDE; }
    if flag.energy            { enable |= ENABLE_ENERGY; }
    if flag.fwdinv            { enable |= ENABLE_FWDINV; }
    if flag.invdiscrete       { enable |= ENABLE_INVDISCRETE; }
    if flag.multiccd          { enable |= ENABLE_MULTICCD; }
    if flag.sleep             { enable |= ENABLE_SLEEP; }

    (disable, enable)
}
```

##### S4. Runtime flag gates

Add flag checks at each pipeline stage. Pattern:
```rust
let disabled = |flag: u32| model.disableflags & flag != 0;
```

| Stage | Guard | Location |
|-------|-------|----------|
| Gravity in `mj_rne()` | `if disabled(DISABLE_GRAVITY) { gravity = [0;3]; }` | bias force computation |
| Collision detection | `if disabled(DISABLE_CONTACT) { skip mj_collision(); }` | collision driver |
| Contact constraint assembly | `if disabled(DISABLE_CONTACT) { skip contact rows; }` | `mj_fwd_constraint()` |
| Equality constraints | `if disabled(DISABLE_EQUALITY) { skip equality rows; }` | `mj_fwd_constraint()` |
| Joint/tendon limits | `if disabled(DISABLE_LIMIT) { skip limit rows; }` | `mj_fwd_constraint()` |
| Friction loss rows | `if disabled(DISABLE_FRICTIONLOSS) { skip floss rows; }` | `mj_fwd_constraint()` |
| Entire constraint solver | `if disabled(DISABLE_CONSTRAINT) { skip solver; }` | constraint solve dispatch |
| Passive springs | `if disabled(DISABLE_SPRING) { skip spring forces; }` | `mj_fwd_passive()` — joint, tendon, flex springs |
| Passive dampers | `if disabled(DISABLE_DAMPER) { skip damper forces; }` | `mj_fwd_passive()` — joint, tendon, flex damping |
| Actuator forces | `if disabled(DISABLE_ACTUATION) { skip mj_fwd_actuation(); }` | actuation stage |
| Ctrl clamping | `if disabled(DISABLE_CLAMPCTRL) { skip ctrl clamp; }` | actuator ctrl processing |
| Sensor evaluation | `if disabled(DISABLE_SENSOR) { skip mj_sensor_*(); }` | sensor stage |
| Warm-start | `if disabled(DISABLE_WARMSTART) { zero warmstart; }` | solver initialization |
| Parent-child filter | `if disabled(DISABLE_FILTERPARENT) { skip filter; }` | collision filtering |
| BVH midphase | `if disabled(DISABLE_MIDPHASE) { brute force; }` | collision broadphase |
| Euler implicit damp | `if disabled(DISABLE_EULERDAMP) { skip euler damp; }` | Euler integrator |
| Solref safety | `if disabled(DISABLE_REFSAFE) { skip solref clamp; }` | constraint parameterization |
| Auto-reset on NaN | `if disabled(DISABLE_AUTORESET) { skip reset; }` | integration step |
| Native CCD | `if disabled(DISABLE_NATIVECCD) { use libccd; }` | narrowphase dispatch |
| Island discovery | Already wired (`DISABLE_ISLAND`) | constraint islands |

**Spring/damper separation** (critical): MuJoCo applies these as multiplicative
factors on the force, not as skip-the-loop flags. The pattern is:

```rust
let has_spring = !disabled(DISABLE_SPRING);
let has_damper = !disabled(DISABLE_DAMPER);
// In passive force loop:
let spring_force = if has_spring { stiffness * displacement } else { 0.0 };
let damper_force = if has_damper { damping * velocity } else { 0.0 };
qfrc_passive[dof] += spring_force + damper_force;
```

This applies to: joint springs/dampers, tendon springs/dampers, flex edge
spring/damper forces (#27C), flex bending forces (spring component), flex
vertex damping.

##### S5. Enable flag gates

| Stage | Guard | Location |
|-------|-------|----------|
| Energy computation | Already partially wired (`ENABLE_ENERGY`) | energy accumulators |
| Sleep/deactivation | Already wired (`ENABLE_SLEEP`) | sleep logic |
| Override contacts | `if enabled(ENABLE_OVERRIDE) { use override params; }` | contact parameter combination |
| Forward/inverse stats | `if enabled(ENABLE_FWDINV) { compute inverse; }` | post-step stats |
| Discrete inverse | `if enabled(ENABLE_INVDISCRETE) { ... }` | inverse dynamics |
| Multi-CCD | `if enabled(ENABLE_MULTICCD) { multi-point CCD; }` | narrowphase |

#### Acceptance Criteria

1. **Contact disable**: `<flag contact="disable"/>` produces zero contacts. No
   collision detection runs. `data.ncon == 0`. Existing non-contact forces
   (gravity, springs) still apply.

2. **Gravity disable**: `<flag gravity="disable"/>` zeros gravitational force.
   A free body with only gravity remains stationary. Other forces (springs,
   actuators) still apply.

3. **Limit disable**: `<flag limit="disable"/>` allows joints to exceed their
   range without constraint forces. A hinge with `range="0 90"` rotates past
   90° freely.

4. **Equality disable**: `<flag equality="disable"/>` deactivates all equality
   constraints (weld, joint, tendon, flex). Bodies linked by weld constraints
   separate freely.

5. **Spring/damper independence**: `<flag spring="disable" damper="enable"/>`
   disables spring stiffness but keeps damping. A joint with `stiffness=100
   damping=10` produces zero spring force but non-zero damping force.

6. **Actuation disable**: `<flag actuation="disable"/>` zeros all actuator
   forces. `ctrl` values have no effect.

7. **Sensor disable**: `<flag sensor="disable"/>` skips sensor evaluation.
   `sensordata` array is not updated.

8. **Warmstart disable**: `<flag warmstart="disable"/>` starts solver from
   zero (not previous solution). May increase solver iterations.

9. **Constraint disable**: `<flag constraint="disable"/>` skips the entire
   solver. No equality, limit, contact, or friction loss constraints are
   applied. Only passive and actuator forces affect the dynamics.

10. **Fluid force disable interaction** (from §40): When both `DISABLE_SPRING`
    and `DISABLE_DAMPER` are set, `mj_fluid()` is skipped — fluid forces
    require at least one of spring/damper to be enabled (MuJoCo semantics).
    Gate location: `mujoco_pipeline.rs` around `mj_fluid()` call in
    `mj_fwd_passive()`.

11. **Default values**: An unmodified `<option/>` with no `<flag>` produces
    `disableflags == 0` (all enable) and `enableflags == 0` (all optional
    features off). Matches MuJoCo defaults.

12. **Backward compat**: `<flag passive="disable"/>` sets both `DISABLE_SPRING`
    and `DISABLE_DAMPER` (legacy behavior).

13. **Fluid sleep filtering** (from §40): Replace `for body_id in 0..model.nbody`
    in `mj_fluid()` with sleep-filtered body iteration. MuJoCo gates the body
    loop on sleep state. No correctness impact (sleeping bodies have zero
    velocity), but skipping them is a performance win and matches MuJoCo
    semantics. Location: `mujoco_pipeline.rs` `mj_fluid()`.

14. **MuJoCo conformance**: For each of the 19 disable flags, compare 10-step
    trajectory against MuJoCo 3.4.0 with only that flag disabled. Tolerance:
    `1e-10` per `qacc` component (flags should produce bit-exact gating).

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — all 19+6 flag constants, runtime checks
  at each pipeline stage (see S4 table for locations)
- `sim/L0/mjcf/src/types.rs` — split `MjcfFlag.passive` into `spring` + `damper`,
  add `fwdinv`, `invdiscrete`, `autoreset` fields
- `sim/L0/mjcf/src/parser.rs` — parse `spring`, `damper` (+ backward compat
  `passive`), `fwdinv`, `invdiscrete`, `autoreset`
- `sim/L0/mjcf/src/model_builder.rs` — `flags_to_bits()` conversion, wire all
  flags to `Model.disableflags` / `Model.enableflags`
- `sim/L0/tests/integration/` — per-flag disable/enable tests

---

### 42. `<flex>` / `<flexcomp>` MJCF Deformable Body Parsing
**Status:** Subsumed by §6B | **Effort:** L | **Prerequisites:** None

> **Note:** This item has been subsumed by [§6B Flex Solver Unification](./future_work_6b_precursor_to_7.md),
> which implements full `<flex>`/`<flexcomp>` parsing along with the unified flex
> constraint pipeline. See `future_work_6b_precursor_to_7.md` §P9 for the
> implemented parsing specification.

#### Current State
The MJCF parser skips `<flex>` and `<flexcomp>` elements inside `<deformable>` — only
`<skin>` is parsed. MuJoCo's `<flex>` is the native MJCF way to define deformable
bodies (soft bodies, cloth, ropes), and `<flexcomp>` is its procedural counterpart
(similar to `<composite>` but for flex bodies).

CortenForge has a functional deformable pipeline (`XpbdSolver`, `DeformableBody`,
`mj_deformable_collision()`, `mj_deformable_step()`) wired into the main pipeline,
but it's only accessible via the programmatic API (`register_deformable()`). Models
that define deformable bodies through MJCF `<flex>` elements silently lose them.

#### Objective
Parse `<flex>` and `<flexcomp>` MJCF elements and wire them into the existing
deformable pipeline.

#### Specification

1. **`<flex>` parsing**: Parse `<flex>` elements from `<deformable>`:
   - `name`, `dim` (1=cable, 2=shell, 3=solid)
   - `<vertex>` — vertex positions
   - `<element>` — element connectivity (edges/triangles/tetrahedra)
   - `<body>` — which body each vertex belongs to
   - Material properties: `young`, `poisson`, `damping`, `thickness`
   - Collision: `selfcollide`, `radius`, `margin`
2. **`<flexcomp>` parsing**: Parse procedural flex definitions:
   - `type` (grid, box, cylinder, etc.)
   - `count`, `spacing` — procedural dimensions
   - Expand to equivalent `<flex>` before model building
3. **Wiring**: Convert parsed flex data to `DeformableBody` instances and register
   them with the pipeline's `deformable_solvers` during model construction.
4. **`<equality><flex>`**: Parse flex equality constraints (flex-flex coupling).

#### Acceptance Criteria
1. A `<flex dim="2">` cloth element loads and simulates correctly.
2. `<flexcomp type="grid">` generates the expected deformable mesh.
3. Programmatic `register_deformable()` path unchanged (regression).

#### Files
- `sim/L0/mjcf/src/parser.rs` — parse `<flex>`, `<flexcomp>` elements
- `sim/L0/mjcf/src/model_builder.rs` — convert to `DeformableBody`, register with pipeline

---

### 42A-i. Sparse Flex Edge Jacobian (`flexedge_J`)
**Status:** Not started | **Effort:** L | **Prerequisites:** §6B ✅, #27D

> **Discovery context:** Found during #27B/#27C spec review (measure-twice pass)
> while cross-checking our edge force application against MuJoCo `engine_passive.c`.

#### Current State

MuJoCo uses a pre-computed sparse edge Jacobian `flexedge_J` for applying
edge forces via `J^T * force`. This handles the general case where flex vertices
may be attached to complex multi-DOF bodies (e.g., a flex vertex attached to a
free-floating body has 6 DOFs, not 3 translational DOFs).

**Our code** computes the Jacobian inline as `±direction` applied to
`flexvert_dofadr`, which assumes each vertex maps to 3 consecutive translational
DOFs. This pattern is used in three places:

1. **Edge passive forces** (`mj_fwd_passive()` edge spring-damper loop)
2. **Bending passive forces** (`mj_fwd_passive()` bending force loop, line ~11642)
3. **Newton penalty path** (line ~14827)

For free vertices (no body attachment), the `±direction` inline Jacobian is
correct — free flex vertices always have exactly 3 translational DOFs.

**For body-attached vertices** (when #27D is implemented), this is WRONG:
- A vertex attached to a body with a free joint has 6 DOFs (3 trans + 3 rot)
- A vertex attached to a body with a hinge joint has 1 DOF
- The force must be projected through the body's full Jacobian at the vertex
  position, not just applied as `±direction` to 3 DOFs

#### Objective

Add `flexedge_J` (sparse edge Jacobian) as a pre-computed Data field, and use
it for edge/bending/penalty force application instead of inline `±direction`.

#### Specification

1. **Model field:** `flex_edge_J_rownnz`, `flex_edge_J_rowadr`, `flex_edge_J_colind`,
   `flex_edge_J_data` — compressed sparse row (CSR) format per MuJoCo convention.
   Each edge has 2 rows (one per endpoint vertex), each row spans the vertex's DOFs.

2. **Pre-computation:** After forward kinematics (`mj_kinematics()`), compute the
   edge Jacobian. For each edge endpoint:
   - If vertex is free: Jacobian row is `±direction` on the 3 translational DOFs
   - If vertex is body-attached: Jacobian row is `mj_jac()` at the vertex position
     for the attached body, multiplied by `±1`

3. **Force application:** Replace all three inline `±direction` patterns with
   `J^T * force` using the sparse Jacobian. This is a single code path that
   handles both free and body-attached vertices correctly.

4. **`flexedge_length` / `flexedge_velocity`:** MuJoCo also pre-computes these
   as Data fields. We currently compute them inline. Optionally pre-compute them
   alongside the Jacobian for efficiency and MuJoCo Data field parity.

#### Acceptance Criteria

1. `flexedge_J` Data field exists and is computed after forward kinematics.
2. Edge spring-damper, bending, and Newton penalty forces all use `J^T * force`
   via the sparse Jacobian.
3. For models with only free vertices: bit-identical results to current inline code
   (the sparse Jacobian degenerates to `±direction` on 3 DOFs for free vertices).
4. For models with body-attached vertices (#27D): correct force projection
   through body Jacobians. Verify via finite-difference: perturb `qpos` by `1e-7`,
   recompute edge forces, compare against `J^T * analytical_force`. Tolerance: `1e-5`.
5. All existing flex tests pass (regression).
6. **Performance**: Pre-computing the sparse Jacobian once and reading it 3 times
   (edge, bending, penalty) is faster than computing it inline 3 times. Verify
   no throughput regression for the common case (free vertices only).

**Note:** For models with only free vertices (all current test cases), the inline
`±direction` and the sparse Jacobian produce identical results. The sparse Jacobian
becomes essential only when body-attached vertices (#27D) exist — the inline pattern
would produce wrong forces for multi-DOF bodies. This item is forward-looking but
required for full MuJoCo Data layout parity.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — `Model` (sparse Jacobian fields),
  `Data` (pre-computed Jacobian + optional length/velocity), edge/bending/penalty
  force application paths

---

### 42A-ii. `flex_rigid` / `flexedge_rigid` Boolean Arrays
**Status:** Not started | **Effort:** S | **Prerequisites:** §6B ✅

> **Discovery context:** Found during #27C spec review while cross-checking
> `engine_passive.c` loop structure.

#### Current State

MuJoCo pre-computes two boolean arrays:
- `flex_rigid[f]` — `true` if ALL vertices of flex `f` are body-attached with
  `invmass == 0` (the entire flex is rigid). Entire flex loops are skipped.
- `flexedge_rigid[e]` — `true` if BOTH endpoints of edge `e` have `invmass == 0`.
  Individual edge iterations are skipped.

**Our code** checks `flexvert_invmass[v] == 0.0` per-vertex inside the inner
loop. This is semantically equivalent but less efficient: we do the check per
edge (2 vertex lookups) instead of one boolean check per flex or per edge.

#### Objective

Add `flex_rigid` and `flexedge_rigid` pre-computed boolean arrays for efficiency.

#### Specification

1. **Model fields:**
   ```rust
   /// Per-flex: true if all vertices have invmass == 0.
   pub flex_rigid: Vec<bool>,
   /// Per-edge: true if both endpoint vertices have invmass == 0.
   pub flexedge_rigid: Vec<bool>,
   ```

2. **Computation:** During model building, after `flexvert_invmass` is populated:
   - `flex_rigid[f]` = all `flexvert_invmass[v] == 0.0` for vertices in flex `f`
   - `flexedge_rigid[e]` = `flexvert_invmass[v0] == 0.0 && flexvert_invmass[v1] == 0.0`

3. **Usage:** Replace per-vertex invmass checks in all flex loops:
   - Outer flex loop: `if flex_rigid[f] { continue; }`
   - Inner edge loop: `if flexedge_rigid[e] { continue; }`

#### Acceptance Criteria

1. `flex_rigid` and `flexedge_rigid` fields exist on Model.
2. Rigid flex bodies and edges are skipped without per-vertex checks.
3. Identical simulation results (optimization only, no behavior change).
4. All existing flex tests pass.

**Downstream dependency:** `flex_rigid` is required by §42A-iv (flex
self-collision dispatch) as the first gate condition: `!flex_rigid[f]`.
See §30 AC7 documentation on `flex_selfcollide` in `mujoco_pipeline.rs`.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — Model fields, flex loop optimizations
- `sim/L0/mjcf/src/model_builder.rs` — pre-compute rigid flags

---

### 42A-iii. `flexedge_length` / `flexedge_velocity` Pre-computed Data Fields
**Status:** Not started | **Effort:** S | **Prerequisites:** §6B ✅

> **Discovery context:** Found during #27C spec review while cross-checking
> `engine_passive.c` field access patterns.

#### Current State

MuJoCo pre-computes `d->flexedge_length[e]` and `d->flexedge_velocity[e]` as
Data fields, computed during forward kinematics. Multiple consumers read these
pre-computed values: passive forces, constraint assembly, Newton penalty.

**Our code** computes both inline from `flexvert_xpos` and `qvel` at each usage
site. This duplicates computation when multiple consumers need the same values
(e.g., edge spring force + Newton penalty both need edge length).

#### Objective

Add `flexedge_length` and `flexedge_velocity` as pre-computed Data fields for
efficiency and MuJoCo Data layout parity.

#### Specification

1. **Data fields:**
   ```rust
   /// Pre-computed edge lengths (Euclidean distance between endpoints).
   pub flexedge_length: Vec<f64>,
   /// Pre-computed edge elongation velocities (rate of length change).
   pub flexedge_velocity: Vec<f64>,
   ```

2. **Computation:** After forward kinematics (when `flexvert_xpos` and `qvel`
   are up to date):
   ```
   direction = flexvert_xpos[v1] - flexvert_xpos[v0]
   length = ||direction||
   unit_dir = direction / length
   velocity = (qvel[dof1..dof1+3] - qvel[dof0..dof0+3]) · unit_dir
   ```

3. **Consumers:** Replace inline computation in:
   - Edge spring-damper passive forces (`mj_fwd_passive()`)
   - Newton penalty edge path
   - Any future edge-based computations

#### Acceptance Criteria

1. `flexedge_length` and `flexedge_velocity` exist as Data fields.
2. Computed once after forward kinematics, read by all consumers.
3. Identical simulation results (optimization only, no behavior change).
4. All existing flex tests pass.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — Data fields, computation in
  kinematics phase, consumer updates

---

### 42A-iv. Flex Self-Collision Dispatch (BVH/SAP Midphase + Narrowphase)
**Status:** Not started | **Effort:** L | **Prerequisites:** §42A-ii (`flex_rigid`), §30 ✅

> **Discovery context:** §30 implemented flex-rigid contype/conaffinity
> filtering and documented the MuJoCo self-collision gating protocol. The
> `flex_selfcollide` flag is parsed and stored but self-collision is not
> dispatched. This item adds the full dispatch path.

#### Current State

`flex_selfcollide: Vec<bool>` exists on Model (parsed from
`<flex><contact selfcollide="..."/>`), but no code reads it during
collision detection. `mj_collision_flex()` only handles flex-rigid pairs.
MuJoCo dispatches flex self-collision from `mj_collision()` in
`engine_collision_driver.c`.

#### MuJoCo Reference

MuJoCo's self-collision dispatch has two independent paths, both gated
behind the same three conditions:

```c
for (int f = 0; f < m->nflex; f++) {
    if (!m->flex_rigid[f] && (m->flex_contype[f] & m->flex_conaffinity[f])) {
        // Path 1: internal collisions (adjacent elements sharing vertices/edges)
        if (m->flex_internal[f]) {
            mj_collideFlexInternal(m, d, f);
        }
        // Path 2: self-collisions (non-adjacent elements)
        if (m->flex_selfcollide[f] != mjFLEXSELF_NONE) {
            switch (m->flex_selfcollide[f]) {
                case mjFLEXSELF_NARROW: mj_collideFlexSelf(m, d, f); break;
                case mjFLEXSELF_BVH:    /* BVH midphase + narrowphase */ break;
                case mjFLEXSELF_SAP:    /* SAP midphase + narrowphase */ break;
                case mjFLEXSELF_AUTO:   /* BVH for dim=3, SAP otherwise */ break;
            }
        }
    }
}
```

Three conjunctive gate conditions:
1. `!flex_rigid[f]` — rigid flexes skip entirely (requires §42A-ii)
2. `(flex_contype[f] & flex_conaffinity[f]) != 0` — self-bitmask check
   (§30 added `flex_contype`/`flex_conaffinity` fields)
3. `flex_selfcollide[f]` / `flex_internal[f]` — per-path enable flags

`mjtFlexSelf` enum values: `NONE=0, NARROW=1, BVH=2, SAP=3, AUTO=4`.

#### Implementation

1. **Add `flex_internal: Vec<bool>` to Model** — parsed from
   `<flex><contact internal="true"/>`, default `false`.

2. **Add self-collision gate in collision pipeline** — after
   `mj_collision_flex()` (flex-rigid), add flex self-collision dispatch:
   ```rust
   for f in 0..model.nflex {
       if model.flex_rigid[f] { continue; }
       if (model.flex_contype[f] & model.flex_conaffinity[f]) == 0 { continue; }
       if model.flex_internal[f] {
           mj_collide_flex_internal(model, data, f);
       }
       if model.flex_selfcollide[f] {
           mj_collide_flex_self(model, data, f);
       }
   }
   ```

3. **Implement `mj_collide_flex_internal()`** — element-element narrowphase
   for adjacent elements (sharing vertices/edges). Generates contacts
   between element faces/edges.

4. **Implement `mj_collide_flex_self()`** — non-adjacent element collision.
   Start with brute-force narrowphase (`NARROW` mode), then add BVH/SAP
   midphase for performance:
   - **BVH**: Per-element AABB tree, updated per-step from `flexvert_xpos`.
   - **SAP**: Sweep-and-prune on element AABBs along the axis of maximum
     variance.
   - **AUTO**: BVH for `dim=3` (solids), SAP otherwise.

5. **Upgrade `flex_selfcollide` to enum** — change from `Vec<bool>` to
   `Vec<FlexSelfCollide>` with variants matching `mjtFlexSelf`. The
   existing `bool` is a placeholder; the enum enables algorithm selection.

#### Acceptance Criteria

1. §30 AC6: Self-collision disabled when `(flex_contype & flex_conaffinity) == 0`,
   even if `selfcollide` flag is set.
2. §30 AC7: Rigid flexes (`flex_rigid=true`) skip all self/internal collision.
3. `internal` and `selfcollide` dispatch independently behind shared gate.
4. `selfcollide="none"` produces zero self-contacts.
5. `selfcollide="narrow"` produces correct element-element contacts.
6. BVH/SAP midphase produces identical contacts to brute-force (correctness).
7. Existing flex-rigid tests unaffected.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — dispatch loop, `mj_collide_flex_internal()`,
  `mj_collide_flex_self()`, `FlexSelfCollide` enum, `flex_internal` field
- `sim/L0/mjcf/src/types.rs` — `internal` field on `MjcfFlex`
- `sim/L0/mjcf/src/parser.rs` — parse `internal` attribute
- `sim/L0/mjcf/src/model_builder.rs` — wire `flex_internal`, upgrade
  `flex_selfcollide` push
- `sim/L0/tests/integration/flex_unified.rs` — self-collision tests

---

### 42A-v. Flex-Flex Cross-Object Collision Filtering
**Status:** Not started | **Effort:** M | **Prerequisites:** §42A-iv, §30 ✅

> **Discovery context:** §30 spec review identified that MuJoCo filters
> flex-flex pairs at the broadphase level via `canCollide2()`, but our
> codebase has no flex-flex collision path at all.

#### Current State

`mj_collision_flex()` only handles flex-rigid pairs (flex vertex vs rigid
geom). There is no flex-flex collision path — two flex objects cannot
collide with each other.

#### MuJoCo Reference

MuJoCo uses a unified bodyflex index space where bodies occupy
`[0, nbody)` and flexes occupy `[nbody, nbody+nflex)`. The broadphase
generates bodyflex pairs, and `canCollide2()` filters them:

```c
static int canCollide2(const mjModel* m, int bf1, int bf2) {
    int nbody = m->nbody;
    int contype1 = (bf1 < nbody) ? m->body_contype[bf1] : m->flex_contype[bf1-nbody];
    int conaffinity1 = (bf1 < nbody) ? m->body_conaffinity[bf1] : m->flex_conaffinity[bf1-nbody];
    int contype2 = (bf2 < nbody) ? m->body_contype[bf2] : m->flex_contype[bf2-nbody];
    int conaffinity2 = (bf2 < nbody) ? m->body_conaffinity[bf2] : m->flex_conaffinity[bf2-nbody];
    return (!filterBitmask(contype1, conaffinity1, contype2, conaffinity2));
}
```

Flex-flex narrowphase uses element-element collision tests (triangle-triangle
for dim=2, tetrahedron-tetrahedron for dim=3), with BVH acceleration per
flex object.

#### Implementation

1. **Add flex-flex broadphase filtering** — for each flex pair `(f1, f2)`,
   apply `filterBitmask(flex_contype[f1], flex_conaffinity[f1],
   flex_contype[f2], flex_conaffinity[f2])`.

2. **Implement flex-flex narrowphase** — element-element collision between
   two different flex objects. Reuse narrowphase primitives from §42A-iv.

3. **Integrate into collision pipeline** — after flex-rigid and flex-self
   collision, add flex-flex collision:
   ```rust
   for f1 in 0..model.nflex {
       for f2 in (f1+1)..model.nflex {
           let ct1 = model.flex_contype[f1];
           let ca1 = model.flex_conaffinity[f1];
           let ct2 = model.flex_contype[f2];
           let ca2 = model.flex_conaffinity[f2];
           if (ct1 & ca2) == 0 && (ct2 & ca1) == 0 { continue; }
           mj_collide_flex_flex(model, data, f1, f2);
       }
   }
   ```

4. **Contact parameter combination** — add `contact_param_flex_flex()`
   following the same priority/solmix protocol as `contact_param_flex_rigid()`.

#### Acceptance Criteria

1. Flex-flex pairs filtered by `filterBitmask()` protocol (same as flex-rigid).
2. Compatible flex objects generate contacts at element-element intersections.
3. Incompatible bitmasks produce zero flex-flex contacts.
4. Contact parameters combined via priority + solmix (matching flex-rigid).
5. Existing flex-rigid and flex-self tests unaffected.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — flex-flex broadphase loop,
  `mj_collide_flex_flex()`, `contact_param_flex_flex()`
- `sim/L0/tests/integration/flex_unified.rs` — flex-flex collision tests

---

### 42B. Flex Bending Discretization: Cotangent Laplacian + Trait Abstraction
**Status:** Not started | **Effort:** L | **Prerequisites:** §6B ✅

#### Background

§6B implemented flex bending as Bridson et al. 2003 dihedral angle springs in
`mj_fwd_passive()`. MuJoCo uses Wardetzky et al. "Discrete Quadratic Curvature
Energies" (cotangent Laplacian) with Garg et al. "Cubic Shells" curved
reference correction — a precomputed 4×4 stiffness matrix per edge.

These are fundamentally different:

| | **Bridson (ours)** | **Wardetzky/Garg (MuJoCo)** |
|-|---|---|
| **Force model** | `F = -k * (θ - θ₀) * grad_i` | `F_i = Σ_j B[i,j] * x_j + B[16] * frc_i` |
| **Gradient norms** | Geometry-dependent, change each step | Constant (precomputed cotangent weights) |
| **Large deformation** | Correct — uses actual dihedral angle | Linearized — accurate only for small deflections |
| **Stability** | Requires per-vertex force clamp | Unconditionally stable (constant matrix) |
| **Runtime cost** | Per-hinge: atan2 + cross products + 4 gradient vectors | Per-edge: 4×4 matrix-vector (16 muls) + cross products |
| **Precomputation** | None (rest angle stored) | 17 coefficients per edge |
| **Conformance** | Diverges from MuJoCo | Exact match |

Neither is strictly superior — Bridson captures nonlinear bending faithfully
(cloth draping, sheet metal forming), while the cotangent Laplacian is efficient,
stable, and matches MuJoCo exactly. We should support both via a trait.

#### Current State

Bending force computation in `mj_fwd_passive()` (`mujoco_pipeline.rs:11370+`):
- Bridson dihedral angle springs with per-vertex force clamping
- `Model.flex_bend_stiffness: Vec<f64>` (scalar per flex body)
- `Model.flex_bend_damping: Vec<f64>` (scalar per flex body)
- `Model.flexhinge_vert: Vec<[usize; 4]>` — hinge topology `[e0, e1, a, b]`
- `Model.flexhinge_angle0: Vec<f64>` — rest dihedral angles
- Force clamp: `fm_max = 1/(dt² * |grad| * invmass)` per vertex per hinge

MuJoCo (`engine_passive.c:206–268`):
- `flex_bending` array: 17 `f64` per edge (4×4 stiffness matrix + 1 curved ref coeff)
- `b[17*e + 4*i + j]` = stiffness coupling between vertex `i` and vertex `j`
- `b[17*e + 16]` = curved reference contribution coefficient
- Force: `spring[3*i+x] += b[4*i+j] * xpos[3*v[j]+x] + b[16] * frc[i][x]`
- Damper: `damper[3*i+x] += b[4*i+j] * vel[j][x]` (same matrix)
- No stability clamp — constant matrix guarantees linear force growth

Precomputation (`user_mesh.cc:3740–3784`, `ComputeBending`):
```
mu = young / (2 * (1 + poisson))
stiffness = 3 * mu * thickness³ / (24 * volume)
```
Uses Wardetzky cotangent operator: `c[i]` weights from cotangent of dihedral
angles at rest configuration. The 4×4 outer product `c[i] * c[j] * cos_theta * stiffness`
gives the stiffness matrix entries. The `b[16]` curved reference term handles
non-flat rest configurations.

#### Objective

1. Implement the Wardetzky/Garg cotangent Laplacian bending model
2. Abstract bending computation behind a `FlexBendingModel` trait
3. Default to cotangent (MuJoCo conformance), allow Bridson via configuration
4. Precompute `flex_bending` coefficients at model compile time

#### Specification

##### S1. `FlexBendingModel` trait

```rust
/// Bending force model for 2D flex bodies (dim=2).
pub trait FlexBendingModel {
    /// One-time precomputation from rest geometry and material properties.
    /// Called during model compilation.
    fn precompute(
        &mut self,
        model: &Model,
        flex_id: usize,
    );

    /// Accumulate bending forces into qfrc_passive.
    /// Called each step from mj_fwd_passive().
    fn apply_forces(
        &self,
        model: &Model,
        data: &mut Data,
        flex_id: usize,
    );
}
```

Two implementations:
- `CotangentBending` — precomputes 17-coefficient matrix per edge, applies via
  matrix-vector multiply. No stability clamp needed.
- `BridsonBending` — current implementation (dihedral angle springs + per-vertex
  force clamp). No precomputation beyond rest angles.

##### S2. Model storage

Add to `Model`:
```rust
/// Bending stiffness matrix (17 f64 per edge, Wardetzky cotangent Laplacian).
/// Layout: [4×4 vertex coupling matrix, 1 curved reference coeff] per edge.
/// Only populated when using cotangent bending model.
pub flex_bending: Vec<f64>,

/// Which bending model each flex body uses.
pub flex_bending_model: Vec<FlexBendingType>,
```

```rust
pub enum FlexBendingType {
    /// Wardetzky/Garg cotangent Laplacian (MuJoCo-conformant, default).
    Cotangent,
    /// Bridson dihedral angle springs (nonlinear, large-deformation accurate).
    Bridson,
}
```

##### S3. Precomputation (cotangent model)

During model compilation in `model_builder.rs`, for each flex body with
`bending_model == Cotangent` and `dim == 2`:

**Prerequisites**: `flexedge_flap` (S7) must be computed first.

For each edge `e` with two adjacent triangles (interior edge):

Given the diamond stencil vertices `v = [v0, v1, va, vb]` where `(v0, v1)` is
the shared edge, `va` is opposite in triangle 1, `vb` is opposite in triangle 2:

```
Step 1: Geometry
  e0 = x[v1] - x[v0]           // edge vector
  e1 = x[va] - x[v0]           // flap vector (triangle 1)
  e2 = x[vb] - x[v0]           // flap vector (triangle 2)
  n1 = e0 × e1                 // triangle 1 normal (unnormalized)
  n2 = e0 × e2                 // triangle 2 normal (unnormalized)
  area1 = |n1| / 2             // triangle 1 area
  area2 = |n2| / 2             // triangle 2 area
  total_area = area1 + area2   // diamond area

Step 2: Cotangent weights (Wardetzky operator)
  For each of the 4 vertices, compute the cotangent of the angle at that
  vertex in its containing triangle(s). MuJoCo uses a specific weight formula
  that encodes the cotangent Laplacian as a 4-element vector c[0..4].
  (See user_mesh.cc:ComputeBending for the exact cotangent computation.)

Step 3: Material stiffness
  mu = young / (2 * (1 + poisson))                    // shear modulus
  stiffness = 3 * mu * thickness³ / (24 * total_area)  // plate bending stiffness

Step 4: Stiffness matrix (4×4)
  For i in 0..4, j in 0..4:
    bending[17*e + 4*i + j] = c[i] * c[j] * stiffness

Step 5: Curved reference coefficient
  cos_theta = -dot(n1_hat, n2_hat)  // cosine of dihedral angle at rest
  If rest shape is flat (cos_theta ≈ -1): bending[17*e + 16] = 0
  If rest shape is curved: bending[17*e + 16] = function of rest curvature
  (Garg et al. "Cubic Shells" correction for non-zero rest curvature)
```

For boundary edges (only one adjacent triangle): zero all 17 coefficients.

**Note:** The exact cotangent weight formulas and curved reference computation
must be verified against MuJoCo's `user_mesh.cc:ComputeBending` at implementation
time. The precomputation is compile-time-only, so matching MuJoCo exactly here is
critical for conformance — any mismatch propagates to every bending force.

##### S4. Runtime force application (cotangent model)

In `mj_fwd_passive()`, replace the current per-hinge loop with a per-edge loop
for cotangent bending:

```rust
for e in 0..model.nflexedge {
    let flex_id = model.flexedge_flexid[e];
    if model.flex_bending_model[flex_id] != FlexBendingType::Cotangent {
        continue;
    }
    let b = &model.flex_bending[17 * e..17 * (e + 1)];
    let [v0, v1] = model.flexedge_vert[e];
    let [va, vb] = model.flexedge_flap[e]; // adjacent vertices
    if vb == usize::MAX { continue; } // boundary edge

    let v = [v0, v1, va, vb];
    // spring: F_i += Σ_j b[4*i+j] * x_j + b[16] * curved_ref_i
    // damper: F_i += damping * Σ_j b[4*i+j] * vel_j
    // (See MuJoCo engine_passive.c:240–266 for exact loop)
}
```

No stability clamp — the constant matrix produces forces linear in positions.

##### S5. Bridson model (preserve current implementation)

Move the current dihedral angle loop into `BridsonBending::apply_forces()`.
No changes to the algorithm — keep the per-vertex force clamp.

##### S6. MJCF configuration

Option A: Custom CortenForge extension attribute on `<flex>`:
```xml
<flex name="cloth" dim="2" young="1e4" bending_model="bridson"/>
```

Default: `cotangent` (MuJoCo conformance).

Option B: Global option:
```xml
<option cortenforge:bending_model="bridson"/>
```

Prefer Option A — per-flex-body granularity is more useful.

##### S7. Edge topology: `flexedge_flap`

MuJoCo stores `flex_edgeflap` — for each edge, the two "flap" vertices (one
from each adjacent triangle, opposite the shared edge). We need the same:

```rust
/// For each edge: the two vertices opposite the edge in adjacent triangles.
/// [va, vb] where va is in triangle 1, vb is in triangle 2.
/// Boundary edges: vb = usize::MAX.
pub flexedge_flap: Vec<[usize; 2]>,
```

This is computed during model compilation from element connectivity.

#### Acceptance Criteria

1. **Cotangent conformance**: Bending forces match MuJoCo 3.4.0 for a 4×4 grid
   shell with `young=1e4, poisson=0.3, thickness=0.01, density=1000`. Compare
   `qfrc_spring` (MuJoCo) vs `qfrc_passive` (ours) after 1 step with gravity.
   Tolerance: `1e-8` per component (the precomputation involves trig functions
   and cotangent weights that accumulate ~8 digits of precision; runtime
   matrix-vector multiply adds minimal further error).
2. **Bridson equivalence**: Switching to `bending_model="bridson"` produces the
   same forces as the current implementation (regression test).
3. **Stability without clamp**: Cotangent model with `young=1e12, dt=0.01`
   (the AC20 test scenario) remains stable for 500 steps without any force
   clamping. The stability comes from the constant matrix, not from clamps.
4. **Curved reference**: A non-flat rest mesh (e.g., a curved shell) produces
   correct bending forces via the `b[16]` curved reference term.
5. **Boundary edges**: Boundary edges (only one adjacent triangle) produce zero
   bending force.
6. **Trait dispatch**: Swapping `FlexBendingType` on a model changes force
   computation without touching any other pipeline code.
7. **Mixed-model dispatch**: A model with two flex bodies — one cotangent, one
   Bridson — dispatches correctly per flex body and produces correct forces for
   both simultaneously.
8. **Zero trait overhead**: `mj_fwd_passive()` throughput with cotangent bending
   through the trait is within 1% of a hardcoded implementation (no performance
   regression from the trait boundary). This validates the pattern for Phases B–E
   of the [trait architecture](../TRAIT_ARCHITECTURE.md).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `FlexBendingModel` trait,
  `CotangentBending`, `BridsonBending`, `mj_fwd_passive()` dispatch
- `sim/L0/mjcf/src/model_builder.rs` — `flex_bending` precomputation,
  `flexedge_flap` topology, `bending_model` attribute parsing
- `sim/L0/mjcf/src/parser.rs` — parse `bending_model` attribute from `<flex>`
- `sim/L0/tests/integration/flex_unified.rs` — conformance tests, stability
  tests, regression tests for Bridson path

---

### 42C. `FlexElasticityModel` Trait — Phase B (Membrane / Edge Elasticity)
**Status:** Not started | **Effort:** L | **Prerequisites:** §42B

#### Background

The flex edge constraint currently uses a soft equality constraint with solref/solimp
(matching MuJoCo's `mjEQ_FLEX` mechanism). This is `LinearElastic` — small-strain
response expressed as constraint rows in the unified Jacobian.

Nonlinear hyperelastic materials (Neo-Hookean, Mooney-Rivlin, Ogden) require
strain-energy-based forces computed from deformation gradients. These cannot be
expressed as constraint rows — they produce direct passive forces from the strain
energy density function.

The divergence:

| Regime | Model | Mechanism | Use Case |
|--------|-------|-----------|----------|
| Small strain | Linear elastic (current) | Constraint row (`FlexEdge`) | Robotics, stiff bodies, MuJoCo conformance |
| Large strain | Neo-Hookean / SVK | Passive force from strain energy | Soft tissue, surgical sim, rubber |
| Hyperelastic | Mooney-Rivlin, Ogden | Passive force from strain energy invariants | Biomechanics, material characterization |

#### Specification

##### S1. `FlexElasticityModel` trait

```rust
pub trait FlexElasticityModel {
    /// Per-flex precomputed data (opaque to the pipeline).
    type Precomputed;

    /// Precompute coefficients at model build time.
    fn precompute(
        flex_id: usize,
        model: &Model,
    ) -> Self::Precomputed;

    /// Apply elasticity forces/constraints for this flex body.
    /// Constraint-based models add rows to the Jacobian.
    /// Force-based models accumulate into qfrc_passive.
    fn apply(
        flex_id: usize,
        pre: &Self::Precomputed,
        model: &Model,
        data: &mut Data,
    );
}
```

##### S2. `LinearElastic` (extract current implementation)

Move the existing `FlexEdge` constraint row assembly behind the trait:
- `precompute()`: Store rest edge lengths (already computed at build time).
- `apply()`: Assemble edge equality constraint rows into the Jacobian, exactly
  as the current code does. No behavioral change.

##### S3. `NeoHookean` (new implementation)

Strain-energy-based passive force computation:
- **Energy**: `W = μ/2 (I₁ - 3) - μ ln(J) + λ/2 (ln J)²`
  where `I₁ = tr(FᵀF)`, `J = det(F)`, `F` is the deformation gradient.
- **Force**: `f = -∂W/∂x` computed per element from the deformation gradient.
- `precompute()`: Compute rest-state inverse reference matrices (`Dm⁻¹`) per element,
  rest volumes, and material parameters (Lamé: `μ = E / (2(1+ν))`,
  `λ = Eν / ((1+ν)(1-2ν))`).
- `apply()`: For each element, compute deformation gradient `F = Ds * Dm⁻¹`,
  compute first Piola-Kirchhoff stress `P = ∂W/∂F`, accumulate nodal forces
  `f = -V₀ * P * Dm⁻ᵀ` into `qfrc_passive`.

For dim=2 (shells): Use the in-plane deformation gradient (2×2 → 3×2 map)
with thickness integrated out. For dim=3 (solids): Full 3×3 deformation gradient.

##### S4. Per-flex dispatch

```rust
pub enum FlexElasticityType {
    /// Linear elastic constraint rows (MuJoCo-conformant, default).
    Linear,
    /// Neo-Hookean hyperelastic passive forces.
    NeoHookean,
}
```

Stored per flex body in `Model.flex_elasticity_model: Vec<FlexElasticityType>`.

##### S5. MJCF configuration

```xml
<flex name="tissue" dim="3" young="5e3" poisson="0.45"
      elasticity_model="neo_hookean"/>
```

Default: `linear` (MuJoCo conformance).

#### Acceptance Criteria

1. `LinearElastic` through the trait produces identical constraint rows as the
   current direct implementation (bit-exact regression).
2. `NeoHookean` on a dim=3 cube under gravity produces physically plausible
   large-deformation response (cube deforms, doesn't explode, conserves volume
   approximately for ν → 0.5).
3. `NeoHookean` on a dim=2 shell under gravity produces membrane stretch forces.
4. Mixed model: one flex body linear, one neo-Hookean, both simulate correctly.
5. Neo-Hookean with small strain converges to linear elastic response
   (validation: compare forces at 1% strain, should agree within 5%).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `FlexElasticityModel` trait,
  `LinearElastic`, `NeoHookean`, dispatch in constraint assembly + passive forces
- `sim/L0/mjcf/src/model_builder.rs` — `elasticity_model` attribute, precomputation
- `sim/L0/mjcf/src/parser.rs` — parse `elasticity_model` from `<flex>`
- `sim/L0/tests/integration/flex_unified.rs` — hyperelastic tests, regression tests

---

### 42D. `ActuatorGainModel` Trait — Phase C (Actuator Gain Models)
**Status:** Not started | **Effort:** M | **Prerequisites:** §42B

#### Background

MuJoCo has three gain types dispatched by enum: `Fixed`, `Affine`, `Muscle`. The set
is closed — adding a new gain model requires modifying the enum. Real actuator modeling
needs an open set: series elastic actuators (SEA compliance), pneumatic actuators
(nonlinear pressure-volume), hydraulic actuators (valve dynamics), cable-driven actuators
(cable elasticity + friction).

The existing enum dispatch is correct for the MuJoCo-compatible types and should remain.
The trait extends it for user-defined models that can't be expressed as
`force = gainprm[0] + gainprm[1]*length + gainprm[2]*velocity`.

#### Specification

##### S1. `ActuatorGainModel` trait

```rust
pub trait ActuatorGainModel {
    type Params;

    /// Compute actuator force contribution.
    fn gain(
        length: f64,
        velocity: f64,
        activation: f64,
        ctrl: f64,
        params: &Self::Params,
    ) -> f64;

    /// Compute ∂force/∂velocity for implicit integration.
    fn dgain_dvel(
        length: f64,
        velocity: f64,
        activation: f64,
        ctrl: f64,
        params: &Self::Params,
    ) -> f64;
}
```

##### S2. Built-in implementations

Extract the existing gain computation into trait implementations:
- `FixedGain`: `force = gainprm[0] * ctrl`
- `AffineGain`: `force = gainprm[0] + gainprm[1]*length + gainprm[2]*velocity`
- `MuscleGain`: Hill-type muscle model (existing `compute_muscle_gain()`)

These must produce identical results to the current enum-dispatched code.

##### S3. `SeriesElasticGain` (new, proof-of-concept)

A simple SEA model as the second non-MuJoCo implementation:
- `force = k_spring * (x_motor - x_joint) + d * (v_motor - v_joint)`
- Where `x_motor = ctrl * gear_ratio`, `v_motor = d(ctrl)/dt * gear_ratio`
- `Params`: `{ k_spring: f64, damping: f64, gear_ratio: f64 }`

This is a common actuator model in legged robotics that cannot be expressed as
MuJoCo's affine gain.

##### S4. Dispatch

The existing `GainType` enum stays for MuJoCo-compatible types (zero-cost match
dispatch in the hot loop). Custom gain models use trait dispatch:

```rust
pub enum GainDispatch {
    /// Built-in MuJoCo types (match dispatch, zero-cost).
    Builtin(GainType),
    /// Custom gain model (trait dispatch).
    Custom(Box<dyn ActuatorGainModel<Params = CustomGainParams>>),
}
```

For monomorphized dispatch (no vtable), the `SimBuilder` approach from §42F
would eliminate the `Box<dyn>`. §42D can use dynamic dispatch as a first step
since actuator force computation is not the innermost hot loop.

##### S5. MJCF configuration

Custom gain models are configured via `<general>` actuator with a custom attribute:

```xml
<general name="sea_hip" joint="hip" gaintype="custom"
         cortenforge:gain_model="series_elastic"
         cortenforge:gain_params="1000 10 50"/>
```

Or via the Rust API:
```rust
model.set_actuator_gain(actuator_id, SeriesElasticGain {
    k_spring: 1000.0, damping: 10.0, gear_ratio: 50.0,
});
```

#### Acceptance Criteria

1. `FixedGain`, `AffineGain`, `MuscleGain` through the trait produce identical
   forces as the current enum dispatch (bit-exact regression).
2. `SeriesElasticGain` produces correct spring-damper forces for a 1-DOF test case.
3. `dgain_dvel` is correct for all implementations (verify numerically with finite
   differences).
4. Models without custom gain types see zero overhead (existing enum path unchanged).
5. Implicit integrator works correctly with custom gain models (uses `dgain_dvel`).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `ActuatorGainModel` trait, built-in
  implementations, `SeriesElasticGain`, dispatch in `mj_fwd_actuation()`
- `sim/L0/mjcf/src/parser.rs` — parse custom gain attributes
- `sim/L0/tests/` — regression tests for built-in gains, SEA model tests

---

### 42E. Contact Solver Trait — Phase E (Contact Formulation Extensibility)
**Status:** Not started | **Effort:** XL | **Prerequisites:** §42B

#### Background

The current architecture has enum dispatch for contact solvers (`SolverType::PGS | CG
| Newton`). This works because all three solve the same LCP formulation — they differ
in numerics, not physics.

Future solvers may differ in **formulation**, not just solution strategy:

| Solver | Formulation | Physics | Use Case |
|--------|------------|---------|----------|
| PGS/CG/Newton | LCP (complementarity) | Rigid contact, Coulomb friction | Robotics, MuJoCo conformance |
| XPBD | Position-based constraints | Compliant contact, regularized | Real-time, games, animation |
| Impulse-based | Velocity-level impulses | Event-driven, exact collision | Billiards, granular media |
| Compliant contact | Kelvin-Voigt / Hunt-Crossley | Viscoelastic contact | Soft robotics, grasping |

These aren't different algorithms for the same problem — they're different **problem
formulations**. An enum can't capture this because the input/output types differ:
LCP solvers consume a Delassus matrix + constraint bounds, XPBD consumes position
constraints + compliance, impulse-based solvers consume collision events + restitution.

#### Specification

##### S1. `ContactSolver` trait

```rust
pub trait ContactSolver {
    /// Solver-specific configuration.
    type Config;

    /// Solver-specific per-step workspace.
    type Workspace;

    /// Allocate workspace for this step's contacts.
    fn prepare(
        config: &Self::Config,
        model: &Model,
        data: &Data,
        contacts: &[Contact],
    ) -> Self::Workspace;

    /// Solve for contact forces / impulses / position corrections.
    /// Modifies data.qacc (or data.qpos for position-level solvers).
    fn solve(
        config: &Self::Config,
        workspace: &mut Self::Workspace,
        model: &Model,
        data: &mut Data,
        contacts: &[Contact],
    );
}
```

##### S2. `LcpSolver` (extract current implementation)

Wraps the existing PGS/CG/Newton solver behind the trait:
- `Config`: `{ solver_type: SolverType, iterations: usize, tolerance: f64, noslip_iterations: usize }`
- `Workspace`: The existing `ConstraintState` (Delassus matrix, lambda, residuals)
- `prepare()`: Assemble Jacobian + Delassus (existing `mj_fwd_constraint()` logic)
- `solve()`: Dispatch to PGS/CG/Newton based on `solver_type` (existing solver code)

The internal PGS/CG/Newton enum dispatch stays — this is numerics-level dispatch within
a single formulation.

##### S3. `XpbdSolver` (new implementation)

Extended Position-Based Dynamics for compliant contact:
- `Config`: `{ iterations: usize, substeps: usize }`
- `Workspace`: Position constraint data, compliance matrices
- `prepare()`: Generate position-level contact constraints from penetration depths
- `solve()`: Iterative constraint projection with compliance:
  `Δx = -C(x) / (∇C·M⁻¹·∇Cᵀ + α/dt²)` where `α` is compliance

XPBD is the standard real-time physics formulation (Macklin et al. 2016). It's
simpler than LCP, faster per iteration, but less physically accurate.

##### S4. Dispatch

```rust
pub enum ContactSolverType {
    /// LCP solver (PGS/CG/Newton). MuJoCo-conformant.
    Lcp(SolverType),
    /// XPBD compliant contact. Real-time, position-based.
    Xpbd,
}
```

Default: `Lcp(SolverType::Newton)`.

##### S5. MJCF configuration

```xml
<option solver="newton"/>  <!-- existing, unchanged -->
<option cortenforge:solver="xpbd" cortenforge:xpbd_substeps="4"/>
```

#### Acceptance Criteria

1. `LcpSolver` through the trait produces identical results as the current direct
   implementation for PGS, CG, and Newton (bit-exact regression).
2. `XpbdSolver` produces stable contact for a box resting on a plane.
3. `XpbdSolver` resolves interpenetration within the specified iterations.
4. Models using default solver see zero overhead (LCP path unchanged).
5. Switching solver type at runtime (between steps) works correctly.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `ContactSolver` trait, `LcpSolver`,
  `XpbdSolver`, dispatch in constraint solve stage
- `sim/L0/tests/` — regression tests for LCP, XPBD stability tests

---

### 42F. `SimBuilder` Composition — Phase D (Trait Assembly)
**Status:** Not started | **Effort:** L | **Prerequisites:** §42C, §42D, §42E

#### Background

§42B (Phase A) and §42C/D/E (Phases B, C, E) each introduce a trait boundary.
Phase D assembles them into a composable builder that produces a fully monomorphized
simulation type — no vtable overhead in the inner loop.

#### Specification

##### S1. Generic `Sim` type

```rust
pub struct Sim<B, E, A, C>
where
    B: FlexBendingModel,
    E: FlexElasticityModel,
    A: ActuatorGainModel,
    C: ContactSolver,
{
    model: Model,
    data: Data,
    bending: B,
    elasticity: E,
    actuators: A,
    contact_solver: C,
}
```

##### S2. `SimBuilder`

```rust
pub struct SimBuilder<B = CotangentBending, E = LinearElastic, A = DefaultGain, C = LcpSolver> {
    bending: B,
    elasticity: E,
    actuators: A,
    contact_solver: C,
}

impl SimBuilder {
    pub fn new() -> Self { /* defaults: MuJoCo-conformant config */ }
}

impl<B, E, A, C> SimBuilder<B, E, A, C> {
    pub fn bending<B2: FlexBendingModel>(self, b: B2) -> SimBuilder<B2, E, A, C> { ... }
    pub fn elasticity<E2: FlexElasticityModel>(self, e: E2) -> SimBuilder<B, E2, A, C> { ... }
    pub fn actuators<A2: ActuatorGainModel>(self, a: A2) -> SimBuilder<B, E, A2, C> { ... }
    pub fn contact_solver<C2: ContactSolver>(self, c: C2) -> SimBuilder<B, E, A, C2> { ... }
    pub fn build(self, model: Model) -> Result<Sim<B, E, A, C>, Error> { ... }
}
```

##### S3. Type aliases for common configurations

```rust
/// MuJoCo-conformant defaults. This is what `Model::make_data()` + `Data::step()` uses.
pub type MujocoSim = Sim<CotangentBending, LinearElastic, DefaultGain, LcpSolver>;

/// Large-deformation cloth/animation preset.
pub type AnimationSim = Sim<BridsonBending, LinearElastic, DefaultGain, XpbdSolver>;

/// Soft robotics / biomechanics preset.
pub type SoftBodySim = Sim<CotangentBending, NeoHookean, DefaultGain, LcpSolver>;
```

##### S4. Backward compatibility

The existing `Model`/`Data` API does not change:
- `load_model()` + `make_data()` + `step()` continues to work exactly as before.
- Internally, this uses `MujocoSim` (the default configuration).
- `SimBuilder` is the power-user API for non-default configurations.

##### S5. `Sim::step()` pipeline

`Sim::step()` calls the same pipeline stages as `Data::step()`, but dispatches
trait calls through the generic parameters instead of hardcoded implementations:

```rust
impl<B, E, A, C> Sim<B, E, A, C>
where
    B: FlexBendingModel,
    E: FlexElasticityModel,
    A: ActuatorGainModel,
    C: ContactSolver,
{
    pub fn step(&mut self) -> Result<(), Error> {
        mj_step_common(&self.model, &mut self.data)?;
        self.bending.apply_forces(...);
        self.elasticity.apply(...);
        self.actuators.gain(...);
        self.contact_solver.solve(...);
        mj_step_integrate(&self.model, &mut self.data)?;
        Ok(())
    }
}
```

All dispatch is monomorphized — the compiler sees concrete types, not trait objects.

#### Acceptance Criteria

1. `SimBuilder::new().build(model)` produces a `MujocoSim` that passes all existing
   conformance tests.
2. `SimBuilder::new().bending(BridsonBending).build(model)` produces an `AnimationSim`
   that uses Bridson bending.
3. Mixing traits: `SimBuilder::new().elasticity(NeoHookean::new(mu, lambda)).contact_solver(XpbdSolver::new(4)).build(model)` compiles and runs.
4. `Model::make_data()` + `Data::step()` is unchanged — no regression, no API break.
5. Compile-time monomorphization: no vtables in the hot path (verify via assembly
   inspection or `cargo-asm`).
6. Domain randomization example: a training loop that randomly selects bending model
   per episode compiles and produces distinct dynamics.

#### Files
- `sim/L0/core/src/sim_builder.rs` — new module: `Sim<B, E, A, C>`, `SimBuilder`,
  type aliases
- `sim/L0/core/src/lib.rs` — re-export `SimBuilder`, `MujocoSim`, etc.
- `sim/L0/core/src/mujoco_pipeline.rs` — refactor pipeline stages to accept trait
  parameters
- `sim/L0/tests/` — builder composition tests, regression tests, domain randomization
  example test

---
