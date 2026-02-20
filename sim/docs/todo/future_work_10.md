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
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State

`density`, `viscosity`, and `wind` fields are parsed from MJCF `<option>` and stored
on `Model`, but no fluid force computation exists in the pipeline. Any model relying
on fluid interaction (swimming, flying, falling with drag) produces incorrect results.

The dm_control fruitfly model in our test assets uses `fluidshape="ellipsoid"` with
`fluidcoef` for wing aerodynamics — this model is silently broken.

#### MuJoCo Reference

MuJoCo has **two** fluid models (`engine_passive.c`), dispatched per-geom by the
`fluidshape` attribute:

| Model | Activation | Scope | Force Components |
|-------|-----------|-------|-----------------|
| **Inertia-box** (legacy) | `fluidshape="none"` (default) | Per-body (uses body inertia box) | Quadratic drag + viscous resistance |
| **Ellipsoid** (advanced) | `fluidshape="ellipsoid"` on geom | Per-geom (uses geom ellipsoid semi-axes) | Added mass + quadratic drag + Magnus lift + Kutta lift + viscous drag + viscous torque |

Both models are gated by `density > 0 || viscosity > 0`. When both are zero
(default), all fluid computation is skipped.

##### Inertia-Box Model (`mj_inertiaBoxFluidModel`)

Computes equivalent box dimensions from the body's diagonal inertia:
```
box[i] = sqrt(max(MINVAL, (I_j + I_k - I_i)) / mass * 6.0)
```

**Quadratic drag** (high Reynolds, per-axis in body frame):
```
f_D,i = -2 * ρ * r_j * r_k * |v_i| * v_i
g_D,i = -0.5 * ρ * r_i * (r_j⁴ + r_k⁴) * |ω_i| * ω_i
```

**Viscous resistance** (low Reynolds):
```
r_eq = (r_x + r_y + r_z) / 3           // equivalent sphere radius
f_V,i = -6 * β * π * r_eq * v_i
g_V,i = -8 * β * π * r_eq³ * ω_i
```

Where `ρ = option.density`, `β = option.viscosity`, `v = body_vel - wind` (in
body frame), `ω = body_angvel` (in body frame).

##### Ellipsoid Model (`mj_ellipsoidFluidModel`)

Per-geom computation using equivalent ellipsoid semi-axes. 6 force components:

**1. Added mass** (acceleration-dependent, from Tuckerman's ellipsoid theory):
```
f_A = -m_A · v̇ + (m_A · v) × ω
g_A = -I_A · ω̇ + (m_A · v) × v + (I_A · ω) × ω
```
Where `m_A[i] = ρ * V * α_i` (added mass coefficients from ellipsoid integrals),
`V = 4π/3 * r_x * r_y * r_z`.

**2. Viscous drag** (blunt + slender decomposition):
```
f_D = -ρ * [C_D_blunt * A_proj + C_D_slender * (A_max - A_proj)] * ‖v‖ * v
```
Where `A_proj` is the projected ellipsoid area normal to velocity.

**3. Angular drag**:
```
g_D = -ρ * C_D_angular * [I_D + C_D_slender/C_D_angular * (I_max - I_D)] · ω
```

**4. Magnus force** (spin-induced lift):
```
f_M = C_M * ρ * V * (ω × v)
```
Zero for zero angular velocity.

**5. Kutta lift** (circulation from asymmetric cross-section):
```
f_K = C_K * ρ * A_proj * (v̂ · n_s) * ((n_s × v) × v) / ‖v‖
```
Zero for spheres (all semi-axes equal). `n_s` is the ellipsoid slender axis.

**6. Viscous resistance** (low Reynolds):
```
r_D = (r_x + r_y + r_z) / 3
f_V = -6 * π * r_D * β * v
g_V = -8 * π * r_D³ * β * ω
```

**5 tunable coefficients** per geom (`fluidcoef` attribute):

| Index | Parameter | Default |
|-------|-----------|---------|
| 0 | `C_D_blunt` (blunt drag) | 0.5 |
| 1 | `C_D_slender` (slender drag) | 0.25 |
| 2 | `C_D_angular` (angular drag) | 1.5 |
| 3 | `C_K` (Kutta lift) | 1.0 |
| 4 | `C_M` (Magnus lift) | 1.0 |

##### Ellipsoid semi-axes (`mju_geom2Ellipsoid`)

MuJoCo computes equivalent ellipsoid semi-axes per geom type:

| Geom Type | Semi-axes (r_x, r_y, r_z) |
|-----------|--------------------------|
| Sphere | `(r, r, r)` |
| Capsule | `(r, r, r + l/2)` |
| Cylinder | `(r, r, l/2)` |
| Ellipsoid | `(r_x, r_y, r_z)` — direct |
| Box | `(sx, sy, sz)` — half-sizes |

Mesh geoms use the bounding box half-extents as the equivalent ellipsoid.

#### Objective

Implement both MuJoCo fluid models (inertia-box and ellipsoid) in the passive
force pipeline.

#### Specification

##### S1. MJCF parsing

Add to `MjcfGeom`:
```rust
pub fluidshape: Option<FluidShape>,      // "none" | "ellipsoid"
pub fluidcoef: Option<[f64; 5]>,         // [C_D_blunt, C_D_slender, C_D_angular, C_K, C_M]
```

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FluidShape {
    #[default]
    None,       // Use inertia-box model (body-level)
    Ellipsoid,  // Use ellipsoid model (per-geom)
}
```

Parse from `<geom>` element and inherit from `<default>` class.

##### S2. Model storage

Add to `Model`:
```rust
pub geom_fluidshape: Vec<FluidShape>,
pub geom_fluidcoef: Vec<[f64; 5]>,       // defaults: [0.5, 0.25, 1.5, 1.0, 1.0]
pub geom_fluid_ellipsoid: Vec<Vector3<f64>>,  // precomputed semi-axes
```

Precompute `geom_fluid_ellipsoid` at model build time via `mju_geom2Ellipsoid`.

##### S3. Fluid force computation

Add `mj_fluid()` helper called from `mj_fwd_passive()`:

```rust
fn mj_fluid(model: &Model, data: &mut Data) {
    let rho = model.density;
    let beta = model.viscosity;
    if rho == 0.0 && beta == 0.0 { return; }  // early exit: no fluid

    // Phase 1: Inertia-box model (body-level, for geoms with fluidshape=None)
    for body_id in 1..model.nbody {  // skip world body
        if !body_has_inertiabox_geoms(model, body_id) { continue; }
        mj_inertia_box_fluid(model, data, body_id, rho, beta);
    }

    // Phase 2: Ellipsoid model (per-geom, for geoms with fluidshape=Ellipsoid)
    for geom_id in 0..model.ngeom {
        if model.geom_fluidshape[geom_id] != FluidShape::Ellipsoid { continue; }
        mj_ellipsoid_fluid(model, data, geom_id, rho, beta);
    }
}
```

##### S4. Inertia-box implementation

For each body, compute equivalent box from diagonal inertia, then apply:
- Quadratic drag forces (body-frame, per-axis)
- Viscous resistance forces
- Transform to world frame, accumulate into `qfrc_passive`

##### S5. Ellipsoid implementation

For each ellipsoid-mode geom:
1. Get body velocity at geom center (linear + angular)
2. Subtract wind: `v_fluid = v_body - wind`
3. Transform to geom-local frame
4. Compute all 6 force components in geom-local frame
5. Transform forces/torques to world frame
6. Accumulate into `qfrc_passive` via body Jacobian at geom center

##### S6. Implicit integration derivatives (future)

MuJoCo provides full derivatives of fluid forces for implicit integration
stability. This is important for high-density fluids and stiff scenarios.
Mark as a sub-item for later — explicit integration is sufficient for initial
conformance. Note: without derivatives, implicit integrator will not account
for fluid forces in the implicit damping matrix.

#### Acceptance Criteria

1. **Zero fluid regression**: `density=0, viscosity=0` produces zero fluid force.
   All existing tests pass unchanged.

2. **Terminal velocity**: A sphere (`density=1000 kg/m³, radius=0.1, mass=1`)
   falling in fluid (`option density=1.2, viscosity=1.8e-5`) reaches terminal
   velocity. Verify steady-state velocity matches MuJoCo within 1%.

3. **Wind force**: A stationary body with `wind="5 0 0"` and `density=1.2`
   experiences a lateral force. Verify force direction and magnitude match MuJoCo.

4. **Viscous Stokes drag**: A small sphere (`radius=0.001`) moving slowly in
   `viscosity=1.0` fluid. Verify `F ≈ -6π * viscosity * r * v` (Stokes law).
   Compare against MuJoCo within `1e-6`.

5. **Ellipsoid model — drag**: A non-spherical ellipsoid (`size="0.1 0.05 0.02"`)
   with `fluidshape="ellipsoid"` falling in fluid. Verify drag depends on
   orientation (projected area changes with attitude).

6. **Ellipsoid model — Magnus lift**: A spinning sphere (`fluidshape="ellipsoid"`)
   moving through fluid produces a lateral force (Magnus effect). Verify
   `f_M = C_M * ρ * V * (ω × v)` matches MuJoCo.

7. **Ellipsoid model — Kutta lift**: A non-spherical ellipsoid with velocity
   at an angle to its principal axis produces Kutta lift force. Zero for spheres.

8. **Fluidcoef tuning**: Custom `fluidcoef="1.0 0.5 2.0 0.0 0.0"` zeroes
   Magnus and Kutta while doubling blunt drag. Verify forces match MuJoCo.

9. **Inertia-box model**: A body with multiple geoms (all `fluidshape="none"`)
   uses the body-level inertia box for drag. Verify equivalent box dimensions
   from `body_inertia` match MuJoCo's computation.

10. **Mixed model**: A body with some geoms `fluidshape="none"` and others
    `fluidshape="ellipsoid"` applies both models correctly (body-level inertia-box
    for none-geoms, per-geom ellipsoid for ellipsoid-geoms).

11. **MuJoCo conformance (comprehensive)**: Load the dm_control fruitfly model
    (which uses `fluidshape="ellipsoid"` for wing geoms). Run 100 steps. Compare
    `qfrc_passive` against MuJoCo 3.4.0 reference. Tolerance: `1e-6` per component.

#### Files

- `sim/L0/mjcf/src/types.rs` — `FluidShape` enum, add `fluidshape`/`fluidcoef` to
  `MjcfGeom`
- `sim/L0/mjcf/src/parser.rs` — parse `fluidshape`, `fluidcoef` attributes
- `sim/L0/mjcf/src/model_builder.rs` — precompute `geom_fluid_ellipsoid`,
  wire `fluidshape`/`fluidcoef` to Model
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fluid()`, `mj_inertia_box_fluid()`,
  `mj_ellipsoid_fluid()` in `mj_fwd_passive()`, Model fields
- `sim/L0/tests/integration/` — fluid force tests (new file)

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

10. **Default values**: An unmodified `<option/>` with no `<flag>` produces
    `disableflags == 0` (all enable) and `enableflags == 0` (all optional
    features off). Matches MuJoCo defaults.

11. **Backward compat**: `<flag passive="disable"/>` sets both `DISABLE_SPRING`
    and `DISABLE_DAMPER` (legacy behavior).

12. **MuJoCo conformance**: For each of the 19 disable flags, compare 10-step
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
