# Equality Constraint Bugs

Found via stress-testing examples for the `equality-constraints/` domain.
Each bug has its own research document with root cause analysis and fix plan.

## Bugs

| # | Bug | Severity | Status |
|---|-----|----------|--------|
| 1 | [Weld-to-world drift in multi-constraint scenes](01_weld_multi_constraint_drift.md) | High | Root cause confirmed |
| 2 | [Body-to-body connect steady-state offset](02_connect_body_to_body_offset.md) | High | Root cause confirmed |
| 3 | [Double connect chain explosion](03_double_connect_explosion.md) | High | Root cause confirmed (cascades from #2) |

## Shared Root Cause: Free Joint Angular Jacobian Frame Error

All three bugs share a **single underlying cause**: the equality constraint
Jacobian functions (`add_body_point_jacobian_row` and `add_body_angular_jacobian_row`)
use **world-frame unit vectors** (`Vector3::x()`, `Vector3::y()`, `Vector3::z()`)
for free joint angular DOFs, but free joint angular velocity in `qvel` is in the
**body-LOCAL frame**.

The correct pattern (already used by the contact Jacobian since DT-75 and by
the Ball joint case) is to use body rotation matrix columns: `rot * Vector3::ith(i, 1.0)`.

At identity rotation (R = I), both approaches give the same result, which is why
simple tests pass. The bug only manifests when a free body rotates away from its
initial orientation AND has angular coupling in the constraint (offset anchor/constraint
point).

### Impact by Bug

| Bug | Angular coupling source | Amplification |
|-----|------------------------|---------------|
| 1 | Weld constraint point far from body center (`anchor + relpose_pos`) | Gravity drives sag → rotation → feedback loop |
| 2 | Connect anchor offset from body center | Angular kick → wrong damping → wrong equilibrium |
| 3 | Same as Bug 2 | Gravity + chain constraints → exponential divergence |

## Additional Issues (Bug 1 Only)

Bug 1 has two secondary issues beyond the shared Jacobian bug:

1. **Weld eq_data layout** differs from MuJoCo: we combine `anchor + relpose_pos`
   on body1's side. MuJoCo stores separate anchors per body.
2. **Weld rotational Jacobian** uses simple `0.5` scaling instead of MuJoCo's
   per-column quaternion correction.

## Fix Priority

1. **Fix the free joint angular Jacobian** (2 functions, ~12 lines) — resolves
   all 3 bugs. This is the DT-75 fix applied to equality constraints.
2. **Refactor weld eq_data layout** — match MuJoCo's separate body1/body2 anchors.
3. **Add body2 anchor to connect** — match MuJoCo's dual-anchor connect formulation.
4. **Quaternion-corrected weld rotational Jacobian** — accuracy far from reference pose.
5. **Velocity from J*qvel** — consistency with MuJoCo's `mj_referenceConstraint`.

## Methodology

- Stress test binary: `examples/fundamentals/sim-cpu/equality-constraints/stress-test/`
- Single-constraint baselines all pass (0.37mm sag for weld, 0.31mm for connect)
- Bugs manifest only when free bodies rotate with offset anchors/constraint points
- Both PGS and Newton solvers produce identical results (solver converges in 1 iteration)
- Issue is in constraint formulation (Jacobian frame), NOT solver convergence
- Root cause confirmed by comparison with MuJoCo source (`engine_core_constraint.c`)
  and our own contact Jacobian (which was already fixed for this in DT-75)

## Key Files

- `sim/L0/core/src/constraint/equality.rs` — Jacobian extraction (THE BUG)
- `sim/L0/core/src/constraint/jacobian.rs` — Contact Jacobian (correct reference)
- `sim/L0/core/src/forward/velocity.rs` — cvel computation (confirms body-local angular velocity)
- `sim/L0/core/src/constraint/equality_assembly.rs` — row assembly into efc_*
- `sim/L0/mjcf/src/builder/equality.rs` — eq_data layout and auto-relpose
