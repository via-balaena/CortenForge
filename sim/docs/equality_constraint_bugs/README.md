# Equality Constraint Bugs

Found via stress-testing examples for the `equality-constraints/` domain.
Each bug has its own research document with root cause analysis.

## Bugs

| # | Bug | Status |
|---|-----|--------|
| 1 | [Weld-to-world drift at offset positions](01_weld_multi_constraint_drift.md) | Not a bug — expected penalty-method physics (matches MuJoCo) |
| 2 | [Body-to-body connect steady-state offset](02_connect_body_to_body_offset.md) | **Fixed** — free joint angular Jacobian frame corrected |
| 3 | [Double connect chain explosion](03_double_connect_explosion.md) | **Fixed** — cascaded from Bug 2 |

## Summary of Fixes

All fixes applied in two commits on `feature/equality-constraint-examples`:

1. **Free joint angular Jacobian** (`equality.rs:444-465, 511-521`): Use body
   rotation columns (`R·eᵢ`) instead of world-frame unit vectors for angular
   DOFs. Matches the contact Jacobian (DT-75) and Ball joint pattern. This was
   the root cause of Bug 2 and Bug 3.

2. **Weld eq_data layout** (`builder/equality.rs`): Refactored to MuJoCo's
   dual-anchor convention: `[0..3]=anchor_body2, [3..6]=anchor_body1,
   [6..10]=relpose_quat`.

3. **Connect body2 anchor** (`builder/equality.rs`): Auto-computed at model
   build so both anchors coincide at qpos0.

4. **Weld rotational Jacobian** (`equality.rs`): Per-column quaternion
   correction `0.5·imag(q2⁻¹·[0,col]·q1·relpose)` instead of simple 0.5
   scaling. Orientation error switched to `imag(q2⁻¹·q1·relpose)`.

5. **Velocity from J·qvel** (`equality.rs`): Replaces cvel-derived velocity
   for Baumgarte stabilization consistency.

## Bug 1 Resolution

Bug 1 (weld drift at offset) is NOT a code bug. MuJoCo exhibits the same
behavior (549mm drift vs our 138mm for body at (0.5, 0, 1) welded to world).
The drift is inherent to soft (penalty-method) weld constraints with offset
anchors under gravity. See the bug doc for mitigations.

## Key Files

- `sim/L0/core/src/constraint/equality.rs` — Jacobian extraction (all 5 fixes)
- `sim/L0/core/src/constraint/jacobian.rs` — Contact Jacobian (reference for correct pattern)
- `sim/L0/core/src/forward/velocity.rs` — cvel computation (confirms body-local angular velocity)
- `sim/L0/mjcf/src/builder/equality.rs` — eq_data layout and auto-relpose
