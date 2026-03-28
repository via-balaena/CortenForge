# Bug 3: Double Connect Chain Explosion

**Status:** FIXED (resolved by Bug 2 fix + solref tuning)
**Severity:** Was High — system explodes (6M rad/s angular velocity)

## Symptom

Two free bodies with two connect constraints (connect-to-world +
connect body-to-body) go violently unstable: angular velocities reach
millions of rad/s, positions diverge by meters.

## Reproduction

```
Test 1 (Connect) in stress-test/src/main.rs

Scene: 2 free bodies, gravity, nv=12
  body 1 (link1): pos=(0, 0, 0), DOFs 0-5
  body 2 (link2): pos=(0, 0, -0.5), DOFs 6-11

Constraints:
  connect body1=link1 to world, anchor="0 0 0"
  connect body1=link1 body2=link2, anchor="0 0 -0.5"

Initial: qvel[4] = 2.0 (angular kick)

Result after 5s:
  pivot1 error: 1029mm (should be ~0)
  pivot2 error: 29480mm (should be ~0)
  max angular velocity: 6,444,832 rad/s (explosion)
```

## Root Cause: Bug 2 Amplified by Gravity

This is **Bug 2** (free joint angular Jacobian uses world-frame axes
instead of body rotation columns) combined with gravity:

### Cascade Mechanism

1. **Angular kick** starts body1 rotating. Body1's orientation deviates
   from identity.

2. **Bug 2 activates:** The body-to-body connect Jacobian (rows 3-5)
   uses wrong angular DOF entries. Constraint forces on body1 project
   incorrectly onto angular DOFs.

3. **Pivot 2 drifts:** The body-to-body constraint fails to maintain
   zero error. Body2's anchor drifts away from link2's origin.

4. **Gravity accelerates body2:** With pivot 2 broken, body2 falls
   freely under gravity. The constraint tries to correct but applies
   force in wrong angular directions (Bug 2).

5. **Error compounds:** Body2's downward acceleration increases the
   constraint violation. The incorrect constraint force creates more
   rotation on body1, which makes the Jacobian even more wrong.

6. **Positive feedback → explosion:** The constraint force magnitude
   grows exponentially. With penalty-based constraints, there's no
   hard limit on force — larger error → larger force → larger wrong
   torque → more rotation → larger error.

### Why Single Constraints Don't Explode

- **Connect-to-world with anchor at origin** (pivot 1): anchor = (0,0,0),
  so `r = 0` for body1. No angular coupling in Jacobian. The body-frame
  vs world-frame distinction is irrelevant.

- **Connect body-to-body in zero gravity** (Bug 2 test): No gravity means
  no sustained external force. The system settles at a wrong equilibrium
  (55mm offset) but doesn't explode. Energy is finite and gets dissipated.

- **Double connect with gravity** (this bug): Gravity provides continuous
  energy input through body2. The wrong Jacobian prevents the constraint
  from correctly opposing gravity's effect, and the error grows without
  bound.

## Fix Spec

**No independent fix needed.** Bug 3 is entirely caused by Bug 2's
free joint angular Jacobian error. Fixing Bug 2 (body rotation columns
in `add_body_point_jacobian_row`) resolves this bug automatically.

See Bug 2 fix spec for the exact code change.

## Verification

After Bug 2's fix is applied:
- Test 1 (double connect pendulum) should show:
  - Pivot 1 error < 10mm (connect-to-world holds body1 at origin)
  - Pivot 2 error < 10mm (body-to-body tracks link2)
  - Angular velocity stays bounded (no explosion)
  - Energy bounded (< 5% growth from initial)
- The double pendulum should swing naturally under gravity

If pivot errors exceed 10mm after Bug 2 fix, investigate:
1. Whether the supplementary body2 anchor fix (from Bug 2) is needed
2. Whether the solref/solimp parameters need tuning for chain stability
3. Whether there's a CFL-like timestep condition for multi-constraint chains

## Files

Same as Bug 2:
- `sim/L0/core/src/constraint/equality.rs:473-491` — THE BUG (Free joint angular Jacobian)
