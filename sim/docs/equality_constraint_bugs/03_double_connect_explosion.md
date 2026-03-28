# Bug 3: Double Connect Chain Explosion

**Status:** Investigating — likely compounds Bug 2
**Severity:** High — system explodes (6M rad/s angular velocity)

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

## Relationship to Other Bugs

This is likely Bug 2 (body-to-body offset) compounded by gravity and
a second constraint:

1. Connect-to-world (rows 0-2) holds body1's origin at (0,0,0)
2. Connect body-to-body (rows 3-5) tries to hold body1's anchor at
   body2's origin
3. Bug 2 causes the body-to-body constraint to have steady-state error
4. Gravity accelerates body2 downward
5. The error compounds: body2 drifts, pulling body1 via the first
   constraint, creating a cascade

If Bug 2 is fixed (body-to-body connect achieves zero error), this
bug may resolve automatically. However, the explosion suggests
additional instability — possibly the angular coupling creating
positive feedback (similar to Bug 1).

## What Needs Investigation

1. **Fix Bug 2 first**, then re-test this scenario.
2. If still unstable, investigate whether the two connect constraints
   (6 rows total) create a coupled system that is inherently stiff
   for the penalty method.
3. Check if explicit integration of the penalty forces requires smaller
   timestep for multi-constraint chains (CFL-like stability condition).

## Files

Same as Bug 2 — this is the same code path but with two constraints
instead of one.
