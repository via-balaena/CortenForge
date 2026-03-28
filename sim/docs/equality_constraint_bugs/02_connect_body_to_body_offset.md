# Bug 2: Body-to-Body Connect Steady-State Offset

**Status:** Investigating
**Severity:** High — 55mm permanent offset in zero gravity

## Symptom

Two free bodies connected body-to-body via a connect constraint converge
to a position with a 55mm offset — even with zero gravity and no external
forces. The offset appears within the first 0.3s and stays permanently.
Angular velocity dampens to near zero.

## Reproduction

```
Diagnostic test D1 in stress-test/src/main.rs

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
there permanently. The system loses all kinetic energy (damped by
constraint) but converges to the WRONG equilibrium.

## Key Observations

1. **Single-body connect to world works perfectly.** Tests A, B, C all
   show sub-1mm error. The bug is specific to body-to-body connect.

2. **Two-body connect to world works perfectly.** Test D (body-to-body
   with anchor at body center, no offset) shows 0.31mm → 0.00mm error.

3. **The offset is permanent.** After energy is damped out, the system
   sits at a wrong equilibrium. This is NOT a transient — it's a steady-
   state error in the constraint formulation.

4. **The offset grows with the angular kick.** Larger initial angular
   velocity → larger steady-state offset. With zero initial velocity,
   the error might be smaller or zero.

## What Needs Investigation

1. **Anchor point interpretation.** In `extract_connect_jacobian()`,
   the anchor is in body1's frame. The constraint enforces
   `body1_pos + R1 * anchor = body2_pos`. But what happens to this
   constraint when body1 ROTATES? Does the Jacobian correctly track
   the rotating anchor?

2. **Angular vs translational Jacobian for anchor offset.** Similar to
   Bug 1, an anchor that's offset from body1's center creates angular
   coupling. The anchor at (0,0,-0.5) means `r = anchor_world - jpos`,
   which gives angular Jacobian entries. Verify these are correct.

3. **Is the body2 Jacobian contribution correct?** The connect constraint
   calls `add_body_point_jacobian_row` twice: once for body1 (sign=+1)
   with point=anchor_world, once for body2 (sign=-1) with point=p2.
   Verify that both calls produce correct Jacobian entries.

4. **Equilibrium analysis.** At the wrong equilibrium (55mm offset),
   what are the constraint forces? If they're zero (as they should be
   at equilibrium), is the Jacobian null space allowing this state?

## Files

- `sim/L0/core/src/constraint/equality.rs:36` — `extract_connect_jacobian()`
- `sim/L0/core/src/constraint/equality.rs:55` — anchor_world computation
- `sim/L0/core/src/constraint/equality.rs:65-77` — body1 and body2 Jacobian
