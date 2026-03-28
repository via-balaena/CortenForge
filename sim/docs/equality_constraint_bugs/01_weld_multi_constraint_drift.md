# Bug 1: Weld-to-World Drift in Multi-Constraint Scenes

**Status:** Investigating
**Severity:** High — body drifts 138mm instead of 0.37mm

## Symptom

A weld-to-world constraint holds a body perfectly in isolation (0.37mm
steady-state sag under gravity). But when a SECOND weld constraint
(body-to-body pair) exists in the same scene — even on completely
independent bodies with no shared DOFs — the weld-to-world body drifts
138mm+ over 5 seconds.

## Reproduction

```
Diagnostic test D2 in stress-test/src/main.rs

Scene: 3 free bodies, nv=18
  body 1 (base): pos=(-0.5, 0, 1.5), DOFs 0-5
  body 2 (arm):  pos=(-0.5, 0, 1.2), DOFs 6-11
  body 3 (fixed): pos=(0.5, 0, 1.0), DOFs 12-17

Constraints: ne=12
  weld body1=base body2=arm    (rows 0-5)
  weld body1=fixed (to world)  (rows 6-11)

Result after 5s:
  fixed body displacement: 137.98mm (expected ~0.37mm)
  weld pair error: 0.00mm (perfect)
  ncon=0 (no contacts)
```

## Key Observations

1. **Both PGS and Newton give identical results.** solver_niter=1 for
   both, meaning the solver converges in 1 iteration. The forces ARE the
   correct mathematical solution to the constraint system as formulated.

2. **The Jacobian is correct.** Verified by hand: the angular coupling
   terms match the expected `ω × r` formula for the constraint point.

3. **Displacement grows over time:** 20mm at 0.1s, 93mm at 1.0s, 138mm
   at 5.0s. This is NOT a steady-state sag — it's a progressive drift.

4. **Rotational force grows:** The ωy constraint force on the fixed body
   grows from -0.07 at step 0 to -12.34 at 5s, indicating the body is
   rotating.

5. **The pair weld (rows 0-5) forces are zero.** At step 0, all 6 forces
   on the pair constraint are -0.0000. The pair starts at equilibrium
   and the solver correctly assigns zero force.

## Jacobian Dump (step 0)

```
Fixed body weld (rows 6-11):
  Row 6 (x-pos): d12=1.000, d16=-1.000    force= 3.4913
  Row 7 (y-pos): d13=1.000, d15=1.000, d17=-0.500  force= 0.0000
  Row 8 (z-pos): d14=1.000, d16=0.500     force= 7.0834
  Row 9 (wx-rot): d15=0.500               force= 0.0000
  Row 10(wy-rot): d16=0.500               force=-0.0698
  Row 11(wz-rot): d17=0.500               force= 0.0000
```

The z-force is 7.08 instead of expected ~9.81. The missing 2.73 N
is being distributed to the x-force (3.49) and wy-force (-0.07)
through the angular coupling in DOF 16 (ωy).

## Hypothesis: Angular-Translational Coupling Creates Positive Feedback

The weld constraint reference point `cpos = body_pos + R * (anchor +
relpose_pos)` is at world origin (0,0,0) for a weld-to-world. But the
body center is at (0.5, 0, 1.0). The offset vector `r = cpos - jpos =
(-0.5, 0, -1)` creates angular coupling in the positional constraint
rows.

**The feedback loop:**
1. Gravity → translational sag → position error
2. Position error + angular coupling → rotational torque
3. Rotational torque → body rotation
4. Rotation → cpos moves → larger position error
5. Larger position error → stronger rotational torque → more rotation
6. Drift grows over time

This explains why single-body weld at (0,0,1) works: when the body is on
the z-axis, the z-constraint has NO angular coupling (ez.cross(r) = 0
for r = (0,0,-1)), so no feedback loop.

## Root Cause Confirmed: Angular-Translational Coupling

**Test C2 (single weld at (0.5, 0, 1)) produces IDENTICAL 137.98mm
drift.** The multi-constraint interaction is NOT the cause. The bug
is purely about the weld constraint formulation for bodies offset
from the constraint reference point.

The body at (0, 0, 1) works because r = cpos - jpos = (0,0,0) -
(0,0,1) = (0,0,-1), and ez.cross(r) = (0,0,1)×(0,0,-1) = (0,0,0),
so there's NO angular coupling in the z-constraint. Gravity acts
only on z, and the z-constraint has only translational stiffness.

The body at (0.5, 0, 1) fails because r = (0,0,0) - (0.5,0,1) =
(-0.5,0,-1), and the z-constraint row gets d16=0.5 angular coupling.
This creates the feedback loop: gravity → sag → angular torque →
rotation → cpos moves → more sag.

## What Needs Investigation

1. **Is this a MuJoCo behavior too?** Run the same model in MuJoCo and
   see if the penalty method produces the same drift. If MuJoCo doesn't
   drift, compare their weld Jacobian formulation.

2. **How does MuJoCo formulate the weld constraint?** The key question
   is: does MuJoCo compute cpos as `p1 + R1*(anchor + relpose_pos)`,
   or does it use a different formulation that separates position and
   orientation constraints?

3. **Alternative formulation.** Instead of tracking the constraint point
   `cpos` (which is offset from body center), track the body center
   directly: `pos_error = p1 - p1_target`. The relpose offset should
   define the TARGET, not modify the constraint point on body1.

4. **Baumgarte stabilization strength.** Even with coupling, if the
   stabilization is strong enough, the drift should be bounded. Check
   if the impedance parameters (solref/solimp) need to be position-
   dependent for weld constraints.

## Root Cause: `relpose_pos` on Wrong Side of Constraint Equation

**File:** `sim/L0/core/src/constraint/equality.rs`, line 130

```rust
// CURRENT (wrong): relpose_pos added to body1's offset
let offset = anchor + relpose_pos;
let cpos = p1 + r1 * offset;
let pos_error = cpos - p2;
```

This places the constraint point `cpos` far from body1's center. For
weld-to-world at (0.5,0,1): cpos = (0,0,0), r = (-0.5,0,-1), creating
angular coupling that drives the feedback loop.

**MuJoCo's actual formulation** keeps `relpose_pos` on body2's side:

```rust
// CORRECT: anchor on body1, relpose on body2 (the target)
let cpos = p1 + r1 * anchor;          // point on body1 (= body center when anchor=0)
let target = p2 + r2 * relpose_pos;   // reference point on body2
let pos_error = cpos - target;
```

For weld-to-world at (0.5,0,1): cpos = (0.5,0,1) = body center,
target = (0.5,0,1), r = 0, NO angular coupling.

## Proposed Fix

1. Change `extract_weld_jacobian()` to separate anchor (body1) and
   relpose_pos (body2)
2. Compute Jacobian at `cpos` for body1 and `target` for body2
3. Update auto-relpose computation in MJCF builder: relpose_pos should
   store `R2^{-1} * (cpos_initial - p2)` instead of
   `R1^{-1} * (p2 - p1) - anchor`
4. Update velocity computation similarly
5. Run existing integration tests to verify no regressions

## Files

- `sim/L0/core/src/constraint/equality.rs:107` — `extract_weld_jacobian()`
- `sim/L0/core/src/constraint/equality.rs:130` — THE BUG (offset computation)
- `sim/L0/core/src/constraint/equality.rs:429` — `add_body_point_jacobian_row()`
- `sim/L0/mjcf/src/builder/equality.rs` — auto-relpose computation
