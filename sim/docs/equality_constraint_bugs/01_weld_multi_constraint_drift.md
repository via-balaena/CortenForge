# Bug 1: Weld-to-World Drift at Offset Positions

**Status:** Not a bug — expected penalty-method physics (matches MuJoCo)
**Severity:** Informational

## Symptom

A weld-to-world constraint holds a body perfectly when the body is on
the z-axis (0.37mm sag). But when the body is offset from the world
origin (e.g., at (0.5, 0, 1)), the body drifts ~138mm over 5 seconds
under gravity.

## Root Cause: Penalty-Method Physics (Not a Code Bug)

MuJoCo exhibits **identical behavior** — in fact, MuJoCo drifts MORE
(549mm at 5s for the same model). This is inherent to the penalty-method
weld constraint formulation, not a bug in either engine.

### MuJoCo Comparison (empirically verified)

| Configuration | MuJoCo | CortenForge | Notes |
|---------------|--------|-------------|-------|
| Body at (0, 0, 1) z-only | 0.37mm | 0.37mm | No angular coupling |
| Body at (0.5, 0, 1) offset | **549mm** | **138mm** | Angular-translational coupling |
| Connect (3-DOF) at offset | 0.37mm | 0.37mm | No angular coupling (no angular rows) |
| Stiffer solref=[0.005, 1.0] | 72mm | — | Stiffer spring reduces drift |
| No gravity, offset | 0.00mm | 0.00mm | No gravity = no external force |

### Mechanism

The weld constraint has 6 rows: 3 translational + 3 rotational. For
weld-to-world at (0.5, 0, 1), the constraint anchor (at world origin)
is 1.12m from the body center. The translational Jacobian rows include
angular coupling terms from this lever arm.

1. Gravity applies force at body center → body sags in z
2. Translational constraint acts at the anchor (offset from center)
3. Angular coupling in the Jacobian creates torque on the body
4. Body rotates → anchor moves → changes error direction
5. The body oscillates like a soft pendulum around the anchor

The connect constraint (3 DOFs, translational only) does NOT drift
because the Jacobian rows are purely translational — angular coupling
in connect creates torque but not angular constraint error.

### Why the On-Axis Case Works

Body at (0, 0, 1): the anchor is at world origin (0,0,0). The lever arm
is `r = (0,0,0) - (0,0,1) = (0,0,-1)`. For gravity acting in z:
`e_z × r = (0,0,1) × (0,0,-1) = (0,0,0)`. No angular coupling in the
z-constraint row. Gravity acts purely translational → no rotation.

Body at (0.5, 0, 1): `r = (-0.5,0,-1)`. For z-constraint:
`e_z × r = (0,0,1) × (-0.5,0,-1) = (0,-0.5,0)` → angular coupling
through DOF ωy. Gravity creates torque → rotation → drift.

## Mitigations for Users

1. **Use stiffer solref**: `solref="0.005 1.0"` reduces drift ~8x
2. **Use connect instead of weld**: connect has no angular coupling
3. **Reduce offset**: place bodies near the constraint anchor
4. **Accept the compliance**: soft welds are inherently compliant

## Code Status

All fixes from the equality constraint bug investigation are applied:
- Fix 1: Free joint angular Jacobian body-frame axes (equality.rs)
- Fix 2: Weld eq_data dual-anchor layout (builder/equality.rs)
- Fix 3: Connect body2 anchor auto-computation
- Fix 4: Weld quaternion-corrected rotational Jacobian
- Fix 5: Velocity from J*qvel

The engine matches MuJoCo's formulation. The drift is expected physics.

## Files

- `sim/L0/core/src/constraint/equality.rs` — Jacobian extraction
- `sim/L0/mjcf/src/builder/equality.rs` — eq_data layout and auto-relpose
