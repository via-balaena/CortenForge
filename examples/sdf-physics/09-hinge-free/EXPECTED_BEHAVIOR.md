# Example 09 — Hinge (Free Swing): Expected Behavior

## Setup

- **Base**: 10×10×10mm PLA cube, welded to world at origin (no joint — fixed anchor)
- **Arm**: 6×6×24mm PLA cuboid, connected to base via revolute joint
- **Joint**: Y-axis revolute at bottom face of base → arm swings in XZ plane
- **Initial angle**: 60° from vertical (rest position = hanging straight down)
- **No ground plane**, no collision between bodies

### Design choice: welded base vs free base

The base is welded to world (no joint) rather than using `JointKind::Free`. A free
base would fall under gravity, and in freefall the effective gravitational torque on
the hinge vanishes (equivalence principle for constrained rigid body systems). A welded
base provides a fixed pivot for clean pendulum dynamics.

## What you should see

1. **t=0–1s**: Arm swings down from 60° initial displacement, accelerating under gravity
2. **t=1–5s**: Arm oscillates back and forth like a pendulum. With RK4 integration
   and no damping, oscillation amplitude stays nearly constant.
3. **t=5s+**: PASS/FAIL checks run. Arm should still be swinging with minimal
   energy drift.

The blue cube (base) stays fixed. The red elongated cuboid (arm) swings smoothly
in the XZ plane (left-right when viewed from the default camera angle).

## qpos/qvel layout

Only one revolute joint in the model:
- `nq = 1`: `qpos[0]` = hinge angle (radians, 0 = rest/hanging down)
- `nv = 1`: `qvel[0]` = angular velocity (rad/s)

## Pass criteria

| Check | Threshold |
|-------|-----------|
| Arm still swinging at t≥5s | \|ω\| > 0.01 rad/s |
| No NaN/Inf | θ, ω both finite |
| No runaway spinning | \|θ\| < 2π |
| Oscillation observed | max(θ) > 0 and min(θ) < 0 |
| Energy conserved | \|ΔE\| < 5% |
| No explosion | \|ω\| < 100 rad/s |

## Key milestone

This is the **first SDF-physics example with articulation**. Examples 01–08 used
only `JointKind::Free` bodies. This proves that revolute joints work correctly
with SDF geom types through the `Mechanism → to_model()` pipeline.
