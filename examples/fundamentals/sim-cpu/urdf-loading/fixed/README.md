# Fixed Joint Fusion

URDF fixed joints rigidly attach a child link to its parent with zero
degrees of freedom. In MuJoCo's MJCF, `fusestatic="true"` (the default
in the URDF converter) merges fixed-joint links into their parent body.
The compiled model has fewer bodies than the URDF has links.

## What it tests

Two scenarios:

1. **Single fixed joint:** base → (fixed) sensor_mount → (revolute) arm.
   The sensor_mount merges into base, leaving only 2 bodies and 1 joint.

2. **Chained fixed joints:** base → (fixed) mount1 → (fixed) mount2 →
   (revolute) arm. Both fixed links merge, still leaving 2 bodies and
   1 joint.

This is important for real-world URDF files which commonly use fixed
joints for sensor mounts, tool flanges, and reference frames that
shouldn't be separate rigid bodies in simulation.

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | Fixed joint reduces body count (nbody < 4) | exact |
| 2 | Only revolute joint survives (njnt = 1) | exact |
| 3 | Fused model simulates 500 steps without NaN | exact |
| 4 | Chained fixed joints both fuse (nbody < 5, njnt = 1) | exact |
| 5 | DOF = 1 (only revolute contributes) | exact |

## Run

```
cargo run -p example-urdf-fixed --release
```
