# Fixed Joint Fusion

Three URDF links: base → (fixed) sensor_mount → (revolute) arm.
With `fusestatic="true"`, the sensor_mount merges into the base. The
compiled model has fewer bodies than the URDF has links.

## What you see

A pendulum arm swinging — identical to the revolute example. The key
insight is in the HUD: "URDF: 3 links" but "Model: 2 bodies, 1 joint"
because the fixed-joint link was eliminated during compilation.

## What it tests

URDF fixed joints rigidly attach a child to its parent with zero DOF.
MuJoCo's `fusestatic="true"` merges these into the parent body. This
is important for real-world URDF files which commonly use fixed joints
for sensor mounts, tool flanges, and reference frames.

## Validation

| Check | Source |
|-------|--------|
| Fixed joint fused (nbody < 4) | `print_report` |
| Only revolute survives (njnt=1) | `print_report` |
| DOF = 1 | `print_report` |

## Run

```
cargo run -p example-urdf-fixed --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
