# URDF Loading — URDF-to-Model Pipeline

Ten examples covering the full `sim-urdf` pipeline: parsing URDF XML,
converting to MJCF, and compiling to a `sim-core::Model`. Each example
isolates a single URDF feature and validates it with headless checks.

The pipeline is: URDF XML → `urdf_to_mjcf()` → MJCF XML → `load_model()` → Model/Data.

## Examples

| Example | What it demonstrates |
|---------|---------------------|
| [revolute](revolute/) | Revolute joint pendulum, period matches analytical prediction |
| [prismatic](prismatic/) | Prismatic (slide) joint with spring, axis mapping |
| [continuous](continuous/) | Unlimited revolute, constant torque → linear velocity ramp |
| [fixed](fixed/) | Fixed joints fuse into parent via `fusestatic`, body count reduces |
| [mimic](mimic/) | `<mimic>` → equality constraint, leader-follower 2:1 coupling |
| [geometry](geometry/) | Box, sphere, cylinder size conversion (URDF full → MJCF half) |
| [inertia](inertia/) | Diagonal vs off-diagonal inertia tensors, precession difference |
| [damping-friction](damping-friction/) | Damping (velocity-dependent) vs frictionloss (constant), decay profiles |
| [error-handling](error-handling/) | Invalid URDFs produce correct error variants, no panics |
| [stress-test](stress-test/) | 31 headless checks covering the full pipeline |

## Key conversions

| URDF | MJCF | Notes |
|------|------|-------|
| `revolute` | `hinge` + `limited="true"` | Range from `<limit>` |
| `continuous` | `hinge` + `limited="false"` | No limits |
| `prismatic` | `slide` + `limited="true"` | Range from `<limit>` |
| `fixed` | eliminated by `fusestatic="true"` | Body merges into parent |
| `floating` | `free` | 6 DOF (3 translate + 3 rotate) |
| `planar` | 2 slides + 1 hinge | 3 DOF in plane |
| `box size="x y z"` | `box size="x/2 y/2 z/2"` | Full → half extents |
| `cylinder r l` | `cylinder size="r l/2"` | Full → half length |
| `sphere r` | `sphere size="r"` | Unchanged |
| `<dynamics damping>` | `damping="..."` | Velocity-dependent |
| `<dynamics friction>` | `frictionloss="..."` | Velocity-independent |
| `<mimic>` | `<equality><joint polycoef>` | Polynomial coupling |

All examples are headless (no Bevy, no window). Run any with:

```
cargo run -p example-urdf-<name> --release
```

Exit code 0 = all checks pass, exit code 1 = failure.
