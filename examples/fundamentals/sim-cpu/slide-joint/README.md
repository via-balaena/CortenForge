# Slide Joint Examples

Prismatic (slide) joints — 1 translational DOF along a specified axis.
Both examples use joint stiffness to create spring-mass oscillators with
animated coil springs.

| Example | What it shows |
|---------|---------------|
| [`horizontal/`](horizontal/) | Block between two wall springs on a frictionless rail. Validates period, energy dissipation, joint limits. |
| [`vertical/`](vertical/) | Block on a vertical rail with overhead spring. Validates gravity-shifted equilibrium `x_eq = -mg/k`, period, energy dissipation. |

## Key concepts

- `type="slide"` — 1 qpos (displacement), 1 qvel (velocity)
- `axis="1 0 0"` or `axis="0 0 1"` — translation direction
- `stiffness` — joint spring (restoring force toward qpos=0)
- `damping` — viscous damping
- `armature` — reflected inertia (effective mass = body mass + armature)
- Analytical period: `T = 2*pi * sqrt((m + armature) / k)`

## Run

```
cargo run -p example-slide-joint-horizontal --release
cargo run -p example-slide-joint-vertical --release
```
