# Cone Limit — Limited Ball Joint

A pendulum on a ball joint with a 45° cone limit. Displaced 60° from
vertical — beyond the limit — and released. The constraint solver pushes
it back inside the cone, where it oscillates and gradually settles.

## What you see

- **Red sphere** — the 1 kg tip mass, swinging within the cone boundary
- **Dark socket** — the ball-and-socket joint at the pivot
- **Steel frame** — horizontal beam with vertical posts (visual only)

## Physics

Ball joint limits define a **rotation cone** around the rest orientation.
The cone apex half-angle is set by `range="0 0.7854"` (45° in radians).
Any rotation exceeding this angle activates the limit constraint, which
applies a restoring force to push the body back inside.

The initial 60° displacement is intentionally **infeasible** — it violates
the constraint from frame zero. The solver resolves this within a few
timesteps, demonstrating dynamic limit enforcement rather than just
steady-state containment.

Light damping (0.01) ensures the system gradually loses energy, showing
the combined dissipation from joint damping and limit constraint work.

| Parameter | Value |
|-----------|-------|
| Mass | 1.0 kg |
| Rod length | 0.5 m |
| Cone limit | 45° (0.7854 rad) |
| Damping | 0.01 Nm·s/rad |
| Integrator | RK4 |
| Initial tilt | 60° about X axis |

## Validation

The example runs three automated checks at t=15s (after 1s settling):

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Quat norm** | Unit quaternion preserved | < 1e-10 deviation |
| **Energy** | Monotonically decreasing (damped) | < 1e-3 J increase |
| **Cone limit** | Angle stays within 45° + solver margin | < 0.02 rad violation |

## Run

```
cargo run -p example-ball-joint-cone-limit --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
