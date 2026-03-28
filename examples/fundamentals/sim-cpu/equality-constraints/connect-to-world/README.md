# Connect to World — Ball-and-Socket Pendulum

A single free body locked to the world origin by a `<connect>` equality
constraint. The body swings like a pendulum — position is pinned, but rotation
is completely free in all three axes. This is the simplest possible connect
constraint.

## What you see

- **Steel rod** with a **red tip sphere** — a capsule-and-mass hanging from a
  fixed point at the world origin
- The body swings freely in 3D, orbiting and tumbling under gravity — not
  restricted to a plane like a hinge joint would be
- The pivot point stays locked at the origin (the constraint holds it there)

## Physics

The `<connect>` constraint removes **3 translational DOFs**, locking a point on
body1 to a point on body2 (or the world). Rotation remains free — this is
equivalent to a ball-and-socket joint, but built from a free joint + equality
constraint instead of an explicit ball joint.

The constraint uses the **penalty method**: a stiff spring (controlled by
`solref`) pulls the body back when it drifts from the anchor. This means
there's always a tiny residual error — sub-millimeter with `solref="0.005 1.0"`.

| Parameter | Value |
|-----------|-------|
| Body mass | 1.5 kg (rod 1.0 + ball 0.5) |
| Rod length | 0.5 m |
| Anchor | World origin (0, 0, 0) |
| solref | 0.005, 1.0 (stiff) |
| Solver | Newton, 50 iterations |
| Timestep | 1 ms |

## Validation

Three automated checks at t=5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Pivot anchored** | Body origin stays at (0,0,0) | < 5 mm |
| **Rotation free** | Angular velocity sustained | > 0.1 rad/s |
| **Energy bounded** | No energy injection from constraint | < 5% growth |

## Run

```
cargo run -p example-equality-connect-to-world --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
