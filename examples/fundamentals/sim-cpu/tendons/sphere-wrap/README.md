# Sphere Wrap — Tendon Wrapping Around a Sphere

A spatial tendon wraps around a sphere when the straight-line path between
its sites would penetrate the sphere surface. The tendon takes the shortest
path that stays outside the sphere, producing a great-circle arc.

## Concept

When a spatial tendon's path encounters a **wrapping geom** (here a sphere),
the engine computes tangent points on the sphere surface and routes the tendon
along the geodesic (great-circle arc). This changes the tendon length and,
critically, the Jacobian — creating a configuration-dependent moment arm.
Sphere wrapping models bone condyles, pulley wheels, and other rounded
anatomical or mechanical features.

## What you see

- A hinge joint rotates an arm around a translucent yellow sphere.
- The tendon origin is on the arm tip; the insertion is fixed on the opposite
  side of the sphere.
- When the straight-line path would cross through the sphere, the tendon
  wraps — you see the path hug the sphere surface as an arc.
- When the path is clear, the tendon goes straight (no wrapping).
- Line color shows tendon length: green (shortest observed) → yellow →
  red (longest observed). Self-calibrates over the first oscillation cycle.
- The HUD shows tendon length, wrap point count, and whether wrapping is
  active.

## Run

```
cargo run -p example-tendon-sphere-wrap --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
