# Cylinder Wrap — Tendon Wrapping Around a Cylinder

A spatial tendon wraps around a cylinder when the straight-line path between
its sites would penetrate the cylinder surface. Unlike sphere wrapping
(great-circle arc), cylinder wrapping follows a geodesic helix on the
cylinder surface.

## Concept

When a spatial tendon wraps around a **cylinder**, the path follows a helical
geodesic — the shortest path on the cylinder surface between the two tangent
points. The helix has a constant radius (the cylinder radius) and interpolates
linearly in the cylinder's axial direction. Cylinder wrapping models tendon
sheaths, muscle routing around bones, and cable guides.

## What you see

- Two arms on opposite sides of a translucent green cylinder, connected by
  a spatial tendon that wraps around the cylinder surface.
- A motor oscillates one arm so the helical wrap path tightens and loosens.
- The tendon path hugs the cylinder as a smooth helix when wrapping is active.
- Line color shows stretch: green (relaxed) → yellow → red (max stretch).
  Self-calibrates over the first oscillation cycle.
- The HUD shows tendon length, wrap status, and joint angles.

## Run

```
cargo run -p example-tendon-cylinder-wrap --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
