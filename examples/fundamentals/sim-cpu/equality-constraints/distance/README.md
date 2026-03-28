# Distance — Rigid Rod (Scalar Constraint)

Two free-floating spheres connected by a visible rigid rod — a `<distance>`
equality constraint that maintains exactly 0.5m of separation between their
centers. The dumbbell falls, lands on the ground, and comes to rest while the
distance stays locked.

## What you see

- **Red sphere** (larger, 1.0 kg) and **blue sphere** (smaller, 0.5 kg) —
  connected by a thin metallic rod
- The dumbbell starts tilted and falls straight down onto the ground
- It lands, tips over naturally, and settles horizontal
- The rod length stays constant throughout — the distance constraint holds

## Physics

The `<distance>` constraint is the simplest equality constraint — it removes
just **1 scalar DOF**: the center-to-center distance between two geoms. Unlike
connect (3 DOFs) or weld (6 DOFs), distance only constrains the magnitude of
separation, not the direction. The bodies are free to orbit each other in any
orientation.

```
constraint: |p1 - p2| = 0.5 m
```

The distance constraint references **geoms**, not bodies — it measures between
geom centers. This is different from connect and weld, which reference body
frames.

| Parameter | Value |
|-----------|-------|
| Sphere A | 1.0 kg, radius 0.08 m, pos (-0.2, 0, 1.3) |
| Sphere B | 0.5 kg, radius 0.06 m, pos (0.2, 0, 1.0) |
| Target distance | 0.5 m |
| Initial orientation | Tilted ~30° from horizontal |
| solref | 0.005, 1.0 |
| Ground | Plane at z=0 |

## Validation

Three automated checks at t=5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Distance maintained** | Center-to-center stays at 0.5 m | < 5 mm deviation |
| **Both move** | Sphere A velocity sustained | > 0.01 m/s |
| **Mass ratio effect** | Lighter sphere moves more | velocity ratio check |

## Run

```
cargo run -p example-equality-distance --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
