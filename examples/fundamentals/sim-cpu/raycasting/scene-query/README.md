# Scene Query — LIDAR-Style Ray Fan

A fan of 36 rays sweeps 360° in the physics XZ plane from a central eye
point. `raycast_scene` returns the nearest hit across all geoms for each
ray. The result is a LIDAR-like distance scan of the scene.

**One concept:** `raycast_scene` with body exclusion and geom group
filtering — the scene-level query that rangefinder sensors use internally.

## What you see

- **White sphere** at the origin — the sensor eye
- **36 ray lines** emanating from the eye in a full circle:
  - **Cyan** lines end at a **green dot** (hit point on a shape)
  - **Faint grey** lines extend to max distance (miss — open space)
- **Three objects** arranged around the eye: a blue sphere (+X), a red
  box (+Z / above), and a green capsule (-X, slightly above)
- **Ground plane** below at z=-1

The green hit dots naturally form a polar distance profile — closer dots
mean nearer surfaces.

## Physics

The 36 rays are evenly spaced at 10° intervals in the XZ plane (physics
Z-up). Each ray calls `raycast_scene` which tests all geoms and returns
the closest intersection with `geom_id`.

| Angle | Direction | Expected hit |
|-------|-----------|-------------|
| 0° | +X | Sphere at dist ≈ 2.5 |
| 90° | +Z | Box at dist ≈ 2.2 |
| ~153° | toward (-2, 0, 1) | Capsule |
| 180°–360° | downward component | Ground plane at z=-1 |
| ~10°–80° | upward, no object | Miss (open space) |

Body exclusion is demonstrated in validation: excluding the sphere's body
makes the 0° ray miss entirely, proving the filter works.

## Validation

Four automated checks run at startup:

| Check | Condition |
|-------|-----------|
| Ray 0° hits sphere | dist ≈ 2.5 (sphere at x=3, r=0.5) |
| geom_id correct | ray 0° returns the sphere geom |
| Open-space miss | at least one ray returns None |
| Body-exclude filter | excluding sphere body, ray 0° misses |

## Run

```
cargo run -p example-raycasting-scene-query --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
