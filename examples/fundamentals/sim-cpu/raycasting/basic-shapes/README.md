# Basic Shapes — Shape-Level Ray Intersection

Six static primitives arranged in a row. A downward ray from z=3 strikes
each one. The hit point and surface normal are drawn as gizmos, and the
HUD reports distance and normal for every shape.

**One concept:** `raycast_scene` returns the nearest `RaycastHit` with
distance, world-space hit point, and outward surface normal.

## What you see

- **Six colored shapes** along X: sphere, box, capsule, cylinder,
  ellipsoid, and the ground plane
- **Faint cyan lines** — the downward rays from z=3
- **Green dots** — hit points on each shape's surface
- **Yellow arrows** — surface normals pointing away from the shape

The scene is static (zero gravity, no joints). The gizmos are recomputed
every frame but the results never change — the point is to see where rays
land and which direction the surface faces.

## Physics

Each ray is cast via `raycast_scene`, which iterates all geoms and returns
the closest intersection. Under the hood, `cf_geometry::ray_cast` handles
each shape type analytically:

| Shape | Intersection method | Expected distance |
|-------|-------------------|-------------------|
| Sphere r=0.5 | Quadratic ray-sphere | 2.000 |
| Box 0.5^3 | Slab test (per-axis min/max) | 2.000 |
| Capsule r=0.3 hl=0.5 | Sphere-swept-line (cap hit) | 1.400 |
| Cylinder r=0.3 hl=0.5 | Flat cap test | 2.000 |
| Ellipsoid (0.6, 0.4, 0.3) | Scaled unit-sphere | 2.400 |
| Ground plane | Ray-plane formula | 3.000 |

All normals point upward (+Z) since every ray strikes the top surface of
each shape.

## Validation

Five automated checks run at startup and print to the console:

| Check | Condition |
|-------|-----------|
| All 6 rays hit | no misses |
| Distance accuracy | max error < 1e-3 |
| Hit point on surface | z-coordinate matches expected |
| Normal unit-length | ‖n‖ - 1 < 1e-3 |
| Normal faces ray | dot(n, ray_dir) < 0 |

## Run

```
cargo run -p example-raycasting-basic-shapes --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
