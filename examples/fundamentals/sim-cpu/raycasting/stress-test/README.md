# Stress Test — Headless Raycast Validation

Exhaustive headless validation of ray intersection against analytical
solutions. No window, no Bevy — pure sim-core assertions for all shape
types, surface normals, scene-level queries, and edge cases.

## What it validates

| # | Check | Invariant |
|---|-------|-----------|
| 1 | Sphere hit distance | dist = center_z - radius = 4.0 |
| 2 | Sphere miss | ray in +X at sphere at z=5 returns None |
| 3 | Plane hit distance | dist = plane_z / (dir . normal) = 3.0 |
| 4 | Plane parallel miss | ray parallel to plane returns None |
| 5 | Box slab intersection | dist = 4.0, normal = (0, 0, -1) |
| 6 | Capsule bottom cap | dist = 5 - half_length - radius = 3.7 |
| 7 | Cylinder flat cap | dist = center_z - half_length = 4.0 |
| 8 | Ellipsoid scaled sphere | dist = center_z - rz = 3.0 |
| 9 | Heightfield ray march | flat field at z=0, dist = 3.0 |
| 10 | Convex mesh hull test | center hit, bounding-sphere-only miss |
| 11 | Normal unit-length | all hits: ‖n‖ in [0.999, 1.001] |
| 12 | Normal faces ray | all hits: dot(n, dir) < 0 |
| 13 | Normal perpendicular | sphere, plane, box: matches analytical |
| 14 | Scene nearest hit | two spheres, returns closer (geom_id=0) |
| 15 | Body exclude | exclude nearer body, hits farther sphere |
| 16 | Geom group filter | group mask skips group-1 sphere |
| 17 | Scene miss | ray away from all geoms returns None |
| 18 | Inside sphere | origin at center, returns exit at dist=radius |
| 19 | Max distance clamp | hit beyond cutoff returns None |
| 20 | Zero max distance | max_distance=0 always returns None |

## Expected output

```
TOTAL: 20/20 checks passed
ALL PASS
```

## Run

```
cargo run -p example-raycasting-stress-test --release
```
