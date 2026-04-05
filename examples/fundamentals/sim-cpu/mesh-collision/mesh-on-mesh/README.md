# Mesh-on-Mesh — Hull-Hull GJK/EPA

A mesh wedge (triangular prism) dropped onto a mesh platform (box slab). Both
objects are `type="mesh"` — no primitives involved. Demonstrates hull-hull
GJK/EPA collision where both colliders are convex hulls computed at load time.

## What you see

- **Copper wedge** (triangular prism) sitting on a **brushed metal mesh platform**
- Both the wedge and platform are defined with inline vertex/face data
- The platform sits on a grey ground plane
- The wedge drops from above, settles on its flat base, and stays stable
- MULTICCD provides multi-point contact for face-face stability

## Physics

Both meshes must have >= 4 non-coplanar vertices for `compute_convex_hull()` to
succeed. The platform (8 vertices) and wedge (6 vertices) both satisfy this.
At load time, convex hulls are built from the vertex data. At collision time,
GJK finds the closest features and EPA refines the penetration depth.

The wedge is centered at its volumetric centroid so body position = center of
mass. The centroid-to-base distance is h/3 = 0.25/3 = 0.0833 m.

| Parameter | Value |
|-----------|-------|
| Platform | 8 vertices, 12 faces (1x1x0.1 box mesh) |
| Wedge | 6 vertices, 8 faces (triangular prism) |
| Wedge cross-section | isosceles triangle, base=0.3, height=0.25 |
| Platform top | z = 0.1 |
| Expected rest height | z = 0.1 + 0.0833 = 0.183 |
| Density | 1000 kg/m^3 |
| Integrator | RK4, dt = 1 ms |
| MULTICCD | enabled |
| Narrow-phase | `gjk_epa_shape_pair()` (hull-hull) |

## Validation

Four automated checks at t=5s:

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Settled | \|vz\| < 0.01 m/s | -- |
| Above platform | z > 0.1 | -- |
| No interpenetration | z > 0.178 | 5mm |
| Contact force / weight | 1.0 | 10% |

## Run

```
cargo run -p example-mesh-collision-mesh --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
