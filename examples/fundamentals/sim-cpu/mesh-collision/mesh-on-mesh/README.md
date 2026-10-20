# Mesh-on-Mesh — Two Convex Hulls

A mesh wedge (triangular prism) dropped onto a mesh platform (box slab). Both
objects are `type="mesh"` — no primitives involved. The pair collides through
`mjc_convex()`, MuJoCo's native convex collision, each mesh as the convex hull
of its vertices.

## What you see

- **Copper wedge** (triangular prism) sitting on a **brushed metal mesh platform**
- Both the wedge and platform are defined with inline vertex/face data
- The platform sits on a grey ground plane
- The wedge drops from above, settles on its flat base, and stays stable
- MULTICCD clips the touching faces for up to four contacts

## Physics

At load time each mesh's convex hull is computed (`compute_convex_hull()`
needs at least 4 non-coplanar vertices; a mesh without a hull is searched
vertex by vertex). At collision time GJK tests the two hulls for overlap and
EPA finds its depth; under MULTICCD, with no margin on either mesh, one pass
of MuJoCo's multi-contact clips the touching faces and keeps up to four
contacts.

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
| Narrow-phase | `mjc_convex()` |

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
