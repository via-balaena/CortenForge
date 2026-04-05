# Mesh-Sphere — BVH-Accelerated Triangle Query

A sphere dropped onto a mesh ground slab. Demonstrates `mesh_sphere_contact()`
with BVH-accelerated triangle lookup — the BVH finds which triangles of the mesh
are near the sphere, then each candidate triangle is tested for contact.

## What you see

- **Copper sphere** (r=0.1) sitting on a **brushed metal mesh slab**
- The sphere drops from z=0.3, bounces, then settles at z = radius = 0.1
- The ground looks flat but is a 2x2x2 box mesh (8 vertices, 12 triangles)

## Physics

The mesh ground slab is a convex box mesh positioned at z=-1 so its top face
sits at z=0. The sphere collides with the top-face triangles via the BVH
acceleration structure built at model load time.

| Parameter | Value |
|-----------|-------|
| Sphere radius | 0.1 m |
| Ground mesh | 8 vertices, 12 faces (2x2x2 box) |
| Density | 1000 kg/m^3 |
| Integrator | RK4, dt = 1 ms |
| Narrow-phase | `mesh_sphere_contact()` via BVH |

## Validation

Three automated checks at t=5s:

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = 0.1 (radius) | < 3mm |
| Settled | \|vz\| < 0.01 m/s | -- |
| Contact force / weight | 1.0 | 5% |

## Run

```
cargo run -p example-mesh-collision-sphere --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
