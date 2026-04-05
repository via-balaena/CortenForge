# Mesh-Capsule — BVH Barrel Contact

A capsule lying on its side on a mesh ground slab. Demonstrates
`mesh_capsule_contact()` with BVH-accelerated triangle lookup. The curved
barrel surface rests against the mesh triangles — visually distinct from the
upright cylinder in mesh-cylinder.

## What you see

- **Copper capsule** (r=0.1, half_length=0.2) lying sideways on a **brushed
  metal mesh slab**
- The capsule axis lies along Y (rotated 90 degrees around X)
- It drops from z=0.2, settles, and rests with its barrel on the surface at
  z = radius = 0.1

## Physics

The capsule is rotated via `quat="0.707 0.707 0 0"` so its axis lies along Y.
In this orientation, the lowest point of the barrel is at z = -radius in the
body frame, so the rest height equals the capsule radius.

The BVH finds candidate triangles near the capsule's swept volume, then each
triangle is tested against the capsule's line segment + radius geometry.

| Parameter | Value |
|-----------|-------|
| Capsule radius | 0.1 m |
| Capsule half-length | 0.2 m |
| Total length | 0.6 m (2 x (half_length + radius)) |
| Ground mesh | 8 vertices, 12 faces (2x2x2 box) |
| Density | 1000 kg/m^3 |
| Integrator | RK4, dt = 1 ms |
| Narrow-phase | `mesh_capsule_contact()` via BVH |

## Validation

Three automated checks at t=5s:

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = 0.1 (radius) | < 5mm |
| Settled | \|vz\| < 0.01 m/s | -- |
| Contact force / weight | 1.0 | 5% |

## Run

```
cargo run -p example-mesh-collision-capsule --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
