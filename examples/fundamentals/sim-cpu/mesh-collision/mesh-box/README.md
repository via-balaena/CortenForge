# Mesh-Box — GJK/EPA with MULTICCD

A box dropped onto a mesh ground slab. Demonstrates mesh-box collision via
GJK/EPA with MULTICCD enabled for face-face stability.

## What you see

- **Polished steel box** (0.15 x 0.1 x 0.05 half-sizes) sitting flat on a
  **brushed metal mesh slab**
- The box drops from z=0.3, lands flat, and settles at z = half_z = 0.05
- No wobble or drift — MULTICCD generates multiple contact points across the
  flat face for rotational stability

## Physics

Mesh-box collision was refactored on this branch to use GJK/EPA (the old
`triangle_box_contact()` path was removed). Without MULTICCD, GJK/EPA produces
a single contact point for flat-on-flat contact, which is insufficient to
prevent rotation. MULTICCD generates additional contact points across the
contact patch, stabilizing the box.

| Parameter | Value |
|-----------|-------|
| Box half-sizes | 0.15 x 0.1 x 0.05 m |
| Ground mesh | 8 vertices, 12 faces (2x2x2 box) |
| Density | 1000 kg/m^3 |
| Integrator | RK4, dt = 1 ms |
| MULTICCD | enabled |
| Narrow-phase | `gjk_epa_shape_pair()` |

## Validation

Three automated checks at t=5s:

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = 0.05 (half_z) | < 3mm |
| Settled | \|vz\| < 0.01 m/s | -- |
| Contact force / weight | 1.0 | 5% |

## Run

```
cargo run -p example-mesh-collision-box --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
