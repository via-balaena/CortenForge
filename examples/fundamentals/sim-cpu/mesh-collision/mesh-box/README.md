# Mesh-Box — Face Clipping with MULTICCD

A box dropped onto a mesh ground slab. The pair collides through
`mjc_convex()`, MuJoCo's native convex collision, against the convex hull of
the slab's vertices, with MULTICCD enabled.

## What you see

- **Polished steel box** (0.15 x 0.1 x 0.05 half-sizes) sitting flat on a
  **brushed metal mesh slab**
- The box drops from z=0.3, lands flat, and settles at z = half_z = 0.05
- No wobble or drift — MULTICCD gives up to four contacts across the flat
  face

## Physics

Without MULTICCD the pair gets one contact. With it, and no margin on either
geom, one pass of MuJoCo's multi-contact clips the box's touching face against
the slab's and keeps up to four contacts.

| Parameter | Value |
|-----------|-------|
| Box half-sizes | 0.15 x 0.1 x 0.05 m |
| Ground mesh | 8 vertices, 12 faces (2x2x2 box) |
| Density | 1000 kg/m^3 |
| Integrator | RK4, dt = 1 ms |
| MULTICCD | enabled |
| Narrow-phase | `mjc_convex()` |

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
