# Mesh-Cylinder — GJK/EPA Flat-Cap Fix (Branch Highlight)

An upright cylinder on a mesh ground slab. This is the "money shot" for the
GJK/EPA cylinder fix landed on this branch.

**Before this branch:** mesh-cylinder collision used a capsule approximation —
the cylinder's flat caps were treated as rounded hemispheres, giving wrong
contact geometry and unstable resting behavior.

**After this branch:** exact GJK/EPA support functions for cylinders. MULTICCD
generates 8 rim contact points around the flat cap edge, providing full
rotational stability for flat-on-flat contact.

## What you see

- **Polished steel cylinder** (r=0.1, half_length=0.2) standing upright on a
  **brushed metal mesh slab**
- The cylinder drops, settles, and stays perfectly upright — no wobble, no drift
- The flat bottom cap makes full contact with the surface
- The flat top face is visible from the elevated camera angle, proving this is a
  true cylinder (not a capsule with rounded ends)

## Physics

| Parameter | Value |
|-----------|-------|
| Cylinder radius | 0.1 m |
| Cylinder half-length | 0.2 m |
| Ground mesh | 8 vertices, 12 faces (2x2x2 box) |
| Density | 1000 kg/m^3 |
| Integrator | RK4, dt = 1 ms |
| MULTICCD | enabled (8 rim contacts) |
| Narrow-phase | `gjk_epa_shape_pair()` |

## Validation

Five automated checks at t=5s:

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = 0.2 (half_length) | < 3mm |
| Settled | \|vz\| < 0.01 m/s | -- |
| Contact force / weight | 1.0 | 5% |
| MULTICCD contacts | ncon >= 2 | -- |
| Rotational stability | angular velocity < 0.1 rad/s | -- |

## Run

```
cargo run -p example-mesh-collision-cylinder --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
