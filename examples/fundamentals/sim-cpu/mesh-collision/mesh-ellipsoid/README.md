# Mesh-Ellipsoid — GJK/EPA Exact Support (Branch Highlight)

An oblate ellipsoid (disc-shaped, like a hockey puck) sitting on a mesh ground
slab. This example exposes the bug that GJK/EPA exact support functions fixed.

**Before this branch:** mesh-ellipsoid collision approximated the ellipsoid as a
sphere using the maximum radius (0.3). The disc would rest at z=0.3 — a 6x
error for an oblate shape with rz=0.05.

**After this branch:** exact GJK/EPA support functions use the true ellipsoid
surface. The disc rests at the correct z = rz = 0.05.

## What you see

- **Copper oblate disc** (radii 0.3 x 0.3 x 0.05) sitting flat on a **brushed
  metal mesh slab**
- The disc shape is unmistakably flat — nothing like a sphere
- It drops from z=0.15, settles, and rests just above the surface at z=0.05
- The top-down camera angle shows the full disc profile

## Physics

| Parameter | Value |
|-----------|-------|
| Ellipsoid radii | rx=0.3, ry=0.3, rz=0.05 |
| Aspect ratio | 6:1 (oblate) |
| Ground mesh | 8 vertices, 12 faces (2x2x2 box) |
| Density | 1000 kg/m^3 |
| Integrator | RK4, dt = 1 ms |
| Narrow-phase | `gjk_epa_shape_pair()` |

**The key number:** rest height z=0.05, not z=0.3. If validation reports
z > 0.1, the sphere approximation bug has regressed.

## Validation

Four automated checks at t=5s:

| Check | Expected | Tolerance |
|-------|----------|-----------|
| Rest height | z = 0.05 (rz) | < 3mm |
| Not sphere approx | z < 0.1 | -- |
| Settled | \|vz\| < 0.01 m/s | -- |
| Contact force / weight | 1.0 | 5% |

## Run

```
cargo run -p example-mesh-collision-ellipsoid --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
