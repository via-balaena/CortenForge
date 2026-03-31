# Primitive Geometry Shapes

One arm with three collision primitives: box (base), sphere (link1),
cylinder (link2). Verifies all three appear in the compiled model with
correct sizes after the URDF → MJCF size convention conversion.

## What it tests

URDF and MJCF use different size conventions:

| Primitive | URDF | MJCF | Conversion |
|-----------|------|------|------------|
| Box | full extents `size="x y z"` | half extents `size="x/2 y/2 z/2"` | divide by 2 |
| Cylinder | `radius` + full `length` | `radius` + half length | halve length |
| Sphere | `radius` | `radius` | unchanged |

The example uses:
- Box: `size="0.2 0.4 0.6"` → MJCF `(0.1, 0.2, 0.3)`
- Sphere: `radius="0.15"` → MJCF `0.15`
- Cylinder: `radius="0.08" length="0.3"` → MJCF `(0.08, 0.15)`

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | Three geoms in model | exact |
| 2 | Box half-extents (0.1, 0.2, 0.3) | 0.001 |
| 3 | Sphere radius 0.15 | 0.001 |
| 4 | Cylinder (r=0.08, hl=0.15) | 0.001 |
| 5 | All 3 primitive types present | exact |

## Run

```
cargo run -p example-urdf-geometry --release
```
