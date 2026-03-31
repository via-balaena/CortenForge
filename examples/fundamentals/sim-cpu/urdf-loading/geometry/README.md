# Primitive Geometry Shapes

A pendulum arm with all three URDF collision primitives on one body:
box (top), cylinder (middle), sphere (bottom). They swing together
as a single rigid unit.

## What you see

Three shapes stacked flush — box at pivot, cylinder rod, sphere tip —
swinging as one pendulum. The HUD shows the geom types present.

## What it tests

URDF and MJCF use different size conventions:

| Primitive | URDF | MJCF | Conversion |
|-----------|------|------|------------|
| Box | full extents `size="x y z"` | half extents `size="x/2 y/2 z/2"` | divide by 2 |
| Cylinder | `radius` + full `length` | `radius` + half length | halve length |
| Sphere | `radius` | `radius` | unchanged |

## Validation

| Check | Source |
|-------|--------|
| All 3 primitives present | `print_report` |
| Sphere radius = 0.15 | `print_report` |
| Cylinder (r=0.08, hl=0.15) | `print_report` |
| Box half-extents (0.1, 0.2, 0.05) | `print_report` |

## Run

```
cargo run -p example-urdf-geometry --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
