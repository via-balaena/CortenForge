# GeomDist + GeomNormal + GeomFromTo Sensors

Geometric proximity sensors using the GJK narrowphase.

## What it demonstrates

- `<distance>` measures signed distance between two geoms (1D)
- `<normal>` measures surface normal at nearest points (3D unit vector)
- `<fromto>` measures nearest surface points on both geoms (6D)
- Uses box-sphere pairing to exercise GJK (sphere-sphere would use the
  analytic fast path, bypassing GJK entirely)
- `cutoff="10"` on all sensors to avoid the cutoff=0 initialization edge case
- Spring-loaded slide joint creates oscillating distance

## Expected visual behavior

A blue box on a spring oscillates horizontally near a fixed red sphere. The box
slides back and forth along the X-axis, with the gap between them changing over
time. Both objects are floating (no ground contact — collisions disabled).

## Expected console output

```
t=  1.0s  dist=+0.XXXX  normal=(+1.000,+0.000,+0.000)
t=  2.0s  dist=+0.XXXX  normal=(+1.000,+0.000,+0.000)
...
```

Distance oscillates with the spring. Normal consistently points along +X
(from box surface toward sphere surface).

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Dist analytical | matches `(sphere_x - r) - (box_x + half)` | < 1e-6 |
| Normal ≈ [1,0,0] | surface normal along separation axis | < 1e-4 |
| Dist range > 0.1m | spring oscillation creates varying distance | range > 0.1 |
| Energy conservation | undamped spring (harness tracker) | < 0.5% drift |

## Run

```bash
cargo run -p example-sensor-geom-distance --release
```
