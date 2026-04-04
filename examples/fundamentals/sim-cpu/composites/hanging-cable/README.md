# Hanging Cable — Pendulum Chain Under Gravity

Three cable composites pinned at one end, hanging freely under gravity. Each
cable has a different segment count (5, 10, 20) to show how resolution affects
the equilibrium shape. More segments produce a smoother curve that converges
toward the continuous catenary solution.

## What you see

- Three capsule-segment cables hang from a horizontal bar at z=1.2m
- **Red** (5 segments) looks angular and stiff
- **Green** (10 segments) shows a smoother hanging curve
- **Blue** (20 segments) looks rope-like and closely approximates a catenary
- All three settle to approximately the same tip height, confirming convergence
- A gray ground plane provides spatial reference

## Physics

Cable composites are rigid-body chains: each segment is a capsule body
connected to its neighbor by a ball joint (3 rotational DOF). The chain is
pinned at the anchor via `initial="none"` (no joint on the first body).

Under gravity, each cable swings from its initial horizontal position into a
natural hanging curve. Joint damping (0.4 N-m-s/rad) dissipates energy so
the cables settle within about 10 seconds.

The equilibrium shape approaches the continuous catenary `y = a cosh(x/a)`
as segment count increases. With finite segments, the shape is piecewise
linear — visible as kinks in the 5-segment cable.

| Parameter | Value |
|-----------|-------|
| Cable length | 0.8 m |
| Segment counts | 5 / 10 / 20 |
| Joint damping | 0.4 N-m-s/rad |
| Segment density | 500 kg/m^3 |
| Capsule radius | 0.012 m |
| Integrator | implicitfast, dt = 2 ms |

## Validation

Five automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| 5-seg tip below anchor | tip_z < 1.2 | -- |
| 10-seg tip below anchor | tip_z < 1.2 | -- |
| 20-seg tip below anchor | tip_z < 1.2 | -- |
| Convergence: 10 vs 20 | err_10v20 < err_5v20 | -- |
| All cables settled | max angular velocity < 0.1 | 0.1 rad/s |

## Run

```
cargo run -p example-composite-hanging-cable --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
