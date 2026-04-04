# Cable Loaded — Midpoint Force on Fixed Cable

A cable composite pinned at both ends with a constant downward force applied
at the midpoint. Compared to the passive catenary, the midpoint dips
significantly lower, creating a V-like shape.

## What you see

- Two gray pylons standing 1.0m apart
- A **cyan cable** (15 segments) spanning the gap between them
- A red sphere at the midpoint indicating the applied load
- The cable sags into a V-shape under the midpoint force, much deeper
  than a passive catenary

## Physics

Same cable geometry as cable-catenary (15 segments, ball joint at left end,
connect constraint at right end, half-sine initial bulge for slack). The
difference: a constant 5N downward force is applied to the midpoint body
via `xfrc_applied`. This creates a concentrated load that pulls the midpoint
well below the natural catenary curve.

| Parameter | Value |
|-----------|-------|
| Cable path length | ~1.06 m |
| Segment count | 15 |
| Pylon span | 1.0 m |
| Joint damping | 0.05 N-m-s/rad |
| Segment density | 800 kg/m^3 |
| Capsule radius | 0.01 m |
| Midpoint load | 5.0 N downward |
| Solver | Newton, 50 iterations |
| Integrator | implicitfast, dt = 2 ms |

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| Cable sags below anchors | sag_depth > 0 | -- |
| Midpoint is lowest point | mid_z < all other body z | -- |
| Cable settled | max angular velocity < 0.01 | 0.01 rad/s |
| Sag exceeds passive catenary | sag > 0.05 | 0.05 m |

## Run

```
cargo run -p example-composite-cable-loaded --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
