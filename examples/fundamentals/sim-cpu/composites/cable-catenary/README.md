# Cable Catenary — Both-Ends-Fixed Passive Sag

A cable composite pinned at both ends, sagging under its own weight into a
catenary shape. The left end is attached via a ball joint (rotates freely but
can't translate). The right end is pinned via an equality `connect` constraint
that attaches the cable's tip to the right pylon.

## What you see

- Two gray pylons standing 1.0m apart
- A **gold cable** (15 segments) spanning the gap between them
- The cable starts bowed slightly upward, then swings down under gravity
  into a smooth catenary curve
- The endpoints stay fixed at the pylon tops while the middle droops

## Physics

The cable is a rigid-body chain of 15 capsule bodies connected by ball joints.
The left end is attached to the left pylon via a ball joint (can rotate to
match the catenary tangent). The right end is attached via an equality
`connect` constraint — a 3-DOF translational constraint that pins a point
on the last cable body to the right pylon.

The cable starts with a half-sine upward bulge (`curve="l 0 s"`) to give the
chain slack — rigid links summing to exactly the span can't sag, so the path
length (~1.06m) must exceed the 1.0m span. Under gravity, the cable swings
down into a shape approximating the continuous catenary `y = a cosh(x/a)`.
With 15 segments the discrete approximation is smooth enough to look rope-like.

| Parameter | Value |
|-----------|-------|
| Cable path length | ~1.06 m |
| Segment count | 15 |
| Pylon span | 1.0 m |
| Joint damping | 0.05 N-m-s/rad |
| Segment density | 800 kg/m^3 |
| Capsule radius | 0.01 m |
| Solver | Newton, 50 iterations |
| Integrator | implicitfast, dt = 2 ms |

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| Cable sags below anchors | sag_depth > 0 | -- |
| Cable settled | max angular velocity < 0.01 | 0.01 rad/s |
| Span preserved | \|span - 1.0\| < 0.07 | 7 cm |
| Symmetric sag | \|left_sag - right_sag\| < 0.01 | 0.01 m |

## Run

```
cargo run -p example-composite-cable-catenary --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
