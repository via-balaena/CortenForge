# Cable Catenary — Both-Ends-Fixed Passive Sag

A cable composite pinned at both ends, sagging under its own weight into a
catenary shape. The left end is fixed via `initial="none"` (no joint on the
first cable body). The right end is pinned via an equality `connect`
constraint that attaches the cable's tip to the right pylon.

## What you see

- Two gray pylons standing 1.0m apart
- A **gold cable** (15 segments) spanning the gap between them
- The cable starts as a straight horizontal line, then sags into a smooth
  downward curve under gravity
- The endpoints stay fixed at the pylon tops while the middle droops

## Physics

The cable is a rigid-body chain of 15 capsule bodies connected by ball joints.
The left end is rigidly attached to the left pylon (no joint on the first
body). The right end is attached via an equality `connect` constraint — a
3-DOF translational constraint that pins a point on the last cable body to
the right pylon.

Under gravity, the cable sags into a shape approximating the continuous
catenary `y = a cosh(x/a)`. With 15 segments the discrete approximation
is smooth enough to look rope-like.

| Parameter | Value |
|-----------|-------|
| Cable length | 1.0 m |
| Segment count | 15 |
| Span | 1.0 m |
| Joint damping | 0.4 N-m-s/rad |
| Segment density | 800 kg/m^3 |
| Capsule radius | 0.01 m |
| Solver | Newton, 50 iterations |
| Integrator | implicitfast, dt = 2 ms |

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| Cable sags below anchors | sag_depth > 0 | -- |
| Cable settled | max angular velocity < 0.05 | 0.05 rad/s |
| Span preserved | \|span - 1.0\| < 0.05 | 5 cm |
| Symmetric sag | \|left_sag - right_sag\| < 0.02 | 0.02 m |

## Run

```
cargo run -p example-composite-cable-catenary --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
